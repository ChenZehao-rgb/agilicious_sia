#!/usr/bin/env python3
"""Deterministic health freshness checks with real messages and MSP decoding."""
import struct
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from agi_ros2.msg import FusedState, MspEvent, OutputStatus
from builtin_interfaces.msg import Time
from msp_evidence import EvidenceNode, output_freshness, stamp
from test_shadow_support import ready_decoder


class CapturedPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class HealthFreshnessTest(unittest.TestCase):
    def health(self, sample_age, publication_age=0.0, output_ros_time=10.0, output_steady_time=100.0,
               decoder_failed=False, output_mapping=True):
        fused = FusedState()
        fused.header.stamp = Time(sec=10, nanosec=0)
        fused.clock_id = 'test-boot'
        fused.initialized = fused.imu_ready = fused.estimator_ready = True
        fused.navigation_ready = fused.navigation_accuracy_ok = fused.accuracy_known = True
        fused.heading_valid = fused.clock_aligned = True
        fused.published_steady_time = 100.0 - publication_age
        output = OutputStatus()
        output.header.stamp = stamp(output_ros_time)
        output.clock_id = 'test-boot'
        output.steady_time = output_steady_time
        output.transport_healthy = True
        output.thrust_mapping_ready = output_mapping
        node = SimpleNamespace(
            decoder=ready_decoder(10.0), battery_timeout=1.5, clock_id='test-boot',
            minimum=[-1.0]*3, maximum=[1.0]*3, fused=fused, output=output,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round((10.0+sample_age)*1e9))))
        node.decoder.failed = decoder_failed
        for name in ['decoded_pub', 'authority_pub', 'health_pub', 'status_pub']:
            setattr(node, name, CapturedPublisher())
        with patch('msp_evidence.time.monotonic', return_value=100.0):
            EvidenceNode.publish(node)
        return node.health_pub.messages[-1]

    def test_24ms_state_retains_diagnostic_readiness(self):
        health = self.health(.024, .024)
        self.assertTrue(health.fc_imu_ready)
        self.assertTrue(health.imu_ready and health.estimator_ready and health.navigation_ready)
        self.assertTrue(health.geofence_ok)

    def test_stale_cached_state_does_not_clear_fc_readiness(self):
        for sample_age, publication_age in [(.026, 0.0), (.001, .026)]:
            with self.subTest(sample_age=sample_age, publication_age=publication_age):
                health = self.health(sample_age, publication_age)
                self.assertTrue(health.fc_imu_ready)
                self.assertFalse(health.imu_ready or health.estimator_ready or health.navigation_ready or health.geofence_ok)
                self.assertIn('Fused state unavailable/stale', health.reason)

    def test_future_state_remains_rejected(self):
        self.assertFalse(self.health(-.001).navigation_ready)

    def test_stale_output_cache_does_not_revoke_mapping_or_transport(self):
        for ros_time, steady_time in ((9.9, 100.), (10., 99.9)):
            with self.subTest(ros_time=ros_time, steady_time=steady_time):
                health = self.health(.001, output_ros_time=ros_time, output_steady_time=steady_time)
                self.assertTrue(health.thrust_mapping_ready and health.transport_healthy)
                self.assertTrue(health.fc_imu_ready and health.imu_ready and health.estimator_ready and health.navigation_ready)

    def test_actual_transport_failure_still_revokes_health(self):
        health = self.health(.001, decoder_failed=True)
        self.assertFalse(health.transport_healthy)

    def test_explicit_mapping_failure_is_not_ignored(self):
        health = self.health(.001, output_steady_time=99.9, output_mapping=False)
        self.assertFalse(health.thrust_mapping_ready)

    def test_missing_output_keeps_mapping_unavailable(self):
        health = self.health(.001, output_steady_time=float('nan'))
        self.assertFalse(health.thrust_mapping_ready)

    def test_future_output_does_not_supply_mapping(self):
        health = self.health(.001, output_steady_time=100.001)
        self.assertFalse(health.thrust_mapping_ready)


class OutputFreshnessTest(unittest.TestCase):
    def output(self):
        message = OutputStatus()
        message.clock_id = 'test-boot'
        message.header.stamp = Time(sec=10)
        message.steady_time = 100.0
        return message

    def test_missing_output_has_explicit_diagnostic(self):
        result = output_freshness(None, 'test-boot', 10., 100.)
        self.assertFalse(result['available'] or result['fresh'])
        self.assertIsNone(result['ros_age_seconds'])

    def test_fresh_and_stale_cache_are_distinguishable(self):
        message = self.output()
        fresh_result = output_freshness(message, 'test-boot', 10.02, 100.02, 100.01)
        self.assertTrue(fresh_result['fresh'])
        self.assertAlmostEqual(fresh_result['cache_age_seconds'], .01)
        # A just-received old DDS snapshot must remain stale.
        stale_result = output_freshness(message, 'test-boot', 10.06, 100.06, 100.06)
        self.assertFalse(stale_result['fresh'])
        self.assertAlmostEqual(stale_result['steady_age_seconds'], .06)
        self.assertEqual(stale_result['cache_age_seconds'], 0.)

    def test_each_clock_deadline_remains_required(self):
        for ros_now, steady_now in ((10.051, 100.), (10., 100.051), (9.999, 100.), (10., 99.999)):
            with self.subTest(ros_now=ros_now, steady_now=steady_now):
                self.assertFalse(output_freshness(self.output(), 'test-boot', ros_now, steady_now)['fresh'])

    def test_wrong_clock_and_nonfinite_time_remain_rejected(self):
        message = self.output()
        result = output_freshness(message, 'other-boot', 10., 100.)
        self.assertFalse(result['clock_matches'] or result['fresh'])
        message.steady_time = float('nan')
        result = output_freshness(message, 'test-boot', 10., 100.)
        self.assertFalse(result['fresh'])
        self.assertIsNone(result['steady_age_seconds'])



class ModeEventTest(unittest.TestCase):
    def node_and_event(self, event='rx', request=95., request_steady=995.):
        node = SimpleNamespace(decoder=ready_decoder(), clock_id='test-boot', publish=lambda: None,
                               get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
        message = MspEvent(code=105, event=event, session_id='test-boot:1',
                           steady_time=999.9, request_steady_time=request_steady,
                           request_stamp=Time(sec=int(request)),
                           payload=list(struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1000, 1000)))
        message.header.stamp = Time(sec=99, nanosec=900_000_000)
        return node, message

    def test_mode_request_age_is_not_a_transport_failure(self):
        node, message = self.node_and_event()
        with patch('msp_evidence.time.monotonic', return_value=1000.):
            EvidenceNode.on_event(node, message)
        self.assertFalse(node.decoder.failed)
        self.assertEqual(node.decoder.frames[105][1], 95.)

    def test_unmatched_reply_keeps_request_time_unknown(self):
        node, message = self.node_and_event('mode_rx_unmatched', 0., 0.)
        with patch('msp_evidence.time.monotonic', return_value=1000.):
            EvidenceNode.on_event(node, message)
        self.assertFalse(node.decoder.failed)
        self.assertEqual(node.decoder.frames[105][1], 0.)

    def test_future_or_different_clock_evidence_still_fails(self):
        for request, steady in ((101., 1001.), (95., 994.)):
            node, message = self.node_and_event(request=request, request_steady=steady)
            with patch('msp_evidence.time.monotonic', return_value=1000.):
                EvidenceNode.on_event(node, message)
            self.assertTrue(node.decoder.failed)


if __name__ == '__main__':
    unittest.main()
