#!/usr/bin/env python3
"""Deterministic health freshness checks with real messages and MSP decoding."""
import json
import struct
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from agi_ros2.msg import FusedState, MspEvent, OutputStatus
from builtin_interfaces.msg import Time
from msp_evidence import ClockEpoch, EvidenceNode, output_freshness, stamp
from test_shadow_support import ready_decoder, config_frames, status_frame


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
            decoder=ready_decoder(10.0, request_steady=100.), battery_timeout=1.5, clock_id='test-boot',
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
    def test_authority_heartbeat_preserves_original_request_clock(self):
        wall_now = [10.04]
        node = SimpleNamespace(
            decoder=ready_decoder(10., request_steady=100.), battery_timeout=1.5,
            clock_id='test-boot', minimum=[0.]*3, maximum=[0.]*3, fused=None, output=None,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(wall_now[0]*1e9))))
        for name in ('decoded_pub', 'authority_pub', 'health_pub', 'status_pub'):
            setattr(node, name, CapturedPublisher())
        for ros_now, steady_now in ((10.04, 100.04), (10.08, 100.08), (10.08, 100.11)):
            wall_now[0] = ros_now
            with patch('msp_evidence.time.monotonic', return_value=steady_now):
                EvidenceNode.publish(node)
            authority = node.authority_pub.messages[-1]
            self.assertEqual(authority.clock_id, 'test-boot')
            self.assertEqual(authority.rc_request_steady_time, 100.)
            self.assertEqual(authority.status_request_steady_time, 100.)
            self.assertEqual(authority.header.stamp, stamp(10.))
        self.assertTrue(node.authority_pub.messages[1].receiver_valid)
        self.assertFalse(node.authority_pub.messages[2].receiver_valid)

    def node_and_event(self, event='rx', request=95., request_steady=995.):
        node = SimpleNamespace(decoder=ready_decoder(), clock_id='test-boot', publish=lambda: None,
                               get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
        message = MspEvent(code=105, event=event, session_id='test-boot:1',
                           steady_time=999.9, request_steady_time=request_steady,
                           request_stamp=Time(sec=int(request)),
                           payload=list(struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1000, 1000)))
        message.header.stamp = Time(sec=99, nanosec=900_000_000)
        return node, message

    def test_old_mode_request_revokes_authority_without_transport_failure(self):
        node, message = self.node_and_event()
        with patch('msp_evidence.time.monotonic', return_value=1000.):
            EvidenceNode.on_event(node, message)
        self.assertFalse(node.decoder.failed)
        self.assertEqual(node.decoder.frames[105][1], 95.)
        self.assertFalse(node.decoder.snapshot(100.)['receiver_valid'])

    def test_unmatched_reply_keeps_request_time_unknown(self):
        node, message = self.node_and_event('mode_rx_unmatched', 0., 0.)
        with patch('msp_evidence.time.monotonic', return_value=1000.):
            EvidenceNode.on_event(node, message)
        self.assertFalse(node.decoder.failed)
        self.assertNotIn(105, node.decoder.frames)
        self.assertFalse(node.decoder.snapshot(100.)['receiver_valid'])

    def test_future_or_different_clock_evidence_still_fails(self):
        for request, steady in ((101., 1001.), (95., 994.)):
            node, message = self.node_and_event(request=request, request_steady=steady)
            with patch('msp_evidence.time.monotonic', return_value=1000.):
                EvidenceNode.on_event(node, message)
            self.assertTrue(node.decoder.failed)


class ClockRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.ros = 10.
        self.steady = 100.
        self.node = SimpleNamespace(
            decoder=ready_decoder(self.ros, request_steady=self.steady),
            clock_epoch=ClockEpoch(self.ros, self.steady),
            battery_timeout=1.5, clock_id='test-boot', minimum=[0.]*3, maximum=[0.]*3,
            fused=None, output=None,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(self.ros*1e9))))
        self.node.decoder.session = 'test-boot:1'
        for name in ('decoded_pub', 'authority_pub', 'health_pub', 'status_pub'):
            setattr(self.node, name, CapturedPublisher())
        self.node.publish = lambda: EvidenceNode.publish(self.node)
        self.monotonic = patch('msp_evidence.time.monotonic', side_effect=lambda: self.steady)
        self.monotonic.start()
        self.addCleanup(self.monotonic.stop)
        self.node.publish()
        self.assertTrue(self.node.authority_pub.messages[-1].receiver_valid)

    def event(self, code, payload, name='', request=None, request_steady=None, event='rx'):
        message = MspEvent(code=code, payload=list(payload), request_name=name,
                           event=event, session_id='test-boot:1', steady_time=self.steady,
                           request_stamp=stamp(self.ros if request is None else request),
                           request_steady_time=self.steady if request_steady is None else request_steady)
        message.header.stamp = stamp(self.ros)
        EvidenceNode.on_event(self.node, message)

    def modes(self, armed=False, auto=False, status_armed=None, status_auto=None):
        self.ros += .001
        self.steady += .001
        self.event(105, struct.pack('<7H', 1500, 1500, 1000, 1500,
                                   1800 if armed else 1000, 1800 if auto else 1000, 1000))
        self.event(150, status_frame(armed=armed if status_armed is None else status_armed,
                                     auto=auto if status_auto is None else status_auto))
        self.node.publish()

    def configuration(self):
        for code, payload in config_frames().items():
            self.event(code, payload)
        for name, value in (('msp_override_channels_mask', '15'), ('msp_override_failsafe', 'OFF'),
                            ('msp_override_timeout_ms', '50')):
            self.event(0x3010, f'{name} = {value}'.encode(), name=name)

    def test_forward_step_drops_cross_epoch_requests_and_requires_disarmed_low(self):
        self.ros += 3295.34
        self.steady += .01
        # The first callback after the step is the crossing STATUS response.
        self.event(150, status_frame(), request=10., request_steady=100.)
        d = self.node.decoder
        self.assertFalse(d.failed)
        self.assertTrue(d.clock_recovery_pending)
        self.assertEqual(self.node.clock_epoch.rejected_events, 1)
        self.assertFalse(self.node.authority_pub.messages[-1].receiver_valid)
        self.assertTrue(self.node.authority_pub.messages[-1].armed)  # Unknown is not disarmed.
        self.ros += .01
        self.steady += .01
        self.modes()
        self.assertFalse(self.node.authority_pub.messages[-1].receiver_valid)  # Configuration not fresh.
        self.configuration()
        for kwargs in (dict(armed=True), dict(auto=True), dict(status_armed=True), dict(status_auto=True)):
            self.modes(**kwargs)
            self.assertFalse(self.node.authority_pub.messages[-1].receiver_valid)
            self.assertTrue(d.clock_recovery_pending)
            self.ros += .001
            self.steady += .001
        self.modes()
        authority = self.node.authority_pub.messages[-1]
        self.assertTrue(authority.receiver_valid)
        self.assertFalse(authority.armed or authority.auto_switch)
        self.assertFalse(d.clock_recovery_pending)
        status = json.loads(self.node.status_pub.messages[-1].data)['clock']
        self.assertEqual(status['epoch_resets'], 1)
        self.assertAlmostEqual(status['last_step_seconds'], 3295.33)
        self.assertFalse(status['recovery_pending'])

    def test_backward_step_from_timer_recovers_using_only_new_epoch_evidence(self):
        self.ros -= 5.
        self.steady += .01
        self.node.publish()
        self.assertTrue(self.node.decoder.clock_recovery_pending)
        self.event(1, config_frames()[1], request=10., request_steady=100.)
        self.assertFalse(self.node.decoder.failed)
        self.assertEqual(self.node.clock_epoch.rejected_events, 1)
        self.ros += .01
        self.steady += .01
        self.configuration()
        self.modes()
        self.assertTrue(self.node.authority_pub.messages[-1].receiver_valid)
        self.assertFalse(self.node.decoder.clock_recovery_pending)

    def test_clock_recovery_never_clears_real_transport_or_configuration_fault(self):
        self.event(1, [], event='transport_error')
        self.ros += 100.
        self.steady += .01
        self.node.publish()
        self.configuration()
        self.modes()
        self.assertTrue(self.node.decoder.failed)
        self.assertFalse(self.node.authority_pub.messages[-1].receiver_valid)

    def test_changed_configuration_across_epoch_still_latches_failure(self):
        self.ros += 100.
        self.steady += .01
        self.node.publish()
        self.event(3, bytes((202, 7, 0)))
        self.assertTrue(self.node.decoder.failed)
        self.assertIn('Configuration changed', self.node.decoder.reason)

    def test_scheduler_pause_between_clock_reads_is_not_a_clock_step(self):
        with patch('msp_evidence.time.monotonic', side_effect=[100., 100.1, 100., 100.1, 100., 100.1]):
            self.node.publish()
        self.assertEqual(self.node.clock_epoch.resets, 0)
        self.assertFalse(self.node.decoder.failed)
        self.assertIn('Clock sampling delayed', self.node.decoder.mode_failure_reason)
        self.node.publish()
        self.assertFalse(self.node.authority_pub.messages[-1].receiver_valid)
        self.modes()
        self.assertTrue(self.node.authority_pub.messages[-1].receiver_valid)

    def test_transport_fault_latches_even_if_local_clock_sampling_is_delayed(self):
        with patch('msp_evidence.time.monotonic', side_effect=[100., 100.1, 100., 100.1, 100., 100.1]):
            self.event(1, [], event='transport_error')
        self.assertTrue(self.node.decoder.failed)


if __name__ == '__main__':
    unittest.main()
