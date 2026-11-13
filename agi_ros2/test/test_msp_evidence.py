#!/usr/bin/env python3
"""Deterministic health freshness checks with real messages and MSP decoding."""
import math
import struct
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from agi_ros2.msg import FcTelemetry, FusedState, MspEvent, OutputStatus
from builtin_interfaces.msg import Time
from msp_evidence import ClockEpoch, EvidenceNode, TelemetryEvidence, output_freshness, stamp


def telemetry(now=10., steady=100., **changes):
    m = FcTelemetry()
    m.header.stamp = stamp(now)
    m.clock_id, m.source_session = 'test-boot', 'session-one'
    m.clock_aligned = m.transport_healthy = True
    m.rc_stamp = m.heartbeat_stamp = m.system_stamp = m.battery_stamp = stamp(now)
    m.rc_sample_steady_time = m.rc_receive_steady_time = steady
    m.heartbeat_receive_steady_time = m.system_receive_steady_time = m.battery_receive_steady_time = steady
    m.channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
    m.base_mode, m.system_status = 0, 3
    m.sensors_present = m.sensors_enabled = m.sensors_health = 3
    m.battery_voltages_mv = [4000]*4 + [65535]*6
    m.battery_current_ca, m.battery_consumed_mah = 123, 456
    m.battery_remaining_percent, m.battery_temperature_cdeg = 75, 2534
    m.system_voltage_mv = 15000
    for key, value in changes.items():
        setattr(m, key, value)
    return m


def ready_decoder():
    decoder = TelemetryEvidence('test-boot')
    decoder.accept(telemetry())
    assert decoder.snapshot(10., steady_now=100.)['receiver_valid']
    return decoder



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
            decoder=ready_decoder(), battery_timeout=1.5, clock_id='test-boot',
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



class TelemetryEvidenceTest(unittest.TestCase):
    def test_channel_edges_and_actual_armed_are_independent(self):
        for index, field in ((4, 'arm_switch'), (7, 'auto_switch')):
            for value, high in ((1874, False), (1875, True), (2099, True), (2100, False)):
                with self.subTest(index=index, value=value):
                    d = ready_decoder()
                    m = telemetry(10.04, 100.04)
                    m.channels[index] = value
                    d.accept(m)
                    data = d.snapshot(10.04, steady_now=100.04)
                    self.assertTrue(data['receiver_valid'])
                    self.assertEqual(data[field], high)
                    self.assertFalse(data['armed'])
        d = ready_decoder()
        d.accept(telemetry(10.04, 100.04, base_mode=128))
        data = d.snapshot(10.04, steady_now=100.04)
        self.assertTrue(data['armed'])
        self.assertFalse(data['arm_switch'])
        self.assertFalse(data['config_verified'] or data['kill'])

    def test_each_source_expires_and_republish_does_not_refresh(self):
        for field in ('rc_sample_steady_time', 'rc_receive_steady_time', 'heartbeat_receive_steady_time',
                      'system_receive_steady_time'):
            with self.subTest(field=field):
                d = ready_decoder()
                m = telemetry(10.04, 100.04)
                m.channels[7] = 2000
                m.base_mode = 128
                d.accept(m)
                self.assertTrue(d.snapshot(10.04, steady_now=100.04)['auto_switch'])
                stale = telemetry(11., 101.)
                stale.channels[7] = 2000
                setattr(stale, field, 100.)
                d.accept(stale)
                data = d.snapshot(11., steady_now=101.)
                self.assertFalse(data['receiver_valid'])
                self.assertTrue(data['auto_switch'])
                d.accept(telemetry(11.04, 101.04, channels=[int(v) for v in stale.channels]))
                self.assertFalse(d.snapshot(11.04, steady_now=101.04)['receiver_valid'])
                d.accept(telemetry(11.08, 101.08))
                self.assertTrue(d.snapshot(11.08, steady_now=101.08)['receiver_valid'])
        d = ready_decoder()
        self.assertTrue(d.snapshot(10.499, steady_now=100.499)['receiver_valid'])
        self.assertFalse(d.snapshot(10.501, steady_now=100.501)['receiver_valid'])

    def test_restart_transport_clock_failure_require_low(self):
        for change in ({'source_session': 'reboot'}, {'transport_healthy': False}, {'clock_aligned': False},
                       {'system_status': 5}):
            with self.subTest(change=change):
                d = ready_decoder()
                m = telemetry(10.04, 100.04)
                m.channels[7] = 2000
                d.accept(m)
                self.assertTrue(d.snapshot(10.04, steady_now=100.04)['receiver_valid'])
                m = telemetry(10.08, 100.08, **change)
                m.channels[7] = 2000
                d.accept(m)
                self.assertFalse(d.snapshot(10.08, steady_now=100.08)['receiver_valid'])
                session = m.source_session
                d.accept(telemetry(10.12, 100.12, source_session=session, channels=[int(v) for v in m.channels]))
                self.assertFalse(d.snapshot(10.12, steady_now=100.12)['receiver_valid'])
                d.accept(telemetry(10.16, 100.16, source_session=session))
                self.assertTrue(d.snapshot(10.16, steady_now=100.16)['receiver_valid'])

    def test_battery_units_pack_cells_extended_unknowns_and_precedence(self):
        cases = [([24000]+[65535]*9, [0]*4, 24.), ([4000]*6+[65535]*4, [0]*4, 24.),
                 ([4000]*10, [4000, 4000, 0, 0], 48.), ([65534, 4466]+[65535]*8, [0]*4, 70.)]
        for volts, ext, expected in cases:
            with self.subTest(volts=volts):
                d = ready_decoder()
                d.accept(telemetry(10.04, 100.04, battery_voltages_mv=volts, battery_voltages_ext_mv=ext))
                data = d.snapshot(10.04, steady_now=100.04)
                self.assertAlmostEqual(data['battery_voltage'], expected)
                self.assertAlmostEqual(data['battery_current_a'], 1.23)
                self.assertEqual(data['battery_consumed_mah'], 456.)
                self.assertEqual(data['battery_remaining_percent'], 75.)
                self.assertAlmostEqual(data['battery_temperature_c'], 25.34)
        d = ready_decoder()
        d.accept(telemetry(10.04, 100.04, battery_current_ca=-1, battery_consumed_mah=-1,
                           battery_remaining_percent=-1, battery_temperature_cdeg=32767))
        data = d.snapshot(10.04, steady_now=100.04)
        for key in ('battery_current_a', 'battery_consumed_mah', 'battery_remaining_percent', 'battery_temperature_c'):
            self.assertTrue(math.isnan(data[key]))

    def test_battery_fallback_expiry_and_default_remaining_is_not_evidence(self):
        d = ready_decoder()
        m = telemetry(10.04, 100.04, battery_voltages_mv=[65535]*10)
        d.accept(m)
        self.assertAlmostEqual(d.snapshot(10.04, steady_now=100.04)['battery_voltage'], 15.)
        m = telemetry(10.08, 100.08, battery_voltages_mv=[65535]*10, system_voltage_mv=0,
                      system_remaining_percent=100)
        d.accept(m)
        data = d.snapshot(10.08, steady_now=100.08)
        self.assertTrue(math.isnan(data['battery_voltage']))
        self.assertTrue(math.isnan(data['battery_remaining_percent']))
        d = ready_decoder()
        self.assertTrue(math.isnan(d.snapshot(11.501, steady_now=101.501)['battery_voltage']))

    def test_wrong_clock_and_old_snapshot_do_not_replace_source(self):
        d = ready_decoder()
        d.accept(telemetry(11., 101., clock_id='other'))
        d.accept(telemetry(9., 99., source_session='old'))
        self.assertEqual(d.session, 'session-one')
        self.assertEqual(d.message.rc_sample_steady_time, 100.)
        d.reset_clock_epoch()
        self.assertFalse(d.snapshot(10., steady_now=100.)['receiver_valid'])
        self.assertTrue(d.recovery_pending)
        epoch = ClockEpoch(10., 100.)
        self.assertFalse(epoch.observe(10.1, 100.1))
        self.assertTrue(epoch.observe(12., 100.2))


if __name__ == '__main__':
    unittest.main()
