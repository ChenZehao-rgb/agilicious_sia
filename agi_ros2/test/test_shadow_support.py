#!/usr/bin/env python3
"""Pure contract tests; no ROS, sockets or physical devices."""
import math
from pathlib import Path
import struct
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from shadow_support import OriginBuilder, local_position, MspEvidence, navigation_accuracy_ok, evidence_config_path

EXPECTED = dict(rx_map=[0, 1, 3, 2, 4, 5, 6, 7], aux_low=1700, aux_high=2100,
                center_rate_deg_s=[70., 70., 70.], max_rate_deg_s=[1000., 1000., 1000.],
                expo_percent=[0., 0., 0.], deadband=0, yaw_deadband=0, min_check=1050)


def config_frames():
    rx = bytearray(40)
    struct.pack_into('<H', rx, 3, 1500)
    struct.pack_into('<H', rx, 5, 1050)
    rates = bytearray(24)
    for i in (0, 11, 12):
        rates[i] = 7
    for i in (2, 3, 4):
        rates[i] = 100
    rates[22] = 3
    for i in range(3):
        struct.pack_into('<H', rates, 16+2*i, 1000)
    return {1: bytes((0, 1, 48)), 2: b'BTFL', 3: bytes((202, 6, 0)),
            34: bytes((0, 0, 32, 48, 50, 1, 32, 48, 27, 2, 32, 48)),
            238: bytes((3, 0, 0, 0, 50, 0, 0, 27, 0, 0)),
            64: bytes((0, 1, 3, 2, 4, 5, 6, 7)), 44: rx, 119: bytes((27, 50, 0)), 111: rates, 125: bytes(5)}


def status_frame(armed=True, auto=False, failsafe=False, flags=0, pid_profile=0, rate_profile=0):
    data = bytearray(26)
    data[6] = (4 if armed else 0) | (2 if auto else 0) | (1 if failsafe else 0)
    struct.pack_into('<H', data, 4, 0x21)
    data[10] = pid_profile
    data[13] = 6
    data[14] = rate_profile
    data[16] = 30
    struct.pack_into('<I', data, 17, flags)
    return data


def ready_decoder(t=10.):
    d = MspEvidence(EXPECTED)
    for code, payload in config_frames().items():
        d.accept(code, payload, t, 'one')
    for name, value in [('msp_override_channels_mask', '15'), ('msp_override_failsafe', 'OFF'), ('msp_override_timeout_ms', '50')]:
        d.accept(0x3010, f'{name} = {value}'.encode(), t, 'one', name)
    d.accept(105, struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1000, 1000), t, 'one')
    d.accept(150, status_frame(), t, 'one')
    return d


class NavigationTests(unittest.TestCase):
    def test_accuracy_requires_real_values_and_configured_limits(self):
        self.assertTrue(navigation_accuracy_ok((1., 2., .2), (2., 3., .5)))
        for values in ((math.nan, 2., .2), (0., 2., .2), (3., 2., .2), (1., 4., .2), (1., 2., .6)):
            self.assertFalse(navigation_accuracy_ok(values, (2., 3., .5)))
        self.assertFalse(navigation_accuracy_ok((1., 2., .2), (0., 3., .5)))
        self.assertFalse(navigation_accuracy_ok((), ()))

    def test_axes_and_height(self):
        east = local_position((0, 0, 100), (0, .001, 100), 'ellipsoid')
        north = local_position((0, 0, 100), (.001, 0, 100), 'ellipsoid')
        self.assertAlmostEqual(east[0], 111.32, delta=.02)
        self.assertAlmostEqual(north[1], 110.57, delta=.02)
        self.assertAlmostEqual(east[1], 0)
        self.assertEqual(local_position((40, 120, 200), (40, 120, 205), 'msl')[2], 5)

    def test_origin_stationarity_and_reconnect(self):
        b = OriginBuilder()
        for i in range(31):
            p = b.add(10+i*.1, 'one', 30, 120, 10, 'ellipsoid', (0, 0, 0))
        self.assertIsNotNone(p)
        origin = b.origin
        self.assertIsNotNone(b.add(20, 'two', 30.001, 120, 10, 'ellipsoid', (1, 0, 0)))
        self.assertEqual(b.origin, origin)
        self.assertIsNone(b.add(20, 'two', 30, 120, 10, 'ellipsoid', (0, 0, 0)))
        self.assertIsNone(b.add(21, 'two', 30, 120, 10, 'msl', (0, 0, 0)))

    def test_invalid_and_gaps_reset_acquisition(self):
        b = OriginBuilder(duration=.2, count=3)
        self.assertIsNone(b.add(1, 's', 30, 120, 10, 'unknown', (0, 0, 0)))
        self.assertIsNone(b.add(1, 's', 30, 120, math.nan, 'ellipsoid', (0, 0, 0)))
        b.add(2, 's', 30, 120, 10, 'ellipsoid', (0, 0, 0))
        b.add(2.1, 's', 30, 120, 10, 'ellipsoid', (1, 0, 0))
        b.add(2.2, 's', 30, 120, 10, 'ellipsoid', (0, 0, 0))
        self.assertIsNone(b.add(3, 's', 30, 120, 10, 'ellipsoid', (0, 0, 0)))
        self.assertEqual(len(b.samples), 1)


class EvidenceTests(unittest.TestCase):
    def test_startup_config_conflict_and_fixed_timeout_policy(self):
        self.assertEqual(evidence_config_path('/tmp/runtime.yaml', ''), '/tmp/runtime.yaml')
        self.assertEqual(evidence_config_path('', '/tmp/bridge.yaml'), '/tmp/bridge.yaml')
        for runtime, bridge in (('/tmp/runtime.yaml', '/tmp/bridge.yaml'), ('', '')):
            with self.assertRaises(ValueError):
                evidence_config_path(runtime, bridge)
        self.assertEqual(MspEvidence(EXPECTED).expected_override_timeout_ms, 50)
        self.assertEqual(MspEvidence(dict(EXPECTED, expected_override_timeout_ms=50)).expected_override_timeout_ms, 50)
        for timeout in (0, 49, 51, 300, 50., True):
            with self.assertRaises(ValueError):
                MspEvidence(dict(EXPECTED, expected_override_timeout_ms=timeout))

    def test_dynamic_boxes_and_failsafe(self):
        d = ready_decoder()
        s = d.snapshot(10.04)
        self.assertTrue(s['config_verified'], s)
        self.assertTrue(s['armed'], s)
        self.assertTrue(s['rc_link'])
        self.assertFalse(s['kill'])
        d.accept(150, status_frame(flags=4), 10.05, 'one')
        self.assertFalse(d.snapshot(10.06)['rc_link'])
        self.assertTrue(d.snapshot(10.06)['kill'])

    def test_age_not_refreshed_battery_independent(self):
        d = ready_decoder()
        battery = bytearray(11)
        battery[0] = 4
        struct.pack_into('<H', battery, 9, 1600)
        d.accept(130, battery, 9.5, 'one')
        self.assertEqual(d.snapshot(10.04)['battery_voltage'], 16.)
        for _ in range(10):
            self.assertEqual(d.snapshot(10.04)['rc_stamp'], 10.)
        self.assertTrue(d.snapshot(10.11)['kill'])
        self.assertTrue(math.isnan(d.snapshot(11.1)['battery_voltage']))

    def test_bad_versions_ranges_and_mask(self):
        for code, payload in [(1, bytes((0, 1, 47))), (34, b'bad'), (150, b'bad'), (119, bytes((0, 0, 50)))]:
            d = ready_decoder()
            d.accept(code, payload, 10.01, 'one')
            self.assertFalse(d.snapshot(10.02)['receiver_valid'])
        d = ready_decoder()
        d.accept(0x3010, b'msp_override_channels_mask = 31', 10.01, 'one', 'msp_override_channels_mask')
        self.assertFalse(d.snapshot(10.02)['config_verified'])

    def test_stale_ignored_timeout_latched_and_session_reset(self):
        d = ready_decoder()
        d.accept(105, bytes(14), 9., 'one')
        self.assertTrue(d.snapshot(10.01)['armed'])
        d.accept(105, b'', 10.01, 'one', event='timeout')
        self.assertFalse(d.snapshot(10.02)['transport_healthy'])
        d.accept(150, status_frame(), 10.03, 'one')
        self.assertFalse(d.snapshot(10.04)['config_verified'])
        d.accept(150, status_frame(), 10.05, 'two')
        self.assertFalse(d.snapshot(10.06)['receiver_valid'])

    def test_auto_mismatch_and_arm_actual(self):
        d = ready_decoder()
        d.accept(150, status_frame(auto=True), 10.01, 'one')
        self.assertTrue(d.snapshot(10.02)['kill'])
        d.accept(150, status_frame(armed=False), 10.03, 'one')
        self.assertFalse(d.snapshot(10.04)['armed'])

    def test_timeout_readback_profiles_and_calibration(self):
        for value in ('300', '0', 'unknown', '-1'):
            d = ready_decoder()
            d.accept(0x3010, f'msp_override_timeout_ms = {value}'.encode(), 10.01, 'one',
                     'msp_override_timeout_ms')
            self.assertFalse(d.snapshot(10.02)['config_verified'])
        for pid, rate in ((1, 0), (0, 1)):
            d = ready_decoder()
            d.accept(150, status_frame(pid_profile=pid, rate_profile=rate), 10.01, 'one')
            self.assertFalse(d.snapshot(10.02)['config_verified'])
            self.assertTrue(d.snapshot(10.02)['kill'])
        for flags in (1 << 12, 1 << 23):
            d = ready_decoder()
            self.assertTrue(d.snapshot(10.01)['imu_ready'])
            d.accept(150, status_frame(flags=flags), 10.01, 'one')
            self.assertFalse(d.snapshot(10.02)['imu_ready'])

    def test_mode_conflicts_and_optional_telemetry(self):
        for box in MspEvidence.CONFLICTING_MODES:
            d = ready_decoder()
            # Extend the startup fixture before examining the first snapshot.
            d.frames[119] = bytes((27, 50, 0, box)), 10.
            frame = status_frame(auto=True)
            frame[6] |= 8
            d.accept(150, frame, 10.01, 'one')
            d.accept(105, struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1800, 1000), 10.01, 'one')
            s = d.snapshot(10.02)
            self.assertTrue(s['receiver_valid'], s)
            self.assertFalse(s['control_mode_ok'], s)
            self.assertTrue(s['auto_switch'])
        d = ready_decoder()
        d.accept(108, b'', 10.01, 'one', event='optional_timeout')
        d.accept(200, b'', 0., 'one', event='ack')
        self.assertTrue(d.snapshot(10.02)['config_verified'])
        self.assertTrue(d.snapshot(10.02)['transport_healthy'])

    def test_rc_status_rise_pairing_never_refreshes_or_fabricates_low(self):
        high = struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1800, 1000)
        d = ready_decoder()
        self.assertFalse(d.snapshot(10.01)['auto_switch'])
        d.accept(105, high, 10.02, 'one')
        waiting = d.snapshot(10.03)
        self.assertFalse(waiting['kill'])
        self.assertFalse(waiting['auto_switch'])
        self.assertEqual(waiting['rc_stamp'], 10.)
        d.accept(150, status_frame(auto=True), 10.04, 'one')
        self.assertTrue(d.snapshot(10.05)['auto_switch'])
        self.assertFalse(d.snapshot(10.05)['kill'])
        d = ready_decoder()
        d.snapshot(10.01)
        d.accept(150, status_frame(auto=True), 10.02, 'one')
        self.assertFalse(d.snapshot(10.03)['kill'])
        d.accept(105, high, 10.04, 'one')
        self.assertTrue(d.snapshot(10.05)['auto_switch'])
        # Startup with only inconsistent high/low evidence cannot invent a low.
        d = ready_decoder()
        d.accept(105, high, 10.02, 'one')
        self.assertTrue(d.snapshot(10.03)['kill'])
        # A coherent old low also cannot be stretched past its original age.
        d = ready_decoder()
        d.snapshot(10.01)
        d.accept(105, high, 10.02, 'one')
        d.accept(150, status_frame(), 10.08, 'one')
        self.assertTrue(d.snapshot(10.11)['kill'])
        # KILL arriving during the pair wait is never deferred.
        d = ready_decoder()
        d.snapshot(10.01)
        d.accept(105, struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1800, 1800), 10.02, 'one')
        self.assertTrue(d.snapshot(10.03)['kill'])

    def test_configurable_authority_aux_indices(self):
        expected = dict(EXPECTED, arm_aux=7, auto_aux=4, kill_aux=13)
        d = MspEvidence(expected)
        frames = config_frames()
        ranges = bytearray(frames[34])
        ranges[1], ranges[5], ranges[9] = 7, 4, 13
        frames[34] = ranges
        for code, payload in frames.items():
            d.accept(code, payload, 10., 'custom')
        for key, value in [('msp_override_channels_mask', '15'), ('msp_override_failsafe', 'OFF'),
                           ('msp_override_timeout_ms', '50')]:
            d.accept(0x3010, f'{key} = {value}'.encode(), 10., 'custom', key)
        channels = [1500] * 18
        channels[11], channels[8], channels[17] = 1800, 1000, 1000
        d.accept(105, struct.pack('<18H', *channels), 10., 'custom')
        d.accept(150, status_frame(), 10., 'custom')
        self.assertTrue(d.snapshot(10.01)['armed'])
        self.assertFalse(d.snapshot(10.01)['kill'])
        channels[8] = 1800
        d.accept(105, struct.pack('<18H', *channels), 10.02, 'custom')
        d.accept(150, status_frame(auto=True), 10.02, 'custom')
        self.assertTrue(d.snapshot(10.03)['auto_switch'])
        channels[17] = 1800
        d.accept(105, struct.pack('<18H', *channels), 10.04, 'custom')
        self.assertTrue(d.snapshot(10.05)['kill'])
        d.accept(105, struct.pack('<7H', *channels[:7]), 10.06, 'custom')
        self.assertFalse(d.snapshot(10.07)['receiver_valid'])
        for options in (dict(arm_aux=14), dict(auto_aux=-1), dict(kill_aux=0), dict(arm_aux=True)):
            with self.assertRaises(ValueError):
                MspEvidence(dict(EXPECTED, **options))


if __name__ == '__main__':
    unittest.main()
