#!/usr/bin/env python3
"""Pure contract tests; no ROS, sockets or physical devices."""
import math
from pathlib import Path
import struct
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from shadow_support import OriginBuilder, local_position, MspEvidence

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


def status_frame(armed=True, auto=False, failsafe=False, flags=0):
    data = bytearray(26)
    data[6] = (4 if armed else 0) | (2 if auto else 0) | (1 if failsafe else 0)
    data[16] = 30
    struct.pack_into('<I', data, 17, flags)
    return data


def ready_decoder(t=10.):
    d = MspEvidence(EXPECTED)
    for code, payload in config_frames().items():
        d.accept(code, payload, t, 'one')
    for name, value in [('msp_override_channels_mask', '15'), ('msp_override_failsafe', 'OFF')]:
        d.accept(0x3010, f'{name} = {value}'.encode(), t, 'one', name)
    d.accept(105, struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1000, 1000), t, 'one')
    d.accept(101, status_frame(), t, 'one')
    return d


class NavigationTests(unittest.TestCase):
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
    def test_dynamic_boxes_and_failsafe(self):
        d = ready_decoder()
        s = d.snapshot(10.04)
        self.assertTrue(s['config_verified'], s)
        self.assertTrue(s['armed'], s)
        self.assertTrue(s['rc_link'])
        self.assertFalse(s['kill'])
        d.accept(101, status_frame(flags=4), 10.05, 'one')
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
        for code, payload in [(1, bytes((0, 1, 47))), (34, b'bad'), (101, b'bad'), (119, bytes((0, 0, 50)))]:
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
        d.accept(101, status_frame(), 10.03, 'one')
        self.assertFalse(d.snapshot(10.04)['config_verified'])
        d.accept(101, status_frame(), 10.05, 'two')
        self.assertFalse(d.snapshot(10.06)['receiver_valid'])

    def test_auto_mismatch_and_arm_actual(self):
        d = ready_decoder()
        d.accept(101, status_frame(auto=True), 10.01, 'one')
        self.assertTrue(d.snapshot(10.02)['kill'])
        d.accept(101, status_frame(armed=False), 10.03, 'one')
        self.assertFalse(d.snapshot(10.04)['armed'])


if __name__ == '__main__':
    unittest.main()
