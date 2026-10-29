#!/usr/bin/env python3
"""Pure contract tests; no ROS, sockets or physical devices."""
import json
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


def status_frame(armed=True, auto=False, failsafe=False, flags=0, pid_profile=0, rate_profile=0, angle=False):
    data = bytearray(26)
    data[6] = (4 if armed else 0) | (2 if auto else 0) | (1 if failsafe else 0) | (8 if angle else 0)
    struct.pack_into('<H', data, 4, 0x21)
    data[10] = pid_profile
    data[13] = 6
    data[14] = rate_profile
    data[16] = 30
    struct.pack_into('<I', data, 17, flags)
    return data


def ready_decoder(t=10., request_steady=None, angle=False):
    d = MspEvidence(EXPECTED)
    frames = config_frames()
    if angle:
        frames[119] += bytes((1,))
    for code, payload in frames.items():
        d.accept(code, payload, t, 'one')
    for name, value in [('msp_override_channels_mask', '15'), ('msp_override_failsafe', 'OFF'), ('msp_override_timeout_ms', '50')]:
        d.accept(0x3010, f'{name} = {value}'.encode(), t, 'one', name)
    d.accept(105, struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1000, 1000), t, 'one', request_steady=request_steady)
    d.accept(150, status_frame(angle=angle), t, 'one', request_steady=request_steady)
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
        self.assertFalse(d.snapshot(10.11)['kill'])
        self.assertTrue(math.isnan(d.snapshot(11.1)['battery_voltage']))

    def test_voltage_identity_survives_heartbeats_and_resets(self):
        d = ready_decoder()
        battery = bytearray(11)
        battery[0] = 6
        struct.pack_into('<H', battery, 9, 2250)
        d.accept(130, battery, 10.01, 'one', request_steady=20., observed_steady=20.02)
        first = d.snapshot(10.02)
        for _ in range(10):
            current = d.snapshot(10.1)
            for key in ('battery_sequence', 'battery_session', 'battery_request_steady_time', 'battery_receive_steady_time'):
                self.assertEqual(first[key], current[key])
        d.accept(130, battery, 10.01, 'one', request_steady=20.1, observed_steady=20.12)
        self.assertEqual(d.snapshot(10.2)['battery_sequence'], first['battery_sequence'])
        d.accept(130, battery, 10.5, 'one', request_steady=20.5, observed_steady=20.52)
        self.assertEqual(d.snapshot(10.6)['battery_sequence'], first['battery_sequence'] + 1)
        d.reset_clock_epoch()
        reset = d.snapshot(10.6)
        self.assertNotEqual(first['battery_session'], reset['battery_session'])
        self.assertTrue(math.isnan(reset['battery_request_steady_time']))
        self.assertTrue(math.isnan(reset['battery_voltage']))

    def test_bad_versions_ranges_and_mask(self):
        for code, payload in [(1, bytes((0, 1, 47))), (34, b'bad'), (150, b'bad'), (119, bytes((0, 0, 50)))]:
            d = ready_decoder()
            d.accept(code, payload, 10.01, 'one')
            self.assertFalse(d.snapshot(10.02)['receiver_valid'])
        d = ready_decoder()
        d.accept(0x3010, b'msp_override_channels_mask = 31', 10.01, 'one', 'msp_override_channels_mask')
        self.assertFalse(d.snapshot(10.02)['config_verified'])

    def test_calver_version_readback(self):
        # Actual hardware reply: 26, 6, 1, length 8, ASCII "2026.6.1".
        for label in (b'2026.6.1', b'2026.6.1-rc1', b'custom/git-0123456'):
            d = ready_decoder()
            d.frames[3] = bytes((26, 6, 1, len(label))) + label, 10.
            s = d.snapshot(10.02)
            self.assertTrue(s['config_verified'], s)
            self.assertTrue(s['receiver_valid'], s)

    def test_malformed_calver_version_readback(self):
        valid = bytes.fromhex('1a060108323032362e362e31')
        payloads = (b'', valid[:2], valid[:4], valid[:-1], valid + b'\x00',
                    bytes((26, 6, 1, 0)), bytes((26, 6, 1, 7)) + b'2026.6.1',
                    bytes((26, 6, 1, 8)) + b'2026.6.\xff',
                    bytes((26, 6, 1, 12)) + b'2026.6.1-rc\x001')
        for payload in payloads:
            with self.subTest(payload=payload):
                d = ready_decoder()
                d.frames[3] = payload, 10.
                s = d.snapshot(10.02)
                self.assertFalse(s['config_verified'], s)
                self.assertFalse(s['receiver_valid'], s)
                self.assertEqual(s['reason'], 'Malformed FC version readback')

    def test_calver_keeps_api_and_override_requirements(self):
        for code, payload in ((1, bytes((0, 1, 47))), (2, b'INAV'), (119, bytes((27, 0)))):
            with self.subTest(code=code):
                d = ready_decoder()
                d.frames[3] = bytes.fromhex('1a060108323032362e362e31'), 10.
                d.frames[code] = payload, 10.
                self.assertFalse(d.snapshot(10.02)['config_verified'])
        for name, value in (('msp_override_channels_mask', '0'), ('msp_override_failsafe', 'ON'),
                            ('msp_override_timeout_ms', '300')):
            with self.subTest(name=name):
                d = ready_decoder()
                d.frames[3] = bytes.fromhex('1a060108323032362e362e31'), 10.
                d.settings[name] = value, 10.
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
        self.assertEqual(waiting['rc_stamp'], 10.02)
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
        # A confirmed low persists regardless of age, without refreshing its timestamp.
        d = ready_decoder()
        d.snapshot(10.01)
        d.accept(105, high, 10.02, 'one')
        d.accept(150, status_frame(), 10.08, 'one')
        self.assertFalse(d.snapshot(10.11)['kill'])
        # KILL arriving during the pair wait is never deferred.
        d = ready_decoder()
        d.snapshot(10.01)
        d.accept(105, struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1800, 1800), 10.02, 'one')
        self.assertTrue(d.snapshot(10.03)['kill'])

    def test_angle_to_auto_wait_preserves_manual_health_in_both_reply_orders(self):
        high = struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1800, 1000)
        for rc_first in (True, False):
            with self.subTest(rc_first=rc_first):
                d = ready_decoder(request_steady=100., angle=True)
                self.assertTrue(d.snapshot(10.01, steady_now=100.01)['control_mode_ok'])
                events = [(105, high), (150, status_frame(auto=True))]
                if not rc_first:
                    events.reverse()
                for i, (code, payload) in enumerate(events):
                    t = 10.02 + .02*i
                    d.accept(code, payload, t, 'one', observed_steady=t+90., request_steady=t+90.)
                    result = d.snapshot(t+.001, steady_now=t+90.001)
                    self.assertTrue(result['receiver_valid'] and result['config_verified'], result)
                    self.assertTrue(result['control_mode_ok'], result)
                    self.assertFalse(result['kill'])
                    self.assertEqual(result['auto_switch'], i == 1)
                    if i == 0:
                        versions = dict(d.mode_versions)
                        requests = dict(d.mode_request_steady)
                        # Heartbeats and duplicate replies cannot confirm the rise.
                        d.accept(code, payload, t, 'one', observed_steady=t+90., request_steady=t+90.)
                        repeated = d.snapshot(t+.002, steady_now=t+90.002)
                        self.assertFalse(repeated['auto_switch'])
                        self.assertEqual(d.mode_versions, versions)
                        self.assertEqual(d.mode_request_steady, requests)

    def test_recorded_third_auto_transition_keeps_health_until_confirmed(self):
        fixture = json.loads((Path(__file__).parent / 'data' / 'auto_mode_transition_20261023.json').read_text())
        d = MspEvidence(fixture['expected'])
        checked = 0
        for event in fixture['events']:
            d.accept(event['code'], bytes.fromhex(event['payload']), event['stamp'], event['session'], event['name'],
                     observed_steady=event['observed_steady'], request_steady=event['request_steady'])
            result = d.snapshot(event['now'], steady_now=event['observed_steady'])
            if event['t'] < 129.2:
                continue
            checked += 1
            self.assertTrue(result['receiver_valid'] and result['config_verified'], result)
            self.assertTrue(result['control_mode_ok'], result)
            self.assertEqual(result['auto_switch'], event['t'] >= 129.377257505)
            if event['code'] == 105 and event['t'] == 129.373044204:
                self.assertIn('Waiting for coherent AUTO rise', result['reason'])
                self.assertAlmostEqual(d.mode_request_steady[105] - d.mode_request_steady[150], .033927601)
        self.assertEqual(checked, 16)

    def test_waiting_never_hides_same_status_auto_and_angle_conflict(self):
        d = ready_decoder(angle=True)
        d.snapshot(10.005)
        # Physical low is authoritative even if the previous STATUS is still high.
        d.accept(150, status_frame(auto=True, angle=True), 10.01, 'one')
        d.accept(105, struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1000, 1000), 10.012, 'one')
        self.assertTrue(d.snapshot(10.02)['control_mode_ok'])
        d.accept(105, struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1800, 1000), 10.03, 'one')
        result = d.snapshot(10.04)
        self.assertTrue(result['receiver_valid'])
        self.assertFalse(result['auto_switch'], 'STATUS predates the observed physical low')
        self.assertFalse(result['control_mode_ok'], result)
        self.assertIn('AUTO conflicts', result['reason'])

    def test_repeated_high_polls_do_not_extend_cached_manual_status_grace(self):
        d = ready_decoder(angle=True)
        d.snapshot(10.01)
        high = struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1800, 1000)
        d.accept(105, high, 10.02, 'one')
        self.assertTrue(d.snapshot(10.025)['control_mode_ok'])
        # One new STATUS is enough to distinguish a persistent conflict from
        # the cached manual STATUS at the edge. New RC requests cannot renew it.
        for code, payload, t in ((150, status_frame(angle=True), 10.04), (105, high, 10.06),
                                 (150, status_frame(angle=True), 10.08)):
            d.accept(code, payload, t, 'one')
            result = d.snapshot(t+.001)
            self.assertTrue(result['receiver_valid'])
            self.assertFalse(result['auto_switch'] or result['control_mode_ok'], result)
        # Only a new physical low begins another transition.
        d.accept(105, struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1000, 1000), 10.09, 'one')
        self.assertTrue(d.snapshot(10.095)['control_mode_ok'])
        d.accept(105, high, 10.10, 'one')
        self.assertTrue(d.snapshot(10.105)['control_mode_ok'])
        self.assertFalse(d.snapshot(10.105)['auto_switch'])

    def test_angle_wait_without_previous_receiver_snapshot_cannot_invent_low(self):
        d = ready_decoder(angle=True)
        d.accept(105, struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1800, 1000), 10.02, 'one')
        result = d.snapshot(10.03)
        self.assertFalse(result['receiver_valid'] or result['control_mode_ok'])
        self.assertIsNone(d.receiver_snapshot)

    def test_angle_after_confirmed_auto_remains_a_conflict(self):
        for fc_auto in (True, False):
            with self.subTest(fc_auto=fc_auto):
                d = ready_decoder(angle=True)
                d.snapshot(10.01)
                d.accept(105, struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1800, 1000), 10.02, 'one')
                d.accept(150, status_frame(auto=True), 10.03, 'one')
                self.assertTrue(d.snapshot(10.04)['auto_switch'])
                d.accept(150, status_frame(auto=fc_auto, angle=True), 10.05, 'one')
                result = d.snapshot(10.06)
                self.assertTrue(result['auto_switch'], 'A mode conflict must not fabricate physical low')
                self.assertFalse(result['control_mode_ok'], result)

    def test_angle_wait_preserves_kill_arm_and_failsafe_checks(self):
        for fault in ('kill', 'arm_low', 'rx_failsafe'):
            with self.subTest(fault=fault):
                d = ready_decoder(angle=True)
                d.snapshot(10.01)
                d.accept(105, struct.pack('<7H', 1500, 1500, 1000, 1500,
                                         1000 if fault == 'arm_low' else 1800,
                                         1800, 1800 if fault == 'kill' else 1000), 10.02, 'one')
                if fault == 'rx_failsafe':
                    d.accept(150, status_frame(angle=True, flags=4), 10.025, 'one')
                result = d.snapshot(10.03)
                self.assertFalse(result['auto_switch'])
                if fault == 'arm_low':
                    self.assertFalse(result['armed'])
                else:
                    self.assertTrue(result['kill'])
                if fault == 'rx_failsafe':
                    self.assertFalse(result['rc_link'])

    def test_each_successful_mode_request_expires_independently(self):
        for stale_code in (105, 150):
            with self.subTest(stale_code=stale_code):
                d = ready_decoder()
                d.snapshot(10.01)
                high = struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1800, 1000)
                d.accept(105, high, 10.02, 'one')
                d.accept(150, status_frame(auto=True), 10.03, 'one')
                self.assertTrue(d.snapshot(10.04)['receiver_valid'])
                other = 150 if stale_code == 105 else 105
                d.accept(other, status_frame(auto=True) if other == 150 else high, 10.13, 'one')
                # ACK/config/other-mode traffic cannot extend either request.
                d.accept(200, b'', 10.14, 'one', event='ack')
                result = d.snapshot(10.141)
                self.assertFalse(result['receiver_valid'], result)
                self.assertTrue(result['auto_switch'] and result['armed'])
                self.assertFalse(result['kill'], 'Stale evidence impersonated a physical switch change')
                d.accept(105, high, 10.15, 'one')
                d.accept(150, status_frame(auto=True), 10.16, 'one')
                self.assertFalse(d.snapshot(10.17)['receiver_valid'], 'Fresh high silently recovered authority')
                low = struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1000, 1000)
                d.accept(105, low, 10.18, 'one')
                self.assertTrue(d.snapshot(10.19)['receiver_valid'])
                self.assertFalse(d.snapshot(10.19)['auto_switch'])
                d.accept(105, high, 10.20, 'one')
                self.assertFalse(d.snapshot(10.21)['auto_switch'], 'Old STATUS high reused after low')
                d.accept(150, status_frame(auto=True), 10.22, 'one')
                self.assertTrue(d.snapshot(10.23)['auto_switch'])

    def test_ros_offset_change_cannot_refresh_original_monotonic_requests(self):
        d = ready_decoder(request_steady=100.)
        self.assertTrue(d.snapshot(10.01, steady_now=100.01)['receiver_valid'])
        # ROS advanced only 70 ms while monotonic elapsed 110 ms: a 40 ms
        # backward wall-clock adjustment must not prolong either request.
        self.assertFalse(d.snapshot(10.07, steady_now=100.11)['receiver_valid'])
        self.assertEqual(d.mode_request_steady, {105: 100., 150: 100.})
        d = ready_decoder(request_steady=100.01)
        self.assertFalse(d.snapshot(10.01, steady_now=100.)['receiver_valid'])
        d = ready_decoder()
        self.assertFalse(d.snapshot(10.01, steady_now=100.)['receiver_valid'], 'Missing steady request became fresh')

    def test_timeout_unmatched_and_session_change_preserve_unknown_switches(self):
        high = struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1800, 1000)
        low = struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1000, 1000)
        for event in ('mode_timeout', 'mode_rx_unmatched', 'session_change'):
            with self.subTest(event=event):
                d = ready_decoder()
                d.snapshot(10.01)
                d.accept(105, high, 10.02, 'one')
                d.accept(150, status_frame(auto=True), 10.03, 'one')
                self.assertTrue(d.snapshot(10.04)['auto_switch'])
                if event == 'session_change':
                    d.accept(150, status_frame(auto=True), 10.05, 'two')
                else:
                    d.accept(105, low, 0., 'one', event=event, observed_steady=100.)
                result = d.snapshot(10.06)
                self.assertFalse(result['receiver_valid'])
                self.assertTrue(result['auto_switch'], 'Unknown/late RC manufactured physical low')
                self.assertTrue(result['armed'])
                self.assertFalse(result['kill'])
                self.assertFalse(d.failed)
                if event == 'session_change':
                    d.accept(105, low, 10.07, 'one')
                    self.assertEqual(d.session, 'two')
                    self.assertTrue(d.snapshot(10.08)['auto_switch'])

    def test_status_low_alone_does_not_fabricate_auto_edge(self):
        d = ready_decoder()
        d.snapshot(10.01)
        high = struct.pack('<7H', 1500, 1500, 1000, 1500, 1800, 1800, 1000)
        d.accept(105, high, 10.02, 'one')
        d.accept(150, status_frame(auto=True), 10.03, 'one')
        self.assertTrue(d.snapshot(10.04)['auto_switch'])
        d.accept(150, status_frame(auto=False), 10.05, 'one')
        self.assertTrue(d.snapshot(10.06)['auto_switch'])

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
