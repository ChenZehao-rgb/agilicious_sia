#!/usr/bin/env python3
"""Real MAVLink receiver with a variable-rate simulated FC on a PTY.

No physical UART or control output is opened. Source ROS and the local install;
AGI_MAVLINK_SENSOR_BINARY may select a candidate build. Requires pymavlink.
"""
import math
from pathlib import Path
import struct
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'test'))
from test_mavlink_sensor import Harness, mav, rclpy
from agi_ros2.msg import FcEstimatedState, FcEstimatorStatus


class FcHarness(Harness):
    def __init__(self):
        self.clock_origin = time.monotonic()
        self.remote_anchor = 100.
        self.clock_rate = 1.
        self.next_state = self.next_health = 0.
        self.state_sequence = 0
        super().__init__(state_source='fc_fused')
        self.streaming_gps = self.streaming_attitude = self.streaming_baro = False
        self.values['fc_state'] = []
        self.values['fc_health'] = []
        for key, cls, topic in [('fc_state', FcEstimatedState, 'estimated_state'),
                                ('fc_health', FcEstimatorStatus, 'estimator_status')]:
            self.subs.append(self.node.create_subscription(
                cls, self.namespace + '/sensors/fc/' + topic,
                lambda m, k=key: self.values[k].append(m), 1000))

    def remote(self):
        return int((self.remote_anchor + self.clock_rate * (time.monotonic() - self.clock_origin)) * 1e6)

    def set_rate(self, rate):
        now = time.monotonic()
        self.remote_anchor += self.clock_rate * (now - self.clock_origin)
        self.clock_origin, self.clock_rate = now, rate

    def send_state(self, stamp):
        covariance = [math.nan] * 21
        self.peer.odometry_send(stamp, mav.MAV_FRAME_LOCAL_NED, mav.MAV_FRAME_BODY_FRD,
                                1., 2., -1., [1., 0., 0., 0.], 0., 0., 0., 0., 0., 0.,
                                covariance, covariance, reset_counter=1,
                                estimator_type=mav.MAV_ESTIMATOR_TYPE_AUTOPILOT, quality=100)

    def imu(self, stamp=None):
        # Reuse the harness's transport loop, substituting the FC fused streams.
        now = time.monotonic()
        sample = self.remote() - int(.004 * self.clock_rate * 1e6)
        if now >= self.next_state:
            self.send_state(sample)
            self.state_sequence += 1
            self.next_state = now + .01
        if now >= self.next_health:
            payload = bytearray(249)
            struct.pack_into('<BBHIIQIIffIB3x6f', payload, 0,
                             1, 68, 0xc3, self.state_sequence, 1, sample,
                             30000, 30000, 1., 1., 3, 3, *([.01] * 6))
            self.peer.v2_extension_send(0, 0, 0, 0xBF01, payload)
            self.next_health = now + .02


class FcClockTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.h = FcHarness()
        self.addCleanup(self.h.close)
        self.h.drive(2.)
        self.assertTrue(self.h.values['fc_state'][-1].valid)

    def test_rate_changes_keep_state_and_health_usable(self):
        h = self.h
        state_start, health_start = len(h.values['fc_state']), len(h.values['fc_health'])
        for rate in (1.008, 1., .992, 1.):
            h.set_rate(rate)
            h.drive(3.)
        states = h.values['fc_state'][state_start:]
        health = h.values['fc_health'][health_start:]
        self.assertGreater(len(states), 800)
        self.assertGreater(len(health), 400)
        self.assertTrue(all(s.valid for s in states), {s.reason for s in states if not s.valid})
        self.assertTrue(all(s.valid for s in health), {s.reason for s in health if not s.valid})
        self.assertTrue(all(0 <= s.receive_steady_time - s.sample_steady_time <= .015 for s in states))
        self.assertTrue(all(a.sample_steady_time < b.sample_steady_time for a, b in zip(states, states[1:])))
        for s in states:
            wall_sample = s.header.stamp.sec + s.header.stamp.nanosec * 1e-9
            self.assertLess(abs((wall_sample - s.sample_steady_time) - (time.time() - time.monotonic())), .001)
        self.assertEqual(h.status()['fc_future_samples'], '0')
        self.assertEqual(h.status()['mapping_regressions'], '0')

    def test_invalid_samples_and_clock_restart_still_revoke(self):
        h = self.h
        session = h.values['fc_state'][-1].source_session
        h.streaming = False
        h.drive(.04)
        h.send_state(h.remote() - 20000)
        h.drive(.02)
        self.assertEqual(h.values['fc_state'][-1].reason, 'FC sample older than 15 ms')
        h.send_state(h.remote() + 30000)
        h.drive(.02)
        self.assertEqual(h.values['fc_state'][-1].reason, 'FC sample mapped to future')
        h.remote_anchor, h.clock_origin = 1., time.monotonic()
        h.streaming = True
        h.drive(2.)
        self.assertNotEqual(h.values['fc_state'][-1].source_session, session)
        self.assertTrue(h.values['fc_state'][-1].valid)
        self.assertTrue(h.values['fc_health'][-1].valid)

    def test_delayed_sync_and_expiration_do_not_refresh_mapping(self):
        h = self.h
        h.sync_delay = .04
        start = len(h.values['fc_state'])
        h.drive(2.5)
        self.assertGreater(int(h.status()['rejected_sync']), 0)
        self.assertTrue(any(not s.valid for s in h.values['fc_state'][start:]))
        self.assertFalse(h.values['fc_state'][-1].valid)
        h.sync_delay = 0.
        h.pending_sync.clear()
        h.drive(2.)
        self.assertTrue(h.values['fc_state'][-1].valid)

    def test_bounded_sync_jitter_preserves_sample_order(self):
        h = self.h
        start = len(h.values['fc_state'])
        for delay in (0., .002, 0., .001, .002, 0.):
            h.sync_delay = delay
            h.drive(.6)
        states = h.values['fc_state'][start:]
        self.assertTrue(all(s.valid for s in states), {s.reason for s in states if not s.valid})
        self.assertTrue(all(a.sample_steady_time < b.sample_steady_time for a, b in zip(states, states[1:])))
        self.assertTrue(all(s.sample_steady_time <= s.receive_steady_time for s in states))


if __name__ == '__main__':
    unittest.main()
