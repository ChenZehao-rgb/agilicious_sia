#!/usr/bin/env python3
"""Real fusion process with synthetic timestamped IMU, navigation and pressure.

Uses an isolated namespace and simulated clock; no physical serial device is opened.
Run after building and sourcing ROS and install/agi_ros2/local_setup.bash.
"""
import math
import time
import unittest

import rclpy
from agi_ros2.msg import Barometer, FusedState, Rtk
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Imu

from test_node_pipeline import Harness


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class BarometerHarness(Harness):
    def __init__(self):
        super().__init__()
        self.simulation = True
        self.armed = False
        self.health_enabled = False
        self.pressure_enabled = True
        self.pressure = 101325.0
        self.source_session = 'synthetic-fc-epoch-1'
        self.navigation_delay = 0.0
        self.pending_navigation = []
        self.last_sample = 0.0
        self.subscribe('fused_state', FusedState)
        self.subscribe('fusion/baro/status', DiagnosticArray)
        self.start('state_fusion_node', parameters={
            'navigation_source': 'rtk', 'baro_enabled': True, 'observation_delay': 0.2,
            'baro_reference_duration': 0.2, 'baro_reference_min_samples': 5,
            'baro_reference_max_vertical_stddev': 0.3, 'baro_pressure_variance_pa2': 4.0})

    def navigation(self, stamp=None):
        message = Rtk()
        message.header.stamp = stamp or self.stamp()
        message.header.frame_id = 'odom'
        message.position.z = 3.0
        message.fixed = message.heading_valid = message.accuracy_ok = message.synchronized = True
        return message

    def barometer(self, valid=True):
        message = Barometer()
        message.header.stamp = self.stamp()
        message.header.frame_id = 'baro_link'
        message.source_session = self.source_session
        message.clock_aligned = valid
        message.valid = valid
        message.pressure_pa = self.pressure if valid else math.nan
        message.pressure_variance = 0.0  # Exercise the positive configured fallback.
        message.reason = 'synthetic pressure' if valid else 'Synthetic clock epoch invalidation'
        return message

    def publish_inputs(self, sensors, commands):
        super().publish_inputs(False, False)
        seconds = self.sim_time_ns * 1e-9
        if seconds - self.last_sample >= 0.05:
            # Identical sample stamps, but pressure is deliberately delivered first.
            sample = self.barometer()
            if self.pressure_enabled:
                self.publisher('sensors/baro/sample', Barometer).publish(sample)
            navigation = self.navigation(sample.header.stamp)
            self.pending_navigation.append((seconds + self.navigation_delay, navigation))
            self.last_sample = seconds
        while self.pending_navigation and self.pending_navigation[0][0] <= seconds:
            _, navigation = self.pending_navigation.pop(0)
            self.publisher('sensors/rtk', Rtk).publish(navigation)
        if seconds - self.last_imu >= 0.002:
            imu = Imu()
            imu.header.stamp = self.stamp()
            imu.header.frame_id = 'base_link'
            imu.linear_acceleration.z = 9.80665
            self.publisher('sensors/imu', Imu, True).publish(imu)
            self.last_imu = seconds

    def drive(self, seconds):
        self.run(seconds, rate=0.5)

    def status(self):
        messages = self.received['fusion/baro/status']
        if not messages:
            return {}
        status = messages[-1].status[0]
        return dict(message=status.message, **{item.key: float(item.value) for item in status.values})

    def state(self):
        return self.received['fused_state'][-1]


class BarometerFusionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.h = BarometerHarness()
        self.addCleanup(self.h.close)
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            self.h.drive(0.2)
            if self.h.status().get('accepted_updates', 0) >= 3:
                break
        self.assertGreaterEqual(self.h.status().get('accepted_updates', 0), 3,
                                (self.h.status(), str(self.h.state()) if self.h.received['fused_state'] else 'no state'))
        self.assertTrue(self.h.state().initialized)

    def assert_height_preserved(self):
        self.assertTrue(self.h.state().initialized)
        self.assertAlmostEqual(self.h.state().position.z, 3.0, delta=0.05)
        self.assertTrue(all(math.isfinite(value) and value >= 0 for value in self.h.state().position_variance))

    def test_nonzero_reference_direction_variance_and_outlier_nis(self):
        h = self.h
        self.assertAlmostEqual(h.status()['reference_height_m'], 3.0, delta=0.01)
        self.assertGreaterEqual(h.status()['reference_bias_variance_m2'], 4.0)
        h.pressure = 101324.0
        h.drive(0.9)
        status = h.status()
        self.assertGreater(status['height_m'], 3.07)
        self.assertLess(status['height_m'], 3.10)
        self.assertGreater(status['height_variance_m2'], 0.25)
        rejected = status['rejected_updates']
        h.pressure = 80000.0
        h.drive(0.9)
        self.assertGreater(h.status()['rejected_updates'], rejected)
        self.assertGreater(h.status()['nis'], 10.828)
        self.assert_height_preserved()

    def test_loss_and_source_epoch_invalidation_preserve_navigation_height(self):
        h = self.h
        reset_counter = h.state().reset_counter
        h.pressure_enabled = False
        h.drive(0.5)
        self.assertEqual(h.status()['reference_valid'], 0)
        self.assertIn('stale', h.status()['message'])
        self.assertEqual(h.state().reset_counter, reset_counter)
        self.assert_height_preserved()
        h.source_session = 'synthetic-fc-epoch-2'
        h.publisher('sensors/baro/sample', Barometer).publish(h.barometer(False))
        h.drive(0.25)
        self.assertEqual(h.status()['reference_valid'], 0)
        self.assertIn('invalidation', h.status()['message'])
        self.assert_height_preserved()
        h.pressure = 100000.0
        h.pressure_enabled = True
        h.drive(1.3)
        self.assertEqual(h.status()['reference_valid'], 1)
        self.assertAlmostEqual(h.status()['reference_pressure_pa'], 100000.0)
        self.assertAlmostEqual(h.status()['reference_height_m'], 3.0, delta=0.01)
        self.assert_height_preserved()

    def test_later_navigation_arrival_and_same_stamp_remain_fusable(self):
        h = self.h
        late_before = h.status()['late_navigation']
        rejected_before = h.state().navigation_rejections
        accepted_before = h.status()['accepted_updates']
        navigation_before = stamp_seconds(h.state().rtk_stamp)
        h.navigation_delay = 0.08
        h.drive(1.5)
        self.assertEqual(h.status()['late_navigation'], late_before)
        self.assertEqual(h.state().navigation_rejections, rejected_before)
        self.assertGreater(h.status()['accepted_updates'], accepted_before)
        self.assertGreater(stamp_seconds(h.state().rtk_stamp), navigation_before + 0.4)
        self.assertTrue(h.state().navigation_valid)
        self.assert_height_preserved()

    def test_observations_older_than_committed_window_are_dropped(self):
        h = self.h
        late_navigation = h.status()['late_navigation']
        late_barometer = h.status()['late_barometer']
        old_ns = h.sim_time_ns - 240_000_000
        old_stamp = Time(sec=old_ns // 1_000_000_000, nanosec=old_ns % 1_000_000_000)
        navigation = h.navigation(old_stamp)
        navigation.position.z = 40.0
        h.publisher('sensors/rtk', Rtk).publish(navigation)
        pressure = h.barometer()
        pressure.header.stamp = old_stamp
        h.publisher('sensors/baro/sample', Barometer).publish(pressure)
        h.drive(0.3)
        self.assertGreater(h.status()['late_navigation'], late_navigation)
        self.assertGreater(h.status()['late_barometer'], late_barometer)
        self.assert_height_preserved()


if __name__ == '__main__':
    unittest.main()
