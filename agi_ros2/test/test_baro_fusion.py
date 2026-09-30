#!/usr/bin/env python3
"""Real fusion process with synthetic timestamped IMU, navigation and pressure.

Uses an isolated namespace and simulated clock; no physical serial device is opened.
Run after building and sourcing ROS and install/agi_ros2/local_setup.bash.
"""
import math
import time
import unittest

import rclpy
from agi_ros2.msg import Barometer, FusedState, LocalNavigation, Rtk
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Imu

from test_node_pipeline import Harness


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class BarometerHarness(Harness):
    def __init__(self, navigation_source='rtk', parameters=None):
        super().__init__()
        self.simulation = True
        self.armed = False
        self.health_enabled = False
        self.pressure_enabled = True
        self.pressure = 101325.0
        self.source_session = 'synthetic-fc-epoch-1'
        self.navigation_source = navigation_source
        self.navigation_position = (0.0, 0.0, 3.0)
        self.navigation_velocity = (0.0, 0.0, 0.0)
        self.navigation_heading = 0.0
        self.navigation_heading_rate = 0.0
        self.navigation_epoch = self.sim_time_ns * 1e-9
        self.navigation_delay = 0.0
        self.pending_navigation = []
        self.imu_delay = 0.0
        self.imu_yaw_rate = 0.0
        self.pending_imus = []
        self.pressure_stamp_offset = 0.0
        self.last_sample = 0.0
        self.subscribe('fused_state', FusedState)
        self.subscribe('fusion/baro/status', DiagnosticArray)
        configuration = {
            'navigation_source': navigation_source, 'baro_enabled': True, 'observation_delay': 0.2,
            'baro_reference_duration': 0.2, 'baro_reference_min_samples': 5,
            'baro_reference_max_vertical_stddev': 0.3, 'baro_pressure_variance_pa2': 4.0}
        if navigation_source == 'gnss':
            configuration.update(gnss_use_baro_height=True, baro_bias_random_walk=0.0,
                                 imu_initialization_duration=0.2, imu_initialization_samples=30)
        configuration.update(parameters or {})
        self.start('state_fusion_node', parameters=configuration)

    def set_navigation(self, position, velocity=(0.0, 0.0, 0.0), heading=0.0, heading_rate=0.0):
        self.navigation_position = position
        self.navigation_velocity = velocity
        self.navigation_heading = heading
        self.navigation_heading_rate = heading_rate
        self.navigation_epoch = self.sim_time_ns * 1e-9

    def navigation(self, stamp=None):
        message = LocalNavigation() if self.navigation_source == 'gnss' else Rtk()
        message.header.stamp = stamp or self.stamp()
        message.header.frame_id = 'odom'
        elapsed = stamp_seconds(message.header.stamp) - self.navigation_epoch
        for index, axis in enumerate(('x', 'y', 'z')):
            setattr(message.position, axis, self.navigation_position[index] + elapsed * self.navigation_velocity[index])
            setattr(message.velocity, axis, self.navigation_velocity[index])
        message.heading = self.navigation_heading + elapsed * self.navigation_heading_rate
        message.heading_valid = message.accuracy_ok = True
        if self.navigation_source == 'gnss':
            message.observation_valid = message.clock_aligned = message.accuracy_known = True
            message.session_id = 'synthetic-local-origin-1'
            message.source_session = self.source_session
            message.altitude_reference = 'msl'
            message.fix_type = 3
            message.position_variance = [0.04, 0.04, 4.0]
            message.velocity_variance = [0.01, 0.01, 0.04]
            message.heading_variance = 0.01
            message.horizontal_accuracy = 0.2
            message.vertical_accuracy = 2.0
            message.velocity_accuracy = 0.2
        else:
            message.fixed = message.synchronized = True
        return message

    def barometer(self, valid=True):
        message = Barometer()
        sample_ns = self.sim_time_ns + round(self.pressure_stamp_offset * 1e9)
        message.header.stamp = Time(sec=sample_ns // 1_000_000_000, nanosec=sample_ns % 1_000_000_000)
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
            navigation = self.navigation()
            self.pending_navigation.append((seconds + self.navigation_delay, navigation))
            self.last_sample = seconds
        while self.pending_navigation and self.pending_navigation[0][0] <= seconds:
            _, navigation = self.pending_navigation.pop(0)
            if self.navigation_source == 'gnss':
                self.publisher('sensors/local_navigation', LocalNavigation, True).publish(navigation)
            else:
                self.publisher('sensors/rtk', Rtk).publish(navigation)
        if seconds - self.last_imu >= 0.002:
            imu = Imu()
            imu.header.stamp = self.stamp()
            imu.header.frame_id = 'base_link'
            imu.linear_acceleration.z = 9.80665
            imu.angular_velocity.z = self.imu_yaw_rate
            self.pending_imus.append((seconds + self.imu_delay, imu))
            self.last_imu = seconds
        while self.pending_imus and self.pending_imus[0][0] <= seconds:
            _, imu = self.pending_imus.pop(0)
            self.publisher('sensors/imu', Imu, True).publish(imu)

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
        self.assertEqual(self.h.status()['gnss_baro_height_active'], 0)
        self.assertEqual(self.h.status()['navigation_nis_dimensions'], 7)

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

    def test_current_pressure_waits_for_backlogged_imu_without_resetting_reference(self):
        h = self.h
        before = h.status()
        reset_counter = h.state().reset_counter
        # The pressure stamp is current in ROS time but over 10 ms ahead of
        # the IMU being processed, reproducing the stationary hardware bag.
        h.imu_delay = 0.03
        h.drive(1.5)
        after = h.status()
        self.assertEqual(after['reference_resets'], before['reference_resets'], after)
        self.assertEqual(after['reference_valid'], 1, after)
        self.assertEqual(after['reference_pressure_pa'], before['reference_pressure_pa'])
        self.assertGreater(after['accepted_updates'], before['accepted_updates'] + 10)
        self.assertEqual(h.state().reset_counter, reset_counter)
        self.assertGreater(h.sim_time_ns * 1e-9 - stamp_seconds(h.state().header.stamp), 0.02)
        self.assert_height_preserved()

    def test_abnormal_pressure_timestamps_do_not_keep_reference_fresh(self):
        h = self.h
        for offset in (0.05, -0.30):
            with self.subTest(pressure_stamp_offset=offset):
                before = h.status()
                h.pressure_stamp_offset = offset
                h.drive(0.7)
                after = h.status()
                self.assertGreater(after['invalid_samples'], before['invalid_samples'])
                self.assertEqual(after['reference_valid'], 0, after)
                self.assertGreater(after['reference_resets'], before['reference_resets'])
                self.assertIn('stale', after['message'])
                self.assert_height_preserved()
                h.pressure_stamp_offset = 0.0
                h.drive(1.3)
                self.assertEqual(h.status()['reference_valid'], 1, h.status())

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


class GnssBarometerFusionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.h = BarometerHarness(navigation_source='gnss', parameters={
            'navigation_ready_updates': 3, 'max_horizontal_position_stddev': 3.0,
            'max_vertical_position_stddev': 5.0, 'max_velocity_stddev': 1.0,
            'max_heading_stddev': 1.0})
        self.addCleanup(self.h.close)
        self.wait_for_policy(active=1, dimensions=5)
        self.h.drive(0.4)
        self.assertTrue(self.h.state().initialized)
        self.assertTrue(self.h.state().estimator_ready, self.h.state().readiness_reason)
        self.assertEqual(self.h.status()['baro_bias_random_walk_m2_s'], 0.0)

    def wait_for_policy(self, active, dimensions):
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            self.h.drive(0.2)
            status = self.h.status()
            if (status.get('gnss_baro_height_active') == active and
                    status.get('navigation_nis_dimensions') == dimensions):
                return status
        self.fail(f'Height policy did not become active={active}, dimensions={dimensions}: {self.h.status()}')

    def test_gnss_vertical_drift_is_excluded_while_horizontal_motion_and_heading_fuse(self):
        h = self.h
        before = h.status()
        # An already initialized reference must work while armed, at a nonzero
        # local height, and with horizontal motion; no stationary clamp is used.
        h.armed = True
        h.set_navigation((0.0, 0.0, 3.0), velocity=(0.15, -0.10, 0.60), heading_rate=0.08)
        h.imu_yaw_rate = 0.08
        h.drive(4.5)
        state = h.state()
        expected = h.navigation(state.header.stamp)
        status = h.status()
        self.assertGreater(expected.position.z, 4.0)
        self.assertAlmostEqual(state.position.z, 3.0, delta=0.10)
        self.assertAlmostEqual(state.velocity.z, 0.0, delta=0.10)
        self.assertAlmostEqual(state.position.x, expected.position.x, delta=0.10)
        self.assertAlmostEqual(state.position.y, expected.position.y, delta=0.10)
        self.assertGreater(state.position.x, 0.20)
        self.assertLess(state.position.y, -0.12)
        q = state.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.assertAlmostEqual(yaw, expected.heading, delta=0.04)
        self.assertGreater(yaw, 0.12)
        self.assertEqual(status['gnss_baro_height_active'], 1)
        self.assertEqual(status['navigation_nis_dimensions'], 5)
        self.assertEqual(status['reference_resets'], before['reference_resets'])
        self.assertGreater(status['accepted_updates'], before['accepted_updates'] + 20)

    def assert_gnss_height_fallback_and_pressure_recovery(self, require_disarmed_reference=False):
        h = self.h
        self.wait_for_policy(active=0, dimensions=7)
        accepted_before = h.status()['accepted_updates']
        navigation_before = stamp_seconds(h.state().rtk_stamp)
        height_before = h.state().position.z
        h.set_navigation((0.0, 0.0, 3.4))
        h.drive(3.0)
        # GNSS height has a 2 m standard deviation: require a clear response
        # to its 0.4 m step, not complete convergence within 1.5 simulated seconds.
        self.assertGreater(h.state().position.z, height_before + 0.15, h.state())
        self.assertGreater(stamp_seconds(h.state().rtk_stamp), navigation_before + 0.5)
        self.assertEqual(h.status()['gnss_baro_height_active'], 0)
        self.assertEqual(h.status()['navigation_nis_dimensions'], 7)
        h.pressure = 101325.0
        h.pressure_enabled = True
        if require_disarmed_reference:
            h.drive(1.5)
            self.assertEqual(h.status()['reference_valid'], 0, h.status())
            self.assertEqual(h.status()['gnss_baro_height_active'], 0)
            self.assertEqual(h.status()['navigation_nis_dimensions'], 7)
            self.assertIn('disarmed', h.status()['message'])
            h.armed = False
        self.wait_for_policy(active=1, dimensions=5)
        self.assertGreater(h.status()['accepted_updates'], accepted_before)
        self.assertEqual(h.status()['reference_valid'], 1)
        self.assertTrue(h.state().initialized)

    def test_pressure_loss_restores_full_gnss_until_pressure_reference_recovers(self):
        h = self.h
        resets = h.status()['reference_resets']
        h.armed = True
        h.pressure_enabled = False
        self.assert_gnss_height_fallback_and_pressure_recovery(require_disarmed_reference=True)
        self.assertGreater(h.status()['reference_resets'], resets)

    def test_continuous_rejected_pressure_does_not_suppress_gnss_height(self):
        h = self.h
        before = h.status()
        h.pressure = 80000.0
        self.wait_for_policy(active=0, dimensions=7)
        failed = h.status()
        self.assertGreater(failed['rejected_updates'], before['rejected_updates'])
        self.assertGreater(failed['nis'], 10.828)
        self.assertEqual(failed['reference_valid'], 1)
        self.assertEqual(failed['reference_resets'], before['reference_resets'])
        self.assert_gnss_height_fallback_and_pressure_recovery()

    def test_large_gnss_vertical_error_preserves_xy_updates_but_revokes_readiness(self):
        h = self.h
        h.armed = True
        h.set_navigation((0.0, 0.0, 100.0), velocity=(0.15, -0.10, 0.0))
        h.drive(0.7)
        self.assertEqual(h.status()['gnss_baro_height_active'], 1)
        self.assertTrue(h.state().estimator_ready, h.state().readiness_reason)
        accepted_stamp = stamp_seconds(h.state().rtk_stamp)
        rejections = h.state().navigation_rejections
        h.pressure_enabled = False
        h.drive(1.5)
        state = h.state()
        status = h.status()
        expected = h.navigation(state.header.stamp)
        self.assertEqual(status['gnss_baro_height_active'], 0)
        self.assertEqual(status['gnss_vertical_recovery_pending'], 1)
        self.assertEqual(status['navigation_nis_dimensions'], 5)
        self.assertGreater(state.navigation_rejections, rejections)
        self.assertGreater(stamp_seconds(state.rtk_stamp), accepted_stamp + 0.4)
        self.assertAlmostEqual(state.position.x, expected.position.x, delta=0.10)
        self.assertAlmostEqual(state.position.y, expected.position.y, delta=0.10)
        self.assertGreater(state.position.x, 0.10)
        self.assertLess(state.position.z, 4.0)
        self.assertTrue(state.navigation_valid)
        self.assertFalse(state.navigation_ready)
        self.assertFalse(state.estimator_ready)
        self.assertEqual(state.navigation_accepted_updates, 0)
        self.assertIn('vertical recovery', state.readiness_reason)


if __name__ == '__main__':
    unittest.main()
