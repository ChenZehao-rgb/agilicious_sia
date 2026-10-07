#!/usr/bin/env python3
"""Real fusion process with synthetic timestamped IMU, navigation and pressure.

Uses an isolated namespace and simulated clock; no physical serial device is opened.
Run after building and sourcing ROS and install/agi_ros2/local_setup.bash.
"""
import math
import time
import unittest

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from agi_ros2.msg import Authority, Barometer, FusedState, LocalNavigation, Rtk
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu

from test_node_pipeline import Harness


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def diagnostic_values(status):
    values = dict(message=status.message)
    for item in status.values:
        try:
            values[item.key] = float(item.value)
        except ValueError:
            values[item.key] = item.value
    return values


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
        self.navigation_position_rate = None
        self.navigation_position_variance = [0.04, 0.04, 4.0]
        self.navigation_velocity_variance = [0.01, 0.01, 0.04]
        self.navigation_valid = True
        self.navigation_heading = 0.0
        self.navigation_heading_rate = 0.0
        self.navigation_epoch = self.sim_time_ns * 1e-9
        self.navigation_delay = 0.0
        self.navigation_period = 0.05
        self.navigation_enabled = True
        self.pending_navigation = []
        self.imu_delay = 0.0
        self.imu_yaw_rate = 0.0
        self.pending_imus = []
        self.pressure_stamp_offset = 0.0
        self.last_sample = 0.0
        self._discovery_ready = False
        self._last_clock_publish_time = None
        self.subscribe('fused_state', FusedState)
        self.subscribe('fusion/baro/status', DiagnosticArray)
        # Match the production authority publishers and rclcpp ClockQoS: keep
        # only current state instead of replaying stale reliable history.
        self.pubs['authority'] = self.node.create_publisher(Authority, 'authority', 1)
        self.pubs['clock'] = self.node.create_publisher(
            Clock, 'clock', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._input_publishers = [self.publisher('clock', Clock), self.publisher('authority', Authority),
                                  self.publisher('sensors/imu', Imu, True), self.publisher('sensors/baro/sample', Barometer)]
        self._input_publishers.append(self.publisher('sensors/local_navigation', LocalNavigation, True)
                                      if navigation_source == 'gnss' else self.publisher('sensors/rtk', Rtk))
        configuration = {
            'navigation_source': navigation_source, 'baro_enabled': True, 'observation_delay': 0.2,
            'baro_reference_duration': 0.2, 'baro_reference_min_samples': 5,
            'baro_reference_max_vertical_stddev': 0.3, 'baro_pressure_variance_pa2': 4.0}
        if navigation_source == 'gnss':
            configuration.update(gnss_use_baro_height=True, baro_bias_random_walk=0.0,
                                 imu_initialization_duration=0.2, imu_initialization_samples=30)
        configuration.update(parameters or {})
        if not configuration['baro_enabled']:
            self._input_publishers.remove(self.pubs['sensors/baro/sample'])
        self.start('state_fusion_node', parameters=configuration)

    def set_navigation(self, position, velocity=(0.0, 0.0, 0.0), heading=0.0, heading_rate=0.0, position_rate=None):
        self.navigation_position = position
        self.navigation_velocity = velocity
        self.navigation_position_rate = position_rate
        self.navigation_heading = heading
        self.navigation_heading_rate = heading_rate
        self.navigation_epoch = self.sim_time_ns * 1e-9

    def navigation(self, stamp=None):
        message = LocalNavigation() if self.navigation_source == 'gnss' else Rtk()
        message.header.stamp = stamp or self.stamp()
        message.header.frame_id = 'odom'
        elapsed = stamp_seconds(message.header.stamp) - self.navigation_epoch
        position_rate = self.navigation_position_rate or self.navigation_velocity
        for index, axis in enumerate(('x', 'y', 'z')):
            setattr(message.position, axis, self.navigation_position[index] + elapsed * position_rate[index])
            setattr(message.velocity, axis, self.navigation_velocity[index])
        message.heading = self.navigation_heading + elapsed * self.navigation_heading_rate
        message.heading_valid = message.accuracy_ok = True
        if self.navigation_source == 'gnss':
            message.observation_valid = self.navigation_valid
            message.clock_aligned = message.accuracy_known = True
            message.session_id = 'synthetic-local-origin-1'
            message.source_session = self.source_session
            message.altitude_reference = 'msl'
            message.fix_type = 3
            message.position_variance = self.navigation_position_variance
            message.velocity_variance = self.navigation_velocity_variance
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
        if seconds - self.last_sample >= self.navigation_period:
            # Identical sample stamps, but pressure is deliberately delivered first.
            sample = self.barometer()
            if self.pressure_enabled:
                self.publisher('sensors/baro/sample', Barometer).publish(sample)
            navigation = self.navigation()
            if self.navigation_enabled:
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
        if not self._discovery_ready:
            deadline = time.monotonic() + 5.0
            while not all(publisher.get_subscription_count() for publisher in self._input_publishers):
                self._check_processes()
                if time.monotonic() >= deadline:
                    missing = [publisher.topic_name for publisher in self._input_publishers
                               if not publisher.get_subscription_count()]
                    raise AssertionError(f'Fusion input DDS discovery incomplete: {missing}')
                rclpy.spin_once(self.node, timeout_sec=0.01)
            self._discovery_ready = True
        start = time.monotonic()
        start_sim = self.sim_time_ns
        while time.monotonic() - start < seconds:
            self.sim_time_ns = start_sim + int((time.monotonic() - start) * 0.5 * 1000) * 1_000_000
            if self.sim_time_ns != self._last_clock_publish_time:
                # Publish each simulated millisecond once. Repeating a clock
                # sample for every ready callback needlessly floods DDS queues.
                self.publisher('clock', Clock).publish(Clock(clock=self.stamp()))
                self.publish_inputs(False, False)
                self._last_clock_publish_time = self.sim_time_ns
            rclpy.spin_once(self.node, timeout_sec=0.0005)
            self.drain()
            self._check_processes()

    def _check_processes(self):
        for process, log in zip(self.processes, self.logs):
            if process.poll() is not None:
                log.seek(0)
                raise AssertionError(log.read())

    def status(self):
        messages = self.received['fusion/baro/status']
        if not messages:
            return {}
        status = messages[-1].status[0]
        return diagnostic_values(status)

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


class WeightedGnssBarometerFusionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.h = BarometerHarness(navigation_source='gnss', parameters={
            'height_fusion_mode': 'baro_gnss_weighted', 'navigation_ready_updates': 3,
            'max_horizontal_position_stddev': 3.0, 'max_vertical_position_stddev': 5.0,
            'max_velocity_stddev': 1.0, 'max_heading_stddev': 1.0})
        self.addCleanup(self.h.close)
        self.wait_for_status(gnss_baro_height_active=1)
        self.h.drive(0.4)
        self.assertTrue(self.h.state().estimator_ready, self.h.state().readiness_reason)

    def wait_for_status(self, **expected):
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            self.h.drive(0.2)
            status = self.h.status()
            if all(status.get(key) == value for key, value in expected.items()):
                return status
        self.fail(f'Diagnostics did not reach {expected}: {self.h.status()}')

    def test_independent_gates_keep_valid_groups_and_revoke_readiness_on_horizontal_failure(self):
        h = self.h
        h.armed = True
        before = h.status()
        h.set_navigation((0.0, 0.0, 100.0))
        status = self.wait_for_status(gnss_height_accepted=0, gnss_vertical_velocity_accepted=1,
                                      gnss_horizontal_accepted=1)
        self.assertEqual(status['gnss_height_reason'], 'innovation')
        self.assertEqual(status['gnss_vertical_degraded'], 1)
        self.assertGreater(status['gnss_height_nis'], 10.828)
        self.assertGreater(status['gnss_vertical_velocity_accepted_updates'], before['gnss_vertical_velocity_accepted_updates'])
        self.assertTrue(h.state().estimator_ready, h.state().readiness_reason)
        self.assertLess(h.state().position.z, 3.2)

        h.set_navigation((0.0, 0.0, 3.0), velocity=(0.0, 0.0, 8.0), position_rate=(0.0, 0.0, 0.0))
        status = self.wait_for_status(gnss_height_accepted=1, gnss_vertical_velocity_accepted=0,
                                      gnss_horizontal_accepted=1)
        self.assertEqual(status['gnss_vertical_velocity_reason'], 'innovation')
        self.assertTrue(h.state().estimator_ready, h.state().readiness_reason)
        self.assertAlmostEqual(h.state().velocity.z, 0.0, delta=0.1)

        h.set_navigation((100.0, 0.0, 3.0))
        status = self.wait_for_status(gnss_horizontal_accepted=0, gnss_height_accepted=1,
                                      gnss_vertical_velocity_accepted=1)
        accepted_stamp = stamp_seconds(h.state().rtk_stamp)
        h.drive(0.4)
        self.assertEqual(stamp_seconds(h.state().rtk_stamp), accepted_stamp)
        self.assertGreater(h.status()['gnss_height_last_accepted_stamp'], accepted_stamp)
        self.assertFalse(h.state().navigation_ready)
        self.assertFalse(h.state().estimator_ready)
        self.assertEqual(h.state().navigation_accepted_updates, 0)

    def test_position_drift_does_not_replace_doppler_velocity_and_weights_follow_reported_variance(self):
        h = self.h
        h.armed = True
        self.assertEqual(h.status()['gnss_effective_height_variance_m2'], 18.0)
        self.assertEqual(h.status()['gnss_effective_vertical_velocity_variance_m2_s2'], 0.09)
        h.navigation_position_variance[2] = 8.0
        h.navigation_velocity_variance[2] = 0.10
        h.set_navigation((0.0, 0.0, 3.0), position_rate=(0.0, 0.0, 0.40))
        h.drive(5.0)
        status = h.status()
        state = h.state()
        self.assertGreater(h.navigation(state.header.stamp).position.z, 3.8)
        self.assertAlmostEqual(state.position.z, 3.0, delta=0.20)
        self.assertAlmostEqual(state.velocity.z, 0.0, delta=0.10)
        self.assertEqual(status['gnss_raw_height_variance_m2'], 8.0)
        self.assertEqual(status['gnss_effective_height_variance_m2'], 36.0)
        self.assertEqual(status['gnss_effective_vertical_velocity_variance_m2_s2'], 0.225)
        self.assertEqual(status['gnss_height_observation_stamp'], status['gnss_vertical_velocity_observation_stamp'])
        self.assertEqual(status['gnss_horizontal_observation_stamp'], status['gnss_height_observation_stamp'])
        self.assertGreater(status['absolute_height_variance_m2'], status['relative_height_variance_m2'])

    def test_pressure_loss_and_rejected_pressure_recover_same_reference_while_armed(self):
        h = self.h
        h.armed = True
        reference = h.status()
        height = h.state().position.z
        h.pressure_enabled = False
        status = self.wait_for_status(gnss_baro_height_active=0, gnss_effective_height_variance_m2=4.0)
        self.assertEqual(status['reference_valid'], 1)
        self.assertEqual(status['relative_reference_active'], 1)
        self.assertEqual(status['reference_resets'], reference['reference_resets'])
        h.drive(0.5)
        self.assertTrue(h.state().estimator_ready, h.state().readiness_reason)
        self.assertAlmostEqual(h.state().position.z, height, delta=0.1)
        h.pressure_enabled = True
        status = self.wait_for_status(gnss_baro_height_active=1)
        self.assertEqual(status['reference_pressure_pa'], reference['reference_pressure_pa'])
        self.assertEqual(status['reference_resets'], reference['reference_resets'])
        h.pressure = 80000.0
        status = self.wait_for_status(gnss_baro_height_active=0)
        self.assertGreater(status['rejected_updates'], reference['rejected_updates'])
        self.assertEqual(status['reference_valid'], 1)
        self.assertEqual(status['gnss_effective_height_variance_m2'], 4.0)
        h.pressure = 101325.0
        status = self.wait_for_status(gnss_baro_height_active=1)
        self.assertEqual(status['reference_resets'], reference['reference_resets'])

    def test_missing_pressure_requires_both_vertical_groups_before_rebuilding_ready_count(self):
        h = self.h
        h.armed = True
        h.set_navigation((0.0, 0.0, 100.0))
        self.wait_for_status(gnss_height_accepted=0)
        h.pressure_enabled = False
        status = self.wait_for_status(gnss_baro_height_active=0, gnss_vertical_recovery_pending=1)
        self.assertTrue(h.state().navigation_valid)
        self.assertFalse(h.state().navigation_ready)
        self.assertFalse(h.state().estimator_ready)
        self.assertEqual(h.state().navigation_accepted_updates, 0)
        # Diagnostics and FusedState publish independently. Compare groups from
        # one diagnostic snapshot instead of a newer FusedState acquisition time.
        self.assertEqual(status['gnss_vertical_velocity_last_accepted_stamp'], status['gnss_horizontal_last_accepted_stamp'])
        self.assertLessEqual(status['gnss_vertical_velocity_accepted_age_s'], 0.45)
        h.set_navigation((0.0, 0.0, 3.0), velocity=(0.0, 0.0, 8.0), position_rate=(0.0, 0.0, 0.0))
        self.wait_for_status(gnss_height_accepted=1, gnss_vertical_velocity_accepted=0,
                             gnss_vertical_recovery_pending=1)
        self.assertFalse(h.state().navigation_ready)
        self.assertFalse(h.state().estimator_ready)
        self.assertEqual(h.state().navigation_accepted_updates, 0)
        state_index = len(h.received['fused_state'])
        h.set_navigation((0.0, 0.0, 3.0))
        self.wait_for_status(gnss_vertical_recovery_pending=0)
        h.drive(0.5)
        self.assertTrue(h.state().estimator_ready, h.state().readiness_reason)
        self.assertEqual(h.state().navigation_accepted_updates, 3)
        counts_by_epoch = {}
        for state in h.received['fused_state'][state_index:]:
            if state.navigation_accepted_updates > 0:
                counts_by_epoch.setdefault(stamp_seconds(state.rtk_stamp), state.navigation_accepted_updates)
        counts = list(counts_by_epoch.values())
        self.assertGreaterEqual(len(counts), 3)
        self.assertEqual(counts[:3], [1, 2, 3])

    def test_explicit_pressure_and_navigation_invalidation_clear_reference(self):
        h = self.h
        h.armed = True
        h.publisher('sensors/baro/sample', Barometer).publish(h.barometer(False))
        status = self.wait_for_status(reference_valid=0, ekf_reference_valid=0)
        self.assertEqual(status['relative_reference_active'], 1)
        h.drive(0.4)
        self.assertEqual(h.status()['reference_valid'], 0)
        h.armed = False
        self.wait_for_status(gnss_baro_height_active=1)
        h.armed = True
        h.navigation_valid = False
        self.wait_for_status(reference_valid=0, ekf_reference_valid=0)
        self.assertFalse(h.state().navigation_ready)
        h.navigation_valid = True
        h.drive(0.6)
        self.assertEqual(h.status()['reference_valid'], 0)

    def test_weighting_and_mode_parameters_cannot_change_at_runtime(self):
        h = self.h
        client = h.node.create_client(SetParameters, 'state_fusion/set_parameters')
        self.addCleanup(h.node.destroy_client, client)
        self.assertTrue(client.wait_for_service(timeout_sec=2.0))
        parameters = [Parameter(name='height_fusion_mode', value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value='legacy_full3d'))]
        for name in ('baro_bias_random_walk', 'gnss_height_variance_floor', 'gnss_height_variance_scale',
                     'gnss_vertical_velocity_variance_floor', 'gnss_vertical_velocity_variance_scale',
                     'navigation_height_nis_threshold', 'navigation_vertical_velocity_nis_threshold'):
            parameters.append(Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=1.0)))
        future = client.call_async(SetParameters.Request(parameters=parameters))
        h.drive(0.3)
        self.assertTrue(future.done())
        self.assertTrue(all(not result.successful for result in future.result().results))
        self.assertEqual(h.status()['height_fusion_mode'], 'baro_gnss_weighted')


    def test_source_session_and_clock_epoch_reset_reference(self):
        h = self.h
        before = h.state().reset_counter
        h.armed = True
        h.source_session = 'synthetic-fc-epoch-2'
        self.wait_for_status(reference_valid=0, ekf_reference_valid=0)
        self.assertGreater(h.state().reset_counter, before)
        self.assertFalse(h.state().estimator_ready)
        h.armed = False
        self.wait_for_status(gnss_baro_height_active=1)
        before = h.state().reset_counter
        h.armed = True
        h.sim_time_ns -= 2_000_000_000
        h.last_sample = h.last_imu = 0.0
        h.pending_navigation.clear()
        h.pending_imus.clear()
        self.wait_for_status(reference_valid=0, ekf_reference_valid=0)
        self.assertGreater(h.state().reset_counter, before)
        self.assertFalse(h.state().estimator_ready)


class NavigationTimingFusionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_reorder_delay_preserves_readiness_but_raw_outage_revokes(self):
        h = BarometerHarness(navigation_source='gnss', parameters={
            'height_fusion_mode': 'baro_gnss_weighted', 'navigation_ready_updates': 3,
            'max_horizontal_position_stddev': 2.0, 'max_vertical_position_stddev': 3.0,
            'max_velocity_stddev': 0.6, 'max_heading_stddev': 0.5})
        self.addCleanup(h.close)
        h.navigation_period = 0.20
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and (not h.received['fused_state'] or not h.state().estimator_ready):
            h.drive(0.2)
        self.assertTrue(h.state().estimator_ready, h.state().readiness_reason)
        before = len(h.received['fused_state'])
        h.drive(1.6)
        states = [s for s in h.received['fused_state'][before:] if s.initialized]
        older = [s for s in states if stamp_seconds(s.header.stamp) - stamp_seconds(s.rtk_stamp) > 0.30]
        self.assertTrue(older, 'Regression did not exercise accepted navigation ages over 300 ms')
        self.assertTrue(all(s.navigation_ready and s.estimator_ready for s in older))
        self.assertTrue(all(s.navigation_sample_stamp != s.rtk_stamp for s in older))
        h.navigation_enabled = False
        h.drive(0.9)
        self.assertFalse(h.state().navigation_valid)
        self.assertFalse(h.state().navigation_ready)
        self.assertFalse(h.state().estimator_ready)


class HeightFusionConfigurationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_legacy_boolean_mapping_and_explicit_mode_precedence(self):
        for parameters, mode, dimensions in (
                ({'gnss_use_baro_height': False}, 'legacy_full3d', 7),
                ({'gnss_use_baro_height': True}, 'baro_primary', 5),
                ({'gnss_use_baro_height': True, 'height_fusion_mode': 'legacy_full3d'}, 'legacy_full3d', 7),
                ({'gnss_use_baro_height': False, 'height_fusion_mode': 'baro_gnss_weighted'}, 'baro_gnss_weighted', 5)):
            with self.subTest(parameters=parameters):
                h = BarometerHarness(navigation_source='gnss', parameters=parameters)
                try:
                    deadline = time.monotonic() + 8.0
                    while time.monotonic() < deadline:
                        h.drive(0.2)
                        if h.status().get('accepted_updates', 0) > 3:
                            break
                    status = h.status()
                    self.assertGreater(status.get('accepted_updates', 0), 3)
                    self.assertEqual(status['height_fusion_mode'], mode)
                    self.assertEqual(status['navigation_nis_dimensions'], dimensions)
                    self.assertEqual(status['relative_reference_active'], int(mode == 'baro_gnss_weighted'))
                finally:
                    h.close()

    def test_absolute_height_covariance_remains_the_readiness_gate(self):
        h = BarometerHarness(navigation_source='gnss', parameters={
            'height_fusion_mode': 'baro_gnss_weighted', 'navigation_ready_updates': 3,
            'max_horizontal_position_stddev': 3.0, 'max_vertical_position_stddev': 0.1,
            'max_velocity_stddev': 1.0, 'max_heading_stddev': 1.0})
        self.addCleanup(h.close)
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            h.drive(0.2)
            if h.status().get('gnss_baro_height_active') == 1 and h.state().navigation_accepted_updates == 3:
                break
        status = h.status()
        state = h.state()
        self.assertTrue(state.navigation_ready)
        self.assertEqual(state.navigation_accepted_updates, 3)
        self.assertGreater(state.position_variance[2], 0.01)
        self.assertGreater(status['absolute_height_variance_m2'], status['relative_height_variance_m2'])
        self.assertFalse(state.estimator_ready)
        self.assertIn('covariance', state.readiness_reason)


    def test_new_gnss_mode_does_not_change_rtk_updates_or_bias_noise_default(self):
        h = BarometerHarness(navigation_source='rtk', parameters={'height_fusion_mode': 'baro_gnss_weighted'})
        self.addCleanup(h.close)
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            h.drive(0.2)
            if h.status().get('accepted_updates', 0) > 3:
                break
        status = h.status()
        self.assertGreater(status.get('accepted_updates', 0), 3)
        self.assertEqual(status['weighted_height_mode_active'], 0)
        self.assertEqual(status['baro_bias_random_walk_m2_s'], 0.01)
        self.assertEqual(status['relative_reference_active'], 0)
        self.assertEqual(status['navigation_nis_dimensions'], 7)
        self.assertAlmostEqual(h.state().position.z, 3.0, delta=0.05)

    def test_invalid_weighted_startup_parameters_are_rejected(self):
        cases = [('height_fusion_mode', 'unknown', 'height_fusion_mode'),
                 ('baro_bias_random_walk', 0.01, 'baro_bias_random_walk')]
        cases += [(name, 0.5, 'variance scales') for name in
                  ('gnss_height_variance_scale', 'gnss_vertical_velocity_variance_scale')]
        cases += [(name, 0.0, name) for name in
                  ('gnss_height_variance_floor', 'gnss_vertical_velocity_variance_floor')]
        cases += [(name, value, name) for name in
                  ('navigation_height_nis_threshold', 'navigation_vertical_velocity_nis_threshold')
                  for value in (0.0, '.nan', '.inf')]
        for name, value, reason in cases:
            with self.subTest(parameter=name, value=value):
                parameters = {'height_fusion_mode': 'baro_gnss_weighted', name: value}
                h = BarometerHarness(navigation_source='gnss', parameters=parameters)
                try:
                    return_code = h.processes[0].wait(timeout=5.0)
                    h.logs[0].seek(0)
                    output = h.logs[0].read()
                    self.assertNotEqual(return_code, 0, output)
                    self.assertIn(reason, output)
                finally:
                    h.close()


if __name__ == '__main__':
    unittest.main()
