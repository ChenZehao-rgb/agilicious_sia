#!/usr/bin/env python3
"""Props-off bench policy through six real nodes and emulated MAVLink/MSP UARTs."""
import math
import time
import unittest

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from agi_ros2.msg import Barometer, LocalNavigation, NavigationOrigin

from test_hardware_pipeline import HardwareHarness
from test_baro_fusion import diagnostic_values


class BenchHarness(HardwareHarness):
    def __init__(self):
        self.next_pressure = self.next_attitude = 0.
        self.attitude_streaming = True
        super().__init__(shadow=False, controller='GEO', thrust_model='quadratic', observation_delay=.2)
        self.gps_streaming = False
        self.subscribe('fusion/baro/status', DiagnosticArray)
        self.subscribe('sensors/baro/sample', Barometer)
        self.subscribe('navigation/origin', NavigationOrigin)
        self.subscribe_sensor('sensors/local_navigation', LocalNavigation)

    def start_node(self, executable, **params):
        if executable == 'mavlink_sensor_node':
            params.update(bench_fixed_gps=True, baro_rate_hz=40)
        elif executable == 'state_fusion_node':
            params.update(baro_enabled=True, height_fusion_mode='baro_gnss_weighted',
                          baro_reference_duration=.3, baro_reference_min_samples=5, baro_bias_random_walk=0.)
        return super().start_node(executable, **params)

    def drive_sensors(self):
        super().drive_sensors()
        now = time.monotonic()
        if self.streaming and now >= self.next_pressure:
            self.peer.scaled_pressure_send((self.remote() // 1000) & 0xffffffff, 1013.25, 0., 2500)
            self.next_pressure = now + .025
        if self.streaming and self.attitude_streaming and now >= self.next_attitude:
            self.peer.attitude_send((self.remote() // 1000) & 0xffffffff, 0., 0., math.pi / 2, 0., 0., 0.)
            self.next_attitude = now + .1


class BenchPipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_fixed_gps_pressure_and_real_authority_gates(self):
        h = BenchHarness()
        self.addCleanup(h.close)
        h.wait_ready()
        samples = [s for s in h.received['sensors/baro/sample'] if s.valid]
        navigation = [s for s in h.received['sensors/local_navigation'] if s.observation_valid]
        self.assertTrue(samples and navigation)
        self.assertEqual(samples[-1].source_session, navigation[-1].source_session)
        self.assertEqual(h.received['navigation/origin'][-1].heading_source, 'fc_attitude_yaw_bench')
        status = diagnostic_values(h.received['fusion/baro/status'][-1].status[0])
        self.assertEqual(status['reference_valid'], 1)
        self.assertGreater(status['accepted_updates'], 0)
        self.assertEqual(status['gnss_baro_height_active'], 1)
        self.assertFalse(h.has_output())
        h.response_armed = True
        h.run(.8)
        self.assertFalse(h.has_output())
        h.response_auto = True
        h.run(.3)
        self.assertTrue(h.has_output(), h.snapshot())
        h.attitude_streaming = False
        h.run(.7)
        start = len(h.codes)
        h.run(.15)
        self.assertFalse(h.has_output(start), h.snapshot())
        self.assertFalse(h.received['fused_state'][-1].navigation_ready)
        h.attitude_streaming = True
        h.response_auto = False
        h.wait_ready()
        h.run(.2)
        h.response_auto = True
        start = len(h.codes)
        h.run(.3)
        self.assertTrue(h.has_output(start), h.snapshot())
        h.response_kill = True
        h.run(.15)
        start = len(h.codes)
        h.run(.15)
        self.assertFalse(h.has_output(start))


if __name__ == '__main__':
    unittest.main()
