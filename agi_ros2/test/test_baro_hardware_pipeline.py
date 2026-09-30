#!/usr/bin/env python3
"""Six-node shadow evaluation with real MAVLink/MSP decoders and two PTYs.

Pressure travels through the MAVLink receiver, atomic session contract and fusion.
GNSS flight-accuracy thresholds remain unconfigured; no physical UART is opened.
"""
import time
import unittest

import rclpy
from agi_ros2.msg import Barometer, LocalNavigation
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import FluidPressure

from test_hardware_pipeline import HardwareHarness


class BarometerHardwareHarness(HardwareHarness):
    def __init__(self):
        self.next_pressure = 0.0
        super().__init__(shadow=True, controller='GEO')
        self.subscribe('fusion/baro/status', DiagnosticArray)
        self.subscribe('sensors/baro/sample', Barometer)
        self.subscribe_sensor('sensors/baro/pressure', FluidPressure)
        self.subscribe_sensor('sensors/local_navigation', LocalNavigation)

    def start_node(self, executable, **params):
        if executable == 'gnss_adapter.py':
            params.update(max_horizontal_accuracy=0.0, max_vertical_accuracy=0.0, max_velocity_accuracy=0.0)
        elif executable == 'state_fusion_node':
            params.update(baro_enabled=True, observation_delay=0.2, baro_reference_duration=0.3,
                          baro_reference_min_samples=5)
        elif executable == 'mavlink_sensor_node':
            params.update(baro_rate_hz=40, baro_pressure_variance_pa2=0.0)
        return super().start_node(executable, **params)

    def drive_sensors(self):
        super().drive_sensors()
        now = time.monotonic()
        if self.streaming and now >= self.next_pressure:
            self.peer.scaled_pressure_send((self.remote() // 1000) & 0xffffffff, 1013.25, 0.0, 2500)
            self.next_pressure = now + 0.025

    def barometer_status(self):
        messages = self.received['fusion/baro/status']
        if not messages:
            return {}
        status = messages[-1].status[0]
        return dict(message=status.message, **{value.key: float(value.value) for value in status.values})


class BarometerHardwarePipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_pressure_fuses_with_known_accuracy_without_flight_limits_in_shadow(self):
        h = BarometerHardwareHarness()
        self.addCleanup(h.close)
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            h.run(0.1)
            if (h.barometer_status().get('accepted_updates', 0) > 10 and
                    any(message.controller_success for message in h.received['computation_status'])):
                break
        self.assertGreater(h.barometer_status().get('accepted_updates', 0), 10, h.snapshot())
        self.assertEqual(h.barometer_status()['reference_valid'], 1)
        pressure = h.received['sensors/baro/pressure']
        samples = [message for message in h.received['sensors/baro/sample'] if message.valid]
        navigation = [message for message in h.received['sensors/local_navigation'] if message.observation_valid]
        self.assertTrue(pressure)
        self.assertTrue(samples)
        self.assertTrue(navigation)
        self.assertAlmostEqual(pressure[-1].fluid_pressure, 101325.0, places=3)
        self.assertEqual(samples[-1].source_session, navigation[-1].source_session)
        self.assertTrue(samples[-1].clock_aligned)
        self.assertTrue(navigation[-1].accuracy_known)
        self.assertFalse(navigation[-1].accuracy_ok)
        state = h.received['fused_state'][-1]
        self.assertTrue(state.initialized)
        self.assertTrue(state.accuracy_known)
        self.assertFalse(state.navigation_accuracy_ok)
        self.assertFalse(state.navigation_ready)
        self.assertFalse(state.estimator_ready)
        self.assertTrue(any(message.controller_success for message in h.received['computation_status']), h.snapshot())
        # Physical ARM/AUTO evidence still cannot make the shadow output send MSP 200.
        h.response_armed = h.response_auto = True
        h.run(0.4)
        self.assertEqual(h.barometer_status()['reference_valid'], 1)
        self.assertFalse(h.has_output())
        self.assertTrue(h.received['output_status'])
        self.assertTrue(all(not message.override_active for message in h.received['output_status']))


if __name__ == '__main__':
    unittest.main()
