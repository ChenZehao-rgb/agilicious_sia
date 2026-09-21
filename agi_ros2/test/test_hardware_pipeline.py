#!/usr/bin/env python3
"""Six real nodes, two emulated UARTs and genuine decoded health evidence.

No physical devices, Gazebo or fabricated Authority/Health publishers are used.
Source ROS and the built workspace first; pymavlink is required.
"""
import os
from pathlib import Path
import pty
import time
import unittest

import rclpy
import yaml
from pymavlink.dialects.v20 import common as mav
from std_msgs.msg import String
from agi_ros2.msg import ComputationStatus, FusedState, Health, MspState, OutputStatus
from test_shadow_pipeline import ShadowHarness
from test_runtime_profile_nodes import dump_profile


class HardwareHarness(ShadowHarness):
    def __init__(self, shadow=False, override_timeout='50', controller='MPC'):
        super().__init__()
        self.response_armed = False
        self.config_frames[119] = bytes((27, 50, 0, 1))  # Include ANGLE in the FC's stable BOXIDS list.
        self.override_settings['msp_override_timeout_ms'] = override_timeout
        self.mav_master, self.mav_slave = pty.openpty()
        os.set_blocking(self.mav_master, False)
        self.peer = mav.MAVLink(self, srcSystem=1, srcComponent=1)
        self.peer.robust_parsing = True
        self.epoch = time.monotonic() - 100.
        self.next_imu = self.next_gps = 0.
        self.heading = 9000
        self.fix_type = 3
        self.invalid_velocity = False
        self.streaming = True
        self.gps_streaming = True
        self.driver = self.drive_sensors
        for topic, cls in [('fused_state', FusedState), ('health', Health), ('msp/decoded_state', MspState),
                           ('output_status', OutputStatus), ('computation_status', ComputationStatus), ('status', String)]:
            self.subscribe(topic, cls)
        table = self.path / 'test_thrust.csv'
        table.write_text('0,1000,1500,2000\n12,0,10,20\n18,0,20,40\n')
        configs = Path(__file__).resolve().parents[1] / 'config'
        profile = yaml.safe_load((configs / 'hardware.yaml').read_text())
        synthetic = yaml.safe_load((configs / 'simulation.yaml').read_text())['pilot']['quadrotor']
        profile['pilot']['quadrotor'] = {key: synthetic[key] for key in
                                         ('mass', 'omega_max', 'thrust_min', 'thrust_max')}
        profile['bridge'] = yaml.safe_load(self.bridge.read_text())
        profile['flight']['shadow_only'] = shadow
        profile['flight']['thrust_table'] = str(table)
        path = self.path / 'hardware.yaml'
        path.write_text(dump_profile(profile))
        # Both controllers use the hardware profile with only measured command limits.
        # Firmware faults stay in the emulated readback, independent of this expected configuration.
        control_parameters = dict(runtime_config=str(path), controller=controller)
        self.start_node('command_output_node', shadow_only=shadow, navigation_source='gnss',
                        device=os.ttyname(self.slave), thrust_table=str(table), **control_parameters,
                        **{'msp.read_configuration': True, 'msp.rc.rate_hz': 25., 'msp.status.rate_hz': 25.,
                           'msp.battery.rate_hz': 2., 'msp.attitude.enabled': False,
                           'msp.analog.enabled': False, 'msp.gps.enabled': False})
        self.start_node('msp_evidence.py', bridge_config=str(self.bridge),
                        geofence_min=[-10., -10., -2.], geofence_max=[10., 10., 10.])
        self.start_node('mavlink_sensor_node', device=os.ttyname(self.mav_slave), gps_mode='gnss',
                        altitude_source='msl', imu_rate_hz=500, gps_rate_hz=10, attitude_rate_hz=0)
        self.start_node('gnss_adapter.py', heading_confirmed=True, fc_declination_applied=True,
                        origin_duration=.3, origin_samples=3,
                        max_horizontal_accuracy=2., max_vertical_accuracy=3., max_velocity_accuracy=.6)
        self.start_node('state_fusion_node', navigation_source='gnss', imu_initialization_duration=.4,
                        imu_initialization_samples=100, navigation_ready_updates=5,
                        max_horizontal_position_stddev=2., max_vertical_position_stddev=3.,
                        max_velocity_stddev=.6, max_heading_stddev=.5)
        self.start_node('control_node', shadow_only=shadow, navigation_source='gnss', **control_parameters)

    def write(self, data):
        os.write(self.mav_master, data)

    def remote(self):
        return int((time.monotonic() - self.epoch) * 1e6)

    def drive_sensors(self):
        try:
            data = os.read(self.mav_master, 8192)
        except BlockingIOError:
            data = b''
        for msg in self.peer.parse_buffer(data) or []:
            if msg.get_type() == 'TIMESYNC':
                self.peer.timesync_send(self.remote()*1000, msg.ts1)
            elif msg.get_type() == 'COMMAND_LONG':
                self.peer.command_ack_send(msg.command, mav.MAV_RESULT_ACCEPTED,
                                           target_system=245, target_component=191)
        now = time.monotonic()
        if self.streaming and now >= self.next_imu:
            self.peer.highres_imu_send(self.remote(), 0., 0., -9.80665, 0., 0., 0.,
                                      0., 0., 0., 0., 0., 0., 0., 63)
            self.next_imu = now + .002
        if self.streaming and self.gps_streaming and now >= self.next_gps:
            t = self.remote()
            self.peer.gps_raw_int_send(t, self.fix_type, 310000000, 1210000000, 20000, 100, 100, 0, 0, 12,
                                      25000, 500, 1000, 200)
            self.peer.global_position_int_send((t//1000) & 0xffffffff, 310000000, 1210000000,
                                               20000, 0, 32767 if self.invalid_velocity else 0, 0, 0, self.heading)
            self.next_gps = now + .1

    def snapshot(self):
        return {k: str(v[-1]) for k, v in self.received.items() if v}

    def wait_ready(self, timeout=12.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.run(.05)
            health = self.received['health']
            computation = self.received['computation_status']
            if (health and computation and health[-1].imu_ready and health[-1].estimator_ready and
                    health[-1].navigation_ready and health[-1].config_verified and health[-1].geofence_ok and
                    computation[-1].warm_cycles >= 50):
                return
        raise AssertionError('hardware pipeline did not become ready: ' + str(self.snapshot()))

    def has_output(self, since=0):
        return any(code == 200 for _, code, _ in self.codes[since:])

    def close(self):
        super().close()
        os.close(self.mav_master)
        os.close(self.mav_slave)


class HardwarePipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def harness(self, **kwargs):
        h = HardwareHarness(**kwargs)
        self.addCleanup(h.close)
        return h

    def test_real_gnss_hover_authorization_and_recovery(self):
        self.check_real_gnss_hover('MPC')

    def test_geo_gnss_hover_authorization_and_recovery(self):
        self.check_real_gnss_hover('GEO')

    def check_real_gnss_hover(self, controller):
        h = self.harness(controller=controller)
        h.wait_ready()
        self.assertTrue(all(s.controller_type == controller for s in h.received['computation_status']))
        self.assertFalse(h.has_output())
        # AUTO raised while disarmed must not be reused after ARM becomes high.
        h.response_auto = True
        h.run(.15)
        h.response_armed = True
        h.run(.3)
        self.assertFalse(h.has_output())
        h.response_auto = False
        h.wait_ready()
        h.run(.05)
        h.response_auto = True
        h.run(.3)
        self.assertTrue(h.has_output(), h.snapshot())
        self.assertTrue(any(s.override_active for s in h.received['output_status']))
        self.assertTrue(all(not s.rtk_fixed and not s.synchronized for s in h.received['fused_state']))
        # A changed rate profile revokes output using STATUS_EX, before the next curve readback.
        h.response_rate_profile = 1
        h.run(.2)
        start = len(h.codes)
        h.run(.15)
        self.assertFalse(h.has_output(start))
        h.response_rate_profile = 0
        h.run(.8)
        start = len(h.codes)
        h.run(.2)
        self.assertFalse(h.has_output(start))  # No automatic restart with AUTO still high.
        h.response_auto = False
        h.wait_ready()
        h.run(.05)
        h.response_conflicting_mode = True
        h.response_auto = True
        start = len(h.codes)
        h.run(.3)
        self.assertFalse(h.has_output(start))
        self.assertTrue(any('conflicts' in s.reason for s in h.received['msp/decoded_state']))

    def test_diagnostic_shadow_never_writes_control(self):
        self.check_shadow('MPC')

    def test_geo_shadow_never_writes_control(self):
        self.check_shadow('GEO')

    def check_shadow(self, controller):
        h = self.harness(shadow=True, controller=controller)
        h.wait_ready()
        h.response_armed = h.response_auto = True
        h.run(.3)
        self.assertTrue(any(s.mpc_success for s in h.received['computation_status']))
        self.assertTrue(any(s.controller_success and s.controller_type == controller
                            for s in h.received['computation_status']))
        self.assertFalse(h.has_output())
        self.assertTrue(all(not s.override_active for s in h.received['output_status']))

    def test_unverified_firmware_timeout_cannot_authorize(self):
        h = self.harness(override_timeout='300')
        h.response_armed = h.response_auto = True
        h.run(3.)
        self.assertFalse(h.has_output())
        self.assertTrue(h.received['msp/decoded_state'])
        self.assertTrue(all(not s.config_verified for s in h.received['msp/decoded_state']))
        self.assertTrue(any('50 ms' in s.reason for s in h.received['msp/decoded_state']))

    def test_explicit_navigation_loss_revokes_before_age_timeout(self):
        h = self.harness()
        h.wait_ready()
        h.response_armed = True
        for field, invalid in (('heading', 65535), ('fix_type', 1), ('invalid_velocity', True)):
            with self.subTest(field=field):
                h.response_auto = False
                h.wait_ready()
                h.run(.05)
                start = len(h.codes)
                h.response_auto = True
                h.run(.15)
                self.assertTrue(h.has_output(start), h.snapshot())
                original = getattr(h, field)
                setattr(h, field, invalid)
                h.next_gps = 0.
                h.run(.08)  # Explicit invalid evidence must not wait for the 300 ms GNSS deadline.
                self.assertFalse(h.received['health'][-1].navigation_ready, h.snapshot())
                self.assertFalse(h.received['output_status'][-1].override_active)
                last_fix = h.received['fused_state'][-1].rtk_stamp
                start = len(h.codes)
                h.run(.08)
                self.assertFalse(h.has_output(start))
                self.assertEqual(h.received['fused_state'][-1].rtk_stamp, last_fix)
                setattr(h, field, original)
                h.next_gps = 0.

    def test_fc_error_ack_latches_transport_closed(self):
        h = self.harness()
        h.wait_ready()
        h.response_armed = True
        h.run(.1)
        h.wait_ready()
        h.response_error_ack = True
        h.response_auto = True
        h.run(.25)
        self.assertTrue(h.has_output(), h.snapshot())  # The first frame was sent but rejected by the FC.
        self.assertFalse(h.received['output_status'][-1].transport_healthy)
        self.assertFalse(h.received['output_status'][-1].override_active)
        self.assertEqual(h.received['output_status'][-1].last_fault, 'Critical MSP telemetry failed')
        start = len(h.codes)
        h.response_error_ack = False
        h.run(.2)
        self.assertFalse(h.has_output(start))


if __name__ == '__main__':
    unittest.main()
