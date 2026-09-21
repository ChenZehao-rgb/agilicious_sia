#!/usr/bin/env python3
"""Real ROS nodes, synthetic navigation and PTY MSP. Never opens hardware."""
import math
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import time
import unittest

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from agi_ros2.msg import (Authority, Health, FusedState, OutputStatus, ControlCommand, ComputationStatus,
                          Navigation, LocalNavigation, NavigationOrigin, MspState, MspEvent)
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from test_node_pipeline import Harness, BIN, PARAMS, CLOCK_ID
from test_shadow_support import config_frames, status_frame


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0xd5 if crc & 128 else crc << 1) & 255
    return crc


class ShadowHarness(Harness):
    def __init__(self):
        super().__init__()
        self.driver = lambda: None
        self.codes = []
        self.drop_code = None
        self.response_auto = False
        self.response_kill = False
        self.response_armed = True
        self.session = 'fc-one'
        self.last_nav = 0.
        self.master, self.slave = pty.openpty()
        os.set_blocking(self.master, False)

    def start_node(self, executable, **params):
        defaults = dict(params_dir=str(PARAMS), mode='hardware')
        defaults.update(params)
        args = [str(BIN / executable), '--ros-args', '-r', '__ns:=' + self.namespace]
        for k, v in defaults.items():
            args += ['-p', k + ':=' + (str(v).lower() if isinstance(v, bool) else str(v))]
        log = open(self.path / (executable + str(len(self.processes)) + '.log'), 'w+')
        proc = subprocess.Popen(args, stdout=log, stderr=log)
        self.processes.append(proc)
        self.logs.append(log)
        return proc

    def subscribe_sensor(self, topic, cls):
        self.received[topic] = []
        self.subs.append(self.node.create_subscription(cls, topic,
                         lambda m: self.received[topic].append(m), qos_profile_sensor_data))

    def publish_inputs(self, sensors, commands):
        self.driver()

    def drain(self):
        while True:
            try:
                data = os.read(self.master, 8192)
                if not data:
                    break
                self.serial_pending.extend(data)
            except BlockingIOError:
                break
        buf = self.serial_pending
        while len(buf) >= 6:
            assert buf[:3] in (b'$M<', b'$X<'), bytes(buf[:8])
            v2 = buf[1] == ord('X')
            header = 8 if v2 else 5
            if len(buf) < header:
                break
            length = int.from_bytes(buf[6:8], 'little') if v2 else buf[3]
            code = int.from_bytes(buf[4:6], 'little') if v2 else buf[4]
            if len(buf) < header + length + 1:
                break
            frame = bytes(buf[:header+length+1])
            del buf[:len(frame)]
            check = crc8(frame[3:-1]) if v2 else __import__('functools').reduce(int.__xor__, frame[3:-1], 0)
            assert check == frame[-1]
            self.codes.append((time.monotonic(), code, frame[header:-1]))
            if code == self.drop_code:
                continue
            if code == 0x3010:
                payload = frame[header:-1]
                assert b'=' not in payload
                name = payload.split(b'\0')[0].decode()
                assert name in ('msp_override_channels_mask', 'msp_override_failsafe')
                value = '15' if name.endswith('mask') else 'OFF'
                reply = f'{name} = {value}'.encode()
            elif code == 105:
                reply = struct.pack('<7H', 1500, 1500, 1000, 1500,
                                    1800 if self.response_armed else 1000,
                                    1800 if self.response_auto else 1000,
                                    1800 if self.response_kill else 1000)
            elif code == 101:
                reply = status_frame(self.response_armed, self.response_auto, self.response_kill)
            elif code == 130:
                reply = bytearray(11)
                reply[0] = 4
                struct.pack_into('<H', reply, 9, 1600)
            else:
                reply = config_frames().get(code, b'')
            if v2:
                body = bytes((0, code & 255, code >> 8, len(reply), 0)) + reply
                os.write(self.master, b'$X>' + body + bytes((crc8(body),)))
            else:
                body = bytes((len(reply), code)) + reply
                checksum = __import__('functools').reduce(int.__xor__, body, 0)
                os.write(self.master, b'$M>' + body + bytes((checksum,)))

    def output(self):
        return self.start_node('command_output_node', shadow_only=True, device=os.ttyname(self.slave),
            bridge_config=str(self.bridge), **{'msp.read_configuration': True, 'msp.rc.rate_hz':25.,
            'msp.status.rate_hz':25., 'msp.battery.rate_hz':2., 'msp.gps.enabled':False,
            'msp.attitude.enabled':False, 'msp.analog.enabled':False})

    def navigation(self):
        m = Navigation()
        m.header.stamp = self.stamp()
        m.header.frame_id = 'gps_enu'
        m.source_session = self.session
        m.fix_type = 3
        m.latitude, m.longitude, m.altitude = 31., 121., 20.
        m.altitude_reference = 'msl'
        m.heading, m.heading_valid, m.clock_aligned = .4, True, True
        m.horizontal_accuracy = m.vertical_accuracy = m.velocity_accuracy = math.nan
        return m

    def sensors(self):
        now = time.monotonic()
        if now - self.last_nav > .1:
            self.publisher('sensors/navigation', Navigation, True).publish(self.navigation())
            self.last_nav = now
        if now - self.last_imu > .002:
            imu = Imu()
            imu.header.stamp = self.stamp()
            imu.header.frame_id = 'base_link'
            imu.linear_acceleration.z = 9.8066
            self.publisher('sensors/imu', Imu, True).publish(imu)
            self.last_imu = now

    def control_inputs(self):
        now = time.monotonic()
        s = FusedState()
        s.header.stamp = self.stamp()
        s.header.frame_id = 'odom'
        s.position.z = 2.
        s.orientation.w = 1.
        s.clock_id = CLOCK_ID
        s.published_steady_time = s.imu_receive_time = s.rtk_receive_time = now
        s.rtk_stamp = self.stamp()
        s.initialized = s.navigation_valid = s.clock_aligned = s.heading_valid = True
        s.navigation_source = 'gnss'
        s.fix_type = 3
        s.reset_counter = 1
        self.publisher('fused_state', FusedState).publish(s)
        a = Authority()
        a.header.stamp = self.stamp()
        a.armed = self.response_armed
        a.auto_switch = self.response_auto
        a.kill = self.response_kill
        a.rc_link = True
        self.publisher('authority', Authority).publish(a)
        health = Health()  # Deliberately no fabricated calibration or convergence.
        health.header.stamp = self.stamp()
        self.publisher('health', Health).publish(health)
        out = OutputStatus()
        out.header.stamp = self.stamp()
        out.clock_id = CLOCK_ID
        out.session_start = 1.
        out.steady_time = now
        out.transport_healthy = True
        self.publisher('output_status', OutputStatus).publish(out)


class ShadowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.h = ShadowHarness()
        self.addCleanup(self.h.close)

    def test_output_interlock_telemetry_and_readonly_parameter(self):
        h = self.h
        h.subscribe('output_status', OutputStatus)
        h.subscribe('msp/decoded_state', MspState)
        h.subscribe('authority', Authority)
        h.subscribe('health', Health)
        h.subscribe('msp/events', MspEvent)
        h.output()
        h.start_node('msp_evidence.py', bridge_config=str(h.bridge))
        def forged():
            m = ControlCommand()
            m.header.stamp = h.stamp()
            m.header.frame_id = 'base_link'
            m.clock_id = CLOCK_ID
            m.permit_override = True
            m.evidence.now = time.monotonic()
            m.total_thrust = 10.
            h.publisher('control_command', ControlCommand).publish(m)
        h.driver = forged
        h.run(2.5)
        self.assertTrue(any(m.config_verified for m in h.received['msp/decoded_state']),
                        [m.reason for m in h.received['msp/decoded_state'][-5:]])
        self.assertTrue(any(m.armed and m.rc_link and not m.kill for m in h.received['authority']))
        self.assertTrue(any(m.battery_voltage == 16. for m in h.received['health']))
        self.assertTrue(all(not m.converged and not m.imu_calibrated for m in h.received['health']))
        h.response_auto = True
        h.run(.3)
        h.response_kill = True
        h.run(.3)
        client = h.node.create_client(SetParameters, 'command_output/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=2))
        request = SetParameters.Request(parameters=[Parameter(name='shadow_only',
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False))])
        future = client.call_async(request)
        h.run(.2)
        self.assertTrue(future.done())
        self.assertFalse(future.result().results[0].successful)
        self.assertTrue(h.received['output_status'])
        self.assertTrue(all(not m.override_active for m in h.received['output_status']))
        self.assertNotIn(200, [code for _, code, _ in h.codes])
        for code in (105, 101):
            stamps = [t for t, c, _ in h.codes if c == code]
            rate = (len(stamps)-1)/(stamps[-1]-stamps[0])
            self.assertAlmostEqual(rate, 25., delta=3.)
        events = [m for m in h.received['msp/events'] if m.event == 'rx' and m.code == 105]
        self.assertTrue(events)
        self.assertLess(events[-1].request_steady_time, events[-1].steady_time)
        h.drop_code = 101
        h.run(.3)
        self.assertTrue(h.received['authority'][-1].kill)
        self.assertFalse(h.received['authority'][-1].rc_link)
        # Restart while physical AUTO is held high: output remains impossible.
        h.processes[0].send_signal(signal.SIGINT)
        h.processes[0].wait(timeout=3)
        h.processes.pop(0)
        h.logs.pop(0).close()
        h.drain()
        h.output()
        h.run(.3)
        self.assertNotIn(200, [code for _, code, _ in h.codes])

    def test_gnss_fusion_session_and_unknown_accuracy(self):
        h = self.h
        h.subscribe_sensor('sensors/local_navigation', LocalNavigation)
        h.subscribe('fused_state', FusedState)
        h.start_node('gnss_adapter.py', heading_confirmed=True, heading_correction_rad=3., origin_duration=.3, origin_samples=3)
        h.start_node('state_fusion_node', navigation_source='gnss')
        h.driver = h.sensors
        h.run(2.)
        self.assertTrue(h.received['sensors/local_navigation'])
        nav = h.received['sensors/local_navigation'][-1]
        self.assertEqual(list(nav.position_variance), [1., 1., 4.])
        self.assertFalse(nav.accuracy_known)
        self.assertEqual(nav.altitude_reference, 'msl')
        self.assertAlmostEqual(nav.heading, math.remainder(3.4, 2*math.pi))
        self.assertTrue(h.received['fused_state'])
        s = h.received['fused_state'][-1]
        self.assertTrue(s.initialized)
        self.assertTrue(s.navigation_valid)
        self.assertFalse(s.rtk_fixed or s.synchronized)
        self.assertEqual(s.fix_type, 3)
        previous = s.reset_counter
        h.session = 'fc-two'
        h.run(.5)
        self.assertGreater(h.received['fused_state'][-1].reset_counter, previous)
        self.assertEqual(h.received['sensors/local_navigation'][-1].session_id, nav.session_id)
        # No new IMU => no new state; a subsequent gap resets the EKF.
        count = len(h.received['fused_state'])
        h.driver = lambda: None
        h.run(.1)
        self.assertLessEqual(len(h.received['fused_state'])-count, 5)
        h.driver = h.sensors
        h.run(.5)
        self.assertGreater(h.received['fused_state'][-1].reset_counter, previous)

    def test_unconfirmed_heading_does_not_initialize(self):
        h = self.h
        h.subscribe_sensor('sensors/local_navigation', LocalNavigation)
        h.start_node('gnss_adapter.py', origin_duration=.2, origin_samples=2)
        h.driver = h.sensors
        h.run(1.2)
        self.assertFalse(h.received['sensors/local_navigation'])

    def test_shadow_mpc_health_separation_and_edges(self):
        h = self.h
        h.subscribe('computation_status', ComputationStatus)
        h.subscribe('control_command', ControlCommand)
        h.subscribe('status', String)
        h.subscribe('reference', Odometry)
        trajectory = h.path / 'trajectory.csv'
        rows = ['t,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,w_x,w_y,w_z']
        for i in range(101):
            values = [i*.1, i*.001, 0., 0., 1., 0., 0., 0., .01, 0., 0., 0., 0., 0.,
                      0., 0., 0., 0., 0., 0., 2., 2., 2., 2., 0., 0., 0., 0., 0., 0.]
            rows.append(','.join(str(v) for v in values))
        trajectory.write_text('\n'.join(rows)+'\n')
        h.start_node('control_node', shadow_only=True, trajectory=str(trajectory))
        h.driver = h.control_inputs
        h.response_auto = True
        h.run(2.)
        self.assertTrue(any(m.mpc_success for m in h.received['computation_status']))
        self.assertFalse(any(m.trajectory_active for m in h.received['computation_status']))
        h.response_auto = False
        h.run(1.)
        self.assertTrue(any(m.warm_cycles == 50 for m in h.received['computation_status']))
        h.response_auto = True
        h.run(.2)
        self.assertTrue(any(m.trajectory_active for m in h.received['computation_status'][-20:]),
                        [(m.warm_cycles, m.reason) for m in h.received['computation_status'][-10:]])
        self.assertTrue(any(m.pose.pose.position.x > .0001 for m in h.received['reference']))
        self.assertTrue(all(not m.permit_override for m in h.received['control_command']))
        self.assertTrue(all(not m.evidence.rtk_fixed and not m.evidence.converged for m in h.received['control_command']))
        h.response_kill = True
        h.run(.1)
        self.assertFalse(h.received['computation_status'][-1].trajectory_active)
        h.response_kill = False
        h.run(.8)
        self.assertFalse(h.received['computation_status'][-1].trajectory_active)
        h.driver = lambda: None
        h.run(.1)
        self.assertFalse(h.received['computation_status'][-1].state_valid)


if __name__ == '__main__':
    unittest.main()
