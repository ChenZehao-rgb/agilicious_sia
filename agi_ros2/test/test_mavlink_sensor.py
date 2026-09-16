#!/usr/bin/env python3
"""Real ROS node + pseudo-UART integration; no physical device is opened.
Source ROS and the agi_ros2 install before running. Requires pymavlink.
"""
import math
import importlib.util
import os
from pathlib import Path
import pty
import subprocess
import tempfile
import time
import unittest
import uuid

import rclpy
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from pymavlink.dialects.v20 import common as mav
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from diagnostic_msgs.msg import DiagnosticArray
from agi_ros2.msg import Heading


class Harness:
    def __init__(self, **params):
        self.tmp = tempfile.TemporaryDirectory(prefix='agi_mavlink_')
        self.device = Path(self.tmp.name) / 'uart'
        self.new_port()
        self.epoch = time.monotonic() - 100
        self.peer = mav.MAVLink(self, srcSystem=1, srcComponent=1)
        self.peer.robust_parsing = True
        self.sync = True
        self.sync_delay = 0
        self.pending_sync = []
        self.streaming = True
        self.fix_type = 3
        self.heading = 0
        self.unknown = False
        self.invalid_velocity = False
        self.freeze_gps = False
        self.gps_time = 0
        self.commands = []
        self.next_imu = self.next_gps = self.next_attitude = 0
        self.values = {k: [] for k in ('imu', 'fix', 'velocity', 'attitude', 'heading', 'status')}
        self.node = rclpy.create_node('test_' + uuid.uuid4().hex)
        self.namespace = '/mavtest_' + uuid.uuid4().hex
        topics = [('imu', Imu, 'sensors/imu'), ('fix', NavSatFix, 'sensors/gps/fix'),
                  ('velocity', TwistStamped, 'sensors/gps/velocity'),
                  ('attitude', QuaternionStamped, 'sensors/fc_attitude'),
                  ('heading', Heading, 'sensors/fc_heading'),
                  ('status', DiagnosticArray, 'sensors/mavlink/status')]
        self.subs = [self.node.create_subscription(typ, self.namespace + '/' + topic,
                     lambda m, k=key: self.values[k].append(m),
                     10 if key == 'status' else qos_profile_sensor_data) for key, typ, topic in topics]
        binary = Path(get_package_prefix('agi_ros2')) / 'lib/agi_ros2/mavlink_sensor_node'
        args = [str(binary), '--ros-args', '-r', '__ns:=' + self.namespace,
                '-p', 'device:=' + str(self.device), '-p', 'attitude_rate_hz:=100']
        for key, value in params.items(): args.extend(['-p', key + ':=' + str(value)])
        self.log = open(Path(self.tmp.name) / 'node.log', 'w+')
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=self.log)

    def new_port(self):
        self.master, self.slave = pty.openpty()
        os.set_blocking(self.master, False)
        self.device.unlink(missing_ok=True)
        self.device.symlink_to(os.ttyname(self.slave))

    def write(self, data):
        # Deliberately split every frame: the receiver must retain parser state.
        os.write(self.master, data[:3])
        os.write(self.master, data[3:])

    def remote(self):
        return int((time.monotonic() - self.epoch) * 1e6)

    def imu(self, stamp=None):
        self.peer.highres_imu_send(stamp or self.remote(), 1, 2, -9.80665,
                                  .1, .2, .3, 0, 0, 0, 0, 0, 0, 0, 63)

    def gps(self):
        if not self.freeze_gps: self.gps_time = self.remote()
        t = self.gps_time
        self.peer.gps_raw_int_send(t, self.fix_type, 310000000, 1210000000, 20000,
             100, 100, 224, 4500, 12, -2147483648 if self.unknown else 25000,
             0 if self.unknown else 500, 0 if self.unknown else 1000, 200)
        self.peer.global_position_int_send((t // 1000) & 0xffffffff, 310000000,
             1210000000, 20000, 0, 32767 if self.invalid_velocity else 100, 200, -300, self.heading)

    def drive(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if self.proc.poll() is not None:
                self.log.seek(0)
                raise AssertionError('node exited: ' + self.log.read())
            try: data = os.read(self.master, 8192)
            except BlockingIOError: data = b''
            for m in self.peer.parse_buffer(data) or []:
                if m.get_type() == 'TIMESYNC' and self.sync:
                    self.pending_sync.append((time.monotonic()+self.sync_delay, self.remote()*1000, m.ts1))
                elif m.get_type() == 'COMMAND_LONG':
                    self.commands.append((m.command, m.param1, m.param2))
                    self.peer.command_ack_send(m.command, mav.MAV_RESULT_ACCEPTED, target_system=245, target_component=191)
            t = time.monotonic()
            while self.pending_sync and self.pending_sync[0][0] <= t:
                _, remote, token = self.pending_sync.pop(0)
                self.peer.timesync_send(remote, token)
            if self.streaming:
                if t >= self.next_imu:
                    self.imu(); self.next_imu = t + .002
                if t >= self.next_gps:
                    self.gps(); self.next_gps = t + .1
                if t >= self.next_attitude:
                    self.peer.attitude_send((self.remote() // 1000) & 0xffffffff, 0, 0, 0, 0, 0, 0)
                    self.next_attitude = t + .01
            rclpy.spin_once(self.node, timeout_sec=.0005)

    def status(self):
        return {v.key: v.value for v in self.values['status'][-1].status[0].values}

    def close(self):
        self.proc.terminate()
        try: self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired: self.proc.kill(); self.proc.wait()
        os.close(self.master); os.close(self.slave)
        self.node.destroy_node(); self.log.close(); self.tmp.cleanup()


class SensorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()
    def setUp(self): self.h = Harness()
    def tearDown(self): self.h.close()

    def test_decode_sources_duplicates_and_units(self):
        h = self.h
        h.drive(2)
        self.assertGreater(len(h.values['imu']), 100)
        v = h.values['imu'][-1]
        self.assertEqual(v.header.frame_id, 'base_link')
        self.assertAlmostEqual(v.linear_acceleration.x, 1)
        self.assertAlmostEqual(v.linear_acceleration.y, -2)
        self.assertAlmostEqual(v.linear_acceleration.z, 9.80665, places=5)
        self.assertAlmostEqual(v.angular_velocity.z, -.3, places=6)
        self.assertEqual(v.orientation_covariance[0], -1)
        self.assertLess(abs(time.time() - v.header.stamp.sec - v.header.stamp.nanosec*1e-9), .1)
        gps = h.values['fix'][-1]
        self.assertAlmostEqual(gps.latitude, 31)
        self.assertTrue(math.isnan(gps.altitude))  # explicit unknown is default
        vel = h.values['velocity'][-1]
        self.assertEqual((vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z), (2., 1., 3.))
        q = h.values['attitude'][-1].quaternion
        self.assertAlmostEqual(q.w, math.sqrt(.5), places=6)
        self.assertAlmostEqual(q.z, math.sqrt(.5), places=6)
        self.assertEqual({int(c[1]) for c in h.commands}, {24, 30, 33, 105})
        self.assertTrue(all(c[0] == mav.MAV_CMD_SET_MESSAGE_INTERVAL for c in h.commands))
        h.streaming = False; h.drive(.1)
        count = len(h.values['imu'])
        stamp = h.remote(); h.imu(stamp); h.imu(stamp)
        h.drive(.1)
        self.assertEqual(len(h.values['imu']), count+1)
        h.peer.srcSystem = 42; h.imu(); h.peer.srcSystem = 1
        bad = bytearray(mav.MAVLink_highres_imu_message(h.remote(),0,0,0,0,0,0,0,0,0,0,0,0,0,63).pack(h.peer))
        bad[-1] ^= 0x80; h.write(bad)
        h.drive(1.1)
        self.assertEqual(len(h.values['imu']), count+1)
        self.assertGreater(int(h.status()['bad_frames']), 0)
        self.assertGreater(int(h.status()['wrong_source']), 0)
        self.assertGreater(int(h.status()['duplicates']), 0)

    def test_fix_validity_and_missing_velocity(self):
        h=self.h; h.drive(1.5)
        h.fix_type=1; h.drive(.3)
        self.assertEqual(h.values['fix'][-1].status.status, -1)
        before=len(h.values['velocity']); h.drive(.3)
        self.assertEqual(len(h.values['velocity']), before)
        h.fix_type=3; h.invalid_velocity=True; h.unknown=True; h.drive(.3)
        self.assertEqual(len(h.values['velocity']), before)
        self.assertEqual(h.values['fix'][-1].position_covariance_type, 0)
        h.invalid_velocity=False; h.drive(.3)
        self.assertGreater(len(h.values['velocity']), before)
        h.freeze_gps=True; h.drive(.2)
        before=len(h.values['fix']); h.drive(.3)
        self.assertEqual(len(h.values['fix']), before)

    def test_sync_reboot_and_reconnect(self):
        h=self.h; h.sync=False; h.drive(1)
        self.assertFalse(h.values['imu'])
        h.sync=True; h.drive(1.5)
        self.assertTrue(h.values['imu'])
        h.epoch=time.monotonic(); h.drive(1.5)
        self.assertEqual(h.status()['synchronized'], 'true')
        before=len(h.values['imu'])
        os.close(h.master); os.close(h.slave); h.new_port()
        h.drive(3)
        self.assertGreater(len(h.values['imu']), before)
        self.assertEqual(h.status()['synchronized'], 'true')

    def test_rtk_selection_and_ellipsoid(self):
        self.h.close(); self.h=Harness(gps_mode='rtk', altitude_source='ellipsoid')
        h=self.h; h.drive(2)
        self.assertEqual(h.status()['gps_ready'], 'false')
        self.assertEqual(h.values['fix'][-1].altitude, 25)
        h.fix_type=5; h.drive(1.1)
        self.assertEqual(h.status()['gps_ready'], 'false')
        h.fix_type=6; h.drive(1.1)
        self.assertEqual(h.status()['gps_ready'], 'true')
        h.unknown=True; h.drive(.2)
        self.assertTrue(math.isnan(h.values['fix'][-1].altitude))

    def test_fc_heading_enu_and_unknown(self):
        h=self.h; h.drive(1.5)
        for centidegrees, expected in [(0, math.pi/2), (9000, 0), (18000, -math.pi/2),
                                       (27000, -math.pi), (35999, math.pi/2+math.pi/18000)]:
            h.heading=centidegrees; h.drive(.15)
            v=h.values['heading'][-1]
            self.assertTrue(v.valid)
            self.assertEqual(v.header.frame_id, 'gps_enu')
            self.assertAlmostEqual(v.heading, expected, places=6)
        for invalid in (65535, 36000):
            h.heading=invalid; h.drive(.15)
            self.assertFalse(h.values['heading'][-1].valid)
            self.assertTrue(math.isnan(h.values['heading'][-1].heading))

    def test_delayed_sync_rejected(self):
        h=self.h; h.sync_delay=.04; h.drive(1.5)
        self.assertFalse(h.values['imu'])
        self.assertGreater(int(h.status()['rejected_sync']), 0)
        h.sync_delay=0; h.pending_sync.clear(); h.drive(1.5)
        self.assertTrue(h.values['imu'])

    def test_extended_device_time(self):
        h=self.h
        # Start just before the historical 32-bit microsecond boundary.
        h.epoch=time.monotonic() - (2**32/1e6 - .8)
        h.drive(2)
        self.assertGreater(len(h.values['imu']), 100)
        stamps=[v.header.stamp.sec+v.header.stamp.nanosec*1e-9 for v in h.values['imu']]
        self.assertTrue(all(a<b for a,b in zip(stamps, stamps[1:])))
        self.assertEqual(h.status()['synchronized'], 'true')



class LaunchTests(unittest.TestCase):
    def test_dedicated_device_and_hardware_only(self):
        from launch import LaunchContext
        path = Path(__file__).resolve().parents[1] / 'launch/flight.launch.py'
        spec = importlib.util.spec_from_file_location('flight_launch_under_test', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        context.launch_configurations.update(module.FLIGHT_CONFIG)
        context.launch_configurations.update({'mavlink_enabled': 'true', 'mode': 'sitl'})
        with self.assertRaisesRegex(ValueError, 'hardware'):
            module.nodes(context)
        context.launch_configurations.update({'mode': 'hardware', 'mavlink_device': ''})
        with self.assertRaisesRegex(ValueError, 'specified'):
            module.nodes(context)
        with tempfile.TemporaryDirectory() as directory:
            original = Path(directory) / 'uart'
            alias = Path(directory) / 'alias'
            original.touch(); alias.symlink_to(original)
            context.launch_configurations.update({'device': str(original), 'mavlink_device': str(alias)})
            with self.assertRaisesRegex(ValueError, 'different'):
                module.nodes(context)


if __name__ == '__main__': unittest.main()
