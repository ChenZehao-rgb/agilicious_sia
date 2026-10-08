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
import yaml
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from pymavlink.dialects.v20 import common as mav
from sensor_msgs.msg import FluidPressure, Imu, NavSatFix, Temperature
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from diagnostic_msgs.msg import DiagnosticArray
from agi_ros2.msg import Barometer, Heading, ImuTiming, Navigation


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
        self.streaming_baro = True
        self.fix_type = 3
        self.heading = 0
        self.unknown = False
        self.invalid_velocity = False
        self.freeze_gps = False
        self.gps_time = 0
        self.commands = []
        self.ack_result_by_id = {}
        self.next_imu = self.next_gps = self.next_attitude = self.next_baro = 0
        self.values = {k: [] for k in ('imu', 'imu_timing', 'fix', 'velocity', 'attitude', 'heading', 'navigation',
                                     'pressure', 'temperature', 'barometer', 'status')}
        self.node = rclpy.create_node('test_' + uuid.uuid4().hex)
        self.namespace = '/mavtest_' + uuid.uuid4().hex
        topics = [('imu', Imu, 'sensors/imu'), ('fix', NavSatFix, 'sensors/gps/fix'),
                  ('imu_timing', ImuTiming, 'sensors/imu/timing'),
                  ('velocity', TwistStamped, 'sensors/gps/velocity'),
                  ('attitude', QuaternionStamped, 'sensors/fc_attitude'),
                  ('heading', Heading, 'sensors/fc_heading'),
                  ('navigation', Navigation, 'sensors/navigation'),
                  ('pressure', FluidPressure, 'sensors/baro/pressure'),
                  ('temperature', Temperature, 'sensors/baro/temperature'),
                  ('barometer', Barometer, 'sensors/baro/sample'),
                  ('status', DiagnosticArray, 'sensors/mavlink/status')]
        self.subs = [self.node.create_subscription(typ, self.namespace + '/' + topic,
                     lambda m, k=key: self.values[k].append(m),
                     100 if key == 'barometer' else 10 if key == 'status' else qos_profile_sensor_data)
                     for key, typ, topic in topics]
        binary = os.environ.get('AGI_MAVLINK_SENSOR_BINARY',
                                str(Path(get_package_prefix('agi_ros2')) / 'lib/agi_ros2/mavlink_sensor_node'))
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

    def baro(self, stamp_ms=None, pressure_hpa=1013.25, temperature_cdeg=2500):
        if stamp_ms is None:
            stamp_ms = self.remote() // 1000
        self.peer.scaled_pressure_send(stamp_ms & 0xffffffff, pressure_hpa, 0., temperature_cdeg)

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
                    result = self.ack_result_by_id.get(int(m.param1), mav.MAV_RESULT_ACCEPTED)
                    self.peer.command_ack_send(m.command, result, target_system=245, target_component=191)
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
                if self.streaming_baro and t >= self.next_baro:
                    self.baro()
                    self.next_baro = t + .025
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
        samples = {(v.header.stamp.sec, v.header.stamp.nanosec) for v in h.values['imu']}
        self.assertTrue(h.values['imu_timing'])
        for timing in h.values['imu_timing']:
            self.assertIn((timing.header.stamp.sec, timing.header.stamp.nanosec), samples)
            self.assertEqual(timing.clock_id, Path('/proc/sys/kernel/random/boot_id').read_text().strip())
            self.assertGreater(timing.fc_time_usec, 0)
            self.assertLessEqual(timing.receive_steady_time, timing.published_steady_time)
            self.assertLessEqual(timing.published_steady_time, timing.publish_return_steady_time)
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
        self.assertEqual({int(c[1]) for c in h.commands}, {24, 29, 30, 33, 105})
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

    def test_atomic_navigation_preserves_quality_and_session(self):
        h = self.h
        h.drive(1.8)
        self.assertTrue(h.values['navigation'])
        n = h.values['navigation'][-1]
        self.assertEqual(n.fix_type, 3)
        self.assertTrue(n.clock_aligned)
        self.assertEqual(n.altitude_reference, 'unknown')
        self.assertTrue(math.isnan(n.altitude))
        self.assertAlmostEqual(n.velocity.x, 2.)
        self.assertAlmostEqual(n.velocity.y, 1.)
        self.assertAlmostEqual(n.heading, math.pi/2)
        self.assertAlmostEqual(n.horizontal_accuracy, .5)
        self.assertTrue(n.source_session)
        session = n.source_session
        previous = len(h.values['navigation'])
        h.invalid_velocity = True
        h.drive(.4)
        failures = h.values['navigation'][previous:]
        self.assertTrue(failures)
        self.assertTrue(all(not n.heading_valid and math.isnan(n.velocity.x) for n in failures))
        h.invalid_velocity = False
        h.epoch = time.monotonic() - 2
        h.drive(2.)
        self.assertNotEqual(h.values['navigation'][-1].source_session, session)

    def test_bad_fix_and_expired_clock_publish_revocation(self):
        h = self.h
        h.drive(1.6)
        previous = len(h.values['navigation'])
        h.fix_type = 1
        h.drive(.15)
        self.assertTrue(any(n.fix_type == 1 and not n.heading_valid for n in h.values['navigation'][previous:]))
        h.fix_type = 3
        h.sync = False
        h.drive(2.3)
        self.assertFalse(h.values['navigation'][-1].clock_aligned)
        self.assertTrue(math.isnan(h.values['navigation'][-1].latitude))

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

    def test_barometer_units_stamps_variance_and_rate_ack(self):
        self.h.close()
        self.h = Harness(baro_pressure_variance_pa2=16.0, baro_temperature_variance_c2=.09)
        h = self.h
        h.drive(1.8)
        self.assertGreater(len(h.values['pressure']), 25)
        self.assertIn((mav.MAV_CMD_SET_MESSAGE_INTERVAL, 29., 25000.), h.commands)
        self.assertEqual(h.status()['rate_commands_ok'], 'true')
        pressure = h.values['pressure'][-2]
        temperature = next(v for v in h.values['temperature'] if v.header.stamp == pressure.header.stamp)
        sample = next(v for v in h.values['barometer'] if v.valid and v.header.stamp == pressure.header.stamp)
        self.assertEqual(pressure.header.frame_id, 'baro_link')
        self.assertEqual(pressure.fluid_pressure, 101325.)
        self.assertEqual(pressure.variance, 16.)
        self.assertEqual(temperature.temperature, 25.)
        self.assertEqual(temperature.variance, .09)
        self.assertEqual(sample.pressure_pa, pressure.fluid_pressure)
        self.assertEqual(sample.pressure_variance, pressure.variance)
        self.assertTrue(sample.clock_aligned)
        self.assertEqual(sample.source_session, h.values['navigation'][-1].source_session)
        stamp = pressure.header.stamp.sec + pressure.header.stamp.nanosec * 1e-9
        self.assertLess(abs(time.time() - stamp), .2)

    def test_barometer_disabled_and_rejected_rate_ack(self):
        self.h.close()
        self.h = Harness(baro_rate_hz=0)
        h = self.h
        h.ack_result_by_id[29] = mav.MAV_RESULT_UNSUPPORTED
        h.drive(1.8)
        self.assertIn((mav.MAV_CMD_SET_MESSAGE_INTERVAL, 29., -1.), h.commands)
        self.assertEqual(h.status()['rate_commands_ok'], 'false')
        self.assertFalse(h.values['pressure'])
        self.assertFalse(h.values['temperature'])
        self.assertFalse(any(v.valid for v in h.values['barometer']))
        self.assertGreater(len(h.values['imu']), 100)

    def test_barometer_dedup_age_and_timeout(self):
        h = self.h
        h.drive(1.6)
        h.streaming_baro = False
        h.drive(.04)
        before = len(h.values['pressure'])
        stamp = h.remote() // 1000
        h.baro(stamp)
        h.baro(stamp)
        h.drive(.04)
        self.assertEqual(len(h.values['pressure']), before + 1)
        # This age passes the existing 0.5 s GPS gate, but must fail the 0.25 s barometer gate.
        h.baro(h.remote() // 1000 - 300)
        h.drive(.35)
        self.assertEqual(len(h.values['pressure']), before + 1)
        failures = [v for v in h.values['barometer'] if v.reason == 'Pressure sample timeout']
        self.assertEqual(len(failures), 1)
        self.assertFalse(failures[0].valid)
        self.assertTrue(failures[0].clock_aligned)
        self.assertTrue(math.isnan(failures[0].pressure_pa))
        count = len(h.values['barometer'])
        h.drive(1.1)
        self.assertEqual(len(h.values['barometer']), count)
        self.assertEqual(h.status()['baro_valid'], 'false')
        self.assertGreater(int(h.status()['baro_stale']), 0)
        self.assertGreater(int(h.status()['baro_duplicates']), 0)
        h.baro()
        h.drive(.04)
        self.assertTrue(h.values['barometer'][-1].valid)

    def test_barometer_invalid_pressure_recovers(self):
        h = self.h
        h.drive(1.6)
        h.streaming_baro = False
        h.drive(.04)
        before = len(h.values['pressure'])
        for value in (float('nan'), float('inf'), 0., -1.):
            h.baro(pressure_hpa=value)
            h.drive(.02)
        self.assertEqual(len(h.values['pressure']), before)
        self.assertFalse(h.values['barometer'][-1].valid)
        self.assertEqual(h.values['barometer'][-1].reason, 'Invalid absolute pressure')
        h.baro(pressure_hpa=1012.25)
        h.drive(.04)
        self.assertTrue(h.values['barometer'][-1].valid)
        self.assertEqual(h.values['pressure'][-1].fluid_pressure, 101225.)

    def test_barometer_millisecond_wrap_and_epoch_revocation(self):
        h = self.h
        h.epoch = time.monotonic() - (2**32 / 1000 - 1.)
        h.drive(2.)
        stamps = [v.header.stamp.sec + v.header.stamp.nanosec * 1e-9 for v in h.values['pressure']]
        self.assertGreater(len(stamps), 30)
        self.assertTrue(all(a < b for a, b in zip(stamps, stamps[1:])))
        self.assertLess(max(b - a for a, b in zip(stamps, stamps[1:])), .1)
        session = h.values['barometer'][-1].source_session
        before = len(h.values['barometer'])
        h.epoch = time.monotonic() - 1.
        h.drive(1.5)
        samples = h.values['barometer'][before:]
        reset = next(v for v in samples if not v.valid and v.reason == 'Flight controller restarted')
        self.assertFalse(reset.clock_aligned)
        self.assertNotEqual(reset.source_session, session)
        self.assertTrue(h.values['barometer'][-1].valid)
        self.assertEqual(h.values['barometer'][-1].source_session, reset.source_session)
        h.sync = False
        h.drive(2.3)
        self.assertFalse(h.values['barometer'][-1].valid)
        self.assertFalse(h.values['barometer'][-1].clock_aligned)
        self.assertEqual(h.values['barometer'][-1].reason, 'Time synchronization expired')

    def test_barometer_publisher_collision_revokes_source(self):
        h = self.h
        h.drive(1.6)
        collision = h.node.create_publisher(FluidPressure, h.namespace + '/sensors/baro/pressure', qos_profile_sensor_data)
        try:
            h.drive(1.2)
            self.assertEqual(h.status()['baro_publisher_conflict'], 'true')
            self.assertEqual(h.values['barometer'][-1].reason, 'Sensor publisher conflict')
            count = len(h.values['pressure'])
            h.drive(.15)
            self.assertEqual(len(h.values['pressure']), count)
        finally:
            h.node.destroy_publisher(collision)
        h.drive(1.2)
        self.assertTrue(h.values['barometer'][-1].valid)



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
        with tempfile.TemporaryDirectory() as directory:
            # An empty override keeps the profile value; make the profile device absent explicitly.
            _, profile = module._runtime.load_profile('hardware')
            profile['mavlink']['device'] = ''
            runtime_config = Path(directory) / 'hardware.yaml'
            runtime_config.write_text(yaml.safe_dump(profile))
            context.launch_configurations['runtime_config'] = str(runtime_config)
            with self.assertRaisesRegex(ValueError, 'specified'):
                module.nodes(context)
            original = Path(directory) / 'uart'
            alias = Path(directory) / 'alias'
            original.touch(); alias.symlink_to(original)
            context.launch_configurations.update({'device': str(original), 'mavlink_device': str(alias)})
            with self.assertRaisesRegex(ValueError, 'different'):
                module.nodes(context)


if __name__ == '__main__': unittest.main()
