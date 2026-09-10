#!/usr/bin/env python3
"""Process-level tests using synthetic sensors, loopback UDP and a pseudo-UART.

Run after build.sh from a shell sourcing ROS 2 and install/agi_ros2/local_setup.bash.
No physical flight controller is opened and no Gazebo process is needed.
"""
import math
import os
from pathlib import Path
import pty
import signal
import socket
import struct
import subprocess
import tempfile
import time
import unittest
import uuid

import rclpy
from rclpy.qos import qos_profile_sensor_data
from agi_ros2.msg import Authority, ControlCommand, FusedState, Health, OutputStatus, Rtk
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

ROOT = Path(__file__).resolve().parents[2]
INSTALL = ROOT / 'install/agi_ros2'
BIN = INSTALL / 'lib/agi_ros2'
PARAMS = INSTALL / 'share/agi_ros2/params'
CLOCK_ID = Path('/proc/sys/kernel/random/boot_id').read_text().strip()


class Harness:
    def __init__(self):
        self.temp = tempfile.TemporaryDirectory(prefix='agi_split_test_')
        self.path = Path(self.temp.name)
        self.namespace = '/test_' + uuid.uuid4().hex
        self.node = rclpy.create_node('driver', namespace=self.namespace)
        self.processes = []
        self.logs = []
        self.pubs = {}
        self.subs = []
        self.received = {}
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(('127.0.0.1', 0))
        self.udp.setblocking(False)
        config = (PARAMS / 'betaflight_udp.yaml').read_text()
        import re
        config = re.sub(r'(?m)^(port:\s*)\d+',
                        lambda m: m[1] + str(self.udp.getsockname()[1]), config)
        self.bridge = self.path / 'bridge.yaml'
        self.bridge.write_text(config)
        self.master = None
        self.slave = None
        self.serial_bytes = bytearray()
        self.packets = []
        self.sequence = 0
        self.armed = True
        self.auto = False
        self.kill = False
        self.command_valid = True
        self.command_age = 0.0
        self.command_clock = CLOCK_ID
        self.total_thrust = 10.0
        self.body_rates = (0.1, 0.2, -0.3)
        self.rtk_enabled = True
        self.last_rtk = 0.0
        self.last_rc = 0.0
        self.last_command = 0.0
        self.imu_enabled = True
        self.last_health = 0.0
        self.last_imu = 0.0
        self.simulation = False
        self.sim_time_ns = 10_000_000_000

    def start(self, executable, hardware=False):
        args = [str(BIN / executable), '--ros-args', '-r', '__ns:=' + self.namespace,
                '-p', 'params_dir:=' + str(PARAMS)]
        if self.simulation:
            args += ['-p', 'use_sim_time:=true', '-r', '/clock:=' + self.namespace + '/clock']
        if executable != 'state_fusion_node':
            args += ['-p', 'mode:=' + ('hardware' if hardware else 'sitl')]
        if executable == 'command_output_node':
            args += ['-p', 'bridge_config:=' + str(self.bridge)]
            if hardware:
                self.master, self.slave = pty.openpty()
                os.set_blocking(self.master, False)
                table = self.path / 'thrust.csv'
                table.write_text('0,1000,1500,2000\n12,0,10,20\n18,0,20,40\n')
                args += ['-p', 'device:=' + os.ttyname(self.slave),
                         '-p', 'thrust_table:=' + str(table)]
        log = open(self.path / (executable + '.log'), 'w+')
        proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
        self.processes.append(proc)
        self.logs.append(log)
        return proc

    def publisher(self, topic, cls, sensor=False):
        if topic not in self.pubs:
            self.pubs[topic] = self.node.create_publisher(
                cls, topic, qos_profile_sensor_data if sensor else 10)
        return self.pubs[topic]

    def subscribe(self, topic, cls):
        self.received[topic] = []
        self.subs.append(self.node.create_subscription(
            cls, topic, lambda m: self.received[topic].append(m), 100))

    def stamp(self):
        if self.simulation:
            return Time(sec=self.sim_time_ns // 1_000_000_000,
                        nanosec=self.sim_time_ns % 1_000_000_000)
        return self.node.get_clock().now().to_msg()

    def publish_inputs(self, sensors, commands):
        wall = self.sim_time_ns / 1e9 if self.simulation else time.monotonic()
        if wall - self.last_rc >= 0.02:
            rc = Authority()
            rc.header.stamp = self.stamp()
            rc.armed, rc.auto_switch, rc.kill, rc.rc_link = self.armed, self.auto, self.kill, True
            rc.manual_aetr = [1500, 1500, 1000, 1500]
            self.publisher('authority', Authority).publish(rc)
            self.last_rc = wall
        if wall - self.last_health >= 0.01:
            health = Health()
            health.header.stamp = self.stamp()
            for field in ('imu_calibrated', 'converged', 'config_verified',
                          'thrust_calibrated', 'geofence_ok', 'transport_healthy'):
                setattr(health, field, True)
            health.battery_voltage = 16.0
            self.publisher('health', Health).publish(health)
            self.last_health = wall
        if sensors and self.rtk_enabled and wall - self.last_rtk >= 0.1:
            fix = Rtk()
            fix.header.stamp = self.stamp()
            fix.header.frame_id = 'odom'
            fix.position.z = 3.0
            fix.fixed = fix.heading_valid = fix.accuracy_ok = fix.synchronized = True
            self.publisher('sensors/rtk', Rtk).publish(fix)
            self.last_rtk = wall
        if sensors and self.imu_enabled and wall - self.last_imu >= 0.002:
            imu = Imu()
            imu.header.stamp = self.stamp()
            imu.header.frame_id = 'base_link'
            imu.linear_acceleration.z = 9.8066
            self.publisher('sensors/imu', Imu, True).publish(imu)
            self.last_imu = wall
        if commands and wall - self.last_command >= 0.01:
            command = ControlCommand()
            command.header.stamp = self.stamp()
            command.header.frame_id = 'base_link'
            command.clock_id = self.command_clock + (':ros' if self.simulation else '')
            self.sequence += 1
            command.sequence = self.sequence
            command.total_thrust = self.total_thrust
            command.body_rates.x, command.body_rates.y, command.body_rates.z = self.body_rates
            command.permit_override = self.auto
            command.mode = 4 if self.auto else 3
            evidence = command.evidence
            for field in ('now', 'imu_time', 'rtk_time', 'rc_time', 'command_time'):
                setattr(evidence, field, wall - self.command_age)
            evidence.solve_seconds = 0.001
            for field in ('rtk_fixed', 'heading_valid', 'accuracy_ok', 'imu_calibrated',
                          'synchronized', 'converged', 'config_verified',
                          'thrust_calibrated', 'geofence_ok', 'msp_healthy',
                          'controller_warm', 'rc_link'):
                setattr(evidence, field, True)
            evidence.command_valid = self.command_valid
            evidence.armed, evidence.auto_switch, evidence.kill = self.armed, self.auto, self.kill
            self.publisher('control_command', ControlCommand).publish(command)
            self.last_command = wall

    def drain(self):
        while True:
            try:
                data = self.udp.recv(128)
                self.packets.append(struct.unpack('<d16H', data))
            except BlockingIOError:
                break
        if self.master is not None:
            while True:
                try:
                    data = os.read(self.master, 8192)
                    if not data:
                        break
                    self.serial_bytes.extend(data)
                except BlockingIOError:
                    break

    def run(self, seconds, sensors=False, commands=False, rate=1.0, paused=False):
        start = time.monotonic()
        start_sim = self.sim_time_ns
        stop = start + seconds
        while time.monotonic() < stop:
            if self.simulation and not paused:
                # Quantized 1 ms physics steps, independently paced wall clock.
                self.sim_time_ns = start_sim + int((time.monotonic() - start) * rate * 1000) * 1_000_000
                clock = Clock(clock=self.stamp())
                self.publisher('clock', Clock).publish(clock)
            if not paused:
                self.publish_inputs(sensors, commands)
            rclpy.spin_once(self.node, timeout_sec=0.0005)
            self.drain()
            for proc, log in zip(self.processes, self.logs):
                if proc.poll() is not None:
                    log.seek(0)
                    raise AssertionError(log.read())

    def clear(self):
        self.drain()
        self.packets.clear()
        self.serial_bytes.clear()

    def close(self):
        for proc in self.processes:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        for proc in self.processes:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        for log in self.logs:
            log.close()
        self.node.destroy_node()
        self.udp.close()
        for fd in (self.master, self.slave):
            if fd is not None:
                os.close(fd)
        self.temp.cleanup()


class NodePipelineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.h = Harness()
        self.addCleanup(self.h.close)

    def test_fusion_control_output_and_sensor_loss(self):
        h = self.h
        for topic, cls in [('fused_state', FusedState), ('control_command', ControlCommand),
                           ('status', String), ('output_status', OutputStatus)]:
            h.subscribe(topic, cls)
        for executable in ('state_fusion_node', 'control_node', 'command_output_node'):
            h.start(executable)
        h.run(3.0, sensors=True)
        states = [s for s in h.received['fused_state'] if s.initialized]
        self.assertGreater(len(states), 200)
        self.assertAlmostEqual(states[-1].position.z, 3.0, places=3)
        self.assertAlmostEqual(states[-1].velocity.z, 0.0, places=3)
        self.assertAlmostEqual(states[-1].orientation.w, 1.0, places=3)
        self.assertTrue(any('AUTO_STANDBY' in s.data for s in h.received['status']))
        self.assertTrue(any(c.evidence.controller_warm for c in h.received['control_command']))
        h.auto = True
        h.run(0.35, sensors=True)
        active = [c for c in h.received['control_command'] if c.permit_override]
        self.assertTrue(active, [s.data for s in h.received['status'][-5:]])
        self.assertTrue(any(s.override_active for s in h.received['output_status']))
        self.assertTrue(all(math.isfinite(c.total_thrust) and c.total_thrust > 0 for c in active))
        h.imu_enabled = False
        h.run(0.15, sensors=True)
        self.assertFalse(h.received['control_command'][-1].permit_override)
        self.assertFalse(h.received['output_status'][-1].override_active)
        self.assertEqual(h.packets[-1][5], 1000)  # AUX1 disarm in SITL.
        h.imu_enabled = True
        h.run(0.8, sensors=True)
        self.assertFalse(h.received['control_command'][-1].permit_override)
        h.auto = False
        h.run(0.8, sensors=True)
        h.auto = True
        h.run(0.2, sensors=True)
        self.assertTrue(any(c.permit_override for c in h.received['control_command'][-15:]))

    def test_udp_deadlines_invalid_command_and_restart_high(self):
        h = self.h
        h.subscribe('output_status', OutputStatus)
        h.auto = True  # No healthy low edge after output process startup.
        h.start('command_output_node')
        h.run(0.8, commands=True)
        self.assertTrue(h.packets)
        self.assertTrue(all(p[5] == 1000 for p in h.packets))
        h.auto = False
        h.run(0.15, commands=True)
        h.auto = True
        h.run(0.1, commands=True)
        self.assertTrue(any(s.override_active for s in h.received['output_status']))
        self.assertEqual(h.packets[-1][5], 2000)
        self.assertGreater(h.packets[-1][2], 1500)  # Positive SITL pitch.
        h.run(0.06, commands=False)
        self.assertEqual(h.packets[-1][5], 1000)
        self.assertGreater(h.received['output_status'][-1].fault_count, 0)
        h.run(0.1, commands=True)
        self.assertFalse(h.received['output_status'][-1].override_active)
        h.auto = False
        h.run(0.1, commands=True)
        h.auto = True
        h.run(0.1, commands=True)
        self.assertTrue(h.received['output_status'][-1].override_active)
        h.total_thrust = float('nan')
        h.run(0.1, commands=True)
        self.assertFalse(h.received['output_status'][-1].override_active)
        self.assertEqual(h.packets[-1][5], 1000)
        h.total_thrust = 10.0
        h.auto = False
        h.run(0.1, commands=True)
        h.command_age = 0.1  # Receipt must not refresh stale producer evidence.
        h.auto = True
        h.run(0.15, commands=True)
        self.assertFalse(h.received['output_status'][-1].override_active)
        h.command_age = 0.0
        h.command_clock = 'different-host'
        h.run(0.1, commands=True)
        self.assertFalse(h.received['output_status'][-1].override_active)

    def test_msp_pseudo_uart_aetr_only_and_kill(self):
        h = self.h
        h.subscribe('output_status', OutputStatus)
        h.start('command_output_node', hardware=True)
        h.run(0.8, commands=True)
        h.clear()
        h.auto = True
        h.run(0.15, commands=True)
        self.assertTrue(h.received['output_status'][-1].override_active)
        data = bytes(h.serial_bytes)
        frames = []
        offset = 0
        while offset < len(data):
            self.assertEqual(data[offset:offset + 3], b'$M<')
            size, code = data[offset + 3:offset + 5]
            self.assertEqual((size, code), (8, 200))
            frame = data[offset:offset + size + 6]
            self.assertEqual(len(frame), size + 6)
            checksum = 0
            for byte in frame[3:-1]:
                checksum ^= byte
            self.assertEqual(checksum, frame[-1])
            frames.append(struct.unpack('<4H', frame[5:-1]))
            offset += size + 6
        self.assertTrue(frames)
        self.assertGreater(frames[-1][0], 1500)
        self.assertLess(frames[-1][1], 1500)  # FLU -> hardware FRD.
        self.assertGreater(frames[-1][3], 1500)
        h.kill = True
        h.run(0.08, commands=True)
        self.assertFalse(h.received['output_status'][-1].override_active)
        h.clear()
        h.run(0.1, commands=True)
        self.assertEqual(bytes(h.serial_bytes), b'')  # No synthetic failsafe/AUX.

    def start_simulated_pipeline(self):
        h = self.h
        h.simulation = True
        for topic, cls in [('control_command', ControlCommand), ('status', String),
                           ('output_status', OutputStatus), ('reference', Odometry)]:
            h.subscribe(topic, cls)
        for executable in ('state_fusion_node', 'control_node', 'command_output_node'):
            h.start(executable)
        h.run(2.5, sensors=True)
        self.assertTrue(h.received['control_command'][-1].evidence.controller_warm,
                        [s.data for s in h.received['status'][-5:]])
        self.rearm_simulation()
        return h

    def rearm_simulation(self):
        h = self.h
        h.auto = False
        h.run(0.8, sensors=True)
        h.auto = True
        h.run(0.15, sensors=True)
        self.assertTrue(h.received['output_status'][-1].override_active,
                        [s.data for s in h.received['status'][-5:]])

    def test_sim_clock_short_pause_slow_rate_and_long_stall(self):
        h = self.start_simulated_pipeline()
        target = h.received['reference'][-1].pose.pose.position
        fault_count = h.received['output_status'][-1].fault_count
        for _ in range(3):
            h.run(0.08, paused=True)  # Longer than the old 10/25 ms wall limits.
            self.assertTrue(h.received['output_status'][-1].override_active)
            h.run(0.15, sensors=True)
            self.assertTrue(h.received['control_command'][-1].permit_override)
            self.assertEqual(h.received['reference'][-1].pose.pose.position, target)
        self.assertEqual(h.received['output_status'][-1].fault_count, fault_count)
        begin = len(h.received['control_command'])
        h.run(1.0, sensors=True, rate=0.5)
        commands = h.received['control_command'][begin:]
        self.assertGreater(len(commands), 40)
        self.assertLess(len(commands), 60)  # 100 Hz simulated, not wall, time.
        self.assertTrue(all(c.permit_override for c in commands))
        stamps = [c.evidence.now for c in commands]
        self.assertTrue(all(b > a for a, b in zip(stamps, stamps[1:])))
        self.assertTrue(all(c.clock_id == CLOCK_ID + ':ros' for c in commands))
        h.run(0.35, paused=True)
        self.assertFalse(h.received['output_status'][-1].override_active)
        self.assertEqual(h.packets[-1][5], 1000)
        h.run(0.8, sensors=True)
        self.assertFalse(h.received['control_command'][-1].permit_override)
        self.rearm_simulation()

    def test_sim_clock_sensor_loss_control_stop_and_rewind(self):
        h = self.start_simulated_pipeline()
        h.imu_enabled = False
        h.run(0.06, sensors=True)  # /clock advances: actual IMU loss still fails.
        self.assertFalse(h.received['output_status'][-1].override_active)
        h.imu_enabled = True
        self.rearm_simulation()
        control = h.processes[1]
        control.send_signal(signal.SIGSTOP)
        try:
            h.run(0.08, sensors=True)  # /clock advances without new commands.
            self.assertFalse(h.received['output_status'][-1].override_active)
        finally:
            control.send_signal(signal.SIGCONT)
        self.rearm_simulation()
        h.sim_time_ns -= 2_000_000_000
        # New epoch publishers must immediately resume with lower stamps.
        h.last_rc = h.last_health = h.last_rtk = h.last_imu = 0.0
        h.run(0.8, sensors=True)
        self.assertFalse(h.received['output_status'][-1].override_active)
        self.rearm_simulation()
        # Safety authority must revoke even when /clock is frozen.
        rc = Authority(header=h.received['control_command'][-1].header,
                       armed=True, auto_switch=True, kill=True, rc_link=True,
                       manual_aetr=[1500, 1500, 1000, 1500])
        h.publisher('authority', Authority).publish(rc)
        h.run(0.03, paused=True)
        self.assertFalse(h.received['output_status'][-1].override_active)


if __name__ == '__main__':
    unittest.main(verbosity=2)
