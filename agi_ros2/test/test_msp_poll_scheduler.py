#!/usr/bin/env python3
"""Regression tests for the built MSP monitor using only an owned pseudo-UART.

Source ROS and install/agi_ros2/local_setup.bash, then execute this file. No
physical serial port, controller, command-output node or rosbag is used.
"""
import collections
import functools
import math
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import tempfile
import time
import unittest
import uuid

import rclpy
from agi_ros2.msg import MspEvent
from ament_index_python.packages import get_package_prefix

from test_shadow_support import EXPECTED, MspEvidence, config_frames, status_frame


def crc8(data):
    checksum = 0
    for value in data:
        checksum ^= value
        for _ in range(8):
            checksum = ((checksum << 1) ^ 0xd5 if checksum & 128 else checksum << 1) & 255
    return checksum


class MonitorHarness:
    """Real monitor and delayed fake FC, isolated in a unique ROS namespace."""

    def __init__(self, domain_id, optional_gps=False):
        self.temp = tempfile.TemporaryDirectory(prefix='agi_msp_scheduler_')
        self.path = Path(self.temp.name)
        self.master, self.slave = pty.openpty()
        os.set_blocking(self.master, False)
        self.namespace = '/msp_scheduler_' + uuid.uuid4().hex
        self.node = rclpy.create_node('observer', namespace=self.namespace)
        self.events = []
        self.decoder = MspEvidence(EXPECTED)
        self.subscription = self.node.create_subscription(MspEvent, 'msp/events', self.on_event, 1000)
        self.log = open(self.path / 'monitor.log', 'w+')
        self.pending_bytes = bytearray()
        self.requests = []
        self.responses = []
        self.max_outstanding = 0
        self.next_service = 0.
        self.fault_code = None
        self.fault_request = None
        self.pause_on_code = None
        self.paused_request = None
        self.stopped = False
        self.config = config_frames()
        self.settings = {'msp_override_channels_mask': '15', 'msp_override_failsafe': 'OFF',
                         'msp_override_timeout_ms': '50'}
        params = dict(mode='monitor', device=os.ttyname(self.slave), baud=115200,
                      **{'msp.response_timeout_ms': 100., 'msp.read_configuration': True,
                         'msp.rc.rate_hz': 25., 'msp.status.rate_hz': 25., 'msp.battery.rate_hz': 2.,
                         'msp.attitude.enabled': False, 'msp.analog.enabled': False,
                         'msp.gps.enabled': optional_gps})
        executable = Path(get_package_prefix('agi_ros2')) / 'lib/agi_ros2/betaflight_msp_node'
        args = [str(executable), '--ros-args', '-r', '__ns:=' + self.namespace]
        for name, value in params.items():
            args += ['-p', name + ':=' + (str(value).lower() if isinstance(value, bool) else str(value))]
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain_id), ROS_LOCALHOST_ONLY='1')
        try:
            self.process = subprocess.Popen(args, env=env, stdout=self.log, stderr=subprocess.STDOUT)
        except Exception:
            self.close()
            raise

    def on_event(self, event):
        self.events.append(event)
        stamp = event.request_stamp.sec + event.request_stamp.nanosec * 1e-9
        self.decoder.accept(event.code, event.payload, stamp, event.session_id, event.request_name, event.event)

    def snapshot(self):
        return self.decoder.snapshot(self.node.get_clock().now().nanoseconds * 1e-9)

    def payload(self, code, request):
        if code == 0x3010:
            name = request.rstrip(b'\x00').decode('ascii')
            assert name in self.settings and b'=' not in request, request
            return f'{name} = {self.settings[name]}'.encode('ascii')
        assert not request, (code, request)
        if code == 105:
            return struct.pack('<7H', 1500, 1500, 1000, 1500, 1000, 1000, 1000)
        if code == 150:
            return status_frame(armed=False)
        if code == 130:
            battery = bytearray(11)
            battery[0] = 4
            struct.pack_into('<H', battery, 9, 1600)
            return battery
        return self.config.get(code, b'')

    def read_requests(self):
        while True:
            try:
                data = os.read(self.master, 8192)
                if not data:
                    break
                self.pending_bytes.extend(data)
            except BlockingIOError:
                break
        buffer = self.pending_bytes
        while len(buffer) >= 6:
            assert buffer[:3] in (b'$M<', b'$X<'), bytes(buffer[:12])
            v2 = buffer[1] == ord('X')
            header = 8 if v2 else 5
            if len(buffer) < header:
                return
            length = int.from_bytes(buffer[6:8], 'little') if v2 else buffer[3]
            code = int.from_bytes(buffer[4:6], 'little') if v2 else buffer[4]
            if len(buffer) < header + length + 1:
                return
            frame = bytes(buffer[:header + length + 1])
            del buffer[:len(frame)]
            checksum = crc8(frame[3:-1]) if v2 else functools.reduce(int.__xor__, frame[3:-1], 0)
            assert checksum == frame[-1], frame
            assert code != 200, 'Monitor must never transmit MSP_SET_RAW_RC'
            now = time.monotonic()
            request = dict(time=now, code=code, payload=frame[header:-1], v2=v2)
            self.requests.append(request)
            reply = self.payload(code, request['payload'])
            if v2:
                body = struct.pack('<BHH', 0, code, len(reply)) + reply
                response = b'$X>' + body + bytes((crc8(body),))
            else:
                body = bytes((len(reply), code)) + reply
                response = b'$M>' + body + bytes((functools.reduce(int.__xor__, body, 0),))
            # Normal replies consume 10 ms of FC service each; a startup burst
            # queues behind earlier requests instead of being answered instantly.
            self.next_service = max(now, self.next_service) + .010
            due = self.next_service
            if code == self.fault_code and self.fault_request is None:
                self.fault_request = request
                due += .250
            self.responses.append((due, response, request))
            self.max_outstanding = max(self.max_outstanding, len(self.responses))
            if code == self.pause_on_code and self.paused_request is None:
                self.paused_request = request
                self.process.send_signal(signal.SIGSTOP)
                self.stopped = True

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=.0005)
        self.read_requests()
        now = time.monotonic()
        ready = [item for item in self.responses if item[0] <= now]
        self.responses = [item for item in self.responses if item[0] > now]
        for _, response, request in ready:
            assert os.write(self.master, response) == len(response)
            request['reply_time'] = time.monotonic()
        if self.process.poll() is not None:
            self.log.seek(0)
            raise AssertionError('Monitor exited:\n' + self.log.read())

    def run(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.step()

    def wait_until(self, condition, timeout=4.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.step()
            if condition():
                return
        raise AssertionError('Timed out waiting for test condition; latest evidence: ' + repr(self.snapshot()))

    def resume(self):
        if self.stopped:
            self.process.send_signal(signal.SIGCONT)
            self.stopped = False

    def close(self):
        process = getattr(self, 'process', None)
        if process is not None and process.poll() is None:
            self.resume()
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        self.log.close()
        self.node.destroy_node()
        os.close(self.master)
        os.close(self.slave)
        self.temp.cleanup()


class MspPollSchedulerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Both ROS participants share an otherwise independent domain. Serial
        # configuration always names our pty.openpty() slave, never hardware.
        cls.domain_id = 150 + os.getpid() % 50
        rclpy.init(domain_id=cls.domain_id)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def harness(self, **kwargs):
        harness = MonitorHarness(self.domain_id, **kwargs)
        self.addCleanup(harness.close)
        return harness

    def test_serialized_queries_preserve_receiver_freshness_and_configuration(self):
        h = self.harness()
        h.run(4.)
        self.assertEqual(h.max_outstanding, 1, 'More than one telemetry query waited for an FC reply')
        self.assertTrue(h.requests)
        first = h.requests[0]['time']
        for code in (105, 150):
            times = [request['time'] for request in h.requests if request['code'] == code]
            self.assertGreater(len(times), 60, (code, len(times)))
            self.assertLess(times[0] - first, .1, f'Code {code} starved at startup')
            gap = max(b - a for a, b in zip(times, times[1:]))
            self.assertLess(gap, .1, f'Code {code} exceeded the receiver freshness budget')
            rate = (len(times) - 1) / (times[-1] - times[0])
            self.assertGreater(rate, 20., (code, rate))
            self.assertLess(rate, 30., (code, rate))
            print(f'  code {code}: {rate:.2f} Hz, largest gap {gap * 1000:.1f} ms', flush=True)
        counts = collections.Counter(request['code'] for request in h.requests)
        for code in MspEvidence.CONFIG_CODES:
            self.assertGreaterEqual(counts[code], 2, (code, counts))
        self.assertGreaterEqual(counts[130], 5)
        setting_counts = collections.Counter(request['payload'].rstrip(b'\x00').decode('ascii')
                                             for request in h.requests if request['code'] == 0x3010)
        for setting in MspEvidence.OVERRIDE_SETTINGS:
            self.assertGreaterEqual(setting_counts[setting], 2, setting_counts)
        self.assertTrue(all(request['v2'] for request in h.requests if request['code'] == 0x3010))
        self.assertTrue(h.events, 'No MSP events discovered')
        failures = [(event.event, event.code) for event in h.events
                    if event.event not in ('tx', 'rx') or event.errors]
        self.assertEqual(failures, [])
        for setting in MspEvidence.OVERRIDE_SETTINGS:
            self.assertTrue(any(event.event == 'rx' and event.request_name == setting for event in h.events), setting)
        self.assertTrue(h.snapshot()['config_verified'], h.snapshot())
        self.assertTrue(h.snapshot()['transport_healthy'], h.snapshot())
        self.assertFalse(any(event.code == 200 for event in h.events))

    def assert_timeout_isolated(self, code, critical, optional_gps=False):
        h = self.harness(optional_gps=optional_gps)
        h.wait_until(lambda: h.snapshot()['config_verified'] and h.snapshot()['transport_healthy'])
        event_start = len(h.events)
        h.fault_code = code
        h.wait_until(lambda: h.fault_request is not None)
        h.run(2.2)
        fault = h.fault_request
        subsequent = [request for request in h.requests if request['time'] >= fault['time']]
        self.assertEqual(sum(request['code'] == code for request in subsequent), 1,
                         'Timed-out code was reused, allowing a delayed response to match a newer request')
        events = h.events[event_start:]
        timeout_kind = 'timeout' if critical else 'optional_timeout'
        timeouts = [event for event in events if event.code == code and event.event == timeout_kind]
        late = [event for event in events if event.code == code and event.event == 'late']
        self.assertEqual(len(timeouts), 1, [(event.event, event.code) for event in events])
        self.assertEqual(len(late), 1, [(event.event, event.code) for event in events])
        self.assertLess(timeouts[0].steady_time, late[0].steady_time)
        self.assertGreater(timeouts[0].request_steady_time, 0.)
        self.assertEqual(late[0].request_steady_time, 0.)
        self.assertEqual(late[0].request_stamp.sec, 0)
        self.assertEqual(late[0].request_stamp.nanosec, 0)
        self.assertEqual(late[0].request_name, '')
        self.assertTrue(math.isnan(late[0].latency_seconds))
        self.assertTrue(all(event.errors >= 1 for event in events if event.steady_time >= timeouts[0].steady_time))
        self.assertFalse(any(event.event == 'rx' and event.code == code
                             and event.steady_time > timeouts[0].steady_time for event in events))
        other_rx = [event for event in events if event.event == 'rx' and event.code != code
                    and event.steady_time > late[0].steady_time]
        self.assertGreater(len(other_rx), 20, 'Unrelated queries did not continue after timeout and late reply')
        self.assertEqual(h.snapshot()['transport_healthy'], not critical, h.snapshot())
        self.assertEqual(h.decoder.failed, critical)
        self.assertFalse(any(request['code'] == 200 for request in h.requests))
        if code == 0x3010:
            self.assertEqual(timeouts[0].request_name, fault['payload'].rstrip(b'\x00').decode('ascii'))

    def test_critical_receiver_timeout_latches_unhealthy_and_quarantines_late_reply(self):
        self.assert_timeout_isolated(105, critical=True)

    def test_setting_timeout_disables_all_requests_sharing_the_v2_code(self):
        self.assert_timeout_isolated(0x3010, critical=True)

    def test_optional_timeout_keeps_other_telemetry_healthy(self):
        self.assert_timeout_isolated(106, critical=False, optional_gps=True)

    def test_reply_buffered_during_monitor_pause_is_expired_before_receive(self):
        h = self.harness()
        h.wait_until(lambda: h.snapshot()['config_verified'] and h.snapshot()['transport_healthy'])
        event_start = len(h.events)
        h.pause_on_code = 105
        h.wait_until(lambda: h.stopped)
        request = h.paused_request
        # The fake FC still answers after its normal 10 ms. Only this test's
        # monitor is stopped, so the response waits in its own PTY receive queue
        # until the original request has already exceeded the 100 ms deadline.
        h.run(.180)
        self.assertIn('reply_time', request)
        self.assertLess(request['reply_time'] - request['time'], .1)
        h.resume()
        h.run(1.2)
        subsequent = [item for item in h.requests if item['time'] >= request['time']]
        self.assertEqual(sum(item['code'] == 105 for item in subsequent), 1)
        events = h.events[event_start:]
        timeouts = [event for event in events if event.code == 105 and event.event == 'timeout']
        late = [event for event in events if event.code == 105 and event.event == 'late']
        self.assertEqual(len(timeouts), 1, [(event.event, event.code) for event in events])
        self.assertEqual(len(late), 1, [(event.event, event.code) for event in events])
        self.assertLess(timeouts[0].steady_time, late[0].steady_time)
        self.assertGreaterEqual(timeouts[0].steady_time - timeouts[0].request_steady_time, .1)
        self.assertEqual(late[0].request_steady_time, 0.)
        self.assertEqual(late[0].request_name, '')
        self.assertTrue(math.isnan(late[0].latency_seconds))
        self.assertFalse(any(event.code == 105 and event.event == 'rx'
                             and event.request_steady_time == timeouts[0].request_steady_time for event in events))
        self.assertTrue(any(event.event == 'rx' and event.code != 105
                            and event.steady_time > late[0].steady_time for event in events))
        self.assertTrue(h.decoder.failed)
        self.assertFalse(h.snapshot()['transport_healthy'], h.snapshot())


if __name__ == '__main__':
    unittest.main(verbosity=2)
