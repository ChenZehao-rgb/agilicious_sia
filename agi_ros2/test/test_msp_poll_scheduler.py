#!/usr/bin/env python3
"""Hardware MSP monitor emits no queries and drains unsolicited ACKs using an owned PTY."""
import os
from pathlib import Path
import pty
import signal
import subprocess
import tempfile
import time
import unittest
import uuid

import rclpy
from agi_ros2.msg import MspEvent
from ament_index_python.packages import get_package_prefix


class MspDrainTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='agi_msp_drain_')
        self.master, self.slave = pty.openpty()
        os.set_blocking(self.master, False)
        self.node = rclpy.create_node('observer', namespace='/drain_' + uuid.uuid4().hex)
        self.events = []
        self.subscription = self.node.create_subscription(MspEvent, 'msp/events', self.events.append, 1000)
        binary = Path(get_package_prefix('agi_ros2')) / 'lib/agi_ros2/betaflight_msp_node'
        self.log = open(Path(self.temp.name) / 'node.log', 'w+')
        self.process = subprocess.Popen([str(binary), '--ros-args', '-r', '__ns:=' + self.node.get_namespace(),
                                        '-p', 'device:=' + os.ttyname(self.slave), '-p', 'mode:=monitor',
                                        '-p', 'msp.read_configuration:=true', '-p', 'msp.rc.rate_hz:=25.0'],
                                       stdout=self.log, stderr=self.log)
        self.run_for(.8)

    def tearDown(self):
        self.process.send_signal(signal.SIGINT)
        self.process.wait(timeout=4.)
        self.log.close()
        if self.master is not None:
            os.close(self.master)
        os.close(self.slave)
        self.node.destroy_node()
        self.temp.cleanup()

    def run_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.001)
            self.assertIsNone(self.process.poll())

    def assert_no_tx(self):
        try:
            sent = os.read(self.master, 8192)
        except BlockingIOError:
            sent = b''
        self.assertEqual(sent, b'')
        self.assertFalse(any(event.event == 'tx' for event in self.events))

    def test_stale_query_parameters_cannot_enable_traffic(self):
        self.run_for(.3)
        self.assert_no_tx()

    def test_ack_bursts_are_drained_without_waiting_or_transmitting(self):
        # 320 frames exceed the per-tick bound and the decoder's 512-byte read.
        ack = b'$M>\x00\xc8\xc8'
        for _ in range(10):
            os.write(self.master, ack*32)
            self.run_for(.02)
        self.run_for(.2)
        self.assertEqual(sum(event.event == 'ack' and event.code == 200 for event in self.events), 320)
        self.assert_no_tx()

    def test_negative_ack_and_disconnect_report_failures(self):
        os.write(self.master, b'$M!\x00\xc8\xc8')
        self.run_for(.1)
        self.assertTrue(any(event.event == 'error' and event.code == 200 for event in self.events))
        self.assert_no_tx()
        os.close(self.master)
        self.master = None
        self.run_for(.1)
        self.assertTrue(any(event.event == 'transport_error' for event in self.events))


if __name__ == '__main__':
    unittest.main()
