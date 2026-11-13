#!/usr/bin/env python3
"""Actual hardware output node and MAVLink receiver, using two PTYs only."""
import math
import os
import time
import unittest

import rclpy
from agi_ros2.msg import Authority, FcTelemetry, MspEvent
from test_mavlink_telemetry_support import FcHarness


class HardwareMavlinkOverrideTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.h = FcHarness('GEO', shadow=False)
        self.addCleanup(self.h.close)
        for topic, kind in [('authority', Authority), ('sensors/fc/telemetry', FcTelemetry), ('msp/events', MspEvent)]:
            self.h.subscribe(topic, kind)
        self.h.wait_warm()
        self.h.run(.1)

    def wait_for(self, predicate, timeout=4.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.h.run(.02)
            if predicate():
                return
        self.fail('Condition not reached: ' + str(self.h.latest('output_status')) + '\n' +
                  str(self.h.latest('health')) + '\n' + str(self.h.latest('computation_status')))

    def active(self):
        return bool(self.h.latest('output_status') and self.h.latest('output_status').override_active)

    def authorize(self):
        self.h.response_auto = False
        self.h.wait_warm()
        self.h.run(.12)
        self.h.response_auto = True
        self.wait_for(self.active)

    def assert_only_override(self):
        self.assertTrue(self.h.codes)
        self.assertEqual({code for _, code, _ in self.h.codes}, {200})
        self.assertEqual(self.h.node.count_publishers('authority'), 1)
        self.assertEqual(self.h.node.count_publishers('health'), 1)
        self.assertFalse(self.h.latest('health').config_verified)

    def test_actual_armed_arm_request_auto_exit_and_ack_drain(self):
        h = self.h
        self.assertEqual(h.codes, [])
        self.authorize()
        h.run(1.)
        self.assertGreater(sum(m.event == 'ack' for m in h.received['msp/events']), 20)
        h.arm_request = False
        self.wait_for(lambda: not self.active(), .2)
        self.assertTrue(h.latest('authority').armed)
        self.assertFalse(h.latest('authority').arm_switch)
        h.arm_request = True
        h.run(.8)
        self.assertFalse(self.active())
        self.authorize()
        h.response_armed = False
        self.wait_for(lambda: not self.active(), .2)
        self.assertFalse(h.latest('authority').armed)
        self.assertTrue(h.latest('authority').arm_switch)
        h.response_armed = True
        h.run(.8)
        self.assertFalse(self.active())
        self.authorize()
        h.response_kill = True  # A nonexistent CH9 input cannot affect this MAVLink path.
        h.run(.15)
        self.assertTrue(self.active())
        h.response_auto = False
        self.wait_for(lambda: not self.active(), .2)
        self.assert_only_override()

    def test_each_mode_stream_and_frozen_rc_revoke_until_new_low(self):
        h = self.h
        for message_id in (35, 0, 1, 'frozen_rc'):
            with self.subTest(source=message_id):
                self.authorize()
                if message_id == 'frozen_rc':
                    h.freeze_rc = True
                else:
                    h.disabled_telemetry.add(message_id)
                h.run(.7)
                self.assertFalse(self.active())
                self.assertFalse(h.latest('authority').receiver_valid)
                self.assertTrue(h.latest('authority').auto_switch)
                h.disabled_telemetry.clear()
                h.freeze_rc = False
                h.run(.8)
                self.assertFalse(self.active())
        self.assert_only_override()

    def test_battery_loss_reboot_and_sync_loss_never_resume_high(self):
        h = self.h
        self.authorize()
        h.disabled_telemetry.add(147)
        h.system_voltage_mv = 0
        h.run(1.7)
        self.assertTrue(math.isnan(h.latest('health').battery_voltage))
        self.assertFalse(self.active())
        h.disabled_telemetry.clear()
        h.system_voltage_mv = 16000
        h.run(.8)
        self.assertFalse(self.active())
        self.authorize()
        session = h.latest('authority').session_id
        h.epoch = time.monotonic() - 10
        h.run(1.5)
        self.assertNotEqual(h.latest('authority').session_id, session)
        self.assertFalse(self.active())
        self.authorize()
        h.sync_enabled = False
        h.run(2.2)
        self.assertFalse(self.active())
        h.sync_enabled = True
        h.run(1.5)
        self.assertFalse(self.active())
        self.assert_only_override()

    def test_negative_ack_latches_fault(self):
        h = self.h
        self.authorize()
        h.response_error_ack = True
        self.wait_for(lambda: not self.active(), .3)
        h.response_error_ack = False
        h.response_auto = False
        h.run(.8)
        h.response_auto = True
        h.run(.3)
        self.assertFalse(self.active())
        self.assertFalse(h.latest('output_status').transport_healthy)
        self.assert_only_override()

    def test_uart_disconnect_latches_fault(self):
        self.authorize()
        os.close(self.h.master)
        self.h.master = None
        self.wait_for(lambda: not self.active(), .3)
        self.assertFalse(self.h.latest('output_status').transport_healthy)
        self.assert_only_override()


if __name__ == '__main__':
    unittest.main()
