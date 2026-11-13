"""MAVLink telemetry generation shared by the existing two-PTY integration harnesses."""
import math
import os
from pathlib import Path
import pty
import struct
import time

import yaml
from agi_ros2.msg import (FusedState, FcEstimatedState, FcEstimatorStatus, ComputationStatus,
                          ControlCommand, Health, OutputStatus)
from test_shadow_pipeline import ShadowHarness
from test_runtime_profile_nodes import dump_profile
from pymavlink.dialects.v20 import common as mav


def init_telemetry(h):
    h.next_telemetry = h.next_battery = 0.
    h.arm_request = None
    h.rc_arm_value = h.rc_auto_value = None
    h.disabled_telemetry = set()
    h.system_voltage_mv = 16000
    h.battery_voltages_mv = [16000] + [65535]*9
    h.battery_current_ca = 123
    h.freeze_rc = False
    h.rc_time = 0
    h.telemetry_commands = []


def send_telemetry(h, now):
    if now >= h.next_telemetry:
        h.next_telemetry = now + .04
        if 35 not in h.disabled_telemetry:
            if not h.freeze_rc:
                h.rc_time = (h.remote()//1000) & 0xffffffff
            arm = h.response_armed if h.arm_request is None else h.arm_request
            ch5 = h.rc_arm_value if h.rc_arm_value is not None else 2000 if arm else 1000
            ch8 = h.rc_auto_value if h.rc_auto_value is not None else 2000 if h.response_auto else 1000
            h.peer.rc_channels_raw_send(h.rc_time, 0, 1500, 1500, 1000, 1500, ch5, 1000, 1000, ch8, 0)
        if 0 not in h.disabled_telemetry:
            h.peer.heartbeat_send(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_GENERIC,
                                 128 if h.response_armed else 0, 0,
                                 mav.MAV_STATE_CRITICAL if h.response_failsafe else
                                 mav.MAV_STATE_ACTIVE if h.response_armed else mav.MAV_STATE_STANDBY)
        if 1 not in h.disabled_telemetry:
            h.peer.sys_status_send(3, 3, 3, 0, h.system_voltage_mv, 123, 100, 0, 0, 0, 0, 0, 0)
    if now >= h.next_battery and 147 not in h.disabled_telemetry:
        h.next_battery = now + .5
        h.peer.battery_status_send(0, 0, 0, 32767, h.battery_voltages_mv, h.battery_current_ca, -1, -1, -1)


class FcHarness(ShadowHarness):
    def __init__(self, controller='MPC', shadow=True):
        super().__init__()
        init_telemetry(self)
        self.mav_master, self.mav_slave = pty.openpty()
        os.set_blocking(self.mav_master, False)
        self.peer = mav.MAVLink(self, srcSystem=1, srcComponent=1)
        self.peer.robust_parsing = True
        self.epoch = time.monotonic() - 4300  # Beyond first micros32 wrap; protocol is uint64.
        self.next_state = self.next_health = 0.
        self.sequence = 0
        self.last_state = 0
        self.reset_count = 0
        self.flags = 255
        self.quality = 0
        self.xy_age = self.z_age = 10000
        self.pvar = .04
        self.vvar = .0025
        self.state_enabled = self.health_enabled = self.sync_enabled = True
        self.freeze = False
        self.state_delay_us = 2000  # Synthetic sample-to-wire delay; avoids zero-latency PTY clock ambiguity.
        self.health_reset_offset = 0
        self.health_time_override = None
        self.version = 1
        self.commands = []
        self.driver = self.drive_sensors
        for topic, cls in [('fused_state', FusedState), ('sensors/fc/estimated_state', FcEstimatedState),
                           ('sensors/fc/estimator_status', FcEstimatorStatus), ('computation_status', ComputationStatus),
                           ('control_command', ControlCommand), ('health', Health), ('output_status', OutputStatus)]:
            self.subscribe(topic, cls)
        source = Path(__file__).resolve().parents[1] / 'config/hardware.yaml'
        profile = yaml.safe_load(source.read_text())
        profile['bridge'] = yaml.safe_load(self.bridge.read_text())
        profile['flight']['shadow_only'] = shadow
        path = self.path / 'fc.yaml'
        path.write_text(dump_profile(profile))
        common = dict(runtime_config=str(path), controller=controller, shadow_only=shadow)
        self.start_node('command_output_node', device=os.ttyname(self.slave), **common)
        self.start_node('msp_evidence.py',
                        geofence_min=[-10., -10., -2.], geofence_max=[10., 10., 10.])
        self.mav_device = self.path / 'fc-uart'
        self.mav_device.symlink_to(os.ttyname(self.mav_slave))
        self.start_node('mavlink_sensor_node', device=str(self.mav_device), state_source='fc_fused')
        self.start_node('fc_state_adapter_node')
        self.start_node('control_node', **common)

    def write(self, data):
        os.write(self.mav_master, data[:3])
        os.write(self.mav_master, data[3:])

    def remote(self):
        return int((time.monotonic() - self.epoch) * 1e6)

    def send_state(self, stamp=None):
        self.peer.odometry_send(stamp or self.last_state, mav.MAV_FRAME_LOCAL_NED, mav.MAV_FRAME_BODY_FRD,
                                0., 0., -1., [math.sqrt(.5), 0., 0., math.sqrt(.5)], 0., 0., 0., 0., 0., 0.,
                                [math.nan]*21, [math.nan]*21, self.reset_count & 255,
                                mav.MAV_ESTIMATOR_TYPE_AUTOPILOT, self.quality)

    def drive_sensors(self):
        try:
            data = os.read(self.mav_master, 8192)
        except BlockingIOError:
            data = b''
        for msg in self.peer.parse_buffer(data) or []:
            if msg.get_type() == 'TIMESYNC' and self.sync_enabled:
                self.peer.timesync_send(self.remote()*1000, msg.ts1)
            elif msg.get_type() == 'COMMAND_LONG':
                self.commands.append(int(msg.param1))
                self.peer.command_ack_send(msg.command, mav.MAV_RESULT_ACCEPTED, target_system=245, target_component=191)
        t = time.monotonic()
        send_telemetry(self, t)
        if self.state_enabled and t >= self.next_state:
            if not self.freeze:
                self.last_state = self.remote() - self.state_delay_us
                self.sequence += 1
            self.send_state()
            self.next_state = max(self.next_state + .01, t + .001)
        if self.health_enabled and self.last_state and t >= self.next_health:
            payload = struct.pack('<BBHIIQIIffIB3x3f3f', self.version, 68, self.flags, self.sequence,
                                  self.reset_count + self.health_reset_offset,
                                  self.last_state if self.health_time_override is None else self.health_time_override,
                                  self.xy_age, self.z_age, .8, .8, 5, 3, *([self.pvar]*3), *([self.vvar]*3))
            self.peer.v2_extension_send(0, 0, 0, 0xBF01, list(payload + bytes(249-68)))
            self.next_health = max(self.next_health + .02, t + .001)

    def latest(self, topic):
        return self.received[topic][-1] if self.received[topic] else None

    def wait_warm(self, timeout=12):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.run(.1)
            c = self.latest('computation_status')
            if c and c.warm_cycles == 50:
                return
        raise AssertionError('FC warmup failed: ' + str(self.latest('computation_status')) +
                             '\nState: ' + str(self.latest('fused_state')))

    def reconnect(self):
        os.close(self.mav_master)
        os.close(self.mav_slave)
        self.mav_master, self.mav_slave = pty.openpty()
        os.set_blocking(self.mav_master, False)
        self.mav_device.unlink()
        self.mav_device.symlink_to(os.ttyname(self.mav_slave))

    def close(self):
        super().close()
        os.close(self.mav_master)
        os.close(self.mav_slave)

