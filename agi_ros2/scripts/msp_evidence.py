#!/usr/bin/env python3
"""Decode matched MSP readbacks and assemble honest authority/health evidence."""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from builtin_interfaces.msg import Time
from std_msgs.msg import String
from agi_ros2.msg import MspEvent, MspState, Authority, Health, OutputStatus, FusedState
import yaml

from shadow_support import MspEvidence, fresh


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp(value):
    value = max(0., value) if math.isfinite(value) else 0.
    ns = round(value * 1e9)
    return Time(sec=ns//1000000000, nanosec=ns % 1000000000)


class EvidenceNode(Node):
    def __init__(self):
        super().__init__('msp_evidence')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('MSP evidence requires system ROS time')
        defaults = dict(bridge_config='', aux_low=1700, aux_high=2100,
                        rx_map=[0, 1, 3, 2, 4, 5, 6, 7], battery_timeout=1.5,
                        geofence_min=[0., 0., 0.], geofence_max=[0., 0., 0.])
        for k, v in defaults.items():
            self.declare_parameter(k, v, ParameterDescriptor(read_only=True))
        values = {k: self.get_parameter(k).value for k in defaults}
        with Path(values['bridge_config']).open() as stream:
            expected = yaml.safe_load(stream)
        expected.update({k: values[k] for k in ('aux_low', 'aux_high', 'rx_map')})
        if not 900 <= values['aux_low'] < values['aux_high'] <= 2100:
            raise ValueError('invalid AUX ranges')
        if sorted(values['rx_map']) != list(range(8)):
            raise ValueError('rx_map must be an eight-channel permutation')
        self.decoder = MspEvidence(expected)
        self.battery_timeout = values['battery_timeout']
        if not math.isfinite(self.battery_timeout) or self.battery_timeout <= 0:
            raise ValueError('battery_timeout')
        self.minimum, self.maximum = values['geofence_min'], values['geofence_max']
        if len(self.minimum) != 3 or len(self.maximum) != 3 or not all(math.isfinite(x) for x in self.minimum+self.maximum):
            raise ValueError('geofence requires three finite coordinates')
        self.clock_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.output = None
        self.fused = None
        self.create_subscription(MspEvent, 'msp/events', self.on_event, 1000)
        self.create_subscription(OutputStatus, 'output_status', self.on_output, 10)
        self.create_subscription(FusedState, 'fused_state', self.on_state, 10)
        self.authority_pub = self.create_publisher(Authority, 'authority', 1)
        self.health_pub = self.create_publisher(Health, 'health', 1)
        self.decoded_pub = self.create_publisher(MspState, 'msp/decoded_state', 10)
        self.status_pub = self.create_publisher(String, 'health/status', 10)
        self.create_timer(.02, self.publish)

    def on_output(self, message):
        self.output = message

    def on_state(self, message):
        self.fused = message

    def on_event(self, message):
        # The receipt stamp is never substituted for a missing request stamp.
        # A wall-clock discontinuity or future evidence invalidates the session.
        now = self.get_clock().now().nanoseconds * 1e-9
        if not message.session_id.startswith(self.clock_id + ':'):
            return
        request_time = seconds(message.request_stamp)
        if message.event == 'rx' and (not fresh(now, request_time, 3.) or
                                     not fresh(time.monotonic(), message.request_steady_time, 3.) or
                                     abs((now-request_time) - (time.monotonic()-message.request_steady_time)) > .05):
            self.decoder.accept(message.code, [], 0., message.session_id, event='transport_error')
            return
        self.decoder.accept(message.code, message.payload, request_time, message.session_id,
                            message.request_name, message.event)
        if message.event != 'tx':
            self.publish()

    def publish(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        data = self.decoder.snapshot(now, self.battery_timeout)
        decoded = MspState()
        decoded.header.stamp = stamp(now)
        decoded.session_id = self.decoder.session or ''
        for key, value in data.items():
            setattr(decoded, key, stamp(value) if key.endswith('_stamp') else value)
        self.decoded_pub.publish(decoded)
        authority = Authority()
        authority.header.stamp = stamp(min(data['rc_stamp'], data['status_stamp']))
        authority.armed = data['armed']
        authority.auto_switch = data['auto_switch']
        authority.kill = data['kill']
        authority.rc_link = data['rc_link']
        # Hardware never treats the possibly overridden AETR as physical input.
        authority.manual_aetr = [1500, 1500, 1000, 1500]
        self.authority_pub.publish(authority)
        health = Health()
        health.header.stamp = stamp(now)
        health.config_verified = data['config_verified']
        health.transport_healthy = data['transport_healthy']
        health.battery_voltage = data['battery_voltage']
        output_fresh = (self.output is not None and self.output.clock_id == self.clock_id and
                        fresh(now, seconds(self.output.header.stamp), .05) and
                        fresh(time.monotonic(), self.output.steady_time, .05))
        health.thrust_calibrated = bool(output_fresh and self.output.thrust_calibrated)
        health.transport_healthy = bool(health.transport_healthy and output_fresh and self.output.transport_healthy)
        configured = all(a < b for a, b in zip(self.minimum, self.maximum))
        if (configured and self.fused is not None and self.fused.clock_id == self.clock_id and
                self.fused.initialized and fresh(now, seconds(self.fused.header.stamp), .01)):
            position = self.fused.position.x, self.fused.position.y, self.fused.position.z
            health.geofence_ok = all(a <= x <= b for a, x, b in zip(self.minimum, position, self.maximum))
        # Neither MSP sensor-present/calibrating bits nor EKF initialization prove
        # completed calibration or statistical convergence. Unknown stays false.
        health.imu_calibrated = False
        health.converged = False
        self.health_pub.publish(health)
        reasons = [data['reason'], 'IMU calibration evidence unavailable', 'EKF convergence evidence unavailable']
        if not health.thrust_calibrated:
            reasons.append('Thrust calibration unavailable')
        if not health.geofence_ok:
            reasons.append('Geofence unconfigured, stale, or exceeded')
        if not math.isfinite(health.battery_voltage):
            reasons.append('Battery unavailable/stale')
        self.status_pub.publish(String(data=json.dumps(dict(reasons=reasons, session=decoded.session_id))))


def main():
    rclpy.init()
    node = EvidenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
