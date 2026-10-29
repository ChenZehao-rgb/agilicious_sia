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

from shadow_support import MspEvidence, evidence_config_path, fresh


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp(value):
    value = max(0., value) if math.isfinite(value) else 0.
    ns = round(value * 1e9)
    return Time(sec=ns//1000000000, nanosec=ns % 1000000000)


def mode_key(data, session):
    return (session,) + tuple(data[key] for key in (
        'receiver_valid', 'armed', 'auto_switch', 'kill', 'rc_link',
        'imu_ready', 'control_mode_ok', 'config_verified'))


class ClockEpoch:
    """Local ROS/CLOCK_MONOTONIC relation, independent of incoming evidence."""
    def __init__(self, ros_now, steady_now):
        self.offset = ros_now - steady_now
        self.started_steady = -math.inf
        self.resets = 0
        self.last_step_seconds = 0.
        self.rejected_events = 0

    def observe(self, ros_now, steady_now):
        offset = ros_now - steady_now
        step = offset - self.offset
        self.offset = offset
        if abs(step) <= .05:
            return False
        self.started_steady = steady_now
        self.resets += 1
        self.last_step_seconds = step
        return True


def check_clock_epoch(node, ros_now, steady_now):
    if not hasattr(node, 'clock_epoch'):
        node.clock_epoch = ClockEpoch(ros_now, steady_now)
    if node.clock_epoch.observe(ros_now, steady_now):
        node.decoder.reset_clock_epoch()
        node.output = None
        node.fused = None
    return node.clock_epoch


def sample_clocks(node):
    # Scheduling between the two reads is not evidence of a wall-clock step.
    # Bound skew well below the 50 ms epoch threshold, retrying interrupted reads.
    for _ in range(3):
        before = time.monotonic()
        ros_now = node.get_clock().now().nanoseconds * 1e-9
        after = time.monotonic()
        if 0. <= after - before <= .005:
            return ros_now, after
    node.decoder.invalidate_modes('Clock sampling delayed; fresh physical AUTO low required')
    return None


def output_freshness(output, clock_id, ros_now, steady_now, received_steady=None):
    """Expose the exact cached evidence used by the unchanged 50 ms interlock."""
    def age(now, sample):
        value = now - sample
        return value if math.isfinite(value) else None

    result = dict(available=output is not None, clock_matches=False,
                  ros_age_seconds=None, steady_age_seconds=None,
                  cache_age_seconds=None, fresh=False)
    if output is None:
        return result
    result.update(clock_matches=output.clock_id == clock_id,
                  ros_age_seconds=age(ros_now, seconds(output.header.stamp)),
                  steady_age_seconds=age(steady_now, output.steady_time),
                  cache_age_seconds=age(steady_now, received_steady) if received_steady is not None else None)
    result['fresh'] = (result['clock_matches'] and
                       fresh(ros_now, seconds(output.header.stamp), .05) and
                       fresh(steady_now, output.steady_time, .05))
    return result


class EvidenceNode(Node):
    def __init__(self):
        super().__init__('msp_evidence')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('MSP evidence requires system ROS time')
        defaults = dict(runtime_config='', bridge_config='', aux_low=1700, aux_high=2100,
                        arm_aux=0, auto_aux=1, kill_aux=2,
                        rx_map=[0, 1, 3, 2, 4, 5, 6, 7], battery_timeout=1.5,
                        expected_pid_profile=0, expected_rate_profile=0, expected_override_timeout_ms=50,
                        geofence_min=[0., 0., 0.], geofence_max=[0., 0., 0.])
        for k, v in defaults.items():
            self.declare_parameter(k, v, ParameterDescriptor(read_only=True))
        values = {k: self.get_parameter(k).value for k in defaults}
        filename = evidence_config_path(values['runtime_config'], values['bridge_config'])
        with Path(filename).open() as stream:
            config = yaml.safe_load(stream)
        expected = config['bridge'] if values['runtime_config'] else config
        expected.update({k: values[k] for k in ('aux_low', 'aux_high', 'rx_map',
                                               'arm_aux', 'auto_aux', 'kill_aux',
                                               'expected_pid_profile', 'expected_rate_profile',
                                               'expected_override_timeout_ms')})
        if not 900 <= values['aux_low'] < values['aux_high'] <= 2100:
            raise ValueError('invalid AUX ranges')
        if sorted(values['rx_map']) != list(range(8)):
            raise ValueError('rx_map must be an eight-channel permutation')
        if any(not 0 <= values[k] <= 254 for k in ('expected_pid_profile', 'expected_rate_profile')):
            raise ValueError('expected profiles must be zero-based indices')
        self.decoder = MspEvidence(expected)
        self.battery_timeout = values['battery_timeout']
        if not math.isfinite(self.battery_timeout) or self.battery_timeout <= 0:
            raise ValueError('battery_timeout')
        self.minimum, self.maximum = values['geofence_min'], values['geofence_max']
        if len(self.minimum) != 3 or len(self.maximum) != 3 or not all(math.isfinite(x) for x in self.minimum+self.maximum):
            raise ValueError('geofence requires three finite coordinates')
        self.clock_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        clocks = sample_clocks(self)
        if clocks is not None:
            self.clock_epoch = ClockEpoch(*clocks)
        self.output = None
        self.output_received_steady = None
        self.fused = None
        # Keep switch updates independent of high-rate control tx/ACK diagnostics.
        self.create_subscription(MspEvent, 'msp/evidence_events', self.on_event, 100)
        for topic in ('msp/rc', 'msp/status', 'msp/battery'):
            self.create_subscription(MspEvent, topic, self.on_event, 10)
        # OutputStatus is a current snapshot, not an event history. Retaining
        # old snapshots can keep this cache stale after a scheduling stall.
        self.create_subscription(OutputStatus, 'output_status', self.on_output, 1)
        self.create_subscription(FusedState, 'fused_state', self.on_state, 1)
        self.authority_pub = self.create_publisher(Authority, 'authority', 1)
        self.health_pub = self.create_publisher(Health, 'health', 1)
        self.decoded_pub = self.create_publisher(MspState, 'msp/decoded_state', 10)
        self.status_pub = self.create_publisher(String, 'health/status', 10)
        self.create_timer(.02, self.publish)

    def on_output(self, message):
        self.output = message
        self.output_received_steady = time.monotonic()

    def on_state(self, message):
        self.fused = message

    def on_event(self, message):
        # The receipt stamp is never substituted for a missing request stamp.
        # Only an independently observed local clock step permits epoch recovery.
        if not message.session_id.startswith(self.clock_id + ':'):
            return
        if message.event in ('error', 'timeout', 'transport_error'):
            # A real transport failure is authoritative even when local clock
            # sampling is delayed or a wall-clock step is being recovered.
            self.decoder.accept(message.code, [], 0., message.session_id, event=message.event)
            self.publish()
            return
        clocks = sample_clocks(self)
        if clocks is None:
            return
        now, steady = clocks
        request_time = seconds(message.request_stamp)
        mode = message.code in (105, 150) and message.event in ('rx', 'mode_rx_unmatched')
        epoch = check_clock_epoch(self, now, steady)
        if message.event in ('rx', 'mode_rx_unmatched') and (
                0. < message.steady_time < epoch.started_steady or
                (message.event == 'rx' and 0. < message.request_steady_time < epoch.started_steady)):
            # Includes requests sent before the step whose replies arrived after
            # it. Their payloads cannot revalidate configuration or physical low.
            epoch.rejected_events += 1
            self.publish()
            return
        if mode:
            # Validate the response clocks independently of the successful
            # request-age limit in snapshot(). Unmatched replies never grant
            # authority and cannot manufacture a physical AUTO low.
            received_ros = seconds(message.header.stamp)
            invalid = (not math.isfinite(message.steady_time) or message.steady_time <= 0. or
                       message.steady_time > steady or received_ros <= 0. or received_ros > now or
                       abs((now-received_ros) - (steady-message.steady_time)) > .05)
            if message.event == 'rx':
                invalid = invalid or (not math.isfinite(request_time) or request_time <= 0. or
                                      request_time > now or not math.isfinite(message.request_steady_time) or
                                      message.request_steady_time <= 0. or message.request_steady_time > steady or
                                      abs((now-request_time) - (steady-message.request_steady_time)) > .05)
        else:
            invalid = message.event == 'rx' and (
                not fresh(now, request_time, 3.) or not fresh(steady, message.request_steady_time, 3.) or
                abs((now-request_time) - (steady-message.request_steady_time)) > .05)
        if invalid:
            self.decoder.accept(message.code, [], 0., message.session_id, event='transport_error')
            return
        self.decoder.accept(message.code, message.payload, request_time, message.session_id,
                            message.request_name, message.event, observed_steady=message.steady_time,
                            request_steady=message.request_steady_time)
        changed = mode and mode_key(self.decoder.snapshot(now, getattr(self, 'battery_timeout', 1.5), steady_now=steady),
                                    self.decoder.session) != getattr(self, '_last_mode_key', None)
        if changed or message.event in ('error', 'timeout', 'transport_error', 'mode_timeout'):
            self.publish()

    def publish(self):
        clocks = sample_clocks(self)
        if clocks is None:
            return
        now, steady = clocks
        epoch = check_clock_epoch(self, now, steady)
        data = self.decoder.snapshot(now, self.battery_timeout, steady_now=steady)
        self._last_mode_key = mode_key(data, self.decoder.session)
        decoded = MspState()
        decoded.header.stamp = stamp(now)
        decoded.session_id = self.decoder.session or ''
        for key, value in data.items():
            setattr(decoded, key, stamp(value) if key.endswith('_stamp') else value)
        self.decoded_pub.publish(decoded)
        authority = Authority()
        authority.header.stamp = stamp(min(data['rc_stamp'], data['status_stamp']))
        authority.rc_stamp = stamp(data['rc_stamp'])
        authority.status_stamp = stamp(data['status_stamp'])
        authority.clock_id = self.clock_id
        authority.rc_request_steady_time = self.decoder.mode_request_steady.get(105, math.nan)
        authority.status_request_steady_time = self.decoder.mode_request_steady.get(150, math.nan)
        authority.receiver_valid = data['receiver_valid']
        authority.session_id = self.decoder.session or ''
        authority.armed = data['armed']
        authority.auto_switch = data['auto_switch']
        authority.kill = data['kill']
        authority.rc_link = data['rc_link']
        # Hardware never treats the possibly overridden AETR as physical input.
        authority.manual_aetr = [1500, 1500, 1000, 1500]
        self.authority_pub.publish(authority)
        health = Health()
        health.header.stamp = stamp(now)
        health.config_verified = data['config_verified'] and data['control_mode_ok']
        health.transport_healthy = data['transport_healthy']
        health.battery_voltage = data['battery_voltage']
        for key in ('battery_session', 'battery_sequence', 'battery_request_steady_time', 'battery_receive_steady_time'):
            setattr(health, key, data[key])
        output_diagnostic = output_freshness(self.output, self.clock_id, now, time.monotonic(),
                                             getattr(self, 'output_received_steady', None))
        # Mapping is configuration, not a heartbeat. Control checks the output
        # process heartbeat directly; feeding its 50 ms diagnostic cache back
        # into Health can spuriously revoke an otherwise current command.
        output_known = (output_diagnostic['clock_matches'] and
                        output_diagnostic['ros_age_seconds'] is not None and
                        output_diagnostic['ros_age_seconds'] >= 0. and
                        output_diagnostic['steady_age_seconds'] is not None and
                        output_diagnostic['steady_age_seconds'] >= 0.)
        health.thrust_calibrated = bool(output_known and self.output.thrust_calibrated)
        health.thrust_mapping_ready = bool(output_known and self.output.thrust_mapping_ready)
        # MSP readback failures remain authoritative. The output process checks
        # its own write health without this asynchronous summary round trip.
        configured = all(a < b for a, b in zip(self.minimum, self.maximum))
        # A diagnostic cache has a 25 ms lifetime. Control checks its own latest
        # 15 ms state and consumes FC readiness independently of this summary.
        fused_fresh = (self.fused is not None and self.fused.clock_id == self.clock_id and
                       (self.fused.fc_state_valid if self.fused.state_source == 'fc_fused' else self.fused.initialized) and
                       fresh(now, seconds(self.fused.header.stamp), .025) and
                       fresh(time.monotonic(), self.fused.published_steady_time, .025))
        if configured and fused_fresh:
            position = self.fused.position.x, self.fused.position.y, self.fused.position.z
            health.geofence_ok = all(a <= x <= b for a, x, b in zip(self.minimum, position, self.maximum))
        # Readiness combines explicit FC checks and the estimator's statistical
        # startup/update checks; legacy calibration/RTK claims remain untouched.
        health.imu_calibrated = False
        health.converged = False
        health.fc_imu_ready = data['imu_ready']
        health.imu_ready = bool(fused_fresh and health.fc_imu_ready and self.fused.imu_ready)
        health.estimator_ready = bool(fused_fresh and self.fused.estimator_ready)
        health.navigation_ready = bool(fused_fresh and self.fused.navigation_ready and
                                       self.fused.navigation_accuracy_ok and self.fused.accuracy_known and
                                       self.fused.heading_valid and self.fused.clock_aligned)
        health.state_source = self.fused.state_source if self.fused else ''
        health.state_session = self.fused.navigation_session if self.fused else ''
        health.state_reset_counter = self.fused.reset_counter if self.fused else 0
        health.msp_healthy = data['transport_healthy']
        health.control_output_healthy = bool(output_known and self.output.transport_healthy)
        health.fc_estimator_healthy = bool(fused_fresh and self.fused.state_source == 'fc_fused' and
                                           self.fused.fc_state_valid and self.fused.clock_aligned and
                                           fresh(time.monotonic(), self.fused.fc_state_sample_steady_time, .015) and
                                           fresh(time.monotonic(), self.fused.fc_health_sample_steady_time, .060))
        reasons = [data['reason']]
        if health.state_source != 'fc_fused' and not health.imu_ready:
            reasons.append('IMU startup checks or FC calibration/sensor status not ready')
        if not health.estimator_ready or not health.navigation_ready:
            reasons.append(self.fused.readiness_reason if fused_fresh else 'Fused state unavailable/stale')
        if not health.thrust_mapping_ready:
            reasons.append('Thrust mapping unavailable')
        elif not health.thrust_calibrated:
            reasons.append(f'Thrust mapping ready: {self.output.thrust_model} / '
                           f'{self.output.thrust_model_source} (uncalibrated estimate)')
        if not health.geofence_ok:
            reasons.append('Geofence unconfigured, stale, or exceeded')
        if not math.isfinite(health.battery_voltage):
            reasons.append('Battery unavailable/stale')
        health.reason = '; '.join(reasons)
        self.health_pub.publish(health)
        self.status_pub.publish(String(data=json.dumps(dict(reasons=reasons, session=decoded.session_id,
                                                           clock=dict(epoch_resets=epoch.resets,
                                                                      last_step_seconds=epoch.last_step_seconds,
                                                                      rejected_events=epoch.rejected_events,
                                                                      recovery_pending=self.decoder.clock_recovery_pending),
                                                           estimator=dict(source=health.state_source, healthy=health.fc_estimator_healthy,
                                                                          session=health.state_session, reset=health.state_reset_counter),
                                                           msp_healthy=health.msp_healthy, output=output_diagnostic))))


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
