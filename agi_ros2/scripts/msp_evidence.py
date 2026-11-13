#!/usr/bin/env python3
"""Assemble hardware authority/health from timestamped MAVLink telemetry and output diagnostics.

The executable filename is retained for launch compatibility; no MSP readback grants authority.
"""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from builtin_interfaces.msg import Time
from std_msgs.msg import String
from agi_ros2.msg import FcTelemetry, MspEvent, Authority, Health, OutputStatus, FusedState
import yaml

from shadow_support import fresh


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp(value):
    value = max(0., value) if math.isfinite(value) else 0.
    ns = round(value * 1e9)
    return Time(sec=ns//1000000000, nanosec=ns % 1000000000)


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


class TelemetryEvidence:
    MODE_MAX_AGE = .500

    def __init__(self, clock_id, aux_low=1875, aux_high=2100):
        self.clock_id = clock_id
        self.low, self.high = aux_low, aux_high
        self.session = ''
        self.message = None
        self.failed = False
        self.recovery_pending = True
        self.fault_versions = (0., 0., 0.)
        self.last = dict(arm_switch=False, armed=False, auto_switch=False, kill=False, rc_link=False)
        self.reason = 'Waiting for MAVLink RC/HEARTBEAT/SYS_STATUS and physical AUTO low'

    def invalidate_modes(self, reason):
        if not self.recovery_pending and self.message is not None:
            self.fault_versions = self.versions(self.message)
        self.recovery_pending = True
        self.reason = reason

    def reset_clock_epoch(self):
        self.invalidate_modes('Local clock changed; fresh physical AUTO low required')
        self.message = None

    @staticmethod
    def versions(message):
        return (message.rc_receive_steady_time, message.heartbeat_receive_steady_time,
                message.system_receive_steady_time)

    def accept(self, message):
        if message.clock_id != self.clock_id:
            return
        if self.message is not None and seconds(message.header.stamp) <= seconds(self.message.header.stamp):
            return
        if message.source_session != self.session:
            self.session = message.source_session
            self.recovery_pending = True
            self.fault_versions = (0., 0., 0.)
            self.reason = 'FC session changed; fresh physical AUTO low required'
        self.message = message

    def snapshot(self, now, battery_timeout=1.5, steady_now=None):
        steady_now = time.monotonic() if steady_now is None else steady_now
        result = dict(self.last, receiver_valid=False, config_verified=False, imu_ready=False,
                      transport_healthy=False, rc_stamp=0., status_stamp=0., battery_stamp=0.,
                      rc_steady=math.nan, status_steady=math.nan, battery_voltage=math.nan,
                      battery_current_a=math.nan, battery_consumed_mah=math.nan,
                      battery_remaining_percent=math.nan, battery_temperature_c=math.nan,
                      reason=self.reason)
        m = self.message
        if m is None:
            return result
        result.update(rc_stamp=seconds(m.rc_stamp), status_stamp=seconds(m.heartbeat_stamp),
                      rc_steady=m.rc_sample_steady_time, status_steady=m.heartbeat_receive_steady_time)

        def current(ros_stamp, steady_stamp, timeout):
            return (steady_stamp > 0. and fresh(steady_now, steady_stamp, timeout) and
                    fresh(now, seconds(ros_stamp), timeout) and
                    abs((now-seconds(ros_stamp)) - (steady_now-steady_stamp)) <= .05)

        battery_fresh = current(m.battery_stamp, m.battery_receive_steady_time, battery_timeout)
        system_battery_fresh = current(m.system_stamp, m.system_receive_steady_time, battery_timeout)
        if battery_fresh:
            cells = [v for v in m.battery_voltages_mv if v != 65535]
            extended = [v for v in m.battery_voltages_ext_mv if v != 0]
            # Cell 0 can contain total pack voltage; summing valid entries also
            # handles per-cell values and the protocol's overflow representation.
            voltage = (sum(cells) + sum(extended)) * .001 if cells else math.nan
            if cells and all(0 < v < 65535 for v in cells + extended) and voltage > 0.:
                result.update(battery_voltage=voltage, battery_stamp=seconds(m.battery_stamp),
                              battery_current_a=m.battery_current_ca * .01 if m.battery_current_ca != -1 else math.nan,
                              battery_consumed_mah=float(m.battery_consumed_mah) if m.battery_consumed_mah >= 0 else math.nan,
                              battery_remaining_percent=float(m.battery_remaining_percent)
                              if 0 <= m.battery_remaining_percent <= 100 else math.nan,
                              battery_temperature_c=m.battery_temperature_cdeg * .01
                              if m.battery_temperature_cdeg != 32767 else math.nan)
        if not math.isfinite(result['battery_voltage']) and system_battery_fresh and 0 < m.system_voltage_mv < 65535:
            result.update(battery_voltage=m.system_voltage_mv * .001, battery_stamp=seconds(m.system_stamp),
                          battery_current_a=m.system_current_ca * .01 if m.system_current_ca != -1 else math.nan,
                          battery_remaining_percent=float(m.system_remaining_percent)
                          if 0 <= m.system_remaining_percent <= 100 else math.nan)

        mode_fresh = (current(m.rc_stamp, m.rc_sample_steady_time, self.MODE_MAX_AGE) and
                      fresh(steady_now, m.rc_receive_steady_time, self.MODE_MAX_AGE) and
                      current(m.heartbeat_stamp, m.heartbeat_receive_steady_time, self.MODE_MAX_AGE) and
                      current(m.system_stamp, m.system_receive_steady_time, self.MODE_MAX_AGE))
        channels_valid = all(900 <= m.channels[i] <= 2100 for i in (4, 7))
        link = m.system_status in (3, 4)  # STANDBY or ACTIVE; CRITICAL includes armed BF failsafe.
        healthy = bool(self.session and m.clock_aligned and m.transport_healthy and not self.failed)
        result['transport_healthy'] = healthy
        result['imu_ready'] = bool(mode_fresh and (m.sensors_present & m.sensors_enabled & m.sensors_health & 3) == 3)
        if not mode_fresh or not channels_valid or not healthy or not link:
            self.invalidate_modes('MAVLink mode evidence stale/invalid or FC/transport fault; cycle physical AUTO')
            result['reason'] = self.reason
            return result
        switches = dict(arm_switch=bool(self.low <= m.channels[4] < self.high),
                        armed=bool(m.base_mode & 128), auto_switch=bool(self.low <= m.channels[7] < self.high),
                        kill=False, rc_link=True)
        # Invalid evidence never manufactures a physical AUTO-low edge. A new
        # session/fault requires refreshed evidence of all three sources and low.
        if self.recovery_pending:
            if switches['auto_switch'] or not all(v > old for v, old in zip(self.versions(m), self.fault_versions)):
                result['reason'] = self.reason
                return result
            self.recovery_pending = False
        self.last = switches
        result.update(switches, receiver_valid=True,
                      reason='MAVLink receiver evidence; disarmed RXLOSS cannot be established; FC configuration not read back')
        return result


class EvidenceNode(Node):
    def __init__(self):
        super().__init__('hardware_evidence')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Hardware evidence requires system ROS time')
        runtime_config = self.declare_parameter('runtime_config', '', ParameterDescriptor(read_only=True)).value
        defaults = dict(aux_low=1875, aux_high=2100, battery_timeout=1.5,
                        geofence_min=[0., 0., 0.], geofence_max=[0., 0., 0.])
        if runtime_config:
            with Path(runtime_config).open() as stream:
                configured = yaml.safe_load(stream).get('evidence', {})
            defaults.update({k: configured[k] for k in defaults if k in configured})
        for k, v in defaults.items():
            self.declare_parameter(k, v, ParameterDescriptor(read_only=True))
        values = {k: self.get_parameter(k).value for k in defaults}
        if not 900 <= values['aux_low'] < values['aux_high'] <= 2100:
            raise ValueError('invalid AUX ranges')
        self.clock_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.decoder = TelemetryEvidence(self.clock_id, values['aux_low'], values['aux_high'])
        self.battery_timeout = values['battery_timeout']
        if not math.isfinite(self.battery_timeout) or self.battery_timeout <= 0:
            raise ValueError('battery_timeout')
        self.minimum, self.maximum = values['geofence_min'], values['geofence_max']
        if len(self.minimum) != 3 or len(self.maximum) != 3 or not all(math.isfinite(x) for x in self.minimum+self.maximum):
            raise ValueError('geofence requires three finite coordinates')
        self.output = self.fused = None
        self.output_received_steady = None
        self.clock_epoch = ClockEpoch(self.get_clock().now().nanoseconds * 1e-9, time.monotonic())
        self.create_subscription(FcTelemetry, 'sensors/fc/telemetry', self.on_telemetry, 10)
        self.create_subscription(MspEvent, 'msp/evidence_events', self.on_event, 100)
        self.create_subscription(OutputStatus, 'output_status', self.on_output, 1)
        self.create_subscription(FusedState, 'fused_state', self.on_state, 1)
        self.authority_pub = self.create_publisher(Authority, 'authority', 1)
        self.health_pub = self.create_publisher(Health, 'health', 1)
        self.status_pub = self.create_publisher(String, 'health/status', 10)
        self.create_timer(.02, self.publish)

    def on_output(self, message):
        self.output = message
        self.output_received_steady = time.monotonic()

    def on_state(self, message):
        self.fused = message

    def on_telemetry(self, message):
        clocks = sample_clocks(self)
        if clocks is None:
            return
        epoch = check_clock_epoch(self, *clocks)
        if (message.clock_id != self.clock_id or
                seconds(message.header.stamp) > clocks[0] or
                not fresh(clocks[0], seconds(message.header.stamp), .5)):
            return
        if any(0 < v < epoch.started_steady for v in self.decoder.versions(message)):
            epoch.rejected_events += 1
            return
        self.decoder.accept(message)
        self.publish()

    def on_event(self, message):
        if message.session_id.startswith(self.clock_id + ':') and message.event in ('error', 'transport_error'):
            self.decoder.failed = True
            self.decoder.invalidate_modes('MSP override transport failed; restart required')
            self.publish()

    def publish(self):
        clocks = sample_clocks(self)
        if clocks is None:
            return
        now, steady = clocks
        epoch = check_clock_epoch(self, now, steady)
        data = self.decoder.snapshot(now, self.battery_timeout, steady_now=steady)
        authority = Authority()
        authority.header.stamp = stamp(min(data['rc_stamp'], data['status_stamp']))
        authority.rc_stamp = stamp(data['rc_stamp'])
        authority.status_stamp = stamp(data['status_stamp'])
        authority.clock_id = self.clock_id
        authority.rc_request_steady_time = data['rc_steady']
        authority.status_request_steady_time = data['status_steady']
        authority.receiver_valid = data['receiver_valid']
        authority.session_id = self.decoder.session
        for key in ('arm_switch', 'armed', 'auto_switch', 'kill', 'rc_link'):
            setattr(authority, key, data[key])
        authority.manual_aetr = [1500, 1500, 1000, 1500]
        self.authority_pub.publish(authority)
        health = Health()
        # This is a newly evaluated summary. Individual source times above and
        # battery_stamp below remain immutable, and snapshot() checks their ages.
        health.header.stamp = stamp(now)
        health.config_verified = False
        health.transport_healthy = data['transport_healthy']
        health.battery_stamp = stamp(data['battery_stamp'])
        for key in ('battery_voltage', 'battery_current_a', 'battery_consumed_mah',
                    'battery_remaining_percent', 'battery_temperature_c'):
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
        # MSP override transport failures remain authoritative. The output process checks
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
        # Readiness combines FC sensor availability and the estimator's statistical
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
        health.msp_healthy = bool(output_known and self.output.transport_healthy and not self.decoder.failed)
        health.control_output_healthy = bool(output_known and self.output.transport_healthy)
        health.fc_estimator_healthy = bool(fused_fresh and self.fused.state_source == 'fc_fused' and
                                           self.fused.fc_state_valid and self.fused.clock_aligned and
                                           fresh(time.monotonic(), self.fused.fc_state_sample_steady_time, .015) and
                                           fresh(time.monotonic(), self.fused.fc_health_sample_steady_time, .060))
        reasons = [data['reason']]
        if health.state_source != 'fc_fused' and not health.imu_ready:
            reasons.append('IMU startup checks or FC sensor status not ready')
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
        if not health.msp_healthy:
            reasons.append('MSP output unavailable/unhealthy')
        health.reason = '; '.join(reasons)
        self.health_pub.publish(health)
        self.status_pub.publish(String(data=json.dumps(dict(reasons=reasons, session=self.decoder.session,
                                                           clock=dict(epoch_resets=epoch.resets,
                                                                      last_step_seconds=epoch.last_step_seconds,
                                                                      rejected_events=epoch.rejected_events,
                                                                      recovery_pending=self.decoder.recovery_pending),
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
