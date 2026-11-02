#!/usr/bin/env python3
"""Run on the control host: latest-only telemetry relay and fixed, explicit runtime start/stop listener."""
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Empty
from runtime_launcher import RuntimeLauncher
from agi_ros2.msg import Authority, ComputationStatus, FusedState, GoalStatus, Health, NavigationOrigin, OutputStatus

LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
RETAINED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
RUNTIME_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
FAST = {'fused_state': FusedState}
SLOW = {'authority': Authority, 'health': Health, 'output_status': OutputStatus, 'computation_status': ComputationStatus}
EVENTS = {
    'authority': ('receiver_valid', 'session_id', 'armed', 'auto_switch', 'kill', 'rc_link'),
    'health': ('fc_imu_ready', 'imu_ready', 'estimator_ready', 'navigation_ready', 'config_verified',
               'thrust_mapping_ready', 'thrust_calibrated', 'geofence_ok', 'transport_healthy',
               'state_source', 'state_session', 'state_reset_counter', 'fc_estimator_healthy',
               'msp_healthy', 'control_output_healthy'),
    'output_status': ('session_start', 'fault_count', 'override_active', 'transport_healthy', 'thrust_mapping_ready'),
    'computation_status': ('shadow_only', 'state_valid', 'controller_success', 'trajectory_active', 'hover_stable',
                           'control_session_start', 'fc_session', 'fc_generation'),
}
RETAINED_TOPICS = {'navigation/origin': NavigationOrigin, 'goal_status': GoalStatus}


class GroundStationTelemetry(Node):
    def __init__(self, **kwargs):
        super().__init__('ground_station_telemetry', **kwargs)
        self.cache = {}
        self.versions = {}
        self.sent = {}
        self.events = {}
        self.publishers_by_topic = {}
        self.timers_by_rate = {}
        self.groups = {'fast_rate_hz': FAST, 'slow_rate_hz': SLOW}
        rates = {name: self.declare_parameter(name, default).value
                 for name, default in [('fast_rate_hz', 20.0), ('slow_rate_hz', 2.0)]}
        self.validate_rates(rates)
        for topic, msg_type in {**FAST, **SLOW, **RETAINED_TOPICS}.items():
            retained = topic in RETAINED_TOPICS
            qos = RETAINED if retained else LATEST
            self.publishers_by_topic[topic] = self.create_publisher(msg_type, 'ground_station/' + topic, qos)
            self.create_subscription(msg_type, topic,
                                     lambda msg, key=topic: self.receive(key, msg), qos)
        mode = self.declare_parameter('runtime_mode', 'hardware',
                                      ParameterDescriptor(read_only=True, description='Startup role: hardware or simulation')).value
        if mode not in ('hardware', 'simulation'):
            raise ValueError('runtime_mode must be hardware or simulation')
        self.runtime_publisher = self.create_publisher(DiagnosticArray, f'ground_station/runtime/{mode}/status', RUNTIME_QOS)
        self.runtime_guard = self.create_guard_condition(self.publish_runtime)
        self.runtime = RuntimeLauncher(mode, self.runtime_changed)
        self.create_subscription(Empty, f'ground_station/runtime/{mode}/start', lambda _: self.runtime.start(), RUNTIME_QOS)
        self.create_subscription(Empty, f'ground_station/runtime/{mode}/stop', lambda _: self.runtime.stop(), RUNTIME_QOS)
        self.publish_runtime()
        self.replace_timers(rates)
        self.add_on_set_parameters_callback(self.set_rates)

    @staticmethod
    def validate_rates(rates):
        for name, value in rates.items():
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
            # rcl timers represent periods as signed 64-bit nanoseconds.
            period = 1e9 / value
            if not math.isfinite(period) or not 1 <= period < 2**63:
                raise ValueError(name + ' period is not representable in nanoseconds')

    def replace_timers(self, rates):
        replacements = {}
        try:
            for name, value in rates.items():
                replacements[name] = self.create_timer(1.0 / value, lambda key=name: self.flush(key))
        except Exception:
            for timer in replacements.values():
                self.destroy_timer(timer)
            raise
        for name, timer in replacements.items():
            if name in self.timers_by_rate:
                self.destroy_timer(self.timers_by_rate[name])
            self.timers_by_rate[name] = timer

    def set_rates(self, parameters):
        rates = {p.name: p.value for p in parameters if p.name in self.groups}
        try:
            self.validate_rates(rates)
            self.replace_timers(rates)
        except (ValueError, OverflowError, RuntimeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def receive(self, topic, message):
        previous = self.cache.get(topic)
        if topic in RETAINED_TOPICS and previous == message:
            return
        if topic == 'navigation/origin' and previous is not None and previous.fc_time_usec and \
                message.fc_time_usec >= previous.fc_time_usec:
            # Origin is a retained coordinate mapping, not a position heartbeat.
            # Keep the original, unmodified snapshot when only its timestamps
            # advance: forwarding it would overtake the throttled fused_state
            # and spuriously fail the UI's origin <= state time check.
            fields = (name for name in message.get_fields_and_field_types() if name not in ('header', 'fc_time_usec'))
            if message.header.frame_id == previous.header.frame_id and \
                    all(getattr(message, name) == getattr(previous, name) for name in fields):
                return
        self.cache[topic] = message
        self.versions[topic] = self.versions.get(topic, 0) + 1
        if topic in RETAINED_TOPICS:
            self.send_latest(topic)
        elif topic in EVENTS:
            event = tuple(getattr(message, name) for name in EVENTS[topic])
            if self.events.get(topic) != event:
                self.events[topic] = event
                self.send_latest(topic)

    def send_latest(self, topic):
        version = self.versions.get(topic, 0)
        if version != self.sent.get(topic, 0):
            self.publishers_by_topic[topic].publish(self.cache[topic])
            self.sent[topic] = version

    def runtime_changed(self):
        try:
            self.runtime_guard.trigger()
        except Exception:
            # rclpy may invalidate guards as SIGTERM shuts down the context.
            if self.context.ok():
                raise

    def publish_runtime(self):
        snapshot = self.runtime.snapshot()
        entries = snapshot.pop('processes')
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        for name, values in [('summary', snapshot), *entries.items()]:
            status = DiagnosticStatus(name=name, hardware_id=self.runtime.instance,
                                      level=DiagnosticStatus.ERROR if values['state'] in ('failed', 'exited') else DiagnosticStatus.OK,
                                      message=values['reason'],
                                      values=[KeyValue(key=key, value=str(value).lower() if isinstance(value, bool) else str(value))
                                              for key, value in values.items()])
            message.status.append(status)
        self.runtime_publisher.publish(message)

    def destroy_node(self):
        self.runtime.notify = lambda: None
        self.runtime.close()
        return super().destroy_node()

    def flush(self, rate):
        if rate == 'slow_rate_hz':
            # A listener heartbeat must repeat even when no flight data exists.
            self.publish_runtime()
        for topic in self.groups[rate]:
            self.send_latest(topic)


def main():
    rclpy.init()
    node = GroundStationTelemetry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
