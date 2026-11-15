#!/usr/bin/env python3
"""Publish host CPU and memory usage at 1 Hz for bag recording."""
from pathlib import Path
import socket
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def read_cpu_times(filename='/proc/stat'):
    counters = {}
    for line in Path(filename).read_text().splitlines():
        fields = line.split()
        if fields and (fields[0] == 'cpu' or (fields[0].startswith('cpu') and fields[0][3:].isdigit())):
            if len(fields) < 5:
                raise ValueError('Incomplete CPU counters: ' + fields[0])
            # guest/guest_nice are already included in user/nice; do not count twice.
            counters[fields[0]] = tuple(int(value) for value in fields[1:9])
    if 'cpu' not in counters:
        raise ValueError('Missing aggregate CPU counters')
    return counters


def cpu_percent(previous, current):
    if previous is None or len(previous) != len(current):
        return None
    delta = [new - old for old, new in zip(previous, current)]
    total = sum(delta)
    # Counters can reset, and Linux documents that iowait can decrease.
    if total <= 0 or any(value < 0 for value in delta):
        return None
    idle = delta[3] + (delta[4] if len(delta) > 4 else 0)
    return 100.0 * (total - idle) / total


def read_memory_usage(filename='/proc/meminfo'):
    memory = {}
    for line in Path(filename).read_text().splitlines():
        fields = line.split()
        if fields and fields[0] in ('MemTotal:', 'MemAvailable:'):
            memory[fields[0][:-1]] = int(fields[1]) * 1024
    total, available = memory.get('MemTotal', 0), memory.get('MemAvailable', -1)
    if total <= 0 or not 0 <= available <= total:
        raise ValueError('Missing or invalid MemTotal/MemAvailable')
    used = total - available
    return {'memory_total_bytes': total, 'memory_available_bytes': available,
            'memory_used_bytes': used, 'memory_used_percent': 100.0 * used / total}


class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.publisher = self.create_publisher(DiagnosticArray, 'system/resource_usage', 10)
        self.hostname = socket.gethostname()
        self.previous_cpu = {}
        self.previous_monotonic_ns = None
        self.sample()
        # Keep sampling host resources through wall-clock corrections and paused simulation.
        self.timer = self.create_timer(1.0, self.sample, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def sample(self):
        status = DiagnosticStatus(name='system/resource_usage', hardware_id=self.hostname,
                                  level=DiagnosticStatus.OK, message='Host resource usage')
        try:
            now_ns = time.monotonic_ns()
            current_cpu = read_cpu_times()
            values = read_memory_usage()
            values['monotonic_ns'] = now_ns
            if self.previous_monotonic_ns is not None:
                values['sample_interval_seconds'] = (now_ns - self.previous_monotonic_ns) / 1e9
            for name, counters in current_cpu.items():
                usage = cpu_percent(self.previous_cpu.get(name), counters)
                if usage is not None:
                    values[name + '_percent'] = usage
            if 'cpu_percent' not in values:
                status.level = DiagnosticStatus.WARN
                status.message = 'CPU interval unavailable; collecting a new baseline'
            self.previous_cpu = current_cpu
            self.previous_monotonic_ns = now_ns
            status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        except (OSError, ValueError, IndexError) as error:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Resource sampling failed: ' + str(error)
            self.previous_cpu = {}
            self.previous_monotonic_ns = None
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = SystemMonitor()
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
