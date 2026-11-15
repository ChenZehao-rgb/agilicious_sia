#!/usr/bin/env python3
"""Resource accounting and recording launch tests without starting flight nodes."""
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from diagnostic_msgs.msg import DiagnosticStatus
from launch import LaunchContext
from launch.actions import RegisterEventHandler
from launch.events.process import ProcessExited
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'agi_ros2/scripts'))
import system_monitor as monitor


class ResourceTests(unittest.TestCase):
    def test_cpu_excludes_guest_double_count_and_iowait(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'stat'
            path.write_text('cpu 100 0 50 500 50 0 0 0 70 0\n'
                            'cpu0 100 0 50 500 50 0 0 0 70 0\nintr 42\n')
            previous = monitor.read_cpu_times(path)
            path.write_text('cpu 120 0 60 550 60 0 0 0 90 0\n'
                            'cpu0 120 0 60 550 60 0 0 0 90 0\n')
            current = monitor.read_cpu_times(path)
            self.assertEqual(set(previous), {'cpu', 'cpu0'})
            for name in current:
                self.assertAlmostEqual(monitor.cpu_percent(previous[name], current[name]), 100.0 / 3)

    def test_cpu_unavailable_intervals_are_not_reported_as_zero(self):
        counters = (100, 0, 50, 500, 50, 0, 0, 0)
        self.assertIsNone(monitor.cpu_percent(None, counters))
        self.assertIsNone(monitor.cpu_percent(counters, counters))
        self.assertIsNone(monitor.cpu_percent(counters, (0,) * 8))
        self.assertIsNone(monitor.cpu_percent(counters, (110, 0, 60, 510, 49, 0, 0, 0)))
        self.assertEqual(monitor.cpu_percent(counters, (100, 0, 50, 600, 50, 0, 0, 0)), 0.)
        self.assertEqual(monitor.cpu_percent(counters, (200, 0, 50, 500, 50, 0, 0, 0)), 100.)

    def test_memory_uses_available_and_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'meminfo'
            path.write_text('MemTotal: 100000 kB\nMemFree: 10000 kB\nMemAvailable: 40000 kB\n')
            values = monitor.read_memory_usage(path)
            self.assertEqual(values['memory_total_bytes'], 102400000)
            self.assertEqual(values['memory_available_bytes'], 40960000)
            self.assertEqual(values['memory_used_bytes'], 61440000)
            self.assertEqual(values['memory_used_percent'], 60.)
            path.write_text('MemTotal: 100000 kB\nMemFree: 10000 kB\n')
            with self.assertRaises(ValueError):
                monitor.read_memory_usage(path)

    def test_sampling_error_and_recovery_reset_cpu_baseline(self):
        node = SimpleNamespace(hostname='test-host', previous_cpu={}, previous_monotonic_ns=None,
                               publisher=Mock(), get_clock=Mock())
        # Use a real ROS timestamp type while avoiding a DDS participant.
        from builtin_interfaces.msg import Time
        node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=100)
        cpu = [{'cpu': (100, 0, 50, 500, 50, 0, 0, 0)},
               {'cpu': (120, 0, 60, 550, 60, 0, 0, 0)}]
        with patch.object(monitor, 'read_cpu_times', side_effect=[cpu[0], cpu[1], OSError('read failed'), cpu[1]]), \
                patch.object(monitor, 'read_memory_usage', side_effect=lambda: {'memory_used_percent': 60.}), \
                patch.object(monitor.time, 'monotonic_ns', side_effect=[1000000000, 2000000000, 3000000000, 4000000000]):
            for _ in range(4):
                monitor.SystemMonitor.sample(node)
        messages = [call.args[0] for call in node.publisher.publish.call_args_list]
        self.assertEqual([msg.status[0].level for msg in messages],
                         [DiagnosticStatus.WARN, DiagnosticStatus.OK, DiagnosticStatus.ERROR, DiagnosticStatus.WARN])
        values = {item.key: item.value for item in messages[1].status[0].values}
        self.assertAlmostEqual(float(values['cpu_percent']), 100.0 / 3)
        self.assertEqual(values['sample_interval_seconds'], '1.0')
        self.assertEqual(values['monotonic_ns'], '2000000000')
        self.assertEqual(messages[1].header.stamp.sec, 100)
        self.assertEqual(messages[2].status[0].values, [])
        self.assertNotIn('cpu_percent', {item.key for item in messages[3].status[0].values})

    def test_monitor_only_runs_with_recorder_and_does_not_stop_flight(self):
        path = ROOT / 'agi_ros2/launch/runtime_launch.py'
        spec = importlib.util.spec_from_file_location('resource_runtime', path)
        runtime = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(runtime)
        for record in ('true', 'false'):
            with self.subTest(record=record), tempfile.TemporaryDirectory() as directory:
                context = LaunchContext()
                context.launch_configurations.update(runtime.DEFAULTS)
                context.launch_configurations.update(mode='hardware', shadow_only='false', bench_fixed_gps='false',
                                                     record_bag=record, bag_output=str(Path(directory) / 'bag'))
                actions = runtime.assemble(context)
                monitors = [action for action in actions if isinstance(action, Node) and
                            action.node_executable == 'system_monitor.py']
                self.assertEqual(len(monitors), int(record == 'true'))
                if monitors:
                    handlers = [action.event_handler for action in actions if isinstance(action, RegisterEventHandler)]
                    exited = ProcessExited(action=monitors[0], name='system_monitor', cmd=[], cwd=None,
                                           env=None, pid=123, returncode=1)
                    self.assertFalse(any(handler.matches(exited) for handler in handlers))


if __name__ == '__main__':
    unittest.main()
