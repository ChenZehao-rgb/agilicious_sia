#!/usr/bin/env python3
"""Launch-produced parameters on real core processes, with synthetic sensors and loopback UDP only."""
import math
import re
import subprocess
import unittest

import rclpy
import yaml
from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from agi_ros2.msg import ComputationStatus, ControlCommand, OutputStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from test_node_pipeline import BIN, Harness
from test_runtime_config import ROOT, runtime


class RuntimeProfileNodes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_default_empty_trajectory_reaches_hover_and_revokes_on_kill(self):
        harness = Harness()
        self.addCleanup(harness.close)
        harness.simulation = True
        # Keep the shipped profile content, including quoted empty strings and
        # inline model/MPC arrays. Only isolate the UDP receiver for this test.
        profile = harness.path / 'simulation.yaml'
        content = (ROOT / 'agi_ros2/config/simulation.yaml').read_text()
        content, count = re.subn(r'(?m)^(  port:) 9004$',
                                lambda match: match[1] + ' ' + str(harness.udp.getsockname()[1]), content)
        self.assertEqual(count, 1)
        profile.write_text(content)
        context = LaunchContext()
        context.launch_configurations.update(runtime.DEFAULTS)
        context.launch_configurations.update(runtime_config=str(profile), record_bag='false')
        actions = [action for action in runtime.assemble(context) if isinstance(action, Node)]
        for topic, message_type in (('computation_status', ComputationStatus), ('control_command', ControlCommand),
                                    ('output_status', OutputStatus), ('status', String), ('reference', Odometry)):
            harness.subscribe(topic, message_type)
        for action in actions:
            parameters = evaluate_parameters(context, action._Node__parameters)[0]
            args = [str(BIN / action.node_executable), '--ros-args', '-r', '__ns:=' + harness.namespace,
                    '-r', '/clock:=' + harness.namespace + '/clock']
            for key, value in parameters.items():
                encoded = yaml.safe_dump(value, default_flow_style=True).removesuffix('...\n').strip()
                args.extend(('-p', key + ':=' + encoded))
            log = open(harness.path / (action.node_executable + '.log'), 'w+')
            harness.processes.append(subprocess.Popen(args, stdout=log, stderr=log))
            harness.logs.append(log)
        harness.run(4.0, sensors=True, rate=.5)
        self.assertTrue(any('AUTO_STANDBY' in item.data for item in harness.received['status']),
                        [item.data for item in harness.received['status'][-10:]])
        self.assertTrue(any(item.mpc_success for item in harness.received['computation_status']))
        self.assertFalse(any(item.trajectory_active for item in harness.received['computation_status']))
        self.assertFalse(any(item.permit_override for item in harness.received['control_command']))
        harness.auto = True
        harness.run(.4, sensors=True, rate=.5)
        active = [item for item in harness.received['control_command'] if item.permit_override]
        self.assertTrue(active, [item.data for item in harness.received['status'][-10:]])
        self.assertTrue(all(math.isfinite(item.total_thrust) and item.total_thrust > 0 for item in active))
        self.assertTrue(any(item.trajectory_active for item in harness.received['computation_status']))
        self.assertAlmostEqual(harness.received['reference'][-1].pose.pose.position.z, 3., places=2)
        harness.kill = True
        harness.run(.15, sensors=True, rate=.5)
        self.assertFalse(harness.received['control_command'][-1].permit_override)
        self.assertFalse(harness.received['output_status'][-1].override_active)
        self.assertFalse(harness.received['computation_status'][-1].trajectory_active)


if __name__ == '__main__':
    unittest.main()
