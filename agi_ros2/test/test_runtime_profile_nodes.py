#!/usr/bin/env python3
"""Launch-produced parameters on real core processes, with synthetic sensors and loopback UDP only."""
import math
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

import rclpy
import yaml
from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from agi_ros2.msg import ComputationStatus, ControlCommand, OutputStatus
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String

from test_node_pipeline import BIN, Harness
from test_runtime_config import ROOT, runtime


class RuntimeDumper(yaml.SafeDumper):
    """The agilib reader accepts block mappings and flow sequences."""


RuntimeDumper.add_representer(list, lambda dumper, value:
                             dumper.represent_sequence('tag:yaml.org,2002:seq', value, flow_style=True))


def dump_profile(document):
    return yaml.dump(document, Dumper=RuntimeDumper, default_flow_style=False)


class RuntimeProfileNodes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_default_empty_trajectory_reaches_hover_and_revokes_on_kill(self):
        self.check_hover('MPC')

    def test_geo_minimal_model_reaches_hover_and_revokes_on_kill(self):
        self.check_hover('GEO')

    def test_direct_profile_cannot_use_implicit_default_gains(self):
        document = yaml.safe_load((ROOT / 'agi_ros2/config/simulation.yaml').read_text())
        document['pilot']['pipeline']['controller'] = {'type': 'GEO'}
        with tempfile.TemporaryDirectory(prefix='agi_missing_gains_') as directory:
            profile = Path(directory) / 'simulation.yaml'
            profile.write_text(dump_profile(document))
            for executable in ('control_node', 'command_output_node'):
                result = subprocess.run([str(BIN / executable), '--ros-args', '-p', 'runtime_config:=' + str(profile)],
                                        capture_output=True, text=True, timeout=10)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('requires parameter_sets, parameters or file', result.stdout + result.stderr)

    def check_hover(self, controller):
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
        if controller == 'GEO':
            document = yaml.safe_load(content)
            # Remove all rigid-body/motor fields: GEO must not silently use an Iris/default model.
            model = document['pilot']['quadrotor']
            document['pilot']['quadrotor'] = {key: model[key] for key in
                                             ('mass', 'omega_max', 'thrust_min', 'thrust_max')}
            content = dump_profile(document)
        profile.write_text(content)
        context = LaunchContext()
        context.launch_configurations.update(runtime.DEFAULTS)
        context.launch_configurations.update(runtime_config=str(profile), record_bag='false', controller=controller)
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
        self.assertTrue(all(item.controller_type == controller for item in harness.received['computation_status']))
        self.assertTrue(any(item.controller_success for item in harness.received['computation_status']))
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
        for node in ('flight_control', 'command_output'):
            client = harness.node.create_client(SetParameters, node + '/set_parameters')
            self.assertTrue(client.wait_for_service(timeout_sec=2.))
            request = SetParameters.Request(parameters=[Parameter(name='controller', value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value='GEO' if controller == 'MPC' else 'MPC'))])
            future = client.call_async(request)
            harness.run(.3, sensors=True, rate=.5)
            self.assertTrue(future.done())
            self.assertFalse(future.result().results[0].successful, 'controller changed during a running session')
            harness.node.destroy_client(client)


if __name__ == '__main__':
    unittest.main()
