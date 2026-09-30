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

    def test_direct_hardware_profile_rejects_invalid_quadratic_before_uart(self):
        document = yaml.safe_load((ROOT / 'agi_ros2/config/hardware.yaml').read_text())
        simulation = yaml.safe_load((ROOT / 'agi_ros2/config/simulation.yaml').read_text())
        document['pilot']['quadrotor'] = simulation['pilot']['quadrotor']
        document['bridge'] = simulation['bridge']
        valid = {'thrust_factor': 898 / 2231, 'max_total_thrust_n': 87.5145446}
        cases = [('missing_model', None), ('empty_model', {})]
        for key in valid:
            missing = dict(valid)
            del missing[key]
            cases.append(('missing_' + key, missing))
            for value in (True, False, '0.4', '.4junk', '87.5junk', float('nan'), float('inf')):
                cases.append((key + '=' + repr(value), {**valid, key: value}))
        cases.extend((name, {**valid, key: value}) for name, key, value in (
            ('negative_factor', 'thrust_factor', -0.1), ('large_factor', 'thrust_factor', 1.1),
            ('zero_maximum', 'max_total_thrust_n', 0.0), ('negative_maximum', 'max_total_thrust_n', -1.0)))
        with tempfile.TemporaryDirectory(prefix='agi_quadratic_profile_') as directory:
            profile = Path(directory) / 'hardware.yaml'
            missing_device = Path(directory) / 'no_physical_uart'
            self.assertFalse(missing_device.exists())
            for name, parameters in cases:
                with self.subTest(case=name):
                    if parameters is None:
                        document['flight'].pop('thrust_quadratic', None)
                    else:
                        document['flight']['thrust_quadratic'] = parameters
                    profile.write_text(dump_profile(document))
                    result = subprocess.run([
                        str(BIN / 'command_output_node'), '--ros-args', '-p', 'mode:=hardware',
                        '-p', 'runtime_config:=' + str(profile), '-p', 'device:=' + str(missing_device)],
                        capture_output=True, text=True, timeout=10)
                    output = result.stdout + result.stderr
                    self.assertNotEqual(result.returncode, 0, output)
                    self.assertRegex(output, 'quadratic thrust model|thrust_quadratic', output)
                    self.assertNotIn('open ' + str(missing_device), output,
                                     'Invalid model was accepted and reached the UART constructor')

            # A selected table in shadow may be absent; unselected estimates must not be parsed.
            document['flight'].update(thrust_model='table', thrust_table='', shadow_only=True,
                                      thrust_quadratic={'thrust_factor': '.4junk', 'max_total_thrust_n': True})
            profile.write_text(dump_profile(document))
            result = subprocess.run([
                str(BIN / 'command_output_node'), '--ros-args', '-p', 'mode:=hardware',
                '-p', 'runtime_config:=' + str(profile), '-p', 'device:=' + str(missing_device)],
                capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('open ' + str(missing_device), result.stdout + result.stderr)

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
