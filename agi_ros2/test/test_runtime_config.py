#!/usr/bin/env python3
"""Profile and launch tests: construct actions without opening serial ports or starting ROS processes."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

import yaml
from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'betaflight_sitl'))


def import_path(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


runtime = import_path('runtime_launch_test', ROOT / 'agi_ros2/launch/runtime_launch.py')
sitl = import_path('sitl_run_test', ROOT / 'betaflight_sitl/run.py')


class RuntimeProfiles(unittest.TestCase):
    def context(self, **values):
        context = LaunchContext()
        context.launch_configurations.update(runtime.DEFAULTS)
        context.launch_configurations.update(record_bag='false')
        context.launch_configurations.update(values)
        return context

    def actions(self, **values):
        context = self.context(**values)
        nodes = [node for node in runtime.assemble(context) if isinstance(node, Node)]
        return {node.node_executable: evaluate_parameters(context, node._Node__parameters)[0] for node in nodes}

    def test_simulation_matches_existing_model_controller_and_fc_settings(self):
        path, profile = runtime.load_profile('sitl')
        params = ROOT / 'agilib/params'
        self.assertEqual(profile['pilot']['quadrotor'], yaml.safe_load((params / 'quads/betaloop_iris.yaml').read_text()))
        self.assertEqual(profile['pilot']['pipeline']['controller']['parameters'],
                         yaml.safe_load((params / 'mpc_betaflight_sitl.yaml').read_text()))
        self.assertEqual(sitl.expected_betaflight_settings(path), sitl.expected_betaflight_settings(params / 'betaflight_udp.yaml'))
        self.assertEqual(profile['flight']['trajectory'], '')
        self.assertFalse(profile['flight']['sitl_delay_test'])

    def test_simulation_remains_three_independent_core_nodes(self):
        nodes = self.actions()
        self.assertEqual(set(nodes), {'state_fusion_node', 'control_node', 'command_output_node'})
        for parameters in nodes.values():
            self.assertTrue(parameters['use_sim_time'])
            self.assertEqual(parameters['navigation_source'], 'rtk')
            self.assertEqual(Path(parameters['runtime_config']).name, 'simulation.yaml')
            self.assertNotIn('params_dir', parameters)
        self.assertEqual(nodes['control_node']['trajectory'], '')

    def test_hardware_placeholders_reject_control_but_allow_diagnostics(self):
        values = dict(mode='hardware', device='/dev/null', mavlink_device='/dev/zero')
        with self.assertRaisesRegex(ValueError, 'measured'):
            self.actions(**values)
        nodes = self.actions(**values, diagnostic_only='true')
        self.assertEqual(set(nodes), {'state_fusion_node', 'mavlink_sensor_node', 'gnss_adapter.py',
                                    'betaflight_msp_node', 'msp_evidence.py'})
        self.assertEqual(nodes['betaflight_msp_node']['mode'], 'monitor')
        self.assertTrue(nodes['betaflight_msp_node']['msp.read_configuration'])
        self.assertEqual(nodes['state_fusion_node']['navigation_source'], 'gnss')
        self.assertEqual(nodes['gnss_adapter.py']['max_horizontal_accuracy'], 0.0)

    def test_hardware_full_graph_and_relative_data_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            _, profile = runtime.load_profile('hardware')
            _, simulation = runtime.load_profile('sitl')
            profile['pilot']['quadrotor'] = simulation['pilot']['quadrotor']
            profile['bridge'] = simulation['bridge']
            profile['flight']['thrust_table'] = 'measured.csv'
            path = Path(directory) / 'hardware.yaml'
            path.write_text(yaml.safe_dump(profile))
            nodes = self.actions(mode='hardware', runtime_config=str(path), device='/dev/null', mavlink_device='/dev/zero')
            self.assertEqual(len(nodes), 6)
            for name in ('state_fusion_node', 'control_node', 'command_output_node'):
                self.assertEqual(nodes[name]['navigation_source'], 'gnss')
                self.assertFalse(nodes[name]['use_sim_time'])
                self.assertEqual(nodes[name]['runtime_config'], str(path))
            self.assertEqual(nodes['command_output_node']['thrust_table'], str(Path(directory) / 'measured.csv'))
            self.assertEqual(nodes['msp_evidence.py']['runtime_config'], str(path))

    def test_conflicting_modes_or_legacy_configuration_rejected(self):
        for values in ({'shadow_only': 'true'}, {'diagnostic_only': 'true'}, {'mavlink_enabled': 'true'},
                       {'mode': 'hardware', 'sitl_delay_test': 'true'}, {'params_dir': '/tmp/old_params'}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.actions(**values)
        path, _ = runtime.load_profile('hardware')
        with self.assertRaisesRegex(ValueError, 'mode'):
            runtime.load_profile('sitl', str(path))
        with self.assertRaisesRegex(RuntimeError, 'simulation'):
            sitl.expected_betaflight_settings(path)

    def test_same_uart_alias_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            device = Path(directory) / 'uart'
            alias = Path(directory) / 'alias'
            device.touch()
            alias.symlink_to(device)
            with self.assertRaisesRegex(ValueError, 'different'):
                self.actions(mode='hardware', device=str(device), mavlink_device=str(alias), diagnostic_only='true')

    def test_shadow_wrapper_cannot_enable_output(self):
        context = self.context(mode='hardware', shadow_only='false')
        with self.assertRaisesRegex(ValueError, 'cannot enable'):
            runtime.assemble(context, forced_shadow=True)

    def test_sensor_entrypoint_uses_same_hardware_profile_without_model(self):
        context = self.context(mode='hardware', device='/dev/null', baud='115200', gps_mode='rtk', imu_rate_hz='250')
        nodes = [node for node in runtime.assemble(context, sensor_only=True) if isinstance(node, Node)]
        self.assertEqual([node.node_executable for node in nodes], ['mavlink_sensor_node'])
        parameters = evaluate_parameters(context, nodes[0]._Node__parameters)[0]
        self.assertEqual(parameters['device'], '/dev/null')
        self.assertEqual(parameters['baud'], 115200)
        self.assertEqual(parameters['gps_mode'], 'rtk')
        self.assertEqual(parameters['imu_rate_hz'], 250)


if __name__ == '__main__':
    unittest.main()
