#!/usr/bin/env python3
"""Profile and launch tests: construct actions without opening serial ports or starting ROS processes."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

import yaml
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions
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
        plant = yaml.safe_load((params / 'quads/betaloop_iris.yaml').read_text())
        self.assertEqual(profile['pilot']['quadrotor'],
                         {key: plant[key] for key in ('mass', 'omega_max', 'thrust_min', 'thrust_max')})
        self.assertEqual(profile['pilot']['pipeline']['controller']['parameter_sets']['MPC'],
                         yaml.safe_load((params / 'mpc_betaflight_sitl.yaml').read_text()))
        self.assertEqual(sitl.expected_betaflight_settings(path), sitl.expected_betaflight_settings(params / 'betaflight_udp.yaml'))
        self.assertEqual(profile['flight']['trajectory'], '')
        self.assertFalse(profile['flight']['sitl_delay_test'])
        self.assertEqual({p.name for p in path.parent.glob('*.yaml')}, {'simulation.yaml', 'hardware.yaml'})

    def test_simulation_remains_three_independent_core_nodes(self):
        nodes = self.actions()
        self.assertEqual(set(nodes), {'state_fusion_node', 'control_node', 'command_output_node'})
        for parameters in nodes.values():
            self.assertTrue(parameters['use_sim_time'])
            self.assertEqual(parameters['navigation_source'], 'rtk')
            self.assertEqual(Path(parameters['runtime_config']).name, 'simulation.yaml')
            self.assertNotIn('params_dir', parameters)
        self.assertEqual(nodes['control_node']['trajectory'], '')
        self.assertEqual(nodes['control_node']['controller'], 'MPC')
        self.assertEqual(nodes['command_output_node']['controller'], 'MPC')

    def test_controller_override_reaches_control_and_output(self):
        for controller in ('geo', 'GEO', 'Geo', 'mpc'):
            with self.subTest(controller=controller):
                nodes = self.actions(controller=controller)
                self.assertEqual(nodes['control_node']['controller'], controller.upper())
                self.assertEqual(nodes['command_output_node']['controller'], controller.upper())
        with self.assertRaisesRegex(ValueError, 'MPC or GEO'):
            self.actions(controller='pid')

    def test_profile_controller_wins_without_override(self):
        with tempfile.TemporaryDirectory() as directory:
            _, profile = runtime.load_profile('sitl')
            profile['pilot']['pipeline']['controller']['type'] = 'GEO'
            path = Path(directory) / 'simulation.yaml'
            path.write_text(yaml.safe_dump(profile))
            nodes = self.actions(runtime_config=str(path))
            self.assertEqual(nodes['control_node']['controller'], 'GEO')
            self.assertEqual(nodes['command_output_node']['controller'], 'GEO')
            self.assertEqual(self.actions(runtime_config=str(path), controller='mpc')['control_node']['controller'], 'MPC')
            selected, arguments = sitl.ros2_flight_selection(path)
            self.assertEqual(selected, 'GEO')
            self.assertEqual(arguments, ['runtime_config:=' + str(path)])
            selected, arguments = sitl.ros2_flight_selection(path, 'mpc')
            self.assertEqual(selected, 'MPC')
            self.assertEqual(arguments[-1], 'controller:=MPC')

    def test_only_selected_parameter_set_is_required(self):
        _, profile = runtime.load_profile('sitl')
        configuration = profile['pilot']['pipeline']['controller']
        configuration['parameter_sets']['GEO'] = None
        self.assertEqual(runtime.selected_controller(profile)[0], 'MPC')
        with self.assertRaisesRegex(ValueError, 'parameter set: GEO'):
            runtime.selected_controller(profile, 'geo')
        configuration['parameter_sets']['GEO'] = {'drag_compensation': True}
        self.assertEqual(runtime.selected_controller(profile)[0], 'MPC')
        with self.assertRaisesRegex(ValueError, 'rotor RPM'):
            runtime.selected_controller(profile, 'geo')
        del configuration['parameter_sets']['MPC']
        with self.assertRaisesRegex(ValueError, 'parameter set: MPC'):
            runtime.selected_controller(profile)

    def test_parameter_sources_are_exclusive_and_legacy_type_cannot_switch(self):
        for legacy in ('parameters', 'file'):
            with self.subTest(legacy=legacy):
                _, profile = runtime.load_profile('sitl')
                configuration = profile['pilot']['pipeline']['controller']
                configuration[legacy] = {} if legacy == 'parameters' else 'old.yaml'
                with self.assertRaisesRegex(ValueError, 'cannot be combined'):
                    runtime.selected_controller(profile)
                del configuration['parameter_sets']
                self.assertEqual(runtime.selected_controller(profile, 'mpc')[0], 'MPC')
                with self.assertRaisesRegex(ValueError, 'Changing controller'):
                    runtime.selected_controller(profile, 'geo')

    def test_both_controllers_require_only_active_rate_thrust_model_fields(self):
        _, profile = runtime.load_profile('hardware')
        profile['pilot']['quadrotor'] = {
            'mass': 1.0, 'omega_max': [1.0, 1.0, 1.0], 'thrust_min': 0.1, 'thrust_max': 5.0}
        for controller in ('MPC', 'GEO'):
            runtime.checked_model(profile, controller)
            for key, value in (('mass', 0.0), ('mass', 100.0), ('mass', float('nan')),
                               ('thrust_min', -0.1), ('thrust_min', 5.0), ('thrust_max', 0.1),
                               ('omega_max', [1.0, 0.0, 1.0])):
                previous = profile['pilot']['quadrotor'][key]
                profile['pilot']['quadrotor'][key] = value
                with self.subTest(controller=controller, key=key, value=value), self.assertRaisesRegex(ValueError, 'measured'):
                    runtime.checked_model(profile, controller)
                profile['pilot']['quadrotor'][key] = previous

    def test_geo_hardware_still_rejects_missing_mass_limits_or_thrust_table(self):
        values = dict(mode='hardware', device='/dev/null', mavlink_device='/dev/zero', controller='geo')
        with self.assertRaisesRegex(ValueError, 'measured'):
            self.actions(**values)
        with tempfile.TemporaryDirectory() as directory:
            _, profile = runtime.load_profile('hardware')
            profile['pilot']['quadrotor'] = {
                'mass': 1.0, 'omega_max': [1.0, 1.0, 1.0], 'thrust_min': 0.0, 'thrust_max': 5.0}
            del profile['flight']['thrust_model']
            path = Path(directory) / 'hardware.yaml'
            path.write_text(yaml.safe_dump(profile))
            nodes = self.actions(**values, runtime_config=str(path))
            self.assertEqual(len(nodes), 7)
            self.assertTrue(nodes['command_output_node']['shadow_only'])
            with self.assertRaisesRegex(ValueError, 'measured flight.thrust_table'):
                self.actions(**values, runtime_config=str(path), shadow_only='false')

    def test_rate_thrust_controllers_reject_pipeline_that_requires_unavailable_full_model(self):
        for controller in ('MPC', 'GEO'):
            for module in ('estimator', 'bridge', 'inner_controller'):
                _, profile = runtime.load_profile('sitl')
                profile['pilot']['pipeline'][module] = {'type': 'Other'}
                with self.subTest(controller=controller, module=module), self.assertRaises(ValueError):
                    runtime.selected_controller(profile, controller)
            _, profile = runtime.load_profile('sitl')
            profile['pilot']['guard'] = {'type': 'Other'}
            with self.subTest(controller=controller), self.assertRaisesRegex(ValueError, 'guard'):
                runtime.selected_controller(profile, controller)

    def test_simulation_geo_gains_have_explicit_runtime_limits(self):
        _, profile = runtime.load_profile('sitl')
        params = profile['pilot']['pipeline']['controller']['parameter_sets']['GEO']
        legacy = yaml.safe_load((ROOT / 'agilib/params/geo_betaflight_sitl.yaml').read_text())
        for key in ('kpacc', 'kdacc', 'kpatt_xy', 'kpatt_z', 'kprate', 'p_err_max', 'v_err_max'):
            self.assertEqual(params[key], legacy[key])
        self.assertEqual(params['filter_sampling_frequency'], 100)
        self.assertEqual(params['filter_cutoff_frequency'], 10)
        self.assertEqual(params['max_tilt_rad'], 0.35)
        self.assertFalse(params['drag_compensation'])

    def test_hardware_placeholders_reject_control_but_allow_diagnostics(self):
        values = dict(mode='hardware', device='/dev/null', mavlink_device='/dev/zero')
        with self.assertRaisesRegex(ValueError, 'measured'):
            self.actions(**values)
        nodes = self.actions(**values, diagnostic_only='true')
        self.assertEqual(set(nodes), {'state_fusion_node', 'mavlink_sensor_node', 'gnss_adapter.py',
                                    'betaflight_msp_node', 'msp_evidence.py', 'static_transform_publisher'})
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
            profile['flight']['thrust_model'] = 'table'
            profile['flight']['thrust_table'] = 'measured.csv'
            path = Path(directory) / 'hardware.yaml'
            path.write_text(yaml.safe_dump(profile))
            nodes = self.actions(mode='hardware', runtime_config=str(path), device='/dev/null', mavlink_device='/dev/zero')
            self.assertEqual(len(nodes), 7)
            for name in ('state_fusion_node', 'control_node', 'command_output_node'):
                self.assertEqual(nodes[name]['navigation_source'], 'gnss')
                self.assertFalse(nodes[name]['use_sim_time'])
                self.assertEqual(nodes[name]['runtime_config'], str(path))
            self.assertEqual(nodes['command_output_node']['thrust_table'], str(Path(directory) / 'measured.csv'))
            self.assertEqual(nodes['command_output_node']['thrust_model'], 'table')
            self.assertEqual(nodes['msp_evidence.py']['runtime_config'], str(path))

    def test_hardware_defaults_to_estimate_without_changing_controller_limits_or_shadow(self):
        _, profile = runtime.load_profile('hardware')
        self.assertEqual(profile['flight']['thrust_model'], 'quadratic')
        self.assertTrue(profile['flight']['shadow_only'])
        self.assertEqual(profile['pilot']['quadrotor']['thrust_max'], 0.0)
        self.assertEqual(profile['pilot']['quadrotor']['omega_max'], [0.0, 0.0, 0.0])

    def test_quadratic_launch_passes_parameters_and_ignores_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            _, profile = runtime.load_profile('hardware')
            profile['pilot']['quadrotor'] = {
                'mass': 0.734, 'omega_max': [1.0, 1.0, 1.0], 'thrust_min': 0.0, 'thrust_max': 5.0}
            # This would fail path resolution if an unselected table were inspected.
            profile['flight']['thrust_table'] = {'not': 'a path'}
            path = Path(directory) / 'hardware.yaml'
            path.write_text(yaml.safe_dump(profile))
            for shadow in ('true', 'false'):
                with self.subTest(shadow=shadow):
                    nodes = self.actions(mode='hardware', runtime_config=str(path), device='/dev/null',
                                         mavlink_device='/dev/zero', shadow_only=shadow)
                    output = nodes['command_output_node']
                    self.assertEqual(output['thrust_model'], 'quadratic')
                    self.assertEqual(output['thrust_table'], '')
                    self.assertEqual(output['thrust_quadratic.thrust_factor'], 898 / 2231)
                    self.assertEqual(output['thrust_quadratic.max_total_thrust_n'], 87.5145446)

    def test_thrust_model_launch_override_selects_only_requested_model(self):
        with tempfile.TemporaryDirectory() as directory:
            _, profile = runtime.load_profile('hardware')
            profile['pilot']['quadrotor'] = {
                'mass': 0.734, 'omega_max': [1.0, 1.0, 1.0], 'thrust_min': 0.0, 'thrust_max': 5.0}
            profile['flight']['thrust_quadratic'] = None
            profile['flight']['thrust_table'] = 'measured.csv'
            path = Path(directory) / 'hardware.yaml'
            path.write_text(yaml.safe_dump(profile))
            nodes = self.actions(mode='hardware', runtime_config=str(path), device='/dev/null',
                                 mavlink_device='/dev/zero', thrust_model='table', shadow_only='false')
            output = nodes['command_output_node']
            self.assertEqual(output['thrust_model'], 'table')
            self.assertEqual(output['thrust_table'], str(Path(directory) / 'measured.csv'))
            self.assertNotIn('thrust_quadratic.thrust_factor', output)

    def test_quadratic_parameters_are_required_and_validated_without_fallback(self):
        path, profile = runtime.load_profile('hardware')
        flight = profile['flight']
        flight['thrust_table'] = 'measured.csv'
        for key in ('thrust_factor', 'max_total_thrust_n'):
            valid = flight['thrust_quadratic'][key]
            for invalid in (None, True, '0.5', float('nan'), float('inf'), float('-inf')):
                with self.subTest(key=key, value=invalid):
                    flight['thrust_quadratic'][key] = invalid
                    with self.assertRaisesRegex(ValueError, key):
                        runtime.checked_thrust_mapping(flight, 'hardware', path, True)
            del flight['thrust_quadratic'][key]
            with self.assertRaisesRegex(ValueError, key):
                runtime.checked_thrust_mapping(flight, 'hardware', path, True)
            flight['thrust_quadratic'][key] = valid
        for key, invalid in (('thrust_factor', -0.01), ('thrust_factor', 1.01),
                             ('max_total_thrust_n', 0), ('max_total_thrust_n', -1)):
            valid = flight['thrust_quadratic'][key]
            flight['thrust_quadratic'][key] = invalid
            with self.subTest(key=key, value=invalid), self.assertRaisesRegex(ValueError, key):
                runtime.checked_thrust_mapping(flight, 'hardware', path, True)
            flight['thrust_quadratic'][key] = valid
        for invalid in (None, [], 'model'):
            flight['thrust_quadratic'] = invalid
            with self.subTest(value=invalid), self.assertRaisesRegex(ValueError, 'flight.thrust_quadratic'):
                runtime.checked_thrust_mapping(flight, 'hardware', path, True)
        del flight['thrust_quadratic']
        with self.assertRaisesRegex(ValueError, 'flight.thrust_quadratic'):
            runtime.checked_thrust_mapping(flight, 'hardware', path, True)

    def test_thrust_model_selection_and_quadratic_coefficient_endpoints(self):
        path, profile = runtime.load_profile('hardware')
        flight = profile['flight']
        for coefficient in (0, 1):
            flight['thrust_quadratic']['thrust_factor'] = coefficient
            parameters = runtime.checked_thrust_mapping(flight, 'hardware', path, True)
            self.assertEqual(parameters['thrust_quadratic.thrust_factor'], coefficient)
            self.assertIsInstance(parameters['thrust_quadratic.thrust_factor'], float)
        with self.assertRaisesRegex(ValueError, 'requires mode=hardware'):
            runtime.checked_thrust_mapping(flight, 'sitl', path, False)
        for invalid in ('', 'unknown', None):
            flight['thrust_model'] = invalid
            with self.subTest(value=invalid), self.assertRaisesRegex(ValueError, 'table or quadratic'):
                runtime.checked_thrust_mapping(flight, 'hardware', path, True)
        del flight['thrust_model']
        parameters = runtime.checked_thrust_mapping(flight, 'hardware', path, True)
        self.assertEqual(parameters, {'thrust_model': 'table', 'thrust_table': ''})
        with self.assertRaisesRegex(ValueError, 'measured flight.thrust_table'):
            runtime.checked_thrust_mapping(flight, 'hardware', path, False)

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
        context = self.context(mode='hardware', device='/dev/null', baud='115200', gps_mode='rtk',
                               imu_rate_hz='250', baro_rate_hz='20')
        nodes = [node for node in runtime.assemble(context, sensor_only=True) if isinstance(node, Node)]
        self.assertEqual([node.node_executable for node in nodes], ['mavlink_sensor_node', 'static_transform_publisher'])
        parameters = evaluate_parameters(context, nodes[0]._Node__parameters)[0]
        self.assertEqual(parameters['device'], '/dev/null')
        self.assertEqual(parameters['baud'], 115200)
        self.assertEqual(parameters['gps_mode'], 'rtk')
        self.assertEqual(parameters['imu_rate_hz'], 250)
        self.assertEqual(parameters['baro_rate_hz'], 20)

    def test_hardware_baro_fusion_and_receiver_parameters_reach_nodes(self):
        nodes = self.actions(mode='hardware', device='/dev/null', mavlink_device='/dev/zero', diagnostic_only='true')
        self.assertTrue(nodes['state_fusion_node']['baro_enabled'])
        self.assertEqual(nodes['state_fusion_node']['observation_delay'], 0.20)
        self.assertEqual(nodes['state_fusion_node']['baro_max_age'], 0.25)
        self.assertEqual(nodes['state_fusion_node']['baro_pressure_variance_pa2'], 4.0)
        self.assertEqual(nodes['mavlink_sensor_node']['baro_rate_hz'], 40)
        self.assertEqual(nodes['mavlink_sensor_node']['baro_max_age_s'], 0.25)
        self.assertEqual(nodes['mavlink_sensor_node']['baro_pressure_variance_pa2'], 0.0)
        self.assertFalse(nodes['static_transform_publisher']['use_sim_time'])
        nodes = self.actions(mode='hardware', device='/dev/null', mavlink_device='/dev/zero',
                             diagnostic_only='true', mavlink_baro_rate_hz='0')
        self.assertEqual(nodes['mavlink_sensor_node']['baro_rate_hz'], 0)

    def test_baro_transform_connects_sensor_frame_without_affecting_msp_only(self):
        context = self.context(mode='hardware', device='/dev/null')
        nodes = [node for node in runtime.assemble(context, sensor_only=True) if isinstance(node, Node)]
        transform = next(node for node in nodes if node.node_executable == 'static_transform_publisher')
        command = transform._Node__arguments
        self.assertEqual(command[command.index('--frame-id') + 1], 'base_link')
        self.assertEqual(command[command.index('--child-frame-id') + 1], 'baro_link')
        for axis in ('--x', '--y', '--z', '--roll', '--pitch', '--yaw'):
            self.assertEqual(command[command.index(axis) + 1], '0')
        nodes = [node for node in runtime.assemble(context, msp_only=True) if isinstance(node, Node)]
        self.assertNotIn('static_transform_publisher', [node.node_executable for node in nodes])

    def test_recording_includes_new_baro_topics_and_static_transform(self):
        with tempfile.TemporaryDirectory() as directory:
            context = self.context(mode='hardware', device='/dev/null', mavlink_device='/dev/zero',
                                   diagnostic_only='true', record_bag='true', bag_output=directory + '/bag')
            recorders = [action for action in runtime.assemble(context)
                         if isinstance(action, ExecuteProcess) and not isinstance(action, Node)]
            self.assertEqual(len(recorders), 1)
            command = [perform_substitutions(context, part) for part in recorders[0].cmd]
            # All-topic recording also discovers baro diagnostics and transient-local /tf_static.
            self.assertEqual(command[:3], ['ros2', 'bag', 'record'])
            self.assertIn('--all', command)
            self.assertIn('--include-hidden-topics', command)


if __name__ == '__main__':
    unittest.main()
