#!/usr/bin/env python3
"""Profile and launch tests: construct actions without opening serial ports or starting ROS processes."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
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
SMALL_THRUST = {'a1_n': 3.0411331993565565, 'a2_n': 138.51944503645788, 'a3_n': -65.93137222790723,
                'reference_voltage_v': 24.0, 'voltage_exponent_p': 1.253471255346232}


class RuntimeProfiles(unittest.TestCase):
    def test_all_launch_entrypoints_assemble_without_clock_service(self):
        core = {'mavlink_sensor_node', 'fc_state_adapter_node', 'control_node', 'command_output_node'}
        cases = [('flight', 'hardware', core | {'msp_evidence.py'}),
                 ('flight', 'sitl', core),
                 ('shadow', 'hardware', core | {'msp_evidence.py'}),
                 ('mavlink_sensors', 'hardware', {'mavlink_sensor_node'}),
                 ('msp', 'hardware', {'betaflight_msp_node'})]
        for entry, mode, expected in cases:
            with self.subTest(entry=entry, mode=mode):
                module = import_path('offline_' + entry, ROOT / 'agi_ros2/launch' / (entry + '.launch.py'))
                context = self.context(mode=mode, state_source='fc_fused')
                actions = []
                # Execute only declarations and assembly, never the returned
                # device nodes or process actions. No NTP service is required.
                with patch('subprocess.run', side_effect=AssertionError('Launch must not query or change clock services')):
                    for entity in module.generate_launch_description().entities:
                        if isinstance(entity, DeclareLaunchArgument):
                            entity.execute(context)
                        else:
                            self.assertIsInstance(entity, OpaqueFunction)
                            actions.extend(entity.execute(context))
                nodes = [action for action in actions if isinstance(action, Node)]
                self.assertEqual({node.node_executable for node in nodes}, expected)
                self.assertFalse(any(isinstance(action, ExecuteProcess) and not isinstance(action, Node)
                                     for action in actions))
                if entry == 'shadow':
                    for node in nodes:
                        if node.node_executable in ('control_node', 'command_output_node'):
                            self.assertTrue(evaluate_parameters(context, node._Node__parameters)[0]['shadow_only'])

    def context(self, **values):
        context = LaunchContext()
        context.launch_configurations.update(runtime.DEFAULTS)
        # These existing graph/bench tests exercise the retained ROS EKF path.
        # FC default and mutual exclusion are covered in test_fc_fused_launch.py.
        context.launch_configurations.update(record_bag='false', state_source='ros_ekf')
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

    def test_bench_fixed_gps_only_changes_sensor_input(self):
        values = dict(mode='hardware', device='/dev/null', mavlink_device='/dev/zero', shadow_only='false')
        normal = self.actions(**values)
        bench = self.actions(**values, bench_fixed_gps='true', bench_gps_latitude='31.0',
                             bench_gps_longitude='121.0', bench_gps_altitude_msl='20.0')
        self.assertEqual(set(normal), set(bench))
        for name in normal:
            if name != 'mavlink_sensor_node':
                self.assertEqual(normal[name], bench[name])
        self.assertFalse(normal['mavlink_sensor_node']['bench_fixed_gps'])
        self.assertTrue(bench['mavlink_sensor_node']['bench_fixed_gps'])
        self.assertEqual(bench['mavlink_sensor_node']['attitude_rate_hz'], 10)
        self.assertEqual(bench['mavlink_sensor_node']['bench_gps_altitude_msl'], 20.)
        for overrides in (dict(mode='sitl'), dict(mavlink_gps_rate_hz='0'),
                          dict(mavlink_gps_mode='rtk'), dict(altitude_source='unknown')):
            with self.subTest(overrides=overrides), self.assertRaisesRegex(ValueError, 'bench_fixed_gps'):
                self.actions(**{**values, **overrides, 'bench_fixed_gps': 'true'})

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

    def test_hardware_telemetry_has_no_readback_configuration(self):
        _, profile = runtime.load_profile('hardware')
        self.assertFalse(any(key.startswith('msp.') for key in profile['output']))
        self.assertEqual(profile['evidence']['aux_low'], 1875)
        self.assertEqual(profile['evidence']['aux_high'], 2100)
        self.assertFalse(set(profile['evidence']) & {'arm_aux', 'auto_aux', 'kill_aux', 'rx_map',
                                                    'expected_pid_profile', 'expected_rate_profile',
                                                    'expected_override_timeout_ms'})
        nodes = self.actions(mode='hardware', diagnostic_only='true', state_source='fc_fused')
        self.assertIn('mavlink_sensor_node', nodes)
        self.assertIn('msp_evidence.py', nodes)
        self.assertFalse(any(key.startswith('msp.') for key in nodes['betaflight_msp_node']))
        self.assertEqual(set(nodes),
                         {'mavlink_sensor_node', 'fc_state_adapter_node', 'betaflight_msp_node', 'msp_evidence.py'})

    def test_hardware_placeholders_reject_control_but_allow_diagnostics(self):
        values = dict(mode='hardware', device='/dev/null', mavlink_device='/dev/zero')
        with self.assertRaisesRegex(ValueError, 'measured'):
            self.actions(**values)
        nodes = self.actions(**values, diagnostic_only='true')
        self.assertEqual(set(nodes), {'state_fusion_node', 'mavlink_sensor_node', 'gnss_adapter.py',
                                    'betaflight_msp_node', 'msp_evidence.py', 'static_transform_publisher'})
        self.assertEqual(nodes['betaflight_msp_node']['mode'], 'monitor')
        self.assertFalse(any(key.startswith('msp.') for key in nodes['betaflight_msp_node']))
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
            for parameters in (profile['flight']['thrust_quadratic'], SMALL_THRUST,
                               dict(SMALL_THRUST, airframe='small')):
                profile['flight']['thrust_quadratic'] = parameters
                path.write_text(yaml.safe_dump(profile))
                for shadow in ('true', 'false'):
                    with self.subTest(airframe=parameters.get('airframe', 'default'), shadow=shadow):
                        nodes = self.actions(mode='hardware', runtime_config=str(path), device='/dev/null',
                                             mavlink_device='/dev/zero', shadow_only=shadow)
                        output = nodes['command_output_node']
                        self.assertEqual(output['thrust_model'], 'quadratic')
                        self.assertEqual(output['thrust_table'], '')
                        self.assertEqual(output['thrust_quadratic.airframe'], parameters.get('airframe', 'small'))
                        for key, value in parameters.items():
                            self.assertEqual(output['thrust_quadratic.' + key], value)
                        if parameters.get('airframe') == 'big':
                            for key in ('a1_n', 'a2_n', 'a3_n'):
                                self.assertNotIn('thrust_quadratic.' + key, output)

    def test_hardware_big_mapping_has_no_cubic_coefficients(self):
        _, profile = runtime.load_profile('hardware')
        self.assertEqual(profile['flight']['thrust_model'], 'quadratic')
        self.assertEqual(profile['flight']['thrust_quadratic'],
                         {'airframe': 'big', 'reference_voltage_v': 24.0, 'voltage_exponent_p': 0.0})

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
            self.assertNotIn('thrust_quadratic.a1_n', output)

    def test_quadratic_parameters_are_required_and_validated_without_fallback(self):
        path, profile = runtime.load_profile('hardware')
        flight = profile['flight']
        flight['thrust_table'] = 'measured.csv'
        for parameters in (flight['thrust_quadratic'], dict(SMALL_THRUST)):
            flight['thrust_quadratic'] = parameters
            for key in set(parameters) - {'airframe'}:
                valid = parameters[key]
                for invalid in (None, True, '0.5', float('nan'), float('inf'), float('-inf')):
                    with self.subTest(airframe=parameters.get('airframe', 'default'), key=key, value=invalid):
                        parameters[key] = invalid
                        with self.assertRaisesRegex(ValueError, key):
                            runtime.checked_thrust_mapping(flight, 'hardware', path, True)
                del parameters[key]
                with self.assertRaisesRegex(ValueError, key):
                    runtime.checked_thrust_mapping(flight, 'hardware', path, True)
                parameters[key] = valid
            for invalid in (0, -1):
                valid = parameters['reference_voltage_v']
                parameters['reference_voltage_v'] = invalid
                with self.subTest(value=invalid), self.assertRaisesRegex(ValueError, 'reference_voltage_v'):
                    runtime.checked_thrust_mapping(flight, 'hardware', path, True)
                parameters['reference_voltage_v'] = valid
        for invalid in (None, [], 'model'):
            flight['thrust_quadratic'] = invalid
            with self.subTest(value=invalid), self.assertRaisesRegex(ValueError, 'flight.thrust_quadratic'):
                runtime.checked_thrust_mapping(flight, 'hardware', path, True)
        del flight['thrust_quadratic']
        with self.assertRaisesRegex(ValueError, 'flight.thrust_quadratic'):
            runtime.checked_thrust_mapping(flight, 'hardware', path, True)

    def test_thrust_model_selection_and_finite_cubic_coefficients(self):
        path, profile = runtime.load_profile('hardware')
        flight = profile['flight']
        flight['thrust_quadratic'] = dict(SMALL_THRUST)
        for coefficient in (-1, 0, 1):
            flight['thrust_quadratic']['a1_n'] = coefficient
            parameters = runtime.checked_thrust_mapping(flight, 'hardware', path, True)
            self.assertEqual(parameters['thrust_quadratic.a1_n'], coefficient)
            self.assertIsInstance(parameters['thrust_quadratic.a1_n'], float)
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

    def test_airframe_selection_and_unused_big_coefficients(self):
        path, profile = runtime.load_profile('hardware')
        flight = profile['flight']
        parameters = runtime.checked_thrust_mapping(flight, 'hardware', path, True)
        for key in ('a1_n', 'a2_n', 'a3_n'):
            flight['thrust_quadratic'][key] = 'unused by big'
        self.assertEqual(runtime.checked_thrust_mapping(flight, 'hardware', path, True), parameters)
        for invalid in ('', 'SMALL', 'unknown', None, True, 1, [], {}):
            flight['thrust_quadratic']['airframe'] = invalid
            with self.subTest(value=invalid), self.assertRaisesRegex(ValueError, 'airframe must be small or big'):
                runtime.checked_thrust_mapping(flight, 'hardware', path, True)

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
        self.assertEqual(nodes['state_fusion_node']['height_fusion_mode'], 'baro_gnss_weighted')
        self.assertEqual(nodes['state_fusion_node']['baro_bias_random_walk'], 0.0)
        self.assertEqual(nodes['state_fusion_node']['navigation_horizontal_nis_threshold'], 18.46682695290317)
        self.assertEqual(nodes['state_fusion_node']['navigation_position_nis_threshold'], 13.815510557964274)
        self.assertEqual(nodes['state_fusion_node']['navigation_velocity_nis_threshold'], 30.0)
        self.assertEqual(nodes['state_fusion_node']['navigation_heading_nis_threshold'], 10.827566170662733)
        self.assertEqual(nodes['state_fusion_node']['ekf_process_noise_model'], 'continuous_psd')
        self.assertEqual(nodes['state_fusion_node']['ekf_integration_step'], .001)
        self.assertEqual(nodes['state_fusion_node']['navigation_recovery_updates'], 5)
        self.assertEqual(nodes['state_fusion_node']['navigation_height_nis_threshold'], 10.828)
        self.assertEqual(nodes['state_fusion_node']['navigation_vertical_velocity_nis_threshold'], 10.828)
        self.assertEqual(nodes['state_fusion_node']['gnss_height_variance_floor'], 18.0)
        self.assertEqual(nodes['state_fusion_node']['gnss_height_variance_scale'], 4.5)
        self.assertEqual(nodes['state_fusion_node']['gnss_vertical_velocity_variance_floor'], 0.09)
        self.assertEqual(nodes['state_fusion_node']['gnss_vertical_velocity_variance_scale'], 2.25)
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
