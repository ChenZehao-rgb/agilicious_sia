"""Shared launch assembly for the two complete runtime profiles."""
import math
import os
from datetime import datetime
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULTS = dict(mode='sitl', runtime_config='', controller='', trajectory='', thrust_table='',
                shadow_only='', diagnostic_only='', sitl_delay_test='', record_bag='', bag_output='',
                device='', baud='', mavlink_device='', mavlink_baud='', mavlink_enabled='',
                mavlink_gps_mode='', mavlink_altitude_source='', mavlink_imu_rate_hz='',
                mavlink_gps_rate_hz='', mavlink_attitude_rate_hz='', altitude_source='',
                heading_confirmed='', heading_correction_rad='', fc_declination_applied='',
                aux_low='', aux_high='', params_dir='', pilot_config='', bridge_config='',
                gps_mode='', imu_rate_hz='', gps_rate_hz='', attitude_rate_hz='')


def boolean(value):
    if isinstance(value, bool):
        return value
    if str(value).lower() not in ('true', 'false'):
        raise ValueError('expected true or false, got ' + str(value))
    return str(value).lower() == 'true'


def load_profile(mode, filename=''):
    if mode not in ('sitl', 'hardware'):
        raise ValueError('mode must be sitl or hardware')
    path = Path(filename) if filename else Path(__file__).resolve().parent.parent / 'config' / (
        'simulation.yaml' if mode == 'sitl' else 'hardware.yaml')
    path = path.resolve()
    with path.open() as stream:
        profile = yaml.safe_load(stream)
    if not isinstance(profile, dict) or profile.get('mode') != mode:
        raise ValueError('runtime_config mode must match mode: ' + str(path))
    for section in ('flight', 'pilot', 'bridge', 'fusion', 'output'):
        if not isinstance(profile.get(section), dict):
            raise ValueError('runtime_config requires mapping ' + section)
    return path, profile


def resolve_data_path(path, filename):
    if not filename:
        return ''
    value = Path(filename).expanduser()
    return str((value if value.is_absolute() else path.parent / value).resolve())


def selected_controller(profile, override=''):
    configuration = profile['pilot'].get('pipeline', {}).get('controller', {})
    if not isinstance(configuration, dict):
        raise ValueError('pilot.pipeline.controller must be a mapping')
    configured = configuration.get('type', '')
    if configured not in ('MPC', 'GEO'):
        raise ValueError('pilot.pipeline.controller.type must be MPC or GEO')
    selected = str(override).upper() if override else configured
    if selected not in ('MPC', 'GEO'):
        raise ValueError('controller must be MPC or GEO')
    if 'parameter_sets' in configuration:
        if 'parameters' in configuration or 'file' in configuration:
            raise ValueError('controller parameter_sets cannot be combined with parameters or file')
        sets = configuration['parameter_sets']
        if not isinstance(sets, dict) or not isinstance(sets.get(selected), dict) or not sets[selected]:
            raise ValueError('Missing controller parameter set: ' + selected)
        parameters = sets[selected]
        source = 'pilot.pipeline.controller.parameter_sets.' + selected
    else:
        if selected != configured:
            raise ValueError('Changing controller requires parameter_sets for the selected controller')
        if 'parameters' in configuration and 'file' in configuration:
            raise ValueError('controller parameters cannot be combined with file')
        parameters = configuration.get('parameters')
        if parameters is not None and not isinstance(parameters, dict):
            raise ValueError('controller parameters must be a mapping')
        if parameters is None and not configuration.get('file'):
            raise ValueError('controller requires parameter_sets, parameters or file')
        source = 'pilot.pipeline.controller.' + ('parameters' if parameters is not None else 'file')
    if selected == 'GEO' and parameters is not None and boolean(parameters.get('drag_compensation', False)):
        raise ValueError('GEO drag_compensation requires rotor RPM feedback unavailable in this ROS 2 pipeline')
    pipeline = profile['pilot']['pipeline']
    if any(pipeline.get(module, {}).get('type') != 'External' for module in ('estimator', 'bridge')):
        raise ValueError('Rate/thrust runtime requires External estimator and bridge')
    if pipeline.get('inner_controller', {}).get('type', 'None') != 'None':
        raise ValueError('Rate/thrust runtime does not support an inner controller')
    if profile['pilot'].get('guard', {}).get('type', 'None') != 'None':
        raise ValueError('Rate/thrust runtime does not support a guard')
    return selected, source


def checked_devices(output, mavlink):
    device, sensor_device = output.get('device', ''), mavlink.get('device', '')
    if not device or not sensor_device:
        raise ValueError('device and mavlink_device must be specified in hardware.yaml or launch arguments')
    same = os.path.realpath(device) == os.path.realpath(sensor_device)
    if os.path.exists(device) and os.path.exists(sensor_device):
        same = same or os.path.samefile(device, sensor_device)
    if same:
        raise ValueError('MSP and MAVLink must use different serial devices')


def checked_model(profile, controller=''):
    controller, _ = selected_controller(profile, controller)
    model = profile['pilot'].get('quadrotor', {})
    if not isinstance(model, dict):
        raise ValueError('pilot.quadrotor must contain the measured model')
    def finite(value):
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    required = ('mass', 'thrust_max')
    invalid = [key for key in required
               if not finite(model.get(key)) or model[key] <= 0]
    if finite(model.get('mass')) and model['mass'] >= 100:
        invalid.append('mass')
    if (not finite(model.get('thrust_min')) or model['thrust_min'] < 0 or
            (finite(model.get('thrust_max')) and model['thrust_min'] >= model['thrust_max'])):
        invalid.append('thrust_min/max')
    for key in ('omega_max',):
        value = model.get(key)
        if not isinstance(value, list) or len(value) != 3 or not all(finite(x) for x in value):
            invalid.append(key)
        elif any(x <= 0 for x in value):
            invalid.append(key)
    if invalid:
        raise ValueError('Missing or invalid measured pilot.quadrotor fields: ' + ', '.join(invalid) +
                         '; use diagnostic_only:=true before configuring the model')


def assemble(context, forced_shadow=False, sensor_only=False, msp_only=False):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    mode = arg('mode')
    if (forced_shadow or sensor_only or msp_only) and mode != 'hardware':
        raise ValueError('hardware diagnostic entrypoints require mode=hardware')
    for legacy in ('params_dir', 'pilot_config', 'bridge_config'):
        if arg(legacy):
            raise ValueError(legacy + ' is a legacy direct-node option; use runtime_config for the unified launch')
    path, profile = load_profile(mode, arg('runtime_config'))
    controller, controller_parameters = selected_controller(profile, arg('controller'))
    flight = dict(profile['flight'])
    for key in ('trajectory', 'thrust_table', 'bag_output'):
        if arg(key):
            flight[key] = arg(key)
    for key in ('shadow_only', 'diagnostic_only', 'sitl_delay_test', 'record_bag'):
        if arg(key):
            flight[key] = boolean(arg(key))
    if forced_shadow:
        if arg('shadow_only') and not boolean(arg('shadow_only')):
            raise ValueError('shadow.launch.py cannot enable hardware output')
        flight['shadow_only'] = True
    shadow = boolean(flight.get('shadow_only', False))
    diagnostic = boolean(flight.get('diagnostic_only', False))
    delay_test = boolean(flight.get('sitl_delay_test', False))
    if mode != 'sitl' and delay_test:
        raise ValueError('sitl_delay_test requires mode=sitl')
    if mode != 'hardware' and (shadow or diagnostic or (arg('mavlink_enabled') and boolean(arg('mavlink_enabled')))):
        raise ValueError('MAVLink, shadow and diagnostic modes require hardware mode')
    if mode == 'hardware' and arg('mavlink_enabled') and not boolean(arg('mavlink_enabled')):
        raise ValueError('The hardware profile requires MAVLink sensors')
    trajectory = resolve_data_path(path, flight.get('trajectory', ''))
    thrust_table = resolve_data_path(path, flight.get('thrust_table', ''))
    navigation_source = 'rtk' if mode == 'sitl' else 'gnss'
    common = dict(mode=mode, use_sim_time=mode == 'sitl', runtime_config=str(path),
                  controller=controller, navigation_source=navigation_source, sitl_delay_test=delay_test)
    output = dict(profile['output'])
    mavlink = dict(profile.get('mavlink', {}))
    navigation = dict(profile.get('navigation', {}))
    evidence = dict(profile.get('evidence', {}))
    for name in ('device', 'baud'):
        if arg(name):
            output[name] = int(arg(name)) if name == 'baud' else arg(name)
        if arg('mavlink_' + name):
            mavlink[name] = int(arg('mavlink_' + name)) if name == 'baud' else arg('mavlink_' + name)
    if sensor_only:
        # Keep the standalone sensor entrypoint's existing device/baud arguments.
        for name in ('device', 'baud', 'gps_mode', 'imu_rate_hz', 'gps_rate_hz', 'attitude_rate_hz'):
            if arg(name):
                mavlink[name] = int(arg(name)) if name == 'baud' or name.endswith('_hz') else arg(name)
    for name in ('gps_mode', 'altitude_source', 'imu_rate_hz', 'gps_rate_hz', 'attitude_rate_hz'):
        if arg('mavlink_' + name):
            mavlink[name] = int(arg('mavlink_' + name)) if name.endswith('_hz') else arg('mavlink_' + name)
    if arg('altitude_source'):
        mavlink['altitude_source'] = arg('altitude_source')
    for name in ('heading_confirmed', 'fc_declination_applied', 'heading_correction_rad'):
        if arg(name):
            navigation[name] = float(arg(name)) if name.endswith('_rad') else boolean(arg(name))
    for name in ('aux_low', 'aux_high'):
        if arg(name):
            evidence[name] = int(arg(name))
    if mode == 'hardware':
        if not sensor_only and not msp_only:
            checked_devices(output, mavlink)
        elif not (mavlink if sensor_only else output).get('device'):
            raise ValueError('The diagnostic serial device must be specified in hardware.yaml or launch arguments')
        if not sensor_only and mavlink.get('gps_mode') != 'gnss':
            raise ValueError('The hardware profile requires gps_mode=gnss')
    if not (diagnostic or sensor_only or msp_only):
        checked_model(profile, controller)
        if mode == 'hardware' and not shadow and not thrust_table:
            raise ValueError('Hardware output requires a measured flight.thrust_table; use diagnostic_only:=true for sensor checks')

    def node(executable, parameters):
        return Node(package='agi_ros2', executable=executable, output='screen', parameters=[parameters])

    processes = []
    if not (sensor_only or msp_only):
        processes.append(node('state_fusion_node', {**profile['fusion'], **common}))
        if not diagnostic:
            processes.append(node('control_node', {**common, 'shadow_only': shadow, 'trajectory': trajectory}))
            processes.append(node('command_output_node', {
                **output, **common, 'shadow_only': shadow, 'thrust_table': thrust_table}))
    if mode == 'hardware':
        if not msp_only:
            processes.append(node('mavlink_sensor_node', {**mavlink, 'use_sim_time': False}))
        if not (sensor_only or msp_only):
            processes.append(node('gnss_adapter.py', {**navigation, 'use_sim_time': False}))
        if diagnostic or msp_only:
            processes.append(node('betaflight_msp_node', {**output, 'mode': 'monitor', 'use_sim_time': False}))
        if not sensor_only:
            processes.append(node('msp_evidence.py', {**evidence, 'runtime_config': str(path), 'use_sim_time': False}))
    controller_model = 'ideal_rate_thrust' if controller == 'MPC' else 'geometric_rate_thrust'
    actions = [LogInfo(msg=f'Runtime configuration: {path}; mode={mode}; navigation={navigation_source}; '
                           f'controller={controller}; controller_parameters={controller_parameters}; '
                           f'controller_model={controller_model}; '
                           f'reference={trajectory or "hover"}; shadow={shadow}; diagnostic={diagnostic}')] + processes
    if boolean(flight.get('record_bag', True)):
        bag_output = resolve_data_path(path, flight.get('bag_output', ''))
        if not bag_output:
            bag_output = str(Path.home() / 'agi_bags' / datetime.now().strftime(mode + '_%Y%m%d_%H%M%S_%f'))
        Path(bag_output).parent.mkdir(parents=True, exist_ok=True)
        command = ['ros2', 'bag', 'record', '--all', '--include-hidden-topics', '--output', bag_output]
        if mode == 'sitl':
            command.append('--use-sim-time')
        recorder = ExecuteProcess(cmd=command, output='screen')
        actions.append(recorder)
        processes.append(recorder)
    actions.extend(RegisterEventHandler(OnProcessExit(target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason='flight component exited'))])) for process in processes)
    return actions


def description(*, shadow=False, sensor_only=False, msp_only=False):
    defaults = dict(DEFAULTS)
    if shadow or sensor_only or msp_only:
        defaults['mode'] = 'hardware'
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] + [
        OpaqueFunction(function=lambda context: assemble(context, shadow, sensor_only, msp_only))])
