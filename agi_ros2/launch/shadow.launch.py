"""Hardware telemetry and MPC evaluation. This entrypoint cannot enable output."""
import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    device, mavlink = arg('device'), arg('mavlink_device')
    if not device or not mavlink:
        raise ValueError('device and mavlink_device must identify different dedicated UARTs')
    if os.path.realpath(device) == os.path.realpath(mavlink) or (
            os.path.exists(device) and os.path.exists(mavlink) and os.path.samefile(device, mavlink)):
        raise ValueError('MSP and MAVLink devices must differ')
    common = dict(mode='hardware', use_sim_time=False, shadow_only=True, sitl_delay_test=False,
                  params_dir=arg('params_dir'), pilot_config=arg('pilot_config'))
    bridge = str(Path(arg('params_dir')) / arg('bridge_config'))
    result = [
        Node(package='agi_ros2', executable='state_fusion_node', output='screen',
             parameters=[dict(mode='hardware', use_sim_time=False, navigation_source='gnss')]),
        Node(package='agi_ros2', executable='control_node', output='screen',
             parameters=[dict(common, trajectory=arg('trajectory'))]),
        Node(package='agi_ros2', executable='command_output_node', output='screen', parameters=[dict(
            common, device=device, baud=int(arg('baud')), bridge_config=arg('bridge_config'),
            thrust_table=arg('thrust_table'), **{
                'msp.read_configuration': True, 'msp.rc.rate_hz': 25., 'msp.status.rate_hz': 25.,
                'msp.battery.rate_hz': 2., 'msp.gps.enabled': False,
                'msp.attitude.enabled': False, 'msp.analog.enabled': False})]),
        Node(package='agi_ros2', executable='mavlink_sensor_node', output='screen', parameters=[dict(
            device=mavlink, baud=int(arg('mavlink_baud')), gps_mode='gnss',
            altitude_source=arg('altitude_source'), imu_rate_hz=500, gps_rate_hz=10, attitude_rate_hz=0)]),
        Node(package='agi_ros2', executable='gnss_adapter.py', output='screen', parameters=[dict(
            heading_confirmed=arg('heading_confirmed').lower() == 'true',
            heading_correction_rad=float(arg('heading_correction_rad')),
            fc_declination_applied=arg('fc_declination_applied').lower() == 'true')]),
        Node(package='agi_ros2', executable='msp_evidence.py', output='screen', parameters=[dict(
            bridge_config=bridge, aux_low=int(arg('aux_low')), aux_high=int(arg('aux_high')))]),
    ]
    if arg('record_bag').lower() == 'true':
        output = arg('bag_output') or str(Path.home() / 'agi_bags' / datetime.now().strftime('shadow_%Y%m%d_%H%M%S_%f'))
        Path(output).parent.mkdir(parents=True, exist_ok=True)
        result.append(ExecuteProcess(cmd=['ros2', 'bag', 'record', '--all', '--include-hidden-topics', '-o', output],
                                     output='screen'))
    for process in list(result):
        result.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='shadow component exited'))])))
    return result


def generate_launch_description():
    defaults = dict(device='', baud='921600', mavlink_device='', mavlink_baud='921600',
                    params_dir=get_package_share_directory('agi_ros2') + '/params',
                    pilot_config='pilot_ros2.yaml', bridge_config='betaflight_udp.yaml',
                    thrust_table='', trajectory='', altitude_source='unknown',
                    heading_confirmed='false', heading_correction_rad='0.0', fc_declination_applied='false',
                    aux_low='1700', aux_high='2100', record_bag='true', bag_output='')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
                             [OpaqueFunction(function=nodes)])
