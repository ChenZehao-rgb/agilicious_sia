from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    mode = arg('mode')
    if mode not in ('sitl', 'hardware'):
        raise ValueError('mode must be sitl or hardware')
    common = {'mode': mode, 'use_sim_time': mode == 'sitl',
              'params_dir': arg('params_dir'), 'pilot_config': arg('pilot_config')}
    fusion = Node(package='agi_ros2', executable='state_fusion_node',
                  output='screen', parameters=[{'use_sim_time': mode == 'sitl'}])
    control = Node(package='agi_ros2', executable='control_node', output='screen',
                   parameters=[{**common, 'trajectory': arg('trajectory')}])
    output = Node(package='agi_ros2', executable='command_output_node',
                  output='screen', parameters=[{
                      **common, 'bridge_config': arg('bridge_config'),
                      'device': arg('device'), 'baud': int(arg('baud')),
                      'thrust_table': arg('thrust_table')}])
    result = [fusion, control, output]
    if mode == 'sitl' and arg('start_sensors').lower() == 'true':
        result.append(Node(package='agi_ros2', executable='gazebo_sensors', output='screen',
                           parameters=[{'config_verified': arg('sitl_config_verified').lower() == 'true'}]))
    for process in list(result):
        result.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='flight component exited'))])))
    if arg('record_bag').lower() == 'true':
        # Keep discovery enabled so topics appearing after startup are recorded too.
        command = ['ros2', 'bag', 'record', '--all', '--include-hidden-topics']
        bag_output = arg('bag_output')
        if not bag_output:
            bag_dir = Path('/home/sia/agilicious_internal-main/bags')
            bag_dir.mkdir(parents=True, exist_ok=True)
            bag_output = str(bag_dir / datetime.now().strftime('flight_%Y%m%d_%H%M%S_%f'))
        command.extend(['--output', bag_output])
        if mode == 'sitl':
            command.append('--use-sim-time')
        result.append(ExecuteProcess(cmd=command, output='screen'))
    return result


def generate_launch_description():
    defaults = dict(mode='sitl', params_dir=get_package_share_directory('agi_ros2') + '/params',
                    pilot_config='pilot_ros2.yaml', bridge_config='betaflight_udp.yaml',
                    device='/dev/ttyAMA0', baud='921600', trajectory='', thrust_table='',
                    sitl_config_verified='false', start_sensors='true',
                    record_bag='true', bag_output='')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
                             [OpaqueFunction(function=nodes)])
