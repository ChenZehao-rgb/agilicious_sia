import os
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
              'params_dir': arg('params_dir'), 'pilot_config': arg('pilot_config'),
              'sitl_delay_test': mode == 'sitl' and arg('sitl_delay_test').lower() == 'true'}
    fusion = Node(package='agi_ros2', executable='state_fusion_node',
                  output='screen', parameters=[{'mode': mode, 'use_sim_time': mode == 'sitl',
                                                'sitl_delay_test': common['sitl_delay_test']}])
    control = Node(package='agi_ros2', executable='control_node', output='screen',
                   parameters=[{**common, 'trajectory': arg('trajectory')}])
    mavlink_enabled = arg('mavlink_enabled').lower() == 'true'
    msp_config = dict(MSP_CONFIG) if mode == 'hardware' else {}
    if mavlink_enabled:
        if mode != 'hardware':
            raise ValueError('MAVLink sensors may only be enabled in hardware mode')
        sensor_device = arg('mavlink_device')
        if not sensor_device:
            raise ValueError('mavlink_device must be specified')
        same = os.path.realpath(sensor_device) == os.path.realpath(arg('device'))
        if os.path.exists(sensor_device) and os.path.exists(arg('device')):
            same = same or os.path.samefile(sensor_device, arg('device'))
        if same:
            raise ValueError('MSP and MAVLink must use different serial devices')
        msp_config['msp.gps.enabled'] = False
        if int(arg('mavlink_attitude_rate_hz')) > 0:
            msp_config['msp.attitude.enabled'] = False
    output = Node(package='agi_ros2', executable='command_output_node',
                  output='screen', parameters=[{
                      **common, **msp_config, 'bridge_config': arg('bridge_config'),
                      'device': arg('device'), 'baud': int(arg('baud')),
                      'thrust_table': arg('thrust_table')}])
    result = [fusion, control, output]
    if mavlink_enabled:
        result.append(Node(package='agi_ros2', executable='mavlink_sensor_node', output='screen',
            parameters=[{'device': arg('mavlink_device'), 'baud': int(arg('mavlink_baud')),
                         'gps_mode': arg('mavlink_gps_mode'), 'altitude_source': arg('mavlink_altitude_source'),
                         'imu_rate_hz': int(arg('mavlink_imu_rate_hz')),
                         'gps_rate_hz': int(arg('mavlink_gps_rate_hz')),
                         'attitude_rate_hz': int(arg('mavlink_attitude_rate_hz')), 'use_sim_time': False}]))
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
        recorder = ExecuteProcess(cmd=command, output='screen')
        result.extend([recorder, RegisterEventHandler(OnProcessExit(target_action=recorder,
            on_exit=[EmitEvent(event=Shutdown(reason='flight recorder exited'))]))])
    return result


# 在这里修改 flight 参数；启动命令无需追加参数。
# Sensor 数据由外部 ROS topic 提供，不启动或检查 gazebo_sensors 进程。
# sitl_delay_test=true：取消 SITL 时间限制，延迟期间持续发送最后有效指令。
# 改为 false 恢复保护；hardware 模式始终保持保护。
# 实机输出串口上的遥测：enabled=False 或 rate_hz=0 关闭该类别。
# 不要同时启动 msp.launch.py；command_output_node 已独占该串口。
MSP_CONFIG = {
    'msp.response_timeout_ms': 100.0,
    'msp.attitude.enabled': True, 'msp.attitude.rate_hz': 10.0,
    'msp.rc.enabled': True,       'msp.rc.rate_hz': 10.0,
    'msp.status.enabled': True,   'msp.status.rate_hz': 5.0,
    'msp.analog.enabled': True,   'msp.analog.rate_hz': 2.0,
    'msp.battery.enabled': True,  'msp.battery.rate_hz': 2.0,
    'msp.gps.enabled': True,      'msp.gps.rate_hz': 2.0,
}

FLIGHT_CONFIG = dict(mavlink_enabled='false', mavlink_device='', mavlink_baud='921600',
                    mavlink_gps_mode='gnss', mavlink_altitude_source='unknown',
                    mavlink_imu_rate_hz='500', mavlink_gps_rate_hz='10', mavlink_attitude_rate_hz='0', mode='sitl', params_dir=get_package_share_directory('agi_ros2') + '/params',
                    pilot_config='pilot_ros2.yaml', bridge_config='betaflight_udp.yaml',
                    device='/dev/ttyAMA0', baud='921600', trajectory='/home/sia/agilicious_internal-main/miscellaneous/datasets/ref_trajs/open_source/HELIX_FWD50_50mps.csv', thrust_table='',
                    sitl_delay_test='true', record_bag='true', bag_output='')


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in FLIGHT_CONFIG.items()] +
                             [OpaqueFunction(function=nodes)])
