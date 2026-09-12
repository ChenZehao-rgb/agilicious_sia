"""Edit this configuration, then launch without command-line parameters."""
from datetime import datetime
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

# 实机通信配置：不需要输入命令行参数。bench 前必须拆桨。
MSP_CONFIG = {
    'mode': 'monitor',                 # monitor / bench（固定 RC 台架测试）
    'device': '/dev/ttyAMA0',
    'baud': 115200,
    'use_sim_time': False,
    'props_removed': False,            # bench 必须显式改为 True
    'bench_aetr': [1500, 1500, 1000, 1500],  # A,E,T,R；固定 100 Hz，不写 AUX
    'msp.response_timeout_ms': 100.0,
    'msp.attitude.enabled': True, 'msp.attitude.rate_hz': 10.0,
    'msp.rc.enabled': True,       'msp.rc.rate_hz': 10.0,
    'msp.status.enabled': True,   'msp.status.rate_hz': 5.0,
    'msp.analog.enabled': True,   'msp.analog.rate_hz': 2.0,
    'msp.battery.enabled': True,  'msp.battery.rate_hz': 2.0,
    'msp.gps.enabled': True,      'msp.gps.rate_hz': 2.0,
}
RECORD_BAG = True
BAG_ROOT = Path.home() / 'agi_bags'


def generate_launch_description():
    node = Node(package='agi_ros2', executable='betaflight_msp_node',
                parameters=[MSP_CONFIG], output='screen')
    actions = []
    if RECORD_BAG:
        BAG_ROOT.mkdir(parents=True, exist_ok=True)
        bag = ExecuteProcess(cmd=['ros2', 'bag', 'record', '--all', '--include-hidden-topics',
            '--output', str(BAG_ROOT / datetime.now().strftime('msp_%Y%m%d_%H%M%S_%f'))], output='screen')
        actions += [bag, RegisterEventHandler(OnProcessExit(target_action=bag,
            on_exit=[EmitEvent(event=Shutdown(reason='MSP recorder exited'))]))]
    actions += [node, RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='MSP node exited'))]))]
    return LaunchDescription(actions)
