"""Dedicated hardware telemetry only: no control or MSP port is opened."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {'device': '', 'baud': '921600', 'gps_mode': 'gnss',
                'altitude_source': 'unknown', 'imu_rate_hz': '500',
                'gps_rate_hz': '10', 'attitude_rate_hz': '0'}
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=int if k.endswith('_hz') or k == 'baud' else str)
              for k in defaults}
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
        [Node(package='agi_ros2', executable='mavlink_sensor_node', output='screen',
              parameters=[params, {'use_sim_time': False}])])
