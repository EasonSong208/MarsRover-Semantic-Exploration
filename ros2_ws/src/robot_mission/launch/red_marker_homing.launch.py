"""Standalone red-marker homing node; no bringup or hardware is included."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml


def _launch_default(value):
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, list):
        return yaml.safe_dump(value, default_flow_style=True).strip()
    return str(value)


def _config_defaults(config_file):
    with open(config_file, encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)[
            'red_marker_homing_test']['ros__parameters']
    return {name: _launch_default(value) for name, value in parameters.items()}


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('robot_mission'),
        'config', 'red_marker_homing.yaml')
    # Every maintained YAML parameter becomes a launch argument. This includes
    # speed, reference-depth tolerance and color thresholds.
    argument_defaults = _config_defaults(config_file)
    launch_arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in argument_defaults.items()
    ]
    launch_parameters = {
        name: LaunchConfiguration(name) for name in argument_defaults
    }
    return LaunchDescription(launch_arguments + [
        Node(
            package='robot_mission',
            executable='red_marker_homing_test',
            name='red_marker_homing_test',
            output='screen',
            parameters=[
                config_file,
                launch_parameters,
            ],
        ),
    ])
