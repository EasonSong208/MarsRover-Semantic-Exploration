"""Arm-only camera-pose guard and fixed TF; never starts a chassis node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _load_pose(pose_name):
    if pose_name != 'vendor_init':
        raise RuntimeError('camera pose guard currently supports only vendor_init')
    share = get_package_share_directory('robot_mission')
    path = os.path.join(share, 'config', 'camera_poses', f'{pose_name}.yaml')
    with open(path, encoding='utf-8') as stream:
        pose = yaml.safe_load(stream)
    if pose.get('pose_name') != pose_name:
        raise RuntimeError(f'pose file {path} does not declare {pose_name}')
    return pose


def _launch_setup(context):
    share = get_package_share_directory('robot_mission')
    pose_name = LaunchConfiguration('pose_name').perform(context)
    pose = _load_pose(pose_name)
    nodes = []
    for joint in pose['fixed_joints']:
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f"fixed_{joint['name']}_tf",
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_fixed_camera_tf')),
            arguments=[
                '--x', str(joint['xyz'][0]),
                '--y', str(joint['xyz'][1]),
                '--z', str(joint['xyz'][2]),
                '--roll', str(joint['rpy'][0]),
                '--pitch', str(joint['rpy'][1]),
                '--yaw', str(joint['rpy'][2]),
                '--frame-id', joint['parent_frame'],
                '--child-frame-id', joint['child_frame'],
            ],
        ))
    nodes.append(Node(
        package='robot_mission',
        executable='camera_pose_guard',
        name='camera_pose_guard',
        output='screen',
        parameters=[
            LaunchConfiguration('camera_pose_config'),
            {
                'pose_name': pose_name,
                'camera_pose_name': pose_name,
                'dry_run': ParameterValue(
                    LaunchConfiguration('dry_run'), value_type=bool),
                'confirmed': ParameterValue(
                    LaunchConfiguration('confirmed'), value_type=bool),
                'arm_torque_confirmed': ParameterValue(
                    LaunchConfiguration('arm_torque_confirmed'),
                    value_type=bool),
                'enable_feedback_check': ParameterValue(
                    LaunchConfiguration('enable_feedback_check'),
                    value_type=bool),
                'allow_time_based_ready': ParameterValue(
                    LaunchConfiguration('allow_time_based_ready'),
                    value_type=bool),
                'max_pose_command_attempts': ParameterValue(
                    LaunchConfiguration('max_pose_command_attempts'),
                    value_type=int),
            },
        ],
    ))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('robot_mission')
    defaults = {
        'camera_pose_config': os.path.join(
            share, 'config', 'camera_navigation_pose.yaml'),
        'pose_name': 'vendor_init',
        'dry_run': 'true',
        'confirmed': 'false',
        'arm_torque_confirmed': 'false',
        'enable_feedback_check': 'true',
        'allow_time_based_ready': 'false',
        'max_pose_command_attempts': '1',
        'start_fixed_camera_tf': 'true',
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]
    return LaunchDescription(
        arguments + [OpaqueFunction(function=_launch_setup)])
