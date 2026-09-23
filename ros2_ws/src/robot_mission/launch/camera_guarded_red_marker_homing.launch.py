"""Guarded red-marker homing plus the selected fixed arm TF; no bringup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _launch_default(value):
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, list):
        return yaml.safe_dump(value, default_flow_style=True).strip()
    return str(value)


def _red_defaults(share):
    path = os.path.join(share, 'config', 'red_marker_homing.yaml')
    with open(path, encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)[
            'red_marker_homing_test']['ros__parameters']
    return {name: _launch_default(value) for name, value in parameters.items()}


def _load_pose(pose_name):
    if pose_name != 'vendor_init':
        raise RuntimeError('guarded homing currently supports only vendor_init')
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
    static_nodes = []
    for joint in pose['fixed_joints']:
        xyz = [str(value) for value in joint['xyz']]
        rpy = [str(value) for value in joint['rpy']]
        static_nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f"fixed_{joint['name']}_tf",
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_fixed_camera_tf')),
            arguments=[
                '--x', xyz[0], '--y', xyz[1], '--z', xyz[2],
                '--roll', rpy[0], '--pitch', rpy[1], '--yaw', rpy[2],
                '--frame-id', joint['parent_frame'],
                '--child-frame-id', joint['child_frame'],
            ],
        ))

    camera_guard = Node(
        package='robot_mission',
        executable='camera_pose_guard',
        name='camera_pose_guard',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_camera_pose_guard')),
        parameters=[
            LaunchConfiguration('camera_pose_config'),
            {
                'pose_name': pose_name,
                'camera_pose_name': pose_name,
                'settle_time_sec': ParameterValue(
                    LaunchConfiguration('settle_time_sec'), value_type=float),
                'enable_feedback_check': ParameterValue(
                    LaunchConfiguration('enable_feedback_check'),
                    value_type=bool),
                'allow_time_based_ready': ParameterValue(
                    LaunchConfiguration('allow_time_based_ready'),
                    value_type=bool),
                'max_pose_command_attempts': ParameterValue(
                    LaunchConfiguration('max_pose_command_attempts'),
                    value_type=int),
                'dry_run': ParameterValue(
                    LaunchConfiguration('dry_run'), value_type=bool),
                'confirmed': ParameterValue(
                    LaunchConfiguration('confirmed'), value_type=bool),
                'arm_torque_confirmed': ParameterValue(
                    LaunchConfiguration('arm_torque_confirmed'),
                    value_type=bool),
                # JetRover_Mecanum vendor odom_publisher advertises this topic
                # for the Ackermann steering branch, but its Mecanum path returns
                # no steering-servo command and never calls publish. The guard
                # still enumerates every endpoint and rejects duplicates/unknowns.
                'allowed_passive_set_state_publishers': ['/odom_publisher'],
                'ready_topic': LaunchConfiguration('camera_pose_ready_topic'),
            },
        ],
    )

    red_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'red_marker_homing.launch.py')),
        launch_arguments={
            name: LaunchConfiguration(name)
            for name in _red_defaults(share)
        }.items(),
    )
    return [*static_nodes, camera_guard, red_launch]


def generate_launch_description():
    share = get_package_share_directory('robot_mission')
    defaults = _red_defaults(share)
    defaults.update({
        'dry_run': 'true',
        'confirmed': 'false',
        'start_camera_pose_guard': 'true',
        'camera_pose_config': os.path.join(
            share, 'config', 'camera_navigation_pose.yaml'),
        'pose_name': 'vendor_init',
        'settle_time_sec': '2.0',
        'enable_feedback_check': 'true',
        'allow_time_based_ready': 'false',
        'max_pose_command_attempts': '1',
        'arm_torque_confirmed': 'false',
        'require_camera_pose_ready': 'true',
        'camera_pose_ready_topic': '/camera_pose_ready',
        'camera_pose_ready_timeout_sec': '0.0',
        'start_fixed_camera_tf': 'true',
    })
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]
    return LaunchDescription(
        arguments + [OpaqueFunction(function=_launch_setup)])
