"""Safely gate a no-LiDAR RGB-D RTAB-Map bringup on a named fixed arm pose."""

from functools import partial
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


KNOWN_CAMERA_POSES = ('vendor_init', 'vendor_horizontal')


def _on_gate_exit(event, _context, *, phase, success_actions):
    if event.returncode == 0:
        return success_actions
    return [
        LogInfo(msg=f'ERROR: {phase} preflight failed; refusing later stages'),
        EmitEvent(event=Shutdown(reason=f'{phase} preflight failed')),
    ]


def _load_camera_pose(pose_name):
    if pose_name not in KNOWN_CAMERA_POSES:
        known = ', '.join(KNOWN_CAMERA_POSES)
        raise RuntimeError(
            f'unknown camera_pose {pose_name!r}; expected one of: {known}')
    share = get_package_share_directory('robot_mission')
    path = os.path.join(share, 'config', 'camera_poses', f'{pose_name}.yaml')
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if data.get('pose_name') != pose_name:
        raise RuntimeError(
            f'camera pose file {path} declares {data.get("pose_name")!r}, '
            f'not {pose_name!r}')
    return data


def _format_number(value):
    return f'{float(value):g}'


def _launch_setup(context):
    camera_pose = LaunchConfiguration('camera_pose').perform(context)
    pose = _load_camera_pose(camera_pose)
    servo_targets = [
        pose['servo_targets'][f'Servo{index}'] for index in range(1, 5)
    ]
    joint_angles_deg = pose['joint_angles_deg']
    nominal_transform = pose['transforms']['base_link_to_depth_cam_link']

    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    publish_tf = LaunchConfiguration('publish_tf')
    use_rviz = LaunchConfiguration('use_rviz')
    use_static_camera_tf = LaunchConfiguration('use_static_camera_tf')
    fixed_pose_confirmed = LaunchConfiguration('fixed_pose_confirmed')
    qos = LaunchConfiguration('qos')

    common_preflight = {
        'camera_pose': camera_pose,
        'loaded_camera_pose': pose['pose_name'],
        'known_camera_poses': list(KNOWN_CAMERA_POSES),
        'fixed_pose_confirmed': ParameterValue(
            fixed_pose_confirmed, value_type=bool),
        'use_static_camera_tf': ParameterValue(
            use_static_camera_tf, value_type=bool),
        'expected_servo_targets': servo_targets,
        'expected_joint_angles_deg': joint_angles_deg,
        'expected_camera_xyz': nominal_transform['xyz'],
        'expected_camera_quaternion': [
            nominal_transform['quaternion'][key]
            for key in ('x', 'y', 'z', 'w')
        ],
        'fixed_camera_parent_frame': nominal_transform['parent_frame'],
        'camera_link_frame': nominal_transform['child_frame'],
        'odom_topic': '/odom',
        'rgb_topic': rgb_topic,
        'depth_topic': depth_topic,
        'camera_info_topic': camera_info_topic,
        'odom_frame': odom_frame,
        'base_frame': base_frame,
        'camera_frame': camera_frame,
    }
    pre_start = Node(
        package='robot_mission',
        executable='rgbd_rtabmap_preflight',
        name='rgbd_rtabmap_preflight_pre_start',
        output='screen',
        parameters=[common_preflight | {'phase': 'pre_start', 'timeout_sec': 5.0}],
    )
    ready = Node(
        package='robot_mission',
        executable='rgbd_rtabmap_preflight',
        name='rgbd_rtabmap_preflight_ready',
        output='screen',
        parameters=[common_preflight | {'phase': 'ready', 'timeout_sec': 15.0}],
    )

    share = get_package_share_directory('robot_mission')
    minimal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'minimal_bringup.launch.py')),
        launch_arguments={
            'enable_lidar': 'false',
            'enable_camera': 'true',
            'enable_imu': 'true',
            'enable_odom': 'true',
            'enable_ekf': 'true',
            'fixed_arm_pose': 'true',
        }.items(),
    )

    static_joint_nodes = []
    for joint in pose['fixed_joints']:
        xyz = [str(value) for value in joint['xyz']]
        rpy = [str(value) for value in joint['rpy']]
        static_joint_nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f"fixed_{joint['name']}_tf",
            output='screen',
            condition=IfCondition(use_static_camera_tf),
            arguments=[
                '--x', xyz[0], '--y', xyz[1], '--z', xyz[2],
                '--roll', rpy[0], '--pitch', rpy[1], '--yaw', rpy[2],
                '--frame-id', joint['parent_frame'],
                '--child-frame-id', joint['child_frame'],
            ],
        ))

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('rgb/image', rgb_topic),
        ('rgb/camera_info', camera_info_topic),
        ('depth/image', depth_topic),
        ('odom', '/odom'),
    ]
    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.008,
            'qos': ParameterValue(qos, value_type=int),
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        remappings=remappings,
    )
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': base_frame,
            'odom_frame_id': odom_frame,
            'map_frame_id': map_frame,
            'database_path': database_path,
            'publish_tf': ParameterValue(publish_tf, value_type=bool),
            'subscribe_rgbd': True,
            'subscribe_scan': False,
            'subscribe_odom': True,
            'queue_size': 50,
            'Reg/Strategy': '0',
            'Reg/Force3DoF': 'true',
            'Grid/FromDepth': 'true',
            'Grid/RangeMin': '0.2',
            'Optimizer/GravitySigma': '0',
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        remappings=remappings,
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(use_rviz),
    )

    ready_stage = [rgbd_sync, rtabmap, rviz]
    bringup_stage = [minimal, *static_joint_nodes, TimerAction(
        period=10.0, actions=[ready])]
    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=pre_start,
            on_exit=partial(
                _on_gate_exit, phase='pre-start',
                success_actions=bringup_stage))),
        RegisterEventHandler(OnProcessExit(
            target_action=ready,
            on_exit=partial(
                _on_gate_exit, phase='ready',
                success_actions=ready_stage))),
    ]
    logs = [
        LogInfo(msg=f'Selected fixed camera pose: {pose["pose_name"]}'),
        LogInfo(msg='Expected servo targets: ' + ','.join(
            str(value) for value in servo_targets)),
        LogInfo(msg='Expected joint angles: ' + ','.join(
            _format_number(value) for value in joint_angles_deg) + ' deg'),
        LogInfo(msg=(
            'WARNING: no real servo feedback closed loop is available; '
            'the physical pose is not automatically verified')),
        LogInfo(msg=(
            'WARNING: fixed_pose_confirmed=true means manual operator '
            'confirmation of the selected pose')),
        LogInfo(msg=(
            'WARNING: stop SLAM immediately if the arm is moved')),
    ]
    return logs + handlers + [pre_start]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'database_path',
            default_value=[EnvironmentVariable('HOME'), '/.ros/rtabmap.db']),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument(
            'camera_frame', default_value='depth_cam_color_optical_frame'),
        DeclareLaunchArgument(
            'rgb_topic', default_value='/depth_cam/rgb/image_raw'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/depth_cam/depth/image_raw'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/depth_cam/rgb/camera_info'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_static_camera_tf', default_value='true'),
        DeclareLaunchArgument('camera_pose', default_value='vendor_init'),
        DeclareLaunchArgument('fixed_pose_confirmed', default_value='false'),
        DeclareLaunchArgument('qos', default_value='2'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
