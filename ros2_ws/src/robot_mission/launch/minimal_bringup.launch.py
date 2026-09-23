"""Compose the smallest audited JetRover state and sensor bringup.

This launch intentionally has no joystick, servo controller, init pose, Nav2,
application demo, or velocity publisher.  It is not executed by tests: tests
inspect its Python syntax and launch description statically.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def _vendor_launch(package, filename):
    """Return an installed vendor launch path without hard-coded workspaces."""
    return PathJoinSubstitution([FindPackageShare(package), 'launch', filename])


def generate_launch_description():
    """Build an inert-by-default sensor/state graph for static M1 validation."""
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_odom = LaunchConfiguration('enable_odom')
    enable_ekf = LaunchConfiguration('enable_ekf')
    fixed_arm_pose = LaunchConfiguration('fixed_arm_pose')

    arguments = [
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('enable_odom', default_value='true'),
        DeclareLaunchArgument('enable_ekf', default_value='true'),
        DeclareLaunchArgument('fixed_arm_pose', default_value='false'),
    ]

    # Normal mode retains the vendor joint-state path. Fixed-arm mode must not
    # start it: publishing zero joint defaults would conflict with the audited
    # selected fixed-pose transforms owned by the RGB-D SLAM launch.
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _vendor_launch('jetrover_description',
                           'robot_description.launch.py')),
        launch_arguments={
            'use_gui': 'false',
            'use_rviz': 'false',
            'use_sim_time': 'false',
            'namespace': '',
            'use_namespace': 'false',
            'frame_prefix': '',
        }.items(),
        condition=UnlessCondition(fixed_arm_pose),
    )

    description_share = FindPackageShare('jetrover_description')
    description_path = PathJoinSubstitution(
        [description_share, 'urdf', 'jetrover.xacro'])
    fixed_arm_robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(fixed_arm_pose),
        parameters=[{
            'robot_description': Command(['xacro ', description_path]),
            'frame_prefix': '',
            'use_sim_time': False,
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    # Direct node composition avoids odom_publisher.launch.py, which would pull
    # in more hardware composition than this launch needs.
    board = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        name='ros_robot_controller',
        output='screen',
        parameters=[{'imu_frame': 'imu_link'}],
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _vendor_launch('peripherals', 'imu_filter.launch.py')),
        condition=IfCondition(enable_imu),
    )

    controller_share = FindPackageShare('controller')
    odom = Node(
        package='controller',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        condition=IfCondition(enable_odom),
        parameters=[
            PathJoinSubstitution(
                [controller_share, 'config', 'calibrate_params.yaml']),
            {
                'base_frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'pub_odom_topic': True,
            },
        ],
    )

    ekf_config = ReplaceString(
        source_file=PathJoinSubstitution(
            [controller_share, 'config', 'ekf.yaml']),
        replacements={'namespace/': ''},
    )
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(enable_ekf),
        parameters=[ekf_config, {'use_sim_time': False}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odometry/filtered', 'odom'),
            ('cmd_vel', 'controller/cmd_vel'),
        ],
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _vendor_launch('peripherals', 'lidar.launch.py')),
        condition=IfCondition(enable_lidar),
        launch_arguments={
            'lidar_frame': 'lidar_frame',
            'scan_raw': 'scan_raw',
            'scan_topic': 'scan',
        }.items(),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _vendor_launch('peripherals', 'depth_camera.launch.py')),
        condition=IfCondition(enable_camera),
        launch_arguments={
            'depth_camera_name': 'depth_cam',
            'tf_prefix': '',
        }.items(),
    )

    return LaunchDescription(arguments + [
        robot_description,
        fixed_arm_robot_description,
        board,
        imu,
        odom,
        ekf,
        lidar,
        camera,
    ])
