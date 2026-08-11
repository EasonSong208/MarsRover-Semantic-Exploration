import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('safety_monitor'), 'config', 'params.yaml'
    )
    enable_rf2o = LaunchConfiguration('enable_rf2o')

    return LaunchDescription([
        DeclareLaunchArgument('enable_rf2o', default_value='true'),
        Node(
            package='safety_monitor',
            executable='wheel_slip_monitor',
            name='wheel_slip_monitor',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='safety_monitor',
            executable='scan_safety_gate',
            name='scan_safety_gate',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='safety_monitor',
            executable='cmd_vel_safety_gate',
            name='cmd_vel_safety_gate',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': 'scan_slam',
                'odom_topic': 'odom_rf2o',
                'publish_tf': False,
                'base_frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 10.0,
            }],
            arguments=['--ros-args', '--log-level', 'WARN'],
            condition=IfCondition(enable_rf2o),
        ),
    ])
