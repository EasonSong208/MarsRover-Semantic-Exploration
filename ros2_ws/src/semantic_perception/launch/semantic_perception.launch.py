"""Launch the Phase 0 fake semantic perception node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the fake semantic perception launch description."""
    input_topic = LaunchConfiguration('input_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic consumed by fake_semantic_node.',
        ),
        Node(
            package='semantic_perception',
            executable='fake_semantic_node',
            name='fake_semantic_node',
            output='screen',
            parameters=[{'input_topic': input_topic}],
        ),
    ])
