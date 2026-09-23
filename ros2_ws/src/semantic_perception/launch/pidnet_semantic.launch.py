"""Launch the independent PIDNet-S hazard5 semantic node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create a configurable PIDNet-S-only launch description."""
    arguments = [
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='Absolute path to the V3 best_miou.pt checkpoint.',
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/depth_cam/rgb/image_raw',
            description='Existing RGB sensor_msgs/Image topic.',
        ),
        DeclareLaunchArgument(
            'mask_topic',
            default_value='/semantic/mask',
            description='mono8 hazard5 class-ID output topic.',
        ),
        DeclareLaunchArgument(
            'device', default_value='cuda',
            description='Torch device; deployment default is CUDA.',
        ),
        DeclareLaunchArgument(
            'precision', default_value='fp16',
            description='Inference precision: fp16 or fp32.',
        ),
        DeclareLaunchArgument(
            'publish_debug_images', default_value='true',
            description='Publish /semantic/color debug image.',
        ),
        DeclareLaunchArgument(
            'publish_overlay', default_value='false',
            description='Publish /semantic/overlay blend (CPU-heavy, default off).',
        ),
        DeclareLaunchArgument(
            'process_every_n', default_value='2',
            description='Frame skip: process every Nth received RGB (2 → ~15fps).',
        ),
    ]
    node = Node(
        package='semantic_perception',
        executable='pidnet_semantic_node',
        name='pidnet_semantic_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'input_topic': LaunchConfiguration('input_topic'),
            'mask_topic': LaunchConfiguration('mask_topic'),
            'device': LaunchConfiguration('device'),
            'precision': LaunchConfiguration('precision'),
            'publish_debug_images': LaunchConfiguration(
                'publish_debug_images'),
            'publish_overlay': LaunchConfiguration('publish_overlay'),
            'process_every_n': LaunchConfiguration('process_every_n'),
        }],
    )
    return LaunchDescription(arguments + [node])
