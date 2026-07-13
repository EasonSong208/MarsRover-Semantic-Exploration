"""Two-phase, read-only safety gate for the RGB-D RTAB-Map bringup."""

import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from robot_mission.graph_discovery import normalize_snapshot


OWNED_NODES = (
    '/depth_cam/depth_cam',
    '/ekf_filter_node',
    '/joint_state_publisher',
    '/odom_publisher',
    '/robot_state_publisher',
    '/ros_robot_controller',
)
REQUIRED_READY_NODES = (
    '/depth_cam/depth_cam',
    '/ekf_filter_node',
    '/odom_publisher',
    '/robot_state_publisher',
    '/ros_robot_controller',
)
FORBIDDEN_NODE_TERMS = (
    'init_pose', 'joystick', 'lidar', 'servo', 'slam_toolbox', 'teleop',
)


class RgbdRtabmapPreflight(Node):
    """Refuse duplicate startup, then validate the complete stationary graph."""

    def __init__(self) -> None:
        super().__init__('rgbd_rtabmap_preflight')
        self.declare_parameter('phase', 'pre_start')
        self.declare_parameter('fixed_pose_confirmed', False)
        self.declare_parameter('timeout_sec', 15.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('rgb_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('depth_topic', '/depth_cam/depth/image_raw')
        self.declare_parameter(
            'camera_info_topic', '/depth_cam/rgb/camera_info')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter(
            'camera_frame', 'depth_cam_color_optical_frame')
        self.frames = {}
        self.tf_buffer = None
        self.tf_listener = None
        self._subscriptions = []

        if self.phase == 'ready':
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self._subscriptions = [
                self.create_subscription(
                    Odometry, self.odom_topic, self._odom, qos_profile_sensor_data),
                self.create_subscription(
                    Image, self.rgb_topic, self._rgb, qos_profile_sensor_data),
                self.create_subscription(
                    Image, self.depth_topic, self._depth, qos_profile_sensor_data),
                self.create_subscription(
                    CameraInfo, self.camera_info_topic, self._camera_info,
                    qos_profile_sensor_data),
            ]

    @property
    def phase(self):
        return str(self.get_parameter('phase').value)

    @property
    def timeout_sec(self):
        return float(self.get_parameter('timeout_sec').value)

    @property
    def odom_topic(self):
        return str(self.get_parameter('odom_topic').value)

    @property
    def rgb_topic(self):
        return str(self.get_parameter('rgb_topic').value)

    @property
    def depth_topic(self):
        return str(self.get_parameter('depth_topic').value)

    @property
    def camera_info_topic(self):
        return str(self.get_parameter('camera_info_topic').value)

    @property
    def odom_frame(self):
        return str(self.get_parameter('odom_frame').value)

    @property
    def base_frame(self):
        return str(self.get_parameter('base_frame').value)

    @property
    def camera_frame(self):
        return str(self.get_parameter('camera_frame').value)

    def _odom(self, message):
        self.frames['odom_header'] = message.header.frame_id
        self.frames['odom_child'] = message.child_frame_id

    def _rgb(self, message):
        self.frames['rgb'] = message.header.frame_id

    def _depth(self, message):
        self.frames['depth'] = message.header.frame_id

    def _camera_info(self, message):
        self.frames['camera_info'] = message.header.frame_id

    def _nodes(self):
        return normalize_snapshot(self.get_node_names_and_namespaces())

    @staticmethod
    def _forbidden_node_errors(nodes):
        errors = []
        for name in nodes:
            lowered = name.lower()
            if any(term in lowered for term in FORBIDDEN_NODE_TERMS):
                errors.append(f'forbidden node discovered: {name}')
        return errors

    def pre_start_errors(self):
        errors = []
        if not bool(self.get_parameter('fixed_pose_confirmed').value):
            errors.append(
                'fixed_pose_confirmed is false; arm must match vendor_horizontal')
        nodes = self._nodes()
        errors.extend(self._forbidden_node_errors(nodes))
        for owned in OWNED_NODES:
            count = nodes.count(owned)
            if count:
                errors.append(f'pre-existing node {owned}: found {count}')
        for topic in (
            self.odom_topic, self.rgb_topic, self.depth_topic,
            self.camera_info_topic, '/joint_states', '/scan', '/scan_raw',
        ):
            count = len(self.get_publishers_info_by_topic(topic))
            if count:
                errors.append(f'pre-existing publisher(s) on {topic}: {count}')
        return tuple(errors)

    def ready_errors(self):
        errors = []
        nodes = self._nodes()
        errors.extend(self._forbidden_node_errors(nodes))
        for required in REQUIRED_READY_NODES:
            count = nodes.count(required)
            if count != 1:
                errors.append(f'{required} must exist exactly once; found {count}')
        if nodes.count('/joint_state_publisher'):
            errors.append('joint_state_publisher conflicts with fixed arm TF')

        for topic in (
            self.odom_topic, self.rgb_topic, self.depth_topic,
            self.camera_info_topic,
        ):
            count = len(self.get_publishers_info_by_topic(topic))
            if count != 1:
                errors.append(f'{topic} must have exactly one publisher; found {count}')
        for topic in ('/joint_states', '/scan', '/scan_raw'):
            count = len(self.get_publishers_info_by_topic(topic))
            if count:
                errors.append(f'forbidden publisher(s) on {topic}: {count}')

        expected_frames = {
            'odom_header': self.odom_frame,
            'odom_child': self.base_frame,
            'rgb': self.camera_frame,
            'depth': self.camera_frame,
            'camera_info': self.camera_frame,
        }
        for key, expected in expected_frames.items():
            actual = self.frames.get(key)
            if actual != expected:
                errors.append(f'{key} frame must be {expected}; found {actual!r}')

        if not self.tf_buffer.can_transform(
                self.odom_frame, self.base_frame, Time()):
            errors.append(
                f'missing TF {self.odom_frame} -> {self.base_frame}')
        if not self.tf_buffer.can_transform(
                self.base_frame, self.camera_frame, Time()):
            errors.append(
                f'missing TF {self.base_frame} -> {self.camera_frame}')
        return tuple(errors)

    def run(self):
        started = time.monotonic()
        last_errors = ('discovery has not completed',)
        while time.monotonic() - started < self.timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.phase == 'pre_start':
                # Give DDS a fixed discovery window before declaring the graph clear.
                if time.monotonic() - started < min(5.0, self.timeout_sec):
                    continue
                last_errors = self.pre_start_errors()
            elif self.phase == 'ready':
                last_errors = self.ready_errors()
            else:
                last_errors = (f'unknown preflight phase: {self.phase}',)
            if not last_errors:
                self.get_logger().info(f'PREFLIGHT_{self.phase.upper()}_PASSED')
                return 0

        for error in last_errors:
            self.get_logger().error(f'PREFLIGHT_{self.phase.upper()}_FAILED: {error}')
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = RgbdRtabmapPreflight()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
