"""Read-only ROS2 graph, lifecycle, action, and TF checks for M1."""

from collections import Counter
import time

from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from robot_mission.policy import (
    CheckResult,
    Severity,
    check_topic,
    check_unique_node,
    summarize,
    warn_publishers,
)


OBSERVATION_TOPICS = {
    '/scan_raw': ('sensor_msgs/msg/LaserScan', 1),
    '/scan': ('sensor_msgs/msg/LaserScan', 1),
    '/imu': ('sensor_msgs/msg/Imu', 1),
    '/odom_raw': ('nav_msgs/msg/Odometry', 1),
    '/odom': ('nav_msgs/msg/Odometry', 1),
    '/map': ('nav_msgs/msg/OccupancyGrid', 1),
    '/depth_cam/rgb/image_raw': ('sensor_msgs/msg/Image', 1),
    '/depth_cam/rgb/camera_info': ('sensor_msgs/msg/CameraInfo', 1),
}

CONTROL_TOPICS = {
    '/cmd_vel_nav': 'geometry_msgs/msg/Twist',
    '/cmd_vel': 'geometry_msgs/msg/Twist',
    '/controller/cmd_vel': 'geometry_msgs/msg/Twist',
}

UNIQUE_NODES = (
    'odom_publisher',
    'ekf_filter_node',
    'imu_filter',
    'ros_robot_controller',
    'scan_to_scan_filter_chain',
)

LIFECYCLE_NODES = (
    '/map_server',
    '/amcl',
    '/controller_server',
    '/planner_server',
    '/bt_navigator',
    '/velocity_smoother',
)

REQUIRED_TFS = (
    ('map', 'odom'),
    ('odom', 'base_footprint'),
    ('base_footprint', 'lidar_frame'),
    ('base_footprint', 'depth_cam_link'),
)

ACTION_SERVICES = (
    '/compute_path_to_pose/_action/send_goal',
    '/navigate_to_pose/_action/send_goal',
)


def _fq_node_name(name: str, namespace: str) -> str:
    namespace = namespace.rstrip('/')
    return f'{namespace}/{name}' if namespace else f'/{name}'


class Preflight(Node):
    """A node that observes existing interfaces and never commands the robot."""

    def __init__(self) -> None:
        super().__init__('preflight')
        self.declare_parameter('discovery_wait_sec', 2.0)
        self.declare_parameter('service_timeout_sec', 2.0)
        self.declare_parameter('tf_timeout_sec', 2.0)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def run(self) -> list[CheckResult]:
        """Collect all checks after a bounded graph discovery period."""
        discovery_wait = float(self.get_parameter('discovery_wait_sec').value)
        deadline = time.monotonic() + discovery_wait
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        results: list[CheckResult] = []
        topic_types = dict(self.get_topic_names_and_types())

        for topic, (expected_type, expected_count) in OBSERVATION_TOPICS.items():
            publishers = self._publisher_names(topic)
            results.append(check_topic(
                topic,
                topic_types.get(topic, []),
                expected_type,
                publishers,
                expected_count,
            ))

        for topic, expected_type in CONTROL_TOPICS.items():
            publishers = self._publisher_names(topic)
            type_result = check_topic(
                topic,
                topic_types.get(topic, []),
                expected_type,
                publishers,
            )
            results.append(type_result)
            if topic == '/controller/cmd_vel' and type_result.severity != Severity.FAIL:
                results.append(warn_publishers(topic + ' endpoints', publishers))

        nodes = [
            (_fq_node_name(name, namespace), name)
            for name, namespace in self.get_node_names_and_namespaces()
            if name != self.get_name()
        ]
        for basename in UNIQUE_NODES:
            matches = sorted(full for full, name in nodes if name == basename)
            results.append(check_unique_node(basename, matches))

        duplicate_names = sorted(
            name for name, count in Counter(full for full, _ in nodes).items()
            if count > 1
        )
        if duplicate_names:
            results.append(CheckResult(
                Severity.WARN,
                'duplicate node names',
                ', '.join(duplicate_names),
            ))
        else:
            results.append(CheckResult(
                Severity.PASS,
                'duplicate node names',
                'none',
            ))

        results.extend(self._check_lifecycle_nodes())
        results.extend(self._check_tfs())
        results.extend(self._check_action_services())
        results.append(CheckResult(
            Severity.WARN,
            '/odom_raw provenance',
            'vendor source audit shows command-integrated odometry, not verified '
            'wheel-encoder feedback',
        ))
        return results

    def _publisher_names(self, topic: str) -> list[str]:
        names = {
            _fq_node_name(endpoint.node_name, endpoint.node_namespace)
            for endpoint in self.get_publishers_info_by_topic(topic)
        }
        return sorted(names)

    def _check_lifecycle_nodes(self) -> list[CheckResult]:
        timeout = float(self.get_parameter('service_timeout_sec').value)
        results = []
        for node_name in LIFECYCLE_NODES:
            client = self.create_client(GetState, node_name + '/get_state')
            if not client.wait_for_service(timeout_sec=timeout):
                results.append(CheckResult(
                    Severity.FAIL, node_name, 'lifecycle service unavailable'))
                self.destroy_client(client)
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            response = future.result() if future.done() else None
            if response is None:
                results.append(CheckResult(
                    Severity.FAIL, node_name, 'lifecycle query timed out'))
            elif response.current_state.label != 'active':
                results.append(CheckResult(
                    Severity.FAIL,
                    node_name,
                    f'lifecycle state={response.current_state.label}',
                ))
            else:
                results.append(CheckResult(
                    Severity.PASS, node_name, 'lifecycle state=active'))
            self.destroy_client(client)
        return results

    def _check_tfs(self) -> list[CheckResult]:
        timeout = float(self.get_parameter('tf_timeout_sec').value)
        duration = Duration(seconds=timeout)
        results = []
        for target, source in REQUIRED_TFS:
            name = f'{target} -> {source}'
            try:
                self.tf_buffer.lookup_transform(target, source, rclpy.time.Time(), duration)
            except TransformException as exc:
                results.append(CheckResult(Severity.FAIL, name, str(exc)))
            else:
                results.append(CheckResult(Severity.PASS, name, 'available'))
        return results

    def _check_action_services(self) -> list[CheckResult]:
        service_names = {name for name, _ in self.get_service_names_and_types()}
        return [
            CheckResult(
                Severity.PASS if service in service_names else Severity.FAIL,
                service,
                'action send-goal service available'
                if service in service_names else 'action send-goal service missing',
            )
            for service in ACTION_SERVICES
        ]


def main(args=None) -> int:
    """Run once, print a stable report, and return nonzero on hard failures."""
    rclpy.init(args=args)
    node = Preflight()
    try:
        results = node.run()
        for result in results:
            print(f'{result.label:<4}  {result.name}: {result.detail}')
        severity, message = summarize(results)
        print(f'SUMMARY: {message}')
        return 1 if severity == Severity.FAIL else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
