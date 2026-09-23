"""Fixed approximately 30-degree in-place turn test for the JetRover."""

import signal
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from robot_mission.graph_discovery import (
    GRAPH_DISCOVERY_TIMEOUT_SEC,
    StableDiscovery,
    normalize_snapshot,
)
from robot_mission.motion_smoke_policy import graph_errors
from robot_mission.turn_step_policy import (
    ANGULAR_SPEED_RAD_S,
    COMMAND_INTEGRATION_SEC,
    COMMAND_TOPIC,
    NONZERO_MESSAGE_COUNT,
    PUBLISH_RATE_HZ,
    SUBSCRIBER_TIMEOUT_SEC,
    THEORETICAL_ANGLE_DEG,
    THEORETICAL_ANGLE_RAD,
    TurnStepRunner,
    TwistSpec,
    direction_to_angular_z,
)


def _fq_name(name: str, namespace: str) -> str:
    namespace = namespace.rstrip('/')
    return f'{namespace}/{name}' if namespace else f'/{name}'


class TurnStepNode(Node):
    """ROS adapter; fixed motion constants are not declared as parameters."""

    def __init__(self) -> None:
        super().__init__('turn_step_test')
        self.declare_parameter('confirmed', False)
        self.declare_parameter('direction', 'left')
        self.publisher = None

    def confirmed(self) -> bool:
        return bool(self.get_parameter('confirmed').value)

    def direction(self) -> str:
        return str(self.get_parameter('direction').value)

    def print_plan(self, direction: str) -> None:
        angular_z = direction_to_angular_z(direction)
        print('TURN STEP PLAN')
        print(f'  direction={direction}')
        print(f'  angular.z={angular_z:+.2f} rad/s')
        print(f'  publish_rate={PUBLISH_RATE_HZ:.1f} Hz')
        print(f'  planned_nonzero_messages={NONZERO_MESSAGE_COUNT}')
        print(f'  command_integration_window={COMMAND_INTEGRATION_SEC:.2f} s')
        print(
            f'  theoretical_angle={THEORETICAL_ANGLE_RAD:.2f} rad / '
            f'{THEORETICAL_ANGLE_DEG:.1f} deg')
        print(f'  subscriber_timeout={SUBSCRIBER_TIMEOUT_SEC:.1f} s')

    def create_command_publisher(self) -> None:
        self.publisher = self.create_publisher(Twist, COMMAND_TOPIC, 1)

    def monotonic(self) -> float:
        return time.monotonic()

    def sleep(self, duration: float) -> None:
        time.sleep(duration)

    def log(self, message: str) -> None:
        print(message)

    def wait_for_graph_discovery(self) -> bool:
        """Wait for two stable controller-chain graph snapshots."""
        started = time.monotonic()
        tracker = StableDiscovery()
        self.log('GRAPH_DISCOVERY_WAIT_STARTED')
        while time.monotonic() - started < GRAPH_DISCOVERY_TIMEOUT_SEC:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.monotonic() - started
            nodes = normalize_snapshot(self.get_node_names_and_namespaces())
            self.log(
                f'GRAPH_DISCOVERY_SNAPSHOT elapsed={elapsed:.3f} '
                f'nodes={list(nodes)}')
            if tracker.observe(nodes):
                self.log(f'GRAPH_DISCOVERY_READY elapsed={elapsed:.3f}')
                return True
        self.log('GRAPH_DISCOVERY_TIMEOUT')
        return False

    def graph_errors(self) -> tuple[str, ...]:
        publishers = {}
        own_name = _fq_name(self.get_name(), self.get_namespace())
        for topic in (COMMAND_TOPIC, '/controller/cmd_vel', '/cmd_vel_nav'):
            names = {
                _fq_name(info.node_name, info.node_namespace)
                for info in self.get_publishers_info_by_topic(topic)
            }
            publishers[topic] = sorted(names - {own_name})
        nodes = [
            name.lstrip('/') for name in normalize_snapshot(
                self.get_node_names_and_namespaces())
            if name != own_name
        ]
        return graph_errors(publishers, nodes)

    def subscriber_count(self) -> int:
        return self.publisher.get_subscription_count()

    def publish(self, command: TwistSpec) -> None:
        message = Twist()
        message.linear.x = command.linear_x
        message.linear.y = command.linear_y
        message.linear.z = command.linear_z
        message.angular.x = command.angular_x
        message.angular.y = command.angular_y
        message.angular.z = command.angular_z
        self.publisher.publish(message)


def main(args=None) -> int:
    """Print plan unless explicitly confirmed, then run the fixed protocol."""
    rclpy.init(args=args)
    node = TurnStepNode()
    old_handlers = {}
    runner = None
    try:
        direction = node.direction()
        try:
            node.print_plan(direction)
        except ValueError as exc:
            print(f'TEST_ABORTED reason={exc}')
            return 2
        if not node.confirmed():
            print('TEST_ABORTED reason=NOT_CONFIRMED; no publisher created')
            return 2

        node.create_command_publisher()
        runner = TurnStepRunner(node)
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.getsignal(signum)
            signal.signal(
                signum,
                lambda received, _frame: runner.request_signal(received),
            )

        try:
            result = runner.run(direction)
        except KeyboardInterrupt:
            return 130
        except BaseException as exc:
            node.get_logger().error(f'turn step test failed: {exc}')
            return 1
        return 0 if result.completed else 3
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
