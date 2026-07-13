"""Hard-limited, wheels-raised-only JetRover motion smoke test."""

import signal
import threading
import time
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from robot_mission.graph_discovery import (
    GRAPH_DISCOVERY_TIMEOUT_SEC,
    StableDiscovery,
    normalize_snapshot,
)
from robot_mission.motion_smoke_policy import (
    MAX_NONZERO_DURATION,
    MIN_ZERO_DURATION,
    PUBLISH_RATE_HZ,
    SAFE_LINEAR_X,
    SmokeParameters,
    execute_with_zero_cleanup,
    graph_errors,
    validate_parameters,
)


COMMAND_TOPIC = '/cmd_vel'


def _fq_node_name(name: str, namespace: str) -> str:
    namespace = namespace.rstrip('/')
    return f'{namespace}/{name}' if namespace else f'/{name}'


class MotionSmokeTest(Node):
    """Publish one bounded straight command and an extended zero tail."""

    def __init__(self) -> None:
        super().__init__('motion_smoke_test')
        self.declare_parameter('confirmed', False)
        self.declare_parameter('linear_x', SAFE_LINEAR_X)
        self.declare_parameter('nonzero_duration', MAX_NONZERO_DURATION)
        self.declare_parameter('publish_rate_hz', PUBLISH_RATE_HZ)
        self.declare_parameter('zero_duration', MIN_ZERO_DURATION)
        self._publisher = None
        self._stop_requested = threading.Event()
        self.interrupted_signal = None

    def parameters(self) -> SmokeParameters:
        """Read parameters into the ROS-independent safety representation."""
        return SmokeParameters(
            confirmed=bool(self.get_parameter('confirmed').value),
            linear_x=float(self.get_parameter('linear_x').value),
            nonzero_duration=float(
                self.get_parameter('nonzero_duration').value),
            publish_rate_hz=float(
                self.get_parameter('publish_rate_hz').value),
            zero_duration=float(self.get_parameter('zero_duration').value),
        )

    def print_plan(self, parameters: SmokeParameters) -> None:
        """Print the complete intended physical behavior before authorization."""
        print('WHEELS-RAISED MOTION PLAN')
        print(f'  topic: {COMMAND_TOPIC}')
        print(f'  linear.x: {SAFE_LINEAR_X:.3f} m/s; all other components: 0')
        print(
            f'  nonzero: <= {parameters.nonzero_duration:.3f} s at '
            f'{PUBLISH_RATE_HZ:.1f} Hz')
        print(f'  zero cleanup: >= {parameters.zero_duration:.3f} s')
        print('  requires wheels raised, clear mechanism, and operator stop access')

    def wait_for_graph_discovery(self) -> bool:
        """Wait for two stable controller-chain graph snapshots."""
        started = time.monotonic()
        tracker = StableDiscovery()
        print('GRAPH_DISCOVERY_WAIT_STARTED')
        while time.monotonic() - started < GRAPH_DISCOVERY_TIMEOUT_SEC:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.monotonic() - started
            nodes = normalize_snapshot(self.get_node_names_and_namespaces())
            print(
                f'GRAPH_DISCOVERY_SNAPSHOT elapsed={elapsed:.3f} '
                f'nodes={list(nodes)}')
            if tracker.observe(nodes):
                print(f'GRAPH_DISCOVERY_READY elapsed={elapsed:.3f}')
                return True
        print('GRAPH_DISCOVERY_TIMEOUT')
        return False

    def check_graph(self) -> tuple[str, ...]:
        """Return blocking graph conflicts before creating our publisher."""
        publishers = {}
        own_name = _fq_node_name(self.get_name(), self.get_namespace())
        for topic in ('/cmd_vel', '/controller/cmd_vel', '/cmd_vel_nav'):
            names = {
                _fq_node_name(info.node_name, info.node_namespace)
                for info in self.get_publishers_info_by_topic(topic)
            }
            publishers[topic] = sorted(names - {own_name})
        nodes = [
            name.lstrip('/') for name in normalize_snapshot(
                self.get_node_names_and_namespaces())
            if name != own_name
        ]
        return graph_errors(publishers, nodes)

    def create_command_publisher(self) -> None:
        """Create the publisher only after confirmation and graph validation."""
        self._publisher = self.create_publisher(Twist, COMMAND_TOPIC, 1)

    def request_stop(self, signum: int) -> None:
        """Ask the nonzero loop to end; cleanup remains in the main thread."""
        self.interrupted_signal = signum
        self._stop_requested.set()

    @staticmethod
    def _command(linear_x: float) -> Twist:
        message = Twist()
        message.linear.x = linear_x
        return message

    def _publish_at_rate(
        self, linear_x: float, sample_count: int, stop_on_request: bool,
    ) -> None:
        period = 1.0 / PUBLISH_RATE_HZ
        next_publish = time.monotonic()
        for _index in range(sample_count):
            if stop_on_request and self._stop_requested.is_set():
                break
            self._publisher.publish(self._command(linear_x))
            next_publish += period
            time.sleep(max(0.0, next_publish - time.monotonic()))

    def publish_nonzero(self, duration: float) -> None:
        """Publish the sole permitted nonzero command."""
        sample_count = math.ceil(duration * PUBLISH_RATE_HZ)
        self._publish_at_rate(SAFE_LINEAR_X, sample_count, True)

    def publish_zero_cleanup(self, duration: float) -> None:
        """Best-effort zero publication for at least the configured duration."""
        period = 1.0 / PUBLISH_RATE_HZ
        # Include the endpoint to guarantee a >=duration zero-command span.
        sample_count = math.ceil(duration * PUBLISH_RATE_HZ) + 1
        next_publish = time.monotonic()
        for _index in range(sample_count):
            try:
                self._publisher.publish(self._command(0.0))
            except KeyboardInterrupt:
                # Do not allow a second Ctrl+C to truncate the mandatory zero
                # tail. Record it and finish every remaining zero sample.
                self.request_stop(signal.SIGINT)
            except Exception as exc:  # Continue attempting after transient writes.
                self.get_logger().error(f'zero cleanup publish failed: {exc}')
            next_publish += period
            time.sleep(max(0.0, next_publish - time.monotonic()))


def main(args=None) -> int:
    """Validate, require confirmation, execute once, and always zero afterward."""
    rclpy.init(args=args)
    node = MotionSmokeTest()
    old_handlers = {}
    try:
        parameters = node.parameters()
        node.print_plan(parameters)
        parameter_errors = validate_parameters(parameters)
        if parameter_errors:
            for error in parameter_errors:
                print(f'REFUSED: {error}')
            return 2
        if not parameters.confirmed:
            print('NOT CONFIRMED: no publisher created; no command sent')
            return 2

        node.create_command_publisher()
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.getsignal(signum)
            signal.signal(
                signum,
                lambda received, _frame: node.request_stop(received),
            )

        cleanup_managed = False
        try:
            if not node.wait_for_graph_discovery():
                return 3
            conflicts = node.check_graph()
            if conflicts:
                for conflict in conflicts:
                    print(f'REFUSED: {conflict}')
                return 3
            cleanup_managed = True
            execute_with_zero_cleanup(
                lambda: node.publish_nonzero(parameters.nonzero_duration),
                lambda: node.publish_zero_cleanup(parameters.zero_duration),
            )
        except KeyboardInterrupt:
            # A KeyboardInterrupt raised outside the installed signal handler is
            # still covered because execute_with_zero_cleanup uses finally.
            return 130
        except Exception as exc:
            node.get_logger().error(f'motion smoke test failed: {exc}')
            return 1
        finally:
            if not cleanup_managed:
                node.publish_zero_cleanup(parameters.zero_duration)

        if node.interrupted_signal is not None:
            return 128 + int(node.interrupted_signal)
        return 0
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
