"""M1-A: Drive(d) -> Turn(180 deg) -> Drive(d), closed on /odom."""

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from robot_mission.out_and_back_policy import (
    CMD_VEL_TOPIC, ODOM_TOPIC, Command, MissionConfig, MissionRunner, Pose2D,
    State, normalize_angle, yaw_from_quaternion,
)


class OutAndBackNode(Node):
    """One publisher, one subscriber and one timer around the pure state machine."""

    def __init__(self) -> None:
        super().__init__('out_and_back_test')
        self.declare_parameter('confirmed', False)
        defaults = MissionConfig()
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        self.runner: MissionRunner | None = None
        self.publisher = None
        self.subscription = None
        self.timer = None
        self.pose: Pose2D | None = None
        self.last_odom_time: float | None = None
        self.last_progress_log = -math.inf
        self.reported = False

    def monotonic(self) -> float:
        return time.monotonic()

    def config(self) -> MissionConfig:
        return MissionConfig(**{
            name: float(self.get_parameter(name).value)
            for name in MissionConfig.__dataclass_fields__
        })

    def confirmed(self) -> bool:
        return bool(self.get_parameter('confirmed').value)

    def print_plan(self, config: MissionConfig) -> None:
        self.get_logger().info(
            f'M1-A PLAN: Drive({config.distance_m:.2f} m) -> '
            f'Turn({config.turn_angle_deg:.1f} deg left) -> '
            f'Drive({config.distance_m:.2f} m); cmd_vel={CMD_VEL_TOPIC} '
            f'odom={ODOM_TOPIC} confirmed={self.confirmed()}')

    def start(self, config: MissionConfig) -> None:
        self.runner = MissionRunner(config)
        self.publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, 1)
        self.subscription = self.create_subscription(
            Odometry, ODOM_TOPIC, self._odom_callback, 10)
        self.timer = self.create_timer(1.0 / config.control_rate_hz, self._tick)

    def _odom_callback(self, message: Odometry) -> None:
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        try:
            pose = Pose2D(float(p.x), float(p.y), yaw_from_quaternion(
                float(q.x), float(q.y), float(q.z), float(q.w)))
            if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
                raise ValueError('pose contains non-finite values')
        except ValueError as exc:
            self.get_logger().error(f'invalid odometry ignored: {exc}')
            return
        self.pose = pose
        self.last_odom_time = self.monotonic()

    @staticmethod
    def _twist(command: Command) -> Twist:
        message = Twist()
        message.linear.x = command.linear_x
        message.angular.z = command.angular_z
        return message

    def publish_stop(self, count: int = 1) -> None:
        if self.publisher is None:
            return
        for _ in range(count):
            try:
                self.publisher.publish(Twist())
            except Exception as exc:
                self.get_logger().error(f'zero-speed cleanup failed: {exc}')

    def _tick(self) -> None:
        assert self.runner is not None
        now = self.monotonic()
        try:
            command = self.runner.tick(now, self.pose, self.last_odom_time)
            if command.linear_x and command.angular_z:
                raise RuntimeError('combined linear/angular command rejected')
            self.publisher.publish(self._twist(command))
            self._log_progress(now)
            if self.runner.report_due and not self.reported:
                self.reported = True
                self._report()
        except BaseException as exc:
            self.runner.request_abort(
                f'EXCEPTION_{type(exc).__name__}', self.pose, now)
            self.publish_stop(10)
            self.get_logger().error(f'mission callback aborted: {exc}')
            if not self.reported:
                self.reported = True
                self._report()

    def _log_progress(self, now: float) -> None:
        if now - self.last_progress_log < 1.0 or not self.runner.stats.segments:
            return
        state = self.runner.state
        if state not in (State.EXECUTE_DRIVE, State.EXECUTE_TURN):
            return
        self.last_progress_log = now
        result = self.runner.stats.segments[-1]
        index = self.runner.segment_index + 1
        if result.kind == 'drive':
            self.get_logger().info(
                f'[DRIVE {index}/3] target={result.target:.2f} m '
                f'progress={result.progress:.2f} m '
                f'max_cross={result.max_abs_cross_track:.2f} m')
        else:
            self.get_logger().info(
                f'[TURN {index}/3] target={result.target:.1f} deg '
                f'progress={result.progress:.1f} deg')

    def _report(self) -> None:
        stats = self.runner.stats
        start, final = stats.mission_start, stats.mission_final
        position_error = heading_error = math.nan
        if start is not None and final is not None:
            position_error = math.hypot(final.x - start.x, final.y - start.y)
            heading_error = math.degrees(normalize_angle(final.yaw - start.yaw))
        self.get_logger().info(
            f'M1-A RESULT success={stats.success} reason={stats.reason} '
            f'cmd_vel={CMD_VEL_TOPIC} odom={ODOM_TOPIC} start={start} final={final} '
            f'final_position_error={position_error:.3f} m '
            f'final_heading_delta={heading_error:.1f} deg '
            f'total={stats.total_duration_sec:.2f} s '
            f'odom_timeout={stats.odom_timeout} '
            f'segment_timeout={stats.segment_timeout} '
            f'cross_track_abort={stats.cross_track_abort}')
        for index, result in enumerate(stats.segments, 1):
            unit = 'm' if result.kind == 'drive' else 'deg'
            self.get_logger().info(
                f'SEGMENT {index} kind={result.kind} '
                f'target={result.target:.3f} {unit} '
                f'progress={result.progress:.3f} {unit} '
                f'max_cross={result.max_abs_cross_track:.3f} m '
                f'duration={result.duration_sec:.2f} s')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = OutAndBackNode()
    exit_code = 1
    try:
        config = node.config()
        node.print_plan(config)
        if not node.confirmed():
            node.get_logger().warning(
                'NOT_CONFIRMED: no publisher, subscriber or timer created')
            return 2
        try:
            node.start(config)
        except ValueError as exc:
            node.get_logger().error(f'INVALID_PARAMETERS: {exc}')
            return 2
        while rclpy.ok() and node.runner.state not in (State.DONE, State.ABORT):
            rclpy.spin_once(node)
        exit_code = 0 if node.runner and node.runner.stats.success else 1
    except KeyboardInterrupt:
        if node.runner:
            node.runner.request_abort('KEYBOARD_INTERRUPT', node.pose, node.monotonic())
            if not node.reported:
                node.reported = True
                node._report()
        exit_code = 130
    except BaseException as exc:
        if node.runner:
            node.runner.request_abort(
                f'EXCEPTION_{type(exc).__name__}', node.pose, node.monotonic())
            if not node.reported:
                node.reported = True
                node._report()
        node.get_logger().error(f'mission process aborted: {exc}')
        exit_code = 1
    finally:
        node.publish_stop(10)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
