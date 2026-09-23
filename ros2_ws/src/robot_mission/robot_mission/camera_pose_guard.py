"""Command and guard the audited vendor_init navigation camera pose once."""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from ros_robot_controller_msgs.msg import (
    BusServoState, GetBusServoCmd, ServoPosition, ServosPosition,
    SetBusServoState,
)
from ros_robot_controller_msgs.srv import GetBusServoState
from std_msgs.msg import Bool, String

from .arm_torque_arming import (
    ArmTorqueArmingRunner, ArmTorqueConfig, ArmTorqueState,
)
from .camera_pose_guard_state_machine import (
    CameraPoseGuardRunner, GuardState, PoseGuardConfig, arm_graph_errors,
    endpoint_node_identity, set_state_publisher_errors,
    validate_passive_set_state_publishers,
)


READY_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class CameraPoseGuardNode(Node):
    """One-shot vendor bus-servo command with a latched readiness contract."""

    def __init__(self):
        super().__init__('camera_pose_guard')
        self._declare_parameters()
        self.pose_name = str(self._value('pose_name'))
        self.camera_pose_name = str(self._value('camera_pose_name'))
        if self.pose_name != self.camera_pose_name:
            raise ValueError('pose_name and camera_pose_name must match')
        if self.pose_name != 'vendor_init':
            raise ValueError('camera_pose_guard currently supports only vendor_init')
        self.dry_run = bool(self._value('dry_run'))
        self.confirmed = bool(self._value('confirmed'))
        self.arm_torque_confirmed = bool(
            self._value('arm_torque_confirmed'))
        self.move_duration_sec = float(self._value('move_duration_sec'))
        self.command_timeout_sec = float(self._value('command_timeout_sec'))
        self.command_topic = str(self._value('command_topic'))
        self.torque_command_topic = str(
            self._value('torque_command_topic'))
        self.intermediate_topic = str(self._value('intermediate_topic'))
        self.feedback_service = str(self._value('feedback_service'))
        self.expected_controller_node = str(
            self._value('expected_controller_node'))
        self.allowed_passive_set_state_publishers = (
            validate_passive_set_state_publishers(
                self._value('allowed_passive_set_state_publishers')))
        self.log_throttle_sec = float(self._value('log_throttle_sec'))
        self.require_unique_fixed_tf = bool(
            self._value('require_unique_fixed_tf'))
        self.fixed_tf_node_names = tuple(
            '/' + str(name).strip('/')
            for name in self._value('fixed_tf_node_names'))
        self._validate_node_parameters()

        self.config = PoseGuardConfig(
            servo_ids=tuple(int(value) for value in self._value('servo_ids')),
            servo_positions=tuple(
                int(value) for value in self._value('servo_positions')),
            settle_time_sec=float(self._value('settle_time_sec')),
            controller_wait_timeout_sec=float(
                self._value('controller_wait_timeout_sec')),
            retry_period_sec=float(self._value('retry_period_sec')),
            max_pose_command_attempts=int(
                self._value('max_pose_command_attempts')),
            enable_feedback_check=bool(self._value('enable_feedback_check')),
            allow_time_based_ready=bool(self._value('allow_time_based_ready')),
            feedback_timeout_sec=float(self._value('feedback_timeout_sec')),
            position_tolerance=int(self._value('position_tolerance')),
            reassert_pose=bool(self._value('reassert_pose')),
            reassert_period_sec=float(self._value('reassert_period_sec')),
        )
        self.runner = CameraPoseGuardRunner(self.config)
        self.arm_runner = ArmTorqueArmingRunner(ArmTorqueConfig(
            servo_ids=self.config.servo_ids,
            feedback_timeout_sec=float(
                self._value('arm_feedback_timeout_sec')),
            preload_settle_sec=float(self._value('preload_settle_sec')),
            torque_settle_sec=float(self._value('torque_settle_sec')),
            position_jump_tolerance=int(
                self._value('torque_enable_position_jump_tolerance')),
        ))
        self.command_publisher = None
        self.torque_command_publisher = None
        self.feedback_client = self.create_client(
            GetBusServoState, self.feedback_service)
        self.feedback_future = None
        self.feedback_requested_at = None
        self.feedback_positions = None
        self.feedback_failed = False
        self.arm_feedback_future = None
        self.arm_feedback_requested_at = None
        self.arm_feedback_positions = None
        self.arm_feedback_torque = None
        self.arm_feedback_failed = False
        self.arm_feedback_includes_torque = False
        self.controller_stable_count = 0
        self.last_state = self.runner.state
        self.last_arm_state = self.arm_runner.state
        self.last_ready = False
        self.last_wait_log = -math.inf

        self.ready_publisher = self.create_publisher(
            Bool, str(self._value('ready_topic')), READY_QOS)
        self.state_publisher = self.create_publisher(
            String, str(self._value('state_topic')), 10)
        self._publish_status(force=True)
        self.timer = self.create_timer(0.2, self._tick)
        self.get_logger().warning(
            f'camera pose={self.pose_name}; interface={self.command_topic}; '
            f'servos={list(zip(self.config.servo_ids, self.config.servo_positions))}; '
            f'move_duration_sec={self.move_duration_sec}; '
            f'settle_time_sec={self.config.settle_time_sec}; '
            f'dry_run={self.dry_run}; confirmed={self.confirmed}; '
            f'arm_torque_confirmed={self.arm_torque_confirmed}; '
            f'max_pose_command_attempts={self.config.max_pose_command_attempts}; '
            f'feedback_check={self.config.enable_feedback_check}; '
            f'time_based_ready={self.config.allow_time_based_ready}; '
            'allowed_passive_set_state_publishers='
            f'{self.allowed_passive_set_state_publishers}')

    def _declare_parameters(self):
        defaults = {
            'pose_name': 'vendor_init',
            'camera_pose_name': 'vendor_init',
            'servo_ids': [1, 2, 3, 4],
            'servo_positions': [500, 765, 15, 150],
            'move_duration_sec': 1.0,
            'settle_time_sec': 2.0,
            'command_timeout_sec': 3.0,
            'controller_wait_timeout_sec': 0.0,
            'retry_period_sec': 2.0,
            'max_pose_command_attempts': 1,
            'enable_feedback_check': True,
            'allow_time_based_ready': False,
            'feedback_timeout_sec': 5.0,
            'position_tolerance': 10,
            'reassert_pose': False,
            'reassert_period_sec': 10.0,
            'dry_run': True,
            'confirmed': False,
            'arm_torque_confirmed': False,
            'arm_feedback_timeout_sec': 10.0,
            'preload_settle_sec': 0.5,
            'torque_settle_sec': 0.5,
            'torque_enable_position_jump_tolerance': 10,
            'command_topic': '/ros_robot_controller/bus_servo/set_position',
            'torque_command_topic': '/ros_robot_controller/bus_servo/set_state',
            'intermediate_topic': '/servo_controller',
            'feedback_service': '/ros_robot_controller/bus_servo/get_state',
            'expected_controller_node': '/ros_robot_controller',
            # Default empty. Only a launch with a reviewed, platform-specific
            # static audit may opt in the exact passive Mecanum endpoint.
            'allowed_passive_set_state_publishers': [],
            'ready_topic': '/camera_pose_ready',
            'state_topic': '/camera_pose_guard/state',
            'log_throttle_sec': 2.0,
            'require_unique_fixed_tf': True,
            'fixed_tf_node_names': [
                'fixed_joint1_tf', 'fixed_joint2_tf',
                'fixed_joint3_tf', 'fixed_joint4_tf',
            ],
        }
        for name, value in defaults.items():
            if name == 'allowed_passive_set_state_publishers':
                parameter = self.declare_parameter(
                    name, Parameter.Type.STRING_ARRAY)
                if parameter.type_ is Parameter.Type.NOT_SET:
                    result = self.set_parameters([
                        Parameter(
                            name, Parameter.Type.STRING_ARRAY, [])])[0]
                    if not result.successful:
                        raise ValueError(
                            'failed to initialize empty passive publisher '
                            f'allowlist: {result.reason}')
            else:
                self.declare_parameter(name, value)

    def _value(self, name):
        return self.get_parameter(name).value

    def _validate_node_parameters(self):
        positive = (
            self.move_duration_sec, self.command_timeout_sec,
            self.log_throttle_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('move duration, command timeout and log throttle must be positive')
        if self.move_duration_sec > 30.0:
            raise ValueError('move_duration_sec exceeds vendor 30 second limit')
        expected = '/ros_robot_controller/bus_servo/set_position'
        if self.command_topic != expected:
            raise ValueError(f'command_topic is fixed to {expected}')
        expected_torque = '/ros_robot_controller/bus_servo/set_state'
        if self.torque_command_topic != expected_torque:
            raise ValueError(
                f'torque_command_topic is fixed to {expected_torque}')
        if self.expected_controller_node != '/ros_robot_controller':
            raise ValueError(
                'expected_controller_node is fixed to /ros_robot_controller')
        if (self.require_unique_fixed_tf
                and (not self.fixed_tf_node_names
                     or len(set(self.fixed_tf_node_names))
                     != len(self.fixed_tf_node_names))):
            raise ValueError(
                'fixed_tf_node_names must be non-empty and unique when required')

    def _now(self):
        return time.monotonic()

    def _endpoint_node_name(self, info):
        identity = endpoint_node_identity(
            getattr(info, 'node_name', None),
            getattr(info, 'node_namespace', None))
        if identity is None:
            raise ValueError('endpoint node identity is not canonical')
        return identity

    def _self_node_name(self):
        identity = endpoint_node_identity(
            self.get_name(), self.get_namespace())
        if identity is None:
            raise ValueError('camera_pose_guard node identity is not canonical')
        return identity

    def _publisher_node_names(self, topic):
        return tuple(
            self._endpoint_node_name(info)
            for info in self.get_publishers_info_by_topic(topic))

    def _other_publishers(self, topic):
        self_name = self._self_node_name()
        return tuple(sorted(
            name for name in self._publisher_node_names(topic)
            if name != self_name))

    def _subscribers(self, topic):
        return tuple(sorted(
            self._endpoint_node_name(info)
            for info in self.get_subscriptions_info_by_topic(topic)))

    def _graph_status(self):
        try:
            self_name = self._self_node_name()
            raw_nodes = tuple(self.get_node_names_and_namespaces())
            resolved_nodes = tuple(
                endpoint_node_identity(name, namespace)
                for name, namespace in raw_nodes)
            if any(identity is None for identity in resolved_nodes):
                raise ValueError('node graph contains an unresolved identity')
            nodes = tuple(
                identity for identity in resolved_nodes
                if identity != self_name)
            command_publishers = self._other_publishers(
                self.command_topic)
            intermediate_publishers = self._other_publishers(
                self.intermediate_topic)
            torque_publishers = self._publisher_node_names(
                self.torque_command_topic)
            topic_subscribers = {
                topic: self._subscribers(topic)
                for topic in (self.command_topic, self.torque_command_topic)
            }
        except Exception as error:  # ROS graph APIs are an external boundary.
            self.controller_stable_count = 0
            return False, (
                'graph query or endpoint identity resolution failed: '
                f'{type(error).__name__}: {error}',)

        errors = arm_graph_errors(
            command_publishers, intermediate_publishers, nodes)
        errors += set_state_publisher_errors(
            torque_publishers,
            guard_node_name=self_name,
            allowed_passive_publishers=(
                self.allowed_passive_set_state_publishers))
        for topic in (self.command_topic, self.torque_command_topic):
            subscribers = topic_subscribers[topic]
            if subscribers != (self.expected_controller_node,):
                errors += (
                    f'{topic} requires sole subscriber '
                    f'{self.expected_controller_node}; found '
                    f'{subscribers or ()}',)
        if self.require_unique_fixed_tf:
            for expected_name in self.fixed_tf_node_names:
                count = nodes.count(expected_name)
                if count != 1:
                    errors += (
                        f'fixed TF node {expected_name} requires exactly one '
                        f'instance; found {count}',)
        if errors:
            self.controller_stable_count = 0
            return False, errors
        self.controller_stable_count += 1
        return self.controller_stable_count >= 2, ()

    def _authorized(self):
        return (
            not self.dry_run
            and self.confirmed
            and self.arm_torque_confirmed
        )

    def _ensure_torque_command_publisher(self):
        if self.torque_command_publisher is None:
            self.torque_command_publisher = self.create_publisher(
                SetBusServoState, self.torque_command_topic, 1)

    def _publish_preload_current(self):
        if not self._authorized() or self.arm_runner.current_positions is None:
            return False
        try:
            self._ensure_torque_command_publisher()
            message = SetBusServoState()
            message.duration = self.move_duration_sec
            for servo_id in self.config.servo_ids:
                state = BusServoState()
                state.present_id = [1, servo_id]
                state.position = [
                    1, self.arm_runner.current_positions[servo_id]]
                message.state.append(state)
            self.torque_command_publisher.publish(message)
            self.get_logger().warning(
                'ARM_CURRENT_PRELOADED through sole ros_robot_controller: '
                f'{self.arm_runner.current_positions}')
            return True
        except (KeyError, TypeError, ValueError, RuntimeError) as error:
            self.get_logger().error(f'ARM_CURRENT_PRELOAD_FAILED: {error}')
            return False

    def _publish_torque_enable(self):
        if not self._authorized():
            return False
        try:
            self._ensure_torque_command_publisher()
            message = SetBusServoState()
            for servo_id in self.config.servo_ids:
                state = BusServoState()
                state.present_id = [1, servo_id]
                state.enable_torque = [1, 1]
                message.state.append(state)
            self.torque_command_publisher.publish(message)
            self.get_logger().warning(
                'ARM_TORQUE_ENABLE_PUBLISHED once through sole '
                'ros_robot_controller')
            return True
        except (TypeError, ValueError, RuntimeError) as error:
            self.get_logger().error(f'ARM_TORQUE_ENABLE_FAILED: {error}')
            return False

    def _publish_pose_command(self):
        if not self._authorized():
            return False
        try:
            if self.command_publisher is None:
                self.command_publisher = self.create_publisher(
                    ServosPosition, self.command_topic, 1)
            message = ServosPosition()
            message.duration = self.move_duration_sec
            message.position = []
            for servo_id, position in zip(
                    self.config.servo_ids, self.config.servo_positions):
                target = ServoPosition()
                target.id = servo_id
                target.position = position
                message.position.append(target)
            self.command_publisher.publish(message)
            self.get_logger().warning(
                f'POSE_COMMAND_PUBLISHED: {self.pose_name}; '
                f'attempt={self.runner.command_attempts}/'
                f'{self.config.max_pose_command_attempts}; '
                f'duration={self.move_duration_sec}s')
            return True
        except (TypeError, ValueError, RuntimeError) as error:
            self.get_logger().error(f'POSE_COMMAND_FAILED: {error}')
            return False

    def _request_feedback(self, now):
        if not self.config.enable_feedback_check:
            return
        if self.feedback_future is not None:
            if not self.feedback_future.done():
                if (self.feedback_requested_at is not None
                        and now - self.feedback_requested_at
                        > self.config.feedback_timeout_sec):
                    self.feedback_failed = True
                    self.feedback_future.cancel()
                    self.feedback_future = None
                return
            try:
                response = self.feedback_future.result()
                if response is None or not response.success:
                    self.feedback_failed = True
                else:
                    positions = {}
                    for servo_id, state in zip(
                            self.config.servo_ids, response.state):
                        if state.position:
                            positions[servo_id] = int(state.position[-1])
                    self.feedback_positions = positions or None
                    self.feedback_failed = self.feedback_positions is None
            except (RuntimeError, TypeError, ValueError) as error:
                self.feedback_failed = True
                self.get_logger().warning(f'POSE_FEEDBACK_FAILED: {error}')
            self.feedback_future = None
            return
        if not self.feedback_client.service_is_ready():
            self.feedback_failed = True
            return
        request = GetBusServoState.Request()
        for servo_id in self.config.servo_ids:
            command = GetBusServoCmd()
            command.id = servo_id
            command.get_position = 1
            request.cmd.append(command)
        self.feedback_positions = None
        self.feedback_failed = False
        self.feedback_requested_at = now
        self.feedback_future = self.feedback_client.call_async(request)

    def _request_arm_feedback(self, now, include_torque):
        if self.arm_feedback_future is not None:
            if not self.arm_feedback_future.done():
                if (self.arm_feedback_requested_at is not None
                        and now - self.arm_feedback_requested_at
                        > self.arm_runner.config.feedback_timeout_sec):
                    self.arm_feedback_failed = True
                    self.arm_feedback_future.cancel()
                    self.arm_feedback_future = None
                return
            try:
                response = self.arm_feedback_future.result()
                if response is None or not response.success:
                    self.arm_feedback_failed = True
                else:
                    positions = {}
                    torque_states = {}
                    for servo_id, state in zip(
                            self.config.servo_ids, response.state):
                        if state.position:
                            positions[servo_id] = int(state.position[-1])
                        if self.arm_feedback_includes_torque:
                            if state.enable_torque:
                                torque_states[servo_id] = int(
                                    state.enable_torque[-1])
                    self.arm_feedback_positions = positions or None
                    self.arm_feedback_torque = (
                        torque_states or None
                        if self.arm_feedback_includes_torque else None)
                    self.arm_feedback_failed = (
                        self.arm_feedback_positions is None
                        or (self.arm_feedback_includes_torque
                            and self.arm_feedback_torque is None))
            except (RuntimeError, TypeError, ValueError) as error:
                self.arm_feedback_failed = True
                self.get_logger().warning(
                    f'ARM_STATE_FEEDBACK_FAILED: {error}')
            self.arm_feedback_future = None
            return
        if not self.feedback_client.service_is_ready():
            self.arm_feedback_failed = True
            return
        request = GetBusServoState.Request()
        for servo_id in self.config.servo_ids:
            command = GetBusServoCmd()
            command.id = servo_id
            command.get_position = 1
            if include_torque:
                command.get_torque_state = 1
            request.cmd.append(command)
        self.arm_feedback_positions = None
        self.arm_feedback_torque = None
        self.arm_feedback_failed = False
        self.arm_feedback_includes_torque = include_torque
        self.arm_feedback_requested_at = now
        self.arm_feedback_future = self.feedback_client.call_async(request)

    def _publish_status(self, force=False):
        ready = bool(self.runner.ready)
        if force or ready != self.last_ready:
            self.ready_publisher.publish(Bool(data=ready))
            self.get_logger().warning(f'camera_pose_ready={ready}')
            self.last_ready = ready
        state_name = (
            self.runner.state.name if self.arm_runner.ready
            else f'ARM_{self.arm_runner.state.name}')
        self.state_publisher.publish(String(data=state_name))

    def _tick(self):
        now = self._now()
        graph_ready, graph_errors = self._graph_status()
        controller_available = self._authorized() and graph_ready
        if not controller_available and now - self.last_wait_log >= self.log_throttle_sec:
            details = '; '.join(graph_errors) if graph_errors else (
                'dry_run, confirmed=false, or arm_torque_confirmed=false: '
                'command path disabled')
            self.get_logger().warning(
                f'WAIT_FOR_CONTROLLER: {details}; node remains active')
            self.last_wait_log = now

        if not self.arm_runner.ready:
            arm_decision = self.arm_runner.tick(
                now, controller_available=controller_available,
                positions=self.arm_feedback_positions,
                torque_states=self.arm_feedback_torque,
                feedback_failed=self.arm_feedback_failed)
            self.arm_feedback_positions = None
            self.arm_feedback_torque = None
            self.arm_feedback_failed = False
            if arm_decision.preload_current:
                self.arm_runner.preload_result(
                    now, self._publish_preload_current())
            if arm_decision.enable_torque:
                self.arm_runner.enable_result(
                    now, self._publish_torque_enable())
            if arm_decision.request_positions:
                self._request_arm_feedback(now, include_torque=False)
            if arm_decision.request_positions_and_torque:
                self._request_arm_feedback(now, include_torque=True)
            if self.arm_runner.state is not self.last_arm_state:
                self.get_logger().info(
                    f'ARM_STATE {self.last_arm_state.name} -> '
                    f'{self.arm_runner.state.name}: '
                    f'{self.arm_runner.reason}')
                self.last_arm_state = self.arm_runner.state
            if not self.arm_runner.ready:
                self._publish_status()
                return

        decision = self.runner.tick(
            now, controller_available=controller_available,
            feedback_positions=self.feedback_positions,
            feedback_failed=self.feedback_failed)
        self.feedback_positions = None
        self.feedback_failed = False
        if decision.send_command:
            self.runner.command_result(now, self._publish_pose_command())
        if decision.request_feedback:
            self._request_feedback(now)
        if self.runner.state is not self.last_state:
            self.get_logger().info(
                f'POSE_STATE {self.last_state.name} -> '
                f'{self.runner.state.name}: {self.runner.reason}')
            self.last_state = self.runner.state
        self._publish_status()

    def publish_not_ready(self):
        self.runner.ready = False
        if rclpy.ok():
            self.ready_publisher.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseGuardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.publish_not_ready()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # ros2 launch can forward a second SIGINT while the first one is
            # already unwinding the node. No actuator cleanup is performed here.
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
