"""ROS-independent policy for commanding one fixed camera pose."""

from collections import Counter
from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Iterable, Mapping, Optional


AUDITED_PASSIVE_SET_STATE_PUBLISHERS = frozenset({'/odom_publisher'})


def endpoint_node_identity(
    node_name: object, node_namespace: object,
) -> Optional[str]:
    """Return one canonical endpoint owner, or ``None`` if it is ambiguous."""
    if not isinstance(node_name, str) or not node_name:
        return None
    if node_name.startswith('/') or '/' in node_name:
        return None
    if not isinstance(node_namespace, str) or not node_namespace:
        return None
    if node_namespace == '/':
        return f'/{node_name}'
    if (not node_namespace.startswith('/')
            or node_namespace.endswith('/')
            or '//' in node_namespace):
        return None
    return f'{node_namespace}/{node_name}'


def validate_passive_set_state_publishers(
    publisher_names: Iterable[object],
) -> tuple[str, ...]:
    """Accept only reviewed, canonical passive owners; default is empty."""
    try:
        names = tuple(publisher_names)
    except TypeError as error:
        raise ValueError(
            'allowed passive set-state publishers must be an array') from error
    if any(not isinstance(name, str) for name in names):
        raise ValueError(
            'allowed passive set-state publishers must be strings')
    if len(set(names)) != len(names):
        raise ValueError(
            'allowed passive set-state publishers must be unique')
    unknown = sorted(
        name for name in names
        if name not in AUDITED_PASSIVE_SET_STATE_PUBLISHERS)
    if unknown:
        raise ValueError(
            'unreviewed passive set-state publisher(s): '
            + ','.join(unknown))
    return names


def set_state_publisher_errors(
    publisher_node_names: tuple[Optional[str], ...],
    *,
    guard_node_name: str,
    allowed_passive_publishers: tuple[str, ...] = (),
) -> tuple[str, ...]:
    """Validate every torque-topic endpoint by exact owner identity."""
    allowed = validate_passive_set_state_publishers(
        allowed_passive_publishers)
    errors = []
    unresolved_count = sum(
        name is None for name in publisher_node_names)
    if unresolved_count:
        errors.append(
            'bus-servo torque publisher endpoint identity unresolved: '
            f'{unresolved_count}')

    resolved = tuple(
        name for name in publisher_node_names if name is not None)
    counts = Counter(resolved)
    permitted = {guard_node_name, *allowed}
    unknown = sorted(
        name for name in counts if name not in permitted)
    if unknown:
        errors.append(
            'unknown bus-servo torque publisher(s): '
            + ','.join(unknown))

    duplicates = sorted(
        name for name, count in counts.items() if count > 1)
    if duplicates:
        errors.append(
            'duplicate bus-servo torque publisher endpoint(s): '
            + ','.join(
                f'{name}={counts[name]}' for name in duplicates))
    return tuple(errors)


class GuardState(Enum):
    WAIT_FOR_CONTROLLER = auto()
    COMMAND_POSE = auto()
    SETTLING = auto()
    READY = auto()
    DEGRADED = auto()
    FAULT = auto()


@dataclass(frozen=True)
class PoseGuardConfig:
    servo_ids: tuple[int, ...] = (1, 2, 3, 4)
    servo_positions: tuple[int, ...] = (500, 765, 15, 150)
    settle_time_sec: float = 2.0
    controller_wait_timeout_sec: float = 0.0
    retry_period_sec: float = 2.0
    max_pose_command_attempts: int = 1
    enable_feedback_check: bool = False
    allow_time_based_ready: bool = True
    feedback_timeout_sec: float = 1.0
    position_tolerance: int = 10
    reassert_pose: bool = False
    reassert_period_sec: float = 10.0


@dataclass(frozen=True)
class GuardDecision:
    state: GuardState
    ready: bool
    send_command: bool = False
    request_feedback: bool = False
    reason: str = ''


def arm_graph_errors(
    command_publishers: tuple[str, ...],
    intermediate_publishers: tuple[str, ...],
    node_names: tuple[str, ...],
) -> tuple[str, ...]:
    """Reject a second arm-command owner before the guard publishes."""
    errors = []
    if command_publishers:
        errors.append(
            'bus-servo command topic already has publisher(s): '
            + ','.join(command_publishers))
    if intermediate_publishers:
        errors.append(
            '/servo_controller already has publisher(s): '
            + ','.join(intermediate_publishers))
    forbidden = ('init_pose', 'joystick_control', 'controller_manager')
    conflicts = sorted({
        name for name in node_names
        if any(token in name for token in forbidden)
    })
    if conflicts:
        errors.append('competing arm node(s): ' + ','.join(conflicts))
    return tuple(errors)


def validate_config(config: PoseGuardConfig) -> None:
    if not config.servo_ids or len(config.servo_ids) != len(config.servo_positions):
        raise ValueError('servo_ids and servo_positions must be non-empty and equal')
    if len(set(config.servo_ids)) != len(config.servo_ids):
        raise ValueError('servo_ids must be unique')
    if not all(1 <= value <= 253 for value in config.servo_ids):
        raise ValueError('servo_ids must be between 1 and 253')
    if not all(0 <= value <= 1000 for value in config.servo_positions):
        raise ValueError('servo positions must be pulse values from 0 to 1000')
    positive = (
        config.settle_time_sec, config.retry_period_sec,
        config.feedback_timeout_sec, config.reassert_period_sec,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        raise ValueError('settle, retry, feedback and reassert periods must be positive')
    if (not math.isfinite(config.controller_wait_timeout_sec)
            or config.controller_wait_timeout_sec < 0.0):
        raise ValueError('controller_wait_timeout_sec cannot be negative')
    if config.position_tolerance < 0:
        raise ValueError('position_tolerance cannot be negative')
    if config.max_pose_command_attempts < 1:
        raise ValueError('max_pose_command_attempts must be at least one')
    if not config.enable_feedback_check and not config.allow_time_based_ready:
        raise ValueError(
            'time-based ready is required when feedback checking is disabled')


class CameraPoseGuardRunner:
    """Pure state machine; ROS publishing and service calls stay outside it."""

    def __init__(self, config: PoseGuardConfig = PoseGuardConfig()) -> None:
        validate_config(config)
        self.config = config
        self.state = GuardState.WAIT_FOR_CONTROLLER
        self.ready = False
        self.reason = 'initializing'
        self.started_at: Optional[float] = None
        self.state_started_at: Optional[float] = None
        self.command_sent_at: Optional[float] = None
        self.last_attempt_at: Optional[float] = None
        self.command_attempts = 0
        self.ready_at: Optional[float] = None
        self.awaiting_command_result = False
        self.time_based_degraded = False

    def _transition(self, state: GuardState, now: float, reason: str) -> None:
        self.state = state
        self.state_started_at = now
        self.ready = state is GuardState.READY
        self.reason = reason

    def _elapsed(self, now: float) -> float:
        origin = self.state_started_at if self.state_started_at is not None else now
        return max(0.0, now - origin)

    def _retry_due(self, now: float) -> bool:
        return (self.last_attempt_at is None
                or now - self.last_attempt_at >= self.config.retry_period_sec)

    def _feedback_matches(self, feedback: Mapping[int, int]) -> bool:
        targets = dict(zip(self.config.servo_ids, self.config.servo_positions))
        return all(
            servo_id in feedback
            and abs(int(feedback[servo_id]) - target)
            <= self.config.position_tolerance
            for servo_id, target in targets.items())

    def command_result(self, now: float, success: bool) -> None:
        """Report the result of the one command requested by ``tick``."""
        if self.state is not GuardState.COMMAND_POSE:
            raise RuntimeError('command_result is only valid in COMMAND_POSE')
        self.awaiting_command_result = False
        self.last_attempt_at = now
        if success:
            self.command_sent_at = now
            self._transition(GuardState.SETTLING, now, 'command_published')
        elif self.command_attempts < self.config.max_pose_command_attempts:
            self.reason = 'command_failed_retry_pending'
        else:
            self._transition(GuardState.FAULT, now, 'command_attempts_exhausted')

    def tick(
        self,
        now: float,
        *,
        controller_available: bool,
        feedback_positions: Optional[Mapping[int, int]] = None,
        feedback_failed: bool = False,
    ) -> GuardDecision:
        if self.started_at is None:
            self.started_at = now
            self.state_started_at = now

        if self.state is GuardState.WAIT_FOR_CONTROLLER:
            if controller_available:
                self._transition(
                    GuardState.COMMAND_POSE, now, 'controller_available')
            elif (self.config.controller_wait_timeout_sec > 0.0
                  and now - self.started_at
                  >= self.config.controller_wait_timeout_sec):
                self._transition(
                    GuardState.DEGRADED, now, 'controller_wait_timeout')

        if self.state is GuardState.COMMAND_POSE:
            if not controller_available:
                self._transition(
                    GuardState.WAIT_FOR_CONTROLLER, now, 'controller_lost')
            elif (not self.awaiting_command_result
                  and self.command_attempts
                  >= self.config.max_pose_command_attempts):
                self._transition(
                    GuardState.FAULT, now, 'command_attempts_exhausted')
            elif not self.awaiting_command_result and self._retry_due(now):
                self.awaiting_command_result = True
                self.command_attempts += 1
                return GuardDecision(
                    self.state, False, send_command=True,
                    reason=(
                        'send_navigation_pose_attempt_'
                        f'{self.command_attempts}'))

        elif self.state is GuardState.SETTLING:
            if not controller_available:
                self._transition(
                    GuardState.DEGRADED, now, 'controller_lost_while_settling')
            elif self._elapsed(now) >= self.config.settle_time_sec:
                if not self.config.enable_feedback_check:
                    self._transition(
                        GuardState.READY, now, 'time_based_ready')
                    self.ready_at = now
                elif feedback_positions is not None:
                    if self._feedback_matches(feedback_positions):
                        self._transition(
                            GuardState.READY, now, 'feedback_confirmed')
                        self.ready_at = now
                    else:
                        self._transition(
                            GuardState.FAULT, now, 'feedback_out_of_tolerance')
                elif (feedback_failed
                      or self._elapsed(now) >= (
                          self.config.settle_time_sec
                          + self.config.feedback_timeout_sec)):
                    if self.config.allow_time_based_ready:
                        self.time_based_degraded = True
                        self._transition(
                            GuardState.DEGRADED, now,
                            'feedback_unavailable_using_time_based_ready')
                    else:
                        self._transition(
                            GuardState.FAULT, now, 'feedback_unavailable')
                else:
                    return GuardDecision(
                        self.state, False, request_feedback=True,
                        reason='waiting_for_feedback')

        elif self.state is GuardState.READY:
            if not controller_available:
                self._transition(
                    GuardState.DEGRADED, now, 'controller_lost_after_ready')
            elif self.config.enable_feedback_check and feedback_failed:
                self._transition(
                    GuardState.DEGRADED, now, 'feedback_lost_after_ready')
            elif (self.config.enable_feedback_check
                  and feedback_positions is not None
                  and not self._feedback_matches(feedback_positions)):
                self._transition(
                    GuardState.DEGRADED, now, 'pose_feedback_changed')
            elif (self.config.reassert_pose and self.ready_at is not None
                  and now - self.ready_at >= self.config.reassert_period_sec):
                if (self.command_attempts
                        >= self.config.max_pose_command_attempts):
                    self._transition(
                        GuardState.FAULT, now,
                        'reassert_blocked_by_command_attempt_limit')
                else:
                    self._transition(
                        GuardState.COMMAND_POSE, now, 'scheduled_reassert')

        elif self.state is GuardState.DEGRADED:
            if self.time_based_degraded and controller_available:
                self.time_based_degraded = False
                self._transition(
                    GuardState.READY, now, 'time_based_ready_after_degraded')
                self.ready_at = now
        elif self.state is GuardState.FAULT:
            # FAULT is terminal. A fresh operator-authorized process is required
            # before any additional physical pose command can be published.
            pass

        request_feedback = (
            self.config.enable_feedback_check
            and (self.state is GuardState.READY
                 or (self.state is GuardState.SETTLING
                     and self._elapsed(now)
                     >= self.config.settle_time_sec)))
        return GuardDecision(
            self.state, self.ready, request_feedback=request_feedback,
            reason=self.reason)
