"""ROS-independent policy and execution state machine for a 30-degree turn."""

from dataclasses import dataclass
import math
from typing import Protocol


COMMAND_TOPIC = '/cmd_vel'
PUBLISH_RATE_HZ = 20.0
PUBLISH_PERIOD_SEC = 0.05
ANGULAR_SPEED_RAD_S = 0.20
NONZERO_MESSAGE_COUNT = 52
COMMAND_INTEGRATION_SEC = 2.60
THEORETICAL_ANGLE_RAD = 0.52
THEORETICAL_ANGLE_DEG = math.degrees(THEORETICAL_ANGLE_RAD)
NONZERO_FIRST_LAST_SPAN_SEC = 2.55
SUBSCRIBER_TIMEOUT_SEC = 10.0
PREZERO_MIN_DURATION_SEC = 1.0
PREZERO_MIN_MESSAGES = 21
CLEANUP_MIN_DURATION_SEC = 2.0
CLEANUP_MIN_MESSAGES = 41

FORBIDDEN_OVERRIDES = frozenset({
    'angular_speed',
    'angular_z',
    'publish_rate',
    'publish_rate_hz',
    'message_count',
    'nonzero_message_count',
    'duration',
    'nonzero_duration',
})


@dataclass(frozen=True)
class TwistSpec:
    """All six velocity components, independent of ROS messages."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class PhaseStats:
    """Successful publish count and first-to-last monotonic span."""

    count: int
    span_sec: float


@dataclass(frozen=True)
class RunResult:
    """Final offline-testable outcome."""

    completed: bool
    reason: str
    prezero: PhaseStats
    turn: PhaseStats
    cleanup: PhaseStats
    graph_checks: tuple[bool, ...]


class Adapter(Protocol):
    """Small boundary implemented by ROS at runtime and fakes in tests."""

    def monotonic(self) -> float: ...
    def sleep(self, duration: float) -> None: ...
    def log(self, message: str) -> None: ...
    def wait_for_graph_discovery(self) -> bool: ...
    def graph_errors(self) -> tuple[str, ...]: ...
    def subscriber_count(self) -> int: ...
    def publish(self, command: TwistSpec) -> None: ...


class TurnAborted(RuntimeError):
    """Expected safety refusal that still requires zero cleanup."""


def direction_to_angular_z(direction: str) -> float:
    """Map the only two accepted directions to the fixed angular speed."""
    if direction == 'left':
        return ANGULAR_SPEED_RAD_S
    if direction == 'right':
        return -ANGULAR_SPEED_RAD_S
    raise ValueError('direction must be exactly "left" or "right"')


def turn_command(direction: str) -> TwistSpec:
    """Return a pure in-place command with no translational component."""
    command = TwistSpec(angular_z=direction_to_angular_z(direction))
    validate_twist(command, allow_nonzero=True)
    return command


def validate_twist(command: TwistSpec, allow_nonzero: bool) -> None:
    """Reject translation, roll/pitch rate, or an arbitrary yaw rate."""
    if any((command.linear_x, command.linear_y, command.linear_z)):
        raise ValueError('all linear components must be zero')
    if command.angular_x or command.angular_y:
        raise ValueError('angular.x and angular.y must be zero')
    allowed_yaw = {0.0}
    if allow_nonzero:
        allowed_yaw.update({ANGULAR_SPEED_RAD_S, -ANGULAR_SPEED_RAD_S})
    if command.angular_z not in allowed_yaw:
        raise ValueError('angular.z is not an allowed fixed value')


def reject_safety_overrides(names: set[str]) -> None:
    """Reject attempts to expose fixed motion constants as user parameters."""
    attempted = sorted(names & FORBIDDEN_OVERRIDES)
    if attempted:
        raise ValueError('fixed safety parameter override: ' + ', '.join(attempted))


def theoretical_values() -> dict[str, float | int]:
    """Return the fixed count/integration values used in reports and tests."""
    return {
        'message_count': NONZERO_MESSAGE_COUNT,
        'integration_sec': NONZERO_MESSAGE_COUNT * PUBLISH_PERIOD_SEC,
        'angle_rad': ANGULAR_SPEED_RAD_S * COMMAND_INTEGRATION_SEC,
        'angle_deg': math.degrees(
            ANGULAR_SPEED_RAD_S * COMMAND_INTEGRATION_SEC),
        'first_last_span_sec': (
            NONZERO_MESSAGE_COUNT - 1) * PUBLISH_PERIOD_SEC,
    }


def _stats(times: list[float]) -> PhaseStats:
    span = times[-1] - times[0] if len(times) > 1 else 0.0
    return PhaseStats(len(times), span)


class TurnStepRunner:
    """Execute the two-check, prezero, fixed-turn, and cleanup protocol."""

    def __init__(self, adapter: Adapter) -> None:
        self.adapter = adapter
        self.stop_requested = False
        self.cleanup_active = False
        self.signal_count = 0
        self.signal_reason = ''
        self.prezero_times: list[float] = []
        self.turn_times: list[float] = []

    def request_signal(self, signum: int) -> None:
        """First signal requests stop; later cleanup signals are ignored."""
        self.signal_count += 1
        if self.cleanup_active and self.signal_count >= 2:
            self.adapter.log('SECOND_SIGNAL_IGNORED')
            return
        if not self.stop_requested:
            self.signal_reason = f'SIGNAL_{signum}'
            self.stop_requested = True

    def _abort_if_requested(self) -> None:
        if self.stop_requested:
            raise TurnAborted(self.signal_reason or 'STOP_REQUESTED')

    def _graph_check(self) -> bool:
        self.adapter.log('GRAPH_CHECK_STARTED')
        errors = self.adapter.graph_errors()
        if errors:
            for error in errors:
                self.adapter.log('GRAPH_CHECK_FAILED ' + error)
            return False
        self.adapter.log('GRAPH_CHECK_PASSED')
        return True

    def _publish_counted(
        self,
        command: TwistSpec,
        count: int,
        interruptible: bool,
        times: list[float],
    ) -> list[float]:
        next_publish = self.adapter.monotonic()
        for _index in range(count):
            if interruptible:
                self._abort_if_requested()
            self.adapter.publish(command)
            times.append(self.adapter.monotonic())
            next_publish += PUBLISH_PERIOD_SEC
            self.adapter.sleep(max(0.0, next_publish - self.adapter.monotonic()))
        return times

    def _wait_for_subscriber(self) -> None:
        self.adapter.log('WAITING_FOR_SUBSCRIBER')
        deadline = self.adapter.monotonic() + SUBSCRIBER_TIMEOUT_SEC
        while self.adapter.monotonic() < deadline:
            self._abort_if_requested()
            if self.adapter.subscriber_count() >= 1:
                self.adapter.log('SUBSCRIBER_READY')
                return
            self.adapter.sleep(PUBLISH_PERIOD_SEC)
        self.adapter.log('SUBSCRIBER_TIMEOUT')
        raise TurnAborted('SUBSCRIBER_TIMEOUT')

    def _cleanup(self) -> PhaseStats:
        self.cleanup_active = True
        self.adapter.log('ZERO_CLEANUP_STARTED')
        times = []
        next_publish = self.adapter.monotonic()
        while True:
            try:
                self.adapter.publish(TwistSpec())
                times.append(self.adapter.monotonic())
            except KeyboardInterrupt:
                self.request_signal(2)
            except Exception as exc:
                self.adapter.log(f'ZERO_PUBLISH_FAILED {exc}')
            stats = _stats(times)
            if (stats.count >= CLEANUP_MIN_MESSAGES
                    and stats.span_sec >= CLEANUP_MIN_DURATION_SEC):
                break
            next_publish += PUBLISH_PERIOD_SEC
            self.adapter.sleep(max(0.0, next_publish - self.adapter.monotonic()))
        self.adapter.log(
            f'ZERO_CLEANUP_FINISHED count={stats.count} '
            f'span={stats.span_sec:.3f}')
        return stats

    def run(self, direction: str) -> RunResult:
        """Run after confirmation and publisher creation."""
        command = turn_command(direction)
        reason = 'UNEXPECTED_EXCEPTION'
        completed = False
        prezero = PhaseStats(0, 0.0)
        turn = PhaseStats(0, 0.0)
        cleanup = PhaseStats(0, 0.0)
        graph_results = []
        try:
            if not self.adapter.wait_for_graph_discovery():
                raise TurnAborted('GRAPH_DISCOVERY_TIMEOUT')
            first = self._graph_check()
            graph_results.append(first)
            if not first:
                raise TurnAborted('FIRST_GRAPH_CHECK_FAILED')
            self._wait_for_subscriber()

            self.adapter.log('PREZERO_STARTED')
            prezero = _stats(self._publish_counted(
                TwistSpec(), PREZERO_MIN_MESSAGES, True,
                self.prezero_times))
            self.adapter.log(
                f'PREZERO_FINISHED count={prezero.count} '
                f'span={prezero.span_sec:.3f}')

            second = self._graph_check()
            graph_results.append(second)
            if not second:
                raise TurnAborted('SECOND_GRAPH_CHECK_FAILED')

            self.adapter.log('TURN_STARTED')
            turn = _stats(self._publish_counted(
                command, NONZERO_MESSAGE_COUNT, True, self.turn_times))
            self.adapter.log(
                f'TURN_FINISHED count={turn.count} span={turn.span_sec:.3f}')
            completed = True
            reason = 'COMPLETED'
        except TurnAborted as exc:
            reason = str(exc)
        except BaseException as exc:
            reason = type(exc).__name__
            raise
        finally:
            prezero = _stats(self.prezero_times)
            turn = _stats(self.turn_times)
            cleanup = self._cleanup()
            self.adapter.log(
                f'FINAL_STATS prezero_count={prezero.count} '
                f'prezero_span={prezero.span_sec:.3f} '
                f'nonzero_count={turn.count} '
                f'nonzero_span={turn.span_sec:.3f} '
                f'cleanup_count={cleanup.count} '
                f'cleanup_span={cleanup.span_sec:.3f} '
                f'graph_checks={tuple(graph_results)} '
                f'integration_window={COMMAND_INTEGRATION_SEC:.2f} '
                f'theoretical_angle={THEORETICAL_ANGLE_RAD:.2f}rad/'
                f'{THEORETICAL_ANGLE_DEG:.1f}deg reason={reason}')
            if completed:
                self.adapter.log('TEST_COMPLETED reason=' + reason)
            else:
                self.adapter.log('TEST_ABORTED reason=' + reason)

        return RunResult(
            completed, reason, prezero, turn, cleanup, tuple(graph_results))
