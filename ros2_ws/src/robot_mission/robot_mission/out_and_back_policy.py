"""ROS-independent M1-A odometry-closed-loop out-and-back state machine."""

from dataclasses import dataclass, field
from enum import Enum, auto
import math
from typing import Union


CMD_VEL_TOPIC = '/cmd_vel'
ODOM_TOPIC = '/odom'


class State(Enum):
    WAIT_ODOM = auto()
    START_SEGMENT = auto()
    EXECUTE_DRIVE = auto()
    EXECUTE_TURN = auto()
    SETTLE = auto()
    FINAL_STOP = auto()
    DONE = auto()
    ABORT = auto()


@dataclass(frozen=True)
class Drive:
    distance_m: float


@dataclass(frozen=True)
class Turn:
    angle_deg: float


Segment = Union[Drive, Turn]


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Command:
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class MissionConfig:
    distance_m: float = 10.0
    linear_speed_mps: float = 0.15
    turn_angle_deg: float = 180.0
    angular_speed_radps: float = 0.20
    position_tolerance_m: float = 0.05
    angle_tolerance_deg: float = 3.0
    settle_time_sec: float = 1.5
    control_rate_hz: float = 20.0
    odom_timeout_sec: float = 0.5
    max_cross_track_error_m: float = 0.75
    final_stop_duration_sec: float = 2.0
    timeout_scale: float = 2.0
    timeout_margin_sec: float = 10.0
    total_mission_timeout_sec: float = 240.0


@dataclass
class SegmentResult:
    kind: str
    target: float
    progress: float = 0.0
    max_abs_cross_track: float = 0.0
    duration_sec: float = 0.0


@dataclass
class MissionStats:
    success: bool = False
    reason: str = 'NOT_FINISHED'
    mission_start: Pose2D | None = None
    mission_final: Pose2D | None = None
    segments: list[SegmentResult] = field(default_factory=list)
    total_duration_sec: float = 0.0
    odom_timeout: bool = False
    segment_timeout: bool = False
    cross_track_abort: bool = False


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion must contain finite values')
    if sum(value * value for value in values) < 1e-12:
        raise ValueError('quaternion norm is zero')
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def project_along_track(start: Pose2D, current: Pose2D) -> float:
    dx, dy = current.x - start.x, current.y - start.y
    return dx * math.cos(start.yaw) + dy * math.sin(start.yaw)


def project_cross_track(start: Pose2D, current: Pose2D) -> float:
    dx, dy = current.x - start.x, current.y - start.y
    return -dx * math.sin(start.yaw) + dy * math.cos(start.yaw)


def validate_config(config: MissionConfig) -> tuple[str, ...]:
    errors = []
    positive = {
        'distance_m': config.distance_m,
        'linear_speed_mps': config.linear_speed_mps,
        'turn_angle_deg': config.turn_angle_deg,
        'angular_speed_radps': config.angular_speed_radps,
        'position_tolerance_m': config.position_tolerance_m,
        'angle_tolerance_deg': config.angle_tolerance_deg,
        'settle_time_sec': config.settle_time_sec,
        'control_rate_hz': config.control_rate_hz,
        'odom_timeout_sec': config.odom_timeout_sec,
        'max_cross_track_error_m': config.max_cross_track_error_m,
        'final_stop_duration_sec': config.final_stop_duration_sec,
        'timeout_scale': config.timeout_scale,
        'total_mission_timeout_sec': config.total_mission_timeout_sec,
    }
    for name, value in positive.items():
        if not math.isfinite(value) or value <= 0.0:
            errors.append(f'{name} must be finite and > 0')
    if not math.isfinite(config.timeout_margin_sec) or config.timeout_margin_sec < 0:
        errors.append('timeout_margin_sec must be finite and >= 0')
    if config.position_tolerance_m >= config.distance_m:
        errors.append('position_tolerance_m must be smaller than distance_m')
    if config.angle_tolerance_deg >= config.turn_angle_deg:
        errors.append('angle_tolerance_deg must be smaller than turn_angle_deg')
    return tuple(errors)


def default_segments(config: MissionConfig) -> tuple[Segment, ...]:
    return (Drive(config.distance_m), Turn(config.turn_angle_deg),
            Drive(config.distance_m))


class MissionRunner:
    """Non-blocking state machine; one tick produces at most one command."""

    def __init__(self, config: MissionConfig) -> None:
        errors = validate_config(config)
        if errors:
            raise ValueError('; '.join(errors))
        self.config = config
        self.segments = default_segments(config)
        self.state = State.WAIT_ODOM
        self.stats = MissionStats()
        self.segment_index = 0
        self.segment_start: Pose2D | None = None
        self.segment_started_at = 0.0
        self.state_started_at = 0.0
        self.mission_started_at = 0.0
        self.previous_yaw = 0.0
        self.accumulated_turn = 0.0
        self.report_due = False

    def _abort(self, reason: str, pose: Pose2D | None, now: float) -> Command:
        if self.stats.segments and self.stats.segments[-1].duration_sec == 0.0:
            self.stats.segments[-1].duration_sec = max(
                0.0, now - self.segment_started_at)
        self.state = State.ABORT
        self.stats.reason = reason
        self.stats.mission_final = pose
        if self.stats.mission_start is not None:
            self.stats.total_duration_sec = now - self.mission_started_at
        self.report_due = True
        return Command()

    def request_abort(self, reason: str, pose: Pose2D | None, now: float) -> Command:
        if self.state not in (State.DONE, State.ABORT):
            return self._abort(reason, pose, now)
        return Command()

    def _finish_segment(self, now: float) -> Command:
        self.stats.segments[-1].duration_sec = now - self.segment_started_at
        self.state = State.SETTLE
        self.state_started_at = now
        return Command()

    def tick(
        self, now: float, pose: Pose2D | None, last_odom_time: float | None,
    ) -> Command:
        if self.state in (State.DONE, State.ABORT):
            return Command()
        if pose is None or last_odom_time is None:
            return Command()
        if now - last_odom_time > self.config.odom_timeout_sec:
            self.stats.odom_timeout = True
            return self._abort('ODOM_TIMEOUT', pose, now)
        if self.stats.mission_start is not None and (
            now - self.mission_started_at > self.config.total_mission_timeout_sec
        ):
            return self._abort('TOTAL_MISSION_TIMEOUT', pose, now)

        if self.state is State.WAIT_ODOM:
            self.stats.mission_start = pose
            self.mission_started_at = now
            self.state = State.START_SEGMENT
            return Command()

        if self.state is State.START_SEGMENT:
            segment = self.segments[self.segment_index]
            self.segment_start = pose
            self.segment_started_at = now
            self.state_started_at = now
            if isinstance(segment, Drive):
                self.stats.segments.append(SegmentResult('drive', segment.distance_m))
                self.state = State.EXECUTE_DRIVE
            else:
                self.stats.segments.append(SegmentResult('turn', segment.angle_deg))
                self.previous_yaw = pose.yaw
                self.accumulated_turn = 0.0
                self.state = State.EXECUTE_TURN
            return Command()

        if self.state is State.EXECUTE_DRIVE:
            segment = self.segments[self.segment_index]
            assert isinstance(segment, Drive) and self.segment_start is not None
            elapsed = now - self.segment_started_at
            timeout = (segment.distance_m / self.config.linear_speed_mps
                       * self.config.timeout_scale + self.config.timeout_margin_sec)
            result = self.stats.segments[-1]
            result.progress = project_along_track(self.segment_start, pose)
            cross = project_cross_track(self.segment_start, pose)
            result.max_abs_cross_track = max(result.max_abs_cross_track, abs(cross))
            if abs(cross) > self.config.max_cross_track_error_m:
                self.stats.cross_track_abort = True
                return self._abort('CROSS_TRACK_LIMIT', pose, now)
            if elapsed > timeout:
                self.stats.segment_timeout = True
                return self._abort('DRIVE_SEGMENT_TIMEOUT', pose, now)
            if result.progress >= segment.distance_m - self.config.position_tolerance_m:
                return self._finish_segment(now)
            return Command(linear_x=self.config.linear_speed_mps)

        if self.state is State.EXECUTE_TURN:
            segment = self.segments[self.segment_index]
            assert isinstance(segment, Turn)
            elapsed = now - self.segment_started_at
            target = math.radians(segment.angle_deg)
            timeout = (target / self.config.angular_speed_radps
                       * self.config.timeout_scale + self.config.timeout_margin_sec)
            delta = normalize_angle(pose.yaw - self.previous_yaw)
            self.accumulated_turn += delta
            self.previous_yaw = pose.yaw
            self.stats.segments[-1].progress = math.degrees(self.accumulated_turn)
            if elapsed > timeout:
                self.stats.segment_timeout = True
                return self._abort('TURN_SEGMENT_TIMEOUT', pose, now)
            if self.accumulated_turn >= target - math.radians(
                    self.config.angle_tolerance_deg):
                return self._finish_segment(now)
            return Command(angular_z=self.config.angular_speed_radps)

        if self.state is State.SETTLE:
            if now - self.state_started_at >= self.config.settle_time_sec:
                self.segment_index += 1
                if self.segment_index < len(self.segments):
                    self.state = State.START_SEGMENT
                else:
                    self.state = State.FINAL_STOP
                    self.state_started_at = now
            return Command()

        if self.state is State.FINAL_STOP:
            if now - self.state_started_at >= self.config.final_stop_duration_sec:
                self.state = State.DONE
                self.stats.success = True
                self.stats.reason = 'COMPLETED'
                self.stats.mission_final = pose
                self.stats.total_duration_sec = now - self.mission_started_at
                self.report_due = True
            return Command()

        return Command()
