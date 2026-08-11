"""Pure-Python motion mismatch detection core.

The ROS node is intentionally kept as an adapter.  This module owns timestamp
alignment, short-window motion increments, confidence gates and the latched
safety state so it can be tested without ROS or robot hardware.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
from typing import Deque, Optional, Sequence, Tuple


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose2D:
    stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ScalarSample:
    stamp: float
    value: float


@dataclass
class DetectorConfig:
    rotation_window: float = 0.35
    rotation_min_expected: float = math.radians(5.0)
    rotation_abs_error: float = math.radians(3.0)
    rotation_rel_error: float = 0.35
    rotation_confirmations: int = 3

    linear_window: float = 0.60
    linear_min_expected: float = 0.04
    linear_abs_error: float = 0.04
    linear_rel_error: float = 0.40
    lateral_abs_error: float = 0.035
    straight_max_expected_yaw: float = math.radians(6.0)
    linear_confirmations: int = 3

    data_timeout: float = 0.50
    maximum_sample_gap: float = 0.15
    buffer_duration: float = 3.0

    tilt_enter: float = math.radians(8.0)
    tilt_exit: float = math.radians(5.0)
    tilt_stop: float = math.radians(15.0)
    stop_on_excessive_tilt: bool = True

    slam_max_expected_yaw_rate: float = 0.35
    slam_max_observed_yaw_rate: float = 0.45
    slam_resume_stable_time: float = 0.80

    reset_max_linear_command: float = 0.03
    reset_max_angular_command: float = 0.08
    reset_max_observed_yaw_rate: float = 0.08

    gyro_bias_learning_rate: float = 0.01
    gyro_bias_update_limit: float = 0.08


class DetectorState(str, Enum):
    DISARMED = "DISARMED"
    ARMED = "ARMED"
    SUSPECT = "SUSPECT"
    TRIPPED = "TRIPPED"
    DEGRADED = "DEGRADED"


@dataclass(frozen=True)
class MotionIncrement:
    dx: float
    dy: float
    dyaw: float

    @property
    def distance(self) -> float:
        return math.hypot(self.dx, self.dy)


@dataclass(frozen=True)
class DetectionResult:
    stamp: float
    state: DetectorState
    reason: str
    stop_requested: bool
    slam_scan_allowed: bool
    tilted: bool
    excessive_tilt: bool
    rotation_healthy: bool
    linear_healthy: bool
    expected_yaw: float = 0.0
    observed_yaw: float = 0.0
    yaw_error: float = 0.0
    expected_forward: float = 0.0
    observed_forward: float = 0.0
    longitudinal_error: float = 0.0
    lateral_error: float = 0.0


def _bracket(samples: Sequence, stamp: float):
    if not samples or stamp < samples[0].stamp or stamp > samples[-1].stamp:
        return None
    for index in range(1, len(samples)):
        before = samples[index - 1]
        after = samples[index]
        if before.stamp <= stamp <= after.stamp:
            return before, after
    if math.isclose(stamp, samples[-1].stamp):
        return samples[-1], samples[-1]
    return None


def _interpolate_pose(samples: Sequence[Pose2D], stamp: float) -> Optional[Pose2D]:
    pair = _bracket(samples, stamp)
    if pair is None:
        return None
    before, after = pair
    if after.stamp == before.stamp:
        return Pose2D(stamp, before.x, before.y, before.yaw)
    ratio = (stamp - before.stamp) / (after.stamp - before.stamp)
    return Pose2D(
        stamp,
        before.x + ratio * (after.x - before.x),
        before.y + ratio * (after.y - before.y),
        normalize_angle(before.yaw + ratio * normalize_angle(after.yaw - before.yaw)),
    )


def _relative_increment(samples: Sequence[Pose2D], start: float, end: float) -> Optional[MotionIncrement]:
    first = _interpolate_pose(samples, start)
    last = _interpolate_pose(samples, end)
    if first is None or last is None:
        return None
    world_dx = last.x - first.x
    world_dy = last.y - first.y
    cosine = math.cos(first.yaw)
    sine = math.sin(first.yaw)
    return MotionIncrement(
        cosine * world_dx + sine * world_dy,
        -sine * world_dx + cosine * world_dy,
        normalize_angle(last.yaw - first.yaw),
    )


def _interpolate_scalar(samples: Sequence[ScalarSample], stamp: float) -> Optional[float]:
    pair = _bracket(samples, stamp)
    if pair is None:
        return None
    before, after = pair
    if after.stamp == before.stamp:
        return before.value
    ratio = (stamp - before.stamp) / (after.stamp - before.stamp)
    return before.value + ratio * (after.value - before.value)


def _integrate_scalar(
    samples: Sequence[ScalarSample],
    start: float,
    end: float,
    bias: float,
    maximum_gap: float,
) -> Optional[float]:
    start_value = _interpolate_scalar(samples, start)
    end_value = _interpolate_scalar(samples, end)
    if start_value is None or end_value is None:
        return None
    points = [ScalarSample(start, start_value)]
    points.extend(sample for sample in samples if start < sample.stamp < end)
    points.append(ScalarSample(end, end_value))
    integral = 0.0
    for before, after in zip(points, points[1:]):
        dt = after.stamp - before.stamp
        if dt <= 0.0 or dt > maximum_gap:
            return None
        integral += 0.5 * ((before.value - bias) + (after.value - bias)) * dt
    return integral


class SlipDetectorCore:
    """Timestamp-aligned rotation and translation mismatch detector."""

    def __init__(self, config: Optional[DetectorConfig] = None):
        self.config = config or DetectorConfig()
        self.odom: Deque[Pose2D] = deque()
        self.rf2o: Deque[Pose2D] = deque()
        self.gyro: Deque[ScalarSample] = deque()
        self.state = DetectorState.DISARMED
        self.reason = "waiting_for_data"
        self.gyro_bias = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.orientation_stamp: Optional[float] = None
        self.tilted = False
        self.command_linear = 0.0
        self.command_angular = 0.0
        self.command_stamp: Optional[float] = None
        self.rotation_evidence = 0
        self.linear_evidence = 0
        self.clean_since: Optional[float] = None
        self.last_result: Optional[DetectionResult] = None

    def _append(self, buffer: Deque, sample) -> None:
        if buffer and sample.stamp <= buffer[-1].stamp:
            # A clock reset or out-of-order replay invalidates increment history.
            if sample.stamp < buffer[-1].stamp:
                buffer.clear()
            else:
                buffer[-1] = sample
                return
        buffer.append(sample)
        cutoff = sample.stamp - self.config.buffer_duration
        while len(buffer) > 2 and buffer[1].stamp < cutoff:
            buffer.popleft()

    def add_odom(self, stamp: float, x: float, y: float, yaw: float) -> None:
        self._append(self.odom, Pose2D(stamp, x, y, normalize_angle(yaw)))

    def add_rf2o(self, stamp: float, x: float, y: float, yaw: float) -> None:
        self._append(self.rf2o, Pose2D(stamp, x, y, normalize_angle(yaw)))

    def add_gyro(self, stamp: float, yaw_rate: float) -> None:
        self._append(self.gyro, ScalarSample(stamp, yaw_rate))
        stationary_command = (
            abs(self.command_linear) <= self.config.reset_max_linear_command
            and abs(self.command_angular) <= self.config.reset_max_angular_command
        )
        if stationary_command and abs(yaw_rate) <= self.config.gyro_bias_update_limit:
            alpha = self.config.gyro_bias_learning_rate
            self.gyro_bias = (1.0 - alpha) * self.gyro_bias + alpha * yaw_rate

    def set_orientation(self, stamp: float, roll: float, pitch: float) -> None:
        self.orientation_stamp = stamp
        self.roll = roll
        self.pitch = pitch
        tilt = max(abs(roll), abs(pitch))
        if self.tilted:
            self.tilted = tilt > self.config.tilt_exit
        else:
            self.tilted = tilt >= self.config.tilt_enter

    def set_command(self, stamp: float, linear_x: float, linear_y: float, angular_z: float) -> None:
        self.command_stamp = stamp
        self.command_linear = math.hypot(linear_x, linear_y)
        self.command_angular = angular_z

    def _fresh(self, latest_stamp: Optional[float], now: float) -> bool:
        if latest_stamp is None:
            return False
        age = now - latest_stamp
        return -1.0e-3 <= age <= self.config.data_timeout

    def _rotation_increment(self, end: float) -> Tuple[Optional[MotionIncrement], Optional[float]]:
        start = end - self.config.rotation_window
        expected = _relative_increment(tuple(self.odom), start, end)
        observed = _integrate_scalar(
            tuple(self.gyro), start, end, self.gyro_bias, self.config.maximum_sample_gap
        )
        return expected, observed

    def _linear_increments(self, end: float) -> Tuple[Optional[MotionIncrement], Optional[MotionIncrement]]:
        start = end - self.config.linear_window
        return (
            _relative_increment(tuple(self.odom), start, end),
            _relative_increment(tuple(self.rf2o), start, end),
        )

    def can_reset(self, now: float) -> Tuple[bool, str]:
        if self.tilted:
            return False, "robot_is_tilted"
        if abs(self.command_linear) > self.config.reset_max_linear_command:
            return False, "linear_command_is_active"
        if abs(self.command_angular) > self.config.reset_max_angular_command:
            return False, "angular_command_is_active"
        if not self.gyro or not self._fresh(self.gyro[-1].stamp, now):
            return False, "gyro_is_stale"
        if abs(self.gyro[-1].value - self.gyro_bias) > self.config.reset_max_observed_yaw_rate:
            return False, "robot_is_rotating"
        return True, "ok"

    def reset(self, now: float) -> Tuple[bool, str]:
        allowed, reason = self.can_reset(now)
        if not allowed:
            return False, reason
        self.state = DetectorState.ARMED
        self.reason = "manual_reset"
        self.rotation_evidence = 0
        self.linear_evidence = 0
        self.clean_since = now
        return True, "reset"

    def evaluate(self, now: float) -> DetectionResult:
        latest_odom = self.odom[-1].stamp if self.odom else None
        latest_gyro = self.gyro[-1].stamp if self.gyro else None
        latest_rf2o = self.rf2o[-1].stamp if self.rf2o else None

        end_candidates = [stamp for stamp in (latest_odom, latest_gyro) if stamp is not None]
        end = min(end_candidates) if len(end_candidates) == 2 else now
        rotation_healthy = self._fresh(latest_odom, now) and self._fresh(latest_gyro, now)
        orientation_healthy = self._fresh(self.orientation_stamp, now)
        linear_healthy = rotation_healthy and self._fresh(latest_rf2o, now)

        expected_yaw = observed_yaw = yaw_error = 0.0
        expected_forward = observed_forward = longitudinal_error = lateral_error = 0.0
        rotation_candidate = False
        linear_candidate = False

        expected_rotation = observed_rotation = None
        if rotation_healthy:
            expected_rotation, observed_rotation = self._rotation_increment(end)
            rotation_healthy = expected_rotation is not None and observed_rotation is not None
        if rotation_healthy:
            expected_yaw = expected_rotation.dyaw
            observed_yaw = observed_rotation
            yaw_error = normalize_angle(expected_yaw - observed_yaw)
            rotation_threshold = max(
                self.config.rotation_abs_error,
                self.config.rotation_rel_error * abs(expected_yaw),
            )
            rotation_candidate = (
                abs(expected_yaw) >= self.config.rotation_min_expected
                and abs(yaw_error) > rotation_threshold
            )

        expected_linear = observed_linear = None
        linear_end_candidates = [stamp for stamp in (latest_odom, latest_rf2o) if stamp is not None]
        linear_end = min(linear_end_candidates) if len(linear_end_candidates) == 2 else now
        if linear_healthy:
            expected_linear, observed_linear = self._linear_increments(linear_end)
            linear_healthy = expected_linear is not None and observed_linear is not None
        if linear_healthy:
            expected_forward = expected_linear.dx
            observed_forward = observed_linear.dx
            longitudinal_error = expected_forward - observed_forward
            lateral_error = expected_linear.dy - observed_linear.dy
            linear_threshold = max(
                self.config.linear_abs_error,
                self.config.linear_rel_error * abs(expected_forward),
            )
            straight = abs(expected_linear.dyaw) <= self.config.straight_max_expected_yaw
            linear_candidate = (
                straight
                and abs(expected_forward) >= self.config.linear_min_expected
                and (
                    abs(longitudinal_error) > linear_threshold
                    or abs(lateral_error) > self.config.lateral_abs_error
                )
            )

        tilt = max(abs(self.roll), abs(self.pitch)) if orientation_healthy else 0.0
        excessive_tilt = orientation_healthy and tilt >= self.config.tilt_stop

        if self.state != DetectorState.TRIPPED:
            self.rotation_evidence = self.rotation_evidence + 1 if rotation_candidate else 0
            self.linear_evidence = self.linear_evidence + 1 if linear_candidate else 0

            if excessive_tilt and self.config.stop_on_excessive_tilt:
                self.state = DetectorState.TRIPPED
                self.reason = "excessive_tilt"
            elif self.rotation_evidence >= self.config.rotation_confirmations:
                self.state = DetectorState.TRIPPED
                self.reason = "rotation_mismatch"
            elif self.linear_evidence >= self.config.linear_confirmations:
                self.state = DetectorState.TRIPPED
                self.reason = "linear_mismatch"
            elif rotation_candidate or linear_candidate:
                self.state = DetectorState.SUSPECT
                self.reason = "rotation_mismatch" if rotation_candidate else "linear_mismatch"
            elif not rotation_healthy or not orientation_healthy:
                self.state = DetectorState.DEGRADED
                self.reason = "required_sensor_unhealthy"
            else:
                self.state = DetectorState.ARMED
                self.reason = "monitoring"

        latest_rate = self.gyro[-1].value - self.gyro_bias if self.gyro else math.inf
        slam_safe_now = (
            self.state == DetectorState.ARMED
            and rotation_healthy
            and orientation_healthy
            and not self.tilted
            and abs(self.command_angular) <= self.config.slam_max_expected_yaw_rate
            and abs(latest_rate) <= self.config.slam_max_observed_yaw_rate
        )
        if slam_safe_now:
            if self.clean_since is None:
                self.clean_since = now
        else:
            self.clean_since = None
        slam_scan_allowed = (
            slam_safe_now
            and self.clean_since is not None
            and now - self.clean_since >= self.config.slam_resume_stable_time
        )

        result = DetectionResult(
            stamp=now,
            state=self.state,
            reason=self.reason,
            # Missing odom/gyro/orientation makes the safety decision unknown.
            # Fail closed while DEGRADED, but do not latch that transient state.
            stop_requested=self.state in (DetectorState.TRIPPED, DetectorState.DEGRADED),
            slam_scan_allowed=slam_scan_allowed,
            tilted=self.tilted,
            excessive_tilt=excessive_tilt,
            rotation_healthy=rotation_healthy,
            linear_healthy=linear_healthy,
            expected_yaw=expected_yaw,
            observed_yaw=observed_yaw,
            yaw_error=yaw_error,
            expected_forward=expected_forward,
            observed_forward=observed_forward,
            longitudinal_error=longitudinal_error,
            lateral_error=lateral_error,
        )
        self.last_result = result
        return result
