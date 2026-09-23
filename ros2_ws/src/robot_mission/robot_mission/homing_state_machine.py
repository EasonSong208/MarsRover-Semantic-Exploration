"""ROS-independent state machine for the red-marker homing experiment."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from .red_marker_detector import ReferenceStats


class State(Enum):
    WAIT_FOR_SENSORS = auto()
    WAIT_FOR_MARKER = auto()
    COLLECT_REFERENCE = auto()
    WAIT_FOR_CONFIRMATION = auto()
    DRIVE_FORWARD_NOMINAL = auto()
    STOP_AFTER_FORWARD = auto()
    TURN_LEFT_NOMINAL = auto()
    STOP_AFTER_LEFT = auto()
    TURN_RIGHT_NOMINAL = auto()
    STOP_AFTER_RIGHT = auto()
    VISUAL_ALIGN_YAW = auto()
    CHECK_DEPTH = auto()
    BACKUP_PULSE = auto()
    STOP_AFTER_BACKUP = auto()
    FINAL_VERIFY = auto()
    SUCCESS = auto()
    OVERSHOOT_ABORT = auto()
    MARKER_LOST_ABORT = auto()
    SENSOR_TIMEOUT_ABORT = auto()
    SAFETY_ABORT = auto()
    FINAL_STOP = auto()
    DONE = auto()


ABORT_STATES = {
    State.OVERSHOOT_ABORT, State.MARKER_LOST_ABORT,
    State.SENSOR_TIMEOUT_ABORT, State.SAFETY_ABORT,
}

MOTION_STATES = {
    State.DRIVE_FORWARD_NOMINAL, State.STOP_AFTER_FORWARD,
    State.TURN_LEFT_NOMINAL, State.STOP_AFTER_LEFT,
    State.TURN_RIGHT_NOMINAL, State.STOP_AFTER_RIGHT,
    State.VISUAL_ALIGN_YAW, State.CHECK_DEPTH, State.BACKUP_PULSE,
    State.STOP_AFTER_BACKUP, State.FINAL_VERIFY,
}


@dataclass(frozen=True)
class Command:
    linear_x: float = 0.0
    angular_z: float = 0.0

    def __post_init__(self):
        if self.linear_x and self.angular_z:
            raise ValueError('linear.x and angular.z cannot both be nonzero')


@dataclass(frozen=True)
class Observation:
    detection: Optional[Any]
    rgb_age: float
    depth_age: float
    camera_info_age: float
    fx: float
    sensor_state: str = 'SYNC_OK'
    depth_fresh: bool = True


@dataclass(frozen=True)
class HomingConfig:
    control_rate_hz: float = 20.0
    forward_distance_nominal: float = 0.25
    forward_speed: float = 0.05
    turn_angle_deg: float = 30.0
    turn_speed: float = 0.15
    yaw_kp: float = 0.8
    yaw_max_speed: float = 0.08
    yaw_min_speed: float = 0.025
    backup_speed: float = -0.03
    backup_pulse_duration: float = 0.35
    backup_settle_duration: float = 0.50
    segment_stop_duration: float = 0.50
    max_backup_pulses: int = 40
    sensor_timeout: float = 0.5
    marker_lost_timeout: float = 0.5
    total_timeout: float = 90.0
    zero_hold_duration: float = 2.0
    reference_collection_duration: float = 2.0
    final_stable_duration: float = 1.0
    state_timeout: float = 15.0


def validate_config(config: HomingConfig) -> None:
    positive = (
        config.control_rate_hz, config.forward_distance_nominal,
        config.forward_speed, config.turn_angle_deg, config.turn_speed,
        config.yaw_kp, config.yaw_max_speed, config.yaw_min_speed,
        config.backup_pulse_duration, config.backup_settle_duration,
        config.segment_stop_duration, config.sensor_timeout,
        config.marker_lost_timeout, config.total_timeout,
        config.zero_hold_duration, config.reference_collection_duration,
        config.final_stable_duration, config.state_timeout,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        raise ValueError('all durations, rates, distances, and gains must be positive')
    if not config.backup_speed < 0.0:
        raise ValueError('backup_speed must be negative')
    if config.yaw_min_speed > config.yaw_max_speed:
        raise ValueError('yaw_min_speed cannot exceed yaw_max_speed')
    if config.max_backup_pulses < 1:
        raise ValueError('max_backup_pulses must be positive')
    if config.control_rate_hz != 20.0:
        raise ValueError('control_rate_hz is fixed at 20 Hz for this experiment')
    if config.forward_distance_nominal > 0.25 or config.forward_speed > 0.05:
        raise ValueError('forward open-loop limits exceed the reviewed envelope')
    if config.turn_angle_deg > 30.0 or config.turn_speed > 0.15:
        raise ValueError('turn open-loop limits exceed the reviewed envelope')
    if config.yaw_max_speed > 0.08:
        raise ValueError('yaw_max_speed exceeds the reviewed envelope')
    if config.backup_speed < -0.03 or config.backup_pulse_duration > 0.35:
        raise ValueError('backup pulse exceeds the reviewed envelope')
    if config.max_backup_pulses > 40 or config.total_timeout > 90.0:
        raise ValueError('mission limits exceed the reviewed envelope')
    if config.zero_hold_duration < 2.0:
        raise ValueError('zero_hold_duration cannot be less than two seconds')
    if config.sensor_timeout > 0.5 or config.marker_lost_timeout > 0.5:
        raise ValueError('sensor and marker timeouts cannot exceed 0.5 seconds')
    if config.segment_stop_duration < 0.5 or config.backup_settle_duration < 0.5:
        raise ValueError('inter-segment zero settles cannot be shortened')
    if config.reference_collection_duration < 2.0:
        raise ValueError('reference collection cannot be shorter than two seconds')
    if config.final_stable_duration < 1.0 or config.state_timeout > 15.0:
        raise ValueError('stability or state timeout weakens the reviewed envelope')


def authorized_command(command: Command, dry_run: bool, confirmed: bool) -> Command:
    """Apply the only authorization boundary for nonzero motion."""
    if dry_run or not confirmed:
        return Command()
    return command


def apply_motion_safety_gate(
    command: Command,
    *,
    dry_run: bool,
    confirmed: bool,
    enable_base_motion: bool,
    graph_safe: bool,
    require_camera_pose_ready: bool,
    camera_pose_ready: bool,
    sensor_state: str,
    depth_fresh: bool,
) -> Command:
    """Single final gate through which every real chassis command passes."""
    if dry_run or not confirmed or not enable_base_motion or not graph_safe:
        return Command()
    if require_camera_pose_ready and not camera_pose_ready:
        return Command()
    if sensor_state in {'NO_RGB', 'DEPTH_STALE', 'SENSOR_SILENCE'}:
        return Command()
    if not depth_fresh and command.linear_x:
        return Command(angular_z=command.angular_z)
    return command


def yaw_command(
    pixel_error: float, fx: float, tolerance_px: float,
    kp: float, minimum: float, maximum: float,
) -> Command:
    """Map image error to chassis yaw; right-of-reference means right turn."""
    if fx <= 0.0 or not math.isfinite(fx):
        raise ValueError('fx must be finite and positive')
    if abs(pixel_error) <= tolerance_px:
        return Command()
    angle_error = math.atan(pixel_error / fx)
    angular_z = max(-maximum, min(maximum, -kp * angle_error))
    if abs(angular_z) < minimum:
        angular_z = math.copysign(minimum, angular_z)
    return Command(angular_z=angular_z)


@dataclass
class HomingRunner:
    config: HomingConfig = field(default_factory=HomingConfig)
    state: State = State.WAIT_FOR_SENSORS
    reference: Optional[ReferenceStats] = None
    reason: str = ''
    backup_pulse_count: int = 0

    def __post_init__(self):
        validate_config(self.config)
        self.start_time: Optional[float] = None
        self.state_start_time: Optional[float] = None
        self.marker_last_seen: Optional[float] = None
        self.stable_start: Optional[float] = None
        self.last_tick_time: Optional[float] = None

    @property
    def terminal(self) -> bool:
        return self.state is State.DONE

    def set_reference(self, reference: ReferenceStats) -> None:
        self.reference = reference

    def transition(self, state: State, now: float, reason: str = '') -> None:
        self.state = state
        self.state_start_time = now
        self.stable_start = None
        if reason:
            self.reason = reason

    def _elapsed(self, now: float) -> float:
        return now - (self.state_start_time if self.state_start_time is not None else now)

    def _abort(self, state: State, now: float, reason: str) -> Command:
        self.transition(state, now, reason)
        return Command()

    def _pause_timers(self, delta: float) -> None:
        """Freeze mission deadlines during a recoverable sensor hold."""
        if delta <= 0.0:
            return
        for name in (
                'start_time', 'state_start_time', 'marker_last_seen',
                'stable_start'):
            value = getattr(self, name)
            if value is not None:
                setattr(self, name, value + delta)

    def _soft_sensor_hold(
        self, now: float, observation: Observation, delta: float,
    ) -> Optional[Command]:
        if self.state in ({State.SUCCESS, State.FINAL_STOP, State.DONE}
                          | ABORT_STATES):
            return None
        no_rgb = observation.sensor_state in {'NO_RGB', 'SENSOR_SILENCE'}
        legacy_timeout = (
            observation.rgb_age > self.config.sensor_timeout
            or observation.camera_info_age > self.config.sensor_timeout)
        depth_unusable = (
            not observation.depth_fresh
            or observation.depth_age > self.config.sensor_timeout)
        detection = observation.detection
        depth_invalid = (
            detection is not None
            and detection.reason == 'insufficient_valid_depth')
        if not (no_rgb or legacy_timeout or depth_unusable or depth_invalid):
            return None
        self._pause_timers(delta)
        self.reason = f'soft_sensor_hold:{observation.sensor_state}'
        if (not no_rgb and observation.rgb_age <= self.config.sensor_timeout
                and observation.camera_info_age <= self.config.sensor_timeout
                and self.state is State.VISUAL_ALIGN_YAW
                and detection is not None and detection.detected):
            return self._alignment_command(observation)
        return Command()

    def _safety_gate(self, now: float, observation: Observation) -> Optional[Command]:
        if self.state in ({State.WAIT_FOR_SENSORS, State.SUCCESS,
                           State.FINAL_STOP, State.DONE} | ABORT_STATES):
            return None
        if observation.fx <= 0.0 or not math.isfinite(observation.fx):
            return Command()
        detection = observation.detection
        if detection is not None and detection.valid:
            self.marker_last_seen = now
        active = self.state not in {
            State.WAIT_FOR_SENSORS, State.WAIT_FOR_MARKER,
            State.COLLECT_REFERENCE, State.WAIT_FOR_CONFIRMATION,
            State.FINAL_STOP, State.DONE,
        }
        if active:
            if detection is not None and detection.detected and not detection.valid:
                reason = ('invalid_depth' if detection.reason ==
                          'insufficient_valid_depth' else 'marker_area_invalid')
                return self._abort(State.SAFETY_ABORT, now, reason)
            if (self.marker_last_seen is None
                    or now - self.marker_last_seen > self.config.marker_lost_timeout):
                return self._abort(State.MARKER_LOST_ABORT, now, 'marker_lost')
        return None

    def tick(
        self, now: float, observation: Observation,
        confirmed: bool = False, graph_safe: bool = True,
        motion_gate_ready: bool = True,
    ) -> Command:
        if self.start_time is None:
            self.start_time = now
            self.state_start_time = now
        delta = (
            0.0 if self.last_tick_time is None
            else max(0.0, now - self.last_tick_time))
        self.last_tick_time = now
        if self.state is State.SUCCESS or self.state in ABORT_STATES:
            self.transition(State.FINAL_STOP, now)
            return Command()
        if not motion_gate_ready and self.state in MOTION_STATES:
            self._pause_timers(delta)
            self.reason = 'soft_motion_hold:camera_pose_not_ready'
            detection = observation.detection
            if (self.state is State.VISUAL_ALIGN_YAW
                    and detection is not None and detection.detected
                    and observation.fx > 0.0):
                return self._alignment_command(observation)
            return Command()
        held = self._soft_sensor_hold(now, observation, delta)
        if held is not None:
            return held
        if self.state not in {State.FINAL_STOP, State.DONE}:
            if now - self.start_time > self.config.total_timeout:
                return self._abort(State.SAFETY_ABORT, now, 'total_timeout')
            if self._elapsed(now) > self.config.state_timeout:
                return self._abort(State.SAFETY_ABORT, now, 'state_timeout')
        if not graph_safe and self.state not in (
                {State.SUCCESS, State.FINAL_STOP, State.DONE} | ABORT_STATES):
            return self._abort(State.SAFETY_ABORT, now, 'command_graph_conflict')

        gated = self._safety_gate(now, observation)
        if gated is not None:
            return gated
        detection = observation.detection

        if self.state is State.WAIT_FOR_SENSORS:
            if (max(observation.rgb_age, observation.depth_age,
                    observation.camera_info_age) <= self.config.sensor_timeout
                    and observation.fx > 0.0):
                self.transition(State.WAIT_FOR_MARKER, now)
        elif self.state is State.WAIT_FOR_MARKER:
            if detection is not None and detection.valid:
                self.transition(State.COLLECT_REFERENCE, now)
        elif self.state is State.COLLECT_REFERENCE:
            if detection is None or not detection.valid:
                self.transition(State.WAIT_FOR_MARKER, now, 'reference_interrupted')
            elif self.reference is not None:
                self.transition(State.WAIT_FOR_CONFIRMATION, now)
        elif self.state is State.WAIT_FOR_CONFIRMATION:
            if confirmed:
                self.transition(State.DRIVE_FORWARD_NOMINAL, now)
        elif self.state is State.DRIVE_FORWARD_NOMINAL:
            duration = self.config.forward_distance_nominal / self.config.forward_speed
            if self._elapsed(now) >= duration:
                self.transition(State.STOP_AFTER_FORWARD, now)
            else:
                return Command(linear_x=self.config.forward_speed)
        elif self.state is State.STOP_AFTER_FORWARD:
            if self._elapsed(now) >= self.config.segment_stop_duration:
                self.transition(State.TURN_LEFT_NOMINAL, now)
        elif self.state is State.TURN_LEFT_NOMINAL:
            duration = math.radians(self.config.turn_angle_deg) / self.config.turn_speed
            if self._elapsed(now) >= duration:
                self.transition(State.STOP_AFTER_LEFT, now)
            else:
                return Command(angular_z=self.config.turn_speed)
        elif self.state is State.STOP_AFTER_LEFT:
            if self._elapsed(now) >= self.config.segment_stop_duration:
                self.transition(State.TURN_RIGHT_NOMINAL, now)
        elif self.state is State.TURN_RIGHT_NOMINAL:
            duration = math.radians(self.config.turn_angle_deg) / self.config.turn_speed
            if self._elapsed(now) >= duration:
                self.transition(State.STOP_AFTER_RIGHT, now)
            else:
                return Command(angular_z=-self.config.turn_speed)
        elif self.state is State.STOP_AFTER_RIGHT:
            if self._elapsed(now) >= self.config.segment_stop_duration:
                self.transition(State.VISUAL_ALIGN_YAW, now)
        elif self.state is State.VISUAL_ALIGN_YAW:
            command = self._alignment_command(observation)
            if command.angular_z:
                self.stable_start = None
                return command
            if self.stable_start is None:
                self.stable_start = now
            if now - self.stable_start >= self.config.final_stable_duration:
                self.transition(State.CHECK_DEPTH, now)
        elif self.state is State.CHECK_DEPTH:
            depth_state = self._depth_state(detection)
            if depth_state == 'near':
                self.transition(State.FINAL_VERIFY, now)
            elif depth_state == 'far':
                self.transition(State.OVERSHOOT_ABORT, now, 'backup_overshoot')
            elif self.backup_pulse_count >= self.config.max_backup_pulses:
                self.transition(State.SAFETY_ABORT, now, 'backup_pulse_limit')
            else:
                self.backup_pulse_count += 1
                self.transition(State.BACKUP_PULSE, now)
        elif self.state is State.BACKUP_PULSE:
            if self._elapsed(now) >= self.config.backup_pulse_duration:
                self.transition(State.STOP_AFTER_BACKUP, now)
            else:
                return Command(linear_x=self.config.backup_speed)
        elif self.state is State.STOP_AFTER_BACKUP:
            if self._elapsed(now) >= self.config.backup_settle_duration:
                self.transition(State.VISUAL_ALIGN_YAW, now)
        elif self.state is State.FINAL_VERIFY:
            command = self._alignment_command(observation)
            if command.angular_z:
                self.transition(State.VISUAL_ALIGN_YAW, now)
                return Command()
            depth_state = self._depth_state(detection)
            if depth_state == 'far':
                self.transition(State.OVERSHOOT_ABORT, now, 'backup_overshoot')
            elif depth_state == 'close':
                if self.backup_pulse_count >= self.config.max_backup_pulses:
                    self.transition(State.SAFETY_ABORT, now, 'backup_pulse_limit')
                else:
                    self.backup_pulse_count += 1
                    self.transition(State.BACKUP_PULSE, now)
            else:
                if self.stable_start is None:
                    self.stable_start = now
                if now - self.stable_start >= self.config.final_stable_duration:
                    self.transition(State.SUCCESS, now, 'visual_reference_restored')
        elif self.state is State.FINAL_STOP:
            if self._elapsed(now) >= self.config.zero_hold_duration:
                self.transition(State.DONE, now)
        return Command()

    def _alignment_command(self, observation: Observation) -> Command:
        if self.reference is None or observation.detection is None:
            return Command()
        return yaw_command(
            observation.detection.u - self.reference.u, observation.fx,
            self.reference.yaw_tolerance_px, self.config.yaw_kp,
            self.config.yaw_min_speed, self.config.yaw_max_speed)

    def _depth_state(self, detection: Optional[Any]) -> str:
        if self.reference is None or detection is None or not detection.valid:
            return 'invalid'
        error = detection.depth - self.reference.depth
        if error > self.reference.depth_tolerance:
            return 'far'
        if error < -self.reference.depth_tolerance:
            return 'close'
        return 'near'
