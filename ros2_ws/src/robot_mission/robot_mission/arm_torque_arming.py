"""Pure fail-closed state machine for explicitly arming bus servos."""

from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Mapping, Optional


class ArmTorqueState(Enum):
    WAIT_FOR_CONTROLLER = auto()
    READ_CURRENT = auto()
    PRELOAD_CURRENT = auto()
    ENABLE_TORQUE = auto()
    VERIFY_TORQUE = auto()
    READY = auto()
    FAULT = auto()


@dataclass(frozen=True)
class ArmTorqueConfig:
    servo_ids: tuple[int, ...] = (1, 2, 3, 4)
    feedback_timeout_sec: float = 10.0
    preload_settle_sec: float = 0.5
    torque_settle_sec: float = 0.5
    position_jump_tolerance: int = 10


@dataclass(frozen=True)
class ArmTorqueDecision:
    state: ArmTorqueState
    ready: bool
    request_positions: bool = False
    request_positions_and_torque: bool = False
    preload_current: bool = False
    enable_torque: bool = False
    reason: str = ''


def validate_arm_torque_config(config: ArmTorqueConfig) -> None:
    if not config.servo_ids or len(set(config.servo_ids)) != len(config.servo_ids):
        raise ValueError('servo_ids must be non-empty and unique')
    if not all(1 <= value <= 253 for value in config.servo_ids):
        raise ValueError('servo_ids must be between 1 and 253')
    positive = (
        config.feedback_timeout_sec,
        config.preload_settle_sec,
        config.torque_settle_sec,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        raise ValueError('arming feedback and settle periods must be positive')
    if config.position_jump_tolerance < 0:
        raise ValueError('position_jump_tolerance cannot be negative')


class ArmTorqueArmingRunner:
    """Require read, preload, enable and measured verification in that order."""

    def __init__(self, config: ArmTorqueConfig = ArmTorqueConfig()) -> None:
        validate_arm_torque_config(config)
        self.config = config
        self.state = ArmTorqueState.WAIT_FOR_CONTROLLER
        self.ready = False
        self.reason = 'initializing'
        self.state_started_at: Optional[float] = None
        self.current_positions: Optional[dict[int, int]] = None
        self.action_pending = False

    def _transition(
        self, state: ArmTorqueState, now: float, reason: str,
    ) -> None:
        self.state = state
        self.state_started_at = now
        self.ready = state is ArmTorqueState.READY
        self.reason = reason
        self.action_pending = False

    def _elapsed(self, now: float) -> float:
        origin = self.state_started_at if self.state_started_at is not None else now
        return max(0.0, now - origin)

    def _complete_positions(
        self, positions: Optional[Mapping[int, int]],
    ) -> bool:
        return positions is not None and all(
            servo_id in positions for servo_id in self.config.servo_ids)

    def preload_result(self, now: float, success: bool) -> None:
        if self.state is not ArmTorqueState.PRELOAD_CURRENT:
            raise RuntimeError('preload_result is only valid in PRELOAD_CURRENT')
        if success:
            self.action_pending = False
            self.state_started_at = now
            self.reason = 'current_positions_preloaded'
        else:
            self._transition(ArmTorqueState.FAULT, now, 'preload_failed')

    def enable_result(self, now: float, success: bool) -> None:
        if self.state is not ArmTorqueState.ENABLE_TORQUE:
            raise RuntimeError('enable_result is only valid in ENABLE_TORQUE')
        if success:
            self._transition(
                ArmTorqueState.VERIFY_TORQUE, now, 'torque_enable_published')
        else:
            self._transition(ArmTorqueState.FAULT, now, 'torque_enable_failed')

    def tick(
        self,
        now: float,
        *,
        controller_available: bool,
        positions: Optional[Mapping[int, int]] = None,
        torque_states: Optional[Mapping[int, int]] = None,
        feedback_failed: bool = False,
    ) -> ArmTorqueDecision:
        if self.state_started_at is None:
            self.state_started_at = now

        if self.state is ArmTorqueState.WAIT_FOR_CONTROLLER:
            if controller_available:
                self._transition(
                    ArmTorqueState.READ_CURRENT, now, 'controller_available')

        if self.state is ArmTorqueState.READ_CURRENT:
            if not controller_available:
                self._transition(
                    ArmTorqueState.WAIT_FOR_CONTROLLER, now, 'controller_lost')
            elif feedback_failed:
                self._transition(
                    ArmTorqueState.FAULT, now, 'initial_position_read_failed')
            elif self._complete_positions(positions):
                self.current_positions = {
                    servo_id: int(positions[servo_id])
                    for servo_id in self.config.servo_ids
                }
                self._transition(
                    ArmTorqueState.PRELOAD_CURRENT, now,
                    'initial_positions_captured')
            else:
                return ArmTorqueDecision(
                    self.state, False, request_positions=True,
                    reason='read_current_positions')

        if self.state is ArmTorqueState.PRELOAD_CURRENT:
            if not controller_available:
                self._transition(
                    ArmTorqueState.FAULT, now,
                    'controller_lost_during_arming')
            elif not self.action_pending and self.reason == 'initial_positions_captured':
                self.action_pending = True
                return ArmTorqueDecision(
                    self.state, False, preload_current=True,
                    reason='preload_current_positions')
            elif (not self.action_pending
                  and self.reason == 'current_positions_preloaded'
                  and self._elapsed(now) >= self.config.preload_settle_sec):
                self._transition(
                    ArmTorqueState.ENABLE_TORQUE, now,
                    'preload_ordering_delay_complete')

        if self.state is ArmTorqueState.ENABLE_TORQUE:
            if not controller_available:
                self._transition(
                    ArmTorqueState.FAULT, now,
                    'controller_lost_during_arming')
            elif not self.action_pending:
                self.action_pending = True
                return ArmTorqueDecision(
                    self.state, False, enable_torque=True,
                    reason='enable_torque_once')

        if self.state is ArmTorqueState.VERIFY_TORQUE:
            if not controller_available:
                self._transition(
                    ArmTorqueState.FAULT, now,
                    'controller_lost_during_verification')
            elif feedback_failed:
                self._transition(
                    ArmTorqueState.FAULT, now,
                    'torque_verification_failed')
            elif self._complete_positions(positions) and torque_states is not None:
                missing_torque = [
                    servo_id for servo_id in self.config.servo_ids
                    if torque_states.get(servo_id) != 1
                ]
                if missing_torque:
                    self._transition(
                        ArmTorqueState.FAULT, now,
                        'torque_not_enabled:'
                        + ','.join(str(value) for value in missing_torque))
                else:
                    jumps = {
                        servo_id: abs(
                            int(positions[servo_id])
                            - int(self.current_positions[servo_id]))
                        for servo_id in self.config.servo_ids
                    }
                    excessive = {
                        servo_id: jump for servo_id, jump in jumps.items()
                        if jump > self.config.position_jump_tolerance
                    }
                    if excessive:
                        detail = ','.join(
                            f'{servo_id}:{jump}'
                            for servo_id, jump in sorted(excessive.items()))
                        self._transition(
                            ArmTorqueState.FAULT, now,
                            'position_jump_after_torque_enable:' + detail)
                    else:
                        self._transition(
                            ArmTorqueState.READY, now,
                            'torque_and_position_verified')
            elif self._elapsed(now) >= self.config.torque_settle_sec:
                return ArmTorqueDecision(
                    self.state, False,
                    request_positions_and_torque=True,
                    reason='verify_torque_and_position')

        return ArmTorqueDecision(
            self.state, self.ready, reason=self.reason)
