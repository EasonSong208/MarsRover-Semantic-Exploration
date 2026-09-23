"""Pure tests for the explicit bus-servo torque arming sequence."""

from robot_mission.arm_torque_arming import (
    ArmTorqueArmingRunner, ArmTorqueConfig, ArmTorqueState,
)


POSITIONS = {1: 506, 2: 398, 3: 253, 4: 152}


def advance_to_preload(runner):
    assert runner.tick(
        0.0, controller_available=True).request_positions
    decision = runner.tick(
        0.1, controller_available=True, positions=POSITIONS)
    assert decision.preload_current
    assert runner.current_positions == POSITIONS


def advance_to_verify(runner):
    advance_to_preload(runner)
    runner.preload_result(0.1, True)
    decision = runner.tick(0.6, controller_available=True)
    assert decision.enable_torque
    runner.enable_result(0.6, True)
    assert runner.state is ArmTorqueState.VERIFY_TORQUE


def test_arming_requires_controller_before_reading_positions():
    runner = ArmTorqueArmingRunner()
    decision = runner.tick(0.0, controller_available=False)
    assert decision.state is ArmTorqueState.WAIT_FOR_CONTROLLER
    assert not decision.ready
    assert not decision.request_positions


def test_read_preload_enable_verify_order_reaches_ready():
    runner = ArmTorqueArmingRunner(ArmTorqueConfig(
        preload_settle_sec=0.5, torque_settle_sec=0.5))
    advance_to_verify(runner)
    assert runner.tick(
        1.1, controller_available=True).request_positions_and_torque
    decision = runner.tick(
        1.2, controller_available=True, positions=POSITIONS,
        torque_states={1: 1, 2: 1, 3: 1, 4: 1})
    assert decision.ready
    assert decision.state is ArmTorqueState.READY


def test_disabled_torque_after_enable_is_terminal_fault():
    runner = ArmTorqueArmingRunner()
    advance_to_verify(runner)
    decision = runner.tick(
        1.2, controller_available=True, positions=POSITIONS,
        torque_states={1: 1, 2: 0, 3: 1, 4: 1})
    assert decision.state is ArmTorqueState.FAULT
    assert 'torque_not_enabled:2' == decision.reason
    assert not runner.tick(100.0, controller_available=True).ready


def test_position_jump_after_enable_is_terminal_fault():
    runner = ArmTorqueArmingRunner(ArmTorqueConfig(
        position_jump_tolerance=10))
    advance_to_verify(runner)
    jumped = dict(POSITIONS)
    jumped[2] += 11
    decision = runner.tick(
        1.2, controller_available=True, positions=jumped,
        torque_states={1: 1, 2: 1, 3: 1, 4: 1})
    assert decision.state is ArmTorqueState.FAULT
    assert decision.reason == 'position_jump_after_torque_enable:2:11'


def test_feedback_failure_never_enables_torque():
    runner = ArmTorqueArmingRunner()
    runner.tick(0.0, controller_available=True)
    decision = runner.tick(
        0.1, controller_available=True, feedback_failed=True)
    assert decision.state is ArmTorqueState.FAULT
    assert not decision.enable_torque


def test_shutdown_behavior_has_no_unload_transition():
    states = set(ArmTorqueState)
    assert all('DISABLE' not in state.name for state in states)
    assert all('UNLOAD' not in state.name for state in states)
