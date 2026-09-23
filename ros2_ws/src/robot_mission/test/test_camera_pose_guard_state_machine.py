"""Pure tests for camera-pose command, settling, feedback and retry policy."""

from robot_mission.camera_pose_guard_state_machine import (
    CameraPoseGuardRunner, GuardState, PoseGuardConfig,
)


def command_success(runner, now=0.0):
    decision = runner.tick(now, controller_available=True)
    assert decision.send_command
    runner.command_result(now, True)


def test_initial_ready_is_false():
    runner = CameraPoseGuardRunner()
    assert not runner.ready
    assert runner.state is GuardState.WAIT_FOR_CONTROLLER


def test_successful_command_and_settle_reaches_ready():
    runner = CameraPoseGuardRunner(PoseGuardConfig(settle_time_sec=2.0))
    command_success(runner)
    assert not runner.tick(1.9, controller_available=True).ready
    decision = runner.tick(2.0, controller_available=True)
    assert decision.ready
    assert decision.state is GuardState.READY


def test_missing_controller_waits_without_becoming_terminal():
    runner = CameraPoseGuardRunner()
    for now in (0.0, 10.0, 1000.0):
        decision = runner.tick(now, controller_available=False)
        assert decision.state is GuardState.WAIT_FOR_CONTROLLER
        assert not decision.ready and not decision.send_command


def test_default_single_command_failure_is_terminal_fault():
    runner = CameraPoseGuardRunner(PoseGuardConfig(
        retry_period_sec=2.0, max_pose_command_attempts=1))
    assert runner.tick(0.0, controller_available=True).send_command
    runner.command_result(0.0, False)
    assert runner.state is GuardState.FAULT and not runner.ready
    for now in (2.0, 20.0, 200.0):
        decision = runner.tick(now, controller_available=True)
        assert decision.state is GuardState.FAULT
        assert not decision.send_command


def test_configured_second_publish_failure_retries_before_fault_only():
    runner = CameraPoseGuardRunner(PoseGuardConfig(
        retry_period_sec=2.0, max_pose_command_attempts=2))
    assert runner.tick(0.0, controller_available=True).send_command
    runner.command_result(0.0, False)
    assert runner.state is GuardState.COMMAND_POSE
    assert not runner.tick(1.9, controller_available=True).send_command
    assert runner.tick(2.0, controller_available=True).send_command
    runner.command_result(2.0, False)
    assert runner.state is GuardState.FAULT
    assert not runner.tick(100.0, controller_available=True).send_command


def test_feedback_unavailable_can_degrade_then_use_time_ready():
    config = PoseGuardConfig(
        settle_time_sec=2.0, enable_feedback_check=True,
        allow_time_based_ready=True, feedback_timeout_sec=1.0)
    runner = CameraPoseGuardRunner(config)
    command_success(runner)
    decision = runner.tick(
        3.0, controller_available=True, feedback_failed=True)
    assert decision.state is GuardState.DEGRADED and not decision.ready
    decision = runner.tick(3.1, controller_available=True)
    assert decision.state is GuardState.READY and decision.ready
    assert decision.reason == 'time_based_ready_after_degraded'


def test_feedback_confirmation_checks_all_configured_servos():
    config = PoseGuardConfig(
        settle_time_sec=1.0, enable_feedback_check=True,
        allow_time_based_ready=False, position_tolerance=10)
    runner = CameraPoseGuardRunner(config)
    command_success(runner)
    positions = {1: 500, 2: 760, 3: 20, 4: 150}
    assert runner.tick(
        1.0, controller_available=True,
        feedback_positions=positions).ready


def test_pose_feedback_is_not_requested_before_settle_completes():
    config = PoseGuardConfig(
        settle_time_sec=2.0, enable_feedback_check=True,
        allow_time_based_ready=False)
    runner = CameraPoseGuardRunner(config)
    command_success(runner)
    assert not runner.tick(
        1.9, controller_available=True).request_feedback
    assert runner.tick(
        2.0, controller_available=True).request_feedback


def test_feedback_out_of_tolerance_faults_ready_false():
    config = PoseGuardConfig(
        settle_time_sec=1.0, enable_feedback_check=True,
        allow_time_based_ready=False, position_tolerance=10)
    runner = CameraPoseGuardRunner(config)
    command_success(runner)
    decision = runner.tick(
        1.0, controller_available=True,
        feedback_positions={1: 500, 2: 700, 3: 15, 4: 150})
    assert decision.state is GuardState.FAULT and not decision.ready
    for now in (3.0, 30.0, 300.0):
        decision = runner.tick(now, controller_available=True)
        assert decision.state is GuardState.FAULT
        assert not decision.send_command


def test_ready_does_not_resend_by_default():
    runner = CameraPoseGuardRunner()
    command_success(runner)
    assert runner.tick(2.0, controller_available=True).ready
    for now in (10.0, 100.0):
        decision = runner.tick(now, controller_available=True)
        assert decision.ready and not decision.send_command


def test_ready_drops_when_graph_becomes_unavailable():
    runner = CameraPoseGuardRunner()
    command_success(runner)
    assert runner.tick(2.0, controller_available=True).ready
    decision = runner.tick(2.1, controller_available=False)
    assert decision.state is GuardState.DEGRADED
    assert not decision.ready and not runner.ready
