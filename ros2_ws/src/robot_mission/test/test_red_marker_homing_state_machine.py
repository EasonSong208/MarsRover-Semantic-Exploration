"""ROS-free direction, depth, timeout, and pure-axis state-machine tests."""

import math
from types import SimpleNamespace

from robot_mission.homing_state_machine import (
    Command, HomingRunner, Observation, State,
    apply_motion_safety_gate, authorized_command, yaw_command,
)
REFERENCE = SimpleNamespace(
    sample_count=60, u=320, v=180, depth=1000, bbox_width=80,
    area=5000, u_mad=1, depth_mad=2,
    yaw_tolerance_px=8, depth_tolerance=30)


def detection(u=320, depth=1000, valid=True, reason='ok'):
    return SimpleNamespace(
        detected=True, valid=valid, reason=reason, u=u, v=180,
        depth=depth, area=5000, bbox=(280, 140, 80, 80),
        valid_depth_count=100 if valid else 1)


def observation(marker=None, rgb_age=0.01, depth_age=0.01, info_age=0.01):
    return Observation(marker or detection(), rgb_age, depth_age, info_age, 358.0)


def runner_in(state):
    runner = HomingRunner()
    runner.set_reference(REFERENCE)
    runner.state = state
    runner.start_time = 0.0
    runner.state_start_time = 0.0
    runner.marker_last_seen = 0.0
    runner.last_tick_time = 0.0
    return runner


def test_marker_right_of_reference_commands_right_turn():
    command = yaw_command(40, 358, 8, 0.8, 0.025, 0.08)
    assert command.angular_z < 0


def test_marker_left_of_reference_commands_left_turn():
    command = yaw_command(-40, 358, 8, 0.8, 0.025, 0.08)
    assert command.angular_z > 0


def test_yaw_inside_pixel_tolerance_is_zero():
    assert yaw_command(8, 358, 8, 0.8, 0.025, 0.08) == Command()


def test_depth_closer_than_reference_requests_backup_pulse():
    runner = runner_in(State.CHECK_DEPTH)
    assert runner.tick(0.1, observation(detection(depth=900))) == Command()
    assert runner.state is State.BACKUP_PULSE
    assert runner.tick(0.2, observation(detection(depth=900))).linear_x < 0


def test_depth_in_reference_range_enters_final_verify():
    runner = runner_in(State.CHECK_DEPTH)
    runner.tick(0.1, observation(detection(depth=1010)))
    assert runner.state is State.FINAL_VERIFY


def test_backup_overshoot_aborts_without_forward_compensation():
    runner = runner_in(State.CHECK_DEPTH)
    command = runner.tick(0.1, observation(detection(depth=1100)))
    assert command == Command()
    assert runner.state is State.OVERSHOOT_ABORT
    assert command.linear_x <= 0.0


def test_marker_lost_timeout_stops_and_aborts():
    runner = runner_in(State.VISUAL_ALIGN_YAW)
    command = runner.tick(
        0.6, Observation(None, 0.01, 0.01, 0.01, 358.0))
    assert command == Command()
    assert runner.state is State.MARKER_LOST_ABORT


def test_rgb_and_depth_timeouts_soft_hold_without_terminal_abort():
    for kwargs in ({'rgb_age': 0.6}, {'depth_age': 0.6}):
        runner = runner_in(State.VISUAL_ALIGN_YAW)
        assert runner.tick(0.1, observation(**kwargs)) == Command()
        assert runner.state is State.VISUAL_ALIGN_YAW
        assert runner.reason.startswith('soft_sensor_hold:')


def test_depth_stale_holds_linear_state_and_recovers():
    runner = runner_in(State.BACKUP_PULSE)
    stale = Observation(
        detection(), 0.01, 2.0, 0.01, 358.0,
        sensor_state='DEPTH_STALE', depth_fresh=False)
    assert runner.tick(2.0, stale) == Command()
    assert runner.state is State.BACKUP_PULSE
    recovered = observation(detection(depth=900))
    assert runner.tick(2.1, recovered).linear_x < 0.0


def test_rgb_only_can_compute_yaw_but_cannot_progress_depth_state():
    runner = runner_in(State.VISUAL_ALIGN_YAW)
    rgb_only = Observation(
        detection(u=360, valid=False, reason='insufficient_valid_depth'),
        0.01, 1.0, 0.01, 358.0,
        sensor_state='RGB_ONLY', depth_fresh=False)
    assert runner.tick(0.1, rgb_only).angular_z < 0.0
    assert runner.state is State.VISUAL_ALIGN_YAW


def test_camera_pose_gate_drop_holds_motion_and_recovers():
    runner = runner_in(State.BACKUP_PULSE)
    assert runner.tick(
        1.0, observation(), motion_gate_ready=False) == Command()
    assert runner.state is State.BACKUP_PULSE
    assert runner.reason == 'soft_motion_hold:camera_pose_not_ready'
    assert runner.tick(
        1.1, observation(), motion_gate_ready=True).linear_x < 0.0


def test_camera_pose_gate_still_computes_visual_yaw_decision():
    runner = runner_in(State.VISUAL_ALIGN_YAW)
    command = runner.tick(
        0.1, observation(detection(u=360)), motion_gate_ready=False)
    assert command.angular_z < 0.0
    assert runner.state is State.VISUAL_ALIGN_YAW


def test_final_safety_gate_immediately_zeros_when_pose_ready_drops():
    common = {
        'dry_run': False, 'confirmed': True, 'graph_safe': True,
        'enable_base_motion': True,
        'require_camera_pose_ready': True, 'sensor_state': 'SYNC_OK',
        'depth_fresh': True,
    }
    command = Command(linear_x=0.05)
    assert apply_motion_safety_gate(
        command, camera_pose_ready=True, **common) == command
    assert apply_motion_safety_gate(
        command, camera_pose_ready=False, **common) == Command()


def test_camera_pose_gate_can_be_disabled_for_backward_compatibility():
    command = Command(angular_z=0.05)
    assert apply_motion_safety_gate(
        command, dry_run=False, confirmed=True, graph_safe=True,
        enable_base_motion=True,
        require_camera_pose_ready=False, camera_pose_ready=False,
        sensor_state='SYNC_OK', depth_fresh=True) == command


def test_base_motion_gate_forces_zero_even_when_other_gates_pass():
    command = Command(linear_x=0.05)
    assert apply_motion_safety_gate(
        command, dry_run=False, confirmed=True, enable_base_motion=False,
        graph_safe=True, require_camera_pose_ready=False,
        camera_pose_ready=True, sensor_state='SYNC_OK',
        depth_fresh=True) == Command()


def test_invalid_depth_soft_holds_active_state():
    runner = runner_in(State.VISUAL_ALIGN_YAW)
    bad = detection(valid=False, reason='insufficient_valid_depth')
    assert runner.tick(0.1, observation(bad)) == Command()
    assert runner.state is State.VISUAL_ALIGN_YAW
    assert runner.reason == 'soft_sensor_hold:SYNC_OK'


def test_dry_run_and_unconfirmed_authorization_are_always_zero():
    nonzero = Command(linear_x=0.05)
    assert authorized_command(nonzero, dry_run=True, confirmed=True) == Command()
    assert authorized_command(nonzero, dry_run=False, confirmed=False) == Command()
    assert authorized_command(nonzero, dry_run=False, confirmed=True) == nonzero


def test_open_loop_and_backup_commands_never_combine_axes():
    commands = [
        Command(linear_x=0.05), Command(angular_z=0.15),
        Command(angular_z=-0.15), Command(linear_x=-0.03), Command(),
    ]
    assert all(not (item.linear_x and item.angular_z) for item in commands)


def test_state_maximum_duration_aborts_to_zero():
    runner = runner_in(State.WAIT_FOR_MARKER)
    command = runner.tick(16.0, observation())
    assert command == Command()
    assert runner.state is State.SAFETY_ABORT
    assert runner.reason == 'state_timeout'


def test_visual_angle_formula_matches_atan_pixel_over_fx():
    expected = -0.8 * math.atan(10 / 358)
    command = yaw_command(10, 358, 8, 0.8, 0.001, 0.08)
    assert math.isclose(command.angular_z, expected)


def test_abort_reaches_two_second_zero_terminal_tail():
    runner = runner_in(State.VISUAL_ALIGN_YAW)
    commands = [runner.tick(
        0.6, Observation(None, 0.01, 0.01, 0.01, 358.0))]
    assert runner.state is State.MARKER_LOST_ABORT
    commands.append(runner.tick(0.7, observation()))
    assert runner.state is State.FINAL_STOP
    commands.append(runner.tick(2.6, observation()))
    assert runner.state is State.FINAL_STOP
    commands.append(runner.tick(2.8, observation()))
    assert runner.state is State.DONE
    assert commands == [Command()] * 4
