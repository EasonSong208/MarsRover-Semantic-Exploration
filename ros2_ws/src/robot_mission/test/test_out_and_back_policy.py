"""Offline tests for M1-A geometry, parameters and state machine."""

import math

import pytest

from robot_mission.out_and_back_policy import (
    Command, Drive, MissionConfig, MissionRunner, Pose2D, State, Turn,
    default_segments, normalize_angle, project_along_track,
    project_cross_track, validate_config,
)


def test_normalize_angle_across_wrap_boundaries():
    assert math.degrees(normalize_angle(math.radians(-179 - 179))) == pytest.approx(2.0)
    assert math.degrees(normalize_angle(math.radians(179 - -179))) == pytest.approx(-2.0)


def test_track_projection_at_zero_and_ninety_degrees():
    origin = Pose2D(0.0, 0.0, 0.0)
    assert project_along_track(origin, Pose2D(1.0, 0.0, 0.0)) == pytest.approx(1.0)
    north = Pose2D(0.0, 0.0, math.pi / 2)
    assert project_along_track(north, Pose2D(0.0, 1.0, 0.0)) == pytest.approx(1.0)
    assert project_cross_track(origin, Pose2D(0.0, 0.2, 0.0)) == pytest.approx(0.2)


@pytest.mark.parametrize('changes', [
    {'distance_m': 0.0}, {'linear_speed_mps': 0.0},
    {'angular_speed_radps': -0.1}, {'control_rate_hz': 0.0},
    {'odom_timeout_sec': math.nan}, {'timeout_margin_sec': -1.0},
])
def test_invalid_parameters_are_rejected(changes):
    values = MissionConfig().__dict__ | changes
    assert validate_config(MissionConfig(**values))
    with pytest.raises(ValueError):
        MissionRunner(MissionConfig(**values))


def test_action_sequence_is_drive_turn_drive():
    segments = default_segments(MissionConfig(distance_m=3.0))
    assert segments == (Drive(3.0), Turn(180.0), Drive(3.0))


def test_wait_odom_and_every_transition_command_are_zero():
    runner = MissionRunner(MissionConfig(distance_m=1.0))
    assert runner.tick(0.0, None, None) == Command()
    pose = Pose2D(0.0, 0.0, 0.0)
    assert runner.tick(0.1, pose, 0.1) == Command()
    assert runner.state is State.START_SEGMENT
    assert runner.tick(0.2, pose, 0.2) == Command()
    assert runner.state is State.EXECUTE_DRIVE
    assert runner.tick(0.25, pose, 0.25) == Command(linear_x=0.15)


def test_complete_one_meter_out_turn_and_back():
    config = MissionConfig(distance_m=1.0)
    runner = MissionRunner(config)
    start = Pose2D(0.0, 0.0, 0.0)
    runner.tick(0.0, start, 0.0)
    runner.tick(0.05, start, 0.05)
    assert runner.tick(0.10, start, 0.10).linear_x > 0
    assert runner.tick(0.20, Pose2D(0.96, 0.0, 0.0), 0.20) == Command()
    assert runner.state is State.SETTLE

    runner.tick(1.70, Pose2D(0.96, 0.0, 0.0), 1.70)
    runner.tick(1.75, Pose2D(0.96, 0.0, 0.0), 1.75)
    assert runner.tick(1.80, Pose2D(0.96, 0.0, math.pi), 1.80) == Command()
    assert runner.state is State.SETTLE

    runner.tick(3.30, Pose2D(0.96, 0.0, math.pi), 3.30)
    runner.tick(3.35, Pose2D(0.96, 0.0, math.pi), 3.35)
    runner.tick(3.40, Pose2D(0.96, 0.0, math.pi), 3.40)
    runner.tick(3.45, Pose2D(-0.01, 0.0, math.pi), 3.45)
    assert runner.state is State.SETTLE
    runner.tick(5.00, Pose2D(-0.01, 0.0, math.pi), 5.00)
    assert runner.state is State.FINAL_STOP
    runner.tick(7.1, Pose2D(-0.01, 0.0, math.pi), 7.1)
    assert runner.state is State.DONE
    assert runner.stats.success
    assert [result.kind for result in runner.stats.segments] == [
        'drive', 'turn', 'drive']
    assert runner.tick(8.0, start, 8.0) == Command()


def test_stale_odom_aborts_without_nonzero_command():
    runner = MissionRunner(MissionConfig())
    pose = Pose2D(0.0, 0.0, 0.0)
    runner.tick(0.0, pose, 0.0)
    command = runner.tick(0.6, pose, 0.0)
    assert command == Command()
    assert runner.state is State.ABORT
    assert runner.stats.odom_timeout


def test_cross_track_limit_aborts_drive():
    runner = MissionRunner(MissionConfig(distance_m=1.0))
    start = Pose2D(0.0, 0.0, 0.0)
    runner.tick(0.0, start, 0.0)
    runner.tick(0.1, start, 0.1)
    command = runner.tick(0.2, Pose2D(0.1, 0.76, 0.0), 0.2)
    assert command == Command()
    assert runner.state is State.ABORT
    assert runner.stats.cross_track_abort


def test_drive_segment_timeout_aborts():
    config = MissionConfig(
        distance_m=1.0, linear_speed_mps=1.0,
        timeout_scale=1.0, timeout_margin_sec=0.0,
    )
    runner = MissionRunner(config)
    pose = Pose2D(0.0, 0.0, 0.0)
    runner.tick(0.0, pose, 0.0)
    runner.tick(0.1, pose, 0.1)
    command = runner.tick(1.2, pose, 1.2)
    assert command == Command()
    assert runner.state is State.ABORT
    assert runner.stats.segment_timeout


def test_commands_are_always_pure_drive_pure_turn_or_zero():
    commands = (Command(), Command(linear_x=0.15), Command(angular_z=0.20))
    assert all(not (command.linear_x and command.angular_z) for command in commands)
