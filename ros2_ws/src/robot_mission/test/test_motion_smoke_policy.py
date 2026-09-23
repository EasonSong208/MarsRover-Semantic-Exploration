"""Offline tests for the hard-limited motion command schedule."""

import pytest

from robot_mission.motion_smoke_policy import (
    MAX_NONZERO_DURATION,
    MIN_ZERO_DURATION,
    PUBLISH_RATE_HZ,
    SAFE_LINEAR_X,
    SmokeParameters,
    build_schedule,
    execute_with_zero_cleanup,
    graph_errors,
    validate_parameters,
)


def test_default_schedule_has_six_nonzero_samples_at_20_hz():
    schedule = build_schedule(SmokeParameters(confirmed=True))
    nonzero = [sample for sample in schedule if sample.linear_x != 0.0]
    assert len(nonzero) == 6
    assert {sample.linear_x for sample in nonzero} == {SAFE_LINEAR_X}
    assert nonzero[-1].time_sec == pytest.approx(0.25)
    assert all(
        right.time_sec - left.time_sec == pytest.approx(1 / PUBLISH_RATE_HZ)
        for left, right in zip(nonzero, nonzero[1:])
    )


def test_zero_tail_spans_at_least_two_seconds_and_is_last():
    schedule = build_schedule(SmokeParameters(confirmed=True))
    zeros = [sample for sample in schedule if sample.linear_x == 0.0]
    assert len(zeros) == 41
    assert zeros[0].time_sec == pytest.approx(MAX_NONZERO_DURATION)
    assert zeros[-1].time_sec - zeros[0].time_sec == pytest.approx(
        MIN_ZERO_DURATION)
    assert schedule[-1].linear_x == 0.0


@pytest.mark.parametrize('parameters', [
    SmokeParameters(linear_x=0.031),
    SmokeParameters(linear_x=-SAFE_LINEAR_X),
    SmokeParameters(nonzero_duration=0.301),
    SmokeParameters(nonzero_duration=0.0),
    SmokeParameters(publish_rate_hz=21.0),
    SmokeParameters(zero_duration=1.99),
])
def test_weakened_or_over_limit_parameters_are_refused(parameters):
    assert validate_parameters(parameters)
    with pytest.raises(ValueError):
        build_schedule(parameters)


def test_unconfirmed_parameters_are_valid_but_not_authorization():
    parameters = SmokeParameters()
    assert not validate_parameters(parameters)
    assert parameters.confirmed is False


def test_graph_requires_no_command_publishers_and_unique_base_nodes():
    assert graph_errors(
        {'/cmd_vel': [], '/controller/cmd_vel': [], '/cmd_vel_nav': []},
        ['ros_robot_controller', 'odom_publisher'],
    ) == ()
    errors = graph_errors(
        {'/cmd_vel': ['/unexpected'], '/controller/cmd_vel': [],
         '/cmd_vel_nav': []},
        ['ros_robot_controller', 'odom_publisher', 'odom_publisher'],
    )
    assert any('/cmd_vel already has publisher' in error for error in errors)
    assert any('odom_publisher must have exactly one' in error for error in errors)


@pytest.mark.parametrize('raised', [None, KeyboardInterrupt, RuntimeError])
def test_all_python_exit_paths_invoke_zero_cleanup(raised):
    events = []

    def nonzero():
        events.append('nonzero')
        if raised is not None:
            raise raised('stop')

    def zeros():
        events.append('zeros')

    if raised is None:
        execute_with_zero_cleanup(nonzero, zeros)
    else:
        with pytest.raises(raised):
            execute_with_zero_cleanup(nonzero, zeros)
    assert events[-1] == 'zeros'
