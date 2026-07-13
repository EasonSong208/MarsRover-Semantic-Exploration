"""Offline unit and state-machine tests for the fixed turn step."""

import pytest

from robot_mission.turn_step_policy import (
    ANGULAR_SPEED_RAD_S,
    CLEANUP_MIN_DURATION_SEC,
    CLEANUP_MIN_MESSAGES,
    COMMAND_INTEGRATION_SEC,
    NONZERO_FIRST_LAST_SPAN_SEC,
    NONZERO_MESSAGE_COUNT,
    PREZERO_MIN_DURATION_SEC,
    PREZERO_MIN_MESSAGES,
    PUBLISH_PERIOD_SEC,
    PUBLISH_RATE_HZ,
    THEORETICAL_ANGLE_DEG,
    THEORETICAL_ANGLE_RAD,
    TurnStepRunner,
    TwistSpec,
    direction_to_angular_z,
    reject_safety_overrides,
    theoretical_values,
    turn_command,
    validate_twist,
)


class FakeAdapter:
    """Deterministic monotonic clock and ROS-free graph/publish boundary."""

    def __init__(
        self, graph_results=((), ()), subscriber_ready=True,
        discovery_ready=True,
    ):
        self.now = 0.0
        self.graph_results = list(graph_results)
        self.graph_calls = 0
        self.subscriber_ready = subscriber_ready
        self.discovery_ready = discovery_ready
        self.logs = []
        self.commands = []
        self.times = []
        self.runner = None
        self.on_publish = None

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration

    def log(self, message):
        self.logs.append(message)

    def graph_errors(self):
        result = self.graph_results[self.graph_calls]
        self.graph_calls += 1
        return tuple(result)

    def wait_for_graph_discovery(self):
        self.logs.append('GRAPH_DISCOVERY_WAIT_STARTED')
        assert not any(command.angular_z for command in self.commands)
        if self.discovery_ready:
            self.logs.append('GRAPH_DISCOVERY_READY elapsed=0.200')
            return True
        self.logs.append('GRAPH_DISCOVERY_TIMEOUT')
        return False

    def subscriber_count(self):
        return 1 if self.subscriber_ready else 0

    def publish(self, command):
        self.commands.append(command)
        self.times.append(self.now)
        if self.on_publish:
            self.on_publish(command, len(self.commands))


def run_normal(direction='left'):
    adapter = FakeAdapter()
    runner = TurnStepRunner(adapter)
    adapter.runner = runner
    return adapter, runner.run(direction)


def test_direction_mapping_is_fixed():
    assert direction_to_angular_z('left') == ANGULAR_SPEED_RAD_S
    assert direction_to_angular_z('right') == -ANGULAR_SPEED_RAD_S
    with pytest.raises(ValueError):
        direction_to_angular_z('clockwise')


@pytest.mark.parametrize('direction', ['left', 'right'])
def test_turn_has_only_yaw_and_no_linear_or_roll_pitch(direction):
    command = turn_command(direction)
    assert command.linear_x == command.linear_y == command.linear_z == 0.0
    assert command.angular_x == command.angular_y == 0.0
    assert abs(command.angular_z) == ANGULAR_SPEED_RAD_S
    validate_twist(command, allow_nonzero=True)


def test_twist_validator_rejects_combined_or_arbitrary_motion():
    with pytest.raises(ValueError):
        validate_twist(TwistSpec(linear_x=0.01, angular_z=0.20), True)
    with pytest.raises(ValueError):
        validate_twist(TwistSpec(angular_x=0.01), True)
    with pytest.raises(ValueError):
        validate_twist(TwistSpec(angular_z=0.19), True)


def test_theoretical_count_window_angle_and_distinct_timestamp_span():
    values = theoretical_values()
    assert values['message_count'] == NONZERO_MESSAGE_COUNT == 52
    assert values['integration_sec'] == pytest.approx(
        COMMAND_INTEGRATION_SEC)
    assert values['angle_rad'] == pytest.approx(THEORETICAL_ANGLE_RAD)
    assert values['angle_deg'] == pytest.approx(THEORETICAL_ANGLE_DEG)
    assert values['first_last_span_sec'] == pytest.approx(
        NONZERO_FIRST_LAST_SPAN_SEC)
    assert values['first_last_span_sec'] == pytest.approx(2.55)
    assert values['first_last_span_sec'] != values['integration_sec']


@pytest.mark.parametrize('name', [
    'angular_speed', 'angular_z', 'publish_rate_hz',
    'nonzero_message_count', 'duration', 'nonzero_duration',
])
def test_user_cannot_override_fixed_safety_parameters(name):
    with pytest.raises(ValueError):
        reject_safety_overrides({name})


def test_normal_protocol_has_two_checks_prezero_52_turn_and_cleanup():
    adapter, result = run_normal('left')
    assert result.completed
    assert result.graph_checks == (True, True)
    assert adapter.graph_calls == 2
    assert result.prezero.count == PREZERO_MIN_MESSAGES == 21
    assert result.prezero.span_sec >= PREZERO_MIN_DURATION_SEC
    assert result.turn.count == NONZERO_MESSAGE_COUNT
    assert result.turn.span_sec == pytest.approx(NONZERO_FIRST_LAST_SPAN_SEC)
    assert result.cleanup.count >= CLEANUP_MIN_MESSAGES
    assert result.cleanup.span_sec >= CLEANUP_MIN_DURATION_SEC

    nonzero = [command for command in adapter.commands if command.angular_z]
    assert len(nonzero) == 52
    assert {command.angular_z for command in nonzero} == {0.20}
    assert all(command.linear_x == 0.0 for command in adapter.commands)


def test_required_protocol_logs_are_present():
    adapter, _result = run_normal()
    joined = '\n'.join(adapter.logs)
    for marker in (
        'GRAPH_DISCOVERY_WAIT_STARTED', 'GRAPH_DISCOVERY_READY',
        'GRAPH_CHECK_STARTED', 'GRAPH_CHECK_PASSED',
        'WAITING_FOR_SUBSCRIBER', 'SUBSCRIBER_READY',
        'PREZERO_STARTED', 'PREZERO_FINISHED',
        'TURN_STARTED', 'TURN_FINISHED',
        'ZERO_CLEANUP_STARTED', 'ZERO_CLEANUP_FINISHED',
        'TEST_COMPLETED',
    ):
        assert marker in joined


def test_subscriber_timeout_never_enters_nonzero_and_still_cleans():
    adapter = FakeAdapter(graph_results=((),), subscriber_ready=False)
    result = TurnStepRunner(adapter).run('left')
    assert not result.completed
    assert result.reason == 'SUBSCRIBER_TIMEOUT'
    assert result.turn.count == 0
    assert not any(command.angular_z for command in adapter.commands)
    assert result.cleanup.count >= 41
    assert 'SUBSCRIBER_TIMEOUT' in adapter.logs


def test_discovery_timeout_never_checks_graph_or_publishes_nonzero_and_cleans():
    adapter = FakeAdapter(discovery_ready=False)
    result = TurnStepRunner(adapter).run('left')
    assert not result.completed
    assert result.reason == 'GRAPH_DISCOVERY_TIMEOUT'
    assert result.graph_checks == ()
    assert adapter.graph_calls == 0
    assert result.turn.count == 0
    assert not any(command.angular_z for command in adapter.commands)
    assert result.cleanup.count >= CLEANUP_MIN_MESSAGES
    assert result.cleanup.span_sec >= CLEANUP_MIN_DURATION_SEC
    assert 'GRAPH_DISCOVERY_TIMEOUT' in adapter.logs


@pytest.mark.parametrize('graph_results, expected_reason, checks', [
    ((('conflict',),), 'FIRST_GRAPH_CHECK_FAILED', (False,)),
    (((), ('conflict',)), 'SECOND_GRAPH_CHECK_FAILED', (True, False)),
])
def test_either_graph_failure_blocks_nonzero_and_cleans(
        graph_results, expected_reason, checks):
    adapter = FakeAdapter(graph_results=graph_results)
    result = TurnStepRunner(adapter).run('right')
    assert result.reason == expected_reason
    assert result.graph_checks == checks
    assert result.turn.count == 0
    assert not any(command.angular_z for command in adapter.commands)
    assert result.cleanup.span_sec >= 2.0


@pytest.mark.parametrize('exception_type', [RuntimeError, KeyboardInterrupt])
def test_exception_and_keyboard_interrupt_paths_complete_cleanup(exception_type):
    adapter = FakeAdapter()
    runner = TurnStepRunner(adapter)

    def fail_on_first_nonzero(command, _count):
        if command.angular_z:
            adapter.on_publish = None
            raise exception_type('injected')

    adapter.on_publish = fail_on_first_nonzero
    with pytest.raises(exception_type):
        runner.run('left')
    zero_times = [
        timestamp for command, timestamp in zip(adapter.commands, adapter.times)
        if command == TwistSpec()
    ]
    # Includes prezero and cleanup; the final cleanup itself is reported in logs.
    assert 'ZERO_CLEANUP_FINISHED' in '\n'.join(adapter.logs)
    assert zero_times


def test_first_signal_stops_turn_and_second_cleanup_signal_is_ignored():
    adapter = FakeAdapter()
    runner = TurnStepRunner(adapter)
    state = {'nonzero': 0, 'cleanup_signals_sent': False}

    def inject_signals(command, _count):
        if command.angular_z:
            state['nonzero'] += 1
            if state['nonzero'] == 3:
                runner.request_signal(15)
        elif runner.cleanup_active and not state['cleanup_signals_sent']:
            state['cleanup_signals_sent'] = True
            runner.request_signal(2)

    adapter.on_publish = inject_signals
    result = runner.run('left')
    assert not result.completed
    assert result.turn.count == 3
    assert result.turn.span_sec == pytest.approx(0.10)
    assert 1 <= state['nonzero'] < NONZERO_MESSAGE_COUNT
    assert 'SECOND_SIGNAL_IGNORED' in adapter.logs
    assert result.cleanup.count >= 41
    assert result.cleanup.span_sec >= 2.0


def test_fixed_publish_period_is_20_hz():
    assert PUBLISH_RATE_HZ == 20.0
    assert PUBLISH_PERIOD_SEC == pytest.approx(0.05)
