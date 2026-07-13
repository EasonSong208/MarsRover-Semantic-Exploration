"""ROS-independent hard limits for the wheels-raised motion smoke test."""

from dataclasses import dataclass
import math
from typing import Callable, Sequence


SAFE_LINEAR_X = 0.03
MAX_NONZERO_DURATION = 0.3
PUBLISH_RATE_HZ = 20.0
MIN_ZERO_DURATION = 2.0


@dataclass(frozen=True)
class SmokeParameters:
    """User-visible parameters constrained by non-negotiable code limits."""

    confirmed: bool = False
    linear_x: float = SAFE_LINEAR_X
    nonzero_duration: float = MAX_NONZERO_DURATION
    publish_rate_hz: float = PUBLISH_RATE_HZ
    zero_duration: float = MIN_ZERO_DURATION


@dataclass(frozen=True)
class CommandSample:
    """One scheduled command expressed without ROS message dependencies."""

    time_sec: float
    linear_x: float


def validate_parameters(parameters: SmokeParameters) -> tuple[str, ...]:
    """Reject every parameter combination that could weaken a hard limit."""
    errors = []
    if not math.isclose(parameters.linear_x, SAFE_LINEAR_X, abs_tol=1e-12):
        errors.append(f'linear_x must equal {SAFE_LINEAR_X}')
    if not 0.0 < parameters.nonzero_duration <= MAX_NONZERO_DURATION:
        errors.append(
            f'nonzero_duration must be in (0, {MAX_NONZERO_DURATION}]')
    if not math.isclose(
            parameters.publish_rate_hz, PUBLISH_RATE_HZ, abs_tol=1e-12):
        errors.append(f'publish_rate_hz must equal {PUBLISH_RATE_HZ}')
    if parameters.zero_duration < MIN_ZERO_DURATION:
        errors.append(f'zero_duration must be at least {MIN_ZERO_DURATION}')
    return tuple(errors)


def build_schedule(parameters: SmokeParameters) -> tuple[CommandSample, ...]:
    """Return 20 Hz nonzero samples followed by a >=2 s zero sequence."""
    errors = validate_parameters(parameters)
    if errors:
        raise ValueError('; '.join(errors))

    period = 1.0 / PUBLISH_RATE_HZ
    nonzero_count = math.ceil(parameters.nonzero_duration * PUBLISH_RATE_HZ)
    nonzero = [
        CommandSample(index * period, SAFE_LINEAR_X)
        for index in range(nonzero_count)
        if index * period < parameters.nonzero_duration
    ]

    zero_start = parameters.nonzero_duration
    # Include both endpoints so the interval between first and last zero command
    # is at least zero_duration, rather than merely emitting N samples in less
    # than that duration.
    zero_count = math.ceil(parameters.zero_duration * PUBLISH_RATE_HZ) + 1
    zeros = [
        CommandSample(zero_start + index * period, 0.0)
        for index in range(zero_count)
    ]
    return tuple(nonzero + zeros)


def graph_errors(
    publishers_by_topic: dict[str, Sequence[str]],
    node_names: Sequence[str],
) -> tuple[str, ...]:
    """Require an uncontested command graph and one base controller chain."""
    errors = []
    for topic in ('/cmd_vel', '/controller/cmd_vel', '/cmd_vel_nav'):
        publishers = tuple(publishers_by_topic.get(topic, ()))
        if publishers:
            errors.append(
                f'{topic} already has publisher(s): {", ".join(publishers)}')
    for required in ('ros_robot_controller', 'odom_publisher'):
        count = sum(name == required for name in node_names)
        if count != 1:
            errors.append(f'{required} must have exactly one instance; found {count}')
    return tuple(errors)


def execute_with_zero_cleanup(
    publish_nonzero: Callable[[], None],
    publish_zeros: Callable[[], None],
) -> None:
    """Run a motion action and attempt zero cleanup on every Python exit path."""
    try:
        publish_nonzero()
    finally:
        publish_zeros()
