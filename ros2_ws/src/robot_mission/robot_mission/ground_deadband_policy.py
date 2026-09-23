"""Hard limits for one low-speed ground deadband probe."""

import math

from robot_mission.motion_smoke_policy import (
    MIN_ZERO_DURATION,
    PUBLISH_RATE_HZ,
    SmokeParameters,
)


PROBE_LINEAR_X = 0.08
PROBE_NONZERO_DURATION = 0.25


def validate_probe_parameters(parameters: SmokeParameters) -> tuple[str, ...]:
    """Reject any parameter change that broadens the approved probe envelope."""
    errors = []
    if not math.isclose(parameters.linear_x, PROBE_LINEAR_X, abs_tol=1e-12):
        errors.append(f'linear_x must equal {PROBE_LINEAR_X}')
    if not 0.0 < parameters.nonzero_duration <= PROBE_NONZERO_DURATION:
        errors.append(
            f'nonzero_duration must be in (0, {PROBE_NONZERO_DURATION}]')
    if not math.isclose(
            parameters.publish_rate_hz, PUBLISH_RATE_HZ, abs_tol=1e-12):
        errors.append(f'publish_rate_hz must equal {PUBLISH_RATE_HZ}')
    if parameters.zero_duration < MIN_ZERO_DURATION:
        errors.append(f'zero_duration must be at least {MIN_ZERO_DURATION}')
    return tuple(errors)
