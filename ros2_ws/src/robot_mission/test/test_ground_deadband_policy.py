"""Offline limits for the fixed low-speed ground deadband probe."""

import pytest

from robot_mission.ground_deadband_policy import (
    PROBE_LINEAR_X,
    PROBE_NONZERO_DURATION,
    validate_probe_parameters,
)
from robot_mission.motion_smoke_policy import SmokeParameters


def probe_parameters(**changes):
    values = {
        'confirmed': True,
        'linear_x': PROBE_LINEAR_X,
        'nonzero_duration': PROBE_NONZERO_DURATION,
        'publish_rate_hz': 20.0,
        'zero_duration': 2.0,
    }
    values.update(changes)
    return SmokeParameters(**values)


def test_exact_probe_envelope_is_valid():
    assert validate_probe_parameters(probe_parameters()) == ()


@pytest.mark.parametrize('changes', [
    {'linear_x': 0.081},
    {'linear_x': -PROBE_LINEAR_X},
    {'nonzero_duration': 0.251},
    {'nonzero_duration': 0.0},
    {'publish_rate_hz': 21.0},
    {'zero_duration': 1.99},
])
def test_any_broader_or_different_envelope_is_refused(changes):
    assert validate_probe_parameters(probe_parameters(**changes))
