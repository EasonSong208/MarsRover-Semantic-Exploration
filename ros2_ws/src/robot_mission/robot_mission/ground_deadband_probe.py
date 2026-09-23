"""One hard-limited ground pulse to distinguish deadband from chain failure."""

from robot_mission.ground_deadband_policy import (
    PROBE_LINEAR_X,
    PROBE_NONZERO_DURATION,
    validate_probe_parameters,
)
from robot_mission.motion_smoke_policy import PUBLISH_RATE_HZ, SmokeParameters
from robot_mission.motion_smoke_test import (
    COMMAND_TOPIC,
    MotionSmokeTest,
    run_once,
)


class GroundDeadbandProbe(MotionSmokeTest):
    """Use the common one-shot executor with a fixed, slightly higher pulse."""

    def __init__(self) -> None:
        super().__init__(
            'ground_deadband_probe',
            linear_x=PROBE_LINEAR_X,
            nonzero_duration=PROBE_NONZERO_DURATION,
        )

    def parameter_errors(self, parameters: SmokeParameters) -> tuple[str, ...]:
        """Apply the ground probe's immutable speed and duration envelope."""
        return validate_probe_parameters(parameters)

    def print_plan(self, parameters: SmokeParameters) -> None:
        """Print the exact motion and diagnostic purpose before authorization."""
        print('GROUND DEADBAND PROBE PLAN')
        print(f'  topic: {COMMAND_TOPIC}')
        print(f'  expected motion: forward only at {PROBE_LINEAR_X:.3f} m/s')
        print('  all lateral and angular components: 0')
        print(
            f'  nonzero: <= {parameters.nonzero_duration:.3f} s at '
            f'{PUBLISH_RATE_HZ:.1f} Hz')
        print(f'  zero cleanup: >= {parameters.zero_duration:.3f} s')
        print('  expected ideal displacement: <= 0.020 m')
        print('  requires clear floor and operator control of physical emergency stop')


def main(args=None) -> int:
    """Run one confirmed probe, otherwise print the plan and refuse."""
    return run_once(GroundDeadbandProbe, args=args)


if __name__ == '__main__':
    raise SystemExit(main())
