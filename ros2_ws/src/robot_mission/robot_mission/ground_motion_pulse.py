"""One hard-limited forward ground pulse for first-motion validation."""

from robot_mission.motion_smoke_policy import (
    PUBLISH_RATE_HZ,
    SAFE_LINEAR_X,
)
from robot_mission.motion_smoke_test import (
    COMMAND_TOPIC,
    MotionSmokeTest,
    run_once,
)


class GroundMotionPulse(MotionSmokeTest):
    """Reuse the proven bounded publisher with an explicit ground-test plan."""

    def __init__(self) -> None:
        super().__init__('ground_motion_pulse')

    def print_plan(self, parameters) -> None:
        """Print the exact ground motion before the confirmation gate."""
        print('GROUND MOTION PULSE PLAN')
        print(f'  topic: {COMMAND_TOPIC}')
        print(f'  expected motion: forward only at {SAFE_LINEAR_X:.3f} m/s')
        print('  all lateral and angular components: 0')
        print(
            f'  nonzero: <= {parameters.nonzero_duration:.3f} s at '
            f'{PUBLISH_RATE_HZ:.1f} Hz')
        print(f'  zero cleanup: >= {parameters.zero_duration:.3f} s')
        print('  requires clear floor and operator control of physical emergency stop')


def main(args=None) -> int:
    """Run exactly one confirmed ground pulse, otherwise print and refuse."""
    return run_once(GroundMotionPulse, args=args)


if __name__ == '__main__':
    raise SystemExit(main())
