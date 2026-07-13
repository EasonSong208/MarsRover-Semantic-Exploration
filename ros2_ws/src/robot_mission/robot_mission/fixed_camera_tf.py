"""ROS-independent forward kinematics for the audited fixed camera pose."""

from dataclasses import dataclass
import math


RADIANS_PER_PULSE = (240.0 / 360.0) * (2.0 * math.pi) / 1000.0
HORIZONTAL_PULSES = (500, 750, 0, 375)


@dataclass(frozen=True)
class Transform:
    """A translation and fixed-axis roll/pitch/yaw rotation."""

    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


def pulse_to_urdf_angle(pulse: int) -> float:
    """Convert a flipped, 500-centred vendor servo pulse to URDF radians."""
    return (500 - pulse) * RADIANS_PER_PULSE


def horizontal_joint_angles() -> tuple[float, float, float, float]:
    """Return joint1..joint4 URDF angles for horizontal.d6a."""
    return tuple(pulse_to_urdf_angle(pulse) for pulse in HORIZONTAL_PULSES)


def _matrix(transform: Transform) -> list[list[float]]:
    x, y, z = transform.xyz
    roll, pitch, yaw = transform.rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
        [-sp, cp * sr, cp * cr, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _multiply(left, right):
    return [
        [sum(left[row][k] * right[k][column] for k in range(4))
         for column in range(4)]
        for row in range(4)
    ]


def base_to_depth_cam_link() -> Transform:
    """Compute base_link -> depth_cam_link from vendor URDF joint origins."""
    joint1, joint2, joint3, joint4 = horizontal_joint_angles()
    chain = (
        Transform((0.0251328065010765, 0.0, 0.0774026880954513),
                  (0.0, 0.0, -joint1)),
        Transform((0.0, 0.0, 0.0338648012164686), (0.0, joint2, 0.0)),
        Transform((0.0, 0.0, 0.129416446394797), (0.0, joint3, 0.0)),
        Transform((0.0, 0.0, 0.129444631186569), (0.0, joint4, 0.0)),
        Transform((-0.0507060266977644, 0.0, 0.0505384841187764),
                  (0.0, 0.0, -math.pi / 2.0)),
        Transform((0.0, 0.0, 0.014475),
                  (math.pi, -math.pi / 2.0, -math.pi / 2.0)),
    )
    result = _matrix(Transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
    for transform in chain:
        result = _multiply(result, _matrix(transform))

    # The horizontal chain simplifies to identity rotation. Keeping the FK
    # calculation here makes the source values independently testable.
    for row in range(3):
        for column in range(3):
            expected = 1.0 if row == column else 0.0
            if not math.isclose(result[row][column], expected, abs_tol=1e-12):
                raise ValueError('horizontal camera rotation is not identity')
    return Transform(
        tuple(result[index][3] for index in range(3)),
        (0.0, 0.0, 0.0),
    )
