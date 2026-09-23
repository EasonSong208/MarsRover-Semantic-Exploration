"""ROS-independent forward kinematics for audited fixed camera poses."""

from dataclasses import dataclass
import math


RADIANS_PER_PULSE = (240.0 / 360.0) * (2.0 * math.pi) / 1000.0
INIT_PULSES = (500, 765, 15, 150)
HORIZONTAL_PULSES = (500, 750, 0, 375)
POSE_PULSES = {
    'vendor_init': INIT_PULSES,
    'vendor_horizontal': HORIZONTAL_PULSES,
}
BASE_FOOTPRINT_TO_BASE_LINK_Z = 0.116091082157675


@dataclass(frozen=True)
class Transform:
    """A translation and fixed-axis roll/pitch/yaw rotation."""

    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


def pulse_to_urdf_angle(pulse: int) -> float:
    """Convert a flipped, 500-centred vendor servo pulse to URDF radians."""
    return (500 - pulse) * RADIANS_PER_PULSE


def pose_joint_angles(pose_name: str) -> tuple[float, float, float, float]:
    """Return joint1..joint4 URDF angles for a named audited pose."""
    try:
        pulses = POSE_PULSES[pose_name]
    except KeyError as error:
        raise ValueError(f'unknown fixed camera pose: {pose_name}') from error
    return tuple(pulse_to_urdf_angle(pulse) for pulse in pulses)


def init_joint_angles() -> tuple[float, float, float, float]:
    """Return joint1..joint4 URDF angles for init.d6a."""
    return pose_joint_angles('vendor_init')


def horizontal_joint_angles() -> tuple[float, float, float, float]:
    """Return joint1..joint4 URDF angles for horizontal.d6a."""
    return pose_joint_angles('vendor_horizontal')


def fixed_joint_transforms(pose_name: str) -> tuple[Transform, ...]:
    """Return the four revolute joint edges frozen at the selected pose."""
    joint1, joint2, joint3, joint4 = pose_joint_angles(pose_name)
    return (
        Transform((0.0251328065010765, 0.0, 0.0774026880954513),
                  (0.0, 0.0, -joint1)),
        Transform((0.0, 0.0, 0.0338648012164686),
                  (0.0, joint2, 0.0)),
        Transform((0.0, 0.0, 0.129416446394797),
                  (0.0, joint3, 0.0)),
        Transform((0.0, 0.0, 0.129444631186569),
                  (0.0, joint4, 0.0)),
    )


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


def _clean(value: float, tolerance: float = 1e-14) -> float:
    return 0.0 if abs(value) < tolerance else value


def _rpy_from_matrix(matrix) -> tuple[float, float, float]:
    """Extract fixed-axis XYZ RPY from a homogeneous transform matrix."""
    pitch = math.atan2(
        -matrix[2][0], math.hypot(matrix[0][0], matrix[1][0]))
    if abs(math.cos(pitch)) > 1e-12:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = math.atan2(-matrix[1][2], matrix[1][1])
        yaw = 0.0
    return tuple(_clean(value) for value in (roll, pitch, yaw))


def _transform_from_matrix(matrix) -> Transform:
    return Transform(
        tuple(_clean(matrix[index][3]) for index in range(3)),
        _rpy_from_matrix(matrix),
    )


def quaternion_from_rpy(
        rpy: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    """Return a normalized quaternion in ROS x,y,z,w order."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    quaternion = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
    norm = math.sqrt(sum(value * value for value in quaternion))
    return tuple(_clean(value / norm) for value in quaternion)


def base_to_depth_cam_link(pose_name: str = 'vendor_init') -> Transform:
    """Compute base_link -> depth_cam_link from the full vendor URDF chain."""
    chain = fixed_joint_transforms(pose_name) + (
        Transform((-0.0507060266977644, 0.0, 0.0505384841187764),
                  (0.0, 0.0, -math.pi / 2.0)),
        Transform((0.0, 0.0, 0.014475),
                  (math.pi, -math.pi / 2.0, -math.pi / 2.0)),
    )
    result = _matrix(Transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
    for transform in chain:
        result = _multiply(result, _matrix(transform))
    return _transform_from_matrix(result)


def base_footprint_to_depth_cam_link(
        pose_name: str = 'vendor_init',
) -> Transform:
    """Compute base_footprint -> depth_cam_link using the fixed base height."""
    base_to_camera = base_to_depth_cam_link(pose_name)
    result = _multiply(
        _matrix(Transform(
            (0.0, 0.0, BASE_FOOTPRINT_TO_BASE_LINK_Z),
            (0.0, 0.0, 0.0))),
        _matrix(base_to_camera),
    )
    return _transform_from_matrix(result)
