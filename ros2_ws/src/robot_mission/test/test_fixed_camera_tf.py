"""Verify both versioned fixed-camera poses independently of ROS."""

import math
from pathlib import Path

import pytest
import yaml

from robot_mission.fixed_camera_tf import (
    BASE_FOOTPRINT_TO_BASE_LINK_Z,
    HORIZONTAL_PULSES,
    INIT_PULSES,
    base_footprint_to_depth_cam_link,
    base_to_depth_cam_link,
    fixed_joint_transforms,
    horizontal_joint_angles,
    init_joint_angles,
    pose_joint_angles,
    quaternion_from_rpy,
)


CONFIG_DIR = Path(__file__).parents[1] / 'config' / 'camera_poses'
EXPECTED = {
    'vendor_init': {
        'pulses': INIT_PULSES,
        'angles': (0.0, -63.6, 116.4, 84.0),
        'base_xyz': (0.093787390048285, 0.0, 0.234390577130383),
        'footprint_xyz': (0.093787390048285, 0.0, 0.350481659288058),
        'rpy': (0.0, 0.816814089933346, 0.0),
        'quaternion': (0.0, 0.397147890634780, 0.0, 0.917754625683981),
    },
    'vendor_horizontal': {
        'pulses': HORIZONTAL_PULSES,
        'angles': (0.0, -60.0, 120.0, 30.0),
        'base_xyz': (0.090170699365528, 0.0, 0.291404054800367),
        'footprint_xyz': (0.090170699365528, 0.0, 0.407495136958042),
        'rpy': (0.0, 0.0, 0.0),
        'quaternion': (0.0, 0.0, 0.0, 1.0),
    },
}


def _load(pose_name):
    return yaml.safe_load(
        (CONFIG_DIR / f'{pose_name}.yaml').read_text(encoding='utf-8'))


def _quaternion_list(transform):
    return list(quaternion_from_rpy(transform.rpy))


def test_init_action_pulses_convert_to_expected_urdf_angles():
    assert INIT_PULSES == (500, 765, 15, 150)
    assert tuple(math.degrees(value) for value in init_joint_angles()) == pytest.approx(
        (0.0, -63.6, 116.4, 84.0))


def test_horizontal_action_remains_available_but_is_not_default():
    assert HORIZONTAL_PULSES == (500, 750, 0, 375)
    assert tuple(
        math.degrees(value) for value in horizontal_joint_angles()) == pytest.approx(
            (0.0, -60.0, 120.0, 30.0))
    assert _load('vendor_init')['default_for_slam'] is True
    assert _load('vendor_init')['slam_pose_id'] == 'SLAM_POSE_V1'
    assert _load('vendor_horizontal')['default_for_slam'] is False


@pytest.mark.parametrize('pose_name', EXPECTED)
def test_yaml_joint_values_and_full_fk_match_calculation(pose_name):
    data = _load(pose_name)
    expected = EXPECTED[pose_name]
    transform = base_to_depth_cam_link(pose_name)
    configured = data['transforms']['base_link_to_depth_cam_link']

    assert data['pose_name'] == pose_name
    assert tuple(data['servo_targets'][f'Servo{index}'] for index in range(1, 5)) \
        == expected['pulses']
    assert data['joint_angles_deg'] == pytest.approx(expected['angles'])
    assert data['joint_angles_rad'] == pytest.approx(pose_joint_angles(pose_name))
    assert configured['parent_frame'] == 'base_link'
    assert configured['child_frame'] == 'depth_cam_link'
    assert transform.xyz == pytest.approx(expected['base_xyz'])
    assert transform.rpy == pytest.approx(expected['rpy'])
    assert _quaternion_list(transform) == pytest.approx(expected['quaternion'])
    assert configured['xyz'] == pytest.approx(transform.xyz)
    assert configured['rpy'] == pytest.approx(transform.rpy)
    assert [configured['quaternion'][key] for key in ('x', 'y', 'z', 'w')] \
        == pytest.approx(_quaternion_list(transform))


@pytest.mark.parametrize('pose_name', EXPECTED)
def test_quaternion_is_normalized(pose_name):
    data = _load(pose_name)
    quaternion = data['transforms']['base_link_to_depth_cam_link']['quaternion']
    values = [quaternion[key] for key in ('x', 'y', 'z', 'w')]
    assert math.sqrt(sum(value * value for value in values)) == pytest.approx(1.0)


@pytest.mark.parametrize('pose_name', EXPECTED)
def test_base_footprint_transform_has_exact_vendor_base_height(pose_name):
    data = _load(pose_name)
    base_link = base_to_depth_cam_link(pose_name)
    base_footprint = base_footprint_to_depth_cam_link(pose_name)
    configured = data['transforms']['base_footprint_to_depth_cam_link']

    assert configured['xyz'] == pytest.approx(base_footprint.xyz)
    assert configured['rpy'] == pytest.approx(base_footprint.rpy)
    assert base_footprint.xyz[2] - base_link.xyz[2] == pytest.approx(
        BASE_FOOTPRINT_TO_BASE_LINK_Z)
    assert base_footprint.xyz == pytest.approx(
        EXPECTED[pose_name]['footprint_xyz'])
    assert base_footprint.xyz[:2] == pytest.approx(base_link.xyz[:2])
    assert base_footprint.rpy == pytest.approx(base_link.rpy)


@pytest.mark.parametrize('pose_name', EXPECTED)
def test_yaml_static_joint_chain_matches_selected_pose_and_has_unique_children(
        pose_name):
    data = _load(pose_name)
    joints = data['fixed_joints']
    calculated = fixed_joint_transforms(pose_name)
    children = [joint['child_frame'] for joint in joints]

    assert children == ['link1', 'link2', 'link3', 'link4']
    assert len(children) == len(set(children))
    assert 'depth_cam_link' not in children
    for joint, transform in zip(joints, calculated):
        assert joint['xyz'] == pytest.approx(transform.xyz)
        assert joint['rpy'] == pytest.approx(transform.rpy)


def test_unknown_pose_is_rejected():
    with pytest.raises(ValueError, match='unknown fixed camera pose'):
        base_to_depth_cam_link('not_a_pose')
