"""Verify the documented fixed-camera FK independently of ROS."""

import math
from pathlib import Path

import pytest
import yaml

from robot_mission.fixed_camera_tf import (
    HORIZONTAL_PULSES, base_to_depth_cam_link, horizontal_joint_angles,
)


CONFIG = Path(__file__).parents[1] / 'config' / 'fixed_camera_extrinsics.yaml'


def test_horizontal_action_pulses_convert_to_expected_urdf_angles():
    assert HORIZONTAL_PULSES == (500, 750, 0, 375)
    assert horizontal_joint_angles() == pytest.approx((
        0.0, -math.pi / 3.0, 2.0 * math.pi / 3.0, math.pi / 6.0,
    ))


def test_fk_matches_versioned_extrinsics():
    data = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    transform = base_to_depth_cam_link()
    assert data['pose_name'] == 'vendor_horizontal'
    assert data['parent_frame'] == 'base_link'
    assert data['child_frame'] == 'depth_cam_link'
    assert transform.xyz == pytest.approx((data['x'], data['y'], data['z']))
    assert transform.rpy == pytest.approx(
        (data['roll'], data['pitch'], data['yaw']))
    assert data['quaternion'] == {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}


def test_static_joint_chain_has_unique_children_and_no_direct_duplicate_edge():
    data = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    joints = data['fixed_joints']
    children = [joint['child_frame'] for joint in joints]
    assert children == ['link1', 'link2', 'link3', 'link4']
    assert len(children) == len(set(children))
    assert data['child_frame'] not in children
    assert 'Valid only' in data['notes']
