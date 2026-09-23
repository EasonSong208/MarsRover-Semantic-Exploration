"""Static safety tests for the observation-only PIDNet ROS2 node."""

from pathlib import Path


SOURCE = (
    Path(__file__).parents[1]
    / 'semantic_perception'
    / 'pidnet_semantic_node.py'
).read_text(encoding='utf-8')


def test_pidnet_node_has_no_actuation_interfaces():
    """The semantic node must not contain robot or arm command endpoints."""
    forbidden = (
        'cmd_vel',
        'controller/cmd_vel',
        'set_position',
        'set_motor',
        'ServosPosition',
    )
    assert all(value not in SOURCE for value in forbidden)


def test_pidnet_node_does_not_use_fake_mask_generator():
    """Model failure must not fall back to the Phase 0 fake generator."""
    assert 'generate_fake_mask' not in SOURCE
    assert 'FakeSemanticNode' not in SOURCE
