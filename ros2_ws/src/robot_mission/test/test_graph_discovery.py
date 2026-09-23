"""Offline tests for stable DDS graph discovery decisions."""

from robot_mission.graph_discovery import (
    GRAPH_DISCOVERY_STABLE_SNAPSHOTS,
    GRAPH_DISCOVERY_TIMEOUT_SEC,
    StableDiscovery,
    normalize_node_name,
    normalize_snapshot,
    required_nodes_exactly_once,
)


def test_root_namespace_names_are_normalized():
    assert normalize_node_name('ros_robot_controller', '/') == (
        '/ros_robot_controller')
    assert normalize_node_name('odom_publisher', '/') == '/odom_publisher'


def test_non_root_namespace_is_normalized():
    assert normalize_node_name('worker', '/robot/front/') == (
        '/robot/front/worker')
    assert normalize_node_name('/worker/', 'robot/front') == (
        '/robot/front/worker')


def test_progressive_discovery_requires_two_consecutive_ready_snapshots():
    snapshots = (
        (),
        (('ros_robot_controller', '/'),),
        (('ros_robot_controller', '/'), ('odom_publisher', '/')),
        (('ros_robot_controller', '/'), ('odom_publisher', '/')),
    )
    tracker = StableDiscovery()
    results = [tracker.observe(normalize_snapshot(snapshot)) for snapshot in snapshots]
    assert results == [False, False, False, True]
    assert tracker.consecutive_ready == GRAPH_DISCOVERY_STABLE_SNAPSHOTS == 2


def test_missing_snapshot_resets_consecutive_ready_count():
    tracker = StableDiscovery()
    ready = normalize_snapshot((
        ('ros_robot_controller', '/'), ('odom_publisher', '/'),
    ))
    assert not tracker.observe(ready)
    assert not tracker.observe(())
    assert not tracker.observe(ready)
    assert tracker.observe(ready)


def test_duplicate_required_node_is_not_ready():
    nodes = normalize_snapshot((
        ('ros_robot_controller', '/'),
        ('ros_robot_controller', '/'),
        ('odom_publisher', '/'),
    ))
    assert not required_nodes_exactly_once(nodes)
    tracker = StableDiscovery()
    assert not tracker.observe(nodes)
    assert not tracker.observe(nodes)


def test_discovery_timeout_is_fixed():
    assert GRAPH_DISCOVERY_TIMEOUT_SEC == 5.0
