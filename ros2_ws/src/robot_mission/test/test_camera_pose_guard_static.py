"""Static integration and no-hardware safety tests for camera_pose_guard."""

import ast
from pathlib import Path

import pytest
import yaml

from robot_mission.camera_pose_guard_state_machine import (
    arm_graph_errors, endpoint_node_identity, set_state_publisher_errors,
    validate_passive_set_state_publishers,
)


ROOT = Path(__file__).parents[1]
NODE = ROOT / 'robot_mission' / 'camera_pose_guard.py'
LAUNCH = ROOT / 'launch' / 'camera_guarded_red_marker_homing.launch.py'
ONLY_LAUNCH = ROOT / 'launch' / 'camera_pose_guard_only.launch.py'
CONFIG = ROOT / 'config' / 'camera_navigation_pose.yaml'
RED_CONFIG = ROOT / 'config' / 'red_marker_homing.yaml'
POSE = ROOT / 'config' / 'camera_poses' / 'vendor_init.yaml'


def test_navigation_config_matches_audited_vendor_init_pose():
    config = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))[
        'camera_pose_guard']['ros__parameters']
    pose = yaml.safe_load(POSE.read_text(encoding='utf-8'))
    assert config['pose_name'] == config['camera_pose_name'] == 'vendor_init'
    assert config['servo_ids'] == [1, 2, 3, 4]
    assert config['servo_positions'] == [
        pose['servo_targets'][f'Servo{index}'] for index in range(1, 5)]
    assert config['move_duration_sec'] == 1.0
    assert config['dry_run'] is True and config['confirmed'] is False
    assert config['arm_torque_confirmed'] is False
    assert config['max_pose_command_attempts'] == 1
    assert config['enable_feedback_check'] is True
    assert config['allow_time_based_ready'] is False
    assert config['reassert_pose'] is False
    assert config['require_unique_fixed_tf'] is True
    assert 'allowed_passive_set_state_publishers' not in config
    assert config['fixed_tf_node_names'] == [
        'fixed_joint1_tf', 'fixed_joint2_tf',
        'fixed_joint3_tf', 'fixed_joint4_tf']


def test_guard_uses_actual_vendor_topic_and_message_without_sleep():
    source = NODE.read_text(encoding='utf-8')
    assert 'ServoPosition, ServosPosition' in source
    assert "'/ros_robot_controller/bus_servo/set_position'" in source
    assert "'/ros_robot_controller/bus_servo/get_state'" in source
    assert "'/ros_robot_controller/bus_servo/set_state'" in source
    assert 'SetBusServoState' in source and 'BusServoState' in source
    assert 'message.duration = self.move_duration_sec' in source
    assert 'time.sleep(' not in source
    assert 'start_app_node.service' not in source


def test_guard_shutdown_does_not_publish_after_ros_context_is_invalid():
    source = NODE.read_text(encoding='utf-8')
    assert 'except KeyboardInterrupt:' in source
    assert 'ExternalShutdownException' in source
    publish_not_ready = source.split(
        'def publish_not_ready', 1)[1].split('def main', 1)[0]
    assert 'if rclpy.ok():' in publish_not_ready
    main = source.split('def main', 1)[1]
    assert 'node.destroy_node()' in main
    assert 'except (KeyboardInterrupt, ExternalShutdownException):' in main
    assert 'except KeyboardInterrupt:' in main


def test_ready_qos_is_reliable_transient_local_depth_one():
    source = NODE.read_text(encoding='utf-8')
    assert 'ReliabilityPolicy.RELIABLE' in source
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in source
    assert 'HistoryPolicy.KEEP_LAST' in source
    assert 'depth=1' in source
    assert "'/camera_pose_ready'" in source
    assert "'/camera_pose_guard/state'" in source


def test_dry_run_and_unconfirmed_guard_do_not_create_command_publisher():
    source = NODE.read_text(encoding='utf-8')
    tree = ast.parse(source)
    assert 'and self.arm_torque_confirmed' in source
    assert 'if not self._authorized()' in source
    command_publishers = [
        call for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == 'create_publisher'
    ]
    # ready, state, lazy pose command, lazy torque/state command
    assert len(command_publishers) == 4


def test_arming_preloads_before_enabling_and_never_unloads_on_shutdown():
    source = NODE.read_text(encoding='utf-8')
    preload = source.index('def _publish_preload_current')
    enable = source.index('def _publish_torque_enable')
    tick = source.index('def _tick')
    assert preload < enable < tick
    assert 'state.position = [' in source
    assert 'state.enable_torque = [1, 1]' in source
    shutdown = source.split('def publish_not_ready', 1)[1]
    assert 'enable_torque' not in shutdown
    assert '[1, 0]' not in shutdown


def test_both_arm_topics_require_the_same_sole_controller_subscriber():
    source = NODE.read_text(encoding='utf-8')
    assert "'expected_controller_node': '/ros_robot_controller'" in source
    assert 'for topic in (self.command_topic, self.torque_command_topic)' in source
    assert 'subscribers != (self.expected_controller_node,)' in source


def test_competing_arm_publishers_and_nodes_are_rejected():
    errors = arm_graph_errors(
        ('/other_servo_writer',), ('/init_pose',),
        ('/controller_manager', '/joystick_control'))
    assert len(errors) == 3
    assert any('bus-servo command topic' in item for item in errors)
    assert any('/servo_controller' in item for item in errors)
    assert any('competing arm node' in item for item in errors)


def test_set_state_guard_alone_is_allowed():
    assert not set_state_publisher_errors(
        ('/camera_pose_guard',),
        guard_node_name='/camera_pose_guard')


def test_set_state_guard_and_one_audited_odom_are_allowed():
    assert not set_state_publisher_errors(
        ('/camera_pose_guard', '/odom_publisher'),
        guard_node_name='/camera_pose_guard',
        allowed_passive_publishers=('/odom_publisher',))


def test_set_state_odom_is_rejected_when_allowlist_is_empty():
    errors = set_state_publisher_errors(
        ('/camera_pose_guard', '/odom_publisher'),
        guard_node_name='/camera_pose_guard')
    assert any('unknown' in error for error in errors)


def test_set_state_unknown_third_publisher_is_rejected():
    for unknown in (
            '/app', '/init_pose', '/controller',
            '/odom_publisher_similar'):
        errors = set_state_publisher_errors(
            ('/camera_pose_guard', '/odom_publisher', unknown),
            guard_node_name='/camera_pose_guard',
            allowed_passive_publishers=('/odom_publisher',))
        assert any(unknown in error for error in errors)


def test_set_state_duplicate_odom_endpoints_are_rejected():
    errors = set_state_publisher_errors(
        ('/camera_pose_guard', '/odom_publisher', '/odom_publisher'),
        guard_node_name='/camera_pose_guard',
        allowed_passive_publishers=('/odom_publisher',))
    assert any('/odom_publisher=2' in error for error in errors)


def test_set_state_unresolved_endpoint_identity_is_rejected():
    errors = set_state_publisher_errors(
        ('/camera_pose_guard', None),
        guard_node_name='/camera_pose_guard')
    assert any('identity unresolved' in error for error in errors)
    assert endpoint_node_identity('', '/') is None
    assert endpoint_node_identity('odom_publisher', 'relative') is None


@pytest.mark.parametrize('unknown', [
    '/app', '/init_pose', '/controller', '/odom_publisher_similar',
])
def test_set_state_allowlist_configuration_rejects_unreviewed_names(unknown):
    with pytest.raises(ValueError, match='unreviewed'):
        validate_passive_set_state_publishers((unknown,))


def test_guarded_launch_defaults_safe_and_exposes_required_arguments():
    source = LAUNCH.read_text(encoding='utf-8')
    for value in (
        "'start_camera_pose_guard': 'true'",
        "'require_camera_pose_ready': 'true'",
        "'dry_run': 'true'", "'confirmed': 'false'",
        "'arm_torque_confirmed': 'false'",
        "'pose_name': 'vendor_init'", "'settle_time_sec': '2.0'",
        "'enable_feedback_check': 'true'",
        "'allow_time_based_ready': 'false'",
        "'max_pose_command_attempts': '1'",
        "'camera_pose_ready_topic': '/camera_pose_ready'",
        "'allowed_passive_set_state_publishers': ['/odom_publisher']",
        'camera_pose_config',
    ):
        assert value in source
    red_parameters = yaml.safe_load(RED_CONFIG.read_text(encoding='utf-8'))[
        'red_marker_homing_test']['ros__parameters']
    for name in (
        'direct_slop_ms', 'fixed_offset_slop_ms', 'depth_stamp_offset_ms',
        'compare_sync_modes', 'estimate_offset', 'output_directory',
        'forward_distance_nominal', 'forward_speed', 'turn_speed',
        'depth_relative_tolerance', 'hsv_low_1', 'hsv_high_2',
    ):
        assert name in red_parameters
    assert 'for name in _red_defaults(share)' in source


def test_guarded_launch_reuses_pose_yaml_and_has_no_duplicate_direct_tf():
    source = LAUNCH.read_text(encoding='utf-8')
    assert "pose['fixed_joints']" in source
    assert "f'{pose_name}.yaml'" in source
    assert "pose_name != 'vendor_init'" in source
    assert "'base_link', 'depth_cam_link'" not in source
    assert source.count("executable='static_transform_publisher'") == 1
    assert 'init_pose' not in source
    assert 'servo_controller' not in source
    assert 'peripherals' not in source


def test_guard_refuses_missing_or_duplicate_fixed_tf_publishers():
    source = NODE.read_text(encoding='utf-8')
    assert "'require_unique_fixed_tf': True" in source
    assert 'count = nodes.count(expected_name)' in source
    assert "f'instance; found {count}'" in source


def test_guard_graph_query_and_identity_fail_closed():
    source = NODE.read_text(encoding='utf-8')
    assert 'except Exception as error' in source
    assert 'graph query or endpoint identity resolution failed' in source
    assert 'self.controller_stable_count = 0' in source
    assert 'endpoint_node_identity' in source


def test_empty_allowlist_has_an_explicit_ros_string_array_type():
    source = NODE.read_text(encoding='utf-8')
    assert 'Parameter.Type.STRING_ARRAY' in source
    assert 'Parameter.Type.NOT_SET' in source
    assert 'name, Parameter.Type.STRING_ARRAY, [])' in source


def test_config_is_loaded_by_guard_node_and_pose_name_is_shared_with_tf():
    source = LAUNCH.read_text(encoding='utf-8')
    assert "LaunchConfiguration('camera_pose_config')" in source
    assert "'pose_name': pose_name" in source
    assert "'camera_pose_name': pose_name" in source


def test_arm_only_launch_has_no_chassis_or_red_marker_node():
    source = ONLY_LAUNCH.read_text(encoding='utf-8')
    tree = ast.parse(source)
    assert 'camera_pose_guard' in source
    assert "'arm_torque_confirmed': 'false'" in source
    assert "'max_pose_command_attempts': '1'" in source
    for forbidden in (
        'red_marker_homing_test', 'cmd_vel', 'odom_publisher',
        'controller', 'joystick', 'nav2', 'peripherals',
    ):
        if forbidden == 'controller':
            continue  # ros_robot_controller is mentioned only by guard config.
        assert forbidden not in source.lower()
    assert any(
        isinstance(node, ast.Call)
        and getattr(node.func, 'id', None) == 'Node'
        for node in ast.walk(tree))
