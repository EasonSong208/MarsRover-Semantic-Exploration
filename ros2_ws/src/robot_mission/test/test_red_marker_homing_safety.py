"""Static safety and graph-policy checks for the ROS-facing homing node."""

import ast
from pathlib import Path

import yaml

from robot_mission.motion_smoke_policy import graph_errors


ROOT = Path(__file__).parents[1]
SOURCE = ROOT / 'robot_mission' / 'red_marker_homing_node.py'
POLICY = ROOT / 'robot_mission' / 'homing_state_machine.py'
LAUNCH = ROOT / 'launch' / 'red_marker_homing.launch.py'
CONFIG = ROOT / 'config' / 'red_marker_homing.yaml'


def test_competing_cmd_vel_publisher_is_rejected():
    errors = graph_errors(
        {'/cmd_vel': ['/other'], '/controller/cmd_vel': [], '/cmd_vel_nav': []},
        ['ros_robot_controller', 'odom_publisher'])
    assert any('/cmd_vel already has publisher' in item for item in errors)


def test_controller_and_odom_nodes_must_each_be_unique():
    errors = graph_errors(
        {'/cmd_vel': [], '/controller/cmd_vel': [], '/cmd_vel_nav': []},
        ['ros_robot_controller', 'odom_publisher', 'odom_publisher'])
    assert any('odom_publisher must have exactly one' in item for item in errors)


def test_launch_defaults_are_dry_and_unconfirmed_and_only_one_node():
    source = LAUNCH.read_text(encoding='utf-8')
    tree = ast.parse(source)
    node_calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and getattr(node.func, 'id', None) == 'Node']
    assert len(node_calls) == 1
    parameters = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))[
        'red_marker_homing_test']['ros__parameters']
    assert parameters['dry_run'] is True
    assert parameters['confirmed'] is False
    assert parameters['enable_base_motion'] is False
    assert 'argument_defaults = _config_defaults(config_file)' in source


def test_launch_contains_no_hardware_or_motion_capable_package():
    source = LAUNCH.read_text(encoding='utf-8').lower()
    for forbidden in (
        'peripherals', 'ros_robot_controller', 'robot_state_publisher',
        'rtabmap', 'joystick', 'init_pose', 'servo', 'nav2', 'lidar',
    ):
        assert forbidden not in source


def test_node_has_unified_finally_signal_and_two_second_zero_cleanup():
    source = SOURCE.read_text(encoding='utf-8')
    assert 'finally:' in source
    assert 'node.zero_cleanup()' in source
    assert 'signal.SIGINT' in source and 'signal.SIGTERM' in source
    assert 'zero_hold_duration' in source
    assert 'time.sleep(1.0 / self.config.control_rate_hz)' in source


def test_node_does_not_import_launch_action_or_process_modules():
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    modules = {
        alias.name for node in ast.walk(tree) if isinstance(node, ast.Import)
        for alias in node.names
    }
    modules.update(
        node.module for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module)
    assert 'subprocess' not in modules
    assert not any(module.startswith('launch') for module in modules)
    assert not any(module.startswith('rclpy.action') for module in modules)


def test_only_cmd_vel_is_a_publish_target_and_forbidden_topics_are_checks():
    source = SOURCE.read_text(encoding='utf-8')
    assert 'self.create_publisher(Twist, self.cmd_vel_topic, 10)' in source
    assert "'/controller/cmd_vel'" in source
    assert "'/cmd_vel_nav'" in source
    assert 'start_app_node.service' not in source


def test_rgb_depth_use_bounded_custom_matcher_not_latest_pairing():
    source = SOURCE.read_text(encoding='utf-8')
    assert 'RgbDepthSynchronizer' in source
    assert 'ApproximateTimeSynchronizer' not in source
    assert 'sync_queue_size' in source
    assert "self._feed_synchronizers('rgb'" in source
    assert "self._feed_synchronizers('depth'" in source


def test_launch_exposes_both_sync_modes_and_soft_timeout_parameters():
    source = LAUNCH.read_text(encoding='utf-8')
    parameters = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))[
        'red_marker_homing_test']['ros__parameters']
    for name in (
        'sync_mode', 'direct_slop_ms', 'fixed_offset_slop_ms',
        'depth_stamp_offset_ms', 'compare_sync_modes', 'estimate_offset',
        'offset_estimation_samples', 'depth_freshness_limit_ms',
        'sensor_warn_timeout_ms', 'sensor_stop_timeout_ms',
        'sensor_log_throttle_ms', 'sync_stats_period_sec',
        'output_directory',
    ):
        assert name in parameters
    assert parameters['sync_mode'] == 'direct_slop'
    assert parameters['compare_sync_modes'] is True
    assert parameters['estimate_offset'] is True
    assert 'for name, value in parameters.items()' in source


def test_launch_exposes_speed_depth_target_and_color_parameters():
    parameters = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))[
        'red_marker_homing_test']['ros__parameters']
    for name in (
        'forward_distance_nominal', 'forward_speed', 'turn_angle_deg',
        'turn_speed', 'yaw_max_speed', 'backup_speed',
        'depth_relative_tolerance', 'hsv_low_1', 'hsv_high_1',
        'hsv_low_2', 'hsv_high_2', 'min_marker_area', 'max_marker_area',
    ):
        assert name in parameters


def test_sync_diagnostics_are_persisted_in_state_log():
    source = SOURCE.read_text(encoding='utf-8')
    for field in (
        'rgb_depth_stamp_delta', 'detection_age',
        'detection_reason', 'detection_sequence',
    ):
        assert field in source


def test_sync_diagnostic_files_and_exception_flag_are_written_safely():
    source = SOURCE.read_text(encoding='utf-8')
    assert "'sync_samples.csv'" in source
    assert "'sync_summary.json'" in source
    assert "'had_exception': self.had_exception" in source
    assert 'except OSError as error:' in source


def test_stale_depth_filters_linear_motion_without_shutdown():
    source = SOURCE.read_text(encoding='utf-8')
    policy = POLICY.read_text(encoding='utf-8')
    assert 'if not depth_fresh and command.linear_x:' in policy
    assert 'return Command(angular_z=command.angular_z)' in policy
    assert 'rclpy.shutdown()' not in source.split('def _update_sensor_state', 1)[1].split(
        'def _observation', 1)[0]


def test_camera_pose_ready_is_part_of_one_final_safety_gate():
    source = SOURCE.read_text(encoding='utf-8')
    policy = POLICY.read_text(encoding='utf-8')
    assert 'apply_motion_safety_gate(' in source
    assert 'require_camera_pose_ready and not camera_pose_ready' in policy
    assert "'require_camera_pose_ready': True" in source
    assert "'camera_pose_ready_topic': '/camera_pose_ready'" in source
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in source


def test_base_motion_requires_independent_explicit_gate():
    source = SOURCE.read_text(encoding='utf-8')
    policy = POLICY.read_text(encoding='utf-8')
    parameters = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))[
        'red_marker_homing_test']['ros__parameters']
    assert parameters['enable_base_motion'] is False
    assert 'or not self.enable_base_motion' in source
    assert 'not enable_base_motion' in policy
    assert 'enable_base_motion=self.enable_base_motion' in source


def test_ready_true_to_false_callback_applies_immediate_zero():
    source = SOURCE.read_text(encoding='utf-8')
    callback = source.split(
        'def _camera_pose_ready_callback', 1)[1].split(
            'def _camera_pose_gate_ready', 1)[0]
    assert 'was_ready and not self.camera_pose_ready' in callback
    assert 'self._publish_command(Command())' in callback
