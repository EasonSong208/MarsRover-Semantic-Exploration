"""Static safety tests for the project-owned RGB-D RTAB-Map launch."""

import ast
from pathlib import Path


ROOT = Path(__file__).parents[1]
LAUNCH = ROOT / 'launch' / 'rgbd_rtabmap_bringup.launch.py'
PREFLIGHT = ROOT / 'robot_mission' / 'rgbd_rtabmap_preflight.py'


def _tree(path=LAUNCH):
    return ast.parse(path.read_text(encoding='utf-8'))


def _node_values():
    values = []
    for call in ast.walk(_tree()):
        if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == 'Node'):
            continue
        item = {}
        for keyword in call.keywords:
            if keyword.arg in ('package', 'executable', 'name'):
                if isinstance(keyword.value, ast.Constant):
                    item[keyword.arg] = keyword.value.value
        values.append(item)
    return values


def test_launch_declares_real_audited_defaults_and_confirmation_gate():
    source = LAUNCH.read_text(encoding='utf-8')
    required = {
        "'base_frame', default_value='base_footprint'",
        "'odom_frame', default_value='odom'",
        "'map_frame', default_value='map'",
        "'camera_frame', default_value='depth_cam_color_optical_frame'",
        "'rgb_topic', default_value='/depth_cam/rgb/image_raw'",
        "'depth_topic', default_value='/depth_cam/depth/image_raw'",
        "'camera_info_topic', default_value='/depth_cam/rgb/camera_info'",
        "'camera_pose', default_value='vendor_init'",
        "'fixed_pose_confirmed', default_value='false'",
    }
    assert all(value in source for value in required)


def test_minimal_include_forces_camera_on_and_lidar_off():
    source = LAUNCH.read_text(encoding='utf-8')
    assert "'enable_lidar': 'false'" in source
    assert "'enable_camera': 'true'" in source
    assert "'fixed_arm_pose': 'true'" in source
    assert 'lidar.launch.py' not in source
    assert 'scan_to_scan_filter_chain' not in source


def test_only_safe_project_and_rtabmap_nodes_are_declared():
    nodes = _node_values()
    configured = {
        value for item in nodes for value in item.values()
    }
    forbidden = {
        'init_pose', 'joystick_control', 'servo_controller',
        'slam_toolbox', 'teleop_key_control', 'rgbd_odometry',
    }
    assert forbidden.isdisjoint(configured)
    assert {'rgbd_sync', 'rtabmap', 'static_transform_publisher'} <= configured


def test_rtabmap_has_no_database_delete_argument_or_scan_subscription():
    source = LAUNCH.read_text(encoding='utf-8')
    assert "arguments=['-d']" not in source
    assert "'subscribe_scan': False" in source
    assert "'database_path': database_path" in source
    assert 'reset_database' not in source


def test_two_successful_process_gates_are_required_before_rtabmap():
    source = LAUNCH.read_text(encoding='utf-8')
    assert source.count('OnProcessExit(') == 2
    assert "phase='pre-start'" in source
    assert "phase='ready'" in source
    assert 'success_actions=bringup_stage' in source
    assert 'success_actions=ready_stage' in source
    assert 'Shutdown(reason=' in source


def test_preflight_is_observation_only_and_checks_forbidden_graph_paths():
    tree = _tree(PREFLIGHT)
    calls = {
        node.func.attr for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert 'create_publisher' not in calls
    assert 'publish' not in calls
    source = PREFLIGHT.read_text(encoding='utf-8')
    for value in (
        "'init_pose'", "'joystick'", "'lidar'", "'servo'", "'teleop'",
        "'/joint_states'", "'/scan'", "'/scan_raw'",
    ):
        assert value in source
    assert 'can_transform(' in source


def test_launch_loads_exactly_one_known_pose_and_logs_pose_contract():
    source = LAUNCH.read_text(encoding='utf-8')
    assert "KNOWN_CAMERA_POSES = ('vendor_init', 'vendor_horizontal')" in source
    assert "f'{pose_name}.yaml'" in source
    assert "pose['fixed_joints']" in source
    assert 'Selected fixed camera pose:' in source
    assert 'Expected servo targets:' in source
    assert 'Expected joint angles:' in source
    assert 'no real servo feedback closed loop is available' in source
    assert 'stop SLAM immediately if the arm is moved' in source
    assert 'fixed_camera_extrinsics.yaml' not in source


def test_preflight_checks_selected_pose_and_nominal_tf_values():
    source = PREFLIGHT.read_text(encoding='utf-8')
    for value in (
        "'camera_pose'", "'loaded_camera_pose'", "'known_camera_poses'",
        "'expected_camera_xyz'", "'expected_camera_quaternion'",
        "'/fixed_joint1_tf'", "'/fixed_joint2_tf'",
        "'/fixed_joint3_tf'", "'/fixed_joint4_tf'",
        'pose_config_errors()', 'fixed_camera_tf_errors()',
        'lookup_transform(', 'loaded TF translation does not match',
        'loaded TF rotation does not match',
    ):
        assert value in source
