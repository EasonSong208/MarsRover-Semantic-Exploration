"""Offline structural tests for the minimal bringup launch file."""

import ast
from pathlib import Path


LAUNCH = (
    Path(__file__).parents[1] / 'launch' / 'minimal_bringup.launch.py'
)
EXPECTED_ARGUMENTS = {
    'enable_lidar': 'true',
    'enable_camera': 'false',
    'enable_imu': 'true',
    'enable_odom': 'true',
    'enable_ekf': 'true',
}


def _tree():
    return ast.parse(LAUNCH.read_text(encoding='utf-8'))


def _constant(call, keyword):
    value = next(item.value for item in call.keywords if item.arg == keyword)
    return value.value if isinstance(value, ast.Constant) else None


def test_launch_is_valid_python_and_has_entry_point():
    tree = _tree()
    functions = {
        node.name for node in tree.body if isinstance(node, ast.FunctionDef)
    }
    assert 'generate_launch_description' in functions


def test_arguments_and_safe_defaults_are_exact():
    found = {}
    for node in ast.walk(_tree()):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name):
            continue
        if node.func.id != 'DeclareLaunchArgument':
            continue
        assert node.args and isinstance(node.args[0], ast.Constant)
        found[node.args[0].value] = _constant(node, 'default_value')
    assert found == EXPECTED_ARGUMENTS


def test_only_recursively_audited_vendor_launches_are_included():
    allowed = {
        'robot_description.launch.py',
        'imu_filter.launch.py',
        'lidar.launch.py',
        'depth_camera.launch.py',
    }
    filenames = {
        node.args[1].value
        for node in ast.walk(_tree())
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == '_vendor_launch'
        and len(node.args) == 2
        and isinstance(node.args[1], ast.Constant)
    }
    assert filenames == allowed


def test_forbidden_nodes_and_launches_are_absent():
    tree = _tree()
    forbidden = {
        'joystick_control',
        'init_pose',
        'servo_controller',
        'start_app_node',
        'nav2_controller',
        'controller_server',
        'planner_server',
        'bt_navigator',
    }
    configured = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != 'Node':
            continue
        for key in ('package', 'executable', 'name'):
            value = _constant(node, key)
            if value:
                configured.add(value)
    assert forbidden.isdisjoint(configured)


def test_launch_has_no_actuation_client_or_publisher_api():
    tree = _tree()
    forbidden_calls = {
        'create_publisher',
        'publish',
        'send_goal',
        'send_goal_async',
        'create_action_client',
    }
    calls = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert forbidden_calls.isdisjoint(calls)

    imports = {
        node.module
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module
    }
    forbidden_imports = ('geometry_msgs', 'nav2_msgs', 'rclpy.action')
    assert not any(name.startswith(forbidden_imports) for name in imports)


def test_velocity_names_are_remaps_only_not_publishers():
    tree = _tree()
    node_executables = {
        _constant(node, 'executable')
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'Node'
    }
    assert 'teleop_key_control' not in node_executables
    assert 'joystick_control' not in node_executables
    assert 'velocity_smoother' not in node_executables


def test_no_servo_or_navigation_command_interface_is_configured():
    tree = _tree()
    string_literals = {
        node.value for node in ast.walk(tree) if isinstance(node, ast.Constant)
        and isinstance(node.value, str)
    }
    forbidden_interfaces = {
        '/servo_controller',
        'ros_robot_controller/bus_servo/set_position',
        'ros_robot_controller/bus_servo/set_state',
        '/navigate_to_pose',
        '/compute_path_to_pose',
    }
    assert forbidden_interfaces.isdisjoint(string_literals)
