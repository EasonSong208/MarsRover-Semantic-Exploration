"""Static safety checks for the ROS-facing fixed turn node."""

import ast
from pathlib import Path


SOURCE = Path(__file__).parents[1] / 'robot_mission' / 'turn_step_test.py'


def _tree():
    return ast.parse(SOURCE.read_text(encoding='utf-8'))


def test_only_confirmation_and_direction_are_declared_parameters():
    declared = set()
    for node in ast.walk(_tree()):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter'
                and node.args
                and isinstance(node.args[0], ast.Constant)):
            declared.add(node.args[0].value)
    assert declared == {'confirmed', 'direction'}


def test_unconfirmed_check_precedes_publisher_creation():
    source = SOURCE.read_text(encoding='utf-8')
    assert source.index('if not node.confirmed()') < source.index(
        'node.create_command_publisher()')


def test_exactly_one_publisher_is_created_for_cmd_vel():
    calls = [
        node for node in ast.walk(_tree())
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'create_publisher'
    ]
    assert len(calls) == 1
    source = SOURCE.read_text(encoding='utf-8')
    assert 'COMMAND_TOPIC' in source


def test_no_servo_arm_or_nav2_action_imports():
    modules = {
        node.module for node in ast.walk(_tree())
        if isinstance(node, ast.ImportFrom) and node.module
    }
    modules.update(
        alias.name for node in ast.walk(_tree())
        if isinstance(node, ast.Import) for alias in node.names
    )
    forbidden = ('nav2_msgs', 'rclpy.action', 'servo_controller', 'moveit')
    assert not any(module.startswith(forbidden) for module in modules)


def test_no_shell_or_process_escape_hatch():
    modules = {
        alias.name for node in ast.walk(_tree())
        if isinstance(node, ast.Import) for alias in node.names
    }
    assert 'subprocess' not in modules
    calls = {
        node.func.attr for node in ast.walk(_tree())
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert 'system' not in calls
    assert 'popen' not in calls


def test_forbidden_motion_topics_are_graph_checks_not_publish_targets():
    source = SOURCE.read_text(encoding='utf-8')
    assert "'/controller/cmd_vel'" in source
    assert "'/cmd_vel_nav'" in source
    assert 'create_publisher(Twist, COMMAND_TOPIC' in source
    assert 'ActionClient' not in source
    assert 'send_goal' not in source


def test_two_graph_checks_and_unified_runner_are_required_by_source():
    policy = (
        Path(__file__).parents[1] / 'robot_mission' / 'turn_step_policy.py'
    ).read_text(encoding='utf-8')
    assert policy.count('self._graph_check()') == 2
    assert 'finally:' in policy
    assert 'cleanup = self._cleanup()' in policy


def test_discovery_uses_node_graph_and_precedes_both_formal_checks():
    source = SOURCE.read_text(encoding='utf-8')
    policy = (
        Path(__file__).parents[1] / 'robot_mission' / 'turn_step_policy.py'
    ).read_text(encoding='utf-8')
    assert 'self.get_node_names_and_namespaces()' in source
    assert 'rclpy.spin_once(self, timeout_sec=0.1)' in source
    assert 'time.monotonic()' in source
    assert policy.index('self.adapter.wait_for_graph_discovery()') < policy.index(
        'first = self._graph_check()')
