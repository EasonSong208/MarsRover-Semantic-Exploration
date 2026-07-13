"""Static guardrails for the ROS-facing motion smoke-test node."""

import ast
from pathlib import Path


SOURCE = (
    Path(__file__).parents[1] / 'robot_mission' / 'motion_smoke_test.py'
)


def _tree():
    return ast.parse(SOURCE.read_text(encoding='utf-8'))


def test_only_cmd_vel_is_used_as_a_command_topic():
    strings = {
        node.value for node in ast.walk(_tree())
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    assert '/cmd_vel' in strings
    assert '/controller/cmd_vel' in strings  # Graph conflict check only.
    assert '/cmd_vel_nav' in strings  # Graph conflict check only.
    assert not any('servo' in value.lower() for value in strings)
    assert not any('navigate_to_pose' in value for value in strings)


def test_no_action_client_or_servo_message_imports():
    modules = {
        node.module for node in ast.walk(_tree())
        if isinstance(node, ast.ImportFrom) and node.module
    }
    modules.update(
        alias.name for node in ast.walk(_tree())
        if isinstance(node, ast.Import) for alias in node.names
    )
    assert not any(module.startswith('nav2_msgs') for module in modules)
    assert not any(module.startswith('rclpy.action') for module in modules)
    assert not any('servo' in module.lower() for module in modules)


def test_publisher_is_created_only_in_named_confirmed_path_method():
    tree = _tree()
    methods = {
        node.name: node for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    publisher_calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'create_publisher'
    ]
    assert len(publisher_calls) == 1
    assert any(
        publisher_calls[0] in set(ast.walk(methods[name]))
        for name in ('create_command_publisher',)
    )


def test_main_uses_unified_finally_cleanup_and_signal_handlers():
    source = SOURCE.read_text(encoding='utf-8')
    assert 'execute_with_zero_cleanup' in source
    assert 'signal.SIGINT' in source
    assert 'signal.SIGTERM' in source
    assert 'if not parameters.confirmed' in source
    assert 'node.create_command_publisher()' in source


def test_discovery_precedes_graph_check_and_nonzero_execution():
    source = SOURCE.read_text(encoding='utf-8')
    discovery = source.index('if not node.wait_for_graph_discovery()')
    check = source.index('conflicts = node.check_graph()')
    execute = source.index('execute_with_zero_cleanup(')
    assert discovery < check < execute
    assert 'rclpy.spin_once(self, timeout_sec=0.1)' in source
    assert 'GRAPH_DISCOVERY_TIMEOUT' in source
