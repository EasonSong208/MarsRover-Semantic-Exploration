"""Guard against accidentally adding actuation to preflight."""

import ast
from pathlib import Path


PREFLIGHT = Path(__file__).parents[1] / 'robot_mission' / 'preflight.py'


def test_preflight_has_no_actuation_api_calls():
    tree = ast.parse(PREFLIGHT.read_text(encoding='utf-8'))
    forbidden = {
        'create_publisher',
        'create_action_client',
        'publish',
        'send_goal',
        'send_goal_async',
    }
    calls = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert forbidden.isdisjoint(calls)


def test_preflight_does_not_import_motion_messages_or_actions():
    tree = ast.parse(PREFLIGHT.read_text(encoding='utf-8'))
    imported_modules = {
        node.module
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module
    }
    imported_modules.update(
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    )
    forbidden_prefixes = ('geometry_msgs', 'rclpy.action', 'nav2_msgs.action')
    assert not any(
        module.startswith(forbidden_prefixes)
        for module in imported_modules
    )
