"""Static guardrails for the ROS-facing M1-A executable."""

import ast
from pathlib import Path


SOURCE = Path(__file__).parents[1] / 'robot_mission' / 'out_and_back_test.py'


def _tree():
    return ast.parse(SOURCE.read_text(encoding='utf-8'))


def test_exactly_one_publisher_subscriber_and_timer_are_created():
    calls = [
        node.func.attr for node in ast.walk(_tree())
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    ]
    assert calls.count('create_publisher') == 1
    assert calls.count('create_subscription') == 1
    assert calls.count('create_timer') == 1


def test_unconfirmed_path_precedes_ros_entities():
    source = SOURCE.read_text(encoding='utf-8')
    assert source.index('if not node.confirmed()') < source.index('node.start(config)')
    assert 'NOT_CONFIRMED: no publisher, subscriber or timer created' in source


def test_no_shell_nav2_servo_or_other_motion_topic():
    source = SOURCE.read_text(encoding='utf-8')
    modules = {
        alias.name for node in ast.walk(_tree())
        if isinstance(node, ast.Import) for alias in node.names
    }
    assert 'subprocess' not in modules
    assert 'os' not in modules
    for forbidden in (
        'ActionClient', 'send_goal', 'nav2_msgs', 'servo',
        '/controller/cmd_vel', '/cmd_vel_nav',
    ):
        assert forbidden not in source


def test_callbacks_have_no_sleep_or_while_loop():
    methods = {
        node.name: node for node in ast.walk(_tree())
        if isinstance(node, ast.FunctionDef)
    }
    for name in ('_odom_callback', '_tick'):
        subtree = methods[name]
        assert not any(isinstance(node, ast.While) for node in ast.walk(subtree))
        assert not any(
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == 'sleep'
            for node in ast.walk(subtree))


def test_twist_only_sets_linear_x_and_angular_z():
    source = SOURCE.read_text(encoding='utf-8')
    assert 'message.linear.x = command.linear_x' in source
    assert 'message.angular.z = command.angular_z' in source
    assert "CMD_VEL_TOPIC = '/cmd_vel'" not in source  # Defined once in policy.
