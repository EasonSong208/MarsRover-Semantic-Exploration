"""Static checks for the ground-only alias of the bounded straight pulse."""

import ast
from pathlib import Path


SOURCE = (
    Path(__file__).parents[1] / 'robot_mission' / 'ground_motion_pulse.py'
)


def test_ground_pulse_delegates_to_the_bounded_smoke_implementation():
    source = SOURCE.read_text(encoding='utf-8')
    tree = ast.parse(source)
    imported_names = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
        for alias in node.names
    }
    assert {'MotionSmokeTest', 'run_once'} <= imported_names
    assert 'GROUND MOTION PULSE PLAN' in source
    assert 'forward only' in source
    assert 'run_once(GroundMotionPulse, args=args)' in source


def test_ground_alias_cannot_create_an_independent_publisher():
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    assert not any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'create_publisher'
        for node in ast.walk(tree)
    )
