"""Static checks for the ground deadband probe's shared executor."""

import ast
from pathlib import Path


SOURCE = (
    Path(__file__).parents[1] / 'robot_mission' / 'ground_deadband_probe.py'
)


def test_probe_uses_shared_executor_and_fixed_policy():
    source = SOURCE.read_text(encoding='utf-8')
    assert 'validate_probe_parameters' in source
    assert 'run_once(GroundDeadbandProbe, args=args)' in source
    assert 'GROUND DEADBAND PROBE PLAN' in source
    assert 'expected ideal displacement: <= 0.020 m' in source


def test_probe_cannot_create_an_independent_publisher():
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    assert not any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'create_publisher'
        for node in ast.walk(tree)
    )
