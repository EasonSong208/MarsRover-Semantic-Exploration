"""Offline AST checks for the audited vendor shutdown control-flow shape."""

import ast


def _motor_zero_calls(source):
    tree = ast.parse(source)
    result = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr != 'set_motor_speed' or not node.args:
            continue
        result.append(ast.unparse(node.args[0]))
    return result


def test_harness_detects_explicit_four_motor_zero():
    source = """
def stop(board):
    board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
"""
    assert _motor_zero_calls(source) == [
        '[[1, 0], [2, 0], [3, 0], [4, 0]]'
    ]


def test_harness_does_not_treat_cleanup_message_as_motor_stop():
    source = """
def main(board):
    try:
        spin()
    finally:
        print('shutdown finish')
"""
    assert _motor_zero_calls(source) == []


def test_unhandled_exception_bypasses_except_keyboard_interrupt():
    events = []

    class FakeBoard:
        def set_motor_speed(self, speeds):
            events.append(speeds)

    board = FakeBoard()
    try:
        try:
            raise RuntimeError('serial failure')
        except KeyboardInterrupt:
            board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
    except RuntimeError:
        pass
    assert events == []
