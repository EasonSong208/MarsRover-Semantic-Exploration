"""Static regression checks for the versioned vendor method-name correction."""

from pathlib import Path


ROOT = Path(__file__).parents[4]
PATCH = ROOT / 'vendor_patches' / 'ros_robot_controller_get_bus_servo_state.patch'


def test_patch_replaces_only_the_two_broken_query_method_names():
    source = PATCH.read_text(encoding='utf-8')
    assert '-                state = self.board.bus_servo_read_voltage(i.id)' in source
    assert '+                state = self.board.bus_servo_read_vin(i.id)' in source
    assert '-                state = self.board.bus_servo_read_torque(i.id)' in source
    assert '+                state = self.board.bus_servo_read_torque_state(i.id)' in source
    added = [line for line in source.splitlines() if line.startswith('+ ')]
    removed = [line for line in source.splitlines() if line.startswith('- ')]
    assert len(added) == 2
    assert len(removed) == 2
