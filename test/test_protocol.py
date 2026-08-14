import math

import pytest

from pico_6dof_robot_driver.protocol import FirmwareProtocol


def test_parses_firmware_telemetry_and_converts_to_radians():
    protocol = FirmwareProtocol()
    state = protocol.parse_telemetry(
        '{"robot_data":{"config":[0,90,-90,180,-180,45],"j1_homed":true}}'
    )
    assert state is not None
    assert state.positions_rad == pytest.approx((0, math.pi / 2, -math.pi / 2, math.pi, -math.pi, math.pi / 4))
    assert state.raw["j1_homed"] is True


@pytest.mark.parametrize("line", ["debug output", "{bad json}", '{"robot_data":{"config":[1,2]}}'])
def test_ignores_non_telemetry_lines(line):
    assert FirmwareProtocol().parse_telemetry(line) is None


def test_uses_atomic_velocity_configuration_when_it_fits():
    command = FirmwareProtocol().configuration_command([0.0] * 6, math.radians(10.0))
    assert command == "VEL_CONFIG(0,0,0,0,0,0,10)"


def test_nonzero_configuration_is_one_atomic_command():
    positions = [math.radians(value) for value in (10, -20, 30, -40, 50, -60)]
    command = FirmwareProtocol().configuration_command(positions, math.radians(12))
    assert command == "VEL_CONFIG(10,-20,30,-40,50,-60,12)"
    assert len(command.encode("ascii")) <= 126


def test_gripper_normalized_mapping():
    protocol = FirmwareProtocol()
    assert protocol.gripper_command(0.0) == "GRIP_SET(10)"
    assert protocol.gripper_command(1.0) == "GRIP_SET(170)"
    assert protocol.gripper_command(0.5) == "GRIP_SET(90)"
    with pytest.raises(ValueError):
        protocol.gripper_command(1.1)
