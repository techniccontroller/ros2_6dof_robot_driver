"""Encoding and decoding for the robot firmware's ASCII serial protocol."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Iterable


JOINT_COUNT = 6
DEG_PER_RAD = 180.0 / math.pi


@dataclass(frozen=True)
class RobotTelemetry:
    """One telemetry record emitted by ``Robot::getRobotDataAsJson``."""

    positions_rad: tuple[float, ...]
    raw: dict[str, Any]


class FirmwareProtocol:
    """Stateless helpers for the newline-delimited firmware protocol."""

    def __init__(self, max_command_bytes: int = 126) -> None:
        # BUFFER_SIZE is 128: reserve one byte for '\n' and one for the NUL.
        if max_command_bytes < 1:
            raise ValueError("max_command_bytes must be positive")
        self.max_command_bytes = max_command_bytes

    @staticmethod
    def _finite(values: Iterable[float], label: str) -> list[float]:
        result = [float(value) for value in values]
        if not all(math.isfinite(value) for value in result):
            raise ValueError(f"{label} must contain only finite values")
        return result

    @staticmethod
    def _number(value: float, precision: int = 4) -> str:
        text = f"{float(value):.{precision}f}".rstrip("0").rstrip(".")
        return "0" if text in {"-0", ""} else text

    @staticmethod
    def encode(command: str) -> bytes:
        if "\n" in command or "\r" in command:
            raise ValueError("command must not contain newline characters")
        return f"{command}\n".encode("ascii")

    def parse_telemetry(self, line: str | bytes) -> RobotTelemetry | None:
        """Return telemetry for a JSON line; ignore firmware debug/ack lines."""
        if isinstance(line, bytes):
            try:
                line = line.decode("ascii")
            except UnicodeDecodeError:
                return None
        text = line.strip()
        if not (text.startswith("{") and text.endswith("}")):
            return None
        try:
            document = json.loads(text)
        except (json.JSONDecodeError, TypeError):
            return None
        data = document.get("robot_data")
        if not isinstance(data, dict):
            return None
        config = data.get("config")
        if not isinstance(config, list) or len(config) != JOINT_COUNT:
            return None
        try:
            degrees = self._finite(config, "robot_data.config")
        except (TypeError, ValueError):
            return None
        return RobotTelemetry(
            positions_rad=tuple(math.radians(value) for value in degrees),
            raw=data,
        )

    def configuration_command(
        self,
        positions_rad: Iterable[float],
        velocity_rad_s: float,
    ) -> str:
        """Create one atomic six-axis ``VEL_CONFIG`` command."""
        positions = self._finite(positions_rad, "positions")
        if len(positions) != JOINT_COUNT:
            raise ValueError(f"positions must contain exactly {JOINT_COUNT} values")
        velocity = float(velocity_rad_s)
        if not math.isfinite(velocity) or velocity <= 0.0:
            raise ValueError("velocity_rad_s must be finite and greater than zero")

        degrees = [value * DEG_PER_RAD for value in positions]
        velocity_deg_s = velocity * DEG_PER_RAD
        compact_values = [self._number(value, 2) for value in degrees]
        compact_velocity = self._number(velocity_deg_s, 2)
        command = f"VEL_CONFIG({','.join(compact_values)},{compact_velocity})"
        if len(command.encode("ascii")) > self.max_command_bytes:
            raise ValueError(
                "target cannot be represented within the firmware command buffer; "
                "increase Communication::BUFFER_SIZE"
            )
        return command

    @staticmethod
    def gripper_command(normalized_position: float, open_deg: float = 10.0, close_deg: float = 170.0) -> str:
        position = float(normalized_position)
        if not math.isfinite(position) or not 0.0 <= position <= 1.0:
            raise ValueError("gripper position must be in [0.0, 1.0]")
        servo_position = open_deg + position * (close_deg - open_deg)
        return f"GRIP_SET({FirmwareProtocol._number(servo_position, 2)})"
