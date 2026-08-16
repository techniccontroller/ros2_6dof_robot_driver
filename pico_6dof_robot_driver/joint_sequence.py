"""Serialization and validation for recorded robot joint sequences."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Iterable


FORMAT_NAME = "pico_6dof_joint_sequence"
FORMAT_VERSION = 1
JOINT_COUNT = 6


@dataclass(frozen=True)
class SequenceStep:
    """One arm/gripper target and the delay before the following target."""

    positions_deg: tuple[float, ...]
    hold_seconds: float
    gripper_position: float | None = None

    def __post_init__(self) -> None:
        try:
            positions = tuple(float(value) for value in self.positions_deg)
            hold_seconds = float(self.hold_seconds)
            gripper_position = (
                None
                if self.gripper_position is None
                else float(self.gripper_position)
            )
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "joint positions and hold_seconds must be numbers"
            ) from exc
        if len(positions) != JOINT_COUNT:
            raise ValueError(
                "a sequence step must contain exactly six joint positions"
            )
        if not all(math.isfinite(value) for value in positions):
            raise ValueError("joint positions must be finite numbers")
        if not math.isfinite(hold_seconds) or hold_seconds < 0.0:
            raise ValueError(
                "hold_seconds must be a finite number greater than or equal "
                "to zero"
            )
        if gripper_position is not None and (
            not math.isfinite(gripper_position)
            or not 0.0 <= gripper_position <= 1.0
        ):
            raise ValueError(
                "gripper_position must be between 0 (open) and 1 (closed)"
            )
        object.__setattr__(self, "positions_deg", positions)
        object.__setattr__(self, "hold_seconds", hold_seconds)
        object.__setattr__(self, "gripper_position", gripper_position)

    def to_dict(self) -> dict[str, Any]:
        value = {
            "positions": list(self.positions_deg),
            "hold_seconds": self.hold_seconds,
        }
        if self.gripper_position is not None:
            value["gripper_position"] = self.gripper_position
        return value

    @classmethod
    def from_dict(cls, value: Any) -> "SequenceStep":
        if not isinstance(value, dict):
            raise ValueError("each sequence step must be a JSON object")
        if "positions" not in value or "hold_seconds" not in value:
            raise ValueError(
                "each sequence step needs positions and hold_seconds"
            )
        positions = value["positions"]
        if not isinstance(positions, (list, tuple)):
            raise ValueError("step positions must be a JSON array")
        return cls(
            tuple(positions),
            value["hold_seconds"],
            value.get("gripper_position"),
        )


@dataclass(frozen=True)
class TrajectorySample:
    """One interpolated arm target at an absolute trajectory time."""

    positions_deg: tuple[float, ...]
    time_seconds: float


def interpolate_sequence(
    start_positions_deg: Iterable[float],
    steps: Iterable[SequenceStep],
    speed_deg_s: float,
    sample_rate_hz: float = 10.0,
) -> tuple[list[TrajectorySample], list[float]]:
    """Create a smooth, synchronized arm path and waypoint arrival times."""
    start = tuple(float(value) for value in start_positions_deg)
    speed = float(speed_deg_s)
    sample_rate = float(sample_rate_hz)
    if len(start) != JOINT_COUNT or not all(
        math.isfinite(value) for value in start
    ):
        raise ValueError("start positions must contain six finite numbers")
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError("trajectory speed must be greater than zero")
    if not math.isfinite(sample_rate) or sample_rate <= 0.0:
        raise ValueError("sample rate must be greater than zero")

    samples: list[TrajectorySample] = []
    arrival_times: list[float] = []
    previous = start
    elapsed = 0.0
    minimum_duration = 1.0 / sample_rate

    for step in steps:
        max_delta = max(
            abs(target - current)
            for target, current in zip(step.positions_deg, previous)
        )
        # Smoothstep peaks at 1.5 times its average velocity, so lengthen the
        # segment to keep the requested value as the actual velocity limit.
        move_duration = max(1.5 * max_delta / speed, minimum_duration)
        sample_count = max(1, math.ceil(move_duration * sample_rate))
        for sample_index in range(1, sample_count + 1):
            fraction = sample_index / sample_count
            # Cubic smoothstep gives zero velocity at both ends of a move.
            progress = fraction * fraction * (3.0 - 2.0 * fraction)
            positions = tuple(
                current + (target - current) * progress
                for current, target in zip(previous, step.positions_deg)
            )
            sample_time = elapsed + move_duration * fraction
            samples.append(TrajectorySample(positions, sample_time))

        elapsed += move_duration
        arrival_times.append(elapsed)
        if step.hold_seconds > 0.0:
            elapsed += step.hold_seconds
            samples.append(TrajectorySample(step.positions_deg, elapsed))
        previous = step.positions_deg

    return samples, arrival_times


def sequence_to_dict(steps: Iterable[SequenceStep]) -> dict[str, Any]:
    """Create the stable, human-readable JSON representation."""
    return {
        "format": FORMAT_NAME,
        "version": FORMAT_VERSION,
        "units": "degrees",
        "steps": [step.to_dict() for step in steps],
    }


def sequence_from_dict(value: Any) -> list[SequenceStep]:
    """Validate and decode a sequence JSON object."""
    if not isinstance(value, dict):
        raise ValueError("the sequence file must contain a JSON object")
    if value.get("format") != FORMAT_NAME:
        raise ValueError(
            f"unsupported sequence format; expected {FORMAT_NAME!r}"
        )
    if value.get("version") != FORMAT_VERSION:
        raise ValueError(
            f"unsupported sequence version; expected {FORMAT_VERSION}"
        )
    if value.get("units") != "degrees":
        raise ValueError("unsupported sequence units; expected 'degrees'")
    raw_steps = value.get("steps")
    if not isinstance(raw_steps, list):
        raise ValueError("sequence steps must be a JSON array")
    return [SequenceStep.from_dict(step) for step in raw_steps]


def save_sequence(path: str | Path, steps: Iterable[SequenceStep]) -> None:
    """Write a joint sequence as indented JSON."""
    with Path(path).open("w", encoding="utf-8") as stream:
        json.dump(sequence_to_dict(steps), stream, indent=2)
        stream.write("\n")


def load_sequence(path: str | Path) -> list[SequenceStep]:
    """Load and validate a joint sequence JSON file."""
    with Path(path).open("r", encoding="utf-8") as stream:
        return sequence_from_dict(json.load(stream))
