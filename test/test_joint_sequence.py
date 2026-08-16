import json

import pytest

from pico_6dof_robot_driver.joint_sequence import (
    SequenceStep,
    interpolate_sequence,
    load_sequence,
    save_sequence,
    sequence_from_dict,
    sequence_to_dict,
)


def test_sequence_json_round_trip(tmp_path):
    steps = [
        SequenceStep((0, -20, 10, 0, 30, 0), 2.5, 0.25),
        SequenceStep((10, -10, 5, 90, 0, -45), 0.0, 1.0),
    ]
    path = tmp_path / "sequence.json"

    save_sequence(path, steps)

    assert load_sequence(path) == steps
    document = json.loads(path.read_text(encoding="utf-8"))
    assert document["format"] == "pico_6dof_joint_sequence"
    assert document["version"] == 1
    assert document["units"] == "degrees"
    assert document["steps"][0]["gripper_position"] == 0.25


@pytest.mark.parametrize(
    "value, message",
    [
        ({}, "unsupported sequence format"),
        (
            {
                "format": "pico_6dof_joint_sequence",
                "version": 2,
                "units": "degrees",
                "steps": [],
            },
            "version",
        ),
        (
            {
                "format": "pico_6dof_joint_sequence",
                "version": 1,
                "units": "radians",
                "steps": [],
            },
            "units",
        ),
    ],
)
def test_rejects_incompatible_documents(value, message):
    with pytest.raises(ValueError, match=message):
        sequence_from_dict(value)


def test_rejects_invalid_step_values():
    with pytest.raises(ValueError, match="six"):
        SequenceStep((0, 1), 1.0)
    with pytest.raises(ValueError, match="hold_seconds"):
        SequenceStep((0, 0, 0, 0, 0, 0), -1.0)
    with pytest.raises(ValueError, match="must be numbers"):
        SequenceStep((0, 0, 0, 0, 0, None), 1.0)
    with pytest.raises(ValueError, match="gripper_position"):
        SequenceStep((0, 0, 0, 0, 0, 0), 1.0, 1.1)


def test_loads_legacy_step_without_gripper_command():
    value = {
        "format": "pico_6dof_joint_sequence",
        "version": 1,
        "units": "degrees",
        "steps": [
            {
                "positions": [1, 2, 3, 4, 5, 6],
                "hold_seconds": 2.0,
            }
        ],
    }

    steps = sequence_from_dict(value)

    assert steps[0].gripper_position is None
    assert "gripper_position" not in steps[0].to_dict()


def test_dict_representation_is_json_serializable():
    value = sequence_to_dict(
        [SequenceStep((1, 2, 3, 4, 5, 6), 1.25)]
    )
    assert json.loads(json.dumps(value))["steps"][0]["positions"] == [
        1,
        2,
        3,
        4,
        5,
        6,
    ]


def test_interpolates_synchronized_smooth_trajectory_and_holds():
    steps = [
        SequenceStep((10, 20, 0, 0, 0, 0), 2.0, 0.5),
        SequenceStep((20, 20, 0, 0, 0, 0), 0.0, 1.0),
    ]

    samples, arrivals = interpolate_sequence(
        (0, 0, 0, 0, 0, 0), steps, speed_deg_s=10.0, sample_rate_hz=10.0
    )

    assert arrivals == pytest.approx([3.0, 6.5])
    assert samples[0].time_seconds == pytest.approx(0.1)
    assert samples[0].positions_deg[1] == pytest.approx(0.065185185)
    assert samples[29].positions_deg == pytest.approx(steps[0].positions_deg)
    assert samples[30].time_seconds == pytest.approx(5.0)
    assert samples[30].positions_deg == pytest.approx(steps[0].positions_deg)
    assert samples[-1].time_seconds == pytest.approx(6.5)
    assert samples[-1].positions_deg == pytest.approx(steps[1].positions_deg)
    assert all(
        later.time_seconds > earlier.time_seconds
        for earlier, later in zip(samples, samples[1:])
    )


@pytest.mark.parametrize("speed", [0.0, -1.0, float("nan")])
def test_rejects_invalid_trajectory_speed(speed):
    with pytest.raises(ValueError, match="speed"):
        interpolate_sequence(
            (0, 0, 0, 0, 0, 0),
            [SequenceStep((1, 0, 0, 0, 0, 0), 0.0)],
            speed,
        )
