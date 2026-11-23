"""Descriptions and input limits for the guided hardware-test menu."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FieldSpec:
    name: str
    label: str
    kind: str
    minimum: float | None = None
    maximum: float | None = None
    choices: tuple[str, ...] = ()
    default: str = ""


@dataclass(frozen=True)
class GuidedTest:
    key: str
    title: str
    instruction: str
    fields: tuple[FieldSpec, ...]


GUIDED_TESTS = (
    GuidedTest(
        "base",
        "Base movement",
        "Lift the wheels. Motors enable, move for the selected time, stop, disable.",
        (
            FieldSpec(
                "direction",
                "Direction",
                "choice",
                choices=("W", "S", "A", "D", "Q", "E"),
                default="W",
            ),
            FieldSpec("speed", "Linear/yaw speed", "float", 0.02, 0.60, default="0.08"),
            FieldSpec("duration", "Duration, seconds", "float", 0.10, 1.00, default="0.30"),
        ),
    ),
    GuidedTest(
        "suction",
        "Suction",
        "Test valves and pumps for one gripper. Use OFF after checking ON.",
        (
            FieldSpec("gripper", "Gripper", "int", 1, 2, default="1"),
            FieldSpec("mask", "Cup mask", "int", 0, 15, default="15"),
            FieldSpec(
                "operation",
                "Operation",
                "choice",
                choices=("ON", "OFF"),
                default="ON",
            ),
        ),
    ),
    GuidedTest(
        "lift",
        "Gripper lift",
        "Move one gripper to a firmware-defined lift state: 0=up, 2=middle, 1=down.",
        (
            FieldSpec("gripper", "Gripper", "int", 1, 2, default="1"),
            FieldSpec(
                "state",
                "State",
                "choice",
                choices=("0", "2", "1"),
                default="0",
            ),
        ),
    ),
    GuidedTest(
        "vertical",
        "Vertical axis",
        "Command an absolute vertical position in millimetres.",
        (FieldSpec("position", "Position, mm", "int", 0, 255, default="20"),),
    ),
    GuidedTest(
        "slide",
        "Horizontal slide",
        "Command an absolute slide position in millimetres.",
        (FieldSpec("position", "Position, mm", "int", -37, 37, default="0"),),
    ),
    GuidedTest(
        "side_grips",
        "Side grips",
        "Open or close the side grips of the selected gripper.",
        (
            FieldSpec("gripper", "Gripper", "int", 1, 2, default="1"),
            FieldSpec(
                "state",
                "State",
                "choice",
                choices=("CLOSE", "OPEN"),
                default="CLOSE",
            ),
        ),
    ),
    GuidedTest(
        "cursor",
        "Cursor orientation",
        "Set the common cursor orientation.",
        (
            FieldSpec(
                "state",
                "Orientation",
                "choice",
                choices=("HORIZONTAL", "VERTICAL"),
                default="HORIZONTAL",
            ),
        ),
    ),
    GuidedTest(
        "blocks_shift",
        "Block shift",
        "Spread or close the block mechanism for one gripper.",
        (
            FieldSpec("gripper", "Gripper", "int", 1, 2, default="1"),
            FieldSpec(
                "state",
                "State",
                "choice",
                choices=("SPREAD", "CLOSE"),
                default="SPREAD",
            ),
        ),
    ),
    GuidedTest(
        "blocks_mask",
        "Block rotation mask",
        "Apply a 4-bit rotation mask to the selected gripper.",
        (
            FieldSpec("gripper", "Gripper", "int", 1, 2, default="1"),
            FieldSpec("mask", "Rotation mask", "int", 0, 15, default="15"),
        ),
    ),
    GuidedTest(
        "grab_rotate",
        "Full grab rotation cycle",
        "Run spread, masked rotation and close as one firmware operation.",
        (
            FieldSpec("gripper", "Gripper", "int", 1, 2, default="1"),
            FieldSpec("mask", "Rotation mask", "int", 0, 15, default="15"),
        ),
    ),
    GuidedTest(
        "telemetry",
        "Full diagnostics",
        "Read motion, BMS, wheel encoders, field side, start cord and nine switches.",
        (),
    ),
)
