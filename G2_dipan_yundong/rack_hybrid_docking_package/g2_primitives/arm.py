"""Dual-arm joint-space primitive.

This module is the importable implementation behind ``move_arm_by_json_path.py``.
The seven-rods runner calls ``ArmJointController`` directly so one Python
process can own logging and checkpointing, while the wrapper script remains
available for manual single-step tests.

Input pose contract:

- JSON must contain all 14 arm joint keys listed below;
- values are sent in left-arm-then-right-arm order expected by GDK;
- this primitive does not infer or clamp missing joint values, because a partial
  pose file is more dangerous than an explicit failure.

Call path:

``industrial_cell_7_rods_single_debug.py`` creates local-plan entries whose
legacy script name is ``move_arm_by_json_path.py``. At runtime those entries are
dispatched to ``ArmJointController.move_to_json`` in this file. The standalone
``move_arm_by_json_path.py`` wrapper calls the same method for manual tests.
"""

from __future__ import annotations

import json
from pathlib import Path
import time

from .gdk_context import gdk_session


LEFT_ARM_KEYS = [
    "idx21_arm_l_joint1",
    "idx22_arm_l_joint2",
    "idx23_arm_l_joint3",
    "idx24_arm_l_joint4",
    "idx25_arm_l_joint5",
    "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
]

# GDK's ``move_arm_joint`` expects one flat 14-joint list.  Keep these key lists
# in the exact output order so calibration JSON files can be read deterministically.
RIGHT_ARM_KEYS = [
    "idx61_arm_r_joint1",
    "idx62_arm_r_joint2",
    "idx63_arm_r_joint3",
    "idx64_arm_r_joint4",
    "idx65_arm_r_joint5",
    "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]


def read_arm_positions(path: Path) -> list[float]:
    """Read and validate a 14-joint arm pose JSON file."""

    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected JSON object")
    missing = [key for key in LEFT_ARM_KEYS + RIGHT_ARM_KEYS if key not in data]
    if missing:
        raise ValueError(f"{path}: missing arm joint keys: {missing}")

    positions = []
    for key in LEFT_ARM_KEYS + RIGHT_ARM_KEYS:
        value = data[key]
        if not isinstance(value, (int, float)):
            raise ValueError(f"{path}: {key} is not numeric: {value!r}")
        positions.append(float(value))
    return positions


class ArmJointController:
    """Small, scoped wrapper around ``Robot.move_arm_joint``.

    The controller owns only one action: move both arms to a fully specified
    joint pose.  Higher-level sequencing, retries, and safety checks stay in the
    mission runner.
    """

    def move_to_json(
        self,
        json_path: str | Path,
        *,
        joint_speed_radps: float = 0.12,
        settle_s: float = 0.8,
        dry_run: bool = False,
    ) -> bool:
        """Move both arms to ``json_path``.

        ``dry_run=True`` validates the input and prints the planned action but
        does not initialize GDK or send a motion command.  Live calls cap speed
        defensively so a mis-typed CLI argument cannot produce a high-speed arm
        move.
        """

        if joint_speed_radps <= 0.0:
            raise ValueError("joint_speed_radps must be positive")
        if joint_speed_radps > 0.5:
            raise ValueError("joint_speed_radps is capped at 0.5")
        if settle_s < 0.0:
            raise ValueError("settle_s must be >= 0")

        path = Path(json_path)
        if not path.exists():
            raise FileNotFoundError(f"arm pose JSON not found: {path}")
        arm_positions = read_arm_positions(path)

        print(
            "move_arm_by_json_path "
            f"json={path} joint_speed_radps={joint_speed_radps:.3f} "
            f"settle_s={settle_s:.3f}",
            flush=True,
        )
        if dry_run:
            print("dry-run: skip GDK init and arm movement", flush=True)
            return True

        with gdk_session() as agibot_gdk:
            robot = agibot_gdk.Robot()
            time.sleep(2.0)
            result = robot.move_arm_joint(arm_positions, [joint_speed_radps] * 14, 2)
            print(f"move_arm_joint_result={result}", flush=True)
            time.sleep(settle_s)
            return result == 0
