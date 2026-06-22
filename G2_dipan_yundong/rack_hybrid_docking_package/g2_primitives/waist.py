"""Waist/body joint-space primitive.

This module is the importable implementation behind ``move_waist_by_json_path.py``.
It controls the five body/waist joints used to move the upper body between
grab, place, and home/default postures.

Why this class is more careful than the arm wrapper:

- the waist can require larger joint deltas than the arms during this workflow;
- large jumps are split into segments to avoid planning/transit failures;
- each segment is polled against current joint feedback before moving on;
- every input value is checked against the known joint limits below.

Call path:

The mission runner uses this for ``waist_for_grab``, ``waist_place_straight``,
and the home/default waist moves. The wrapper ``move_waist_by_json_path.py`` is
kept only for manual single-step validation of one pose JSON.
"""

from __future__ import annotations

import json
from pathlib import Path
import time

from .gdk_context import gdk_session


WAIST_KEYS = [
    "idx01_body_joint1",
    "idx02_body_joint2",
    "idx03_body_joint3",
    "idx04_body_joint4",
    "idx05_body_joint5",
]

# These limits are copied into the primitive so bad calibration files fail
# before GDK receives the command.  The values are radians.
WAIST_LIMITS = {
    "idx01_body_joint1": (-1.082104, 0.000174),
    "idx02_body_joint2": (-0.000174, 2.652900),
    "idx03_body_joint3": (-1.919862, 1.570970),
    "idx04_body_joint4": (-0.436332, 0.436332),
    "idx05_body_joint5": (-3.045599, 3.045599),
}


def read_waist_positions(path: Path) -> list[float]:
    """Read and validate a five-joint waist/body pose JSON file."""

    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected JSON object")
    missing = [key for key in WAIST_KEYS if key not in data]
    if missing:
        raise ValueError(f"{path}: missing waist joint keys: {missing}")

    positions = []
    for key in WAIST_KEYS:
        value = data[key]
        if not isinstance(value, (int, float)):
            raise ValueError(f"{path}: {key} is not numeric: {value!r}")
        value = float(value)
        lo, hi = WAIST_LIMITS[key]
        if not lo <= value <= hi:
            raise ValueError(f"{path}: {key}={value:.6f} outside limit [{lo:.6f}, {hi:.6f}]")
        positions.append(value)
    return positions


def read_current_waist(robot) -> list[float]:
    """Read current waist/body feedback from GDK joint states."""

    states = robot.get_joint_states()
    mapping = {state["name"]: state for state in states["states"]}
    missing = [key for key in WAIST_KEYS if key not in mapping]
    if missing:
        raise RuntimeError(f"missing current waist states: {missing}")
    return [float(mapping[key]["position"]) for key in WAIST_KEYS]


def max_abs_delta(a: list[float], b: list[float]) -> float:
    """Return the largest absolute joint difference between two poses."""

    return max(abs(x - y) for x, y in zip(a, b))


class WaistController:
    """Segmented controller for the five waist/body joints."""

    def move_to_json(
        self,
        json_path: str | Path,
        *,
        joint_speed_radps: float = 0.75,
        max_step_rad: float = 0.75,
        settle_tol_rad: float = 0.025,
        settle_timeout_s: float = 3.0,
        poll_s: float = 0.08,
        settle_s: float = 0.8,
        segment_settle_s: float = 0.35,
        command_retries: int = 3,
        dry_run: bool = False,
    ) -> bool:
        """Move waist/body joints to ``json_path``.

        ``dry_run=True`` validates the target JSON and arguments only.  Live
        execution opens one scoped GDK session, reads the current waist pose,
        moves through one or more interpolated waypoints, verifies each segment
        settles, and finally reports the remaining joint error.
        """

        self._validate_args(
            joint_speed_radps=joint_speed_radps,
            max_step_rad=max_step_rad,
            settle_tol_rad=settle_tol_rad,
            settle_timeout_s=settle_timeout_s,
            poll_s=poll_s,
            settle_s=settle_s,
            segment_settle_s=segment_settle_s,
            command_retries=command_retries,
        )
        path = Path(json_path)
        if not path.exists():
            raise FileNotFoundError(f"waist pose JSON not found: {path}")
        waist_positions = read_waist_positions(path)
        print(
            "move_waist_by_json_path "
            f"json={path} joint_speed_radps={joint_speed_radps:.3f} "
            f"positions={waist_positions}",
            flush=True,
        )
        if dry_run:
            print("dry-run: skip GDK init and waist movement", flush=True)
            return True

        with gdk_session() as agibot_gdk:
            robot = agibot_gdk.Robot()
            time.sleep(1.0)
            self.move_segmented(
                robot,
                waist_positions,
                joint_speed_radps,
                max_step_rad=max_step_rad,
                settle_tol_rad=settle_tol_rad,
                settle_timeout_s=settle_timeout_s,
                poll_s=poll_s,
                segment_settle_s=segment_settle_s,
                command_retries=command_retries,
            )
            final_error = max_abs_delta(read_current_waist(robot), waist_positions)
            print(f"move_waist_joint_result=0 final_max_error_rad={final_error:.6f}", flush=True)
            time.sleep(settle_s)
            return True

    def move_segmented(
        self,
        robot,
        target: list[float],
        speed: float,
        *,
        max_step_rad: float,
        settle_tol_rad: float,
        settle_timeout_s: float,
        poll_s: float,
        segment_settle_s: float,
        command_retries: int,
    ) -> None:
        """Move to ``target`` through bounded intermediate waypoints."""

        current = read_current_waist(robot)
        max_delta = max_abs_delta(current, target)
        if max_delta <= settle_tol_rad:
            print(f"waist_already_at_target max_error_rad={max_delta:.6f}", flush=True)
            return

        segments = max(1, int(max_delta / max_step_rad + 0.999))
        velocities = [speed] * len(target)
        print(
            "waist_segmented_move "
            f"segments={segments} max_delta_rad={max_delta:.6f} "
            f"max_step_rad={max_step_rad:.6f}",
            flush=True,
        )
        for index in range(1, segments + 1):
            alpha = index / segments
            waypoint = [c + (t - c) * alpha for c, t in zip(current, target)]
            print(f"waist_segment_start index={index}/{segments} waypoint={waypoint}", flush=True)
            last_error = float("inf")
            settled = False
            for attempt in range(1, command_retries + 1):
                # GDK sometimes rejects a command while the motion controller is
                # transitioning.  For waist moves we retry the same segment only
                # after checking feedback; if it already settled, no extra
                # command is sent.
                try:
                    result = robot.move_waist_joint(waypoint, velocities)
                    print(
                        f"waist_segment_command_result index={index}/{segments} "
                        f"attempt={attempt}/{command_retries} result={result}",
                        flush=True,
                    )
                except RuntimeError as exc:
                    print(
                        f"waist_segment_command_error index={index}/{segments} "
                        f"attempt={attempt}/{command_retries} error={exc}",
                        flush=True,
                    )

                deadline = time.time() + settle_timeout_s
                while True:
                    now = read_current_waist(robot)
                    last_error = max_abs_delta(now, waypoint)
                    if last_error <= settle_tol_rad:
                        print(
                            f"waist_segment_settled index={index}/{segments} "
                            f"attempt={attempt}/{command_retries} max_error_rad={last_error:.6f}",
                            flush=True,
                        )
                        settled = True
                        break
                    if time.time() >= deadline:
                        break
                    time.sleep(poll_s)

                if settled:
                    break
                if attempt < command_retries:
                    print(
                        f"waist_segment_retry index={index}/{segments} "
                        f"next_attempt={attempt + 1}/{command_retries} max_error_rad={last_error:.6f}",
                        flush=True,
                    )
                    time.sleep(segment_settle_s)

            if not settled:
                raise RuntimeError(f"waist segment {index}/{segments} did not settle: max_error_rad={last_error:.6f}")
            if segment_settle_s > 0.0 and index < segments:
                time.sleep(segment_settle_s)

    def _validate_args(
        self,
        *,
        joint_speed_radps: float,
        max_step_rad: float,
        settle_tol_rad: float,
        settle_timeout_s: float,
        poll_s: float,
        settle_s: float,
        segment_settle_s: float,
        command_retries: int,
    ) -> None:
        """Validate operator-tunable motion parameters before live movement."""

        if joint_speed_radps <= 0.0:
            raise ValueError("joint_speed_radps must be positive")
        if joint_speed_radps > 0.8:
            raise ValueError("joint_speed_radps is capped at 0.8")
        if max_step_rad <= 0.0:
            raise ValueError("max_step_rad must be positive")
        if max_step_rad > 0.8:
            raise ValueError("max_step_rad is capped at 0.8")
        if settle_tol_rad <= 0.0:
            raise ValueError("settle_tol_rad must be positive")
        if settle_timeout_s <= 0.0:
            raise ValueError("settle_timeout_s must be positive")
        if poll_s <= 0.0:
            raise ValueError("poll_s must be positive")
        if settle_s < 0.0:
            raise ValueError("settle_s must be >= 0")
        if segment_settle_s < 0.0:
            raise ValueError("segment_settle_s must be >= 0")
        if command_retries < 1:
            raise ValueError("command_retries must be >= 1")
