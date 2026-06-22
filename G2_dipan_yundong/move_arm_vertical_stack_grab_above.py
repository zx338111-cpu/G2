#!/usr/bin/env python3
"""Move to a vertical-stack grab-above pose.

The rack rods are treated as one vertical column: all layers share the same
end-effector X/Y and orientation. The first rod joint pose is used as the
baseline, then the selected rod applies a pure end-effector Z offset.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time


DEFAULT_BASE_JSON = (
    "/data/btgys/bengtian_backup_20260608_081250/wxf/"
    "positions/arm_position_to_grab_第一根.json"
)

LEFT_ARM_KEYS = [
    "idx21_arm_l_joint1",
    "idx22_arm_l_joint2",
    "idx23_arm_l_joint3",
    "idx24_arm_l_joint4",
    "idx25_arm_l_joint5",
    "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
]
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
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected JSON object")

    missing = [key for key in LEFT_ARM_KEYS + RIGHT_ARM_KEYS if key not in data]
    if missing:
        raise ValueError(f"{path}: missing arm joint keys: {missing}")

    values = []
    for key in LEFT_ARM_KEYS + RIGHT_ARM_KEYS:
        value = data[key]
        if not isinstance(value, (int, float)):
            raise ValueError(f"{path}: {key} is not numeric: {value!r}")
        values.append(float(value))
    return values


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rod-index", type=int, required=True, choices=range(1, 8))
    parser.add_argument(
        "--pitch-m",
        type=float,
        required=True,
        help="Layer spacing in meters. Positive Z is up; negative Z is down.",
    )
    parser.add_argument("--base-index", type=int, default=1, choices=range(1, 8))
    parser.add_argument("--base-json", default=DEFAULT_BASE_JSON)
    parser.add_argument("--joint-speed-radps", type=float, default=0.2)
    parser.add_argument("--settle-s", type=float, default=0.6)
    parser.add_argument("--dry-run", action="store_true", help="Print computed offsets without moving.")
    args = parser.parse_args()

    if abs(args.pitch_m) > 0.20:
        raise SystemExit("--pitch-m is capped at +/-0.20m per layer")
    if args.joint_speed_radps <= 0:
        raise SystemExit("--joint-speed-radps must be positive")
    if args.settle_s < 0:
        raise SystemExit("--settle-s must be >= 0")
    return args


def main() -> int:
    args = parse_args()
    base_json = Path(args.base_json)
    if not base_json.exists():
        print(f"base JSON not found: {base_json}", flush=True)
        return 1

    try:
        arm_positions = read_arm_positions(base_json)
    except Exception as exc:
        print(f"failed to read base arm pose: {type(exc).__name__}: {exc}", flush=True)
        return 1

    z_offset_m = (args.rod_index - args.base_index) * args.pitch_m
    print(
        "vertical_stack_grab_above "
        f"rod_index={args.rod_index} base_index={args.base_index} "
        f"pitch_m={args.pitch_m:.6f} z_offset_m={z_offset_m:.6f} "
        f"base_json={base_json}",
        flush=True,
    )
    if args.dry_run:
        print("dry-run: skip GDK init and arm movement", flush=True)
        return 0

    import agibot_gdk
    from end_effector_controller import EndEffectorController

    gdk_inited = False
    try:
        if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
            print("GDK init failed", flush=True)
            return 1
        gdk_inited = True

        robot = agibot_gdk.Robot()
        time.sleep(2.0)

        velocities = [args.joint_speed_radps] * len(arm_positions)
        result = robot.move_arm_joint(arm_positions, velocities, 2)
        print(f"base_joint_move_result={result}", flush=True)
        time.sleep(args.settle_s)

        if abs(z_offset_m) > 1e-6:
            controller = EndEffectorController(robot)
            ok = controller.adjust_arms_relative(
                offset_l=(0.0, 0.0, z_offset_m),
                offset_r=(0.0, 0.0, z_offset_m),
            )
            if not ok:
                print("vertical_stack_z_offset_failed", flush=True)
                return 1
        else:
            print("vertical_stack_z_offset skipped for base layer", flush=True)
        return 0
    except Exception as exc:
        print(f"vertical_stack_grab_above failed: {type(exc).__name__}: {exc}", flush=True)
        return 1
    finally:
        if gdk_inited:
            try:
                agibot_gdk.gdk_release()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
