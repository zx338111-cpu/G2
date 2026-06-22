#!/usr/bin/env python3
"""Generate a direct raised place arm-joint JSON through live empty-arm calibration.

This script intentionally requires --confirm-physical. It moves the waist/body
to the calibrated place waist, moves both arms to the original final place joint
pose, applies a small end-effector Z offset through the live controller, captures
the resulting arm joints, and returns the upper body to home.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time
from typing import Any


PACKAGE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_DIR.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


ARM_KEYS = [
    "idx21_arm_l_joint1",
    "idx22_arm_l_joint2",
    "idx23_arm_l_joint3",
    "idx24_arm_l_joint4",
    "idx25_arm_l_joint5",
    "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
    "idx61_arm_r_joint1",
    "idx62_arm_r_joint2",
    "idx63_arm_r_joint3",
    "idx64_arm_r_joint4",
    "idx65_arm_r_joint5",
    "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]
WAIST_KEYS = [
    "idx01_body_joint1",
    "idx02_body_joint2",
    "idx03_body_joint3",
    "idx04_body_joint4",
    "idx05_body_joint5",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base-place-arm-json",
        default=str(PACKAGE_DIR / "calibration_records" / "rod07_place_final_arm_latest.json"),
    )
    parser.add_argument(
        "--place-waist-json",
        default=str(PACKAGE_DIR / "calibration_records" / "rod07_place_waist_adjusted_latest.json"),
    )
    parser.add_argument("--home-json", default="/data/wxf/wxf/positions/arm_default.json")
    parser.add_argument(
        "--output-json",
        default=str(PACKAGE_DIR / "calibration_records" / "rod07_place_final_arm_up020_latest.json"),
    )
    parser.add_argument("--z-m", type=float, default=0.02)
    parser.add_argument("--arm-joint-speed-radps", type=float, default=0.12)
    parser.add_argument("--waist-joint-speed-radps", type=float, default=0.75)
    parser.add_argument("--waist-max-step-rad", type=float, default=0.75)
    parser.add_argument("--settle-s", type=float, default=0.8)
    parser.add_argument("--waist-settle-tol-rad", type=float, default=0.05)
    parser.add_argument("--waist-settle-timeout-s", type=float, default=2.0)
    parser.add_argument("--poll-s", type=float, default=0.08)
    parser.add_argument("--startup-wait-s", type=float, default=2.0)
    parser.add_argument("--no-return-home", action="store_true")
    parser.add_argument("--confirm-physical", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if not 0.0 < args.z_m <= 0.05:
        raise SystemExit("--z-m must be in (0, 0.05]")
    if not 0.0 < args.arm_joint_speed_radps <= 0.5:
        raise SystemExit("--arm-joint-speed-radps must be in (0, 0.5]")
    if not 0.0 < args.waist_joint_speed_radps <= 0.8:
        raise SystemExit("--waist-joint-speed-radps must be in (0, 0.8]")
    if not 0.0 < args.waist_max_step_rad <= 0.8:
        raise SystemExit("--waist-max-step-rad must be in (0, 0.8]")
    if args.settle_s < 0.0:
        raise SystemExit("--settle-s must be >= 0")
    return args


def read_joint_values(path: Path, keys: list[str]) -> list[float]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected JSON object")
    missing = [key for key in keys if key not in data]
    if missing:
        raise ValueError(f"{path}: missing joint keys: {missing}")
    values: list[float] = []
    for key in keys:
        value = data[key]
        if not isinstance(value, (int, float)):
            raise ValueError(f"{path}: {key} is not numeric: {value!r}")
        values.append(float(value))
    return values


def scalar_fields(obj: Any) -> dict[str, Any]:
    fields: dict[str, Any] = {}
    for name in dir(obj):
        if name.startswith("_"):
            continue
        try:
            value = getattr(obj, name)
        except Exception as exc:
            fields[name] = f"<read_error {type(exc).__name__}: {exc}>"
            continue
        if callable(value):
            continue
        if isinstance(value, (str, int, float, bool, type(None))):
            fields[name] = value
    return fields


def normalize_joint_container(raw: Any) -> list[Any]:
    if isinstance(raw, dict):
        states = raw.get("states")
        return list(states) if isinstance(states, (list, tuple)) else []
    states = getattr(raw, "states", None)
    if states is not None:
        return list(states)
    if isinstance(raw, (list, tuple)):
        return list(raw)
    try:
        return list(raw)
    except TypeError:
        return []


def read_positions_by_name(robot: Any) -> dict[str, float]:
    raw = robot.get_joint_states()
    positions: dict[str, float] = {}
    for index, joint in enumerate(normalize_joint_container(raw)):
        record = dict(joint) if isinstance(joint, dict) else scalar_fields(joint)
        name = str(record.get("name", f"joint_{index}"))
        value = record.get("position")
        if isinstance(value, (int, float)):
            positions[name] = float(value)
    return positions


def max_abs_delta(a: list[float], b: list[float]) -> float:
    return max(abs(left - right) for left, right in zip(a, b))


def read_current(robot: Any, keys: list[str]) -> list[float]:
    positions = read_positions_by_name(robot)
    missing = [key for key in keys if key not in positions]
    if missing:
        raise RuntimeError(f"missing current joint states: {missing}")
    return [positions[key] for key in keys]


def move_waist_segmented(robot: Any, target: list[float], speed: float, args: argparse.Namespace) -> None:
    current = read_current(robot, WAIST_KEYS)
    max_delta = max_abs_delta(current, target)
    if max_delta <= args.waist_settle_tol_rad:
        print(f"waist_already_at_target max_error_rad={max_delta:.6f}", flush=True)
        return
    segments = max(1, int(max_delta / args.waist_max_step_rad + 0.999))
    velocities = [speed] * len(target)
    print(f"waist_segmented_move segments={segments} max_delta_rad={max_delta:.6f}", flush=True)
    for index in range(1, segments + 1):
        alpha = index / segments
        waypoint = [start + (end - start) * alpha for start, end in zip(current, target)]
        print(f"waist_segment_start index={index}/{segments} waypoint={waypoint}", flush=True)
        result = robot.move_waist_joint(waypoint, velocities)
        print(f"waist_segment_command_result index={index}/{segments} result={result}", flush=True)
        deadline = time.time() + args.waist_settle_timeout_s
        while True:
            error = max_abs_delta(read_current(robot, WAIST_KEYS), waypoint)
            if error <= args.waist_settle_tol_rad:
                print(f"waist_segment_settled index={index}/{segments} max_error_rad={error:.6f}", flush=True)
                break
            if time.time() >= deadline:
                raise RuntimeError(f"waist segment {index}/{segments} did not settle: max_error_rad={error:.6f}")
            time.sleep(args.poll_s)


def move_arm(robot: Any, target: list[float], speed: float, settle_s: float, label: str) -> None:
    print(f"arm_move_start label={label} speed={speed:.3f}", flush=True)
    result = robot.move_arm_joint(target, [speed] * len(target), 2)
    print(f"arm_move_result label={label} result={result}", flush=True)
    if result != 0:
        raise RuntimeError(f"{label} move_arm_joint failed: result={result}")
    if settle_s > 0.0:
        time.sleep(settle_s)


def main() -> int:
    args = parse_args()
    base_place_arm_json = Path(args.base_place_arm_json).resolve()
    place_waist_json = Path(args.place_waist_json).resolve()
    home_json = Path(args.home_json).resolve()
    output_json = Path(args.output_json).resolve()
    capture_json = output_json.with_name(output_json.stem.replace("_latest", "") + f"_capture_{datetime.now():%Y%m%d_%H%M%S}.json")

    required_paths = (base_place_arm_json, place_waist_json) if args.dry_run else (base_place_arm_json, place_waist_json, home_json)
    for path in required_paths:
        if not path.exists():
            raise SystemExit(f"missing required path: {path}")

    plan = {
        "event": "direct_place_pose_offset_plan",
        "base_place_arm_json": str(base_place_arm_json),
        "place_waist_json": str(place_waist_json),
        "home_json": str(home_json),
        "output_json": str(output_json),
        "capture_json": str(capture_json),
        "z_m": args.z_m,
        "return_home": not args.no_return_home,
    }
    print(json.dumps(plan, ensure_ascii=False), flush=True)
    if args.dry_run:
        print("dry-run: no GDK init and no robot motion", flush=True)
        return 0
    if not args.confirm_physical:
        raise SystemExit("--confirm-physical is required for live calibration")

    import agibot_gdk
    from end_effector_controller import EndEffectorController

    base_arm = read_joint_values(base_place_arm_json, ARM_KEYS)
    place_waist = read_joint_values(place_waist_json, WAIST_KEYS)
    home_arm = read_joint_values(home_json, ARM_KEYS)
    home_waist = read_joint_values(home_json, WAIST_KEYS)

    gdk_inited = False
    try:
        result = agibot_gdk.gdk_init()
        gdk_res = getattr(agibot_gdk, "GDKRes", None)
        if gdk_res is not None and result not in (None, gdk_res.kSuccess):
            raise RuntimeError(f"GDK init failed: {result}")
        gdk_inited = True
        robot = agibot_gdk.Robot()
        time.sleep(args.startup_wait_s)

        move_waist_segmented(robot, place_waist, args.waist_joint_speed_radps, args)
        move_arm(robot, base_arm, args.arm_joint_speed_radps, args.settle_s, "base_place_arm")

        controller = EndEffectorController(robot)
        ok = controller.adjust_arms_relative(offset_l=(0.0, 0.0, args.z_m), offset_r=(0.0, 0.0, args.z_m))
        if not ok:
            raise RuntimeError("end-effector Z offset failed")
        if args.settle_s > 0.0:
            time.sleep(args.settle_s)

        positions = read_positions_by_name(robot)
        missing = [key for key in ARM_KEYS if key not in positions]
        if missing:
            raise RuntimeError(f"missing captured arm joints: {missing}")
        output_json.parent.mkdir(parents=True, exist_ok=True)
        arm_output = {key: positions[key] for key in ARM_KEYS}
        output_json.write_text(json.dumps(arm_output, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        capture = {
            "event": "direct_place_pose_offset_captured",
            "timestamp_local": datetime.now().isoformat(timespec="seconds"),
            "z_m": args.z_m,
            "base_place_arm_json": str(base_place_arm_json),
            "place_waist_json": str(place_waist_json),
            "output_json": str(output_json),
            "arm_output": arm_output,
            "all_positions_by_name": positions,
        }
        capture_json.write_text(json.dumps(capture, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(json.dumps({"event": "direct_place_pose_offset_done", "output_json": str(output_json), "capture_json": str(capture_json)}, ensure_ascii=False), flush=True)

        if not args.no_return_home:
            move_arm(robot, home_arm, min(args.arm_joint_speed_radps, 0.2), args.settle_s, "home_arm")
            move_waist_segmented(robot, home_waist, args.waist_joint_speed_radps, args)
        return 0
    finally:
        if gdk_inited:
            try:
                agibot_gdk.gdk_release()
                print("GDK release ok", flush=True)
            except Exception as exc:
                print(f"GDK release failed: {exc}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
