#!/usr/bin/env python3
"""Read-only capture for a manually tuned grab calibration point.

This script intentionally does not call request_chassis_control(), move_chassis(),
normal_navi(), relative_move(), move_arm_joint(), move_ee_pos(), or cancel_task().
It only reads the current GDK state and writes a timestamped JSON record.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import sys
import time
from typing import Any


PACKAGE_DIR = Path(__file__).resolve().parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from industrial_map_nav_guarded import build_preflight, load_config, pose_to_station_dict
from site_profile import (
    SiteProfileError,
    calibration_dir_from_profile,
    grab_latest_path,
    load_site_profile,
    station_config_path,
)


INVALID_DISTANCE_MM = 65535
ULTRASONIC_GROUPS = {
    "front": (0, 1),
    "right": (2, 3),
    "rear": (4, 5),
    "left": (6, 7),
}


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


def task_to_dict(pnc: Any) -> dict[str, Any] | None:
    try:
        task = pnc.get_task_state()
    except Exception as exc:
        return {"error": f"{type(exc).__name__}: {exc}"}
    return scalar_fields(task)


def odom_to_dict(slam: Any) -> dict[str, Any] | None:
    try:
        odom = slam.get_odom_info()
    except Exception as exc:
        return {"error": f"{type(exc).__name__}: {exc}"}
    fields = scalar_fields(odom)
    velocity_body = getattr(odom, "velocity_body", None)
    if velocity_body is not None:
        vx = float(getattr(velocity_body, "x", 0.0))
        vy = float(getattr(velocity_body, "y", 0.0))
        vz = float(getattr(velocity_body, "z", 0.0))
        fields["velocity_body"] = {"x": vx, "y": vy, "z": vz}
        fields["linear_speed_mps"] = math.sqrt(vx * vx + vy * vy + vz * vz)
    return fields


def normalize_joint_container(raw: Any) -> list[Any]:
    if raw is None:
        return []
    if isinstance(raw, dict):
        states = raw.get("states")
        if isinstance(states, (list, tuple)):
            return list(states)
        return []
    if isinstance(raw, (list, tuple)):
        return list(raw)
    states = getattr(raw, "states", None)
    if states is not None:
        return list(states)
    try:
        return list(raw)
    except TypeError:
        return [raw]


def read_joint_records(robot: Any) -> dict[str, Any]:
    try:
        raw = robot.get_joint_states()
    except Exception as exc:
        return {"error": f"{type(exc).__name__}: {exc}", "all": [], "arm_positions": {}}

    records = []
    position_by_name: dict[str, float] = {}
    arm_positions: dict[str, float] = {}
    for index, joint in enumerate(normalize_joint_container(raw)):
        if isinstance(joint, dict):
            record = dict(joint)
        else:
            record = scalar_fields(joint)
        record.setdefault("index", index)
        name = str(record.get("name", f"joint_{index}"))
        position = record.get("position")
        records.append(record)
        if isinstance(position, (int, float)):
            position_by_name[name] = float(position)
            if "arm" in name.lower():
                arm_positions[name] = float(position)
    return {"all": records, "position_by_name": position_by_name, "arm_positions": arm_positions}


def selected_ultrasonic_rows(radar_data: dict[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for row in radar_data.get("ultrasonic_radar_datas", []):
        radar_id = row.get("id")
        try:
            radar_id = int(radar_id)
        except (TypeError, ValueError):
            continue
        if not 0 <= radar_id <= 7:
            continue
        rows.append(
            {
                "id": radar_id,
                "distance_mm": row.get("distance_mm"),
                "fault_state": row.get("fault_state"),
            }
        )
    rows.sort(key=lambda item: item["id"])
    return rows


def group_distances(rows: list[dict[str, Any]], ids: tuple[int, ...]) -> dict[str, Any]:
    by_id = {int(row["id"]): row for row in rows}
    grouped = []
    valid_values = []
    for radar_id in ids:
        row = by_id.get(radar_id)
        if row is None:
            grouped.append({"id": radar_id, "distance_mm": None, "fault_state": "missing", "valid": False})
            continue
        distance = row.get("distance_mm")
        fault_state = row.get("fault_state")
        valid = False
        try:
            distance_int = int(distance)
        except (TypeError, ValueError):
            distance_int = None
        if fault_state == 0 and distance_int is not None and 0 < distance_int < INVALID_DISTANCE_MM:
            valid = True
            valid_values.append(distance_int)
        grouped.append(
            {
                "id": radar_id,
                "distance_mm": distance_int,
                "fault_state": fault_state,
                "valid": valid,
            }
        )
    return {"ids": ids, "min_mm": min(valid_values) if valid_values else None, "rows": grouped}


def read_ultrasonic_samples(radar: Any, samples: int, interval_s: float) -> list[dict[str, Any]]:
    start = time.time()
    result = []
    for index in range(samples):
        data = radar.get_latest_ultrasonic_radar()
        rows = selected_ultrasonic_rows(data)
        groups = {name: group_distances(rows, ids) for name, ids in ULTRASONIC_GROUPS.items()}
        result.append(
            {
                "index": index,
                "elapsed_s": round(time.time() - start, 3),
                "rows": rows,
                "groups": groups,
            }
        )
        if index + 1 < samples:
            time.sleep(interval_s)
    return result


def summarize_ultrasonic(samples: list[dict[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    for name in ULTRASONIC_GROUPS:
        values = [
            sample["groups"][name]["min_mm"]
            for sample in samples
            if sample["groups"][name]["min_mm"] is not None
        ]
        if not values:
            summary[name] = {"valid_samples": 0, "min_mm": None, "max_mm": None, "median_mm": None}
            continue
        summary[name] = {
            "valid_samples": len(values),
            "min_mm": min(values),
            "max_mm": max(values),
            "median_mm": statistics.median(values),
            "last_mm": values[-1],
        }
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read-only capture for grab point calibration")
    parser.add_argument("--rod-index", type=int, required=True)
    parser.add_argument("--label", default="")
    parser.add_argument("--note", default="")
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--interval-s", type=float, default=0.12)
    parser.add_argument("--startup-wait-s", type=float, default=0.8)
    parser.add_argument(
        "--profile",
        default="",
        help="Site profile JSON or directory. When set, writes under that profile by default.",
    )
    parser.add_argument("--output-dir", default="", help="Calibration output directory; overrides --profile default")
    parser.add_argument("--config", default="", help="Station config path; overrides --profile station_config")
    parser.add_argument(
        "--update-latest",
        action="store_true",
        help="Update rodXX_grab_pose_latest.json. With --profile this is already the default.",
    )
    parser.add_argument(
        "--no-update-latest",
        action="store_true",
        help="Do not update rodXX_grab_pose_latest.json after writing the timestamped pose.",
    )
    return parser.parse_args()


def resolve_paths(args: argparse.Namespace) -> tuple[Path, Path, Path | None, str | None]:
    """Resolve config/output/latest paths from CLI/profile inputs."""

    if args.update_latest and args.no_update_latest:
        raise SiteProfileError("--update-latest and --no-update-latest cannot be used together")
    if args.profile:
        profile_file, profile_dir, profile = load_site_profile(args.profile)
        config_path = Path(args.config).resolve() if args.config else station_config_path(profile_dir, profile)
        output_dir = Path(args.output_dir).resolve() if args.output_dir else calibration_dir_from_profile(profile_dir, profile)
        latest_path = None if args.no_update_latest else grab_latest_path(profile_dir, profile, args.rod_index)
        return config_path, output_dir, latest_path, str(profile_file)
    config_path = Path(args.config or (PACKAGE_DIR / "industrial_station_config.json")).resolve()
    output_dir = Path(args.output_dir or (PACKAGE_DIR / "calibration_records")).resolve()
    latest_path = output_dir / f"rod{args.rod_index:02d}_grab_pose_latest.json" if args.update_latest else None
    return config_path, output_dir, latest_path, None


def main() -> int:
    args = parse_args()
    if not 1 <= args.rod_index <= 7:
        raise SystemExit("--rod-index must be in 1..7")
    if args.samples <= 0:
        raise SystemExit("--samples must be positive")
    if args.interval_s < 0.0:
        raise SystemExit("--interval-s must be >= 0")

    import agibot_gdk

    try:
        config_path, output_dir, latest_pose_path, profile_file = resolve_paths(args)
    except SiteProfileError as exc:
        raise SystemExit(str(exc)) from exc
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = output_dir / f"rod{args.rod_index:02d}_grab_calibration_{timestamp}.json"
    arm_pose_path = output_dir / f"rod{args.rod_index:02d}_grab_pose_{timestamp}.json"

    gdk_inited = False
    radar = None
    try:
        result = agibot_gdk.gdk_init()
        gdk_res = getattr(agibot_gdk, "GDKRes", None)
        if gdk_res is not None and result not in (None, gdk_res.kSuccess):
            raise RuntimeError(f"GDK init failed: {result}")
        gdk_inited = True

        robot = agibot_gdk.Robot()
        pnc = agibot_gdk.Pnc()
        slam = agibot_gdk.Slam()
        map_manager = agibot_gdk.Map()
        radar = agibot_gdk.UltrasonicRadar()
        time.sleep(args.startup_wait_s)

        config = load_config(config_path)
        readiness = build_preflight(robot, pnc, slam, map_manager, config)
        pose = pose_to_station_dict(slam.get_curr_pose())
        task = task_to_dict(pnc)
        odom = odom_to_dict(slam)
        joints = read_joint_records(robot)
        ultrasonic_samples = read_ultrasonic_samples(radar, args.samples, args.interval_s)
        ultrasonic_summary = summarize_ultrasonic(ultrasonic_samples)

        record = {
            "schema": "g2_grab_calibration_point_v1",
            "timestamp_local": timestamp,
            "rod_index": args.rod_index,
            "label": args.label,
            "note": args.note,
            "profile": profile_file,
            "read_only": True,
            "no_motion_commands_sent": True,
            "readiness": readiness,
            "current_pose": pose,
            "task_state": task,
            "odom": odom,
            "joints": joints,
            "arm_pose_json_path": str(arm_pose_path),
            "latest_pose_json_path": None if latest_pose_path is None else str(latest_pose_path),
            "ultrasonic_summary": ultrasonic_summary,
            "ultrasonic_samples": ultrasonic_samples,
        }
        output_path.write_text(json.dumps(record, indent=2, ensure_ascii=False), encoding="utf-8")
        pose_json = json.dumps(joints.get("position_by_name", {}), indent=2, ensure_ascii=False) + "\n"
        arm_pose_path.write_text(pose_json, encoding="utf-8")
        if latest_pose_path is not None:
            latest_pose_path.parent.mkdir(parents=True, exist_ok=True)
            latest_pose_path.write_text(pose_json, encoding="utf-8")

        print(
            json.dumps(
                {
                    "event": "grab_calibration_point_captured",
                    "profile": profile_file,
                    "path": str(output_path),
                    "arm_pose_json_path": str(arm_pose_path),
                    "latest_pose_json_path": None if latest_pose_path is None else str(latest_pose_path),
                    "rod_index": args.rod_index,
                    "label": args.label,
                    "readiness_ok": readiness.get("ok"),
                    "readiness_problems": readiness.get("problems"),
                    "pose": pose,
                    "front_distance_summary_mm": ultrasonic_summary.get("front"),
                    "arm_joint_count": len(joints.get("arm_positions", {})),
                    "all_joint_count": len(joints.get("all", [])),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
        return 0
    finally:
        if radar is not None:
            try:
                radar.close_ultrasonic_radar()
            except Exception:
                pass
        if gdk_inited:
            try:
                agibot_gdk.gdk_release()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
