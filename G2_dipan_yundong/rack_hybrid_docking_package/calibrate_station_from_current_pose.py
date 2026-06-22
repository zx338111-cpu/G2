#!/usr/bin/env python3
"""Calibrate a map station from the robot's current SLAM pose.

The default mode is read/update only: it does not request chassis control,
navigation, relative motion, arm motion, or task cancellation.
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

from industrial_map_nav_guarded import load_config, pose_to_station_dict, station_names, yaw_deg_from_quaternion
from site_profile import (
    SiteProfileError,
    calibration_dir_from_profile,
    load_site_profile,
    station_config_path,
)


def wrap_deg(value: float) -> float:
    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def planar_distance(a: dict[str, Any], b: dict[str, Any]) -> float:
    dx = float(a["position"]["x"]) - float(b["position"]["x"])
    dy = float(a["position"]["y"]) - float(b["position"]["y"])
    return math.hypot(dx, dy)


def read_pose_samples(slam: Any, *, samples: int, interval_s: float) -> list[dict[str, Any]]:
    poses: list[dict[str, Any]] = []
    for index in range(samples):
        poses.append(pose_to_station_dict(slam.get_curr_pose()))
        if index + 1 < samples:
            time.sleep(interval_s)
    return poses


def pose_stability(poses: list[dict[str, Any]]) -> dict[str, Any]:
    xs = [float(pose["position"]["x"]) for pose in poses]
    ys = [float(pose["position"]["y"]) for pose in poses]
    yaws = [yaw_deg_from_quaternion(pose["orientation"]) for pose in poses]
    yaw0 = yaws[0]
    yaw_offsets = [wrap_deg(yaw - yaw0) for yaw in yaws]
    xy_span_m = 0.0
    for lhs in poses:
        for rhs in poses:
            xy_span_m = max(xy_span_m, planar_distance(lhs, rhs))
    return {
        "samples": len(poses),
        "xy_span_m": xy_span_m,
        "yaw_span_deg": max(yaw_offsets) - min(yaw_offsets),
        "x_median": statistics.median(xs),
        "y_median": statistics.median(ys),
        "yaw_first_deg": yaw0,
        "yaw_last_deg": yaws[-1],
    }


def write_json(path: Path, data: Any) -> None:
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--station", required=True, help="Station key in industrial_station_config.json")
    parser.add_argument(
        "--mode",
        choices=("yaw-only", "position-only", "full"),
        default="yaw-only",
        help="Which parts of the current pose should replace the configured station.",
    )
    parser.add_argument(
        "--profile",
        default="",
        help="Site profile JSON or directory. When set, --config and --output-dir default inside that profile.",
    )
    parser.add_argument("--config", default="", help="Station config path; overrides --profile station_config")
    parser.add_argument("--output-dir", default="", help="Calibration output directory; overrides --profile default")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--interval-s", type=float, default=0.12)
    parser.add_argument("--max-xy-span-m", type=float, default=0.015)
    parser.add_argument("--max-yaw-span-deg", type=float, default=1.0)
    parser.add_argument("--allow-moving", action="store_true", help="Write even if sampled pose is not stable")
    parser.add_argument("--allow-map-mismatch", action="store_true")
    parser.add_argument("--note", default="")
    parser.add_argument(
        "--no-write",
        action="store_true",
        help="Capture and print the proposed update without modifying files.",
    )
    return parser.parse_args()


def resolve_paths(args: argparse.Namespace) -> tuple[Path, Path, str | None]:
    """Resolve station config and output directory from CLI/profile inputs."""

    if args.profile:
        profile_file, profile_dir, profile = load_site_profile(args.profile)
        config_path = Path(args.config).resolve() if args.config else station_config_path(profile_dir, profile)
        output_dir = Path(args.output_dir).resolve() if args.output_dir else calibration_dir_from_profile(profile_dir, profile)
        return config_path, output_dir, str(profile_file)
    config_path = Path(args.config or (PACKAGE_DIR / "industrial_station_config.json")).resolve()
    output_dir = Path(args.output_dir or (PACKAGE_DIR / "calibration_records")).resolve()
    return config_path, output_dir, None


def main() -> int:
    args = parse_args()
    if args.samples <= 0:
        raise SystemExit("--samples must be positive")
    if args.interval_s < 0:
        raise SystemExit("--interval-s must be >= 0")

    try:
        config_path, output_dir, profile_file = resolve_paths(args)
    except SiteProfileError as exc:
        raise SystemExit(str(exc)) from exc
    config = load_config(config_path)
    stations = config.get("stations") or {}
    if args.station not in stations:
        raise SystemExit(f"unknown station {args.station!r}; choices: {', '.join(station_names(config))}")
    old_station = stations[args.station] or None
    if old_station is None and args.mode != "full":
        raise SystemExit(f"station {args.station!r} is blank; use --mode full for the first capture")

    import agibot_gdk

    gdk_inited = False
    try:
        result = agibot_gdk.gdk_init()
        gdk_res = getattr(agibot_gdk, "GDKRes", None)
        if gdk_res is not None and result not in (None, gdk_res.kSuccess):
            raise RuntimeError(f"GDK init failed: {result}")
        gdk_inited = True

        slam = agibot_gdk.Slam()
        map_manager = agibot_gdk.Map()
        time.sleep(0.5)

        current_map = map_manager.get_curr_map()
        current_map_id = getattr(current_map, "id", None)
        expected_map_id = config.get("map_id")
        if (
            expected_map_id is not None
            and current_map_id is not None
            and int(current_map_id) != int(expected_map_id)
            and not args.allow_map_mismatch
        ):
            raise RuntimeError(f"map id mismatch: current={current_map_id}, expected={expected_map_id}")

        poses = read_pose_samples(slam, samples=args.samples, interval_s=args.interval_s)
        captured_pose = poses[-1]
        stability = pose_stability(poses)
        moving = (
            stability["xy_span_m"] > args.max_xy_span_m
            or abs(stability["yaw_span_deg"]) > args.max_yaw_span_deg
        )
        if moving and not args.allow_moving:
            raise RuntimeError(
                "current pose is not stable enough for calibration: "
                f"xy_span_m={stability['xy_span_m']:.4f}, yaw_span_deg={stability['yaw_span_deg']:.3f}"
            )

        new_station = json.loads(json.dumps(old_station)) if old_station is not None else {}
        if args.mode in ("position-only", "full"):
            new_station["position"] = captured_pose["position"]
        if args.mode in ("yaw-only", "full"):
            new_station["orientation"] = captured_pose["orientation"]

        old_yaw = yaw_deg_from_quaternion(old_station["orientation"]) if old_station is not None else None
        captured_yaw = yaw_deg_from_quaternion(captured_pose["orientation"])
        new_yaw = yaw_deg_from_quaternion(new_station["orientation"])
        delta = {
            "xy_from_old_to_captured_m": None if old_station is None else planar_distance(old_station, captured_pose),
            "old_yaw_deg": old_yaw,
            "captured_yaw_deg": captured_yaw,
            "new_yaw_deg": new_yaw,
            "yaw_delta_old_to_new_deg": None if old_yaw is None else wrap_deg(new_yaw - old_yaw),
        }

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir.mkdir(parents=True, exist_ok=True)
        record_path = output_dir / f"map{expected_map_id}_{args.station.lower()}_calibration_{timestamp}.json"
        backup_path = config_path.with_name(f"{config_path.name}.bak_{args.station}_{timestamp}")

        record = {
            "schema": "g2_station_calibration_v1",
            "timestamp_local": timestamp,
            "station": args.station,
            "mode": args.mode,
            "map_id_expected": expected_map_id,
            "map_id_current": current_map_id,
            "config": str(config_path),
            "profile": profile_file,
            "backup_path": None if args.no_write else str(backup_path),
            "record_path": str(record_path),
            "note": args.note,
            "no_motion_commands_sent": True,
            "stability": stability,
            "old_station": old_station,
            "captured_pose": captured_pose,
            "new_station": new_station,
            "delta": delta,
        }

        if not args.no_write:
            backup_path.write_text(config_path.read_text(encoding="utf-8"), encoding="utf-8")
            config["stations"][args.station] = new_station
            write_json(config_path, config)
            write_json(record_path, record)
            if args.station == "GRAB_PRE":
                latest_path = output_dir / f"map{expected_map_id}_grab_target_latest.json"
                latest = {
                    "label": "GRAB_PRE",
                    "map_id": expected_map_id,
                    "source": "calibrate_station_from_current_pose",
                    "captured_at": timestamp,
                    "mode": args.mode,
                    "pose": {
                        **new_station,
                        "yaw_deg": new_yaw,
                    },
                    "captured_pose": {
                        **captured_pose,
                        "yaw_deg": captured_yaw,
                    },
                    "delta": delta,
                    "stability": stability,
                    "notes": args.note,
                }
                write_json(latest_path, latest)

        print(
            json.dumps(
                {
                    "event": "station_calibration_update",
                    "station": args.station,
                    "mode": args.mode,
                    "written": not args.no_write,
                    "profile": profile_file,
                    "config": str(config_path),
                    "backup_path": None if args.no_write else str(backup_path),
                    "record_path": str(record_path),
                    "map_id_current": current_map_id,
                    "delta": delta,
                    "stability": stability,
                    "new_station": new_station,
                },
                indent=2,
                ensure_ascii=False,
            ),
            flush=True,
        )
        return 0
    finally:
        if gdk_inited:
            try:
                agibot_gdk.gdk_release()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
