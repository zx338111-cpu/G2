#!/usr/bin/env python3
"""
Read-only G2 chassis and ultrasonic status snapshot.

This script is intentionally diagnostic-only. It does not call
request_chassis_control(), move_chassis(), relative_move(), or cancel_task().

Why the seven-rods runner calls this before any live phase:

- It captures the robot-side truth for charge state, motion-control error,
  PNC task state, whole-body error fields, ultrasonic readings, and odometry.
- It writes plain text that is easy to inspect in the run log if preflight later
  blocks motion.
- It intentionally samples odom more than once. A single missing or stale odom
  frame is not enough to prove the robot is stopped.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time


INVALID_DISTANCE_MM = 65535
ULTRASONIC_GROUPS = (
    ("front", (0, 1)),
    ("right", (2, 3)),
    ("rear", (4, 5)),
    ("left", (6, 7)),
)

PACKAGE_DIR = Path(__file__).resolve().parent / "rack_hybrid_docking_package"
if PACKAGE_DIR.exists() and str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from gdk_status_utils import read_motion_control_status_with_retry


def public_fields(obj):
    """Extract simple public fields from a GDK object for log-friendly output."""

    fields = {}
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
        if isinstance(value, (int, float, str, bool, type(None))):
            fields[name] = value
    return fields


def fmt_fields(label, obj):
    """Format a GDK object as both parsed fields and raw repr."""

    try:
        fields = public_fields(obj)
    except Exception as exc:
        return f"{label}_error={type(exc).__name__}: {exc}"
    return f"{label}_fields={fields} {label}_repr={obj!r}"


def selected_ultrasonic_rows(radar_data):
    """Normalize GDK ultrasonic rows and keep only physical sensor IDs 0-7."""

    rows = []
    for row in radar_data.get("ultrasonic_radar_datas", []):
        radar_id = row.get("id")
        try:
            radar_id = int(radar_id)
        except (TypeError, ValueError):
            continue
        if not (0 <= radar_id <= 7):
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


def group_distances(rows, ids):
    """Return a min valid distance plus per-sensor status for one sensor group."""

    by_id = {row["id"]: row for row in rows}
    result = []
    valid_values = []
    for radar_id in ids:
        row = by_id.get(radar_id)
        if row is None:
            result.append((radar_id, None, "missing"))
            continue
        distance = row.get("distance_mm")
        fault_state = row.get("fault_state")
        if fault_state != 0:
            result.append((radar_id, distance, f"fault={fault_state}"))
            continue
        try:
            distance = int(distance)
        except (TypeError, ValueError):
            result.append((radar_id, distance, "invalid"))
            continue
        if distance >= INVALID_DISTANCE_MM or distance <= 0:
            result.append((radar_id, distance, "invalid"))
            continue
        result.append((radar_id, distance, "ok"))
        valid_values.append(distance)
    min_mm = min(valid_values) if valid_values else None
    return min_mm, tuple(result)


def read_ultrasonic(radar):
    """Print all ultrasonic rows and grouped front/right/rear/left summaries."""

    rows = selected_ultrasonic_rows(radar.get_latest_ultrasonic_radar())
    print(f"ultrasonic_all={tuple((r['id'], r['distance_mm'], r['fault_state']) for r in rows)}", flush=True)
    for name, ids in ULTRASONIC_GROUPS:
        min_mm, grouped = group_distances(rows, ids)
        print(f"ultrasonic_group name={name} ids={ids} min_mm={min_mm} rows={grouped}", flush=True)


def read_task(pnc):
    """Read and print the current PNC task state without cancelling anything."""

    try:
        task = pnc.get_task_state()
    except Exception as exc:
        print(f"task_state_error={type(exc).__name__}: {exc}", flush=True)
        return None
    print(fmt_fields("task_state", task), flush=True)
    return task


def read_robot_status(robot):
    """Read chassis, motion-control, and whole-body status without side effects."""

    for label, getter_name in (
        ("chassis_power", "get_chassis_power_state"),
        ("motion_control", "get_motion_control_status"),
        ("whole_body", "get_whole_body_status"),
    ):
        try:
            if label == "motion_control":
                value = read_motion_control_status_with_retry(robot)
            else:
                value = getattr(robot, getter_name)()
        except Exception as exc:
            print(f"{label}_error={type(exc).__name__}: {exc}", flush=True)
            continue
        print(fmt_fields(label, value), flush=True)


def read_odom_speed(slam):
    """Read odometry velocity and return scalar speed, or None if unavailable."""

    try:
        odom = slam.get_odom_info()
    except Exception as exc:
        print(f"odom_error={type(exc).__name__}: {exc}", flush=True)
        return None

    velocity_body = getattr(odom, "velocity_body", None)
    if velocity_body is None:
        print(fmt_fields("odom", odom), flush=True)
        return None

    vx = float(getattr(velocity_body, "x", 0.0))
    vy = float(getattr(velocity_body, "y", 0.0))
    vz = float(getattr(velocity_body, "z", 0.0))
    linear = math.sqrt(vx * vx + vy * vy + vz * vz)
    print(
        "odom_velocity "
        f"vx={vx:.4f} vy={vy:.4f} vz={vz:.4f} linear={linear:.4f} "
        f"odom_repr={odom!r}",
        flush=True,
    )
    return linear


def main():
    """CLI entrypoint for a read-only status snapshot."""

    parser = argparse.ArgumentParser(description="Read-only G2 status snapshot")
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--interval-s", type=float, default=0.2)
    parser.add_argument("--stopped-speed-threshold", type=float, default=0.02)
    args = parser.parse_args()

    if args.samples <= 0:
        raise SystemExit("--samples must be positive")
    if args.interval_s < 0.0:
        raise SystemExit("--interval-s must be >= 0")

    import agibot_gdk

    result = agibot_gdk.gdk_init()
    gdk_res = getattr(agibot_gdk, "GDKRes", None)
    if gdk_res is not None and result not in (None, gdk_res.kSuccess):
        raise RuntimeError(f"GDK init failed: {result}")

    radar = agibot_gdk.UltrasonicRadar()
    robot = agibot_gdk.Robot()
    pnc = agibot_gdk.Pnc()
    slam = agibot_gdk.Slam()

    speeds = []
    try:
        time.sleep(0.8)
        print("status_snapshot_start read_only=true", flush=True)
        read_robot_status(robot)
        read_task(pnc)
        for index in range(args.samples):
            print(f"sample={index + 1}/{args.samples}", flush=True)
            read_ultrasonic(radar)
            speed = read_odom_speed(slam)
            if speed is not None:
                speeds.append(speed)
            if index + 1 < args.samples:
                time.sleep(args.interval_s)

        if speeds:
            max_speed = max(speeds)
            print(
                "stopped_check "
                f"odom_available=true max_linear_speed_mps={max_speed:.4f} "
                f"threshold_mps={args.stopped_speed_threshold:.4f} "
                f"stopped={max_speed <= args.stopped_speed_threshold}",
                flush=True,
            )
        else:
            print(
                "stopped_check "
                "odom_available=false stopped=unknown "
                "reason=no usable odom velocity samples",
                flush=True,
            )
        print("status_snapshot_done", flush=True)
    finally:
        try:
            radar.close_ultrasonic_radar()
        except Exception:
            pass
        try:
            agibot_gdk.gdk_release()
        except Exception:
            pass


if __name__ == "__main__":
    main()
