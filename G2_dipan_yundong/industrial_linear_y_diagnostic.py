#!/usr/bin/env python3
"""Small live diagnostic for G2 chassis linear.y motion.

This script is deliberately separate from the seven-rods controller. It verifies
whether move_chassis(Twist.linear.y) produces measurable lateral odom movement
before lateral rack-centering control is allowed in production.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
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


def selected_ultrasonic_rows(radar_data):
    rows = []
    for row in radar_data.get("ultrasonic_radar_datas", []):
        radar_id = row.get("id")
        try:
            radar_id = int(radar_id)
        except (TypeError, ValueError):
            continue
        if 0 <= radar_id <= 7:
            rows.append(
                {
                    "id": radar_id,
                    "distance_mm": row.get("distance_mm"),
                    "fault_state": row.get("fault_state"),
                }
            )
    rows.sort(key=lambda item: item["id"])
    return rows


def valid_distance_mm(value):
    try:
        distance = int(value)
    except (TypeError, ValueError):
        return None
    if distance <= 0 or distance >= INVALID_DISTANCE_MM:
        return None
    return distance


def group_distances(rows, ids):
    by_id = {row["id"]: row for row in rows}
    grouped = []
    values = []
    for radar_id in ids:
        row = by_id.get(radar_id)
        if row is None:
            grouped.append((radar_id, None, "missing"))
            continue
        distance = row.get("distance_mm")
        fault_state = row.get("fault_state")
        if fault_state != 0:
            grouped.append((radar_id, distance, f"fault={fault_state}"))
            continue
        distance_mm = valid_distance_mm(distance)
        if distance_mm is None:
            grouped.append((radar_id, distance, "invalid"))
            continue
        grouped.append((radar_id, distance_mm, "ok"))
        values.append(distance_mm)
    return min(values) if values else None, tuple(grouped)


def read_ultrasonic_groups(radar):
    rows = selected_ultrasonic_rows(radar.get_latest_ultrasonic_radar())
    groups = {}
    for name, ids in ULTRASONIC_GROUPS:
        min_mm, grouped = group_distances(rows, ids)
        groups[name] = {"min_mm": min_mm, "rows": grouped}
    return {
        "rows": tuple((row["id"], row["distance_mm"], row["fault_state"]) for row in rows),
        "groups": groups,
    }


def median(values):
    ordered = sorted(values)
    if not ordered:
        return None
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def read_clearance_window(radar, samples: int, interval_s: float):
    snapshots = []
    values = {name: [] for name, _ in ULTRASONIC_GROUPS}
    for index in range(samples):
        snapshot = read_ultrasonic_groups(radar)
        snapshots.append(snapshot)
        for name in values:
            min_mm = snapshot["groups"][name]["min_mm"]
            if min_mm is not None:
                values[name].append(min_mm)
        if index + 1 < samples:
            time.sleep(interval_s)
    summary = {name: median(group_values) for name, group_values in values.items()}
    return summary, snapshots


def yaw_rad_from_odom(odom):
    orientation_euler = getattr(odom, "orientation_euler", None)
    if orientation_euler is not None:
        for attr in ("z", "yaw"):
            if hasattr(orientation_euler, attr):
                return float(getattr(orientation_euler, attr))
        try:
            return float(orientation_euler[2])
        except (TypeError, ValueError, IndexError):
            pass
    match = re.search(r"orientation_euler=\(([^)]*)\)", repr(odom))
    if match:
        parts = [part.strip() for part in match.group(1).split(",")]
        if len(parts) >= 3:
            try:
                return float(parts[2])
            except ValueError:
                pass
    return None


def xy_from_odom(odom):
    pose = getattr(odom, "pose", None)
    if pose is not None and hasattr(pose, "x") and hasattr(pose, "y"):
        return float(getattr(pose, "x")), float(getattr(pose, "y"))
    if pose is not None:
        try:
            return float(pose[0]), float(pose[1])
        except (TypeError, ValueError, IndexError):
            pass
    match = re.search(r"pose=\(([^)]*)\)", repr(odom))
    if match:
        parts = [part.strip() for part in match.group(1).split(",")]
        if len(parts) >= 2:
            try:
                return float(parts[0]), float(parts[1])
            except ValueError:
                pass
    return None


def read_odom_pose(slam, label: str):
    odom = slam.get_odom_info()
    xy = xy_from_odom(odom)
    yaw_rad = yaw_rad_from_odom(odom)
    if xy is None or yaw_rad is None:
        raise RuntimeError(f"{label}: odom xy/yaw unavailable: {odom!r}")
    velocity_body = getattr(odom, "velocity_body", None)
    vx = float(getattr(velocity_body, "x", 0.0)) if velocity_body is not None else None
    vy = float(getattr(velocity_body, "y", 0.0)) if velocity_body is not None else None
    return {
        "x": xy[0],
        "y": xy[1],
        "yaw_rad": yaw_rad,
        "yaw_deg": math.degrees(yaw_rad),
        "velocity_body_x": vx,
        "velocity_body_y": vy,
        "loc_confidence": getattr(odom, "loc_confidence", None),
        "loc_state": getattr(odom, "loc_state", None),
        "repr": repr(odom),
    }


def body_frame_delta(start_pose, end_pose):
    dx = end_pose["x"] - start_pose["x"]
    dy = end_pose["y"] - start_pose["y"]
    yaw = start_pose["yaw_rad"]
    forward_m = dx * math.cos(yaw) + dy * math.sin(yaw)
    lateral_m = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return {
        "world_dx_m": dx,
        "world_dy_m": dy,
        "body_forward_m": forward_m,
        "body_lateral_m": lateral_m,
        "yaw_delta_deg": end_pose["yaw_deg"] - start_pose["yaw_deg"],
    }


def make_twist(agibot_gdk, vy_mps: float):
    twist = agibot_gdk.Twist()
    twist.linear = agibot_gdk.Vector3()
    twist.angular = agibot_gdk.Vector3()
    twist.linear.x = 0.0
    twist.linear.y = float(vy_mps)
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    return twist


def stop_chassis(pnc, agibot_gdk, repeats: int = 12):
    stop = make_twist(agibot_gdk, 0.0)
    for _ in range(repeats):
        try:
            pnc.move_chassis(stop)
        except Exception:
            pass
        time.sleep(0.03)


def cancel_blocking_task(pnc, label: str):
    try:
        task = pnc.get_task_state()
    except Exception as exc:
        print(f"cancel_blocking_task_read_failed label={label} error={exc}", flush=True)
        return
    state = getattr(task, "state", None)
    task_id = getattr(task, "id", None)
    task_type = getattr(task, "type", None)
    print(
        f"cancel_blocking_task_check label={label} state={state} "
        f"id={task_id} type={task_type}",
        flush=True,
    )
    if task_id is None or state in (0, 3, 6, 7, 8, 9):
        return
    pnc.cancel_task(task_id)
    print(f"cancel_blocking_task_done label={label} id={task_id} state={state}", flush=True)
    time.sleep(0.5)


def request_control_with_retry(pnc, mode: int, retries: int, wait_s: float):
    last_error = None
    cancel_blocking_task(pnc, "before_request_chassis_control")
    for attempt in range(1, retries + 1):
        try:
            result = pnc.request_chassis_control(mode)
            print(f"request_chassis_control attempt={attempt} mode={mode} result={result}", flush=True)
            return result
        except Exception as exc:
            last_error = exc
            print(f"request_chassis_control_failed attempt={attempt} error={exc}", flush=True)
            cancel_blocking_task(pnc, f"after_request_fail_{attempt}")
            time.sleep(wait_s)
    raise RuntimeError(f"RequestChassisControl failed after retries: {last_error}")


def check_robot_preflight(robot, args):
    power = robot.get_chassis_power_state()
    motion = read_motion_control_status_with_retry(robot)
    problems = []
    warnings = []
    motion_error = getattr(motion, "error_code", 0)
    charge_plug = getattr(power, "charge_plug_insert_state", 0)
    estop_state = getattr(power, "emergency_stop_pedal_state", 0)
    estop_fault = getattr(power, "emergency_stop_pedal_fault_state", 0)
    ultrasonic_power = getattr(power, "chassis_ultrasonic_radar_power_state", 0)
    if motion_error != 0:
        problems.append(f"motion_control_error={motion_error}")
    if charge_plug != 0:
        problems.append("charge_plug_insert_state=1")
    if estop_state != 0:
        problems.append("emergency_stop_pedal_state!=0")
    if estop_fault != 0 and not args.allow_estop_pedal_fault:
        problems.append("emergency_stop_pedal_fault_state!=0")
    elif estop_fault != 0:
        warnings.append("emergency_stop_pedal_fault_state=1 allowed")
    if ultrasonic_power != 1:
        problems.append("chassis_ultrasonic_radar_power_state!=1")
    print(
        "linear_y_preflight "
        f"motion_error={motion_error} charge_plug_insert_state={charge_plug} "
        f"emergency_stop_pedal_state={estop_state} "
        f"emergency_stop_pedal_fault_state={estop_fault} "
        f"chassis_ultrasonic_radar_power_state={ultrasonic_power} "
        f"warnings={tuple(warnings)} problems={tuple(problems)}",
        flush=True,
    )
    if problems:
        raise RuntimeError("linear y preflight blocked: " + ", ".join(problems))


def check_clearance(summary, args, label: str):
    thresholds = {
        "front": args.min_front_rear_clearance_mm,
        "rear": args.min_front_rear_clearance_mm,
        "left": args.min_side_clearance_mm,
        "right": args.min_side_clearance_mm,
    }
    problems = []
    for name, threshold in thresholds.items():
        value = summary.get(name)
        if value is None:
            problems.append(f"{name}_clearance_unavailable")
        elif value < threshold:
            problems.append(f"{name}_clearance_mm={value}<min_{threshold}")
    print(f"{label}_clearance_median_mm={summary} problems={tuple(problems)}", flush=True)
    if problems:
        raise RuntimeError(f"{label} clearance blocked: " + ", ".join(problems))


def parse_sequence(text: str):
    if text == "positive":
        return [1.0]
    if text == "negative":
        return [-1.0]
    if text == "positive-negative":
        return [1.0, -1.0]
    if text == "negative-positive":
        return [-1.0, 1.0]
    raise ValueError(f"invalid sequence: {text}")


def run(args):
    if args.dry_run:
        print(f"dry_run args={args}", flush=True)
        return
    if not args.confirm_live:
        raise RuntimeError("live linear.y diagnostic requires --confirm-live")

    import agibot_gdk

    result = agibot_gdk.gdk_init()
    gdk_res = getattr(agibot_gdk, "GDKRes", None)
    if gdk_res is not None and result not in (None, gdk_res.kSuccess):
        raise RuntimeError(f"GDK init failed: {result}")

    radar = None
    pnc = None
    report = {
        "status": "started",
        "speed_mps": args.speed_mps,
        "duration_s": args.duration_s,
        "sequence": args.sequence,
        "legs": [],
    }
    run_error = None
    try:
        robot = agibot_gdk.Robot()
        pnc = agibot_gdk.Pnc()
        slam = agibot_gdk.Slam()
        radar = agibot_gdk.UltrasonicRadar()
        time.sleep(args.init_wait_s)

        check_robot_preflight(robot, args)
        initial_clearance, initial_samples = read_clearance_window(
            radar, args.clearance_samples, args.clearance_interval_s
        )
        check_clearance(initial_clearance, args, "initial")
        report["initial_clearance_median_mm"] = initial_clearance
        report["initial_clearance_samples"] = initial_samples

        initial_pose = read_odom_pose(slam, "initial")
        report["initial_pose"] = initial_pose
        print(
            "linear_y_initial_pose "
            f"x={initial_pose['x']:.4f} y={initial_pose['y']:.4f} "
            f"yaw_deg={initial_pose['yaw_deg']:.3f}",
            flush=True,
        )

        request_control_with_retry(pnc, args.mode, args.retries, args.retry_wait_s)
        interval_s = 1.0 / args.hz
        command_count = max(1, math.ceil(args.duration_s * args.hz))
        current_pose = initial_pose
        for leg_index, sign in enumerate(parse_sequence(args.sequence), 1):
            leg_clearance, leg_clearance_samples = read_clearance_window(
                radar, args.clearance_samples, args.clearance_interval_s
            )
            check_clearance(leg_clearance, args, f"leg_{leg_index}_before")
            start_pose = read_odom_pose(slam, f"leg_{leg_index}_start")
            vy_mps = sign * args.speed_mps
            twist = make_twist(agibot_gdk, vy_mps)
            print(
                f"linear_y_leg_start index={leg_index} vy_mps={vy_mps:.4f} "
                f"duration_s={args.duration_s:.2f} commands={command_count}",
                flush=True,
            )
            for _ in range(command_count):
                pnc.move_chassis(twist)
                time.sleep(interval_s)
            stop_chassis(pnc, agibot_gdk)
            time.sleep(args.settle_s)
            end_pose = read_odom_pose(slam, f"leg_{leg_index}_end")
            delta = body_frame_delta(start_pose, end_pose)
            current_pose = end_pose
            leg_report = {
                "index": leg_index,
                "vy_mps": vy_mps,
                "duration_s": args.duration_s,
                "start_pose": start_pose,
                "end_pose": end_pose,
                "delta": delta,
                "clearance_median_mm": leg_clearance,
                "clearance_samples": leg_clearance_samples,
            }
            report["legs"].append(leg_report)
            print(
                "linear_y_leg_result "
                f"index={leg_index} vy_mps={vy_mps:.4f} "
                f"world_dx_m={delta['world_dx_m']:.4f} "
                f"world_dy_m={delta['world_dy_m']:.4f} "
                f"body_forward_m={delta['body_forward_m']:.4f} "
                f"body_lateral_m={delta['body_lateral_m']:.4f} "
                f"yaw_delta_deg={delta['yaw_delta_deg']:.3f}",
                flush=True,
            )
            if leg_index < len(parse_sequence(args.sequence)):
                time.sleep(args.leg_pause_s)

        total_delta = body_frame_delta(initial_pose, current_pose)
        report["final_pose"] = current_pose
        report["total_delta"] = total_delta
        report["measured_lateral_abs_max_m"] = max(
            abs(leg["delta"]["body_lateral_m"]) for leg in report["legs"]
        )
        report["passes_min_displacement"] = (
            report["measured_lateral_abs_max_m"] >= args.min_expected_lateral_m
        )
        print(
            "linear_y_validation_summary "
            f"measured_lateral_abs_max_m={report['measured_lateral_abs_max_m']:.4f} "
            f"min_expected_lateral_m={args.min_expected_lateral_m:.4f} "
            f"passes_min_displacement={report['passes_min_displacement']} "
            f"total_body_forward_m={total_delta['body_forward_m']:.4f} "
            f"total_body_lateral_m={total_delta['body_lateral_m']:.4f}",
            flush=True,
        )
        report["status"] = "completed"
    except Exception as exc:
        run_error = exc
        report["status"] = "failed"
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if pnc is not None:
            stop_chassis(pnc, agibot_gdk)
            try:
                cancel_blocking_task(pnc, "final_cleanup")
            except Exception as exc:
                print(f"final_cleanup_cancel_failed error={exc}", flush=True)
        if radar is not None:
            try:
                radar.close_ultrasonic_radar()
            except Exception:
                pass
        try:
            agibot_gdk.gdk_release()
        except Exception:
            pass
        if args.report_json:
            try:
                output = Path(args.report_json)
                output.parent.mkdir(parents=True, exist_ok=True)
                with output.open("w", encoding="utf-8") as f:
                    json.dump(report, f, ensure_ascii=False, indent=2, sort_keys=True)
                    f.write("\n")
                print(f"linear_y_report_json={output}", flush=True)
            except Exception as report_exc:
                print(f"linear_y_report_json_failed error={report_exc}", flush=True)
        if run_error is not None:
            print(f"linear_y_diagnostic_failed error={run_error}", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(description="G2 linear.y chassis diagnostic")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--confirm-live", action="store_true")
    parser.add_argument("--mode", type=int, default=0)
    parser.add_argument("--speed-mps", type=float, default=0.05)
    parser.add_argument("--duration-s", type=float, default=1.0)
    parser.add_argument(
        "--sequence",
        choices=("positive", "negative", "positive-negative", "negative-positive"),
        default="positive-negative",
    )
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--settle-s", type=float, default=0.8)
    parser.add_argument("--leg-pause-s", type=float, default=0.8)
    parser.add_argument("--init-wait-s", type=float, default=0.8)
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument("--retry-wait-s", type=float, default=0.8)
    parser.add_argument("--clearance-samples", type=int, default=5)
    parser.add_argument("--clearance-interval-s", type=float, default=0.12)
    parser.add_argument("--min-side-clearance-mm", type=int, default=500)
    parser.add_argument("--min-front-rear-clearance-mm", type=int, default=450)
    parser.add_argument("--min-expected-lateral-m", type=float, default=0.02)
    parser.add_argument("--allow-estop-pedal-fault", action="store_true", default=True)
    parser.add_argument("--strict-estop-pedal-fault", dest="allow_estop_pedal_fault", action="store_false")
    parser.add_argument("--report-json", default=None)
    args = parser.parse_args()

    if args.dry_run and args.confirm_live:
        raise SystemExit("--dry-run and --confirm-live cannot be used together")
    if args.speed_mps <= 0.0 or args.speed_mps > 0.12:
        raise SystemExit("--speed-mps must be in (0, 0.12]")
    if args.duration_s <= 0.0 or args.duration_s > 3.0:
        raise SystemExit("--duration-s must be in (0, 3]")
    if args.hz <= 0.0:
        raise SystemExit("--hz must be positive")
    if args.clearance_samples <= 0:
        raise SystemExit("--clearance-samples must be positive")
    if args.clearance_interval_s < 0.0:
        raise SystemExit("--clearance-interval-s must be >= 0")
    if args.min_side_clearance_mm <= 0 or args.min_front_rear_clearance_mm <= 0:
        raise SystemExit("clearance thresholds must be positive")
    if args.min_expected_lateral_m <= 0.0:
        raise SystemExit("--min-expected-lateral-m must be positive")
    return args


if __name__ == "__main__":
    run(parse_args())
