#!/usr/bin/env python3
"""Trace rack pose and odom around a small linear.y chassis motion.

This diagnostic is intentionally separate from production centering. It records
front-lidar rack pose, selected ROI/bin, odom pose, and ultrasonic clearance
before/during/after a controlled lateral command so we can see whether active
centering failures come from chassis motion, pose ROI jumps, or timing drift.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime
import json
import math
from pathlib import Path
import re
import statistics
import time
from typing import Any

from gdk_status_utils import read_motion_control_status_with_retry
from industrial_rack_pose_roi_sweep import RoiConfig, read_rack_pose


INVALID_DISTANCE_MM = 65535
ULTRASONIC_GROUPS = (
    ("front", (0, 1)),
    ("right", (2, 3)),
    ("rear", (4, 5)),
    ("left", (6, 7)),
)


def median(values: list[float]) -> float | None:
    return float(statistics.median(values)) if values else None


def round_or_none(value: float | None, digits: int = 4) -> float | None:
    return None if value is None else round(value, digits)


def lateral_sample_stats(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {
            "span_m": None,
            "robust_span_m": None,
            "mad_m": None,
            "trim_count": 0,
        }
    ordered = sorted(float(value) for value in values)
    sample_median = float(statistics.median(ordered))
    trim_count = 0
    if len(ordered) >= 5:
        trim_count = max(1, int(len(ordered) * 0.10))
        if len(ordered) - 2 * trim_count < 3:
            trim_count = 0
    stable_values = ordered[trim_count : len(ordered) - trim_count] if trim_count else ordered
    return {
        "span_m": ordered[-1] - ordered[0],
        "robust_span_m": stable_values[-1] - stable_values[0],
        "mad_m": float(statistics.median([abs(value - sample_median) for value in ordered])),
        "trim_count": trim_count,
    }


def parse_sequence(text: str) -> list[float]:
    if text == "positive":
        return [1.0]
    if text == "negative":
        return [-1.0]
    if text == "positive-negative":
        return [1.0, -1.0]
    if text == "negative-positive":
        return [-1.0, 1.0]
    raise ValueError(f"unsupported sequence: {text}")


def valid_distance_mm(value) -> int | None:
    try:
        distance = int(value)
    except (TypeError, ValueError):
        return None
    if distance <= 0 or distance >= INVALID_DISTANCE_MM:
        return None
    return distance


def read_ultrasonic_groups(radar) -> dict[str, Any]:
    data = radar.get_latest_ultrasonic_radar()
    rows = []
    for row in data.get("ultrasonic_radar_datas", []):
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
    by_id = {row["id"]: row for row in rows}
    groups = {}
    for name, ids in ULTRASONIC_GROUPS:
        group_rows = []
        valid = []
        for radar_id in ids:
            row = by_id.get(radar_id)
            if row is None:
                group_rows.append((radar_id, None, "missing"))
                continue
            distance = row.get("distance_mm")
            fault_state = row.get("fault_state")
            if fault_state != 0:
                group_rows.append((radar_id, distance, f"fault={fault_state}"))
                continue
            distance_mm = valid_distance_mm(distance)
            if distance_mm is None:
                group_rows.append((radar_id, distance, "invalid"))
                continue
            group_rows.append((radar_id, distance_mm, "ok"))
            valid.append(distance_mm)
        groups[name] = {"min_mm": min(valid) if valid else None, "rows": group_rows}
    return {
        "rows": [(row["id"], row["distance_mm"], row["fault_state"]) for row in rows],
        "groups": groups,
    }


def read_clearance_window(radar, samples: int, interval_s: float) -> dict[str, Any]:
    values = {name: [] for name, _ in ULTRASONIC_GROUPS}
    snapshots = []
    for index in range(samples):
        snapshot = read_ultrasonic_groups(radar)
        snapshots.append(snapshot)
        for name in values:
            min_mm = snapshot["groups"][name]["min_mm"]
            if min_mm is not None:
                values[name].append(float(min_mm))
        if index + 1 < samples:
            time.sleep(interval_s)
    return {
        "median_mm": {name: median(group_values) for name, group_values in values.items()},
        "min_mm": {
            name: min(group_values) if group_values else None
            for name, group_values in values.items()
        },
        "snapshots": snapshots,
    }


def check_clearance(clearance: dict[str, Any], args, label: str):
    medians = clearance["median_mm"]
    minimums = clearance["min_mm"]
    thresholds = {
        "front": args.min_front_rear_clearance_mm,
        "rear": args.min_front_rear_clearance_mm,
        "left": args.min_side_clearance_mm,
        "right": args.min_side_clearance_mm,
    }
    hard_thresholds = {
        "front": args.hard_min_front_rear_clearance_mm,
        "rear": args.hard_min_front_rear_clearance_mm,
        "left": args.hard_min_side_clearance_mm,
        "right": args.hard_min_side_clearance_mm,
    }
    problems = []
    for name, threshold in thresholds.items():
        value = medians.get(name)
        if value is None:
            problems.append(f"{name}_clearance_unavailable")
        elif value < threshold:
            problems.append(f"{name}_clearance_mm={value}<min_{threshold}")
        min_value = minimums.get(name)
        hard_threshold = hard_thresholds[name]
        if min_value is None:
            problems.append(f"{name}_clearance_min_unavailable")
        elif min_value < hard_threshold:
            problems.append(f"{name}_clearance_raw_min_mm={min_value}<hard_min_{hard_threshold}")
    print(
        f"{label}_clearance_median_mm={medians} "
        f"raw_min_mm={minimums} problems={tuple(problems)}",
        flush=True,
    )
    if problems:
        raise RuntimeError(f"{label} clearance blocked: " + ", ".join(problems))


def yaw_rad_from_odom(odom) -> float | None:
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


def xy_from_odom(odom) -> tuple[float, float] | None:
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


def read_odom_pose(slam) -> dict[str, Any] | None:
    try:
        odom = slam.get_odom_info()
    except Exception as exc:
        return {"status": "error", "error": f"{type(exc).__name__}: {exc}"}
    xy = xy_from_odom(odom)
    yaw_rad = yaw_rad_from_odom(odom)
    if xy is None or yaw_rad is None:
        return {"status": "unavailable", "repr": repr(odom)}
    return {
        "status": "ok",
        "x": xy[0],
        "y": xy[1],
        "yaw_rad": yaw_rad,
        "yaw_deg": math.degrees(yaw_rad),
        "loc_confidence": getattr(odom, "loc_confidence", None),
        "loc_state": getattr(odom, "loc_state", None),
        "repr": repr(odom),
    }


def body_frame_delta(start_pose: dict[str, Any], end_pose: dict[str, Any]) -> dict[str, float] | None:
    if start_pose.get("status") != "ok" or end_pose.get("status") != "ok":
        return None
    dx = float(end_pose["x"]) - float(start_pose["x"])
    dy = float(end_pose["y"]) - float(start_pose["y"])
    yaw = float(start_pose["yaw_rad"])
    forward_m = dx * math.cos(yaw) + dy * math.sin(yaw)
    lateral_m = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return {
        "world_dx_m": dx,
        "world_dy_m": dy,
        "body_forward_m": forward_m,
        "body_lateral_m": lateral_m,
        "yaw_delta_deg": float(end_pose["yaw_deg"]) - float(start_pose["yaw_deg"]),
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
    if pnc is None:
        return
    twist = make_twist(agibot_gdk, 0.0)
    for _ in range(repeats):
        try:
            pnc.move_chassis(twist)
        except Exception:
            pass
        time.sleep(0.03)


def cancel_blocking_task(pnc, label: str):
    task = pnc.get_task_state()
    state = getattr(task, "state", None)
    task_id = getattr(task, "id", None)
    print(f"cancel_task_check label={label} id={task_id} state={state}", flush=True)
    if task_id is None or state in (0, 3, 6, 7, 8, 9):
        return
    pnc.cancel_task(task_id)
    time.sleep(0.5)


def request_chassis_control(pnc, mode: int):
    cancel_blocking_task(pnc, "before_request")
    try:
        result = pnc.request_chassis_control(mode)
        print(f"request_chassis_control mode={mode} result={result}", flush=True)
        return result
    except Exception:
        cancel_blocking_task(pnc, "after_request_fail")
        time.sleep(0.5)
        result = pnc.request_chassis_control(mode)
        print(f"request_chassis_control_retry mode={mode} result={result}", flush=True)
        return result


def check_robot_preflight(robot, args):
    power = robot.get_chassis_power_state()
    motion = read_motion_control_status_with_retry(robot)
    motion_error = getattr(motion, "error_code", 0)
    charge_plug = getattr(power, "charge_plug_insert_state", 0)
    estop_state = getattr(power, "emergency_stop_pedal_state", 0)
    estop_fault = getattr(power, "emergency_stop_pedal_fault_state", 0)
    ultrasonic_power = getattr(power, "chassis_ultrasonic_radar_power_state", 0)
    problems = []
    warnings = []
    if motion_error != 0:
        problems.append(f"motion_control_error={motion_error}")
    if charge_plug != 0 and args.read_only:
        warnings.append("charge_plug_insert_state=1 read_only allowed")
    elif charge_plug != 0:
        problems.append("charge_plug_insert_state=1")
    if estop_state != 0:
        problems.append("emergency_stop_pedal_state!=0")
    if estop_fault != 0 and not args.allow_estop_pedal_fault:
        problems.append("emergency_stop_pedal_fault_state=1")
    elif estop_fault != 0:
        warnings.append("emergency_stop_pedal_fault_state=1 allowed")
    if ultrasonic_power != 1:
        problems.append("chassis_ultrasonic_radar_power_state!=1")
    print(
        "trace_preflight "
        f"motion_error={motion_error} charge_plug_insert_state={charge_plug} "
        f"emergency_stop_pedal_state={estop_state} "
        f"emergency_stop_pedal_fault_state={estop_fault} "
        f"ultrasonic_power={ultrasonic_power} warnings={tuple(warnings)} "
        f"problems={tuple(problems)}",
        flush=True,
    )
    if problems:
        raise RuntimeError("trace preflight blocked: " + ", ".join(problems))


def sample_once(lidar, lidar_type, slam, config: RoiConfig, *, phase: str, t0: float, index: int):
    now = time.time()
    pose = None
    pose_error = None
    try:
        pose = read_rack_pose(lidar, lidar_type, config)
    except Exception as exc:
        pose_error = f"{type(exc).__name__}: {exc}"
    odom = read_odom_pose(slam)
    row = {
        "index": index,
        "phase": phase,
        "elapsed_s": now - t0,
        "wall_time": datetime.now().isoformat(timespec="milliseconds"),
        "rack_pose": pose,
        "rack_pose_error": pose_error,
        "odom": odom,
    }
    if pose is not None:
        print(
            "trace_sample "
            f"phase={phase} index={index} elapsed_s={row['elapsed_s']:.3f} "
            f"distance_m={pose.get('distance_m'):.3f} "
            f"lateral_center_m={pose.get('lateral_center_m'):.4f} "
            f"yaw_deg={pose.get('yaw_deg')} "
            f"bin={pose.get('bin_start_m'):.3f}-{pose.get('bin_end_m'):.3f} "
            f"cluster_points={pose.get('cluster_points')}",
            flush=True,
        )
    else:
        print(
            "trace_sample "
            f"phase={phase} index={index} elapsed_s={row['elapsed_s']:.3f} "
            f"pose_unavailable error={pose_error}",
            flush=True,
        )
    return row


def sample_phase(rows: list[dict[str, Any]], lidar, lidar_type, slam, config, *, phase: str, duration_s: float, sample_hz: float, t0: float):
    interval_s = 1.0 / sample_hz
    deadline = time.time() + duration_s
    while time.time() <= deadline:
        rows.append(
            sample_once(
                lidar,
                lidar_type,
                slam,
                config,
                phase=phase,
                t0=t0,
                index=len(rows) + 1,
            )
        )
        sleep_s = min(interval_s, max(0.0, deadline - time.time()))
        if sleep_s <= 0.0:
            break
        time.sleep(sleep_s)


def run_motion_phase(rows, lidar, lidar_type, slam, config, pnc, agibot_gdk, *, phase: str, vy_mps: float, duration_s: float, command_hz: float, sample_hz: float, t0: float):
    command_interval_s = 1.0 / command_hz
    sample_interval_s = 1.0 / sample_hz
    twist = make_twist(agibot_gdk, vy_mps)
    deadline = time.time() + duration_s
    next_sample = time.time()
    command_count = 0
    while time.time() <= deadline:
        pnc.move_chassis(twist)
        command_count += 1
        if time.time() >= next_sample:
            rows.append(
                sample_once(
                    lidar,
                    lidar_type,
                    slam,
                    config,
                    phase=phase,
                    t0=t0,
                    index=len(rows) + 1,
                )
            )
            next_sample += sample_interval_s
        time.sleep(command_interval_s)
    stop_chassis(pnc, agibot_gdk)
    print(
        f"trace_motion_phase_done phase={phase} vy_mps={vy_mps:.4f} "
        f"duration_s={duration_s:.3f} commands={command_count}",
        flush=True,
    )
    return command_count


def pose_values(rows: list[dict[str, Any]]) -> list[float]:
    values = []
    for row in rows:
        pose = row.get("rack_pose")
        if pose is not None and pose.get("lateral_center_m") is not None:
            values.append(float(pose["lateral_center_m"]))
    return values


def bin_counts(rows: list[dict[str, Any]]) -> dict[str, int]:
    counts = Counter()
    for row in rows:
        pose = row.get("rack_pose")
        if not pose:
            continue
        start = pose.get("bin_start_m")
        end = pose.get("bin_end_m")
        if start is not None and end is not None:
            counts[f"{float(start):.3f}-{float(end):.3f}"] += 1
    return dict(sorted(counts.items()))


def summarize_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    by_phase = {}
    for phase in sorted({row["phase"] for row in rows}):
        phase_rows = [row for row in rows if row["phase"] == phase]
        lateral = pose_values(phase_rows)
        stats = lateral_sample_stats(lateral)
        by_phase[phase] = {
            "count": len(phase_rows),
            "pose_valid_count": len(lateral),
            "lateral_center_m_median": round_or_none(median(lateral)),
            "lateral_sample_span_m": round_or_none(stats["span_m"]),
            "lateral_sample_robust_span_m": round_or_none(stats["robust_span_m"]),
            "lateral_sample_mad_m": round_or_none(stats["mad_m"]),
            "lateral_sample_trim_count": stats["trim_count"],
            "bin_counts": bin_counts(phase_rows),
        }
    return by_phase


def run(args):
    if args.dry_run:
        print(f"dry_run args={args}", flush=True)
        return
    if not args.read_only and not args.confirm_live:
        raise RuntimeError("motion trace requires --confirm-live unless --read-only is set")

    import agibot_gdk

    result = agibot_gdk.gdk_init()
    gdk_res = getattr(agibot_gdk, "GDKRes", None)
    if gdk_res is not None and result not in (None, gdk_res.kSuccess):
        raise RuntimeError(f"GDK init failed: {result}")

    lidar = None
    radar = None
    pnc = None
    rows = []
    report = {
        "status": "started",
        "args": vars(args),
        "rows": rows,
        "legs": [],
    }
    try:
        robot = agibot_gdk.Robot()
        slam = agibot_gdk.Slam()
        lidar = agibot_gdk.Lidar()
        lidar_type = agibot_gdk.LidarType.kLidarFront
        radar = agibot_gdk.UltrasonicRadar()
        if not args.read_only:
            pnc = agibot_gdk.Pnc()
        time.sleep(args.init_wait_s)

        check_robot_preflight(robot, args)
        clearance = read_clearance_window(
            radar,
            samples=args.clearance_samples,
            interval_s=args.clearance_interval_s,
        )
        check_clearance(clearance, args, "initial")
        report["initial_clearance"] = clearance

        roi_config = RoiConfig(
            name="trace_roi",
            min_range_m=args.rack_pose_min_range_m,
            max_range_m=args.rack_pose_max_range_m,
            lateral_half_width_m=args.rack_pose_lateral_half_width_m,
            z_min_m=args.rack_pose_z_min_m,
            z_max_m=args.rack_pose_z_max_m,
            bin_width_m=args.rack_pose_bin_width_m,
            min_cluster_points=args.rack_pose_min_cluster_points,
        )
        report["roi_config"] = asdict(roi_config)
        t0 = time.time()
        if args.read_only:
            sample_phase(
                rows,
                lidar,
                lidar_type,
                slam,
                roi_config,
                phase="read_only",
                duration_s=args.read_only_duration_s,
                sample_hz=args.sample_hz,
                t0=t0,
            )
        else:
            request_chassis_control(pnc, args.mode)
            for leg_index, sign in enumerate(parse_sequence(args.sequence), 1):
                vy_mps = sign * args.speed_mps
                pre_phase = f"leg_{leg_index}_pre"
                move_phase = f"leg_{leg_index}_move"
                post_phase = f"leg_{leg_index}_post"
                clearance = read_clearance_window(
                    radar,
                    samples=args.clearance_samples,
                    interval_s=args.clearance_interval_s,
                )
                check_clearance(clearance, args, f"leg_{leg_index}_before")
                start_odom = read_odom_pose(slam)
                sample_phase(
                    rows,
                    lidar,
                    lidar_type,
                    slam,
                    roi_config,
                    phase=pre_phase,
                    duration_s=args.pre_sample_s,
                    sample_hz=args.sample_hz,
                    t0=t0,
                )
                command_count = run_motion_phase(
                    rows,
                    lidar,
                    lidar_type,
                    slam,
                    roi_config,
                    pnc,
                    agibot_gdk,
                    phase=move_phase,
                    vy_mps=vy_mps,
                    duration_s=args.duration_s,
                    command_hz=args.command_hz,
                    sample_hz=args.sample_hz,
                    t0=t0,
                )
                time.sleep(args.settle_s)
                sample_phase(
                    rows,
                    lidar,
                    lidar_type,
                    slam,
                    roi_config,
                    phase=post_phase,
                    duration_s=args.post_sample_s,
                    sample_hz=args.sample_hz,
                    t0=t0,
                )
                end_odom = read_odom_pose(slam)
                delta = body_frame_delta(start_odom, end_odom)
                pre_values = pose_values([row for row in rows if row["phase"] == pre_phase])
                post_values = pose_values([row for row in rows if row["phase"] == post_phase])
                before_lateral = median(pre_values)
                after_lateral = median(post_values)
                improvement = (
                    abs(before_lateral) - abs(after_lateral)
                    if before_lateral is not None and after_lateral is not None
                    else None
                )
                leg_report = {
                    "leg_index": leg_index,
                    "vy_mps": vy_mps,
                    "command_count": command_count,
                    "start_odom": start_odom,
                    "end_odom": end_odom,
                    "odom_delta": delta,
                    "before_lateral_center_m_median": before_lateral,
                    "after_lateral_center_m_median": after_lateral,
                    "improvement_m": improvement,
                    "pre_bin_counts": bin_counts([row for row in rows if row["phase"] == pre_phase]),
                    "move_bin_counts": bin_counts([row for row in rows if row["phase"] == move_phase]),
                    "post_bin_counts": bin_counts([row for row in rows if row["phase"] == post_phase]),
                }
                report["legs"].append(leg_report)
                print(
                    "trace_leg_summary "
                    f"leg={leg_index} vy_mps={vy_mps:.4f} "
                    f"before_lateral={before_lateral} after_lateral={after_lateral} "
                    f"improvement_m={improvement} odom_delta={delta}",
                    flush=True,
                )
                if leg_index < len(parse_sequence(args.sequence)):
                    time.sleep(args.leg_pause_s)

        report["phase_summary"] = summarize_rows(rows)
        report["status"] = "completed"
    except Exception as exc:
        report["status"] = "failed"
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if pnc is not None:
            stop_chassis(pnc, agibot_gdk)
            try:
                cancel_blocking_task(pnc, "final_cleanup")
            except Exception as exc:
                print(f"final_cancel_failed error={exc}", flush=True)
        if lidar is not None:
            try:
                lidar.close_lidar()
            except Exception:
                pass
        if radar is not None:
            try:
                radar.close_ultrasonic_radar()
            except Exception:
                pass
        try:
            agibot_gdk.gdk_release()
        except Exception:
            pass
        write_outputs(report, args)


def render_markdown(report: dict[str, Any]) -> str:
    lines = [
        "# Lateral Motion Trace",
        "",
        f"- status: `{report.get('status')}`",
        f"- read_only: {report.get('args', {}).get('read_only')}",
        f"- sequence: `{report.get('args', {}).get('sequence')}`",
        f"- speed_mps: {report.get('args', {}).get('speed_mps')}",
        f"- duration_s: {report.get('args', {}).get('duration_s')}",
        "",
        "## Phase Summary",
        "",
    ]
    for phase, summary in (report.get("phase_summary") or {}).items():
        lines.append(
            f"- `{phase}`: count={summary['count']}, valid={summary['pose_valid_count']}, "
            f"lat_med={summary['lateral_center_m_median']}, "
            f"lat_span={summary['lateral_sample_span_m']}, "
            f"robust_span={summary.get('lateral_sample_robust_span_m')}, "
            f"mad={summary.get('lateral_sample_mad_m')}, bins={summary['bin_counts']}"
        )
    if report.get("legs"):
        lines.extend(["", "## Legs", ""])
        for leg in report["legs"]:
            lines.append(
                f"- leg={leg['leg_index']} vy_mps={leg['vy_mps']}: "
                f"before={round_or_none(leg['before_lateral_center_m_median'])}, "
                f"after={round_or_none(leg['after_lateral_center_m_median'])}, "
                f"improvement={round_or_none(leg['improvement_m'])}, "
                f"odom_delta={leg['odom_delta']}, "
                f"pre_bins={leg['pre_bin_counts']}, post_bins={leg['post_bin_counts']}"
            )
    if report.get("error"):
        lines.extend(["", "## Error", "", f"- {report['error']}"])
    return "\n".join(lines).rstrip() + "\n"


def write_outputs(report: dict[str, Any], args):
    if args.output_json:
        output = Path(args.output_json)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"trace_output_json={output}", flush=True)
    if args.output_jsonl:
        output = Path(args.output_jsonl)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="utf-8") as f:
            for row in report.get("rows", []):
                f.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
        print(f"trace_output_jsonl={output}", flush=True)
    markdown = render_markdown(report)
    if args.output_md:
        output = Path(args.output_md)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(markdown, encoding="utf-8")
        print(f"trace_output_md={output}", flush=True)
    print(markdown, end="")


def parse_args():
    parser = argparse.ArgumentParser(description="Trace rack pose and odom around lateral motion")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--read-only", action="store_true")
    parser.add_argument("--confirm-live", action="store_true")
    parser.add_argument("--mode", type=int, default=0)
    parser.add_argument("--sequence", choices=("positive", "negative", "positive-negative", "negative-positive"), default="positive-negative")
    parser.add_argument("--speed-mps", type=float, default=0.02)
    parser.add_argument("--duration-s", type=float, default=0.35)
    parser.add_argument("--command-hz", type=float, default=20.0)
    parser.add_argument("--sample-hz", type=float, default=8.0)
    parser.add_argument("--pre-sample-s", type=float, default=1.0)
    parser.add_argument("--post-sample-s", type=float, default=1.2)
    parser.add_argument("--read-only-duration-s", type=float, default=4.0)
    parser.add_argument("--settle-s", type=float, default=0.6)
    parser.add_argument("--leg-pause-s", type=float, default=0.8)
    parser.add_argument("--init-wait-s", type=float, default=0.8)
    parser.add_argument("--clearance-samples", type=int, default=5)
    parser.add_argument("--clearance-interval-s", type=float, default=0.12)
    parser.add_argument("--min-side-clearance-mm", type=int, default=650)
    parser.add_argument("--min-front-rear-clearance-mm", type=int, default=500)
    parser.add_argument("--hard-min-side-clearance-mm", type=int, default=450)
    parser.add_argument("--hard-min-front-rear-clearance-mm", type=int, default=350)
    parser.add_argument("--allow-estop-pedal-fault", action="store_true")
    parser.add_argument("--rack-pose-min-range-m", type=float, default=0.80)
    parser.add_argument("--rack-pose-max-range-m", type=float, default=1.40)
    parser.add_argument("--rack-pose-lateral-half-width-m", type=float, default=0.50)
    parser.add_argument("--rack-pose-z-min-m", type=float, default=0.60)
    parser.add_argument("--rack-pose-z-max-m", type=float, default=1.20)
    parser.add_argument("--rack-pose-bin-width-m", type=float, default=0.20)
    parser.add_argument("--rack-pose-min-cluster-points", type=int, default=20)
    parser.add_argument("--output-json", default=None)
    parser.add_argument("--output-jsonl", default=None)
    parser.add_argument("--output-md", default=None)
    args = parser.parse_args()

    if args.dry_run and args.confirm_live:
        raise SystemExit("--dry-run and --confirm-live cannot be used together")
    if args.read_only and args.confirm_live:
        raise SystemExit("--read-only and --confirm-live cannot be used together")
    if args.speed_mps <= 0.0 or args.speed_mps > 0.05:
        raise SystemExit("--speed-mps must be in (0, 0.05]")
    if args.duration_s <= 0.0 or args.duration_s > 1.0:
        raise SystemExit("--duration-s must be in (0, 1.0]")
    if args.command_hz <= 0.0 or args.sample_hz <= 0.0:
        raise SystemExit("command/sample hz must be positive")
    if args.min_side_clearance_mm <= 0 or args.min_front_rear_clearance_mm <= 0:
        raise SystemExit("clearance thresholds must be positive")
    if args.hard_min_side_clearance_mm <= 0 or args.hard_min_front_rear_clearance_mm <= 0:
        raise SystemExit("hard clearance thresholds must be positive")
    if args.hard_min_side_clearance_mm > args.min_side_clearance_mm:
        raise SystemExit("--hard-min-side-clearance-mm must be <= --min-side-clearance-mm")
    if args.hard_min_front_rear_clearance_mm > args.min_front_rear_clearance_mm:
        raise SystemExit("--hard-min-front-rear-clearance-mm must be <= --min-front-rear-clearance-mm")
    if args.pre_sample_s < 0.0 or args.post_sample_s < 0.0:
        raise SystemExit("sample durations must be >= 0")
    if args.read_only_duration_s <= 0.0:
        raise SystemExit("--read-only-duration-s must be positive")
    if args.rack_pose_min_range_m <= 0.0 or args.rack_pose_max_range_m <= args.rack_pose_min_range_m:
        raise SystemExit("invalid rack pose range")
    if args.rack_pose_lateral_half_width_m <= 0.0:
        raise SystemExit("--rack-pose-lateral-half-width-m must be positive")
    if args.rack_pose_z_min_m >= args.rack_pose_z_max_m:
        raise SystemExit("--rack-pose-z-min-m must be smaller than --rack-pose-z-max-m")
    if args.rack_pose_bin_width_m <= 0.0:
        raise SystemExit("--rack-pose-bin-width-m must be positive")
    if args.rack_pose_min_cluster_points <= 0:
        raise SystemExit("--rack-pose-min-cluster-points must be positive")
    return args


if __name__ == "__main__":
    run(parse_args())
