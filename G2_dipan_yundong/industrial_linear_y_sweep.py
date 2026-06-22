#!/usr/bin/env python3
"""Repeated linear.y sweep for G2 chassis lateral calibration.

This script keeps the calibration outside the seven-rods controller. It uses
the same safety gates as industrial_linear_y_diagnostic.py, then repeats small
positive/negative linear.y legs to estimate direction, gain, forward coupling,
and yaw drift before lateral rack centering is allowed to become active.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics
import sys
import time


from industrial_linear_y_diagnostic import (
    body_frame_delta,
    cancel_blocking_task,
    check_clearance,
    check_robot_preflight,
    make_twist,
    parse_sequence,
    read_clearance_window,
    read_odom_pose,
    request_control_with_retry,
    stop_chassis,
)


def parse_float_list(text: str) -> list[float]:
    values = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        values.append(float(item))
    if not values:
        raise ValueError("list must not be empty")
    return values


def median(values: list[float]) -> float | None:
    return float(statistics.median(values)) if values else None


def round_or_none(value: float | None, digits: int = 4) -> float | None:
    return None if value is None else round(value, digits)


def leg_plan(speed_mps_values: list[float], sequence: str, repeat_count: int):
    signs = parse_sequence(sequence)
    for repeat_index in range(1, repeat_count + 1):
        for speed_mps in speed_mps_values:
            for sign in signs:
                yield repeat_index, speed_mps, sign


def summarize_legs(legs: list[dict]) -> dict:
    if not legs:
        return {
            "leg_count": 0,
            "passes_min_displacement": False,
            "recommendation": "no_completed_legs",
        }

    lateral_abs = [abs(leg["delta"]["body_lateral_m"]) for leg in legs]
    forward_abs = [abs(leg["delta"]["body_forward_m"]) for leg in legs]
    yaw_abs = [abs(leg["delta"]["yaw_delta_deg"]) for leg in legs]
    gains = []
    grouped = defaultdict(list)
    for leg in legs:
        command_distance = abs(leg["vy_mps"]) * leg["duration_s"]
        if command_distance > 0:
            leg["lateral_gain_body_per_command"] = (
                leg["delta"]["body_lateral_m"] / (leg["vy_mps"] * leg["duration_s"])
            )
            leg["forward_gain_body_per_command_abs"] = (
                abs(leg["delta"]["body_forward_m"]) / command_distance
            )
            gains.append(leg["lateral_gain_body_per_command"])
        sign_key = "positive_y" if leg["vy_mps"] > 0 else "negative_y"
        grouped[(round(abs(leg["vy_mps"]), 4), sign_key)].append(leg)

    by_speed_sign = []
    for (speed_mps, sign_key), group in sorted(grouped.items()):
        group_lateral = [leg["delta"]["body_lateral_m"] for leg in group]
        group_forward = [leg["delta"]["body_forward_m"] for leg in group]
        group_yaw = [leg["delta"]["yaw_delta_deg"] for leg in group]
        group_gains = [
            leg.get("lateral_gain_body_per_command")
            for leg in group
            if leg.get("lateral_gain_body_per_command") is not None
        ]
        by_speed_sign.append(
            {
                "speed_mps": speed_mps,
                "command_sign": sign_key,
                "leg_count": len(group),
                "body_lateral_m_median": round_or_none(median(group_lateral)),
                "body_lateral_abs_m_median": round_or_none(
                    median([abs(value) for value in group_lateral])
                ),
                "body_forward_m_median": round_or_none(median(group_forward)),
                "yaw_delta_deg_median": round_or_none(median(group_yaw)),
                "lateral_gain_body_per_command_median": round_or_none(
                    median(group_gains)
                ),
            }
        )

    positive_gains = [
        leg["lateral_gain_body_per_command"]
        for leg in legs
        if leg["vy_mps"] > 0 and leg.get("lateral_gain_body_per_command") is not None
    ]
    positive_gain_median = median(positive_gains)
    if positive_gain_median is None:
        command_to_body_sign = "unknown"
    elif positive_gain_median > 0:
        command_to_body_sign = "positive_linear_y_produces_positive_body_lateral"
    else:
        command_to_body_sign = "positive_linear_y_produces_negative_body_lateral"

    return {
        "leg_count": len(legs),
        "max_abs_body_lateral_m": round(max(lateral_abs), 4),
        "max_abs_body_forward_m": round(max(forward_abs), 4),
        "max_abs_yaw_delta_deg": round(max(yaw_abs), 4),
        "lateral_gain_body_per_command_median": round_or_none(median(gains)),
        "positive_linear_y_gain_median": round_or_none(positive_gain_median),
        "command_to_body_lateral_sign": command_to_body_sign,
        "by_speed_sign": by_speed_sign,
        "recommendation": "usable_for_shadow_review_not_active_control",
    }


def run(args):
    speed_mps_values = parse_float_list(args.speed_mps_list)
    planned_legs = list(leg_plan(speed_mps_values, args.sequence, args.repeat_count))
    report = {
        "status": "started",
        "speed_mps_list": speed_mps_values,
        "duration_s": args.duration_s,
        "sequence": args.sequence,
        "repeat_count": args.repeat_count,
        "planned_leg_count": len(planned_legs),
        "legs": [],
    }

    if args.dry_run:
        report["status"] = "dry_run"
        report["planned_legs"] = [
            {
                "repeat_index": repeat_index,
                "speed_mps": speed_mps,
                "sign": sign,
                "vy_mps": sign * speed_mps,
                "duration_s": args.duration_s,
            }
            for repeat_index, speed_mps, sign in planned_legs
        ]
        print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True), flush=True)
        return
    if not args.confirm_live:
        raise RuntimeError("live linear.y sweep requires --confirm-live")

    import agibot_gdk

    result = agibot_gdk.gdk_init()
    gdk_res = getattr(agibot_gdk, "GDKRes", None)
    if gdk_res is not None and result not in (None, gdk_res.kSuccess):
        raise RuntimeError(f"GDK init failed: {result}")

    radar = None
    pnc = None
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
        check_clearance(initial_clearance, args, "sweep_initial")
        initial_pose = read_odom_pose(slam, "sweep_initial")
        report["initial_clearance_median_mm"] = initial_clearance
        report["initial_clearance_samples"] = initial_samples
        report["initial_pose"] = initial_pose
        print(
            "linear_y_sweep_initial "
            f"planned_leg_count={len(planned_legs)} "
            f"x={initial_pose['x']:.4f} y={initial_pose['y']:.4f} "
            f"yaw_deg={initial_pose['yaw_deg']:.3f}",
            flush=True,
        )

        request_control_with_retry(pnc, args.mode, args.retries, args.retry_wait_s)
        interval_s = 1.0 / args.hz
        command_count = max(1, math.ceil(args.duration_s * args.hz))
        current_pose = initial_pose
        for leg_index, (repeat_index, speed_mps, sign) in enumerate(planned_legs, 1):
            leg_clearance, leg_clearance_samples = read_clearance_window(
                radar, args.clearance_samples, args.clearance_interval_s
            )
            check_clearance(leg_clearance, args, f"sweep_leg_{leg_index}_before")
            start_pose = read_odom_pose(slam, f"sweep_leg_{leg_index}_start")
            vy_mps = sign * speed_mps
            twist = make_twist(agibot_gdk, vy_mps)
            print(
                "linear_y_sweep_leg_start "
                f"index={leg_index} repeat={repeat_index} "
                f"vy_mps={vy_mps:.4f} duration_s={args.duration_s:.2f} "
                f"commands={command_count}",
                flush=True,
            )
            for _ in range(command_count):
                pnc.move_chassis(twist)
                time.sleep(interval_s)
            stop_chassis(pnc, agibot_gdk)
            time.sleep(args.settle_s)
            end_pose = read_odom_pose(slam, f"sweep_leg_{leg_index}_end")
            delta = body_frame_delta(start_pose, end_pose)
            current_pose = end_pose
            leg_report = {
                "index": leg_index,
                "repeat_index": repeat_index,
                "speed_mps": speed_mps,
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
                "linear_y_sweep_leg_result "
                f"index={leg_index} repeat={repeat_index} "
                f"vy_mps={vy_mps:.4f} "
                f"body_forward_m={delta['body_forward_m']:.4f} "
                f"body_lateral_m={delta['body_lateral_m']:.4f} "
                f"yaw_delta_deg={delta['yaw_delta_deg']:.3f}",
                flush=True,
            )
            if leg_index < len(planned_legs):
                time.sleep(args.leg_pause_s)

        final_clearance, final_samples = read_clearance_window(
            radar, args.clearance_samples, args.clearance_interval_s
        )
        report["final_clearance_median_mm"] = final_clearance
        report["final_clearance_samples"] = final_samples
        final_pose = read_odom_pose(slam, "sweep_final")
        report["final_pose"] = final_pose
        report["total_delta"] = body_frame_delta(initial_pose, current_pose)
        summary = summarize_legs(report["legs"])
        summary["passes_min_displacement"] = (
            summary.get("max_abs_body_lateral_m", 0.0) >= args.min_expected_lateral_m
        )
        report["summary"] = summary
        report["status"] = "completed"
        print(
            "linear_y_sweep_summary "
            f"leg_count={summary['leg_count']} "
            f"max_abs_body_lateral_m={summary.get('max_abs_body_lateral_m')} "
            f"max_abs_body_forward_m={summary.get('max_abs_body_forward_m')} "
            f"command_to_body_lateral_sign={summary.get('command_to_body_lateral_sign')} "
            f"passes_min_displacement={summary['passes_min_displacement']}",
            flush=True,
        )
    except Exception as exc:
        run_error = exc
        report["status"] = "failed"
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if pnc is not None:
            stop_chassis(pnc, agibot_gdk)
            try:
                cancel_blocking_task(pnc, "sweep_final_cleanup")
            except Exception as exc:
                print(f"sweep_final_cleanup_cancel_failed error={exc}", flush=True)
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
                print(f"linear_y_sweep_report_json={output}", flush=True)
            except Exception as report_exc:
                print(f"linear_y_sweep_report_json_failed error={report_exc}", flush=True)
        if run_error is not None:
            print(f"linear_y_sweep_failed error={run_error}", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(description="G2 linear.y repeated sweep calibration")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--confirm-live", action="store_true")
    parser.add_argument("--mode", type=int, default=0)
    parser.add_argument("--speed-mps-list", default="0.03,0.05")
    parser.add_argument("--duration-s", type=float, default=0.8)
    parser.add_argument("--repeat-count", type=int, default=2)
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
    parser.add_argument("--min-side-clearance-mm", type=int, default=650)
    parser.add_argument("--min-front-rear-clearance-mm", type=int, default=500)
    parser.add_argument("--min-expected-lateral-m", type=float, default=0.015)
    parser.add_argument("--allow-estop-pedal-fault", action="store_true", default=True)
    parser.add_argument(
        "--strict-estop-pedal-fault",
        dest="allow_estop_pedal_fault",
        action="store_false",
    )
    parser.add_argument("--report-json", default=None)
    args = parser.parse_args()

    if args.dry_run and args.confirm_live:
        raise SystemExit("--dry-run and --confirm-live cannot be used together")
    try:
        speed_mps_values = parse_float_list(args.speed_mps_list)
    except ValueError as exc:
        raise SystemExit(f"--speed-mps-list invalid: {exc}") from exc
    if any(speed <= 0.0 or speed > 0.08 for speed in speed_mps_values):
        raise SystemExit("--speed-mps-list values must be in (0, 0.08]")
    if args.duration_s <= 0.0 or args.duration_s > 1.5:
        raise SystemExit("--duration-s must be in (0, 1.5]")
    if args.repeat_count <= 0 or args.repeat_count > 5:
        raise SystemExit("--repeat-count must be in [1, 5]")
    planned_count = len(speed_mps_values) * len(parse_sequence(args.sequence)) * args.repeat_count
    if planned_count > 30:
        raise SystemExit("planned sweep leg count is capped at 30")
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
