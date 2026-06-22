#!/usr/bin/env python3
"""Guarded low-speed forward recovery to an absolute front-ultrasonic target."""

import argparse
import json
import statistics
import time

from gdk_status_utils import read_motion_control_status_with_retry
from rack_radar_docking import RackRadarDockingController


def _field_size(value) -> int:
    try:
        return len(value)
    except TypeError:
        size = getattr(value, "size", None)
        if callable(size):
            return int(size())
        if size is not None:
            return int(size)
    return 0


def _motion_summary(motion) -> dict:
    return {
        "error_code": int(getattr(motion, "error_code", 0)),
        "error_msg": str(getattr(motion, "error_msg", "")),
        "mode": int(getattr(motion, "mode", 0)),
        "collision_pairs_1_size": _field_size(getattr(motion, "collision_pairs_1", ())),
        "collision_pairs_2_size": _field_size(getattr(motion, "collision_pairs_2", ())),
    }


def _front_state(controller: RackRadarDockingController, required_ids: set[int]) -> dict:
    distances = controller.selected_distances()
    by_id = {int(radar_id): int(distance) for radar_id, distance in distances}
    if not required_ids.issubset(by_id):
        return {
            "ok": False,
            "reason": "missing_front_id",
            "front_raw": tuple(sorted(by_id.items())),
        }
    values = [by_id[radar_id] for radar_id in sorted(required_ids)]
    return {
        "ok": True,
        "front_min_mm": min(values),
        "front_avg_mm": int(round(sum(values) / len(values))),
        "front_span_mm": max(values) - min(values),
        "front_raw": tuple((radar_id, by_id[radar_id]) for radar_id in sorted(required_ids)),
    }


def _print_event(event: str, **fields):
    print(json.dumps({"event": event, **fields}, ensure_ascii=True), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-front-mm", type=int, required=True)
    parser.add_argument("--tolerance-mm", type=int, default=30)
    parser.add_argument("--speed-mps", type=float, default=0.035)
    parser.add_argument("--max-forward-m", type=float, default=0.60)
    parser.add_argument("--max-duration-s", type=float, default=25.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--front-ids", default="0,1")
    parser.add_argument("--max-front-span-mm", type=int, default=120)
    parser.add_argument("--lost-timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=0.6)
    parser.add_argument("--allow-estop-pedal-fault", action="store_true")
    parser.add_argument(
        "--allow-motion-error-2",
        action="store_true",
        help="Allow stale collision-imminent state only when collision pair lists are empty.",
    )
    args = parser.parse_args()

    if args.target_front_mm <= 0:
        raise SystemExit("--target-front-mm must be positive")
    if args.tolerance_mm < 0:
        raise SystemExit("--tolerance-mm must be >= 0")
    if args.speed_mps <= 0:
        raise SystemExit("--speed-mps must be positive")
    if args.max_forward_m <= 0:
        raise SystemExit("--max-forward-m must be positive")
    if args.max_duration_s <= 0:
        raise SystemExit("--max-duration-s must be positive")
    if args.hz <= 0:
        raise SystemExit("--hz must be positive")

    front_ids = tuple(int(part.strip()) for part in args.front_ids.split(",") if part.strip())
    required_ids = set(front_ids)
    if not required_ids:
        raise SystemExit("--front-ids must not be empty")

    start = time.time()
    moved_est_m = 0.0
    status = "unknown"
    final_state = None

    with RackRadarDockingController(front_ids=front_ids, control_mode=0, init_wait_s=0.2) as front:
        power = front.robot.get_chassis_power_state()
        if getattr(power, "charge_plug_insert_state", 0) != 0:
            _print_event("blocked", reason="charge_plug_insert_state=1")
            return 2
        if getattr(power, "emergency_stop_pedal_state", 0) != 0:
            _print_event("blocked", reason="emergency_stop_pedal_state!=0")
            return 2
        if (
            getattr(power, "emergency_stop_pedal_fault_state", 0) != 0
            and not args.allow_estop_pedal_fault
        ):
            _print_event("blocked", reason="emergency_stop_pedal_fault_state=1")
            return 2
        if getattr(power, "chassis_ultrasonic_radar_power_state", 0) != 1:
            _print_event("blocked", reason="chassis_ultrasonic_radar_power_state!=1")
            return 2

        motion = read_motion_control_status_with_retry(front.robot)
        motion_info = _motion_summary(motion)
        _print_event("motion_status", **motion_info)
        if motion_info["error_code"] not in (0, 2):
            _print_event("blocked", reason="motion_control_error", motion=motion_info)
            return 2
        if motion_info["error_code"] == 2:
            if not args.allow_motion_error_2:
                _print_event("blocked", reason="motion_control_error=2_not_allowed")
                return 2
            if motion_info["collision_pairs_1_size"] or motion_info["collision_pairs_2_size"]:
                _print_event("blocked", reason="collision_pairs_present", motion=motion_info)
                return 2

        initial_states = []
        for sample_index in range(6):
            state = _front_state(front, required_ids)
            _print_event("pre_sample", sample=sample_index + 1, state=state)
            if not state["ok"]:
                _print_event("blocked", reason=state["reason"], state=state)
                return 2
            if state["front_span_mm"] > args.max_front_span_mm:
                _print_event("blocked", reason="front_span_too_large", state=state)
                return 2
            initial_states.append(state)
            time.sleep(0.1)

        initial_avg = int(round(statistics.median(s["front_avg_mm"] for s in initial_states)))
        initial_min = int(round(statistics.median(s["front_min_mm"] for s in initial_states)))
        if initial_min <= args.target_front_mm + args.tolerance_mm:
            final_state = initial_states[-1]
            status = "already_in_window"
            _print_event(
                "result",
                status=status,
                target_front_mm=args.target_front_mm,
                tolerance_mm=args.tolerance_mm,
                initial_avg_mm=initial_avg,
                initial_min_mm=initial_min,
                final_state=final_state,
            )
            return 0

        requested_m = min(
            args.max_forward_m,
            max(0.0, (initial_avg - args.target_front_mm) / 1000.0 + 0.02),
        )
        _print_event(
            "forward_recovery_start",
            target_front_mm=args.target_front_mm,
            tolerance_mm=args.tolerance_mm,
            initial_avg_mm=initial_avg,
            initial_min_mm=initial_min,
            requested_max_forward_m=round(requested_m, 4),
            speed_mps=args.speed_mps,
            hz=args.hz,
        )

        try:
            front.request_chassis_control_ready()
            time.sleep(0.2)
            last_valid_s = time.time()
            motion_start = time.time()
            next_tick_s = motion_start
            interval_s = 1.0 / args.hz
            while True:
                now = time.time()
                elapsed_s = now - motion_start
                moved_est_m = min(requested_m, elapsed_s * args.speed_mps)
                if elapsed_s > args.max_duration_s:
                    status = "timeout"
                    _print_event("stop_reason", status=status, elapsed_s=round(elapsed_s, 3))
                    break
                if moved_est_m >= requested_m:
                    status = "max_forward_reached"
                    _print_event(
                        "stop_reason",
                        status=status,
                        elapsed_s=round(elapsed_s, 3),
                        moved_est_m=round(moved_est_m, 4),
                    )
                    break

                state = _front_state(front, required_ids)
                _print_event(
                    "sample",
                    elapsed_s=round(elapsed_s, 3),
                    moved_est_m=round(moved_est_m, 4),
                    state=state,
                )
                if not state["ok"]:
                    if now - last_valid_s > args.lost_timeout_s:
                        status = state["reason"]
                        break
                    front.send_velocity(args.speed_mps)
                    time.sleep(max(0.0, next_tick_s - time.time()))
                    next_tick_s += interval_s
                    continue

                last_valid_s = now
                if state["front_span_mm"] > args.max_front_span_mm:
                    status = "front_span_too_large"
                    break
                if state["front_min_mm"] <= args.target_front_mm:
                    status = "target_reached"
                    final_state = state
                    break

                front.send_velocity(args.speed_mps)
                time.sleep(max(0.0, next_tick_s - time.time()))
                next_tick_s += interval_s
        finally:
            try:
                front.stop()
            except Exception as exc:
                _print_event("stop_failed", error=str(exc))

        time.sleep(args.settle_s)
        settled_states = []
        for sample_index in range(8):
            state = _front_state(front, required_ids)
            _print_event("settled_sample", sample=sample_index + 1, state=state)
            if state["ok"]:
                settled_states.append(state)
            time.sleep(0.1)

        if settled_states:
            final_state = settled_states[-1]
            median_avg = int(round(statistics.median(s["front_avg_mm"] for s in settled_states)))
            median_min = int(round(statistics.median(s["front_min_mm"] for s in settled_states)))
            final_span = max(s["front_avg_mm"] for s in settled_states) - min(
                s["front_avg_mm"] for s in settled_states
            )
            if median_min < args.target_front_mm - args.tolerance_mm:
                status = "overshot_too_close"
            elif median_avg > args.target_front_mm + args.tolerance_mm:
                status = "still_too_far"
            elif final_span > args.max_front_span_mm:
                status = "settled_unstable"
            elif status in ("unknown", "max_forward_reached"):
                status = "target_window_confirmed"
            _print_event(
                "result",
                status=status,
                target_front_mm=args.target_front_mm,
                tolerance_mm=args.tolerance_mm,
                moved_est_m=round(moved_est_m, 4),
                settled_median_avg_mm=median_avg,
                settled_median_min_mm=median_min,
                settled_avg_span_mm=final_span,
                final_state=final_state,
                elapsed_s=round(time.time() - start, 3),
            )
            return 0 if status in ("target_window_confirmed", "already_in_window", "target_reached") else 3

        _print_event(
            "result",
            status="no_settled_front",
            target_front_mm=args.target_front_mm,
            moved_est_m=round(moved_est_m, 4),
            elapsed_s=round(time.time() - start, 3),
        )
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
