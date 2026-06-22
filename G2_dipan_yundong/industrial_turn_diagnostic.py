#!/usr/bin/env python3
"""
G2 底盘 90 度转向诊断脚本。

用途：
  单独验证向左/向右 90 度转向，不执行机械臂、不执行抓放料流程。

运行前：
  cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1
  source /home/agi/app/env.sh

只打印参数，不运动：
  python3 industrial_turn_diagnostic.py --dry-run --direction right

确认现场安全后执行一次右转：
  python3 industrial_turn_diagnostic.py --confirm-live --direction right

确认现场安全后执行一次左转：
  python3 industrial_turn_diagnostic.py --confirm-live --direction left

如果要对比旧的速度开环方法：
  python3 industrial_turn_diagnostic.py --confirm-live --direction right --method velocity

关键点：
  - 默认使用 request_chassis_control(0)+move_chassis(Twist) 做 odom yaw 闭环；
  - Pnc.relative_move(yaw=±90) 只保留为对比诊断；
  - 转向过程中实时读取 odom yaw，误差进容差就停车，误差不收敛就失败；
  - charge_plug_insert_state=1、motion_control_error 非 0 时不发转向；
  - state=7 是取消/结束，不能当成 90 度到位；
  - 速度开环 method=velocity 只作为诊断/标定，仍必须通过 yaw 校验。
"""

from __future__ import annotations

import argparse
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


def base_dir() -> Path:
    return Path(__file__).resolve().parent


def add_package_path():
    package_dir = base_dir() / "rack_hybrid_docking_package"
    if str(package_dir) not in sys.path:
        sys.path.insert(0, str(package_dir))


def make_twist(agibot_gdk, vx: float = 0.0, wz: float = 0.0):
    twist = agibot_gdk.Twist()
    twist.linear = agibot_gdk.Vector3()
    twist.angular = agibot_gdk.Vector3()
    twist.linear.x = vx
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = wz
    return twist


def selected_ultrasonic_rows(radar_data):
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
    by_id = {row["id"]: row for row in rows}
    raw = []
    valid_values = []
    for radar_id in ids:
        row = by_id.get(radar_id)
        if row is None:
            continue
        distance = row.get("distance_mm")
        fault_state = row.get("fault_state")
        if fault_state != 0:
            continue
        try:
            distance = int(distance)
        except (TypeError, ValueError):
            continue
        if distance >= INVALID_DISTANCE_MM or distance < 50:
            continue
        raw.append((radar_id, distance))
        valid_values.append(distance)
    min_mm = min(valid_values) if valid_values else None
    return min_mm, tuple(raw)


def read_snapshot(label: str, radar):
    rows = selected_ultrasonic_rows(radar.get_latest_ultrasonic_radar())
    groups = {}
    for name, ids in ULTRASONIC_GROUPS:
        groups[name] = group_distances(rows, ids)
    print(
        f"{label} all={tuple((r['id'], r['distance_mm'], r['fault_state']) for r in rows)} "
        f"front_min={groups['front'][0]} front_raw={groups['front'][1]} "
        f"right_min={groups['right'][0]} right_raw={groups['right'][1]} "
        f"rear_min={groups['rear'][0]} rear_raw={groups['rear'][1]} "
        f"left_min={groups['left'][0]} left_raw={groups['left'][1]}",
        flush=True,
    )


def cancel_blocking_task(pnc, label: str):
    try:
        task = pnc.get_task_state()
    except Exception as exc:
        print(f"cancel_blocking_task_read_failed label={label} error={exc}", flush=True)
        return

    state = getattr(task, "state", None)
    task_id = getattr(task, "id", None)
    task_type = getattr(task, "type", None)
    message = getattr(task, "message", "")
    print(
        f"cancel_blocking_task_check label={label} state={state} "
        f"id={task_id} type={task_type} message={message}",
        flush=True,
    )
    if task_id is None or state in (0, 3, 6, 7, 8, 9):
        return
    try:
        pnc.cancel_task(task_id)
        print(f"cancel_blocking_task_done label={label} id={task_id} state={state}", flush=True)
        time.sleep(0.5)
    except RuntimeError as exc:
        if "Task is not in RUNNING or PAUSED state" not in str(exc):
            raise
        print(f"cancel_blocking_task_ignored label={label} id={task_id} error={exc}", flush=True)


def request_control_with_retry(pnc, mode: int, retries: int, wait_s: float):
    last_error = None
    cancel_blocking_task(pnc, "before_request_chassis_control")
    for attempt in range(1, retries + 1):
        try:
            result = pnc.request_chassis_control(mode)
            print(f"request_chassis_control attempt={attempt} mode={mode} result={result}", flush=True)
            return
        except Exception as exc:
            last_error = exc
            print(f"request_chassis_control_failed attempt={attempt} mode={mode} error={exc}", flush=True)
            try:
                task = pnc.get_task_state()
                print(f"task_state_after_request_fail state={task.state} id={task.id}", flush=True)
                cancel_blocking_task(pnc, f"after_request_fail_{attempt}")
            except Exception as task_exc:
                print(f"task_state_read_failed {task_exc}", flush=True)
            time.sleep(wait_s)
    raise RuntimeError(f"request_chassis_control failed after retries: {last_error}")


def stop_chassis(pnc, agibot_gdk):
    stop = make_twist(agibot_gdk, 0.0, 0.0)
    for _ in range(12):
        try:
            pnc.move_chassis(stop)
        except Exception as exc:
            print(f"stop_move_chassis_failed {exc}", flush=True)
        time.sleep(0.03)


def parse_state_list(text: str) -> tuple[int, ...]:
    states = []
    for part in text.split(","):
        item = part.strip()
        if item:
            states.append(int(item))
    if not states:
        raise ValueError("state list must not be empty")
    return tuple(states)


def make_turn_req(agibot_gdk, yaw_deg: float):
    req = agibot_gdk.NaviReq()
    req.target.position.x = 0.0
    req.target.position.y = 0.0
    req.target.position.z = 0.0
    half = math.radians(yaw_deg) / 2.0
    req.target.orientation.x = 0.0
    req.target.orientation.y = 0.0
    req.target.orientation.z = math.sin(half)
    req.target.orientation.w = math.cos(half)
    return req


def read_task(pnc, label: str):
    task = pnc.get_task_state()
    state = getattr(task, "state", None)
    task_id = getattr(task, "id", None)
    task_type = getattr(task, "type", None)
    message = getattr(task, "message", "")
    print(
        f"relative_turn_task {label} state={state} id={task_id} "
        f"type={task_type} message={message}",
        flush=True,
    )
    return task


def check_turn_preflight(robot, args):
    power = robot.get_chassis_power_state()
    motion = robot.get_motion_control_status()

    motion_error = getattr(motion, "error_code", 0)
    charge_plug = getattr(power, "charge_plug_insert_state", 0)
    estop_state = getattr(power, "emergency_stop_pedal_state", 0)
    estop_fault = getattr(power, "emergency_stop_pedal_fault_state", 0)
    ultrasonic_power = getattr(power, "chassis_ultrasonic_radar_power_state", 0)

    problems = []
    warnings = []
    if motion_error != 0:
        problems.append(f"motion_control_error={motion_error}")
    if charge_plug != 0:
        problems.append("charge_plug_insert_state=1")
    if estop_state != 0:
        problems.append("emergency_stop_pedal_state!=0")
    if ultrasonic_power != 1:
        problems.append("chassis_ultrasonic_radar_power_state!=1")

    print(
        "turn_preflight "
        f"motion_error={motion_error} charge_plug_insert_state={charge_plug} "
        f"emergency_stop_pedal_state={estop_state} "
        f"emergency_stop_pedal_fault_state={estop_fault} "
        f"chassis_ultrasonic_radar_power_state={ultrasonic_power} "
        f"warnings={tuple(warnings)} problems={tuple(problems)} "
        f"power={power!r} motion={motion!r}",
        flush=True,
    )
    if problems:
        raise RuntimeError("turn preflight blocked: " + ", ".join(problems))


def extract_yaw_deg_from_odom(odom) -> float | None:
    orientation_euler = getattr(odom, "orientation_euler", None)

    yaw_rad = None
    if orientation_euler is not None:
        for attr in ("z", "yaw"):
            if hasattr(orientation_euler, attr):
                yaw_rad = getattr(orientation_euler, attr)
                break
        if yaw_rad is None:
            try:
                yaw_rad = orientation_euler[2]
            except Exception:
                yaw_rad = None
    if yaw_rad is None:
        match = re.search(r"orientation_euler=\(([^)]*)\)", repr(odom))
        if match is not None:
            parts = [part.strip() for part in match.group(1).split(",")]
            if len(parts) >= 3:
                yaw_rad = parts[2]
    try:
        return math.degrees(float(yaw_rad))
    except Exception:
        return None


def read_yaw_deg(slam, label: str) -> float | None:
    try:
        odom = slam.get_odom_info()
    except Exception as exc:
        print(f"turn_yaw_read_failed label={label} error={type(exc).__name__}: {exc}", flush=True)
        return None

    yaw_deg = extract_yaw_deg_from_odom(odom)
    if yaw_deg is None:
        print(f"turn_yaw_unavailable label={label} odom={odom!r}", flush=True)
        return None

    print(f"turn_yaw label={label} yaw_deg={yaw_deg:.3f} odom={odom!r}", flush=True)
    return yaw_deg


def read_yaw_deg_quiet(slam) -> float | None:
    try:
        odom = slam.get_odom_info()
    except Exception:
        return None
    return extract_yaw_deg_from_odom(odom)


def normalize_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def validate_yaw_delta(label: str, expected_delta_deg: float, before_yaw: float, after_yaw: float, tolerance_deg: float):
    actual_delta = normalize_angle_deg(after_yaw - before_yaw)
    error = normalize_angle_deg(actual_delta - expected_delta_deg)
    print(
        f"{label}_yaw_validation expected_delta_deg={expected_delta_deg:.3f} "
        f"actual_delta_deg={actual_delta:.3f} error_deg={error:.3f} "
        f"tolerance_deg={tolerance_deg:.3f}",
        flush=True,
    )
    if abs(error) > tolerance_deg:
        raise RuntimeError(
            f"{label} yaw error too large: expected={expected_delta_deg:.3f}, "
            f"actual={actual_delta:.3f}, error={error:.3f}, tolerance={tolerance_deg:.3f}"
        )


def turn_error_deg(expected_delta_deg: float, before_yaw: float, current_yaw: float) -> float:
    actual_delta = normalize_angle_deg(current_yaw - before_yaw)
    return normalize_angle_deg(actual_delta - expected_delta_deg)


def wait_relative_turn_done(pnc, before_task_id, timeout_s: float, success_states: tuple[int, ...]) -> int:
    deadline = time.time() + timeout_s
    seen_new_task = False
    seen_running = False
    last_state = None
    last_task_id = None
    last_log_s = 0.0

    while time.time() < deadline:
        time.sleep(0.25)
        try:
            task = pnc.get_task_state()
        except Exception as exc:
            print(f"relative_turn_task_read_failed {type(exc).__name__}: {exc}", flush=True)
            continue

        state = getattr(task, "state", None)
        task_id = getattr(task, "id", None)
        task_type = getattr(task, "type", None)
        message = getattr(task, "message", "")
        now = time.time()
        elapsed_s = timeout_s - (deadline - now)

        if now - last_log_s >= 1.0 or state != last_state or task_id != last_task_id:
            print(
                "relative_turn_task poll "
                f"elapsed_s={elapsed_s:.2f} state={state} "
                f"id={task_id} type={task_type} message={message}",
                flush=True,
            )
            last_log_s = now
            last_state = state
            last_task_id = task_id

        if task_id is not None and task_id != before_task_id:
            seen_new_task = True
        if state in (1, 2, 4, 5, 6, 8):
            seen_running = True

        if not seen_new_task and not seen_running:
            if elapsed_s >= 4.0:
                raise RuntimeError(
                    "relative turn task did not start within 4s: "
                    f"before_task_id={before_task_id}, last_state={state}, "
                    f"last_task_id={task_id}, message={message}"
                )
            continue

        if state == 7:
            raise RuntimeError(
                "relative turn task was canceled before success: "
                f"state={state}, id={task_id}, before_id={before_task_id}, message={message}"
            )
        if state in success_states:
            return int(state)

    raise RuntimeError(
        "relative turn timed out or never started: "
        f"before_task_id={before_task_id}, last_state={last_state}, "
        f"last_task_id={last_task_id}, seen_new_task={seen_new_task}, seen_running={seen_running}"
    )


def run_relative_turn(args, agibot_gdk, pnc, angle_deg: float):
    before_id = None
    try:
        before_task = read_task(pnc, "before_submit")
        before_state = getattr(before_task, "state", None)
        before_id = getattr(before_task, "id", None)
        if before_state not in (0, 3, 7, 8, 9):
            print(f"relative_turn_cancel_blocking_task id={before_id} state={before_state}", flush=True)
            pnc.cancel_task(before_id)
            time.sleep(0.8)
            read_task(pnc, "after_cancel")
    except Exception as exc:
        print(f"relative_turn_before_task_read_failed {type(exc).__name__}: {exc}", flush=True)

    req = make_turn_req(agibot_gdk, angle_deg)
    print("relative_turn_submit_request", flush=True)
    pnc.relative_move(req)
    state = wait_relative_turn_done(
        pnc=pnc,
        before_task_id=before_id,
        timeout_s=args.timeout_s,
        success_states=args.success_states,
    )
    print(f"relative_turn_result state={state}", flush=True)
    return state


def velocity_for_error(args, error_deg: float) -> float:
    abs_error = abs(error_deg)
    if abs_error > 35.0:
        return args.angular_speed_radps
    if abs_error > 12.0:
        return min(args.angular_speed_radps, 0.30)
    if abs_error > max(args.yaw_tolerance_deg * 2.0, 3.0):
        return min(args.angular_speed_radps, 0.16)
    return min(args.angular_speed_radps, args.fine_angular_speed_radps)


def run_velocity_turn(args, agibot_gdk, pnc, slam, expected_yaw_delta_deg: float, before_yaw: float):
    request_control_with_retry(pnc, args.mode, args.retries, args.retry_wait_s)
    time.sleep(0.2)
    stop = make_twist(agibot_gdk, 0.0, 0.0)
    interval_s = 1.0 / args.hz
    max_duration_s = min(
        args.timeout_s,
        max(args.duration_s * 2.0, args.duration_s + 3.0, 6.0),
    )
    start = time.time()
    deadline = start + max_duration_s
    command_count = 0
    stable_count = 0
    yaw_miss_count = 0
    best_abs_error = float("inf")
    last_progress_s = start
    last_log_s = 0.0
    last_wz = 0.0
    final_yaw = before_yaw

    try:
        while time.time() < deadline:
            now = time.time()
            current_yaw = read_yaw_deg_quiet(slam)
            if current_yaw is None:
                yaw_miss_count += 1
                pnc.move_chassis(stop)
                if yaw_miss_count >= 5:
                    raise RuntimeError("velocity turn lost yaw feedback during closed-loop control")
                time.sleep(interval_s)
                continue
            yaw_miss_count = 0
            final_yaw = current_yaw
            error = turn_error_deg(expected_yaw_delta_deg, before_yaw, current_yaw)
            abs_error = abs(error)

            if abs_error < best_abs_error - 0.8:
                best_abs_error = abs_error
                last_progress_s = now
            elif now - start > 1.0 and now - last_progress_s > args.no_progress_timeout_s:
                raise RuntimeError(
                    "velocity turn yaw is not converging: "
                    f"best_abs_error_deg={best_abs_error:.3f}, "
                    f"current_error_deg={error:.3f}, "
                    f"no_progress_s={now - last_progress_s:.2f}"
                )

            if abs_error <= args.yaw_tolerance_deg:
                stable_count += 1
                pnc.move_chassis(stop)
                if stable_count >= args.stable_samples:
                    break
                time.sleep(max(interval_s, 0.08))
                continue

            stable_count = 0
            correction_delta = -error
            odom_direction = 1.0 if correction_delta > 0.0 else -1.0
            speed = max(args.min_angular_speed_radps, velocity_for_error(args, error))
            # On this chassis, move_chassis angular.z is opposite to SLAM odom yaw.
            wz = -odom_direction * speed
            twist = make_twist(agibot_gdk, 0.0, wz)
            pnc.move_chassis(twist)
            command_count += 1

            if now - last_log_s >= 0.5 or abs(wz - last_wz) > 1e-6:
                actual_delta = normalize_angle_deg(current_yaw - before_yaw)
                print(
                    "velocity_turn_loop "
                    f"elapsed_s={now - start:.2f} yaw_deg={current_yaw:.3f} "
                    f"actual_delta_deg={actual_delta:.3f} target_delta_deg={expected_yaw_delta_deg:.3f} "
                    f"error_deg={error:.3f} wz_radps={wz:.3f} "
                    f"best_abs_error_deg={best_abs_error:.3f}",
                    flush=True,
                )
                last_log_s = now
                last_wz = wz
            time.sleep(interval_s)
        else:
            raise RuntimeError(
                "velocity turn timed out before yaw reached tolerance: "
                f"timeout_s={max_duration_s:.2f}, final_yaw_deg={final_yaw:.3f}, "
                f"final_error_deg={turn_error_deg(expected_yaw_delta_deg, before_yaw, final_yaw):.3f}"
            )
    finally:
        for _ in range(12):
            try:
                pnc.move_chassis(stop)
            except Exception as exc:
                print(f"stop_move_chassis_failed {exc}", flush=True)
            time.sleep(0.03)

    elapsed_s = time.time() - start
    final_error = turn_error_deg(expected_yaw_delta_deg, before_yaw, final_yaw)
    print(
        "turn_velocity_closed_loop_done "
        f"commands={command_count} elapsed_s={elapsed_s:.2f} "
        f"final_yaw_deg={final_yaw:.3f} final_error_deg={final_error:.3f} "
        f"tolerance_deg={args.yaw_tolerance_deg:.3f}",
        flush=True,
    )


def run(args):
    angular_z = args.angular_speed_radps if args.direction == "left" else -args.angular_speed_radps
    angle_deg = 90.0 if args.direction == "left" else -90.0
    expected_yaw_delta_deg = -angle_deg if args.method == "velocity" else angle_deg
    print(
        "turn_diagnostic_config "
        f"direction={args.direction} method={args.method} angle_deg={angle_deg} "
        f"expected_yaw_delta_deg={expected_yaw_delta_deg} "
        f"mode={args.mode} angular_z={angular_z} duration_s={args.duration_s} "
        f"hz={args.hz} dry_run={args.dry_run}",
        flush=True,
    )

    if args.dry_run:
        return
    if not args.confirm_live:
        raise RuntimeError("真实转向必须传 --confirm-live")

    import agibot_gdk

    result = agibot_gdk.gdk_init()
    gdk_res = getattr(agibot_gdk, "GDKRes", None)
    if gdk_res is not None and result not in (None, gdk_res.kSuccess):
        raise RuntimeError(f"GDK init failed: {result}")

    radar = None
    try:
        robot = agibot_gdk.Robot()
        pnc = agibot_gdk.Pnc()
        slam = agibot_gdk.Slam()
        radar = agibot_gdk.UltrasonicRadar()
        time.sleep(0.8)

        for iteration in range(1, args.repeat + 1):
            print(f"turn_iteration_start index={iteration} repeat={args.repeat}", flush=True)
            check_turn_preflight(robot, args)
            read_snapshot(f"snapshot_before_turn_{iteration}", radar)
            before_yaw = read_yaw_deg(slam, f"before_turn_{iteration}")
            if before_yaw is None:
                raise RuntimeError("turn cannot start: yaw feedback unavailable")

            try:
                if args.method == "relative":
                    run_relative_turn(args, agibot_gdk, pnc, angle_deg)
                else:
                    run_velocity_turn(args, agibot_gdk, pnc, slam, expected_yaw_delta_deg, before_yaw)
            finally:
                stop_chassis(pnc, agibot_gdk)

            time.sleep(args.settle_s)
            after_yaw = read_yaw_deg(slam, f"after_turn_{iteration}")
            if after_yaw is None:
                raise RuntimeError("turn cannot be validated: yaw feedback unavailable after motion")
            read_snapshot(f"snapshot_after_turn_{iteration}", radar)
            validate_yaw_delta(
                label=f"{args.method}_turn_{iteration}",
                expected_delta_deg=expected_yaw_delta_deg,
                before_yaw=before_yaw,
                after_yaw=after_yaw,
                tolerance_deg=args.yaw_tolerance_deg,
            )
            print(f"turn_iteration_done index={iteration}", flush=True)
            if iteration < args.repeat:
                time.sleep(args.pause_s)
    finally:
        if radar is not None:
            try:
                radar.close_ultrasonic_radar()
            except Exception:
                pass
        try:
            agibot_gdk.gdk_release()
        except Exception:
            pass

    print("turn_diagnostic_done", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(description="G2 90 degree turn diagnostic")
    parser.add_argument("--direction", choices=("left", "right"), required=True)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--confirm-live", action="store_true")
    parser.add_argument(
        "--allow-estop-pedal-fault",
        dest="allow_estop_pedal_fault",
        action="store_true",
        default=True,
        help="legacy option; emergency_stop_pedal_fault_state is ignored",
    )
    parser.add_argument(
        "--strict-estop-pedal-fault",
        dest="allow_estop_pedal_fault",
        action="store_false",
        help="legacy option; emergency_stop_pedal_fault_state no longer blocks",
    )
    parser.add_argument("--method", choices=("relative", "velocity"), default="velocity")
    parser.add_argument("--mode", type=int, choices=(0, 1), default=0)
    parser.add_argument("--angular-speed-radps", type=float, default=0.5236)
    parser.add_argument("--fine-angular-speed-radps", type=float, default=0.08)
    parser.add_argument("--min-angular-speed-radps", type=float, default=0.06)
    parser.add_argument("--duration-s", type=float, default=3.0)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    parser.add_argument("--success-states", default="3,9")
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument("--retry-wait-s", type=float, default=0.8)
    parser.add_argument("--settle-s", type=float, default=0.7)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--pause-s", type=float, default=1.0)
    parser.add_argument("--yaw-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--stable-samples", type=int, default=3)
    parser.add_argument("--no-progress-timeout-s", type=float, default=2.0)
    args = parser.parse_args()
    if args.dry_run and args.confirm_live:
        raise SystemExit("--dry-run and --confirm-live cannot be used together")
    if args.angular_speed_radps <= 0.0:
        raise SystemExit("--angular-speed-radps must be positive")
    if args.fine_angular_speed_radps <= 0.0:
        raise SystemExit("--fine-angular-speed-radps must be positive")
    if args.min_angular_speed_radps <= 0.0:
        raise SystemExit("--min-angular-speed-radps must be positive")
    if args.min_angular_speed_radps > args.angular_speed_radps:
        raise SystemExit("--min-angular-speed-radps must be <= --angular-speed-radps")
    if args.fine_angular_speed_radps > args.angular_speed_radps:
        raise SystemExit("--fine-angular-speed-radps must be <= --angular-speed-radps")
    if args.duration_s <= 0.0:
        raise SystemExit("--duration-s must be positive")
    if args.timeout_s <= 0.0:
        raise SystemExit("--timeout-s must be positive")
    if args.hz <= 0.0:
        raise SystemExit("--hz must be positive")
    if args.retries <= 0:
        raise SystemExit("--retries must be positive")
    if args.settle_s < 0:
        raise SystemExit("--settle-s must be >= 0")
    if args.repeat <= 0:
        raise SystemExit("--repeat must be positive")
    if args.pause_s < 0:
        raise SystemExit("--pause-s must be >= 0")
    if args.yaw_tolerance_deg <= 0:
        raise SystemExit("--yaw-tolerance-deg must be positive")
    if args.yaw_tolerance_deg > 20.0:
        raise SystemExit("--yaw-tolerance-deg is capped at 20deg")
    if args.stable_samples <= 0:
        raise SystemExit("--stable-samples must be positive")
    if args.no_progress_timeout_s <= 0.0:
        raise SystemExit("--no-progress-timeout-s must be positive")
    try:
        args.success_states = parse_state_list(args.success_states)
    except ValueError as exc:
        raise SystemExit(f"--success-states invalid: {exc}") from exc
    if 7 in args.success_states:
        raise SystemExit("--success-states must not include 7; state=7 is canceled/ended, not turn success")
    return args


if __name__ == "__main__":
    run(parse_args())
