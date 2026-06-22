#!/usr/bin/env python3
"""
Drive forward slowly until selected ultrasonic radar ids reach a threshold.

Default mode is dry-run: it reads radar data and prints the stop decision, but
does not move the robot.  Live execution requires --execute and --front-ids.

Robot-side example:
  source /home/agi/app/env.sh
  python3 radar_stop_forward.py --front-ids 0,1 --stop-mm 300 --dry-run-samples 10

Live example after the front ids have been verified:
  source /home/agi/app/env.sh
  python3 radar_stop_forward.py --front-ids 0,1,2,3 --stop-mm 300 --speed 0.05 --execute

Known G2 emergency-stop pedal fault, only when vendor confirms velocity control
is allowed and an operator is guarding the robot:
  python3 radar_stop_forward.py --front-ids 0,1,2,3 --stop-mm 300 --speed 0.03 \
    --execute --allow-estop-pedal-fault

Relative-step fallback:
  python3 radar_stop_forward.py --front-ids 0,1,2,3 --method relative-step --step-m 0.05 --execute
"""

import argparse
import statistics
import time

import agibot_gdk


INVALID_DISTANCE_MM = 65535


def parse_ids(text):
    if not text:
        return []
    return [int(part.strip()) for part in text.split(",") if part.strip()]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Move forward until selected ultrasonic radar ids are near a rack."
    )
    parser.add_argument(
        "--front-ids",
        default="",
        help="Comma-separated ultrasonic radar ids that face the rack.",
    )
    parser.add_argument("--stop-mm", type=int, default=300)
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument(
        "--control-mode",
        type=int,
        default=0,
        help="Chassis control mode. Official keyboard example defaults to 0.",
    )
    parser.add_argument(
        "--method",
        choices=("velocity", "relative-step"),
        default="velocity",
        help="velocity uses move_chassis; relative-step uses small relative_move pulses.",
    )
    parser.add_argument(
        "--step-m",
        type=float,
        default=0.05,
        help="Forward step size for --method relative-step.",
    )
    parser.add_argument(
        "--step-timeout",
        type=float,
        default=6.0,
        help="Timeout for each relative_move step.",
    )
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--max-duration", type=float, default=8.0)
    parser.add_argument("--min-valid-mm", type=int, default=50)
    parser.add_argument("--history", type=int, default=3)
    parser.add_argument("--dry-run-samples", type=int, default=5)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--allow-estop-pedal-fault",
        action="store_true",
        help=(
            "Allow motion when emergency_stop_pedal_fault_state=1. "
            "Use only for the known vendor-confirmed G2 hardware fault."
        ),
    )
    return parser.parse_args()


def valid_distance(distance_mm, min_valid_mm):
    return min_valid_mm <= distance_mm < INVALID_DISTANCE_MM


def selected_distances(data, ids, min_valid_mm):
    distances = []
    for row in data.get("ultrasonic_radar_datas", []):
        radar_id = row.get("id")
        distance_mm = row.get("distance_mm")
        fault_state = row.get("fault_state")
        if ids and radar_id not in ids:
            continue
        if fault_state != 0:
            continue
        if valid_distance(distance_mm, min_valid_mm):
            distances.append((radar_id, distance_mm))
    return distances


def read_min_distance(radar, ids, min_valid_mm):
    data = radar.get_latest_ultrasonic_radar()
    distances = selected_distances(data, ids, min_valid_mm)
    if not distances:
        return None, data, distances
    return min(distance for _, distance in distances), data, distances


def check_motion_safety(robot, allow_estop_pedal_fault=False):
    problems = []
    warnings = []
    power = robot.get_chassis_power_state()
    motion = robot.get_motion_control_status()

    if getattr(motion, "error_code", 0) != 0:
        problems.append(f"motion_control_error={motion.error_code}")
    if getattr(power, "charge_plug_insert_state", 0) != 0:
        problems.append("charge_plug_insert_state=1")
    if getattr(power, "emergency_stop_pedal_fault_state", 0) != 0:
        if allow_estop_pedal_fault:
            warnings.append("emergency_stop_pedal_fault_state=1 allowed by CLI flag")
        else:
            problems.append("emergency_stop_pedal_fault_state=1")
    if getattr(power, "chassis_ultrasonic_radar_power_state", 0) != 1:
        problems.append("chassis_ultrasonic_radar_power_state!=1")
    return problems, warnings


def send_velocity(pnc, vx):
    twist = agibot_gdk.Twist()
    twist.linear = agibot_gdk.Vector3()
    twist.angular = agibot_gdk.Vector3()
    twist.linear.x = vx
    twist.linear.y = 0.0
    twist.angular.z = 0.0
    pnc.move_chassis(twist)


def stop(pnc):
    send_velocity(pnc, 0.0)


def cancel_blocking_task(pnc):
    task = pnc.get_task_state()
    if task.state not in (0, 3, 7, 9):
        pnc.cancel_task(task.id)
        time.sleep(0.3)


def wait_relative_done(pnc, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        time.sleep(0.2)
        task = pnc.get_task_state()
        if task.state in (3, 7, 9):
            time.sleep(0.4)
            return task.state
    return -1


def move_relative_forward_step(pnc, step_m, timeout_s):
    cancel_blocking_task(pnc)
    req = agibot_gdk.NaviReq()
    req.target.position.x = step_m
    req.target.position.y = 0.0
    req.target.position.z = 0.0
    req.target.orientation.x = 0.0
    req.target.orientation.y = 0.0
    req.target.orientation.z = 0.0
    req.target.orientation.w = 1.0
    pnc.relative_move(req)
    return wait_relative_done(pnc, timeout_s)


def request_chassis_control_ready(pnc, control_mode):
    cancel_blocking_task(pnc)
    try:
        return pnc.request_chassis_control(control_mode)
    except RuntimeError:
        cancel_blocking_task(pnc)
        time.sleep(0.5)
        return pnc.request_chassis_control(control_mode)


def main():
    args = parse_args()
    front_ids = parse_ids(args.front_ids)

    if args.execute and not front_ids:
        raise SystemExit("--execute requires --front-ids after physical id mapping")
    if args.stop_mm < args.min_valid_mm:
        raise SystemExit("--stop-mm must be >= --min-valid-mm")
    if args.speed <= 0.0:
        raise SystemExit("--speed must be positive")
    if args.step_m <= 0.0:
        raise SystemExit("--step-m must be positive")
    if args.hz <= 0.0:
        raise SystemExit("--hz must be positive")

    agibot_gdk.gdk_init()
    radar = agibot_gdk.UltrasonicRadar()
    robot = agibot_gdk.Robot()
    pnc = agibot_gdk.Pnc()

    try:
        time.sleep(0.5)
        ids_for_reading = front_ids
        if not args.execute and not ids_for_reading:
            ids_for_reading = []

        if not args.execute:
            for index in range(args.dry_run_samples):
                min_mm, data, distances = read_min_distance(
                    radar, ids_for_reading, args.min_valid_mm
                )
                print(
                    f"[dry-run] sample={index + 1} ids={ids_for_reading or 'all'} "
                    f"distances={distances} min_mm={min_mm} "
                    f"would_stop={min_mm is not None and min_mm <= args.stop_mm}"
                )
                time.sleep(0.2)
            return

        if args.allow_estop_pedal_fault and args.method != "velocity":
            raise SystemExit(
                "--allow-estop-pedal-fault is only supported with --method velocity"
            )

        problems, warnings = check_motion_safety(
            robot, allow_estop_pedal_fault=args.allow_estop_pedal_fault
        )
        if problems:
            raise SystemExit("Refusing to move: " + ", ".join(problems))
        for warning in warnings:
            print("WARNING:", warning)

        min_mm, _, distances = read_min_distance(radar, front_ids, args.min_valid_mm)
        if min_mm is None:
            raise SystemExit(f"No valid distance from front ids {front_ids}")
        if min_mm <= args.stop_mm:
            print(f"Already at stop threshold: min_mm={min_mm}, distances={distances}")
            return

        if args.method == "relative-step":
            print(
                f"Using relative-step; step_m={args.step_m:.3f} "
                f"stop_mm={args.stop_mm} ids={front_ids}"
            )
            start = time.time()
            while time.time() - start < args.max_duration:
                min_mm, _, distances = read_min_distance(
                    radar, front_ids, args.min_valid_mm
                )
                if min_mm is None:
                    raise SystemExit(f"Lost valid front radar distances for ids {front_ids}")
                print(f"before_step min_mm={min_mm} distances={distances}")
                if min_mm <= args.stop_mm:
                    print(f"Stopped before next step: min_mm={min_mm}")
                    return

                state = move_relative_forward_step(pnc, args.step_m, args.step_timeout)
                print(f"relative_step state={state}")
                if state not in (3, 9):
                    raise SystemExit(f"relative_move step failed or was canceled: state={state}")

            raise SystemExit(
                f"Timed out after {args.max_duration:.1f}s before stop threshold"
            )

        print(
            f"Requesting chassis control mode={args.control_mode}; "
            f"speed={args.speed:.3f}m/s stop_mm={args.stop_mm} ids={front_ids}"
        )
        request_chassis_control_ready(pnc, args.control_mode)
        time.sleep(0.3)

        interval = 1.0 / args.hz
        history = []
        start = time.time()
        while time.time() - start < args.max_duration:
            min_mm, _, distances = read_min_distance(
                radar, front_ids, args.min_valid_mm
            )
            if min_mm is None:
                stop(pnc)
                raise SystemExit(f"Lost valid front radar distances for ids {front_ids}")

            history.append(min_mm)
            history = history[-args.history :]
            filtered = int(statistics.median(history))
            print(f"min_mm={min_mm} filtered_mm={filtered} distances={distances}")

            if len(history) >= args.history and filtered <= args.stop_mm:
                stop(pnc)
                print(f"Stopped at filtered_mm={filtered}")
                return

            send_velocity(pnc, args.speed)
            time.sleep(interval)

        stop(pnc)
        raise SystemExit(f"Timed out after {args.max_duration:.1f}s before stop threshold")
    finally:
        try:
            stop(pnc)
        except Exception:
            pass
        time.sleep(0.1)
        try:
            cancel_blocking_task(pnc)
        except Exception:
            pass
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
