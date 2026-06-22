#!/usr/bin/env python3
"""Guarded map-station navigation for the G2 industrial cell.

Default mode is dry-run/read-only. Physical navigation requires --confirm-live.

This file is the lower-level navigation implementation behind
``g2_primitives.nav.MapNavController``. It is intentionally usable as both:

- a standalone CLI for station debugging; and
- an import target for the class/import mission runner.

Safety model:

- station definitions are read from ``industrial_station_config.json``;
- read-only readiness checks happen before any live navigation;
- charge plug, motion-control errors, PNC task state, map id, pose, and odom
  speed are checked together;
- yaw refinement only runs after navigation has reached or nearly reached the
  target station, and it sends only low-speed angular velocity.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Any


PACKAGE_DIR = Path(__file__).resolve().parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from gdk_status_utils import read_motion_control_status_with_retry


IDLE_TASK_STATES = (0, 3, 6, 7, 8, 9)
DEFAULT_MAX_ABS_POSE_COORDINATE_M = 20.0
DEFAULT_MAX_STATION_POSE_DISTANCE_M = 8.0


def load_config(path: Path) -> dict[str, Any]:
    """Load the station/safety JSON config used by map20 navigation."""

    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def station_names(config: dict[str, Any]) -> list[str]:
    """Return configured station names for CLI listing and error messages."""

    return sorted((config.get("stations") or {}).keys())


def yaw_deg_from_quaternion(q: dict[str, float]) -> float:
    """Convert a station/current-pose quaternion into map-frame yaw degrees."""

    x = float(q.get("x", 0.0))
    y = float(q.get("y", 0.0))
    z = float(q.get("z", 0.0))
    w = float(q.get("w", 1.0))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(yaw)


def wrap_deg(value: float) -> float:
    """Normalize an angle to [-180, 180] for station yaw error calculations."""

    while value > 180.0:
        value -= 360.0
    while value < -180.0:
        value += 360.0
    return value


def pose_to_station_dict(pose) -> dict[str, dict[str, float]]:
    """Convert a GDK pose object into the same dict shape used by station JSON."""

    return {
        "position": {
            "x": round(float(pose.position.x), 6),
            "y": round(float(pose.position.y), 6),
            "z": round(float(pose.position.z), 6),
        },
        "orientation": {
            "x": round(float(pose.orientation.x), 6),
            "y": round(float(pose.orientation.y), 6),
            "z": round(float(pose.orientation.z), 6),
            "w": round(float(pose.orientation.w), 6),
        },
    }


def validate_station(config: dict[str, Any], name: str) -> dict[str, Any]:
    """Ensure a named station exists and has numeric position/orientation fields."""

    stations = config.get("stations") or {}
    if name not in stations:
        raise SystemExit(f"unknown station {name!r}; choices: {', '.join(station_names(config))}")
    station = stations[name]
    if station is None:
        raise SystemExit(
            f"station {name!r} is not calibrated yet; use --capture-current-pose "
            "at the real station and write the printed pose into the config"
        )
    for section in ("position", "orientation"):
        if section not in station:
            raise SystemExit(f"station {name!r} missing {section}")
    for key in ("x", "y", "z"):
        float(station["position"][key])
    for key in ("x", "y", "z", "w"):
        float(station["orientation"][key])
    return station


def make_navi_req(agibot_gdk, station: dict[str, Any]):
    """Build the GDK NaviReq for a calibrated station pose."""

    req = agibot_gdk.NaviReq()
    pos = station["position"]
    ori = station["orientation"]
    req.target.position.x = float(pos["x"])
    req.target.position.y = float(pos["y"])
    req.target.position.z = float(pos["z"])
    req.target.orientation.x = float(ori["x"])
    req.target.orientation.y = float(ori["y"])
    req.target.orientation.z = float(ori["z"])
    req.target.orientation.w = float(ori["w"])
    return req


def make_twist(agibot_gdk, wz: float = 0.0):
    """Build a pure-yaw Twist used by yaw refinement and stop commands."""

    twist = agibot_gdk.Twist()
    twist.linear = agibot_gdk.Vector3()
    twist.angular = agibot_gdk.Vector3()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = wz
    return twist


def init_gdk():
    """Initialize GDK for standalone navigation helpers."""

    import agibot_gdk

    result = agibot_gdk.gdk_init()
    gdk_res = getattr(agibot_gdk, "GDKRes", None)
    if gdk_res is not None and result not in (None, gdk_res.kSuccess):
        raise RuntimeError(f"GDK init failed: {result}")
    return agibot_gdk


def release_gdk(agibot_gdk) -> None:
    """Release GDK best-effort so cleanup does not hide the original failure."""

    try:
        agibot_gdk.gdk_release()
    except Exception:
        pass


def read_odom_sample(slam) -> dict[str, Any] | None:
    """Read one odom sample with speed and localization-quality fields.

    ``Slam.get_odom_info`` carries more than velocity. After a failed or weak
    relocalization it can return zero body velocity while the pose jumps to
    absurd map coordinates or reports low localization fields. Returning a dict
    keeps the old stopped check and the new quality logging tied to the same
    odom sample.
    """

    try:
        odom = slam.get_odom_info()
    except Exception:
        return None
    velocity_body = getattr(odom, "velocity_body", None)
    if velocity_body is None:
        return None
    try:
        vx = float(getattr(velocity_body, "x", 0.0))
        vy = float(getattr(velocity_body, "y", 0.0))
        vz = float(getattr(velocity_body, "z", 0.0))
    except Exception:
        return None
    sample = {
        "speed_mps": math.sqrt(vx * vx + vy * vy + vz * vz),
        "loc_confidence": getattr(odom, "loc_confidence", None),
        "loc_state": getattr(odom, "loc_state", None),
    }
    pose = getattr(odom, "pose", None)
    if pose is not None:
        sample["odom_pose"] = repr(pose)
    return sample


def read_odom_speed_mps(slam) -> float | None:
    """Return scalar odom speed from Slam.get_odom_info(), or None if missing."""

    sample = read_odom_sample(slam)
    if sample is None:
        return None
    return float(sample["speed_mps"])


def pose_quality_check(current_pose: dict[str, Any] | None, config: dict[str, Any]) -> dict[str, Any]:
    """Validate that the current SLAM pose is plausible for this station map.

    A non-null pose is not enough for live navigation. On this stack, a bad
    localization path can produce finite-looking poses with coordinates far
    outside map20. The defaults below are intentionally conservative for the
    industrial-cell station map and can be overridden in the config ``safety``
    section if a larger map is later used.
    """

    if current_pose is None:
        return {"problems": [], "warnings": [], "summary": None}

    safety = config.get("safety") or {}
    max_abs_coord = float(safety.get("max_abs_pose_coordinate_m", DEFAULT_MAX_ABS_POSE_COORDINATE_M))
    max_station_distance = float(
        safety.get("max_station_pose_distance_m", DEFAULT_MAX_STATION_POSE_DISTANCE_M)
    )
    position = current_pose.get("position") or {}
    orientation = current_pose.get("orientation") or {}
    problems: list[str] = []
    warnings: list[str] = []

    try:
        x = float(position["x"])
        y = float(position["y"])
        z = float(position["z"])
    except Exception as exc:
        return {
            "problems": [f"pose_position_invalid={type(exc).__name__}: {exc}"],
            "warnings": [],
            "summary": None,
        }

    max_abs_seen = max(abs(x), abs(y), abs(z))
    if max_abs_seen > max_abs_coord:
        problems.append(f"pose_coordinate_out_of_range={max_abs_seen:.3f}>{max_abs_coord:.3f}")

    try:
        qx = float(orientation["x"])
        qy = float(orientation["y"])
        qz = float(orientation["z"])
        qw = float(orientation["w"])
        quat_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not 0.8 <= quat_norm <= 1.2:
            problems.append(f"pose_orientation_norm_invalid={quat_norm:.3f}")
    except Exception as exc:
        problems.append(f"pose_orientation_invalid={type(exc).__name__}: {exc}")
        quat_norm = None

    station_distances = []
    for name, station in (config.get("stations") or {}).items():
        if not isinstance(station, dict):
            continue
        station_pos = station.get("position") or {}
        try:
            sx = float(station_pos["x"])
            sy = float(station_pos["y"])
            sz = float(station_pos.get("z", 0.0))
        except Exception:
            continue
        dist = math.sqrt((x - sx) ** 2 + (y - sy) ** 2 + (z - sz) ** 2)
        station_distances.append((dist, name))

    nearest_station = None
    nearest_station_distance_m = None
    if station_distances:
        nearest_station_distance_m, nearest_station = min(station_distances)
        if nearest_station_distance_m > max_station_distance:
            problems.append(
                "pose_far_from_configured_stations="
                f"{nearest_station_distance_m:.3f}>{max_station_distance:.3f},nearest={nearest_station}"
            )
    else:
        warnings.append("pose_station_distance_unavailable")

    return {
        "problems": problems,
        "warnings": warnings,
        "summary": {
            "max_abs_coordinate_m": round(max_abs_seen, 6),
            "max_abs_coordinate_limit_m": max_abs_coord,
            "nearest_station": nearest_station,
            "nearest_station_distance_m": (
                None if nearest_station_distance_m is None else round(nearest_station_distance_m, 6)
            ),
            "nearest_station_distance_limit_m": max_station_distance,
            "orientation_norm": None if quat_norm is None else round(quat_norm, 6),
        },
    }


def odom_quality_check(odom_samples: list[dict[str, Any] | None], config: dict[str, Any]) -> dict[str, Any]:
    """Summarize optional odom localization fields for preflight output.

    The hard safety gate remains pose plausibility plus odom availability. Some
    firmware builds report ``loc_confidence=0`` even when speed samples are
    usable, so confidence/state are logged by default and can be made hard gates
    with these optional ``industrial_station_config.json`` safety keys:

    - ``min_odom_loc_confidence``
    - ``allowed_odom_loc_states``
    """

    safety = config.get("safety") or {}
    valid = [sample for sample in odom_samples if sample is not None]
    problems: list[str] = []
    warnings: list[str] = []
    confidences = []
    states = []
    for sample in valid:
        confidence = sample.get("loc_confidence")
        state = sample.get("loc_state")
        if isinstance(confidence, (int, float)):
            confidences.append(float(confidence))
        if isinstance(state, (int, float)):
            states.append(int(state))

    min_required = safety.get("min_odom_loc_confidence")
    if min_required is not None and confidences:
        min_confidence = min(confidences)
        min_required_f = float(min_required)
        if min_confidence < min_required_f:
            problems.append(f"odom_loc_confidence_low={min_confidence:.3f}<{min_required_f:.3f}")
    elif confidences and max(confidences) <= 0.0:
        warnings.append("odom_loc_confidence_not_positive")

    allowed_states = safety.get("allowed_odom_loc_states")
    if allowed_states is not None and states:
        allowed = {int(value) for value in allowed_states}
        unexpected = sorted({state for state in states if state not in allowed})
        if unexpected:
            problems.append(f"odom_loc_state_unexpected={unexpected},allowed={sorted(allowed)}")

    return {
        "problems": problems,
        "warnings": warnings,
        "summary": {
            "loc_confidence_samples": confidences,
            "loc_state_samples": states,
        },
    }


def stop_chassis(pnc, agibot_gdk) -> None:
    """Send several zero-velocity commands to make yaw refinement stop firmly."""

    stop = make_twist(agibot_gdk, 0.0)
    for _ in range(12):
        try:
            pnc.move_chassis(stop)
        except Exception as exc:
            print(f"stop_move_chassis_failed {type(exc).__name__}: {exc}", flush=True)
        time.sleep(0.03)


def request_chassis_control_with_retry(pnc, retries: int = 3, wait_s: float = 0.4) -> None:
    """Request remote chassis control, retrying transient PNC release races."""

    last_error = None
    for attempt in range(1, retries + 1):
        try:
            result = pnc.request_chassis_control(0)
            print(
                json.dumps(
                    {"event": "request_chassis_control", "attempt": attempt, "mode": 0, "result": str(result)},
                    ensure_ascii=False,
                ),
                flush=True,
            )
            return
        except Exception as exc:
            last_error = exc
            print(
                json.dumps(
                    {
                        "event": "request_chassis_control_failed",
                        "attempt": attempt,
                        "mode": 0,
                        "error": f"{type(exc).__name__}: {exc}",
                    },
                    ensure_ascii=False,
                ),
                flush=True,
            )
            time.sleep(wait_s)
    raise RuntimeError(f"request_chassis_control failed after retries: {last_error}")


def cancel_active_pnc_task(pnc, label: str) -> None:
    """Best-effort cancel for a still-running PNC task during cleanup."""

    try:
        task = pnc.get_task_state()
    except Exception as exc:
        print(
            json.dumps(
                {"event": "pnc_cleanup_read_failed", "label": label, "error": f"{type(exc).__name__}: {exc}"},
                ensure_ascii=False,
            ),
            flush=True,
        )
        return
    state = getattr(task, "state", None)
    task_id = getattr(task, "id", None)
    task_type = getattr(task, "type", None)
    print(
        json.dumps(
            {"event": "pnc_cleanup_check", "label": label, "state": state, "task_id": task_id, "task_type": task_type},
            ensure_ascii=False,
        ),
        flush=True,
    )
    if task_id is None or state in IDLE_TASK_STATES:
        return
    try:
        pnc.cancel_task(task_id)
        print(
            json.dumps(
                {"event": "pnc_cleanup_cancel_sent", "label": label, "state": state, "task_id": task_id},
                ensure_ascii=False,
            ),
            flush=True,
        )
        time.sleep(0.4)
    except RuntimeError as exc:
        if "Task is not in RUNNING or PAUSED state" not in str(exc):
            raise
        print(
            json.dumps(
                {
                    "event": "pnc_cleanup_cancel_ignored",
                    "label": label,
                    "task_id": task_id,
                    "error": str(exc),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )


def pose_error(curr: dict[str, Any], target: dict[str, Any]) -> dict[str, float]:
    """Compute station XY and yaw error from current and target pose dicts."""

    dx = float(curr["position"]["x"]) - float(target["position"]["x"])
    dy = float(curr["position"]["y"]) - float(target["position"]["y"])
    xy = math.sqrt(dx * dx + dy * dy)
    yaw_curr = yaw_deg_from_quaternion(curr["orientation"])
    yaw_target = yaw_deg_from_quaternion(target["orientation"])
    return {
        "xy_error_m": xy,
        "yaw_error_deg": wrap_deg(yaw_curr - yaw_target),
        "current_yaw_deg": yaw_curr,
        "target_yaw_deg": yaw_target,
    }


def refine_yaw_to_station(
    agibot_gdk,
    robot,
    pnc,
    slam,
    map_manager,
    target: dict[str, Any],
    config: dict[str, Any],
    *,
    tolerance_deg: float,
    max_error_deg: float,
    angular_speed_radps: float,
    fine_angular_speed_radps: float,
    timeout_s: float,
    hz: float,
    stable_samples: int,
) -> dict[str, Any]:
    """Low-speed in-place yaw correction after map navigation reaches a station."""
    preflight = build_preflight(robot, pnc, slam, map_manager, config)
    print(json.dumps({"event": "yaw_refine_preflight", **preflight}, ensure_ascii=False), flush=True)
    if not preflight["ok"]:
        raise RuntimeError("yaw refine preflight blocked: " + ", ".join(preflight["problems"]))

    target_yaw = yaw_deg_from_quaternion(target["orientation"])
    initial_pose = pose_to_station_dict(slam.get_curr_pose())
    initial_error = wrap_deg(yaw_deg_from_quaternion(initial_pose["orientation"]) - target_yaw)
    if abs(initial_error) > max_error_deg:
        raise RuntimeError(
            f"yaw refine initial error too large: error={initial_error:.3f}, max={max_error_deg:.3f}"
        )

    request_chassis_control_with_retry(pnc)
    stop = make_twist(agibot_gdk, 0.0)
    interval_s = 1.0 / hz
    start = time.time()
    deadline = start + timeout_s
    command_count = 0
    stable_count = 0
    best_abs_error = abs(initial_error)
    last_progress_s = start
    last_log_s = 0.0
    final_pose = initial_pose
    final_error = initial_error

    try:
        while time.time() < deadline:
            now = time.time()
            current_pose = pose_to_station_dict(slam.get_curr_pose())
            current_yaw = yaw_deg_from_quaternion(current_pose["orientation"])
            error = wrap_deg(current_yaw - target_yaw)
            abs_error = abs(error)
            final_pose = current_pose
            final_error = error

            if abs_error < best_abs_error - 0.2:
                best_abs_error = abs_error
                last_progress_s = now
            elif now - start > 1.0 and now - last_progress_s > 2.5:
                raise RuntimeError(
                    "yaw refine is not converging: "
                    f"best_abs_error_deg={best_abs_error:.3f}, current_error_deg={error:.3f}"
                )

            if abs_error <= tolerance_deg:
                stable_count += 1
                pnc.move_chassis(stop)
                if stable_count >= stable_samples:
                    break
                time.sleep(max(interval_s, 0.08))
                continue

            stable_count = 0
            correction_delta = -error
            odom_direction = 1.0 if correction_delta > 0.0 else -1.0
            speed = fine_angular_speed_radps if abs_error <= 3.0 else angular_speed_radps
            # This loop compares against Slam.get_curr_pose() map yaw; positive
            # angular.z increases that yaw on this stack.
            wz = odom_direction * speed
            pnc.move_chassis(make_twist(agibot_gdk, wz))
            command_count += 1

            if now - last_log_s >= 0.4:
                print(
                    json.dumps(
                        {
                            "event": "yaw_refine_poll",
                            "elapsed_s": round(now - start, 3),
                            "current_yaw_deg": current_yaw,
                            "target_yaw_deg": target_yaw,
                            "error_deg": error,
                            "wz_radps": wz,
                            "best_abs_error_deg": best_abs_error,
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
                last_log_s = now
            time.sleep(interval_s)
        else:
            raise RuntimeError(
                "yaw refine timed out: "
                f"timeout_s={timeout_s:.2f}, final_error_deg={final_error:.3f}"
            )
    finally:
        stop_chassis(pnc, agibot_gdk)
        cancel_active_pnc_task(pnc, "yaw_refine")

    elapsed_s = time.time() - start
    result = {
        "status": "ok",
        "elapsed_s": round(elapsed_s, 3),
        "commands": command_count,
        "initial_error_deg": initial_error,
        "final_error_deg": final_error,
        "target_yaw_deg": target_yaw,
        "final_pose": final_pose,
        "tolerance_deg": tolerance_deg,
    }
    print(json.dumps({"event": "yaw_refine_result", **result}, ensure_ascii=False), flush=True)
    return result


def build_preflight(robot, pnc, slam, map_manager, config: dict[str, Any]) -> dict[str, Any]:
    """Read every navigation readiness signal without sending motion commands."""

    problems: list[str] = []
    warnings: list[str] = []
    safety = config.get("safety") or {}

    current_map_id = None
    current_map_name = ""
    charge_plug = None
    charge_current = None
    charge_voltage = None
    motion_error = None
    task_state = None
    task_id = None
    current_pose = None
    pose_quality = None
    odom_quality = None

    try:
        current_map = map_manager.get_curr_map()
        current_map_id = getattr(current_map, "id", None)
        current_map_name = getattr(current_map, "name", "")
    except Exception as exc:
        problems.append(f"map_unavailable={type(exc).__name__}: {exc}")

    try:
        power = robot.get_chassis_power_state()
        charge_plug = int(getattr(power, "charge_plug_insert_state", 0))
        charge_current = float(getattr(power, "charge_plug_input_current", 0.0))
        charge_voltage = float(getattr(power, "charge_plug_input_voltage", 0.0))
        if safety.get("require_charge_plug_unplugged", True) and charge_plug != 0:
            problems.append("charge_plug_insert_state=1")
        max_charge_current = float(safety.get("max_charge_input_current_a", 0.5))
        if charge_current > max_charge_current:
            problems.append(f"charge_input_current={charge_current:.3f}>{max_charge_current:.3f}")
        if int(getattr(power, "emergency_stop_pedal_state", 0)) != 0:
            problems.append("emergency_stop_pedal_state!=0")
        if int(getattr(power, "chassis_ultrasonic_radar_power_state", 0)) != 1:
            problems.append("chassis_ultrasonic_radar_power_state!=1")
    except Exception as exc:
        problems.append(f"chassis_power_unavailable={type(exc).__name__}: {exc}")

    try:
        motion = read_motion_control_status_with_retry(robot)
        motion_error = int(getattr(motion, "error_code", 0))
        if safety.get("require_motion_control_error_zero", True) and motion_error != 0:
            problems.append(f"motion_control_error={motion_error}")
    except Exception as exc:
        problems.append(f"motion_control_unavailable={type(exc).__name__}: {exc}")

    try:
        task = pnc.get_task_state()
        task_state = getattr(task, "state", None)
        task_id = getattr(task, "id", None)
        if safety.get("require_pnc_idle_before_navigation", True) and task_state not in IDLE_TASK_STATES:
            problems.append(f"pnc_task_state_not_idle={task_state},id={task_id}")
    except Exception as exc:
        problems.append(f"pnc_task_unavailable={type(exc).__name__}: {exc}")

    try:
        pose = slam.get_curr_pose()
        current_pose = pose_to_station_dict(pose)
    except Exception as exc:
        problems.append(f"pose_unavailable={type(exc).__name__}: {exc}")

    pose_quality_result = pose_quality_check(current_pose, config)
    problems.extend(pose_quality_result["problems"])
    warnings.extend(pose_quality_result["warnings"])
    pose_quality = pose_quality_result["summary"]

    odom_samples: list[dict[str, Any] | None] = []
    speed_samples: list[float | None] = []
    for index in range(3):
        odom_sample = read_odom_sample(slam)
        odom_samples.append(odom_sample)
        speed_samples.append(None if odom_sample is None else float(odom_sample["speed_mps"]))
        if index < 2:
            time.sleep(0.15)

    odom_quality_result = odom_quality_check(odom_samples, config)
    problems.extend(odom_quality_result["problems"])
    warnings.extend(odom_quality_result["warnings"])
    odom_quality = odom_quality_result["summary"]

    expected_map_id = config.get("map_id")
    if expected_map_id is not None and current_map_id is not None and int(current_map_id) != int(expected_map_id):
        problems.append(f"map_id_mismatch={current_map_id}!={expected_map_id}")

    stopped_limit = float((config.get("arrival") or {}).get("stopped_speed_mps", 0.02))
    valid_speeds = [speed for speed in speed_samples if speed is not None]
    max_speed = max(valid_speeds) if valid_speeds else None
    if max_speed is None:
        problems.append("odom_velocity_unavailable")
    elif max_speed > stopped_limit:
        problems.append(f"robot_not_stopped_by_odom_speed={max_speed:.4f}>{stopped_limit:.4f}")

    return {
        "ok": not problems,
        "problems": problems,
        "warnings": warnings,
        "map_id": current_map_id,
        "map_name": current_map_name,
        "charge_plug_insert_state": charge_plug,
        "charge_input_current_a": charge_current,
        "charge_input_voltage_v": charge_voltage,
        "motion_control_error": motion_error,
        "pnc_task_state": task_state,
        "pnc_task_id": task_id,
        "odom_speed_samples_mps": speed_samples,
        "odom_quality": odom_quality,
        "current_pose": current_pose,
        "pose_quality": pose_quality,
    }


def wait_for_arrival(pnc, slam, target: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    """Poll PNC, SLAM pose, and odom until the station arrival contract is met."""

    arrival = config.get("arrival") or {}
    xy_tol = float(arrival.get("xy_tolerance_m", 0.08))
    yaw_tol = float(arrival.get("yaw_tolerance_deg", 3.0))
    timeout_s = float(arrival.get("timeout_s", 60.0))
    poll_s = float(arrival.get("poll_interval_s", 0.5))
    stopped_limit = float(arrival.get("stopped_speed_mps", 0.02))

    start = time.time()
    last: dict[str, Any] | None = None
    while True:
        pose = pose_to_station_dict(slam.get_curr_pose())
        err = pose_error(pose, target)
        speed = read_odom_speed_mps(slam)
        try:
            task = pnc.get_task_state()
            task_state = getattr(task, "state", None)
            task_id = getattr(task, "id", None)
        except Exception:
            task_state = None
            task_id = None
        last = {
            "elapsed_s": round(time.time() - start, 3),
            "pose": pose,
            "error": err,
            "odom_speed_mps": speed,
            "pnc_task_state": task_state,
            "pnc_task_id": task_id,
        }
        arrived = (
            err["xy_error_m"] <= xy_tol
            and abs(err["yaw_error_deg"]) <= yaw_tol
            and speed is not None
            and speed <= stopped_limit
            and task_state in IDLE_TASK_STATES
        )
        print(json.dumps({"event": "nav_poll", **last}, ensure_ascii=False), flush=True)
        if arrived:
            return {"status": "arrived", **last}
        if time.time() - start > timeout_s:
            return {"status": "timeout", **last}
        if task_state in IDLE_TASK_STATES and time.time() - start > 2.0:
            return {"status": "pnc_idle_before_arrival", **last}
        time.sleep(poll_s)


def main() -> int:
    """Standalone CLI for listing/capturing/checking/navigating map stations."""

    parser = argparse.ArgumentParser(description="Guarded G2 map station navigation")
    parser.add_argument("--config", default=str(PACKAGE_DIR / "industrial_station_config.json"))
    parser.add_argument("--station", default=None, help="Station name to navigate to")
    parser.add_argument("--dry-run", action="store_true", help="Plan only; default unless --confirm-live")
    parser.add_argument("--confirm-live", action="store_true", help="Allow physical map navigation")
    parser.add_argument("--list-stations", action="store_true", help="Print configured stations")
    parser.add_argument("--capture-current-pose", action="store_true", help="Read-only: print current pose JSON")
    parser.add_argument("--readiness-check", action="store_true", help="Read-only GDK preflight without navigation")
    parser.add_argument(
        "--refine-yaw",
        action="store_true",
        help="After live arrival, run a guarded low-speed in-place yaw correction",
    )
    parser.add_argument("--refine-yaw-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--refine-yaw-max-error-deg", type=float, default=6.0)
    parser.add_argument("--refine-yaw-angular-speed-radps", type=float, default=0.08)
    parser.add_argument("--refine-yaw-fine-angular-speed-radps", type=float, default=0.035)
    parser.add_argument("--refine-yaw-timeout-s", type=float, default=8.0)
    parser.add_argument("--refine-yaw-hz", type=float, default=10.0)
    parser.add_argument("--refine-yaw-stable-samples", type=int, default=3)
    args = parser.parse_args()

    if args.dry_run and args.confirm_live:
        raise SystemExit("--dry-run and --confirm-live cannot be used together")
    if args.refine_yaw and not args.confirm_live:
        raise SystemExit("--refine-yaw requires --confirm-live")
    if args.refine_yaw_tolerance_deg <= 0:
        raise SystemExit("--refine-yaw-tolerance-deg must be positive")
    if args.refine_yaw_tolerance_deg > 3.0:
        raise SystemExit("--refine-yaw-tolerance-deg must be <= 3.0")
    if args.refine_yaw_max_error_deg < args.refine_yaw_tolerance_deg:
        raise SystemExit("--refine-yaw-max-error-deg must be >= tolerance")
    if args.refine_yaw_max_error_deg > 10.0:
        raise SystemExit("--refine-yaw-max-error-deg must be <= 10.0")
    if args.refine_yaw_angular_speed_radps <= 0 or args.refine_yaw_angular_speed_radps > 0.12:
        raise SystemExit("--refine-yaw-angular-speed-radps must be in (0, 0.12]")
    if args.refine_yaw_fine_angular_speed_radps <= 0 or args.refine_yaw_fine_angular_speed_radps > 0.08:
        raise SystemExit("--refine-yaw-fine-angular-speed-radps must be in (0, 0.08]")
    if args.refine_yaw_timeout_s <= 0 or args.refine_yaw_timeout_s > 20.0:
        raise SystemExit("--refine-yaw-timeout-s must be in (0, 20]")
    if args.refine_yaw_hz <= 0 or args.refine_yaw_hz > 30.0:
        raise SystemExit("--refine-yaw-hz must be in (0, 30]")
    if args.refine_yaw_stable_samples <= 0:
        raise SystemExit("--refine-yaw-stable-samples must be positive")

    config_path = Path(args.config).resolve()
    config = load_config(config_path)
    dry_run = not args.confirm_live

    if args.list_stations:
        for name in station_names(config):
            station = (config.get("stations") or {}).get(name)
            print(f"{name}: {'CALIBRATED' if station else 'UNSET'}")
        return 0

    if args.capture_current_pose or args.readiness_check:
        agibot_gdk = init_gdk()
        try:
            robot = agibot_gdk.Robot()
            pnc = agibot_gdk.Pnc()
            slam = agibot_gdk.Slam()
            map_manager = agibot_gdk.Map()
            time.sleep(0.5)
            if args.capture_current_pose:
                pose = pose_to_station_dict(slam.get_curr_pose())
                print(json.dumps(pose, indent=2, ensure_ascii=False))
            if args.readiness_check:
                preflight = build_preflight(robot, pnc, slam, map_manager, config)
                print(json.dumps({"event": "readiness_check", **preflight}, indent=2, ensure_ascii=False))
                return 0 if preflight["ok"] else 2
        finally:
            release_gdk(agibot_gdk)
        return 0

    if not args.station:
        raise SystemExit("--station is required unless --list-stations or --capture-current-pose is used")

    target_station = validate_station(config, args.station)
    print(
        json.dumps(
            {
                "event": "nav_plan",
                "dry_run": dry_run,
                "station": args.station,
                "target": target_station,
                "map_id": config.get("map_id"),
                "config": str(config_path),
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    if dry_run:
        return 0

    agibot_gdk = init_gdk()
    try:
        robot = agibot_gdk.Robot()
        pnc = agibot_gdk.Pnc()
        slam = agibot_gdk.Slam()
        map_manager = agibot_gdk.Map()
        time.sleep(0.5)

        preflight = build_preflight(robot, pnc, slam, map_manager, config)
        print(json.dumps({"event": "preflight", **preflight}, ensure_ascii=False), flush=True)
        if not preflight["ok"]:
            raise RuntimeError("navigation preflight blocked: " + ", ".join(preflight["problems"]))

        req = make_navi_req(agibot_gdk, target_station)
        pnc.normal_navi(req)
        print(json.dumps({"event": "normal_navi_sent", "station": args.station}, ensure_ascii=False), flush=True)
        result = wait_for_arrival(pnc, slam, target_station, config)
        print(json.dumps({"event": "nav_result", **result}, ensure_ascii=False), flush=True)
        arrival = config.get("arrival") or {}
        yaw_refine_eligible_idle = False
        if args.refine_yaw and result["status"] == "pnc_idle_before_arrival":
            error = result.get("error") or {}
            xy_error_m = float(error.get("xy_error_m", 999.0))
            yaw_error_deg = abs(float(error.get("yaw_error_deg", 999.0)))
            xy_tol = float(arrival.get("xy_tolerance_m", 0.08))
            yaw_refine_eligible_idle = xy_error_m <= xy_tol and yaw_error_deg <= args.refine_yaw_max_error_deg
            if yaw_refine_eligible_idle:
                print(
                    json.dumps(
                        {
                            "event": "nav_idle_yaw_refine_takeover",
                            "status": result["status"],
                            "xy_error_m": xy_error_m,
                            "yaw_error_deg": yaw_error_deg,
                            "xy_tolerance_m": xy_tol,
                            "refine_yaw_max_error_deg": args.refine_yaw_max_error_deg,
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
        if result["status"] != "arrived" and not yaw_refine_eligible_idle:
            try:
                task = pnc.get_task_state()
                task_id = getattr(task, "id", None)
                if task_id is not None:
                    pnc.cancel_task(task_id)
            except Exception:
                pass
            return 3
        if args.refine_yaw:
            refine_yaw_to_station(
                agibot_gdk,
                robot,
                pnc,
                slam,
                map_manager,
                target_station,
                config,
                tolerance_deg=args.refine_yaw_tolerance_deg,
                max_error_deg=args.refine_yaw_max_error_deg,
                angular_speed_radps=args.refine_yaw_angular_speed_radps,
                fine_angular_speed_radps=args.refine_yaw_fine_angular_speed_radps,
                timeout_s=args.refine_yaw_timeout_s,
                hz=args.refine_yaw_hz,
                stable_samples=args.refine_yaw_stable_samples,
            )
        return 0
    finally:
        release_gdk(agibot_gdk)


if __name__ == "__main__":
    raise SystemExit(main())
