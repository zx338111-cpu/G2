#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Map 7 safe wrapper for the G2 BOX_528_1 workflow.

This file is intended to live only inside a copied workflow directory.
It does not modify the original robot controller or navigation config.

The current map 7 issue is that PNC reaches the x/y position of guide point
2, then fails during the final SpinToGoal phase because collision checking
reports a collision. For guide point 2 only, this wrapper allows the workflow
to continue when the measured map-frame position is already within tolerance.
All other guide points keep the original controller behavior.
"""

import importlib.util
import json
import math
import os
import time
from pathlib import Path
from typing import Optional

import agibot_gdk


WRAPPER_DIR = Path(__file__).resolve().parent
ORIGINAL_CONTROLLER_PATH = Path(
    os.environ.get(
        "G2_ORIGINAL_CONTROLLER",
        "/data/hondagys/wxf/BOX_528_1/robot_controller.py",
    )
)
CACHED_WAYPOINTS_PATH = WRAPPER_DIR / "map7_waypoints.json"

DEFAULT_POSITION_ONLY_WAYPOINTS = {"2"}
DEFAULT_POSITION_ONLY_TOL_M = 0.08
DEFAULT_PNC_READY_TIMEOUT_S = 3.0
DEFAULT_REQUIRE_LOC_READY = True
DEFAULT_MIN_LOC_CONFIDENCE = 50.0
DEFAULT_REQUIRE_NOT_CHARGING = True


def _load_original_controller():
    if not ORIGINAL_CONTROLLER_PATH.exists():
        raise FileNotFoundError(f"Original controller not found: {ORIGINAL_CONTROLLER_PATH}")

    spec = importlib.util.spec_from_file_location(
        "_g2_original_robot_controller",
        str(ORIGINAL_CONTROLLER_PATH),
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load original controller: {ORIGINAL_CONTROLLER_PATH}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_original = _load_original_controller()
_BaseRobotController = _original.RobotController


def _parse_bool_env(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _parse_position_only_names() -> set[str]:
    raw = os.environ.get("G2_POSITION_ONLY_WAYPOINTS")
    if raw is None:
        return set(DEFAULT_POSITION_ONLY_WAYPOINTS)
    names = {item.strip() for item in raw.split(",") if item.strip()}
    if names == {"none"}:
        return set()
    return names


def _position_only_tolerance_m() -> float:
    return _float_env("G2_POSITION_ONLY_TOL_M", DEFAULT_POSITION_ONLY_TOL_M)


def _float_env(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _yaw_deg_from_xyzw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def _wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


class RobotController(_BaseRobotController):
    """Original RobotController plus a narrow map7 point-2 workaround."""

    def _load_map_waypoints(self) -> tuple:
        # Prefer the cached map7 points to avoid repeated 18 MB map responses.
        # Set G2_USE_LIVE_MAP=1 to fall back to the original live Map API path.
        if not _parse_bool_env("G2_USE_LIVE_MAP", default=False):
            try:
                return self._load_cached_map7_waypoints()
            except Exception as exc:
                self._log(f"Cached map7 waypoints unavailable, falling back to live map: {exc}")
        return super()._load_map_waypoints()

    def _load_cached_map7_waypoints(self) -> tuple:
        with CACHED_WAYPOINTS_PATH.open("r", encoding="utf-8") as f:
            data = json.load(f)

        waypoints = {}
        for item in data["waypoints"]:
            name = str(item["name"])
            waypoints[name] = {
                "position": list(item["position"]),
                "orientation": list(item["orientation"]),
                "type": item.get("type", 0),
                "source": "map7_cache",
            }
        return waypoints, int(data.get("map_id", 7))

    def go(self, waypoint, high_precision: bool = False, timeout: float = 120.0) -> bool:
        name: Optional[str] = self._resolve_wp(waypoint)

        # After a full genie_app restart the first /pnc/task_state sample can be
        # absent for about one second. Waiting here is read-only and prevents a
        # false startup failure before a navigation command is sent.
        if not self._wait_for_pnc_task_state():
            return False

        # Chassis navigation must not start while the charge plug is inserted.
        # This gate is intentionally in the copied overlay only; it protects the
        # new map7 run without changing the colleague's original controller.
        if not self._chassis_power_ready_for_navigation():
            return False

        # Do not send chassis navigation while SLAM is still recovering. On this
        # robot get_slam_state() can be unavailable even when odom is healthy, so
        # use get_odom_info() and its loc_confidence as the motion gate.
        if not self._localization_ready_for_navigation():
            return False

        ok = super().go(waypoint, high_precision=high_precision, timeout=timeout)
        if ok:
            return True

        if high_precision or name is None:
            return False

        if name not in _parse_position_only_names():
            return False

        reached, detail = self._is_position_reached(name)
        if not reached:
            self._log(f"Position-only acceptance denied for '{name}': {detail}")
            return False

        self._log(
            f"Position-only acceptance for '{name}': {detail}. "
            "PNC terminal yaw/collision failure is ignored only for this configured waypoint."
        )
        return True

    def _wait_for_pnc_task_state(self) -> bool:
        timeout_s = _float_env("G2_PNC_READY_TIMEOUT_S", DEFAULT_PNC_READY_TIMEOUT_S)
        if timeout_s <= 0.0:
            return True

        deadline = time.monotonic() + timeout_s
        last_exc: Optional[BaseException] = None
        while time.monotonic() < deadline:
            try:
                self.pnc.get_task_state()
                return True
            except Exception as exc:
                last_exc = exc
                time.sleep(0.2)

        self._log(f"PNC task state unavailable before navigation: {last_exc}")
        return False

    def _chassis_power_ready_for_navigation(self) -> bool:
        if not _parse_bool_env("G2_REQUIRE_NOT_CHARGING", DEFAULT_REQUIRE_NOT_CHARGING):
            return True

        try:
            power = agibot_gdk.Robot().get_chassis_power_state()
        except Exception as exc:
            self._log(f"Chassis power gate blocked navigation: state unavailable: {exc}")
            return False

        charge_plug = getattr(power, "charge_plug_insert_state", None)
        if charge_plug == 1:
            voltage = getattr(power, "charge_plug_input_voltage", None)
            current = getattr(power, "charge_plug_input_current", None)
            self._log(
                "Chassis power gate blocked navigation: "
                f"charge_plug_insert_state={charge_plug}, "
                f"charge_voltage={voltage}, charge_current={current}"
            )
            return False

        estop = getattr(power, "emergency_stop_pedal_state", None)
        if estop == 1:
            self._log("Chassis power gate blocked navigation: emergency stop pedal is active")
            return False

        return True

    def _localization_ready_for_navigation(self) -> bool:
        if not _parse_bool_env("G2_REQUIRE_LOC_READY", DEFAULT_REQUIRE_LOC_READY):
            return True

        try:
            odom = self.slam.get_odom_info()
        except Exception as exc:
            self._log(f"Localization gate blocked navigation: odom unavailable: {exc}")
            return False

        min_conf = _float_env("G2_MIN_LOC_CONFIDENCE", DEFAULT_MIN_LOC_CONFIDENCE)
        conf = getattr(odom, "loc_confidence", None)
        if conf is not None and float(conf) < min_conf:
            self._log(
                f"Localization gate blocked navigation: loc_confidence={conf} "
                f"below min={min_conf}"
            )
            return False

        return True

    def _is_position_reached(self, name: str) -> tuple[bool, str]:
        wp = self.waypoints[name]
        target = wp["position"]
        tol = _position_only_tolerance_m()

        pose = self.slam.get_curr_pose()
        dx = target[0] - pose.position.x
        dy = target[1] - pose.position.y
        dist = math.hypot(dx, dy)

        yaw = _yaw_deg_from_xyzw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        target_q = wp["orientation"]
        target_yaw = _yaw_deg_from_xyzw(
            target_q[0],
            target_q[1],
            target_q[2],
            target_q[3],
        )
        yaw_err = _wrap_deg(target_yaw - yaw)

        detail = (
            f"distance={dist:.3f}m tolerance={tol:.3f}m "
            f"pose=({pose.position.x:.4f},{pose.position.y:.4f}) "
            f"target=({target[0]:.4f},{target[1]:.4f}) "
            f"yaw={yaw:.1f}deg target_yaw={target_yaw:.1f}deg yaw_err={yaw_err:.1f}deg"
        )
        return dist <= tol, detail


__all__ = ["RobotController"]
