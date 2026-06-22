"""Map-station navigation primitive.

This module wraps the validated functions in ``industrial_map_nav_guarded.py``
behind an importable class.  The mission runner uses this class for all named
station moves:

- ``GRAB_PRE`` before local pick;
- ``PLACE_PRE`` before local place;
- ``RECOVERY_SAFE`` after place;
- ``HOME_SAFE`` between rods and at mission end.

The class keeps navigation policy local to one place: readiness checks, station
lookup from ``industrial_station_config.json``, normal map navigation, arrival
validation, and optional yaw refinement after arrival.

Call path:

The mission runner enters this class for all coarse station-to-station motion.
Local rack approach, gripper, and arm motions are deliberately not in this file.
That separation lets preflight decide whether map navigation is safe before any
PNC task is started.
"""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Any

try:
    from industrial_map_nav_guarded import (
        build_preflight,
        init_gdk,
        load_config,
        make_navi_req,
        refine_yaw_to_station,
        release_gdk,
        station_names,
        validate_station,
        wait_for_arrival,
    )
except ImportError:
    from rack_hybrid_docking_package.industrial_map_nav_guarded import (
        build_preflight,
        init_gdk,
        load_config,
        make_navi_req,
        refine_yaw_to_station,
        release_gdk,
        station_names,
        validate_station,
        wait_for_arrival,
    )


class MapNavController:
    """Guarded map-station navigation with optional yaw refinement."""

    def __init__(self, config_path: str | Path):
        """Load the station config once for this controller instance."""

        self.config_path = Path(config_path).resolve()
        self.config = load_config(self.config_path)

    def list_stations(self) -> list[str]:
        """Return station names defined in the loaded config."""

        return station_names(self.config)

    def readiness_check(self) -> dict[str, Any]:
        """Run a read-only navigation readiness check.

        This checks the same live layers used before real navigation: robot
        power state, PNC task state, SLAM/map state, charge state, odom samples,
        and motion-control error status.  It does not send a motion command.
        """

        agibot_gdk = init_gdk()
        try:
            robot = agibot_gdk.Robot()
            pnc = agibot_gdk.Pnc()
            slam = agibot_gdk.Slam()
            map_manager = agibot_gdk.Map()
            time.sleep(0.5)
            preflight = build_preflight(robot, pnc, slam, map_manager, self.config)
            payload = {"event": "readiness_check", **preflight}
            print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
            return preflight
        finally:
            release_gdk(agibot_gdk)

    def goto_station(
        self,
        station: str,
        *,
        live: bool,
        refine_yaw: bool,
        refine_yaw_tolerance_deg: float = 1.0,
        refine_yaw_max_error_deg: float = 6.0,
        refine_yaw_angular_speed_radps: float = 0.08,
        refine_yaw_fine_angular_speed_radps: float = 0.035,
        refine_yaw_timeout_s: float = 8.0,
        refine_yaw_hz: float = 10.0,
        refine_yaw_stable_samples: int = 3,
    ) -> dict[str, Any]:
        """Navigate to ``station`` and return a structured result payload.

        In dry-run mode this validates the station and prints the target only.
        In live mode it performs preflight, sends ``normal_navi``, waits for
        arrival, and then refines yaw if requested.  The runner treats
        ``{"ok": False}`` as a hard phase failure.
        """

        self._validate_refine_args(
            live=live,
            refine_yaw=refine_yaw,
            refine_yaw_tolerance_deg=refine_yaw_tolerance_deg,
            refine_yaw_max_error_deg=refine_yaw_max_error_deg,
            refine_yaw_angular_speed_radps=refine_yaw_angular_speed_radps,
            refine_yaw_fine_angular_speed_radps=refine_yaw_fine_angular_speed_radps,
            refine_yaw_timeout_s=refine_yaw_timeout_s,
            refine_yaw_hz=refine_yaw_hz,
            refine_yaw_stable_samples=refine_yaw_stable_samples,
        )
        target_station = validate_station(self.config, station)
        dry_run = not live
        print(
            json.dumps(
                {
                    "event": "nav_plan",
                    "dry_run": dry_run,
                    "station": station,
                    "target": target_station,
                    "map_id": self.config.get("map_id"),
                    "config": str(self.config_path),
                },
                indent=2,
                ensure_ascii=False,
            ),
            flush=True,
        )
        if dry_run:
            return {"ok": True, "status": "dry_run", "station": station, "target": target_station}

        agibot_gdk = init_gdk()
        try:
            robot = agibot_gdk.Robot()
            pnc = agibot_gdk.Pnc()
            slam = agibot_gdk.Slam()
            map_manager = agibot_gdk.Map()
            time.sleep(0.5)

            preflight = build_preflight(robot, pnc, slam, map_manager, self.config)
            print(json.dumps({"event": "preflight", **preflight}, ensure_ascii=False), flush=True)
            if not preflight["ok"]:
                raise RuntimeError("navigation preflight blocked: " + ", ".join(preflight["problems"]))

            req = make_navi_req(agibot_gdk, target_station)
            pnc.normal_navi(req)
            print(json.dumps({"event": "normal_navi_sent", "station": station}, ensure_ascii=False), flush=True)
            result = wait_for_arrival(pnc, slam, target_station, self.config)
            print(json.dumps({"event": "nav_result", **result}, ensure_ascii=False), flush=True)

            arrival = self.config.get("arrival") or {}
            yaw_refine_eligible_idle = False
            if refine_yaw and result["status"] == "pnc_idle_before_arrival":
                error = result.get("error") or {}
                xy_error_m = float(error.get("xy_error_m", 999.0))
                yaw_error_deg = abs(float(error.get("yaw_error_deg", 999.0)))
                xy_tol = float(arrival.get("xy_tolerance_m", 0.08))
                yaw_refine_eligible_idle = (
                    xy_error_m <= xy_tol
                    and yaw_error_deg <= refine_yaw_max_error_deg
                )
                if yaw_refine_eligible_idle:
                    print(
                        json.dumps(
                            {
                                "event": "nav_idle_yaw_refine_takeover",
                                "status": result["status"],
                                "xy_error_m": xy_error_m,
                                "yaw_error_deg": yaw_error_deg,
                                "xy_tolerance_m": xy_tol,
                                "refine_yaw_max_error_deg": refine_yaw_max_error_deg,
                            },
                            ensure_ascii=False,
                        ),
                        flush=True,
                    )

            if result["status"] != "arrived" and not yaw_refine_eligible_idle:
                # If PNC stopped before the station tolerances are met, cancel
                # best-effort and let the mission runner fail the phase.  Do
                # not silently treat this as success; the physical station
                # alignment matters for the following arm motion.
                self._cancel_active_task_best_effort(pnc)
                return {"ok": False, "station": station, "nav_result": result}

            yaw_result = None
            if refine_yaw:
                yaw_result = refine_yaw_to_station(
                    agibot_gdk,
                    robot,
                    pnc,
                    slam,
                    map_manager,
                    target_station,
                    self.config,
                    tolerance_deg=refine_yaw_tolerance_deg,
                    max_error_deg=refine_yaw_max_error_deg,
                    angular_speed_radps=refine_yaw_angular_speed_radps,
                    fine_angular_speed_radps=refine_yaw_fine_angular_speed_radps,
                    timeout_s=refine_yaw_timeout_s,
                    hz=refine_yaw_hz,
                    stable_samples=refine_yaw_stable_samples,
                )
            return {
                "ok": True,
                "station": station,
                "nav_result": result,
                "yaw_refine_result": yaw_result,
            }
        finally:
            release_gdk(agibot_gdk)

    def _cancel_active_task_best_effort(self, pnc) -> None:
        """Try to cancel a still-active PNC task without masking the real error."""

        try:
            task = pnc.get_task_state()
            task_id = getattr(task, "id", None)
            if task_id is not None:
                pnc.cancel_task(task_id)
        except Exception:
            pass

    def _validate_refine_args(
        self,
        *,
        live: bool,
        refine_yaw: bool,
        refine_yaw_tolerance_deg: float,
        refine_yaw_max_error_deg: float,
        refine_yaw_angular_speed_radps: float,
        refine_yaw_fine_angular_speed_radps: float,
        refine_yaw_timeout_s: float,
        refine_yaw_hz: float,
        refine_yaw_stable_samples: int,
    ) -> None:
        """Validate yaw-refine parameters before sending live velocity commands."""

        if refine_yaw and not live:
            raise ValueError("refine_yaw requires live navigation")
        if refine_yaw_tolerance_deg <= 0 or refine_yaw_tolerance_deg > 3.0:
            raise ValueError("refine_yaw_tolerance_deg must be in (0, 3]")
        if refine_yaw_max_error_deg < refine_yaw_tolerance_deg or refine_yaw_max_error_deg > 10.0:
            raise ValueError("refine_yaw_max_error_deg must be >= tolerance and <= 10")
        if refine_yaw_angular_speed_radps <= 0 or refine_yaw_angular_speed_radps > 0.12:
            raise ValueError("refine_yaw_angular_speed_radps must be in (0, 0.12]")
        if refine_yaw_fine_angular_speed_radps <= 0 or refine_yaw_fine_angular_speed_radps > 0.08:
            raise ValueError("refine_yaw_fine_angular_speed_radps must be in (0, 0.08]")
        if refine_yaw_timeout_s <= 0 or refine_yaw_timeout_s > 20.0:
            raise ValueError("refine_yaw_timeout_s must be in (0, 20]")
        if refine_yaw_hz <= 0 or refine_yaw_hz > 30.0:
            raise ValueError("refine_yaw_hz must be in (0, 30]")
        if refine_yaw_stable_samples <= 0:
            raise ValueError("refine_yaw_stable_samples must be positive")
