#!/usr/bin/env python3
"""
Probe the active rack lateral centering logic without running arm actions.

This script reuses Industrial7RodsController.center_rack_lateral_before_approach()
so the live test exercises the same crab-walk centering code used before grab
and place approach, but stops before coarse/fine front ultrasonic approach.
"""

from __future__ import annotations

from industrial_7_rods_total_controller import (
    Industrial7RodsController,
    parse_args,
)


def main() -> int:
    config = parse_args()
    controller = Industrial7RodsController(config)
    controller.current_rod_index = config.start_index

    try:
        controller.require_live_allowed()
        if config.rack_centering_mode != "active":
            raise RuntimeError(
                "industrial_lateral_centering_probe requires "
                "--rack-centering-mode active"
            )

        controller.next_step("料架横向 active 居中探针")
        if config.dry_run:
            controller.log("dry-run: skip active lateral centering probe")
            controller.complete_current_step("dry_run")
            controller.write_final_report("completed")
            return 0

        controller.check_live_startup_safety()

        def _run(rack):
            preflight = rack.preflight(
                allow_estop_pedal_fault=config.allow_estop_pedal_fault
            )
            controller.log(f"probe_preflight={preflight}")
            if preflight.status != "ok":
                raise RuntimeError(f"probe preflight blocked: {preflight}")

            before_snapshot = rack.read_snapshot()
            controller.log(f"probe_before_snapshot={before_snapshot}")
            before_pose = controller.monitor_rack_pose(
                rack,
                label="active_probe:before_approach",
                target_mm=None,
            )
            controller.center_rack_lateral_before_approach(
                rack,
                label="active_probe:before_approach",
                target_mm=None,
                initial_pose=before_pose,
            )
            after_pose = controller.monitor_rack_pose(
                rack,
                label="active_probe:after_centering",
                target_mm=None,
            )
            after_snapshot = rack.read_snapshot()
            controller.log(f"probe_after_snapshot={after_snapshot}")
            return after_pose

        after_pose = controller.with_industrial_rack(_run)
        controller.complete_current_step(
            "completed",
            after_lateral_center_m=None
            if after_pose is None
            else after_pose.get("lateral_center_m"),
            after_yaw_deg=None if after_pose is None else after_pose.get("yaw_deg"),
            after_confidence=None
            if after_pose is None
            else after_pose.get("confidence"),
        )
        controller.write_final_report("completed")
        return 0
    except Exception as exc:
        controller.fail_current_step(exc)
        controller.write_final_report("failed", exc)
        raise


if __name__ == "__main__":
    raise SystemExit(main())
