"""Rack fine-positioning and retreat primitive.

This class is a narrow importable wrapper around the existing
``RackIndustrialDockingController``.  The underlying controller owns the
ultrasonic filtering, fine-position stop logic, and relative chassis retreat
logic.  Keeping this wrapper thin lets the mission runner call those validated
methods without spawning a child process.

Call path:

``LOCAL_PICK`` uses ``fine_position`` before closing the grippers and then
``retreat`` after lifting/pulling back. ``LOCAL_PLACE`` uses the same two
operations around the rack-side release. This wrapper exists so the mission
runner only depends on one small class, while the larger industrial docking
controller keeps the sensor filtering details.
"""

from __future__ import annotations

try:
    from rack_industrial_docking import RackIndustrialDockingController
except ImportError:
    from rack_hybrid_docking_package.rack_industrial_docking import RackIndustrialDockingController


class RackDockingController:
    """Fine-position and retreat operations near the rack."""

    def fine_position(
        self,
        *,
        final_stop_mm: int,
        final_brake_margin_mm: int,
        final_speed_mps: float,
        max_duration_s: float,
        allow_estop_pedal_fault: bool,
    ):
        """Approach the rack until the configured ultrasonic stop threshold."""

        with RackIndustrialDockingController() as rack:
            return rack.fine_position(
                final_stop_mm=final_stop_mm,
                final_brake_margin_mm=final_brake_margin_mm,
                final_speed_mps=final_speed_mps,
                max_duration_s=max_duration_s,
                allow_estop_pedal_fault=allow_estop_pedal_fault,
            )

    def retreat(
        self,
        *,
        distance_m: float,
        speed_mps: float,
        allow_estop_pedal_fault: bool,
    ):
        """Retreat the chassis by a relative distance after local arm work."""

        with RackIndustrialDockingController() as rack:
            return rack.retreat(
                distance_m=distance_m,
                speed_mps=speed_mps,
                allow_estop_pedal_fault=allow_estop_pedal_fault,
            )

    def preflight(self, *, allow_estop_pedal_fault: bool = False):
        """Expose the rack controller's read-only preflight for diagnostics."""

        with RackIndustrialDockingController() as rack:
            return rack.preflight(allow_estop_pedal_fault=allow_estop_pedal_fault)
