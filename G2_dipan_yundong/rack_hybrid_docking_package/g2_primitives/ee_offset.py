"""Cartesian relative end-effector offset primitive.

This module is the importable implementation behind
``move_ee_relative_offset.py``.  It applies explicit Cartesian offsets to the
left and right end effectors, and is used for the fine pull-back/drop motions
around pick and place.

Offsets are intentionally small and validated before motion.  Larger motion
should be represented as a calibrated joint pose or a chassis move, not as one
large Cartesian offset.

Call path:

The mission runner uses this primitive for small, explicit field-tuned offsets:
pick pull-back, place final release offset, and post-release pull/drop. It is
not a general-purpose Cartesian planner.
"""

from __future__ import annotations

import time

from .gdk_context import gdk_session


Offset = tuple[float, float, float]


def validate_offset(label: str, offset: Offset, max_abs_m: float) -> None:
    """Reject malformed or unexpectedly large X/Y/Z offsets."""

    if len(offset) != 3:
        raise ValueError(f"{label} offset must be X,Y,Z")
    too_large = [axis for axis in offset if abs(axis) > max_abs_m]
    if too_large:
        raise ValueError(f"{label} offset exceeds max_abs_m={max_abs_m}: {offset}")


class EndEffectorOffsetController:
    """Move both arms by explicit relative Cartesian offsets."""

    def move_relative(
        self,
        *,
        left: Offset,
        right: Offset,
        max_abs_m: float = 0.25,
        settle_s: float = 0.3,
        dry_run: bool = False,
    ) -> bool:
        """Apply left/right relative offsets.

        ``left`` and ``right`` are ``(x, y, z)`` in meters.  ``dry_run=True``
        validates and logs the requested offsets without initializing GDK.
        """

        if max_abs_m <= 0.0:
            raise ValueError("max_abs_m must be positive")
        if settle_s < 0.0:
            raise ValueError("settle_s must be >= 0")
        validate_offset("left", left, max_abs_m)
        validate_offset("right", right, max_abs_m)

        print(
            "move_ee_relative_offset "
            f"left={left} right={right} max_abs_m={max_abs_m:.3f}",
            flush=True,
        )
        if dry_run:
            print("dry-run: skip GDK init and end-effector movement", flush=True)
            return True

        with gdk_session() as agibot_gdk:
            from end_effector_controller import EndEffectorController

            robot = agibot_gdk.Robot()
            time.sleep(2.0)
            controller = EndEffectorController(robot)
            ok = controller.adjust_arms_relative(offset_l=left, offset_r=right)
            time.sleep(settle_s)
            print(f"relative_offset_result={ok}", flush=True)
            return bool(ok)
