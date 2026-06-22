#!/usr/bin/env python3
"""Small GDK status helpers shared by the docking controllers.

The important helper here is a retry wrapper for
``Robot.get_motion_control_status``. On this G2 stack, DDS can briefly return an
empty ``MotionControlStatus message is nullptr`` sample while services are
starting or while a task is settling. Treating one empty sample as a real
motion-control failure creates false blockers; treating repeated empty samples
as healthy would be unsafe. This helper keeps that distinction in one place.
"""

from __future__ import annotations

import time


def read_motion_control_status_with_retry(robot, attempts: int = 32, interval_s: float = 0.25):
    """
    Read Robot.get_motion_control_status() with a short retry window.

    On this G2 the DDS sample can briefly be unavailable and GDK reports
    "MotionControlStatus message is nullptr". A single empty sample should not
    be confused with a real non-zero motion-control error, but no status at all
    must still block motion.
    """
    if attempts <= 0:
        raise ValueError("attempts must be positive")
    if interval_s < 0.0:
        raise ValueError("interval_s must be >= 0")

    last_exc: Exception | None = None
    for attempt in range(attempts):
        try:
            return robot.get_motion_control_status()
        except RuntimeError as exc:
            last_exc = exc
            if attempt + 1 < attempts:
                time.sleep(interval_s)

    raise RuntimeError(
        "get_motion_control_status unavailable "
        f"after {attempts} attempts: {last_exc}"
    )
