#!/usr/bin/env python3
"""Move both G2 arms to a joint pose JSON path.

Compatibility wrapper for manual one-step debugging.

The current seven-rods runner imports ``ArmJointController`` directly instead
of spawning this script.  Keep this file because it is still useful on the robot
when an operator wants to validate one arm pose JSON in isolation.

Architecture note:

This is a CLI wrapper, not the mission implementation. Any behavior change for
arm joint movement should normally go into
``rack_hybrid_docking_package/g2_primitives/arm.py`` so the full mission and
manual CLI stay consistent.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from rack_hybrid_docking_package.g2_primitives.arm import ArmJointController


def parse_args() -> argparse.Namespace:
    """Parse the standalone arm-move CLI."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", required=True, help="Arm joint pose JSON path")
    parser.add_argument("--joint-speed-radps", type=float, default=0.12)
    parser.add_argument("--settle-s", type=float, default=0.8)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    """Validate arguments and delegate to the importable arm controller."""

    args = parse_args()
    try:
        ok = ArmJointController().move_to_json(
            Path(args.json),
            joint_speed_radps=args.joint_speed_radps,
            settle_s=args.settle_s,
            dry_run=args.dry_run,
        )
        return 0 if ok else 1
    except Exception as exc:
        print(f"move_arm_by_json_path failed: {type(exc).__name__}: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
