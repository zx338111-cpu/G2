#!/usr/bin/env python3
"""Open both G2 omnipicker grippers.

Compatibility wrapper for manual one-step debugging.

The full mission imports ``GripperController`` directly.  This script is kept so
an operator can open both grippers without running the mission state machine.

Architecture note:

The open position is defined in ``GripperController``. Do not duplicate gripper
position constants here; this file should stay a thin compatibility wrapper.
"""

from __future__ import annotations

import argparse
import sys

from rack_hybrid_docking_package.g2_primitives.gripper import GripperController


def parse_args() -> argparse.Namespace:
    """Parse the standalone gripper-open CLI."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--settle-s", type=float, default=0.05)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    """Delegate to the importable gripper controller."""

    args = parse_args()
    try:
        ok = GripperController().open_both(dry_run=args.dry_run, settle_s=args.settle_s)
        return 0 if ok else 1
    except Exception as exc:
        print(f"move_ee_pose_open_2 failed: {type(exc).__name__}: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
