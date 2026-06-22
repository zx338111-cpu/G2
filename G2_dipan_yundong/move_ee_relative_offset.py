#!/usr/bin/env python3
"""Move both end effectors by explicit relative offsets.

Compatibility wrapper for manual one-step debugging.

The full mission imports ``EndEffectorOffsetController`` directly.  Use this
script only when you intentionally want to test one explicit left/right Cartesian
offset from the current robot pose.

Architecture note:

The full seven-rods runner keeps the old script name in structured logs, but it
dispatches to the imported class directly. This wrapper remains a manual test
entrypoint for one offset at a time.
"""

from __future__ import annotations

import argparse
import sys

from rack_hybrid_docking_package.g2_primitives.ee_offset import EndEffectorOffsetController


def xyz_arg(value: str) -> tuple[float, float, float]:
    """Parse an X,Y,Z offset string in meters."""

    parts = value.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected X,Y,Z in meters")
    try:
        return tuple(float(part) for part in parts)  # type: ignore[return-value]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid numeric offset: {value}") from exc


def parse_args() -> argparse.Namespace:
    """Parse the standalone relative-offset CLI."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", type=xyz_arg, required=True, help="Left arm offset X,Y,Z in meters")
    parser.add_argument("--right", type=xyz_arg, required=True, help="Right arm offset X,Y,Z in meters")
    parser.add_argument("--max-abs-m", type=float, default=0.25)
    parser.add_argument("--settle-s", type=float, default=0.3)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    """Validate arguments and delegate to the importable offset controller."""

    args = parse_args()
    try:
        ok = EndEffectorOffsetController().move_relative(
            left=args.left,
            right=args.right,
            max_abs_m=args.max_abs_m,
            settle_s=args.settle_s,
            dry_run=args.dry_run,
        )
        return 0 if ok else 1
    except Exception as exc:
        print(f"move_ee_relative_offset failed: {type(exc).__name__}: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
