#!/usr/bin/env python3
"""Move the G2 waist/body joints to values from a joint pose JSON file.

Compatibility wrapper for manual one-step debugging.

The current seven-rods runner imports ``WaistController`` directly.  This script
remains as a convenient CLI for testing a captured waist/body JSON without
running the full mission state machine.

Architecture note:

This wrapper intentionally delegates all validation, segmentation, retries, and
feedback-settle checks to ``WaistController``. Keeping the wrapper thin avoids
two different waist-motion implementations drifting apart.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from rack_hybrid_docking_package.g2_primitives.waist import WaistController


def parse_args() -> argparse.Namespace:
    """Parse the standalone waist-move CLI."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", required=True, help="Joint pose JSON path containing idx01-idx05 body joints")
    parser.add_argument("--joint-speed-radps", type=float, default=0.75)
    parser.add_argument("--max-step-rad", type=float, default=0.75)
    parser.add_argument("--settle-tol-rad", type=float, default=0.025)
    parser.add_argument("--settle-timeout-s", type=float, default=3.0)
    parser.add_argument("--poll-s", type=float, default=0.08)
    parser.add_argument("--settle-s", type=float, default=0.8)
    parser.add_argument("--segment-settle-s", type=float, default=0.35)
    parser.add_argument("--command-retries", type=int, default=3)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    """Validate arguments and delegate to the importable waist controller."""

    args = parse_args()
    try:
        ok = WaistController().move_to_json(
            Path(args.json),
            joint_speed_radps=args.joint_speed_radps,
            max_step_rad=args.max_step_rad,
            settle_tol_rad=args.settle_tol_rad,
            settle_timeout_s=args.settle_timeout_s,
            poll_s=args.poll_s,
            settle_s=args.settle_s,
            segment_settle_s=args.segment_settle_s,
            command_retries=args.command_retries,
            dry_run=args.dry_run,
        )
        return 0 if ok else 1
    except Exception as exc:
        print(f"move_waist_by_json_path failed: {type(exc).__name__}: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
