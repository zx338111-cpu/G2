#!/usr/bin/env python3
"""Standalone validation for the strict front-ultrasonic 1m retreat primitive."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import sys

import industrial_7_rods_total_controller as total


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate only the strict front-ultrasonic retreat primitive. "
            "No arm scripts, turns, rack approach, or rod sequence are executed."
        )
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--dry-run",
        action="store_true",
        help="Print and record the validation plan only; this is also the default.",
    )
    mode.add_argument(
        "--confirm-live",
        action="store_true",
        help="Allow the real chassis retreat after all controller safety gates pass.",
    )
    parser.add_argument(
        "--base-dir",
        default=str(total.detect_default_base_dir()),
        help="BOX_528_1 root directory; defaults to the total-controller detector.",
    )
    parser.add_argument("--distance-m", type=float, default=1.0, help="Retreat target distance.")
    parser.add_argument(
        "--retreat-escape-delta-m",
        type=float,
        default=0.0,
        help=(
            "Pass-through for the total controller's hybrid escape distance. "
            "Standalone short corrections default to 0 so distances below 0.22m are valid."
        ),
    )
    parser.add_argument(
        "--grab-retreat-front-occlusion-escape-m",
        type=float,
        default=0.0,
        help=(
            "Pass-through for grab-retreat occlusion escape. Standalone validation calls "
            "the strict front-ultrasonic primitive directly, so short corrections default to 0."
        ),
    )
    parser.add_argument(
        "--tolerance-mm",
        type=int,
        default=20,
        help="Allowed front ultrasonic target-delta error; production default is 20mm.",
    )
    parser.add_argument(
        "--odom-tolerance-m",
        type=float,
        default=0.02,
        help="Allowed SLAM odom displacement error; production default is 0.02m.",
    )
    parser.add_argument(
        "--retreat-front-delta-consistency-mm",
        type=int,
        default=180,
        help="Maximum allowed delta disagreement between front ultrasonic IDs 0 and 1.",
    )
    parser.add_argument(
        "--retreat-speed-mps",
        type=float,
        default=0.50,
        help="Retreat speed passed through to the total controller config.",
    )
    parser.add_argument(
        "--max-duration-s",
        type=float,
        default=20.0,
        help="Maximum live retreat duration before the primitive fails closed.",
    )
    parser.add_argument(
        "--allow-estop-pedal-fault",
        action="store_true",
        default=True,
        help="Compatibility pass-through; physical emergency_stop_pedal_state is still checked.",
    )
    parser.add_argument(
        "--strict-estop-pedal-fault",
        dest="allow_estop_pedal_fault",
        action="store_false",
        help="Compatibility pass-through to the total controller parser.",
    )
    parser.add_argument(
        "--no-retreat-require-odom-crosscheck",
        action="store_true",
        help="Diagnostic downgrade only. Production validation should keep odom required.",
    )
    parser.add_argument("--log-file", default=None, help="Log file path.")
    parser.add_argument("--event-file", default=None, help="JSONL event file path.")
    parser.add_argument("--checkpoint-file", default=None, help="Checkpoint JSON path.")
    parser.add_argument("--report-file", default=None, help="Final report JSON path.")
    args = parser.parse_args()

    if args.distance_m <= 0:
        raise SystemExit("--distance-m must be positive")
    if args.retreat_escape_delta_m < 0:
        raise SystemExit("--retreat-escape-delta-m must be >= 0")
    if args.retreat_escape_delta_m >= args.distance_m:
        raise SystemExit("--retreat-escape-delta-m must be smaller than --distance-m")
    if args.grab_retreat_front_occlusion_escape_m < 0:
        raise SystemExit("--grab-retreat-front-occlusion-escape-m must be >= 0")
    if args.grab_retreat_front_occlusion_escape_m >= args.distance_m:
        raise SystemExit("--grab-retreat-front-occlusion-escape-m must be smaller than --distance-m")
    if args.tolerance_mm <= 0:
        raise SystemExit("--tolerance-mm must be positive")
    if args.tolerance_mm > 100:
        raise SystemExit("--tolerance-mm is capped at 100mm")
    if args.odom_tolerance_m < 0:
        raise SystemExit("--odom-tolerance-m must be >= 0")
    if args.retreat_front_delta_consistency_mm < 0:
        raise SystemExit("--retreat-front-delta-consistency-mm must be >= 0")
    if args.retreat_speed_mps <= 0:
        raise SystemExit("--retreat-speed-mps must be positive")
    if args.max_duration_s <= 0:
        raise SystemExit("--max-duration-s must be positive")
    return args


def default_artifacts(base_dir: Path, log_file: str | None) -> dict[str, Path]:
    if log_file is not None:
        log_path = Path(log_file).resolve()
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = base_dir / "logs" / f"industrial_retreat_1m_validation_{stamp}.log"
    return {
        "log_file": log_path,
        "event_file": log_path.with_suffix(".jsonl"),
        "checkpoint_file": log_path.with_name(f"{log_path.stem}_checkpoint.json"),
        "report_file": log_path.with_name(f"{log_path.stem}_report.json"),
    }


def parse_total_config(args: argparse.Namespace) -> total.RuntimeConfig:
    base_dir = Path(args.base_dir).resolve()
    artifacts = default_artifacts(base_dir, args.log_file)

    event_file = Path(args.event_file).resolve() if args.event_file else artifacts["event_file"]
    checkpoint_file = (
        Path(args.checkpoint_file).resolve() if args.checkpoint_file else artifacts["checkpoint_file"]
    )
    report_file = Path(args.report_file).resolve() if args.report_file else artifacts["report_file"]

    total_argv = [
        "industrial_7_rods_total_controller.py",
        "--base-dir",
        str(base_dir),
        "--start-index",
        "1",
        "--end-index",
        "1",
        "--retreat-method",
        "front-ultrasonic",
        "--retreat-distance-m",
        str(args.distance_m),
        "--retreat-escape-delta-m",
        str(args.retreat_escape_delta_m),
        "--grab-retreat-front-occlusion-escape-m",
        str(args.grab_retreat_front_occlusion_escape_m),
        "--retreat-target-tolerance-mm",
        str(args.tolerance_mm),
        "--retreat-odom-tolerance-m",
        str(args.odom_tolerance_m),
        "--retreat-front-delta-consistency-mm",
        str(args.retreat_front_delta_consistency_mm),
        "--retreat-speed-mps",
        str(args.retreat_speed_mps),
        "--log-file",
        str(artifacts["log_file"]),
        "--event-file",
        str(event_file),
        "--checkpoint-file",
        str(checkpoint_file),
        "--report-file",
        str(report_file),
    ]
    if args.confirm_live:
        total_argv.append("--confirm-live")
    else:
        total_argv.append("--dry-run")
    if args.allow_estop_pedal_fault:
        total_argv.append("--allow-estop-pedal-fault")
    else:
        total_argv.append("--strict-estop-pedal-fault")
    if args.no_retreat_require_odom_crosscheck:
        total_argv.append("--no-retreat-require-odom-crosscheck")

    old_argv = sys.argv[:]
    try:
        sys.argv = total_argv
        return total.parse_args()
    finally:
        sys.argv = old_argv


def run_validation(
    controller: total.Industrial7RodsController,
    args: argparse.Namespace,
) -> None:
    controller.recorder.name = "industrial_retreat_1m_validation"
    controller.next_step(f"standalone strict retreat {args.distance_m:.3f}m")
    controller.emit_event(
        "standalone_retreat_validation_config",
        distance_m=args.distance_m,
        retreat_escape_delta_m=args.retreat_escape_delta_m,
        grab_retreat_front_occlusion_escape_m=args.grab_retreat_front_occlusion_escape_m,
        tolerance_mm=args.tolerance_mm,
        odom_tolerance_m=args.odom_tolerance_m,
        odom_required=not args.no_retreat_require_odom_crosscheck,
        confirm_live=args.confirm_live,
        max_duration_s=args.max_duration_s,
    )

    if controller.config.dry_run:
        controller.log(
            "dry_run: standalone retreat validation plan only; "
            "no chassis, arm, turn, rack approach, or rod sequence motion will run"
        )
        controller.complete_current_step(
            "completed",
            retreat_distance_m=args.distance_m,
            retreat_target_tolerance_mm=args.tolerance_mm,
            retreat_odom_tolerance_m=args.odom_tolerance_m,
            odom_required=controller.config.retreat_require_odom_crosscheck,
        )
        return

    if args.no_retreat_require_odom_crosscheck:
        controller.log(
            "warning: odom crosscheck is disabled for this diagnostic run; "
            "do not use this as production 1m validation evidence"
        )
    controller.require_live_allowed()
    delta_by_id = controller._retreat_by_front_ultrasonic_delta(
        distance_m=args.distance_m,
        tolerance_mm=args.tolerance_mm,
        max_duration_s=args.max_duration_s,
    )
    controller.complete_current_step(
        "completed",
        retreat_distance_m=args.distance_m,
        retreat_target_tolerance_mm=args.tolerance_mm,
        retreat_odom_tolerance_m=args.odom_tolerance_m,
        odom_required=controller.config.retreat_require_odom_crosscheck,
        delta_by_id=delta_by_id,
    )


def main() -> None:
    args = parse_args()
    controller = total.Industrial7RodsController(parse_total_config(args))
    try:
        run_validation(controller, args)
    except Exception as exc:
        controller.fail_current_step(exc)
        controller.write_final_report("failed", exc)
        controller.log(f"standalone_retreat_validation_failed: {type(exc).__name__}: {exc}")
        raise
    else:
        controller.write_final_report("completed")


if __name__ == "__main__":
    main()
