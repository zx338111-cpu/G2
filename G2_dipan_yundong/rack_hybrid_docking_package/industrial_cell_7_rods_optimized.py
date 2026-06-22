#!/usr/bin/env python3
"""One-command optimized runner for the G2 seven-rod industrial cell flow.

This script is a thin, explicit orchestrator over industrial_cell_mission_controller.py.
It keeps the tuned live parameters in one place and lets the checkpoint controller
own the detailed navigation, fine positioning, arm, gripper, and waist steps.

Default mode is no-motion dry-run. Use --live for physical execution.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


PACKAGE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_DIR.parent
CONTROLLER = PACKAGE_DIR / "industrial_cell_mission_controller.py"
MAP_NAV = PACKAGE_DIR / "industrial_map_nav_guarded.py"
CONFIG = PACKAGE_DIR / "industrial_station_config.json"
LOG_DIR = PROJECT_ROOT / "logs"
DEFAULT_CHECKPOINT = LOG_DIR / "industrial_cell_7_rods_optimized_checkpoint.json"


TUNED = {
    # Fast and safe: waist/body joints are free-space moves, now with segmented retry.
    "waist_joint_speed_radps": "0.75",
    "arm_joint_speed_radps": "0.12",
    "arm_settle_s": "0.80",
    "waist_settle_s": "0.40",
    "offset_settle_s": "0.30",
    # Stable: final approach to rack/place is sensor controlled and slower.
    "grab_final_stop_mm": "328",
    "grab_final_brake_margin_mm": "20",
    "grab_final_speed_mps": "0.08",
    "place_final_stop_mm": "308",
    "place_final_brake_margin_mm": "20",
    "place_final_speed_mps": "0.15",
    # Pick: first 8.5 cm backward with no Z dip, then finish the 20 cm pull.
    "pick_down_z_m": "0.0",
    "pick_back_x_m": "-0.20",
    "pick_back_down_x_m": "-0.085",
    # Place points already include insertion geometry; do not add chassis forward offset after fine positioning.
    "place_forward_after_fine_m": "0.0",
    # Place release: transition2 is now the transition point; final release moves both arms
    # forward 3 cm and down 2.5 cm before opening.
    "place_transition2_json": str(PACKAGE_DIR / "calibration_records" / "rod07_place_transition2_arm_latest.json"),
    "place_pose_json": str(PACKAGE_DIR / "calibration_records" / "rod07_place_final_arm_up050_latest.json"),
    "skip_place_pose_after_transition2": True,
    "place_final_before_open_x_m": "0.03",
    "place_final_before_open_z_m": "-0.025",
    "place_raise_before_open_z_m": "0.0",
    # Place pull-out: back 2 cm while lowering 1 cm, reach 6 cm, then drop 7 cm.
    # Total post-release downward clearance is 8 cm during the pull-out phase.
    "place_pull_back_down_x_m": "-0.02",
    "place_pull_back_down_z_m": "-0.01",
    "place_pull_drop_after_x_m": "-0.06",
    "place_pull_drop_z_m": "-0.07",
    "chassis_relative_max_abs_m": "0.20",
    "chassis_relative_timeout_s": "12.0",
    # Retreats: quick enough for cycle time, still relative-motion bounded.
    "local_retreat_m": "0.45",
    "local_retreat_speed_mps": "0.20",
    # Navigation yaw refine: low speed, tight enough for the shelf poses.
    "refine_yaw_tolerance_deg": "1.5",
    "refine_yaw_max_error_deg": "10.0",
    "refine_yaw_angular_speed_radps": "0.05",
    "refine_yaw_fine_angular_speed_radps": "0.02",
    "refine_yaw_timeout_s": "12.0",
    "refine_yaw_stations": "GRAB_PRE,PLACE_PRE,RECOVERY_SAFE,HOME_SAFE",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true", help="Execute physical robot motion")
    parser.add_argument("--init", action="store_true", help="Reset checkpoint before running")
    parser.add_argument("--status-only", action="store_true", help="Print current checkpoint status and exit")
    parser.add_argument("--start-index", type=int, default=1)
    parser.add_argument("--end-index", type=int, default=7)
    parser.add_argument("--stop-after-rod", type=int, default=0, help="Stop after this rod index, 0 means no limit")
    parser.add_argument("--checkpoint-file", default=str(DEFAULT_CHECKPOINT))
    parser.add_argument("--log-dir", default=str(LOG_DIR))
    parser.add_argument("--run-log", default="", help="Combined wrapper log file; default is logs/optimized timestamp")
    parser.add_argument("--skip-readiness-check", action="store_true")
    parser.add_argument("--skip-file-check", action="store_true")
    parser.add_argument(
        "--refine-yaw-stations",
        default=TUNED["refine_yaw_stations"],
        help="Comma-separated station names where live yaw refine is applied; use 'none' to disable all refinements.",
    )
    parser.add_argument(
        "--direct-home-after-place",
        action="store_true",
        help="Experimental validation mode: after LOCAL_PLACE, skip RECOVERY_SAFE and navigate directly to HOME_SAFE.",
    )
    parser.add_argument(
        "--allow-holding-resume",
        action="store_true",
        help="Allow live resume when checkpoint says holding_rod=true; use only after physical state is confirmed.",
    )
    parser.add_argument(
        "--dry-run-keep-checkpoint",
        action="store_true",
        help="In dry-run mode, reuse the given checkpoint instead of initializing a temporary no-motion run.",
    )
    return parser.parse_args()


def event(name: str, **fields: object) -> None:
    print(json.dumps({"event": name, **fields}, ensure_ascii=False), flush=True)


def json_from_stdout(stdout: str) -> dict[str, object]:
    start = stdout.find("{")
    if start < 0:
        raise RuntimeError(f"command did not print JSON: {stdout[-500:]}")
    return json.loads(stdout[start:])


def run_capture(command: list[str], *, cwd: Path) -> dict[str, object]:
    proc = subprocess.run(command, cwd=str(cwd), text=True, capture_output=True)
    if proc.stdout:
        print(proc.stdout, end="" if proc.stdout.endswith("\n") else "\n", flush=True)
    if proc.stderr:
        print(proc.stderr, end="" if proc.stderr.endswith("\n") else "\n", file=sys.stderr, flush=True)
    if proc.returncode != 0:
        raise RuntimeError(f"command failed return_code={proc.returncode}: {' '.join(command)}")
    return json_from_stdout(proc.stdout)


def stream_command(command: list[str], *, cwd: Path, run_log: Path) -> int:
    run_log.parent.mkdir(parents=True, exist_ok=True)
    event("command_start", command=command)
    with run_log.open("a", encoding="utf-8") as log:
        log.write(json.dumps({"event": "command_start", "command": command}, ensure_ascii=False) + "\n")
        proc = subprocess.Popen(
            command,
            cwd=str(cwd),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert proc.stdout is not None
        try:
            for line in proc.stdout:
                print(line, end="", flush=True)
                log.write(line)
        except KeyboardInterrupt:
            proc.terminate()
            event("command_interrupted", command=command)
            raise
        return_code = proc.wait()
        log.write(json.dumps({"event": "command_done", "return_code": return_code}, ensure_ascii=False) + "\n")
    event("command_done", return_code=return_code)
    return return_code


def controller_base_args(args: argparse.Namespace, *, local_mode: str) -> list[str]:
    log_dir = str(Path(args.log_dir).resolve())
    base = [
        sys.executable,
        str(CONTROLLER),
        "--config",
        str(CONFIG),
        "--checkpoint-file",
        str(Path(args.checkpoint_file).resolve()),
        "--staging",
        "--local-action-mode",
        local_mode,
        "--allow-estop-pedal-fault",
        "--nav-log-dir",
        log_dir,
        "--full-local-log-dir",
        log_dir,
        "--arm-joint-speed-radps",
        TUNED["arm_joint_speed_radps"],
        "--waist-joint-speed-radps",
        TUNED["waist_joint_speed_radps"],
        "--arm-settle-s",
        TUNED["arm_settle_s"],
        "--waist-settle-s",
        TUNED["waist_settle_s"],
        "--offset-settle-s",
        TUNED["offset_settle_s"],
        "--grab-final-stop-mm",
        TUNED["grab_final_stop_mm"],
        "--grab-final-brake-margin-mm",
        TUNED["grab_final_brake_margin_mm"],
        "--grab-final-speed-mps",
        TUNED["grab_final_speed_mps"],
        "--place-final-stop-mm",
        TUNED["place_final_stop_mm"],
        "--place-final-brake-margin-mm",
        TUNED["place_final_brake_margin_mm"],
        "--place-final-speed-mps",
        TUNED["place_final_speed_mps"],
        "--pick-down-z-m",
        TUNED["pick_down_z_m"],
        "--pick-back-x-m",
        TUNED["pick_back_x_m"],
        "--pick-back-down-x-m",
        TUNED["pick_back_down_x_m"],
        "--skip-pick-down-after-close",
        "--place-forward-after-fine-m",
        TUNED["place_forward_after_fine_m"],
        "--place-transition2-json",
        TUNED["place_transition2_json"],
        "--place-final-before-open-x-m",
        TUNED["place_final_before_open_x_m"],
        "--place-final-before-open-z-m",
        TUNED["place_final_before_open_z_m"],
        "--place-raise-before-open-z-m",
        TUNED["place_raise_before_open_z_m"],
        "--place-pull-back-down-x-m",
        TUNED["place_pull_back_down_x_m"],
        "--place-pull-back-down-z-m",
        TUNED["place_pull_back_down_z_m"],
        "--place-pull-drop-after-x-m",
        TUNED["place_pull_drop_after_x_m"],
        "--place-pull-drop-z-m",
        TUNED["place_pull_drop_z_m"],
        "--chassis-relative-max-abs-m",
        TUNED["chassis_relative_max_abs_m"],
        "--chassis-relative-timeout-s",
        TUNED["chassis_relative_timeout_s"],
        "--local-retreat-m",
        TUNED["local_retreat_m"],
        "--local-retreat-speed-mps",
        TUNED["local_retreat_speed_mps"],
    ]
    if args.direct_home_after_place:
        base.append("--direct-home-after-place")
    if TUNED["skip_place_pose_after_transition2"]:
        base.append("--skip-place-pose-after-transition2")
    else:
        base += ["--place-pose-json", TUNED["place_pose_json"]]
    if args.live:
        base += [
            "--confirm-live",
            "--confirm-local-physical",
            "--refine-yaw",
            "--refine-yaw-stations",
            args.refine_yaw_stations,
            "--refine-yaw-tolerance-deg",
            TUNED["refine_yaw_tolerance_deg"],
            "--refine-yaw-max-error-deg",
            TUNED["refine_yaw_max_error_deg"],
            "--refine-yaw-angular-speed-radps",
            TUNED["refine_yaw_angular_speed_radps"],
            "--refine-yaw-fine-angular-speed-radps",
            TUNED["refine_yaw_fine_angular_speed_radps"],
            "--refine-yaw-timeout-s",
            TUNED["refine_yaw_timeout_s"],
        ]
    return base


def status(args: argparse.Namespace) -> dict[str, object]:
    command = [
        sys.executable,
        str(CONTROLLER),
        "--config",
        str(CONFIG),
        "--checkpoint-file",
        str(Path(args.checkpoint_file).resolve()),
        "--staging",
        "--status",
    ]
    return run_capture(command, cwd=PROJECT_ROOT)


def init_checkpoint(args: argparse.Namespace) -> None:
    command = [
        sys.executable,
        str(CONTROLLER),
        "--config",
        str(CONFIG),
        "--checkpoint-file",
        str(Path(args.checkpoint_file).resolve()),
        "--start-index",
        str(args.start_index),
        "--end-index",
        str(args.end_index),
        "--init",
        "--staging",
        "--status",
    ]
    run_capture(command, cwd=PROJECT_ROOT)


def readiness_check(args: argparse.Namespace, run_log: Path) -> None:
    command = [
        sys.executable,
        str(MAP_NAV),
        "--config",
        str(CONFIG),
        "--readiness-check",
    ]
    return_code = stream_command(command, cwd=PROJECT_ROOT, run_log=run_log)
    if return_code != 0:
        raise RuntimeError(f"readiness check failed return_code={return_code}")


def latest_grab_pose_exists(rod_index: int) -> bool:
    return bool(sorted((PACKAGE_DIR / "calibration_records").glob(f"rod{rod_index:02d}_grab_pose_*.json")))


def check_files(args: argparse.Namespace) -> None:
    required = [
        CONTROLLER,
        MAP_NAV,
        CONFIG,
        PROJECT_ROOT / "move_waist_by_json_path.py",
        PROJECT_ROOT / "move_arm_by_json_path.py",
        PROJECT_ROOT / "move_ee_pose_open_2.py",
        PROJECT_ROOT / "move_ee_pose_close_2.py",
        PROJECT_ROOT / "move_ee_relative_offset.py",
        PACKAGE_DIR / "rack_industrial_docking.py",
        PACKAGE_DIR / "calibration_records" / "rod07_place_waist_adjusted_latest.json",
        PACKAGE_DIR / "calibration_records" / "rod07_place_above_arm_latest.json",
        PACKAGE_DIR / "calibration_records" / "rod07_place_transition_arm_latest.json",
        Path(TUNED["place_transition2_json"]),
        Path("/data/wxf/wxf/positions/arm_default.json"),
    ]
    if not TUNED["skip_place_pose_after_transition2"]:
        required.append(Path(TUNED["place_pose_json"]))
    missing = [str(path) for path in required if not path.exists()]
    for rod_index in range(args.start_index, args.end_index + 1):
        if not latest_grab_pose_exists(rod_index):
            missing.append(str(PACKAGE_DIR / "calibration_records" / f"rod{rod_index:02d}_grab_pose_*.json"))
    if missing:
        raise RuntimeError("missing required files:\n" + "\n".join(missing))


def run_current_rod(args: argparse.Namespace, run_log: Path) -> None:
    local_mode = "full" if args.live else "full-dry-run"
    command = controller_base_args(args, local_mode=local_mode) + ["--run-current-rod"]
    return_code = stream_command(command, cwd=PROJECT_ROOT, run_log=run_log)
    if return_code != 0:
        raise RuntimeError(f"rod execution failed return_code={return_code}")


def state_tuple(status_obj: dict[str, object]) -> tuple[object, ...]:
    state = status_obj["state"]
    assert isinstance(state, dict)
    return (
        state.get("rod_index"),
        state.get("phase"),
        state.get("holding_rod"),
        state.get("current_station"),
        state.get("last_success_step"),
    )


def main() -> int:
    args = parse_args()
    if not (1 <= args.start_index <= 7 and 1 <= args.end_index <= 7):
        raise SystemExit("start/end index must be in 1..7")
    if args.start_index > args.end_index:
        raise SystemExit("--start-index must be <= --end-index")
    if args.stop_after_rod and not (args.start_index <= args.stop_after_rod <= args.end_index):
        raise SystemExit("--stop-after-rod must be inside start/end range")

    if not args.run_log:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        args.run_log = str(Path(args.log_dir).resolve() / f"industrial_cell_7_rods_optimized_{stamp}.log")
    run_log = Path(args.run_log).resolve()
    checkpoint = Path(args.checkpoint_file)

    if args.status_only and not checkpoint.exists():
        raise SystemExit(f"--status-only checkpoint missing: {checkpoint}")

    if not args.live and not args.dry_run_keep_checkpoint and not args.status_only:
        args.init = True
        if checkpoint == DEFAULT_CHECKPOINT:
            args.checkpoint_file = str(LOG_DIR / "industrial_cell_7_rods_optimized_dryrun_checkpoint.json")

    event(
        "optimized_runner_start",
        live=args.live,
        checkpoint=str(Path(args.checkpoint_file).resolve()),
        run_log=str(run_log),
        start_index=args.start_index,
        end_index=args.end_index,
    )

    if not args.skip_file_check and not args.status_only:
        check_files(args)

    if args.init:
        init_checkpoint(args)

    current = status(args)
    if args.status_only:
        return 0

    if args.live and not args.skip_readiness_check:
        readiness_check(args, run_log)

    rods_started = 0
    while True:
        current = status(args)
        state = current["state"]
        assert isinstance(state, dict)
        phase = state.get("phase")
        rod_index = int(state.get("rod_index", 0))
        if phase == "MISSION_DONE":
            event("optimized_runner_done", state=state)
            return 0
        if args.stop_after_rod and rod_index > args.stop_after_rod:
            event("optimized_runner_stopped_by_limit", state=state, stop_after_rod=args.stop_after_rod)
            return 0
        if args.live and state.get("holding_rod") and not args.allow_holding_resume:
            event(
                "optimized_runner_blocked_holding_resume",
                state=state,
                note="checkpoint says holding_rod=true; confirm physical state and rerun with --allow-holding-resume",
            )
            return 2

        before = state_tuple(current)
        run_current_rod(args, run_log)
        rods_started += 1
        after_status = status(args)
        after = state_tuple(after_status)
        if after == before:
            event("optimized_runner_no_progress", state=after_status.get("state"))
            return 3


if __name__ == "__main__":
    raise SystemExit(main())
