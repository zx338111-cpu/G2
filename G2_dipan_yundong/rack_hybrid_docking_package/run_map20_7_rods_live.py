#!/usr/bin/env python3
"""One-command launcher for the map20 G2 seven-rods live workflow.

This local wrapper SSHes to the robot, runs read-only preflight checks, creates
a fresh timestamped checkpoint/log pair, runs rods 1-7 with the optimized
robot-side Python runner, then analyzes the resulting log.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import os
from pathlib import Path
import shlex
import subprocess
import sys
import textwrap


DEFAULT_HOST = "agi@192.168.0.7"
DEFAULT_REMOTE_DIR = "/data/g2_industrial_cell_20260612/wxf/BOX_528_1"


REMOTE_SCRIPT = r"""set -euo pipefail

source /home/agi/app/env.sh
cd "$REMOTE_DIR"
mkdir -p logs

echo "[remote] workspace: $PWD"
echo "[remote] checking map id and tuned live parameters"

python3 - <<'PY'
import importlib.util
import json
from pathlib import Path

config_path = Path("rack_hybrid_docking_package/industrial_station_config.json")
config = json.loads(config_path.read_text(encoding="utf-8"))
if config.get("map_id") != 20:
    raise SystemExit(f"expected map_id=20 in {config_path}, got {config.get('map_id')!r}")

runner_path = Path("rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py")
spec = importlib.util.spec_from_file_location("optimized_runner", runner_path)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(module)
tuned = module.TUNED
expected = {
    "pick_down_z_m": "0.0",
    "place_final_before_open_x_m": "0.03",
    "place_final_before_open_z_m": "-0.025",
    "place_pull_back_down_x_m": "-0.02",
    "place_pull_back_down_z_m": "-0.01",
    "place_pull_drop_after_x_m": "-0.06",
    "place_pull_drop_z_m": "-0.07",
}
bad = {key: (tuned.get(key), value) for key, value in expected.items() if str(tuned.get(key)) != value}
if bad:
    raise SystemExit(f"runner tuned parameters do not match the expected map20 release/pull-out state: {bad}")
if not tuned.get("skip_place_pose_after_transition2"):
    raise SystemExit("expected skip_place_pose_after_transition2=True")
print(json.dumps({"event": "script_tuned_parameter_check", "ok": True, "map_id": config.get("map_id")}, ensure_ascii=False))
PY

echo "[remote] compiling critical Python files"
python3 -m py_compile \
  industrial_status_snapshot.py \
  rack_hybrid_docking_package/industrial_map_nav_guarded.py \
  rack_hybrid_docking_package/industrial_cell_mission_controller.py \
  rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  rack_hybrid_docking_package/analyze_industrial_cell_run.py

echo "[remote] checking for existing mission/motion processes"
if pgrep -af 'industrial_cell_7_rods_optimized.py|industrial_cell_mission_controller.py|industrial_map_nav_guarded.py|move_(arm|waist|ee)|rack_industrial_docking.py' >/tmp/map20_7rods_existing_processes.txt; then
  echo "[remote] BLOCKED: existing mission/motion-related process found"
  cat /tmp/map20_7rods_existing_processes.txt
  exit 20
fi

if [[ -e "$CHECKPOINT_FILE" ]]; then
  echo "[remote] BLOCKED: checkpoint already exists, refusing to reuse stale checkpoint: $CHECKPOINT_FILE"
  exit 21
fi
if [[ -e "$RUN_LOG" ]]; then
  echo "[remote] BLOCKED: run log already exists, refusing to append to old run log: $RUN_LOG"
  exit 22
fi

echo "[remote] read-only status snapshot -> $SNAPSHOT_LOG"
python3 industrial_status_snapshot.py --samples 3 --interval-s 0.1 | tee "$SNAPSHOT_LOG"

echo "[remote] read-only readiness check"
python3 rack_hybrid_docking_package/industrial_map_nav_guarded.py \
  --config rack_hybrid_docking_package/industrial_station_config.json \
  --readiness-check

if [[ "$PREFLIGHT_ONLY" == "1" ]]; then
  echo "[remote] preflight-only complete; no physical motion executed"
  exit 0
fi

echo "[remote] starting live seven-rods run"
set +e
python3 -u rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --live \
  --init \
  --start-index "$START_INDEX" \
  --end-index "$END_INDEX" \
  --checkpoint-file "$CHECKPOINT_FILE" \
  --run-log "$RUN_LOG"
run_rc=$?
set -e

echo "[remote] final checkpoint status"
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --status-only \
  --dry-run-keep-checkpoint \
  --checkpoint-file "$CHECKPOINT_FILE" || true

if [[ -f "$RUN_LOG" ]]; then
  echo "[remote] analyzing run log -> $SUMMARY_LOG"
  python3 rack_hybrid_docking_package/analyze_industrial_cell_run.py "$RUN_LOG" | tee "$SUMMARY_LOG" || true

  echo "[remote] scanning run log for obvious failures"
  if grep -En 'Traceback|Exception|KeyboardInterrupt|"return_code": [1-9]|return_code=[1-9]' "$RUN_LOG"; then
    echo "[remote] WARN: failure-like text was found in $RUN_LOG"
  else
    echo "[remote] no obvious failure text found in $RUN_LOG"
  fi
else
  echo "[remote] WARN: run log was not created: $RUN_LOG"
fi

echo "[remote] checkpoint: $CHECKPOINT_FILE"
echo "[remote] run_log: $RUN_LOG"
echo "[remote] snapshot_log: $SNAPSHOT_LOG"
echo "[remote] summary_log: $SUMMARY_LOG"
exit "$run_rc"
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yes", action="store_true", help="Skip the local physical-motion confirmation prompt")
    parser.add_argument("--preflight-only", action="store_true", help="Run checks only; do not execute robot motion")
    parser.add_argument("--host", default=os.environ.get("ROBOT_HOST", DEFAULT_HOST), help=f"SSH target, default {DEFAULT_HOST}")
    parser.add_argument("--remote-dir", default=os.environ.get("REMOTE_DIR", DEFAULT_REMOTE_DIR), help="Robot workspace")
    parser.add_argument("--start-index", type=int, default=int(os.environ.get("START_INDEX", "1")))
    parser.add_argument("--end-index", type=int, default=int(os.environ.get("END_INDEX", "7")))
    parser.add_argument("--checkpoint-file", default=os.environ.get("CHECKPOINT_FILE", ""))
    parser.add_argument("--run-log", default=os.environ.get("RUN_LOG", ""))
    parser.add_argument("--snapshot-log", default=os.environ.get("SNAPSHOT_LOG", ""))
    parser.add_argument("--summary-log", default=os.environ.get("SUMMARY_LOG", ""))
    return parser.parse_args()


def quote_env(values: dict[str, str]) -> str:
    return " ".join(f"{key}={shlex.quote(value)}" for key, value in values.items())


def confirm_motion(args: argparse.Namespace) -> None:
    if args.preflight_only or args.yes:
        return
    print()
    print("This will execute physical robot motion.")
    print("Confirm the robot is on map20, at a safe start/home state, upper body is clear, and there is no interference.")
    answer = input("Type RUN_MAP20_7RODS to continue: ").strip()
    if answer != "RUN_MAP20_7RODS":
        raise SystemExit("aborted")


def main() -> int:
    args = parse_args()
    if not (1 <= args.start_index <= 7 and 1 <= args.end_index <= 7 and args.start_index <= args.end_index):
        raise SystemExit("rod range must satisfy 1 <= start <= end <= 7")

    stamp = os.environ.get("RUN_STAMP") or datetime.now().strftime("%Y%m%d_%H%M%S")
    checkpoint_file = args.checkpoint_file or f"logs/live_map20_full_{args.start_index}_{args.end_index}_{stamp}_checkpoint.json"
    run_log = args.run_log or f"logs/live_map20_full_{args.start_index}_{args.end_index}_{stamp}.log"
    snapshot_log = args.snapshot_log or f"logs/live_map20_full_{args.start_index}_{args.end_index}_{stamp}_preflight_snapshot.log"
    summary_log = args.summary_log or f"logs/live_map20_full_{args.start_index}_{args.end_index}_{stamp}_analysis.txt"

    print(f"robot:        {args.host}")
    print(f"remote_dir:   {args.remote_dir}")
    print(f"rod_range:    {args.start_index}-{args.end_index}")
    print(f"checkpoint:   {checkpoint_file}")
    print(f"run_log:      {run_log}")
    print(f"snapshot_log: {snapshot_log}")
    print(f"summary_log:  {summary_log}")

    confirm_motion(args)

    env = {
        "REMOTE_DIR": args.remote_dir,
        "START_INDEX": str(args.start_index),
        "END_INDEX": str(args.end_index),
        "CHECKPOINT_FILE": checkpoint_file,
        "RUN_LOG": run_log,
        "SNAPSHOT_LOG": snapshot_log,
        "SUMMARY_LOG": summary_log,
        "PREFLIGHT_ONLY": "1" if args.preflight_only else "0",
    }
    remote_command = f"{quote_env(env)} bash -s"
    proc = subprocess.run(["ssh", args.host, remote_command], input=REMOTE_SCRIPT, text=True)
    return proc.returncode


if __name__ == "__main__":
    raise SystemExit(main())
