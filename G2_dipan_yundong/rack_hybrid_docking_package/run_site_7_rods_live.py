#!/usr/bin/env python3
"""Profile-aware launcher for the G2 seven-rods industrial-cell workflow.

This wrapper keeps the operator-facing command site-based instead of map20
hard-coded. It validates the local profile first, then SSHes to the robot and
checks that the active robot-side files match the expected map/profile shape
before running preflight or live motion.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys


PACKAGE_DIR = Path(__file__).resolve().parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from validate_site_profile import DEFAULT_PROFILE, validate_profile
from process_vision_capture import DEFAULT_CAMERAS, parse_camera_list


REMOTE_SCRIPT = r"""set -eo pipefail

source /home/agi/app/env.sh
set -u
cd "$REMOTE_DIR"
mkdir -p logs

echo "[remote] workspace: $PWD"
echo "[remote] validating active robot-side site files"

python3 - <<'PY'
import importlib.util
import json
import math
import os
import sys
from pathlib import Path

expected_map_id = int(os.environ["EXPECTED_MAP_ID"])
remote_profile = Path(os.environ["REMOTE_PROFILE"])
required_files = json.loads(os.environ["REQUIRED_REMOTE_FILES_JSON"])
expected_tuned = json.loads(os.environ["EXPECTED_TUNED_JSON"])

missing = [path for path in required_files if not Path(path).exists()]
if missing:
    raise SystemExit(f"missing required robot-side profile files: {missing}")

profile = json.loads(remote_profile.read_text(encoding="utf-8"))
station_config = profile.get("station_config")
if not isinstance(station_config, str) or not station_config:
    raise SystemExit(f"profile.station_config is invalid: {remote_profile}")
config_path = remote_profile.parent / station_config
config = json.loads(config_path.read_text(encoding="utf-8"))
if int(config.get("map_id")) != expected_map_id:
    raise SystemExit(f"map_id mismatch: robot config={config.get('map_id')!r}, profile={expected_map_id!r}")

runner_path = Path("rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py")
spec = importlib.util.spec_from_file_location("industrial_cell_7_rods_single_debug", runner_path)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = module
spec.loader.exec_module(module)
module.apply_site_profile(remote_profile)

actual_tuned = module.TUNED
bad = {}
for key, expected in expected_tuned.items():
    actual = actual_tuned.get(key)
    if isinstance(expected, list):
        if sorted(actual) != sorted(expected):
            bad[key] = {"actual": sorted(actual), "expected": sorted(expected)}
        continue
    if isinstance(expected, (int, float)):
        if not isinstance(actual, (int, float)) or not math.isclose(float(actual), float(expected), rel_tol=0.0, abs_tol=1e-9):
            bad[key] = {"actual": actual, "expected": expected}
        continue
    if actual != expected:
        bad[key] = {"actual": actual, "expected": expected}
if bad:
    raise SystemExit(f"runner TUNED values differ from profile: {bad}")

print(json.dumps({
    "event": "site_profile_remote_check",
    "ok": True,
    "profile": str(remote_profile),
    "map_id": config.get("map_id"),
    "required_file_count": len(required_files),
}, ensure_ascii=False))
PY

echo "[remote] compiling critical Python files"
python3 -m py_compile \
  rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py \
  rack_hybrid_docking_package/site_profile.py \
  rack_hybrid_docking_package/industrial_map_nav_guarded.py \
  rack_hybrid_docking_package/process_vision_capture.py \
  rack_hybrid_docking_package/analyze_industrial_cell_run.py

if [[ -f industrial_status_snapshot.py ]]; then
  echo "[remote] read-only status snapshot -> $SNAPSHOT_LOG"
  python3 industrial_status_snapshot.py --samples 3 --interval-s 0.1 | tee "$SNAPSHOT_LOG"
else
  echo "[remote] WARN: industrial_status_snapshot.py not found; skipping standalone snapshot"
fi

echo "[remote] checking for existing mission/motion processes"
if pgrep -af 'industrial_cell_7_rods_single_debug.py|industrial_cell_7_rods_optimized.py|industrial_cell_mission_controller.py|industrial_map_nav_guarded.py|move_(arm|waist|ee)|rack_industrial_docking.py' >/tmp/site_7rods_existing_processes.txt; then
  echo "[remote] BLOCKED: existing mission/motion-related process found"
  cat /tmp/site_7rods_existing_processes.txt
  exit 20
fi

echo "[remote] runner preflight"
python3 rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py \
  --profile "$REMOTE_PROFILE" \
  --preflight-only \
  --start-index "$START_INDEX" \
  --end-index "$END_INDEX" \
  --checkpoint-file "$CHECKPOINT_FILE.preflight" \
  --run-log "$RUNNER_PREFLIGHT_LOG" \
  --snapshot-log "$RUNNER_SNAPSHOT_LOG" \
  --summary-log "$SUMMARY_LOG.preflight"

if [[ "$PREFLIGHT_ONLY" == "1" ]]; then
  echo "[remote] preflight-only complete; no physical motion executed"
  exit 0
fi

if [[ -e "$CHECKPOINT_FILE" ]]; then
  echo "[remote] BLOCKED: checkpoint already exists, refusing stale checkpoint: $CHECKPOINT_FILE"
  exit 21
fi
if [[ -e "$RUN_LOG" ]]; then
  echo "[remote] BLOCKED: run log already exists, refusing to append: $RUN_LOG"
  exit 22
fi

echo "[remote] starting live seven-rods run"
VISION_ARGS=()
if [[ "$VISION_CAPTURE" == "1" ]]; then
  VISION_ARGS+=(--vision-capture)
  VISION_ARGS+=(--vision-capture-dir "$VISION_CAPTURE_DIR")
  VISION_ARGS+=(--vision-capture-cameras "$VISION_CAPTURE_CAMERAS")
  VISION_ARGS+=(--vision-capture-timeout-ms "$VISION_CAPTURE_TIMEOUT_MS")
  VISION_ARGS+=(--vision-capture-interval-s "$VISION_CAPTURE_INTERVAL_S")
  if [[ "$VISION_CAPTURE_DEPTH_VIS" == "0" ]]; then
    VISION_ARGS+=(--vision-capture-no-depth-vis)
  fi
fi

set +e
python3 -u rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py \
  --profile "$REMOTE_PROFILE" \
  --live \
  --confirm-physical \
  --start-index "$START_INDEX" \
  --end-index "$END_INDEX" \
  "${VISION_ARGS[@]}" \
  --checkpoint-file "$CHECKPOINT_FILE" \
  --run-log "$RUN_LOG" \
  --snapshot-log "$RUNNER_SNAPSHOT_LOG" \
  --summary-log "$SUMMARY_LOG"
run_rc=$?
set -e

if [[ -f "$RUN_LOG" ]]; then
  echo "[remote] analyzing run log -> $SUMMARY_LOG"
  python3 rack_hybrid_docking_package/analyze_industrial_cell_run.py "$RUN_LOG" | tee "$SUMMARY_LOG" || true
fi

echo "[remote] checkpoint: $CHECKPOINT_FILE"
echo "[remote] run_log: $RUN_LOG"
echo "[remote] snapshot_log: $SNAPSHOT_LOG"
echo "[remote] runner_snapshot_log: $RUNNER_SNAPSHOT_LOG"
echo "[remote] summary_log: $SUMMARY_LOG"
exit "$run_rc"
"""


def load_profile(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def quote_env(values: dict[str, str]) -> str:
    return " ".join(f"{key}={shlex.quote(value)}" for key, value in values.items())


def remote_profile_path(profile_path: Path, profile: dict[str, object], override: str) -> str:
    """Return the robot-side path to the same profile.

    For normal repo profiles this preserves the package-relative path, e.g.
    ``profiles/map20_box528/profile.json`` becomes
    ``rack_hybrid_docking_package/profiles/map20_box528/profile.json``.
    """

    if override:
        return override
    try:
        rel_path = profile_path.relative_to(PACKAGE_DIR)
        return str(Path("rack_hybrid_docking_package") / rel_path)
    except ValueError:
        site_name = str(profile.get("site_name") or "site")
        return str(Path("rack_hybrid_docking_package") / "profiles" / site_name / "profile.json")


def remote_profile_relative_path(remote_profile: str, rel_path: str) -> str:
    path = Path(rel_path)
    if path.is_absolute():
        return str(path)
    return str(Path(remote_profile).parent / path)


def remote_required_files(profile: dict[str, object], remote_profile: str) -> list[str]:
    files = [remote_profile]
    station_config = profile.get("station_config")
    if isinstance(station_config, str):
        files.append(remote_profile_relative_path(remote_profile, station_config))
    grab_poses = profile.get("grab_poses") or {}
    if isinstance(grab_poses, dict):
        files.extend(remote_profile_relative_path(remote_profile, path) for path in grab_poses.values() if isinstance(path, str))
    place_sequence = profile.get("place_sequence") or {}
    if isinstance(place_sequence, dict):
        files.extend(remote_profile_relative_path(remote_profile, path) for path in place_sequence.values() if isinstance(path, str))
    return sorted(set(files))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default=str(DEFAULT_PROFILE), help="Profile JSON or profile directory")
    parser.add_argument("--preflight-only", action="store_true", help="Run read-only checks only; no physical motion")
    parser.add_argument("--yes", action="store_true", help="Skip the local physical-motion confirmation prompt")
    parser.add_argument("--host", default=os.environ.get("ROBOT_HOST", ""), help="Override SSH target")
    parser.add_argument("--remote-dir", default=os.environ.get("REMOTE_DIR", ""), help="Override robot workspace")
    parser.add_argument("--remote-profile", default=os.environ.get("REMOTE_PROFILE", ""), help="Robot-side profile path")
    parser.add_argument("--start-index", type=int, default=int(os.environ.get("START_INDEX", "1")))
    parser.add_argument("--end-index", type=int, default=int(os.environ.get("END_INDEX", "7")))
    parser.add_argument("--checkpoint-file", default=os.environ.get("CHECKPOINT_FILE", ""))
    parser.add_argument("--run-log", default=os.environ.get("RUN_LOG", ""))
    parser.add_argument("--snapshot-log", default=os.environ.get("SNAPSHOT_LOG", ""))
    parser.add_argument("--runner-snapshot-log", default=os.environ.get("RUNNER_SNAPSHOT_LOG", ""))
    parser.add_argument("--summary-log", default=os.environ.get("SUMMARY_LOG", ""))
    parser.add_argument("--vision-capture", action="store_true", help="Capture read-only images/state around local pick/place steps")
    parser.add_argument("--vision-capture-dir", default=os.environ.get("VISION_CAPTURE_DIR", ""))
    parser.add_argument("--vision-capture-cameras", default=os.environ.get("VISION_CAPTURE_CAMERAS", ",".join(DEFAULT_CAMERAS)))
    parser.add_argument("--vision-capture-timeout-ms", type=float, default=float(os.environ.get("VISION_CAPTURE_TIMEOUT_MS", "1000.0")))
    parser.add_argument("--vision-capture-interval-s", type=float, default=float(os.environ.get("VISION_CAPTURE_INTERVAL_S", "1.0")))
    parser.add_argument("--vision-capture-no-depth-vis", action="store_true")
    return parser.parse_args()


def normalized_profile_path(value: str) -> Path:
    path = Path(value)
    if path.is_dir():
        return path / "profile.json"
    return path


def confirm_motion(args: argparse.Namespace, profile: dict[str, object], host: str, remote_dir: str) -> None:
    if args.preflight_only or args.yes:
        return
    print()
    print("This will execute physical robot motion.")
    print(f"site:       {profile.get('site_name')}")
    print(f"map_id:     {profile.get('map_id')}")
    print(f"robot:      {host}")
    print(f"remote_dir: {remote_dir}")
    print("Confirm the robot is at a safe start/home state, upper body is clear, and there is no interference.")
    answer = input("Type RUN_SITE_7RODS to continue: ").strip()
    if answer != "RUN_SITE_7RODS":
        raise SystemExit("aborted")


def main() -> int:
    args = parse_args()
    profile_path = normalized_profile_path(args.profile).resolve()
    validation = validate_profile(profile_path)
    if not validation.get("ok"):
        print(json.dumps(validation, indent=2, ensure_ascii=False))
        raise SystemExit("profile validation failed")

    profile = load_profile(profile_path)
    robot = profile.get("robot") or {}
    if not isinstance(robot, dict):
        raise SystemExit("profile.robot must be an object")
    host = args.host or str(robot.get("host") or "")
    remote_dir = args.remote_dir or str(robot.get("remote_dir") or "")
    if not host or not remote_dir:
        raise SystemExit("robot host and remote_dir are required")
    if not (1 <= args.start_index <= 7 and 1 <= args.end_index <= 7 and args.start_index <= args.end_index):
        raise SystemExit("rod range must satisfy 1 <= start <= end <= 7")
    if args.vision_capture and args.vision_capture_interval_s <= 0:
        raise SystemExit("--vision-capture-interval-s must be positive")

    stamp = os.environ.get("RUN_STAMP") or datetime.now().strftime("%Y%m%d_%H%M%S")
    site_name = str(profile.get("site_name") or "site")
    checkpoint_file = args.checkpoint_file or f"logs/{site_name}_7rods_{args.start_index}_{args.end_index}_{stamp}_checkpoint.json"
    run_log = args.run_log or f"logs/{site_name}_7rods_{args.start_index}_{args.end_index}_{stamp}.log"
    snapshot_log = args.snapshot_log or f"logs/{site_name}_7rods_{args.start_index}_{args.end_index}_{stamp}_preflight_snapshot.log"
    runner_snapshot_log = args.runner_snapshot_log or f"logs/{site_name}_7rods_{args.start_index}_{args.end_index}_{stamp}_runner_snapshot.log"
    runner_preflight_log = f"logs/{site_name}_7rods_{args.start_index}_{args.end_index}_{stamp}_runner_preflight.log"
    summary_log = args.summary_log or f"logs/{site_name}_7rods_{args.start_index}_{args.end_index}_{stamp}_analysis.txt"
    vision_capture_cameras = parse_camera_list(args.vision_capture_cameras) if args.vision_capture else ()
    vision_capture_dir = args.vision_capture_dir or f"logs/{site_name}_vision_dataset_{stamp}"
    remote_profile = remote_profile_path(profile_path, profile, args.remote_profile)

    print(f"profile:      {profile_path}")
    print(f"remote_prof:  {remote_profile}")
    print(f"site:         {site_name}")
    print(f"map_id:       {profile.get('map_id')}")
    print(f"robot:        {host}")
    print(f"remote_dir:   {remote_dir}")
    print(f"rod_range:    {args.start_index}-{args.end_index}")
    print(f"checkpoint:   {checkpoint_file}")
    print(f"run_log:      {run_log}")
    print(f"snapshot_log: {snapshot_log}")
    print(f"preflight:    {runner_preflight_log}")
    print(f"summary_log:  {summary_log}")
    if args.vision_capture:
        print(f"vision_dir:   {vision_capture_dir}")
        print(f"vision_cams:  {','.join(vision_capture_cameras)}")
        print(f"vision_rate:  every {args.vision_capture_interval_s:.3f}s")

    confirm_motion(args, profile, host, remote_dir)

    env = {
        "REMOTE_DIR": remote_dir,
        "REMOTE_PROFILE": remote_profile,
        "START_INDEX": str(args.start_index),
        "END_INDEX": str(args.end_index),
        "CHECKPOINT_FILE": checkpoint_file,
        "RUN_LOG": run_log,
        "SNAPSHOT_LOG": snapshot_log,
        "RUNNER_SNAPSHOT_LOG": runner_snapshot_log,
        "RUNNER_PREFLIGHT_LOG": runner_preflight_log,
        "SUMMARY_LOG": summary_log,
        "PREFLIGHT_ONLY": "1" if args.preflight_only else "0",
        "VISION_CAPTURE": "1" if args.vision_capture else "0",
        "VISION_CAPTURE_DIR": vision_capture_dir,
        "VISION_CAPTURE_CAMERAS": ",".join(vision_capture_cameras),
        "VISION_CAPTURE_TIMEOUT_MS": str(args.vision_capture_timeout_ms),
        "VISION_CAPTURE_INTERVAL_S": str(args.vision_capture_interval_s),
        "VISION_CAPTURE_DEPTH_VIS": "0" if args.vision_capture_no_depth_vis else "1",
        "EXPECTED_MAP_ID": str(profile.get("map_id")),
        "REQUIRED_REMOTE_FILES_JSON": json.dumps(remote_required_files(profile, remote_profile), ensure_ascii=False),
        "EXPECTED_TUNED_JSON": json.dumps(profile.get("tuned") or {}, ensure_ascii=False),
    }
    remote_command = f"{quote_env(env)} bash -s"
    proc = subprocess.run(["ssh", host, remote_command], input=REMOTE_SCRIPT, text=True)
    return proc.returncode


if __name__ == "__main__":
    raise SystemExit(main())
