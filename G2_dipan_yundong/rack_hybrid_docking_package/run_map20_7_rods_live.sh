#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Run the map20 G2 industrial-cell seven-rods workflow on the robot.

Default behavior:
  - SSH to agi@192.168.0.7
  - use /data/g2_industrial_cell_20260612/wxf/BOX_528_1 as the robot workspace
  - run read-only status and readiness checks
  - create a fresh timestamped checkpoint and run log
  - run rods 1 through 7 with the current optimized live runner
  - analyze the run log after completion or failure

Options:
  --yes                  skip the local physical-motion confirmation prompt
  --preflight-only       run checks only; do not execute robot motion
  --host USER@HOST       robot SSH target, default agi@192.168.0.7
  --remote-dir PATH      robot workspace path
  --start-index N        first rod index, default 1
  --end-index N          last rod index, default 7
  --checkpoint-file PATH remote checkpoint path, default logs/live_map20_full_...
  --run-log PATH         remote runner log path, default logs/live_map20_full_...
  --snapshot-log PATH    remote read-only snapshot log path
  --summary-log PATH     remote analyzer output path
  -h, --help             show this help

Examples:
  ./rack_hybrid_docking_package/run_map20_7_rods_live.sh
  ./rack_hybrid_docking_package/run_map20_7_rods_live.sh --preflight-only
  ./rack_hybrid_docking_package/run_map20_7_rods_live.sh --yes
EOF
}

ROBOT_HOST="${ROBOT_HOST:-agi@192.168.0.7}"
REMOTE_DIR="${REMOTE_DIR:-/data/g2_industrial_cell_20260612/wxf/BOX_528_1}"
START_INDEX="${START_INDEX:-1}"
END_INDEX="${END_INDEX:-7}"
ASSUME_YES=0
PREFLIGHT_ONLY=0
CHECKPOINT_FILE="${CHECKPOINT_FILE:-}"
RUN_LOG="${RUN_LOG:-}"
SNAPSHOT_LOG="${SNAPSHOT_LOG:-}"
SUMMARY_LOG="${SUMMARY_LOG:-}"
STAMP="${RUN_STAMP:-$(date +%Y%m%d_%H%M%S)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --yes)
      ASSUME_YES=1
      shift
      ;;
    --preflight-only)
      PREFLIGHT_ONLY=1
      shift
      ;;
    --host)
      ROBOT_HOST="${2:?--host requires USER@HOST}"
      shift 2
      ;;
    --remote-dir)
      REMOTE_DIR="${2:?--remote-dir requires PATH}"
      shift 2
      ;;
    --start-index)
      START_INDEX="${2:?--start-index requires N}"
      shift 2
      ;;
    --end-index)
      END_INDEX="${2:?--end-index requires N}"
      shift 2
      ;;
    --checkpoint-file)
      CHECKPOINT_FILE="${2:?--checkpoint-file requires PATH}"
      shift 2
      ;;
    --run-log)
      RUN_LOG="${2:?--run-log requires PATH}"
      shift 2
      ;;
    --snapshot-log)
      SNAPSHOT_LOG="${2:?--snapshot-log requires PATH}"
      shift 2
      ;;
    --summary-log)
      SUMMARY_LOG="${2:?--summary-log requires PATH}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$START_INDEX" in
  ''|*[!0-9]*)
    echo "--start-index must be an integer" >&2
    exit 2
    ;;
esac
case "$END_INDEX" in
  ''|*[!0-9]*)
    echo "--end-index must be an integer" >&2
    exit 2
    ;;
esac
if (( START_INDEX < 1 || START_INDEX > 7 || END_INDEX < 1 || END_INDEX > 7 || START_INDEX > END_INDEX )); then
  echo "rod range must satisfy 1 <= start <= end <= 7" >&2
  exit 2
fi

: "${CHECKPOINT_FILE:=logs/live_map20_full_${START_INDEX}_${END_INDEX}_${STAMP}_checkpoint.json}"
: "${RUN_LOG:=logs/live_map20_full_${START_INDEX}_${END_INDEX}_${STAMP}.log}"
: "${SNAPSHOT_LOG:=logs/live_map20_full_${START_INDEX}_${END_INDEX}_${STAMP}_preflight_snapshot.log}"
: "${SUMMARY_LOG:=logs/live_map20_full_${START_INDEX}_${END_INDEX}_${STAMP}_analysis.txt}"

echo "robot:        ${ROBOT_HOST}"
echo "remote_dir:   ${REMOTE_DIR}"
echo "rod_range:    ${START_INDEX}-${END_INDEX}"
echo "checkpoint:   ${CHECKPOINT_FILE}"
echo "run_log:      ${RUN_LOG}"
echo "snapshot_log: ${SNAPSHOT_LOG}"
echo "summary_log:  ${SUMMARY_LOG}"

if (( PREFLIGHT_ONLY == 0 && ASSUME_YES == 0 )); then
  echo
  echo "This will execute physical robot motion."
  echo "Confirm the robot is on map20, at a safe start/home state, upper body is clear, and there is no interference."
  read -r -p "Type RUN_MAP20_7RODS to continue: " confirmation
  if [[ "$confirmation" != "RUN_MAP20_7RODS" ]]; then
    echo "aborted"
    exit 130
  fi
fi

remote_cmd=$(printf \
  'REMOTE_DIR=%q START_INDEX=%q END_INDEX=%q CHECKPOINT_FILE=%q RUN_LOG=%q SNAPSHOT_LOG=%q SUMMARY_LOG=%q PREFLIGHT_ONLY=%q bash -s' \
  "$REMOTE_DIR" "$START_INDEX" "$END_INDEX" "$CHECKPOINT_FILE" "$RUN_LOG" "$SNAPSHOT_LOG" "$SUMMARY_LOG" "$PREFLIGHT_ONLY")

ssh "$ROBOT_HOST" "$remote_cmd" <<'REMOTE_SCRIPT'
set -euo pipefail

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
REMOTE_SCRIPT
