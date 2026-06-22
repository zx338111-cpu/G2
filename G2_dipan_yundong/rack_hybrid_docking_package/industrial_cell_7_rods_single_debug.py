#!/usr/bin/env python3
"""Map20 seven-rods live/debug runner.

This is the top-level operator script for the current class/import architecture.
It intentionally keeps the *mission policy* in one readable place:

- mission state machine and checkpoint semantics;
- tuned pick/place offsets and speed limits;
- per-rod pick/place step ordering;
- preflight, process checks, logs, and post-run analysis;
- dispatch from high-level steps to importable primitive classes.

The actual robot motion primitives live in ``rack_hybrid_docking_package/
g2_primitives``.  This runner imports those classes directly for normal use,
while the old command-line scripts remain as thin compatibility wrappers for
manual one-step debugging.

Safety contract:

- default mode is a no-motion dry run;
- physical motion requires both ``--live`` and ``--confirm-physical``;
- each completed phase is checkpointed only after the phase succeeds;
- if a local step is stopped manually, the checkpoint is deliberately not
  advanced, because the physical robot may be between named phases;
- stale checkpoints must be reconciled against the physical robot before
  resuming live motion.

File map for maintainers:

- This file is the mission coordinator. It owns phase transitions, checkpoint
  timing, tuned offsets, safety gates, and log layout.
- ``g2_primitives/*.py`` files own the small importable robot actions. The
  runner imports those classes only at the point where an action is about to
  execute, so dry-run and preflight paths do not initialize GDK unnecessarily.
- The old top-level ``move_*.py`` scripts are compatibility wrappers around the
  same primitive classes. They are kept for manual one-step tests, but the
  mission path below does not shell out to them for arm/waist/gripper/offset
  actions.
- ``industrial_station_config.json`` is the map20 station contract. It cannot
  contain comments because it is JSON, so this runner documents how the station
  and safety fields are used.
- Pose JSON files under ``calibration_records`` are treated as calibrated field
  data. They are validated before movement and should not be silently edited by
  the mission runner.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager, redirect_stderr, redirect_stdout
from dataclasses import asdict, dataclass, is_dataclass
import json
from pathlib import Path
import py_compile
import subprocess
import sys
import time


# Path layout is kept explicit because the same tree is copied to the robot.
# On the robot the project root is normally:
#   /data/g2_industrial_cell_20260612/wxf/BOX_528_1
PACKAGE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from site_profile import (
    SiteProfileError,
    calibration_dir_from_profile,
    load_site_profile,
    resolve_profile_path,
    station_config_path,
)

CONFIG = PACKAGE_DIR / "industrial_station_config.json"
LOG_DIR = PROJECT_ROOT / "logs"
ARM_GRAB_POSE_DIR = PACKAGE_DIR / "calibration_records"
ARM_DEFAULT_JSON = Path("/data/wxf/wxf/positions/arm_default.json")

# The place sequence uses one shared calibrated pose set for the rack-side
# placement posture.  Per-rod variation is currently on the grab side; the
# place side is driven by this final tuned pose plus the relative offsets below.
PLACE_WAIST_JSON = ARM_GRAB_POSE_DIR / "rod07_place_waist_adjusted_latest.json"
PLACE_ABOVE_JSON = ARM_GRAB_POSE_DIR / "rod07_place_above_arm_latest.json"
PLACE_TRANSITION_JSON = ARM_GRAB_POSE_DIR / "rod07_place_transition_arm_latest.json"
PLACE_TRANSITION2_JSON = ARM_GRAB_POSE_DIR / "rod07_place_transition2_arm_latest.json"

# These globals describe the active site after optional profile loading.
# The legacy default preserves the original map20 behavior for old commands;
# passing ``--profile`` replaces all site-specific paths and tuning below.
ACTIVE_SITE_NAME = "map20_legacy"
ACTIVE_MAP_ID = 20
ACTIVE_PROFILE_FILE: Path | None = None
PROFILE_GRAB_POSES: dict[int, Path] = {}

# Mission phases are deliberately coarse.  Each phase is large enough to have a
# clear physical meaning, but small enough that a failed run can be diagnosed
# from the checkpoint without parsing the entire log.
PHASES = (
    "NAV_TO_GRAB",
    "LOCAL_PICK",
    "NAV_TO_PLACE",
    "LOCAL_PLACE",
    "NAV_TO_RECOVERY",
    "NAV_TO_HOME",
    "ROD_DONE",
    "MISSION_DONE",
)

# ``advance_state`` is the only place that moves the mission forward.  Keeping
# the transition table data-driven makes it harder for a local action to update
# ``holding_rod`` or ``current_station`` inconsistently.
NEXT_PHASE = {
    "NAV_TO_GRAB": "LOCAL_PICK",
    "LOCAL_PICK": "NAV_TO_PLACE",
    "NAV_TO_PLACE": "LOCAL_PLACE",
    "LOCAL_PLACE": "NAV_TO_RECOVERY",
    "NAV_TO_RECOVERY": "NAV_TO_HOME",
    "NAV_TO_HOME": "ROD_DONE",
    "ROD_DONE": "NAV_TO_GRAB",
}

# Navigation phases resolve to named stations from industrial_station_config.
# Local phases run arm/gripper/rack primitives while the chassis is already at
# the correct station.
NAV_PHASE_STATIONS = {
    "NAV_TO_GRAB": "GRAB_PRE",
    "NAV_TO_PLACE": "PLACE_PRE",
    "NAV_TO_RECOVERY": "RECOVERY_SAFE",
    "NAV_TO_HOME": "HOME_SAFE",
}
LOCAL_PHASES = {"LOCAL_PICK", "LOCAL_PLACE"}
LOCAL_STEP_LABEL_ALIASES = {
    "waist_grab_after_place": "waist_home_after_place",
}

# All live tuning that changes robot motion is centralized here so field
# adjustments are easy to review.  Values are in SI units unless the key says
# otherwise.  Keep behavior changes here small and explicit; the safety checks
# below assert the invariants that matter most after field tuning.
TUNED = {
    "arm_joint_speed_radps": 0.12,
    "fast_safe_arm_joint_speed_radps": 0.20,
    "waist_joint_speed_radps": 0.75,
    "arm_settle_s": 0.80,
    "waist_settle_s": 0.40,
    "offset_settle_s": 0.30,
    "offset_max_abs_m": 0.25,
    "grab_final_stop_mm": 328,
    "grab_final_brake_margin_mm": 20,
    "grab_final_speed_mps": 0.08,
    "place_final_stop_mm": 308,
    "place_final_brake_margin_mm": 20,
    "place_final_speed_mps": 0.15,
    # Pick: no downward dip.  The first arm retreat is 8.5 cm, then the second
    # retreat finishes the total 20 cm pull before the chassis retreats.
    "pick_down_z_m": 0.0,
    "pick_back_x_m": -0.20,
    "pick_back_down_x_m": -0.085,
    # Place: this is the final release point requested in live tuning:
    # move both arms forward 3 cm and down 2.5 cm before opening the grippers.
    "place_final_before_open_x_m": 0.03,
    "place_final_before_open_z_m": -0.025,
    "place_raise_before_open_z_m": 0.0,
    # After release: pull out while dropping a total of 8 cm.  This is not the
    # release height; the drop happens only after the grippers have opened.
    "place_pull_x_m": -0.25,
    "place_pull_back_down_x_m": -0.02,
    "place_pull_back_down_z_m": -0.01,
    "place_pull_drop_after_x_m": -0.06,
    "place_pull_drop_z_m": -0.07,
    "local_retreat_m": 0.45,
    "local_retreat_speed_mps": 0.20,
    "fine_position_max_duration_s": 60.0,
    "refine_yaw_tolerance_deg": 1.5,
    "refine_yaw_max_error_deg": 10.0,
    "refine_yaw_angular_speed_radps": 0.05,
    "refine_yaw_fine_angular_speed_radps": 0.02,
    "refine_yaw_timeout_s": 12.0,
    "refine_yaw_stations": {"GRAB_PRE", "PLACE_PRE", "RECOVERY_SAFE", "HOME_SAFE"},
}

FAST_SAFE_ARM_STEP_LABELS = {"arm_place_above", "arm_default_after_place"}
FINE_POSITION_SUCCESS_STATUSES = {"stopped", "already_at_threshold"}
RETREAT_SUCCESS_STATUSES = {"completed"}
PLACE_RETREAT_SUCCESS_STATUSES = {"completed", "rear_obstacle"}
RETRYABLE_LOCAL_COMMAND_SCRIPTS = {
    "move_arm_by_json_path.py",
    "move_waist_by_json_path.py",
    "move_ee_pose_open_2.py",
    "move_ee_pose_close_2.py",
}


@dataclass
class MissionState:
    """Checkpointed high-level state of the seven-rods mission.

    This object records only stable, operator-meaningful facts.  It does not
    try to remember sub-step progress inside a local pick/place phase, because
    partial arm/gripper motion must be reconciled against the real robot before
    continuing live.
    """

    rod_index: int
    end_index: int
    phase: str
    holding_rod: bool
    current_station: str | None
    last_success_step: str | None
    updated_at: float


def parse_args() -> argparse.Namespace:
    """Parse CLI flags for dry-run, live execution, and checkpoint handling."""

    stamp = time.strftime("%Y%m%d_%H%M%S")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="", help="Site profile JSON or profile directory")
    parser.add_argument("--live", action="store_true", help="Execute physical robot motion")
    parser.add_argument("--confirm-physical", action="store_true", help="Required with --live")
    parser.add_argument("--preflight-only", action="store_true", help="Run checks only; no motion")
    parser.add_argument("--status-only", action="store_true", help="Print checkpoint state and exit")
    parser.add_argument("--resume", action="store_true", help="Resume from an existing checkpoint")
    parser.add_argument("--allow-holding-resume", action="store_true")
    parser.add_argument("--start-index", type=int, default=1)
    parser.add_argument("--end-index", type=int, default=7)
    parser.add_argument("--stop-after-rod", type=int, default=0)
    parser.add_argument("--checkpoint-file", default=str(LOG_DIR / f"single_debug_map20_{stamp}_checkpoint.json"))
    parser.add_argument("--run-log", default=str(LOG_DIR / f"single_debug_map20_{stamp}.log"))
    parser.add_argument("--log-dir", default=str(LOG_DIR))
    parser.add_argument("--snapshot-log", default=str(LOG_DIR / f"single_debug_map20_{stamp}_snapshot.log"))
    parser.add_argument("--summary-log", default=str(LOG_DIR / f"single_debug_map20_{stamp}_analysis.txt"))
    parser.add_argument("--skip-readiness-check", action="store_true")
    parser.add_argument("--skip-process-check", action="store_true")
    parser.add_argument("--skip-file-check", action="store_true")
    parser.add_argument("--skip-status-snapshot", action="store_true", help="Skip robot status snapshot for offline smoke tests")
    parser.add_argument("--start-at-local-step", default="")
    parser.add_argument("--stop-after-local-step", default="")
    parser.add_argument("--vision-capture", action="store_true", help="Read-only image capture before/after local pick/place steps")
    parser.add_argument("--vision-capture-dir", default=str(LOG_DIR / "vision_dataset"))
    parser.add_argument(
        "--vision-capture-cameras",
        default="head_stereo_left,head_color,head_depth,hand_left_color,hand_right_color",
        help="Comma-separated camera names for --vision-capture",
    )
    parser.add_argument("--vision-capture-timeout-ms", type=float, default=1000.0)
    parser.add_argument("--vision-capture-interval-s", type=float, default=1.0)
    parser.add_argument("--vision-capture-no-depth-vis", action="store_true")
    return parser.parse_args()


def apply_site_profile(profile_arg: str | Path | None) -> None:
    """Load a site profile and replace all map/site-specific runtime inputs.

    The mission code below intentionally keeps using the simple module-level
    names ``CONFIG``, ``PLACE_*_JSON`` and ``TUNED``.  This function is the one
    boundary where those names are rebound from a profile.  Keeping that switch
    centralized lets field operators replace a map by editing/capturing a
    profile directory instead of searching through mission code.
    """

    if not profile_arg:
        return

    global ACTIVE_MAP_ID
    global ACTIVE_PROFILE_FILE
    global ACTIVE_SITE_NAME
    global ARM_GRAB_POSE_DIR
    global CONFIG
    global PLACE_ABOVE_JSON
    global PLACE_TRANSITION2_JSON
    global PLACE_TRANSITION_JSON
    global PLACE_WAIST_JSON
    global PROFILE_GRAB_POSES

    try:
        profile_file, profile_dir, profile = load_site_profile(profile_arg)
        config_path = station_config_path(profile_dir, profile)
    except SiteProfileError as exc:
        raise RuntimeError(str(exc)) from exc

    site_name = profile.get("site_name")
    if not isinstance(site_name, str) or not site_name:
        raise RuntimeError(f"profile.site_name must be a non-empty string: {profile_file}")
    map_id = profile.get("map_id")
    if not isinstance(map_id, int):
        raise RuntimeError(f"profile.map_id must be an integer: {profile_file}")

    grab_poses = profile.get("grab_poses")
    if not isinstance(grab_poses, dict):
        raise RuntimeError(f"profile.grab_poses must be an object: {profile_file}")
    resolved_grab_poses: dict[int, Path] = {}
    for key, rel_path in grab_poses.items():
        if not isinstance(rel_path, str) or not rel_path:
            raise RuntimeError(f"profile.grab_poses.{key} must be a path: {profile_file}")
        try:
            rod_index = int(key)
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"profile.grab_poses key must be a rod index, got {key!r}: {profile_file}") from exc
        resolved_grab_poses[rod_index] = resolve_profile_path(profile_dir, rel_path)

    place_sequence = profile.get("place_sequence")
    if not isinstance(place_sequence, dict):
        raise RuntimeError(f"profile.place_sequence must be an object: {profile_file}")
    missing_place_keys = [key for key in ("waist", "above_arm", "transition_arm", "transition2_arm") if not isinstance(place_sequence.get(key), str)]
    if missing_place_keys:
        raise RuntimeError(f"profile.place_sequence missing required paths {missing_place_keys}: {profile_file}")
    resolved_place = {
        key: resolve_profile_path(profile_dir, place_sequence[key])
        for key in ("waist", "above_arm", "transition_arm", "transition2_arm")
    }

    tuned = profile.get("tuned")
    if not isinstance(tuned, dict):
        raise RuntimeError(f"profile.tuned must be an object: {profile_file}")
    missing_tuned = sorted(set(TUNED) - set(tuned))
    if missing_tuned:
        raise RuntimeError(f"profile.tuned missing keys {missing_tuned}: {profile_file}")
    merged_tuned = dict(TUNED)
    for key, value in tuned.items():
        if key == "refine_yaw_stations":
            if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
                raise RuntimeError(f"profile.tuned.refine_yaw_stations must be a list of station names: {profile_file}")
            merged_tuned[key] = set(value)
        else:
            merged_tuned[key] = value

    ACTIVE_PROFILE_FILE = profile_file
    ACTIVE_SITE_NAME = site_name
    ACTIVE_MAP_ID = map_id
    CONFIG = config_path
    ARM_GRAB_POSE_DIR = calibration_dir_from_profile(profile_dir, profile)
    PROFILE_GRAB_POSES = resolved_grab_poses
    PLACE_WAIST_JSON = resolved_place["waist"]
    PLACE_ABOVE_JSON = resolved_place["above_arm"]
    PLACE_TRANSITION_JSON = resolved_place["transition_arm"]
    PLACE_TRANSITION2_JSON = resolved_place["transition2_arm"]
    TUNED.clear()
    TUNED.update(merged_tuned)


def event(name: str, **fields: object) -> None:
    """Emit one structured JSON event to stdout and the global run log."""

    print(json.dumps({"event": name, **jsonable(fields)}, ensure_ascii=False), flush=True)


def jsonable(value):
    """Best-effort conversion of GDK/result objects into JSON-loggable data."""

    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [jsonable(item) for item in value]
    fields = {}
    for name in dir(value):
        if name.startswith("_"):
            continue
        try:
            item = getattr(value, name)
        except Exception:
            continue
        if callable(item):
            continue
        if isinstance(item, (str, int, float, bool, type(None), list, tuple, dict)):
            fields[name] = jsonable(item)
    return fields if fields else repr(value)


def stream_command(command: list[str], log_file: Path) -> int:
    """Run an auxiliary command while teeing its output into a step log.

    The main runner now imports motion primitives directly.  This helper is
    still useful for read-only status snapshots and the offline analyzer.
    """

    log_file.parent.mkdir(parents=True, exist_ok=True)
    event("command_start", command=command, log_file=log_file)
    with log_file.open("a", encoding="utf-8") as log:
        log.write(json.dumps({"event": "command_start", "command": command}, ensure_ascii=False) + "\n")
        proc = subprocess.Popen(
            command,
            cwd=str(PROJECT_ROOT),
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
    event("command_done", command=command, return_code=return_code)
    return return_code


class TeeStream:
    """File-like object that mirrors stdout/stderr to multiple streams."""

    def __init__(self, *streams):
        self.streams = streams

    def write(self, data: str) -> int:
        for stream in self.streams:
            stream.write(data)
            stream.flush()
        return len(data)

    def flush(self) -> None:
        for stream in self.streams:
            stream.flush()


def install_global_log(run_log: Path):
    """Mirror all runner output to the run log without changing print callers."""

    run_log.parent.mkdir(parents=True, exist_ok=True)
    log = run_log.open("a", encoding="utf-8")
    original_stdout = sys.stdout
    original_stderr = sys.stderr
    sys.stdout = TeeStream(original_stdout, log)
    sys.stderr = TeeStream(original_stderr, log)
    return log, original_stdout, original_stderr


def run_with_step_log(log_file: Path, func):
    """Run one step and copy its stdout/stderr into a per-step log file."""

    log_file.parent.mkdir(parents=True, exist_ok=True)
    with log_file.open("a", encoding="utf-8") as log:
        stdout = TeeStream(sys.stdout, log)
        stderr = TeeStream(sys.stderr, log)
        with redirect_stdout(stdout), redirect_stderr(stderr):
            return func()


def initial_state(start_index: int, end_index: int) -> MissionState:
    """Create a fresh mission checkpoint starting from HOME_SAFE."""

    return MissionState(
        rod_index=start_index,
        end_index=end_index,
        phase="NAV_TO_GRAB",
        holding_rod=False,
        current_station="HOME_SAFE",
        last_success_step=None,
        updated_at=time.time(),
    )


def load_state(path: Path) -> MissionState:
    """Load a checkpoint exactly as written by ``save_state``."""

    return MissionState(**json.loads(path.read_text(encoding="utf-8")))


def save_state(path: Path, state: MissionState) -> None:
    """Persist the checkpoint after a phase has safely completed."""

    path.parent.mkdir(parents=True, exist_ok=True)
    state.updated_at = time.time()
    path.write_text(json.dumps(asdict(state), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def describe_action(state: MissionState) -> dict[str, object]:
    """Return a human/log readable description of the next physical action."""

    if state.phase in NAV_PHASE_STATIONS:
        station = NAV_PHASE_STATIONS[state.phase]
        action = f"guarded map navigation to {station}"
    elif state.phase == "LOCAL_PICK":
        station = "GRAB_PRE"
        action = "local pick sequence"
    elif state.phase == "LOCAL_PLACE":
        station = "PLACE_PRE"
        action = "local place sequence"
    elif state.phase == "ROD_DONE":
        station = state.current_station
        action = "advance to next rod"
    else:
        station = state.current_station
        action = "mission done or unknown"
    return {
        "rod_index": state.rod_index,
        "phase": state.phase,
        "holding_rod": state.holding_rod,
        "station": station,
        "action": action,
    }


def advance_state(state: MissionState) -> MissionState:
    """Advance the high-level state after the current phase succeeds.

    This function is intentionally the single writer for ``holding_rod`` and
    ``current_station``.  If a live run fails or is manually stopped before this
    function runs, the checkpoint remains on the previous phase so an operator
    must inspect the physical robot instead of blindly resuming a stale state.
    """

    if state.phase not in NEXT_PHASE:
        raise ValueError(f"unknown phase: {state.phase}")
    previous = state.phase
    state.last_success_step = previous
    state.phase = NEXT_PHASE[previous]
    if previous == "NAV_TO_GRAB":
        state.current_station = "GRAB_PRE"
    elif previous == "LOCAL_PICK":
        state.holding_rod = True
        state.current_station = "GRAB_PRE"
    elif previous == "NAV_TO_PLACE":
        state.current_station = "PLACE_PRE"
    elif previous == "LOCAL_PLACE":
        state.holding_rod = False
        state.current_station = "PLACE_PRE"
    elif previous == "NAV_TO_RECOVERY":
        state.current_station = "RECOVERY_SAFE"
    elif previous == "NAV_TO_HOME":
        state.current_station = "HOME_SAFE"
    elif previous == "ROD_DONE":
        if state.rod_index >= state.end_index:
            state.phase = "MISSION_DONE"
        else:
            state.rod_index += 1
            state.current_station = "HOME_SAFE"
            state.last_success_step = f"rod_{state.rod_index - 1}_completed"
    return state


def latest_rod_grab_pose_json(rod_index: int) -> Path:
    """Select the latest captured grab pose for one rod index."""

    if rod_index in PROFILE_GRAB_POSES:
        path = PROFILE_GRAB_POSES[rod_index]
        if not path.exists():
            raise RuntimeError(f"profile grab pose JSON missing for rod {rod_index}: {path}")
        return path

    pattern = f"rod{rod_index:02d}_grab_pose_*.json"
    matches = sorted(ARM_GRAB_POSE_DIR.glob(pattern))
    if not matches:
        raise RuntimeError(f"no captured grab pose JSON found for rod {rod_index}: {ARM_GRAB_POSE_DIR / pattern}")
    return matches[-1]


def arm_speed_for_label(label: str) -> float:
    """Use faster arm speed only on labels that have already been validated."""

    if label in FAST_SAFE_ARM_STEP_LABELS:
        return float(TUNED["fast_safe_arm_joint_speed_radps"])
    return float(TUNED["arm_joint_speed_radps"])


def command_step(label: str, script: str, **fields: object) -> dict[str, object]:
    """Describe a local primitive command in the local phase plan."""

    return {"kind": "command", "label": label, "script": script, **fields}


def fine_step(label: str, final_stop_mm: int, final_brake_margin_mm: int, final_speed_mps: float) -> dict[str, object]:
    """Describe an ultrasonic/chassis fine-positioning step."""

    return {
        "kind": "fine_position",
        "label": label,
        "final_stop_mm": final_stop_mm,
        "final_brake_margin_mm": final_brake_margin_mm,
        "final_speed_mps": final_speed_mps,
    }


def retreat_step(label: str, success_statuses: set[str]) -> dict[str, object]:
    """Describe a guarded chassis retreat step."""

    return {
        "kind": "retreat",
        "label": label,
        "distance_m": TUNED["local_retreat_m"],
        "speed_mps": TUNED["local_retreat_speed_mps"],
        "success_statuses": success_statuses,
    }


def place_pull_out_steps() -> list[dict[str, object]]:
    """Build the post-release arm pull-out sequence.

    The live-tuned requirement is easy to misunderstand: the part is released
    first, then the arms retreat while dropping 8 cm total.  This helper splits
    that into small explicit offsets so the log shows exactly where the drop
    happened:

    1. small back/down move immediately after opening;
    2. back to the configured drop point;
    3. vertical drop;
    4. remaining horizontal pull-out.
    """

    steps: list[dict[str, object]] = []
    x_done = 0.0
    back_down_x = float(TUNED["place_pull_back_down_x_m"])
    back_down_z = float(TUNED["place_pull_back_down_z_m"])
    drop_after_x = float(TUNED["place_pull_drop_after_x_m"])
    drop_z = float(TUNED["place_pull_drop_z_m"])
    total_pull_x = float(TUNED["place_pull_x_m"])

    if abs(back_down_x) > 1e-6 or abs(back_down_z) > 1e-6:
        steps.append(
            command_step(
                "place_pull_back_down_offset",
                "move_ee_relative_offset.py",
                left=(back_down_x, 0.0, back_down_z),
                right=(back_down_x, 0.0, back_down_z),
            )
        )
        x_done += back_down_x

    before_drop_x = drop_after_x - x_done
    if abs(before_drop_x) > 1e-6:
        steps.append(
            command_step(
                "place_pull_back_before_drop",
                "move_ee_relative_offset.py",
                left=(before_drop_x, 0.0, 0.0),
                right=(before_drop_x, 0.0, 0.0),
            )
        )
        x_done += before_drop_x
    if abs(drop_z) > 1e-6:
        steps.append(
            command_step(
                "place_pull_drop_offset",
                "move_ee_relative_offset.py",
                left=(0.0, 0.0, drop_z),
                right=(0.0, 0.0, drop_z),
            )
        )

    remaining_x = total_pull_x - x_done
    if abs(remaining_x) > 1e-6:
        steps.append(
            command_step(
                "place_pull_back_remaining_offset",
                "move_ee_relative_offset.py",
                left=(remaining_x, 0.0, 0.0),
                right=(remaining_x, 0.0, 0.0),
            )
        )
    return steps


def local_plan(phase: str, rod_index: int) -> list[dict[str, object]]:
    """Generate the ordered local arm/gripper/rack steps for one phase.

    ``LOCAL_PICK`` uses each rod's captured grab pose.  ``LOCAL_PLACE`` uses the
    shared tuned place pose plus relative offsets, because the field validation
    showed the place-side bottleneck was repeatability at the rack, not per-rod
    variation.
    """

    if phase == "LOCAL_PICK":
        grab_pose_json = latest_rod_grab_pose_json(rod_index)
        remaining_pick_back = float(TUNED["pick_back_x_m"]) - float(TUNED["pick_back_down_x_m"])
        return [
            command_step("open_gripper", "move_ee_pose_open_2.py"),
            command_step(
                "waist_for_grab",
                "move_waist_by_json_path.py",
                json=grab_pose_json,
                joint_speed_radps=TUNED["waist_joint_speed_radps"],
            ),
            command_step(
                "arm_grab_pose",
                "move_arm_by_json_path.py",
                json=grab_pose_json,
                joint_speed_radps=TUNED["arm_joint_speed_radps"],
            ),
            fine_step(
                "grab_fine_position",
                int(TUNED["grab_final_stop_mm"]),
                int(TUNED["grab_final_brake_margin_mm"]),
                float(TUNED["grab_final_speed_mps"]),
            ),
            command_step("close_gripper", "move_ee_pose_close_2.py"),
            command_step(
                "pick_back_down_offset",
                "move_ee_relative_offset.py",
                left=(TUNED["pick_back_down_x_m"], 0.0, TUNED["pick_down_z_m"]),
                right=(TUNED["pick_back_down_x_m"], 0.0, TUNED["pick_down_z_m"]),
            ),
            command_step(
                "pick_back_remaining_offset",
                "move_ee_relative_offset.py",
                left=(remaining_pick_back, 0.0, 0.0),
                right=(remaining_pick_back, 0.0, 0.0),
            ),
            retreat_step("retreat_after_pick", RETREAT_SUCCESS_STATUSES),
            command_step(
                "waist_home_after_pick",
                "move_waist_by_json_path.py",
                json=ARM_DEFAULT_JSON,
                joint_speed_radps=TUNED["waist_joint_speed_radps"],
            ),
        ]

    if phase == "LOCAL_PLACE":
        steps = [
            command_step(
                "waist_place_straight",
                "move_waist_by_json_path.py",
                json=PLACE_WAIST_JSON,
                joint_speed_radps=TUNED["waist_joint_speed_radps"],
            ),
            command_step(
                "arm_place_above",
                "move_arm_by_json_path.py",
                json=PLACE_ABOVE_JSON,
                joint_speed_radps=arm_speed_for_label("arm_place_above"),
            ),
            fine_step(
                "place_fine_position",
                int(TUNED["place_final_stop_mm"]),
                int(TUNED["place_final_brake_margin_mm"]),
                float(TUNED["place_final_speed_mps"]),
            ),
            command_step(
                "arm_place_transition",
                "move_arm_by_json_path.py",
                json=PLACE_TRANSITION_JSON,
                joint_speed_radps=TUNED["arm_joint_speed_radps"],
            ),
            command_step(
                "arm_place_transition2",
                "move_arm_by_json_path.py",
                json=PLACE_TRANSITION2_JSON,
                joint_speed_radps=TUNED["arm_joint_speed_radps"],
            ),
            command_step(
                "place_final_before_open_offset",
                "move_ee_relative_offset.py",
                left=(TUNED["place_final_before_open_x_m"], 0.0, TUNED["place_final_before_open_z_m"]),
                right=(TUNED["place_final_before_open_x_m"], 0.0, TUNED["place_final_before_open_z_m"]),
            ),
            command_step("open_gripper_place", "move_ee_pose_open_2.py"),
        ]
        steps.extend(place_pull_out_steps())
        steps.extend(
            [
                retreat_step("retreat_after_place", PLACE_RETREAT_SUCCESS_STATUSES),
                command_step(
                    "arm_default_after_place",
                    "move_arm_by_json_path.py",
                    json=ARM_DEFAULT_JSON,
                    joint_speed_radps=arm_speed_for_label("arm_default_after_place"),
                ),
                command_step(
                    "waist_home_after_place",
                    "move_waist_by_json_path.py",
                    json=ARM_DEFAULT_JSON,
                    joint_speed_radps=TUNED["waist_joint_speed_radps"],
                ),
            ]
        )
        return steps

    raise ValueError(f"unsupported local phase: {phase}")


def local_action_log_path(log_dir: Path, phase: str, rod_index: int, label: str, attempt: int | None = None) -> Path:
    """Create a deterministic per-step log path for local primitive actions."""

    suffix = f"_attempt{attempt}" if attempt else ""
    return log_dir / f"single_debug_rod{rod_index:02d}_{phase}_{label}{suffix}.log"


def run_imported_command_step(
    *,
    step: dict[str, object],
    phase: str,
    rod_index: int,
    log_dir: Path,
) -> dict[str, object]:
    """Execute one local command by importing its primitive class.

    The ``script`` field is kept in the plan for compatibility with old logs
    and operator vocabulary, but this function no longer shells out to those
    scripts.  It maps the script name to the matching class method:

    - gripper scripts -> ``GripperController``;
    - arm JSON script -> ``ArmJointController``;
    - waist JSON script -> ``WaistController``;
    - relative offset script -> ``EndEffectorOffsetController``.

    Retrying is allowed only for primitives that are known to be idempotent
    enough for this workflow.  Fine positioning and retreats have their own
    result checks and are not retried here.
    """

    script = str(step["script"])
    label = str(step["label"])
    max_attempts = 2 if script in RETRYABLE_LOCAL_COMMAND_SCRIPTS else 1
    attempts = []
    start = time.time()
    ok = False

    for attempt in range(1, max_attempts + 1):
        log_file = local_action_log_path(log_dir, phase, rod_index, label, attempt if max_attempts > 1 else None)
        attempt_start = time.time()

        def execute_step() -> bool:
            # Each branch creates a fresh GDK-scoped primitive.  This keeps
            # one failed local action from leaking a stale GDK session into the
            # next action and makes individual step logs self-contained.
            if script == "move_ee_pose_open_2.py":
                from g2_primitives.gripper import GripperController

                return GripperController().open_both(settle_s=0.05)
            if script == "move_ee_pose_close_2.py":
                from g2_primitives.gripper import GripperController

                return GripperController().close_both(settle_s=0.05)
            if script == "move_arm_by_json_path.py":
                from g2_primitives.arm import ArmJointController

                return ArmJointController().move_to_json(
                    Path(step["json"]),
                    joint_speed_radps=float(step["joint_speed_radps"]),
                    settle_s=float(TUNED["arm_settle_s"]),
                )
            if script == "move_waist_by_json_path.py":
                from g2_primitives.waist import WaistController

                return WaistController().move_to_json(
                    Path(step["json"]),
                    joint_speed_radps=float(step["joint_speed_radps"]),
                    max_step_rad=0.75,
                    settle_tol_rad=0.05,
                    settle_timeout_s=2.0,
                    poll_s=0.08,
                    settle_s=float(TUNED["waist_settle_s"]),
                )
            if script == "move_ee_relative_offset.py":
                from g2_primitives.ee_offset import EndEffectorOffsetController

                return EndEffectorOffsetController().move_relative(
                    left=tuple(float(value) for value in step["left"]),
                    right=tuple(float(value) for value in step["right"]),
                    max_abs_m=float(TUNED["offset_max_abs_m"]),
                    settle_s=float(TUNED["offset_settle_s"]),
                )
            raise ValueError(f"unsupported imported command step: {script}")

        try:
            ok = bool(run_with_step_log(log_file, execute_step))
            return_code = 0 if ok else 1
        except Exception as exc:
            ok = False
            return_code = 1
            with log_file.open("a", encoding="utf-8") as log:
                print(f"imported_step_failed label={label} error={type(exc).__name__}: {exc}", file=log, flush=True)
            print(f"imported_step_failed label={label} error={type(exc).__name__}: {exc}", flush=True)

        attempts.append(
            {
                "attempt": attempt,
                "max_attempts": max_attempts,
                "return_code": return_code,
                "log_file": str(log_file),
                "elapsed_s": round(time.time() - attempt_start, 3),
                "mode": "import",
            }
        )
        event("local_child_step_attempt_done", label=label, phase=phase, rod_index=rod_index, **attempts[-1])
        if ok:
            break
        if attempt < max_attempts:
            event("local_child_step_retry", label=label, phase=phase, rod_index=rod_index, next_attempt=attempt + 1)
            time.sleep(1.0)

    result = {
        "label": label,
        "phase": phase,
        "rod_index": rod_index,
        "script": script,
        "return_code": 0 if ok else 1,
        "attempts": attempts,
        "elapsed_s": round(time.time() - start, 3),
        "mode": "import",
    }
    event("local_child_step_done", **result)
    if not ok:
        raise RuntimeError(f"local imported step failed: {label}")
    return result


def run_fine_position_step(step: dict[str, object], phase: str, rod_index: int) -> dict[str, object]:
    """Run the rack-side ultrasonic fine-positioning primitive."""

    from g2_primitives.chassis_motion import ChassisMotionController

    result = ChassisMotionController(CONFIG).fine_position(
        final_stop_mm=int(step["final_stop_mm"]),
        final_brake_margin_mm=int(step["final_brake_margin_mm"]),
        final_speed_mps=float(step["final_speed_mps"]),
        max_duration_s=float(TUNED["fine_position_max_duration_s"]),
        allow_estop_pedal_fault=True,
    )
    payload = {
        "label": step["label"],
        "phase": phase,
        "rod_index": rod_index,
        "final_stop_mm": step["final_stop_mm"],
        "final_brake_margin_mm": step["final_brake_margin_mm"],
        "final_speed_mps": step["final_speed_mps"],
        "result": result,
    }
    event("local_fine_position_done", **payload)
    if getattr(result, "status", None) not in FINE_POSITION_SUCCESS_STATUSES:
        raise RuntimeError(f"{step['label']} failed: {result}")
    return payload


def run_retreat_step(step: dict[str, object], phase: str, rod_index: int) -> dict[str, object]:
    """Run the guarded chassis retreat primitive after pick/place local work."""

    from g2_primitives.chassis_motion import ChassisMotionController

    result = ChassisMotionController(CONFIG).retreat(
        distance_m=float(step["distance_m"]),
        speed_mps=float(step["speed_mps"]),
        allow_estop_pedal_fault=True,
    )
    payload = {
        "label": step["label"],
        "phase": phase,
        "rod_index": rod_index,
        "distance_m": step["distance_m"],
        "speed_mps": step["speed_mps"],
        "result": result,
    }
    event("local_retreat_done", **payload)
    if getattr(result, "status", None) not in set(step["success_statuses"]):
        raise RuntimeError(f"{step['label']} retreat failed: {result}")
    return payload


def run_local_vision_capture(
    *,
    enabled: bool,
    phase: str,
    rod_index: int,
    step_index: int,
    step_label: str,
    step_kind: str,
    moment: str,
    output_root: Path,
    cameras: tuple[str, ...],
    timeout_ms: float,
    make_depth_visualization: bool,
) -> dict[str, object] | None:
    """Capture read-only camera/state data around local pick/place actions.

    This hook is deliberately best-effort.  Vision data is useful for later
    analysis, but it is not part of the current safety or motion-control loop.
    If a camera times out or a frame cannot be saved, the mission should keep
    using the already validated point-based process and only log the capture
    failure.
    """

    if not enabled:
        return None
    try:
        from process_vision_capture import capture_process_vision_snapshot

        manifest = capture_process_vision_snapshot(
            output_root=output_root,
            site=ACTIVE_SITE_NAME,
            profile=ACTIVE_PROFILE_FILE,
            phase=phase,
            rod_index=rod_index,
            step_index=step_index,
            step_label=step_label,
            step_kind=step_kind,
            moment=moment,
            cameras=cameras,
            timeout_ms=timeout_ms,
            make_depth_visualization=make_depth_visualization,
        )
        camera_ok = {
            name: bool(payload.get("ok")) if isinstance(payload, dict) else False
            for name, payload in dict(manifest.get("cameras") or {}).items()
        }
        event(
            "local_vision_capture_done",
            phase=phase,
            rod_index=rod_index,
            step_index=step_index,
            step_label=step_label,
            step_kind=step_kind,
            moment=moment,
            output_dir=manifest.get("output_dir"),
            manifest_file=manifest.get("manifest_file"),
            cameras=camera_ok,
        )
        return manifest
    except Exception as exc:
        event(
            "local_vision_capture_failed",
            phase=phase,
            rod_index=rod_index,
            step_index=step_index,
            step_label=step_label,
            step_kind=step_kind,
            moment=moment,
            error=f"{type(exc).__name__}: {exc}",
        )
        return None


def start_local_vision_sequence(
    *,
    enabled: bool,
    phase: str,
    rod_index: int,
    output_root: Path,
    cameras: tuple[str, ...],
    timeout_ms: float,
    interval_s: float,
    make_depth_visualization: bool,
) -> tuple[subprocess.Popen | None, Path | None]:
    """Start a read-only subprocess that samples cameras through a local phase."""

    if not enabled:
        return None, None
    control_dir = output_root / "_control"
    control_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    token = f"{ACTIVE_SITE_NAME}_{phase}_rod{rod_index:02d}_{stamp}_{time.monotonic_ns()}"
    stop_file = control_dir / f"{token}.stop"
    log_file = control_dir / f"{token}.log"
    command = [
        sys.executable,
        str(PACKAGE_DIR / "process_vision_capture.py"),
        "--sequence",
        "--output-root",
        str(output_root),
        "--site",
        ACTIVE_SITE_NAME,
        "--profile",
        "" if ACTIVE_PROFILE_FILE is None else str(ACTIVE_PROFILE_FILE),
        "--phase",
        phase,
        "--rod-index",
        str(rod_index),
        "--cameras",
        ",".join(cameras),
        "--timeout-ms",
        str(timeout_ms),
        "--interval-s",
        str(interval_s),
        "--stop-file",
        str(stop_file),
        "--note",
        "automatic local pick/place sequence capture",
    ]
    if not make_depth_visualization:
        command.append("--no-depth-visualization")

    log = log_file.open("a", encoding="utf-8")
    try:
        proc = subprocess.Popen(
            command,
            cwd=str(PROJECT_ROOT),
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
        )
    finally:
        log.close()
    event(
        "local_vision_sequence_started",
        phase=phase,
        rod_index=rod_index,
        pid=proc.pid,
        stop_file=stop_file,
        log_file=log_file,
        interval_s=interval_s,
        cameras=cameras,
    )
    return proc, stop_file


def stop_local_vision_sequence(
    *,
    proc: subprocess.Popen | None,
    stop_file: Path | None,
    phase: str,
    rod_index: int,
) -> None:
    """Stop the read-only sequence sampler without failing the mission."""

    if proc is None:
        return
    try:
        if stop_file is not None:
            stop_file.parent.mkdir(parents=True, exist_ok=True)
            stop_file.write_text("stop\n", encoding="utf-8")
        try:
            return_code = proc.wait(timeout=8.0)
            stop_method = "stop_file"
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                return_code = proc.wait(timeout=5.0)
                stop_method = "terminate"
            except subprocess.TimeoutExpired:
                proc.kill()
                return_code = proc.wait(timeout=5.0)
                stop_method = "kill"
        event(
            "local_vision_sequence_stopped",
            phase=phase,
            rod_index=rod_index,
            pid=proc.pid,
            return_code=return_code,
            stop_method=stop_method,
            stop_file=stop_file,
        )
    except Exception as exc:
        event(
            "local_vision_sequence_stop_failed",
            phase=phase,
            rod_index=rod_index,
            pid=getattr(proc, "pid", None),
            error=f"{type(exc).__name__}: {exc}",
        )


@contextmanager
def local_vision_sequence_context(
    *,
    enabled: bool,
    phase: str,
    rod_index: int,
    output_root: Path,
    cameras: tuple[str, ...],
    timeout_ms: float,
    interval_s: float,
    make_depth_visualization: bool,
):
    """Context manager for continuous read-only process capture."""

    proc, stop_file = start_local_vision_sequence(
        enabled=enabled,
        phase=phase,
        rod_index=rod_index,
        output_root=output_root,
        cameras=cameras,
        timeout_ms=timeout_ms,
        interval_s=interval_s,
        make_depth_visualization=make_depth_visualization,
    )
    try:
        yield
    finally:
        stop_local_vision_sequence(proc=proc, stop_file=stop_file, phase=phase, rod_index=rod_index)


def run_local_phase(
    *,
    phase: str,
    rod_index: int,
    live: bool,
    confirm_physical: bool,
    log_dir: Path,
    start_at_local_step: str,
    stop_after_local_step: str,
    vision_capture: bool,
    vision_capture_dir: Path,
    vision_capture_cameras: tuple[str, ...],
    vision_capture_timeout_ms: float,
    vision_capture_interval_s: float,
    vision_capture_make_depth_vis: bool,
) -> dict[str, object]:
    """Run or dry-run all local actions for ``LOCAL_PICK`` or ``LOCAL_PLACE``.

    ``start_at_local_step`` and ``stop_after_local_step`` are debugging aids.
    When the runner stops after a local step, it returns without advancing the
    mission checkpoint.  That behavior is intentional: the robot may be in a
    physical posture that cannot be represented by the coarse phase state.
    """

    plan = local_plan(phase, rod_index)
    start_at_local_step = LOCAL_STEP_LABEL_ALIASES.get(start_at_local_step, start_at_local_step)
    stop_after_local_step = LOCAL_STEP_LABEL_ALIASES.get(stop_after_local_step, stop_after_local_step)
    labels = [str(step["label"]) for step in plan]
    missing_labels = [label for label in (start_at_local_step, stop_after_local_step) if label and label not in labels]
    if missing_labels:
        raise ValueError(f"unknown local step label(s): {missing_labels}; available={labels}")
    start_index = labels.index(start_at_local_step) if start_at_local_step else 0
    execution_plan = plan[start_index:]
    skipped_steps = plan[:start_index]
    event(
        "local_single_debug_plan",
        live=live,
        phase=phase,
        rod_index=rod_index,
        start_at_local_step=start_at_local_step,
        stop_after_local_step=stop_after_local_step,
        skipped_steps=skipped_steps,
        steps=execution_plan,
    )
    if not live:
        # Dry-run returns the generated plan only; no GDK object is created and
        # no arm, gripper, rack, or chassis command is sent.
        return {
            "mode": "dry-run",
            "phase": phase,
            "rod_index": rod_index,
            "steps": execution_plan,
            "skipped_steps": skipped_steps,
            "note": "no arm, gripper, rack, or chassis command was sent",
        }
    if not confirm_physical:
        raise RuntimeError("--live requires --confirm-physical before local physical actions")

    results = []
    with local_vision_sequence_context(
        enabled=vision_capture,
        phase=phase,
        rod_index=rod_index,
        output_root=vision_capture_dir,
        cameras=vision_capture_cameras,
        timeout_ms=vision_capture_timeout_ms,
        interval_s=vision_capture_interval_s,
        make_depth_visualization=vision_capture_make_depth_vis,
    ):
        run_local_vision_capture(
            enabled=vision_capture,
            phase=phase,
            rod_index=rod_index,
            step_index=0,
            step_label="phase_start",
            step_kind="phase",
            moment="phase_start",
            output_root=vision_capture_dir,
            cameras=vision_capture_cameras,
            timeout_ms=vision_capture_timeout_ms,
            make_depth_visualization=vision_capture_make_depth_vis,
        )
        for step_index, step in enumerate(execution_plan, start=start_index + 1):
            kind = str(step["kind"])
            label = str(step["label"])
            run_local_vision_capture(
                enabled=vision_capture,
                phase=phase,
                rod_index=rod_index,
                step_index=step_index,
                step_label=label,
                step_kind=kind,
                moment="before_step",
                output_root=vision_capture_dir,
                cameras=vision_capture_cameras,
                timeout_ms=vision_capture_timeout_ms,
                make_depth_visualization=vision_capture_make_depth_vis,
            )
            try:
                if kind == "command":
                    results.append(run_imported_command_step(step=step, phase=phase, rod_index=rod_index, log_dir=log_dir))
                elif kind == "fine_position":
                    results.append(run_fine_position_step(step, phase, rod_index))
                elif kind == "retreat":
                    results.append(run_retreat_step(step, phase, rod_index))
                else:
                    raise RuntimeError(f"unknown local step kind: {kind}")
            except Exception:
                run_local_vision_capture(
                    enabled=vision_capture,
                    phase=phase,
                    rod_index=rod_index,
                    step_index=step_index,
                    step_label=label,
                    step_kind=kind,
                    moment="after_step_error",
                    output_root=vision_capture_dir,
                    cameras=vision_capture_cameras,
                    timeout_ms=vision_capture_timeout_ms,
                    make_depth_visualization=vision_capture_make_depth_vis,
                )
                raise
            run_local_vision_capture(
                enabled=vision_capture,
                phase=phase,
                rod_index=rod_index,
                step_index=step_index,
                step_label=label,
                step_kind=kind,
                moment="after_step",
                output_root=vision_capture_dir,
                cameras=vision_capture_cameras,
                timeout_ms=vision_capture_timeout_ms,
                make_depth_visualization=vision_capture_make_depth_vis,
            )
            if stop_after_local_step == label:
                payload = {
                    "mode": "live",
                    "phase": phase,
                    "rod_index": rod_index,
                    "stopped_after_step": label,
                    "checkpoint_advanced": False,
                    "results": results,
                }
                event("local_single_debug_stopped", **payload)
                return payload
        run_local_vision_capture(
            enabled=vision_capture,
            phase=phase,
            rod_index=rod_index,
            step_index=len(plan) + 1,
            step_label="phase_done",
            step_kind="phase",
            moment="phase_done",
            output_root=vision_capture_dir,
            cameras=vision_capture_cameras,
            timeout_ms=vision_capture_timeout_ms,
            make_depth_visualization=vision_capture_make_depth_vis,
        )
    return {"mode": "live", "phase": phase, "rod_index": rod_index, "results": results}


def nav_log_path(log_dir: Path, state: MissionState, station: str) -> Path:
    """Create a deterministic per-phase log path for station navigation."""

    return log_dir / f"single_debug_rod{state.rod_index:02d}_{state.phase}_to_{station.lower()}.log"


def run_nav_phase(
    *,
    station: str,
    live: bool,
    log_file: Path,
) -> dict[str, object]:
    """Navigate to a named station and optionally refine yaw.

    The heavy lifting is in ``ChassisMotionController``.  The runner only supplies the
    tuned yaw-refine limits and enforces that the returned payload says ``ok``.
    """

    from g2_primitives.chassis_motion import ChassisMotionController

    def execute_nav() -> dict[str, object]:
        return ChassisMotionController(CONFIG).goto_station(
            station,
            live=live,
            refine_yaw=live,
            refine_yaw_tolerance_deg=float(TUNED["refine_yaw_tolerance_deg"]),
            refine_yaw_max_error_deg=float(TUNED["refine_yaw_max_error_deg"]),
            refine_yaw_angular_speed_radps=float(TUNED["refine_yaw_angular_speed_radps"]),
            refine_yaw_fine_angular_speed_radps=float(TUNED["refine_yaw_fine_angular_speed_radps"]),
            refine_yaw_timeout_s=float(TUNED["refine_yaw_timeout_s"]),
        )

    event("nav_phase_start", station=station, live=live, log_file=log_file)
    result = run_with_step_log(log_file, execute_nav)
    event("nav_phase_done", station=station, live=live, result=result)
    if not result.get("ok"):
        raise RuntimeError(f"guarded navigation failed for {station}: {result}")
    return result


def execute_current_phase(args: argparse.Namespace, state: MissionState, checkpoint: Path) -> MissionState:
    """Execute exactly one mission phase and checkpoint on success."""

    event("mission_phase_start", action=describe_action(state), state=state)
    log_dir = Path(args.log_dir).resolve()
    if state.phase in NAV_PHASE_STATIONS:
        station = NAV_PHASE_STATIONS[state.phase]
        run_nav_phase(
            station=station,
            live=args.live,
            log_file=nav_log_path(log_dir, state, station),
        )
        state = advance_state(state)
        save_state(checkpoint, state)
        event("mission_phase_done", state=state)
        return state

    if state.phase in LOCAL_PHASES:
        result = run_local_phase(
            phase=state.phase,
            rod_index=state.rod_index,
            live=args.live,
            confirm_physical=args.confirm_physical,
            log_dir=log_dir,
            start_at_local_step=args.start_at_local_step,
            stop_after_local_step=args.stop_after_local_step,
            vision_capture=args.vision_capture,
            vision_capture_dir=Path(args.vision_capture_dir).resolve(),
            vision_capture_cameras=args.vision_capture_camera_names,
            vision_capture_timeout_ms=args.vision_capture_timeout_ms,
            vision_capture_interval_s=args.vision_capture_interval_s,
            vision_capture_make_depth_vis=not args.vision_capture_no_depth_vis,
        )
        if result.get("stopped_after_step"):
            save_state(checkpoint, state)
            event(
                "mission_phase_paused",
                state=state,
                stopped_after_step=result["stopped_after_step"],
                note="checkpoint was not advanced; resume or recapture before continuing",
            )
            return state
        state = advance_state(state)
        save_state(checkpoint, state)
        event("mission_phase_done", state=state)
        return state

    if state.phase == "ROD_DONE":
        state = advance_state(state)
        save_state(checkpoint, state)
        event("mission_phase_done", state=state)
        return state

    if state.phase == "MISSION_DONE":
        event("mission_already_done", state=state)
        return state

    raise RuntimeError(f"unknown mission phase: {state.phase}")


def check_map_and_tuned_state() -> None:
    """Assert invariants that should never drift silently during live tuning."""

    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    if config.get("map_id") != ACTIVE_MAP_ID:
        raise RuntimeError(f"expected map_id={ACTIVE_MAP_ID} in {CONFIG}, got {config.get('map_id')!r}")
    if TUNED["place_pull_back_down_z_m"] + TUNED["place_pull_drop_z_m"] != -0.08:
        raise RuntimeError("place pull-out Z tuning must total -0.08m after release")
    event(
        "single_debug_tuned_state_ok",
        site=ACTIVE_SITE_NAME,
        profile=ACTIVE_PROFILE_FILE,
        map_id=config.get("map_id"),
        station_config=CONFIG,
        tuned=TUNED,
    )


def check_required_files(start_index: int, end_index: int) -> None:
    """Verify required scripts, class modules, configs, and pose JSON files."""

    required = [
        PROJECT_ROOT / "industrial_status_snapshot.py",
        PROJECT_ROOT / "move_waist_by_json_path.py",
        PROJECT_ROOT / "move_arm_by_json_path.py",
        PROJECT_ROOT / "move_ee_pose_open_2.py",
        PROJECT_ROOT / "move_ee_pose_close_2.py",
        PROJECT_ROOT / "move_ee_relative_offset.py",
        PACKAGE_DIR / "g2_primitives" / "__init__.py",
        PACKAGE_DIR / "g2_primitives" / "arm.py",
        PACKAGE_DIR / "g2_primitives" / "ee_offset.py",
        PACKAGE_DIR / "g2_primitives" / "gdk_context.py",
        PACKAGE_DIR / "g2_primitives" / "gripper.py",
        PACKAGE_DIR / "g2_primitives" / "nav.py",
        PACKAGE_DIR / "g2_primitives" / "rack.py",
        PACKAGE_DIR / "g2_primitives" / "waist.py",
        PACKAGE_DIR / "industrial_map_nav_guarded.py",
        PACKAGE_DIR / "rack_industrial_docking.py",
        PACKAGE_DIR / "analyze_industrial_cell_run.py",
        PACKAGE_DIR / "process_vision_capture.py",
        PACKAGE_DIR / "site_profile.py",
        CONFIG,
        PLACE_WAIST_JSON,
        PLACE_ABOVE_JSON,
        PLACE_TRANSITION_JSON,
        PLACE_TRANSITION2_JSON,
        ARM_DEFAULT_JSON,
    ]
    if ACTIVE_PROFILE_FILE is not None:
        required.append(ACTIVE_PROFILE_FILE)
    missing = [str(path) for path in required if not path.exists()]
    for rod_index in range(start_index, end_index + 1):
        try:
            latest_rod_grab_pose_json(rod_index)
        except RuntimeError as exc:
            missing.append(str(exc))
    if missing:
        raise RuntimeError("missing required files:\n" + "\n".join(missing))
    for path in (
        PROJECT_ROOT / "industrial_status_snapshot.py",
        PACKAGE_DIR / "industrial_map_nav_guarded.py",
        PACKAGE_DIR / "rack_industrial_docking.py",
        PACKAGE_DIR / "analyze_industrial_cell_run.py",
        PACKAGE_DIR / "process_vision_capture.py",
        PACKAGE_DIR / "site_profile.py",
        PACKAGE_DIR / "g2_primitives" / "__init__.py",
        PACKAGE_DIR / "g2_primitives" / "arm.py",
        PACKAGE_DIR / "g2_primitives" / "ee_offset.py",
        PACKAGE_DIR / "g2_primitives" / "gdk_context.py",
        PACKAGE_DIR / "g2_primitives" / "gripper.py",
        PACKAGE_DIR / "g2_primitives" / "nav.py",
        PACKAGE_DIR / "g2_primitives" / "rack.py",
        PACKAGE_DIR / "g2_primitives" / "waist.py",
        Path(__file__).resolve(),
    ):
        py_compile.compile(str(path), doraise=True)
    event("single_debug_file_check_ok")


def check_existing_processes() -> None:
    """Refuse to start if another known motion/debug process is still running."""

    pattern = (
        "industrial_cell_7_rods_optimized.py|industrial_cell_mission_controller.py|"
        "industrial_map_nav_guarded.py|move_(arm|waist|ee)|rack_industrial_docking.py"
    )
    proc = subprocess.run(["pgrep", "-af", pattern], text=True, capture_output=True)
    if proc.returncode == 0 and proc.stdout.strip():
        raise RuntimeError("existing mission/motion-related process found:\n" + proc.stdout)
    event("single_debug_process_check_ok")


def run_status_snapshot(snapshot_log: Path) -> None:
    """Capture a read-only robot status snapshot before mission execution."""

    command = [sys.executable, str(PROJECT_ROOT / "industrial_status_snapshot.py"), "--samples", "3", "--interval-s", "0.1"]
    return_code = stream_command(command, snapshot_log)
    if return_code != 0:
        raise RuntimeError(f"status snapshot failed return_code={return_code}")


def run_readiness_check(run_log: Path) -> None:
    """Run the navigation readiness check before any live mission phase."""

    from g2_primitives.chassis_motion import ChassisMotionController

    preflight = ChassisMotionController(CONFIG).readiness_check()
    if not preflight.get("ok"):
        raise RuntimeError("readiness check failed: " + ", ".join(preflight.get("problems", [])))


def analyze_run(run_log: Path, summary_log: Path) -> None:
    """Generate the post-run timing summary from the structured run log."""

    if not run_log.exists():
        event("single_debug_analyze_skipped", reason="run log missing", run_log=run_log)
        return
    command = [sys.executable, str(PACKAGE_DIR / "analyze_industrial_cell_run.py"), str(run_log)]
    return_code = stream_command(command, summary_log)
    event("single_debug_analyze_done", return_code=return_code, summary_log=summary_log)


def load_or_create_state(args: argparse.Namespace, checkpoint: Path) -> MissionState:
    """Load an existing checkpoint for resume/status or create a fresh one."""

    if args.status_only or args.resume:
        if not checkpoint.exists():
            raise RuntimeError(f"checkpoint missing: {checkpoint}")
        state = load_state(checkpoint)
    else:
        if checkpoint.exists():
            raise RuntimeError(f"checkpoint already exists; refusing to overwrite: {checkpoint}")
        state = initial_state(args.start_index, args.end_index)
        save_state(checkpoint, state)
    if state.phase not in PHASES:
        raise RuntimeError(f"unknown checkpoint phase: {state.phase}")
    return state


def main() -> int:
    """CLI entrypoint.

    The ordering is part of the safety model:

    1. validate CLI and file/process preconditions;
    2. run read-only status and readiness checks;
    3. create/load checkpoint;
    4. execute one coarse phase at a time;
    5. analyze logs before returning.
    """

    args = parse_args()
    if not (1 <= args.start_index <= 7 and 1 <= args.end_index <= 7 and args.start_index <= args.end_index):
        raise SystemExit("rod range must satisfy 1 <= start <= end <= 7")
    if args.stop_after_rod and not (args.start_index <= args.stop_after_rod <= args.end_index):
        raise SystemExit("--stop-after-rod must be inside the selected rod range")
    if args.live and not args.confirm_physical:
        raise SystemExit("--live requires --confirm-physical")
    if args.vision_capture and args.vision_capture_interval_s <= 0:
        raise SystemExit("--vision-capture-interval-s must be positive")
    if args.preflight_only:
        args.live = False

    checkpoint = Path(args.checkpoint_file).resolve()
    run_log = Path(args.run_log).resolve()
    snapshot_log = Path(args.snapshot_log).resolve()
    summary_log = Path(args.summary_log).resolve()
    Path(args.log_dir).mkdir(parents=True, exist_ok=True)
    global_log, original_stdout, original_stderr = install_global_log(run_log)

    try:
        apply_site_profile(args.profile)
        if args.vision_capture:
            from process_vision_capture import parse_camera_list

            args.vision_capture_camera_names = parse_camera_list(args.vision_capture_cameras)
        else:
            args.vision_capture_camera_names = ()
        event(
            "single_debug_start",
            site=ACTIVE_SITE_NAME,
            profile=ACTIVE_PROFILE_FILE,
            map_id=ACTIVE_MAP_ID,
            live=args.live,
            preflight_only=args.preflight_only,
            checkpoint=checkpoint,
            run_log=run_log,
            start_index=args.start_index,
            end_index=args.end_index,
            resume=args.resume,
            vision_capture=args.vision_capture,
            vision_capture_dir=Path(args.vision_capture_dir).resolve(),
            vision_capture_cameras=args.vision_capture_camera_names,
            vision_capture_interval_s=args.vision_capture_interval_s,
        )

        check_map_and_tuned_state()
        if not args.skip_file_check:
            check_required_files(args.start_index, args.end_index)
        if not args.skip_process_check:
            check_existing_processes()

        if not args.skip_status_snapshot:
            run_status_snapshot(snapshot_log)
        if not args.skip_readiness_check:
            run_readiness_check(run_log)
        if args.preflight_only:
            event("single_debug_preflight_only_done")
            return 0

        state = load_or_create_state(args, checkpoint)
        if args.status_only:
            event("single_debug_status", state=state, next_action=describe_action(state))
            return 0
        if args.live and args.resume and state.holding_rod and not args.allow_holding_resume:
            raise RuntimeError("checkpoint says holding_rod=true; rerun with --allow-holding-resume only after physical confirmation")

        while True:
            if state.phase == "MISSION_DONE":
                event("single_debug_done", state=state)
                sys.stdout.flush()
                analyze_run(run_log, summary_log)
                return 0
            if args.stop_after_rod and state.rod_index > args.stop_after_rod:
                event("single_debug_stopped_by_rod_limit", state=state, stop_after_rod=args.stop_after_rod)
                sys.stdout.flush()
                analyze_run(run_log, summary_log)
                return 0
            before = asdict(state)
            state = execute_current_phase(args, state, checkpoint)
            if asdict(state) == before:
                event("single_debug_paused_no_state_advance", state=state)
                sys.stdout.flush()
                analyze_run(run_log, summary_log)
                return 2
    finally:
        sys.stdout = original_stdout
        sys.stderr = original_stderr
        global_log.close()


if __name__ == "__main__":
    raise SystemExit(main())
