#!/usr/bin/env python3
"""Persistent mission-state controller for the G2 industrial cell.

Default status and dry-run modes do not move the robot. Live staging is limited
to guarded map navigation phases plus no-motion local gates until the rack and
arm primitives are explicitly promoted to physical actions.
"""

from __future__ import annotations

import argparse
import ast
from dataclasses import asdict, dataclass, is_dataclass
import hashlib
import json
from pathlib import Path
import py_compile
import subprocess
import sys
import tempfile
import time


PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = PACKAGE_DIR / "industrial_station_config.json"
DEFAULT_CHECKPOINT = PACKAGE_DIR.parent / "logs" / "industrial_cell_mission_checkpoint.json"
PROJECT_ROOT = PACKAGE_DIR.parent
DEFAULT_ARM_DRY_RUN_BASE_JSON = PROJECT_ROOT.parent / "positions" / "arm_position_to_grab_第一根.json"
DEFAULT_ARM_DRY_RUN_PITCH_M = -0.060
DEFAULT_ARM_GRAB_POSE_DIR = PACKAGE_DIR / "calibration_records"
DEFAULT_PLACE_WAIST_JSON = DEFAULT_ARM_GRAB_POSE_DIR / "rod07_place_waist_adjusted_latest.json"
DEFAULT_PLACE_ABOVE_JSON = DEFAULT_ARM_GRAB_POSE_DIR / "rod07_place_above_arm_latest.json"
DEFAULT_PLACE_TRANSITION_JSON = DEFAULT_ARM_GRAB_POSE_DIR / "rod07_place_transition_arm_latest.json"
DEFAULT_PLACE_TRANSITION2_JSON = DEFAULT_ARM_GRAB_POSE_DIR / "rod07_place_transition2_arm_latest.json"
DEFAULT_PLACE_POSE_JSON = DEFAULT_ARM_GRAB_POSE_DIR / "rod07_place_final_arm_latest.json"
DEFAULT_ARM_DEFAULT_JSON = Path("/data/wxf/wxf/positions/arm_default.json")
FULL_LOCAL_SAFETY_LOCK = PROJECT_ROOT / "logs" / "RACK_FALL_SAFETY_LOCK"
DEFAULT_WAIST_JOINT_SPEED_RADPS = 0.75
DEFAULT_WAIST_MAX_STEP_RAD = 0.75
DEFAULT_WAIST_SETTLE_TOL_RAD = 0.05
DEFAULT_WAIST_SETTLE_TIMEOUT_S = 2.0
DEFAULT_WAIST_POLL_S = 0.08
DEFAULT_PICK_BACK_DOWN_X_M = -0.085
DEFAULT_PLACE_FORWARD_AFTER_FINE_M = 0.0
DEFAULT_PLACE_LATERAL_RIGHT_M = 0.0
DEFAULT_PLACE_GRAB_Z_OFFSET_M = 0.0
DEFAULT_PLACE_FINAL_BEFORE_OPEN_X_M = 0.0
DEFAULT_PLACE_FINAL_BEFORE_OPEN_Z_M = 0.0
DEFAULT_PLACE_PULL_X_M = -0.25
DEFAULT_PLACE_PULL_BACK_DOWN_X_M = 0.0
DEFAULT_PLACE_PULL_BACK_DOWN_Z_M = 0.0
DEFAULT_PLACE_PULL_DROP_AFTER_X_M = 0.0
DEFAULT_PLACE_PULL_DROP_Z_M = 0.0
DEFAULT_CHASSIS_RELATIVE_MAX_ABS_M = 0.20
DEFAULT_CHASSIS_RELATIVE_TIMEOUT_S = 12.0
CHASSIS_RELATIVE_SUCCESS_STATES = (3, 9)
CHASSIS_RELATIVE_RUNNING_STATES = (1, 2, 4, 5, 6, 8)
ARM_DRY_RUN_ALLOWLIST = frozenset({"move_arm_vertical_stack_grab_above.py"})
FINE_POSITION_SUCCESS_STATUSES = frozenset({"stopped", "already_at_threshold"})
RETREAT_SUCCESS_STATUSES = frozenset({"completed"})
RETRYABLE_LOCAL_COMMAND_SCRIPTS = frozenset(
    {
        "move_arm_by_json_path.py",
        "move_waist_by_json_path.py",
        "move_ee_pose_open_2.py",
        "move_ee_pose_close_2.py",
    }
)
DEFAULT_RETRYABLE_LOCAL_COMMAND_ATTEMPTS = 2
DEFAULT_RETRYABLE_LOCAL_COMMAND_DELAY_S = 1.0
DEFAULT_FAST_SAFE_ARM_JOINT_SPEED_RADPS = 0.20
FAST_SAFE_ARM_STEP_LABELS = frozenset({"arm_place_above", "arm_default_after_place"})
ARM_SCRIPT_PATTERNS = (
    "move_ee_pose*.py",
    "move_arm*.py",
    "move_waist*.py",
    "offset_move*.py",
    "end_effector_controller.py",
)
ARM_EXPECTED_SCRIPT_NAMES = (
    "move_ee_pose_open_2.py",
    "move_ee_pose_close_2.py",
    "offset_move_down.py",
    "move_arm_vertical_stack_grab_above.py",
)
DANGEROUS_ARM_TOKENS = (
    "agibot_gdk",
    "gdk_init",
    "move_arm",
    "move_ee",
    "move_waist",
    "adjust_arms_relative",
    "EndEffectorController",
)

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

NEXT_PHASE = {
    "NAV_TO_GRAB": "LOCAL_PICK",
    "LOCAL_PICK": "NAV_TO_PLACE",
    "NAV_TO_PLACE": "LOCAL_PLACE",
    "LOCAL_PLACE": "NAV_TO_RECOVERY",
    "NAV_TO_RECOVERY": "NAV_TO_HOME",
    "NAV_TO_HOME": "ROD_DONE",
    "ROD_DONE": "NAV_TO_GRAB",
}

NAV_PHASE_STATIONS = {
    "NAV_TO_GRAB": "GRAB_PRE",
    "NAV_TO_PLACE": "PLACE_PRE",
    "NAV_TO_RECOVERY": "RECOVERY_SAFE",
    "NAV_TO_HOME": "HOME_SAFE",
}
DEFAULT_REFINE_YAW_STATIONS = ("GRAB_PRE", "PLACE_PRE", "RECOVERY_SAFE", "HOME_SAFE")

LOCAL_PHASES = {
    "LOCAL_PICK": "pick primitive",
    "LOCAL_PLACE": "place primitive",
}

LOCAL_STEP_LABEL_ALIASES = {
    # Historical label from the first waypoint-place patch. The current motion
    # uses arm_default.json, so the precise name is waist_home_after_place.
    "waist_grab_after_place": "waist_home_after_place",
}


@dataclass
class MissionState:
    rod_index: int
    end_index: int
    phase: str
    holding_rod: bool
    current_station: str | None
    last_success_step: str | None
    updated_at: float


def initial_state(start_index: int, end_index: int) -> MissionState:
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
    data = json.loads(path.read_text(encoding="utf-8"))
    return MissionState(**data)


def save_state(path: Path, state: MissionState) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    state.updated_at = time.time()
    path.write_text(json.dumps(asdict(state), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def jsonable(value):
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    return repr(value)


def latest_rod_grab_pose_json(pose_dir: Path, rod_index: int) -> Path:
    pattern = f"rod{rod_index:02d}_grab_pose_*.json"
    matches = sorted(pose_dir.glob(pattern))
    if not matches:
        raise RuntimeError(f"no captured grab pose JSON found for rod {rod_index}: {pose_dir / pattern}")
    return matches[-1]


def describe_action(state: MissionState) -> dict[str, str | int | bool | None]:
    if state.phase in NAV_PHASE_STATIONS:
        station = NAV_PHASE_STATIONS[state.phase]
        action = f"run guarded map navigation to {station}"
    elif state.phase == "LOCAL_PICK":
        action = "run configured local pick primitive"
        station = "GRAB_PRE"
    elif state.phase == "LOCAL_PLACE":
        action = "run configured local place primitive"
        station = "PLACE_PRE"
    elif state.phase == "ROD_DONE":
        action = "mark rod complete and advance to next rod"
        station = state.current_station
    else:
        action = "unknown phase"
        station = state.current_station
    return {
        "rod_index": state.rod_index,
        "phase": state.phase,
        "holding_rod": state.holding_rod,
        "station": station,
        "action": action,
    }


def parse_station_list(value: str) -> set[str]:
    if value.strip().lower() in ("", "none"):
        return set()
    stations = {item.strip() for item in value.split(",") if item.strip()}
    valid = set(NAV_PHASE_STATIONS.values())
    unknown = sorted(stations - valid)
    if unknown:
        raise ValueError(f"unknown station(s) in list: {', '.join(unknown)}; valid={', '.join(sorted(valid))}")
    return stations


def advance_state(state: MissionState, *, direct_home_after_place: bool = False) -> MissionState:
    if state.phase not in NEXT_PHASE:
        raise ValueError(f"unknown phase: {state.phase}")
    previous = state.phase
    state.last_success_step = previous
    if previous == "LOCAL_PLACE" and direct_home_after_place:
        state.phase = "NAV_TO_HOME"
    else:
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


def nav_log_path(log_dir: Path, state: MissionState, station: str) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    name = (
        f"industrial_cell_mission_rod{state.rod_index}_"
        f"{state.phase.lower()}_to_{station.lower()}_{stamp}.log"
    )
    return log_dir / name


def stream_command(command: list[str], log_file: Path) -> int:
    log_file.parent.mkdir(parents=True, exist_ok=True)
    print(
        json.dumps(
            {
                "event": "child_command_start",
                "command": command,
                "log_file": str(log_file),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    with log_file.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="", flush=True)
            log.write(line)
        return_code = process.wait()
    print(
        json.dumps(
            {
                "event": "child_command_done",
                "return_code": return_code,
                "log_file": str(log_file),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return return_code


def _is_docstring_node(node: ast.AST) -> bool:
    return (
        isinstance(node, ast.Expr)
        and isinstance(node.value, ast.Constant)
        and isinstance(node.value.value, str)
    )


def _is_name_constant_compare(test: ast.AST, *, name: str, value: str) -> bool:
    if not isinstance(test, ast.Compare):
        return False
    nodes = [test.left] + list(test.comparators)
    has_name = any(isinstance(node, ast.Name) and node.id == name for node in nodes)
    has_value = any(isinstance(node, ast.Constant) and node.value == value for node in nodes)
    return has_name and has_value


def _is_main_guard(node: ast.AST) -> bool:
    return isinstance(node, ast.If) and _is_name_constant_compare(node.test, name="__name__", value="__main__")


def _node_has_call(node: ast.AST) -> bool:
    return any(isinstance(child, ast.Call) for child in ast.walk(node))


def _compile_script_to_temp(path: Path) -> dict[str, object]:
    digest = hashlib.sha1(str(path).encode("utf-8")).hexdigest()[:16]
    cfile = Path(tempfile.gettempdir()) / f"industrial_arm_gate_{digest}.pyc"
    try:
        py_compile.compile(str(path), cfile=str(cfile), doraise=True)
        return {"ok": True, "error": None}
    except py_compile.PyCompileError as exc:
        return {"ok": False, "error": str(exc)}
    finally:
        try:
            cfile.unlink()
        except FileNotFoundError:
            pass


def discover_arm_scripts(project_root: Path) -> list[Path]:
    paths = {project_root / name for name in ARM_EXPECTED_SCRIPT_NAMES}
    for pattern in ARM_SCRIPT_PATTERNS:
        paths.update(project_root.glob(pattern))
    return sorted(paths, key=lambda item: item.name)


def analyze_arm_script(path: Path) -> dict[str, object]:
    required = path.name in ARM_EXPECTED_SCRIPT_NAMES
    if not path.exists():
        return {
            "script": path.name,
            "path": str(path),
            "required": required,
            "exists": False,
            "compile_ok": False,
            "dry_run_supported": False,
            "main_guard_present": False,
            "top_level_unprotected_calls": [],
            "dangerous_tokens": [],
            "dry_run_execution_allowed": False,
        }

    text = path.read_text(encoding="utf-8")
    compile_result = _compile_script_to_temp(path)
    dry_run_supported = "--dry-run" in text
    dangerous_tokens = [token for token in DANGEROUS_ARM_TOKENS if token in text]
    top_level_calls: list[dict[str, object]] = []
    main_guard_present = False

    try:
        tree = ast.parse(text, filename=str(path))
        for node in tree.body:
            if _is_docstring_node(node):
                continue
            if isinstance(node, (ast.Import, ast.ImportFrom, ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                continue
            if _is_main_guard(node):
                main_guard_present = True
                continue
            if _node_has_call(node):
                top_level_calls.append(
                    {
                        "line": getattr(node, "lineno", None),
                        "node": type(node).__name__,
                    }
                )
    except SyntaxError as exc:
        compile_result = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}

    dry_run_execution_allowed = (
        path.name in ARM_DRY_RUN_ALLOWLIST
        and dry_run_supported
        and bool(compile_result["ok"])
        and not top_level_calls
    )
    return {
        "script": path.name,
        "path": str(path),
        "required": required,
        "exists": True,
        "compile_ok": bool(compile_result["ok"]),
        "compile_error": compile_result["error"],
        "dry_run_supported": dry_run_supported,
        "main_guard_present": main_guard_present,
        "top_level_unprotected_calls": top_level_calls,
        "dangerous_tokens": dangerous_tokens,
        "dry_run_execution_allowed": dry_run_execution_allowed,
        "physical_execution_blocked_by_gate": not dry_run_execution_allowed,
    }


def build_arm_gate_manifest(*, phase: str, rod_index: int, project_root: Path) -> dict[str, object]:
    scripts = [analyze_arm_script(path) for path in discover_arm_scripts(project_root)]
    missing_required = [item["script"] for item in scripts if item["required"] and not item["exists"]]
    compile_failures = [item["script"] for item in scripts if item["exists"] and not item["compile_ok"]]
    allowed_dry_runs = [item["script"] for item in scripts if item["dry_run_execution_allowed"]]
    unsafe_top_level = [
        item["script"]
        for item in scripts
        if item["exists"] and item["top_level_unprotected_calls"]
    ]
    summary = {
        "script_count": len(scripts),
        "missing_required": missing_required,
        "compile_failures": compile_failures,
        "allowed_dry_runs": allowed_dry_runs,
        "unsafe_top_level_scripts": unsafe_top_level,
        "gate_ok": not missing_required and not compile_failures,
    }
    manifest = {
        "mode": "manifest",
        "phase": phase,
        "rod_index": rod_index,
        "project_root": str(project_root),
        "summary": summary,
        "scripts": scripts,
        "note": "manifest only; no arm, gripper, rack docking, or chassis command was sent",
    }
    print(json.dumps({"event": "arm_gate_manifest", **jsonable(manifest)}, ensure_ascii=False), flush=True)
    return manifest


def assert_arm_manifest_gate_ok(manifest: dict[str, object]) -> None:
    summary = manifest["summary"]
    assert isinstance(summary, dict)
    if summary["missing_required"] or summary["compile_failures"]:
        raise RuntimeError(
            "arm gate manifest blocked: "
            f"missing_required={summary['missing_required']} "
            f"compile_failures={summary['compile_failures']}"
        )


def arm_gate_log_path(log_dir: Path, phase: str, rod_index: int, mode: str) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return log_dir / f"industrial_cell_mission_rod{rod_index}_{phase.lower()}_arm_gate_{mode}_{stamp}.log"


def local_action_log_path(log_dir: Path, phase: str, rod_index: int, label: str, *, attempt: int | None = None) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    safe_label = "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in label)
    if attempt is not None:
        safe_label = f"{safe_label}_attempt{attempt}"
    return log_dir / f"industrial_cell_mission_rod{rod_index}_{phase.lower()}_{safe_label}_{stamp}.log"


def run_arm_gate(
    *,
    mode: str,
    phase: str,
    rod_index: int,
    project_root: Path,
    dry_run_base_json: Path,
    dry_run_pitch_m: float,
    log_dir: Path,
) -> dict[str, object] | None:
    if mode == "disabled":
        return None

    manifest = build_arm_gate_manifest(phase=phase, rod_index=rod_index, project_root=project_root)
    assert_arm_manifest_gate_ok(manifest)
    if mode == "manifest":
        return manifest
    if mode != "dryrun":
        raise ValueError(f"unknown arm gate mode: {mode}")
    if phase != "LOCAL_PICK":
        raise RuntimeError("arm dry-run gate is currently allowlisted only for LOCAL_PICK")

    script = project_root / "move_arm_vertical_stack_grab_above.py"
    script_info = next(
        (item for item in manifest["scripts"] if item["script"] == script.name),
        None,
    )
    if not isinstance(script_info, dict) or not script_info.get("dry_run_execution_allowed"):
        raise RuntimeError(f"arm dry-run candidate is not execution-allowed: {script}")
    if not dry_run_base_json.exists():
        raise RuntimeError(f"arm dry-run base JSON missing: {dry_run_base_json}")

    command = [
        sys.executable,
        str(script),
        "--rod-index",
        str(rod_index),
        "--base-index",
        "1",
        "--pitch-m",
        f"{dry_run_pitch_m:.6f}",
        "--base-json",
        str(dry_run_base_json),
        "--dry-run",
    ]
    log_file = arm_gate_log_path(log_dir, phase, rod_index, mode)
    return_code = stream_command(command, log_file)
    result = {
        "mode": "dryrun",
        "phase": phase,
        "rod_index": rod_index,
        "command": command,
        "return_code": return_code,
        "log_file": str(log_file),
        "base_json": str(dry_run_base_json),
        "pitch_m": dry_run_pitch_m,
        "note": "allowlisted dry-run only; GDK init and arm movement are skipped by the child script",
    }
    print(json.dumps({"event": "arm_gate_dryrun", **jsonable(result)}, ensure_ascii=False), flush=True)
    if return_code != 0:
        raise RuntimeError(f"arm dry-run gate failed: return_code={return_code}")
    return result


def run_local_child_command(
    *,
    label: str,
    command: list[str],
    phase: str,
    rod_index: int,
    log_dir: Path,
    max_attempts: int = 1,
    retry_delay_s: float = 0.0,
) -> dict[str, object]:
    max_attempts = max(1, max_attempts)
    start = time.time()
    attempts: list[dict[str, object]] = []
    return_code = 1
    for attempt in range(1, max_attempts + 1):
        log_file = local_action_log_path(log_dir, phase, rod_index, label, attempt=attempt if max_attempts > 1 else None)
        attempt_start = time.time()
        return_code = stream_command(command, log_file)
        attempt_result = {
            "attempt": attempt,
            "max_attempts": max_attempts,
            "return_code": return_code,
            "log_file": str(log_file),
            "elapsed_s": round(time.time() - attempt_start, 3),
        }
        attempts.append(attempt_result)
        print(
            json.dumps(
                {
                    "event": "local_child_step_attempt_done",
                    "label": label,
                    "phase": phase,
                    "rod_index": rod_index,
                    **jsonable(attempt_result),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
        if return_code == 0:
            break
        if attempt < max_attempts:
            print(
                json.dumps(
                    {
                        "event": "local_child_step_retry",
                        "label": label,
                        "phase": phase,
                        "rod_index": rod_index,
                        "attempt": attempt,
                        "next_attempt": attempt + 1,
                        "delay_s": retry_delay_s,
                    },
                    ensure_ascii=False,
                ),
                flush=True,
            )
            if retry_delay_s > 0.0:
                time.sleep(retry_delay_s)
    result = {
        "label": label,
        "phase": phase,
        "rod_index": rod_index,
        "command": command,
        "return_code": return_code,
        "log_file": str(attempts[-1]["log_file"]),
        "attempts": attempts,
        "elapsed_s": round(time.time() - start, 3),
    }
    print(json.dumps({"event": "local_child_step_done", **jsonable(result)}, ensure_ascii=False), flush=True)
    if return_code != 0:
        raise RuntimeError(f"local child step failed: label={label} return_code={return_code}")
    return result


def run_fine_position_step(
    *,
    label: str,
    phase: str,
    rod_index: int,
    final_stop_mm: int,
    final_brake_margin_mm: int,
    final_speed_mps: float,
    max_duration_s: float,
    allow_estop_pedal_fault: bool,
) -> dict[str, object]:
    from rack_industrial_docking import RackIndustrialDockingController

    with RackIndustrialDockingController() as rack:
        result = rack.fine_position(
            final_stop_mm=final_stop_mm,
            final_brake_margin_mm=final_brake_margin_mm,
            final_speed_mps=final_speed_mps,
            max_duration_s=max_duration_s,
            allow_estop_pedal_fault=allow_estop_pedal_fault,
        )
    payload = {
        "label": label,
        "phase": phase,
        "rod_index": rod_index,
        "final_stop_mm": final_stop_mm,
        "final_brake_margin_mm": final_brake_margin_mm,
        "final_speed_mps": final_speed_mps,
        "result": result,
    }
    print(json.dumps({"event": "local_fine_position_done", **jsonable(payload)}, ensure_ascii=False), flush=True)
    if getattr(result, "status", None) not in FINE_POSITION_SUCCESS_STATUSES:
        raise RuntimeError(f"{label} fine positioning failed: {result}")
    return payload


def run_retreat_step(
    *,
    label: str,
    phase: str,
    rod_index: int,
    distance_m: float,
    speed_mps: float,
    allow_estop_pedal_fault: bool,
    success_statuses: set[str] | None = None,
) -> dict[str, object]:
    from rack_industrial_docking import RackIndustrialDockingController

    with RackIndustrialDockingController() as rack:
        result = rack.retreat(
            distance_m=distance_m,
            speed_mps=speed_mps,
            allow_estop_pedal_fault=allow_estop_pedal_fault,
        )
    payload = {
        "label": label,
        "phase": phase,
        "rod_index": rod_index,
        "distance_m": distance_m,
        "speed_mps": speed_mps,
        "result": result,
    }
    print(json.dumps({"event": "local_retreat_done", **jsonable(payload)}, ensure_ascii=False), flush=True)
    allowed_statuses = RETREAT_SUCCESS_STATUSES if success_statuses is None else success_statuses
    if getattr(result, "status", None) not in allowed_statuses:
        raise RuntimeError(f"{label} retreat failed: {result}")
    return payload


def run_chassis_relative_step(
    *,
    label: str,
    phase: str,
    rod_index: int,
    x_m: float,
    y_m: float,
    timeout_s: float,
    max_abs_m: float,
    allow_estop_pedal_fault: bool,
) -> dict[str, object]:
    if timeout_s <= 0.0:
        raise ValueError("timeout_s must be positive")
    if max_abs_m <= 0.0:
        raise ValueError("max_abs_m must be positive")
    if abs(x_m) > max_abs_m or abs(y_m) > max_abs_m:
        raise ValueError(
            f"relative chassis offset exceeds max_abs_m={max_abs_m:.3f}: "
            f"x_m={x_m:.3f} y_m={y_m:.3f}"
        )

    from rack_industrial_docking import RackIndustrialDockingController
    import agibot_gdk

    start = time.time()
    status = "started"
    message = ""
    before_state = None
    before_id = None
    last_state = None
    last_id = None
    final_state = None
    task_id = None
    seen_new_task = False
    seen_running = False

    with RackIndustrialDockingController() as rack:
        preflight = rack.preflight(allow_estop_pedal_fault=allow_estop_pedal_fault)
        if getattr(preflight, "status", None) != "ok":
            payload = {
                "label": label,
                "phase": phase,
                "rod_index": rod_index,
                "status": "blocked",
                "elapsed_s": time.time() - start,
                "x_m": x_m,
                "y_m": y_m,
                "preflight": preflight,
            }
            print(json.dumps({"event": "local_chassis_relative_done", **jsonable(payload)}, ensure_ascii=False), flush=True)
            raise RuntimeError(f"{label} chassis relative move blocked: {preflight}")

        pnc = rack.retreat_controller.rear.pnc
        try:
            before_task = pnc.get_task_state()
            before_state = getattr(before_task, "state", None)
            before_id = getattr(before_task, "id", None)
            if before_state not in (0, 3, 7, 8, 9):
                try:
                    pnc.cancel_task(before_id)
                    time.sleep(0.5)
                except RuntimeError as exc:
                    if "Task is not in RUNNING or PAUSED state" not in str(exc):
                        raise
        except Exception:
            before_id = None

        req = agibot_gdk.NaviReq()
        req.target.position.x = float(x_m)
        req.target.position.y = float(y_m)
        req.target.position.z = 0.0
        req.target.orientation.x = 0.0
        req.target.orientation.y = 0.0
        req.target.orientation.z = 0.0
        req.target.orientation.w = 1.0

        pnc.relative_move(req)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            time.sleep(0.25)
            try:
                task = pnc.get_task_state()
                state = getattr(task, "state", None)
                task_id = getattr(task, "id", None)
                message = getattr(task, "message", "")
            except Exception as exc:
                state = None
                task_id = None
                message = str(exc)

            if task_id is not None and task_id != before_id:
                seen_new_task = True
            if state in CHASSIS_RELATIVE_RUNNING_STATES:
                seen_running = True

            elapsed_s = time.time() - start
            if not seen_new_task and not seen_running:
                if elapsed_s >= 4.0:
                    status = "not_started"
                    break
                continue

            if state == 7:
                status = "canceled"
                final_state = state
                break
            if state in CHASSIS_RELATIVE_SUCCESS_STATES:
                status = "completed"
                final_state = state
                break

            last_state = state
            last_id = task_id

        if status == "started":
            status = "timeout"
            try:
                task = pnc.get_task_state()
                pnc.cancel_task(task.id)
            except Exception:
                pass
            try:
                rack.retreat_controller.rear.stop()
            except Exception:
                pass

    payload = {
        "label": label,
        "phase": phase,
        "rod_index": rod_index,
        "status": status,
        "elapsed_s": time.time() - start,
        "x_m": x_m,
        "y_m": y_m,
        "before_state": before_state,
        "before_task_id": before_id,
        "final_state": final_state,
        "task_id": task_id,
        "last_state": last_state,
        "last_task_id": last_id,
        "message": message,
    }
    print(json.dumps({"event": "local_chassis_relative_done", **jsonable(payload)}, ensure_ascii=False), flush=True)
    if status != "completed":
        raise RuntimeError(f"{label} chassis relative move failed: status={status} message={message}")
    return payload


def full_local_plan(
    *,
    phase: str,
    rod_index: int,
    arm_grab_pose_dir: Path,
    place_waist_json: Path,
    place_above_json: Path,
    place_transition_json: Path,
    place_transition2_json: Path,
    place_pose_json: Path,
    arm_default_json: Path,
    arm_joint_speed_radps: float,
    waist_joint_speed_radps: float,
    grab_final_stop_mm: int,
    grab_final_brake_margin_mm: int,
    grab_final_speed_mps: float,
    place_final_stop_mm: int,
    place_final_brake_margin_mm: int,
    place_final_speed_mps: float,
    pick_down_z_m: float,
    pick_back_x_m: float,
    pick_back_down_x_m: float,
    place_pull_x_m: float,
    place_pull_back_down_x_m: float,
    place_pull_back_down_z_m: float,
    place_pull_drop_after_x_m: float,
    place_pull_drop_z_m: float,
    place_down_z_m: float,
    place_forward_after_fine_m: float,
    place_final_before_open_x_m: float,
    place_final_before_open_z_m: float,
    place_raise_before_open_z_m: float,
    place_lateral_right_m: float,
    place_use_grab_pose: bool,
    place_mirror_grab_waypoints: bool,
    place_grab_z_offset_m: float,
    chassis_relative_max_abs_m: float,
    chassis_relative_timeout_s: float,
    local_retreat_m: float,
    local_retreat_speed_mps: float,
    skip_local_retreat: bool,
    skip_pick_down_after_close: bool,
    skip_pick_offsets_after_close: bool,
    skip_waist_home_after_pick: bool,
    use_place_waypoint_jsons: bool,
    use_place_pose_json: bool,
    skip_place_pose_after_transition2: bool,
    skip_place_pull_out_after_open: bool,
) -> list[dict[str, object]]:
    if phase == "LOCAL_PICK":
        arm_grab_pose_json = latest_rod_grab_pose_json(arm_grab_pose_dir, rod_index)
        steps: list[dict[str, object]] = [
            {"kind": "command", "label": "open_gripper", "script": "move_ee_pose_open_2.py"},
            {
                "kind": "command",
                "label": "waist_for_grab",
                "script": "move_waist_by_json_path.py",
                "json": str(arm_grab_pose_json),
                "joint_speed_radps": waist_joint_speed_radps,
            },
            {
                "kind": "command",
                "label": "arm_grab_pose",
                "script": "move_arm_by_json_path.py",
                "json": str(arm_grab_pose_json),
                "joint_speed_radps": arm_joint_speed_radps,
            },
            {
                "kind": "fine_position",
                "label": "grab_fine_position",
                "final_stop_mm": grab_final_stop_mm,
                "final_brake_margin_mm": grab_final_brake_margin_mm,
                "final_speed_mps": grab_final_speed_mps,
            },
            {"kind": "command", "label": "close_gripper", "script": "move_ee_pose_close_2.py"},
        ]
        if not (skip_pick_offsets_after_close or skip_pick_down_after_close):
            steps.append(
                {
                    "kind": "command",
                    "label": "pick_down_offset",
                    "script": "move_ee_relative_offset.py",
                    "left": (0.0, 0.0, pick_down_z_m),
                    "right": (0.0, 0.0, pick_down_z_m),
                }
            )
        if not skip_pick_offsets_after_close:
            if skip_pick_down_after_close:
                steps.append(
                    {
                        "kind": "command",
                        "label": "pick_back_down_offset",
                        "script": "move_ee_relative_offset.py",
                        "left": (pick_back_down_x_m, 0.0, pick_down_z_m),
                        "right": (pick_back_down_x_m, 0.0, pick_down_z_m),
                    }
                )
                remaining_pick_back_x_m = pick_back_x_m - pick_back_down_x_m
                if abs(remaining_pick_back_x_m) > 1e-6:
                    steps.append(
                        {
                            "kind": "command",
                            "label": "pick_back_remaining_offset",
                            "script": "move_ee_relative_offset.py",
                            "left": (remaining_pick_back_x_m, 0.0, 0.0),
                            "right": (remaining_pick_back_x_m, 0.0, 0.0),
                        }
                    )
            else:
                steps.append(
                    {
                        "kind": "command",
                        "label": "pick_back_offset",
                        "script": "move_ee_relative_offset.py",
                        "left": (pick_back_x_m, 0.0, 0.0),
                        "right": (pick_back_x_m, 0.0, 0.0),
                    }
                )
        if not skip_local_retreat:
            steps.append(
                {
                    "kind": "retreat",
                    "label": "retreat_after_pick",
                    "distance_m": local_retreat_m,
                    "speed_mps": local_retreat_speed_mps,
                }
            )
        if not skip_waist_home_after_pick:
            steps.append(
                {
                    "kind": "command",
                    "label": "waist_home_after_pick",
                    "script": "move_waist_by_json_path.py",
                    "json": str(arm_default_json),
                    "joint_speed_radps": waist_joint_speed_radps,
                }
            )
        return steps

    if phase == "LOCAL_PLACE":
        arm_grab_pose_json = latest_rod_grab_pose_json(arm_grab_pose_dir, rod_index)
        place_fine_step = {
            "kind": "fine_position",
            "label": "place_fine_position",
            "final_stop_mm": place_final_stop_mm,
            "final_brake_margin_mm": place_final_brake_margin_mm,
            "final_speed_mps": place_final_speed_mps,
        }
        if place_mirror_grab_waypoints:
            place_above_index = max(1, rod_index - 2)
            place_transition_index = max(1, rod_index - 1)
            place_above_pose_json = latest_rod_grab_pose_json(arm_grab_pose_dir, place_above_index)
            place_transition_pose_json = latest_rod_grab_pose_json(arm_grab_pose_dir, place_transition_index)
            steps = [
                {
                    "kind": "command",
                    "label": "waist_place_above",
                    "script": "move_waist_by_json_path.py",
                    "json": str(place_above_pose_json),
                    "joint_speed_radps": waist_joint_speed_radps,
                },
                {
                    "kind": "command",
                    "label": "arm_place_above",
                    "script": "move_arm_by_json_path.py",
                    "json": str(place_above_pose_json),
                    "joint_speed_radps": arm_step_joint_speed("arm_place_above", arm_joint_speed_radps),
                },
                place_fine_step,
            ]
        elif place_use_grab_pose:
            steps = [
                {
                    "kind": "command",
                    "label": "arm_place_grab_pose",
                    "script": "move_arm_by_json_path.py",
                    "json": str(arm_grab_pose_json),
                    "joint_speed_radps": arm_joint_speed_radps,
                }
            ]
            if abs(place_grab_z_offset_m) > 1e-6:
                steps.append(
                    {
                        "kind": "command",
                        "label": "place_grab_pose_z_offset",
                        "script": "move_ee_relative_offset.py",
                        "left": (0.0, 0.0, place_grab_z_offset_m),
                        "right": (0.0, 0.0, place_grab_z_offset_m),
                    }
                )
            steps.append(place_fine_step)
        elif use_place_waypoint_jsons:
            steps = [
                {
                    "kind": "command",
                    "label": "waist_place_straight",
                    "script": "move_waist_by_json_path.py",
                    "json": str(place_waist_json),
                    "joint_speed_radps": waist_joint_speed_radps,
                },
                {
                    "kind": "command",
                    "label": "arm_place_above",
                    "script": "move_arm_by_json_path.py",
                    "json": str(place_above_json),
                    "joint_speed_radps": arm_step_joint_speed("arm_place_above", arm_joint_speed_radps),
                },
                place_fine_step,
            ]
        else:
            steps = [
                {
                    "kind": "command",
                    "label": "arm_place_above",
                    "script": "move_arm_by_json_path.py",
                    "json": str(place_above_json),
                    "joint_speed_radps": arm_step_joint_speed("arm_place_above", arm_joint_speed_radps),
                },
                place_fine_step,
            ]
        if place_forward_after_fine_m > 0.0:
            steps.append(
                {
                    "kind": "chassis_relative",
                    "label": "place_forward_after_fine_offset",
                    "x_m": place_forward_after_fine_m,
                    "y_m": 0.0,
                    "forward_m": place_forward_after_fine_m,
                    "max_abs_m": chassis_relative_max_abs_m,
                    "timeout_s": chassis_relative_timeout_s,
                }
            )
        if place_lateral_right_m > 0.0:
            steps.append(
                {
                    "kind": "chassis_relative",
                    "label": "place_lateral_right_offset",
                    "x_m": 0.0,
                    "y_m": -place_lateral_right_m,
                    "right_m": place_lateral_right_m,
                    "max_abs_m": chassis_relative_max_abs_m,
                    "timeout_s": chassis_relative_timeout_s,
                }
            )
        if place_mirror_grab_waypoints:
            steps.extend(
                [
                    {
                        "kind": "command",
                        "label": "waist_place_transition",
                        "script": "move_waist_by_json_path.py",
                        "json": str(place_transition_pose_json),
                        "joint_speed_radps": waist_joint_speed_radps,
                    },
                    {
                        "kind": "command",
                        "label": "arm_place_transition",
                        "script": "move_arm_by_json_path.py",
                        "json": str(place_transition_pose_json),
                        "joint_speed_radps": arm_joint_speed_radps,
                    },
                    {
                        "kind": "command",
                        "label": "waist_place_pose",
                        "script": "move_waist_by_json_path.py",
                        "json": str(arm_grab_pose_json),
                        "joint_speed_radps": waist_joint_speed_radps,
                    },
                    {
                        "kind": "command",
                        "label": "arm_place_pose",
                        "script": "move_arm_by_json_path.py",
                        "json": str(arm_grab_pose_json),
                        "joint_speed_radps": arm_joint_speed_radps,
                    },
                ]
            )
        elif not place_use_grab_pose and use_place_waypoint_jsons:
            steps.extend(
                [
                    {
                        "kind": "command",
                        "label": "arm_place_transition",
                        "script": "move_arm_by_json_path.py",
                        "json": str(place_transition_json),
                        "joint_speed_radps": arm_joint_speed_radps,
                    },
                    {
                        "kind": "command",
                        "label": "arm_place_transition2",
                        "script": "move_arm_by_json_path.py",
                        "json": str(place_transition2_json),
                        "joint_speed_radps": arm_joint_speed_radps,
                    },
                ]
            )
            if not skip_place_pose_after_transition2:
                steps.append(
                    {
                        "kind": "command",
                        "label": "arm_place_pose",
                        "script": "move_arm_by_json_path.py",
                        "json": str(place_pose_json),
                        "joint_speed_radps": arm_joint_speed_radps,
                    }
                )
        elif not place_use_grab_pose and use_place_pose_json:
            steps.append(
                {
                    "kind": "command",
                    "label": "arm_place_pose",
                    "script": "move_arm_by_json_path.py",
                    "json": str(place_pose_json),
                    "joint_speed_radps": arm_joint_speed_radps,
                }
            )
        elif not place_use_grab_pose:
            steps.append(
                {
                    "kind": "command",
                    "label": "place_down_offset",
                    "script": "move_ee_relative_offset.py",
                    "left": (0.0, 0.0, place_down_z_m),
                    "right": (0.0, 0.0, place_down_z_m),
                }
            )
        if abs(place_final_before_open_x_m) > 1e-6 or abs(place_final_before_open_z_m) > 1e-6:
            steps.append(
                {
                    "kind": "command",
                    "label": "place_final_before_open_offset",
                    "script": "move_ee_relative_offset.py",
                    "left": (place_final_before_open_x_m, 0.0, place_final_before_open_z_m),
                    "right": (place_final_before_open_x_m, 0.0, place_final_before_open_z_m),
                }
            )
        if abs(place_raise_before_open_z_m) > 1e-6:
            steps.append(
                {
                    "kind": "command",
                    "label": "place_raise_before_open_offset",
                    "script": "move_ee_relative_offset.py",
                    "left": (0.0, 0.0, place_raise_before_open_z_m),
                    "right": (0.0, 0.0, place_raise_before_open_z_m),
                }
            )
        steps.append({"kind": "command", "label": "open_gripper_place", "script": "move_ee_pose_open_2.py"})
        if not skip_place_pull_out_after_open:
            steps.extend(
                place_pull_out_steps(
                    place_pull_x_m=place_pull_x_m,
                    place_pull_back_down_x_m=place_pull_back_down_x_m,
                    place_pull_back_down_z_m=place_pull_back_down_z_m,
                    place_pull_drop_after_x_m=place_pull_drop_after_x_m,
                    place_pull_drop_z_m=place_pull_drop_z_m,
                )
            )
        if not skip_local_retreat:
            steps.append(
                {
                    "kind": "retreat",
                    "label": "retreat_after_place",
                    "distance_m": local_retreat_m,
                    "speed_mps": local_retreat_speed_mps,
                    "success_statuses": tuple(sorted(RETREAT_SUCCESS_STATUSES | {"rear_obstacle"})),
                }
            )
        steps.append(
            {
                "kind": "command",
                "label": "arm_default_after_place",
                "script": "move_arm_by_json_path.py",
                "json": str(arm_default_json),
                "joint_speed_radps": arm_step_joint_speed("arm_default_after_place", arm_joint_speed_radps),
            }
        )
        steps.append(
            {
                "kind": "command",
                "label": "waist_home_after_place",
                "script": "move_waist_by_json_path.py",
                "json": str(arm_default_json),
                "joint_speed_radps": waist_joint_speed_radps,
            }
        )
        return steps
    raise ValueError(f"full local action is unsupported for phase: {phase}")


def same_direction_or_zero(reference: float, value: float) -> bool:
    return value == 0.0 or reference == 0.0 or reference * value > 0.0


def arm_step_joint_speed(label: str, default_speed_radps: float) -> float:
    if label in FAST_SAFE_ARM_STEP_LABELS:
        return DEFAULT_FAST_SAFE_ARM_JOINT_SPEED_RADPS
    return default_speed_radps


def place_pull_out_steps(
    *,
    place_pull_x_m: float,
    place_pull_back_down_x_m: float,
    place_pull_back_down_z_m: float,
    place_pull_drop_after_x_m: float,
    place_pull_drop_z_m: float,
) -> list[dict[str, object]]:
    steps: list[dict[str, object]] = []
    x_done_m = 0.0

    if abs(place_pull_back_down_x_m) > 1e-6 or abs(place_pull_back_down_z_m) > 1e-6:
        steps.append(
            {
                "kind": "command",
                "label": "place_pull_back_down_offset",
                "script": "move_ee_relative_offset.py",
                "left": (place_pull_back_down_x_m, 0.0, place_pull_back_down_z_m),
                "right": (place_pull_back_down_x_m, 0.0, place_pull_back_down_z_m),
            }
        )
        x_done_m += place_pull_back_down_x_m

    if abs(place_pull_drop_after_x_m) > 1e-6:
        before_drop_x_m = place_pull_drop_after_x_m - x_done_m
        if abs(before_drop_x_m) > 1e-6:
            steps.append(
                {
                    "kind": "command",
                    "label": "place_pull_back_before_drop",
                    "script": "move_ee_relative_offset.py",
                    "left": (before_drop_x_m, 0.0, 0.0),
                    "right": (before_drop_x_m, 0.0, 0.0),
                }
            )
            x_done_m += before_drop_x_m
        if abs(place_pull_drop_z_m) > 1e-6:
            steps.append(
                {
                    "kind": "command",
                    "label": "place_pull_drop_offset",
                    "script": "move_ee_relative_offset.py",
                    "left": (0.0, 0.0, place_pull_drop_z_m),
                    "right": (0.0, 0.0, place_pull_drop_z_m),
                }
            )

    remaining_x_m = place_pull_x_m - x_done_m
    if abs(remaining_x_m) > 1e-6:
        label = "place_pull_back_remaining_offset" if steps else "place_pull_out_offset"
        steps.append(
            {
                "kind": "command",
                "label": label,
                "script": "move_ee_relative_offset.py",
                "left": (remaining_x_m, 0.0, 0.0),
                "right": (remaining_x_m, 0.0, 0.0),
            }
        )
    return steps


def command_for_full_step(
    *,
    step: dict[str, object],
    rod_index: int,
    arm_joint_speed_radps: float,
    arm_settle_s: float,
    waist_settle_s: float,
    offset_settle_s: float,
    offset_max_abs_m: float,
) -> list[str]:
    script = step["script"]
    if script == "move_ee_pose_open_2.py" or script == "move_ee_pose_close_2.py":
        return [sys.executable, str(PROJECT_ROOT / str(script))]
    if script == "move_waist_by_json_path.py":
        return [
            sys.executable,
            str(PROJECT_ROOT / str(script)),
            "--json",
            str(step["json"]),
            "--joint-speed-radps",
            f"{float(step.get('joint_speed_radps', arm_joint_speed_radps)):.6f}",
            "--max-step-rad",
            f"{DEFAULT_WAIST_MAX_STEP_RAD:.6f}",
            "--settle-tol-rad",
            f"{DEFAULT_WAIST_SETTLE_TOL_RAD:.6f}",
            "--settle-timeout-s",
            f"{DEFAULT_WAIST_SETTLE_TIMEOUT_S:.6f}",
            "--poll-s",
            f"{DEFAULT_WAIST_POLL_S:.6f}",
            "--settle-s",
            f"{waist_settle_s:.6f}",
        ]
    if script == "move_arm_by_json_path.py":
        return [
            sys.executable,
            str(PROJECT_ROOT / str(script)),
            "--json",
            str(step["json"]),
            "--joint-speed-radps",
            f"{float(step.get('joint_speed_radps', arm_joint_speed_radps)):.6f}",
            "--settle-s",
            f"{arm_settle_s:.6f}",
        ]
    if script == "move_ee_relative_offset.py":
        left = step["left"]
        right = step["right"]
        assert isinstance(left, tuple) and isinstance(right, tuple)
        left_arg = ",".join(f"{value:.6f}" for value in left)
        right_arg = ",".join(f"{value:.6f}" for value in right)
        return [
            sys.executable,
            str(PROJECT_ROOT / str(script)),
            f"--left={left_arg}",
            f"--right={right_arg}",
            "--max-abs-m",
            f"{offset_max_abs_m:.6f}",
            "--settle-s",
            f"{offset_settle_s:.6f}",
        ]
    raise ValueError(f"unsupported full local command step: {script}")


def run_full_local_action(
    *,
    mode: str,
    phase: str,
    rod_index: int,
    confirm_local_physical: bool,
    allow_estop_pedal_fault: bool,
    log_dir: Path,
    arm_grab_pose_dir: Path,
    place_waist_json: Path,
    place_above_json: Path,
    place_transition_json: Path,
    place_transition2_json: Path,
    place_pose_json: Path,
    arm_default_json: Path,
    arm_joint_speed_radps: float,
    waist_joint_speed_radps: float,
    arm_settle_s: float,
    waist_settle_s: float,
    offset_settle_s: float,
    grab_final_stop_mm: int,
    grab_final_brake_margin_mm: int,
    grab_final_speed_mps: float,
    place_final_stop_mm: int,
    place_final_brake_margin_mm: int,
    place_final_speed_mps: float,
    fine_position_max_duration_s: float,
    pick_down_z_m: float,
    pick_back_x_m: float,
    pick_back_down_x_m: float,
    place_pull_x_m: float,
    place_pull_back_down_x_m: float,
    place_pull_back_down_z_m: float,
    place_pull_drop_after_x_m: float,
    place_pull_drop_z_m: float,
    place_down_z_m: float,
    place_forward_after_fine_m: float,
    place_final_before_open_x_m: float,
    place_final_before_open_z_m: float,
    place_raise_before_open_z_m: float,
    place_lateral_right_m: float,
    place_use_grab_pose: bool,
    place_mirror_grab_waypoints: bool,
    place_grab_z_offset_m: float,
    chassis_relative_max_abs_m: float,
    chassis_relative_timeout_s: float,
    offset_max_abs_m: float,
    local_retreat_m: float,
    local_retreat_speed_mps: float,
    skip_local_retreat: bool,
    skip_pick_down_after_close: bool,
    skip_pick_offsets_after_close: bool,
    skip_waist_home_after_pick: bool,
    use_place_waypoint_jsons: bool,
    use_place_pose_json: bool,
    skip_place_pose_after_transition2: bool,
    skip_place_pull_out_after_open: bool,
    start_at_local_step: str | None,
    stop_after_local_step: str | None,
) -> dict[str, object]:
    plan = full_local_plan(
        phase=phase,
        rod_index=rod_index,
        arm_grab_pose_dir=arm_grab_pose_dir,
        place_waist_json=place_waist_json,
        place_above_json=place_above_json,
        place_transition_json=place_transition_json,
        place_transition2_json=place_transition2_json,
        place_pose_json=place_pose_json,
        arm_default_json=arm_default_json,
        arm_joint_speed_radps=arm_joint_speed_radps,
        waist_joint_speed_radps=waist_joint_speed_radps,
        grab_final_stop_mm=grab_final_stop_mm,
        grab_final_brake_margin_mm=grab_final_brake_margin_mm,
        grab_final_speed_mps=grab_final_speed_mps,
        place_final_stop_mm=place_final_stop_mm,
        place_final_brake_margin_mm=place_final_brake_margin_mm,
        place_final_speed_mps=place_final_speed_mps,
        pick_down_z_m=pick_down_z_m,
        pick_back_x_m=pick_back_x_m,
        pick_back_down_x_m=pick_back_down_x_m,
        place_pull_x_m=place_pull_x_m,
        place_pull_back_down_x_m=place_pull_back_down_x_m,
        place_pull_back_down_z_m=place_pull_back_down_z_m,
        place_pull_drop_after_x_m=place_pull_drop_after_x_m,
        place_pull_drop_z_m=place_pull_drop_z_m,
        place_down_z_m=place_down_z_m,
        place_forward_after_fine_m=place_forward_after_fine_m,
        place_final_before_open_x_m=place_final_before_open_x_m,
        place_final_before_open_z_m=place_final_before_open_z_m,
        place_raise_before_open_z_m=place_raise_before_open_z_m,
        place_lateral_right_m=place_lateral_right_m,
        place_use_grab_pose=place_use_grab_pose,
        place_mirror_grab_waypoints=place_mirror_grab_waypoints,
        place_grab_z_offset_m=place_grab_z_offset_m,
        chassis_relative_max_abs_m=chassis_relative_max_abs_m,
        chassis_relative_timeout_s=chassis_relative_timeout_s,
        local_retreat_m=local_retreat_m,
        local_retreat_speed_mps=local_retreat_speed_mps,
        skip_local_retreat=skip_local_retreat,
        skip_pick_down_after_close=skip_pick_down_after_close,
        skip_pick_offsets_after_close=skip_pick_offsets_after_close,
        skip_waist_home_after_pick=skip_waist_home_after_pick,
        use_place_waypoint_jsons=use_place_waypoint_jsons,
        use_place_pose_json=use_place_pose_json,
        skip_place_pose_after_transition2=skip_place_pose_after_transition2,
        skip_place_pull_out_after_open=skip_place_pull_out_after_open,
    )
    start_at_local_step = LOCAL_STEP_LABEL_ALIASES.get(start_at_local_step or "", start_at_local_step)
    stop_after_local_step = LOCAL_STEP_LABEL_ALIASES.get(stop_after_local_step or "", stop_after_local_step)
    labels = [str(step["label"]) for step in plan]
    missing_labels = [
        label
        for label in (start_at_local_step, stop_after_local_step)
        if label and label not in labels
    ]
    if missing_labels:
        raise ValueError(
            "unknown local step label(s): "
            + ", ".join(missing_labels)
            + "; available labels: "
            + ", ".join(labels)
        )
    start_index = 0
    if start_at_local_step:
        for index, step in enumerate(plan):
            if str(step["label"]) == start_at_local_step:
                start_index = index
                break
    execution_plan = plan[start_index:]
    skipped_steps = plan[:start_index]
    print(
        json.dumps(
            {
                "event": "local_full_action_plan",
                "mode": mode,
                "phase": phase,
                "rod_index": rod_index,
                "start_at_local_step": start_at_local_step,
                "stop_after_local_step": stop_after_local_step,
                "skipped_steps": jsonable(skipped_steps),
                "steps": jsonable(execution_plan),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    if mode == "full-dry-run":
        stopped_after_step = (
            stop_after_local_step
            if stop_after_local_step and any(str(step["label"]) == stop_after_local_step for step in execution_plan)
            else None
        )
        return {
            "mode": mode,
            "phase": phase,
            "rod_index": rod_index,
            "steps": execution_plan,
            "skipped_steps": skipped_steps,
            "start_at_local_step": start_at_local_step,
            "stop_after_local_step": stop_after_local_step,
            "stopped_after_step": stopped_after_step,
            "note": "plan only; no arm, gripper, rack, or chassis command was sent",
        }
    if mode != "full":
        raise ValueError(f"unknown full local mode: {mode}")
    if not confirm_local_physical:
        raise RuntimeError("--local-action-mode full requires --confirm-local-physical")

    results: list[dict[str, object]] = []
    for step in execution_plan:
        kind = step["kind"]
        label = str(step["label"])
        if kind == "command":
            command = command_for_full_step(
                step=step,
                    rod_index=rod_index,
                    arm_joint_speed_radps=arm_joint_speed_radps,
                    arm_settle_s=arm_settle_s,
                    waist_settle_s=waist_settle_s,
                    offset_settle_s=offset_settle_s,
                    offset_max_abs_m=offset_max_abs_m,
                )
            results.append(
                run_local_child_command(
                    label=label,
                    command=command,
                    phase=phase,
                    rod_index=rod_index,
                    log_dir=log_dir,
                    max_attempts=(
                        DEFAULT_RETRYABLE_LOCAL_COMMAND_ATTEMPTS
                        if str(step.get("script")) in RETRYABLE_LOCAL_COMMAND_SCRIPTS
                        else 1
                    ),
                    retry_delay_s=DEFAULT_RETRYABLE_LOCAL_COMMAND_DELAY_S,
                )
            )
        elif kind == "fine_position":
            results.append(
                run_fine_position_step(
                    label=label,
                    phase=phase,
                    rod_index=rod_index,
                    final_stop_mm=int(step["final_stop_mm"]),
                    final_brake_margin_mm=int(step["final_brake_margin_mm"]),
                    final_speed_mps=float(step["final_speed_mps"]),
                    max_duration_s=fine_position_max_duration_s,
                    allow_estop_pedal_fault=allow_estop_pedal_fault,
                )
            )
        elif kind == "retreat":
            results.append(
                run_retreat_step(
                    label=label,
                    phase=phase,
                    rod_index=rod_index,
                    distance_m=float(step["distance_m"]),
                    speed_mps=float(step["speed_mps"]),
                    allow_estop_pedal_fault=allow_estop_pedal_fault,
                    success_statuses=set(step["success_statuses"]) if "success_statuses" in step else None,
                )
            )
        elif kind == "chassis_relative":
            results.append(
                run_chassis_relative_step(
                    label=label,
                    phase=phase,
                    rod_index=rod_index,
                    x_m=float(step["x_m"]),
                    y_m=float(step["y_m"]),
                    timeout_s=float(step["timeout_s"]),
                    max_abs_m=float(step["max_abs_m"]),
                    allow_estop_pedal_fault=allow_estop_pedal_fault,
                )
            )
        else:
            raise RuntimeError(f"unknown full local step kind: {kind}")
        if stop_after_local_step == label:
            payload = {
                "mode": mode,
                "phase": phase,
                "rod_index": rod_index,
                "stopped_after_step": label,
                "checkpoint_advanced": False,
                "results": results,
            }
            print(json.dumps({"event": "local_full_action_stopped", **jsonable(payload)}, ensure_ascii=False), flush=True)
            return payload
    return {
        "mode": mode,
        "phase": phase,
        "rod_index": rod_index,
        "results": results,
    }


def run_local_readonly_gate(
    *,
    phase: str,
    rod_index: int,
    allow_estop_pedal_fault: bool,
    rack_read_samples: int,
    rack_read_interval_s: float,
) -> dict[str, object]:
    """Read-only rack/drive preflight and sensor snapshot gate."""
    if rack_read_samples <= 0:
        raise ValueError("rack_read_samples must be positive")
    if rack_read_interval_s < 0.0:
        raise ValueError("rack_read_interval_s must be >= 0")

    from rack_industrial_docking import RackIndustrialDockingController

    with RackIndustrialDockingController() as rack:
        preflight = rack.preflight(allow_estop_pedal_fault=allow_estop_pedal_fault)
        samples = rack.read_snapshots(samples=rack_read_samples, interval_s=rack_read_interval_s)

    result = {
        "phase": phase,
        "rod_index": rod_index,
        "preflight": preflight,
        "samples": samples,
        "sample_count": len(samples),
        "note": "read-only local gate; no arm, gripper, rack docking, or chassis command was sent",
    }
    print(json.dumps({"event": "local_action_readonly", **jsonable(result)}, ensure_ascii=False), flush=True)
    if getattr(preflight, "status", None) != "ok":
        raise RuntimeError(f"local read-only preflight blocked: {preflight}")
    return result


def build_nav_command(
    *,
    config: Path,
    station: str,
    confirm_live: bool,
    refine_yaw: bool,
    refine_yaw_tolerance_deg: float,
    refine_yaw_max_error_deg: float,
    refine_yaw_angular_speed_radps: float,
    refine_yaw_fine_angular_speed_radps: float,
    refine_yaw_timeout_s: float,
) -> list[str]:
    command = [
        sys.executable,
        str(PACKAGE_DIR / "industrial_map_nav_guarded.py"),
        "--config",
        str(config),
        "--station",
        station,
    ]
    if confirm_live:
        command.append("--confirm-live")
        if refine_yaw:
            command.extend(
                [
                    "--refine-yaw",
                    "--refine-yaw-tolerance-deg",
                    str(refine_yaw_tolerance_deg),
                    "--refine-yaw-max-error-deg",
                    str(refine_yaw_max_error_deg),
                    "--refine-yaw-angular-speed-radps",
                    str(refine_yaw_angular_speed_radps),
                    "--refine-yaw-fine-angular-speed-radps",
                    str(refine_yaw_fine_angular_speed_radps),
                    "--refine-yaw-timeout-s",
                    str(refine_yaw_timeout_s),
                ]
            )
    else:
        command.append("--dry-run")
    return command


def execute_current_phase(
    *,
    state: MissionState,
    checkpoint: Path,
    config: Path,
    staging: bool,
    confirm_live: bool,
    local_action_mode: str,
    nav_log_dir: Path,
    refine_yaw: bool,
    refine_yaw_stations: set[str],
    refine_yaw_tolerance_deg: float,
    refine_yaw_max_error_deg: float,
    refine_yaw_angular_speed_radps: float,
    refine_yaw_fine_angular_speed_radps: float,
    refine_yaw_timeout_s: float,
    allow_estop_pedal_fault: bool,
    rack_read_samples: int,
    rack_read_interval_s: float,
    arm_gate_mode: str,
    arm_dry_run_base_json: Path,
    arm_dry_run_pitch_m: float,
    arm_grab_pose_dir: Path,
    arm_gate_log_dir: Path,
    confirm_local_physical: bool,
    full_local_log_dir: Path,
    place_waist_json: Path,
    place_above_json: Path,
    place_transition_json: Path,
    place_transition2_json: Path,
    place_pose_json: Path,
    arm_default_json: Path,
    arm_joint_speed_radps: float,
    waist_joint_speed_radps: float,
    arm_settle_s: float,
    waist_settle_s: float,
    offset_settle_s: float,
    grab_final_stop_mm: int,
    grab_final_brake_margin_mm: int,
    grab_final_speed_mps: float,
    place_final_stop_mm: int,
    place_final_brake_margin_mm: int,
    place_final_speed_mps: float,
    fine_position_max_duration_s: float,
    pick_down_z_m: float,
    pick_back_x_m: float,
    pick_back_down_x_m: float,
    place_pull_x_m: float,
    place_pull_back_down_x_m: float,
    place_pull_back_down_z_m: float,
    place_pull_drop_after_x_m: float,
    place_pull_drop_z_m: float,
    place_down_z_m: float,
    place_forward_after_fine_m: float,
    place_final_before_open_x_m: float,
    place_final_before_open_z_m: float,
    place_raise_before_open_z_m: float,
    place_lateral_right_m: float,
    place_use_grab_pose: bool,
    place_mirror_grab_waypoints: bool,
    place_grab_z_offset_m: float,
    chassis_relative_max_abs_m: float,
    chassis_relative_timeout_s: float,
    offset_max_abs_m: float,
    local_retreat_m: float,
    local_retreat_speed_mps: float,
    skip_local_retreat: bool,
    skip_pick_down_after_close: bool,
    skip_pick_offsets_after_close: bool,
    skip_waist_home_after_pick: bool,
    use_place_waypoint_jsons: bool,
    use_place_pose_json: bool,
    skip_place_pose_after_transition2: bool,
    skip_place_pull_out_after_open: bool,
    start_at_local_step: str | None,
    stop_after_local_step: str | None,
    direct_home_after_place: bool,
) -> MissionState:
    if state.phase == "MISSION_DONE":
        print(json.dumps({"event": "mission_already_done", "state": asdict(state)}, ensure_ascii=False))
        return state

    action = describe_action(state)
    print(json.dumps({"event": "mission_phase_start", "action": action, "state": asdict(state)}, ensure_ascii=False))

    if state.phase in NAV_PHASE_STATIONS:
        if not confirm_live and not staging:
            raise RuntimeError("dry-run navigation checkpoint advancement requires --staging")
        station = NAV_PHASE_STATIONS[state.phase]
        command = build_nav_command(
            config=config,
            station=station,
            confirm_live=confirm_live,
            refine_yaw=refine_yaw and station in refine_yaw_stations,
            refine_yaw_tolerance_deg=refine_yaw_tolerance_deg,
            refine_yaw_max_error_deg=refine_yaw_max_error_deg,
            refine_yaw_angular_speed_radps=refine_yaw_angular_speed_radps,
            refine_yaw_fine_angular_speed_radps=refine_yaw_fine_angular_speed_radps,
            refine_yaw_timeout_s=refine_yaw_timeout_s,
        )
        return_code = stream_command(command, nav_log_path(nav_log_dir, state, station))
        if return_code != 0:
            raise RuntimeError(f"guarded navigation failed for {station}: return_code={return_code}")
        state = advance_state(state, direct_home_after_place=direct_home_after_place)
        save_state(checkpoint, state)
        print(json.dumps({"event": "mission_phase_done", "state": asdict(state)}, ensure_ascii=False))
        return state

    if state.phase in LOCAL_PHASES:
        if not staging or local_action_mode not in ("noop", "readonly", "full-dry-run", "full"):
            raise RuntimeError(
                f"{state.phase} is not implemented for physical action; use "
                "--staging --local-action-mode noop, readonly, full-dry-run, or full"
            )
        if local_action_mode == "noop":
            print(
                json.dumps(
                    {
                        "event": "local_action_noop",
                        "phase": state.phase,
                        "rod_index": state.rod_index,
                        "holding_rod_before": state.holding_rod,
                        "note": "no arm, no gripper, no rack docking command was sent",
                    },
                    ensure_ascii=False,
                )
            )
            run_arm_gate(
                mode=arm_gate_mode,
                phase=state.phase,
                rod_index=state.rod_index,
                project_root=PROJECT_ROOT,
                dry_run_base_json=arm_dry_run_base_json,
                dry_run_pitch_m=arm_dry_run_pitch_m,
                log_dir=arm_gate_log_dir,
            )
        elif local_action_mode == "readonly":
            run_local_readonly_gate(
                phase=state.phase,
                rod_index=state.rod_index,
                allow_estop_pedal_fault=allow_estop_pedal_fault,
                rack_read_samples=rack_read_samples,
                rack_read_interval_s=rack_read_interval_s,
            )
            run_arm_gate(
                mode=arm_gate_mode,
                phase=state.phase,
                rod_index=state.rod_index,
                project_root=PROJECT_ROOT,
                dry_run_base_json=arm_dry_run_base_json,
                dry_run_pitch_m=arm_dry_run_pitch_m,
                log_dir=arm_gate_log_dir,
            )
            print(
                json.dumps(
                    {
                        "event": "mission_phase_paused",
                        "state": asdict(state),
                        "note": "readonly local action did not advance the checkpoint",
                    },
                    ensure_ascii=False,
                )
            )
            return state
        elif local_action_mode in ("full-dry-run", "full"):
            local_result = run_full_local_action(
                mode=local_action_mode,
                phase=state.phase,
                rod_index=state.rod_index,
                confirm_local_physical=confirm_local_physical,
                allow_estop_pedal_fault=allow_estop_pedal_fault,
                log_dir=full_local_log_dir,
                arm_grab_pose_dir=arm_grab_pose_dir,
                place_waist_json=place_waist_json,
                place_above_json=place_above_json,
                place_transition_json=place_transition_json,
                place_transition2_json=place_transition2_json,
                place_pose_json=place_pose_json,
                arm_default_json=arm_default_json,
                arm_joint_speed_radps=arm_joint_speed_radps,
                waist_joint_speed_radps=waist_joint_speed_radps,
                arm_settle_s=arm_settle_s,
                waist_settle_s=waist_settle_s,
                offset_settle_s=offset_settle_s,
                grab_final_stop_mm=grab_final_stop_mm,
                grab_final_brake_margin_mm=grab_final_brake_margin_mm,
                grab_final_speed_mps=grab_final_speed_mps,
                place_final_stop_mm=place_final_stop_mm,
                place_final_brake_margin_mm=place_final_brake_margin_mm,
                place_final_speed_mps=place_final_speed_mps,
                fine_position_max_duration_s=fine_position_max_duration_s,
                pick_down_z_m=pick_down_z_m,
                pick_back_x_m=pick_back_x_m,
                pick_back_down_x_m=pick_back_down_x_m,
                place_pull_x_m=place_pull_x_m,
                place_pull_back_down_x_m=place_pull_back_down_x_m,
                place_pull_back_down_z_m=place_pull_back_down_z_m,
                place_pull_drop_after_x_m=place_pull_drop_after_x_m,
                place_pull_drop_z_m=place_pull_drop_z_m,
                place_down_z_m=place_down_z_m,
                place_forward_after_fine_m=place_forward_after_fine_m,
                place_final_before_open_x_m=place_final_before_open_x_m,
                place_final_before_open_z_m=place_final_before_open_z_m,
                place_raise_before_open_z_m=place_raise_before_open_z_m,
                place_lateral_right_m=place_lateral_right_m,
                place_use_grab_pose=place_use_grab_pose,
                place_mirror_grab_waypoints=place_mirror_grab_waypoints,
                place_grab_z_offset_m=place_grab_z_offset_m,
                chassis_relative_max_abs_m=chassis_relative_max_abs_m,
                chassis_relative_timeout_s=chassis_relative_timeout_s,
                offset_max_abs_m=offset_max_abs_m,
                local_retreat_m=local_retreat_m,
                local_retreat_speed_mps=local_retreat_speed_mps,
                skip_local_retreat=skip_local_retreat,
                skip_pick_down_after_close=skip_pick_down_after_close,
                skip_pick_offsets_after_close=skip_pick_offsets_after_close,
                skip_waist_home_after_pick=skip_waist_home_after_pick,
                use_place_waypoint_jsons=use_place_waypoint_jsons,
                use_place_pose_json=use_place_pose_json,
                skip_place_pose_after_transition2=skip_place_pose_after_transition2,
                skip_place_pull_out_after_open=skip_place_pull_out_after_open,
                start_at_local_step=start_at_local_step,
                stop_after_local_step=stop_after_local_step,
            )
            if local_result.get("stopped_after_step"):
                save_state(checkpoint, state)
                print(
                    json.dumps(
                        {
                            "event": "mission_phase_paused",
                            "state": asdict(state),
                            "stopped_after_step": local_result["stopped_after_step"],
                            "note": "checkpoint was not advanced; resume or recapture before continuing LOCAL_PLACE",
                        },
                        ensure_ascii=False,
                    )
                )
                return state
        else:
            raise RuntimeError(f"unsupported local action mode: {local_action_mode}")
        state = advance_state(state, direct_home_after_place=direct_home_after_place)
        save_state(checkpoint, state)
        print(json.dumps({"event": "mission_phase_done", "state": asdict(state)}, ensure_ascii=False))
        return state

    if state.phase == "ROD_DONE":
        if not staging:
            raise RuntimeError("ROD_DONE checkpoint advancement requires --staging until real local actions exist")
        state = advance_state(state, direct_home_after_place=direct_home_after_place)
        save_state(checkpoint, state)
        print(json.dumps({"event": "mission_phase_done", "state": asdict(state)}, ensure_ascii=False))
        return state

    raise RuntimeError(f"unknown mission phase: {state.phase}")


def main() -> int:
    parser = argparse.ArgumentParser(description="G2 industrial cell mission-state skeleton")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--checkpoint-file", default=str(DEFAULT_CHECKPOINT))
    parser.add_argument("--start-index", type=int, default=1)
    parser.add_argument("--end-index", type=int, default=7)
    parser.add_argument("--init", action="store_true", help="Create a fresh checkpoint")
    parser.add_argument("--status", action="store_true", help="Print checkpoint and next action")
    parser.add_argument("--advance-dry-run", action="store_true", help="Advance one phase without robot motion")
    parser.add_argument("--execute-next", action="store_true", help="Execute the current checkpoint phase")
    parser.add_argument("--run-current-rod", action="store_true", help="Execute phases until the current rod is done")
    parser.add_argument("--staging", action="store_true", help="Allow dry-run/noop checkpoint advancement")
    parser.add_argument(
        "--local-action-mode",
        choices=("disabled", "noop", "readonly", "full-dry-run", "full"),
        default="disabled",
        help=(
            "Local pick/place behavior. full-dry-run prints the complete local plan; "
            "full executes arm, gripper, fine-position, offset, and retreat steps."
        ),
    )
    parser.add_argument("--nav-log-dir", default=str(PACKAGE_DIR.parent / "logs"))
    parser.add_argument("--confirm-live", action="store_true", help="Allow live map navigation phases")
    parser.add_argument("--confirm-local-physical", action="store_true", help="Allow physical local pick/place actions")
    parser.add_argument("--allow-estop-pedal-fault", action="store_true")
    parser.add_argument("--rack-read-samples", type=int, default=6)
    parser.add_argument("--rack-read-interval-s", type=float, default=0.15)
    parser.add_argument(
        "--arm-gate-mode",
        choices=("disabled", "manifest", "dryrun"),
        default="disabled",
        help="No-motion arm/gripper gate; dryrun only executes an allowlisted child --dry-run",
    )
    parser.add_argument("--arm-gate-only", action="store_true", help="Run the arm gate without checkpoint changes")
    parser.add_argument(
        "--arm-gate-phase",
        choices=tuple(LOCAL_PHASES.keys()),
        default="LOCAL_PICK",
        help="Phase label for --arm-gate-only",
    )
    parser.add_argument("--arm-gate-log-dir", default=str(PACKAGE_DIR.parent / "logs"))
    parser.add_argument("--arm-dry-run-base-json", default=str(DEFAULT_ARM_DRY_RUN_BASE_JSON))
    parser.add_argument("--arm-dry-run-pitch-m", type=float, default=DEFAULT_ARM_DRY_RUN_PITCH_M)
    parser.add_argument("--arm-grab-pose-dir", default=str(DEFAULT_ARM_GRAB_POSE_DIR))
    parser.add_argument("--full-local-log-dir", default=str(PACKAGE_DIR.parent / "logs"))
    parser.add_argument("--place-waist-json", default=str(DEFAULT_PLACE_WAIST_JSON))
    parser.add_argument("--place-above-json", default=str(DEFAULT_PLACE_ABOVE_JSON))
    parser.add_argument("--place-transition-json", default=str(DEFAULT_PLACE_TRANSITION_JSON))
    parser.add_argument("--place-transition2-json", default=str(DEFAULT_PLACE_TRANSITION2_JSON))
    parser.add_argument("--place-pose-json", default=str(DEFAULT_PLACE_POSE_JSON))
    parser.add_argument("--arm-default-json", default=str(DEFAULT_ARM_DEFAULT_JSON))
    parser.add_argument("--arm-joint-speed-radps", type=float, default=0.12)
    parser.add_argument("--waist-joint-speed-radps", type=float, default=DEFAULT_WAIST_JOINT_SPEED_RADPS)
    parser.add_argument("--arm-settle-s", type=float, default=0.8)
    parser.add_argument("--waist-settle-s", type=float, default=0.8)
    parser.add_argument("--offset-settle-s", type=float, default=0.8)
    parser.add_argument("--grab-final-stop-mm", type=int, default=308)
    parser.add_argument("--grab-final-brake-margin-mm", type=int, default=20)
    parser.add_argument("--grab-final-speed-mps", type=float, default=0.08)
    parser.add_argument("--place-final-stop-mm", type=int, default=308)
    parser.add_argument("--place-final-brake-margin-mm", type=int, default=20)
    parser.add_argument("--place-final-speed-mps", type=float, default=0.15)
    parser.add_argument("--fine-position-max-duration-s", type=float, default=60.0)
    parser.add_argument("--pick-down-z-m", type=float, default=-0.02)
    parser.add_argument("--pick-back-x-m", type=float, default=-0.20)
    parser.add_argument(
        "--pick-back-down-x-m",
        type=float,
        default=DEFAULT_PICK_BACK_DOWN_X_M,
        help=(
            "When --skip-pick-down-after-close is used, move this much backward while "
            "applying --pick-down-z-m, then finish the remaining --pick-back-x-m with no extra Z."
        ),
    )
    parser.add_argument("--place-pull-x-m", type=float, default=DEFAULT_PLACE_PULL_X_M)
    parser.add_argument(
        "--place-pull-back-down-x-m",
        type=float,
        default=DEFAULT_PLACE_PULL_BACK_DOWN_X_M,
        help="First post-place pull-out X offset while lowering both end effectors. Signed meters.",
    )
    parser.add_argument(
        "--place-pull-back-down-z-m",
        type=float,
        default=DEFAULT_PLACE_PULL_BACK_DOWN_Z_M,
        help="First post-place pull-out Z offset paired with --place-pull-back-down-x-m. Signed meters.",
    )
    parser.add_argument(
        "--place-pull-drop-after-x-m",
        type=float,
        default=DEFAULT_PLACE_PULL_DROP_AFTER_X_M,
        help="Cumulative post-place pull-out X offset where the vertical drop step is inserted. Signed meters.",
    )
    parser.add_argument(
        "--place-pull-drop-z-m",
        type=float,
        default=DEFAULT_PLACE_PULL_DROP_Z_M,
        help="Vertical Z offset inserted after reaching --place-pull-drop-after-x-m. Signed meters.",
    )
    parser.add_argument("--place-down-z-m", type=float, default=-0.06)
    parser.add_argument(
        "--place-forward-after-fine-m",
        type=float,
        default=DEFAULT_PLACE_FORWARD_AFTER_FINE_M,
        help=(
            "After place fine-position, move the chassis forward by this body-frame "
            "distance before moving arms to the place transition/final pose. Set 0 to disable."
        ),
    )
    parser.add_argument(
        "--place-lateral-right-m",
        type=float,
        default=DEFAULT_PLACE_LATERAL_RIGHT_M,
        help=(
            "After place fine-position, move the chassis right by this body-frame distance "
            "before lowering to the place pose. Set 0 to disable."
        ),
    )
    parser.add_argument(
        "--place-raise-before-open-z-m",
        type=float,
        default=0.0,
        help=(
            "After the final place arm pose and before opening grippers, raise both end "
            "effectors by this Z offset. Positive is up; set 0 to disable."
        ),
    )
    parser.add_argument(
        "--place-final-before-open-x-m",
        type=float,
        default=DEFAULT_PLACE_FINAL_BEFORE_OPEN_X_M,
        help=(
            "After the final calibrated place transition and before opening grippers, "
            "move both end effectors by this X offset. Positive is forward; set 0 to disable."
        ),
    )
    parser.add_argument(
        "--place-final-before-open-z-m",
        type=float,
        default=DEFAULT_PLACE_FINAL_BEFORE_OPEN_Z_M,
        help=(
            "After the final calibrated place transition and before opening grippers, "
            "move both end effectors by this Z offset. Positive is up; set 0 to disable."
        ),
    )
    parser.add_argument(
        "--place-use-grab-pose",
        action="store_true",
        help=(
            "After place fine-position, use the current rod's captured grab arm pose as the "
            "place pose base, then apply --place-grab-z-offset-m to both end effectors."
        ),
    )
    parser.add_argument(
        "--place-mirror-grab-waypoints",
        action="store_true",
        help=(
            "For LOCAL_PLACE, use captured grab pose JSONs as whole-body place waypoints: "
            "above=max(1,N-2), transition=max(1,N-1), final=N. Each waypoint moves waist and arms."
        ),
    )
    parser.add_argument(
        "--disable-place-waypoint-jsons",
        action="store_true",
        help=(
            "Use the legacy LOCAL_PLACE flow instead of the calibrated sequence: "
            "waist_place_straight, arm_place_above, fine-position, "
            "arm_place_transition, arm_place_pose."
        ),
    )
    parser.add_argument(
        "--skip-place-pose-after-transition2",
        action="store_true",
        help=(
            "For waypoint LOCAL_PLACE, open grippers after arm_place_transition2 "
            "instead of moving to the final arm_place_pose."
        ),
    )
    parser.add_argument(
        "--place-grab-z-offset-m",
        type=float,
        default=DEFAULT_PLACE_GRAB_Z_OFFSET_M,
        help=(
            "Z offset applied to both end effectors after --place-use-grab-pose. "
            "Positive is up; negative is down."
        ),
    )
    parser.add_argument("--chassis-relative-max-abs-m", type=float, default=DEFAULT_CHASSIS_RELATIVE_MAX_ABS_M)
    parser.add_argument("--chassis-relative-timeout-s", type=float, default=DEFAULT_CHASSIS_RELATIVE_TIMEOUT_S)
    parser.add_argument(
        "--use-place-pose-json",
        action="store_true",
        help="After place fine-position, move arms to --place-pose-json instead of applying --place-down-z-m.",
    )
    parser.add_argument(
        "--skip-place-pull-out-after-open",
        action="store_true",
        help="After opening grippers in LOCAL_PLACE, skip the arm pull-out offset and retreat chassis first.",
    )
    parser.add_argument("--offset-max-abs-m", type=float, default=0.25)
    parser.add_argument("--local-retreat-m", type=float, default=0.45)
    parser.add_argument("--local-retreat-speed-mps", type=float, default=0.20)
    parser.add_argument("--skip-local-retreat", action="store_true")
    parser.add_argument(
        "--skip-pick-offsets-after-close",
        action="store_true",
        help="After closing the grippers in LOCAL_PICK, skip the pick-down and arm-back offsets and go directly to local retreat.",
    )
    parser.add_argument(
        "--skip-pick-down-after-close",
        action="store_true",
        help=(
            "After closing the grippers in LOCAL_PICK, skip the separate pick-down offset; "
            "combine --pick-down-z-m into the arm-back offset so back and down move together."
        ),
    )
    parser.add_argument(
        "--skip-waist-home-after-pick",
        action="store_true",
        help="Skip the waist/body return-to-home step after pick retreat; useful when the next map navigation can start smoothly from the captured rod waist pose.",
    )
    parser.add_argument(
        "--stop-after-local-step",
        default="",
        help="For full/full-dry-run local actions, stop after this local step label without advancing the checkpoint.",
    )
    parser.add_argument(
        "--start-at-local-step",
        default="",
        help="For full/full-dry-run local actions, start execution at this local step label when it exists in the current phase plan.",
    )
    parser.add_argument("--refine-yaw", action="store_true", help="Enable yaw refinement for live navigation phases")
    parser.add_argument("--refine-yaw-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--refine-yaw-max-error-deg", type=float, default=10.0)
    parser.add_argument("--refine-yaw-angular-speed-radps", type=float, default=0.05)
    parser.add_argument("--refine-yaw-fine-angular-speed-radps", type=float, default=0.02)
    parser.add_argument("--refine-yaw-timeout-s", type=float, default=12.0)
    parser.add_argument(
        "--refine-yaw-stations",
        default=",".join(DEFAULT_REFINE_YAW_STATIONS),
        help="Comma-separated station names where --refine-yaw is applied; use 'none' to disable all refinements.",
    )
    parser.add_argument(
        "--direct-home-after-place",
        action="store_true",
        help="Experimental: after LOCAL_PLACE, skip NAV_TO_RECOVERY and advance directly to NAV_TO_HOME.",
    )
    args = parser.parse_args()

    if args.execute_next and args.run_current_rod:
        raise SystemExit("--execute-next and --run-current-rod cannot be used together")
    if args.refine_yaw and not args.confirm_live:
        raise SystemExit("--refine-yaw requires --confirm-live")
    try:
        refine_yaw_stations = parse_station_list(args.refine_yaw_stations)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    if args.local_action_mode in ("noop", "readonly", "full-dry-run", "full") and not args.staging:
        raise SystemExit("--local-action-mode noop/readonly/full-dry-run/full requires --staging")
    if args.local_action_mode == "full" and FULL_LOCAL_SAFETY_LOCK.exists():
        raise SystemExit(
            f"--local-action-mode full blocked by safety lock: {FULL_LOCAL_SAFETY_LOCK}. "
            "Remove this lock only after the rack is reset, inspected, and the live plan is re-approved."
        )
    if args.local_action_mode == "full" and not args.confirm_live:
        raise SystemExit("--local-action-mode full requires --confirm-live")
    if args.local_action_mode == "full" and not args.confirm_local_physical:
        raise SystemExit("--local-action-mode full requires --confirm-local-physical")
    if args.stop_after_local_step and args.local_action_mode not in ("full-dry-run", "full"):
        raise SystemExit("--stop-after-local-step requires --local-action-mode full-dry-run or full")
    if args.start_at_local_step and args.local_action_mode not in ("full-dry-run", "full"):
        raise SystemExit("--start-at-local-step requires --local-action-mode full-dry-run or full")
    if args.rack_read_samples <= 0:
        raise SystemExit("--rack-read-samples must be positive")
    if args.rack_read_interval_s < 0.0:
        raise SystemExit("--rack-read-interval-s must be >= 0")
    if args.arm_gate_mode != "disabled" and not (args.staging or args.arm_gate_only):
        raise SystemExit("--arm-gate-mode manifest/dryrun requires --staging or --arm-gate-only")
    if args.arm_gate_only and args.arm_gate_mode == "disabled":
        raise SystemExit("--arm-gate-only requires --arm-gate-mode manifest or dryrun")
    if abs(args.arm_dry_run_pitch_m) > 0.20:
        raise SystemExit("--arm-dry-run-pitch-m is capped at +/-0.20m per layer")
    if args.arm_joint_speed_radps <= 0.0:
        raise SystemExit("--arm-joint-speed-radps must be positive")
    if args.arm_joint_speed_radps > 0.5:
        raise SystemExit("--arm-joint-speed-radps is capped at 0.5")
    if args.waist_joint_speed_radps <= 0.0:
        raise SystemExit("--waist-joint-speed-radps must be positive")
    if args.waist_joint_speed_radps > 0.8:
        raise SystemExit("--waist-joint-speed-radps is capped at 0.8")
    if args.arm_settle_s < 0.0:
        raise SystemExit("--arm-settle-s must be >= 0")
    if args.waist_settle_s < 0.0:
        raise SystemExit("--waist-settle-s must be >= 0")
    if args.offset_settle_s < 0.0:
        raise SystemExit("--offset-settle-s must be >= 0")
    if args.grab_final_stop_mm <= 0 or args.place_final_stop_mm <= 0:
        raise SystemExit("--grab-final-stop-mm and --place-final-stop-mm must be positive")
    if args.grab_final_brake_margin_mm < 0 or args.place_final_brake_margin_mm < 0:
        raise SystemExit("--grab-final-brake-margin-mm and --place-final-brake-margin-mm must be >= 0")
    if args.grab_final_speed_mps <= 0.0 or args.place_final_speed_mps <= 0.0:
        raise SystemExit("--grab-final-speed-mps and --place-final-speed-mps must be positive")
    if args.fine_position_max_duration_s <= 0.0:
        raise SystemExit("--fine-position-max-duration-s must be positive")
    if args.offset_max_abs_m <= 0.0:
        raise SystemExit("--offset-max-abs-m must be positive")
    if abs(args.pick_down_z_m) > args.offset_max_abs_m:
        raise SystemExit("--pick-down-z-m exceeds --offset-max-abs-m")
    if abs(args.pick_back_x_m) > args.offset_max_abs_m:
        raise SystemExit("--pick-back-x-m exceeds --offset-max-abs-m")
    if abs(args.pick_back_down_x_m) > args.offset_max_abs_m:
        raise SystemExit("--pick-back-down-x-m exceeds --offset-max-abs-m")
    if args.skip_pick_down_after_close and not args.skip_pick_offsets_after_close:
        if args.pick_back_x_m == 0.0:
            raise SystemExit("--pick-back-x-m must be non-zero when split pick back/down is enabled")
        if args.pick_back_down_x_m == 0.0:
            raise SystemExit("--pick-back-down-x-m must be non-zero when split pick back/down is enabled")
        if args.pick_back_x_m * args.pick_back_down_x_m < 0.0:
            raise SystemExit("--pick-back-down-x-m must move in the same direction as --pick-back-x-m")
        if abs(args.pick_back_down_x_m) > abs(args.pick_back_x_m):
            raise SystemExit("--pick-back-down-x-m cannot exceed total --pick-back-x-m")
        if abs(args.pick_back_x_m - args.pick_back_down_x_m) > args.offset_max_abs_m:
            raise SystemExit("remaining pick-back offset exceeds --offset-max-abs-m")
    if abs(args.place_pull_x_m) > args.offset_max_abs_m:
        raise SystemExit("--place-pull-x-m exceeds --offset-max-abs-m")
    if abs(args.place_pull_back_down_x_m) > args.offset_max_abs_m:
        raise SystemExit("--place-pull-back-down-x-m exceeds --offset-max-abs-m")
    if abs(args.place_pull_back_down_z_m) > args.offset_max_abs_m:
        raise SystemExit("--place-pull-back-down-z-m exceeds --offset-max-abs-m")
    if abs(args.place_pull_drop_after_x_m) > args.offset_max_abs_m:
        raise SystemExit("--place-pull-drop-after-x-m exceeds --offset-max-abs-m")
    if abs(args.place_pull_drop_z_m) > args.offset_max_abs_m:
        raise SystemExit("--place-pull-drop-z-m exceeds --offset-max-abs-m")
    if args.place_pull_x_m == 0.0 and any(
        abs(value) > 1e-6
        for value in (
            args.place_pull_back_down_x_m,
            args.place_pull_back_down_z_m,
            args.place_pull_drop_after_x_m,
            args.place_pull_drop_z_m,
        )
    ):
        raise SystemExit("segmented place pull-out requires non-zero --place-pull-x-m")
    if not same_direction_or_zero(args.place_pull_x_m, args.place_pull_back_down_x_m):
        raise SystemExit("--place-pull-back-down-x-m must move in the same direction as --place-pull-x-m")
    if not same_direction_or_zero(args.place_pull_x_m, args.place_pull_drop_after_x_m):
        raise SystemExit("--place-pull-drop-after-x-m must move in the same direction as --place-pull-x-m")
    if abs(args.place_pull_back_down_x_m) > abs(args.place_pull_x_m):
        raise SystemExit("--place-pull-back-down-x-m cannot exceed total --place-pull-x-m")
    if abs(args.place_pull_drop_after_x_m) > abs(args.place_pull_x_m):
        raise SystemExit("--place-pull-drop-after-x-m cannot exceed total --place-pull-x-m")
    if args.place_pull_drop_after_x_m and (
        abs(args.place_pull_drop_after_x_m) + 1e-9 < abs(args.place_pull_back_down_x_m)
    ):
        raise SystemExit("--place-pull-drop-after-x-m cannot be before --place-pull-back-down-x-m")
    if abs(args.place_down_z_m) > args.offset_max_abs_m:
        raise SystemExit("--place-down-z-m exceeds --offset-max-abs-m")
    if abs(args.place_final_before_open_x_m) > args.offset_max_abs_m:
        raise SystemExit("--place-final-before-open-x-m exceeds --offset-max-abs-m")
    if abs(args.place_final_before_open_z_m) > args.offset_max_abs_m:
        raise SystemExit("--place-final-before-open-z-m exceeds --offset-max-abs-m")
    if abs(args.place_raise_before_open_z_m) > args.offset_max_abs_m:
        raise SystemExit("--place-raise-before-open-z-m exceeds --offset-max-abs-m")
    if args.place_forward_after_fine_m < 0.0:
        raise SystemExit("--place-forward-after-fine-m must be >= 0")
    if args.place_lateral_right_m < 0.0:
        raise SystemExit("--place-lateral-right-m must be >= 0")
    if args.chassis_relative_max_abs_m <= 0.0:
        raise SystemExit("--chassis-relative-max-abs-m must be positive")
    if args.place_forward_after_fine_m > args.chassis_relative_max_abs_m:
        raise SystemExit("--place-forward-after-fine-m exceeds --chassis-relative-max-abs-m")
    if args.place_lateral_right_m > args.chassis_relative_max_abs_m:
        raise SystemExit("--place-lateral-right-m exceeds --chassis-relative-max-abs-m")
    if args.place_use_grab_pose and args.use_place_pose_json:
        raise SystemExit("--place-use-grab-pose and --use-place-pose-json are mutually exclusive")
    if args.place_mirror_grab_waypoints and args.place_use_grab_pose:
        raise SystemExit("--place-mirror-grab-waypoints and --place-use-grab-pose are mutually exclusive")
    if args.place_mirror_grab_waypoints and args.use_place_pose_json:
        raise SystemExit("--place-mirror-grab-waypoints and --use-place-pose-json are mutually exclusive")
    if abs(args.place_grab_z_offset_m) > args.offset_max_abs_m:
        raise SystemExit("--place-grab-z-offset-m exceeds --offset-max-abs-m")
    if args.chassis_relative_timeout_s <= 0.0:
        raise SystemExit("--chassis-relative-timeout-s must be positive")
    if not args.skip_local_retreat and (args.local_retreat_m <= 0.0 or args.local_retreat_speed_mps <= 0.0):
        raise SystemExit("--local-retreat-m and --local-retreat-speed-mps must be positive")
    if args.confirm_live and not args.staging:
        raise SystemExit("--confirm-live mission execution is limited to --staging")
    if not (1 <= args.start_index <= 7 and 1 <= args.end_index <= 7):
        raise SystemExit("start/end index must be in 1..7")
    if args.start_index > args.end_index:
        raise SystemExit("--start-index must be <= --end-index")

    checkpoint = Path(args.checkpoint_file).resolve()
    config = Path(args.config).resolve()
    if not config.exists():
        raise SystemExit(f"missing config: {config}")
    arm_gate_log_dir = Path(args.arm_gate_log_dir).resolve()
    arm_dry_run_base_json = Path(args.arm_dry_run_base_json).resolve()
    arm_grab_pose_dir = Path(args.arm_grab_pose_dir).resolve()
    full_local_log_dir = Path(args.full_local_log_dir).resolve()
    place_waist_json = Path(args.place_waist_json).resolve()
    place_above_json = Path(args.place_above_json).resolve()
    place_transition_json = Path(args.place_transition_json).resolve()
    place_transition2_json = Path(args.place_transition2_json).resolve()
    place_pose_json = Path(args.place_pose_json).resolve()
    arm_default_json = Path(args.arm_default_json).resolve()
    use_place_waypoint_jsons = not args.disable_place_waypoint_jsons
    if args.local_action_mode in ("full-dry-run", "full"):
        required_paths = [
            ("--arm-grab-pose-dir", arm_grab_pose_dir),
            ("--arm-default-json", arm_default_json),
        ]
        if use_place_waypoint_jsons and not args.place_use_grab_pose and not args.place_mirror_grab_waypoints:
            required_paths.extend(
                [
                    ("--place-waist-json", place_waist_json),
                    ("--place-above-json", place_above_json),
                    ("--place-transition-json", place_transition_json),
                    ("--place-transition2-json", place_transition2_json),
                ]
            )
            if not args.skip_place_pose_after_transition2:
                required_paths.append(("--place-pose-json", place_pose_json))
        elif not args.place_use_grab_pose and not args.place_mirror_grab_waypoints:
            required_paths.append(("--place-above-json", place_above_json))
        if args.use_place_pose_json:
            required_paths.append(("--place-pose-json", place_pose_json))
        for label, path in required_paths:
            if not path.exists():
                raise SystemExit(f"{label} path missing: {path}")

    if args.arm_gate_only:
        run_arm_gate(
            mode=args.arm_gate_mode,
            phase=args.arm_gate_phase,
            rod_index=args.start_index,
            project_root=PROJECT_ROOT,
            dry_run_base_json=arm_dry_run_base_json,
            dry_run_pitch_m=args.arm_dry_run_pitch_m,
            log_dir=arm_gate_log_dir,
        )
        return 0

    if args.init or not checkpoint.exists():
        state = initial_state(args.start_index, args.end_index)
        save_state(checkpoint, state)
    else:
        state = load_state(checkpoint)

    if args.advance_dry_run:
        state = advance_state(state, direct_home_after_place=args.direct_home_after_place)
        save_state(checkpoint, state)

    if args.execute_next:
        state = execute_current_phase(
            state=state,
            checkpoint=checkpoint,
            config=config,
            staging=args.staging,
            confirm_live=args.confirm_live,
            local_action_mode=args.local_action_mode,
            nav_log_dir=Path(args.nav_log_dir).resolve(),
            refine_yaw=args.refine_yaw,
            refine_yaw_stations=refine_yaw_stations,
            refine_yaw_tolerance_deg=args.refine_yaw_tolerance_deg,
            refine_yaw_max_error_deg=args.refine_yaw_max_error_deg,
            refine_yaw_angular_speed_radps=args.refine_yaw_angular_speed_radps,
            refine_yaw_fine_angular_speed_radps=args.refine_yaw_fine_angular_speed_radps,
            refine_yaw_timeout_s=args.refine_yaw_timeout_s,
            allow_estop_pedal_fault=args.allow_estop_pedal_fault,
            rack_read_samples=args.rack_read_samples,
            rack_read_interval_s=args.rack_read_interval_s,
            arm_gate_mode=args.arm_gate_mode,
            arm_dry_run_base_json=arm_dry_run_base_json,
            arm_dry_run_pitch_m=args.arm_dry_run_pitch_m,
            arm_grab_pose_dir=arm_grab_pose_dir,
            arm_gate_log_dir=arm_gate_log_dir,
            confirm_local_physical=args.confirm_local_physical,
            full_local_log_dir=full_local_log_dir,
            place_waist_json=place_waist_json,
            place_above_json=place_above_json,
            place_transition_json=place_transition_json,
            place_transition2_json=place_transition2_json,
            place_pose_json=place_pose_json,
            arm_default_json=arm_default_json,
            arm_joint_speed_radps=args.arm_joint_speed_radps,
            waist_joint_speed_radps=args.waist_joint_speed_radps,
            arm_settle_s=args.arm_settle_s,
            waist_settle_s=args.waist_settle_s,
            offset_settle_s=args.offset_settle_s,
            grab_final_stop_mm=args.grab_final_stop_mm,
            grab_final_brake_margin_mm=args.grab_final_brake_margin_mm,
            grab_final_speed_mps=args.grab_final_speed_mps,
            place_final_stop_mm=args.place_final_stop_mm,
            place_final_brake_margin_mm=args.place_final_brake_margin_mm,
            place_final_speed_mps=args.place_final_speed_mps,
            fine_position_max_duration_s=args.fine_position_max_duration_s,
            pick_down_z_m=args.pick_down_z_m,
            pick_back_x_m=args.pick_back_x_m,
            pick_back_down_x_m=args.pick_back_down_x_m,
            place_pull_x_m=args.place_pull_x_m,
            place_pull_back_down_x_m=args.place_pull_back_down_x_m,
            place_pull_back_down_z_m=args.place_pull_back_down_z_m,
            place_pull_drop_after_x_m=args.place_pull_drop_after_x_m,
            place_pull_drop_z_m=args.place_pull_drop_z_m,
            place_down_z_m=args.place_down_z_m,
            place_forward_after_fine_m=args.place_forward_after_fine_m,
            place_final_before_open_x_m=args.place_final_before_open_x_m,
            place_final_before_open_z_m=args.place_final_before_open_z_m,
            place_raise_before_open_z_m=args.place_raise_before_open_z_m,
            place_lateral_right_m=args.place_lateral_right_m,
            place_use_grab_pose=args.place_use_grab_pose,
            place_mirror_grab_waypoints=args.place_mirror_grab_waypoints,
            place_grab_z_offset_m=args.place_grab_z_offset_m,
            chassis_relative_max_abs_m=args.chassis_relative_max_abs_m,
            chassis_relative_timeout_s=args.chassis_relative_timeout_s,
            offset_max_abs_m=args.offset_max_abs_m,
            local_retreat_m=args.local_retreat_m,
            local_retreat_speed_mps=args.local_retreat_speed_mps,
            skip_local_retreat=args.skip_local_retreat,
            skip_pick_down_after_close=args.skip_pick_down_after_close,
            skip_pick_offsets_after_close=args.skip_pick_offsets_after_close,
            skip_waist_home_after_pick=args.skip_waist_home_after_pick,
            use_place_waypoint_jsons=use_place_waypoint_jsons,
            use_place_pose_json=args.use_place_pose_json,
            skip_place_pose_after_transition2=args.skip_place_pose_after_transition2,
            skip_place_pull_out_after_open=args.skip_place_pull_out_after_open,
            start_at_local_step=args.start_at_local_step or None,
            stop_after_local_step=args.stop_after_local_step or None,
            direct_home_after_place=args.direct_home_after_place,
        )

    if args.run_current_rod:
        start_rod = state.rod_index
        while state.phase != "MISSION_DONE" and state.rod_index == start_rod:
            previous_progress = (
                state.rod_index,
                state.phase,
                state.holding_rod,
                state.current_station,
                state.last_success_step,
            )
            state = execute_current_phase(
                state=state,
                checkpoint=checkpoint,
                config=config,
                staging=args.staging,
                confirm_live=args.confirm_live,
                local_action_mode=args.local_action_mode,
                nav_log_dir=Path(args.nav_log_dir).resolve(),
                refine_yaw=args.refine_yaw,
                refine_yaw_stations=refine_yaw_stations,
                refine_yaw_tolerance_deg=args.refine_yaw_tolerance_deg,
                refine_yaw_max_error_deg=args.refine_yaw_max_error_deg,
                refine_yaw_angular_speed_radps=args.refine_yaw_angular_speed_radps,
                refine_yaw_fine_angular_speed_radps=args.refine_yaw_fine_angular_speed_radps,
                refine_yaw_timeout_s=args.refine_yaw_timeout_s,
                allow_estop_pedal_fault=args.allow_estop_pedal_fault,
                rack_read_samples=args.rack_read_samples,
                rack_read_interval_s=args.rack_read_interval_s,
                arm_gate_mode=args.arm_gate_mode,
                arm_dry_run_base_json=arm_dry_run_base_json,
                arm_dry_run_pitch_m=args.arm_dry_run_pitch_m,
                arm_grab_pose_dir=arm_grab_pose_dir,
                arm_gate_log_dir=arm_gate_log_dir,
                confirm_local_physical=args.confirm_local_physical,
                full_local_log_dir=full_local_log_dir,
                place_waist_json=place_waist_json,
                place_above_json=place_above_json,
                place_transition_json=place_transition_json,
                place_transition2_json=place_transition2_json,
                place_pose_json=place_pose_json,
                arm_default_json=arm_default_json,
                arm_joint_speed_radps=args.arm_joint_speed_radps,
                waist_joint_speed_radps=args.waist_joint_speed_radps,
                arm_settle_s=args.arm_settle_s,
                waist_settle_s=args.waist_settle_s,
                offset_settle_s=args.offset_settle_s,
                grab_final_stop_mm=args.grab_final_stop_mm,
                grab_final_brake_margin_mm=args.grab_final_brake_margin_mm,
                grab_final_speed_mps=args.grab_final_speed_mps,
                place_final_stop_mm=args.place_final_stop_mm,
                place_final_brake_margin_mm=args.place_final_brake_margin_mm,
                place_final_speed_mps=args.place_final_speed_mps,
                fine_position_max_duration_s=args.fine_position_max_duration_s,
                pick_down_z_m=args.pick_down_z_m,
                pick_back_x_m=args.pick_back_x_m,
                pick_back_down_x_m=args.pick_back_down_x_m,
                place_pull_x_m=args.place_pull_x_m,
                place_pull_back_down_x_m=args.place_pull_back_down_x_m,
                place_pull_back_down_z_m=args.place_pull_back_down_z_m,
                place_pull_drop_after_x_m=args.place_pull_drop_after_x_m,
                place_pull_drop_z_m=args.place_pull_drop_z_m,
                place_down_z_m=args.place_down_z_m,
                place_forward_after_fine_m=args.place_forward_after_fine_m,
                place_final_before_open_x_m=args.place_final_before_open_x_m,
                place_final_before_open_z_m=args.place_final_before_open_z_m,
                place_raise_before_open_z_m=args.place_raise_before_open_z_m,
                place_lateral_right_m=args.place_lateral_right_m,
                place_use_grab_pose=args.place_use_grab_pose,
                place_mirror_grab_waypoints=args.place_mirror_grab_waypoints,
                place_grab_z_offset_m=args.place_grab_z_offset_m,
                chassis_relative_max_abs_m=args.chassis_relative_max_abs_m,
                chassis_relative_timeout_s=args.chassis_relative_timeout_s,
                offset_max_abs_m=args.offset_max_abs_m,
                local_retreat_m=args.local_retreat_m,
                local_retreat_speed_mps=args.local_retreat_speed_mps,
                skip_local_retreat=args.skip_local_retreat,
                skip_pick_down_after_close=args.skip_pick_down_after_close,
                skip_pick_offsets_after_close=args.skip_pick_offsets_after_close,
                skip_waist_home_after_pick=args.skip_waist_home_after_pick,
                use_place_waypoint_jsons=use_place_waypoint_jsons,
                use_place_pose_json=args.use_place_pose_json,
                skip_place_pose_after_transition2=args.skip_place_pose_after_transition2,
                skip_place_pull_out_after_open=args.skip_place_pull_out_after_open,
                start_at_local_step=args.start_at_local_step or None,
                stop_after_local_step=args.stop_after_local_step or None,
                direct_home_after_place=args.direct_home_after_place,
            )
            current_progress = (
                state.rod_index,
                state.phase,
                state.holding_rod,
                state.current_station,
                state.last_success_step,
            )
            if current_progress == previous_progress:
                break

    output = {
        "checkpoint": str(checkpoint),
        "config": str(config),
        "state": asdict(state),
        "next_action": describe_action(state) if state.phase != "MISSION_DONE" else None,
    }
    print(json.dumps(output, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
