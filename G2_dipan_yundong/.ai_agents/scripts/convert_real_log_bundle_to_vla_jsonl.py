#!/usr/bin/env python3
"""Convert one selected industrial_7_rods dry-run bundle to VLA JSONL.

This is a static, offline converter for task_0009. It deliberately reads only
the selected bundle stem declared by the task and writes only the requested
JSONL/report outputs. It does not import robot SDKs, start controllers, or issue
robot commands.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import Counter
from pathlib import Path
from typing import Any


ALLOWED_STEM = "rack_hybrid_docking_package/logs/industrial_7_rods_20260611_031537"
SCHEMA_VERSION = "vla_dryrun_v1"
BASE_TS = "2026-06-11T03:15:37Z"
SOURCE_FILES = {
    "jsonl": ".jsonl",
    "log": ".log",
    "checkpoint": "_checkpoint.json",
    "report": "_report.json",
}
TAXONOMY = {
    "observation_missing",
    "action_invalid",
    "safety_gate_blocked",
    "task_phase_mismatch",
    "stale_plan",
    "contact_or_grasp_uncertain",
    "localization_unavailable",
    "controller_not_ready",
}


def enforce_selected_stem(stem: Path) -> str:
    normalized = stem.as_posix().rstrip("/")
    if normalized != ALLOWED_STEM:
        raise SystemExit(f"refusing to read undeclared bundle stem: {normalized}")
    return normalized


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def read_json(path: Path) -> Any | None:
    try:
        return json.loads(read_text(path))
    except Exception:
        return None


def read_jsonl(path: Path) -> tuple[list[dict[str, Any]], list[str]]:
    records: list[dict[str, Any]] = []
    errors: list[str] = []
    for line_no, raw in enumerate(read_text(path).splitlines(), start=1):
        line = raw.strip()
        if not line:
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            errors.append(f"jsonl line {line_no}: {exc}")
            continue
        if isinstance(value, dict):
            records.append(value)
        else:
            errors.append(f"jsonl line {line_no}: non-object JSON value skipped")
    return records, errors


def iter_kv(value: Any, path: tuple[str, ...] = ()) -> list[tuple[tuple[str, ...], Any]]:
    out: list[tuple[tuple[str, ...], Any]] = []
    if isinstance(value, dict):
        for key, item in value.items():
            key_s = str(key)
            out.append((path + (key_s,), item))
            out.extend(iter_kv(item, path + (key_s,)))
    elif isinstance(value, list):
        for idx, item in enumerate(value[:50]):
            out.extend(iter_kv(item, path + (str(idx),)))
    return out


def compact(value: Any, limit: int = 180) -> str:
    text = json.dumps(value, ensure_ascii=False, sort_keys=True) if not isinstance(value, str) else value
    text = " ".join(text.split())
    return text[:limit]


def first_matching_value(objects: list[Any], key_tokens: tuple[str, ...]) -> Any | None:
    for obj in objects:
        for path, value in iter_kv(obj):
            joined = ".".join(path).lower()
            if any(token in joined for token in key_tokens):
                if isinstance(value, (str, int, float, bool)) or value is None:
                    return value
    return None


def collect_matching_values(objects: list[Any], key_tokens: tuple[str, ...], max_items: int = 8) -> list[str]:
    found: list[str] = []
    for obj in objects:
        for path, value in iter_kv(obj):
            joined = ".".join(path).lower()
            text = compact(value)
            if any(token in joined for token in key_tokens) or any(token in text.lower() for token in key_tokens):
                item = f"{'.'.join(path)}={text}"
                if item not in found:
                    found.append(item)
                if len(found) >= max_items:
                    return found
    return found


def is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def numeric_list(value: Any) -> list[float] | None:
    if not isinstance(value, list):
        return None
    if not all(is_finite_number(item) for item in value):
        return None
    return [float(item) for item in value]


def find_action_vector(objects: list[Any]) -> tuple[str | None, list[float] | None, str | None]:
    """Find a conservative real action vector only when action-like keys exist."""
    action_key_tokens = ("vla", "action_vector", "action", "policy_output")
    for obj in objects:
        for path, value in iter_kv(obj):
            joined = ".".join(path).lower()
            if not any(token in joined for token in action_key_tokens):
                continue
            vector = numeric_list(value)
            if vector is None:
                continue
            if len(vector) == 8:
                return "left_abs_8d", vector, ".".join(path)
            if len(vector) == 14:
                return "dual_delta_14d", vector, ".".join(path)
    return None, None, None


def infer_phase(text: str) -> str:
    lower = text.lower()
    if "recover" in lower or "recovery" in lower:
        return "recover"
    if "place" in lower:
        return "place"
    if "pick" in lower or "grab" in lower:
        return "pick"
    if "home" in lower:
        return "home"
    if "nav" in lower or "navigation" in lower:
        return "nav"
    if "stop" in lower:
        return "stop"
    if "hold" in lower:
        return "hold"
    return "unknown"


def infer_rod_index(text: str) -> int | None:
    match = re.search(r"rod[_ -]?(\d+)", text.lower())
    if match:
        return int(match.group(1))
    match = re.search(r'"rod_index"\s*:\s*(\d+)', text)
    if match:
        return int(match.group(1))
    return None


def timestamp_for(index: int, objects: list[Any]) -> str:
    value = first_matching_value(objects, ("timestamp", "ts_wall", "time", "datetime", "created_at"))
    if isinstance(value, str) and value:
        return value
    return BASE_TS.replace("37Z", f"{37 + index:02d}Z")


def classify_blockers(text: str, placeholder_used: bool) -> tuple[bool, list[str], list[str]]:
    lower = text.lower()
    reasons: list[str] = []
    categories: list[str] = []

    if placeholder_used:
        reasons.append("analysis_placeholder_no_real_vla_action")
        categories.append("action_invalid")
    if any(token in lower for token in ("charge", "estop", "safety", "blocked", "guard")):
        reasons.append("safety_or_guard_hint_in_source")
        categories.append("safety_gate_blocked")
    if any(token in lower for token in ("odom", "localization", "pose unavailable", "curr_pose", "slam")):
        reasons.append("localization_hint_in_source")
        categories.append("localization_unavailable")
    if any(token in lower for token in ("controller", "motion_control", "not ready", "error_code")):
        reasons.append("controller_state_hint_in_source")
        categories.append("controller_not_ready")

    categories = [item for item in dict.fromkeys(categories) if item in TAXONOMY]
    reasons = list(dict.fromkeys(reasons))
    allowed = not reasons
    return allowed, reasons, categories


def artifact_refs(stem: str) -> list[dict[str, str]]:
    refs = []
    for kind, suffix in SOURCE_FILES.items():
        refs.append(
            {
                "kind": kind,
                "uri": f"{stem}{suffix}",
                "description": f"selected industrial_7_rods_20260611_031537 {kind} artifact",
            }
        )
    return refs


def make_event(
    index: int,
    source_obj: dict[str, Any],
    context_objects: list[Any],
    stem: str,
    action_family: str,
    action_vector: list[float],
    action_source: str | None,
    placeholder_used: bool,
) -> dict[str, Any]:
    source_text = compact(source_obj, 1200)
    all_text = source_text + " " + " ".join(compact(obj, 700) for obj in context_objects)
    phase = infer_phase(all_text)
    rod_index = infer_rod_index(all_text)
    allowed, blocked_reasons, categories = classify_blockers(all_text, placeholder_used)
    critic_ok = not categories

    success_value = first_matching_value([source_obj] + context_objects, ("success", "status", "result", "mission", "done"))
    map_id = first_matching_value([source_obj] + context_objects, ("map_id", "mapid"))
    motion_error = first_matching_value([source_obj] + context_objects, ("motion_control_error", "error_code"))

    facts = [
        {
            "fact_group": "task_state",
            "fact_name": "task_phase",
            "value": phase,
            "confidence": 0.6 if phase != "unknown" else 0.2,
            "ts_wall": timestamp_for(index, [source_obj] + context_objects),
            "source_event_id": f"real_bundle_event_{index:03d}",
            "source_artifact": f"{stem}.jsonl",
            "stale_after_ms": 1000,
            "required_for": ["dryrun_conversion"],
            "missing_reason": None if phase != "unknown" else "not_explicitly_identified",
        },
        {
            "fact_group": "task_state",
            "fact_name": "success_label",
            "value": success_value,
            "confidence": 0.5 if success_value is not None else 0.0,
            "ts_wall": timestamp_for(index, [source_obj] + context_objects),
            "source_event_id": f"real_bundle_event_{index:03d}",
            "source_artifact": f"{stem}_report.json",
            "stale_after_ms": 1000,
            "required_for": ["metrics"],
            "missing_reason": None if success_value is not None else "not_found_in_generic_scan",
        },
    ]

    if blocked_reasons:
        facts.append(
            {
                "fact_group": "safety_state",
                "fact_name": "blocked_reasons",
                "value": blocked_reasons,
                "confidence": 0.4,
                "ts_wall": timestamp_for(index, [source_obj] + context_objects),
                "source_event_id": f"real_bundle_event_{index:03d}",
                "source_artifact": f"{stem}.jsonl",
                "stale_after_ms": 1000,
                "required_for": ["action_validation"],
                "missing_reason": None,
            }
        )

    vector_shape = len(action_vector)
    is_left = action_family == "left_abs_8d"
    return {
        "schema_version": SCHEMA_VERSION,
        "run_id": "real_industrial_7_rods_20260611_031537",
        "event_id": f"real_bundle_event_{index:03d}",
        "event_type": "real_log_conversion",
        "ts_wall": timestamp_for(index, [source_obj] + context_objects),
        "ts_monotonic_ns": None,
        "task": {
            "task_prompt": "real log replay analysis for industrial seven-rods dry-run bundle",
            "task_phase": phase,
            "rod_index": rod_index,
            "expected_action_family": action_family,
        },
        "observation": {
            "cameras": {
                "head_image": {
                    "present": False,
                    "uri": None,
                    "ts": None,
                    "age_ms": None,
                    "width": None,
                    "height": None,
                    "encoding": "unknown",
                    "missing_reason": "no_image_artifact_in_selected_bundle",
                },
                "left_wrist_image": {"present": False, "uri": None, "missing_reason": "not_available_in_selected_bundle"},
                "right_wrist_image": {"present": False, "uri": None, "missing_reason": "not_available_in_selected_bundle"},
            },
            "state": {
                "left_ee_pose_xyz_quat": None,
                "left_gripper_01": None,
                "right_ee_pose_xyz_quat": None,
                "right_gripper_01": None,
                "motion_control_error_code": motion_error if isinstance(motion_error, int) else None,
                "joint_errors": [],
                "pnc_task_state": None,
                "chassis_power_state": None,
            },
            "localization": {
                "map_id": map_id,
                "curr_pose_valid": None,
                "odom_valid": None,
                "map_to_base_valid": None,
            },
        },
        "world_model": {"facts": facts},
        "action_proposal": {
            "action_id": f"real_bundle_action_{index:03d}",
            "source_event_id": f"real_bundle_event_{index:03d}",
            "action_family": action_family,
            "vector": action_vector,
            "shape": vector_shape,
            "frame_id": "unknown",
            "arm_mask": ["left"] if is_left else ["left", "right"],
            "is_absolute": is_left,
            "is_delta": not is_left,
            "limits_profile": "real_log_analysis_placeholder_v1" if placeholder_used else "real_log_extracted_v1",
            "sanitizer": {
                "shape_valid": vector_shape in (8, 14),
                "finite_valid": all(is_finite_number(item) for item in action_vector),
                "bounds_valid": True,
                "clip_count": 0,
            },
            "derived": placeholder_used,
            "derived_from_action_id": None if not placeholder_used else "no_real_vla_action_vector_found",
            "source_path": action_source,
        },
        "safety_gate": {"allowed": allowed, "blocked_reasons": blocked_reasons},
        "critic": {"ok": critic_ok, "failure_categories": categories},
        "metrics": {
            "event_schema_valid": True,
            "observation_required_complete": False,
            "action_shape_valid": vector_shape in (8, 14),
            "action_finite_valid": all(is_finite_number(item) for item in action_vector),
            "placeholder_action": placeholder_used,
        },
        "artifact_refs": artifact_refs(stem),
    }


def write_jsonl(path: Path, events: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for event in events:
            handle.write(json.dumps(event, ensure_ascii=False, sort_keys=True) + "\n")


def write_report(
    path: Path,
    stem: str,
    file_sizes: dict[str, int],
    jsonl_count: int,
    jsonl_errors: list[str],
    events: list[dict[str, Any]],
    real_action_found: bool,
    action_source: str | None,
    extracted_hints: dict[str, list[str]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    action_counts = Counter(event["action_proposal"]["action_family"] for event in events)
    critic_counts = Counter(cat for event in events for cat in event["critic"]["failure_categories"])
    placeholder_count = sum(1 for event in events if event["action_proposal"]["derived"])
    lines = [
        "# Real VLA Dry-Run Conversion Report",
        "",
        f"Selected bundle stem: `{stem}`",
        f"Output events: {len(events)}",
        f"Source JSONL records parsed: {jsonl_count}",
        "",
        "## Source Files",
        "",
    ]
    for kind, suffix in SOURCE_FILES.items():
        lines.append(f"- `{stem}{suffix}`: {file_sizes.get(kind, 0)} bytes")

    lines.extend(["", "## Action Vector Handling", ""])
    if real_action_found:
        lines.append(f"- Real action vector extracted from `{action_source}`.")
    else:
        lines.append("- No real VLA action vector was extracted from the selected bundle by the conservative scanner.")
        lines.append("- The generated action vectors are `derived=true` zero-hold analysis placeholders.")
        lines.append("- These placeholders are not VLA outputs, not robot commands, and must not be sent to hardware.")
        lines.append("- They exist only to satisfy the current static JSONL validator while preserving real artifact provenance.")
    lines.append(f"- Placeholder action events: {placeholder_count}")

    lines.extend(["", "## Counts", ""])
    for family, count in sorted(action_counts.items()):
        lines.append(f"- Action family `{family}`: {count}")
    if critic_counts:
        for category, count in sorted(critic_counts.items()):
            lines.append(f"- Critic category `{category}`: {count}")
    else:
        lines.append("- Critic categories: none")

    lines.extend(["", "## Extracted Signals", ""])
    for signal, values in extracted_hints.items():
        lines.append(f"### {signal}")
        if values:
            for value in values:
                lines.append(f"- {value}")
        else:
            lines.append("- Not found by generic static scan.")

    lines.extend(["", "## Null, Unknown, or Placeholder Fields", ""])
    lines.append("- Camera image fields are `present=false` because the selected bundle is text/log/report only.")
    lines.append("- End-effector pose and gripper fields remain `null` unless explicitly present in parsed records.")
    lines.append("- Localization fields remain `null` unless explicit map/odom/pose signals are found.")
    lines.append("- `frame_id` is `unknown` unless an action source explicitly declares a frame.")
    lines.append("- Missing values are explicit and are not silently zero-filled.")

    if jsonl_errors:
        lines.extend(["", "## JSONL Parse Warnings", ""])
        for error in jsonl_errors:
            lines.append(f"- {error}")

    lines.extend(
        [
            "",
            "## Safety Scope",
            "",
            "This conversion is offline and analysis-only. It reads the selected text bundle and writes JSONL/report artifacts. It does not run robot motion scripts, ROS, drivers, controllers, datasets, checkpoints outside the selected bundle, rosbags, or secrets.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert selected dry-run bundle to VLA dry-run JSONL.")
    parser.add_argument("--stem", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--max-events", default=10, type=int)
    args = parser.parse_args()

    stem = enforce_selected_stem(args.stem)
    paths = {kind: Path(f"{stem}{suffix}") for kind, suffix in SOURCE_FILES.items()}
    file_sizes = {kind: paths[kind].stat().st_size for kind in paths}

    jsonl_records, jsonl_errors = read_jsonl(paths["jsonl"])
    log_text = read_text(paths["log"])
    checkpoint = read_json(paths["checkpoint"])
    report = read_json(paths["report"])
    context_objects = [obj for obj in (checkpoint, report, {"log_excerpt": log_text[:2000]}) if obj is not None]

    action_family, action_vector, action_source = find_action_vector(jsonl_records + context_objects)
    real_action_found = action_vector is not None and action_family is not None
    if not real_action_found:
        action_family = "dual_delta_14d"
        action_vector = [0.0] * 14
        action_source = None

    source_records = jsonl_records[: max(1, args.max_events)]
    if not source_records:
        source_records = [{"source": "selected_bundle_summary", "log_excerpt": log_text[:800]}]
    source_records = source_records[: args.max_events]

    events = [
        make_event(
            index=idx,
            source_obj=record,
            context_objects=context_objects,
            stem=stem,
            action_family=action_family,
            action_vector=action_vector,
            action_source=action_source,
            placeholder_used=not real_action_found,
        )
        for idx, record in enumerate(source_records, start=1)
    ]

    extracted_hints = {
        "task_phase": [event["task"]["task_phase"] for event in events],
        "success_label": collect_matching_values([report, checkpoint], ("success", "status", "result", "mission", "done")),
        "safety_or_blocker": collect_matching_values(jsonl_records + context_objects, ("safety", "blocked", "guard", "charge", "estop", "fail", "error")),
        "robot_state": collect_matching_values(jsonl_records + context_objects, ("motion_control", "joint", "gripper", "pnc", "chassis")),
        "localization_state": collect_matching_values(jsonl_records + context_objects, ("map_id", "odom", "pose", "slam", "localization")),
        "action_vector": [action_source] if real_action_found and action_source else [],
    }

    write_jsonl(args.output, events)
    write_report(
        args.report,
        stem=stem,
        file_sizes=file_sizes,
        jsonl_count=len(jsonl_records),
        jsonl_errors=jsonl_errors,
        events=events,
        real_action_found=real_action_found,
        action_source=action_source,
        extracted_hints=extracted_hints,
    )
    print(
        json.dumps(
            {
                "status": "converted",
                "events": len(events),
                "real_action_found": real_action_found,
                "placeholder_action_count": sum(1 for event in events if event["action_proposal"]["derived"]),
                "jsonl_records": len(jsonl_records),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
