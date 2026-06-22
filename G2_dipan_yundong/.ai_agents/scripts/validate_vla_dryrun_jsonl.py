#!/usr/bin/env python3
"""Static validator for VLA dry-run JSONL fixtures.

The validator intentionally performs schema checks only. It does not import
robot SDKs, start controllers, read datasets, or inspect model artifacts.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from pathlib import Path
from typing import Any


EXPECTED_SCHEMA_VERSION = "vla_dryrun_v1"
REQUIRED_TOP_LEVEL_FIELDS = {
    "schema_version",
    "run_id",
    "event_id",
    "event_type",
    "ts_wall",
    "task",
    "observation",
    "world_model",
    "action_proposal",
    "safety_gate",
    "critic",
    "metrics",
    "artifact_refs",
}
ACTION_LENGTHS = {
    "left_abs_8d": 8,
    "dual_delta_14d": 14,
}
CRITIC_TAXONOMY = {
    "observation_missing",
    "action_invalid",
    "safety_gate_blocked",
    "task_phase_mismatch",
    "stale_plan",
    "contact_or_grasp_uncertain",
    "localization_unavailable",
    "controller_not_ready",
}


def is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def is_finite_number(value: Any) -> bool:
    return is_number(value) and math.isfinite(float(value))


def require_dict(value: Any, path: str, errors: list[str]) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        errors.append(f"{path} must be an object")
        return None
    return value


def require_list(value: Any, path: str, errors: list[str]) -> list[Any] | None:
    if not isinstance(value, list):
        errors.append(f"{path} must be an array")
        return None
    return value


def validate_action(event: dict[str, Any], line_no: int, errors: list[str], warnings: list[str]) -> str | None:
    action = require_dict(event.get("action_proposal"), f"line {line_no}.action_proposal", errors)
    if action is None:
        return None

    action_family = action.get("action_family")
    if action_family not in ACTION_LENGTHS:
        errors.append(f"line {line_no}.action_proposal.action_family is unsupported: {action_family!r}")
        return None

    vector = require_list(action.get("vector"), f"line {line_no}.action_proposal.vector", errors)
    if vector is not None:
        expected_len = ACTION_LENGTHS[action_family]
        if len(vector) != expected_len:
            errors.append(
                f"line {line_no}.action_proposal.vector length {len(vector)} does not match {action_family} length {expected_len}"
            )
        bad_indexes = [idx for idx, item in enumerate(vector) if not is_finite_number(item)]
        if bad_indexes:
            errors.append(f"line {line_no}.action_proposal.vector has non-finite/non-number values at indexes {bad_indexes}")

    shape = action.get("shape")
    if shape != ACTION_LENGTHS[action_family]:
        errors.append(f"line {line_no}.action_proposal.shape must be {ACTION_LENGTHS[action_family]} for {action_family}")

    arm_mask = require_list(action.get("arm_mask"), f"line {line_no}.action_proposal.arm_mask", errors)
    if arm_mask is not None:
        expected_mask = ["left"] if action_family == "left_abs_8d" else ["left", "right"]
        if arm_mask != expected_mask:
            warnings.append(f"line {line_no}.action_proposal.arm_mask is {arm_mask!r}, expected {expected_mask!r}")

    if not isinstance(action.get("sanitizer"), dict):
        errors.append(f"line {line_no}.action_proposal.sanitizer must be an object")

    task = require_dict(event.get("task"), f"line {line_no}.task", errors)
    if task is not None:
        expected_action_family = task.get("expected_action_family")
        if expected_action_family not in ACTION_LENGTHS and expected_action_family != "none":
            errors.append(f"line {line_no}.task.expected_action_family is unsupported: {expected_action_family!r}")
        if expected_action_family in ACTION_LENGTHS and expected_action_family != action_family:
            warnings.append(
                f"line {line_no} action_family {action_family} differs from task.expected_action_family {expected_action_family}"
            )

    return action_family


def validate_safety_gate(event: dict[str, Any], line_no: int, errors: list[str]) -> None:
    gate = require_dict(event.get("safety_gate"), f"line {line_no}.safety_gate", errors)
    if gate is None:
        return
    if not isinstance(gate.get("allowed"), bool):
        errors.append(f"line {line_no}.safety_gate.allowed must be boolean")
    reasons = require_list(gate.get("blocked_reasons"), f"line {line_no}.safety_gate.blocked_reasons", errors)
    if reasons is not None and not all(isinstance(item, str) for item in reasons):
        errors.append(f"line {line_no}.safety_gate.blocked_reasons must contain only strings")


def validate_critic(event: dict[str, Any], line_no: int, errors: list[str]) -> list[str]:
    critic = require_dict(event.get("critic"), f"line {line_no}.critic", errors)
    if critic is None:
        return []
    if not isinstance(critic.get("ok"), bool):
        errors.append(f"line {line_no}.critic.ok must be boolean")
    categories = require_list(critic.get("failure_categories"), f"line {line_no}.critic.failure_categories", errors)
    if categories is None:
        return []
    bad = [item for item in categories if item not in CRITIC_TAXONOMY]
    if bad:
        errors.append(f"line {line_no}.critic.failure_categories contains unsupported labels: {bad!r}")
    if critic.get("ok") is True and categories:
        errors.append(f"line {line_no}.critic.ok=true must not include failure_categories")
    if critic.get("ok") is False and not categories:
        errors.append(f"line {line_no}.critic.ok=false must include at least one failure category")
    return [item for item in categories if item in CRITIC_TAXONOMY]


def validate_artifact_refs(event: dict[str, Any], line_no: int, errors: list[str], warnings: list[str]) -> None:
    refs = require_list(event.get("artifact_refs"), f"line {line_no}.artifact_refs", errors)
    if refs is None:
        return
    if not refs:
        warnings.append(f"line {line_no}.artifact_refs is empty")
    for idx, ref in enumerate(refs):
        if not isinstance(ref, dict):
            errors.append(f"line {line_no}.artifact_refs[{idx}] must be an object")
            continue
        for key in ("kind", "uri"):
            if not isinstance(ref.get(key), str) or not ref.get(key):
                errors.append(f"line {line_no}.artifact_refs[{idx}].{key} must be a non-empty string")


def validate_event(
    event: dict[str, Any],
    line_no: int,
    seen_event_ids: set[str],
    errors: list[str],
    warnings: list[str],
) -> tuple[str | None, list[str]]:
    missing = sorted(REQUIRED_TOP_LEVEL_FIELDS - set(event))
    if missing:
        errors.append(f"line {line_no} missing required top-level fields: {missing}")

    if event.get("schema_version") != EXPECTED_SCHEMA_VERSION:
        errors.append(f"line {line_no}.schema_version must be {EXPECTED_SCHEMA_VERSION!r}")

    event_id = event.get("event_id")
    if not isinstance(event_id, str) or not event_id:
        errors.append(f"line {line_no}.event_id must be a non-empty string")
    elif event_id in seen_event_ids:
        errors.append(f"line {line_no}.event_id is duplicated: {event_id}")
    else:
        seen_event_ids.add(event_id)

    for field in ("run_id", "event_type", "ts_wall"):
        if not isinstance(event.get(field), str) or not event.get(field):
            errors.append(f"line {line_no}.{field} must be a non-empty string")

    require_dict(event.get("observation"), f"line {line_no}.observation", errors)
    require_dict(event.get("world_model"), f"line {line_no}.world_model", errors)
    require_dict(event.get("metrics"), f"line {line_no}.metrics", errors)

    action_family = validate_action(event, line_no, errors, warnings)
    validate_safety_gate(event, line_no, errors)
    categories = validate_critic(event, line_no, errors)
    validate_artifact_refs(event, line_no, errors, warnings)

    return action_family, categories


def load_and_validate(path: Path) -> tuple[dict[str, Any], list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []
    seen_event_ids: set[str] = set()
    action_family_counts: Counter[str] = Counter()
    critic_category_counts: Counter[str] = Counter()
    event_count = 0

    with path.open("r", encoding="utf-8") as handle:
        for line_no, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line:
                warnings.append(f"line {line_no} is blank and was skipped")
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError as exc:
                errors.append(f"line {line_no} is not valid JSON: {exc}")
                continue
            if not isinstance(event, dict):
                errors.append(f"line {line_no} must be a JSON object")
                continue
            event_count += 1
            action_family, categories = validate_event(event, line_no, seen_event_ids, errors, warnings)
            if action_family:
                action_family_counts[action_family] += 1
            critic_category_counts.update(categories)

    summary = {
        "status": "PASS" if not errors else "FAIL",
        "event_count": event_count,
        "action_family_counts": dict(sorted(action_family_counts.items())),
        "critic_category_counts": dict(sorted(critic_category_counts.items())),
        "warning_count": len(warnings),
        "error_count": len(errors),
    }
    return summary, errors, warnings


def write_report(report_path: Path, source_path: Path, summary: dict[str, Any], errors: list[str], warnings: list[str]) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# VLA Dry-Run JSONL Validation Report",
        "",
        f"Source: `{source_path}`",
        f"Status: **{summary['status']}**",
        "",
        "## Counts",
        "",
        f"- Events: {summary['event_count']}",
        f"- Errors: {summary['error_count']}",
        f"- Warnings: {summary['warning_count']}",
        "",
        "## Action Families",
        "",
    ]
    if summary["action_family_counts"]:
        for key, value in summary["action_family_counts"].items():
            lines.append(f"- `{key}`: {value}")
    else:
        lines.append("- None")

    lines.extend(["", "## Critic Categories", ""])
    if summary["critic_category_counts"]:
        for key, value in summary["critic_category_counts"].items():
            lines.append(f"- `{key}`: {value}")
    else:
        lines.append("- None")

    lines.extend(["", "## Warnings", ""])
    if warnings:
        for warning in warnings:
            lines.append(f"- {warning}")
    else:
        lines.append("- None")

    lines.extend(["", "## Errors", ""])
    if errors:
        for error in errors:
            lines.append(f"- {error}")
    else:
        lines.append("- None")

    lines.extend(
        [
            "",
            "## Validation Scope",
            "",
            "This report is generated by a static JSONL validator only. It does not run robot motion scripts, ROS, drivers, controllers, datasets, checkpoints, rosbags, or secret access.",
        ]
    )
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate a VLA dry-run JSONL fixture.")
    parser.add_argument("jsonl_path", type=Path)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    summary, errors, warnings = load_and_validate(args.jsonl_path)
    write_report(args.report, args.jsonl_path, summary, errors, warnings)

    print(json.dumps(summary, sort_keys=True))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
