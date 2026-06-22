#!/usr/bin/env python3
"""Analyze active rack lateral-centering response from JSONL event logs."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import statistics
from typing import Any


def median(values: list[float]) -> float | None:
    return float(statistics.median(values)) if values else None


def round_or_none(value: float | None, digits: int = 4) -> float | None:
    return None if value is None else round(value, digits)


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_no}: invalid JSON: {exc}") from exc
            event["_source"] = str(path)
            rows.append(event)
    return rows


def as_tuple(value) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    return tuple(str(item) for item in value)


def event_key(event: dict[str, Any]) -> tuple[str, str, int]:
    return (
        str(event.get("run_id") or ""),
        str(event.get("label") or ""),
        int(event.get("pass_index") or 0),
    )


def simple_key(event: dict[str, Any]) -> tuple[str, str]:
    return str(event.get("run_id") or ""), str(event.get("label") or "")


def collect_summary(events: list[dict[str, Any]]) -> dict[str, Any]:
    starts = {}
    decisions = {}
    step_starts = {}
    step_results = {}
    rollback_starts = {}
    rollback_results = {}
    results_by_key = {}
    results_by_simple = {}
    pose_by_label = defaultdict(list)
    reason_counts = Counter()

    for event in events:
        name = event.get("event")
        if name == "rack_lateral_centering_start":
            starts[simple_key(event)] = event
        elif name == "rack_lateral_centering_decision":
            decisions[event_key(event)] = event
            reason_counts.update(as_tuple(event.get("reasons")))
        elif name == "rack_lateral_centering_step_start":
            step_starts[event_key(event)] = event
        elif name == "rack_lateral_centering_step_result":
            step_results[event_key(event)] = event
        elif name == "rack_lateral_centering_rollback_start":
            rollback_starts[event_key(event)] = event
        elif name == "rack_lateral_centering_rollback_result":
            rollback_results[event_key(event)] = event
        elif name == "rack_lateral_centering_result":
            results_by_key[event_key(event)] = event
            results_by_simple[simple_key(event)] = event
            reason_counts.update(as_tuple(event.get("reasons")))
        elif name in ("rack_lateral_centering_pose", "rack_pose_monitor"):
            pose_by_label[(str(event.get("run_id") or ""), str(event.get("label") or ""))].append(event)

    step_rows = []
    for key, step in sorted(step_results.items()):
        run_id, label, pass_index = key
        decision = decisions.get(key, {})
        start = starts.get((run_id, label), {})
        step_start = step_starts.get(key, {})
        rollback_start = rollback_starts.get(key, {})
        rollback = rollback_results.get(key, {})
        result = results_by_key.get(key, {})
        before = step.get("before_lateral_center_m")
        after = step.get("after_lateral_center_m")
        improvement = step.get("improvement_m")
        row = {
            "source": step.get("_source"),
            "run_id": run_id,
            "label": label,
            "pass_index": pass_index,
            "active_direction": start.get("direction") or decision.get("thresholds", {}).get("active_direction"),
            "target_lateral_m": step.get("target_lateral_m") or start.get("target_lateral_m"),
            "vy_mps": step_start.get("vy_mps") if step_start.get("vy_mps") is not None else decision.get("vy_mps"),
            "before_lateral_center_m": before,
            "after_lateral_center_m": after,
            "improvement_m": improvement,
            "before_abs_m": abs(float(before)) if before is not None else None,
            "after_abs_m": abs(float(after)) if after is not None else None,
            "lateral_sample_span_m": decision.get("lateral_sample_span_m"),
            "yaw_deg": decision.get("yaw_deg"),
            "fit_residual_m": decision.get("fit_residual_m"),
            "rollback_vy_mps": rollback_start.get("rollback_vy_mps"),
            "rollback_lateral_center_m": rollback.get("rollback_lateral_center_m"),
            "rollback_improvement_m": rollback.get("rollback_improvement_m"),
            "result_status": result.get("status"),
        }
        step_rows.append(row)

    blocked_rows = []
    for key, decision in sorted(decisions.items()):
        if decision.get("decision") != "blocked":
            continue
        run_id, label, pass_index = key
        start = starts.get((run_id, label), {})
        result = results_by_key.get(key) or results_by_simple.get((run_id, label), {})
        blocked_rows.append(
            {
                "source": decision.get("_source"),
                "run_id": run_id,
                "label": label,
                "pass_index": pass_index,
                "active_direction": start.get("direction") or decision.get("thresholds", {}).get("active_direction"),
                "lateral_center_m": decision.get("lateral_center_m"),
                "lateral_sample_span_m": decision.get("lateral_sample_span_m"),
                "fit_residual_m": decision.get("fit_residual_m"),
                "yaw_deg": decision.get("yaw_deg"),
                "reasons": as_tuple(decision.get("reasons")),
                "result_status": result.get("status"),
            }
        )

    improvements = [
        float(row["improvement_m"])
        for row in step_rows
        if row.get("improvement_m") is not None
    ]
    rollback_improvements = [
        float(row["rollback_improvement_m"])
        for row in step_rows
        if row.get("rollback_improvement_m") is not None
    ]
    by_vy = defaultdict(list)
    for row in step_rows:
        vy = row.get("vy_mps")
        improvement = row.get("improvement_m")
        if vy is not None and improvement is not None:
            by_vy["positive" if float(vy) > 0 else "negative"].append(float(improvement))

    by_vy_summary = {}
    for sign, values in sorted(by_vy.items()):
        by_vy_summary[sign] = {
            "count": len(values),
            "improvement_m_median": round_or_none(median(values)),
            "improvement_m_min": round_or_none(min(values)),
            "improvement_m_max": round_or_none(max(values)),
            "worse_count": sum(1 for value in values if value < 0.0),
            "improved_count": sum(1 for value in values if value > 0.0),
        }

    if step_rows and all((row.get("improvement_m") or 0.0) < 0.0 for row in step_rows):
        recommendation = "keep_active_lateral_motion_disabled"
        recommendation_reason = "all_executed_lateral_steps_worsened_pose"
    elif blocked_rows and not step_rows:
        recommendation = "keep_active_gate_blocking"
        recommendation_reason = "all_decisions_blocked_before_motion"
    elif any((row.get("improvement_m") or 0.0) > 0.0 for row in step_rows):
        recommendation = "review_positive_steps_before_any_enablement"
        recommendation_reason = "some_steps_improved_but_consistency_required"
    else:
        recommendation = "collect_more_probe_data"
        recommendation_reason = "insufficient_executed_steps"

    return {
        "files": sorted({event.get("_source") for event in events if event.get("_source")}),
        "event_count": len(events),
        "step_count": len(step_rows),
        "blocked_count": len(blocked_rows),
        "result_status_counts": dict(
            sorted(
                Counter(str(row.get("result_status") or "unknown") for row in step_rows + blocked_rows).items()
            )
        ),
        "reason_counts": dict(sorted(reason_counts.items())),
        "improvement_m_median": round_or_none(median(improvements)),
        "improvement_m_min": round_or_none(min(improvements) if improvements else None),
        "rollback_improvement_m_median": round_or_none(median(rollback_improvements)),
        "by_vy_sign": by_vy_summary,
        "recommendation": recommendation,
        "recommendation_reason": recommendation_reason,
        "steps": step_rows,
        "blocked": blocked_rows,
    }


def render_markdown(summary: dict[str, Any]) -> str:
    lines = [
        "# Lateral Active Response Analysis",
        "",
        f"- files: {len(summary['files'])}",
        f"- event_count: {summary['event_count']}",
        f"- step_count: {summary['step_count']}",
        f"- blocked_count: {summary['blocked_count']}",
        f"- improvement_m_median: {summary['improvement_m_median']}",
        f"- improvement_m_min: {summary['improvement_m_min']}",
        f"- rollback_improvement_m_median: {summary['rollback_improvement_m_median']}",
        f"- recommendation: `{summary['recommendation']}`",
        f"- recommendation_reason: `{summary['recommendation_reason']}`",
        "",
        "## By Vy Sign",
        "",
    ]
    if summary["by_vy_sign"]:
        for sign, item in summary["by_vy_sign"].items():
            parts = ", ".join(f"{key}={value}" for key, value in item.items())
            lines.append(f"- {sign}: {parts}")
    else:
        lines.append("- no executed lateral steps")

    if summary["reason_counts"]:
        lines.extend(["", "## Reasons", ""])
        for reason, count in summary["reason_counts"].items():
            lines.append(f"- {reason}: {count}")

    if summary["steps"]:
        lines.extend(["", "## Executed Steps", ""])
        lines.append(
            "| run_id | pass | direction | vy_mps | before | after | improvement | rollback_lateral | rollback_improvement | status |"
        )
        lines.append("|---|---:|---|---:|---:|---:|---:|---:|---:|---|")
        for row in summary["steps"]:
            lines.append(
                "| {run_id} | {pass_index} | {active_direction} | {vy_mps} | "
                "{before_lateral_center_m} | {after_lateral_center_m} | {improvement_m} | "
                "{rollback_lateral_center_m} | {rollback_improvement_m} | {result_status} |".format(
                    **{
                        key: round(value, 4) if isinstance(value, float) else value
                        for key, value in row.items()
                    }
                )
            )

    if summary["blocked"]:
        lines.extend(["", "## Blocked Decisions", ""])
        lines.append("| run_id | direction | lateral | sample_span | reasons | status |")
        lines.append("|---|---|---:|---:|---|---|")
        for row in summary["blocked"]:
            row = {
                key: round(value, 4) if isinstance(value, float) else value
                for key, value in row.items()
            }
            lines.append(
                f"| {row['run_id']} | {row['active_direction']} | {row['lateral_center_m']} | "
                f"{row['lateral_sample_span_m']} | {', '.join(row['reasons'])} | {row['result_status']} |"
            )

    lines.extend(["", "## Files", ""])
    for file_name in summary["files"]:
        lines.append(f"- `{file_name}`")
    return "\n".join(lines).rstrip() + "\n"


def parse_args():
    parser = argparse.ArgumentParser(description="Analyze active lateral response JSONL logs")
    parser.add_argument("event_files", nargs="+", help="JSONL event files")
    parser.add_argument("--output-md", default=None)
    parser.add_argument("--output-json", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    events = []
    for text in args.event_files:
        events.extend(load_jsonl(Path(text)))
    summary = collect_summary(events)
    markdown = render_markdown(summary)
    if args.output_json:
        output = Path(args.output_json)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.output_md:
        output = Path(args.output_md)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(markdown, encoding="utf-8")
    print(markdown, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
