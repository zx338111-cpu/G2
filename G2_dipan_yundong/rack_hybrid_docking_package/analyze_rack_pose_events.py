#!/usr/bin/env python3
"""Analyze rack_pose_monitor events from an industrial 7-rods JSONL log."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass, asdict
import json
from pathlib import Path
import statistics
import sys
from typing import Any


def percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * p
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def median(values: list[float]) -> float | None:
    return float(statistics.median(values)) if values else None


def round_or_none(value: float | None, digits: int = 4) -> float | None:
    return None if value is None else round(value, digits)


@dataclass(frozen=True)
class RackPoseStats:
    count: int
    unavailable_count: int
    warn_count: int
    ok_count: int
    yaw_available_count: int
    low_confidence_count: int
    lateral_warn_count: int
    yaw_warn_count: int
    confidence_median: float | None
    confidence_min: float | None
    distance_m_median: float | None
    lateral_m_median: float | None
    lateral_abs_p95_m: float | None
    yaw_deg_median: float | None
    yaw_abs_p95_deg: float | None
    fit_residual_m_median: float | None


def load_named_events(path: Path, event_name: str) -> list[dict[str, Any]]:
    events = []
    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_no}: invalid JSON: {exc}") from exc
            if payload.get("event") == event_name:
                events.append(payload)
    return events


def load_events(path: Path) -> list[dict[str, Any]]:
    return load_named_events(path, "rack_pose_monitor")


def warn_reasons(event: dict[str, Any]) -> set[str]:
    reasons = event.get("warn_reasons") or ()
    if isinstance(reasons, str):
        return {reasons}
    return {str(reason) for reason in reasons}


def compute_stats(events: list[dict[str, Any]]) -> RackPoseStats:
    available = [event for event in events if event.get("status") != "unavailable"]
    confidences = [
        float(event["confidence"])
        for event in available
        if event.get("confidence") is not None
    ]
    distances = [
        float(event["distance_m"])
        for event in available
        if event.get("distance_m") is not None
    ]
    laterals = [
        float(event["lateral_center_m"])
        for event in available
        if event.get("lateral_center_m") is not None
    ]
    yaws = [
        float(event["yaw_deg"])
        for event in available
        if event.get("yaw_deg") is not None
    ]
    residuals = [
        float(event["fit_residual_m"])
        for event in available
        if event.get("fit_residual_m") is not None
    ]
    reason_counts = Counter()
    for event in available:
        reason_counts.update(warn_reasons(event))

    statuses = Counter(str(event.get("status", "unknown")) for event in events)
    return RackPoseStats(
        count=len(events),
        unavailable_count=statuses.get("unavailable", 0),
        warn_count=statuses.get("warn", 0),
        ok_count=statuses.get("ok", 0),
        yaw_available_count=len(yaws),
        low_confidence_count=reason_counts.get("low_confidence", 0),
        lateral_warn_count=reason_counts.get("lateral_offset", 0),
        yaw_warn_count=reason_counts.get("yaw_offset", 0),
        confidence_median=median(confidences),
        confidence_min=min(confidences) if confidences else None,
        distance_m_median=median(distances),
        lateral_m_median=median(laterals),
        lateral_abs_p95_m=percentile([abs(value) for value in laterals], 0.95),
        yaw_deg_median=median(yaws),
        yaw_abs_p95_deg=percentile([abs(value) for value in yaws], 0.95),
        fit_residual_m_median=median(residuals),
    )


def group_key(event: dict[str, Any]) -> tuple[str, str, str]:
    rod = str(event.get("rod_index") or "unknown")
    target = str(event.get("target_mm") or "unknown")
    label = str(event.get("label") or "")
    phase = "after" if "after_approach" in label else "before" if "before_approach" in label else "other"
    return rod, target, phase


def stats_to_json(stats: RackPoseStats) -> dict[str, Any]:
    return {
        key: round_or_none(value) if isinstance(value, float) or value is None else value
        for key, value in asdict(stats).items()
    }


def summarize_yaw_shadow(events: list[dict[str, Any]]) -> dict[str, Any]:
    decisions = Counter(str(event.get("decision", "unknown")) for event in events)
    reason_counts = Counter()
    candidates = []
    for event in events:
        reasons = event.get("reasons") or ()
        if isinstance(reasons, str):
            reason_counts[reasons] += 1
        else:
            reason_counts.update(str(reason) for reason in reasons)
        candidate = event.get("candidate_robot_yaw_correction_deg")
        if candidate is not None:
            candidates.append(float(candidate))

    return {
        "count": len(events),
        "decision_counts": dict(sorted(decisions.items())),
        "reason_counts": dict(sorted(reason_counts.items())),
        "candidate_count": len(candidates),
        "candidate_yaw_correction_deg_median": round_or_none(median(candidates)),
        "candidate_yaw_correction_abs_p95_deg": round_or_none(
            percentile([abs(value) for value in candidates], 0.95)
        ),
        "candidate_sign_calibrated": False if events else None,
    }


def summarize_lateral_shadow(events: list[dict[str, Any]]) -> dict[str, Any]:
    decisions = Counter(str(event.get("decision", "unknown")) for event in events)
    reason_counts = Counter()
    execution_blockers = Counter()
    candidates = []
    execution_allowed_count = 0
    for event in events:
        reasons = event.get("reasons") or ()
        if isinstance(reasons, str):
            reason_counts[reasons] += 1
        else:
            reason_counts.update(str(reason) for reason in reasons)
        blockers = event.get("execution_blockers") or ()
        if isinstance(blockers, str):
            execution_blockers[blockers] += 1
        else:
            execution_blockers.update(str(blocker) for blocker in blockers)
        candidate = event.get("candidate_body_lateral_correction_m")
        if candidate is not None:
            candidates.append(float(candidate))
        if event.get("candidate_execution_allowed"):
            execution_allowed_count += 1

    return {
        "count": len(events),
        "decision_counts": dict(sorted(decisions.items())),
        "reason_counts": dict(sorted(reason_counts.items())),
        "execution_blocker_counts": dict(sorted(execution_blockers.items())),
        "candidate_count": len(candidates),
        "candidate_body_lateral_correction_m_median": round_or_none(
            median(candidates)
        ),
        "candidate_body_lateral_correction_abs_p95_m": round_or_none(
            percentile([abs(value) for value in candidates], 0.95)
        ),
        "candidate_execution_allowed_count": execution_allowed_count,
        "candidate_sign_calibrated": False if events else None,
    }


def summarize_lateral_active(
    decision_events: list[dict[str, Any]],
    step_result_events: list[dict[str, Any]],
    rollback_result_events: list[dict[str, Any]],
    result_events: list[dict[str, Any]],
) -> dict[str, Any]:
    decisions = Counter(str(event.get("decision", "unknown")) for event in decision_events)
    result_statuses = Counter(str(event.get("status", "unknown")) for event in result_events)
    reasons = Counter()
    improvements = []
    before_abs = []
    after_abs = []
    rollback_improvements = []
    final_laterals = []

    for event in decision_events:
        event_reasons = event.get("reasons") or ()
        if isinstance(event_reasons, str):
            reasons[event_reasons] += 1
        else:
            reasons.update(str(reason) for reason in event_reasons)
    for event in step_result_events:
        improvement = event.get("improvement_m")
        if improvement is not None:
            improvements.append(float(improvement))
        before = event.get("before_lateral_center_m")
        if before is not None:
            before_abs.append(abs(float(before)))
        after = event.get("after_lateral_center_m")
        if after is not None:
            after_abs.append(abs(float(after)))
    for event in rollback_result_events:
        improvement = event.get("rollback_improvement_m")
        if improvement is not None:
            rollback_improvements.append(float(improvement))
    for event in result_events:
        final_lateral = event.get("final_lateral_center_m")
        if final_lateral is not None:
            final_laterals.append(float(final_lateral))

    return {
        "decision_count": len(decision_events),
        "step_result_count": len(step_result_events),
        "rollback_result_count": len(rollback_result_events),
        "result_count": len(result_events),
        "decision_counts": dict(sorted(decisions.items())),
        "result_status_counts": dict(sorted(result_statuses.items())),
        "reason_counts": dict(sorted(reasons.items())),
        "improvement_m_median": round_or_none(median(improvements)),
        "improvement_m_min": round_or_none(min(improvements) if improvements else None),
        "before_lateral_abs_m_median": round_or_none(median(before_abs)),
        "after_lateral_abs_m_median": round_or_none(median(after_abs)),
        "rollback_improvement_m_median": round_or_none(median(rollback_improvements)),
        "final_lateral_m_median": round_or_none(median(final_laterals)),
        "final_lateral_abs_p95_m": round_or_none(
            percentile([abs(value) for value in final_laterals], 0.95)
        ),
    }


def recommendation(
    stats: RackPoseStats,
    *,
    min_events: int,
    max_unavailable_ratio: float,
    min_yaw_available_ratio: float,
    min_confidence: float,
    residual_limit_m: float,
    yaw_warn_deg: float,
) -> dict[str, Any]:
    if stats.count == 0:
        return {
            "pose_quality": "no_data",
            "yaw_next_step": "collect_live_pose_events",
            "lateral_next_step": "do_not_enable_lateral_control",
            "reasons": ["no rack_pose_monitor events found"],
        }

    unavailable_ratio = stats.unavailable_count / stats.count
    yaw_available_ratio = stats.yaw_available_count / max(1, stats.count - stats.unavailable_count)
    reasons = []
    if stats.count < min_events:
        reasons.append(f"need_at_least_{min_events}_events")
    if unavailable_ratio > max_unavailable_ratio:
        reasons.append("too_many_unavailable_pose_events")
    if yaw_available_ratio < min_yaw_available_ratio:
        reasons.append("yaw_not_available_enough")
    if stats.confidence_median is None or stats.confidence_median < min_confidence:
        reasons.append("median_confidence_too_low")
    if stats.fit_residual_m_median is None or stats.fit_residual_m_median > residual_limit_m:
        reasons.append("fit_residual_too_high_or_unavailable")

    pose_quality = "usable_for_yaw_review" if not reasons else "monitor_only"
    yaw_abs_p95 = stats.yaw_abs_p95_deg or 0.0
    if pose_quality != "usable_for_yaw_review":
        yaw_next_step = "keep_monitoring"
    elif yaw_abs_p95 > yaw_warn_deg:
        yaw_next_step = "candidate_for_yaw_correction_test"
    else:
        yaw_next_step = "no_yaw_correction_needed_yet"

    return {
        "pose_quality": pose_quality,
        "yaw_next_step": yaw_next_step,
        "lateral_next_step": "do_not_enable_lateral_control_until_linear_y_is_verified",
        "unavailable_ratio": round(unavailable_ratio, 4),
        "yaw_available_ratio": round(yaw_available_ratio, 4),
        "reasons": reasons,
    }


def render_markdown(summary: dict[str, Any]) -> str:
    lines = [
        "# Rack Pose Monitor Analysis",
        "",
        f"- source: `{summary['source']}`",
        f"- total rack_pose_monitor events: {summary['overall']['count']}",
        f"- pose_quality: `{summary['recommendation']['pose_quality']}`",
        f"- yaw_next_step: `{summary['recommendation']['yaw_next_step']}`",
        f"- lateral_next_step: `{summary['recommendation']['lateral_next_step']}`",
        "",
        "## Overall",
        "",
    ]
    for key, value in summary["overall"].items():
        lines.append(f"- {key}: {value}")

    if summary["recommendation"]["reasons"]:
        lines.extend(["", "## Reasons", ""])
        for reason in summary["recommendation"]["reasons"]:
            lines.append(f"- {reason}")

    yaw_shadow = summary.get("yaw_shadow") or {}
    if yaw_shadow.get("count", 0):
        lines.extend(["", "## Yaw Shadow", ""])
        for key, value in yaw_shadow.items():
            lines.append(f"- {key}: {value}")

    lateral_shadow = summary.get("lateral_shadow") or {}
    if lateral_shadow.get("count", 0):
        lines.extend(["", "## Lateral Shadow", ""])
        for key, value in lateral_shadow.items():
            lines.append(f"- {key}: {value}")

    lateral_active = summary.get("lateral_active") or {}
    if lateral_active.get("result_count", 0) or lateral_active.get("decision_count", 0):
        lines.extend(["", "## Lateral Active", ""])
        for key, value in lateral_active.items():
            lines.append(f"- {key}: {value}")

    lines.extend(["", "## By Rod/Target/Phase", ""])
    for item in summary["groups"]:
        header = (
            f"rod={item['rod_index']} target_mm={item['target_mm']} "
            f"phase={item['phase']}"
        )
        lines.append(f"### {header}")
        for key, value in item["stats"].items():
            lines.append(f"- {key}: {value}")
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def build_summary(args) -> dict[str, Any]:
    source = Path(args.event_file).resolve()
    events = load_events(source)
    yaw_shadow_events = load_named_events(source, "rack_pose_yaw_shadow")
    lateral_shadow_events = load_named_events(source, "rack_pose_lateral_shadow")
    lateral_active_decision_events = load_named_events(
        source, "rack_lateral_centering_decision"
    )
    lateral_active_step_result_events = load_named_events(
        source, "rack_lateral_centering_step_result"
    )
    lateral_active_rollback_result_events = load_named_events(
        source, "rack_lateral_centering_rollback_result"
    )
    lateral_active_result_events = load_named_events(
        source, "rack_lateral_centering_result"
    )
    overall_stats = compute_stats(events)
    groups = defaultdict(list)
    for event in events:
        groups[group_key(event)].append(event)

    group_summaries = []
    for (rod_index, target_mm, phase), group_events in sorted(groups.items()):
        group_summaries.append(
            {
                "rod_index": rod_index,
                "target_mm": target_mm,
                "phase": phase,
                "stats": stats_to_json(compute_stats(group_events)),
            }
        )

    overall = stats_to_json(overall_stats)
    rec = recommendation(
        overall_stats,
        min_events=args.min_events,
        max_unavailable_ratio=args.max_unavailable_ratio,
        min_yaw_available_ratio=args.min_yaw_available_ratio,
        min_confidence=args.min_confidence,
        residual_limit_m=args.residual_limit_m,
        yaw_warn_deg=args.yaw_warn_deg,
    )
    lateral_shadow = summarize_lateral_shadow(lateral_shadow_events)
    lateral_active = summarize_lateral_active(
        lateral_active_decision_events,
        lateral_active_step_result_events,
        lateral_active_rollback_result_events,
        lateral_active_result_events,
    )
    if lateral_shadow["count"]:
        if lateral_shadow["candidate_count"]:
            rec["lateral_next_step"] = (
                "review_lateral_shadow_candidates_and_linear_y_sweep_before_active_control"
            )
        elif lateral_shadow["decision_counts"].get("no_correction_needed", 0):
            rec["lateral_next_step"] = "no_lateral_correction_needed_yet"
        else:
            rec["lateral_next_step"] = "keep_lateral_shadow_monitoring"
    if lateral_active["result_count"]:
        if lateral_active["result_status_counts"].get("centered_after_step", 0) or lateral_active[
            "result_status_counts"
        ].get("centered", 0):
            rec["lateral_next_step"] = "review_active_centering_result_before_multi_rod_run"
        else:
            rec["lateral_next_step"] = "active_centering_failed_or_blocked_review_required"
    return {
        "source": str(source),
        "overall": overall,
        "groups": group_summaries,
        "recommendation": rec,
        "yaw_shadow": summarize_yaw_shadow(yaw_shadow_events),
        "lateral_shadow": lateral_shadow,
        "lateral_active": lateral_active,
        "thresholds": {
            "min_events": args.min_events,
            "max_unavailable_ratio": args.max_unavailable_ratio,
            "min_yaw_available_ratio": args.min_yaw_available_ratio,
            "min_confidence": args.min_confidence,
            "residual_limit_m": args.residual_limit_m,
            "yaw_warn_deg": args.yaw_warn_deg,
        },
    }


def parse_args():
    parser = argparse.ArgumentParser(description="Analyze rack_pose_monitor JSONL events")
    parser.add_argument("event_file", help="industrial_7_rods_total_controller JSONL event file")
    parser.add_argument("--output-md", default=None, help="optional Markdown report path")
    parser.add_argument("--output-json", default=None, help="optional JSON summary path")
    parser.add_argument("--min-events", type=int, default=12, help="minimum events before judging yaw readiness")
    parser.add_argument("--max-unavailable-ratio", type=float, default=0.10)
    parser.add_argument("--min-yaw-available-ratio", type=float, default=0.80)
    parser.add_argument("--min-confidence", type=float, default=0.45)
    parser.add_argument("--residual-limit-m", type=float, default=0.08)
    parser.add_argument("--yaw-warn-deg", type=float, default=3.0)
    args = parser.parse_args()

    if args.min_events < 0:
        raise SystemExit("--min-events must be >= 0")
    if not (0.0 <= args.max_unavailable_ratio <= 1.0):
        raise SystemExit("--max-unavailable-ratio must be in [0, 1]")
    if not (0.0 <= args.min_yaw_available_ratio <= 1.0):
        raise SystemExit("--min-yaw-available-ratio must be in [0, 1]")
    if not (0.0 <= args.min_confidence <= 1.0):
        raise SystemExit("--min-confidence must be in [0, 1]")
    if args.residual_limit_m <= 0.0:
        raise SystemExit("--residual-limit-m must be positive")
    if args.yaw_warn_deg <= 0.0:
        raise SystemExit("--yaw-warn-deg must be positive")
    return args


def main() -> int:
    args = parse_args()
    summary = build_summary(args)
    markdown = render_markdown(summary)

    if args.output_json:
        output_json = Path(args.output_json)
        output_json.parent.mkdir(parents=True, exist_ok=True)
        with output_json.open("w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2, sort_keys=True)
            f.write("\n")
    if args.output_md:
        output_md = Path(args.output_md)
        output_md.parent.mkdir(parents=True, exist_ok=True)
        output_md.write_text(markdown, encoding="utf-8")

    sys.stdout.write(markdown)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
