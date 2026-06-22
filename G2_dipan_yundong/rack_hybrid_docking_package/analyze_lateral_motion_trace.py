#!/usr/bin/env python3
"""Analyze lateral motion trace JSON reports."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path
import statistics
from typing import Any


def median(values: list[float]) -> float | None:
    return float(statistics.median(values)) if values else None


def round_or_none(value: float | None, digits: int = 4) -> float | None:
    return None if value is None else round(value, digits)


def load_trace(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    data["_source"] = str(path)
    return data


def collect_rows(traces: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for trace in traces:
        args = trace.get("args") or {}
        phase_summary = trace.get("phase_summary") or {}
        expected_lateral_m = float(args.get("speed_mps") or 0.0) * float(args.get("duration_s") or 0.0)
        for leg in trace.get("legs") or []:
            vy_mps = leg.get("vy_mps")
            odom_delta = leg.get("odom_delta") or {}
            before = leg.get("before_lateral_center_m_median")
            after = leg.get("after_lateral_center_m_median")
            row = {
                "source": trace.get("_source"),
                "status": trace.get("status"),
                "sequence": args.get("sequence"),
                "speed_mps": args.get("speed_mps"),
                "duration_s": args.get("duration_s"),
                "expected_lateral_m": expected_lateral_m,
                "leg_index": leg.get("leg_index"),
                "vy_mps": vy_mps,
                "sign": "positive" if vy_mps is not None and float(vy_mps) > 0.0 else "negative",
                "before_lateral_center_m": before,
                "after_lateral_center_m": after,
                "improvement_m": leg.get("improvement_m"),
                "odom_body_lateral_m": odom_delta.get("body_lateral_m"),
                "odom_body_forward_m": odom_delta.get("body_forward_m"),
                "odom_yaw_delta_deg": odom_delta.get("yaw_delta_deg"),
                "pre_summary": phase_summary.get(f"leg_{leg.get('leg_index')}_pre") or {},
                "post_summary": phase_summary.get(f"leg_{leg.get('leg_index')}_post") or {},
            }
            rows.append(row)
    return rows


def float_values(rows: list[dict[str, Any]], key: str) -> list[float]:
    values = []
    for row in rows:
        value = row.get(key)
        if value is not None:
            values.append(float(value))
    return values


def collect_summary(traces: list[dict[str, Any]]) -> dict[str, Any]:
    rows = collect_rows(traces)
    by_sign_rows = defaultdict(list)
    for row in rows:
        by_sign_rows[row["sign"]].append(row)

    by_sign = {}
    for sign, sign_rows in sorted(by_sign_rows.items()):
        improvements = float_values(sign_rows, "improvement_m")
        odom_lateral = float_values(sign_rows, "odom_body_lateral_m")
        odom_forward = float_values(sign_rows, "odom_body_forward_m")
        yaw_delta = float_values(sign_rows, "odom_yaw_delta_deg")
        by_sign[sign] = {
            "count": len(sign_rows),
            "improvement_m_median": round_or_none(median(improvements)),
            "improvement_m_min": round_or_none(min(improvements) if improvements else None),
            "improvement_m_max": round_or_none(max(improvements) if improvements else None),
            "improved_count": sum(1 for value in improvements if value > 0.0),
            "worse_count": sum(1 for value in improvements if value < 0.0),
            "odom_body_lateral_m_median": round_or_none(median(odom_lateral)),
            "odom_body_forward_m_median": round_or_none(median(odom_forward)),
            "odom_yaw_delta_deg_median": round_or_none(median(yaw_delta)),
        }

    improvements = float_values(rows, "improvement_m")
    odom_lateral = float_values(rows, "odom_body_lateral_m")
    expected = float_values(rows, "expected_lateral_m")
    expected_median = median(expected)
    abs_lateral_median = median([abs(value) for value in odom_lateral])

    reasons = []
    if rows and all((row.get("improvement_m") or 0.0) < 0.0 for row in rows):
        reasons.append("all_trace_steps_worsened_rack_lateral_pose")
    if "positive" in by_sign and "negative" in by_sign:
        pos_lateral = by_sign["positive"].get("odom_body_lateral_m_median")
        neg_lateral = by_sign["negative"].get("odom_body_lateral_m_median")
        if pos_lateral is not None and neg_lateral is not None and pos_lateral * neg_lateral > 0.0:
            reasons.append("positive_and_negative_linear_y_have_same_odom_lateral_sign")
    if expected_median and abs_lateral_median is not None and abs_lateral_median < expected_median * 0.50:
        reasons.append("odom_lateral_response_less_than_half_expected_open_loop_distance")

    if reasons:
        recommendation = "keep_active_lateral_motion_disabled"
    elif any((row.get("improvement_m") or 0.0) > 0.0 for row in rows):
        recommendation = "collect_repeated_single_direction_traces_before_enablement"
    else:
        recommendation = "collect_more_trace_data"

    return {
        "files": [trace.get("_source") for trace in traces],
        "trace_count": len(traces),
        "leg_count": len(rows),
        "improvement_m_median": round_or_none(median(improvements)),
        "expected_lateral_m_median": round_or_none(expected_median),
        "odom_body_lateral_abs_m_median": round_or_none(abs_lateral_median),
        "by_sign": by_sign,
        "recommendation": recommendation,
        "recommendation_reasons": reasons,
        "legs": rows,
    }


def render_markdown(summary: dict[str, Any]) -> str:
    lines = [
        "# Lateral Motion Trace Analysis",
        "",
        f"- files: {len(summary['files'])}",
        f"- trace_count: {summary['trace_count']}",
        f"- leg_count: {summary['leg_count']}",
        f"- improvement_m_median: {summary['improvement_m_median']}",
        f"- expected_lateral_m_median: {summary['expected_lateral_m_median']}",
        f"- odom_body_lateral_abs_m_median: {summary['odom_body_lateral_abs_m_median']}",
        f"- recommendation: `{summary['recommendation']}`",
        f"- recommendation_reasons: {summary['recommendation_reasons']}",
        "",
        "## By Sign",
        "",
    ]
    for sign, sign_summary in summary["by_sign"].items():
        lines.append(
            f"- `{sign}`: count={sign_summary['count']}, "
            f"improvement_m_median={sign_summary['improvement_m_median']}, "
            f"improved={sign_summary['improved_count']}, worse={sign_summary['worse_count']}, "
            f"odom_lateral_median={sign_summary['odom_body_lateral_m_median']}, "
            f"odom_forward_median={sign_summary['odom_body_forward_m_median']}, "
            f"yaw_delta_deg_median={sign_summary['odom_yaw_delta_deg_median']}"
        )

    lines.extend(["", "## Legs", ""])
    for row in summary["legs"]:
        lines.append(
            f"- `{Path(row['source']).name}` leg={row['leg_index']} vy={row['vy_mps']}: "
            f"before={round_or_none(row.get('before_lateral_center_m'))}, "
            f"after={round_or_none(row.get('after_lateral_center_m'))}, "
            f"improvement={round_or_none(row.get('improvement_m'))}, "
            f"odom_lateral={round_or_none(row.get('odom_body_lateral_m'))}, "
            f"odom_forward={round_or_none(row.get('odom_body_forward_m'))}, "
            f"yaw_delta={round_or_none(row.get('odom_yaw_delta_deg'))}, "
            f"pre_robust_span={row.get('pre_summary', {}).get('lateral_sample_robust_span_m')}, "
            f"post_robust_span={row.get('post_summary', {}).get('lateral_sample_robust_span_m')}"
        )
    return "\n".join(lines).rstrip() + "\n"


def parse_args():
    parser = argparse.ArgumentParser(description="Analyze lateral motion trace JSON reports")
    parser.add_argument("json_files", nargs="+", type=Path)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-md", type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    traces = [load_trace(path) for path in args.json_files]
    summary = collect_summary(traces)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    markdown = render_markdown(summary)
    if args.output_md:
        args.output_md.parent.mkdir(parents=True, exist_ok=True)
        args.output_md.write_text(markdown, encoding="utf-8")
    print(markdown, end="")


if __name__ == "__main__":
    main()
