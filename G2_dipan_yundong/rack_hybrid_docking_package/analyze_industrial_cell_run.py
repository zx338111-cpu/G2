#!/usr/bin/env python3
"""Summarize one industrial-cell seven-rod runner log.

The runner log is mostly JSON-lines mixed with GDK text output. This tool only
reads the file and parses JSON event lines; it never connects to GDK and never
sends robot commands.

Role in the class/import architecture:

The mission runner emits structured ``event`` rows for phase starts, phase
completion, child primitive timing, fine positioning, retreat, and yaw refine.
This analyzer turns that mixed log into a compact timing report after a run.
Because it is read-only, it is safe to run after a failed or interrupted live
mission.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path
import statistics
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_file", help="Combined runner log, for example logs/industrial_cell_7_rods_optimized_live.log")
    parser.add_argument(
        "--max-gap-s",
        type=float,
        default=120.0,
        help="Ignore larger phase gaps in averages; they usually mean manual pause/resume work.",
    )
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON instead of text")
    return parser.parse_args()


def load_events(path: Path) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = line.strip()
        if not line.startswith("{") or '"event"' not in line:
            continue
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(item, dict):
            events.append(item)
    return events


def number_summary(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {"n": 0}
    return {
        "n": len(values),
        "avg_s": round(statistics.mean(values), 3),
        "min_s": round(min(values), 3),
        "max_s": round(max(values), 3),
    }


def summarize(events: list[dict[str, Any]], *, max_gap_s: float) -> dict[str, Any]:
    final_state: dict[str, Any] | None = None
    phase_rows: list[dict[str, Any]] = []
    previous_updated_at: float | None = None

    for event in events:
        if isinstance(event.get("state"), dict):
            final_state = event["state"]
        if event.get("event") != "mission_phase_done":
            continue
        state = event.get("state")
        if not isinstance(state, dict):
            continue
        final_state = state
        updated_at = state.get("updated_at")
        if not isinstance(updated_at, (int, float)):
            continue
        elapsed_s = None if previous_updated_at is None else float(updated_at - previous_updated_at)
        previous_updated_at = float(updated_at)
        completed = state.get("last_success_step")
        phase_rows.append(
            {
                "rod_index": state.get("rod_index"),
                "next_phase": state.get("phase"),
                "completed_step": completed,
                "elapsed_s": None if elapsed_s is None else round(elapsed_s, 3),
                "used_in_average": elapsed_s is not None and 0.0 < elapsed_s <= max_gap_s,
            }
        )

    phase_by_step: dict[str, list[float]] = defaultdict(list)
    excluded_phase_rows: list[dict[str, Any]] = []
    for row in phase_rows:
        elapsed_s = row.get("elapsed_s")
        completed = row.get("completed_step")
        if not isinstance(completed, str) or not isinstance(elapsed_s, (int, float)) or elapsed_s <= 0.0:
            continue
        if elapsed_s > max_gap_s:
            excluded_phase_rows.append(row)
            continue
        phase_by_step[completed].append(float(elapsed_s))

    measured: dict[str, list[float]] = defaultdict(list)
    yaw_initial_abs: list[float] = []
    yaw_commands: list[float] = []
    for event in events:
        name = event.get("event")
        if name == "nav_result" and isinstance(event.get("elapsed_s"), (int, float)):
            measured["nav_result"].append(float(event["elapsed_s"]))
        elif name == "yaw_refine_result":
            if isinstance(event.get("elapsed_s"), (int, float)):
                measured["yaw_refine"].append(float(event["elapsed_s"]))
            if isinstance(event.get("initial_error_deg"), (int, float)):
                yaw_initial_abs.append(abs(float(event["initial_error_deg"])))
            if isinstance(event.get("commands"), (int, float)):
                yaw_commands.append(float(event["commands"]))
        elif name == "local_fine_position_done":
            result = event.get("result")
            if isinstance(result, dict) and isinstance(result.get("elapsed_s"), (int, float)):
                measured[str(event.get("label", "fine_position"))].append(float(result["elapsed_s"]))
        elif name == "local_child_step_done" and isinstance(event.get("elapsed_s"), (int, float)):
            measured[str(event.get("label", "local_child_step"))].append(float(event["elapsed_s"]))
        elif name == "local_chassis_relative_done" and isinstance(event.get("elapsed_s"), (int, float)):
            measured[str(event.get("label", "chassis_relative"))].append(float(event["elapsed_s"]))
        elif name == "local_retreat_done":
            result = event.get("result")
            if isinstance(result, dict) and isinstance(result.get("elapsed_s"), (int, float)):
                measured[str(event.get("label", "retreat"))].append(float(result["elapsed_s"]))

    phase_summary = {
        label: number_summary(values)
        for label, values in sorted(phase_by_step.items())
    }
    measured_summary = {
        label: number_summary(values)
        for label, values in sorted(measured.items())
    }
    bottlenecks = sorted(
        (
            {"label": label, **summary}
            for label, summary in phase_summary.items()
            if summary.get("n", 0)
        ),
        key=lambda item: float(item.get("avg_s", 0.0)),
        reverse=True,
    )
    return {
        "event_count": len(events),
        "final_state": final_state,
        "phase_summary": phase_summary,
        "measured_summary": measured_summary,
        "yaw_initial_abs_deg": number_summary(yaw_initial_abs),
        "yaw_command_count": number_summary(yaw_commands),
        "top_phase_bottlenecks": bottlenecks[:8],
        "excluded_phase_rows": excluded_phase_rows,
    }


def print_text(summary: dict[str, Any]) -> None:
    print("Industrial cell run summary")
    print(f"events: {summary['event_count']}")
    print("final_state:")
    print(json.dumps(summary.get("final_state"), ensure_ascii=False, indent=2))
    print("")
    print("top phase bottlenecks:")
    for item in summary["top_phase_bottlenecks"]:
        print(
            f"- {item['label']}: n={item['n']} avg={item['avg_s']}s "
            f"min={item['min_s']}s max={item['max_s']}s"
        )
    print("")
    print("measured primitives:")
    for label, item in summary["measured_summary"].items():
        if not item.get("n"):
            continue
        print(
            f"- {label}: n={item['n']} avg={item['avg_s']}s "
            f"min={item['min_s']}s max={item['max_s']}s"
        )
    yaw_error = summary["yaw_initial_abs_deg"]
    yaw_commands = summary["yaw_command_count"]
    print("")
    print(
        "yaw refine initial abs error: "
        f"n={yaw_error.get('n', 0)} avg={yaw_error.get('avg_s', 0)}deg "
        f"max={yaw_error.get('max_s', 0)}deg"
    )
    print(
        "yaw refine commands: "
        f"n={yaw_commands.get('n', 0)} avg={yaw_commands.get('avg_s', 0)} "
        f"max={yaw_commands.get('max_s', 0)}"
    )
    if summary["excluded_phase_rows"]:
        print("")
        print("excluded large gaps:")
        for row in summary["excluded_phase_rows"]:
            print(
                f"- rod={row['rod_index']} completed={row['completed_step']} "
                f"elapsed={row['elapsed_s']}s"
            )


def main() -> int:
    args = parse_args()
    path = Path(args.log_file).resolve()
    summary = summarize(load_events(path), max_gap_s=args.max_gap_s)
    if args.json:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    else:
        print_text(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
