#!/usr/bin/env python3
"""Read-only sweep for rack pose ROI stability.

The active lateral-centering gate depends on LidarRackPose.lateral_center_m.
This diagnostic keeps the robot stationary and samples multiple ROI variants so
we can pick parameters that are stable before allowing any lateral correction.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import time

import numpy as np


@dataclass(frozen=True)
class RoiConfig:
    name: str
    min_range_m: float
    max_range_m: float
    lateral_half_width_m: float
    z_min_m: float
    z_max_m: float
    bin_width_m: float
    min_cluster_points: int


DEFAULT_CONFIGS = (
    RoiConfig("current_min012_max25_lh08_z06_12_bin25_pts20", 0.12, 2.5, 0.80, 0.60, 1.20, 0.25, 20),
    RoiConfig("dock_min08_max16_lh08_z06_12_bin25_pts20", 0.80, 1.60, 0.80, 0.60, 1.20, 0.25, 20),
    RoiConfig("dock_min08_max16_lh06_z06_12_bin25_pts20", 0.80, 1.60, 0.60, 0.60, 1.20, 0.25, 20),
    RoiConfig("dock_min08_max16_lh05_z06_12_bin25_pts20", 0.80, 1.60, 0.50, 0.60, 1.20, 0.25, 20),
    RoiConfig("dock_min08_max16_lh04_z06_12_bin25_pts20", 0.80, 1.60, 0.40, 0.60, 1.20, 0.25, 20),
    RoiConfig("near_min08_max14_lh06_z06_12_bin20_pts20", 0.80, 1.40, 0.60, 0.60, 1.20, 0.20, 20),
    RoiConfig("near_min08_max14_lh05_z06_12_bin20_pts20", 0.80, 1.40, 0.50, 0.60, 1.20, 0.20, 20),
    RoiConfig("near_min09_max14_lh05_z06_12_bin20_pts20", 0.90, 1.40, 0.50, 0.60, 1.20, 0.20, 20),
    RoiConfig("near_min10_max14_lh05_z06_12_bin20_pts20", 1.00, 1.40, 0.50, 0.60, 1.20, 0.20, 20),
    RoiConfig("high_min08_max16_lh06_z07_13_bin25_pts20", 0.80, 1.60, 0.60, 0.70, 1.30, 0.25, 20),
    RoiConfig("high_min08_max16_lh05_z07_13_bin25_pts20", 0.80, 1.60, 0.50, 0.70, 1.30, 0.25, 20),
    RoiConfig("high_min09_max14_lh05_z07_13_bin20_pts20", 0.90, 1.40, 0.50, 0.70, 1.30, 0.20, 20),
)


def median(values: list[float]) -> float | None:
    return float(statistics.median(values)) if values else None


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


def round_or_none(value: float | None, digits: int = 4) -> float | None:
    return None if value is None else round(value, digits)


def parse_config(text: str) -> RoiConfig:
    parts = text.split(":")
    if len(parts) != 8:
        raise argparse.ArgumentTypeError(
            "config must be name:min_range:max_range:lateral_half:z_min:z_max:bin_width:min_points"
        )
    name = parts[0]
    try:
        return RoiConfig(
            name=name,
            min_range_m=float(parts[1]),
            max_range_m=float(parts[2]),
            lateral_half_width_m=float(parts[3]),
            z_min_m=float(parts[4]),
            z_max_m=float(parts[5]),
            bin_width_m=float(parts[6]),
            min_cluster_points=int(parts[7]),
        )
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def parse_xyz(pointcloud):
    """Parse PointCloud2-like GDK data into x/y/z numpy arrays."""
    if pointcloud is None or not hasattr(pointcloud, "data"):
        return None
    point_step = int(getattr(pointcloud, "point_step", 0))
    if point_step <= 0:
        return None

    raw_data = getattr(pointcloud, "data", None)
    if raw_data is None:
        return None
    raw = raw_data.astype(np.uint8) if isinstance(raw_data, np.ndarray) else np.frombuffer(raw_data, dtype=np.uint8)
    point_count = len(raw) // point_step
    if point_count <= 0:
        return None

    raw = raw[: point_count * point_step].reshape((point_count, point_step))
    values = {}
    for field in getattr(pointcloud, "fields", []):
        name = getattr(field, "name", "")
        if name not in ("x", "y", "z"):
            continue
        offset = int(getattr(field, "offset", -1))
        if offset < 0 or offset + 4 > point_step:
            continue
        field_raw = np.ascontiguousarray(raw[:, offset : offset + 4])
        values[name] = field_raw.view(np.float32).reshape(-1)

    if not all(name in values for name in ("x", "y", "z")):
        return None
    return values["x"], values["y"], values["z"]


def read_rack_cluster(lidar, lidar_type, config: RoiConfig):
    pointcloud = lidar.get_latest_pointcloud(lidar_type, 1000.0)
    xyz = parse_xyz(pointcloud)
    if xyz is None:
        return None

    x, y, z = xyz
    forward_coord = x
    lateral_coord = y
    vertical_coord = z
    valid = (
        np.isfinite(forward_coord)
        & np.isfinite(lateral_coord)
        & np.isfinite(vertical_coord)
        & (forward_coord >= config.min_range_m)
        & (forward_coord <= config.max_range_m)
        & (np.abs(lateral_coord) <= config.lateral_half_width_m)
        & (vertical_coord >= config.z_min_m)
        & (vertical_coord <= config.z_max_m)
    )

    forward = forward_coord[valid]
    lateral = lateral_coord[valid]
    if len(forward) < config.min_cluster_points:
        return None

    bins = np.arange(
        config.min_range_m,
        config.max_range_m + config.bin_width_m,
        config.bin_width_m,
    )
    counts, edges = np.histogram(forward, bins=bins)
    selected_index = None
    for index, count in enumerate(counts):
        if count >= config.min_cluster_points:
            selected_index = index
            break
    if selected_index is None:
        return None

    bin_start = float(edges[selected_index])
    bin_end = float(edges[selected_index + 1])
    in_cluster = (forward >= bin_start) & (forward < bin_end)
    cluster_forward = forward[in_cluster]
    cluster_lateral = lateral[in_cluster]
    if len(cluster_forward) < config.min_cluster_points:
        return None

    return {
        "forward": cluster_forward,
        "lateral": cluster_lateral,
        "nearest_m": float(np.min(forward)),
        "roi_points": int(len(forward)),
        "bin_start_m": bin_start,
        "bin_end_m": bin_end,
    }


def read_rack_pose(lidar, lidar_type, config: RoiConfig):
    cluster = read_rack_cluster(lidar, lidar_type, config)
    if cluster is None:
        return None

    cluster_forward = cluster["forward"]
    cluster_lateral = cluster["lateral"]
    distance_m = float(np.percentile(cluster_forward, 10))
    lateral_center_m = float(np.median(cluster_lateral))
    lateral_p05 = float(np.percentile(cluster_lateral, 5))
    lateral_p95 = float(np.percentile(cluster_lateral, 95))
    lateral_span_m = max(0.0, lateral_p95 - lateral_p05)

    yaw_deg = None
    fit_residual_m = None
    if lateral_span_m >= 0.08 and len(cluster_lateral) >= max(6, config.min_cluster_points // 2):
        slope, intercept = np.polyfit(cluster_lateral, cluster_forward, 1)
        predicted = slope * cluster_lateral + intercept
        residual = np.abs(cluster_forward - predicted)
        fit_residual_m = float(np.median(residual))
        yaw_deg = float(math.degrees(math.atan(float(slope))))

    point_score = min(1.0, len(cluster_forward) / float(config.min_cluster_points * 3))
    span_score = min(1.0, max(0.0, (lateral_span_m - 0.10) / 0.40))
    residual_score = 0.35
    if fit_residual_m is not None:
        residual_score = min(1.0, max(0.0, 1.0 - fit_residual_m / 0.08))
    confidence = max(
        0.0,
        min(1.0, 0.40 * point_score + 0.30 * span_score + 0.30 * residual_score),
    )

    return {
        "distance_m": distance_m,
        "lateral_center_m": lateral_center_m,
        "yaw_deg": yaw_deg,
        "confidence": confidence,
        "cluster_points": int(len(cluster_forward)),
        "roi_points": cluster["roi_points"],
        "lateral_span_m": lateral_span_m,
        "fit_residual_m": fit_residual_m,
        "bin_start_m": cluster["bin_start_m"],
        "bin_end_m": cluster["bin_end_m"],
    }


def sample_config(lidar, lidar_type, config: RoiConfig, samples: int, interval_s: float):
    sample_rows = []
    errors = []
    for sample_index in range(1, samples + 1):
        try:
            pose = read_rack_pose(lidar, lidar_type, config)
        except Exception as exc:
            pose = None
            errors.append(f"{sample_index}:{type(exc).__name__}:{exc}")

        if pose is None:
            sample_rows.append({"sample_index": sample_index, "status": "unavailable"})
        else:
            row = dict(pose)
            row.update(sample_index=sample_index, status="ok")
            sample_rows.append(row)
        if sample_index < samples:
            time.sleep(interval_s)
    return sample_rows, errors


def summarize_config(config: RoiConfig, samples: list[dict], errors: list[str]):
    valid = [sample for sample in samples if sample.get("status") == "ok"]
    laterals = [float(sample["lateral_center_m"]) for sample in valid]
    distances = [float(sample["distance_m"]) for sample in valid]
    yaws = [
        float(sample["yaw_deg"])
        for sample in valid
        if sample.get("yaw_deg") is not None
    ]
    confidences = [float(sample["confidence"]) for sample in valid]
    residuals = [
        float(sample["fit_residual_m"])
        for sample in valid
        if sample.get("fit_residual_m") is not None
    ]
    cluster_points = [float(sample["cluster_points"]) for sample in valid]
    roi_points = [float(sample["roi_points"]) for sample in valid]
    bins = Counter(
        (
            round(float(sample["bin_start_m"]), 3),
            round(float(sample["bin_end_m"]), 3),
        )
        for sample in valid
        if sample.get("bin_start_m") is not None and sample.get("bin_end_m") is not None
    )

    lateral_span = (max(laterals) - min(laterals)) if laterals else None
    valid_ratio = len(valid) / max(1, len(samples))
    result = {
        "config": asdict(config),
        "sample_count": len(samples),
        "valid_count": len(valid),
        "valid_ratio": round(valid_ratio, 4),
        "unavailable_count": len(samples) - len(valid),
        "lateral_center_m_median": round_or_none(median(laterals)),
        "lateral_abs_median": round_or_none(median([abs(value) for value in laterals])),
        "lateral_sample_span_m": round_or_none(lateral_span),
        "lateral_abs_p95_m": round_or_none(percentile([abs(value) for value in laterals], 0.95)),
        "distance_m_median": round_or_none(median(distances)),
        "yaw_deg_median": round_or_none(median(yaws)),
        "yaw_abs_p95_deg": round_or_none(percentile([abs(value) for value in yaws], 0.95)),
        "confidence_median": round_or_none(median(confidences)),
        "fit_residual_m_median": round_or_none(median(residuals)),
        "cluster_points_median": round_or_none(median(cluster_points), digits=1),
        "roi_points_median": round_or_none(median(roi_points), digits=1),
        "bin_counts": {f"{start:.3f}-{end:.3f}": count for (start, end), count in sorted(bins.items())},
        "errors": errors[-3:],
    }
    result["passes_active_stability_gate"] = (
        len(valid) >= max(6, int(0.75 * len(samples)))
        and lateral_span is not None
        and lateral_span <= 0.08
        and result["confidence_median"] is not None
        and result["confidence_median"] >= 0.65
        and (
            result["fit_residual_m_median"] is None
            or result["fit_residual_m_median"] <= 0.07
        )
    )
    return result


def rank_summary(summary: dict):
    return (
        0 if summary["passes_active_stability_gate"] else 1,
        -summary["valid_ratio"],
        999.0 if summary["lateral_sample_span_m"] is None else summary["lateral_sample_span_m"],
        999.0 if summary["fit_residual_m_median"] is None else summary["fit_residual_m_median"],
        999.0 if summary["lateral_abs_median"] is None else summary["lateral_abs_median"],
    )


def render_markdown(report: dict) -> str:
    lines = [
        "# Rack Pose ROI Sweep",
        "",
        f"- timestamp: `{report['timestamp']}`",
        f"- samples_per_config: {report['samples_per_config']}",
        f"- interval_s: {report['interval_s']}",
        "",
        "## Ranked Summary",
        "",
    ]
    for index, item in enumerate(report["ranked"], 1):
        status = "PASS" if item["passes_active_stability_gate"] else "BLOCK"
        cfg = item["config"]
        lines.append(
            f"{index}. `{item['config']['name']}` {status}: "
            f"valid={item['valid_count']}/{item['sample_count']}, "
            f"lat_median={item['lateral_center_m_median']}, "
            f"lat_span={item['lateral_sample_span_m']}, "
            f"yaw_med={item['yaw_deg_median']}, "
            f"conf_med={item['confidence_median']}, "
            f"res_med={item['fit_residual_m_median']}, "
            f"range={cfg['min_range_m']}-{cfg['max_range_m']}, "
            f"lh={cfg['lateral_half_width_m']}, z={cfg['z_min_m']}-{cfg['z_max_m']}, "
            f"bin={cfg['bin_width_m']}, pts={cfg['min_cluster_points']}"
        )
    lines.extend(["", "## Details", ""])
    for item in report["ranked"]:
        lines.append(f"### {item['config']['name']}")
        for key, value in item.items():
            if key in ("config", "samples"):
                continue
            lines.append(f"- {key}: {value}")
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def parse_args():
    parser = argparse.ArgumentParser(description="Read-only rack pose ROI sweep")
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--interval-s", type=float, default=0.12)
    parser.add_argument("--config", action="append", type=parse_config, default=[])
    parser.add_argument("--output-json", default=None)
    parser.add_argument("--output-md", default=None)
    args = parser.parse_args()
    if args.samples <= 0:
        raise SystemExit("--samples must be positive")
    if args.interval_s < 0:
        raise SystemExit("--interval-s must be >= 0")
    return args


def main() -> int:
    args = parse_args()
    configs = tuple(args.config) if args.config else DEFAULT_CONFIGS

    import agibot_gdk

    summaries = []
    all_samples = {}
    result = agibot_gdk.gdk_init()
    gdk_res = getattr(agibot_gdk, "GDKRes", None)
    if gdk_res is not None and result not in (None, gdk_res.kSuccess):
        raise RuntimeError(f"GDK init failed: {result}")

    lidar = agibot_gdk.Lidar()
    lidar_type = agibot_gdk.LidarType.kLidarFront
    time.sleep(0.8)
    try:
        for config in configs:
            samples, errors = sample_config(
                lidar,
                lidar_type,
                config=config,
                samples=args.samples,
                interval_s=args.interval_s,
            )
            summary = summarize_config(config, samples, errors)
            summaries.append(summary)
            all_samples[config.name] = samples
            print(
                "roi_sweep_config "
                f"name={config.name} valid={summary['valid_count']}/{summary['sample_count']} "
                f"lat_median={summary['lateral_center_m_median']} "
                f"lat_span={summary['lateral_sample_span_m']} "
                f"conf_med={summary['confidence_median']} "
                f"res_med={summary['fit_residual_m_median']} "
                f"pass={summary['passes_active_stability_gate']}",
                flush=True,
            )
    finally:
        try:
            lidar.close_lidar()
        except Exception as exc:
            print(f"close_lidar_error={type(exc).__name__}: {exc}", flush=True)
        try:
            agibot_gdk.gdk_release()
        except Exception as exc:
            print(f"gdk_release_error={type(exc).__name__}: {exc}", flush=True)

    ranked = sorted(summaries, key=rank_summary)
    report = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "samples_per_config": args.samples,
        "interval_s": args.interval_s,
        "ranked": ranked,
        "samples": all_samples,
    }

    if args.output_json:
        output_json = Path(args.output_json)
        output_json.parent.mkdir(parents=True, exist_ok=True)
        with output_json.open("w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2, sort_keys=True)
            f.write("\n")
    markdown = render_markdown(report)
    if args.output_md:
        output_md = Path(args.output_md)
        output_md.parent.mkdir(parents=True, exist_ok=True)
        output_md.write_text(markdown, encoding="utf-8")

    print(markdown, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
