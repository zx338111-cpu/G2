#!/usr/bin/env python3
"""Read-only GDK lidar snapshot for front/back point clouds."""

import math
import time

import agibot_gdk
import numpy as np


def _field_offsets(pointcloud):
    offsets = {}
    for field in getattr(pointcloud, "fields", []):
        name = getattr(field, "name", "")
        if name in ("x", "y", "z", "intensity"):
            offsets[name] = int(getattr(field, "offset", -1))
    return offsets


def _point_stats(pointcloud):
    data = getattr(pointcloud, "data", None)
    point_step = int(getattr(pointcloud, "point_step", 0))
    if data is None or point_step <= 0:
        return "stats unavailable: no data or invalid point_step"

    raw = data.astype(np.uint8) if isinstance(data, np.ndarray) else np.frombuffer(data, dtype=np.uint8)
    point_count = len(raw) // point_step
    if point_count <= 0:
        return "stats unavailable: zero points"

    raw = raw[: point_count * point_step].reshape((point_count, point_step))
    offsets = _field_offsets(pointcloud)
    missing = [name for name in ("x", "y", "z") if name not in offsets or offsets[name] < 0]
    if missing:
        return "stats unavailable: missing fields " + ",".join(missing)

    coords = []
    for name in ("x", "y", "z"):
        offset = offsets[name]
        if offset + 4 > point_step:
            return f"stats unavailable: field {name} offset out of range"
        coords.append(np.ascontiguousarray(raw[:, offset : offset + 4]).view(np.float32).reshape(-1))

    xyz = np.column_stack(coords)
    finite_mask = np.isfinite(xyz).all(axis=1)
    finite = xyz[finite_mask]
    if finite.size == 0:
        return f"points={point_count} finite=0"

    distances = np.linalg.norm(finite, axis=1)
    return (
        f"points={point_count} finite={len(finite)} "
        f"x=[{finite[:, 0].min():.3f},{finite[:, 0].max():.3f}] "
        f"y=[{finite[:, 1].min():.3f},{finite[:, 1].max():.3f}] "
        f"z=[{finite[:, 2].min():.3f},{finite[:, 2].max():.3f}] "
        f"dist=[{distances.min():.3f},{distances.max():.3f}] "
        f"mean_dist={distances.mean():.3f}"
    )


def _print_latency(lidar, lidar_type, name):
    try:
        stats = lidar.get_lidar_latency(lidar_type, 5.0)
        print(
            f"{name} latency_ms max={stats.max_latency_ms:.2f} "
            f"avg={stats.avg_latency_ms:.2f} p99={stats.p99_latency_ms:.2f}"
        )
    except Exception as exc:
        print(f"{name} latency_error={type(exc).__name__}: {exc}")


def _sample_lidar(lidar, lidar_type, name, samples):
    try:
        fps = lidar.get_lidar_fps(lidar_type)
        print(f"{name} fps={fps:.2f}")
    except Exception as exc:
        print(f"{name} fps_error={type(exc).__name__}: {exc}")

    _print_latency(lidar, lidar_type, name)

    ok = 0
    for idx in range(1, samples + 1):
        try:
            pc = lidar.get_latest_pointcloud(lidar_type, 1000.0)
        except Exception as exc:
            print(f"{name} sample={idx} error={type(exc).__name__}: {exc}")
            time.sleep(0.2)
            continue

        if pc is None:
            print(f"{name} sample={idx} none")
            time.sleep(0.2)
            continue

        ok += 1
        fields = ",".join(getattr(field, "name", "") for field in getattr(pc, "fields", []))
        data_len = len(getattr(pc, "data", []))
        width = int(getattr(pc, "width", 0))
        height = int(getattr(pc, "height", 0))
        point_step = int(getattr(pc, "point_step", 0))
        row_step = int(getattr(pc, "row_step", 0))
        timestamp_ns = int(getattr(pc, "timestamp_ns", 0))
        expected = width * max(height, 1)
        raw_points = math.floor(data_len / point_step) if point_step > 0 else 0
        print(
            f"{name} sample={idx} ts={timestamp_ns} size={width}x{height} "
            f"point_step={point_step} row_step={row_step} data_len={data_len} "
            f"expected_points={expected} raw_points={raw_points} fields={fields}"
        )
        print(f"{name} sample={idx} {_point_stats(pc)}")
        time.sleep(0.2)

    print(f"{name} success={ok}/{samples}")


def main():
    agibot_gdk.gdk_init()
    lidar = agibot_gdk.Lidar()
    time.sleep(1.0)
    try:
        _sample_lidar(lidar, agibot_gdk.LidarType.kLidarFront, "front", 3)
        _sample_lidar(lidar, agibot_gdk.LidarType.kLidarBack, "back", 3)
    finally:
        try:
            lidar.close_lidar()
        except Exception as exc:
            print(f"close_lidar_error={type(exc).__name__}: {exc}")
        agibot_gdk.gdk_release()


if __name__ == "__main__":
    main()
