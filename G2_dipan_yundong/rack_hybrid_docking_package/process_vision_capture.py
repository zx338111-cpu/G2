#!/usr/bin/env python3
"""Read-only multi-camera capture for pick/place process review.

This module is intentionally passive: it initializes GDK, reads camera frames,
robot state, and TF metadata, then writes files under a dataset directory.  It
does not command chassis, arm, waist, or gripper motion.

The seven-rods runner can call this before and after each local pick/place step
when ``--vision-capture`` is enabled.  The resulting dataset is meant for later
AI/vision feasibility analysis and model training.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import signal
import time
from typing import Any

try:
    from .g2_primitives.gdk_context import gdk_session
except ImportError:
    from g2_primitives.gdk_context import gdk_session


DEFAULT_CAMERAS = (
    "head_stereo_left",
    "head_color",
    "head_depth",
    "hand_left_color",
    "hand_right_color",
)

CAMERA_ENUMS = {
    "head_color": "kHeadColor",
    "head_depth": "kHeadDepth",
    "head_stereo_left": "kHeadStereoLeft",
    "head_stereo_right": "kHeadStereoRight",
    "hand_left_color": "kHandLeftColor",
    "hand_right_color": "kHandRightColor",
}

SENSOR_EXTRINSICS = (
    "kHeadRGBDToHeadLink3",
    "kHeadDepthToHeadColor",
    "kHeadLeftStereoToHeadLink3",
    "kHeadRightStereoToHeadLink3",
    "kHeadLeftStereoToHeadRightStereo",
    "kLeftHandRGBDToArmLEndLink",
    "kRightHandRGBDToArmREndLink",
)

BASE_FRAMES = (
    "head_link3",
    "arm_l_end_link",
    "arm_r_end_link",
    "gripper_l_camera_link",
    "gripper_r_camera_link",
    "gripper_l_center_link",
    "gripper_r_center_link",
    "base_link",
)


def jsonable(value: Any) -> Any:
    """Convert common GDK/Python objects into JSON-safe values."""

    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [jsonable(item) for item in value]
    if hasattr(value, "translation") and hasattr(value, "rotation"):
        return {
            "translation": {
                "x": float(value.translation.x),
                "y": float(value.translation.y),
                "z": float(value.translation.z),
            },
            "rotation": {
                "x": float(value.rotation.x),
                "y": float(value.rotation.y),
                "z": float(value.rotation.z),
                "w": float(value.rotation.w),
            },
        }
    return repr(value)


def safe_name(value: str) -> str:
    """Make a short filesystem-safe label."""

    value = value.strip() or "unknown"
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "unknown"


def parse_camera_list(value: str | None) -> tuple[str, ...]:
    """Parse comma-separated camera names."""

    if not value:
        return DEFAULT_CAMERAS
    cameras = tuple(safe_name(item).lower() for item in value.split(",") if item.strip())
    unknown = [camera for camera in cameras if camera not in CAMERA_ENUMS]
    if unknown:
        raise ValueError(f"unknown vision capture cameras {unknown}; supported={sorted(CAMERA_ENUMS)}")
    return cameras


def read_intrinsics(camera: Any, agibot_gdk: Any, camera_names: tuple[str, ...]) -> dict[str, Any]:
    """Read camera intrinsics where supported."""

    intrinsics: dict[str, Any] = {}
    for name in camera_names:
        enum_name = CAMERA_ENUMS[name]
        camera_type = getattr(agibot_gdk.CameraType, enum_name)
        try:
            intr = camera.get_camera_intrinsic(camera_type)
            intrinsics[name] = {
                "intrinsic": [float(item) for item in getattr(intr, "intrinsic", [])],
                "distortion": [float(item) for item in getattr(intr, "distortion", [])],
            }
        except Exception as exc:
            intrinsics[name] = {"error": f"{type(exc).__name__}: {exc}"}
    return intrinsics


def read_tf(tf: Any, agibot_gdk: Any) -> dict[str, Any]:
    """Read useful fixed and live transforms for later coordinate work."""

    payload: dict[str, Any] = {"base_frames": {}, "sensor_extrinsics": {}}
    try:
        payload["frame_names"] = list(tf.get_all_frame_names())
    except Exception as exc:
        payload["frame_names_error"] = f"{type(exc).__name__}: {exc}"

    for frame in BASE_FRAMES:
        try:
            payload["base_frames"][frame] = jsonable(tf.get_tf_from_base_link(frame))
        except Exception as exc:
            payload["base_frames"][frame] = {"error": f"{type(exc).__name__}: {exc}"}

    for enum_name in SENSOR_EXTRINSICS:
        try:
            enum_value = getattr(agibot_gdk.SensorExtrinsicType, enum_name)
            payload["sensor_extrinsics"][enum_name] = jsonable(tf.get_tf_from_sensor(enum_value))
        except Exception as exc:
            payload["sensor_extrinsics"][enum_name] = {"error": f"{type(exc).__name__}: {exc}"}
    return payload


def save_depth_frame(
    *,
    image: Any,
    path_prefix: Path,
    make_visualization: bool,
) -> dict[str, Any]:
    """Save a Z16 depth image as NPY when numpy is available."""

    result: dict[str, Any] = {}
    try:
        import numpy as np
    except Exception as exc:
        raw_path = path_prefix.with_suffix(".bin")
        raw_path.write_bytes(bytes(image.data))
        result["raw_file"] = str(raw_path)
        result["warning"] = f"numpy unavailable; saved raw bytes only: {type(exc).__name__}: {exc}"
        return result

    arr = np.frombuffer(image.data, dtype=np.uint16).reshape((int(image.height), int(image.width)))
    npy_path = path_prefix.with_suffix(".npy")
    np.save(npy_path, arr)
    result["raw_file"] = str(npy_path)

    valid = arr[arr > 0]
    result["depth_mm_min"] = int(valid.min()) if valid.size else None
    result["depth_mm_median"] = float(np.median(valid)) if valid.size else None
    result["depth_mm_max"] = int(valid.max()) if valid.size else None

    if make_visualization:
        try:
            import cv2

            vis = np.zeros_like(arr, dtype=np.uint8)
            if valid.size:
                lo, hi = np.percentile(valid, [2, 98])
                if hi > lo:
                    vis = np.clip((arr.astype(np.float32) - lo) / (hi - lo) * 255.0, 0, 255).astype(np.uint8)
            color = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
            vis_path = path_prefix.with_name(path_prefix.name + "_vis.jpg")
            cv2.imwrite(str(vis_path), color)
            result["visualization_file"] = str(vis_path)
        except Exception as exc:
            result["visualization_error"] = f"{type(exc).__name__}: {exc}"
    return result


def save_camera_frame(
    *,
    camera: Any,
    agibot_gdk: Any,
    camera_name: str,
    output_dir: Path,
    timeout_ms: float,
    make_depth_visualization: bool,
    file_prefix: str = "",
) -> dict[str, Any]:
    """Read and save one camera frame."""

    enum_name = CAMERA_ENUMS[camera_name]
    camera_type = getattr(agibot_gdk.CameraType, enum_name)
    try:
        image = camera.get_latest_image(camera_type, timeout_ms)
    except Exception as exc:
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    if image is None:
        return {"ok": False, "error": "no image"}

    payload = {
        "ok": True,
        "width": int(image.width),
        "height": int(image.height),
        "encoding": str(image.encoding),
        "color_format": str(image.color_format),
        "bit_depth": int(image.bit_depth),
        "timestamp_ns": int(image.timestamp_ns),
        "bytes": len(image.data) if hasattr(image, "data") else None,
    }

    file_stem = f"{safe_name(file_prefix)}_{camera_name}" if file_prefix else camera_name
    path_prefix = output_dir / file_stem
    encoding = str(image.encoding)
    color_format = str(image.color_format)
    try:
        if "JPEG" in encoding:
            path = path_prefix.with_suffix(".jpg")
            path.write_bytes(bytes(image.data))
            payload["file"] = str(path)
        elif "PNG" in encoding:
            path = path_prefix.with_suffix(".png")
            path.write_bytes(bytes(image.data))
            payload["file"] = str(path)
        elif "Z16" in color_format or "GRAY16" in color_format:
            payload.update(
                save_depth_frame(
                    image=image,
                    path_prefix=path_prefix,
                    make_visualization=make_depth_visualization,
                )
            )
        else:
            path = path_prefix.with_suffix(".bin")
            path.write_bytes(bytes(image.data))
            payload["file"] = str(path)
    except Exception as exc:
        payload["ok"] = False
        payload["save_error"] = f"{type(exc).__name__}: {exc}"
    return payload


def read_robot_state(robot: Any) -> dict[str, Any]:
    """Read robot state fields that help correlate images with arm posture."""

    payload: dict[str, Any] = {}
    for key, getter in (
        ("joint_states", robot.get_joint_states),
        ("whole_body_status", robot.get_whole_body_status),
        ("motion_control_status", robot.get_motion_control_status),
        ("chassis_power_state", robot.get_chassis_power_state),
    ):
        try:
            payload[key] = jsonable(getter())
        except Exception as exc:
            payload[key] = {"error": f"{type(exc).__name__}: {exc}"}
    return payload


def capture_process_vision_snapshot(
    *,
    output_root: str | Path,
    site: str,
    profile: str | Path | None,
    phase: str,
    rod_index: int,
    step_index: int,
    step_label: str,
    moment: str,
    step_kind: str = "",
    cameras: tuple[str, ...] = DEFAULT_CAMERAS,
    timeout_ms: float = 1000.0,
    make_depth_visualization: bool = True,
    note: str = "",
) -> dict[str, Any]:
    """Capture a timestamped process snapshot and return its manifest."""

    stamp = time.strftime("%Y%m%d_%H%M%S")
    monotonic_ns = time.monotonic_ns()
    folder = (
        Path(output_root)
        / safe_name(site)
        / safe_name(phase)
        / f"rod{rod_index:02d}"
        / f"{step_index:02d}_{safe_name(step_label)}_{safe_name(moment)}_{stamp}_{monotonic_ns}"
    )
    folder.mkdir(parents=True, exist_ok=True)

    manifest: dict[str, Any] = {
        "schema": "g2_pick_place_process_vision_snapshot_v1",
        "created_at": stamp,
        "monotonic_ns": monotonic_ns,
        "site": site,
        "profile": None if profile is None else str(profile),
        "phase": phase,
        "rod_index": rod_index,
        "step_index": step_index,
        "step_label": step_label,
        "step_kind": step_kind,
        "moment": moment,
        "note": note,
        "output_dir": str(folder),
        "cameras": {},
        "robot_state": {},
        "tf": {},
        "intrinsics": {},
    }

    with gdk_session() as agibot_gdk:
        camera = agibot_gdk.Camera()
        robot = agibot_gdk.Robot()
        tf = agibot_gdk.TF()
        time.sleep(0.2)

        for camera_name in cameras:
            manifest["cameras"][camera_name] = save_camera_frame(
                camera=camera,
                agibot_gdk=agibot_gdk,
                camera_name=camera_name,
                output_dir=folder,
                timeout_ms=timeout_ms,
                make_depth_visualization=make_depth_visualization,
            )

        manifest["intrinsics"] = read_intrinsics(camera, agibot_gdk, cameras)
        manifest["tf"] = read_tf(tf, agibot_gdk)
        manifest["robot_state"] = read_robot_state(robot)

    manifest_path = folder / "manifest.json"
    manifest["manifest_file"] = str(manifest_path)
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return manifest


def write_sequence_manifest(folder: Path, manifest: dict[str, Any]) -> Path:
    """Persist the rolling sequence manifest after every captured frame."""

    manifest_path = folder / "sequence_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return manifest_path


def capture_process_vision_sequence(
    *,
    output_root: str | Path,
    site: str,
    profile: str | Path | None,
    phase: str,
    rod_index: int,
    cameras: tuple[str, ...] = DEFAULT_CAMERAS,
    timeout_ms: float = 1000.0,
    interval_s: float = 1.0,
    duration_s: float = 0.0,
    stop_file: str | Path | None = None,
    make_depth_visualization: bool = True,
    note: str = "",
) -> dict[str, Any]:
    """Continuously sample process images until duration, stop file, or SIGTERM.

    This is still read-only.  It samples camera frames at a controlled interval
    instead of trying to consume every hardware frame, which keeps disk and CPU
    usage predictable during live robot motion.
    """

    if interval_s <= 0:
        raise ValueError("interval_s must be positive")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    monotonic_ns = time.monotonic_ns()
    folder = (
        Path(output_root)
        / safe_name(site)
        / safe_name(phase)
        / f"rod{rod_index:02d}"
        / f"sequence_{stamp}_{monotonic_ns}"
    )
    folder.mkdir(parents=True, exist_ok=True)
    stop_path = None if stop_file is None else Path(stop_file)
    started = time.monotonic()
    stop_requested = {"value": False}

    def request_stop(signum: int, _frame: Any) -> None:
        stop_requested["value"] = True

    old_sigterm = signal.getsignal(signal.SIGTERM)
    old_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    manifest: dict[str, Any] = {
        "schema": "g2_pick_place_process_vision_sequence_v1",
        "created_at": stamp,
        "monotonic_ns": monotonic_ns,
        "site": site,
        "profile": None if profile is None else str(profile),
        "phase": phase,
        "rod_index": rod_index,
        "output_dir": str(folder),
        "interval_s": interval_s,
        "duration_s": duration_s,
        "stop_file": None if stop_path is None else str(stop_path),
        "note": note,
        "frames": [],
        "tf": {},
        "intrinsics": {},
    }

    try:
        with gdk_session() as agibot_gdk:
            camera = agibot_gdk.Camera()
            robot = agibot_gdk.Robot()
            tf = agibot_gdk.TF()
            time.sleep(0.2)
            manifest["intrinsics"] = read_intrinsics(camera, agibot_gdk, cameras)
            manifest["tf"] = read_tf(tf, agibot_gdk)
            write_sequence_manifest(folder, manifest)

            frame_index = 0
            while True:
                if stop_requested["value"]:
                    manifest["stop_reason"] = "signal"
                    break
                if stop_path is not None and stop_path.exists():
                    manifest["stop_reason"] = "stop_file"
                    break
                elapsed_s = time.monotonic() - started
                if duration_s > 0 and elapsed_s >= duration_s:
                    manifest["stop_reason"] = "duration"
                    break

                frame_stamp = time.strftime("%Y%m%d_%H%M%S")
                frame_mono_ns = time.monotonic_ns()
                frame_dir = folder / f"frame_{frame_index:05d}_{frame_stamp}_{frame_mono_ns}"
                frame_dir.mkdir(parents=True, exist_ok=True)
                frame_payload: dict[str, Any] = {
                    "index": frame_index,
                    "created_at": frame_stamp,
                    "monotonic_ns": frame_mono_ns,
                    "elapsed_s": round(elapsed_s, 3),
                    "output_dir": str(frame_dir),
                    "cameras": {},
                    "robot_state": read_robot_state(robot),
                }
                for camera_name in cameras:
                    frame_payload["cameras"][camera_name] = save_camera_frame(
                        camera=camera,
                        agibot_gdk=agibot_gdk,
                        camera_name=camera_name,
                        output_dir=frame_dir,
                        timeout_ms=timeout_ms,
                        make_depth_visualization=make_depth_visualization,
                    )
                manifest["frames"].append(frame_payload)
                manifest["frame_count"] = len(manifest["frames"])
                write_sequence_manifest(folder, manifest)
                frame_index += 1

                next_deadline = started + frame_index * interval_s
                while True:
                    if stop_requested["value"]:
                        break
                    if stop_path is not None and stop_path.exists():
                        break
                    sleep_s = min(0.1, next_deadline - time.monotonic())
                    if sleep_s <= 0:
                        break
                    time.sleep(sleep_s)
            manifest.setdefault("stop_reason", "completed")
    finally:
        signal.signal(signal.SIGTERM, old_sigterm)
        signal.signal(signal.SIGINT, old_sigint)

    manifest["finished_at"] = time.strftime("%Y%m%d_%H%M%S")
    manifest["elapsed_s"] = round(time.monotonic() - started, 3)
    manifest_path = write_sequence_manifest(folder, manifest)
    manifest["manifest_file"] = str(manifest_path)
    write_sequence_manifest(folder, manifest)
    return manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", default="logs/vision_dataset")
    parser.add_argument("--site", default="manual")
    parser.add_argument("--profile", default="")
    parser.add_argument("--phase", default="manual")
    parser.add_argument("--rod-index", type=int, default=0)
    parser.add_argument("--step-index", type=int, default=0)
    parser.add_argument("--step-label", default="manual")
    parser.add_argument("--step-kind", default="")
    parser.add_argument("--moment", default="manual")
    parser.add_argument("--sequence", action="store_true")
    parser.add_argument("--cameras", default=",".join(DEFAULT_CAMERAS))
    parser.add_argument("--timeout-ms", type=float, default=1000.0)
    parser.add_argument("--interval-s", type=float, default=1.0)
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--stop-file", default="")
    parser.add_argument("--no-depth-visualization", action="store_true")
    parser.add_argument("--note", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cameras = parse_camera_list(args.cameras)
    if args.sequence:
        manifest = capture_process_vision_sequence(
            output_root=args.output_root,
            site=args.site,
            profile=args.profile or None,
            phase=args.phase,
            rod_index=args.rod_index,
            cameras=cameras,
            timeout_ms=args.timeout_ms,
            interval_s=args.interval_s,
            duration_s=args.duration_s,
            stop_file=args.stop_file or None,
            make_depth_visualization=not args.no_depth_visualization,
            note=args.note,
        )
        print(
            json.dumps(
                {
                    "event": "process_vision_sequence_captured",
                    "output_dir": manifest.get("output_dir"),
                    "manifest_file": manifest.get("manifest_file"),
                    "frame_count": manifest.get("frame_count", 0),
                    "stop_reason": manifest.get("stop_reason"),
                    "elapsed_s": manifest.get("elapsed_s"),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
    else:
        manifest = capture_process_vision_snapshot(
            output_root=args.output_root,
            site=args.site,
            profile=args.profile or None,
            phase=args.phase,
            rod_index=args.rod_index,
            step_index=args.step_index,
            step_label=args.step_label,
            moment=args.moment,
            step_kind=args.step_kind,
            cameras=cameras,
            timeout_ms=args.timeout_ms,
            make_depth_visualization=not args.no_depth_visualization,
            note=args.note,
        )
        print(json.dumps({"event": "process_vision_snapshot_captured", **manifest}, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
