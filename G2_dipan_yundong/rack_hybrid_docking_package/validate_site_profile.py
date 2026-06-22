#!/usr/bin/env python3
"""Validate a G2 industrial-cell site profile.

The validator is intentionally local and no-motion. It checks that a profile
directory contains the station config, seven grab poses, shared place pose
chain, and tuned values needed by the seven-rods runner.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_PROFILE = PACKAGE_DIR / "profiles" / "map20_box528" / "profile.json"

ARM_KEYS = [
    "idx21_arm_l_joint1",
    "idx22_arm_l_joint2",
    "idx23_arm_l_joint3",
    "idx24_arm_l_joint4",
    "idx25_arm_l_joint5",
    "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
    "idx61_arm_r_joint1",
    "idx62_arm_r_joint2",
    "idx63_arm_r_joint3",
    "idx64_arm_r_joint4",
    "idx65_arm_r_joint5",
    "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]

WAIST_KEYS = [
    "idx01_body_joint1",
    "idx02_body_joint2",
    "idx03_body_joint3",
    "idx04_body_joint4",
    "idx05_body_joint5",
]

REQUIRED_STATIONS = ("HOME_SAFE", "GRAB_PRE", "PLACE_PRE", "RECOVERY_SAFE")
REQUIRED_PLACE_KEYS = ("waist", "above_arm", "transition_arm", "transition2_arm")

TUNED_RANGES = {
    "arm_joint_speed_radps": (0.01, 0.5),
    "fast_safe_arm_joint_speed_radps": (0.01, 0.5),
    "waist_joint_speed_radps": (0.01, 0.8),
    "arm_settle_s": (0.0, 5.0),
    "waist_settle_s": (0.0, 5.0),
    "offset_settle_s": (0.0, 5.0),
    "offset_max_abs_m": (0.01, 0.35),
    "grab_final_stop_mm": (100, 1000),
    "grab_final_brake_margin_mm": (0, 300),
    "grab_final_speed_mps": (0.01, 0.5),
    "place_final_stop_mm": (100, 1000),
    "place_final_brake_margin_mm": (0, 300),
    "place_final_speed_mps": (0.01, 0.5),
    "pick_down_z_m": (-0.15, 0.15),
    "pick_back_x_m": (-0.4, -0.02),
    "pick_back_down_x_m": (-0.25, 0.0),
    "place_final_before_open_x_m": (-0.15, 0.15),
    "place_final_before_open_z_m": (-0.15, 0.15),
    "place_pull_x_m": (-0.4, -0.02),
    "place_pull_back_down_x_m": (-0.25, 0.05),
    "place_pull_back_down_z_m": (-0.15, 0.05),
    "place_pull_drop_after_x_m": (-0.25, 0.05),
    "place_pull_drop_z_m": (-0.15, 0.05),
    "local_retreat_m": (0.05, 1.5),
    "local_retreat_speed_mps": (0.02, 0.5),
    "fine_position_max_duration_s": (5.0, 120.0),
    "refine_yaw_tolerance_deg": (0.1, 10.0),
    "refine_yaw_max_error_deg": (1.0, 30.0),
    "refine_yaw_timeout_s": (1.0, 30.0),
}


class ProfileError(Exception):
    """Raised when a profile is malformed before full validation can continue."""


def load_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ProfileError(f"missing file: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ProfileError(f"invalid JSON in {path}: {exc}") from exc


def profile_path(value: str | Path) -> Path:
    path = Path(value)
    if path.is_dir():
        return path / "profile.json"
    return path


def resolve(profile_dir: Path, rel_path: str) -> Path:
    path = Path(rel_path)
    if path.is_absolute():
        return path
    return profile_dir / path


def is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require_numeric_fields(data: dict[str, Any], keys: list[str], path: Path, errors: list[str]) -> None:
    missing = [key for key in keys if key not in data]
    if missing:
        errors.append(f"{path}: missing keys {missing}")
        return
    bad = [key for key in keys if not is_number(data.get(key))]
    if bad:
        errors.append(f"{path}: non-numeric keys {bad}")


def validate_station_config(
    *,
    profile: dict[str, Any],
    profile_dir: Path,
    errors: list[str],
    warnings: list[str],
) -> dict[str, Any] | None:
    rel_path = profile.get("station_config")
    if not isinstance(rel_path, str):
        errors.append("profile.station_config must be a relative path string")
        return None

    path = resolve(profile_dir, rel_path)
    config = load_json(path)
    expected_map_id = profile.get("map_id")
    if config.get("map_id") != expected_map_id:
        errors.append(f"{path}: map_id={config.get('map_id')!r} does not match profile map_id={expected_map_id!r}")

    stations = config.get("stations")
    if not isinstance(stations, dict):
        errors.append(f"{path}: stations must be an object")
        return config

    for station in REQUIRED_STATIONS:
        value = stations.get(station)
        if not isinstance(value, dict):
            errors.append(f"{path}: missing station {station}")
            continue
        position = value.get("position")
        orientation = value.get("orientation")
        if not isinstance(position, dict):
            errors.append(f"{path}: station {station} missing position object")
        else:
            require_numeric_fields(position, ["x", "y", "z"], path, errors)
        if not isinstance(orientation, dict):
            errors.append(f"{path}: station {station} missing orientation object")
        else:
            require_numeric_fields(orientation, ["x", "y", "z", "w"], path, errors)
            if all(is_number(orientation.get(key)) for key in ("x", "y", "z", "w")):
                norm = sum(float(orientation[key]) ** 2 for key in ("x", "y", "z", "w")) ** 0.5
                if not 0.8 <= norm <= 1.2:
                    warnings.append(f"{path}: station {station} quaternion norm looks unusual: {norm:.3f}")
    return config


def validate_pose_file(
    *,
    profile_dir: Path,
    rel_path: str,
    required_keys: list[str],
    label: str,
    errors: list[str],
    seen_files: dict[str, str],
) -> None:
    path = resolve(profile_dir, rel_path)
    try:
        data = load_json(path)
    except ProfileError as exc:
        errors.append(str(exc))
        return
    if not isinstance(data, dict):
        errors.append(f"{path}: expected JSON object for {label}")
        return
    require_numeric_fields(data, required_keys, path, errors)
    if path.exists():
        seen_files[rel_path] = sha256(path)


def validate_calibration_files(
    *,
    profile: dict[str, Any],
    profile_dir: Path,
    errors: list[str],
) -> dict[str, str]:
    seen_files: dict[str, str] = {}
    rod_count = profile.get("rod_count")
    if not isinstance(rod_count, int) or rod_count <= 0:
        errors.append("profile.rod_count must be a positive integer")
        rod_count = 7

    grab_poses = profile.get("grab_poses")
    if not isinstance(grab_poses, dict):
        errors.append("profile.grab_poses must be an object")
        grab_poses = {}

    for index in range(1, rod_count + 1):
        rel_path = grab_poses.get(str(index))
        if not isinstance(rel_path, str):
            errors.append(f"profile.grab_poses.{index} must be a path")
            continue
        validate_pose_file(
            profile_dir=profile_dir,
            rel_path=rel_path,
            required_keys=WAIST_KEYS + ARM_KEYS,
            label=f"rod {index} grab pose",
            errors=errors,
            seen_files=seen_files,
        )

    place_sequence = profile.get("place_sequence")
    if not isinstance(place_sequence, dict):
        errors.append("profile.place_sequence must be an object")
        place_sequence = {}

    for key in REQUIRED_PLACE_KEYS:
        rel_path = place_sequence.get(key)
        if not isinstance(rel_path, str):
            errors.append(f"profile.place_sequence.{key} must be a path")
            continue
        required = WAIST_KEYS if key == "waist" else ARM_KEYS
        validate_pose_file(
            profile_dir=profile_dir,
            rel_path=rel_path,
            required_keys=required,
            label=f"place {key}",
            errors=errors,
            seen_files=seen_files,
        )

    for key, rel_path in place_sequence.items():
        if key in REQUIRED_PLACE_KEYS or not isinstance(rel_path, str):
            continue
        path = resolve(profile_dir, rel_path)
        if not path.exists():
            errors.append(f"profile.place_sequence.{key} points to missing file: {path}")
        else:
            seen_files[rel_path] = sha256(path)

    for key, rel_path in (profile.get("field_records") or {}).items():
        if not isinstance(rel_path, str):
            errors.append(f"profile.field_records.{key} must be a path")
            continue
        path = resolve(profile_dir, rel_path)
        if not path.exists():
            errors.append(f"profile.field_records.{key} points to missing file: {path}")
        else:
            seen_files[rel_path] = sha256(path)
    return seen_files


def validate_tuned(profile: dict[str, Any], errors: list[str], warnings: list[str]) -> None:
    tuned = profile.get("tuned")
    if not isinstance(tuned, dict):
        errors.append("profile.tuned must be an object")
        return

    for key, (low, high) in TUNED_RANGES.items():
        value = tuned.get(key)
        if not is_number(value):
            errors.append(f"profile.tuned.{key} must be numeric")
            continue
        if not float(low) <= float(value) <= float(high):
            errors.append(f"profile.tuned.{key}={value!r} outside expected range [{low}, {high}]")

    refine_stations = tuned.get("refine_yaw_stations")
    if refine_stations is not None:
        if not isinstance(refine_stations, list) or not all(isinstance(item, str) for item in refine_stations):
            errors.append("profile.tuned.refine_yaw_stations must be a list of station names")
        else:
            unknown = sorted(set(refine_stations) - set(REQUIRED_STATIONS))
            if unknown:
                warnings.append(f"profile.tuned.refine_yaw_stations contains non-standard station names: {unknown}")


def validate_profile(profile_file: str | Path) -> dict[str, Any]:
    profile_file = profile_path(profile_file).resolve()
    profile_dir = profile_file.parent
    errors: list[str] = []
    warnings: list[str] = []
    profile = load_json(profile_file)

    if not isinstance(profile, dict):
        raise ProfileError(f"{profile_file}: expected JSON object")
    if profile.get("schema") != "g2_industrial_cell_site_profile_v1":
        errors.append("profile.schema must be g2_industrial_cell_site_profile_v1")
    if not isinstance(profile.get("site_name"), str) or not profile["site_name"]:
        errors.append("profile.site_name must be a non-empty string")
    if not isinstance(profile.get("map_id"), int):
        errors.append("profile.map_id must be an integer")

    station_config = validate_station_config(
        profile=profile,
        profile_dir=profile_dir,
        errors=errors,
        warnings=warnings,
    )
    seen_files = validate_calibration_files(profile=profile, profile_dir=profile_dir, errors=errors)
    validate_tuned(profile, errors, warnings)

    robot = profile.get("robot")
    if not isinstance(robot, dict):
        errors.append("profile.robot must be an object")
    else:
        if not isinstance(robot.get("host"), str) or not robot["host"]:
            errors.append("profile.robot.host must be a non-empty string")
        if not isinstance(robot.get("remote_dir"), str) or not robot["remote_dir"]:
            errors.append("profile.robot.remote_dir must be a non-empty string")

    result = {
        "ok": not errors,
        "profile": str(profile_file),
        "site_name": profile.get("site_name"),
        "map_id": profile.get("map_id"),
        "station_count": len((station_config or {}).get("stations", {}) if isinstance(station_config, dict) else {}),
        "calibration_file_count": len(seen_files),
        "errors": errors,
        "warnings": warnings,
        "file_hashes": seen_files,
    }
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default=str(DEFAULT_PROFILE), help="Profile JSON or profile directory")
    parser.add_argument("--json", action="store_true", help="Print only the JSON validation report")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        result = validate_profile(args.profile)
    except ProfileError as exc:
        result = {"ok": False, "profile": str(args.profile), "errors": [str(exc)], "warnings": []}

    if args.json:
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        print(json.dumps({key: value for key, value in result.items() if key != "file_hashes"}, indent=2, ensure_ascii=False))
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
