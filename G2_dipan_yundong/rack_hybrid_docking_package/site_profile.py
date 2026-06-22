"""Helpers for site-profile aware scripts.

The profile helpers are deliberately file-only. They never initialize GDK and
never command robot motion; they only resolve paths inside a profile directory.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_PROFILE = PACKAGE_DIR / "profiles" / "map20_box528" / "profile.json"


class SiteProfileError(Exception):
    """Raised when a profile file is missing or malformed."""


def profile_file_from_arg(value: str | Path) -> Path:
    """Accept either a profile JSON path or a profile directory."""

    path = Path(value)
    if path.is_dir():
        return path / "profile.json"
    return path


def load_site_profile(value: str | Path) -> tuple[Path, Path, dict[str, Any]]:
    """Return ``(profile_file, profile_dir, profile_data)``."""

    profile_file = profile_file_from_arg(value).resolve()
    try:
        profile = json.loads(profile_file.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise SiteProfileError(f"profile not found: {profile_file}") from exc
    except json.JSONDecodeError as exc:
        raise SiteProfileError(f"profile is not valid JSON: {profile_file}: {exc}") from exc
    if not isinstance(profile, dict):
        raise SiteProfileError(f"profile must be a JSON object: {profile_file}")
    return profile_file, profile_file.parent, profile


def resolve_profile_path(profile_dir: Path, rel_path: str) -> Path:
    """Resolve a profile-relative path."""

    path = Path(rel_path)
    if path.is_absolute():
        return path
    return (profile_dir / path).resolve()


def station_config_path(profile_dir: Path, profile: dict[str, Any]) -> Path:
    """Return the station config path declared by a profile."""

    rel_path = profile.get("station_config")
    if not isinstance(rel_path, str) or not rel_path:
        raise SiteProfileError("profile.station_config must be a non-empty path")
    return resolve_profile_path(profile_dir, rel_path)


def calibration_dir_from_profile(profile_dir: Path, profile: dict[str, Any]) -> Path:
    """Return the calibration directory implied by profile grab-pose paths."""

    grab_poses = profile.get("grab_poses")
    if isinstance(grab_poses, dict):
        for rel_path in grab_poses.values():
            if isinstance(rel_path, str) and rel_path:
                return resolve_profile_path(profile_dir, rel_path).parent
    return (profile_dir / "calibration_records").resolve()


def grab_latest_path(profile_dir: Path, profile: dict[str, Any], rod_index: int) -> Path:
    """Return the ``rodXX`` latest grab-pose path from a profile."""

    grab_poses = profile.get("grab_poses")
    key = str(rod_index)
    if not isinstance(grab_poses, dict) or not isinstance(grab_poses.get(key), str):
        return calibration_dir_from_profile(profile_dir, profile) / f"rod{rod_index:02d}_grab_pose_latest.json"
    return resolve_profile_path(profile_dir, grab_poses[key])
