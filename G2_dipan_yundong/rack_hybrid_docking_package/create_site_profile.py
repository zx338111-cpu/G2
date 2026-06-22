#!/usr/bin/env python3
"""Create a blank site profile from an existing validated profile.

The new profile intentionally does not copy live station/grab/place calibration
JSON files. It creates a directory scaffold and paths that the field calibration
scripts can fill later, so a new map cannot accidentally inherit old map20
points and look validated.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import sys
from typing import Any


PACKAGE_DIR = Path(__file__).resolve().parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from site_profile import DEFAULT_PROFILE, SiteProfileError, load_site_profile


REQUIRED_STATIONS = ("HOME_SAFE", "GRAB_PRE", "PLACE_PRE", "RECOVERY_SAFE")
GENERIC_PLACE_SEQUENCE = {
    "waist": "calibration_records/place_waist_latest.json",
    "above_arm": "calibration_records/place_above_arm_latest.json",
    "transition_arm": "calibration_records/place_transition_arm_latest.json",
    "transition2_arm": "calibration_records/place_transition2_arm_latest.json",
    "final_arm_reference": "calibration_records/place_final_arm_latest.json",
}


def write_json(path: Path, data: Any) -> None:
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def slug(value: str) -> str:
    """Return a conservative directory-safe site name."""

    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    cleaned = cleaned.strip("._-")
    if not cleaned:
        raise ValueError("site name cannot be empty after sanitizing")
    return cleaned


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site", required=True, help="New site/profile name, for example map21_box528")
    parser.add_argument("--map-id", required=True, type=int, help="New robot map id")
    parser.add_argument("--from-profile", default=str(DEFAULT_PROFILE), help="Template profile JSON or directory")
    parser.add_argument("--output-root", default=str(PACKAGE_DIR / "profiles"), help="Directory that will contain the new site")
    parser.add_argument("--host", default="", help="Robot SSH host for the new profile; defaults to template host")
    parser.add_argument("--remote-dir", default="", help="Robot workspace for the new profile; defaults to template remote_dir")
    parser.add_argument("--description", default="", help="Optional profile description")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.map_id <= 0:
        raise SystemExit("--map-id must be positive")
    try:
        template_file, _template_dir, template = load_site_profile(args.from_profile)
        site_name = slug(args.site)
    except (SiteProfileError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc

    output_root = Path(args.output_root).resolve()
    site_dir = output_root / site_name
    if site_dir.exists():
        raise SystemExit(f"refusing to overwrite existing profile directory: {site_dir}")

    robot = template.get("robot") if isinstance(template.get("robot"), dict) else {}
    host = args.host or str(robot.get("host") or "")
    remote_dir = args.remote_dir or str(robot.get("remote_dir") or "")

    site_dir.mkdir(parents=True)
    calibration_dir = site_dir / "calibration_records"
    calibration_dir.mkdir()

    station_config = {
        "map_id": args.map_id,
        "stations": {station: {} for station in REQUIRED_STATIONS},
        "arrival": template.get("arrival")
        or {
            "xy_tolerance_m": 0.08,
            "yaw_tolerance_deg": 3.0,
            "timeout_s": 60.0,
            "poll_interval_s": 0.5,
            "stopped_speed_mps": 0.02,
        },
        "safety": template.get("safety")
        or {
            "require_charge_plug_unplugged": True,
            "max_charge_input_current_a": 0.5,
            "require_motion_control_error_zero": True,
            "require_pnc_idle_before_navigation": True,
        },
    }
    # Template profiles keep arrival/safety under station_config, not at the
    # profile root. Prefer those fields when they exist in the template config.
    template_station_config = template.get("station_config")
    if isinstance(template_station_config, str):
        template_config_path = template_file.parent / template_station_config
        if template_config_path.exists():
            try:
                template_config = json.loads(template_config_path.read_text(encoding="utf-8"))
                if isinstance(template_config.get("arrival"), dict):
                    station_config["arrival"] = template_config["arrival"]
                if isinstance(template_config.get("safety"), dict):
                    station_config["safety"] = template_config["safety"]
            except json.JSONDecodeError:
                pass

    profile = {
        "schema": "g2_industrial_cell_site_profile_v1",
        "site_name": site_name,
        "description": args.description or f"Blank profile for map {args.map_id}; station and calibration points must be captured.",
        "created_at_local": datetime.now().strftime("%Y%m%d_%H%M%S"),
        "created_from_profile": str(template_file),
        "status": "NEEDS_FIELD_CALIBRATION",
        "map_id": args.map_id,
        "robot": {
            "host": host,
            "remote_dir": remote_dir,
        },
        "station_config": "industrial_station_config.json",
        "rod_count": int(template.get("rod_count") or 7),
        "grab_poses": {
            str(index): f"calibration_records/rod{index:02d}_grab_pose_latest.json"
            for index in range(1, int(template.get("rod_count") or 7) + 1)
        },
        "place_sequence": GENERIC_PLACE_SEQUENCE,
        "field_records": {},
        "tuned": template.get("tuned") or {},
        "runner": {
            "script": "rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py",
            "validated_command": "",
            "last_verified_final_state": None,
            "evidence_logs": {},
        },
        "handoff_docs": [
            "rack_hybrid_docking_package/NEW_SITE_REPLICATION_GUIDE.md",
        ],
    }

    write_json(site_dir / "industrial_station_config.json", station_config)
    write_json(site_dir / "profile.json", profile)
    (calibration_dir / "README.md").write_text(
        "# Pending Calibration Records\n\n"
        "This directory is intentionally blank except for this README.\n\n"
        "Capture station poses first, then capture rod01 through rod07 grab poses,\n"
        "then create or copy validated place pose files for this site.\n"
        "Do not copy map20 latest files here unless they have been validated in the new scene.\n",
        encoding="utf-8",
    )
    (site_dir / "README.md").write_text(
        f"# {site_name} Profile\n\n"
        "This profile was created as a blank field-calibration scaffold.\n\n"
        "Next steps:\n\n"
        "1. Capture HOME_SAFE, GRAB_PRE, PLACE_PRE, and RECOVERY_SAFE with calibrate_station_from_current_pose.py --profile.\n"
        "2. Capture rod01 through rod07 grab poses with capture_grab_calibration_point.py --profile.\n"
        "3. Add validated place pose files listed in profile.json place_sequence.\n"
        "4. Run validate_site_profile.py until it reports ok=true.\n"
        "5. Run run_site_7_rods_live.py --profile ... --preflight-only before any live motion.\n",
        encoding="utf-8",
    )

    print(
        json.dumps(
            {
                "event": "site_profile_created",
                "site": site_name,
                "map_id": args.map_id,
                "profile": str(site_dir / "profile.json"),
                "station_config": str(site_dir / "industrial_station_config.json"),
                "calibration_dir": str(calibration_dir),
                "status": "NEEDS_FIELD_CALIBRATION",
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
