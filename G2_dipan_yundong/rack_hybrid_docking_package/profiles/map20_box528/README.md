# map20_box528 Profile

This directory freezes the current validated map20 seven-rods site data.

Use it as the reference profile when creating a new site:

```bash
python3 rack_hybrid_docking_package/validate_site_profile.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json
```

The profile contains:

- `industrial_station_config.json`: the four map stations for map20.
- `calibration_records/rod01_grab_pose_latest.json` through `rod07_grab_pose_latest.json`: per-rod grab poses.
- `calibration_records/rod07_place_*_latest.json`: the active shared place pose chain.
- `profile.json`: the manifest tying the map, station config, grab files, place files, tuned parameters, robot host, and evidence logs together.

For a new map/site, copy this directory, rename it, update `profile.json`, recapture the four station poses, recapture the seven grab poses, validate, then run a single-rod smoke before running rods 1-7.
