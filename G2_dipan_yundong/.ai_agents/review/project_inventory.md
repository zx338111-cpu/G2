# Project Inventory: G2_dipan_yundong

Task: `task_0001_project_inventory`
Date: 2026-06-22
Role: AI-B Execution Engineer

## Scope and Method

This inventory is based on safe static inspection only:

- Required guard passed before execution.
- Top-level files were written to `.ai_agents/logs/root_files.txt`.
- Directories up to depth 2 were written to `.ai_agents/logs/dirs_depth2.txt`.
- Additional file-name-only inspection was used to classify likely components.
- No source code was modified.
- No robot motion scripts, ROS drivers, hardware controllers, datasets, checkpoints, rosbags, secrets, or credential files were executed or read.

## Repository Shape

Top-level project areas:

- `.ai_agents/`: AI-A / AI-B collaboration protocol, prompts, guard script, task inbox, logs, review outputs, and result JSONs.
- `rack_hybrid_docking_package/`: main industrial rack docking and seven-rods workflow package.
- `tools/`: helper shell tools for robot discovery, RTC audio probing, and tunnel camera access.
- `overlays/`: site/map-specific overlay configuration, including `BOX_528_1_map7_safe_20260622_1340`.
- `logs/`: local dry-run, live-run, checkpoint, report, and analysis artifacts.
- `agi@192.168.168.45/`: copied or mirrored rack docking package snapshot for a robot host/workspace.
- `__pycache__/`: generated Python bytecode cache.
- `.codex/` and `.agents/`: local agent/tooling state.

## Root-Level Files

Planning and context documents:

- `G2_GDK_SECONDARY_DEVELOPMENT_VLA_WORLDMODEL_GUIDE.md`
- `G2_LIVE_ARCHITECTURE_DEEP_DIVE_20260618.md`
- `G2_WEEKLY_REPORT_20260618.md`
- `G2_WEEKLY_REPORT_BRIEF_20260618.md`

Head camera / audio / browser viewer candidates:

- `g2_head_av_viewer.py`
- `g2_head_tunnel_viewer.py`
- `g2_head_webrtc_viewer.py`

Industrial status and diagnostic scripts:

- `industrial_lidar_snapshot.py`
- `industrial_linear_y_diagnostic.py`
- `industrial_status_snapshot.py`

High-risk robot motion or manipulation entrypoints present at root and not executed:

- `industrial_7_rods_one_click.py`
- `industrial_cancel_pnc_task.py`
- `industrial_linear_y_sweep.py`
- `industrial_turn_diagnostic.py`
- `move_arm_by_json_path.py`
- `move_arm_vertical_stack_grab_above.py`
- `move_ee_pose_close_2.py`
- `move_ee_pose_open_2.py`
- `move_ee_relative_offset.py`
- `move_waist_by_json_path.py`
- `offset_move_down.py`

## Main Package: rack_hybrid_docking_package

Likely main docs and handoff files:

- `README.md`
- `NEW_SITE_REPLICATION_GUIDE.md`
- `rack_hybrid_docking_project_handoff.md`
- `rack_hybrid_docking_usage.md`
- `rack_industrial_docking_usage.md`
- `industrial_automation_architecture_review_20260610.md`
- `industrial_three_core_primitives_20260610.md`
- multiple dated live handoff files from 2026-06-10 through 2026-06-16

Likely robot primitive modules:

- `g2_primitives/arm.py`
- `g2_primitives/chassis_motion.py`
- `g2_primitives/ee_offset.py`
- `g2_primitives/gdk_context.py`
- `g2_primitives/gripper.py`
- `g2_primitives/nav.py`
- `g2_primitives/rack.py`
- `g2_primitives/waist.py`

Likely rack docking modules:

- `rack_hybrid_docking.py`
- `rack_hybrid_docking_demo.py`
- `rack_industrial_docking.py`
- `rack_lidar_docking.py`
- `rack_radar_docking.py`
- `rack_retreat_controller.py`

Likely industrial-cell / seven-rods workflow modules:

- `industrial_7_rods_total_controller.py`
- `industrial_cell_7_rods_optimized.py`
- `industrial_cell_7_rods_single_debug.py`
- `industrial_cell_mission_controller.py`
- `industrial_docking_integration_template.py`
- `run_map20_7_rods_live.py`
- `run_map20_7_rods_live.sh`
- `run_site_7_rods_live.py`

Likely calibration and site profile modules:

- `calibrate_direct_place_pose_offset.py`
- `calibrate_station_from_current_pose.py`
- `capture_grab_calibration_point.py`
- `create_site_profile.py`
- `site_profile.py`
- `validate_site_profile.py`
- `industrial_station_config.json`
- `calibration_records/`
- `profiles/`

Likely diagnostics, analysis, and artifact processing:

- `analyze_industrial_cell_run.py`
- `analyze_lateral_active_response.py`
- `analyze_lateral_motion_trace.py`
- `analyze_rack_pose_events.py`
- `industrial_lateral_centering_probe.py`
- `industrial_lateral_motion_trace.py`
- `industrial_rack_pose_roi_sweep.py`
- `industrial_retreat_1m_validation.py`
- `industrial_run_artifacts.py`
- `process_vision_capture.py`
- `gdk_status_utils.py`

System/service artifacts:

- `g2_arm_ethercat_boot_recover.service`
- `g2_arm_ethercat_boot_recover.sh`

## VLA / World Model Signals

Direct VLA/world-model planning appears to be represented mainly by documentation at the repository root:

- `G2_GDK_SECONDARY_DEVELOPMENT_VLA_WORLDMODEL_GUIDE.md`
- `G2_LIVE_ARCHITECTURE_DEEP_DIVE_20260618.md`
- weekly reports from 2026-06-18

No safe shallow inventory evidence showed a dedicated training package, model checkpoint directory, dataset directory, or world-model source module in the allowed scan. This is not proof they do not exist; forbidden directories were intentionally pruned.

## ROS / Robot Middleware Signals

No ROS package markers were identified from the safe shallow inventory, such as `package.xml`, `CMakeLists.txt`, `launch/`, or `rosbags/`.

Robot middleware appears more likely GDK/service-oriented from filenames such as:

- `gdk_status_utils.py`
- `g2_primitives/gdk_context.py`
- `tools/g2_robot_discovery.sh`
- `g2_arm_ethercat_boot_recover.service`

ROS hardware drivers or robot controllers were not started.

## Viewer / AV Components

Likely viewer and head AV files:

- root `g2_head_av_viewer.py`
- root `g2_head_tunnel_viewer.py`
- root `g2_head_webrtc_viewer.py`
- `tools/g2_rtc_audio_probe.sh`
- `tools/g2_rtc_audio_matrix_probe.sh`
- `tools/open_g2_tunnel_camera.sh`

These are likely relevant for camera/audio access and browser-based robot observation.

## Logs and Runtime Artifacts

The repository contains many local artifacts under:

- `logs/`
- `rack_hybrid_docking_package/logs/`

Observed artifact types include:

- `.log`
- `.jsonl`
- checkpoint `.json`
- report `.json`
- analysis `.md`

These appear useful for post-run analysis and dry-run/live-run audit trails. They were listed by filename only during this task.

## Safety Notes for Future Tasks

Do not execute the root motion/manipulation scripts or live-run package entrypoints without explicit user approval and a fresh safety preflight.

Treat these as high-risk until reviewed in a separate task:

- root `move_*` scripts
- root `offset_move_down.py`
- root `industrial_7_rods_one_click.py`
- package `run_*_live.py` and `run_*_live.sh`
- package `industrial_*controller.py`
- package calibration scripts that may capture live robot state or command motion

Continue pruning or avoiding:

- `.env`
- SSH keys
- API keys, tokens, credentials
- `datasets/`
- `dataset/`
- `checkpoints/`
- `rosbags/`

## Unknowns and Recommended Next Inspection Targets

Recommended next safe tasks:

1. Read only the root VLA/world-model docs and summarize the intended data/model workflow.
2. Read `rack_hybrid_docking_package/README.md` and current dated handoff files to identify the stable entrypoints.
3. Inspect `g2_primitives/` source interfaces without executing anything, to map robot abstraction boundaries.
4. Inspect `industrial_run_artifacts.py` and analysis scripts to define log schemas and evaluation metrics.
5. Create a separate explicit task before reading calibration JSON contents, because they may encode live robot poses.

Open questions:

- Whether VLA/world-model code lives outside this checkout or only in planning docs.
- Whether datasets/checkpoints are intentionally absent from this repo or pruned by safety policy.
- Which dated handoff is the current operational source of truth for map20/map7 workflows.
- Which scripts are safe dry-run-only versus capable of robot motion.
