# Real Log Candidate Inventory for VLA Dry-Run Conversion

Task: `task_0008_real_log_candidate_inventory`  
Date: 2026-06-22  
Role: AI-B Execution Engineer

## Scope

This inventory is based on static, low-risk inspection only.

- Candidate paths were generated from `logs/` and `rack_hybrid_docking_package/logs/`.
- Only text-like files below 2 MB were listed: `.log`, `.json`, `.jsonl`, `.md`, and `.txt`.
- No candidate log body was opened beyond the generated candidate path list, because this task did not declare shell commands for reading individual candidate files.
- No robot motion script, ROS driver, controller, dataset, checkpoint, rosbag, secret, or credential path was accessed.

The usefulness classifications below are therefore conservative and filename/schema based. They identify likely next targets for a later explicit conversion task.

## Candidate List Summary

The generated list is stored in:

`.ai_agents/review/real_log_candidates.txt`

Observed candidate families:

- `logs/dryrun_*`: dry-run JSONL/log/checkpoint/report bundles.
- `logs/industrial_cell_mission_rod1_*`: phase-specific industrial-cell navigation logs.
- `rack_hybrid_docking_package/logs/industrial_7_rods_*`: seven-rods run bundles.
- `rack_hybrid_docking_package/logs/live_retreat_*`: live retreat validation bundles.
- `rack_hybrid_docking_package/logs/lateral_motion_trace_*`: motion-trace JSON/JSONL/MD artifacts.
- `rack_hybrid_docking_package/logs/rack_pose_roi_sweep_*`: rack pose analysis artifacts.
- `logs/g2_robot_discovery_20260617.md`: robot discovery/status note.

## Usefulness Classes

### High

High-value candidates are likely to contain several fields needed for VLA dry-run JSONL conversion, especially when a `.jsonl`, `.log`, `_checkpoint.json`, and `_report.json` bundle exists for the same run stem.

| Candidate group | Example paths | Likely extractable signals |
| --- | --- | --- |
| Seven-rods industrial run bundle | `rack_hybrid_docking_package/logs/industrial_7_rods_20260611_031537.jsonl`, `.log`, `_checkpoint.json`, `_report.json` | `task_phase`, `success_label`, `safety_gate`, `robot_state`, `localization_state`, `artifact_refs`; possibly `critic_category` from failures and recovery labels. |
| Guarded dry-run review bundles | `logs/dryrun_review_rod3_guarded_20260612.*`, `logs/dryrun_review_rerun_artifact_gate_20260612.*` | `task_phase`, `safety_gate`, `critic_category`, `success_label`, `artifact_refs`; possibly `robot_state` and `localization_state` if snapshots are present. |
| Automation/shadow dry-run bundles | `logs/dryrun_local_after_automation_patch_20260611.*`, `logs/dryrun_local_shadow_mode_20260611.*`, `logs/dryrun_tail_correction_parse_check.*` | `task_phase`, `safety_gate`, `critic_category`, `artifact_refs`; possible dry-run action validation and parser outcomes. |
| Odom/ultrasonic/lateral guarded dry-run bundles | `logs/dryrun_guarded_auto_odom_local_20260611.*`, `logs/dryrun_disable_auto_odom_local_20260611.*`, `logs/dryrun_guarded_ultrasonic_override_local_20260611.*`, `logs/dryrun_active_lateral_centering_local_20260611.*` | `localization_state`, `safety_gate`, `robot_state`, `critic_category`, `artifact_refs`; possibly task phase and readiness blockers. |
| Retreat validation dry-run bundles | `rack_hybrid_docking_package/logs/dryrun_retreat_1m_validation_20260611_1958.*`, `rack_hybrid_docking_package/logs/local_dryrun_retreat_validation_after_odom_retry.*`, `rack_hybrid_docking_package/logs/local_dryrun_retreat_validation_test.*` | `task_phase`, `localization_state`, `safety_gate`, `success_label`, `robot_state`, `artifact_refs`; useful for stop/recover/nav context. |
| Phase-specific rod1 mission logs | `logs/industrial_cell_mission_rod1_nav_to_grab_to_grab_pre_*.log`, `logs/industrial_cell_mission_rod1_nav_to_place_to_place_pre_*.log`, `logs/industrial_cell_mission_rod1_nav_to_recovery_to_recovery_safe_*.log`, `logs/industrial_cell_mission_rod1_nav_to_home_to_home_safe_*.log` | Strong `task_phase` and `artifact_refs`; likely `success_label` or failure labels; possible `localization_state` and `safety_gate`. |

High-value notes:

- The best first conversion target is a bundle with `.jsonl`, `.log`, `_checkpoint.json`, and `_report.json` for the same stem, because it can map one run into observation/action/safety/critic metrics with traceable artifacts.
- `industrial_7_rods_20260611_031537` is the strongest single candidate by filename because it combines seven-rods task context with JSONL, log, checkpoint, and report artifacts.
- Dry-run bundles are safer first targets than live bundles because their intended purpose already matches the no-motion VLA validation workflow.

### Medium

Medium-value candidates likely contain useful supporting signals, but may be narrower diagnostics rather than full VLA event sources.

| Candidate group | Example paths | Likely extractable signals |
| --- | --- | --- |
| Lateral motion trace artifacts | `rack_hybrid_docking_package/logs/lateral_motion_trace_readonly_20260611_1815.*`, `lateral_motion_trace_readonly_robust_20260611_1835.*`, `lateral_motion_trace_positive_tiny_20260611_1845.*`, `lateral_motion_trace_negative_tiny_20260611_1850.*` | `robot_state`, `localization_state`, `safety_gate`, `artifact_refs`; possibly low-level action or response traces. |
| Rack pose and ROI analysis | `logs/dryrun_rack_pose_analysis_20260611.*`, `rack_hybrid_docking_package/logs/rack_pose_roi_sweep_highsample_20260611_1816.*` | `perception_state`, possible object/pose confidence, `artifact_refs`; likely less direct task/action coverage. |
| Lateral shadow analysis | `logs/dryrun_lateral_shadow_analysis.*`, `logs/dryrun_lateral_shadow_local.*`, `logs/live_rod1_lateral_shadow_20260611_1651.log` | `critic_category`, lateral alignment context, `artifact_refs`; possible `robot_state`/`localization_state`. |
| Hard-min/robust centering reports | `rack_hybrid_docking_package/logs/dryrun_hard_min_lateral_safety_20260611_1905.*`, `dryrun_robust_centering_20260611_1830.*` | `safety_gate`, centering outcome, possible critic/retry labels. |
| Live retreat reports | `rack_hybrid_docking_package/logs/live_retreat_1m_validation_20260611_1907_retry.*`, `live_retreat_1m_validation_20260611_2008.*` | `success_label`, `safety_gate`, `localization_state`, `artifact_refs`; useful after dry-run conversion is proven. |

### Low

Low-value candidates may still support metadata or context, but are not first choices for VLA event conversion.

| Candidate group | Example paths | Reason |
| --- | --- | --- |
| Robot discovery note | `logs/g2_robot_discovery_20260617.md` | Useful for robot identity or network context, but unlikely to contain action or task-phase events. |
| Single diagnostic JSON/log without bundle context | `logs/linear_y_sweep_pilot_20260611_1642.json`, `logs/local_dryrun_place_raise_005_20260615.log`, `logs/local_dryrun_after_rear_precheck_patch_20260615.log` | May be useful, but lower traceability without paired report/checkpoint/JSONL. |
| Analysis-only summaries | `rack_hybrid_docking_package/logs/lateral_motion_trace_tiny_analysis_20260611_1855.*`, `rack_hybrid_docking_package/logs/lateral_active_response_combined_20260611_1808.*` | Good for engineering notes, less direct as event-level JSONL source. |

### Unknown

Unknown candidates require explicit follow-up permission to inspect small text contents before classification can be improved.

- `.log` files without paired JSON/report/checkpoint files.
- Any candidate whose filename does not reveal whether it contains timestamps, state snapshots, action proposals, or success/failure labels.
- Any live-run artifact that might include noisy operator/control context and needs careful redaction-aware inspection.

## High-Value Signal Map

For future conversion, these signal names should map into the task_0005 JSONL contract.

| Signal | Strongest candidate families | Conversion note |
| --- | --- | --- |
| `task_phase` | Seven-rods bundle, rod1 phase logs, retreat validation bundles | Use phase names from file stems and internal log markers when a future task reads contents. |
| `success_label` | `_report.json`, mission phase logs, live/dryrun validation reports | Prefer explicit terminal labels; do not infer success from missing errors alone. |
| `safety_gate` | Guarded dry-run bundles, odom/ultrasonic bundles, hard-min lateral safety reports | Preserve blocker reason strings as candidate `blocked_reasons`. |
| `robot_state` | Checkpoint JSON, JSONL traces, motion-trace artifacts | Extract only logged state values; missing state remains `null` or explicit missing. |
| `localization_state` | Odom/retreat/lateral trace artifacts | Candidate fields include odom availability, pose validity, map id, and navigation readiness. |
| `media_state` | Not strongly represented in this candidate list | Media conversion likely needs a separate viewer/media log inventory task. |
| `action_family` | Dry-run JSONL/checkpoint/report bundles | Likely must be derived from event semantics; preserve `unknown` if not explicit. |
| `action_vector` | Dry-run JSONL or trace artifacts only if vectors are logged | Do not synthesize vectors from filenames. |
| `critic_category` | Guarded dry-run and negative/blocked/failure logs | Map observed failure/blocker text to task_0005 taxonomy conservatively. |
| `artifact_refs` | All listed candidates | Store source path, file kind, and description for traceability. |

## Recommended task_0009

Recommended task:

`task_0009_convert_one_dryrun_bundle_to_vla_jsonl`

Low-risk objective:

Read one small high-value dry-run bundle, preferably
`rack_hybrid_docking_package/logs/industrial_7_rods_20260611_031537.*`, and
convert a small capped sample into VLA dry-run JSONL plus a validation report
using the existing static validator.

Suggested constraints:

- Read only the selected bundle files and the validator/schema files.
- Cap content reads to small text ranges or files below a strict size limit.
- Do not read binary files, datasets, checkpoints outside the selected bundle,
  rosbags, secrets, or robot-control source files.
- Do not run robot motion scripts, ROS, drivers, controllers, or live services.
- Output a sample real-log JSONL file, a conversion report, and a task result
  JSON under `.ai_agents`.

Acceptance target:

- At least one event validates with the task_0006 validator.
- Any unknown or missing field is explicit rather than inferred silently.
- The conversion report states which signals were extracted, which were absent,
  and which were deliberately left as `null` or `unknown`.
