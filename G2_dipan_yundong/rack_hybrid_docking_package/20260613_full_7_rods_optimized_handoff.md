# 2026-06-13 Full Seven-Rod Optimized Run Handoff

This is the direct resume note for the G2 map-station seven-rod workflow.

## Current Truth

- Local workspace: `/home/davie/G2/G2_dipan_yundong`
- Robot host: `agi@10.20.15.60`
- Robot workspace: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1`
- Active map: `map_id=19`
- Final checkpoint:
  `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/logs/industrial_cell_7_rods_optimized_live_checkpoint.json`
- Final checkpoint state:
  `rod_index=7`, `end_index=7`, `phase=MISSION_DONE`,
  `holding_rod=false`, `current_station=HOME_SAFE`,
  `last_success_step=ROD_DONE`.

Latest read-only readiness check after all code/doc sync:

- `ok=true`
- `charge_plug_insert_state=0`
- `charge_input_current_a=0.0`
- `charge_input_voltage_v=0.0`
- `motion_control_error=0`
- `pnc_task_state=7`
- current pose near HOME: `x=0.595950`, `y=-0.969844`

Save-time note: this shortcut handoff was created locally after the final
readiness check. An immediate `scp` to `10.20.15.60` failed with
`No route to host`, so verify or sync this file to the robot after SSH/network
reachability is restored. The rolling robot-side handoff
`industrial_cell_20260612_handoff.md` had already been synced before the route
dropped.

Operator shutdown note: after the local save, the robot was powered off. Do not
attempt robot-side sync or readiness checks until the robot is powered on and
SSH to `agi@10.20.15.60` is reachable again.

## Production Entry

Use this wrapper for the next full live run only after a fresh readiness check
and operator confirmation that the cell is reset for a new seven-rod cycle:

```bash
source /home/agi/app/env.sh
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --live --init --start-index 1 --end-index 7 \
  --checkpoint-file logs/industrial_cell_7_rods_optimized_live_checkpoint.json \
  --run-log logs/industrial_cell_7_rods_optimized_live.log
```

For tomorrow's first action, do not immediately run motion. Start with:

```bash
source /home/agi/app/env.sh
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
python3 rack_hybrid_docking_package/industrial_map_nav_guarded.py \
  --readiness-check \
  --config rack_hybrid_docking_package/industrial_station_config.json
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --status-only \
  --checkpoint-file logs/industrial_cell_7_rods_optimized_live_checkpoint.json
```

## Code State

Robot-side hashes after sync:

- `industrial_cell_7_rods_optimized.py`
  `203c1b3be16d5a70da0a7314e9f972becacb6b59928f391228844e451673c823`
- `industrial_cell_mission_controller.py`
  `7b28cb64351e0ad9069047d11ad3791c8bb97cc1287a16ad7124b05d4ad5cfd2`
- `analyze_industrial_cell_run.py`
  `811574d57e1ada3ec47ea3f2b7ae7be0462952967a6b4abb33cab5ed15b33753`
- `industrial_cell_20260612_handoff.md`
  `f96f673f8f631c2d5844e0d5ad6d9f5b470b3959d5bd0afe3627f5314706b9bf`

Important production settings in `industrial_cell_7_rods_optimized.py`:

- waist speed: `0.75 rad/s`
- arm speed: `0.12 rad/s`
- grab final stop: `341mm`
- place final stop: `327mm`, brake margin `60mm`
- after grab: first move back `8.5cm` while lowering `2cm`, then finish
  the `20cm` arm pull
- after pick arm pull: chassis retreat `45cm`
- after pick retreat: keep the grab waist/body pose; do not run
  `waist_home_after_pick`. The waist/body changes to the place pose at
  `LOCAL_PLACE` after navigation reaches `PLACE_PRE`.
- after place fine-position: chassis forward `10cm`
- after place/open grippers: segmented arm pull-out totaling `25cm`:
  `X=-2cm/Z=-1cm`, then `X=-4cm`, then `Z=-3.5cm`, then remaining `X=-19cm`;
  chassis retreat remains `45cm`
- yaw refine tolerance: `1.5deg`

`LOCAL_PLACE` now ends with `waist_home_after_place`, using
`/data/wxf/wxf/positions/arm_default.json`. The old label
`waist_grab_after_place` is only kept as a compatibility alias for resume
commands.

`--start-at-local-step` and `--stop-after-local-step` now fail fast on unknown
labels; they no longer silently start from the first local step.

## Run Evidence

Full combined log:

```bash
/data/g2_industrial_cell_20260612/wxf/BOX_528_1/logs/industrial_cell_7_rods_optimized_live.log
```

The run completed all seven rods. It stopped once during rod 3 `NAV_TO_HOME`
because yaw tolerance `1.0deg` was too strict for a physically arrived
`~1.38deg` residual. The wrapper was patched to `1.5deg`, synced, and the run
resumed from checkpoint. Rods 4 through 7 completed normally.

Use the read-only analyzer:

```bash
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
python3 rack_hybrid_docking_package/analyze_industrial_cell_run.py \
  logs/industrial_cell_7_rods_optimized_live.log
```

Latest analysis result:

- `events=1527`
- final state: `MISSION_DONE`, `HOME_SAFE`, `holding_rod=false`
- top bottleneck: `LOCAL_PLACE`, `n=7`, avg `71.978s`
- second bottleneck: `LOCAL_PICK`, `n=7`, avg `42.38s`
- `grab_fine_position`: avg `5.507s`
- `place_fine_position`: avg `3.542s`
- `yaw_refine`: avg `2.544s`, max initial abs error `2.79deg`
- one excluded large gap: rod 3 `NAV_TO_HOME`, `145.309s`, caused by the
  manual patch/resume pause

## Next Optimization

Do next in this order:

1. Reconfirm readiness and checkpoint only.
2. If optimizing speed, target `LOCAL_PLACE` first.
3. Safe candidates: reduce non-critical settle waits and improve summary
   reporting.
4. Candidate needing separate validation: direct route from `PLACE_PRE` to
   `HOME_SAFE` without the `RECOVERY_SAFE` detour.
5. Do not change the post-place end-effector pull-out stepping, final place
   offsets, grab final distance, or yaw tolerance without a single-step live
   validation plan.

## 2026-06-14 Continuation Note

Active robot workspace:

```bash
/data/g2_industrial_cell_20260612/wxf/BOX_528_1
```

Current live checkpoint after the interrupted rod-5 run:

- checkpoint: `logs/industrial_cell_2_to_7_optimized_live_checkpoint_20260615_0938.json`
- state: `rod_index=5`, `phase=LOCAL_PLACE`, `holding_rod=true`,
  `current_station=PLACE_PRE`, `last_success_step=NAV_TO_PLACE`
- readiness after code patch: `ok=true`, no problems/warnings, no lingering
  mission/controller/arm/nav process

Important physical/log caveat: rod 5 had already completed `open_gripper_place`
and started the old pure `place_pull_out_offset` command, but the log has no
`relative_offset_result=True` or `child_command_done` for that step. Do not
assume the old `25cm` pure pull-out fully completed.

Post-place pull-out logic was patched and synced to the robot:

- `industrial_cell_7_rods_optimized.py`
  `74c3636531722b02c574f9cd97de7105dfe2c9e1525fceab87dda0217f37c278`
- `industrial_cell_mission_controller.py`
  `5a88ba1f0330bcc58be70761e0a65dfa733143ad67c2f7e96351bd7c2ac8279a`

No-motion validation completed on a copied `/tmp` checkpoint. The generated
rod-5 `LOCAL_PLACE` plan after `open_gripper_place` is:

1. `place_pull_back_down_offset`: left/right `(-0.02, 0.0, -0.01)`
2. `place_pull_back_before_drop`: left/right `(-0.04, 0.0, 0.0)`
3. `place_pull_drop_offset`: left/right `(0.0, 0.0, -0.035)`
4. `place_pull_back_remaining_offset`: left/right `(-0.19, 0.0, 0.0)`

Before any live resume, visually confirm the current rod-5 arm pose and choose
the correct `--start-at-local-step`; do not blindly restart the whole
`LOCAL_PLACE` phase.

Robot sync note: the requested `6cm/3.5cm` post-place pull-out change has been
synced to the robot and no-motion verified. Robot-side hashes after sync:

- `industrial_cell_7_rods_optimized.py`
  `2f9f8c8d14c6c2975a01b47117ef4c3d5a774ca9ba12750ca995183b80b4cb8c`
- `industrial_cell_mission_controller.py`
  `5a88ba1f0330bcc58be70761e0a65dfa733143ad67c2f7e96351bd7c2ac8279a`

Rod 6 was completed after a transient `arm_default_after_place` planning
timeout was recovered by resuming from that local step. Current live checkpoint:

- checkpoint: `logs/industrial_cell_2_to_7_optimized_live_checkpoint_20260615_0938.json`
- state: `rod_index=7`, `phase=NAV_TO_GRAB`, `holding_rod=false`,
  `current_station=HOME_SAFE`, `last_success_step=rod_6_completed`
- final readiness: `ok=true`, `motion_control_error=0`, no lingering mission,
  controller, nav, arm, or offset process

## 2026-06-14 Final Rod-7 Completion

Rod 7 completed with the updated post-place pull-out sequence:

1. `place_pull_back_down_offset`: left/right `(-0.02, 0.0, -0.01)`
2. `place_pull_back_before_drop`: left/right `(-0.04, 0.0, 0.0)`
3. `place_pull_drop_offset`: left/right `(0.0, 0.0, -0.035)`
4. `place_pull_back_remaining_offset`: left/right `(-0.19, 0.0, 0.0)`

The mission reached:

- checkpoint: `logs/industrial_cell_2_to_7_optimized_live_checkpoint_20260615_0938.json`
- state: `rod_index=7`, `end_index=7`, `phase=MISSION_DONE`,
  `holding_rod=false`, `current_station=HOME_SAFE`,
  `last_success_step=ROD_DONE`

## 2026-06-15 Direct +2cm Place Pose and Rod-7 Hold

The production wrapper now sends `--skip-waist-home-after-pick`, so after
`LOCAL_PICK` it keeps the grab waist/body posture and navigates to `PLACE_PRE`.
`LOCAL_PLACE` then moves the waist/body directly to the place posture. This
removes the extra grab-pose-to-home-to-place waist transition.

The direct final place pose is now:

- `rack_hybrid_docking_package/calibration_records/rod07_place_final_arm_up020_latest.json`
- production wrapper passes it with `--place-pose-json`
- wrapper sets `--place-raise-before-open-z-m 0.0`, so there is no extra
  two-step lift before opening the grippers

Latest live checkpoint after the 2026-06-15 retry run:

- checkpoint:
  `logs/industrial_cell_full_7_industrial_retry_checkpoint_20260615_103648.json`
- state: `rod_index=7`, `end_index=7`, `phase=LOCAL_PLACE`,
  `holding_rod=true`, `current_station=PLACE_PRE`,
  `last_success_step=NAV_TO_PLACE`
- physical/log caveat: rod 7 had already reached the direct +2cm final place
  pose, opened the grippers, completed all four arm pull-out segments, then
  stopped on `retreat_after_place` because the rear ultrasonic reported a rear
  obstacle (`rear_filtered_mm=557`, raw sample around `390mm`).

Do not restart the whole `LOCAL_PLACE` from this checkpoint. The checkpoint is
stale because local substeps are not persisted inside the phase. Recovery should
be chosen from the physical pose: after operator confirmation, either clear the
rear obstacle and resume from `retreat_after_place`, or skip the chassis retreat
and run only the remaining post-place safe steps (`arm_default_after_place`,
`waist_home_after_place`) before navigating home.

Rod 7 was recovered after rear safety confirmation:

- resume entry: `--start-at-local-step retreat_after_place`
- `retreat_after_place`: completed, rear distance around `2276mm`
- `arm_default_after_place`: completed
- `waist_home_after_place`: completed
- final checkpoint:
  `logs/industrial_cell_full_7_industrial_retry_checkpoint_20260615_103648.json`
- final state: `MISSION_DONE`, `HOME_SAFE`, `holding_rod=false`,
  `last_success_step=ROD_DONE`
- final readiness: `ok=true`, `motion_control_error=0`, `pnc_task_state=9`

## 2026-06-15 Next-Round Stability Patch

Log review from the 2026-06-15 retry run:

- Rod 3 had one `PLACE_PRE` navigation stop with
  `pnc_idle_before_arrival`: XY error was already about `1.4cm`, but yaw was
  still about `3.24deg`. Current `industrial_map_nav_guarded.py` already
  handles this by taking over with low-speed yaw refine when XY is within
  tolerance and yaw is within `--refine-yaw-max-error-deg`; robot-side hash:
  `55ead5e0835bb743757d770c321b4f9dafaf385ea24a857fe39dbf214e22eaaa`.
- Rods 4 and 7 hit `rear_obstacle` around `retreat_after_place`.
  The post-place arm pull-out had already completed. The new
  `rack_industrial_docking.py` patch only changes the no-motion start check:
  before chassis retreat it samples rear ultrasonic up to 4 times at 0.08s
  intervals and requires 2 hit samples, or a hard two-sensor hit, before
  failing. During actual retreat, the existing immediate hard stop and
  filtered stop behavior is unchanged.
- `industrial_cell_7_rods_optimized.py` now sends
  `--skip-waist-home-after-pick`, saving the extra grab-pose-to-home waist move
  and letting `LOCAL_PLACE` switch directly to the place waist posture.

Robot-side validation:

- `py_compile` passed for `rack_industrial_docking.py`,
  `industrial_cell_7_rods_optimized.py`, `industrial_map_nav_guarded.py`, and
  `industrial_cell_mission_controller.py`.
- no-motion dry-run:
  `logs/dryrun_next_round_optimized_20260615_1138.log`
- confirmed dry-run plan:
  `LOCAL_PICK` ends at `retreat_after_pick` with no `waist_home_after_pick`;
  `LOCAL_PLACE` uses
  `rod07_place_final_arm_up020_latest.json` directly and has no
  `place_raise_before_open_offset`.
- robot-side hashes:
  - `rack_industrial_docking.py`
    `accf09bdf46be4622e2cefe038dfb65d799ce8d29331d7e0502bed58a10e63c1`
  - `industrial_cell_7_rods_optimized.py`
    `bb01176e3f412cf0c7344b3cf3ccddda87dd691d9ae15ca6d588fdc199863028`
  - `industrial_map_nav_guarded.py`
    `55ead5e0835bb743757d770c321b4f9dafaf385ea24a857fe39dbf214e22eaaa`

Next live run command after operator confirms the cell is reloaded/reset and
safe:

```bash
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --live --init --start-index 1 --end-index 7 \
  --checkpoint-file logs/industrial_cell_next_round_checkpoint_20260615_HHMM.json \
  --run-log logs/industrial_cell_next_round_live_20260615_HHMM.log \
  --direct-home-after-place \
  --refine-yaw-stations GRAB_PRE,PLACE_PRE
```
- final readiness: `ok=true`, `motion_control_error=0`,
  `charge_plug_insert_state=0`
- final pose was near `HOME_SAFE`: x `0.618365`, y `-0.968163`
- no lingering `industrial_cell_7_rods_optimized.py`,
  `industrial_cell_mission_controller.py`, `move_ee_relative_offset.py`,
  `move_arm_by_json_path.py`, or `industrial_map_nav_guarded.py` process

The combined live log remains:

```bash
logs/industrial_cell_2_to_7_optimized_live_20260615_0938.log
```

## 2026-06-15 Industrial Automation Hardening

Goal: make the next full 7-rod cycle more suitable for unattended industrial
operation without increasing the risk of repeated relative motion.

Patch applied:

- `industrial_cell_mission_controller.py` now auto-retries only idempotent
  absolute local child commands:
  - `move_arm_by_json_path.py`
  - `move_waist_by_json_path.py`
  - `move_ee_pose_open_2.py`
  - `move_ee_pose_close_2.py`
- retry count is `2`, retry delay is `1.0s`
- each retry attempt writes its own `_attemptN` local log and the final
  `local_child_step_done` event includes an `attempts` list

Reason: the completed run's main unattended blocker was a transient
`arm_default_after_place` planning timeout after rod 6. That step is an
absolute arm pose command and can safely be retried once. Relative offset
commands, chassis-relative commands, fine positioning, retreat, and map
navigation are intentionally not auto-retried because partial completion could
duplicate motion.

Validation completed:

- local `py_compile`: passed
- robot-side sync hash:
  - `industrial_cell_mission_controller.py`
    `b7e228f4de70a994a27fa061c7eb7893976c01c21a27bf5a63eb8da220424d77`
  - `industrial_cell_7_rods_optimized.py`
    `2f9f8c8d14c6c2975a01b47117ef4c3d5a774ca9ba12750ca995183b80b4cb8c`
- robot-side `py_compile`: passed
- robot-side retry constants import:
  - attempts: `2`
  - retryable scripts:
    `move_arm_by_json_path.py,move_ee_pose_close_2.py,move_ee_pose_open_2.py,move_waist_by_json_path.py`
- no-motion dry-run with temporary checkpoint completed rod 1 to
  `MISSION_DONE`

Current read-only robot status before the next live run:

- readiness: `ok=true`
- `charge_plug_insert_state=0`
- `motion_control_error=0`
- current pose is near `HOME_SAFE`
- previous full-run checkpoint remains `MISSION_DONE`, `holding_rod=false`

Next intended full-cycle command after现场 safety confirmation:

```bash
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --live --init --start-index 1 --end-index 7 \
  --checkpoint-file logs/industrial_cell_full_7_industrial_retry_checkpoint_20260615_HHMM.json \
  --run-log logs/industrial_cell_full_7_industrial_retry_live_20260615_HHMM.log \
  --direct-home-after-place \
  --refine-yaw-stations GRAB_PRE,PLACE_PRE
```

## 2026-06-15 Live Place Height Adjustment

Operator feedback during the retry run: the placed rods were sitting too low.
Patch applied without sending robot motion:

- `industrial_cell_mission_controller.py` adds
  `--place-raise-before-open-z-m`, default `0.0`.
- When nonzero, LOCAL_PLACE inserts
  `place_raise_before_open_offset` after `arm_place_pose` and before
  `open_gripper_place`.
- `industrial_cell_7_rods_optimized.py` now passes
  `--place-raise-before-open-z-m 0.04`, raising both end effectors 4 cm before
  opening the grippers.
- This is a relative end-effector offset and is not auto-retried.

Validation:

- local `py_compile`: passed.
- local plan check confirmed:
  `arm_place_pose` -> `place_raise_before_open_offset`
  left/right `(0.0, 0.0, 0.04)` -> `open_gripper_place`.
- robot-side sync hashes:
  - `industrial_cell_mission_controller.py`
    `f9b604a66cb4639f4343e28f033f71fcb6f109584ade9a72fc4bd230547a5ccd`
  - `industrial_cell_7_rods_optimized.py`
    `edf350afbf35036b3cff73b3f6a129a382a0304cc368433c75ce6c79be3b7744`
- robot-side `py_compile`: passed.
- robot-side status-only after sync showed checkpoint still at
  `rod_index=4`, `phase=LOCAL_PLACE`, `holding_rod=true`,
  `current_station=PLACE_PRE`, `last_success_step=NAV_TO_PLACE`.

Important live-state caveat:

- Physically, rod 4 had already been placed, grippers opened, and arms pulled
  out before the rear obstacle stop.
- The failure was at `retreat_after_place` before chassis retreat, with rear
  ultrasonic reporting an obstacle.
- Do not resume rod 4 by rerunning the whole `LOCAL_PLACE`, because the
  checkpoint does not record local substep progress.
- After rear safety is confirmed, resume rod 4 only from
  `--start-at-local-step retreat_after_place`; rods 5-7 will then use the new
  4 cm pre-open raise step.

## 2026-06-15 Direct Place Height Correction

Operator clarification after rod 5: the desired behavior is not
`old final place pose -> relative Z lift -> open grippers`. The final place
pose itself must be recalibrated higher, so the arm goes directly to the new
height before opening.

No-motion patch applied:

- `industrial_cell_7_rods_optimized.py` now passes
  `--place-pose-json rack_hybrid_docking_package/calibration_records/rod07_place_final_arm_up020_latest.json`.
- `--place-raise-before-open-z-m` is set to `0.0`, so
  `place_raise_before_open_offset` is no longer present in the LOCAL_PLACE
  plan.
- `--status-only` skips file validation so checkpoint/status can still be read.
- Normal live/dry-run execution still validates files first and will stop if
  `rod07_place_final_arm_up020_latest.json` is missing.

Validation:

- local `py_compile`: passed.
- local plan check confirmed:
  `arm_place_pose` using `rod07_place_final_arm_up020_latest.json` ->
  `open_gripper_place`, with `has_raise_step=false`.
- robot-side wrapper hash after sync:
  `26a293ab89816cea7f43deadf1b6f9d8127aa9cbf2d614abf3eb195e05905825`.
- robot-side calibration helper hash after sync:
  `8cf31fca7d6990d1c8fb21f80d9d34a6ebe97fa5b7a4e7e1f33b4d0a2014bba5`.
- robot-side `py_compile`: passed.
- robot-side calibration helper `--dry-run --z-m 0.02`: passed; no GDK init
  and no motion.
- robot-side runner no-motion file check fails as intended until the new JSON
  exists:
  `missing required files: .../rod07_place_final_arm_up020_latest.json`.
- robot-side `--status-only` still shows:
  `rod_index=6`, `phase=NAV_TO_GRAB`, `holding_rod=false`,
  `current_station=HOME_SAFE`, `last_success_step=rod_5_completed`.

Important: `rod07_place_final_arm_up020_latest.json` still must be generated
from a live, empty-gripper calibration before continuing rod 6. The safe
calibration sequence is:

1. confirm the robot is still at `HOME_SAFE`, not holding a rod, and the area
   around the upper body is clear;
2. run:

```bash
python3 rack_hybrid_docking_package/calibrate_direct_place_pose_offset.py \
  --confirm-physical --z-m 0.02
```

The helper moves waist/body to `rod07_place_waist_adjusted_latest.json`, moves
arms to the original `rod07_place_final_arm_latest.json`, raises both end
effectors by `+0.02m`, captures the resulting arm joint positions as
`rod07_place_final_arm_up020_latest.json`, then returns arms and waist/body to
`/data/wxf/wxf/positions/arm_default.json`.

3. run a no-motion plan/status check before restarting rod 6.

## 2026-06-15 Local Continuation Audit

This continuation was local/read-only only. No robot motion command was sent.

Current local artifact state:

- `rack_hybrid_docking_package/calibration_records/rod07_place_final_arm_up020_latest.json`
  now exists locally.
- capture file:
  `rack_hybrid_docking_package/calibration_records/rod07_place_final_arm_up020_capture_20260615_112207.json`
- local hashes:
  - `rod07_place_final_arm_up020_latest.json`
    `e082dcb8cb9cc0a448e3c1c229780c4c52437ceb8819bc8c49220d4166283106`
  - `rod07_place_final_arm_up020_capture_20260615_112207.json`
    `b0140a519e1c656dc5951a5d03355f91e31138a440dbd4b7e6c005ee6d8da6ed`
  - `industrial_cell_7_rods_optimized.py`
    `bb01176e3f412cf0c7344b3cf3ccddda87dd691d9ae15ca6d588fdc199863028`
  - `industrial_cell_mission_controller.py`
    `f9b604a66cb4639f4343e28f033f71fcb6f109584ade9a72fc4bd230547a5ccd`
  - `calibrate_direct_place_pose_offset.py`
    `8cf31fca7d6990d1c8fb21f80d9d34a6ebe97fa5b7a4e7e1f33b4d0a2014bba5`
  - `rack_industrial_docking.py`
    `accf09bdf46be4622e2cefe038dfb65d799ce8d29331d7e0502bed58a10e63c1`

Local validation:

- `python3 -m py_compile` passed for:
  - `industrial_cell_7_rods_optimized.py`
  - `industrial_cell_mission_controller.py`
  - `rack_industrial_docking.py`
  - `calibrate_direct_place_pose_offset.py`
- Local wrapper dry-run is not authoritative because this laptop mirror does not
  contain the robot-only files such as `/data/wxf/wxf/positions/arm_default.json`
  and the per-rod `rod*_grab_pose_*.json` files.

Robot reachability:

- `ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=5 agi@10.20.15.60 hostname`
  returned `No route to host`.
- The same command also returned `No route to host` when run outside the local
  sandbox, so this is a real network/robot reachability blocker, not a local
  tool permission issue.
- Therefore robot-side file presence, hashes, checkpoint, readiness, and
  no-motion plan were not reverified in this continuation.

When the robot is reachable again, start with read-only checks only:

```bash
source /home/agi/app/env.sh
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
ls -lt logs/*checkpoint*.json | head -20
python3 rack_hybrid_docking_package/industrial_map_nav_guarded.py \
  --config rack_hybrid_docking_package/industrial_station_config.json \
  --readiness-check
```

Then run `--status-only` against the active recent checkpoint before any live
resume:

```bash
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --status-only \
  --checkpoint-file logs/<ACTIVE_CHECKPOINT>.json
```

Do not continue rod 6 or any later rod from memory alone. The direct-height
correction branch must be resumed only after the robot-side checkpoint confirms
the active `rod_index`, `phase`, `holding_rod`, `current_station`, and
`last_success_step`, and after the robot-side copy of
`rod07_place_final_arm_up020_latest.json` is present with the intended content.

## 2026-06-15 Robot Reachable at 192.168.0.7

Read-only recovery checks were run against the restored robot network address
`agi@192.168.0.7`.

Confirmed:

- robot workspace exists:
  `/data/g2_industrial_cell_20260612/wxf/BOX_528_1`
- robot-side hashes match local for:
  - `rod07_place_final_arm_up020_latest.json`
  - `industrial_cell_7_rods_optimized.py`
  - `industrial_cell_mission_controller.py`
  - `calibrate_direct_place_pose_offset.py`
  - `rack_industrial_docking.py`
- key checkpoints are complete, including
  `logs/industrial_cell_full_7_industrial_retry_checkpoint_20260615_103648.json`:
  `rod_index=7`, `end_index=7`, `phase=MISSION_DONE`,
  `holding_rod=false`, `current_station=HOME_SAFE`,
  `last_success_step=ROD_DONE`
- robot-side `py_compile` passed for the optimized wrapper, mission controller,
  rack docking primitive, and direct place calibration helper.
- robot-side no-motion dry-run passed with the direct +2cm place JSON:
  `arm_place_pose` uses
  `rack_hybrid_docking_package/calibration_records/rod07_place_final_arm_up020_latest.json`
  and there is no `place_raise_before_open_offset`.

Map update:

- Operator confirmed current map is `20` and the navigation points are the same.
- `rack_hybrid_docking_package/industrial_station_config.json` was updated from
  `map_id=19` to `map_id=20` locally and on the robot.
- Robot-side backup of the old config:
  `rack_hybrid_docking_package/industrial_station_config.json.bak_map19_to20_20260615_1418`
- New config hash:
  `c7b3cd42b039d93a75e8b182ec4c4c06aed4ddca1e4c58ece1136f786ff765a8`

Current blocker:

- readiness no longer reports a map mismatch.
- readiness still fails because the robot is charging:
  `charge_plug_insert_state=1`,
  `charge_input_voltage_v=50.5`,
  `charge_input_current_a=15.0`.
- `motion_control_error=0`, `pnc_task_state=7`, odom speed samples are all
  `0.0`.

Do not run live chassis/navigation until charging is physically disconnected and
readiness returns `ok=true`.

## 2026-06-15 Map-20 Live Attempt Stopped at Rod 1

After the charge plug was physically removed, readiness returned `ok=true`:

- `map_id=20`
- `charge_plug_insert_state=0`
- charge input `0V / 0A`
- `motion_control_error=0`
- `pnc_task_state=7`
- odom speed samples all `0.0`

A new live run was started with:

```bash
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --live --init --start-index 1 --end-index 7 \
  --checkpoint-file logs/industrial_cell_full_7_map20_after_unplug_checkpoint_20260615_1421.json \
  --run-log logs/industrial_cell_full_7_map20_after_unplug_live_20260615_1421.log \
  --direct-home-after-place \
  --refine-yaw-stations GRAB_PRE,PLACE_PRE
```

Observed sequence:

- readiness passed again inside the runner.
- `NAV_TO_GRAB` reached `GRAB_PRE` with final XY error about `0.007m` and yaw
  error about `0.44deg`.
- `LOCAL_PICK` completed:
  - `open_gripper`
  - `waist_for_grab`
  - `arm_grab_pose`
- `grab_fine_position` then failed before any gripper close or pickup:
  `no_front_ultrasonic_lock`,
  `No stable front radar history for (0, 1): 0/3`.

Read-only diagnosis:

- checkpoint after stop:
  `rod_index=1`, `phase=LOCAL_PICK`, `holding_rod=false`,
  `current_station=GRAB_PRE`, `last_success_step=NAV_TO_GRAB`
- readiness after stop remained `ok=true`.
- front ultrasonic IDs `0/1` were invalid or missing; raw payload showed
  repeated `distance_mm=65535` for IDs `0` and `1`.
- rack industrial snapshot at the stopped pose showed front ultrasonic mostly
  unavailable and lidar rack distance around `3.87-4.13m`.
- That means the current map20 `GRAB_PRE` coordinate is not a valid near-rack
  pre-grab station for this workflow, even though the navigation controller
  reached the configured coordinate.

Safety recovery performed:

- No rod was grabbed and grippers were not closed.
- No chassis recovery motion was sent.
- Upper body was returned to default only:
  - `move_arm_by_json_path.py --json /data/wxf/wxf/positions/arm_default.json`
    returned `move_arm_joint_result=0`.
  - `move_waist_by_json_path.py --json /data/wxf/wxf/positions/arm_default.json`
    returned `move_waist_joint_result=0`, `final_max_error_rad=0.000478`.
- Final readiness after recovery remained `ok=true`; robot pose is still near
  the configured `GRAB_PRE` coordinate.

Do not resume this checkpoint directly. The next required work is map20 station
validation/recapture, especially `GRAB_PRE`, before another live seven-rod run.
If continuing from the current physical pose, first perform read-only station
capture/pose verification; do not run `LOCAL_PICK` again from the stale
checkpoint.

## 2026-06-15 End-of-Day Map20 Rod 5 Stop

Detailed handoff:

```bash
rack_hybrid_docking_package/20260615_map20_live_stop_handoff.md
```

Current state for the next session:

- active robot: `agi@192.168.0.7`
- active remote workspace:
  `/data/g2_industrial_cell_20260612/wxf/BOX_528_1`
- latest live run:
  `logs/live_rod5_to_7_place_retreat_tolerate_20260615_192856.log`
- rod 5 was physically placed and grippers opened
- checkpoint is stale and still says:
  `rod_index=5`, `phase=LOCAL_PLACE`, `holding_rod=true`,
  `current_station=PLACE_PRE`
- after release, `retreat_after_place` stopped before moving because rear
  ultrasonic precheck was `203mm`
- direct `arm_default_after_place` failed with GDK PLANNING timeout, including a
  standalone retry to `/data/wxf/wxf/positions/arm_default.json`
- final read-only status was clean except for the unrecovered upper-body pose:
  charge disconnected, `motion_control_error=0`, PNC state `7`, odom stopped,
  whole-body errors all `0`

Next session rule: do not continue from the stale rod-5 checkpoint. First do
read-only status, recover arms/waist from the post-place pull-back posture, get
to a known safe station, then start the next live run from rod 6.
