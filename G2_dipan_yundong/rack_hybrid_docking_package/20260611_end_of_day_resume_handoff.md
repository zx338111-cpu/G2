# 2026-06-11 End-of-Day Resume Handoff

## Current Truth Source

- Local workspace: `/home/davie/G2/G2_dipan_yundong`
- Robot host: `agi@10.20.15.199`
- Robot project root:
  `/data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1`
- Main controller:
  `rack_hybrid_docking_package/industrial_7_rods_total_controller.py`
- Long engineering record:
  `rack_hybrid_docking_package/20260611_full_run_automation_optimization.md`

## Current Robot State

Latest live result, 2026-06-12 09:56 robot time:

- **Stopped safely, but the robot is charging again. Do not send physical
  motion while `charge_plug_insert_state=1` or charging current/voltage is
  present.**
- Rod 7 is **partially complete**. It is already grabbed, pulled out, grab-
  retreated, right-turned, and moved to the shared place-above pose. Do not
  rerun rod 7 from the beginning unless the physical scene is manually reset.
- Rod 7 successful partial run:
  `logs/live_guarded_rod7_after_front_recovery_roi04_20260612_095312.log`
  - grab-side post-approach passed as `inconclusive_ultrasonic_verified`;
  - grab retreat front-ultrasonic delta was `{0: 806, 1: 805}` for the `820mm`
    target, remaining `15mm`;
  - grab retreat odom crosscheck was `0.812m` for a `0.820m` target;
  - right-turn yaw validation expected `90.000deg`, actual `89.895deg`, error
    `-0.105deg`.
- That partial run stopped at the place-side before-approach guarded check with
  the old rod-7 place ROI (`lateral_half_width=0.4`, `z=0.6-1.2`):
  `lateral_center_m=0.0875`, `yaw_deg=-6.451`, reason
  `yaw_too_large_for_lateral_active`. No place approach, down move, open, place
  retreat, or final left turn was executed.
- Read-only ROI sweep after the stop:
  `logs/rack_pose_roi_sweep_rod7_place_above_20260612_*.json` and `.md`
  showed the best resume ROI is range `0.8-1.6m`, lateral half-width `0.5m`,
  z `0.7-1.3m`, bin `0.25m`, min cluster points `20`. It measured
  `lateral_center_m=0.0375`, `yaw_deg=-3.4777`, confidence `0.891`, valid
  `8/8`.
- The attempted resume using that high ROI was correctly blocked before motion
  because startup preflight saw charging:
  `logs/live_resume_rod7_place_above_roi_high05_20260612_095557.log`.
- Latest read-only charging snapshot after the blocked resume:
  `charge_plug_insert_state=1`, charge input about `50.5V / 15A`, one battery
  reporting `battery_charging_status=1` and `battery_charging_current=9A`,
  `motion_control error_code=0`, PNC task state `7`, odom available,
  `stopped=True`.
- Do not treat any of this as permission to enable active `linear.y` lateral
  correction; active lateral correction is still not production-calibrated.

Next physical continuation must start from a fresh read-only status check. Only
after the robot is confirmed not charging should the sequence resume from
`--resume-after-place-above-index 7` with the high ROI listed below. Do not run
all 7 rods unattended.

## Code Synced to Robot

Current synchronized hashes:

- 2026-06-12 rod-3 guarded continuation patches:
  - `industrial_7_rods_total_controller.py`
    `d5b348c8da27a59ab8594a04fd8bc359ebeb016c4775e027506873812470c9d5`
  - robot-side backups before these patches:
    `industrial_7_rods_total_controller.py.bak_20260612_avg_window_accept`
    `industrial_7_rods_total_controller.py.bak_20260612_near_target_skip`
  - change summary:
    grab approach can now, only when explicitly requested, accept the 155mm
    target by average distance if the minimum distance remains above the hard
    safety limit and the dual-front span is within the requested cap; near-target
    restart skips the before-approach lidar centering check and still runs the
    post-approach guarded check.

- 2026-06-12 review patch:
  - `industrial_7_rods_total_controller.py`
    `1382383b8dd44ed6bbce5391313f0d2c553dd551e119f2e2ae778924310e7666`
  - robot-side backup before this patch:
    `industrial_7_rods_total_controller.py.bak_20260612_startup_preflight_review`
  - robot-side backup before the artifact-gate follow-up:
    `industrial_7_rods_total_controller.py.bak_20260612_artifact_gate`
  - change summary:
    startup live preflight now blocks before any arm/chassis action if PNC task
    state is active, odom xy/yaw is unavailable for this velocity-turn/front-
    ultrasonic-retreat run, or odom velocity cannot prove the robot is stopped;
    successful child action scripts now write `step_done`/checkpoint events;
    live runs now reject pre-existing log/jsonl/checkpoint/report artifacts
    unless `--allow-existing-artifacts` is passed.

Previous end-of-day synchronized hashes:

- `industrial_7_rods_total_controller.py`
  `f96e71cd76bd56a77b931e4688b9547b08e7c75fe4a1083eec4ea98557a14a6c`
- `guarded_front_target_recovery.py`
  `17698fc66cf6833d6d4bfa6946b3e3178162f7730d8146b0a073bda41d3c3a41`
- `20260611_full_run_automation_optimization.md`
  `7645e4c264dc04b8af563483381e4e78fbf2eccd6f4a7cafa75ef88cebce36d2`

Robot-side backups from today:

- `industrial_7_rods_total_controller.py.bak_20260611_auto_odom_guarded`
- `industrial_7_rods_total_controller.py.bak_20260611_guarded_ultrasonic_override`
- `industrial_7_rods_total_controller.py.bak_20260611_clearance_retry`
- `industrial_7_rods_total_controller.py.bak_20260611_postcheck`

## What Is Completed

- Rod 1 completed after guarded validation, recovery, and resume.
- Rod 2 completed after guarded validation and resume from grab-retreat.
- Rod 3 completed after guarded validation, ROI revalidation, place-retreat
  target recovery, and yaw-validated left turn.
- Rod 4 completed after guarded validation and ROI revalidation with
  `rack-pose-lateral-half-width-m=0.4`.
- Rod 5 completed after guarded validation and a place-retreat target recovery
  from the report hint.
- Rod 6 completed after guarded validation, without needing a recovery resume.
- Rod 7 has not completed. It is currently stopped after grab, grab-retreat,
  right turn, and move-to-place-above, with the rod still presumed held.
- Current blocker is charging state, not controller code: do not continue until
  read-only status shows no charge plug, no charge voltage/current, and no
  battery charging bits.
- Do not treat this as proof that 7 rods can run unattended.

## Main Patches Now Active

- `front-ultrasonic` retreat keeps the `1m +/-20mm` target and odom `+/-0.02m`
  crosscheck.
- Odom tail auto-correction is enabled after front-ultrasonic retreat reaches the
  target window but odom is barely outside tolerance.
- Odom auto-correction clearance now retries transient ultrasonic empty frames
  for `0.6s`; real hard-min violations still stop immediately.
- `rack-centering-mode guarded` checks rack pose before grab/place approach.
- Guarded mode can accept a stable dual-front-ultrasonic override when lidar yaw
  is noisy but lateral offset is within the safe override range.
- New post-approach guarded check runs after reaching the grab/place front target
  and before the next arm action:
  - stable pose lateral over `0.12m` blocks;
  - unsafe front ultrasonic blocks;
  - pose unavailable but stable/safe dual-front ultrasonic records
    `inconclusive_ultrasonic_verified` and continues.

## Evidence Logs

Rod 2 failed first at grab retreat auto-correction because rear ultrasonic was a
single transient empty frame:

- `logs/live_guarded_rod2_after_adaptive_patch_20260611_2015.log`
- `logs/live_guarded_rod2_after_adaptive_patch_20260611_2015_report.json`

Rod 2 completed after resume:

- `logs/live_resume_rod2_after_grab_retreat_guarded_20260611_2023.log`
- `logs/live_resume_rod2_after_grab_retreat_guarded_20260611_2023_report.json`

Post-approach guarded dry-run:

- `logs/dryrun_post_approach_guarded_rod3_20260611.log`
- `logs/dryrun_post_approach_guarded_rod3_20260611_report.json`

Rod 3 guarded continuation on 2026-06-12:

- `logs/live_guarded_rod3_postcheck_20260612_rerun_084940.log`
  stopped before grab because dual-front span was `22mm` with the old hard
  `20mm` threshold.
- `logs/live_guarded_rod3_avg_accept_20260612_085405.log`
  stopped on near-target restart before the near-target skip patch.
- `logs/live_guarded_rod3_near_target_20260612_085608.log`
  grabbed rod 3, retreated, right-turned, moved to place above, then stopped on
  a conservative place lateral offset.
- `logs/rack_pose_roi_sweep_rod3_place_above_20260612_*.json` and `.md`
  showed the stable place ROI:
  range `0.8-1.6m`, lateral half-width `0.5m`, z `0.6-1.2m`, bin `0.25m`,
  min cluster points `20`.
- `logs/live_resume_rod3_place_above_roi_20260612_090240.log`
  placed rod 3 and stopped during place-retreat odom tail auto-correction
  because rear ultrasonic samples were transiently unavailable after the front
  target was already near `1327mm`.
- `logs/live_resume_rod3_left_turn_roi_20260612_090730.log`
  resumed from `--resume-after-place-retreat-target-index 3
  --place-retreat-front-target-mm 1327`, confirmed the target window, left
  turned 90 degrees, and completed rod 3.

Rod 4 guarded continuation on 2026-06-12:

- `logs/live_guarded_rod4_single_20260612_092435.log`
  stopped before the grab approach because the `0.5m` lateral-half-width ROI
  measured `lateral_center_m=-0.0995`, outside the default `0.08m` guarded
  target.
- `logs/rack_pose_roi_sweep_rod4_grab_above_20260612_*.json` and `.md`
  showed `rack-pose-lateral-half-width-m=0.4` was stable and inside the same
  guarded target: median lateral about `-0.058m`, valid `8/8`, confidence about
  `0.949`.
- `logs/live_guarded_rod4_single_roi04_20260612_092601.log`
  completed rod 4. Grab retreat odom crosscheck was `0.818m` for a `0.820m`
  target, and place retreat odom crosscheck was `0.986m` for a `1.000m`
  target.

Rod 5 guarded continuation on 2026-06-12:

- `logs/live_guarded_rod5_single_roi04_20260612_093015.log`
  completed grab, grab retreat, right turn, place, open, and pull clear, then
  stopped during final place retreat. The front ultrasonic deltas became
  inconsistent and odom crosscheck measured `1.324m` against the `1.000m`
  target; resume hint was
  `--resume-after-place-retreat-target-index 5 --place-retreat-front-target-mm 1327`.
- Read-only checks after that stop showed lidar around `1.75-1.82m` and front
  ultrasonic mostly around `1.75-1.78m`, confirming over-retreat before recovery.
- `logs/live_resume_rod5_place_retreat_target_roi04_20260612_093340.log`
  recovered the front target to `1320mm` in the `1327 +/-70mm` window, left
  turned 90 degrees with yaw error `-0.160deg`, and completed rod 5.

Rod 6 guarded continuation on 2026-06-12:

- `logs/live_guarded_rod6_single_roi04_20260612_093902.log`
  completed rod 6 in one run. Grab retreat odom crosscheck was `0.812m` for a
  `0.820m` target, place retreat odom crosscheck was `0.985m` for a `1.000m`
  target, right-turn yaw error was `-0.136deg`, and final left-turn yaw error
  was `0.137deg`.

Rod 7 guarded continuation on 2026-06-12:

- `logs/live_guarded_rod7_single_roi04_20260612_094803.log`
  stopped before grab because a single front ultrasonic sample jumped to
  `856/162mm`, producing `front_ultrasonic_max_span_too_large=694>110`.
- `logs/live_guarded_rod7_rerun_after_front_transient_roi04_20260612_094937.log`
  stopped before grab because the robot was slightly too close
  (`front_min_mm=131`) and the automatic small safe backoff was blocked by a
  transient rear low value.
- `logs/live_rod7_front_safe_backoff_004m_20260612_095115.log`
  backed off by the strict front-ultrasonic primitive. It targeted `40mm`,
  reached `{0: 36, 1: 33}` delta, remaining `7mm`, and odom measured
  `0.025m / 0.040m`.
- `guarded_front_target_recovery.py --target-front-mm 155 --tolerance-mm 10`
  then moved forward about `0.019m`, settling around front avg `153mm`.
- `logs/live_guarded_rod7_after_front_recovery_roi04_20260612_095312.log`
  successfully grabbed rod 7, pulled it out, completed grab retreat and right
  turn, then stopped at place-side guarded before-approach with the old place
  ROI (`yaw=-6.451deg`).
- `logs/rack_pose_roi_sweep_rod7_place_above_20260612_*.json` and `.md`
  identified the stable place resume ROI:
  range `0.8-1.6m`, lateral half-width `0.5m`, z `0.7-1.3m`, bin `0.25m`,
  min cluster points `20`.
- `logs/live_resume_rod7_place_above_roi_high05_20260612_095557.log`
  attempted that resume but executed no motion because startup preflight blocked
  on `charge_plug_insert_state=1`.

Function-level local checks already done:

- The rod 2 after-approach sample with lateral `-0.151m` is now blocked as
  `blocked_pose_offset`.
- Pose unavailable with stable safe dual-front ultrasonic passes as
  `inconclusive_ultrasonic_verified`.

2026-06-12 unattended-automation patch prepared locally:

- `industrial_7_rods_total_controller.py` now supports place-stage-only rack
  pose ROI overrides (`--rack-place-pose-*`). This keeps grab-side guarded ROI
  tight while allowing the empirically stable high place-side ROI for rod 7.
- Near-target front-ultrasonic recovery now skips lidar lateral centering when
  the robot is already within `target+tolerance+margin`; this avoids the
  close-range lidar dead zone seen around the rod 7 front recovery, while the
  post-approach guarded check still protects the arm action.
- Post-approach guarded check now resamples only when the sole problem is a
  single-window front ultrasonic max-span outlier. Median span, minimum safe
  distance, and pose offset blockers remain hard stops.
- Front-too-close safe backoff now retries only the rear precheck transient
  case. Persistent rear obstacle, charging, estop, hard distance, or motion
  faults still stop the run.
- Local verification: `python3 -m py_compile` passed; argparse/config
  instantiation passed; invalid automation parameter caps are rejected. Full
  local dry-run is not possible in this checkout because the local folder does
  not contain the full BOX action script set.
- Deployed to `agi@10.20.15.199` under
  `/data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1`.
  Remote backups were created with suffix
  `.bak_20260612_unattended_automation`.
- Remote verification passed:
  controller sha256
  `4aad6161c9595dd1a272c00b2ca606fc4070fbc9fe629355ff09718c01407277`,
  handoff sha256
  `beea848e00cba0dd604f28d68b37d8d0b047d5d36ff794fff0cb0340db54a6d8`
  at deployment time, `python3 -m py_compile` passed, and
  `logs/dryrun_unattended_automation_patch_20260612.log` completed a full
  one-rod dry-run plan without `--confirm-live`.
- Post-deploy read-only status still shows charging:
  `charge_plug_insert_state=1`, charge input about `50.5V/14.8A`, one battery
  charging status set. Motion state was clean (`motion_control error_code=0`,
  PNC `state=7`, stopped check true), but no live motion is allowed while
  charging remains connected.

## First Command Tomorrow

Run this read-only status check first:

```bash
SSHPASS='<robot-password>' sshpass -e ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password \
  -o NumberOfPasswordPrompts=1 -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
  agi@10.20.15.199 \
  'source /home/agi/app/env.sh; cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1; python3 industrial_status_snapshot.py --samples 8 --interval-s 0.25'
```

Proceed only if:

- `charge_plug_insert_state=0`
- no charging current/voltage and no battery charging-status bits
- `motion_control error_code=0`
- `emergency_stop_pedal_state=0`
- PNC task state is idle, normally `7`; do not proceed while it is `2`
- odom/yaw is available
- `stopped_check=True`
- there is enough rear clearance for 1m retreat

## Recommended Next Live Run

Do not run all 7 rods. Do not restart rods 3, 4, 5, 6, or rod 7 from the
beginning. Rod 7 is already past grab, grab-retreat, right turn, and place-above.
If the field is still safe and the charging state is cleared, use a fresh
read-only status check first, then resume **rod 7 from place-above** only:

```bash
SSHPASS='<robot-password>' sshpass -e ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password \
  -o NumberOfPasswordPrompts=1 -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
  agi@10.20.15.199 \
  'source /home/agi/app/env.sh; cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1; python3 rack_hybrid_docking_package/industrial_7_rods_total_controller.py \
    --confirm-live \
    --resume-after-place-above-index 7 --end-index 7 \
    --turn-method velocity \
    --turn-validation-ok \
    --allow-turn-motion-error-2 \
    --allow-estop-pedal-fault \
    --rack-centering-mode guarded \
    --rack-pose-samples 8 \
    --rack-pose-min-range-m 0.8 \
    --rack-pose-max-range-m 1.6 \
    --rack-pose-lateral-half-width-m 0.4 \
    --rack-pose-z-min-m 0.6 \
    --rack-pose-z-max-m 1.2 \
    --rack-pose-bin-width-m 0.25 \
    --rack-pose-min-cluster-points 20 \
    --rack-place-pose-min-range-m 0.8 \
    --rack-place-pose-max-range-m 1.6 \
    --rack-place-pose-lateral-half-width-m 0.5 \
    --rack-place-pose-z-min-m 0.7 \
    --rack-place-pose-z-max-m 1.3 \
    --rack-place-pose-bin-width-m 0.25 \
    --rack-place-pose-min-cluster-points 20 \
    --rack-near-target-skip-centering-margin-mm 80 \
    --rack-post-approach-front-retry-windows 2 \
    --front-too-close-safe-backoff-retries 2 \
    --retreat-method front-ultrasonic \
    --retreat-target-tolerance-mm 20 \
    --retreat-odom-tolerance-m 0.02 \
    --grab-target-avg-accept-span-mm 25 \
    --log-file logs/live_resume_rod7_place_above_roi_high05_20260612_$(date +%H%M%S).log'
```

Use a fresh `--log-file` for every physical rerun. The controller now blocks
live execution if the selected log/jsonl/checkpoint/report artifact already
exists, so a reset-and-rerun cannot accidentally mix evidence from two attempts.

Expected behavior:

- It resumes at rod 7 place approach. It must not reopen/regrab or redo the
  grab-side retreat/right turn.
- Continue monitoring:
  - `rack_pre_approach_guarded_check`
  - `rack_post_approach_guarded_check`
  - `front_ultrasonic_retreat_odom_crosscheck`
  - `front_ultrasonic_retreat_odom_auto_correction_*`
  - final `1m +/-20mm` retreat result
  - final yaw validation

## If A Later Rod Blocks

Do not restart the same rod from the beginning without checking the current
physical state. Use the report's `resume_hint` first.

Likely resume options for rod N:

- After grab pull but before successful grab retreat:
  `--resume-after-grab-pull-index N`
- After grab retreat completed and ready to turn/place:
  `--resume-after-grab-retreat-index N`
- After reaching place-above and still holding the rod:
  `--resume-after-place-above-index N`
- After placing, opening the gripper, pulling clear, and interrupting during
  place retreat:
  `--resume-after-place-retreat-target-index N
  --place-retreat-front-target-mm <from report resume_hint>`

If postcheck blocks at place after approach, the rod may still be held near the
rack. First inspect the report and current front ultrasonic distance; do not
blindly repeat the whole rod.

## Key Caution

The post-approach check is intentionally conservative. It protects the arm
actions after the chassis has moved close to the rack. It is not active crab
correction. Active lateral crab walking is still gated because `linear.y`
direction/gain is not fully production-calibrated.
