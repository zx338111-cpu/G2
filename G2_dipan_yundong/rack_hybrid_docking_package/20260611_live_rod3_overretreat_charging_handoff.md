# 2026-06-11 live handoff: rod3 grab retreat overrun and charging block

## Current live scope

- Local workspace: `/home/davie/G2/G2_dipan_yundong`
- Robot host: `agi@10.20.15.199`
- Robot workdir: `/data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1`
- Active production controller:
  `rack_hybrid_docking_package/industrial_7_rods_total_controller.py`
- User direction: use the old recorded grab points, not vertical stack mode.
  Do not pass `--grab-vertical-stack` or `--grab-vertical-stack-pitch-m`.

## Latest verified physical/process state

The full old-point flow was restarted from rod 1. Rod 1 completed. Rod 2 was
manually recovered after a too-close grab approach, then completed after resume.

Rod 3 status:

- Rod 3 used old script `move_arm_by_json_grab_above_第三根.py`.
- Rod 3 grab approach reached about `160 mm`.
- Gripper close and pull completed.
- The process failed during rod 3 grab retreat, before any right turn or place.
- Rod 3 has therefore been grabbed and pulled, but not placed.

Latest read-only status after the failure:

- Front ultrasonic was stable around `1620-1635 mm`.
- Expected absolute front target after this grab retreat is about `1180 mm`.
- The robot is therefore over-retreated by about `440-455 mm`.
- `motion_control_error` later cleared to `0`.
- Latest power state showed real charging:
  - `charge_plug_insert_state=1`
  - `charge_plug_input_voltage=50.0`
  - `charge_plug_input_current=15.0`
  - both batteries reported `battery_charging_status=1`
- Odom was unavailable in the latest snapshot with `Slam odom is null`.

Do not send chassis motion while `charge_plug_insert_state=1` or charging
current/voltage are present.

## Code changes already made

### `industrial_7_rods_total_controller.py`

Added grab-retreat front occlusion escape:

- New config fields:
  - `grab_retreat_front_occlusion_escape_threshold_mm`
  - `grab_retreat_front_occlusion_escape_m`
  - `grab_retreat_front_occlusion_escape_speed_mps`
- New CLI args:
  - `--grab-retreat-front-occlusion-escape-threshold-mm`
  - `--grab-retreat-front-occlusion-escape-m`
  - `--grab-retreat-front-occlusion-escape-speed-mps`
- Behavior:
  - On grab retreat only, if the stable front min is near the rack, do a short
    guarded velocity retreat first.
  - Default short escape is `0.18 m` at `0.12 m/s`.
  - Then the front-ultrasonic retreat only handles the remaining distance.

Validation already done:

- Local `python3 -m py_compile rack_hybrid_docking_package/industrial_7_rods_total_controller.py`
  passed.
- Robot-side compile passed.
- Dry-run with old point scripts and no vertical mode passed for rods 1-2.

### `guarded_front_target_recovery.py`

New recovery script:

`rack_hybrid_docking_package/guarded_front_target_recovery.py`

Purpose:

- Low-speed forward correction to an absolute front-ultrasonic target.
- Designed for the current over-retreat recovery case.
- Does not close/open gripper, turn, or continue the rod sequence.
- Reads both front sensors `0,1`; blocks if either is missing or the span is too
  large.
- Blocks on charging plug state.
- Allows stale `motion_control_error=2` only when explicitly requested and
  collision-pair lists are empty.

Validation already done:

- Local `python3 -m py_compile rack_hybrid_docking_package/guarded_front_target_recovery.py`
  passed.
- Synced to robot.
- Robot-side compile passed.
- First live attempt correctly blocked because the robot was charging.

## Important live evidence

Rod 3 retreat failure report:

`/data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1/logs/live_resume_rod2_after_manual_pull_old_points_20260611_112953_report.json`

Key failure pattern:

- During rod 3 grab retreat, front sensors disagreed badly.
- One side indicated a large over-retreat while the other was near the target.
- The controller stopped before turning, which was the right safety behavior.

Later read-only snapshot showed both front sensors agreeing around `1.63 m`,
confirming the robot is too far from the rack for the intended post-grab-retreat
position.

## Next safe resume procedure

### 1. Verify charging is disconnected

Run:

```bash
SSHPASS='<robot-password>' sshpass -e ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password \
  -o NumberOfPasswordPrompts=1 -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
  agi@10.20.15.199 \
  'source /home/agi/app/env.sh; cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1; python3 industrial_status_snapshot.py --samples 8 --interval-s 0.25'
```

Required before motion:

- `charge_plug_insert_state=0`
- charging input voltage/current not present
- `battery_charging_status=0`
- front sensors still stable and reasonably consistent

### 2. Recover rod 3 post-grab-retreat front target

Only after charging is disconnected, run:

```bash
SSHPASS='<robot-password>' sshpass -e ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password \
  -o NumberOfPasswordPrompts=1 -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
  agi@10.20.15.199 \
  'source /home/agi/app/env.sh; cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1/rack_hybrid_docking_package; python3 guarded_front_target_recovery.py --allow-estop-pedal-fault --allow-motion-error-2 --target-front-mm 1180 --tolerance-mm 30 --speed-mps 0.035 --max-forward-m 0.60 --max-duration-s 25 --max-front-span-mm 120'
```

Expected result:

- status `target_window_confirmed`
- settled front median around `1180 mm`, within `±30 mm`

If it returns `still_too_far`, do not jump to the main sequence. Re-read the
front sensors and, if still stable and not charging, repeat with a smaller
`--max-forward-m` based on the remaining error.

If it returns `overshot_too_close`, stop and recover backward slowly. Do not
turn or place.

### 3. Re-check state after recovery

Run the snapshot command again. Confirm:

- front target is stable around `1180 mm`
- charging is off
- robot is stopped
- no unexpected `motion_control_error`

### 4. Resume rod 3 after grab retreat

Only after step 3 passes, resume from rod 3 after grab retreat:

```bash
SSHPASS='<robot-password>' sshpass -e ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password \
  -o NumberOfPasswordPrompts=1 -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
  agi@10.20.15.199 \
  'source /home/agi/app/env.sh; cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1; python3 rack_hybrid_docking_package/industrial_7_rods_total_controller.py --confirm-live --allow-estop-pedal-fault --allow-turn-motion-error-2 --resume-after-grab-retreat-index 3 --end-index 7 --grab-distance-mm 155 --retreat-method front-ultrasonic --turn-method velocity --turn-validation-ok --turn-yaw-tolerance-deg 0.5 --log-file logs/live_resume_rod3_after_front_target_recovery_old_points_$(date +%Y%m%d_%H%M%S).log'
```

Do not add vertical-stack arguments.

## Follow-up engineering fix

The next controller improvement should make this recovery automatic:

- After every front-ultrasonic retreat, read a stable absolute front snapshot.
- Compare against the intended absolute target.
- If too far, perform a guarded low-speed forward correction.
- If too close, perform a guarded low-speed backward correction.
- Only allow turn/place after the absolute target window is confirmed.
- Keep the current hard stop behavior when front sensors are missing,
  inconsistent, or the robot is charging.

This change matches the user's request: "退多了就往前开一点" and "要实时检查一下".
