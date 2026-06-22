# 2026-06-15 Map20 Live Stop Handoff

Active robot and workspace:

- robot: `agi@192.168.0.7`
- remote workspace: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1`
- local workspace: `/home/davie/G2/G2_dipan_yundong`
- active map: `20`

No more motion commands were sent after the final read-only status check on
2026-06-15.

## Synced Code State

The following behavior is present locally and was synced to the robot before the
last live run:

- `grab_final_stop_mm=308`, `grab_final_brake_margin_mm=20`
- `place_final_stop_mm=308`, `place_final_brake_margin_mm=20`
- no extra `place_forward_after_fine_m` offset after place fine positioning
- place sequence includes `arm_place_transition2`
- `retreat_after_place` accepts `rear_obstacle` as a successful non-motion
  stop condition, then continues to upper-body recovery
- `LOCAL_PICK` now returns waist/body to home after pick before navigating to
  `PLACE_PRE`

Key files:

- `rack_hybrid_docking_package/industrial_cell_mission_controller.py`
- `rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py`
- `rack_hybrid_docking_package/calibration_records/rod07_place_waist_adjusted_latest.json`
- `rack_hybrid_docking_package/calibration_records/rod07_place_above_arm_latest.json`
- `rack_hybrid_docking_package/calibration_records/rod07_place_transition_arm_latest.json`
- `rack_hybrid_docking_package/calibration_records/rod07_place_transition2_arm_latest.json`
- `rack_hybrid_docking_package/calibration_records/rod07_place_final_arm_latest.json`
- `rack_hybrid_docking_package/calibration_records/place_fine_rack_distance_latest.json`

The current `rod07_place_above_arm_latest.json` is the lower validated place
above point after the earlier high point was rejected by GDK. It was finally
saved with arm joints around:

- `idx24_arm_l_joint4=-2.4473524969349394`
- `idx64_arm_r_joint4=-2.443979297150829`

## Last Live Run

Started rod 5 through rod 7 with:

```bash
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --live --start-index 5 --end-index 7 \
  --allow-holding-resume \
  --run-log logs/live_rod5_to_7_place_retreat_tolerate_20260615_192856.log
```

Rod 5 progress:

- readiness passed
- `NAV_TO_GRAB` reached `GRAB_PRE`
- rod 5 pick completed
- grab fine positioning used target `308mm`, brake margin `20mm`, and stopped
  at front filtered distance about `330mm`
- pick pull-back and `retreat_after_pick=0.45m` completed
- waist/body returned home after pick
- `NAV_TO_PLACE` reached `PLACE_PRE`
- place waist and place above succeeded
- place fine positioning used target `308mm`, brake margin `20mm`, and stopped
  at front filtered distance about `338mm`
- `arm_place_transition`, `arm_place_transition2`, and `arm_place_pose`
  succeeded
- grippers opened successfully
- all place pull-back offsets after release succeeded

The run then stopped during upper-body recovery after place:

- `retreat_after_place` did not move the chassis because rear ultrasonic
  precheck reported `rear_filtered_mm=203`
- this was treated as an accepted `rear_obstacle` stop, as intended
- `arm_default_after_place` then failed twice with:
  `Joint position control request failed to transit to PLANNING state within timeout`
- a later standalone retry of:
  `move_arm_by_json_path.py --json /data/wxf/wxf/positions/arm_default.json`
  failed with the same PLANNING timeout

Important physical interpretation:

- rod 5 was already placed and the grippers were opened
- the checkpoint was not advanced after rod 5 because the failure happened in
  `arm_default_after_place`
- the robot is therefore not physically holding a rod, but the checkpoint still
  says `holding_rod=true`
- do not trust that checkpoint as a direct resume source

## Current Stop State

No relevant mission or move process was left running after the failure.

Current robot-side checkpoint:

```json
{
  "rod_index": 5,
  "end_index": 7,
  "phase": "LOCAL_PLACE",
  "holding_rod": true,
  "current_station": "PLACE_PRE",
  "last_success_step": "NAV_TO_PLACE"
}
```

This checkpoint is stale relative to the physical state because rod 5 has
already been released.

Final read-only status snapshot after stopping:

- charge plug: disconnected, `charge_plug_insert_state=0`, `0V / 0A`
- `motion_control_error=0`
- `motion_control mode=0`
- whole-body errors: right arm `0`, left arm `0`, right end `0`, left end `0`,
  waist `0`, chassis `0`
- PNC task state: `7`
- odom velocity samples: `0.0m/s`
- front ultrasonic around `269-270mm`
- right ultrasonic valid around `1555mm`
- rear ultrasonic was invalid in the final read-only samples
- robot pose was still near `PLACE_PRE`
- arms were not at `/data/wxf/wxf/positions/arm_default.json`
- waist/body was still in the place waist posture, not home

## Tomorrow Start Procedure

Do not issue chassis or navigation commands first.

1. Run a read-only process and status check:

```bash
ps -eo pid,ppid,stat,etime,cmd | grep -E "industrial_cell_7_rods_optimized|industrial_cell_mission_controller|move_|industrial_map_nav_guarded" | grep -v grep
python3 industrial_status_snapshot.py --samples 3 --interval-s 0.1
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py --status-only
```

2. Treat `logs/industrial_cell_7_rods_optimized_checkpoint.json` as stale. It
   says rod 5 is still in `LOCAL_PLACE` and `holding_rod=true`, but the rod was
   physically released.

3. Recover the upper body before any full-cycle continuation:

- first capture/read the current arm and waist joint pose
- do not keep retrying a direct move to
  `/data/wxf/wxf/positions/arm_default.json`; it already failed from this
  posture
- use a staged upper-body recovery path or manual jogging if needed
- after arms are in a safe non-interfering posture, return waist/body to the
  default/home posture

4. Only after the upper body is safe and a fresh readiness check is clean, choose
   the next live action:

- if the chassis is at or has been verified/recovered to `HOME_SAFE`, start a
  new run from rod 6:

```bash
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --live --init --start-index 6 --end-index 7 \
  --run-log logs/live_rod6_to_7_YYYYMMDD_HHMM.log
```

## 2026-06-16 Rod 6 Live Run Stopped After Place

Robot network became reachable again at `agi@192.168.0.7`, so the rod-6
continuation was started from the handoff path. The stale rod-5 checkpoint was
not resumed.

Preparation completed on the robot:

- Read-only checks:
  - no lingering mission/move process before the live run
  - `charge_plug_insert_state=0`
  - `motion_control_error=0`
  - whole-body errors were `0`
  - readiness check passed with `ok=true`
  - current chassis pose was at `HOME_SAFE` within tolerance
- Direct +5 cm place pose:
  - generated robot-side
    `rack_hybrid_docking_package/calibration_records/rod07_place_final_arm_up050_latest.json`
  - generated from the already validated `up020` pose plus an additional
    `+0.03m` Z offset, so net height is old final `+0.05m`
  - calibration helper returned arms and waist/body to
    `/data/wxf/wxf/positions/arm_default.json`
- Robot-side runner was patched so `place_pose_json` points to
  `rod07_place_final_arm_up050_latest.json`.
- Robot-side `py_compile` passed.
- No-motion rod-6 plan passed and confirmed:
  - `arm_place_pose` uses `rod07_place_final_arm_up050_latest.json`
  - no `place_raise_before_open_offset`

Live command used:

```bash
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --live --init --start-index 6 --end-index 6 \
  --checkpoint-file logs/live_rod6_up050_checkpoint_20260616_0913.json \
  --run-log logs/live_rod6_up050_20260616_0913.log
```

Confirmed live progress before the stop:

- `NAV_TO_GRAB` reached `GRAB_PRE` and yaw refine completed.
- `LOCAL_PICK` completed:
  - grippers opened
  - waist and arms moved to rod-6 grab pose
  - grab fine positioning stopped at front filtered distance about `331mm`
  - grippers closed
  - pick pull-back offsets completed
  - `retreat_after_pick=0.45m` completed
  - waist returned home after pick
- `NAV_TO_PLACE` reached `PLACE_PRE` and yaw refine completed.
- `LOCAL_PLACE` completed the actual rod release:
  - place waist and place-above succeeded
  - place fine positioning stopped at front filtered distance about `331mm`
  - `arm_place_transition` succeeded
  - `arm_place_transition2` succeeded
  - `arm_place_pose` using `rod07_place_final_arm_up050_latest.json` succeeded
  - `open_gripper_place` succeeded
  - all place pull-back/drop offsets after release succeeded
  - `retreat_after_place` stopped with accepted `rear_obstacle`,
    `rear_filtered_mm=260`

Stop/failure:

- The run failed at `arm_default_after_place`.
- Attempt 1 failed with:
  `Joint position control request failed to transit to PLANNING state within timeout`
- Attempt 2 failed with:
  `JointControlRequest timeout`
- This matches the previous rod-5 post-place recovery failure mode.
- The rod-6 business action was already completed before this failure: the rod
  was placed, grippers were opened, and arm pull-back offsets completed.
- Because the failure happened before `LOCAL_PLACE` advanced the checkpoint, the
  live checkpoint is expected to still say `rod_index=6`, `phase=LOCAL_PLACE`,
  `holding_rod=true`, `current_station=PLACE_PRE`, even though physically the
  robot should no longer be holding the rod.

Important blocker after the stop:

- Immediately after the failure, SSH to `agi@192.168.0.7` timed out and then
  returned `No route to host`.
- Backup SSH probes to `10.20.15.199`, `192.168.168.45`, and `10.20.15.169`
  also timed out.
- Therefore no post-stop read-only status snapshot or upper-body recovery could
  be performed in this continuation.
- The robot-side `rod07_place_final_arm_up050_latest.json` was not synced back
  to the local workspace because the network dropped.

Next required action after network returns:

1. Do not start rod 7 and do not resume from the checkpoint directly.
2. Run read-only process/status checks first:

```bash
pgrep -af 'industrial_cell_7_rods_optimized|industrial_cell_mission_controller|move_|industrial_map_nav_guarded'
source /home/agi/app/env.sh
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
python3 industrial_status_snapshot.py --samples 3 --interval-s 0.1
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --checkpoint-file logs/live_rod6_up050_checkpoint_20260616_0913.json \
  --status-only
```

3. Treat the rod-6 checkpoint as stale relative to physical state if it still
   says `holding_rod=true`.
4. Recover the upper body before any chassis/navigation continuation. Do not
   keep retrying direct `arm_default_after_place` from the stopped posture.
   The likely staged recovery is to first lift both end effectors upward from
   the post-place pull-back posture, then retry a slower absolute move to
   `/data/wxf/wxf/positions/arm_default.json`, but only after the read-only
   status confirms no active errors and the operator confirms clearance.
5. After arms and waist/body are home and a fresh readiness check is clean,
   start rod 7 with a new initialized checkpoint only if the operator wants to
   continue:

```bash
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --live --init --start-index 7 --end-index 7 \
  --checkpoint-file logs/live_rod7_up050_checkpoint_YYYYMMDD_HHMM.json \
  --run-log logs/live_rod7_up050_YYYYMMDD_HHMM.log
```

## 2026-06-15 Local Patch: Skip Final Place Point

Operator correction after the rod-6 stop:

- Do not go to the final `arm_place_pose` place point anymore.
- Keep `arm_place_transition2` as the last arm waypoint before opening the
  grippers.
- `arm_place_transition2` must remain a normal dual-arm joint JSON move.
- Cancel the separate final place point and any left-arm-only forward 3 cm
  behavior.

Local patch prepared, with no robot motion:

- `rack_hybrid_docking_package/industrial_cell_mission_controller.py`
  now supports `--skip-place-pose-after-transition2`.
- When this flag is set in waypoint LOCAL_PLACE mode, the plan becomes:

```text
waist_place_straight ->
arm_place_above ->
place_fine_position ->
arm_place_transition ->
arm_place_transition2 ->
open_gripper_place ->
...
```

- `arm_place_pose` is not appended in that mode.
- `rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py` now sets
  `skip_place_pose_after_transition2=True` and passes
  `--skip-place-pose-after-transition2`.
- The optimized wrapper no longer requires `place_pose_json` in its local file
  check when this skip is active. `rod07_place_final_arm_up050_latest.json` is
  therefore no longer required for future runs that use this mode.

Local validation:

- `python3 -m py_compile` passed for:
  - `rack_hybrid_docking_package/industrial_cell_mission_controller.py`
  - `rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py`
- A local `full-dry-run` LOCAL_PLACE plan using temporary `/tmp` inputs
  confirmed:
  - `arm_place_transition2` is present
  - `open_gripper_place` immediately follows it
  - `arm_place_pose` is absent from the active plan
  - no left-arm-only offset appears in the active plan
- Local hashes after the patch:
  - `industrial_cell_mission_controller.py`
    `1f2fbfdcae1a4c66bce08fdf9e16f12c90cef2a556ecd67cccd63d10046435e5`
  - `industrial_cell_7_rods_optimized.py`
    `d01f72fb77b38eb1ad2ea95bd6924dddc43d71233a2af8643e192687b3e36ef7`

Robot sync status:

- Robot `agi@192.168.0.7` was still unreachable with `No route to host`.
- This patch has not been synced to the robot.
- Do not continue rod 7 until the robot-side files are updated and a robot-side
  no-motion plan confirms the same `transition2 -> open_gripper_place` order.

Required next steps after network returns:

1. First run the read-only post-stop checks from the previous section.
2. Sync these two local files to
   `/data/g2_industrial_cell_20260612/wxf/BOX_528_1`:
   - `rack_hybrid_docking_package/industrial_cell_mission_controller.py`
   - `rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py`
3. Run robot-side `py_compile`.
4. Run a robot-side no-motion LOCAL_PLACE/rod-7 plan check and verify:
   - no `arm_place_pose`
   - `arm_place_transition2` is a dual-arm `move_arm_by_json_path.py` step
   - `open_gripper_place` follows `arm_place_transition2`
5. Only then recover the upper body and consider a fresh initialized rod-7 run.

- if the chassis is still near `PLACE_PRE`, recover or navigate to a known safe
  station first, then start rod 6

Do not resume from rod 5 unless the operator explicitly wants to redo rod 5.

## 2026-06-15 Continuation Attempt: Place Point +5cm

Operator feedback: the current place/release point is too low and should be
raised by 5 cm before continuing rod 6.

Local-only patch prepared:

- `rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py` now points
  `place_pose_json` at
  `rack_hybrid_docking_package/calibration_records/rod07_place_final_arm_up050_latest.json`.
- `place_raise_before_open_z_m` is `0.0`.
- This intentionally uses a direct higher final place joint JSON instead of
  moving to the old low final pose and then adding a relative lift before
  opening the grippers.

Validation completed locally only:

- `python3 -m py_compile` passed for:
  - `rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py`
  - `rack_hybrid_docking_package/industrial_cell_mission_controller.py`
  - `rack_hybrid_docking_package/calibrate_direct_place_pose_offset.py`
- Dry-run of the calibration helper passed with no GDK init and no motion:

```bash
python3 rack_hybrid_docking_package/calibrate_direct_place_pose_offset.py \
  --dry-run --z-m 0.05 \
  --output-json rack_hybrid_docking_package/calibration_records/rod07_place_final_arm_up050_latest.json
```

Important blocker:

- The robot at `agi@192.168.0.7` was not reachable from the local machine.
- SSH failed with `No route to host`, including an escalated retry.
- No robot-side file sync, read-only status check, upper-body recovery, or live
  motion command was sent in this continuation attempt.
- A local no-motion wrapper test reset the local
  `logs/industrial_cell_7_rods_optimized_checkpoint.json` to rod 6. Treat this
  local file as a test artifact only; it is not evidence of the robot-side
  state.

When the robot network is reachable again, do not start rod 6 immediately.
Resume with this order:

1. Run the read-only process/status checks from the start procedure above on
   `agi@192.168.0.7`.
2. Treat the robot-side stale rod-5 checkpoint as physical-state-inaccurate
   until the read-only checks confirm otherwise.
3. Recover the upper body first. Do not keep retrying a direct move to
   `/data/wxf/wxf/positions/arm_default.json` from the stopped place posture.
4. After the upper body is safe, generate the direct +5 cm final place JSON on
   the robot:

```bash
python3 rack_hybrid_docking_package/calibrate_direct_place_pose_offset.py \
  --confirm-physical --z-m 0.05 \
  --output-json rack_hybrid_docking_package/calibration_records/rod07_place_final_arm_up050_latest.json
```

If moving through the old final place pose is judged unsafe during recovery,
do not run the helper; manually jog/capture the +5 cm place point instead.

5. Run a no-motion plan check and confirm LOCAL_PLACE uses
   `rod07_place_final_arm_up050_latest.json` and has no
   `place_raise_before_open_offset`.
6. Only after the chassis is verified/recovered to `HOME_SAFE`, start a new
   initialized run from rod 6:

```bash
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --live --init --start-index 6 --end-index 7 \
  --run-log logs/live_rod6_to_7_YYYYMMDD_HHMM.log
```

## 2026-06-16 10.42.1.101 System Diagnosis

Operator reported that the robot was now reachable at `agi@10.42.1.101`, the
arms looked not enabled, and hard reboots had not brought the robot completely
up.

Read-only / non-motion checks and actions performed:

- Used the same remote workspace:
  `/data/g2_industrial_cell_20260612/wxf/BOX_528_1`.
- No chassis, navigation, arm, gripper, or waist motion command was sent.
- `genie_app.service` was initially blocked by the time-sync guard while the
  robot clock was still near 1970. Chrony later stepped the clock to
  2026-06-16 09:43:40 CST.
- `wifi_client.service` was restarting every 3 seconds and repeatedly restarting
  `dnsmasq.service`; `dnsmasq` failed with `unknown interface wlanap0`.
  The runtime loop was stopped with:

```bash
echo 1 | sudo -S systemctl stop wifi_client.service dnsmasq.service
```

- `genie_app.service` was restarted once after time sync was valid. The restart
  passed `/usr/local/sbin/g2_time_sync_guard.sh` and created
  `/data/logs/boot00000313`.
- Arm-side read-only state was clean:
  - `motion_control_error=0`
  - whole-body right/left arm errors `0`
  - end-effector, waist, lift, neck, and chassis errors `0`
  - `right_arm_control=False` and `left_arm_control=False` meant no active arm
    control task, not a confirmed arm hardware fault.

Confirmed startup/localization issue:

- On app restart, SLAM attempted relocalization before all TF publishers were
  connected.
- In `/data/logs/slam_state_machine.log.INFO.20260616-095607.81291.0`,
  relocalization failed at 09:56:23 with repeated:
  `Transform lookup failed: "base_link" ... does not exist`.
- DR/tagloc `/tf` connected only after 09:56:27, and motion-control `/tf`
  connected only after 09:56:40.
- A non-motion retry was then published to `/slam/global_loc_request` with
  `control=1`, `relocalization_mode=0`. The publish calls returned `True`.
- After that retry, readiness no longer reported `pose_unavailable` or
  `odom_velocity_unavailable`; odom samples were readable and stopped.
- SLAM logs still reported `Localization Abnormal`, sampled odom showed
  `loc_confidence=0`, and a later current pose sample drifted to obviously
  invalid coordinates. Treat odom/pose as recovered for API availability only,
  not as production navigation quality.

Current hard blockers after the non-motion repair attempt:

- The charge plug bit remains set: `charge_plug_insert_state=1`.
- Charging readings are unstable. A read-only sample showed real charging at
  about `51.5V / 15A` with battery charging status `1`; the last readiness
  sample had `1.0V / 0A` but still reported `charge_plug_insert_state=1`.
- The latest readiness check therefore still failed:

```json
{
  "ok": false,
  "problems": [
    "charge_plug_insert_state=1"
  ],
  "map_id": 20,
  "motion_control_error": 0,
  "pnc_task_state": 0
}
```

Rod workflow state remains stale:

- `logs/live_rod6_up050_checkpoint_20260616_0913.json` still says
  `rod_index=6`, `phase=LOCAL_PLACE`, `holding_rod=true`,
  `current_station=PLACE_PRE`.
- This checkpoint must not be resumed directly. The physical interpretation from
  the prior live run still stands: rod 6 was placed, grippers were opened, and
  the failure happened in `arm_default_after_place`.

Next safe order:

1. Do not start rod 7 and do not run chassis/navigation while
   `charge_plug_insert_state=1`.
2. Verify the physical charging connection and charger contact. Require
   `charge_plug_insert_state=0` and near-`0A` current before any chassis motion.
3. Re-run read-only readiness. Require both usable odom/current pose and no
   charging blocker before any chassis motion.
4. Treat `Localization Abnormal`, `loc_confidence=0`, or impossible current pose
   coordinates as localization-quality blockers even if the current readiness
   script only reports charging.
5. Only after operator confirmation of physical clearance, recover the upper
   body with a staged arm path. Do not retry the direct
   `/data/wxf/wxf/positions/arm_default.json` move from the stopped post-place
   posture as the first recovery motion.

## 2026-06-16 Recovery Continued on 192.168.0.7

The `10.42.1.101` SSH path timed out again, but the same robot was reachable at
`agi@192.168.0.7`. Robot network interfaces on that host still included
`xfi2.10g=10.42.1.101`, so `192.168.0.7` was used as the live SSH entrypoint.

No physical motion was sent in this continuation.

System state recovered:

- `/data/logs/latest` advanced to `/data/logs/boot00000314`, so the robot had
  restarted or relaunched services after the previous diagnosis.
- `wifi_client.service` came back enabled and restarted every 3 seconds,
  repeatedly restarting `dnsmasq.service`.
- `wifi_client.service` was persistently disabled and stopped:

```bash
echo 1 | sudo -S systemctl disable --now wifi_client.service
```

- Final service state:
  - `wifi_client.service`: `disabled`, `inactive`
  - `dnsmasq.service`: `inactive`
  - `genie_app.service`: `active`
  - `chrony.service`: `active`
- Final readiness check passed:

```json
{
  "ok": true,
  "map_id": 20,
  "charge_plug_insert_state": 0,
  "charge_input_current_a": 0.0,
  "charge_input_voltage_v": 0.0,
  "motion_control_error": 0,
  "pnc_task_state": 7,
  "current_pose": {
    "position": {
      "x": -1.739704,
      "y": 0.539411,
      "z": 0.012686
    }
  }
}
```

Robot-side code sync completed:

- Before syncing, robot-side backups were written with suffix
  `.20260616_1032_before_skip_place_sync.bak`.
- These local files were synced to the robot and now match local hashes:
  - `industrial_cell_mission_controller.py`
    `1f2fbfdcae1a4c66bce08fdf9e16f12c90cef2a556ecd67cccd63d10046435e5`
  - `industrial_cell_7_rods_optimized.py`
    `d01f72fb77b38eb1ad2ea95bd6924dddc43d71233a2af8643e192687b3e36ef7`
- Robot-side `py_compile` passed for both files.

Place transition correction:

- The old robot-side `rod07_place_transition2_arm_latest.json` still contained
  metadata saying it was generated by moving both arms down, then moving the
  left end effector forward `+0.03m`.
- This conflicted with the operator correction: no final place point and no
  left-arm-only forward 3 cm.
- No intermediate joint snapshot existed for "both arms down only, no left
  forward". To avoid inventing unvalidated IK, `rod07_place_transition2_arm_latest.json`
  was conservatively replaced with a duplicate of the validated
  `rod07_place_transition_arm_latest.json`, with metadata explaining that the
  left-only forward component was canceled.
- Backups:
  - `rod07_place_transition2_arm_latest.json.bak_before_20260616_1036_cancel_left_forward`
  - `place_transition2_future_latest.json.bak_before_20260616_1036_cancel_left_forward`
- The corrected transition2 JSON was synced back to the local workspace.
- Local and robot-side hash:
  `0091826ac923300ae2fdbdc1827446f917784f6df95a2729bbd4737fffe2816f`

No-motion validation:

- Robot-side dry-run command:

```bash
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --init --start-index 7 --end-index 7 \
  --run-log logs/dryrun_rod7_skip_place_pose_no_left_forward_20260616_1036.log
```

- Dry-run completed with return code `0`.
- LOCAL_PLACE plan now confirms:
  - `arm_place_transition`
  - `arm_place_transition2`
  - `open_gripper_place`
  - no `arm_place_pose`
  - no `place_raise_before_open_offset`
  - transition2 metadata says the left-arm-only `+0.03m` was canceled.

Remaining live-run blockers:

- The robot is physically near `PLACE_PRE`, not `HOME_SAFE`.
- The stale rod-6 checkpoint still must not be resumed directly.
- A direct `Robot.get_joint_states()` probe returned
  `RuntimeError: Failed to get joint states`, even though readiness and
  `industrial_status_snapshot.py` reported clean motion/whole-body/odom state.
- Do not start rod 7 until the upper body is recovered or explicitly accepted
  by the operator as safe, and the robot is navigated or otherwise verified at a
  known safe start station.

## 2026-06-16 Rod 7 Completed After Clearance Confirmation

Operator confirmed there was no physical interference/obstruction before any
recovery motion was sent.

Upper-body recovery was performed as a staged recovery instead of retrying the
previous failing direct `arm_default_after_place` path:

- lifted both end effectors by `+0.04m`
- moved arms to the corrected no-left-forward
  `rod07_place_transition2_arm_latest.json`
- moved arms to `/data/wxf/wxf/positions/arm_default.json`
- moved waist/body to `/data/wxf/wxf/positions/arm_default.json`
- navigated from `PLACE_PRE` to `HOME_SAFE`

All recovery steps completed successfully. The waist recovery ran in 6
segments, with final max error about `0.000382rad`. The navigation to
`HOME_SAFE` arrived with about `0.02m` xy error and about `2.75deg` yaw error.

Rod-7 live command used:

```bash
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --live --init --start-index 7 --end-index 7 \
  --checkpoint-file logs/live_rod7_skip_place_pose_no_left_forward_checkpoint_20260616_1046.json \
  --run-log logs/live_rod7_skip_place_pose_no_left_forward_20260616_1046.log
```

Rod 7 completed end to end:

- `NAV_TO_GRAB` reached `GRAB_PRE`; yaw refine completed.
- `LOCAL_PICK` completed; grab fine positioning stopped at front filtered
  distance about `339mm`.
- `NAV_TO_PLACE` reached `PLACE_PRE`; yaw refine completed.
- `LOCAL_PLACE` used the corrected plan:
  `arm_place_transition -> arm_place_transition2 -> open_gripper_place`.
- There was no final `arm_place_pose` and no left-arm-only forward `+0.03m`
  action.
- `open_gripper_place` succeeded, so rod 7 was released.
- Post-release pull-back/drop offsets, `retreat_after_place`,
  `arm_default_after_place`, and `waist_home_after_place` all succeeded.
- `NAV_TO_RECOVERY` reached `RECOVERY_SAFE`.
- `NAV_TO_HOME` reached `HOME_SAFE`.
- Runner exited with return code `0`.

Final rod-7 checkpoint:

```json
{
  "rod_index": 7,
  "end_index": 7,
  "phase": "MISSION_DONE",
  "holding_rod": false,
  "current_station": "HOME_SAFE",
  "last_success_step": "ROD_DONE",
  "next_action": null
}
```

Final read-only checks after completion:

- no remaining `industrial_cell_7_rods_optimized`,
  `industrial_cell_mission_controller`, `move_`, or
  `industrial_map_nav_guarded` process was left running
- `wifi_client.service`: `disabled`, `inactive`
- `genie_app.service`: `active`
- `chrony.service`: `active`
- charge plug disconnected: `charge_plug_insert_state=0`, `0V / 0A`
- `motion_control_error=0`
- whole-body errors: right arm `0`, left arm `0`, right end `0`, left end `0`,
  waist `0`, chassis `0`
- PNC task state: `7`
- odom velocity samples: `0.0m/s`
- localization confidence: `80`
- front ultrasonic minimum ranged about `1142-1290mm`; right about
  `1836-1887mm`; rear ultrasonic remained invalid in these final samples

Current state:

- map `20`
- robot is at `HOME_SAFE`
- not holding a rod
- all seven rods are complete for this run

Do not resume the old rod-5 or rod-6 stale checkpoints. For any future run,
start from a fresh initialized checkpoint after a normal read-only readiness
check.

## 2026-06-15 22:06 PDT - Round-2 Stop: Right Gripper Blue LED / No Physical Motion

After the successful rod-7 completion above, a fresh full-round rerun was
started with:

```bash
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --live --init --start-index 1 --end-index 7 \
  --checkpoint-file logs/live_round2_skip_place_pose_no_left_forward_checkpoint_20260616_1050.json \
  --run-log logs/live_round2_skip_place_pose_no_left_forward_20260616_1050.log
```

The rerun completed rod 1 and entered rod 2. The operator then observed that
the right gripper did not physically open/close. The live runner/controller/nav
processes were stopped with SIGINT. Do not resume this round-2 checkpoint
directly because the physical robot pose and the checkpoint are no longer a
safe resume pair.

Last known round-2 checkpoint at stop time:

```json
{
  "rod_index": 2,
  "end_index": 7,
  "phase": "LOCAL_PICK",
  "holding_rod": false,
  "current_station": "GRAB_PRE",
  "last_success_step": "NAV_TO_GRAB"
}
```

Right-gripper diagnostics performed:

- GDK `move_ee_pos()` right-gripper open command returned `ret=0`.
- Right end feedback stayed near `position=120.0`, `enable=True`,
  `err_code=0`; no physical open motion was seen by the operator.
- Left-end test did not prove a side-mapping issue.
- `/wbc/retarget` and `/wbc/joint_position_control` right-tool commands did
  not move the gripper.
- `/hal/fault_clear` for the right side was accepted; after that, right end
  status changed to `2`, but position still stayed at `120.0`.
- `get_whole_body_status()` showed arm/end error fields at `0` during the
  checks, and `/monitor/g02_fault_topic` later showed `counter: 0`.
- EtherCAT slaves were checked with sudo; all 18 slaves were OP, including
  slave 17/right end plate. The EtherCAT bus itself did not look down.
- HAL exposes `/hal/fault_clear` and `/hal/set_zero_pose`; no live
  end-specific reset/init service was found.
- Operator observation: right gripper LED stayed blue while the left gripper
  LED stayed green.

Interpretation:

- This no longer looks like a wrong high-level gripper command path. Commands
  were accepted by the software stack, but the right gripper feedback stayed
  fixed and the hardware did not move.
- The blue LED on the right gripper is the strongest current clue: treat the
  right gripper/end-effector module as not ready or not fully initialized even
  though the upper software fault fields are clear.
- Do not continue rods or send more blind gripper open/close commands until
  the right gripper LED/state is corrected and a no-motion status check
  confirms normal readiness.

Network state after the gripper checks:

- `ssh agi@192.168.0.7` first showed the robot time as
  `Thu Jan 1 08:00:52 AM CST 1970`, then became unreachable.
- Current local host is connected to Wi-Fi `ZTE_5GCPE_4D56_5G` as
  `192.168.0.9/24`.
- `192.168.0.7` is not reachable from this LAN; ARP is `FAILED`.
- `10.42.1.101` does not answer ping/SSH from the current host because this
  host is not on the `10.42.1.x` robot network.

Next safe steps:

1. Put the operator PC back onto the robot network that can reach
   `10.42.1.101`, or restore the robot to the `192.168.0.x` LAN.
2. Before any motion, run read-only SSH checks only:

   ```bash
   source /home/agi/app/env.sh
   cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
   python3 industrial_status_snapshot.py --samples 1 --interval-s 0.1
   pgrep -af 'industrial_cell_7_rods_optimized|industrial_mission_controller|industrial_map_nav_guarded|move_ee_pose|python3.*rack_hybrid'
   systemctl --no-pager --full status genie_app chrony ethercat
   ```

3. Physically inspect or reseat the right gripper/end-effector power and signal
   connection if the LED remains blue. If the right module has a documented
   hardware reset/initialize procedure, use that before another live run.
4. Only after the right gripper LED/state becomes normal and a read-only
   status snapshot is clean, recover the robot from the actual physical pose.
   Do not reuse the stale round-2 rod-2 checkpoint as an automatic resume
   point.

## 2026-06-15 22:53 PDT - System Recovery and Right-Gripper Open/Close Test

The robot was reconnected on `10.42.1.101` after switching the operator PC to
the robot network. The system had not fully started:

- robot time was reset to `1970-01-01`
- `genie_app.service` was stuck behind or not yet through
  `/usr/local/sbin/g2_time_sync_guard.sh`
- read-only GDK snapshot initially failed because Aorta/discovery
  `10.42.1.101:2379` was not available
- after one later reboot, `genie_app.service` and `chrony.service` were both
  inactive while `ethercat.service` was active

Recovery performed:

- manually set robot system time from the operator host clock
- started `chrony.service`, `share_time_sync.service`, and
  `genie_app.service`
- verified `genie_app.service`, `chrony.service`, `share_time_sync.service`,
  and `ethercat.service` active
- verified Aorta listening on `10.42.1.101:2380` and `*:2379`
- verified `MotionControlStatus mode=5 error_code=0`
- verified whole-body errors clear:
  right arm `0`, left arm `0`, right end `0`, left end `0`, waist `0`,
  chassis `0`

Remaining non-mission blockers:

- `charge_plug_insert_state=1`, about `51V / 15A`
- `emergency_stop_pedal_fault_state=1`
- odom still unavailable in the read-only snapshot
- `right_arm_control=False` and `left_arm_control=False`

Do not run navigation or the seven-rods workflow while the robot is still in
this charging/no-odom state.

Right-gripper-only test performed after service recovery:

- command sent only to `right_tool`; no chassis, arm, waist, or left-gripper
  command was sent
- right open target: `-0.785`, return `0`
- feedback after open: `enable=True`, `position=0.0`, `status=0`,
  `err_code=0`
- right close target: `0.0`, return `0`
- feedback after close: `enable=True`, `position=120.0`, `status=0`,
  `err_code=0`

The right gripper was left in the closed state after the test. Software
feedback now changes correctly for right-gripper open/close, unlike the earlier
blue-LED/no-motion failure.

## 2026-06-15 23:01 PDT - Left/Right Arm Joint Communication Fault Recovery

The robot was reachable on `192.168.0.7`. `10.42.1.101` was no longer the
active SSH path from the operator host at the start of this check.

Initial read-only diagnosis:

- `genie_app.service`, `chrony.service`, `ethercat.service`, and
  `share_time_sync.service` were active.
- EtherCAT master was in Operation, active, with 18 slaves and `Lost frames: 0`.
- `ethercat slaves` showed all 18 slaves in OP.
- No seven-rods runner/controller/nav process was active.
- GDK still had joint position feedback for all 22 joints, so this was not a
  complete feedback blackout.
- Whole-body summary showed arm/end/chassis error fields as `0`, but
  `right_arm_control=False` and `left_arm_control=False`.
- GDK arm joint errors were latched across both arms:
  - most arm joints showed companion error `65535`
  - `idx23_arm_l_joint3`: `65361` (`0xff51`)
  - `idx62_arm_r_joint2`: `65361` (`0xff51`)
  - `idx63_arm_r_joint3`: `65336` (`0xff38`)
  - `idx64_arm_r_joint4`: `65361` (`0xff51`)
- `fault_manager` was publishing 14 arm joint `Communication` faults.

Authoritative HAL/SDO mapping:

- HAL was repeatedly reporting `motor 8 err code 0xff51`.
- Known mapping used:
  - `motor 0-6 -> slave 2-8`
  - `motor 7-13 -> slave 10-16`
- The first SDO scan found these locked slaves:
  - `slave 4`: `status=0x0638`, `error=0xff51`
  - `slave 11`: `status=0x0638`, `error=0xff51`
  - `slave 12`: `status=0x0638`, `error=0xff38`
  - `slave 13`: `status=0x0638`, `error=0xff51`

Recovery performed:

1. Ran the minimum no-motion clear sequence on slaves `4, 11, 12, 13`:

   ```bash
   sudo ethercat download -p <slave> -t uint16 0x3002 0x00 0x0000
   sudo ethercat states -p <slave> INIT
   sudo ethercat states -p <slave> OP
   ```

2. The first clear did not hold; the same SDO errors re-latched immediately.
3. Restarted `genie_app.service`, waited for `hal`, `gdk_service`,
   `fault_manager`, `genie_motion_control`, and `run_corobot_app` to return.
4. HAL then transitioned through the boot fault reports and finally logged:
   - `All motors ready, motor operator enable.`
   - `All motor errors cleared`

Final verification after recovery:

- all arm EtherCAT slaves `2-8, 10-16` showed `status=0x9737`,
  `error=0x0000`
- GDK arm joint error count was `0`
- `MotionControlStatus mode=5`, `error_code=0`, `error_msg=''`
- whole-body errors remained clear:
  right arm `0`, left arm `0`, right end `0`, left end `0`, waist `0`,
  chassis `0`
- grippers were still enabled and closed:
  left/right end `enable=True`, `position=120.0`, `err_code=0`
- no arm, chassis, waist, or gripper motion command was sent during this arm
  fault recovery

Remaining blockers before any navigation or seven-rods run:

- robot is still charging: `charge_plug_insert_state=1`, about `51V / 15A`
- `emergency_stop_pedal_fault_state=1`
- localization/odom became invalid in the final snapshot:
  `loc_confidence=0`, `loc_state=0`, and the reported pose was clearly
  nonsensical

Current interpretation:

- The left/right joint communication fault is recovered at the EtherCAT, HAL,
  and GDK joint-state layers.
- Do not continue the seven-rods mission yet. First remove/resolve the charging
  state and restore localization/odom confidence, then run a read-only
  readiness check again.

## 2026-06-15 23:13 PDT - Full 1-7 Rerun Requested, Blocked by Chassis/Localization Readiness

The operator requested a fresh full rerun from rod 1 through rod 7. No physical
mission was started.

Read-only preflight on `192.168.0.7`:

- robot time was sane: `Tue Jun 16 14:06 CST 2026`
- `genie_app.service`, `chrony.service`, `ethercat.service`, and
  `share_time_sync.service` were active
- no `industrial_cell_7_rods_optimized`,
  `industrial_mission_controller`, `industrial_map_nav_guarded`,
  `move_ee_pose`, or `python3.*rack_hybrid` mission process was active
- arm recovery remained good:
  `MotionControlStatus mode=5 error_code=0`, whole-body arm/end errors `0`
- stations were still calibrated:
  `GRAB_PRE`, `HOME_SAFE`, `PLACE_PRE`, `RECOVERY_SAFE`

Blocking evidence:

- robot still reports charging:
  `charge_plug_insert_state=1`, about `52V / 15A`
- `emergency_stop_pedal_fault_state=1`
- localization/odom is not usable:
  repeated `industrial_status_snapshot.py` samples reported
  `Slam odom is null` / `GetOdomInfo failed`
- `industrial_map_nav_guarded.py --capture-current-pose` failed with
  `RuntimeError: GetCurrPose failed`
- `industrial_map_nav_guarded.py --readiness-check` reported
  `Get curr map rsp timeout` and then waited on unavailable motion/map status;
  no physical command was sent

Decision:

- Do not start the full rod-1-to-rod-7 mission in this state.
- Before rerun, the operator must resolve the charging/estop condition and the
  robot must recover valid SLAM/odom/localization. Then run:

  ```bash
  source /home/agi/app/env.sh
  cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
  python3 industrial_status_snapshot.py --samples 1 --interval-s 0.1
  python3 rack_hybrid_docking_package/industrial_map_nav_guarded.py --capture-current-pose
  ```

- Only if those are clean should a fresh `--init --start-index 1 --end-index 7`
  checkpoint/log be created for the rerun. Do not reuse the stale round-2
  rod-2 checkpoint.

## 2026-06-16 14:48 CST - Round 3 Completed, Final Place Offset Patch Added

Robot/workspace used:

- robot: `agi@192.168.0.7`
- remote workspace: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1`
- active checkpoint:
  `logs/live_round3_fresh_1_7_after_arm_recovery_checkpoint_20260615_232124.json`
- final recovery log:
  `logs/live_round3_rod7_recovery_after_final_offset_patch_20260616_1444.log`

Live round result:

- The fresh 1-7 round completed through `MISSION_DONE`.
- Final checkpoint is:
  `rod_index=7`, `end_index=7`, `phase=MISSION_DONE`,
  `holding_rod=false`, `current_station=HOME_SAFE`,
  `last_success_step=ROD_DONE`.
- Final readiness check returned `ok=true`, `map_id=20`,
  `charge_plug_insert_state=0`, `motion_control_error=0`, no problems or
  warnings, and current pose near `HOME_SAFE`.

Grab fine-positioning change:

- `grab_final_stop_mm` was changed from `308` to `328` in
  `rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py`.
- Remote backup before this tuning:
  `rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py.bak_20260616_1430_grab328`.
- Rod 5, rod 6, and rod 7 all verified the new grab stop in live logs:
  `final_target_mm=328`, `final_trigger_mm=348`.

Place release change requested during rod 7:

- The operator requested: use the current release point as the place transition
  point, then make the final release point `+0.03m` forward and `-0.025m`
  down from that pose.
- Local and robot-side controller were patched:
  `rack_hybrid_docking_package/industrial_cell_mission_controller.py`
  now supports `--place-final-before-open-x-m` and
  `--place-final-before-open-z-m`.
- Optimized runner now passes:
  `--place-final-before-open-x-m 0.03`
  and `--place-final-before-open-z-m -0.025`.
- New place sequence is:
  `arm_place_transition2` -> `place_final_before_open_offset`
  `(left/right = 0.03, 0.0, -0.025)` -> `open_gripper_place`.
- `arm_place_transition2` remains a dual-arm JSON move. The final offset is
  also dual-arm. There is still no left-arm-only forward move.
- Remote backups before this patch:
  - `rack_hybrid_docking_package/industrial_cell_mission_controller.py.bak_20260616_1442_final_release_offset`
  - `rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py.bak_20260616_1442_final_release_offset`
- Remote `py_compile` passed for both patched files.
- Remote dry-run plan confirmed the order:
  1. `arm_place_transition2`
  2. `place_final_before_open_offset (0.03, 0.0, -0.025)`
  3. `open_gripper_place`

Important physical note:

- The rod-7 live release had already happened at old `transition2` before the
  interrupt reached the controller. Therefore rod 7 in this completed round did
  not use the new final offset.
- After that, only safe recovery was performed:
  `move_arm_by_json_path.py` to `/data/wxf/wxf/positions/arm_default.json`,
  `move_waist_by_json_path.py` to the same default, checkpoint corrected to
  `NAV_TO_RECOVERY`, then normal navigation to `RECOVERY_SAFE` and `HOME_SAFE`.
- Do not resume the old interrupted `LOCAL_PLACE` state; the corrected
  checkpoint is already `MISSION_DONE`.
