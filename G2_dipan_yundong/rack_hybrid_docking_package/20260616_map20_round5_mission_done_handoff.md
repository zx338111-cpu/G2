# 2026-06-16 Map20 Round 5 Mission Done Handoff

Active robot and workspace:

- robot: `agi@192.168.0.7`
- remote workspace: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1`
- local workspace: `/home/davie/G2/G2_dipan_yundong`
- active map: `20`

## Final State

Round 5 resumed from rod 4 and completed through rod 7.

Final checkpoint:

```json
{
  "rod_index": 7,
  "end_index": 7,
  "phase": "MISSION_DONE",
  "holding_rod": false,
  "current_station": "HOME_SAFE",
  "last_success_step": "ROD_DONE"
}
```

Final read-only checks after the run:

- no matching mission, navigation, arm, waist, or gripper process left running
- `motion_control_error=0`
- robot stopped, odom linear speed `0.0m/s`
- front ultrasonic minimum about `1549mm`
- right ultrasonic minimum about `1885mm`
- left ultrasonic valid sample about `2081mm`
- charge plug disconnected

Run log:

- `logs/live_round5_resume_rod4_after_pick_retreat_placepull8_20260616_1540.log`
- grep scan found no `Traceback`, `Exception`, `KeyboardInterrupt`, or nonzero
  `return_code`

## Recovery Performed

Before continuation, the physical robot had already grabbed rod 4 but had not
retreated after grab. The checkpoint was still:

```json
{
  "rod_index": 4,
  "phase": "LOCAL_PICK",
  "holding_rod": false,
  "current_station": "GRAB_PRE",
  "last_success_step": "NAV_TO_GRAB"
}
```

Do not replay `LOCAL_PICK` from the beginning in this state, because that would
open the grippers. The recovery command resumed only from `close_gripper`.

Confirmed recovery results:

- right gripper closed successfully
- left gripper closed successfully
- pick pull-back offsets completed: `-0.085m` then `-0.115m`
- `retreat_after_pick=0.45m` completed
- waist returned home
- checkpoint advanced to `rod4 / NAV_TO_PLACE / holding_rod=true`

## Parameter State

Correct final behavior now synced on the robot and compiled:

- grab retreat has no downward Z dip:
  - `pick_down_z_m=0.0`
  - `--skip-pick-down-after-close`
- final place before release:
  - `place_final_before_open_x_m=0.03`
  - `place_final_before_open_z_m=-0.025`
- after release, while retreating:
  - first pull-back/down: `x=-0.02`, `z=-0.01`
  - additional back-before-drop: `x=-0.04`, `z=0.0`
  - drop during pull-out: `z=-0.07`
  - total post-release downward clearance during pull-out: `0.08m`

Important distinction:

- release point itself is not down 8 cm
- the 8 cm downward motion happens after opening the grippers, during retreat

Robot-side backup created before the final script sync:

- `rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py.bak_20260616_1539_place_pull_down8cm_sync`

## Run Notes

Rod 4 through rod 7 all completed.

Observed but non-blocking:

- GDK printed repeated `promise is nullptr` / cleanup messages during PNC
  cancel/remote-control cleanup, but child commands returned `0` and mission
  phases advanced normally.
- rod 4 `retreat_after_place` ended with accepted `rear_obstacle` at about
  `456mm`; the controller continued upper-body recovery and finished the rod.
- later place retreats completed normally.

## Next Session

Do not resume the old interrupted rod-4 `LOCAL_PICK` state. This round is done.

Start with read-only checks:

```bash
ssh agi@192.168.0.7
source /home/agi/app/env.sh
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
python3 industrial_status_snapshot.py --samples 1 --interval-s 0.1
cat logs/live_round5_full_1_7_pickz0_placez50_checkpoint_20260616_1522.json
```

If another full physical run is needed, initialize a fresh checkpoint instead of
continuing the completed one.
