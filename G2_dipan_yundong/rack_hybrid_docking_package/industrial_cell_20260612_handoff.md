# 2026-06-12 Industrial Cell Skeleton Handoff

## Scope

This handoff covers the new map-station industrial-cell skeleton, station
calibration, and first empty-map-navigation validation for G2 seven-rods
automation. It does not validate arm or rack-docking actions.

## Development Copy

- Host: `agi@10.20.15.60`
- Project root:
  `/data/g2_industrial_cell_20260612/wxf/BOX_528_1`
- This is a development copy created from:
  `/data/g2_robot_10_20_15_199_wxf_backup_20260612_1025/wxf`
- The backup directory should remain unchanged.

## New Files

- `rack_hybrid_docking_package/industrial_station_config.json`
  - map ID and station-pose configuration;
  - `HOME_SAFE`, `GRAB_PRE`, `PLACE_PRE`, and `RECOVERY_SAFE` are calibrated
    on map `19`.
- `rack_hybrid_docking_package/industrial_map_nav_guarded.py`
  - guarded map navigation wrapper;
  - default behavior is dry-run/read-only;
  - physical navigation requires `--confirm-live`;
  - live preflight blocks charging, charge current, emergency stop,
    motion-control error, non-idle PNC, map mismatch, missing odom velocity,
    or moving robot.
  - live arrival now requires target pose tolerance and PNC idle state.
- `rack_hybrid_docking_package/industrial_cell_mission_controller.py`
  - persistent mission-state controller;
  - default status and dry-run modes do not move the robot;
  - live staging uses guarded map navigation phases;
  - local pick/place modes now include `disabled`, `noop`, `readonly`,
    `full-dry-run`, and `full`;
  - physical local pick/place requires both `--confirm-live` and
    `--confirm-local-physical`.

## Verified

On `10.20.15.60`:

- `python3 -m py_compile` passed for the two new Python scripts.
- `industrial_map_nav_guarded.py --list-stations` originally showed all stations
  as `UNSET`.
- `industrial_cell_mission_controller.py --init` created:
  `logs/industrial_cell_mission_checkpoint_dryrun.json`.
- `industrial_map_nav_guarded.py --readiness-check` was read-only and blocked
  as expected because the robot is charging:
  - `charge_plug_insert_state=1`
  - `charge_input_current=15.000>0.500`
  - `map_id=16`
  - `motion_control_error=0`
  - `pnc_task_state=7`
  - odom speed samples were all `0.0`

Additional calibration on 2026-06-12:

- The operator placed the robot at `HOME_SAFE`.
- Read-only capture wrote this station into
  `industrial_station_config.json` on the local workspace and robot development
  copy:
  - position: `x=0.608583`, `y=-0.965382`, `z=0.004137`
  - orientation: `x=-0.004672`, `y=0.001119`, `z=0.233578`, `w=0.972326`
- The current map was verified as `map_id=19`; the config was updated from the
  old placeholder `16` to `19`.
- Robot-side backup before station calibration:
  `industrial_station_config.json.bak_HOME_SAFE_20260612`.
- Dry-run navigation planning to `HOME_SAFE` accepts the calibrated target.
- Latest read-only readiness check returns `ok: true`:
  `charge_plug_insert_state=0`, charge input `0V/0A`,
  `motion_control_error=0`, `pnc_task_state=7`, odom speed samples all `0.0`.

Additional `GRAB_PRE` calibration on 2026-06-12:

- The operator placed the robot at the grab-material target point.
- Read-only capture wrote `GRAB_PRE` into `industrial_station_config.json` on
  the local workspace and robot development copy:
  - position: `x=0.37263`, `y=-0.341633`, `z=-0.00526`
  - orientation: `x=0.006085`, `y=0.00563`, `z=0.860144`, `w=0.509985`
- Robot-side backup before this station calibration:
  `industrial_station_config.json.bak_GRAB_PRE_20260612`.
- Dry-run navigation planning to `GRAB_PRE` accepts the calibrated target on
  map `19`.
- Latest read-only readiness check still returns `ok: true`:
  `charge_plug_insert_state=0`, charge input `0V/0A`,
  `motion_control_error=0`, `pnc_task_state=7`, odom speed samples all `0.0`.

Additional `RECOVERY_SAFE` calibration on 2026-06-12:

- The operator placed the robot at the recovery-safe point.
- Read-only capture wrote `RECOVERY_SAFE` into
  `industrial_station_config.json` on the local workspace and robot development
  copy:
  - position: `x=-0.844978`, `y=-1.524548`, `z=0.025406`
  - orientation: `x=-0.002417`, `y=0.005244`, `z=0.241367`, `w=0.970417`
- Robot-side backup before this station calibration:
  `industrial_station_config.json.bak_RECOVERY_SAFE_20260612`.
- Dry-run navigation planning to `RECOVERY_SAFE` accepts the calibrated target
  on map `19`.
- Latest read-only readiness check still returns `ok: true`:
  `charge_plug_insert_state=0`, charge input `0V/0A`,
  `motion_control_error=0`, `pnc_task_state=7`, odom speed samples all `0.0`.

Additional `PLACE_PRE` calibration on 2026-06-12:

- The operator placed the robot at the place-material target point.
- Read-only capture wrote `PLACE_PRE` into `industrial_station_config.json` on
  the local workspace and robot development copy:
  - position: `x=1.671997`, `y=-0.20726`, `z=-0.014003`
  - orientation: `x=0.000794`, `y=0.004637`, `z=0.239325`, `w=0.970928`
- Robot-side backup before this station calibration:
  `industrial_station_config.json.bak_PLACE_PRE_20260612`.
- Dry-run navigation planning to `PLACE_PRE` accepts the calibrated target on
  map `19`.
- Latest read-only readiness check still returns `ok: true`:
  `charge_plug_insert_state=0`, charge input `0V/0A`,
  `motion_control_error=0`, `pnc_task_state=7`, odom speed samples all `0.0`.

## Empty Navigation Validation

On 2026-06-12, empty map navigation was validated on `10.20.15.60` without any
arm or rack-docking actions.

Preflight:

- All four stations dry-run successfully on map `19`.
- Read-only readiness was `ok: true`: no charging, charge input `0V/0A`,
  `motion_control_error=0`, PNC idle, and odom speed samples `0.0`.

First full empty loop:

- `logs/empty_nav_recovery_to_home_20260612.log`
  - `RECOVERY_SAFE -> HOME_SAFE`
  - result `arrived`, elapsed `6.007s`, final `xy_error_m=0.0673`,
    `yaw_error_deg=-0.5116`
- `logs/empty_nav_home_to_grab_20260612.log`
  - `HOME_SAFE -> GRAB_PRE`
  - result `arrived`, elapsed `5.005s`, final `xy_error_m=0.0231`,
    `yaw_error_deg=-2.9618`
  - This is inside the configured `3deg` yaw tolerance but close enough that
    repeatability should be watched before connecting to rack actions.
- `logs/empty_nav_grab_to_place_20260612.log`
  - `GRAB_PRE -> PLACE_PRE`
  - result `arrived`, elapsed `8.508s`, final `xy_error_m=0.0476`,
    `yaw_error_deg=-0.5494`
- `logs/empty_nav_place_to_recovery_20260612.log`
  - `PLACE_PRE -> RECOVERY_SAFE`
  - result `arrived`, elapsed `15.529s`, final `xy_error_m=0.0665`,
    `yaw_error_deg=-0.5154`

Controller hardening after the first loop:

- The first loop exposed that `nav_result` could return `arrived` while PNC was
  still reporting `state=2`; a later readiness check returned idle.
- `industrial_map_nav_guarded.py` was patched so live arrival requires both
  target pose tolerance and `task_state in IDLE_TASK_STATES`.
- Robot-side backup before this patch:
  `industrial_map_nav_guarded.py.bak_require_idle_arrival_20260612`.
- Local and robot-side `python3 -m py_compile` passed.
- Dry-run to `HOME_SAFE` and read-only readiness check passed after the patch.
- Patch live validation:
  `logs/empty_nav_recovery_to_home_idlegate_20260612.log`
  - `RECOVERY_SAFE -> HOME_SAFE`
  - result `arrived`, elapsed `8.512s`, final `xy_error_m=0.0167`,
    `yaw_error_deg=-1.6125`, `pnc_task_state=9`

Second full empty loop with idle-gated arrival:

- `logs/empty_nav_idlegate_loop2_home_to_grab_20260612.log`
  - `HOME_SAFE -> GRAB_PRE`
  - result `arrived`, elapsed `7.511s`, final `xy_error_m=0.0212`,
    `yaw_error_deg=-2.2504`, `pnc_task_state=9`
- `logs/empty_nav_idlegate_loop2_grab_to_place_20260612.log`
  - `GRAB_PRE -> PLACE_PRE`
  - result `arrived`, elapsed `11.019s`, final `xy_error_m=0.0175`,
    `yaw_error_deg=2.7260`, `pnc_task_state=9`
  - Yaw is inside the configured `3deg` tolerance but still close enough to
    monitor before rack action integration.
- `logs/empty_nav_idlegate_loop2_place_to_recovery_20260612.log`
  - `PLACE_PRE -> RECOVERY_SAFE`
  - result `arrived`, elapsed `17.521s`, final `xy_error_m=0.0019`,
    `yaw_error_deg=-0.1801`, `pnc_task_state=9`
- `logs/empty_nav_idlegate_loop2_recovery_to_home_20260612.log`
  - `RECOVERY_SAFE -> HOME_SAFE`
  - result `arrived`, elapsed `8.513s`, final `xy_error_m=0.0144`,
    `yaw_error_deg=-2.3785`, `pnc_task_state=9`

Yaw-refinement hardening after the idle-gated loop:

- Re-sending map navigation to the current station does not refine yaw once the
  pose is already inside PNC tolerance:
  `logs/empty_nav_refine_probe_grab_to_grab_20260612.log` stayed near
  `yaw_error_deg=-2.7629`.
- `industrial_map_nav_guarded.py` now supports an optional `--refine-yaw`
  phase after successful map navigation.
  - Default behavior is unchanged; yaw refinement is off unless requested.
  - `--refine-yaw` requires `--confirm-live`.
  - The refine phase reads map yaw from `Slam.get_curr_pose()`, compares it to
    the calibrated station yaw, and uses low-speed `pnc.move_chassis(Twist)`
    for in-place correction.
  - Bounds are enforced for tolerance, maximum starting yaw error, angular
    speed, timeout, sample rate, and stable-sample count.
  - The refine phase stops chassis motion and cancels any active remote-control
    PNC task before returning.
- Robot-side backups before yaw-refine edits:
  - `industrial_map_nav_guarded.py.bak_yaw_refine_20260612`
  - `industrial_map_nav_guarded.py.bak_yaw_refine_bad_sign_20260612`
  - `industrial_map_nav_guarded.py.bak_yaw_refine_no_cleanup_20260612`
- One live probe exposed the yaw-control sign was wrong for
  `Slam.get_curr_pose()` yaw:
  `logs/empty_nav_yaw_refine_grab_20260612.log` moved from about `-2.6deg`
  toward `-7deg` and was aborted by the non-convergence guard. The stale PNC
  task was cancelled manually with `industrial_cancel_pnc_task.py --confirm-live`.
- The sign and cleanup were then fixed and validated with low-speed yaw-refine
  commands:
  - `logs/empty_nav_yaw_refine_grab_fixed_20260612.log`
    - `GRAB_PRE`, final refine error `-0.8216deg`
  - `logs/empty_nav_yaw_refine_grab_to_place_20260612.log`
    - `GRAB_PRE -> PLACE_PRE`, map-nav final yaw `-2.6660deg`,
      refine final yaw `-0.8313deg`
  - `logs/empty_nav_yaw_refine_place_to_recovery_20260612.log`
    - `PLACE_PRE -> RECOVERY_SAFE`, map-nav final yaw `0.4577deg`,
      refine final yaw `0.4064deg`
  - `logs/empty_nav_yaw_refine_recovery_to_home_20260612.log`
    - `RECOVERY_SAFE -> HOME_SAFE`, map-nav final yaw `-2.1115deg`,
      refine final yaw `-0.6755deg`
- A complete empty navigation loop with `--refine-yaw` enabled was then run
  from `HOME_SAFE` back to `HOME_SAFE`:
  - `logs/empty_nav_yaw_refine_fullloop_home_to_grab_20260612.log`
    - `HOME_SAFE -> GRAB_PRE`, map-nav final `xy_error_m=0.0191`,
      `yaw_error_deg=-2.4061`; refine final yaw `-0.8621deg`
  - `logs/empty_nav_yaw_refine_fullloop_grab_to_place_20260612.log`
    - `GRAB_PRE -> PLACE_PRE`, map-nav final `xy_error_m=0.0144`,
      `yaw_error_deg=2.4867`; refine final yaw `0.8454deg`
  - `logs/empty_nav_yaw_refine_fullloop_place_to_recovery_20260612.log`
    - `PLACE_PRE -> RECOVERY_SAFE`, map-nav final `xy_error_m=0.0040`,
      `yaw_error_deg=-0.6138`; already within refine tolerance, final yaw
      `-0.6352deg`
  - `logs/empty_nav_yaw_refine_fullloop_recovery_to_home_20260612.log`
    - `RECOVERY_SAFE -> HOME_SAFE`, map-nav final `xy_error_m=0.0102`,
      `yaw_error_deg=-2.5451`; refine final yaw `-0.5607deg`
- After the complete yaw-refine loop, read-only readiness returned `ok: true`:
  `map_id=19`, `charge_plug_insert_state=0`, charge input `0V/0A`,
  `motion_control_error=0`, `pnc_task_state=7`, odom speed samples all `0.0`.

## Mission Staging Validation

On 2026-06-12, `industrial_cell_mission_controller.py` was connected to
`industrial_map_nav_guarded.py` in a restricted staging mode.

Controller changes:

- The mission phase sequence is now:
  `NAV_TO_GRAB -> LOCAL_PICK -> NAV_TO_PLACE -> LOCAL_PLACE ->
  NAV_TO_RECOVERY -> NAV_TO_HOME -> ROD_DONE`.
- `NAV_TO_GRAB`, `NAV_TO_PLACE`, `NAV_TO_RECOVERY`, and `NAV_TO_HOME` call the
  guarded map-navigation wrapper.
- At this earlier validation point, `LOCAL_PICK` and `LOCAL_PLACE` were not
  physical actions yet. They could advance only with
  `--staging --local-action-mode noop` or
  `--staging --local-action-mode readonly`.
- `readonly` local mode runs `RackIndustrialDockingController.preflight()` and
  `read_snapshots()` only; it logs rack lidar, front ultrasonic, and rear
  ultrasonic evidence without sending arm, gripper, rack-docking, or chassis
  commands.
- `--confirm-live` mission execution is accepted only with `--staging`.
- Robot-side backup before this patch:
  `industrial_cell_mission_controller.py.bak_staging_nav_only_20260612`.
- Robot-side backup before the local readonly-gate patch:
  `industrial_cell_mission_controller.py.bak_local_readonly_gate_20260612`.

Validation:

- Local py-compile and local staging dry-run passed.
- Robot-side py-compile passed.
- Robot-side staging dry-run passed and ended at `MISSION_DONE`:
  `logs/industrial_cell_mission_staging_dryrun_20260612_checkpoint.json`.
- Live no-arm/no-rack mission staging passed:
  `logs/industrial_cell_mission_staging_live_navonly_20260612.log`
  - `HOME_SAFE -> GRAB_PRE`: map-nav final `xy_error_m=0.0173`,
    `yaw_error_deg=-2.0611`; refine final yaw `-0.8549deg`
  - `LOCAL_PICK`: noop only; no arm, gripper, or rack command sent
  - `GRAB_PRE -> PLACE_PRE`: map-nav final `xy_error_m=0.0108`,
    `yaw_error_deg=1.3361`; refine final yaw `0.9005deg`
  - `LOCAL_PLACE`: noop only; no arm, gripper, or rack command sent
  - `PLACE_PRE -> RECOVERY_SAFE`: map-nav final `xy_error_m=0.0041`,
    `yaw_error_deg=-0.1838`; already inside refine tolerance, final yaw
    `-0.1914deg`
  - `RECOVERY_SAFE -> HOME_SAFE`: map-nav final `xy_error_m=0.0097`,
    `yaw_error_deg=-1.7647`; refine final yaw `-0.2589deg`
- Live staging checkpoint:
  `logs/industrial_cell_mission_staging_live_navonly_20260612_checkpoint.json`
  - final phase `MISSION_DONE`
  - `current_station=HOME_SAFE`
  - `holding_rod=false`
- Individual child navigation logs:
  - `logs/industrial_cell_mission_rod1_nav_to_grab_to_grab_pre_20260612_143350.log`
  - `logs/industrial_cell_mission_rod1_nav_to_place_to_place_pre_20260612_143402.log`
  - `logs/industrial_cell_mission_rod1_nav_to_recovery_to_recovery_safe_20260612_143417.log`
  - `logs/industrial_cell_mission_rod1_nav_to_home_to_home_safe_20260612_143438.log`
- After live mission staging, read-only readiness returned `ok: true`:
  `map_id=19`, `charge_plug_insert_state=0`, charge input `0V/0A`,
  `motion_control_error=0`, `pnc_task_state=7`, odom speed samples all `0.0`.
- A single isolated local readonly gate was first tested with a hand-written
  `LOCAL_PICK` checkpoint:
  `logs/industrial_cell_mission_readonly_gate_single_20260612_checkpoint.json`.
  - Without `--allow-estop-pedal-fault`, it correctly blocked on the known
    `emergency_stop_pedal_fault_state=1`.
  - With `--allow-estop-pedal-fault`, it passed and recorded only read-only
    lidar/front-ultrasonic/rear-ultrasonic samples.
- Live mission staging with readonly local gates then passed:
  `logs/industrial_cell_mission_staging_live_readonly_20260612.log`
  - `HOME_SAFE -> GRAB_PRE`: map-nav final `xy_error_m=0.0161`,
    `yaw_error_deg=-2.7258`; refine final yaw `-0.8349deg`
  - `LOCAL_PICK` readonly gate:
    - preflight `ok` with known
      `emergency_stop_pedal_fault_state=1 allowed`
    - front ultrasonic stable at about `762-765mm`
    - lidar distance about `0.870-0.888m`
    - no arm, gripper, rack-docking, or chassis command sent
  - `GRAB_PRE -> PLACE_PRE`: map-nav final `xy_error_m=0.0132`,
    `yaw_error_deg=1.4243`; refine final yaw `0.8864deg`
  - `LOCAL_PLACE` readonly gate:
    - preflight `ok` with known
      `emergency_stop_pedal_fault_state=1 allowed`
    - front ultrasonic stable at about `918-930mm`
    - lidar distance about `0.932-0.946m`
    - no arm, gripper, rack-docking, or chassis command sent
  - `PLACE_PRE -> RECOVERY_SAFE`: map-nav final `xy_error_m=0.0090`,
    `yaw_error_deg=0.1600`; final yaw `0.1471deg`
  - `RECOVERY_SAFE -> HOME_SAFE`: map-nav final `xy_error_m=0.0149`,
    `yaw_error_deg=-0.6443`; final yaw `-0.6713deg`
- Live readonly checkpoint:
  `logs/industrial_cell_mission_staging_live_readonly_20260612_checkpoint.json`
  - final phase `MISSION_DONE`
  - `current_station=HOME_SAFE`
  - `holding_rod=false`
- Individual child navigation logs for the readonly run:
  - `logs/industrial_cell_mission_rod1_nav_to_grab_to_grab_pre_20260612_144207.log`
  - `logs/industrial_cell_mission_rod1_nav_to_place_to_place_pre_20260612_144223.log`
  - `logs/industrial_cell_mission_rod1_nav_to_recovery_to_recovery_safe_20260612_144241.log`
  - `logs/industrial_cell_mission_rod1_nav_to_home_to_home_safe_20260612_144301.log`
- After live readonly staging, read-only readiness returned `ok: true`:
  `map_id=19`, `charge_plug_insert_state=0`, charge input `0V/0A`,
  `motion_control_error=0`, `pnc_task_state=7`, odom speed samples all `0.0`.

## Arm/Gripper Gate Validation

On 2026-06-12, `industrial_cell_mission_controller.py` gained a no-motion
arm/gripper gate:

- `--arm-gate-mode manifest` scans arm/gripper/waist/offset scripts, runs
  syntax compilation checks, and marks scripts that contain unguarded top-level
  motion/GDK calls.
- `--arm-gate-mode dryrun` is allowlisted only for
  `move_arm_vertical_stack_grab_above.py --dry-run`; it does not run gripper,
  JSON arm-pose, waist, offset, rack-docking, or chassis commands.
- `--arm-gate-only` runs the gate without checkpoint changes.
- Robot-side backup before this patch:
  `industrial_cell_mission_controller.py.bak_arm_gate_20260612`.

Validation:

- Local py-compile passed for
  `rack_hybrid_docking_package/industrial_cell_mission_controller.py`.
- Robot-side py-compile passed for:
  - `rack_hybrid_docking_package/industrial_cell_mission_controller.py`
  - `move_arm_vertical_stack_grab_above.py`
- Robot-side manifest gate passed:
  - scanned `34` scripts
  - `missing_required=[]`
  - `compile_failures=[]`
  - `allowed_dry_runs=["move_arm_vertical_stack_grab_above.py"]`
  - all direct gripper, JSON arm-pose, waist, and offset scripts remain blocked
    from automatic execution by the gate.
- Robot-side dry-run gate passed with:
  `--arm-dry-run-base-json /data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_grab_第一根.json`
  and `--arm-dry-run-pitch-m -0.060`.
  Log:
  `logs/industrial_cell_mission_rod1_local_pick_arm_gate_dryrun_20260612_145127.log`.
  The child script printed `dry-run: skip GDK init and arm movement`.
- No-motion `LOCAL_PICK` checkpoint integration passed in both manifest and
  dry-run modes using `/tmp` checkpoints:
  - `/tmp/industrial_cell_arm_gate_manifest_checkpoint.json`
  - `/tmp/industrial_cell_arm_gate_dryrun_checkpoint.json`
  Both advanced only from `LOCAL_PICK` to `NAV_TO_PLACE`; no arm, gripper,
  rack-docking, or chassis command was sent.
- After arm-gate validation, read-only readiness returned `ok: true`:
  `map_id=19`, `charge_plug_insert_state=0`, charge input `0V/0A`,
  `motion_control_error=0`, `pnc_task_state=7`, odom speed samples all `0.0`.

## Live Single Local Action Validation

After the environment was confirmed safe on 2026-06-12, the next live work was
kept to isolated local actions only. No map navigation, rack docking, or
multi-step pick/place sequence was run in this pass.

Pre-action readiness:

- `ok=true`
- `map_id=19`
- `charge_plug_insert_state=0`
- `motion_control_error=0`
- `pnc_task_state=7`
- odom speed samples all `0.0`
- pose about `x=0.597049`, `y=-0.970596`, `z=0.006783`

Validated live local actions:

- Open both grippers:
  `logs/live_single_gripper_open_20260612_145423.log`
  - `右夹爪张开成功`
  - `左夹爪张开成功`
  - `GDK释放成功`
- Close both grippers:
  `logs/live_single_gripper_close_20260612_145448.log`
  - `右夹爪闭合成功`
  - `左夹爪闭合成功`
  - `GDK释放成功`
- Re-open both grippers to leave the pickup start state open:
  `logs/live_single_gripper_reopen_20260612_145516.log`
  - `右夹爪张开成功`
  - `左夹爪张开成功`
  - `GDK释放成功`
- Move both arms to the first-rod grab-above posture:
  `logs/live_single_arm_grab_above_rod1_20260612_145539.log`
  - command used `move_arm_vertical_stack_grab_above.py --rod-index 1`
  - base JSON:
    `/data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_grab_第一根.json`
  - `--joint-speed-radps 0.12`
  - `base_joint_move_result=0`
  - `vertical_stack_z_offset skipped for base layer`
- Restore both arms to the default posture:
  `logs/live_single_arm_default_20260612_145624.log`
  - `JSON 配置读取成功`
  - `GDK初始化成功`
  - `手臂控制成功`
  - `GDK释放成功`
- Return both arms to the first-rod grab posture on request:
  `logs/live_return_arm_grab_rod1_20260612_145901.log`
  - command used `move_arm_vertical_stack_grab_above.py --rod-index 1`
  - base JSON:
    `/data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_grab_第一根.json`
  - `--joint-speed-radps 0.12`
  - `base_joint_move_result=0`
  - `vertical_stack_z_offset skipped for base layer`
- Close both grippers while the arms are at the first-rod grab posture:
  `logs/live_local_pick_close_gripper_rod1_20260612_150121.log`
  - `右夹爪闭合成功`
  - `左夹爪闭合成功`
  - `GDK释放成功`
- Restore both arms to the default posture while keeping the grippers closed:
  `logs/live_local_pick_arm_default_after_close_rod1_20260612_150406.log`
  - `JSON 配置读取成功`
  - `GDK初始化成功`
  - `手臂控制成功`
  - `GDK释放成功`

## Live One-Rod Basic Station Run Stopped Before Fine Positioning

On 2026-06-12, a basic one-rod station run was started after the operator asked
to run a pick/place flow. This run used map stations and validated arm/gripper
primitives, but it did not execute the rack ultrasonic fine-positioning step.
The operator then pointed out that the process should go to the fine-position
outside point before grabbing, so the run was stopped after the current arm was
returned to default posture.

Executed steps:

- Open grippers:
  `logs/live_one_rod_basic_step01_open_gripper_20260612_150844.log`
- Navigate `HOME_SAFE -> GRAB_PRE`:
  `logs/live_one_rod_basic_step02_nav_to_grab_pre_20260612_1508.log`
  - final yaw refine error about `-0.804deg`
- Move arms to first-rod grab posture:
  `logs/live_one_rod_basic_step03_arm_grab_rod1_20260612_150932.log`
  - `base_joint_move_result=0`
- Close grippers:
  `logs/live_one_rod_basic_step04_close_gripper_20260612_150945.log`
- Restore arms to default for navigation:
  `logs/live_one_rod_basic_step05_arm_default_after_pick_20260612_150956.log`
  - `move_arm_joint_result=0`
- Navigate `GRAB_PRE -> PLACE_PRE`:
  `logs/live_one_rod_basic_step06_nav_to_place_pre_20260612_1510.log`
  - final yaw refine error about `0.789deg`
- Move arms to place-above posture using explicit JSON helper:
  `logs/live_one_rod_basic_step07_arm_place_above_20260612_151056.log`
  - helper: `move_arm_by_json_path.py`
  - JSON:
    `/data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_grab_2.json`
  - `move_arm_joint_result=0`
- Open grippers at place:
  `logs/live_one_rod_basic_step08_open_gripper_place_20260612_151111.log`
- Restore arms to default:
  `logs/live_one_rod_basic_step09_arm_default_after_place_20260612_151123.log`
  - `move_arm_joint_result=0`

Important correction:

- This run did not execute the rack ultrasonic fine-positioning/approach step
  before grabbing. The old full controller's grab fine-position target is
  documented as about `155mm` using front ultrasonic `0/1`.
- `offset_move_down.py` was not executed in this basic station run.
- The hardcoded legacy `move_arm_by_json_grab_above_2.py` points at a missing
  `/data/btgys/.../arm_position_to_grab_2.json`; use
  `move_arm_by_json_path.py --json /data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_grab_2.json`
  instead until the legacy script is repaired.

Current stopped state after the correction:

- Robot is near `PLACE_PRE`.
- Latest readiness after stopping was `ok=true`: `map_id=19`,
  `motion_control_error=0`, `pnc_task_state=7`, odom speed samples all `0.0`.
- Latest pose was about `x=1.653661`, `y=-0.209653`, `z=-0.011905`.
- Grippers are open.
- Arms are default.
- Do not continue this partial run as a successful fine-positioned pick/place
  cycle. To correct it, restart from a safe state and explicitly include the
  grab fine-positioning step before closing grippers.

## Corrected Full Local Flow Integration

After the operator pointed out that the basic station run skipped fine
positioning, the mission controller was updated so `LOCAL_PICK` and
`LOCAL_PLACE` can run the full local sequence.

Robot-side backup before this patch:

- `rack_hybrid_docking_package/industrial_cell_mission_controller.py.bak_full_local_flow_20260612`

New/updated helpers:

- `move_arm_by_json_path.py`
  - explicit arm-pose JSON path helper;
  - used instead of the legacy `move_arm_by_json_grab_above_2.py`, which still
    points at a missing `/data/btgys/.../arm_position_to_grab_2.json`.
- `move_ee_relative_offset.py`
  - explicit `--left X,Y,Z` and `--right X,Y,Z` relative end-effector offset;
  - validates `--max-abs-m`;
  - supports `--dry-run`.

`LOCAL_PICK` full sequence:

- open grippers;
- move arms to first-rod grab-above pose with
  `move_arm_vertical_stack_grab_above.py`;
- front-ultrasonic fine positioning:
  `grab_fine_position`, default `final_stop_mm=155`,
  `final_brake_margin_mm=70`, `final_speed_mps=0.15`;
- close grippers;
- pull out by relative offset, default `x=-0.15m`;
- restore arms to default JSON;
- retreat locally, default `1.0m` at `0.30m/s` unless
  `--skip-local-retreat` is used.

`LOCAL_PLACE` full sequence:

- move arms to place-above JSON:
  `/data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_grab_2.json`;
- front-ultrasonic fine positioning:
  `place_fine_position`, default `final_stop_mm=327`,
  `final_brake_margin_mm=60`, `final_speed_mps=0.15`;
- lower by explicit relative offset, default `z=-0.06m`;
- open grippers;
- pull out by relative offset, default `x=-0.15m`;
- retreat locally, default `1.0m` at `0.30m/s` unless
  `--skip-local-retreat` is used.
- restore arms to default JSON after the local retreat.

Safety gates:

- `--local-action-mode full-dry-run` prints the corrected full sequence and
  sends no arm, gripper, rack, or chassis command.
- `--local-action-mode full` requires both `--confirm-live` and
  `--confirm-local-physical`.
- `full` also checks that the grab base JSON, place-above JSON, and arm-default
  JSON exist before execution.
- After the rack-fall report, physical `full` mode is blocked by:
  `logs/RACK_FALL_SAFETY_LOCK`.
- The explicit down offset defaults to `-0.06m`; this avoids silently reusing
  the larger legacy `offset_move_down.py` hardcoded `-0.18m` motion.

Validation after the patch:

- Local py-compile passed for:
  - `rack_hybrid_docking_package/industrial_cell_mission_controller.py`
  - `move_arm_by_json_path.py`
  - `move_ee_relative_offset.py`
- Local one-rod `full-dry-run` completed to `MISSION_DONE` with both
  `grab_fine_position` and `place_fine_position` in the printed plan.
- Robot-side py-compile passed for the same three files.
- Robot-side one-rod `full-dry-run` completed to `MISSION_DONE` using:
  `/tmp/industrial_cell_full_local_dryrun_rod1_20260612.json`.
- Robot-side `full` mode without `--confirm-live` correctly exits with:
  `--local-action-mode full requires --confirm-live`.
- Robot-side `full` mode with `--confirm-live` but without
  `--confirm-local-physical` correctly exits with:
  `--local-action-mode full requires --confirm-local-physical`.
- Required arm JSON paths exist on the robot:
  - `/data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_grab_第一根.json`
  - `/data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_grab_2.json`
  - `/data/wxf/wxf/positions/arm_default.json`

First corrected physical `full` run has now been executed for rod `1`; see the
next section for details.

Current final state:

- Robot is back near `HOME_SAFE`.
- Latest final readiness after the corrected rod-1 full-local run was
  `ok=true`, `map_id=19`, `motion_control_error=0`, `pnc_task_state=7`, and odom
  speed samples all `0.0`.
- Latest final pose was about `x=0.603475`, `y=-0.970982`, `z=0.005614`.
- Checkpoint
  `logs/industrial_cell_mission_full_local_rod1_20260612_checkpoint.json` is at
  `MISSION_DONE`, `holding_rod=false`, `current_station=HOME_SAFE`.

## First Corrected Rod-1 Full-Local Live Run

On 2026-06-12, one corrected rod-1 full-local cycle was run on
`agi@10.20.15.60` with front-ultrasonic fine positioning at both grab and
place.

Command checkpoint:

- `logs/industrial_cell_mission_full_local_rod1_20260612_checkpoint.json`

Important mid-run fix:

- The first attempt reached `grab_fine_position` and closed the grippers, but
  failed at `pull_out_offset` because negative offset values were passed to
  argparse as separate option values:
  `--left -0.150000,0.000000,0.000000`.
- `industrial_cell_mission_controller.py` was patched so relative-offset
  commands use equals form:
  `--left=-0.150000,0.000000,0.000000`.
- The patch was compiled and synced to the robot, then the remaining
  `LOCAL_PICK` steps were completed manually:
  - `move_ee_relative_offset.py --left=-0.150000,0.000000,0.000000 --right=-0.150000,0.000000,0.000000`
  - `move_arm_by_json_path.py --json /data/wxf/wxf/positions/arm_default.json`
  - `use_industrial_docking_methods.py --mode retreat --retreat-distance-m 1.0 --retreat-speed-mps 0.30 --allow-estop-pedal-fault`
- The checkpoint was then advanced from `LOCAL_PICK` to `NAV_TO_PLACE` with
  `--advance-dry-run` because the physical pick remainder had already been
  completed.

Successful run evidence:

- `NAV_TO_GRAB -> GRAB_PRE`:
  `logs/industrial_cell_mission_rod1_nav_to_grab_to_grab_pre_20260612_153227.log`
  - arrived; yaw refine final error about `-0.826deg`
- `LOCAL_PICK`:
  - open gripper log:
    `logs/industrial_cell_mission_rod1_local_pick_open_gripper_20260612_153240.log`
  - arm grab pose log:
    `logs/industrial_cell_mission_rod1_local_pick_arm_grab_above_20260612_153243.log`
    with `base_joint_move_result=0`
  - `grab_fine_position` completed with `status=stopped`,
    `final_stop_mm=155`, `final_brake_margin_mm=70`,
    `front_filtered_mm=226`
  - close gripper log:
    `logs/industrial_cell_mission_rod1_local_pick_close_gripper_20260612_153255.log`
  - the corrected manual pull-out offset returned `relative_offset_result=True`
  - manual arm-default returned `move_arm_joint_result=0`
  - manual local retreat returned `status=completed`
- `NAV_TO_PLACE -> PLACE_PRE`:
  `logs/industrial_cell_mission_rod1_nav_to_place_to_place_pre_20260612_153614.log`
  - arrived; yaw refine final error about `-0.421deg`
- `LOCAL_PLACE`:
  - arm place-above log:
    `logs/industrial_cell_mission_rod1_local_place_arm_place_above_20260612_153628.log`
    with `move_arm_joint_result=0`
  - `place_fine_position` completed with `status=stopped`,
    `final_stop_mm=327`, `final_brake_margin_mm=60`,
    `front_filtered_mm=388`
  - down offset log:
    `logs/industrial_cell_mission_rod1_local_place_place_down_offset_20260612_153642.log`
    with `relative_offset_result=True`
  - open gripper log:
    `logs/industrial_cell_mission_rod1_local_place_open_gripper_place_20260612_153646.log`
  - pull-out offset log:
    `logs/industrial_cell_mission_rod1_local_place_place_pull_out_offset_20260612_153649.log`
    with `relative_offset_result=True`
  - arm default log:
    `logs/industrial_cell_mission_rod1_local_place_arm_default_after_place_20260612_153655.log`
    with `move_arm_joint_result=0`
  - `retreat_after_place` returned `status=completed`
- `NAV_TO_RECOVERY -> RECOVERY_SAFE`:
  `logs/industrial_cell_mission_rod1_nav_to_recovery_to_recovery_safe_20260612_153710.log`
  - arrived; final `xy_error_m=0.0015`, yaw refine final error about
    `-0.046deg`
- `NAV_TO_HOME -> HOME_SAFE`:
  `logs/industrial_cell_mission_rod1_nav_to_home_to_home_safe_20260612_153730.log`
  - arrived; final `xy_error_m=0.0070`, yaw refine final error about
    `-0.603deg`
- Final checkpoint is `MISSION_DONE`, `holding_rod=false`,
  `current_station=HOME_SAFE`.
- Final readiness returned `ok=true`, `motion_control_error=0`,
  `pnc_task_state=7`, and odom speed samples all `0.0`.

## Rack Fall Safety Stop

After the corrected rod-1 full-local run, the operator reported that the rack
was knocked over. All physical motion work is stopped until the fixture is
reset and the approach parameters are re-approved.

Immediate stop-state check:

- Final read-only readiness after the report returned `ok=true`.
- Robot pose was near `HOME_SAFE`: about `x=0.605634`, `y=-0.968644`,
  `z=0.005916`.
- `motion_control_error=0`, `pnc_task_state=7`, and odom speed samples were all
  `0.0`.
- `industrial_cancel_pnc_task.py` is not present in this development copy, but
  PNC was already idle.

Safety lock:

- `logs/RACK_FALL_SAFETY_LOCK` was created on the robot.
- `industrial_cell_mission_controller.py` now refuses
  `--local-action-mode full` while this file exists.
- Lock probe passed: `full` mode exits before motion with
  `--local-action-mode full blocked by safety lock`.
- `full-dry-run` still works for plan review only.

Current program paths:

- Current station-based total controller:
  `rack_hybrid_docking_package/industrial_cell_mission_controller.py`
- Older seven-rods total controller still exists but is not the current
  station-based program:
  `rack_hybrid_docking_package/industrial_7_rods_total_controller.py`

Grab fine-position evidence:

- Current configured grab target is `grab_final_stop_mm=155` with
  `grab_final_brake_margin_mm=70`.
- The approach therefore uses a trigger around `225mm`.
- The rod-1 run stopped with front ultrasonic readings around
  `front_filtered_mm=226`, raw `id0=242mm`, `id1=220mm`.
- The operator observed the physical rack clearance looked closer to about
  `160mm`, which is plausible if the ultrasonic sensor origin and the nearest
  robot/fixture contact point differ by about `60-70mm`.
- Do not reuse the `155mm/70mm` grab fine-position settings until the rack is
  reset and the sensor-to-contact offset is re-measured.

## Next Validation Work

All four map stations are calibrated. The idle-gated wrapper has completed a
full empty loop, optional yaw refinement has completed a second full empty loop,
the mission controller has completed a no-arm/no-rack live staging run, and the
local rack readonly gates have passed at `GRAB_PRE` and `PLACE_PRE`. The
arm/gripper manifest and one allowlisted pick dry-run gate have also passed.
The live single-action gripper and arm primitives passed, and rod `1` completed
one physical full-local run, but that run is now associated with a rack-fall
safety incident.

The next validation step is not another physical run. Required first:

- physically reset and inspect the rack/fixture;
- re-measure the front ultrasonic sensor readings versus actual rack clearance;
- revise grab fine-position settings away from the current `155mm/70mm`
  configuration;
- review the place sequence with the updated order:
  place pull-out -> retreat -> arm default;
- remove `logs/RACK_FALL_SAFETY_LOCK` only after the revised live plan is
  explicitly approved.

Do not run physical map navigation unless the target station is calibrated and
readiness check returns `ok: true`.

## First Non-Motion Commands

```bash
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
python3 rack_hybrid_docking_package/industrial_map_nav_guarded.py --list-stations
python3 rack_hybrid_docking_package/industrial_map_nav_guarded.py --readiness-check
python3 rack_hybrid_docking_package/industrial_cell_mission_controller.py --status \
  --checkpoint-file logs/industrial_cell_mission_staging_live_readonly_20260612_checkpoint.json
python3 rack_hybrid_docking_package/industrial_cell_mission_controller.py \
  --arm-gate-only \
  --arm-gate-mode manifest \
  --start-index 1
python3 rack_hybrid_docking_package/industrial_cell_mission_controller.py \
  --arm-gate-only \
  --arm-gate-mode dryrun \
  --start-index 1 \
  --arm-dry-run-base-json /data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_grab_第一根.json
python3 rack_hybrid_docking_package/industrial_cell_mission_controller.py \
  --checkpoint-file /tmp/industrial_cell_full_local_dryrun_next.json \
  --init \
  --staging \
  --local-action-mode full-dry-run \
  --start-index 1 \
  --end-index 1 \
  --run-current-rod
```

## First Live Readonly Staging Command

Use only after `--readiness-check` returns `ok: true` and the target station is
calibrated. This command runs live navigation and readonly local rack gates;
it still does not run arm, gripper, or rack-docking actions:

```bash
python3 rack_hybrid_docking_package/industrial_cell_mission_controller.py \
  --checkpoint-file logs/industrial_cell_mission_staging_live_readonly_next_checkpoint.json \
  --nav-log-dir logs \
  --start-index 1 \
  --end-index 1 \
  --init \
  --staging \
  --local-action-mode readonly \
  --allow-estop-pedal-fault \
  --rack-read-samples 5 \
  --rack-read-interval-s 0.12 \
  --arm-gate-mode manifest \
  --confirm-live \
  --refine-yaw \
  --refine-yaw-tolerance-deg 1.0 \
  --refine-yaw-max-error-deg 10.0 \
  --refine-yaw-angular-speed-radps 0.05 \
  --refine-yaw-fine-angular-speed-radps 0.02 \
  --refine-yaw-timeout-s 12 \
  --run-current-rod
```

This readonly command remains useful for a fresh preflight pass, but it is not
the corrected physical full-local command. Use the next section for the next
one-rod full-local run after the robot state is verified safe.

## Manual Grab Recalibration Notes

2026-06-12 grab points are being manually retuned with the robot at the rack.
No motion was sent during these captures.

Rod 1 capture:

- record: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod01_grab_calibration_20260612_160543.json`
- reusable pose JSON: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod01_grab_pose_20260612_160543.json`
- chassis pose at capture: position `{x: 0.145266, y: 0.048116, z: -0.006703}`, orientation `{x: 0.006018, y: 0.005858, z: 0.858525, w: 0.512703}`
- front ultrasonic rack distance at capture: 10 valid samples, min `313mm`, max `316mm`, median `313mm`, last `313mm`
- readiness during capture was blocked by charging: `charge_plug_insert_state=1`, `charge_input_current=15.000>0.500`

Rod 2 valid capture:

- record: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod02_grab_calibration_20260612_161238.json`
- reusable pose JSON: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod02_grab_pose_20260612_161238.json`
- chassis pose at capture: position `{x: 0.146499, y: 0.050376, z: -0.004631}`, orientation `{x: 0.006102, y: 0.005898, z: 0.858516, w: 0.512717}`
- front ultrasonic rack distance at capture: 10 valid samples, min `313mm`, max `319mm`, median `319mm`, last `313mm`
- readiness during capture was blocked by charging: `charge_plug_insert_state=1`, `charge_input_current=15.000>0.500`

Rod 3 valid capture:

- record: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod03_grab_calibration_20260612_161855.json`
- reusable pose JSON: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod03_grab_pose_20260612_161855.json`
- chassis pose at capture: position `{x: 0.144789, y: 0.050034, z: -0.005771}`, orientation `{x: 0.006101, y: 0.005813, z: 0.858366, w: 0.512969}`
- front ultrasonic rack distance at capture: 10 valid samples, min `316mm`, max `320mm`, median `320mm`, last `320mm`
- readiness during capture was blocked by charging/PNC state: `charge_plug_insert_state=1`, `charge_input_current=15.000>0.500`, `pnc_task_state_not_idle=2,id=2005192013`

Rod 4 valid capture:

- record: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod04_grab_calibration_20260612_162209.json`
- reusable pose JSON: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod04_grab_pose_20260612_162209.json`
- chassis pose at capture: position `{x: 0.145171, y: 0.047248, z: -0.006432}`, orientation `{x: 0.006101, y: 0.005786, z: 0.858350, w: 0.512996}`
- front ultrasonic rack distance at capture: 10 valid samples, min `316mm`, max `319mm`, median `319mm`, last `319mm`
- readiness during capture was blocked by charging/PNC state: `charge_plug_insert_state=1`, `charge_input_current=15.000>0.500`, `pnc_task_state_not_idle=2,id=2005192013`

Rod 5 valid capture:

- record: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod05_grab_calibration_20260612_162921.json`
- reusable pose JSON: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod05_grab_pose_20260612_162921.json`
- chassis pose at capture: position `{x: 0.146406, y: 0.046989, z: -0.004992}`, orientation `{x: 0.006017, y: 0.005796, z: 0.858350, w: 0.512997}`
- front ultrasonic rack distance at capture: 10 valid samples, min `316mm`, max `320mm`, median `319mm`, last `320mm`
- readiness during capture was blocked by charging/PNC state: `charge_plug_insert_state=1`, `charge_input_current=15.000>0.500`, `pnc_task_state_not_idle=2,id=2005192013`

Rod 6 valid capture:

- record: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod06_grab_calibration_20260612_163400.json`
- reusable pose JSON: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod06_grab_pose_20260612_163400.json`
- chassis pose at capture: position `{x: 0.143547, y: 0.047770, z: -0.005875}`, orientation `{x: 0.005892, y: 0.005971, z: 0.858418, w: 0.512882}`
- front ultrasonic rack distance at capture: 10 valid samples, min `316mm`, max `319mm`, median `316mm`, last `316mm`
- readiness during capture was blocked by charging/PNC state: `charge_plug_insert_state=1`, `charge_input_current=15.000>0.500`, `pnc_task_state_not_idle=2,id=2005192013`
- rod 6 waist/body pose: `idx01_body_joint1=-0.6508540080337357`, `idx02_body_joint2=2.0535670396290415`, `idx03_body_joint3=-1.4571637267965485`, `idx04_body_joint4=-0.018404858546708577`, `idx05_body_joint5=0.03882274849696341`
- rod 6 left arm pose: `idx21_arm_l_joint1=1.6584004615760781`, `idx22_arm_l_joint2=-1.4821139113778554`, `idx23_arm_l_joint3=-1.3007115652125962`, `idx24_arm_l_joint4=-1.405372198156056`, `idx25_arm_l_joint5=0.05165093076184534`, `idx26_arm_l_joint6=-0.28562206948759705`, `idx27_arm_l_joint7=0.22473357832645385`
- rod 6 right arm pose: `idx61_arm_r_joint1=-1.7720330844949335`, `idx62_arm_r_joint2=-1.4610044482211264`, `idx63_arm_r_joint3=1.548820254374699`, `idx64_arm_r_joint4=-1.4009491802702363`, `idx65_arm_r_joint5=0.09786977190083972`, `idx66_arm_r_joint6=-0.4063280216300918`, `idx67_arm_r_joint7=-0.10650093051992657`

Rod 7 valid capture:

- record: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod07_grab_calibration_20260612_163717.json`
- reusable pose JSON: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod07_grab_pose_20260612_163717.json`
- chassis pose at capture: position `{x: 0.147043, y: 0.047588, z: -0.006360}`, orientation `{x: 0.006035, y: 0.005750, z: 0.858325, w: 0.513039}`
- front ultrasonic rack distance at capture: 10 valid samples, min `316mm`, max `320mm`, median `319mm`, last `316mm`
- readiness during capture was blocked by charging/PNC state: `charge_plug_insert_state=1`, `charge_input_current=15.000>0.500`, `pnc_task_state_not_idle=2,id=2005192013`
- rod 7 waist/body pose: `idx01_body_joint1=-0.6508540080337357`, `idx02_body_joint2=2.0535670396290415`, `idx03_body_joint3=-1.4571637267965485`, `idx04_body_joint4=-0.018404858546708577`, `idx05_body_joint5=0.03882274849696341`
- rod 7 left arm pose: `idx21_arm_l_joint1=1.576353711489133`, `idx22_arm_l_joint2=-1.5053682012527503`, `idx23_arm_l_joint3=-1.2896114165785078`, `idx24_arm_l_joint4=-1.1439473573326153`, `idx25_arm_l_joint5=-0.013868864113972946`, `idx26_arm_l_joint6=-0.4248962598562022`, `idx27_arm_l_joint7=0.2363356262995786`
- rod 7 right arm pose: `idx61_arm_r_joint1=-1.7206383365141484`, `idx62_arm_r_joint2=-1.5061432310642335`, `idx63_arm_r_joint3=1.5221861534182874`, `idx64_arm_r_joint4=-1.2273563642514067`, `idx65_arm_r_joint5=0.07764279710557889`, `idx66_arm_r_joint6=-0.4525198982630491`, `idx67_arm_r_joint7=-0.2123252317016967`

Waist/body capture note:

- Operator stated rods 1 through 5 use this waist home pose.
- Waist home reference captured with rod 5: `idx01_body_joint1=-0.6980164580596764`, `idx02_body_joint2=1.5707270786927334`, `idx03_body_joint3=-0.8725218920237588`, `idx04_body_joint4=0.0`, `idx05_body_joint5=0.0`.
- Rods 6 and 7 have their own captured bent-waist/body pose and must not reuse the rods 1-5 waist home pose.

Rod 2 premature capture, do not use:

- record: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod02_grab_calibration_20260612_160727.json`
- reusable pose JSON: `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package/calibration_records/rod02_grab_pose_20260612_160727.json`
- chassis pose at capture: position `{x: 0.143319, y: 0.049097, z: -0.008302}`, orientation `{x: 0.006006, y: 0.005950, z: 0.858469, w: 0.512795}`
- front ultrasonic rack distance at capture: 10 valid samples, min `310mm`, max `317mm`, median `316.5mm`, last `317mm`
- readiness during capture was blocked by charging: `charge_plug_insert_state=1`, `charge_input_current=15.000>0.500`
- operator had not moved to rod 2 yet; this is an invalid duplicate/nearby state and must not be used as rod 2.

User-stated post-grab extraction sequence:

1. Close/grab with both arms together at the retuned grab point.
2. Move both arms down `0.020m`.
3. Move both arms backward `0.200m`, away from the rack/material frame.
4. Then use the chassis to retreat straight back to the grab target / rack-exit point.
5. Move the waist/body back to the home pose.
6. Only after this straight extraction should the workflow continue with arm reset or navigation.

Latest operator correction: after grabbing, do not lift the material. Move the
arms down `0.020m`, then backward `0.200m`. The earlier `0.035m` lift, `0.010m`
lift, and direct `0.100m` backward extraction ideas are superseded.

2026-06-12 controller update for new grab flow:

- `industrial_cell_mission_controller.py` now uses captured per-rod grab pose JSONs from `rack_hybrid_docking_package/calibration_records/rodXX_grab_pose_*.json`.
- The old vertical-stack grab pose (`first rod + pitch`) is no longer used for `LOCAL_PICK`.
- `LOCAL_PICK` plan is now: open gripper, waist to captured rod pose, arms to captured rod pose, grab fine-position, close gripper, arms down `0.020m`, arms back `0.200m`, chassis retreat `0.45m`, waist home.
- Grab fine-position defaults changed from the unsafe old `155mm + 70mm brake` to `final_stop_mm=316`, `final_brake_margin_mm=0`, `final_speed_mps=0.08`.
- `move_waist_by_json_path.py` was added for bounded waist/body movement from JSON.
- `RACK_FALL_SAFETY_LOCK` is still present; physical `--local-action-mode full` remains intentionally blocked until the rack and robot start state are re-approved.

Validated dry-runs:

- Rod 1 full dry-run checkpoint `/tmp/industrial_cell_newlogic_rod1_dryrun.json` reached `MISSION_DONE`.
- Rod 1 `LOCAL_PICK` used `rod01_grab_pose_20260612_160543.json`, `pick_down_offset=-0.020m`, `pick_back_offset=-0.200m`, and `retreat_after_pick=0.45m`.
- Rod 7 full dry-run checkpoint `/tmp/industrial_cell_newlogic_rod7_dryrun.json` reached `MISSION_DONE`.
- Rod 7 `LOCAL_PICK` used `rod07_grab_pose_20260612_163717.json`, including its captured bent-waist/body pose, with the same `-0.020m` down and `-0.200m` back extraction.
- Both dry-runs were plan-only; no arm, gripper, rack, or chassis command was sent.

2026-06-12 live rod-7 extraction execution:

- preflight before physical motion returned `ok=true`: charge unplugged, motion control error `0`, PNC state `7`, odom speeds all `0.0`
- `move_ee_pose_close_2.py`: right and left grippers closed successfully
- `move_ee_relative_offset.py --left=0,0,0.010 --right=0,0,0.010`: `relative_offset_result=True`
- `move_ee_relative_offset.py --left=-0.100,0,0 --right=-0.100,0,0`: `relative_offset_result=True`
- chassis retreat used `use_industrial_docking_methods.py --mode retreat --retreat-distance-m 0.45 --retreat-speed-mps 0.20 --retreat-method velocity`; result `status='completed'`, estimated distance `0.45m`, rear final `2300mm`, rear filtered `2318mm`
- waist return used `move_waist_by_json_path.py --json /data/g2_industrial_cell_20260612/wxf/positions/arm_default.json --joint-speed-radps 0.06`; GDK returned `JointControlRequest timeout`, but follow-up read confirmed waist/body reached home: `idx01_body_joint1=-0.6979205994214123`, `idx02_body_joint2=1.5709187959692616`, `idx03_body_joint3=-0.8728094679385512`, `idx04_body_joint4=0.0`, `idx05_body_joint5=0.0`
- final readiness after extraction returned `ok=true`, PNC state `7`, odom speeds all `0.0`, current pose position `{x: 0.395568, y: -0.418166, z: -0.002900}`, orientation `{x: 0.005934, y: 0.006568, z: 0.858564, w: 0.512630}`
- superseded detail: this live rod-7 extraction included a `0.010m` lift and `0.100m` backward arm extraction, but later operator correction says future extraction should use `0.020m` down and `0.200m` backward instead.

Do not resume the physical full-local flow until this extraction order is
implemented and checked in dry-run. In particular, avoid rotating, lateral
chassis motion, or arm default/reset while the material is still inside the rack
clearance envelope.

## Disabled Corrected Live Full-Local Command

This command is intentionally disabled by `logs/RACK_FALL_SAFETY_LOCK` after the
rack-fall report. Keep it here only as a template for later review; do not run
it physically until the lock is removed after rack reset, inspection, and
parameter re-approval.

```bash
python3 rack_hybrid_docking_package/industrial_cell_mission_controller.py \
  --checkpoint-file logs/industrial_cell_mission_full_local_rod2_next_checkpoint.json \
  --nav-log-dir logs \
  --full-local-log-dir logs \
  --start-index 2 \
  --end-index 2 \
  --init \
  --staging \
  --local-action-mode full \
  --confirm-live \
  --confirm-local-physical \
  --allow-estop-pedal-fault \
  --refine-yaw \
  --refine-yaw-tolerance-deg 1.0 \
  --refine-yaw-max-error-deg 10.0 \
  --refine-yaw-angular-speed-radps 0.05 \
  --refine-yaw-fine-angular-speed-radps 0.02 \
  --refine-yaw-timeout-s 12 \
  --run-current-rod
```

## 2026-06-12 Live Rod-1 New-Logic Validation

This section supersedes the earlier "safety lock still present" note for the
post-rack-fall state.

- The rack was reset/inspected by the operator and explicitly re-approved as
  safe before physical motion.
- The previous lock was preserved, not deleted:
  `logs/RACK_FALL_SAFETY_LOCK.cleared_20260612_1727_newlogic`.
- `industrial_cell_mission_controller.py` now passes faster bounded waist
  parameters to `move_waist_by_json_path.py`: `--joint-speed-radps 0.120000`,
  `--max-step-rad 0.150000`, `--settle-tol-rad 0.050000`,
  `--settle-timeout-s 2.000000`, and `--poll-s 0.080000`.
- Rod 1 full physical run completed successfully with checkpoint:
  `logs/industrial_cell_newlogic_live_rod1_checkpoint.json`.
- Final checkpoint state was `MISSION_DONE`, `current_station=HOME_SAFE`,
  `holding_rod=false`, `last_success_step=ROD_DONE`.
- Post-run readiness returned `ok=true`, no problems/warnings,
  `motion_control_error=0`, `pnc_task_state=7`, odom samples all `0.0`.
- Final pose after the run was approximately position
  `{x: 0.607383, y: -0.967563, z: 0.006744}`, orientation
  `{x: -0.004780, y: 0.001152, z: 0.235786, w: 0.971793}`.

Observed rod-1 run details:

- `NAV_TO_GRAB` arrived at `GRAB_PRE`; yaw refine passed with final error
  about `1.44deg` using tolerance `1.5deg`.
- `LOCAL_PICK` used `rod01_grab_pose_20260612_160543.json`.
- Grab fine-position stopped at the front ultrasonic target:
  final target `316mm`, front filtered `320mm`, raw distances included
  `320mm` and `316mm`.
- Both grippers closed successfully.
- Post-grab extraction executed the corrected order: arms down `0.020m`, arms
  back `0.200m`, chassis retreat `0.45m` at `0.20m/s`, then waist home.
- `NAV_TO_PLACE` arrived at `PLACE_PRE`; yaw refine passed with final error
  about `1.37deg` using tolerance `1.5deg`.
- Place fine-position stopped at final trigger `387mm`; front filtered `391mm`,
  raw distances included `389mm` and `379mm`.
- Place sequence executed in the requested order: arms down `0.060m`, open
  grippers, arms pull out `0.150m`, chassis retreat `0.45m` at `0.20m/s`,
  then arms default.
- `NAV_TO_RECOVERY` and `NAV_TO_HOME` both completed; final `HOME_SAFE` yaw
  error was about `0.16deg`.

Recommended next physical validation:

- Continue with a single rod first, preferably rod 2 only, using the same
  `1.5deg` yaw tolerance.
- Do not jump directly to all 7 rods until rod 2 completes cleanly, because rod
  2 was the previously confused capture point and should be validated under the
  new latest-pose selector.

## Next-Session Resume State

Use this section first when resuming after 2026-06-12.

Operator-facing opener:

```text
继续 /home/davie/G2/G2_dipan_yundong 七根料流程。先读
rack_hybrid_docking_package/industrial_cell_20260612_handoff.md 的
"Next-Session Resume State" 和 "2026-06-12 Live Rod-1 New-Logic Validation"。
今天停在第一根完整实跑通过，机器人回 HOME_SAFE。先做只读 readiness，
如果 ok，再只跑第二根验证，不要直接跑 7 根。
```

Confirmed final state at stop:

- Local workspace: `/home/davie/G2/G2_dipan_yundong`.
- Robot host: `agi@10.20.15.60`.
- Robot project root:
  `/data/g2_industrial_cell_20260612/wxf/BOX_528_1`.
- Active full-local safety lock is not present; the previous rack-fall lock was
  preserved as `logs/RACK_FALL_SAFETY_LOCK.cleared_20260612_1727_newlogic`.
- Rod 1 live checkpoint:
  `logs/industrial_cell_newlogic_live_rod1_checkpoint.json`.
- Rod 1 checkpoint status: `MISSION_DONE`, `current_station=HOME_SAFE`,
  `holding_rod=false`, `last_success_step=ROD_DONE`.
- Post-run readiness at stop: `ok=true`, `motion_control_error=0`,
  `pnc_task_state=7`, odom samples all `0.0`.

First command tomorrow should be read-only:

```bash
ssh agi@10.20.15.60 'source /home/agi/app/env.sh; cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1; python3 rack_hybrid_docking_package/industrial_map_nav_guarded.py --readiness-check'
```

Late 2026-06-12 local network attempt:

- Read-only readiness was not executed because the workstation could not reach
  `10.20.15.60`.
- `ssh agi@10.20.15.60 ... --readiness-check` returned
  `No route to host`; `ping -c 2 -W 2 10.20.15.60` had 100% packet loss.
- Local route was present on `wlp3s0f0` (`10.20.15.107/24`), but
  `ip neigh show 10.20.15.60` stayed `INCOMPLETE`.
- The old host `10.20.15.199` also failed reachability (`FAILED` neighbor
  entry), so no robot command was executed and no physical motion was sent.
- Next session should first restore robot network reachability, then repeat the
  read-only readiness command above before considering rod 2.

Follow-up at `2026-06-12T22:30:55-07:00`:

- Workstation network reachability to `10.20.15.60` recovered:
  `ping -c 2 -W 2 10.20.15.60` returned 2/2 packets.
- Read-only readiness reached the robot and executed successfully, but returned
  `ok=false` because the robot is charging:
  `charge_plug_insert_state=1`, `charge_input_current_a=15.0`,
  `charge_input_voltage_v=50.5`.
- Other key read-only fields were clean enough to keep inspecting only:
  `map_id=19`, `motion_control_error=0`, `pnc_task_state=0`,
  odom speed samples all `0.0`.
- No rod 2 run or physical motion was started. Next live step remains blocked
  until charging is fully disconnected and readiness returns `ok=true`.

Place-down mismatch review after operator feedback:

- The older seven-rods flow did not open the grippers immediately at place. Its
  sequence was: move to place-above, front-ultrasonic place fine-position, run
  `offset_move_down.py`, then open grippers, then pull out.
- The current station-based mission controller keeps the same ordering, but uses
  `move_ee_relative_offset.py` with the default `--place-down-z-m -0.06`
  instead of calling `offset_move_down.py`.
- Robot-side evidence from the rod-1 new-logic run confirms both place-down logs
  used only `left/right z=-0.06`.
- The current robot development copy's legacy `offset_move_down.py` is
  hardcoded to `offset_l=(0,0,-0.18)` and `offset_r=(0,0,-0.18)`. This is much
  larger than the station controller default and explains why the new flow can
  look like the material is being dropped from height even though a down-offset
  step exists.
- Do not run the next rod-2 physical validation with the implicit default if
  the place-down height is considered unsafe. Use an explicit reviewed value,
  for example a conservative `--place-down-z-m -0.12` single-rod trial after a
  full-dry-run plan check, or re-approve the legacy `-0.18` only after fixture
  clearance is checked.

Rod-2 place-only dry-run plan check:

- A temporary checkpoint at `/tmp/industrial_cell_place_plan_rod2_checkpoint.json`
  was created with `phase=LOCAL_PLACE`, `rod_index=2`, and
  `current_station=PLACE_PRE`.
- Running `industrial_cell_mission_controller.py --local-action-mode
  full-dry-run --execute-next --place-down-z-m -0.12` printed the place plan
  only. No arm, gripper, rack, navigation, or chassis command was sent.
- The printed order was:
  `arm_place_above -> place_fine_position -> place_down_offset(-0.12m) ->
  open_gripper_place -> place_pull_out_offset(-0.15m) ->
  retreat_after_place(0.45m) -> arm_default_after_place`.
- The planned `arm_default_after_place` path is `/data/wxf/wxf/positions/arm_default.json`.
  It exists and is byte-identical to
  `/data/g2_industrial_cell_20260612/wxf/positions/arm_default.json`
  (`sha256=57d0a253b9fd84239165069a962453269f6e95a323d9906942b813086fb3d960`).

## 2026-06-13 Live Rod-2 Place-Down Validation

This section supersedes the earlier rod-2-next recommendation.

- Readiness before the run returned `ok=true`: no charging, charge input
  `0V/0A`, `motion_control_error=0`, and odom speed samples all `0.0`.
- Physical rod 2 was run as a single-rod mission only, with explicit
  `--place-down-z-m -0.12`.
- Main run log:
  `logs/industrial_cell_newlogic_live_rod2_placedown012_20260613_144447.log`.
- Final checkpoint:
  `logs/industrial_cell_newlogic_live_rod2_checkpoint.json`.
- Final checkpoint state:
  `MISSION_DONE`, `current_station=HOME_SAFE`, `holding_rod=false`,
  `last_success_step=ROD_DONE`.
- Final readiness after the run returned `ok=true`, `problems=[]`,
  `warnings=[]`, `map_id=19`, `charge_plug_insert_state=0`,
  `charge_input_current_a=0.0`, `charge_input_voltage_v=0.0`,
  `motion_control_error=0`, `pnc_task_state=7`, and odom speed samples all
  `0.0`.
- Final pose after the run was approximately position
  `{x=0.599956, y=-0.971757, z=0.004772}`.

Rod-2 important evidence:

- `NAV_TO_GRAB` arrived and yaw-refine passed within the `1.5deg` tolerance.
- `LOCAL_PICK` used the correct valid rod-2 pose:
  `rack_hybrid_docking_package/calibration_records/rod02_grab_pose_20260612_161238.json`.
- Grab fine-position completed at front ultrasonic `filtered=323mm`,
  raw `0=323mm`, `1=311mm`.
- Pick sequence completed: close grippers, arms down `0.020m`, arms back
  `0.200m`, local retreat `0.45m`, waist home.
- `NAV_TO_PLACE` arrived and yaw-refine passed within the `1.5deg` tolerance.
- Place fine-position completed with front ultrasonic `filtered=403mm`,
  raw `0=395mm`, `1=379mm`.
- The place-down correction was applied before opening the grippers:
  `move_ee_relative_offset.py --left=0,0,-0.120000 --right=0,0,-0.120000`
  returned `relative_offset_result=True`.
- Then grippers opened successfully, arms pulled out `0.150m`, local retreat
  `0.45m` completed, and arms returned to default.
- `NAV_TO_RECOVERY` and `NAV_TO_HOME` completed; final HOME yaw error was inside
  tolerance.

Next-session resume state after rod 2:

- Do not rerun rods 1 or 2 unless the physical scene is manually reset and a
  specific reason is recorded.
- First command next time should still be read-only readiness:

```bash
ssh agi@10.20.15.60 'source /home/agi/app/env.sh; cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1; python3 rack_hybrid_docking_package/industrial_map_nav_guarded.py --readiness-check'
```

- If readiness is clean and the physical scene is re-approved, the next
  validation should be rod 3 only, with the same explicit place-down setting:

```bash
ssh agi@10.20.15.60 'source /home/agi/app/env.sh; cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1; python3 rack_hybrid_docking_package/industrial_cell_mission_controller.py --checkpoint-file logs/industrial_cell_newlogic_live_rod3_checkpoint.json --init --staging --local-action-mode full --start-index 3 --end-index 3 --run-current-rod --confirm-live --confirm-local-physical --allow-estop-pedal-fault --nav-log-dir logs --full-local-log-dir logs --refine-yaw --refine-yaw-tolerance-deg 1.5 --refine-yaw-max-error-deg 10.0 --refine-yaw-angular-speed-radps 0.05 --refine-yaw-fine-angular-speed-radps 0.02 --refine-yaw-timeout-s 12 --place-down-z-m -0.12'
```

Keep the boundary narrow:

- Validate rod 3 as a single-rod live run first.
- Watch that rod 3 uses
  `rack_hybrid_docking_package/calibration_records/rod03_grab_pose_20260612_161855.json`.
- Continue carrying explicit `--place-down-z-m -0.12` until enough rod evidence
  exists to promote it to the controller default or choose a different value.

## 2026-06-13 Live Rod-3 and Rod-4 Pause State

Rod 3 completed as a single-rod live run.

- Main run log:
  `logs/industrial_cell_newlogic_live_rod3_skipwaist_placedown012_20260613_145404.log`.
- Final checkpoint:
  `logs/industrial_cell_newlogic_live_rod3_checkpoint.json`.
- Final checkpoint state:
  `MISSION_DONE`, `current_station=HOME_SAFE`, `holding_rod=false`,
  `last_success_step=ROD_DONE`.
- Rod 3 used explicit `--place-down-z-m -0.12` and
  `--skip-waist-home-after-pick`.
- `LOCAL_PICK` did not include `waist_home_after_pick`; after local retreat it
  went directly to `NAV_TO_PLACE`.
- Place-down was still applied before opening the grippers:
  `move_ee_relative_offset.py --left=0,0,-0.120000 --right=0,0,-0.120000`
  returned success.
- Post-run readiness was clean: no charging, `motion_control_error=0`,
  `pnc_task_state=7`, and odom samples all `0.0`.

Controller changes now synced to robot:

- Current controller path:
  `rack_hybrid_docking_package/industrial_cell_mission_controller.py`.
- Current local and robot sha256:
  `f8ed8247a866d98dd28fe5a45a88d8d9e072356303397167dfeddf3da3627ef1`.
- Added `--skip-pick-offsets-after-close`: after gripper close in
  `LOCAL_PICK`, skip `pick_down_offset` and `pick_back_offset`, then go
  directly to `retreat_after_pick`.
- Added `--stop-after-local-step place_fine_position`: pause after the named
  local step without advancing the checkpoint.
- Fixed the pause loop check so `run-current-rod` exits after a pause even
  though saving the checkpoint refreshes `updated_at`.
- Robot-side backups made during this edit chain:
  `industrial_cell_mission_controller.py.bak_20260613_skip_pick_offsets_after_close`,
  `industrial_cell_mission_controller.py.bak_20260613_stop_after_place_fine_position`,
  `industrial_cell_mission_controller.py.bak_20260613_dryrun_pause_sim`,
  `industrial_cell_mission_controller.py.bak_20260613_pause_loop_fix`.

Rod 4 was intentionally stopped for new place-pose capture.

- Main run log:
  `logs/industrial_cell_newlogic_live_rod4_stop_placefine_nooffset_skipwaist_20260613_150404.log`.
- Checkpoint:
  `logs/industrial_cell_newlogic_live_rod4_stop_placefine_checkpoint.json`.
- Checkpoint state after stop:
  `rod_index=4`, `phase=LOCAL_PLACE`, `holding_rod=true`,
  `current_station=PLACE_PRE`, `last_success_step=NAV_TO_PLACE`.
- Pre-run readiness was clean: no charging, `motion_control_error=0`,
  `pnc_task_state=7`, and odom samples all `0.0`.
- `LOCAL_PICK` used
  `rack_hybrid_docking_package/calibration_records/rod04_grab_pose_20260612_162209.json`.
- The rod-4 pick plan contained:
  `open_gripper -> waist_for_grab -> arm_grab_pose -> grab_fine_position ->
  close_gripper -> retreat_after_pick`.
- It did not contain `pick_down_offset`, `pick_back_offset`, or
  `waist_home_after_pick`.
- `NAV_TO_PLACE` completed and yaw-refine passed within `1.5deg`.
- `LOCAL_PLACE` reached `arm_place_above` and `place_fine_position`, then
  paused before `place_down_offset`, gripper open, pull-out, retreat, or arm
  default.
- The first place fine-position result stopped with front ultrasonic
  `filtered=392mm`, raw `0=392mm`, `1=382mm`.
- Because the first pause-loop implementation compared the whole state,
  `updated_at` changed on save and the controller repeated the same
  `arm_place_above -> place_fine_position` pause cycle. The process was killed
  before any `place_down_offset` or place gripper-open child command was
  started.
- The log contains 10 completed `arm_place_above` and 10 completed
  `place_fine_position` events before the stop. Subsequent code was fixed and
  dry-run verified to pause once and exit.
- Final read-only readiness after the kill returned `ok=true`, `problems=[]`,
  `warnings=[]`, `map_id=19`, `charge_plug_insert_state=0`,
  `charge_input_current_a=0.0`, `charge_input_voltage_v=0.0`,
  `motion_control_error=0`, `pnc_task_state=7`, and odom speed samples all
  `0.0`.
- Final chassis/map pose at the capture point was approximately position
  `{x=2.158482, y=0.068164, z=-0.020570}`, orientation
  `{x=0.002073, y=-0.000234, z=0.250850, w=0.968024}`.

Current operator boundary:

- The robot is at the rod-4 place fine-position/capture point and is still
  holding the rod.
- Do not run `LOCAL_PLACE` completion until the operator has captured and
  approved the new place pose.
- Do not rerun rods 1, 2, or 3.
- Before any next motion, run read-only readiness:

```bash
ssh agi@10.20.15.60 'source /home/agi/app/env.sh; cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1; python3 rack_hybrid_docking_package/industrial_map_nav_guarded.py --readiness-check'
```

- The current rod-4 resume checkpoint is already at `LOCAL_PLACE`. If the new
  place pose is saved and the operator approves completing the placement, resume
  from this checkpoint deliberately; do not use `--init`.

### Rod-4 Place-Above Arm Pose Captured

At `2026-06-13 15:13:42` robot-local time, the current arm pose was recorded as
the active place-above point.

- Active place-above JSON updated:
  `/data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_grab_2.json`.
- Previous active JSON backup:
  `/data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_grab_2.json.bak_20260613_151342_before_place_above_capture`.
- Full read-only capture record:
  `rack_hybrid_docking_package/calibration_records/rod04_place_above_arm_pose_20260613_151342.json`.
- Active JSON copy:
  `rack_hybrid_docking_package/calibration_records/rod04_place_above_active_json_20260613_151342.json`.
- Active JSON sha256:
  `96942aec5befdf5596d459fc11cb3b26928d916175d91d344b987204c7bba775`.
- Capture record sha256:
  `41e6ad214afddb26e205c2327a3ed6955d6d074560143a21cd6a511391e9fdf7`.
- `move_arm_by_json_path.py --json /data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_grab_2.json --dry-run`
  succeeded, so the new JSON format is consumable by the existing arm move
  primitive. No arm motion was sent by this validation.

Captured active arm joint values:

```json
{
  "idx21_arm_l_joint1": 1.0369736591270269,
  "idx22_arm_l_joint2": -1.8437462369201532,
  "idx23_arm_l_joint3": -1.0441412942138173,
  "idx24_arm_l_joint4": -2.2648806885259805,
  "idx25_arm_l_joint5": 0.514259868603718,
  "idx26_arm_l_joint6": 0.8982972319098466,
  "idx27_arm_l_joint7": 0.044812012977354564,
  "idx61_arm_r_joint1": -1.2003701268198592,
  "idx62_arm_r_joint2": -1.717469807317713,
  "idx63_arm_r_joint3": 1.2098810873366648,
  "idx64_arm_r_joint4": -2.350000084023264,
  "idx65_arm_r_joint5": -0.38515297480256827,
  "idx66_arm_r_joint6": 0.5481325612452139,
  "idx67_arm_r_joint7": -0.2609924499888554
}
```

Important state after capture:

- The capture was read-only and sent no motion commands.
- A later readiness check still showed the chassis stationary and not charging,
  but returned `ok=false` because `motion_control_error=2`.
- Do not continue placement until that fault state is understood or cleared by
  the operator-approved recovery path.

### Rod-4 Place Point Captured and Updated Local Logic

At `2026-06-13 15:18:38` robot-local time, the operator indicated the current
arm pose was the actual rod-4 place point. It was captured read-only.

- Legacy/full place JSON updated:
  `/data/g2_industrial_cell_20260612/wxf/positions/waist_to_put.json`.
- New explicit arm place JSON created:
  `/data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_place_rod04.json`.
- Previous `waist_to_put.json` backup:
  `/data/g2_industrial_cell_20260612/wxf/positions/waist_to_put.json.bak_20260613_151838_before_rod04_place_capture`.
- Full read-only capture record:
  `rack_hybrid_docking_package/calibration_records/rod04_place_arm_pose_20260613_151838.json`.
- Active full-place JSON copy:
  `rack_hybrid_docking_package/calibration_records/rod04_place_waist_to_put_active_json_20260613_151838.json`.
- Active arm-place JSON copy:
  `rack_hybrid_docking_package/calibration_records/rod04_place_arm_position_to_place_active_json_20260613_151838.json`.
- `waist_to_put.json`, `arm_position_to_place_rod04.json`, and both active
  copies have sha256:
  `30bf3fcb3ba4769d52e0d8d635d94ddbfb349dd991fb34df3a9e5358f0e6c5f0`.
- Capture record sha256:
  `4bd75a2167c76f78ad1931043277346c4391c7133aeb46da613451b0ee5964d6`.
- `move_arm_by_json_path.py --json /data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_place_rod04.json --dry-run`
  succeeded. No arm motion was sent by this validation.

Captured place-point joint values:

```json
{
  "idx01_body_joint1": -0.6979205994214123,
  "idx02_body_joint2": 1.5709187959692616,
  "idx03_body_joint3": -0.8728094679385512,
  "idx04_body_joint4": 0.0,
  "idx05_body_joint5": 0.0,
  "idx11_head_joint1": 0.0,
  "idx12_head_joint2": 0.0,
  "idx13_head_joint3": 0.0,
  "idx21_arm_l_joint1": 1.0578059371485489,
  "idx22_arm_l_joint2": -1.8066805772502692,
  "idx23_arm_l_joint3": -1.3004968079022923,
  "idx24_arm_l_joint4": -2.0370235418201945,
  "idx25_arm_l_joint5": 0.5669662500529781,
  "idx26_arm_l_joint6": 0.9509765290108206,
  "idx27_arm_l_joint7": -0.07625394528129711,
  "idx61_arm_r_joint1": -1.2200295486973105,
  "idx62_arm_r_joint2": -1.6799928390622227,
  "idx63_arm_r_joint3": 1.3879262825923324,
  "idx64_arm_r_joint4": -2.0787250492233635,
  "idx65_arm_r_joint5": -0.38926368378735465,
  "idx66_arm_r_joint6": 0.5521590211289157,
  "idx67_arm_r_joint7": -0.11850864450609763
}
```

Controller logic updated and synced:

- Current controller sha256:
  `01bfb3a941b6fd13fa8fd31fd3903c501126ffc1c75bb8dc47f4be438fe10e40`.
- New robot-side backups:
  `industrial_cell_mission_controller.py.bak_20260613_skip_down_keep_back`,
  `industrial_cell_mission_controller.py.bak_20260613_place_pose_logic`,
  `industrial_cell_mission_controller.py.bak_20260613_start_at_local_step`.
- Use `--skip-pick-down-after-close` instead of
  `--skip-pick-offsets-after-close` for future rods. This skips only the
  post-grasp down motion and keeps `pick_back_offset(-0.20m)` before chassis
  retreat.
- Use `--use-place-pose-json --place-pose-json
  /data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_place_rod04.json`
  for the new place-point logic.
- Use `--skip-place-pull-out-after-open` so after gripper open the arms do not
  pull out; chassis retreats first, then arms return to default.
- Optional resume-only flag `--start-at-local-step open_gripper_place` is
  available for the current physical state if the arms are already at the
  recorded place point and the operator approves opening immediately.

Dry-run verified local plans:

- Full future `LOCAL_PLACE` plan:
  `arm_place_above -> place_fine_position -> arm_place_pose ->
  open_gripper_place -> retreat_after_place -> arm_default_after_place`.
- Current-state resume plan with `--start-at-local-step open_gripper_place`:
  `open_gripper_place -> retreat_after_place -> arm_default_after_place`.
- Future `LOCAL_PICK` plan with `--skip-pick-down-after-close`:
  `open_gripper -> waist_for_grab -> arm_grab_pose -> grab_fine_position ->
  close_gripper -> pick_back_offset(-0.20m) -> retreat_after_pick`.

Latest safety boundary:

- The last readiness check after place-point capture showed the robot had
  reconnected to charging: `charge_plug_insert_state=1`,
  `charge_input_current_a≈14.8`, `charge_input_voltage_v=51.5`.
- `motion_control_error=0`, `pnc_task_state=7`, and odom speed samples were all
  `0.0`.
- Do not run physical placement or recovery while charging is connected. First
  disconnect charging and repeat read-only readiness.

### Rod-4 Completed, Rod-5 Picked

After the operator disconnected charging on 2026-06-13, rod 4 was resumed from
the already-recorded place point and rod 5 was picked.

- Combined live log:
  `logs/industrial_cell_finish_rod4_then_pick_rod5_20260613_153018.log`.
- Active checkpoint:
  `logs/industrial_cell_newlogic_live_rod4_stop_placefine_checkpoint.json`.
- Final checkpoint state after the run:

```json
{
  "rod_index": 5,
  "end_index": 5,
  "phase": "NAV_TO_PLACE",
  "holding_rod": true,
  "current_station": "GRAB_PRE",
  "last_success_step": "LOCAL_PICK",
  "updated_at": 1781335914.0712676
}
```

Rod-4 placement completion:

- The checkpoint was deliberately extended to `end_index=5` after a backup so
  rod 4 could finish and the controller could continue to rod 5.
- Rod 4 used resume-only local step `--start-at-local-step open_gripper_place`
  because the arms were already at the recorded place point.
- Executed steps were:
  `open_gripper_place -> retreat_after_place -> arm_default_after_place`.
- No `place_pull_out_offset` was executed. This matches the current process:
  open grippers at the place point, keep arms still while the chassis retreats
  to the place target, then return arms to default.
- `NAV_TO_RECOVERY`, `NAV_TO_HOME`, and `ROD_DONE` completed, then the
  controller advanced to rod 5.

Rod-5 pick:

- `NAV_TO_GRAB` arrived at `GRAB_PRE` and yaw refinement passed within the
  `1.5deg` tolerance.
- `LOCAL_PICK` used
  `rack_hybrid_docking_package/calibration_records/rod05_grab_pose_20260612_162921.json`.
- The rod-5 pick plan was:
  `open_gripper -> waist_for_grab -> arm_grab_pose -> grab_fine_position ->
  close_gripper -> pick_back_offset -> retreat_after_pick`.
- It did not include `pick_down_offset`, and did not include
  `waist_home_after_pick`.
- `grab_fine_position` stopped with `front_filtered_mm=317`, raw ultrasonic
  distances `0=308`, `1=308`.
- Gripper close succeeded.
- `pick_back_offset` executed with left/right offsets `(-0.20m, 0, 0)` and
  returned `relative_offset_result=True`.
- Chassis `retreat_after_pick` completed with distance `0.45m`; rear raw
  ultrasonic sample was `[[4, 2337]]`.

Post-run read-only readiness:

- Readiness returned `ok=true`, `problems=[]`, `warnings=[]`.
- The robot was not charging:
  `charge_plug_insert_state=0`, charge input `0V/0A`.
- `motion_control_error=0`, `pnc_task_state=9`, and odom speed samples were all
  `0.0`.
- Current map remained `map_id=19`.

Current boundary:

- Rod 5 is currently held.
- The next phase is `NAV_TO_PLACE`, with next action "run guarded map
  navigation to `PLACE_PRE`".
- Do not rerun rod-5 pick unless the physical scene is manually reset and the
  checkpoint is intentionally changed.
- For rod-5 placement, do not use `--start-at-local-step open_gripper_place`.
  That flag was only for the rod-4 resume state. Rod 5 should run the full
  current `LOCAL_PLACE` plan:
  `arm_place_above -> place_fine_position -> arm_place_pose ->
  open_gripper_place -> retreat_after_place -> arm_default_after_place`.

### Rod-5 Placed and Grab Fine-Position Retuned

Rod 5 was then placed from the held `NAV_TO_PLACE` checkpoint.

- Main live log:
  `logs/industrial_cell_place_rod5_20260613_1540.log`.
- Active checkpoint:
  `logs/industrial_cell_newlogic_live_rod4_stop_placefine_checkpoint.json`.
- Final checkpoint state:

```json
{
  "rod_index": 5,
  "end_index": 5,
  "phase": "MISSION_DONE",
  "holding_rod": false,
  "current_station": "HOME_SAFE",
  "last_success_step": "ROD_DONE",
  "updated_at": 1781336343.2644393
}
```

Rod-5 placement evidence:

- `NAV_TO_PLACE` arrived at `PLACE_PRE`; final map-nav error was about
  `xy_error_m=0.0130`, `yaw_error_deg=-0.4787`.
- Yaw refine passed with final error about `-0.4839deg`.
- Full `LOCAL_PLACE` plan ran:
  `arm_place_above -> place_fine_position -> arm_place_pose ->
  open_gripper_place -> retreat_after_place -> arm_default_after_place`.
- `place_fine_position` stopped with `front_filtered_mm=397`, raw ultrasonic
  distances `0=398`, `1=385`.
- `arm_place_pose` used
  `/data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_place_rod04.json`.
- Grippers opened successfully.
- No `place_pull_out_offset` was executed.
- `retreat_after_place` completed with distance `0.45m`; rear raw ultrasonic
  sample was `[[5, 2383]]`.
- Arms returned to default after the chassis retreat.
- `NAV_TO_RECOVERY` and `NAV_TO_HOME` completed; final HOME yaw refine error
  was about `-0.3928deg`.

Post-run read-only readiness:

- Readiness returned `ok=true`, `problems=[]`, `warnings=[]`.
- The robot was not charging:
  `charge_plug_insert_state=0`, charge input `0V/0A`.
- `motion_control_error=0`, `pnc_task_state=7`, and odom speed samples were all
  `0.0`.
- Current station/checkpoint is `HOME_SAFE`, `MISSION_DONE`, no rod held.

Grab fine-position distance retune:

- Operator observed that the grab fine-position stopped too close to the rack.
- Controller default `--grab-final-stop-mm` was changed from `316` to `341`,
  moving future grab fine-position stops `25mm` farther back from the rack.
- Current local and robot controller sha256:
  `d4a5d793039c121aa5642e4c28293a7a9f77ad19c98ad1338c8903d309adf3fa`.
- Robot-side backup before this edit:
  `industrial_cell_mission_controller.py.bak_20260613_grab_stop_341mm_153948`.
- Robot-side `python3 -m py_compile` passed after sync.
- Dry-run rod-6 `LOCAL_PICK` plan using the new default confirmed:
  `grab_fine_position.final_stop_mm=341`.
- The same dry-run confirmed the future pick plan still skips
  `pick_down_offset`, keeps `pick_back_offset(-0.20m)`, then runs
  `retreat_after_pick`.

Current boundary after rod 5:

- Rod 5 is complete and the robot is at `HOME_SAFE`.
- No rod is held.
- To continue, start rod 6 deliberately from a new or reset checkpoint; do not
  reuse the rod-5 `MISSION_DONE` checkpoint as if it were mid-cycle.

### Rod-6 Stopped After Opening at Place Point

Rod 6 was started as a deliberate single-rod live run with the retuned grab
fine-position distance.

- Main live log:
  `logs/industrial_cell_newlogic_live_rod6_grabstop341_20260613_154144.log`.
- Active checkpoint:
  `logs/industrial_cell_newlogic_live_rod6_grabstop341_checkpoint.json`.
- Controller command explicitly used `--grab-final-stop-mm 341`.

Rod-6 pick evidence:

- `NAV_TO_GRAB` arrived at `GRAB_PRE`; yaw refine passed with final error about
  `-1.3706deg`.
- `LOCAL_PICK` used
  `rack_hybrid_docking_package/calibration_records/rod06_grab_pose_20260612_163400.json`.
- The pick plan contained:
  `open_gripper -> waist_for_grab -> arm_grab_pose -> grab_fine_position ->
  close_gripper -> pick_back_offset -> retreat_after_pick`.
- The plan did not include `pick_down_offset` or `waist_home_after_pick`.
- `grab_fine_position` used `final_stop_mm=341` and stopped with
  `front_filtered_mm=346`, raw ultrasonic distances `0=341`, `1=341`.
- Gripper close succeeded.
- `pick_back_offset(-0.20m)` succeeded.
- `retreat_after_pick` completed with distance `0.45m`; rear raw ultrasonic
  sample was `[[4, 2316]]`.

Rod-6 place progress:

- `NAV_TO_PLACE` arrived at `PLACE_PRE`; final map-nav error was about
  `xy_error_m=0.0092`, `yaw_error_deg=1.2558`.
- Yaw refine passed with final error about `1.2723deg`.
- `LOCAL_PLACE` completed these steps:
  `arm_place_above -> place_fine_position -> arm_place_pose ->
  open_gripper_place`.
- `place_fine_position` stopped with `front_filtered_mm=394`, raw ultrasonic
  distances `0=395`, `1=385`.
- `arm_place_pose` used
  `/data/g2_industrial_cell_20260612/wxf/positions/arm_position_to_place_rod04.json`.
- Grippers opened successfully.

Stop reason:

- The next step, `retreat_after_place`, was blocked before chassis control by
  rear ultrasonic guard:
  `status=rear_obstacle`, `rear_filtered_mm=270`,
  `rear_raw=[[4, 270], [5, 1264]]`.
- No chassis retreat was sent after the grippers opened.
- Because the exception happened inside `LOCAL_PLACE`, the checkpoint still
  says `phase=LOCAL_PLACE`, `holding_rod=true`, `last_success_step=NAV_TO_PLACE`
  even though the physical grippers have already opened.

Read-only state after the stop:

- Readiness returned `ok=true`, no charging, `motion_control_error=0`, and odom
  speed samples all `0.0`.
- Current map pose was approximately position
  `{x=2.150888, y=0.060345, z=-0.020956}`, orientation
  `{x=0.002235, y=-0.000286, z=0.250282, w=0.968170}`.
- A read-only ultrasonic snapshot showed intermittent close readings:
  rear id `5` reported `201mm` and `189mm` on early samples, then rear became
  invalid; left id `6` later reported about `225-228mm`.

Current boundary after rod 6 stop:

- Do not rerun the full rod-6 `LOCAL_PLACE`; it would repeat already-completed
  place steps.
- If the operator confirms the rear/left-rear area is clear, the intended
  recovery is to resume only from:
  `--start-at-local-step retreat_after_place`.
- That recovery should execute only:
  `retreat_after_place -> arm_default_after_place`, then continue to
  `NAV_TO_RECOVERY -> NAV_TO_HOME -> ROD_DONE`.
- If the rear or left-rear area is not clearly safe, stop here and inspect the
  physical scene before any more chassis or arm motion.

Operator correction after the rod-6 stop:

- Rod 6 and rod 7 should return the waist/body to HOME after picking, matching
  the earlier rods.
- Do not use `--skip-waist-home-after-pick` for rod 7 or any future rod-6
  rerun.
- The already-executed rod-6 pick skipped `waist_home_after_pick`; after the
  rear/left-rear obstacle is cleared and `retreat_after_place ->
  arm_default_after_place` is recovered, manually run the same waist-home
  primitive used by `waist_home_after_pick` before continuing to recovery/HOME:
  `move_waist_by_json_path.py --json /data/wxf/wxf/positions/arm_default.json`.

Pick-back/down logic correction:

- After gripper close, future picks should move backward and downward at the
  same time instead of doing a separate down step or only moving backward.
- The intended ratio is 5cm backward to 0.5cm downward; the current 20cm pick
  back therefore uses 2cm downward.
- Local controller change: when `--skip-pick-down-after-close` is used,
  `pick_back_offset` now combines `--pick-back-x-m` and `--pick-down-z-m` in
  one `move_ee_relative_offset.py` command.
- With current defaults, the planned offset is:
  `left=(-0.20, 0.0, -0.02)`, `right=(-0.20, 0.0, -0.02)`.
- Local `python3 -m py_compile` passed.
- Local dry-run rod-7 `LOCAL_PICK` plan confirmed:
  `pick_back_offset.left/right=[-0.2, 0.0, -0.02]`, followed by
  `retreat_after_pick`, then `waist_home_after_pick`.
- New local controller sha256:
  `d49ad605996afb60add4a278810a11908c86605bfb01446b8b0fc2b95b40a572`.
- Robot sync completed after SSH recovered; robot-side `python3 -m py_compile`
  passed and robot-side controller sha256 is also
  `d49ad605996afb60add4a278810a11908c86605bfb01446b8b0fc2b95b40a572`.

Resume attempt after SSH recovered:

- SSH to `10.20.15.60` recovered.
- PNC had a stale type-3 task in state `2`; `industrial_cancel_pnc_task.py
  --confirm-live` cancelled it successfully and task state returned to `7`.
- A read-only status snapshot showed rear/left ultrasonic no longer reporting
  the previous close obstacle; rear ids `4/5` were invalid and left ids `6/7`
  were about `1550/1790mm`.
- The same snapshot showed `charge_plug_insert_state=1`,
  `charge_plug_input_voltage=1.5`, and repeated `Slam odom is null` /
  `GetOdomInfo failed`.
- Re-running `industrial_map_nav_guarded.py --readiness-check` still failed
  before returning an `ok` field because `Slam.get_curr_pose()` raised
  `RuntimeError: GetCurrPose failed`.
- Do not resume rod-6 chassis retreat until charging/plug state is clear and
  Slam odom/current pose is available again.

### Rod-6 Recovered and Rod-7 Completed

After charging state cleared and Slam odom returned, rod 6 was recovered from
the post-open stop point and rod 7 was completed.

Rod-6 recovery:

- Recovery log:
  `logs/industrial_cell_rod6_recover_retreat_after_place_20260613_173458.log`.
- The controller resumed `LOCAL_PLACE` with
  `--start-at-local-step retreat_after_place`.
- It skipped the already-completed steps:
  `arm_place_above`, `place_fine_position`, `arm_place_pose`,
  `open_gripper_place`.
- It executed only:
  `retreat_after_place -> arm_default_after_place`.
- `retreat_after_place` completed `0.45m` with no rear obstacle.
- `arm_default_after_place` succeeded.
- Manual waist HOME correction log:
  `logs/industrial_cell_rod6_manual_waist_home_after_recover_20260613_173530.log`.
- Waist was already at target with `final_max_error_rad=0.000478`.
- Finish log:
  `logs/industrial_cell_rod6_finish_recovery_home_20260613_173613.log`.
- `NAV_TO_RECOVERY`, `NAV_TO_HOME`, and `ROD_DONE` completed.
- Rod-6 final checkpoint:

```json
{
  "rod_index": 6,
  "end_index": 6,
  "phase": "MISSION_DONE",
  "holding_rod": false,
  "current_station": "HOME_SAFE",
  "last_success_step": "ROD_DONE",
  "updated_at": 1781343405.6976998
}
```

Rod-7 full run:

- Main live log:
  `logs/industrial_cell_newlogic_live_rod7_backdown341_20260613_173732.log`.
- Active checkpoint:
  `logs/industrial_cell_newlogic_live_rod7_backdown341_checkpoint.json`.
- `NAV_TO_GRAB` arrived at `GRAB_PRE`; yaw refine final error was about
  `-1.3730deg`.
- `LOCAL_PICK` used
  `rack_hybrid_docking_package/calibration_records/rod07_grab_pose_20260612_163717.json`.
- `grab_fine_position` used `final_stop_mm=341` and stopped with
  `front_filtered_mm=350`, raw ultrasonic distances `0=343`, `1=341`.
- Gripper close succeeded.
- `pick_back_offset` executed the new combined back/down offset in one command:
  `left=(-0.20, 0.0, -0.02)`, `right=(-0.20, 0.0, -0.02)`,
  `relative_offset_result=True`.
- `retreat_after_pick` completed `0.45m`, rear raw ultrasonic sample
  `[[4, 2303], [5, 1946]]`.
- `waist_home_after_pick` executed and reached HOME with
  `final_max_error_rad=0.000111`.
- `NAV_TO_PLACE` arrived at `PLACE_PRE`; yaw refine final error was about
  `1.3958deg`.
- `LOCAL_PLACE` completed:
  `arm_place_above -> place_fine_position -> arm_place_pose ->
  open_gripper_place -> retreat_after_place -> arm_default_after_place`.
- `place_fine_position` stopped with `front_filtered_mm=396`, raw ultrasonic
  distances `0=396`, `1=385`.
- Grippers opened successfully.
- `retreat_after_place` completed `0.45m`, rear raw ultrasonic sample
  `[[4, 2091]]`.
- `NAV_TO_RECOVERY`, `NAV_TO_HOME`, and `ROD_DONE` completed.
- Rod-7 final checkpoint:

```json
{
  "rod_index": 7,
  "end_index": 7,
  "phase": "MISSION_DONE",
  "holding_rod": false,
  "current_station": "HOME_SAFE",
  "last_success_step": "ROD_DONE",
  "updated_at": 1781343625.4698553
}
```

Final read-only readiness:

- Readiness returned `ok=true`, `problems=[]`, `warnings=[]`.
- The robot was not charging:
  `charge_plug_insert_state=0`, charge input `0V/0A`.
- `motion_control_error=0`, `pnc_task_state=7`, and odom speed samples were all
  `0.0`.
- Current map remained `map_id=19`.
- Current station/checkpoint is `HOME_SAFE`; no rod is held.

### Waist Speed and Place Right-Shift Patch

Applied on 2026-06-13 after the completed rod-7 run.

- Controller file:
  `rack_hybrid_docking_package/industrial_cell_mission_controller.py`.
- Robot-side backup before this patch:
  `industrial_cell_mission_controller.py.bak_20260613_025401`.
- Robot-side patched controller hash:
  `1d365d9807c93811db086c263942d5927b4ccc02473ad87ca7086de70efcf1f7`.
- Waist/body speed is now independent of `--arm-joint-speed-radps`.
  Default `--waist-joint-speed-radps` is `0.20`, matching the existing
  `move_waist_by_json_path.py` safety cap.
- Waist segmented max step is now `0.20rad`; the child waist command still
  enforces its own `<=0.20` cap.
- `LOCAL_PLACE` now inserts a chassis relative move after
  `place_fine_position` and before `arm_place_pose`:
  this move is now disabled by default with `place_lateral_right_m=0.0`.
- The right-shift uses `Pnc.relative_move(NaviReq)` task monitoring, not
  open-loop `linear.y`. If the task does not start, cancels, or times out, the
  controller raises and does not continue to the arm place pose.
- Robot-side no-motion verification passed:
  `python3 rack_hybrid_docking_package/industrial_cell_mission_controller.py`
  with checkpoint
  `logs/dryrun_waist_lateral_patch_checkpoint.json`,
  `--run-current-rod --staging --local-action-mode full-dry-run`,
  `--use-place-pose-json --skip-pick-down-after-close
  --skip-place-pull-out-after-open`.
- Dry-run evidence:
  - `LOCAL_PICK`: `waist_for_grab` and `waist_home_after_pick` both show
    `joint_speed_radps=0.2`.
  - `LOCAL_PICK`: the post-grab back/down arm motion is split into
    `pick_back_down_offset` with `left/right=(-0.085, 0.0, -0.02)`,
    followed by `pick_back_remaining_offset` with
    `left/right=(-0.115, 0.0, 0.0)`. Total arm retreat remains `0.20m`;
    the arms have lowered `0.02m` when the first `0.085m` retreat completes.
  - `LOCAL_PLACE`: step order is now
    `arm_place_above -> place_fine_position -> arm_place_pose ->
    open_gripper_place -> retreat_after_place -> arm_default_after_place`.

Before the next physical round, re-run the read-only readiness check and verify
that the material rack and place area have been physically reset. The previous
live checkpoint is already `MISSION_DONE` at `HOME_SAFE` with no rod held.

Post-patch readiness check:

- Command was run with robot env loaded:
  `industrial_map_nav_guarded.py --readiness-check`.
- Result blocked physical motion:
  `ok=false`, `charge_plug_insert_state=1`,
  `charge_input_current_a=12.199999809265137`,
  `charge_input_voltage_v=53.5`.
- Other motion gates were clean:
  `motion_control_error=0`, `pnc_task_state=7`, map `19`, odom speed samples
  were all `0.0`.
- Do not start the next physical round until the robot is off charge and a fresh
  readiness check returns `ok=true`.

Live patch validation:

- The next-round rod 1 completed through `ROD_DONE` and returned to `HOME_SAFE`.
- It used the earlier `3.5cm` right-shift patch:
  `place_lateral_right_offset status=completed`, `x_m=0.0`, `y_m=-0.035`,
  final PNC state `9`.
- After operator feedback, the default place right-shift was changed from
  `0.035m` to `0.050m` for rod 2 onward.
- The active checkpoint after rod 1 is:
  `logs/industrial_cell_next_round_patch_live_checkpoint.json`,
  `rod_index=2`, `phase=NAV_TO_GRAB`, `current_station=HOME_SAFE`,
  `holding_rod=false`.

Latest operator tuning before continuing rod 7:

- The default place right-shift was later changed from `0.050m` to `0.150m`,
  then retuned to `0.120m` after the rod-4 live validation.
- After rod 5, the operator requested returning to the pre-right-shift place
  motion, including no chassis right move after place fine-position. The
  default `--place-lateral-right-m` is now `0.0`, so
  `place_lateral_right_offset` is omitted from the place plan.
- The chassis relative guard default was increased from `0.10m` to `0.20m` so
  earlier right-shift tests were inside the configured bound.
- The post-grab arm motion now lowers during only the first `0.085m` of arm
  retreat:
  `pick_back_down_offset=(-0.085, 0.0, -0.02)` then
  `pick_back_remaining_offset=(-0.115, 0.0, 0.0)`.
- Robot-side controller hash after disabling the place right-shift:
  `c1d9f9ccd5c1f72219d84f0c8038fe7e415d1f94c9f70f16d4e2dc2bb664420e`.
- Robot-side backups before these patches:
  `industrial_cell_mission_controller.py.bak_20260613_031729_pick_split_85mm`.
  `industrial_cell_mission_controller.py.bak_20260613_032433_place_right_12cm`.
- No-motion dry-run verification passed with checkpoint:
  `logs/dryrun_disable_place_right_rod6_checkpoint.json`.
- Rod 4 then completed with the `0.150m` right-shift and the split pick-back
  arm motion.
- Rod 5 completed with the `0.120m` right-shift and returned HOME.
- Rod 6 completed with the no-right-shift place plan:
  `arm_place_above -> place_fine_position -> arm_place_pose ->
  open_gripper_place -> retreat_after_place -> arm_default_after_place`.
- Rod-6 live log:
  `logs/industrial_cell_next_round_patch_live_rod6_no_place_right_split85_20260613_0336.log`.
- Rod-6 grab evidence:
  `grab_fine_position` stopped with `front_filtered_mm=346`, raw ultrasonic
  distances `0=337`, `1=338`.
- Rod-6 pick-back evidence:
  `pick_back_down_offset=(-0.085, 0.0, -0.02)` and
  `pick_back_remaining_offset=(-0.115, 0.0, 0.0)` both returned
  `relative_offset_result=True`; `retreat_after_pick` completed `0.45m`.
- Rod-6 place evidence:
  `place_fine_position` stopped with `front_filtered_mm=393`, raw ultrasonic
  distances `0=393`, `1=385`, then went directly to `arm_place_pose` without
  any chassis right move.
- The active live checkpoint after rod 6 is:
  `logs/industrial_cell_next_round_patch_live_checkpoint.json`,
  `rod_index=7`, `phase=NAV_TO_GRAB`, `current_station=HOME_SAFE`,
  `holding_rod=false`.
- Read-only readiness after rod 6 returned `ok=true`, with no charging,
  `motion_control_error=0`, `pnc_task_state=7`, and zero odom speed samples.
- The no-right-shift dry-run showed no `place_lateral_right_offset` step
  between `place_fine_position` and `arm_place_pose`.

### Place With Grab Pose Patch

Applied on 2026-06-13 after operator feedback that the board is held by both
hands and the place action must preserve the same hand-to-board geometry used
at grab time.

- Controller file:
  `rack_hybrid_docking_package/industrial_cell_mission_controller.py`.
- Robot-side backup before this patch:
  `industrial_cell_mission_controller.py.bak_20260613_place_use_grab_pose`.
- Robot-side patched controller hash:
  `085481ce0681ecb73bd60dd375cd671244ccf90e75d7eb99d4eca9032a5042f4`.
- New place mode:
  `--place-use-grab-pose --place-grab-z-offset-m <meters>`.
- This mode is mutually exclusive with `--use-place-pose-json`.
- In `LOCAL_PLACE`, this mode replaces both old place arm steps
  (`arm_place_above` and `arm_place_pose`) with:
  - `arm_place_grab_pose`: move both arms to the current rod's captured
    `rodXX_grab_pose_*.json`;
  - optional `place_grab_pose_z_offset`: move both end effectors by the same
    pure-Z offset using `move_ee_relative_offset.py`;
  - then `place_fine_position`, `open_gripper_place`, chassis retreat, and arm
    default.
- No-motion robot-side verification passed:
  - `python3 -m py_compile rack_hybrid_docking_package/industrial_cell_mission_controller.py`
  - `--help` shows `--place-use-grab-pose` and `--place-grab-z-offset-m`.
  - dry-run plan for rod 7 with an example `0.02m` Z offset showed:
    `arm_place_grab_pose -> place_grab_pose_z_offset ->
    place_fine_position -> open_gripper_place -> retreat_after_place ->
    arm_default_after_place`.
  - The plan used
    `rack_hybrid_docking_package/calibration_records/rod07_grab_pose_20260612_163717.json`
    for `arm_place_grab_pose`.
- Current important caveat:
  the existing grab/place calibration JSONs contain joint positions and chassis
  pose, but not left/right end-effector XYZ at the recorded place point. Do not
  invent `--place-grab-z-offset-m`; either provide the measured vertical offset
  or pause at the place point and recapture left/right end-effector height.
- Checkpoint distinction:
  - default `logs/industrial_cell_mission_checkpoint.json` is at
    `rod_index=1`, `phase=NAV_TO_GRAB`;
  - the previous live run checkpoint remains
    `logs/industrial_cell_next_round_patch_live_checkpoint.json`,
    `rod_index=7`, `phase=NAV_TO_GRAB`, `last_success_step=rod_6_completed`.
  Any continuation of the previous live run must explicitly use the latter
  checkpoint file.
- Latest read-only readiness after this patch returned `ok=true`:
  `charge_plug_insert_state=0`, `charge_input_current_a=0.0`,
  `charge_input_voltage_v=0.0`, `motion_control_error=0`,
  `pnc_task_state=7`, map `19`, and odom samples were all `0.0`.
## 2026-06-13 Rod 7 Place Waypoint Flow Update

Operator-confirmed LOCAL_PLACE sequence is now:

1. chassis arrives at `PLACE_PRE`;
2. waist/body goes to the straight/high place waist point;
3. arms go to the place-above point;
4. chassis performs place fine-positioning;
5. arms go to the place transition point;
6. arms go to the final place point;
7. grippers open;
8. arms retreat by `0.25m` in end-effector X;
9. chassis retreats back to the place target point;
10. arms return home/default;
11. waist/body returns to the default HOME/grab-safe waist pose from
    `/data/wxf/wxf/positions/arm_default.json`.

Controller update:

- `industrial_cell_mission_controller.py` now defaults LOCAL_PLACE to the
  calibrated waypoint sequence above unless `--place-use-grab-pose` or
  `--disable-place-waypoint-jsons` is explicitly used.
- New default waypoint files:
  - `rack_hybrid_docking_package/calibration_records/rod07_place_waist_adjusted_latest.json`
  - `rack_hybrid_docking_package/calibration_records/rod07_place_above_arm_latest.json`
  - `rack_hybrid_docking_package/calibration_records/rod07_place_transition_arm_latest.json`
  - `rack_hybrid_docking_package/calibration_records/rod07_place_final_arm_latest.json`
- Default `--place-pull-x-m` is now `-0.25`, matching the operator-requested
  hand retreat after gripper open.
- After `arm_default_after_place`, the controller appends
  `waist_home_after_place`, using `/data/wxf/wxf/positions/arm_default.json`.
  The older `waist_grab_after_place` label is historical wording only; the
  behavior is the default HOME/grab-safe waist pose.
- Robot-side controller hash after sync:
  `7a0c785e24b39b0c7cab5e0fd306f0539408c1c2d26ab0a2c79cac40d4932acd`.
- Robot-side backup before this patch:
  `rack_hybrid_docking_package/industrial_cell_mission_controller.py.bak_20260613_place_waypoints_flow`.

Validation:

- Local `python3 -m py_compile rack_hybrid_docking_package/industrial_cell_mission_controller.py` passed.
- Robot-side `python3 -m py_compile rack_hybrid_docking_package/industrial_cell_mission_controller.py` passed.
- Robot-side full-dry-run confirmed the LOCAL_PLACE order:
  `waist_place_straight -> arm_place_above -> place_fine_position ->
  arm_place_transition -> arm_place_pose -> open_gripper_place ->
  place_pull_out_offset(-0.25m) -> retreat_after_place ->
  arm_default_after_place -> waist_grab_after_place`.

Current live physical state after manual tuning:

- Rod 7 final place point was captured at
  `rack_hybrid_docking_package/calibration_records/rod07_place_final_capture_20260613_193951.json`.
- Grippers were opened successfully.
- Arms already retreated `0.25m` with
  `move_ee_relative_offset.py --left=-0.25,0,0 --right=-0.25,0,0 --max-abs-m 0.30`.
- Post-retreat readback showed `motion_error_code=0`, `pnc_state=7`, and no
  whole-body arm/end/waist/chassis errors.
- Checkpoint is still intentionally not advanced:
  `phase=LOCAL_PLACE`, `rod_index=7`, `holding_rod=true`,
  `last_success_step=NAV_TO_PLACE`.

Do not rerun full LOCAL_PLACE from the beginning for the current physical rod;
that would repeat already-completed arm placement/open-gripper/arm-retreat
actions. To finish the current rod only after chassis motion is allowed, resume
from:

```bash
source /home/agi/app/env.sh
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
python3 rack_hybrid_docking_package/industrial_cell_mission_controller.py \
  --checkpoint-file logs/industrial_cell_next_round_patch_live_checkpoint.json \
  --staging --confirm-live --execute-next \
  --local-action-mode full --confirm-local-physical \
  --allow-estop-pedal-fault \
  --start-at-local-step retreat_after_place \
  --local-retreat-m 0.45 --local-retreat-speed-mps 0.20 \
  --full-local-log-dir logs
```

The last attempt to run that resume slice was blocked before chassis motion by
`charge_plug_insert_state=1` with live charge input current around `15A`; no
chassis retreat or arm-home command was executed in that attempt.

Follow-up waist return correction:

- Operator clarified that after each placement the waist should not remain at
  the straight/high place waist pose. The current production flow returns it to
  the default HOME/grab-safe waist pose.
- Local and robot-side `python3 -m py_compile` passed after this patch.
- Robot-side backup before this patch:
  `rack_hybrid_docking_package/industrial_cell_mission_controller.py.bak_20260613_waist_grab_after_place`.
- Robot-side controller hash after this patch:
  `399b608302093d8c5af7cd4719ccd7fe70cc5938c3e698b56e992ff1137d68a5`.
- Robot-side no-motion dry-run with a temporary rod-7 `LOCAL_PLACE`
  checkpoint confirmed the final step:
  `waist_grab_after_place` using
  `rack_hybrid_docking_package/calibration_records/rod07_grab_pose_20260612_163717.json`.
- The already-completed live rod-7 run had left the waist at the place straight
  pose:
  `[-0.172711, 0.181927, -0.174479, -0.051860, 0.0]`.
- A manual correction was executed after mission completion with only the waist
  primitive:
  `move_waist_by_json_path.py --json rack_hybrid_docking_package/calibration_records/rod07_grab_pose_20260612_163717.json`.
- The manual correction completed with `move_waist_joint_result=0` and
  `final_max_error_rad=0.000383`.
- Waist readback after correction was:
  `[-0.650758, 2.053375, -1.456876, -0.018309, 0.038727]`, matching the rod-7
  grab waist pose.
- No chassis, arm, or gripper motion was sent during the manual waist
  correction.
- The active checkpoint remains complete:
  `rod_index=7`, `phase=MISSION_DONE`, `holding_rod=false`,
  `current_station=HOME_SAFE`, `last_success_step=ROD_DONE`.
- Final read-only status after the correction: `charge_plug_insert_state=0`,
  charge input `0V/0A`, `motion_control_error=0`, whole-body arm/end/waist/
  chassis errors all `0`, PNC task state `7`, odom available with
  `loc_confidence=80`, and stopped check `true`.

## 2026-06-13 Full Seven-Rod Optimized Run Completion

Production wrapper:

```bash
source /home/agi/app/env.sh
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
python3 rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py \
  --live --init --start-index 1 --end-index 7 \
  --checkpoint-file logs/industrial_cell_7_rods_optimized_live_checkpoint.json \
  --run-log logs/industrial_cell_7_rods_optimized_live.log
```

Result:

- All seven rods completed and the wrapper ended with `optimized_runner_done`.
- Final checkpoint:
  `rod_index=7`, `end_index=7`, `phase=MISSION_DONE`,
  `holding_rod=false`, `current_station=HOME_SAFE`,
  `last_success_step=ROD_DONE`.
- Final read-only readiness check returned `ok=true`,
  `charge_plug_insert_state=0`, charge input `0V/0A`,
  `motion_control_error=0`, map `19`, and final pose near HOME:
  `x=0.599219`, `y=-0.969695`.
- Combined run log:
  `/data/g2_industrial_cell_20260612/wxf/BOX_528_1/logs/industrial_cell_7_rods_optimized_live.log`.

Live issue found and fixed during the run:

- Rod 3 physically arrived at `HOME_SAFE`, but yaw refinement failed because
  the old tolerance was `1.0deg` and the station yaw residual was about
  `1.38deg`.
- `industrial_cell_7_rods_optimized.py` was patched to use
  `refine_yaw_tolerance_deg=1.5`.
- After syncing that wrapper to the robot, the run resumed from the checkpoint
  and rods 4 through 7 completed normally.

Measured performance from the successful run:

- `LOCAL_PLACE` is the main bottleneck, about `71-72s` per rod.
- `LOCAL_PICK` is about `38-40s` on rods 1-5 and about `48s` on rods 6-7.
- Map navigation is stable:
  `NAV_TO_GRAB` about `11-13s`, `NAV_TO_PLACE` about `16s`,
  `NAV_TO_RECOVERY` about `21s`, and `NAV_TO_HOME` about `12s`.
- Sensor-controlled fine positioning is consistent:
  grab fine position about `5.5s`, place fine position about `3.5s`.
- The 25cm end-effector pull-out after placing takes about `13s`; keep it
  conservative until the mechanical clearance is verified for faster stepping.
- Waist speed is already at the current script cap: `0.5rad/s`. Do not increase
  it by bypassing the cap without a separate joint-speed safety review.

Post-run code hygiene:

- The LOCAL_PLACE final waist label was renamed from the misleading historical
  `waist_grab_after_place` to `waist_home_after_place`. The old label remains a
  compatibility alias for `--start-at-local-step` and `--stop-after-local-step`.
- `start_at_local_step` and `stop_after_local_step` now fail fast on unknown
  labels instead of silently starting from the first local step.
- Added a no-motion log analyzer:

```bash
python3 rack_hybrid_docking_package/analyze_industrial_cell_run.py \
  logs/industrial_cell_7_rods_optimized_live.log
```

Next optimization boundary:

- Safe to tune next: reduce non-critical settle waits, add clearer post-run
  summaries, and evaluate whether a validated direct `PLACE_PRE -> HOME_SAFE`
  route can remove the recovery detour.
- Do not tune next without a dedicated single-step validation: end-effector
  pull-out stepping speed, final place offsets, grab final distance, or the
  `PLACE_PRE` recovery route.

## 2026-06-15 Local Continuation Audit for Direct Height Correction

This local continuation preserved the current state after the direct place
height correction work, but did not reach the robot. No robot motion command was
sent.

Local state:

- `rack_hybrid_docking_package/calibration_records/rod07_place_final_arm_up020_latest.json`
  exists locally.
- capture file:
  `rack_hybrid_docking_package/calibration_records/rod07_place_final_arm_up020_capture_20260615_112207.json`
- local hash for the final direct +2cm arm JSON:
  `e082dcb8cb9cc0a448e3c1c229780c4c52437ceb8819bc8c49220d4166283106`
- local `py_compile` passed for the optimized wrapper, mission controller,
  rack docking primitive, and direct place calibration helper.

Reachability:

- `ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=5 agi@10.20.15.60 hostname`
  returned `No route to host` both inside the normal tool sandbox and in the
  escalated direct SSH check.
- Robot-side status, file hashes, no-motion plan, and readiness were therefore
  not refreshed in this continuation.

Resume rule:

1. Do not choose a live resume point from this document alone.
2. Once `10.20.15.60` is reachable, list recent checkpoint files on the robot,
   run `industrial_map_nav_guarded.py --readiness-check`, then run
   `industrial_cell_7_rods_optimized.py --status-only --checkpoint-file ...`
   on the active checkpoint.
3. Only after status confirms the physical/log state should the operator choose
   whether the next live action is a fresh full-cycle start, a rod-6
   continuation, or a rod-local recovery slice.

## 2026-06-15 Map-20 Robot Recovery Check

Robot network recovered on `agi@192.168.0.7`; this is the active address for the
same industrial-cell workspace:

```bash
/data/g2_industrial_cell_20260612/wxf/BOX_528_1
```

Read-only checks:

- Robot-side direct +2cm place JSON and key scripts match the local hashes.
- `logs/industrial_cell_full_7_industrial_retry_checkpoint_20260615_103648.json`
  is complete:
  `rod_index=7`, `phase=MISSION_DONE`, `holding_rod=false`,
  `current_station=HOME_SAFE`, `last_success_step=ROD_DONE`.
- Robot-side compile check passed for the optimized wrapper, mission controller,
  rack docking primitive, and direct place calibration helper.
- Robot-side no-motion dry-run completed one rod with the direct +2cm place JSON
  and no extra relative pre-open Z raise step.

Configuration change:

- Current live map is `20`; operator confirmed navigation points are unchanged.
- `industrial_station_config.json` was changed from `map_id=19` to `map_id=20`.
- Robot backup before the change:
  `rack_hybrid_docking_package/industrial_station_config.json.bak_map19_to20_20260615_1418`
- New config hash:
  `c7b3cd42b039d93a75e8b182ec4c4c06aed4ddca1e4c58ece1136f786ff765a8`

Current readiness result:

- map mismatch is resolved.
- still blocked by charging:
  `charge_plug_insert_state=1`, charge input about `50.5V / 15.0A`.
- `motion_control_error=0`, `pnc_task_state=7`, and odom speed samples are
  stopped.

Next live step is gated only after the charge plug/current clears and a fresh
`industrial_map_nav_guarded.py --readiness-check` returns `ok=true`.

## 2026-06-15 Map-20 Live Attempt Result

After unplugging charge, readiness returned `ok=true` and a new map20 live run
was started with checkpoint:

```bash
logs/industrial_cell_full_7_map20_after_unplug_checkpoint_20260615_1421.json
```

Result:

- `NAV_TO_GRAB` succeeded and reached configured `GRAB_PRE`.
- `LOCAL_PICK` completed `open_gripper`, `waist_for_grab`, and
  `arm_grab_pose`.
- The run stopped at `grab_fine_position` before any gripper close or rod pickup:
  `no_front_ultrasonic_lock`, with no stable front radar history for IDs `(0,1)`.

Diagnosis:

- checkpoint after stop:
  `rod_index=1`, `phase=LOCAL_PICK`, `holding_rod=false`,
  `current_station=GRAB_PRE`, `last_success_step=NAV_TO_GRAB`.
- front ultrasonic IDs `0/1` were invalid or missing at the configured
  `GRAB_PRE` pose.
- rack/lidar diagnosis saw the target around `3.87-4.13m`, far outside the
  expected near-rack fine-positioning start.
- Therefore the map20 configured `GRAB_PRE` coordinate is not currently a valid
  pre-grab point for the rack workflow, despite navigation arriving at that
  coordinate.

Safety recovery:

- No rod was grabbed; grippers were not closed.
- Only upper-body recovery was sent:
  arms returned to `/data/wxf/wxf/positions/arm_default.json`, then waist/body
  returned to the same default.
- No chassis recovery motion was sent.
- Final readiness remained `ok=true`.

Next rule:

- Do not resume the stopped checkpoint into `LOCAL_PICK`.
- Revalidate or recapture map20 station points first, especially `GRAB_PRE`.
- The current physical pose is near the configured `GRAB_PRE`, but the upper body
  is back at default and the robot is not holding a rod.

## 2026-06-15 Map-20 Later Live Stop at Rod 5

The detailed end-of-day handoff is:

```bash
rack_hybrid_docking_package/20260615_map20_live_stop_handoff.md
```

Short version:

- Robot host is `agi@192.168.0.7`; remote workspace is
  `/data/g2_industrial_cell_20260612/wxf/BOX_528_1`.
- The later calibrated map20 workflow reached rod 5 placement and opened the
  grippers.
- The run log is
  `logs/live_rod5_to_7_place_retreat_tolerate_20260615_192856.log`.
- Rod 5 is physically placed, but the default checkpoint is stale:
  `rod_index=5`, `phase=LOCAL_PLACE`, `holding_rod=true`,
  `current_station=PLACE_PRE`.
- `retreat_after_place` stopped before chassis motion due to rear ultrasonic
  `rear_filtered_mm=203`.
- `arm_default_after_place` failed with GDK PLANNING timeout, and a standalone
  direct arm-default retry failed the same way.
- Final read-only status was otherwise clean:
  charge disconnected, `motion_control_error=0`, PNC state `7`, odom stopped,
  and whole-body errors all `0`.

Tomorrow: do not resume the stale checkpoint directly. Start with read-only
status, recover the upper body from the post-place pull-back posture, then
return waist/body to home/default. Once readiness is clean and the robot is at a
known safe station, start from rod 6 rather than rod 5 unless rod 5 is meant to
be redone.
