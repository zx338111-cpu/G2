# VLA / World Model Documentation Review

Task: `task_0002_vla_worldmodel_doc_review`
Date: 2026-06-22
Role: AI-B Execution Engineer

## Scope

This review is based only on these four root-level documents:

- `G2_GDK_SECONDARY_DEVELOPMENT_VLA_WORLDMODEL_GUIDE.md`
- `G2_LIVE_ARCHITECTURE_DEEP_DIVE_20260618.md`
- `G2_WEEKLY_REPORT_20260618.md`
- `G2_WEEKLY_REPORT_BRIEF_20260618.md`

No source code, robot motion script, ROS driver, controller, dataset, checkpoint, rosbag, secret, or credential file was read or executed.

## Current Objective

The project objective is to turn the current Agibot G2 into a safe second-development platform where GDK-based robot IO, camera/audio sensing, guarded execution, VLA policy inference, and a structured world model can be connected without letting learned policy output bypass robot safety.

The immediate practical direction is not full autonomous deployment. The documents point to a staged path:

1. Stabilize read-only observation and runtime state capture.
2. Define a consistent VLA observation/action contract.
3. Run VLA dry-run and safety-gate validation.
4. Only later consider small, controlled live execution.

The live chassis path is currently blocked by `charge_plug_insert_state=1`, missing SLAM `curr_pose/odom`, and incomplete `map -> base_link` localization. The arm/head upper-body state is described as healthier, but any motion still needs a fresh preflight and explicit approval.

## Data Flow

### Camera

The VLA observation path is centered on robot-mounted images:

- `observation/head_image`
- `observation/left_wrist_image`
- `observation/right_wrist_image`

The architecture document also lists head color, head depth, stereo, fisheye, hand cameras, lidar, IMU, TF, joint, gripper, power, and audio/tunnel status as available or relevant sensor channels.

For the current viewer path, the head camera is the proven practical signal. The weekly report says the head video chain is usable through a custom viewer and SSH tunnel; official RTC is not the preferred short-term path.

### Logs

The documents identify several log or artifact classes:

- robot runtime logs under `/data/logs/latest`
- status snapshots from GDK objects
- viewer status including camera fps and audio packet freshness
- task/run logs from map20 seven-rods execution
- weekly phase timing and final task status summaries

These logs are important because the project is safety-gated: logs must prove status, timing, blockers, failures, and recovery behavior before any learned policy can be trusted.

### State

The recommended VLA state contract in the GDK/VLA guide is 16D:

- left end-effector position, 3D
- left end-effector quaternion, 4D
- left gripper, 1D
- right end-effector position, 3D
- right end-effector quaternion, 4D
- right gripper, 1D

The broader runtime state includes:

- whole-body status
- joint states and joint errors
- motion-control error code
- end-effector/gripper state
- chassis power state
- PNC task state
- SLAM state, current pose, odom
- map id
- TF availability
- safety blockers

The weekly report also says the current practical VLA path is closer to an 8D left-arm absolute action interface: `[x, y, z, qx, qy, qz, qw, grip_01]`. This differs from the 16D state / 14D action dual-arm contract in the guide and should be resolved before implementation.

### Action

The guide recommends a 14D action contract:

- left `dx, dy, dz`
- left `droll, dpitch, dyaw`
- left gripper
- right `dx, dy, dz`
- right `droll, dpitch, dyaw`
- right gripper

The action must pass shape, dtype, finite-value, per-step limit, and safety-gate checks before execution. VLA output is treated as an action suggestion, not as a direct robot command.

The weekly report's 8D left-arm action path should be treated as a current engineering reality or near-term simplified target, while the 14D dual-arm path is a fuller target contract.

### Task

Task context appears in three forms:

- language prompt for VLA, such as picking or placing an aluminum profile
- structured task phase, such as pick/place/recovery/home
- execution state, such as `MISSION_DONE`, `HOME_SAFE`, and `holding_rod=false`

The world model should preserve task state so planner prompts, action checks, recovery behavior, and evaluation labels stay aligned.

## Model Flow

### VLA

The VLA consumes images, robot state, and a language/task prompt, then emits action chunks. The guide recommends keeping the trained observation/action contract stable and not pushing arbitrary world-model context into the policy unless the policy is explicitly trained for it.

### World Model

The world model is defined as a runtime fact table, not as the safety controller or a replacement for HAL/PNC/SLAM. Its responsibilities are:

- fuse robot, localization, perception, task, and safety state
- maintain scene object and short-term memory
- summarize context for planner/VLA use
- record visualization/debug logs
- decide whether safety state allows the next action to be considered

It should update robot/safety state quickly, image/object state at a moderate cadence, and SLAM/map/TF state more slowly.

### Critic

The documents do not define a concrete implemented critic module. The nearest documented role is risk assessment and policy replay evaluation in the world-model plan.

For task planning, a practical critic should be treated as an offline or dry-run evaluator first:

- check whether observations are complete and synchronized
- check action shape/range/NaN violations
- compare predicted action intent with task phase
- classify safety-gate rejection reasons
- measure whether logged execution reached success states

This keeps the critic documentation/analysis-only until the data and log schema are stable.

### Memory

Memory appears as `short_term_memory` in the world model and as task/scene object history. It should remember:

- recently seen objects and confidence
- task phase and completed rods/steps
- safety blockers
- media/control latency state
- recovery context

Memory should support stable prompts and recovery decisions, but it should not override hard safety gates.

### Planner

The planner chooses task phase and prompt/context. It should decide whether the VLA is being asked to pick, place, hold, recover, or stop. The documents explicitly separate planner/context from the VLA observation: the planner can use a world-model summary to choose prompts and determine whether actions are allowed.

### Controller

The controller layer is GDK-facing and safety-gated:

- `Robot` for upper-body, joint, gripper, head, and motion-control status
- `Pnc` for chassis control and navigation task state
- `Slam`, `Map`, and `TF` for localization dependencies
- action sanitizer before any arm execution
- watchdog and heartbeat for any future chassis teleop path

The controller must own hardware safety boundaries. VLA and world model can inform decisions, but they do not replace preflight, limit checks, watchdogs, or emergency stop logic.

## Candidate Evaluation Metrics

Metrics realistically measurable from the described documents/logs:

- task success: `MISSION_DONE`, `HOME_SAFE`, `holding_rod=false`
- safety health: `motion_control_error=0`, whole-body errors 0, joint errors absent
- program health: no Traceback, Exception, KeyboardInterrupt, or nonzero return code in run logs
- phase timing: `LOCAL_PLACE`, `LOCAL_PICK`, `NAV_TO_PLACE`, `NAV_TO_RECOVERY`, `NAV_TO_HOME`
- recovery correctness: no unsafe duplicate gripper open/regrab during interrupted recovery
- readiness blockers: `charge_plug_insert_state`, pose availability, odom availability, PNC idle state
- media quality: head camera fps, audio packet age, reconnect count, weak-network behavior
- observation quality: image availability, missing frame rate, timestamp alignment between image/state/action
- action validity: shape mismatch count, NaN/Inf count, per-step clipping count, limit violation count
- safety-gate behavior: reject reason counts and whether execution stops/holds on blocker
- VLA dry-run alignment: whether predicted action type matches task phase and stays inside the allowed action contract

The weekly report contains concrete timing examples:

- `LOCAL_PLACE` average about 62.268 seconds
- `LOCAL_PICK` average about 35.752 seconds
- `NAV_TO_PLACE` average about 16.989 seconds
- `NAV_TO_RECOVERY` average about 16.426 seconds
- `NAV_TO_HOME` average about 11.629 seconds

These are good initial baselines for engineering evaluation, even before any model is connected.

## Key Gaps

1. The VLA action contract is not fully reconciled: guide-level 14D dual-arm delta action versus weekly-report 8D left-arm absolute action.
2. The critic is conceptual, not an implementation-ready component yet.
3. Existing log schemas are described indirectly; a dedicated schema map is needed before reliable automated metrics.
4. Chassis autonomy is not ready because charge-plug and SLAM/odom gates are not satisfied.
5. Official RTC is not the stable media path; custom viewer plus SSH tunnel is the current practical path.

## Recommended task_0003

Proposed task:

`task_0003_vla_contract_and_metric_schema`

One-line rationale:

Create a documentation-only schema that reconciles the 8D left-arm and 14D dual-arm action contracts, maps existing logs/artifacts to observation/state/action/task fields, and defines measurable dry-run metrics before any code, dataset, checkpoint, or robot execution work.

Suggested scope:

- Read only the four reviewed docs plus task_0001/task_0002 outputs.
- Do not read datasets, checkpoints, rosbags, secrets, or source code.
- Produce a contract table for observation, state, action, task, safety gate, and metric fields.
- Mark each field as already evidenced, inferred, missing, or requires future safe inspection.
- Keep the next step documentation/analysis-only.
