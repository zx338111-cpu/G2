# VLA Contract and Metric Schema

Task: `task_0005_vla_contract_and_metric_schema`  
Date: 2026-06-22  
Role: AI-B Execution Engineer

## Scope

This document defines a documentation-only contract for future G2 VLA dry-run
work. It reconciles the current practical 8D left-arm absolute action path with
the guide-level 14D dual-arm delta action contract, and it defines measurable
dry-run metrics that can be computed from existing logs and artifacts only.

No robot motion, controller startup, ROS startup, dataset access, checkpoint
access, rosbag access, secret access, or source-code modification is part of this
contract.

## Contract Principles

1. VLA output is an action proposal, not a direct robot command.
2. Hard safety gates remain owned by the robot/controller layer.
3. Missing required observation fields must be explicit; they must not be filled
   with silent zero defaults.
4. Every dry-run event must carry enough timestamp and artifact information to
   reproduce metric calculations from logs after the fact.
5. The 8D and 14D action interfaces must remain distinguishable until an offline
   migration adapter is validated.

## VLA Observation Contract

Each observation is a structured event. Required means required for a given
dry-run evaluator stage, not required for live execution.

| Group | Field | Type | Stage | Description | Missing-data handling |
| --- | --- | --- | --- | --- | --- |
| identity | `schema_version` | string | required | Contract version, initially `vla_dryrun_v1`. | Reject event if absent. |
| identity | `run_id` | string | required | Stable identifier for one dry-run or log replay. | Reject event if absent. |
| identity | `event_id` | string | required | Unique event id inside the run. | Reject event if absent. |
| identity | `robot_id` | string | optional | Robot or host label when known. | Set `null`; do not infer. |
| time | `ts_wall` | string RFC3339 or log timestamp | required | Human-readable event time. | Reject event if absent. |
| time | `ts_monotonic_ns` | integer or null | recommended | Monotonic timestamp when available from runtime capture. | Set `null`; mark weaker sync. |
| time | `source_log_time` | string or null | recommended | Original timestamp from run log line or artifact. | Set `null`; keep event but mark lower traceability. |
| task | `task_prompt` | string | required for VLA proposal | Language prompt, for example pick/place/recover. | Block VLA proposal if absent. |
| task | `task_phase` | enum | required | One of `home`, `nav`, `pick`, `place`, `recover`, `hold`, `stop`, `unknown`. | Use `unknown`; critic may emit `task_phase_mismatch`. |
| task | `rod_index` | integer or null | optional | Rod index when a seven-rods run identifies it. | Set `null`. |
| task | `expected_action_family` | enum | required | `left_abs_8d`, `dual_delta_14d`, or `none`. | Block action validation if absent. |
| camera | `cameras.head_image.present` | boolean | required for image VLA | Whether head image frame is available. | If false, emit `observation_missing`. |
| camera | `cameras.head_image.uri` | string or null | optional | Artifact path or opaque reference, not raw image data. | Set `null`; keep metadata. |
| camera | `cameras.head_image.ts` | string or null | recommended | Frame timestamp. | Mark frame age unknown. |
| camera | `cameras.head_image.age_ms` | number or null | recommended | Age relative to event time. | Mark sync unknown. |
| camera | `cameras.head_image.width` | integer or null | optional | Image width when known. | Set `null`. |
| camera | `cameras.head_image.height` | integer or null | optional | Image height when known. | Set `null`. |
| camera | `cameras.head_image.encoding` | string or null | optional | Encoding such as `rgb8`, `bgr8`, `jpeg`. | Set `unknown`. |
| camera | `cameras.left_wrist_image.*` | same as head | optional now, required later | Left wrist image metadata. | If required stage needs it, emit `observation_missing`. |
| camera | `cameras.right_wrist_image.*` | same as head | optional now, required later | Right wrist image metadata. | If required stage needs it, emit `observation_missing`. |
| media | `media.head_fps` | number or null | recommended | Viewer/log-derived head camera frame rate. | Set `null`; exclude from fps denominator if unknown. |
| media | `media.audio_packet_age_ms` | number or null | optional | Audio freshness when relevant. | Set `null`. |
| state | `state.left_ee_pose_xyz_quat` | array[7] or null | required for 8D | `[x,y,z,qx,qy,qz,qw]` left end-effector pose. | Block 8D-to-delta derivation if absent. |
| state | `state.left_gripper_01` | number or null | required for 8D | Normalized left gripper state. | Mark action context incomplete. |
| state | `state.right_ee_pose_xyz_quat` | array[7] or null | required for 14D | Right end-effector pose. | Block dual-arm validation if absent. |
| state | `state.right_gripper_01` | number or null | required for 14D | Normalized right gripper state. | Block dual-arm validation if absent. |
| state | `state.motion_control_error_code` | integer or null | required | Motion-control health code from logs/status snapshots. | Emit `controller_not_ready` if missing for action stage. |
| state | `state.joint_errors` | array[string] | required | Joint error labels or ids. | Empty means no known errors; missing means unknown. |
| state | `state.pnc_task_state` | object or null | recommended | Chassis navigation task id/state/type/message. | Set `null`; chassis metrics marked unavailable. |
| state | `state.chassis_power_state` | object or null | recommended | Charge plug, estop, power states when logged. | Missing can block chassis-related plans. |
| localization | `localization.map_id` | string or integer or null | recommended | Active map id when known. | Set `null`; localization metrics unknown. |
| localization | `localization.curr_pose_valid` | boolean or null | recommended | Whether SLAM current pose exists. | If false for nav plan, emit `localization_unavailable`. |
| localization | `localization.odom_valid` | boolean or null | recommended | Whether odom exists. | If false for nav plan, emit `localization_unavailable`. |
| localization | `localization.map_to_base_valid` | boolean or null | optional | Whether map to base transform is available. | Set `null`; do not infer. |
| artifact | `artifact_refs` | array[object] | recommended | Log/report/image references used to build event. | Empty array allowed, but traceability metric decreases. |

## Missing-Data Rules

- Required identity/time fields are hard rejects.
- Required observation fields for a chosen VLA stage are safety rejects, not
  defaulted values.
- Optional fields should use `null`, `unknown`, or `present=false` explicitly.
- Derived fields must include a `derived_from` reference to the source event or
  artifact.
- A stale field is treated as missing for action validation if its age exceeds
  the configured threshold for that stage.

## Action Contract Comparison

| Aspect | Current practical 8D left-arm absolute path | Guide-level 14D dual-arm delta path |
| --- | --- | --- |
| Contract id | `left_abs_8d` | `dual_delta_14d` |
| Vector shape | 8 values | 14 values |
| Arms | Left arm only | Left and right arms |
| Pose semantics | Absolute left EE target | Delta motion command per arm |
| Orientation semantics | Quaternion `qx,qy,qz,qw` | Roll/pitch/yaw deltas |
| Gripper semantics | One normalized left gripper value | Left and right gripper values |
| Typical vector | `[x,y,z,qx,qy,qz,qw,grip_01]` | `[l_dx,l_dy,l_dz,l_droll,l_dpitch,l_dyaw,l_grip,r_dx,r_dy,r_dz,r_droll,r_dpitch,r_dyaw,r_grip]` |
| Primary value now | Matches the documented near-term practical path | Matches the fuller guide-level target |
| Main risk | Absolute target can be unsafe if frame or state context is stale | Delta target can accumulate error and needs both-arm readiness |
| Dry-run validation | Shape, finite values, frame, absolute bounds, left state availability | Shape, finite values, per-step limits, arm masks, dual state availability |

The two interfaces must not be treated as interchangeable. A dry-run event must
state the `action_family`, `frame_id`, `arm_mask`, `is_absolute`, and
`is_delta`. If an offline adapter derives a 14D-like delta from an 8D absolute
target, that event must be marked `derived=true` and must preserve the original
8D vector.

## Canonical Action Proposal Envelope

Every proposed action should be wrapped in this envelope before any dry-run
critic or metric calculation:

| Field | Type | Required | Description |
| --- | --- | --- | --- |
| `action_id` | string | yes | Unique id for the proposal. |
| `source_event_id` | string | yes | Observation event used by the proposal. |
| `action_family` | enum | yes | `left_abs_8d` or `dual_delta_14d`. |
| `vector` | array[number] | yes | Raw model or fixture output. |
| `shape` | integer | yes | Length of `vector`. |
| `frame_id` | string | yes | Coordinate frame or `unknown`. |
| `arm_mask` | array[string] | yes | `["left"]` or `["left","right"]`. |
| `is_absolute` | boolean | yes | True for 8D absolute path. |
| `is_delta` | boolean | yes | True for 14D delta path. |
| `limits_profile` | string | yes | Name of dry-run limit set. |
| `sanitizer` | object | yes | Shape, finite, bound, and clipping results. |
| `derived` | boolean | yes | Whether produced by adapter instead of model. |
| `derived_from_action_id` | string or null | yes | Source action id if derived. |

## Safe Migration Path: 8D Dry-Run to 14D Dual-Arm Validation

1. Schema-only validation:
   - Validate JSONL events and action envelopes from synthetic or existing log
     artifacts.
   - No robot process, ROS process, controller, dataset, checkpoint, or rosbag is
     accessed.

2. 8D left-arm dry-run validation:
   - Accept only `left_abs_8d`.
   - Require left EE pose and left gripper state if an absolute-to-delta
     comparison is attempted.
   - Measure shape, finite values, frame presence, task-phase alignment, and
     safety-gate reject reasons.

3. Offline 8D-to-14D bridge analysis:
   - Preserve the original 8D vector.
   - Derive a left-arm delta only when current left pose is available and fresh.
   - Set right-arm delta to hold/no-op only as an analysis label, not as a robot
     command.
   - Emit `action_invalid` if quaternion/frame/state are not sufficient for a
     defensible conversion.

4. 14D dual-arm dry-run validation:
   - Accept only `dual_delta_14d`.
   - Require both left and right state fields.
   - Validate per-step limits and gripper bounds independently per arm.
   - Track right-arm no-op rates so a nominal dual-arm policy does not silently
     remain left-arm-only.

5. Future live-readiness review:
   - A separate task must review controller readiness, safety gates, and human
     approval before any live execution is considered.
   - Passing this document's metrics is necessary but not sufficient for live
     robot motion.

## World Model Fact Table

The world model is a runtime fact table for dry-run context and evaluation. It
does not replace HAL, PNC, SLAM, GDK, or hard safety gates.

| Fact group | Example fields | Freshness target | Confidence | Blocks action when |
| --- | --- | --- | --- | --- |
| `robot_state` | EE poses, gripper states, joint errors, motion-control error code, whole-body status | Event-level for action validation | From status/log source | Controller state missing, stale, or erroring. |
| `perception_state` | Head image present, wrist image present, object labels, object confidence, frame age | Image-level | Per detector or artifact | Required image/object evidence missing. |
| `task_state` | Task phase, prompt, rod index, completed rods, expected action family, success label | Phase-level | From controller log or annotation | Proposed action does not match task phase. |
| `safety_state` | Safety gate result, charge plug, estop, localization available, PNC idle/running, blocker reasons | Event-level | From robot status/log source | Any hard gate reports blocked. |
| `memory_state` | Last known object, last successful phase, recent blocker, retry count, stale-plan age | Run-level with explicit age | Derived from event history | Plan is stale or retry limit exceeded. |
| `media_state` | Head fps, reconnect count, audio packet age, tunnel/viewer status | Viewer/log-level | From media logs | Required media signal is absent or too stale. |

Each fact row should use:

```json
{
  "fact_group": "robot_state",
  "fact_name": "motion_control_error_code",
  "value": 0,
  "confidence": 1.0,
  "ts_wall": "2026-06-22T00:00:00Z",
  "source_event_id": "event_000001",
  "source_artifact": "logs/example.log",
  "stale_after_ms": 500,
  "required_for": ["action_validation"],
  "missing_reason": null
}
```

## Critic and Retry Failure Taxonomy

| Category | Trigger | Retry or stop behavior |
| --- | --- | --- |
| `observation_missing` | Required camera, state, prompt, timestamp, or artifact metadata is absent or stale. | Do not generate a new action from the incomplete event; request a fresh observation in dry-run labels. |
| `action_invalid` | Shape mismatch, non-finite value, gripper out of range, frame missing, excessive delta, invalid quaternion, or unsupported action family. | Reject action; record sanitizer details; do not adapt silently. |
| `safety_gate_blocked` | Charge plug, estop, controller fault, joint error, PNC conflict, or configured hard gate blocks execution. | Stop action progression; keep blocker reason as primary failure. |
| `task_phase_mismatch` | Proposed action family or intent does not match `task_phase`, prompt, rod index, or expected transition. | Reject and require planner/task-state correction. |
| `stale_plan` | Plan/action was generated from old observation, old memory, or old task state beyond threshold. | Replan from a fresh observation; increment stale-plan metric. |
| `contact_or_grasp_uncertain` | Gripper/contact/holding state is missing, contradictory, or below confidence after pick/place. | Mark phase uncertain; require recovery/verification label before next phase. |
| `localization_unavailable` | `curr_pose`, `odom`, map id, or map-to-base transform is unavailable for nav/chassis-related context. | Block navigation-related plan evaluation; keep arm-only dry-run separate. |
| `controller_not_ready` | Motion-control status, PNC state, or required robot health status is missing, stale, or erroring. | Reject action; classify as readiness failure, not model failure. |

## Dry-Run Metrics From Existing Logs and Artifacts

All metrics below are computed from existing logs, status snapshots, reports, or
JSONL dry-run artifacts. None require live robot execution.

| Metric | Definition | Source class |
| --- | --- | --- |
| `event_schema_valid_rate` | Valid JSONL events divided by total parsed events. | Future JSONL dry-run artifact. |
| `observation_required_complete_rate` | Events with all stage-required observation fields divided by total stage events. | JSONL/log-derived observations. |
| `head_image_available_rate` | Events where `cameras.head_image.present=true` divided by image-required events. | Viewer logs or dry-run JSONL. |
| `image_state_sync_p95_ms` | 95th percentile absolute age difference between image timestamp and state timestamp. | Dry-run JSONL with timestamps. |
| `media_fps_min` | Minimum logged head camera fps during run. | Viewer/media logs. |
| `action_shape_valid_rate` | Actions with expected vector length divided by total actions for that family. | Dry-run action envelopes. |
| `action_finite_valid_rate` | Actions with no NaN/Inf divided by total actions. | Dry-run action envelopes. |
| `action_clip_count` | Count of fields clipped by dry-run sanitizer. | Sanitizer output. |
| `action_family_mismatch_count` | Count where action family differs from expected task family. | Action envelope plus task state. |
| `safety_gate_reject_count_by_reason` | Rejections grouped by hard-gate reason. | Safety gate output and logs. |
| `critic_failure_count_by_category` | Count grouped by the eight taxonomy categories. | Critic output. |
| `controller_ready_rate` | Events with healthy motion-control and required controller state divided by action-stage events. | Status snapshots/logs. |
| `localization_available_rate` | Events with required localization fields available divided by nav-context events. | SLAM/PNC/status logs. |
| `program_error_count` | Traceback, Exception, KeyboardInterrupt, or nonzero return markers in logs. | Run logs. |
| `task_success_label` | Boolean labels such as `MISSION_DONE`, `HOME_SAFE`, and `holding_rod=false` when present. | Run reports/log summaries. |
| `phase_duration_seconds` | Duration per known phase, such as `LOCAL_PICK`, `LOCAL_PLACE`, `NAV_TO_PLACE`, `NAV_TO_RECOVERY`, `NAV_TO_HOME`. | Existing run logs/reports. |
| `recovery_duplicate_action_count` | Duplicate unsafe gripper open/regrab or repeated phase action during recovery. | Run logs and phase labels. |
| `traceability_rate` | Events with at least one usable `artifact_ref` divided by total events. | JSONL dry-run artifact. |

Initial phase timing baselines from prior review can be used as non-binding
engineering references: `LOCAL_PLACE` about 62.268s, `LOCAL_PICK` about 35.752s,
`NAV_TO_PLACE` about 16.989s, `NAV_TO_RECOVERY` about 16.426s, and
`NAV_TO_HOME` about 11.629s.

## Minimum Viable JSONL Event Schema

Each line is one JSON object. The schema is intentionally minimal so future
tasks can validate it with static tools before any model or robot process is
introduced.

Required top-level fields:

- `schema_version`
- `run_id`
- `event_id`
- `event_type`
- `ts_wall`
- `task`
- `observation`
- `world_model`
- `action_proposal`
- `safety_gate`
- `critic`
- `metrics`
- `artifact_refs`

Example line:

```json
{"schema_version":"vla_dryrun_v1","run_id":"dryrun_0001","event_id":"event_000001","event_type":"action_proposal","ts_wall":"2026-06-22T00:00:00Z","ts_monotonic_ns":null,"task":{"task_prompt":"pick aluminum profile","task_phase":"pick","rod_index":1,"expected_action_family":"left_abs_8d"},"observation":{"cameras":{"head_image":{"present":true,"uri":"logs/frame_000001.jpg","ts":"2026-06-22T00:00:00Z","age_ms":42,"width":1280,"height":720,"encoding":"jpeg"},"left_wrist_image":{"present":false,"uri":null,"missing_reason":"not_required_for_stage"},"right_wrist_image":{"present":false,"uri":null,"missing_reason":"not_required_for_stage"}},"state":{"left_ee_pose_xyz_quat":[0.1,0.2,0.3,0.0,0.0,0.0,1.0],"left_gripper_01":1.0,"right_ee_pose_xyz_quat":null,"right_gripper_01":null,"motion_control_error_code":0,"joint_errors":[],"pnc_task_state":null,"chassis_power_state":null},"localization":{"map_id":null,"curr_pose_valid":null,"odom_valid":null,"map_to_base_valid":null}},"world_model":{"facts":[{"fact_group":"robot_state","fact_name":"motion_control_error_code","value":0,"confidence":1.0,"stale_after_ms":500,"missing_reason":null}]},"action_proposal":{"action_id":"action_000001","source_event_id":"event_000001","action_family":"left_abs_8d","vector":[0.1,0.2,0.3,0.0,0.0,0.0,1.0,1.0],"shape":8,"frame_id":"base_link","arm_mask":["left"],"is_absolute":true,"is_delta":false,"limits_profile":"dryrun_left_abs_v1","sanitizer":{"shape_valid":true,"finite_valid":true,"bounds_valid":true,"clip_count":0},"derived":false,"derived_from_action_id":null},"safety_gate":{"allowed":true,"blocked_reasons":[]},"critic":{"ok":true,"failure_categories":[]},"metrics":{"event_schema_valid":true,"observation_required_complete":true,"action_shape_valid":true,"action_finite_valid":true},"artifact_refs":[{"kind":"log","uri":"logs/example.log","description":"dry-run source"}]}
```

## Acceptance Criteria for Future task_0006

Recommended task:

`task_0006_vla_dryrun_jsonl_fixture_and_static_validator`

Low-risk objective:

Create a small documentation/sample-only JSONL fixture and a static validation
report for this contract, without reading datasets, checkpoints, rosbags,
secrets, or robot-motion source files, and without starting robot software.

Acceptance criteria:

1. A sample JSONL file contains at least one `left_abs_8d` event, one
   `dual_delta_14d` event, and one rejected event for each critic taxonomy
   category.
2. A static schema or checklist validates required fields, missing-data markers,
   action shapes, finite values, and taxonomy names.
3. The result report lists metric values for the sample fixture, including
   schema validity, observation completeness, action validity, and critic failure
   counts.
4. The task remains documentation/sample-only and modifies only explicitly
   allowed `.ai_agents` files.
5. No robot motion script, ROS driver, controller, dataset, checkpoint, rosbag,
   secret, or credential file is read or executed.
6. The task does not attempt to solve live robot execution; it only proves that
   future dry-run VLA events can be measured consistently.
