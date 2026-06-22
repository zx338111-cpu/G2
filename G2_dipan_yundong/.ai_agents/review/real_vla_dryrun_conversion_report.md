# Real VLA Dry-Run Conversion Report

Selected bundle stem: `rack_hybrid_docking_package/logs/industrial_7_rods_20260611_031537`
Output events: 1
Source JSONL records parsed: 1

## Source Files

- `rack_hybrid_docking_package/logs/industrial_7_rods_20260611_031537.jsonl`: 3712 bytes
- `rack_hybrid_docking_package/logs/industrial_7_rods_20260611_031537.log`: 1597 bytes
- `rack_hybrid_docking_package/logs/industrial_7_rods_20260611_031537_checkpoint.json`: 4067 bytes
- `rack_hybrid_docking_package/logs/industrial_7_rods_20260611_031537_report.json`: 8349 bytes

## Action Vector Handling

- No real VLA action vector was extracted from the selected bundle by the conservative scanner.
- The generated action vectors are `derived=true` zero-hold analysis placeholders.
- These placeholders are not VLA outputs, not robot commands, and must not be sent to hardware.
- They exist only to satisfy the current static JSONL validator while preserving real artifact provenance.
- Placeholder action events: 1

## Counts

- Action family `dual_delta_14d`: 1
- Critic category `action_invalid`: 1
- Critic category `controller_not_ready`: 1

## Extracted Signals

### task_phase
- pick
### success_label
- last_checkpoint.status=failed
- status=failed
### safety_or_blocker
- error={"message": "缺少必要脚本:\n/home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_open_2.py\n/home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_c
- error.message=缺少必要脚本: /home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_open_2.py /home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_close_2.py /home
- error.traceback=["Traceback (most recent call last):\n", " File \"/home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/industrial_7_rods_total_controller.py\", line 6964, in main\n controll
- error.type=FileNotFoundError
- event=step_failed
- status=failed
- last_checkpoint={"dry_run": true, "end_index": 1, "error": {"message": "缺少必要脚本:\n/home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_open_2.py\n/home/davie/G2/G2_dipan_yundong
- last_checkpoint.error={"message": "缺少必要脚本:\n/home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_open_2.py\n/home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_c
### robot_state
- Not found by generic static scan.
### localization_state
- error={"message": "缺少必要脚本:\n/home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_open_2.py\n/home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_c
- error.message=缺少必要脚本: /home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_open_2.py /home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_close_2.py /home
- last_checkpoint={"dry_run": true, "end_index": 1, "error": {"message": "缺少必要脚本:\n/home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_open_2.py\n/home/davie/G2/G2_dipan_yundong
- last_checkpoint.error={"message": "缺少必要脚本:\n/home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_open_2.py\n/home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_c
- last_checkpoint.error.message=缺少必要脚本: /home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_open_2.py /home/davie/G2/G2_dipan_yundong/rack_hybrid_docking_package/move_ee_pose_close_2.py /home
### action_vector
- Not found by generic static scan.

## Null, Unknown, or Placeholder Fields

- Camera image fields are `present=false` because the selected bundle is text/log/report only.
- End-effector pose and gripper fields remain `null` unless explicitly present in parsed records.
- Localization fields remain `null` unless explicit map/odom/pose signals are found.
- `frame_id` is `unknown` unless an action source explicitly declares a frame.
- Missing values are explicit and are not silently zero-filled.

## Safety Scope

This conversion is offline and analysis-only. It reads the selected text bundle and writes JSONL/report artifacts. It does not run robot motion scripts, ROS, drivers, controllers, datasets, checkpoints outside the selected bundle, rosbags, or secrets.
