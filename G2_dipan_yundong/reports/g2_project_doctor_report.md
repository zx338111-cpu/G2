# G2 项目状态静态诊断报告

- 生成时间: 2026-06-22T17:59:42-07:00
- 声明: 本报告为静态分析, 未运行任何机器人脚本
- 仓库根目录: `/home/davie/G2/G2_dipan_yundong`

## 1. 项目结构概览

- ✅ 存在 `logs/`
- ✅ 存在 `tools/`
- ✅ 存在 `reports/`
- ✅ 存在 `overlays/`
- ✅ 存在 `rack_hybrid_docking_package/`
- ✅ 存在 `handoff/`
- ✅ 存在 `.ai_agents/`
- ✅ 存在 `AGENTS.md`
- ✅ 存在 `CLAUDE.md`
- ✅ 存在 `G2_GDK_SECONDARY_DEVELOPMENT_VLA_WORLDMODEL_GUIDE.md`

## 2. 日志检查 (logs/)

- ✅ 存在 `logs/`
- `*.log`: count=25, total_size=40.9 KiB, latest_mtime=2026-06-15T17:40:23-07:00
- `*.jsonl`: count=10, total_size=38.1 KiB, latest_mtime=2026-06-11T17:37:51-07:00
- `*.json`: count=25, total_size=170.1 KiB, latest_mtime=2026-06-15T17:40:23-07:00
- ⚠️ 零字节 / 空文件:
- 无

## 3. Viewer 文件检查

- `g2_head_av_viewer.py`: ✅ 存在, size=43.4 KiB, ✅ 语法 OK
- `g2_head_tunnel_viewer.py`: ✅ 存在, size=45.2 KiB, ✅ 语法 OK
- `g2_head_webrtc_viewer.py`: ✅ 存在, size=27.7 KiB, ✅ 语法 OK

## 4. VLA / World Model 文件检查

- 文件名含 `vla` 或 `world` 的文件数量: 22
- `.ai_agents/fixtures/real_vla_dryrun_industrial_7_rods_20260611_031537.jsonl` (4.0 KiB)
- `.ai_agents/fixtures/vla_dryrun_negative_sample.jsonl` (10.0 KiB)
- `.ai_agents/fixtures/vla_dryrun_sample.jsonl` (9.2 KiB)
- `.ai_agents/review/real_vla_dryrun_conversion_report.md` (4.2 KiB)
- `.ai_agents/review/real_vla_dryrun_validation_report.md` (544 B)
- `.ai_agents/review/vla_contract_and_metric_schema.md` (20.5 KiB)
- `.ai_agents/review/vla_dryrun_negative_validation_report.md` (938 B)
- `.ai_agents/review/vla_dryrun_validation_report.md` (535 B)
- `.ai_agents/review/vla_worldmodel_doc_review.md` (9.6 KiB)
- `.ai_agents/scripts/convert_real_log_bundle_to_vla_jsonl.py` (20.6 KiB)
- `.ai_agents/scripts/validate_vla_dryrun_jsonl.py` (11.7 KiB)
- `.ai_agents/tasks/inbox/task_0002_vla_worldmodel_doc_review.json` (4.8 KiB)
- `.ai_agents/tasks/inbox/task_0005_vla_contract_and_metric_schema.json` (5.3 KiB)
- `.ai_agents/tasks/inbox/task_0006_vla_dryrun_jsonl_fixture_and_static_validator.json` (5.7 KiB)
- `.ai_agents/tasks/inbox/task_0007_vla_negative_fixture_validator.json` (5.1 KiB)
- `.ai_agents/tasks/inbox/task_0009_convert_one_dryrun_bundle_to_vla_jsonl.json` (7.1 KiB)
- `.ai_agents/tasks/results/result_0002_vla_worldmodel_doc_review.json` (2.7 KiB)
- `.ai_agents/tasks/results/result_0005_vla_contract_and_metric_schema.json` (2.5 KiB)
- `.ai_agents/tasks/results/result_0006_vla_dryrun_jsonl_fixture_and_static_validator.json` (3.0 KiB)
- `.ai_agents/tasks/results/result_0007_vla_negative_fixture_validator.json` (3.6 KiB)
- `.ai_agents/tasks/results/result_0009_convert_one_dryrun_bundle_to_vla_jsonl.json` (4.6 KiB)
- `G2_GDK_SECONDARY_DEVELOPMENT_VLA_WORLDMODEL_GUIDE.md` (32.6 KiB)
- `.ai_agents/fixtures/` 下 `*.jsonl` 夹具数量: 3
- `.ai_agents/fixtures/real_vla_dryrun_industrial_7_rods_20260611_031537.jsonl` (4.0 KiB)
- `.ai_agents/fixtures/vla_dryrun_negative_sample.jsonl` (10.0 KiB)
- `.ai_agents/fixtures/vla_dryrun_sample.jsonl` (9.2 KiB)

## 5. 机器人运动脚本入口 (只列出, 禁止运行)

**醒目提示: 以下脚本禁止由本工具或 Codex 运行；本章节只做源码文本清点。**
- `industrial_7_rods_one_click.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `industrial_cancel_pnc_task.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `industrial_lidar_snapshot.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `industrial_linear_y_diagnostic.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `industrial_linear_y_sweep.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `industrial_status_snapshot.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `industrial_turn_diagnostic.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `move_arm_by_json_path.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `move_arm_vertical_stack_grab_above.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `move_ee_pose_close_2.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `move_ee_pose_open_2.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `move_ee_relative_offset.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `move_waist_by_json_path.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `offset_move_down.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `rack_hybrid_docking_package/rack_hybrid_docking.py`: ⚠️ no __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `rack_hybrid_docking_package/rack_hybrid_docking_demo.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `rack_hybrid_docking_package/rack_industrial_docking.py`: ⚠️ no __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `rack_hybrid_docking_package/rack_lidar_docking.py`: ⚠️ no __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `rack_hybrid_docking_package/rack_radar_docking.py`: ⚠️ no __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `rack_hybrid_docking_package/rack_retreat_controller.py`: ⚠️ no __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `rack_hybrid_docking_package/run_map20_7_rods_live.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN
- `rack_hybrid_docking_package/run_site_7_rods_live.py`: ✅ has __main__ guard | MOTION ENTRYPOINT — DO NOT RUN

## 6. 安全相关文件 (受保护, 只读)

- `industrial_7_rods_one_click.py`: keywords=velocity, matched_lines=3 | PROTECTED — DO NOT MODIFY
- `industrial_linear_y_diagnostic.py`: keywords=estop, velocity, matched_lines=14 | PROTECTED — DO NOT MODIFY
- `industrial_linear_y_sweep.py`: keywords=estop, safety, matched_lines=4 | PROTECTED — DO NOT MODIFY
- `industrial_status_snapshot.py`: keywords=velocity, matched_lines=8 | PROTECTED — DO NOT MODIFY
- `industrial_turn_diagnostic.py`: keywords=estop, velocity, matched_lines=22 | PROTECTED — DO NOT MODIFY
- `.ai_agents/scripts/convert_real_log_bundle_to_vla_jsonl.py`: keywords=estop, limit, safety, matched_lines=11 | PROTECTED — DO NOT MODIFY
- `.ai_agents/scripts/validate_vla_dryrun_jsonl.py`: keywords=safety, matched_lines=8 | PROTECTED — DO NOT MODIFY
- `agi@192.168.168.45/rack_hybrid_docking.py`: keywords=e_stop, estop, safety, velocity, matched_lines=17 | PROTECTED — DO NOT MODIFY
- `agi@192.168.168.45/rack_hybrid_docking_demo.py`: keywords=e_stop, estop, matched_lines=5 | PROTECTED — DO NOT MODIFY
- `agi@192.168.168.45/rack_lidar_docking.py`: keywords=estop, safety, velocity, matched_lines=25 | PROTECTED — DO NOT MODIFY
- `agi@192.168.168.45/rack_radar_docking.py`: keywords=estop, safety, velocity, matched_lines=14 | PROTECTED — DO NOT MODIFY
- `overlays/BOX_528_1_map7_safe_20260622_1340/robot_controller.py`: keywords=estop, matched_lines=2 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/analyze_rack_pose_events.py`: keywords=limit, matched_lines=7 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/capture_grab_calibration_point.py`: keywords=velocity, matched_lines=6 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/create_site_profile.py`: keywords=safety, matched_lines=4 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/demo_chassis_motion_controller.py`: keywords=e_stop, estop, matched_lines=6 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/guarded_front_target_recovery.py`: keywords=estop, velocity, matched_lines=4 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/industrial_7_rods_total_controller.py`: keywords=e_stop, estop, safety, velocity, matched_lines=115 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/industrial_cell_7_rods_optimized.py`: keywords=estop, limit, matched_lines=3 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py`: keywords=e_stop, estop, limit, safety, matched_lines=13 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/industrial_cell_mission_controller.py`: keywords=estop, limit, safety, matched_lines=23 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/industrial_docking_integration_template.py`: keywords=e_stop, estop, matched_lines=8 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/industrial_lateral_centering_probe.py`: keywords=estop, safety, matched_lines=2 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/industrial_lateral_motion_trace.py`: keywords=estop, matched_lines=8 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/industrial_map_nav_guarded.py`: keywords=limit, safety, velocity, matched_lines=33 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/industrial_retreat_1m_validation.py`: keywords=estop, safety, matched_lines=7 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/rack_hybrid_docking.py`: keywords=e_stop, estop, limit, safety, velocity, matched_lines=23 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/rack_hybrid_docking_demo.py`: keywords=e_stop, estop, matched_lines=7 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/rack_industrial_docking.py`: keywords=e_stop, estop, limit, safety, velocity, matched_lines=45 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/rack_lidar_docking.py`: keywords=estop, safety, velocity, matched_lines=25 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/rack_radar_docking.py`: keywords=estop, safety, velocity, matched_lines=13 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/rack_retreat_controller.py`: keywords=estop, safety, velocity, matched_lines=5 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/use_industrial_docking_methods.py`: keywords=e_stop, estop, velocity, matched_lines=14 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/g2_primitives/arm.py`: keywords=safety, matched_lines=1 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/g2_primitives/chassis_motion.py`: keywords=estop, matched_lines=10 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/g2_primitives/nav.py`: keywords=velocity, matched_lines=1 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/g2_primitives/rack.py`: keywords=estop, matched_lines=6 | PROTECTED — DO NOT MODIFY
- `rack_hybrid_docking_package/g2_primitives/waist.py`: keywords=limit, matched_lines=5 | PROTECTED — DO NOT MODIFY
- `tools/g2_project_doctor.py`: keywords=e_stop, estop, limit, safety, torque, velocity, matched_lines=7 | PROTECTED — DO NOT MODIFY

## 7. 诊断结论 / 摘要

- 整体状态: 有 ⚠️
- 项目结构检查项: 10
- 日志文件计数: *.log=25, *.jsonl=10, *.json=25
- Viewer 文件数: 3
- VLA / World Model 文件数: 22
- VLA dry-run JSONL 夹具数: 3
- 机器人运动脚本入口数: 22
- 安全关键字命中文件数: 39
- Warning 数: 5
- Error 数: 0
- 安全声明: 本报告为静态分析, 未运行任何机器人脚本
- 未导入项目运动、docking、controller 模块；未启动任何运行时。
