# 2026-06-16 Map20 Class/Import 架构与整轮跑通交接

## 当前结论

- 当前机器人：`agi@192.168.0.7`
- 当前机器人工作目录：`/data/g2_industrial_cell_20260612/wxf/BOX_528_1`
- 当前地图：`map_id=20`
- 当前主入口：`rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py`
- 当前架构：主流程脚本 + `g2_primitives` importable class 层 + 旧 CLI 兼容入口
- 2026-06-16 已完成 `1 -> 7` 根整轮验证，最终状态：
  - `phase=MISSION_DONE`
  - `current_station=HOME_SAFE`
  - `holding_rod=false`
  - 只读快照确认底盘静止、`motion_control_error=0`

## 运行命令

机器人端正常整轮运行命令：

```bash
source /home/agi/app/env.sh
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
./rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py --live --confirm-physical --start-index 1 --end-index 7
```

远程一条命令：

```bash
ssh agi@192.168.0.7 'source /home/agi/app/env.sh; cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1; ./rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py --live --confirm-physical --start-index 1 --end-index 7'
```

只读预检：

```bash
ssh agi@192.168.0.7 'source /home/agi/app/env.sh; cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1; ./rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py --preflight-only'
```

## 今日 live 结果

第 1 根先单独 smoke：

- log：`/data/g2_industrial_cell_20260612/wxf/BOX_528_1/logs/single_debug_map20_20260616_170317.log`
- final：`MISSION_DONE / HOME_SAFE / holding_rod=false`

第 2-7 根连续运行：

- log：`/data/g2_industrial_cell_20260612/wxf/BOX_528_1/logs/single_debug_map20_20260616_170612.log`
- analysis：`/data/g2_industrial_cell_20260612/wxf/BOX_528_1/logs/single_debug_map20_20260616_170612_analysis.txt`
- final：`rod_index=7 / end_index=7 / phase=MISSION_DONE / HOME_SAFE / holding_rod=false`

分析报告里的主要耗时：

- `LOCAL_PLACE`: 6 次，平均约 `62.268s`
- `LOCAL_PICK`: 6 次，平均约 `35.752s`
- `NAV_TO_PLACE`: 6 次，平均约 `16.989s`
- `NAV_TO_RECOVERY`: 6 次，平均约 `16.426s`
- `NAV_TO_HOME`: 6 次，平均约 `11.629s`

运行中仍会看到这类 GDK/PNC 警告：

```text
Pnc remote control rsp promise is nullptr
Pnc task cancel rsp promise is nullptr
Pnc task cancel failed, reason: Task is not in RUNNING or PAUSED state
```

本轮这些警告未导致任务失败。后续如果同时出现脚本异常、PNC 不到点、持料状态异常，才按故障处理。

## 今日确认过的关键调参

当前调参集中在 `industrial_cell_7_rods_single_debug.py` 的 `TUNED` 字典里。

抓料：

- 抓料精定位：`grab_final_stop_mm=328`
- 抓料后退：总计 `-0.20m`
- 抓料后退不再下压：`pick_down_z_m=0.0`
- 第一段后退 `-0.085m`，第二段补齐剩余后退

放料：

- 放料精定位：`place_final_stop_mm=308`
- 开爪前最终放料点：双臂前进 `0.03m`、下压 `0.025m`
- 放完料后才后退下移，总下移 `0.08m`
- 放完料后手臂总后退 `0.25m`
- 放料后的底盘退出：`retreat_after_place distance_m=0.45`

特别注意：`0.08m` 下移是“放完料后、后退过程中下移”，不是放料前下移 `0.08m`。

## 当前架构

主流程：

- `rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py`

主流程职责：

- CLI 参数和安全 gate
- `preflight-only`、readiness、process/file check
- checkpoint 创建/加载/保存
- 状态机推进
- pick/place local plan 编排
- station navigation 编排
- 总日志、单步日志、post-run analysis

底层 class：

- `rack_hybrid_docking_package/g2_primitives/arm.py`
  - `ArmJointController`
  - 双臂按 14 关节 JSON 到位
- `rack_hybrid_docking_package/g2_primitives/waist.py`
  - `WaistController`
  - 腰部/上半身 5 关节到位，带分段和反馈 settle
- `rack_hybrid_docking_package/g2_primitives/gripper.py`
  - `GripperController`
  - 左右夹爪开合
- `rack_hybrid_docking_package/g2_primitives/ee_offset.py`
  - `EndEffectorOffsetController`
  - 左右末端相对偏移
- `rack_hybrid_docking_package/g2_primitives/nav.py`
  - `MapNavController`
  - map station 导航、到点判断、yaw refine、readiness
- `rack_hybrid_docking_package/g2_primitives/rack.py`
  - `RackDockingController`
  - 超声精定位、底盘相对后退
- `rack_hybrid_docking_package/g2_primitives/gdk_context.py`
  - `gdk_session`
  - 统一 GDK init/release

旧 CLI 兼容入口仍保留：

- `move_arm_by_json_path.py`
- `move_waist_by_json_path.py`
- `move_ee_relative_offset.py`
- `move_ee_pose_open_2.py`
- `move_ee_pose_close_2.py`

这些旧脚本现在主要用于单步调试。主程序内部已经优先 import class，不再靠 shell 子进程串动作。

## 今日代码整理

已给以下文件补详细注释，方便后续看架构和排障：

- `rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py`
- `rack_hybrid_docking_package/g2_primitives/__init__.py`
- `rack_hybrid_docking_package/g2_primitives/arm.py`
- `rack_hybrid_docking_package/g2_primitives/waist.py`
- `rack_hybrid_docking_package/g2_primitives/gripper.py`
- `rack_hybrid_docking_package/g2_primitives/ee_offset.py`
- `rack_hybrid_docking_package/g2_primitives/nav.py`
- `rack_hybrid_docking_package/g2_primitives/rack.py`
- `rack_hybrid_docking_package/g2_primitives/gdk_context.py`
- `move_arm_by_json_path.py`
- `move_waist_by_json_path.py`
- `move_ee_relative_offset.py`
- `move_ee_pose_open_2.py`
- `move_ee_pose_close_2.py`

## 后续建议

1. 下次如果只想确认安全，不要直接跑 live，先跑 `--preflight-only`。
2. 如果完整跑下一轮，仍用 `industrial_cell_7_rods_single_debug.py --live --confirm-physical --start-index 1 --end-index 7`。
3. 如果机器人物理状态和 checkpoint 不一致，不要直接 `--resume`；先做 read-only snapshot，再按物理状态恢复。
4. 后续若要进一步产品化，可以从 `single_debug` 拆一个更短的稳定入口，比如 `run_map20_7_rods_live.py` 只负责调用固定参数。
