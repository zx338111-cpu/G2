# G2 七根料工业自动化架构审查

日期：2026-06-10

## 当前判断

项目已经从“临时脚本拼流程”走到了可实机验证的工业流程雏形：

- 抓料距离已经固化为前方 0/1 超声 `170mm`。
- 放料距离已经固化为前方 0/1 超声 `500mm`。
- 后退 1m 已经改为前超声距离增量闭环，不再依赖速度乘时间。
- 90 度转向已经接入 `Slam.get_odom_info()` yaw 校验，不再只看超声场景变化。
- 本轮新增三大核心动作增强：抓料目标窗口和低速补近、后退 odom 交叉校验、
  转向 yaw 低速闭环补角。
- 现场方向映射固定为 `0/1` 前、`2/3` 右、`4/5` 后、`6/7` 左。

但按工业自动化标准看，现在还不能把完整七根作为默认一键生产动作。
首要门槛仍然是：左右 90 度转向要先多次单独测试并通过 yaw gate。

## 主要问题

1. 转向还没有形成已标定能力

代码已经有 yaw gate 和低速闭环补角，但左右转多次物理单测还没完成。
只要这个没有过，完整七根流程就可能在第二根之前累积姿态误差。

2. 总控脚本过大

   `industrial_7_rods_total_controller.py` 已经超过 2600 行，里面同时承担：
   CLI 参数、流程编排、子进程动作、传感器闭环、后退控制、转向控制、
   恢复逻辑、日志。继续堆下去会降低现场可维护性。

3. 机械臂动作脚本仍是黑盒子进程

   当前用子进程调用是正确的安全隔离方式，但每个动作脚本没有统一的
   输入/输出协议。总控只能看返回码和 stdout，不能知道夹爪是否真的闭合、
   末端是否到位、是否发生软失败。

4. 恢复点靠人工参数选择

   现在有多个 `--resume-after-*` 参数，但之前没有机器可读 checkpoint。
   中断后必须人工翻文本日志判断停在哪一步，容易选错恢复点。

5. 运行证据不够结构化

   文本日志适合人看，不适合自动复盘。工业现场需要 JSONL 事件、
   checkpoint、final report，方便快速定位第几根、哪一步、什么传感器门槛失败。

6. preflight 分散

   状态快照、转向 preflight、靠近/后退 preflight 已经存在，但没有一个统一的
   “开跑前健康门禁”。实际生产前应该统一检查：机器人停稳、充电插入为 0、
   motion error 为 0、超声供电、SLAM odom/yaw、前后雷达有效性。

## 本轮已落地优化

新增：

```text
rack_hybrid_docking_package/industrial_run_artifacts.py
```

它提供统一的 `RunRecorder`，负责：

- JSONL 事件流：每一步开始、完成、失败；
- checkpoint JSON：最近运行到哪根、哪一步、建议恢复入口；
- final report JSON：成功/失败、异常栈、最后 checkpoint、关联日志路径。

已接入：

```text
rack_hybrid_docking_package/industrial_7_rods_total_controller.py
```

新增 CLI：

```bash
--event-file
--checkpoint-file
--report-file
```

默认会和文本日志一起写到 `logs/`：

```text
industrial_7_rods_YYYYMMDD_HHMMSS.log
industrial_7_rods_YYYYMMDD_HHMMSS.jsonl
industrial_7_rods_YYYYMMDD_HHMMSS_checkpoint.json
industrial_7_rods_YYYYMMDD_HHMMSS_report.json
```

这次改动不改变运动策略、不改变速度/距离/转向参数，只增强工业运行框架。

随后补强了三大核心工业 primitive：

- 抓料精定位：新增 `grab_target_tolerance_mm=45`、`grab_correction_speed_mps=0.035`、
  `grab_correction_max_passes=1`。抓料停稳后必须进入 `170±45mm`，且不能低于
  `150mm` 安全下限；太远可低速补近，太近或角度不一致直接停机。
- 后退 1m：`front-ultrasonic` 仍为主闭环；新增 SLAM odom 交叉校验，
  odom 可读时位移必须在 `1.0±0.25m` 内，否则失败停机。可用
  `--retreat-require-odom-crosscheck` 强制要求 odom。
- 左/右转 90 度：`relative_move(±90)` 后新增 yaw 低速补角。默认最多
  `3` 次，补角速度 `0.20rad/s`，只有误差在 `25deg` 内才补；超过则认为主转向异常。

详细动作说明见：

```text
rack_hybrid_docking_package/industrial_three_core_primitives_20260610.md
```

## 缺陷模拟后新增修复

2026-06-10 继续按工业现场异常输入做了一轮代码级模拟，已修复这些边界：

- 超声 `distance_mm=None` 或非数字时，不再在比较运算中抛 Python 异常；
  统一当作无效帧过滤，流程按“丢传感器/无稳定锁定”停机。
- `Slam.get_odom_info()` 如果返回的 `pose`/`orientation_euler` 不是普通
  Python 可下标对象，会从 `repr(odom)` 里兜底解析 yaw 和 xy，避免明明日志有
  odom 数据但程序读不到。
- `state=7` 被硬编码为取消/结束，不能通过 `--turn-success-states` 人为放行；
  单独转向诊断脚本和底层 relative 后退也采用同一规则。
- 转向低速补角前和每一轮补角前都会重新做底盘安全检查，避免主转向后
  充电插入、motion error 等状态变化还继续补角。
- 速度开环转向和 yaw 补角的命令帧数改为向上取整，保证至少发够配置时长；
  单独转向诊断 `--repeat` 每次物理转向前也重新做 preflight。
- 多根连续真实运行新增 `--turn-validation-ok` 门禁；未确认左右转多次单测通过时，
  `--confirm-live --start-index 1 --end-index 7` 会被总控拒绝。

## 下一步建议

1. 只读状态门禁

   每次物理运动前先跑：

   ```bash
   python3 industrial_status_snapshot.py --samples 8
   ```

   必须确认 `stopped=True`、`charge_plug_insert_state=0`、`motion_control error_code=0`、
   odom/yaw 可读。

2. 先做转向单测

   ```bash
   python3 industrial_turn_diagnostic.py --confirm-live --direction right --method relative --repeat 3
   python3 industrial_turn_diagnostic.py --confirm-live --direction left  --method relative --repeat 3
   ```

   不通过就不要跑完整七根。通过后跑多根必须显式加 `--turn-validation-ok`。

3. 把总控拆成四层

   后续建议拆分为：

   - `industrial_process_model.py`：七根料工艺步骤定义；
   - `industrial_robot_adapters.py`：GDK/PNC/SLAM/超声适配；
   - `industrial_safety_gate.py`：统一 preflight 和运动前门禁；
   - `industrial_7_rods_total_controller.py`：只保留 CLI 和流程调度。

4. 统一动作脚本协议

   每个机械臂/夹爪脚本后续应输出一行 JSON：

   ```json
   {"status":"ok","action":"open_gripper","elapsed_s":1.2}
   ```

   总控先兼容旧 stdout，逐步把返回码升级为结构化动作结果。

5. 完整七根前先做局部闭环

   推荐顺序：

   - 右转 3 次；
   - 左转 3 次；
   - 单独跑第 1 根到放料前；
   - 第 1 根完整闭环；
   - 再考虑 `start-index 1 end-index 7`。
