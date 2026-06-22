# 2026-06-11 完整流程后的工业自动化优化记录

## 完整流程暴露的问题

- 第 3 根放料后后退时，前超声目标恢复需要人工指定目标距离继续；说明放料后退虽然已有前超声增量和 odom 交叉校验，但异常恢复还依赖人工读日志判断目标。
- 第 5 根抓料靠近时，抓取目标为 155mm，停稳纠偏后前超声最小值落到 133mm，低于 135mm 安全下限，程序直接失败，需要人工低速后退约 4cm 后继续。
- 程序默认抓取纠偏速度仍是 0.035m/s、最多 3 次；完整流程最终跑通使用的是 0.02m/s、最多 5 次，默认值没有反映现场验证结果。
- 前超声在后退和近距离纠偏时仍会出现短时跳变或两侧跨度变化；现有多帧稳定读数和单前超声加 odom 保护能兜底，但仍应作为后续传感器安装/标定项。

## 已优化

- 抓料/放料前超声靠近后，如果稳定复检发现 `front_min_mm < min_safe_mm`，不再立即把流程交给人工；程序会先执行一次受限低速安全后退：
  - 后退方向远离料架；
  - 后退距离按目标窗口和安全下限自动计算，限制在 0.02m 到 0.08m；
  - 后退速度不超过 0.025m/s；
  - 保留后方超声保护；
  - 后退后再次读取多帧稳定前超声；
  - 复检仍低于安全下限时才失败停机。
- 抓取纠偏默认参数改为完整流程实测稳定值：
  - `--grab-correction-speed-mps` 默认 `0.02`；
  - `--grab-correction-max-passes` 默认 `5`。
- 新增 `front_too_close_safe_backoff_start` 和 `front_too_close_safe_backoff_confirmed` 事件，后续可以在 report/event log 中确认自动恢复是否触发、退了多少、复检距离是多少。
- 新增料架居中监控第一阶段：
  - 默认 `--rack-centering-mode monitor`；
  - 每次抓料/放料前雷达靠近前后都会记录 `rack_pose_monitor` 事件；
  - 点云 pose 输出 `distance_m`、`lateral_center_m`、`yaw_deg`、`confidence`、点簇数量和拟合残差；
  - 当前只记录 `ok/warn/unavailable`，不控制底盘、不阻断流程；
  - 侧超声仍只做诊断，不参与前后保护或居中闭环。
- 新增离线分析工具 `analyze_rack_pose_events.py`：
  - 输入总控 `.jsonl` 事件文件；
  - 汇总每次 `rack_pose_monitor` 的横向偏移、yaw、置信度和拟合残差；
  - 识别 `rack_pose_yaw_shadow` 事件，统计候选纠偏次数、拒绝原因和候选转角范围；
  - 按整体、每根、抓/放料目标、靠近前/靠近后输出 Markdown/JSON；
  - 自动给出 `pose_quality`、`yaw_next_step` 和横移控制边界建议。
- 新增料架 yaw 纠偏第二阶段 shadow 模式：
  - `--rack-centering-mode shadow` 仍不执行底盘纠偏，只生成 `rack_pose_yaw_shadow` 决策事件；
  - 只允许 `before_approach` 的候选进入评估；
  - 默认拒绝近距离 `distance_m < 0.70`、远距离 `distance_m > 1.60`、`confidence < 0.65`、拟合残差 `> 0.045m`、yaw 过小或过大的样本；
  - 候选输出 `candidate_robot_yaw_correction_deg`，但 `candidate_sign_calibrated=false`，必须通过后续小角度实测确认符号后才能进入 active 控制。

## 2026-06-11 第1-3根料架监控验证

- 第1根 + 第2-3根合并后共有 `12` 条 `rack_pose_monitor` 事件，达到离线分析最小样本数。
- 合并报告结论：
  - `pose_quality=usable_for_yaw_review`；
  - `yaw_next_step=candidate_for_yaw_correction_test`；
  - `lateral_next_step=do_not_enable_lateral_control_until_linear_y_is_verified`。
- 近距离 `after_approach` 样本不适合作为自动纠偏输入：
  - 155mm 抓料后出现 `yaw=None`、`confidence=0.3317`；
  - 155mm 抓料后也出现过 `yaw=29.9265deg` 的明显异常候选；
  - 260mm 到 420mm 附近的点云姿态更容易受料架边缘、手臂/料件遮挡和 ROI 截断影响。
- 目前只允许在 shadow 模式观察“靠近前”的 yaw 候选；横向 `linear.y` 居中控制仍禁止启用，直到单独验证底盘横移指令的真实运动方向、速度和里程反馈。

## 2026-06-11 第1-7根 shadow 完整验证

- 第1-7根已全部跑完，使用 `front-ultrasonic` 后退、速度闭环转向和料架 pose/shadow 监控。
- 合并报告 `logs/live_rod1_7_rack_pose_20260611_combined_analysis.md`：
  - `rack_pose_monitor` 事件共 `28` 条；
  - `pose_quality=usable_for_yaw_review`；
  - `yaw_next_step=candidate_for_yaw_correction_test`；
  - `lateral_next_step=do_not_enable_lateral_control_until_linear_y_is_verified`。
- shadow 决策事件共 `16` 条：
  - `no_correction_needed=7`；
  - `rejected=9`；
  - `candidate_count=0`；
  - 说明当前实跑中尚没有一条满足阈值并需要 yaw 纠偏的样本。
- 近距离 `after_approach` 仍不适合作为自动纠偏输入：
  - 多次出现低置信度、yaw 不可用或 yaw 大幅跳变；
  - shadow 规则均能用 `distance_too_close`、`phase_not_before_approach`、`confidence_too_low` 等原因拒绝。
- 每次成功流程结束后，PNC 仍可能残留 `state=2 id=2`。总控已加入 `cleanup_final_pnc_task()`：
  - 成功跑完所有请求杆后自动读取 PNC task；
  - 若 task 仍在运行/暂停类状态，自动 `cancel_task()`；
  - dry-run 不执行；
  - 清理失败只记录 `final_pnc_cleanup` 事件，不把已完成流程改判失败。
- 总控已加入 `record_final_status_snapshot()`：
  - 成功收尾后自动记录电源、motion control、whole body、PNC task、四向超声、odom 速度采样；
  - 自动输出 `odom_available`、`max_linear_speed_mps` 和 `stopped`；
  - 结果写入总控 `.jsonl` 的 `final_status_snapshot` 事件。

## 2026-06-11 linear.y 横移验证

- 新增独立脚本 `industrial_linear_y_diagnostic.py`，用于在主流程外验证底盘横移：
  - 默认 `--speed-mps 0.05 --duration-s 1.0 --sequence positive-negative`；
  - 先检查 `charge_plug_insert_state=0`、`motion_control_error=0`、急停、超声供电；
  - 采样前/后/左/右超声 clearance，中位数低于阈值则不发运动；
  - 读取 odom xy/yaw，把世界坐标位移换算成起点车体坐标下的 forward/lateral；
  - 结束后自动停车并取消 PNC 残留任务。
- 第一次预检读到 `charge_plug_insert_state=1`，脚本按预期阻断，没有发底盘运动。
- 第二次预检变为 `charge_plug_insert_state=0` 后，执行小距离横移验证：
  - `linear.y=+0.05m/s, 1.0s`：body lateral 约 `-0.0267m`，body forward 约 `+0.0408m`；
  - `linear.y=-0.05m/s, 1.0s`：body lateral 约 `+0.0439m`，body forward 约 `-0.0714m`；
  - 最大横向位移约 `0.0439m`，大于 `0.02m` 判定阈值，说明 `linear.y` 物理上可动；
  - 但命令符号与 odom body lateral 解算符号相反，且存在明显前向耦合，不能直接接入料架横向闭环。
- 当前结论：
  - 横移能力已从“未知”变成“可动但需标定”；
  - 下一步应做更系统的横移标定：多次 `+y/-y`、不同速度/时长、统计横向增益、前向耦合和 yaw 漂移；
  - 横移闭环应继续先做 shadow，不允许直接 active。

## 2026-06-11 横移 sweep 与 lateral shadow

- 新增 `industrial_linear_y_sweep.py`：
  - 默认 `--speed-mps-list 0.03,0.05 --duration-s 0.8 --repeat-count 2 --sequence positive-negative`；
  - 复用 `industrial_linear_y_diagnostic.py` 的充电、motion control、急停、超声 clearance、odom、安全停车和 PNC 清理逻辑；
  - 输出每条横移腿的 body forward/body lateral/yaw delta；
  - JSON summary 会给出 `command_to_body_lateral_sign`、横向增益中位数、最大前向耦合和最大 yaw 漂移；
  - 默认侧向 clearance 提高到 `650mm`，用于多腿 sweep 前的保守保护。
- 总控 `--rack-centering-mode shadow` 已新增 `rack_pose_lateral_shadow`：
  - 只在 `before_approach`、距离/置信度/拟合残差满足阈值时生成横移候选；
  - 默认横向触发阈值 `0.025m`，单次候选修正量截断到 `0.03m`；
  - 事件记录 `candidate_body_lateral_correction_m`、`reasons` 和 `execution_blockers`；
  - `candidate_execution_allowed=False`，不会发任何 `linear.y` 运动；
  - 执行阻断原因固定包含 `active_lateral_control_disabled` 和 `linear_y_direction_gain_not_sweep_calibrated`。
- `analyze_rack_pose_events.py` 已支持汇总 `rack_pose_lateral_shadow`，会统计候选横移量、拒绝原因和 active 阻断原因。
- 已执行一次保守 live pilot sweep：
  - 命令：`python3 industrial_linear_y_sweep.py --confirm-live --speed-mps-list 0.03,0.05 --duration-s 0.6 --repeat-count 1 --sequence positive-negative --min-side-clearance-mm 650 --min-front-rear-clearance-mm 500 --report-json logs/linear_y_sweep_pilot_20260611_1642.json`；
  - 报告：`logs/linear_y_sweep_pilot_20260611_1642.json`；
  - `0.03m/s` 正向：body lateral `-0.0096m`，body forward `+0.0042m`，yaw delta `+0.5687deg`；
  - `0.03m/s` 反向：body lateral `+0.0098m`，body forward `-0.0128m`，yaw delta `+0.0078deg`；
  - `0.05m/s` 正向：body lateral `-0.0258m`，body forward `+0.0461m`，yaw delta `+0.0161deg`；
  - `0.05m/s` 反向：body lateral `+0.0295m`，body forward `-0.0469m`，yaw delta `+0.0675deg`；
  - summary：`command_to_body_lateral_sign=positive_linear_y_produces_negative_body_lateral`，`positive_linear_y_gain_median=-0.6973`，`max_abs_body_lateral_m=0.0295`，`max_abs_body_forward_m=0.0469`，`passes_min_displacement=True`；
  - 结论：可用于 shadow 分析，不足以直接 active；`0.05m/s` 横移有效但前向耦合过大，后续 active 初测应优先考虑 `0.03m/s` 或更短时长。
- pilot 后只读快照确认：`charge_plug_insert_state=0`，`motion_control_error=0`，PNC `state=7`，odom 速度 `0`，四向超声仍在安全范围。

## 2026-06-11 单根 lateral shadow 与横向修正探针

- 单根 `rack_pose_lateral_shadow` 验证：
  - 第 1 根完整动作被拆成两段完成：前半段在抓取后退完成、右转前被 `motion_control_error=2` 阻断；只读快照确认 `collision_pairs_1/2` 为空后，使用 `--resume-after-grab-retreat-index 1 --allow-turn-motion-error-2` 从右转继续，并完成第 1 根。
  - 右转 yaw 闭环实际约 `89.988deg`，左转约 `-89.906deg`，均在 `0.5deg` 容差内。
  - 合并分析 `logs/live_rod1_lateral_shadow_20260611_combined_analysis.md`：4 条 `rack_pose_monitor`，4 条 `rack_pose_lateral_shadow`，全部 rejected，`candidate_count=0`。
  - rejected 主因：`lateral_too_large_for_shadow`，说明不是没有识别横向偏差，而是偏差超过当前安全 shadow 候选范围。
- 当前“未居中”现场复核：
  - 只读点云 rack pose：中位 `lateral_center_m=-0.1363m`，`yaw_deg` 约 `0deg`，主要是横向偏，不是角度偏。
  - 四向超声也显示左右侧距离不对称，但侧向超声偶有 invalid/跳变，只能作为辅助判断，不能单独当闭环输入。
- 横向修正探针：
  - step1：`+linear.y 0.03m/s 0.6s`，odom body lateral `-0.0069m`，rack pose 约 `-0.136m -> -0.115m`；
  - step2：`+linear.y 0.03m/s 0.8s`，odom body lateral `-0.0095m`，rack pose 约 `-0.115m -> -0.089m`；
  - step3：`+linear.y 0.03m/s 1.0s`，odom body lateral `-0.0149m`，但 rack pose 变为约 `-0.160m`；
  - step4：`-linear.y 0.03m/s 0.6s`，odom body lateral 仅 `+0.0022m`，yaw drift 约 `-0.455deg`，rack pose 约 `-0.170m`。
- 工程结论：
  - 横向底盘运动确实可以作为居中修正手段，但不能用固定时长开环一次性修。
  - 可用策略必须是“小步闭环”：读 rack pose -> 单步横移 -> 停稳复测 -> 变好才继续，变差立即停止并要求复核。
  - 首选单步应限制在 `0.03m/s`、`0.6~0.8s`，且每步检查 `motion_error=0`、`charge_plug_insert_state=0`、PNC `state=7`、odom 停止、前/后/左右超声 clearance。
  - 不允许用固定时长开环 active；必须把“变好才继续”的规则固化到主流程入口。

## 2026-06-11 抓料/放料前 active 横向居中逻辑

- 总控 `industrial_7_rods_total_controller.py` 已把横向居中逻辑接入 `approach_by_front_ultrasonic()`：
  - 该函数是抓料靠近和放料靠近共用入口，所以每次抓料前、放料前都会先看料架是否居中；
  - 默认 `--rack-centering-mode monitor` 不发横移动作；
  - `--rack-centering-mode shadow` 继续只记录 yaw/横向候选；
  - `--rack-centering-mode active` 才启用真实小步蟹行横向修正。
- active 模式策略：
  - 先读 `before_approach` rack pose；
  - 若 `|lateral_center_m| <= 0.08m`，直接认为可进场；
  - 若横向偏差在 `0.08~0.18m` 且距离、置信度、拟合残差、yaw 都满足阈值，则执行一次小步 `linear.y`；
  - 当前方向按实测固定：`lateral_center_m < 0` 时发 `linear.y > 0`，`lateral_center_m > 0` 时发 `linear.y < 0`；
  - 单步默认 `0.03m/s * 0.6s`，最多 3 步；
  - 每步后停稳 `0.6s` 并重新读 rack pose；
  - 只有 `|lateral_center_m|` 至少改善 `0.006m` 才允许继续；变差或改善不足直接停流程复核。
- active 模式每步发运动前会重新做安全门：
  - `rack.preflight()` 必须通过；
  - `motion_control_error` 必须为 0；
  - `charge_plug_insert_state` 必须为 0；
  - 前/后/左/右有效超声低于阈值会拒绝横移；
  - 使用现有 `rack.front.pnc` 和 `request_chassis_control_ready()`，不在 rack 控制器上下文内重复 `gdk_init/gdk_release`。
- 推荐下一步验证：
  - 先远端 dry-run 确认参数和完整脚本检查；
  - 再只跑第 1 根或当前现场可控的一次抓/放进场，命令中显式加 `--rack-centering-mode active`；
  - active 单根通过后，再考虑把该模式用于多根连续流程。

## 2026-06-11 active 横向居中 probe

- 新增 `industrial_lateral_centering_probe.py`：
  - 复用总控的 `monitor_rack_pose()` 和 `center_rack_lateral_before_approach()`；
  - 只测试进场前横向居中，不执行手臂动作、不前进靠近、不后退、不转向；
  - 用于在完整单根 active 前验证小步蟹行闭环。
- 远端 dry-run 已通过：
  - `python3 rack_hybrid_docking_package/industrial_lateral_centering_probe.py --dry-run --rack-centering-mode active --log-file logs/dryrun_lateral_centering_probe_20260611.log`
- live probe 第一次因脚本方法名写错停在 Python 调用层，未发运动；已修正为 `with_industrial_rack()`。
- live probe 第二次在启动预检阻断，仍未发运动：
  - `charge_plug_insert_state=1`；
  - 只读快照确认充电电压约 `50V`、输入电流约 `14.6A`，电池处于 charging；
  - `motion_control_error=0`、PNC `state=7`、odom 静止、四向超声安全，但充电状态是硬阻断，不能绕过。
- 解除充电后的 active probe：
  - 启动预检通过，`charge_plug_insert_state=0`、`motion_control_error=0`、PNC `state=7`；
  - `before_approach` pose 读到 `lateral_center_m=-0.0975m`、yaw 约 `0.42deg`、confidence 约 `0.804`；
  - 按当前方向表执行 `+linear.y 0.03m/s 0.6s` 后，复测变为 `lateral_center_m=-0.2195m`，误差变大；
  - active 逻辑按预期停机，状态为 `no_improvement`，没有继续第二步；
  - 报告 `logs/live_lateral_centering_probe_20260611_next_analysis.md` 中 `Lateral Active` 显示 `improvement_m_median=-0.122`。
- 修正后的 active 保护：
  - 新增 `lateral_sample_span_m`，active 会检查多帧横向 pose 是否稳定；
  - 默认 `--rack-lateral-active-max-sample-span-m 0.08`，超过 8cm 直接 blocked；
  - active 现在要求 `--rack-pose-samples >= 8`，避免默认 3 帧采样误触发；
  - 若某步后横向误差变大，会先反向回退同等小步，然后停机复核；只有回退后已经进目标窗口才允许返回成功。
- 稳定性门限 live 验证：
  - 使用 `--rack-pose-samples 8` 复测，`lateral_sample_span_m=0.226m`；
  - active 在发运动前以 `lateral_sample_unstable` 阻断，未发横移；
  - 报告 `logs/live_lateral_centering_probe_stability_gate_20260611_analysis.md` 中 `Lateral Active` 显示 `decision_counts={'blocked': 1}`。
- 反向恢复诊断：
  - 执行 `-linear.y 0.03m/s 0.6s` 尝试抵消前一次小步；
  - odom 显示 body lateral 仅约 `+0.0018m`，yaw drift 约 `-0.452deg`，不足以有效恢复横向；
  - 最终只读快照确认 `charge_plug_insert_state=0`、`motion_control_error=0`、PNC `state=7`、odom 速度为 0；
  - 最终 8 帧 rack pose 中位 `lateral_center_m=-0.134m`，但横向采样跨度仍约 `0.263m`。
- 当前结论：
  - active 横移闭环保护有效，但当前 lidar pose 横向读数不稳定，且 `linear.y` 小步方向/增益在现场表现不一致；
  - 不应继续盲跑 active 单根，更不能跑 7 根 active；
  - 下一步应先处理 pose 稳定性：收紧/重设 rack pose ROI、提高采样质量，或引入更可靠的横向观测源。
- 后续如要继续只做受保护 probe，必须显式加 8 帧采样：

```bash
python3 rack_hybrid_docking_package/industrial_lateral_centering_probe.py \
  --confirm-live \
  --rack-centering-mode active \
  --rack-pose-samples 8 \
  --rack-lateral-shadow-max-fit-residual-m 0.07 \
  --rack-lateral-active-max-initial-m 0.23 \
  --rack-lateral-active-max-sample-span-m 0.08 \
  --retreat-method front-ultrasonic \
  --turn-method velocity \
  --log-file logs/live_lateral_centering_probe_20260611_next.log
```

## 2026-06-11 rack pose ROI 稳定化与 active 方向门控

- 新增只读 ROI 扫描工具 `industrial_rack_pose_roi_sweep.py`：
  - 直接使用 `agibot_gdk.Lidar()` 读取前激光点云，不创建 PNC/Robot 控制对象，不发 `stop/cancel`；
  - 扫描多组 `min/max range`、横向半宽、Z 高度、前向分箱和点簇点数；
  - 输出 `logs/rack_pose_roi_sweep_20260611_1745.json` 和 `.md`。
- ROI 扫描结论：
  - 旧默认 ROI `0.12-2.5m, lateral_half=0.8, z=0.6-1.2, bin=0.25`：`lateral_sample_span_m=0.141m`，不满足 active 稳定门；
  - 最优通过组 `0.8-1.4m, lateral_half=0.5, z=0.6-1.2, bin=0.2, pts=20`：8/8 有效，`lateral_center_m_median=-0.0622m`，`lateral_sample_span_m=0.058m`，`confidence_median=0.8418`，`fit_residual_m_median=0.0422`。
- 生产代码修改：
  - `rack_lidar_docking.py` 的 `read_rack_pose()` 支持按次覆盖 ROI 参数；
  - 总控默认 rack pose ROI 改为上述稳定组；
  - `rack_lateral_shadow_max_fit_residual_m` 默认从 `0.045m` 调到 `0.070m`，避免稳定 ROI 被旧残差阈值误挡；
  - `rack_pose_monitor` 和 `rack_lateral_centering_pose` 事件会记录 `pose_roi`，便于后续日志复盘；
  - 包内入口和根目录入口均已同步并远端 `py_compile` 通过。
- ROI 默认值下的 active probe：
  - 第一次默认 `0.08m` 目标：`lateral_center_m=-0.072m`、`lateral_sample_span_m=0.062m`，判定 `centered`，未发横移；
  - 后续收紧到 `0.05m` 目标验证真实蟹行：`+linear.y` 和 `-linear.y` 两个方向都未稳定改善 rack pose，且 rollback 也不能可靠恢复到起点；
  - 最新门控 probe：`direction=disabled`，偏差 `-0.1095m` 且 `lateral_sample_span_m=0.092m` 时以 `linear_y_direction_not_calibrated` 和 `lateral_sample_unstable` 阻断，未发横移。
- 当前生产结论：
  - “每次抓料/放料前看是否居中”的逻辑已经接入，并且 ROI 稳定性比旧版本明显改善；
  - 但“自动蟹行纠正”在当前现场还不能作为工业默认动作，因为横移后 lidar 横向估计会继续漂，无法证明固定方向/增益可靠；
  - 因此 `--rack-centering-mode active` 默认只允许 `centered` 通过，偏差需要修正时会阻断；
  - 若后续要做方向诊断，必须显式传 `--rack-lateral-active-direction same-sign` 或 `opposite-sign`，并继续只用 probe，不要直接放进 7 根流程。

## 2026-06-11 lateral active 响应合并分析

- 新增 `analyze_lateral_active_response.py`：
  - 只读 JSONL 事件；
  - 汇总每次 `rack_lateral_centering_step_result` 的 `vy_mps`、before/after lateral、improvement、rollback 结果；
  - 输出按 `vy` 正负分组的改善统计和推荐结论。
- 已在机器人端合并分析 3 份关键 probe：
  - `logs/lateral_centering_probe_tight_target_20260611_1752.jsonl`
  - `logs/lateral_centering_probe_tight_target_fixed_sign_20260611_1757.jsonl`
  - `logs/lateral_centering_probe_direction_disabled_20260611_1801.jsonl`
- 输出：
  - `logs/lateral_active_response_combined_20260611_1808.md`
  - `logs/lateral_active_response_combined_20260611_1808.json`
- 合并结论：
  - `step_count=2`，`blocked_count=1`；
  - `+linear.y` 一次：`improvement_m=-0.0360`；
  - `-linear.y` 一次：`improvement_m=-0.0247`；
  - `improvement_m_median=-0.0304`；
  - `rollback_improvement_m_median=0.0050`，回退也不稳定；
  - recommendation 为 `keep_active_lateral_motion_disabled`，原因是 `all_executed_lateral_steps_worsened_pose`。
- 工程判断：
  - 现在不能再把问题简化成“方向符号选错”；
  - 更可能是横移时车体 yaw/forward 耦合、lidar ROI 选中料架不同边/层，或 SLAM/点云时序导致 pose 与真实横移不同步；
  - 下一步如果继续攻这个点，应先采“横移前/中/后高频点云 + odom”的诊断包，而不是继续让 active probe 直接驱动生产流程。

## 2026-06-11 高频只读 trace 与高样本 ROI 复扫

- 新增 `industrial_lateral_motion_trace.py`：
  - 默认只读，可记录 rack pose、前向 bin、odom、超声 clearance；
  - live 模式需要显式 `--confirm-live`，默认小步 `0.02m/s * 0.35s`；
  - 本轮只执行 `--read-only`，没有发底盘运动。
- 只读 trace 输出：
  - `logs/lateral_motion_trace_readonly_20260611_1815.md`
  - `logs/lateral_motion_trace_readonly_20260611_1815.json`
  - `logs/lateral_motion_trace_readonly_20260611_1815.jsonl`
- 只读 trace 结论：
  - 32 帧全部选中同一个 `1.200-1.400m` bin；
  - 但 `lateral_center_m` 静态 span 仍为 `0.092m`；
  - 说明当前抖动不主要来自 bin 跳变，而是同一距离簇内的横向中心不稳定。
- 高样本 ROI 复扫输出：
  - `logs/rack_pose_roi_sweep_highsample_20260611_1816.md`
  - `logs/rack_pose_roi_sweep_highsample_20260611_1816.json`
- 24 帧/配置 ROI 复扫结果：
  - 当前生产默认 `0.8-1.4m, lateral_half=0.5, z=0.6-1.2, bin=0.2, pts=20` 本轮通过：`lat_span=0.0655m`；
  - `1.2-1.4m, lateral_half=0.35, z=0.6-1.2, bin=0.2, pts=20` 最稳：`lat_span=0.063m`，但横向中位变为 `-0.047m`；
  - `1.2-1.4m, lateral_half=0.4, pts=20/40` 也能通过，`lat_span` 约 `0.070-0.0735m`；
  - `bin=0.1` 的配置出现 `1.2-1.3` 与 `1.3-1.4` bin 切换，span 明显变差，不建议使用。
- 工程判断：
  - 更窄 lateral ROI 可以降低 span，但会裁剪料架宽度，可能把真实横向偏差低估为“已居中”；
  - 当前生产默认暂不改窄，继续保留 `lateral_half=0.5` 作为更保守的偏差观测；
  - active 横移仍保持方向门控 `disabled`；
  - 后续若继续做 live trace，应先在只读 trace 连续稳定后，再用 `0.02m/s * 0.35s` 单方向采“运动中 pose + odom”，不要直接恢复 active 自动纠偏。

## 2026-06-11 robust 采样稳定门控

- 问题复现：
  - 机器人当前 `charge_plug_insert_state=1`，所以不允许任何 live 横移；
  - trace 工具原先在只读模式也会被插枪状态阻断，本轮已修正为 `--read-only` 下只记录 warning，live 模式仍继续 hard block；
  - 10 秒只读 trace 证明同一 `1.200-1.400m` bin 内仍有尖峰，raw max-min span 会被拉大。
- 新增/修改：
  - `industrial_7_rods_total_controller.py` 新增 `lateral_sample_stats()`；
  - `rack_pose_monitor` 和 `rack_lateral_centering_pose` 同时记录：
    - `lateral_sample_span_m`：原始 max-min，保留用于复盘；
    - `lateral_sample_robust_span_m`：按两端 10% 去尖峰后的稳定 span；
    - `lateral_sample_mad_m`：中位绝对偏差；
    - `lateral_sample_trim_count`：本次每端裁剪样本数；
  - active 稳定性判定现在优先使用 `lateral_sample_robust_span_m`，旧日志缺字段时才回退到 raw span；
  - `industrial_lateral_motion_trace.py` 的 phase summary 也输出 robust span 和 MAD。
- 本轮机器人端 10 秒只读 trace 输出：
  - `logs/lateral_motion_trace_readonly_robust_20260611_1835.md`
  - `logs/lateral_motion_trace_readonly_robust_20260611_1835.json`
  - `logs/lateral_motion_trace_readonly_robust_20260611_1835.jsonl`
- 现场数据：
  - preflight：`motion_error=0`，`charge_plug_insert_state=1 read_only allowed`，`emergency_stop_pedal_fault_state=1 allowed`；
  - initial clearance median：front `1079mm`，right `1400mm`，rear `720mm`，left `1618.5mm`；
  - 79 帧全部有效，全部选中 `1.200-1.400m` bin；
  - `lat_med=-0.109m`，raw `lat_span=0.113m`；
  - robust `span=0.057m`，MAD `0.014m`，`trim_count=7`。
- 逻辑验证：
  - 输入 raw `0.113m`、robust `0.057m`、偏差 `-0.109m` 时，active decision 不再给 `lateral_sample_unstable`，只因默认 `--rack-lateral-active-direction disabled` 阻断；
  - 输入 robust `0.091m` 时仍会产生 `lateral_sample_unstable`，真正不稳定仍然会挡。
- 验证结果：
  - 本地 `py_compile` 通过；
  - 机器人端 `py_compile` 通过；
  - 机器人端第 1 根 dry-run 通过，日志：
    - `logs/dryrun_robust_centering_20260611_1830.log`
    - `logs/dryrun_robust_centering_20260611_1830_report.json`
- 工程判断：
  - 这次修的是“居中判断稳定性”，不是开放自动横移；
  - 生产默认仍保持 `--rack-lateral-active-direction disabled`；
  - 当前插枪状态下不能做 live trace；
  - 以后若继续验证蟹行方向，必须先解除充电/插枪，再用单方向 `0.02m/s * 0.35s` 的 `industrial_lateral_motion_trace.py --confirm-live` 采运动中 pose + odom，不要直接把 active 自动横移放回 7 根流程。

## 2026-06-11 tiny live 横移 trace 与结论

- 解除插枪后，先做状态快照：
  - `charge_plug_insert_state=0`；
  - `motion_control_error=0`；
  - PNC `state=7`；
  - odom 速度为 0；
  - 初始四向超声满足本轮 tiny trace 安全阈值。
- 执行的 live trace：
  - `+linear.y`：`0.02m/s * 0.35s`，约 7mm 期望横移；
  - `-linear.y`：`0.02m/s * 0.35s`，约 7mm 期望横移；
  - 两次都只跑单方向，不进入总控 active，不跑抓/放料动作。
- 输出：
  - `logs/lateral_motion_trace_positive_tiny_20260611_1845.md`
  - `logs/lateral_motion_trace_positive_tiny_20260611_1845.json`
  - `logs/lateral_motion_trace_negative_tiny_20260611_1850.md`
  - `logs/lateral_motion_trace_negative_tiny_20260611_1850.json`
  - `logs/lateral_motion_trace_tiny_analysis_20260611_1855.md`
  - `logs/lateral_motion_trace_tiny_analysis_20260611_1855.json`
- 合并分析脚本：
  - 新增 `analyze_lateral_motion_trace.py`；
  - 专门分析 `industrial_lateral_motion_trace.py` 的 JSON 输出；
  - 汇总 rack lateral 改善量、odom body lateral、odom body forward、yaw delta，并给出推荐结论。
- 结果：
  - `+linear.y`：rack lateral 从 `-0.0905m` 变为 `-0.1053m`，`improvement=-0.0148m`；
  - `-linear.y`：rack lateral 从 `-0.1067m` 变为 `-0.1245m`，`improvement=-0.0178m`；
  - 两个方向都让 `|lateral_center_m|` 变大；
  - `expected_lateral_m_median=0.007m`，但 `odom_body_lateral_abs_m_median=0.0015m`；
  - 正负两个方向的 odom body lateral 都是负值，说明当前 `linear.y` 指令没有表现出可用于闭环的对称横向响应；
  - `recommendation=keep_active_lateral_motion_disabled`；
  - 原因：
    - `all_trace_steps_worsened_rack_lateral_pose`；
    - `positive_and_negative_linear_y_have_same_odom_lateral_sign`；
    - `odom_lateral_response_less_than_half_expected_open_loop_distance`。
- 停机复核：
  - 两次 trace 后状态快照确认 odom 速度为 0，PNC `state=7`；
  - 但右侧 2 号超声出现低值尖峰，单帧曾到 `338mm`，后续 5 帧中仍有 `476mm`；
  - 因此本轮不再继续发任何底盘运动。
- 安全门增强：
  - `industrial_lateral_motion_trace.py` 的 clearance window 现在同时输出 `median_mm` 和 `raw_min_mm`；
  - 新增参数：
    - `--hard-min-side-clearance-mm`，默认 `450mm`；
    - `--hard-min-front-rear-clearance-mm`，默认 `350mm`；
  - live trace 前只要窗口内有效最小值低于 hard-min，也会阻断；
  - 这比单看 median 更适合当前右侧超声会偶发低值的现场。
- 工程结论：
  - 当前不能把蟹行横移接入抓料/放料前自动居中；
  - 总控仍保持 `--rack-lateral-active-direction disabled`；
  - “识别料架并判断是否居中”可以继续保留；
  - 一旦发现偏差超出目标，不应自动横移，应该阻断并记录原因；
  - 后续若要继续攻横移，需要先解决底盘 `linear.y` 实际运动响应和右侧超声低值尖峰，而不是继续在 7 根流程里试 active。

## 2026-06-11 总控横移 hard-min 安全门

- 已把 trace 工具验证过的 hard-min clearance 思路同步进 `industrial_7_rods_total_controller.py` 的横向 active 发运动前安全检查。
- 新增 active 横移安全参数：
  - `--rack-lateral-active-hard-min-front-mm`，默认 `350mm`；
  - `--rack-lateral-active-hard-min-rear-mm`，默认 `350mm`；
  - `--rack-lateral-active-hard-min-side-mm`，默认 `450mm`；
  - `--rack-lateral-active-clearance-samples`，默认 `5`；
  - `--rack-lateral-active-clearance-interval-s`，默认 `0.12s`。
- 新逻辑：
  - 横移前连续采样一个超声窗口；
  - 原有 `rack_lateral_active_min_*` 阈值用于窗口 median；
  - 新增 `hard_min_*` 阈值用于窗口 raw-min；
  - 只要窗口内任一有效 raw-min 低于 hard-min，就阻断横移；
  - safety 事件会记录 `*_median_mm`、`*_raw_min_mm`、`*_samples_mm` 和阈值。
- 本地 fake radar 反例验证：
  - 右侧 5 帧为 `1000,1000,338,1000,1000mm`；
  - median 为 `1000mm`，但 raw-min 为 `338mm`；
  - 总控按预期以 `right_ultrasonic_raw_min_too_close=338.0<hard_min_450` 阻断。
- 机器人端验证：
  - `python3 -m py_compile rack_hybrid_docking_package/industrial_7_rods_total_controller.py` 通过；
  - 第 1 根 dry-run 通过，日志：
    - `logs/dryrun_hard_min_lateral_safety_20260611_1905.log`
    - `logs/dryrun_hard_min_lateral_safety_20260611_1905_report.json`
- 说明：
  - 这只是把生产路径安全门补齐；
  - 不代表 active 横移可以启用；
  - `--rack-lateral-active-direction` 默认仍是 `disabled`。

## 2026-06-11 反光贴/视觉标记方案判断

- 现场问题不是单纯“识别不到料架”，而是底盘 `linear.y` 响应不能可靠作为纠偏执行器：
  - 正负 tiny trace 都让 `|lateral_center_m|` 变大；
  - odom 横向响应小于理论位移的一半；
  - 正负指令的 odom 横向符号还一致。
- 反光贴可以考虑，但不能直接解决当前闭环问题：
  - 当前 `rack_lidar_docking.py` 的点云算法只使用 xyz 几何；
  - 代码注释中也明确 intensity/timestamp 等字段被忽略；
  - 因此普通反光贴不会被现有算法利用。
- 反光贴/标记要真正有效，必须二选一：
  - 若前激光点云暴露 intensity 字段：新增“反光目标 ROI/强度阈值”检测，再用反光贴做定位；
  - 若不用 lidar intensity：在料架上贴 AprilTag/ArUco，用相机识别位姿。
- 更工业的替代方案：
  - 在料架上加两个固定几何定位件，如竖板、圆柱、V 形导向件，让 lidar 几何更稳定；
  - 如果底盘不能可靠蟹行，小偏差范围内可考虑用机械导向或手臂末端 offset 吸收，但要单独做碰撞和抓取窗口验证；
  - 先修底盘 `linear.y` 实际响应，再谈自动横移闭环。

## 2026-06-11 后退 1m 正负 20mm 强校验

- 目标：
  - 每次抓料后退和放料后退都必须验证 `1m` 后退距离；
  - 不能跑着跑着累计偏差；
  - 完成窗口为 `1000mm ±20mm`。
- 代码变更：
  - `industrial_7_rods_total_controller.py` 新增 `--retreat-target-tolerance-mm`，默认 `20`；
  - `front-ultrasonic` 后退默认使用该窗口；
  - 后退完成前会记录 `front_ultrasonic_retreat_target_verified` 事件；
  - 事件包含 `target_delta_mm`、`tolerance_mm`、`remaining_mm`、`delta_by_id`、`front_filtered_by_id`；
  - 放料后退触发 rear guard 时的“前超声目标已到窗口”也改为同一个 `20mm` 窗口，不再使用旧的更宽窗口。
- odom 交叉校验：
  - `--retreat-odom-tolerance-m` 默认从 `0.25m` 收紧为 `0.02m`；
  - `--retreat-require-odom-crosscheck` 默认开启；
  - 如果 SLAM odom 读不到，或后退位移不在 `1.0m ±0.02m`，直接停机；
  - 仅诊断/应急时可显式传 `--no-retreat-require-odom-crosscheck`，生产不建议使用。
- 机器人端 dry-run 验证：
  - `logs/dryrun_retreat_1m_pm20mm_20260611_1915.log`
  - `logs/dryrun_retreat_1m_pm20mm_20260611_1915_report.json`
  - 第 1 根 dry-run 中抓后退和放后退入口均打印 `retreat_target_tolerance_mm=20`。
- 工程结论：
  - 后续完整流程必须继续使用 `--retreat-method front-ultrasonic`；
  - 不要改回纯速度开环后退；
  - 如果某次后退超出 `±20mm`，程序应继续低速修正或停机，不应进入下一步转向/抓放。

### 独立 1m 后退验证入口

为避免每次验证 1m 后退都必须进入七根料总流程，新增只跑后退原语的脚本：

```bash
python3 rack_hybrid_docking_package/industrial_retreat_1m_validation.py \
  --dry-run \
  --base-dir /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1 \
  --log-file logs/dryrun_retreat_1m_validation_YYYYMMDD_HHMM.log
```

真实单次验证命令为：

```bash
python3 rack_hybrid_docking_package/industrial_retreat_1m_validation.py \
  --confirm-live \
  --base-dir /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1 \
  --distance-m 1.0 \
  --tolerance-mm 20 \
  --odom-tolerance-m 0.02 \
  --log-file logs/live_retreat_1m_validation_YYYYMMDD_HHMM.log
```

该脚本不执行机械臂、靠近料架、转向或七根料循环，只复用
`industrial_7_rods_total_controller.py` 里的
`_retreat_by_front_ultrasonic_delta()`。因此实测通过后，完整流程使用的是同一套
前超声 `1000mm±20mm` + odom `1.0m±0.02m` 约束。

已验证：

- 本地 `py_compile` 通过；
- 机器人端 `py_compile` 通过；
- 机器人端 dry-run 通过并已拉回本地：
  - `logs/dryrun_retreat_1m_validation_20260611_1958.log`
  - `logs/dryrun_retreat_1m_validation_20260611_1958_report.json`

最新只读状态：

- `charge_plug_insert_state=0`，不再充电；
- `motion_control error_code=0`，odom 可读且停稳；
- 前方 `0/1` 稳定约 `1079/1099mm`；
- 后方 `4` 稳定约 `2.25m` 以上，后方 `5` 持续 `65535` invalid。

因此下一次实退前，优先现场确认机器人后方至少 `1.5m` 以上无遮挡；若后方 5 号仍持续
invalid，这次实退可用于 1m 距离验证，但不能把它当作“双后探头保护完全健康”的证据。

### 2026-06-11 独立 1m 实退结果

第一次实退尝试在运动前被正确阻断：

- 日志：`logs/live_retreat_1m_validation_20260611_2008.log`
- 报告：`logs/live_retreat_1m_validation_20260611_2008_report.json`
- 原因：`Slam odom is null`，`front-ultrasonic retreat blocked: odom xy unavailable before motion`
- 结论：这次没有进入运动，属于 odom 起点读取空帧触发的 fail-closed。

随后修复：

- `industrial_7_rods_total_controller.py` 的 `_read_odom_xy_from_slam()` 从单次读取改为
  最多 `12` 次、间隔 `0.18s` 的短重试；
- 仍然强制 odom，读不到仍失败，不降级为无 odom 实跑。

第二次独立实退成功：

- 日志：`logs/live_retreat_1m_validation_20260611_1907_retry.log`
- 报告：`logs/live_retreat_1m_validation_20260611_1907_retry_report.json`
- 起点前超声：`{0: 1079, 1: 1099}`
- 目标前超声：`{0: 2079, 1: 2099}`
- 终点滤波前超声：`{0: 2085, 1: 2089}`
- 后退增量：`{0: 1006, 1: 990}`，最小增量剩余误差 `10mm`
- odom 位移：`0.9819m`，相对 `1.000m` 误差 `-0.018m`
- 结论：前超声 `1000mm±20mm` 和 odom `1.0m±0.02m` 都通过。

后置只读快照：

- `charge_plug_insert_state=0`；
- `motion_control error_code=0`；
- `stopped_check=True`；
- 前方 `0/1` 后续稳定约 `2088~2092mm`；
- 后方 `4/5` 后续约 `1300/1280mm`，恢复为双后探头有效。

### 2026-06-11 后续自动化补丁已落地

已同步到机器人：

- 文件：`rack_hybrid_docking_package/industrial_7_rods_total_controller.py`
- 机器人侧备份：`rack_hybrid_docking_package/industrial_7_rods_total_controller.py.bak_20260611_auto_odom_guarded`
- 机器人侧 dry-run：`logs/dryrun_guarded_auto_odom_remote_20260611.log`

本次补丁：

- `front-ultrasonic` 后退在前超声到 `1000mm±20mm` 窗口后，如果 odom 仍超过
  `1.0m±0.02m`，会先执行受保护低速尾差补偿：
  - 默认最多 `2` 次；
  - 速度默认 `0.025m/s`；
  - 单次可处理误差默认上限 `0.08m`，硬上限 `0.12m`；
  - 继续后退补偿时要求后方超声不低于 `500mm`；
  - 向前补偿时要求前方超声不低于 `260mm`；
  - 补偿后仍用原始 odom 容差门禁，仍不放宽 `±20mm`/`±0.02m` 目标。
- 新增 `--disable-retreat-odom-auto-correction` 和一组
  `--retreat-odom-auto-correction-*` 参数，便于现场临时关闭或收紧补偿。
- 新增 `--rack-centering-mode guarded`：
  - 每次抓料/放料进场前必须读取料架 pose；
  - 若横向/yaw/置信度/稳定性不满足 active 判定条件，则直接停机；
  - 不会盲目蟹行，适合在蟹行方向未标定前先防止“不居中还继续抓放”。
- `--rack-centering-mode active` 仍保留真实小步蟹行；方向仍需显式指定
  `--rack-lateral-active-direction same-sign|opposite-sign`，默认 `disabled` 会 fail-closed。
- 修复最终超声快照 `_group_final_ultrasonic_rows()` 的参数名错误，避免流程结束时报
  `unexpected keyword argument 'invalid_below_mm'`。

已验证：

- 本地 `python3 -m py_compile rack_hybrid_docking_package/industrial_7_rods_total_controller.py` 通过；
- 机器人端 `python3 -m py_compile rack_hybrid_docking_package/industrial_7_rods_total_controller.py` 通过；
- 机器人完整工程 dry-run `--rack-centering-mode guarded --rack-pose-samples 8` 通过；
- 本地函数级 `_group_final_ultrasonic_rows()` 检查通过，`65535` 会被标为 invalid。

### 2026-06-11 guarded 实跑前状态

用户要求继续后，先做只读状态检查，没有执行物理运动。

只读结果：

- `charge_plug_insert_state=1`，当前仍在充电/插枪状态，不能实跑；
- `motion_control error_code=0`；
- `emergency_stop_pedal_state=0`，但 `emergency_stop_pedal_fault_state=1` 仍是现场已知状态；
- PNC task state 为 `7`；
- 前超声 `0/1` 约 `1127~1141mm`；
- 后超声 `4` 约 `2238~2259mm`，`5` 约 `592~598mm`，如果立即做 1m 后退补偿，后方空间偏紧。

在不能实跑的状态下，已完成无运动验证：

- 机器人端完整 7 根 guarded dry-run 通过；
- 日志：`logs/dryrun_guarded_auto_odom_7rods_20260611.log`；
- 报告：`logs/dryrun_guarded_auto_odom_7rods_20260611_report.json`。

解除充电插头状态后，下一步不要直接跑 7 根；先跑第 1 根 guarded 实测，验证：

- 每次抓料/放料进场前的 `rack_lateral_centering_decision`；
- 1m 后退如果 odom 尾差超 `±0.02m`，是否触发
  `front_ultrasonic_retreat_odom_auto_correction_*`；
- 最终状态快照是否正常记录，不再出现 `invalid_below_mm` 参数错误。

### 2026-06-11 第 1 根 guarded 单根实测结果

本轮已完成第 1 根的抓取、转向、放料、后退和回正，最终机器人停稳且无运动错误。
但完成过程依赖一次受保护恢复和一次 `monitor` fallback，因此还不能直接扩大到 7 根 guarded 实跑。

第一次 guarded 实跑：

- 日志：`logs/live_guarded_rod1_auto_odom_20260611_2000.log`
- 报告：`logs/live_guarded_rod1_auto_odom_20260611_2000_report.json`
- 结果：在抓料进场前 fail-closed；
- 原因：料架 lateral 约 `-0.067m`，但 lidar 拟合 yaw 约 `-7.367deg`，
  且 lateral sample 不稳定；
- 结论：guarded 门禁生效，没有在“不确定居中”的状态下继续抓料。

随后做只读 pose 复核：

- 日志：`logs/read_only_pose_after_guarded_block_20260611_2003.log`
- 报告：`logs/read_only_pose_after_guarded_block_20260611_2003_report.json`
- 三个窗口 lateral 约 `-0.104m`、`-0.082m`、`-0.079m`；
- yaw 约 `-5.879deg`、`-6.648deg`、`-5.409deg`；
- lateral sample span 约 `0.168~0.258m`；
- 同时双前超声左右差只有约 `8~14mm`。

判断：lidar 的 yaw/横向拟合在当前料架边缘上存在不稳定，不应该直接蟹行纠偏；
但如果横向偏差仍在安全范围内，且双前超声稳定、左右差小、距离充足，可以把双前超声作为 guarded 复核条件。

因此新增 guarded 超声复核补丁：

- 当 guarded 只因为 `lateral_sample_unstable`、`yaw_too_large_for_lateral_active`
  或 `lateral_within_active_target` 被阻断；
- 且 pose lateral 绝对值不超过默认 `0.12m`；
- 且多帧双前超声左右 span 不超过默认 `35mm`、前方距离不小于默认 `450mm`；
- 则记录 `rack_guarded_ultrasonic_override` accepted，并以
  `guarded_ultrasonic_verified` 继续流程；
- 其它 pose 不可用、超声跳变、前方过近等情况仍 fail-closed。

第二次 guarded 实跑：

- 日志：`logs/live_guarded_rod1_ultrasonic_override_20260611_2007.log`
- 报告：`logs/live_guarded_rod1_ultrasonic_override_20260611_2007_report.json`
- 抓料侧 guarded 超声复核通过：
  - pose lateral `-0.07325m`；
  - pose yaw `-6.67deg`；
  - 前超声采样如 `1133/1135`、`1130/1138`、`1133/1144`；
  - median/max span `11mm`，最小前方距离 `1130mm`；
  - 记录 `guarded_ultrasonic_verified` 后继续。
- 抓料 approach 成功，最终前超声稳定在约 `154/149mm`，满足 `155±10mm`。
- 抓料后 1m 后退触发并验证了 odom 尾差自动补偿：
  - 原始前超声后退后 odom 位移 `0.798m`，相对 `0.820m` 目标误差 `-0.022m`；
  - 自动补偿低速后退约 `0.0231m`；
  - 最终 odom 位移 `0.8261m`，误差 `+0.0061m`；
  - 证明 `front_ultrasonic_retreat_odom_auto_correction_*` 可以把 1m 距离拉回
    `±20mm` 门限内。
- 右转成功，实际 yaw delta `89.789deg`，误差约 `-0.211deg`。
- 放料侧 guarded 被正确阻断：
  - `rack_pose_monitor_unavailable`；
  - 一组前超声出现 `349mm` 瞬态，max span 达 `1012mm`；
  - guarded 超声复核 rejected；
  - 结论：放料侧视觉/超声瞬态还没有达到 7 根 guarded 放大条件。

为避免机械臂夹持料杆长时间停留，使用受保护恢复：

- `monitor` fallback 续跑日志：
  `logs/live_resume_rod1_place_after_guarded_pose_block_20260611_2008.log`
- 报告：
  `logs/live_resume_rod1_place_after_guarded_pose_block_20260611_2008_report.json`
- 问题：放料 approach 初始稳定前超声约 `1238/1241mm`，原固定 `15s`
  correction 在约 `504mm` 处超时，没有到 `327mm` 目标。

随后使用独立前超声目标恢复脚本：

- 日志：`logs/guarded_front_target_recovery_rod1_place_20260611_2009.jsonl`
- 起点前超声约 `508/488mm`；
- 低速前进估算约 `0.175m`；
- settle 后 median avg `329mm`，median min `320mm`；
- 最终前超声约 `338/320mm`；
- 状态为 `target_reached`。

恢复后再次从放料上方续跑：

- 日志：`logs/live_resume_rod1_place_after_front_recovery_20260611_2010.log`
- 报告：`logs/live_resume_rod1_place_after_front_recovery_20260611_2010_report.json`
- 放料 approach 进入时已经在目标窗口，前超声约 `341/323mm`；
- 放料、开爪、抽离完成；
- 放料后 1m 后退成功：
  - 起点前超声 `{0: 341, 1: 323}`；
  - 终点前超声 `{0: 1349, 1: 1331}`；
  - 双探头增量均为 `1008mm`；
  - odom 位移 `0.99476m`，误差约 `-0.005m`。
- 最终左转成功，实际 yaw delta `-89.803deg`，误差约 `0.197deg`。
- 最终状态：`charge_plug_insert_state=0`，`motion_control error_code=0`，
  PNC task state `7`，底盘停稳。

本轮追加修复：

- `guarded_front_target_recovery.py` 修复退出码：
  `target_reached` 现在和 `target_window_confirmed`、`already_in_window`
  一样返回 `0`；
- 放料/抓料前超声 correction 的最长时间改为按距离自适应：
  基于当前稳定前超声到目标前超声的距离估算，默认在 `15~35s` 之间；
- 机器人端 `py_compile` 已验证
  `industrial_7_rods_total_controller.py` 和
  `guarded_front_target_recovery.py` 均通过。

下一步建议：不要直接跑完整 7 根 guarded。先用第 2 根或再跑一次第 1 根做单根验证，
重点观察放料侧 `rack_pose_monitor_unavailable` 是否复现，以及 adaptive correction
是否能从远距离自动进到 `327mm` 前超声目标。

### 2026-06-11 第 2 根 guarded 单根实测结果

第 2 根最终已完成抓取、转向、放料、后退和回正；最终状态快照为
`charge_plug_insert_state=0`、`motion_control error_code=0`、PNC task state `7`、
`stopped=True`。

第一次第 2 根 guarded 实跑：

- 日志：`logs/live_guarded_rod2_after_adaptive_patch_20260611_2015.log`
- 报告：`logs/live_guarded_rod2_after_adaptive_patch_20260611_2015_report.json`
- 抓料侧 guarded 超声复核通过：
  - before approach pose lateral 约 `+0.009m`；
  - yaw 约 `-5.22deg`；
  - 前超声多帧左右 span 约 `7~10mm`；
  - 记录 `guarded_ultrasonic_verified` 后继续。
- 抓料 approach 成功，目标 `155±10mm`，最终稳定前超声约 `160/158mm`。
- 抓料后退时，前超声基本到位但 odom 尾差略超：
  - 原始 odom 位移 `0.799m`，目标 `0.820m`，误差约 `-0.021m`；
  - 原容差为 `±0.020m`，因此触发 odom 尾差补偿；
  - 补偿开始前 clearance 正常，后距约 `2198mm`；
  - 补偿过程中出现单次 `rear_raw=()`，被旧逻辑直接判为
    `rear_ultrasonic_unavailable` 并 fail-closed。
- 只读状态确认：底盘停稳、无运动错误、未插枪；当前前超声约 `1150mm`，
  距离目标约 `1170mm` 只差约 `15~20mm`，不应重复完整 1m 后退。

针对这次失败新增补丁：

- 机器人侧备份：
  `rack_hybrid_docking_package/industrial_7_rods_total_controller.py.bak_20260611_clearance_retry`
- 新增参数：
  `--retreat-odom-auto-correction-clearance-retry-s`，默认 `0.6s`，最大 `2.0s`；
- 行为：
  - odom 尾差补偿 clearance 检查遇到前/后超声 `unavailable` 时短重试；
  - 只有连续超过重试窗口仍不可用才 fail-closed；
  - 若读到真实距离低于 hard-min，仍立即 stop 并 fail-closed；
  - 不放宽 `front_hard_min=260mm`、`rear_hard_min=500mm`、`±20mm`、
    `±0.02m` 目标。
- 本地和机器人端 `py_compile` 均通过。

随后从“抓料后已后退”恢复，不重复抓料和 1m 抓料后退：

- 命令模式：`--resume-after-grab-retreat-index 2 --end-index 2`
- 日志：`logs/live_resume_rod2_after_grab_retreat_guarded_20260611_2023.log`
- 报告：`logs/live_resume_rod2_after_grab_retreat_guarded_20260611_2023_report.json`
- 右转成功：
  - expected delta `90.000deg`；
  - actual delta `90.111deg`；
  - error `0.111deg`。
- 放料侧 before approach guarded 通过：
  - 初始 pose 曾短暂 unavailable；
  - active_initial 重新采到 pose：lateral 约 `-0.017m`，yaw 约 `-0.704deg`；
  - 判定 `centered`，没有触发 fallback。
- 放料 approach 成功：
  - 目标 `327±30mm`；
  - 最终稳定前超声约 `352/332mm`；
  - `front_target_window_confirmed`。
- 放料、开爪、抽离完成。
- 放料后 1m 后退成功：
  - 起点前超声 `{0: 352, 1: 332}`；
  - 终点 filtered `{0: 1350, 1: 1337}`；
  - 增量 `{0: 998, 1: 1005}`，remaining `2mm`；
  - odom 位移 `0.984m`，误差 `-0.016m`，满足 `±0.020m`。
- 最终左转成功：
  - expected delta `-90.000deg`；
  - actual delta `-89.995deg`；
  - error `0.005deg`。

本轮新暴露的问题：

- 放料侧 before approach guarded 判定居中后，贴近完成后的 pose monitor 又报
  lateral 约 `-0.151m`、yaw 约 `4.641deg`；
- 当前 after approach 只做 shadow/monitor，不阻断下移和开爪；
- 这说明“靠近前居中”不能完全代表“贴近后仍居中”，下一步应增加贴近后的二次门禁：
  - 先只做 `guarded_postcheck`，用多帧 lidar + 双前超声一致性复核；
  - 仅在证据稳定且偏差明显时阻断下移/开爪；
  - 不建议此时直接进入第 3 根或完整 7 根 guarded。

### 2026-06-11 贴近后二次 guarded postcheck

已新增并同步到机器人：

- 文件：`rack_hybrid_docking_package/industrial_7_rods_total_controller.py`
- 机器人侧备份：
  `rack_hybrid_docking_package/industrial_7_rods_total_controller.py.bak_20260611_postcheck`
- 远端 dry-run：
  `logs/dryrun_post_approach_guarded_rod3_20260611.log`
- 远端 report：
  `logs/dryrun_post_approach_guarded_rod3_20260611_report.json`

新增行为：

- `rack-centering-mode=guarded/active` 时，抓料和放料 `approach_by_front_ultrasonic()`
  到目标前超声窗口后，会在下一步机械臂动作前执行
  `rack_post_approach_guarded_check`；
- 若贴近后 lidar pose 证据稳定，且横向偏移超过默认 `0.12m`，则
  fail-closed，不再继续闭爪、下移或开爪；
- 若贴近后 lidar pose 不可用，但双前超声仍安全稳定，则记录
  `inconclusive_ultrasonic_verified`，不因单纯 pose 不可用误停；
- 若贴近后双前超声不安全，例如低于当前阶段 `min_safe_mm`、有效双前样本不足、
  或左右 span 过大，则 fail-closed。

新增参数：

- `--disable-rack-post-approach-guarded-check`
- `--rack-post-approach-max-lateral-m`，默认 `0.12`
- `--rack-post-approach-max-yaw-deg`，默认 `7.0`
- `--rack-post-approach-max-front-span-mm`，默认 `80`

已验证：

- 本地 `py_compile` 通过；
- 机器人端 `py_compile` 通过；
- 本地函数级复现第 2 根贴近后读数：
  - lateral `-0.151m`；
  - yaw `4.641deg`；
  - confidence `0.844`；
  - fit residual `0.0296m`；
  - robust span `0.036m`；
  - 双前超声 `352/332mm`；
  - 结果为 `blocked_pose_offset`，会在下移/开爪前阻断；
- 本地函数级验证 pose unavailable + 双前超声安全稳定时，结果为
  `inconclusive_ultrasonic_verified`，不会误停抓料侧常见的近距离不可见场景。

下一步建议：先跑第 3 根单根 guarded。预期如果第 3 根贴近后出现类似第 2 根的
稳定横向偏差，会在机械臂动作前停住；如果 postcheck 通过，再继续观察放料后
`1m±20mm` 和 odom `±0.02m`。

## 仍建议后续继续工业化的点

- 放料后退目标恢复已有 checkpoint/resume hint 和 odom 尾差自动补偿；后续仍建议把异常恢复报告做得更直接，比如在失败报告顶部单独输出“下一条可执行 resume 命令”。
- PNC 残留任务清理、最终状态快照、`rack_pose_lateral_shadow`、`rack-centering-mode guarded/active` 保护逻辑已内置；下一步不要直接跑 7 根 active。
- 横向 active 已接入但真实横移方向默认门控关闭；下一步如果继续研究蟹行，先修底盘 `linear.y` 响应和右侧超声低值尖峰，再用 `industrial_lateral_motion_trace.py` 做单次受保护方向诊断。
- 对前超声 0/1 做安装/标定复核，特别是 150mm 到 350mm 的近距离区域；这一区域决定抓料和放料最终安全边界。
- 如果要长期无人值守，增加“每根完成后的可恢复状态包”：checkpoint、最近一次稳定传感器读数、最近一次 odom、下一条 resume 命令。
- 料架居中闭环后续升级路线：
  - 先继续跑 probe，不再直接用单根流程测试真实横移；
  - 如果 probe 能证明某个方向连续多次 `centered_after_step`，再考虑单根 `--rack-centering-mode active`；
  - 用 `analyze_rack_pose_events.py` 查看 `Lateral Active` 汇总中的 step 数、改善量和最终横向偏差；
  - 单根稳定后再扩大到 2 根，最后才考虑完整 7 根 active；
  - yaw active 仍未开放，yaw 只继续保留 shadow/监控，避免把横移问题和角度问题混在一起。

下一次完整流程结束后，在机器人项目根目录执行：

```bash
python3 rack_hybrid_docking_package/analyze_rack_pose_events.py \
  logs/本次运行.jsonl \
  --output-md logs/本次运行_rack_pose_analysis.md \
  --output-json logs/本次运行_rack_pose_analysis.json
```
