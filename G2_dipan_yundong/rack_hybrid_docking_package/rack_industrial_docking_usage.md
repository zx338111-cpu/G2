# G2 料架工业方法使用说明

这个文件说明上层业务应该怎么调用 `rack_industrial_docking.py`。它把现场动作拆成几个清晰方法：

- `forward()`：受控前进，带前方超声硬保护。
- `coarse_position()`：粗定位，用前激光雷达靠近到前方超声稳定接管区。
- `fine_position()`：精定位，用前方超声停车到目标距离。
- `retreat()`：正式后退，带后方超声保护。
- `approach()`：粗定位 + 精定位。
- `cycle()`：后退 + 只读快照 + 粗定位 + 精定位。

## 最推荐的业务写法

```python
from rack_industrial_docking import RackIndustrialDockingController


with RackIndustrialDockingController(
    front_ultrasonic_ids=(0, 1),
    rear_ultrasonic_ids=(4, 5),
) as rack:
    preflight = rack.preflight(allow_estop_pedal_fault=True)
    if preflight.status != "ok":
        raise RuntimeError(preflight)

    snapshots = rack.read_snapshots(samples=8)
    print(snapshots)

    result = rack.approach(
        allow_estop_pedal_fault=True,
        coarse_speed_mps=0.60,
        switch_ultrasonic_mm=2200,
        ultrasonic_takeover_mm=2500,
        final_stop_mm=540,
        final_brake_margin_mm=80,
        final_speed_mps=0.30,
    )
    print(result)
```

成功时：

```text
IndustrialFlowResult(flow='approach', status='completed', ...)
```

`approach()` 内部顺序：

1. `coarse_position()`：激光粗定位，直到前超声稳定。
2. `fine_position()`：前超声精定位，内部触发距离为 `540 + 80 = 620mm`。

## 分开调用四个方法

### 1. 受控前进

只适合短距离补偿或调试，不建议把它当成料架定位主流程。

```python
result = rack.forward(
    distance_m=0.20,
    speed_mps=0.05,
    front_hard_stop_mm=700,
    allow_estop_pedal_fault=True,
)
print(result)
```

状态：

- `completed`：按时间/距离估算完成。
- `front_obstacle`：前方超声小于 `front_hard_stop_mm`，已停车。
- `blocked`：底盘安全状态不允许运动。
- `error`：GDK 或 PNC 调用异常。

### 2. 粗定位

```python
coarse = rack.coarse_position(
    coarse_speed_mps=0.60,
    coarse_stop_m=1.6,
    switch_ultrasonic_mm=2200,
    ultrasonic_takeover_mm=2500,
    allow_estop_pedal_fault=True,
)
print(coarse)
```

只有 `status='ready_for_fine'` 才继续精定位：

```python
if coarse.status != "ready_for_fine":
    raise RuntimeError(coarse)
```

常见状态：

- `ready_for_fine`：前方超声已经稳定，可以进入精定位。
- `coarse_guard`：激光已到粗定位保护下限，但前超声仍不稳定，不能继续盲走。
- `lost_lidar`：粗定位阶段连续丢失激光目标。
- `target_lost`：激光目标突然跳远，疑似追到背景。
- `timeout`：粗定位超时。
- `blocked`：底盘安全状态不允许运动。

### 3. 精定位

```python
fine = rack.fine_position(
    final_stop_mm=540,
    final_brake_margin_mm=80,
    final_speed_mps=0.30,
    allow_estop_pedal_fault=True,
)
print(fine)
```

状态：

- `stopped`：成功按超声停车。
- `already_at_threshold`：启动时已经在触发距离内，没有继续前进。
- `no_front_ultrasonic_lock`：启动前没有稳定前方超声，不能盲目精停。
- `lost_radar`：精停过程中前方超声连续丢失。
- `timeout`：精停超时。

### 4. 后退

```python
retreat = rack.retreat(
    distance_m=1.0,
    speed_mps=0.50,
    method="relative",
    rear_stop_mm=700,
    rear_hard_stop_mm=500,
    rear_stop_min_sensors=2,
    allow_estop_pedal_fault=True,
)
print(retreat)
```

状态：

- `completed`：后退完成。
- `rear_obstacle`：后方超声触发保护，已停车。
- `timeout`：后退超时。
- `blocked`：底盘安全状态不允许运动。

说明：

- 这是底层后退方法，仍适合单独诊断后方障碍保护。
- 七根料一键总控当前默认不再直接用纯 `relative` 或 `velocity` 做 1m 后退，而是用 `industrial_7_rods_total_controller.py --retreat-method front-ultrasonic`。
- `front-ultrasonic` 后退的判断方式是：记录后退开始时前方 `0/1` 超声距离，后退过程中要求两个前探头距离增量都接近 `1000mm`；少退继续慢速后退，多退慢速前补，完成容差默认 `20mm`。
- `speed_mps=0.50` 是后退阶段速度上限，末段会自动降速；不要再使用固定速度乘时间来代表实际 1m。
- 如果必须使用 `method="velocity"`，需要按现场尺量结果设置制动补偿；一键工业流程默认不使用开环后退。
- 后退失败后的恢复不能重复执行完整后退动作。总控会在后退启动时写入
  `retreat_start_front_by_id`、`retreat_target_front_by_id` 和
  `retreat_target_front_avg_mm`。恢复时应按该目标前超声距离纠偏；否则少退/多退都会改变
  后续抓料或放料的平移基准。

放料后退中断或过退的恢复示例：

```bash
python3 rack_hybrid_docking_package/industrial_7_rods_total_controller.py \
  --confirm-live \
  --resume-after-place-retreat-target-index 3 \
  --place-retreat-front-target-mm 1340 \
  --end-index 7 \
  --retreat-method front-ultrasonic \
  --turn-method velocity \
  --turn-yaw-tolerance-deg 1 \
  --turn-validation-ok
```

这个入口会先把当前前超声恢复到目标窗口，再允许左转并从下一根继续；它不会重复放料、
不会重复完整 `1m` 后退。

七根料一键流程里的后退调用示例：

```bash
python3 rack_hybrid_docking_package/industrial_7_rods_total_controller.py \
  --confirm-live \
  --grab-vertical-stack \
  --retreat-method front-ultrasonic \
  --retreat-distance-m 1.0 \
  --retreat-speed-mps 0.50
```

竖排抓取说明：

- 加 `--grab-vertical-stack` 后，第 1 根作为基准 XY/姿态，第 2-7 根只做末端 Z 偏移。
- 当前料架按现场确认：第 2 根比第 1 根低，第 3 根比第 2 根低，以此类推，默认层距为 `-0.060m`。
- 如需临时覆盖层距，可传 `--grab-vertical-stack-pitch-m <层间距米>`；正值向上，负值向下。

## 七根料总控运行产物

`industrial_7_rods_total_controller.py` 现在除了文本日志，还会默认生成三类
机器可读产物，便于工业现场复盘和恢复：

```text
logs/industrial_7_rods_YYYYMMDD_HHMMSS.log
logs/industrial_7_rods_YYYYMMDD_HHMMSS.jsonl
logs/industrial_7_rods_YYYYMMDD_HHMMSS_checkpoint.json
logs/industrial_7_rods_YYYYMMDD_HHMMSS_report.json
```

- `.jsonl`：每一步开始、完成、失败的事件流。
- `_checkpoint.json`：最近停在哪根、哪一步，以及保守的恢复建议。
- `_report.json`：本次运行最终成功/失败、异常栈、关联日志路径。

也可以显式指定：

```bash
python3 rack_hybrid_docking_package/industrial_7_rods_total_controller.py \
  --dry-run \
  --start-index 1 \
  --end-index 1 \
  --log-file logs/dryrun_turn_gate.log \
  --event-file logs/dryrun_turn_gate.jsonl \
  --checkpoint-file logs/dryrun_turn_gate_checkpoint.json \
  --report-file logs/dryrun_turn_gate_report.json
```

启动时总控会先做姿态文件预检。它从
`move_arm_by_json_grab_above_第一根.py` 到
`move_arm_by_json_grab_above_第七根.py`，以及
`move_arm_by_json_grab_above_2.py` 里解析 `JSON_FILE_PATH`，再检查对应
`arm_position_to_grab_*.json` 是否存在、是否包含左右臂 14 个关节键、值是否为数字。

这个检查是为了防止原姿态脚本在 JSON 缺键时默认补 `0.0`。dry-run 也会打印每个姿态
JSON 的 `sha256` 短 hash 和修改时间，方便确认现场正在使用的是最新位姿。

## 一键工业循环

```python
result = rack.cycle(
    allow_estop_pedal_fault=True,
    retreat_distance_m=2.5,
    retreat_speed_mps=0.50,
    retreat_method="relative",
    snapshot_samples=8,
    coarse_speed_mps=0.60,
    final_stop_mm=540,
    final_brake_margin_mm=80,
)
print(result)
```

执行顺序：

1. `retreat()`：先后退，后方障碍会中止流程。
2. `read_snapshots()`：只读传感器快照。
3. `coarse_position()`：激光粗定位。
4. `fine_position()`：前超声精定位。

## 工厂现场稳定性原则

- 不把单帧传感器读数当成稳定目标，前超声需要连续有效且波动小。
- 激光粗定位只负责把目标带到超声接管区，不负责最终 `0.5m` 精停。
- 激光目标突然跳远时停车，不追背景。
- 激光短时丢一帧只在远距离允许短暂 keepalive，接近料架时不盲走。
- 精定位用 `hard_stop_mm=final_stop_mm + final_brake_margin_mm`，减少滤波延迟。
- 七根料主流程的 1m 后退必须做后退后距离校验和补偿，不能只发一次开环速度。
- 后退障碍保护必须只看真实后方 `4/5`，不能把左侧 `6/7` 混入后方保护。
- 只允许放行已知急停踏板故障；充电插入、运动错误、超声供电异常不能绕过。

## 2026-06-09 七根料总控当前参数

当前总控脚本：

```text
rack_hybrid_docking_package/industrial_7_rods_total_controller.py
```

现场确认参数：

```text
front_ultrasonic_ids=(0, 1)
right_ultrasonic_ids=(2, 3)
rear_ultrasonic_ids=(4, 5)
left_ultrasonic_ids=(6, 7)

coarse_speed_mps=0.60
grab_approach_speed_mps=0.15
place_approach_speed_mps=0.15

grab_distance_mm=155
grab_brake_margin_mm=70
grab_min_safe_mm=135
grab_target_tolerance_mm=10
grab_correction_speed_mps=0.035
grab_correction_max_passes=3
grab_angle_correction_max_span_mm=25
grab_angle_correction_max_passes=2
grab_angle_correction_angular_speed_radps=0.05
grab_angle_correction_probe_s=0.20

place_distance_mm=327
place_brake_margin_mm=60
place_min_safe_mm=280
place_target_tolerance_mm=30
place_correction_speed_mps=0.05
place_correction_max_passes=2

retreat_method=front-ultrasonic
retreat_distance_m=1.0
retreat_speed_mps=0.50
retreat_front_delta_consistency_mm=180
retreat_odom_tolerance_m=0.02
turn_method=velocity
turn_yaw_tolerance_deg=0.5
turn_confirm_samples=5
turn_confirm_interval_s=0.12
turn_confirm_max_span_deg=0.8
turn_correction_max_passes=5
turn_correction_angular_speed_radps=0.08
turn_correction_max_error_deg=25.0
```

三大核心动作已经按工业 primitive 处理：

- 抓料/放料精定位：粗定位进入超声接管，前方 `0/1` 精停，多帧停稳复核；
  抓料要求 `155±10mm` 目标窗口，太远时低速前进补近，太近时低速后退拉回。
  如果抓料停稳后 `0/1` 左右差超过 `25mm`，会做小角度试探纠偏；试探方向由实测
  左右差是否变小决定，不硬编码 0/1 左右方向。
- 后退 `1m`：默认 `front-ultrasonic` 双前探头增量闭环；能读到 SLAM odom 时，
  后退位移还要通过 `1.0±0.02m` 交叉校验；默认要求 SLAM odom 可读。
  2026-06-10 补充：带杆后退时如果前 `0/1` 超声动态分叉，控制器先停车做
  停稳复核；停稳后双前超声重新一致且退距达标则判完成。若停稳后仍分叉，
  只有单侧退距达标且 odom 位移也满足严格窗口时才允许通过，否则继续失败停机。
  放料后退触发后方保护时，会先停车复核后方超声；若复核是假触发则继续，若复核
  为真但前超声目标已经到窗口且没有后方硬近障，则按已到目标完成。
- 左右转 `90` 度：默认 `velocity`，走
  `request_chassis_control(0)+move_chassis(Twist)`，但不按固定 3 秒开环；
  控制循环实时读取 odom yaw，按误差分段降速，进入 `1deg` 容差后停车并复核。
  `relative_move` 只保留为对比诊断。实测这台底盘的 `move_chassis angular.z`
  符号与 SLAM odom yaw 符号相反：业务右转命令仍是负角速度，但 odom yaw
  目标增量为 `+90deg`；业务左转相反。

详细说明见：

```text
rack_hybrid_docking_package/industrial_three_core_primitives_20260610.md
```

2026-06-10 完整流程实测：

```text
日志：
  logs/live_full7_place327_grab70_20260610_113836.log
  logs/live_full7_place327_grab70_20260610_113836_report.json

结果：
  第 1、2 根完整通过。
  第 3 根抓料精定位、闭爪、拉出完成；抓料后 1m 后退实际接近完成，
  但运行时前超声 0/1 增量动态分叉：
    id0 delta≈975mm
    id1 delta≈587mm
  旧逻辑按双前超声一致性失败停机。

已修复：
  industrial_7_rods_total_controller.py 的 front-ultrasonic 后退逻辑已加入
  stop-and-settle 复核和单侧+odom 严格兜底。

当前现场阻塞：
  后续只读状态显示 charge_plug_insert_state=1，且有充电电压/电流；
  PNC 控制任务已取消回 state=7。
  在充电拔掉、charge_plug_insert_state=0 之前不要恢复流程。

最新只读复查：
  本次只执行 `industrial_status_snapshot.py --samples 8`，没有执行任何物理动作。
  结果仍为 charge_plug_insert_state=1，charge_plug_input_voltage=51.0，
  charge_plug_input_current=14.8，motion_control error_code=2
  error_msg='collision imminent'。机器人已停稳，odom/yaw 可读，loc_confidence=80；
  前方 0/1 稳定约 1180mm，后方 5 号约 895~907mm，4 号间歇 65535。
  结论不变：必须先拔掉充电并确认 motion_control_error=0 后再恢复。

继续复查：
  充电已拔掉，charge_plug_insert_state=0，charge_plug_input_voltage/current 都为 0。
  但 motion_control 仍为 error_code=2，error_msg='collision imminent'，因此仍不能
  执行从第 3 根恢复后的右转。当前超声并不显示近距离障碍：前方 0/1 约 1180mm，
  后方 4/5 约 1.5~2.2m，左右侧也在 1m 以上；odom/yaw 可读，stopped_check=True。
  已做过两种非位移清理：`clean_navi.py` 取消 PNC task，以及
  `request_chassis_control(0)+move_chassis(0)` 发送零速度停车帧，再取消远控任务；
  PNC 已回 state=7，但 motion_control_error=2 没有清除。
  恢复入口 dry-run 已通过：
  `--dry-run --resume-after-grab-retreat-index 3 --end-index 7 --turn-validation-ok`。
  当前阻塞不是姿态脚本或恢复参数，而是 GDK 状态接口仍报 collision imminent。

2026-06-10 13:44 继续复查：
  `motion_control_status` 首帧/短窗口读取会偶发 `MotionControlStatus message is nullptr`。
  已新增 `gdk_status_utils.py`，把 motion status 读取改成最多约 8 秒短重试；
  只要最终读到非 0 `error_code` 仍阻断，读不到也仍阻断。机器人端已单独验证
  启动预检和转向预检都能拿到 `motion_error=0`，且
  `charge_plug_insert_state=0`、`stopped_check=True`、超声供电正常。
  `industrial_status_snapshot.py` 也已接入同一重试逻辑，最新只读快照显示
  `motion_control error_code=0`。
  本轮没有执行物理恢复动作；下一步 resume 会真实转向和机械臂动作，必须先确认
  现场安全。
```

下一步命令：

```bash
source /home/agi/app/env.sh
cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1
python3 industrial_status_snapshot.py --samples 8

# 只有 charge_plug_insert_state=0、stopped_check=True、motion_control_error=0
# 且 yaw 可读后，才能从第三根抓料后已后退处恢复：
python3 rack_hybrid_docking_package/industrial_7_rods_total_controller.py \
  --confirm-live \
  --resume-after-grab-retreat-index 3 \
  --end-index 7 \
  --retreat-method front-ultrasonic \
  --turn-method velocity \
  --turn-yaw-tolerance-deg 1 \
  --turn-validation-ok \
  --log-file logs/live_resume_rod3_after_grab_retreat_YYYYMMDD_HHMM.log
```

如果重新从第一根开始跑，连续多根真实运行必须显式确认：

```bash
python3 rack_hybrid_docking_package/industrial_7_rods_total_controller.py \
  --confirm-live \
  --start-index 1 \
  --end-index 7 \
  --retreat-method front-ultrasonic \
  --turn-validation-ok \
  --log-file logs/full_7_rods_YYYYMMDD_HHMM.log
```

## 独立验证 1m 后退

如果只想验证“每次后退 1m 必须在正负 20mm 内”，不要启动完整七根料流程，先用单项脚本：

```bash
source /home/agi/app/env.sh
cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1

python3 rack_hybrid_docking_package/industrial_retreat_1m_validation.py \
  --dry-run \
  --base-dir /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1 \
  --log-file logs/dryrun_retreat_1m_validation_YYYYMMDD_HHMM.log
```

现场确认后方安全后，才可实退：

```bash
python3 rack_hybrid_docking_package/industrial_retreat_1m_validation.py \
  --confirm-live \
  --base-dir /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1 \
  --distance-m 1.0 \
  --tolerance-mm 20 \
  --odom-tolerance-m 0.02 \
  --log-file logs/live_retreat_1m_validation_YYYYMMDD_HHMM.log
```

该脚本只调用总控同一个 `front-ultrasonic` 后退闭环，不跑机械臂、不靠近料架、不转向。
实退通过的证据以 `front_ultrasonic_retreat_target_verified` 事件为准，同时要看到
`retreat_target_tolerance_mm=20` 和 `retreat_odom_tolerance_m=0.02`。
