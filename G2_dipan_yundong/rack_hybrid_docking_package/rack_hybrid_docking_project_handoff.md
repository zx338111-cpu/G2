# G2 料架相对靠近项目交接文档

日期：2026-06-05

## 一句话结论

这个项目实现的是：G2 机器人面对上料架/下料架时，不让用户输入移动距离，而是用传感器实时判断相对距离，自动靠近并最终停在料架前约 `0.5m`。

当前已经实机验证的方案是：

1. 远距离用前激光雷达识别料架高处结构，做粗靠近。
2. 靠近到前方超声波能稳定识别后，切换到超声精停。
3. 前进精停使用官方确认可用的第二种方法：`Pnc.request_chassis_control(0)` + `Pnc.move_chassis(Twist)`；90 度转向和精确后退使用 `Pnc.relative_move(...)` 并监控新任务 state。
4. 最终目标距离仍写 `500mm`，但程序内部加入 `80mm` 制动补偿，会在 `580mm` 左右触发停车，停稳后接近实际 `500mm`。

## 2026-06-08 稳定化更新

现场出现过“激光 ROI 稳定看到约 `0.65m` 近点，但前方超声稳定看到约
`2.2m` 目标面”的复杂情况。该近点现场确认不是危险障碍，如果继续依赖
激光粗靠近，会反复触发 `coarse_stopped`。

当前代码已改成稳定策略：

- demo 默认目标为 `final_stop_mm=540`，`final_brake_margin_mm=80`，
  内部触发距离 `620mm`。
- 常规超声切换阈值为 `switch_ultrasonic_mm=2200`。
- 新增稳定超声优先接管：`ultrasonic_takeover_mm=2500`，
  `ultrasonic_stable_tolerance_mm=250`。前方超声连续稳定在 2.5m 内时，
  直接进入超声精停，不再因为激光误抓近处非危险结构而粗停。
- 超声接管和超声精停启动阶段都要求连续有效回波。远距离偶发一两帧
  假回波不能累积成“稳定目标”，避免过早进入 final 后 `lost_radar`。
- 早期完整默认执行复查：曾用 `Twist.linear.x=-0.25m/s` 后退 `10s`
  约 `2.5m`；只读激光看到约 `2.86~3.10m`，超声大多无回波；默认
  demo 先激光粗靠近 `17` 帧，再切换到超声精停，最终
  `HybridDockingResult(status='stopped', stage='final_ultrasonic')`。
  停稳后只读超声约 `531~534mm`。

后续现场默认运行命令保持：

```bash
python3 rack_hybrid_docking_demo.py --execute --allow-estop-pedal-fault
```

不要再通过每次临时改 `--switch-ultrasonic-mm` 来绕过现场波动。

## 2026-06-09 夜间七根料总控交接

现场目标流程已经固化为七根料循环：

```text
每一根：
  张开夹爪
  移动到第 N 根抓取上方
  前方 0/1 超声靠近到抓料距离
  闭合夹爪
  拉出
  后退 1m
  右转 90 度
  移动到放料上方
  前方 0/1 超声靠近到放料距离
  下移
  张开夹爪放料
  拉出
  后退 1m
  左转 90 度
  进入下一根
```

当前一键脚本：

```text
rack_hybrid_docking_package/industrial_7_rods_total_controller.py
```

机器人端路径：

```text
/data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1/rack_hybrid_docking_package/industrial_7_rods_total_controller.py
```

推荐实机命令：

```bash
source /home/agi/app/env.sh
cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1
python3 rack_hybrid_docking_package/industrial_7_rods_total_controller.py \
  --confirm-live \
  --allow-estop-pedal-fault \
  --start-index 1 \
  --end-index 7 \
  --retreat-method front-ultrasonic \
  --turn-method velocity \
  --log-file logs/full_7_rods_YYYYMMDD_HHMM.log
```

今天已经验证和修正的内容：

- 超声方向按现场贴标确认：`0/1` 前方，`2/3` 右侧，`4/5` 后方，`6/7` 左侧。
- 抓料定位默认目标为 `155mm`，精定位速度 `0.15m/s`，提前停车补偿保持 `70mm`，触发距离 `225mm`，停稳窗口 `155±10mm`，硬安全下限 `135mm`；停稳后如果太远会低速前进补近，太近会低速后退拉回；若前 `0/1` 左右差超过 `25mm`，会先做小角度试探纠偏，方向按实测差值是否变小自动选择。
- 竖排抓取可用 `--grab-vertical-stack` 启用：第 1 根作为基准 XY/姿态，第 2-7 根只做末端 Z 偏移；现场确认第 2 根比第 1 根低，默认层距 `-0.060m`。
- 放料定位当前默认目标已从 `500mm` 改为现场只读超声实测的 `327mm`，精定位速度 `0.15m/s`，提前停车补偿 `60mm`，触发距离 `387mm`，停稳窗口 `327±30mm`，硬安全下限 `280mm`，低速补近速度 `0.05m/s`，最多补近 `2` 次。
- 放料下移动作 `offset_move_down.py` 当前机器人端实际参数为 `Z=-0.12m`；后续以机器人端脚本为准，不再按旧 `-0.06m` 文档执行。
- 后退主流程改为 `front-ultrasonic`：以开始时前方 `0/1` 超声距离为基准，目标是两个前探头距离都增加 `1000mm`；少退继续慢速后退，多退慢速前补，完成容差 `20mm`。放料后退触发后方保护时会先停车复核，假触发继续，若前超声目标已到窗口且没有后方硬近障则按完成处理。这解决了纯速度开环后退有时多、有时少的问题。
- 后退速度上限按现场要求保留 `0.50m/s`，末段自动降速；粗定位速度上限 `0.60m/s`。
- `RackIndustrialDockingController.read_snapshot()` 已容错前激光 `LatestPointCloud` 偶发空帧：只读快照阶段点云空帧不再阻断基于前超声的精定位和后退；真正需要激光的粗定位阶段仍按自身逻辑处理丢点。

今天实机完整流程跑到第 1 根结束，第 2 根抓取靠近阶段时被现场人工反馈打断，原因不是底盘撞停，而是左转 90 度现场观感仍不可靠。

关键实机证据：

```text
第 1 根抓料定位：
  目标 170mm，停稳多帧确认 stable_raw=((0, 180), (1, 153))，最小 153mm。

第 1 根抓取后后退 1m：
  start_front_by_id={0:180, 1:150}
  final front_filtered_by_id={0:1175, 1:1126}
  delta_by_id={0:995, 1:976}
  remaining_mm=24
  status=completed

第 1 根右转 90 度：
  使用 velocity 转向，duration=3.00s，angular_z=-0.524rad/s。
  只能用超声场景变化校验，没有 yaw/里程计闭环。

第 1 根放料定位：
  目标 500mm，停稳 stable_raw=((0, 503), (1, 492))，平均约 498mm。

第 1 根放料后后退 1m：
  start_front_by_id={0:571, 1:480}
  final front_filtered_by_id={0:1563, 1:1557}
  delta_by_id={0:992, 1:1077}
  remaining_mm=8
  status=completed

第 1 根左转 90 度：
  使用 velocity 转向，duration=3.00s，angular_z=+0.524rad/s。
  日志显示超声场景变化通过，但现场反馈“左转90度还是不行”。
```

当前未解决的首要问题：

- 左转 90 度不能只靠 `3.00s * 0.524rad/s` 的速度开环和超声场景变化判断。超声变化只能证明“机器人动了/场景变了”，不能证明 yaw 到了 `90` 度。
- `get_slam_state`、`get_odom_info`、`get_curr_pose` 当前都曾返回失败，导致总控没有可靠 yaw 闭环来源。明天优先要先解决 yaw 反馈或做独立角度标定，不要直接再跑完整 7 根。
- 如果继续用 velocity 开环，至少需要单独测试左转/右转多次，分别标定 `left_turn_duration_s` 和 `right_turn_duration_s`，并记录现场实际角度；不能把 `3.00s` 当工业级 90 度。

## 2026-06-10 转向诊断更新

今天先按安全要求做了只读状态确认，没有跑完整 7 根：

```text
industrial_status_snapshot.py --samples 8

stopped_check odom_available=true max_linear_speed_mps=0.0000 threshold_mps=0.0200 stopped=True
PNC task_state: id=101, state=7, type=3
motion_control: mode=5, error_code=0
chassis_ultrasonic_radar_power_state=1
charge_plug_insert_state=1
emergency_stop_pedal_fault_state=1 allowed

8 个超声方向仍按现场贴标：
  0/1 前，2/3 右，4/5 后，6/7 左。
  本次 8 帧前方约 1605~1620mm，后方多帧约 1505~1558mm，
  左侧 6 号偶发 65535/约 680mm，后续只作为侧向诊断，不并入前后控制。
```

关键变化：

- `Slam.get_odom_info()` 今天可用，并能读到 `orientation_euler` yaw，所以转向不再只依赖超声场景变化。
- `charge_plug_insert_state=0` 后，实测 `Pnc.relative_move(yaw=-90)` 会提交新任务并短暂进入 `state=2`，随后卡在 `state=8`，超时失败；转向前 yaw 约 `-0.055deg`，失败后仍约 `+0.1deg`，说明底盘没有真正转起来。这个路径不能作为生产默认。
- 已新增/更新 `industrial_turn_diagnostic.py`：默认 `--method velocity`，真实转向前检查 `charge_plug_insert_state`、`motion_control_error`、超声供电和急停踏板故障；转向过程中实时读取 odom yaw，按误差分段降速，进入 `1deg` 容差后停车并复核。
- 已更新 `rack_hybrid_docking_package/industrial_7_rods_total_controller.py`：七根料总控里的右转/左转也改为同一套 `request_chassis_control(0)+move_chassis(Twist)` 的 odom yaw 闭环，默认 `--turn-method velocity --turn-yaw-tolerance-deg 1`。即使底盘命令返回正常，如果 yaw 实际不到 90 度，也不会继续下一步。
- 2026-06-10 实测 `move_chassis angular.z` 与 SLAM odom yaw 符号相反：发 `wz=-0.524` 时，odom yaw 从 `0.1deg` 增加到约 `65deg`。因此业务右转仍发负角速度，但 velocity 闭环的 odom 目标增量是 `+90deg`；业务左转相反。
- 充电未拔时仍会被诊断脚本阻断：

```text
RuntimeError: turn preflight blocked: charge_plug_insert_state=1
```

下一步必须先重新读状态，确认 `charge_plug_insert_state=0`、`stopped_check=True`、odom yaw 可读后，再单独测试左右转。

最新只读复查仍显示 `charge_plug_insert_state=1`，充电输入约 `51.0V/14.8A`，
`motion_control error_code=2` 且 `error_msg='collision imminent'`。机器人已停稳、
odom/yaw 可读、`loc_confidence=80`，但在充电拔掉并恢复 `motion_control_error=0`
之前仍不能执行恢复、转向或七根料物理流程。

继续复查后，充电已经拔掉：`charge_plug_insert_state=0`，
`charge_plug_input_voltage=0`，`charge_plug_input_current=0`。恢复入口
`--dry-run --resume-after-grab-retreat-index 3 --end-index 7 --turn-validation-ok`
已经在机器人端通过，说明第 3 根恢复路径和姿态 JSON 预检没问题。
但 `motion_control error_code=2` / `collision imminent` 仍未清除；前方 0/1
约 `1180mm`、后方约 `1.5~2.2m`、机器人停稳且 odom/yaw 可读。已尝试取消
PNC task、发送零速度 `move_chassis`、再取消远控任务，PNC 回到 `state=7`，
但 error=2 仍存在。当前不能绕过这个门禁去做右转或恢复七根料流程。

2026-06-10 13:44 继续复查更新：`motion_control_status` DDS/GDK 首帧会偶发
`MotionControlStatus message is nullptr`，但延长读取窗口后能拿到
`motion_error=0`。已新增 `gdk_status_utils.py`，并把启动预检、转向预检、前超声
后退预检以及底层激光/超声控制器的 `get_motion_control_status()` 改为短重试。
安全门槛没有放宽：若最终仍读不到状态，或读到非 0 `error_code`，仍会阻断运动。
机器人端已验证：

```text
startup_live_preflight motion_error=0 charge_plug_insert_state=0 ...
turn_preflight motion_error=0 charge_plug_insert_state=0 ... problems=()
industrial_status_snapshot motion_control error_code=0, stopped_check=True
```

本轮没有执行任何物理恢复动作。下一步只有在现场确认夹持、料架、人员和底盘周边
安全后，才从第 3 根抓取后已后退处恢复。

建议下一步命令：

```bash
source /home/agi/app/env.sh
cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1
python3 industrial_status_snapshot.py --samples 8

# 只有 charge_plug_insert_state=0、stopped_check=True 且 yaw 可读后，才做物理转向：
python3 industrial_turn_diagnostic.py --confirm-live --direction right --method velocity --repeat 1
python3 industrial_turn_diagnostic.py --confirm-live --direction left  --method velocity --repeat 1

# 单次左右都通过后，再做交替多次；不要再用 relative 作为默认生产路径。
```

完成左右转多次单测前，不建议直接执行完整 7 根一键流程。

### 2026-06-10 现场转向闭环实测结果

先完成两组左右交替单测，未跑完整 7 根。注意：这组是旧 `3deg` 容差下的结果，实际只有 `88.xdeg`，不应作为最终 90 度合格判据：

```text
left  #1: expected_delta=-90.000deg, actual_delta=-88.797deg, error=+1.203deg
right #1: expected_delta=+90.000deg, actual_delta=+88.435deg, error=-1.565deg
left  #2: expected_delta=-90.000deg, actual_delta=-88.505deg, error=+1.495deg
right #2: expected_delta=+90.000deg, actual_delta=+88.356deg, error=-1.644deg
```

结论：

- `relative_move(yaw=±90)` 已确认不能作为当前机器人生产转向链路：右转任务进入 `state=8`，yaw 基本不动。
- `velocity` 闭环转向已通过两组左右交替测试，但用户现场指出不是严格 90 度；因此默认容差已从 `3deg` 收紧到 `1deg`，低速段从 `0.20rad/s` 收到 `0.08rad/s`，并要求连续 3 帧进入容差后才停车。
- 诊断脚本已补齐旧 PNC 任务清理：每次申请远控前会取消非终态 `task_state=2/id=2/type=3`，避免 `Task is not in IDLE...` 导致 request 失败。
- 仍不要直接跑完整 7 根；下一步建议只跑 `--start-index 1 --end-index 1` 单根实流程，重点看抓料 155mm、后退 1m、放料 327mm 三个闭环 primitive 的日志。
- 最终状态已读回：`charge_plug_insert_state=0`、`motion_error=0`、`loc_confidence=80`、`stopped_check=True`。最后状态采样中前超声 0 有一次 `559mm` 单帧跳变，其他帧约 `1400mm`；后续抓放料必须继续依赖多帧稳定和双前探头一致性门禁，不能退回单帧 min 判据。

收紧后又完成一组 `1deg` 容差左右转验证：

```text
left  strict #1: expected_delta=-90.000deg, actual_delta=-90.055deg, error=-0.055deg
right strict #1: expected_delta=+90.000deg, actual_delta=+89.853deg, error=-0.147deg
```

这组结果才作为当前 90 度转向合格基线。最终读回状态：
`charge_plug_insert_state=0`、`motion_error=0`、`loc_confidence=80`、
`stopped_check=True`。

### 2026-06-10 单根实跑结果

已执行：

```bash
python3 rack_hybrid_docking_package/industrial_7_rods_total_controller.py \
  --confirm-live --start-index 1 --end-index 1 \
  --retreat-method front-ultrasonic \
  --turn-method velocity --turn-yaw-tolerance-deg 1 \
  --log-file logs/live_single_rod1_1deg_20260610.log
```

结果摘要：

- 抓料前精定位命中：`stable_raw=((0, 170), (1, 171))`，目标 `170mm`。
- 抓取后后退 1m：前超声增量 `{0: 981, 1: 991}mm`，闭环完成。
- 右转 90 度：`actual_delta=89.306deg`，`error=-0.694deg`，1 度容差内。
- 放料前靠近：停在 `stable_raw=((0, 556), (1, 561))`，平均约 `558mm`。这只是在旧 `500±60mm` 窗口边缘，不满足“精定位到 500mm”的工程目标。
- 放料后后退 1m：前超声增量 `{0: 999, 1: 987}mm`，闭环完成；过程中出现过前 0/1 增量不一致的瞬时样本，但最终双探头一致收敛。
- 左转 90 度：`actual_delta=-89.337deg`，`error=+0.663deg`，1 度容差内。

已根据这次实跑修正：

- 放料补近不再补到 `target+tolerance`，而是补到业务目标本体。
- 当时曾把放料窗口从 `500±60mm` 收紧为 `500±30mm`；该旧目标已被后续现场确认的 `327±30mm` 覆盖。
- `move_ee_pose_close_2.py` 的实际闭合值仍为 `position=0`，但脚本文案从“张开成功”修正为“闭合成功”；open 脚本为 `position=-0.785`。

下一次不建议直接 7 根；如果现场第一根工位已经复位，优先再跑单根，确认放料停稳进入最新目标 `327±30mm`。

### 2026-06-10 放料精定位目标更新

现场只读前超声复测 8 帧，前方 `0/1` 稳定在约 `327~332mm`：

```text
front id0=327mm
front id1=329~332mm
front min=327mm
stopped_check=True
```

用户确认这是放料时精定位需要使用的距离。总控默认已更新：

```text
place_distance_mm=327
place_brake_margin_mm=60
place_trigger_mm=387
place_min_safe_mm=280
place_target_tolerance_mm=30
```

注意：这段是当时只改放料前方精定位目标的历史记录；当前抓料目标后续已按现场反馈改为 `155mm`。

### 2026-06-10 最新姿态脚本复核

已按机器人端最新文件重新读取，不再使用旧 `move_arm_magnet_*` 姿态脚本。

工业总控当前抓取姿态来源：

```text
move_arm_by_json_grab_above_第一根.py -> arm_position_to_grab_第一根.json
move_arm_by_json_grab_above_第二根.py -> arm_position_to_grab_第二根.json
move_arm_by_json_grab_above_第三根.py -> arm_position_to_grab_第三根.json
move_arm_by_json_grab_above_第四根.py -> arm_position_to_grab_第四根.json
move_arm_by_json_grab_above_第五根.py -> arm_position_to_grab_第五根.json
move_arm_by_json_grab_above_第六根.py -> arm_position_to_grab_第六根.json
move_arm_by_json_grab_above_第七根.py -> arm_position_to_grab_第七根.json
move_arm_by_json_grab_above_2.py      -> arm_position_to_grab_2.json
```

机器人端 dry-run 已确认读取到最新 8 个姿态 JSON，并记录 hash/mtime：

```text
第一根 aee10086fabf 2026-06-10T09:03:51
第二根 5a14abf64030 2026-06-10T09:30:31
第三根 b24e407abf73 2026-06-10T09:33:35
第四根 de3ae5d2d901 2026-06-09T13:34:35
第五根 42c3e9efa36e 2026-06-09T13:41:38
第六根 74a610af7b1a 2026-06-09T13:49:13
第七根 64201f14b35e 2026-06-09T13:53:22
放置   fe674bf9b142 2026-06-10T09:09:12
```

总控启动前已新增姿态 JSON 预检：每个 by-json 姿态必须存在并包含左右臂 14 个关节键，否则直接失败停机，避免原子脚本缺键时默认补 `0.0` 造成错误姿态。

当前 offset 脚本实际参数：

```text
offset_move_pull.py: X=-0.15m
offset_move_down.py: Z=-0.12m
offset_move_up.py:   X=+0.02m, Z=+0.02m
```

注意：`main_controller_v2.py` 和当前工业总控流程都没有调用 `offset_move_up.py`；已经读取该文件，但未擅自插入流程。若现场需要放料后上抬避碰，合理插入点是“张开夹爪放料”之后、“放料后拉出”之前。

`main_controller_v2.py` 已读取，当前流程包含 `move-ready1.py`、`move-pick1.py`、`move-adjust1.py`、`move-put1.py` 的地图点到点步骤。但它的后退和旋转仍不是已验证的前超声/odom 闭环，并且子脚本非零返回时会记录错误后继续执行后续步骤；因此暂不把它作为生产主线替换当前工业总控。当前推荐策略是：地图点到点可作为粗移动候选，抓料 `155mm`、放料 `327mm`、后退 `1m`、左右 `90deg` 仍由工业总控闭环 primitive 执行。

### 工业循环后退能力

2026-06-08 已新增 `rack_retreat_controller.py`，后续又把总控默认后退改成前超声闭环：

- 生产默认 `--retreat-method front-ultrasonic`：用前方 `0/1` 从靠近料架后的距离增量闭环证明后退约 `1m`。
- 前方 `0/1` 增量差超过 `180mm` 直接停机，避免斜退或单探头误读继续执行。
- SLAM odom 默认必须可读，并用 `1.0±0.02m` 做交叉校验；读不到或超出窗口都停机，不继续放宽超声判据。
- 退出时无论正常完成、异常还是障碍触发，都会补发零速度。

可单独后退：

```bash
python3 rack_hybrid_docking_demo.py --retreat --allow-estop-pedal-fault
```

也可跑一键工业循环：

```bash
python3 rack_hybrid_docking_demo.py --cycle --allow-estop-pedal-fault
```

`--cycle` 的顺序是：受控后退、只读快照、自动靠近、超声精停。后退未完成
时不会继续执行靠近。

## 当前项目文件

本地路径：`/home/davie/G2`

机器人路径：`/home/agi`

核心文件：

- `rack_hybrid_docking.py`：主控制类，推荐业务入口。
- `rack_hybrid_docking_demo.py`：命令行 demo，可直接实机运行。
- `rack_hybrid_docking_usage.md`：简版使用说明。
- `rack_lidar_docking.py`：前激光雷达粗靠近距离提取和速度控制。
- `rack_radar_docking.py`：前方超声精停控制。
- `rack_hybrid_docking_project_handoff.md`：当前交接文档。

这些文件已经同步到机器人端 `/home/agi/`，其中新增制动补偿后的版本也已经在机器人端通过 `py_compile`。

## 底盘控制链路

前进靠近当前使用：

```python
pnc.request_chassis_control(0)
pnc.move_chassis(twist)
```

90 度转向和精确后退当前使用：

```python
pnc.relative_move(req)
```

要求：提交前记录旧 task，提交后必须看到新 task 或运行态，最终只接受 `state=3/9`；`state=7` 不能当成功。

速度方向约定：

- `Twist.linear.x > 0`：机器人向当前车头方向前进，靠近料架。
- `Twist.linear.x < 0`：机器人后退，远离料架。
- 不发送横移 `linear.y`，不发送旋转 `angular.z`，避免靠近料架时发生横向漂移或转向。

## 传感器约定

前方超声波 ID：

```text
0, 1
```

右侧超声波 ID：

```text
2, 3
```

后方超声波 ID：

```text
4, 5
```

左侧超声波 ID：

```text
6, 7
```

2026-06-09 现场实机贴标确认：`0/1` 是车头前方，`2/3` 是机器人右侧，`4/5` 是车尾后方，`6/7` 是机器人左侧。旧的 `0,1,2,3` 作为前方、`4,5,6,7` 作为后方的分组是错误的，会把侧向探头误并入前后保护，导致后退/靠近误判。

前激光雷达点云坐标：

- 前进方向：raw `+X`
- 横向：raw `Y`
- 高度：raw `Z`

粗靠近使用的点云 ROI：

- 前向范围：`0.8m ~ 6.0m`
- 横向范围：`|Y| <= 0.8m`
- 高度范围：`0.6m <= Z <= 1.2m`
- 分箱宽度：`0.25m`
- 稳定点簇阈值：至少 `20` 个点

2026-06-08 现场确认激光 ROI 内会长期出现约 `0.65m` 的近处非危险点簇。
默认前向起点已从 `0.2m` 提到 `0.8m`。排掉该近点后，后退约 `2.5m`
时激光能稳定看到约 `2.9~3.1m` 的真实远处目标。

毫米波雷达说明：

用户提到 G2 上有毫米波雷达，也提供了 GDK 地址和文档路径。但在当前 GDK Python 可用接口里，没有找到明确的毫米波 API。当前生产可跑方案使用的是前激光雷达 + 前方超声波。

## 当前默认参数

主入口 `RackHybridDockingController.approach_to_rack()` 当前默认参数：

```python
coarse_speed_mps=0.60
final_speed_mps=0.30
final_stop_mm=500
final_brake_margin_mm=80
switch_ultrasonic_mm=1800
coarse_stop_m=1.6
coarse_hz=10.0
coarse_dropout_keepalive_s=0.3
final_hz=10.0
```

参数含义：

- `coarse_speed_mps=0.60`：激光雷达粗靠近速度，当前工业感较好。
- `final_speed_mps=0.30`：超声精停速度，速度较快，会有制动惯性。
- `final_stop_mm=500`：业务目标，表示希望停稳后距离料架约 `0.5m`。
- `final_brake_margin_mm=80`：制动补偿。内部实际触发距离为 `500 + 80 = 580mm`。
- `switch_ultrasonic_mm=1800`：前方超声滤波值进入 `1.8m` 后，切换到超声精停。
- `coarse_stop_m=1.6`：激光雷达粗靠近保护下限；到这里还没有稳定超声就停车。
- `coarse_hz=10.0`：粗靠近速度指令刷新频率。之前 `5Hz` 在 `0.60m/s` 下有顿挫，已改成 `10Hz`。
- `coarse_dropout_keepalive_s=0.3`：激光偶发丢一帧时，最多保持当前速度 `0.3s`，避免“走一下、停一下”的抖动。

## 为什么需要制动补偿

原始逻辑把 `500mm` 直接作为停车触发阈值：

```text
超声最小距离 <= 500mm -> 发零速度停车
```

实测用户用尺量机器人到障碍物约 `430mm`。原因是 `0.30m/s` 精停时，底盘从收到零速度到完全停稳有惯性，触发时接近 `500mm`，停稳后会继续向前走约 `70~90mm`。

现在逻辑改成：

```text
期望停稳距离 final_stop_mm = 500mm
制动补偿 final_brake_margin_mm = 80mm
内部触发距离 final_trigger_mm = 580mm
超声最小距离 <= 580mm -> 发零速度停车
```

最近一次实机验证：

```text
final_target_mm=500
final_trigger_mm=580
final_brake_margin_mm=80
触发停车附近最小原始超声：554mm
停稳后超声：0号=507mm, 1号=481mm
motion_error=0
```

这个结果已经把之前约 `430mm` 的实际距离拉回到接近 `500mm`。

## 直接运行

机器人端先加载环境：

```bash
source /home/agi/app/env.sh
```

只读查看当前激光雷达和超声波：

```bash
python3 /home/agi/rack_hybrid_docking_demo.py --read-only --samples 8
```

执行自动靠近：

```bash
python3 /home/agi/rack_hybrid_docking_demo.py --execute --allow-estop-pedal-fault
```

带完整显式参数执行：

```bash
python3 /home/agi/rack_hybrid_docking_demo.py \
  --execute \
  --allow-estop-pedal-fault \
  --coarse-speed 0.60 \
  --final-speed 0.30 \
  --final-stop-mm 500 \
  --final-brake-margin-mm 80 \
  --switch-ultrasonic-mm 1800 \
  --coarse-stop-m 1.6 \
  --coarse-max-duration 90 \
  --final-max-duration 60
```

demo 运行时会先打印：

```text
final_target_mm=500 final_trigger_mm=580 final_brake_margin_mm=80
```

看到这行，说明当前跑的是带补偿的新版本。

## 代码调用方式

业务代码建议直接调用主类：

```python
from rack_hybrid_docking import RackHybridDockingController

with RackHybridDockingController(front_ultrasonic_ids=(0, 1)) as dock:
    result = dock.approach_to_rack(
        coarse_speed_mps=0.60,
        final_speed_mps=0.30,
        final_stop_mm=500,
        final_brake_margin_mm=80,
        switch_ultrasonic_mm=1800,
        coarse_stop_m=1.6,
        allow_estop_pedal_fault=True,
    )

print(result)
```

返回结果示例：

```text
HybridDockingResult(
    status='stopped',
    stage='final_ultrasonic',
    elapsed_s=5.98,
    lidar_filtered_m=None,
    ultrasonic_filtered_mm=1455,
    final_status='stopped',
    coarse_samples=0,
    final_samples=38
)
```

`stage='final_ultrasonic'` 表示最终是由超声精停完成停车。

## 典型流程

### 距离较远，大约 2m 到 3m

1. 启动后前方超声可能没有回波。
2. 程序进入前激光雷达粗靠近。
3. 激光高处点簇稳定下降。
4. 前方超声滤波距离小于 `1800mm` 后，切换到超声精停。
5. 超声精停内部在 `580mm` 左右触发停车，停稳后接近 `500mm`。

### 距离较近，大约 1m 到 1.5m

1. 启动时前方超声已经能看到料架。
2. 程序直接进入超声精停。
3. 不会跑激光粗靠近。
4. 停稳距离由 `final_stop_mm + final_brake_margin_mm` 控制。

## 实测记录

### 0.10/0.08 低速版本

早期验证：

- 粗靠近速度：`0.10m/s`
- 精停速度：`0.08m/s`
- 可稳定停在 `0.45m ~ 0.47m`，但整体速度太慢，不符合工业场景。

### 0.60/0.30 初版高速版本

提高速度后：

- 粗靠近速度：`0.60m/s`
- 精停速度：`0.30m/s`
- 能稳定完成任务
- 但 `500mm` 直接触发停车时，用户实测停稳约 `430mm`

### 粗靠近抖动修正

现象：

- 粗靠近速度 `0.60m/s` 时，运动中有抖动。

原因：

- 激光点簇偶发单帧不足，原逻辑一帧识别不到就立刻 `stop()`。
- 下一帧识别回来又发 `0.60m/s`。
- 表现为“走、停、走”的控制抖动。

修正：

- 粗靠近控制频率从 `5Hz` 改成 `10Hz`。
- 增加 `coarse_dropout_keepalive_s=0.3`，只对短暂单帧丢失保持速度。
- 连续丢失或接近切超声区域时仍停车。

结果：

- 粗靠近日志连续。
- 现场观察稳定性明显好转。

### 500mm 停稳补偿

现象：

- 用户尺量约 `430mm`，而不是设置的 `500mm`。

修正：

- 加入 `final_brake_margin_mm=80`。
- 内部触发距离从 `500mm` 改为 `580mm`。

最近一次结果：

```text
final_target_mm=500 final_trigger_mm=580 final_brake_margin_mm=80
触发停车附近最小原始超声：554mm
停稳后：0号=507mm, 1号=481mm
motion_error=0
```

结论：

- 该补偿基本解决了最终距离偏近的问题。
- 如果现场尺量仍偏近，可以把 `final_brake_margin_mm` 调到 `100`。
- 如果现场尺量偏远，可以把 `final_brake_margin_mm` 调到 `60`。

## 安全和已知状态

当前 G2 上的已知状态：

```text
emergency_stop_pedal_fault_state=1
```

这是现场已知硬件问题，用户说明官方确认可以用第二种底盘运动方法继续测试。所以运行 demo 时需要：

```bash
--allow-estop-pedal-fault
```

但以下状态必须正常：

```text
charge_plug_insert_state=0
emergency_stop_pedal_state=0
motion_error=0
chassis_ultrasonic_radar_power_state=1
```

测试后常用状态检查脚本：

```bash
python3 - <<'PY'
import time, agibot_gdk
agibot_gdk.gdk_init()
r = agibot_gdk.Robot()
p = agibot_gdk.Pnc()
uss = agibot_gdk.UltrasonicRadar()
time.sleep(0.5)
rows = []
for row in uss.get_latest_ultrasonic_radar().get('ultrasonic_radar_datas', []):
    if row.get('fault_state') == 0 and 50 <= row.get('distance_mm') < 65535:
        rows.append((row.get('id'), row.get('distance_mm')))
t = p.get_task_state()
power = r.get_chassis_power_state()
motion = r.get_motion_control_status()
print('uss_valid', rows)
print('task_after', t.id, t.state, t.type)
print(
    'safety_after',
    'charge=', power.charge_plug_insert_state,
    'estop=', power.emergency_stop_pedal_state,
    'estop_fault=', power.emergency_stop_pedal_fault_state,
    'ultra_power=', power.chassis_ultrasonic_radar_power_state,
    'motion_error=', motion.error_code,
    'motion_mode=', motion.mode,
)
uss.close_ultrasonic_radar()
agibot_gdk.gdk_release()
PY
```

## 后退测试说明

项目核心类只负责“向前靠近料架并停在目标距离”。现场为了重复测试，会先让机器人后退 `1m` 或 `2.5m`。

后退测试使用的原则：

- 用 `Pnc.relative_move(x=-distance_m)` 做相对位移闭环后退。
- 后方 `4/5` 超声只负责障碍保护，不再用速度时间估算作为主流程距离基准。
- 用后方超声 `4,5` 做防撞保护。
- 后方最近距离小于等于 `350mm` 时提前停车。

这部分目前是测试脚本逻辑，没有封装到主业务类里。原因是生产靠近任务不应该先默认倒车；倒车应由现场测试流程或上层调度决定。

## 调参建议

### 实际停稳距离偏近

现象：

```text
尺量 < 500mm
```

处理：

```bash
--final-brake-margin-mm 100
```

或者在代码调用中：

```python
final_brake_margin_mm=100
```

### 实际停稳距离偏远

现象：

```text
尺量 > 550mm
```

处理：

```bash
--final-brake-margin-mm 60
```

### 精停段仍觉得太快

处理：

```bash
--final-speed 0.20
```

代价：

- 精停段速度更慢。
- 停稳距离更容易控制。
- 工业节拍会略降。

### 粗靠近仍有抖动

先检查日志中 `coarse` 行是否连续。如果连续但现场仍抖，可能是底盘速度阶跃造成的，不是激光丢帧。

后续可加速度斜坡：

```text
0.20 -> 0.40 -> 0.60
```

当前未实现速度斜坡，因为 `10Hz + keepalive` 后现场效果已经明显改善。

## 当前可交付状态

当前状态可以交给下一个人直接上手。机器人端项目路径：

```bash
/data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1/rack_hybrid_docking_package
```

明天接续时先执行：

```bash
cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1/rack_hybrid_docking_package
source /home/agi/app/env.sh
```

只读确认传感器，不运动：

```bash
python3 rack_hybrid_docking_demo.py --read-only --samples 8
```

只靠近料架：

```bash
python3 rack_hybrid_docking_demo.py --execute --allow-estop-pedal-fault
```

只后退，带后方超声保护：

```bash
python3 rack_hybrid_docking_demo.py --retreat --allow-estop-pedal-fault
```

一键工业循环：

```bash
python3 rack_hybrid_docking_demo.py --cycle --allow-estop-pedal-fault
```

当前已经实现并同步的工业稳定化能力：

- 前激光 ROI 默认从 `0.8m` 开始，排除现场固定出现的约 `0.65m` 近处非危险点簇。
- 前方超声稳定接管：`switch_ultrasonic_mm=2200`，`ultrasonic_takeover_mm=2500`。
- 超声接管和精停启动都要求连续有效回波，避免远距离偶发假回波触发 `lost_radar`。
- 正式后退默认使用 `RackIndustrialDockingController.retreat(method="relative")`，提交 `relative_move(x=-distance_m)`；`0.50m/s` 只用于开环诊断。
- 后方保护为两级：任一后方原始读数小于 `500mm` 立即硬停车；`700mm` 稳定障碍需要至少 2 个后方探头同时低于阈值。
- 最新从约 `2.5m` 后退后的完整靠近复测已经成功：先激光粗靠近，再切超声精停，最终 `status='stopped'`，停稳超声约 `531~534mm`。

当前最后状态：

- 用户确认后方当前有障碍物；默认 `--cycle` 因后方保护中止是正确行为。
- 机器人曾检测到 `charge_plug_insert_state=1`，即充电插入/充电输入状态。该状态下代码会拒绝运动，不能绕过。
- 明天要继续完整 cycle，先确认后方障碍物已经移开，并且 `charge_plug_insert_state=0`。

## 下一步建议

短期：

- 移开后方障碍物，确认 `charge_plug_insert_state=0` 后，再跑默认 `--cycle`。
- 用实体尺量 3 次最终停稳距离，记录前方超声读数和尺量差异。
- 如果 3 次平均值仍小于 `500mm`，优先把 `final_stop_mm` 或 `final_brake_margin_mm` 按现场尺量重新校准，不要改激光 ROI。

中期：

- 增加日志文件输出，保存每次后退、只读快照、靠近的传感器和结果。
- 把不同料架参数做成 profile 配置，避免后续再通过临时命令改参数。
- 如果后续找到毫米波 GDK API，可以把粗靠近距离源从激光点云替换为毫米波，超声精停仍保留。

## 2026-06-10 放料后退距离恢复修正

第 3 根真实恢复中，放料、开夹、拉出已经完成，故不能再重复执行
`--resume-after-place-pull-index 3`。现场先后两次从该恢复点执行，等价于把
第 3 根放料后的后退累计到了约 `1.6m`，超过业务目标 `1.0m`。这个误差会直接改变
左转前的平移基准：即使左转角度正确，左转后的第 4 根抓料位置也会对不准料架。

当前工业修正：

- 后退启动后会把实测起点和目标前超声写入 checkpoint：
  `retreat_start_front_by_id`、`retreat_target_front_by_id`、
  `retreat_target_front_avg_mm`。
- 失败发生在“放料后后退”时，恢复提示不再要求重复完整后退，而是提示按前超声目标恢复。
- 新增恢复入口：

```bash
python3 rack_hybrid_docking_package/industrial_7_rods_total_controller.py \
  --confirm-live \
  --resume-after-place-retreat-target-index 3 \
  --place-retreat-front-target-mm 1340 \
  --end-index 7 \
  --retreat-method front-ultrasonic \
  --turn-method velocity \
  --turn-yaw-tolerance-deg 1 \
  --turn-validation-ok \
  --log-file logs/live_resume_after_place_retreat_target_YYYYMMDD_HHMM.log
```

该入口只做三件事：读取当前稳定前超声；少退则继续按前超声闭环后退，多退则低速前补；
进入 `1340±70mm` 目标窗口后才允许第 3 根左转，然后从第 4 根继续完整流程。
它不会重复第 3 根放料、开夹、拉出，也不会重复完整 `1m` 后退。

本次第 3 根的目标 `1340mm` 来自实测：放料靠近停稳约 `340mm`，业务后退目标为
`+1000mm`。如果未来 checkpoint 中已经有 `retreat_target_front_avg_mm`，优先使用
checkpoint 里的目标值，不要按经验重算。

已验证：

```text
远端 py_compile 通过。
远端 dry-run 通过：
  STEP 001 第3根：放料后退目标纠偏到前超声 1340mm
  STEP 002 第3根：向左转 90 度
  然后从第4根完整流程继续。
日志：
  logs/dryrun_resume_after_place_retreat_target_20260610_1408.log
```

最新只读状态显示当前不能实跑：`charge_plug_insert_state=1`，机器人在充电；
`motion_control error_code=0`，机器人停稳，odom/yaw 可读。当前前超声多数稳定在
约 `2050/2070mm`，相对 `1340mm` 目标仍偏远约 `0.7m`，所以禁止直接左转。
只有拔掉充电、确认现场安全后，才可执行上述恢复命令。

## 2026-06-11 独立 1m 后退验证入口

新增脚本：

```bash
rack_hybrid_docking_package/industrial_retreat_1m_validation.py
```

用途：只验证总控里的 `front-ultrasonic` 后退闭环，不执行机械臂、料架靠近、转向或七根料循环。
它复用 `industrial_7_rods_total_controller.py` 的
`_retreat_by_front_ultrasonic_delta()`，所以通过后能直接证明完整流程同一后退原语满足
前超声 `1000mm±20mm` 和 odom `1.0m±0.02m`。

dry-run：

```bash
source /home/agi/app/env.sh
cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1
python3 rack_hybrid_docking_package/industrial_retreat_1m_validation.py \
  --dry-run \
  --base-dir /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1 \
  --log-file logs/dryrun_retreat_1m_validation_YYYYMMDD_HHMM.log
```

真实单次验证：

```bash
python3 rack_hybrid_docking_package/industrial_retreat_1m_validation.py \
  --confirm-live \
  --base-dir /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1 \
  --distance-m 1.0 \
  --tolerance-mm 20 \
  --odom-tolerance-m 0.02 \
  --log-file logs/live_retreat_1m_validation_YYYYMMDD_HHMM.log
```

已完成验证：

- 本地和机器人端 `py_compile` 通过；
- 机器人端 dry-run 通过；
- 本地已保存远端 dry-run 证据：
  - `rack_hybrid_docking_package/logs/dryrun_retreat_1m_validation_20260611_1958.log`
  - `rack_hybrid_docking_package/logs/dryrun_retreat_1m_validation_20260611_1958_report.json`

最新只读状态：充电已拔掉，`charge_plug_insert_state=0`，`motion_control error_code=0`，
前方 `0/1` 稳定约 `1079/1099mm`，odom 可读且停稳；后方 `4` 稳定约 `2.25m` 以上，
后方 `5` 持续 `65535` invalid。下一步可做单次实退 1m，但在后方 5 号仍 invalid 时，
必须现场确认机器人后方至少 `1.5m` 以上无遮挡；这次只能作为 1m 距离验证，不能作为双后探头
保护完全健康的证据。

实退结果：

- 用户现场确认后方安全后，执行了独立 1m 后退验证。
- 第一次命令在运动前失败，日志
  `logs/live_retreat_1m_validation_20260611_2008.log`，原因为
  `front-ultrasonic retreat blocked: odom xy unavailable before motion`。这次没有物理后退。
- 已修复 `_read_odom_xy_from_slam()`：单次 odom 读取改为最多 `12` 次、间隔 `0.18s`
  的短重试；仍要求 odom，读不到仍停机。
- 第二次命令成功，日志
  `logs/live_retreat_1m_validation_20260611_1907_retry.log`，报告
  `logs/live_retreat_1m_validation_20260611_1907_retry_report.json`。
- 结果：
  - 起点前超声 `{0: 1079, 1: 1099}`
  - 目标前超声 `{0: 2079, 1: 2099}`
  - 终点滤波前超声 `{0: 2085, 1: 2089}`
  - 增量 `{0: 1006, 1: 990}`，剩余误差 `10mm`
  - odom 位移 `0.9819m`，误差 `-0.018m`
- 结论：独立 1m 后退已验证通过，满足前超声 `1000mm±20mm` 和 odom `1.0m±0.02m`。
- 后置快照显示 `charge_plug_insert_state=0`、`motion_control error_code=0`、
  `stopped_check=True`，前方 `0/1` 后续稳定约 `2088~2092mm`。
