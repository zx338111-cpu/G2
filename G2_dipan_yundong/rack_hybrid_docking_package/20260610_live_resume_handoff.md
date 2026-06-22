# 2026-06-10 G2 七根料现场接续文档

本文件用于下次开工直接接续，不重新分析历史。

## 当前结论

当前不能继续跑第 4 根。目标机器人已经重新在线，但现场状态仍有两个硬阻塞：

1. `charge_plug_insert_state=1`，充电输入仍在 `50.5V / 15A` 左右，两块电池处于 charging。
2. `Slam.get_odom_info()` 又返回 `Slam odom is null`，所以 yaw 不可用。

在这两个条件恢复前，不要执行底盘运动，不要继续左转 90 度，也不要从第 4 根开始抓料。

## 目标机器人和项目路径

SSH：

```bash
SSHPASS='<robot-password>' sshpass -e ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password \
  -o NumberOfPasswordPrompts=1 -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
  agi@10.20.15.199
```

机器人端项目根目录：

```text
/data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1
```

主控制器：

```text
rack_hybrid_docking_package/industrial_7_rods_total_controller.py
```

状态快照脚本：

```text
industrial_status_snapshot.py
```

## 已经确认过的现场进度

- 第 1 根、第 2 根、第 3 根已经完成到第 3 根放料、开夹、拉出、放料后退恢复点。
- 当前应从“第 3 根放料退后完成，准备左转 90 度去第 4 根”继续。
- 第 3 根放料退后目标窗口已被确认过：
  - 目标前方距离：`1357mm`
  - 容差：`±70mm`
  - 最近稳定前超声约 `1320~1330mm`
- 后退距离必须继续用 `front-ultrasonic` 闭环，不要再用纯速度开环；后退距离错误会直接影响第 4 根对准。

## 最近一次网络和状态确认

曾出现 `10.20.15.199` 临时不可达：

```text
ssh: connect to host 10.20.15.199 port 22: No route to host
10.20.15.199 dev wlp3s0f0 FAILED/incomplete
```

后续通过并发 SSH 端口扫描重新发现 `.199`：

```text
10.20.15.1
10.20.15.60
10.20.15.107
10.20.15.170
10.20.15.199
```

已确认 `.199` 是目标机器人：

```text
hostname=G2
whoami=agi
BOX528_PRESENT
```

但最新只读快照仍显示：

```text
charge_plug_insert_state=1
charge_plug_input_voltage=50.5
charge_plug_input_current=15.0
battery_charging_status=1
motion_control error_code=0
motion_control mode=1
emergency_stop_pedal_state=0
emergency_stop_pedal_fault_state=1
odom_error=RuntimeError: GetOdomInfo failed
stopped_check odom_available=false
```

最新超声大致：

```text
front 0/1: 1208~1239mm
right 2/3: 632~970mm
rear 4/5: one side about 740mm, one side about 2.1m
left 6/7: 1109~1183mm
```

## 今天处理过的关键问题

### 1. 第 4 根前不能盲转

用户明确指出：后退不是退得越多越安全，退多了左转后会无法对准料架，直接导致第 4 根抓不到。当前流程必须把后退和转向都作为工业级闭环控制问题处理。

当前总控里的生产转向要求：

```text
--turn-method velocity
--turn-yaw-tolerance-deg 1
```

它要求 `Slam.get_odom_info()` 可读 yaw。yaw 不可用时控制器会拒绝转向：

```text
RuntimeError: velocity turn cannot start: yaw feedback unavailable
```

这是正确的安全阻断，不要绕过后直接全流程继续。

### 2. SLAM/odom 曾恢复过，但后来又丢失

用户现场做过重定位后，状态一度恢复：

```text
odom_available=true
orientation_euler yaw about 1.522 rad
loc_confidence=80
```

但当时 `charge_plug_insert_state=1`，机器人仍在充电，所以没有继续运动。

后续网络恢复后再次读取，`charge_plug_insert_state` 仍为 `1`，且 `Slam odom is null` 又出现。

### 3. 自动重定位尝试结果

使用 GDK DDS 直接发布过非运动 SLAM 全局重定位请求：

```text
/slam/global_loc_request
control=1
relocalization_mode=0
```

请求被接受，`/slam/global_loc_response result=0`，但 SLAM 最终失败：

```text
Relocalization Is Failed!!!
Localization --- stop
```

也试过高精定位启动：

```text
/pnc/task_service/high_precision_localization_start/request
type=0
type=1
type=2
```

全部返回：

```text
TASK_SERVICE_RESULT_FAILURE
```

tagloc 日志里的关键失败原因：

```text
Charger point cloud too small: 0 points
Please Check Whether The Charger Is Obstructed!!!
反光条检测失败，初始化失败
TagLoc Initialier Failed!!!
```

所以目前最可靠恢复方式仍是现场/HMI/智元重定位，而不是让总控盲转。

## 下次开工第一步

先只读检查：

```bash
source /home/agi/app/env.sh
cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1
python3 industrial_status_snapshot.py --samples 8 --interval-s 0.25
```

必须同时满足：

```text
charge_plug_insert_state=0
motion_control error_code=0
emergency_stop_pedal_state=0
odom_available=true
stopped_check=True
front 0/1 距离仍在第 3 根放料退后目标窗口附近，或能解释偏差来源
```

如果 `charge_plug_insert_state=1`：

- 不要继续。
- 让现场确认充电插头和充电状态已解除。

如果 `odom_available=false`：

- 不要继续。
- 让现场/HMI/智元重新做定位。
- 重新确认 `Slam.get_odom_info()` 能读到 yaw 后再恢复流程。

## 恢复执行命令

只有上述只读条件都满足后，才执行：

```bash
source /home/agi/app/env.sh
cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1
python3 rack_hybrid_docking_package/industrial_7_rods_total_controller.py \
  --confirm-live \
  --resume-after-place-retreat-target-index 3 \
  --place-retreat-front-target-mm 1357 \
  --end-index 7 \
  --retreat-method front-ultrasonic \
  --turn-method velocity \
  --turn-yaw-tolerance-deg 1 \
  --turn-validation-ok \
  --log-file logs/live_resume_rod3_place_retreat_target_to_rod7_YYYYMMDD_HHMM.log
```

恢复后控制器应先做第 3 根放料后退目标窗口确认，然后左转 90 度去第 4 根。若 yaw 不可读，控制器应再次阻断，不要手动绕过到全流程。

## 关键历史日志

```text
logs/live_full_from_rod1_20260610_1608.log
logs/live_resume_after_status_ok_rod1_place_retreat_to_rod7_20260610_1620.log
logs/live_resume_rod2_after_grab_retreat_corrected_to_rod7_20260610_1622.log
logs/live_resume_rod3_after_manual_turn_place_approach_to_rod7_20260610_1644.log
logs/live_resume_rod3_after_place_pull_to_rod7_20260610_1648.log
logs/live_resume_rod3_place_retreat_target_to_rod7_20260610_1654.log
```

当前最重要的失败日志：

```text
logs/live_resume_rod3_place_retreat_target_to_rod7_20260610_1654.log
```

核心报错：

```text
velocity turn cannot start: yaw feedback unavailable
```

## 给下一次 Codex 的操作边界

- 不要从第 1 根重跑，除非用户明确要求。
- 默认从第 3 根放料后退完成点恢复。
- 不要在 `charge_plug_insert_state=1` 时执行底盘运动。
- 不要在 `Slam odom is null` 时执行 90 度转向或继续第 4 根。
- 后退必须使用 `front-ultrasonic`，不要临时改成纯速度开环。
- 如果用户现场确认安全但 yaw 仍不可用，最多只能讨论单步开环转向风险，不要直接跑完整 4-7 根。
