# G2 七根料三大核心工业动作

日期：2026-06-10

这个工程真正决定稳定性的不是“完整七根流程”，而是三个底层工业动作：

1. 抓料/放料前超声精定位；
2. 每次后退 `1m`；
3. 左/右转 `90` 度。

完整七根只能在这三个动作分别稳定后才有意义。

## 1. 抓料/放料精定位

### 输入条件

- `charge_plug_insert_state=0`。
- `motion_control error_code=0`。
- 前方超声供电正常。
- 前方 `0/1` 同时有稳定有效距离。

### 主闭环

- 粗定位仍由前激光把机器人带入前超声接管区。
- 精定位只看前方 `0/1` 超声。
- 停车触发点不是业务目标，而是：

```text
trigger_mm = target_mm + brake_margin_mm
```

当前默认：

```text
抓料 target=170mm, brake=45mm, trigger=215mm
放料 target=500mm, brake=60mm, trigger=560mm
```

### 停稳复核

停车后不会用单帧数据判定成功，而是多帧读取前方 `0/1`：

- 必须连续获得稳定前超声；
- 两个探头跨度不能超过一致性阈值；
- 使用每个探头的中位数作为最终证据；
- 如果低于安全下限，立即失败停机。
- 单帧 `distance_mm=None`、`65535` 或非数字值只作为无效帧过滤，不作为
  成功距离，也不会导致未分类 Python 异常。

新增抓料目标窗口：

```text
grab_target_tolerance_mm=45
```

也就是抓料停稳后必须进入 `170±45mm`，同时不能低于 `150mm` 安全下限。
如果停得太远，允许一次 `0.035m/s` 的低速补近；如果已经太近或两个探头
显示角度不一致，不继续盲目前进。

放料目标窗口仍为：

```text
place_target_tolerance_mm=60
```

也就是 `500±60mm`，最多两次 `0.05m/s` 低速补近。

## 2. 后退 1m

### 输入条件

- 后方 `4/5` 不触发硬保护。
- 前方 `0/1` 在后退开始前稳定可读。
- `motion_control error_code` 只允许 `0`，或贴近料架时的可解释 `2` 撤离场景。

### 主闭环

默认不使用速度乘时间，也不只信任 `relative_move`。

当前生产默认是：

```text
--retreat-method front-ultrasonic
```

逻辑：

```text
start_front_by_id = {0: start0, 1: start1}
target_delta_mm = 1000
```

后退过程中要求：

```text
front0_delta ≈ 1000mm
front1_delta ≈ 1000mm
```

- 退少：继续低速后退；
- 退多：低速前补；
- 两个前探头增量差过大：停车等待，持续不一致则失败；
- 后方任一原始距离小于 `500mm`：硬停车；
- 后方两个探头稳定小于 `700mm`：障碍停车。

### 交叉校验

新增 SLAM odom 交叉校验：

```text
retreat_odom_tolerance_m=0.25
```

如果 `Slam.get_odom_info()` 可读，则后退完成后计算 map 坐标位移：

```text
odom_displacement = hypot(end_x - start_x, end_y - start_y)
```

前超声认为退了 `1m`，但 odom 位移不在 `1.0±0.25m` 内时，直接失败停机。

默认不强制 odom 必须可读，因为现场 odom 会在未重定位或充电状态下为 null；
如果需要强制双尺一致，运行时加：

```bash
--retreat-require-odom-crosscheck
```

## 3. 左/右转 90 度

### 输入条件

- `charge_plug_insert_state=0`。
- `motion_control error_code=0`。
- `Slam.get_odom_info()` yaw 可读。
- 机器人已停稳。

### 主闭环

默认方法仍然是语义正确的：

```text
Pnc.relative_move(yaw=±90deg)
```

但不再信任旧任务状态：

- 必须看到新 task，或看到任务进入运行态；
- 只接受 `state=3/9`；
- `state=7` 仍视为取消/失败；
- 即使命令行误传成功状态，也不允许把 `7` 加入成功集合；
- 转向前后必须读 odom yaw。

### 低速补角

新增转向后 yaw 闭环补角，默认开启：

```text
turn_correction_enabled=true
turn_correction_max_passes=3
turn_correction_angular_speed_radps=0.20
turn_correction_max_error_deg=25
turn_yaw_tolerance_deg=8
```

流程：

1. `relative_move(±90)` 完成；
2. 读取当前 yaw；
3. 如果误差小于 `8deg`，通过；
4. 如果误差在 `25deg` 内，低速短时补角；
5. 每次补角前重新检查底盘安全状态，补角后重新读 yaw；
6. 三次仍不进 `8deg`，失败停机；
7. 一开始误差超过 `25deg`，说明主转向异常，不做补角，直接失败。

这不是“多转几秒”的开环补偿，而是基于 yaw 的闭环小修正。

## 推荐现场顺序

不要从完整七根开始。先按下面顺序验证：

```bash
python3 industrial_status_snapshot.py --samples 8

python3 industrial_turn_diagnostic.py --confirm-live --direction right --method relative --repeat 3
python3 industrial_turn_diagnostic.py --confirm-live --direction left  --method relative --repeat 3

python3 rack_hybrid_docking_package/industrial_7_rods_total_controller.py \
  --confirm-live \
  --start-index 1 \
  --end-index 1 \
  --retreat-method front-ultrasonic \
  --log-file logs/rod1_stability_YYYYMMDD_HHMM.log
```

只有第 1 根完整闭环稳定后，再考虑 `--start-index 1 --end-index 7`。
连续多根真实运行必须显式传 `--turn-validation-ok`，表示左右 `90` 度已经
先用 `industrial_turn_diagnostic.py` 多次单测通过：

```bash
python3 rack_hybrid_docking_package/industrial_7_rods_total_controller.py \
  --confirm-live \
  --start-index 1 \
  --end-index 7 \
  --retreat-method front-ultrasonic \
  --turn-validation-ok \
  --log-file logs/full_7_rods_YYYYMMDD_HHMM.log
```
