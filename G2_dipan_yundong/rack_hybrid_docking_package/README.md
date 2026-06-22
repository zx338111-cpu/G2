# G2 料架靠近控制包说明

本文件夹是一套已经在 G2 实机验证过的“料架相对靠近”代码。目标是让机器人面对上料架或下料架时，不让用户输入走多少米，而是根据前方传感器自动靠近，并最终停在料架前约 `0.5m`。

当前验证结论：

- 远距离用前激光雷达做粗靠近。
- 近距离用前方超声波做精停。
- 前进精停使用 `Pnc.request_chassis_control(0)` + `Pnc.move_chassis(Twist)`；90 度转向和精确后退使用 `Pnc.relative_move(...)`。
- 当前默认目标停稳距离是 `540mm`，制动补偿 `80mm`，内部会在 `620mm` 触发停车。
- 复杂现场下，前方超声连续稳定看到 `2500mm` 内目标时会优先接管精停，避免激光误抓近处非危险结构后反复 `coarse_stopped`。
- 前激光 ROI 默认忽略 `0.8m` 内点云，排除现场固定出现的约 `0.65m` 近处非危险点簇。
- 最近一次完整复查：先后退约 `2.5m`，只读激光看到约 `2.86~3.10m`；
  默认 demo 先激光粗靠近，再切超声精停，最终 `status='stopped'`，
  停稳后前方超声约 `531~534mm`。

## 文件说明

| 文件 | 作用 | 新手怎么看 |
| --- | --- | --- |
| `rack_hybrid_docking.py` | 主控制类。把“激光粗靠近 + 超声精停”封装成一个业务入口。 | 业务代码优先调用这个文件里的 `RackHybridDockingController`。 |
| `rack_industrial_docking.py` | 工业上层流程控制器。把前进、粗定位、精定位、后退拆成独立方法，并提供 `approach()`/`cycle()` 组合流程。 | 上层业务集成优先看这个文件。 |
| `industrial_cell_7_rods_single_debug.py` | 七根料杆主执行脚本。支持 `--profile`，从现场 profile 读取地图、站点、抓料点、放料点和调参。 | 需要直接调主流程时看这个；正常运行优先用 `run_site_7_rods_live.py` 包一层安全预检。 |
| `rack_hybrid_docking_demo.py` | 命令行 demo。可以直接在机器人上运行测试。 | 不写代码时用这个文件跑实机。 |
| `rack_lidar_docking.py` | 前激光雷达粗靠近模块。负责从点云里估算料架距离，并发送粗靠近速度。 | 一般不用直接调用，除非调试远距离识别。 |
| `rack_radar_docking.py` | 前方超声波精停模块。负责读取前方超声，并在目标距离处停车。 | 一般不用直接调用，除非只做近距离精停。 |
| `rack_retreat_controller.py` | 带后方超声保护的正式后退控制器。 | 工业循环里用它后退，不再写临时后退脚本。 |
| `rack_industrial_docking_usage.md` | 工业方法使用说明。 | 想把方法接到业务代码时看它。 |
| `rack_hybrid_docking_usage.md` | 简版运行说明。 | 想快速复制命令时看它。 |
| `NEW_SITE_REPLICATION_GUIDE.md` | 换地图、重新采抓料/放料点、复刻七根流程的保姆级教程。 | 新现场交给新人复刻时先看这个。 |
| `profiles/map20_box528/profile.json` | 当前 map20 成功基线的现场 profile。 | 想复刻或换地图时，把它作为模板复制。 |
| `site_profile.py` | profile 路径解析工具，不会动机器人。 | 采点、校验、运行入口共用它解析 profile 内的文件。 |
| `create_site_profile.py` | 从已验证 profile 创建一个空白新现场 profile。 | 新地图开始前先用它创建目录，避免误用旧点位。 |
| `validate_site_profile.py` | 现场 profile 本地校验器，不会动机器人。 | 新 profile 做完后先跑它检查文件和参数。 |
| `run_site_7_rods_live.py` | 按 profile 运行七根流程的通用入口。 | 后续不要再写死 map20，优先用这个入口。 |
| `process_vision_capture.py` | 抓料/放料过程视觉采集模块。只读相机、TF、内参和机器人状态，不发运动指令。 | 后续评估 AI/视觉纠偏前，用它先积累现场真实过程图片。 |
| `rack_hybrid_docking_project_handoff.md` | 完整交接文档。包含项目背景、实测过程、参数来源、问题处理和后续建议。 | 接手项目前完整读一遍。 |
| `README.md` | 当前入口文档。说明每个文件、每个类、每种典型用法。 | 新人先看这个。 |

## 机器人端运行环境

机器人端推荐路径：

```bash
cd /data/bengtian/wxf/BOX_528_1/rack_hybrid_docking_package
source /home/agi/app/env.sh
```

必须先 `source /home/agi/app/env.sh`，否则 `agibot_gdk` 可能无法导入或 DDS/GDK 连接不正常。

## 七根流程 profile 入口

后续换地图、换点位时，不要直接在主程序里改路径。先创建或选择一个
`profiles/<site>/profile.json`，再用 profile 入口检查和运行：

```bash
python3 rack_hybrid_docking_package/validate_site_profile.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json

python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --preflight-only
```

如果必须直接调用主执行脚本，也要带同一个 profile：

```bash
./rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --preflight-only
```

## 抓料/放料过程视觉采集

现在先不做闭环视觉纠偏，只做数据采集。现场跑单根或七根时加
`--vision-capture`，主流程会在 `LOCAL_PICK` 和 `LOCAL_PLACE` 的每个本地
动作前后保存一组图片和元数据，同时在整个抓料/放料阶段启动低频连续采样。
采集失败只写日志，不改变原来的点位式运动。

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --start-index 1 \
  --end-index 1 \
  --vision-capture
```

默认保存位置类似：

```text
logs/map20_box528_vision_dataset_<时间>/
```

每个采集点都会生成一个 `manifest.json`，里面记录当前杆号、阶段、步骤、
相机文件、相机内参、TF 外参、关节状态和机器人状态。默认相机包括：

- `head_stereo_left`
- `head_color`
- `head_depth`
- `hand_left_color`
- `hand_right_color`

如果现场磁盘空间紧张，可以先只采主视角和头部深度：

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --start-index 1 \
  --end-index 1 \
  --vision-capture \
  --vision-capture-cameras head_stereo_left,head_depth
```

连续采样默认每 `1.0s` 采一轮多相机图片。如果动作比较快，可以缩短间隔；
如果磁盘或 CPU 压力大，可以加大间隔：

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --start-index 1 \
  --end-index 1 \
  --vision-capture \
  --vision-capture-interval-s 0.5
```

## 最快运行方式

### 1. 只读查看传感器

这个命令不会让机器人运动，只读取前激光雷达和前方超声波。

```bash
python3 rack_hybrid_docking_demo.py --read-only --samples 8
```

输出示例：

```text
sample=1 lidar=LidarRackDistance(distance_m=2.72, ...) ultrasonic_min_mm=None ultrasonic_raw=()
sample=2 lidar=LidarRackDistance(distance_m=2.70, ...) ultrasonic_min_mm=1500 ultrasonic_raw=((0, 1510), (1, 1500))
```

怎么看：

- `lidar=...distance_m=2.72` 表示激光看到料架约 `2.72m`。
- `ultrasonic_min_mm=None` 表示前方超声暂时没有有效回波。
- `ultrasonic_min_mm=1500` 表示前方超声看到最近目标约 `1.5m`。

### 2. 直接执行自动靠近

```bash
python3 rack_hybrid_docking_demo.py --execute --allow-estop-pedal-fault
```

当前默认参数会打印：

```text
final_target_mm=540 final_trigger_mm=620 final_brake_margin_mm=80 switch_ultrasonic_mm=2200 ultrasonic_takeover_mm=2500 ultrasonic_stable_tolerance_mm=250
```

这表示：

- 业务目标停稳距离：`540mm`
- 内部提前触发停车距离：`620mm`
- 制动补偿：`80mm`
- 稳定超声接管上限：`2500mm`

如果最后输出：

```text
result=HybridDockingResult(status='stopped', stage='final_ultrasonic', ...)
```

说明已经完成自动靠近，并由超声精停成功停车。

### 3. 正式后退 2.5m

```bash
python3 rack_hybrid_docking_demo.py --retreat --allow-estop-pedal-fault
```

默认参数：

- 后退距离：`2.5m`
- 后退默认方法：`relative_move(x=-distance_m)` 相对位移闭环
- 速度开环诊断速度：`0.50m/s`
- 后方超声 ID：`4,5`
- 后方稳定停车距离：`700mm`
- 后方原始硬停车距离：`500mm`
- 后方稳定停车确认：至少 `2` 个后方探头同时低于 `700mm`

如果输出：

```text
retreat_result=RetreatResult(status='completed', ...)
```

说明后退完成。如果输出 `status='rear_obstacle'`，说明后方超声检测到障碍，程序已经停车。后方保护是两级：任一原始读数小于 `500mm` 立即停车；`700mm` 稳定障碍需要至少 2 个后方探头同时低于阈值，避免单个探头看固定结构时误停。

### 4. 一键工业循环

```bash
python3 rack_hybrid_docking_demo.py --cycle --allow-estop-pedal-fault
```

循环内容：

1. 受控后退 `2.5m`。
2. 只读采样激光和前方超声。
3. 自动靠近料架。
4. 超声精停。
5. 输出 `cycle_result=retreat:completed,approach:stopped`。

## 主类怎么用

如果是业务系统集成，优先使用工业上层类：

```python
from rack_industrial_docking import RackIndustrialDockingController

with RackIndustrialDockingController() as rack:
    coarse = rack.coarse_position(allow_estop_pedal_fault=True)
    if coarse.status != "ready_for_fine":
        raise RuntimeError(coarse)

    fine = rack.fine_position(allow_estop_pedal_fault=True)
    if fine.status not in ("stopped", "already_at_threshold"):
        raise RuntimeError(fine)
```

完整方法说明见 `rack_industrial_docking_usage.md`。底层两段式主类仍保留在
`rack_hybrid_docking.py`，适合只需要一个“自动靠近”入口的场景。

主类在 `rack_hybrid_docking.py`：

```python
from rack_hybrid_docking import RackHybridDockingController
```

类名：

```python
RackHybridDockingController
```

推荐只调用一个方法：

```python
approach_to_rack(...)
```

这个方法会自动完成：

1. 检查底盘安全状态。
2. 启动时先看前方超声是否已经能看到料架。
3. 如果距离较远，先进入激光粗靠近。
4. 超声距离进入 `switch_ultrasonic_mm`，或稳定超声进入 `ultrasonic_takeover_mm` 后，切换到超声精停。
5. 在补偿后的触发距离停车。
6. 返回 `HybridDockingResult`。

### 示例 1：最推荐的业务调用

```python
from rack_hybrid_docking import RackHybridDockingController

with RackHybridDockingController(front_ultrasonic_ids=(0, 1)) as dock:
    result = dock.approach_to_rack(
        allow_estop_pedal_fault=True,
    )

print(result)
```

适用场景：

- 机器人正面对料架。
- 使用当前实机验证过的默认参数。
- 最终希望停稳在料架前约 `0.5m`。

### 示例 2：显式写出当前生产参数

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

适用场景：

- 想让代码读起来非常清楚。
- 想明确告诉后续维护者当前所有关键参数。

### 示例 3：实际停稳还是偏近，提前更多停车

如果尺量发现实际停稳距离仍小于 `500mm`，例如停在 `470mm`，可以把补偿从 `80mm` 调到 `100mm`。

```python
with RackHybridDockingController(front_ultrasonic_ids=(0, 1)) as dock:
    result = dock.approach_to_rack(
        final_stop_mm=500,
        final_brake_margin_mm=100,
        allow_estop_pedal_fault=True,
    )
```

含义：

```text
目标停稳距离：500mm
内部触发距离：600mm
```

### 示例 4：实际停稳偏远，减少提前量

如果尺量发现实际停稳距离大于 `550mm`，可以把补偿从 `80mm` 调到 `60mm`。

```python
with RackHybridDockingController(front_ultrasonic_ids=(0, 1)) as dock:
    result = dock.approach_to_rack(
        final_stop_mm=500,
        final_brake_margin_mm=60,
        allow_estop_pedal_fault=True,
    )
```

含义：

```text
目标停稳距离：500mm
内部触发距离：560mm
```

### 示例 5：精停段想更稳，降低精停速度

如果不追求速度，想让最终距离更稳，可以把精停速度降到 `0.20m/s`。

```python
with RackHybridDockingController(front_ultrasonic_ids=(0, 1)) as dock:
    result = dock.approach_to_rack(
        final_speed_mps=0.20,
        final_stop_mm=500,
        final_brake_margin_mm=50,
        allow_estop_pedal_fault=True,
    )
```

注意：

- 速度降低后制动惯性会变小。
- `final_brake_margin_mm` 也应相应减小。
- 需要重新尺量验证。

### 示例 6：粗靠近仍觉得抖，保守一点

如果现场地面或料架反光导致粗靠近还是有轻微抖动，可以先把粗靠近速度降到 `0.45m/s`。

```python
with RackHybridDockingController(front_ultrasonic_ids=(0, 1)) as dock:
    result = dock.approach_to_rack(
        coarse_speed_mps=0.45,
        final_speed_mps=0.30,
        final_stop_mm=500,
        final_brake_margin_mm=80,
        allow_estop_pedal_fault=True,
    )
```

注意：

- 粗靠近速度只影响远距离段。
- 近距离最终停车仍由超声精停控制。

### 示例 7：接入上层业务函数

可以把靠近封装成业务函数：

```python
from rack_hybrid_docking import RackHybridDockingController


def dock_to_loading_rack():
    with RackHybridDockingController(front_ultrasonic_ids=(0, 1)) as dock:
        result = dock.approach_to_rack(
            final_stop_mm=500,
            final_brake_margin_mm=80,
            allow_estop_pedal_fault=True,
        )

    if result.status != "stopped":
        raise RuntimeError(f"Docking failed: {result}")

    return result
```

上层逻辑只需要判断：

```python
result.status == "stopped"
```

## `approach_to_rack()` 参数说明

| 参数 | 默认值 | 说明 | 什么时候改 |
| --- | --- | --- | --- |
| `coarse_speed_mps` | `0.60` | 激光粗靠近速度，单位 m/s。 | 远距离段太快或太抖时调低。 |
| `final_speed_mps` | `0.30` | 超声精停速度，单位 m/s。 | 停车距离不稳或近距离太快时调低。 |
| `final_stop_mm` | `500` | 期望停稳后的距离，单位 mm。 | 业务目标不是 0.5m 时才改。 |
| `final_brake_margin_mm` | `80` | 制动补偿，内部触发距离会提前这么多。 | 尺量偏近就调大，偏远就调小。 |
| `switch_ultrasonic_mm` | `1800` | 超声滤波距离小于该值后，从激光切到超声。 | 料架超声更早/更晚稳定时调整。 |
| `coarse_stop_m` | `1.6` | 激光粗靠近保护下限。 | 超声不稳定时保护用，一般不改。 |
| `coarse_hz` | `10.0` | 粗靠近速度刷新频率。 | 一般不改。 |
| `coarse_dropout_keepalive_s` | `0.3` | 激光单帧丢失时保持速度的时间。 | 粗靠近抖动或误停时微调。 |
| `final_max_duration_s` | `60.0` | 精停最长时间。 | 现场距离很远但直接超声精停时可加大。 |
| `allow_estop_pedal_fault` | `False` | 是否允许已知急停踏板故障。 | 当前这台 G2 实机测试必须传 `True`。 |

## 返回结果怎么看

返回类型是 `HybridDockingResult`，字段如下：

| 字段 | 含义 |
| --- | --- |
| `status` | 最终状态。`stopped` 表示成功停车。 |
| `stage` | 停车发生在哪个阶段。通常成功时是 `final_ultrasonic`。 |
| `elapsed_s` | 整个靠近动作耗时。 |
| `lidar_filtered_m` | 切换前最后的激光滤波距离。近距离直接超声时可能是 `None`。 |
| `ultrasonic_filtered_mm` | 切换到超声时的超声滤波距离。 |
| `final_status` | 超声精停内部返回状态。 |
| `coarse_samples` | 粗靠近阶段有效激光样本数。 |
| `final_samples` | 精停阶段有效超声样本数。 |

常见状态：

| 状态 | 含义 | 处理 |
| --- | --- | --- |
| `stopped` | 成功到达停车条件。 | 正常。 |
| `already_at_threshold` | 启动时已经在停车阈值内。 | 不会继续前进。 |
| `lost_lidar` | 粗靠近阶段连续丢失激光目标。 | 检查料架位置、激光 ROI、遮挡。 |
| `target_lost` | 激光目标突然跳远，被认为追到背景。 | 检查料架是否偏离正前方。 |
| `coarse_stopped` | 激光到保护下限仍没有稳定超声。 | 检查前方超声供电和 ID。 |
| `timeout` | 超过最大运行时间。 | 检查目标是否在正前方。 |

## 底层类说明

### `RackLidarDockingController`

位置：`rack_lidar_docking.py`

作用：

- 读取前激光雷达点云。
- 只看车头前方、高处料架结构点。
- 从点云分箱中找到最近稳定点簇。
- 用于远距离粗靠近。

一般业务不直接调用它，因为主类会自动调用。

只读调试示例：

```python
from rack_lidar_docking import RackLidarDockingController

with RackLidarDockingController() as lidar:
    distance = lidar.read_rack_distance()
    print(distance)
```

### `RackRadarDockingController`

位置：`rack_radar_docking.py`

作用：

- 读取前方超声波。
- 取前方 ID 中的最小有效距离。
- 做中位数滤波。
- 在目标距离处停车。

只做近距离超声精停时可以直接调用：

```python
from rack_radar_docking import RackRadarDockingController

with RackRadarDockingController(front_ids=(0, 1)) as radar:
    result = radar.approach_until_distance(
        stop_mm=580,
        speed_mps=0.30,
        hard_stop_mm=580,
        allow_estop_pedal_fault=True,
    )

print(result)
```

注意：

- 这里的 `stop_mm=580` 是内部触发距离，不是业务目标 `500mm`。
- 直接调用底层类时，制动补偿需要你自己换算。
- 推荐业务仍调用 `RackHybridDockingController`，让主类处理补偿。

## 传感器 ID 约定

前方超声：

```text
0, 1
```

右侧超声：

```text
2, 3
```

后方超声：

```text
4, 5
```

左侧超声：

```text
6, 7
```

当前主类 `RackHybridDockingController` 只负责向前靠近。正式后退由
`RackRetreatController` 负责，demo 中的 `--cycle` 会把“受控后退 -> 只读快照
-> 自动靠近 -> 超声精停”串成一个工业循环。

## 安全注意事项

当前机器人存在已知状态：

```text
emergency_stop_pedal_fault_state=1
```

现场已确认这是已知硬件问题，官方建议可以使用第二种底盘控制方式继续测试。因此 demo 需要加：

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

当前最后状态：

- 后方当前有障碍物时，`--retreat/--cycle` 触发后方保护并中止是正确行为。
- `charge_plug_insert_state=1` 表示充电插入/充电输入状态，代码会拒绝底盘运动。
- 明天完整测试前，先确认后方障碍物移开，并且 `charge_plug_insert_state=0`。

## 常见问题

### 为什么不是直接走 0.5m？

因为需求是相对料架定位，不是开环走固定距离。机器人和料架初始距离可能不同，所以必须用传感器闭环判断距离。

### 哪些动作使用 `relative_move`？

分场景使用：

- 前进靠近料架：继续用 `request_chassis_control(0)` + `move_chassis(Twist)`，因为需要前超声实时精停。
- 右/左转 90 度：使用 `relative_move(yaw=±90)`，并严格监控新任务最终 state，只接受 `3/9`。
- 后退 1m：使用 `relative_move(x=-distance_m)` 做相对位移闭环，后方 `4/5` 超声只做障碍保护。

### 为什么设置 500mm，程序打印触发 580mm？

`500mm` 是希望停稳后的真实距离。`580mm` 是考虑底盘制动惯性的提前触发距离。上一次实测中，提前到 `580mm` 后，停稳读数约 `507mm / 481mm`。

### 如果尺子量出来还是不准怎么办？

只调 `final_brake_margin_mm`：

- 停得太近：`80 -> 100`
- 停得太远：`80 -> 60`

不要随便改 `final_stop_mm=500`，它表示业务目标。

### 如果从 1m 左右启动，为什么没有粗靠近日志？

因为前方超声一开始就能看到料架，主类会直接进入超声精停，这是正常的。

### 如果从 3m 左右启动，为什么一开始超声是 None？

前方超声距离远时不一定有稳定回波，这是实机验证过的正常现象。此时程序会用前激光雷达先粗靠近。
