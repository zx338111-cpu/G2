# G2 料架相对靠近项目交接文档

日期：2026-06-05

## 一句话结论

这个项目实现的是：G2 机器人面对上料架/下料架时，不让用户输入移动距离，而是用传感器实时判断相对距离，自动靠近并最终停在料架前约 `0.5m`。

当前已经实机验证的方案是：

1. 远距离用前激光雷达识别料架高处结构，做粗靠近。
2. 靠近到前方超声波能稳定识别后，切换到超声精停。
3. 底盘控制不用 `relative_move`，使用官方确认可用的第二种方法：`Pnc.request_chassis_control(0)` + `Pnc.move_chassis(Twist)`。
4. 最终目标距离仍写 `500mm`，但程序内部加入 `80mm` 制动补偿，会在 `580mm` 左右触发停车，停稳后接近实际 `500mm`。

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

不要使用：

```python
relative_move(...)
```

原因：现场已知 `relative_move` 会报错/不可用，用户也确认官方建议走第二种底盘控制方式。

当前使用：

```python
pnc.request_chassis_control(0)
pnc.move_chassis(twist)
```

速度方向约定：

- `Twist.linear.x > 0`：机器人向当前车头方向前进，靠近料架。
- `Twist.linear.x < 0`：机器人后退，远离料架。
- 不发送横移 `linear.y`，不发送旋转 `angular.z`，避免靠近料架时发生横向漂移或转向。

## 传感器约定

前方超声波 ID：

```text
0, 1, 2, 3
```

后方超声波 ID：

```text
4, 5, 6, 7
```

当前精停主要依赖前方 `0/1`，但代码保留 `0,1,2,3`，现场如果角度变化或不同料架反射更好，可以继续覆盖更多前方传感器。

前激光雷达点云坐标：

- 前进方向：raw `+X`
- 横向：raw `Y`
- 高度：raw `Z`

粗靠近使用的点云 ROI：

- 前向范围：`0.2m ~ 6.0m`
- 横向范围：`|Y| <= 0.8m`
- 高度范围：`0.6m <= Z <= 1.2m`
- 分箱宽度：`0.25m`
- 稳定点簇阈值：至少 `20` 个点

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

with RackHybridDockingController(front_ultrasonic_ids=(0, 1, 2, 3)) as dock:
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

- 用 `agibot_gdk.Slam().get_odom_info()` 读取里程计。
- 用 `Twist.linear.x < 0` 控制后退。
- 用后方超声 `4,5,6,7` 做防撞保护。
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

当前状态可以交给下一个人直接上手：

1. 到机器人端 `/home/agi/`。
2. `source /home/agi/app/env.sh`。
3. 执行 `python3 /home/agi/rack_hybrid_docking_demo.py --read-only --samples 8`。
4. 确认前方料架在传感器范围内。
5. 执行 `python3 /home/agi/rack_hybrid_docking_demo.py --execute --allow-estop-pedal-fault`。
6. 看到 `final_target_mm=500 final_trigger_mm=580 final_brake_margin_mm=80`。
7. 观察靠近，停稳后用尺量距离。
8. 如果略偏近或偏远，只调 `final_brake_margin_mm`，不要改 `final_stop_mm` 的业务语义。

## 下一步建议

短期：

- 再用实体尺量 3 次，记录实际停稳距离。
- 如果 3 次平均值仍小于 `490mm`，把 `final_brake_margin_mm` 从 `80` 调到 `100`。
- 如果 3 次平均值大于 `530mm`，把 `final_brake_margin_mm` 从 `80` 调到 `60`。

中期：

- 把现场后退测试脚本封装成单独的 `rack_backup_demo.py`，只用于测试，不放入生产靠近类。
- 增加日志文件输出，保存每次靠近的传感器和结果。
- 如果后续找到毫米波 GDK API，可以把粗靠近距离源从激光点云替换为毫米波，超声精停仍保留。

