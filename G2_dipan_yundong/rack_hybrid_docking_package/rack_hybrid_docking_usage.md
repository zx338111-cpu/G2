# 料架两段式靠近类使用说明

新接手时建议按这个顺序看：

1. `README.md`：最快理解每个文件、每个类和常见调用方式。
2. `rack_hybrid_docking_project_handoff.md`：完整项目交接、实测过程和调参依据。
3. 当前文件：保留最小运行命令和参数说明。

## 结论

当前实机验证后，单靠前激光雷达不能直接做到 `0.5m` 精停：高处点簇在约
`1.7m` 后会丢失并跳到远处背景。可靠方案是两段式：

- 远距离：前激光雷达 raw `+X`、`Z>0.6m` 做粗靠近
- 近距离：前方超声 `0,1` 稳定后优先接管，并按默认 `540mm + 80mm`
  补偿精停。当前复查停稳超声约 `519mm`。
- 复杂现场下，稳定超声在 `2500mm` 内会优先接管，避免激光误抓近处
  非危险结构后反复 `coarse_stopped`。

实测最终超声精停结果：

```text
DockingResult(status='stopped', min_mm=492, filtered_mm=498)
```

## 机器人端运行

```bash
source /home/agi/app/env.sh
```

只读查看当前激光雷达和超声：

```bash
python3 /home/agi/rack_hybrid_docking_demo.py --read-only
```

执行两段式靠近：

```bash
python3 /home/agi/rack_hybrid_docking_demo.py --execute --allow-estop-pedal-fault
```

正式后退 `2.5m`，带后方超声保护：

```bash
python3 /home/agi/rack_hybrid_docking_demo.py --retreat --allow-estop-pedal-fault
```

一键工业循环：先后退，再只读快照，再自动靠近精停。

```bash
python3 /home/agi/rack_hybrid_docking_demo.py --cycle --allow-estop-pedal-fault
```

## 代码调用

```python
from rack_hybrid_docking import RackHybridDockingController

with RackHybridDockingController(front_ultrasonic_ids=(0, 1)) as dock:
    result = dock.approach_to_rack(
        coarse_speed_mps=0.60,
        final_speed_mps=0.30,
        final_stop_mm=540,
        final_brake_margin_mm=80,
        switch_ultrasonic_mm=2200,
        ultrasonic_takeover_mm=2500,
        coarse_stop_m=1.6,
        allow_estop_pedal_fault=True,
    )

print(result)
```

## 参数含义

- `switch_ultrasonic_mm`：常规超声切换阈值，默认 `2200mm`。
- `ultrasonic_takeover_mm`：复杂现场稳定超声优先接管上限，默认 `2500mm`。
- `coarse_stop_m`：激光雷达粗靠近保护下限；到这个距离还没有稳定超声就停车。
- `coarse_hz`：粗靠近速度指令刷新频率，默认 `10Hz`。`0.60m/s` 时比 `5Hz` 更平顺。
- `coarse_dropout_keepalive_s`：粗靠近时单帧激光点簇丢失的速度保持时间，默认
  `0.3s`。连续丢失或接近切超声区域时仍会停车。
- `max_lidar_increase_m`：激光雷达目标突然跳远时判定目标丢失，防止继续追背景。
- `final_stop_mm`：期望停稳后的最终距离，当前现场默认 `540mm`。
- `final_brake_margin_mm`：精停制动补偿，默认 `80mm`。内部会在
  `final_stop_mm + final_brake_margin_mm` 处触发停车，抵消 `0.30m/s` 下的制动惯性。
- `retreat_distance_m`：正式后退距离，默认 `2.5m`。
- `retreat_speed`：速度开环诊断后退速度，默认 `0.50m/s`；正式工业流程默认使用相对位移闭环后退。
- `rear_stop_mm`：后方超声稳定停车阈值，默认 `700mm`。
- `rear_hard_stop_mm`：后方超声原始硬停车阈值，默认 `500mm`。
- `rear_stop_min_sensors`：稳定障碍需要同时触发的后方探头数量，默认 `2`。

当前 GDK Python 没发现毫米波接口；如果后续找到毫米波 topic/API，可以替换粗靠近
距离源，超声精停阶段仍可保留。
