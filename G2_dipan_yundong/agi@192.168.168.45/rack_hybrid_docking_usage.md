# 料架两段式靠近类使用说明

新接手时建议按这个顺序看：

1. `README.md`：最快理解每个文件、每个类和常见调用方式。
2. `rack_hybrid_docking_project_handoff.md`：完整项目交接、实测过程和调参依据。
3. 当前文件：保留最小运行命令和参数说明。

## 结论

当前实机验证后，单靠前激光雷达不能直接做到 `0.5m` 精停：高处点簇在约
`1.7m` 后会丢失并跳到远处背景。可靠方案是两段式：

- 远距离：前激光雷达 raw `+X`、`Z>0.6m` 做粗靠近
- 近距离：前方超声 `0,1,2,3` 稳定后，切换到超声并在 `500mm` 停车

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

## 代码调用

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

## 参数含义

- `switch_ultrasonic_mm`：前方超声滤波距离进入该阈值后，从激光雷达切换到超声。
- `coarse_stop_m`：激光雷达粗靠近保护下限；到这个距离还没有稳定超声就停车。
- `coarse_hz`：粗靠近速度指令刷新频率，默认 `10Hz`。`0.60m/s` 时比 `5Hz` 更平顺。
- `coarse_dropout_keepalive_s`：粗靠近时单帧激光点簇丢失的速度保持时间，默认
  `0.3s`。连续丢失或接近切超声区域时仍会停车。
- `max_lidar_increase_m`：激光雷达目标突然跳远时判定目标丢失，防止继续追背景。
- `final_stop_mm`：期望停稳后的最终距离，默认 `500mm`。
- `final_brake_margin_mm`：精停制动补偿，默认 `80mm`。内部会在
  `final_stop_mm + final_brake_margin_mm` 处触发停车，抵消 `0.30m/s` 下的制动惯性。

当前 GDK Python 没发现毫米波接口；如果后续找到毫米波 topic/API，可以替换粗靠近
距离源，超声精停阶段仍可保留。
