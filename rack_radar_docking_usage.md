# 料架前方雷达靠近类使用说明

## 已验证硬件路径

- 底盘控制：`request_chassis_control(mode=0)` + `move_chassis(Twist)`
- 前方雷达 ID：`0,1,2,3`
- 停车阈值：默认 `500mm`，也就是距离料架 `0.5m` 自动停车
- 低速靠近：建议 `0.02m/s`
- 滤波：最近 `3` 次最小距离取中位数
- 现场验证结果：机器人从约 `2.08m` 稳定靠近；现在业务默认改为 `500mm` 更保守停车

## 机器人端运行环境

```bash
source /home/agi/app/env.sh
```

## 先只读雷达

```bash
python3 /home/agi/rack_radar_docking_demo.py --read-only
```

## 执行自动靠近

当前这台 G2 有官方确认的急停踏板已知硬件故障，所以需要显式加
`--allow-estop-pedal-fault`：

保守调试速度：

```bash
python3 /home/agi/rack_radar_docking_demo.py \
  --execute \
  --allow-estop-pedal-fault \
  --front-ids 0,1,2,3 \
  --stop-mm 500 \
  --speed 0.02 \
  --max-duration 70 \
  --hz 10 \
  --history 3 \
  --initial-radar-timeout 2.0 \
  --acquire-speed 0.03 \
  --acquire-max-distance 0.6 \
  --acquire-step-duration 0.4 \
  --acquire-timeout 20 \
  --lost-radar-timeout 1.0
```

正常速度。当前业务默认在 500mm，也就是料架前 0.5m 停：

```bash
python3 /home/agi/rack_radar_docking_demo.py \
  --execute \
  --allow-estop-pedal-fault \
  --front-ids 0,1,2,3 \
  --stop-mm 500 \
  --speed 0.05 \
  --max-duration 80 \
  --hz 10 \
  --history 3 \
  --initial-radar-timeout 2.0 \
  --acquire-speed 0.03 \
  --acquire-max-distance 0.6 \
  --acquire-step-duration 0.4 \
  --acquire-timeout 20 \
  --lost-radar-timeout 1.0
```

## 在代码里调用：最小版本

```python
from rack_radar_docking import RackRadarDockingController

with RackRadarDockingController(front_ids=(0, 1, 2, 3)) as dock:
    result = dock.approach_to_rack(
        speed_mps=0.05,
        max_duration_s=80,
        allow_estop_pedal_fault=True,
    )
    print(result.status, result.filtered_mm)
```

## 在代码里调用：带实时日志版本

```python
from rack_radar_docking import RackRadarDockingController


def print_sample(sample):
    # sample.min_mm 是当前一帧前方雷达最小值。
    # sample.filtered_mm 是最近 history_size 帧的中位数，停车判断用这个值。
    # sample.distances 是原始有效读数，例如 ((0, 285), (1, 523))。
    print(
        f"t={sample.elapsed_s:.1f}s "
        f"min={sample.min_mm}mm "
        f"filtered={sample.filtered_mm}mm "
        f"raw={sample.distances}"
    )


with RackRadarDockingController(
    front_ids=(0, 1, 2, 3),  # 前方雷达 ID，现场已确认
    control_mode=0,          # 官方 move_chassis 示例默认模式，现场验证可用
) as dock:
    result = dock.approach_to_rack(
        speed_mps=0.05,              # 正常速度；0.02 可用于保守调试
        max_duration_s=80,           # 最长运行 80 秒，超时也会停车
        hz=10,                       # 10Hz 控制和雷达检查
        history_size=3,              # 3 帧中位数滤波，抑制单帧跳变
        initial_radar_timeout_s=2.0, # 启动前最多等 2 秒收集有效前方雷达
        acquire_speed_mps=0.03,      # 锁不到雷达时，用 0.03m/s 小步找回波
        acquire_max_distance_m=0.6,  # 内部搜索上限，不是让用户输入走多少米
        acquire_step_duration_s=0.4, # 每次搜索脉冲约 1.2cm，脉冲之间停车观察
        acquire_timeout_s=20,        # 最长搜索 20 秒，仍无回波就退出
        lost_radar_timeout_s=1.0,    # 短时丢雷达先停等恢复，超过 1 秒才失败
        allow_estop_pedal_fault=True,# 当前机器已知硬件故障，现场看护时允许
        on_sample=print_sample,      # 可选：实时打印每一帧距离
    )

print(result)
```

## 核心方法参数

业务代码推荐用 `approach_to_rack()`，它固定按 `500mm` 停车，不需要再传
“走多少米”或停车距离：

```python
result = dock.approach_to_rack(
    speed_mps=0.05,
    max_duration_s=80,
    hz=10,
    history_size=3,
    initial_radar_timeout_s=2.0,
    acquire_speed_mps=0.03,
    acquire_max_distance_m=0.6,
    acquire_step_duration_s=0.4,
    acquire_timeout_s=20,
    lost_radar_timeout_s=1.0,
    allow_estop_pedal_fault=True,
    on_sample=None,
)
```

如果临时调试要改停车距离，才用通用方法：

```python
result = dock.approach_until_distance(
    stop_mm=500,
    speed_mps=0.05,
    max_duration_s=80,
    hz=10,
    history_size=3,
    initial_radar_timeout_s=2.0,
    acquire_if_needed=True,
    acquire_speed_mps=0.03,
    acquire_max_distance_m=0.6,
    acquire_step_duration_s=0.4,
    acquire_timeout_s=20,
    lost_radar_timeout_s=1.0,
    allow_estop_pedal_fault=True,
    on_sample=None,
)
```

- `stop_mm`：停车距离，单位 mm。现在业务默认是 `500`，即料架前 0.5m 停车。
- `speed_mps`：前进速度，单位 m/s。`0.02` 慢速调试，`0.05` 正常速度。
- `max_duration_s`：最长运行时间。超时后会停车并返回 `timeout`。
- `hz`：控制频率。`10` 已经实机验证稳定。
- `history_size`：中位数滤波窗口。`3` 可以过滤单帧雷达跳变。
  启动前会先静止采样 `history_size` 帧；少于 `history_size` 帧时不会进入主靠近段。
  如果稳定滤波距离已经小于 `500mm`，不会请求底盘运动。
- `initial_radar_timeout_s`：启动前最多等待多久收集有效雷达。等待期间不移动；默认 `2.0s`。
- `acquire_speed_mps`：锁不到前方雷达时的搜索速度，默认 `0.03m/s`。
- `acquire_max_distance_m`：搜索阶段内部前进上限，默认 `0.6m`；这不是用户输入的行走距离，而是安全保护。
- `acquire_step_duration_s`：每次搜索脉冲时间，默认 `0.4s`，每次只动很小一段。
- `acquire_timeout_s`：搜索阶段最长时间，默认 `20s`。
- `lost_radar_timeout_s`：丢雷达容忍时间。丢雷达时会先发零速度，超过这个时间仍无雷达才失败。
- `allow_estop_pedal_fault`：当前这台 G2 的已知急停踏板故障需要填 `True`；换正常机器应填 `False`。
- `on_sample`：可选回调，用来打印或记录每帧雷达数据。

`result.status` 的含义：

- `stopped`：达到阈值，已自动停车
- `already_at_threshold`：启动时已经小于等于阈值，没有移动
- `timeout`：到达最大运行时间，还没到阈值
- `lost_radar`：运行中丢失有效前方雷达数据，已停车
- `acquire_timeout`：启动时锁不到前方雷达，小步搜索到内部上限仍没有锁定目标，已停车

## 注意

这个类只封装已经验证稳定的前向靠近停车，不再使用 `relative_move`。
如果启动时前方雷达没有回波，`approach_to_rack()` 会小步低速搜索回波；
搜索仍失败就返回 `acquire_timeout`，不会一直前进。
运行中如果短时间丢失雷达数据，类会先发零速度并等待恢复；默认持续
`1.0s` 没有有效前方雷达才返回 `lost_radar`。
如需精细横向对齐，应在停车后结合视觉/毫米波/人工标定再做上层相对位姿计算。
