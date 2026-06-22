# 前激光雷达料架靠近类使用说明

## 使用场景

- 机器人离料架约 `1m-6m`，超声波前方 `0,1,2,3` 没有稳定回波
- 料架在机器人正前方，需要靠近到料架前 `0.5m` 自动停车
- 不需要用户输入走多少米，距离由前激光雷达点云实时计算

## 已验证点云方向

- 前激光雷达：`agibot_gdk.LidarType.kLidarFront`
- 底盘前进方向：raw 点云 `+X`
- 横向：raw 点云 `Y`
- 高度：raw 点云 `Z`
- 当前现场短距离持续发送速度验证：3 秒前进约 `0.16m`
- 当前较稳定的料架候选：`+X`、`Z>0.6m` 的高处结构点簇

## 机器人端运行环境

```bash
source /home/agi/app/env.sh
```

## 先只读前激光雷达

```bash
python3 /home/agi/rack_lidar_docking_demo.py \
  --read-only \
  --samples 10
```

只读输出示例字段：

```text
distance_m=4.132 nearest_m=2.542 cluster_points=14 roi_points=224 bin=[4.00,4.25)
```

- `distance_m`：最近稳定点簇距离，主控制使用
- `nearest_m`：ROI 内最近单点距离，只做近距离安全参考
- `cluster_points`：当前点簇点数
- `roi_points`：整个前方 ROI 点数
- `bin`：被选中点簇所在的前向距离箱

## 执行 0.5m 自动靠近

当前这台 G2 有官方确认的急停踏板已知硬件故障，所以需要显式加
`--allow-estop-pedal-fault`：

```bash
python3 /home/agi/rack_lidar_docking_demo.py \
  --execute \
  --allow-estop-pedal-fault \
  --stop-m 0.5 \
  --speed 0.05 \
  --max-duration 90 \
  --hz 5 \
  --history 3 \
  --initial-lidar-timeout 3.0 \
  --lost-lidar-timeout 1.0 \
  --nearest-safety-stop-m 0.0
```

第一次现场验证可以先把 `--stop-m` 临时设成当前距离附近的较大值，比如
`2.5`，确认距离下降和停车逻辑正确后，再执行 `0.5`。

## 在代码里调用：最小版本

```python
from rack_lidar_docking import RackLidarDockingController

with RackLidarDockingController() as dock:
    result = dock.approach_to_rack(
        speed_mps=0.05,
        max_duration_s=90,
        allow_estop_pedal_fault=True,
    )
    print(result.status, result.filtered_m)
```

## 在代码里调用：带实时日志版本

```python
from rack_lidar_docking import RackLidarDockingController


def print_sample(sample):
    print(
        f"t={sample.elapsed_s:.1f}s "
        f"distance={sample.distance_m:.3f}m "
        f"filtered={sample.filtered_m:.3f}m "
        f"nearest={sample.nearest_m:.3f}m "
        f"cluster_points={sample.cluster_points}"
    )


with RackLidarDockingController(
    control_mode=0,
    forward_axis="x",
    lateral_axis="y",
    forward_sign=1.0,
    lateral_half_width_m=0.8,
    z_min_m=0.6,
    z_max_m=1.2,
    min_range_m=0.2,
    max_range_m=6.0,
    bin_width_m=0.25,
    min_cluster_points=20,
) as dock:
    result = dock.approach_to_rack(
        speed_mps=0.05,
        max_duration_s=90,
        hz=5,
        history_size=3,
        initial_lidar_timeout_s=3.0,
        lost_lidar_timeout_s=1.0,
        allow_estop_pedal_fault=True,
        on_sample=print_sample,
    )

print(result)
```

## 核心参数

- `stop_m`：停车距离，单位 m。业务默认 `0.5`。
- `speed_mps`：前进速度，单位 m/s。`0.03` 可用于第一次保守验证，`0.05` 可用于正常低速靠近。
- `forward_axis/lateral_axis/forward_sign`：点云坐标轴。现场默认用 `+X` 前向、`Y` 横向。
- `lateral_half_width_m`：前方 ROI 横向半宽。默认 `0.8m`。
- `z_min_m/z_max_m`：高度 ROI。默认 `0.6m` 到 `1.2m`，用于排除地面和车体近点。
- `bin_width_m`：点簇分箱宽度。默认 `0.25m`。
- `min_cluster_points`：最近稳定点簇至少需要的点数。默认 `20`，用于过滤稀疏边缘点。
- `history_size`：中位数滤波窗口。默认 `3`。
- `initial_lidar_timeout_s`：启动前收集稳定点簇的等待时间。等待期间不移动。
- `lost_lidar_timeout_s`：运行中丢失稳定点簇时的容忍时间；丢失期间会先停车。
- `nearest_safety_stop_m`：ROI 内最近单点的近距离安全停车阈值，默认 `0` 表示禁用。
  当前点云有车体/自反射近点，业务停车默认使用稳定点簇距离。

`result.status` 的含义：

- `stopped`：达到阈值，已自动停车
- `already_at_threshold`：启动时已经小于等于阈值，没有移动
- `timeout`：到达最大运行时间，还没到阈值
- `lost_lidar`：运行中丢失稳定前方点簇，已停车

## 和超声类的关系

- `rack_lidar_docking.py`：适合从几米外靠近料架，当前优先使用
- `rack_radar_docking.py`：使用超声波雷达，适合近距离且前方超声有稳定回波时使用

目前机器人端没有发现 GDK Python 暴露的毫米波接口；如果后续找到毫米波 topic/API，
可以把 `read_rack_distance()` 的数据源替换成毫米波，底盘控制和停车状态机可以保留。
