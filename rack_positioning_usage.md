# 上料架 / 下料架相对定位使用说明

## 核心思路

不要直接用固定距离硬走到料架，而是按下面流程闭环：

1. 传感器或人工测量得到当前料架相对机器人位姿：`rack_x / rack_y / rack_yaw`
2. `rack_positioning.py` 计算当前机器人到目标对接位的残差
3. 用 `ChassisController.move_relative(x, y, yaw)` 通过 `relative_move` 执行小步修正
4. 重新测量料架位姿，继续修正，直到误差进入容差

这仍然使用当前底盘类的 PNC/DR 闭环控制，不依赖 `move_chassis` 开环速度。

## 坐标约定

机器人坐标系：

- `x` 正值：向前
- `y` 正值：向左
- `yaw` 正值：逆时针

料架对接坐标系：

- `rack_pose_in_robot` 表示料架对接坐标系在当前机器人坐标系下的位置
- 示例中假设料架对接坐标系的朝向就是机器人对齐后应该保持的朝向
- `standoff` 表示机器人最终停在料架对接原点后方多少米

例如：

```bash
python3 rack_positioning_demo.py \
  --rack load \
  --rack-x 1.20 \
  --rack-y 0.04 \
  --rack-yaw 2.0 \
  --standoff 0.65
```

含义：当前上料架对接点在机器人前方 `1.20m`、左侧 `0.04m`、相对偏航 `2deg`；目标是停在对接点后方 `0.65m`。

## 本地 dry-run

默认不运动，只打印规划结果：

```bash
python3 rack_positioning_demo.py --rack load --rack-x 1.20 --rack-y 0.04 --rack-yaw 2.0
python3 rack_positioning_demo.py --rack unload --rack-x 0.90 --rack-y -0.03 --rack-yaw -1.0
```

输出里的 `raw` 是一次性残差，`cmd` 是经过小步限幅后的实际命令。默认单步最大平移 `0.25m`，最大旋转 `12deg`。

## 机器人上执行

确认现场安全、料架位姿测量可信后，再加 `--execute`：

```bash
source /home/agi/app/env.sh
python3 rack_positioning_demo.py \
  --rack load \
  --rack-x 1.20 \
  --rack-y 0.04 \
  --rack-yaw 2.0 \
  --standoff 0.65 \
  --execute
```

第一次建议把 `--max-step` 降到 `0.10`：

```bash
python3 rack_positioning_demo.py \
  --rack load \
  --rack-x 1.20 \
  --rack-y 0.04 \
  --rack-yaw 2.0 \
  --standoff 0.65 \
  --max-step 0.10 \
  --max-yaw-step 5 \
  --execute
```

## 后续接入真实检测

实际闭环时，把视觉、AprilTag、激光或人工测量函数接到：

```python
from rack_positioning import Pose2D, RackRelativePositioner, DEFAULT_PROFILES
from chassis_controller import ChassisController

def measure_load_rack_pose():
    # TODO: return current measured rack pose in robot frame
    return Pose2D(x=1.20, y=0.04, yaw_deg=2.0)

with ChassisController() as ctrl:
    positioner = RackRelativePositioner(controller=ctrl, dry_run=False)
    results = positioner.align_closed_loop(
        measure_rack_pose_fn=measure_load_rack_pose,
        profile=DEFAULT_PROFILES["load"],
        max_iterations=4,
    )
```

每动一步后必须重新测量，不要把第一次测量值重复用于多轮闭环。
