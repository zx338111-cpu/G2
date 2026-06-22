# ChassisController 使用说明

G2 机器人底盘控制封装类，文件路径：
- 机器人端：`/home/agi/chassis_controller.py`
- 本地备份：`/home/davie/G2/chassis_controller.py`

连接机器人：`ssh agi@10.20.15.169`（密码：`<robot-password>`）

---

## 运行前准备

```bash
source /home/agi/app/env.sh
```

---

## 导入与初始化

```python
from chassis_controller import ChassisController

# 方式一：with 语句（推荐，自动释放 GDK）
with ChassisController() as ctrl:
    ctrl.move_forward(1.0)

# 方式二：手动管理
ctrl = ChassisController()
ctrl.move_forward(1.0)
ctrl.release()
```

---

## 方法一览

### 精确移动（闭环，基于死算里程计）

| 方法 | 说明 | 示例 |
|------|------|------|
| `move_forward(distance_m)` | 向前移动，负值后退 | `ctrl.move_forward(1.0)` |
| `move_backward(distance_m)` | 向后移动 | `ctrl.move_backward(0.5)` |
| `crab_left(distance_m)` | 向左蟹行 | `ctrl.crab_left(1.0)` |
| `crab_right(distance_m)` | 向右蟹行 | `ctrl.crab_right(1.5)` |
| `rotate(angle_deg)` | 原地旋转，正值逆时针，负值顺时针 | `ctrl.rotate(90)` |
| `move_diagonal(x_m, y_m)` | 斜向移动，x 前后，y 正左负右 | `ctrl.move_diagonal(1.0, 0.5)` |
| `move_sequence(steps)` | 按顺序执行多个动作 | 见下方示例 |

所有精确移动方法均有 `timeout` 参数（默认 30 秒），返回任务最终 state（9=成功）。

### 速度开环控制（遥控风格）

| 方法 | 说明 |
|------|------|
| `velocity_control(vx, vy, vz, duration, mode, hz)` | 按指定速度持续运动 duration 秒 |
| `stop()` | 立即停止 |

参数说明：
- `vx`：前后速度 m/s，正值前进
- `vy`：左右速度 m/s，正值向左
- `vz`：旋转速度 rad/s，正值逆时针
- `duration`：持续时间（秒）
- `mode`：0=阿克曼，1=蟹行（默认 1）
- `hz`：发送频率（默认 20Hz）

### 状态查询

```python
ts = ctrl.get_task_state()
print(ts.id, ts.state, ts.type)
```

task state 含义：2=运行中，7=已取消，9=完成

---

## 使用示例

### 单步移动

```python
with ChassisController() as ctrl:
    ctrl.move_forward(1.0)       # 前进 1m
    ctrl.move_backward(0.5)      # 后退 0.5m
    ctrl.crab_left(1.0)          # 左蟹行 1m
    ctrl.crab_right(1.5)         # 右蟹行 1.5m
    ctrl.rotate(90)              # 逆时针 90°
    ctrl.rotate(-45)             # 顺时针 45°
    ctrl.move_diagonal(1.0, 0.5) # 斜向前进 1m 同时向左 0.5m
```

### 序列执行

```python
with ChassisController() as ctrl:
    ctrl.move_sequence([
        {"action": "forward",    "distance": 1.0},
        {"action": "crab_right", "distance": 0.5},
        {"action": "rotate",     "angle": -90},
        {"action": "backward",   "distance": 1.0},
        {"action": "crab_left",  "distance": 0.5},
        {"action": "rotate",     "angle": 90},
        {"action": "diagonal",   "x": 0.5, "y": 0.3},
    ])
```

### 速度开环

```python
with ChassisController() as ctrl:
    ctrl.velocity_control(vx=0.3, duration=2.0)   # 前进 2 秒
    ctrl.velocity_control(vy=0.3, duration=1.5)   # 左蟹行 1.5 秒
    ctrl.velocity_control(vy=-0.3, duration=1.5)  # 右蟹行 1.5 秒
    ctrl.velocity_control(vz=0.5, duration=1.0)   # 旋转 1 秒
    ctrl.stop()
```

### 走正方形

```python
with ChassisController() as ctrl:
    for _ in range(4):
        ctrl.move_forward(1.0)
        ctrl.rotate(90)
```

### 异常处理

```python
with ChassisController() as ctrl:
    state = ctrl.move_forward(1.0, timeout=15.0)
    if state != 9:
        print(f"移动失败或超时，state={state}")
        ctrl.stop()
```

---

## 注意事项

- **精确移动**（`move_forward` 等）基于 `relative_move` + 死算里程计，不依赖 SLAM，可靠性高
- **速度开环**（`velocity_control`）不保证精度，适合实时遥控场景
- 每次调用精确移动前会自动取消正在运行的任务，无需手动处理
- 旋转精度依赖 IMU，长时间运行后可能有漂移
