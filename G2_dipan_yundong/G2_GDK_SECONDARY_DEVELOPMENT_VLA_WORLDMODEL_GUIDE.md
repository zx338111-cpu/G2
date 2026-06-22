# G2 GDK 二次开发、VLA 与世界模型工程手册

> 版本：2026-06-18  
> 机器人：Agibot G2 / G02  
> 当前实机：`agi@10.185.207.191`  
> 运行栈：`genie_g02_rb_2.2.0_314fa4fb_2026-03-24-05-30-19_thor.tar.gz`  
> GDK：`agibot_gdk/2.6.3@genie`  
> 目标：把这台 G2 当成可二开的机器人平台，统一说明 GDK、IO、传感器、运动控制、VLA 和世界模型怎么接。

这份文档按这台机器人的真实运行状态写，不是通用 SDK 摘抄。当前实机的事实是：主服务和 GDK 链路是活的，上半身运动状态健康，视频/音频旁路能用；但底盘导航当前不能直接用，因为 `charge_plug_insert_state=1`，SLAM 无 `curr_pose/odom`，没有完整的 `map -> base_link` 定位链路。

## 1. 这台 G2 的系统分层

```text
用户程序 / Web / VLA / 世界模型
  -> agibot_gdk Python API / DDS topic / HTTP tunnel
  -> gdk_service / gdk_http_server / task_manager / pico_adapter
  -> AORTA + Cosine + FastDDS discovery
  -> genie_motion_control / quark_navigation / SLAM / DR / camera / lidar
  -> HAL / hal_lowerlimb
  -> EtherCAT / CAN / SPI / power board / motors / cameras / lidars / audio
```

实机上的关键路径：

- `/home/agi/app/bin/run.sh`：主启动脚本。
- `/home/agi/app/conf/sys/run.conf`：运行环境变量。
- `/home/agi/app/conf/manifest.d/base.json`：默认 scene 模块。
- `/data/logs/latest`：当前 boot 日志。
- `/data/parameters`：硬件、传感器、外参、工具参数。
- `/home/agi/app/gdk/examples/python`：官方/厂家 GDK 示例。

当前主要服务：

- `genie_app.service`：主应用服务。
- `gdk_service` / `gdk_http_server`：GDK 和 HTTP 服务。
- `genie_motion_control`：上半身/全身运动控制。
- `quark_navigation`：底盘规划控制。
- `slam_state_machine` / `dr_state_machine` / `tagloc_state_machine`：定位相关。
- `hal` / `hal_lowerlimb`：上半身和底盘硬件抽象层。
- `camera_service` / `camera_dlb` / `lidar`：传感器输入。

## 2. 开发环境

在机器人上跑 GDK Python 脚本，优先使用系统 Python：

```bash
ssh agi@10.185.207.191
source /home/agi/app/env.sh
/usr/bin/python3 your_script.py
```

原因：

- `agibot_gdk` 是装在机器人运行环境里的，不一定存在于你的 venv。
- `env.sh` 会设置 `LD_LIBRARY_PATH`、`PYTHONPATH`、`APP_CONF_PATH` 等 GDK 必需变量。
- 不要在没有确认的情况下用 venv Python 直接 import GDK。

最小 GDK 初始化模板：

```python
#!/usr/bin/env python3
import agibot_gdk

def main():
    result = agibot_gdk.gdk_init()
    if result not in (None, agibot_gdk.GDKRes.kSuccess):
        raise RuntimeError(f"GDK init failed: {result}")
    try:
        robot = agibot_gdk.Robot()
        status = robot.get_whole_body_status()
        print(status)
    finally:
        agibot_gdk.gdk_release()

if __name__ == "__main__":
    main()
```

## 3. GDK 核心对象

这台 G2 当前可用的 GDK 类：

| 类 | 用途 | 常用方法 |
|---|---|---|
| `Robot` | 上半身、关节、电源、运动控制状态 | `get_joint_states`, `get_whole_body_status`, `get_motion_control_status`, `get_chassis_power_state`, `move_head_joint`, `move_arm_joint`, `end_effector_pose_control` |
| `Pnc` | 底盘和导航任务 | `get_task_state`, `request_chassis_control`, `move_chassis`, `relative_move`, `normal_navi`, `high_precision_navi`, `cancel_task` |
| `Slam` | 定位和建图状态 | `get_slam_state`, `get_curr_pose`, `get_odom_info`, `start_mapping`, `stop_mapping` |
| `Map` | 地图管理 | `get_curr_map`, `get_all_map`, `switch_map`, `get_map` |
| `TF` | 坐标变换 | `get_tf_from_base_link`, `lookup_transform`, `can_transform`, `get_all_frame_names` |
| `Camera` | 相机图像 | `get_latest_image`, `get_nearest_image`, `get_image_shape` |
| `Lidar` | 点云 | `get_latest_pointcloud`, `get_nearest_pointcloud` |
| `Imu` | IMU | `get_latest_imu`, `get_nearest_imu` |

建议封装顺序：

```text
G2Runtime
  - RobotIO: Robot / power / joint / gripper / head / arm
  - ChassisIO: Pnc / Slam / Map / TF / chassis safety gate
  - SensorIO: Camera / Lidar / Imu / audio tunnel status
  - WorldModel: state fusion / object memory / task context
  - PolicyRunner: VLA observation -> action -> safety -> execution
```

## 4. 只读状态快照示例

这个脚本不会运动，只读取机器人状态。

```python
#!/usr/bin/env python3
import json
import agibot_gdk

def as_dict(obj):
    if isinstance(obj, dict):
        return obj
    return repr(obj)

def main():
    agibot_gdk.gdk_init()
    try:
        robot = agibot_gdk.Robot()
        pnc = agibot_gdk.Pnc()
        slam = agibot_gdk.Slam()
        maps = agibot_gdk.Map()
        tf = agibot_gdk.TF()

        out = {}
        out["whole_body"] = as_dict(robot.get_whole_body_status())
        out["motion_control"] = repr(robot.get_motion_control_status())
        out["chassis_power"] = repr(robot.get_chassis_power_state())
        out["pnc_task"] = repr(pnc.get_task_state())

        try:
            out["slam_state"] = slam.get_slam_state()
        except Exception as exc:
            out["slam_state_error"] = f"{type(exc).__name__}: {exc}"

        try:
            out["curr_pose"] = repr(slam.get_curr_pose())
        except Exception as exc:
            out["curr_pose_error"] = f"{type(exc).__name__}: {exc}"

        try:
            out["odom"] = repr(slam.get_odom_info())
        except Exception as exc:
            out["odom_error"] = f"{type(exc).__name__}: {exc}"

        try:
            curr = maps.get_curr_map()
            out["map"] = {"id": getattr(curr, "id", None), "name": getattr(curr, "name", "")}
        except Exception as exc:
            out["map_error"] = f"{type(exc).__name__}: {exc}"

        out["tf_can_odom_base"] = tf.can_transform("odom", "base_link")
        out["tf_can_map_base"] = tf.can_transform("map", "base_link")

        print(json.dumps(out, ensure_ascii=False, indent=2))
    finally:
        agibot_gdk.gdk_release()

if __name__ == "__main__":
    main()
```

对当前机器人的预期现象：

- `motion_control.error_code` 应该是 `0`。
- `Pnc.get_task_state()` 当前是 idle。
- `Map.get_curr_map()` 当前是 id `20`。
- `Slam.get_curr_pose()` 当前会失败。
- `Slam.get_odom_info()` 当前会报 `Slam odom is null`。
- `tf.can_transform("map", "base_link")` 当前是 false。

## 5. 运动前安全门禁

任何会动的脚本都应该先做门禁。不要只看 `motion_control_error=0`。

硬门禁建议：

```text
Robot.get_motion_control_status().error_code == 0
Robot.get_whole_body_status() 关键错误字段 == 0
Robot.get_chassis_power_state().emergency_stop_pedal_state == 0
Pnc.get_task_state().state in idle-like states
若要走地图导航：Slam.get_curr_pose() 成功且 Slam.get_odom_info() 成功
若要走底盘：charge_plug_insert_state 必须确认不是插枪状态
```

当前机器人的实际阻塞：

```text
charge_plug_insert_state=1
GetCurrPose failed
Slam odom is null
```

所以现在不能直接跑底盘导航或隧道行走。

只读门禁函数：

```python
def build_motion_gate(robot, pnc, slam=None, require_slam=False):
    problems = []

    whole = robot.get_whole_body_status()
    for key in [
        "right_arm_error", "left_arm_error",
        "right_end_error", "left_end_error",
        "waist_error", "lift_error", "neck_error", "chassis_error",
    ]:
        if int(whole.get(key, 0)) != 0:
            problems.append(f"{key}={whole.get(key)}")

    mc = robot.get_motion_control_status()
    if int(getattr(mc, "error_code", 0)) != 0:
        problems.append(f"motion_control_error={getattr(mc, 'error_code', None)}")

    power = robot.get_chassis_power_state()
    if int(getattr(power, "emergency_stop_pedal_state", 0)) != 0:
        problems.append("emergency_stop_pedal_state!=0")
    if int(getattr(power, "charge_plug_insert_state", 0)) != 0:
        problems.append("charge_plug_insert_state=1")

    task = pnc.get_task_state()
    if getattr(task, "state", None) not in (0, 3, 6, 7, 8, 9):
        problems.append(f"pnc_task_not_idle={getattr(task, 'state', None)}")

    if require_slam:
        try:
            slam.get_curr_pose()
        except Exception as exc:
            problems.append(f"pose_unavailable={type(exc).__name__}: {exc}")
        try:
            slam.get_odom_info()
        except Exception as exc:
            problems.append(f"odom_unavailable={type(exc).__name__}: {exc}")

    return {"ok": not problems, "problems": problems}
```

## 6. 传感器 IO 示例

### 6.1 读取头部相机

```python
#!/usr/bin/env python3
import cv2
import numpy as np
import agibot_gdk

def image_to_bgr(img):
    data = np.frombuffer(bytes(img.data), dtype=np.uint8)
    h = int(getattr(img, "height", 0))
    w = int(getattr(img, "width", 0))

    # 部分 GDK 相机返回 RGB raw，部分场景可能返回 JPEG buffer。
    if h > 0 and w > 0 and data.size == h * w * 3:
        rgb = data.reshape(h, w, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError("cannot decode image")
    return bgr

def main():
    agibot_gdk.gdk_init()
    try:
        camera = agibot_gdk.Camera()
        img = camera.get_latest_image(agibot_gdk.CameraType.kHeadColor, 1000.0)
        bgr = image_to_bgr(img)
        cv2.imwrite("/tmp/g2_head_color.jpg", bgr)
        print("saved /tmp/g2_head_color.jpg", bgr.shape)
    finally:
        agibot_gdk.gdk_release()

if __name__ == "__main__":
    main()
```

注意：短生命周期 `Camera()` 客户端刚启动时，`get_image_fps()` 可能返回 0，因为还没有统计窗口。判断实时相机是否健康，优先取 `get_latest_image()` 或用常驻 viewer 的 `/status`。

### 6.2 读取末端 TF、夹爪和关节

```python
#!/usr/bin/env python3
import json
import numpy as np
import agibot_gdk

LEFT_EE_FRAME = "arm_l_end_link"
RIGHT_EE_FRAME = "arm_r_end_link"
GRIPPER_RAW_OPEN_MM = 120.0
GRIPPER_NORM_OPEN = -0.78

def quat_norm(q):
    q = np.asarray(q, dtype=np.float32)
    n = np.linalg.norm(q)
    return q / n if n > 1e-8 else np.array([0, 0, 0, 1], dtype=np.float32)

def ee_pose(tf, frame):
    t = tf.get_tf_from_base_link(frame)
    q = quat_norm([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
    return {
        "position": [t.translation.x, t.translation.y, t.translation.z],
        "orientation_xyzw": q.tolist(),
    }

def gripper_norm(robot, side):
    key = f"{side}_end_state"
    state = robot.get_end_state()
    end_states = state.get(key, {}).get("end_states", [])
    if not end_states:
        return None
    raw_mm = float(end_states[0].get("position", 0.0))
    return {
        "raw_mm": raw_mm,
        "norm_rad": raw_mm * (GRIPPER_NORM_OPEN / GRIPPER_RAW_OPEN_MM),
    }

def main():
    agibot_gdk.gdk_init()
    try:
        robot = agibot_gdk.Robot()
        tf = agibot_gdk.TF()
        js = robot.get_joint_states()
        out = {
            "left_ee": ee_pose(tf, LEFT_EE_FRAME),
            "right_ee": ee_pose(tf, RIGHT_EE_FRAME),
            "left_gripper": gripper_norm(robot, "left"),
            "right_gripper": gripper_norm(robot, "right"),
            "joint_count": len(js.get("states", [])),
            "joint_errors": [
                [s.get("name"), s.get("error_code")]
                for s in js.get("states", [])
                if int(s.get("error_code", 0)) != 0
            ],
        }
        print(json.dumps(out, ensure_ascii=False, indent=2))
    finally:
        agibot_gdk.gdk_release()

if __name__ == "__main__":
    main()
```

### 6.3 读取地图、SLAM 和 TF

```python
#!/usr/bin/env python3
import json
import agibot_gdk

def safe_call(fn):
    try:
        return {"ok": True, "value": repr(fn())}
    except Exception as exc:
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}

def main():
    agibot_gdk.gdk_init()
    try:
        slam = agibot_gdk.Slam()
        maps = agibot_gdk.Map()
        tf = agibot_gdk.TF()

        curr_map = safe_call(maps.get_curr_map)
        out = {
            "curr_map": curr_map,
            "all_maps": safe_call(maps.get_all_map),
            "slam_state": safe_call(slam.get_slam_state),
            "curr_pose": safe_call(slam.get_curr_pose),
            "odom": safe_call(slam.get_odom_info),
            "can_odom_base": tf.can_transform("odom", "base_link"),
            "can_map_base": tf.can_transform("map", "base_link"),
            "frames": tf.get_all_frame_names(),
        }
        print(json.dumps(out, ensure_ascii=False, indent=2))
    finally:
        agibot_gdk.gdk_release()

if __name__ == "__main__":
    main()
```

## 7. 运动控制示例

### 7.1 头部运动

头部运动也属于物理运动，必须确认安全后再执行。

```python
#!/usr/bin/env python3
import time
import agibot_gdk

def move_head(robot, yaw, roll, pitch, vel=0.15):
    # 这台 G2 的头部关节顺序在现有脚本里按 [yaw, roll, pitch] 使用。
    return robot.move_head_joint([yaw, roll, pitch], [vel, vel, vel])

def main():
    agibot_gdk.gdk_init()
    try:
        robot = agibot_gdk.Robot()
        # 小幅动作示例：确认现场安全后再运行。
        for pose in [(0.0, 0.0, 0.0), (0.12, 0.0, 0.0), (-0.12, 0.0, 0.0), (0.0, 0.0, 0.0)]:
            print(move_head(robot, *pose))
            time.sleep(0.8)
    finally:
        agibot_gdk.gdk_release()

if __name__ == "__main__":
    main()
```

### 7.2 底盘遥控必须带 watchdog

`Pnc.move_chassis()` 是实时速度控制，不应该让网页或策略直接裸发。正确做法：

```text
浏览器 / 上位机
  -> 每 50-100ms 发送期望速度和 heartbeat
  -> 机器人端 TeleopServer
  -> 检查 gate + 限速 + TTL
  -> Pnc.move_chassis(twist)
  -> heartbeat 超时立即发 0 速度，必要时 cancel_task
```

安全骨架：

```python
class ChassisWatchdog:
    def __init__(self, agibot_gdk, pnc, max_vx=0.15, max_vy=0.08, max_wz=0.20, ttl_s=0.25):
        self.gdk = agibot_gdk
        self.pnc = pnc
        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_wz = max_wz
        self.ttl_s = ttl_s
        self.last_cmd_time = 0.0

    def make_twist(self, vx, vy, wz):
        twist = self.gdk.Twist()
        twist.linear = self.gdk.Vector3()
        twist.angular = self.gdk.Vector3()
        twist.linear.x = max(-self.max_vx, min(self.max_vx, float(vx)))
        twist.linear.y = max(-self.max_vy, min(self.max_vy, float(vy)))
        twist.angular.z = max(-self.max_wz, min(self.max_wz, float(wz)))
        return twist

    def send(self, vx, vy, wz):
        import time
        self.last_cmd_time = time.time()
        self.pnc.move_chassis(self.make_twist(vx, vy, wz))

    def stop(self):
        import time
        zero = self.make_twist(0.0, 0.0, 0.0)
        for _ in range(8):
            self.pnc.move_chassis(zero)
            time.sleep(0.03)

    def check_timeout(self):
        import time
        if time.time() - self.last_cmd_time > self.ttl_s:
            self.stop()
```

当前这台机器人的底盘不要直接跑这个，因为 SLAM 和 `charge_plug_insert_state` 当前不满足门禁。若要隧道里低速人工遥控，需要先明确这是“无地图低速遥控模式”，并额外引入超声/激光障碍停止。

## 8. VLA：怎么把 G2 接到视觉语言动作模型

### 8.1 推荐数据契约

这台 G2 已经跑过 OpenPI/pi0.5 风格的双臂数据链路。推荐从最稳定的末端空间开始，而不是直接学习 22 个关节。

Observation：

```text
images:
  observation/head_image:        uint8 HWC, 224x224x3
  observation/left_wrist_image:  uint8 HWC, 224x224x3
  observation/right_wrist_image: uint8 HWC, 224x224x3

state 16D:
  left_ee_pos(3)
  left_ee_quat_xyzw(4)
  left_gripper(1)
  right_ee_pos(3)
  right_ee_quat_xyzw(4)
  right_gripper(1)

language:
  prompt: "pick up the aluminum profile"
```

Action：

```text
action 14D:
  left_dx, left_dy, left_dz
  left_droll, left_dpitch, left_dyaw
  left_gripper
  right_dx, right_dy, right_dz
  right_droll, right_dpitch, right_dyaw
  right_gripper
```

夹爪约定：

```text
GDK get_end_state raw: 120.0 mm = 全开，0.0 mm = 全闭
训练 normalized: -0.78 = 全开，0.0 = 全闭
normalized = raw_mm * (-0.78 / 120.0)
```

### 8.2 VLA 数据采集 episode 结构

```text
episode_000001/
  arrays.npz
    states:     (T, 16)
    actions:    (T, 14)
    ee_poses:   (T, 14)
    grippers:   (T, 2)
    timestamps: (T,)
  frames.jsonl
  images/
    head_color_000000.jpg
    hand_left_000000.jpg
    hand_right_000000.jpg
  preview_head_color.jpg
manifest.csv
```

采集原则：

- 每条 episode 起点一致，例如 Home 位。
- 图像、state、action 必须同频或带 timestamp。
- 失败 episode 可以保留，但要明确标注 `success=false`。
- 不要混用不同 gripper 语义的数据集。
- 不要把底盘、手臂、夹爪、头部动作混在同一个未标注 action 维度里。

### 8.3 构建 VLA observation 示例

```python
import cv2
import numpy as np
import agibot_gdk

GRIPPER_RAW_OPEN_MM = 120.0
GRIPPER_NORM_OPEN = -0.78

def capture_rgb224(camera, camera_type):
    img = camera.get_latest_image(camera_type, 1000.0)
    if img is None:
        return np.zeros((224, 224, 3), dtype=np.uint8)

    data = np.frombuffer(bytes(img.data), dtype=np.uint8)
    h = int(getattr(img, "height", 0))
    w = int(getattr(img, "width", 0))

    if h > 0 and w > 0 and data.size == h * w * 3:
        rgb = data.reshape(h, w, 3).copy()
    else:
        bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if bgr is None:
            return np.zeros((224, 224, 3), dtype=np.uint8)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    if rgb.shape[:2] != (224, 224):
        rgb = cv2.resize(rgb, (224, 224), interpolation=cv2.INTER_AREA)
    return rgb.astype(np.uint8)

def normalize_quat(q):
    q = np.asarray(q, dtype=np.float32)
    n = np.linalg.norm(q)
    return q / n if n > 1e-8 else np.array([0, 0, 0, 1], dtype=np.float32)

def get_ee_pose_7d(tf, frame):
    t = tf.get_tf_from_base_link(frame)
    q = normalize_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
    return np.array([
        t.translation.x,
        t.translation.y,
        t.translation.z,
        q[0], q[1], q[2], q[3],
    ], dtype=np.float32)

def get_gripper_norm(robot, side):
    state = robot.get_end_state()
    end_states = state.get(f"{side}_end_state", {}).get("end_states", [])
    if not end_states:
        return 0.0
    raw_mm = float(end_states[0].get("position", 0.0))
    return raw_mm * (GRIPPER_NORM_OPEN / GRIPPER_RAW_OPEN_MM)

def build_vla_observation(camera, tf, robot, prompt):
    head = capture_rgb224(camera, agibot_gdk.CameraType.kHeadColor)
    left = capture_rgb224(camera, agibot_gdk.CameraType.kHandLeftColor)
    right = capture_rgb224(camera, agibot_gdk.CameraType.kHandRightColor)

    left_pose = get_ee_pose_7d(tf, "arm_l_end_link")
    right_pose = get_ee_pose_7d(tf, "arm_r_end_link")
    left_grip = get_gripper_norm(robot, "left")
    right_grip = get_gripper_norm(robot, "right")

    state = np.concatenate([
        left_pose, [left_grip],
        right_pose, [right_grip],
    ]).astype(np.float32)

    return {
        "observation/head_image": head,
        "observation/left_wrist_image": left,
        "observation/right_wrist_image": right,
        "observation/state": state,
        "prompt": prompt,
    }
```

### 8.4 VLA 动作执行安全层

模型输出永远不能直接发给机器人。至少做四层处理：

1. shape/dtype 检查：必须是 `(14,)` 或 action chunk `(N, 14)`。
2. 数值检查：不能有 NaN/Inf。
3. 单步限幅：每步 `dx/dy/dz/drot/grip` 都限幅。
4. 状态门禁：动作执行前后都读 motion-control、关节错误、夹爪状态。

示例：

```python
def sanitize_action14(action):
    a = np.asarray(action, dtype=np.float32).reshape(-1)
    if a.shape != (14,):
        raise ValueError(f"expected action14, got {a.shape}")
    if not np.isfinite(a).all():
        raise ValueError("action contains NaN/Inf")

    # left xyz, right xyz
    for base in (0, 7):
        a[base + 0] = np.clip(a[base + 0], -0.03, 0.03)
        a[base + 1] = np.clip(a[base + 1], -0.03, 0.03)
        a[base + 2] = np.clip(a[base + 2], -0.04, 0.04)
        a[base + 3] = np.clip(a[base + 3], -0.17, 0.17)
        a[base + 4] = np.clip(a[base + 4], -0.17, 0.17)
        a[base + 5] = np.clip(a[base + 5], -0.17, 0.17)
        a[base + 6] = np.clip(a[base + 6], -0.78, 0.0)
    return a
```

### 8.5 VLA 推理闭环

```text
while task_running:
  observation = read_g2_observation()
  action_chunk = policy.infer(observation)
  for action in action_chunk[:k]:
    action = sanitize_action14(action)
    if not gate.ok:
      stop_or_hold()
      break
    execute_arm_delta(action)
    sleep(policy_dt)
```

执行频率建议：

- policy 推理：5-10Hz。
- EE servo：20-50Hz 内部插值。
- 每个 policy step 的末端增量要小，不要让模型一次跳大距离。

## 9. 世界模型：G2 上应该怎么做

这里的“世界模型”不要先理解成大模型本身，而是机器人运行时维护的一份结构化世界状态。VLA 看图出动作，但世界模型负责把多帧、多传感器、多任务状态变成稳定上下文。

### 9.1 世界模型在这台 G2 上的职责

```text
传感器输入:
  head camera / wrist cameras / lidar / imu / slam / TF / joint / gripper / power

世界模型:
  robot_state
  localization_state
  scene_objects
  map_context
  task_state
  safety_state
  short_term_memory

输出:
  VLA prompt/context
  task planner state
  safety gate decision
  visualization/debug log
```

为什么需要世界模型：

- 单帧图像容易误判，世界模型可以做跨帧稳定。
- VLA 不应该自己记所有安全状态，安全状态应由世界模型/执行器维护。
- 隧道环境下 SLAM 可能不稳，世界模型可以降级成局部 ego-frame 记忆。
- 工业抓取任务里，物体、货架、障碍、已完成杆件，需要一个任务状态表。

### 9.2 推荐状态 schema

```python
from dataclasses import dataclass, field
from typing import Any

@dataclass
class RobotState:
    timestamp: float
    joint_count: int
    joint_errors: list[tuple[str, int]]
    left_ee: Any | None
    right_ee: Any | None
    left_gripper: float | None
    right_gripper: float | None
    motion_control_error: int | None
    pnc_task_state: int | None

@dataclass
class LocalizationState:
    map_id: int | None = None
    slam_state: int | None = None
    pose_ok: bool = False
    odom_ok: bool = False
    curr_pose_repr: str | None = None
    odom_repr: str | None = None
    can_map_base: bool = False
    can_odom_base: bool = False

@dataclass
class SceneObject:
    object_id: str
    label: str
    confidence: float
    frame: str
    position: list[float] | None = None
    bbox_xyxy: list[float] | None = None
    last_seen: float = 0.0
    attributes: dict[str, Any] = field(default_factory=dict)

@dataclass
class SafetyState:
    ok_for_arm: bool
    ok_for_chassis: bool
    blockers: list[str]

@dataclass
class G2WorldModel:
    robot: RobotState | None = None
    localization: LocalizationState = field(default_factory=LocalizationState)
    objects: dict[str, SceneObject] = field(default_factory=dict)
    task: dict[str, Any] = field(default_factory=dict)
    safety: SafetyState | None = None
    memory: list[dict[str, Any]] = field(default_factory=list)
```

### 9.3 世界模型更新流程

```text
每 50-100ms:
  读取 Robot/Pnc 基础状态
  更新 safety_state

每 100-200ms:
  读取 head/wrist 图像
  跑检测/分割/目标跟踪
  更新 scene_objects

每 200-500ms:
  读取 SLAM/Map/TF
  更新 localization_state

每个任务 step:
  把 world_model 摘要转成 VLA prompt/context
  把 VLA action 经过 world_model safety gate
  执行动作或拒绝动作
```

### 9.4 世界模型最小实现示例

```python
import time
import json

class G2WorldModelRuntime:
    def __init__(self, robot, pnc, slam, maps, tf):
        self.robot = robot
        self.pnc = pnc
        self.slam = slam
        self.maps = maps
        self.tf = tf
        self.state = G2WorldModel()

    def update_robot_state(self):
        now = time.time()
        js = self.robot.get_joint_states()
        joint_errors = [
            (s.get("name", ""), int(s.get("error_code", 0)))
            for s in js.get("states", [])
            if int(s.get("error_code", 0)) != 0
        ]
        mc = self.robot.get_motion_control_status()
        task = self.pnc.get_task_state()

        left_ee = None
        right_ee = None
        try:
            left_ee = repr(self.tf.get_tf_from_base_link("arm_l_end_link"))
            right_ee = repr(self.tf.get_tf_from_base_link("arm_r_end_link"))
        except Exception:
            pass

        self.state.robot = RobotState(
            timestamp=now,
            joint_count=len(js.get("states", [])),
            joint_errors=joint_errors,
            left_ee=left_ee,
            right_ee=right_ee,
            left_gripper=None,
            right_gripper=None,
            motion_control_error=int(getattr(mc, "error_code", -1)),
            pnc_task_state=getattr(task, "state", None),
        )

    def update_localization(self):
        loc = LocalizationState()
        try:
            curr_map = self.maps.get_curr_map()
            loc.map_id = getattr(curr_map, "id", None)
        except Exception:
            pass
        try:
            loc.slam_state = self.slam.get_slam_state()
        except Exception:
            pass
        try:
            loc.curr_pose_repr = repr(self.slam.get_curr_pose())
            loc.pose_ok = True
        except Exception:
            loc.pose_ok = False
        try:
            loc.odom_repr = repr(self.slam.get_odom_info())
            loc.odom_ok = True
        except Exception:
            loc.odom_ok = False
        loc.can_map_base = self.tf.can_transform("map", "base_link")
        loc.can_odom_base = self.tf.can_transform("odom", "base_link")
        self.state.localization = loc

    def update_safety(self):
        blockers = []
        if self.state.robot is None:
            blockers.append("robot_state_missing")
        else:
            if self.state.robot.joint_errors:
                blockers.append(f"joint_errors={self.state.robot.joint_errors}")
            if self.state.robot.motion_control_error not in (0, None):
                blockers.append(f"motion_control_error={self.state.robot.motion_control_error}")

        try:
            power = self.robot.get_chassis_power_state()
            if int(getattr(power, "emergency_stop_pedal_state", 0)) != 0:
                blockers.append("emergency_stop_pedal_state!=0")
            if int(getattr(power, "charge_plug_insert_state", 0)) != 0:
                blockers.append("charge_plug_insert_state=1")
        except Exception as exc:
            blockers.append(f"power_state_unavailable={type(exc).__name__}: {exc}")

        ok_for_arm = not any("joint_errors" in b or "motion_control_error" in b for b in blockers)
        ok_for_chassis = (
            ok_for_arm
            and not blockers
            and self.state.localization.pose_ok
            and self.state.localization.odom_ok
        )
        self.state.safety = SafetyState(
            ok_for_arm=ok_for_arm,
            ok_for_chassis=ok_for_chassis,
            blockers=blockers,
        )

    def tick(self):
        self.update_robot_state()
        self.update_localization()
        self.update_safety()
        return self.state

    def to_vla_context(self):
        s = self.state
        return {
            "robot": None if s.robot is None else {
                "joint_count": s.robot.joint_count,
                "joint_errors": s.robot.joint_errors,
                "motion_control_error": s.robot.motion_control_error,
                "pnc_task_state": s.robot.pnc_task_state,
            },
            "localization": {
                "map_id": s.localization.map_id,
                "pose_ok": s.localization.pose_ok,
                "odom_ok": s.localization.odom_ok,
                "can_map_base": s.localization.can_map_base,
                "can_odom_base": s.localization.can_odom_base,
            },
            "objects": [
                {
                    "id": obj.object_id,
                    "label": obj.label,
                    "confidence": obj.confidence,
                    "frame": obj.frame,
                    "position": obj.position,
                    "bbox_xyxy": obj.bbox_xyxy,
                }
                for obj in s.objects.values()
            ],
            "safety": None if s.safety is None else {
                "ok_for_arm": s.safety.ok_for_arm,
                "ok_for_chassis": s.safety.ok_for_chassis,
                "blockers": s.safety.blockers,
            },
            "task": s.task,
        }
```

### 9.5 世界模型怎么接 VLA

不要把完整世界模型全部塞进语言 prompt。推荐分两层：

1. 模型 observation：图像 + state，保持训练契约稳定。
2. planner/context：世界模型摘要，用来选择 prompt、检查 action、决定是否允许执行。

示例：

```python
world = wm.tick()
context = wm.to_vla_context()

if not context["safety"]["ok_for_arm"]:
    raise RuntimeError("arm action blocked: " + ",".join(context["safety"]["blockers"]))

prompt = "pick up the aluminum profile"
if context["task"].get("phase") == "place":
    prompt = "place the aluminum profile into the rack slot"

obs = build_vla_observation(camera, tf, robot, prompt)
obs["world_context"] = context  # 只有支持额外上下文的策略才使用；pi0.5 训练契约默认不依赖它。
action = policy.infer(obs)
```

### 9.6 隧道场景的世界模型

隧道里有两个模式：

**模式 A：有 SLAM/地图定位**

- 世界模型维护 `map_id/current_pose/odom`。
- 可做地图导航、路径跟踪、障碍停止。
- VLA 可以只负责局部动作或语义决策。

**模式 B：无 SLAM，纯人工低速遥控**

- 世界模型降级为 ego-frame 局部记忆。
- 不允许 `normal_navi/high_precision_navi`。
- 只能低速 `move_chassis`，强 watchdog。
- 需要近场障碍输入：超声、激光、TOF 或视觉障碍检测。
- 视频延迟、音频延迟、控制 heartbeat 都要进入 `safety_state`。

当前这台 G2 处于更接近模式 B 的状态，因为 SLAM pose/odom 当前不可用。但因为 `charge_plug_insert_state=1`，即使模式 B 也不能直接启动底盘。

## 10. 任务工程化建议

### 10.1 抓取类任务

推荐架构：

```text
WorldModel 识别/记忆目标
  -> VLA 生成左/右臂 action
  -> ActionSanitizer 限幅
  -> ArmExecutor 执行 EE servo / gripper
  -> WorldModel 更新目标是否被抓起
```

不要让 VLA 同时管：

- 地图导航。
- 急停和电源状态。
- DDS/GDK 异常恢复。
- 长距离底盘行走。

这些应该由外层工程代码管。

### 10.2 隧道遥控

推荐架构：

```text
HeadTunnelViewer:
  head video + audio + latency stats

TeleopControlServer:
  browser/gamepad command
  heartbeat
  speed limit
  watchdog zero velocity

WorldModel:
  chassis safety
  SLAM/odom availability
  obstacle state
  network/media latency

Executor:
  Pnc.request_chassis_control
  Pnc.move_chassis
  repeated zero stop
```

核心原则：

- 媒体流和运动流分开。
- 媒体卡顿不能延长运动命令。
- 运动命令必须有 TTL。
- 失联先停，不做补偿性追赶。

## 11. 推荐目录结构

```text
g2_app/
  README.md
  g2_runtime/
    gdk_init.py
    robot_io.py
    sensor_io.py
    chassis_io.py
    safety.py
    world_model.py
  vla/
    observation.py
    action.py
    collector.py
    policy_client.py
    executor.py
  tunnel/
    media_status.py
    teleop_server.py
    watchdog.py
  scripts/
    readonly_snapshot.py
    capture_observation.py
    run_vla_dryrun.py
    run_tunnel_readiness.py
```

## 12. 当前机器人的下一步开发路线

第一阶段：只读稳定化

- 固化 `readonly_snapshot.py`。
- 固化 `capture_observation.py`，能保存 head/wrist 图像和 16D state。
- 固化世界模型 tick，持续输出 JSONL。

第二阶段：VLA 数据链

- 用一致 Home 位采集 episode。
- 统一 gripper 语义。
- 转 LeRobot/OpenPI。
- 训练/推理保持同一个 observation/action contract。

第三阶段：执行闭环

- 先做单臂/双臂无底盘任务。
- 所有 action 通过 sanitizer。
- 每一步执行前后记录 world_model。

第四阶段：隧道底盘

- 先修 `charge_plug_insert_state` 和 SLAM/odom。
- 若 SLAM 不可靠，明确做低速无地图 teleop。
- 做运动 server-side watchdog。
- 用世界模型记录媒体延迟、控制心跳、障碍状态。

## 13. 最重要的边界

- GDK 是控制入口，不是安全保证本身。
- VLA 是动作建议器，不是机器人安全控制器。
- 世界模型是运行时事实表，不是替代底层 HAL/PNC/SLAM。
- 底盘走隧道前，必须把定位、电源、充电插头、急停、障碍停止、网络失联停机全部闭环。
