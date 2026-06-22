#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
g2_fetch_bottle.py  —  G2机器人：导航到A点 → 视觉识别瓶子 → 抓取 → 导航到B点 → 放置

整合子系统：
  1. SLAM导航  (Pnc.normal_navi / relative_move)
  2. 相机视觉  (Camera.get_latest_image + OpenCV)
  3. 末端位姿伺服控制 (Robot.end_effector_pose_control, 50Hz)
  4. 夹爪控制  (Robot.move_ee_pos)

用法：
  python3 g2_fetch_bottle.py --config task_config.json
  python3 g2_fetch_bottle.py --ax 2.0 --ay 1.0 --bx 5.0 --by 3.0 --arm left

依赖：
  agibot_gdk, numpy, opencv-python (cv2)

作者备注：
  - 视觉检测部分提供了 颜色检测 / YOLO 两种方案，默认用颜色检测
  - 瓶子3D坐标通过 深度相机 + 相机内参 + TF 计算得到
  - end_effector_pose_control 必须在伺服模式 (mode=1/5) 下使用
  - 所有API调用均对齐官方SDK示例 (pick_bottle_servo.py, slam_demo.py, etc.)
"""

import argparse
import json
import math
import time
import sys
import os
import signal
import threading
from enum import Enum, auto
from typing import Optional, Tuple, List, Dict, Any

import numpy as np

try:
    import cv2
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False
    print("⚠️  OpenCV 未安装，视觉检测功能不可用")
    print("    pip install opencv-python")

import agibot_gdk


# ═══════════════════════════════════════════════════════════════════
#  全局常量
# ═══════════════════════════════════════════════════════════════════

LEFT_EE_FRAME  = "arm_l_end_link"
RIGHT_EE_FRAME = "arm_r_end_link"

# EndEffectorControlGroup 数值兜底 (SDK版本兼容)
GROUP_LEFT  = 4
GROUP_RIGHT = 8
GROUP_BOTH  = 12

# 22个关节名称 (参考 mc_example.py)
JOINT_NAMES = [
    "idx01_body_joint1", "idx02_body_joint2",
    "idx03_body_joint3", "idx04_body_joint4", "idx05_body_joint5",
    "idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3",
    "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
    "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
    "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
    "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]


# ═══════════════════════════════════════════════════════════════════
#  任务状态机
# ═══════════════════════════════════════════════════════════════════

class TaskState(Enum):
    INIT            = auto()   # 初始化
    NAVI_TO_A       = auto()   # 导航到A点
    WAIT_NAVI_A     = auto()   # 等待导航A完成
    DETECT_BOTTLE   = auto()   # 视觉检测瓶子
    APPROACH_BOTTLE = auto()   # 手臂接近瓶子
    GRASP_BOTTLE    = auto()   # 抓取瓶子
    LIFT_BOTTLE     = auto()   # 抬起瓶子
    NAVI_TO_B       = auto()   # 导航到B点
    WAIT_NAVI_B     = auto()   # 等待导航B完成
    PLACE_BOTTLE    = auto()   # 放置瓶子
    RELEASE_BOTTLE  = auto()   # 松开夹爪
    RETRACT_ARM     = auto()   # 手臂回收
    DONE            = auto()   # 完成
    ERROR           = auto()   # 错误


# ═══════════════════════════════════════════════════════════════════
#  配置
# ═══════════════════════════════════════════════════════════════════

class TaskConfig:
    """任务配置参数"""

    # ---- 导航目标 (map坐标系) ----
    # A点：取瓶子的位置
    point_a_position    = [2.0, 1.0, 0.0]
    point_a_orientation = [0.0, 0.0, 0.0, 1.0]  # 四元数 xyzw

    # B点：放瓶子的位置
    point_b_position    = [5.0, 3.0, 0.0]
    point_b_orientation = [0.0, 0.0, 0.0, 1.0]

    # ---- 导航参数 ----
    navi_timeout_s      = 120.0   # 导航超时
    navi_poll_hz        = 2.0     # 轮询导航状态频率

    # ---- 使用的手臂 ----
    arm = "left"   # "left" or "right"

    # ---- 手臂伺服控制参数 ----
    servo_rate_hz  = 50.0    # 50Hz (文档要求)
    servo_lifetime = 0.02    # 单条指令生命周期(s)
    move_duration  = 1.5     # 每段运动时长(s)

    # ---- 抓取参数 ----
    approach_height = 0.12   # 瓶子上方接近高度(m)
    grasp_down      = 0.01   # 下探偏移(m)
    lift_height     = 0.18   # 抬升高度(m)
    place_height    = 0.05   # 放置时高于桌面的高度(m)

    # ---- 夹爪参数 ----
    gripper_open   = 0.0     # 张开位置(rad)
    gripper_close  = 0.6     # 闭合位置(rad)

    # ---- 视觉检测参数 ----
    detect_camera      = "kHeadColor"     # 用于检测的相机
    depth_camera       = "kHeadDepth"     # 深度相机
    detect_timeout_ms  = 1000.0           # 取图超时(ms)
    detect_max_retries = 10               # 最大检测重试次数
    detect_method      = "color"          # "color" or "yolo"

    # HSV颜色范围 (用于简单颜色检测, 默认: 蓝色瓶盖)
    hsv_lower = [90, 50, 50]
    hsv_upper = [130, 255, 255]
    min_contour_area = 500   # 最小轮廓面积(像素)

    # YOLO模型路径 (如果 detect_method == "yolo")
    yolo_model_path = "yolov8n.pt"
    yolo_target_class = "bottle"
    yolo_confidence   = 0.5

    # ---- 放置位置 (base_link 坐标系, 手动指定) ----
    # 如果不用视觉检测放置位置, 直接指定末端放置坐标
    place_position_base = None  # [x, y, z] or None (用当前位置下方)

    # ---- 头部朝向 (用于低头看桌面) ----
    head_look_down = [0.0, 0.3, 0.0]       # [joint1, joint2, joint3]
    head_look_forward = [0.0, 0.0, 0.0]
    head_velocity = [0.2, 0.2, 0.2]

    @classmethod
    def from_json(cls, path: str) -> "TaskConfig":
        cfg = cls()
        with open(path, "r") as f:
            data = json.load(f)
        for k, v in data.items():
            if hasattr(cfg, k):
                setattr(cfg, k, v)
        return cfg

    @classmethod
    def from_args(cls, args) -> "TaskConfig":
        cfg = cls()
        if args.ax is not None:
            cfg.point_a_position = [args.ax, args.ay, args.az]
        if args.ao is not None:
            cfg.point_a_orientation = args.ao
        if args.bx is not None:
            cfg.point_b_position = [args.bx, args.by, args.bz]
        if args.bo is not None:
            cfg.point_b_orientation = args.bo
        if args.arm:
            cfg.arm = args.arm
        return cfg


# ═══════════════════════════════════════════════════════════════════
#  数学工具
# ═══════════════════════════════════════════════════════════════════

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _quat_norm(q):
    """四元数归一化 [x,y,z,w]"""
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [x/n, y/n, z/n, w/n]


def _quat_slerp(q0, q1, t):
    """球面线性插值 (shortest path)"""
    q0 = _quat_norm(q0)
    q1 = list(_quat_norm(q1))

    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = [-x for x in q1]
        dot = -dot
    dot = _clamp(dot, -1.0, 1.0)

    if dot > 0.9995:
        out = [q0[i] + t * (q1[i] - q0[i]) for i in range(4)]
        return _quat_norm(out)

    theta0 = math.acos(dot)
    sin0 = math.sin(theta0)
    theta = theta0 * t
    sinT = math.sin(theta)

    s0 = math.cos(theta) - dot * sinT / sin0
    s1 = sinT / sin0
    return _quat_norm([s0 * q0[i] + s1 * q1[i] for i in range(4)])


def _lerp3(a, b, t):
    """3D线性插值"""
    return [a[i] + (b[i] - a[i]) * t for i in range(3)]


def _sleep_to_rate(next_t: float, period: float) -> float:
    """精确定频休眠"""
    now = time.monotonic()
    if next_t > now:
        time.sleep(next_t - now)
    return next_t + period


# ═══════════════════════════════════════════════════════════════════
#  GDK 工具函数 (对齐官方示例)
# ═══════════════════════════════════════════════════════════════════

def _read_tf(tf_obj: agibot_gdk.TF, frame: str):
    """
    读取TF变换 → (位置列表, 四元数列表)
    对齐: print_ee_tf.py, pick_bottle_servo.py
    """
    t = tf_obj.get_tf_from_base_link(frame)
    pos = [t.translation.x, t.translation.y, t.translation.z]
    quat = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
    return pos, quat


def _get_group_constants():
    """
    获取 EndEffectorControlGroup 枚举值, 兼容不同SDK版本
    对齐: pick_bottle_servo.py
    """
    grp = getattr(agibot_gdk, "EndEffectorControlGroup", None)
    if grp is None:
        return GROUP_LEFT, GROUP_RIGHT, GROUP_BOTH
    return (
        getattr(grp, "kLeftArm", GROUP_LEFT),
        getattr(grp, "kRightArm", GROUP_RIGHT),
        getattr(grp, "kBothArms", GROUP_BOTH),
    )


def _make_navi_target(position, orientation):
    """
    构造 NaviReq 导航请求
    对齐: pnc_example.py, slam_demo.py
    """
    target = agibot_gdk.NaviReq()
    target.target.position.x = float(position[0])
    target.target.position.y = float(position[1])
    target.target.position.z = float(position[2])
    target.target.orientation.x = float(orientation[0])
    target.target.orientation.y = float(orientation[1])
    target.target.orientation.z = float(orientation[2])
    target.target.orientation.w = float(orientation[3])
    return target


def _make_twist(lx=0.0, ly=0.0, az=0.0):
    """
    构造 Twist 速度指令
    对齐: move_chassis.py — 必须显式创建 Vector3()
    """
    twist = agibot_gdk.Twist()
    twist.linear = agibot_gdk.Vector3()
    twist.angular = agibot_gdk.Vector3()
    twist.linear.x = float(lx)
    twist.linear.y = float(ly)
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = float(az)
    return twist


# ═══════════════════════════════════════════════════════════════════
#  视觉检测模块
# ═══════════════════════════════════════════════════════════════════

class BottleDetector:
    """
    瓶子视觉检测器
    支持两种方式: 颜色检测 / YOLO
    """

    def __init__(self, camera: agibot_gdk.Camera, cfg: TaskConfig):
        self.camera = camera
        self.cfg = cfg

        # 解析相机类型
        self.color_cam_type = getattr(
            agibot_gdk.CameraType, cfg.detect_camera, agibot_gdk.CameraType.kHeadColor
        )
        self.depth_cam_type = getattr(
            agibot_gdk.CameraType, cfg.depth_camera, agibot_gdk.CameraType.kHeadDepth
        )

        # 获取相机内参
        self._intrinsic = None
        try:
            intr = self.camera.get_camera_intrinsic(self.color_cam_type)
            self._intrinsic = {
                "fx": intr.intrinsic[0],
                "fy": intr.intrinsic[1],
                "cx": intr.intrinsic[2],
                "cy": intr.intrinsic[3],
            }
            print(f"[视觉] 相机内参: fx={self._intrinsic['fx']:.1f}, "
                  f"fy={self._intrinsic['fy']:.1f}, "
                  f"cx={self._intrinsic['cx']:.1f}, "
                  f"cy={self._intrinsic['cy']:.1f}")
        except Exception as e:
            print(f"[视觉] ⚠️ 无法获取相机内参: {e}")
            print("[视觉] 将使用默认内参 (可能不准确)")
            self._intrinsic = {"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0}

        # YOLO模型 (如果需要)
        self._yolo_model = None
        if cfg.detect_method == "yolo":
            self._load_yolo()

    def _load_yolo(self):
        """加载YOLO模型"""
        try:
            from ultralytics import YOLO
            self._yolo_model = YOLO(self.cfg.yolo_model_path)
            print(f"[视觉] YOLO模型加载成功: {self.cfg.yolo_model_path}")
        except ImportError:
            print("[视觉] ⚠️ ultralytics 未安装, 回退到颜色检测")
            print("    pip install ultralytics")
            self.cfg.detect_method = "color"
        except Exception as e:
            print(f"[视觉] ⚠️ YOLO加载失败: {e}, 回退到颜色检测")
            self.cfg.detect_method = "color"

    def _decode_image(self, image) -> Optional[np.ndarray]:
        """
        解码GDK Image → OpenCV BGR numpy数组
        对齐: camera_web_viewer.py
        """
        if image is None or not hasattr(image, 'data') or image.data is None:
            return None
        if len(image.data) == 0:
            return None

        try:
            if image.encoding == agibot_gdk.Encoding.JPEG:
                nparr = np.frombuffer(image.data, np.uint8)
                return cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            elif image.encoding == agibot_gdk.Encoding.PNG:
                nparr = np.frombuffer(image.data, np.uint8)
                return cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            elif image.encoding == agibot_gdk.Encoding.UNCOMPRESSED:
                if image.color_format == agibot_gdk.ColorFormat.RGB:
                    arr = np.frombuffer(image.data, dtype=np.uint8)
                    arr = arr.reshape((image.height, image.width, 3))
                    return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                elif image.color_format == agibot_gdk.ColorFormat.BGR:
                    arr = np.frombuffer(image.data, dtype=np.uint8)
                    return arr.reshape((image.height, image.width, 3))
                elif image.color_format == agibot_gdk.ColorFormat.GRAY8:
                    arr = np.frombuffer(image.data, dtype=np.uint8)
                    arr = arr.reshape((image.height, image.width))
                    return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        except Exception as e:
            print(f"[视觉] 图像解码失败: {e}")
        return None

    def _decode_depth(self, image) -> Optional[np.ndarray]:
        """
        解码深度图像 → float32 numpy (单位: 米)
        """
        if image is None or not hasattr(image, 'data') or len(image.data) == 0:
            return None
        try:
            if image.encoding == agibot_gdk.Encoding.UNCOMPRESSED:
                if image.color_format == agibot_gdk.ColorFormat.GRAY16 or \
                   image.color_format == agibot_gdk.ColorFormat.RS2_FORMAT_Z16:
                    arr = np.frombuffer(image.data, dtype=np.uint16)
                    arr = arr.reshape((image.height, image.width))
                    return arr.astype(np.float32) / 1000.0  # mm → m
            # 尝试通用方式
            nparr = np.frombuffer(image.data, np.uint8)
            decoded = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
            if decoded is not None and decoded.dtype == np.uint16:
                return decoded.astype(np.float32) / 1000.0
        except Exception as e:
            print(f"[视觉] 深度图解码失败: {e}")
        return None

    def detect_bottle_color(self, bgr_image: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """
        颜色检测瓶子 → (cx, cy, w, h) 像素坐标
        """
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        lower = np.array(self.cfg.hsv_lower, dtype=np.uint8)
        upper = np.array(self.cfg.hsv_upper, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        # 形态学处理
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # 选最大轮廓
        best = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)
        if area < self.cfg.min_contour_area:
            return None

        x, y, w, h = cv2.boundingRect(best)
        cx = x + w // 2
        cy = y + h // 2
        return (cx, cy, w, h)

    def detect_bottle_yolo(self, bgr_image: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """
        YOLO检测瓶子 → (cx, cy, w, h)
        """
        if self._yolo_model is None:
            return None

        results = self._yolo_model(bgr_image, conf=self.cfg.yolo_confidence, verbose=False)
        if not results or len(results) == 0:
            return None

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = r.names[cls_id]
                if cls_name == self.cfg.yolo_target_class:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    w = int(x2 - x1)
                    h = int(y2 - y1)
                    conf = float(box.conf[0])
                    print(f"[视觉] YOLO检测到瓶子: ({cx},{cy}) {w}x{h} conf={conf:.2f}")
                    return (cx, cy, w, h)
        return None

    def pixel_to_3d(self, px: int, py: int, depth_m: float) -> List[float]:
        """
        像素坐标 + 深度 → 相机坐标系3D点 [x, y, z]
        使用针孔模型: X = (px - cx) * Z / fx, Y = (py - cy) * Z / fy
        """
        fx = self._intrinsic["fx"]
        fy = self._intrinsic["fy"]
        cx = self._intrinsic["cx"]
        cy = self._intrinsic["cy"]
        X = (px - cx) * depth_m / fx
        Y = (py - cy) * depth_m / fy
        Z = depth_m
        return [X, Y, Z]

    def detect(self) -> Optional[Dict[str, Any]]:
        """
        完整检测流程：取彩色图 + 深度图 → 检测 → 3D坐标
        返回: {"pixel": (cx,cy), "depth_m": float, "camera_3d": [x,y,z]} 或 None
        """
        if not HAS_OPENCV:
            print("[视觉] ❌ OpenCV不可用")
            return None

        # 获取彩色图
        try:
            color_img = self.camera.get_latest_image(
                self.color_cam_type, self.cfg.detect_timeout_ms
            )
        except Exception as e:
            print(f"[视觉] 获取彩色图失败: {e}")
            return None

        bgr = self._decode_image(color_img)
        if bgr is None:
            print("[视觉] 彩色图解码失败")
            return None

        # 检测瓶子
        if self.cfg.detect_method == "yolo":
            det = self.detect_bottle_yolo(bgr)
        else:
            det = self.detect_bottle_color(bgr)

        if det is None:
            return None

        cx, cy, w, h = det
        print(f"[视觉] 检测到瓶子像素: ({cx}, {cy}), 大小: {w}x{h}")

        # 获取深度图
        depth_m = 0.5  # 默认深度 (m)
        try:
            depth_img = self.camera.get_latest_image(
                self.depth_cam_type, self.cfg.detect_timeout_ms
            )
            depth_arr = self._decode_depth(depth_img)
            if depth_arr is not None:
                # 在检测中心点附近取深度中值 (更鲁棒)
                h_d, w_d = depth_arr.shape
                h_c, w_c = bgr.shape[:2]
                dx = int(cx * w_d / w_c)
                dy = int(cy * h_d / h_c)
                radius = 5
                y_lo = max(0, dy - radius)
                y_hi = min(h_d, dy + radius)
                x_lo = max(0, dx - radius)
                x_hi = min(w_d, dx + radius)
                patch = depth_arr[y_lo:y_hi, x_lo:x_hi]
                valid = patch[patch > 0.05]  # 过滤无效深度
                if len(valid) > 0:
                    depth_m = float(np.median(valid))
                    print(f"[视觉] 深度: {depth_m:.3f} m")
                else:
                    print(f"[视觉] ⚠️ 深度无效, 使用默认值 {depth_m:.2f} m")
            else:
                print(f"[视觉] ⚠️ 深度图解码失败, 使用默认值 {depth_m:.2f} m")
        except Exception as e:
            print(f"[视觉] ⚠️ 获取深度图失败: {e}, 使用默认值 {depth_m:.2f} m")

        # 像素 → 相机坐标系 3D
        cam_3d = self.pixel_to_3d(cx, cy, depth_m)
        print(f"[视觉] 相机坐标系3D: ({cam_3d[0]:.3f}, {cam_3d[1]:.3f}, {cam_3d[2]:.3f})")

        return {
            "pixel": (cx, cy),
            "bbox": (cx, cy, w, h),
            "depth_m": depth_m,
            "camera_3d": cam_3d,
        }


# ═══════════════════════════════════════════════════════════════════
#  手臂控制模块
# ═══════════════════════════════════════════════════════════════════

class ArmController:
    """
    手臂伺服控制器
    对齐: pick_bottle_servo.py, pick_bottle_demo.py 的官方文档API风格
    """

    def __init__(self, robot: agibot_gdk.Robot, tf: agibot_gdk.TF, cfg: TaskConfig):
        self.robot = robot
        self.tf = tf
        self.cfg = cfg
        self.grp_left, self.grp_right, self.grp_both = _get_group_constants()

    def ensure_servo_mode(self) -> bool:
        """
        检查伺服模式 (mode=1/5)
        对齐: pick_bottle_servo.py ensure_servo_mode()
        """
        try:
            status = self.robot.get_motion_control_status()
            mode = int(status.mode)
            print(f"[手臂] 运动模式: mode={mode}, error_code={status.error_code}")
            if mode not in (1, 5):
                print("[手臂] ⚠️ 当前不是伺服模式 (需要 mode=1 或 mode=5)")
                print("       请在示教器/上位机切换到伺服模式后再运行")
                return False
            return True
        except Exception as e:
            print(f"[手臂] ❌ 获取运动状态失败: {e}")
            return False

    def get_current_pose(self, arm: str = None) -> Tuple[List[float], List[float]]:
        """
        获取当前末端位姿
        对齐: pick_bottle_servo.py _read_tf()
        """
        arm = arm or self.cfg.arm
        frame = LEFT_EE_FRAME if arm == "left" else RIGHT_EE_FRAME
        return _read_tf(self.tf, frame)

    def get_both_poses(self):
        """获取双臂当前位姿"""
        l_pos, l_q = _read_tf(self.tf, LEFT_EE_FRAME)
        r_pos, r_q = _read_tf(self.tf, RIGHT_EE_FRAME)
        return l_pos, l_q, r_pos, r_q

    def set_gripper(self, left: float = None, right: float = None):
        """
        设置夹爪位置, 必须同时提供左右
        对齐: gripper_test.py, robot_demo.py
        """
        cur_l, cur_r = self._get_gripper_pos()
        tgt_l = float(left) if left is not None else cur_l
        tgt_r = float(right) if right is not None else cur_r

        action = {
            "left_ee_state": {"joint_position": tgt_l},
            "right_ee_state": {"joint_position": tgt_r},
        }
        self.robot.move_ee_pos(action)
        print(f"[夹爪] 左={tgt_l:.2f}, 右={tgt_r:.2f}")

    def _get_gripper_pos(self) -> Tuple[float, float]:
        """
        获取当前夹爪位置
        对齐: robot_demo.py get_end_state()
        """
        left, right = 0.0, 0.0
        try:
            es = self.robot.get_end_state()
            for side, key in [("left", "left_end_state"), ("right", "right_end_state")]:
                state = es.get(key, {})
                end_states = state.get("end_states", [])
                if end_states:
                    pos = end_states[0].get("position", 0.0)
                    if side == "left":
                        left = float(pos)
                    else:
                        right = float(pos)
        except Exception:
            pass
        return left, right

    def open_gripper(self, arm: str = None):
        arm = arm or self.cfg.arm
        if arm == "left":
            self.set_gripper(left=self.cfg.gripper_open)
        else:
            self.set_gripper(right=self.cfg.gripper_open)

    def close_gripper(self, arm: str = None):
        arm = arm or self.cfg.arm
        if arm == "left":
            self.set_gripper(left=self.cfg.gripper_close)
        else:
            self.set_gripper(right=self.cfg.gripper_close)

    def servo_move(
        self,
        target_pos: List[float],
        target_quat: List[float],
        arm: str = None,
        duration_s: float = None,
    ):
        """
        50Hz伺服移动到目标位姿 (带插值, 无阶跃)

        使用官方文档 EndEffectorPose 结构 (left_end_effector_pose / right_end_effector_pose):
          end_pose.left_end_effector_pose.position.x = ...
          end_pose.left_end_effector_pose.orientation.x = ...

        对齐: pick_bottle_demo.py stream_pose(), 官方文档 end_effector_pose_control() 示例
        """
        arm = arm or self.cfg.arm
        duration_s = duration_s or self.cfg.move_duration
        rate_hz = self.cfg.servo_rate_hz
        lifetime = self.cfg.servo_lifetime
        period = 1.0 / rate_hz
        steps = max(1, int(duration_s * rate_hz))

        # 读取双臂当前位姿
        l_pos, l_q, r_pos, r_q = self.get_both_poses()

        if arm == "left":
            start_pos, start_q = l_pos, l_q
            group = self.grp_left
        else:
            start_pos, start_q = r_pos, r_q
            group = self.grp_right

        next_t = time.monotonic()
        for i in range(steps):
            t = (i + 1) / steps
            cur_pos = _lerp3(start_pos, target_pos, t)
            cur_q = _quat_slerp(start_q, target_quat, t)

            end_pose = agibot_gdk.EndEffectorPose()
            end_pose.life_time = float(lifetime)
            end_pose.group = int(group)

            if arm == "left":
                # 左臂: 插值移动
                end_pose.left_end_effector_pose.position.x = float(cur_pos[0])
                end_pose.left_end_effector_pose.position.y = float(cur_pos[1])
                end_pose.left_end_effector_pose.position.z = float(cur_pos[2])
                end_pose.left_end_effector_pose.orientation.x = float(cur_q[0])
                end_pose.left_end_effector_pose.orientation.y = float(cur_q[1])
                end_pose.left_end_effector_pose.orientation.z = float(cur_q[2])
                end_pose.left_end_effector_pose.orientation.w = float(cur_q[3])
                # 右臂: 保持不动
                end_pose.right_end_effector_pose.position.x = float(r_pos[0])
                end_pose.right_end_effector_pose.position.y = float(r_pos[1])
                end_pose.right_end_effector_pose.position.z = float(r_pos[2])
                end_pose.right_end_effector_pose.orientation.x = float(r_q[0])
                end_pose.right_end_effector_pose.orientation.y = float(r_q[1])
                end_pose.right_end_effector_pose.orientation.z = float(r_q[2])
                end_pose.right_end_effector_pose.orientation.w = float(r_q[3])
            else:
                # 左臂: 保持不动
                end_pose.left_end_effector_pose.position.x = float(l_pos[0])
                end_pose.left_end_effector_pose.position.y = float(l_pos[1])
                end_pose.left_end_effector_pose.position.z = float(l_pos[2])
                end_pose.left_end_effector_pose.orientation.x = float(l_q[0])
                end_pose.left_end_effector_pose.orientation.y = float(l_q[1])
                end_pose.left_end_effector_pose.orientation.z = float(l_q[2])
                end_pose.left_end_effector_pose.orientation.w = float(l_q[3])
                # 右臂: 插值移动
                end_pose.right_end_effector_pose.position.x = float(cur_pos[0])
                end_pose.right_end_effector_pose.position.y = float(cur_pos[1])
                end_pose.right_end_effector_pose.position.z = float(cur_pos[2])
                end_pose.right_end_effector_pose.orientation.x = float(cur_q[0])
                end_pose.right_end_effector_pose.orientation.y = float(cur_q[1])
                end_pose.right_end_effector_pose.orientation.z = float(cur_q[2])
                end_pose.right_end_effector_pose.orientation.w = float(cur_q[3])

            self.robot.end_effector_pose_control(end_pose)
            next_t = _sleep_to_rate(next_t, period)

        # 到位保持 0.3s (避免不稳)
        hold_steps = max(1, int(0.3 * rate_hz))
        for _ in range(hold_steps):
            self.robot.end_effector_pose_control(end_pose)
            next_t = _sleep_to_rate(next_t, period)

        # 打印最终位置
        final_pos, _ = self.get_current_pose(arm)
        print(f"[手臂] {arm} 末端到达: "
              f"({final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f})")


# ═══════════════════════════════════════════════════════════════════
#  导航控制模块
# ═══════════════════════════════════════════════════════════════════

class NaviController:
    """
    导航控制器
    对齐: pnc_example.py, slam_demo.py
    """

    def __init__(self, cfg: TaskConfig):
        self.cfg = cfg
        self.pnc = agibot_gdk.Pnc()
        self.slam = agibot_gdk.Slam()
        time.sleep(2)
        print("[导航] PNC + SLAM 初始化完成")

    def get_current_position(self) -> Optional[Dict]:
        """
        获取当前位置 (通过里程计)
        对齐: slam_demo.py get_odom_info()
        """
        try:
            odom = self.slam.get_odom_info()
            pos = {
                "x": odom.pose.pose.position.x,
                "y": odom.pose.pose.position.y,
                "z": odom.pose.pose.position.z,
            }
            ori = {
                "x": odom.pose.pose.orientation.x,
                "y": odom.pose.pose.orientation.y,
                "z": odom.pose.pose.orientation.z,
                "w": odom.pose.pose.orientation.w,
            }
            return {"position": pos, "orientation": ori,
                    "loc_state": odom.loc_state,
                    "loc_confidence": odom.loc_confidence}
        except Exception as e:
            print(f"[导航] 获取位置失败: {e}")
            return None

    def navigate_to(self, position: List[float], orientation: List[float],
                    label: str = ""):
        """
        发送导航请求 (异步)
        对齐: pnc_example.py
        """
        target = _make_navi_target(position, orientation)
        try:
            self.pnc.normal_navi(target)
            print(f"[导航] 导航请求已发送 → {label} "
                  f"({position[0]:.2f}, {position[1]:.2f})")
        except Exception as e:
            raise RuntimeError(f"导航请求失败: {e}")

    def wait_navigation_complete(self, timeout_s: float = None) -> bool:
        """
        等待导航完成
        task_state.state: 0=空闲, 1=执行中, 2=暂停, 3=成功, 4=失败, 5=取消
        对齐: slam_demo.py
        """
        timeout_s = timeout_s or self.cfg.navi_timeout_s
        poll_interval = 1.0 / self.cfg.navi_poll_hz
        t0 = time.monotonic()

        while time.monotonic() - t0 < timeout_s:
            try:
                task_state = self.pnc.get_task_state()
                state = task_state.state
                elapsed = time.monotonic() - t0

                if state == 3:  # 成功
                    print(f"[导航] ✅ 到达目标 (耗时 {elapsed:.1f}s)")
                    return True
                elif state == 4:  # 失败
                    print(f"[导航] ❌ 导航失败: {task_state.message}")
                    return False
                elif state == 5:  # 取消
                    print(f"[导航] ⚠️ 导航被取消")
                    return False
                elif state == 1:  # 执行中
                    if int(elapsed) % 5 == 0 and int(elapsed) > 0:
                        odom = self.get_current_position()
                        if odom:
                            print(f"[导航] 行进中... "
                                  f"({odom['position']['x']:.2f}, "
                                  f"{odom['position']['y']:.2f}) "
                                  f"置信度={odom['loc_confidence']:.2f}")
            except Exception as e:
                print(f"[导航] 查询状态异常: {e}")

            time.sleep(poll_interval)

        print(f"[导航] ⏱ 导航超时 ({timeout_s:.0f}s)")
        # 超时取消任务
        try:
            task_state = self.pnc.get_task_state()
            self.pnc.cancel_task(task_state.id)
        except Exception:
            pass
        return False

    def cancel_current_task(self):
        """取消当前导航任务"""
        try:
            task_state = self.pnc.get_task_state()
            self.pnc.cancel_task(task_state.id)
            print("[导航] 当前任务已取消")
        except Exception as e:
            print(f"[导航] 取消任务异常: {e}")


# ═══════════════════════════════════════════════════════════════════
#  坐标变换: 相机坐标 → base_link 坐标
# ═══════════════════════════════════════════════════════════════════

def camera_to_base_link(
    cam_point: List[float],
    tf_obj: agibot_gdk.TF,
    head_frame: str = "head_link3",
) -> Optional[List[float]]:
    """
    将相机坐标系的3D点转换到 base_link 坐标系

    流程:
    1. 相机坐标系 → head_link 坐标系 (近似: cam_z前→head_x, cam_x右→head_y, cam_y下→head_-z)
    2. head_link → base_link (通过TF)

    注意: 实际项目需用 SensorExtrinsicType 获取精确外参标定
    对齐: tf_check.py, print_ee_tf.py
    """
    try:
        t = tf_obj.get_tf_from_base_link(head_frame)
        hx = t.translation.x
        hy = t.translation.y
        hz = t.translation.z
        qx = t.rotation.x
        qy = t.rotation.y
        qz = t.rotation.z
        qw = t.rotation.w

        # 四元数 → 旋转矩阵
        R = _quat_to_rotation_matrix([qx, qy, qz, qw])

        # 相机坐标系 → head_link 坐标系的近似变换
        # 相机: x右, y下, z前 → head_link: x前, y左, z上
        cam_in_head = np.array([cam_point[2], -cam_point[0], -cam_point[1]])

        # head_link → base_link
        p_head = np.array([hx, hy, hz])
        p_base = R @ cam_in_head + p_head

        return p_base.tolist()

    except Exception as e:
        print(f"[坐标] ⚠️ 坐标变换失败: {e}")
        # 粗略估计 fallback
        return [cam_point[2] + 0.3, -cam_point[0], -cam_point[1] + 1.0]


def _quat_to_rotation_matrix(q):
    """四元数 [x,y,z,w] → 3x3旋转矩阵"""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),       1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),       2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


# ═══════════════════════════════════════════════════════════════════
#  主任务控制器 (状态机)
# ═══════════════════════════════════════════════════════════════════

class FetchBottleTask:
    """
    瓶子取放任务状态机

    流程:
    INIT → NAVI_TO_A → WAIT_NAVI_A → DETECT_BOTTLE → APPROACH_BOTTLE
    → GRASP_BOTTLE → LIFT_BOTTLE → NAVI_TO_B → WAIT_NAVI_B
    → PLACE_BOTTLE → RELEASE_BOTTLE → RETRACT_ARM → DONE
    """

    def __init__(self, cfg: TaskConfig):
        self.cfg = cfg
        self.state = TaskState.INIT
        self._running = True
        self._bottle_base_pos = None   # 瓶子在 base_link 下的坐标
        self._bottle_quat = None       # 抓取时保持的姿态
        self._error_msg = ""

        # 信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        print(f"\n[任务] 收到信号 {signum}, 正在安全停止...")
        self._running = False

    def run(self):
        """执行完整任务"""
        print("=" * 60)
        print("  G2 瓶子取放任务")
        print("=" * 60)

        if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
            print("[任务] ❌ GDK初始化失败")
            return False

        print("[任务] ✅ GDK初始化成功")

        navi = None
        camera = None

        try:
            # ---- 初始化子系统 ----
            print("\n[任务] 正在初始化子系统...")

            robot = agibot_gdk.Robot()
            time.sleep(2)
            print("[任务] ✅ Robot 就绪")

            tf = agibot_gdk.TF()
            time.sleep(0.5)
            print("[任务] ✅ TF 就绪")

            camera = agibot_gdk.Camera()
            time.sleep(2)
            print("[任务] ✅ Camera 就绪")

            navi = NaviController(self.cfg)
            arm_ctrl = ArmController(robot, tf, self.cfg)
            detector = BottleDetector(camera, self.cfg)

            # ---- 检查伺服模式 ----
            if not arm_ctrl.ensure_servo_mode():
                print("[任务] ⚠️ 伺服模式未就绪, 抓取功能可能不可用")
                print("[任务] 继续执行导航部分...")

            # ---- 检查定位状态 ----
            odom = navi.get_current_position()
            if odom:
                print(f"[任务] 当前位置: ({odom['position']['x']:.2f}, "
                      f"{odom['position']['y']:.2f}), "
                      f"定位状态={odom['loc_state']}, "
                      f"置信度={odom['loc_confidence']:.2f}")

            print(f"\n[任务] 配置:")
            print(f"  A点: ({self.cfg.point_a_position[0]:.2f}, "
                  f"{self.cfg.point_a_position[1]:.2f})")
            print(f"  B点: ({self.cfg.point_b_position[0]:.2f}, "
                  f"{self.cfg.point_b_position[1]:.2f})")
            print(f"  手臂: {self.cfg.arm}")
            print(f"  检测方式: {self.cfg.detect_method}")

            # ---- 状态机循环 ----
            self.state = TaskState.NAVI_TO_A
            while self._running and self.state not in (TaskState.DONE, TaskState.ERROR):
                self._tick(navi, arm_ctrl, detector, robot, tf)
                time.sleep(0.01)

            # ---- 结果 ----
            if self.state == TaskState.DONE:
                print("\n" + "=" * 60)
                print("  ✅ 任务完成！")
                print("=" * 60)
                return True
            else:
                print(f"\n[任务] ❌ 任务失败: {self._error_msg}")
                return False

        except Exception as e:
            print(f"\n[任务] ❌ 异常: {e}")
            import traceback
            traceback.print_exc()
            return False

        finally:
            # ---- 清理 ----
            print("\n[任务] 正在清理...")
            if camera:
                try:
                    camera.close_camera()
                except Exception:
                    pass
            if navi:
                try:
                    navi.cancel_current_task()
                except Exception:
                    pass
            agibot_gdk.gdk_release()
            print("[任务] GDK资源已释放")

    def _tick(self, navi: NaviController, arm: ArmController,
              detector: BottleDetector, robot: agibot_gdk.Robot,
              tf: agibot_gdk.TF):
        """状态机单步"""

        # ──────── 1. 导航到A点 ────────
        if self.state == TaskState.NAVI_TO_A:
            print("\n" + "─" * 40)
            print("[阶段1] 导航到A点 (取瓶子位置)")
            print("─" * 40)
            try:
                navi.navigate_to(
                    self.cfg.point_a_position,
                    self.cfg.point_a_orientation,
                    label="A点"
                )
                self.state = TaskState.WAIT_NAVI_A
            except Exception as e:
                self._error_msg = f"导航到A失败: {e}"
                self.state = TaskState.ERROR

        # ──────── 等待到达A ────────
        elif self.state == TaskState.WAIT_NAVI_A:
            if navi.wait_navigation_complete():
                self.state = TaskState.DETECT_BOTTLE
            else:
                self._error_msg = "导航到A点失败或超时"
                self.state = TaskState.ERROR

        # ──────── 2. 视觉检测瓶子 ────────
        elif self.state == TaskState.DETECT_BOTTLE:
            print("\n" + "─" * 40)
            print("[阶段2] 视觉检测瓶子")
            print("─" * 40)

            # 低头看桌面
            try:
                robot.move_head_joint(
                    self.cfg.head_look_down,
                    self.cfg.head_velocity
                )
                time.sleep(1.0)
                print("[视觉] 头部已低头")
            except Exception as e:
                print(f"[视觉] ⚠️ 头部控制失败: {e}")

            # 多次尝试检测
            result = None
            for attempt in range(self.cfg.detect_max_retries):
                print(f"[视觉] 检测尝试 {attempt + 1}/{self.cfg.detect_max_retries}")
                result = detector.detect()
                if result is not None:
                    break
                time.sleep(0.5)

            if result is None:
                self._error_msg = "视觉检测失败: 未找到瓶子"
                self.state = TaskState.ERROR
                return

            # 相机坐标 → base_link 坐标
            cam_3d = result["camera_3d"]
            base_pos = camera_to_base_link(cam_3d, tf)
            if base_pos is None:
                self._error_msg = "坐标变换失败"
                self.state = TaskState.ERROR
                return

            self._bottle_base_pos = base_pos
            print(f"[视觉] ✅ 瓶子 base_link 坐标: "
                  f"({base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f})")

            # 抬头
            try:
                robot.move_head_joint(
                    self.cfg.head_look_forward,
                    self.cfg.head_velocity
                )
            except Exception:
                pass

            self.state = TaskState.APPROACH_BOTTLE

        # ──────── 3. 手臂接近瓶子 ────────
        elif self.state == TaskState.APPROACH_BOTTLE:
            print("\n" + "─" * 40)
            print("[阶段3] 手臂接近瓶子")
            print("─" * 40)

            if not arm.ensure_servo_mode():
                self._error_msg = "伺服模式不可用"
                self.state = TaskState.ERROR
                return

            bx, by, bz = self._bottle_base_pos

            # 保持当前姿态, 只移动位置
            _, cur_quat = arm.get_current_pose()
            self._bottle_quat = cur_quat

            # 张开夹爪
            arm.open_gripper()
            time.sleep(0.3)

            # 移动到瓶子上方
            above = [bx, by, bz + self.cfg.approach_height]
            print(f"[手臂] → 瓶子上方: "
                  f"({above[0]:.3f}, {above[1]:.3f}, {above[2]:.3f})")
            arm.servo_move(above, cur_quat)

            # 下探到抓取高度
            grasp = [bx, by, bz + self.cfg.grasp_down]
            print(f"[手臂] → 下探: "
                  f"({grasp[0]:.3f}, {grasp[1]:.3f}, {grasp[2]:.3f})")
            arm.servo_move(grasp, cur_quat, duration_s=1.2)

            self.state = TaskState.GRASP_BOTTLE

        # ──────── 4. 闭合夹爪 ────────
        elif self.state == TaskState.GRASP_BOTTLE:
            print("\n[阶段4] 闭合夹爪")
            arm.close_gripper()
            time.sleep(0.5)
            print("[夹爪] ✅ 夹爪已闭合")
            self.state = TaskState.LIFT_BOTTLE

        # ──────── 5. 抬起瓶子 ────────
        elif self.state == TaskState.LIFT_BOTTLE:
            print("\n[阶段5] 抬起瓶子")
            bx, by, bz = self._bottle_base_pos
            lift = [bx, by, bz + self.cfg.lift_height]
            print(f"[手臂] → 抬起: "
                  f"({lift[0]:.3f}, {lift[1]:.3f}, {lift[2]:.3f})")
            arm.servo_move(lift, self._bottle_quat)
            print("[手臂] ✅ 瓶子已抬起")
            self.state = TaskState.NAVI_TO_B

        # ──────── 6. 导航到B点 ────────
        elif self.state == TaskState.NAVI_TO_B:
            print("\n" + "─" * 40)
            print("[阶段6] 导航到B点 (放瓶子位置)")
            print("─" * 40)
            try:
                navi.navigate_to(
                    self.cfg.point_b_position,
                    self.cfg.point_b_orientation,
                    label="B点"
                )
                self.state = TaskState.WAIT_NAVI_B
            except Exception as e:
                self._error_msg = f"导航到B失败: {e}"
                self.state = TaskState.ERROR

        # ──────── 等待到达B ────────
        elif self.state == TaskState.WAIT_NAVI_B:
            if navi.wait_navigation_complete():
                self.state = TaskState.PLACE_BOTTLE
            else:
                self._error_msg = "导航到B点失败或超时"
                self.state = TaskState.ERROR

        # ──────── 7. 放置瓶子 ────────
        elif self.state == TaskState.PLACE_BOTTLE:
            print("\n" + "─" * 40)
            print("[阶段7] 放置瓶子")
            print("─" * 40)

            if not arm.ensure_servo_mode():
                self._error_msg = "伺服模式不可用"
                self.state = TaskState.ERROR
                return

            # 放置位置
            if self.cfg.place_position_base:
                place_pos = self.cfg.place_position_base
            else:
                cur_pos, _ = arm.get_current_pose()
                place_pos = [cur_pos[0], cur_pos[1],
                             cur_pos[2] - self.cfg.lift_height + self.cfg.place_height]

            print(f"[手臂] → 放置位置: "
                  f"({place_pos[0]:.3f}, {place_pos[1]:.3f}, {place_pos[2]:.3f})")
            arm.servo_move(place_pos, self._bottle_quat, duration_s=1.5)

            self.state = TaskState.RELEASE_BOTTLE

        # ──────── 8. 松开夹爪 ────────
        elif self.state == TaskState.RELEASE_BOTTLE:
            print("\n[阶段8] 松开夹爪")
            arm.open_gripper()
            time.sleep(0.3)
            print("[夹爪] ✅ 瓶子已放下")
            self.state = TaskState.RETRACT_ARM

        # ──────── 9. 手臂回收 ────────
        elif self.state == TaskState.RETRACT_ARM:
            print("\n[阶段9] 手臂回收")
            cur_pos, cur_quat = arm.get_current_pose()
            retract = [cur_pos[0], cur_pos[1], cur_pos[2] + 0.10]
            arm.servo_move(retract, cur_quat, duration_s=1.0)
            print("[手臂] ✅ 手臂已回收")
            self.state = TaskState.DONE


# ═══════════════════════════════════════════════════════════════════
#  命令行入口
# ═══════════════════════════════════════════════════════════════════

def parse_args():
    ap = argparse.ArgumentParser(
        description="G2机器人瓶子取放任务",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 基本用法 (指定AB点坐标)
  python3 g2_fetch_bottle.py --ax 2.0 --ay 1.0 --bx 5.0 --by 3.0

  # 指定手臂和完整朝向
  python3 g2_fetch_bottle.py --ax 2.0 --ay 1.0 --bx 5.0 --by 3.0 \\
      --arm right --ao 0 0 0.7 0.7

  # 使用配置文件 (支持YOLO检测等高级参数)
  python3 g2_fetch_bottle.py --config task_config.json

  # 生成默认配置文件
  python3 g2_fetch_bottle.py --gen-config
        """
    )

    ap.add_argument("--config", type=str, help="JSON配置文件路径")
    ap.add_argument("--gen-config", action="store_true",
                    help="生成默认配置文件并退出")

    # A点
    ap.add_argument("--ax", type=float, default=None, help="A点 X (map坐标)")
    ap.add_argument("--ay", type=float, default=0.0, help="A点 Y")
    ap.add_argument("--az", type=float, default=0.0, help="A点 Z")
    ap.add_argument("--ao", type=float, nargs=4, default=None,
                    help="A点朝向四元数 x y z w")

    # B点
    ap.add_argument("--bx", type=float, default=None, help="B点 X (map坐标)")
    ap.add_argument("--by", type=float, default=0.0, help="B点 Y")
    ap.add_argument("--bz", type=float, default=0.0, help="B点 Z")
    ap.add_argument("--bo", type=float, nargs=4, default=None,
                    help="B点朝向四元数 x y z w")

    # 手臂
    ap.add_argument("--arm", choices=["left", "right"], default=None,
                    help="使用的手臂")

    return ap.parse_args()


def generate_default_config():
    """生成默认配置文件"""
    cfg = TaskConfig()
    data = {}
    for k in sorted(dir(cfg)):
        if k.startswith("_") or callable(getattr(cfg, k)):
            continue
        data[k] = getattr(cfg, k)

    path = "task_config.json"
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"✅ 默认配置已生成: {path}")
    print(f"   请编辑 A/B 点坐标和检测参数后运行:")
    print(f"   python3 g2_fetch_bottle.py --config {path}")


def main():
    args = parse_args()

    # 生成配置
    if args.gen_config:
        generate_default_config()
        return 0

    # 加载配置
    if args.config:
        cfg = TaskConfig.from_json(args.config)
        # 命令行参数覆盖
        if args.ax is not None:
            cfg = TaskConfig.from_args(args)
    else:
        cfg = TaskConfig.from_args(args)

    # 执行任务
    task = FetchBottleTask(cfg)
    success = task.run()
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
