#!/usr/bin/env python3
"""
智元 G2 机器人 - 关节控制 / 动作录制 / 动作编排系统
基于 agibot_gdk (GDK) Python SDK

功能：
  1. 安全的关节控制（含限位保护、急停检测、生命周期管理）
  2. 动作录制（示教模式，定时采样关节状态并保存为 JSON）
  3. 动作编排（多段动作序列串联、过渡插值、循环播放）

键盘操作：
  [a/d]  切换关节
  [w/s]  调节当前关节位置 (+/- step)
  [1/2]  减小/增大步长
  [3/4]  减小/增大速度
  [r]    开始/停止录制
  [p]    播放当前动作文件（逐帧）
  [o]    播放当前动作文件（自动完整播放）
  [m]    切换动作文件
  [c]    播放编排序列（所有动作文件依次执行）
  [h]    显示帮助
  [i]    显示当前状态信息
  [e]    急停 / 全关节回零
  [q]    退出
"""

import os
import sys
import json
import glob
import time
import math
import datetime
from typing import List, Dict, Any, Optional, Tuple

try:
    import agibot_gdk
except ImportError:
    print("警告: 未找到 agibot_gdk 模块，进入模拟模式（仅供开发调试）")
    agibot_gdk = None

# ──────────────────────────────────────────────
# 平台适配的按键读取
# ──────────────────────────────────────────────
try:
    import msvcrt
    def getch() -> str:
        return msvcrt.getch().decode(sys.stdout.encoding)
except ImportError:
    import tty, termios
    def getch() -> str:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch


# ══════════════════════════════════════════════
# 第一部分：关节限位表与安全检查
# ══════════════════════════════════════════════

# 关节名称（完整22个关节，按文档顺序）
JOINT_NAMES = [
    # 腰部 (5)
    "idx01_body_joint1",
    "idx02_body_joint2",
    "idx03_body_joint3",
    "idx04_body_joint4",
    "idx05_body_joint5",
    # 头部 (3)
    "idx11_head_joint1",
    "idx12_head_joint2",
    "idx13_head_joint3",
    # 左臂 (7)
    "idx21_arm_l_joint1",
    "idx22_arm_l_joint2",
    "idx23_arm_l_joint3",
    "idx24_arm_l_joint4",
    "idx25_arm_l_joint5",
    "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
    # 右臂 (7)
    "idx61_arm_r_joint1",
    "idx62_arm_r_joint2",
    "idx63_arm_r_joint3",
    "idx64_arm_r_joint4",
    "idx65_arm_r_joint5",
    "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]

# 关节限位（弧度），来自 GDK 文档
JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
    "idx01_body_joint1": (-1.082104,  0.000174),
    "idx02_body_joint2": (-0.000174,  2.652900),
    "idx03_body_joint3": (-1.919862,  1.570970),
    "idx04_body_joint4": (-0.436332,  0.436332),
    "idx05_body_joint5": (-3.045599,  3.045599),
    "idx11_head_joint1": (-1.570970,  1.570970),
    "idx12_head_joint2": (-0.349240,  0.349240),
    "idx13_head_joint3": (-0.534773,  0.534773),
    "idx21_arm_l_joint1": (-3.071796,  3.071796),
    "idx22_arm_l_joint2": (-2.059505,  2.059505),
    "idx23_arm_l_joint3": (-3.071796,  3.071796),
    "idx24_arm_l_joint4": (-2.495838,  1.012308),
    "idx25_arm_l_joint5": (-3.071796,  3.071796),
    "idx26_arm_l_joint6": (-1.012308,  1.012308),
    "idx27_arm_l_joint7": (-1.535907,  1.535907),
    "idx61_arm_r_joint1": (-3.071796,  3.071796),
    "idx62_arm_r_joint2": (-2.059505,  2.059505),
    "idx63_arm_r_joint3": (-3.071796,  3.071796),
    "idx64_arm_r_joint4": (-2.495838,  1.012308),
    "idx65_arm_r_joint5": (-3.071796,  3.071796),
    "idx66_arm_r_joint6": (-1.012308,  1.012308),
    "idx67_arm_r_joint7": (-1.535907,  1.535907),
}

# 关节分组（用于 move_xxx_joint 高级接口）
WAIST_JOINTS = [f"idx0{i}_body_joint{i}" for i in range(1, 6)]
HEAD_JOINTS  = [f"idx1{i}_head_joint{i}" for i in range(1, 4)]
LEFT_ARM_JOINTS  = [f"idx2{i}_arm_l_joint{i}" for i in range(1, 8)]
RIGHT_ARM_JOINTS = [f"idx6{i}_arm_r_joint{i}" for i in range(1, 8)]


def clamp_position(joint_name: str, target: float) -> float:
    """将目标位置限制在关节安全范围内"""
    if joint_name in JOINT_LIMITS:
        lo, hi = JOINT_LIMITS[joint_name]
        clamped = max(lo, min(hi, target))
        if clamped != target:
            print(f"  ⚠️  限位保护: {joint_name} 目标 {target:.4f} 被限制到 {clamped:.4f} "
                  f"(范围: [{lo:.4f}, {hi:.4f}])")
        return clamped
    return target


def rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


# ══════════════════════════════════════════════
# 第二部分：动作录制器
# ══════════════════════════════════════════════

class MotionRecorder:
    """动作录制器：定时采样关节状态并保存为 JSON"""

    def __init__(self, save_dir: str = "saved_commands"):
        self.save_dir = save_dir
        self.is_recording = False
        self.recorded_frames: List[Dict[str, Any]] = []
        self.record_interval = 0.1   # 录制采样间隔（秒），即 10Hz
        self.last_record_time = 0.0
        os.makedirs(save_dir, exist_ok=True)

    def start_recording(self):
        """开始录制"""
        self.is_recording = True
        self.recorded_frames = []
        self.last_record_time = time.time()
        print("������ 开始录制动作...")

    def stop_recording(self) -> Optional[str]:
        """停止录制并保存文件，返回文件路径"""
        if not self.is_recording:
            return None
        self.is_recording = False

        if len(self.recorded_frames) == 0:
            print("⚠️  未录制到任何帧，取消保存")
            return None

        # 生成文件名
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"motion_{timestamp}.json"
        filepath = os.path.join(self.save_dir, filename)

        # 保存为与 mc_example.py 兼容的格式
        data = {
            "metadata": {
                "created": timestamp,
                "frame_count": len(self.recorded_frames),
                "interval_s": self.record_interval,
                "description": "G2 motion recording",
            },
            "recorded_commands": self.recorded_frames,
        }

        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

        print(f"⬜ 录制停止，共 {len(self.recorded_frames)} 帧，已保存到: {filepath}")
        return filepath

    def try_record_frame(self, robot) -> bool:
        """如果正在录制且达到采样间隔，采样一帧，返回是否采样成功"""
        if not self.is_recording:
            return False

        now = time.time()
        if now - self.last_record_time < self.record_interval:
            return False

        self.last_record_time = now

        try:
            joint_states = robot.get_joint_states()
            frame = {
                "joint_names":     [s["name"] for s in joint_states["states"]],
                "joint_positions":  [s["motor_position"] for s in joint_states["states"]],
                "timestamp_ns":    joint_states.get("timestamp", 0),
            }
            self.recorded_frames.append(frame)

            frame_num = len(self.recorded_frames)
            if frame_num % 10 == 0:
                print(f"  ������ 已录制 {frame_num} 帧...")
            return True
        except Exception as e:
            print(f"  ⚠️  录制采样失败: {e}")
            return False


# ══════════════════════════════════════════════
# 第三部分：动作编排系统
# ══════════════════════════════════════════════

class MotionChoreographer:
    """动作编排系统：管理多段动作文件，支持序列播放和插值过渡"""

    def __init__(self, save_dir: str = "saved_commands"):
        self.save_dir = save_dir
        self.files: List[str] = []
        self.file_names: List[str] = []
        self.commands: List[List[Dict[str, Any]]] = []
        self.current_file_index = 0
        self.current_play_index = 0
        self.reload_files()

    def reload_files(self):
        """重新加载 saved_commands 目录下所有 JSON 文件"""
        files = sorted(glob.glob(os.path.join(self.save_dir, "*.json")))
        self.files = files
        self.file_names = [os.path.basename(p) for p in files]
        self.commands = []
        for fp in files:
            try:
                with open(fp, encoding="utf-8") as f:
                    data = json.load(f)
                self.commands.append(data.get("recorded_commands", []))
            except Exception as e:
                print(f"  ⚠️  加载 {fp} 失败: {e}")
                self.commands.append([])

        self.current_file_index = 0
        self.current_play_index = 0
        count = len(self.files)
        print(f"������ 已加载 {count} 个动作文件" + (f": {self.file_names}" if count > 0 else ""))

    def switch_file(self):
        """切换到下一个动作文件"""
        if not self.commands:
            print("⚠️  没有可用的动作文件")
            return
        self.current_file_index = (self.current_file_index + 1) % len(self.commands)
        self.current_play_index = 0
        print(f"������ 切换到: {self.file_names[self.current_file_index]} "
              f"({len(self.commands[self.current_file_index])} 帧)")

    def get_current_frame(self) -> Optional[Dict[str, Any]]:
        """获取当前文件的当前帧，并推进索引"""
        if not self.commands or not self.commands[self.current_file_index]:
            print("⚠️  当前文件无可用帧")
            return None
        cmds = self.commands[self.current_file_index]
        frame = cmds[self.current_play_index]
        print(f"▶️  播放 [{self.file_names[self.current_file_index]}] "
              f"帧 {self.current_play_index + 1}/{len(cmds)}")
        self.current_play_index = (self.current_play_index + 1) % len(cmds)
        return frame

    @staticmethod
    def interpolate_frames(frame_a: Dict, frame_b: Dict, t: float) -> Dict:
        """
        线性插值两帧之间的关节位置
        t: 0.0 = frame_a, 1.0 = frame_b
        """
        t = max(0.0, min(1.0, t))
        positions_a = frame_a["joint_positions"]
        positions_b = frame_b["joint_positions"]
        interpolated = [
            a + t * (b - a) for a, b in zip(positions_a, positions_b)
        ]
        return {
            "joint_names": frame_a["joint_names"],
            "joint_positions": interpolated,
        }

    def play_sequence(self, robot, publish_fn, speed: float = 0.1,
                      transition_steps: int = 20, frame_delay: float = 0.1):
        """
        顺序播放所有动作文件（编排模式）

        参数:
            robot:             Robot 对象（用于读取当前关节状态）
            publish_fn:        发布控制命令的回调函数
            speed:             关节运动速度
            transition_steps:  文件之间过渡插值的步数
            frame_delay:       每帧之间的延迟（秒）
        """
        if not self.commands:
            print("⚠️  没有可用的动作文件，无法编排")
            return

        total_files = len(self.commands)
        print(f"\n������ 开始编排播放，共 {total_files} 个动作文件")
        print("   按 Ctrl+C 中断\n")

        try:
            last_frame = None

            for file_idx in range(total_files):
                cmds = self.commands[file_idx]
                if not cmds:
                    continue

                name = self.file_names[file_idx]
                print(f"  ▶️  播放 [{name}] ({len(cmds)} 帧)")

                # ---- 过渡插值：从上一个动作的末尾平滑过渡到当前动作的开头 ----
                if last_frame is not None and transition_steps > 0:
                    first_frame = cmds[0]
                    print(f"     ������ 过渡插值 ({transition_steps} 步)...")
                    for step in range(transition_steps):
                        t = (step + 1) / transition_steps
                        interp = self.interpolate_frames(last_frame, first_frame, t)

                        # 安全检查
                        safe_positions = [
                            clamp_position(n, p)
                            for n, p in zip(interp["joint_names"], interp["joint_positions"])
                        ]
                        publish_fn(interp["joint_names"], safe_positions, speed)
                        time.sleep(frame_delay)

                # ---- 播放当前文件所有帧 ----
                for frame_idx, frame in enumerate(cmds):
                    safe_positions = [
                        clamp_position(n, p)
                        for n, p in zip(frame["joint_names"], frame["joint_positions"])
                    ]
                    publish_fn(frame["joint_names"], safe_positions, speed)

                    if (frame_idx + 1) % 20 == 0:
                        print(f"     帧 {frame_idx + 1}/{len(cmds)}")
                    time.sleep(frame_delay)

                last_frame = cmds[-1]
                print(f"  ✅ [{name}] 播放完成")

            print("\n������ 编排播放全部完成！\n")

        except KeyboardInterrupt:
            print("\n⏹  编排播放被中断")


# ══════════════════════════════════════════════
# 主控制器（整合三个部分）
# ══════════════════════════════════════════════

class G2MotionController:
    """G2 机器人完整运动控制系统"""

    def __init__(self):
        # 控制参数
        self.step = 0.05          # 单步步长（弧度）
        self.speed = 0.1          # 关节运动速度（弧度/秒）
        self.joint_index = 0      # 当前选中的关节索引
        self.robot = None

        # 子系统
        self.recorder = MotionRecorder()
        self.choreographer = MotionChoreographer()

    # ---- 初始化与释放 ----

    def init_gdk(self) -> bool:
        """初始化 GDK 系统和 Robot 对象"""
        if agibot_gdk is None:
            print("⚠️  模拟模式，跳过 GDK 初始化")
            return False

        print("正在初始化 GDK 系统...")
        result = agibot_gdk.gdk_init()
        if result != agibot_gdk.GDKRes.kSuccess:
            print(f"❌ GDK 初始化失败，错误码: {result}")
            return False
        print("✅ GDK 初始化成功")

        self.robot = agibot_gdk.Robot()
        time.sleep(2)  # 等待 DDS 连接建立
        print("✅ Robot 对象已创建")

        # 启动后检查全身状态
        self._check_whole_body_status()
        return True

    def release_gdk(self):
        """释放 GDK 系统资源"""
        if agibot_gdk is None:
            return

        print("正在释放 GDK 系统资源...")
        result = agibot_gdk.gdk_release()
        if result == agibot_gdk.GDKRes.kSuccess:
            print("✅ GDK 释放成功")
        else:
            print(f"⚠️  GDK 释放失败，错误码: {result}")

    # ---- 安全检查 ----

    def _check_whole_body_status(self):
        """检查全身状态，输出错误和急停信息"""
        if self.robot is None:
            return

        try:
            status = self.robot.get_whole_body_status()

            errors = []
            if status.get("left_arm_error", 0) != 0:
                errors.append(f"左臂错误码: {status['left_arm_error']}")
            if status.get("right_arm_error", 0) != 0:
                errors.append(f"右臂错误码: {status['right_arm_error']}")
            if status.get("waist_error", 0) != 0:
                errors.append(f"腰部错误码: {status['waist_error']}")
            if status.get("neck_error", 0) != 0:
                errors.append(f"头部错误码: {status['neck_error']}")
            if status.get("chassis_error", 0) != 0:
                errors.append(f"底盘错误码: {status['chassis_error']}")

            if status.get("left_arm_estop", False):
                errors.append("左臂急停已触发!")
            if status.get("right_arm_estop", False):
                errors.append("右臂急停已触发!")

            if errors:
                print("⚠️  全身状态检查发现问题:")
                for err in errors:
                    print(f"   ❌ {err}")
            else:
                print("✅ 全身状态正常")

        except Exception as e:
            print(f"⚠️  全身状态检查失败: {e}")

    def _is_safe_to_move(self) -> bool:
        """检查是否可以安全运动（急停未触发）"""
        if self.robot is None:
            return False
        try:
            status = self.robot.get_whole_body_status()
            if status.get("left_arm_estop", False) or status.get("right_arm_estop", False):
                print("������ 急停已触发，无法运动！请先解除急停。")
                return False
            return True
        except Exception:
            return True  # 检查失败时不阻塞

    # ---- 控制命令发布 ----

    def publish_control(self, joint_names: List[str],
                        joint_positions: List[float],
                        speed: Optional[float] = None):
        """发布关节控制请求（带安全检查）"""
        if self.robot is None:
            print(f"  [模拟] 控制 {joint_names} -> {[f'{p:.4f}' for p in joint_positions]}")
            return

        req = agibot_gdk.JointControlReq()
        req.life_time = 1.0
        req.joint_names = joint_names
        req.joint_positions = joint_positions
        req.joint_velocities = [speed or self.speed]
        self.robot.joint_control_request(req)

    def adjust_joint(self, delta: float):
        """调节当前关节位置"""
        if not self._is_safe_to_move():
            return

        joint_name = JOINT_NAMES[self.joint_index]

        # 读取当前位置
        if self.robot is not None:
            try:
                states = self.robot.get_joint_states()
                current_pos = states["states"][self.joint_index]["motor_position"]
            except Exception as e:
                print(f"  ⚠️  读取关节状态失败: {e}")
                return
        else:
            current_pos = 0.0

        target = clamp_position(joint_name, current_pos + delta)
        print(f"  ������ {joint_name}: {current_pos:.4f} -> {target:.4f} "
              f"({rad_to_deg(target):.1f}°)")
        self.publish_control([joint_name], [target])

    def move_all_to_zero(self):
        """所有关节回零位"""
        if not self._is_safe_to_move():
            return
        print("������ 全关节回零...")
        self.publish_control(JOINT_NAMES, [0.0] * len(JOINT_NAMES), speed=0.05)
        print("  已发送回零指令")

    # ---- 状态显示 ----

    def show_status(self):
        """显示当前系统状态"""
        joint_name = JOINT_NAMES[self.joint_index]
        lo, hi = JOINT_LIMITS.get(joint_name, (-999, 999))

        print("\n" + "=" * 55)
        print(f"  当前关节: [{self.joint_index}] {joint_name}")
        print(f"  限位范围: [{lo:.4f}, {hi:.4f}] rad "
              f"([{rad_to_deg(lo):.1f}°, {rad_to_deg(hi):.1f}°])")
        print(f"  步长: {self.step:.4f} rad ({rad_to_deg(self.step):.2f}°)")
        print(f"  速度: {self.speed:.3f} rad/s")
        print(f"  录制状态: {'������ 录制中' if self.recorder.is_recording else '⬜ 未录制'}")
        if self.recorder.is_recording:
            print(f"  已录制帧数: {len(self.recorder.recorded_frames)}")
        print(f"  动作文件数: {len(self.choreographer.files)}")
        if self.choreographer.files:
            print(f"  当前文件: {self.choreographer.file_names[self.choreographer.current_file_index]}")

        # 读取实时关节位置
        if self.robot is not None:
            try:
                states = self.robot.get_joint_states()
                pos = states["states"][self.joint_index]["motor_position"]
                print(f"  当前位置: {pos:.4f} rad ({rad_to_deg(pos):.1f}°)")
            except Exception:
                pass

        print("=" * 55 + "\n")

    # ---- 播放控制 ----

    def play_full_file(self):
        """自动播放当前动作文件的所有帧"""
        if not self._is_safe_to_move():
            return
        if not self.choreographer.commands or not self.choreographer.commands[self.choreographer.current_file_index]:
            print("⚠️  当前文件无可用帧")
            return

        cmds = self.choreographer.commands[self.choreographer.current_file_index]
        name = self.choreographer.file_names[self.choreographer.current_file_index]
        total = len(cmds)
        print(f"\n▶️  自动播放 [{name}] 共 {total} 帧，按 Ctrl+C 中断")

        try:
            for i, frame in enumerate(cmds):
                safe_positions = [
                    clamp_position(n, p)
                    for n, p in zip(frame["joint_names"], frame["joint_positions"])
                ]
                self.publish_control(frame["joint_names"], safe_positions)

                if (i + 1) % 20 == 0 or (i + 1) == total:
                    print(f"     帧 {i + 1}/{total}")
                time.sleep(0.1)

            print(f"✅ [{name}] 播放完成\n")
            self.choreographer.current_play_index = 0
        except KeyboardInterrupt:
            print(f"\n⏹  播放中断，停在第 {i + 1}/{total} 帧\n")

    def play_single_frame(self):
        """播放当前动作文件的下一帧"""
        frame = self.choreographer.get_current_frame()
        if frame is None:
            return
        if not self._is_safe_to_move():
            return

        safe_positions = [
            clamp_position(n, p)
            for n, p in zip(frame["joint_names"], frame["joint_positions"])
        ]
        self.publish_control(frame["joint_names"], safe_positions)

    def play_choreography(self):
        """播放完整编排序列"""
        if not self._is_safe_to_move():
            return
        self.choreographer.reload_files()  # 重新加载以包含新录制的文件
        self.choreographer.play_sequence(
            robot=self.robot,
            publish_fn=self.publish_control,
            speed=self.speed,
            transition_steps=20,
            frame_delay=0.1,
        )

    # ---- 主循环 ----

    @staticmethod
    def print_help():
        print("""
╔══════════════════════════════════════════════╗
║       G2 机器人运动控制系统 - 帮助           ║
╠══════════════════════════════════════════════╣
║  关节控制:                                    ║
║    [a/d]  切换关节 (上一个/下一个)            ║
║    [w/s]  调节位置 (+step / -step)           ║
║    [1/2]  减小/增大步长                       ║
║    [3/4]  减小/增大速度                       ║
║                                              ║
║  录制:                                        ║
║    [r]    开始/停止录制                       ║
║                                              ║
║  播放:                                        ║
║    [p]    逐帧播放当前动作文件               ║
║    [o]    自动播放完整动作文件               ║
║    [m]    切换动作文件                        ║
║    [c]    编排播放 (所有文件顺序执行)         ║
║                                              ║
║  系统:                                        ║
║    [i]    显示状态信息                        ║
║    [e]    全关节回零                          ║
║    [h]    显示此帮助                          ║
║    [q]    退出                                ║
╚══════════════════════════════════════════════╝
""")

    def run(self):
        """主键盘控制循环"""
        self.print_help()
        self.show_status()

        while True:
            # 如果正在录制，尝试采样
            if self.recorder.is_recording and self.robot is not None:
                self.recorder.try_record_frame(self.robot)

            key = getch()
            k = key.lower()

            if k == "q":
                # 退出前停止录制
                if self.recorder.is_recording:
                    self.recorder.stop_recording()
                break

            elif k == "a":
                self.joint_index = (self.joint_index - 1) % len(JOINT_NAMES)
                jn = JOINT_NAMES[self.joint_index]
                lo, hi = JOINT_LIMITS.get(jn, (-999, 999))
                print(f"  ⬅️  关节: [{self.joint_index}] {jn}  "
                      f"范围: [{rad_to_deg(lo):.1f}°, {rad_to_deg(hi):.1f}°]")

            elif k == "d":
                self.joint_index = (self.joint_index + 1) % len(JOINT_NAMES)
                jn = JOINT_NAMES[self.joint_index]
                lo, hi = JOINT_LIMITS.get(jn, (-999, 999))
                print(f"  ➡️  关节: [{self.joint_index}] {jn}  "
                      f"范围: [{rad_to_deg(lo):.1f}°, {rad_to_deg(hi):.1f}°]")

            elif k == "w":
                self.adjust_joint(self.step)

            elif k == "s":
                self.adjust_joint(-self.step)

            elif k == "1":
                self.step = max(0.005, self.step / 2)
                print(f"  步长: {self.step:.4f} rad ({rad_to_deg(self.step):.2f}°)")

            elif k == "2":
                self.step = min(0.5, self.step * 2)
                print(f"  步长: {self.step:.4f} rad ({rad_to_deg(self.step):.2f}°)")

            elif k == "3":
                self.speed = max(0.01, self.speed / 2)
                print(f"  速度: {self.speed:.3f} rad/s")

            elif k == "4":
                self.speed = min(2.0, self.speed * 2)
                print(f"  速度: {self.speed:.3f} rad/s")

            elif k == "r":
                if self.recorder.is_recording:
                    filepath = self.recorder.stop_recording()
                    if filepath:
                        self.choreographer.reload_files()
                else:
                    self.recorder.start_recording()

            elif k == "p":
                self.play_single_frame()

            elif k == "o":
                self.play_full_file()

            elif k == "m":
                self.choreographer.switch_file()

            elif k == "c":
                self.play_choreography()

            elif k == "i":
                self.show_status()
                self._check_whole_body_status()

            elif k == "e":
                self.move_all_to_zero()

            elif k == "h":
                self.print_help()

            else:
                print("  按 [h] 查看帮助")


# ══════════════════════════════════════════════
# 入口
# ══════════════════════════════════════════════

def main():
    controller = G2MotionController()

    # 初始化 GDK
    gdk_ready = controller.init_gdk()

    try:
        controller.run()
    except Exception as e:
        print(f"\n❌ 程序运行出错: {e}")
    finally:
        # 确保录制停止
        if controller.recorder.is_recording:
            controller.recorder.stop_recording()
        # 释放 GDK 资源
        if gdk_ready:
            controller.release_gdk()
        print("\n������ 程序退出")


if __name__ == "__main__":
    main()
