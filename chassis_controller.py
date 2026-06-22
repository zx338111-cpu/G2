#!/usr/bin/env python3
"""
G2 底盘控制器
============
封装 agibot_gdk 的底盘运动接口，提供两种控制方式：

  1. 精确闭环（推荐）：relative_move + 死算里程计（DR/IMU）
     → 提交目标位移给 quark_navigation，由导航系统自动闭环执行
     → 方法：move_forward / move_backward / crab_left / crab_right / rotate / move_diagonal / move_sequence

  2. 速度开环（遥控）：move_chassis + Twist 速度指令
     → 持续向底盘发速度，不保证精度，适合实时遥控场景
     → 方法：velocity_control / stop

关键设计说明：
  - 每次精确移动前调用 _cancel_blocking_task()，取消可能占用底盘的旧任务
  - 每次任务完成（state 3/7/9）后等待 0.5s，让导航系统内部完成状态重置
    否则立即发第二条指令会报 "Task is not in IDLE" 错误

运行环境：
  - 机器人端运行，需先 source /home/agi/app/env.sh
  - 文件路径：/data/chassis_demo/chassis_controller.py
"""

import agibot_gdk
import time
import math


class ChassisController:

    def __init__(self):
        """初始化 GDK 和 Pnc 对象。"""
        if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
            raise RuntimeError("GDK 初始化失败")
        self._pnc = agibot_gdk.Pnc()
        time.sleep(0.5)  # 等待 DDS 连接建立

    def release(self):
        """释放 GDK 资源，程序结束时调用。"""
        agibot_gdk.gdk_release()

    # ------------------------------------------------------------------ #
    #  内部工具方法（不对外暴露）                                           #
    # ------------------------------------------------------------------ #

    def _cancel_blocking_task(self):
        """
        取消正在运行的任务，释放底盘控制权。

        背景：quark_navigation 同一时间只允许一个活跃任务。
        若有旧任务在跑（state=2），直接调用 relative_move 会被拒绝。
        state 含义：0=空闲 2=运行中 3=成功 7=已取消 9=完成
        """
        ts = self._pnc.get_task_state()
        if ts.state not in (0, 7, 9):
            self._pnc.cancel_task(ts.id)
            time.sleep(0.3)  # 等待取消指令生效

    def _wait_done(self, timeout=30.0):
        """
        轮询等待当前任务结束，返回最终 state。

        state=3/7/9 均表示任务已结束（成功或取消），机器人已到位。
        检测到结束后额外等待 0.5s，确保 quark_navigation 内部完全重置，
        避免下一条 relative_move 触发 "Task is not in IDLE" 错误。
        返回 -1 表示超时。
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(0.3)
            ts = self._pnc.get_task_state()
            if ts.state in (3, 7, 9):
                time.sleep(0.5)  # 导航系统内部状态重置缓冲
                return ts.state
        return -1

    def _make_req(self, x=0.0, y=0.0, yaw_deg=0.0):
        """
        构造 NaviReq 相对位移请求。

        坐标系（机器人本体系）：
          x 轴：前为正，后为负
          y 轴：左为正，右为负
          yaw ：逆时针为正，顺时针为负（转换为 Z 轴四元数）
        """
        req = agibot_gdk.NaviReq()
        req.target.position.x = x
        req.target.position.y = y
        req.target.position.z = 0.0
        # 绕 Z 轴旋转 yaw_deg 度的四元数：z=sin(θ/2), w=cos(θ/2)
        half = math.radians(yaw_deg) / 2.0
        req.target.orientation.x = 0.0
        req.target.orientation.y = 0.0
        req.target.orientation.z = math.sin(half)
        req.target.orientation.w = math.cos(half)
        return req

    # ------------------------------------------------------------------ #
    #  精确闭环控制（relative_move）                                        #
    # ------------------------------------------------------------------ #

    def move_forward(self, distance_m, timeout=30.0):
        """
        向前移动 distance_m 米。
        distance_m 为负值时向后移动。
        返回任务最终 state（9 或 7 = 成功到位，-1 = 超时）。
        """
        self._cancel_blocking_task()
        self._pnc.relative_move(self._make_req(x=distance_m))
        return self._wait_done(timeout)

    def move_relative(self, x_m=0.0, y_m=0.0, yaw_deg=0.0, timeout=30.0):
        """
        Execute one relative chassis pose command.

        x_m: forward/backward, positive forward
        y_m: left/right, positive left
        yaw_deg: positive counter-clockwise
        """
        self._cancel_blocking_task()
        self._pnc.relative_move(self._make_req(x=x_m, y=y_m, yaw_deg=yaw_deg))
        return self._wait_done(timeout)

    def move_backward(self, distance_m, timeout=30.0):
        """
        向后移动 distance_m 米（distance_m 填正值）。
        """
        return self.move_forward(-distance_m, timeout)

    def crab_left(self, distance_m, timeout=30.0):
        """
        向左横移（蟹行）distance_m 米。
        底盘四轮全向，可纯侧向平移，不转向。
        """
        self._cancel_blocking_task()
        self._pnc.relative_move(self._make_req(y=distance_m))
        return self._wait_done(timeout)

    def crab_right(self, distance_m, timeout=30.0):
        """
        向右横移（蟹行）distance_m 米。
        """
        return self.crab_left(-distance_m, timeout)

    def rotate(self, angle_deg, timeout=30.0):
        """
        原地旋转 angle_deg 度。
        正值逆时针，负值顺时针。
        精度依赖 IMU，长时间累积后可能有轻微漂移。
        """
        self._cancel_blocking_task()
        self._pnc.relative_move(self._make_req(yaw_deg=angle_deg))
        return self._wait_done(timeout)

    def move_diagonal(self, x_m, y_m, timeout=30.0):
        """
        斜向移动：同时指定前后和左右位移。
        x_m：前后（正前负后）
        y_m：左右（正左负右）
        """
        self._cancel_blocking_task()
        self._pnc.relative_move(self._make_req(x=x_m, y=y_m))
        return self._wait_done(timeout)

    def move_sequence(self, steps):
        """
        按顺序执行多个动作，每步完成后自动继续。

        steps 格式（list of dict）：
          {"action": "forward",    "distance": 1.0}
          {"action": "backward",   "distance": 0.5}
          {"action": "crab_left",  "distance": 1.0}
          {"action": "crab_right", "distance": 1.0}
          {"action": "rotate",     "angle": 90}       # 正逆时针
          {"action": "diagonal",   "x": 1.0, "y": 0.5}

        返回每步 state 列表。
        """
        results = []
        for i, step in enumerate(steps):
            action = step["action"]
            print(f"[{i+1}/{len(steps)}] {action} → {step}")
            if action == "forward":
                state = self.move_forward(step["distance"])
            elif action == "backward":
                state = self.move_backward(step["distance"])
            elif action == "crab_left":
                state = self.crab_left(step["distance"])
            elif action == "crab_right":
                state = self.crab_right(step["distance"])
            elif action == "rotate":
                state = self.rotate(step["angle"])
            elif action == "diagonal":
                state = self.move_diagonal(step["x"], step["y"])
            else:
                print(f"  未知动作: {action}")
                continue
            print(f"  完成 state={state}")
            results.append(state)
            time.sleep(0.3)
        return results

    # ------------------------------------------------------------------ #
    #  速度开环控制（move_chassis）                                         #
    # ------------------------------------------------------------------ #

    def velocity_control(self, vx=0.0, vy=0.0, vz=0.0,
                         duration=1.0, mode=1, hz=20):
        """
        速度开环控制，按 hz 频率持续发送 duration 秒后自动停止。

        参数：
          vx      前后速度 m/s，正值前进，负值后退
          vy      左右速度 m/s，正值向左，负值向右
          vz      旋转速度 rad/s，正值逆时针，负值顺时针
          duration 持续时间（秒）
          mode    0=阿克曼（转向行驶）1=蟹行（全向平移，默认）
          hz      发送频率，建议 20Hz

        注意：此方法无位置闭环，实际位移取决于地面摩擦和负载。
        """
        self._cancel_blocking_task()
        self._pnc.request_chassis_control(mode)
        time.sleep(0.3)

        twist = agibot_gdk.Twist()
        twist.linear  = agibot_gdk.Vector3()
        twist.angular = agibot_gdk.Vector3()
        twist.linear.x  = vx
        twist.linear.y  = vy
        twist.angular.z = vz

        interval = 1.0 / hz
        start = time.time()
        while time.time() - start < duration:
            self._pnc.move_chassis(twist)
            time.sleep(interval)

        # 发送零速确保停止
        twist.linear.x = twist.linear.y = twist.angular.z = 0.0
        self._pnc.move_chassis(twist)

    def stop(self):
        """
        立即停止所有运动。
        取消当前任务 + 发送零速指令。
        """
        self._cancel_blocking_task()
        twist = agibot_gdk.Twist()
        twist.linear  = agibot_gdk.Vector3()
        twist.angular = agibot_gdk.Vector3()
        self._pnc.move_chassis(twist)

    # ------------------------------------------------------------------ #
    #  状态查询                                                            #
    # ------------------------------------------------------------------ #

    def get_task_state(self):
        """
        查询当前任务状态。
        返回 PNCTaskState 对象，含 id / state / type 字段。
        state：0=空闲 2=运行中 3=成功 7=已取消 9=完成
        """
        return self._pnc.get_task_state()

    # ------------------------------------------------------------------ #
    #  上下文管理（with 语句支持）                                          #
    # ------------------------------------------------------------------ #

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.release()
