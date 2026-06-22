#!/usr/bin/env python3
"""
ChassisController 完整调用示例
==============================
演示每一个方法的调用方式和注意事项。

运行方式（在机器人上）：
  source /home/agi/app/env.sh
  cd /data/chassis_demo
  python3 chassis_demo.py
"""

import sys
sys.path.insert(0, "/data/chassis_demo")  # 让 Python 找到 chassis_controller.py

from chassis_controller import ChassisController


# ======================================================================
# 方式一：with 语句（推荐）
# 自动调用 release()，即使中途报错也能正确释放 GDK 资源
# ======================================================================
with ChassisController() as ctrl:

    # ------------------------------------------------------------------
    # move_forward(distance_m, timeout=30.0)
    # 向前移动，单位：米
    # 返回值：任务最终 state（9 或 7 = 成功，-1 = 超时）
    # ------------------------------------------------------------------
    print("=== move_forward ===")
    state = ctrl.move_forward(0.5)          # 前进 0.5 米
    print(f"state={state}")

    state = ctrl.move_forward(-0.5)         # 传负值 = 后退 0.5 米（等同 move_backward）
    print(f"state={state}")

    # ------------------------------------------------------------------
    # move_backward(distance_m, timeout=30.0)
    # 向后移动，单位：米（填正值）
    # ------------------------------------------------------------------
    print("=== move_backward ===")
    state = ctrl.move_backward(0.5)         # 后退 0.5 米
    print(f"state={state}")

    # ------------------------------------------------------------------
    # crab_left(distance_m, timeout=30.0)
    # 向左横移（蟹行），单位：米
    # ------------------------------------------------------------------
    print("=== crab_left ===")
    state = ctrl.crab_left(0.5)             # 向左平移 0.5 米
    print(f"state={state}")

    # ------------------------------------------------------------------
    # crab_right(distance_m, timeout=30.0)
    # 向右横移（蟹行），单位：米
    # ------------------------------------------------------------------
    print("=== crab_right ===")
    state = ctrl.crab_right(0.5)            # 向右平移 0.5 米
    print(f"state={state}")

    # ------------------------------------------------------------------
    # rotate(angle_deg, timeout=30.0)
    # 原地旋转，单位：度
    # 正值 = 逆时针，负值 = 顺时针
    # ------------------------------------------------------------------
    print("=== rotate ===")
    state = ctrl.rotate(90)                 # 逆时针旋转 90 度
    print(f"state={state}")

    state = ctrl.rotate(-90)                # 顺时针旋转 90 度（转回来）
    print(f"state={state}")

    # ------------------------------------------------------------------
    # move_diagonal(x_m, y_m, timeout=30.0)
    # 斜向移动：同时指定前后和左右位移
    # x_m：前后（正前负后）
    # y_m：左右（正左负右）
    # ------------------------------------------------------------------
    print("=== move_diagonal ===")
    state = ctrl.move_diagonal(0.5, 0.3)   # 同时前进 0.5m 且向左 0.3m
    print(f"state={state}")

    state = ctrl.move_diagonal(-0.5, -0.3) # 同时后退 0.5m 且向右 0.3m
    print(f"state={state}")

    # ------------------------------------------------------------------
    # move_sequence(steps)
    # 按顺序执行多个动作，自动等待每步完成后再执行下一步
    # action 可选值：forward / backward / crab_left / crab_right / rotate / diagonal
    # ------------------------------------------------------------------
    print("=== move_sequence ===")
    ctrl.move_sequence([
        {"action": "forward",    "distance": 0.5},   # 前进 0.5m
        {"action": "crab_right", "distance": 0.5},   # 右蟹行 0.5m
        {"action": "backward",   "distance": 0.5},   # 后退 0.5m
        {"action": "crab_left",  "distance": 0.5},   # 左蟹行 0.5m（回原点）
        {"action": "rotate",     "angle": 180},       # 原地转 180 度
        {"action": "rotate",     "angle": -180},      # 转回来
        {"action": "diagonal",   "x": 0.3, "y": 0.3},# 斜向左前方
        {"action": "diagonal",   "x": -0.3, "y": -0.3}, # 斜向右后方（回原点）
    ])

    # ------------------------------------------------------------------
    # velocity_control(vx, vy, vz, duration, mode, hz)
    # 速度开环控制，不保证精度，适合实时遥控场景
    # vx：前后 m/s，vy：左右 m/s，vz：旋转 rad/s
    # duration：持续秒数，mode：0=阿克曼 1=蟹行（默认）
    # ------------------------------------------------------------------
    print("=== velocity_control ===")
    ctrl.velocity_control(vx=0.2, duration=1.0)        # 以 0.2m/s 前进 1 秒
    ctrl.velocity_control(vx=-0.2, duration=1.0)       # 以 0.2m/s 后退 1 秒
    ctrl.velocity_control(vy=0.2, duration=1.0)        # 以 0.2m/s 向左蟹行 1 秒
    ctrl.velocity_control(vy=-0.2, duration=1.0)       # 以 0.2m/s 向右蟹行 1 秒
    ctrl.velocity_control(vz=0.3, duration=1.0)        # 以 0.3rad/s 逆时针旋转 1 秒
    ctrl.velocity_control(vz=-0.3, duration=1.0)       # 以 0.3rad/s 顺时针旋转 1 秒

    # ------------------------------------------------------------------
    # stop()
    # 立即停止：取消当前任务 + 发送零速指令
    # 紧急情况下调用
    # ------------------------------------------------------------------
    print("=== stop ===")
    ctrl.stop()

    # ------------------------------------------------------------------
    # get_task_state()
    # 查询当前任务状态，返回 PNCTaskState 对象
    # state：0=空闲 2=运行中 3=成功 7=已取消 9=完成
    # ------------------------------------------------------------------
    print("=== get_task_state ===")
    ts = ctrl.get_task_state()
    print(f"id={ts.id}  state={ts.state}  type={ts.type}")

    # ------------------------------------------------------------------
    # 超时处理示例
    # timeout 参数控制最长等待时间，超时返回 -1
    # ------------------------------------------------------------------
    print("=== 超时处理 ===")
    state = ctrl.move_forward(1.0, timeout=15.0)  # 最多等 15 秒
    if state == -1:
        print("超时！强制停止")
        ctrl.stop()
    else:
        print(f"正常完成 state={state}")


# ======================================================================
# 方式二：手动管理生命周期
# 需要自己调用 release()，否则 GDK 资源不释放
# ======================================================================
ctrl = ChassisController()
try:
    ctrl.move_forward(0.3)
    ctrl.crab_right(0.3)
finally:
    ctrl.release()   # 必须调用，建议放在 finally 里
