#!/usr/bin/env python3
import sys
import time
import threading
import agibot_gdk
import argparse
import tty
import termios
import select
import fcntl

# 全局变量保存终端设置
_old_termios = None
_terminal_setup = False

def setup_terminal():
    """设置终端为原始模式（一次性设置）"""
    global _old_termios, _terminal_setup
    if _terminal_setup:
        return
    
    fd = sys.stdin.fileno()
    _old_termios = termios.tcgetattr(fd)
    
    # 设置为原始模式
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~(termios.ICANON | termios.ECHO)
    new[6][termios.VMIN] = 0
    new[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, new)
    
    _terminal_setup = True

def restore_terminal():
    """恢复终端设置"""
    global _old_termios, _terminal_setup
    if _old_termios is not None:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, _old_termios)
        _terminal_setup = False

def getch():
    """非阻塞读取一个字符（终端已设置为原始模式）"""
    try:
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            print(f"key: {ch}")
            return ch
    except (IOError, OSError):
        pass
    return None


class AckermannConverter:
    def __init__(self, max_linear_speed=1.0, max_steering_angle=1.0, wheel_base=0.3):
        self.max_linear_speed = max_linear_speed
        self.max_steering_angle = max_steering_angle
        self.wheel_base = wheel_base

    def convert(self, x, y):
        """
        x: 左右(-1~1)，y: 前后(-1~1)
        """
        linear_speed = y * self.max_linear_speed
        steering_angle = -x * self.max_steering_angle
        
        twist = agibot_gdk.Twist()
        twist.linear = agibot_gdk.Vector3()
        twist.angular = agibot_gdk.Vector3()
        twist.linear.x = linear_speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = steering_angle
        
        return twist


class KeyboardController:
    def __init__(self, control_mode=0):
        self.x = 0.0
        self.y = 0.0
        self.control_mode = control_mode  # 0: 阿克曼, 1: 蟹行（固定，不支持切换）
        self.speed_scale = 0.1
        self.running = True
        self.need_request_control = False  # 是否需要请求控制模式
        self.pressed_keys = set()  # 当前按下的键
        self.lock = threading.Lock()
        
    def update_key(self, key):
        """更新按键状态 - 检测到按键就认为按下，检测不到就认为松开"""
        with self.lock:
            if key is None:
                # 没有检测到按键，清除所有按下的键（模拟松开）
                self.pressed_keys.clear()
                self._update_control_values()
                return
            
            key_lower = key.lower()
            
            # 处理特殊键（不需要持续按下的）
            if key == '+':
                self.speed_scale = min(1.0, self.speed_scale + 0.1)
                print(f"速度比例: {self.speed_scale:.1f}")
                self._update_control_values()
                return
            elif key == '-':
                self.speed_scale = max(0.01, self.speed_scale - 0.1)
                print(f"速度比例: {self.speed_scale:.1f}")
                self._update_control_values()
                return
            elif key_lower == 'q':
                self.running = False
                self.pressed_keys.clear()
                self._update_control_values()
                return
            
            # 方向键：检测到就添加到集合（按下），检测不到就自动清除（松开）
            if key_lower in ['w', 's', 'a', 'd', ' ']:
                self.pressed_keys.add(key_lower)
                self.need_request_control = True
                self._update_control_values()
    
    def _update_control_values(self):
        """根据按下的键更新控制值"""
        self.x = 0.0
        self.y = 0.0
        
        if 'w' in self.pressed_keys:
            self.y = self.speed_scale
        elif 's' in self.pressed_keys:
            self.y = -self.speed_scale
        
        if 'a' in self.pressed_keys:
            self.x = -self.speed_scale
        elif 'd' in self.pressed_keys:
            self.x = self.speed_scale
        
        if ' ' in self.pressed_keys:
            self.x = 0.0
            self.y = 0.0
    
    def get_state(self):
        """获取当前控制状态"""
        with self.lock:
            return self.x, self.y, self.need_request_control, self.running
    
    def clear_request_flag(self):
        """清除请求控制标志"""
        with self.lock:
            self.need_request_control = False


def keyboard_listener(controller):
    """键盘监听线程 - 持续检测按键状态（按下即捕获，松开即停止）"""
    while controller.running:
        key = getch()
        controller.update_key(key)  # 检测到按键=按下，检测不到=松开
        time.sleep(0.01)  # 10ms 检查一次按键（快速检测以实现实时响应）


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--control_mode", type=int, default=0)
    args = parser.parse_args()
    if args.control_mode not in [0, 1]:
        print("Invalid control mode")
        exit(1)
    if args.control_mode == 0:
        control_mode = 0
    elif args.control_mode == 1:
        control_mode = 1
    else:
        print("Invalid control mode")
        return
    print("================ 键盘遥控示例 ================")
    print("W/S: 前进 / 后退（按下控制，松开停止）")
    print("A/D: 左转 / 右转（按下控制，松开停止）")
    print("+/-: 加速 / 减速")
    print("空格: 停止")
    print("Q  : 退出")
    print("==============================================")
    
    # 设置终端为原始模式（按键按下立即响应，不需要回车）
    setup_terminal()
    
    # 初始化 GDK
    agibot_gdk.gdk_init()
    pnc = agibot_gdk.Pnc()
    time.sleep(0.5)  # 等待初始化

    pnc.request_chassis_control(control_mode)
    time.sleep(0.5)
    converter = AckermannConverter(1.0, 1.0, 0.3)
    controller = KeyboardController(control_mode=control_mode)
    
    # 启动键盘监听线程
    kb_thread = threading.Thread(target=keyboard_listener, args=(controller,), daemon=True)
    kb_thread.start()
    
    # 控制循环：20Hz (50ms 间隔)
    control_interval = 1.0 / 20.0  # 50ms
    last_control_time = 0.0
    
    try:
        while controller.running:
            current_time = time.time()
            
            # 检查当前控制状态
            x, y, need_request_control, running = controller.get_state()
            
            if not running:
                break 
            
            # 判断是否有活跃的控制命令（x 或 y 不为 0）
            has_active_control = (abs(x) > 0.001 or abs(y) > 0.001)
            
            # 只在有活跃控制时，按 20Hz 发送命令
            if has_active_control:
                # 检查是否到了发送时间（20Hz）
                if current_time - last_control_time >= control_interval:
                    if control_mode == 0:
                        # 阿克曼模式
                        twist = converter.convert(x, y)
                        pnc.move_chassis(twist)
                    else:
                        # 蟹行模式
                        twist = agibot_gdk.Twist()
                        twist.linear = agibot_gdk.Vector3()
                        twist.angular = agibot_gdk.Vector3()
                        twist.linear.x = y
                        twist.linear.y = -x
                        twist.linear.z = 0.0
                        twist.angular.x = 0.0
                        twist.angular.y = 0.0
                        twist.angular.z = 0.0
                        pnc.move_chassis(twist)
                    
                    last_control_time = current_time
            else:
                # 没有活跃控制，重置时间戳（下次按键时立即发送）
                last_control_time = 0.0
            
            # 短暂休眠，避免 CPU 占用过高
            time.sleep(0.001)  # 1ms
            
    except KeyboardInterrupt:
        print("\n收到中断信号")
    finally:
        # 退出前发送停止命令
        twist = agibot_gdk.Twist()
        twist.linear = agibot_gdk.Vector3()
        twist.angular = agibot_gdk.Vector3()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pnc.move_chassis(twist)

        task_state = pnc.get_task_state()
        task_id = task_state.id
        pnc.cancel_task(task_id)
        
        # 恢复终端设置
        restore_terminal()
        print("退出遥控")

if __name__ == "__main__":
    main()

