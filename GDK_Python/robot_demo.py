#!/usr/bin/env python3
"""
机器人控制(Robot)接口使用演示程序
展示如何使用agibot_gdk进行机器人状态获取和控制
"""

import time
import agibot_gdk
from typing import Dict, Any, Optional


class RobotDemo:
    def __init__(self):
        """初始化机器人对象"""
        print("正在初始化机器人对象...")
        self.robot = agibot_gdk.Robot()
        time.sleep(2)  # 等待机器人初始化
        print("机器人初始化完成！")

    def test_get_joint_states(self):
        """测试获取关节状态"""
        print(f"\n{'='*60}")
        print("测试 get_joint_states() - 获取关节状态")
        print(f"{'='*60}")
        
        try:
            joint_states = self.robot.get_joint_states()
            print(f"关节数量: {joint_states['nums']}")
            print(f"时间戳: {joint_states['timestamp']}")
            
            for i, state in enumerate(joint_states['states']):
                print(f"  关节 {i+1}: {state['name']}")
                print(f"    位置: {state['position']:.3f} rad")
                print(f"    速度: {state['velocity']:.3f} rad/s")
                print(f"    力矩: {state['effort']:.3f} N·m")
                print(f"    电机位置: {state['motor_position']:.3f} rad")
                print(f"    电机电流: {state['motor_current']:.3f} A")
                print(f"    错误码: {state['error_code']}")
            return True
            
        except Exception as e:
            print(f"获取关节状态失败: {e}")
            return False

    def test_get_whole_body_status(self):
        """测试获取全身状态"""
        print(f"\n{'='*60}")
        print("测试 get_whole_body_status() - 获取全身状态")
        print(f"{'='*60}")
        
        try:
            status = self.robot.get_whole_body_status()
            print(f"时间戳: {status['timestamp']}")
            print(f"右执行器型号: {status['right_end_model']}")
            print(f"左执行器型号: {status['left_end_model']}")
            
            print(f"\n错误状态检查:")
            error_items = [
                ('右臂', status['right_arm_error']),
                ('左臂', status['left_arm_error']),
                ('右执行器', status['right_end_error']),
                ('左执行器', status['left_end_error']),
                ('腰部', status['waist_error']),
                ('升降', status['lift_error']),
                ('头部', status['neck_error']),
                ('底盘', status['chassis_error']),
            ]
            
            for name, error_code in error_items:
                if error_code == 0:
                    print(f"  {name}: 正常")
                else:
                    print(f"  {name}: 错误码 {error_code}")
            
            print(f"\n控制状态:")
            print(f"  右臂控制: {'是' if status['right_arm_control'] else '否'}")
            print(f"  左臂控制: {'是' if status['left_arm_control'] else '否'}")
            print(f"  右臂急停: {'是' if status['right_arm_estop'] else '否'}")
            print(f"  左臂急停: {'是' if status['left_arm_estop'] else '否'}")
            return True
        except Exception as e:
            print(f"获取全身状态失败: {e}")
            return False

    def test_get_end_state(self):
        """测试获取末端执行器状态"""
        print(f"\n{'='*60}")
        print("测试 get_end_state() - 获取末端执行器状态")
        print(f"{'='*60}")
        
        try:
            end_state = self.robot.get_end_state()
            
            for side in ['left', 'right']:
                state = end_state[f'{side}_end_state']
                print(f"\n{side}执行器:")
                print(f"  控制状态: {state['controlled']}")
                print(f"  类型: {state['type']}")
                print(f"  关节名称: {state['names']}")
                
                if state['end_states']:
                    print(f"  关节数量: {len(state['end_states'])}")
                    for i, joint_state in enumerate(state['end_states'][:3]):
                        print(f"    关节 {i+1}:")
                        print(f"      ID: {joint_state['id']}")
                        print(f"      启用: {joint_state['enable']}")
                        print(f"      位置: {joint_state['position']:.3f} rad")
                        print(f"      速度: {joint_state['velocity']:.3f} rad/s")
                        print(f"      电流: {joint_state['current']:.3f} A")
                        print(f"      温度: {joint_state['temperature']:.1f} °C")
                        print(f"      错误码: {joint_state['err_code']}")
            return True
        except Exception as e:
            print(f"获取末端执行器状态失败: {e}")
            return False

    def test_get_motion_control_status(self):
        """测试获取运动控制状态"""
        print(f"\n{'='*60}")
        print("测试 get_motion_control_status() - 获取运动控制状态")
        print(f"{'='*60}")
        
        try:
            status = self.robot.get_motion_control_status()
            
            mode_map = {
                0: "停止",
                1: "G1_伺服",
                2: "路径规划",
                5: "G2_伺服"
            }
            
            print(f"运动模式: {mode_map.get(status.mode, f'未知({status.mode})')}")
            print(f"错误码: {status.error_code}")
            if status.error_msg:
                print(f"错误信息: {status.error_msg}")
            print(f"关节数量: {len(status.frame_names)}")
            print(f"碰撞对数量: {len(status.collision_pairs_1)}")
            
            for i, frame_name in enumerate(status.frame_names):
                print(f"  {i}: {frame_name}")
            
            return True
        except Exception as e:
            print(f"获取运动控制状态失败: {e}")
            return False

    def test_move_head_joint(self):
        """测试控制头部位置"""
        print(f"\n{'='*60}")
        print("测试 move_head_joint() - 控制头部位置")
        print(f"{'='*60}")
        
        try:
            # 获取当前关节状态
            joint_states = self.robot.get_joint_states()
            current_positions = {}
            for state in joint_states['states']:
                if 'head' in state['name'].lower():
                    current_positions[state['name']] = state['position']
            
            # 头部关节顺序（按实际代码实现）：idx11_head_joint1, idx12_head_joint2, idx13_head_joint3
            head_positions = [0.0, 0.0, 0.0]
            head_velocities = [0.2, 0.2, 0.2]
            
            print(f"发送头部控制命令...")
            print(f"  位置: {head_positions}")
            print(f"  速度: {head_velocities}")
            
            result = self.robot.move_head_joint(head_positions, head_velocities)
            print(f"头部控制成功 (返回值: {result})")
            return True
        except Exception as e:
            print(f"头部控制失败: {e}")
            return False

    def test_move_ee_pos(self):
        """测试控制末端执行器位置（夹爪）"""
        print(f"\n{'='*60}")
        print("测试 move_ee_pos() - 控制末端执行器位置")
        print(f"{'='*60}")
        
        try:
            action = {
                "left_ee_state": {
                    "joint_position": 0.0,  # 左手夹爪位置
                },
                "right_ee_state": {
                    "joint_position": 0.0,  # 右手夹爪位置
                }
            }
            
            print(f"发送夹爪控制命令...")
            print(f"  左夹爪位置: {action['left_ee_state']['joint_position']} rad")
            print(f"  右夹爪位置: {action['right_ee_state']['joint_position']} rad")
            
            result = self.robot.move_ee_pos(action)
            print(f"夹爪控制成功 (返回值: {result})")
            return True
        except Exception as e:
            print(f"夹爪控制失败: {e}")
            return False

    def test_get_chassis_power_state(self):
        """测试获取底盘电源状态"""
        print(f"\n{'='*60}")
        print("测试 get_chassis_power_state() - 获取底盘电源状态")
        print(f"{'='*60}")
        
        try:
            power_state = self.robot.get_chassis_power_state()
            print(f"时间戳: {power_state.timestamp}")
            print(f"电池主电源开关状态: {power_state.battery_main_power_switch_state}")
            print(f"急停踏板状态: {power_state.emergency_stop_pedal_state}")
            print(f"底盘电源板状态: {power_state.chassis_power_board_state}")
            print(f"底盘电源板温度: {power_state.chassis_power_board_temperature:.1f} °C")
            print(f"电池数量: {len(power_state.battery_states)}")
            
            if power_state.battery_states:
                print(f"\n电池状态:")
                for i, battery in enumerate(power_state.battery_states[:3]):
                    print(f"  电池 {i+1}: 电压={battery.voltage:.2f}V, "
                          f"电流={battery.current:.2f}A, "
                          f"温度={battery.temperature:.1f}°C")
            return True
        except Exception as e:
            print(f"获取底盘电源状态失败: {e}")
            return False

    def test_get_chest_power_state(self):
        """测试获取胸部电源状态"""
        print(f"\n{'='*60}")
        print("测试 get_chest_power_state() - 获取胸部电源状态")
        print(f"{'='*60}")
        
        try:
            power_state = self.robot.get_chest_power_state()
            print(f"时间戳: {power_state.timestamp}")
            print(f"开关机请求: {power_state.power_onoff_req}")
            print(f"急停按钮请求: {power_state.emergency_stop_button_req}")
            print(f"胸部电源板状态: {power_state.chest_power_board_power_state}")
            print(f"域控制器电源状态: {power_state.domain_controller_power_state}")
            print(f"胸部电源板温度: {power_state.chest_power_board_temperature:.1f} °C")
            print(f"胸部电源板故障状态: {power_state.chest_power_board_fault_state}")
            return True
        except Exception as e:
            print(f"获取胸部电源状态失败: {e}")
            return False

    def get_test_menu(self):
        """获取测试菜单选项"""
        return [
            ("1", "获取关节状态", self.test_get_joint_states),
            ("2", "获取全身状态", self.test_get_whole_body_status),
            ("3", "获取末端执行器状态", self.test_get_end_state),
            ("4", "获取运动控制状态", self.test_get_motion_control_status),
            ("5", "获取底盘电源状态", self.test_get_chassis_power_state),
            ("6", "获取胸部电源状态", self.test_get_chest_power_state),
            ("7", "控制头部位置", self.test_move_head_joint),
            ("8", "控制末端执行器位置", self.test_move_ee_pos),
            ("0", "运行全部测试", None),
        ]

    def print_menu(self):
        """打印测试菜单"""
        print("\n" + "=" * 60)
        print("机器人控制测试菜单")
        print("=" * 60)
        menu = self.get_test_menu()
        for key, name, _ in menu:
            print(f"{key}. {name}")
        print("\n注意: 控制接口（选项7-8）会实际控制机器人，请确保安全环境")

    def run_single_test(self, test_func, test_name):
        """运行单个测试"""
        try:
            result = test_func()
            status = "成功" if result else "失败"
            print(f"\n{test_name}: {status}")
            return result
        except Exception as e:
            print(f"\n{test_name}执行出错: {e}")
            return False

    def run_comprehensive_test(self):
        """运行综合测试"""
        print("机器人控制综合测试程序")
        print("=" * 50)
        
        test_results = []
        
        # 1. 状态获取测试
        print("\n1. 状态获取接口测试")
        test_results.append(("获取关节状态", self.test_get_joint_states()))
        test_results.append(("获取全身状态", self.test_get_whole_body_status()))
        test_results.append(("获取末端执行器状态", self.test_get_end_state()))
        test_results.append(("获取运动控制状态", self.test_get_motion_control_status()))
        
        # 2. 电源状态测试
        print("\n2. 电源状态接口测试")
        test_results.append(("获取底盘电源状态", self.test_get_chassis_power_state()))
        test_results.append(("获取胸部电源状态", self.test_get_chest_power_state()))
        
        # 3. 控制接口测试（注意：这些接口会实际控制机器人，请谨慎使用）
        print("\n3. 控制接口测试（只发送命令，不等待执行完成）")
        print("注意: 控制接口会实际控制机器人，请确保安全环境")
        test_results.append(("控制头部位置", self.test_move_head_joint()))
        test_results.append(("控制末端执行器位置", self.test_move_ee_pos()))
        
        # 4. 显示测试总结
        print(f"\n{'='*60}")
        print("测试总结")
        print(f"{'='*60}")
        success_count = sum(1 for _, result in test_results if result)
        total_count = len(test_results)
        
        for test_name, result in test_results:
            status = "成功" if result else "失败"
            print(f"{test_name}: {status}")
        
        print(f"\n总测试数: {total_count}")
        print(f"成功数: {success_count}")
        print(f"失败数: {total_count - success_count}")
        print(f"成功率: {success_count/total_count*100:.1f}%")

    def run_interactive_test(self):
        """运行交互式测试"""
        print("机器人控制测试程序")
        print("=" * 50)
        
        menu = self.get_test_menu()
        menu_dict = {key: (name, func) for key, name, func in menu}
        
        while True:
            self.print_menu()
            choice = input("\n请选择测试项 (输入数字，q退出): ").strip()
            
            if choice.lower() == 'q':
                print("退出测试程序")
                break
            
            if choice == "0":
                # 运行全部测试
                self.run_comprehensive_test()
            elif choice in menu_dict:
                name, func = menu_dict[choice]
                if func:
                    print(f"\n开始测试: {name}")
                    self.run_single_test(func, name)
                else:
                    self.run_comprehensive_test()
            else:
                print("无效选择，请重新输入")


def main():
    """主函数 - 运行交互式测试"""
    # 初始化GDK系统
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return
    
    print("GDK初始化成功")
    
    try:
        demo = RobotDemo()
        demo.run_interactive_test()
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"\n程序运行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 释放GDK系统资源
        if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
            print("GDK释放失败")
        else:
            print("\nGDK释放成功")


if __name__ == "__main__":
    main()

