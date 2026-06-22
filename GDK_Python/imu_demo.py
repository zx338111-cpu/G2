#!/usr/bin/env python3
"""
IMU（惯性测量单元）接口使用演示程序
展示如何使用agibot_gdk进行IMU数据获取和处理
"""

import time
import agibot_gdk
from typing import Optional, Tuple, List


class ImuDemo:
    def __init__(self):
        """初始化IMU对象"""
        print("正在初始化IMU对象...")
        self.imu = agibot_gdk.Imu()
        time.sleep(2)  # 等待IMU初始化，确保DDS连接建立
        print("IMU初始化完成！")
        
        # 所有可用的IMU类型
        self.imu_types = [
            (agibot_gdk.ImuType.kImuFront, "前部IMU"),
            (agibot_gdk.ImuType.kImuBack, "后部IMU"),
            (agibot_gdk.ImuType.kImuChassis, "底盘IMU")
        ]

    def test_get_latest_imu(self, imu_type: agibot_gdk.ImuType, imu_name: str):
        """测试获取最新IMU数据"""
        print(f"\n{'='*60}")
        print(f"测试 get_latest_imu() - 获取最新{imu_name}数据")
        print(f"{'='*60}")
        
        try:
            imu_data = self.imu.get_latest_imu(imu_type, 1000.0)
            
            if imu_data is not None:
                print(f"时间戳: {imu_data.timestamp_ns} ns")
                
                # 角速度 (弧度/秒)
                print(f"角速度 (rad/s):")
                print(f"  x: {imu_data.angular_velocity.x:.4f}")
                print(f"  y: {imu_data.angular_velocity.y:.4f}")
                print(f"  z: {imu_data.angular_velocity.z:.4f}")
                
                # 线性加速度 (米/秒²)
                print(f"线性加速度 (m/s²):")
                print(f"  x: {imu_data.linear_acceleration.x:.4f}")
                print(f"  y: {imu_data.linear_acceleration.y:.4f}")
                print(f"  z: {imu_data.linear_acceleration.z:.4f}")
                
                return True
            else:
                print(f"未收到{imu_name}数据")
                return False
        except Exception as e:
            print(f"获取最新{imu_name}数据失败: {e}")
            return False

    def test_get_nearest_imu(self, imu_type: agibot_gdk.ImuType, imu_name: str):
        """测试获取指定时间戳附近最近的IMU数据"""
        print(f"\n{'='*60}")
        print(f"测试 get_nearest_imu() - 获取指定时间戳附近最近的{imu_name}数据")
        print(f"{'='*60}")
        
        try:
            # 先获取最新数据以获取当前时间戳
            latest_data = self.imu.get_latest_imu(imu_type, 1000.0)
            
            if latest_data is None:
                print(f"无法获取最新{imu_name}数据，无法测试get_nearest_imu")
                return False
            
            print(f"当前最新数据时间戳: {latest_data.timestamp_ns} ns")
            
            # 获取1秒前的数据
            target_timestamp = latest_data.timestamp_ns - 1000000000  # 1秒 = 10^9 纳秒
            print(f"查找时间戳: {target_timestamp} ns (往前1秒)")
            
            nearest_data = self.imu.get_nearest_imu(imu_type, target_timestamp, 1000.0)
            
            if nearest_data is not None:
                print(f"找到最近数据时间戳: {nearest_data.timestamp_ns} ns")
                print(f"时间差: {(latest_data.timestamp_ns - nearest_data.timestamp_ns) / 1e9:.3f} 秒")
                
                # 角速度 (弧度/秒)
                print(f"角速度 (rad/s):")
                print(f"  x: {nearest_data.angular_velocity.x:.4f}")
                print(f"  y: {nearest_data.angular_velocity.y:.4f}")
                print(f"  z: {nearest_data.angular_velocity.z:.4f}")
                
                # 线性加速度 (米/秒²)
                print(f"线性加速度 (m/s²):")
                print(f"  x: {nearest_data.linear_acceleration.x:.4f}")
                print(f"  y: {nearest_data.linear_acceleration.y:.4f}")
                print(f"  z: {nearest_data.linear_acceleration.z:.4f}")
                
                return True
            else:
                print(f"未找到指定时间戳附近的{imu_name}数据")
                return False
        except Exception as e:
            print(f"获取最近{imu_name}数据失败: {e}")
            return False

    def test_get_imu_fps(self, imu_type: agibot_gdk.ImuType, imu_name: str):
        """测试获取IMU数据采集帧率"""
        print(f"\n{'='*60}")
        print(f"测试 get_imu_fps() - 获取{imu_name}帧率")
        print(f"{'='*60}")
        
        try:
            fps = self.imu.get_imu_fps(imu_type)
            print(f"{imu_name}帧率: {fps} FPS")
            return True
        except RuntimeError as e:
            print(f"获取{imu_name}帧率失败: {e}")
            print("注意: 此接口可能尚未实现")
            return False
        except Exception as e:
            print(f"获取{imu_name}帧率失败: {e}")
            return False

    def test_get_imu_latency(self, imu_type: agibot_gdk.ImuType, imu_name: str, window_seconds: float = 10.0):
        """测试获取IMU数据延迟统计信息"""
        print(f"\n{'='*60}")
        print(f"测试 get_imu_latency() - 获取{imu_name}延迟统计")
        print(f"{'='*60}")
        
        try:
            print(f"收集数据中，等待 {window_seconds} 秒...")
            time.sleep(window_seconds)
            
            latency_stats = self.imu.get_imu_latency(imu_type, window_seconds)
            
            print(f"{imu_name}延迟统计 (窗口: {window_seconds}秒):")
            print(f"  最大延迟: {latency_stats.max_latency_ms:.2f} ms")
            print(f"  平均延迟: {latency_stats.avg_latency_ms:.2f} ms")
            print(f"  P99延迟: {latency_stats.p99_latency_ms:.2f} ms")
            print(f"  P99.9延迟: {latency_stats.p999_latency_ms:.2f} ms")
            print(f"  P99.99延迟: {latency_stats.p9999_latency_ms:.2f} ms")
            
            return True
        except RuntimeError as e:
            print(f"获取{imu_name}延迟统计失败: {e}")
            print("注意: 此接口可能尚未实现，或需要先进行时间同步")
            return False
        except Exception as e:
            print(f"获取{imu_name}延迟统计失败: {e}")
            return False

    def continuous_monitoring(self, imu_type: agibot_gdk.ImuType, imu_name: str, duration: int = 30):
        """连续监控IMU数据"""
        print(f"\n{'='*60}")
        print(f"连续监控模式 - {imu_name}")
        print(f"{'='*60}")
        print(f"监控时长: {duration} 秒 (按Ctrl+C可提前停止)")
        print("=" * 60)
        
        start_time = time.time()
        count = 0
        
        try:
            while time.time() - start_time < duration:
                imu_data = self.imu.get_latest_imu(imu_type, 1000.0)
                
                if imu_data is not None:
                    count += 1
                    elapsed = time.time() - start_time
                    print(f"\r[{elapsed:.1f}s] #{count} - "
                          f"时间戳: {imu_data.timestamp_ns} - "
                          f"角速度: ({imu_data.angular_velocity.x:.2f}, "
                          f"{imu_data.angular_velocity.y:.2f}, "
                          f"{imu_data.angular_velocity.z:.2f}) rad/s - "
                          f"加速度: ({imu_data.linear_acceleration.x:.2f}, "
                          f"{imu_data.linear_acceleration.y:.2f}, "
                          f"{imu_data.linear_acceleration.z:.2f}) m/s²", end="", flush=True)
                else:
                    print(f"\r监控中 - 无数据", end="", flush=True)
                
                time.sleep(0.1)
            
            print(f"\n\n监控完成，共获取 {count} 条数据")
            if count > 0:
                print(f"平均帧率: {count / duration:.2f} FPS")
        except KeyboardInterrupt:
            elapsed = time.time() - start_time
            print(f"\n\n监控已停止，共运行 {elapsed:.1f} 秒，获取 {count} 条数据")
            if count > 0 and elapsed > 0:
                print(f"平均帧率: {count / elapsed:.2f} FPS")

    def get_available_imu_types(self) -> List[Tuple[agibot_gdk.ImuType, str]]:
        """检测可用的IMU类型"""
        available = []
        for imu_type, imu_name in self.imu_types:
            try:
                data = self.imu.get_latest_imu(imu_type, 500.0)
                if data is not None:
                    available.append((imu_type, imu_name))
            except:
                pass
        return available

    def get_test_menu(self):
        """获取测试菜单选项"""
        available = self.get_available_imu_types()
        
        menu = []
        menu.append(("0", "运行全部测试", None))
        
        # 为每个可用的IMU类型添加测试选项
        for idx, (imu_type, imu_name) in enumerate(available, 1):
            base_idx = (idx - 1) * 4 + 1
            menu.append((f"{base_idx}", f"获取最新{imu_name}数据", (self.test_get_latest_imu, imu_type, imu_name)))
            menu.append((f"{base_idx+1}", f"获取最近{imu_name}数据", (self.test_get_nearest_imu, imu_type, imu_name)))
            menu.append((f"{base_idx+2}", f"获取{imu_name}帧率", (self.test_get_imu_fps, imu_type, imu_name)))
            menu.append((f"{base_idx+3}", f"获取{imu_name}延迟统计", (self.test_get_imu_latency, imu_type, imu_name)))
        
        # 添加连续监控选项
        if available:
            menu.append(("m", "连续监控模式", None))
        
        return menu, available

    def print_menu(self):
        """打印测试菜单"""
        menu, available = self.get_test_menu()
        
        print("\n" + "=" * 60)
        print("IMU测试菜单")
        print("=" * 60)
        
        if not available:
            print("警告: 未检测到可用的IMU类型")
            print("请确保机器人已连接并正常运行")
            return
        
        print(f"\n检测到 {len(available)} 个可用的IMU类型:")
        for imu_type, imu_name in available:
            print(f"  - {imu_name}")
        
        print("\n测试选项:")
        for key, name, _ in menu:
            if key == "0":
                print(f"  {key}. {name}")
            elif key == "m":
                print(f"  {key}. {name}")
            else:
                print(f"  {key}. {name}")
        
        print("  q. 退出")

    def run_single_test(self, test_func, *args):
        """运行单个测试"""
        try:
            result = test_func(*args)
            if result:
                print("\n测试通过")
            else:
                print("\n测试失败")
        except Exception as e:
            print(f"\n测试出错: {e}")

    def run_comprehensive_test(self):
        """运行综合测试"""
        print("\n" + "=" * 60)
        print("开始综合测试")
        print("=" * 60)
        
        available = self.get_available_imu_types()
        
        if not available:
            print("未检测到可用的IMU类型，无法进行测试")
            return
        
        total_tests = 0
        passed_tests = 0
        
        # 测试每个可用的IMU类型
        for imu_type, imu_name in available:
            print(f"\n测试 {imu_name}...")
            
            # 测试 get_latest_imu
            total_tests += 1
            if self.test_get_latest_imu(imu_type, imu_name):
                passed_tests += 1
            time.sleep(0.5)
            
            # 测试 get_nearest_imu
            total_tests += 1
            if self.test_get_nearest_imu(imu_type, imu_name):
                passed_tests += 1
            time.sleep(0.5)
            
            # 测试 get_imu_fps
            total_tests += 1
            if self.test_get_imu_fps(imu_type, imu_name):
                passed_tests += 1
            time.sleep(0.5)
            
            # 测试 get_imu_latency (使用较短的窗口时间)
            total_tests += 1
            if self.test_get_imu_latency(imu_type, imu_name, window_seconds=5.0):
                passed_tests += 1
            time.sleep(1.0)
        
        # 显示测试总结
        print("\n" + "=" * 60)
        print("测试总结")
        print("=" * 60)
        print(f"总测试数: {total_tests}")
        print(f"通过数: {passed_tests}")
        print(f"失败数: {total_tests - passed_tests}")
        if total_tests > 0:
            print(f"通过率: {passed_tests / total_tests * 100:.1f}%")

    def run_interactive_test(self):
        """运行交互式测试"""
        print("IMU测试程序")
        print("=" * 60)
        
        menu, available = self.get_test_menu()
        menu_dict = {key: (name, func) for key, name, func in menu}
        
        while True:
            self.print_menu()
            
            if not available:
                print("\n无法继续，退出程序")
                break
            
            choice = input("\n请选择测试项 (输入数字，q退出): ").strip()
            
            if choice.lower() == 'q':
                print("退出测试程序")
                break
            
            if choice == "0":
                # 运行全部测试
                self.run_comprehensive_test()
            elif choice == "m":
                # 连续监控模式
                if available:
                    print("\n可用的IMU类型:")
                    for idx, (_, imu_name) in enumerate(available, 1):
                        print(f"  {idx}. {imu_name}")
                    
                    imu_choice = input("请选择要监控的IMU (输入数字): ").strip()
                    try:
                        imu_idx = int(imu_choice) - 1
                        if 0 <= imu_idx < len(available):
                            imu_type, imu_name = available[imu_idx]
                            duration = input("监控时长(秒，默认30): ").strip()
                            duration = int(duration) if duration else 30
                            self.continuous_monitoring(imu_type, imu_name, duration)
                        else:
                            print("无效选择")
                    except ValueError:
                        print("无效输入")
            elif choice in menu_dict:
                name, func = menu_dict[choice]
                if func:
                    if isinstance(func, tuple):
                        # 解包函数和参数
                        test_func, *args = func
                        print(f"\n开始测试: {name}")
                        self.run_single_test(test_func, *args)
                    else:
                        print(f"\n开始测试: {name}")
                        self.run_single_test(func)
                else:
                    self.run_comprehensive_test()
            else:
                print("无效选择，请重新输入")

    def cleanup(self):
        """清理资源"""
        try:
            if self.imu:
                result = self.imu.close_imu()
                if result == agibot_gdk.GDKRes.kSuccess:
                    print("IMU已关闭")
                else:
                    print("IMU关闭失败")
        except Exception as e:
            print(f"清理资源时出错: {e}")


def main():
    """主函数 - 运行交互式测试"""
    # 初始化GDK系统
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK初始化失败")
        return
    print("GDK初始化成功")
    
    demo = None
    try:
        demo = ImuDemo()
        demo.run_interactive_test()
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"\n程序运行出错: {e}")
    finally:
        # 清理资源
        if demo:
            demo.cleanup()
        
        # 释放GDK系统资源
        if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
            print("GDK释放失败")
        else:
            print("GDK释放成功")


if __name__ == "__main__":
    main()
