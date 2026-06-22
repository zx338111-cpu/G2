#!/usr/bin/env python3
"""
超声波雷达(UltrasonicRadar)接口使用演示程序
展示如何使用agibot_gdk进行超声波雷达数据获取和处理
"""

import time
import agibot_gdk
from typing import Optional


class UltrasonicRadarDemo:
    def __init__(self):
        """初始化超声波雷达对象"""
        print("正在初始化超声波雷达对象...")
        self.radar = agibot_gdk.UltrasonicRadar()
        time.sleep(1)  # 等待初始化完成
        print("超声波雷达初始化完成！")

    def test_get_latest_ultrasonic_radar(self):
        """测试获取最新超声波雷达数据"""
        print(f"\n{'='*60}")
        print("测试 get_latest_ultrasonic_radar() - 获取最新超声波雷达数据")
        print(f"{'='*60}")
        
        try:
            radar_data = self.radar.get_latest_ultrasonic_radar()
            
            print(f"时间戳: {radar_data['timestamp_ns']} ns")
            print(f"超声波雷达数量: {len(radar_data['ultrasonic_radar_datas'])}")
            
            if radar_data['ultrasonic_radar_datas']:
                print(f"\n超声波雷达数据:")
                for data in radar_data['ultrasonic_radar_datas']:
                    fault_status = "正常" if data['fault_state'] == 0 else f"故障({data['fault_state']})"
                    print(f"  雷达ID {data['id']}: "
                          f"距离={data['distance_mm']} mm, "
                          f"故障状态={fault_status}")
            else:
                print("未检测到超声波雷达数据")
            
            return True
        except Exception as e:
            print(f"获取最新超声波雷达数据失败: {e}")
            return False

    def test_get_nearest_ultrasonic_radar(self):
        """测试获取指定时间戳附近的数据"""
        print(f"\n{'='*60}")
        print("测试 get_nearest_ultrasonic_radar() - 获取指定时间戳附近的数据")
        print(f"{'='*60}")
        
        try:
            # 先获取最新数据
            latest_data = self.radar.get_latest_ultrasonic_radar()
            print(f"最新数据时间戳: {latest_data['timestamp_ns']} ns")
            
            # 查找最近的数据（往前1秒）
            target_timestamp = latest_data['timestamp_ns'] - 1000000000  # 1秒 = 1,000,000,000 纳秒
            print(f"目标时间戳: {target_timestamp} ns (往前1秒)")
            
            nearest_data = self.radar.get_nearest_ultrasonic_radar(target_timestamp)
            
            print(f"找到的数据时间戳: {nearest_data['timestamp_ns']} ns")
            time_diff = abs(nearest_data['timestamp_ns'] - target_timestamp)
            print(f"时间差: {time_diff} ns ({time_diff / 1000000:.2f} ms)")
            print(f"超声波雷达数量: {len(nearest_data['ultrasonic_radar_datas'])}")
            
            if nearest_data['ultrasonic_radar_datas']:
                print(f"\n超声波雷达数据:")
                for i, data in enumerate(nearest_data['ultrasonic_radar_datas']):
                    fault_status = "正常" if data['fault_state'] == 0 else f"故障({data['fault_state']})"
                    print(f"  雷达[{i}]: "
                          f"距离={data['distance_mm']} mm, "
                          f"故障状态={fault_status}")
            
            return True
        except Exception as e:
            print(f"获取最近超声波雷达数据失败: {e}")
            return False

    def test_get_ultrasonic_radar_fps(self):
        """测试获取超声波雷达帧率"""
        print(f"\n{'='*60}")
        print("测试 get_ultrasonic_radar_fps() - 获取超声波雷达帧率")
        print(f"{'='*60}")
        
        try:
            print("等待数据积累（2秒）...")
            time.sleep(2)
            
            fps = self.radar.get_ultrasonic_radar_fps()
            print(f"超声波雷达帧率: {fps:.2f} FPS")
            return True
        except Exception as e:
            print(f"获取超声波雷达帧率失败: {e}")
            return False

    def test_get_ultrasonic_radar_latency(self):
        """测试获取超声波雷达延迟统计"""
        print(f"\n{'='*60}")
        print("测试 get_ultrasonic_radar_latency() - 获取超声波雷达延迟统计")
        print(f"{'='*60}")
        
        try:
            window_seconds = 10.0
            print(f"等待数据积累（{window_seconds}秒）以进行延迟统计...")
            time.sleep(window_seconds)
            
            latency_stats = self.radar.get_ultrasonic_radar_latency(window_seconds)
            print(f"超声波雷达延迟统计 (窗口: {window_seconds}秒):")
            print(f"  最大延迟: {latency_stats.max_latency_ms:.2f} ms")
            print(f"  平均延迟: {latency_stats.avg_latency_ms:.2f} ms")
            print(f"  99%延迟: {latency_stats.p99_latency_ms:.2f} ms")
            print(f"  99.9%延迟: {latency_stats.p999_latency_ms:.2f} ms")
            print(f"  99.99%延迟: {latency_stats.p9999_latency_ms:.2f} ms")
            return True
        except Exception as e:
            print(f"获取超声波雷达延迟统计失败: {e}")
            return False

    def continuous_monitoring(self, duration_seconds=30):
        """连续监控超声波雷达数据"""
        print(f"\n{'='*60}")
        print(f"连续监控超声波雷达数据 ({duration_seconds}秒)")
        print(f"{'='*60}")
        print("按Ctrl+C停止监控")
        
        start_time = time.time()
        count = 0
        
        try:
            while time.time() - start_time < duration_seconds:
                radar_data = self.radar.get_latest_ultrasonic_radar()
                
                if radar_data is not None:
                    count += 1
                    elapsed = time.time() - start_time
                    
                    # 显示基本信息
                    print(f"\r监控 #{count} - "
                          f"时间戳: {radar_data['timestamp_ns']} - "
                          f"雷达数量: {len(radar_data['ultrasonic_radar_datas'])} - "
                          f"运行时间: {elapsed:.1f}s", end="")
                    
                    # 每10帧显示一次详细信息
                    if count % 10 == 0:
                        print(f"\n  第{count}帧详细信息:")
                        for data in radar_data['ultrasonic_radar_datas'][:5]:  # 只显示前5个
                            fault_status = "正常" if data['fault_state'] == 0 else f"故障({data['fault_state']})"
                            print(f"    雷达ID {data['id']}: "
                                  f"距离={data['distance_mm']} mm, "
                                  f"状态={fault_status}")
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print(f"\n\n监控被用户中断")
        
        if count > 0:
            print(f"\n\n监控统计:")
            print(f"  总帧数: {count}")
            print(f"  监控时长: {time.time() - start_time:.1f} 秒")

    def get_test_menu(self):
        """获取测试菜单选项"""
        return [
            ("1", "获取最新超声波雷达数据", self.test_get_latest_ultrasonic_radar),
            ("2", "获取指定时间戳附近的数据", self.test_get_nearest_ultrasonic_radar),
            ("3", "获取超声波雷达帧率", self.test_get_ultrasonic_radar_fps),
            ("4", "获取超声波雷达延迟统计", self.test_get_ultrasonic_radar_latency),
            ("5", "连续监控模式", None),
            ("0", "运行全部测试", None),
        ]

    def print_menu(self):
        """打印测试菜单"""
        print("\n" + "=" * 60)
        print("超声波雷达测试菜单")
        print("=" * 60)
        menu = self.get_test_menu()
        for key, name, _ in menu:
            print(f"{key}. {name}")

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
        print("超声波雷达综合测试程序")
        print("=" * 50)
        
        test_results = []
        
        # 1. 数据获取测试
        print("\n1. 数据获取接口测试")
        test_results.append(("获取最新数据", self.test_get_latest_ultrasonic_radar()))
        test_results.append(("获取指定时间戳附近的数据", self.test_get_nearest_ultrasonic_radar()))
        
        # 2. 性能信息测试
        print("\n2. 性能信息接口测试")
        test_results.append(("获取帧率", self.test_get_ultrasonic_radar_fps()))
        test_results.append(("获取延迟统计", self.test_get_ultrasonic_radar_latency()))
        
        # 3. 显示测试总结
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
        print("超声波雷达测试程序")
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
            elif choice == "5":
                # 连续监控模式
                duration = input("监控时长(秒，默认30): ").strip()
                duration = int(duration) if duration else 30
                self.continuous_monitoring(duration)
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
    
    demo = None
    try:
        demo = UltrasonicRadarDemo()
        demo.run_interactive_test()
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"\n程序运行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 关闭超声波雷达
        if demo is not None:
            try:
                demo.radar.close_ultrasonic_radar()
                print("超声波雷达已关闭")
            except:
                pass
        
        # 释放GDK系统资源
        if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
            print("GDK释放失败")
        else:
            print("\nGDK释放成功")


if __name__ == "__main__":
    main()

