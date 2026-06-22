#!/usr/bin/env python3
"""
激光雷达(Lidar)接口使用演示程序
展示如何使用agibot_gdk进行激光雷达数据获取和处理
"""

import time
import agibot_gdk
import numpy as np
import struct
from typing import List, Dict, Any, Optional

class LidarDemo:
    def __init__(self):
        """初始化激光雷达对象"""
        print("正在初始化激光雷达对象...")
        self.lidar = agibot_gdk.Lidar()
        time.sleep(3)
        
        # 所有可用的激光雷达类型
        self.lidar_types = [
            (agibot_gdk.LidarType.kLidarFront, "前部激光雷达"),
            (agibot_gdk.LidarType.kLidarBack, "后部激光雷达")
        ]
        
        print("激光雷达初始化完成！")

    def test_lidar_type(self, lidar_type, lidar_name, test_count=3):
        """测试指定类型的激光雷达"""
        print(f"\n{'='*60}")
        print(f"测试 {lidar_name} ({lidar_type})")
        print(f"{'='*60}")
        
        success_count = 0
        
        for i in range(test_count):
            print(f"\n--- {lidar_name} 数据 #{i+1} ---")
            
            # 获取最新点云数据
            pointcloud = self.lidar.get_latest_pointcloud(lidar_type, 1000.0)
            
            if pointcloud is not None:
                success_count += 1
                self.print_pointcloud_info(pointcloud, lidar_name, i+1)
                
                # 测试 get_nearest_pointcloud 方法
                print(f"--------------------------------")
                print("测试 get_nearest_pointcloud 方法:")
                pointcloud_nearest = self.lidar.get_nearest_pointcloud(
                    lidar_type, pointcloud.timestamp_ns-1000000000, 1000.0)
                
                if pointcloud_nearest is not None:
                    print(f"✅ 最近点云数据: {pointcloud_nearest.timestamp_ns}")
                    print(f"点云尺寸: {pointcloud_nearest.width} x {pointcloud_nearest.height}")
                else:
                    print(f"❌ 未找到最近的 {lidar_name} 数据")
                    
            else:
                print(f"❌ 未收到 {lidar_name} 数据 #{i+1}")
            
            time.sleep(1.0)
        
        print(f"\n{lidar_name} 测试结果: {success_count}/{test_count} 成功")
        return success_count, test_count

    def print_pointcloud_info(self, pointcloud, lidar_name, data_num):
        """打印点云信息"""
        print(f"✅ 时间戳: {pointcloud.timestamp_ns}")
        print(f"点云尺寸: {pointcloud.width} x {pointcloud.height}")
        print(f"点步长: {pointcloud.point_step}")
        print(f"行步长: {pointcloud.row_step}")
        print(f"是否大端序: {pointcloud.is_bigendian}")
        print(f"是否密集: {pointcloud.is_dense}")
        
        # 打印字段信息
        print(f"字段数量: {len(pointcloud.fields)}")
        for j, field in enumerate(pointcloud.fields):
            print(f"  字段 {j+1}: {field.name} (偏移: {field.offset}, "
                  f"类型: {field.datatype}, 数量: {field.count})")

    def get_lidar_fps(self, lidar_type, lidar_name):
        """获取激光雷达帧率"""
        try:
            fps = self.lidar.get_lidar_fps(lidar_type)
            print(f"{lidar_name} 帧率: {fps:.2f} FPS")
            return fps
        except Exception as e:
            print(f"获取 {lidar_name} 帧率失败: {e}")
            return None

    def get_lidar_latency(self, lidar_type, lidar_name, window_seconds=10.0):
        """获取激光雷达延迟统计"""
        try:
            latency_stats = self.lidar.get_lidar_latency(lidar_type, window_seconds)
            print(f"{lidar_name} 延迟统计 (窗口: {window_seconds}秒):")
            print(f"  最大延迟: {latency_stats.max_latency_ms:.2f} ms")
            print(f"  平均延迟: {latency_stats.avg_latency_ms:.2f} ms")
            print(f"  99%延迟: {latency_stats.p99_latency_ms:.2f} ms")
            print(f"  99.9%延迟: {latency_stats.p999_latency_ms:.2f} ms")
            print(f"  99.99%延迟: {latency_stats.p9999_latency_ms:.2f} ms")
            return latency_stats
        except Exception as e:
            print(f"获取 {lidar_name} 延迟统计失败: {e}")
            return None

    def parse_pointcloud_data(self, pointcloud) -> Optional[np.ndarray]:
        """解析点云数据为numpy数组"""
        if not hasattr(pointcloud, 'data'):
            print("点云数据为空")
            return None
        
        try:
            # 将字节数据转换为numpy数组
            # pointcloud.data 已经是 numpy 数组，不需要调用
            if isinstance(pointcloud.data, np.ndarray):
                data = pointcloud.data.astype(np.uint8)
            else:
                data = np.frombuffer(pointcloud.data, dtype=np.uint8)
            
            # 根据点步长重新整形数据
            if pointcloud.point_step > 0:
                num_points = len(data) // pointcloud.point_step
                data = data[:num_points * pointcloud.point_step]
                data = data.reshape((num_points, pointcloud.point_step))
                
                # 提取常见的字段 (x, y, z, intensity)
                points = []
                for field in pointcloud.fields:
                    if field.name in ['x', 'y', 'z']:
                        field_slice = data[:, field.offset:field.offset+4]
                        field_data = np.ascontiguousarray(field_slice).view(np.float32)
                        points.append(field_data)
                    elif field.name == 'intensity':
                        intensity_slice = data[:, field.offset:field.offset+4]
                        intensity_data = np.ascontiguousarray(intensity_slice).view(np.float32)
                        points.append(intensity_data)
                
                if len(points) >= 3:  # 至少有x, y, z
                    result = np.column_stack(points)
                    print(f"解析出 {result.shape[0]} 个点，{result.shape[1]} 个字段")
                    return result
                else:
                    print("无法解析点云数据：缺少必要的字段")
                    return None
            else:
                print("点步长为0，无法解析数据")
                return None
                
        except Exception as e:
            print(f"解析点云数据失败: {e}")
            return None

    def analyze_pointcloud(self, pointcloud, lidar_name):
        """分析点云数据"""
        print(f"\n--- {lidar_name} 点云分析 ---")
        
        points = self.parse_pointcloud_data(pointcloud)
        if points is not None and len(points) > 0:
            # 基本统计
            print(f"点云统计:")
            print(f"  点数: {len(points)}")
            
            if points.shape[1] >= 3:  # 至少有x, y, z
                x_coords = points[:, 0]
                y_coords = points[:, 1] 
                z_coords = points[:, 2]
                
                print(f"  X范围: [{x_coords.min():.3f}, {x_coords.max():.3f}]")
                print(f"  Y范围: [{y_coords.min():.3f}, {y_coords.max():.3f}]")
                print(f"  Z范围: [{z_coords.min():.3f}, {z_coords.max():.3f}]")
                
                # 计算距离
                distances = np.sqrt(x_coords**2 + y_coords**2 + z_coords**2)
                print(f"  距离范围: [{distances.min():.3f}, {distances.max():.3f}] 米")
                print(f"  平均距离: {distances.mean():.3f} 米")
                
                # 强度统计（如果有的话）
                if points.shape[1] >= 4:
                    intensities = points[:, 3]
                    print(f"  强度范围: [{intensities.min():.3f}, {intensities.max():.3f}]")
                    print(f"  平均强度: {intensities.mean():.3f}")
            
            return points
        else:
            print("无法分析点云数据")
            return None

    def save_pointcloud_to_file(self, pointcloud, filename, lidar_name):
        """保存点云数据到文件"""
        try:
            points = self.parse_pointcloud_data(pointcloud)
            if points is not None:
                # 保存为简单的文本格式
                np.savetxt(filename, points, fmt='%.6f', 
                          header=f"Point cloud from {lidar_name}\nTimestamp: {pointcloud.timestamp_ns}",
                          comments='# ')
                print(f"✅ 点云数据已保存到: {filename}")
                return True
            else:
                print(f"❌ 无法保存点云数据")
                return False
        except Exception as e:
            print(f"❌ 保存点云数据失败: {e}")
            return False

    def continuous_monitoring(self, lidar_type, lidar_name, duration_seconds=30):
        """连续监控激光雷达数据"""
        print(f"\n{'='*60}")
        print(f"连续监控 {lidar_name} ({duration_seconds}秒)")
        print(f"{'='*60}")
        print("按Ctrl+C停止监控")
        
        start_time = time.time()
        count = 0
        fps_sum = 0
        fps_count = 0
        
        try:
            while time.time() - start_time < duration_seconds:
                pointcloud = self.lidar.get_latest_pointcloud(lidar_type, 1000.0)
                
                if pointcloud is not None:
                    count += 1
                    current_time = time.time()
                    elapsed = current_time - start_time
                    
                    if count > 1:
                        fps = 1.0 / (current_time - last_time)
                        fps_sum += fps
                        fps_count += 1
                    
                    last_time = current_time
                    
                    # 显示基本信息
                    print(f"\r{lidar_name} 监控 #{count} - "
                          f"时间戳: {pointcloud.timestamp_ns} - "
                          f"尺寸: {pointcloud.width}x{pointcloud.height} - "
                          f"运行时间: {elapsed:.1f}s", end="")
                    
                    # 每10帧显示一次详细信息
                    if count % 10 == 0:
                        print(f"\n  第{count}帧详细信息:")
                        self.analyze_pointcloud(pointcloud, lidar_name)
                else:
                    print(f"\r{lidar_name} 监控 - 无数据", end="")
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print(f"\n\n监控被用户中断")
        
        # 显示统计信息
        if count > 0:
            avg_fps = fps_sum / fps_count if fps_count > 0 else 0
            print(f"\n\n监控统计:")
            print(f"  总帧数: {count}")
            print(f"  平均帧率: {avg_fps:.2f} FPS")
            print(f"  监控时长: {time.time() - start_time:.1f} 秒")

    def run_comprehensive_test(self):
        """运行综合测试"""
        print("🔍 激光雷达综合测试程序")
        print("=" * 50)
        
        total_success = 0
        total_tests = 0
        available_lidars = []
        
        # 1. 测试所有激光雷达类型
        print("\n1. 测试所有激光雷达类型")
        for lidar_type, lidar_name in self.lidar_types:
            success, tests = self.test_lidar_type(lidar_type, lidar_name)
            total_success += success
            total_tests += tests
            
            if success > 0:
                available_lidars.append((lidar_type, lidar_name))
        
        # 2. 显示总结
        print(f"\n{'='*60}")
        print("测试总结")
        print(f"{'='*60}")
        print(f"总测试次数: {total_tests}")
        print(f"成功次数: {total_success}")
        print(f"成功率: {total_success/total_tests*100:.1f}%")
        
        if available_lidars:
            print(f"\n可用的激光雷达类型:")
            for lidar_type, lidar_name in available_lidars:
                print(f"  ✅ {lidar_name}")
            
            # 3. 获取性能信息
            print(f"\n2. 获取性能信息")
            for lidar_type, lidar_name in available_lidars:
                self.get_lidar_fps(lidar_type, lidar_name)
                self.get_lidar_latency(lidar_type, lidar_name)
            
            # 4. 连续监控演示
            if available_lidars:
                print(f"\n3. 连续监控演示")
                first_lidar_type, first_lidar_name = available_lidars[0]
                self.continuous_monitoring(first_lidar_type, first_lidar_name, 10)
        else:
            print(f"\n❌ 没有检测到可用的激光雷达")

def main():
    """主函数"""
    demo = LidarDemo()
    
    print("选择运行模式:")
    print("1. 综合测试")
    print("2. 只测试特定激光雷达")
    print("3. 连续监控模式")
    
    try:
        choice = input("请输入选择 (1, 2, 或 3): ").strip()
        
        if choice == "2":
            print("\n可用的激光雷达类型:")
            for i, (lidar_type, lidar_name) in enumerate(demo.lidar_types):
                print(f"{i}: {lidar_name}")
            
            lidar_choice = input("请选择激光雷达 (输入数字): ").strip()
            try:
                lidar_index = int(lidar_choice)
                if 0 <= lidar_index < len(demo.lidar_types):
                    lidar_type, lidar_name = demo.lidar_types[lidar_index]
                    demo.test_lidar_type(lidar_type, lidar_name, 5)
                else:
                    print("无效选择，运行综合测试")
                    demo.run_comprehensive_test()
            except ValueError:
                print("无效输入，运行综合测试")
                demo.run_comprehensive_test()
                
        elif choice == "3":
            print("\n可用的激光雷达类型:")
            for i, (lidar_type, lidar_name) in enumerate(demo.lidar_types):
                print(f"{i}: {lidar_name}")
            
            lidar_choice = input("请选择激光雷达 (输入数字): ").strip()
            duration = input("监控时长(秒，默认30): ").strip()
            duration = int(duration) if duration else 30
            
            try:
                lidar_index = int(lidar_choice)
                if 0 <= lidar_index < len(demo.lidar_types):
                    lidar_type, lidar_name = demo.lidar_types[lidar_index]
                    demo.continuous_monitoring(lidar_type, lidar_name, duration)
                else:
                    print("无效选择")
            except ValueError:
                print("无效输入")
        else:
            # 默认运行综合测试
            demo.run_comprehensive_test()
            
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断程序")
    except Exception as e:
        print(f"\n❌ 程序运行出错: {e}")
        print("运行综合测试...")
        demo.run_comprehensive_test()

if __name__ == "__main__":
    main()
