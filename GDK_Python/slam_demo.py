#!/usr/bin/env python3
"""
SLAM (Simultaneous Localization and Mapping) 接口使用演示程序
展示如何使用agibot_gdk进行SLAM建图、定位和导航
"""

import time
import agibot_gdk

class SlamDemo:
    def __init__(self):
        """初始化SLAM相关对象"""
        print("正在初始化SLAM相关对象...")
        
        # 初始化SLAM对象
        self.slam = agibot_gdk.Slam()
        time.sleep(2)
        
        # 初始化PNC (Planning and Control) 对象
        self.pnc = agibot_gdk.Pnc()
        time.sleep(2)
        
        # 初始化Map对象
        self.map = agibot_gdk.Map()
        time.sleep(2)
        
        print("SLAM相关对象初始化完成！")

    def test_slam_basic_functions(self):
        """测试SLAM基本功能"""
        print("\n=== 测试SLAM基本功能 ===")
        
        try:
            # 1. 获取SLAM状态
            print("1. 获取SLAM状态...")
            slam_state = self.slam.get_slam_state()
            print(f"   SLAM状态: {slam_state}")
            
            # 2. 获取里程计信息
            print("2. 获取里程计信息...")
            odom_info = self.slam.get_odom_info()
            print(f"   位置: ({odom_info.pose.pose.position.x:.3f}, "
                  f"{odom_info.pose.pose.position.y:.3f}, "
                  f"{odom_info.pose.pose.position.z:.3f}) m")
            print(f"   姿态(四元数): ({odom_info.pose.pose.orientation.x:.3f}, "
                  f"{odom_info.pose.pose.orientation.y:.3f}, "
                  f"{odom_info.pose.pose.orientation.z:.3f}, "
                  f"{odom_info.pose.pose.orientation.w:.3f})")
            print(f"   姿态(欧拉角): ({odom_info.orientation_euler.x:.3f}, "
                  f"{odom_info.orientation_euler.y:.3f}, "
                  f"{odom_info.orientation_euler.z:.3f}) rad")
            print(f"   线速度: ({odom_info.twist.twist.linear.x:.3f}, "
                  f"{odom_info.twist.twist.linear.y:.3f}, "
                  f"{odom_info.twist.twist.linear.z:.3f}) m/s")
            print(f"   角速度: ({odom_info.twist.twist.angular.x:.3f}, "
                  f"{odom_info.twist.twist.angular.y:.3f}, "
                  f"{odom_info.twist.twist.angular.z:.3f}) rad/s")
            print(f"   速度(世界系): ({odom_info.velocity.x:.3f}, "
                  f"{odom_info.velocity.y:.3f}, "
                  f"{odom_info.velocity.z:.3f}) m/s")
            print(f"   速度(本体系): ({odom_info.velocity_body.x:.3f}, "
                  f"{odom_info.velocity_body.y:.3f}, "
                  f"{odom_info.velocity_body.z:.3f}) m/s")
            print(f"   加速度: ({odom_info.acceleration.x:.3f}, "
                  f"{odom_info.acceleration.y:.3f}, "
                  f"{odom_info.acceleration.z:.3f}) m/s²")
            print(f"   角速度: ({odom_info.ang_vel.x:.3f}, "
                  f"{odom_info.ang_vel.y:.3f}, "
                  f"{odom_info.ang_vel.z:.3f}) rad/s")
            print(f"   定位状态: {odom_info.loc_state}, "
                  f"置信度: {odom_info.loc_confidence:.3f}")
            print(f"   是否静止: {odom_info.is_stationary}, "
                  f"是否打滑: {odom_info.is_sliping}")
            
            return True
        except Exception as e:
            print(f"   ❌ SLAM基本功能测试失败: {e}")
            return False

    def test_mapping_functions(self):
        """测试建图功能"""
        print("\n=== 测试建图功能 ===")
        
        try:
            # 1. 开始建图
            print("1. 开始建图...")
            self.slam.start_mapping()
            print("   ✅ 建图已开始")
            
            # 等待一段时间
            print("2. 等待建图进行...")
            time.sleep(3)
            
            # 2. 获取建图状态
            print("3. 获取建图状态...")
            slam_state = self.slam.get_slam_state()
            print(f"   当前SLAM状态: {slam_state}")
            
            # 3. 停止建图
            print("4. 停止建图...")
            self.slam.stop_mapping()
            print("   ✅ 建图已停止")
            
            return True
        except Exception as e:
            print(f"   ❌ 建图功能测试失败: {e}")
            return False

    def test_map_management(self):
        """测试地图管理功能"""
        print("\n=== 测试地图管理功能 ===")
        
        try:
            # 1. 获取所有地图
            print("1. 获取所有地图...")
            all_maps = self.map.get_all_map()
            print(f"   所有地图: {all_maps}")
            
            # 2. 获取当前地图
            print("2. 获取当前地图...")
            current_map = self.map.get_curr_map()
            print(f"   当前地图: {current_map}")
            
            # 3. 如果有地图，测试获取地图信息
            if all_maps:
                print("3. 获取地图详细信息...")
                for map_info in all_maps:
                    map_id = map_info.id
                    print(f"   地图ID: {map_id}, 名称: {map_info.name}")
                    
                    # 获取地图数据
                    try:
                        map_data = self.map.get_map(map_id)
                        print(f"   地图数据: {map_data}")
                    except Exception as e:
                        print(f"   获取地图{map_id}数据失败: {e}")
            
            return True
        except Exception as e:
            print(f"   ❌ 地图管理功能测试失败: {e}")
            return False

    def test_navigation_functions(self):
        """测试导航功能"""
        print("\n=== 测试导航功能 ===")
        
        try:
            # 1. 获取任务状态
            print("1. 获取当前任务状态...")
            task_state = self.pnc.get_task_state()
            print(f"   任务状态: {task_state}")
            
            # 2. 创建导航请求
            print("2. 创建导航请求...")
            navi_req = agibot_gdk.NaviReq()
            
            # 设置目标位置 (示例坐标)
            navi_req.target.position.x = 0.1
            navi_req.target.position.y = 0.0
            navi_req.target.position.z = 0.0
            
            # 设置目标姿态 (示例四元数)
            navi_req.target.orientation.x = 0.0
            navi_req.target.orientation.y = 0.0
            navi_req.target.orientation.z = 0.0
            navi_req.target.orientation.w = 0.0
            
            print("   ✅ 导航请求已创建")
            
            # 3. 测试相对移动
            print("3. 测试相对移动...")
            try:
                self.pnc.relative_move(navi_req)#相对位移
                #self.pnc.normal_navi(navi_req)#普通导航
                print("   ✅ 相对移动命令已发送")
            except Exception as e:
                print(f"   ⚠️ 相对移动失败: {e}")
            
            # 4. 测试任务控制
            print("4. 测试任务控制...")
            try:
                # 获取当前任务状态以获取任务ID
                task_state = self.pnc.get_task_state()
                task_id = task_state.id
                print(f"   当前任务ID: {task_id}, 状态: {task_state.state}, 类型: {task_state.type}")
                
                # 暂停任务
                self.pnc.pause_task(task_id)
                print(f"   ✅ 任务 {task_id} 已暂停")
                
                time.sleep(1)
                
                # 恢复任务
                self.pnc.resume_task(task_id)
                print(f"   ✅ 任务 {task_id} 已恢复")
                
                time.sleep(1)
                
                # 取消任务
                self.pnc.cancel_task(task_id)
                print(f"   ✅ 任务 {task_id} 已取消")
                
            except Exception as e:
                print(f"   ⚠️ 任务控制失败: {e}")
            
            return True
        except Exception as e:
            print(f"   ❌ 导航功能测试失败: {e}")
            return False

    def continuous_monitoring(self, duration_seconds=30):
        """连续监控SLAM状态"""
        print(f"\n=== 连续监控SLAM状态 ({duration_seconds}秒) ===")
        print("按Ctrl+C停止监控")
        
        start_time = time.time()
        count = 0
        
        try:
            while time.time() - start_time < duration_seconds:
                count += 1
                current_time = time.time()
                elapsed = current_time - start_time
                
                # 获取SLAM状态
                try:
                    slam_state = self.slam.get_slam_state()
                    odom_info = self.slam.get_odom_info()
                    
                    print(f"\r监控 #{count} - "
                          f"SLAM状态: {slam_state} - "
                          f"位置: ({odom_info.pose.pose.position.x:.2f}, "
                          f"{odom_info.pose.pose.position.y:.2f}, "
                          f"{odom_info.pose.pose.position.z:.2f}) - "
                          f"速度: ({odom_info.velocity.x:.2f}, "
                          f"{odom_info.velocity.y:.2f}) - "
                          f"定位状态: {odom_info.loc_state} - "
                          f"运行时间: {elapsed:.1f}s", end="")
                    
                except Exception as e:
                    print(f"\r监控 #{count} - 获取状态失败: {e} - 运行时间: {elapsed:.1f}s", end="")
                
                time.sleep(1.0)
                
        except KeyboardInterrupt:
            print(f"\n\n监控被用户中断")
        
        print(f"\n\n监控统计:")
        print(f"  总监控次数: {count}")
        print(f"  监控时长: {time.time() - start_time:.1f} 秒")

    def run_comprehensive_test(self):
        """运行综合测试"""
        print("🗺️ SLAM综合测试程序")
        print("=" * 50)
        
        test_results = {}
        
        # 1. 测试SLAM基本功能
        print("\n1. 测试SLAM基本功能")
        test_results['slam_basic'] = self.test_slam_basic_functions()
        
        # 2. 测试建图功能
        print("\n2. 测试建图功能")
        test_results['mapping'] = self.test_mapping_functions()
        
        # 3. 测试地图管理功能
        print("\n3. 测试地图管理功能")
        test_results['map_management'] = self.test_map_management()
        
        # 4. 测试导航功能
        print("\n4. 测试导航功能")
        test_results['navigation'] = self.test_navigation_functions()
        
        # 5. 显示测试总结
        print(f"\n{'='*60}")
        print("测试总结")
        print(f"{'='*60}")
        
        total_tests = len(test_results)
        passed_tests = sum(1 for result in test_results.values() if result)
        
        for test_name, result in test_results.items():
            status = "✅ 通过" if result else "❌ 失败"
            print(f"{test_name}: {status}")
        
        print(f"\n总体结果: {passed_tests}/{total_tests} 测试通过")
        print(f"成功率: {passed_tests/total_tests*100:.1f}%")

def main():
    """主函数"""
    demo = SlamDemo()
    
    print("选择运行模式:")
    print("1. 综合测试")
    print("2. 只测试SLAM基本功能")
    print("3. 只测试建图功能")
    print("4. 只测试地图管理功能")
    print("5. 只测试导航功能")
    print("6. 只测试重定位功能")
    print("7. 连续监控模式")
    
    try:
        choice = input("请输入选择 (1-7): ").strip()
        
        if choice == "2":
            demo.test_slam_basic_functions()
        elif choice == "3":
            demo.test_mapping_functions()
        elif choice == "4":
            demo.test_map_management()
        elif choice == "5":
            demo.test_navigation_functions()
        elif choice == "6":
            duration = input("监控时长(秒，默认300): ").strip()
            duration = int(duration) if duration else 300
            demo.continuous_monitoring(duration)
        else:
            # 默认运行综合测试
            demo.run_comprehensive_test()
            
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断程序")
    except Exception as e:
        print(f"\n❌ 程序运行出错: {e}")
        print("运行综合测试...")
        demo.run_comprehensive_test()
    finally:
        print("\n程序结束")

if __name__ == "__main__":
    main()
