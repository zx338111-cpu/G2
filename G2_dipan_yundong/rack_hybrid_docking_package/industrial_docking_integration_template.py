#!/usr/bin/env python3
"""
G2 料架靠近功能：业务集成模板。

这个文件的用途：
  这个文件不是底层算法，也不是必须长期运行的服务。
  它是给你把“料架靠近/精停/后退”接进自己业务代码时参考的模板。

你真正接入自己的代码时，只需要复制下面几块：
  1. import RackIndustrialDockingController
  2. check_robot_ready()
  3. dock_to_rack()
  4. retreat_from_rack()
  5. 在你的业务主流程里按 do_one_loading_rack_workflow() 的顺序调用

机器人端运行前必须先加载 GDK 环境：
  cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1/rack_hybrid_docking_package
  source /home/agi/app/env.sh

重要安全说明：
  - read_snapshot() / preflight() 不会让机器人运动。
  - dock_to_rack() 会让机器人实际向前靠近料架。
  - retreat_from_rack() 会让机器人实际后退。
  - 直接运行本文件会执行一次完整“靠近 -> 你的业务 -> 后退”流程。
  - 如果你只想看当前距离，请运行：
      python3 use_industrial_docking_methods.py --mode read --samples 10

当前现场实测结论：
  - 连续 5 轮默认精停成功率 5/5。
  - final_stop_mm=540、final_brake_margin_mm=80 时，
    内部触发距离为 620mm。
  - 停稳后前超声最小值约 512~531mm，平均约 522.6mm。
  - 如果你希望停得更远一点，可以先把 final_stop_mm 调到 555 再复测。
"""

from rack_industrial_docking import RackIndustrialDockingController


# =============================================================================
# 1. 全局安全开关
# =============================================================================

# 当前这台 G2 有一个已知状态：
#   emergency_stop_pedal_fault_state=1
#
# 这个状态在之前现场测试里已经确认是急停踏板故障位，不是当前有人按下急停。
# 所以在现场确认安全、有人看护时，可以把 ALLOW_ESTOP_PEDAL_FAULT 设为 True。
#
# 这个开关只放行“已知急停踏板故障位”。
# 代码不会放行以下真正危险/阻断状态：
#   - charge_plug_insert_state=1：充电插头/充电状态未解除；
#   - motion_control_error != 0：运动控制层有错误；
#   - chassis_ultrasonic_radar_power_state != 1：超声供电异常；
#   - 其他 GDK/PNC 调用失败。
ALLOW_ESTOP_PEDAL_FAULT = True


# =============================================================================
# 2. 业务前置检查：只读，不运动
# =============================================================================

def check_robot_ready(rack):
    """
    检查机器人当前是否允许做底盘运动。

    参数：
      rack:
        RackIndustrialDockingController 对象。
        这个对象内部已经创建好了激光、前超声、后超声和 PNC 控制器。

    会不会运动：
      不会。这个函数只读取底盘安全状态。

    什么时候调用：
      每次开始靠近料架前都建议先调用。
      如果这个函数抛异常，后面不要继续调用 dock_to_rack() 或 retreat_from_rack()。

    返回：
      IndustrialStageResult。
      正常时 result.status == "ok"。

    常见失败：
      result.status == "blocked"：
        说明充电状态、运动错误、急停状态、超声供电等不满足运动条件。
    """
    result = rack.preflight(
        allow_estop_pedal_fault=ALLOW_ESTOP_PEDAL_FAULT,
    )
    print("preflight_result=", result)

    if result.status != "ok":
        # 这里直接 raise，让上层业务停止。
        # 如果你的业务有自己的错误码系统，可以在这里改成返回业务错误码。
        raise RuntimeError(f"Robot is not ready for chassis motion: {result}")

    return result


def read_rack_sensors(rack):
    """
    读取当前料架相关传感器。

    会不会运动：
      不会。这个函数只读传感器。

    主要字段：
      snapshot.front_min_mm:
        前方超声 ID 0/1 中的最小有效距离，单位 mm。
        这个值越小，说明机器人离前方料架/障碍越近。

      snapshot.front_raw:
        前方超声原始读数，例如 ((0, 530), (1, 531))。
        如果某个 ID 没出现，通常表示本帧无有效回波或 fault_state 不为 0。

      snapshot.rear_min_mm:
        后方超声 ID 4/5 中的最小有效距离，单位 mm。
        后退前建议看这个值，太小就不要后退。

      snapshot.rear_raw:
        后方超声原始读数。

      snapshot.lidar_distance_m:
        前激光雷达在 ROI 中估计到的料架距离，单位 m。
        如果为 None，表示本帧点云没有稳定点簇。

    什么时候调用：
      - 运动前想确认现场状态；
      - 调试时想看当前距离；
      - 业务日志里想记录动作前/后的距离证据。
    """
    snapshot = rack.read_snapshot()
    print("sensor_snapshot=", snapshot)
    return snapshot


# =============================================================================
# 3. 靠近料架：粗定位 + 精定位，会实际向前运动
# =============================================================================

def dock_to_rack(rack):
    """
    靠近料架，并停在料架前目标距离。

    这是你最应该接入业务系统的“靠近料架”函数。

    会不会运动：
      会。这个函数可能会让机器人向前移动。

    内部流程：
      第一步：coarse_position()
        目的：
          把机器人从较远位置带到“前方超声能稳定接管”的区域。

        当前现场常见情况：
          因为机器人后退后前方超声通常已经能看到 1.8~2.0m 左右的料架面，
          coarse_position() 会很快返回 ready_for_fine，不一定真的让激光粗走。
          这不是问题，说明超声已经足够稳定，可以直接精停。

        成功条件：
          coarse_result.status == "ready_for_fine"

        失败时不能继续精停：
          如果粗定位失败还继续精停，可能代表前超声不稳定或者激光目标异常，
          工业现场里应该停下来排查，而不是盲目前进。

      第二步：fine_position()
        目的：
          用前方超声闭环停车。

        当前参数：
          final_stop_mm=540：
            业务目标距离。意思是希望机器人最终停稳在料架前约 540mm。

          final_brake_margin_mm=80：
            制动补偿。因为机器人在 0.30m/s 下发零速度后还会滑一点，
            所以实际内部触发距离 = 540 + 80 = 620mm。

          final_speed_mps=0.30：
            精停段速度。这个速度已经做过连续 5 轮测试。

        成功条件：
          fine_result.status in ("stopped", "already_at_threshold")

        stopped：
          正常精停成功。

        already_at_threshold：
          调用时已经在触发距离以内，所以代码不会继续向前开。
          这也算安全成功。

    返回：
      fine_result。
      你自己的业务通常只需要拿 fine_result.status 判断是否可以开始上料/下料。
    """

    # ---------------------------
    # 第一步：粗定位
    # ---------------------------
    coarse_result = rack.coarse_position(
        # 激光粗靠近速度。只有当前超声还不稳定、需要激光先靠近时才会用到。
        coarse_speed_mps=0.60,

        # 激光粗定位保护下限。
        # 如果激光已经估计到 1.6m 内，但前超声还不稳定，就返回 coarse_guard。
        # 这时不能继续盲走。
        coarse_stop_m=1.6,

        # 常规超声切换阈值。
        # 前超声滤波距离进入 2200mm 内时，认为可以切到精停段。
        switch_ultrasonic_mm=2200,

        # 复杂现场的稳定超声优先接管上限。
        # 只要超声连续稳定并小于 2500mm，就让超声接管，避免激光误抓现场近点。
        ultrasonic_takeover_mm=2500,

        # 放行已知急停踏板故障位。
        allow_estop_pedal_fault=ALLOW_ESTOP_PEDAL_FAULT,
    )
    print("coarse_result=", coarse_result)

    if coarse_result.status != "ready_for_fine":
        # 这里必须中断。
        #
        # 常见状态说明：
        #   coarse_guard：
        #     激光已经到保护下限，但前超声还不稳定。
        #     处理：检查前方超声 ID、料架角度、遮挡、现场反光。
        #
        #   lost_lidar：
        #     粗定位时连续丢失激光点簇。
        #     处理：检查料架是否在正前方、激光 ROI 是否合适。
        #
        #   target_lost：
        #     激光目标突然跳远，通常是追到背景。
        #     处理：不要继续走，重新摆正机器人或检查现场结构。
        #
        #   timeout：
        #     粗定位超过最大时间。
        #     处理：检查目标是否在正前方。
        #
        #   blocked：
        #     底盘安全状态不允许运动。
        #     处理：看 result.message 里的具体阻断原因。
        raise RuntimeError(f"Coarse positioning failed: {coarse_result}")

    # ---------------------------
    # 第二步：精定位
    # ---------------------------
    fine_result = rack.fine_position(
        # 希望最终停稳后的业务距离。
        # 当前连续 5 轮测试停稳在 512~531mm，平均 522.6mm。
        final_stop_mm=540,

        # 制动补偿。
        # 内部触发距离 = final_stop_mm + final_brake_margin_mm = 620mm。
        final_brake_margin_mm=80,

        # 精停速度。速度越快，制动补偿越重要。
        final_speed_mps=0.30,

        allow_estop_pedal_fault=ALLOW_ESTOP_PEDAL_FAULT,
    )
    print("fine_result=", fine_result)

    if fine_result.status not in ("stopped", "already_at_threshold"):
        # 失败状态说明：
        #   lost_radar：
        #     精停过程中前方超声连续丢失。
        #
        #   no_front_ultrasonic_lock：
        #     启动精停前没有稳定超声回波。
        #
        #   timeout：
        #     精停超过最大时间。
        #
        #   blocked：
        #     运动前安全检查失败。
        #
        #   error：
        #     GDK/PNC 调用异常。
        #
        # 遇到这些状态，上层业务不应该执行上料/下料动作。
        raise RuntimeError(f"Fine positioning failed: {fine_result}")

    return fine_result


# =============================================================================
# 4. 后退离开料架，会实际后退
# =============================================================================

def retreat_from_rack(rack, distance_m=1.0):
    """
    从料架前后退。

    会不会运动：
      会。这个函数会让机器人向后退。

    为什么后退距离默认 1.0m：
      当前七根料工艺要求抓取后/放料后都实实在在后退 1m，后续动作基于
      这个位移设计。

    后退控制方式：
      默认使用 Pnc.relative_move(x=-distance_m) 做相对位移闭环。
      速度开环只允许作为诊断/应急，不作为正式工业流程距离基准。

    后方保护：
      后退过程中会持续读取后方超声 4/5。

      rear_hard_stop_mm=500：
        任一后方原始读数小于等于 500mm，立即停车。

      rear_stop_mm=700：
        稳定障碍停车阈值。

    成功条件：
      retreat_result.status == "completed"
    """
    retreat_result = rack.retreat(
        distance_m=distance_m,
        speed_mps=0.50,
        method="relative",
        rear_stop_mm=700,
        rear_hard_stop_mm=500,
        allow_estop_pedal_fault=ALLOW_ESTOP_PEDAL_FAULT,
    )
    print("retreat_result=", retreat_result)

    if retreat_result.status != "completed":
        # 失败状态说明：
        #   rear_obstacle：
        #     后方超声检测到障碍，代码已经停车。
        #
        #   timeout：
        #     后退超时，代码已经停车。
        #
        #   blocked：
        #     底盘安全状态不允许运动。
        #
        #   error：
        #     GDK/PNC 调用异常。
        #
        # 遇到 rear_obstacle 时，不要继续发后退命令，应先处理后方障碍。
        raise RuntimeError(f"Retreat failed: {retreat_result}")

    return retreat_result


# =============================================================================
# 5. 一个完整业务流程示例
# =============================================================================

def do_one_loading_rack_workflow():
    """
    一个完整业务流程。

    你自己的业务代码可以按这个顺序接：

      1. 创建 RackIndustrialDockingController。
         用 with 语句是为了保证异常退出时也会 close()，底层会尽量发零速度。

      2. check_robot_ready(rack)。
         运动前检查。如果这里失败，后面不要继续。

      3. read_rack_sensors(rack)。
         记录动作前传感器状态，便于现场排查。

      4. dock_to_rack(rack)。
         真正靠近料架并精停。

      5. 执行你自己的上料/下料逻辑。
         例如机械臂动作、夹爪动作、扫码、等待 PLC 信号等。

      6. retreat_from_rack(rack)。
         业务做完后后退，离开料架。

    返回：
      (fine_result, retreat_result)
    """
    with RackIndustrialDockingController(
        # 前方超声 ID。当前现场已确认 0/1 是机器人前方。
        front_ultrasonic_ids=(0, 1),

        # 后方超声 ID。当前现场已确认 4/5 是机器人后方。
        rear_ultrasonic_ids=(4, 5),
    ) as rack:
        check_robot_ready(rack)
        read_rack_sensors(rack)

        # 到这里会实际靠近料架。
        fine_result = dock_to_rack(rack)

        # ---------------------------------------------------------------------
        # 在这里接入你的业务动作。
        #
        # 例子：
        #   run_loading_task()
        #   run_unloading_task()
        #   wait_plc_signal()
        #   control_arm_to_pick_or_place()
        #
        # 注意：
        #   如果你的业务动作失败，也建议进入 finally 或异常处理里调用
        #   retreat_from_rack()，让机器人离开料架。
        # ---------------------------------------------------------------------
        print("rack is reached, run your loading/unloading task here")

        # 业务完成后后退离开料架。
        retreat_result = retreat_from_rack(rack, distance_m=1.4)

        return fine_result, retreat_result


if __name__ == "__main__":
    # 直接运行本文件会让机器人实际执行：
    #   安全检查 -> 读传感器 -> 靠近料架 -> 打印业务占位 -> 后退
    #
    # 如果你只是想看当前传感器距离，不要运行本文件，运行：
    #   python3 use_industrial_docking_methods.py --mode read --samples 10
    #
    # 如果你只想先检查安全状态，运行：
    #   python3 use_industrial_docking_methods.py --mode preflight --allow-estop-pedal-fault
    do_one_loading_rack_workflow()
