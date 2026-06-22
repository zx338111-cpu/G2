#!/usr/bin/env python3
"""
RackHybridDockingController 调用模板。

这个文件是给现场测试/业务集成看的最小可运行入口：
  - 想先确认传感器是否看到料架，用 --read-only；
  - 想让机器人实际靠近料架，用 --execute；
  - 当前这台 G2 有已知急停踏板故障，需要现场确认安全后加
    --allow-estop-pedal-fault。

只读查看当前两类传感器：
  python3 rack_hybrid_docking_demo.py --read-only

执行两段式靠近并在 0.5m 停车：
  python3 rack_hybrid_docking_demo.py --execute --allow-estop-pedal-fault
"""

import argparse
import time

from rack_hybrid_docking import RackHybridDockingController


def parse_ids(text):
    """把命令行里的 "0,1,2,3" 转成 (0, 1, 2, 3)。"""
    return tuple(int(part.strip()) for part in text.split(",") if part.strip())


def parse_args():
    parser = argparse.ArgumentParser()

    # front_ids 是前方超声波 ID。现场确认 0/1/2/3 在车头方向，
    # 其中 0/1 对料架回波最稳定，代码仍保留 0~3 作为冗余。
    parser.add_argument("--front-ids", default="0,1,2,3")

    # 粗靠近速度：只在前方超声还看不稳定、需要激光雷达先靠近时使用。
    # 0.60m/s 是现场验证后比较接近工业节拍的速度。
    parser.add_argument("--coarse-speed", type=float, default=0.60)

    # 精停速度：切到前方超声后使用。0.30m/s 速度较快，所以后面配套
    # final_brake_margin_mm 做提前停车补偿。
    parser.add_argument("--final-speed", type=float, default=0.30)

    # final_stop_mm 是“希望最终停稳后的距离”，不是内部触发停车距离。
    # 当前目标是料架前 0.5m，所以默认 500。
    parser.add_argument("--final-stop-mm", type=int, default=500)

    # 制动补偿：0.30m/s 下如果到 500mm 才发停车，实际会滑到约 430mm。
    # 因此默认提前 80mm，内部触发距离 = 500 + 80 = 580mm。
    parser.add_argument("--final-brake-margin-mm", type=int, default=80)

    # 前方超声滤波距离小于该阈值时，从激光粗靠近切到超声精停。
    # 1.8m 是实机验证较稳的切换点。
    parser.add_argument("--switch-ultrasonic-mm", type=int, default=1800)

    # 激光粗靠近保护下限：如果已经到 1.6m 还没有稳定超声，就停止，
    # 防止激光点云追到背景后继续往前开。
    parser.add_argument("--coarse-stop-m", type=float, default=1.6)

    # 两段最大运行时间用于兜底，避免料架不在前方时无限运动。
    parser.add_argument("--coarse-max-duration", type=float, default=90.0)
    parser.add_argument("--final-max-duration", type=float, default=60.0)

    # 只读采样次数。只读模式不会发底盘速度，适合先检查现场传感器。
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--read-only", action="store_true")
    parser.add_argument("--execute", action="store_true")

    # 当前机器人 emergency_stop_pedal_fault_state=1 是已知硬件问题。
    # 只有现场确认安全且急停有人看护时，才允许带这个参数运动。
    parser.add_argument("--allow-estop-pedal-fault", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()

    # read-only 和 execute 必须二选一，避免用户忘记模式后误触发运动。
    if args.execute == args.read_only:
        raise SystemExit("Choose exactly one: --read-only or --execute")

    # 用 with 管理 GDK 生命周期。退出时主类会自动发零速度、关闭传感器并释放 GDK。
    with RackHybridDockingController(front_ultrasonic_ids=parse_ids(args.front_ids)) as dock:
        if args.read_only:
            # 只读模式：每 0.2s 打印一帧激光估距和前方超声原始读数。
            # 这一步不会占用底盘控制权，也不会让机器人动。
            for index in range(args.samples):
                lidar_distance = dock.lidar.read_rack_distance()
                ultrasonic_min, ultrasonic_raw = dock.ultrasonic.read_min_distance()
                print(
                    f"sample={index + 1} "
                    f"lidar={lidar_distance} "
                    f"ultrasonic_min_mm={ultrasonic_min} "
                    f"ultrasonic_raw={ultrasonic_raw}"
                )
                time.sleep(0.2)
            return

        def print_coarse(elapsed_s, distance, filtered_m):
            # 粗靠近回调：每次激光成功识别料架点簇时打印。
            # cluster_points 越大，说明该距离箱里的点云越稳定。
            print(
                f"coarse t={elapsed_s:.1f}s lidar_m={distance.distance_m:.3f} "
                f"filtered_m={filtered_m:.3f} cluster_points={distance.cluster_points}"
            )

        def print_final(sample):
            # 精停回调：每次前方超声有有效读数时打印。
            # min_mm 是本帧最小原始距离，filtered_mm 是最近几帧中位数。
            print(
                f"final t={sample.elapsed_s:.1f}s min_mm={sample.min_mm} "
                f"filtered_mm={sample.filtered_mm} raw={sample.distances}"
            )

        # 明确打印“业务目标”和“内部触发距离”，避免把 500mm 与 580mm 混淆。
        print(
            f"final_target_mm={args.final_stop_mm} "
            f"final_trigger_mm={args.final_stop_mm + args.final_brake_margin_mm} "
            f"final_brake_margin_mm={args.final_brake_margin_mm}"
        )

        # 所有命令行参数最终都传给主业务类。主业务类内部会自动决定：
        # 启动时能看到超声 -> 直接精停；看不到超声 -> 先激光粗靠近再切超声。
        result = dock.approach_to_rack(
            coarse_speed_mps=args.coarse_speed,
            final_speed_mps=args.final_speed,
            final_stop_mm=args.final_stop_mm,
            final_brake_margin_mm=args.final_brake_margin_mm,
            switch_ultrasonic_mm=args.switch_ultrasonic_mm,
            coarse_stop_m=args.coarse_stop_m,
            coarse_max_duration_s=args.coarse_max_duration,
            final_max_duration_s=args.final_max_duration,
            allow_estop_pedal_fault=args.allow_estop_pedal_fault,
            on_coarse_sample=print_coarse,
            on_final_sample=print_final,
        )
        print(f"result={result}")


if __name__ == "__main__":
    main()
