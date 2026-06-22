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
from rack_retreat_controller import RackRetreatController


def parse_ids(text):
    """把命令行里的 "0,1" 转成 (0, 1)。"""
    return tuple(int(part.strip()) for part in text.split(",") if part.strip())


def parse_args():
    parser = argparse.ArgumentParser()

    # front_ids 是前方超声波 ID。现场已确认 0/1 在车头方向。
    parser.add_argument("--front-ids", default="0,1")

    # rear_ids 是后方超声波 ID。正式后退流程必须持续看后方障碍。
    # 现场已确认 4/5 在车尾方向；2/3 是右侧，6/7 是左侧，不能混入后退保护。
    parser.add_argument("--rear-ids", default="4,5")

    # 粗靠近速度：只在前方超声还看不稳定、需要激光雷达先靠近时使用。
    # 0.60m/s 是现场验证后比较接近工业节拍的速度。
    parser.add_argument("--coarse-speed", type=float, default=0.60)

    # 精停速度：切到前方超声后使用。0.30m/s 速度较快，所以后面配套
    # final_brake_margin_mm 做提前停车补偿。
    parser.add_argument("--final-speed", type=float, default=0.30)

    # final_stop_mm 是“希望最终停稳后的距离”，不是内部触发停车距离。
    # 当前现场按 540mm 目标 + 80mm 补偿，停稳超声读数更接近 0.5m。
    parser.add_argument("--final-stop-mm", type=int, default=540)

    # 制动补偿：0.30m/s 下如果到 500mm 才发停车，实际会滑到约 430mm。
    # 因此默认提前 80mm，内部触发距离 = 500 + 80 = 580mm。
    parser.add_argument("--final-brake-margin-mm", type=int, default=80)

    # 前方超声滤波距离小于该阈值时，从激光粗靠近切到超声精停。
    # 2.2m 是当前现场验证过的常规切换点。
    parser.add_argument("--switch-ultrasonic-mm", type=int, default=2200)

    # 复杂现场稳定策略：如果超声连续几帧稳定地看到 2.5m 内目标，
    # 就优先交给超声精停，避免激光误抓近处非危险结构后 coarse_stopped。
    parser.add_argument("--ultrasonic-takeover-mm", type=int, default=2500)
    parser.add_argument("--ultrasonic-stable-tolerance-mm", type=int, default=250)

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
    parser.add_argument("--retreat", action="store_true")
    parser.add_argument("--cycle", action="store_true")

    # 这个旧 demo 的 --retreat 仍直接调用 RackRetreatController，属于速度开环诊断。
    # 一键工业流程请使用 industrial_7_rods_one_click.py 或 RackIndustrialDockingController，
    # 默认通过 relative_move 做相对位移闭环。
    parser.add_argument("--retreat-distance-m", type=float, default=2.5)
    parser.add_argument("--retreat-speed", type=float, default=0.50)
    parser.add_argument("--rear-stop-mm", type=int, default=700)
    parser.add_argument("--rear-hard-stop-mm", type=int, default=500)
    parser.add_argument("--rear-stop-min-sensors", type=int, default=2)
    parser.add_argument("--retreat-hz", type=float, default=10.0)
    parser.add_argument("--cycle-snapshot-samples", type=int, default=8)

    # 当前机器人 emergency_stop_pedal_fault_state=1 是已知硬件问题。
    # 只有现场确认安全且急停有人看护时，才允许带这个参数运动。
    parser.add_argument("--allow-estop-pedal-fault", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()

    mode_count = sum(
        bool(flag)
        for flag in (args.read_only, args.execute, args.retreat, args.cycle)
    )
    if mode_count != 1:
        raise SystemExit("Choose exactly one: --read-only, --execute, --retreat, or --cycle")

    def print_retreat(sample):
        print(
            f"retreat t={sample.elapsed_s:.1f}s "
            f"estimated_m={sample.estimated_distance_m:.2f} "
            f"rear_min_mm={sample.rear_min_mm} "
            f"rear_filtered_mm={sample.rear_filtered_mm} "
            f"raw={sample.rear_distances}"
        )

    def print_coarse(elapsed_s, distance, filtered_m):
        # 粗靠近回调：每次激光成功识别料架点簇时打印。
        print(
            f"coarse t={elapsed_s:.1f}s lidar_m={distance.distance_m:.3f} "
            f"filtered_m={filtered_m:.3f} cluster_points={distance.cluster_points}"
        )

    def print_final(sample):
        # 精停回调：每次前方超声有有效读数时打印。
        print(
            f"final t={sample.elapsed_s:.1f}s min_mm={sample.min_mm} "
            f"filtered_mm={sample.filtered_mm} raw={sample.distances}"
        )

    def run_read_only(samples):
        with RackHybridDockingController(front_ultrasonic_ids=parse_ids(args.front_ids)) as dock:
            for index in range(samples):
                lidar_distance = dock.lidar.read_rack_distance()
                ultrasonic_min, ultrasonic_raw = dock.ultrasonic.read_min_distance()
                print(
                    f"sample={index + 1} "
                    f"lidar={lidar_distance} "
                    f"ultrasonic_min_mm={ultrasonic_min} "
                    f"ultrasonic_raw={ultrasonic_raw}"
                )
                time.sleep(0.2)

    def run_retreat():
        print(
            f"retreat_target_m={args.retreat_distance_m} "
            f"retreat_speed_mps={args.retreat_speed} "
            f"rear_stop_mm={args.rear_stop_mm} "
            f"rear_hard_stop_mm={args.rear_hard_stop_mm} "
            f"rear_stop_min_sensors={args.rear_stop_min_sensors} "
            f"rear_ids={parse_ids(args.rear_ids)}"
        )
        with RackRetreatController(rear_ultrasonic_ids=parse_ids(args.rear_ids)) as retreat:
            result = retreat.retreat_distance(
                distance_m=args.retreat_distance_m,
                speed_mps=args.retreat_speed,
                rear_stop_mm=args.rear_stop_mm,
                rear_hard_stop_mm=args.rear_hard_stop_mm,
                rear_stop_min_sensors=args.rear_stop_min_sensors,
                hz=args.retreat_hz,
                allow_estop_pedal_fault=args.allow_estop_pedal_fault,
                on_sample=print_retreat,
            )
        print(f"retreat_result={result}")
        return result

    def run_execute():
        with RackHybridDockingController(front_ultrasonic_ids=parse_ids(args.front_ids)) as dock:
            # 明确打印“业务目标”和“内部触发距离”，避免把目标距离与触发距离混淆。
            print(
                f"final_target_mm={args.final_stop_mm} "
                f"final_trigger_mm={args.final_stop_mm + args.final_brake_margin_mm} "
                f"final_brake_margin_mm={args.final_brake_margin_mm} "
                f"switch_ultrasonic_mm={args.switch_ultrasonic_mm} "
                f"ultrasonic_takeover_mm={args.ultrasonic_takeover_mm} "
                f"ultrasonic_stable_tolerance_mm={args.ultrasonic_stable_tolerance_mm}"
            )
            result = dock.approach_to_rack(
                coarse_speed_mps=args.coarse_speed,
                final_speed_mps=args.final_speed,
                final_stop_mm=args.final_stop_mm,
                final_brake_margin_mm=args.final_brake_margin_mm,
                switch_ultrasonic_mm=args.switch_ultrasonic_mm,
                ultrasonic_takeover_mm=args.ultrasonic_takeover_mm,
                ultrasonic_stable_tolerance_mm=args.ultrasonic_stable_tolerance_mm,
                coarse_stop_m=args.coarse_stop_m,
                coarse_max_duration_s=args.coarse_max_duration,
                final_max_duration_s=args.final_max_duration,
                allow_estop_pedal_fault=args.allow_estop_pedal_fault,
                on_coarse_sample=print_coarse,
                on_final_sample=print_final,
            )
        print(f"result={result}")
        return result

    if args.read_only:
        run_read_only(args.samples)
        return

    if args.retreat:
        run_retreat()
        return

    if args.execute:
        run_execute()
        return

    print("cycle_stage=retreat")
    retreat_result = run_retreat()
    if retreat_result.status != "completed":
        print("cycle_result=aborted_after_retreat")
        return

    print("cycle_stage=read_only_snapshot")
    run_read_only(args.cycle_snapshot_samples)

    print("cycle_stage=approach")
    approach_result = run_execute()
    print(f"cycle_result=retreat:{retreat_result.status},approach:{approach_result.status}")


if __name__ == "__main__":
    main()
