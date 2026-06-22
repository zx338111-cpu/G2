#!/usr/bin/env python3
"""
RackLidarDockingController 调用模板。

用途：
  1. 从命令行只读前激光雷达，确认当前料架距离；
  2. 执行从几米外靠近料架，并在 0.5m 自动停车；
  3. 给主工程提供带注释的调用示例。

机器人端运行前先加载 GDK 环境：
  source /home/agi/app/env.sh

只读前激光雷达，不移动：
  python3 rack_lidar_docking_demo.py --read-only

执行靠近，默认 0.5m 自动停：
  python3 rack_lidar_docking_demo.py --execute --allow-estop-pedal-fault --speed 0.05
"""

import argparse
import time

from rack_lidar_docking import RackLidarDockingController


def parse_args():
    parser = argparse.ArgumentParser()

    # 停车阈值，单位 m。业务默认 0.5m；保留参数只是方便调试分阶段验证。
    parser.add_argument("--stop-m", type=float, default=0.5)

    # 前进速度，单位 m/s。0.03 适合第一次验证；0.05 是正常低速靠近。
    parser.add_argument("--speed", type=float, default=0.05)

    # 最长运行时间。超过时间还没到 stop-m，也会停车并返回 timeout。
    parser.add_argument("--max-duration", type=float, default=90.0)

    # 点云读取和控制频率。前激光雷达点云解析比超声重一些，5Hz 足够。
    parser.add_argument("--hz", type=float, default=5.0)

    # 中位数滤波窗口。3 表示最近 3 帧稳定点簇距离取中位数。
    parser.add_argument("--history", type=int, default=3)

    # 启动前最多等待多少秒收集稳定前方点簇。等待期间不移动。
    parser.add_argument("--initial-lidar-timeout", type=float, default=3.0)

    # 运行中丢失稳定前方点簇时，先停车等待；超过该时间才返回 lost_lidar。
    parser.add_argument("--lost-lidar-timeout", type=float, default=1.0)

    # 最近单点近距离保护阈值。0 表示禁用；当前点云有车体/自反射近点，默认禁用。
    parser.add_argument("--nearest-safety-stop-m", type=float, default=0.0)

    # 点云坐标轴。现场短距离运动确认：底盘前进方向对应 raw +X，横向为 raw Y。
    parser.add_argument("--forward-axis", default="x", choices=("x", "y", "z"))
    parser.add_argument("--lateral-axis", default="y", choices=("x", "y", "z"))
    parser.add_argument("--forward-sign", type=float, default=1.0, choices=(-1.0, 1.0))

    # 前方 ROI 横向半宽，单位 m。0.8 表示只看车头中线左右 0.8m。
    parser.add_argument("--lateral-half-width", type=float, default=0.8)

    # 高度 ROI，单位 m。默认只看料架较高结构，排除地面和车体近点。
    parser.add_argument("--z-min", type=float, default=0.6)
    parser.add_argument("--z-max", type=float, default=1.2)

    # 前向距离 ROI，单位 m。默认看 0.2m 到 6m，覆盖料架靠近范围。
    parser.add_argument("--min-range", type=float, default=0.2)
    parser.add_argument("--max-range", type=float, default=6.0)

    # 点簇分箱。最近一个点数足够的距离箱被认为是料架/前方实体。
    parser.add_argument("--bin-width", type=float, default=0.25)
    parser.add_argument("--min-cluster-points", type=int, default=20)

    # 底盘远控模式。现场验证 mode=0 可用。
    parser.add_argument("--control-mode", type=int, default=0)

    # read-only 模式下读取几次距离。
    parser.add_argument("--samples", type=int, default=10)

    # --read-only：只读点云，不移动。
    # --execute：执行靠近动作，会真的动底盘。
    parser.add_argument("--read-only", action="store_true")
    parser.add_argument("--execute", action="store_true")

    # 当前这台 G2 有官方确认的 emergency_stop_pedal_fault_state=1 已知硬件故障。
    # 只有现场有人看护、确认允许运动时才加这个参数。
    parser.add_argument("--allow-estop-pedal-fault", action="store_true")
    return parser.parse_args()


def make_controller(args):
    return RackLidarDockingController(
        control_mode=args.control_mode,
        forward_axis=args.forward_axis,
        lateral_axis=args.lateral_axis,
        forward_sign=args.forward_sign,
        lateral_half_width_m=args.lateral_half_width,
        z_min_m=args.z_min,
        z_max_m=args.z_max,
        min_range_m=args.min_range,
        max_range_m=args.max_range,
        bin_width_m=args.bin_width,
        min_cluster_points=args.min_cluster_points,
    )


def main():
    args = parse_args()

    # 防止误操作：必须明确选择只读还是执行，不能都不填，也不能两个都填。
    if args.execute == args.read_only:
        raise SystemExit("Choose exactly one: --read-only or --execute")

    with make_controller(args) as controller:
        if args.read_only:
            # 只读模式。用于确认当前 ROI 里能否看到料架，以及距离是否合理。
            for index in range(args.samples):
                distance = controller.read_rack_distance()
                if distance is None:
                    print(f"sample={index + 1} no_stable_cluster")
                else:
                    print(
                        f"sample={index + 1} distance_m={distance.distance_m:.3f} "
                        f"nearest_m={distance.nearest_m:.3f} "
                        f"cluster_points={distance.cluster_points} "
                        f"roi_points={distance.roi_points} "
                        f"bin=[{distance.bin_start_m:.2f},{distance.bin_end_m:.2f})"
                    )
                time.sleep(0.2)
            return

        def print_sample(sample):
            # 每一帧有效点簇都会打印出来，方便现场看距离是否稳定下降。
            # 停车主判断用 filtered_m，同时 nearest_m 作为近距离安全参考。
            print(
                f"t={sample.elapsed_s:.1f}s distance_m={sample.distance_m:.3f} "
                f"filtered_m={sample.filtered_m:.3f} nearest_m={sample.nearest_m:.3f} "
                f"cluster_points={sample.cluster_points} roi_points={sample.roi_points} "
                f"bin=[{sample.bin_start_m:.2f},{sample.bin_end_m:.2f})"
            )

        result = controller.approach_until_distance(
            stop_m=args.stop_m,
            speed_mps=args.speed,
            max_duration_s=args.max_duration,
            hz=args.hz,
            history_size=args.history,
            initial_lidar_timeout_s=args.initial_lidar_timeout,
            lost_lidar_timeout_s=args.lost_lidar_timeout,
            nearest_safety_stop_m=args.nearest_safety_stop_m,
            allow_estop_pedal_fault=args.allow_estop_pedal_fault,
            on_sample=print_sample,
        )
        print(f"result={result}")


if __name__ == "__main__":
    main()
