#!/usr/bin/env python3
"""
RackRadarDockingController 调用模板。

这个 demo 有两个用途：
  1. 命令行直接测试靠近料架；
  2. 给主工程提供一份带详细注释的调用示例。

机器人端运行前先加载 GDK 环境：
  source /home/agi/app/env.sh

只读雷达，不移动：
  python3 rack_radar_docking_demo.py --read-only

执行靠近，默认 500mm 自动停：
  python3 rack_radar_docking_demo.py --execute --allow-estop-pedal-fault --speed 0.05
"""

import argparse
import time

from rack_radar_docking import RackRadarDockingController


def parse_ids(text):
    """把命令行的 '0,1,2,3' 转成 (0, 1, 2, 3)。"""
    return tuple(int(part.strip()) for part in text.split(",") if part.strip())


def parse_args():
    parser = argparse.ArgumentParser()
    # 前方雷达 ID。现场已经确认：0,1,2,3 是机器人前方；4,5,6,7 是后方。
    parser.add_argument("--front-ids", default="0,1,2,3")

    # 停车阈值，单位 mm。500 表示距离料架约 0.5m 自动停车。
    # 业务默认不需要传这个参数；保留 CLI 参数只是方便临时调试。
    parser.add_argument("--stop-mm", type=int, default=500)

    # 前进速度，单位 m/s。
    # 0.02 适合保守调试；0.05 已现场验证，可以作为正常速度。
    parser.add_argument("--speed", type=float, default=0.02)

    # 最长运行时间。超过这个时间还没到 stop-mm，也会主动停车并返回 timeout。
    parser.add_argument("--max-duration", type=float, default=70.0)

    # 控制循环频率。10Hz 已经实机验证稳定。
    parser.add_argument("--hz", type=float, default=10.0)

    # 雷达中位数滤波窗口。3 表示最近 3 帧取中位数，抑制单帧跳变。
    parser.add_argument("--history", type=int, default=3)

    # 启动前最多等待多少秒，收集足够的有效前方雷达数据。
    # 等待期间不移动；少于 --history 帧时不会进入主靠近段。
    parser.add_argument("--initial-radar-timeout", type=float, default=2.0)

    # 如果启动时前方雷达没有稳定回波，approach_to_rack 会用短脉冲搜索回波。
    # 这些参数是内部安全上限，不是业务要求用户输入走多少米。
    parser.add_argument("--acquire-speed", type=float, default=0.03)
    parser.add_argument("--acquire-max-distance", type=float, default=0.6)
    parser.add_argument("--acquire-step-duration", type=float, default=0.4)
    parser.add_argument("--acquire-timeout", type=float, default=20.0)

    # 运行中如果一帧雷达没数据，类会先发零速度并等待恢复；
    # 连续超过这个秒数还没有有效前方雷达，才返回 lost_radar。
    parser.add_argument("--lost-radar-timeout", type=float, default=1.0)

    # 底盘远控模式。官方 move_chassis 示例默认 0，现场验证这个模式可用。
    parser.add_argument("--control-mode", type=int, default=0)

    # read-only 模式下读取几次雷达。
    parser.add_argument("--samples", type=int, default=5)

    # --read-only：只读雷达，不移动。
    # --execute：执行靠近动作，会真的动底盘。
    parser.add_argument("--read-only", action="store_true")
    parser.add_argument("--execute", action="store_true")

    # 当前这台 G2 有官方确认的 emergency_stop_pedal_fault_state=1 已知硬件故障。
    # 只有现场有人看护、确认允许运动时才加这个参数。
    parser.add_argument("--allow-estop-pedal-fault", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()

    # 防止误操作：必须明确选择只读还是执行，不能都不填，也不能两个都填。
    if args.execute == args.read_only:
        raise SystemExit("Choose exactly one: --read-only or --execute")

    front_ids = parse_ids(args.front_ids)

    # 用 with 创建控制器，程序退出时会自动：
    #   1. 发零速度；
    #   2. 取消底盘远控任务；
    #   3. 关闭雷达；
    #   4. release GDK。
    with RackRadarDockingController(
        front_ids=front_ids,
        control_mode=args.control_mode,
    ) as controller:
        if args.read_only:
            # 只读雷达模式。用于确认 front_ids 是否正确、距离是否合理。
            for index in range(args.samples):
                min_mm, distances = controller.read_min_distance()
                print(
                    f"sample={index + 1} front_ids={front_ids} "
                    f"distances={distances} min_mm={min_mm}"
                )
                time.sleep(0.2)
            return

        def print_sample(sample):
            # 每一帧有效雷达都会打印出来，方便现场看距离是否在稳定下降。
            # 真正的停车判断用 filtered_mm，不直接用瞬时 min_mm。
            print(
                f"t={sample.elapsed_s:.1f}s min_mm={sample.min_mm} "
                f"filtered_mm={sample.filtered_mm} distances={sample.distances}"
            )

        # 核心调用就在这里。
        # 如果你的业务固定要求“距离料架 0.5m 停”，主工程推荐直接调用：
        #   controller.approach_to_rack(...)
        # 这里为了 CLI 还能临时改 --stop-mm，继续调用通用方法。
        result = controller.approach_until_distance(
            stop_mm=args.stop_mm,
            speed_mps=args.speed,
            max_duration_s=args.max_duration,
            hz=args.hz,
            history_size=args.history,
            initial_radar_timeout_s=args.initial_radar_timeout,
            acquire_if_needed=True,
            acquire_speed_mps=args.acquire_speed,
            acquire_max_distance_m=args.acquire_max_distance,
            acquire_step_duration_s=args.acquire_step_duration,
            acquire_timeout_s=args.acquire_timeout,
            lost_radar_timeout_s=args.lost_radar_timeout,
            allow_estop_pedal_fault=args.allow_estop_pedal_fault,
            on_sample=print_sample,
        )

        # result.status 常见值：
        #   stopped：正常达到 stop-mm 并停车；
        #   already_at_threshold：一开始已经小于 stop-mm，所以没有移动；
        #   timeout：到 max-duration 还没到阈值；
        #   lost_radar：连续丢失前方雷达超过 lost-radar-timeout。
        #   acquire_timeout：前方雷达搜索到内部上限仍没有锁定目标。
        print(f"result={result}")


if __name__ == "__main__":
    main()
