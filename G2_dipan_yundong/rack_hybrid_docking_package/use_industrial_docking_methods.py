#!/usr/bin/env python3
"""
RackIndustrialDockingController usage example.

Run this file on the robot after loading the GDK environment:

    cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1/rack_hybrid_docking_package
    source /home/agi/app/env.sh

Read sensors only:

    python3 use_industrial_docking_methods.py --mode read --samples 10

Run coarse + fine approach:

    python3 use_industrial_docking_methods.py --mode approach --allow-estop-pedal-fault

Run a full cycle:

    python3 use_industrial_docking_methods.py --mode cycle --allow-estop-pedal-fault
"""

import argparse
import time

from rack_industrial_docking import RackIndustrialDockingController


def parse_ids(text):
    return tuple(int(part.strip()) for part in text.split(",") if part.strip())


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        required=True,
        choices=(
            "preflight",
            "read",
            "forward",
            "coarse",
            "fine",
            "retreat",
            "approach",
            "cycle",
        ),
    )
    parser.add_argument("--front-ids", default="0,1")
    parser.add_argument("--rear-ids", default="4,5")
    parser.add_argument("--allow-estop-pedal-fault", action="store_true")

    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--sample-interval-s", type=float, default=0.2)

    parser.add_argument("--forward-distance-m", type=float, default=0.2)
    parser.add_argument("--forward-speed-mps", type=float, default=0.05)
    parser.add_argument("--front-hard-stop-mm", type=int, default=700)

    parser.add_argument("--coarse-speed-mps", type=float, default=0.60)
    parser.add_argument("--coarse-stop-m", type=float, default=1.6)
    parser.add_argument("--switch-ultrasonic-mm", type=int, default=2200)
    parser.add_argument("--ultrasonic-takeover-mm", type=int, default=2500)

    parser.add_argument("--final-stop-mm", type=int, default=540)
    parser.add_argument("--final-brake-margin-mm", type=int, default=80)
    parser.add_argument("--final-speed-mps", type=float, default=0.30)

    parser.add_argument("--retreat-distance-m", type=float, default=1.0)
    parser.add_argument("--retreat-speed-mps", type=float, default=0.50)
    parser.add_argument(
        "--retreat-method",
        choices=("relative", "velocity"),
        default="relative",
    )
    parser.add_argument("--rear-stop-mm", type=int, default=700)
    parser.add_argument("--rear-hard-stop-mm", type=int, default=500)
    return parser.parse_args()


def print_snapshot(snapshot):
    print(
        "snapshot "
        f"lidar_m={snapshot.lidar_distance_m} "
        f"front_min_mm={snapshot.front_min_mm} "
        f"front_raw={snapshot.front_raw} "
        f"rear_min_mm={snapshot.rear_min_mm} "
        f"rear_raw={snapshot.rear_raw}"
    )


def print_fine_sample(sample):
    print(
        f"fine t={sample.elapsed_s:.2f}s "
        f"min_mm={sample.min_mm} "
        f"filtered_mm={sample.filtered_mm} "
        f"raw={sample.distances}"
    )


def print_retreat_sample(sample):
    print(
        f"retreat t={sample.elapsed_s:.2f}s "
        f"estimated_m={sample.estimated_distance_m:.2f} "
        f"rear_min_mm={sample.rear_min_mm} "
        f"rear_filtered_mm={sample.rear_filtered_mm} "
        f"raw={sample.rear_distances}"
    )


def main():
    args = parse_args()

    with RackIndustrialDockingController(
        front_ultrasonic_ids=parse_ids(args.front_ids),
        rear_ultrasonic_ids=parse_ids(args.rear_ids),
    ) as rack:
        if args.mode == "preflight":
            print(rack.preflight(allow_estop_pedal_fault=args.allow_estop_pedal_fault))
            return

        if args.mode == "read":
            for _ in range(args.samples):
                print_snapshot(rack.read_snapshot())
                time.sleep(args.sample_interval_s)
            return

        if args.mode == "forward":
            result = rack.forward(
                distance_m=args.forward_distance_m,
                speed_mps=args.forward_speed_mps,
                front_hard_stop_mm=args.front_hard_stop_mm,
                allow_estop_pedal_fault=args.allow_estop_pedal_fault,
            )
            print(result)
            return

        if args.mode == "coarse":
            result = rack.coarse_position(
                coarse_speed_mps=args.coarse_speed_mps,
                coarse_stop_m=args.coarse_stop_m,
                switch_ultrasonic_mm=args.switch_ultrasonic_mm,
                ultrasonic_takeover_mm=args.ultrasonic_takeover_mm,
                allow_estop_pedal_fault=args.allow_estop_pedal_fault,
            )
            print(result)
            return

        if args.mode == "fine":
            result = rack.fine_position(
                final_stop_mm=args.final_stop_mm,
                final_brake_margin_mm=args.final_brake_margin_mm,
                final_speed_mps=args.final_speed_mps,
                allow_estop_pedal_fault=args.allow_estop_pedal_fault,
                on_sample=print_fine_sample,
            )
            print(result)
            return

        if args.mode == "retreat":
            result = rack.retreat(
                distance_m=args.retreat_distance_m,
                speed_mps=args.retreat_speed_mps,
                method=args.retreat_method,
                rear_stop_mm=args.rear_stop_mm,
                rear_hard_stop_mm=args.rear_hard_stop_mm,
                allow_estop_pedal_fault=args.allow_estop_pedal_fault,
                on_sample=print_retreat_sample,
            )
            print(result)
            return

        if args.mode == "approach":
            result = rack.approach(
                allow_estop_pedal_fault=args.allow_estop_pedal_fault,
                coarse_speed_mps=args.coarse_speed_mps,
                coarse_stop_m=args.coarse_stop_m,
                switch_ultrasonic_mm=args.switch_ultrasonic_mm,
                ultrasonic_takeover_mm=args.ultrasonic_takeover_mm,
                final_stop_mm=args.final_stop_mm,
                final_brake_margin_mm=args.final_brake_margin_mm,
                final_speed_mps=args.final_speed_mps,
                on_final_sample=print_fine_sample,
            )
            print(result)
            return

        if args.mode == "cycle":
            result = rack.cycle(
                allow_estop_pedal_fault=args.allow_estop_pedal_fault,
                retreat_distance_m=args.retreat_distance_m,
                retreat_speed_mps=args.retreat_speed_mps,
                retreat_method=args.retreat_method,
                rear_stop_mm=args.rear_stop_mm,
                rear_hard_stop_mm=args.rear_hard_stop_mm,
                coarse_speed_mps=args.coarse_speed_mps,
                coarse_stop_m=args.coarse_stop_m,
                switch_ultrasonic_mm=args.switch_ultrasonic_mm,
                ultrasonic_takeover_mm=args.ultrasonic_takeover_mm,
                final_stop_mm=args.final_stop_mm,
                final_brake_margin_mm=args.final_brake_margin_mm,
                final_speed_mps=args.final_speed_mps,
                on_retreat_sample=print_retreat_sample,
                on_final_sample=print_fine_sample,
            )
            print(result)
            return


if __name__ == "__main__":
    main()
