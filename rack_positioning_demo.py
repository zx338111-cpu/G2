#!/usr/bin/env python3
"""
One-shot rack-relative positioning demo.

Default mode is dry-run. It prints the planned chassis command and does not
move the robot unless --execute is provided.

Robot-side example:
  source /home/agi/app/env.sh
  python3 rack_positioning_demo.py --rack load --rack-x 1.20 --rack-y 0.04 --rack-yaw 2.0

Live execution:
  source /home/agi/app/env.sh
  python3 rack_positioning_demo.py --rack load --rack-x 1.20 --rack-y 0.04 --rack-yaw 2.0 --execute
"""

import argparse

from rack_positioning import (
    DockingTolerance,
    Pose2D,
    RackRelativePositioner,
    make_front_dock_profile,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plan or execute one rack-relative chassis correction."
    )
    parser.add_argument(
        "--rack",
        choices=("load", "unload"),
        required=True,
        help="Rack profile name.",
    )
    parser.add_argument(
        "--rack-x",
        type=float,
        required=True,
        help="Measured rack docking-frame x in robot frame, meters.",
    )
    parser.add_argument(
        "--rack-y",
        type=float,
        required=True,
        help="Measured rack docking-frame y in robot frame, meters.",
    )
    parser.add_argument(
        "--rack-yaw",
        type=float,
        default=0.0,
        help="Measured rack docking-frame yaw in robot frame, degrees.",
    )
    parser.add_argument(
        "--standoff",
        type=float,
        default=0.65,
        help="Desired robot standoff behind the rack docking origin, meters.",
    )
    parser.add_argument(
        "--lateral-offset",
        type=float,
        default=0.0,
        help="Desired robot lateral offset in rack frame, meters; positive left.",
    )
    parser.add_argument(
        "--yaw-offset",
        type=float,
        default=0.0,
        help="Desired robot yaw offset in rack frame, degrees.",
    )
    parser.add_argument("--tol-x", type=float, default=0.03)
    parser.add_argument("--tol-y", type=float, default=0.02)
    parser.add_argument("--tol-yaw", type=float, default=1.5)
    parser.add_argument(
        "--max-step",
        type=float,
        default=0.25,
        help="Maximum one-shot translation correction, meters.",
    )
    parser.add_argument(
        "--max-yaw-step",
        type=float,
        default=12.0,
        help="Maximum one-shot yaw correction, degrees.",
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually call ChassisController.move_relative().",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    profile = make_front_dock_profile(
        name=args.rack,
        standoff_m=args.standoff,
        lateral_offset_m=args.lateral_offset,
        yaw_offset_deg=args.yaw_offset,
        tolerance=DockingTolerance(
            x_m=args.tol_x,
            y_m=args.tol_y,
            yaw_deg=args.tol_yaw,
        ),
        max_step_m=args.max_step,
        max_step_yaw_deg=args.max_yaw_step,
        timeout_s=args.timeout,
    )

    rack_pose = Pose2D(args.rack_x, args.rack_y, args.rack_yaw)

    controller = None
    if args.execute:
        from chassis_controller import ChassisController

        controller = ChassisController()

    try:
        positioner = RackRelativePositioner(
            controller=controller,
            dry_run=not args.execute,
        )
        result = positioner.align_once(rack_pose, profile)
        if args.execute:
            print(f"state={result.state} success={result.success}")
        else:
            print("dry_run=True; add --execute only after the measured pose is verified.")
    finally:
        if controller is not None:
            controller.release()


if __name__ == "__main__":
    main()
