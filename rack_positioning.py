#!/usr/bin/env python3
"""
Rack-relative docking planner for the G2 chassis.

This module is intentionally independent from agibot_gdk.  It only computes
the relative chassis command that should be sent through ChassisController.

Coordinate convention:
  - Robot frame: +x forward, +y left, +yaw counter-clockwise.
  - rack_pose_in_robot: pose of the rack docking frame measured in the current
    robot frame.
  - desired_robot_pose_in_rack: desired final robot pose expressed in the rack
    docking frame.

If the robot is already at the desired rack-relative pose, then:
  compose(rack_pose_in_robot, desired_robot_pose_in_rack) == identity
"""

from dataclasses import dataclass, field
import math
import time


def normalize_yaw_deg(yaw_deg):
    """Normalize an angle to [-180, 180)."""
    return (yaw_deg + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw_deg: float = 0.0

    @property
    def translation_m(self):
        return math.hypot(self.x, self.y)

    def normalized(self):
        return Pose2D(self.x, self.y, normalize_yaw_deg(self.yaw_deg))


def compose_pose(a, b):
    """Return pose a * b."""
    yaw = math.radians(a.yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    return Pose2D(
        x=a.x + c * b.x - s * b.y,
        y=a.y + s * b.x + c * b.y,
        yaw_deg=normalize_yaw_deg(a.yaw_deg + b.yaw_deg),
    )


def inverse_pose(pose):
    """Return inverse(pose)."""
    yaw = math.radians(pose.yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    return Pose2D(
        x=-(c * pose.x + s * pose.y),
        y=s * pose.x - c * pose.y,
        yaw_deg=normalize_yaw_deg(-pose.yaw_deg),
    )


@dataclass(frozen=True)
class DockingTolerance:
    x_m: float = 0.03
    y_m: float = 0.02
    yaw_deg: float = 1.5


@dataclass(frozen=True)
class DockingProfile:
    name: str
    desired_robot_pose_in_rack: Pose2D
    tolerance: DockingTolerance = field(default_factory=DockingTolerance)
    max_step_m: float = 0.25
    max_step_yaw_deg: float = 12.0
    max_total_m: float = 2.0
    max_total_yaw_deg: float = 90.0
    timeout_s: float = 30.0
    settle_s: float = 0.5
    success_states: tuple = (3, 9)


@dataclass(frozen=True)
class RackMovePlan:
    profile_name: str
    rack_pose_in_robot: Pose2D
    desired_robot_pose_in_rack: Pose2D
    raw_command: Pose2D
    command: Pose2D
    in_tolerance: bool
    limited: bool

    def format_summary(self):
        return (
            f"profile={self.profile_name} "
            f"rack=({self.rack_pose_in_robot.x:+.3f},"
            f"{self.rack_pose_in_robot.y:+.3f},"
            f"{self.rack_pose_in_robot.yaw_deg:+.2f}deg) "
            f"raw=({self.raw_command.x:+.3f},"
            f"{self.raw_command.y:+.3f},"
            f"{self.raw_command.yaw_deg:+.2f}deg) "
            f"cmd=({self.command.x:+.3f},"
            f"{self.command.y:+.3f},"
            f"{self.command.yaw_deg:+.2f}deg) "
            f"in_tolerance={self.in_tolerance} limited={self.limited}"
        )


@dataclass(frozen=True)
class RackMoveResult:
    plan: RackMovePlan
    state: int = None
    moved: bool = False

    @property
    def success(self):
        if self.plan.in_tolerance:
            return True
        return self.moved and self.state in (3, 9)


def make_front_dock_profile(
    name,
    standoff_m,
    lateral_offset_m=0.0,
    yaw_offset_deg=0.0,
    tolerance=None,
    max_step_m=0.25,
    max_step_yaw_deg=12.0,
    timeout_s=30.0,
):
    """
    Build a common front-docking profile.

    Assumption: the rack docking frame has the same yaw direction as the robot
    should have when aligned. The robot's target position is standoff_m behind
    the rack origin, so desired x is negative.
    """
    return DockingProfile(
        name=name,
        desired_robot_pose_in_rack=Pose2D(
            x=-float(standoff_m),
            y=float(lateral_offset_m),
            yaw_deg=float(yaw_offset_deg),
        ),
        tolerance=tolerance or DockingTolerance(),
        max_step_m=float(max_step_m),
        max_step_yaw_deg=float(max_step_yaw_deg),
        timeout_s=float(timeout_s),
    )


DEFAULT_PROFILES = {
    "load": make_front_dock_profile("load", standoff_m=0.65),
    "unload": make_front_dock_profile("unload", standoff_m=0.65),
}


class RackRelativePositioner:
    def __init__(self, controller=None, dry_run=True):
        self.controller = controller
        self.dry_run = dry_run

    def plan(self, rack_pose_in_robot, profile):
        rack_pose_in_robot = rack_pose_in_robot.normalized()
        desired = profile.desired_robot_pose_in_rack.normalized()
        raw = compose_pose(rack_pose_in_robot, desired)

        if raw.translation_m > profile.max_total_m:
            raise ValueError(
                f"Rack correction {raw.translation_m:.3f}m exceeds "
                f"max_total_m={profile.max_total_m:.3f}"
            )
        if abs(raw.yaw_deg) > profile.max_total_yaw_deg:
            raise ValueError(
                f"Rack yaw correction {raw.yaw_deg:.2f}deg exceeds "
                f"max_total_yaw_deg={profile.max_total_yaw_deg:.2f}"
            )

        in_tolerance = (
            abs(raw.x) <= profile.tolerance.x_m
            and abs(raw.y) <= profile.tolerance.y_m
            and abs(raw.yaw_deg) <= profile.tolerance.yaw_deg
        )

        command = self._clamp_command(raw, profile)
        if in_tolerance:
            command = Pose2D()

        limited = (
            abs(command.x - raw.x) > 1e-9
            or abs(command.y - raw.y) > 1e-9
            or abs(command.yaw_deg - raw.yaw_deg) > 1e-9
        ) and not in_tolerance

        return RackMovePlan(
            profile_name=profile.name,
            rack_pose_in_robot=rack_pose_in_robot,
            desired_robot_pose_in_rack=desired,
            raw_command=raw,
            command=command,
            in_tolerance=in_tolerance,
            limited=limited,
        )

    def align_once(self, rack_pose_in_robot, profile, dry_run=None, verbose=True):
        plan = self.plan(rack_pose_in_robot, profile)
        if verbose:
            print(plan.format_summary())

        should_dry_run = self.dry_run if dry_run is None else dry_run
        if plan.in_tolerance or should_dry_run:
            return RackMoveResult(plan=plan, state=None, moved=False)
        if self.controller is None:
            raise RuntimeError("controller is required when dry_run is False")

        state = self.controller.move_relative(
            x_m=plan.command.x,
            y_m=plan.command.y,
            yaw_deg=plan.command.yaw_deg,
            timeout=profile.timeout_s,
        )
        return RackMoveResult(plan=plan, state=state, moved=True)

    def align_closed_loop(
        self,
        measure_rack_pose_fn,
        profile,
        max_iterations=4,
        dry_run=None,
        verbose=True,
    ):
        """
        Re-measure after each move until the rack-relative residual is small.

        measure_rack_pose_fn must return Pose2D in the current robot frame.
        """
        results = []
        for index in range(max_iterations):
            rack_pose = measure_rack_pose_fn()
            if verbose:
                print(f"[rack-align] iteration {index + 1}/{max_iterations}")
            result = self.align_once(
                rack_pose_in_robot=rack_pose,
                profile=profile,
                dry_run=dry_run,
                verbose=verbose,
            )
            results.append(result)

            if result.plan.in_tolerance:
                break
            if result.moved and result.state not in profile.success_states:
                break
            time.sleep(profile.settle_s)
        return results

    @staticmethod
    def _clamp_command(command, profile):
        x = command.x
        y = command.y
        yaw = command.yaw_deg

        length = math.hypot(x, y)
        if length > profile.max_step_m:
            scale = profile.max_step_m / length
            x *= scale
            y *= scale

        if yaw > profile.max_step_yaw_deg:
            yaw = profile.max_step_yaw_deg
        elif yaw < -profile.max_step_yaw_deg:
            yaw = -profile.max_step_yaw_deg

        return Pose2D(x=x, y=y, yaw_deg=normalize_yaw_deg(yaw))
