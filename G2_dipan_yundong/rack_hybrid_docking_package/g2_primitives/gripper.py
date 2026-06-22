"""Dual omnipicker gripper primitive.

This module is the importable implementation behind:

- ``move_ee_pose_open_2.py``;
- ``move_ee_pose_close_2.py``.

Both grippers are commanded every time.  The mission runner treats a failed
left or right command as a failed local step so the operator can inspect the
physical gripper state before continuing.

Call path:

``move_ee_pose_open_2.py`` maps to ``open_both`` and
``move_ee_pose_close_2.py`` maps to ``close_both``. The names are preserved in
logs because operators already recognize those old CLI scripts.
"""

from __future__ import annotations

import time

from .gdk_context import gdk_session


class GripperController:
    """Open/close both omnipicker grippers through GDK end-effector commands."""

    # These are the validated positions used in the live runs.  Open is a
    # negative omnipicker joint position; closed is zero.
    OPEN_POSITION = -0.785
    CLOSED_POSITION = 0.0

    def _make_joint_states(self, agibot_gdk, group: str, position: float):
        """Build the GDK JointStates object for one gripper group."""

        joint_states = agibot_gdk.JointStates()
        joint_states.group = group
        joint_states.target_type = "omnipicker"
        joint_state = agibot_gdk.JointState()
        joint_state.position = position
        joint_states.states = [joint_state]
        joint_states.nums = len(joint_states.states)
        return joint_states

    def set_both(self, position: float, *, dry_run: bool = False, settle_s: float = 0.05) -> bool:
        """Command right then left gripper to the same omnipicker position."""

        if settle_s < 0.0:
            raise ValueError("settle_s must be >= 0")
        action = "open" if position < -0.1 else "close"
        print(f"move_ee_pose_{action}_2 position={position:.3f}", flush=True)
        if dry_run:
            print("dry-run: skip GDK init and gripper movement", flush=True)
            return True

        ok = True
        with gdk_session() as agibot_gdk:
            robot = agibot_gdk.Robot()
            time.sleep(2.0)
            for group, label in (("right_tool", "right"), ("left_tool", "left")):
                # Commanding the two grippers separately gives the log a clear
                # left/right success signal, which helped diagnose previous
                # right-gripper enable/blue-light issues.
                try:
                    robot.move_ee_pos(self._make_joint_states(agibot_gdk, group, position))
                    print(f"{label}_gripper_{action}_ok", flush=True)
                    time.sleep(settle_s)
                except Exception as exc:
                    ok = False
                    print(f"{label}_gripper_{action}_failed: {exc}", flush=True)
        return ok

    def open_both(self, *, dry_run: bool = False, settle_s: float = 0.05) -> bool:
        """Open both grippers."""

        return self.set_both(self.OPEN_POSITION, dry_run=dry_run, settle_s=settle_s)

    def close_both(self, *, dry_run: bool = False, settle_s: float = 0.05) -> bool:
        """Close both grippers."""

        return self.set_both(self.CLOSED_POSITION, dry_run=dry_run, settle_s=settle_s)
