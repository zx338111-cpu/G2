"""Importable G2 motion primitives used by the industrial-cell runner.

The package provides the class/import layer for the current architecture.  The
top-level runner imports these classes directly, while legacy command scripts
delegate to the same classes for manual testing.

Imports are lazy through ``__getattr__`` so importing ``g2_primitives`` alone
does not initialize GDK or pull in modules that only exist on the robot.

Why lazy imports matter:

- Some modules import robot-only packages such as ``agibot_gdk`` or
  ``end_effector_controller``. A pure file check or dry-run should be able to
  import this package without failing on a laptop or before ``env.sh`` is
  sourced on the robot.
- Each primitive file has a narrow ownership boundary: arm joint pose, waist
  pose, gripper state, Cartesian end-effector offset, map navigation, or rack
  docking. The mission runner should compose them rather than add new GDK calls
  inline.
"""

__all__ = [
    "ArmJointController",
    "ChassisMotionController",
    "EndEffectorOffsetController",
    "GripperController",
    "MapNavController",
    "RackDockingController",
    "WaistController",
]


def __getattr__(name: str):
    """Load primitive classes only when the caller asks for them."""

    if name == "ArmJointController":
        from .arm import ArmJointController

        return ArmJointController
    if name == "ChassisMotionController":
        from .chassis_motion import ChassisMotionController

        return ChassisMotionController
    if name == "EndEffectorOffsetController":
        from .ee_offset import EndEffectorOffsetController

        return EndEffectorOffsetController
    if name == "GripperController":
        from .gripper import GripperController

        return GripperController
    if name == "MapNavController":
        from .nav import MapNavController

        return MapNavController
    if name == "RackDockingController":
        from .rack import RackDockingController

        return RackDockingController
    if name == "WaistController":
        from .waist import WaistController

        return WaistController
    raise AttributeError(name)
