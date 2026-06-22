# BOX_528_1 Map 7 Safe Overlay

This overlay is designed for a copied workflow directory only. It must not be
placed over `/data/hondagys/wxf/BOX_528_1` unless that directory is itself a
copy.

Current verified state on 2026-06-22:

- Robot host: `agi@192.168.0.6`.
- Robot-side overlay path: `/data/hondagys/wxf/BOX_528_1_map7_safe_20260622_1340`.
- Current map: `7`.
- The robot is already near map guide point `2` in x/y:
  distance about `0.044 m`, tolerance `0.080 m`.
- The remaining point-2 mismatch is yaw: current yaw about `162.7 deg`, target
  yaw about `92.5 deg`, error about `-70 deg`.
- SLAM odom and localization are usable; latest observed `loc_confidence=80`.
- Do not run chassis navigation while the latest chassis power state reports
  `charge_plug_insert_state=1` and batteries are charging.

What it changes:

- Uses cached map 7 guide points to avoid repeated large `Map.get_map()` reads.
- Waits briefly for `/pnc/task_state` to become readable before sending a
  navigation command.
- Blocks chassis navigation while the charge plug is inserted or the emergency
  stop pedal is active.
- Blocks navigation while SLAM odom is unavailable or localization confidence is
  below the configured threshold.
- Keeps the original `RobotController` behavior for all normal navigation.
- Adds a narrow fallback for map guide point `2`: if PNC reaches x/y within
  `0.08 m` but fails in terminal yaw because `SpinToGoal` collision checking
  reports collision, the wrapper returns success for that configured waypoint.

What it does not change:

- No navigation config files are modified.
- The original `/data/hondagys/wxf/BOX_528_1/robot_controller.py` is not
  modified.
- No motion behavior is changed for other guide points unless
  `G2_POSITION_ONLY_WAYPOINTS` is set.

Runtime switches:

- `G2_POSITION_ONLY_WAYPOINTS=2` is the default.
- `G2_POSITION_ONLY_WAYPOINTS=none` disables the fallback.
- `G2_POSITION_ONLY_TOL_M=0.08` changes the position acceptance tolerance.
- `G2_USE_LIVE_MAP=1` disables cached map points and uses the live Map API.
- `G2_PNC_READY_TIMEOUT_S=3.0` controls the startup wait for task-state data.
- `G2_REQUIRE_NOT_CHARGING=0` disables the charge-plug gate.
- `G2_REQUIRE_LOC_READY=0` disables the odom/confidence gate.
- `G2_MIN_LOC_CONFIDENCE=50` changes the minimum localization confidence.
