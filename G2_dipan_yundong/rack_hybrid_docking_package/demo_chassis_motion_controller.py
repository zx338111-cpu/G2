#!/usr/bin/env python3
"""ChassisMotionController 调用示例。

这个 demo 用来说明业务代码应该怎么调用
``g2_primitives.chassis_motion.ChassisMotionController``。

默认行为是安全的：

- 不传任何参数时，只做 ``GRAB_PRE`` 的导航 dry-run；
- dry-run 只读取 station config 并打印目标，不会连接机器人，也不会运动；
- 所有真实底盘运动都必须显式选择 live 动作，并同时传
  ``--allow-live --confirm-physical``。

常用示例：

1. 只列出当前配置里的站点，不连接机器人：

   python3 rack_hybrid_docking_package/demo_chassis_motion_controller.py \
     --action list-stations

2. 只做导航 dry-run，不连接机器人，不运动：

   python3 rack_hybrid_docking_package/demo_chassis_motion_controller.py \
     --action nav-dry-run --station GRAB_PRE

3. 做导航 readiness，只读连接机器人，不运动：

   python3 rack_hybrid_docking_package/demo_chassis_motion_controller.py \
     --action readiness

4. 真实导航到 GRAB_PRE，会让底盘运动。只允许在现场确认安全后执行：

   python3 rack_hybrid_docking_package/demo_chassis_motion_controller.py \
     --action nav-live --station GRAB_PRE \
     --allow-live --confirm-physical

5. 真实料架前精定位，会让底盘低速向前靠近：

   python3 rack_hybrid_docking_package/demo_chassis_motion_controller.py \
     --action fine-position-live \
     --fine-stop-mm 328 --fine-brake-margin-mm 20 --fine-speed-mps 0.08 \
     --allow-live --confirm-physical

6. 真实局部后退，会让底盘后退：

   python3 rack_hybrid_docking_package/demo_chassis_motion_controller.py \
     --action retreat-live \
     --retreat-distance-m 0.45 --retreat-speed-mps 0.20 \
     --allow-live --confirm-physical

7. 真实小范围相对移动，会让底盘按车体坐标系移动：

   python3 rack_hybrid_docking_package/demo_chassis_motion_controller.py \
     --action relative-live \
     --relative-x-m 0.03 --relative-y-m 0.0 \
     --allow-live --confirm-physical

坐标和动作边界：

- 地图导航使用 station config 里的全局地图点；
- 精定位和后退是料架局部动作，不依赖地图目标，但依赖底盘安全状态和
  超声传感器；
- relative-live 是车体坐标系小位移，x 正向为前，y 正向为左；
- demo 只展示调用方式，不负责替代主任务脚本的完整状态机、checkpoint、
  视觉采集或日志分析。
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, is_dataclass
import json
from pathlib import Path
import sys


PACKAGE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_DIR.parent
DEFAULT_CONFIG = PACKAGE_DIR / "industrial_station_config.json"

# 直接从源码目录运行 demo 时，Python 默认只把当前文件所在目录加入
# sys.path。这里显式加入 package 目录，保证 ``from g2_primitives...`` 在
# 本地和机器人侧都能工作。
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from g2_primitives.chassis_motion import ChassisMotionController


READ_ONLY_ACTIONS = frozenset(
    {
        "list-stations",
        "nav-dry-run",
        "readiness",
        "rack-preflight",
    }
)
LIVE_ACTIONS = frozenset(
    {
        "nav-live",
        "fine-position-live",
        "retreat-live",
        "relative-live",
    }
)


def jsonable(value):
    """把 GDK/控制器返回对象转换成 JSON 友好的结构。

    控制器底层有些返回值是 dataclass，有些是普通对象。demo 统一输出
    JSON，方便复制到日志里排查。
    """

    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [jsonable(item) for item in value]

    # 对普通 GDK 对象做 best-effort 展开，只保留简单字段。这样既能看到
    # status/message 等有价值内容，又不会把方法、连接句柄等不可序列化对象
    # 打进日志。
    fields = {}
    for name in dir(value):
        if name.startswith("_"):
            continue
        try:
            item = getattr(value, name)
        except Exception:
            continue
        if callable(item):
            continue
        if isinstance(item, (str, int, float, bool, type(None), list, tuple, dict)):
            fields[name] = jsonable(item)
    return fields if fields else repr(value)


def emit(event: str, **fields: object) -> None:
    """打印一行结构化 JSON 事件。"""

    print(json.dumps({"event": event, **jsonable(fields)}, ensure_ascii=False, indent=2), flush=True)


def parse_args() -> argparse.Namespace:
    """解析 demo 参数。

    参数分三组：

    - 通用参数：action、station、station-config；
    - 安全确认参数：allow-live、confirm-physical；
    - 各具体动作的 tuning 参数。
    """

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--action",
        choices=sorted(READ_ONLY_ACTIONS | LIVE_ACTIONS),
        default="nav-dry-run",
        help="要演示的调用。默认 nav-dry-run，不运动。",
    )
    parser.add_argument(
        "--station-config",
        default=str(DEFAULT_CONFIG),
        help="地图站点 JSON。导航/readiness/list-stations 会读取它。",
    )
    parser.add_argument("--station", default="GRAB_PRE", help="地图导航目标站点名。")

    # 真实运动必须同时满足两个确认。用两个 flag 是为了避免用户误输入一个
    # 参数就让机器人运动。
    parser.add_argument("--allow-live", action="store_true", help="第一层实机运动确认。")
    parser.add_argument("--confirm-physical", action="store_true", help="第二层实机运动确认。")

    parser.add_argument(
        "--allow-estop-pedal-fault",
        action="store_true",
        help="传给局部底盘动作的现场安全放行参数；只在你确认该状态可接受时使用。",
    )

    # yaw refine 参数用于 nav-live。它会发低速角速度，所以只在 live 导航时
    # 生效。nav-dry-run 强制 refine_yaw=False。
    parser.add_argument("--refine-yaw-tolerance-deg", type=float, default=1.5)
    parser.add_argument("--refine-yaw-max-error-deg", type=float, default=10.0)
    parser.add_argument("--refine-yaw-angular-speed-radps", type=float, default=0.05)
    parser.add_argument("--refine-yaw-fine-angular-speed-radps", type=float, default=0.02)
    parser.add_argument("--refine-yaw-timeout-s", type=float, default=12.0)

    # 精定位参数：对应 ChassisMotionController.fine_position()。
    parser.add_argument("--fine-stop-mm", type=int, default=328)
    parser.add_argument("--fine-brake-margin-mm", type=int, default=20)
    parser.add_argument("--fine-speed-mps", type=float, default=0.08)
    parser.add_argument("--fine-max-duration-s", type=float, default=60.0)

    # 后退参数：对应 ChassisMotionController.retreat()。
    parser.add_argument("--retreat-distance-m", type=float, default=0.45)
    parser.add_argument("--retreat-speed-mps", type=float, default=0.20)

    # 小范围相对移动参数：对应 ChassisMotionController.relative_move()。
    # max_abs_m 是硬限幅，x/y 任一方向超过这个值都会直接拒绝。
    parser.add_argument("--relative-x-m", type=float, default=0.03)
    parser.add_argument("--relative-y-m", type=float, default=0.0)
    parser.add_argument("--relative-timeout-s", type=float, default=12.0)
    parser.add_argument("--relative-max-abs-m", type=float, default=0.20)
    return parser.parse_args()


def require_live_confirmation(args: argparse.Namespace) -> None:
    """真实运动前的 demo 级确认门。

    注意：这只是 demo 的第一层防误触保护。真实项目里仍然要在主流程里做
    read-only snapshot、readiness、充电状态、定位/odom、周围障碍和现场人员
    确认。
    """

    if args.action not in LIVE_ACTIONS:
        return
    if args.allow_live and args.confirm_physical:
        return
    raise SystemExit(
        f"{args.action} 会让底盘运动，必须同时传 --allow-live --confirm-physical。"
    )


def build_controller(args: argparse.Namespace) -> ChassisMotionController:
    """创建统一底盘控制类。

    即使只做 fine-position/retreat，这里也传入 station config。这样 demo 和
    主流程构造方式一致，后续统一加日志或 site profile 时更简单。
    """

    return ChassisMotionController(args.station_config)


def main() -> int:
    """根据 action 演示 ChassisMotionController 的不同调用方式。"""

    args = parse_args()
    require_live_confirmation(args)
    controller = build_controller(args)

    emit(
        "demo_start",
        action=args.action,
        station=args.station,
        station_config=str(Path(args.station_config).resolve()),
        read_only=args.action in READ_ONLY_ACTIONS,
    )

    if args.action == "list-stations":
        emit("stations", stations=controller.list_stations())
        return 0

    if args.action == "nav-dry-run":
        result = controller.goto_station(args.station, live=False, refine_yaw=False)
        emit("nav_dry_run_result", result=result)
        return 0

    if args.action == "readiness":
        result = controller.readiness_check()
        emit("readiness_result", result=result)
        return 0

    if args.action == "rack-preflight":
        result = controller.rack_preflight(allow_estop_pedal_fault=args.allow_estop_pedal_fault)
        emit("rack_preflight_result", result=result)
        return 0

    if args.action == "nav-live":
        result = controller.goto_station(
            args.station,
            live=True,
            refine_yaw=True,
            refine_yaw_tolerance_deg=args.refine_yaw_tolerance_deg,
            refine_yaw_max_error_deg=args.refine_yaw_max_error_deg,
            refine_yaw_angular_speed_radps=args.refine_yaw_angular_speed_radps,
            refine_yaw_fine_angular_speed_radps=args.refine_yaw_fine_angular_speed_radps,
            refine_yaw_timeout_s=args.refine_yaw_timeout_s,
        )
        emit("nav_live_result", result=result)
        return 0

    if args.action == "fine-position-live":
        result = controller.fine_position(
            final_stop_mm=args.fine_stop_mm,
            final_brake_margin_mm=args.fine_brake_margin_mm,
            final_speed_mps=args.fine_speed_mps,
            max_duration_s=args.fine_max_duration_s,
            allow_estop_pedal_fault=args.allow_estop_pedal_fault,
        )
        emit("fine_position_live_result", result=result)
        return 0

    if args.action == "retreat-live":
        result = controller.retreat(
            distance_m=args.retreat_distance_m,
            speed_mps=args.retreat_speed_mps,
            allow_estop_pedal_fault=args.allow_estop_pedal_fault,
        )
        emit("retreat_live_result", result=result)
        return 0

    if args.action == "relative-live":
        result = controller.relative_move(
            x_m=args.relative_x_m,
            y_m=args.relative_y_m,
            timeout_s=args.relative_timeout_s,
            max_abs_m=args.relative_max_abs_m,
            allow_estop_pedal_fault=args.allow_estop_pedal_fault,
        )
        emit("relative_live_result", result=result)
        return 0

    raise SystemExit(f"unsupported action: {args.action}")


if __name__ == "__main__":
    raise SystemExit(main())
