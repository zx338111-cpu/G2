"""底盘运动统一入口。

这个文件的目标不是重新写一套底盘控制算法，而是把当前项目里所有“会
占用或移动底盘”的能力收口到一个类里，方便主流程和后续业务集成调用。

为什么要单独做这个类：

1. 主任务脚本只应该表达业务流程，比如“去 GRAB_PRE”“料架前精定位”
   “放完后退出”。它不应该到处散落 GDK、PNC、超声、地图导航的细节。
2. 底盘动作的安全边界比手臂/夹爪更敏感。地图导航、精定位、后退、
   小范围相对位移都应该经过统一的入口，方便后续统一加日志、限幅、
   现场确认和回退策略。
3. 现有底层能力已经验证过，所以这里保持“薄封装”：只负责组合和
   命名，不改底层传感器滤波、导航到站判定、刹车补偿或后方保护逻辑。

当前类里包含四类动作：

- ``goto_station``：地图站点导航。用于 HOME_SAFE、GRAB_PRE、
  PLACE_PRE、RECOVERY_SAFE 这种全局地图点。
- ``fine_position``：料架前超声精定位。用于机器人已经到达抓/放预备
  点后，低速靠近料架并停在指定毫米距离。
- ``retreat``：局部后退。用于抓完或放完后从料架前退出，带后方超声
  保护。
- ``relative_move``：小范围底盘相对位移。用于现场微调，例如精定位后
  再往前 3cm 或横向挪一点。这个能力风险更高，必须小距离限幅。

非常重要的安全说明：

- 导入本模块不会初始化 GDK，也不会连接机器人。
- 真正依赖 ``agibot_gdk`` 或底层机器人服务的 import 都放在方法内部，
  只有调用对应动作时才会发生。
- ``readiness_check`` 和 ``rack_preflight`` 是只读检查，不应该发运动命令。
- ``goto_station(live=False, refine_yaw=False)`` 是导航 dry-run，只验证站点
  和打印目标，不发运动命令。
- ``goto_station(live=True, ...)``、``fine_position``、``retreat``、
  ``relative_move`` 都可能让底盘运动，业务层必须先做现场安全确认。
"""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any


# PNC task-state 这些数字来自现场 GDK/PNC 行为观察。这里不把它们翻译成
# Enum，是因为不同版本 SDK 暴露的名字不稳定；保留数字能和历史日志对应。
#
# relative_move 只认为 3/9 是成功终态。其它 RUNNING/PLANNING/PAUSED 类状态
# 只能说明任务还在变化，不能当成已经到位。
CHASSIS_RELATIVE_SUCCESS_STATES = (3, 9)
CHASSIS_RELATIVE_RUNNING_STATES = (1, 2, 4, 5, 6, 8)

# 执行新的 relative_move 前，如果旧任务不在这些“空闲或已结束”状态里，
# 会尝试 cancel 一次，避免旧 PNC 任务占用底盘控制权。
PNC_IDLE_OR_DONE_STATES = (0, 3, 7, 8, 9)


class ChassisMotionController:
    """底盘运动统一控制类。

    这个类给上层业务一个稳定的调用面：

    .. code-block:: python

        chassis = ChassisMotionController("industrial_station_config.json")
        chassis.goto_station("GRAB_PRE", live=False, refine_yaw=False)

    设计边界：

    - 本类只处理底盘相关动作，不处理手臂、腰部、夹爪。
    - 地图导航继续委托给 ``MapNavController``。
    - 料架前精定位和后退继续委托给 ``RackDockingController``。
    - ``relative_move`` 保留在这里，是因为它本质也是底盘相对运动，不能
      散落到任务脚本里。

    ``station_config_path`` 的含义：

    - 需要地图站点导航或 readiness 时必须传入。
    - 只调用 ``fine_position`` / ``retreat`` 时理论上不需要地图配置，但
      实际主流程仍然传入同一个 config，方便统一构造。
    """

    def __init__(self, station_config_path: str | Path | None = None):
        """保存站点配置路径，但不在构造函数里连接机器人。

        构造函数必须保持轻量。很多离线工具、py_compile、dry-run 会 import
        这个类；如果这里初始化 GDK，离线环境就会失败，也会增加误触发风险。
        """

        self.station_config_path = Path(station_config_path).resolve() if station_config_path else None
        self._map_nav = None

    def list_stations(self) -> list[str]:
        """列出 station config 里的地图站点名称。

        典型用途：

        - demo 或调试脚本先确认当前配置里有哪些站点；
        - 防止业务层把 ``GRAB_PRE``、``PLACE_PRE`` 这种名字拼错；
        - 不发运动命令，只读取 JSON 配置。
        """

        return self._map_nav_controller().list_stations()

    def readiness_check(self) -> dict[str, Any]:
        """执行地图导航前的只读 readiness 检查。

        这个检查会读取机器人状态，但不应该发运动命令。底层
        ``MapNavController`` 会检查：

        - 底盘电源/充电状态；
        - motion-control 错误码；
        - PNC 当前任务状态；
        - SLAM/map/pose/odom 是否可用；
        - 当前地图和 station config 是否匹配。

        返回值是结构化 dict，上层一般看 ``preflight["ok"]``。如果为
        False，不要继续下发底盘运动。
        """

        return self._map_nav_controller().readiness_check()

    def goto_station(
        self,
        station: str,
        *,
        live: bool,
        refine_yaw: bool,
        refine_yaw_tolerance_deg: float = 1.0,
        refine_yaw_max_error_deg: float = 6.0,
        refine_yaw_angular_speed_radps: float = 0.08,
        refine_yaw_fine_angular_speed_radps: float = 0.035,
        refine_yaw_timeout_s: float = 8.0,
        refine_yaw_hz: float = 10.0,
        refine_yaw_stable_samples: int = 3,
    ) -> dict[str, Any]:
        """导航到一个命名地图站点。

        参数说明：

        - ``station``：站点名，例如 ``HOME_SAFE``、``GRAB_PRE``、
          ``PLACE_PRE``、``RECOVERY_SAFE``。
        - ``live=False``：只做 dry-run，验证站点存在并打印目标，不发运动。
        - ``live=True``：真实调用 PNC ``normal_navi``，会让底盘移动。
        - ``refine_yaw``：到站后是否低速修正 yaw。只能和 ``live=True`` 一起
          使用，因为 yaw refine 本身会发角速度。
        - ``refine_yaw_*``：yaw 精修的限幅和超时，必须保持小速度和短超时。

        返回值：

        - 成功时返回 ``{"ok": True, ...}``。
        - 导航未到位时返回 ``{"ok": False, ...}``，主流程会把它当作硬失败。

        调用建议：

        - 调试配置时先用 ``live=False``。
        - 实机运动前先单独跑 ``readiness_check``。
        - 只有现场确认安全、机器人不充电、定位/odom 正常时才用
          ``live=True``。
        """

        return self._map_nav_controller().goto_station(
            station,
            live=live,
            refine_yaw=refine_yaw,
            refine_yaw_tolerance_deg=refine_yaw_tolerance_deg,
            refine_yaw_max_error_deg=refine_yaw_max_error_deg,
            refine_yaw_angular_speed_radps=refine_yaw_angular_speed_radps,
            refine_yaw_fine_angular_speed_radps=refine_yaw_fine_angular_speed_radps,
            refine_yaw_timeout_s=refine_yaw_timeout_s,
            refine_yaw_hz=refine_yaw_hz,
            refine_yaw_stable_samples=refine_yaw_stable_samples,
        )

    def fine_position(
        self,
        *,
        final_stop_mm: int,
        final_brake_margin_mm: int,
        final_speed_mps: float,
        max_duration_s: float,
        allow_estop_pedal_fault: bool,
    ):
        """执行料架前方超声精定位。

        使用场景：

        - 机器人已经通过地图导航到 ``GRAB_PRE`` 或 ``PLACE_PRE`` 附近；
        - 前方料架在超声传感器可见范围内；
        - 需要低速向前靠近，并在 ``final_stop_mm`` 附近停车。

        参数说明：

        - ``final_stop_mm``：业务希望最终距离料架的毫米数。
        - ``final_brake_margin_mm``：刹车补偿。底盘发 stop 后仍有惯性，
          所以底层会提前触发停车。
        - ``final_speed_mps``：最后靠近速度，现场建议保持低速。
        - ``max_duration_s``：最长精定位时间，避免传感器异常时一直前进。
        - ``allow_estop_pedal_fault``：是否允许当前项目里已确认的急停踏板
          状态继续执行。这个参数必须由上层按现场安全结论决定。

        注意：

        - 这个方法会真实移动底盘，不是 dry-run。
        - 它委托 ``RackDockingController``，底层仍会做 preflight、超声滤波、
          stop/cancel/close 等保护。
        """

        return self._rack_controller().fine_position(
            final_stop_mm=final_stop_mm,
            final_brake_margin_mm=final_brake_margin_mm,
            final_speed_mps=final_speed_mps,
            max_duration_s=max_duration_s,
            allow_estop_pedal_fault=allow_estop_pedal_fault,
        )

    def retreat(
        self,
        *,
        distance_m: float,
        speed_mps: float,
        allow_estop_pedal_fault: bool,
    ):
        """执行局部后退，带后方超声保护。

        使用场景：

        - LOCAL_PICK 抓完料并完成手臂后撤后，需要底盘退出料架区域；
        - LOCAL_PLACE 放完料并开爪/拉出后，需要底盘退出料架区域。

        参数说明：

        - ``distance_m``：计划后退距离，当前 map20 主流程常用 0.45m。
        - ``speed_mps``：后退速度。
        - ``allow_estop_pedal_fault``：同精定位，必须由上层按现场状态决定。

        返回的 result 里主要看 ``status``：

        - ``completed``：后退完成；
        - ``rear_obstacle``：后方保护触发，某些放料流程允许把它当成安全停止；
        - 其它状态由上层作为失败处理。
        """

        return self._rack_controller().retreat(
            distance_m=distance_m,
            speed_mps=speed_mps,
            allow_estop_pedal_fault=allow_estop_pedal_fault,
        )

    def rack_preflight(self, *, allow_estop_pedal_fault: bool = False):
        """执行 rack 局部动作前的只读 preflight。

        这个检查面向 ``fine_position`` / ``retreat`` 这类局部底盘动作，和
        ``readiness_check`` 的地图导航检查不是同一个层次。

        - ``readiness_check`` 关心 map、SLAM、PNC、odom、站点导航。
        - ``rack_preflight`` 关心局部底盘控制、前后超声、电源、motion
          control 和当前是否能安全发局部速度。

        两者都不应该发运动命令。
        """

        return self._rack_controller().preflight(allow_estop_pedal_fault=allow_estop_pedal_fault)

    def relative_move(
        self,
        *,
        x_m: float,
        y_m: float,
        timeout_s: float,
        max_abs_m: float,
        allow_estop_pedal_fault: bool,
    ) -> dict[str, Any]:
        """执行小范围底盘相对位移。

        坐标约定：

        - ``x_m > 0``：车体坐标系向前；
        - ``x_m < 0``：车体坐标系向后；
        - ``y_m > 0``：车体坐标系向左；
        - ``y_m < 0``：车体坐标系向右。

        这个方法只适合“很小的现场微调”，例如：

        - 精定位后再向前补 0.03m；
        - 放料前横向向右补 0.02m。

        安全策略：

        - 调用前先跑 rack preflight；
        - ``max_abs_m`` 限制 x/y 任一方向最大位移；
        - 发送 ``Pnc.relative_move`` 后轮询 PNC task；
        - 超时或未启动时尝试 cancel，并调用底层 stop；
        - 只有 PNC 进入成功终态才返回 ``status=completed``。

        这个方法会真实移动底盘。业务 demo 里必须额外加命令行确认参数，
        不允许把它作为默认动作。
        """

        self._validate_relative_args(x_m=x_m, y_m=y_m, timeout_s=timeout_s, max_abs_m=max_abs_m)

        # 这两个 import 都可能只在机器人运行环境里存在。保持在方法内部，
        # 让本地 py_compile、文档生成、dry-run 不依赖 GDK 环境。
        RackIndustrialDockingController = self._rack_industrial_controller_class()
        agibot_gdk = self._agibot_gdk_module()

        start = time.time()
        status = "started"
        message = ""
        before_state = None
        before_id = None
        last_state = None
        last_id = None
        final_state = None
        task_id = None
        seen_new_task = False
        seen_running = False

        with RackIndustrialDockingController() as rack:
            # 这里使用 rack controller 的 preflight，而不是 map navigation 的
            # readiness。relative_move 属于局部底盘动作，不依赖地图站点，但
            # 仍然必须确认底盘安全链路和传感器状态。
            preflight = rack.preflight(allow_estop_pedal_fault=allow_estop_pedal_fault)
            if getattr(preflight, "status", None) != "ok":
                raise RuntimeError(f"relative chassis move blocked: {preflight}")

            # 复用 rack.retreat_controller.rear.pnc，是为了沿用已验证过的
            # GDK 初始化和 PNC 实例来源，不在这里另起一套底盘连接逻辑。
            pnc = rack.retreat_controller.rear.pnc
            before_state, before_id = self._clear_non_idle_task_best_effort(pnc)

            # GDK 的 relative_move 需要 NaviReq。这里目标是“相对位移”，不是
            # 地图全局点；orientation 保持单位四元数，表示不额外旋转。
            req = agibot_gdk.NaviReq()
            req.target.position.x = float(x_m)
            req.target.position.y = float(y_m)
            req.target.position.z = 0.0
            req.target.orientation.x = 0.0
            req.target.orientation.y = 0.0
            req.target.orientation.z = 0.0
            req.target.orientation.w = 1.0

            pnc.relative_move(req)
            deadline = time.time() + timeout_s
            while time.time() < deadline:
                time.sleep(0.25)
                try:
                    # 不仅看函数调用是否返回，还要看 PNC task 是否真的出现、
                    # 是否进入运行态、最终是否进入成功终态。否则“命令发出”
                    # 可能被误判成“机器人动完了”。
                    task = pnc.get_task_state()
                    state = getattr(task, "state", None)
                    task_id = getattr(task, "id", None)
                    message = getattr(task, "message", "")
                except Exception as exc:
                    state = None
                    task_id = None
                    message = str(exc)

                if task_id is not None and task_id != before_id:
                    seen_new_task = True
                if state in CHASSIS_RELATIVE_RUNNING_STATES:
                    seen_running = True

                elapsed_s = time.time() - start
                if not seen_new_task and not seen_running:
                    # 有些失败会表现为 relative_move 返回了，但 PNC 迟迟没有
                    # 新任务，也没有运行态。等 4 秒是为了给任务创建一点余量，
                    # 但不能等完整 timeout 才发现根本没启动。
                    if elapsed_s >= 4.0:
                        status = "not_started"
                        break
                    continue

                if state == 7:
                    status = "canceled"
                    final_state = state
                    break
                if state in CHASSIS_RELATIVE_SUCCESS_STATES:
                    status = "completed"
                    final_state = state
                    break

                last_state = state
                last_id = task_id

            if status == "started":
                status = "timeout"
                # 超时后尽量取消 PNC 任务并发 stop。这里是 best-effort，
                # 不能让清理异常覆盖前面的真实失败原因。
                self._cancel_active_task_best_effort(pnc)
                try:
                    rack.retreat_controller.rear.stop()
                except Exception:
                    pass

        payload = {
            "status": status,
            "elapsed_s": time.time() - start,
            "x_m": x_m,
            "y_m": y_m,
            "before_state": before_state,
            "before_task_id": before_id,
            "final_state": final_state,
            "task_id": task_id,
            "last_state": last_state,
            "last_task_id": last_id,
            "message": message,
        }
        if status != "completed":
            raise RuntimeError(f"relative chassis move failed: status={status} message={message}")
        return payload

    def _map_nav_controller(self):
        """懒加载地图导航控制器。

        ``MapNavController`` 会读取 station config，并在 live/readiness 路径
        初始化 GDK。把它延迟到这里，是为了让只导入 ``ChassisMotionController``
        不产生机器人侧副作用。
        """

        if self.station_config_path is None:
            raise ValueError("station_config_path is required for map navigation")
        if self._map_nav is None:
            try:
                from g2_primitives.nav import MapNavController
            except ImportError:
                from rack_hybrid_docking_package.g2_primitives.nav import MapNavController

            self._map_nav = MapNavController(self.station_config_path)
        return self._map_nav

    def _rack_controller(self):
        """懒加载 rack 局部底盘控制器。

        ``RackDockingController`` 最终会触达 ``rack_industrial_docking``，
        后者依赖 ``agibot_gdk``。所以这里必须放在方法内部 import。
        """

        try:
            from g2_primitives.rack import RackDockingController
        except ImportError:
            from rack_hybrid_docking_package.g2_primitives.rack import RackDockingController

        return RackDockingController()

    def _rack_industrial_controller_class(self):
        """仅在 relative_move 需要时导入更底层的工业料架控制器。

        ``fine_position`` 和 ``retreat`` 已经通过 ``RackDockingController``
        封装；``relative_move`` 需要直接拿到底层 PNC 实例，所以这里单独
        获取 ``RackIndustrialDockingController``。
        """

        try:
            from rack_industrial_docking import RackIndustrialDockingController
        except ImportError:
            from rack_hybrid_docking_package.rack_industrial_docking import RackIndustrialDockingController

        return RackIndustrialDockingController

    def _agibot_gdk_module(self):
        """只在真实机器人侧动作里导入 GDK。"""

        import agibot_gdk

        return agibot_gdk

    def _clear_non_idle_task_best_effort(self, pnc) -> tuple[int | None, int | None]:
        """执行 relative_move 前清理可能残留的 PNC 任务。

        为什么要做：

        - 现场调试时，PNC 可能还残留上一次未结束/未清理的任务；
        - 新的 relative_move 如果直接下发，可能被旧任务拒绝；
        - 尝试 cancel 一次可以提高后续动作确定性。

        为什么是 best-effort：

        - 读取 task_state 或 cancel 失败不能说明机器人一定不能动；
        - 真正的安全判断仍由 preflight 和后续 task 轮询负责；
        - 这里返回 None/None，让调用方继续走严格的结果判断。
        """

        try:
            before_task = pnc.get_task_state()
            before_state = getattr(before_task, "state", None)
            before_id = getattr(before_task, "id", None)
            if before_state not in PNC_IDLE_OR_DONE_STATES:
                try:
                    pnc.cancel_task(before_id)
                    time.sleep(0.5)
                except RuntimeError as exc:
                    if "Task is not in RUNNING or PAUSED state" not in str(exc):
                        raise
            return before_state, before_id
        except Exception:
            return None, None

    def _cancel_active_task_best_effort(self, pnc) -> None:
        """取消当前 PNC 任务，用于失败或超时后的清理。"""

        try:
            task = pnc.get_task_state()
            pnc.cancel_task(task.id)
        except Exception:
            pass

    def _validate_relative_args(self, *, x_m: float, y_m: float, timeout_s: float, max_abs_m: float) -> None:
        """校验小范围相对位移参数。

        这里故意只做硬性参数限制，不做现场状态判断。现场状态必须由
        ``relative_move`` 内部 preflight 读取真实机器人状态后决定。
        """

        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        if max_abs_m <= 0.0:
            raise ValueError("max_abs_m must be positive")
        if abs(x_m) > max_abs_m or abs(y_m) > max_abs_m:
            raise ValueError(
                f"relative chassis offset exceeds max_abs_m={max_abs_m:.3f}: "
                f"x_m={x_m:.3f} y_m={y_m:.3f}"
            )
