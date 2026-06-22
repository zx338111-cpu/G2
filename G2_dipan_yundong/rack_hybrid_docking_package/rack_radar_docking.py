#!/usr/bin/env python3
"""
G2 料架前方雷达靠近控制类。

这个文件只封装已经实机验证稳定的一条链路：
  request_chassis_control(mode=0) + move_chassis(Twist) + 前方雷达 500mm 停车

使用前必须在机器人端加载 GDK 环境：
  source /home/agi/app/env.sh

典型调用方式：
  from rack_radar_docking import RackRadarDockingController

  with RackRadarDockingController(front_ids=(0, 1)) as dock:
      result = dock.approach_to_rack(
          speed_mps=0.05,
          max_duration_s=80,
          allow_estop_pedal_fault=True,
      )
      print(result)

在 class/import 架构中的位置：
  - RackIndustrialDockingController 会用这个类作为前方超声精定位基础；
  - RackRetreatController 也复用这个类，只是把 selected IDs 换成后方超声；
  - map20 主流程不会直接 import 本文件，而是通过
    g2_primitives.rack.RackDockingController 间接调用。

安全边界：
  - 这个类只发直线 x 方向速度和零速度；
  - 不发横移、不旋转、不做地图导航；
  - 任何靠近动作前都检查充电、急停、超声供电和 motion-control 错误。
"""

from dataclasses import dataclass
import statistics
import time

import agibot_gdk

from gdk_status_utils import read_motion_control_status_with_retry


INVALID_DISTANCE_MM = 65535
DEFAULT_RACK_STOP_MM = 500


@dataclass(frozen=True)
class RadarSample:
    """一次雷达采样，用于 on_sample 回调实时打印/记录。"""

    # 本次靠近启动后的时间，单位秒。
    elapsed_s: float
    # 当前一帧里 front_ids 中的最小有效距离，单位 mm。
    min_mm: int
    # 最近 history_size 个 min_mm 的中位数，真正用于停车判断。
    filtered_mm: int
    # 本次有效雷达原始读数，例如 ((0, 285), (1, 523))。
    distances: tuple


@dataclass(frozen=True)
class DockingResult:
    """一次靠近动作的最终结果。"""

    # stopped/already_at_threshold/timeout/lost_radar/acquire_timeout 之一。
    status: str
    # 动作持续时间，单位秒。
    elapsed_s: float
    # 结束时最后一帧最小距离，单位 mm；丢雷达时为 None。
    min_mm: int | None
    # 结束时最后一次滤波距离，单位 mm；丢雷达时为 None。
    filtered_mm: int | None
    # 结束时最后一帧有效雷达原始读数。
    distances: tuple
    # 本次动作累计处理了多少帧有效雷达数据。
    samples: int


class RackRadarDockingController:
    """
    低速靠近料架，并在前方雷达达到阈值时自动停车。

    这台 G2 上已经验证过的控制约定：
      - 前方雷达 ID：0, 1
      - 底盘控制模式：mode=0
      - Twist.linear.x 为正时，机器人朝当前车头方向前进
      - 当前料架靠近任务默认 stop_mm=500，也就是料架前 0.5m 停车
      - speed_mps=0.05 已实机验证，雷达距离下降稳定
    """

    def __init__(
        self,
        front_ids=(0, 1),
        min_valid_mm=50,
        control_mode=0,
        init_gdk=True,
        init_wait_s=0.5,
    ):
        """
        创建控制器并初始化 GDK 对象。

        front_ids:
            面向料架的前方雷达 ID。现场已确认前方是 0,1。
        min_valid_mm:
            小于这个值的距离认为无效，避免异常近距离噪声。
        control_mode:
            传给 pnc.request_chassis_control() 的底盘模式。官方 move_chassis
            demo 默认用 0，现场验证 mode=0 才是这套前进靠近链路。
        init_gdk:
            True 表示类内部调用 agibot_gdk.gdk_init()/gdk_release()。
            如果你的主程序已经统一初始化 GDK，可以传 False。
        init_wait_s:
            初始化后等待 DDS/GDK 连接稳定的时间。
        """
        # front_ids 为空时无法判断前方距离，直接拒绝初始化。
        if not front_ids:
            raise ValueError("front_ids must not be empty")

        self.front_ids = tuple(front_ids)
        self.min_valid_mm = min_valid_mm
        self.control_mode = control_mode
        self._init_gdk = init_gdk
        self._closed = False

        # 该类既可以单独跑近距离精停，也可以被 hybrid 主类组合使用。
        # 被主类组合时 init_gdk=False，避免重复初始化/释放 GDK。
        if init_gdk:
            result = agibot_gdk.gdk_init()
            gdk_res = getattr(agibot_gdk, "GDKRes", None)
            if gdk_res is not None and result not in (None, gdk_res.kSuccess):
                raise RuntimeError(f"GDK init failed: {result}")

        self.radar = agibot_gdk.UltrasonicRadar()
        self.robot = agibot_gdk.Robot()
        self.pnc = agibot_gdk.Pnc()

        # DDS/GDK 初始化后第一帧数据不一定立刻到，等待一下减少空读。
        time.sleep(init_wait_s)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def close(self):
        """释放资源。会先发零速度，再取消占用底盘的远控任务。"""
        if self._closed:
            return
        try:
            self.stop()
        except Exception:
            pass
        time.sleep(0.1)
        try:
            self.cancel_blocking_task()
        except Exception:
            pass
        try:
            self.radar.close_ultrasonic_radar()
        except Exception:
            pass
        if self._init_gdk:
            try:
                agibot_gdk.gdk_release()
            except Exception:
                pass
        self._closed = True

    def check_motion_safety(self, allow_estop_pedal_fault=False):
        """
        检查靠近前的底盘安全状态。

        返回 (problems, warnings)：
          - problems 非空时拒绝运动
          - warnings 只是提示，例如这台机器的已知急停踏板硬件故障
        """
        problems = []
        warnings = []

        # 底盘电源状态里包含充电插入、急停、超声供电等关键状态。
        # 运动控制状态里包含 motion_error，非 0 时不能继续发速度。
        power = self.robot.get_chassis_power_state()
        try:
            motion = read_motion_control_status_with_retry(self.robot)
        except RuntimeError as exc:
            motion = None
            problems.append(f"motion_control_status_unavailable={exc}")

        if motion is not None and getattr(motion, "error_code", 0) != 0:
            problems.append(f"motion_control_error={motion.error_code}")
        if getattr(power, "charge_plug_insert_state", 0) != 0:
            problems.append("charge_plug_insert_state=1")
        if getattr(power, "emergency_stop_pedal_state", 0) != 0:
            problems.append("emergency_stop_pedal_state!=0")
        if getattr(power, "chassis_ultrasonic_radar_power_state", 0) != 1:
            problems.append("chassis_ultrasonic_radar_power_state!=1")

        return problems, warnings

    def selected_distances(self):
        """读取 front_ids 中所有有效雷达距离，返回 ((id, distance_mm), ...)。"""
        data = self.radar.get_latest_ultrasonic_radar()
        distances = []
        for row in data.get("ultrasonic_radar_datas", []):
            radar_id = row.get("id")
            distance_mm = row.get("distance_mm")
            fault_state = row.get("fault_state")

            # 只保留前方指定 ID。后方超声和侧向超声不能参与前方停车判断。
            if radar_id not in self.front_ids:
                continue

            # fault_state != 0 表示该传感器本帧不可靠。
            if fault_state != 0:
                continue

            # 65535 是无效距离；过小距离也当成噪声过滤掉。
            if distance_mm is None:
                continue
            try:
                distance_mm = int(distance_mm)
            except (TypeError, ValueError):
                continue
            if self.min_valid_mm <= distance_mm < INVALID_DISTANCE_MM:
                distances.append((radar_id, distance_mm))
        return tuple(distances)

    def read_min_distance(self):
        """读取前方最小有效距离，返回 (min_mm, distances)。"""
        distances = self.selected_distances()
        if not distances:
            return None, distances

        # 使用前方多个超声里的最小距离作为安全侧距离。
        # 只要某个前方探头先看到料架，就按最近的那一个停车。
        return min(distance for _, distance in distances), distances

    def _collect_front_history(
        self,
        history_size,
        timeout_s,
        hz,
        on_sample=None,
        start_time=None,
    ):
        """
        静止采样前方雷达，尽量收集 history_size 帧有效数据。

        这个方法只读雷达，不发底盘速度。
        """
        interval_s = 1.0 / hz
        deadline = time.time() + timeout_s
        history = []
        latest_distances = ()

        # 启动前静止采样，不发速度。目的是确认前方超声不是单帧误报。
        while len(history) < history_size and time.time() <= deadline:
            min_mm, distances = self.read_min_distance()
            if min_mm is not None:
                history.append(min_mm)
                latest_distances = distances
                if on_sample is not None:
                    elapsed_s = 0.0 if start_time is None else time.time() - start_time
                    on_sample(
                        RadarSample(
                            elapsed_s=elapsed_s,
                            min_mm=min_mm,
                            filtered_mm=int(statistics.median(history)),
                            distances=distances,
                        )
                    )
            else:
                # 启动锁定阶段必须是连续有效回波。远距离偶发一两帧假回波
                # 不能累积成“稳定目标”，否则会过早切入超声精停后立刻 lost_radar。
                history = []
                latest_distances = ()
            time.sleep(interval_s)

        return history, latest_distances

    def cancel_blocking_task(self):
        """取消 PNC 中未结束的旧任务，释放底盘控制权。"""
        task = self.pnc.get_task_state()
        # 当前实机上 0/3/7/8/9 可视为不需要取消的状态；
        # state=8 是失败/异常终态，GDK 不允许 cancel，不能把它当运行中任务。
        if task.state not in (0, 3, 7, 8, 9):
            try:
                self.pnc.cancel_task(task.id)
                time.sleep(0.3)
            except RuntimeError as exc:
                if "Task is not in RUNNING or PAUSED state" not in str(exc):
                    raise

    def request_chassis_control_ready(self):
        """先清旧任务，再请求底盘远控；如果第一次失败，会清理后重试一次。"""
        self.cancel_blocking_task()
        try:
            # mode=0 是当前验证过的 move_chassis 远控模式。
            return self.pnc.request_chassis_control(self.control_mode)
        except RuntimeError:
            # 如果第一次申请失败，通常是旧任务释放慢，清理后重试一次。
            self.cancel_blocking_task()
            time.sleep(0.5)
            return self.pnc.request_chassis_control(self.control_mode)

    def send_velocity(self, speed_mps):
        """
        发送底盘速度。

        speed_mps > 0：向当前车头方向前进，也就是靠近前方料架。
        speed_mps = 0：停车。
        这个类不发送 lateral/rotation，避免靠近料架时产生横移或转向。
        """
        # 所有非 x 方向都明确置零，保证这个类只做直线前进/停车。
        twist = agibot_gdk.Twist()
        twist.linear = agibot_gdk.Vector3()
        twist.angular = agibot_gdk.Vector3()
        twist.linear.x = speed_mps
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pnc.move_chassis(twist)

    def stop(self):
        """发送零速度停车。"""
        self.send_velocity(0.0)

    def _acquire_front_history(
        self,
        stop_mm,
        acquire_speed_mps,
        acquire_max_distance_m,
        acquire_step_duration_s,
        acquire_timeout_s,
        hz,
        history_size,
        on_sample=None,
    ):
        """
        前方雷达锁定流程。

        如果启动时前方雷达没有稳定回波，就用很小的前进脉冲搜索回波：
          1. 每个脉冲前后都会先发零速度；
          2. 每个脉冲持续时间很短；
          3. 脉冲期间一旦读到前方雷达，立即停车并返回；
          4. 达到内部搜索距离/时间上限仍没有回波，返回 acquire_timeout。

        返回值：
          (history, latest_distances, final_result)

        如果 final_result 不是 None，说明已经产生最终结果，应直接返回给业务。
        如果 final_result 是 None，说明已经锁定前方雷达，调用方可以进入靠近阶段。
        """
        if acquire_speed_mps <= 0.0:
            raise ValueError("acquire_speed_mps must be positive")
        if acquire_max_distance_m < 0.0:
            raise ValueError("acquire_max_distance_m must be >= 0")
        if acquire_step_duration_s <= 0.0:
            raise ValueError("acquire_step_duration_s must be positive")
        if acquire_timeout_s < 0.0:
            raise ValueError("acquire_timeout_s must be >= 0")

        interval_s = 1.0 / hz
        history = []
        latest_distances = ()
        samples = 0
        estimated_distance_m = 0.0
        start = time.time()
        latest_min_mm = None
        latest_filtered_mm = None

        # 小步搜索也需要先拿到底盘控制权。搜索阶段的速度很小，并且每个脉冲
        # 前后都会停车，避免在没有超声回波时长时间盲目前进。
        self.request_chassis_control_ready()
        time.sleep(0.3)

        try:
            while (
                estimated_distance_m < acquire_max_distance_m
                and time.time() - start < acquire_timeout_s
            ):
                # 先静止读几帧。雷达刚好恢复时，不需要再发前进脉冲。
                still_history, still_distances = self._collect_front_history(
                    history_size=history_size,
                    timeout_s=min(0.4, acquire_timeout_s),
                    hz=hz,
                    on_sample=on_sample,
                    start_time=start,
                )
                if still_history:
                    # 静止读到了雷达，就不用继续发搜索脉冲。
                    history.extend(still_history)
                    history = history[-history_size:]
                    latest_distances = still_distances
                    latest_min_mm = min(still_history)
                    latest_filtered_mm = int(statistics.median(history))
                    samples += len(still_history)
                    if min(history) <= stop_mm or latest_filtered_mm <= stop_mm:
                        # 搜索前/搜索中发现已经到阈值内，立即返回已到位。
                        self.stop()
                        return (
                            None,
                            None,
                            DockingResult(
                                status="already_at_threshold",
                                elapsed_s=time.time() - start,
                                min_mm=min(history),
                                filtered_mm=latest_filtered_mm,
                                distances=latest_distances,
                                samples=samples,
                            ),
                        )
                    if len(history) >= history_size:
                        self.stop()
                        return history, latest_distances, None

                pulse_start = time.time()
                while time.time() - pulse_start < acquire_step_duration_s:
                    min_mm, distances = self.read_min_distance()
                    if min_mm is not None:
                        # 脉冲期间一旦读到前方超声，先停车，再判断是否已经到位。
                        history.append(min_mm)
                        history = history[-history_size:]
                        latest_distances = distances
                        latest_min_mm = min_mm
                        latest_filtered_mm = int(statistics.median(history))
                        samples += 1

                        sample = RadarSample(
                            elapsed_s=time.time() - start,
                            min_mm=min_mm,
                            filtered_mm=latest_filtered_mm,
                            distances=distances,
                        )
                        if on_sample is not None:
                            on_sample(sample)

                        self.stop()
                        if min(history) <= stop_mm or latest_filtered_mm <= stop_mm:
                            # 注意这里用 min(history) 或 filtered，保证出现近距离时不会继续走。
                            return (
                                None,
                                None,
                                DockingResult(
                                    status="stopped",
                                    elapsed_s=time.time() - start,
                                    min_mm=min_mm,
                                    filtered_mm=latest_filtered_mm,
                                    distances=distances,
                                    samples=samples,
                                ),
                            )
                        if len(history) >= history_size:
                            # 已经收集到足够稳定的超声历史，交给主靠近循环继续。
                            return history, latest_distances, None

                    # 还没看到超声，就继续本次短脉冲。
                    self.send_velocity(acquire_speed_mps)
                    time.sleep(interval_s)

                self.stop()

                # estimated_distance_m 只是内部盲走保护估计，不作为最终定位依据。
                estimated_distance_m += acquire_speed_mps * acquire_step_duration_s
                time.sleep(0.1)

            self.stop()
            # 搜索到距离/时间上限仍没有锁定前方雷达，返回 acquire_timeout。
            return (
                None,
                None,
                DockingResult(
                    status="acquire_timeout",
                    elapsed_s=time.time() - start,
                    min_mm=latest_min_mm,
                    filtered_mm=latest_filtered_mm,
                    distances=latest_distances,
                    samples=samples,
                ),
            )
        finally:
            # 搜索流程任何退出都补零速度。
            try:
                self.stop()
            except Exception:
                pass

    def approach_until_distance(
        self,
        stop_mm=DEFAULT_RACK_STOP_MM,
        speed_mps=0.02,
        max_duration_s=70.0,
        hz=10.0,
        history_size=3,
        initial_radar_timeout_s=2.0,
        acquire_if_needed=False,
        acquire_speed_mps=0.03,
        acquire_max_distance_m=0.6,
        acquire_step_duration_s=0.4,
        acquire_timeout_s=20.0,
        lost_radar_timeout_s=1.0,
        hard_stop_mm=None,
        hard_stop_consistency_tolerance_mm=120,
        allow_estop_pedal_fault=False,
        on_sample=None,
    ):
        """
        核心调用方法：向前靠近，直到前方雷达滤波距离 <= stop_mm。

        参数说明：
          stop_mm:
              停车阈值，单位 mm。默认 500，表示检测到料架前 0.5m 停车。
          speed_mps:
              前进速度，单位 m/s。现场验证：
                0.02 比较慢，适合第一次调试；
                0.05 是已经验证过的正常速度。
          max_duration_s:
              最长运行时间。到时间还没到 stop_mm，会停车并返回 timeout。
          hz:
              控制和雷达检查频率。10Hz 已实机验证稳定。
          history_size:
              中位数滤波窗口。3 表示最近 3 个 min_mm 取中位数判断停车，
              可以抑制单帧雷达跳变。启动前也会先静止采样 history_size 帧，
              确认没有已经小于 stop_mm 后才请求底盘运动。
          initial_radar_timeout_s:
              启动前最多等待多少秒来收集 history_size 帧有效前方雷达。
              等待期间机器人不移动；少于 history_size 帧时不会进入主靠近段。
          acquire_if_needed:
              True 时，如果启动阶段没有稳定前方雷达，会进入小步搜索流程。
              搜索期间没有雷达就短脉冲前进，脉冲之间停车观察。
          acquire_speed_mps/acquire_max_distance_m/acquire_step_duration_s/acquire_timeout_s:
              前方雷达搜索阶段的内部保护参数。业务层不用传“走多少米”；
              acquire_max_distance_m 只是防止雷达一直无回波时盲目前进。
          lost_radar_timeout_s:
              如果运行中某一帧没有有效前方雷达，类会立即发零速度等待恢复；
              持续超过这个时间仍没有雷达，返回 lost_radar。
          hard_stop_mm:
              原始最小距离小于等于这个值时，不等中位数滤波，立即停车。
              高速精停建议设成 stop_mm，用来减少滤波带来的停车延迟。
          hard_stop_consistency_tolerance_mm:
              使用 hard_stop_mm 时，前方双超声如果只有一颗探头低于阈值，
              且两颗读数差值超过这个容差，认为是边缘/反射跳变，只短暂停车
              观察，不把这一帧写入滤波历史，也不返回 stopped。
          allow_estop_pedal_fault:
              兼容旧参数。当前代码已屏蔽 emergency_stop_pedal_fault_state，
              只用真实 emergency_stop_pedal_state 做急停硬阻断。
          on_sample:
              可选回调函数。每次有有效雷达数据时会调用 on_sample(sample)，
              适合打印日志或写文件。

        返回 DockingResult：
          status="stopped":
              达到 stop_mm，已自动停车。
          status="already_at_threshold":
              启动时已经在阈值内，没有前进。
          status="timeout":
              到 max_duration_s 还没到阈值，已停车。
          status="lost_radar":
              连续丢失前方有效雷达超过 lost_radar_timeout_s，已停车。
          status="acquire_timeout":
              启动时锁不到稳定前方雷达，小步搜索到内部上限仍没有锁定目标，已停车。
        """
        if speed_mps <= 0.0:
            raise ValueError("speed_mps must be positive for front-rack approach")
        if stop_mm < self.min_valid_mm:
            raise ValueError("stop_mm must be >= min_valid_mm")
        if hz <= 0.0:
            raise ValueError("hz must be positive")
        if history_size <= 0:
            raise ValueError("history_size must be positive")
        if initial_radar_timeout_s < 0.0:
            raise ValueError("initial_radar_timeout_s must be >= 0")
        if acquire_max_distance_m < 0.0:
            raise ValueError("acquire_max_distance_m must be >= 0")
        if lost_radar_timeout_s < 0.0:
            raise ValueError("lost_radar_timeout_s must be >= 0")
        if hard_stop_mm is not None and hard_stop_mm < self.min_valid_mm:
            raise ValueError("hard_stop_mm must be >= min_valid_mm")
        if hard_stop_consistency_tolerance_mm < 0:
            raise ValueError("hard_stop_consistency_tolerance_mm must be >= 0")

        # 运动前安全检查。当前 G2 的急停踏板故障可以由参数放行，但
        # 充电、急停按下、运动错误、超声供电异常都不能放行。
        problems, warnings = self.check_motion_safety(
            allow_estop_pedal_fault=allow_estop_pedal_fault
        )
        if problems:
            raise RuntimeError("Refusing to move: " + ", ".join(problems))
        for warning in warnings:
            print("WARNING:", warning)

        interval_s = 1.0 / hz

        # 启动前先静止采样几帧，避免单帧雷达跳高导致本来已经到位却继续前进。
        # 雷达偶尔会短时无有效回波，所以这里最多等待 initial_radar_timeout_s。
        initial_history, initial_distances = self._collect_front_history(
            history_size=history_size,
            timeout_s=initial_radar_timeout_s,
            hz=hz,
        )

        if len(initial_history) < history_size:
            if not acquire_if_needed:
                # 不允许搜索时，启动阶段没有稳定雷达就直接报错，避免盲动。
                raise RuntimeError(
                    f"No stable front radar history for {self.front_ids}: "
                    f"{len(initial_history)}/{history_size}"
                )

            initial_history, initial_distances, final_result = self._acquire_front_history(
                stop_mm=stop_mm,
                acquire_speed_mps=acquire_speed_mps,
                acquire_max_distance_m=acquire_max_distance_m,
                acquire_step_duration_s=acquire_step_duration_s,
                acquire_timeout_s=acquire_timeout_s,
                hz=hz,
                history_size=history_size,
                on_sample=on_sample,
            )
            if final_result is not None:
                # 搜索阶段可能已经判断到位/超时，直接把结果交给上层。
                return final_result

        initial_min_mm = min(initial_history)
        initial_filtered_mm = int(statistics.median(initial_history))
        if initial_min_mm <= stop_mm or initial_filtered_mm <= stop_mm:
            # 启动时已经在阈值内，不请求底盘控制，不继续前进。
            return DockingResult(
                status="already_at_threshold",
                elapsed_s=0.0,
                min_mm=initial_min_mm,
                filtered_mm=initial_filtered_mm,
                distances=initial_distances,
                samples=len(initial_history),
            )

        self.request_chassis_control_ready()
        time.sleep(0.3)

        history = list(initial_history[-history_size:])
        samples = len(history)
        start = time.time()
        latest_min_mm = initial_min_mm
        latest_filtered_mm = initial_filtered_mm
        latest_distances = initial_distances
        lost_radar_since = None

        try:
            while time.time() - start < max_duration_s:
                min_mm, distances = self.read_min_distance()
                elapsed_s = time.time() - start
                if min_mm is None:
                    # 运行中丢失前方超声时，立即发零速度等待恢复。
                    # 持续超过 lost_radar_timeout_s 才返回 lost_radar。
                    self.stop()
                    if lost_radar_since is None:
                        lost_radar_since = elapsed_s
                    if elapsed_s - lost_radar_since >= lost_radar_timeout_s:
                        return DockingResult(
                            status="lost_radar",
                            elapsed_s=elapsed_s,
                            min_mm=None,
                            filtered_mm=None,
                            distances=distances,
                            samples=samples,
                        )
                    time.sleep(interval_s)
                    continue
                lost_radar_since = None

                distance_values = [distance for _, distance in distances]
                unconfirmed_hard_stop_jump = False
                if hard_stop_mm is not None and len(distance_values) >= 2:
                    sensors_under_hard_stop = sum(
                        1 for distance in distance_values if distance <= hard_stop_mm
                    )
                    distance_span_mm = max(distance_values) - min(distance_values)
                    unconfirmed_hard_stop_jump = (
                        min_mm <= hard_stop_mm
                        and sensors_under_hard_stop == 1
                        and distance_span_mm > hard_stop_consistency_tolerance_mm
                    )

                if unconfirmed_hard_stop_jump:
                    # 只一颗前超声突然跳到阈值内，另一颗还差很远时，不能直接
                    # 判定精定位完成。先停车一拍，让下一帧确认是否真实目标。
                    latest_min_mm = min_mm
                    latest_filtered_mm = int(statistics.median(history))
                    latest_distances = distances
                    if on_sample is not None:
                        on_sample(
                            RadarSample(
                                elapsed_s=elapsed_s,
                                min_mm=min_mm,
                                filtered_mm=latest_filtered_mm,
                                distances=distances,
                            )
                        )
                    self.stop()
                    time.sleep(interval_s)
                    continue

                # 使用滑动窗口中位数作为滤波距离，抵抗单帧跳变。
                history.append(min_mm)
                history = history[-history_size:]
                filtered_mm = int(statistics.median(history))
                samples += 1

                latest_min_mm = min_mm
                latest_filtered_mm = filtered_mm
                latest_distances = distances

                sample = RadarSample(
                    elapsed_s=elapsed_s,
                    min_mm=min_mm,
                    filtered_mm=filtered_mm,
                    distances=distances,
                )
                if on_sample is not None:
                    on_sample(sample)

                if hard_stop_mm is not None and min_mm <= hard_stop_mm:
                    # hard_stop 使用原始 min_mm，不等中位数滤波。
                    # 高速精停时这能减少 1~2 帧滤波延迟。
                    self.stop()
                    return DockingResult(
                        status="stopped",
                        elapsed_s=elapsed_s,
                        min_mm=min_mm,
                        filtered_mm=filtered_mm,
                        distances=distances,
                        samples=samples,
                    )

                if len(history) >= history_size and filtered_mm <= stop_mm:
                    # 正常滤波停车条件。
                    self.stop()
                    return DockingResult(
                        status="stopped",
                        elapsed_s=elapsed_s,
                        min_mm=min_mm,
                        filtered_mm=filtered_mm,
                        distances=distances,
                        samples=samples,
                    )

                # 还没到停车阈值，继续向前发送速度。
                self.send_velocity(speed_mps)
                time.sleep(interval_s)

            self.stop()
            # 超时兜底：不管是否到位，先停车并把最后读数返回给上层。
            return DockingResult(
                status="timeout",
                elapsed_s=time.time() - start,
                min_mm=latest_min_mm,
                filtered_mm=latest_filtered_mm,
                distances=latest_distances,
                samples=samples,
            )
        finally:
            # 退出时确保停车并释放 PNC 任务，避免下次 request_chassis_control 被旧任务卡住。
            try:
                self.stop()
            except Exception:
                pass
            time.sleep(0.1)
            try:
                self.cancel_blocking_task()
            except Exception:
                pass

    def approach_to_rack(
        self,
        speed_mps=0.05,
        max_duration_s=80.0,
        hz=10.0,
        history_size=3,
        initial_radar_timeout_s=2.0,
        acquire_speed_mps=0.03,
        acquire_max_distance_m=0.6,
        acquire_step_duration_s=0.4,
        acquire_timeout_s=20.0,
        lost_radar_timeout_s=1.0,
        hard_stop_mm=None,
        hard_stop_consistency_tolerance_mm=120,
        allow_estop_pedal_fault=False,
        on_sample=None,
    ):
        """
        推荐给业务代码调用的入口：自动靠近料架，并在 0.5m 处停车。

        这个方法不需要用户输入“走多少米”。机器人会一直低速向前，
        实时读取前方雷达；当滤波后的最小距离 <= 500mm 时自动停车。

        参数只保留运行策略：
          speed_mps:
              前进速度。0.05m/s 已现场验证；如果第一次调试可改成 0.02。
          max_duration_s:
              最长运行时间，防止料架不在前方时无限前进。
          acquire_speed_mps/acquire_max_distance_m/acquire_step_duration_s/acquire_timeout_s:
              前方雷达锁定参数。锁不到前方雷达时会小步搜索，达到内部上限
              就退出，不会一直盲目前进。
          hz/history_size/initial_radar_timeout_s/lost_radar_timeout_s:
              雷达读取、滤波和丢雷达保护参数，默认值已实机验证。
          hard_stop_mm:
              高速靠近时的原始距离立即停车阈值；默认 None 表示只用滤波阈值。
          hard_stop_consistency_tolerance_mm:
              前方双超声硬停一致性容差，防止单颗探头边缘反射误触发停车。
          allow_estop_pedal_fault:
              当前这台 G2 的已知急停踏板硬件故障需要填 True。
          on_sample:
              可选实时日志回调。
        """
        return self.approach_until_distance(
            stop_mm=DEFAULT_RACK_STOP_MM,
            speed_mps=speed_mps,
            max_duration_s=max_duration_s,
            hz=hz,
            history_size=history_size,
            initial_radar_timeout_s=initial_radar_timeout_s,
            acquire_if_needed=True,
            acquire_speed_mps=acquire_speed_mps,
            acquire_max_distance_m=acquire_max_distance_m,
            acquire_step_duration_s=acquire_step_duration_s,
            acquire_timeout_s=acquire_timeout_s,
            lost_radar_timeout_s=lost_radar_timeout_s,
            hard_stop_mm=hard_stop_mm,
            hard_stop_consistency_tolerance_mm=hard_stop_consistency_tolerance_mm,
            allow_estop_pedal_fault=allow_estop_pedal_fault,
            on_sample=on_sample,
        )
