#!/usr/bin/env python3
"""
G2 料架工业流程控制器。

这个文件是给上层业务集成用的稳定接口。它不重新实现传感器解析，
而是复用已经验证过的三个底层控制器：
  - RackLidarDockingController：前激光雷达粗定位；
  - RackRadarDockingController：前方超声精定位；
  - RackRetreatController：带后方超声保护的后退。

推荐业务直接调用 RackIndustrialDockingController 的这些方法：
  - forward(): 受控前进，带前方超声硬保护；
  - coarse_position(): 激光粗定位，直到前方超声稳定可接管；
  - fine_position(): 前方超声精定位停车；
  - retreat(): 带后方超声保护后退；
  - approach(): 粗定位 + 精定位；
  - cycle(): 后退 + 只读快照 + 粗定位 + 精定位。

在 map20 七根料杆主流程里的实际用法：
  - g2_primitives/rack.py 只调用 fine_position() 和 retreat()；
  - fine_position() 用于抓料/放料前靠近料架到超声阈值；
  - retreat() 用于抓完/放完后退出料架区域；
  - coarse_position()/approach()/cycle() 保留给单独调试或未来更完整流程。

安全边界：
  - 所有会发底盘运动的方法先走 _preflight_result()；
  - 前进/精定位只看前方传感器，后退只看后方传感器；
  - 任何异常退出都会尽量 stop/cancel/close，避免底盘控制权残留。
"""

from dataclasses import dataclass
import statistics
import time

import agibot_gdk

from rack_lidar_docking import RackLidarDockingController
from rack_radar_docking import RackRadarDockingController
from rack_retreat_controller import RackRetreatController


@dataclass(frozen=True)
class SensorSnapshot:
    """一次只读传感器快照。

    这个结构只用于诊断和流程中间确认，不代表可以直接运动。业务仍要看
    preflight、charge、motion-control、PNC 等状态。
    """

    elapsed_s: float
    lidar_distance_m: float | None
    lidar_nearest_m: float | None
    lidar_cluster_points: int
    front_min_mm: int | None
    front_raw: tuple
    rear_min_mm: int | None
    rear_raw: tuple


@dataclass(frozen=True)
class IndustrialStageResult:
    """单个工业阶段的结构化结果。

    status 是上层判断是否继续的主要字段。不要只看 Python 异常；很多安全
    停止会以正常返回的 blocked/rear_obstacle/coarse_guard 表示。
    """

    stage: str
    status: str
    elapsed_s: float
    message: str
    lidar_filtered_m: float | None = None
    front_filtered_mm: int | None = None
    rear_filtered_mm: int | None = None
    samples: int = 0
    detail: object | None = None


@dataclass(frozen=True)
class IndustrialFlowResult:
    """组合流程结果，例如 approach 或 cycle。

    stages 保留每个子阶段原始结果，方便现场回看是哪一层阻断。
    """

    flow: str
    status: str
    stages: tuple


class RackIndustrialDockingController:
    """
    工业级料架流程控制器。

    设计原则：
      1. 所有运动前先做底盘安全检查；
      2. 所有运动都有时间/距离上限；
      3. 前进只发 linear.x，不发横移和旋转；
      4. 粗定位只负责把目标带入前超声稳定接管区；
      5. 精定位只依赖前超声，并使用硬停车阈值抵消滤波延迟；
      6. 后退持续看后方超声，任一硬近障碍立即停车；
      7. 工厂现场常见的单帧丢包、点云跳变、超声偶发回波都必须被滤波或保护。
    """

    def __init__(
        self,
        front_ultrasonic_ids=(0, 1),
        rear_ultrasonic_ids=(4, 5),
        control_mode=0,
        init_gdk=True,
    ):
        self.front_ultrasonic_ids = tuple(front_ultrasonic_ids)
        self.rear_ultrasonic_ids = tuple(rear_ultrasonic_ids)
        self.control_mode = control_mode
        self._init_gdk = init_gdk
        self._closed = False

        if init_gdk:
            result = agibot_gdk.gdk_init()
            gdk_res = getattr(agibot_gdk, "GDKRes", None)
            if gdk_res is not None and result not in (None, gdk_res.kSuccess):
                raise RuntimeError(f"GDK init failed: {result}")

        self.lidar = RackLidarDockingController(
            control_mode=control_mode,
            init_gdk=False,
            init_wait_s=0.5,
        )
        self.front = RackRadarDockingController(
            front_ids=self.front_ultrasonic_ids,
            control_mode=control_mode,
            init_gdk=False,
            init_wait_s=0.2,
        )
        self.retreat_controller = RackRetreatController(
            rear_ultrasonic_ids=self.rear_ultrasonic_ids,
            control_mode=control_mode,
            init_gdk=False,
            init_wait_s=0.2,
        )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def close(self):
        """释放所有底层控制器，并尽量确保底盘停住。"""
        if self._closed:
            return
        for controller in (self.lidar, self.front, self.retreat_controller):
            try:
                controller.close()
            except Exception:
                pass
        if self._init_gdk:
            try:
                agibot_gdk.gdk_release()
            except Exception:
                pass
        self._closed = True

    def _stage_error(self, stage, start, exc):
        """Convert an unexpected exception into a structured stage failure."""

        return IndustrialStageResult(
            stage=stage,
            status="error",
            elapsed_s=time.time() - start,
            message=str(exc),
        )

    def _preflight_result(self, allow_estop_pedal_fault):
        """Aggregate safety checks from lidar/front-ultrasonic/rear-ultrasonic controllers."""

        problems = []
        warnings = []
        for checker in (
            self.lidar,
            self.front,
            self.retreat_controller.rear,
        ):
            current_problems, current_warnings = checker.check_motion_safety(
                allow_estop_pedal_fault=allow_estop_pedal_fault
            )
            problems.extend(current_problems)
            warnings.extend(current_warnings)

        problems = tuple(sorted(set(problems)))
        warnings = tuple(sorted(set(warnings)))
        if problems:
            return IndustrialStageResult(
                stage="preflight",
                status="blocked",
                elapsed_s=0.0,
                message=", ".join(problems),
                detail={"problems": problems, "warnings": warnings},
            )
        return IndustrialStageResult(
            stage="preflight",
            status="ok",
            elapsed_s=0.0,
            message="ok",
            detail={"warnings": warnings},
        )

    def preflight(self, allow_estop_pedal_fault=False):
        """
        只做运动前安全检查，不发底盘速度。

        status:
          - ok：可继续；
          - blocked：存在 charge_plug_insert_state、motion_error、急停等阻断项。
        """
        return self._preflight_result(allow_estop_pedal_fault)

    def read_snapshot(self):
        """只读一帧传感器，不占用底盘控制权。"""
        try:
            lidar_distance = self.lidar.read_rack_distance()
        except RuntimeError:
            # 点云偶发空帧不能阻断基于前超声的精定位/后退流程。
            # 粗定位阶段仍然会在真正需要激光时按自身逻辑处理点云丢失。
            lidar_distance = None
        front_min_mm, front_raw = self.front.read_min_distance()
        rear_min_mm, rear_raw = self.retreat_controller.read_rear_min_distance()

        return SensorSnapshot(
            elapsed_s=0.0,
            lidar_distance_m=None if lidar_distance is None else lidar_distance.distance_m,
            lidar_nearest_m=None if lidar_distance is None else lidar_distance.nearest_m,
            lidar_cluster_points=0 if lidar_distance is None else lidar_distance.cluster_points,
            front_min_mm=front_min_mm,
            front_raw=front_raw,
            rear_min_mm=rear_min_mm,
            rear_raw=rear_raw,
        )

    def read_snapshots(self, samples=8, interval_s=0.2, on_sample=None):
        """只读多帧快照，用于运动前确认现场传感器状态。"""
        if samples <= 0:
            raise ValueError("samples must be positive")
        if interval_s < 0.0:
            raise ValueError("interval_s must be >= 0")

        start = time.time()
        snapshots = []
        for _ in range(samples):
            snapshot = self.read_snapshot()
            snapshot = SensorSnapshot(
                elapsed_s=time.time() - start,
                lidar_distance_m=snapshot.lidar_distance_m,
                lidar_nearest_m=snapshot.lidar_nearest_m,
                lidar_cluster_points=snapshot.lidar_cluster_points,
                front_min_mm=snapshot.front_min_mm,
                front_raw=snapshot.front_raw,
                rear_min_mm=snapshot.rear_min_mm,
                rear_raw=snapshot.rear_raw,
            )
            snapshots.append(snapshot)
            if on_sample is not None:
                on_sample(snapshot)
            time.sleep(interval_s)
        return tuple(snapshots)

    def _append_mm(self, history, value, history_size):
        """Append one millimeter sample and return the rolling median."""

        if value is None:
            return None
        history.append(value)
        del history[:-history_size]
        return int(statistics.median(history))

    def _front_ultrasonic_ready(
        self,
        history,
        filtered_mm,
        consecutive_samples,
        history_size,
        takeover_mm,
        stable_tolerance_mm,
    ):
        """Decide whether front ultrasonic readings are stable enough to take over."""

        if (
            filtered_mm is None
            or len(history) < history_size
            or consecutive_samples < history_size
        ):
            return False
        recent = history[-history_size:]
        if max(recent) - min(recent) > stable_tolerance_mm:
            return False
        return filtered_mm <= takeover_mm

    def forward(
        self,
        distance_m=None,
        duration_s=None,
        speed_mps=0.10,
        front_hard_stop_mm=700,
        hz=10.0,
        allow_estop_pedal_fault=False,
        on_sample=None,
    ):
        """
        受控前进。

        工厂里不建议裸发速度，所以这个方法必须指定 distance_m 或 duration_s
        之一，并且前进过程中持续检查前方超声硬停车阈值。

        典型用途：
          - 调试时小步前进；
          - 上层工艺需要短距离补偿，但仍要保留前方硬保护。
        """
        start = time.time()
        if (distance_m is None) == (duration_s is None):
            raise ValueError("choose exactly one of distance_m or duration_s")
        if speed_mps <= 0.0:
            raise ValueError("speed_mps must be positive")
        if front_hard_stop_mm <= 0:
            raise ValueError("front_hard_stop_mm must be positive")
        if hz <= 0.0:
            raise ValueError("hz must be positive")

        if duration_s is None:
            if distance_m <= 0.0:
                raise ValueError("distance_m must be positive")
            duration_s = distance_m / speed_mps
        elif duration_s <= 0.0:
            raise ValueError("duration_s must be positive")

        preflight = self._preflight_result(allow_estop_pedal_fault)
        if preflight.status != "ok":
            return IndustrialStageResult(
                stage="forward",
                status="blocked",
                elapsed_s=time.time() - start,
                message=preflight.message,
                detail=preflight.detail,
            )

        interval_s = 1.0 / hz
        samples = 0
        latest_min_mm = None
        latest_distances = ()

        try:
            min_mm, distances = self.front.read_min_distance()
            if min_mm is not None and min_mm <= front_hard_stop_mm:
                return IndustrialStageResult(
                    stage="forward",
                    status="front_obstacle",
                    elapsed_s=time.time() - start,
                    message=f"front_min_mm <= {front_hard_stop_mm}",
                    front_filtered_mm=min_mm,
                    samples=1,
                    detail={"front_raw": distances, "before_chassis_control": True},
                )

            self.front.request_chassis_control_ready()
            time.sleep(0.3)
            motion_start = time.time()
            while time.time() - motion_start < duration_s:
                min_mm, distances = self.front.read_min_distance()
                latest_min_mm = min_mm
                latest_distances = distances
                samples += 1

                if on_sample is not None:
                    on_sample(
                        {
                            "stage": "forward",
                            "elapsed_s": time.time() - start,
                            "front_min_mm": min_mm,
                            "front_raw": distances,
                        }
                    )

                if min_mm is not None and min_mm <= front_hard_stop_mm:
                    self.front.stop()
                    return IndustrialStageResult(
                        stage="forward",
                        status="front_obstacle",
                        elapsed_s=time.time() - start,
                        message=f"front_min_mm <= {front_hard_stop_mm}",
                        front_filtered_mm=min_mm,
                        samples=samples,
                        detail={"front_raw": latest_distances},
                    )

                self.front.send_velocity(speed_mps)
                time.sleep(interval_s)

            self.front.stop()
            return IndustrialStageResult(
                stage="forward",
                status="completed",
                elapsed_s=time.time() - start,
                message="completed",
                front_filtered_mm=latest_min_mm,
                samples=samples,
                detail={"front_raw": latest_distances},
            )
        except Exception as exc:
            return self._stage_error("forward", start, exc)
        finally:
            try:
                self.front.stop()
                time.sleep(0.1)
                self.front.cancel_blocking_task()
            except Exception:
                pass

    def coarse_position(
        self,
        coarse_speed_mps=0.60,
        coarse_stop_m=1.6,
        switch_ultrasonic_mm=2200,
        ultrasonic_takeover_mm=2500,
        ultrasonic_stable_tolerance_mm=250,
        max_duration_s=90.0,
        hz=10.0,
        lidar_history_size=3,
        ultrasonic_history_size=3,
        initial_ultrasonic_wait_s=2.0,
        lidar_lost_timeout_s=2.0,
        lidar_dropout_keepalive_s=0.3,
        max_lidar_increase_m=0.8,
        allow_estop_pedal_fault=False,
        on_lidar_sample=None,
        on_front_sample=None,
    ):
        """
        粗定位：用前激光雷达把机器人带到前方超声可稳定接管的位置。

        成功状态不是 stopped，而是 ready_for_fine，表示可以继续调用
        fine_position()。如果激光已经到 coarse_stop_m 但前超声仍不稳定，
        返回 coarse_guard，要求人工检查料架、超声 ID 或现场遮挡。
        """
        start = time.time()
        if coarse_speed_mps <= 0.0:
            raise ValueError("coarse_speed_mps must be positive")
        if coarse_stop_m <= 0.0:
            raise ValueError("coarse_stop_m must be positive")
        if switch_ultrasonic_mm <= 0:
            raise ValueError("switch_ultrasonic_mm must be positive")
        if ultrasonic_takeover_mm <= 0:
            raise ValueError("ultrasonic_takeover_mm must be positive")
        if ultrasonic_stable_tolerance_mm < 0:
            raise ValueError("ultrasonic_stable_tolerance_mm must be >= 0")
        if max_duration_s <= 0.0:
            raise ValueError("max_duration_s must be positive")
        if hz <= 0.0:
            raise ValueError("hz must be positive")
        if lidar_dropout_keepalive_s < 0.0:
            raise ValueError("lidar_dropout_keepalive_s must be >= 0")

        preflight = self._preflight_result(allow_estop_pedal_fault)
        if preflight.status != "ok":
            return IndustrialStageResult(
                stage="coarse_position",
                status="blocked",
                elapsed_s=time.time() - start,
                message=preflight.message,
                detail=preflight.detail,
            )

        ultrasonic_takeover_limit_mm = max(
            switch_ultrasonic_mm,
            ultrasonic_takeover_mm,
        )
        keepalive_min_lidar_m = max(
            coarse_stop_m + 0.4,
            ultrasonic_takeover_limit_mm / 1000.0 + 0.2,
        )

        interval_s = 1.0 / hz
        lidar_history = []
        ultrasonic_history = []
        latest_lidar_filtered = None
        latest_ultrasonic_filtered = None
        ultrasonic_consecutive_samples = 0
        lost_lidar_since = None
        lidar_samples = 0

        try:
            initial_deadline = time.time() + initial_ultrasonic_wait_s
            while time.time() < initial_deadline:
                min_mm, distances = self.front.read_min_distance()
                if min_mm is None:
                    ultrasonic_consecutive_samples = 0
                else:
                    ultrasonic_consecutive_samples += 1
                latest_ultrasonic_filtered = self._append_mm(
                    ultrasonic_history,
                    min_mm,
                    ultrasonic_history_size,
                )
                if on_front_sample is not None and min_mm is not None:
                    on_front_sample(min_mm, latest_ultrasonic_filtered, distances)
                if self._front_ultrasonic_ready(
                    ultrasonic_history,
                    latest_ultrasonic_filtered,
                    ultrasonic_consecutive_samples,
                    ultrasonic_history_size,
                    ultrasonic_takeover_limit_mm,
                    ultrasonic_stable_tolerance_mm,
                ):
                    return IndustrialStageResult(
                        stage="coarse_position",
                        status="ready_for_fine",
                        elapsed_s=time.time() - start,
                        message="front ultrasonic is already stable",
                        front_filtered_mm=latest_ultrasonic_filtered,
                        samples=lidar_samples,
                    )
                time.sleep(0.1)

            self.lidar.request_chassis_control_ready()
            time.sleep(0.3)

            while time.time() - start < max_duration_s:
                elapsed_s = time.time() - start

                min_mm, distances = self.front.read_min_distance()
                if min_mm is None:
                    ultrasonic_consecutive_samples = 0
                else:
                    ultrasonic_consecutive_samples += 1
                latest_ultrasonic_filtered = self._append_mm(
                    ultrasonic_history,
                    min_mm,
                    ultrasonic_history_size,
                )
                if on_front_sample is not None and min_mm is not None:
                    on_front_sample(min_mm, latest_ultrasonic_filtered, distances)
                if self._front_ultrasonic_ready(
                    ultrasonic_history,
                    latest_ultrasonic_filtered,
                    ultrasonic_consecutive_samples,
                    ultrasonic_history_size,
                    ultrasonic_takeover_limit_mm,
                    ultrasonic_stable_tolerance_mm,
                ):
                    self.lidar.stop()
                    return IndustrialStageResult(
                        stage="coarse_position",
                        status="ready_for_fine",
                        elapsed_s=elapsed_s,
                        message="front ultrasonic takeover is stable",
                        lidar_filtered_m=latest_lidar_filtered,
                        front_filtered_mm=latest_ultrasonic_filtered,
                        samples=lidar_samples,
                    )

                lidar_distance = self.lidar.read_rack_distance()
                if lidar_distance is None:
                    if lost_lidar_since is None:
                        lost_lidar_since = elapsed_s
                    lidar_lost_s = elapsed_s - lost_lidar_since
                    can_keepalive = (
                        latest_lidar_filtered is not None
                        and lidar_lost_s <= lidar_dropout_keepalive_s
                        and latest_lidar_filtered > keepalive_min_lidar_m
                    )
                    if can_keepalive:
                        self.lidar.send_velocity(coarse_speed_mps)
                    else:
                        self.lidar.stop()
                    if lidar_lost_s >= lidar_lost_timeout_s:
                        return IndustrialStageResult(
                            stage="coarse_position",
                            status="lost_lidar",
                            elapsed_s=elapsed_s,
                            message="front lidar cluster lost",
                            lidar_filtered_m=latest_lidar_filtered,
                            front_filtered_mm=latest_ultrasonic_filtered,
                            samples=lidar_samples,
                        )
                    time.sleep(interval_s)
                    continue

                lost_lidar_since = None
                if (
                    latest_lidar_filtered is not None
                    and lidar_distance.distance_m > latest_lidar_filtered + max_lidar_increase_m
                ):
                    self.lidar.stop()
                    return IndustrialStageResult(
                        stage="coarse_position",
                        status="target_lost",
                        elapsed_s=elapsed_s,
                        message="lidar target jumped to far background",
                        lidar_filtered_m=latest_lidar_filtered,
                        front_filtered_mm=latest_ultrasonic_filtered,
                        samples=lidar_samples,
                        detail=lidar_distance,
                    )

                lidar_history.append(lidar_distance.distance_m)
                del lidar_history[:-lidar_history_size]
                latest_lidar_filtered = float(statistics.median(lidar_history))
                lidar_samples += 1

                if on_lidar_sample is not None:
                    on_lidar_sample(elapsed_s, lidar_distance, latest_lidar_filtered)

                if (
                    len(lidar_history) >= lidar_history_size
                    and latest_lidar_filtered <= coarse_stop_m
                ):
                    self.lidar.stop()
                    return IndustrialStageResult(
                        stage="coarse_position",
                        status="coarse_guard",
                        elapsed_s=elapsed_s,
                        message="lidar reached coarse guard but front ultrasonic is not stable",
                        lidar_filtered_m=latest_lidar_filtered,
                        front_filtered_mm=latest_ultrasonic_filtered,
                        samples=lidar_samples,
                        detail=lidar_distance,
                    )

                self.lidar.send_velocity(coarse_speed_mps)
                time.sleep(interval_s)

            self.lidar.stop()
            return IndustrialStageResult(
                stage="coarse_position",
                status="timeout",
                elapsed_s=time.time() - start,
                message="coarse positioning timeout",
                lidar_filtered_m=latest_lidar_filtered,
                front_filtered_mm=latest_ultrasonic_filtered,
                samples=lidar_samples,
            )
        except Exception as exc:
            return self._stage_error("coarse_position", start, exc)
        finally:
            try:
                self.lidar.stop()
                time.sleep(0.1)
                self.lidar.cancel_blocking_task()
            except Exception:
                pass

    def fine_position(
        self,
        final_stop_mm=540,
        final_brake_margin_mm=80,
        final_speed_mps=0.30,
        max_duration_s=60.0,
        hz=10.0,
        history_size=3,
        initial_lock_timeout_s=2.0,
        lost_timeout_s=1.0,
        allow_estop_pedal_fault=False,
        on_sample=None,
    ):
        """
        精定位：前方超声闭环停车。

        final_stop_mm 是希望停稳后的业务距离；final_brake_margin_mm 是制动补偿。
        内部触发距离 = final_stop_mm + final_brake_margin_mm，并同时作为 hard_stop_mm。
        """
        start = time.time()
        if final_stop_mm <= 0:
            raise ValueError("final_stop_mm must be positive")
        if final_brake_margin_mm < 0:
            raise ValueError("final_brake_margin_mm must be >= 0")
        if final_speed_mps <= 0.0:
            raise ValueError("final_speed_mps must be positive")

        preflight = self._preflight_result(allow_estop_pedal_fault)
        if preflight.status != "ok":
            return IndustrialStageResult(
                stage="fine_position",
                status="blocked",
                elapsed_s=time.time() - start,
                message=preflight.message,
                detail=preflight.detail,
            )

        final_trigger_mm = final_stop_mm + final_brake_margin_mm
        try:
            result = self.front.approach_until_distance(
                stop_mm=final_trigger_mm,
                speed_mps=final_speed_mps,
                max_duration_s=max_duration_s,
                hz=hz,
                history_size=history_size,
                initial_radar_timeout_s=initial_lock_timeout_s,
                acquire_if_needed=False,
                lost_radar_timeout_s=lost_timeout_s,
                hard_stop_mm=final_trigger_mm,
                allow_estop_pedal_fault=allow_estop_pedal_fault,
                on_sample=on_sample,
            )
            return IndustrialStageResult(
                stage="fine_position",
                status=result.status,
                elapsed_s=result.elapsed_s,
                message=(
                    f"final_target_mm={final_stop_mm}, "
                    f"final_trigger_mm={final_trigger_mm}"
                ),
                front_filtered_mm=result.filtered_mm,
                samples=result.samples,
                detail=result,
            )
        except RuntimeError as exc:
            status = "no_front_ultrasonic_lock"
            if "No stable front radar history" not in str(exc):
                status = "error"
            return IndustrialStageResult(
                stage="fine_position",
                status=status,
                elapsed_s=time.time() - start,
                message=str(exc),
            )
        except Exception as exc:
            return self._stage_error("fine_position", start, exc)

    def retreat(
        self,
        distance_m=2.5,
        speed_mps=0.50,
        rear_stop_mm=700,
        rear_hard_stop_mm=500,
        rear_stop_min_sensors=2,
        rear_start_check_samples=4,
        rear_start_check_interval_s=0.08,
        rear_start_check_confirm_samples=2,
        hz=10.0,
        history_size=3,
        max_duration_s=None,
        allow_estop_pedal_fault=False,
        on_sample=None,
        control_retry_count=2,
        control_retry_wait_s=0.6,
        method="relative",
        success_states=(3, 9),
        allow_motion_control_error_retreat_escape=False,
    ):
        """
        带后方超声保护的正式后退。

        method="relative" 是默认工业模式：提交 Pnc.relative_move(x=-distance_m)
        给底盘导航闭环执行，距离由导航系统闭环完成，不再用速度 * 时间估算。

        method="velocity" 只保留给诊断：用 move_chassis 按 speed_mps 和时间
        开环后退。它不能保证实距，不应作为依赖动作基准的一键流程默认。
        """
        start = time.time()
        preflight = self._preflight_result(allow_estop_pedal_fault)
        if preflight.status != "ok":
            problems = set((preflight.detail or {}).get("problems", ()))
            can_escape = (
                allow_motion_control_error_retreat_escape
                and problems == {"motion_control_error=2"}
            )
            if not can_escape:
                return IndustrialStageResult(
                    stage="retreat",
                    status="blocked",
                    elapsed_s=time.time() - start,
                    message=preflight.message,
                    detail=preflight.detail,
                )

        try:
            if method not in ("relative", "velocity"):
                raise ValueError("retreat method must be 'relative' or 'velocity'")
            if rear_start_check_samples <= 0:
                raise ValueError("rear_start_check_samples must be positive")
            if rear_start_check_interval_s < 0.0:
                raise ValueError("rear_start_check_interval_s must be >= 0")
            if rear_start_check_confirm_samples <= 0:
                raise ValueError("rear_start_check_confirm_samples must be positive")

            precheck_samples = []
            hit_count = 0
            hard_pair_hit = False
            last_min_mm = None
            last_distances = ()
            for index in range(rear_start_check_samples):
                min_mm, distances = self.retreat_controller.read_rear_min_distance()
                last_min_mm = min_mm
                last_distances = distances
                hard_stop_hit = min_mm is not None and min_mm <= rear_hard_stop_mm
                sensors_under_stop = sum(
                    1 for _, distance_mm in distances if distance_mm <= rear_stop_mm
                )
                sensors_under_hard_stop = sum(
                    1 for _, distance_mm in distances if distance_mm <= rear_hard_stop_mm
                )
                stable_stop_hit = (
                    min_mm is not None
                    and min_mm <= rear_stop_mm
                    and sensors_under_stop >= rear_stop_min_sensors
                )
                sample_hit = hard_stop_hit or stable_stop_hit
                if sample_hit:
                    hit_count += 1
                hard_pair_hit = hard_pair_hit or sensors_under_hard_stop >= rear_stop_min_sensors
                precheck_samples.append(
                    {
                        "index": index + 1,
                        "min_mm": min_mm,
                        "rear_raw": distances,
                        "hard_stop_hit": hard_stop_hit,
                        "stable_stop_hit": stable_stop_hit,
                        "sensors_under_stop": sensors_under_stop,
                        "sensors_under_hard_stop": sensors_under_hard_stop,
                    }
                )
                if hard_pair_hit or hit_count >= rear_start_check_confirm_samples:
                    break
                if index + 1 < rear_start_check_samples:
                    time.sleep(rear_start_check_interval_s)
            if hard_pair_hit or hit_count >= rear_start_check_confirm_samples:
                return IndustrialStageResult(
                    stage="retreat",
                    status="rear_obstacle",
                    elapsed_s=time.time() - start,
                    message="rear obstacle before chassis control",
                    rear_filtered_mm=last_min_mm,
                    samples=len(precheck_samples),
                    detail={
                        "rear_raw": last_distances,
                        "before_chassis_control": True,
                        "precheck_samples": precheck_samples,
                        "hit_count": hit_count,
                        "confirm_samples": rear_start_check_confirm_samples,
                        "hard_pair_hit": hard_pair_hit,
                    },
                )

            if method == "relative":
                return self._retreat_relative(
                    start=start,
                    distance_m=distance_m,
                    rear_stop_mm=rear_stop_mm,
                    rear_hard_stop_mm=rear_hard_stop_mm,
                    rear_stop_min_sensors=rear_stop_min_sensors,
                    hz=hz,
                    history_size=history_size,
                    max_duration_s=max_duration_s,
                    success_states=tuple(success_states),
                )

            last_error = None
            for attempt in range(control_retry_count + 1):
                try:
                    result = self.retreat_controller.retreat_distance(
                        distance_m=distance_m,
                        speed_mps=speed_mps,
                        rear_stop_mm=rear_stop_mm,
                        rear_hard_stop_mm=rear_hard_stop_mm,
                        rear_stop_min_sensors=rear_stop_min_sensors,
                        hz=hz,
                        history_size=history_size,
                        max_duration_s=max_duration_s,
                        allow_estop_pedal_fault=allow_estop_pedal_fault,
                        on_sample=on_sample,
                    )
                    break
                except RuntimeError as exc:
                    last_error = exc
                    transient = (
                        "CancelTask failed" in str(exc)
                        or "RequestChassisControl failed" in str(exc)
                    )
                    if not transient or attempt >= control_retry_count:
                        raise
                    try:
                        self.retreat_controller.rear.stop()
                    except Exception:
                        pass
                    time.sleep(control_retry_wait_s)
            else:
                raise last_error

            return IndustrialStageResult(
                stage="retreat",
                status=result.status,
                elapsed_s=result.elapsed_s,
                message="retreat finished",
                rear_filtered_mm=result.rear_filtered_mm,
                samples=result.samples,
                detail=result,
            )
        except Exception as exc:
            return self._stage_error("retreat", start, exc)

    def _retreat_relative(
        self,
        start,
        distance_m,
        rear_stop_mm,
        rear_hard_stop_mm,
        rear_stop_min_sensors,
        hz,
        history_size,
        max_duration_s,
        success_states,
    ):
        """用 Pnc.relative_move(x=-distance_m) 做闭环后退，并实时看后方超声。

        这里和 velocity 后退不同：相对移动由 PNC 闭环执行，脚本负责监控任务
        状态和后方超声。如果后方出现障碍，会 cancel 当前 PNC 任务并发零速度。
        """
        if distance_m <= 0.0:
            raise ValueError("distance_m must be positive")
        if rear_stop_mm <= 0:
            raise ValueError("rear_stop_mm must be positive")
        if rear_hard_stop_mm <= 0:
            raise ValueError("rear_hard_stop_mm must be positive")
        if rear_hard_stop_mm > rear_stop_mm:
            raise ValueError("rear_hard_stop_mm must be <= rear_stop_mm")
        if rear_stop_min_sensors <= 0:
            raise ValueError("rear_stop_min_sensors must be positive")
        if hz <= 0.0:
            raise ValueError("hz must be positive")
        if history_size <= 0:
            raise ValueError("history_size must be positive")

        if max_duration_s is None:
            max_duration_s = max(12.0, distance_m * 15.0)

        pnc = self.retreat_controller.rear.pnc
        before_state = None
        before_id = None
        try:
            before_task = pnc.get_task_state()
            before_state = getattr(before_task, "state", None)
            before_id = getattr(before_task, "id", None)
            if before_state not in (0, 3, 7, 8, 9):
                try:
                    pnc.cancel_task(before_id)
                    time.sleep(0.5)
                except RuntimeError as exc:
                    if "Task is not in RUNNING or PAUSED state" not in str(exc):
                        raise
        except Exception:
            before_id = None

        req = agibot_gdk.NaviReq()
        req.target.position.x = -float(distance_m)
        req.target.position.y = 0.0
        req.target.position.z = 0.0
        req.target.orientation.x = 0.0
        req.target.orientation.y = 0.0
        req.target.orientation.z = 0.0
        req.target.orientation.w = 1.0

        pnc.relative_move(req)

        deadline = time.time() + max_duration_s
        interval_s = 1.0 / hz
        history = []
        latest_min_mm = None
        latest_filtered_mm = None
        latest_distances = ()
        samples = 0
        seen_new_task = False
        seen_running = False
        last_state = None
        last_id = None

        try:
            while time.time() < deadline:
                min_mm, distances = self.retreat_controller.read_rear_min_distance()
                samples += 1
                if min_mm is None:
                    history = []
                    filtered_mm = None
                    distances = ()
                else:
                    history.append(min_mm)
                    history = history[-history_size:]
                    filtered_mm = int(statistics.median(history))
                    latest_min_mm = min_mm
                    latest_filtered_mm = filtered_mm
                    latest_distances = distances

                hard_stop_hit = min_mm is not None and min_mm <= rear_hard_stop_mm
                sensors_under_stop = sum(
                    1 for _, distance_mm in distances if distance_mm <= rear_stop_mm
                )
                stable_stop_hit = (
                    filtered_mm is not None
                    and len(history) >= history_size
                    and filtered_mm <= rear_stop_mm
                    and sensors_under_stop >= rear_stop_min_sensors
                )
                if hard_stop_hit or stable_stop_hit:
                    try:
                        task = pnc.get_task_state()
                        pnc.cancel_task(task.id)
                    except Exception:
                        pass
                    self.retreat_controller.rear.stop()
                    return IndustrialStageResult(
                        stage="retreat",
                        status="rear_obstacle",
                        elapsed_s=time.time() - start,
                        message="relative retreat stopped by rear obstacle",
                        rear_filtered_mm=filtered_mm,
                        samples=samples,
                        detail={
                            "method": "relative",
                            "target_distance_m": distance_m,
                            "rear_raw": distances,
                        },
                    )

                try:
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
                if state in (1, 2, 4, 5, 6, 8):
                    seen_running = True

                elapsed_s = time.time() - start
                if not seen_new_task and not seen_running:
                    if elapsed_s >= 4.0:
                        return IndustrialStageResult(
                            stage="retreat",
                            status="not_started",
                            elapsed_s=elapsed_s,
                            message="relative retreat task did not start",
                            rear_filtered_mm=latest_filtered_mm,
                            samples=samples,
                            detail={
                                "method": "relative",
                                "target_distance_m": distance_m,
                                "before_task_id": before_id,
                                "last_state": state,
                                "last_task_id": task_id,
                                "message": message,
                            },
                        )
                    time.sleep(interval_s)
                    continue

                if state == 7:
                    return IndustrialStageResult(
                        stage="retreat",
                        status="canceled",
                        elapsed_s=elapsed_s,
                        message="relative retreat task canceled before accepted completion",
                        rear_filtered_mm=latest_filtered_mm,
                        samples=samples,
                        detail={
                            "method": "relative",
                            "target_distance_m": distance_m,
                            "before_task_id": before_id,
                            "last_state": state,
                            "last_task_id": task_id,
                            "message": message,
                        },
                    )
                if state in success_states:
                    return IndustrialStageResult(
                        stage="retreat",
                        status="completed",
                        elapsed_s=elapsed_s,
                        message="relative retreat completed",
                        rear_filtered_mm=latest_filtered_mm,
                        samples=samples,
                        detail={
                            "method": "relative",
                            "target_distance_m": distance_m,
                            "final_state": state,
                            "task_id": task_id,
                            "rear_raw": latest_distances,
                        },
                    )

                last_state = state
                last_id = task_id
                time.sleep(interval_s)

            try:
                task = pnc.get_task_state()
                pnc.cancel_task(task.id)
            except Exception:
                pass
            self.retreat_controller.rear.stop()
            return IndustrialStageResult(
                stage="retreat",
                status="timeout",
                elapsed_s=time.time() - start,
                message="relative retreat timed out",
                rear_filtered_mm=latest_filtered_mm,
                samples=samples,
                detail={
                    "method": "relative",
                    "target_distance_m": distance_m,
                    "last_state": last_state,
                    "last_task_id": last_id,
                    "rear_raw": latest_distances,
                },
            )
        finally:
            try:
                self.retreat_controller.rear.stop()
            except Exception:
                pass

    def approach(self, allow_estop_pedal_fault=False, **kwargs):
        """
        粗定位 + 精定位。

        kwargs 可以传给 coarse_position/fine_position，例如：
          coarse_speed_mps=0.45, final_speed_mps=0.20

        七根主流程当前没有调用这个组合方法，因为 map20 已经先用地图导航到
        GRAB_PRE/PLACE_PRE，再只需要近距离超声精定位。
        """
        coarse_keys = {
            "coarse_speed_mps",
            "coarse_stop_m",
            "switch_ultrasonic_mm",
            "ultrasonic_takeover_mm",
            "ultrasonic_stable_tolerance_mm",
            "max_duration_s",
            "hz",
            "lidar_history_size",
            "ultrasonic_history_size",
            "initial_ultrasonic_wait_s",
            "lidar_lost_timeout_s",
            "lidar_dropout_keepalive_s",
            "max_lidar_increase_m",
            "on_lidar_sample",
            "on_front_sample",
        }
        fine_keys = {
            "final_stop_mm",
            "final_brake_margin_mm",
            "final_speed_mps",
            "final_max_duration_s",
            "final_hz",
            "final_history_size",
            "initial_lock_timeout_s",
            "lost_timeout_s",
            "on_final_sample",
        }

        coarse_kwargs = {
            key: value
            for key, value in kwargs.items()
            if key in coarse_keys
        }
        fine_kwargs = {}
        for key, value in kwargs.items():
            if key not in fine_keys:
                continue
            mapped_key = key
            if key == "final_max_duration_s":
                mapped_key = "max_duration_s"
            elif key == "final_hz":
                mapped_key = "hz"
            elif key == "final_history_size":
                mapped_key = "history_size"
            elif key == "on_final_sample":
                mapped_key = "on_sample"
            fine_kwargs[mapped_key] = value

        coarse = self.coarse_position(
            allow_estop_pedal_fault=allow_estop_pedal_fault,
            **coarse_kwargs,
        )
        if coarse.status != "ready_for_fine":
            return IndustrialFlowResult(
                flow="approach",
                status="aborted",
                stages=(coarse,),
            )

        fine = self.fine_position(
            allow_estop_pedal_fault=allow_estop_pedal_fault,
            **fine_kwargs,
        )
        flow_status = "completed" if fine.status in ("stopped", "already_at_threshold") else "aborted"
        return IndustrialFlowResult(
            flow="approach",
            status=flow_status,
            stages=(coarse, fine),
        )

    def cycle(
        self,
        allow_estop_pedal_fault=False,
        snapshot_samples=8,
        snapshot_interval_s=0.2,
        **kwargs,
    ):
        """
        工业循环：后退 -> 只读快照 -> 粗定位 -> 精定位。

        后退未 completed 时不会继续靠近。这样后方有障碍或充电状态未解除时，
        流程会停在安全边界内。

        这是一个更完整的工业循环模板，当前七根主流程只复用其中的基础能力。
        """
        retreat_keys = {
            "retreat_distance_m",
            "retreat_speed_mps",
            "rear_stop_mm",
            "rear_hard_stop_mm",
            "rear_stop_min_sensors",
            "retreat_hz",
            "retreat_history_size",
            "retreat_max_duration_s",
            "on_retreat_sample",
            "retreat_method",
            "retreat_success_states",
            "allow_motion_control_error_retreat_escape",
        }
        retreat_kwargs = {}
        for key, value in kwargs.items():
            if key not in retreat_keys:
                continue
            mapped_key = key
            if key == "retreat_distance_m":
                mapped_key = "distance_m"
            elif key == "retreat_speed_mps":
                mapped_key = "speed_mps"
            elif key == "retreat_hz":
                mapped_key = "hz"
            elif key == "retreat_history_size":
                mapped_key = "history_size"
            elif key == "retreat_max_duration_s":
                mapped_key = "max_duration_s"
            elif key == "on_retreat_sample":
                mapped_key = "on_sample"
            elif key == "retreat_method":
                mapped_key = "method"
            elif key == "retreat_success_states":
                mapped_key = "success_states"
            retreat_kwargs[mapped_key] = value

        retreat = self.retreat(
            allow_estop_pedal_fault=allow_estop_pedal_fault,
            **retreat_kwargs,
        )
        if retreat.status != "completed":
            return IndustrialFlowResult(
                flow="cycle",
                status="aborted",
                stages=(retreat,),
            )

        snapshot_start = time.time()
        snapshots = self.read_snapshots(
            samples=snapshot_samples,
            interval_s=snapshot_interval_s,
        )
        snapshot_elapsed_s = time.time() - snapshot_start

        snapshot_stage = IndustrialStageResult(
            stage="read_snapshot",
            status="completed",
            elapsed_s=snapshot_elapsed_s,
            message=f"{len(snapshots)} snapshots",
            samples=len(snapshots),
            detail=snapshots,
        )

        approach = self.approach(
            allow_estop_pedal_fault=allow_estop_pedal_fault,
            **kwargs,
        )
        status = "completed" if approach.status == "completed" else "aborted"
        return IndustrialFlowResult(
            flow="cycle",
            status=status,
            stages=(retreat, snapshot_stage) + tuple(approach.stages),
        )
