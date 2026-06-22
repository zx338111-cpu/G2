#!/usr/bin/env python3
"""
G2 料架两段式靠近控制类。

实机验证结论：
  1. 3m 级远距离时，前方超声不一定有回波；
  2. 前激光雷达 raw +X、Z>0.6m 的高处结构点可以用于粗靠近；
  3. 靠近到约 1.3m 后，前方超声 0/1 能稳定读到料架；
  4. 最终 0.5m 精停应交给超声类完成。

这个类把上面两步封装为一个业务入口：approach_to_rack()。
"""

from dataclasses import dataclass
import statistics
import time

import agibot_gdk

from rack_lidar_docking import RackLidarDockingController
from rack_radar_docking import RackRadarDockingController


@dataclass(frozen=True)
class HybridDockingResult:
    """两段式靠近动作的最终结果。"""

    # stopped/already_at_threshold/timeout/lost_lidar/target_lost/coarse_stopped 之一。
    status: str
    # coarse_lidar/final_ultrasonic
    stage: str
    elapsed_s: float
    lidar_filtered_m: float | None
    ultrasonic_filtered_mm: int | None
    final_status: str | None
    coarse_samples: int
    final_samples: int


class RackHybridDockingController:
    """
    远距离前激光雷达粗靠近 + 近距离超声 0.5m 精停。

    推荐业务代码直接调用 approach_to_rack()，不需要输入行走距离。

    内部组合了两个底层控制器：
      - RackLidarDockingController：负责远距离看激光点云；
      - RackRadarDockingController：负责近距离看前方超声波。

    业务层只需要关心“靠近料架并停在目标距离”，不用关心当前到底处在
    激光阶段还是超声阶段。
    """

    def __init__(
        self,
        front_ultrasonic_ids=(0, 1),
        control_mode=0,
        init_gdk=True,
    ):
        self._init_gdk = init_gdk
        self._closed = False

        # init_gdk=True 时，主类负责初始化和释放 GDK。
        # 如果更大的业务程序已经统一初始化 GDK，可以传 init_gdk=False。
        if init_gdk:
            result = agibot_gdk.gdk_init()
            gdk_res = getattr(agibot_gdk, "GDKRes", None)
            if gdk_res is not None and result not in (None, gdk_res.kSuccess):
                raise RuntimeError(f"GDK init failed: {result}")

        # 两个子控制器共用同一次 GDK 初始化，所以这里 init_gdk=False。
        # control_mode=0 是现场验证可用的底盘远控模式。
        self.lidar = RackLidarDockingController(
            control_mode=control_mode,
            init_gdk=False,
            init_wait_s=0.5,
        )
        self.ultrasonic = RackRadarDockingController(
            front_ids=front_ultrasonic_ids,
            control_mode=control_mode,
            init_gdk=False,
            init_wait_s=0.2,
        )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def close(self):
        """释放资源，并确保退出前尽量把底盘速度置零。"""
        if self._closed:
            return
        try:
            self.lidar.stop()
        except Exception:
            pass
        try:
            self.ultrasonic.stop()
        except Exception:
            pass
        try:
            self.lidar.close()
        except Exception:
            pass
        try:
            self.ultrasonic.close()
        except Exception:
            pass
        if self._init_gdk:
            try:
                agibot_gdk.gdk_release()
            except Exception:
                pass
        self._closed = True

    def _read_ultrasonic_min(self):
        """读取前方超声最小有效距离。返回 None 表示本帧无有效回波。"""
        min_mm, distances = self.ultrasonic.read_min_distance()
        return min_mm, distances

    def _append_ultrasonic(self, history, min_mm, history_size):
        """
        维护一个固定长度的超声历史窗口，并返回中位数。

        这里不用平均值，是因为超声偶尔会跳一帧；中位数对单帧跳变更稳。
        """
        if min_mm is None:
            return None
        history.append(min_mm)
        del history[:-history_size]
        return int(statistics.median(history))

    def _ultrasonic_takeover_ready(
        self,
        history,
        filtered_mm,
        consecutive_samples,
        history_size,
        takeover_mm,
        stable_tolerance_mm,
    ):
        """
        判断前方超声是否已经足够稳定，可以优先接管靠近闭环。

        复杂现场里，激光 ROI 可能看到料架局部横梁、反光件或非危险近点，
        但前方超声 0/1 已经稳定看到真正的停车面。此时应把控制权交给
        超声精停，而不是继续让激光近点触发 coarse_stopped。
        """
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

    def _switch_to_ultrasonic_final(
        self,
        start,
        lidar_filtered_m,
        ultrasonic_filtered_mm,
        coarse_samples,
        final_speed_mps,
        final_stop_mm,
        final_brake_margin_mm,
        final_max_duration_s,
        final_hz,
        final_history_size,
        final_lost_timeout_s,
        allow_estop_pedal_fault,
        on_final_sample,
    ):
        self.lidar.stop()

        # final_stop_mm 是“希望停稳后的距离”，例如 500mm。
        # 但底盘在 0.30m/s 精停时有制动惯性，所以要提前触发停车。
        # 当前现场验证 80mm 补偿后，停稳读数能接近 500mm。
        final_trigger_mm = final_stop_mm + final_brake_margin_mm

        # 切到超声精停后，不再依赖激光距离。此时使用超声的硬停车阈值
        # hard_stop_mm，确保原始最小距离一到触发值就立即发零速度。
        result = self.ultrasonic.approach_until_distance(
            stop_mm=final_trigger_mm,
            speed_mps=final_speed_mps,
            max_duration_s=final_max_duration_s,
            hz=final_hz,
            history_size=final_history_size,
            initial_radar_timeout_s=2.0,
            acquire_if_needed=False,
            lost_radar_timeout_s=final_lost_timeout_s,
            hard_stop_mm=final_trigger_mm,
            allow_estop_pedal_fault=allow_estop_pedal_fault,
            on_sample=on_final_sample,
        )
        return HybridDockingResult(
            status=result.status,
            stage="final_ultrasonic",
            elapsed_s=time.time() - start,
            lidar_filtered_m=lidar_filtered_m,
            ultrasonic_filtered_mm=ultrasonic_filtered_mm,
            final_status=result.status,
            coarse_samples=coarse_samples,
            final_samples=result.samples,
        )

    def approach_to_rack(
        self,
        coarse_speed_mps=0.60,
        final_speed_mps=0.30,
        final_stop_mm=540,
        final_brake_margin_mm=80,
        switch_ultrasonic_mm=2200,
        ultrasonic_takeover_mm=2500,
        ultrasonic_stable_tolerance_mm=250,
        coarse_stop_m=1.6,
        coarse_max_duration_s=90.0,
        coarse_hz=10.0,
        coarse_history_size=3,
        ultrasonic_history_size=3,
        coarse_lost_timeout_s=2.0,
        coarse_dropout_keepalive_s=0.3,
        coarse_keepalive_min_lidar_m=None,
        max_lidar_increase_m=0.8,
        final_max_duration_s=60.0,
        final_hz=10.0,
        final_history_size=3,
        final_lost_timeout_s=1.0,
        allow_estop_pedal_fault=False,
        on_coarse_sample=None,
        on_final_sample=None,
    ):
        """
        两段式靠近料架并在 0.5m 停车。

        整体状态机：
          1. 先静止读取前方超声 2 秒；
          2. 如果超声已经稳定且进入接管阈值，直接进入超声精停；
          3. 如果超声还没有稳定回波，请求底盘控制，进入激光粗靠近；
          4. 粗靠近过程中每一轮都优先检查超声，一旦超声稳定就切换；
          5. 如果激光目标丢失太久、目标突然跳远或粗靠近超时，立即停车退出；
          6. 最终由超声精停控制实际停车距离。

        coarse_stop_m:
            激光雷达粗靠近的保护下限。到这个距离还没有稳定超声，就停车。
        switch_ultrasonic_mm:
            前方超声滤波距离小于该值时，切换到超声精停。
        ultrasonic_takeover_mm:
            复杂现场的超声优先接管上限。只要前方超声连续稳定且小于该值，
            就直接交给超声精停，避免激光误识别近处非危险结构后粗停。
            默认 2500mm 覆盖当前现场约 2.2m 边界波动。
        ultrasonic_stable_tolerance_mm:
            判定超声稳定的最近 history_size 帧最大波动，单位 mm。
        final_stop_mm:
            期望停稳后的距离，单位 mm。默认 540 是当前现场复测后更接近
            实际 0.5m 尺量结果的业务目标。
        final_brake_margin_mm:
            精停制动补偿，单位 mm。底盘 0.30m/s 时会有约 70-90mm 惯性，
            所以默认提前 80mm 触发停车，即内部触发距离为 580mm。
        max_lidar_increase_m:
            激光雷达目标突然跳远超过该值，认为目标丢失，停车或切换超声。
        coarse_dropout_keepalive_s:
            粗靠近时，单帧激光点簇丢失允许继续发送原速度的最长时间。
            这样可以避免 0.6m/s 时因为偶发 None 造成“走-停-走”抖动。
        coarse_keepalive_min_lidar_m:
            只有最近一次滤波激光距离大于这个值时才允许 keepalive。
            None 时按 coarse_stop_m 和 switch_ultrasonic_mm 自动计算。
        """
        if coarse_hz <= 0.0:
            raise ValueError("coarse_hz must be positive")
        if switch_ultrasonic_mm <= 0:
            raise ValueError("switch_ultrasonic_mm must be positive")
        if ultrasonic_takeover_mm <= 0:
            raise ValueError("ultrasonic_takeover_mm must be positive")
        if ultrasonic_stable_tolerance_mm < 0:
            raise ValueError("ultrasonic_stable_tolerance_mm must be >= 0")
        if coarse_dropout_keepalive_s < 0.0:
            raise ValueError("coarse_dropout_keepalive_s must be >= 0")
        if final_stop_mm <= 0:
            raise ValueError("final_stop_mm must be positive")
        if final_brake_margin_mm < 0:
            raise ValueError("final_brake_margin_mm must be >= 0")
        # 兼容旧参数：switch_ultrasonic_mm 仍然表示常规切换点；复杂场景下
        # 稳定超声接管允许更宽一点，避免卡在 2.2m 这种边界值上。
        ultrasonic_takeover_limit_mm = max(
            switch_ultrasonic_mm,
            ultrasonic_takeover_mm,
        )
        if coarse_keepalive_min_lidar_m is None:
            # keepalive 只允许在“离料架还比较远”的地方发生。
            # 默认门槛取两个保护值中更保守的一个：
            #   - coarse_stop_m + 0.4：离激光保护下限至少 0.4m；
            #   - ultrasonic_takeover_limit_mm/1000 + 0.2：离超声接管区至少 0.2m。
            coarse_keepalive_min_lidar_m = max(
                coarse_stop_m + 0.4,
                ultrasonic_takeover_limit_mm / 1000.0 + 0.2,
            )

        # 所有运动前先检查底盘状态。当前这台 G2 的急停踏板故障可按现场确认
        # 放行，但充电插入、急停按下、运动错误码等不能放行。
        problems, warnings = self.lidar.check_motion_safety(
            allow_estop_pedal_fault=allow_estop_pedal_fault
        )
        if problems:
            raise RuntimeError("Refusing to move: " + ", ".join(problems))
        for warning in warnings:
            print("WARNING:", warning)

        start = time.time()
        interval_s = 1.0 / coarse_hz
        ultrasonic_history = []
        lidar_history = []
        coarse_samples = 0
        latest_lidar_filtered = None
        latest_ultrasonic_filtered = None
        ultrasonic_consecutive_samples = 0
        lost_lidar_since = None

        # 阶段 1：启动时先静止等前方超声。
        # 从 1m 左右启动时，超声通常已经能看到料架；这时没必要再用激光粗靠近。
        # 从 2~3m 启动时，超声通常没有回波，等待结束后会进入激光粗靠近。
        initial_deadline = time.time() + 2.0
        while time.time() < initial_deadline:
            min_mm, _ = self._read_ultrasonic_min()
            if min_mm is None:
                ultrasonic_consecutive_samples = 0
            else:
                ultrasonic_consecutive_samples += 1
            latest_ultrasonic_filtered = self._append_ultrasonic(
                ultrasonic_history,
                min_mm,
                ultrasonic_history_size,
            )
            if self._ultrasonic_takeover_ready(
                ultrasonic_history,
                latest_ultrasonic_filtered,
                ultrasonic_consecutive_samples,
                ultrasonic_history_size,
                ultrasonic_takeover_limit_mm,
                ultrasonic_stable_tolerance_mm,
            ):
                return self._switch_to_ultrasonic_final(
                    start,
                    latest_lidar_filtered,
                    latest_ultrasonic_filtered,
                    coarse_samples,
                    final_speed_mps,
                    final_stop_mm,
                    final_brake_margin_mm,
                    final_max_duration_s,
                    final_hz,
                    final_history_size,
                    final_lost_timeout_s,
                    allow_estop_pedal_fault,
                    on_final_sample,
                )
            time.sleep(0.1)

        self.lidar.request_chassis_control_ready()
        time.sleep(0.3)

        try:
            # 阶段 2：激光粗靠近循环。
            # 注意：每一轮先看超声，再看激光。这样只要超声稳定出现，就尽快
            # 切到更适合近距离的超声精停。
            while time.time() - start < coarse_max_duration_s:
                elapsed_s = time.time() - start

                min_mm, _ = self._read_ultrasonic_min()
                if min_mm is None:
                    ultrasonic_consecutive_samples = 0
                else:
                    ultrasonic_consecutive_samples += 1
                latest_ultrasonic_filtered = self._append_ultrasonic(
                    ultrasonic_history,
                    min_mm,
                    ultrasonic_history_size,
                )
                if self._ultrasonic_takeover_ready(
                    ultrasonic_history,
                    latest_ultrasonic_filtered,
                    ultrasonic_consecutive_samples,
                    ultrasonic_history_size,
                    ultrasonic_takeover_limit_mm,
                    ultrasonic_stable_tolerance_mm,
                ):
                    return self._switch_to_ultrasonic_final(
                        start,
                        latest_lidar_filtered,
                        latest_ultrasonic_filtered,
                        coarse_samples,
                        final_speed_mps,
                        final_stop_mm,
                        final_brake_margin_mm,
                        final_max_duration_s,
                        final_hz,
                        final_history_size,
                        final_lost_timeout_s,
                        allow_estop_pedal_fault,
                        on_final_sample,
                    )

                lidar_distance = self.lidar.read_rack_distance()
                if lidar_distance is None:
                    # 激光点云偶发一帧没有达到稳定点簇阈值是正常现象。
                    # 早期版本一帧 None 就 stop，0.60m/s 时表现为“走-停-走”抖动。
                    # 现在只在远距离区域允许 0.3s 短暂 keepalive；连续丢失仍停车。
                    if lost_lidar_since is None:
                        lost_lidar_since = elapsed_s
                    lidar_lost_s = elapsed_s - lost_lidar_since
                    can_keepalive = (
                        latest_lidar_filtered is not None
                        and lidar_lost_s <= coarse_dropout_keepalive_s
                        and latest_lidar_filtered > coarse_keepalive_min_lidar_m
                    )
                    if can_keepalive:
                        # 短暂丢一帧且距离还远：继续发同样速度，让底盘运动更平顺。
                        self.lidar.send_velocity(coarse_speed_mps)
                    else:
                        # 离料架较近、没有历史距离或丢失时间超过 keepalive：停车等待/退出。
                        self.lidar.stop()
                    if lidar_lost_s >= coarse_lost_timeout_s:
                        return HybridDockingResult(
                            status="lost_lidar",
                            stage="coarse_lidar",
                            elapsed_s=elapsed_s,
                            lidar_filtered_m=latest_lidar_filtered,
                            ultrasonic_filtered_mm=latest_ultrasonic_filtered,
                            final_status=None,
                            coarse_samples=coarse_samples,
                            final_samples=0,
                        )
                    time.sleep(interval_s)
                    continue

                lost_lidar_since = None
                if latest_lidar_filtered is not None:
                    # 如果本帧距离突然比上一轮滤波值大很多，通常不是料架突然变远，
                    # 而是激光点簇追到了背景或其他物体。此时停车比继续追背景安全。
                    if lidar_distance.distance_m > latest_lidar_filtered + max_lidar_increase_m:
                        self.lidar.stop()
                        return HybridDockingResult(
                            status="target_lost",
                            stage="coarse_lidar",
                            elapsed_s=elapsed_s,
                            lidar_filtered_m=latest_lidar_filtered,
                            ultrasonic_filtered_mm=latest_ultrasonic_filtered,
                            final_status=None,
                            coarse_samples=coarse_samples,
                            final_samples=0,
                        )

                # 激光距离用最近 coarse_history_size 帧取中位数，减少单帧点云跳变。
                lidar_history.append(lidar_distance.distance_m)
                del lidar_history[:-coarse_history_size]
                latest_lidar_filtered = float(statistics.median(lidar_history))
                coarse_samples += 1

                if on_coarse_sample is not None:
                    on_coarse_sample(elapsed_s, lidar_distance, latest_lidar_filtered)

                if (
                    len(lidar_history) >= coarse_history_size
                    and latest_lidar_filtered <= coarse_stop_m
                ):
                    # 已经接近到激光保护下限，但超声仍没稳定出现。
                    # 这不是正常闭环状态，所以停车并把状态返回给上层排查。
                    self.lidar.stop()
                    return HybridDockingResult(
                        status="coarse_stopped",
                        stage="coarse_lidar",
                        elapsed_s=elapsed_s,
                        lidar_filtered_m=latest_lidar_filtered,
                        ultrasonic_filtered_mm=latest_ultrasonic_filtered,
                        final_status=None,
                        coarse_samples=coarse_samples,
                        final_samples=0,
                    )

                # 正常粗靠近：只发前进速度，不发横移/旋转。
                self.lidar.send_velocity(coarse_speed_mps)
                time.sleep(interval_s)

            # 粗靠近超时仍未切到超声，停车退出。
            self.lidar.stop()
            return HybridDockingResult(
                status="timeout",
                stage="coarse_lidar",
                elapsed_s=time.time() - start,
                lidar_filtered_m=latest_lidar_filtered,
                ultrasonic_filtered_mm=latest_ultrasonic_filtered,
                final_status=None,
                coarse_samples=coarse_samples,
                final_samples=0,
            )
        finally:
            # 不管正常返回、异常还是外层中断，都尽量补一条零速度。
            try:
                self.lidar.stop()
            except Exception:
                pass
