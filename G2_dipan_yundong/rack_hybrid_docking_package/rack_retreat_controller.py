#!/usr/bin/env python3
"""
G2 料架流程中的受控后退控制器。

工业现场不能依赖临时 Python 片段来后退。这个类把后退封装成正式能力：
  1. 运动前复用底盘安全检查；
  2. 后退时只发送 Twist.linear.x 负速度；
  3. 持续读取后方超声 4,5；
  4. 后方出现近距离障碍时立即停车；
  5. finally 中强制发零速度并释放底盘控制。

在 class/import 架构中的位置：
  - RackIndustrialDockingController.retreat(method="velocity") 会调用这个类；
  - 当前 map20 主流程默认使用 PNC relative retreat，但仍依赖这里的后方超声
    读取和底盘安全检查能力；
  - 这个文件是“开环速度后退”的保守实现，主要用于诊断和备用路径。
"""

from dataclasses import dataclass
import statistics
import time

from rack_radar_docking import RackRadarDockingController


@dataclass(frozen=True)
class RetreatSample:
    """一次后退过程采样。"""

    elapsed_s: float
    estimated_distance_m: float
    rear_min_mm: int | None
    rear_filtered_mm: int | None
    rear_distances: tuple


@dataclass(frozen=True)
class RetreatResult:
    """一次后退动作的最终结果。"""

    # completed/rear_obstacle/timeout 之一。
    status: str
    elapsed_s: float
    target_distance_m: float
    estimated_distance_m: float
    speed_mps: float
    rear_min_mm: int | None
    rear_filtered_mm: int | None
    rear_distances: tuple
    samples: int


class RackRetreatController:
    """带后方超声保护的直线后退控制器。

    注意：这个类按 speed * time 估算距离，不是地图/里程闭环。需要精确相对
    后退时应优先使用 RackIndustrialDockingController 的 relative 方法。
    """

    def __init__(
        self,
        rear_ultrasonic_ids=(4, 5),
        control_mode=0,
        init_gdk=True,
        init_wait_s=0.5,
    ):
        # 复用已有 GDK/Radar/PNC 封装。这里把 rear_ultrasonic_ids 作为
        # RackRadarDockingController 的 selected IDs 使用。
        self.rear = RackRadarDockingController(
            front_ids=rear_ultrasonic_ids,
            control_mode=control_mode,
            init_gdk=init_gdk,
            init_wait_s=init_wait_s,
        )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def close(self):
        """Close the wrapped radar controller and release its GDK resources."""

        self.rear.close()

    def read_rear_min_distance(self):
        """读取后方超声最小有效距离。None 通常表示后方较空，没有近障碍回波。"""
        return self.rear.read_min_distance()

    def retreat_distance(
        self,
        distance_m=2.5,
        speed_mps=0.50,
        rear_stop_mm=700,
        rear_hard_stop_mm=500,
        rear_stop_min_sensors=2,
        hz=10.0,
        history_size=3,
        max_duration_s=None,
        allow_estop_pedal_fault=False,
        on_sample=None,
    ):
        """
        后退指定距离，并持续检查后方超声。

        distance_m/speed_mps:
            当前 GDK 直线速度接口没有在本项目里接入可靠里程闭环，因此后退距离
            采用速度 * 时间的工程估算。后续如果接入底盘里程计，可替换这里。
        rear_stop_mm:
            后方超声滤波距离小于该距离时停车。用于稳定障碍判定。
        rear_hard_stop_mm:
            任一后方超声原始最小值小于该距离时立即停车。用于近距离硬保护。
        rear_stop_min_sensors:
            稳定障碍判定需要同时低于 rear_stop_mm 的后方探头数量。默认 2，
            避免单个边缘探头看到固定侧向结构时反复打断工业循环。
        history_size:
            后方超声滤波窗口。只用于日志和连续障碍判断；硬停车看原始 min_mm。
        """
        if distance_m <= 0.0:
            raise ValueError("distance_m must be positive")
        if speed_mps <= 0.0:
            raise ValueError("speed_mps must be positive")
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

        target_duration_s = distance_m / speed_mps
        if max_duration_s is None:
            max_duration_s = target_duration_s + 2.0
        if max_duration_s < target_duration_s:
            raise ValueError("max_duration_s must cover distance_m / speed_mps")

        problems, warnings = self.rear.check_motion_safety(
            allow_estop_pedal_fault=allow_estop_pedal_fault
        )
        if problems:
            raise RuntimeError("Refusing to retreat: " + ", ".join(problems))
        for warning in warnings:
            print("WARNING:", warning)

        self.rear.request_chassis_control_ready()
        time.sleep(0.3)

        start = time.time()
        interval_s = 1.0 / hz
        history = []
        latest_min_mm = None
        latest_filtered_mm = None
        latest_distances = ()
        samples = 0

        try:
            while True:
                elapsed_s = time.time() - start
                estimated_distance_m = min(elapsed_s * speed_mps, distance_m)

                if elapsed_s >= max_duration_s:
                    self.rear.stop()
                    return RetreatResult(
                        status="timeout",
                        elapsed_s=elapsed_s,
                        target_distance_m=distance_m,
                        estimated_distance_m=estimated_distance_m,
                        speed_mps=speed_mps,
                        rear_min_mm=latest_min_mm,
                        rear_filtered_mm=latest_filtered_mm,
                        rear_distances=latest_distances,
                        samples=samples,
                    )

                min_mm, distances = self.read_rear_min_distance()
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

                sample = RetreatSample(
                    elapsed_s=elapsed_s,
                    estimated_distance_m=estimated_distance_m,
                    rear_min_mm=min_mm,
                    rear_filtered_mm=filtered_mm,
                    rear_distances=distances,
                )
                samples += 1
                if on_sample is not None:
                    on_sample(sample)

                hard_stop_hit = min_mm is not None and min_mm <= rear_hard_stop_mm
                sensors_under_stop = sum(
                    1 for _, distance_mm in distances if distance_mm <= rear_stop_mm
                )
                filtered_stop_hit = (
                    filtered_mm is not None
                    and len(history) >= history_size
                    and filtered_mm <= rear_stop_mm
                    and sensors_under_stop >= rear_stop_min_sensors
                )
                if hard_stop_hit or filtered_stop_hit:
                    self.rear.stop()
                    return RetreatResult(
                        status="rear_obstacle",
                        elapsed_s=elapsed_s,
                        target_distance_m=distance_m,
                        estimated_distance_m=estimated_distance_m,
                        speed_mps=speed_mps,
                        rear_min_mm=min_mm,
                        rear_filtered_mm=filtered_mm,
                        rear_distances=distances,
                        samples=samples,
                    )

                if elapsed_s >= target_duration_s:
                    self.rear.stop()
                    return RetreatResult(
                        status="completed",
                        elapsed_s=elapsed_s,
                        target_distance_m=distance_m,
                        estimated_distance_m=distance_m,
                        speed_mps=speed_mps,
                        rear_min_mm=latest_min_mm,
                        rear_filtered_mm=latest_filtered_mm,
                        rear_distances=latest_distances,
                        samples=samples,
                    )

                # 后退方向是负 x；不发送横移和旋转。
                self.rear.send_velocity(-speed_mps)
                time.sleep(interval_s)
        finally:
            try:
                self.rear.stop()
                time.sleep(0.2)
                self.rear.stop()
            except Exception:
                pass
