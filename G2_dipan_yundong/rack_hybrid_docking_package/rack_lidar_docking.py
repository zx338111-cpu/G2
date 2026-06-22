#!/usr/bin/env python3
"""
G2 前激光雷达料架靠近控制类。

这个类用于比超声波更远的距离段：
  前激光雷达点云检测料架/障碍物距离 + request_chassis_control(mode=0)
  + move_chassis(Twist) + 500mm 停车

当前现场点云坐标验证：
  - 前激光雷达 raw 点云中，底盘前进方向对应 +X
  - Y 是横向，Z 是高度
  - 料架高处结构在 +X、Z>0.6m 的 ROI 中能看到稳定点

使用前必须在机器人端加载 GDK 环境：
  source /home/agi/app/env.sh

在 class/import 架构中的位置：
  - RackIndustrialDockingController 会组合这个类做远距离粗定位；
  - map20 七根主流程当前主要使用超声 fine_position 和 retreat，不直接调用
    本文件的 coarse lidar 靠近；
  - 保留本文件是为了之后需要从更远处自动靠近料架时复用已验证的 ROI/点簇逻辑。
"""

from dataclasses import dataclass
import math
import statistics
import time

import agibot_gdk
import numpy as np

from gdk_status_utils import read_motion_control_status_with_retry


DEFAULT_RACK_STOP_M = 0.5
DEFAULT_NEAREST_SAFETY_STOP_M = 0.0


@dataclass(frozen=True)
class LidarRackDistance:
    """一帧点云中提取出来的前方料架距离。"""

    # 前方最近稳定点簇距离，单位 m。主停车判断使用这个值的历史中位数。
    distance_m: float
    # ROI 内最近单点距离，单位 m。只做近距离安全停车参考。
    nearest_m: float
    # 被选中点簇的点数。
    cluster_points: int
    # 整个前方 ROI 内有效点数。
    roi_points: int
    # 被选中点簇所在的前向距离区间。
    bin_start_m: float
    bin_end_m: float


@dataclass(frozen=True)
class LidarRackPose:
    """一帧点云中估计出的料架相对姿态，只用于监控/纠偏决策输入。"""

    # 料架最近稳定前向距离，单位 m。
    distance_m: float
    # 点簇横向中心，单位 m；0 表示车身中心线附近。
    lateral_center_m: float
    # 料架面相对车身横向轴的角度估计。0 表示基本正对；None 表示点簇横向跨度不足。
    yaw_deg: float | None
    # 0~1 的粗略置信度，只用于日志筛选，不作为安全证明。
    confidence: float
    # 被选中点簇的点数。
    cluster_points: int
    # 整个前方 ROI 内有效点数。
    roi_points: int
    # 点簇横向 5%~95% 跨度，单位 m。
    lateral_span_m: float
    # 拟合残差中位数，单位 m；None 表示未做直线拟合。
    fit_residual_m: float | None
    # 被选中点簇所在的前向距离区间。
    bin_start_m: float
    bin_end_m: float


@dataclass(frozen=True)
class LidarRackSample:
    """一次有效激光雷达采样，用于实时打印/记录。"""

    elapsed_s: float
    distance_m: float
    filtered_m: float
    nearest_m: float
    cluster_points: int
    roi_points: int
    bin_start_m: float
    bin_end_m: float


@dataclass(frozen=True)
class LidarDockingResult:
    """一次激光雷达靠近动作的最终结果。"""

    # stopped/already_at_threshold/timeout/lost_lidar 之一。
    status: str
    elapsed_s: float
    distance_m: float | None
    filtered_m: float | None
    nearest_m: float | None
    cluster_points: int
    roi_points: int
    samples: int


class RackLidarDockingController:
    """
    用前激光雷达点云靠近料架，并在约 0.5m 自动停车。

    适用场景：
      - 机器人离料架 1m 以上，超声波前方没有稳定回波；
      - 需要从约 3m 粗靠近到料架前 0.5m；
      - 料架在机器人正前方，没有要求横向自动对齐。

    距离提取逻辑：
      1. 取前激光雷达点云；
      2. 只保留前方 +X、横向 |Y| 小于 lateral_half_width_m、高度在
         [z_min_m, z_max_m] 的点；
      3. 沿 +X 分箱，选择最近的、点数达到 min_cluster_points 的稳定点簇；
      4. 对连续 history_size 帧距离取中位数后停车。
    """

    def __init__(
        self,
        control_mode=0,
        forward_axis="x",
        lateral_axis="y",
        forward_sign=1.0,
        lateral_half_width_m=0.8,
        z_min_m=0.6,
        z_max_m=1.2,
        min_range_m=0.8,
        max_range_m=6.0,
        bin_width_m=0.25,
        min_cluster_points=20,
        init_gdk=True,
        init_wait_s=1.0,
    ):
        """
        创建控制器并初始化 GDK 对象。

        control_mode:
            传给 pnc.request_chassis_control() 的底盘模式。现场验证 mode=0 可用。
        forward_axis/lateral_axis/forward_sign:
            点云坐标轴约定。现场用短距离运动确认：底盘 linear.x 前进时，
            前方目标应按 raw +X 方向处理，横向为 raw Y。
        lateral_half_width_m:
            前方 ROI 横向半宽。0.8 表示只看车头中线左右 0.8m。
        z_min_m/z_max_m:
            高度过滤，默认只看料架较高结构，排除地面和车体近点。
        min_range_m/max_range_m:
            前向距离过滤范围。当前现场会稳定出现约 0.65m 的近处非危险点簇，
            默认从 0.8m 开始看，避免粗靠近阶段误把它当料架。
        bin_width_m:
            前向点簇分箱宽度。0.25m 能过滤少量孤立点。
        min_cluster_points:
            最近点簇至少需要的点数，避免把少量边缘点当料架。
        init_gdk:
            True 表示类内部调用 agibot_gdk.gdk_init()/gdk_release()。
        init_wait_s:
            初始化后等待 DDS/GDK 连接稳定的时间。
        """
        # 这些参数直接决定点云 ROI 和分箱算法，配置错误会导致识别不到料架，
        # 所以初始化时先做显式校验，尽早暴露问题。
        if lateral_half_width_m <= 0.0:
            raise ValueError("lateral_half_width_m must be positive")
        if z_min_m >= z_max_m:
            raise ValueError("z_min_m must be < z_max_m")
        if min_range_m <= 0.0:
            raise ValueError("min_range_m must be positive")
        if min_range_m >= max_range_m:
            raise ValueError("min_range_m must be < max_range_m")
        if bin_width_m <= 0.0:
            raise ValueError("bin_width_m must be positive")
        if min_cluster_points <= 0:
            raise ValueError("min_cluster_points must be positive")
        if forward_axis not in ("x", "y", "z"):
            raise ValueError("forward_axis must be x, y, or z")
        if lateral_axis not in ("x", "y", "z"):
            raise ValueError("lateral_axis must be x, y, or z")
        if lateral_axis == forward_axis:
            raise ValueError("lateral_axis must differ from forward_axis")
        if forward_sign not in (-1.0, 1.0):
            raise ValueError("forward_sign must be 1.0 or -1.0")

        self.control_mode = control_mode
        self.forward_axis = forward_axis
        self.lateral_axis = lateral_axis
        self.forward_sign = forward_sign
        self.lateral_half_width_m = lateral_half_width_m
        self.z_min_m = z_min_m
        self.z_max_m = z_max_m
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        self.bin_width_m = bin_width_m
        self.min_cluster_points = min_cluster_points
        self._init_gdk = init_gdk
        self._closed = False

        # 该类可以独立运行，也可以被 RackHybridDockingController 组合使用。
        # 独立运行时 init_gdk=True；被主类组合时 init_gdk=False，避免重复初始化。
        if init_gdk:
            result = agibot_gdk.gdk_init()
            gdk_res = getattr(agibot_gdk, "GDKRes", None)
            if gdk_res is not None and result not in (None, gdk_res.kSuccess):
                raise RuntimeError(f"GDK init failed: {result}")

        self.lidar = agibot_gdk.Lidar()
        self.robot = agibot_gdk.Robot()
        self.pnc = agibot_gdk.Pnc()

        # GDK 对象创建后，DDS 订阅不一定立刻有数据，给它一点连接时间。
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
            self.lidar.close_lidar()
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

        # get_chassis_power_state 检查充电、急停、传感器供电等硬件状态。
        # get_motion_control_status 检查运动控制层有没有错误码。
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
        if getattr(power, "emergency_stop_pedal_fault_state", 0) != 0:
            if allow_estop_pedal_fault:
                warnings.append("emergency_stop_pedal_fault_state=1 allowed")
            else:
                problems.append("emergency_stop_pedal_fault_state=1")

        return problems, warnings

    def cancel_blocking_task(self):
        """取消 PNC 中未结束的旧任务，释放底盘控制权。"""
        task = self.pnc.get_task_state()
        # 0/3/7/8/9 是当前实机上常见的非运行/结束类状态，不需要 cancel。
        # state=8 是失败/异常终态，GDK 不允许 cancel，不能把它当运行中任务。
        # 如果仍有 RUNNING 或 PAUSED 的旧任务，先取消，避免 request_chassis_control 失败。
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
            # mode=0 是现场验证能让 move_chassis 生效的远控模式。
            return self.pnc.request_chassis_control(self.control_mode)
        except RuntimeError:
            # PNC 偶尔会因为旧任务状态未释放而第一次失败，清理后重试一次。
            self.cancel_blocking_task()
            time.sleep(0.5)
            return self.pnc.request_chassis_control(self.control_mode)

    def send_velocity(self, speed_mps):
        """
        发送底盘速度。

        speed_mps > 0：向当前车头方向前进。
        speed_mps = 0：停车。
        这个类不发送 lateral/rotation，避免靠近料架时产生横移或转向。
        """
        # GDK 的 Twist 需要显式填 linear 和 angular，否则不同版本绑定里
        # 默认字段可能为空对象。
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

    def _parse_xyz(self, pointcloud):
        """把 GDK 点云解析成 x/y/z 三个 numpy 数组。"""
        # GDK 返回的是 PointCloud2 风格数据：一大段 bytes + fields 描述。
        # 这里不按字符串解析，而是根据每个 field 的 offset 从二进制中取 float32。
        if pointcloud is None or not hasattr(pointcloud, "data"):
            return None
        if pointcloud.point_step <= 0:
            return None

        raw = pointcloud.data
        # 有些 GDK 版本 data 已经是 numpy 数组，有些是 bytes；统一转成 uint8。
        raw = raw.astype(np.uint8) if isinstance(raw, np.ndarray) else np.frombuffer(raw, dtype=np.uint8)
        point_count = len(raw) // pointcloud.point_step
        if point_count <= 0:
            return None

        raw = raw[: point_count * pointcloud.point_step].reshape(
            (point_count, pointcloud.point_step)
        )

        values = {}
        for field in pointcloud.fields:
            # 当前距离估计只需要 xyz 三个字段，其他 intensity/timestamp 等字段忽略。
            if field.name not in ("x", "y", "z"):
                continue
            if field.offset + 4 > pointcloud.point_step:
                continue
            # 每个 field 是点结构体里的 4 字节 float32。ascontiguousarray 是为了
            # 避免 view(np.float32) 读非连续内存时报错或得到错误结果。
            field_raw = np.ascontiguousarray(raw[:, field.offset : field.offset + 4])
            values[field.name] = field_raw.view(np.float32).reshape(-1)

        if not all(name in values for name in ("x", "y", "z")):
            return None
        return values["x"], values["y"], values["z"]

    def _read_rack_cluster(
        self,
        min_range_m=None,
        max_range_m=None,
        lateral_half_width_m=None,
        z_min_m=None,
        z_max_m=None,
        bin_width_m=None,
        min_cluster_points=None,
    ):
        """Read one pointcloud and extract the nearest stable rack-like cluster."""

        pointcloud = self.lidar.get_latest_pointcloud(
            agibot_gdk.LidarType.kLidarFront,
            1000.0,
        )
        xyz = self._parse_xyz(pointcloud)
        if xyz is None:
            return None

        x, y, z = xyz
        coords = {"x": x, "y": y, "z": z}

        # forward_sign 允许适配不同坐标约定。当前现场验证前方是 raw +X，
        # 所以默认 forward_axis="x", forward_sign=1.0。
        forward_coord = self.forward_sign * coords[self.forward_axis]
        lateral_coord = coords[self.lateral_axis]
        vertical_coord = z

        # ROI 过滤：
        #   - 只看车头前方一定距离内的点；
        #   - 只看车身中线附近，避免旁边墙/人/其他架子干扰；
        #   - 只看 0.6~1.2m 高处结构，避开地面和车体近点。
        effective_min_range_m = self.min_range_m if min_range_m is None else min_range_m
        effective_max_range_m = self.max_range_m if max_range_m is None else max_range_m
        effective_lateral_half_width_m = (
            self.lateral_half_width_m
            if lateral_half_width_m is None
            else lateral_half_width_m
        )
        effective_z_min_m = self.z_min_m if z_min_m is None else z_min_m
        effective_z_max_m = self.z_max_m if z_max_m is None else z_max_m
        effective_bin_width_m = self.bin_width_m if bin_width_m is None else bin_width_m
        effective_min_cluster_points = (
            self.min_cluster_points
            if min_cluster_points is None
            else min_cluster_points
        )
        if effective_min_range_m <= 0.0:
            raise ValueError("min_range_m must be positive")
        if effective_min_range_m >= effective_max_range_m:
            raise ValueError("min_range_m must be < max_range_m")
        if effective_lateral_half_width_m <= 0.0:
            raise ValueError("lateral_half_width_m must be positive")
        if effective_z_min_m >= effective_z_max_m:
            raise ValueError("z_min_m must be < z_max_m")
        if effective_bin_width_m <= 0.0:
            raise ValueError("bin_width_m must be positive")
        if effective_min_cluster_points <= 0:
            raise ValueError("min_cluster_points must be positive")

        valid = (
            np.isfinite(forward_coord)
            & np.isfinite(lateral_coord)
            & np.isfinite(vertical_coord)
            & (forward_coord >= effective_min_range_m)
            & (forward_coord <= effective_max_range_m)
            & (np.abs(lateral_coord) <= effective_lateral_half_width_m)
            & (vertical_coord >= effective_z_min_m)
            & (vertical_coord <= effective_z_max_m)
        )

        forward = forward_coord[valid]
        lateral = lateral_coord[valid]
        if len(forward) < effective_min_cluster_points:
            # ROI 内点太少，不足以认为看到了一个稳定实体。
            return None

        nearest_m = float(np.min(forward))

        # 沿前进方向分箱，找“最近的稳定距离段”。
        # 这样比直接取最近点更稳，因为最近单点可能是噪声/反光/车体边缘。
        bins = np.arange(
            effective_min_range_m,
            effective_max_range_m + effective_bin_width_m,
            effective_bin_width_m,
        )
        counts, edges = np.histogram(forward, bins=bins)

        selected_index = None
        for index, count in enumerate(counts):
            if count >= effective_min_cluster_points:
                # 从近到远找到第一个点数足够的箱，认为它是最近实体边界。
                selected_index = index
                break

        if selected_index is None:
            return None

        bin_start = float(edges[selected_index])
        bin_end = float(edges[selected_index + 1])
        in_cluster = (forward >= bin_start) & (forward < bin_end)
        cluster_forward = forward[in_cluster]
        cluster_lateral = lateral[in_cluster]
        if len(cluster_forward) < effective_min_cluster_points:
            # 理论上 histogram 已经保证点数，这里再防一次边界条件。
            return None

        return {
            "forward": cluster_forward,
            "lateral": cluster_lateral,
            "nearest_m": nearest_m,
            "roi_points": int(len(forward)),
            "bin_start_m": bin_start,
            "bin_end_m": bin_end,
            "min_cluster_points": effective_min_cluster_points,
        }

    def read_rack_distance(self):
        """
        读取前激光雷达，并返回前方最近稳定点簇距离。

        返回 None 表示本帧没有足够稳定的前方点簇。
        """
        cluster = self._read_rack_cluster()
        if cluster is None:
            return None

        cluster_forward = cluster["forward"]
        # 用点簇内 10 分位距离，比 min 抗噪，又比 median 更贴近最近实体边界。
        distance_m = float(np.percentile(cluster_forward, 10))
        return LidarRackDistance(
            distance_m=distance_m,
            nearest_m=cluster["nearest_m"],
            cluster_points=int(len(cluster_forward)),
            roi_points=cluster["roi_points"],
            bin_start_m=cluster["bin_start_m"],
            bin_end_m=cluster["bin_end_m"],
        )

    def read_rack_pose(
        self,
        min_range_m=None,
        max_range_m=None,
        lateral_half_width_m=None,
        z_min_m=None,
        z_max_m=None,
        bin_width_m=None,
        min_cluster_points=None,
    ):
        """
        读取前激光雷达，并估计料架相对车身的距离、横向中心和 yaw。

        这个结果只作为“居中监控/后续纠偏输入”，不替代超声安全停车。
        返回 None 表示本帧没有足够稳定的前方点簇。
        """
        cluster = self._read_rack_cluster(
            min_range_m=min_range_m,
            max_range_m=max_range_m,
            lateral_half_width_m=lateral_half_width_m,
            z_min_m=z_min_m,
            z_max_m=z_max_m,
            bin_width_m=bin_width_m,
            min_cluster_points=min_cluster_points,
        )
        if cluster is None:
            return None

        cluster_forward = cluster["forward"]
        cluster_lateral = cluster["lateral"]

        distance_m = float(np.percentile(cluster_forward, 10))
        lateral_center_m = float(np.median(cluster_lateral))
        lateral_p05 = float(np.percentile(cluster_lateral, 5))
        lateral_p95 = float(np.percentile(cluster_lateral, 95))
        lateral_span_m = max(0.0, lateral_p95 - lateral_p05)

        yaw_deg = None
        fit_residual_m = None
        effective_min_cluster_points = int(cluster["min_cluster_points"])
        if lateral_span_m >= 0.08 and len(cluster_lateral) >= max(6, effective_min_cluster_points // 2):
            slope, intercept = np.polyfit(cluster_lateral, cluster_forward, 1)
            predicted = slope * cluster_lateral + intercept
            residual = np.abs(cluster_forward - predicted)
            fit_residual_m = float(np.median(residual))
            yaw_deg = float(math.degrees(math.atan(float(slope))))

        point_score = min(1.0, len(cluster_forward) / float(effective_min_cluster_points * 3))
        span_score = min(1.0, max(0.0, (lateral_span_m - 0.10) / 0.40))
        residual_score = 0.35
        if fit_residual_m is not None:
            residual_score = min(1.0, max(0.0, 1.0 - fit_residual_m / 0.08))
        confidence = max(
            0.0,
            min(1.0, 0.40 * point_score + 0.30 * span_score + 0.30 * residual_score),
        )

        return LidarRackPose(
            distance_m=distance_m,
            lateral_center_m=lateral_center_m,
            yaw_deg=yaw_deg,
            confidence=confidence,
            cluster_points=int(len(cluster_forward)),
            roi_points=cluster["roi_points"],
            lateral_span_m=lateral_span_m,
            fit_residual_m=fit_residual_m,
            bin_start_m=cluster["bin_start_m"],
            bin_end_m=cluster["bin_end_m"],
        )

    def _collect_history(self, history_size, timeout_s, hz, on_sample=None, start_time=None):
        """静止采样前激光雷达，尽量收集 history_size 帧有效点簇距离。"""
        interval_s = 1.0 / hz
        deadline = time.time() + timeout_s
        history = []
        latest = None

        # 启动前先静止采样，是为了确认目标不是偶然出现的一帧噪声。
        # 只有收集到 history_size 帧有效点簇，才允许进入运动阶段。
        while len(history) < history_size and time.time() <= deadline:
            distance = self.read_rack_distance()
            if distance is not None:
                latest = distance
                history.append(distance.distance_m)
                if on_sample is not None:
                    elapsed_s = 0.0 if start_time is None else time.time() - start_time
                    on_sample(
                        LidarRackSample(
                            elapsed_s=elapsed_s,
                            distance_m=distance.distance_m,
                            filtered_m=float(statistics.median(history)),
                            nearest_m=distance.nearest_m,
                            cluster_points=distance.cluster_points,
                            roi_points=distance.roi_points,
                            bin_start_m=distance.bin_start_m,
                            bin_end_m=distance.bin_end_m,
                        )
                    )
            time.sleep(interval_s)

        return history, latest

    def approach_until_distance(
        self,
        stop_m=DEFAULT_RACK_STOP_M,
        speed_mps=0.05,
        max_duration_s=90.0,
        hz=5.0,
        history_size=3,
        initial_lidar_timeout_s=3.0,
        lost_lidar_timeout_s=1.0,
        nearest_safety_stop_m=DEFAULT_NEAREST_SAFETY_STOP_M,
        allow_estop_pedal_fault=False,
        on_sample=None,
    ):
        """
        核心调用方法：向前靠近，直到前激光雷达滤波距离 <= stop_m。

        stop_m:
            停车阈值，单位 m。默认 0.5，表示料架前 0.5m 停车。
        speed_mps:
            前进速度，单位 m/s。建议第一次验证 0.03，正常靠近 0.05。
        max_duration_s:
            最长运行时间。到时间还没到 stop_m，会停车并返回 timeout。
        hz/history_size:
            点云读取频率和中位数滤波窗口。
        initial_lidar_timeout_s:
            启动前最多等待多久收集稳定点簇。等待期间不移动。
        lost_lidar_timeout_s:
            运行中丢失稳定点簇时先停车等待，超过这个时间返回 lost_lidar。
        nearest_safety_stop_m:
            ROI 内最近单点的硬安全停车阈值。默认 0 表示禁用，因为当前
            点云会出现车体/自反射近点；主停车判断使用稳定点簇距离。
        allow_estop_pedal_fault:
            当前这台 G2 的急停踏板故障是官方确认硬件问题，现场看护时可填 True。
        on_sample:
            可选回调，用于实时打印每帧估距。
        """
        if stop_m <= 0.0:
            raise ValueError("stop_m must be positive")
        if speed_mps <= 0.0:
            raise ValueError("speed_mps must be positive for front-rack approach")
        if hz <= 0.0:
            raise ValueError("hz must be positive")
        if history_size <= 0:
            raise ValueError("history_size must be positive")
        if initial_lidar_timeout_s < 0.0:
            raise ValueError("initial_lidar_timeout_s must be >= 0")
        if lost_lidar_timeout_s < 0.0:
            raise ValueError("lost_lidar_timeout_s must be >= 0")
        if nearest_safety_stop_m < 0.0:
            raise ValueError("nearest_safety_stop_m must be >= 0")

        # 激光粗靠近也会检查底盘安全，但主 hybrid 类通常会先统一检查一次。
        # 保留这里的检查，是为了该类被单独调用时仍然安全。
        problems, warnings = self.check_motion_safety(
            allow_estop_pedal_fault=allow_estop_pedal_fault
        )
        if problems:
            raise RuntimeError("Refusing to move: " + ", ".join(problems))
        for warning in warnings:
            print("WARNING:", warning)

        initial_history, latest = self._collect_history(
            history_size=history_size,
            timeout_s=initial_lidar_timeout_s,
            hz=hz,
        )
        if len(initial_history) < history_size or latest is None:
            # 启动时没有稳定激光点簇，不能盲目前进。
            raise RuntimeError(
                "No stable front lidar rack cluster: "
                f"{len(initial_history)}/{history_size}"
            )

        initial_filtered = float(statistics.median(initial_history))
        nearest_safety_hit = (
            nearest_safety_stop_m > 0.0 and latest.nearest_m <= nearest_safety_stop_m
        )
        if nearest_safety_hit or initial_filtered <= stop_m:
            # 已经在阈值内就不运动，直接返回 already_at_threshold。
            return LidarDockingResult(
                status="already_at_threshold",
                elapsed_s=0.0,
                distance_m=latest.distance_m,
                filtered_m=initial_filtered,
                nearest_m=latest.nearest_m,
                cluster_points=latest.cluster_points,
                roi_points=latest.roi_points,
                samples=len(initial_history),
            )

        self.request_chassis_control_ready()
        time.sleep(0.3)

        interval_s = 1.0 / hz
        history = list(initial_history[-history_size:])
        samples = len(history)
        start = time.time()
        latest_distance = latest
        latest_filtered = initial_filtered
        lost_lidar_since = None

        try:
            while time.time() - start < max_duration_s:
                distance = self.read_rack_distance()
                elapsed_s = time.time() - start
                if distance is None:
                    # 纯激光靠近模式下，丢失目标就立刻停车等待恢复。
                    # hybrid 主类里针对高速粗靠近做了短丢帧 keepalive，这个底层
                    # 单独模式保持保守策略。
                    self.stop()
                    if lost_lidar_since is None:
                        lost_lidar_since = elapsed_s
                    if elapsed_s - lost_lidar_since >= lost_lidar_timeout_s:
                        return LidarDockingResult(
                            status="lost_lidar",
                            elapsed_s=elapsed_s,
                            distance_m=None,
                            filtered_m=None,
                            nearest_m=None,
                            cluster_points=0,
                            roi_points=0,
                            samples=samples,
                        )
                    time.sleep(interval_s)
                    continue

                lost_lidar_since = None

                # 更新滑动窗口，用中位数作为真正的停车判断距离。
                history.append(distance.distance_m)
                history = history[-history_size:]
                filtered_m = float(statistics.median(history))
                samples += 1
                latest_distance = distance
                latest_filtered = filtered_m

                sample = LidarRackSample(
                    elapsed_s=elapsed_s,
                    distance_m=distance.distance_m,
                    filtered_m=filtered_m,
                    nearest_m=distance.nearest_m,
                    cluster_points=distance.cluster_points,
                    roi_points=distance.roi_points,
                    bin_start_m=distance.bin_start_m,
                    bin_end_m=distance.bin_end_m,
                )
                if on_sample is not None:
                    on_sample(sample)

                nearest_safety_hit = (
                    nearest_safety_stop_m > 0.0
                    and distance.nearest_m <= nearest_safety_stop_m
                )
                if nearest_safety_hit or filtered_m <= stop_m:
                    # filtered_m 到阈值，或者最近单点触发安全阈值，立即停车。
                    self.stop()
                    return LidarDockingResult(
                        status="stopped",
                        elapsed_s=elapsed_s,
                        distance_m=distance.distance_m,
                        filtered_m=filtered_m,
                        nearest_m=distance.nearest_m,
                        cluster_points=distance.cluster_points,
                        roi_points=distance.roi_points,
                        samples=samples,
                    )

                # 距离还没到阈值，继续向前给速度。
                self.send_velocity(speed_mps)
                time.sleep(interval_s)

            # 超时还没到目标，停车返回 timeout。
            self.stop()
            return LidarDockingResult(
                status="timeout",
                elapsed_s=time.time() - start,
                distance_m=latest_distance.distance_m,
                filtered_m=latest_filtered,
                nearest_m=latest_distance.nearest_m,
                cluster_points=latest_distance.cluster_points,
                roi_points=latest_distance.roi_points,
                samples=samples,
            )
        finally:
            # 任何退出路径都补发零速度，避免异常时底盘继续执行最后一条速度。
            try:
                self.stop()
            except Exception:
                pass

    def approach_to_rack(
        self,
        speed_mps=0.05,
        max_duration_s=90.0,
        hz=5.0,
        history_size=3,
        initial_lidar_timeout_s=3.0,
        lost_lidar_timeout_s=1.0,
        nearest_safety_stop_m=DEFAULT_NEAREST_SAFETY_STOP_M,
        allow_estop_pedal_fault=False,
        on_sample=None,
    ):
        """
        业务推荐调用：靠近前方料架，并在 0.5m 自动停车。

        不需要用户输入行走距离；距离来自前激光雷达点云实时估计。
        """
        return self.approach_until_distance(
            stop_m=DEFAULT_RACK_STOP_M,
            speed_mps=speed_mps,
            max_duration_s=max_duration_s,
            hz=hz,
            history_size=history_size,
            initial_lidar_timeout_s=initial_lidar_timeout_s,
            lost_lidar_timeout_s=lost_lidar_timeout_s,
            nearest_safety_stop_m=nearest_safety_stop_m,
            allow_estop_pedal_fault=allow_estop_pedal_fault,
            on_sample=on_sample,
        )
