#!/usr/bin/env python3
"""
七根料工业总控程序。

部署位置（机器人端建议放这里）：
  /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1/industrial_7_rods_total_controller.py

运行前必须加载机器人环境：
  cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1
  source /home/agi/app/env.sh

先做 dry-run，只检查文件和打印完整步骤，不执行任何动作：
  python3 industrial_7_rods_total_controller.py --dry-run

确认现场安全后才允许真实执行：
  python3 industrial_7_rods_total_controller.py --confirm-live

90 度转向：
  默认使用 request_chassis_control(0)+move_chassis(Twist) 做 odom yaw 闭环。
  现场已经复现 Pnc.relative_move(yaw=±90) 会进入 state=8 且 yaw 基本不动，
  所以 relative 只保留为对比诊断选项：
    --turn-method relative

  工业流程里，转向返回 state=7 不再当成功。state=7 表示任务被取消/结束，
  不能证明物理角度已经到 90 度；一旦出现该状态，总控会停在转向步骤，
  不继续执行放料或下一根抓料。

从某一根开始跑，例如从第三根跑到第七根：
  python3 industrial_7_rods_total_controller.py --confirm-live --start-index 3 --end-index 7

如果现场已经抓住并拉出某一根，流程停在“抓取后后退”之前，可以从该点恢复：
  python3 industrial_7_rods_total_controller.py --confirm-live --resume-after-grab-pull-index 1

如果现场已经完成抓取后的后退，流程停在“右转去放料”之前，可以从该点恢复：
  python3 industrial_7_rods_total_controller.py --confirm-live --resume-after-grab-retreat-index 1

如果现场已经右转并移动到放料上方，流程停在“前雷达到放料目标距离”之前，可以从该点恢复：
  python3 industrial_7_rods_total_controller.py --confirm-live --resume-after-place-above-index 2

如果现场已经完成某一根的放料、开夹和拉出，流程停在“放料后后退”之前，可以从该点恢复：
  python3 industrial_7_rods_total_controller.py --confirm-live --resume-after-place-pull-index 2

如果某一根放料后退中断、少退或多退，不能重复跑完整后退；应先恢复到该次后退的
前超声目标距离，再左转继续下一根：
  python3 industrial_7_rods_total_controller.py --confirm-live --resume-after-place-retreat-target-index 3 --place-retreat-front-target-mm 1340

设计目标：
  - 不修改已有动作脚本；
  - 已有手臂/夹爪/offset 脚本全部作为子进程顺序执行；
  - 底盘前进到指定前超声距离使用 RackIndustrialDockingController 的
    粗定位 0.60m/s + 前超声精定位 0.30m/s；
  - 后退 1m 默认使用前超声 0/1 增量闭环，并用 odom 交叉校验；
  - 左右转 90 度默认使用速度控制 + odom yaw 闭环，开环时长只作为最大
    保护时间，不作为角度精度来源；
  - 不再把 relative_move 的 state=7 当成角度到位；
  - 每一步有清晰日志、返回码检查和失败即停；
  - 默认 dry-run，避免误执行完整工业动作。
"""

from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import re
import statistics
import subprocess
import sys
import time
from typing import Callable

from gdk_status_utils import read_motion_control_status_with_retry
from industrial_run_artifacts import RunRecorder


ROD_SCRIPT_NAMES = [
    "move_arm_by_json_grab_above_第一根.py",
    "move_arm_by_json_grab_above_第二根.py",
    "move_arm_by_json_grab_above_第三根.py",
    "move_arm_by_json_grab_above_第四根.py",
    "move_arm_by_json_grab_above_第五根.py",
    "move_arm_by_json_grab_above_第六根.py",
    "move_arm_by_json_grab_above_第七根.py",
]

DEFAULT_GRAB_VERTICAL_STACK_PITCH_M = -0.060

ARM_JOINT_KEYS = [
    "idx21_arm_l_joint1",
    "idx22_arm_l_joint2",
    "idx23_arm_l_joint3",
    "idx24_arm_l_joint4",
    "idx25_arm_l_joint5",
    "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
    "idx61_arm_r_joint1",
    "idx62_arm_r_joint2",
    "idx63_arm_r_joint3",
    "idx64_arm_r_joint4",
    "idx65_arm_r_joint5",
    "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]

FRONT_ULTRASONIC_IDS = (0, 1)
RIGHT_ULTRASONIC_IDS = (2, 3)
REAR_ULTRASONIC_IDS = (4, 5)
LEFT_ULTRASONIC_IDS = (6, 7)


def detect_default_base_dir() -> Path:
    """
    自动判断动作脚本目录。

    推荐部署：
      BOX_528_1/industrial_7_rods_total_controller.py

    兼容部署：
      BOX_528_1/rack_hybrid_docking_package/industrial_7_rods_total_controller.py

    如果脚本放在 rack_hybrid_docking_package 里，并且上一层目录存在动作脚本，
    就自动把 base_dir 设成上一层 BOX 目录。
    """
    here = Path(__file__).resolve().parent
    if (here / "move_ee_pose_open_2.py").exists():
        return here
    parent = here.parent
    if here.name == "rack_hybrid_docking_package" and (parent / "move_ee_pose_open_2.py").exists():
        return parent
    return here


def parse_state_list(text: str) -> tuple[int, ...]:
    """把命令行里的 '3,9' 解析成状态码元组。"""
    states: list[int] = []
    for part in text.split(","):
        item = part.strip()
        if not item:
            continue
        states.append(int(item))
    if not states:
        raise ValueError("state list must not be empty")
    return tuple(states)


def lateral_sample_stats(values: list[float]) -> dict[str, float | int]:
    """Return raw and outlier-trimmed lateral sample stability metrics."""
    ordered = sorted(float(value) for value in values)
    sample_min = ordered[0]
    sample_max = ordered[-1]
    sample_median = float(statistics.median(ordered))
    trim_count = 0
    if len(ordered) >= 5:
        trim_count = max(1, int(len(ordered) * 0.10))
        if len(ordered) - 2 * trim_count < 3:
            trim_count = 0
    stable_values = ordered[trim_count : len(ordered) - trim_count] if trim_count else ordered
    abs_deviation_values = [abs(value - sample_median) for value in ordered]
    return {
        "min_m": sample_min,
        "max_m": sample_max,
        "span_m": sample_max - sample_min,
        "median_m": sample_median,
        "robust_min_m": stable_values[0],
        "robust_max_m": stable_values[-1],
        "robust_span_m": stable_values[-1] - stable_values[0],
        "mad_m": float(statistics.median(abs_deviation_values)),
        "trim_count": trim_count,
    }


@dataclass(frozen=True)
class RuntimeConfig:
    base_dir: Path
    dry_run: bool
    confirm_live: bool
    allow_estop_pedal_fault: bool
    start_index: int
    end_index: int
    settle_s: float
    script_timeout_s: float
    turn_timeout_s: float
    grab_distance_mm: int
    grab_vertical_stack_pitch_m: float | None
    place_distance_mm: int
    grab_brake_margin_mm: int
    place_brake_margin_mm: int
    grab_min_safe_mm: int
    place_min_safe_mm: int
    grab_target_tolerance_mm: int
    grab_correction_speed_mps: float
    grab_correction_max_passes: int
    grab_angle_correction_max_span_mm: int
    grab_angle_correction_max_passes: int
    grab_angle_correction_angular_speed_radps: float
    grab_angle_correction_probe_s: float
    grab_target_avg_accept_span_mm: int
    place_target_tolerance_mm: int
    place_correction_speed_mps: float
    place_correction_max_passes: int
    place_retreat_front_target_mm: int | None
    place_retreat_target_tolerance_mm: int
    place_retreat_forward_speed_mps: float
    place_retreat_forward_brake_margin_mm: int
    place_retreat_forward_correction_speed_mps: float
    place_retreat_correction_max_passes: int
    rack_centering_mode: str
    rack_pose_samples: int
    rack_pose_interval_s: float
    rack_pose_min_range_m: float
    rack_pose_max_range_m: float
    rack_pose_lateral_half_width_m: float
    rack_pose_z_min_m: float
    rack_pose_z_max_m: float
    rack_pose_bin_width_m: float
    rack_pose_min_cluster_points: int
    rack_place_pose_min_range_m: float | None
    rack_place_pose_max_range_m: float | None
    rack_place_pose_lateral_half_width_m: float | None
    rack_place_pose_z_min_m: float | None
    rack_place_pose_z_max_m: float | None
    rack_place_pose_bin_width_m: float | None
    rack_place_pose_min_cluster_points: int | None
    rack_pose_warn_lateral_m: float
    rack_pose_warn_yaw_deg: float
    rack_pose_min_confidence: float
    rack_near_target_skip_centering_margin_mm: int
    rack_guarded_ultrasonic_override: bool
    rack_guarded_ultrasonic_max_pose_lateral_m: float
    rack_guarded_ultrasonic_max_front_span_mm: int
    rack_guarded_ultrasonic_min_front_mm: int
    rack_post_approach_guarded_check: bool
    rack_post_approach_max_lateral_m: float
    rack_post_approach_max_yaw_deg: float
    rack_post_approach_max_front_span_mm: int
    rack_post_approach_front_retry_windows: int
    front_too_close_safe_backoff_retries: int
    rack_yaw_shadow_min_distance_m: float
    rack_yaw_shadow_max_distance_m: float
    rack_yaw_shadow_min_confidence: float
    rack_yaw_shadow_max_fit_residual_m: float
    rack_yaw_shadow_trigger_deg: float
    rack_yaw_shadow_max_deg: float
    rack_lateral_shadow_min_distance_m: float
    rack_lateral_shadow_max_distance_m: float
    rack_lateral_shadow_min_confidence: float
    rack_lateral_shadow_max_fit_residual_m: float
    rack_lateral_shadow_trigger_m: float
    rack_lateral_shadow_max_m: float
    rack_lateral_shadow_max_correction_m: float
    rack_lateral_active_target_m: float
    rack_lateral_active_max_initial_m: float
    rack_lateral_active_max_yaw_deg: float
    rack_lateral_active_max_passes: int
    rack_lateral_active_speed_mps: float
    rack_lateral_active_direction: str
    rack_lateral_active_step_s: float
    rack_lateral_active_settle_s: float
    rack_lateral_active_min_improvement_m: float
    rack_lateral_active_min_front_mm: int
    rack_lateral_active_min_rear_mm: int
    rack_lateral_active_min_side_mm: int
    rack_lateral_active_hard_min_front_mm: int
    rack_lateral_active_hard_min_rear_mm: int
    rack_lateral_active_hard_min_side_mm: int
    rack_lateral_active_clearance_samples: int
    rack_lateral_active_clearance_interval_s: float
    rack_lateral_active_hz: float
    rack_lateral_active_max_sample_span_m: float
    rack_lateral_active_rollback_on_worse: bool
    rack_lateral_active_rollback_step_scale: float
    coarse_speed_mps: float
    grab_approach_speed_mps: float
    place_approach_speed_mps: float
    retreat_distance_m: float
    retreat_target_tolerance_mm: int
    retreat_speed_mps: float
    retreat_method: str
    retreat_escape_delta_m: float
    grab_retreat_front_occlusion_escape_threshold_mm: int
    grab_retreat_front_occlusion_escape_m: float
    grab_retreat_front_occlusion_escape_speed_mps: float
    retreat_front_delta_consistency_mm: int
    retreat_odom_tolerance_m: float
    retreat_require_odom_crosscheck: bool
    retreat_odom_auto_correction: bool
    retreat_odom_auto_correction_max_m: float
    retreat_odom_auto_correction_min_m: float
    retreat_odom_auto_correction_speed_mps: float
    retreat_odom_auto_correction_max_passes: int
    retreat_odom_auto_correction_front_hard_min_mm: int
    retreat_odom_auto_correction_rear_hard_min_mm: int
    retreat_odom_auto_correction_clearance_retry_s: float
    retreat_open_loop_brake_compensation_m: float
    turn_angular_speed_radps: float
    right_turn_duration_s: float
    left_turn_duration_s: float
    turn_hz: float
    turn_control_mode: int
    turn_method: str
    turn_success_states: tuple[int, ...]
    turn_min_sensor_delta_mm: int
    turn_yaw_tolerance_deg: float
    turn_confirm_samples: int
    turn_confirm_interval_s: float
    turn_confirm_max_span_deg: float
    turn_correction_enabled: bool
    turn_correction_max_passes: int
    turn_correction_angular_speed_radps: float
    turn_correction_max_error_deg: float
    allow_turn_motion_error_2: bool
    turn_validation_ok: bool
    resume_after_grab_pull_index: int | None
    resume_after_grab_retreat_index: int | None
    resume_after_place_above_index: int | None
    resume_after_place_pull_index: int | None
    resume_after_place_retreat_target_index: int | None
    log_file: Path | None
    event_file: Path | None
    checkpoint_file: Path | None
    report_file: Path | None


class Industrial7RodsController:
    """七根料顺序搬运总控。"""

    def __init__(self, config: RuntimeConfig):
        self.config = config
        self.step_no = 0
        self.current_step_title: str | None = None
        self.current_step_started_s: float | None = None
        self.current_rod_index: int | None = None
        self.recorder = RunRecorder(
            name="industrial_7_rods",
            log_file=config.log_file,
            event_file=config.event_file,
            checkpoint_file=config.checkpoint_file,
            report_file=config.report_file,
        )

    def log(self, message: str):
        """同时打印到终端和可选日志文件。"""
        line = f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {message}"
        print(line, flush=True)
        if self.config.log_file is not None:
            self.config.log_file.parent.mkdir(parents=True, exist_ok=True)
            with self.config.log_file.open("a", encoding="utf-8") as f:
                f.write(line + "\n")

    def next_step(self, title: str):
        self.step_no += 1
        self.current_step_title = title
        self.current_step_started_s = time.time()
        self.log(f"STEP {self.step_no:03d}: {title}")
        self.emit_event("step_start", step_no=self.step_no, title=title, rod_index=self.current_rod_index)
        self.write_checkpoint(status="step_running", resume_hint=self.resume_hint_for_current_context())

    def emit_event(self, event_type: str, **fields):
        """Write one structured machine-readable event."""
        return self.recorder.event(event_type, **fields)

    def write_checkpoint(self, status: str, **fields):
        """Persist the latest resumable workflow position."""
        payload_fields = dict(fields)
        step_no = payload_fields.pop("step_no", self.step_no)
        step_title = payload_fields.pop("step_title", self.current_step_title)
        rod_index = payload_fields.pop("rod_index", self.current_rod_index)
        return self.recorder.checkpoint(
            status=status,
            step_no=step_no,
            step_title=step_title,
            rod_index=rod_index,
            start_index=self.config.start_index,
            end_index=self.config.end_index,
            dry_run=self.config.dry_run,
            **payload_fields,
        )

    def complete_current_step(self, status: str = "completed", **fields):
        elapsed_s = None
        if self.current_step_started_s is not None:
            elapsed_s = round(time.time() - self.current_step_started_s, 3)
        if "elapsed_s" in fields:
            elapsed_s = fields.pop("elapsed_s")
        self.emit_event(
            "step_done",
            step_no=self.step_no,
            title=self.current_step_title,
            rod_index=self.current_rod_index,
            status=status,
            elapsed_s=elapsed_s,
            **fields,
        )
        self.write_checkpoint(status=f"step_{status}", resume_hint=self.resume_hint_for_current_context(), **fields)

    def fail_current_step(self, exc: BaseException):
        elapsed_s = None
        if self.current_step_started_s is not None:
            elapsed_s = round(time.time() - self.current_step_started_s, 3)
        self.emit_event(
            "step_failed",
            step_no=self.step_no,
            title=self.current_step_title,
            rod_index=self.current_rod_index,
            elapsed_s=elapsed_s,
            error=exc,
            resume_hint=self.resume_hint_for_current_context(),
        )
        self.write_checkpoint(
            status="failed",
            error=exc,
            resume_hint=self.resume_hint_for_current_context(),
        )

    def write_final_report(self, status: str, exc: BaseException | None = None):
        report = self.recorder.report(
            status,
            error=exc,
            step_no=self.step_no,
            step_title=self.current_step_title,
            rod_index=self.current_rod_index,
        )
        if self.config.report_file is not None:
            self.log(f"run_report={self.config.report_file}")
        return report

    def resume_hint_for_current_context(self) -> str | None:
        if self.current_rod_index is None or self.current_step_title is None:
            return None
        title = self.current_step_title
        rod = self.current_rod_index
        if "拉出" in title and "放料后" not in title:
            return f"--resume-after-grab-pull-index {rod}"
        if "后退" in title and "放料后" not in title:
            return (
                f"第{rod}根抓取后后退未完成；不要直接右转，"
                "按 checkpoint 里的 retreat_target_front_avg_mm 纠偏后再恢复"
            )
        if "移动到放置上方" in title:
            return f"--resume-after-place-above-index {rod}"
        if "放料后拉出" in title:
            return f"--resume-after-place-pull-index {rod}"
        if "放料后后退" in title:
            return (
                f"--resume-after-place-retreat-target-index {rod} "
                f"--place-retreat-front-target-mm {self.default_place_retreat_front_target_mm()}"
            )
        if "向右转" in title or "向左转" in title:
            return f"先单独用 industrial_turn_diagnostic.py 复测转向，再决定恢复点"
        if "抓取上方" in title or f"靠近到 {self.config.grab_distance_mm}mm" in title:
            return f"重新从第{rod}根开始，或确认现场状态后手工选择恢复点"
        if "闭合夹爪" in title:
            return f"确认第{rod}根已闭合且已拉出后，才可用 --resume-after-grab-pull-index {rod}"
        if f"靠近到 {self.config.place_distance_mm}mm" in title or "下移" in title or "张开夹爪放料" in title:
            return f"确认第{rod}根已放料且已拉出后，才可用 --resume-after-place-pull-index {rod}"
        if "向左转" in title:
            return f"第{rod}根放料后流程未完成；确认姿态后优先单独诊断左转"
        return None

    def default_place_retreat_front_target_mm(self) -> int:
        if self.config.place_retreat_front_target_mm is not None:
            return self.config.place_retreat_front_target_mm
        return int(round(self.config.place_distance_mm + self.config.retreat_distance_m * 1000.0))

    def start_rod(self, rod_index: int, mode: str, **fields):
        self.current_rod_index = rod_index
        self.emit_event("rod_start", rod_index=rod_index, mode=mode, **fields)
        self.write_checkpoint(status="rod_running", rod_index=rod_index, mode=mode, **fields)

    def finish_rod(self, rod_index: int, mode: str):
        self.emit_event("rod_done", rod_index=rod_index, mode=mode)
        self.write_checkpoint(status="rod_completed", rod_index=rod_index, mode=mode)

    def require_live_allowed(self):
        """
        防误触发保护。

        dry-run 模式只打印计划，不执行运动。
        真实执行必须显式传 --confirm-live。
        """
        if self.config.dry_run:
            return
        if not self.config.confirm_live:
            raise RuntimeError("真实执行必须传 --confirm-live；否则只允许 --dry-run")

    def check_live_startup_safety(self):
        """真实运行第一步动作前做全局状态预检，避免充电插着时先动机械臂。"""
        if self.config.dry_run:
            self.log("startup live preflight skipped in dry-run")
            return

        import agibot_gdk

        gdk_inited = False
        try:
            result = agibot_gdk.gdk_init()
            gdk_res = getattr(agibot_gdk, "GDKRes", None)
            if gdk_res is not None and result not in (None, gdk_res.kSuccess):
                raise RuntimeError(f"GDK init failed during startup preflight: {result}")
            gdk_inited = True

            robot = agibot_gdk.Robot()
            pnc = agibot_gdk.Pnc()
            slam = agibot_gdk.Slam()
            time.sleep(0.5)
            power = robot.get_chassis_power_state()
            motion_status_error = None
            try:
                motion = read_motion_control_status_with_retry(robot)
            except RuntimeError as exc:
                motion = None
                motion_status_error = str(exc)
            whole = robot.get_whole_body_status()
            task_status_error = None
            try:
                task = pnc.get_task_state()
            except Exception as exc:
                task = None
                task_status_error = f"{type(exc).__name__}: {exc}"

            problems = []
            warnings = []

            charge_plug = getattr(power, "charge_plug_insert_state", 0)
            estop_state = getattr(power, "emergency_stop_pedal_state", 0)
            estop_fault = getattr(power, "emergency_stop_pedal_fault_state", 0)
            ultrasonic_power = getattr(power, "chassis_ultrasonic_radar_power_state", 0)
            motion_error = None if motion is None else getattr(motion, "error_code", 0)
            task_state = None if task is None else getattr(task, "state", None)
            task_id = None if task is None else getattr(task, "id", None)
            task_type = None if task is None else getattr(task, "type", None)
            idle_task_states = (0, 3, 6, 7, 8, 9)

            odom_required = (
                self.config.retreat_require_odom_crosscheck
                or self.config.turn_method == "velocity"
            )
            odom_errors = []
            odom_xy_samples = []
            odom_yaw_samples = []
            odom_speeds = []
            for sample_index in range(3):
                try:
                    odom = slam.get_odom_info()
                except Exception as exc:
                    odom_errors.append(f"{type(exc).__name__}: {exc}")
                else:
                    xy = self._extract_xy_from_odom(odom)
                    yaw_deg = self._extract_yaw_deg_from_odom(odom)
                    velocity_body = getattr(odom, "velocity_body", None)
                    linear_speed = None
                    if velocity_body is not None:
                        try:
                            vx = float(getattr(velocity_body, "x", 0.0))
                            vy = float(getattr(velocity_body, "y", 0.0))
                            vz = float(getattr(velocity_body, "z", 0.0))
                            linear_speed = math.sqrt(vx * vx + vy * vy + vz * vz)
                        except Exception:
                            linear_speed = None
                    if xy is not None:
                        odom_xy_samples.append(xy)
                    if yaw_deg is not None:
                        odom_yaw_samples.append(yaw_deg)
                    if linear_speed is not None:
                        odom_speeds.append(linear_speed)
                if sample_index < 2:
                    time.sleep(0.15)

            if charge_plug != 0:
                problems.append("charge_plug_insert_state=1")
            if motion_status_error is not None:
                problems.append(f"motion_control_status_unavailable={motion_status_error}")
            elif motion_error == 2 and self.config.allow_turn_motion_error_2:
                collision_pairs_1 = getattr(motion, "collision_pairs_1", ()) or ()
                collision_pairs_2 = getattr(motion, "collision_pairs_2", ()) or ()
                if collision_pairs_1 or collision_pairs_2:
                    problems.append(
                        "motion_control_error=2_with_collision_pairs="
                        f"{collision_pairs_1},{collision_pairs_2}"
                    )
                else:
                    warnings.append(
                        "motion_control_error=2 ignored by explicit "
                        "--allow-turn-motion-error-2 override"
                    )
            elif motion_error != 0:
                problems.append(f"motion_control_error={motion_error}")
            if ultrasonic_power != 1:
                problems.append("chassis_ultrasonic_radar_power_state!=1")
            if estop_state != 0:
                problems.append("emergency_stop_pedal_state!=0")
            if task_status_error is not None:
                problems.append(f"pnc_task_state_unavailable={task_status_error}")
            elif task_state not in idle_task_states:
                problems.append(
                    f"pnc_task_state_not_idle={task_state},"
                    f"id={task_id},type={task_type}"
                )
            if odom_required:
                if not odom_xy_samples:
                    problems.append(f"odom_xy_unavailable={tuple(odom_errors[-3:])}")
                if self.config.turn_method == "velocity" and not odom_yaw_samples:
                    problems.append(f"odom_yaw_unavailable={tuple(odom_errors[-3:])}")
                if not odom_speeds:
                    problems.append(
                        "odom_velocity_unavailable_for_stopped_check="
                        f"{tuple(odom_errors[-3:])}"
                    )
                else:
                    max_odom_speed = max(odom_speeds)
                    if max_odom_speed > 0.02:
                        problems.append(
                            f"robot_not_stopped_by_odom_speed={max_odom_speed:.4f}>0.0200"
                        )
            else:
                max_odom_speed = max(odom_speeds) if odom_speeds else None

            body_error_fields = (
                "right_arm_error",
                "left_arm_error",
                "right_end_error",
                "left_end_error",
                "waist_error",
                "lift_error",
                "neck_error",
                "chassis_error",
            )
            body_errors = {}
            for field in body_error_fields:
                value = getattr(whole, field, None)
                if isinstance(value, int) and value != 0:
                    body_errors[field] = value
            if body_errors:
                problems.append(f"whole_body_errors={body_errors}")

            self.log(
                "startup_live_preflight "
                f"motion_error={motion_error} charge_plug_insert_state={charge_plug} "
                f"emergency_stop_pedal_state={estop_state} "
                f"emergency_stop_pedal_fault_state={estop_fault} "
                f"chassis_ultrasonic_radar_power_state={ultrasonic_power} "
                f"pnc_task_state={task_state} pnc_task_id={task_id} "
                f"pnc_task_type={task_type} "
                f"odom_required={odom_required} "
                f"odom_xy_samples={len(odom_xy_samples)} "
                f"odom_yaw_samples={len(odom_yaw_samples)} "
                f"odom_speed_samples={len(odom_speeds)} "
                f"max_odom_speed_mps={max(odom_speeds) if odom_speeds else None} "
                f"body_errors={body_errors} warnings={tuple(warnings)} "
                f"problems={tuple(problems)}"
            )
            if problems:
                raise RuntimeError("startup preflight blocked: " + ", ".join(problems))
        finally:
            if gdk_inited:
                try:
                    agibot_gdk.gdk_release()
                except Exception:
                    pass

    def check_required_files(self):
        """启动前检查所有被调用脚本存在，缺一个就不开始。"""
        required = [
            "move_ee_pose_open_2.py",
            "move_ee_pose_close_2.py",
            "offset_move_pull.py",
            "offset_move_down.py",
            "move_arm_by_json_grab_above_2.py",
            "rack_hybrid_docking_package/industrial_docking_integration_template.py",
            "rack_hybrid_docking_package/rack_industrial_docking.py",
        ] + ROD_SCRIPT_NAMES
        if self.config.grab_vertical_stack_pitch_m is not None:
            required.append("move_arm_vertical_stack_grab_above.py")

        missing = []
        for name in required:
            path = self.config.base_dir / name
            if not path.exists():
                missing.append(str(path))

        if missing:
            raise FileNotFoundError("缺少必要脚本:\n" + "\n".join(missing))

        self.log("required file check passed")
        self.check_pose_json_files()

    def check_pose_json_files(self):
        """校验抓取/放置姿态脚本声明的 JSON 文件完整，避免缺键默认 0.0。"""
        pose_scripts = ["move_arm_by_json_grab_above_2.py"]
        if self.config.grab_vertical_stack_pitch_m is None:
            pose_scripts = ROD_SCRIPT_NAMES + pose_scripts
        else:
            pose_scripts = ["move_arm_by_json_grab_above_第一根.py"] + pose_scripts
        failures: list[str] = []

        for script_name in pose_scripts:
            script_path = self.config.base_dir / script_name
            try:
                script_text = script_path.read_text(encoding="utf-8")
            except OSError as exc:
                failures.append(f"{script_path}: 读取失败: {exc}")
                continue

            match = re.search(r"JSON_FILE_PATH\s*=\s*['\"]([^'\"]+)['\"]", script_text)
            if not match:
                failures.append(f"{script_path}: 未找到 JSON_FILE_PATH")
                continue

            json_path = Path(match.group(1))
            if not json_path.is_absolute():
                json_path = script_path.parent / json_path

            if not json_path.exists():
                failures.append(f"{script_path}: 姿态 JSON 不存在: {json_path}")
                continue

            try:
                raw = json_path.read_bytes()
                pose_data = json.loads(raw.decode("utf-8"))
            except Exception as exc:
                failures.append(f"{json_path}: JSON 读取/解析失败: {exc}")
                continue

            if not isinstance(pose_data, dict):
                failures.append(f"{json_path}: 顶层必须是 dict，当前是 {type(pose_data).__name__}")
                continue

            missing_keys = [key for key in ARM_JOINT_KEYS if key not in pose_data]
            bad_type_keys = [
                key
                for key in ARM_JOINT_KEYS
                if key in pose_data and not isinstance(pose_data[key], (int, float))
            ]
            if missing_keys:
                failures.append(f"{json_path}: 缺少手臂关节键: {', '.join(missing_keys)}")
                continue
            if bad_type_keys:
                failures.append(f"{json_path}: 手臂关节值不是数字: {', '.join(bad_type_keys)}")
                continue

            stat = json_path.stat()
            digest = hashlib.sha256(raw).hexdigest()[:12]
            mtime = datetime.fromtimestamp(stat.st_mtime).isoformat(timespec="seconds")
            self.log(
                "pose_json_check "
                f"script={script_name} json={json_path} "
                f"sha256={digest} mtime={mtime}"
            )

        if failures:
            raise RuntimeError("姿态 JSON 预检失败:\n" + "\n".join(failures))

        self.log(f"pose json check passed scripts={len(pose_scripts)} joints_per_pose={len(ARM_JOINT_KEYS)}")

    def run_python_script(
        self,
        script_name: str,
        title: str,
        extra_args: list[str] | None = None,
    ):
        """
        顺序执行一个已有动作脚本。

        为什么用子进程而不是 import：
          这些脚本都是顶层执行型脚本，import 时会立刻 gdk_init 并发动作。
          工业总控必须用子进程运行，并检查返回码。
        """
        self.next_step(title)
        script_path = self.config.base_dir / script_name
        command = [sys.executable, str(script_path)] + list(extra_args or [])
        self.log(f"script={script_path} args={tuple(extra_args or ())}")

        if self.config.dry_run:
            self.log("dry-run: skip script execution")
            self.complete_current_step(
                "dry_run",
                script=str(script_path),
                args=tuple(extra_args or ()),
            )
            return

        started = time.time()
        self.emit_event("script_start", step_no=self.step_no, script=str(script_path), title=title)
        try:
            result = subprocess.run(
                command,
                cwd=str(self.config.base_dir),
                timeout=self.config.script_timeout_s,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
            )
        except subprocess.TimeoutExpired as exc:
            elapsed = time.time() - started
            output = exc.output or ""
            if isinstance(output, bytes):
                output = output.decode("utf-8", errors="replace")
            for line in output.splitlines():
                self.log(f"{script_name}: {line}")
            self.log(f"script timeout elapsed_s={elapsed:.1f}")
            self.emit_event(
                "script_timeout",
                step_no=self.step_no,
                script=str(script_path),
                elapsed_s=round(elapsed, 3),
                timeout_s=self.config.script_timeout_s,
            )
            raise RuntimeError(
                f"script timeout: {script_name}, timeout_s={self.config.script_timeout_s}"
            ) from exc

        elapsed = time.time() - started
        for line in (result.stdout or "").splitlines():
            self.log(f"{script_name}: {line}")
        self.log(f"script finished returncode={result.returncode} elapsed_s={elapsed:.1f}")
        self.emit_event(
            "script_finish",
            step_no=self.step_no,
            script=str(script_path),
            returncode=result.returncode,
            elapsed_s=round(elapsed, 3),
        )

        if result.returncode != 0:
            self.write_checkpoint(
                status="script_failed",
                script=str(script_path),
                returncode=result.returncode,
                resume_hint=self.resume_hint_for_current_context(),
            )
            raise RuntimeError(f"script failed: {script_name}, returncode={result.returncode}")

        self.complete_current_step(
            "completed",
            script=str(script_path),
            args=tuple(extra_args or ()),
            elapsed_s=round(elapsed, 3),
        )
        time.sleep(self.config.settle_s)

    def with_industrial_rack(self, action: Callable):
        """
        创建 RackIndustrialDockingController 并执行 action。

        注意：
          每次前雷达靠近/后退都单独创建并释放控制器，避免与手臂脚本的
          gdk_init/gdk_release 生命周期互相影响。
        """
        rack_package_dir = str(self.config.base_dir / "rack_hybrid_docking_package")
        if rack_package_dir not in sys.path:
            sys.path.insert(0, rack_package_dir)

        # 从集成模板导入该控制器，保持与 industrial_docking_integration_template.py
        # 的业务接入方式一致；模板本身不会在 import 时执行动作。
        from industrial_docking_integration_template import RackIndustrialDockingController

        with RackIndustrialDockingController(
            front_ultrasonic_ids=FRONT_ULTRASONIC_IDS,
            rear_ultrasonic_ids=REAR_ULTRASONIC_IDS,
        ) as rack:
            return action(rack)

    def rack_pose_roi_kwargs(self, target_mm: int | None = None):
        roi = {
            "min_range_m": self.config.rack_pose_min_range_m,
            "max_range_m": self.config.rack_pose_max_range_m,
            "lateral_half_width_m": self.config.rack_pose_lateral_half_width_m,
            "z_min_m": self.config.rack_pose_z_min_m,
            "z_max_m": self.config.rack_pose_z_max_m,
            "bin_width_m": self.config.rack_pose_bin_width_m,
            "min_cluster_points": self.config.rack_pose_min_cluster_points,
        }
        if target_mm == self.config.place_distance_mm:
            overrides = {
                "min_range_m": self.config.rack_place_pose_min_range_m,
                "max_range_m": self.config.rack_place_pose_max_range_m,
                "lateral_half_width_m": self.config.rack_place_pose_lateral_half_width_m,
                "z_min_m": self.config.rack_place_pose_z_min_m,
                "z_max_m": self.config.rack_place_pose_z_max_m,
                "bin_width_m": self.config.rack_place_pose_bin_width_m,
                "min_cluster_points": self.config.rack_place_pose_min_cluster_points,
            }
            for key, value in overrides.items():
                if value is not None:
                    roi[key] = value
        return roi

    def monitor_rack_pose(self, rack, label: str, target_mm: int | None = None):
        """
        只读料架姿态监控。

        第一阶段只记录 distance/lateral/yaw/confidence，不用它控制底盘。
        横移和 yaw 自动纠偏必须等多轮日志证明识别稳定后再单独打开。
        """
        if self.config.rack_centering_mode == "off":
            return None

        samples = []
        errors = []
        for sample_index in range(1, self.config.rack_pose_samples + 1):
            try:
                pose = rack.lidar.read_rack_pose(**self.rack_pose_roi_kwargs(target_mm))
            except Exception as exc:
                pose = None
                errors.append(f"{sample_index}:{type(exc).__name__}:{exc}")

            if pose is not None:
                samples.append(pose)
            if sample_index < self.config.rack_pose_samples:
                time.sleep(self.config.rack_pose_interval_s)

        if not samples:
            self.log(
                "rack_pose_monitor_unavailable "
                f"label={label} target_mm={target_mm} "
                f"samples=0 requested_samples={self.config.rack_pose_samples} "
                f"errors={tuple(errors[-3:])}"
            )
            self.emit_event(
                "rack_pose_monitor",
                step_no=self.step_no,
                rod_index=self.current_rod_index,
                label=label,
                target_mm=target_mm,
                status="unavailable",
                sample_count=0,
                requested_samples=self.config.rack_pose_samples,
                errors=tuple(errors[-3:]),
            )
            return None

        lateral_sample_values = [float(pose.lateral_center_m) for pose in samples]
        lateral_stats = lateral_sample_stats(lateral_sample_values)
        lateral_sample_min_m = lateral_stats["min_m"]
        lateral_sample_max_m = lateral_stats["max_m"]
        lateral_sample_span_m = lateral_stats["span_m"]
        lateral_sample_robust_span_m = lateral_stats["robust_span_m"]
        lateral_sample_mad_m = lateral_stats["mad_m"]
        distance_m = float(statistics.median(pose.distance_m for pose in samples))
        lateral_center_m = float(statistics.median(lateral_sample_values))
        confidence = float(statistics.median(pose.confidence for pose in samples))
        cluster_points = int(statistics.median(pose.cluster_points for pose in samples))
        roi_points = int(statistics.median(pose.roi_points for pose in samples))
        lateral_span_m = float(statistics.median(pose.lateral_span_m for pose in samples))
        residual_values = [
            pose.fit_residual_m for pose in samples if pose.fit_residual_m is not None
        ]
        fit_residual_m = (
            float(statistics.median(residual_values)) if residual_values else None
        )
        yaw_values = [pose.yaw_deg for pose in samples if pose.yaw_deg is not None]
        yaw_deg = float(statistics.median(yaw_values)) if yaw_values else None

        warn_reasons = []
        if confidence < self.config.rack_pose_min_confidence:
            warn_reasons.append("low_confidence")
        if abs(lateral_center_m) > self.config.rack_pose_warn_lateral_m:
            warn_reasons.append("lateral_offset")
        if yaw_deg is None:
            warn_reasons.append("yaw_unavailable")
        elif abs(yaw_deg) > self.config.rack_pose_warn_yaw_deg:
            warn_reasons.append("yaw_offset")

        yaw_shadow = None
        lateral_shadow = None
        if self.config.rack_centering_mode in ("shadow", "guarded", "active"):
            yaw_shadow = self.build_rack_yaw_shadow_decision(
                label=label,
                target_mm=target_mm,
                distance_m=distance_m,
                yaw_deg=yaw_deg,
                confidence=confidence,
                fit_residual_m=fit_residual_m,
            )
            lateral_shadow = self.build_rack_lateral_shadow_decision(
                label=label,
                target_mm=target_mm,
                distance_m=distance_m,
                lateral_center_m=lateral_center_m,
                confidence=confidence,
                fit_residual_m=fit_residual_m,
            )

        status = "warn" if warn_reasons else "ok"
        self.log(
            "rack_pose_monitor "
            f"label={label} status={status} target_mm={target_mm} "
            f"distance_m={distance_m:.3f} lateral_center_m={lateral_center_m:.3f} "
            f"yaw_deg={None if yaw_deg is None else round(yaw_deg, 3)} "
            f"confidence={confidence:.3f} sample_count={len(samples)} "
            f"cluster_points={cluster_points} roi_points={roi_points} "
            f"lateral_span_m={lateral_span_m:.3f} "
            f"lateral_sample_span_m={lateral_sample_span_m:.3f} "
            f"lateral_sample_robust_span_m={lateral_sample_robust_span_m:.3f} "
            f"lateral_sample_mad_m={lateral_sample_mad_m:.3f} "
            f"fit_residual_m={None if fit_residual_m is None else round(fit_residual_m, 4)} "
            f"warn_reasons={tuple(warn_reasons)}"
        )
        event_payload = dict(
            step_no=self.step_no,
            rod_index=self.current_rod_index,
            label=label,
            target_mm=target_mm,
            status=status,
            distance_m=distance_m,
            lateral_center_m=lateral_center_m,
            yaw_deg=yaw_deg,
            confidence=confidence,
            sample_count=len(samples),
            requested_samples=self.config.rack_pose_samples,
            cluster_points=cluster_points,
            roi_points=roi_points,
            lateral_span_m=lateral_span_m,
            lateral_sample_min_m=lateral_sample_min_m,
            lateral_sample_max_m=lateral_sample_max_m,
            lateral_sample_span_m=lateral_sample_span_m,
            lateral_sample_robust_min_m=lateral_stats["robust_min_m"],
            lateral_sample_robust_max_m=lateral_stats["robust_max_m"],
            lateral_sample_robust_span_m=lateral_sample_robust_span_m,
            lateral_sample_mad_m=lateral_sample_mad_m,
            lateral_sample_trim_count=lateral_stats["trim_count"],
            fit_residual_m=fit_residual_m,
            warn_reasons=tuple(warn_reasons),
            pose_roi=self.rack_pose_roi_kwargs(target_mm),
        )
        if yaw_shadow is not None:
            self.log(
                "rack_pose_yaw_shadow "
                f"label={label} target_mm={target_mm} "
                f"decision={yaw_shadow['decision']} phase={yaw_shadow['phase']} "
                f"candidate_robot_yaw_correction_deg="
                f"{yaw_shadow['candidate_robot_yaw_correction_deg']} "
                f"reasons={yaw_shadow['reasons']}"
            )
            self.emit_event(
                "rack_pose_yaw_shadow",
                step_no=self.step_no,
                rod_index=self.current_rod_index,
                label=label,
                target_mm=target_mm,
                **yaw_shadow,
            )
            event_payload.update(
                yaw_shadow_decision=yaw_shadow["decision"],
                yaw_shadow_candidate_robot_yaw_correction_deg=(
                    yaw_shadow["candidate_robot_yaw_correction_deg"]
                ),
                yaw_shadow_reasons=yaw_shadow["reasons"],
            )
        if lateral_shadow is not None:
            self.log(
                "rack_pose_lateral_shadow "
                f"label={label} target_mm={target_mm} "
                f"decision={lateral_shadow['decision']} phase={lateral_shadow['phase']} "
                f"candidate_body_lateral_correction_m="
                f"{lateral_shadow['candidate_body_lateral_correction_m']} "
                f"execution_allowed={lateral_shadow['candidate_execution_allowed']} "
                f"reasons={lateral_shadow['reasons']} "
                f"execution_blockers={lateral_shadow['execution_blockers']}"
            )
            self.emit_event(
                "rack_pose_lateral_shadow",
                step_no=self.step_no,
                rod_index=self.current_rod_index,
                label=label,
                target_mm=target_mm,
                **lateral_shadow,
            )
            event_payload.update(
                lateral_shadow_decision=lateral_shadow["decision"],
                lateral_shadow_candidate_body_lateral_correction_m=(
                    lateral_shadow["candidate_body_lateral_correction_m"]
                ),
                lateral_shadow_execution_allowed=lateral_shadow[
                    "candidate_execution_allowed"
                ],
                lateral_shadow_reasons=lateral_shadow["reasons"],
                lateral_shadow_execution_blockers=lateral_shadow[
                    "execution_blockers"
                ],
            )
        self.emit_event("rack_pose_monitor", **event_payload)
        return {
            "status": status,
            "distance_m": distance_m,
            "lateral_center_m": lateral_center_m,
            "yaw_deg": yaw_deg,
            "confidence": confidence,
            "warn_reasons": tuple(warn_reasons),
            "yaw_shadow": yaw_shadow,
            "lateral_shadow": lateral_shadow,
            "sample_count": len(samples),
            "cluster_points": cluster_points,
            "roi_points": roi_points,
            "lateral_span_m": lateral_span_m,
            "lateral_sample_min_m": lateral_sample_min_m,
            "lateral_sample_max_m": lateral_sample_max_m,
            "lateral_sample_span_m": lateral_sample_span_m,
            "lateral_sample_robust_span_m": lateral_sample_robust_span_m,
            "lateral_sample_mad_m": lateral_sample_mad_m,
            "fit_residual_m": fit_residual_m,
            "pose_roi": self.rack_pose_roi_kwargs(target_mm),
        }

    def build_rack_yaw_shadow_decision(
        self,
        *,
        label: str,
        target_mm: int | None,
        distance_m: float,
        yaw_deg: float | None,
        confidence: float,
        fit_residual_m: float | None,
    ):
        phase = (
            "before_approach"
            if "before_approach" in label
            else "after_approach"
            if "after_approach" in label
            else "other"
        )
        reasons = []
        if phase != "before_approach":
            reasons.append("phase_not_before_approach")
        if distance_m < self.config.rack_yaw_shadow_min_distance_m:
            reasons.append("distance_too_close")
        if distance_m > self.config.rack_yaw_shadow_max_distance_m:
            reasons.append("distance_too_far")
        if confidence < self.config.rack_yaw_shadow_min_confidence:
            reasons.append("confidence_too_low")
        if fit_residual_m is None:
            reasons.append("fit_residual_unavailable")
        elif fit_residual_m > self.config.rack_yaw_shadow_max_fit_residual_m:
            reasons.append("fit_residual_too_high")

        candidate_robot_yaw_correction_deg = None
        if yaw_deg is None:
            reasons.append("yaw_unavailable")
        else:
            yaw_abs = abs(yaw_deg)
            if yaw_abs < self.config.rack_yaw_shadow_trigger_deg:
                reasons.append("yaw_below_trigger")
            if yaw_abs > self.config.rack_yaw_shadow_max_deg:
                reasons.append("yaw_too_large_for_shadow")

        blocking_reasons = [reason for reason in reasons if reason != "yaw_below_trigger"]
        if blocking_reasons:
            decision = "rejected"
        elif "yaw_below_trigger" in reasons:
            decision = "no_correction_needed"
        else:
            decision = "candidate"
            # The sign is intentionally still marked uncalibrated. Shadow mode logs the
            # proposed robot correction without sending motion so the next live runs can
            # verify this against observed rack pose changes.
            candidate_robot_yaw_correction_deg = round(-float(yaw_deg), 3)

        return {
            "mode": "shadow",
            "decision": decision,
            "phase": phase,
            "distance_m": distance_m,
            "yaw_deg": yaw_deg,
            "confidence": confidence,
            "fit_residual_m": fit_residual_m,
            "candidate_robot_yaw_correction_deg": candidate_robot_yaw_correction_deg,
            "candidate_sign_calibrated": False,
            "reasons": tuple(reasons),
            "thresholds": {
                "min_distance_m": self.config.rack_yaw_shadow_min_distance_m,
                "max_distance_m": self.config.rack_yaw_shadow_max_distance_m,
                "min_confidence": self.config.rack_yaw_shadow_min_confidence,
                "max_fit_residual_m": self.config.rack_yaw_shadow_max_fit_residual_m,
                "trigger_yaw_deg": self.config.rack_yaw_shadow_trigger_deg,
                "max_yaw_deg": self.config.rack_yaw_shadow_max_deg,
            },
        }

    def build_rack_lateral_shadow_decision(
        self,
        *,
        label: str,
        target_mm: int | None,
        distance_m: float,
        lateral_center_m: float,
        confidence: float,
        fit_residual_m: float | None,
    ):
        phase = (
            "before_approach"
            if "before_approach" in label
            else "after_approach"
            if "after_approach" in label
            else "other"
        )
        reasons = []
        if phase != "before_approach":
            reasons.append("phase_not_before_approach")
        if distance_m < self.config.rack_lateral_shadow_min_distance_m:
            reasons.append("distance_too_close")
        if distance_m > self.config.rack_lateral_shadow_max_distance_m:
            reasons.append("distance_too_far")
        if confidence < self.config.rack_lateral_shadow_min_confidence:
            reasons.append("confidence_too_low")
        if fit_residual_m is None:
            reasons.append("fit_residual_unavailable")
        elif fit_residual_m > self.config.rack_lateral_shadow_max_fit_residual_m:
            reasons.append("fit_residual_too_high")

        lateral_abs = abs(lateral_center_m)
        candidate_body_lateral_correction_m = None
        if lateral_abs < self.config.rack_lateral_shadow_trigger_m:
            reasons.append("lateral_below_trigger")
        if lateral_abs > self.config.rack_lateral_shadow_max_m:
            reasons.append("lateral_too_large_for_shadow")

        blocking_reasons = [
            reason
            for reason in reasons
            if reason != "lateral_below_trigger"
        ]
        if blocking_reasons:
            decision = "rejected"
        elif "lateral_below_trigger" in reasons:
            decision = "no_correction_needed"
        else:
            decision = "candidate"
            raw_correction_m = -float(lateral_center_m)
            max_correction_m = self.config.rack_lateral_shadow_max_correction_m
            candidate_body_lateral_correction_m = round(
                max(-max_correction_m, min(max_correction_m, raw_correction_m)),
                4,
            )

        execution_blockers = (
            "active_lateral_control_disabled",
            "linear_y_direction_gain_not_sweep_calibrated",
        )
        return {
            "mode": "shadow",
            "decision": decision,
            "phase": phase,
            "distance_m": distance_m,
            "lateral_center_m": lateral_center_m,
            "confidence": confidence,
            "fit_residual_m": fit_residual_m,
            "candidate_body_lateral_correction_m": candidate_body_lateral_correction_m,
            "candidate_execution_allowed": False,
            "candidate_sign_calibrated": False,
            "execution_blockers": execution_blockers,
            "reasons": tuple(reasons),
            "thresholds": {
                "min_distance_m": self.config.rack_lateral_shadow_min_distance_m,
                "max_distance_m": self.config.rack_lateral_shadow_max_distance_m,
                "min_confidence": self.config.rack_lateral_shadow_min_confidence,
                "max_fit_residual_m": self.config.rack_lateral_shadow_max_fit_residual_m,
                "trigger_lateral_m": self.config.rack_lateral_shadow_trigger_m,
                "max_lateral_m": self.config.rack_lateral_shadow_max_m,
                "max_correction_m": self.config.rack_lateral_shadow_max_correction_m,
            },
        }

    def read_rack_pose_summary_for_centering(
        self,
        rack,
        *,
        label: str,
        target_mm: int | None,
    ):
        samples = []
        errors = []
        for sample_index in range(1, self.config.rack_pose_samples + 1):
            try:
                pose = rack.lidar.read_rack_pose(**self.rack_pose_roi_kwargs(target_mm))
            except Exception as exc:
                pose = None
                errors.append(f"{sample_index}:{type(exc).__name__}:{exc}")

            if pose is not None:
                samples.append(pose)
            if sample_index < self.config.rack_pose_samples:
                time.sleep(self.config.rack_pose_interval_s)

        if not samples:
            self.log(
                "rack_lateral_centering_pose_unavailable "
                f"label={label} target_mm={target_mm} "
                f"samples=0 requested_samples={self.config.rack_pose_samples} "
                f"errors={tuple(errors[-3:])}"
            )
            self.emit_event(
                "rack_lateral_centering_pose",
                step_no=self.step_no,
                rod_index=self.current_rod_index,
                label=label,
                target_mm=target_mm,
                status="unavailable",
                sample_count=0,
                requested_samples=self.config.rack_pose_samples,
                errors=tuple(errors[-3:]),
            )
            return None

        lateral_sample_values = [float(pose.lateral_center_m) for pose in samples]
        lateral_stats = lateral_sample_stats(lateral_sample_values)
        lateral_sample_min_m = lateral_stats["min_m"]
        lateral_sample_max_m = lateral_stats["max_m"]
        lateral_sample_span_m = lateral_stats["span_m"]
        lateral_sample_robust_span_m = lateral_stats["robust_span_m"]
        lateral_sample_mad_m = lateral_stats["mad_m"]
        distance_m = float(statistics.median(pose.distance_m for pose in samples))
        lateral_center_m = float(statistics.median(lateral_sample_values))
        confidence = float(statistics.median(pose.confidence for pose in samples))
        cluster_points = int(statistics.median(pose.cluster_points for pose in samples))
        roi_points = int(statistics.median(pose.roi_points for pose in samples))
        lateral_span_m = float(statistics.median(pose.lateral_span_m for pose in samples))
        residual_values = [
            pose.fit_residual_m for pose in samples if pose.fit_residual_m is not None
        ]
        fit_residual_m = (
            float(statistics.median(residual_values)) if residual_values else None
        )
        yaw_values = [pose.yaw_deg for pose in samples if pose.yaw_deg is not None]
        yaw_deg = float(statistics.median(yaw_values)) if yaw_values else None

        summary = {
            "status": "ok",
            "distance_m": distance_m,
            "lateral_center_m": lateral_center_m,
            "yaw_deg": yaw_deg,
            "confidence": confidence,
            "sample_count": len(samples),
            "requested_samples": self.config.rack_pose_samples,
            "cluster_points": cluster_points,
            "roi_points": roi_points,
            "lateral_span_m": lateral_span_m,
            "lateral_sample_min_m": lateral_sample_min_m,
            "lateral_sample_max_m": lateral_sample_max_m,
            "lateral_sample_span_m": lateral_sample_span_m,
            "lateral_sample_robust_min_m": lateral_stats["robust_min_m"],
            "lateral_sample_robust_max_m": lateral_stats["robust_max_m"],
            "lateral_sample_robust_span_m": lateral_sample_robust_span_m,
            "lateral_sample_mad_m": lateral_sample_mad_m,
            "lateral_sample_trim_count": lateral_stats["trim_count"],
            "fit_residual_m": fit_residual_m,
            "pose_roi": self.rack_pose_roi_kwargs(target_mm),
        }
        self.log(
            "rack_lateral_centering_pose "
            f"label={label} target_mm={target_mm} "
            f"distance_m={distance_m:.3f} lateral_center_m={lateral_center_m:.3f} "
            f"yaw_deg={None if yaw_deg is None else round(yaw_deg, 3)} "
            f"confidence={confidence:.3f} sample_count={len(samples)} "
            f"cluster_points={cluster_points} roi_points={roi_points} "
            f"lateral_span_m={lateral_span_m:.3f} "
            f"lateral_sample_span_m={lateral_sample_span_m:.3f} "
            f"lateral_sample_robust_span_m={lateral_sample_robust_span_m:.3f} "
            f"lateral_sample_mad_m={lateral_sample_mad_m:.3f} "
            f"fit_residual_m={None if fit_residual_m is None else round(fit_residual_m, 4)}"
        )
        self.emit_event(
            "rack_lateral_centering_pose",
            step_no=self.step_no,
            rod_index=self.current_rod_index,
            label=label,
            target_mm=target_mm,
            **summary,
        )
        return summary

    def build_rack_lateral_active_decision(
        self,
        *,
        label: str,
        pose,
    ):
        phase = (
            "before_approach"
            if "before_approach" in label
            else "after_approach"
            if "after_approach" in label
            else "other"
        )
        reasons = []
        if phase != "before_approach":
            reasons.append("phase_not_before_approach")
        if pose is None:
            reasons.append("pose_unavailable")
            return {
                "decision": "blocked",
                "phase": phase,
                "reasons": tuple(reasons),
                "distance_m": None,
                "lateral_center_m": None,
                "yaw_deg": None,
                "confidence": None,
                "fit_residual_m": None,
                "vy_mps": None,
            }

        distance_m = pose.get("distance_m")
        lateral_center_m = pose.get("lateral_center_m")
        yaw_deg = pose.get("yaw_deg")
        confidence = pose.get("confidence")
        fit_residual_m = pose.get("fit_residual_m")
        lateral_sample_span_m = pose.get("lateral_sample_span_m")
        lateral_sample_robust_span_m = pose.get("lateral_sample_robust_span_m")
        lateral_sample_stability_span_m = lateral_sample_robust_span_m
        if lateral_sample_stability_span_m is None:
            lateral_sample_stability_span_m = lateral_sample_span_m
        if distance_m is None:
            reasons.append("distance_unavailable")
        elif distance_m < self.config.rack_lateral_shadow_min_distance_m:
            reasons.append("distance_too_close")
        elif distance_m > self.config.rack_lateral_shadow_max_distance_m:
            reasons.append("distance_too_far")
        if lateral_center_m is None:
            reasons.append("lateral_unavailable")
        if confidence is None:
            reasons.append("confidence_unavailable")
        elif confidence < self.config.rack_lateral_shadow_min_confidence:
            reasons.append("confidence_too_low")
        if fit_residual_m is None:
            reasons.append("fit_residual_unavailable")
        elif fit_residual_m > self.config.rack_lateral_shadow_max_fit_residual_m:
            reasons.append("fit_residual_too_high")
        if lateral_sample_stability_span_m is None:
            reasons.append("lateral_sample_span_unavailable")
        elif lateral_sample_stability_span_m > self.config.rack_lateral_active_max_sample_span_m:
            reasons.append("lateral_sample_unstable")
        if yaw_deg is not None and abs(yaw_deg) > self.config.rack_lateral_active_max_yaw_deg:
            reasons.append("yaw_too_large_for_lateral_active")

        vy_mps = None
        if lateral_center_m is not None:
            lateral_abs = abs(float(lateral_center_m))
            if lateral_abs <= self.config.rack_lateral_active_target_m:
                reasons.append("lateral_within_active_target")
            elif lateral_abs > self.config.rack_lateral_active_max_initial_m:
                reasons.append("lateral_too_large_for_active")
            else:
                if self.config.rack_lateral_active_direction == "disabled":
                    reasons.append("linear_y_direction_not_calibrated")
                else:
                    direction_sign = (
                        1.0
                        if self.config.rack_lateral_active_direction == "same-sign"
                        else -1.0
                    )
                    vy_mps = direction_sign * math.copysign(
                        self.config.rack_lateral_active_speed_mps,
                        float(lateral_center_m),
                    )

        blocking_reasons = [
            reason
            for reason in reasons
            if reason != "lateral_within_active_target"
        ]
        if blocking_reasons:
            decision = "blocked"
            vy_mps = None
        elif "lateral_within_active_target" in reasons:
            decision = "centered"
        else:
            decision = "step"

        return {
            "decision": decision,
            "phase": phase,
            "reasons": tuple(reasons),
            "distance_m": distance_m,
            "lateral_center_m": lateral_center_m,
            "yaw_deg": yaw_deg,
            "confidence": confidence,
            "fit_residual_m": fit_residual_m,
            "lateral_sample_span_m": lateral_sample_span_m,
            "lateral_sample_robust_span_m": lateral_sample_robust_span_m,
            "lateral_sample_stability_span_m": lateral_sample_stability_span_m,
            "vy_mps": vy_mps,
            "thresholds": {
                "target_lateral_m": self.config.rack_lateral_active_target_m,
                "max_initial_lateral_m": self.config.rack_lateral_active_max_initial_m,
                "max_yaw_deg": self.config.rack_lateral_active_max_yaw_deg,
                "min_distance_m": self.config.rack_lateral_shadow_min_distance_m,
                "max_distance_m": self.config.rack_lateral_shadow_max_distance_m,
                "min_confidence": self.config.rack_lateral_shadow_min_confidence,
                "max_fit_residual_m": self.config.rack_lateral_shadow_max_fit_residual_m,
                "max_sample_span_m": self.config.rack_lateral_active_max_sample_span_m,
                "sample_span_metric": "lateral_sample_robust_span_m",
                "active_direction": self.config.rack_lateral_active_direction,
            },
        }

    def _read_lateral_centering_ultrasonic_snapshot(self, rack):
        from rack_radar_docking import INVALID_DISTANCE_MM

        groups = {
            "front": FRONT_ULTRASONIC_IDS,
            "right": RIGHT_ULTRASONIC_IDS,
            "rear": REAR_ULTRASONIC_IDS,
            "left": LEFT_ULTRASONIC_IDS,
        }
        data = rack.front.radar.get_latest_ultrasonic_radar()
        all_distances = {}
        for row in data.get("ultrasonic_radar_datas", []):
            radar_id = row.get("id")
            distance_mm = row.get("distance_mm")
            fault_state = row.get("fault_state")
            if fault_state != 0:
                continue
            distance_mm = self._valid_ultrasonic_distance_mm(
                distance_mm,
                min_valid_mm=50,
                invalid_distance_mm=INVALID_DISTANCE_MM,
            )
            if distance_mm is not None:
                all_distances[radar_id] = distance_mm

        snapshot = {}
        for name, ids in groups.items():
            raw = tuple(
                (radar_id, all_distances[radar_id])
                for radar_id in ids
                if radar_id in all_distances
            )
            min_mm = min((distance for _, distance in raw), default=None)
            snapshot[name] = (min_mm, raw)
        return snapshot

    def _read_lateral_centering_ultrasonic_window(self, rack):
        snapshots = []
        values = {name: [] for name in ("front", "right", "rear", "left")}
        for sample_index in range(1, self.config.rack_lateral_active_clearance_samples + 1):
            snapshot = self._read_lateral_centering_ultrasonic_snapshot(rack)
            snapshots.append(snapshot)
            for name in values:
                min_mm = snapshot[name][0]
                if min_mm is not None:
                    values[name].append(float(min_mm))
            if sample_index < self.config.rack_lateral_active_clearance_samples:
                time.sleep(self.config.rack_lateral_active_clearance_interval_s)

        groups = {}
        for name, group_values in values.items():
            groups[name] = {
                "median_mm": (
                    float(statistics.median(group_values)) if group_values else None
                ),
                "raw_min_mm": min(group_values) if group_values else None,
                "samples_mm": tuple(group_values),
                "last_raw": snapshots[-1][name][1] if snapshots else (),
            }
        return {
            "groups": groups,
            "snapshots": snapshots,
        }

    def _check_lateral_centering_motion_safety(self, rack, *, label: str, vy_mps: float):
        preflight = rack.preflight(
            allow_estop_pedal_fault=self.config.allow_estop_pedal_fault
        )
        problems = []
        if preflight.status != "ok":
            problems.append(f"preflight={preflight}")

        ultrasonic_window = self._read_lateral_centering_ultrasonic_window(rack)
        ultrasonic_groups = ultrasonic_window["groups"]
        median_thresholds = {
            "front": self.config.rack_lateral_active_min_front_mm,
            "rear": self.config.rack_lateral_active_min_rear_mm,
            "left": self.config.rack_lateral_active_min_side_mm,
            "right": self.config.rack_lateral_active_min_side_mm,
        }
        hard_thresholds = {
            "front": self.config.rack_lateral_active_hard_min_front_mm,
            "rear": self.config.rack_lateral_active_hard_min_rear_mm,
            "left": self.config.rack_lateral_active_hard_min_side_mm,
            "right": self.config.rack_lateral_active_hard_min_side_mm,
        }
        for name in ("front", "rear", "left", "right"):
            median_mm = ultrasonic_groups[name]["median_mm"]
            raw_min_mm = ultrasonic_groups[name]["raw_min_mm"]
            median_threshold = median_thresholds[name]
            hard_threshold = hard_thresholds[name]
            if median_mm is None:
                problems.append(f"{name}_ultrasonic_unavailable")
            elif median_mm < median_threshold:
                problems.append(
                    f"{name}_ultrasonic_median_too_close={median_mm}<min_{median_threshold}"
                )
            if raw_min_mm is None:
                problems.append(f"{name}_ultrasonic_raw_min_unavailable")
            elif raw_min_mm < hard_threshold:
                problems.append(
                    f"{name}_ultrasonic_raw_min_too_close={raw_min_mm}<hard_min_{hard_threshold}"
                )

        self.log(
            "rack_lateral_centering_safety "
            f"label={label} vy_mps={vy_mps:.3f} "
            f"front={ultrasonic_groups['front']} "
            f"right={ultrasonic_groups['right']} "
            f"rear={ultrasonic_groups['rear']} "
            f"left={ultrasonic_groups['left']} "
            f"problems={tuple(problems)}"
        )
        self.emit_event(
            "rack_lateral_centering_safety",
            step_no=self.step_no,
            rod_index=self.current_rod_index,
            label=label,
            vy_mps=vy_mps,
            front_median_mm=ultrasonic_groups["front"]["median_mm"],
            front_raw_min_mm=ultrasonic_groups["front"]["raw_min_mm"],
            front_samples_mm=ultrasonic_groups["front"]["samples_mm"],
            front_raw=ultrasonic_groups["front"]["last_raw"],
            right_median_mm=ultrasonic_groups["right"]["median_mm"],
            right_raw_min_mm=ultrasonic_groups["right"]["raw_min_mm"],
            right_samples_mm=ultrasonic_groups["right"]["samples_mm"],
            right_raw=ultrasonic_groups["right"]["last_raw"],
            rear_median_mm=ultrasonic_groups["rear"]["median_mm"],
            rear_raw_min_mm=ultrasonic_groups["rear"]["raw_min_mm"],
            rear_samples_mm=ultrasonic_groups["rear"]["samples_mm"],
            rear_raw=ultrasonic_groups["rear"]["last_raw"],
            left_median_mm=ultrasonic_groups["left"]["median_mm"],
            left_raw_min_mm=ultrasonic_groups["left"]["raw_min_mm"],
            left_samples_mm=ultrasonic_groups["left"]["samples_mm"],
            left_raw=ultrasonic_groups["left"]["last_raw"],
            problems=tuple(problems),
            thresholds={
                "min_front_mm": self.config.rack_lateral_active_min_front_mm,
                "min_rear_mm": self.config.rack_lateral_active_min_rear_mm,
                "min_side_mm": self.config.rack_lateral_active_min_side_mm,
                "hard_min_front_mm": self.config.rack_lateral_active_hard_min_front_mm,
                "hard_min_rear_mm": self.config.rack_lateral_active_hard_min_rear_mm,
                "hard_min_side_mm": self.config.rack_lateral_active_hard_min_side_mm,
                "clearance_samples": self.config.rack_lateral_active_clearance_samples,
                "clearance_interval_s": self.config.rack_lateral_active_clearance_interval_s,
            },
        )
        if problems:
            raise RuntimeError(
                "rack lateral centering blocked by motion safety: "
                + ", ".join(problems)
            )

    def _guarded_front_ultrasonic_centering_override(
        self,
        rack,
        *,
        label: str,
        target_mm: int | None,
        pose,
        decision,
    ):
        allowed_pose_reasons = {
            "lateral_sample_unstable",
            "yaw_too_large_for_lateral_active",
            "lateral_within_active_target",
        }
        reasons = set(decision.get("reasons", ()))
        blockers = tuple(sorted(reasons - allowed_pose_reasons))
        lateral_center_m = decision.get("lateral_center_m")
        problems = []
        if not self.config.rack_guarded_ultrasonic_override:
            problems.append("override_disabled")
        if pose is None:
            problems.append("pose_unavailable")
        if blockers:
            problems.append(f"pose_blockers={blockers}")
        if lateral_center_m is None:
            problems.append("lateral_unavailable")
        elif abs(float(lateral_center_m)) > self.config.rack_guarded_ultrasonic_max_pose_lateral_m:
            problems.append(
                "pose_lateral_too_large="
                f"{float(lateral_center_m):.3f}>"
                f"{self.config.rack_guarded_ultrasonic_max_pose_lateral_m:.3f}"
            )

        front_pairs = []
        front_spans_mm = []
        front_mins_mm = []
        latest_front_raw = ()
        for sample_index in range(1, self.config.rack_lateral_active_clearance_samples + 1):
            snapshot = self._read_lateral_centering_ultrasonic_snapshot(rack)
            front_raw = snapshot["front"][1]
            latest_front_raw = front_raw
            front_by_id = dict(front_raw)
            if all(radar_id in front_by_id for radar_id in FRONT_ULTRASONIC_IDS):
                values = [int(front_by_id[radar_id]) for radar_id in FRONT_ULTRASONIC_IDS]
                front_pairs.append(tuple(values))
                front_spans_mm.append(max(values) - min(values))
                front_mins_mm.append(min(values))
            if sample_index < self.config.rack_lateral_active_clearance_samples:
                time.sleep(self.config.rack_lateral_active_clearance_interval_s)

        valid_pair_count = len(front_pairs)
        median_span_mm = (
            int(statistics.median(front_spans_mm)) if front_spans_mm else None
        )
        max_span_mm = max(front_spans_mm) if front_spans_mm else None
        min_front_mm = min(front_mins_mm) if front_mins_mm else None
        required_pairs = max(3, min(5, self.config.rack_lateral_active_clearance_samples))
        if valid_pair_count < required_pairs:
            problems.append(f"front_pair_samples_insufficient={valid_pair_count}<{required_pairs}")
        if min_front_mm is None:
            problems.append("front_ultrasonic_unavailable")
        elif min_front_mm < self.config.rack_guarded_ultrasonic_min_front_mm:
            problems.append(
                "front_ultrasonic_too_close="
                f"{min_front_mm}<min_{self.config.rack_guarded_ultrasonic_min_front_mm}"
            )
        if median_span_mm is None:
            problems.append("front_ultrasonic_span_unavailable")
        elif median_span_mm > self.config.rack_guarded_ultrasonic_max_front_span_mm:
            problems.append(
                "front_ultrasonic_median_span_too_large="
                f"{median_span_mm}>{self.config.rack_guarded_ultrasonic_max_front_span_mm}"
            )
        if max_span_mm is not None and max_span_mm > self.config.rack_guarded_ultrasonic_max_front_span_mm + 20:
            problems.append(
                "front_ultrasonic_max_span_too_large="
                f"{max_span_mm}>{self.config.rack_guarded_ultrasonic_max_front_span_mm + 20}"
            )

        status = "accepted" if not problems else "rejected"
        self.log(
            "rack_guarded_ultrasonic_override "
            f"label={label} target_mm={target_mm} status={status} "
            f"decision_reasons={decision.get('reasons')} blockers={blockers} "
            f"lateral_center_m={lateral_center_m} "
            f"front_pairs={tuple(front_pairs)} median_span_mm={median_span_mm} "
            f"max_span_mm={max_span_mm} min_front_mm={min_front_mm} "
            f"latest_front_raw={latest_front_raw} problems={tuple(problems)}"
        )
        self.emit_event(
            "rack_guarded_ultrasonic_override",
            step_no=self.step_no,
            rod_index=self.current_rod_index,
            label=label,
            target_mm=target_mm,
            status=status,
            decision_reasons=decision.get("reasons"),
            blockers=blockers,
            lateral_center_m=lateral_center_m,
            front_pairs=tuple(front_pairs),
            median_span_mm=median_span_mm,
            max_span_mm=max_span_mm,
            min_front_mm=min_front_mm,
            latest_front_raw=latest_front_raw,
            problems=tuple(problems),
            thresholds={
                "max_pose_lateral_m": self.config.rack_guarded_ultrasonic_max_pose_lateral_m,
                "max_front_span_mm": self.config.rack_guarded_ultrasonic_max_front_span_mm,
                "min_front_mm": self.config.rack_guarded_ultrasonic_min_front_mm,
                "required_pairs": required_pairs,
            },
        )
        return {
            "status": status,
            "problems": tuple(problems),
            "front_pairs": tuple(front_pairs),
            "median_span_mm": median_span_mm,
            "max_span_mm": max_span_mm,
            "min_front_mm": min_front_mm,
        }

    def _read_front_pair_window_for_guard(self, rack):
        front_pairs = []
        front_spans_mm = []
        front_mins_mm = []
        latest_front_raw = ()
        for sample_index in range(1, self.config.rack_lateral_active_clearance_samples + 1):
            snapshot = self._read_lateral_centering_ultrasonic_snapshot(rack)
            front_raw = snapshot["front"][1]
            latest_front_raw = front_raw
            front_by_id = dict(front_raw)
            if all(radar_id in front_by_id for radar_id in FRONT_ULTRASONIC_IDS):
                values = [int(front_by_id[radar_id]) for radar_id in FRONT_ULTRASONIC_IDS]
                front_pairs.append(tuple(values))
                front_spans_mm.append(max(values) - min(values))
                front_mins_mm.append(min(values))
            if sample_index < self.config.rack_lateral_active_clearance_samples:
                time.sleep(self.config.rack_lateral_active_clearance_interval_s)

        return {
            "front_pairs": tuple(front_pairs),
            "valid_pair_count": len(front_pairs),
            "median_span_mm": (
                int(statistics.median(front_spans_mm)) if front_spans_mm else None
            ),
            "max_span_mm": max(front_spans_mm) if front_spans_mm else None,
            "min_front_mm": min(front_mins_mm) if front_mins_mm else None,
            "latest_front_raw": latest_front_raw,
        }

    def guarded_rack_post_approach_check(
        self,
        rack,
        *,
        label: str,
        target_mm: int | None,
        min_safe_mm: int,
        pose,
    ):
        if self.config.rack_centering_mode not in ("guarded", "active"):
            return None
        if not self.config.rack_post_approach_guarded_check:
            return None

        required_pairs = max(3, min(5, self.config.rack_lateral_active_clearance_samples))

        def build_front_problems(front_window: dict) -> list[str]:
            problems = []
            if front_window["valid_pair_count"] < required_pairs:
                problems.append(
                    "front_pair_samples_insufficient="
                    f"{front_window['valid_pair_count']}<{required_pairs}"
                )
            if front_window["min_front_mm"] is None:
                problems.append("front_ultrasonic_unavailable")
            elif front_window["min_front_mm"] < min_safe_mm:
                problems.append(
                    "front_ultrasonic_too_close="
                    f"{front_window['min_front_mm']}<min_safe_{min_safe_mm}"
                )
            if front_window["median_span_mm"] is None:
                problems.append("front_ultrasonic_span_unavailable")
            elif (
                front_window["median_span_mm"]
                > self.config.rack_post_approach_max_front_span_mm
            ):
                problems.append(
                    "front_ultrasonic_median_span_too_large="
                    f"{front_window['median_span_mm']}>"
                    f"{self.config.rack_post_approach_max_front_span_mm}"
                )
            max_span_mm = front_window["max_span_mm"]
            if (
                max_span_mm is not None
                and max_span_mm > self.config.rack_post_approach_max_front_span_mm + 30
            ):
                problems.append(
                    "front_ultrasonic_max_span_too_large="
                    f"{max_span_mm}>{self.config.rack_post_approach_max_front_span_mm + 30}"
                )
            return problems

        front_window = None
        front_problems = []
        for retry_index in range(self.config.rack_post_approach_front_retry_windows + 1):
            front_window = self._read_front_pair_window_for_guard(rack)
            front_problems = build_front_problems(front_window)
            max_span_only = (
                len(front_problems) == 1
                and front_problems[0].startswith("front_ultrasonic_max_span_too_large=")
            )
            if not max_span_only or retry_index >= self.config.rack_post_approach_front_retry_windows:
                break
            self.log(
                "rack_post_approach_front_window_retry "
                f"label={label} target_mm={target_mm} retry={retry_index + 1} "
                f"front_window={front_window} front_problems={tuple(front_problems)}"
            )
            self.emit_event(
                "rack_post_approach_front_window_retry",
                step_no=self.step_no,
                rod_index=self.current_rod_index,
                label=label,
                target_mm=target_mm,
                retry_index=retry_index + 1,
                front_window=front_window,
                front_problems=tuple(front_problems),
            )
            time.sleep(self.config.rack_lateral_active_clearance_interval_s)

        pose_reliability_problems = []
        pose_problems = []
        if pose is None:
            pose_reliability_problems.append("pose_unavailable")
            pose_status = "unavailable"
            distance_m = None
            lateral_center_m = None
            yaw_deg = None
            confidence = None
            fit_residual_m = None
            lateral_sample_stability_span_m = None
        else:
            pose_status = pose.get("status")
            distance_m = pose.get("distance_m")
            lateral_center_m = pose.get("lateral_center_m")
            yaw_deg = pose.get("yaw_deg")
            confidence = pose.get("confidence")
            fit_residual_m = pose.get("fit_residual_m")
            lateral_sample_stability_span_m = pose.get("lateral_sample_robust_span_m")
            if lateral_sample_stability_span_m is None:
                lateral_sample_stability_span_m = pose.get("lateral_sample_span_m")
            if pose.get("sample_count", 0) < max(2, min(3, self.config.rack_pose_samples)):
                pose_reliability_problems.append("pose_sample_count_too_low")
            if distance_m is None:
                pose_reliability_problems.append("distance_unavailable")
            elif distance_m < self.config.rack_lateral_shadow_min_distance_m:
                pose_reliability_problems.append("distance_too_close")
            elif distance_m > self.config.rack_lateral_shadow_max_distance_m:
                pose_reliability_problems.append("distance_too_far")
            if lateral_center_m is None:
                pose_reliability_problems.append("lateral_unavailable")
            if confidence is None:
                pose_reliability_problems.append("confidence_unavailable")
            elif confidence < self.config.rack_lateral_shadow_min_confidence:
                pose_reliability_problems.append("confidence_too_low")
            if fit_residual_m is None:
                pose_reliability_problems.append("fit_residual_unavailable")
            elif fit_residual_m > self.config.rack_lateral_shadow_max_fit_residual_m:
                pose_reliability_problems.append("fit_residual_too_high")
            if lateral_sample_stability_span_m is None:
                pose_reliability_problems.append("lateral_sample_span_unavailable")
            elif (
                lateral_sample_stability_span_m
                > self.config.rack_lateral_active_max_sample_span_m
            ):
                pose_reliability_problems.append("lateral_sample_unstable")

            if lateral_center_m is not None and (
                abs(float(lateral_center_m))
                > self.config.rack_post_approach_max_lateral_m
            ):
                pose_problems.append(
                    "post_lateral_too_large="
                    f"{float(lateral_center_m):.3f}>"
                    f"{self.config.rack_post_approach_max_lateral_m:.3f}"
                )
            if yaw_deg is not None and (
                abs(float(yaw_deg)) > self.config.rack_post_approach_max_yaw_deg
            ):
                pose_problems.append(
                    "post_yaw_too_large="
                    f"{float(yaw_deg):.3f}>"
                    f"{self.config.rack_post_approach_max_yaw_deg:.3f}"
                )

        pose_reliable = not pose_reliability_problems
        if front_problems:
            status = "blocked_front_ultrasonic"
        elif pose_reliable and pose_problems:
            status = "blocked_pose_offset"
        elif pose_reliable:
            status = "verified_centered"
        else:
            status = "inconclusive_ultrasonic_verified"

        self.log(
            "rack_post_approach_guarded_check "
            f"label={label} target_mm={target_mm} status={status} "
            f"pose_status={pose_status} pose_reliable={pose_reliable} "
            f"distance_m={distance_m} lateral_center_m={lateral_center_m} "
            f"yaw_deg={yaw_deg} confidence={confidence} "
            f"fit_residual_m={fit_residual_m} "
            f"lateral_sample_stability_span_m={lateral_sample_stability_span_m} "
            f"pose_reliability_problems={tuple(pose_reliability_problems)} "
            f"pose_problems={tuple(pose_problems)} "
            f"front_window={front_window} front_problems={tuple(front_problems)}"
        )
        self.emit_event(
            "rack_post_approach_guarded_check",
            step_no=self.step_no,
            rod_index=self.current_rod_index,
            label=label,
            target_mm=target_mm,
            status=status,
            pose_status=pose_status,
            pose_reliable=pose_reliable,
            distance_m=distance_m,
            lateral_center_m=lateral_center_m,
            yaw_deg=yaw_deg,
            confidence=confidence,
            fit_residual_m=fit_residual_m,
            lateral_sample_stability_span_m=lateral_sample_stability_span_m,
            pose_reliability_problems=tuple(pose_reliability_problems),
            pose_problems=tuple(pose_problems),
            front_window=front_window,
            front_problems=tuple(front_problems),
            thresholds={
                "max_lateral_m": self.config.rack_post_approach_max_lateral_m,
                "max_yaw_deg": self.config.rack_post_approach_max_yaw_deg,
                "max_front_span_mm": self.config.rack_post_approach_max_front_span_mm,
                "min_safe_mm": min_safe_mm,
                "required_front_pairs": required_pairs,
                "min_confidence": self.config.rack_lateral_shadow_min_confidence,
                "max_fit_residual_m": self.config.rack_lateral_shadow_max_fit_residual_m,
                "max_sample_span_m": self.config.rack_lateral_active_max_sample_span_m,
            },
        )
        if status.startswith("blocked"):
            raise RuntimeError(
                "rack post-approach guarded check blocked: "
                f"label={label}, status={status}, "
                f"pose_problems={tuple(pose_problems)}, "
                f"front_problems={tuple(front_problems)}, "
                f"pose_reliability_problems={tuple(pose_reliability_problems)}"
            )
        return {
            "status": status,
            "pose_reliable": pose_reliable,
            "pose_problems": tuple(pose_problems),
            "pose_reliability_problems": tuple(pose_reliability_problems),
            "front_problems": tuple(front_problems),
        }

    def _send_rack_lateral_velocity_step(
        self,
        rack,
        *,
        label: str,
        vy_mps: float,
        duration_s: float,
    ):
        import agibot_gdk

        def make_twist(linear_y: float):
            twist = agibot_gdk.Twist()
            twist.linear = agibot_gdk.Vector3()
            twist.angular = agibot_gdk.Vector3()
            twist.linear.x = 0.0
            twist.linear.y = float(linear_y)
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            return twist

        self._check_lateral_centering_motion_safety(
            rack,
            label=label,
            vy_mps=vy_mps,
        )
        rack.front.request_chassis_control_ready()
        time.sleep(0.15)

        pnc = rack.front.pnc
        interval_s = 1.0 / self.config.rack_lateral_active_hz
        command_count = max(1, math.ceil(duration_s * self.config.rack_lateral_active_hz))
        twist = make_twist(vy_mps)
        stop = make_twist(0.0)
        start = time.time()
        try:
            for _ in range(command_count):
                pnc.move_chassis(twist)
                time.sleep(interval_s)
        finally:
            for _ in range(10):
                try:
                    pnc.move_chassis(stop)
                except Exception:
                    pass
                time.sleep(0.03)

        elapsed_s = time.time() - start
        self.log(
            "rack_lateral_centering_velocity_step_done "
            f"label={label} vy_mps={vy_mps:.3f} "
            f"duration_s={duration_s:.3f} commands={command_count} "
            f"elapsed_s={elapsed_s:.3f}"
        )
        self.emit_event(
            "rack_lateral_centering_velocity_step_done",
            step_no=self.step_no,
            rod_index=self.current_rod_index,
            label=label,
            vy_mps=vy_mps,
            duration_s=duration_s,
            commands=command_count,
            elapsed_s=round(elapsed_s, 4),
        )

    def center_rack_lateral_before_approach(
        self,
        rack,
        *,
        label: str,
        target_mm: int | None,
        initial_pose,
    ):
        if self.config.rack_centering_mode not in ("guarded", "active"):
            return initial_pose

        self.log(
            "rack_lateral_centering_start "
            f"label={label} target_mm={target_mm} mode={self.config.rack_centering_mode} "
            f"target_lateral_m={self.config.rack_lateral_active_target_m:.3f} "
            f"max_passes={self.config.rack_lateral_active_max_passes} "
            f"speed_mps={self.config.rack_lateral_active_speed_mps:.3f} "
            f"step_s={self.config.rack_lateral_active_step_s:.3f} "
            f"direction={self.config.rack_lateral_active_direction}"
        )
        self.emit_event(
            "rack_lateral_centering_start",
            step_no=self.step_no,
            rod_index=self.current_rod_index,
            label=label,
            target_mm=target_mm,
            mode=self.config.rack_centering_mode,
            target_lateral_m=self.config.rack_lateral_active_target_m,
            max_passes=self.config.rack_lateral_active_max_passes,
            speed_mps=self.config.rack_lateral_active_speed_mps,
            step_s=self.config.rack_lateral_active_step_s,
            direction=self.config.rack_lateral_active_direction,
        )

        pose = initial_pose
        if pose is None:
            pose = self.read_rack_pose_summary_for_centering(
                rack,
                label=f"{label}:active_initial",
                target_mm=target_mm,
            )

        for pass_index in range(1, self.config.rack_lateral_active_max_passes + 1):
            decision = self.build_rack_lateral_active_decision(
                label=label,
                pose=pose,
            )
            self.log(
                "rack_lateral_centering_decision "
                f"label={label} pass={pass_index} decision={decision['decision']} "
                f"lateral_center_m={decision['lateral_center_m']} "
                f"distance_m={decision['distance_m']} yaw_deg={decision['yaw_deg']} "
                f"confidence={decision['confidence']} "
                f"lateral_sample_stability_span_m={decision.get('lateral_sample_stability_span_m')} "
                f"vy_mps={decision['vy_mps']} reasons={decision['reasons']}"
            )
            self.emit_event(
                "rack_lateral_centering_decision",
                step_no=self.step_no,
                rod_index=self.current_rod_index,
                label=label,
                target_mm=target_mm,
                pass_index=pass_index,
                **decision,
            )

            if decision["decision"] == "centered":
                self.emit_event(
                    "rack_lateral_centering_result",
                    step_no=self.step_no,
                    rod_index=self.current_rod_index,
                    label=label,
                    target_mm=target_mm,
                    status="centered",
                    pass_index=pass_index - 1,
                    final_lateral_center_m=decision["lateral_center_m"],
                    target_lateral_m=self.config.rack_lateral_active_target_m,
                )
                self.log(
                    "rack_lateral_centering_result "
                    f"label={label} status=centered "
                    f"passes={pass_index - 1} "
                    f"final_lateral_center_m={decision['lateral_center_m']}"
                )
                return pose

            if self.config.rack_centering_mode == "guarded":
                ultrasonic_override = self._guarded_front_ultrasonic_centering_override(
                    rack,
                    label=label,
                    target_mm=target_mm,
                    pose=pose,
                    decision=decision,
                )
                if ultrasonic_override["status"] == "accepted":
                    self.emit_event(
                        "rack_lateral_centering_result",
                        step_no=self.step_no,
                        rod_index=self.current_rod_index,
                        label=label,
                        target_mm=target_mm,
                        status="guarded_ultrasonic_verified",
                        pass_index=pass_index - 1,
                        reasons=decision["reasons"],
                        lateral_center_m=decision["lateral_center_m"],
                        target_lateral_m=self.config.rack_lateral_active_target_m,
                        ultrasonic_override=ultrasonic_override,
                    )
                    self.log(
                        "rack_lateral_centering_result "
                        f"label={label} status=guarded_ultrasonic_verified "
                        f"lateral_center_m={decision['lateral_center_m']} "
                        f"reasons={decision['reasons']} "
                        f"ultrasonic_override={ultrasonic_override}"
                    )
                    return pose
                self.emit_event(
                    "rack_lateral_centering_result",
                    step_no=self.step_no,
                    rod_index=self.current_rod_index,
                    label=label,
                    target_mm=target_mm,
                    status="blocked_guarded_correction_required",
                    pass_index=pass_index - 1,
                    reasons=decision["reasons"],
                    lateral_center_m=decision["lateral_center_m"],
                    target_lateral_m=self.config.rack_lateral_active_target_m,
                    ultrasonic_override=ultrasonic_override,
                )
                raise RuntimeError(
                    "rack lateral centering guarded mode blocked before approach: "
                    f"label={label}, lateral_center_m={decision['lateral_center_m']}, "
                    f"target_lateral_m={self.config.rack_lateral_active_target_m}, "
                    f"reasons={decision['reasons']}, "
                    f"ultrasonic_override_problems={ultrasonic_override['problems']}"
                )

            if decision["decision"] == "blocked":
                self.emit_event(
                    "rack_lateral_centering_result",
                    step_no=self.step_no,
                    rod_index=self.current_rod_index,
                    label=label,
                    target_mm=target_mm,
                    status="blocked",
                    pass_index=pass_index - 1,
                    reasons=decision["reasons"],
                    lateral_center_m=decision["lateral_center_m"],
                )
                raise RuntimeError(
                    "rack lateral centering blocked before approach: "
                    f"label={label}, reasons={decision['reasons']}, "
                    f"lateral_center_m={decision['lateral_center_m']}"
                )

            before_lateral = float(decision["lateral_center_m"])
            before_abs = abs(before_lateral)
            vy_mps = float(decision["vy_mps"])
            step_label = f"{label}:active_pass_{pass_index}"
            self.log(
                "rack_lateral_centering_step_start "
                f"label={label} pass={pass_index} "
                f"before_lateral_center_m={before_lateral:.4f} "
                f"vy_mps={vy_mps:.3f} "
                f"duration_s={self.config.rack_lateral_active_step_s:.3f}"
            )
            self.emit_event(
                "rack_lateral_centering_step_start",
                step_no=self.step_no,
                rod_index=self.current_rod_index,
                label=label,
                target_mm=target_mm,
                pass_index=pass_index,
                before_lateral_center_m=before_lateral,
                vy_mps=vy_mps,
                duration_s=self.config.rack_lateral_active_step_s,
            )
            self._send_rack_lateral_velocity_step(
                rack,
                label=step_label,
                vy_mps=vy_mps,
                duration_s=self.config.rack_lateral_active_step_s,
            )
            time.sleep(self.config.rack_lateral_active_settle_s)
            updated_pose = self.read_rack_pose_summary_for_centering(
                rack,
                label=f"{step_label}:after_step",
                target_mm=target_mm,
            )
            if updated_pose is None:
                raise RuntimeError(
                    "rack lateral centering pose unavailable after crab step: "
                    f"label={label}, pass={pass_index}"
                )

            after_lateral = float(updated_pose["lateral_center_m"])
            after_abs = abs(after_lateral)
            improvement_m = before_abs - after_abs
            self.log(
                "rack_lateral_centering_step_result "
                f"label={label} pass={pass_index} "
                f"before_lateral_center_m={before_lateral:.4f} "
                f"after_lateral_center_m={after_lateral:.4f} "
                f"improvement_m={improvement_m:.4f} "
                f"target_lateral_m={self.config.rack_lateral_active_target_m:.4f}"
            )
            self.emit_event(
                "rack_lateral_centering_step_result",
                step_no=self.step_no,
                rod_index=self.current_rod_index,
                label=label,
                target_mm=target_mm,
                pass_index=pass_index,
                before_lateral_center_m=before_lateral,
                after_lateral_center_m=after_lateral,
                improvement_m=improvement_m,
                target_lateral_m=self.config.rack_lateral_active_target_m,
                min_improvement_m=self.config.rack_lateral_active_min_improvement_m,
            )
            pose = updated_pose
            if after_abs <= self.config.rack_lateral_active_target_m:
                self.emit_event(
                    "rack_lateral_centering_result",
                    step_no=self.step_no,
                    rod_index=self.current_rod_index,
                    label=label,
                    target_mm=target_mm,
                    status="centered_after_step",
                    pass_index=pass_index,
                    final_lateral_center_m=after_lateral,
                    target_lateral_m=self.config.rack_lateral_active_target_m,
                )
                self.log(
                    "rack_lateral_centering_result "
                    f"label={label} status=centered_after_step "
                    f"passes={pass_index} final_lateral_center_m={after_lateral:.4f}"
                )
                return pose
            if improvement_m < self.config.rack_lateral_active_min_improvement_m:
                rollback_pose = None
                rollback_lateral = None
                rollback_improvement_m = None
                rollback_duration_s = (
                    self.config.rack_lateral_active_step_s
                    * self.config.rack_lateral_active_rollback_step_scale
                )
                if (
                    improvement_m < 0.0
                    and self.config.rack_lateral_active_rollback_on_worse
                    and rollback_duration_s > 0.0
                ):
                    rollback_label = f"{label}:active_pass_{pass_index}:rollback"
                    rollback_vy_mps = -vy_mps
                    self.log(
                        "rack_lateral_centering_rollback_start "
                        f"label={label} pass={pass_index} "
                        f"after_lateral_center_m={after_lateral:.4f} "
                        f"rollback_vy_mps={rollback_vy_mps:.3f} "
                        f"duration_s={rollback_duration_s:.3f}"
                    )
                    self.emit_event(
                        "rack_lateral_centering_rollback_start",
                        step_no=self.step_no,
                        rod_index=self.current_rod_index,
                        label=label,
                        target_mm=target_mm,
                        pass_index=pass_index,
                        after_lateral_center_m=after_lateral,
                        rollback_vy_mps=rollback_vy_mps,
                        duration_s=rollback_duration_s,
                    )
                    self._send_rack_lateral_velocity_step(
                        rack,
                        label=rollback_label,
                        vy_mps=rollback_vy_mps,
                        duration_s=rollback_duration_s,
                    )
                    time.sleep(self.config.rack_lateral_active_settle_s)
                    rollback_pose = self.read_rack_pose_summary_for_centering(
                        rack,
                        label=f"{rollback_label}:after_rollback",
                        target_mm=target_mm,
                    )
                    if rollback_pose is not None:
                        rollback_lateral = float(rollback_pose["lateral_center_m"])
                        rollback_improvement_m = after_abs - abs(rollback_lateral)
                    self.log(
                        "rack_lateral_centering_rollback_result "
                        f"label={label} pass={pass_index} "
                        f"rollback_lateral_center_m={rollback_lateral} "
                        f"rollback_improvement_m={rollback_improvement_m}"
                    )
                    self.emit_event(
                        "rack_lateral_centering_rollback_result",
                        step_no=self.step_no,
                        rod_index=self.current_rod_index,
                        label=label,
                        target_mm=target_mm,
                        pass_index=pass_index,
                        before_lateral_center_m=before_lateral,
                        after_lateral_center_m=after_lateral,
                        rollback_lateral_center_m=rollback_lateral,
                        rollback_improvement_m=rollback_improvement_m,
                    )
                    if (
                        rollback_lateral is not None
                        and abs(rollback_lateral) <= self.config.rack_lateral_active_target_m
                    ):
                        self.emit_event(
                            "rack_lateral_centering_result",
                            step_no=self.step_no,
                            rod_index=self.current_rod_index,
                            label=label,
                            target_mm=target_mm,
                            status="centered_after_rollback",
                            pass_index=pass_index,
                            final_lateral_center_m=rollback_lateral,
                            target_lateral_m=self.config.rack_lateral_active_target_m,
                        )
                        self.log(
                            "rack_lateral_centering_result "
                            f"label={label} status=centered_after_rollback "
                            f"passes={pass_index} "
                            f"final_lateral_center_m={rollback_lateral:.4f}"
                        )
                        return rollback_pose

                self.emit_event(
                    "rack_lateral_centering_result",
                    step_no=self.step_no,
                    rod_index=self.current_rod_index,
                    label=label,
                    target_mm=target_mm,
                    status=(
                        "no_improvement_rollback_done"
                        if rollback_pose is not None
                        else "no_improvement"
                    ),
                    pass_index=pass_index,
                    final_lateral_center_m=(
                        rollback_lateral if rollback_lateral is not None else after_lateral
                    ),
                    improvement_m=improvement_m,
                    rollback_lateral_center_m=rollback_lateral,
                    rollback_improvement_m=rollback_improvement_m,
                )
                raise RuntimeError(
                    "rack lateral centering did not improve enough after crab step: "
                    f"label={label}, pass={pass_index}, before={before_lateral:.4f}, "
                    f"after={after_lateral:.4f}, improvement_m={improvement_m:.4f}, "
                    f"min_improvement_m={self.config.rack_lateral_active_min_improvement_m:.4f}, "
                    f"rollback_lateral_center_m={rollback_lateral}"
                )

        final_lateral = None if pose is None else pose.get("lateral_center_m")
        self.emit_event(
            "rack_lateral_centering_result",
            step_no=self.step_no,
            rod_index=self.current_rod_index,
            label=label,
            target_mm=target_mm,
            status="max_passes_exhausted",
            pass_index=self.config.rack_lateral_active_max_passes,
            final_lateral_center_m=final_lateral,
            target_lateral_m=self.config.rack_lateral_active_target_m,
        )
        raise RuntimeError(
            "rack lateral centering failed to reach target before approach: "
            f"label={label}, passes={self.config.rack_lateral_active_max_passes}, "
            f"final_lateral_center_m={final_lateral}, "
            f"target_lateral_m={self.config.rack_lateral_active_target_m}"
        )

    def approach_by_front_ultrasonic(
        self,
        target_mm: int,
        speed_mps: float,
        title: str,
        brake_margin_mm: int,
        min_safe_mm: int,
        target_tolerance_mm: int | None = None,
        correction_speed_mps: float | None = None,
        correction_max_passes: int = 0,
        angle_correction_max_span_mm: int | None = None,
        angle_correction_max_passes: int = 0,
        angle_correction_angular_speed_radps: float = 0.0,
        angle_correction_probe_s: float = 0.0,
        target_avg_accept_span_mm: int | None = None,
    ):
        """
        使用“粗定位 + 前方超声精定位”向前靠近，直到距离进入 target_mm。

        这段逻辑与 industrial_docking_integration_template.py 的 dock_to_rack()
        思路一致，但 final_stop_mm 改成总控业务要求的抓料/放料目标距离：
          1. coarse_position()：
             先确认目标已经进入前超声稳定接管区。当前超声已经稳定时，
             这一步会很快返回 ready_for_fine，不一定实际运动。
          2. fine_position()：
             用前方超声闭环停车。
             final_stop_mm=target_mm
             final_brake_margin_mm=brake_margin_mm

        为什么要有制动补偿：
          0.30m/s 精定位速度下，如果到业务目标距离才发停车，现场已经出现
          停稳后只有 80~130mm 的过冲，存在撞料架风险。因此业务目标仍
          是业务目标距离，但停车触发点必须提前。

        停稳后校验：
          靠近结束后会再次读取前方 0/1 超声；如果停稳距离低于
          min_safe_mm，总控直接失败停机，不继续夹取/放料或下一根。

        成功状态：
          - coarse_result.status == "ready_for_fine"；
          - stopped：正常靠近后到阈值停车；
          - already_at_threshold：开始时已经小于等于目标距离，所以没有继续前进。
        """
        self.next_step(title)
        self.log(
            f"target_front_distance_mm={target_mm} "
            f"brake_margin_mm={brake_margin_mm} "
            f"trigger_front_distance_mm={target_mm + brake_margin_mm} "
            f"min_safe_front_distance_mm={min_safe_mm} "
            f"coarse_speed_mps={self.config.coarse_speed_mps} "
            f"fine_speed_mps={speed_mps} "
            f"target_tolerance_mm={target_tolerance_mm} "
            f"correction_speed_mps={correction_speed_mps} "
            f"correction_max_passes={correction_max_passes} "
            f"angle_correction_max_span_mm={angle_correction_max_span_mm} "
            f"angle_correction_max_passes={angle_correction_max_passes} "
            f"angle_correction_angular_speed_radps={angle_correction_angular_speed_radps} "
            f"angle_correction_probe_s={angle_correction_probe_s} "
            f"target_avg_accept_span_mm={target_avg_accept_span_mm}"
        )

        if self.config.dry_run:
            self.log("dry-run: skip front ultrasonic approach")
            self.complete_current_step(
                "dry_run",
                target_mm=target_mm,
                brake_margin_mm=brake_margin_mm,
                min_safe_mm=min_safe_mm,
                target_tolerance_mm=target_tolerance_mm,
                correction_speed_mps=correction_speed_mps,
                correction_max_passes=correction_max_passes,
                angle_correction_max_span_mm=angle_correction_max_span_mm,
                angle_correction_max_passes=angle_correction_max_passes,
                target_avg_accept_span_mm=target_avg_accept_span_mm,
            )
            return

        def _run(rack):
            preflight = rack.preflight(
                allow_estop_pedal_fault=self.config.allow_estop_pedal_fault
            )
            self.log(f"preflight={preflight}")
            if preflight.status != "ok":
                raise RuntimeError(f"preflight blocked before approach: {preflight}")

            before = rack.read_snapshot()
            self.log(f"before_approach_snapshot={before}")
            skip_before_centering = False
            skip_before_centering_reason = None
            if target_tolerance_mm is not None and before.front_min_mm is not None:
                near_target_upper_mm = target_mm + target_tolerance_mm
                near_target_recovery_upper_mm = (
                    near_target_upper_mm
                    + self.config.rack_near_target_skip_centering_margin_mm
                )
                if before.front_min_mm <= near_target_upper_mm:
                    skip_before_centering = True
                    skip_before_centering_reason = "already_near_target"
                elif before.front_min_mm <= near_target_recovery_upper_mm:
                    skip_before_centering = True
                    skip_before_centering_reason = "near_target_front_recovery_window"
            if skip_before_centering:
                self.log(
                    "rack_lateral_centering_skipped_already_near_target "
                    f"label={title}:before_approach "
                    f"target_mm={target_mm} tolerance_mm={target_tolerance_mm} "
                    f"front_min_mm={before.front_min_mm} front_raw={before.front_raw} "
                    f"reason={skip_before_centering_reason} "
                    f"skip_margin_mm={self.config.rack_near_target_skip_centering_margin_mm}"
                )
                self.emit_event(
                    "rack_lateral_centering_skipped_already_near_target",
                    step_no=self.step_no,
                    rod_index=self.current_rod_index,
                    label=f"{title}:before_approach",
                    target_mm=target_mm,
                    tolerance_mm=target_tolerance_mm,
                    front_min_mm=before.front_min_mm,
                    front_raw=before.front_raw,
                    reason=skip_before_centering_reason,
                    skip_margin_mm=self.config.rack_near_target_skip_centering_margin_mm,
                )
            else:
                before_pose = self.monitor_rack_pose(
                    rack,
                    label=f"{title}:before_approach",
                    target_mm=target_mm,
                )
                self.center_rack_lateral_before_approach(
                    rack,
                    label=f"{title}:before_approach",
                    target_mm=target_mm,
                    initial_pose=before_pose,
                )

            coarse_result = rack.coarse_position(
                coarse_speed_mps=self.config.coarse_speed_mps,
                coarse_stop_m=1.6,
                switch_ultrasonic_mm=2200,
                ultrasonic_takeover_mm=2500,
                allow_estop_pedal_fault=self.config.allow_estop_pedal_fault,
            )
            self.log(f"coarse_result={coarse_result}")
            if coarse_result.status != "ready_for_fine":
                raise RuntimeError(f"coarse positioning failed before fine approach: {coarse_result}")

            after_coarse = rack.read_snapshot()
            self.log(f"after_coarse_snapshot={after_coarse}")

            result = rack.fine_position(
                final_stop_mm=target_mm,
                final_brake_margin_mm=brake_margin_mm,
                final_speed_mps=speed_mps,
                max_duration_s=40.0,
                allow_estop_pedal_fault=self.config.allow_estop_pedal_fault,
            )
            self.log(f"front_approach_result={result}")

            front_consistency_tolerance_mm = max(120, brake_margin_mm + 80)
            after = rack.read_snapshot()
            self.log(f"after_approach_snapshot={after}")

            def summarize_front_state(
                front_raw: tuple[tuple[int, int], ...]
            ) -> tuple[list[int], int, int]:
                front_values = [distance for _, distance in front_raw]
                front_span_mm = max(front_values) - min(front_values)
                front_avg_mm = int(round(sum(front_values) / len(front_values)))
                return front_values, front_span_mm, front_avg_mm

            def read_stable_front_state(
                label: str,
            ) -> tuple[int, tuple[tuple[int, int], ...], list[int], int, int]:
                front_min_mm, front_raw = self._read_stable_front_snapshot(
                    rack=rack,
                    label=label,
                    consistency_tolerance_mm=front_consistency_tolerance_mm,
                )
                front_values, front_span_mm, front_avg_mm = summarize_front_state(front_raw)
                return front_min_mm, front_raw, front_values, front_span_mm, front_avg_mm

            stable_front_min_mm, stable_front_raw = self._read_stable_front_snapshot(
                rack=rack,
                label="after_approach",
                consistency_tolerance_mm=front_consistency_tolerance_mm,
            )
            (
                final_front_values,
                final_front_span_mm,
                stable_front_avg_mm,
            ) = summarize_front_state(stable_front_raw)

            result_distances = getattr(getattr(result, "detail", None), "distances", ())
            result_values = [distance for _, distance in result_distances]
            if len(result_values) >= len(FRONT_ULTRASONIC_IDS):
                result_span_mm = max(result_values) - min(result_values)
                if result_span_mm > front_consistency_tolerance_mm:
                    raise RuntimeError(
                        "front ultrasonic approach stopped on inconsistent front sensors: "
                        f"front_raw={result_distances}, span_mm={result_span_mm}, "
                        f"tolerance_mm={front_consistency_tolerance_mm}"
                    )

            correction_speed = correction_speed_mps or min(speed_mps, 0.05)
            too_close_recovery_count = 0
            max_too_close_recoveries = 2

            def recover_front_too_close(
                *,
                label: str,
                front_min_mm: int,
                front_avg_mm: int,
                front_raw: tuple[tuple[int, int], ...],
            ) -> tuple[int, tuple[tuple[int, int], ...], list[int], int, int]:
                nonlocal too_close_recovery_count

                if too_close_recovery_count >= max_too_close_recoveries:
                    raise RuntimeError(
                        "front ultrasonic approach remains too close after safe backoff: "
                        f"label={label}, front_min_mm={front_min_mm}, "
                        f"min_safe_mm={min_safe_mm}, target_mm={target_mm}, "
                        f"front_raw={front_raw}, recoveries={too_close_recovery_count}"
                    )

                too_close_recovery_count += 1
                effective_tolerance_mm = (
                    target_tolerance_mm if target_tolerance_mm is not None else 10
                )
                safe_target_mm = max(
                    target_mm + effective_tolerance_mm + 10,
                    min_safe_mm + 35,
                )
                recovery_distance_mm = max(
                    safe_target_mm - front_avg_mm,
                    min_safe_mm + 25 - front_min_mm,
                    20,
                )
                recovery_distance_m = min(
                    max(recovery_distance_mm / 1000.0, 0.02),
                    0.08,
                )
                recovery_speed_mps = min(correction_speed, 0.025)

                self.log(
                    "front_too_close_safe_backoff_start "
                    f"label={label} pass={too_close_recovery_count} "
                    f"front_min_mm={front_min_mm} front_avg_mm={front_avg_mm} "
                    f"min_safe_mm={min_safe_mm} target_mm={target_mm} "
                    f"safe_target_mm={safe_target_mm} "
                    f"distance_m={recovery_distance_m:.3f} "
                    f"speed_mps={recovery_speed_mps} front_raw={front_raw}"
                )
                self.emit_event(
                    "front_too_close_safe_backoff_start",
                    step_no=self.step_no,
                    rod_index=self.current_rod_index,
                    label=label,
                    pass_index=too_close_recovery_count,
                    front_min_mm=front_min_mm,
                    front_avg_mm=front_avg_mm,
                    min_safe_mm=min_safe_mm,
                    target_mm=target_mm,
                    safe_target_mm=safe_target_mm,
                    distance_m=recovery_distance_m,
                    speed_mps=recovery_speed_mps,
                    front_raw=front_raw,
                )

                recovery_result = None
                for retry_index in range(self.config.front_too_close_safe_backoff_retries + 1):
                    recovery_result = rack.retreat(
                        distance_m=recovery_distance_m,
                        speed_mps=recovery_speed_mps,
                        rear_stop_mm=700,
                        rear_hard_stop_mm=500,
                        method="velocity",
                        max_duration_s=max(
                            2.0,
                            recovery_distance_m / recovery_speed_mps + 1.0,
                        ),
                        allow_motion_control_error_retreat_escape=True,
                        allow_estop_pedal_fault=self.config.allow_estop_pedal_fault,
                    )
                    self.log(
                        "front_too_close_safe_backoff_result "
                        f"label={label} pass={too_close_recovery_count} "
                        f"retry={retry_index} result={recovery_result}"
                    )
                    if recovery_result.status == "completed":
                        break
                    detail = getattr(recovery_result, "detail", None) or {}
                    retryable_rear_precheck = (
                        recovery_result.status == "rear_obstacle"
                        and bool(detail.get("before_chassis_control"))
                    )
                    if (
                        not retryable_rear_precheck
                        or retry_index >= self.config.front_too_close_safe_backoff_retries
                    ):
                        raise RuntimeError(
                            "front ultrasonic too-close safe backoff failed: "
                            f"label={label}, result={recovery_result}"
                        )
                    self.emit_event(
                        "front_too_close_safe_backoff_retry",
                        step_no=self.step_no,
                        rod_index=self.current_rod_index,
                        label=label,
                        pass_index=too_close_recovery_count,
                        retry_index=retry_index + 1,
                        result=repr(recovery_result),
                    )
                    time.sleep(0.25)

                refreshed = read_stable_front_state(
                    f"{label}_after_safe_backoff_{too_close_recovery_count}"
                )
                refreshed_min_mm, refreshed_raw, _, refreshed_span_mm, refreshed_avg_mm = refreshed
                self.log(
                    "front_too_close_safe_backoff_confirmed "
                    f"label={label} pass={too_close_recovery_count} "
                    f"front_min_mm={refreshed_min_mm} "
                    f"front_avg_mm={refreshed_avg_mm} "
                    f"front_span_mm={refreshed_span_mm} "
                    f"front_raw={refreshed_raw}"
                )
                self.emit_event(
                    "front_too_close_safe_backoff_confirmed",
                    step_no=self.step_no,
                    rod_index=self.current_rod_index,
                    label=label,
                    pass_index=too_close_recovery_count,
                    front_min_mm=refreshed_min_mm,
                    front_avg_mm=refreshed_avg_mm,
                    front_span_mm=refreshed_span_mm,
                    front_raw=refreshed_raw,
                )
                if refreshed_min_mm < min_safe_mm:
                    raise RuntimeError(
                        "front ultrasonic approach still too close after safe backoff: "
                        f"label={label}, front_min_mm={refreshed_min_mm}, "
                        f"min_safe_mm={min_safe_mm}, target_mm={target_mm}, "
                        f"front_raw={refreshed_raw}"
                    )
                return refreshed

            if stable_front_min_mm < min_safe_mm:
                (
                    stable_front_min_mm,
                    stable_front_raw,
                    final_front_values,
                    final_front_span_mm,
                    stable_front_avg_mm,
                ) = recover_front_too_close(
                    label="after_approach",
                    front_min_mm=stable_front_min_mm,
                    front_avg_mm=stable_front_avg_mm,
                    front_raw=stable_front_raw,
                )
            if target_tolerance_mm is not None:
                if target_tolerance_mm < 0:
                    raise ValueError("target_tolerance_mm must be >= 0")
                correction_pass = 0
                angle_correction_pass = 0
                target_lower_mm = target_mm - target_tolerance_mm
                target_upper_mm = target_mm + target_tolerance_mm
                angled_span_threshold_mm = max(20, target_tolerance_mm)
                while True:
                    too_close_values = [
                        distance
                        for _, distance in stable_front_raw
                        if distance < target_lower_mm
                    ]
                    too_far_values = [
                        distance
                        for _, distance in stable_front_raw
                        if distance > target_upper_mm
                    ]
                    if (
                        target_avg_accept_span_mm is not None
                        and target_avg_accept_span_mm > 0
                        and target_lower_mm <= stable_front_avg_mm <= target_upper_mm
                        and stable_front_min_mm >= min_safe_mm
                        and final_front_span_mm <= target_avg_accept_span_mm
                    ):
                        self.log(
                            "front_target_window_avg_confirmed "
                            f"target_mm={target_mm} tolerance_mm={target_tolerance_mm} "
                            f"front_avg_mm={stable_front_avg_mm} "
                            f"front_min_mm={stable_front_min_mm} "
                            f"front_span_mm={final_front_span_mm} "
                            f"accept_span_mm={target_avg_accept_span_mm} "
                            f"front_raw={stable_front_raw}"
                        )
                        break
                    if not too_close_values and not too_far_values:
                        break
                    if (
                        angle_correction_max_span_mm is not None
                        and angle_correction_max_span_mm > 0
                        and final_front_span_mm > angle_correction_max_span_mm
                        and angle_correction_pass < angle_correction_max_passes
                    ):
                        angle_correction_pass += 1
                        (
                            stable_front_min_mm,
                            stable_front_raw,
                            stable_front_avg_mm,
                            final_front_span_mm,
                        ) = self._correct_front_angle_by_probe(
                            rack=rack,
                            stable_front_raw=stable_front_raw,
                            consistency_tolerance_mm=front_consistency_tolerance_mm,
                            pass_index=angle_correction_pass,
                            max_span_mm=angle_correction_max_span_mm,
                            angular_speed_radps=angle_correction_angular_speed_radps,
                            probe_s=angle_correction_probe_s,
                        )
                        final_front_values = [distance for _, distance in stable_front_raw]
                        if stable_front_min_mm < min_safe_mm:
                            (
                                stable_front_min_mm,
                                stable_front_raw,
                                final_front_values,
                                final_front_span_mm,
                                stable_front_avg_mm,
                            ) = recover_front_too_close(
                                label=f"after_angle_correction_{angle_correction_pass}",
                                front_min_mm=stable_front_min_mm,
                                front_avg_mm=stable_front_avg_mm,
                                front_raw=stable_front_raw,
                            )
                        continue
                    if (
                        angle_correction_max_span_mm is not None
                        and angle_correction_max_span_mm > 0
                        and final_front_span_mm > angle_correction_max_span_mm
                    ):
                        raise RuntimeError(
                            "front ultrasonic approach remains angled after correction: "
                            f"front_span_mm={final_front_span_mm}, "
                            f"max_span_mm={angle_correction_max_span_mm}, "
                            f"front_raw={stable_front_raw}, target_mm={target_mm}, "
                            f"tolerance_mm={target_tolerance_mm}"
                        )
                    if too_close_values and too_far_values:
                        raise RuntimeError(
                            "front ultrasonic approach is angled for target window: "
                            f"front_raw={stable_front_raw}, target_mm={target_mm}, "
                            f"tolerance_mm={target_tolerance_mm}, "
                            f"target_window=({target_lower_mm}, {target_upper_mm}), "
                            f"span_mm={final_front_span_mm}"
                        )
                    if correction_pass >= correction_max_passes:
                        direction = "too_far" if too_far_values else "too_close"
                        raise RuntimeError(
                            "front ultrasonic approach exceeded correction passes: "
                            f"direction={direction}, "
                            f"front_avg_mm={stable_front_avg_mm}, target_mm={target_mm}, "
                            f"tolerance_mm={target_tolerance_mm}, front_raw={stable_front_raw}"
                        )

                    correction_pass += 1

                    if too_far_values:
                        if (
                            stable_front_min_mm <= target_upper_mm
                            and final_front_span_mm > angled_span_threshold_mm
                        ):
                            raise RuntimeError(
                                "front ultrasonic approach is angled or inconsistent for target window: "
                                f"front_min_mm={stable_front_min_mm}, front_avg_mm={stable_front_avg_mm}, "
                                f"target_mm={target_mm}, tolerance_mm={target_tolerance_mm}, "
                                f"front_raw={stable_front_raw}, span_mm={final_front_span_mm}, "
                                f"angled_span_threshold_mm={angled_span_threshold_mm}"
                            )
                        correction_stop_mm = target_mm
                        self.log(
                            "front_approach_correction_start "
                            f"pass={correction_pass} direction=forward "
                            f"stop_mm={correction_stop_mm} speed_mps={correction_speed} "
                            f"front_avg_mm={stable_front_avg_mm} front_raw={stable_front_raw}"
                        )
                        correction_distance_m = max(
                            0.0,
                            (stable_front_avg_mm - target_mm) / 1000.0 + 0.04,
                        )
                        correction_max_duration_s = min(
                            35.0,
                            max(15.0, correction_distance_m / correction_speed + 4.0),
                        )
                        self.log(
                            "front_approach_correction_duration "
                            f"pass={correction_pass} direction=forward "
                            f"estimated_distance_m={correction_distance_m:.3f} "
                            f"max_duration_s={correction_max_duration_s:.2f}"
                        )
                        correction_result = rack.fine_position(
                            final_stop_mm=correction_stop_mm,
                            final_brake_margin_mm=0,
                            final_speed_mps=correction_speed,
                            max_duration_s=correction_max_duration_s,
                            allow_estop_pedal_fault=self.config.allow_estop_pedal_fault,
                        )
                        ok_statuses = ("stopped", "already_at_threshold")
                    else:
                        if (
                            max(final_front_values) >= target_lower_mm
                            and final_front_span_mm > angled_span_threshold_mm
                        ):
                            raise RuntimeError(
                                "front ultrasonic approach is angled or inconsistent for target window: "
                                f"front_min_mm={stable_front_min_mm}, front_avg_mm={stable_front_avg_mm}, "
                                f"target_mm={target_mm}, tolerance_mm={target_tolerance_mm}, "
                                f"front_raw={stable_front_raw}, span_mm={final_front_span_mm}, "
                                f"angled_span_threshold_mm={angled_span_threshold_mm}"
                            )
                        correction_distance_mm = max(
                            target_mm - stable_front_avg_mm,
                            target_lower_mm - stable_front_min_mm,
                            10,
                        )
                        correction_distance_m = min(
                            max(correction_distance_mm / 1000.0, 0.01),
                            0.12,
                        )
                        self.log(
                            "front_approach_correction_start "
                            f"pass={correction_pass} direction=backward "
                            f"distance_m={correction_distance_m:.3f} "
                            f"speed_mps={correction_speed} "
                            f"front_avg_mm={stable_front_avg_mm} front_raw={stable_front_raw}"
                        )
                        correction_result = rack.retreat(
                            distance_m=correction_distance_m,
                            speed_mps=correction_speed,
                            rear_stop_mm=700,
                            rear_hard_stop_mm=500,
                            method="velocity",
                            max_duration_s=max(2.0, correction_distance_m / correction_speed + 1.0),
                            allow_motion_control_error_retreat_escape=True,
                            allow_estop_pedal_fault=self.config.allow_estop_pedal_fault,
                        )
                        ok_statuses = ("completed",)
                    self.log(f"front_approach_correction_result={correction_result}")
                    if correction_result.status not in ok_statuses:
                        raise RuntimeError(
                            f"front ultrasonic correction failed: {correction_result}"
                        )
                    (
                        stable_front_min_mm,
                        stable_front_raw,
                        final_front_values,
                        final_front_span_mm,
                        stable_front_avg_mm,
                    ) = read_stable_front_state(
                        f"after_approach_correction_{correction_pass}"
                    )
                    if stable_front_min_mm < min_safe_mm:
                        (
                            stable_front_min_mm,
                            stable_front_raw,
                            final_front_values,
                            final_front_span_mm,
                            stable_front_avg_mm,
                        ) = recover_front_too_close(
                            label=f"after_approach_correction_{correction_pass}",
                            front_min_mm=stable_front_min_mm,
                            front_avg_mm=stable_front_avg_mm,
                            front_raw=stable_front_raw,
                        )

                self.log(
                    "front_target_window_confirmed "
                    f"target_mm={target_mm} tolerance_mm={target_tolerance_mm} "
                    f"front_avg_mm={stable_front_avg_mm} front_min_mm={stable_front_min_mm} "
                    f"front_raw={stable_front_raw}"
                )
                self.emit_event(
                    "front_target_window_confirmed",
                    step_no=self.step_no,
                    rod_index=self.current_rod_index,
                    target_mm=target_mm,
                    tolerance_mm=target_tolerance_mm,
                    front_avg_mm=stable_front_avg_mm,
                    front_min_mm=stable_front_min_mm,
                    front_raw=stable_front_raw,
                    front_span_mm=final_front_span_mm,
                )

            max_expected_front_mm = target_mm + max(120, brake_margin_mm + 30)
            if stable_front_min_mm > max_expected_front_mm:
                raise RuntimeError(
                    "front ultrasonic approach did not reach target window: "
                    f"front_min_mm={stable_front_min_mm}, max_expected_front_mm={max_expected_front_mm}, "
                    f"target_mm={target_mm}, brake_margin_mm={brake_margin_mm}, "
                    f"front_raw={stable_front_raw}"
                )

            if result.status not in ("stopped", "already_at_threshold"):
                raise RuntimeError(f"front ultrasonic approach failed: {result}")

            after_pose = self.monitor_rack_pose(
                rack,
                label=f"{title}:after_approach",
                target_mm=target_mm,
            )
            self.guarded_rack_post_approach_check(
                rack,
                label=f"{title}:after_approach",
                target_mm=target_mm,
                min_safe_mm=min_safe_mm,
                pose=after_pose,
            )

            return result

        result = self.with_industrial_rack(_run)
        self.complete_current_step(
            "completed",
            target_mm=target_mm,
            brake_margin_mm=brake_margin_mm,
            min_safe_mm=min_safe_mm,
            target_tolerance_mm=target_tolerance_mm,
            result=repr(result),
        )
        time.sleep(self.config.settle_s)

    def _correct_front_angle_by_probe(
        self,
        *,
        rack,
        stable_front_raw: tuple[tuple[int, int], ...],
        consistency_tolerance_mm: int,
        pass_index: int,
        max_span_mm: int,
        angular_speed_radps: float,
        probe_s: float,
    ) -> tuple[int, tuple[tuple[int, int], ...], int, int]:
        """
        Small yaw probe to reduce front 0/1 ultrasonic span.

        The physical left/right mapping of sensor IDs and yaw sign is easy to
        confuse in the field, so this does not hard-code a direction. It tries
        a tiny positive yaw pulse, keeps it if the span improves, otherwise
        probes the opposite side and restores if neither side helps.
        """
        if angular_speed_radps <= 0.0:
            raise ValueError("angle correction angular_speed_radps must be positive")
        if probe_s <= 0.0:
            raise ValueError("angle correction probe_s must be positive")

        def summarize(raw: tuple[tuple[int, int], ...]) -> tuple[int, int, int]:
            values = [distance for _, distance in raw]
            min_mm = min(values)
            avg_mm = int(round(sum(values) / len(values)))
            span_mm = max(values) - min_mm
            return min_mm, avg_mm, span_mm

        def read_state(label: str) -> tuple[int, tuple[tuple[int, int], ...], int, int]:
            min_mm, raw = self._read_stable_front_snapshot(
                rack=rack,
                label=label,
                consistency_tolerance_mm=consistency_tolerance_mm,
            )
            _, avg_mm, span_mm = summarize(raw)
            return min_mm, raw, avg_mm, span_mm

        baseline_min_mm, baseline_avg_mm, baseline_span_mm = summarize(stable_front_raw)
        self.log(
            "front_angle_correction_start "
            f"pass={pass_index} baseline_span_mm={baseline_span_mm} "
            f"max_span_mm={max_span_mm} baseline_avg_mm={baseline_avg_mm} "
            f"baseline_min_mm={baseline_min_mm} baseline_raw={stable_front_raw} "
            f"angular_speed_radps={angular_speed_radps} probe_s={probe_s}"
        )

        try:
            rack.front.request_chassis_control_ready()
        except Exception as exc:
            self.log(f"front_angle_correction_request_ready_failed={exc}")
            rack.front.pnc.request_chassis_control(0)
        time.sleep(0.15)

        def pulse(label: str, wz_radps: float, duration_s: float):
            self.log(
                "front_angle_correction_probe "
                f"pass={pass_index} label={label} wz_radps={wz_radps:.4f} "
                f"duration_s={duration_s:.3f}"
            )
            self._send_turn_velocity_existing_pnc(
                rack.front.pnc,
                wz_radps=wz_radps,
                duration_s=duration_s,
                hz=20.0,
            )
            time.sleep(max(0.25, self.config.settle_s))
            state = read_state(f"front_angle_correction_{pass_index}_{label}")
            self.log(
                "front_angle_correction_probe_result "
                f"pass={pass_index} label={label} span_mm={state[3]} "
                f"avg_mm={state[2]} raw={state[1]}"
            )
            return state

        positive_state = pulse("positive", angular_speed_radps, probe_s)
        if positive_state[3] < baseline_span_mm:
            self.log(
                "front_angle_correction_selected "
                f"pass={pass_index} direction=positive "
                f"span_before_mm={baseline_span_mm} span_after_mm={positive_state[3]}"
            )
            return positive_state

        negative_state = pulse("negative", -angular_speed_radps, probe_s * 2.0)
        if negative_state[3] < baseline_span_mm:
            self.log(
                "front_angle_correction_selected "
                f"pass={pass_index} direction=negative "
                f"span_before_mm={baseline_span_mm} span_after_mm={negative_state[3]}"
            )
            return negative_state

        restored_state = pulse("restore", angular_speed_radps, probe_s)
        self.log(
            "front_angle_correction_no_improvement "
            f"pass={pass_index} baseline_span_mm={baseline_span_mm} "
            f"positive_span_mm={positive_state[3]} negative_span_mm={negative_state[3]} "
            f"restored_span_mm={restored_state[3]} restored_raw={restored_state[1]}"
        )
        return restored_state

    def _read_stable_front_snapshot(
        self,
        rack,
        label: str,
        consistency_tolerance_mm: int,
        required_samples: int = 5,
        max_samples: int = 18,
        interval_s: float = 0.08,
    ) -> tuple[int, tuple[tuple[int, int], ...]]:
        """
        运动停止后多帧确认前方 0/1 超声。

        现场已经复现过单帧跳变：刚停稳时某个前雷达会突然跳到 1m+
        或短暂丢失，但 0.5~1s 后又恢复正常。工业流程不能用单帧数据决定
        是否下放/夹取，也不能忽略真正持续不一致的传感器状态。

        判据：
          - 必须连续收集到 required_samples 帧；
          - 每一帧都必须同时包含前方 0/1；
          - 每一帧 0/1 跨度不得超过 consistency_tolerance_mm；
          - 最终使用每个雷达 ID 的中位数作为停稳证据。
        """
        if required_samples <= 0:
            raise ValueError("required_samples must be positive")
        if max_samples < required_samples:
            raise ValueError("max_samples must be >= required_samples")

        accepted: list[tuple[tuple[int, int], ...]] = []
        rejected: list[str] = []

        for _ in range(max_samples):
            snapshot = rack.read_snapshot()
            raw_by_id = {radar_id: distance for radar_id, distance in snapshot.front_raw}
            if all(radar_id in raw_by_id for radar_id in FRONT_ULTRASONIC_IDS):
                raw = tuple((radar_id, int(raw_by_id[radar_id])) for radar_id in FRONT_ULTRASONIC_IDS)
                values = [distance for _, distance in raw]
                span_mm = max(values) - min(values)
                if span_mm <= consistency_tolerance_mm:
                    accepted.append(raw)
                    if len(accepted) >= required_samples:
                        break
                else:
                    rejected.append(f"{raw}/span={span_mm}")
            else:
                rejected.append(f"{snapshot.front_raw}/incomplete")
            time.sleep(interval_s)

        if len(accepted) < required_samples:
            raise RuntimeError(
                "front ultrasonic approach has no stable post-stop confirmation: "
                f"label={label}, accepted={len(accepted)}, required={required_samples}, "
                f"tolerance_mm={consistency_tolerance_mm}, recent_rejected={tuple(rejected[-8:])}"
            )

        medians: list[tuple[int, int]] = []
        for radar_id in FRONT_ULTRASONIC_IDS:
            values = sorted(dict(sample)[radar_id] for sample in accepted)
            medians.append((radar_id, int(values[len(values) // 2])))

        stable_raw = tuple(medians)
        stable_min_mm = min(distance for _, distance in stable_raw)
        stable_span_mm = max(distance for _, distance in stable_raw) - stable_min_mm
        self.log(
            "stable_front_snapshot "
            f"label={label} stable_min_mm={stable_min_mm} "
            f"stable_raw={stable_raw} stable_span_mm={stable_span_mm} "
            f"accepted_samples={len(accepted)} rejected_samples={len(rejected)}"
        )
        return stable_min_mm, stable_raw

    def _read_stable_front_state(
        self,
        rack,
        label: str,
        consistency_tolerance_mm: int,
    ) -> dict:
        stable_front_min_mm, stable_front_raw = self._read_stable_front_snapshot(
            rack=rack,
            label=label,
            consistency_tolerance_mm=consistency_tolerance_mm,
        )
        values = [distance for _, distance in stable_front_raw]
        stable_front_avg_mm = int(round(sum(values) / len(values)))
        stable_front_span_mm = max(values) - min(values)
        state = {
            "front_min_mm": stable_front_min_mm,
            "front_avg_mm": stable_front_avg_mm,
            "front_span_mm": stable_front_span_mm,
            "front_raw": stable_front_raw,
        }
        self.log(f"stable_front_state label={label} {state}")
        return state

    def retreat_by_industrial_rack(self, title: str):
        """使用 RackIndustrialDockingController 后退指定距离。"""
        self.next_step(title)
        self.log(
            f"retreat_target_distance_m={self.config.retreat_distance_m} "
            f"retreat_target_tolerance_mm={self.config.retreat_target_tolerance_mm} "
            f"retreat_method={self.config.retreat_method} "
            f"speed_mps={self.config.retreat_speed_mps} "
            f"open_loop_brake_compensation_m={self.config.retreat_open_loop_brake_compensation_m}"
        )

        if self.config.dry_run:
            self.log("dry-run: skip retreat")
            self.complete_current_step(
                "dry_run",
                retreat_method=self.config.retreat_method,
                retreat_distance_m=self.config.retreat_distance_m,
                retreat_target_tolerance_mm=self.config.retreat_target_tolerance_mm,
            )
            return

        if self.config.retreat_method == "hybrid":
            self._retreat_by_hybrid_relative()
            self.complete_current_step(
                "completed",
                retreat_method=self.config.retreat_method,
                retreat_distance_m=self.config.retreat_distance_m,
                retreat_target_tolerance_mm=self.config.retreat_target_tolerance_mm,
            )
            time.sleep(self.config.settle_s)
            return

        if self.config.retreat_method == "front-ultrasonic":
            occlusion_escape_m = self._maybe_grab_retreat_front_occlusion_escape()
            remaining_distance_m = max(0.05, self.config.retreat_distance_m - occlusion_escape_m)
            delta_by_id = self._retreat_by_front_ultrasonic_delta(
                distance_m=remaining_distance_m
            )
            self.complete_current_step(
                "completed",
                retreat_method=self.config.retreat_method,
                retreat_distance_m=self.config.retreat_distance_m,
                retreat_target_tolerance_mm=self.config.retreat_target_tolerance_mm,
                grab_retreat_front_occlusion_escape_m=round(occlusion_escape_m, 4),
                front_ultrasonic_remaining_distance_m=round(remaining_distance_m, 4),
                delta_by_id=delta_by_id,
            )
            time.sleep(self.config.settle_s)
            return

        def _run(rack):
            preflight = rack.preflight(
                allow_estop_pedal_fault=self.config.allow_estop_pedal_fault
            )
            self.log(f"preflight={preflight}")
            if preflight.status != "ok":
                if self._is_front_collision_retreat_escape(preflight):
                    self.log(
                        "preflight has motion_control_error=2; "
                        "continue with relative retreat escape under rear ultrasonic guard"
                    )
                else:
                    raise RuntimeError(f"preflight blocked before retreat: {preflight}")

            before = rack.read_snapshot()
            self.log(f"before_retreat_snapshot={before}")

            result = rack.retreat(
                distance_m=self._retreat_command_distance_m(),
                speed_mps=self.config.retreat_speed_mps,
                rear_stop_mm=700,
                rear_hard_stop_mm=500,
                method=self.config.retreat_method,
                allow_motion_control_error_retreat_escape=True,
                allow_estop_pedal_fault=self.config.allow_estop_pedal_fault,
            )
            self.log(f"retreat_result={result}")

            after = rack.read_snapshot()
            self.log(f"after_retreat_snapshot={after}")

            if result.status != "completed":
                raise RuntimeError(f"retreat failed: {result}")

            return result

        result = self.with_industrial_rack(_run)
        self.complete_current_step(
            "completed",
            retreat_method=self.config.retreat_method,
            retreat_distance_m=self.config.retreat_distance_m,
            retreat_target_tolerance_mm=self.config.retreat_target_tolerance_mm,
            result=repr(result),
        )
        time.sleep(self.config.settle_s)

    def _place_retreat_recovery_min_safe_mm(self, target_mm: int) -> int:
        return max(
            self.config.place_min_safe_mm,
            target_mm - max(240, self.config.place_retreat_target_tolerance_mm + 180),
        )

    def correct_place_retreat_to_front_target(self, rod_index: int):
        """
        Restore the post-place retreat target before the left turn.

        The next grab alignment depends on this translation. A failed retreat
        must not be resumed by blindly repeating the whole 1m retreat: the
        controller first restores the absolute front-ultrasonic target, then
        allows the left turn.
        """
        target_mm = self.default_place_retreat_front_target_mm()
        tolerance_mm = self.config.place_retreat_target_tolerance_mm
        min_safe_mm = self._place_retreat_recovery_min_safe_mm(target_mm)
        front_consistency_tolerance_mm = max(
            120,
            self.config.place_retreat_forward_brake_margin_mm + 80,
            tolerance_mm + 80,
        )

        self.next_step(f"第{rod_index}根：放料后退目标纠偏到前超声 {target_mm}mm")
        self.log(
            "place_retreat_target_recovery_start "
            f"target_mm={target_mm} tolerance_mm={tolerance_mm} "
            f"min_safe_front_mm={min_safe_mm} "
            f"front_consistency_tolerance_mm={front_consistency_tolerance_mm} "
            f"forward_speed_mps={self.config.place_retreat_forward_speed_mps} "
            f"forward_brake_margin_mm={self.config.place_retreat_forward_brake_margin_mm} "
            f"forward_correction_speed_mps={self.config.place_retreat_forward_correction_speed_mps} "
            f"correction_max_passes={self.config.place_retreat_correction_max_passes}"
        )

        if self.config.dry_run:
            self.log("dry-run: skip place retreat target recovery")
            self.complete_current_step(
                "dry_run",
                target_mm=target_mm,
                tolerance_mm=tolerance_mm,
                min_safe_mm=min_safe_mm,
            )
            return

        def read_current_front_state(label: str) -> dict:
            def _run(rack):
                preflight = rack.preflight(
                    allow_estop_pedal_fault=self.config.allow_estop_pedal_fault
                )
                self.log(f"{label}_preflight={preflight}")
                if preflight.status != "ok":
                    raise RuntimeError(f"preflight blocked before place retreat recovery: {preflight}")
                snapshot = rack.read_snapshot()
                self.log(f"{label}_snapshot={snapshot}")
                return self._read_stable_front_state(
                    rack=rack,
                    label=label,
                    consistency_tolerance_mm=front_consistency_tolerance_mm,
                )

            return self.with_industrial_rack(_run)

        state = read_current_front_state("place_retreat_recovery_before")
        front_avg_mm = state["front_avg_mm"]

        if front_avg_mm < target_mm - tolerance_mm:
            remaining_mm = target_mm - front_avg_mm
            remaining_m = remaining_mm / 1000.0
            max_duration_s = max(
                6.0,
                remaining_m / max(min(self.config.retreat_speed_mps, 0.20), 0.01) + 6.0,
            )
            self.log(
                "place_retreat_target_recovery_direction=backward "
                f"front_avg_mm={front_avg_mm} target_mm={target_mm} "
                f"remaining_mm={remaining_mm} remaining_m={remaining_m:.3f} "
                f"max_duration_s={max_duration_s:.2f}"
            )
            self._retreat_by_front_ultrasonic_delta(
                distance_m=remaining_m,
                tolerance_mm=tolerance_mm,
                max_duration_s=max_duration_s,
            )
            state = read_current_front_state("place_retreat_recovery_after_backward")
            front_avg_mm = state["front_avg_mm"]

        if front_avg_mm > target_mm + tolerance_mm:
            state = self._forward_correct_place_retreat_front_target(
                target_mm=target_mm,
                tolerance_mm=tolerance_mm,
                min_safe_mm=min_safe_mm,
                front_consistency_tolerance_mm=front_consistency_tolerance_mm,
                initial_state=state,
            )
            front_avg_mm = state["front_avg_mm"]

        if front_avg_mm < target_mm - tolerance_mm:
            raise RuntimeError(
                "place retreat target recovery still under target after correction: "
                f"front_avg_mm={front_avg_mm}, target_mm={target_mm}, "
                f"tolerance_mm={tolerance_mm}, front_raw={state['front_raw']}"
            )
        if front_avg_mm > target_mm + tolerance_mm:
            raise RuntimeError(
                "place retreat target recovery still over target after correction: "
                f"front_avg_mm={front_avg_mm}, target_mm={target_mm}, "
                f"tolerance_mm={tolerance_mm}, front_raw={state['front_raw']}"
            )
        if state["front_min_mm"] < min_safe_mm:
            raise RuntimeError(
                "place retreat target recovery stopped too close to front obstacle: "
                f"front_min_mm={state['front_min_mm']}, min_safe_mm={min_safe_mm}, "
                f"target_mm={target_mm}, front_raw={state['front_raw']}"
            )

        self.log(
            "place_retreat_target_window_confirmed "
            f"target_mm={target_mm} tolerance_mm={tolerance_mm} "
            f"front_avg_mm={front_avg_mm} front_min_mm={state['front_min_mm']} "
            f"front_raw={state['front_raw']}"
        )
        self.emit_event(
            "place_retreat_target_window_confirmed",
            step_no=self.step_no,
            rod_index=self.current_rod_index,
            target_mm=target_mm,
            tolerance_mm=tolerance_mm,
            front_avg_mm=front_avg_mm,
            front_min_mm=state["front_min_mm"],
            front_raw=state["front_raw"],
        )
        self.complete_current_step(
            "completed",
            target_mm=target_mm,
            tolerance_mm=tolerance_mm,
            min_safe_mm=min_safe_mm,
            final_front_state=state,
        )
        time.sleep(self.config.settle_s)

    def _forward_correct_place_retreat_front_target(
        self,
        *,
        target_mm: int,
        tolerance_mm: int,
        min_safe_mm: int,
        front_consistency_tolerance_mm: int,
        initial_state: dict,
    ) -> dict:
        def _run(rack):
            preflight = rack.preflight(
                allow_estop_pedal_fault=self.config.allow_estop_pedal_fault
            )
            self.log(f"place_retreat_forward_recovery_preflight={preflight}")
            if preflight.status != "ok":
                raise RuntimeError(f"preflight blocked before forward recovery: {preflight}")

            state = initial_state
            max_runs = 1 + self.config.place_retreat_correction_max_passes
            run_no = 0
            while state["front_avg_mm"] > target_mm + tolerance_mm:
                if state["front_min_mm"] < min_safe_mm:
                    raise RuntimeError(
                        "place retreat forward recovery is too close before motion: "
                        f"front_min_mm={state['front_min_mm']}, min_safe_mm={min_safe_mm}, "
                        f"target_mm={target_mm}, front_raw={state['front_raw']}"
                    )
                if run_no >= max_runs:
                    raise RuntimeError(
                        "place retreat forward recovery exceeded correction passes: "
                        f"front_avg_mm={state['front_avg_mm']}, target_mm={target_mm}, "
                        f"tolerance_mm={tolerance_mm}, front_raw={state['front_raw']}"
                    )

                use_primary = run_no == 0
                brake_margin_mm = (
                    self.config.place_retreat_forward_brake_margin_mm
                    if use_primary
                    else 0
                )
                speed_mps = (
                    self.config.place_retreat_forward_speed_mps
                    if use_primary
                    else self.config.place_retreat_forward_correction_speed_mps
                )
                run_no += 1
                self.log(
                    "place_retreat_forward_recovery_run "
                    f"run_no={run_no} target_mm={target_mm} "
                    f"brake_margin_mm={brake_margin_mm} speed_mps={speed_mps} "
                    f"front_state={state}"
                )
                result = rack.fine_position(
                    final_stop_mm=target_mm,
                    final_brake_margin_mm=brake_margin_mm,
                    final_speed_mps=speed_mps,
                    max_duration_s=25.0,
                    allow_estop_pedal_fault=self.config.allow_estop_pedal_fault,
                )
                self.log(f"place_retreat_forward_recovery_result={result}")
                if result.status not in ("stopped", "already_at_threshold"):
                    raise RuntimeError(f"place retreat forward recovery failed: {result}")

                state = self._read_stable_front_state(
                    rack=rack,
                    label=f"place_retreat_forward_recovery_after_{run_no}",
                    consistency_tolerance_mm=front_consistency_tolerance_mm,
                )
                if state["front_avg_mm"] < target_mm - tolerance_mm:
                    raise RuntimeError(
                        "place retreat forward recovery overshot target: "
                        f"front_avg_mm={state['front_avg_mm']}, target_mm={target_mm}, "
                        f"tolerance_mm={tolerance_mm}, front_raw={state['front_raw']}"
                    )

            return state

        return self.with_industrial_rack(_run)

    def _retreat_relative_once(self, distance_m: float, label: str):
        """提交一次底盘 relative_move 后退任务，成功才返回。"""
        def _run(rack):
            result = rack.retreat(
                distance_m=distance_m,
                speed_mps=self.config.retreat_speed_mps,
                rear_stop_mm=700,
                rear_hard_stop_mm=500,
                method="relative",
                allow_motion_control_error_retreat_escape=True,
                allow_estop_pedal_fault=self.config.allow_estop_pedal_fault,
            )
            self.log(f"{label}_relative_retreat_result={result}")
            if result.status != "completed":
                raise RuntimeError(f"{label} relative retreat failed: {result}")
            return result

        return self.with_industrial_rack(_run)

    def _retreat_by_hybrid_relative(self):
        """
        后退 1m 的生产默认策略。

        第一选择是 PNC relative_move(x=-distance)，这是当前 GDK 栈里唯一
        具有“相对位移任务”语义的方法。前超声只能测与前方物体的距离变化，
        不能证明底盘真实后退了 1m。

        贴近料架后 relative_move 可能被 motion_control_error=2/collision
        imminent 拦住。此时只允许做一个很短的双前雷达一致性脱离段，然后
        再把剩余距离交给 relative_move。脱离段失败或双前雷达增量不一致
        时，流程直接停机，不继续抓放。
        """
        target_m = self.config.retreat_distance_m
        try:
            self._retreat_relative_once(target_m, "hybrid_primary")
            self.log(f"hybrid_retreat_result status=completed mode=relative distance_m={target_m}")
            return
        except Exception as exc:
            message = str(exc)
            recoverable = any(
                token in message
                for token in (
                    "motion_control_error=2",
                    "collision",
                    "not_started",
                    "canceled",
                    "state=8",
                    "relative_move",
                    "RequestChassisControl failed",
                    "CancelTask failed",
                )
            )
            if not recoverable:
                raise
            self.log(f"hybrid_retreat_primary_failed recoverable=True error={exc}")

        escape_m = min(self.config.retreat_escape_delta_m, target_m)
        if escape_m <= 0.0:
            raise RuntimeError("hybrid retreat cannot escape: retreat_escape_delta_m <= 0")
        if escape_m >= target_m:
            raise RuntimeError("hybrid retreat escape delta must be smaller than target distance")

        self.log(
            "hybrid_retreat_escape_start "
            f"escape_delta_m={escape_m} remaining_relative_m={target_m - escape_m:.3f}"
        )
        self._retreat_by_front_ultrasonic_delta(
            distance_m=escape_m,
            tolerance_mm=20,
            max_duration_s=max(6.0, escape_m / max(self.config.retreat_speed_mps, 0.01) + 4.0),
        )

        remaining_m = target_m - escape_m
        self._retreat_relative_once(remaining_m, "hybrid_remaining")
        self.log(
            "hybrid_retreat_result status=completed "
            f"escape_delta_m={escape_m} remaining_relative_m={remaining_m:.3f} "
            "note=first_escape_uses_dual_front_ultrasonic_delta"
        )

    def _retreat_by_front_ultrasonic_delta(
        self,
        distance_m: float | None = None,
        tolerance_mm: int | None = None,
        rear_stop_mm: int = 700,
        rear_hard_stop_mm: int = 500,
        rear_stop_min_sensors: int = 2,
        hz: float = 20.0,
        history_size: int = 3,
        max_duration_s: float = 20.0,
        lost_timeout_s: float = 1.0,
        inconsistent_timeout_s: float = 1.2,
    ):
        """
        用前方超声“距离增加量”闭环后退。

        目的：
          贴近料架后，底盘会进入 motion_control_error=2/collision imminent，
          这时 relative_move(x=-1m) 可能不接任务。这个方法不按时间估算，
          而是持续读取前方 0/1 超声，让前方实测距离增加 1000mm。

        工业策略：
          - 远离目标还很多时用用户要求的 0.50m/s 提高节拍；
          - 接近目标时自动降速，避免制动惯性导致多退；
          - 如果停稳后发现退少了，就继续慢速后退；
          - 如果停稳后发现退多了，就慢速向前补回来；
          - 后方 4/5 持续保护；双后探头近障立即失败；
          - 单后探头 <=500mm 先停车复查，避免 4/5 偶发假低值误杀；
          - 不能稳定读取前方超声时失败停机，不继续后续抓放动作。
        """
        rack_package_dir = str(self.config.base_dir / "rack_hybrid_docking_package")
        if rack_package_dir not in sys.path:
            sys.path.insert(0, rack_package_dir)

        from rack_radar_docking import INVALID_DISTANCE_MM, RackRadarDockingController
        import agibot_gdk

        retreat_distance_m = self.config.retreat_distance_m if distance_m is None else distance_m
        if retreat_distance_m <= 0.0:
            raise ValueError("front-ultrasonic retreat distance_m must be positive")
        tolerance_mm = (
            self.config.retreat_target_tolerance_mm
            if tolerance_mm is None
            else int(tolerance_mm)
        )
        if tolerance_mm <= 0:
            raise ValueError("front-ultrasonic retreat tolerance_mm must be positive")
        target_delta_mm = int(round(retreat_distance_m * 1000.0))
        max_front_delta_span_mm = self.config.retreat_front_delta_consistency_mm
        interval_s = 1.0 / hz
        start = time.time()
        last_front_seen_s = start
        inconsistent_since_s = None
        front_histories: dict[int, list[int]] = {radar_id: [] for radar_id in FRONT_ULTRASONIC_IDS}
        rear_history: list[int] = []
        samples = 0

        def selected_raw(radar, ids: tuple[int, ...]):
            try:
                data = radar.radar.get_latest_ultrasonic_radar()
            except Exception:
                return tuple()
            distances = []
            for row in data.get("ultrasonic_radar_datas", []):
                radar_id = row.get("id")
                distance_mm = row.get("distance_mm")
                fault_state = row.get("fault_state")
                if radar_id not in ids or fault_state != 0:
                    continue
                distance_mm = self._valid_ultrasonic_distance_mm(
                    distance_mm,
                    min_valid_mm=50,
                    invalid_distance_mm=INVALID_DISTANCE_MM,
                )
                if distance_mm is not None:
                    distances.append((radar_id, distance_mm))
            return tuple(sorted(distances))

        def median(values: list[int]) -> int:
            ordered = sorted(values)
            return int(ordered[len(ordered) // 2])

        def min_distance(raw: tuple[tuple[int, int], ...]) -> int | None:
            return min((distance for _, distance in raw), default=None)

        def read_odom_crosscheck_status(start_xy, end_xy, label: str):
            if start_xy is None or end_xy is None:
                failure_message = (
                    "front-ultrasonic retreat odom crosscheck unavailable: "
                    f"start_xy={start_xy}, end_xy={end_xy}"
                )
                if self.config.retreat_require_odom_crosscheck:
                    self.log(
                        "front_ultrasonic_retreat_odom_crosscheck "
                        f"status=unavailable label={label} start_xy={start_xy} end_xy={end_xy} "
                        "required=True"
                    )
                    self.emit_event(
                        "front_ultrasonic_retreat_odom_crosscheck",
                        step_no=self.step_no,
                        rod_index=self.current_rod_index,
                        status="unavailable",
                        label=label,
                        start_xy=start_xy,
                        end_xy=end_xy,
                        odom_required=True,
                        within_tolerance=False,
                    )
                    return {
                        "status": "unavailable",
                        "label": label,
                        "available": False,
                        "within_tolerance": False,
                        "displacement_m": None,
                        "target_m": retreat_distance_m,
                        "error_m": None,
                        "tolerance_m": self.config.retreat_odom_tolerance_m,
                        "start_xy": start_xy,
                        "end_xy": end_xy,
                        "failure_message": failure_message,
                    }
                self.log(
                    "front_ultrasonic_retreat_odom_crosscheck "
                    f"status=unavailable label={label} start_xy={start_xy} end_xy={end_xy} "
                    "required=False"
                )
                return None

            displacement_m = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
            error_m = displacement_m - retreat_distance_m
            within_tolerance = (
                self.config.retreat_odom_tolerance_m < 0
                or abs(error_m) <= self.config.retreat_odom_tolerance_m
            )
            status = "available" if within_tolerance else "failed"
            self.log(
                "front_ultrasonic_retreat_odom_crosscheck "
                f"status={status} label={label} displacement_m={displacement_m:.3f} "
                f"target_m={retreat_distance_m:.3f} error_m={error_m:.3f} "
                f"tolerance_m={self.config.retreat_odom_tolerance_m:.3f} "
                f"start_xy={start_xy} end_xy={end_xy}"
            )
            self.emit_event(
                "front_ultrasonic_retreat_odom_crosscheck",
                step_no=self.step_no,
                rod_index=self.current_rod_index,
                displacement_m=round(displacement_m, 4),
                target_m=round(retreat_distance_m, 4),
                error_m=round(error_m, 4),
                tolerance_m=self.config.retreat_odom_tolerance_m,
                start_xy=start_xy,
                end_xy=end_xy,
                status=status,
                within_tolerance=within_tolerance,
            )
            failure_message = (
                "front-ultrasonic retreat odom crosscheck failed: "
                f"displacement_m={displacement_m:.3f}, target_m={retreat_distance_m:.3f}, "
                f"error_m={error_m:.3f}, tolerance_m={self.config.retreat_odom_tolerance_m:.3f}"
            )
            return {
                "status": status,
                "label": label,
                "available": True,
                "within_tolerance": within_tolerance,
                "displacement_m": displacement_m,
                "target_m": retreat_distance_m,
                "error_m": error_m,
                "tolerance_m": self.config.retreat_odom_tolerance_m,
                "start_xy": start_xy,
                "end_xy": end_xy,
                "failure_message": failure_message,
            }

        def raise_odom_crosscheck_failure(status):
            raise RuntimeError(status["failure_message"])

        def check_odom_crosscheck(start_xy, end_xy, label: str):
            status = read_odom_crosscheck_status(start_xy, end_xy, label)
            if status is None:
                return None
            if not status["within_tolerance"]:
                raise_odom_crosscheck_failure(status)
            return status["displacement_m"]

        def odom_correction_clearance(command_vx_mps: float, label: str):
            retry_deadline_s = (
                time.time()
                + self.config.retreat_odom_auto_correction_clearance_retry_s
            )
            attempt = 0
            last_problems: tuple[str, ...] = ()
            while True:
                attempt += 1
                front_raw = selected_raw(radar, FRONT_ULTRASONIC_IDS)
                rear_raw = selected_raw(radar, REAR_ULTRASONIC_IDS)
                front_min_mm = min_distance(front_raw)
                rear_min_mm = min_distance(rear_raw)
                problems = []
                if command_vx_mps > 0.0:
                    if front_min_mm is None:
                        problems.append("front_ultrasonic_unavailable")
                    elif front_min_mm < self.config.retreat_odom_auto_correction_front_hard_min_mm:
                        problems.append(
                            "front_ultrasonic_too_close="
                            f"{front_min_mm}<hard_min_"
                            f"{self.config.retreat_odom_auto_correction_front_hard_min_mm}"
                        )
                if command_vx_mps < 0.0:
                    if rear_min_mm is None:
                        problems.append("rear_ultrasonic_unavailable")
                    elif rear_min_mm < self.config.retreat_odom_auto_correction_rear_hard_min_mm:
                        problems.append(
                            "rear_ultrasonic_too_close="
                            f"{rear_min_mm}<hard_min_"
                            f"{self.config.retreat_odom_auto_correction_rear_hard_min_mm}"
                        )

                last_problems = tuple(problems)
                retryable = bool(problems) and all(
                    problem.endswith("_ultrasonic_unavailable")
                    for problem in problems
                )
                self.log(
                    "front_ultrasonic_retreat_odom_auto_correction_clearance "
                    f"label={label} attempt={attempt} "
                    f"command_vx_mps={command_vx_mps:.3f} "
                    f"front_min_mm={front_min_mm} rear_min_mm={rear_min_mm} "
                    f"front_raw={front_raw} rear_raw={rear_raw} "
                    f"problems={last_problems} retryable={retryable}"
                )
                if not problems:
                    return front_raw, rear_raw, front_min_mm, rear_min_mm
                if not retryable or time.time() >= retry_deadline_s:
                    radar.stop()
                    raise RuntimeError(
                        "front-ultrasonic retreat odom auto-correction blocked by clearance: "
                        + ", ".join(problems)
                    )
                time.sleep(0.08)

        def run_odom_auto_correction(odom_status, *, odom_label: str):
            if odom_status is None or odom_status["within_tolerance"]:
                return odom_status
            if not odom_status.get("available"):
                return odom_status
            if not self.config.retreat_odom_auto_correction:
                return odom_status
            if self.config.retreat_odom_auto_correction_max_passes <= 0:
                return odom_status
            if self.config.retreat_odom_auto_correction_max_m <= 0.0:
                return odom_status

            error_m = float(odom_status["error_m"])
            if abs(error_m) > self.config.retreat_odom_auto_correction_max_m:
                self.log(
                    "front_ultrasonic_retreat_odom_auto_correction_rejected "
                    f"label={odom_label} reason=error_too_large error_m={error_m:.4f} "
                    f"max_m={self.config.retreat_odom_auto_correction_max_m:.4f}"
                )
                self.emit_event(
                    "front_ultrasonic_retreat_odom_auto_correction_result",
                    step_no=self.step_no,
                    rod_index=self.current_rod_index,
                    label=odom_label,
                    status="rejected_error_too_large",
                    error_m=round(error_m, 4),
                    max_m=self.config.retreat_odom_auto_correction_max_m,
                )
                return odom_status

            speed_mps = self.config.retreat_odom_auto_correction_speed_mps
            interval_s = max(0.05, 1.0 / hz)
            last_status = odom_status

            for pass_index in range(1, self.config.retreat_odom_auto_correction_max_passes + 1):
                error_m = float(last_status["error_m"])
                if abs(error_m) <= self.config.retreat_odom_tolerance_m:
                    return last_status
                if abs(error_m) > self.config.retreat_odom_auto_correction_max_m:
                    break

                correction_target_m = min(
                    abs(error_m),
                    self.config.retreat_odom_auto_correction_max_m,
                )
                correction_target_m = max(
                    correction_target_m,
                    self.config.retreat_odom_auto_correction_min_m,
                )
                command_vx_mps = -speed_mps if error_m < 0.0 else speed_mps
                direction = "backward" if command_vx_mps < 0.0 else "forward"
                label = f"{odom_label}:odom_auto_correction_pass_{pass_index}"

                correction_start_xy = self._read_odom_xy_from_slam(
                    slam,
                    f"{label}:start",
                    attempts=3,
                    interval_s=0.05,
                )
                if correction_start_xy is None:
                    self.log(
                        "front_ultrasonic_retreat_odom_auto_correction_rejected "
                        f"label={label} reason=correction_start_odom_unavailable"
                    )
                    return last_status

                odom_correction_clearance(command_vx_mps, label)
                try:
                    radar.request_chassis_control_ready()
                except Exception as exc:
                    self.log(f"front_ultrasonic_retreat_odom_auto_correction_request_ready_failed={exc}")
                    radar.pnc.request_chassis_control(0)
                time.sleep(0.12)

                deadline_s = time.time() + max(2.0, correction_target_m / speed_mps + 1.5)
                command_count = 0
                measured_correction_m = 0.0
                self.log(
                    "front_ultrasonic_retreat_odom_auto_correction_start "
                    f"label={label} direction={direction} error_m={error_m:.4f} "
                    f"target_correction_m={correction_target_m:.4f} "
                    f"speed_mps={speed_mps:.3f} start_xy={correction_start_xy}"
                )
                self.emit_event(
                    "front_ultrasonic_retreat_odom_auto_correction_start",
                    step_no=self.step_no,
                    rod_index=self.current_rod_index,
                    label=label,
                    pass_index=pass_index,
                    direction=direction,
                    error_m=round(error_m, 4),
                    target_correction_m=round(correction_target_m, 4),
                    speed_mps=speed_mps,
                    start_xy=correction_start_xy,
                )

                try:
                    while time.time() < deadline_s:
                        odom_correction_clearance(command_vx_mps, label)
                        current_xy = self._read_odom_xy_from_slam(
                            slam,
                            f"{label}:sample",
                            attempts=1,
                            interval_s=0.0,
                        )
                        if current_xy is not None:
                            measured_correction_m = math.hypot(
                                current_xy[0] - correction_start_xy[0],
                                current_xy[1] - correction_start_xy[1],
                            )
                            if measured_correction_m >= correction_target_m:
                                break
                        radar.send_velocity(command_vx_mps)
                        command_count += 1
                        time.sleep(interval_s)
                finally:
                    radar.stop()
                    time.sleep(0.25)
                    radar.stop()

                corrected_end_xy = self._read_odom_xy_from_slam(
                    slam,
                    f"{label}:done",
                    attempts=5,
                    interval_s=0.08,
                )
                corrected_status = read_odom_crosscheck_status(
                    start_odom_xy,
                    corrected_end_xy,
                    label,
                )
                if corrected_status is not None:
                    last_status = corrected_status

                (
                    settled_front_filtered_by_id,
                    settled_delta_by_id,
                    settled_delta_span_mm,
                    settled_remaining_mm,
                    settled_front_raw,
                ) = read_settled_front_after_stop(settle_s=0.8)
                self.log(
                    "front_ultrasonic_retreat_odom_auto_correction_result "
                    f"label={label} status={last_status['status']} "
                    f"within_tolerance={last_status['within_tolerance']} "
                    f"measured_correction_m={measured_correction_m:.4f} "
                    f"commands={command_count} "
                    f"displacement_m={last_status['displacement_m']} "
                    f"error_m={last_status['error_m']} "
                    f"settled_front_filtered_by_id={settled_front_filtered_by_id} "
                    f"settled_delta_by_id={settled_delta_by_id} "
                    f"settled_delta_span_mm={settled_delta_span_mm} "
                    f"settled_remaining_mm={settled_remaining_mm} "
                    f"settled_front_raw={settled_front_raw}"
                )
                self.emit_event(
                    "front_ultrasonic_retreat_odom_auto_correction_result",
                    step_no=self.step_no,
                    rod_index=self.current_rod_index,
                    label=label,
                    pass_index=pass_index,
                    status=last_status["status"],
                    within_tolerance=last_status["within_tolerance"],
                    measured_correction_m=round(measured_correction_m, 4),
                    command_count=command_count,
                    displacement_m=(
                        None
                        if last_status["displacement_m"] is None
                        else round(float(last_status["displacement_m"]), 4)
                    ),
                    error_m=(
                        None
                        if last_status["error_m"] is None
                        else round(float(last_status["error_m"]), 4)
                    ),
                    settled_remaining_mm=settled_remaining_mm,
                    settled_delta_by_id=settled_delta_by_id,
                    settled_delta_span_mm=settled_delta_span_mm,
                )
                if last_status["within_tolerance"]:
                    return last_status

            return last_status
            return displacement_m

        with RackRadarDockingController(
            front_ids=FRONT_ULTRASONIC_IDS,
            control_mode=0,
            init_gdk=True,
        ) as radar:
            power = radar.robot.get_chassis_power_state()
            motion_status_error = None
            try:
                motion = read_motion_control_status_with_retry(radar.robot)
            except RuntimeError as exc:
                motion = None
                motion_status_error = str(exc)
            problems = []
            if getattr(power, "charge_plug_insert_state", 0) != 0:
                problems.append("charge_plug_insert_state=1")
            if getattr(power, "chassis_ultrasonic_radar_power_state", 0) != 1:
                problems.append("chassis_ultrasonic_radar_power_state!=1")
            if getattr(power, "emergency_stop_pedal_state", 0) != 0:
                problems.append("emergency_stop_pedal_state!=0")
            motion_error = None if motion is None else getattr(motion, "error_code", 0)
            if motion_status_error is not None:
                problems.append(f"motion_control_status_unavailable={motion_status_error}")
            elif motion_error not in (0, 2):
                problems.append(f"motion_control_error={motion_error}")
            if problems:
                raise RuntimeError("front-ultrasonic retreat blocked: " + ", ".join(problems))

            slam = agibot_gdk.Slam()
            start_odom_xy = self._read_odom_xy_from_slam(slam, "retreat_start")
            if start_odom_xy is None and self.config.retreat_require_odom_crosscheck:
                raise RuntimeError("front-ultrasonic retreat blocked: odom xy unavailable before motion")

            initial_samples: list[dict[int, int]] = []
            candidate_samples: list[dict[int, int]] = []
            latest_front_raw = ()
            start_stability_mm = min(120, max_front_delta_span_mm)
            start_dual_span_mm = max_front_delta_span_mm + 40
            for _ in range(30):
                front_raw = selected_raw(radar, FRONT_ULTRASONIC_IDS)
                front_by_id = dict(front_raw)
                if all(radar_id in front_by_id for radar_id in FRONT_ULTRASONIC_IDS):
                    latest_front_raw = front_raw
                    candidate_samples.append(front_by_id)
                    del candidate_samples[:-history_size]
                    if len(candidate_samples) >= history_size:
                        stable_window = True
                        for radar_id in FRONT_ULTRASONIC_IDS:
                            values = [sample[radar_id] for sample in candidate_samples]
                            if max(values) - min(values) > start_stability_mm:
                                stable_window = False
                                break
                        if stable_window:
                            median_by_id = {
                                radar_id: median([sample[radar_id] for sample in candidate_samples])
                                for radar_id in FRONT_ULTRASONIC_IDS
                            }
                            median_span_mm = max(median_by_id.values()) - min(median_by_id.values())
                            if median_span_mm > start_dual_span_mm:
                                stable_window = False
                        if stable_window:
                            initial_samples = list(candidate_samples)
                            break
                else:
                    candidate_samples = []
                time.sleep(0.05)
            if len(initial_samples) < history_size:
                raise RuntimeError(
                    "front-ultrasonic retreat has no stable dual-front lock before motion: "
                    f"front_raw={latest_front_raw}, required_ids={FRONT_ULTRASONIC_IDS}, "
                    f"start_stability_mm={start_stability_mm}, "
                    f"start_dual_span_mm={start_dual_span_mm}"
                )

            unstable_start = {}
            for radar_id in FRONT_ULTRASONIC_IDS:
                values = [sample[radar_id] for sample in initial_samples]
                value_span_mm = max(values) - min(values)
                if value_span_mm > start_stability_mm:
                    unstable_start[radar_id] = {
                        "values": values,
                        "span_mm": value_span_mm,
                    }
            if unstable_start:
                raise RuntimeError(
                    "front-ultrasonic retreat has unstable dual-front start samples: "
                    f"unstable_start={unstable_start}, "
                    f"start_stability_mm={start_stability_mm}, "
                    f"front_raw={latest_front_raw}"
                )

            start_front_by_id = {
                radar_id: median([sample[radar_id] for sample in initial_samples])
                for radar_id in FRONT_ULTRASONIC_IDS
            }
            target_front_by_id = {
                radar_id: start_front_by_id[radar_id] + target_delta_mm
                for radar_id in FRONT_ULTRASONIC_IDS
            }
            target_front_avg_mm = int(round(sum(target_front_by_id.values()) / len(target_front_by_id)))
            resume_hint = self.resume_hint_for_current_context()
            if (
                self.current_rod_index is not None
                and self.current_step_title is not None
                and "放料后后退" in self.current_step_title
            ):
                resume_hint = (
                    f"--resume-after-place-retreat-target-index {self.current_rod_index} "
                    f"--place-retreat-front-target-mm {target_front_avg_mm}"
                )
            self.write_checkpoint(
                status="step_running",
                resume_hint=resume_hint,
                retreat_start_front_by_id=start_front_by_id,
                retreat_target_delta_mm=target_delta_mm,
                retreat_target_front_by_id=target_front_by_id,
                retreat_target_front_avg_mm=target_front_avg_mm,
            )
            self.log(
                "front_ultrasonic_retreat_start "
                f"start_front_by_id={start_front_by_id} "
                f"target_front_by_id={target_front_by_id} "
                f"target_front_avg_mm={target_front_avg_mm} "
                f"target_delta_mm={target_delta_mm} tolerance_mm={tolerance_mm} "
                f"max_front_delta_span_mm={max_front_delta_span_mm} "
                f"start_stability_mm={start_stability_mm} "
                f"start_dual_span_mm={start_dual_span_mm}"
            )

            def read_settled_front_after_stop(settle_s: float = 2.2):
                """Stop-and-settle check for dynamic rod/rack occlusion during retreat."""
                radar.stop()
                stable_samples: list[dict[int, int]] = []
                latest_raw = ()
                deadline_s = time.time() + settle_s
                while time.time() < deadline_s:
                    front_raw = selected_raw(radar, FRONT_ULTRASONIC_IDS)
                    front_by_id = dict(front_raw)
                    if all(radar_id in front_by_id for radar_id in FRONT_ULTRASONIC_IDS):
                        latest_raw = front_raw
                        stable_samples.append(front_by_id)
                        del stable_samples[:-history_size]
                        if len(stable_samples) >= history_size:
                            stable_window = True
                            for radar_id in FRONT_ULTRASONIC_IDS:
                                values = [sample[radar_id] for sample in stable_samples]
                                if max(values) - min(values) > start_stability_mm:
                                    stable_window = False
                                    break
                            if stable_window:
                                filtered_by_id = {
                                    radar_id: median([sample[radar_id] for sample in stable_samples])
                                    for radar_id in FRONT_ULTRASONIC_IDS
                                }
                                settled_delta_by_id = {
                                    radar_id: filtered_by_id[radar_id] - start_front_by_id[radar_id]
                                    for radar_id in FRONT_ULTRASONIC_IDS
                                }
                                settled_delta_min_mm = min(settled_delta_by_id.values())
                                settled_delta_span_mm = (
                                    max(settled_delta_by_id.values()) - settled_delta_min_mm
                                )
                                settled_remaining_mm = target_delta_mm - settled_delta_min_mm
                                return (
                                    filtered_by_id,
                                    settled_delta_by_id,
                                    settled_delta_span_mm,
                                    settled_remaining_mm,
                                    front_raw,
                                )
                    else:
                        stable_samples = []
                    time.sleep(0.05)
                return None, None, None, None, latest_raw

            def confirm_rear_obstacle_after_stop(settle_s: float = 1.2):
                """
                Re-check a rear stop after braking.

                A real rear obstacle should either be seen by both rear sensors
                for several samples or persist on the same hard-stop sensor
                after the chassis has stopped.
                A one-frame 50mm spike on only one rear sensor is treated as an
                unreliable sample and the retreat may continue.
                """
                radar.stop()
                deadline_s = time.time() + settle_s
                latest_raw = ()
                latest_min_mm = None
                latest_filtered_mm = None
                single_hard_counts = {radar_id: 0 for radar_id in REAR_ULTRASONIC_IDS}
                dual_stop_count = 0
                settled_history: list[int] = []
                while time.time() < deadline_s:
                    raw = selected_raw(radar, REAR_ULTRASONIC_IDS)
                    latest_raw = raw
                    latest_min_mm = min_distance(raw)
                    if latest_min_mm is not None:
                        settled_history.append(latest_min_mm)
                        settled_history = settled_history[-history_size:]
                        if len(settled_history) >= history_size:
                            latest_filtered_mm = median(settled_history)

                    sensors_under_stop = sum(
                        1 for _, distance_mm in raw if distance_mm <= rear_stop_mm
                    )
                    if sensors_under_stop >= rear_stop_min_sensors:
                        dual_stop_count += 1
                    else:
                        dual_stop_count = 0
                    if dual_stop_count >= history_size:
                        return (
                            True,
                            "rear obstacle confirmed after stop: persistent dual rear sensors under stop",
                            latest_min_mm,
                            latest_filtered_mm,
                            raw,
                        )

                    hard_ids = {
                        radar_id
                        for radar_id, distance_mm in raw
                        if distance_mm <= rear_hard_stop_mm
                    }
                    for radar_id in REAR_ULTRASONIC_IDS:
                        if radar_id in hard_ids:
                            single_hard_counts[radar_id] += 1
                        else:
                            single_hard_counts[radar_id] = 0
                    if max(single_hard_counts.values(), default=0) >= history_size:
                        return (
                            True,
                            "rear obstacle confirmed after stop: persistent single rear hard-stop",
                            latest_min_mm,
                            latest_filtered_mm,
                            raw,
                        )
                    time.sleep(0.05)
                return (
                    False,
                    "rear stop not confirmed after stop",
                    latest_min_mm,
                    latest_filtered_mm,
                    latest_raw,
                )

            def finish_retreat_result(
                *,
                status: str,
                elapsed_s: float,
                front_filtered_by_id: dict[int, int],
                delta_by_id: dict[int, int],
                remaining_mm: int,
                front_raw: tuple[tuple[int, int], ...],
                rear_raw: tuple[tuple[int, int], ...],
                odom_label: str,
                require_odom: bool = False,
                strict_odom_error_m: float | None = None,
            ):
                radar.stop()
                end_odom_xy = self._read_odom_xy_from_slam(slam, "retreat_done")
                odom_status = read_odom_crosscheck_status(
                    start_odom_xy,
                    end_odom_xy,
                    odom_label,
                )
                if odom_status is not None and not odom_status["within_tolerance"]:
                    odom_status = run_odom_auto_correction(
                        odom_status,
                        odom_label=odom_label,
                    )
                if odom_status is not None and not odom_status["within_tolerance"]:
                    raise_odom_crosscheck_failure(odom_status)
                odom_displacement_m = (
                    None if odom_status is None else odom_status["displacement_m"]
                )
                if require_odom and odom_displacement_m is None:
                    self.log(
                        "front_ultrasonic_retreat_fallback_rejected "
                        f"reason=odom_unavailable status={status} "
                        f"delta_by_id={delta_by_id} front_raw={front_raw}"
                    )
                    return None
                if strict_odom_error_m is not None and odom_displacement_m is not None:
                    odom_error_m = odom_displacement_m - retreat_distance_m
                    if abs(odom_error_m) > strict_odom_error_m:
                        self.log(
                            "front_ultrasonic_retreat_fallback_rejected "
                            f"reason=odom_error_too_large status={status} "
                            f"odom_displacement_m={odom_displacement_m:.3f} "
                            f"target_m={retreat_distance_m:.3f} "
                            f"strict_tolerance_m={strict_odom_error_m:.3f} "
                            f"delta_by_id={delta_by_id} front_raw={front_raw}"
                        )
                        return None
                self.emit_event(
                    "front_ultrasonic_retreat_target_verified",
                    step_no=self.step_no,
                    rod_index=self.current_rod_index,
                    status=status,
                    target_delta_mm=target_delta_mm,
                    tolerance_mm=tolerance_mm,
                    remaining_mm=remaining_mm,
                    delta_by_id=delta_by_id,
                    front_filtered_by_id=front_filtered_by_id,
                    odom_displacement_m=odom_displacement_m,
                    odom_tolerance_m=self.config.retreat_odom_tolerance_m,
                    odom_required=self.config.retreat_require_odom_crosscheck,
                )
                self.log(
                    f"front_ultrasonic_retreat_result status={status} "
                    f"elapsed_s={elapsed_s:.2f} samples={samples} "
                    f"start_front_by_id={start_front_by_id} "
                    f"front_filtered_by_id={front_filtered_by_id} "
                    f"delta_by_id={delta_by_id} "
                    f"remaining_mm={remaining_mm} "
                    f"front_raw={front_raw} rear_raw={rear_raw} "
                    f"odom_displacement_m={odom_displacement_m}"
                )
                return delta_by_id

            try:
                radar.request_chassis_control_ready()
            except Exception as exc:
                self.log(f"front_ultrasonic_retreat_request_ready_failed={exc}")
                radar.pnc.request_chassis_control(0)
            time.sleep(0.2)

            try:
                while True:
                    elapsed_s = time.time() - start
                    if elapsed_s > max_duration_s:
                        radar.stop()
                        raise RuntimeError(
                            "front-ultrasonic retreat timeout: "
                            f"elapsed_s={elapsed_s:.2f}, target_delta_mm={target_delta_mm}"
                        )

                    front_raw = selected_raw(radar, FRONT_ULTRASONIC_IDS)
                    rear_raw = selected_raw(radar, REAR_ULTRASONIC_IDS)
                    front_min_mm = min_distance(front_raw)
                    rear_min_mm = min_distance(rear_raw)
                    samples += 1

                    front_by_id = dict(front_raw)
                    if not all(radar_id in front_by_id for radar_id in FRONT_ULTRASONIC_IDS):
                        radar.stop()
                        front_filtered_by_id = None
                        delta_by_id = None
                        delta_min_mm = None
                        if time.time() - last_front_seen_s > lost_timeout_s:
                            raise RuntimeError(
                                "front-ultrasonic retreat lost dual-front radar: "
                                f"last_front_raw={front_raw}, rear_raw={rear_raw}"
                            )
                    else:
                        last_front_seen_s = time.time()
                        for radar_id in FRONT_ULTRASONIC_IDS:
                            history = front_histories[radar_id]
                            history.append(front_by_id[radar_id])
                            del history[:-history_size]
                        if all(
                            len(front_histories[radar_id]) >= history_size
                            for radar_id in FRONT_ULTRASONIC_IDS
                        ):
                            front_filtered_by_id = {
                                radar_id: median(front_histories[radar_id])
                                for radar_id in FRONT_ULTRASONIC_IDS
                            }
                            delta_by_id = {
                                radar_id: front_filtered_by_id[radar_id] - start_front_by_id[radar_id]
                                for radar_id in FRONT_ULTRASONIC_IDS
                            }
                            delta_min_mm = min(delta_by_id.values())
                            delta_span_mm = max(delta_by_id.values()) - delta_min_mm
                            if delta_span_mm > max_front_delta_span_mm:
                                radar.stop()
                                now = time.time()
                                if inconsistent_since_s is None:
                                    inconsistent_since_s = now
                                self.log(
                                    "front_ultrasonic_retreat_inconsistent_delta "
                                    f"elapsed_s={elapsed_s:.2f} delta_by_id={delta_by_id} "
                                    f"delta_span_mm={delta_span_mm} "
                                    f"max_span_mm={max_front_delta_span_mm} "
                                    f"front_filtered_by_id={front_filtered_by_id} "
                                    f"front_raw={front_raw}"
                                )
                                if now - inconsistent_since_s >= inconsistent_timeout_s:
                                    (
                                        settled_front_filtered_by_id,
                                        settled_delta_by_id,
                                        settled_delta_span_mm,
                                        settled_remaining_mm,
                                        settled_front_raw,
                                    ) = read_settled_front_after_stop()
                                    if settled_delta_by_id is not None:
                                        self.log(
                                            "front_ultrasonic_retreat_settled_front_check "
                                            f"filtered_by_id={settled_front_filtered_by_id} "
                                            f"delta_by_id={settled_delta_by_id} "
                                            f"delta_span_mm={settled_delta_span_mm} "
                                            f"remaining_mm={settled_remaining_mm} "
                                            f"front_raw={settled_front_raw}"
                                        )
                                        if settled_delta_span_mm <= max_front_delta_span_mm:
                                            for radar_id in FRONT_ULTRASONIC_IDS:
                                                front_histories[radar_id] = [
                                                    settled_front_filtered_by_id[radar_id]
                                                ] * history_size
                                            if abs(settled_remaining_mm) <= tolerance_mm:
                                                completed = finish_retreat_result(
                                                    status="completed_after_settled_front",
                                                    elapsed_s=elapsed_s,
                                                    front_filtered_by_id=settled_front_filtered_by_id,
                                                    delta_by_id=settled_delta_by_id,
                                                    remaining_mm=settled_remaining_mm,
                                                    front_raw=settled_front_raw,
                                                    rear_raw=rear_raw,
                                                    odom_label="settled_front_recovery",
                                                )
                                                if completed is not None:
                                                    return completed
                                            inconsistent_since_s = None
                                            time.sleep(interval_s)
                                            continue

                                    single_front_delta_mm = max(delta_by_id.values())
                                    single_front_remaining_mm = target_delta_mm - single_front_delta_mm
                                    if abs(single_front_remaining_mm) <= tolerance_mm:
                                        strict_odom_error_m = min(
                                            0.08,
                                            self.config.retreat_odom_tolerance_m
                                            if self.config.retreat_odom_tolerance_m >= 0
                                            else 0.08,
                                        )
                                        completed = finish_retreat_result(
                                            status="completed_single_front_odom_guard",
                                            elapsed_s=elapsed_s,
                                            front_filtered_by_id=front_filtered_by_id,
                                            delta_by_id=delta_by_id,
                                            remaining_mm=single_front_remaining_mm,
                                            front_raw=front_raw,
                                            rear_raw=rear_raw,
                                            odom_label="single_front_odom_guard",
                                            require_odom=True,
                                            strict_odom_error_m=strict_odom_error_m,
                                        )
                                        if completed is not None:
                                            return completed
                                    raise RuntimeError(
                                        "front-ultrasonic retreat rejected persistent inconsistent dual-front delta: "
                                        f"delta_by_id={delta_by_id}, delta_span_mm={delta_span_mm}, "
                                        f"max_span_mm={max_front_delta_span_mm}, "
                                        f"front_filtered_by_id={front_filtered_by_id}, "
                                        f"front_raw={front_raw}"
                                    )
                                time.sleep(interval_s)
                                continue
                            inconsistent_since_s = None
                        else:
                            front_filtered_by_id = None
                            delta_by_id = None
                            delta_min_mm = None

                    if rear_min_mm is not None:
                        rear_history.append(rear_min_mm)
                        rear_history = rear_history[-history_size:]
                        rear_filtered_mm = int(sorted(rear_history)[len(rear_history) // 2])
                    else:
                        rear_history = []
                        rear_filtered_mm = None

                    sensors_under_stop = sum(
                        1 for _, distance_mm in rear_raw if distance_mm <= rear_stop_mm
                    )
                    hard_stop_hit = rear_min_mm is not None and rear_min_mm <= rear_hard_stop_mm
                    stable_stop_hit = (
                        rear_filtered_mm is not None
                        and len(rear_history) >= history_size
                        and rear_filtered_mm <= rear_stop_mm
                        and sensors_under_stop >= rear_stop_min_sensors
                    )
                    if hard_stop_hit or stable_stop_hit:
                        radar.stop()
                        (
                            rear_confirmed,
                            reason,
                            checked_rear_min_mm,
                            checked_rear_filtered_mm,
                            checked_rear_raw,
                        ) = confirm_rear_obstacle_after_stop()
                        self.log(
                            "front_ultrasonic_retreat_rear_guard_check "
                            f"confirmed={rear_confirmed} reason={reason} "
                            f"hard_stop_hit={hard_stop_hit} stable_stop_hit={stable_stop_hit} "
                            f"initial_rear_min_mm={rear_min_mm} "
                            f"initial_rear_filtered_mm={rear_filtered_mm} "
                            f"initial_rear_raw={rear_raw} "
                            f"checked_rear_min_mm={checked_rear_min_mm} "
                            f"checked_rear_filtered_mm={checked_rear_filtered_mm} "
                            f"checked_rear_raw={checked_rear_raw}"
                        )
                        if not rear_confirmed:
                            rear_history = []
                            time.sleep(interval_s)
                            continue
                        rear_min_mm = checked_rear_min_mm
                        rear_filtered_mm = checked_rear_filtered_mm
                        rear_raw = checked_rear_raw

                        if (
                            self.current_step_title is not None
                            and "放料后后退" in self.current_step_title
                            and rear_min_mm is not None
                            and rear_min_mm > rear_hard_stop_mm
                        ):
                            (
                                settled_front_filtered_by_id,
                                settled_delta_by_id,
                                settled_delta_span_mm,
                                settled_remaining_mm,
                                settled_front_raw,
                            ) = read_settled_front_after_stop(settle_s=1.0)
                            target_window_mm = tolerance_mm
                            self.log(
                                "front_ultrasonic_retreat_rear_guard_front_target_check "
                                f"front_filtered_by_id={settled_front_filtered_by_id} "
                                f"delta_by_id={settled_delta_by_id} "
                                f"delta_span_mm={settled_delta_span_mm} "
                                f"remaining_mm={settled_remaining_mm} "
                                f"target_window_mm={target_window_mm} "
                                f"front_raw={settled_front_raw}"
                            )
                            if (
                                settled_delta_by_id is not None
                                and settled_remaining_mm is not None
                                and abs(settled_remaining_mm) <= target_window_mm
                            ):
                                self.write_checkpoint(
                                    status="blocked_rear_guard_after_place_retreat_target_reached",
                                    resume_hint=self.resume_hint_for_current_context(),
                                    retreat_front_target_reached=True,
                                    retreat_front_filtered_by_id=settled_front_filtered_by_id,
                                    retreat_delta_by_id=settled_delta_by_id,
                                    retreat_remaining_mm=settled_remaining_mm,
                                    rear_min_mm=rear_min_mm,
                                    rear_filtered_mm=rear_filtered_mm,
                                    rear_raw=rear_raw,
                                )
                                self.log(
                                    "front_ultrasonic_retreat_rear_guard_target_reached_but_blocked "
                                    f"remaining_mm={settled_remaining_mm} "
                                    f"delta_by_id={settled_delta_by_id} "
                                    f"rear_min_mm={rear_min_mm} "
                                    f"rear_filtered_mm={rear_filtered_mm} "
                                    f"rear_raw={rear_raw} "
                                    "action=stop_for_onsite_rear_clearance_check"
                                )
                        raise RuntimeError(
                            f"front-ultrasonic retreat stopped: {reason}: "
                            f"rear_min_mm={rear_min_mm}, rear_filtered_mm={rear_filtered_mm}, "
                            f"rear_raw={rear_raw}, required_rear_sensors={rear_stop_min_sensors}"
                        )

                    if delta_min_mm is None:
                        command_vx_mps = 0.0
                        remaining_mm = None
                        retreat_direction = "hold"
                    else:
                        remaining_mm = target_delta_mm - delta_min_mm
                        if abs(remaining_mm) <= tolerance_mm:
                            completed = finish_retreat_result(
                                status="completed",
                                elapsed_s=elapsed_s,
                                front_filtered_by_id=front_filtered_by_id,
                                delta_by_id=delta_by_id,
                                remaining_mm=remaining_mm,
                                front_raw=front_raw,
                                rear_raw=rear_raw,
                                odom_label="completed",
                            )
                            if completed is not None:
                                return completed

                        if remaining_mm > 0:
                            retreat_direction = "backward"
                            if remaining_mm > 350:
                                speed_mps = self.config.retreat_speed_mps
                            elif remaining_mm > 160:
                                speed_mps = min(0.20, self.config.retreat_speed_mps)
                            elif remaining_mm > 60:
                                speed_mps = 0.06
                            else:
                                speed_mps = 0.025
                            command_vx_mps = -speed_mps
                        else:
                            # 退多了以后只允许低速前补，避免再次靠近料架过冲。
                            retreat_direction = "forward_correction"
                            overshoot_mm = -remaining_mm
                            if overshoot_mm > 160:
                                speed_mps = 0.05
                            elif overshoot_mm > 60:
                                speed_mps = 0.035
                            else:
                                speed_mps = 0.018
                            command_vx_mps = speed_mps

                    if samples == 1 or samples % int(hz) == 0:
                        self.log(
                            "front_ultrasonic_retreat_sample "
                            f"elapsed_s={elapsed_s:.2f} front_min_mm={front_min_mm} "
                            f"front_filtered_by_id={front_filtered_by_id} "
                            f"delta_by_id={delta_by_id} "
                            f"remaining_mm={remaining_mm} direction={retreat_direction} "
                            f"command_vx_mps={command_vx_mps} "
                            f"front_raw={front_raw} rear_raw={rear_raw}"
                        )

                    if command_vx_mps == 0.0:
                        radar.stop()
                    else:
                        radar.send_velocity(command_vx_mps)
                    time.sleep(interval_s)
            finally:
                try:
                    radar.stop()
                    time.sleep(0.1)
                    radar.stop()
                except Exception:
                    pass

    def _retreat_command_distance_m(self) -> float:
        """
        返回真正提交给底盘的后退距离。

        front-ultrasonic 模式：不用这个距离，直接让前超声距离增加目标值。
        relative 模式：提交业务目标距离本身，由底盘导航闭环完成 1m 实距。
        velocity 模式：只作为诊断/应急，按补偿后距离开环发速度，依赖现场尺量校准。
        """
        if self.config.retreat_method == "front-ultrasonic":
            return self.config.retreat_distance_m
        if self.config.retreat_method == "relative":
            return self.config.retreat_distance_m
        compensated = self.config.retreat_distance_m - self.config.retreat_open_loop_brake_compensation_m
        return max(0.01, compensated)

    def _is_grab_retreat_step(self) -> bool:
        title = self.current_step_title or ""
        return "后退" in title and "放料后" not in title

    def _maybe_grab_retreat_front_occlusion_escape(self) -> float:
        """
        抓料拉出后，料或末端可能短时遮挡前超声。

        在这个近距离区域直接用前超声做 1m 增量闭环，容易把遮挡变化误判成
        料架距离变化。先低速后撤一个短距离，让前超声脱离遮挡，再把剩余距离
        交回原来的双前超声闭环。
        """
        if not self._is_grab_retreat_step():
            return 0.0

        threshold_mm = self.config.grab_retreat_front_occlusion_escape_threshold_mm
        escape_m = self.config.grab_retreat_front_occlusion_escape_m
        speed_mps = self.config.grab_retreat_front_occlusion_escape_speed_mps
        if threshold_mm <= 0 or escape_m <= 0.0:
            return 0.0
        if escape_m >= self.config.retreat_distance_m:
            raise RuntimeError(
                "grab retreat front occlusion escape must be smaller than retreat distance: "
                f"escape_m={escape_m}, retreat_distance_m={self.config.retreat_distance_m}"
            )

        def _run(rack):
            state = self._read_stable_front_state(
                rack=rack,
                label="grab_retreat_front_occlusion_precheck",
                consistency_tolerance_mm=max(80, self.config.grab_angle_correction_max_span_mm),
            )
            self.log(
                "grab_retreat_front_occlusion_check "
                f"threshold_mm={threshold_mm} escape_m={escape_m:.3f} "
                f"speed_mps={speed_mps:.3f} state={state}"
            )
            if state["front_min_mm"] > threshold_mm:
                return 0.0
            return min(escape_m, self.config.retreat_distance_m - 0.05)

        planned_escape_m = self.with_industrial_rack(_run)
        if planned_escape_m <= 0.0:
            return 0.0

        self.log(
            "grab_retreat_front_occlusion_escape_start "
            f"distance_m={planned_escape_m:.3f} speed_mps={speed_mps:.3f} "
            f"threshold_mm={threshold_mm}"
        )
        self._guarded_escape_retreat(
            distance_m=planned_escape_m,
            speed_mps=speed_mps,
        )
        self.log(
            "grab_retreat_front_occlusion_escape_done "
            f"distance_m={planned_escape_m:.3f} "
            f"remaining_front_ultrasonic_distance_m={self.config.retreat_distance_m - planned_escape_m:.3f}"
        )
        time.sleep(self.config.settle_s)
        return planned_escape_m

    def _is_front_collision_retreat_escape(self, preflight) -> bool:
        """
        判断是否允许从 motion_control_error=2 状态执行撤离后退。

        这个分支只允许一种情况：
          - 唯一阻塞项是 motion_control_error=2；
          - 该错误在现场表现为前方已贴近料架后的 collision imminent；
          - 后退动作本身仍会实时监控后方超声并硬停车。
        """
        detail = preflight.detail or {}
        problems = tuple(detail.get("problems", ()))
        return set(problems) == {"motion_control_error=2"}

    def _guarded_escape_retreat(
        self,
        distance_m: float | None = None,
        speed_mps: float | None = None,
        rear_stop_mm: int = 700,
        rear_hard_stop_mm: int = 500,
        rear_stop_min_sensors: int = 2,
        hz: float = 10.0,
        history_size: int = 3,
    ):
        """
        从前方近距离碰撞预警中撤离。

        普通 retreat() 会因为 motion_control_error=2 被通用 preflight 拦住；
        但当前工艺里，前方抓取精定位后必须后退离开料架。这里直接申请
        底盘控制，只发送负 X 速度，同时用后方超声 4/5 做硬保护。
        """
        if rear_hard_stop_mm > rear_stop_mm:
            raise ValueError("rear_hard_stop_mm must be <= rear_stop_mm")

        rack_package_dir = str(self.config.base_dir / "rack_hybrid_docking_package")
        if rack_package_dir not in sys.path:
            sys.path.insert(0, rack_package_dir)

        from rack_radar_docking import RackRadarDockingController

        target_distance_m = self.config.retreat_distance_m if distance_m is None else distance_m
        target_speed_mps = self.config.retreat_speed_mps if speed_mps is None else speed_mps
        if target_distance_m <= 0.0:
            raise ValueError("escape retreat distance_m must be positive")
        if target_speed_mps <= 0.0:
            raise ValueError("escape retreat speed_mps must be positive")

        target_duration_s = target_distance_m / target_speed_mps
        interval_s = 1.0 / hz
        history: list[int] = []
        samples = 0
        start = time.time()

        self.log(
            "escape_retreat_start "
            f"distance_m={target_distance_m} "
            f"speed_mps={target_speed_mps} "
            f"rear_stop_mm={rear_stop_mm} rear_hard_stop_mm={rear_hard_stop_mm}"
        )

        with RackRadarDockingController(
            front_ids=REAR_ULTRASONIC_IDS,
            control_mode=0,
            init_gdk=True,
        ) as rear:
            min_mm, distances = rear.read_min_distance()
            self.log(f"escape_retreat_before rear_min_mm={min_mm} rear_raw={distances}")
            if min_mm is not None and min_mm <= rear_hard_stop_mm:
                raise RuntimeError(
                    f"escape retreat blocked: rear hard obstacle before motion, "
                    f"rear_min_mm={min_mm}, rear_raw={distances}"
                )

            try:
                rear.request_chassis_control_ready()
            except Exception as exc:
                self.log(f"escape_retreat_request_ready_failed={exc}")
                rear.pnc.request_chassis_control(0)
            time.sleep(0.2)

            try:
                while True:
                    elapsed_s = time.time() - start
                    if elapsed_s >= target_duration_s:
                        rear.stop()
                        self.log(
                            "escape_retreat_result status=completed "
                            f"elapsed_s={elapsed_s:.2f} samples={samples}"
                        )
                        return

                    min_mm, distances = rear.read_min_distance()
                    samples += 1
                    if min_mm is None:
                        history = []
                        filtered_mm = None
                    else:
                        history.append(min_mm)
                        history = history[-history_size:]
                        filtered_mm = int(sorted(history)[len(history) // 2])

                    if samples == 1 or samples % max(1, int(hz)) == 0:
                        self.log(
                            "escape_retreat_sample "
                            f"elapsed_s={elapsed_s:.2f} rear_min_mm={min_mm} "
                            f"rear_filtered_mm={filtered_mm} rear_raw={distances}"
                        )

                    sensors_under_stop = sum(
                        1 for _, distance_mm in distances if distance_mm <= rear_stop_mm
                    )
                    hard_stop_hit = min_mm is not None and min_mm <= rear_hard_stop_mm
                    stable_stop_hit = (
                        filtered_mm is not None
                        and len(history) >= history_size
                        and filtered_mm <= rear_stop_mm
                        and sensors_under_stop >= rear_stop_min_sensors
                    )
                    if hard_stop_hit or stable_stop_hit:
                        rear.stop()
                        raise RuntimeError(
                            "escape retreat stopped by rear obstacle: "
                            f"rear_min_mm={min_mm}, rear_filtered_mm={filtered_mm}, "
                            f"rear_raw={distances}"
                        )

                    rear.send_velocity(-target_speed_mps)
                    time.sleep(interval_s)
            finally:
                try:
                    rear.stop()
                    time.sleep(0.1)
                    rear.stop()
                except Exception:
                    pass

    def rotate_chassis(self, angle_deg: float, title: str):
        """
        使用底盘控制原地旋转。

        现场问题记录：
          旧版本使用速度开环 request_chassis_control(mode=1) + move_chassis(Twist)，
          能发出旋转速度，但不能证明实际物理角度到 90 度。

          chassis_demo.py 提供的 rotate(angle_deg) 使用 Pnc.relative_move()。
          这更符合“右转/左转 90 度”的业务语义，所以总控默认使用这个方法。

          但 state=7 不能当成成功。state=7 表示取消/结束，不等价于“物理
          角度已经到 90 度”。默认只有 state=3/9 才允许继续。

        当前版本会额外读取 Slam.get_odom_info().orientation_euler 的 yaw：
          - relative_move 必须先达到 state=3/9；
          - 转向前后 yaw delta 必须落在目标角度容差内；
          - yaw 读不到时停止流程，不继续放料/抓下一根。

        与 chassis_demo.py 的方向约定一致：
          angle_deg > 0：逆时针，也就是向左转；
          angle_deg < 0：顺时针，也就是向右转。
        """
        self.next_step(title)
        self.log(
            f"rotate_angle_deg={angle_deg} "
            f"turn_angular_speed_radps={self.config.turn_angular_speed_radps} "
            f"turn_control_mode={self.config.turn_control_mode} "
            f"turn_method={self.config.turn_method} "
            f"turn_yaw_tolerance_deg={self.config.turn_yaw_tolerance_deg}"
        )

        if self.config.dry_run:
            self.log("dry-run: skip chassis rotate")
            self.complete_current_step("dry_run", angle_deg=angle_deg, turn_method=self.config.turn_method)
            return

        if angle_deg == 0:
            self.log("rotate_angle_deg is 0, skip")
            self.complete_current_step("skipped", angle_deg=angle_deg, turn_method=self.config.turn_method)
            return

        if self.config.turn_method == "relative":
            self._rotate_chassis_by_relative_move(angle_deg)
        elif self.config.turn_method == "velocity":
            self._rotate_chassis_by_velocity(angle_deg)
        else:
            raise RuntimeError(f"unknown turn method: {self.config.turn_method}")

        self.complete_current_step("completed", angle_deg=angle_deg, turn_method=self.config.turn_method)
        time.sleep(self.config.settle_s)

    def _rotate_chassis_by_relative_move(self, angle_deg: float):
        """
        使用 Pnc.relative_move() 执行 90 度转向，并严格监控任务生命周期。

        不直接调用 chassis_controller.py 的 rotate()，原因是该文件里的
        _wait_done() 会把 state=7 当成功，而且没有确认读到的是不是新提交的
        转向任务。现场已经复现：函数 1 秒多返回 state=7，但机器人没有完成
        90 度转向。

        这里的工业级判据：
          - 先记录提交前的 task id/state；
          - 提交 relative_move(yaw=±90)；
          - 必须看到新 task id，或看到任务进入运行态；
          - 只有新任务最终进入 state=3/9 才允许继续；
          - state=7 一律视为取消/失败，不继续执行后续放料或抓料。
        """
        self.log(
            "rotate_control=relative_move "
            f"angle_deg={angle_deg} "
            f"timeout_s={self.config.turn_timeout_s} "
            f"success_states={self.config.turn_success_states}"
        )
        self._log_turn_sensor_snapshot("before_turn")

        import agibot_gdk

        def make_turn_req(yaw_deg: float):
            req = agibot_gdk.NaviReq()
            req.target.position.x = 0.0
            req.target.position.y = 0.0
            req.target.position.z = 0.0
            half = math.radians(yaw_deg) / 2.0
            req.target.orientation.x = 0.0
            req.target.orientation.y = 0.0
            req.target.orientation.z = math.sin(half)
            req.target.orientation.w = math.cos(half)
            return req

        def read_task(pnc, label: str):
            task = pnc.get_task_state()
            state = getattr(task, "state", None)
            task_id = getattr(task, "id", None)
            task_type = getattr(task, "type", None)
            message = getattr(task, "message", "")
            self.log(
                f"relative_turn_task {label} "
                f"state={state} id={task_id} type={task_type} message={message}"
            )
            return task

        def send_zero(pnc):
            twist = agibot_gdk.Twist()
            twist.linear = agibot_gdk.Vector3()
            twist.angular = agibot_gdk.Vector3()
            twist.linear.x = twist.linear.y = twist.linear.z = 0.0
            twist.angular.x = twist.angular.y = twist.angular.z = 0.0
            for _ in range(8):
                try:
                    pnc.move_chassis(twist)
                except Exception:
                    pass
                time.sleep(0.03)

        gdk_inited = False
        pnc = None
        started = time.time()
        turn_started = False
        before_yaw_deg = None
        after_yaw_deg = None
        try:
            result = agibot_gdk.gdk_init()
            gdk_res = getattr(agibot_gdk, "GDKRes", None)
            if gdk_res is not None and result not in (None, gdk_res.kSuccess):
                raise RuntimeError(f"GDK init failed before relative turn: {result}")
            gdk_inited = True

            robot = agibot_gdk.Robot()
            pnc = agibot_gdk.Pnc()
            slam = agibot_gdk.Slam()
            time.sleep(0.8)
            self._check_turn_motion_safety(robot)
            before_yaw_deg = self._read_turn_yaw_from_slam(slam, "before_turn")
            if before_yaw_deg is None:
                raise RuntimeError("relative turn cannot start: yaw feedback unavailable")

            before_state = None
            before_id = None
            try:
                before_task = read_task(pnc, "before_submit")
                before_state = getattr(before_task, "state", None)
                before_id = getattr(before_task, "id", None)
                if before_state not in (0, 3, 7, 8, 9):
                    self.log(f"relative_turn_cancel_blocking_task id={before_id} state={before_state}")
                    try:
                        pnc.cancel_task(before_id)
                        time.sleep(0.8)
                        read_task(pnc, "after_cancel")
                    except RuntimeError as exc:
                        if "Task is not in RUNNING or PAUSED state" not in str(exc):
                            raise
            except Exception as exc:
                self.log(f"relative_turn_before_task_read_failed={type(exc).__name__}: {exc}")

            req = make_turn_req(angle_deg)
            self.log("relative_turn_submit_request")
            pnc.relative_move(req)
            turn_started = True

            final_state = self._wait_relative_turn_done(
                pnc=pnc,
                before_task_id=before_id,
                timeout_s=self.config.turn_timeout_s,
            )
            time.sleep(0.5)
            after_yaw_deg = self._read_turn_yaw_from_slam(slam, "after_turn")
            after_yaw_deg = self._confirm_turn_yaw_after_settle(
                slam=slam,
                expected_delta_deg=angle_deg,
                before_yaw_deg=before_yaw_deg,
                label="relative_turn_confirm_before_correction",
            )
            after_yaw_deg = self._correct_turn_yaw_if_needed(
                pnc=pnc,
                slam=slam,
                expected_delta_deg=angle_deg,
                before_yaw_deg=before_yaw_deg,
                current_yaw_deg=after_yaw_deg,
                label="relative_turn_confirmed",
                robot=robot,
            )
            after_yaw_deg = self._confirm_turn_yaw_after_settle(
                slam=slam,
                expected_delta_deg=angle_deg,
                before_yaw_deg=before_yaw_deg,
                label="relative_turn_final_confirm",
            )
        except Exception as exc:
            if pnc is not None and turn_started:
                try:
                    send_zero(pnc)
                except Exception:
                    pass
            self._log_turn_sensor_snapshot("after_turn_failed")
            raise RuntimeError(f"relative turn failed: {type(exc).__name__}: {exc}") from exc
        finally:
            if gdk_inited:
                try:
                    agibot_gdk.gdk_release()
                except Exception:
                    pass

        elapsed_s = time.time() - started
        self.log(f"relative_turn_result state={final_state} elapsed_s={elapsed_s:.2f}")
        self._log_turn_sensor_snapshot("after_turn")

        if final_state not in self.config.turn_success_states:
            raise RuntimeError(
                "relative turn did not reach an accepted final state: "
                f"state={final_state}, accepted={self.config.turn_success_states}. "
                "Stop here; do not continue the industrial sequence until the "
                "90 degree turn is confirmed."
            )

        self._validate_turn_yaw_delta(
            label="relative_turn",
            expected_delta_deg=angle_deg,
            before_yaw_deg=before_yaw_deg,
            after_yaw_deg=after_yaw_deg,
        )

    def _wait_relative_turn_done(self, pnc, before_task_id, timeout_s: float) -> int:
        """
        等待 relative_move 的真实新任务结束。

        这个函数专门防止读到旧 state=7 造成“假完成”。如果提交后一直没有
        新任务，也没有进入运行态，会直接失败，而不是继续执行后续工艺。
        """
        deadline = time.time() + timeout_s
        seen_new_task = False
        seen_running = False
        last_state = None
        last_task_id = None
        last_log_s = 0.0

        while time.time() < deadline:
            time.sleep(0.25)
            try:
                task = pnc.get_task_state()
            except Exception as exc:
                self.log(f"relative_turn_task_read_failed={type(exc).__name__}: {exc}")
                continue

            state = getattr(task, "state", None)
            task_id = getattr(task, "id", None)
            task_type = getattr(task, "type", None)
            message = getattr(task, "message", "")
            now = time.time()
            elapsed_s = timeout_s - (deadline - now)

            if now - last_log_s >= 1.0 or state != last_state or task_id != last_task_id:
                self.log(
                    "relative_turn_task poll "
                    f"elapsed_s={elapsed_s:.2f} state={state} "
                    f"id={task_id} type={task_type} message={message}"
                )
                last_log_s = now
                last_state = state
                last_task_id = task_id

            if task_id is not None and task_id != before_task_id:
                seen_new_task = True
            if state in (1, 2, 4, 5, 6, 8):
                seen_running = True

            if not seen_new_task and not seen_running:
                # 旧的 7/9/0 状态不能证明新转向任务已经开始。
                if elapsed_s >= 4.0:
                    raise RuntimeError(
                        "relative turn task did not start within 4s: "
                        f"before_task_id={before_task_id}, last_state={state}, "
                        f"last_task_id={task_id}, message={message}"
                    )
                continue

            if state == 7:
                raise RuntimeError(
                    "relative turn task was canceled before success: "
                    f"state={state}, id={task_id}, before_id={before_task_id}, "
                    f"message={message}"
                )
            if state in self.config.turn_success_states:
                return int(state)

        raise RuntimeError(
            "relative turn timed out or never started: "
            f"before_task_id={before_task_id}, last_state={last_state}, "
            f"last_task_id={last_task_id}, seen_new_task={seen_new_task}, "
            f"seen_running={seen_running}"
        )

    def _rotate_chassis_by_velocity(self, angle_deg: float):
        """
        使用速度控制 + odom yaw 闭环转向。

        request_chassis_control + move_chassis 是当前实机上可用的底盘控制链路。
        这里不再依赖固定 3 秒开环，而是实时用 SLAM odom yaw 收敛到目标角度。
        """
        nominal_duration_s = self._turn_duration_for_angle(angle_deg)
        max_duration_s = min(
            self.config.turn_timeout_s,
            max(nominal_duration_s * 2.0, nominal_duration_s + 3.0, 6.0),
        )
        if max_duration_s <= 0.0:
            raise RuntimeError(
                f"turn duration {max_duration_s:.2f}s exceeds timeout "
                f"{self.config.turn_timeout_s:.2f}s"
            )

        direction = 1.0 if angle_deg > 0 else -1.0
        angular_z = direction * self.config.turn_angular_speed_radps
        expected_yaw_delta_deg = -angle_deg
        self.log(
            "rotate_control=velocity_yaw_closed_loop "
            f"mode={self.config.turn_control_mode} "
            f"initial_angular_z={angular_z:.3f} nominal_duration_s={nominal_duration_s:.2f} "
            f"max_duration_s={max_duration_s:.2f} "
            f"business_angle_deg={angle_deg:.3f} "
            f"expected_yaw_delta_deg={expected_yaw_delta_deg:.3f} "
            "feedback=odom_yaw"
        )

        import agibot_gdk

        gdk_inited = False
        pnc = None
        before = None
        after = None
        before_yaw_deg = None
        after_yaw_deg = None
        try:
            result = agibot_gdk.gdk_init()
            gdk_res = getattr(agibot_gdk, "GDKRes", None)
            if gdk_res is not None and result not in (None, gdk_res.kSuccess):
                raise RuntimeError(f"GDK init failed before velocity turn: {result}")
            gdk_inited = True

            robot = agibot_gdk.Robot()
            pnc = agibot_gdk.Pnc()
            slam = agibot_gdk.Slam()
            time.sleep(0.6)
            self._check_turn_motion_safety(robot)
            self._cancel_chassis_task_if_blocking(pnc)

            before = self._log_turn_sensor_snapshot("before_turn")
            before_yaw_deg = self._read_turn_yaw_from_slam(slam, "before_turn")
            if before_yaw_deg is None:
                raise RuntimeError("velocity turn cannot start: yaw feedback unavailable")

            self._request_turn_chassis_control(pnc, mode=self.config.turn_control_mode)
            time.sleep(0.2)
            after_yaw_deg = self._drive_chassis_yaw_closed_loop(
                pnc=pnc,
                slam=slam,
                robot=robot,
                expected_delta_deg=expected_yaw_delta_deg,
                before_yaw_deg=before_yaw_deg,
                max_duration_s=max_duration_s,
            )
            after_yaw_deg = self._confirm_turn_yaw_after_settle(
                slam=slam,
                expected_delta_deg=expected_yaw_delta_deg,
                before_yaw_deg=before_yaw_deg,
                label="velocity_turn_confirm_before_correction",
            )
            after_yaw_deg = self._correct_turn_yaw_if_needed(
                pnc=pnc,
                slam=slam,
                expected_delta_deg=expected_yaw_delta_deg,
                before_yaw_deg=before_yaw_deg,
                current_yaw_deg=after_yaw_deg,
                label="velocity_turn_confirmed",
                robot=robot,
            )
            after_yaw_deg = self._confirm_turn_yaw_after_settle(
                slam=slam,
                expected_delta_deg=expected_yaw_delta_deg,
                before_yaw_deg=before_yaw_deg,
                label="velocity_turn_final_confirm",
            )
            after = self._log_turn_sensor_snapshot("after_turn")
        except Exception as exc:
            self._log_turn_sensor_snapshot("after_turn_failed")
            raise RuntimeError(f"velocity turn failed: {type(exc).__name__}: {exc}") from exc
        finally:
            if pnc is not None:
                try:
                    self._send_turn_velocity_existing_pnc(
                        pnc=pnc,
                        wz_radps=0.0,
                        duration_s=0.15,
                        hz=self.config.turn_hz,
                    )
                except Exception:
                    pass
            if gdk_inited:
                try:
                    agibot_gdk.gdk_release()
                except Exception:
                    pass

        self._validate_turn_yaw_delta(
            label="velocity_turn",
            expected_delta_deg=expected_yaw_delta_deg,
            before_yaw_deg=before_yaw_deg,
            after_yaw_deg=after_yaw_deg,
        )
        self._validate_velocity_turn_observed(angle_deg, before, after)

    def _log_turn_sensor_snapshot(self, label: str):
        """记录转向前后的四向超声快照，用于现场转角问题追溯。"""
        snapshot = self._read_turn_sensor_snapshot()
        if snapshot is None:
            self.log(f"turn_sensor_snapshot_failed {label}: unavailable")
            return None

        self.log(
            f"turn_sensor_snapshot {label} "
            f"front_min_mm={snapshot['front'][0]} front_raw={snapshot['front'][1]} "
            f"right_min_mm={snapshot['right'][0]} right_raw={snapshot['right'][1]} "
            f"rear_min_mm={snapshot['rear'][0]} rear_raw={snapshot['rear'][1]} "
            f"left_min_mm={snapshot['left'][0]} left_raw={snapshot['left'][1]}"
        )
        return snapshot

    def _read_turn_sensor_snapshot(self):
        """一次性读取 8 个超声并按四向分组，避免转向前后反复 GDK 初始化。"""
        rack_package_dir = str(self.config.base_dir / "rack_hybrid_docking_package")
        if rack_package_dir not in sys.path:
            sys.path.insert(0, rack_package_dir)

        import agibot_gdk
        from rack_radar_docking import INVALID_DISTANCE_MM

        groups = {
            "front": FRONT_ULTRASONIC_IDS,
            "right": RIGHT_ULTRASONIC_IDS,
            "rear": REAR_ULTRASONIC_IDS,
            "left": LEFT_ULTRASONIC_IDS,
        }
        gdk_inited = False
        radar = None
        try:
            result = agibot_gdk.gdk_init()
            gdk_res = getattr(agibot_gdk, "GDKRes", None)
            if gdk_res is not None and result not in (None, gdk_res.kSuccess):
                raise RuntimeError(f"GDK init failed: {result}")
            gdk_inited = True
            radar = agibot_gdk.UltrasonicRadar()
            time.sleep(0.25)
            data = radar.get_latest_ultrasonic_radar()
            all_distances = {}
            for row in data.get("ultrasonic_radar_datas", []):
                radar_id = row.get("id")
                distance_mm = row.get("distance_mm")
                fault_state = row.get("fault_state")
                if fault_state != 0:
                    continue
                distance_mm = self._valid_ultrasonic_distance_mm(
                    distance_mm,
                    min_valid_mm=50,
                    invalid_distance_mm=INVALID_DISTANCE_MM,
                )
                if distance_mm is not None:
                    all_distances[radar_id] = distance_mm

            snapshot = {}
            for name, ids in groups.items():
                raw = tuple((radar_id, all_distances[radar_id]) for radar_id in ids if radar_id in all_distances)
                min_mm = min((distance for _, distance in raw), default=None)
                snapshot[name] = (min_mm, raw)
            return snapshot
        except Exception as exc:
            self.log(f"turn_sensor_snapshot_read_failed: {exc}")
            return None
        finally:
            if radar is not None:
                try:
                    radar.close_ultrasonic_radar()
                except Exception:
                    pass
            if gdk_inited:
                try:
                    agibot_gdk.gdk_release()
                except Exception:
                    pass

    def _check_turn_motion_safety(self, robot):
        """转向前检查会直接阻断运动的底盘状态。"""
        power = robot.get_chassis_power_state()
        motion_status_error = None
        try:
            motion = read_motion_control_status_with_retry(robot)
        except RuntimeError as exc:
            motion = None
            motion_status_error = str(exc)

        problems = []
        warnings = []
        motion_error = None if motion is None else getattr(motion, "error_code", 0)
        charge_plug = getattr(power, "charge_plug_insert_state", 0)
        estop_state = getattr(power, "emergency_stop_pedal_state", 0)
        estop_fault = getattr(power, "emergency_stop_pedal_fault_state", 0)
        ultrasonic_power = getattr(power, "chassis_ultrasonic_radar_power_state", 0)

        if motion_status_error is not None:
            problems.append(f"motion_control_status_unavailable={motion_status_error}")
        elif motion_error == 2 and self.config.allow_turn_motion_error_2:
            collision_pairs_1 = getattr(motion, "collision_pairs_1", ()) or ()
            collision_pairs_2 = getattr(motion, "collision_pairs_2", ()) or ()
            if collision_pairs_1 or collision_pairs_2:
                problems.append(
                    "motion_control_error=2_with_collision_pairs="
                    f"{collision_pairs_1},{collision_pairs_2}"
                )
            else:
                warnings.append(
                    "motion_control_error=2 ignored by explicit "
                    "--allow-turn-motion-error-2 override"
                )
        elif motion_error != 0:
            problems.append(f"motion_control_error={motion_error}")
        if charge_plug != 0:
            problems.append("charge_plug_insert_state=1")
        if estop_state != 0:
            problems.append("emergency_stop_pedal_state!=0")
        if ultrasonic_power != 1:
            problems.append("chassis_ultrasonic_radar_power_state!=1")

        self.log(
            "turn_preflight "
            f"motion_error={motion_error} charge_plug_insert_state={charge_plug} "
            f"emergency_stop_pedal_state={estop_state} "
            f"emergency_stop_pedal_fault_state={estop_fault} "
            f"chassis_ultrasonic_radar_power_state={ultrasonic_power} "
            f"warnings={tuple(warnings)} problems={tuple(problems)}"
        )
        if problems:
            raise RuntimeError("turn preflight blocked: " + ", ".join(problems))

    def _read_turn_yaw_snapshot(self, label: str) -> float | None:
        """读取一次 SLAM odom yaw，失败时返回 None 并记录原因。"""
        import agibot_gdk

        gdk_inited = False
        try:
            result = agibot_gdk.gdk_init()
            gdk_res = getattr(agibot_gdk, "GDKRes", None)
            if gdk_res is not None and result not in (None, gdk_res.kSuccess):
                raise RuntimeError(f"GDK init failed: {result}")
            gdk_inited = True
            slam = agibot_gdk.Slam()
            time.sleep(0.25)
            return self._read_turn_yaw_from_slam(slam, label)
        except Exception as exc:
            self.log(f"turn_yaw_read_failed label={label} error={type(exc).__name__}: {exc}")
            return None
        finally:
            if gdk_inited:
                try:
                    agibot_gdk.gdk_release()
                except Exception:
                    pass

    def _read_turn_yaw_from_slam(self, slam, label: str) -> float | None:
        try:
            odom = slam.get_odom_info()
        except Exception as exc:
            self.log(f"turn_yaw_read_failed label={label} error={type(exc).__name__}: {exc}")
            return None

        yaw_deg = self._extract_yaw_deg_from_odom(odom)
        if yaw_deg is None:
            self.log(f"turn_yaw_unavailable label={label} odom={odom!r}")
            return None

        self.log(f"turn_yaw label={label} yaw_deg={yaw_deg:.3f} odom={odom!r}")
        return yaw_deg

    @staticmethod
    def _valid_ultrasonic_distance_mm(
        value,
        min_valid_mm: int = 50,
        invalid_distance_mm: int = 65535,
    ) -> int | None:
        if value is None:
            return None
        try:
            distance_mm = int(value)
        except (TypeError, ValueError):
            return None
        if min_valid_mm <= distance_mm < invalid_distance_mm:
            return distance_mm
        return None

    def _read_odom_xy_from_slam(
        self,
        slam,
        label: str,
        attempts: int = 12,
        interval_s: float = 0.18,
    ) -> tuple[float, float] | None:
        last_error = None
        last_odom = None
        for attempt in range(1, attempts + 1):
            try:
                odom = slam.get_odom_info()
            except Exception as exc:
                last_error = f"{type(exc).__name__}: {exc}"
                self.log(
                    f"odom_xy_read_failed label={label} attempt={attempt}/{attempts} "
                    f"error={last_error}"
                )
            else:
                last_odom = odom
                xy = self._extract_xy_from_odom(odom)
                if xy is not None:
                    self.log(
                        f"odom_xy label={label} attempt={attempt}/{attempts} "
                        f"x={xy[0]:.4f} y={xy[1]:.4f} odom={odom!r}"
                    )
                    return xy
                self.log(
                    f"odom_xy_unavailable label={label} attempt={attempt}/{attempts} "
                    f"odom={odom!r}"
                )
            if attempt < attempts:
                time.sleep(interval_s)

        self.log(
            f"odom_xy_unavailable_after_retry label={label} attempts={attempts} "
            f"last_error={last_error} last_odom={last_odom!r}"
        )
        return None

    @staticmethod
    def _extract_yaw_deg_from_odom(odom) -> float | None:
        orientation_euler = getattr(odom, "orientation_euler", None)

        yaw_rad = None
        if orientation_euler is not None:
            for attr in ("z", "yaw"):
                if hasattr(orientation_euler, attr):
                    yaw_rad = getattr(orientation_euler, attr)
                    break
            if yaw_rad is None:
                try:
                    yaw_rad = orientation_euler[2]
                except Exception:
                    yaw_rad = None
        if yaw_rad is None:
            match = re.search(r"orientation_euler=\(([^)]*)\)", repr(odom))
            if match is not None:
                parts = [part.strip() for part in match.group(1).split(",")]
                if len(parts) >= 3:
                    yaw_rad = parts[2]
        try:
            return math.degrees(float(yaw_rad))
        except Exception:
            return None

    @staticmethod
    def _extract_xy_from_odom(odom) -> tuple[float, float] | None:
        pose = getattr(odom, "pose", None)

        if pose is not None and hasattr(pose, "x") and hasattr(pose, "y"):
            try:
                return float(getattr(pose, "x")), float(getattr(pose, "y"))
            except Exception:
                pass

        position = getattr(pose, "position", None) if pose is not None else None
        if position is not None and hasattr(position, "x") and hasattr(position, "y"):
            try:
                return float(getattr(position, "x")), float(getattr(position, "y"))
            except Exception:
                pass

        if pose is not None:
            try:
                return float(pose[0]), float(pose[1])
            except Exception:
                pass

        match = re.search(r"pose=\(([^)]*)\)", repr(odom))
        if match is not None:
            parts = [part.strip() for part in match.group(1).split(",")]
            if len(parts) >= 2:
                try:
                    return float(parts[0]), float(parts[1])
                except Exception:
                    return None
        return None

    @staticmethod
    def _normalize_angle_deg(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0

    def _turn_delta_deg(self, before_yaw_deg: float, after_yaw_deg: float) -> float:
        return self._normalize_angle_deg(after_yaw_deg - before_yaw_deg)

    def _turn_error_deg(
        self,
        expected_delta_deg: float,
        before_yaw_deg: float,
        current_yaw_deg: float,
    ) -> float:
        actual_delta_deg = self._turn_delta_deg(before_yaw_deg, current_yaw_deg)
        return self._normalize_angle_deg(actual_delta_deg - expected_delta_deg)

    def _turn_velocity_for_error(self, error_deg: float) -> float:
        """
        根据剩余 yaw 误差分段降速。

        误差大时使用现场验证过的 0.5236rad/s；接近目标时降到补角速度，
        避免靠固定 3 秒开环撞运气。
        """
        abs_error_deg = abs(error_deg)
        if abs_error_deg > 35.0:
            return self.config.turn_angular_speed_radps
        if abs_error_deg > 12.0:
            return min(self.config.turn_angular_speed_radps, 0.30)
        if abs_error_deg > max(self.config.turn_yaw_tolerance_deg * 2.0, 3.0):
            return min(self.config.turn_angular_speed_radps, 0.16)
        return min(
            self.config.turn_angular_speed_radps,
            self.config.turn_correction_angular_speed_radps,
        )

    def _read_turn_yaw_from_slam_quiet(self, slam) -> float | None:
        try:
            odom = slam.get_odom_info()
        except Exception:
            return None
        return self._extract_yaw_deg_from_odom(odom)

    def _drive_chassis_yaw_closed_loop(
        self,
        pnc,
        slam,
        robot,
        expected_delta_deg: float,
        before_yaw_deg: float,
        max_duration_s: float,
    ) -> float:
        import agibot_gdk

        def make_twist(wz_radps: float):
            twist = agibot_gdk.Twist()
            twist.linear = agibot_gdk.Vector3()
            twist.angular = agibot_gdk.Vector3()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(wz_radps)
            return twist

        stop = make_twist(0.0)
        interval_s = 1.0 / self.config.turn_hz
        start = time.time()
        deadline = start + max_duration_s
        command_count = 0
        stable_count = 0
        yaw_miss_count = 0
        best_abs_error_deg = float("inf")
        last_progress_s = start
        last_log_s = 0.0
        last_safety_check_s = 0.0
        last_wz_radps = 0.0
        final_yaw_deg = before_yaw_deg
        no_progress_timeout_s = 2.0

        try:
            while time.time() < deadline:
                now = time.time()
                if now - last_safety_check_s >= 0.8:
                    self._check_turn_motion_safety(robot)
                    last_safety_check_s = now

                current_yaw_deg = self._read_turn_yaw_from_slam_quiet(slam)
                if current_yaw_deg is None:
                    yaw_miss_count += 1
                    pnc.move_chassis(stop)
                    if yaw_miss_count >= 5:
                        raise RuntimeError(
                            "velocity turn lost yaw feedback during closed-loop control"
                        )
                    time.sleep(interval_s)
                    continue

                yaw_miss_count = 0
                final_yaw_deg = current_yaw_deg
                error_deg = self._turn_error_deg(
                    expected_delta_deg,
                    before_yaw_deg,
                    current_yaw_deg,
                )
                abs_error_deg = abs(error_deg)

                if abs_error_deg < best_abs_error_deg - 0.8:
                    best_abs_error_deg = abs_error_deg
                    last_progress_s = now
                elif now - start > 1.0 and now - last_progress_s > no_progress_timeout_s:
                    raise RuntimeError(
                        "velocity turn yaw is not converging: "
                        f"best_abs_error_deg={best_abs_error_deg:.3f}, "
                        f"current_error_deg={error_deg:.3f}, "
                        f"no_progress_s={now - last_progress_s:.2f}"
                    )

                if abs_error_deg <= self.config.turn_yaw_tolerance_deg:
                    stable_count += 1
                    pnc.move_chassis(stop)
                    if stable_count >= 3:
                        break
                    time.sleep(max(interval_s, 0.08))
                    continue

                stable_count = 0
                correction_delta_deg = -error_deg
                odom_direction = 1.0 if correction_delta_deg > 0.0 else -1.0
                speed = max(0.06, self._turn_velocity_for_error(error_deg))
                # On this chassis, move_chassis angular.z is opposite to SLAM odom yaw.
                wz_radps = -odom_direction * speed
                pnc.move_chassis(make_twist(wz_radps))
                command_count += 1

                if now - last_log_s >= 0.5 or abs(wz_radps - last_wz_radps) > 1e-6:
                    actual_delta_deg = self._turn_delta_deg(before_yaw_deg, current_yaw_deg)
                    self.log(
                        "velocity_turn_loop "
                        f"elapsed_s={now - start:.2f} "
                        f"yaw_deg={current_yaw_deg:.3f} "
                        f"actual_delta_deg={actual_delta_deg:.3f} "
                        f"target_delta_deg={expected_delta_deg:.3f} "
                        f"error_deg={error_deg:.3f} "
                        f"wz_radps={wz_radps:.3f} "
                        f"best_abs_error_deg={best_abs_error_deg:.3f}"
                    )
                    self.emit_event(
                        "velocity_turn_loop",
                        step_no=self.step_no,
                        rod_index=self.current_rod_index,
                        elapsed_s=round(now - start, 3),
                        yaw_deg=round(current_yaw_deg, 4),
                        actual_delta_deg=round(actual_delta_deg, 4),
                        target_delta_deg=round(expected_delta_deg, 4),
                        error_deg=round(error_deg, 4),
                        wz_radps=round(wz_radps, 4),
                    )
                    last_log_s = now
                    last_wz_radps = wz_radps

                time.sleep(interval_s)
            else:
                raise RuntimeError(
                    "velocity turn timed out before yaw reached tolerance: "
                    f"timeout_s={max_duration_s:.2f}, "
                    f"final_yaw_deg={final_yaw_deg:.3f}, "
                    f"final_error_deg={self._turn_error_deg(expected_delta_deg, before_yaw_deg, final_yaw_deg):.3f}"
                )
        finally:
            for _ in range(12):
                try:
                    pnc.move_chassis(stop)
                except Exception:
                    pass
                time.sleep(0.03)

        elapsed_s = time.time() - start
        final_error_deg = self._turn_error_deg(expected_delta_deg, before_yaw_deg, final_yaw_deg)
        self.log(
            "velocity_turn_closed_loop_done "
            f"commands={command_count} elapsed_s={elapsed_s:.2f} "
            f"final_yaw_deg={final_yaw_deg:.3f} "
            f"final_error_deg={final_error_deg:.3f} "
            f"tolerance_deg={self.config.turn_yaw_tolerance_deg:.3f}"
        )
        self.emit_event(
            "velocity_turn_closed_loop_done",
            step_no=self.step_no,
            rod_index=self.current_rod_index,
            commands=command_count,
            elapsed_s=round(elapsed_s, 3),
            final_yaw_deg=round(final_yaw_deg, 4),
            final_error_deg=round(final_error_deg, 4),
            tolerance_deg=self.config.turn_yaw_tolerance_deg,
        )
        return final_yaw_deg

    def _send_turn_velocity_existing_pnc(self, pnc, wz_radps: float, duration_s: float, hz: float):
        import agibot_gdk

        twist = agibot_gdk.Twist()
        twist.linear = agibot_gdk.Vector3()
        twist.angular = agibot_gdk.Vector3()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(wz_radps)

        stop = agibot_gdk.Twist()
        stop.linear = agibot_gdk.Vector3()
        stop.angular = agibot_gdk.Vector3()
        stop.linear.x = stop.linear.y = stop.linear.z = 0.0
        stop.angular.x = stop.angular.y = stop.angular.z = 0.0

        interval_s = 1.0 / hz
        command_count = max(1, math.ceil(duration_s * hz))
        try:
            for _ in range(command_count):
                pnc.move_chassis(twist)
                time.sleep(interval_s)
        finally:
            for _ in range(10):
                try:
                    pnc.move_chassis(stop)
                except Exception:
                    pass
                time.sleep(0.03)

    def _confirm_turn_yaw_after_settle(
        self,
        slam,
        expected_delta_deg: float,
        before_yaw_deg: float | None,
        label: str,
    ) -> float:
        if before_yaw_deg is None:
            raise RuntimeError(f"{label} cannot confirm yaw: before_yaw_deg is None")

        time.sleep(max(0.25, self.config.settle_s))
        samples: list[float] = []
        for sample_index in range(1, self.config.turn_confirm_samples + 1):
            yaw_deg = self._read_turn_yaw_from_slam_quiet(slam)
            if yaw_deg is not None:
                samples.append(yaw_deg)
            else:
                self.log(f"{label}_yaw_confirm_sample_missing index={sample_index}")
            if sample_index < self.config.turn_confirm_samples:
                time.sleep(self.config.turn_confirm_interval_s)

        if not samples:
            raise RuntimeError(f"{label} cannot confirm yaw: no valid yaw samples")

        sorted_samples = sorted(samples)
        median_yaw_deg = sorted_samples[len(sorted_samples) // 2]
        sample_offsets = [
            self._normalize_angle_deg(sample - median_yaw_deg) for sample in samples
        ]
        span_deg = max(sample_offsets) - min(sample_offsets)
        actual_delta_deg = self._turn_delta_deg(before_yaw_deg, median_yaw_deg)
        error_deg = self._normalize_angle_deg(actual_delta_deg - expected_delta_deg)
        rounded_samples = tuple(round(sample, 3) for sample in samples)
        self.log(
            f"{label}_yaw_confirm "
            f"samples={rounded_samples} median_yaw_deg={median_yaw_deg:.3f} "
            f"span_deg={span_deg:.3f} expected_delta_deg={expected_delta_deg:.3f} "
            f"actual_delta_deg={actual_delta_deg:.3f} error_deg={error_deg:.3f} "
            f"tolerance_deg={self.config.turn_yaw_tolerance_deg:.3f}"
        )
        self.emit_event(
            "turn_yaw_confirm",
            step_no=self.step_no,
            rod_index=self.current_rod_index,
            label=label,
            samples=[round(sample, 4) for sample in samples],
            median_yaw_deg=round(median_yaw_deg, 4),
            span_deg=round(span_deg, 4),
            expected_delta_deg=round(expected_delta_deg, 4),
            actual_delta_deg=round(actual_delta_deg, 4),
            error_deg=round(error_deg, 4),
            tolerance_deg=self.config.turn_yaw_tolerance_deg,
        )
        if span_deg > self.config.turn_confirm_max_span_deg:
            raise RuntimeError(
                f"{label} yaw confirmation is unstable: span_deg={span_deg:.3f}, "
                f"max_span_deg={self.config.turn_confirm_max_span_deg:.3f}, "
                f"samples={rounded_samples}"
            )
        return median_yaw_deg

    def _correct_turn_yaw_if_needed(
        self,
        pnc,
        slam,
        expected_delta_deg: float,
        before_yaw_deg: float | None,
        current_yaw_deg: float | None,
        label: str,
        robot=None,
    ) -> float | None:
        if before_yaw_deg is None or current_yaw_deg is None:
            return current_yaw_deg
        if not self.config.turn_correction_enabled:
            return current_yaw_deg

        error_deg = self._turn_error_deg(expected_delta_deg, before_yaw_deg, current_yaw_deg)
        if abs(error_deg) <= self.config.turn_yaw_tolerance_deg:
            self.log(
                f"{label}_yaw_correction status=not_needed "
                f"error_deg={error_deg:.3f} tolerance_deg={self.config.turn_yaw_tolerance_deg:.3f}"
            )
            return current_yaw_deg

        if abs(error_deg) > self.config.turn_correction_max_error_deg:
            raise RuntimeError(
                f"{label} yaw error too large for closed-loop correction: "
                f"error_deg={error_deg:.3f}, max_correctable_deg={self.config.turn_correction_max_error_deg:.3f}"
            )
        if self.config.turn_correction_max_passes <= 0:
            raise RuntimeError(
                f"{label} yaw correction needed but turn_correction_max_passes="
                f"{self.config.turn_correction_max_passes}"
            )

        if robot is not None:
            self._check_turn_motion_safety(robot)
        self._request_turn_chassis_control(pnc, mode=self.config.turn_control_mode)
        corrected_yaw_deg = current_yaw_deg
        for correction_pass in range(1, self.config.turn_correction_max_passes + 1):
            if robot is not None:
                self._check_turn_motion_safety(robot)
            correction_delta_deg = -error_deg
            odom_direction = 1.0 if correction_delta_deg > 0.0 else -1.0
            speed = self.config.turn_correction_angular_speed_radps
            raw_duration_s = abs(math.radians(correction_delta_deg)) / speed
            duration_s = min(1.2, max(0.12, raw_duration_s * 0.75))
            wz = -odom_direction * speed
            self.log(
                f"{label}_yaw_correction_start pass={correction_pass} "
                f"error_deg={error_deg:.3f} correction_delta_deg={correction_delta_deg:.3f} "
                f"wz_radps={wz:.3f} duration_s={duration_s:.3f}"
            )
            self.emit_event(
                "turn_yaw_correction_start",
                step_no=self.step_no,
                rod_index=self.current_rod_index,
                label=label,
                correction_pass=correction_pass,
                error_deg=round(error_deg, 4),
                correction_delta_deg=round(correction_delta_deg, 4),
                wz_radps=round(wz, 4),
                duration_s=round(duration_s, 4),
            )
            self._send_turn_velocity_existing_pnc(
                pnc=pnc,
                wz_radps=wz,
                duration_s=duration_s,
                hz=self.config.turn_hz,
            )
            time.sleep(max(0.25, self.config.settle_s))
            corrected_yaw_deg = self._read_turn_yaw_from_slam(
                slam,
                f"{label}_after_correction_{correction_pass}",
            )
            if corrected_yaw_deg is None:
                raise RuntimeError(f"{label} yaw unavailable after correction pass {correction_pass}")
            error_deg = self._turn_error_deg(expected_delta_deg, before_yaw_deg, corrected_yaw_deg)
            self.log(
                f"{label}_yaw_correction_result pass={correction_pass} "
                f"yaw_deg={corrected_yaw_deg:.3f} error_deg={error_deg:.3f} "
                f"tolerance_deg={self.config.turn_yaw_tolerance_deg:.3f}"
            )
            self.emit_event(
                "turn_yaw_correction_result",
                step_no=self.step_no,
                rod_index=self.current_rod_index,
                label=label,
                correction_pass=correction_pass,
                yaw_deg=round(corrected_yaw_deg, 4),
                error_deg=round(error_deg, 4),
                tolerance_deg=self.config.turn_yaw_tolerance_deg,
            )
            if abs(error_deg) <= self.config.turn_yaw_tolerance_deg:
                return corrected_yaw_deg

        raise RuntimeError(
            f"{label} yaw correction failed after {self.config.turn_correction_max_passes} passes: "
            f"last_error_deg={error_deg:.3f}, tolerance_deg={self.config.turn_yaw_tolerance_deg:.3f}"
        )

    def _validate_turn_yaw_delta(
        self,
        label: str,
        expected_delta_deg: float,
        before_yaw_deg: float | None,
        after_yaw_deg: float | None,
    ):
        if before_yaw_deg is None or after_yaw_deg is None:
            raise RuntimeError(
                f"{label} cannot validate yaw delta: before={before_yaw_deg}, "
                f"after={after_yaw_deg}. Stop here; do not continue the "
                "industrial sequence without yaw feedback."
            )

        actual_delta_deg = self._turn_delta_deg(before_yaw_deg, after_yaw_deg)
        error_deg = self._normalize_angle_deg(actual_delta_deg - expected_delta_deg)
        abs_error_deg = abs(error_deg)
        self.log(
            f"{label}_yaw_validation "
            f"expected_delta_deg={expected_delta_deg:.3f} "
            f"actual_delta_deg={actual_delta_deg:.3f} "
            f"error_deg={error_deg:.3f} "
            f"tolerance_deg={self.config.turn_yaw_tolerance_deg:.3f}"
        )
        if abs_error_deg > self.config.turn_yaw_tolerance_deg:
            raise RuntimeError(
                f"{label} yaw error too large: expected_delta_deg={expected_delta_deg:.3f}, "
                f"actual_delta_deg={actual_delta_deg:.3f}, error_deg={error_deg:.3f}, "
                f"tolerance_deg={self.config.turn_yaw_tolerance_deg:.3f}. "
                "Stop here; do not continue until 90 degree turn is calibrated."
            )

    def _validate_velocity_turn_observed(self, angle_deg: float, before, after):
        """
        开环转向后的最低限度观测校验。

        这不能证明角度精确等于 90 度；真正工业级还需要 yaw/odom/IMU
        闭环。但它可以拦住“速度发了、车基本没转”的最危险假成功。
        """
        if before is None or after is None:
            raise RuntimeError(
                "velocity turn cannot be validated: ultrasonic snapshot unavailable; "
                "do not continue without onsite 90 degree confirmation"
            )

        deltas = []
        for name in ("front", "right", "rear", "left"):
            before_min = before[name][0]
            after_min = after[name][0]
            if before_min is None or after_min is None:
                continue
            deltas.append((name, abs(after_min - before_min), before_min, after_min))

        if not deltas:
            raise RuntimeError(
                "velocity turn cannot be validated: no comparable ultrasonic groups"
            )

        max_name, max_delta_mm, before_mm, after_mm = max(deltas, key=lambda item: item[1])
        self.log(
            "velocity_turn_sensor_validation "
            f"angle_deg={angle_deg} max_delta_group={max_name} "
            f"max_delta_mm={max_delta_mm} before_mm={before_mm} after_mm={after_mm} "
            f"threshold_mm={self.config.turn_min_sensor_delta_mm} "
            "yaw_feedback=odom"
        )
        if max_delta_mm < self.config.turn_min_sensor_delta_mm:
            self.log(
                "velocity_turn_sensor_validation_soft_warning "
                f"max_delta_mm={max_delta_mm} "
                f"threshold_mm={self.config.turn_min_sensor_delta_mm} "
                "reason=odom_yaw_validation_already_passed"
            )

    def _turn_duration_for_angle(self, angle_deg: float) -> float:
        """根据方向选择现场标定过的 90 度转向时长，并按角度等比例缩放。"""
        ratio = abs(angle_deg) / 90.0
        if angle_deg > 0:
            return self.config.left_turn_duration_s * ratio
        return self.config.right_turn_duration_s * ratio

    def _drive_chassis_velocity(
        self,
        vx_mps: float,
        wz_radps: float,
        duration_s: float,
        hz: float,
    ):
        """
        直接发送底盘速度，并在 finally 中强制停车释放。

        这个函数只用于总控里的原地转向。前进精停和后退仍然使用
        RackIndustrialDockingController，避免扩大改动面。
        """
        if duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        if hz <= 0.0:
            raise ValueError("hz must be positive")

        rack_package_dir = str(self.config.base_dir / "rack_hybrid_docking_package")
        if rack_package_dir not in sys.path:
            sys.path.insert(0, rack_package_dir)

        import agibot_gdk

        def make_twist(vx: float, wz: float):
            twist = agibot_gdk.Twist()
            twist.linear = agibot_gdk.Vector3()
            twist.angular = agibot_gdk.Vector3()
            twist.linear.x = float(vx)
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(wz)
            return twist

        gdk_inited = False
        pnc = None
        try:
            result = agibot_gdk.gdk_init()
            gdk_res = getattr(agibot_gdk, "GDKRes", None)
            if gdk_res is not None and result not in (None, gdk_res.kSuccess):
                raise RuntimeError(f"GDK init failed before turn: {result}")
            gdk_inited = True

            robot = agibot_gdk.Robot()
            pnc = agibot_gdk.Pnc()
            time.sleep(0.5)
            self._check_turn_motion_safety(robot)
            self._cancel_chassis_task_if_blocking(pnc)
            self._request_turn_chassis_control(pnc, mode=self.config.turn_control_mode)
            time.sleep(0.2)

            twist = make_twist(vx_mps, wz_radps)
            interval_s = 1.0 / hz
            command_count = max(1, math.ceil(duration_s * hz))
            start = time.time()
            for _ in range(command_count):
                pnc.move_chassis(twist)
                time.sleep(interval_s)

            elapsed_s = time.time() - start
            self.log(
                f"turn_velocity_sent commands={command_count} "
                f"elapsed_s={elapsed_s:.2f}"
            )
        finally:
            if pnc is not None:
                stop = make_twist(0.0, 0.0)
                for _ in range(12):
                    try:
                        pnc.move_chassis(stop)
                    except Exception:
                        pass
                    time.sleep(0.03)
            if gdk_inited:
                try:
                    agibot_gdk.gdk_release()
                except Exception:
                    pass

    def _request_turn_chassis_control(
        self,
        pnc,
        mode: int,
        retries: int = 3,
        wait_s: float = 0.8,
    ):
        """转向前申请底盘远控；现场偶发 request timeout，允许有限重试。"""
        last_exc = None
        for attempt in range(1, retries + 1):
            try:
                result = pnc.request_chassis_control(mode)
                self.log(
                    f"turn_request_chassis_control attempt={attempt} "
                    f"mode={mode} result={result}"
                )
                return result
            except Exception as exc:
                last_exc = exc
                self.log(f"turn_request_chassis_control_failed attempt={attempt} error={exc}")
                try:
                    self._cancel_chassis_task_if_blocking(pnc)
                except Exception as cancel_exc:
                    self.log(f"turn_request_cancel_after_failure_failed={cancel_exc}")
                time.sleep(wait_s)
        raise RuntimeError(f"RequestChassisControl failed after retries: {last_exc}")

    def _cancel_chassis_task_if_blocking(self, pnc):
        """清理可能占用底盘控制权的旧 PNC 任务。"""
        try:
            task = pnc.get_task_state()
        except Exception as exc:
            self.log(f"turn_task_state_read_failed={exc}")
            return

        task_state = getattr(task, "state", None)
        task_id = getattr(task, "id", None)
        self.log(f"turn_task_state_before state={task_state} id={task_id}")
        if task_id is None:
            return
        if task_state in (0, 3, 6, 7, 8, 9):
            return

        try:
            pnc.cancel_task(task_id)
            self.log(f"turn_cancel_task id={task_id}")
            time.sleep(0.3)
        except Exception as exc:
            self.log(f"turn_cancel_task_failed id={task_id} error={exc}")

    def run_grab_above_script(self, rod_index: int, rod_script: str):
        title = f"第{rod_index}根：移动到抓取上方"
        if self.config.grab_vertical_stack_pitch_m is None:
            self.run_python_script(rod_script, title)
            return

        self.run_python_script(
            "move_arm_vertical_stack_grab_above.py",
            title,
            extra_args=[
                "--rod-index",
                str(rod_index),
                "--pitch-m",
                f"{self.config.grab_vertical_stack_pitch_m:.6f}",
            ],
        )

    def run_one_rod(self, rod_index: int, rod_script: str):
        """
        执行单根料完整流程。

        单根料顺序严格按用户要求：
          1. 张开夹爪；
          2. 移到对应第 N 根上方；
          3. 前雷达靠近到抓料目标距离；
          4. 闭合夹爪；
          5. 拉出；
          6. 后退 1m；
          7. 右转 90 度；
          8. 移到放置上方；
          9. 前雷达靠近到放料目标距离；
         10. 下移；
         11. 张开夹爪；
         12. 拉出；
         13. 后退 1m；
         14. 左转 90 度。
        """
        self.log("=" * 80)
        self.start_rod(rod_index, "full", script=rod_script)
        self.log(f"START ROD {rod_index}: script={rod_script}")

        self.run_python_script("move_ee_pose_open_2.py", f"第{rod_index}根：张开夹爪")
        self.run_grab_above_script(rod_index, rod_script)
        self.approach_by_front_ultrasonic(
            self.config.grab_distance_mm,
            self.config.grab_approach_speed_mps,
            f"第{rod_index}根：前雷达靠近到 {self.config.grab_distance_mm}mm",
            self.config.grab_brake_margin_mm,
            self.config.grab_min_safe_mm,
            target_tolerance_mm=self.config.grab_target_tolerance_mm,
            correction_speed_mps=self.config.grab_correction_speed_mps,
            correction_max_passes=self.config.grab_correction_max_passes,
            angle_correction_max_span_mm=self.config.grab_angle_correction_max_span_mm,
            angle_correction_max_passes=self.config.grab_angle_correction_max_passes,
            angle_correction_angular_speed_radps=self.config.grab_angle_correction_angular_speed_radps,
            angle_correction_probe_s=self.config.grab_angle_correction_probe_s,
            target_avg_accept_span_mm=self.config.grab_target_avg_accept_span_mm,
        )
        self.run_python_script("move_ee_pose_close_2.py", f"第{rod_index}根：闭合夹爪")
        self.run_python_script("offset_move_pull.py", f"第{rod_index}根：拉出")
        self.retreat_by_industrial_rack(f"第{rod_index}根：后退 {self.config.retreat_distance_m}m")
        self.rotate_chassis(-90.0, f"第{rod_index}根：向右转 90 度")
        self.run_python_script("move_arm_by_json_grab_above_2.py", f"第{rod_index}根：移动到放置上方")
        self.approach_by_front_ultrasonic(
            self.config.place_distance_mm,
            self.config.place_approach_speed_mps,
            f"第{rod_index}根：前雷达靠近到 {self.config.place_distance_mm}mm",
            self.config.place_brake_margin_mm,
            self.config.place_min_safe_mm,
            target_tolerance_mm=self.config.place_target_tolerance_mm,
            correction_speed_mps=self.config.place_correction_speed_mps,
            correction_max_passes=self.config.place_correction_max_passes,
        )
        self.run_python_script("offset_move_down.py", f"第{rod_index}根：下移")
        self.run_python_script("move_ee_pose_open_2.py", f"第{rod_index}根：张开夹爪放料")
        self.run_python_script("offset_move_pull.py", f"第{rod_index}根：放料后拉出")
        self.retreat_by_industrial_rack(f"第{rod_index}根：放料后后退 {self.config.retreat_distance_m}m")
        self.rotate_chassis(90.0, f"第{rod_index}根：向左转 90 度")

        self.log(f"END ROD {rod_index}: success")
        self.finish_rod(rod_index, "full")

    def run_one_rod_after_grab_pull(self, rod_index: int):
        """
        从“第 N 根已经抓住并拉出，准备后退去放料”恢复。

        当前现场失败点就是第 1 根 STEP 006 之前：
          - 开爪完成；
          - 到第 1 根上方完成；
          - 前雷达到抓料目标距离完成；
          - 闭爪完成；
          - 拉出完成；
          - 后退前被 motion_control_error=2 拦住。
        """
        self.log("=" * 80)
        self.start_rod(rod_index, "resume_after_grab_pull")
        self.log(f"RESUME ROD {rod_index}: after grab pull")

        self.retreat_by_industrial_rack(f"第{rod_index}根：后退 {self.config.retreat_distance_m}m")
        self.rotate_chassis(-90.0, f"第{rod_index}根：向右转 90 度")
        self.run_python_script("move_arm_by_json_grab_above_2.py", f"第{rod_index}根：移动到放置上方")
        self.approach_by_front_ultrasonic(
            self.config.place_distance_mm,
            self.config.place_approach_speed_mps,
            f"第{rod_index}根：前雷达靠近到 {self.config.place_distance_mm}mm",
            self.config.place_brake_margin_mm,
            self.config.place_min_safe_mm,
            target_tolerance_mm=self.config.place_target_tolerance_mm,
            correction_speed_mps=self.config.place_correction_speed_mps,
            correction_max_passes=self.config.place_correction_max_passes,
        )
        self.run_python_script("offset_move_down.py", f"第{rod_index}根：下移")
        self.run_python_script("move_ee_pose_open_2.py", f"第{rod_index}根：张开夹爪放料")
        self.run_python_script("offset_move_pull.py", f"第{rod_index}根：放料后拉出")
        self.retreat_by_industrial_rack(f"第{rod_index}根：放料后后退 {self.config.retreat_distance_m}m")
        self.rotate_chassis(90.0, f"第{rod_index}根：向左转 90 度")

        self.log(f"END ROD {rod_index}: resumed success")
        self.finish_rod(rod_index, "resume_after_grab_pull")

    def run_one_rod_after_grab_retreat(self, rod_index: int):
        """从“第 N 根抓取后已经后退，准备右转去放料”恢复。"""
        self.log("=" * 80)
        self.start_rod(rod_index, "resume_after_grab_retreat")
        self.log(f"RESUME ROD {rod_index}: after grab retreat")

        self.rotate_chassis(-90.0, f"第{rod_index}根：向右转 90 度")
        self.run_python_script("move_arm_by_json_grab_above_2.py", f"第{rod_index}根：移动到放置上方")
        self.approach_by_front_ultrasonic(
            self.config.place_distance_mm,
            self.config.place_approach_speed_mps,
            f"第{rod_index}根：前雷达靠近到 {self.config.place_distance_mm}mm",
            self.config.place_brake_margin_mm,
            self.config.place_min_safe_mm,
            target_tolerance_mm=self.config.place_target_tolerance_mm,
            correction_speed_mps=self.config.place_correction_speed_mps,
            correction_max_passes=self.config.place_correction_max_passes,
        )
        self.run_python_script("offset_move_down.py", f"第{rod_index}根：下移")
        self.run_python_script("move_ee_pose_open_2.py", f"第{rod_index}根：张开夹爪放料")
        self.run_python_script("offset_move_pull.py", f"第{rod_index}根：放料后拉出")
        self.retreat_by_industrial_rack(f"第{rod_index}根：放料后后退 {self.config.retreat_distance_m}m")
        self.rotate_chassis(90.0, f"第{rod_index}根：向左转 90 度")

        self.log(f"END ROD {rod_index}: resumed after retreat success")
        self.finish_rod(rod_index, "resume_after_grab_retreat")

    def run_one_rod_after_place_above(self, rod_index: int):
        """从“第 N 根已经到放料上方，准备前雷达到放料目标距离”恢复。"""
        self.log("=" * 80)
        self.start_rod(rod_index, "resume_after_place_above")
        self.log(f"RESUME ROD {rod_index}: after place above")

        self.approach_by_front_ultrasonic(
            self.config.place_distance_mm,
            self.config.place_approach_speed_mps,
            f"第{rod_index}根：前雷达靠近到 {self.config.place_distance_mm}mm",
            self.config.place_brake_margin_mm,
            self.config.place_min_safe_mm,
            target_tolerance_mm=self.config.place_target_tolerance_mm,
            correction_speed_mps=self.config.place_correction_speed_mps,
            correction_max_passes=self.config.place_correction_max_passes,
        )
        self.run_python_script("offset_move_down.py", f"第{rod_index}根：下移")
        self.run_python_script("move_ee_pose_open_2.py", f"第{rod_index}根：张开夹爪放料")
        self.run_python_script("offset_move_pull.py", f"第{rod_index}根：放料后拉出")
        self.retreat_by_industrial_rack(f"第{rod_index}根：放料后后退 {self.config.retreat_distance_m}m")
        self.rotate_chassis(90.0, f"第{rod_index}根：向左转 90 度")

        self.log(f"END ROD {rod_index}: resumed after place above success")
        self.finish_rod(rod_index, "resume_after_place_above")

    def run_one_rod_after_place_pull(self, rod_index: int):
        """从“第 N 根已经放料、开夹、拉出，准备放料后后退”恢复。"""
        self.log("=" * 80)
        self.start_rod(rod_index, "resume_after_place_pull")
        self.log(f"RESUME ROD {rod_index}: after place pull")

        self.retreat_by_industrial_rack(f"第{rod_index}根：放料后后退 {self.config.retreat_distance_m}m")
        self.rotate_chassis(90.0, f"第{rod_index}根：向左转 90 度")

        self.log(f"END ROD {rod_index}: resumed after place pull success")
        self.finish_rod(rod_index, "resume_after_place_pull")

    def run_one_rod_after_place_retreat_target(self, rod_index: int):
        """从“第 N 根放料后退中断/过退，需恢复到后退目标距离后左转”恢复。"""
        self.log("=" * 80)
        self.start_rod(rod_index, "resume_after_place_retreat_target")
        self.log(
            f"RESUME ROD {rod_index}: recover place retreat front target before left turn"
        )

        self.correct_place_retreat_to_front_target(rod_index)
        self.rotate_chassis(90.0, f"第{rod_index}根：向左转 90 度")

        self.log(f"END ROD {rod_index}: resumed after place retreat target success")
        self.finish_rod(rod_index, "resume_after_place_retreat_target")

    def run(self):
        self.check_required_files()
        self.require_live_allowed()
        self.check_live_startup_safety()

        self.log(
            "controller_start "
            f"base_dir={self.config.base_dir} "
            f"dry_run={self.config.dry_run} "
            f"rod_range={self.config.start_index}-{self.config.end_index}"
        )
        self.emit_event("controller_start", config=self.config)
        self.write_checkpoint(status="controller_started")

        first_full_rod = self.config.start_index
        resume_count = sum(
            value is not None
            for value in (
                self.config.resume_after_grab_pull_index,
                self.config.resume_after_grab_retreat_index,
                self.config.resume_after_place_above_index,
                self.config.resume_after_place_pull_index,
                self.config.resume_after_place_retreat_target_index,
            )
        )
        if resume_count > 1:
            raise RuntimeError(
                "choose only one resume option: "
                "--resume-after-grab-pull-index, "
                "--resume-after-grab-retreat-index, or "
                "--resume-after-place-above-index, or "
                "--resume-after-place-pull-index, or "
                "--resume-after-place-retreat-target-index"
            )

        resume_indices = (
            self.config.resume_after_grab_pull_index,
            self.config.resume_after_grab_retreat_index,
            self.config.resume_after_place_above_index,
            self.config.resume_after_place_pull_index,
            self.config.resume_after_place_retreat_target_index,
        )
        resume_index = next((value for value in resume_indices if value is not None), None)
        first_context_rod = self.config.start_index if resume_index is None else resume_index
        if (
            not self.config.dry_run
            and not self.config.turn_validation_ok
            and self.config.end_index > first_context_rod
        ):
            raise RuntimeError(
                "multi-rod live run blocked: run industrial_turn_diagnostic.py for "
                "right and left turns repeatedly first, then pass --turn-validation-ok. "
                f"requested_context={first_context_rod}-{self.config.end_index}"
            )

        if self.config.resume_after_grab_pull_index is not None:
            resume_index = self.config.resume_after_grab_pull_index
            self.run_one_rod_after_grab_pull(resume_index)
            first_full_rod = resume_index + 1
        elif self.config.resume_after_grab_retreat_index is not None:
            resume_index = self.config.resume_after_grab_retreat_index
            self.run_one_rod_after_grab_retreat(resume_index)
            first_full_rod = resume_index + 1
        elif self.config.resume_after_place_above_index is not None:
            resume_index = self.config.resume_after_place_above_index
            self.run_one_rod_after_place_above(resume_index)
            first_full_rod = resume_index + 1
        elif self.config.resume_after_place_pull_index is not None:
            resume_index = self.config.resume_after_place_pull_index
            self.run_one_rod_after_place_pull(resume_index)
            first_full_rod = resume_index + 1
        elif self.config.resume_after_place_retreat_target_index is not None:
            resume_index = self.config.resume_after_place_retreat_target_index
            self.run_one_rod_after_place_retreat_target(resume_index)
            first_full_rod = resume_index + 1

        for rod_index in range(first_full_rod, self.config.end_index + 1):
            rod_script = ROD_SCRIPT_NAMES[rod_index - 1]
            self.run_one_rod(rod_index, rod_script)

        self.cleanup_final_pnc_task()
        self.record_final_status_snapshot()
        self.log("controller_finished: all requested rods completed")
        self.emit_event("controller_finished", status="completed")
        self.write_checkpoint(status="completed")

    def cleanup_final_pnc_task(self):
        """Cancel a stale PNC task after a successful requested run."""
        if self.config.dry_run:
            self.log("final_pnc_cleanup skipped dry_run=True")
            return

        try:
            import agibot_gdk
        except Exception as exc:
            self.log(f"final_pnc_cleanup import_failed={exc}")
            self.emit_event("final_pnc_cleanup", status="failed", error=str(exc))
            return

        gdk_inited = False
        try:
            result = agibot_gdk.gdk_init()
            gdk_res = getattr(agibot_gdk, "GDKRes", None)
            if gdk_res is not None and result not in (None, gdk_res.kSuccess):
                raise RuntimeError(f"GDK init failed: {result}")
            gdk_inited = True

            pnc = agibot_gdk.Pnc()
            time.sleep(0.8)
            task = self._read_pnc_task_with_retry(pnc, "final_pnc_task_before")
            state = getattr(task, "state", None)
            task_id = getattr(task, "id", None)
            task_type = getattr(task, "type", None)
            self.log(f"final_pnc_task_before state={state} id={task_id} type={task_type}")

            if task_id is None:
                self.emit_event(
                    "final_pnc_cleanup",
                    status="skipped",
                    reason="no_task_id",
                    state=state,
                    task_id=task_id,
                    task_type=task_type,
                )
                return
            if state in (0, 3, 6, 7, 8, 9):
                self.emit_event(
                    "final_pnc_cleanup",
                    status="not_needed",
                    state=state,
                    task_id=task_id,
                    task_type=task_type,
                )
                return

            try:
                pnc.cancel_task(task_id)
                cleanup_status = "canceled"
                cleanup_error = None
                self.log(f"final_pnc_cancel_task id={task_id} state={state}")
            except RuntimeError as exc:
                if "Task is not in RUNNING or PAUSED state" not in str(exc):
                    raise
                cleanup_status = "ignored"
                cleanup_error = str(exc)
                self.log(f"final_pnc_cancel_task_ignored id={task_id} error={exc}")

            time.sleep(0.5)
            after = self._read_pnc_task_with_retry(pnc, "final_pnc_task_after")
            after_state = getattr(after, "state", None)
            after_id = getattr(after, "id", None)
            after_type = getattr(after, "type", None)
            self.log(f"final_pnc_task_after state={after_state} id={after_id} type={after_type}")
            self.emit_event(
                "final_pnc_cleanup",
                status=cleanup_status,
                state=state,
                task_id=task_id,
                task_type=task_type,
                after_state=after_state,
                after_task_id=after_id,
                after_task_type=after_type,
                error=cleanup_error,
            )
        except Exception as exc:
            self.log(f"final_pnc_cleanup_failed={type(exc).__name__}: {exc}")
            self.emit_event(
                "final_pnc_cleanup",
                status="failed",
                error=f"{type(exc).__name__}: {exc}",
            )
        finally:
            if gdk_inited:
                try:
                    agibot_gdk.gdk_release()
                except Exception:
                    pass

    def _read_pnc_task_with_retry(self, pnc, label: str, retries: int = 5, wait_s: float = 0.4):
        last_error = None
        for attempt in range(1, retries + 1):
            try:
                task = pnc.get_task_state()
                state = getattr(task, "state", None)
                task_id = getattr(task, "id", None)
                task_type = getattr(task, "type", None)
                self.log(f"{label} attempt={attempt} state={state} id={task_id} type={task_type}")
                return task
            except Exception as exc:
                last_error = exc
                self.log(f"{label}_read_failed attempt={attempt} error={exc}")
                if attempt < retries:
                    time.sleep(wait_s)
        raise RuntimeError(f"{label} failed after {retries} attempts: {last_error}")

    def record_final_status_snapshot(self, samples: int = 3, interval_s: float = 0.3):
        """Record a read-only final robot status snapshot in the run JSONL."""
        if self.config.dry_run:
            self.log("final_status_snapshot skipped dry_run=True")
            return

        try:
            import agibot_gdk
        except Exception as exc:
            self.log(f"final_status_snapshot import_failed={exc}")
            self.emit_event("final_status_snapshot", status="failed", error=str(exc))
            return

        gdk_inited = False
        radar = None
        try:
            result = agibot_gdk.gdk_init()
            gdk_res = getattr(agibot_gdk, "GDKRes", None)
            if gdk_res is not None and result not in (None, gdk_res.kSuccess):
                raise RuntimeError(f"GDK init failed: {result}")
            gdk_inited = True

            robot = agibot_gdk.Robot()
            pnc = agibot_gdk.Pnc()
            slam = agibot_gdk.Slam()
            radar = agibot_gdk.UltrasonicRadar()
            time.sleep(0.8)

            power = robot.get_chassis_power_state()
            try:
                motion = read_motion_control_status_with_retry(robot)
                motion_error = None
            except Exception as exc:
                motion = None
                motion_error = f"{type(exc).__name__}: {exc}"
            try:
                whole_body = robot.get_whole_body_status()
                whole_body_error = None
            except Exception as exc:
                whole_body = None
                whole_body_error = f"{type(exc).__name__}: {exc}"
            try:
                task = self._read_pnc_task_with_retry(pnc, "final_status_task")
                task_error = None
            except Exception as exc:
                task = None
                task_error = f"{type(exc).__name__}: {exc}"

            ultrasonic_samples = []
            odom_samples = []
            odom_speeds = []
            for sample_index in range(1, samples + 1):
                ultrasonic = self._read_final_ultrasonic_snapshot(radar)
                odom = self._read_final_odom_snapshot(slam)
                ultrasonic_samples.append(ultrasonic)
                odom_samples.append(odom)
                if odom.get("available") and odom.get("linear_speed_mps") is not None:
                    odom_speeds.append(float(odom["linear_speed_mps"]))
                if sample_index < samples:
                    time.sleep(interval_s)

            max_linear_speed_mps = max(odom_speeds) if odom_speeds else None
            stopped = (
                max_linear_speed_mps <= 0.02
                if max_linear_speed_mps is not None
                else None
            )
            power_fields = self._simple_public_fields(power)
            motion_fields = self._simple_public_fields(motion) if motion is not None else None
            whole_body_fields = (
                self._simple_public_fields(whole_body) if whole_body is not None else None
            )
            task_fields = self._simple_public_fields(task) if task is not None else None

            self.log(
                "final_status_snapshot "
                f"charge_plug_insert_state={power_fields.get('charge_plug_insert_state')} "
                f"motion_error={None if motion_fields is None else motion_fields.get('error_code')} "
                f"task_state={None if task_fields is None else task_fields.get('state')} "
                f"task_id={None if task_fields is None else task_fields.get('id')} "
                f"odom_available={bool(odom_speeds)} "
                f"max_linear_speed_mps={max_linear_speed_mps} "
                f"stopped={stopped}"
            )
            self.emit_event(
                "final_status_snapshot",
                status="ok",
                samples=samples,
                interval_s=interval_s,
                chassis_power=power_fields,
                motion_control=motion_fields,
                motion_control_error=motion_error,
                whole_body=whole_body_fields,
                whole_body_error=whole_body_error,
                task_state=task_fields,
                task_state_error=task_error,
                ultrasonic_samples=tuple(ultrasonic_samples),
                odom_samples=tuple(odom_samples),
                odom_available=bool(odom_speeds),
                max_linear_speed_mps=max_linear_speed_mps,
                stopped=stopped,
            )
        except Exception as exc:
            self.log(f"final_status_snapshot_failed={type(exc).__name__}: {exc}")
            self.emit_event(
                "final_status_snapshot",
                status="failed",
                error=f"{type(exc).__name__}: {exc}",
            )
        finally:
            if radar is not None:
                try:
                    radar.close_ultrasonic_radar()
                except Exception:
                    pass
            if gdk_inited:
                try:
                    agibot_gdk.gdk_release()
                except Exception:
                    pass

    def _simple_public_fields(self, obj):
        if obj is None:
            return None
        fields = {}
        for name in dir(obj):
            if name.startswith("_"):
                continue
            try:
                value = getattr(obj, name)
            except Exception as exc:
                fields[name] = f"<read_error {type(exc).__name__}: {exc}>"
                continue
            if callable(value):
                continue
            if isinstance(value, (int, float, str, bool, type(None))):
                fields[name] = value
        return fields

    def _read_final_ultrasonic_snapshot(self, radar):
        try:
            data = radar.get_latest_ultrasonic_radar()
            rows = self._selected_final_ultrasonic_rows(data)
            groups = {}
            for name, ids in (
                ("front", FRONT_ULTRASONIC_IDS),
                ("right", RIGHT_ULTRASONIC_IDS),
                ("rear", REAR_ULTRASONIC_IDS),
                ("left", LEFT_ULTRASONIC_IDS),
            ):
                min_mm, grouped = self._group_final_ultrasonic_rows(rows, ids)
                groups[name] = {"min_mm": min_mm, "rows": grouped}
            return {
                "available": True,
                "rows": tuple((row["id"], row["distance_mm"], row["fault_state"]) for row in rows),
                "groups": groups,
            }
        except Exception as exc:
            return {"available": False, "error": f"{type(exc).__name__}: {exc}"}

    def _selected_final_ultrasonic_rows(self, radar_data):
        rows = []
        for row in radar_data.get("ultrasonic_radar_datas", []):
            radar_id = row.get("id")
            try:
                radar_id = int(radar_id)
            except (TypeError, ValueError):
                continue
            if 0 <= radar_id <= 7:
                rows.append(
                    {
                        "id": radar_id,
                        "distance_mm": row.get("distance_mm"),
                        "fault_state": row.get("fault_state"),
                    }
                )
        rows.sort(key=lambda item: item["id"])
        return rows

    def _group_final_ultrasonic_rows(self, rows, ids):
        by_id = {row["id"]: row for row in rows}
        grouped = []
        valid_values = []
        for radar_id in ids:
            row = by_id.get(radar_id)
            if row is None:
                grouped.append((radar_id, None, "missing"))
                continue
            distance = row.get("distance_mm")
            fault_state = row.get("fault_state")
            if fault_state != 0:
                grouped.append((radar_id, distance, f"fault={fault_state}"))
                continue
            distance_mm = self._valid_ultrasonic_distance_mm(distance, min_valid_mm=1)
            if distance_mm is None:
                grouped.append((radar_id, distance, "invalid"))
                continue
            grouped.append((radar_id, distance_mm, "ok"))
            valid_values.append(distance_mm)
        return (min(valid_values) if valid_values else None), tuple(grouped)

    def _read_final_odom_snapshot(self, slam):
        try:
            odom = slam.get_odom_info()
        except Exception as exc:
            return {"available": False, "error": f"{type(exc).__name__}: {exc}"}

        velocity_body = getattr(odom, "velocity_body", None)
        vx = float(getattr(velocity_body, "x", 0.0)) if velocity_body is not None else None
        vy = float(getattr(velocity_body, "y", 0.0)) if velocity_body is not None else None
        vz = float(getattr(velocity_body, "z", 0.0)) if velocity_body is not None else None
        linear = None
        if vx is not None and vy is not None and vz is not None:
            linear = math.sqrt(vx * vx + vy * vy + vz * vz)

        xy = self._odom_xy_from_info(odom)
        yaw_deg = self._extract_yaw_deg_from_odom(odom)
        return {
            "available": True,
            "x": None if xy is None else xy[0],
            "y": None if xy is None else xy[1],
            "yaw_deg": yaw_deg,
            "vx_mps": vx,
            "vy_mps": vy,
            "vz_mps": vz,
            "linear_speed_mps": linear,
            "loc_confidence": getattr(odom, "loc_confidence", None),
            "loc_state": getattr(odom, "loc_state", None),
            "is_sliping": getattr(odom, "is_sliping", None),
        }

    def _odom_xy_from_info(self, odom):
        pose = getattr(odom, "pose", None)
        if pose is not None and hasattr(pose, "x") and hasattr(pose, "y"):
            try:
                return float(getattr(pose, "x")), float(getattr(pose, "y"))
            except (TypeError, ValueError):
                pass
        if pose is not None:
            try:
                return float(pose[0]), float(pose[1])
            except (TypeError, ValueError, IndexError):
                pass
        match = re.search(r"pose=\(([^)]*)\)", repr(odom))
        if match:
            parts = [part.strip() for part in match.group(1).split(",")]
            if len(parts) >= 2:
                try:
                    return float(parts[0]), float(parts[1])
                except ValueError:
                    pass
        return None


def parse_args() -> RuntimeConfig:
    parser = argparse.ArgumentParser(description="G2 七根料工业总控程序")
    parser.add_argument(
        "--base-dir",
        default=str(detect_default_base_dir()),
        help="动作脚本所在目录，默认是本文件所在目录",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印计划，不执行任何动作；不传 --confirm-live 时默认也是 dry-run",
    )
    parser.add_argument("--confirm-live", action="store_true", help="确认现场安全，允许真实执行")
    parser.add_argument(
        "--allow-estop-pedal-fault",
        dest="allow_estop_pedal_fault",
        action="store_true",
        default=True,
        help="兼容旧参数；当前总控已默认屏蔽 emergency_stop_pedal_fault_state",
    )
    parser.add_argument(
        "--strict-estop-pedal-fault",
        dest="allow_estop_pedal_fault",
        action="store_false",
        help="兼容旧参数；当前不再用 emergency_stop_pedal_fault_state 阻断流程",
    )
    parser.add_argument("--start-index", type=int, default=1, help="从第几根开始，1-7")
    parser.add_argument("--end-index", type=int, default=7, help="到第几根结束，1-7")
    parser.add_argument(
        "--resume-after-grab-pull-index",
        type=int,
        default=None,
        help="从第 N 根已抓住并拉出后的后退步骤恢复，1-7",
    )
    parser.add_argument(
        "--resume-after-grab-retreat-index",
        type=int,
        default=None,
        help="从第 N 根抓取后已后退、准备右转去放料的步骤恢复，1-7",
    )
    parser.add_argument(
        "--resume-after-place-above-index",
        type=int,
        default=None,
        help="从第 N 根已移动到放料上方、准备靠近放料目标距离的步骤恢复，1-7",
    )
    parser.add_argument(
        "--resume-after-place-pull-index",
        type=int,
        default=None,
        help="从第 N 根已放料、开夹、拉出后的放料后后退步骤恢复，1-7",
    )
    parser.add_argument(
        "--resume-after-place-retreat-target-index",
        "--resume-after-place-overretreat-index",
        dest="resume_after_place_retreat_target_index",
        type=int,
        default=None,
        help="从第 N 根放料后后退中断/过退处恢复：先恢复到前超声目标距离，再左转，1-7",
    )
    parser.add_argument("--settle-s", type=float, default=0.5, help="每步完成后的等待时间")
    parser.add_argument("--script-timeout-s", type=float, default=90.0, help="单个动作脚本超时")
    parser.add_argument("--turn-timeout-s", type=float, default=45.0, help="转向 90 度超时")
    parser.add_argument("--grab-distance-mm", type=int, default=155, help="抓取前靠近目标距离")
    parser.add_argument(
        "--grab-vertical-stack",
        action="store_true",
        help=(
            "启用竖排抓取优化：第 1 根作为基准 XY/姿态，第 2-7 根只按 Z "
            f"逐层下移，默认层距 {abs(DEFAULT_GRAB_VERTICAL_STACK_PITCH_M):.3f}m"
        ),
    )
    parser.add_argument(
        "--grab-vertical-stack-pitch-m",
        type=float,
        default=None,
        help=(
            "覆盖竖排抓取层间距，单位 m；正值向上，负值向下。"
            "本料架第 2 根比第 1 根低，默认使用 -0.060。"
        ),
    )
    parser.add_argument("--place-distance-mm", type=int, default=327, help="放置前靠近目标距离")
    parser.add_argument(
        "--grab-brake-margin-mm",
        type=int,
        default=70,
        help="抓料精定位提前停车补偿；默认触发距离=155+70=225mm",
    )
    parser.add_argument(
        "--place-brake-margin-mm",
        type=int,
        default=60,
        help="放料精定位提前停车补偿；默认触发距离=327+60=387mm",
    )
    parser.add_argument(
        "--grab-min-safe-mm",
        type=int,
        default=135,
        help="抓料停稳后前超声最低安全距离，低于该值立即失败停机",
    )
    parser.add_argument(
        "--place-min-safe-mm",
        type=int,
        default=280,
        help="放料停稳后前超声硬安全下限；目标为 327mm，低于该值立即失败停机",
    )
    parser.add_argument(
        "--grab-target-tolerance-mm",
        type=int,
        default=10,
        help="抓料停稳后目标窗口，默认 155±10mm；同时受 grab-min-safe-mm 约束",
    )
    parser.add_argument(
        "--grab-correction-speed-mps",
        type=float,
        default=0.02,
        help="抓料停稳后偏离目标窗口时低速前补/后退速度，默认 0.02m/s",
    )
    parser.add_argument(
        "--grab-correction-max-passes",
        type=int,
        default=5,
        help="抓料停稳后偏离 155±10mm 时最多双向纠偏次数，默认 5 次",
    )
    parser.add_argument(
        "--grab-angle-correction-max-span-mm",
        type=int,
        default=25,
        help="抓料停稳后前 0/1 超声左右差超过该值时做小角度试探纠偏，默认 25mm；0 表示关闭",
    )
    parser.add_argument(
        "--grab-angle-correction-max-passes",
        type=int,
        default=2,
        help="抓料小角度试探纠偏最多次数，默认 2 次",
    )
    parser.add_argument(
        "--grab-angle-correction-angular-speed-radps",
        type=float,
        default=0.05,
        help="抓料角度试探纠偏角速度，默认 0.05rad/s",
    )
    parser.add_argument(
        "--grab-angle-correction-probe-s",
        type=float,
        default=0.20,
        help="抓料角度试探单次脉冲时长，默认 0.20s",
    )
    parser.add_argument(
        "--grab-target-avg-accept-span-mm",
        type=int,
        default=0,
        help=(
            "抓料目标窗口边界补偿；>0 时允许双前超声平均值在 155±tolerance "
            "且跨度不超过该值时通过，默认关闭"
        ),
    )
    parser.add_argument(
        "--place-target-tolerance-mm",
        type=int,
        default=30,
        help="放料停稳后必须进入 327mm 正负该窗口，默认 327±30mm",
    )
    parser.add_argument(
        "--place-correction-speed-mps",
        type=float,
        default=0.05,
        help="放料停得太远时二次低速补近速度，默认 0.05m/s",
    )
    parser.add_argument(
        "--place-correction-max-passes",
        type=int,
        default=2,
        help="放料停得太远时最多二次补近次数，默认 2 次",
    )
    parser.add_argument(
        "--place-retreat-front-target-mm",
        "--place-overretreat-front-target-mm",
        dest="place_retreat_front_target_mm",
        type=int,
        default=None,
        help="放料后退恢复的前超声绝对目标；不传则用 place-distance-mm + retreat-distance-m",
    )
    parser.add_argument(
        "--place-retreat-target-tolerance-mm",
        type=int,
        default=70,
        help="放料后退恢复目标窗口，默认目标正负 70mm",
    )
    parser.add_argument(
        "--place-retreat-forward-speed-mps",
        type=float,
        default=0.10,
        help="放料后退过退时低速前补速度，默认 0.10m/s",
    )
    parser.add_argument(
        "--place-retreat-forward-brake-margin-mm",
        type=int,
        default=100,
        help="放料后退过退前补的提前停车补偿，默认 100mm",
    )
    parser.add_argument(
        "--place-retreat-forward-correction-speed-mps",
        type=float,
        default=0.04,
        help="放料后退过退前补仍偏远时的二次低速补近速度，默认 0.04m/s",
    )
    parser.add_argument(
        "--place-retreat-correction-max-passes",
        type=int,
        default=3,
        help="放料后退目标恢复最多二次纠偏次数，默认 3 次",
    )
    parser.add_argument(
        "--rack-centering-mode",
        choices=("off", "monitor", "shadow", "guarded", "active"),
        default="monitor",
        help=(
            "料架居中识别模式。monitor=只记录点云 pose；"
            "shadow=记录 pose 并生成不执行的 yaw/横移候选；"
            "guarded=抓料/放料进场前必须判定在居中窗口内，否则停机；"
            "active=抓料/放料进场前小步蟹行横向闭环；off=关闭"
        ),
    )
    parser.add_argument(
        "--rack-pose-samples",
        type=int,
        default=3,
        help="每次料架 pose 监控采样帧数，默认 3",
    )
    parser.add_argument(
        "--rack-pose-interval-s",
        type=float,
        default=0.12,
        help="料架 pose 监控采样间隔，默认 0.12s",
    )
    parser.add_argument(
        "--rack-pose-min-range-m",
        type=float,
        default=0.80,
        help="料架 pose 点云 ROI 最小前向距离，默认 0.80m，为实测稳定居中窗口",
    )
    parser.add_argument(
        "--rack-pose-max-range-m",
        type=float,
        default=1.40,
        help="料架 pose 点云 ROI 最大前向距离，默认 1.40m，为实测稳定居中窗口",
    )
    parser.add_argument(
        "--rack-pose-lateral-half-width-m",
        type=float,
        default=0.50,
        help="料架 pose 点云 ROI 横向半宽，默认 0.50m，为实测稳定居中窗口",
    )
    parser.add_argument(
        "--rack-pose-z-min-m",
        type=float,
        default=0.60,
        help="料架 pose 点云 ROI 最小高度，默认 0.60m",
    )
    parser.add_argument(
        "--rack-pose-z-max-m",
        type=float,
        default=1.20,
        help="料架 pose 点云 ROI 最大高度，默认 1.20m",
    )
    parser.add_argument(
        "--rack-pose-bin-width-m",
        type=float,
        default=0.20,
        help="料架 pose 点云前向分箱宽度，默认 0.20m，为实测稳定居中窗口",
    )
    parser.add_argument(
        "--rack-pose-min-cluster-points",
        type=int,
        default=20,
        help="料架 pose 最近稳定点簇最少点数，默认 20",
    )
    parser.add_argument(
        "--rack-place-pose-min-range-m",
        type=float,
        default=None,
        help="放置阶段专用料架 pose ROI 最小前向距离；默认沿用 --rack-pose-min-range-m",
    )
    parser.add_argument(
        "--rack-place-pose-max-range-m",
        type=float,
        default=None,
        help="放置阶段专用料架 pose ROI 最大前向距离；默认沿用 --rack-pose-max-range-m",
    )
    parser.add_argument(
        "--rack-place-pose-lateral-half-width-m",
        type=float,
        default=None,
        help="放置阶段专用料架 pose ROI 横向半宽；默认沿用 --rack-pose-lateral-half-width-m",
    )
    parser.add_argument(
        "--rack-place-pose-z-min-m",
        type=float,
        default=None,
        help="放置阶段专用料架 pose ROI 最小高度；默认沿用 --rack-pose-z-min-m",
    )
    parser.add_argument(
        "--rack-place-pose-z-max-m",
        type=float,
        default=None,
        help="放置阶段专用料架 pose ROI 最大高度；默认沿用 --rack-pose-z-max-m",
    )
    parser.add_argument(
        "--rack-place-pose-bin-width-m",
        type=float,
        default=None,
        help="放置阶段专用料架 pose ROI 前向分箱宽度；默认沿用 --rack-pose-bin-width-m",
    )
    parser.add_argument(
        "--rack-place-pose-min-cluster-points",
        type=int,
        default=None,
        help="放置阶段专用料架 pose 最近稳定点簇最少点数；默认沿用 --rack-pose-min-cluster-points",
    )
    parser.add_argument(
        "--rack-pose-warn-lateral-m",
        type=float,
        default=0.08,
        help="料架横向中心偏移超过该值时只记录 warn，默认 0.08m",
    )
    parser.add_argument(
        "--rack-pose-warn-yaw-deg",
        type=float,
        default=3.0,
        help="料架 yaw 偏差超过该值时只记录 warn，默认 3deg",
    )
    parser.add_argument(
        "--rack-pose-min-confidence",
        type=float,
        default=0.45,
        help="料架 pose 置信度低于该值时只记录 warn，默认 0.45",
    )
    parser.add_argument(
        "--rack-near-target-skip-centering-margin-mm",
        type=int,
        default=80,
        help=(
            "进场前若前超声已经进入 target+tolerance+该余量，则跳过 lidar 居中，"
            "交给前超声目标窗口纠偏；默认 80mm"
        ),
    )
    parser.add_argument(
        "--disable-rack-guarded-ultrasonic-override",
        dest="rack_guarded_ultrasonic_override",
        action="store_false",
        default=True,
        help=(
            "关闭 guarded 模式下的双前超声平行复核；默认开启，用于 lidar yaw/样本不稳但"
            "双前超声稳定一致时避免误停"
        ),
    )
    parser.add_argument(
        "--rack-guarded-ultrasonic-max-pose-lateral-m",
        type=float,
        default=0.12,
        help="guarded 前超声复核允许的最大 pose 横向偏移，默认 0.12m",
    )
    parser.add_argument(
        "--rack-guarded-ultrasonic-max-front-span-mm",
        type=int,
        default=35,
        help="guarded 前超声复核允许 0/1 左右差最大值，默认 35mm",
    )
    parser.add_argument(
        "--rack-guarded-ultrasonic-min-front-mm",
        type=int,
        default=450,
        help="guarded 前超声复核要求前方有效距离不小于该值，默认 450mm",
    )
    parser.add_argument(
        "--disable-rack-post-approach-guarded-check",
        dest="rack_post_approach_guarded_check",
        action="store_false",
        default=True,
        help=(
            "关闭贴近到目标距离后的二次 guarded 复核；默认开启，"
            "只在 rack-centering-mode=guarded/active 时生效"
        ),
    )
    parser.add_argument(
        "--rack-post-approach-max-lateral-m",
        type=float,
        default=0.12,
        help="贴近后 postcheck 允许的最大稳定横向偏移，默认 0.12m",
    )
    parser.add_argument(
        "--rack-post-approach-max-yaw-deg",
        type=float,
        default=7.0,
        help="贴近后 postcheck 允许的最大稳定 yaw，默认 7deg",
    )
    parser.add_argument(
        "--rack-post-approach-max-front-span-mm",
        type=int,
        default=80,
        help="贴近后 postcheck 允许双前超声左右差最大中位数，默认 80mm",
    )
    parser.add_argument(
        "--rack-post-approach-front-retry-windows",
        type=int,
        default=2,
        help=(
            "postcheck 仅出现单窗口 max-span 离群时的只读重采样次数；"
            "中位 span/最小安全距离异常仍立即停机，默认 2"
        ),
    )
    parser.add_argument(
        "--front-too-close-safe-backoff-retries",
        type=int,
        default=2,
        help=(
            "前超声过近自动小后退时，若后超声在运动前单次报障，允许只读重试次数；"
            "持续报障仍停机，默认 2"
        ),
    )
    parser.add_argument(
        "--rack-yaw-shadow-min-distance-m",
        type=float,
        default=0.70,
        help="shadow yaw 候选只使用该距离以上的 before_approach pose，默认 0.70m",
    )
    parser.add_argument(
        "--rack-yaw-shadow-max-distance-m",
        type=float,
        default=1.60,
        help="shadow yaw 候选只使用该距离以下的 before_approach pose，默认 1.60m",
    )
    parser.add_argument(
        "--rack-yaw-shadow-min-confidence",
        type=float,
        default=0.65,
        help="shadow yaw 候选最小 pose 置信度，默认 0.65",
    )
    parser.add_argument(
        "--rack-yaw-shadow-max-fit-residual-m",
        type=float,
        default=0.045,
        help="shadow yaw 候选最大点云直线拟合残差，默认 0.045m",
    )
    parser.add_argument(
        "--rack-yaw-shadow-trigger-deg",
        type=float,
        default=3.0,
        help="shadow yaw 候选触发角度阈值，低于该值记录 no_correction_needed，默认 3deg",
    )
    parser.add_argument(
        "--rack-yaw-shadow-max-deg",
        type=float,
        default=12.0,
        help="shadow yaw 候选最大角度，超过视为需要人工复核，默认 12deg",
    )
    parser.add_argument(
        "--rack-lateral-shadow-min-distance-m",
        type=float,
        default=0.70,
        help="shadow 横移候选只使用该距离以上的 before_approach pose，默认 0.70m",
    )
    parser.add_argument(
        "--rack-lateral-shadow-max-distance-m",
        type=float,
        default=1.60,
        help="shadow 横移候选只使用该距离以下的 before_approach pose，默认 1.60m",
    )
    parser.add_argument(
        "--rack-lateral-shadow-min-confidence",
        type=float,
        default=0.65,
        help="shadow 横移候选最小 pose 置信度，默认 0.65",
    )
    parser.add_argument(
        "--rack-lateral-shadow-max-fit-residual-m",
        type=float,
        default=0.070,
        help="shadow/active 横移候选最大点云直线拟合残差，默认 0.070m",
    )
    parser.add_argument(
        "--rack-lateral-shadow-trigger-m",
        type=float,
        default=0.025,
        help="shadow 横移候选触发横向偏移阈值，低于该值记录 no_correction_needed，默认 0.025m",
    )
    parser.add_argument(
        "--rack-lateral-shadow-max-m",
        type=float,
        default=0.12,
        help="shadow 横向偏移最大可信值，超过视为需要人工复核，默认 0.12m",
    )
    parser.add_argument(
        "--rack-lateral-shadow-max-correction-m",
        type=float,
        default=0.03,
        help="shadow 单次候选横移修正量截断值，不执行真实运动，默认 0.03m",
    )
    parser.add_argument(
        "--rack-lateral-active-target-m",
        type=float,
        default=0.08,
        help="active 横向居中目标窗口，|lateral_center_m| 小于等于该值直接进场，默认 0.08m",
    )
    parser.add_argument(
        "--rack-lateral-active-max-initial-m",
        type=float,
        default=0.18,
        help="active 允许自动蟹行修正的最大初始横向偏移，超过则停机复核，默认 0.18m",
    )
    parser.add_argument(
        "--rack-lateral-active-max-yaw-deg",
        type=float,
        default=5.0,
        help="active 横向修正允许的最大料架 yaw，超过说明不是纯横向偏，默认 5deg",
    )
    parser.add_argument(
        "--rack-lateral-active-max-passes",
        type=int,
        default=3,
        help="active 横向居中最多小步蟹行次数，默认 3 次",
    )
    parser.add_argument(
        "--rack-lateral-active-speed-mps",
        type=float,
        default=0.03,
        help="active 蟹行单步 linear.y 速度，默认 0.03m/s",
    )
    parser.add_argument(
        "--rack-lateral-active-direction",
        choices=("disabled", "same-sign", "opposite-sign"),
        default="disabled",
        help=(
            "active 横移方向。disabled=默认，偏差超出目标时只阻断不横移；"
            "same-sign=linear.y 与 lateral_center_m 同号；"
            "opposite-sign=linear.y 与 lateral_center_m 反号，仅用于受保护诊断"
        ),
    )
    parser.add_argument(
        "--rack-lateral-active-step-s",
        type=float,
        default=0.6,
        help="active 蟹行单步持续时间，默认 0.6s",
    )
    parser.add_argument(
        "--rack-lateral-active-settle-s",
        type=float,
        default=0.6,
        help="active 单步蟹行后等待点云/底盘停稳时间，默认 0.6s",
    )
    parser.add_argument(
        "--rack-lateral-active-min-improvement-m",
        type=float,
        default=0.006,
        help="active 每步后 |横向偏移| 至少改善该值，否则停机复核，默认 0.006m",
    )
    parser.add_argument(
        "--rack-lateral-active-min-front-mm",
        type=int,
        default=450,
        help="active 蟹行前若前方有效超声小于该值则拒绝横移，默认 450mm",
    )
    parser.add_argument(
        "--rack-lateral-active-min-rear-mm",
        type=int,
        default=450,
        help="active 蟹行前若后方有效超声小于该值则拒绝横移，默认 450mm",
    )
    parser.add_argument(
        "--rack-lateral-active-min-side-mm",
        type=int,
        default=450,
        help="active 蟹行前若任一侧向有效超声小于该值则拒绝横移，默认 450mm",
    )
    parser.add_argument(
        "--rack-lateral-active-hard-min-front-mm",
        type=int,
        default=350,
        help="active 蟹行前窗口内前方 raw-min 小于该值则拒绝横移，默认 350mm",
    )
    parser.add_argument(
        "--rack-lateral-active-hard-min-rear-mm",
        type=int,
        default=350,
        help="active 蟹行前窗口内后方 raw-min 小于该值则拒绝横移，默认 350mm",
    )
    parser.add_argument(
        "--rack-lateral-active-hard-min-side-mm",
        type=int,
        default=450,
        help="active 蟹行前窗口内任一侧向 raw-min 小于该值则拒绝横移，默认 450mm",
    )
    parser.add_argument(
        "--rack-lateral-active-clearance-samples",
        type=int,
        default=5,
        help="active 蟹行前超声 clearance 窗口采样帧数，默认 5",
    )
    parser.add_argument(
        "--rack-lateral-active-clearance-interval-s",
        type=float,
        default=0.12,
        help="active 蟹行前超声 clearance 窗口采样间隔，默认 0.12s",
    )
    parser.add_argument(
        "--rack-lateral-active-hz",
        type=float,
        default=20.0,
        help="active 蟹行速度命令发送频率，默认 20Hz",
    )
    parser.add_argument(
        "--rack-lateral-active-max-sample-span-m",
        type=float,
        default=0.08,
        help="active 决策前多帧 lateral_center_m 最大允许跨度，超过说明 pose 不稳定，默认 0.08m",
    )
    parser.add_argument(
        "--disable-rack-lateral-active-rollback",
        dest="rack_lateral_active_rollback_on_worse",
        action="store_false",
        default=True,
        help="关闭 active 横移变差后的自动反向回退；默认开启",
    )
    parser.add_argument(
        "--rack-lateral-active-rollback-step-scale",
        type=float,
        default=1.0,
        help="active 横移变差时反向回退步长比例，默认 1.0 表示同等时间回退",
    )
    parser.add_argument(
        "--coarse-speed-mps",
        type=float,
        default=0.60,
        help="初定位/粗定位速度，默认 0.60m/s",
    )
    parser.add_argument(
        "--grab-approach-speed-mps",
        type=float,
        default=0.15,
        help="靠近到抓料目标距离的前雷达精定位速度，标定默认 0.15m/s",
    )
    parser.add_argument(
        "--place-approach-speed-mps",
        type=float,
        default=0.15,
        help="靠近到放料目标距离的前雷达精定位速度，标定默认 0.15m/s",
    )
    parser.add_argument("--retreat-distance-m", type=float, default=1.0, help="每次后退距离")
    parser.add_argument(
        "--retreat-target-tolerance-mm",
        type=int,
        default=20,
        help="front-ultrasonic 后退目标增量完成窗口，默认 20mm",
    )
    parser.add_argument("--retreat-speed-mps", type=float, default=0.50, help="后退速度；relative 模式下只保留为记录/开环备选")
    parser.add_argument(
        "--retreat-method",
        choices=("hybrid", "front-ultrasonic", "relative", "velocity"),
        default="front-ultrasonic",
        help=(
            "后退方法。front-ultrasonic=默认，双前雷达增量闭环，少退继续退，多退慢速前补；"
            "hybrid=先 relative，必要时短距离双前雷达脱离后再 relative；"
            "relative=底盘相对位移任务；"
            "velocity=速度开环，仅诊断/应急"
        ),
    )
    parser.add_argument(
        "--retreat-escape-delta-m",
        type=float,
        default=0.22,
        help="hybrid 后退中用于脱离近距离保护区的短距离，默认 0.22m",
    )
    parser.add_argument(
        "--grab-retreat-front-occlusion-escape-threshold-mm",
        type=int,
        default=220,
        help="抓料拉出后，若前超声稳定起点小于该值，先低速短退脱离料/末端遮挡；<=0 关闭",
    )
    parser.add_argument(
        "--grab-retreat-front-occlusion-escape-m",
        type=float,
        default=0.18,
        help="抓料拉出后前超声近距遮挡时的低速短退距离，默认 0.18m",
    )
    parser.add_argument(
        "--grab-retreat-front-occlusion-escape-speed-mps",
        type=float,
        default=0.12,
        help="抓料拉出后前超声近距遮挡短退速度，默认 0.12m/s",
    )
    parser.add_argument(
        "--retreat-front-delta-consistency-mm",
        type=int,
        default=180,
        help="前超声后退时 0/1 两个探头增量最大允许差，超过则停机",
    )
    parser.add_argument(
        "--retreat-odom-tolerance-m",
        type=float,
        default=0.02,
        help="后退 1m 时 SLAM odom 位移交叉校验容差；默认必须在目标±0.02m",
    )
    parser.add_argument(
        "--retreat-require-odom-crosscheck",
        dest="retreat_require_odom_crosscheck",
        action="store_true",
        default=True,
        help="要求后退前后必须读到 SLAM odom；默认开启",
    )
    parser.add_argument(
        "--no-retreat-require-odom-crosscheck",
        dest="retreat_require_odom_crosscheck",
        action="store_false",
        help="诊断/应急时关闭后退 odom 必须可读要求；生产不建议使用",
    )
    parser.add_argument(
        "--disable-retreat-odom-auto-correction",
        dest="retreat_odom_auto_correction",
        action="store_false",
        default=True,
        help=(
            "关闭 front-ultrasonic 后退到窗内后的 odom 尾差自动补偿；"
            "默认开启，补偿后仍执行原始 odom 容差门禁"
        ),
    )
    parser.add_argument(
        "--retreat-odom-auto-correction-max-m",
        type=float,
        default=0.08,
        help="odom 尾差自动补偿单次允许处理的最大误差，默认 0.08m",
    )
    parser.add_argument(
        "--retreat-odom-auto-correction-min-m",
        type=float,
        default=0.010,
        help="odom 尾差自动补偿最小执行距离，小于该值仍按该值低速尝试，默认 0.010m",
    )
    parser.add_argument(
        "--retreat-odom-auto-correction-speed-mps",
        type=float,
        default=0.025,
        help="odom 尾差自动补偿速度，默认 0.025m/s",
    )
    parser.add_argument(
        "--retreat-odom-auto-correction-max-passes",
        type=int,
        default=2,
        help="odom 尾差自动补偿最多小步次数，默认 2 次",
    )
    parser.add_argument(
        "--retreat-odom-auto-correction-front-hard-min-mm",
        type=int,
        default=260,
        help="odom 尾差向前补偿时前超声硬下限，默认 260mm",
    )
    parser.add_argument(
        "--retreat-odom-auto-correction-rear-hard-min-mm",
        type=int,
        default=500,
        help="odom 尾差继续后退补偿时后超声硬下限，默认 500mm",
    )
    parser.add_argument(
        "--retreat-odom-auto-correction-clearance-retry-s",
        type=float,
        default=0.6,
        help=(
            "odom 尾差补偿 clearance 检查遇到超声空帧时的短重试时间，"
            "默认 0.6s；读到真实距离低于 hard-min 仍立即阻断"
        ),
    )
    parser.add_argument(
        "--retreat-open-loop-brake-compensation-m",
        type=float,
        default=0.10,
        help="velocity 开环后退的制动惯性补偿，relative 闭环模式不使用",
    )
    parser.add_argument(
        "--turn-angular-speed-radps",
        type=float,
        default=0.5236,
        help="速度开环原地转向角速度；与 _chassis_rotate.py 一致，约 30deg/s",
    )
    parser.add_argument(
        "--right-turn-duration-s",
        type=float,
        default=3.0,
        help="右转 90 度的速度开环时长；与 _chassis_rotate.py 一致",
    )
    parser.add_argument(
        "--left-turn-duration-s",
        type=float,
        default=3.0,
        help="左转 90 度的速度开环时长；与 _chassis_rotate.py 一致",
    )
    parser.add_argument(
        "--turn-hz",
        type=float,
        default=20.0,
        help="转向速度命令发送频率",
    )
    parser.add_argument(
        "--turn-control-mode",
        type=int,
        default=0,
        choices=(0, 1),
        help="速度开环转向时的底盘控制模式；_chassis_rotate.py 使用 mode=0",
    )
    parser.add_argument(
        "--turn-method",
        choices=("relative", "velocity"),
        default="velocity",
        help=(
            "90度转向方法。velocity=速度控制+odom yaw闭环，默认生产路径；"
            "relative=PNC相对转向任务，仅保留为对比诊断"
        ),
    )
    parser.add_argument(
        "--turn-success-states",
        default="3,9",
        help=(
            "relative 转向允许继续流程的最终状态，默认 3,9；"
            "不要默认加入 7，因为 7 是取消/结束，不证明已到90度"
        ),
    )
    parser.add_argument(
        "--turn-min-sensor-delta-mm",
        type=int,
        default=180,
        help=(
            "velocity 开环转向后的最低超声场景变化阈值；"
            "只能发现基本没转，不能替代 yaw 闭环"
        ),
    )
    parser.add_argument(
        "--turn-yaw-tolerance-deg",
        type=float,
        default=0.5,
        help="转向后 odom yaw 实际角度允许误差，默认 0.5deg",
    )
    parser.add_argument(
        "--turn-confirm-samples",
        type=int,
        default=5,
        help="90 度转向后停稳二次确认的 yaw 采样次数，默认 5 次",
    )
    parser.add_argument(
        "--turn-confirm-interval-s",
        type=float,
        default=0.12,
        help="90 度转向后 yaw 二次确认采样间隔，默认 0.12s",
    )
    parser.add_argument(
        "--turn-confirm-max-span-deg",
        type=float,
        default=0.8,
        help="90 度转向后 yaw 多帧确认允许的最大采样跨度，默认 0.8deg",
    )
    parser.add_argument(
        "--disable-turn-correction",
        action="store_true",
        help="关闭 90 度转向后的低速 yaw 闭环补角；默认开启",
    )
    parser.add_argument(
        "--turn-correction-max-passes",
        type=int,
        default=5,
        help="90 度转向后低速补角最多次数，默认 5 次",
    )
    parser.add_argument(
        "--turn-correction-angular-speed-radps",
        type=float,
        default=0.08,
        help="90 度转向低速补角角速度，默认 0.08rad/s",
    )
    parser.add_argument(
        "--turn-correction-max-error-deg",
        type=float,
        default=25.0,
        help="允许低速补角的最大 yaw 误差，超过则失败停机，默认 25deg",
    )
    parser.add_argument(
        "--allow-turn-motion-error-2",
        action="store_true",
        help=(
            "仅在现场确认可忽略 collision imminent 时使用："
            "允许转向预检忽略 motion_control_error=2，"
            "但仍阻断其它 motion error、充电、急停和超声供电异常"
        ),
    )
    parser.add_argument(
        "--turn-validation-ok",
        action="store_true",
        help=(
            "确认已单独多次通过左右 90 度转向诊断。真实多根连续运行必须显式传这个参数；"
            "单根 dry-run 或单根实跑不需要。"
        ),
    )
    parser.add_argument(
        "--log-file",
        default=None,
        help="日志文件路径；默认自动写到 base_dir/logs/industrial_7_rods_时间.log",
    )
    parser.add_argument(
        "--event-file",
        default=None,
        help="JSONL 结构化事件文件；默认与日志同名 .jsonl",
    )
    parser.add_argument(
        "--checkpoint-file",
        default=None,
        help="最近一步 checkpoint JSON；默认写到日志同目录 *_checkpoint.json",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="最终运行报告 JSON；默认写到日志同目录 *_report.json",
    )
    parser.add_argument(
        "--allow-existing-artifacts",
        action="store_true",
        help=(
            "允许真实运行复用已存在的 log/jsonl/checkpoint/report 文件；"
            "默认禁止，避免复跑证据混在一起"
        ),
    )

    args = parser.parse_args()
    if args.dry_run and args.confirm_live:
        raise SystemExit("--dry-run and --confirm-live cannot be used together")
    if not (1 <= args.start_index <= 7):
        raise SystemExit("--start-index must be in 1..7")
    if not (1 <= args.end_index <= 7):
        raise SystemExit("--end-index must be in 1..7")
    if args.start_index > args.end_index:
        raise SystemExit("--start-index must be <= --end-index")
    if args.resume_after_grab_pull_index is not None:
        if not (1 <= args.resume_after_grab_pull_index <= 7):
            raise SystemExit("--resume-after-grab-pull-index must be in 1..7")
        if args.resume_after_grab_pull_index > args.end_index:
            raise SystemExit("--resume-after-grab-pull-index must be <= --end-index")
    if args.resume_after_grab_retreat_index is not None:
        if not (1 <= args.resume_after_grab_retreat_index <= 7):
            raise SystemExit("--resume-after-grab-retreat-index must be in 1..7")
        if args.resume_after_grab_retreat_index > args.end_index:
            raise SystemExit("--resume-after-grab-retreat-index must be <= --end-index")
    if args.resume_after_place_above_index is not None:
        if not (1 <= args.resume_after_place_above_index <= 7):
            raise SystemExit("--resume-after-place-above-index must be in 1..7")
        if args.resume_after_place_above_index > args.end_index:
            raise SystemExit("--resume-after-place-above-index must be <= --end-index")
    if args.resume_after_place_pull_index is not None:
        if not (1 <= args.resume_after_place_pull_index <= 7):
            raise SystemExit("--resume-after-place-pull-index must be in 1..7")
        if args.resume_after_place_pull_index > args.end_index:
            raise SystemExit("--resume-after-place-pull-index must be <= --end-index")
    if args.resume_after_place_retreat_target_index is not None:
        if not (1 <= args.resume_after_place_retreat_target_index <= 7):
            raise SystemExit("--resume-after-place-retreat-target-index must be in 1..7")
        if args.resume_after_place_retreat_target_index > args.end_index:
            raise SystemExit("--resume-after-place-retreat-target-index must be <= --end-index")
    resume_count = sum(
        value is not None
        for value in (
            args.resume_after_grab_pull_index,
            args.resume_after_grab_retreat_index,
            args.resume_after_place_above_index,
            args.resume_after_place_pull_index,
            args.resume_after_place_retreat_target_index,
        )
    )
    if resume_count > 1:
        raise SystemExit("choose only one resume option")
    if args.grab_distance_mm <= 0 or args.place_distance_mm <= 0:
        raise SystemExit("front distance targets must be positive")
    grab_vertical_stack_pitch_m = args.grab_vertical_stack_pitch_m
    if args.grab_vertical_stack and grab_vertical_stack_pitch_m is None:
        grab_vertical_stack_pitch_m = DEFAULT_GRAB_VERTICAL_STACK_PITCH_M
    if grab_vertical_stack_pitch_m is not None and abs(grab_vertical_stack_pitch_m) > 0.20:
        raise SystemExit("--grab-vertical-stack-pitch-m is capped at +/-0.20m per layer")
    if args.grab_brake_margin_mm < 0 or args.place_brake_margin_mm < 0:
        raise SystemExit("brake margins must be >= 0")
    if args.grab_min_safe_mm <= 0 or args.place_min_safe_mm <= 0:
        raise SystemExit("minimum safe front distances must be positive")
    if args.grab_target_tolerance_mm < 0 or args.place_target_tolerance_mm < 0:
        raise SystemExit("target tolerances must be >= 0")
    if args.place_retreat_front_target_mm is not None and args.place_retreat_front_target_mm <= 0:
        raise SystemExit("--place-retreat-front-target-mm must be positive")
    if args.place_retreat_target_tolerance_mm < 0:
        raise SystemExit("--place-retreat-target-tolerance-mm must be >= 0")
    if args.place_retreat_forward_brake_margin_mm < 0:
        raise SystemExit("--place-retreat-forward-brake-margin-mm must be >= 0")
    if args.grab_correction_speed_mps <= 0:
        raise SystemExit("--grab-correction-speed-mps must be positive")
    if args.place_correction_speed_mps <= 0:
        raise SystemExit("--place-correction-speed-mps must be positive")
    if args.place_retreat_forward_speed_mps <= 0:
        raise SystemExit("--place-retreat-forward-speed-mps must be positive")
    if args.place_retreat_forward_correction_speed_mps <= 0:
        raise SystemExit("--place-retreat-forward-correction-speed-mps must be positive")
    if args.grab_correction_max_passes < 0 or args.place_correction_max_passes < 0:
        raise SystemExit("correction max passes must be >= 0")
    if args.grab_angle_correction_max_span_mm < 0:
        raise SystemExit("--grab-angle-correction-max-span-mm must be >= 0")
    if args.grab_angle_correction_max_passes < 0:
        raise SystemExit("--grab-angle-correction-max-passes must be >= 0")
    if args.grab_angle_correction_angular_speed_radps <= 0:
        raise SystemExit("--grab-angle-correction-angular-speed-radps must be positive")
    if args.grab_angle_correction_angular_speed_radps > 0.10:
        raise SystemExit("--grab-angle-correction-angular-speed-radps is capped at 0.10")
    if args.grab_angle_correction_probe_s <= 0:
        raise SystemExit("--grab-angle-correction-probe-s must be positive")
    if args.grab_angle_correction_probe_s > 1.0:
        raise SystemExit("--grab-angle-correction-probe-s is capped at 1.0s")
    if args.grab_target_avg_accept_span_mm < 0:
        raise SystemExit("--grab-target-avg-accept-span-mm must be >= 0")
    if args.grab_target_avg_accept_span_mm > args.grab_angle_correction_max_span_mm:
        raise SystemExit(
            "--grab-target-avg-accept-span-mm must be <= "
            "--grab-angle-correction-max-span-mm"
        )
    if args.place_retreat_correction_max_passes < 0:
        raise SystemExit("--place-retreat-correction-max-passes must be >= 0")
    if args.rack_pose_samples <= 0:
        raise SystemExit("--rack-pose-samples must be positive")
    if args.rack_centering_mode in ("guarded", "active") and args.rack_pose_samples < 8:
        raise SystemExit("--rack-centering-mode guarded/active requires --rack-pose-samples >= 8")
    if args.rack_pose_interval_s < 0:
        raise SystemExit("--rack-pose-interval-s must be >= 0")
    if args.rack_pose_min_range_m <= 0:
        raise SystemExit("--rack-pose-min-range-m must be positive")
    if args.rack_pose_max_range_m <= args.rack_pose_min_range_m:
        raise SystemExit("--rack-pose-max-range-m must be larger than --rack-pose-min-range-m")
    if args.rack_pose_lateral_half_width_m <= 0:
        raise SystemExit("--rack-pose-lateral-half-width-m must be positive")
    if args.rack_pose_z_min_m >= args.rack_pose_z_max_m:
        raise SystemExit("--rack-pose-z-min-m must be smaller than --rack-pose-z-max-m")
    if args.rack_pose_bin_width_m <= 0:
        raise SystemExit("--rack-pose-bin-width-m must be positive")
    if args.rack_pose_min_cluster_points <= 0:
        raise SystemExit("--rack-pose-min-cluster-points must be positive")
    place_pose_min_range_m = (
        args.rack_place_pose_min_range_m
        if args.rack_place_pose_min_range_m is not None
        else args.rack_pose_min_range_m
    )
    place_pose_max_range_m = (
        args.rack_place_pose_max_range_m
        if args.rack_place_pose_max_range_m is not None
        else args.rack_pose_max_range_m
    )
    place_pose_lateral_half_width_m = (
        args.rack_place_pose_lateral_half_width_m
        if args.rack_place_pose_lateral_half_width_m is not None
        else args.rack_pose_lateral_half_width_m
    )
    place_pose_z_min_m = (
        args.rack_place_pose_z_min_m
        if args.rack_place_pose_z_min_m is not None
        else args.rack_pose_z_min_m
    )
    place_pose_z_max_m = (
        args.rack_place_pose_z_max_m
        if args.rack_place_pose_z_max_m is not None
        else args.rack_pose_z_max_m
    )
    place_pose_bin_width_m = (
        args.rack_place_pose_bin_width_m
        if args.rack_place_pose_bin_width_m is not None
        else args.rack_pose_bin_width_m
    )
    place_pose_min_cluster_points = (
        args.rack_place_pose_min_cluster_points
        if args.rack_place_pose_min_cluster_points is not None
        else args.rack_pose_min_cluster_points
    )
    if place_pose_min_range_m <= 0:
        raise SystemExit("--rack-place-pose-min-range-m must be positive")
    if place_pose_max_range_m <= place_pose_min_range_m:
        raise SystemExit(
            "--rack-place-pose-max-range-m must be larger than effective place min range"
        )
    if place_pose_lateral_half_width_m <= 0:
        raise SystemExit("--rack-place-pose-lateral-half-width-m must be positive")
    if place_pose_z_min_m >= place_pose_z_max_m:
        raise SystemExit(
            "--rack-place-pose-z-min-m must be smaller than effective place z max"
        )
    if place_pose_bin_width_m <= 0:
        raise SystemExit("--rack-place-pose-bin-width-m must be positive")
    if place_pose_min_cluster_points <= 0:
        raise SystemExit("--rack-place-pose-min-cluster-points must be positive")
    if args.rack_pose_warn_lateral_m <= 0:
        raise SystemExit("--rack-pose-warn-lateral-m must be positive")
    if args.rack_pose_warn_yaw_deg <= 0:
        raise SystemExit("--rack-pose-warn-yaw-deg must be positive")
    if not (0.0 <= args.rack_pose_min_confidence <= 1.0):
        raise SystemExit("--rack-pose-min-confidence must be in [0, 1]")
    if args.rack_near_target_skip_centering_margin_mm < 0:
        raise SystemExit("--rack-near-target-skip-centering-margin-mm must be >= 0")
    if args.rack_near_target_skip_centering_margin_mm > 300:
        raise SystemExit("--rack-near-target-skip-centering-margin-mm is capped at 300mm")
    if args.rack_guarded_ultrasonic_max_pose_lateral_m <= 0:
        raise SystemExit("--rack-guarded-ultrasonic-max-pose-lateral-m must be positive")
    if args.rack_guarded_ultrasonic_max_pose_lateral_m > 0.20:
        raise SystemExit("--rack-guarded-ultrasonic-max-pose-lateral-m is capped at 0.20m")
    if args.rack_guarded_ultrasonic_max_front_span_mm <= 0:
        raise SystemExit("--rack-guarded-ultrasonic-max-front-span-mm must be positive")
    if args.rack_guarded_ultrasonic_max_front_span_mm > 120:
        raise SystemExit("--rack-guarded-ultrasonic-max-front-span-mm is capped at 120mm")
    if args.rack_guarded_ultrasonic_min_front_mm <= 0:
        raise SystemExit("--rack-guarded-ultrasonic-min-front-mm must be positive")
    if args.rack_post_approach_max_lateral_m <= 0:
        raise SystemExit("--rack-post-approach-max-lateral-m must be positive")
    if args.rack_post_approach_max_lateral_m > 0.20:
        raise SystemExit("--rack-post-approach-max-lateral-m is capped at 0.20m")
    if args.rack_post_approach_max_yaw_deg <= 0:
        raise SystemExit("--rack-post-approach-max-yaw-deg must be positive")
    if args.rack_post_approach_max_yaw_deg > 15.0:
        raise SystemExit("--rack-post-approach-max-yaw-deg is capped at 15deg")
    if args.rack_post_approach_max_front_span_mm <= 0:
        raise SystemExit("--rack-post-approach-max-front-span-mm must be positive")
    if args.rack_post_approach_max_front_span_mm > 200:
        raise SystemExit("--rack-post-approach-max-front-span-mm is capped at 200mm")
    if args.rack_post_approach_front_retry_windows < 0:
        raise SystemExit("--rack-post-approach-front-retry-windows must be >= 0")
    if args.rack_post_approach_front_retry_windows > 5:
        raise SystemExit("--rack-post-approach-front-retry-windows is capped at 5")
    if args.front_too_close_safe_backoff_retries < 0:
        raise SystemExit("--front-too-close-safe-backoff-retries must be >= 0")
    if args.front_too_close_safe_backoff_retries > 5:
        raise SystemExit("--front-too-close-safe-backoff-retries is capped at 5")
    if args.rack_yaw_shadow_min_distance_m <= 0:
        raise SystemExit("--rack-yaw-shadow-min-distance-m must be positive")
    if args.rack_yaw_shadow_max_distance_m <= args.rack_yaw_shadow_min_distance_m:
        raise SystemExit(
            "--rack-yaw-shadow-max-distance-m must be larger than "
            "--rack-yaw-shadow-min-distance-m"
        )
    if not (0.0 <= args.rack_yaw_shadow_min_confidence <= 1.0):
        raise SystemExit("--rack-yaw-shadow-min-confidence must be in [0, 1]")
    if args.rack_yaw_shadow_max_fit_residual_m <= 0:
        raise SystemExit("--rack-yaw-shadow-max-fit-residual-m must be positive")
    if args.rack_yaw_shadow_trigger_deg <= 0:
        raise SystemExit("--rack-yaw-shadow-trigger-deg must be positive")
    if args.rack_yaw_shadow_max_deg <= args.rack_yaw_shadow_trigger_deg:
        raise SystemExit("--rack-yaw-shadow-max-deg must be larger than trigger")
    if args.rack_lateral_shadow_min_distance_m <= 0:
        raise SystemExit("--rack-lateral-shadow-min-distance-m must be positive")
    if args.rack_lateral_shadow_max_distance_m <= args.rack_lateral_shadow_min_distance_m:
        raise SystemExit(
            "--rack-lateral-shadow-max-distance-m must be larger than "
            "--rack-lateral-shadow-min-distance-m"
        )
    if not (0.0 <= args.rack_lateral_shadow_min_confidence <= 1.0):
        raise SystemExit("--rack-lateral-shadow-min-confidence must be in [0, 1]")
    if args.rack_lateral_shadow_max_fit_residual_m <= 0:
        raise SystemExit("--rack-lateral-shadow-max-fit-residual-m must be positive")
    if args.rack_lateral_shadow_trigger_m <= 0:
        raise SystemExit("--rack-lateral-shadow-trigger-m must be positive")
    if args.rack_lateral_shadow_max_m <= args.rack_lateral_shadow_trigger_m:
        raise SystemExit("--rack-lateral-shadow-max-m must be larger than trigger")
    if args.rack_lateral_shadow_max_correction_m <= 0:
        raise SystemExit("--rack-lateral-shadow-max-correction-m must be positive")
    if args.rack_lateral_shadow_max_correction_m > args.rack_lateral_shadow_max_m:
        raise SystemExit("--rack-lateral-shadow-max-correction-m must be <= max lateral")
    if args.rack_lateral_active_target_m <= 0:
        raise SystemExit("--rack-lateral-active-target-m must be positive")
    if args.rack_lateral_active_max_initial_m <= args.rack_lateral_active_target_m:
        raise SystemExit("--rack-lateral-active-max-initial-m must be larger than target")
    if args.rack_lateral_active_max_initial_m > 0.30:
        raise SystemExit("--rack-lateral-active-max-initial-m is capped at 0.30m")
    if args.rack_lateral_active_max_yaw_deg <= 0:
        raise SystemExit("--rack-lateral-active-max-yaw-deg must be positive")
    if args.rack_lateral_active_max_yaw_deg > 15.0:
        raise SystemExit("--rack-lateral-active-max-yaw-deg is capped at 15deg")
    if args.rack_lateral_active_max_passes <= 0:
        raise SystemExit("--rack-lateral-active-max-passes must be positive")
    if args.rack_lateral_active_speed_mps <= 0:
        raise SystemExit("--rack-lateral-active-speed-mps must be positive")
    if args.rack_lateral_active_speed_mps > 0.05:
        raise SystemExit("--rack-lateral-active-speed-mps is capped at 0.05")
    if args.rack_lateral_active_step_s <= 0:
        raise SystemExit("--rack-lateral-active-step-s must be positive")
    if args.rack_lateral_active_step_s > 1.0:
        raise SystemExit("--rack-lateral-active-step-s is capped at 1.0s")
    if args.rack_lateral_active_settle_s < 0:
        raise SystemExit("--rack-lateral-active-settle-s must be >= 0")
    if args.rack_lateral_active_settle_s > 2.0:
        raise SystemExit("--rack-lateral-active-settle-s is capped at 2.0s")
    if args.rack_lateral_active_min_improvement_m < 0:
        raise SystemExit("--rack-lateral-active-min-improvement-m must be >= 0")
    if args.rack_lateral_active_min_improvement_m > args.rack_lateral_active_target_m:
        raise SystemExit("--rack-lateral-active-min-improvement-m must be <= active target")
    if args.rack_lateral_active_min_front_mm <= 0:
        raise SystemExit("--rack-lateral-active-min-front-mm must be positive")
    if args.rack_lateral_active_min_rear_mm <= 0:
        raise SystemExit("--rack-lateral-active-min-rear-mm must be positive")
    if args.rack_lateral_active_min_side_mm <= 0:
        raise SystemExit("--rack-lateral-active-min-side-mm must be positive")
    if args.rack_lateral_active_hard_min_front_mm <= 0:
        raise SystemExit("--rack-lateral-active-hard-min-front-mm must be positive")
    if args.rack_lateral_active_hard_min_rear_mm <= 0:
        raise SystemExit("--rack-lateral-active-hard-min-rear-mm must be positive")
    if args.rack_lateral_active_hard_min_side_mm <= 0:
        raise SystemExit("--rack-lateral-active-hard-min-side-mm must be positive")
    if args.rack_lateral_active_hard_min_front_mm > args.rack_lateral_active_min_front_mm:
        raise SystemExit(
            "--rack-lateral-active-hard-min-front-mm must be <= "
            "--rack-lateral-active-min-front-mm"
        )
    if args.rack_lateral_active_hard_min_rear_mm > args.rack_lateral_active_min_rear_mm:
        raise SystemExit(
            "--rack-lateral-active-hard-min-rear-mm must be <= "
            "--rack-lateral-active-min-rear-mm"
        )
    if args.rack_lateral_active_hard_min_side_mm > args.rack_lateral_active_min_side_mm:
        raise SystemExit(
            "--rack-lateral-active-hard-min-side-mm must be <= "
            "--rack-lateral-active-min-side-mm"
        )
    if args.rack_lateral_active_clearance_samples <= 0:
        raise SystemExit("--rack-lateral-active-clearance-samples must be positive")
    if args.rack_lateral_active_clearance_samples > 20:
        raise SystemExit("--rack-lateral-active-clearance-samples is capped at 20")
    if args.rack_lateral_active_clearance_interval_s < 0:
        raise SystemExit("--rack-lateral-active-clearance-interval-s must be >= 0")
    if args.rack_lateral_active_clearance_interval_s > 1.0:
        raise SystemExit("--rack-lateral-active-clearance-interval-s is capped at 1.0s")
    if args.rack_lateral_active_hz <= 0:
        raise SystemExit("--rack-lateral-active-hz must be positive")
    if args.rack_lateral_active_hz > 30.0:
        raise SystemExit("--rack-lateral-active-hz is capped at 30Hz")
    if args.rack_lateral_active_max_sample_span_m <= 0:
        raise SystemExit("--rack-lateral-active-max-sample-span-m must be positive")
    if args.rack_lateral_active_max_sample_span_m > 0.20:
        raise SystemExit("--rack-lateral-active-max-sample-span-m is capped at 0.20m")
    if args.rack_lateral_active_rollback_step_scale <= 0:
        raise SystemExit("--rack-lateral-active-rollback-step-scale must be positive")
    if args.rack_lateral_active_rollback_step_scale > 1.5:
        raise SystemExit("--rack-lateral-active-rollback-step-scale is capped at 1.5")
    if args.grab_min_safe_mm >= args.grab_distance_mm + args.grab_brake_margin_mm:
        raise SystemExit("--grab-min-safe-mm must be smaller than grab trigger distance")
    if args.place_min_safe_mm >= args.place_distance_mm + args.place_brake_margin_mm:
        raise SystemExit("--place-min-safe-mm must be smaller than place trigger distance")
    if args.place_min_safe_mm >= args.place_distance_mm:
        raise SystemExit("--place-min-safe-mm must be smaller than --place-distance-mm")
    if args.retreat_distance_m <= 0:
        raise SystemExit("--retreat-distance-m must be positive")
    if args.retreat_target_tolerance_mm <= 0:
        raise SystemExit("--retreat-target-tolerance-mm must be positive")
    if args.retreat_target_tolerance_mm > 100:
        raise SystemExit("--retreat-target-tolerance-mm is capped at 100mm")
    if args.coarse_speed_mps <= 0:
        raise SystemExit("--coarse-speed-mps must be positive")
    if args.grab_approach_speed_mps <= 0 or args.place_approach_speed_mps <= 0:
        raise SystemExit("fine approach speeds must be positive")
    if args.retreat_speed_mps <= 0:
        raise SystemExit("--retreat-speed-mps must be positive")
    if args.retreat_escape_delta_m < 0:
        raise SystemExit("--retreat-escape-delta-m must be >= 0")
    if args.retreat_escape_delta_m >= args.retreat_distance_m:
        raise SystemExit("--retreat-escape-delta-m must be smaller than retreat distance")
    if args.grab_retreat_front_occlusion_escape_m < 0:
        raise SystemExit("--grab-retreat-front-occlusion-escape-m must be >= 0")
    if args.grab_retreat_front_occlusion_escape_m >= args.retreat_distance_m:
        raise SystemExit(
            "--grab-retreat-front-occlusion-escape-m must be smaller than retreat distance"
        )
    if args.grab_retreat_front_occlusion_escape_speed_mps <= 0:
        raise SystemExit("--grab-retreat-front-occlusion-escape-speed-mps must be positive")
    if args.retreat_front_delta_consistency_mm < 0:
        raise SystemExit("--retreat-front-delta-consistency-mm must be >= 0")
    if args.retreat_odom_tolerance_m < 0:
        raise SystemExit("--retreat-odom-tolerance-m must be >= 0")
    if args.retreat_odom_auto_correction_max_m < 0:
        raise SystemExit("--retreat-odom-auto-correction-max-m must be >= 0")
    if args.retreat_odom_auto_correction_max_m > 0.12:
        raise SystemExit("--retreat-odom-auto-correction-max-m is capped at 0.12m")
    if args.retreat_odom_auto_correction_min_m < 0:
        raise SystemExit("--retreat-odom-auto-correction-min-m must be >= 0")
    if (
        args.retreat_odom_auto_correction
        and args.retreat_odom_auto_correction_min_m > args.retreat_odom_auto_correction_max_m
    ):
        raise SystemExit(
            "--retreat-odom-auto-correction-min-m must be <= "
            "--retreat-odom-auto-correction-max-m"
        )
    if args.retreat_odom_auto_correction_speed_mps <= 0:
        raise SystemExit("--retreat-odom-auto-correction-speed-mps must be positive")
    if args.retreat_odom_auto_correction_speed_mps > 0.04:
        raise SystemExit("--retreat-odom-auto-correction-speed-mps is capped at 0.04")
    if args.retreat_odom_auto_correction_max_passes < 0:
        raise SystemExit("--retreat-odom-auto-correction-max-passes must be >= 0")
    if args.retreat_odom_auto_correction_max_passes > 3:
        raise SystemExit("--retreat-odom-auto-correction-max-passes is capped at 3")
    if args.retreat_odom_auto_correction_front_hard_min_mm <= 0:
        raise SystemExit("--retreat-odom-auto-correction-front-hard-min-mm must be positive")
    if args.retreat_odom_auto_correction_rear_hard_min_mm <= 0:
        raise SystemExit("--retreat-odom-auto-correction-rear-hard-min-mm must be positive")
    if args.retreat_odom_auto_correction_clearance_retry_s < 0:
        raise SystemExit("--retreat-odom-auto-correction-clearance-retry-s must be >= 0")
    if args.retreat_odom_auto_correction_clearance_retry_s > 2.0:
        raise SystemExit("--retreat-odom-auto-correction-clearance-retry-s is capped at 2.0s")
    if args.retreat_open_loop_brake_compensation_m < 0:
        raise SystemExit("--retreat-open-loop-brake-compensation-m must be >= 0")
    if args.retreat_method == "velocity" and args.retreat_open_loop_brake_compensation_m >= args.retreat_distance_m:
        raise SystemExit("--retreat-open-loop-brake-compensation-m must be smaller than retreat distance")
    if args.turn_angular_speed_radps <= 0:
        raise SystemExit("--turn-angular-speed-radps must be positive")
    if args.right_turn_duration_s <= 0 or args.left_turn_duration_s <= 0:
        raise SystemExit("turn durations must be positive")
    if args.turn_hz <= 0:
        raise SystemExit("--turn-hz must be positive")
    if args.coarse_speed_mps > 0.60:
        raise SystemExit("--coarse-speed-mps is capped at 0.60")
    if args.grab_approach_speed_mps > 0.30:
        raise SystemExit("--grab-approach-speed-mps is capped at 0.30 for grab docking")
    if args.place_approach_speed_mps > 0.30:
        raise SystemExit("--place-approach-speed-mps is capped at 0.30 for place docking")
    if args.place_correction_speed_mps > 0.10:
        raise SystemExit("--place-correction-speed-mps is capped at 0.10")
    if args.grab_correction_speed_mps > 0.08:
        raise SystemExit("--grab-correction-speed-mps is capped at 0.08")
    if args.place_retreat_forward_speed_mps > 0.15:
        raise SystemExit("--place-retreat-forward-speed-mps is capped at 0.15")
    if args.place_retreat_forward_correction_speed_mps > 0.08:
        raise SystemExit("--place-retreat-forward-correction-speed-mps is capped at 0.08")
    if args.place_retreat_forward_brake_margin_mm > 250:
        raise SystemExit("--place-retreat-forward-brake-margin-mm is capped at 250mm")
    if args.retreat_speed_mps > 0.50:
        raise SystemExit("--retreat-speed-mps is capped at 0.50")
    if args.turn_angular_speed_radps > 0.60:
        raise SystemExit("--turn-angular-speed-radps is capped at 0.60")
    if args.turn_min_sensor_delta_mm < 0:
        raise SystemExit("--turn-min-sensor-delta-mm must be >= 0")
    if args.turn_yaw_tolerance_deg <= 0:
        raise SystemExit("--turn-yaw-tolerance-deg must be positive")
    if args.turn_yaw_tolerance_deg > 20.0:
        raise SystemExit("--turn-yaw-tolerance-deg is capped at 20deg")
    if args.turn_confirm_samples <= 0:
        raise SystemExit("--turn-confirm-samples must be positive")
    if args.turn_confirm_interval_s < 0:
        raise SystemExit("--turn-confirm-interval-s must be >= 0")
    if args.turn_confirm_max_span_deg <= 0:
        raise SystemExit("--turn-confirm-max-span-deg must be positive")
    if args.turn_correction_max_passes < 0:
        raise SystemExit("--turn-correction-max-passes must be >= 0")
    if args.turn_correction_angular_speed_radps <= 0:
        raise SystemExit("--turn-correction-angular-speed-radps must be positive")
    if args.turn_correction_angular_speed_radps > 0.30:
        raise SystemExit("--turn-correction-angular-speed-radps is capped at 0.30")
    if args.turn_correction_max_error_deg <= 0:
        raise SystemExit("--turn-correction-max-error-deg must be positive")
    if args.turn_correction_max_error_deg > 45.0:
        raise SystemExit("--turn-correction-max-error-deg is capped at 45deg")
    if args.right_turn_duration_s > 10.0 or args.left_turn_duration_s > 10.0:
        raise SystemExit("turn durations are capped at 10s")
    if args.script_timeout_s <= 0 or args.turn_timeout_s <= 0:
        raise SystemExit("timeouts must be positive")
    try:
        turn_success_states = parse_state_list(args.turn_success_states)
    except ValueError as exc:
        raise SystemExit(f"--turn-success-states invalid: {exc}") from exc
    if 7 in turn_success_states:
        raise SystemExit("--turn-success-states must not include 7; state=7 is canceled/ended, not turn success")

    base_dir = Path(args.base_dir).resolve()
    if args.log_file:
        log_file = Path(args.log_file).resolve()
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = base_dir / "logs" / f"industrial_7_rods_{stamp}.log"
    if args.event_file:
        event_file = Path(args.event_file).resolve()
    else:
        event_file = log_file.with_suffix(".jsonl")
    if args.checkpoint_file:
        checkpoint_file = Path(args.checkpoint_file).resolve()
    else:
        checkpoint_file = log_file.with_name(f"{log_file.stem}_checkpoint.json")
    if args.report_file:
        report_file = Path(args.report_file).resolve()
    else:
        report_file = log_file.with_name(f"{log_file.stem}_report.json")

    if args.confirm_live and not args.allow_existing_artifacts:
        artifact_paths = (log_file, event_file, checkpoint_file, report_file)
        existing_artifacts = [str(path) for path in artifact_paths if path.exists()]
        if existing_artifacts:
            raise SystemExit(
                "live run artifact paths already exist; choose a fresh --log-file "
                "or pass --allow-existing-artifacts:\n" + "\n".join(existing_artifacts)
            )

    return RuntimeConfig(
        base_dir=base_dir,
        dry_run=not args.confirm_live,
        confirm_live=args.confirm_live,
        allow_estop_pedal_fault=args.allow_estop_pedal_fault,
        start_index=args.start_index,
        end_index=args.end_index,
        settle_s=args.settle_s,
        script_timeout_s=args.script_timeout_s,
        turn_timeout_s=args.turn_timeout_s,
        grab_distance_mm=args.grab_distance_mm,
        grab_vertical_stack_pitch_m=grab_vertical_stack_pitch_m,
        place_distance_mm=args.place_distance_mm,
        grab_brake_margin_mm=args.grab_brake_margin_mm,
        place_brake_margin_mm=args.place_brake_margin_mm,
        grab_min_safe_mm=args.grab_min_safe_mm,
        place_min_safe_mm=args.place_min_safe_mm,
        grab_target_tolerance_mm=args.grab_target_tolerance_mm,
        grab_correction_speed_mps=args.grab_correction_speed_mps,
        grab_correction_max_passes=args.grab_correction_max_passes,
        grab_angle_correction_max_span_mm=args.grab_angle_correction_max_span_mm,
        grab_angle_correction_max_passes=args.grab_angle_correction_max_passes,
        grab_angle_correction_angular_speed_radps=args.grab_angle_correction_angular_speed_radps,
        grab_angle_correction_probe_s=args.grab_angle_correction_probe_s,
        grab_target_avg_accept_span_mm=args.grab_target_avg_accept_span_mm,
        place_target_tolerance_mm=args.place_target_tolerance_mm,
        place_correction_speed_mps=args.place_correction_speed_mps,
        place_correction_max_passes=args.place_correction_max_passes,
        place_retreat_front_target_mm=args.place_retreat_front_target_mm,
        place_retreat_target_tolerance_mm=args.place_retreat_target_tolerance_mm,
        place_retreat_forward_speed_mps=args.place_retreat_forward_speed_mps,
        place_retreat_forward_brake_margin_mm=args.place_retreat_forward_brake_margin_mm,
        place_retreat_forward_correction_speed_mps=args.place_retreat_forward_correction_speed_mps,
        place_retreat_correction_max_passes=args.place_retreat_correction_max_passes,
        rack_centering_mode=args.rack_centering_mode,
        rack_pose_samples=args.rack_pose_samples,
        rack_pose_interval_s=args.rack_pose_interval_s,
        rack_pose_min_range_m=args.rack_pose_min_range_m,
        rack_pose_max_range_m=args.rack_pose_max_range_m,
        rack_pose_lateral_half_width_m=args.rack_pose_lateral_half_width_m,
        rack_pose_z_min_m=args.rack_pose_z_min_m,
        rack_pose_z_max_m=args.rack_pose_z_max_m,
        rack_pose_bin_width_m=args.rack_pose_bin_width_m,
        rack_pose_min_cluster_points=args.rack_pose_min_cluster_points,
        rack_place_pose_min_range_m=args.rack_place_pose_min_range_m,
        rack_place_pose_max_range_m=args.rack_place_pose_max_range_m,
        rack_place_pose_lateral_half_width_m=args.rack_place_pose_lateral_half_width_m,
        rack_place_pose_z_min_m=args.rack_place_pose_z_min_m,
        rack_place_pose_z_max_m=args.rack_place_pose_z_max_m,
        rack_place_pose_bin_width_m=args.rack_place_pose_bin_width_m,
        rack_place_pose_min_cluster_points=args.rack_place_pose_min_cluster_points,
        rack_pose_warn_lateral_m=args.rack_pose_warn_lateral_m,
        rack_pose_warn_yaw_deg=args.rack_pose_warn_yaw_deg,
        rack_pose_min_confidence=args.rack_pose_min_confidence,
        rack_near_target_skip_centering_margin_mm=(
            args.rack_near_target_skip_centering_margin_mm
        ),
        rack_guarded_ultrasonic_override=args.rack_guarded_ultrasonic_override,
        rack_guarded_ultrasonic_max_pose_lateral_m=(
            args.rack_guarded_ultrasonic_max_pose_lateral_m
        ),
        rack_guarded_ultrasonic_max_front_span_mm=(
            args.rack_guarded_ultrasonic_max_front_span_mm
        ),
        rack_guarded_ultrasonic_min_front_mm=args.rack_guarded_ultrasonic_min_front_mm,
        rack_post_approach_guarded_check=args.rack_post_approach_guarded_check,
        rack_post_approach_max_lateral_m=args.rack_post_approach_max_lateral_m,
        rack_post_approach_max_yaw_deg=args.rack_post_approach_max_yaw_deg,
        rack_post_approach_max_front_span_mm=(
            args.rack_post_approach_max_front_span_mm
        ),
        rack_post_approach_front_retry_windows=(
            args.rack_post_approach_front_retry_windows
        ),
        front_too_close_safe_backoff_retries=args.front_too_close_safe_backoff_retries,
        rack_yaw_shadow_min_distance_m=args.rack_yaw_shadow_min_distance_m,
        rack_yaw_shadow_max_distance_m=args.rack_yaw_shadow_max_distance_m,
        rack_yaw_shadow_min_confidence=args.rack_yaw_shadow_min_confidence,
        rack_yaw_shadow_max_fit_residual_m=args.rack_yaw_shadow_max_fit_residual_m,
        rack_yaw_shadow_trigger_deg=args.rack_yaw_shadow_trigger_deg,
        rack_yaw_shadow_max_deg=args.rack_yaw_shadow_max_deg,
        rack_lateral_shadow_min_distance_m=args.rack_lateral_shadow_min_distance_m,
        rack_lateral_shadow_max_distance_m=args.rack_lateral_shadow_max_distance_m,
        rack_lateral_shadow_min_confidence=args.rack_lateral_shadow_min_confidence,
        rack_lateral_shadow_max_fit_residual_m=args.rack_lateral_shadow_max_fit_residual_m,
        rack_lateral_shadow_trigger_m=args.rack_lateral_shadow_trigger_m,
        rack_lateral_shadow_max_m=args.rack_lateral_shadow_max_m,
        rack_lateral_shadow_max_correction_m=args.rack_lateral_shadow_max_correction_m,
        rack_lateral_active_target_m=args.rack_lateral_active_target_m,
        rack_lateral_active_max_initial_m=args.rack_lateral_active_max_initial_m,
        rack_lateral_active_max_yaw_deg=args.rack_lateral_active_max_yaw_deg,
        rack_lateral_active_max_passes=args.rack_lateral_active_max_passes,
        rack_lateral_active_speed_mps=args.rack_lateral_active_speed_mps,
        rack_lateral_active_direction=args.rack_lateral_active_direction,
        rack_lateral_active_step_s=args.rack_lateral_active_step_s,
        rack_lateral_active_settle_s=args.rack_lateral_active_settle_s,
        rack_lateral_active_min_improvement_m=args.rack_lateral_active_min_improvement_m,
        rack_lateral_active_min_front_mm=args.rack_lateral_active_min_front_mm,
        rack_lateral_active_min_rear_mm=args.rack_lateral_active_min_rear_mm,
        rack_lateral_active_min_side_mm=args.rack_lateral_active_min_side_mm,
        rack_lateral_active_hard_min_front_mm=args.rack_lateral_active_hard_min_front_mm,
        rack_lateral_active_hard_min_rear_mm=args.rack_lateral_active_hard_min_rear_mm,
        rack_lateral_active_hard_min_side_mm=args.rack_lateral_active_hard_min_side_mm,
        rack_lateral_active_clearance_samples=args.rack_lateral_active_clearance_samples,
        rack_lateral_active_clearance_interval_s=args.rack_lateral_active_clearance_interval_s,
        rack_lateral_active_hz=args.rack_lateral_active_hz,
        rack_lateral_active_max_sample_span_m=args.rack_lateral_active_max_sample_span_m,
        rack_lateral_active_rollback_on_worse=args.rack_lateral_active_rollback_on_worse,
        rack_lateral_active_rollback_step_scale=args.rack_lateral_active_rollback_step_scale,
        coarse_speed_mps=args.coarse_speed_mps,
        grab_approach_speed_mps=args.grab_approach_speed_mps,
        place_approach_speed_mps=args.place_approach_speed_mps,
        retreat_distance_m=args.retreat_distance_m,
        retreat_target_tolerance_mm=args.retreat_target_tolerance_mm,
        retreat_speed_mps=args.retreat_speed_mps,
        retreat_method=args.retreat_method,
        retreat_escape_delta_m=args.retreat_escape_delta_m,
        grab_retreat_front_occlusion_escape_threshold_mm=args.grab_retreat_front_occlusion_escape_threshold_mm,
        grab_retreat_front_occlusion_escape_m=args.grab_retreat_front_occlusion_escape_m,
        grab_retreat_front_occlusion_escape_speed_mps=args.grab_retreat_front_occlusion_escape_speed_mps,
        retreat_front_delta_consistency_mm=args.retreat_front_delta_consistency_mm,
        retreat_odom_tolerance_m=args.retreat_odom_tolerance_m,
        retreat_require_odom_crosscheck=args.retreat_require_odom_crosscheck,
        retreat_odom_auto_correction=args.retreat_odom_auto_correction,
        retreat_odom_auto_correction_max_m=args.retreat_odom_auto_correction_max_m,
        retreat_odom_auto_correction_min_m=args.retreat_odom_auto_correction_min_m,
        retreat_odom_auto_correction_speed_mps=args.retreat_odom_auto_correction_speed_mps,
        retreat_odom_auto_correction_max_passes=args.retreat_odom_auto_correction_max_passes,
        retreat_odom_auto_correction_front_hard_min_mm=(
            args.retreat_odom_auto_correction_front_hard_min_mm
        ),
        retreat_odom_auto_correction_rear_hard_min_mm=(
            args.retreat_odom_auto_correction_rear_hard_min_mm
        ),
        retreat_odom_auto_correction_clearance_retry_s=(
            args.retreat_odom_auto_correction_clearance_retry_s
        ),
        retreat_open_loop_brake_compensation_m=args.retreat_open_loop_brake_compensation_m,
        turn_angular_speed_radps=args.turn_angular_speed_radps,
        right_turn_duration_s=args.right_turn_duration_s,
        left_turn_duration_s=args.left_turn_duration_s,
        turn_hz=args.turn_hz,
        turn_control_mode=args.turn_control_mode,
        turn_method=args.turn_method,
        turn_success_states=turn_success_states,
        turn_min_sensor_delta_mm=args.turn_min_sensor_delta_mm,
        turn_yaw_tolerance_deg=args.turn_yaw_tolerance_deg,
        turn_confirm_samples=args.turn_confirm_samples,
        turn_confirm_interval_s=args.turn_confirm_interval_s,
        turn_confirm_max_span_deg=args.turn_confirm_max_span_deg,
        turn_correction_enabled=not args.disable_turn_correction,
        turn_correction_max_passes=args.turn_correction_max_passes,
        turn_correction_angular_speed_radps=args.turn_correction_angular_speed_radps,
        turn_correction_max_error_deg=args.turn_correction_max_error_deg,
        allow_turn_motion_error_2=args.allow_turn_motion_error_2,
        turn_validation_ok=args.turn_validation_ok,
        resume_after_grab_pull_index=args.resume_after_grab_pull_index,
        resume_after_grab_retreat_index=args.resume_after_grab_retreat_index,
        resume_after_place_above_index=args.resume_after_place_above_index,
        resume_after_place_pull_index=args.resume_after_place_pull_index,
        resume_after_place_retreat_target_index=args.resume_after_place_retreat_target_index,
        log_file=log_file,
        event_file=event_file,
        checkpoint_file=checkpoint_file,
        report_file=report_file,
    )


def main():
    config = parse_args()
    controller = Industrial7RodsController(config)
    try:
        controller.run()
    except Exception as exc:
        controller.fail_current_step(exc)
        controller.write_final_report("failed", exc)
        controller.log(f"controller_failed: {type(exc).__name__}: {exc}")
        raise
    else:
        controller.write_final_report("completed")


if __name__ == "__main__":
    main()
