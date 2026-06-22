#!/usr/bin/env python3
"""
七根料一键总控入口。

运行位置：
  放在 BOX_528_1 根目录，和这些动作脚本在同一级：
    move_ee_pose_open_2.py
    move_ee_pose_close_2.py
    move_arm_by_json_grab_above_第一根.py ... move_arm_by_json_grab_above_第七根.py
    offset_move_pull.py
    offset_move_down.py

运行前加载机器人环境：
  cd /data/btgys/bengtian_backup_20260608_081250/wxf/BOX_528_1
  source /home/agi/app/env.sh

先做无运动检查，只打印完整流程：
  python3 industrial_7_rods_one_click.py --dry-run

确认现场安全后，先只执行第 1 根：
  python3 industrial_7_rods_one_click.py --confirm-live --start-index 1 --end-index 1

90 度转向默认使用 chassis_demo.py 同源的 ChassisController.rotate(±90)：
  python3 industrial_7_rods_one_click.py --confirm-live --turn-method relative

如果转向返回 state=7，总控会停在转向步骤，不继续放料或抓下一根。
state=7 是取消/结束，不能证明机器人已经转到 90 度。

只有诊断/应急时才使用速度开环转向：
  python3 industrial_7_rods_one_click.py --confirm-live --turn-method velocity

速度开环时，如果右转/左转欠角，才调整这些参数：
  python3 industrial_7_rods_one_click.py --confirm-live --turn-method velocity --right-turn-duration-s 5.0
  python3 industrial_7_rods_one_click.py --confirm-live --turn-method velocity --left-turn-duration-s 5.5

连续多根真实运行前必须先单独多次通过左右转诊断，然后显式确认：
  python3 industrial_turn_diagnostic.py --confirm-live --direction right --method relative --repeat 3
  python3 industrial_turn_diagnostic.py --confirm-live --direction left  --method relative --repeat 3
  python3 industrial_7_rods_one_click.py --confirm-live --start-index 1 --end-index 7 --turn-validation-ok

如果流程已经抓住并拉出某一根，停在“后退去放料”之前，例如第 1 根：
  python3 industrial_7_rods_one_click.py --confirm-live --resume-after-grab-pull-index 1

如果流程已经完成抓取后的后退，停在“右转去放料”之前，例如第 1 根：
  python3 industrial_7_rods_one_click.py --confirm-live --resume-after-grab-retreat-index 1

如果流程已经右转并移动到放料上方，停在“前雷达到 500mm”之前，例如第 2 根：
  python3 industrial_7_rods_one_click.py --confirm-live --resume-after-place-above-index 2

说明：
  - 这个入口不包含动作逻辑，真正的工业流程在
    rack_hybrid_docking_package/industrial_7_rods_total_controller.py。
  - 保持入口很薄，是为了避免两份流程代码不一致。
"""

from pathlib import Path
import sys


def main():
    base_dir = Path(__file__).resolve().parent
    package_dir = base_dir / "rack_hybrid_docking_package"
    if str(package_dir) not in sys.path:
        sys.path.insert(0, str(package_dir))

    from industrial_7_rods_total_controller import main as controller_main

    controller_main()


if __name__ == "__main__":
    main()
