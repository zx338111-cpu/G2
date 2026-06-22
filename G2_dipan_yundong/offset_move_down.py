import agibot_gdk
import time
from end_effector_controller import EndEffectorController


# ═══════════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════════

def main():
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("GDK 初始化失败")
        return
    print("GDK 初始化成功")

    robot = agibot_gdk.Robot()
    time.sleep(2)   # 等待机器人就绪

    try:
        controller = EndEffectorController(robot)
        
        # ───────────────────────────────────────────────────────
        # 使用说明：
        # offset 参数格式为 (X偏移, Y偏移, Z偏移)，单位为 米。
        # 坐标系规则： X+(向前)， Y+(向左)， Z+(向上)
        # ───────────────────────────────────────────────────────

        # 示例 1：仅仅让左臂向左移动 50mm (Y+方向)，右臂保持不动
        # controller.adjust_arms_relative(offset_l=(0, 0.05, 0), offset_r=(0, 0, 0))

        # 示例 2：仅仅让右臂向下移动 50mm (Z-方向)，左臂保持不动
        # controller.adjust_arms_relative(offset_l=(0, 0, 0), offset_r=(0, 0, -0.05))
        
        # 示例 3：左臂向左 (Y+) 50mm，右臂向右 (Y-) 50mm，同时执行
        # controller.adjust_arms_relative(offset_l=(0, -0.01, 0), offset_r=(0,    0.01, 0))
        # controller.adjust_arms_relative(offset_l=(0, 0, -0.01   ), offset_r=(0,    0, -0.01))
        
        # 放料时下移总量改为 60mm：
        # 原来是 -0.03m；现场确认还需要再往下 3cm，所以改成 -0.06m。
        controller.adjust_arms_relative(offset_l=(0, 0, -0.06), offset_r=(0, 0, -0.06))
        # 示例 4：双臂同时向前 (X+) 伸出 50mm
        # controller.adjust_arms_relative(offset_l=(0.05, 0, 0), offset_r=(0.05, 0, 0))

    except Exception as e:
        print(f"[运行错误] {e}")

    # 释放GDK系统资源
    if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
        print("GDK释放失败")
    else:
        print("GDK释放成功")

if __name__ == "__main__":
    main()
