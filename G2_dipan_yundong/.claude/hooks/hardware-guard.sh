#!/bin/bash
# hardware-guard.sh — 硬件安全硬护栏
# 拦截任何会驱动 G2 硬件 / 不可逆的命令，强制由 David 手动执行。
#
# 机制：作为 PreToolUse hook，在权限检查之前运行（即使开了
# --dangerously-skip-permissions 也照拦）。命中即 exit 2 阻断，
# 并把原因喂回给 Claude，让它把命令交给人执行而不是自己跑。
#
# 安装：放在 <项目根>/.claude/hooks/hardware-guard.sh 并 chmod +x
# 依赖：jq（Ubuntu: sudo apt install jq）

INPUT=$(cat)
CMD=$(echo "$INPUT" | jq -r '.tool_input.command // empty')

# ============ 可编辑：会驱动硬件 / 不可逆的命令关键字 ============
# 按你 G2 的实际 CLI 调整。命中任意一条即拦截。
# 大小写不敏感，按扩展正则匹配。
BLOCKED_PATTERNS=(
  "control_v[0-9]"     # 主控制脚本（会驱动机械臂），如 control_v90.py
  "send_joint"         # 关节指令下发
  "set_joint"
  "move_to"
  "move_joint"
  "goto_pose"
  "can_send"           # CAN 总线写入
  "cansend"
  "estop"              # estop 相关
  "gripper"            # 夹爪动作
  "actuate"
)
# =================================================================

for p in "${BLOCKED_PATTERNS[@]}"; do
  if echo "$CMD" | grep -qiE "$p"; then
    echo "⛔ 硬件安全拦截：命令命中关键字 '$p'。" >&2
    echo "按 AGENTS.md 第 3 节，涉及机械臂运动 / CAN 写入的命令不得自动执行。" >&2
    echo "请把下面这条确切命令原样交给 David 手动执行，不要自行运行：" >&2
    echo "    $CMD" >&2
    exit 2
  fi
done

exit 0
