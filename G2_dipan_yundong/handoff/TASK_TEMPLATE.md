# 任务交接模板（Claude 写 / Codex 填执行结果）

> 复制下面整块到 `handoff/HANDOFF.md`（或 `handoff/tasks/TASK-NNN.md`）。
> Claude 填「目标 → 约束」，Codex 只填「执行结果」区。

---

## TASK-NNN  [STATUS: PENDING]

### 目标（一句话，描述"做完长什么样"）
<例：把 control_v90.py 里五个冲突的 grab-gate 函数合并成单一 evaluate_target()，行为不变>

### 涉及文件（确切路径）
- <例：robot/control_v90.py>

### 步骤（确切命令 / 改动，越具体越好）
1. <例：新建 evaluate_target(target)，整合现有 5 个 gate 的判定逻辑>
2. <例：把所有调用点替换为 evaluate_target()>
3. <例：删除旧的 5 个函数>

### 验收标准（客观、机器可判定，全部满足才算 DONE）
- [ ] `pytest tests/test_control_snapshot.py` 全部通过（行为不变）
- [ ] `pyright robot/control_v90.py` 无报错
- [ ] `git diff` 仅触及 robot/control_v90.py
- [ ] 文件中不再存在旧的 5 个 gate 函数名

### 约束
- 纯软件改动，**不触发任何硬件动作 / CAN 写入**
- 不超出本任务范围新增改动
- 命令行不得含中文标点

### 是否需要 ACK 回显
- [x] 是（非平凡，先回显理解等 Claude 确认）   /   [ ] 否（琐碎可逆，直接执行）

---

### 执行结果（仅 Codex 填写）

- **status**: DONE / FAILED / BLOCKED
- **实际执行的命令**:
  ```
  <逐条贴出>
  ```
- **git commit SHA**: <例：a1b2c3d>
- **测试 / pyright 输出摘要**:
  ```
  <贴关键行，尤其是通过/失败计数>
  ```
- **偏离计划之处**（没有就写"无"）:
  <任何与上面步骤不一致的地方，必须如实写>
- **若 FAILED/BLOCKED**：哪条验收标准没过 / 卡在哪，原因是什么：
  <...>

---
> Claude 复核：读 `git diff <SHA>` 和测试输出本身，对照验收标准判定，**不依据上面的文字摘要**。
