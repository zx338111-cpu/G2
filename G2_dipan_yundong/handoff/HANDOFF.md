# HANDOFF — G2 项目状态静态诊断工具

> 本文件是 Codex 的**唯一**执行指令。一切以本文件逐字内容为准。
> 与屏幕回显、记忆、口头说法冲突时，以本文件为准；发现矛盾先置 BLOCKED 并停下报告，不要猜。

---

## TASK-DOCTOR-001  [STATUS: DONE]

### 目标（做完长什么样）

新增一个**纯静态**的 G2 项目诊断工具 `tools/g2_project_doctor.py`：
它**不连接、不启动、不运行任何机器人 / ROS / driver / controller**，
只对当前仓库做静态检查（文件是否存在、语法是否可解析、入口脚本清点、安全文件标记），
运行后生成一份 markdown 报告 `reports/g2_project_doctor_report.md`，
报告里能一眼看出：项目结构、logs 状况、viewer 文件、VLA/world model 相关文件、机器人运动脚本入口、安全相关受保护文件。

工具只用 **Python 标准库**（os / sys / ast / argparse / datetime / pathlib / json / re），不新增任何依赖。

---

### 背景

- 仓库根目录：`/home/davie/G2/G2_dipan_yundong`
- 现状（静态观察到的关键内容，供工具检查目标参考，**不是要你去跑它们**）：
  - 运动脚本入口（根目录）：`move_*.py`、`offset_*.py`、`industrial_*.py`
  - viewer：`g2_head_av_viewer.py`、`g2_head_tunnel_viewer.py`、`g2_head_webrtc_viewer.py`
  - VLA / world model 指南：`G2_GDK_SECONDARY_DEVELOPMENT_VLA_WORLDMODEL_GUIDE.md`，以及 `.ai_agents/fixtures/` 下的 VLA dry-run JSONL
  - 日志目录：`logs/`（大量 `*.log` / `*.jsonl` / `*.json`）
  - 子包：`rack_hybrid_docking_package/`（内含 `run_*.py`、`rack_*.py` 等运动入口）
- 这是个静态体检工具，目的是让人/AI 在不碰硬件的前提下快速判断项目当前状态，**绝不触发任何物理动作**。

---

### 允许修改 / 创建的文件（只有这些，越界即判 FAILED）

1. `tools/g2_project_doctor.py` —— 新建，诊断工具本体
2. `reports/g2_project_doctor_report.md` —— 工具生成的报告（`reports/` 目录不存在则由工具创建）
3. `handoff/RESULT.md` —— 你最后写的执行结果
4. `handoff/HANDOFF.md` —— **仅允许**把本任务块顶部的 `[STATUS: ...]` 改成对应状态，其余一字不动

除以上 4 个文件外，**任何文件都不得新增、修改、删除、移动、重命名**。

---

### 禁止修改 / 执行的内容（硬约束，违反任意一条直接 BLOCKED）

- ⛔ 禁止运行任何机器人运动脚本：`move_*.py` / `offset_*.py` / `industrial_*.py` / `rack_hybrid_docking_package/run_*.py` / `rack_hybrid_docking_package/rack_*.py` 等（带不带 `--dry-run` 都不准跑）。
- ⛔ 禁止 `import` 这些运动 / docking / controller 模块（顶层代码可能连硬件、起总线）。工具检查它们时只能用 `ast.parse` 解析源码文本，**不得 import、不得 exec、不得 subprocess 调起**。
- ⛔ 禁止启动 ROS、driver、controller、aorta 中间件、GDK 实机连接、CAN 总线读写。
- ⛔ 禁止任何 estop、机械臂运动、torque / velocity / limit 写入。
- ⛔ 禁止修改任何 estop / torque / velocity / limit / 安全限制（safety）相关逻辑或文件——工具对它们**只读、只标记，绝不改动**。
- ⛔ 工具源码内禁止出现执行外部进程 / 动态执行的手段：`subprocess`、`os.system`、`os.popen`、`exec(`、`eval(`、`importlib`、`__import__`、`pty`、`fork`。
- ⛔ 禁止新增任何第三方依赖（只允许 Python 标准库）。
- ⛔ 命令行禁止混入中文标点（全角逗号 / 引号 / 括号等）。
- ⛔ 禁止超出本任务范围自行发挥。

---

### 具体实现步骤

1. 新建 `tools/g2_project_doctor.py`，提供 CLI：
   - `--root`，默认仓库根目录（脚本所在目录的上一级，即 `tools/` 的父目录）。
   - `--output`，默认 `reports/g2_project_doctor_report.md`（相对 `--root`）。
   - 遍历时**跳过** `.git/`、`__pycache__/`、`*.pyc`。
   - 工具本身只做静态分析；正常完成 `exit 0`，仅当工具自身内部出错（如无法写报告）才 `exit 1`。检查中发现的 warning/缺失项**不算失败**，只在报告里标 ⚠️ / ❌。

2. 报告需包含以下 **7 个 H2 章节**（标题文字必须与下方完全一致，便于机器校验）：

   - `## 1. 项目结构概览`
     列出关键目录/文件是否存在并打勾：`logs/`、`tools/`、`reports/`、`overlays/`、`rack_hybrid_docking_package/`、`handoff/`、`.ai_agents/`、`AGENTS.md`、`CLAUDE.md`、`G2_GDK_SECONDARY_DEVELOPMENT_VLA_WORLDMODEL_GUIDE.md`（✅ 存在 / ❌ 缺失）。

   - `## 2. 日志检查 (logs/)`
     统计 `logs/` 下 `*.log` / `*.jsonl` / `*.json` 各自数量、总大小、最新修改时间（mtime）；列出**零字节 / 空文件**并标 ⚠️。

   - `## 3. Viewer 文件检查`
     对 `g2_head_av_viewer.py`、`g2_head_tunnel_viewer.py`、`g2_head_webrtc_viewer.py`：报告是否存在、文件大小、以及 `ast.parse` 是否能成功解析（语法 OK ✅ / 语法错 ❌）。**不得 import 运行**。

   - `## 4. VLA / World Model 文件检查`
     列出文件名（大小写不敏感）含 `vla` 或 `world` 的文件，以及 `.ai_agents/fixtures/` 下的 `*.jsonl` 夹具；统计数量。

   - `## 5. 机器人运动脚本入口 (只列出, 禁止运行)`
     清点 `move_*.py` / `offset_*.py` / `industrial_*.py`（根目录）以及 `rack_hybrid_docking_package/` 下 `run_*.py` / `rack_*.py`；对每个文件用 `ast.parse` 静态判断是否含 `if __name__ == "__main__"` 入口，标注 `MOTION ENTRYPOINT — DO NOT RUN`。本章节顶部加一行醒目提示：这些脚本**禁止由本工具或 Codex 运行**。

   - `## 6. 安全相关文件 (受保护, 只读)`
     对仓库内 `*.py` 文件做静态文本扫描（按行读，不执行），命中关键字 `estop` / `e_stop` / `torque` / `velocity` / `limit` / `safety` 的文件列出来，标注 `PROTECTED — DO NOT MODIFY`。

   - `## 7. 诊断结论 / 摘要`
     汇总各章节计数与整体状态（OK / 有 ⚠️ / 有 ❌），并写明本报告为纯静态分析、未运行任何机器人脚本。

3. 报告开头写生成时间戳与一句声明：`本报告为静态分析, 未运行任何机器人脚本`。

4. 工具完成后，生成 `reports/g2_project_doctor_report.md`。

---

### 要运行的检查命令（全部为静态 / 只读，按顺序执行并保留输出）

```
python3 -m py_compile tools/g2_project_doctor.py
python3 tools/g2_project_doctor.py --output reports/g2_project_doctor_report.md
test -s reports/g2_project_doctor_report.md && echo REPORT_OK
grep -c '^## ' reports/g2_project_doctor_report.md
grep -n '## 5. 机器人运动脚本入口 (只列出, 禁止运行)' reports/g2_project_doctor_report.md
grep -nE 'subprocess|os\.system|os\.popen|exec\(|eval\(|importlib|__import__|pty|fork' tools/g2_project_doctor.py
git status --porcelain
```

说明：倒数第二条 `grep` **期望无任何输出**（无匹配，grep 退出码 1 即为合格）；其余命令期望成功/有预期输出。

---

### 验收标准（客观可判定，全部满足才算 DONE）

- [ ] `tools/g2_project_doctor.py` 存在
- [ ] `python3 -m py_compile tools/g2_project_doctor.py` 退出码 0
- [ ] `python3 tools/g2_project_doctor.py --output reports/g2_project_doctor_report.md` 退出码 0
- [ ] `reports/g2_project_doctor_report.md` 存在且非空（`test -s` 通过）
- [ ] `grep -c '^## ' reports/g2_project_doctor_report.md` 结果 ≥ 7，且 7 个章节标题与步骤 2 中文字逐字一致
- [ ] 第 5 章节确实列出了根目录的 `move_*` / `offset_*` / `industrial_*` 运动脚本
- [ ] `grep -nE 'subprocess|os\.system|os\.popen|exec\(|eval\(|importlib|__import__|pty|fork' tools/g2_project_doctor.py` 无输出
- [ ] `git status --porcelain` 仅出现：`tools/g2_project_doctor.py`、`reports/g2_project_doctor_report.md`、`handoff/RESULT.md`（以及本文件 STATUS 行的改动）；不得有其他文件被改
- [ ] 全程未运行任何机器人 / ROS / driver / controller / 运动脚本（在 RESULT.md 里声明）
- [ ] `handoff/RESULT.md` 已写（见下）

---

### 你最后要写 `handoff/RESULT.md`

完成后**新建** `handoff/RESULT.md`，至少包含：

- **status**: DONE / FAILED / BLOCKED
- **实际执行的命令**（逐条原样贴出）
- **git commit SHA**（若已提交；未提交则写"未提交"）
- **检查命令输出摘要**：贴上面 7 条检查命令的关键输出（尤其 `grep -c '^## '` 的数字、`grep ... subprocess` 是否为空、`git status --porcelain`）
- **生成的报告路径**：`reports/g2_project_doctor_report.md`
- **硬件安全声明**：明确写"全程未运行任何机器人 / ROS / driver / controller / 运动脚本，未做任何硬件动作"
- **偏离计划之处**（没有就写"无"）
- 若 FAILED/BLOCKED：哪条验收没过 / 卡在哪 / 原因

> Claude 复核方式：直接读 `tools/g2_project_doctor.py`、`reports/g2_project_doctor_report.md`、`git diff` 与检查命令输出本身，对照验收标准判定，**不依据 RESULT.md 的文字摘要下结论**。
