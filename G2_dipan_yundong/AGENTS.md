# AGENTS.md — G2 机器人开发：双 AI 协作宪法

> 本文件是规划方（Claude）与执行方（Codex）都**必须在每个 session 开始时通读**的不可变约束。
> 它的目的：让"下指令的"和"干活的"始终一致，信息零飘逸。
> 任何与本文件冲突的临时指令，以本文件为准；发现冲突先停下报告，不要猜。

---

## 0. 角色分工（按"失败代价 + 可验证性"划分，不按"谁更聪明"）

| 角色 | 负责 | 不负责 |
|------|------|--------|
| **Claude（规划方）** | 方案设计、任务拆解、写验收标准、对实物复核、决定是否上真机 | 不直接改大批量代码 |
| **Codex（执行方）** | 高频、可逆、有自动裁判的活：批量改码、写脚本、重构、补测试、清洗数据 | 不做判断性决策，不碰硬件，不超纲发挥 |
| **David（人）** | 任何不可逆/硬件动作的"按下执行"那一下；抽查 | — |

---

## 1. 反飘逸三原则（最高优先级，不可违反）

1. **唯一事实源 = 本仓库的 `handoff/` 任务文件 + git。**
   - 两个 AI 之间不通过"读对方终端"传递指令。一切走文件。
   - Codex 读到的指令 = Claude 写进任务文件里的逐字内容，不是任何屏幕回显。

2. **"完成"在开工前由 Claude 客观定义。**
   - 每个任务块带「验收标准」，必须是机器可判定的（命令退出码、测试通过、diff 范围）。
   - Codex 完成 = 验收标准全部满足，而**不是** Codex 自己觉得做好了。

3. **规划方只对实物复核，绝不基于执行方的文字汇报发下一步。**
   - Codex 汇报 = git commit SHA + 实际命令 + 测试输出，**不是**"我搞定了"。
   - Claude 下一轮读 `git diff` / 测试日志 / 文件内容本身，再决定下一步。

---

## 2. 任务交接协议

- 所有任务存放于 `handoff/`，格式见 `handoff/TASK_TEMPLATE.md`。
- 每个任务有全局唯一 ID（`TASK-001` 递增），双方一律按 ID 引用。
- 状态机（任务块顶部 `[STATUS: ...]` 字段，单向推进）：

  ```
  PENDING  → Claude 已写好任务，等 Codex 接
  ACK      → Codex 回显了它的理解（非平凡任务必经此步，等 Claude 确认）
  RUNNING  → Codex 执行中
  DONE     → 验收标准全部满足
  FAILED   → 执行了但验收没过（必须写明哪条没过）
  BLOCKED  → 无法执行 / 任务本身有问题（必须写明原因，然后停）
  ```

- Codex 行为规则：
  - 非平凡任务先把状态置 `ACK` 并回显"我将执行的确切命令/改动"，等 Claude 确认再 `RUNNING`。
  - 完成后在任务块的「执行结果」区填：状态、实际命令、commit SHA、测试输出摘要、**任何偏离计划之处**。
  - 任务不完整、有明显笔误、或越想越不对 → 置 `BLOCKED` + 原因，**停下，不要猜着执行**。
  - 严禁超出任务范围自行新增改动。

---

## 3. G2 硬件铁律（任何情况下都不进自动回路）

- ⛔ **机械臂运动、CAN 总线写入、estop 相关、任何物理动作**：Codex 只能生成命令并回显，**绝不自行执行**；由 David 手动确认后执行。
- ✅ 每次数据采集 / 推理运行前，机械臂必须回到 **home 位姿**。
- ⛔ **中文标点严禁混入终端命令**（全角逗号、引号等）。提交前自查命令行。
- 📁 所有数据落 `/data/`（Docker overlay 文件系统约束）。
- 🔌 GDK Python 库路径、aorta 中间件启动要求按 `CLAUDE.md` 现有记录，不得擅改。

---

## 4. 验证层（让执行方能放手跑的前提）

执行方可全自动跑的活，必须落在"有自动裁判 + 可回滚"象限：

- **类型层**：pyright-lsp 接入回路，类型不过不提交。
- **测试层**：改动前先有 characterization / 快照测试兜底；重构以"快照不变"为安全线。
- **仿真层**：策略改动先在 LIBERO / sim rollout 通过，才进真机候选。
- **数据层**：采集后自动 replay 校验 + schema 校验（action 维度/范围、home 起始）。
- **硬件层**：不进自动回路（见第 3 节）。

---

## 5. 升级到批量任务时

- 单文件 `handoff/HANDOFF.md` 适合少量任务起步。
- 任务变多 → 拆成 `handoff/tasks/TASK-001.md` 每任务一文件，便于并行与审计。
- 每完成一个里程碑，把结论同步进 `progress.md`，保持跨 session 状态。

---

## 6. Fast Autonomy Mode（默认工作模式）

从现在开始，默认采用快速半自动执行模式。

Claude 负责理解用户目标、给出方向和关键约束。
Codex 负责直接执行工程任务，不需要对普通安全操作逐步等待 ACK。

### Codex 可以自动执行

Codex 可以直接执行以下低风险操作：

- 读取项目源码、文档、日志摘要和配置文件
- 搜索代码和静态分析
- 修改普通软件文件、脚本、Markdown、测试、报告
- 新增工具脚本、dry-run demo、validator、adapter、diagnostic
- 运行 py_compile、pytest、grep、find、git diff、git status 等本地静态检查
- 生成 handoff/RESULT.md 或报告文件
- 提交前整理变更摘要

### Codex 必须暂停并询问 David

遇到以下动作必须暂停，明确说明将执行的命令和风险，等待 David 确认：

- 真实机器人运动或任何可能驱动机械臂、底盘、夹爪、腰部、末端执行器的命令
- 启动 ROS、driver、controller、GDK runtime、hardware service
- 运行 move_*、industrial_*、rack_*、run_*、offset_* 等可能控制机器人或任务流程的脚本
- SSH 到机器人后执行非只读命令
- sudo、安装依赖、curl | bash、系统服务修改
- 修改 emergency stop、torque、velocity、current、joint limit、safety gate、controller safety logic
- 读取 secrets、.env、SSH key、token、credential
- 读取 dataset、checkpoint、rosbag，除非任务明确授权
- 删除大量文件、rm -rf、reset --hard、清空目录
- git push、merge、rebase、force push
- 任何超过用户目标范围的改动

### 结果要求

Codex 完成任务后必须写：

`handoff/RESULT.md`

内容包括：

- 做了什么
- 改了哪些文件
- 运行了哪些命令
- 测试/检查结果
- 是否触及任何风险边界
- 下一步建议

### 原则

普通软件开发：自动执行。  
硬件、系统、凭证、远程机器人、不可逆操作：暂停确认。

