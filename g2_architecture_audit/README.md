# G2 Architecture Audit

这组文档用于沉淀这台 G2 机器人的底层架构、硬件控制链路、通信服务和诊断方法。

当前状态：

- 目标机器人：`agi@10.20.15.152`
- 机器人主机名：`G2`
- 已现场验证的应用版本：`genie_g02_rb_2.2.0_320dcc1f_2026-04-01-09-31-20_r1.tar.gz`
- 已确认内核：`Linux 5.10.220-rt112 aarch64`
- 已确认 GDK 包版本：`agibot_gdk 2.6.3`
- 审计期间本机到 `10.20.15.152` 曾短暂 ping 全丢包；后续已恢复，SSH 复核完成。
- 已修复：PTP 设备权限，`agi` 现在可以打开 `/dev/ptp_xgi0`。
- 已补丁待继续复核：`/home/agi/app/bin/run.sh` 已加入动态选择 `LOCATOR_IP` 逻辑；但 2026-06-04 15:17 新启动后运行态仍选择 `10.42.1.101`，需继续查 boot 日志里的选择路径。

文档索引：

- [G2_system_architecture_handbook.md](G2_system_architecture_handbook.md): 系统启动链路、进程职责、通信服务、硬件控制链路、源码/配置边界。
- [G2_build_level_architecture_20260604.md](G2_build_level_architecture_20260604.md): 面向“自己造出来”的构建级架构、硬件控制分层、复刻缺口和下一轮逆向计划。
- [G2_expert_context.md](G2_expert_context.md): 下次会话优先读取的 G2 专家上下文入口，包含当前基线、排障入口和沉淀规则。
- [G2_expert_mastery_plan.md](G2_expert_mastery_plan.md): 专家级掌握覆盖度矩阵、补全任务和长期维护标准。
- [G2_hardware_control_matrix_20260604.md](G2_hardware_control_matrix_20260604.md): 当前已确认的 EtherCAT、CAN-FD、底盘、传感器、电源/MCU/时间同步控制矩阵。
- [G2_runtime_snapshot_20260604.md](G2_runtime_snapshot_20260604.md): 2026-06-04 现场运行态快照，包含 systemd、进程、网络、日志和重定位状态。
- [G2_topic_service_map.md](G2_topic_service_map.md): 模块间 topic/service 映射和消息类型风险点。
- [G2_diagnostic_runbook.md](G2_diagnostic_runbook.md): 常用只读诊断命令、故障定位路径和安全验证顺序。
- [scripts/g2_quick_snapshot.sh](scripts/g2_quick_snapshot.sh): 只读采集现场快照的脚本，用于故障开始时快速保存 systemd、进程、网络、日志、总线和 topic 证据。

证据等级：

- `现场已验证`: 来自本次 SSH 到机器人后的实时命令、systemd、进程、日志、设备状态或此前同机现场修复记录。
- `本地资料印证`: 来自 `/home/davie/G2` 下的 GDK 文档、Python 示例和既有 runbook。
- `待复核`: 需要重启、运动或更高风险现场验证的项，本轮不主动触发。
