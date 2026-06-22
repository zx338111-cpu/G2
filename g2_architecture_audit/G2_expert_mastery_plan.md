# G2 专家级掌握计划

日期：2026-06-04  
目标：把 G2 知识库维护到下次遇到问题时可以快速定位、修复、验证。

## 1. 覆盖度矩阵

| 领域 | 当前掌握度 | 已有证据 | 下一步 |
|---|---:|---|---|
| 启动链路 | 85% | `genie_app.service`, `run.sh`, `base.json`, launcher 进程 | 抽取 boot 日志中的网络选择路径 |
| 进程职责 | 80% | ps/systemd/manifest/logs | 给每个进程补 owner、输入、输出、失败症状 |
| AORTA/FastDDS/Cosine | 70% | 端口、topic、queue、IP 绑定 | 整理 topic type、publisher/subscriber 完整表 |
| GDK API | 60% | GDK 环境、Robot/Pnc/Slam 常用检查 | 建 API -> service/topic -> 故障入口矩阵 |
| SLAM/重定位 | 75% | localization.yaml、队列修复、GICP 分数 | 梳理地图数据库、map.pcd、grid_map 关系 |
| PNC/导航 | 70% | quark_navigation、BT、costmap、controller | 抽取行为树和 controller 参数关键项 |
| MotionControl/WBC | 75% | G2_t2_crs profile、分组、URDF/SRDF | 抽取限位、质量、惯量、碰撞体、工具 TCP |
| HAL 上半身 | 80% | tianji、EtherCAT 18 slave、关节映射 | 建 slave -> joint -> config -> statusword 表 |
| hal_lowerlimb 底盘 | 75% | ecat1、四轮配置、电源板版本 | 建底盘板/四轮 PDO/状态字段表 |
| CAN-FD 头腰 | 70% | can0/can1、node_id、JXZN 型号 | 查协议/帧格式/故障码/固件版本 |
| 传感器与标定 | 65% | MID360、xt_chest、/data/parameters/sensor | 生成内外参总表和标定流程 |
| 电源/安全 | 50% | power_manager、fault_manager、急停字段 | 梳理电池/BMS/急停/充电/电源板链路 |
| 机械复刻 | 45% | URDF/SRDF/MJCF、STL mesh | 需要 CAD/BOM/加工/装配资料 |
| 电气复刻 | 35% | 设备节点、总线、板卡版本 | 需要原理图、线束图、固件、ESI |

## 2. 专家级排障闭环

每个问题都要形成六件事：

```text
1. 现象: 用户看到什么，机器人当前状态是什么
2. 证据: 最新日志、进程、总线、API 状态
3. 定位: 问题在哪一层，为什么不是其他层
4. 修复: 改了什么，备份在哪里，如何回滚
5. 验证: 进程/通信/API/日志/硬件动作是否闭环
6. 沉淀: 更新哪个项目文件，是否需要写入记忆扩展
```

## 3. 近期补全任务

优先级 P0：

- 从 `/data/logs/latest` 或 boot 目录确认 2026-06-04 15:17 启动时 `run.sh` 为什么选 `10.42.1.101`。
- 把本次抽取的 topic/service 列表补进 `G2_topic_service_map.md`，按模块而不是纯字母序组织。
- 把 EtherCAT 18 从站映射成表：slave、关节、配置项、状态字、错误码、故障清除方式。

优先级 P1：

- 抽取 `G2_t2_crs.urdf` 的 joint limit、mass、inertia、collision，建立机械复刻表。
- 抽取 `/data/parameters/sensor` 的所有内参、外参、结构参数，建立传感器标定总表。
- 梳理 `navigation/g02` 的 BT、costmap、controller、planner 参数。

优先级 P2：

- 建立 GDK API 调用矩阵：初始化环境、对象、方法、返回字段、对应 topic/service、常见失败。
- 梳理 `fault_manager` 配置和故障码表，建立故障码快速索引。
- 建立“最小 G2-compatible 自研平台”BOM 草案：主控、实时网络、总线、执行器、传感器、电源、安全链。

## 4. 下次会话启动流程

下次任何 G2 任务先做：

```text
1. 读 g2_architecture_audit/G2_expert_context.md
2. 根据问题读对应 runbook
3. 如果是 live fault，SSH 到 agi@10.20.15.152 读最新状态
4. 不直接相信历史状态，先刷新 /data/logs/latest 和 systemd/ps/ip/ss
5. 修完更新项目文件，必要时追加 memory extension note
```

## 5. 专家级标准

达到专家级不是知道名词，而是能做到：

- 看一个现象，能快速落到 HMI/GDK/通信/算法/HAL/总线/硬件的某一层。
- 知道每层的关键日志、配置、topic、进程和验证命令。
- 能把修复和回滚路径写清楚。
- 能用最小风险验证，不用大动作赌硬件。
- 能持续把经验沉淀为项目文件和可检索记忆。
