# G2 专家上下文入口

日期：2026-06-04  
用途：每次重新打开会话时，先读本文件，快速恢复对这台 G2 的系统理解和排障入口。

## 1. 工作原则

目标不是“会跑几个 GDK 示例”，而是形成能稳定排障、修复、验证、沉淀的工程闭环。

每次处理 G2 问题按这个顺序：

```text
1. 明确当前现象和风险边界
2. 先读最新日志和当前运行态，不用旧结论替代现场证据
3. 按架构链路定位：HMI/GDK -> service -> bus -> algorithm/control -> HAL -> bus -> hardware
4. 小补丁、可回滚、先备份
5. 修复后做分层验证：进程/通信/状态/API/日志/必要时小幅物理动作
6. 把可复用结论写入项目文档
7. 如果用户明确要求记忆化，把摘要写入 memory extension
```

高风险动作边界：

- 不在未确认现场安全时重启 `genie_app.service`、清 EtherCAT fault、上电、释放急停、发运动命令。
- 不为“看起来成功”降低 SLAM 重定位阈值或绕过安全状态。
- 不只信 API 返回值、`RUNNING`、`SUCCESS`、命令退出码，必须看真实状态和日志。

## 2. 必读入口

```text
g2_architecture_audit/G2_build_level_architecture_20260604.md
  构建级架构、硬件控制分层、复刻缺口。

g2_architecture_audit/G2_system_architecture_handbook.md
  systemd、run.sh、launcher、进程职责、硬件链路。

g2_architecture_audit/G2_topic_service_map.md
  topic/service 和消息类型风险。

g2_architecture_audit/G2_diagnostic_runbook.md
  常用诊断命令和安全排障顺序。

G2_relocalization_debug_runbook_20260604.md
  本次重定位失败排查、通信修复、SLAM 剩余问题。
```

## 3. 当前基线

机器人：

```text
SSH: agi@10.20.15.152
password: <set via ROBOT_PASS when password-based SSH is required>
host: G2
app: /home/agi/app
logs: /data/logs/latest
data: /data
version: genie_g02_rb_2.2.0_320dcc1f_2026-04-01-09-31-20_r1.tar.gz
kernel: Linux 5.10.220-rt112 PREEMPT_RT aarch64
GDK: agibot_gdk 2.6.3
```

启动链路：

```text
genie_app.service
  -> /home/agi/app/bin/run.sh
    -> /home/agi/app/conf/sys/run.conf
    -> FastDDS discovery
    -> AORTA service
    -> /home/agi/app/bin/launcher
      -> /home/agi/app/conf/manifest.d/base.json
```

当前 scene：

```text
base
```

核心运行时：

```text
gdk_service
gdk_http_server :8849
hmi_proxy_end
task_manager
fault_manager
power_manager
monitor_app
hal
hal_lowerlimb
lidar
slam_state_machine
dr_state_machine
quark_navigation
genie_motion_control
camera_service
camera_dlb
cosine_runner
```

## 4. 核心架构心智模型

```text
用户/Pad/脚本/GDK
  -> gdk_service / hmi_proxy_end / task_manager
  -> AORTA + FastDDS + Cosine bus
  -> SLAM/DR/PNC/MotionControl
  -> HAL / hal_lowerlimb
  -> EtherCAT / CAN-FD / SPI MCU / 10.42.x 传感器网络
  -> 执行器、传感器、电源和底盘硬件
```

故障定位时按层切：

```text
入口层: Pad、HTTP、GDK、脚本参数
服务层: gdk_service、task_manager、hmi_proxy_end
通信层: AORTA、FastDDS、topic type、queue depth、IP 绑定
算法层: SLAM、DR、PNC、MotionControl、WBC
硬件抽象层: HAL、hal_lowerlimb、fault_manager
总线层: EtherCAT、CAN-FD、SPI MCU、PTP
硬件层: 伺服、底盘板、电源板、传感器、线束、急停
```

## 5. 硬件控制速记

上半身 EtherCAT：

```text
Master0: 18 slaves, 1 kHz, OP
0-1: Microchip LAN9254 junction
2-8: left arm CoolDrive JMDT
9: left end plate/app
10-16: right arm CoolDrive JMDT
17: right end plate/app
current arm producer: tianji
left config: /home/agi/app/conf/hal/tianji_left_arm_ccs_t1_config.yaml
right config: /home/agi/app/conf/hal/tianji_right_arm_ccs_t1_config.yaml
```

头腰 CAN-FD：

```text
head: can0, 3 axes, idx11..idx13, JXZN ZY_R48
waist: can1, 5 axes, idx01..idx05, JXZN ZY_R120/ZY_R120MAX
CAN-FD: 1 Mbps arbitration, 5 Mbps data
```

底盘：

```text
hal_lowerlimb
config: /home/agi/app/conf/hal/lowerlimb_config.yaml
ethercat_interface: ecat1
four-wheel chassis: steering + traction for 4 wheels
power board: g02_chassis, hw 1.0.1, sw 0.0.17
```

传感器：

```text
MID360: 10.42.0.122, 10.42.0.123
host: 10.42.0.101
topics: /lidar/livox_front, /lidar/livox_back, /imu/livox_front, /imu/livox_back
xt_chest: 10.42.0.121, /lidar/xt_chest
calibration: /data/parameters/sensor
```

运动模型：

```text
MotionControl profile: G2_t2_crs
URDF: /home/agi/app/share/genie_robot_description/urdf/G2_t2_crs/G2_t2_crs.urdf
SRDF: /home/agi/app/share/genie_robot_description/srdf/G2_t2_crs/G2_t2_crs.srdf
config: /home/agi/app/bin/motion-control/configuration/robot/G2_t2_crs
```

## 6. 常用只读入口

登录：

```bash
ssh agi@10.20.15.152
```

系统和进程：

```bash
date
hostname
cat /home/agi/app/.version
systemctl status genie_app.service --no-pager -l
ps -e -o pid,ppid,nlwp,stat,pcpu,pmem,comm,args --sort=-nlwp | head -80
pgrep -a -f 'launcher|aorta|fastdds|gdk|hal|motion|quark|slam|dr|fault|camera|lidar|task'
```

网络和中间件：

```bash
ip -br addr
ss -lntup | grep -E '2379|2380|8849|11811'
grep --color=never -nE 'LOCATOR|AORTA|fastdds|launcher|DEFAULT_LAUNCH_SCENE' /home/agi/app/bin/run.sh
```

日志：

```bash
ls -l /data/logs/latest
find /data/logs/latest/ -maxdepth 2 -type f | sort | sed -n '1,200p'
grep -R "ERROR\\|WARN\\|Failed\\|fault\\|DataLoss\\|type mismatch" /data/logs/latest -n | tail -200
```

硬件总线：

```bash
sudo ethercat master
sudo ethercat slaves
ip -details link show can0
ip -details link show can1
ls -l /dev/EtherCAT* /dev/ptp* /dev/spidev2.0 2>/dev/null
```

## 7. 高价值故障入口

重定位失败：

```text
先看 G2_relocalization_debug_runbook_20260604.md
再看 /data/logs/latest/slam*、lidar*、dr*、quark_navigation*
重点分清：
  通信/队列/TF 问题
  传感器输入问题
  地图/现场/初始位姿匹配问题
  阈值保护问题
```

底盘不动：

```text
先看 Robot.get_chassis_power_state()
再看 pnc.get_task_state()
再看 quark_navigation.log 和 hal_lowerlimb.log
重点字段：
  emergency_stop_pedal_state
  emergency_stop_pedal_fault_state
  charge_plug_insert_state
  chassis motor power states
```

手臂故障：

```text
HAL 最新日志 -> HAL motor id -> EtherCAT slave -> drive status/error -> GDK joint state
已知映射：
  motor 0-6 -> slave 2-8
  motor 7-13 -> slave 10-16
验证标准：
  EtherCAT status/error
  GDK joint state
  HAL log transition
  必要时小幅真实动作
```

通信异常：

```text
看 AORTA_DISCOVERY_URI、LOCATOR_IP、FastDDS 绑定、topic type mismatch、queue depth、DataLoss
当前特别注意：
  文件 run.sh 有动态 IP 逻辑
  2026-06-04 15:17 运行态仍选 10.42.1.101
  需从 boot 日志确认选择路径
```

## 8. 项目沉淀规则

每次新增有效知识，写到对应文件：

```text
架构/硬件/通信: g2_architecture_audit/G2_system_architecture_handbook.md
topic/service:  g2_architecture_audit/G2_topic_service_map.md
排障流程:       g2_architecture_audit/G2_diagnostic_runbook.md
制造复刻:       g2_architecture_audit/G2_build_level_architecture_20260604.md
专家入口:       g2_architecture_audit/G2_expert_context.md
阶段清单:       g2_architecture_audit/G2_expert_mastery_plan.md
```

如果用户明确要求“记住/下次知道/写入记忆”，再在：

```text
/home/davie/.codex/memories/extensions/ad_hoc/notes/
```

新增一条短记忆更新，不直接改主记忆文件。

## 9. 当前不能冒充已掌握的部分

还缺这些才能真正接近自研制造：

```text
CAD / 加工图 / 装配公差
BOM / 线束图 / 接插件编号
电源板、底盘板、驱动板原理图
MCU、EtherCAT 从站、头腰电机固件
ESI/PDO/SDO 完整描述
标定工装和生产 EOL 流程
```

结论：当前已经具备专家级排障入口和系统地图的雏形，但制造级专家还需要继续把机械、电气、固件、标定和生产资料补齐。
