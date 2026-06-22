# G2 系统底层架构手册

更新时间：2026-06-04  
目标：掌握这台 G2 的运行入口、后台服务、硬件控制链路、通信方式、配置边界和诊断入口。

## 1. 总结

这台 G2 不是单个 Python 项目，而是一套 systemd 拉起的机器人运行时。主要真相在机器人侧：

```text
/home/agi/app          安装包与运行入口
/home/agi/app/conf     系统、DDS、HAL、record、edge 等配置
/home/agi/app/config   fault_manager、HMI、电池阈值等产品配置
/home/agi/app/bin      各业务/驱动/控制二进制和启动脚本
/data/logs/latest      最新运行日志
/data/parameters       现场参数覆盖层
```

本地 `/home/davie/G2` 更像开发机资料库：GDK Python 示例、故障 runbook、手册摘录和实验脚本。它能说明 API 使用方式，但不能替代机器人 `/home/agi/app` 的运行配置和日志。

核心控制链路：

```text
用户脚本 / 任务程序 / HMI / 遥操作
        |
        v
agibot_gdk / ROS2 / HTTP / remote_hal
        |
        v
gdk_service / gdk_http_server / task_manager / pico_adapter
        |
        v
AORTA + FastDDS discovery
        |
        +--------------------+---------------------+
        |                    |                     |
        v                    v                     v
genie_motion_control     quark_navigation      camera/lidar/slam/dr
        |                    |
        v                    v
HAL 上肢/全身             hal_lowerlimb 底盘/电源
        |                    |
        v                    v
EtherCAT 伺服驱动器       CAN/SPI/底盘电源/MCU
        |
        v
腰部、头部、双臂、末端执行器
```

诊断原则：不能只信 API 返回、`RUNNING`、或者命令成功。必须同时看 GDK 状态、实时日志、HAL/PNC 状态、总线状态，必要时再做受控的小幅物理验证。

## 2. 版本与系统

现场已验证：

- 主机名：`G2`
- 用户：`agi`
- 应用版本：`/home/agi/app/.version`
  - `genie_g02_rb_2.2.0_320dcc1f_2026-04-01-09-31-20_r1.tar.gz`
- 内核：`Linux 5.10.220-rt112 #1 SMP PREEMPT_RT ... aarch64`
- CPU：24 核 Cortex-A78AE
- 内存：约 61 GiB
- 主要包版本：
  - `agibot_gdk 2.6.3`
  - `aorta 2.2.0`
  - `camera snap-collinear-0309`

存储布局现场已验证：

```text
sdc        系统盘，含 /root_ro、/root_rw、/mnt/user_data
nvme0n1    约 1.9T 数据盘
  /shared/pad
  /shared/head
  /shared/tmp
  /data
```

## 3. 启动链路

顶层服务是：

```text
genie_app.service
```

现场已验证 systemd 关系：

```text
genie_app.service
  After=network.target agibot_perfguard.service rhino_start.service rhino_time_sync.service rhino_ptp4l_domain0.service
  ExecStart=/bin/bash /home/agi/app/bin/run.sh
  Restart=on-failure
```

`/home/agi/app/bin/run.sh` 的关键动作：

1. 读取 `/home/agi/app/conf/sys/run.conf`。
2. 检查 `/data` 是否在 SSD/NVMe 上。
3. 创建 `/data/logs/bootNNNNNNNN`，并维护 `/data/logs/latest`。
4. 配置 `LD_LIBRARY_PATH`、ROS Humble、`genie_msgs`、日志目录。
5. 在 `ENABLE_G02_GDK=1` 时执行 `/home/agi/app/gdk/scripts/prepare_server.sh`。
6. 设置 `ROS_LOCALHOST_ONLY=1`、`DYLOG_log_dir`、`ROS_LOG_DIR`。
7. 启动 FastDDS discovery。
8. 启动 `perfetto_traced`。
9. 启动 AORTA service。
10. 执行 EtherCAT 配置脚本 `ecat_config.sh`。
11. 启动 `launcher`，由 launcher 拉起业务模块。

现场已验证 `run.conf` 关键项：

```bash
PRODUCT_GENERATION=2
ENABLE_GDK=0
ENABLE_G02_GDK=1
ENABLE_PERFGUARD=1
APP_DIR=/home/agi/app
LOG_PATH=/data/logs/
AORTA_URI=http://10.42.1.101
DEFAULT_LAUNCH_SCENE=base
LOCATOR_IP=10.42.1.101
AORTA_DISCOVERY_URI=http://10.42.1.101:2379
COSINE_BUS_DEFAULT_MIDDLEWARE=aorta
FASTDDS_PARTICIPANT_PROFILE=FastDDSProfile
FASTDDS_DEFAULT_PROFILES_FILE=$APP_DIR/conf/dds/fastdds_profile.xml
COSINE_ENABLE_ROS_TOPIC_PREFIX=1
```

注意：本轮在 `10.20.15.152` 上验证到原始 `run.sh` 写死 FastDDS `10.42.1.101`，`run.conf` 也默认指向 `10.42.1.101`。实际现场 `enp1s0` 当前是 DOWN，当前 SSH 走 `wlan0=10.20.15.152`。已给 `/home/agi/app/bin/run.sh` 做动态 IP 选择补丁并通过 `bash -n`，备份为 `/home/agi/app/bin/run.sh.bak.20260604-141506`。2026-06-04 15:17 新启动后，运行态 AORTA/FastDDS 仍选择 `10.42.1.101`，下一轮应从 boot 日志确认启动时接口状态和选择路径。

## 4. Launcher 模块

当前实际 scene 已从 launcher 日志确认：

```text
switch scene from idle to base
```

因此实际 manifest 是 `/home/agi/app/conf/manifest.d/base.json`。核心模块包括：

```text
hal
hal_lowerlimb
power_manager
camera_service -i 1
camera_dlb
camera_copilot
teleop_it
remote_hal / pico_adapter
dlb
dds_record
fault_manager
monitor_app
hmi_proxy_end
lidar
pnc / quark_navigation
slam
dr
mc / genie_motion_control
tagloc
task_manager
freespace
media_manager
gdk_service
gdk_http_server
arbitrator
corobot
```

`base-remote.json` 是备选 manifest，比 `base.json` 多 `camera_rtc` 等远程相关项；不要把它当作当前运行真相。

## 5. 运行进程职责

现场已验证高线程/核心进程：

```text
cosine_runner              camera_copilot/模型或协同运行器
camera_dlb                 相机 DLB
camera_service             相机服务入口
quark_navigation           PNC 导航、任务、底盘控制
pico_adapter               remote_hal/遥操作适配
hal                        上肢/全身硬件抽象
genie_motion_control       全身运动控制
fault_manager              故障聚合与上报
dlb                        数据链路/业务模块
slam_state_machine         SLAM 状态机
hal_lowerlimb              底盘/下肢/电源 HAL
media_manager              媒体服务
dds_record                 话题录制
run_corobot_app            corobot 应用
freespace_state_machine    freespace
lidar                      雷达服务
monitor_app                监控
task_manager               任务系统
fast-discovery-server      FastDDS discovery
aorta-service              AORTA/etcd 服务发现
launcher                   模块进程管理
power_manager              电源管理
arbitrator_runner          仲裁/WBC 相关
dr_state_machine           DR 死算里程计
tagloc_state_machine       标签定位
gdk_service                GDK 后台服务
teleop_main_node           遥操作
gdk_http_server            8849 文档/日志/安装服务
```

关键服务：

- `agibot_perfguard.service`: 在 `genie_app` 前启动，启用 performance guard。
- `ethercat.service`: oneshot，发现 `ecat*` 网口后执行 `/etc/systemd/ethercat.sh start`。
- `rhino_ptp4l_domain0.service`: PTP 时间同步。
- `rhino_mcu_daemon.service`: `/usr/bin/mcu_daemon /dev/spidev2.0`。
- `edge-client.service`: 云/边缘客户端，配置为 `/home/agi/app/conf/edge-client/edge-client-config.yaml`。
- `chrony.service`: 系统 NTP/时间同步。

## 6. 通信架构

### 6.1 AORTA

AORTA 是机器人内部主要消息总线和服务发现层。`run.conf` 设置：

```bash
COSINE_BUS_DEFAULT_MIDDLEWARE=aorta
AORTA_DISCOVERY_URI=http://...:2379
```

`run.sh` 启动：

```bash
/home/agi/app/bin/aorta service -g -d \
  --data-dir="$ETCD_DATA_DIR" \
  --listen-peer-urls $AORTA_URI:2380 \
  --listen-client-urls $AORTA_URI:2379
```

机器人模块通过 AORTA 创建 publisher/subscriber。日志中会显示 topic、publisher、subscriber 和 message type。

### 6.2 FastDDS

FastDDS discovery 在 `run.sh` 中启动，默认端口为 `11811`。原始文件写死 `10.42.1.101`，本轮已改为：

```bash
./bin/fastdds discovery -i 0 -l <LOCATOR_IP> -q 11811 -p 11811
```

GDK、ROS2 示例和部分模块依赖 DDS discovery。网络 IP 绑定错误会导致开发机能 SSH，但 GDK 或 ROS2 发现失败。

### 6.3 ROS2/GDK

本地 GDK 文档说明开发机一般走：

```bash
curl -sSL http://10.42.1.101:8849/install.sh | bash
source ~/.cache/agibot/app/env.sh
```

Python 入口通常是：

```python
import agibot_gdk
agibot_gdk.gdk_init()
robot = agibot_gdk.Robot()
pnc = agibot_gdk.Pnc()
```

常用对象：

```text
Robot()              关节、全身状态、电源、末端、运动控制状态
Pnc()                底盘、导航任务、相对移动
Slam()               SLAM/odom/定位
Map()                地图
TF()                 坐标外参和 TF
Camera()             相机图像、内参
Lidar()              点云
Imu()                IMU
UltrasonicRadar()    超声波
```

## 7. 硬件控制链路

### 7.1 上肢/腰部/头部/末端

现场和 runbook 共同确认的链路：

```text
agibot_gdk Robot APIs
  -> gdk_service
  -> MotionControlService / genie_motion_control
  -> HAL
  -> EtherCAT master
  -> CoolDrive/JuXie servo drives
  -> 腰部、头部、左臂、右臂关节
```

`genie_motion_control` 现场日志显示模型分组：

```text
Waist Lift    offset 0   length 5   topic /hal/joint_cmd_raw
Head Yaw      offset 5   length 1
Head Roll     offset 6   length 1
Head Pitch    offset 7   length 1
Left Arm      offset 8   length 7   topic /hal/joint_cmd_raw
Left Tool     offset 15  length 1   topic /wbc/left_ee_command
Right Arm     offset 16  length 7
Right Tool    offset 23  length 1
```

GDK 文档和本地示例使用的 22 个主要关节名：

```text
idx01_body_joint1 ... idx05_body_joint5
idx11_head_joint1 ... idx13_head_joint3
idx21_arm_l_joint1 ... idx27_arm_l_joint7
idx61_arm_r_joint1 ... idx67_arm_r_joint7
```

EtherCAT 手臂映射：

```text
HAL motor 0-6   -> EtherCAT slave 2-8
HAL motor 7-13  -> EtherCAT slave 10-16
```

已验证实例：

```text
HAL motor 2 -> EtherCAT slave 4 -> GDK joint idx23_arm_l_joint3
HAL motor 6 -> EtherCAT slave 8 -> GDK joint idx27_arm_l_joint7
```

### 7.2 底盘/下肢/电源

底盘链路：

```text
agibot_gdk Pnc APIs
  -> gdk_service / AORTA
  -> quark_navigation
  -> /pnc/chassis_joint_cmd
  -> hal_lowerlimb
  -> CAN/底盘驱动/电源板/MCU
```

已知底盘关节：

```text
idx111_chassis_lwheel_front_joint1
idx121_chassis_lwheel_rear_joint1
idx131_chassis_rwheel_front_joint1
idx141_chassis_rwheel_rear_joint1
```

底盘控制经验：

- `move_chassis(Twist)` 是速度式控制，容易受控制权和任务状态影响。
- `relative_move(NaviReq)` 走 quark_navigation 任务系统，使用 DR 死算里程计闭环，实测比 `move_chassis` 稳。
- SLAM 不可用不代表底盘不能相对移动；`relative_move` 可依赖 DR/IMU。

### 7.3 传感器

本地 GDK 示例和现场模块共同显示的传感器类别：

```text
Camera:
  HeadStereoLeft, HeadStereoRight
  HeadColor, HeadDepth
  HeadBackFisheye, HeadLeftFisheye, HeadRightFisheye
  HandLeftColor, HandRightColor

Lidar:
  LidarFront, LidarBack

IMU:
  ImuFront, ImuBack, ImuChassis

Ultrasonic:
  UltrasonicRadar
```

现场设备节点：

```text
/dev/video*
/dev/ttyUSB0..3
/dev/ttyS0..
/dev/spidev2.0
/dev/ptp*
can0, can1
```

## 8. 总线与网络

现场已验证网口：

```text
xgi0                 10.42.0.101/24
xgi1                 10.42.12.101/24
enp1s0               10.42.1.101/24 linkdown
pad                  10.42.6.101/24
xgi0.80@xgi0         192.168.1.101/24
ecat0@xgi1           10.42.30.101/24
ecat1@xgi1           10.42.40.101/24
wlan0                10.20.15.152/24
can0, can1           UP
```

默认外部 SSH 走 `wlan0`。调试网文档默认机器人有线调试 IP 为 `10.42.1.101`，开发机为 `10.42.1.102`。

EtherCAT 配置现场已验证：

```bash
MASTER0_DEVICE="xgi0.30"
MASTER1_DEVICE="xgi0.40"
MASTER2_DEVICE="xgi0.50"
DEVICE_MODULES="generic"
```

此前现场验证：

```text
Master0 OP, 18 slaves all OP, 1kHz, Tx errors 0
slave 0,1      Microchip-LAN9254-EtherCAT-Junction-Box
slave 2-8      CoolDrive JMDT
slave 9        app
slave 10-16    CoolDrive JMDT
slave 17       app
```

CAN 现场已验证：

```text
can0, can1:
  CAN-FD
  ERROR-ACTIVE
  bitrate 1000000
  dbitrate 5000000
  restart-ms 1000
```

PTP 现场已验证并修复权限：

```text
/dev/ptp_clock -> /dev/ptp_xgi0
/dev/ptp_xgi0  -> ptp1
/dev/ptp_xgi1  -> ptp2
/dev/ptp_pci   -> ptp3
/dev/ptp_rgi0  -> ptp0
```

修复前 `/dev/ptp0..3` 是 `root:root 0600`，`agi` 运行 GDK 会提示 `No valid PTP device found`。本轮已增加 `/etc/udev/rules.d/99-g2-ptp-permissions.rules`：

```text
KERNEL=="ptp[0-9]*", GROUP="plugdev", MODE="0660"
```

并即时设置 `/dev/ptp0..3` 为 `root:plugdev 0660`。复测结果：

```text
PTP device /dev/ptp_xgi0 opened successfully
gdk_init GDKRes.kSuccess
```

## 9. 配置边界

高风险但核心的配置区域：

```text
/home/agi/app/conf/sys/run.conf
/home/agi/app/bin/run.sh
/home/agi/app/conf/manifest.d/*.json
/home/agi/app/conf/dds/fastdds_profile.xml
/home/agi/app/conf/hal/*
/home/agi/app/bin/motion-control/configuration/robot/G2_t2_crs/*
/home/agi/app/bin/navigation/config/g02/*
/home/agi/app/config/g02_fault_manager.json
/home/agi/app/config/g02_fault_code.json
/home/agi/app/config/battery_threshold.yaml
/home/agi/app/bin/remote_hal/g02_remote_hal_cfg.json
/home/agi/app/gdk/config/app_conf.json
/home/agi/app/conf/record/record_topic.json
```

现场参数覆盖：

```text
/data/parameters/hardware/arm_parameters.yaml
```

本轮已修复：该文件之前缺失，已从 `/home/agi/app/conf/hal/arm_parameters.default.yaml` 补齐。需要 `genie_app.service` 重启后才会被运行态重新加载；本轮没有重启，因为重启会影响 HAL/MC/PNC，必须现场安全确认。

## 10. 已知问题与修复状态

已低风险修复：

- `/data/parameters/hardware/arm_parameters.yaml` 已补齐。
- `/dev/ptp_clock -> /dev/ptp_xgi0` 已创建并通过 `/etc/tmpfiles.d/g2-ptp-clock.conf` 持久化。
- `/dev/ptp0..3` 权限已改为 `root:plugdev 0660`，并通过 udev 规则持久化。
- chrony 已增加公共 NTP fallback，系统时间已从错误时间域校正到 2026-06-04，并显示同步成功。
- `/home/agi/app/bin/run.sh` 已备份并补丁为动态选择 `LOCATOR_IP`；下次 `genie_app` 启动时 FastDDS/AORTA 会按可达网卡绑定。

仍需安全确认后生效：

- `arm_parameters.yaml` 需要重启 `genie_app.service` 才会被 HAL/MC 重新加载。
- `run.sh` 动态 IP 补丁需要重启 `genie_app.service` 才会进入运行态。

仍需排查：

- `/wbc/left_ee_command`、`/wbc/right_ee_command` 现场日志存在消息类型不一致：
  - publisher `arbitrator_runner`: `genie_msgs.msg.pb.JointState`
  - subscriber `hal`: `sensor_msgs.msg.pb.JointState`
  - 这更像编译包/配置 ABI 不匹配，不建议用临时软改。
- `/MotionControlService/SetLoad/request` 存在 `SetLoadRequest` vs `SetLoadRequst` 拼写差异。安装包生成 ABI 里使用 `SetLoadRequst`，如果外部脚本使用 `SetLoadRequest` 会产生不匹配。
- GDK 只读健康检查显示 `emergency_stop_pedal_state=0` 但 `emergency_stop_pedal_fault_state=1`，同时 `charge_plug_insert_state=1`。当前 whole-body `chassis_error=0`、PNC task idle，但该故障位需要结合底盘硬件/启动瞬态继续判断。

## 11. 后续需要现场验证

1. 安全确认后重启 `genie_app.service`，验证动态 IP 和 `arm_parameters.yaml` 进入运行态。
2. 对 `SetLoadRequest/Requst` 做 ABI 来源定位：`run_corobot_app` 侧和 `genie_motion_control` 侧生成 proto 是否来自不同包。
3. 对 `/wbc/*_ee_command` 的多 publisher 做运行态责任划分：`run_corobot_app`、`arbitrator_runner`、`genie_motion_control` 谁应保留。
4. 底盘急停踏板故障位 `emergency_stop_pedal_fault_state=1` 需要继续判断是启动瞬态、充电状态联动还是硬件线路问题。
5. 如需达到开发级掌握，下一轮应导出 `G2_t2_crs` 运动模型、`navigation/g02` 参数和 GDK Python API 的完整可调用矩阵。
