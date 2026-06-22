# G2 构建级架构与复刻路线图

日期：2026-06-04  
目标机器人：`agi@10.20.15.152`  
目标：把这台 G2 的架构理解推进到“可以指导自研复刻”的层级，而不是只停留在会用 GDK 或会看日志。

## 1. 当前结论

这台 G2 是一个多总线、多厂家模块融合的移动双臂机器人。它不是单一 ROS 工程，也不是单一 Python/GDK 工程。

当前已确认的主链路：

```text
HMI / Pad / 用户脚本 / GDK / 遥操作
    -> gdk_service / gdk_http_server / task_manager / hmi_proxy_end
    -> AORTA + FastDDS + Cosine bus
    -> SLAM / DR / PNC / MotionControl / HAL / hal_lowerlimb
    -> EtherCAT / CAN-FD / SPI MCU / 10.42.x 传感器网络
    -> 双臂、头腰、底盘、末端、电源板、雷达、相机、IMU
```

当前掌握度评估：

- 软件启动链路：较高。`genie_app.service -> run.sh -> launcher -> base.json` 已确认。
- 进程职责和通信：较高。已抽取当前运行进程、topic/service、AORTA/FastDDS 绑定。
- 定位导航：中高。已确认 SLAM/DR/PNC 配置、地图入口、重定位阈值和近期失败原因。
- 硬件控制链路：中高。已确认 EtherCAT、CAN-FD、SPI MCU、雷达网络和主要执行器配置。
- 机械制造复刻：中。已有 URDF/SRDF/MJCF 和关节/连杆/限位入口，但缺真实 CAD、加工图、装配公差、线束图。
- 电气/板级复刻：偏低。已知设备节点和总线，但缺电源板、底盘板、驱动板、MCU 固件、线束和保护电路原理图。

要达到“我们自己也能造出来”，下一阶段必须把 CAD/BOM/电气原理图/固件/标定工装补齐。现在这份文档是复刻路线图和当前证据基线。

## 2. 主机与运行时

现场确认：

```text
hostname: G2
version: genie_g02_rb_2.2.0_320dcc1f_2026-04-01-09-31-20_r1.tar.gz
kernel: Linux 5.10.220-rt112 PREEMPT_RT aarch64
CPU: 24 x Cortex-A78AE, 6 clusters
GDK: agibot_gdk 2.6.3
AORTA: 2.2.0
cosine_bus: 3.5.0
HAL: 2.2.7
motion-control: 0.5.18
navigation: 1.1.7-hotfix3-cv410
slam: 1.1.12
```

存储：

```text
/root_ro, /root_rw       系统根
/mnt/user_data           用户分区
/data                    NVMe 数据分区，日志、地图、标定、运行数据
/shared/pad              Pad 相关数据
/shared/head             头部相关数据
/shared/tmp              临时数据
```

关键原则：制造复刻时应把系统镜像、应用包、地图数据、日志数据和现场标定分层管理，不能把 `/data/parameters` 当成可有可无的配置。

## 3. 启动架构

顶层服务：

```text
genie_app.service
  ExecStart=/bin/bash /home/agi/app/bin/run.sh
  User=root
  Restart=on-failure
```

`run.sh` 做的事情：

```text
1. source /home/agi/app/conf/sys/run.conf
2. 解析 LOCATOR_IP / AORTA_DISCOVERY_URI
3. 创建 /data/logs/bootNNNNNNNN 并维护 /data/logs/latest
4. source ROS Humble 和 genie_msgs 环境
5. 设置 ROS_LOCALHOST_ONLY、DYLOG_log_dir、ROS_LOG_DIR
6. 启动 FastDDS discovery
7. 启动 perfetto_traced
8. 清理旧 aorta-service 并启动新 AORTA discovery
9. 执行 ecat_config.sh
10. 启动 launcher
```

当前运行 scene：

```text
/home/agi/app/conf/manifest.d/base.json
```

核心模块：

```text
hal
hal_lowerlimb
camera_service
camera_dlb
cosine_runner
remote_hal / pico_adapter
dlb
dds_record
fault_manager
power_manager
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
teleop_it
```

## 4. 当前进程模型

现场运行态主进程：

```text
launcher
  run_corobot_app
  gdk_service
  media_manager
  gdk_http_server -ip 0.0.0.0 -p 8849
  hmi_proxy_end
  power_manager
  arbitrator_runner
  monitor_app
  task_manager
  hal
  hal_lowerlimb
  lidar
  dr_state_machine
  tagloc_state_machine
  freespace_state_machine
  camera_service
  camera_dlb
  cosine_runner camera_copilot
  quark_navigation
  slam_state_machine
  genie_motion_control
```

后台基础服务：

```text
aorta-service
fast-discovery-server
rhino_mcu_daemon /dev/spidev2.0
agibot_perfguard.service
ethercat.service
rhino_ptp4l_domain0.service
chrony.service
edge-client.service
```

## 5. 网络与通信

当前主要接口：

```text
wlan0      10.20.15.152    外部 SSH/调试
xgi0       10.42.0.101     传感器/内部网络
xgi1       10.42.12.101    内部网络
enp1s0     10.42.1.101     当前 DOWN，但配置层仍常用
ecat0      10.42.30.101    EtherCAT 相关
ecat1      10.42.40.101    lowerlimb EtherCAT 相关
wlanap0    10.42.6.101     AP/HMI 类链路
can0       CAN-FD
can1       CAN-FD
```

服务端口：

```text
2379/tcp    AORTA client/discovery
2380/tcp    AORTA peer
11811/tcp   FastDDS discovery
11811/udp   FastDDS discovery
8849/tcp    GDK HTTP server
```

运行态仍绑定：

```text
FastDDS: 10.42.1.101:11811
AORTA:   http://10.42.1.101:2379
```

`run.sh` 文件已经有动态 IP 选择逻辑，但这次启动仍选到 `10.42.1.101`。推测启动时 `10.42.1.101` 曾处于可用状态，或 route/接口状态和后续不同。后续要从 boot 日志确认选择路径。

## 6. 硬件控制分层

### 6.1 上半身 EtherCAT

系统 EtherCAT master：

```text
Master0: Operation
Slaves: 18
Cycle: 1000 frames/s
Lost frames: 0
```

从站：

```text
0-1      Microchip LAN9254 EtherCAT Junction Box
2-8      CoolDrive JMDT, 左臂 7 轴
9        app, 左端板/末端相关
10-16    CoolDrive JMDT, 右臂 7 轴
17       app, 右端板/末端相关
```

HAL 日志确认当前使用：

```text
force-control-arm producer: tianji
IgH EtherCAT Tianji arm
left config:  /home/agi/app/conf/hal/tianji_left_arm_ccs_t1_config.yaml
right config: /home/agi/app/conf/hal/tianji_right_arm_ccs_t1_config.yaml
```

左臂驱动级关节：

```text
id 3-9
idx21_arm_l_joint1 ... idx27_arm_l_joint7
mode: csp
gear_ratio: 120,120,100,100,100,100,100
```

右臂驱动级关节：

```text
id 11-17
idx61_arm_r_joint1 ... idx67_arm_r_joint7
mode: csp
gear_ratio: 120,120,100,100,100,100,100
```

`/data/parameters/hardware/arm_parameters.yaml` 是现场硬件覆盖层，包含 arm_type、方向、编码器分辨率、齿比、力矩系数和 torque_base。

### 6.2 头部与腰部 CAN-FD

头部：

```text
config: /home/agi/app/conf/hal/juxie_head_config_t2.yaml
interface: can0
frequency: 1000 Hz
node_id: 3,2,1
joints: idx11_head_joint1, idx12_head_joint2, idx13_head_joint3
motor models from log: ZY_R48
producer: JXZN
```

腰部：

```text
config: /home/agi/app/conf/hal/juxie_waist_config_t2.yaml
interface: can1
frequency: 1000 Hz
node_id: 8,7,6,5,4
joints: idx01_body_joint1 ... idx05_body_joint5
motor models from log: ZY_R120 / ZY_R120MAX
producer: JXZN
```

CAN-FD 状态：

```text
can0/can1: UP, ERROR-ACTIVE
arbitration bitrate: 1 Mbps
data bitrate: 5 Mbps
restart-ms: 1000
```

### 6.3 底盘 EtherCAT

`hal_lowerlimb` 配置：

```text
/home/agi/app/conf/hal/lowerlimb_config.yaml
ethercat_interface: ecat1
joint_states_frequency_hz: 200
```

底盘配置：

```text
/home/agi/app/conf/hal/fourwheel_chassis_config.yaml
left_front_steering:   idx111_chassis_lwheel_front_joint1
left_front_traction:   idx112_chassis_lwheel_front_joint2
left_rear_steering:    idx121_chassis_lwheel_rear_joint1
left_rear_traction:    idx122_chassis_lwheel_rear_joint2
right_front_steering:  idx131_chassis_rwheel_front_joint1
right_front_traction:  idx132_chassis_rwheel_front_joint2
right_rear_steering:   idx141_chassis_rwheel_rear_joint1
right_rear_traction:   idx142_chassis_rwheel_rear_joint2
```

日志确认：

```text
ecat1 init success
found 2 slaves, expected 2
all registered slaves OPERATIONAL
power board serial: g02_chassis
power board hw: 1.0.1
power board sw: 0.0.17
traction motor version: 2025.514.300
steer motor version: 1.14.06
```

### 6.4 MCU / PTP / 时间

```text
rhino_mcu_daemon.service active
command: /usr/bin/mcu_daemon /dev/spidev2.0
```

PTP：

```text
/dev/ptp0..3
/dev/ptp_clock -> /dev/ptp_xgi0 -> ptp1
rhino_ptp4l_domain0.service active
chrony.service active
```

这说明控制系统依赖精确时间和硬件时钟。复刻时不能只跑应用，需要设计 PTP、系统时钟、MCU 通信和权限策略。

## 7. 机械与运动模型

当前运动控制 profile：

```text
G2_t2_crs
URDF: /home/agi/app/share/genie_robot_description/urdf/G2_t2_crs/G2_t2_crs.urdf
SRDF: /home/agi/app/share/genie_robot_description/srdf/G2_t2_crs/G2_t2_crs.srdf
MotionControl config: /home/agi/app/bin/motion-control/configuration/robot/G2_t2_crs
```

URDF 关节结构：

```text
body:     idx01_body_joint1 ... idx05_body_joint5
head:     idx11_head_joint1 ... idx13_head_joint3
chassis:  idx111/112, idx121/122, idx131/132, idx141/142
left arm: idx21_arm_l_joint1 ... idx27_arm_l_joint7
right arm: idx61_arm_r_joint1 ... idx67_arm_r_joint7
```

MotionControl 当前分组：

```text
Waist Lift   offset 0,  length 5, topic /hal/joint_cmd_raw
Head Yaw     offset 5,  length 1, topic /hal/joint_cmd_raw
Head Roll    offset 6,  length 1, topic /hal/joint_cmd_raw
Head Pitch   offset 7,  length 1, topic /hal/joint_cmd_raw
Left Arm     offset 8,  length 7, topic /hal/joint_cmd_raw
Left Tool    offset 15, length 1, topic /wbc/left_ee_command
Right Arm    offset 16, length 7, topic /hal/joint_cmd_raw
Right Tool   offset 23, length 1, topic /wbc/right_ee_command
```

运动控制软件包含：

```text
IK
joint limits
collision / self collision
mass center limits
arm move
arm primitives
se3 track
replay
retarget
admittance / impedance
tool calibration
object/world model
```

复刻时的关键不是只发关节角，而是要重建模型、限位、碰撞、工具 TCP、payload、标定和状态机。

## 8. 传感器与标定

雷达：

```text
MID360 front/back: 10.42.0.122, 10.42.0.123
host_ip: 10.42.0.101
point ports: 56300 device, 6862 host
imu ports:   56400 device, 6863 host
topics:
  /lidar/livox_front
  /lidar/livox_back
  /imu/livox_front
  /imu/livox_back
```

胸部传感器：

```text
xt_chest
type: xintan
device_address: 10.42.0.121
topic: /lidar/xt_chest
```

相机：

```text
camera_service -i 1
camera_dlb
camera topics include head, hand, fisheye, stereo, rgb/depth streams
configs under /home/agi/app/conf/camera_config
runtime calibration under /data/parameters/sensor
```

现场标定入口：

```text
/data/parameters/sensor/
  intrinsic_*.json
  extrinsic_*.json
  struct_*.json
```

这里包含相机内参、手眼外参、雷达外参、IMU 外参、超声结构参数。复刻时需要一套标定工装和标定流程，否则软件即使跑起来，定位、避障和抓取也不会稳定。

## 9. 定位、建图与导航

SLAM 启动：

```text
slam_state_machine --configuration=/home/agi/app/bin/slam/config
```

关键配置：

```text
/home/agi/app/bin/slam/config/a2d_v01/quark/quark.yaml
/home/agi/app/bin/slam/config/a2d_v01/alg/localization.yaml
/home/agi/app/bin/slam/config/a2d_v01/alg/mapping.yaml
/home/agi/app/conf/dds/cosine_ipc.prototxt
```

当前定位配置要点：

```text
initializer_type: pre_pose
reloc_type: ndt
step: 7
radius_deg: 180
min_init_score: 0.8
timeout_bound: 20
lidar_min_range: 0.7
lidar_max_range: 35.0
tiled_map.map_path: ./data/
```

导航启动：

```text
quark_navigation --config=/home/agi/app/bin/navigation/config --robot=g02 --mode=robot
```

导航架构：

```text
dynamic libs:
  transform_proxy / transform_provider
  collision_checker
  costmap_2d / costmap_3d
  perception
  pointcloud_filter_server
  planner_server
  controller_server
  re_plan_check_server
  robot
  servo_server
  behavior_tree
  joystick_node

nodes:
  tf_proxy
  robot
  collision_checker
  costmap_node
  costmap_3d_node
  perception_node
  pointcloud_filter_server
  planner_server
  controller_server
  re_plan_check_server
  servo_server
  behavior_tree
  joystick_node
```

PNC 到底盘：

```text
PNC publishes /pnc/chassis_joint_cmd
hal_lowerlimb subscribes /pnc/chassis_joint_cmd
hal_lowerlimb drives four-wheel chassis over ecat1
```

## 10. 通信主题分组

当前运行日志抽取到的主题可分组：

```text
HAL:
  /hal/joint_state
  /hal/joint_cmd
  /hal/whole_body_status
  /hal/chassis_joint_state
  /hal/chassis_power_state
  /hal/chassis_power_ctrl
  /hal/chest_power_state
  /hal/left_ee_data
  /hal/right_ee_data
  /hal/left_ee_force_data
  /hal/right_ee_force_data

MotionControl / WBC:
  /MotionControlService/*
  /wbc/joint_control
  /wbc/joint_position_control
  /wbc/end_effector_pose_control
  /wbc/left_ee_command
  /wbc/right_ee_command
  /wbc/motion_control_status
  /wbc/model_predict

SLAM / DR / TF:
  /slam/odom
  /slam/state
  /slam/global_loc_request
  /slam/global_loc_response
  /slam/set_map_req
  /dr/odom
  /tf
  /tf_static

PNC:
  /pnc/chassis_joint_cmd
  /pnc/task_state
  /pnc/global_plan_path
  /pnc/layered_map
  /pnc/task_service/relative_move/*
  /pnc/task_service/normal_navigation/*
  /pnc/task_service/task_cancel/*

Sensors:
  /lidar/livox_front
  /lidar/livox_back
  /imu/livox_front
  /imu/livox_back
  /lidar/xt_chest
  /camera/head_color
  /camera/head_depth
  /camera/hand_left_color
  /camera/hand_right_color
```

完整 topic 表已抽取，后续应整理进 `G2_topic_service_map.md`。

## 11. 当前已知风险

1. 运行态网络绑定和文件逻辑不完全一致。
   - 文件里 `run.sh` 已动态选择 IP。
   - 本次运行态 AORTA/FastDDS 仍是 `10.42.1.101`。
   - 后续要看 boot 日志确认启动时接口状态。

2. SLAM 点云 topic 队列仍是默认 5。
   - `/imu/livox_front`、`/imu/livox_back` 已通过配置变成 50。
   - `/lidar/livox_front`、`/lidar/livox_back` 仍在日志中显示 depth 5。
   - 是否扩大点云队列要结合延迟、内存和 SLAM 消费能力评估。

3. 配置支持列表和当前在线硬件必须分开。
   - `realman RM_75` 配置存在。
   - 当前 HAL 日志显示在线使用 `tianji` 力控臂。
   - 不可只看 `config.yaml` 判定真实硬件。

4. 运动控制存在已知消息/类型风险。
   - `/wbc/*_ee_command` 和 `SetLoadRequst/SetLoadRequest` 风险在既有 topic 文档中已有记录。
   - 这是构建/消息契约问题，不应在 Python API 层硬绕。

5. 重定位当前剩余问题偏向地图/现场/初始位姿匹配。
   - 后台通信和 SLAM 输入链路已修复。
   - GICP/NDT 分数低于 `min_init_score=0.8`。
   - 不建议直接降阈值制造假定位。

## 12. 要自己造还缺什么

机械：

- 真实 CAD，包括机身、底盘、腰、头、双臂安装座、末端安装板、传感器支架。
- 关键结构件材料、加工方式、表面处理、装配公差。
- 线束走线、拖链/转动余量、传感器视场无遮挡约束。
- 整机重心、轮距、轴距、悬挂/减震、碰撞外形。

执行器与驱动：

- CoolDrive JMDT 的型号、电压、电流、PDO/SDO 映射、故障字定义。
- 头腰 JXZN 电机协议、CAN-FD 帧格式、固件升级方式。
- 四轮底盘板、牵引电机、舵向电机、功率板的电气原理和固件。
- 末端 `omnipicker` 的通信协议、力/夹持状态、标定方式。

电气：

- 电池规格、BMS 通信、急停链路、充电检测。
- 电源分配板、保险/熔断、软启动、掉电保护。
- 主控 SoC、交换机/网口、CAN-FD、EtherCAT、SPI MCU 的硬件连接图。
- 接地、屏蔽、EMC、线束编号。

固件：

- MCU daemon 对应 MCU 固件。
- 底盘板固件。
- 头腰电机固件。
- EtherCAT 从站固件或 ESI/PDO 描述。

标定与生产：

- DH/URDF 到实机的零位标定流程。
- 雷达/IMU/相机/手眼/超声标定流程。
- 工厂 EOL 测试项。
- 故障码体系和清错流程。
- 安全验证标准：空载、限速、单轴、双臂、底盘、重定位、避障。

## 13. 下一轮工作计划

优先级 1：

- 把当前运行态 topic/service 按模块补全到 `G2_topic_service_map.md`。
- 从 boot 日志确认 `run.sh` 动态 IP 为什么仍选 `10.42.1.101`。
- 抽取 `G2_t2_crs` 的 URDF/SRDF 关节限位、质量、惯量、碰撞体，整理成机械复刻表。

优先级 2：

- 对 EtherCAT 18 从站建立 slave -> joint -> config -> HAL motor 的映射表。
- 对 CAN-FD 头腰建立 node_id -> joint -> 电机型号 -> 方向/offset 的映射表。
- 对底盘 ecat1 两从站建立板卡 -> 四轮关节 -> PNC 命令的映射表。

优先级 3：

- 读取 GDK Python API 到 AORTA topic/service 的映射。
- 把地图数据库 `/data/dlb/dlb.db`、`/home/agi/app/data/map.pcd`、`grid_map` 的关系整理清楚。
- 形成“从零构建一台 G2-compatible 机器人”的最小可行 BOM 和软件启动方案。

## 14. 结论

目前已经不是“不了解这台机器人”的状态。我们已经能说明它怎么启动、哪些进程构成系统、主要硬件走什么总线、各模块怎么通信、定位导航怎么接到底盘、运动控制怎么接到 HAL，以及哪些现场标定影响真实效果。

但要达到“自己造出来”，还需要从软件架构理解推进到制造资料闭环：CAD、BOM、线束、电源板、驱动板、MCU/从站固件、标定工装和生产测试。下一轮应围绕这些缺口继续做结构化逆向，而不是只排单个故障。
