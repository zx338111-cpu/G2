# G2 硬件控制矩阵

日期：2026-06-04  
来源：`agi@10.20.15.152` 当前运行态、HAL/hal_lowerlimb/lidar/motion-control 日志、机器人侧配置文件。

## 1. 控制总览

```text
MotionControl / WBC
  -> HAL
    -> IgH EtherCAT Master0
      -> 双臂 14 轴 + 端板/末端
    -> CAN-FD can0
      -> 头部 3 轴
    -> CAN-FD can1
      -> 腰部 5 轴

PNC / quark_navigation
  -> /pnc/chassis_joint_cmd
  -> hal_lowerlimb
    -> EtherCAT ecat1
      -> 四轮转向/牵引底盘 + 电源板

lidar / camera / sensor services
  -> 10.42.0.x 传感器网络
  -> AORTA/Cosine topic

rhino_mcu_daemon
  -> /dev/spidev2.0
  -> MCU/底层管理链
```

## 2. 上半身 EtherCAT

当前系统 EtherCAT master：

```text
command: sudo ethercat master
phase: Operation
active: yes
slaves: 18
cycle: 1000 frames/s
lost frames: 0
main device: 02:11:22:00:00:30
```

从站表：

| Slave | 状态 | 设备 | 推断职责 |
|---:|---|---|---|
| 0 | OP | Microchip-LAN9254-EtherCAT-Junction-Box | EtherCAT junction |
| 1 | OP | Microchip-LAN9254-EtherCAT-Junction-Box - Dev D | EtherCAT junction |
| 2 | OP | CoolDrive JMDT | 左臂 joint1 / HAL motor 0 |
| 3 | OP | CoolDrive JMDT | 左臂 joint2 / HAL motor 1 |
| 4 | OP | CoolDrive JMDT | 左臂 joint3 / HAL motor 2 |
| 5 | OP | CoolDrive JMDT | 左臂 joint4 / HAL motor 3 |
| 6 | OP | CoolDrive JMDT | 左臂 joint5 / HAL motor 4 |
| 7 | OP | CoolDrive JMDT | 左臂 joint6 / HAL motor 5 |
| 8 | OP | CoolDrive JMDT | 左臂 joint7 / HAL motor 6 |
| 9 | OP | app | 左端板/末端相关 |
| 10 | OP | CoolDrive JMDT | 右臂 joint1 / HAL motor 7 |
| 11 | OP | CoolDrive JMDT | 右臂 joint2 / HAL motor 8 |
| 12 | OP | CoolDrive JMDT | 右臂 joint3 / HAL motor 9 |
| 13 | OP | CoolDrive JMDT | 右臂 joint4 / HAL motor 10 |
| 14 | OP | CoolDrive JMDT | 右臂 joint5 / HAL motor 11 |
| 15 | OP | CoolDrive JMDT | 右臂 joint6 / HAL motor 12 |
| 16 | OP | CoolDrive JMDT | 右臂 joint7 / HAL motor 13 |
| 17 | OP | app | 右端板/末端相关 |

已确认映射：

```text
HAL motor 0-6  -> EtherCAT slave 2-8
HAL motor 7-13 -> EtherCAT slave 10-16
```

当前 HAL 实际硬件：

```text
producer: tianji
wrapper: IgH EtherCAT Tianji arm
left config: /home/agi/app/conf/hal/tianji_left_arm_ccs_t1_config.yaml
right config: /home/agi/app/conf/hal/tianji_right_arm_ccs_t1_config.yaml
```

左臂：

| ID | Joint | Mode | Direction | Encoder bits | Gear | Torque coeff |
|---:|---|---|---:|---:|---:|---:|
| 3 | idx21_arm_l_joint1 | csp | 1 | 20 | 120 | 150 |
| 4 | idx22_arm_l_joint2 | csp | 1 | 20 | 120 | 150 |
| 5 | idx23_arm_l_joint3 | csp | 1 | 19 | 100 | 100 |
| 6 | idx24_arm_l_joint4 | csp | 1 | 19 | 100 | 100 |
| 7 | idx25_arm_l_joint5 | csp | 1 | 19 | 100 | 80 |
| 8 | idx26_arm_l_joint6 | csp | -1 | 19 | 100 | 80 |
| 9 | idx27_arm_l_joint7 | csp | 1 | 19 | 100 | 80 |
| 10 | end_plate | - | - | - | - | - |

右臂：

| ID | Joint | Mode | Direction | Encoder bits | Gear | Torque coeff |
|---:|---|---|---:|---:|---:|---:|
| 11 | idx61_arm_r_joint1 | csp | 1 | 20 | 120 | 150 |
| 12 | idx62_arm_r_joint2 | csp | 1 | 20 | 120 | 150 |
| 13 | idx63_arm_r_joint3 | csp | 1 | 19 | 100 | 100 |
| 14 | idx64_arm_r_joint4 | csp | 1 | 19 | 100 | 100 |
| 15 | idx65_arm_r_joint5 | csp | 1 | 19 | 100 | 80 |
| 16 | idx66_arm_r_joint6 | csp | 1 | 19 | 100 | 80 |
| 17 | idx67_arm_r_joint7 | csp | 1 | 19 | 100 | 80 |
| 18 | end_plate | - | - | - | - | - |

现场覆盖参数：

```text
/data/parameters/hardware/arm_parameters.yaml
arm_type: CRS
gear_ratio: left/right [120,120,100,100,100,100,100]
encoder_resolution: left/right [20,20,19,19,19,19,19]
effort_coefficient: left/right [150,150,100,100,80,80,80]
```

## 3. 头部 CAN-FD

选择链路：

```text
/home/agi/app/conf/hal/genie02_head_config.yaml
  -> producer juxie
  -> /home/agi/app/conf/hal/juxie_head_config_t2.yaml
```

配置：

| 项 | 值 |
|---|---|
| interface | `can0` |
| frequency | 1000 Hz |
| node_id | `[3, 2, 1]` |
| joints | `idx11_head_joint1`, `idx12_head_joint2`, `idx13_head_joint3` |
| reduction_ratio | `[101, 101, 101]` |
| gear_ratio | `[1.2, 1, 1]` |
| rectification | `[1, -1, 1]` |

日志确认：

```text
model: ZY_R48
producer: JXZN
hardware_version: V1.20
software_version: V1.16.04
```

## 4. 腰部 CAN-FD

选择链路：

```text
/home/agi/app/conf/hal/genie02_waist_config.yaml
  -> producer juxie
  -> /home/agi/app/conf/hal/juxie_waist_config_t2.yaml
```

配置：

| 项 | 值 |
|---|---|
| interface | `can1` |
| frequency | 1000 Hz |
| node_id | `[8, 7, 6, 5, 4]` |
| joints | `idx01_body_joint1` ... `idx05_body_joint5` |
| reduction_ratio | `[161, 161, 161, 161, 161]` |
| gear_ratio | `[1, 1, 1, 1, 1]` |
| rectification | `[-1, 1, -1, 1, -1]` |
| offset | `[0.541, -1.326, 0.174, 0.0, 0.0]` |

日志确认：

```text
joint 4-6: ZY_R120
joint 7-8: ZY_R120MAX
producer: JXZN
hardware_version: V1.30
software_version: V1.16.08
```

## 5. CAN-FD 总线状态

当前 `can0` 和 `can1`：

```text
state: ERROR-ACTIVE
bitrate: 1000000
dbitrate: 5000000
restart-ms: 1000
mtu: 72
tx/rx berr-counter: 0/0
```

复刻要求：

```text
主控需要可靠 CAN-FD 控制器
收发器要支持 5 Mbps data phase
需要 120 ohm 终端、电源隔离、线束屏蔽和错误恢复策略
```

## 6. 底盘与下肢 EtherCAT

启动模块：

```text
process: hal_lowerlimb
config: /home/agi/app/conf/hal/lowerlimb_config.yaml
ethercat_interface: ecat1
joint_states_frequency_hz: 200
```

底盘关节配置：

| Wheel | Steering joint | Traction joint |
|---|---|---|
| left front | idx111_chassis_lwheel_front_joint1 | idx112_chassis_lwheel_front_joint2 |
| left rear | idx121_chassis_lwheel_rear_joint1 | idx122_chassis_lwheel_rear_joint2 |
| right front | idx131_chassis_rwheel_front_joint1 | idx132_chassis_rwheel_front_joint2 |
| right rear | idx141_chassis_rwheel_rear_joint1 | idx142_chassis_rwheel_rear_joint2 |

日志确认：

```text
ecat1 init success
found 2 slaves, expected 2
state to SAFE_OP then OPERATIONAL
power board serial: g02_chassis
power board hw: 1.0.1
power board sw: 0.0.17
tractionMotorVersion: 2025.514.300
SteerMotorVersion: 1.14.06
```

PNC 控制链：

```text
quark_navigation publishes /pnc/chassis_joint_cmd
hal_lowerlimb subscribes /pnc/chassis_joint_cmd
hal_lowerlimb publishes:
  /hal/chassis_joint_state
  /hal/chassis_power_state
  /hal/usr_state
  /hal/chest_power_state
  /imu/chassis
```

## 7. 传感器网络

MID360：

| 项 | 值 |
|---|---|
| lidar IP | `10.42.0.122`, `10.42.0.123` |
| host IP | `10.42.0.101` |
| device point port | 56300 |
| device imu port | 56400 |
| host point port | 6862 |
| host imu port | 6863 |
| point topics | `/lidar/livox_front`, `/lidar/livox_back` |
| imu topics | `/imu/livox_front`, `/imu/livox_back` |

xt_chest：

| 项 | 值 |
|---|---|
| type | xintan |
| address | `10.42.0.121` |
| topic | `/lidar/xt_chest` |
| log | `/data/logs/xtlog/` |

标定：

```text
/data/parameters/sensor
  intrinsic_*.json
  extrinsic_*.json
  struct_*.json
```

## 8. 电源、MCU、时间

MCU：

```text
rhino_mcu_daemon.service active
/usr/bin/mcu_daemon /dev/spidev2.0
```

PTP：

```text
/dev/ptp0..3
/dev/ptp_clock -> /dev/ptp_xgi0 -> ptp1
rhino_ptp4l_domain0.service active
chrony.service active
```

电源相关 topic：

```text
/hal/chassis_power_state
/hal/chassis_power_ctrl
/hal/chest_power_state
/hal/chest_power_ctrl
/hal/soc_power_ctrl
/power_manager/power_mode
/power_manager/power_mode_request
/power_manager/power_mode_response
```

## 9. 待补完整制造资料

还需要：

```text
EtherCAT ESI/PDO/SDO 完整表
CoolDrive JMDT 具体型号和驱动参数
JXZN CAN-FD 协议和故障码
底盘板/电源板原理图和固件
MCU 固件和 SPI 协议
完整线束、接插件、保险、急停链路
```

这份矩阵作为排障和复刻的硬件入口，后续每次拿到更底层资料都补进来。
