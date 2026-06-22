# G2 运行态快照

采集时间：2026-06-04  
目标机器人：`agi@10.20.15.152`

## 1. 系统

```text
hostname: G2
date on robot: Thu Jun 4 15:51 CST 2026
kernel: Linux 5.10.220-rt112 PREEMPT_RT aarch64
CPU: 24 x Cortex-A78AE
app version: genie_g02_rb_2.2.0_320dcc1f_2026-04-01-09-31-20_r1.tar.gz
```

关键包：

```text
agibot_gdk 2.6.3
aorta 2.2.0
cosine_bus 3.5.0
hal 2.2.7
motion-control 0.5.18
navigation 1.1.7-hotfix3-cv410
slam 1.1.12
camera snap-collinear-0309
```

## 2. systemd 状态

```text
genie_app.service active
agibot_perfguard.service active
ethercat.service active
rhino_ptp4l_domain0.service active
rhino_mcu_daemon.service active
chrony.service active
edge-client.service active
```

`genie_app.service`：

```text
ExecStart=/bin/bash /home/agi/app/bin/run.sh
User=root
Restart=on-failure
```

## 3. 运行 scene

```text
DEFAULT_LAUNCH_SCENE=base
manifest: /home/agi/app/conf/manifest.d/base.json
```

当前 launcher 模块：

```text
hal
hal_lowerlimb
camera_service
camera_dlb
cosine_runner
teleop_it
remote_hal
dlb
dds_record
fault_manager
power_manager
monitor_app
hmi_proxy_end
lidar
pnc
slam
dr
mc
tagloc
task_manager
freespace
media_manager
gdk_service
gdk_http_server
arbitrator
corobot
```

## 4. 进程热点

高线程/核心进程：

```text
cosine_runner
camera_dlb
camera_service
genie_motion_control
slam_state_machine
quark_navigation
pico_adapter
hal
fault_manager
dlb
hal_lowerlimb
dds_record
run_corobot_app
freespace_state_machine
lidar
monitor_app
task_manager
aorta-service
fast-discovery-server
launcher
power_manager
arbitrator_runner
dr_state_machine
tagloc_state_machine
gdk_service
teleop_main_node
gdk_http_server
mcu_daemon
```

## 5. 网络

接口：

```text
wlan0       UP   10.20.15.152/24
xgi0        UP   10.42.0.101/24
xgi1        UP   10.42.12.101/24
enp1s0      DOWN 10.42.1.101/24
xgi0.80     UP   192.168.1.101/24
ecat0       UP   10.42.30.101/24
ecat1       UP   10.42.40.101/24
wlanap0     UP   10.42.6.101/24
can0        UP
can1        UP
```

监听：

```text
8849/tcp    gdk_http_server
11811/tcp   FastDDS discovery
11811/udp   FastDDS discovery
2379/tcp    AORTA client
2380/tcp    AORTA peer on 10.42.1.101
```

当前运行态发现服务：

```text
FastDDS: 10.42.1.101:11811
AORTA: http://10.42.1.101:2379
```

注意：`run.sh` 文件已经有动态 IP 选择逻辑，但本次运行态仍选择 `10.42.1.101`。下次排网络/GDK/ROS2 discovery 必须先刷新这个事实。

## 6. 日志入口

```text
/data/logs/latest -> /data/logs/boot00000153
```

本次关键日志文件：

```text
launcher.log.INFO.*
gdk_service.log.INFO.*
hal.log.INFO.*
hal_lowerlimb.log.INFO.*
lidar.log.INFO.*
quark_navigation.log.INFO.*
motion-control/motion-control_20260604_151850.log
motion-control/wbc_20260604_151850.log
fault_manager.log.INFO.*
slam/dr/tagloc/freespace related logs
```

## 7. 近期重定位状态

已修复：

```text
stale aorta-service cleanup
SLAM env 加入 COSINE bus config path
SLAM 高频输入 queue depth for imu/dr/tf 从 5 提到 50
gdk_service/lidar/dr/slam/motion-control 统一到 AORTA discovery
```

剩余现象：

```text
SLAM 能收到重定位请求
IMU/DR/TF/LiDAR 数据进入算法
GICP/NDT score 约 0.66-0.72
threshold min_init_score=0.8
Relocalization Is Failed
```

当前判断：

```text
更像当前现场环境/初始位姿和 map id=6 不匹配。
不要直接降低 min_init_score 制造假定位。
```

## 8. 下次刷新命令

```bash
date
hostname
cat /home/agi/app/.version
systemctl status genie_app.service --no-pager -l
ip -br addr
ss -lntup | grep -E '2379|2380|8849|11811'
ps -e -o pid,ppid,nlwp,stat,pcpu,pmem,comm,args --sort=-nlwp | head -80
find /data/logs/latest/ -maxdepth 2 -type f | sort | sed -n '1,200p'
grep -R "ERROR\\|WARN\\|Failed\\|fault\\|DataLoss\\|type mismatch" /data/logs/latest -n | tail -200
```
