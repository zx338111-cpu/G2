# G2 诊断 Runbook

目标：用最少风险确认机器人后台服务、通信、硬件总线和故障位置。除非现场安全确认，不执行重启、清错、上电、运动。

## 1. 登录和环境

```bash
ssh agi@10.20.15.152
```

机器人侧 GDK 环境：

```bash
cd /home/agi/app
source /home/agi/app/env.sh /home/agi/app
```

如果 SSH 不通，先在开发机确认：

```bash
ip route get 10.20.15.152
ping -c 2 -W 2 10.20.15.152
```

本轮曾出现路由存在但 ping 全丢包，后续恢复。若复现，先等 Wi-Fi 链路恢复或检查机器人 `wlan0`。

## 2. 一眼确认系统状态

只读：

```bash
date
hostname
uname -a
cat /home/agi/app/.version
head -40 /home/agi/app/.pkg_version
systemctl is-active genie_app.service agibot_perfguard.service ethercat.service rhino_ptp4l_domain0.service chrony.service
```

进程：

```bash
ps -e -o pid,ppid,nlwp,stat,pcpu,pmem,comm,args --sort=-nlwp | head -80
pgrep -a -f 'launcher|aorta|fastdds|gdk|hal|motion|quark|slam|dr|fault|camera|lidar|remote|task'
```

日志入口：

```bash
ls -l /data/logs/latest
tail -200 /data/logs/latest/hal.INFO
tail -200 /data/logs/latest/fault_manager.INFO
tail -200 /data/logs/latest/quark_navigation.INFO
tail -200 /data/logs/latest/motion-control/motion-control.log
```

HTTP 日志服务：

```text
http://10.20.15.152:8849/latest/fault_manager.INFO
http://10.20.15.152:8849/latest/quark_navigation.INFO
http://10.20.15.152:8849/latest/hal.INFO
```

## 3. 启动链路检查

```bash
systemctl status genie_app.service --no-pager -l
systemctl cat genie_app.service
sed -n '1,260p' /home/agi/app/bin/run.sh
sed -n '1,200p' /home/agi/app/conf/sys/run.conf
ls -l /home/agi/app/conf/manifest.d
```

重点看：

- `run.sh` 是否启动 AORTA、FastDDS、launcher。
- `LOCATOR_IP` 是否绑定到当前可达 IP。当前文件已补丁为动态选择，运行态需下次重启验证。
- `DEFAULT_LAUNCH_SCENE` 和 launcher 实际使用的 manifest 是否一致。当前现场为 `base`。

## 4. 网络和中间件

```bash
ip -br addr
ip route
ss -lntup | grep -E '2379|2380|8849|11811'
pgrep -a -f 'aorta|fastdds|gdk_http_server|gdk_service'
```

判断：

- SSH 通但 GDK/ROS2 发现失败：优先怀疑 FastDDS/AORTA 绑定到错误 IP。
- 有线调试默认 `10.42.1.101`，Wi-Fi 访问是 `10.20.15.x`，两者不要混。

## 5. EtherCAT

只读检查：

```bash
sudo ethercat master
sudo ethercat slaves
sudo ethercat states
cat /etc/ethercat.conf
ls -l /dev/EtherCAT*
```

单关节故障定位：

```bash
sudo ethercat upload -p <slave> -t uint16 0x6041 0x00
sudo ethercat upload -p <slave> -t uint16 0x603f 0x00
```

G2 手臂映射：

```text
HAL motor 0-6   -> EtherCAT slave 2-8
HAL motor 7-13  -> EtherCAT slave 10-16
```

常见正常/异常：

```text
0x9737 / 0x0000   驱动正常
0x0638 / 0xff38   锁存故障，需要按 runbook 清错
0xffff            通信或状态不可用，继续查 HAL/EtherCAT
```

清错和 `genie_app.service` 重启会改变机器人状态，必须现场安全确认后再做。

## 6. CAN / PTP / 时间

```bash
ip -details link show can0
ip -details link show can1
ls -l /dev/ptp* /dev/ptp_clock 2>/dev/null
timedatectl
chronyc sources -v
systemctl status chrony.service rhino_ptp4l_domain0.service --no-pager -l
```

经验：

- 系统时间从 1970 跳到真实时间，会影响 HAL/驱动时间戳链路。
- `/dev/ptp_clock` 缺失会让部分 quark/aorta 组件找不到默认 PTP 设备。
- `/dev/ptp0..3` 如果是 `root:root 0600`，`agi` 运行 GDK 会提示 `No valid PTP device found`；当前已修成 `root:plugdev 0660` 并加 udev 规则。
- 时间修复后仍要看 HAL 和 motion-control 是否持续无 error。

## 7. GDK 只读健康检查

不要发运动命令，只读：

```python
import time
import agibot_gdk

ret = agibot_gdk.gdk_init()
print("gdk_init", ret)
robot = agibot_gdk.Robot()
pnc = agibot_gdk.Pnc()
time.sleep(1)

print("motion", robot.get_motion_control_status())
print("whole_body", robot.get_whole_body_status())
print("chassis_power", robot.get_chassis_power_state())
print("task", pnc.get_task_state())
js = robot.get_joint_states()
print("joint nums", js.get("nums"))
for s in js.get("states", []):
    if s.get("error_code", 0):
        print(s.get("name"), hex(s.get("error_code")))

agibot_gdk.gdk_release()
```

判断：

- `whole_body_status` 的 arm/chassis error 为 0 不够，仍要看 `get_joint_states()`。
- `motion_control_status.mode == 5` 通常表示 G2 伺服模式正常，但仍要结合 error code 和日志。

## 8. 底盘故障路径

优先顺序：

1. 看 `get_chassis_power_state()`：电机上电、急停、底盘电源板、电池/充电。
2. 看 `pnc.get_task_state()`：是否已有任务占用控制权。
3. 看 `quark_navigation.INFO`：GDK 指令是否连上 publisher/subscriber。
4. 看 DR 日志：relative move 是否有里程计。
5. 看 `fault_manager.INFO`：底盘四轮是否当前断联，注意时间戳是否旧 boot。

经验：

```text
relative_move(NaviReq) 优先于 move_chassis(Twist)
relative_move 走任务系统和 DR 闭环
move_chassis 需要抢控制权，容易被任务压制
连续任务 state=9 后留 0.5s 缓冲
```

## 9. 手臂/关节故障路径

优先顺序：

1. HAL 最新日志搜索 `motor N err code`。
2. 根据 HAL motor 映射 EtherCAT slave。
3. 读 `0x6041` 状态字和 `0x603f` 错误码。
4. GDK `get_joint_states()` 找具体关节名和 error。
5. 若清错后复发，看时间同步、HAL enable 时序、motion-control 错误。
6. 最后用 EtherCAT、GDK、HAL、受控小幅动作四层验证。

不要只根据 `joint_control_request()` 返回成功判断手臂真的动了。

## 10. 消息类型不匹配

排查命令：

```bash
grep -R "type mismatch\|msg type\|SetLoad\|left_ee_command\|right_ee_command" /data/logs/latest -n | tail -200
find /home/agi/app -path '*genie_msgs*' -o -name '*SetLoad*'
```

已知风险：

- `/wbc/left_ee_command` 和 `/wbc/right_ee_command` 的 publisher/subscriber 消息类型不一致。
- `SetLoadRequest`/`SetLoadRequst` 拼写差异来自包内 ABI 与外部客户端不一致。

处理原则：

- 先确认哪个进程在发布错误类型。
- 如果是外部脚本，修脚本。
- 如果是包内二进制互相不一致，不能用 Python 临时绕，需要从安装包版本、proto、manifest、二进制 ABI 排。

## 11. 修改前的最低安全规则

任何机器人侧修改都应遵循：

```text
backup -> minimal patch -> syntax/compile check -> service/log validation -> controlled hardware validation
```

必须现场确认后才执行：

- `systemctl restart genie_app.service`
- EtherCAT 清错、INIT/OP 状态切换
- 任意 GDK 运动命令
- 电源控制、急停相关命令
