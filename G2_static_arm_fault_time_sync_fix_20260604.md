# G2 静止自报手臂 0xFF38 故障根因与彻底修复记录

日期：2026-06-04  
机器人：`agi@10.20.15.60`  
故障主题：机器人未执行手臂动作时，手臂 EtherCAT 驱动器锁存 `0xff38`，GDK 显示手臂关节错误。

## 1. 结论

这次故障从证据看是软件/系统启动时序和时间同步问题触发的驱动器保护，不是机械臂硬件永久损坏。

证据：

- 清错和 EtherCAT INIT/OP 重建后，所有手臂驱动器可恢复到 `status=0x9737 / error=0x0000`。
- GDK 恢复为 `motion mode=5`、`error_code=0`、`joint_error_count=0`。
- 故障发生前 HAL 已经在错误时间域 `1970-01-01` 启动并进入 enable，之后系统时间跳到 `2026-06-04`，紧接着 HAL 进入 ESTOP 并出现 `0xff38`。
- `share_time_sync.service` 早期执行时 `phc2sys` 使用的接口尚未 ready，日志出现 `unknown clock xfi0.20`、`unknown clock xfi2.10g`，但服务仍返回 success。
- `agi` 用户没有 `/dev/ptp*` 访问权限，导致用户态 GDK 初始化提示 `No valid PTP device found`。

因此本次根因链路为：

```text
系统启动初期时间仍在 1970
        ->
time-sync oneshot 太早执行且失败后仍成功
        ->
genie_app/HAL 在时间未稳定时启动并 enable motor operator
        ->
系统时间后续发生大步跳变到 2026
        ->
HAL/驱动器控制时间戳链路异常
        ->
手臂驱动器锁存 0xff38
```

## 2. 故障现象

HAL 持续刷：

```text
motor 2 err code 0xff38
```

EtherCAT 首次诊断时：

```text
slave 4 motor 2 status=0x0638 error=0xff38
slave 8 motor 6 status=0x0638 error=0xff38
```

GDK 侧：

```text
idx23_arm_l_joint3 error_code=0xff38
idx27_arm_l_joint7 error_code=0xff38
其他手臂关节伴随 0xffff
motion mode=0
joint_error_count=14
```

其中映射关系为：

```text
HAL motor 0-6  -> EtherCAT slave 2-8
HAL motor 7-13 -> EtherCAT slave 10-16
```

所以：

```text
motor 2 -> slave 4 -> idx23_arm_l_joint3
motor 6 -> slave 8 -> idx27_arm_l_joint7
```

## 3. 根因证据

### 3.1 genie_app 早于稳定时间域启动

故障 boot 的 HAL 日志文件：

```text
/data/logs/boot00000242/hal.log.INFO.19700101-080050.4185.0
```

关键日志：

```text
I0101 08:01:30.560867 igh_ecat_master.cpp:481] [hal]All 16 slaves reached Op state successfully
I0101 08:01:30.661113 igh_ecat_master.cpp:516] [hal]All motors ready, motor operator enable.

E0604 08:24:34.351516 igh_ecat_master.cpp:491] [hal]motor 2 status word error -0x7ac1
E0604 08:24:34.351560 igh_ecat_master.cpp:491] [hal]motor 6 status word error -0x7ac1
E0604 08:24:34.351567 igh_ecat_master.cpp:505] [hal][ESTOP]Has reached OP state, but motor operator disable
I0604 08:24:35.252311 igh_ecat_master.cpp:534] [hal]motor 6 err code 0xff38
I0604 08:24:41.857759 igh_ecat_master.cpp:534] [hal]motor 2 err code 0xff38
```

这说明机器人应用/HAL 在 `1970-01-01` 已经启用 motor operator，后续系统时间切换到 `2026-06-04` 后立即触发手臂驱动器保护。

### 3.2 time sync oneshot 执行太早且失败后仍成功

`share_time_sync.service` 状态中出现：

```text
phc2sys: unknown clock xfi0.20: No such device
phc2sys: unknown clock xfi2.10g: No such device
```

原始 `/usr/bin/share_time_sync.sh` 问题：

- 硬编码直接执行 `phc2sys -c xfi0.20` 和 `phc2sys -c xfi2.10g`
- 不等待接口创建完成
- `phc2sys` 失败后又执行 `timedatectl set-ntp true`，最终服务仍显示 success

实际当前接口在系统稳定后是存在的：

```text
xfi0.20@mgbe0_0 UP
xfi2.10g@mgbe2_0 UP
```

所以问题不是接口不存在，而是启动时机太早且没有 retry/失败保护。

### 3.3 agi 用户无法访问 PTP 设备

修复前：

```text
/dev/ptp_xgi0 -> /dev/ptp0
/dev/ptp0 crw------- root root
/dev/ptp1 crw------- root root
```

`agi` 用户运行 GDK 时：

```text
Default PTP device /dev/ptp_xgi0 is not valid, try /dev/ptp1
Default PTP device /dev/ptp1 is not valid.
No valid PTP device found
```

这是权限问题：`agi` 属于 `plugdev`，但 `/dev/ptp*` 没有给 `plugdev` 访问权限。

## 4. 当场恢复步骤

### 4.1 首次清错

先对故障从站 `slave 4` 和 `slave 8` 执行：

```bash
sudo ethercat download -p 4 -t uint16 0x3002 0x00 0x0000
sudo ethercat download -p 8 -t uint16 0x3002 0x00 0x0000
sudo ethercat states -p 4 INIT
sudo ethercat states -p 8 INIT
sudo ethercat states -p 4 OP
sudo ethercat states -p 8 OP
```

第一次清错后两个从站仍回到 `0xff38`，说明 HAL/驱动器故障循环没有完全打断。

### 4.2 重启应用栈后再次清错

```bash
sudo systemctl restart genie_app.service
```

重启后出现伴随 `0xff51`，根故障仍是 `slave 4`、`slave 8` 的 `0xff38`。再次清错：

```bash
sudo ethercat download -p 4 -t uint16 0x3002 0x00 0x0000
sudo ethercat download -p 8 -t uint16 0x3002 0x00 0x0000
sudo ethercat states -p 4 INIT
sudo ethercat states -p 8 INIT
sudo ethercat states -p 4 OP
sudo ethercat states -p 8 OP
```

恢复结果：

```text
all arm slaves status=0x9737 error=0x0000
GDK motion mode=5
GDK joint_error_count=0
```

## 5. 持久化修复

### 5.1 备份

机器人上创建备份目录：

```text
/home/agi/g2_fix_backup_20260604_0845
```

备份文件：

```text
/usr/bin/share_time_sync.sh
/usr/bin/share_phc2sys.sh
/usr/bin/share_ptp4l_domain2.sh
/etc/systemd/system/genie_app.service
/etc/systemd/system/share_time_sync.service
/etc/systemd/system/share_phc2sys.service
/etc/systemd/system/share_ptp4l_domain2.service
```

### 5.2 修复 share_time_sync.sh

文件：

```text
/usr/bin/share_time_sync.sh
```

修改点：

- 等待 `xfi0.20`、`xfi2.10g` 出现，最长 60 秒
- `phc2sys` 立即失败时返回非 0
- `timeout 30` 正常到期返回 `124` 时视为 one-shot 成功
- 不再让失败的 PHC 同步被后续 NTP 命令掩盖

### 5.3 修复 share_phc2sys.sh

文件：

```text
/usr/bin/share_phc2sys.sh
```

修改点：

- 启动前等待 `xfi0.20`、`xfi2.10g`，最长 90 秒
- 启动两个 `phc2sys` 子进程
- 任意一个子进程退出时，脚本退出，让 systemd `Restart=always` 接管重启

### 5.4 修复 share_ptp4l_domain2.sh

文件：

```text
/usr/bin/share_ptp4l_domain2.sh
```

修改点：

- 启动前等待 `xfi0.20`、`xfi2.10g`，最长 90 秒
- 启动两个 `ptp4l` 子进程
- 任意一个子进程退出时，脚本退出，让 systemd `Restart=always` 接管重启

### 5.5 增加 genie_app 启动前 guard

新增文件：

```text
/usr/local/sbin/g2_time_sync_guard.sh
```

检查项：

- 当前年份必须大于等于 2025，避免 `1970` 时间域启动
- `xfi0.20`、`xfi2.10g` 必须存在
- `share_phc2sys.service` 必须 active
- `share_ptp4l_domain2.service` 必须 active

如果 120 秒内不满足条件，guard 返回失败，`genie_app` 不启动。

### 5.6 增加 systemd drop-in

新增文件：

```text
/etc/systemd/system/share_time_sync.service.d/10-network-ready.conf
/etc/systemd/system/share_phc2sys.service.d/10-network-ready.conf
/etc/systemd/system/share_ptp4l_domain2.service.d/10-network-ready.conf
/etc/systemd/system/genie_app.service.d/10-time-sync-guard.conf
```

作用：

- 时间同步相关服务在 `network-online.target` 和 `ethernet_init.service` 后启动
- `genie_app` 依赖 `share_phc2sys.service`、`share_ptp4l_domain2.service`
- `genie_app` 启动前执行 `/usr/local/sbin/g2_time_sync_guard.sh`

验证：

```text
genie_app.service:
  Drop-In: /etc/systemd/system/genie_app.service.d/10-time-sync-guard.conf
  Process: ExecStartPre=/usr/local/sbin/g2_time_sync_guard.sh (status=0/SUCCESS)
  Active since Thu 2026-06-04 08:50:41 CST
```

新的 HAL 日志文件：

```text
/data/logs/latest/hal.log.INFO.20260604-085049.167148.0
```

说明这次应用栈从正确时间域启动，不再是 `19700101`。

### 5.7 修复 agi 用户访问 PTP 权限

新增 udev 规则：

```text
/etc/udev/rules.d/70-g2-ptp-permissions.rules
```

内容：

```text
KERNEL=="ptp[0-9]*", GROUP="plugdev", MODE="0660"
```

并立即应用当前设备权限：

```bash
sudo chgrp plugdev /dev/ptp0 /dev/ptp1 /dev/ptp2 /dev/ptp3
sudo chmod 0660 /dev/ptp0 /dev/ptp1 /dev/ptp2 /dev/ptp3
```

修复后：

```text
/dev/ptp0 crw-rw---- root plugdev
/dev/ptp1 crw-rw---- root plugdev
```

GDK 验证：

```text
PTP device /dev/ptp_xgi0 opened successfully
gdk_init: GDKRes.kSuccess
motion: 5 0 ''
joint_error_count: 0
```

## 6. 修复后验证

### 6.1 EtherCAT

最终扫描：

```text
slave 2  motor 0  status=0x9737 error=0x0000
slave 3  motor 1  status=0x9737 error=0x0000
slave 4  motor 2  status=0x9737 error=0x0000
slave 5  motor 3  status=0x9737 error=0x0000
slave 6  motor 4  status=0x9737 error=0x0000
slave 7  motor 5  status=0x9737 error=0x0000
slave 8  motor 6  status=0x9737 error=0x0000
slave 10 motor 7  status=0x9737 error=0x0000
slave 11 motor 8  status=0x9737 error=0x0000
slave 12 motor 9  status=0x9737 error=0x0000
slave 13 motor 10 status=0x9737 error=0x0000
slave 14 motor 11 status=0x9737 error=0x0000
slave 15 motor 12 status=0x9737 error=0x0000
slave 16 motor 13 status=0x9737 error=0x0000
```

### 6.2 HAL

重启验证过程中 HAL 在初始化中有短暂 `0xff38`，随后自动清除：

```text
I0604 08:51:32.782306 igh_ecat_master.cpp:481] [hal]All 16 slaves reached Op state successfully
I0604 08:51:32.882390 igh_ecat_master.cpp:516] [hal]All motors ready, motor operator enable.
I0604 08:51:32.882405 igh_ecat_master.cpp:549] [hal]All motor errors cleared
```

截至：

```text
2026-06-04 08:54:17 CST
```

最后一条 `err code` 仍停在：

```text
2026-06-04 08:51:32
```

没有继续刷新。

### 6.3 GDK

```text
PTP device /dev/ptp_xgi0 opened successfully
motion: 5 0 ''
joint_error_count: 0
```

未执行远程机械臂动作测试，因为 SSH 无法确认现场周边安全和机械干涉。

## 7. 回滚方法

如果新配置导致启动异常，可以恢复备份：

```bash
sudo cp -a /home/agi/g2_fix_backup_20260604_0845/share_time_sync.sh /usr/bin/share_time_sync.sh
sudo cp -a /home/agi/g2_fix_backup_20260604_0845/share_phc2sys.sh /usr/bin/share_phc2sys.sh
sudo cp -a /home/agi/g2_fix_backup_20260604_0845/share_ptp4l_domain2.sh /usr/bin/share_ptp4l_domain2.sh

sudo rm -f /usr/local/sbin/g2_time_sync_guard.sh
sudo rm -f /etc/udev/rules.d/70-g2-ptp-permissions.rules
sudo rm -rf /etc/systemd/system/genie_app.service.d
sudo rm -rf /etc/systemd/system/share_time_sync.service.d
sudo rm -rf /etc/systemd/system/share_phc2sys.service.d
sudo rm -rf /etc/systemd/system/share_ptp4l_domain2.service.d

sudo systemctl daemon-reload
sudo systemctl restart share_phc2sys.service
sudo systemctl restart share_ptp4l_domain2.service
sudo systemctl restart genie_app.service
```

回滚后仍需重新检查 EtherCAT 和 GDK 状态。

## 8. 后续建议

1. 以后不要在系统刚开机、时间还可能是 `1970` 时启动手臂控制脚本。
2. 用户脚本必须在 `gdk_init()` 后等待 2 秒，并先检查 `joint_error_count`。
3. 不要 `kill -9` 手臂控制脚本；脚本必须在 `finally` 中调用 `agibot_gdk.gdk_release()`。
4. 如果再次出现静止自报 `0xff38`，先查：

```bash
systemctl status genie_app.service share_phc2sys.service share_ptp4l_domain2.service --no-pager -l
timedatectl status
chronyc tracking
grep -E 'err code|ESTOP|status word|All motor|operator' /data/logs/latest/hal.log.INFO.* | tail -120
```

5. 如果 EtherCAT 恢复但 GDK 仍提示 `No valid PTP device found`，优先检查：

```bash
namei -l /dev/ptp_xgi0 /dev/ptp1
id agi
```

期望 `/dev/ptp*` 为：

```text
crw-rw---- root plugdev
```

## 9. 本次最终状态

```text
genie_app: active, ExecStartPre guard 成功
share_phc2sys: active
share_ptp4l_domain2: active
GDK PTP: /dev/ptp_xgi0 opened successfully
EtherCAT arm slaves: all status=0x9737 error=0x0000
GDK: motion mode=5, error_code=0, joint_error_count=0
HAL: 08:51:32 后无新增 err code
```

结论：本次已经完成软件层面的持久化修复。后续如果在无时间跳变、PTP 正常、启动 guard 生效的情况下仍频繁出现同一从站 `0xff38`，再转向检查该关节驱动器、供电、EtherCAT 线束和接插件。

## 10. 现场低速动作验证

日期：2026-06-04 09:01-09:04 CST  
现场条件：用户在现场观察，确认机械臂周边空间充足。  
测试策略：只对历史故障关节做低速小幅往返，不运行会把全身关节发到 0 位的官方示例。

动作参数：

```text
joint delta: 0.008 rad
joint velocity: 0.012
life_time: 2.0
```

### 10.1 动作前状态

```text
PTP device /dev/ptp_xgi0 opened successfully
gdk_init: GDKRes.kSuccess
motion: 5 0 ''
joint_error_count: 0
idx23_arm_l_joint3 pos=-1.234761896189423 err=0x0
idx27_arm_l_joint7 pos=0.38660606207234277 err=0x0
```

### 10.2 idx23_arm_l_joint3

```text
before: pos=-1.2347621358739207
target: pos=-1.2267621358739207
move_ret: 0
after move: pos=-1.226761946696101 error=0x0
return_ret: 0
after return: pos=-1.2347617763471737 error=0x0
motion after: 5 0 ''
```

结果：`idx23_arm_l_joint3` 低速小幅移动和回原位成功，无新增错误。

### 10.3 idx27_arm_l_joint7

```text
before: pos=0.3866059422300937
target: pos=0.3946059422300937
move_ret: 0
after move: pos=0.39460816872614746 error=0x0
return_ret: 0
after return: pos=0.3866035453851126 error=0x0
motion after: 5 0 ''
```

结果：`idx27_arm_l_joint7` 低速小幅移动和回原位成功，无新增错误。

### 10.4 动作后验证

GDK：

```text
motion: 5 0 ''
arm_control: False False
arm_error: 0 0
joint_error_count: 0
idx23_arm_l_joint3 pos=-1.2347620160316717 err=0x0
idx27_arm_l_joint7 pos=0.3866059422300937 err=0x0
```

EtherCAT：

```text
slave 2  motor 0  status=0x9737 error=0x0000
slave 3  motor 1  status=0x9737 error=0x0000
slave 4  motor 2  status=0x9737 error=0x0000
slave 5  motor 3  status=0x9737 error=0x0000
slave 6  motor 4  status=0x9737 error=0x0000
slave 7  motor 5  status=0x9737 error=0x0000
slave 8  motor 6  status=0x9737 error=0x0000
slave 10 motor 7  status=0x9737 error=0x0000
slave 11 motor 8  status=0x9737 error=0x0000
slave 12 motor 9  status=0x9737 error=0x0000
slave 13 motor 10 status=0x9737 error=0x0000
slave 14 motor 11 status=0x9737 error=0x0000
slave 15 motor 12 status=0x9737 error=0x0000
slave 16 motor 13 status=0x9737 error=0x0000
```

HAL：

```text
current time: 2026-06-04 09:04:09 CST
latest hal log: /data/logs/latest/hal.log.INFO.20260604-085049.167148.0
last err code remained at: 2026-06-04 08:51:32
```

结论：现场低速小幅动作验证通过。两个历史故障关节均能按命令移动并回原位，动作后 GDK、EtherCAT、HAL 均无新增故障。
