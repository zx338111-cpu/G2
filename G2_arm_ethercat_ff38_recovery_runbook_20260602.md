# G2 手臂 EtherCAT 0xFF38 故障排查与修复 Runbook

版本：2026-06-02  
适用对象：Agibot G2 机器人手臂关节故障、双臂无法使能、GDK 控制返回正常但手臂不动、HAL 持续报 `motor N err code 0xff38` 或相关 EtherCAT 驱动器故障。

本文档基于一次真实修复记录编写。本次故障点为：

- HAL：`motor 2 err code 0xff38`
- EtherCAT：`slave 4`
- GDK 关节：`idx23_arm_l_joint3`
- 修复结果：EtherCAT 全部手臂驱动器恢复 `status=0x9737 / error=0x0000`，GDK 关节错误数为 0，小幅动作验证通过

---

## 1. 先看结论

这类问题不要先改策略代码，也不要只看 GDK 接口返回值。正确判断链路是：

1. 看 HAL 最新日志是否持续刷 `motor N err code ...`
2. 把 `HAL motor N` 映射到 `EtherCAT slave`
3. 用 `ethercat upload` 读取从站状态字 `0x6041` 和错误码 `0x603f`
4. 用 GDK `get_joint_states()` 找到具体关节
5. 对故障从站执行 `CW2 清零 + INIT -> OP`
6. 如果回 OP 后马上复发，重启 `genie_app.service`，等 HAL 重建后再对同一从站清错
7. 最后必须用 EtherCAT、GDK、HAL 日志和小幅实际动作四层验证

本次第一次只对 `slave 4` 清错后，HAL 确实触发了清错序列，但 `slave 4` 回 OP 后立刻重报 `0xff38`。重启 `genie_app.service` 后，在新的 HAL 周期里再次对 `slave 4` 执行清错，故障才彻底解除。

---

## 2. G2 机器人系统架构流程

### 2.1 控制链路

```text
用户脚本 / 任务程序 / HMI
        |
        v
GDK Python API: agibot_gdk
        |
        v
gdk_service / gdk_http_server
        |
        v
genie_motion_control
        |
        v
HAL: /home/agi/app/bin/hal
        |
        v
EtherCAT master: igh_ecat_master / EtherCAT-OP
        |
        v
CoolDrive / JuXie 伺服驱动器
        |
        v
G2 关节电机
```

### 2.2 状态与故障上报链路

```text
CoolDrive / JuXie 驱动器
        |
        v
EtherCAT SDO/PDO 状态
        |
        v
HAL 日志: /data/logs/latest/hal.log.INFO.*
        |
        +----> fault_manager: /data/logs/latest/fault_manager.log.INFO.*
        |
        +----> GDK 状态接口:
              - robot.get_joint_states()
              - robot.get_whole_body_status()
              - robot.get_motion_control_status()
```

### 2.3 systemd 服务与关键进程

顶层服务：

```bash
systemctl status genie_app.service --no-pager -l
```

常见关键进程：

```text
/home/agi/app/bin/run_corobot_app
/home/agi/app/bin/gdk_service
/home/agi/app/bin/gdk_http_server
/home/agi/app/bin/hal
/home/agi/app/bin/fault_manager
/home/agi/app/bin/motion-control/bin/genie_motion_control
```

GDK Python 环境入口：

```bash
cd /home/agi/app
source /home/agi/app/env.sh /home/agi/app
```

`env.sh` 主要设置：

- `LD_LIBRARY_PATH`
- `PATH`
- `PYTHONPATH=/home/agi/app/gdk/lib`
- `APP_CONF_PATH=/home/agi/app/gdk/config/app_conf.json`

---

## 3. HAL motor 与 EtherCAT slave 映射

手臂相关 EtherCAT 从站为：

```text
slave 2 3 4 5 6 7 8 10 11 12 13 14 15 16
```

映射规则：

```text
HAL motor 0-6   -> EtherCAT slave 2-8
HAL motor 7-13  -> EtherCAT slave 10-16
```

也就是：

| HAL motor | EtherCAT slave | 本次相关说明 |
|---:|---:|---|
| 0 | 2 | 手臂驱动器 |
| 1 | 3 | 手臂驱动器 |
| 2 | 4 | 本次故障，GDK 关节 `idx23_arm_l_joint3` |
| 3 | 5 | 手臂驱动器 |
| 4 | 6 | 手臂驱动器 |
| 5 | 7 | 手臂驱动器 |
| 6 | 8 | 手臂驱动器 |
| 7 | 10 | 手臂驱动器 |
| 8 | 11 | 手臂驱动器 |
| 9 | 12 | 手臂驱动器 |
| 10 | 13 | 手臂驱动器 |
| 11 | 14 | 手臂驱动器 |
| 12 | 15 | 手臂驱动器 |
| 13 | 16 | 手臂驱动器 |

---

## 4. 状态码速查

| 值 | 含义 | 处理 |
|---|---|---|
| `0x9737` | Operation Enabled，驱动器正常使能 | 正常 |
| `0x0000` | 错误码为 0，无错误 | 正常 |
| `0x0638` / `0x0438` / `0x8638` | 状态字 fault 位相关，驱动器未正常使能 | 需要清错 |
| `0xffff` | 启动阶段或总线保护后的通信/状态不可用 | 若持续存在，需要继续查 HAL/EtherCAT |
| `0xff38` | 本次根故障码，驱动器锁存故障 | 优先清故障从站 |
| `0xff51` | 重启或 ESTOP 联动后的暂态/伴随错误 | 看是否随 HAL 恢复自动清除 |

判断驱动器是否 fault 的快速规则：

```text
status_word & 0x0008 != 0
```

---

## 5. 本次真实故障时间线

### 5.1 故障现象

HAL 最新日志持续刷：

```text
I0602 11:09:57.548085  6813 igh_ecat_master.cpp:534] [hal]motor 2 err code 0xff38
I0602 11:09:59.549520  6813 igh_ecat_master.cpp:534] [hal]motor 2 err code 0xff38
I0602 11:10:01.550931  6813 igh_ecat_master.cpp:534] [hal]motor 2 err code 0xff38
```

EtherCAT 扫描确认：

```text
slave 4 (HAL motor 2): status=0x0638 error=0xff38
```

GDK 侧确认：

```text
motion_control_status: mode=0, error_code=0, error_msg=''
whole_body_status:
  right_arm_control=False
  left_arm_control=False
  right_arm_error=0
  left_arm_error=0
joint_states:
  idx23_arm_l_joint3 error_code=0xff38
  其他手臂关节多为 0xffff
```

注意：`right_arm_error=0` 和 `left_arm_error=0` 不代表手臂能动。真实故障在 HAL/EtherCAT 层，必须看 `get_joint_states()` 和 HAL 日志。

### 5.2 根因证据

HAL 日志中，在持续 `0xff38` 之前出现：

```text
E0602 08:59:42.828745  juxie_wrapper.cpp:382] [hal]timestamp_ns: ..., joints_info_[0].time_stamp: ...
E0602 08:59:42.828783  juxie_wrapper.cpp:393] [hal]set error code 262144
E0602 09:02:58.882143  igh_ecat_master.cpp:505] [hal][ESTOP]Has reached OP state, but motor operator disable
I0602 09:03:04.286240  igh_ecat_master.cpp:534] [hal]motor 2 err code 0xff38
```

这说明故障不是凭空出现的随机现象，而是：

```text
控制/反馈时间戳跳变
        ->
juxie_wrapper 设置 error code 262144
        ->
HAL 进入 ESTOP / motor operator disable
        ->
slave 4 驱动器锁存 0xff38
        ->
GDK 侧手臂关节出现 0xff38 / 0xffff
```

可能触发时间戳跳变的常见原因：

- GDK/Python 控制脚本异常退出、被 kill、Ctrl+C 后没有正常 release
- 控制脚本刚启动就发命令，未等待 GDK 和时间同步稳定
- PTP/NTP 或系统时间发生跳变
- 控制循环长时间卡顿后继续发送旧时间戳/不连续时间戳命令
- 多个控制源同时接入或控制权切换不干净

本次 GDK 初始化时也多次出现：

```text
Default PTP device /dev/ptp_xgi0 is not valid, try /dev/ptp1
Default PTP device /dev/ptp1 is not valid.
No valid PTP device found
```

这条日志不能单独证明就是根因，但它和“时间戳跳变触发保护”属于同一风险方向，后续如果频繁复发，应重点检查时间同步链路。

---

## 6. 快速排查流程

### 6.1 确认服务是否运行

```bash
systemctl is-active genie_app.service
systemctl status genie_app.service --no-pager -l | sed -n '1,80p'
```

确认关键进程：

```bash
ps -eo pid,lstart,cmd | grep -E '/home/agi/app/bin/(run_corobot_app|hal$|gdk_service|fault_manager)|genie_motion_control' | grep -v grep
```

如果服务和进程都正常，不要急着重启。先看 HAL 和 EtherCAT。

### 6.2 看 HAL 最新错误

```bash
HAL_LOG=$(ls -t /data/logs/latest/hal.log.INFO.* 2>/dev/null | head -1)
echo "$HAL_LOG"
tail -80 "$HAL_LOG"
```

只看持续重复的非启动错误：

```bash
grep 'err code' "$HAL_LOG" | grep -v '0xffff' | tail -20
```

### 6.3 扫描手臂 EtherCAT 从站

```bash
for slave in 2 3 4 5 6 7 8 10 11 12 13 14 15 16; do
  SW=$(sudo ethercat upload -p "$slave" -t uint16 0x6041 0x00 2>/dev/null | awk '{print $1}')
  EC=$(sudo ethercat upload -p "$slave" -t uint16 0x603f 0x00 2>/dev/null | awk '{print $1}')
  if [ "$slave" -le 8 ]; then
    MOTOR=$((slave - 2))
  else
    MOTOR=$((slave - 3))
  fi
  echo "slave $slave motor $MOTOR status=$SW error=$EC"
done
```

正常输出应全部接近：

```text
status=0x9737 error=0x0000
```

本次故障输出为：

```text
slave 4 motor 2 status=0x0638 error=0xff38
```

### 6.4 用 GDK 确认具体关节

```bash
cd /home/agi/app
source /home/agi/app/env.sh /home/agi/app
python3 - <<'PY'
import time, sys
import agibot_gdk

ret = agibot_gdk.gdk_init()
print("gdk_init:", ret)
if ret != agibot_gdk.GDKRes.kSuccess:
    sys.exit(1)

robot = agibot_gdk.Robot()
time.sleep(2)

mc = robot.get_motion_control_status()
print("motion:", "mode=", mc.mode, "error_code=", mc.error_code, "error_msg=", repr(mc.error_msg))

ws = robot.get_whole_body_status()
print("arm_control:", "left=", ws.get("left_arm_control"), "right=", ws.get("right_arm_control"))
print("arm_error:", "left=", ws.get("left_arm_error"), "right=", ws.get("right_arm_error"))

js = robot.get_joint_states()
errs = [(s.get("name"), s.get("error_code")) for s in js.get("states", []) if s.get("error_code", 0)]
print("joint_count:", js.get("nums"), "joint_error_count:", len(errs))
for name, code in errs:
    print("JOINT_ERR", name, hex(code))

agibot_gdk.gdk_release()
PY
```

---

## 7. 修复流程

### 7.1 安全前提

执行前确认：

- 机器人周围无人
- 手臂附近没有夹具、线缆、工件干涉
- 没有策略脚本、HMI、遥操作端正在占用手臂
- 当前修复不是运动命令，但恢复驱动器后手臂会重新进入可使能状态

不要在未确认安全的情况下执行清错或重启。

### 7.2 方法一：单从站清错

以本次 `slave 4` 为例：

```bash
# 读修复前状态
sudo ethercat upload -p 4 -t uint16 0x6041 0x00
sudo ethercat upload -p 4 -t uint16 0x603f 0x00

# 清 CoolDrive/JuXie 二级控制字 CW2
sudo ethercat download -p 4 -t uint16 0x3002 0x00 0x0000
sleep 0.3

# 触发从站状态重建
sudo ethercat states -p 4 INIT
sleep 0.5
sudo ethercat states -p 4 OP
sleep 3

# 读修复后状态
sudo ethercat upload -p 4 -t uint16 0x6041 0x00
sudo ethercat upload -p 4 -t uint16 0x603f 0x00
```

期望结果：

```text
0x9737
0x0000
```

如果 HAL 日志出现：

```text
All motor errors cleared
All 16 slaves reached Op state successfully
All motors ready, motor operator enable.
```

说明 HAL 完成清错序列。

### 7.3 如果单从站清错后马上复发

本次第一次清错后，HAL 有清错动作：

```text
All motor errors cleared
All 16 slaves reached Op state successfully
```

但随后又出现：

```text
motor 2 err code 0xff38
```

这表示驱动器回 OP 后立刻再次锁存故障，单次清错没有打断 HAL/驱动器的故障循环。

此时重启机器人应用栈：

```bash
sudo systemctl restart genie_app.service
```

等待关键进程回来：

```bash
ps -eo pid,lstart,cmd | grep -E '/home/agi/app/bin/(run_corobot_app|hal$|gdk_service|fault_manager)|genie_motion_control' | grep -v grep
```

确认新 HAL 日志：

```bash
ls -lt /data/logs/latest/hal.log.INFO.* /data/logs/latest/fault_manager.log.INFO.* | head
```

重启后如果看到多个 `0xff51` 或 `0xffff`，先不要慌。这可能是 HAL 重建和 ESTOP 联动后的暂态。继续确认根故障是否仍然是同一个从站，例如本次仍为：

```text
slave 4 error=0xff38
```

然后再次对同一个故障从站执行：

```bash
sudo ethercat download -p 4 -t uint16 0x3002 0x00 0x0000
sudo ethercat states -p 4 INIT
sleep 0.5
sudo ethercat states -p 4 OP
sleep 4
```

本次第二次执行后恢复成功。

---

## 8. 修复后验证标准

修复不能只看命令返回值。必须至少通过以下四层验证。

### 8.1 EtherCAT 全从站验证

再次扫描：

```bash
for slave in 2 3 4 5 6 7 8 10 11 12 13 14 15 16; do
  SW=$(sudo ethercat upload -p "$slave" -t uint16 0x6041 0x00 2>/dev/null | awk '{print $1}')
  EC=$(sudo ethercat upload -p "$slave" -t uint16 0x603f 0x00 2>/dev/null | awk '{print $1}')
  if [ "$slave" -le 8 ]; then
    MOTOR=$((slave - 2))
  else
    MOTOR=$((slave - 3))
  fi
  echo "slave $slave motor $MOTOR status=$SW error=$EC"
done
```

本次恢复后的结果：

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

### 8.2 GDK 状态验证

修复后 GDK 输出：

```text
gdk_init: GDKRes.kSuccess
motion_mode: 5 error_code: 0 error_msg: ''
arm_control: left=False right=False
arm_error: left=0 right=0
joint_count: 22 joint_error_count: 0
```

说明：

- `motion_mode=5`：G2 Servo 模式恢复
- `joint_error_count=0`：GDK 关节层没有错误
- `left_arm_control=False / right_arm_control=False`：当前没有控制任务占用双臂，不等于故障

### 8.3 HAL 日志验证

修复后 HAL 出现：

```text
I0602 11:23:11.911770  igh_ecat_master.cpp:481] [hal]All 16 slaves reached Op state successfully
I0602 11:23:12.011865  igh_ecat_master.cpp:516] [hal]All motors ready, motor operator enable.
I0602 11:23:12.011878  igh_ecat_master.cpp:549] [hal]All motor errors cleared
```

最后一次 `err code` 停在：

```text
I0602 11:23:11.911790  igh_ecat_master.cpp:534] [hal]motor 2 err code 0xff38
```

之后没有新增 `err code`，说明错误不再持续刷。

### 8.4 小幅动作验证

不要直接运行官方 `servo_control.py` 默认示例。它会先给所有关节发 `0.0` 目标位，风险较高。

建议只对故障关节做极小幅度动作，然后回原位。以本次 `idx23_arm_l_joint3` 为例：

```bash
cd /home/agi/app
source /home/agi/app/env.sh /home/agi/app
python3 - <<'PY'
import time, sys
import agibot_gdk

JOINT = "idx23_arm_l_joint3"
DELTA = 0.015
VEL = 0.05

def find_joint(robot, name):
    js = robot.get_joint_states()
    for st in js.get("states", []):
        if st.get("name") == name:
            return st
    raise RuntimeError("joint not found: " + name)

def send_joint(robot, name, pos, life=1.0):
    req = agibot_gdk.JointControlReq()
    req.life_time = life
    req.joint_names = [name]
    req.joint_positions = [pos]
    req.joint_velocities = [VEL]
    return robot.joint_control_request(req)

ret = agibot_gdk.gdk_init()
print("gdk_init:", ret)
if ret != agibot_gdk.GDKRes.kSuccess:
    sys.exit(1)

robot = agibot_gdk.Robot()
time.sleep(2.0)

try:
    mc = robot.get_motion_control_status()
    print("motion before:", "mode=", mc.mode, "error=", mc.error_code, "msg=", repr(mc.error_msg))

    st0 = find_joint(robot, JOINT)
    if st0.get("error_code", 0):
        raise RuntimeError("%s has error_code 0x%04X" % (JOINT, st0.get("error_code")))

    p0 = float(st0.get("motor_position"))
    p1 = p0 + DELTA
    print("before:", JOINT, "pos=", p0, "target=", p1, "delta=", DELTA)

    print("move_ret:", send_joint(robot, JOINT, p1))
    time.sleep(1.2)
    st1 = find_joint(robot, JOINT)
    print("after move:", "pos=", st1.get("motor_position"), "error=", hex(st1.get("error_code", 0)))

    print("return_ret:", send_joint(robot, JOINT, p0))
    time.sleep(1.2)
    st2 = find_joint(robot, JOINT)
    print("after return:", "pos=", st2.get("motor_position"), "error=", hex(st2.get("error_code", 0)))

    mc2 = robot.get_motion_control_status()
    print("motion after:", "mode=", mc2.mode, "error=", mc2.error_code, "msg=", repr(mc2.error_msg))
finally:
    agibot_gdk.gdk_release()
PY
```

本次实测：

```text
motion before: mode=5 error=0 msg=''
before: idx23_arm_l_joint3 pos=-1.617453550880929 target=-1.602453550880929 delta=0.015
move_ret: 0
after move: pos=-1.6024533759358903 error=0x0
return_ret: 0
after return: pos=-1.6174530715119326 error=0x0
motion after: mode=5 error=0 msg=''
```

这说明：

- 命令返回正常
- 故障关节实际位置按目标变化
- 回原位成功
- 动作后没有新增关节错误
- EtherCAT `slave 4` 保持 `status=0x9737 / error=0x0000`

---

## 9. 故障原因解释

### 9.1 为什么看起来像“莫名其妙坏了”

驱动器故障是锁存型的。触发瞬间可能已经过去，用户看到的是后续状态：

```text
手臂不动
GDK 控制返回 0
运动模式异常或手臂无控制权
HAL 每隔几秒刷 motor N err code
```

如果只看当前 GDK 返回值，会误以为代码没问题但机器人随机坏了。实际应回到 HAL 日志找首次触发点。

### 9.2 本次最可信根因

本次证据链最强的是 `juxie_wrapper` 时间戳跳变：

```text
timestamp_ns 与 joints_info_[0].time_stamp 差异异常
set error code 262144
```

随后 HAL 进入 ESTOP，并且 `slave 4` 锁存 `0xff38`。

因此本次更像是：

```text
控制时间戳异常 / 控制链路异常断续
触发 HAL 保护
驱动器进入锁存故障
HAL 自动清错无法彻底打断
需要手动清 CW2 + 从站 INIT/OP
```

### 9.3 后续如果复发，重点查什么

优先查这些问题：

1. 故障前是否运行过 GDK/Python 控制脚本
2. 脚本是否被 Ctrl+C、kill、断 SSH、异常退出
3. 脚本是否在 `gdk_init()` 后立即发控制命令，没有 `sleep 2-3s`
4. 是否有多个控制源同时发手臂控制
5. 是否有系统时间、PTP、NTP 跳变
6. `/data/logs/latest/hal.log.INFO.*` 中是否反复出现 `juxie_wrapper.cpp` 时间戳异常
7. `fault_manager.log.INFO.*` 是否把多个手臂关节都报成 Communication fault

---

## 10. 预防措施

### 10.1 Python/GDK 控制脚本规范

建议所有手臂控制脚本遵守：

```python
import time
import agibot_gdk

ret = agibot_gdk.gdk_init()
if ret != agibot_gdk.GDKRes.kSuccess:
    raise RuntimeError(ret)

robot = agibot_gdk.Robot()
time.sleep(2.0)  # 等 GDK、DDS、时间同步和状态订阅稳定

try:
    # 先读状态，确认无错误，再发控制
    js = robot.get_joint_states()
    errs = [s for s in js["states"] if s.get("error_code", 0)]
    if errs:
        raise RuntimeError("joint error exists")

    # 控制逻辑
finally:
    agibot_gdk.gdk_release()
```

不要：

- `kill -9` 控制脚本
- 长时间阻塞后继续发旧目标
- 多个脚本同时控制同一批关节
- 跳过状态检查直接发运动
- 直接运行会把所有关节发到 0 位的示例

### 10.2 运行前健康检查

```bash
cd /home/agi/app
source /home/agi/app/env.sh /home/agi/app
python3 - <<'PY'
import time, sys
import agibot_gdk

ret = agibot_gdk.gdk_init()
if ret != agibot_gdk.GDKRes.kSuccess:
    print("GDK init failed:", ret)
    sys.exit(1)

r = agibot_gdk.Robot()
time.sleep(2)

mc = r.get_motion_control_status()
print("motion:", mc.mode, mc.error_code, repr(mc.error_msg))

js = r.get_joint_states()
errs = [(s["name"], s["error_code"]) for s in js["states"] if s.get("error_code", 0)]
if errs:
    print("JOINT ERRORS:")
    for n, e in errs:
        print(n, hex(e))
else:
    print("all joint error_code = 0")

agibot_gdk.gdk_release()
PY
```

---

## 11. 一页快速处理卡片

### 11.1 定位

```bash
HAL=$(ls -t /data/logs/latest/hal.log.INFO.* | head -1)
grep 'err code' "$HAL" | tail -20
```

看到类似：

```text
motor 2 err code 0xff38
```

换算：

```text
motor 2 -> slave 4
```

扫描：

```bash
sudo ethercat upload -p 4 -t uint16 0x6041 0x00
sudo ethercat upload -p 4 -t uint16 0x603f 0x00
```

### 11.2 修复

```bash
sudo ethercat download -p 4 -t uint16 0x3002 0x00 0x0000
sudo ethercat states -p 4 INIT
sleep 0.5
sudo ethercat states -p 4 OP
sleep 3
```

如果马上复发：

```bash
sudo systemctl restart genie_app.service
# 等 run_corobot_app、hal、gdk_service、genie_motion_control 回来
sudo ethercat download -p 4 -t uint16 0x3002 0x00 0x0000
sudo ethercat states -p 4 INIT
sleep 0.5
sudo ethercat states -p 4 OP
sleep 4
```

### 11.3 验证

```bash
sudo ethercat upload -p 4 -t uint16 0x6041 0x00
sudo ethercat upload -p 4 -t uint16 0x603f 0x00
```

期望：

```text
0x9737
0x0000
```

再看：

```bash
tail -50 /data/logs/latest/hal.log.INFO.*.0
```

期望：

```text
All 16 slaves reached Op state successfully
All motors ready, motor operator enable.
All motor errors cleared
```

最后用 GDK 读状态和小幅动作验证。

---

## 12. 本次最终状态

修复完成后：

```text
EtherCAT:
  all arm slaves status=0x9737 error=0x0000

GDK:
  motion_mode=5
  error_code=0
  error_msg=''
  joint_error_count=0

HAL:
  last err code at 2026-06-02 11:23:11
  2026-06-02 11:23:12 All motors ready, motor operator enable.

Motion test:
  idx23_arm_l_joint3 +0.015 rad and return succeeded
  no new joint error
```

结论：本次不是硬件永久损坏。故障关节恢复后能按小幅命令正常运动。若之后重复出现，应按本文档优先追查控制脚本异常退出和时间同步/时间戳跳变问题。
