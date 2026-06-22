# G2 机器人 10.20.15.199 充电告警与 WiFi 发现修复文档

记录时间：2026-06-04  
目标机器人：`G2`  
机器人 WiFi IP：`10.20.15.199`  
电脑侧 IP：`10.20.15.107`  
SSH 用户：`agi`

## 1. 这次修复了什么

这次实际修复了两个问题。

第一，`fault_manager` 的 `BatteryMonitor` 启动时读取 `/home/agi/app/config/hmi_warning.yaml`，但机器人上这个文件缺失，日志里出现：

```text
Parse YAML error: bad file: /home/agi/app/config/hmi_warning.yaml
```

这个问题会导致电池相关故障/告警映射不完整。它不是充电电流控制逻辑本身的 bug，但会影响电池状态告警和 HMI 展示。

第二，拔掉网线后，机器人内部通信仍然绑定旧以太网地址 `10.42.1.101`。虽然 SSH 走的是 WiFi `10.20.15.199`，但是 `AORTA` 和 `FastDDS discovery` 仍然用 `10.42.1.101`，会导致你电脑无法稳定发现机器人或连接机器人应用层服务。

修复后验证结果：

```text
genie_app.service: active
FastDDS discovery: 10.20.15.199
fault_manager LOCATOR_IP: 10.20.15.199
hmi_proxy LOCATOR_IP: 10.20.15.199
power_manager LOCATOR_IP: 10.20.15.199
hal_lowerlimb LOCATOR_IP: 10.20.15.199
hmi_warning YAML error count: 0
```

电池也确认还在充电：

```text
charge_plug_insert_state=1
charge_plug_input_voltage=51V
charge_plug_input_current=15A

battery1:
  battery_charging_status=1
  battery_charging_current=3.6A
  battery_soc=68%
  battery_other_fault_state=0

battery2:
  battery_charging_status=1
  battery_charging_current=4.6A
  battery_soc=60.8%
  battery_other_fault_state=0
```

## 2. 机器人底层架构

这台 G2 机器人的 `/home/agi/app` 是主要运行目录。系统由 `systemd` 拉起一个总服务：

```text
genie_app.service
  -> /home/agi/app/bin/run.sh
      -> fastdds discovery
      -> aorta service
      -> perfetto_traced
      -> launcher
          -> hmi_proxy_end
          -> power_manager
          -> hal_lowerlimb
          -> fault_manager
          -> gdk_service
          -> gdk_http_server
          -> task_manager
          -> dds_record
          -> remote_hal/pico_adapter
          -> other app modules
```

几个核心层的职责如下。

`systemd / genie_app.service`  
负责开机启动整套机器人应用。重启它会重启大部分机器人应用进程。

`/home/agi/app/bin/run.sh`  
这是主启动脚本。它读取 `/home/agi/app/conf/sys/run.conf`，设置运行环境变量，启动 discovery 服务、AORTA 服务和 `launcher`。

`/home/agi/app/conf/sys/run.conf`  
这是系统运行配置。里面配置了应用目录、日志目录、AORTA 地址、FastDDS 配置、默认 launch scene 等。

`AORTA`  
机器人内部的消息总线/服务发现层。日志里能看到：

```text
AORTA_DISCOVERY_URI=http://10.20.15.199:2379
```

机器人各模块通过 AORTA 创建 publisher/subscriber，进行内部通信。

`FastDDS discovery`  
DDS 发现服务。GDK、ROS2 或部分机器人模块需要通过它发现通信节点。之前这里写死了 `10.42.1.101`，拔网线后就会出问题。

`launcher`  
真正拉起各个业务模块的进程管理器，例如 `fault_manager`、`power_manager`、`hal_lowerlimb`。

`hal_lowerlimb`  
底盘/下肢 HAL 层。它靠近硬件，负责发布硬件状态，例如：

```text
/hal/chassis_power_state
/hal/chest_power_state
/hal/chassis_joint_state
```

电池、充电插头、电源板状态都是从这里进入上层系统。

`power_manager`  
电源管理模块。它订阅 HAL 电源状态，也发布电源控制命令，例如：

```text
/hal/chassis_power_ctrl
/hal/chest_power_ctrl
/hal/soc_power_ctrl
```

它负责电源模式、电源开关、充电相关控制请求等。

`fault_manager`  
故障管理模块。它加载多个 monitor：

```text
ThirdFaultMonitor
MotorMonitor
BatteryMonitor
OtherMonitor
```

其中 `BatteryMonitor` 会订阅电池/电源状态，并根据阈值和告警配置生成故障/告警。

`hmi_proxy_end`  
HMI 代理模块。它处理 HMI 状态、告警、WiFi 状态、用户状态等 topic。电池告警最终会通过 HMI 配置显示出来。

`GDK`  
机器人开发接口。我们用 Python GDK 验证过实时电池状态：

```python
import agibot_gdk

agibot_gdk.gdk_init()
robot = agibot_gdk.Robot()
state = robot.get_chassis_power_state()
print(state)
agibot_gdk.gdk_release()
```

## 3. 网络架构和这次网络问题的根因

机器人本机有多个网卡和多个固定网段：

```text
xgi0      10.42.0.101/24
xgi1      10.42.12.101/24
enp1s0    10.42.1.101/24
wlan0     10.20.15.199/24
wlanap0   10.42.6.101/24
ecat0     10.42.30.101/24
ecat1     10.42.40.101/24
```

当前默认路由是：

```text
default via 10.20.15.1 dev wlan0
```

也就是说，当前机器人对外访问应该走 WiFi：

```text
机器人: 10.20.15.199
电脑:   10.20.15.107
```

问题在于，`run.conf` 里原来写死了：

```bash
export AORTA_URI=http://10.42.1.101
export LOCATOR_IP=10.42.1.101
export AORTA_DISCOVERY_URI=http://10.42.1.101:2379
```

同时，`run.sh` 里原来还有一行写死：

```bash
./bin/fastdds discovery -i 0 -l 10.42.1.101 -q 11811 -p 11811 &
```

拔掉网线后，`enp1s0` 仍然保留 `10.42.1.101/24`，所以程序误以为这个地址可用。但是你的电脑实际在 WiFi 网段，机器人外部通信应该绑定 `10.20.15.199`。

## 4. 电池告警配置问题的根因

`fault_manager` 的配置文件是：

```text
/home/agi/app/config/g02_fault_manager.json
```

关键配置如下：

```json
{
  "monitor_name": "BatteryMonitor",
  "config_path": "/home/agi/app/config/hmi_warning.yaml",
  "http_url": "",
  "time_out": 300,
  "power_system_config_path": "/home/agi/app/config/battery_threshold.yaml"
}
```

这些字段含义：

`monitor_name`  
指定加载哪个监控器，这里是 `BatteryMonitor`。

`config_path`  
电池告警对应 HMI 显示内容、语音、灯带等配置。这里要求存在：

```text
/home/agi/app/config/hmi_warning.yaml
```

`power_system_config_path`  
电池阈值配置，例如电压、电流、温度、SOC、短路/开路等状态阈值。这个文件存在：

```text
/home/agi/app/config/battery_threshold.yaml
```

这次不是 `battery_threshold.yaml` 错，而是 `hmi_warning.yaml` 缺失。

从 `fault_manager` 二进制字符串里确认，`BatteryMonitor` 实际读取的 YAML 字段包括：

```text
hmi_request_id
hmi_request_name
text_type
text_chinese
text_english
text_japanese
text_korean
text_priority
audio
voice_chinese
voice_english
lighting_strip
```

这些字段在现有的 `/home/agi/app/config/hmi_info.json` 里都存在，所以修复策略是：从 `hmi_info.json` 生成 `hmi_warning.yaml`，而不是手写一份不完整配置。

## 5. 实际改动的文件

### 5.1 新增文件：`/home/agi/app/config/hmi_warning.yaml`

文件信息：

```text
-rw-r--r-- 1 agi agi 1076582 Jun 4 10:34 /home/agi/app/config/hmi_warning.yaml
sha256: d65510c4411dc11fa25c6c320382ab4092dca7af6a3c4734b4c195bd593d1caf
entries: 2670
first_id: 0xE1011101
last_id: 0xE4042603
```

文件格式是顶层 YAML list，每条告警类似这样：

```yaml
-
  hmi_request_id: "0xE1011101"
  hmi_request_name: "放电单体电压过低故障"
  text_type: "interruptive_pop_up"
  text_chinese: "电池1号电压低，请注意！"
  text_english: "Battery No. 1 voltage is low, please note!"
  text_japanese: "NA"
  text_korean: "NA"
  text_priority: "error"
  audio: "NA"
  voice_chinese: "NA"
  voice_english: "NA"
  lighting_strip: "NA"
```

生成逻辑：

```python
import json

src = "/home/agi/app/config/hmi_info.json"
dst = "/tmp/hmi_warning.yaml"
fields = [
    "hmi_request_id",
    "hmi_request_name",
    "text_type",
    "text_chinese",
    "text_english",
    "text_japanese",
    "text_korean",
    "text_priority",
    "audio",
    "voice_chinese",
    "voice_english",
    "lighting_strip",
]

with open(src, "r", encoding="utf-8") as f:
    raw = json.load(f)

items = raw["AlertNotification"]

with open(dst, "w", encoding="utf-8") as f:
    f.write("# Generated from /home/agi/app/config/hmi_info.json for fault_manager BatteryMonitor.\n")
    f.write("# Schema is a top-level YAML sequence using the keys read by /home/agi/app/bin/fault_manager.\n")
    for item in items:
        f.write("-\n")
        for key in fields:
            value = item.get(key, "NA")
            if value is None:
                value = "NA"
            f.write(f"  {key}: {json.dumps(str(value), ensure_ascii=False)}\n")
```

安装命令：

```bash
install -m 0644 /tmp/hmi_warning.yaml /home/agi/app/config/hmi_warning.yaml
```

### 5.2 修改文件：`/home/agi/app/bin/run.sh`

修改前已备份：

```text
/home/agi/app/bin/run.sh.bak.20260604-103758
```

新增了 `resolve_network_env_for_aorta()`，位置在 `run.sh` 第 5-41 行：

```bash
function resolve_network_env_for_aorta() {
    local selected_ip=""

    if [ -n "${AORTA_PREFERRED_IP:-}" ] && command -v ip >/dev/null 2>&1; then
        if ip -o -4 addr show | awk '{print $4}' | cut -d/ -f1 | grep -qx "${AORTA_PREFERRED_IP}"; then
            selected_ip="${AORTA_PREFERRED_IP}"
        else
            echo "[WARN] AORTA_PREFERRED_IP=${AORTA_PREFERRED_IP} is not assigned to a local interface."
        fi
    fi

    if [ -z "${selected_ip}" ] && command -v ip >/dev/null 2>&1; then
        selected_ip=$(ip -o -4 route get 1.1.1.1 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')
    fi

    if [ -z "${selected_ip}" ] && [ -n "${LOCATOR_IP:-}" ] && command -v ip >/dev/null 2>&1; then
        if ip -o -4 addr show | awk '{print $4}' | cut -d/ -f1 | grep -qx "${LOCATOR_IP}"; then
            selected_ip="${LOCATOR_IP}"
        fi
    fi

    if [ -z "${selected_ip}" ] && command -v ip >/dev/null 2>&1; then
        selected_ip=$(ip -o -4 addr show scope global up | awk '{split($4, a, "/"); print a[1]; exit}')
    fi

    if [ -z "${selected_ip}" ]; then
        echo "[ERROR] cannot resolve a local IPv4 address for aorta/fastdds discovery" >&2
        exit 1
    fi

    export LOCATOR_IP="${selected_ip}"
    export AORTA_URI="http://${selected_ip}"
    export AORTA_DISCOVERY_URI="http://${selected_ip}:2379"
    echo "[INFO] network env: LOCATOR_IP=${LOCATOR_IP}, AORTA_DISCOVERY_URI=${AORTA_DISCOVERY_URI}"
}

resolve_network_env_for_aorta
```

这段代码的优先级：

1. 如果设置了 `AORTA_PREFERRED_IP`，并且这个 IP 确实在本机网卡上，则使用它。
2. 否则从默认路由解析本机出口 IP，例如当前解析到 `10.20.15.199`。
3. 如果默认路由解析失败，再尝试使用 `run.conf` 里的 `LOCATOR_IP`。
4. 再失败就取第一个 up 的全局 IPv4。
5. 仍然失败则退出启动，避免绑定到空地址。

同时将第 215 行改为：

```bash
./bin/fastdds discovery -i 0 -l "$LOCATOR_IP" -q 11811 -p 11811 &
```

这保证 FastDDS discovery 和 AORTA 使用同一个运行时 IP。

### 5.3 未修改但需要理解的文件：`/home/agi/app/conf/sys/run.conf`

当前内容仍包含原始默认值：

```bash
export AORTA_URI=http://10.42.1.101
export LOCATOR_IP=10.42.1.101
export AORTA_DISCOVERY_URI=http://10.42.1.101:2379
```

这次没有直接改 `run.conf`，原因是：

1. `run.conf` 像是出厂/产品配置，直接改死成 WiFi IP 会影响有线部署场景。
2. WiFi IP 可能变化，写死 `10.20.15.199` 不够稳。
3. 更稳的做法是在 `run.sh` 启动时根据默认路由动态覆盖。

如果将来你要强制指定某个 IP，可以在服务环境里设置：

```bash
export AORTA_PREFERRED_IP=10.20.15.199
```

然后 `run.sh` 会优先使用它。

## 6. 完整修复流程

### 6.1 SSH 登录机器人

```bash
ssh agi@10.20.15.199
```

密码：

```text
1
```

注意：网线拔掉后，不要使用 `10.42.1.101` 登录；当前应使用 WiFi IP `10.20.15.199`。

### 6.2 确认网络状态

```bash
ip -br addr
ip route
who -u
ss -tnp | grep ':22'
```

本次看到：

```text
wlan0  UP  10.20.15.199/24
default via 10.20.15.1 dev wlan0
SSH peer: 10.20.15.107
```

验证机器人能 ping 到电脑：

```bash
ping -c 2 -W 1 10.20.15.107
```

### 6.3 确认 `fault_manager` 配置

```bash
nl -ba /home/agi/app/config/g02_fault_manager.json | sed -n '1,80p'
```

重点确认：

```json
"monitor_name": "BatteryMonitor",
"config_path": "/home/agi/app/config/hmi_warning.yaml",
"power_system_config_path": "/home/agi/app/config/battery_threshold.yaml"
```

### 6.4 生成并安装 `hmi_warning.yaml`

生成到 `/tmp`：

```bash
python3 - <<'PY'
import json, os

src = "/home/agi/app/config/hmi_info.json"
dst = "/tmp/hmi_warning.yaml"
fields = [
    "hmi_request_id",
    "hmi_request_name",
    "text_type",
    "text_chinese",
    "text_english",
    "text_japanese",
    "text_korean",
    "text_priority",
    "audio",
    "voice_chinese",
    "voice_english",
    "lighting_strip",
]

with open(src, "r", encoding="utf-8") as f:
    raw = json.load(f)

items = raw.get("AlertNotification")
if not isinstance(items, list):
    raise SystemExit("AlertNotification is not a list")

missing = {key: 0 for key in fields}

with open(dst, "w", encoding="utf-8") as f:
    f.write("# Generated from /home/agi/app/config/hmi_info.json for fault_manager BatteryMonitor.\n")
    f.write("# Schema is a top-level YAML sequence using the keys read by /home/agi/app/bin/fault_manager.\n")
    for item in items:
        f.write("-\n")
        for key in fields:
            if key not in item or item[key] is None:
                missing[key] += 1
            value = item.get(key, "NA")
            if value is None:
                value = "NA"
            f.write("  {}: {}\n".format(key, json.dumps(str(value), ensure_ascii=False)))

print("wrote", dst, "items", len(items), "bytes", os.path.getsize(dst))
print("missing", {key: value for key, value in missing.items() if value})
PY
```

检查 YAML 能加载：

```bash
python3 - <<'PY'
import yaml

with open("/tmp/hmi_warning.yaml", "r", encoding="utf-8") as f:
    data = yaml.safe_load(f)

print(type(data).__name__, len(data))
print(data[0]["hmi_request_id"], data[0]["text_type"], data[0]["text_priority"])
PY
```

安装：

```bash
install -m 0644 /tmp/hmi_warning.yaml /home/agi/app/config/hmi_warning.yaml
sha256sum /tmp/hmi_warning.yaml /home/agi/app/config/hmi_warning.yaml
```

### 6.5 备份并修改 `run.sh`

备份：

```bash
cp /home/agi/app/bin/run.sh /home/agi/app/bin/run.sh.bak.$(date +%Y%m%d-%H%M%S)
```

修改点：

1. 在 `source /home/agi/app/conf/sys/run.conf` 后加入 `resolve_network_env_for_aorta()`。
2. 把 FastDDS discovery 的 `-l 10.42.1.101` 改成 `-l "$LOCATOR_IP"`。

语法检查：

```bash
bash -n /home/agi/app/bin/run.sh
```

确认改动：

```bash
grep -nE 'resolve_network_env_for_aorta|AORTA_PREFERRED_IP|network env|fastdds discovery|AORTA_URI|LOCATOR_IP|AORTA_DISCOVERY_URI' /home/agi/app/bin/run.sh
```

### 6.6 重启机器人应用

```bash
printf '1\n' | sudo -S systemctl restart genie_app.service
sleep 12
systemctl is-active genie_app.service
```

### 6.7 验证进程绑定地址

```bash
ps -ef | grep -E 'fastdds discovery|aorta service|/home/agi/app/bin/(fault_manager|hmi_proxy_end|power_manager|hal_lowerlimb)' | grep -v grep
```

期望看到：

```text
./bin/fastdds discovery -i 0 -l 10.20.15.199 -q 11811 -p 11811
```

### 6.8 验证 AORTA 运行时地址

```bash
grep -RInE 'use env var \[LOCATOR_IP\]|use env var \[AORTA_DISCOVERY_URI\]|aorta create domain success|Loaded monitor: BatteryMonitor|Parse YAML|hmi_warning.yaml|bad file' \
  /data/logs/latest/fault_manager* \
  /data/logs/latest/hmi_proxy_end* \
  /data/logs/latest/power_manager* \
  /data/logs/latest/hal_lowerlimb* 2>/dev/null | head -80
```

期望看到：

```text
fault_manager use env var [LOCATOR_IP]:10.20.15.199
fault_manager use env var [AORTA_DISCOVERY_URI]:http://10.20.15.199:2379
fault_manager aorta create domain success. uri:http://10.20.15.199:2379
fault_manager Loaded monitor: BatteryMonitor

hmi_proxy_end use env var [LOCATOR_IP]:10.20.15.199
power_manager use env var [LOCATOR_IP]:10.20.15.199
hal_lowerlimb use env var [LOCATOR_IP]:10.20.15.199
```

验证 YAML 错误为 0：

```bash
grep -RInE 'Parse YAML|Error loading YAML config|hmi_warning.yaml|bad file' /data/logs/latest/fault_manager* 2>/dev/null | wc -l
```

期望：

```text
0
```

### 6.9 验证电池状态

先加载环境：

```bash
source /home/agi/app/env.sh
```

读取状态：

```bash
python3 - <<'PY'
import time
import agibot_gdk

res = agibot_gdk.gdk_init()
print("gdk_init", res)

robot = agibot_gdk.Robot()
time.sleep(2)

state = robot.get_chassis_power_state()

print("charge_plug",
      int(state.charge_plug_insert_state),
      round(state.charge_plug_input_voltage, 2),
      round(state.charge_plug_input_current, 2))

for i, battery in enumerate(state.battery_states, 1):
    print("battery", i)
    print("  battery_charging_status", battery.battery_charging_status)
    print("  battery_output_voltage", battery.battery_output_voltage)
    print("  battery_charging_current", battery.battery_charging_current)
    print("  battery_soc", battery.battery_soc)
    print("  battery_other_fault_state", battery.battery_other_fault_state)
    print("  battery_switch_state", battery.battery_switch_state)
    print("  battery_charging_mos_switch_state", battery.battery_charging_mos_switch_state)

print("battery_count", len(state.battery_states))
print("gdk_release", agibot_gdk.gdk_release())
PY
```

## 7. 怎么判断问题真的修好了

满足下面条件，才算修好：

1. `systemctl is-active genie_app.service` 输出 `active`。
2. `fastdds discovery` 进程参数里是 `10.20.15.199`，不是 `10.42.1.101`。
3. `fault_manager` 日志里有 `Loaded monitor: BatteryMonitor`。
4. 最新 `fault_manager` 日志中 `hmi_warning.yaml` / `Parse YAML` / `bad file` 错误数量为 0。
5. `fault_manager`、`hmi_proxy_end`、`power_manager`、`hal_lowerlimb` 日志里都显示 `LOCATOR_IP=10.20.15.199`。
6. GDK 能读取 `get_chassis_power_state()`。
7. 电池充电插头状态为 1，输入电压/电流正常。

本次全部满足。

## 8. 回滚方法

### 8.1 回滚网络启动脚本

如果动态 IP 逻辑导致异常，可以回滚：

```bash
cp /home/agi/app/bin/run.sh.bak.20260604-103758 /home/agi/app/bin/run.sh
bash -n /home/agi/app/bin/run.sh
printf '1\n' | sudo -S systemctl restart genie_app.service
```

回滚后会恢复到旧行为，也就是继续绑定 `10.42.1.101`。如果网线仍然拔掉，外部 WiFi 发现问题可能会复发。

### 8.2 删除生成的 HMI warning 文件

一般不建议删除，因为 `g02_fault_manager.json` 明确引用它。如果确实要恢复缺文件状态：

```bash
mv /home/agi/app/config/hmi_warning.yaml /home/agi/app/config/hmi_warning.yaml.disabled.$(date +%Y%m%d-%H%M%S)
printf '1\n' | sudo -S systemctl restart genie_app.service
```

删除后 `fault_manager` 很可能再次出现：

```text
Parse YAML error: bad file: /home/agi/app/config/hmi_warning.yaml
```

## 9. 后续建议

### 9.1 把 `hmi_warning.yaml` 纳入正式发布包

`10.20.15.199` 和对照机器人 `10.20.15.60` 都缺这个文件，说明这不是单机误删，更像是发布包漏文件。

正式修复应该在打包流程里保证：

```text
/home/agi/app/config/hmi_warning.yaml
```

随 `g02_fault_manager.json` 一起发布。

### 9.2 不建议在 `run.conf` 里写死 WiFi IP

WiFi IP 可能会变。更合理的是：

1. 默认使用默认路由的 `src` IP。
2. 如果需要强制固定，使用 `AORTA_PREFERRED_IP`。
3. 如果机器人回到有线部署，默认路由切回有线后会自动选有线地址。

### 9.3 后续如果要排查通信问题，先看这几条

```bash
ip -br addr
ip route
ps -ef | grep -E 'fastdds discovery|aorta service' | grep -v grep
grep -RInE 'LOCATOR_IP|AORTA_DISCOVERY_URI|aorta create domain success' /data/logs/latest/fault_manager* /data/logs/latest/hmi_proxy_end* | head
```

### 9.4 后续如果要排查充电问题，先看这几条

```bash
source /home/agi/app/env.sh
python3 - <<'PY'
import time
import agibot_gdk

agibot_gdk.gdk_init()
robot = agibot_gdk.Robot()
time.sleep(2)
state = robot.get_chassis_power_state()
print(state)
agibot_gdk.gdk_release()
PY
```

重点看：

```text
charge_plug_insert_state
charge_plug_input_voltage
charge_plug_input_current
battery_charging_status
battery_charging_current
battery_soc
battery_other_fault_state
battery_input_fault_state
battery_charging_mos_switch_state
```

如果 `charge_plug_input_current` 有 10A 以上，通常说明充电输入链路是通的。如果两块电池 `battery_charging_status=1` 且 `battery_charging_current>0`，说明 BMS 正在充电。
