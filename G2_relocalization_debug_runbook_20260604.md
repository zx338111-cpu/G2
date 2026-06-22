# G2 重定位失败排障与修复记录

日期：2026-06-04  
机器人：`agi@10.20.15.152`  
主题：Pad 点重定位失败，SLAM 无法完成 relocalization。

## 1. 当前结论

本轮已经修复了后台通信和启动链路里的确定问题：

- `genie_app.service` 重启后会清理旧 `aorta-service`，避免复用旧 discovery 状态。
- `gdk_service`、`lidar`、`dr_state_machine`、`slam_state_machine`、`genie_motion_control` 已统一到同一个 AORTA discovery：

```text
AORTA_DISCOVERY_URI=http://10.42.1.101:2379
LOCATOR_IP=10.42.1.101
```

- SLAM 已真正读取 `/home/agi/app/conf/dds/cosine_ipc.prototxt`。
- SLAM 高频输入 topic 的实际队列已从 5 提高到 50：

```text
/imu/livox_front
/imu/livox_back
/dr/odom
/tf
```

修复后，重定位请求可以进入 SLAM，IMU、DR odom、TF、LiDAR 数据也正常进入算法。但 GICP 匹配分数仍低于阈值：

```text
refined gicp, score: 0.66-0.72, th=0.8
Relocalization Is Failed!!!
```

因此当前剩余问题不是后台线程没起来，也不是通信丢包，而是：

```text
当前激光看到的现场环境 / 初始点位姿
        和
当前加载地图 id=6
匹配不上。
```

不要直接把 `min_init_score` 从 `0.8` 降到 `0.65` 让系统假成功。这样会把错误定位交给导航链路，风险更高。

## 2. 机器人侧启动架构简述

本轮确认的启动链路：

```text
genie_app.service
  -> /home/agi/app/bin/run.sh
      -> source /home/agi/app/conf/sys/run.conf
      -> start fastdds discovery
      -> start aorta service
      -> /home/agi/app/bin/launcher
          -> /home/agi/app/conf/manifest.d/base.json
              -> gdk_service
              -> dlb
              -> lidar
              -> dr_state_machine
              -> slam_state_machine
              -> motion-control/start_mc.sh
                  -> genie_motion_control
```

SLAM 配置入口：

```text
/home/agi/app/bin/slam/config/a2d_v01/quark/quark.yaml
/home/agi/app/bin/slam/config/a2d_v01/alg/localization.yaml
/home/agi/app/conf/dds/cosine_ipc.prototxt
```

当前地图入口：

```text
/data/dlb/dlb.db
/home/agi/app/data/map.pcd
/home/agi/app/data/grid_map/
```

## 3. 初始现象

Pad 点击重定位后，SLAM 收到请求并进入定位算法，但一直失败。

典型日志：

```text
Recive Localization control type: 1
relocalization_mode:1
Need To Start Localization Algorithm
refined gicp, score: 0.66-0.70, th=0.8
Relocalization Is Failed!!!
```

同时早期日志存在通信和 TF 问题：

```text
DataLoss ... topic:/imu/livox_front configured_queue_size:5
DataLoss ... topic:/imu/livox_back configured_queue_size:5
DataLoss ... topic:/dr/odom configured_queue_size:5
DataLoss ... topic:/tf configured_queue_size:5
Lookup Newest TF Failed!!!
```

## 4. 已发现并修复的问题

### 4.1 SLAM 没有拿到 COSINE bus 配置

问题证据：

```text
cosine bus config path not set, use default config.
```

并且 `/imu/livox_front`、`/imu/livox_back`、`/dr/odom`、`/tf` 创建 buffer 时仍是：

```text
block num 5
qos depth:5
```

根因：

`/home/agi/app/conf/manifest.d/base.json` 里 `hal` 和 `mc` 有：

```text
COSINE_BUS_ENABLE_TOPIC_CONFIG_MERGE=1
COSINE_BUS_CONFIG_PATH=/home/agi/app/conf/dds/cosine_ipc.prototxt
```

但 `slam` 这一项原来只有：

```text
DYLOG_log_dir=/data/logs
LD_LIBRARY_PATH=...
```

所以 SLAM 进程一直用默认队列深度 5。

修复：

在 `base.json` 的 `slam` env 中加入：

```text
COSINE_BUS_ENABLE_TOPIC_CONFIG_MERGE=1
COSINE_BUS_CONFIG_PATH=/home/agi/app/conf/dds/cosine_ipc.prototxt
AORTA_COMMUNICATOR_NUM_FOR_PUB=5
AORTA_COMMUNICATOR_NUM_FOR_SUB=5
AORTA_DISPATCHER_THREAD_NUM=5
```

### 4.2 全局 topic 配置缺少 SLAM 高频输入

原 `/home/agi/app/conf/dds/cosine_ipc.prototxt` 只配置了：

```text
/hal/joint_state_raw
/hal/joint_cmd_raw
```

修复后增加：

```text
topic_config{
    bus_topic_name: "/imu/livox_front"
    proto_type:"sensor_msgs.msg.pb.Imu"
    middleware:"aorta"
    qos{
        depth:50
    }
}

topic_config{
    bus_topic_name: "/imu/livox_back"
    proto_type:"sensor_msgs.msg.pb.Imu"
    middleware:"aorta"
    qos{
        depth:50
    }
}

topic_config{
    bus_topic_name: "/dr/odom"
    proto_type:"genie_msgs.msg.pb.OdomInfo"
    middleware:"aorta"
    qos{
        depth:50
    }
}

topic_config{
    bus_topic_name: "/tf"
    proto_type:"tf2_msgs.msg.pb.TFMessage"
    middleware:"aorta"
    qos{
        depth:50
    }
}
```

验证证据：

```text
topic config merge is enabled for TopicConfig based interfaces.
bus_buffer___imu__livox_front ... block num 50
bus_buffer___imu__livox_back ... block num 50
bus_buffer___dr__odom ... block num 50
bus_buffer___tf ... block num 50
```

### 4.3 motion-control 重新 source 旧 run.conf，覆盖 AORTA 地址

问题证据：

重启后一度出现：

```text
gdk_service       AORTA_DISCOVERY_URI=http://10.20.15.152:2379
dr_state_machine  AORTA_DISCOVERY_URI=http://10.20.15.152:2379
lidar             AORTA_DISCOVERY_URI=http://10.20.15.152:2379
slam_state_machine AORTA_DISCOVERY_URI=http://10.20.15.152:2379
genie_motion_control AORTA_DISCOVERY_URI=http://10.42.1.101:2379
```

根因：

`motion-control/bin/start_mc.sh` 会 source `/home/agi/app/bin/motion-control/bin/env.sh`，该文件再 source `/home/agi/app/env.sh`。而 `/home/agi/app/env.sh` 会重新 source `/home/agi/app/conf/sys/run.conf`，把父进程已经解析出的 AORTA 地址覆盖掉。

修复：

修改 `/home/agi/app/env.sh`，source `run.conf` 前保存父进程已有的：

```text
LOCATOR_IP
AORTA_URI
AORTA_DISCOVERY_URI
```

source 完 `run.conf` 后再恢复这些继承值。

### 4.4 run.sh 网络选择策略不稳定

原修复过程中曾让 `run.sh` 优先根据默认路由选择 IP，结果会选到 Wi-Fi `10.20.15.152`。但机器人内部 AORTA/有线域更适合统一在 `10.42.1.101`。

最终策略：

- 如果 `run.conf` 中配置的 `LOCATOR_IP=10.42.1.101` 当前在本机网卡上存在，则优先使用它。
- 默认路由 IP 只作为 fallback。

最终所有关键进程统一为：

```text
AORTA_DISCOVERY_URI=http://10.42.1.101:2379
AORTA_URI=http://10.42.1.101
LOCATOR_IP=10.42.1.101
```

### 4.5 stale aorta-service 不会随 genie_app.service 清理

问题证据：

`genie_app.service restart` 后，旧进程仍在：

```text
root 8688 1 ... aorta-service --data-dir=/tmp/etcd-aorta-1773420482-8488 ...
```

由于 `aorta service -d` 会 daemonize，旧 `aorta-service` 脱离了 systemd 管理。

修复：

在 `/home/agi/app/bin/run.sh` 启动新的 aorta service 前增加旧进程清理：

```bash
if pgrep -f '^aorta-service( |$)' >/dev/null 2>&1; then
    echo "[INFO] stopping stale aorta-service before starting fresh discovery"
    pkill -TERM -f '^aorta-service( |$)' || true
    sleep 1
    if pgrep -f '^aorta-service( |$)' >/dev/null 2>&1; then
        pkill -KILL -f '^aorta-service( |$)' || true
    fi
fi
```

验证证据：

重启后旧 PID `8688` 消失，新 aorta service 使用新的 data dir：

```text
aorta-service --data-dir=/tmp/etcd-aorta-1780556235-331378 ...
```

后续再次重启为：

```text
aorta-service --data-dir=/tmp/etcd-aorta-1780557438-446351 ...
```

### 4.6 SLAM 初始化采样时间过短，已做保守优化

文件：

```text
/home/agi/app/bin/slam/config/a2d_v01/alg/localization.yaml
```

修改：

```text
init_time_seconds: 0.5 -> 1.5
timeout_bound: 10 -> 20
min_init_score: 0.8 保持不变
```

目的：

增加初始化采样和等待时间，排除短窗口造成的低质量输入，但不降低成功判定阈值。

验证：

修改后直接发布测试请求，数据组正常，但分数仍然只有 `0.66-0.69`，因此失败原因不在采样时间。

## 5. 备份文件

本轮修改前已在机器人侧备份：

```text
/home/agi/app/bin/run.sh.bak_20260604_145456
/home/agi/app/bin/run.sh.bak_20260604_145526_aorta_cleanup
/home/agi/app/bin/run.sh.bak_20260604_145713_aorta_cleanup_match

/home/agi/app/env.sh.bak_20260604_145456

/home/agi/app/conf/manifest.d/base.json.bak_20260604_145456

/home/agi/app/conf/dds/cosine_ipc.prototxt.bak_20260604_145456

/home/agi/app/bin/slam/config/a2d_v01/quark/quark.yaml.bak_20260604_144413

/home/agi/app/bin/slam/config/a2d_v01/alg/localization.yaml.bak_20260604_151603_relocal_sampling
```

说明：

- `quark.yaml` 里也加过 subscriber `qos.depth: 50`，但后来验证发现实际 AORTA 队列不是由这个字段控制。
- 真正让 SLAM 队列变成 50 的是 `base.json` 的 SLAM env 加 `COSINE_BUS_CONFIG_PATH`，以及 `cosine_ipc.prototxt` 的 topic config。

## 6. 地图现状

当前 DLB 普通地图只有一张：

```text
id=6
version=7
is_current=1
aid=G2A0104C300043
timestamp=1773430497
ts=2026-03-14 03:34:57
status=stg
map_info bytes=6321552
updated_map_info bytes=700
```

SLAM 实际地图文件：

```text
/home/agi/app/data/map.pcd
timestamp=2026-03-14 03:34:39
size=1.3M
```

解析 `map_info` 得到：

```text
nonground_cloud width=100292
point_step=32
grid width=1566
grid height=1987
grid resolution=0.05
origin=(-49.600002, -75.599998, 0)
```

结论：

机器人当前用于重定位的地图就是 2026-03-14 这张老地图。如果现场环境已经变化，或者机器人实际位置不在这张图对应位置，GICP 分数卡在 0.66-0.72 是合理现象。

## 7. 重定位验证记录

### 7.1 修复通信后，Pad 单次重定位

时间：15:07:46

```text
x: 972.232, y: 1519.72, z: 0
pose_in_world : 0.987727 0.00570898 0.156086 -0.824605
Measurement_group scan_raw size : 7559 / 7670
Measurement_group imu size : 20 / 49
Measurement_group odom size : 4 / 24
refined gicp, score: 0.665-0.682, th=0.8
Relocalization Is Failed!!!
```

### 7.2 换点后，Pad 请求

时间：15:10:18

```text
x: 972.811, y: 1507.9, z: -0.0724635
pose_in_world : 0.984702 0.0780928 0.155764 -0.782597
Measurement_group scan_raw size : 6441 / 7304
Measurement_group imu size : 49
Measurement_group odom size : 16 / 20
best refined gicp score: 0.721949, th=0.8
Relocalization Is Failed!!!
```

时间：15:10:39

```text
x: 975.627, y: 1522.22, z: -0.0724635
pose_in_world : 0.984702 0.0780928 0.155764 -0.641774
Measurement_group scan_raw size : 7052 / 6973
Measurement_group imu size : 49
Measurement_group odom size : 16
refined gicp score mostly 0.62-0.67, th=0.8
Relocalization Is Failed!!!
```

### 7.3 直接发布 `/slam/global_loc_request` 测试

目的：绕过 Pad/HMI，确认 SLAM 本身是否能接收并处理请求。

请求：

```text
topic=/slam/global_loc_request
type=genie_msgs.msg.pb.GlobalLocalRequest
control=1
relocalization_mode=1
x=972.811
y=1507.9
theta=-0.0724635
```

发布时间：15:24:57

结果：

```text
Recive Localization control type: 1
x: 972.811, y: 1507.9, z: -0.0724635
pose_in_world : 0.984702 0.0780928 0.155764 -0.78259
Measurement_group scan_raw size : 7638 / 7692
Measurement_group imu size : 49
Measurement_group odom size : 20
refined gicp, score: 0.660-0.690, th=0.8
Relocalization Is Failed!!!
```

结论：

Pad 不是唯一问题。直接发请求仍然失败，说明 SLAM 算法匹配地图失败。

## 8. DLB/HD map 额外发现

重启后的 DLB 日志出现：

```text
No current map found (is_current = true)
No current map available
[Odom] File not found: /data/dlb/hd_maps/odom.bin
Failed to load location for Relocation
Relocation ACK timeout, retrying...
```

但 GDK 普通 Map API 当前正常：

```text
get_all_map -> [<MapName id=6, name='', is_curr_map=1>]
get_curr_map -> <MapName id=6, name='', is_curr_map=1>
```

文件系统里存在：

```text
/data/dlb/maps/odom.bin
```

但不存在：

```text
/data/dlb/hd_maps/odom.bin
```

说明 DLB 的 HD map 辅助链路缺文件或没有当前 HD map。普通 SLAM 重定位不一定依赖 HD map，但如果 Pad 点击后没有新的 `/slam/global_loc_request` 进入 SLAM，应优先查这里。

本轮没有直接创建 `/data/dlb/hd_maps/odom.bin`，因为尚未确认 HD map 和普通 map 的目录语义是否可以直接复用，不能随意伪造地图状态。

## 9. 电源与安全状态

充电已拔掉后，底盘状态：

```text
charge_plug_insert_state=0
battery_charging_status=0
motion_control mode=5
motion_control error_code=0
```

但仍存在：

```text
emergency_stop_pedal_fault_state=1
```

这不是 GICP 分数低的直接原因，但后续导航或移动可能仍会被安全状态拦住。重定位成功后进入导航前，需要单独清查这个安全状态。

## 10. 后续操作建议

### 10.1 不要继续盲点同一个位置

如果地图或初始点不准，继续点同一片区域只会重复：

```text
score < 0.8
Relocalization Is Failed
```

### 10.2 优先做现场地图匹配确认

可选方案：

1. 把机器人推到这张 2026-03-14 地图中非常确定的位置，最好是激光特征丰富、环境遮挡少的位置。
2. 在 Pad 上准确点机器人真实位置和朝向，只点一次。
3. 监听 SLAM 日志，确认分数是否能超过 0.8。

监听命令：

```bash
latest_slam=$(readlink -f /data/logs/slam_state_machine.INFO)
tail -n 0 -F "$latest_slam" | grep --line-buffered -Ei \
  'Recive Localization|relocalization_mode|x: |pose_in_world|Need To Start|Measurement_group|refined gicp|Relocalization Is|Lookup Newest TF Failed|DataLoss'
```

### 10.3 如果现场已经变化，应重新建图或更新地图

当前地图是 2026-03-14 的老地图。如果现场家具、货架、设备、遮挡或机器人所在区域发生过明显变化，应重新建图或加载正确地图。

建图/更新地图前，先备份当前 DLB：

```bash
sudo cp -a /data/dlb/dlb.db /data/dlb/dlb.db.bak_$(date +%Y%m%d_%H%M%S)
sudo cp -a /home/agi/app/data /home/agi/app/data.bak_$(date +%Y%m%d_%H%M%S)
```

### 10.4 如果 Pad 点了但 SLAM 没收到请求

先查当前 boot 的 DLB/HMI 日志：

```bash
boot=$(readlink -f /home/agi/app/logs)
grep -nEi 'Relocation|global_loc|slam|map|No current map|odom.bin|timeout|Failed' "$boot"/dlb.log.INFO.*
grep -nEi 'reloc|localization|global_loc|slam|map|request|response|timeout|fail' "$boot"/hmi_proxy_end.log.INFO.*
```

重点看：

```text
Relocation ACK timeout
No current map found
/data/dlb/hd_maps/odom.bin missing
```

### 10.5 不建议的操作

不要为了让 UI 显示成功而直接改：

```text
min_init_score: 0.8 -> 0.65
```

这会把低置信度位姿写入定位链路，后续导航可能偏移、撞障或规划失败。

## 11. 快速复查命令

服务和进程：

```bash
systemctl is-active genie_app.service
ps -ef | grep -E 'launcher|slam_state_machine|dr_state_machine|lidar|gdk_service|genie_motion_control|aorta-service' | grep -v grep
```

关键进程环境：

```bash
sudo sh -c '
for name in gdk_service dr_state_machine lidar slam_state_machine genie_motion_control; do
  set -- $(pidof $name)
  p=$1
  echo ===$name pid=$p
  if [ -n "$p" ]; then
    tr "\0" "\n" < /proc/$p/environ | grep -E "^(LOCATOR_IP|AORTA_URI|AORTA_DISCOVERY_URI|COSINE_BUS|AORTA_DISPATCHER|AORTA_COMMUNICATOR)=" | sort
  fi
done'
```

SLAM 队列确认：

```bash
latest_slam=$(readlink -f /data/logs/slam_state_machine.INFO)
grep -nEi 'topic config merge|bus_buffer___imu__livox_front|bus_buffer___imu__livox_back|bus_buffer___dr__odom|bus_buffer___tf|configured_queue_size' "$latest_slam" | head -120
```

地图状态：

```bash
sqlite3 -header -column /data/dlb/dlb.db \
  "SELECT id,version,is_current,name,aid,timestamp,datetime(timestamp,'unixepoch','localtime') ts,status,length(map_info),length(updated_map_info) FROM maps ORDER BY id;"

cd /home/agi/app
source /home/agi/app/env.sh /home/agi/app
python3 - <<'PY'
import agibot_gdk, time
m = agibot_gdk.Map()
time.sleep(1)
print("all_map", m.get_all_map())
print("curr_map", m.get_curr_map())
PY
```

底盘安全状态：

```bash
cd /home/agi/app
source /home/agi/app/env.sh /home/agi/app
python3 - <<'PY'
import agibot_gdk
r = agibot_gdk.Robot()
print(r.get_chassis_power_state())
print(r.get_motion_control_status())
PY
```
