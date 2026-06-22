# G2 Live Architecture Deep Dive - 2026-06-18

This note captures the current read-only understanding of the live Agibot G2 at
`agi@10.185.207.191`. It is meant as the working reference for tunnel-mode
video/audio, guarded remote driving, and future SLAM/navigation repair.

## Ground Rules

- Treat `/home/agi/app`, `/home/agi/app/conf`, `/home/agi/app/config`,
  `/home/agi/app/bin`, `/data/logs/latest`, and `/data/parameters` on the robot
  as the runtime source of truth.
- Do not send chassis, arm, head, waist, gripper, power, restart, EtherCAT, or
  fault-clear commands from this note. Everything below was gathered read-only.
- Chassis movement needs a fresh preflight immediately before motion. A clean
  motion-control status is not enough if SLAM pose/odom, charge-plug state, or
  PNC state is bad.

## Host And Version

- Robot host: `agi@10.185.207.191`
- App version: `genie_g02_rb_2.2.0_314fa4fb_2026-03-24-05-30-19_thor.tar.gz`
- Key package versions seen in the snapshot:
  - `agibot_gdk/2.6.3@genie`, commit `91b3a5dc`
  - `agibot_voice/snap-fix-ota@genie`, commit `703456d6`
  - `aorta/2.2.0@genie`, commit `7b1b7ddc`
  - `cosine_bus/3.5.0`
- Main service: `genie_app.service`, active since `2026-06-18 09:31:59 CST`.

## Startup Chain

Runtime chain:

```text
genie_app.service
  -> /home/agi/app/bin/run.sh
  -> FastDDS discovery at 10.42.1.101:11811
  -> AORTA discovery/service at 10.42.1.101:2379/2380
  -> launcher
  -> manifest scene "base"
  -> HAL, lowerlimb HAL, camera, lidar, SLAM/DR/tagloc, navigation,
     motion-control, task_manager, GDK service, HTTP service, media, fault,
     power, HMI proxy, remote HAL / pico_adapter
```

Important environment/config values:

- `PRODUCT_GENERATION=2`
- `ENABLE_G02_GDK=1`
- `APP_DIR=/home/agi/app`
- `AORTA_URI=http://10.42.1.101`
- `LOCATOR_IP=10.42.1.101`
- `AORTA_DISCOVERY_URI=http://10.42.1.101:2379`
- `COSINE_BUS_DEFAULT_MIDDLEWARE=aorta`
- `DEFAULT_LAUNCH_SCENE=base`

## Networks And Middleware

Observed robot interfaces:

- `ztfca6sezd`: `10.185.207.191/24`, current operator access path
- `wlan0`: `192.168.0.7/24`
- `xfi2.10g@mgbe2_0`: `10.42.1.101/24`, service/discovery network
- `xfi0.20@mgbe0_0`: `10.42.0.101/24`, sensor network
- `pad`: `10.42.6.101/24`

Observed listeners:

- `127.0.0.1:5061`: custom head tunnel viewer
- `0.0.0.0:8849`: `gdk_http_server`
- `*:2379`: AORTA discovery
- `10.42.1.101:2380`: AORTA service
- `11811`: FastDDS discovery

The active runtime bus is AORTA/Cosine with FastDDS discovery also started.
GDK clients initialize DDS/AORTA and can read robot state through
`Robot/Pnc/Slam/Map/TF/Camera/Lidar/Imu`.

## Control Architecture

High-level control path:

```text
GDK Python / HMI / task_manager
  -> Pnc / Robot / Slam / Map / TF APIs
  -> quark_navigation, genie_motion_control, task_manager
  -> HAL / hal_lowerlimb
  -> EtherCAT, CAN, chassis power board, arm/head/waist/end-effectors
```

GDK public control/read classes confirmed:

- `Robot`: whole-body state, joint state, motion-control state, power state,
  arm/head/waist/end-effector motion APIs
- `Pnc`: `request_chassis_control`, `move_chassis`, `relative_move`,
  `normal_navi`, `high_precision_navi`, `cancel_task`, `pause_task`,
  `resume_task`, `get_task_state`
- `Slam`: `get_slam_state`, `get_curr_pose`, `get_odom_info`, mapping APIs
- `Map`: `get_curr_map`, `get_all_map`, `get_map`, `switch_map`, `remove_map`
- `TF`: transform lookup and frame-list APIs

## Current Body And Motion State

Read-only GDK status:

- `Robot.get_motion_control_status()`:
  - `mode=5`
  - `error_code=0`
  - no collision pairs
- `Robot.get_whole_body_status()`:
  - arm/end/waist/lift/neck/chassis errors all `0`
  - arm control flags false
  - end models: `omnipicker`
- `Robot.get_joint_states()`:
  - 22 joints
  - no joint errors
  - head joints centered at `0.0`
- `Pnc.get_task_state()`:
  - `id=0`, `state=0`, `type=0`, idle

Startup logs showed early arm/gripper communication faults and motor `0xffff`
transients, but HAL later logged motor errors cleared and all motors ready. The
current GDK state does not show active arm/joint errors.

## Power And Chassis Safety State

Current chassis power state includes:

- main battery power switch state `1`
- chassis motor power states `1`
- `emergency_stop_pedal_state=0`
- `emergency_stop_pedal_fault_state=1`
- `charge_plug_insert_state=1`
- charge input voltage/current `0`
- battery around `48.5V`, SOC around `36%`

For guarded chassis movement, the existing map-navigation preflight treats
`charge_plug_insert_state=1` as a hard blocker even if input current and voltage
are zero. Keep that gate until the physical charger state is verified.

## Lowerlimb, EtherCAT, CAN

`hal_lowerlimb` startup log:

- registered `ecat1`
- found expected 2 EtherCAT slaves
- config success
- all registered slaves operational
- all slaves resumed OPERATIONAL

Chassis/power board versions:

- chassis power board serial `g02_chassis`, HW `1.0.1`, SW `0.0.17`
- traction motor version `2025.514.300`
- steer motor version `1.14.06`
- chest power board serial `G02T2`, HW `1.3`, SW `1.0.8.2.2`
- head motors: three `ZY_R48` joints from `JXZN`, HW `V1.20`, SW `V1.16.04`
- arm motors: 14 `tianji` arm joints, HW `H00`, SW `V3.0.2.16`
- arm end plates: `tianji`, HW `01.04.20230925`, SW `00.04.03.0402.20251219`

CAN status:

- `can0`: up/lower-up, state unknown
- `can1`: up/lower-up, CAN-FD, error-active, bitrate 1M, dbitrate 5M

## Cameras, Lidar, Audio

Camera runtime config:

- head color: `/dev/video14`, `640x400@30`, YUYV
- head depth: `/dev/video16`, `640x400@30`, Z16
- head stereo left/right: `/dev/video12` and `/dev/video10`, `1920x1536`
- head fisheyes: `/dev/video32`, `/dev/video34`, `/dev/video36`
- hand cameras and depths are also configured.

Lidar runtime config:

- Livox front/back topics: `/lidar/livox_front`, `/lidar/livox_back`
- IMU topics: `/imu/livox_front`, `/imu/livox_back`
- MID360 devices on sensor network `10.42.0.122` and `10.42.0.123`
- host sensor IP `10.42.0.101`

Lidar process is alive and logs regular `[lidar]running`, but that alone does
not mean SLAM pose/odom is valid.

Head audio/video tunnel:

- process: `python3 ./g2_head_tunnel_viewer.py --host 127.0.0.1 --port 5061 ...`
- local robot listener: `127.0.0.1:5061`
- browser path usually goes through SSH tunnel to local `127.0.0.1:15061`
- video source: GDK `CameraType.kHeadColor`
- audio source: AISpeech head board websocket `10.42.0.111:50002`, stream
  `aec.pcm`, `64000Hz`, mono
- current `/status` showed camera ok at about `20fps`, audio packets current
  with about `0.02s` last-packet age, and server playback downsampled to
  `16000Hz` mono.

## SLAM, Map, TF, PNC

Map state:

- current map from GDK: id `20`, name empty
- all known map IDs from GDK: `16`, `17`, `18`, `19`, `20`
- local map files exist under `/home/agi/app/data`:
  - `map.pcd`
  - `grid_map/grid_map_info.txt`
  - occupancy map PNGs
  - gravity file

SLAM state:

- `Slam.get_slam_state()` returns `0`
- `Slam.get_curr_pose()` fails with `GetCurrPose failed`
- `Slam.get_odom_info()` fails with `Slam odom is null`
- readiness check reports `odom_velocity_unavailable`

Current SLAM log root cause evidence:

- startup repeated `Lookup Newest TF Failed`
- repeated `Transform lookup failed: "base_link" ... target_frame does not exist`
- NDT/pre-pose initialization attempted with about `860428` init points
- then `Relocalization Is Failed!!!`
- then `Localization --- stop`

TF state from GDK:

- `odom -> base_link` is available
- `map -> base_link` is not available
- `base_link -> livox_front`, `base_link -> livox_back`, and `head_color` lookups
  are not available through GDK TF
- `TF.get_all_frame_names()` contains robot model frames but not the sensor
  frames needed to prove the full perception/navigation tree is closed.

Navigation dependency:

- `quark_navigation` subscribes `/slam/odom` and `/tf`.
- Topic connection exists, but GDK still reads odom as null; connection is not
  equivalent to valid localization data.
- `normal_navi` and `high_precision_navi` require map/pose/odom to be valid.
- `relative_move` and `move_chassis` can be used for short guarded movement, but
  they still need power, PNC, and safety gates.

## Current Readiness Result

Read-only command:

```bash
source /home/agi/app/env.sh
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1/rack_hybrid_docking_package
python3 industrial_map_nav_guarded.py --readiness-check
```

Result:

```json
{
  "ok": false,
  "problems": [
    "charge_plug_insert_state=1",
    "pose_unavailable=RuntimeError: GetCurrPose failed",
    "odom_velocity_unavailable"
  ],
  "map_id": 20,
  "motion_control_error": 0,
  "pnc_task_state": 0,
  "pnc_task_id": 0
}
```

Meaning:

- The robot software stack is mostly up.
- Upper-body and motion-control status are currently healthy.
- Chassis navigation is not currently ready.
- Do not equate a working video page with permission to drive through a tunnel.

## Tunnel-Mode Design Direction

For a tunnel use case, split the system into two independent planes:

1. Media plane
   - current head tunnel viewer already provides low-latency head video and
     audio over an SSH/TCP path
   - continue optimizing browser playback, latest-frame video, audio jitter
     buffer, and observability

2. Motion plane
   - use GDK `Pnc.request_chassis_control()` plus `Pnc.move_chassis()` only
     through a server-side watchdog
   - command cadence should be around 10-20Hz, with low speed caps
   - every command must expire quickly; missed heartbeat sends zero velocity
   - cleanup must send repeated zero twists and cancel active PNC task if needed
   - hard gates before any live drive:
     - `charge_plug_insert_state == 0`
     - `emergency_stop_pedal_state == 0`
     - `motion_control_error == 0`
     - PNC idle or explicitly controlled by this teleop session
     - either valid SLAM pose/odom or an explicit no-SLAM teleop mode with very
       conservative speed caps and obstacle-stop signals
   - for autonomous tunnel navigation, fix SLAM relocalization/TF/map first.

## Practical Next Steps

1. Resolve physical/sensor gate: verify whether the charger is really inserted;
   if not, investigate why `charge_plug_insert_state` is stuck high.
2. Repair SLAM readiness:
   - confirm current physical location matches map 20
   - check why startup point-cloud filter cannot see `base_link`
   - confirm sensor static frames are published or intentionally handled outside
     TF
   - rerun relocalization only under a safe read-only/restart plan
3. Build tunnel teleop as a separate guarded server endpoint, not as direct
   browser-to-GDK commands.
4. Keep video/audio and motion control decoupled so media lag cannot extend a
   motion command.
