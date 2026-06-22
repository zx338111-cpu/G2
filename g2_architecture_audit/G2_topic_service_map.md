# G2 Topic / Service Map

本表按模块归纳当前已知通信关系。来源包括本次现场日志观察、既有 runbook 和本地 GDK 示例。SSH 恢复后应继续用最新 `/data/logs/latest` 自动提取补全。

## 1. 通信层

```text
AORTA:
  机器人内部主要 pub/sub 和服务发现层

FastDDS:
  discovery server，默认 11811
  支撑 GDK/ROS2/跨进程 DDS 发现

GDK HTTP:
  gdk_http_server，端口 8849
  提供 install.sh、文档、latest 日志访问
```

## 2. HAL

`hal` 负责上肢/腰部/头部/末端的硬件抽象和 EtherCAT 交互。

已知 topic：

```text
Publish:
  /hal/joint_state_raw
  /hal/joint_state
  /hal/whole_body_status
  /hal/left_ee_data
  /hal/right_ee_data
  /hal/left_ee_force_data
  /hal/right_ee_force_data

Subscribe:
  /hal/joint_cmd_raw
  /hal/chassis_power_state
  /hal/chest_power_state
  /state_machine/power_mode_request
  /wbc/left_ee_command
  /wbc/right_ee_command

Service:
  /hal/fault_clear
```

已知风险：

```text
/wbc/left_ee_command:
  publisher arbitrator_runner -> genie_msgs.msg.pb.JointState
  subscriber hal              -> sensor_msgs.msg.pb.JointState

/wbc/right_ee_command:
  同类消息类型不一致
```

判断：这不是普通 Python API 参数错误，更像包内二进制/配置生成期不一致。不要用脚本层绕过，应该从 manifest、proto、genie_msgs、hal/arbitrator 构建来源排。

补充现场事实：

- `run_corobot_app`、`arbitrator_runner`、`genie_motion_control` 都曾在 `/wbc/*_ee_command` 上作为 publisher 出现。
- `genie_motion_control` 当前发布类型是 `sensor_msgs.msg.pb.JointState`，与 HAL 订阅类型匹配。
- `arbitrator_runner` 发布类型是 `genie_msgs.msg.pb.JointState`，触发 HAL 日志里的类型不一致。

## 3. hal_lowerlimb

`hal_lowerlimb` 负责底盘、下肢、电源、IMU 等靠近硬件的状态和控制。

已知 topic：

```text
Publish:
  /hal/chassis_joint_state
  /hal/chassis_power_state
  /hal/chest_power_state
  /hal/usr_state
  /imu/chassis

Subscribe:
  /hal/chassis_power_ctrl
  /hal/chest_power_ctrl
  /hal/multi_led_strip_control
  /hal/soc_power_ctrl
  /pnc/chassis_joint_cmd
```

底盘电源诊断重点字段在 GDK `robot.get_chassis_power_state()` 中可见：

```text
chassis_left_traction_motor_power_state
chassis_right_traction_motor_power_state
emergency_stop_pedal_state
emergency_stop_pedal_fault_state
chassis_power_board_state
charge_plug_insert_state
battery_states
```

## 4. genie_motion_control

`genie_motion_control` 是全身运动控制核心，负责把上层关节/末端/轨迹控制转换成 HAL 命令。

已知模型分组：

```text
Waist Lift  -> /hal/joint_cmd_raw, length 5
Head        -> head yaw/roll/pitch
Left Arm    -> /hal/joint_cmd_raw, length 7
Left Tool   -> /wbc/left_ee_command, length 1
Right Arm   -> /hal/joint_cmd_raw, length 7
Right Tool  -> /wbc/right_ee_command, length 1
```

已知 GDK/服务接口：

```text
/MotionControlService/ControlMode/JointPosition
/MotionControlService/ControlMode/JointPosition/MotionPlan/MultiMotionPlan
/MotionControlService/MotionControl/response
/MotionControlService/SafeStop/request
/MotionControlService/SetLoad/request
```

已知风险：

```text
/MotionControlService/SetLoad/request:
  包内 ABI 拼写为 SetLoadRequst
  外部或旧客户端可能发布 SetLoadRequest
```

现场复核更具体：发布方是 `run_corobot_app`，类型 `genie_msgs.msg.pb.SetLoadRequest`；订阅方是 `genie_motion_control`，类型 `genie_msgs.msg.pb.SetLoadRequst`。这是当前实时日志仍存在的问题。

## 5. quark_navigation / PNC

`quark_navigation` 是底盘导航和任务控制核心。

已知 topic/service：

```text
Subscribe:
  /pnc/remote_control_cmd
  /lidar/livox_front
  /lidar/livox_back
  /slam/odom
  /tf

Publish:
  /pnc/chassis_joint_cmd
  /pnc/task_state
  /pnc/global_plan_path
  /pnc/layered_map

Task services:
  /pnc/task_service/relative_move
  /pnc/task_service/normal_navigation
  /pnc/task_service/task_cancel
  /pnc/task_service/task_pause
  /pnc/task_service/task_resume
```

GDK 使用层：

```python
pnc = agibot_gdk.Pnc()
pnc.get_task_state()
pnc.cancel_task(task_id)
pnc.relative_move(req)
pnc.normal_navi(req)
pnc.move_chassis(twist)
```

经验：

- `relative_move` 是任务式闭环，优先使用。
- `move_chassis` 是速度式控制，需抢控制权，容易被已有任务压制。
- `relative_move` 可依赖 DR，不强依赖 SLAM。

## 6. slam / dr / lidar / tagloc / freespace

导航感知子系统：

```text
slam_state_machine:
  SLAM 状态、地图、定位、odom

dr_state_machine:
  Dead Reckoning，IMU/轮速等融合的相对里程计
  relative_move 可依赖 DR 执行

lidar:
  前/后雷达数据源

tagloc_state_machine:
  标签定位

freespace_state_machine:
  可通行区域/障碍相关
```

常见诊断：

```text
/data/logs/latest/quark_navigation.INFO
/data/logs/latest/dr_state_machine.log.INFO.*
/data/logs/latest/slam_state_machine.log.INFO.*
/data/logs/latest/lidar.INFO
```

## 7. fault_manager

`fault_manager` 聚合 HAL、电源、电池、关节、通信等故障。

已知 monitor：

```text
ThirdFaultMonitor
MotorMonitor
BatteryMonitor
OtherMonitor
```

已知配置：

```text
/home/agi/app/config/g02_fault_manager.json
/home/agi/app/config/g02_fault_code.json
/home/agi/app/config/battery_threshold.yaml
/home/agi/app/config/hmi_warning.yaml
```

重点：

- BatteryMonitor 会读取 HMI warning 配置和 power threshold 配置。
- 早期 boot 的瞬态故障要结合时间戳判断，不要把已自动清除的旧 fault 当作当前硬件故障。

## 8. GDK

GDK 是开发者接口层，不直接驱动硬件。

本地资料确认对象：

```text
agibot_gdk.Robot
agibot_gdk.Pnc
agibot_gdk.Slam
agibot_gdk.Map
agibot_gdk.TF
agibot_gdk.Camera
agibot_gdk.Lidar
agibot_gdk.Imu
agibot_gdk.UltrasonicRadar
```

初始化：

```python
import agibot_gdk
ret = agibot_gdk.gdk_init()
...
agibot_gdk.gdk_release()
```

机器人侧环境：

```bash
cd /home/agi/app
source /home/agi/app/env.sh /home/agi/app
```

开发机混合部署：

```bash
curl -sSL http://<robot_ip>:8849/install.sh | bash
source ~/.cache/agibot/app/env.sh
```

## 9. 推荐补全命令

SSH 恢复后，用这些只读命令刷新表：

```bash
grep -RhoE '/[A-Za-z0-9_./-]+' /data/logs/latest/*.INFO /data/logs/latest/*/*.INFO 2>/dev/null | sort -u
grep -R "publisher\|subscriber\|service\|topic" /data/logs/latest -n | head -500
systemctl status genie_app.service --no-pager -l
pgrep -a -f 'launcher|aorta|fastdds|gdk|hal|motion|quark|slam|dr|fault|camera|lidar|remote'
```

## 10. 2026-06-04 运行态 Topic 索引

来源：`grep -RhoE 'topic:?/[A-Za-z0-9_./-]+' /data/logs/latest/`。本节是快速索引，不等同于完整消息契约；消息类型、发布方、订阅方仍要从各模块日志继续补。

### 10.1 HAL / 硬件状态

```text
/hal/arm_joint_state
/hal/batt_state
/hal/chassis_joint_state
/hal/chassis_power_ctrl
/hal/chassis_power_state
/hal/chest_power_ctrl
/hal/chest_power_state
/hal/fault_clear_request
/hal/gripper_joint_state
/hal/joint_cmd
/hal/joint_state
/hal/left_arm_data
/hal/left_ee_data
/hal/left_ee_force_data
/hal/multi_led_strip_control
/hal/neck_state
/hal/position
/hal/right_arm_data
/hal/right_ee_data
/hal/right_ee_force_data
/hal/set_chassis_info
/hal/soc_power_ctrl
/hal/usr_state
/hal/waist_state
/hal/whole_body_status
/imu/chassis
```

### 10.2 MotionControl / WBC

```text
/MotionControlService/AddObjects/request
/MotionControlService/AddObjects/response
/MotionControlService/ControlMode/request
/MotionControlService/ControlMode/response
/MotionControlService/ErrorRecovery/request
/MotionControlService/ErrorRecovery/response
/MotionControlService/FrankaPose/request
/MotionControlService/FrankaPose/response
/MotionControlService/GetAllObjects/request
/MotionControlService/GetAllObjects/response
/MotionControlService/JointPosition/request
/MotionControlService/JointPosition/response
/MotionControlService/MotionControl/request
/MotionControlService/MotionControl/response
/MotionControlService/MotionPlan/request
/MotionControlService/MotionPlan/response
/MotionControlService/MotionPlanServo/request
/MotionControlService/MotionPlanServo/response
/MotionControlService/MultiMotionPlan/request
/MotionControlService/MultiMotionPlan/response
/MotionControlService/PlanPath/request
/MotionControlService/PlanPath/response
/MotionControlService/Record/request
/MotionControlService/Record/response
/MotionControlService/RecordStop/request
/MotionControlService/RecordStop/response
/MotionControlService/RemoveObjects/request
/MotionControlService/RemoveObjects/response
/MotionControlService/Replay/request
/MotionControlService/Replay/response
/MotionControlService/SafeStop/request
/MotionControlService/SafeStop/response
/MotionControlService/SetComplianceParam/request
/MotionControlService/SetComplianceParam/response
/MotionControlService/SetComplianceParams/request
/MotionControlService/SetComplianceParams/response
/MotionControlService/SetLoad/request
/MotionControlService/SetLoad/response
/MotionControlService/SwitchController/request
/MotionControlService/SwitchController/response
/MotionControlService/TorqueSensorCalibration/request
/MotionControlService/TorqueSensorCalibration/response
/MotionControlService/UpdateObjects/request
/MotionControlService/UpdateObjects/response
/wbc/arm_command
/wbc/end_effector_pose_control
/wbc/force_position_cmd
/wbc/force_protect_level_pb
/wbc/gripper_command
/wbc/hand_command
/wbc/head_command
/wbc/joint_control
/wbc/joint_control_low_delay
/wbc/joint_position_control
/wbc/left_ee_command
/wbc/mocap_enable_state
/wbc/model_predict
/wbc/motion_control_status
/wbc/retarget
/wbc/right_ee_command
/wbc/set_control_mode
/wbc/waist_command
/wbc/wheel_command
```

### 10.3 SLAM / DR / TF

```text
/dr/odom
/hd_slam/global_loc_request
/hd_slam/global_loc_response
/hd_slam/mapping_response
/hd_slam/mapping_result_req
/hd_slam/odom
/hd_slam/set_map_req
/hd_slam/set_map_rsp
/hd_slam/state
/slam/global_loc_request
/slam/global_loc_response
/slam/mapping_response
/slam/mapping_result_req
/slam/odom
/slam/set_map_req
/slam/set_map_rsp
/slam/state
/tf
/tf_static
```

### 10.4 PNC / 导航

```text
/pnc/chassis_joint_cmd
/pnc/esdf_slice
/pnc/global_plan_path
/pnc/hp_target
/pnc/init_static_map
/pnc/layered_map
/pnc/layered_update_map
/pnc/local_esdf_slice
/pnc/local_layered_map
/pnc/map_service/get_map_id_version/request
/pnc/map_service/get_map_id_version/response
/pnc/map_service/get_map/request
/pnc/map_service/get_map/response
/pnc/map_service/set_map/request
/pnc/map_service/set_map/response
/pnc/mppi_trajectories
/pnc/remote_command
/pnc/remote_control_cmd
/pnc/robot_sphere_set
/pnc/task_service/high_precision_localization_start/request
/pnc/task_service/high_precision_localization_start/response
/pnc/task_service/high_precision_localization_stop/request
/pnc/task_service/high_precision_localization_stop/response
/pnc/task_service/high_precision_navigation/request
/pnc/task_service/high_precision_navigation/response
/pnc/task_service/normal_navigation/request
/pnc/task_service/normal_navigation/response
/pnc/task_service/path_follow_navigation/request
/pnc/task_service/path_follow_navigation/response
/pnc/task_service/relative_move/request
/pnc/task_service/relative_move/response
/pnc/task_service/remote_control/request
/pnc/task_service/remote_control/response
/pnc/task_service/replay_csv/request
/pnc/task_service/replay_csv/response
/pnc/task_service/task_cancel/request
/pnc/task_service/task_cancel/response
/pnc/task_service/task_pause/request
/pnc/task_service/task_pause/response
/pnc/task_service/task_resume/request
/pnc/task_service/task_resume/response
/pnc/task_state
/pnc/ttc
```

### 10.5 地图 / DLB

```text
/hd_map/get_all_maps_req
/hd_map/get_all_maps_rsp
/hd_map/get_curr_map_req
/hd_map/get_curr_map_rsp
/hd_map/get_map_req
/hd_map/get_map_rsp
/hd_map/remove_map_req
/hd_map/remove_map_rsp
/hd_map/switch_map_req
/hd_map/switch_map_rsp
/hd_map/update_map_req
/hd_map/update_map_rsp
/hd_mm/map_service/get_map/request
/hd_mm/map_service/get_map/response
/map/get_all_maps_req
/map/get_all_maps_rsp
/map/get_curr_map_req
/map/get_curr_map_rsp
/map/get_map_req
/map/get_map_rsp
/map/remove_map_req
/map/remove_map_rsp
/map/switch_map_req
/map/switch_map_rsp
/map/update_map_req
/map/update_map_rsp
/mm/map_service/get_map/request
/mm/map_service/get_map/response
```

### 10.6 相机 / 雷达 / 感知

```text
/camera/hand_left_color
/camera/hand_left_depth
/camera/hand_left_lower_color
/camera/hand_left_lower_depth
/camera/hand_left_upper_color
/camera/hand_left_upper_depth
/camera/hand_right_color
/camera/hand_right_depth
/camera/hand_right_lower_color
/camera/hand_right_lower_depth
/camera/hand_right_upper_color
/camera/hand_right_upper_depth
/camera/head_back_fisheye
/camera/head_color
/camera/head_depth
/camera/head_left_fisheye
/camera/head_right_fisheye
/camera/head_stereo_left
/camera/head_stereo_right
/imu/livox_back
/imu/livox_front
/imu/xt_chest
/lidar/livox_back
/lidar/livox_front
/lidar/xt_chest
/occ_list
/perception/lidar_tracking_objects_3d
/perception/object_list
```

### 10.7 HMI / Pad / 用户状态

```text
/hmi_proxy/aid_info_status_request
/hmi_proxy/aid_info_status_response
/hmi_proxy/bluetooth_info_status_request
/hmi_proxy/bluetooth_info_status_response
/hmi_proxy/fault_clear_request
/hmi_proxy/fault_clear_response
/hmi_proxy/featureconfig_info_status_response
/hmi_proxy/hmi_activatewifi_message_response
/hmi_proxy/hmi_conn_message_request
/hmi_proxy/hmi_conn_message_response
/hmi_proxy/hmi_disk_status_response
/hmi_proxy/hmi_info_status_request
/hmi_proxy/hmi_set_conn_status_request
/hmi_proxy/hmi_set_conn_status_response
/hmi_proxy/hmi_switch_message_request
/hmi_proxy/hmi_switch_message_response
/hmi_proxy/hmi_task_data_request
/hmi_proxy/hmi_task_data_response
/hmi_proxy/hmiwarning_info_status_response
/hmi_proxy/hmi_warning_status_request
/hmi_proxy/hmi_wifi_message_request
/hmi_proxy/hmi_wifi_message_response
/hmi_proxy/http_req_forward_request
/hmi_proxy/http_req_forward_response
/hmi_proxy/initial_pose_request
/hmi_proxy/initial_pose_response
/hmi_proxy/login_info_status_request
/hmi_proxy/login_info_status_response
/hmi_proxy/network_info_status_request
/hmi_proxy/network_info_status_response
/hmi_proxy/restart_genie_app_request
/hmi_proxy/restart_genie_app_response
/hmi_proxy/user_info
/hmi_proxy/user_info_status_request
/hmi_proxy/user_info_status_response
/ipad_hmi/feature_notice_request
```

### 10.8 GDK / 远程 / 遥操作 / 任务

```text
/gdk/camera_conf_request
/gdk/camera_conf_response
/gdk/config_request
/gdk/config_response
/gdk/model_predict
/gdk/retarget
/remote/get_config_list_robot
/remote/operator_event
/remote/set_config_robot
/remote/vr_data
/remote/vr_network
/teleop/joint_position
/teleop/retarget
/taskflow/skill_request
/taskflow/skill_response
/taskflow/skill_state
/task_manager/co_tree_status
/task_manager/feature_task_request
/task_manager/feature_task_state
/task_manager/tree_status
```

### 10.9 电源 / 监控 / 其他业务

```text
/launcher/scene_mode
/launcher/scene_request
/launcher/scene_response
/monitor/g02_fault_topic
/monitor/third_fault_topic
/power_manager/power_mode
/power_manager/power_mode_request
/power_manager/power_mode_response
/state_machine/power_mode_request
/voice/func_status
/dlb_msgs/record_dds_request
/dlb_msgs/record_dds_response
/dlb_msgs/record_image_request
/dlb_msgs/record_image_response
/dlb_msgs/record_video_request
/dlb_msgs/record_video_response
/dlb_msgs/trigger_record
/dlb_msgs/trigger_record_resp
/dlb_msgs/upload_request
/dlb_msgs/upload_response
```

### 10.10 调试 / 模型 / Retarget

```text
/anchorpose/model_predict
/arbitrator/action_src
/copilot/reward
/copilot/state
/debug/cartesian_impedance_debug_pb
/debug/joint_commands_serial_pb
/debug/joint_states_serial_pb
/debug/retarget_pose_traj
/debug/retarget_traj
/feature_manager/eol
/genie/bundle_data
/guangtie/abnormal_event_request
/mbc/wheel_command
/model/eef_position
/model/joint_position
/retarget/admit_offsets
/retarget/debug_msg
/retarget/ik_marks
/retarget/ik_state
/retarget/state_marks
/retarget/target_marks
/retarget/target_state
```
