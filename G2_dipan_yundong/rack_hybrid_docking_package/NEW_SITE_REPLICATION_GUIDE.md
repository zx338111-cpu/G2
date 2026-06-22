# G2 七根料杆换地图与重新采点保姆级教程

本文档写给第一次接手这套程序的人。目标是：换一张地图、换一套抓料点和放料点以后，仍然能按固定流程把七根料杆任务复刻出来。

当前已跑通的参考现场是 `map20`，机器人工作目录是：

```bash
/data/g2_industrial_cell_20260612/wxf/BOX_528_1
```

当前主程序是：

```bash
rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py
```

本文档不会假设你已经熟悉代码。你只需要先理解几个概念，再按步骤做。

## 先记住三句话

1. `industrial_station_config.json` 管“地图上的导航点”，也就是底盘要开到哪里。
2. `calibration_records/rodXX_grab_pose_latest.json` 管“每根杆怎么抓”，也就是机械臂和腰部的抓取姿态。
3. `calibration_records/*place*latest.json` 加上主程序里的 `TUNED` 参数管“怎么放”，也就是到下料架以后机械臂如何放料、开爪、退出。

换地图时，不是只改一个 `map_id` 就够了。完整复刻至少要重新确认：

- 地图 ID；
- 四个导航站点；
- 七根杆的抓料姿态；
- 放料姿态链；
- 抓料和放料的超声精定位距离；
- 单根验证和整轮验证日志。

## 安全边界

先分清楚哪些命令不会动机器人，哪些命令会动机器人。

只读命令：

- 读取状态；
- 读取当前地图；
- 读取当前定位；
- 读取关节角；
- 读取超声；
- 写 JSON 记录。

会动机器人的命令：

- 导航到站点；
- 手臂移动；
- 腰部移动；
- 夹爪开合；
- 超声精定位；
- 底盘后退；
- 整轮七根任务。

任何会动机器人的命令都必须满足：

- 现场人员确认周围没有人和障碍；
- 机器人不在充电状态；
- 地图和定位正常；
- `motion_control_error=0`；
- 只读 preflight 通过；
- 命令里显式带 `--live --confirm-physical`，或者脚本明确要求现场确认。

如果你不确定一个命令会不会动机器人，先不要运行，先看它有没有 `--live`、`--confirm-physical`、`normal_navi`、`move_arm`、`move_waist`、`move_ee`、`fine_position`、`retreat` 这类字样。

## 目录结构怎么看

进入机器人工作目录：

```bash
source /home/agi/app/env.sh
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
```

关键文件如下：

```text
rack_hybrid_docking_package/
  industrial_cell_7_rods_single_debug.py
  industrial_station_config.json
  industrial_map_nav_guarded.py
  capture_grab_calibration_point.py
  calibrate_station_from_current_pose.py
  calibrate_direct_place_pose_offset.py
  analyze_industrial_cell_run.py
  calibration_records/
  g2_primitives/
```

每个文件的作用：

| 文件 | 作用 | 新人应该怎么理解 |
| --- | --- | --- |
| `industrial_cell_7_rods_single_debug.py` | 七根料杆主流程 | 状态机、抓放顺序、checkpoint、日志、安全 gate 都在这里 |
| `industrial_station_config.json` | 地图站点配置 | 换地图时必须检查和重新采集 |
| `industrial_map_nav_guarded.py` | 导航与 readiness | 用它做只读 readiness，也用它验证某个站点能不能到 |
| `calibrate_station_from_current_pose.py` | 从机器人当前定位采集站点 | 人工把机器人放到目标位置后，用它写入站点 |
| `capture_grab_calibration_point.py` | 抓料点只读采集 | 人工把手臂调到抓取姿态后，用它保存关节角 |
| `calibrate_direct_place_pose_offset.py` | 放料姿态辅助生成 | 需要现场确认，会移动上半身，主要用于生成某个放料偏移姿态 |
| `analyze_industrial_cell_run.py` | 跑完后分析日志 | 看每个阶段耗时、最终状态和失败点 |
| `calibration_records/` | 所有现场采点数据 | 换现场时最重要的数据目录 |
| `g2_primitives/` | 底层动作类 | 主程序调用这些 class 去移动手臂、腰、夹爪、底盘 |

## 主流程到底做了什么

七根料杆不是一个大动作，而是重复七次以下流程：

```text
NAV_TO_GRAB
LOCAL_PICK
NAV_TO_PLACE
LOCAL_PLACE
NAV_TO_RECOVERY
NAV_TO_HOME
ROD_DONE
```

每个阶段含义：

| 阶段 | 中文解释 | 使用哪些数据 |
| --- | --- | --- |
| `NAV_TO_GRAB` | 底盘导航到抓料预备点 | `industrial_station_config.json` 里的 `GRAB_PRE` |
| `LOCAL_PICK` | 到料架前精定位、手臂抓杆、后退退出 | `rodXX_grab_pose_latest.json` 和 `TUNED` 抓料参数 |
| `NAV_TO_PLACE` | 底盘导航到放料预备点 | `industrial_station_config.json` 里的 `PLACE_PRE` |
| `LOCAL_PLACE` | 到下料架前精定位、放杆、开爪、退出 | 放料姿态 JSON 和 `TUNED` 放料参数 |
| `NAV_TO_RECOVERY` | 去中间安全点 | `industrial_station_config.json` 里的 `RECOVERY_SAFE` |
| `NAV_TO_HOME` | 回初始安全点 | `industrial_station_config.json` 里的 `HOME_SAFE` |
| `ROD_DONE` | 当前杆完成，准备下一根 | checkpoint |

所以换地图时，先不要改主程序逻辑。优先替换现场数据：

- 地图站点；
- 抓料姿态；
- 放料姿态；
- 现场调参。

## 四个导航站点怎么理解

`industrial_station_config.json` 里面至少有四个站点：

```json
{
  "map_id": 20,
  "stations": {
    "HOME_SAFE": {},
    "GRAB_PRE": {},
    "PLACE_PRE": {},
    "RECOVERY_SAFE": {}
  }
}
```

每个站点都有：

```json
"position": {
  "x": 0.0,
  "y": 0.0,
  "z": 0.0
},
"orientation": {
  "x": 0.0,
  "y": 0.0,
  "z": 0.0,
  "w": 1.0
}
```

这些值来自 SLAM 当前地图坐标，不是机械臂坐标。

四个站点的选择原则：

| 站点 | 应该停在哪里 | 为什么 |
| --- | --- | --- |
| `HOME_SAFE` | 开始和结束都安全的位置 | 手臂能归位，底盘周围没有障碍 |
| `GRAB_PRE` | 正对上料架，距离适合超声精定位的位置 | 后面会用超声靠近料架抓料 |
| `PLACE_PRE` | 正对下料架，距离适合超声精定位的位置 | 后面会用超声靠近料架放料 |
| `RECOVERY_SAFE` | 从放料区回 HOME 的中间安全点 | 避免直接回家时绕路或贴近障碍 |

## 第 0 步：确认当前机器人和路径

从本机 SSH 到机器人：

```bash
ssh agi@192.168.0.7
```

进入项目：

```bash
source /home/agi/app/env.sh
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
```

确认主程序存在：

```bash
ls rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py
```

确认站点配置存在：

```bash
cat rack_hybrid_docking_package/profiles/map20_box528/industrial_station_config.json
```

如果这些文件不存在，说明你进错目录了，不要继续。

## 第 1 步：只读检查机器人状态

先跑只读 preflight。这个命令不应该执行物理运动：

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --preflight-only
```

你希望看到：

```text
ok: true
problems: []
```

如果失败，先看常见原因：

| 问题 | 含义 | 处理 |
| --- | --- | --- |
| `map_id_mismatch` | 当前机器人地图和配置里的 `map_id` 不一致 | 先确认机器人实际地图，再更新配置或换地图 |
| `charge_plug_insert_state` 异常 | 机器人还在充电 | 需要拔掉充电，确认电流为 0 或接近 0 |
| `motion_control_error` 非 0 | 运动控制层有错误 | 不要继续跑任务，先处理底层故障 |
| PNC 不 idle | 底盘还有任务或状态没清干净 | 等待、取消任务或重启相关服务，确认后再继续 |

## 第 2 步：换地图时先改 map_id

推荐不要手工复制 map20 目录，而是先用脚本创建一个空白新现场 profile：

```bash
python3 rack_hybrid_docking_package/create_site_profile.py \
  --site mapXX_new_site \
  --map-id XX \
  --from-profile rack_hybrid_docking_package/profiles/map20_box528/profile.json
```

这个脚本会创建：

```text
rack_hybrid_docking_package/profiles/mapXX_new_site/profile.json
rack_hybrid_docking_package/profiles/mapXX_new_site/industrial_station_config.json
rack_hybrid_docking_package/profiles/mapXX_new_site/calibration_records/
```

新 profile 会故意保持“待采点”状态：四个 station 是空的，七根抓料点和放料点文件也不会复制旧 map20 的 latest。这样 `validate_site_profile.py` 会明确报缺项，防止新人误把旧地图点位当成新现场点位。

下面命令里的 `profiles/map20_box528/profile.json` 是当前成功基线示例。真正换现场时，
把它替换成刚创建的 `profiles/mapXX_new_site/profile.json`。

打开：

```bash
rack_hybrid_docking_package/profiles/mapXX_new_site/industrial_station_config.json
```

把：

```json
"map_id": 20
```

改成现场当前地图 ID。

地图 ID 不能凭记忆填，要从机器人读取或由现场导航系统确认。填错以后 readiness 会失败，防止你在错误地图上跑错站点。

改完以后先不要跑 live，再跑一次：

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --preflight-only
```

如果还有 `map_id_mismatch`，说明配置和机器人当前地图仍然不一致。

## 第 3 步：采集 HOME_SAFE

人工把机器人开到“开始和结束都安全”的位置。

要求：

- 底盘周围没有障碍；
- 手臂如果回默认姿态不会撞东西；
- 从这里去 `GRAB_PRE` 有合理路径；
- 这里也适合作为整轮任务完成后的最终位置。

采集命令：

```bash
python3 rack_hybrid_docking_package/calibrate_station_from_current_pose.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --station HOME_SAFE \
  --mode full \
  --note "new site home safe"
```

这个脚本只读当前定位并写配置，不会主动移动机器人。

成功后会：

- 备份旧 `industrial_station_config.json`；
- 更新 `HOME_SAFE`；
- 在 `calibration_records/` 里写一份带时间戳的采集记录。

## 第 4 步：采集 GRAB_PRE

人工把机器人开到上料架前的抓料预备位。

选点原则：

- 机器人要大致正对上料架；
- 前方超声能看到料架；
- 离料架不要太近，要留给 `grab_fine_position` 一小段靠近距离；
- 后面手臂展开到抓料姿态时不会撞到料架和周边结构；
- 七根料都能从这个底盘位置配合手臂姿态抓到。

采集命令：

```bash
python3 rack_hybrid_docking_package/calibrate_station_from_current_pose.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --station GRAB_PRE \
  --mode full \
  --note "new site grab pre"
```

采完以后，必须只读确认：

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --preflight-only
```

## 第 5 步：采集 PLACE_PRE

人工把机器人开到下料架前的放料预备位。

选点原则：

- 机器人要大致正对下料架；
- 前方超声能看到下料架或目标结构；
- 距离不能太近，要留给 `place_fine_position` 一小段靠近距离；
- 放料动作、开爪和退出动作不能刮到架子；
- 放完料以后底盘后退 `local_retreat_m` 时，后方不能有障碍。

采集命令：

```bash
python3 rack_hybrid_docking_package/calibrate_station_from_current_pose.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --station PLACE_PRE \
  --mode full \
  --note "new site place pre"
```

## 第 6 步：采集 RECOVERY_SAFE

人工把机器人开到一个中间安全点。

这个点不是抓料点，也不是放料点。它的作用是让机器人从放料区回 HOME 时更稳定，不要直接穿过复杂区域。

采集命令：

```bash
python3 rack_hybrid_docking_package/calibrate_station_from_current_pose.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --station RECOVERY_SAFE \
  --mode full \
  --note "new site recovery safe"
```

## 第 7 步：空车验证四个导航点

四个站点采完以后，不要马上抓料。先空车验证导航闭环。

推荐顺序：

```text
HOME_SAFE -> GRAB_PRE -> PLACE_PRE -> RECOVERY_SAFE -> HOME_SAFE
```

如果有专门的 station 导航命令，就逐个站点跑；如果没有，至少先跑主程序 preflight：

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --preflight-only
```

空车导航要确认：

- 每个站点能到；
- 到点后朝向合理；
- 没有擦边；
- 没有反复规划失败；
- 日志里最终 `ok=true`；
- 定位置信度稳定。

空车导航没稳定前，不要进入抓料采点。

## 第 8 步：理解抓料点是什么

抓料点不是地图点。抓料点是机械臂和腰部的关节姿态。

当前主程序每根杆会找：

```text
calibration_records/rod01_grab_pose_latest.json
calibration_records/rod02_grab_pose_latest.json
...
calibration_records/rod07_grab_pose_latest.json
```

每个 JSON 里应该有机械臂关节，也可以包含腰部关节。主流程会用同一个文件分别给：

- `waist_for_grab`
- `arm_grab_pose`

换现场时，七根杆的相对位置可能变了，所以这七个抓料姿态通常都要重新采。

## 第 9 步：采集第 1 根抓料点

先把机器人放在 `GRAB_PRE` 附近，并确认空车导航已经能稳定到 `GRAB_PRE`。

然后现场人工把上半身调到“第 1 根杆的最佳抓取姿态”。

最佳抓取姿态的标准：

- 两个夹爪正对杆；
- 夹爪闭合后能夹住杆；
- 夹爪没有明显偏左偏右；
- 手臂关节没有接近极限；
- 腰部姿态不会让手臂或身体撞料架；
- 抓住后按当前后退参数退出时不会刮到周边。

采集命令：

```bash
python3 rack_hybrid_docking_package/capture_grab_calibration_point.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --rod-index 1 \
  --label "new site rod1 grab" \
  --note "manual teach rod1 grab pose"
```

这个命令只读状态并写 JSON，不会主动移动机器人。

它会生成类似：

```text
calibration_records/rod01_grab_calibration_YYYYMMDD_HHMMSS.json
calibration_records/rod01_grab_pose_YYYYMMDD_HHMMSS.json
calibration_records/rod01_grab_pose_latest.json
```

用了 `--profile` 以后，脚本会自动把本次采到的姿态写到 profile 里的
`rod01_grab_pose_latest.json`。如果只是想临时采一份记录、不想覆盖 latest，
加 `--no-update-latest`。

## 第 10 步：采集第 2 到第 7 根抓料点

重复第 9 步。

每根杆都要生成自己的 latest：

```text
rod01_grab_pose_latest.json
rod02_grab_pose_latest.json
rod03_grab_pose_latest.json
rod04_grab_pose_latest.json
rod05_grab_pose_latest.json
rod06_grab_pose_latest.json
rod07_grab_pose_latest.json
```

每采完一根，建议马上记录：

```text
第几根：
现场姿态：
是否偏左/偏右：
夹爪是否居中：
人工观察是否会碰撞：
对应文件：
```

不要靠记忆判断哪个 JSON 是最新。文件名里有时间戳，`latest` 文件必须明确指向你确认过的那一版。

## 第 11 步：理解放料点是什么

放料侧目前不是每根杆一套点，而是一套共享放料姿态链。

主程序当前用这些文件：

```text
calibration_records/rod07_place_waist_adjusted_latest.json
calibration_records/rod07_place_above_arm_latest.json
calibration_records/rod07_place_transition_arm_latest.json
calibration_records/rod07_place_transition2_arm_latest.json
```

它们分别表示：

| 文件 | 含义 |
| --- | --- |
| `rod07_place_waist_adjusted_latest.json` | 放料前腰部/身体姿态 |
| `rod07_place_above_arm_latest.json` | 放料架上方的手臂准备姿态 |
| `rod07_place_transition_arm_latest.json` | 从上方进入放料区的中间姿态 |
| `rod07_place_transition2_arm_latest.json` | 接近最终放料姿态前的第二个中间姿态 |

之后主程序还会根据 `TUNED` 做几个相对偏移：

```text
place_final_before_open_x_m
place_final_before_open_z_m
place_pull_x_m
place_pull_back_down_x_m
place_pull_back_down_z_m
place_pull_drop_after_x_m
place_pull_drop_z_m
```

所以放料效果由两部分共同决定：

1. 放料姿态 JSON；
2. 主程序里的放料偏移参数。

## 第 12 步：重新采放料姿态链

放料姿态链建议按从安全到危险的顺序采：

1. 腰部正对下料架姿态；
2. 手臂在下料架上方的安全姿态；
3. 手臂下降/靠近的第一个过渡姿态；
4. 手臂下降/靠近的第二个过渡姿态；
5. 最终放料姿态或最终放料偏移。

当前工程还没有完全通用的“放料一键采集脚本”。所以现场复刻时要更保守：

- 先保留旧 map20 放料链；
- 空手在新 `PLACE_PRE` 附近单步验证腰部和手臂能不能到位；
- 如果位置不对，再生成新的放料 JSON；
- 每生成一个新 JSON，都先单步 dry-run 或空手验证；
- 不要带杆直接试未知放料姿态。

如果只是需要在现有最终放料姿态基础上抬高一点，可以参考：

```bash
python3 rack_hybrid_docking_package/calibrate_direct_place_pose_offset.py \
  --dry-run
```

确认计划没问题后，现场确认安全，再运行真实采集：

```bash
python3 rack_hybrid_docking_package/calibrate_direct_place_pose_offset.py \
  --confirm-physical \
  --z-m 0.02
```

这个脚本会移动上半身，所以必须有人在现场看着。

## 第 13 步：检查主程序里的 TUNED 参数

打开：

```bash
rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py
```

找到：

```python
TUNED = {
    ...
}
```

最常需要根据新现场调整的是：

| 参数 | 当前作用 | 什么时候要调 |
| --- | --- | --- |
| `grab_final_stop_mm` | 抓料前超声精停距离 | 夹爪离杆太远或太近 |
| `grab_final_brake_margin_mm` | 抓料制动补偿 | 实际停车总是过冲或提前 |
| `grab_final_speed_mps` | 抓料精定位速度 | 靠近太猛或太慢 |
| `place_final_stop_mm` | 放料前超声精停距离 | 放料架前停车距离不合适 |
| `place_final_brake_margin_mm` | 放料制动补偿 | 放料前过冲或提前 |
| `place_final_speed_mps` | 放料精定位速度 | 靠近下料架太猛或太慢 |
| `pick_back_x_m` | 抓住后手臂总后退 | 抓住杆以后退出不够或太多 |
| `pick_back_down_x_m` | 抓住后第一段后退 | 第一段退出是否容易刮碰 |
| `pick_down_z_m` | 抓住后是否下压 | 当前已调成 `0.0`，一般不要随便改 |
| `place_final_before_open_x_m` | 开爪前向前偏移 | 放料最终前后位置 |
| `place_final_before_open_z_m` | 开爪前上下偏移 | 放料高度 |
| `place_pull_x_m` | 开爪后总后退 | 放完后退出是否安全 |
| `place_pull_drop_z_m` | 开爪后下移量 | 退出时是否会挂住 |
| `local_retreat_m` | 本地动作结束后底盘后退距离 | 后退空间不足或退出不够 |

调参原则：

- 一次只改一个或两个参数；
- 每次改完先跑单根；
- 单根通过再跑 1-7；
- 每次改动都写进交接文档；
- 不要在失败后连续乱改多个参数，否则无法知道哪一个改动有效。

## 第 14 步：先做配置完整性检查

正式跑之前，至少人工检查这些文件存在：

```bash
ls rack_hybrid_docking_package/profiles/map20_box528/profile.json
ls rack_hybrid_docking_package/profiles/map20_box528/industrial_station_config.json
ls rack_hybrid_docking_package/profiles/map20_box528/calibration_records/rod01_grab_pose_latest.json
ls rack_hybrid_docking_package/profiles/map20_box528/calibration_records/rod02_grab_pose_latest.json
ls rack_hybrid_docking_package/profiles/map20_box528/calibration_records/rod03_grab_pose_latest.json
ls rack_hybrid_docking_package/profiles/map20_box528/calibration_records/rod04_grab_pose_latest.json
ls rack_hybrid_docking_package/profiles/map20_box528/calibration_records/rod05_grab_pose_latest.json
ls rack_hybrid_docking_package/profiles/map20_box528/calibration_records/rod06_grab_pose_latest.json
ls rack_hybrid_docking_package/profiles/map20_box528/calibration_records/rod07_grab_pose_latest.json
ls rack_hybrid_docking_package/profiles/map20_box528/calibration_records/rod07_place_waist_adjusted_latest.json
ls rack_hybrid_docking_package/profiles/map20_box528/calibration_records/rod07_place_above_arm_latest.json
ls rack_hybrid_docking_package/profiles/map20_box528/calibration_records/rod07_place_transition_arm_latest.json
ls rack_hybrid_docking_package/profiles/map20_box528/calibration_records/rod07_place_transition2_arm_latest.json
```

再检查 Python 语法：

```bash
python3 -m py_compile \
  rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py \
  rack_hybrid_docking_package/run_site_7_rods_live.py \
  rack_hybrid_docking_package/site_profile.py \
  rack_hybrid_docking_package/validate_site_profile.py \
  rack_hybrid_docking_package/industrial_map_nav_guarded.py \
  rack_hybrid_docking_package/process_vision_capture.py \
  rack_hybrid_docking_package/calibrate_station_from_current_pose.py \
  rack_hybrid_docking_package/capture_grab_calibration_point.py \
  rack_hybrid_docking_package/analyze_industrial_cell_run.py
```

## 第 15 步：按 profile 跑只读/离线检查

先校验 profile 本身。这个命令只读本地文件，不会动机器人：

```bash
python3 rack_hybrid_docking_package/validate_site_profile.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json
```

如果要在没有 GDK 的本机做离线 smoke，只检查 profile 是否能被主程序吃进去，
可以显式跳过机器人状态读取：

```bash
./rack_hybrid_docking_package/industrial_cell_7_rods_single_debug.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --preflight-only \
  --skip-status-snapshot \
  --skip-readiness-check \
  --skip-process-check \
  --skip-file-check \
  --start-index 1 \
  --end-index 1
```

看输出里有没有：

- `site` 是当前 profile 的名字；
- `profile` 指向当前现场 profile；
- `station_config` 指向 profile 目录里的 `industrial_station_config.json`；
- `tuned` 和 profile 里的参数一致；
- 没有异常。

如果这一步都失败，绝对不要跑 live。

## 第 16 步：只读 preflight

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --preflight-only
```

只有通过以后，才允许进入单根 live 验证。

## 第 17 步：只跑第 1 根 live

第一次新地图复刻，不要直接跑七根。

先跑第 1 根：

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --start-index 1 \
  --end-index 1
```

这个入口会先提示你确认物理安全。只有输入确认口令后，才会真正启动 live。

如果这次跑单根的目的还包括后续评估视觉/AI 纠偏，在命令末尾加
`--vision-capture`。它不会改变运动控制，只会在抓料和放料本地步骤前后
保存图片、相机参数、TF 外参、关节状态和机器人状态；同时会在整个
`LOCAL_PICK`/`LOCAL_PLACE` 阶段按固定间隔连续采样：

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --start-index 1 \
  --end-index 1 \
  --vision-capture
```

默认数据目录：

```text
logs/map20_box528_vision_dataset_<时间>/
```

每个采集点一个子目录，里面的 `manifest.json` 是索引文件；真正图片文件
会在同级目录下，例如 `head_stereo_left.jpg`、`head_color.jpg`、
`head_depth.npy`、`hand_left_color.jpg`。后面做视觉方案评估时，不要只看
最终成功/失败，要把 `before_step`、`after_step`、`after_step_error` 都一起看。

连续采样默认每 `1.0s` 存一轮图。如果现场希望采得更密，可以这样跑：

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --start-index 1 \
  --end-index 1 \
  --vision-capture \
  --vision-capture-interval-s 0.5
```

现场观察重点：

- `NAV_TO_GRAB` 到点是否准确；
- yaw 是否正对上料架；
- 抓料超声精定位是否停在合适距离；
- 夹爪是否能稳定夹住杆；
- 抓住后退出是否刮碰；
- `NAV_TO_PLACE` 是否准确；
- 放料精定位是否停在合适距离；
- 放料高度是否合适；
- 开爪后杆是否落在正确位置；
- 放料后退出是否刮碰；
- 是否回到 `HOME_SAFE`；
- 最终是否 `holding_rod=false`。

成功标准：

```text
phase=MISSION_DONE
current_station=HOME_SAFE
holding_rod=false
return_code=0
```

## 第 18 步：分析第 1 根日志

跑完以后找日志，一般在：

```bash
logs/
```

运行分析器：

```bash
python3 rack_hybrid_docking_package/analyze_industrial_cell_run.py \
  logs/你的_run_log.log
```

重点看：

- final state；
- 是否有 Traceback；
- 哪个 phase 耗时异常；
- `grab_fine_position` 最终距离；
- `place_fine_position` 最终距离；
- yaw refine 误差；
- 有没有失败的 local step。

如果第 1 根失败，不要继续跑第 2 根。先按失败阶段处理。

## 第 19 步：第 1 根失败时怎么判断

常见失败按阶段分：

| 阶段 | 常见原因 | 处理 |
| --- | --- | --- |
| `NAV_TO_GRAB` | 地图点不准、路径不可达、定位差 | 重采 `GRAB_PRE` 或检查地图 |
| `LOCAL_PICK` 精定位失败 | 超声距离不合适、目标不在前方 | 调 `GRAB_PRE` 或 `grab_final_stop_mm` |
| `LOCAL_PICK` 抓不到 | 抓料姿态不准 | 重新采对应 `rodXX_grab_pose_latest.json` |
| 抓住后退出刮碰 | 后退方向或距离不合适 | 调 `pick_back_x_m` / `pick_back_down_x_m` |
| `NAV_TO_PLACE` | `PLACE_PRE` 不准或路径不好 | 重采 `PLACE_PRE` |
| `LOCAL_PLACE` 精定位失败 | 下料架距离不合适 | 调 `PLACE_PRE` 或 `place_final_stop_mm` |
| 放料位置不对 | 放料姿态或最终偏移不对 | 调放料 JSON 或 `place_final_before_open_*` |
| 放完退出刮碰 | pull-out 轨迹不合适 | 调 `place_pull_*` |
| 回 HOME 失败 | `RECOVERY_SAFE` 或 `HOME_SAFE` 不好 | 重采安全点 |

## 第 20 步：第 1 根通过后，跑第 2 根

第 1 根通过后，不要马上跑七根。再跑第 2 根：

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --start-index 2 \
  --end-index 2
```

第 2 根主要验证：

- 第二根抓料姿态是否独立正确；
- 同一套放料姿态是否能继续适用；
- 第 1 根放完以后，现场布局没有影响后续动作。

## 第 21 步：第 1、2 根都通过后，再跑 1-7

整轮命令：

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --start-index 1 \
  --end-index 7
```

整轮成功标准：

```text
rod_index=7
end_index=7
phase=MISSION_DONE
current_station=HOME_SAFE
holding_rod=false
return_code=0
```

跑完整轮以后必须保存：

- run log；
- checkpoint；
- snapshot；
- analysis；
- 本次使用的 `industrial_station_config.json`；
- 本次使用的所有 `*_latest.json`；
- 本次修改过的 `TUNED` 参数。

## 第 22 步：现场必须留下的交接记录

每换一个地图或现场，都应该写一份交接文档，至少包含：

```text
现场名称：
机器人 IP：
机器人工作目录：
地图 ID：
上料架位置说明：
下料架位置说明：
HOME_SAFE 怎么选的：
GRAB_PRE 怎么选的：
PLACE_PRE 怎么选的：
RECOVERY_SAFE 怎么选的：
七根抓料点文件：
放料姿态文件：
TUNED 参数改动：
单根验证日志：
整轮验证日志：
最终状态：
已知警告：
下次继续前先跑什么命令：
```

不要只写“已跑通”。必须写清楚跑通用的是哪一套文件。

## 当前 map20 成功基线

当前参考成功基线：

```text
机器人：agi@192.168.0.7
工作目录：/data/g2_industrial_cell_20260612/wxf/BOX_528_1
地图：map_id=20
主入口：rack_hybrid_docking_package/run_site_7_rods_live.py -> industrial_cell_7_rods_single_debug.py
最终状态：MISSION_DONE / HOME_SAFE / holding_rod=false
```

当前整轮命令：

```bash
source /home/agi/app/env.sh
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --start-index 1 \
  --end-index 7
```

当前只读 preflight：

```bash
source /home/agi/app/env.sh
cd /data/g2_industrial_cell_20260612/wxf/BOX_528_1
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --preflight-only
```

## 新人最容易犯的错误

1. 只改 `map_id`，没有重新采四个站点。
2. 直接复制旧地图的抓料点，以为手臂还能抓到。
3. 只跑第 1 根成功，就直接认为七根都没问题。
4. 把 checkpoint 当成真实物理状态，不看机器人现在到底在哪里。
5. 看到 GDK/PNC 的非阻断 warning 就以为任务失败，或者反过来看到 return code 为 0 就不看实物状态。
6. 修改多个参数后一起测试，最后不知道哪个改动有效。
7. 没有保存 latest 文件对应的时间戳来源。
8. 带杆测试未知放料点，没有先空手验证。

## 当前 profile 化入口

当前程序已经开始按 profile 方式整理。map20 成功基线在：

```text
rack_hybrid_docking_package/profiles/map20_box528/profile.json
```

先做本地 profile 校验：

```bash
python3 rack_hybrid_docking_package/validate_site_profile.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json
```

按 profile 做只读 preflight：

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --preflight-only
```

采站点和采抓料点也已经支持 `--profile`。新现场复制 `profiles/map20_box528`
以后，把命令里的 profile 路径替换成新现场 profile 即可。profile 方式会让
采集结果写进对应现场目录，避免把新地图点位混进旧 map20 目录。

按 profile 跑单根：

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --start-index 1 \
  --end-index 1
```

按 profile 跑整轮：

```bash
python3 rack_hybrid_docking_package/run_site_7_rods_live.py \
  --profile rack_hybrid_docking_package/profiles/map20_box528/profile.json \
  --start-index 1 \
  --end-index 7
```

`profile.json` 里统一写：

- `map_id`；
- station config 路径；
- 七根抓料点路径；
- 放料姿态链路径；
- TUNED 参数；
- 机器人工作目录；
- 已验证日志。

这样新人不需要在主程序里找散落的硬编码，只需要替换一个现场 profile。

## 最短复刻清单

现场新人只想知道“我到底要做什么”，就照这个清单执行：

```text
1. 进入机器人目录，source /home/agi/app/env.sh。
2. 确认当前地图 ID。
3. 更新 industrial_station_config.json 的 map_id。
4. 采 HOME_SAFE。
5. 采 GRAB_PRE。
6. 采 PLACE_PRE。
7. 采 RECOVERY_SAFE。
8. 空车验证四个站点。
9. 逐根采 rod01 到 rod07 抓料姿态。
10. 检查或重采放料姿态链。
11. 检查 TUNED 参数。
12. py_compile。
13. dry-run 第 1 根。
14. preflight-only。
15. live 跑第 1 根。
16. 分析第 1 根日志。
17. live 跑第 2 根。
18. 第 1、2 根都稳定后，live 跑 1-7。
19. 分析整轮日志。
20. 保存本现场交接文档。
```

只要这 20 步没有跳过，这套程序就可以从一个地图复刻到另一个地图。
