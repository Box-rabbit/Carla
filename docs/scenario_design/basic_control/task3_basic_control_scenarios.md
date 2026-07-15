# 任务3：基础操控场景设计

## 目标
设计保持车道、加速/减速、变道、路口转向、停车等基础语音操控场景。

## 原则
短路线、固定起点终点、固定随机种子、固定指令触发点、明确成功/失败判定。

## 当前状态

当前仓库已落地并闭环跑通的基础操控场景为：

- `S01_keep_lane_speed_60`
- `S02_lane_change`
- `S11_basic_control_scene1_5km`

`S03_intersection_turn` 仍属于规划项，尚未在当前仓库中单独落地为短路线 config 与 runner。PDF 场景1对应的长路线基础操控已由 `S11_basic_control_scene1_5km` 承接。

## S01_keep_lane_speed_60
- 地图：`Town03`
- 天气：ClearNoon
- 路线：`routes/basic_control/S01_keep_lane_speed_60.xml`
- 指令：“保持当前车道，提速至 60km/h”
- 成功：未偏离路线走廊，达到目标速度并保持指定时长，无碰撞
- 指标：target_speed_error、route_completion、collision_count、lane_invasion_count、response_latency

## S02_lane_change
- 地图：`Town03`
- 天气：ClearNoon
- 路线：`routes/basic_control/S02_lane_change.xml`
- 指令：“确认安全后向左变道”
- 成功：进入目标车道并稳定保持，无碰撞，无明显违规
- 指标：lane_change_success、lane_change_latency、collision_count、route_deviation

## S11_basic_control_scene1_5km
- 地图：`Town05`
- 天气：ClearNoon
- 路线：`route.mode: carla_lane_trace`，由 CARLA waypoint 拓扑按固定起点、固定分支决策生成约 `5km` 路线
- 指令：当前支持通过 `configs/lmdrive/route_audio_matches.yaml` 将 5 条离线语音匹配到 S11 route/action，并可用 `--voice-overlay` 固定窗口显示语音文本；车辆控制仍由场景控制器执行，尚未由真实 LMDrive/VLA 模型驱动
- 子任务：保持路线连续驾驶、正常车速约 `50km/h`、完成 route 上全部自动检测到的真实路口左/右转、向左变道、提速至 `80km/h`、减速至 `30km/h`
- 成功：完成 `5km` 进度，全部转弯、变道、提速、减速子任务完成，无碰撞，无红灯/车道/路线偏离失败
- 指标：route_completion、collision_count、lane_invasion_count、red_light_violation_count、target_speed_error、subtask_completion_rate
- 交通路况：场景层默认保留 CARLA 地图自带 traffic light；如需演示可通过 `traffic_conditions.traffic_lights` 固定红绿灯状态。红灯停车由 VLA / Agent 决策，场景层只记录 `active_traffic_light_state`、`active_stop_line_progress_m` 并统计 `red_light_violation`。

## S03_intersection_turn
- 当前状态：规划中，当前仓库尚未落地
- 参考地图：Town03
- 天气：ClearNoon
- 指令：“前方路口右转”
- 成功：通过指定路口，进入目标道路，无红灯违规，无碰撞
- 指标：turn_success、route_completion、red_light_violation、collision_count
