# 任务5：应急响应场景设计

## 目标
覆盖雨夜、低光照、突发加塞、前车急刹、施工路段、危险路况减速停车、紧急响应延时统计。

## 当前状态

当前仓库已落地并闭环跑通的应急响应场景为：

- `S07_cut_in_brake`
- `S08_rain_night_danger_slowdown`

`S09_construction_merge_stop` 与前车急刹类场景仍属于规划项，尚未在当前仓库中落地为实际 config 与 runner。

## S07_cut_in_brake
- 地图：`Town03`
- 天气：`CloudySunset`
- 路线：`routes/emergency_response/S07_cut_in_brake.xml`
- 动态参与者：side vehicle cut-in
- 指令：“突发车辆加塞并急刹，立即制动保持安全距离”
- 成功：未碰撞，制动及时，最小 TTC 高于安全阈值，保持安全跟驰距离
- 指标：emergency_response_latency、min_ttc、brake_reaction_success、collision_count

## S08_rain_night_danger_slowdown
- 地图：`Town03`
- 天气：`CustomHardRainNight`
- 路线：`routes/emergency_response/S08_rain_night_danger_slowdown.xml`
- 指令：“前方路况危险，保持安全车速”
- 成功：车辆速度低于安全阈值，未碰撞，未违规
- 指标：target_speed_limit_success、collision_count、lane_invasion_count、mean_speed

## S09_construction_merge_stop
- 当前状态：规划中，当前仓库尚未落地
- 参考地图：Town03
- 天气：HardRainNight 或 CloudySunset
- 静态参与者：cones / blocked lane
- 指令：“施工路段，减速并道至左侧车道”
- 成功：施工区前完成减速和并道，无碰撞，无违规
- 指标：merge_success、speed_drop、detour_success、collision_count、subtask_missing_rate

## 延时定义
- 模块端到端延时：control_output_time - instruction_input_time
- 行为响应延时：first_behavior_change_time - instruction_trigger_time
