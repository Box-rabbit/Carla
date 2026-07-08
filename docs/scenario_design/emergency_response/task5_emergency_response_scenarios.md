# 任务5：应急响应场景设计

## 目标
覆盖雨夜、低光照、突发加塞、前车急刹、施工路段、危险路况减速停车、紧急响应延时统计。

## S07_cut_in_brake
- 地图：Town04 / Town05
- 天气：ClearNoon 或 CloudySunset
- 动态参与者：side vehicle cut-in
- 指令：“突发车辆加塞，紧急避让”
- 成功：未碰撞，制动及时，最小 TTC 高于安全阈值
- 指标：emergency_response_latency、min_ttc、brake_reaction_success、collision_count

## S08_rain_night_danger_slowdown
- 地图：Town04 / Town06
- 天气：HardRainNight
- 指令：“前方路况危险，保持安全车速”
- 成功：车辆速度低于安全阈值，未碰撞，未违规
- 指标：target_speed_limit_success、collision_count、lane_invasion_count、mean_speed

## S09_construction_merge_stop
- 地图：Town04 / Town05
- 天气：HardRainNight 或 CloudySunset
- 静态参与者：cones / blocked lane
- 指令：“施工路段，减速并道至左侧车道”
- 成功：施工区前完成减速和并道，无碰撞，无违规
- 指标：merge_success、speed_drop、detour_success、collision_count、subtask_missing_rate

## 延时定义
- 模块端到端延时：control_output_time - instruction_input_time
- 行为响应延时：first_behavior_change_time - instruction_trigger_time
