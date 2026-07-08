# 任务3：基础操控场景设计

## 目标
设计保持车道、加速/减速、变道、路口转向、停车等基础语音操控场景。

## 原则
短路线、固定起点终点、固定随机种子、固定指令触发点、明确成功/失败判定。

## S01_keep_lane_speed
- 地图：Town04 / Town05
- 天气：ClearNoon
- 指令：“保持当前车道，提速至 60km/h”
- 成功：未偏离车道，速度达到目标速度 ±5 km/h，无碰撞
- 指标：target_speed_error、route_completion、collision_count、lane_invasion_count、response_latency

## S02_lane_change
- 地图：Town04 / Town05
- 天气：ClearNoon
- 指令：“确认安全后向左变道”
- 成功：进入目标车道并稳定保持，无碰撞，无明显违规
- 指标：lane_change_success、lane_change_latency、collision_count、route_deviation

## S03_intersection_turn
- 地图：Town03 / Town05
- 天气：ClearNoon
- 指令：“前方路口右转”
- 成功：通过指定路口，进入目标道路，无红灯违规，无碰撞
- 指标：turn_success、route_completion、red_light_violation、collision_count
