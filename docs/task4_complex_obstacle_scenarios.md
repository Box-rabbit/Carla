# 任务4：复杂避障场景设计

## 目标
覆盖行人横穿、公交站减速、慢车超越、锥桶/施工障碍绕行、避障后回归原车道、组合指令子任务检测。

## S04_pedestrian_slowdown
- 地图：Town03 / Town05
- 天气：CloudySunset
- 动态参与者：pedestrian crossing
- 指令：“看到前方行人，减速避让”
- 成功：与行人保持安全距离，无碰撞，速度明显下降
- 指标：min_distance_to_pedestrian、speed_drop、collision_count、response_latency

## S05_cone_detour
- 地图：Town04 / Town05
- 天气：CloudySunset
- 静态参与者：cones / construction props
- 指令：“前方施工锥桶，减速绕行后回到原车道”
- 成功：未碰撞障碍物，完成绕行，回归目标车道
- 指标：detour_success、return_to_lane_success、collision_count、subtask_missing_rate

## S06_slow_vehicle_overtake
- 地图：Town04 / Town05
- 天气：CloudySunset
- 动态参与者：slow vehicle ahead
- 指令：“避让前方慢车，确认安全后向左变道超越”
- 成功：安全变道，超过慢车，无碰撞，无安全距离违规
- 指标：overtake_success、safe_distance_violation、lane_change_success、collision_count

## 组合指令拆解示例
“看到前方横穿马路的行人，减速避让后向左变道超越慢车”拆为：detect_pedestrian、slow_down、avoid_pedestrian、check_left_lane_safe、change_lane_left、overtake_slow_vehicle、continue_driving。
