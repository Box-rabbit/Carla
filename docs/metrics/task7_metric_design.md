# 任务7：指标统计方法设计

## 三层指标架构
```text
第一层：每帧日志
第二层：事件检测
第三层：最终指标统计
```

## 每帧日志字段
记录 timestamp、frame、scenario_id、instruction_id、ego 位置、ego_speed_kmh、steer、throttle、brake、collision、lane_invasion、red_light_violation、route_deviation、distance_to_front_actor、asr_latency_ms、parser_latency_ms、model_latency_ms、end_to_end_latency_ms。

## 事件检测
检测 speed_target_reached、lane_change_completed、turn_completed、stop_completed、obstacle_avoided、emergency_brake_started、collision_happened、violation_happened、task_success、task_failure。

## 最终指标
| 指标 | 计算方法 |
|---|---|
| task_completion_rate | 成功场景数 / 总场景数 |
| collision_count | collision event 总数 |
| violation_count | lane/red light/route deviation 等违规总数 |
| mean_response_latency_ms | 行为响应延时均值 |
| mean_end_to_end_latency_ms | ASR + parser + model + control 总延时均值 |
| target_speed_error | 目标区间内 abs(actual-target) 均值 |
| subtask_missing_rate | 未完成子任务数 / 总子任务数 |
| action_success_rate | 指定动作完成数 / 指定动作总数 |
| min_ttc | 应急场景最小 TTC |
| route_completion | 复用 Leaderboard 或自定义 route progress |

## 多模态语义对齐精度工程定义
对于一条指令中提到的目标对象、动作、方向或位置，系统是否正确关联到场景中的 ground truth actor、触发点和车辆行为。
