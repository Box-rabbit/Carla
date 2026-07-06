# 任务9：事件检测脚本设计

## 目标
实现指标统计第二层：事件检测。读取每帧日志，将连续轨迹和控制量转换为离散事件。

## 输入输出
输入：`logs/{scenario_id}/frames.jsonl` 与 `configs/scenarios/{scenario_id}.yaml`。  
输出：`logs/{scenario_id}/events.json`。

## 事件类型
instruction_triggered、speed_target_reached、speed_drop_started、lane_change_started、lane_change_completed、turn_completed、stop_completed、obstacle_avoided、emergency_brake_started、collision_happened、violation_happened、task_success、task_failure。

## 检测规则示例
- 目标速度：速度进入目标范围并持续 required_hold_seconds；
- 变道完成：lane_id 变为目标车道并稳定保持；
- 应急制动：brake 大于阈值或速度连续下降；
- 子任务遗漏：expected_subtasks 中任一子任务未产生对应 event。

## 接口
```python
detector = EventDetector(config)
events = detector.detect(frame_records)
```
