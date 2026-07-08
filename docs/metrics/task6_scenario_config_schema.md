# 任务6：场景配置文件 Schema 设计

## 目标
设计统一 YAML 场景配置格式，描述场景 ID、地图、天气、起终点、参与者、指令、触发条件、成功判定、失败条件和指标。

## 说明

本文件描述的是统一 schema 与推荐字段，不是某一个当前场景的逐字镜像。

当前仓库已经落地的真实场景配置以 `configs/scenarios/*.yaml` 为准。对于当前实现：

- 起点主要由 `ego.spawn_point` 指定；
- 终点主要由 `route.route_file + route_id` 隐式定义；
- 道路、天气、参与者、触发条件、成功判定均以具体场景 YAML 为唯一真源。

## 推荐字段
```yaml
scenario_id: "S01_keep_lane_speed_60"
category: "basic_control"
version: "v0.1"
description: "保持当前车道并提速到目标速度"
runtime:
  random_seed: 42
  carla_version: "0.9.10.1"
  fixed_delta_seconds: 0.05
  max_duration_seconds: 300
map:
  town: "Town03"
  weather: "ClearNoon"
ego:
  vehicle_type: "vehicle.tesla.model3"
  spawn_point: {x: -36.543, y: -198.423, z: 0.5, yaw: 1.44}
route:
  route_file: "routes/basic_control/S01_keep_lane_speed_60.xml"
  route_id: 0
actors:
  vehicles: []
  pedestrians: []
  static_props: []
instructions:
  - id: "cmd_001"
    trigger: {type: "time", value: 5.0}
    text_zh: "保持当前车道，提速至60公里每小时"
    text_en: "Keep the current lane and accelerate to 60 km/h."
    intent: {type: "target_speed_control", target_speed_kmh: 60, lane_action: "keep_lane"}
    expected_subtasks: ["keep_lane", "reach_target_speed"]
success_criteria:
  no_collision: true
  max_lane_invasion_count: 1
  route_completion_min: 0.9
  max_response_latency_ms: 150
  target_speed: {enabled: true, value_kmh: 60, tolerance_kmh: 8, required_hold_seconds: 2}
failure_criteria:
  collision: true
  route_deviation: true
  timeout: true
  blocked: true
metrics:
  required: ["route_completion", "collision_count", "target_speed_error", "response_latency_ms"]
```
