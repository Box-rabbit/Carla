# 任务6：场景配置文件 Schema 设计

## 目标
设计统一 YAML 场景配置格式，描述场景 ID、地图、天气、起终点、参与者、指令、触发条件、成功判定、失败条件和指标。

## 推荐字段
```yaml
scenario_id: "S01_keep_lane_speed"
category: "basic_control"
version: "v0.1"
description: "保持当前车道并提速到目标速度"
runtime:
  random_seed: 42
  carla_version: "0.9.10.1"
  fixed_delta_seconds: 0.05
  max_duration_seconds: 120
map:
  town: "Town04"
  weather: "ClearNoon"
ego:
  vehicle_type: "vehicle.tesla.model3"
  spawn_point: {x: 0.0, y: 0.0, z: 0.3, yaw: 0.0}
route:
  route_file: "routes/basic_control/dongfeng_basic_control_example.xml"
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
  route_completion_min: 0.95
  target_speed: {enabled: true, value_kmh: 60, tolerance_kmh: 5, required_hold_seconds: 3}
failure_criteria:
  collision: true
  route_deviation: true
  timeout: true
  blocked: true
metrics:
  required: ["route_completion", "collision_count", "target_speed_error", "response_latency_ms"]
```
