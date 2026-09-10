# PDF Three-Scenario Delivery Index

本文档汇总 PDF 要求的 3 个标准化连续仿真工况：S11 场景1基础操控、S12 场景2复杂避障、S13 场景3极限应急语音操控。

## 总入口

- Benchmark routes: `routes/dongfeng_benchmark.xml`
- SimLingo-adapted routes: `routes/dongfeng_simlingo_benchmark.xml`
- Scenario annotations: `configs/scenario_annotations/dongfeng_benchmark.yaml`
- Voice matches: `configs/simlingo/route_audio_matches.yaml`
- Scenario bundles: `scenario_bundles/S11_basic_control_scene1_5km/`, `scenario_bundles/S12_complex_obstacle_scene2_8km/`, `scenario_bundles/S13_extreme_emergency_scene3_6km/`

## S11 场景1基础操控

- Scenario id: `S11_basic_control_scene1_5km`
- Category: `basic_control`
- Map: `Town05`
- Length target: `5.00 km` (`4.75-5.25 km` accepted during route validation)
- Config: `configs/scenarios/basic_control/S11_basic_control_scene1_5km.yaml`
- Global Route design: `docs/scenario_design/s11_global_route_design_v04.yaml`
- Voice package: `data/audio/S11/20260715/`
- Bundle: `scenario_bundles/S11_basic_control_scene1_5km/`

v0.4 design requirements:

- CARLA `0.9.15` with Bench2Drive/Leaderboard 2.0
- Global Route must be generated and agent-validated before actors and voice tasks
- Initial straight segment: at least `200 m`
- Minimum distance between route scenes/tasks: `200 m`
- Voice task targets: `250, 850, 1450, 2050, 2700, 3350, 4050, 4700 m`
- Voice coverage: keep lane, accelerate, right turn, left turn, left lane change,
  right lane change, slow down, final parking

The previous S11 route XML and its reference trajectory remain legacy assets and
are not the v0.4 route source.

## S12 场景2复杂避障

- Scenario id: `S12_complex_obstacle_scene2_8km`
- Category: `complex_obstacle`
- Map: `Town05`
- Length: about `8.19 km`
- Config: `configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml`
- Global Route design: `docs/scenario_design/s12_global_route_design_v04.yaml`
- Dense route XML: `routes/complex_obstacle/S12_complex_obstacle_scene2_8km.xml`
- SimLingo route XML: `routes/complex_obstacle/S12_complex_obstacle_scene2_8km_simlingo.xml`
- Voice package: `data/audio/S12/20260721/`
- Bundle: `scenario_bundles/S12_complex_obstacle_scene2_8km/`

Voice triggers:

- `280 m`: pedestrian crossing caution
- `1090 m`: slow vehicle overtake, left lane change and return
- `1720 m`: bus stop caution

SimLingo route action alignment:

- Overtake left lane change: `1130-1170 m`
- Return right lane change: `1210-1255 m`

The S12 v0.4 route source is the validated dense XML with an added initial
straight prefix, so the start segment stays quiet before the first obstacle.

## S13 场景3极限应急语音操控

- Scenario id: `S13_extreme_emergency_scene3_6km`
- Category: `emergency_response`
- Map: `Town05`
- Length: about `6.00 km`
- Config: `configs/scenarios/emergency_response/S13_extreme_emergency_scene3_6km.yaml`
- Dense route XML: `routes/emergency_response/S13_extreme_emergency_scene3_6km.xml`
- SimLingo route XML: `routes/emergency_response/S13_extreme_emergency_scene3_6km_simlingo.xml`
- Voice package: `data/audio/S13/20260722/`
- Bundle: `scenario_bundles/S13_extreme_emergency_scene3_6km/`

Voice triggers:

- `0 m`: 前方路况危险，保持安全车速
- `1080 m`: sudden cut-in emergency
- `2480 m`: construction merge left

SimLingo route action alignment:

- Construction merge left: `2520-2555 m`
- Return right lane change: `2650-2700 m`

## 运行示例

```bash
python carla_eval/run_carla_s11_basic_control_scene1.py --voice-overlay
python carla_eval/run_carla_s12_complex_obstacle_scene2.py --voice-overlay
python carla_eval/run_carla_s13_extreme_emergency_scene3.py --voice-overlay
```

S12/S13 的 SimLingo 交付路线用于 Leaderboard/SimLingo 导航对齐；当前自定义 evaluator 默认仍使用 dense design route，避免与场景控制器中的横向控制重复叠加。
