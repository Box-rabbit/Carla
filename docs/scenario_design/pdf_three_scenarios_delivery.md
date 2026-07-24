# PDF Three-Scenario Delivery Index

本文档汇总 PDF 要求的 3 个标准化连续仿真工况：S11 场景1基础操控、S12 场景2复杂避障、S13 场景3极限应急语音操控。

## 总入口

- Benchmark routes: `routes/dongfeng_benchmark.xml`
- LMDrive-adapted routes: `routes/dongfeng_lmdrive_benchmark.xml`
- Scenario annotations: `configs/scenario_annotations/dongfeng_benchmark.yaml`
- Voice matches: `configs/lmdrive/route_audio_matches.yaml`
- Scenario bundles: `scenario_bundles/S11_basic_control_scene1_5km/`, `scenario_bundles/S12_complex_obstacle_scene2_8km/`, `scenario_bundles/S13_extreme_emergency_scene3_6km/`

## S11 场景1基础操控

- Scenario id: `S11_basic_control_scene1_5km`
- Category: `basic_control`
- Map: `Town05`
- Length: about `5.28 km`
- Config: `configs/scenarios/basic_control/S11_basic_control_scene1_5km.yaml`
- Route XML: `routes/basic_control/S11_basic_control_scene1_5km.xml`
- Voice package: `data/audio/S11/20260715/`
- Bundle: `scenario_bundles/S11_basic_control_scene1_5km/`

Voice triggers:

- `55 m`: right turn
- `145 m`: left turn
- `380 m`: lane change left
- `1460 m`: accelerate to 80 km/h
- `2860 m`: slow down to 30 km/h

## S12 场景2复杂避障

- Scenario id: `S12_complex_obstacle_scene2_8km`
- Category: `complex_obstacle`
- Map: `Town05`
- Length: about `8.08 km`
- Config: `configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml`
- Dense route XML: `routes/complex_obstacle/S12_complex_obstacle_scene2_8km.xml`
- LMDrive route XML: `routes/complex_obstacle/S12_complex_obstacle_scene2_8km_lmdrive.xml`
- Voice package: `data/audio/S12/20260721/`
- Bundle: `scenario_bundles/S12_complex_obstacle_scene2_8km/`

Voice triggers:

- `280 m`: pedestrian crossing caution
- `1090 m`: slow vehicle overtake, left lane change and return
- `1720 m`: bus stop caution

LMDrive route action alignment:

- Overtake left lane change: `1130-1170 m`
- Return right lane change: `1210-1255 m`

## S13 场景3极限应急语音操控

- Scenario id: `S13_extreme_emergency_scene3_6km`
- Category: `emergency_response`
- Map: `Town05`
- Length: about `6.00 km`
- Config: `configs/scenarios/emergency_response/S13_extreme_emergency_scene3_6km.yaml`
- Dense route XML: `routes/emergency_response/S13_extreme_emergency_scene3_6km.xml`
- LMDrive route XML: `routes/emergency_response/S13_extreme_emergency_scene3_6km_lmdrive.xml`
- Voice package: `data/audio/S13/20260722/`
- Bundle: `scenario_bundles/S13_extreme_emergency_scene3_6km/`

Voice triggers:

- `0 m`: 前方路况危险，保持安全车速
- `1080 m`: sudden cut-in emergency
- `2480 m`: construction merge left

LMDrive route action alignment:

- Construction merge left: `2520-2555 m`
- Return right lane change: `2650-2700 m`

## 运行示例

```bash
python carla_eval/run_carla_s11_basic_control_scene1.py --voice-overlay
python carla_eval/run_carla_s12_complex_obstacle_scene2.py --voice-overlay
python carla_eval/run_carla_s13_extreme_emergency_scene3.py --voice-overlay
```

S12/S13 的 LMDrive 交付路线用于 Leaderboard/LMDrive 导航对齐；当前自定义 evaluator 默认仍使用 dense design route，避免与场景控制器中的横向控制重复叠加。
