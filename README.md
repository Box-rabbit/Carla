# Dongfeng CARLA Scenario Evaluation

面向东风 `XH-202602` 赛题的 CARLA 场景构建与指标评测仓库。

当前仓库的主线职责：

- 构建可复现的 CARLA 闭环测试场景
- 维护统一的场景 YAML / route XML / 评测指标体系
- 为后续 `LMDrive / Voice2LMDrive` 接入提供清晰边界

当前主线场景类别：

- `basic_control`
- `complex_obstacle`
- `emergency_response`

当前并不直接复用 `LMDrive` 官方 `leaderboard evaluator` 作为主运行入口，而是采用独立的 `carla_eval` runner、运行时日志采集和离线报告生成流程。

同时，仓库已经新增一层轻量 `LMDrive / CARLA Leaderboard` 风格 benchmark 入口：

- route XML: [routes/dongfeng_benchmark.xml](routes/dongfeng_benchmark.xml)
- scenario annotation: [configs/scenario_annotations/dongfeng_benchmark.yaml](configs/scenario_annotations/dongfeng_benchmark.yaml)
- unified runner: [carla_eval/run_benchmark.py](carla_eval/run_benchmark.py)
- 说明文档: [docs/pipeline/lmdrive_style_benchmark.md](docs/pipeline/lmdrive_style_benchmark.md)

## 先看哪里

第一次进入仓库，建议按这个顺序看：

1. `README.md`
2. [docs/README.md](docs/README.md)
3. [docs/scenario_design/pdf_three_scenarios_delivery.md](docs/scenario_design/pdf_three_scenarios_delivery.md)
4. [configs/README.md](configs/README.md)
5. [routes/README.md](routes/README.md)

## 当前状态

当前已完成 6 个基础 CARLA 闭环评测场景，并完成 PDF 要求的 3 个 Town05 连续长路线标准化场景：

- `S01_keep_lane_speed_60`
- `S02_lane_change`
- `S04_pedestrian_slowdown`
- `S05_cone_detour`
- `S07_cut_in_brake`
- `S08_rain_night_danger_slowdown`
- `S11_basic_control_scene1_5km`
- `S12_complex_obstacle_scene2_8km`
- `S13_extreme_emergency_scene3_6km`

这些场景均有固定配置、路线和运行日志；已有闭环报告的场景另提供离线评测报告：

- 固定随机种子
- 固定 route
- 固定 ego spawn 点
- 固定 actor spawn / 触发逻辑
- 有对应的运行日志；已完成离线评测的场景另有报告输出

场景总览见：
- [docs/scenario_design/pdf_three_scenarios_delivery.md](docs/scenario_design/pdf_three_scenarios_delivery.md)

配置与路线说明见：
- [configs/README.md](configs/README.md)
- [routes/README.md](routes/README.md)

## 仓库结构

```text
carla_eval/                  场景运行脚本、运行时检测、离线评测、报告生成
configs/scenarios/           场景配置唯一真源
configs/scenario_annotations/ LMDrive-style 场景 annotation
configs/lmdrive/             LMDrive / Voice2LMDrive 最小接入配置
configs/metrics/             指标输出 schema
configs/taxonomy/            场景分类与核心指标分类
routes/                      route XML
data/audio/                  按场景归档的离线语音 wav/json 样例
logs/                        场景运行日志输出
reports/                     单场景评测报告与汇总表
docs/scenario_design/        场景设计与 benchmark 映射
docs/metrics/                指标、日志、事件、报告设计说明
docs/pipeline/               LMDrive / benchmark 调研与接入说明
```

当前约定：

- 场景真源：`configs/scenarios/*.yaml`
- route 真源：短场景使用 `routes/*.xml`；`S11/S12/S13` 长路线均已导出 dense route XML，其中 `S12/S13` 另提供 LMDrive/Leaderboard 适配的稀疏 route XML

## 运行环境

- `CARLA 0.9.10.1`
- Python 环境需安装 CARLA Python API 与本仓库运行依赖
- 当前场景脚本默认连接 `localhost:2000`

## 快速运行

### 1. 运行单个场景

```bash
python carla_eval/run_carla_s01_keep_lane_speed.py
python carla_eval/run_carla_s02_lane_change.py
python carla_eval/run_carla_s04_pedestrian_slowdown.py
python carla_eval/run_carla_s05_cone_detour.py
python carla_eval/run_carla_s07_cut_in_brake.py
python carla_eval/run_carla_s08_rain_night_danger_slowdown.py
python carla_eval/run_carla_s11_basic_control_scene1.py
python carla_eval/run_carla_s12_complex_obstacle_scene2.py --voice-overlay
python carla_eval/run_carla_s13_extreme_emergency_scene3.py --voice-overlay
```

### 2. 使用统一 benchmark 入口

列出全部 route/scenario：

```bash
python carla_eval/run_benchmark.py --list
```

运行单个 route：

```bash
python carla_eval/run_benchmark.py --route-id S05_cone_detour
```

运行全部 benchmark：

```bash
python carla_eval/run_benchmark.py
```

默认输出：

- 帧日志：`logs/<category>/<scenario_id>/frames.jsonl`

其中：

- `S07` 默认日志目录为 `logs/emergency_response/S07_cut_in_brake_realistic_urgent/`
- 其他场景默认日志目录与 `scenario_id` 对应

### 3. 离线生成评测报告

```bash
python carla_eval/evaluate.py \
  --scenario_config configs/scenarios/basic_control/S01_keep_lane_speed_60.yaml \
  --frames logs/basic_control/S01_keep_lane_speed_60/frames.jsonl \
  --output_dir reports/basic_control/S01_keep_lane_speed_60
```

默认输出：

- `events.json`
- `evaluation_report.json`
- `evaluation_report.csv`

## 当前场景清单

### basic_control

- `S01_keep_lane_speed_60`
  - 目标：保持车道并提速至 `60 km/h`
  - 配置：[configs/scenarios/basic_control/S01_keep_lane_speed_60.yaml](configs/scenarios/basic_control/S01_keep_lane_speed_60.yaml)
  - 路线：[routes/basic_control/S01_keep_lane_speed_60.xml](routes/basic_control/S01_keep_lane_speed_60.xml)

- `S02_lane_change`
  - 目标：按指令向左变道并保持目标车道
  - 配置：[configs/scenarios/basic_control/S02_lane_change.yaml](configs/scenarios/basic_control/S02_lane_change.yaml)
  - 路线：[routes/basic_control/S02_lane_change.xml](routes/basic_control/S02_lane_change.xml)

- `S11_basic_control_scene1_5km`
  - 目标：对应 PDF 场景1基础操控工况；晴天白天城市道路净空连续驾驶 `5km`，正常车速约 `50 km/h`，完成 route 上全部真实路口左/右转、向左变道、提速至 `80 km/h`、减速至 `30 km/h`
  - 配置：[configs/scenarios/basic_control/S11_basic_control_scene1_5km.yaml](configs/scenarios/basic_control/S11_basic_control_scene1_5km.yaml)
  - 路线：[routes/basic_control/S11_basic_control_scene1_5km.xml](routes/basic_control/S11_basic_control_scene1_5km.xml)，并在 [routes/dongfeng_benchmark.xml](routes/dongfeng_benchmark.xml) 中注册统一 benchmark route id

### complex_obstacle

- `S04_pedestrian_slowdown`
  - 目标：检测前方行人并减速避让
  - 配置：[configs/scenarios/complex_obstacle/S04_pedestrian_slowdown.yaml](configs/scenarios/complex_obstacle/S04_pedestrian_slowdown.yaml)
  - 路线：[routes/complex_obstacle/S04_pedestrian_slowdown.xml](routes/complex_obstacle/S04_pedestrian_slowdown.xml)

- `S05_cone_detour`
  - 目标：检测锥桶后单车道左绕并回原车道
  - 配置：[configs/scenarios/complex_obstacle/S05_cone_detour.yaml](configs/scenarios/complex_obstacle/S05_cone_detour.yaml)
  - 路线：[routes/complex_obstacle/S05_cone_detour.xml](routes/complex_obstacle/S05_cone_detour.xml)

- `S12_complex_obstacle_scene2_8km`
  - 目标：对应 PDF 场景2复杂避障工况；阴天傍晚城市次干道连续驾驶 `8km`，串联完成前方行人减速避让、慢车左变道超越、公交站减速谨慎通过
  - 配置：[configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml](configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml)
  - 路线：[routes/complex_obstacle/S12_complex_obstacle_scene2_8km.xml](routes/complex_obstacle/S12_complex_obstacle_scene2_8km.xml)
  - LMDrive 路线：[routes/complex_obstacle/S12_complex_obstacle_scene2_8km_lmdrive.xml](routes/complex_obstacle/S12_complex_obstacle_scene2_8km_lmdrive.xml)

### emergency_response

- `S07_cut_in_brake`
  - 目标：应对前车切入急刹，基于距离与 `TTC` 触发应急制动
  - 配置：[configs/scenarios/emergency_response/S07_cut_in_brake.yaml](configs/scenarios/emergency_response/S07_cut_in_brake.yaml)
  - 路线：[routes/emergency_response/S07_cut_in_brake.xml](routes/emergency_response/S07_cut_in_brake.xml)

- `S08_rain_night_danger_slowdown`
  - 目标：雨夜低能见度环境下识别危险并保持安全低速
  - 配置：[configs/scenarios/emergency_response/S08_rain_night_danger_slowdown.yaml](configs/scenarios/emergency_response/S08_rain_night_danger_slowdown.yaml)
  - 路线：[routes/emergency_response/S08_rain_night_danger_slowdown.xml](routes/emergency_response/S08_rain_night_danger_slowdown.xml)

- `S13_extreme_emergency_scene3_6km`
  - 目标：对应 PDF 场景3极限应急语音操控工况；雨夜低能见度连续驾驶 `6km`，起始危险路况安全车速提示，随后完成突发车辆加塞紧急避让和施工路段减速并道
  - 配置：[configs/scenarios/emergency_response/S13_extreme_emergency_scene3_6km.yaml](configs/scenarios/emergency_response/S13_extreme_emergency_scene3_6km.yaml)
  - 路线：[routes/emergency_response/S13_extreme_emergency_scene3_6km.xml](routes/emergency_response/S13_extreme_emergency_scene3_6km.xml)
  - LMDrive 路线：[routes/emergency_response/S13_extreme_emergency_scene3_6km_lmdrive.xml](routes/emergency_response/S13_extreme_emergency_scene3_6km_lmdrive.xml)

## 指标与评测流程

当前评测采用三层结构：

1. 每帧日志
2. 事件检测
3. 最终报告生成

关键运行时检测模块：

- `RouteTracker`
- `LaneInvasionTracker`
- `RedLightViolationTracker`

关键指标包括：

- `task_completion_rate`
- `collision_count`
- `lane_invasion_count`
- `red_light_violation_count`
- `route_deviation_count`
- `target_speed_error_kmh`
- `subtask_missing_rate`
- `min_ttc`
- `emergency_response_latency_ms`

相关实现见：

- [configs/metrics/metric_schema.yaml](configs/metrics/metric_schema.yaml)
- [carla_eval/metrics/event_detector.py](carla_eval/metrics/event_detector.py)
- [carla_eval/metrics/report_generator.py](carla_eval/metrics/report_generator.py)

## 当前实现与 LMDrive 的关系

当前主线是：

- 先把东风三类场景变成可复现 CARLA 闭环测试场景
- 建立独立、可控、可解释的指标统计体系
- 后续再把 `LMDrive` 作为语言引导驾驶主模型接入当前评测底座

也就是说：

- 当前仓库已经有自己的场景运行与评测链路
- `LMDrive` 是上层模型接入方向，不是当前 runner 的直接依赖
- 新增的 `run_benchmark.py` 已经把场景入口、route 来源、scenario annotation 和 agent interface 向 LMDrive 风格对齐

相关说明见：

- [docs/pipeline/lmdrive_style_benchmark.md](docs/pipeline/lmdrive_style_benchmark.md)
- [docs/scenario_delivery/standalone_bundle_workflow.md](docs/scenario_delivery/standalone_bundle_workflow.md)

## 说明

- `configs/scenarios/*.yaml` 是当前场景定义的唯一真源
- 短场景 route 由 `routes/*.xml` 定义；S11/S12/S13 长路线均有 dense route XML，并在 `routes/dongfeng_benchmark.xml` 中注册统一 route id
- 早期 `docs/*/task*.md` 中部分文件属于设计稿或阶段性说明，阅读时应优先以当前 config、runner 和 report 为准
- 当前 README 仅描述仓库已落地的场景与评测能力，不代表语音链路、LMDrive 主模型接入、车规级轻量化部署已经全部完成
