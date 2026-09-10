# Dongfeng CARLA Scenario Evaluation

面向东风 `XH-202602` 赛题的 CARLA 场景构建与指标评测仓库。

当前仓库的主线职责：

- 构建可复现的 CARLA 闭环测试场景
- 维护统一的场景 YAML / route XML / 评测指标体系
- 为后续 `SimLingo` 接入提供清晰边界

当前主线场景类别：

- `basic_control`
- `complex_obstacle`
- `emergency_response`

当前同时保留项目自有 `carla_eval` runner，并提供 CARLA 0.9.15 + 官方 Leaderboard 2.0 evaluator 入口。官方入口使用 `scripts/run_official_leaderboard.sh`，路线格式使用 `routes/dongfeng_leaderboard_2.0.xml`。

官方 evaluator 接入边界：路线 XML 已按官方 parser 校验，Agent 入口位于 `leaderboard_agents/dongfeng_agent.py`。真实 SimLingo 策略需要通过 `DONGFENG_POLICY_MODULE` 注入。S12/S13 的车辆、行人、公交站和施工锥桶已经转换为 ScenarioRunner 场景定义；S11 没有自定义 actor，保留空场景列表。

场景转换后的官方路线构建命令：

```bash
scripts/build_official_leaderboard_routes.sh
```

该命令会从项目路线重新生成 Leaderboard XML，并注入 `third_party/scenario_runner/srunner/scenarios/dongfeng_route_scenarios.py` 中的 S12/S13 场景定义。运行 evaluator 前也可以设置 `AUTO_BUILD_ROUTES=1` 自动执行构建。

同时，仓库已经新增一层轻量 `SimLingo / CARLA Leaderboard` 风格 benchmark 入口：

- route XML: [routes/dongfeng_simlingo_benchmark.xml](routes/dongfeng_simlingo_benchmark.xml)
- scenario annotation: [configs/scenario_annotations/dongfeng_benchmark.yaml](configs/scenario_annotations/dongfeng_benchmark.yaml)
- unified runner: [carla_eval/run_benchmark.py](carla_eval/run_benchmark.py)
- 说明文档: [docs/pipeline/simlingo_benchmark.md](docs/pipeline/simlingo_benchmark.md)

## 先看哪里

第一次进入仓库，建议按这个顺序看：

1. `README.md`
2. [docs/README.md](docs/README.md)
3. [docs/scenario_design/pdf_three_scenarios_delivery.md](docs/scenario_design/pdf_three_scenarios_delivery.md)
4. [configs/README.md](configs/README.md)
5. [routes/README.md](routes/README.md)

## 当前状态

当前交付主线是 PDF 要求的 3 个 Town05 连续长路线标准化场景：

- `S11_basic_control_scene1_5km`
- `S12_complex_obstacle_scene2_8km`
- `S13_extreme_emergency_scene3_6km`

归档目录中保留早期 6 个 Town03 短场景原型，但它们不属于当前赛题交付主线：

- [archive/legacy_short_scenarios](archive/legacy_short_scenarios)

S11/S12/S13 均有固定配置、路线和运行日志：

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
- [reports/README.md](reports/README.md)
- [data/audio/README.md](data/audio/README.md)

## 仓库结构

```text
carla_eval/                  场景运行脚本、运行时检测、离线评测、报告生成
configs/scenarios/           场景配置唯一真源
configs/scenario_annotations/ SimLingo-style 场景 annotation
configs/simlingo/            SimLingo / Voice2SimLingo 最小接入配置
configs/metrics/             指标输出 schema
configs/taxonomy/            场景分类与核心指标分类
routes/                      route XML
data/audio/                  按场景归档的离线语音 wav/json 样例和资产清单
logs/                        场景运行日志输出（不纳入 Git）
reports/                     PDF 交付报告
archive/legacy_short_scenarios/ 早期短场景归档，不参与主线交付
docs/scenario_design/        场景设计与 benchmark 映射
docs/metrics/                指标、日志、事件、报告设计说明
docs/pipeline/               SimLingo / benchmark 调研与接入说明
```

当前约定：

- 场景真源：`configs/scenarios/*.yaml`
- route 真源：场景 YAML 的 `route.route_file`；S11/S12/S13 长路线均已导出 dense route XML，其中 S12/S13 另提供 SimLingo/Leaderboard 适配的稀疏 route XML
- 默认 benchmark 套件：`configs/benchmark_suites.yaml` 的 `pdf_delivery`

## 运行环境

- `CARLA 0.9.15`
- Python 3.7/3.8 环境：`/data/hdt_workspace/my_env/carla0915`
- CARLA Python API 必须来自 CARLA 0.9.15
- 官方控制侧入口使用 Leaderboard 2.0 evaluator
- 当前场景脚本默认连接 `localhost:2000`

控制侧交付、接口定义和验收命令统一见：

- [docs/scenario_delivery/control_side_delivery.md](docs/scenario_delivery/control_side_delivery.md)
- `scripts/validate_scene_delivery.sh`

## 快速运行

控制侧接入 SimLingo 时优先使用官方 Leaderboard 2.0 入口：

```bash
cd /data/hdt_workspace/dongfeng
source scripts/env_carla0915.sh
export DONGFENG_POLICY_MODULE=your_simlingo_adapter
ROUTE_ID=S12_complex_obstacle_scene2_8km scripts/run_official_leaderboard.sh
```

交付前静态验收：

```bash
scripts/validate_scene_delivery.sh
```

### 1. 运行单个场景

```bash
python carla_eval/run_carla_s11_basic_control_scene1.py
python carla_eval/run_carla_s12_complex_obstacle_scene2.py --voice-overlay
python carla_eval/run_carla_s13_extreme_emergency_scene3.py --voice-overlay
```

### 2. 使用统一 benchmark 入口

列出默认 PDF 交付 route/scenario：

```bash
python carla_eval/run_benchmark.py --list
```

运行一个 PDF 交付场景：

```bash
python carla_eval/run_benchmark.py --route-id S12_complex_obstacle_scene2_8km
```

默认情况下统一 runner 保持场景 YAML 内的运行路线。仅在验证 SimLingo
适配 XML 时才传入 `--route-source benchmark` 和对应的总路线 XML。

默认输出：

- 帧日志：`logs/<category>/<scenario_id>/frames.jsonl`

其中，日志目录默认与 `scenario_id` 对应。

### 3. 离线生成评测报告

```bash
python carla_eval/evaluate.py \
  --scenario_config configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml \
  --frames logs/complex_obstacle/S12_complex_obstacle_scene2_8km/frames.jsonl \
  --output_dir reports/pdf_delivery/S12_complex_obstacle_scene2_8km
```

默认输出：

- `events.json`
- `evaluation_report.json`
- `evaluation_report.csv`

## 当前场景清单

### basic_control

- `S11_basic_control_scene1_5km`
  - 目标：对应 PDF 场景1基础操控工况；晴天白天城市道路净空连续驾驶 `5km`，正常车速约 `50 km/h`，完成 route 上全部真实路口左/右转、向左变道、提速至 `80 km/h`、减速至 `30 km/h`
  - 配置：[configs/scenarios/basic_control/S11_basic_control_scene1_5km.yaml](configs/scenarios/basic_control/S11_basic_control_scene1_5km.yaml)
  - 路线：[routes/basic_control/S11_basic_control_scene1_5km.xml](routes/basic_control/S11_basic_control_scene1_5km.xml)，并在 [routes/dongfeng_benchmark.xml](routes/dongfeng_benchmark.xml) 中注册统一 route id

### complex_obstacle

- `S12_complex_obstacle_scene2_8km`
  - 目标：对应 PDF 场景2复杂避障工况；阴天傍晚城市次干道连续驾驶 `8km`，串联完成前方行人减速避让、慢车左变道超越、公交站减速谨慎通过
  - 配置：[configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml](configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml)
  - 路线：[routes/complex_obstacle/S12_complex_obstacle_scene2_8km.xml](routes/complex_obstacle/S12_complex_obstacle_scene2_8km.xml)
  - SimLingo 路线：[routes/complex_obstacle/S12_complex_obstacle_scene2_8km_simlingo.xml](routes/complex_obstacle/S12_complex_obstacle_scene2_8km_simlingo.xml)

### emergency_response

- `S13_extreme_emergency_scene3_6km`
  - 目标：对应 PDF 场景3极限应急语音操控工况；雨夜低能见度连续驾驶 `6km`，起始危险路况安全车速提示，随后完成突发车辆加塞紧急避让和施工路段减速并道
  - 配置：[configs/scenarios/emergency_response/S13_extreme_emergency_scene3_6km.yaml](configs/scenarios/emergency_response/S13_extreme_emergency_scene3_6km.yaml)
  - 路线：[routes/emergency_response/S13_extreme_emergency_scene3_6km.xml](routes/emergency_response/S13_extreme_emergency_scene3_6km.xml)
  - SimLingo 路线：[routes/emergency_response/S13_extreme_emergency_scene3_6km_simlingo.xml](routes/emergency_response/S13_extreme_emergency_scene3_6km_simlingo.xml)

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

## 当前实现与 SimLingo 的关系

当前主线是：

- 先把东风三类场景变成可复现 CARLA 闭环测试场景
- 建立独立、可控、可解释的指标统计体系
- 后续再把 `SimLingo` 作为语言引导驾驶主模型接入当前评测底座

也就是说：

- 当前仓库已经有自己的场景运行与评测链路
- `SimLingo` 是上层模型接入方向，不是当前 runner 的直接依赖
- 新增的 `run_benchmark.py` 已经把场景入口、route 来源、scenario annotation 和 agent interface 向 SimLingo 风格对齐

相关说明见：

- [docs/pipeline/simlingo_benchmark.md](docs/pipeline/simlingo_benchmark.md)
- [docs/scenario_delivery/standalone_bundle_workflow.md](docs/scenario_delivery/standalone_bundle_workflow.md)

## 说明

- `configs/scenarios/*.yaml` 是当前场景定义的唯一真源
- 当前主线只维护 S11/S12/S13；早期短场景保留在 `archive/legacy_short_scenarios/`
- 早期 `docs/*/task*.md` 中部分文件属于设计稿或阶段性说明，阅读时应优先以当前 config、runner 和 report 为准
- 当前 README 仅描述仓库已落地的场景与评测能力，不代表语音链路、SimLingo 主模型接入、车规级轻量化部署已经全部完成
