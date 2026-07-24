# LMDrive-Style Benchmark Runner

本文档说明当前仓库新增的 LMDrive/Leaderboard 风格场景组织方式。

## 目标

当前仓库保留已有可运行场景，同时新增一层统一 benchmark 编排：

```text
route XML
  -> RouteIndexer
  -> scenario annotation YAML
  -> DongfengRouteScenario
  -> ScenarioEvaluator
  -> frames.jsonl / report
```

这对应 LMDrive 的核心思想：

- route XML 负责 ego 全局路线；
- scenario annotation 负责场景类型、触发条件和期望结果；
- RouteScenario 把 route 和 scenario 组合成可运行实例；
- evaluator 统一运行和保存结果。

## 新增文件

- `carla_eval/run_benchmark.py`
  统一 benchmark 入口，替代分散运行脚本作为批量评测主入口。

- `routes/dongfeng_benchmark.xml`
  当前 7 个场景的总 route XML，route id 直接使用场景 id。
  其中 S11/S12/S13 在仓库中均有独立的 dense route XML；总 route XML
  负责统一注册 route id，旧场景 YAML 中保留的 `lane_trace` 仅用于兼容旧 runner。

- `configs/scenario_annotations/dongfeng_benchmark.yaml`
  场景 annotation 文件，描述 route id 到场景实现、配置、触发条件、期望结果的映射。

- `carla_eval/benchmark/`
  LMDrive-style benchmark 适配层。

- `carla_eval/agents/base_agent.py`
  标准 agent 接口，为后续 LMDrive agent adapter 预留。

## 运行

列出当前 benchmark 中所有 route/scenario：

```bash
python carla_eval/run_benchmark.py --list
```

运行单个 route：

```bash
python carla_eval/run_benchmark.py --route-id S05_cone_detour
```

运行全部 route：

```bash
python carla_eval/run_benchmark.py
```

常用参数：

- `--routes`: 指定 route XML，默认 `routes/dongfeng_benchmark.xml`
- `--scenarios`: 指定 annotation 文件，默认 `configs/scenario_annotations/dongfeng_benchmark.yaml`
- `--route-id`: 只运行某个 route
- `--repetitions`: 每条 route 重复次数
- `--checkpoint`: checkpoint 文件
- `--resume`: 从 checkpoint 恢复
- `--draw-route`: 在 CARLA 中绘制 route

## 与旧入口的关系

旧入口仍然保留：

```bash
python carla_eval/run_carla_s05_cone_detour.py
python carla_eval/run_carla_s11_basic_control_scene1.py
```

建议：

- 单场景调试时可以继续用旧入口；
- 汇报、批量评测、后续接 LMDrive 时优先用 `run_benchmark.py`；
- 新场景应同时补充 `routes/dongfeng_benchmark.xml` 和 `configs/scenario_annotations/dongfeng_benchmark.yaml`。

## 当前限制

当前实现是轻量 LMDrive-style，不是完整 CARLA Leaderboard：

- 没有直接复用 LMDrive 的 `LeaderboardEvaluator`；
- 没有完整 ScenarioRunner behavior tree；
- 背景交通仍由当前 `ScenarioEvaluator` 管理；
- 当前 rule/controller 仍在 `scenarios_impl` 内，后续 LMDrive 接入时再替换为 `BaseAgent` adapter。

长路线交付文件：

- S11 使用 `routes/basic_control/S11_basic_control_scene1_5km.xml`
- S12 使用 `routes/complex_obstacle/S12_complex_obstacle_scene2_8km_lmdrive.xml`
- S13 使用 `routes/emergency_response/S13_extreme_emergency_scene3_6km_lmdrive.xml`

这些 XML 是经过 CARLA 地图校验的交付路线；`routes/dongfeng_benchmark.xml`
中的 S11/S12/S13 条目只承担共享 benchmark 注册，不是完整长路线。

这个选择是为了保留当前东风场景的稳定性，同时把入口、route、scenario annotation 和 agent interface 逐步对齐 LMDrive。
