# Docs Guide

建议按下面顺序阅读：

1. [../README.md](../README.md)
2. [scenario_design/pdf_three_scenarios_delivery.md](scenario_design/pdf_three_scenarios_delivery.md)
3. [pipeline/lmdrive_style_benchmark.md](pipeline/lmdrive_style_benchmark.md)
4. [scenario_delivery/standalone_bundle_workflow.md](scenario_delivery/standalone_bundle_workflow.md)
5. [../configs/README.md](../configs/README.md)
6. [../routes/README.md](../routes/README.md)

目录说明：

- `scenario_design/`: 三场景交付索引
- `metrics/`: 运行时指标实现与 schema
- `pipeline/`: LMDrive/Leaderboard 参考流程与接入边界

阅读原则：

- 当前真实实现优先以 `configs/scenarios/*.yaml`、`routes/*.xml`、LMDrive 适配路线和 `carla_eval/` 代码为准。
- 具体场景行为以 `configs/scenarios/` 和 `carla_eval/` 实现为准。
