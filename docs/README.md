# Docs Guide

建议按下面顺序阅读：

1. [../README.md](../README.md)
2. [scenario_design/completed_scenarios_overview.md](scenario_design/completed_scenarios_overview.md)
3. [scenario_design/benchmark_mapping.md](scenario_design/benchmark_mapping.md)
4. [metrics/task7_metric_design.md](metrics/task7_metric_design.md)
5. [pipeline/task0_lmdrive_pipeline_check.md](pipeline/task0_lmdrive_pipeline_check.md)

目录说明：

- `scenario_design/`: 场景设计、benchmark 映射、每类场景说明
- `metrics/`: 场景配置 schema、帧日志、事件检测、报告生成设计
- `pipeline/`: LMDrive/Leaderboard 参考流程与接入边界

阅读原则：

- 当前真实实现优先以 `configs/scenarios/*.yaml`、`routes/*.xml` / YAML `route.lane_trace`、`carla_eval/` 代码为准。
- `docs/*/task*.md` 中部分内容属于设计过程文档，可能早于当前实现。
