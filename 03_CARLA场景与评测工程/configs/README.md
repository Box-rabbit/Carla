# Configs Guide

`configs/` 下的当前主线配置分工如下：

- `scenarios/`: CARLA 场景真源，定义地图、ego、actors、指令、成功/失败条件、指标
- `scenario_annotations/`: LMDrive/Leaderboard 风格 annotation，负责 route id 到场景实现、触发条件、期望结果的映射
- `benchmark_suites.yaml`: 默认 PDF 交付套件的成员清单
- `lmdrive/`: LMDrive 路线改造配置
- `metrics/`: 日志、事件、报告的字段 schema
- `taxonomy/`: 场景分类与设计原则

## 当前真源

场景运行时应优先读取以下配置：

- `scenarios/`: 场景行为和评测条件的唯一真源
- `scenario_annotations/dongfeng_benchmark.yaml`: 统一 route/scenario 注册和事件 annotation
- `../routes/`: route XML 真源；S11/S12/S13 的长路线以 dense XML 为准

本交付包仅包含 S11/S12/S13 的当前版本；历史短场景保留在源仓库归档中，
不参与本目录的运行或评测。

## LMDrive 配置关系

`lmdrive/route_adaptations.yaml` 和 `../triggers/route_action_alignment.yaml`
是人工维护的路线改造与语音动作对齐输入。它们分别描述：

- 稀疏 LMDrive route XML 如何从 dense route XML 生成
- 语音触发点、路线动作和纵向控制窗口如何对应

`../triggers/route_audio_matches.yaml` 由
`carla_eval/tools/match_route_audio.py` 自动生成，不建议手工编辑。重新整理
`audio/` 或修改统一 annotation 后，应重新运行：

```bash
python code/carla_eval/tools/match_route_audio.py
```

该命令会覆盖本交付包中的全局音频匹配结果。当前 route/scenario 匹配统一使用
`routes/dongfeng_benchmark.xml` 与 `scenario_annotations/dongfeng_benchmark.yaml`。

## 运行路线选择

`code/carla_eval/run_benchmark.py` 默认使用 `pdf_delivery` 套件，并保持每个
场景 YAML 中的 `route.route_file` 作为运行路线真源。只有显式传入
`--route-source benchmark` 时，统一 benchmark XML 才会覆盖场景路线。

早期 S01/S02/S04/S05/S07/S08 短场景不在本交付包中，也不在活跃 benchmark
配置中。
