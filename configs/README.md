# Configs Guide

`configs/` 下的当前主线配置分工如下：

- `scenarios/`: CARLA 场景真源，定义地图、ego、actors、指令、成功/失败条件、指标
- `scenario_annotations/`: LMDrive/Leaderboard 风格 annotation，负责 route id 到场景实现、触发条件、期望结果的映射
- `lmdrive/`: LMDrive/Voice2LMDrive 轻量桥接配置；其中路线改造和动作对齐是维护输入，音频匹配是自动生成结果
- `metrics/`: 日志、事件、报告的字段 schema
- `taxonomy/`: 场景分类与设计原则

## 当前真源

场景运行时应优先读取以下配置：

- `scenarios/`: 场景行为和评测条件的唯一真源
- `scenario_annotations/dongfeng_benchmark.yaml`: 统一 route/scenario 注册和事件 annotation
- `../routes/`: route XML 真源；S11/S12/S13 的长路线以 dense XML 为准

`scenario_bundles/<scenario_id>/` 是可独立交付的场景副本，内部配置用于
独立运行和交付，不替代根目录下的场景真源。

## LMDrive 配置关系

`lmdrive/route_adaptations.yaml` 和 `lmdrive/route_action_alignment.yaml`
是人工维护的路线改造与语音动作对齐输入。它们分别描述：

- 稀疏 LMDrive route XML 如何从 dense route XML 生成
- 语音触发点、路线动作和纵向控制窗口如何对应

`lmdrive/route_audio_matches.yaml` 由
`carla_eval/tools/match_route_audio.py` 自动生成，不建议手工编辑。重新整理
`data/audio/` 或修改统一 annotation 后，应重新运行：

```bash
python carla_eval/tools/match_route_audio.py
```

该命令会覆盖全局音频匹配结果；每个 standalone bundle 中的
`route_audio_matches_<scenario_id>.yaml` 是对应场景的交付副本。

旧的 `scenarios_db/` Town03 索引已移除。当前 route/scenario 匹配统一使用
`routes/dongfeng_benchmark.xml` 与 `scenario_annotations/dongfeng_benchmark.yaml`。
