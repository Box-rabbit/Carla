# Configs Guide

`configs/` 下的当前主线配置分工如下：

- `scenarios/`: CARLA 场景真源，定义地图、ego、actors、指令、成功/失败条件、指标
- `scenario_annotations/`: LMDrive/Leaderboard 风格 annotation，负责 route id 到场景实现、触发条件、期望结果的映射
- `lmdrive/`: LMDrive/Voice2LMDrive 轻量桥接配置
- `metrics/`: 日志、事件、报告的字段 schema
- `scenarios_db/`: 场景索引数据库，偏向 route/scenario 匹配和上层流程索引
- `taxonomy/`: 场景分类与设计原则
