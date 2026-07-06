# 东风 XH-202602：CARLA 场景构建与指标评测模块任务 0-10 正式产出包

项目：面向智能驾驶的大模型应用场景研究  
负责模块：CARLA 场景构建与指标评测模块  
当前主线：基于 LMDrive 复现和改造，场景与评测体系兼容 LMDrive / CARLA Leaderboard pipeline。

## 包含内容

```text
docs/                         任务0-10正式文档
configs/taxonomy/             场景分类体系
configs/scenarios/            第一阶段场景YAML样例
configs/metrics/              指标schema
carla_eval/                   评测模块代码骨架
routes/                       route XML样例
refs/                         参考来源说明
```

## 总体方案

```text
配置文件管理场景
+ 三层指标评测：每帧日志 / 事件检测 / 最终指标统计
+ LMDrive/LangAuto 主接入方式
+ CARLA Leaderboard 基础路线与违规指标
+ ScenarioRunner 触发与 actor 编排思想
+ Bench2Drive 短路线、单能力、多场景评测思想
```

第一阶段优先实现短路线、固定 actor、固定触发点、固定随机种子，保证场景可控、可复现、可稳定输出指标。
