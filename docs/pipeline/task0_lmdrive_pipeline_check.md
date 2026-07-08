# 任务0：LMDrive 官方评测流程参考与当前仓库实现说明

## 目标
确认 LMDrive 官方闭环评测流程、依赖版本、route/scenario 文件格式、agent 入口、结果输出位置，以及东风三类自定义场景的接入方式。

## 结论

LMDrive 官方评测流程仍然是本项目的重要参考，但当前仓库的实际实现已经采用独立的 `carla_eval` runner、独立场景 YAML、独立指标统计脚本来完成东风三类场景闭环评测。

因此应区分两层含义：

- `LMDrive 官方 pipeline`：作为语言引导闭环驾驶框架与 CARLA Leaderboard 接入方式的参考；
- `当前仓库实现`：作为东风三类场景的实际运行与评测底座。

| 项目 | 官方 LMDrive 参考 | 当前仓库实际 |
|---|---|---|
| CARLA | `0.9.10.1` | `0.9.10.1` |
| 场景入口 | `leaderboard/` | `carla_eval/run_carla_*.py` |
| 场景定义 | 官方 route / scenario 文件 | `configs/scenarios/*.yaml` + `routes/*.xml` |
| 评测入口 | `leaderboard/scripts/run_evaluation.sh` | `carla_eval/evaluate.py` |
| 结果输出 | 官方 checkpoint / result json | `logs/.../frames.jsonl` + `reports/.../evaluation_report.json/csv` |
| 当前是否已直接接入 LMDrive 官方 evaluator | 参考方案 | 否，当前未直接使用 |

## 当前仓库场景接入策略

当前仓库第一阶段实际采用：

```text
新增自定义 route XML
+ 自定义 YAML 场景配置
+ 独立 CARLA runner
+ 独立事件检测与报告生成
```

后续如需把 LMDrive 真正接入当前场景库，可将：

- 文本/语音指令解析层接到 LMDrive；
- 车辆控制输出仍接入当前 `carla_eval` 场景 runner；
- 或在第二阶段再考虑回接官方 `leaderboard` / `scenario_runner`。

## 官方可复用指标
- route completion；
- driving score；
- collision with pedestrian / vehicle / static object；
- red light / stop sign infraction；
- outside route lanes；
- route deviation；
- timeout；
- blocked。

## 需要自定义补充的指标
- 目标速度误差；
- 指令响应延时；
- 端到端延时；
- 指定动作是否完成；
- 组合指令子任务遗漏率；
- 场景任务完成率；
- 语音/文本指令解析准确率；
- 多模态语义对齐精度。
