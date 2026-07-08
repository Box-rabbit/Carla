# 任务0：LMDrive 官方评测流程与自定义场景接入方式确认

## 目标
确认 LMDrive 官方闭环评测流程、依赖版本、route/scenario 文件格式、agent 入口、结果输出位置，以及东风三类自定义场景的接入方式。

## 结论
LMDrive 当前评测流程基于 CARLA Leaderboard pipeline。建议优先使用 LMDrive 仓库自带的 `leaderboard/` 与 `scenario_runner/`，不要另起一套独立 CARLA runner。

| 项目 | 建议 |
|---|---|
| CARLA | 0.9.10.1 |
| Leaderboard | LMDrive 仓库自带版本 |
| ScenarioRunner | LMDrive 仓库自带版本 |
| 评测入口 | `leaderboard/scripts/run_evaluation.sh` |
| Evaluator | `leaderboard/leaderboard/leaderboard_evaluator.py` |
| Agent | `leaderboard/team_code/lmdriver_agent.py` |
| Config | `leaderboard/team_code/lmdriver_config.py` |
| Route | `langauto/benchmark_long.xml` 或自定义 XML |
| Scenario | `leaderboard/data/official/all_towns_traffic_scenarios_public.json` |
| Result | `results/sample_result.json` 或自定义 checkpoint |

## 自定义场景接入策略
第一阶段采用：

```text
新增自定义 route XML
+ 复用官方 scenario JSON
+ 修改 ROUTES 环境变量接入 LMDrive evaluation
```

第二阶段如需固定行人横穿、锥桶施工区、突发加塞等复杂触发行为，再新增自定义 scenario JSON 或 ScenarioRunner 脚本。

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
