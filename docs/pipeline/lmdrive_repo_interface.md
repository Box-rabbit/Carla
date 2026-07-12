# LMDrive 与本仓库的接口说明

## 1. 文档目的

本文档用于说明 `LMDrive` 作为上层 language-guided driving 模型时，如何与当前 `Dongfeng CARLA Scenario Evaluation` 仓库进行对接。

当前仓库已经完成的是：

- 三类可复现场景
- 独立 CARLA runner
- 运行时指标采集
- 离线事件检测与报告生成

当前仓库尚未完成的是：

- `LMDrive` 模型本体的正式接入
- 语音链路 `ASR -> instruction`
- 多模态观测到 `LMDrive` 输入张量的适配

当前已经补上的最小接入骨架是：

- `configs/lmdrive/scenarios/S04_pedestrian_slowdown.yaml`
- `configs/lmdrive/triggers/S04_pedestrian_slowdown.yaml`
- `carla_eval/lmdrive/Voice2LMDriveAdapter`
- `carla_eval/lmdrive/LMDriveTriggerRuntime`

它的职责是：

- 沿用现有 route XML 与 CARLA 场景 YAML；
- 在 route-distance 条件满足时触发一次语音输入；
- 调用 `Voice2LMDrive` 适配层得到 intents / target speed 上限；
- 将输出写回当前 `frames.jsonl` 与离线报告。

因此，本文件重点回答三个问题：

1. 当前仓库已经给 `LMDrive` 留了哪些接口；
2. `LMDrive` 真正接入时应该插在哪一层；
3. 还缺哪些适配模块。

## 2. 当前总体关系

当前仓库与 `LMDrive` 的关系应理解为：

```text
东风三类场景与指标体系 = 当前仓库已实现
语言驱动闭环主模型 = LMDrive 作为后续接入方向
```

也就是说：

- 当前仓库是场景与评测底座；
- `LMDrive` 是未来接入的上层驾驶决策模型；
- 当前 runner 并未直接调用 `LMDrive` 官方 `leaderboard evaluator`。

## 3. 当前仓库中已经存在的接口边界

### 3.1 场景配置接口

每个场景 YAML 已经定义了与语言模型相关的字段：

- `instructions[].id`
- `instructions[].trigger`
- `instructions[].text_zh`
- `instructions[].text_en`
- `instructions[].intent`
- `instructions[].expected_subtasks`

示例文件：

- [configs/scenarios/basic_control/S01_keep_lane_speed_60.yaml](/data/hdt_workspace/dongfeng/configs/scenarios/basic_control/S01_keep_lane_speed_60.yaml:33)
- [configs/scenarios/basic_control/S02_lane_change.yaml](/data/hdt_workspace/dongfeng/configs/scenarios/basic_control/S02_lane_change.yaml:34)
- [configs/scenarios/complex_obstacle/S05_cone_detour.yaml](/data/hdt_workspace/dongfeng/configs/scenarios/complex_obstacle/S05_cone_detour.yaml:98)
- [configs/scenarios/emergency_response/S07_cut_in_brake.yaml](/data/hdt_workspace/dongfeng/configs/scenarios/emergency_response/S07_cut_in_brake.yaml:47)

这些字段的作用分工建议如下：

| 字段 | 面向 LMDrive | 面向本仓库评测 |
|---|---|---|
| `text_zh` / `text_en` | 作为自然语言 instruction 输入 | 用于日志回溯与任务说明 |
| `trigger` | 决定何时把 instruction 交给模型 | 决定 `instruction_id` 在日志中的激活时刻 |
| `intent` | 可选，作为结构化提示或监督信号 | 用于状态机、动作判定、指标解释 |
| `expected_subtasks` | 可选，帮助约束动作链路 | 用于 `subtask_missing_rate` 统计 |

在最小语音接入模式下，另外补一层独立 trigger 配置：

```yaml
scenario_id: S04_pedestrian_slowdown
trigger:
  type: route_distance
  distance_m: 40.0
  input_mode: wav
  audio_path: data/audio/S04_pedestrian_slowdown.wav
expected:
  intents: [PEDESTRIAN_CAUTION, SLOW_DOWN]
  target_speed_max_kmh: 25
  no_collision: true
```

这层配置不替代原有 `configs/scenarios/*.yaml`，只负责补充：

- 语音何时触发
- 语音输入来自哪个 `wav`
- 对 `Voice2LMDrive` 输出的最小期望

### 3.2 Agent 调用接口

仓库中已经存在一个最小 agent 形式的接口示例：

- [carla_eval/agents/rule_agent.py](/data/hdt_workspace/dongfeng/carla_eval/agents/rule_agent.py:1)

当前接口形式为：

```python
control, debug_info = agent.run_step(observation, instruction=None)
```

其中：

- `observation`：环境观测输入
- `instruction`：当前场景激活的自然语言指令
- `control`：车辆控制输出
- `debug_info`：调试信息和延时信息

这就是未来 `LMDriveAgentAdapter` 最直接的替换点。

### 3.3 运行日志接口

当前日志 schema 已经为语言模型接入预留了关键字段：

- `instruction_id`
- `asr_latency_ms`
- `parser_latency_ms`
- `model_latency_ms`
- `end_to_end_latency_ms`

相关位置：

- [configs/metrics/metric_schema.yaml](/data/hdt_workspace/dongfeng/configs/metrics/metric_schema.yaml:4)
- [docs/metrics/task7_metric_design.md](/data/hdt_workspace/dongfeng/docs/metrics/task7_metric_design.md:10)

各 runner 也已经在记录 `instruction_id`，例如：

- [carla_eval/run_carla_s01_keep_lane_speed.py](/data/hdt_workspace/dongfeng/carla_eval/run_carla_s01_keep_lane_speed.py:327)
- [carla_eval/run_carla_s05_cone_detour.py](/data/hdt_workspace/dongfeng/carla_eval/run_carla_s05_cone_detour.py:835)
- [carla_eval/run_carla_s07_cut_in_brake.py](/data/hdt_workspace/dongfeng/carla_eval/run_carla_s07_cut_in_brake.py:623)

因此，`LMDrive` 接入后只要返回：

- 当前执行的 instruction
- 模型推理耗时
- 控制输出

就能继续沿用当前日志和报告体系。

### 3.4 统一接入占位入口

仓库中已经有一个明确的统一接入占位脚本：

- [carla_eval/run_scenario.py](/data/hdt_workspace/dongfeng/carla_eval/run_scenario.py:1)

当前它还是 placeholder，注释已经说明：

- 后续接入 `LMDrive / CARLA Leaderboard evaluation`

因此，未来如果要做统一化接入，`run_scenario.py` 比直接改每个 `run_carla_sXX_*.py` 更适合做第一层适配入口。

## 4. 推荐的接入架构

### 4.1 推荐链路

建议采用以下接入链路：

```text
场景 YAML
-> 读取 instruction / intent / trigger
-> 构造 observation
-> 调用 LMDriveAgentAdapter.run_step(observation, instruction)
-> 返回 control + debug_info
-> 写入 frames.jsonl
-> 离线事件检测
-> 生成 evaluation_report.json/csv
```

### 4.2 分层职责

| 层 | 模块 | 职责 |
|---|---|---|
| 场景层 | `configs/scenarios/*.yaml` | 定义地图、路线、参与者、指令、成功判定 |
| 运行层 | `carla_eval/run_carla_*.py` 或 `run_scenario.py` | 驱动 CARLA、组织 tick、调用 agent |
| 适配层 | `LMDriveAgentAdapter` | 把本仓库 observation / instruction 转成 LMDrive 可接受输入 |
| 模型层 | `LMDrive` | 语言引导闭环驾驶决策 |
| 日志层 | `FrameLogger` / runner 记录 | 输出控制、状态、延时 |
| 评测层 | `event_detector` + `report_generator` | 事件检测与最终指标统计 |

## 5. 推荐的 LMDrive 输入输出约定

### 5.1 输入

建议 `LMDriveAgentAdapter.run_step()` 接收：

```python
observation = {
    "rgb_front": ...,
    "rgb_left": ...,
    "rgb_right": ...,
    "lidar": ...,
    "ego_speed_kmh": ...,
    "ego_pose": ...,
    "route_hint": ...,
    "traffic_context": ...,
}

instruction = {
    "instruction_id": "cmd_001",
    "text": "保持当前车道，提速至60公里每小时",
    "intent": {...},
    "trigger_time": 5.0,
}
```

其中：

- `text` 是给 `LMDrive` 的自然语言输入；
- `intent` 不一定要给 `LMDrive`，但建议保留给评测与安全监督使用；
- `route_hint` 可按需要提供 route waypoint / route progress / target waypoint。

### 5.2 输出

建议 `LMDrive` 适配后统一返回：

```python
control = {
    "steer": float,
    "throttle": float,
    "brake": float,
}

debug_info = {
    "agent_type": "LMDriveAgent",
    "asr_latency_ms": float,
    "parser_latency_ms": float,
    "model_latency_ms": float,
    "end_to_end_latency_ms": float,
}
```

如果 `LMDrive` 输出的是高层动作而不是底层控制，则需要在适配层增加：

- 高层动作到 `VehicleControl` 的转换器；
- 或高层动作到当前场景状态机目标的映射器。

## 6. 当前还缺的模块

当前仓库还没有以下正式模块：

### 6.1 `LMDriveAgentAdapter`

需要新增一个适配类，职责包括：

- 组装 `LMDrive` 所需 observation；
- 接收当前激活的 instruction；
- 调用 `LMDrive` 推理；
- 解析输出并返回 `control`；
- 回填 `model_latency_ms` 等调试字段。

### 6.2 observation builder

需要把当前 CARLA 环境状态组织为 `LMDrive` 需要的输入格式，例如：

- 多视角图像
- 激光雷达点云
- ego 速度与位姿
- route hint

### 6.3 指令桥接器

需要把场景 YAML 中的：

- `text_zh` / `text_en`
- `trigger`
- `intent`

组织为统一 instruction 对象，并在运行时按触发时刻激活。

### 6.4 控制输出转换器

如果 `LMDrive` 返回不是直接 `steer / throttle / brake`，则需要补：

- 高层决策动作到底层控制量的映射；
- 安全限幅；
- 紧急接管逻辑。

## 7. 推荐实施顺序

建议按以下顺序接入：

1. 先保持当前场景与评测体系不变；
2. 新增 `LMDriveAgentAdapter`，先在 `S01` / `S02` 两个基础场景验证；
3. 确认文本指令能真实影响车辆行为；
4. 再扩到 `S04` / `S05` / `S07` / `S08`；
5. 最后再接语音链路 `ASR -> instruction`。

原因是：

- `S01` / `S02` 对模型要求最低；
- 更容易验证 instruction 是否起作用；
- 出问题时更容易判断是场景层、模型层还是控制层的问题。

## 8. 当前最准确的表述

当前关于 `LMDrive` 与本仓库关系的最准确表述应为：

> 本仓库已经完成东风三类 CARLA 场景与统一指标评测底座，并为 `LMDrive` 预留了 instruction、intent、日志字段与 agent 调用边界。当前尚未正式接入 `LMDrive` 模型推理，后续将通过 `LMDriveAgentAdapter` 将其作为上层语言引导闭环驾驶模型接入当前 runner 与评测体系。

## 9. 相关文件

- [README.md](/data/hdt_workspace/dongfeng/README.md:1)
- [docs/pipeline/task0_lmdrive_pipeline_check.md](/data/hdt_workspace/dongfeng/docs/pipeline/task0_lmdrive_pipeline_check.md:1)
- [docs/pipeline/task1_benchmark_survey.md](/data/hdt_workspace/dongfeng/docs/pipeline/task1_benchmark_survey.md:1)
- [docs/scenario_design/benchmark_mapping.md](/data/hdt_workspace/dongfeng/docs/scenario_design/benchmark_mapping.md:1)
- [carla_eval/run_scenario.py](/data/hdt_workspace/dongfeng/carla_eval/run_scenario.py:1)
- [carla_eval/agents/rule_agent.py](/data/hdt_workspace/dongfeng/carla_eval/agents/rule_agent.py:1)
- [configs/metrics/metric_schema.yaml](/data/hdt_workspace/dongfeng/configs/metrics/metric_schema.yaml:1)
