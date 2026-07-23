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
- 实时语音链路 `麦克风/ASR -> Voice2LMDrive -> LMDrive policy`

当前仓库已经完成的是：

- 离线语音识别结果与 wav 文件的 route/action 匹配
- S11 语音输入固定窗口显示
- 语音触发字段写入 `frames.jsonl`

当前仓库已经新增的 LMDrive 输入适配层：

- `carla_eval/lmdrive/input_adapter.py`
- `LMDriveInputAdapter`
- `LMDriveAgentAdapter`

它已经能把当前 runner 的 camera / LiDAR / speed / route hint / instruction 整理成 LMDrive-style 字段；但尚未加载和调用 LMDrive 官方权重。

当前已经补上的最小接入骨架是：

- `configs/lmdrive/scenarios/S04_pedestrian_slowdown.yaml`
- `configs/lmdrive/triggers/S04_pedestrian_slowdown.yaml`
- `configs/lmdrive/route_audio_matches.yaml`
- `carla_eval/tools/match_route_audio.py`
- `carla_eval/lmdrive/Voice2LMDriveAdapter`
- `carla_eval/lmdrive/LMDriveTriggerRuntime`
- `carla_eval/lmdrive/RouteAudioRuntime`
- `carla_eval/visualization/FixedVoiceOverlay`

它的职责是：

- 沿用现有 route XML 与 CARLA 场景 YAML；
- 在 route-distance 条件满足时触发一次语音输入；
- 调用 `Voice2LMDrive` 适配层得到 intents / target speed 上限；
- 将 `voice_results/**/*.wav` 和 `voice_results/**/*.json` 中的离线语音记录匹配到 S11/S12 route/action；
- 在运行时按 `route_progress_m` 固定窗口显示当前触发语音；
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

当前 S11 使用的多语音 route/action 匹配文件是：

- [configs/lmdrive/route_audio_matches.yaml](/data/hdt_workspace/dongfeng/configs/lmdrive/route_audio_matches.yaml:1)

它由以下脚本生成：

```bash
python carla_eval/tools/match_route_audio.py
```

运行 S11 时可以用固定窗口显示当前语音输入：

```bash
python carla_eval/run_carla_s11_basic_control_scene1.py --voice-overlay
```

### 3.2 Agent 调用接口

仓库中已经存在一个统一 agent 接口：

- [carla_eval/agents/base_agent.py](/data/hdt_workspace/dongfeng/carla_eval/agents/base_agent.py:1)

当前接口形式为：

```python
throttle, brake, steer = agent.run_step(input_data, timestamp, instruction=None)
```

其中：

- `input_data`：环境观测、ego、actors、scenario state 和 cfg
- `timestamp`：当前仿真时间
- `instruction`：当前场景激活的自然语言指令
- 返回值：底层车辆控制 `(throttle, brake, steer)`

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

### 3.4 统一 benchmark 接入入口

仓库中已经有一个明确的统一 benchmark 入口脚本：

- [carla_eval/run_benchmark.py](/data/hdt_workspace/dongfeng/carla_eval/run_benchmark.py:1)

它负责按 `routes/dongfeng_benchmark.xml` 和 `configs/scenario_annotations/dongfeng_benchmark.yaml` 组织 route/scenario，并通过 agent 接口调用当前场景控制器或后续 LMDrive adapter。

因此，后续真正接入 LMDrive 时应优先扩展 `run_benchmark.py` 的 agent 选择逻辑，而不是恢复旧的占位入口。

## 4. 推荐的接入架构

### 4.1 推荐链路

当前推荐接入链路：

```text
场景 YAML
-> 读取 instruction / intent / trigger
-> 构造 observation
-> 调用 LMDriveAgentAdapter.run_step(input_data, timestamp, instruction)
-> 返回 throttle / brake / steer
-> 写入 frames.jsonl
-> 离线事件检测
-> 生成 evaluation_report.json/csv
```

### 4.2 分层职责

| 层 | 模块 | 职责 |
|---|---|---|
| 场景层 | `configs/scenarios/*.yaml` | 定义地图、路线、参与者、指令、成功判定 |
| 运行层 | `carla_eval/run_carla_*.py` 或 `run_benchmark.py` | 驱动 CARLA、组织 tick、调用 agent |
| 适配层 | `LMDriveAgentAdapter` | 把本仓库 observation / instruction 转成 LMDrive 可接受输入 |
| 模型层 | `LMDrive` | 语言引导闭环驾驶决策 |
| 日志层 | `FrameLogger` / runner 记录 | 输出控制、状态、延时 |
| 评测层 | `event_detector` + `report_generator` | 事件检测与最终指标统计 |

## 5. 推荐的 LMDrive 输入输出约定

### 5.1 输入

当前 `LMDriveAgentAdapter.run_step()` 接收的是 evaluator 传入的 `input_data`，内部再转换为 LMDrive-style 字段：

```python
input_data = {
    "scenario": ...,
    "ego": ...,
    "actors": ...,
    "state": ...,
    "obs": {
        "rgb_front": ...,
        "rgb_left": ...,
        "rgb_right": ...,
        "rgb_rear": ...,
        "lidar": ...,
        "speed_kmh": ...,
        "route_metrics": ...,
        "route_tracker": ...,
    },
    "cfg": ...,
}

instruction = {
    "instruction_id": "cmd_001",
    "text": "保持当前车道，提速至60公里每小时",
    "intent": {...},
    "trigger_time": 5.0,
}
```

`LMDriveInputAdapter` 会进一步生成：

- `rgb_front / rgb_left / rgb_right / rgb_rear`
- `rgb_center`
- `lidar`
- `velocity`
- `measurements`
- `target_point`
- `route_hint`
- `text_input`

### 5.2 输出

当前统一 agent 接口返回：

```python
throttle, brake, steer = agent.run_step(input_data, timestamp, instruction)
```

如果 `LMDrive` 输出的是高层动作而不是底层控制，则需要在适配层增加：

- 高层动作到 `VehicleControl` 的转换器；
- 或高层动作到当前场景状态机目标的映射器。

## 6. 当前还缺的模块

当前仓库还没有以下正式模块：

### 6.1 `LMDriveAgentAdapter`

当前已新增一个适配类，职责包括：

- 组装 `LMDrive` 所需 observation；
- 接收当前激活的 instruction；
- 将 camera / LiDAR / speed / route hint 转成 LMDrive-style 输入字段；
- 调用外部注入的 `policy.run_step(...)` 或 `policy.predict(...)`；
- 解析输出并返回 `(throttle, brake, steer)`。

当前还没有完成的是：加载 LMDrive 官方模型权重并实现真实 `policy`。

### 6.2 observation builder

当前 `ObservationBuilder` 已采集并输出：

- `rgb_front`
- `rgb_left`
- `rgb_right`
- `rgb_rear`
- `lidar`

`LMDriveInputAdapter` 会进一步生成：

- `rgb_front / rgb_left / rgb_right / rgb_rear`: RGB `uint8` 图像；
- `rgb_center`: front camera 中心裁剪；
- `lidar`: `40000 x 4` padded LiDAR tensor；
- `num_points`: 有效 LiDAR 点数；
- `velocity`: `[[m/s]]`；
- `measurements`: `[x, y, yaw_rad, speed_mps]`；
- `target_point`: ego 坐标系下的前方 route target；
- `route_hint`: route progress、completion、target world/local 信息；
- `text_input`: 当前 instruction 文本列表。

### 6.3 指令 / 语音桥接器

当前已经有两条桥接路径：

- `LMDriveTriggerRuntime`: 读取单场景 trigger YAML，按 time/route_distance 触发；
- `RouteAudioRuntime`: 读取 `route_audio_matches.yaml`，按 `route_progress_m` 激活对应语音输入；
- `FixedVoiceOverlay`: 将当前触发语音固定显示在屏幕窗口。

仍未完成的是：实时 ASR、真实 Voice2LMDrive 模型调用、以及将语音结果真正送入 LMDrive 官方 policy 影响车辆控制。

### 6.4 控制输出转换器

如果 `LMDrive` 返回不是直接 `steer / throttle / brake`，则需要补：

- 高层决策动作到底层控制量的映射；
- 安全限幅；
- 紧急接管逻辑。

## 7. S11 接入 LMDrive 的具体操作

### 7.1 当前结论

`S11_basic_control_scene1_5km` 已经具备接入 LMDrive 的接口基础，但还不是“直接加载 LMDrive 官方权重即可运行”的状态。

已经具备：

- S11 场景配置：[configs/scenarios/basic_control/S11_basic_control_scene1_5km.yaml](/data/hdt_workspace/dongfeng/configs/scenarios/basic_control/S11_basic_control_scene1_5km.yaml:1)
- S11 单场景入口：[carla_eval/run_carla_s11_basic_control_scene1.py](/data/hdt_workspace/dongfeng/carla_eval/run_carla_s11_basic_control_scene1.py:1)
- 统一 benchmark route id：[routes/dongfeng_benchmark.xml](/data/hdt_workspace/dongfeng/routes/dongfeng_benchmark.xml:1)
- benchmark annotation：[configs/scenario_annotations/dongfeng_benchmark.yaml](/data/hdt_workspace/dongfeng/configs/scenario_annotations/dongfeng_benchmark.yaml:1)
- camera / LiDAR / speed / route hint 观测构造：[carla_eval/sensors/observation_builder.py](/data/hdt_workspace/dongfeng/carla_eval/sensors/observation_builder.py:1)
- LMDrive-style 输入转换：[carla_eval/lmdrive/input_adapter.py](/data/hdt_workspace/dongfeng/carla_eval/lmdrive/input_adapter.py:1)
- 统一 agent 接口：[carla_eval/agents/base_agent.py](/data/hdt_workspace/dongfeng/carla_eval/agents/base_agent.py:1)

仍需补齐：

- 官方 LMDrive repo/checkpoint 的加载 wrapper，即真正的 `policy` 对象。
- `run_benchmark.py` 或 S11 runner 的 agent 选择参数，例如 `--agent scenario_controller|lmdrive`。
- 如果要让语音逐条影响 LMDrive，需把 `RouteAudioRuntime` 当前激活的 `voice_event` 转成传给 agent 的 `instruction`，而不只是 overlay 显示和日志记录。

### 7.2 推荐接入点

推荐优先从统一 benchmark 入口接入，而不是改 S11 场景逻辑本身：

```text
carla_eval/run_benchmark.py
    ↓ creates agent
LMDriveAgentAdapter(policy=official_lmdrive_policy)
    ↓ passed into
DongfengRouteScenario.run(...)
    ↓ passed into
ScenarioEvaluator.run(agent=agent, enable_cameras=True)
```

原因：

- S11 与其他场景都能复用同一个 agent 接口。
- `ScenarioEvaluator` 已经支持 `agent` 参数。
- `enable_cameras=True` 时会自动创建 `ObservationBuilder`，满足 LMDrive 多模态输入需要。

当前 `run_benchmark.py` 仍写死：

```python
agent = ScenarioControllerAgent()
```

正式接入 LMDrive 时应改成：

```python
from carla_eval.lmdrive import LMDriveAgentAdapter

policy = load_official_lmdrive_policy(...)
agent = LMDriveAgentAdapter(policy=policy)
agent.setup({"mode": "lmdrive"})
```

### 7.3 S11 到 LMDrive 的数据流

S11 接入 LMDrive 后，单 tick 的数据流应为：

```text
CARLA world tick
    ↓
ObservationBuilder
    - rgb_front: 1200 x 900
    - rgb_left / rgb_right / rgb_rear: 400 x 300
    - lidar: N x 4
    ↓
ScenarioEvaluator 构造 input_data
    - ego
    - actors
    - state
    - obs.speed_kmh
    - obs.route_metrics
    - obs.route_tracker
    - obs.rgb_*
    - obs.lidar
    ↓
LMDriveAgentAdapter.run_step(input_data, timestamp, instruction)
    ↓
LMDriveInputAdapter.build(...)
    - rgb_front / rgb_left / rgb_right / rgb_rear
    - rgb_center
    - lidar: 40000 x 4
    - velocity: [[m/s]]
    - measurements: [x, y, yaw_rad, speed_mps]
    - target_point: ego-frame route target
    - route_hint
    - text_input
    ↓
official LMDrive policy
    ↓
throttle / brake / steer
    ↓
CARLA VehicleControl
```

### 7.4 S11 语音输入如何接入

当前 S11 语音链路是离线模式：

```text
voice_results/**/*.wav + voice_results/**/*.json
    ↓
carla_eval/tools/match_route_audio.py
    ↓
configs/lmdrive/route_audio_matches.yaml
    ↓
RouteAudioRuntime
    ↓
FixedVoiceOverlay + frames.jsonl voice_* 字段
```

运行显示：

```bash
python carla_eval/run_carla_s11_basic_control_scene1.py --voice-overlay
```

当前 overlay 能显示当前触发语音，但 `ScenarioEvaluator` 调用 agent 时仍默认传：

```python
instruction=cfg.get("instructions", [{}])[0]
```

如果要让 LMDrive 真正根据每条语音变化行为，需要在 `ScenarioEvaluator` 中把当前 `voice_event` 转成 `active_instruction`：

```python
active_instruction = cfg.get("instructions", [{}])[0]
if voice_event is not None:
    voice = voice_event.get("voice", {})
    active_instruction = {
        "id": voice_event.get("audio_id"),
        "text_zh": voice.get("input_text") or voice.get("normalized_text"),
        "text_en": voice.get("instruction"),
        "intent": {
            "recognized_intents": voice.get("recognized_intents", []),
            "event_id": voice_event.get("event_id"),
        },
    }
```

然后把 agent 调用改为：

```python
throttle, brake, steer = agent.run_step(
    input_data,
    timestamp,
    instruction=active_instruction,
)
```

这样 `LMDriveInputAdapter` 生成的 `text_input` 就会从固定总任务指令变成当前触发语音，例如：

```text
向右转弯。
注意前方向左转弯。
向左变道。
保持当前车道加速到80公里每小时。
前方有人注意减速。
```

### 7.5 官方 LMDrive policy wrapper 需要做什么

当前仓库不会直接假设 LMDrive 官方 repo 的本地路径。建议新增一个本地 wrapper，例如：

```text
carla_eval/lmdrive/official_policy.py
```

职责：

- 把 LMDrive 官方仓库加入 `PYTHONPATH` 或通过配置传入路径。
- 加载官方 config/checkpoint。
- 暴露统一方法：

```python
class OfficialLMDrivePolicy:
    def __init__(self, repo_root, checkpoint, config):
        ...

    def run_step(self, lmdrive_input, timestamp):
        ...
        return {
            "throttle": throttle,
            "brake": brake,
            "steer": steer,
        }
```

`lmdrive_input` 就是 [LMDriveInputAdapter](/data/hdt_workspace/dongfeng/carla_eval/lmdrive/input_adapter.py:116) 生成的字段，不建议让官方 policy 直接读取本仓库的 `input_data`，否则接口会耦合过重。

### 7.6 最小代码改造清单

要让 S11 由 LMDrive 接管控制，最小改造是：

1. 新增 `OfficialLMDrivePolicy`，负责加载 LMDrive 官方模型。
2. 给 `run_benchmark.py` 增加 `--agent`、`--lmdrive-repo`、`--lmdrive-checkpoint`、`--lmdrive-config` 参数。
3. 当 `--agent lmdrive` 时创建：

```python
policy = OfficialLMDrivePolicy(...)
agent = LMDriveAgentAdapter(policy=policy)
```

4. 运行时保持 `enable_cameras=True`，不要加 `--no-cameras`。
5. 如需语音逐条驱动，按 7.4 把 `voice_event` 转成 `active_instruction` 后传入 agent。

推荐运行形式：

```bash
python carla_eval/run_benchmark.py \
  --route-id S11_basic_control_scene1_5km \
  --agent lmdrive \
  --lmdrive-repo /path/to/LMDrive \
  --lmdrive-config /path/to/lmdrive_config.py \
  --lmdrive-checkpoint /path/to/checkpoint.pth \
  --voice-overlay
```

当前这条命令是目标形态；仓库尚未实现 `--agent lmdrive` 参数和 `OfficialLMDrivePolicy`。

### 7.7 接入验证顺序

建议不要一开始直接跑完整 `5km`，按以下顺序验证：

1. 只运行 `--route-id S11_basic_control_scene1_5km --voice-overlay`，确认 route 和语音触发正常。
2. 用假 policy 接入 `LMDriveAgentAdapter`，确认 `lmdrive_input` 字段完整、shape 正确。
3. 接入官方 LMDrive policy，但先短时间运行，检查输出控制量范围。
4. 再跑完整 S11，检查：

- `collision = false`
- `route_completion >= 0.95`
- `red_light_violation_count = 0`
- `voice_text / voice_event_id` 是否按 route progress 触发
- `model_latency_ms / end_to_end_latency_ms` 是否被记录

## 8. 推荐实施顺序

建议按以下顺序接入：

1. 先保持当前场景与评测体系不变；
2. 新增 `LMDriveAgentAdapter`，先在 `S01` / `S02` 两个基础场景验证；
3. 确认文本指令能真实影响车辆行为；
4. 再扩到 `S04` / `S05` / `S07` / `S08`；
5. 最后再把当前离线语音匹配升级为实时语音链路 `麦克风/ASR -> Voice2LMDrive -> LMDrive policy`。

原因是：

- `S01` / `S02` 对模型要求最低；
- 更容易验证 instruction 是否起作用；
- 出问题时更容易判断是场景层、模型层还是控制层的问题。

## 9. 当前最准确的表述

当前关于 `LMDrive` 与本仓库关系的最准确表述应为：

> 本仓库已经完成东风三类 CARLA 场景、长路线组合场景与统一指标评测底座，并为 `LMDrive` 预留了 camera / LiDAR / speed / route hint / instruction 输入适配、agent 调用边界和语音触发日志字段。当前已支持离线语音与 route/action 匹配及固定窗口显示，但尚未正式接入 `LMDrive` 官方模型权重和实时 ASR 推理链路。

## 10. 相关文件

- [README.md](/data/hdt_workspace/dongfeng/README.md:1)
- [docs/pipeline/task0_lmdrive_pipeline_check.md](/data/hdt_workspace/dongfeng/docs/pipeline/task0_lmdrive_pipeline_check.md:1)
- [docs/pipeline/task1_benchmark_survey.md](/data/hdt_workspace/dongfeng/docs/pipeline/task1_benchmark_survey.md:1)
- [docs/scenario_design/benchmark_mapping.md](/data/hdt_workspace/dongfeng/docs/scenario_design/benchmark_mapping.md:1)
- [carla_eval/run_benchmark.py](/data/hdt_workspace/dongfeng/carla_eval/run_benchmark.py:1)
- [carla_eval/agents/base_agent.py](/data/hdt_workspace/dongfeng/carla_eval/agents/base_agent.py:1)
- [carla_eval/lmdrive/input_adapter.py](/data/hdt_workspace/dongfeng/carla_eval/lmdrive/input_adapter.py:1)
- [carla_eval/lmdrive/route_audio_runtime.py](/data/hdt_workspace/dongfeng/carla_eval/lmdrive/route_audio_runtime.py:1)
- [carla_eval/tools/match_route_audio.py](/data/hdt_workspace/dongfeng/carla_eval/tools/match_route_audio.py:1)
- [configs/metrics/metric_schema.yaml](/data/hdt_workspace/dongfeng/configs/metrics/metric_schema.yaml:1)
