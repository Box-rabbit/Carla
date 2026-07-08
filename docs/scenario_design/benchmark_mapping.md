# Benchmark Mapping for Dongfeng Scenario Design

## 1. 目的

本文档用于说明 `Dongfeng CARLA Scenario Evaluation` 当前场景库与评测体系如何参考现有 CARLA benchmark 和开源场景工具构建，而不是完全从零设计。

当前项目在场景与评测设计上采用以下组合思路：

```text
LMDrive / LangAuto 作为语言引导闭环驾驶主线参考
+ CARLA Leaderboard 作为路线与基础安全评测参考
+ ScenarioRunner 作为危险场景触发与 actor 编排参考
+ Bench2Drive 作为短路线、单能力、多场景评测组织参考
```

## 2. 参考基准总览

| 参考来源 | 主要借鉴内容 | 在本项目中的落地方式 |
|---|---|---|
| `CARLA Leaderboard` | route XML、route completion、collision、lane invasion、red light、route deviation、timeout、blocked 等通用评测指标 | 统一 route 配置、基础路线跟踪指标、违规/失败项统计 |
| `ScenarioRunner` | actor 编排、触发条件、pass/fail criteria、危险场景状态机 | 行人横穿、锥桶施工、突发加塞、前车急刹等触发式场景设计 |
| `Bench2Drive` | 短路线、单能力任务、多场景组合、任务级统计方式 | 第一版优先做可控短路线、固定起终点、单场景单能力验证 |
| `LMDrive / LangAuto` | language-guided closed-loop driving、自然语言指令驱动车辆行为 | 将文本/语音指令映射到场景任务，作为闭环驾驶上层接口 |

## 3. 东风三类场景与 Benchmark 总体映射

| 东风类别 | 东风要求关键词 | 主要参考 benchmark | 借鉴重点 | 第一版落地策略 |
|---|---|---|---|---|
| `basic_control` | 启动/停止、加速/减速、左转/右转、变道 | `LMDrive / LangAuto`、`CARLA Leaderboard` | 路线跟踪、基础动作执行、基础安全指标 | 先做直道速度控制、变道、路口转向、停车 |
| `complex_obstacle` | 行人、公交站、慢车、锥桶、施工绕行、变道超车 | `ScenarioRunner`、`Bench2Drive`、`CARLA Leaderboard` | 动态/静态障碍触发、组合任务、避障后回归车道 | 先做行人减速、锥桶绕行、慢车超越 |
| `emergency_response` | 雨夜、低光照、突发加塞、危险路况、紧急减速停车 | `ScenarioRunner`、`Bench2Drive`、`CARLA Leaderboard` | TTC 风险触发、极端环境、应急制动、低速安全保持 | 先做 cut-in brake、雨夜危险降速、施工并道/停车 |

## 4. 场景级映射表

### 4.1 基础操控 `basic_control`

| 本项目场景 | 东风要求对应 | 主要 benchmark 参考 | 借鉴点 | 当前状态 |
|---|---|---|---|---|
| `S01_keep_lane_speed_60` | 保持当前车道、提速/稳速 | `CARLA Leaderboard`、`LMDrive / LangAuto` | route following、target speed hold、route completion、lane keeping | 已完成 |
| `S02_lane_change` | 向左/向右变道 | `CARLA Leaderboard`、`Bench2Drive`、`LMDrive / LangAuto` | lane change、目标车道保持、变道后稳定前进 | 已完成 |
| `S03_intersection_turn` | 路口左/右转 | `CARLA Leaderboard`、`ScenarioRunner` | intersection turn、红灯违规检测、目标道路进入判定 | 规划中 |
| `B05_stop_at_target` | 减速停车/目标区域停车 | `CARLA Leaderboard`、`Bench2Drive` | stop success、速度接近零、目标区停车判定 | 规划中 |

### 4.2 复杂避障 `complex_obstacle`

| 本项目场景 | 东风要求对应 | 主要 benchmark 参考 | 借鉴点 | 当前状态 |
|---|---|---|---|---|
| `S04_pedestrian_slowdown` | 前方行人减速避让 | `ScenarioRunner`、`CARLA Leaderboard` | pedestrian crossing、距离触发、最小安全距离统计 | 已完成 |
| `S05_cone_detour` | 施工锥桶绕行并回原车道 | `ScenarioRunner`、`Bench2Drive` | static obstacle in lane、detour、return-to-lane、子任务状态机 | 已完成 |
| `S06_slow_vehicle_overtake` | 慢车避让/变道超车 | `ScenarioRunner`、`Bench2Drive`、`CARLA Leaderboard` | leading vehicle、safe overtake、相邻车道安全检查 | 规划中 |
| `C04_bus_stop_caution` | 公交站区域减速避让 | `ScenarioRunner`、`Bench2Drive` | 区域减速、低速保持、行人/公交站复合风险 | 规划中 |

### 4.3 应急响应 `emergency_response`

| 本项目场景 | 东风要求对应 | 主要 benchmark 参考 | 借鉴点 | 当前状态 |
|---|---|---|---|---|
| `S07_cut_in_brake` | 突发加塞、紧急制动/避让 | `ScenarioRunner`、`Bench2Drive`、`CARLA Leaderboard` | cut-in、TTC 风险触发、紧急制动延时、最小车距 | 已完成 |
| `S08_rain_night_danger_slowdown` | 雨夜低光照危险路况降速 | `CARLA Leaderboard`、`Bench2Drive` | 极端天气、危险环境低速保持、低能见度鲁棒性 | 已完成 |
| `S09_construction_merge_stop` | 施工路段减速并道/停车 | `ScenarioRunner`、`Bench2Drive` | blocked lane、merge、施工区前动作完成判定 | 规划中 |
| `E04_leading_vehicle_brake` | 前车急刹 | `ScenarioRunner`、`CARLA Leaderboard` | leading vehicle sudden brake、追尾风险控制 | 规划中 |

## 5. 指标映射表

| 指标类别 | 本项目指标 | 主要参考来源 | 说明 |
|---|---|---|---|
| 任务完成 | `task_success`、`task_completion_rate` | `Bench2Drive`、`ScenarioRunner` | 用于场景级 pass/fail 与多场景汇总统计 |
| 路线执行 | `route_completion`、`route_deviation` | `CARLA Leaderboard` | 用于真实路线跟踪与偏航/偏离检测 |
| 安全 | `collision_count`、`min_distance_to_obstacle`、`unsafe_close_distance` | `CARLA Leaderboard`、`ScenarioRunner` | 碰撞和危险接近作为核心失败项 |
| 违规 | `lane_invasion_count`、`red_light_violation_count` | `CARLA Leaderboard` | 作为基础赛道“无违规”要求的直接对应项 |
| 行为完成 | `lane_change_completed`、`turn_completed`、`return_to_lane_completed`、`stop_completed` | `ScenarioRunner`、`Bench2Drive` | 面向动作型任务的显式子目标完成判定 |
| 速度控制 | `target_speed_error`、`mean_speed`、`speed_drop_kmh`、`safe_speed_hold_time` | `Leaderboard`、项目自定义 | 用于稳速、降速、限速、安全低速场景 |
| 响应时延 | `response_latency_ms`、`decision_latency_ms`、`end_to_end_latency_ms` | 赛题要求 + 项目自定义 | 对应东风赛题中的延时约束 |
| 指令执行完整性 | `subtask_missing_rate`、`instruction_following_success` | `LMDrive / LangAuto`、项目自定义 | 面向组合指令拆解与动作遗漏检测 |
| 紧急风险 | `min_ttc`、`brake_reaction_success`、`emergency_response_latency` | `ScenarioRunner`、项目自定义 | 面向 cut-in、急刹等应急响应场景 |

## 6. 场景设计原则与 Benchmark 一致性

当前项目采用的第一版设计原则与现有 benchmark 的一致点如下：

| 设计原则 | 对应 benchmark 思想 | 本项目实施方式 |
|---|---|---|
| 优先可控、可重复、稳定出结果 | `ScenarioRunner` 固定 actor 编排；`Bench2Drive` 单能力场景 | 固定随机种子、固定 spawn 点、固定 route、固定触发条件 |
| 先短路线再逐步扩展复杂度 | `Bench2Drive` 短路线任务化评测 | 第一版优先短路线、低变量、强可复现 |
| 明确成功/失败判定 | `ScenarioRunner` pass/fail criteria | 每个场景定义 goal、触发、关键事件、成功判定、失败项 |
| 使用统一基础安全指标 | `CARLA Leaderboard` | route completion、collision、lane invasion、red light、route deviation |
| 组合任务拆解成子动作 | `LMDrive / LangAuto` 的语言驱动任务接口 + 项目状态机 | 将复杂指令拆成减速、变道、绕行、回正、停车等子任务 |

## 7. 当前项目与赛题要求的覆盖关系

| 东风赛题要求 | 本项目当前对应能力 | 备注 |
|---|---|---|
| 基础操控场景 | `S01`、`S02`，以及规划中的 `S03/B05` | 已有基础速度控制与变道，后续补路口转向和停车更完整 |
| 复杂避障场景 | `S04`、`S05`，以及规划中的 `S06/C04` | 已覆盖行人与锥桶绕行，后续补慢车和公交站 |
| 应急响应场景 | `S07`、`S08`，以及规划中的 `S09/E04` | 已覆盖 cut-in 与雨夜危险降速，后续补前车急刹和施工并道 |
| 可复现测试场景 | 已支持配置唯一真源、固定 route、固定 actor spawn | 符合第一版稳定复现目标 |
| 指标统计脚本 | 已有 runtime metrics、report generator、frame logger | 后续继续补充多模态与语音链路延时指标 |

## 8. 结论

当前项目的场景与指标体系并不是完全从零设计，而是有意识地参考了：

- `CARLA Leaderboard` 的路线跟踪与安全评测思路
- `ScenarioRunner` 的触发机制、actor 编排与 pass/fail 设计
- `Bench2Drive` 的短路线、多场景、单能力逐步扩展组织方式
- `LMDrive / LangAuto` 的语言驱动闭环驾驶接口思路

在此基础上，项目将东风赛题要求的三类能力映射为一组可复现、可度量、可扩展的 CARLA 闭环评测场景。当前已完成的 `S01`、`S02`、`S04`、`S05`、`S07`、`S08` 已构成第一版场景样板，后续新增场景应继续沿用本映射表中的参考逻辑与统一指标体系。
