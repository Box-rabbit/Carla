# Completed CARLA Closed-loop Scenario Overview

## 1. 项目当前完成情况概述

当前项目 `Dongfeng CARLA Scenario Evaluation` 已完成 6 个真实 CARLA 闭环评测场景，覆盖三类核心能力：

- `basic_control`
- `complex_obstacle`
- `emergency_response`

这些场景均采用闭环控制方式运行，ego 车辆在 CARLA 环境中根据当前观测结果执行控制，并通过对应的事件检测与指标统计机制完成评测。当前已完成场景包括：

- `S01_keep_lane_speed_60`
- `S02_lane_change`
- `S04_pedestrian_slowdown`
- `S05_cone_detour`
- `S07_cut_in_brake_realistic_urgent`
- `S08_rain_night_danger_slowdown`

## 2. 三大类别说明

### 2.1 basic_control

`basic_control` 主要用于验证基础驾驶操控能力，包括保持车道、速度控制、变道等。该类场景强调 ego 对明确驾驶指令的执行能力，以及在基础道路条件下的稳定闭环控制表现。

### 2.2 complex_obstacle

`complex_obstacle` 主要用于验证 ego 在存在外部障碍物时的感知、减速、绕行和回归原路径能力。该类场景强调障碍感知驱动的实时决策，而不是依赖预知终点或人工脚本直接判定成功。

### 2.3 emergency_response

`emergency_response` 主要用于验证 ego 在紧急或高风险环境中的安全响应能力，包括应急制动、雨夜低能见度危险环境降速等。该类场景强调基于当前帧风险状态进行响应，而不是使用 oracle 信息提前知道风险发生时刻。

## 3. 已完成场景详述

### 3.1 S01_keep_lane_speed_60

- Scenario ID: `S01_keep_lane_speed_60`
- Category: `basic_control`
- Goal: ego 保持当前车道，提速到 `60 km/h`，并在目标速度范围内保持指定时间。
- Config path: `configs/scenarios/basic_control/S01_keep_lane_speed_60.yaml`
- Run script: `carla_eval/run_carla_s01_keep_lane_speed.py`
- Logs path: `logs/basic_control/S01_keep_lane_speed_60/`
- Reports path: `reports/basic_control/S01_keep_lane_speed_60/`
- Trigger / control mechanism: 基于目标速度保持时长的评测机制，属于 `target speed hold based evaluation`。
- Key events: `speed_target_reached`, `task_success`
- Key metrics: `success = true`, `collision_count = 0`
- Result: 场景成功完成，无碰撞。

### 3.2 S02_lane_change

- Scenario ID: `S02_lane_change`
- Category: `basic_control`
- Goal: ego 根据指令完成向左变道，并在目标车道保持指定时间。
- Config path: `configs/scenarios/basic_control/S02_lane_change.yaml`
- Run script: `carla_eval/run_carla_s02_lane_change.py`
- Logs path: `logs/basic_control/S02_lane_change/`
- Reports path: `reports/basic_control/S02_lane_change/`
- Trigger / control mechanism: 基于变道执行过程与目标车道保持时长的评测机制，属于 `lane-change execution and target-lane hold based evaluation`。
- Key events: `lane_change_started`, `lane_change_completed`, `task_success`
- Key metrics: `initial lane = 3`, `target lane = 2`, `success = true`, `collision_count = 0`
- Result: 场景成功完成，ego 正确进入目标车道并稳定保持，无碰撞。

### 3.3 S04_pedestrian_slowdown

- Scenario ID: `S04_pedestrian_slowdown`
- Category: `complex_obstacle`
- Goal: ego 检测前方行人，并主动减速避让，保持安全距离，全程无碰撞。
- Config path: `configs/scenarios/complex_obstacle/S04_pedestrian_slowdown.yaml`
- Run script: `carla_eval/run_carla_s04_pedestrian_slowdown.py`
- Logs path: `logs/complex_obstacle/S04_pedestrian_slowdown/`
- Reports path: `reports/complex_obstacle/S04_pedestrian_slowdown/`
- Trigger / control mechanism: 基于行人与 ego 距离的减速评测机制，属于 `pedestrian distance based slowdown evaluation`。
- Key events: `pedestrian_detected`, `slowdown_started`, `safe_slowdown_completed`, `task_success`
- Key metrics: `min_distance_to_pedestrian ≈ 19.50 m`, `speed_drop_kmh ≈ 16.33`, `success = true`, `collision_count = 0`
- Result: 场景成功完成，ego 能够检测行人并完成安全减速，无碰撞。

### 3.4 S05_cone_detour

- Scenario ID: `S05_cone_detour`
- Category: `complex_obstacle`
- Goal: ego 检测前方原始行驶走廊内的施工锥桶，向左绕行，通过后回到原车道。
- Config path: `configs/scenarios/complex_obstacle/S05_cone_detour.yaml`
- Run script: `carla_eval/run_carla_s05_cone_detour.py`
- Logs path: `logs/complex_obstacle/S05_cone_detour/`
- Reports path: `reports/complex_obstacle/S05_cone_detour/`
- Trigger / control mechanism: 基于检测驱动的前向锥桶观察与绕行机制，属于 `detection-based front cone observation`。ego 每帧根据当前前方原始行驶走廊内是否检测到锥桶决定是否触发绕行，并在前方无锥桶持续一定时间后回归原车道。
- Key events: `cone_detected`, `detour_started`, `detour_completed`, `return_to_lane_completed`, `task_success`
- Key metrics: `min_distance_to_cone ≈ 3.06 m`, `max_lateral_offset_m ≈ 3.47 m`, `success = true`, `collision_count = 0`
- Result: 场景成功完成，ego 实现检测驱动的绕行和回归原车道，无碰撞。

### 3.5 S07_cut_in_brake_realistic_urgent

- Scenario ID: `S07_cut_in_brake_realistic_urgent`
- Category: `emergency_response`
- Goal: NPC 车辆从相邻车道切入 ego 前方并制动，ego 基于当前帧前车距离和 `TTC` 触发应急制动，保持安全距离且无碰撞。
- Config path: `configs/scenarios/emergency_response/S07_cut_in_brake.yaml`
- Run script: `carla_eval/run_carla_s07_cut_in_brake.py`
- Logs path: `logs/emergency_response/S07_cut_in_brake_realistic_urgent/`
- Reports path: `reports/emergency_response/S07_cut_in_brake_realistic_urgent/`
- Trigger / control mechanism: 基于前车距离与 `TTC` 的应急制动评测机制，属于 `front-vehicle distance / TTC based emergency braking`。
- Key events: `cut_in_detected`, `emergency_brake_started`, `safe_brake_completed`, `task_success`
- Key metrics: `min_front_vehicle_distance ≈ 15.19 m`, `min_front_gap ≈ 15.19 m`, `min_TTC ≈ 2.98 s`, `max_brake ≈ 0.80`, `success = true`, `collision_count = 0`
- Result: 场景成功完成，ego 在切入和制动风险下能够及时触发应急制动并保持安全距离，无碰撞。

Realistic urgent setting:

- `ego target speed = 40 km/h`
- `NPC initial longitudinal distance = 26 m`
- `NPC lateral offset = -3.5 m`
- `NPC cruise speed = 18 km/h`
- `cut-in start time = 2.0 s`
- `cut-in duration = 2.0 s`
- `brake delay after cut-in = 0.5 s`
- `brake strength = 0.75`

### 3.6 S08_rain_night_danger_slowdown

- Scenario ID: `S08_rain_night_danger_slowdown`
- Category: `emergency_response`
- Goal: ego 在雨夜低能见度危险环境下，根据天气风险 `hazard score` 识别危险环境并保持安全低速行驶。
- Config path: `configs/scenarios/emergency_response/S08_rain_night_danger_slowdown.yaml`
- Run script: `carla_eval/run_carla_s08_rain_night_danger_slowdown.py`
- Logs path: `logs/emergency_response/S08_rain_night_danger_slowdown/`
- Reports path: `reports/emergency_response/S08_rain_night_danger_slowdown/`
- Trigger / control mechanism: 基于天气 `hazard score` 的降速评测机制，属于 `weather hazard-score based slowdown`。
- Key events: `danger_detected`, `slowdown_started`, `safe_speed_reached`, `task_success`
- Key metrics: `mean_speed ≈ 9.05 km/h`, `max_speed ≈ 12.22 km/h`, `mean_hazard_score ≈ 0.78`, `safe_speed_hold_time = 2.0 s`, `travelled_distance ≈ 26.66 m`, `success = true`, `collision_count = 0`
- Result: 场景成功完成，ego 在雨夜危险环境下能够稳定降速并维持安全低速，无碰撞。

## 4. 关于 S05 / S07 / S08 的非 oracle 设计说明

### 4.1 S05_cone_detour

`S05_cone_detour` 不使用预先已知的最后一个锥桶位置，也不使用 oracle 终点判断。ego 的绕行决策完全基于当前帧前方原始行驶走廊内是否检测到锥桶。当前方无锥桶持续一段时间后，ego 才回归原车道，因此该场景体现的是检测驱动的真实闭环绕行逻辑。

### 4.2 S07_cut_in_brake_realistic_urgent

`S07_cut_in_brake_realistic_urgent` 中 ego 不使用 NPC 的脚本切入时间，也不使用 NPC 制动时间作为 oracle。ego 仅根据当前帧检测到的前方车辆距离与 `TTC` 触发制动，因此应急响应逻辑来自在线风险判断，而不是依赖脚本内部时序信息。

### 4.3 S08_rain_night_danger_slowdown

`S08_rain_night_danger_slowdown` 中 ego 不会突然把速度设为 `0`，也不依赖 scripted speed override。系统先根据当前天气参数计算 `hazard score`，再通过正常 CARLA `throttle/brake` 物理控制过程维持安全低速，因此该场景体现的是具有物理一致性的闭环减速控制。

## 5. Summary

当前项目已完成的 6 个闭环评测场景，已经覆盖：

- 基础操控 `basic_control`
- 复杂避障 `complex_obstacle`
- 应急响应 `emergency_response`

从能力维度看，这些场景已经覆盖了基础车道保持与速度控制、目标车道变换、行人避让、施工锥桶绕行、突发切入应急制动以及雨夜危险环境安全降速等关键任务。整体上，这 6 个场景构成了项目当前阶段较完整的 CARLA closed-loop scenario evaluation 基础能力集合。
