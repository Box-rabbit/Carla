# 控制侧接入交付说明

本文档是 S11、S12、S13 交给控制侧接入 SimLingo 的唯一入口说明。

## 1. 交付边界

场景侧已交付：

- CARLA 0.9.15 Python API 环境约定；
- 官方 Leaderboard 2.0 evaluator 路线格式；
- Town05 三条 Global Route；
- S12/S13 的官方 ScenarioRunner 场景 actor 和行为树；
- 按 Global Route 累计进度触发的场景门控；
- 天气、场景 actor、路线和场景 XML；
- 官方 Agent 到 SimLingo 的适配接口。

场景侧没有修改 SimLingo、ego 控制器、轨迹跟踪、转向/油门/制动算法或 Leaderboard 评分逻辑。

语音文件不是当前控制侧接入的前置条件。控制侧可以先关闭语音或使用占位语音验证路线和动态场景接入；语音补充属于后续数据完善。

## 2. 唯一交付入口

控制侧使用以下文件：

```text
routes/dongfeng_leaderboard_2.0.xml
leaderboard_agents/dongfeng_agent.py
configs/leaderboard/agent_config.yaml
scripts/env_carla0915.sh
scripts/run_official_leaderboard.sh
```

重新生成官方路线：

```bash
cd /data/hdt_workspace/dongfeng
source scripts/env_carla0915.sh
scripts/build_official_leaderboard_routes.sh
```

不要手工编辑生成后的官方 XML。

## 3. 三条路线和场景

| Route ID | 地图 | 长度 | 天气 | 官方场景 |
|---|---|---:|---|---:|
| `S11_basic_control_scene1_5km` | Town05 | 4.96 km | ClearNoon | 无自定义 actor |
| `S12_complex_obstacle_scene2_8km` | Town05 | 8.19 km | CloudySunset/低光照 | 4 个 |
| `S13_extreme_emergency_scene3_6km` | Town05 | 6.00 km | 雨夜、湿滑、低能见度 | 2 个 |

S12 场景进度：360m 行人横穿、1050m 慢车、1250m 环境交通流、1850m 公交站。

S13 场景进度：1220m 动态加塞、2520m 施工区域/施工人员/锥桶。

actor 在达到对应 Global Route 累计进度前不会放置到道路上，Town05 空间回环不会改变这些触发进度。

## 4. SimLingo Agent 接口

通过环境变量指定控制侧适配模块：

```bash
export DONGFENG_POLICY_MODULE=your_simlingo_adapter
```

模块必须提供：

```python
def build_agent(config_path: str):
    return SimLingoAdapter(config_path)
```

返回对象必须提供：

```python
def run_step(self, input_data, timestamp, global_plan_world_coord):
    ...

def destroy(self):
    ...
```

`run_step()` 返回 `carla.VehicleControl`，或返回：

```python
{"throttle": 0.0, "brake": 0.0, "steer": 0.0}
```

控制值范围分别为 throttle 0..1、brake 0..1、steer -1..1。

## 5. Agent 输入

传感器 ID 与数据：

```text
Center  1200x900 RGB 前视相机
Left    400x300 RGB 左前视相机
Right   400x300 RGB 右前视相机
GPS     GNSS
IMU     IMU
Speed   speedometer，20Hz
```

`input_data` 使用 Leaderboard 标准 `(frame_number, sensor_payload)` 结构。`global_plan_world_coord` 由 evaluator 提供，SimLingo 可用于导航，但不得依赖场景侧内部控制器。

## 6. 启动与验证

可视化启动 CARLA：

```bash
cd /data/hdt_workspace/CARLA_0.9.15
DISPLAY=:1 ./CarlaUE4.sh -quality-level=Low -carla-rpc-port=2000
```

启动单条路线：

```bash
cd /data/hdt_workspace/dongfeng
source scripts/env_carla0915.sh
export DONGFENG_POLICY_MODULE=your_simlingo_adapter
ROUTE_ID=S12_complex_obstacle_scene2_8km scripts/run_official_leaderboard.sh
```

启动三条路线：

```bash
ROUTES_SUBSET=S11_basic_control_scene1_5km,S12_complex_obstacle_scene2_8km,S13_extreme_emergency_scene3_6km \
  scripts/run_official_leaderboard.sh
```

场景侧静态验收：

```bash
scripts/validate_scene_delivery.sh
```

S12/S13 actor smoke test：

```bash
python carla_eval/tools/official_scenario_smoke_test.py --route-id S12_complex_obstacle_scene2_8km --instantiate-pending
python carla_eval/tools/official_scenario_smoke_test.py --route-id S13_extreme_emergency_scene3_6km --instantiate-pending
```

smoke test 只验证场景初始化、actor 创建和行为树构造，不代表 SimLingo 已完成全路线驾驶。

## 7. 验收边界

场景侧已验证官方 RouteParser、CARLA 0.9.15 API、S12/S13 actor 实例化、route-progress 触发树以及三条路线 XML 结构。

控制侧接收后继续验证 SimLingo 全路线驾驶、传感器兼容、车道保持、转弯、变道、停车、动态场景响应、Leaderboard 评分和语音闭环。

## 8. 不要使用的旧入口

以下文件是旧 runner 或兼容层，不是本次官方控制侧入口：

```text
carla_eval/run_carla_s11_basic_control_scene1.py
carla_eval/run_carla_s12_complex_obstacle_scene2.py
carla_eval/run_carla_s13_extreme_emergency_scene3.py
carla_eval/run_benchmark.py
routes/dongfeng_benchmark.xml
routes/dongfeng_lmdrive_benchmark.xml
```
