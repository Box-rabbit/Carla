# CARLA 场景与评测工程

本目录是 S11、S12、S13 的独立交付副本，覆盖比赛方案中的三个连续驾驶场景：

- S11：基础操控，5 km
- S12：复杂避障，8 km
- S13：极限应急，6 km

运行环境为 CARLA 0.9.10.1，Python 环境需安装与 CARLA 服务端匹配的 CARLA Python API，以及本目录 `requirements.txt` 中的依赖。CARLA 服务端须在运行脚本前启动并监听默认 `localhost:2000`。

## 目录说明

- `code/`：场景实现、评测器、离线报告及辅助工具
- `configs/`：S11/S12/S13 场景真源、基准套件、指标和 LMDrive 路线改造配置
- `routes/`：运行路线、dense 路线和 LMDrive 适配路线
- `triggers/`：语音触发匹配和路线动作对齐
- `audio/`：按场景和录音日期整理的语音文件及识别结果
- `scripts/`：统一运行、验证和报告生成脚本
- `reports_example/`：保留的历史离线报告示例，不代表最终验收结果

## 快速开始

```bash
cd 03_CARLA场景与评测工程
python -m pip install -r requirements.txt

# CARLA 服务端已启动后运行单个场景
./scripts/run_s11.sh --draw-route
./scripts/run_s12.sh --draw-route
./scripts/run_s13.sh --draw-route

# 只检查场景注册和配置，不连接 CARLA
./scripts/validate.sh
./scripts/run_benchmark.sh --list
```

通过 `--host`、`--port`、`--no-cameras`、`--voice-overlay` 等参数可覆盖默认运行选项，参数会原样透传给 Python 入口。

## 语音与路线

语音只在已规划动作窗口内调节纵向速度。转向和变道由 `routes/` 中的 XML 路线与场景控制器执行。语音匹配结果在 `triggers/route_audio_matches.yaml`，动作对齐在 `triggers/route_action_alignment.yaml`。

如新增或替换音频文件，可重新生成匹配结果：

```bash
./scripts/match_audio.sh
```

## 离线报告

每次场景运行会在 `logs/` 下生成 `frames.jsonl`。使用下面命令生成对应离线报告：

```bash
./scripts/generate_report.sh S12
```

可通过环境变量 `FRAMES` 和 `OUTPUT_DIR` 覆盖输入日志和报告输出路径。
