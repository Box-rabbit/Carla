# 任务8：每帧日志记录脚本设计

## 目标
实现指标统计第一层：每帧日志记录。模块负责在每个 CARLA tick 中记录车辆状态、控制量、碰撞、违规、障碍物距离和各模块延时。

## 输入
CARLA world、ego vehicle、sensor 状态、agent control 输出、instruction / parsed intent、ASR/parser/model 时间戳、场景 YAML 配置。

## 输出
```text
logs/{scenario_id}/frames.jsonl
```

当前仓库默认输出只有 `frames.jsonl`。  
如需 `frames.csv`，可基于 [carla_eval/metrics/logger.py](/data/hdt_workspace/dongfeng/carla_eval/metrics/logger.py:17) 中的 `write_csv()` 做离线导出，但当前 runner 默认不直接写出 `frames.csv`。

## 核心接口
```python
logger = FrameLogger(output_dir)
logger.log_frame(frame_record)
logger.close()
```

## 关键要求
日志记录不能明显影响仿真 FPS；优先 JSONL 流式写入；每条日志保留 scenario_id 和 instruction_id；后续事件检测器只依赖日志文件，不直接依赖 CARLA world。
