# 任务10：最终指标统计与报告生成脚本设计

## 目标
实现指标统计第三层：读取每帧日志和事件检测结果，输出每个场景的评测结果、分组统计、CSV/JSON 文件和基础可视化图表。

## 输入输出
输入：frames.jsonl、events.json、场景 YAML。  
当前仓库默认输出：evaluation_report.json、evaluation_report.csv。  
其中 `events.json` 由离线评测入口单独保存。速度曲线、延时曲线、事件时间轴、summary_report、summary_table 仍可作为后续增强项，但当前实现默认不生成。

## 单场景报告字段
```json
{
  "scenario_id": "S01_keep_lane_speed",
  "category": "basic_control",
  "success": true,
  "task_completion_rate": 1.0,
  "route_completion": 0.98,
  "collision_count": 0,
  "violation_count": 0,
  "mean_response_latency_ms": 108.3,
  "mean_end_to_end_latency_ms": 136.2,
  "target_speed_error_kmh": 2.8,
  "subtask_missing_rate": 0.0,
  "failure_reason": null
}
```

## 分组统计
按 basic_control、complex_obstacle、emergency_response 汇总场景数、成功数、任务完成率、平均碰撞次数、平均违规次数、平均响应延时、平均端到端延时、平均子任务遗漏率。

## 可视化
当前实现未默认生成图表文件；如后续需要答辩展示材料，可在报告层继续补速度曲线、延时曲线、事件时间轴和三类场景汇总图。
