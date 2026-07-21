# Standalone Bundle Workflow

长路线场景不应该只保留在共享 benchmark 文件里。后续每个场景都应当可以导出为单独 bundle，至少包含：

- 一个独立 `route XML`
- 一个独立 `scenario YAML`
- 一个独立 `annotation YAML/JSON`
- 一个独立 `voice matches YAML`
- 一个 `manifest.yaml`

## 输出位置

导出脚本默认写到：

`scenario_bundles/<scenario_id>/`

目录结构：

```text
scenario_bundles/<scenario_id>/
├── configs/
│   ├── <scenario_id>.yaml
│   ├── <scenario_id>.annotations.yaml
│   ├── route_audio_matches_<scenario_id>.yaml
│   └── manifest.yaml
└── routes/
    └── <scenario_id>.xml
```

其中导出的 `configs/<scenario_id>.yaml` 会被改写为直接引用 `routes/<scenario_id>.xml`。
如果源场景原本使用 `carla_lane_trace`，导出后的 YAML 会保留：

- `route.source_mode`
- `route.source_lane_trace`

这样既能直接作为独立包使用，也不会丢失原始生成参数。

## 导出命令

```bash
python carla_eval/tools/export_standalone_scenario_bundle.py \
  --scenario-id S11_basic_control_scene1_5km
```

如果下游必须吃 JSON annotations：

```bash
python carla_eval/tools/export_standalone_scenario_bundle.py \
  --scenario-id S11_basic_control_scene1_5km \
  --annotations-format json
```

## 对 lane-trace 场景的要求

`S11`、`S12` 这类场景当前仍以场景 YAML 里的运行时 route 生成逻辑为真源。导出脚本会把共享 benchmark 中注册的 route 条目拆成单独 XML，并生成完整 bundle，但这还不等于“可直接外发的最终路线”。

对外正式交付前，仍然应补齐：

- 与目标 CARLA 地图一致的稠密 route XML
- GlobalRoutePlanner 插值校验结果
- route 长度、关键转向、关键点验证结果

## 约定

后续新场景应保持：

- 共享 benchmark 文件继续用于统一跑批
- 每个场景都能通过导出脚本生成单独 bundle
- 如果场景使用 `carla_lane_trace`，在完成真实地图验证后补一份独立 route XML
