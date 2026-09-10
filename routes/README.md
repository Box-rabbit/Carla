# Routes Guide

`routes/` 是当前仓库的 route XML 规范位置。

使用约定：

- 场景配置中的 `route.route_file` 应优先引用这里的路径。
- 新增 route 时按场景类别放到对应子目录。
- `routes/dongfeng_benchmark.xml` 是场景 route/scenario 注册总表，不替代长场景的运行路线。
- `routes/dongfeng_simlingo_benchmark.xml` 是 S12/S13 的 SimLingo/Leaderboard 适配总 route 文件，仅在显式选择 benchmark 路线覆盖时使用。
- S11/S12/S13 均维护 dense route XML；S12/S13 另维护 `_simlingo.xml` 稀疏适配路线。

控制侧接入 SimLingo 时，唯一使用的官方交付路线是：

```text
routes/dongfeng_leaderboard_2.0.xml
```

该文件由 `scripts/build_official_leaderboard_routes.sh` 生成，包含 Leaderboard 2.0 路线结构以及 S12/S13 的官方 ScenarioRunner 场景节点。`dongfeng_benchmark.xml` 和 `_simlingo.xml` 保留作兼容、设计或离线评测用途，不作为官方 evaluator 的默认输入。

默认运行时，`carla_eval/run_benchmark.py` 保持场景 YAML 中声明的
`route.route_file`。需要验证统一 benchmark XML 时，显式使用：

```bash
python carla_eval/run_benchmark.py \
  --suite pdf_delivery \
  --routes routes/dongfeng_simlingo_benchmark.xml \
  --route-source benchmark \
  --route-id S12_complex_obstacle_scene2_8km
```
