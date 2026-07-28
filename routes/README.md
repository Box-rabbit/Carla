# Routes Guide

`routes/` 是当前仓库的 route XML 规范位置。

使用约定：

- 场景配置中的 `route.route_file` 应优先引用这里的路径。
- 新增 route 时按场景类别放到对应子目录。
- `routes/dongfeng_benchmark.xml` 是 route/scenario 注册总表，不替代长场景的运行路线。
- `routes/dongfeng_lmdrive_benchmark.xml` 是 S12/S13 的 LMDrive/Leaderboard 适配总 route 文件，仅在显式选择 benchmark 路线覆盖时使用。
- S11/S12/S13 均维护 dense route XML；S12/S13 另维护 `_lmdrive.xml` 稀疏适配路线。

默认运行时，`carla_eval/run_benchmark.py` 保持场景 YAML 中声明的
`route.route_file`。需要验证统一 benchmark XML 时，显式使用：

```bash
python carla_eval/run_benchmark.py \
  --suite pdf_delivery \
  --routes routes/dongfeng_lmdrive_benchmark.xml \
  --route-source benchmark \
  --route-id S12_complex_obstacle_scene2_8km
```
