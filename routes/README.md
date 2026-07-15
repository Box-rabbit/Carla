# Routes Guide

`routes/` 是当前仓库的 route XML 规范位置。

使用约定：

- 场景配置中的 `route.route_file` 应优先引用这里的路径。
- 新增 route 时按场景类别放到对应子目录。
- `routes/dongfeng_benchmark.xml` 是 LMDrive-style 统一 benchmark 入口使用的总 route 文件。
- S11 长路线当前不维护单独长 XML，实际路线由对应场景 YAML 的 `route.mode: carla_lane_trace` 生成；这里只注册统一 benchmark route id。
