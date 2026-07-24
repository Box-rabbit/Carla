# Routes Guide

`routes/` 是当前仓库的 route XML 规范位置。

使用约定：

- 场景配置中的 `route.route_file` 应优先引用这里的路径。
- 新增 route 时按场景类别放到对应子目录。
- `routes/dongfeng_benchmark.xml` 是 LMDrive-style 统一 benchmark 入口使用的总 route 文件。
- `routes/dongfeng_lmdrive_benchmark.xml` 是 S12/S13 的 LMDrive/Leaderboard 适配总 route 文件。
- S11/S12/S13 均维护 dense route XML；S12/S13 另维护 `_lmdrive.xml` 稀疏适配路线。
