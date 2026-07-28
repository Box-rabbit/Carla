# Legacy Short Scenarios Archive

This archive keeps the short Town03 prototype scenarios that were useful during
early development but are not required by the PDF contest delivery.

Archived scenario ids:

- `S01_keep_lane_speed_60`
- `S02_lane_change`
- `S04_pedestrian_slowdown`
- `S05_cone_detour`
- `S07_cut_in_brake`
- `S08_rain_night_danger_slowdown`

Contents:

- `carla_eval/`: original short-scenario runner scripts.
- `scenarios_impl/`: original short-scenario controller implementations.
- `configs/`: original scenario YAML files.
- `routes/`: original short route XML files.
- `reports/legacy_baselines/`: historical reports generated before archive.

These files are intentionally not imported by the active benchmark registry.
The active contest delivery is S11/S12/S13 only. If a short scenario is needed
again, restore its runner, implementation, config, route XML, annotation, and
registry entry together rather than mixing archived files with active configs.
