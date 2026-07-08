# Scenario Summary Table

| ID | Scenario | Category | Trigger Mechanism | Main Success Events | Collision | Key Metrics | Status |
|---|---|---|---|---|---|---|---|
| S01 | `S01_keep_lane_speed_60` | `basic_control` | target speed hold based evaluation | `speed_target_reached`, `task_success` | 0 | `success = true`; target speed = `60 km/h` | Completed |
| S02 | `S02_lane_change` | `basic_control` | lane-change execution and target-lane hold based evaluation | `lane_change_started`, `lane_change_completed`, `task_success` | 0 | `initial lane = 3`; `target lane = 2`; `success = true` | Completed |
| S04 | `S04_pedestrian_slowdown` | `complex_obstacle` | pedestrian distance based slowdown evaluation | `pedestrian_detected`, `slowdown_started`, `safe_slowdown_completed`, `task_success` | 0 | `min_distance_to_pedestrian ≈ 19.50 m`; `speed_drop_kmh ≈ 16.33`; `success = true` | Completed |
| S05 | `S05_cone_detour` | `complex_obstacle` | detection-based front cone observation and return-to-lane evaluation | `cone_detected`, `detour_started`, `detour_completed`, `return_to_lane_completed`, `task_success` | 0 | `min_distance_to_cone ≈ 3.06 m`; `max_lateral_offset_m ≈ 3.47 m`; `success = true` | Completed |
| S07 | `S07_cut_in_brake_realistic_urgent` | `emergency_response` | front-vehicle distance / TTC based emergency braking | `cut_in_detected`, `emergency_brake_started`, `safe_brake_completed`, `task_success` | 0 | `min_front_vehicle_distance ≈ 15.19 m`; `min_TTC ≈ 2.98 s`; `max_brake ≈ 0.80`; `success = true` | Completed |
| S08 | `S08_rain_night_danger_slowdown` | `emergency_response` | weather hazard-score based slowdown | `danger_detected`, `slowdown_started`, `safe_speed_reached`, `task_success` | 0 | `mean_speed ≈ 9.05 km/h`; `max_speed ≈ 12.22 km/h`; `mean_hazard_score ≈ 0.78`; `success = true` | Completed |
