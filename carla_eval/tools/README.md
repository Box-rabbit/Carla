# CARLA Eval Tools

`tools/` contains command-line utilities. They are not imported by the
scenario runtime and should be run explicitly from the repository root.

## Route And Bundle Delivery

- `export_validated_dense_route.py`: export and validate a dense CARLA route.
- `build_lmdrive_adapted_routes.py`: generate sparse LMDrive/Leaderboard
  delivery routes from `configs/lmdrive/route_adaptations.yaml`.
- `export_standalone_scenario_bundle.py`: export one route, scenario config,
  annotation, voice matches, and manifest into `scenario_bundles/`.
- `match_route_audio.py`: regenerate
  `configs/lmdrive/route_audio_matches.yaml` from `data/audio/`.

## Route Discovery

- `find_basic_control_route.py`: search Town maps for a basic-control route.
- `find_left_lane_spawn_points.py`: find spawn points with a usable left lane.
- `find_straight_spawn_points.py`: find long straight-road spawn candidates.

These discovery tools support scenario design only. Their output must be
validated and then committed as route XML or scenario configuration; they are
not runtime dependencies.

## CARLA Operations

- `carla_smoke_test.py`: verify CARLA connection and inspect the loaded map.
- `cleanup_carla.py`: destroy actors remaining in the connected CARLA world.

Run operational tools only against the intended CARLA server and map.
