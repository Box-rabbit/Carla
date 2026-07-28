#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$ROOT_DIR"

"${PYTHON:-python}" code/carla_eval/validate_config.py \
  --config configs/scenarios/basic_control/S11_basic_control_scene1_5km.yaml
"${PYTHON:-python}" code/carla_eval/validate_config.py \
  --config configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml
"${PYTHON:-python}" code/carla_eval/validate_config.py \
  --config configs/scenarios/emergency_response/S13_extreme_emergency_scene3_6km.yaml
"${PYTHON:-python}" code/carla_eval/run_benchmark.py --list
