#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$ROOT_DIR"

SCENARIO=${1:?usage: generate_report.sh S11|S12|S13 [evaluate.py options]}
shift

case "$SCENARIO" in
  S11)
    CONFIG=configs/scenarios/basic_control/S11_basic_control_scene1_5km.yaml
    DEFAULT_FRAMES=logs/basic_control/S11_basic_control_scene1_5km/frames.jsonl
    DEFAULT_OUTPUT=reports/S11_basic_control_scene1_5km
    ;;
  S12)
    CONFIG=configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml
    DEFAULT_FRAMES=logs/complex_obstacle/S12_complex_obstacle_scene2_8km/frames.jsonl
    DEFAULT_OUTPUT=reports/S12_complex_obstacle_scene2_8km
    ;;
  S13)
    CONFIG=configs/scenarios/emergency_response/S13_extreme_emergency_scene3_6km.yaml
    DEFAULT_FRAMES=logs/emergency_response/S13_extreme_emergency_scene3_6km/frames.jsonl
    DEFAULT_OUTPUT=reports/S13_extreme_emergency_scene3_6km
    ;;
  *)
    echo "unknown scenario: $SCENARIO (expected S11, S12, or S13)" >&2
    exit 2
    ;;
esac

exec "${PYTHON:-python}" code/carla_eval/evaluate.py \
  --scenario_config "$CONFIG" \
  --frames "${FRAMES:-$DEFAULT_FRAMES}" \
  --output_dir "${OUTPUT_DIR:-$DEFAULT_OUTPUT}" \
  "$@"
