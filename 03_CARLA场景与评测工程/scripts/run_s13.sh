#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$ROOT_DIR"
exec "${PYTHON:-python}" code/carla_eval/run_carla_s13_extreme_emergency_scene3.py "$@"
