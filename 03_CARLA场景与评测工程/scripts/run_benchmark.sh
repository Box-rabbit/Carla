#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$ROOT_DIR"
exec "${PYTHON:-python}" code/carla_eval/run_benchmark.py "$@"
