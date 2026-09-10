#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CARLA_ENV="${CARLA_ENV:-/data/hdt_workspace/my_env/carla0915}"
export PYTHONPATH="${ROOT}:${ROOT}/third_party/leaderboard:${ROOT}/third_party/scenario_runner:${ROOT}/CARLA_0.9.15/PythonAPI/carla:${PYTHONPATH:-}"

cd "${ROOT}"
"${CARLA_ENV}/bin/python" carla_eval/tools/export_official_leaderboard_routes.py \
  --input routes/dongfeng_benchmark.xml \
  --output routes/dongfeng_leaderboard_2.0.xml
"${CARLA_ENV}/bin/python" carla_eval/tools/add_official_scenarios.py \
  --input routes/dongfeng_leaderboard_2.0.xml \
  --output routes/dongfeng_leaderboard_2.0.xml
"${CARLA_ENV}/bin/python" carla_eval/tools/validate_official_leaderboard.py \
  --routes routes/dongfeng_leaderboard_2.0.xml
