#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CARLA_ROOT="${CARLA_ROOT:-/data/hdt_workspace/CARLA_0.9.15}"
PYTHON="${CARLA_ENV:-/data/hdt_workspace/my_env/carla0915}/bin/python"
if [[ ! -x "${PYTHON}" ]]; then
  echo "CARLA_ENV Python not found: ${PYTHON}" >&2
  exit 2
fi
cd "${ROOT}"
export PYTHONPATH="${ROOT}:${ROOT}/third_party/leaderboard:${ROOT}/third_party/scenario_runner:${CARLA_ROOT}/PythonAPI/carla:${CARLA_ROOT}/PythonAPI/carla/agents:${PYTHONPATH:-}"
"${PYTHON}" scripts/validate_scene_delivery.py
"${PYTHON}" carla_eval/tools/validate_official_leaderboard.py --routes routes/dongfeng_leaderboard_2.0.xml
