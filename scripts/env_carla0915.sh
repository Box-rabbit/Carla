#!/usr/bin/env bash
set -euo pipefail

export CARLA_ROOT=/data/hdt_workspace/CARLA_0.9.15
export CARLA_ENV=/data/hdt_workspace/my_env/carla0915
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export LEADERBOARD_ROOT="${PROJECT_ROOT}/third_party/leaderboard"
export SCENARIO_RUNNER_ROOT="${PROJECT_ROOT}/third_party/scenario_runner"
export PATH="$CARLA_ENV/bin:$PATH"
export PYTHONPATH="${PROJECT_ROOT}:${LEADERBOARD_ROOT}:${SCENARIO_RUNNER_ROOT}:${CARLA_ROOT}/PythonAPI/carla:${CARLA_ROOT}/PythonAPI/carla/agents:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$CARLA_ROOT/Engine/Binaries/ThirdParty/PhysX3/Linux/x86_64-unknown-linux-gnu:${LD_LIBRARY_PATH:-}"

echo "CARLA_ROOT=$CARLA_ROOT"
echo "LEADERBOARD_ROOT=$LEADERBOARD_ROOT"
echo "SCENARIO_RUNNER_ROOT=$SCENARIO_RUNNER_ROOT"
echo "python=$(command -v python)"
python - <<'PY'
import carla
print("carla_api=" + carla.__file__)
PY
