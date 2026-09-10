#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export LEADERBOARD_ROOT="${ROOT}/third_party/leaderboard"
export SCENARIO_RUNNER_ROOT="${ROOT}/third_party/scenario_runner"
export CARLA_ROOT="${CARLA_ROOT:-/data/hdt_workspace/CARLA_0.9.15}"
export CARLA_ENV="${CARLA_ENV:-/data/hdt_workspace/my_env/carla0915}"
export PYTHONPATH="${ROOT}:${LEADERBOARD_ROOT}:${SCENARIO_RUNNER_ROOT}:${CARLA_ROOT}/PythonAPI/carla:${CARLA_ROOT}/PythonAPI/carla/agents:${PYTHONPATH:-}"
export PATH="${CARLA_ENV}/bin:${PATH}"

ROUTES="${ROUTES:-${ROOT}/routes/dongfeng_leaderboard_2.0.xml}"
AGENT="${AGENT:-${ROOT}/leaderboard_agents/dongfeng_agent.py}"
AGENT_CONFIG="${AGENT_CONFIG:-${ROOT}/configs/leaderboard/agent_config.yaml}"
CHECKPOINT="${CHECKPOINT:-${ROOT}/logs/leaderboard_2.0/results.json}"
DEBUG_CHECKPOINT="${DEBUG_CHECKPOINT:-${ROOT}/logs/leaderboard_2.0/live_results.txt}"

if [[ ! -x "${CARLA_ENV}/bin/python" ]]; then
  echo "CARLA_ENV Python not found: ${CARLA_ENV}/bin/python" >&2
  exit 2
fi
if [[ ! -f "${ROUTES}" ]]; then
  echo "Route XML not found. Run the route exporter first." >&2
  exit 2
fi

if [[ "${AUTO_BUILD_ROUTES:-0}" == "1" && "${ROUTES}" == "${ROOT}/routes/dongfeng_leaderboard_2.0.xml" ]]; then
  "${ROOT}/scripts/build_official_leaderboard_routes.sh"
fi

mkdir -p "$(dirname "${CHECKPOINT}")"

if [[ -n "${ROUTE_ID:-}" ]]; then
  ROUTES="${ROOT}/logs/leaderboard_2.0/routes/${ROUTE_ID}.xml"
  "${CARLA_ENV}/bin/python" \
    "${ROOT}/carla_eval/tools/select_official_leaderboard_route.py" \
    --input="${ROOT}/routes/dongfeng_leaderboard_2.0.xml" \
    --route-id="${ROUTE_ID}" \
    --output="${ROUTES}"
fi

if [[ ! -f "${ROUTES}" ]]; then
  echo "Route XML not found: ${ROUTES}" >&2
  exit 2
fi

args=(
  --host="${CARLA_HOST:-127.0.0.1}"
  --port="${CARLA_PORT:-2000}"
  --traffic-manager-port="${TRAFFIC_MANAGER_PORT:-8000}"
  --routes="${ROUTES}"
  --repetitions="${REPETITIONS:-1}"
  --track="${CHALLENGE_TRACK_CODENAME:-SENSORS}"
  --checkpoint="${CHECKPOINT}"
  --debug-checkpoint="${DEBUG_CHECKPOINT}"
  --agent="${AGENT}"
  --agent-config="${AGENT_CONFIG}"
  --debug="${DEBUG_CHALLENGE:-1}"
)
if [[ -n "${ROUTES_SUBSET:-}" ]]; then
  args+=(--routes-subset="${ROUTES_SUBSET}")
fi
if [[ -n "${RECORD_PATH:-}" ]]; then
  args+=(--record="${RECORD_PATH}")
fi

exec "${CARLA_ENV}/bin/python" \
  -u "${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator.py" "${args[@]}" "$@"
