"""Entry point for S01 – Keep lane and reach target speed.

Usage:
    python -m carla_eval.run_carla_s01_keep_lane_speed [--host HOST] [--port PORT]
"""
import argparse
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from carla_eval.evaluator import ScenarioEvaluator, load_scenario_config
from carla_eval.scenarios_impl import KeepLaneSpeed


_DEFAULT_CFG = Path(__file__).parent.parent / "configs/scenarios/basic_control/S01_keep_lane_speed_60.yaml"


def _parse_args(argv=None):
    p = argparse.ArgumentParser(description="S01 – keep lane / target speed")
    p.add_argument("--config", default=str(_DEFAULT_CFG))
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--timeout", type=float, default=60.0)
    p.add_argument("--log-id", default=None)
    p.add_argument("--no-cameras", action="store_true")
    return p.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    cfg = load_scenario_config(args.config)
    scenario = KeepLaneSpeed()
    result = ScenarioEvaluator(scenario, cfg, args.config).run(
        host=args.host,
        port=args.port,
        timeout=args.timeout,
        log_id=args.log_id,
        enable_cameras=not args.no_cameras,
    )
    print(result)
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    sys.exit(main())
