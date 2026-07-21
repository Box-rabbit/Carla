"""Entry point for S12 – PDF scene 2 complex-obstacle 8km drive."""

import argparse
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from carla_eval.evaluator import ScenarioEvaluator, load_scenario_config
from carla_eval.scenarios_impl import ComplexObstacleScene2


_DEFAULT_CFG = (
    Path(__file__).parent.parent
    / "configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml"
)


def _parse_args(argv=None):
    p = argparse.ArgumentParser(description="S12 – PDF scene 2 complex-obstacle 8km drive")
    p.add_argument("--config", default=str(_DEFAULT_CFG))
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--timeout", type=float, default=60.0)
    p.add_argument("--log-id", default=None)
    p.add_argument("--no-cameras", action="store_true")
    p.add_argument("--draw-route", action="store_true")
    p.add_argument("--draw-route-labels", action="store_true")
    p.add_argument("--draw-route-stride", type=int, default=8)
    p.add_argument("--draw-route-lifetime", type=float, default=900.0)
    p.add_argument("--voice-overlay", action="store_true", help="Show matched voice command in a fixed screen window")
    p.add_argument("--voice-match-config", default="configs/lmdrive/route_audio_matches.yaml")
    return p.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    cfg = load_scenario_config(args.config)
    scenario = ComplexObstacleScene2()
    result = ScenarioEvaluator(scenario, cfg, args.config).run(
        host=args.host,
        port=args.port,
        timeout=args.timeout,
        log_id=args.log_id,
        enable_cameras=not args.no_cameras,
        draw_route=args.draw_route,
        draw_route_stride=args.draw_route_stride,
        draw_route_lifetime=args.draw_route_lifetime,
        draw_route_labels=args.draw_route_labels,
        voice_overlay=args.voice_overlay,
        voice_match_config=args.voice_match_config,
    )
    print(result)
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    sys.exit(main())
