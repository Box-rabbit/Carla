"""Unified LMDrive-style benchmark runner for Dongfeng CARLA scenarios."""

import argparse
import json
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from carla_eval.agents import ScenarioControllerAgent
from carla_eval.benchmark import DongfengRouteScenario, ScenarioAnnotationStore
from carla_eval.utils.route_indexer import RouteIndexer


DEFAULT_ROUTES = Path("routes/dongfeng_benchmark.xml")
DEFAULT_SCENARIOS = Path("configs/scenario_annotations/dongfeng_benchmark.yaml")


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Run Dongfeng scenarios with an LMDrive-style route/scenario benchmark interface."
    )
    parser.add_argument("--routes", default=str(DEFAULT_ROUTES), help="Route XML file")
    parser.add_argument("--scenarios", default=str(DEFAULT_SCENARIOS), help="Scenario annotation YAML/JSON")
    parser.add_argument("--route-id", default=None, help="Run only one route id")
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--checkpoint", default="logs/benchmark/checkpoint.json")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--list", action="store_true", help="List matched routes without running CARLA")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--no-cameras", action="store_true")
    parser.add_argument("--draw-route", action="store_true")
    parser.add_argument("--draw-route-labels", action="store_true")
    parser.add_argument("--draw-route-stride", type=int, default=4)
    parser.add_argument("--draw-route-lifetime", type=float, default=900.0)
    parser.add_argument("--voice-overlay", action="store_true", help="Show matched voice command in a fixed screen window")
    parser.add_argument("--voice-match-config", default="configs/lmdrive/route_audio_matches.yaml")
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    routes_file = Path(args.routes)
    scenarios_file = Path(args.scenarios)
    checkpoint_path = Path(args.checkpoint)

    annotation_store = ScenarioAnnotationStore.from_file(scenarios_file)
    indexer = RouteIndexer(routes_file, scenarios_file, repetitions=args.repetitions)
    if args.resume:
        skipped = indexer.resume(checkpoint_path)
        print(f"[BENCHMARK] resumed from checkpoint, skipped={skipped}")

    results = []
    while indexer.peek():
        route_config = indexer.next()
        if args.route_id is not None and route_config.scenario_id != str(args.route_id):
            continue

        annotation = annotation_store.find(route_config.town, route_config.scenario_id)
        if annotation is None:
            raise RuntimeError(
                f"No scenario annotation for town={route_config.town}, route_id={route_config.scenario_id}"
            )

        route_scenario = DongfengRouteScenario(route_config, annotation, routes_file)
        print(
            "[BENCHMARK] "
            f"{indexer.current_index}/{indexer.total} "
            f"route_id={route_config.scenario_id} "
            f"town={route_config.town} "
            f"scenario={annotation.scenario_id} "
            f"type={annotation.scenario_type}"
        )

        if args.list:
            results.append({
                "route_id": route_config.scenario_id,
                "town": route_config.town,
                "scenario_id": annotation.scenario_id,
                "scenario_type": annotation.scenario_type,
                "config_path": str(annotation.config_path),
            })
            continue

        agent = ScenarioControllerAgent()
        agent.setup({"mode": "scenario_controller"})
        result = route_scenario.run(
            agent=agent,
            host=args.host,
            port=args.port,
            timeout=args.timeout,
            log_id=route_config.name,
            enable_cameras=not args.no_cameras,
            draw_route=args.draw_route,
            draw_route_stride=args.draw_route_stride,
            draw_route_lifetime=args.draw_route_lifetime,
            draw_route_labels=args.draw_route_labels,
            voice_overlay=args.voice_overlay,
            voice_match_config=args.voice_match_config,
        )
        results.append(result)
        indexer.save_state(checkpoint_path, extra_data={"last_result": result})

    if args.list:
        print(json.dumps(results, indent=2, ensure_ascii=False))
        return 0

    success_count = sum(1 for item in results if item.get("success"))
    print(
        "[BENCHMARK_DONE] "
        f"success={success_count}/{len(results)} "
        f"checkpoint={checkpoint_path}"
    )
    return 0 if success_count == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
