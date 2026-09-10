"""Instantiate one official Leaderboard RouteScenario and inspect its actors."""

from __future__ import print_function

import argparse
import os

import carla

from leaderboard.scenarios.route_scenario import RouteScenario
from leaderboard.utils.route_parser import RouteParser
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument(
        "--routes", default="routes/dongfeng_leaderboard_2.0.xml"
    )
    parser.add_argument(
        "--route-id", default="S12_complex_obstacle_scene2_8km"
    )
    parser.add_argument("--traffic-manager-port", type=int, default=8000)
    parser.add_argument(
        "--initialize-all",
        action="store_true",
        help="Initialize every route scenario immediately for actor smoke testing.",
    )
    parser.add_argument(
        "--instantiate-pending",
        action="store_true",
        help="Instantiate pending route scenarios one by one for actor smoke testing.",
    )
    args = parser.parse_args()

    configs = RouteParser.parse_routes_file(args.routes)
    config = next(
        item for item in configs if item.name == "RouteScenario_" + args.route_id
    )

    client = carla.Client(args.host, args.port)
    client.set_timeout(120.0)
    world = client.load_world(config.town, reset_settings=False)
    settings = world.get_settings()
    # Smoke tests do not run a ScenarioManager tick loop. Async mode lets
    # BasicScenario finish its actor-registration waits while we inspect spawn.
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager(args.traffic_manager_port)
    traffic_manager.set_synchronous_mode(False)
    CarlaDataProvider.set_client(client)
    CarlaDataProvider.set_traffic_manager_port(args.traffic_manager_port)
    CarlaDataProvider.set_world(world)

    route_scenario = None
    try:
        print(
            "configured_scenarios:",
            [
                (item.name, item.type, len(item.other_actors))
                for item in config.scenario_configs
            ],
            flush=True,
        )
        if args.initialize_all:
            RouteScenario.INIT_THRESHOLD = 100000.0
        route_scenario = RouteScenario(world, config, debug_mode=1)
        print("route:", args.route_id)
        print("initialized_scenarios:", len(route_scenario.list_scenarios))
        for scenario in route_scenario.list_scenarios:
            print(
                "scenario:",
                scenario.__class__.__name__,
                "actors=",
                [
                    actor.type_id
                    for actor in scenario.other_actors
                    if actor is not None and actor.is_alive
                ],
            )
        if args.instantiate_pending:
            classes = route_scenario.get_all_scenario_classes()
            for pending in list(route_scenario.missing_scenario_configurations):
                pending.ego_vehicles = route_scenario.ego_vehicles
                pending.route = route_scenario.route
                scenario_class = classes[pending.type]
                instance = scenario_class(
                    world,
                    route_scenario.ego_vehicles,
                    pending,
                    debug_mode=1,
                    criteria_enable=False,
                )
                print(
                    "pending_initialized:",
                    instance.__class__.__name__,
                    "actors=",
                    [
                        actor.type_id
                        for actor in instance.other_actors
                        if actor is not None and actor.is_alive
                    ],
                    flush=True,
                )
                instance.remove_all_actors()
        print("pending_scenarios:", len(route_scenario.missing_scenario_configurations))
        print(
            "ego:",
            [
                actor.type_id
                for actor in route_scenario.ego_vehicles
                if actor is not None and actor.is_alive
            ],
        )
        return 0
    finally:
        if route_scenario is not None:
            route_scenario.remove_all_actors()
        CarlaDataProvider.cleanup()
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        traffic_manager.set_synchronous_mode(False)


if __name__ == "__main__":
    raise SystemExit(main())
