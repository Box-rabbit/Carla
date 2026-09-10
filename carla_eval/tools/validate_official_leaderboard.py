"""Validate the project route XML against the official Leaderboard parser."""

from __future__ import annotations

import argparse
from pathlib import Path

from leaderboard.utils.route_parser import RouteParser


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--routes", default="routes/dongfeng_leaderboard_2.0.xml"
    )
    args = parser.parse_args()

    route_path = Path(args.routes)
    configs = RouteParser.parse_routes_file(str(route_path))
    if not configs:
        raise SystemExit(f"no routes found in {route_path}")

    print(f"official parser: OK ({len(configs)} route(s))")
    for config in configs:
        scenario_count = len(config.scenario_configs)
        print(
            f"{config.name}: town={config.town} "
            f"keypoints={len(config.keypoints)} scenarios={scenario_count}"
        )
        if scenario_count == 0:
            print("  warning: route has no ScenarioRunner scenario definitions")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
