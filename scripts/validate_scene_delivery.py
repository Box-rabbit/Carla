#!/usr/bin/env python3
"""Validate the scene-side delivery contract without connecting to CARLA."""

from __future__ import annotations

import math
import py_compile
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROUTES = ROOT / "routes" / "dongfeng_leaderboard_2.0.xml"
FILES = (
    ROOT / "leaderboard_agents" / "dongfeng_agent.py",
    ROOT / "third_party/scenario_runner/srunner/scenarios/dongfeng_route_scenarios.py",
)
EXPECTED = {
    "S11_basic_control_scene1_5km": ((4750.0, 5250.0), 0, [], "clear"),
    "S12_complex_obstacle_scene2_8km": ((7700.0, 8500.0), 4, [360.0, 1050.0, 1250.0, 1850.0], "low"),
    "S13_extreme_emergency_scene3_6km": ((5700.0, 6300.0), 2, [1220.0, 2520.0], "rain"),
}


def route_length(route):
    points = route.findall("./waypoints/position")
    return sum(
        math.hypot(float(b.get("x")) - float(a.get("x")), float(b.get("y")) - float(a.get("y")))
        for a, b in zip(points, points[1:])
    )


def check_route(route):
    route_id = route.get("id")
    length_range, scenario_count, expected_progress, weather_kind = EXPECTED[route_id]
    errors = []
    length = route_length(route)
    if not length_range[0] <= length <= length_range[1]:
        errors.append(f"{route_id}: route length {length:.1f}m outside {length_range}")
    scenarios = route.findall("./scenarios/scenario")
    if len(scenarios) != scenario_count:
        errors.append(f"{route_id}: expected {scenario_count} scenarios, found {len(scenarios)}")
    progress = []
    for scenario in scenarios:
        node = scenario.find("route_progress")
        if node is None:
            errors.append(f"{route_id}/{scenario.get('name')}: missing route_progress")
            continue
        value = float(node.get("value"))
        progress.append(value)
        if value < 200.0:
            errors.append(f"{route_id}/{scenario.get('name')}: progress below 200m")
    if progress != expected_progress:
        errors.append(f"{route_id}: progress {progress} != {expected_progress}")
    weather = route.find("./weathers/weather")
    if weather is None:
        errors.append(f"{route_id}: missing weather")
    elif weather_kind == "clear" and (float(weather.get("cloudiness", 100)) > 10 or float(weather.get("precipitation", 100)) > 1):
        errors.append(f"{route_id}: weather is not clear daytime")
    elif weather_kind == "low" and float(weather.get("sun_altitude_angle", 90)) > 30:
        errors.append(f"{route_id}: weather is not low-light")
    elif weather_kind == "rain" and (float(weather.get("precipitation", 0)) <= 0 or float(weather.get("sun_altitude_angle", 0)) >= 0):
        errors.append(f"{route_id}: weather is not rainy night")
    return errors


def main():
    errors = []
    if not ROUTES.is_file():
        errors.append(f"missing route file: {ROUTES}")
    errors.extend(f"missing delivery file: {path}" for path in FILES if not path.is_file())
    if errors:
        for error in errors:
            print("ERROR:", error)
        return 1
    root = ET.parse(ROUTES).getroot()
    routes = {route.get("id"): route for route in root.findall("route")}
    errors.extend(f"missing expected route: {route_id}" for route_id in set(EXPECTED) - set(routes))
    errors.extend(f"unexpected route: {route_id}" for route_id in set(routes) - set(EXPECTED))
    for route_id in EXPECTED:
        if route_id in routes:
            errors.extend(check_route(routes[route_id]))
    for path in FILES:
        try:
            py_compile.compile(str(path), doraise=True)
        except py_compile.PyCompileError as exc:
            errors.append(str(exc))
    if errors:
        for error in errors:
            print("ERROR:", error)
        return 1
    print("scene delivery contract: OK")
    for route_id, route in routes.items():
        print(f"{route_id}: length={route_length(route):.1f}m scenarios={len(route.findall('./scenarios/scenario'))}")
    print("note: this check does not run SimLingo or a full route")
    return 0


if __name__ == "__main__":
    sys.exit(main())
