"""Add Dongfeng ScenarioRunner definitions to a Leaderboard 2.0 route file.

The source route remains the Global Route. Scenario positions are projected
from route distance and lateral offset, so no reference trajectory is added.
"""

from __future__ import annotations

import argparse
import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path

def _route_points(route):
    return [
        (float(node.get("x")), float(node.get("y")), float(node.get("z", 0.5)))
        for node in route.findall("./waypoints/position")
    ]


def _progress_table(points):
    distances = [0.0]
    for first, second in zip(points, points[1:]):
        distances.append(
            distances[-1]
            + math.hypot(second[0] - first[0], second[1] - first[1])
        )
    return distances


def _point_at(points, distances, progress, lateral=0.0):
    progress = max(0.0, min(float(progress), distances[-1]))
    index = 0
    while index + 1 < len(distances) and distances[index + 1] < progress:
        index += 1
    if index + 1 >= len(points):
        index = len(points) - 2
    span = max(distances[index + 1] - distances[index], 1e-6)
    ratio = (progress - distances[index]) / span
    x = points[index][0] + ratio * (points[index + 1][0] - points[index][0])
    y = points[index][1] + ratio * (points[index + 1][1] - points[index][1])
    z = points[index][2] + ratio * (points[index + 1][2] - points[index][2])
    dx = points[index + 1][0] - points[index][0]
    dy = points[index + 1][1] - points[index][1]
    norm = max(math.hypot(dx, dy), 1e-6)
    # CARLA right vector for a forward tangent (dx, dy).
    right_x, right_y = dy / norm, -dx / norm
    yaw = math.degrees(math.atan2(dy, dx))
    return x + lateral * right_x, y + lateral * right_y, z + 0.35, yaw


def _actor(parent, model, transform, rolename="scenario", speed=None):
    x, y, z, yaw = transform
    attrs = {
        "model": model,
        "x": f"{x:.3f}",
        "y": f"{y:.3f}",
        "z": f"{z:.3f}",
        "yaw": f"{yaw:.3f}",
        "rolename": rolename,
    }
    if speed is not None:
        attrs["speed"] = str(speed)
    ET.SubElement(parent, "other_actor", attrib=attrs)


def _scenario(
    parent,
    name,
    scenario_type,
    trigger,
    actors,
    parameters=None,
    route_progress=None,
):
    node = ET.SubElement(
        parent,
        "scenario",
        name=name,
        type=scenario_type,
    )
    x, y, z, yaw = trigger
    ET.SubElement(
        node,
        "trigger_point",
        x=f"{x:.3f}", y=f"{y:.3f}", z=f"{z:.3f}", yaw=f"{yaw:.3f}",
    )
    for actor_data in actors:
        _actor(node, *actor_data)
    for key, value in (parameters or {}).items():
        ET.SubElement(node, key, value=str(value))
    if route_progress is not None:
        ET.SubElement(node, "route_progress", value=f"{float(route_progress):.1f}")


def _s12(route, points, distances):
    scenarios = route.find("scenarios")
    # Re-running the exporter must be deterministic.
    scenarios.clear()

    pedestrian_trigger = _point_at(points, distances, 360.0)
    pedestrian_actors = [
        ("walker.pedestrian.0001", _point_at(points, distances, 360.0, 4.5), "pedestrian"),
        ("walker.pedestrian.0002", _point_at(points, distances, 365.0, 4.0), "pedestrian"),
    ]
    _scenario(
        scenarios, "S12_pedestrian_crossing_0360", "DongfengPedestrianCrossing",
        pedestrian_trigger, pedestrian_actors, route_progress=360.0,
    )

    slow_trigger = _point_at(points, distances, 1050.0)
    slow_actor = [("vehicle.audi.tt", _point_at(points, distances, 1070.0, -3.5), "slow_vehicle", "5.6")]
    _scenario(
        scenarios,
        "S12_slow_vehicle_overtake_1050",
        "DongfengSlowVehicle",
        slow_trigger,
        slow_actor,
        route_progress=1050.0,
    )

    ambient_trigger = _point_at(points, distances, 1250.0)
    ambient_actors = [
        ("vehicle.tesla.model3", _point_at(points, distances, 1278.0, -3.5), "ambient"),
        ("vehicle.lincoln.mkz_2020", _point_at(points, distances, 1295.0, 3.5), "ambient"),
    ]
    _scenario(
        scenarios,
        "S12_ambient_flow_1250",
        "DongfengAmbientFlow",
        ambient_trigger,
        ambient_actors,
        route_progress=1250.0,
    )

    bus_trigger = _point_at(points, distances, 1850.0)
    bus_actors = [
        ("vehicle.mitsubishi.fusorosa", _point_at(points, distances, 1850.0, 4.8), "bus_stop"),
        ("static.prop.busstop", _point_at(points, distances, 1850.0, 7.0), "bus_stop"),
        ("static.prop.bench01", _point_at(points, distances, 1854.0, 7.4), "bus_stop"),
        ("walker.pedestrian.0002", _point_at(points, distances, 1855.0, 6.4), "pedestrian"),
        ("walker.pedestrian.0003", _point_at(points, distances, 1856.0, 6.0), "pedestrian"),
    ]
    _scenario(
        scenarios,
        "S12_bus_stop_1850",
        "DongfengBusStop",
        bus_trigger,
        bus_actors,
        route_progress=1850.0,
    )


def _s13(route, points, distances):
    scenarios = route.find("scenarios")
    scenarios.clear()

    cutin_trigger = _point_at(points, distances, 1220.0)
    cutin_actor = [("vehicle.audi.etron", _point_at(points, distances, 1260.0, -3.5), "cut_in_vehicle")]
    _scenario(
        scenarios,
        "S13_cut_in_1220",
        "DongfengCutIn",
        cutin_trigger,
        cutin_actor,
        route_progress=1220.0,
    )

    construction_trigger = _point_at(points, distances, 2520.0)
    construction_actors = [
        ("walker.pedestrian.0004", _point_at(points, distances, 2545.0, 2.3), "construction_worker"),
    ]
    for index, progress in enumerate(range(2545, 2620, 10), start=1):
        construction_actors.append(
            ("static.prop.trafficcone01", _point_at(points, distances, progress, 0.2 + index * 0.2), "construction")
        )
    _scenario(
        scenarios, "S13_construction_zone_2520", "DongfengConstructionZone",
        construction_trigger, construction_actors, route_progress=2520.0,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="routes/dongfeng_leaderboard_2.0.xml")
    parser.add_argument("--output", default="routes/dongfeng_leaderboard_2.0.xml")
    args = parser.parse_args()

    root = ET.parse(args.input).getroot()
    for route in root.findall("route"):
        route_id = route.get("id")
        points = _route_points(route)
        if len(points) < 2:
            raise SystemExit(f"route has too few waypoints: {route_id}")
        distances = _progress_table(points)
        if route_id == "S12_complex_obstacle_scene2_8km":
            _s12(route, points, distances)
        elif route_id == "S13_extreme_emergency_scene3_6km":
            _s13(route, points, distances)
        elif route.find("scenarios") is None:
            ET.SubElement(route, "scenarios")

    try:
        ET.indent(root, space="  ")
    except AttributeError:
        pass
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        + ET.tostring(root, encoding="unicode")
        + "\n",
        encoding="utf-8",
    )
    print(f"added official scenarios to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
