"""Convert Dongfeng route XML files to official Leaderboard 2.0 route XML.

The project-owned route XML deliberately stays in its existing flat format.
Leaderboard 2.0 expects:

    <route>
      <weathers>...</weathers>
      <waypoints><position ... /></waypoints>
      <scenarios />
    </route>

This exporter performs only a schema conversion; it does not alter route
coordinates or add a reference trajectory.
"""

from __future__ import annotations

import argparse
import copy
import xml.etree.ElementTree as ET
from pathlib import Path


ROUTE_IDS = (
    "S11_basic_control_scene1_5km",
    "S12_complex_obstacle_scene2_8km",
    "S13_extreme_emergency_scene3_6km",
)


def _indent(root: ET.Element) -> None:
    try:
        ET.indent(root, space="  ")
    except AttributeError:
        pass


def _weather(route: ET.Element) -> ET.Element:
    source = route.find("weather")
    output = ET.Element("weathers")
    weather_attrs = dict(source.attrib) if source is not None else {
        "cloudiness": "0",
        "precipitation": "0",
        "sun_altitude_angle": "70",
    }
    weather_attrs["route_percentage"] = "0"
    ET.SubElement(output, "weather", attrib=weather_attrs)
    return output


def convert_route(route: ET.Element) -> ET.Element:
    output = ET.Element(
        "route",
        id=str(route.get("id")),
        town=str(route.get("town")),
        category=str(route.get("category", "")),
    )
    output.append(_weather(route))
    waypoints = ET.SubElement(output, "waypoints")
    for waypoint in route.findall("waypoint"):
        attrs = {
            "x": str(waypoint.get("x", "0")),
            "y": str(waypoint.get("y", "0")),
            "z": str(waypoint.get("z", "0.5")),
            "yaw": str(waypoint.get("yaw", "0.0")),
        }
        ET.SubElement(waypoints, "position", attrib=attrs)
    # The current Dongfeng scenario actors are managed by the scene-side
    # runner. Keep the official route structurally valid without inventing
    # ScenarioRunner scenario types that do not exist upstream.
    ET.SubElement(output, "scenarios")
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="routes/dongfeng_benchmark.xml")
    parser.add_argument(
        "--output",
        default="routes/dongfeng_leaderboard_2.0.xml",
    )
    parser.add_argument("--route-id", action="append", dest="route_ids")
    args = parser.parse_args()

    selected = set(args.route_ids or ROUTE_IDS)
    source_root = ET.parse(args.input).getroot()
    output_root = ET.Element("routes")
    count = 0
    for route in source_root.findall("route"):
        if route.get("id") not in selected:
            continue
        output_root.append(convert_route(copy.deepcopy(route)))
        count += 1

    missing = selected - {route.get("id") for route in output_root.findall("route")}
    if missing:
        raise SystemExit(f"route ids not found: {', '.join(sorted(missing))}")

    _indent(output_root)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        + ET.tostring(output_root, encoding="unicode")
        + "\n",
        encoding="utf-8",
    )
    print(f"exported {count} Leaderboard 2.0 routes to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
