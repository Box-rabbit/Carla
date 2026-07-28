"""Build sparse LMDrive delivery routes from validated dense design routes.

The generated XML remains Leaderboard-compatible: each waypoint only requires
the standard x/y/z fields. Extra road_option metadata documents the maneuver,
while the actual adjacent-lane geometry is encoded by the shifted waypoints.
"""

from __future__ import annotations

import argparse
import bisect
import copy
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple

import yaml


Point = Tuple[float, float, float]


def _indent_xml(elem: ET.Element, level: int = 0) -> None:
    indent = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent + "  "
        for child in elem:
            _indent_xml(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = indent
    elif level and (not elem.tail or not elem.tail.strip()):
        elem.tail = indent


def _route_element(xml_path: Path, route_id: str) -> ET.Element:
    root = ET.parse(xml_path).getroot()
    for route in root.findall("route"):
        if route.get("id") == route_id:
            return route
    raise ValueError(f"Route '{route_id}' not found in {xml_path}")


def _load_points(route: ET.Element) -> List[Point]:
    points = [
        (float(wp.get("x", 0.0)), float(wp.get("y", 0.0)), float(wp.get("z", 0.5)))
        for wp in route.findall("waypoint")
    ]
    if len(points) < 2:
        raise ValueError("At least two waypoints are required")
    return points


def _progresses(points: Sequence[Point]) -> List[float]:
    values = [0.0]
    for start, end in zip(points, points[1:]):
        values.append(values[-1] + math.hypot(end[0] - start[0], end[1] - start[1]))
    return values


def _sample(points: Sequence[Point], progress: Sequence[float], distance_m: float) -> Point:
    if distance_m <= 0.0:
        return points[0]
    if distance_m >= progress[-1]:
        return points[-1]
    idx = max(1, bisect.bisect_left(progress, distance_m))
    start, end = points[idx - 1], points[idx]
    span = max(progress[idx] - progress[idx - 1], 1e-9)
    ratio = (distance_m - progress[idx - 1]) / span
    return (
        start[0] + (end[0] - start[0]) * ratio,
        start[1] + (end[1] - start[1]) * ratio,
        start[2] + (end[2] - start[2]) * ratio,
    )


def _smoothstep(ratio: float) -> float:
    value = max(0.0, min(1.0, ratio))
    return value * value * (3.0 - 2.0 * value)


def _offset_at(progress_m: float, lane_changes: Sequence[Dict[str, Any]]) -> float:
    offset = 0.0
    for change in lane_changes:
        start = float(change["start_progress_m"])
        end = float(change["end_progress_m"])
        from_offset = float(change["from_offset_m"])
        to_offset = float(change["to_offset_m"])
        if progress_m < start:
            return offset
        if progress_m <= end:
            ratio = _smoothstep((progress_m - start) / max(end - start, 1e-6))
            return from_offset + ratio * (to_offset - from_offset)
        offset = to_offset
    return offset


def _road_option_at(progress_m: float, lane_changes: Sequence[Dict[str, Any]]) -> str:
    for change in lane_changes:
        if float(change["start_progress_m"]) <= progress_m <= float(change["end_progress_m"]):
            return str(change["command"])
    return "LANEFOLLOW"


def _sample_progresses(total_m: float, spacing_m: float, lane_changes: Sequence[Dict[str, Any]]) -> List[float]:
    values = {0.0, total_m}
    step_count = int(math.floor(total_m / spacing_m))
    values.update(round(step * spacing_m, 4) for step in range(1, step_count + 1))
    for change in lane_changes:
        start = float(change["start_progress_m"])
        end = float(change["end_progress_m"])
        values.update({start, end})
        # Keep maneuver geometry denser than ordinary cruise segments.
        current = start + 5.0
        while current < end:
            values.add(round(current, 4))
            current += 5.0
    return sorted(value for value in values if 0.0 <= value <= total_m)


def _shifted_points(points: Sequence[Point], progress: Sequence[float], output_progress: Iterable[float], lane_changes: Sequence[Dict[str, Any]]) -> List[Tuple[float, Point, str]]:
    shifted = []
    for distance_m in output_progress:
        point = _sample(points, progress, distance_m)
        before = _sample(points, progress, max(0.0, distance_m - 2.0))
        after = _sample(points, progress, min(progress[-1], distance_m + 2.0))
        dx, dy = after[0] - before[0], after[1] - before[1]
        norm = math.hypot(dx, dy)
        if norm <= 1e-9:
            shifted_point = point
        else:
            right_x, right_y = -dy / norm, dx / norm
            offset = _offset_at(distance_m, lane_changes)
            shifted_point = (point[0] + offset * right_x, point[1] + offset * right_y, point[2])
        shifted.append((distance_m, shifted_point, _road_option_at(distance_m, lane_changes)))
    return shifted


def _write_route_xml(output_path: Path, source_route: ET.Element, points: Sequence[Tuple[float, Point, str]]) -> None:
    root = ET.Element("routes")
    route = ET.SubElement(
        root,
        "route",
        id=str(source_route.get("id")),
        town=str(source_route.get("town")),
        category=str(source_route.get("category", "")),
        route_variant="lmdrive_adapted",
    )
    weather = source_route.find("weather")
    if weather is not None:
        ET.SubElement(route, "weather", attrib=dict(weather.attrib))
    for progress_m, point, road_option in points:
        ET.SubElement(
            route,
            "waypoint",
            x=f"{point[0]:.3f}",
            y=f"{point[1]:.3f}",
            z=f"{point[2]:.3f}",
            road_option=road_option,
            route_progress_m=f"{progress_m:.1f}",
        )
    _indent_xml(root)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding="unicode") + "\n",
        encoding="utf-8",
    )


def _route_length(points: Sequence[Tuple[float, Point, str]]) -> float:
    return sum(
        math.hypot(end[1][0] - start[1][0], end[1][1] - start[1][1])
        for start, end in zip(points, points[1:])
    )


def _write_validation(path: Path, route_cfg: Dict[str, Any], source_length_m: float, output: Sequence[Tuple[float, Point, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "scenario_id": route_cfg["scenario_id"],
        "route_variant": "lmdrive_adapted",
        "source_route": route_cfg["source_xml"],
        "source_length_m": round(source_length_m, 3),
        "interpolated_length_m": round(_route_length(output), 3),
        "waypoint_count": len(output),
        "max_adjacent_gap_m": round(
            max(
                math.hypot(end[1][0] - start[1][0], end[1][1] - start[1][1])
                for start, end in zip(output, output[1:])
            ),
            3,
        ),
        "lane_changes": list(route_cfg.get("lane_changes", [])),
        "validation_status": "offline_geometry_checked",
        "carla_lane_validation": "pending_carla_python_0.9.10_environment",
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _replace_route(combined_root: ET.Element, source_route: ET.Element, points: Sequence[Tuple[float, Point, str]]) -> None:
    route_id = source_route.get("id")
    for existing in list(combined_root.findall("route")):
        if existing.get("id") == route_id:
            combined_root.remove(existing)
    route = ET.SubElement(
        combined_root,
        "route",
        id=str(route_id),
        town=str(source_route.get("town")),
        category=str(source_route.get("category", "")),
        route_variant="lmdrive_adapted",
    )
    weather = source_route.find("weather")
    if weather is not None:
        ET.SubElement(route, "weather", attrib=dict(weather.attrib))
    for progress_m, point, road_option in points:
        ET.SubElement(
            route,
            "waypoint",
            x=f"{point[0]:.3f}",
            y=f"{point[1]:.3f}",
            z=f"{point[2]:.3f}",
            road_option=road_option,
            route_progress_m=f"{progress_m:.1f}",
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Build sparse LMDrive-adapted routes from dense design XML files.")
    parser.add_argument("--config", default="configs/lmdrive/route_adaptations.yaml")
    args = parser.parse_args()

    config_path = Path(args.config)
    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    combined_source = Path(cfg["combined_route_source"])
    combined_root = copy.deepcopy(ET.parse(combined_source).getroot())

    for route_cfg in cfg.get("routes", []):
        source_route = _route_element(Path(route_cfg["source_xml"]), str(route_cfg["scenario_id"]))
        source_points = _load_points(source_route)
        source_progress = _progresses(source_points)
        lane_changes = list(route_cfg.get("lane_changes", []))
        sample_progress = _sample_progresses(
            source_progress[-1],
            float(route_cfg.get("waypoint_spacing_m", 10.0)),
            lane_changes,
        )
        output = _shifted_points(source_points, source_progress, sample_progress, lane_changes)
        _write_route_xml(Path(route_cfg["output_xml"]), source_route, output)
        _write_route_xml(Path(route_cfg["bundle_output_xml"]), source_route, output)
        _write_validation(Path(route_cfg["validation_output"]), route_cfg, source_progress[-1], output)
        _replace_route(combined_root, source_route, output)

    _indent_xml(combined_root)
    combined_output = Path(cfg["combined_route_output"])
    combined_output.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(combined_root, encoding="unicode") + "\n",
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
