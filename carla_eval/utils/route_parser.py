"""
Route XML and Scenario JSON parser, modelled after LMDrive's RouteParser.

Responsibilities:
- parse_routes_file()   : XML → list[RouteScenarioConfiguration]
- parse_annotations_file(): Scenario JSON → dict{town: [scenario_list]}
- scan_route_for_scenarios(): spatial match between dense route and trigger db
"""

import json
import math
import xml.etree.ElementTree as ET
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import carla


@dataclass
class RouteScenarioConfiguration:
    """Parsed description of one route + its associated scenario file."""
    name: str                          # e.g. "RouteScenario_S01"
    town: str                          # e.g. "Town03"
    scenario_file: Optional[Path]      # path to Scenario JSON
    trajectory: List[carla.Location]   # sparse waypoints from XML
    weather: Dict[str, Any]            # weather kwargs for carla.WeatherParameters
    scenario_id: str = ""              # original route id from XML
    category: str = ""                 # basic_control / complex_obstacle / emergency_response
    extra: Dict[str, Any] = field(default_factory=dict)   # raw yaml config if needed


_PRESET_WEATHER = {
    "1":  "ClearNoon",
    "2":  "CloudyNoon",
    "3":  "WetNoon",
    "4":  "WetCloudyNoon",
    "5":  "SoftRainNoon",
    "6":  "MidRainyNoon",
    "7":  "HardRainNoon",
    "8":  "ClearSunset",
    "9":  "CloudySunset",
    "10": "WetSunset",
    "11": "WetCloudySunset",
    "12": "SoftRainSunset",
    "13": "MidRainSunset",
    "14": "SoftRainSunset",
}


def _parse_weather_element(route_elem: ET.Element) -> Dict[str, Any]:
    """Parse <weather> child element or weather attribute into carla kwargs."""
    weather_elem = route_elem.find("weather")
    if weather_elem is not None:
        kwargs: Dict[str, Any] = {}
        for attr in (
            "cloudiness", "precipitation", "precipitation_deposits",
            "wetness", "wind_intensity", "fog_density", "fog_distance",
            "sun_altitude_angle", "sun_azimuth_angle",
        ):
            val = weather_elem.get(attr)
            if val is not None:
                kwargs[attr] = float(val)
        return kwargs

    preset_key = route_elem.get("weather", "")
    if preset_key in _PRESET_WEATHER:
        preset_name = _PRESET_WEATHER[preset_key]
        if hasattr(carla.WeatherParameters, preset_name):
            wp = getattr(carla.WeatherParameters, preset_name)
            return {
                "cloudiness": wp.cloudiness,
                "precipitation": wp.precipitation,
                "precipitation_deposits": wp.precipitation_deposits,
                "sun_altitude_angle": wp.sun_altitude_angle,
            }

    named = route_elem.get("weather", "")
    if named and hasattr(carla.WeatherParameters, named):
        wp = getattr(carla.WeatherParameters, named)
        return {
            "cloudiness": wp.cloudiness,
            "precipitation": wp.precipitation,
            "precipitation_deposits": wp.precipitation_deposits,
            "sun_altitude_angle": wp.sun_altitude_angle,
        }

    return {"sun_altitude_angle": 70.0, "cloudiness": 30.0}


class RouteParser:
    """Parse route XML and scenario JSON files (LMDrive-style)."""

    @staticmethod
    def parse_routes_file(
        route_file: Path,
        scenario_file: Optional[Path] = None,
        single_route: Optional[str] = None,
    ) -> List[RouteScenarioConfiguration]:
        """
        Parse a route XML file into a list of RouteScenarioConfiguration.

        Args:
            route_file:    path to route XML
            scenario_file: path to scenario JSON (attached to every config)
            single_route:  if set, only return the route with this id
        """
        route_file = Path(route_file)
        if not route_file.exists():
            raise FileNotFoundError(f"Route file not found: {route_file}")

        tree = ET.parse(route_file)
        root = tree.getroot()

        configs: List[RouteScenarioConfiguration] = []
        for route_elem in root.findall("route"):
            route_id = route_elem.get("id", "0")
            if single_route is not None and route_id != str(single_route):
                continue

            town = route_elem.get("town", "Town03")
            category = route_elem.get("category", "")
            weather = _parse_weather_element(route_elem)

            trajectory: List[carla.Location] = []
            for wp_elem in route_elem.findall("waypoint"):
                trajectory.append(carla.Location(
                    x=float(wp_elem.get("x", 0.0)),
                    y=float(wp_elem.get("y", 0.0)),
                    z=float(wp_elem.get("z", 0.0)),
                ))

            cfg = RouteScenarioConfiguration(
                name=f"RouteScenario_{route_id}",
                town=town,
                scenario_file=Path(scenario_file) if scenario_file else None,
                trajectory=trajectory,
                weather=weather,
                scenario_id=route_id,
                category=category,
            )
            configs.append(cfg)

        return configs

    @staticmethod
    def parse_annotations_file(scenario_file: Path) -> OrderedDict:
        """
        Parse a scenario JSON file into {town: [scenario_dict, ...]} mapping.

        Mirrors LMDrive's parse_annotations_file().
        """
        scenario_file = Path(scenario_file)
        if not scenario_file.exists():
            return OrderedDict()

        with scenario_file.open("r", encoding="utf-8") as f:
            data = json.load(f)

        result: OrderedDict = OrderedDict()
        for town_block in data.get("available_scenarios", []):
            for town_name, scenario_list in town_block.items():
                if town_name not in result:
                    result[town_name] = []
                result[town_name].extend(scenario_list)

        return result

    @staticmethod
    def scan_route_for_scenarios(
        town: str,
        dense_route: List[carla.Location],
        world_annotations: OrderedDict,
        trigger_radius_m: float = 5.0,
        yaw_tolerance_deg: float = 30.0,
    ) -> Dict[str, List[Dict]]:
        """
        Spatial match: find which scenario triggers from the JSON fall on this route.

        Returns dict keyed by trigger_id, each value is a list of matching scenario
        configs (different scenario_types at the same location).
        """
        if town not in world_annotations:
            return {}

        scenario_list = world_annotations[town]
        potential: Dict[str, List[Dict]] = {}

        for scenario_info in scenario_list:
            scenario_type = scenario_info.get("scenario_type", "")
            for idx, event_cfg in enumerate(
                scenario_info.get("available_event_configurations", [])
            ):
                trigger = event_cfg.get("transform", {})
                trigger_loc = carla.Location(
                    x=float(trigger.get("x", 0.0)),
                    y=float(trigger.get("y", 0.0)),
                    z=float(trigger.get("z", 0.0)),
                )
                trigger_yaw = float(trigger.get("yaw", 0.0))

                match = RouteParser._match_to_route(
                    trigger_loc, trigger_yaw, dense_route,
                    trigger_radius_m, yaw_tolerance_deg,
                )
                if match is None:
                    continue

                trigger_id = f"{scenario_type}_{idx}"
                if trigger_id not in potential:
                    potential[trigger_id] = []

                potential[trigger_id].append({
                    "scenario_type": scenario_type,
                    "trigger_transform": trigger,
                    "other_actors": event_cfg.get("other_actors", {}),
                    "route_progress_m": match["progress_m"],
                    "trigger_id": trigger_id,
                })

        return potential

    @staticmethod
    def _match_to_route(
        loc: carla.Location,
        yaw_deg: float,
        route: List[carla.Location],
        radius_m: float,
        yaw_tol_deg: float,
    ) -> Optional[Dict]:
        """Check if loc is within radius_m of any route segment."""
        best_dist = float("inf")
        best_progress = 0.0
        cumulative = 0.0

        for i in range(1, len(route)):
            a = route[i - 1]
            b = route[i]
            seg_len = math.hypot(b.x - a.x, b.y - a.y)
            if seg_len < 1e-6:
                continue

            dx, dy = b.x - a.x, b.y - a.y
            t = ((loc.x - a.x) * dx + (loc.y - a.y) * dy) / (seg_len * seg_len)
            t = max(0.0, min(1.0, t))
            proj_x = a.x + t * dx
            proj_y = a.y + t * dy
            dist = math.hypot(loc.x - proj_x, loc.y - proj_y)

            if dist < best_dist:
                best_dist = dist
                best_progress = cumulative + t * seg_len

                seg_yaw = math.degrees(math.atan2(dy, dx))
                yaw_diff = abs((yaw_deg - seg_yaw + 180) % 360 - 180)

            cumulative += seg_len

        if best_dist <= radius_m:
            return {"progress_m": best_progress, "dist_m": best_dist}
        return None
