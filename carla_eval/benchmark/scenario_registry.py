"""Registry mapping benchmark scenario ids/types to current implementations."""

from pathlib import Path
from typing import Dict, Type

from carla_eval.scenarios_impl import (
    BasicControlScene1,
    ComplexObstacleScene2,
    ConeDetour,
    CutInBrake,
    KeepLaneSpeed,
    LaneChange,
    PedestrianSlowdown,
    RainNightSlowdown,
)
from carla_eval.scenarios_impl.base import BaseScenario


_SCENARIO_CLASS_BY_ID: Dict[str, Type[BaseScenario]] = {
    "S01_keep_lane_speed_60": KeepLaneSpeed,
    "S02_lane_change": LaneChange,
    "S04_pedestrian_slowdown": PedestrianSlowdown,
    "S05_cone_detour": ConeDetour,
    "S07_cut_in_brake": CutInBrake,
    "S08_rain_night_danger_slowdown": RainNightSlowdown,
    "S11_basic_control_scene1_5km": BasicControlScene1,
    "S12_complex_obstacle_scene2_8km": ComplexObstacleScene2,
}

_SCENARIO_CLASS_BY_TYPE: Dict[str, Type[BaseScenario]] = {
    "KeepLaneSpeed": KeepLaneSpeed,
    "LaneChange": LaneChange,
    "PedestrianSlowdown": PedestrianSlowdown,
    "ConeDetour": ConeDetour,
    "CutInBrake": CutInBrake,
    "RainNightSlowdown": RainNightSlowdown,
    "BasicControlScene1": BasicControlScene1,
    "ComplexObstacleScene2": ComplexObstacleScene2,
}

_DEFAULT_CONFIG_BY_ID: Dict[str, Path] = {
    "S01_keep_lane_speed_60": Path("configs/scenarios/basic_control/S01_keep_lane_speed_60.yaml"),
    "S02_lane_change": Path("configs/scenarios/basic_control/S02_lane_change.yaml"),
    "S04_pedestrian_slowdown": Path("configs/scenarios/complex_obstacle/S04_pedestrian_slowdown.yaml"),
    "S05_cone_detour": Path("configs/scenarios/complex_obstacle/S05_cone_detour.yaml"),
    "S07_cut_in_brake": Path("configs/scenarios/emergency_response/S07_cut_in_brake.yaml"),
    "S08_rain_night_danger_slowdown": Path("configs/scenarios/emergency_response/S08_rain_night_danger_slowdown.yaml"),
    "S11_basic_control_scene1_5km": Path("configs/scenarios/basic_control/S11_basic_control_scene1_5km.yaml"),
    "S12_complex_obstacle_scene2_8km": Path("configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml"),
}


def create_scenario(scenario_id: str = "", scenario_type: str = "") -> BaseScenario:
    cls = None
    if scenario_id:
        cls = _SCENARIO_CLASS_BY_ID.get(scenario_id)
    if cls is None and scenario_type:
        cls = _SCENARIO_CLASS_BY_TYPE.get(scenario_type)
    if cls is None:
        raise KeyError(f"Unknown scenario id/type: {scenario_id!r} / {scenario_type!r}")
    return cls()


def get_default_config_path(scenario_id: str) -> Path:
    try:
        return _DEFAULT_CONFIG_BY_ID[scenario_id]
    except KeyError as exc:
        raise KeyError(f"No default config registered for scenario_id={scenario_id!r}") from exc
