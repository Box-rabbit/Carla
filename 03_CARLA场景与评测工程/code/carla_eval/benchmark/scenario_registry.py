"""Registry mapping benchmark scenario ids/types to current implementations."""

from pathlib import Path
from typing import Dict, Type

from carla_eval.scenarios_impl import (
    BasicControlScene1,
    ComplexObstacleScene2,
    EmergencyResponseScene3,
)
from carla_eval.scenarios_impl.base import BaseScenario


_SCENARIO_CLASS_BY_ID: Dict[str, Type[BaseScenario]] = {
    "S11_basic_control_scene1_5km": BasicControlScene1,
    "S12_complex_obstacle_scene2_8km": ComplexObstacleScene2,
    "S13_extreme_emergency_scene3_6km": EmergencyResponseScene3,
}

_SCENARIO_CLASS_BY_TYPE: Dict[str, Type[BaseScenario]] = {
    "BasicControlScene1": BasicControlScene1,
    "ComplexObstacleScene2": ComplexObstacleScene2,
    "EmergencyResponseScene3": EmergencyResponseScene3,
}

_DEFAULT_CONFIG_BY_ID: Dict[str, Path] = {
    "S11_basic_control_scene1_5km": Path("configs/scenarios/basic_control/S11_basic_control_scene1_5km.yaml"),
    "S12_complex_obstacle_scene2_8km": Path("configs/scenarios/complex_obstacle/S12_complex_obstacle_scene2_8km.yaml"),
    "S13_extreme_emergency_scene3_6km": Path("configs/scenarios/emergency_response/S13_extreme_emergency_scene3_6km.yaml"),
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
