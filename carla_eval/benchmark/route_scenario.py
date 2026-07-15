"""LMDrive-style RouteScenario adapter for current Dongfeng scenarios."""

import copy
from pathlib import Path
from typing import Any, Dict, Optional

from carla_eval.benchmark.annotations import ScenarioAnnotation
from carla_eval.benchmark.scenario_registry import create_scenario, get_default_config_path
from carla_eval.evaluator import ScenarioEvaluator, load_scenario_config
from carla_eval.utils.route_parser import RouteScenarioConfiguration


class DongfengRouteScenario:
    """
    Combines route XML config and scenario annotation into a runnable scenario.

    This is intentionally lighter than CARLA Leaderboard's RouteScenario:
    it preserves the current ScenarioEvaluator while moving scene selection,
    route source and annotations into LMDrive-style files.
    """

    def __init__(
        self,
        route_config: RouteScenarioConfiguration,
        annotation: Optional[ScenarioAnnotation],
        routes_file: Path,
    ):
        self.route_config = route_config
        self.annotation = annotation
        self.routes_file = Path(routes_file)

    def build(self):
        scenario_id = self.scenario_id
        config_path = self.config_path
        cfg = load_scenario_config(str(config_path))
        cfg = self._apply_route_config(cfg)
        cfg = self._apply_annotation(cfg)
        scenario = create_scenario(scenario_id=scenario_id, scenario_type=self.scenario_type)
        return scenario, cfg, config_path

    @property
    def scenario_id(self) -> str:
        if self.annotation and self.annotation.scenario_id:
            return self.annotation.scenario_id
        return self.route_config.scenario_id

    @property
    def scenario_type(self) -> str:
        return self.annotation.scenario_type if self.annotation else ""

    @property
    def config_path(self) -> Path:
        if self.annotation and self.annotation.config_path:
            return self.annotation.config_path
        return get_default_config_path(self.scenario_id)

    def run(self, agent=None, **kwargs) -> Dict[str, Any]:
        scenario, cfg, config_path = self.build()
        return ScenarioEvaluator(scenario, cfg, config_path).run(agent=agent, **kwargs)

    def _apply_route_config(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        cfg = copy.deepcopy(cfg)
        cfg.setdefault("map", {})["town"] = self.route_config.town

        if self.route_config.weather and "weather_parameters" not in cfg.get("map", {}):
            cfg.setdefault("map", {})["weather_parameters"] = dict(self.route_config.weather)

        route_cfg = cfg.setdefault("route", {})
        if route_cfg.get("mode") != "carla_lane_trace":
            route_cfg["route_file"] = str(self.routes_file)
            route_cfg["route_id"] = self.route_config.scenario_id

        cfg["scenario_id"] = self.scenario_id
        if self.route_config.category:
            cfg["category"] = self.route_config.category
        cfg.setdefault("runtime", {})["random_seed"] = self.route_config.extra.get("random_seed", 0)
        return cfg

    def _apply_annotation(self, cfg: Dict[str, Any]) -> Dict[str, Any]:
        if self.annotation is None:
            return cfg

        cfg.setdefault("benchmark", {})
        cfg["benchmark"].update({
            "route_id": self.annotation.route_id,
            "scenario_type": self.annotation.scenario_type,
            "trigger": self.annotation.trigger,
            "expected": self.annotation.expected,
        })
        return cfg
