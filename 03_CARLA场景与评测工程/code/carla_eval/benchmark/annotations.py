"""Scenario annotation parser, modelled after Leaderboard scenario JSON."""

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


@dataclass
class ScenarioAnnotation:
    town: str
    scenario_id: str
    scenario_type: str
    config_path: Optional[Path] = None
    route_id: str = ""
    trigger: Dict[str, Any] = field(default_factory=dict)
    expected: Dict[str, Any] = field(default_factory=dict)
    actors: Dict[str, Any] = field(default_factory=dict)
    raw: Dict[str, Any] = field(default_factory=dict)


class ScenarioAnnotationStore:
    """Load and query Dongfeng scenario annotations."""

    def __init__(self, annotations: List[ScenarioAnnotation]):
        self.annotations = annotations

    @classmethod
    def from_file(cls, path: Path) -> "ScenarioAnnotationStore":
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Scenario annotation file not found: {path}")

        with path.open("r", encoding="utf-8") as f:
            if path.suffix.lower() == ".json":
                data = json.load(f)
            else:
                data = yaml.safe_load(f)

        annotations: List[ScenarioAnnotation] = []
        for town_block in data.get("available_scenarios", []):
            for town, scenario_list in town_block.items():
                for item in scenario_list:
                    event_cfgs = item.get("available_event_configurations", [])
                    event_cfg = event_cfgs[0] if event_cfgs else {}
                    config_path = item.get("config_path")
                    annotations.append(ScenarioAnnotation(
                        town=town,
                        scenario_id=str(item.get("scenario_id", item.get("route_id", ""))),
                        scenario_type=str(item.get("scenario_type", "")),
                        config_path=Path(config_path) if config_path else None,
                        route_id=str(item.get("route_id", item.get("scenario_id", ""))),
                        trigger=event_cfg.get("trigger", item.get("trigger", {})),
                        expected=event_cfg.get("expected", item.get("expected", {})),
                        actors=event_cfg.get("actors", item.get("actors", {})),
                        raw=item,
                    ))
        return cls(annotations)

    def find(self, town: str, route_id: str) -> Optional[ScenarioAnnotation]:
        route_id = str(route_id)
        for ann in self.annotations:
            if ann.town == town and ann.route_id == route_id:
                return ann
        for ann in self.annotations:
            if ann.route_id == route_id:
                return ann
        return None
