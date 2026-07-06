from dataclasses import dataclass, field
from typing import Any, Dict, List
@dataclass
class BaseScenario:
    scenario_id: str
    category: str
    config: Dict[str, Any]
    instructions: List[Dict[str, Any]] = field(default_factory=list)
    @classmethod
    def from_config(cls, config):
        return cls(config['scenario_id'], config['category'], config, config.get('instructions', []))
