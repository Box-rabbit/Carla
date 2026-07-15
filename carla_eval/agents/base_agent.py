"""Agent interface aligned with CARLA Leaderboard / LMDrive style."""

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Tuple


class BaseAgent(ABC):
    """
    Minimal benchmark agent contract.

    A real LMDrive adapter should implement this interface and return
    (throttle, brake, steer) from multimodal observations and route commands.
    Current rule scenarios still use their scenario-local controller, but the
    benchmark runner now has a stable place to plug a model agent in.
    """

    def setup(self, config: Optional[Dict[str, Any]] = None) -> None:
        self.config = config or {}

    def sensors(self):
        return []

    @abstractmethod
    def run_step(
        self,
        input_data: Dict[str, Any],
        timestamp: float,
        instruction: Optional[Dict[str, Any]] = None,
    ) -> Tuple[float, float, float]:
        """Return (throttle, brake, steer)."""

    def destroy(self) -> None:
        pass


class ScenarioControllerAgent(BaseAgent):
    """
    Adapter for the current repository's scenario-local controllers.

    It exists to make the benchmark interface explicit while preserving the
    current stable scenario implementations.
    """

    def run_step(
        self,
        input_data: Dict[str, Any],
        timestamp: float,
        instruction: Optional[Dict[str, Any]] = None,
    ) -> Tuple[float, float, float]:
        scenario = input_data["scenario"]
        ego = input_data["ego"]
        actors = input_data["actors"]
        state = input_data["state"]
        obs = input_data["obs"]
        cfg = input_data["cfg"]
        return scenario.compute_control(ego, actors, state, obs, cfg)
