"""
BaseScenario: abstract base class for all scenario implementations.

Concrete scenarios implement:
  spawn_actors()   – create and return NPC actors
  initial_state()  – return initial mutable state dict
  update_state()   – update state from latest observation
  compute_control()– return (throttle, brake, steer)
  is_success()     – return True when task is done
  extra_record()   – return scenario-specific log fields
"""

import math
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple

import carla


def clip(x: float, low: float, high: float) -> float:
    return max(low, min(high, x))


def get_speed_kmh(vehicle: carla.Actor) -> float:
    v = vehicle.get_velocity()
    return 3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


def compute_steer_to_location(
    vehicle: carla.Actor,
    target_loc: carla.Location,
    gain: float = 1.30,
    max_steer: float = 0.45,
) -> float:
    tf = vehicle.get_transform()
    loc = tf.location
    forward = tf.get_forward_vector()
    right = tf.get_right_vector()
    dx = target_loc.x - loc.x
    dy = target_loc.y - loc.y
    angle = math.atan2(dx * right.x + dy * right.y, dx * forward.x + dy * forward.y)
    return clip(gain * angle, -max_steer, max_steer)


def compute_speed_pid(
    speed_kmh: float,
    target_kmh: float,
    steer: float = 0.0,
    steer_throttle_penalty: float = 0.0,
) -> Tuple[float, float]:
    """Simple proportional speed controller → (throttle, brake)."""
    error = target_kmh - speed_kmh
    if error > 10:
        throttle, brake = 0.80, 0.0
    elif error > 5:
        throttle, brake = 0.55, 0.0
    elif error > 1:
        throttle, brake = 0.35, 0.0
    elif error < -8:
        throttle, brake = 0.0, 0.30
    elif error < -3:
        throttle, brake = 0.0, 0.15
    else:
        throttle, brake = 0.20, 0.0

    if steer_throttle_penalty > 0 and abs(steer) > 0.30:
        throttle = min(throttle, steer_throttle_penalty)
    return throttle, brake


class BaseScenario(ABC):
    """Abstract base for all Dongfeng CARLA scenario implementations."""

    @abstractmethod
    def spawn_actors(
        self,
        world: carla.World,
        ego: carla.Actor,
        cfg: Dict[str, Any],
    ) -> List[carla.Actor]:
        """Spawn NPC actors; return list so evaluator can clean them up."""

    @abstractmethod
    def initial_state(
        self,
        ego: carla.Actor,
        actors: List[carla.Actor],
        cfg: Dict[str, Any],
    ) -> Dict[str, Any]:
        """Return initial mutable state dict."""

    @abstractmethod
    def update_state(
        self,
        ego: carla.Actor,
        actors: List[carla.Actor],
        state: Dict[str, Any],
        obs: Dict[str, Any],
        dt: float,
        cfg: Dict[str, Any],
    ) -> Dict[str, Any]:
        """Update and return state."""

    @abstractmethod
    def compute_control(
        self,
        ego: carla.Actor,
        actors: List[carla.Actor],
        state: Dict[str, Any],
        obs: Dict[str, Any],
        cfg: Dict[str, Any],
    ) -> Tuple[float, float, float]:
        """Return (throttle, brake, steer)."""

    @abstractmethod
    def is_success(
        self,
        state: Dict[str, Any],
        obs: Dict[str, Any],
        cfg: Dict[str, Any],
    ) -> bool:
        """Return True when the scenario task is complete."""

    def extra_record(
        self,
        ego: carla.Actor,
        actors: List[carla.Actor],
        state: Dict[str, Any],
        obs: Dict[str, Any],
        cfg: Dict[str, Any],
    ) -> Dict[str, Any]:
        """Scenario-specific log fields (default: empty)."""
        return {}
