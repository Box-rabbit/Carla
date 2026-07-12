"""S01 – Keep current lane and reach target speed."""
from typing import Any, Dict, List, Tuple

import carla

from carla_eval.runtime_metrics import RouteTracker
from .base import BaseScenario, clip, compute_steer_to_location, get_speed_kmh


class KeepLaneSpeed(BaseScenario):

    def spawn_actors(self, world, ego, cfg) -> List[carla.Actor]:
        return []

    def initial_state(self, ego, actors, cfg) -> Dict[str, Any]:
        sc = cfg.get("success_criteria", {}).get("target_speed", {})
        return {
            "target_speed_kmh": float(cfg.get("target_speed", {}).get("value_kmh", 60.0)),
            "tolerance_kmh": float(sc.get("tolerance_kmh", 8.0)),
            "required_hold_s": float(sc.get("required_hold_seconds", 2.0)),
            "hold_time": 0.0,
            "success": False,
        }

    def update_state(self, ego, actors, state, obs, dt, cfg) -> Dict[str, Any]:
        speed = obs["speed_kmh"]
        trigger_time = cfg.get("instructions", [{}])[0].get("trigger", {}).get("value", 5.0)
        lo = state["target_speed_kmh"] - state["tolerance_kmh"]
        hi = state["target_speed_kmh"] + state["tolerance_kmh"]
        if lo <= speed <= hi and obs["timestamp"] >= trigger_time:
            state["hold_time"] += dt
        else:
            state["hold_time"] = 0.0
        if state["hold_time"] >= state["required_hold_s"]:
            state["success"] = True
        return state

    def compute_control(self, ego, actors, state, obs, cfg) -> Tuple[float, float, float]:
        route_metrics = obs["route_metrics"]
        lookahead = float(cfg.get("controller", {}).get("lookahead_m", 18.0))
        progress = route_metrics["route_progress_m"]
        route_tracker: RouteTracker = obs.get("route_tracker")

        if route_tracker is not None:
            target_loc = route_tracker.point_at_progress(progress + lookahead)
        else:
            target_loc = ego.get_location()

        steer = compute_steer_to_location(ego, target_loc, gain=1.25, max_steer=0.45)
        speed = obs["speed_kmh"]
        target = state["target_speed_kmh"]
        error = target - speed

        if error > 18:
            throttle, brake = 1.00, 0.0
        elif error > 10:
            throttle, brake = 0.90, 0.0
        elif error > 3:
            throttle, brake = 0.75, 0.0
        elif error < -8:
            throttle, brake = 0.0, 0.25
        elif error < -3:
            throttle, brake = 0.0, 0.10
        else:
            throttle, brake = 0.45, 0.0

        if abs(steer) > 0.25:
            throttle = min(throttle, 0.65)
        if abs(steer) > 0.38 and speed > 58:
            throttle, brake = 0.0, max(brake, 0.10)

        return throttle, brake, steer

    def is_success(self, state, obs, cfg) -> bool:
        return state["success"]

    def extra_record(self, ego, actors, state, obs, cfg) -> Dict[str, Any]:
        return {
            "target_speed_kmh": state["target_speed_kmh"],
            "speed_hold_time": state["hold_time"],
            "task_success": state["success"],
        }
