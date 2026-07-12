"""S08 – Rain-night hazard: compute hazard score and maintain safe low speed."""
import math
from typing import Any, Dict, List, Tuple

import carla

from carla_eval.runtime_metrics import RouteTracker
from .base import BaseScenario, clip, compute_steer_to_location, get_speed_kmh


def _compute_hazard_score(weather_params: Dict) -> float:
    """Derive a 0-1 risk score from CARLA weather parameters."""
    precip = float(weather_params.get("precipitation", 0.0)) / 100.0
    fog = min(float(weather_params.get("fog_density", 0.0)) / 80.0, 1.0)
    altitude = float(weather_params.get("sun_altitude_angle", 45.0))
    night = max(0.0, min(1.0, (-altitude - 10.0) / 30.0)) if altitude < -10 else 0.0
    wetness = float(weather_params.get("wetness", 0.0)) / 100.0
    score = 0.35 * precip + 0.20 * fog + 0.25 * night + 0.20 * wetness
    return min(1.0, score)


class RainNightSlowdown(BaseScenario):

    def spawn_actors(self, world, ego, cfg) -> List[carla.Actor]:
        return []

    def initial_state(self, ego, actors, cfg) -> Dict[str, Any]:
        sc = cfg.get("success_criteria", {}).get("rain_night_slowdown", {})
        weather_params = cfg.get("map", {}).get("weather_parameters", {})
        hazard_score = _compute_hazard_score(weather_params)
        return {
            "hazard_score_threshold": float(sc.get("hazard_score_threshold", 0.60)),
            "safe_speed_kmh": float(sc.get("safe_speed_kmh", 18.0)),
            "min_operating_kmh": float(sc.get("min_operating_speed_kmh", 8.0)),
            "required_hold_s": float(sc.get("required_safe_speed_hold_seconds", 2.0)),
            "min_travel_m": float(sc.get("min_travel_distance_m", 20.0)),
            "hazard_score": hazard_score,
            "danger_detected": hazard_score >= float(sc.get("hazard_score_threshold", 0.60)),
            "slowdown_started": False,
            "hold_time": 0.0,
            "success": False,
            "total_travel_m": 0.0,
            "_last_loc": None,
        }

    def update_state(self, ego, actors, state, obs, dt, cfg) -> Dict[str, Any]:
        speed = obs["speed_kmh"]
        ego_loc = ego.get_location()

        if state["_last_loc"] is not None:
            prev = state["_last_loc"]
            state["total_travel_m"] += math.hypot(
                ego_loc.x - prev.x, ego_loc.y - prev.y
            )
        state["_last_loc"] = ego_loc

        if state["danger_detected"]:
            if not state["slowdown_started"] and speed <= state["safe_speed_kmh"]:
                state["slowdown_started"] = True

            if (
                state["slowdown_started"]
                and state["min_operating_kmh"] <= speed <= state["safe_speed_kmh"]
            ):
                state["hold_time"] += dt
            else:
                state["hold_time"] = 0.0

            if (
                state["hold_time"] >= state["required_hold_s"]
                and state["total_travel_m"] >= state["min_travel_m"]
            ):
                state["success"] = True

        return state

    def compute_control(self, ego, actors, state, obs, cfg) -> Tuple[float, float, float]:
        ctrl = cfg.get("controller", {})
        lookahead = float(ctrl.get("lookahead_m", 14.0))
        normal_speed = float(ctrl.get("normal_speed_kmh", 35.0))
        route_tracker: RouteTracker = obs.get("route_tracker")
        progress = obs["route_metrics"]["route_progress_m"]
        target_loc = route_tracker.point_at_progress(progress + lookahead) if route_tracker else ego.get_location()
        steer = compute_steer_to_location(ego, target_loc)

        speed = obs["speed_kmh"]
        if state["danger_detected"]:
            target_speed = state["safe_speed_kmh"]
        else:
            target_speed = normal_speed

        error = target_speed - speed
        if error > 8:
            throttle, brake = 0.45, 0.0
        elif error > 3:
            throttle, brake = 0.30, 0.0
        elif error < -5:
            throttle, brake = 0.0, 0.30
        elif error < -2:
            throttle, brake = 0.0, 0.15
        else:
            throttle, brake = 0.20, 0.0

        return throttle, brake, steer

    def is_success(self, state, obs, cfg) -> bool:
        return state["success"]

    def extra_record(self, ego, actors, state, obs, cfg) -> Dict[str, Any]:
        return {
            "hazard_score": state["hazard_score"],
            "danger_detected": state["danger_detected"],
            "slowdown_started": state["slowdown_started"],
            "safe_speed_hold_time": state["hold_time"],
            "total_travel_m": state["total_travel_m"],
            "task_success": state["success"],
        }
