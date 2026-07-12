"""S02 – Lane change to the left."""
import math
from typing import Any, Dict, List, Tuple

import carla

from carla_eval.runtime_metrics import RouteTracker
from .base import BaseScenario, clip, compute_steer_to_location, get_speed_kmh


class LaneChange(BaseScenario):

    def spawn_actors(self, world, ego, cfg) -> List[carla.Actor]:
        return []

    def initial_state(self, ego, actors, cfg) -> Dict[str, Any]:
        carla_map = ego.get_world().get_map()
        wp = carla_map.get_waypoint(ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
        initial_lane = wp.lane_id if wp else None
        sc = cfg.get("success_criteria", {}).get("lane_change", {})
        return {
            "initial_lane_id": initial_lane,
            "target_lane_id": None,          # resolved after change starts
            "direction": cfg.get("success_criteria", {}).get("lane_change", {}).get("direction", "left"),
            "required_hold_s": float(sc.get("required_hold_seconds", 2.0)),
            "max_completion_s": float(sc.get("max_completion_time_seconds", 20.0)),
            "phase": "approach",             # approach → changing → holding
            "hold_time": 0.0,
            "change_start_time": None,
            "change_complete_time": None,
            "success": False,
        }

    def update_state(self, ego, actors, state, obs, dt, cfg) -> Dict[str, Any]:
        carla_map = ego.get_world().get_map()
        wp = carla_map.get_waypoint(ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
        current_lane = wp.lane_id if wp else state["initial_lane_id"]
        trigger_time = float(cfg.get("instructions", [{}])[0].get("trigger", {}).get("value", 5.0))
        t = obs["timestamp"]

        if state["phase"] == "approach" and t >= trigger_time:
            state["phase"] = "changing"
            state["change_start_time"] = t
            # Target lane: for left change in CARLA left lane has smaller |lane_id|
            if wp and wp.get_left_lane():
                state["target_lane_id"] = wp.get_left_lane().lane_id
            else:
                state["target_lane_id"] = current_lane - 1

        elif state["phase"] == "changing":
            if current_lane == state["target_lane_id"]:
                state["phase"] = "holding"
                state["change_complete_time"] = t

            if state["change_start_time"] and t - state["change_start_time"] > state["max_completion_s"]:
                state["phase"] = "timeout"

        elif state["phase"] == "holding":
            if current_lane == state["target_lane_id"]:
                state["hold_time"] += dt
            else:
                state["hold_time"] = 0.0
            if state["hold_time"] >= state["required_hold_s"]:
                state["success"] = True

        return state

    def compute_control(self, ego, actors, state, obs, cfg) -> Tuple[float, float, float]:
        lookahead = float(cfg.get("controller", {}).get("lookahead_m", 12.0))
        target_speed = float(cfg.get("controller", {}).get("target_speed_kmh", 35.0))
        route_tracker: RouteTracker = obs.get("route_tracker")
        progress = obs["route_metrics"]["route_progress_m"]

        if state["phase"] == "changing" and state.get("target_lane_id") is not None:
            carla_map = ego.get_world().get_map()
            wp = carla_map.get_waypoint(ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
            left_wp = wp.get_left_lane() if wp else None
            if left_wp:
                target_loc = left_wp.next(lookahead)[0].transform.location if left_wp.next(lookahead) else left_wp.transform.location
            elif route_tracker:
                target_loc = route_tracker.point_at_progress(progress + lookahead, lateral_offset_m=-3.5)
            else:
                target_loc = ego.get_location()
        elif route_tracker:
            target_loc = route_tracker.point_at_progress(progress + lookahead)
        else:
            target_loc = ego.get_location()

        steer = compute_steer_to_location(ego, target_loc, gain=1.35, max_steer=0.55)
        error = target_speed - obs["speed_kmh"]
        if error > 10:
            throttle, brake = 0.65, 0.0
        elif error > 3:
            throttle, brake = 0.45, 0.0
        elif error < -8:
            throttle, brake = 0.0, 0.25
        elif error < -3:
            throttle, brake = 0.0, 0.10
        else:
            throttle, brake = 0.25, 0.0
        if abs(steer) > 0.35:
            throttle = min(throttle, 0.35)

        return throttle, brake, steer

    def is_success(self, state, obs, cfg) -> bool:
        return state["success"]

    def extra_record(self, ego, actors, state, obs, cfg) -> Dict[str, Any]:
        return {
            "lane_change_phase": state["phase"],
            "hold_time": state["hold_time"],
            "change_complete_time": state["change_complete_time"],
            "task_success": state["success"],
        }
