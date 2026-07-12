"""S05 – Detect construction cones ahead, detour left, return to lane."""
import math
from typing import Any, Dict, List, Optional, Tuple

import carla

from carla_eval.runtime_metrics import RouteTracker
from .base import BaseScenario, clip, compute_steer_to_location, get_speed_kmh


def _spawn_cone(world, blueprint_library, loc: carla.Location) -> Optional[carla.Actor]:
    try:
        bp = blueprint_library.find("static.prop.trafficcone01")
    except RuntimeError:
        bp = blueprint_library.filter("static.prop.*cone*")[0]
    return world.try_spawn_actor(bp, carla.Transform(loc))


class ConeDetour(BaseScenario):

    def spawn_actors(self, world, ego, cfg) -> List[carla.Actor]:
        bp_lib = world.get_blueprint_library()
        ego_tf = ego.get_transform()
        fwd = ego_tf.get_forward_vector()
        actors = []
        for prop in cfg.get("actors", {}).get("static_props", []):
            if "cone" not in prop.get("type", ""):
                continue
            rel = prop.get("relative_to_ego", {})
            long_d = float(rel.get("longitudinal_distance_m", 30.0))
            lat_d = float(rel.get("lateral_offset_m", 0.0))
            right = ego_tf.get_right_vector()
            loc = carla.Location(
                x=ego_tf.location.x + long_d * fwd.x + lat_d * right.x,
                y=ego_tf.location.y + long_d * fwd.y + lat_d * right.y,
                z=ego_tf.location.z + 0.05,
            )
            cone = _spawn_cone(world, bp_lib, loc)
            if cone:
                actors.append(cone)
        return actors

    def initial_state(self, ego, actors, cfg) -> Dict[str, Any]:
        sc = cfg.get("success_criteria", {}).get("cone_detour", {})
        return {
            "detection_dist_m": float(sc.get("detection_distance_m", 35.0)),
            "corridor_half_w": float(sc.get("detection_corridor_half_width_m", 1.8)),
            "detour_lat_m": float(sc.get("detour_lateral_offset_m", 3.5)),
            "no_cone_hold_s": float(sc.get("no_cone_ahead_hold_seconds", 1.0)),
            "required_return_s": float(sc.get("required_return_hold_seconds", 1.0)),
            "phase": "approach",    # approach → detouring → returning → done
            "no_cone_timer": 0.0,
            "return_hold_time": 0.0,
            "cone_detected": False,
            "max_lateral_offset_m": 0.0,
            "success": False,
        }

    def _front_corridor_cones(self, ego, actors, state) -> List[carla.Actor]:
        ego_tf = ego.get_transform()
        fwd = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()
        ego_loc = ego_tf.location
        half_w = state["corridor_half_w"]
        found = []
        for cone in actors:
            cl = cone.get_location()
            dx = cl.x - ego_loc.x
            dy = cl.y - ego_loc.y
            forward_dist = dx * fwd.x + dy * fwd.y
            lateral_dist = dx * right.x + dy * right.y
            if 0 < forward_dist <= state["detection_dist_m"] and abs(lateral_dist) <= half_w:
                found.append(cone)
        return found

    def update_state(self, ego, actors, state, obs, dt, cfg) -> Dict[str, Any]:
        front_cones = self._front_corridor_cones(ego, actors, state)

        if front_cones and not state["cone_detected"]:
            state["cone_detected"] = True

        route_metrics = obs["route_metrics"]
        state["max_lateral_offset_m"] = max(
            state["max_lateral_offset_m"],
            abs(route_metrics.get("lateral_offset_from_route_m", 0.0))
        )

        if state["phase"] == "approach" and front_cones:
            state["phase"] = "detouring"

        elif state["phase"] == "detouring":
            if not front_cones:
                state["no_cone_timer"] += dt
                if state["no_cone_timer"] >= state["no_cone_hold_s"]:
                    state["phase"] = "returning"
            else:
                state["no_cone_timer"] = 0.0

        elif state["phase"] == "returning":
            lat_off = route_metrics.get("lateral_offset_from_route_m", 999.0)
            if abs(lat_off) < 1.0:
                state["return_hold_time"] += dt
            else:
                state["return_hold_time"] = 0.0
            if state["return_hold_time"] >= state["required_return_s"]:
                state["phase"] = "done"
                state["success"] = True

        return state

    def compute_control(self, ego, actors, state, obs, cfg) -> Tuple[float, float, float]:
        ctrl = cfg.get("controller", {})
        route_tracker: RouteTracker = obs.get("route_tracker")
        progress = obs["route_metrics"]["route_progress_m"]
        phase = state["phase"]

        if phase == "approach":
            lookahead = float(ctrl.get("approach_lookahead_m", 14.0))
            target_speed = float(ctrl.get("approach_target_speed_kmh", 24.0))
            gain = float(ctrl.get("approach_steer_gain", 1.35))
            max_steer = float(ctrl.get("approach_max_steer", 0.35))
            lat_off = 0.0
        elif phase == "detouring":
            lookahead = float(ctrl.get("change_left_lookahead_m", 6.0))
            target_speed = float(ctrl.get("change_left_target_speed_kmh", 13.0))
            gain = float(ctrl.get("change_left_steer_gain", 1.20))
            max_steer = float(ctrl.get("change_left_max_steer", 0.38))
            lat_off = -float(ctrl.get("follow_left_target_lateral_offset_m", 3.2))
        elif phase == "returning":
            lookahead = float(ctrl.get("return_lookahead_m", 10.0))
            target_speed = float(ctrl.get("return_target_speed_kmh", 22.0))
            gain = float(ctrl.get("return_steer_gain", 1.55))
            max_steer = float(ctrl.get("return_max_steer", 0.34))
            lat_off = 0.0
        else:
            lookahead = 12.0
            target_speed = float(ctrl.get("complete_target_speed_kmh", 22.0))
            gain, max_steer, lat_off = 1.35, 0.25, 0.0

        if route_tracker:
            target_loc = route_tracker.point_at_progress(progress + lookahead, lateral_offset_m=lat_off)
        else:
            target_loc = ego.get_location()

        steer = compute_steer_to_location(ego, target_loc, gain=gain, max_steer=max_steer)
        speed = obs["speed_kmh"]
        error = target_speed - speed
        if error > 8:
            throttle, brake = 0.55, 0.0
        elif error > 3:
            throttle, brake = 0.35, 0.0
        elif error < -5:
            throttle, brake = 0.0, 0.20
        elif error < -2:
            throttle, brake = 0.0, 0.10
        else:
            throttle, brake = 0.20, 0.0

        large_steer_t = float(ctrl.get("large_steer_throttle_cap", 0.15))
        thresh = float(ctrl.get("large_steer_threshold", 0.35))
        if abs(steer) > thresh:
            throttle = min(throttle, large_steer_t)

        return throttle, brake, steer

    def is_success(self, state, obs, cfg) -> bool:
        return state["success"]

    def extra_record(self, ego, actors, state, obs, cfg) -> Dict[str, Any]:
        return {
            "detour_phase": state["phase"],
            "cone_detected": state["cone_detected"],
            "max_lateral_offset_m": state["max_lateral_offset_m"],
            "return_hold_time": state["return_hold_time"],
            "task_success": state["success"],
        }
