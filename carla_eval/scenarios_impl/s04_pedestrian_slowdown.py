"""S04 – Detect pedestrian ahead and slow down safely."""
import math
from typing import Any, Dict, List, Tuple

import carla

from carla_eval.runtime_metrics import RouteTracker
from .base import BaseScenario, clip, compute_steer_to_location, get_speed_kmh


class PedestrianSlowdown(BaseScenario):

    def spawn_actors(self, world, ego, cfg) -> List[carla.Actor]:
        bp_lib = world.get_blueprint_library()
        ped_cfgs = cfg.get("actors", {}).get("pedestrians", [{}])
        ped_cfg = ped_cfgs[0] if ped_cfgs else {}
        ped_type = ped_cfg.get("type", "walker.pedestrian.0001")

        try:
            ped_bp = bp_lib.find(ped_type)
        except RuntimeError:
            ped_bp = bp_lib.filter("walker.pedestrian.*")[0]

        if ped_bp.has_attribute("is_invincible"):
            ped_bp.set_attribute("is_invincible", "false")

        rel = ped_cfg.get("relative_to_ego", {})
        long_dist = float(rel.get("longitudinal_distance_m", 35.0))
        lat_off = float(rel.get("lateral_offset_m", 0.0))

        ego_tf = ego.get_transform()
        fwd = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()
        ped_loc = carla.Location(
            x=ego_tf.location.x + long_dist * fwd.x + lat_off * right.x,
            y=ego_tf.location.y + long_dist * fwd.y + lat_off * right.y,
            z=ego_tf.location.z + 0.8,
        )
        yaw = math.degrees(math.atan2(fwd.y, fwd.x)) + 90.0
        ped_tf = carla.Transform(ped_loc, carla.Rotation(yaw=yaw))

        ped = world.try_spawn_actor(ped_bp, ped_tf)
        return [ped] if ped else []

    def initial_state(self, ego, actors, cfg) -> Dict[str, Any]:
        sc = cfg.get("success_criteria", {}).get("pedestrian_slowdown", {})
        return {
            "detection_dist_m": float(sc.get("detection_distance_m", 30.0)),
            "slowdown_dist_m": float(sc.get("slowdown_distance_m", 22.0)),
            "safe_speed_kmh": float(sc.get("safe_speed_kmh", 15.0)),
            "min_safe_dist_m": float(sc.get("min_safe_distance_m", 6.0)),
            "required_hold_s": float(sc.get("required_slowdown_seconds", 1.0)),
            "hold_time": 0.0,
            "pedestrian_detected": False,
            "slowdown_started": False,
            "first_detection_time": None,
            "slowdown_started_time": None,
            "success": False,
        }

    def update_state(self, ego, actors, state, obs, dt, cfg) -> Dict[str, Any]:
        if not actors:
            return state
        ped = actors[0]
        dist = ego.get_location().distance(ped.get_location())
        t = obs["timestamp"]
        speed = obs["speed_kmh"]
        collision = obs.get("collision", False)

        if dist <= state["detection_dist_m"] and not state["pedestrian_detected"]:
            state["pedestrian_detected"] = True
            state["first_detection_time"] = t

        if dist <= state["slowdown_dist_m"] and not state["slowdown_started"]:
            state["slowdown_started"] = True
            state["slowdown_started_time"] = t

        state["dist_to_ped"] = dist
        speed_ok = speed <= state["safe_speed_kmh"]
        dist_ok = dist >= state["min_safe_dist_m"]

        if state["slowdown_started"] and speed_ok and dist_ok and not collision:
            state["hold_time"] += dt
        else:
            state["hold_time"] = 0.0

        if state["hold_time"] >= state["required_hold_s"]:
            state["success"] = True

        return state

    def compute_control(self, ego, actors, state, obs, cfg) -> Tuple[float, float, float]:
        target_speed = float(cfg.get("controller", {}).get("target_speed_kmh", 35.0))
        lookahead = float(cfg.get("controller", {}).get("lookahead_m", 12.0))
        route_tracker: RouteTracker = obs.get("route_tracker")
        progress = obs["route_metrics"]["route_progress_m"]
        target_loc = route_tracker.point_at_progress(progress + lookahead) if route_tracker else ego.get_location()
        steer = compute_steer_to_location(ego, target_loc)

        speed = obs["speed_kmh"]
        dist = state.get("dist_to_ped", 999.0)
        slowdown_dist = state["slowdown_dist_m"]

        if dist <= 10.0:
            return 0.0, 0.85, steer
        if dist <= slowdown_dist:
            if speed > 15.0:
                return 0.0, 0.60, steer
            if speed > 8.0:
                return 0.0, 0.25, steer
            return 0.0, 0.0, steer

        throttle, brake = 0.0, 0.0
        error = target_speed - speed
        if error > 8:
            throttle = 0.55
        elif error > 3:
            throttle = 0.35
        elif error < -5:
            brake = 0.15
        else:
            throttle = 0.20

        return throttle, brake, steer

    def is_success(self, state, obs, cfg) -> bool:
        return state["success"]

    def extra_record(self, ego, actors, state, obs, cfg) -> Dict[str, Any]:
        ped_loc = actors[0].get_location() if actors else carla.Location()
        return {
            "pedestrian_x": ped_loc.x,
            "pedestrian_y": ped_loc.y,
            "distance_to_pedestrian": state.get("dist_to_ped", None),
            "pedestrian_detected": state["pedestrian_detected"],
            "slowdown_started": state["slowdown_started"],
            "safe_slowdown_completed": state["success"],
            "slowdown_hold_time": state["hold_time"],
            "task_success": state["success"],
        }
