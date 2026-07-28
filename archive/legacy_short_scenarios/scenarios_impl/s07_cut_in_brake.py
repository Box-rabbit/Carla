"""S07 – NPC cuts in from adjacent lane then brakes; ego must respond."""
import math
from typing import Any, Dict, List, Tuple

import carla

from carla_eval.runtime_metrics import RouteTracker
from .base import BaseScenario, clip, compute_steer_to_location, get_speed_kmh


class CutInBrake(BaseScenario):

    def spawn_actors(self, world, ego, cfg) -> List[carla.Actor]:
        bp_lib = world.get_blueprint_library()
        vehicles = cfg.get("actors", {}).get("vehicles", [])
        if not vehicles:
            return []
        npc_cfg = vehicles[0]
        npc_type = npc_cfg.get("type", "vehicle.audi.tt")
        try:
            bp = bp_lib.find(npc_type)
        except RuntimeError:
            bp = bp_lib.filter("vehicle.audi.*")[0]

        rel = npc_cfg.get("relative_to_ego", {})
        long_d = float(rel.get("longitudinal_distance_m", 26.0))
        lat_d = float(rel.get("lateral_offset_m", -3.5))
        ego_tf = ego.get_transform()
        fwd = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()
        npc_loc = carla.Location(
            x=ego_tf.location.x + long_d * fwd.x + lat_d * right.x,
            y=ego_tf.location.y + long_d * fwd.y + lat_d * right.y,
            z=ego_tf.location.z + 0.3,
        )
        yaw = ego_tf.rotation.yaw
        npc = world.try_spawn_actor(bp, carla.Transform(npc_loc, carla.Rotation(yaw=yaw)))
        return [npc] if npc else []

    def initial_state(self, ego, actors, cfg) -> Dict[str, Any]:
        sc = cfg.get("success_criteria", {}).get("cut_in_brake", {})
        npc_beh = cfg.get("actors", {}).get("vehicles", [{}])[0].get("cut_in_behavior", {})
        return {
            "detection_dist_m": float(sc.get("detection_distance_m", 45.0)),
            "front_half_w": float(sc.get("front_corridor_half_width_m", 1.8)),
            "emergency_brake_dist_m": float(sc.get("emergency_brake_distance_m", 17.0)),
            "emergency_ttc_s": float(sc.get("emergency_brake_ttc_s", 3.0)),
            "safe_follow_dist_m": float(sc.get("safe_follow_distance_m", 7.0)),
            "safe_speed_kmh": float(sc.get("safe_speed_kmh", 10.0)),
            "required_safe_s": float(sc.get("required_safe_follow_seconds", 1.0)),
            # NPC scripted behaviour params
            "cut_in_start_s": float(npc_beh.get("cut_in_start_time_s", 2.0)),
            "cut_in_duration_s": float(npc_beh.get("cut_in_duration_s", 2.0)),
            "cruise_kmh": float(npc_beh.get("cruise_speed_kmh", 18.0)),
            "brake_delay_s": float(npc_beh.get("brake_start_after_cut_in_s", 0.5)),
            "brake_strength": float(npc_beh.get("brake_strength", 0.75)),
            # state
            "cut_in_detected": False,
            "emergency_brake_started": False,
            "safe_follow_time": 0.0,
            "min_front_dist": float("inf"),
            "min_ttc": float("inf"),
            "success": False,
        }

    def _npc_control(self, npc, state, t) -> None:
        cut_start = state["cut_in_start_s"]
        cut_end = cut_start + state["cut_in_duration_s"]
        brake_start = cut_end + state["brake_delay_s"]
        cruise_kmh = state["cruise_kmh"]
        ego_world = npc.get_world()

        if t < cut_start:
            ctrl = carla.VehicleControl(throttle=0.35, steer=0.0, brake=0.0)
        elif t < cut_end:
            progress = (t - cut_start) / (cut_end - cut_start)
            steer = clip(0.45 * (1 - progress), -0.55, 0.55)
            ctrl = carla.VehicleControl(throttle=0.30, steer=steer, brake=0.0)
        elif t < brake_start:
            ctrl = carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0)
        else:
            ctrl = carla.VehicleControl(throttle=0.0, steer=0.0, brake=state["brake_strength"])

        npc.apply_control(ctrl)

    def update_state(self, ego, actors, state, obs, dt, cfg) -> Dict[str, Any]:
        if not actors:
            return state
        npc = actors[0]
        t = obs["timestamp"]
        self._npc_control(npc, state, t)

        ego_loc = ego.get_location()
        npc_loc = npc.get_location()
        dist = ego_loc.distance(npc_loc)

        ego_tf = ego.get_transform()
        fwd = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()
        dx = npc_loc.x - ego_loc.x
        dy = npc_loc.y - ego_loc.y
        forward_dist = dx * fwd.x + dy * fwd.y
        lateral_dist = abs(dx * right.x + dy * right.y)

        in_front_corridor = forward_dist > 0 and lateral_dist < state["front_half_w"]

        npc_spd = get_speed_kmh(npc)
        ego_spd = obs["speed_kmh"]
        rel_spd = (ego_spd - npc_spd) / 3.6
        ttc = forward_dist / rel_spd if rel_spd > 0.5 and forward_dist > 0 else float("inf")

        if in_front_corridor and forward_dist < state["detection_dist_m"]:
            if not state["cut_in_detected"]:
                state["cut_in_detected"] = True
            state["min_front_dist"] = min(state["min_front_dist"], forward_dist)
            state["min_ttc"] = min(state["min_ttc"], ttc)

            if forward_dist <= state["emergency_brake_dist_m"] or ttc <= state["emergency_ttc_s"]:
                state["emergency_brake_started"] = True

        if state["emergency_brake_started"] and forward_dist >= state["safe_follow_dist_m"] and ego_spd <= state["safe_speed_kmh"]:
            state["safe_follow_time"] += dt
        else:
            state["safe_follow_time"] = 0.0

        state["forward_dist"] = forward_dist
        state["npc_in_corridor"] = in_front_corridor

        if state["safe_follow_time"] >= state["required_safe_s"] and state["cut_in_detected"]:
            state["success"] = True

        return state

    def compute_control(self, ego, actors, state, obs, cfg) -> Tuple[float, float, float]:
        target_speed = float(cfg.get("controller", {}).get("target_speed_kmh", 30.0))
        lookahead = float(cfg.get("controller", {}).get("lookahead_m", 14.0))
        route_tracker: RouteTracker = obs.get("route_tracker")
        progress = obs["route_metrics"]["route_progress_m"]
        target_loc = route_tracker.point_at_progress(progress + lookahead) if route_tracker else ego.get_location()
        steer = compute_steer_to_location(ego, target_loc)
        speed = obs["speed_kmh"]
        fwd_dist = state.get("forward_dist", 999.0)
        in_corridor = state.get("npc_in_corridor", False)

        if in_corridor and fwd_dist <= 10.0:
            return 0.0, 0.95, steer
        if state["emergency_brake_started"]:
            if speed > 12.0:
                return 0.0, 0.60, steer
            if speed > 5.0:
                return 0.0, 0.30, steer
            return 0.0, 0.0, steer

        error = target_speed - speed
        if error > 8:
            throttle, brake = 0.55, 0.0
        elif error > 3:
            throttle, brake = 0.40, 0.0
        elif error < -5:
            throttle, brake = 0.0, 0.20
        else:
            throttle, brake = 0.25, 0.0

        return throttle, brake, steer

    def is_success(self, state, obs, cfg) -> bool:
        return state["success"]

    def extra_record(self, ego, actors, state, obs, cfg) -> Dict[str, Any]:
        npc_loc = actors[0].get_location() if actors else carla.Location()
        return {
            "cut_in_detected": state["cut_in_detected"],
            "emergency_brake_started": state["emergency_brake_started"],
            "min_front_vehicle_distance": state["min_front_dist"],
            "min_TTC": state["min_ttc"],
            "safe_follow_time": state["safe_follow_time"],
            "npc_x": npc_loc.x,
            "npc_y": npc_loc.y,
            "task_success": state["success"],
        }
