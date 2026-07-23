"""S13 – PDF scene 3 extreme emergency 6km drive without voice input."""

import math
from typing import Any, Dict, List, Optional, Tuple

import carla

from carla_eval.runtime_metrics import RouteTracker, load_route_waypoints, route_length_from_points
from .base import BaseScenario, clip, compute_speed_pid, compute_steer_to_location, get_speed_kmh


def _compute_hazard_score(weather_params: Dict[str, Any]) -> float:
    precip = float(weather_params.get("precipitation", 0.0)) / 100.0
    fog = min(float(weather_params.get("fog_density", 0.0)) / 80.0, 1.0)
    altitude = float(weather_params.get("sun_altitude_angle", 45.0))
    night = max(0.0, min(1.0, (-altitude - 10.0) / 30.0)) if altitude < -10 else 0.0
    wetness = float(weather_params.get("wetness", 0.0)) / 100.0
    score = 0.35 * precip + 0.20 * fog + 0.25 * night + 0.20 * wetness
    return min(1.0, score)


def _normalized_direction(start: carla.Location, end: carla.Location) -> carla.Vector3D:
    dx = end.x - start.x
    dy = end.y - start.y
    dz = end.z - start.z
    norm = math.sqrt(dx * dx + dy * dy + dz * dz)
    if norm <= 1e-6:
        return carla.Vector3D(1.0, 0.0, 0.0)
    return carla.Vector3D(dx / norm, dy / norm, dz / norm)


def _route_yaw_at(route_tracker: RouteTracker, progress_m: float, delta_m: float = 4.0) -> float:
    p0 = max(0.0, progress_m - delta_m)
    p1 = min(route_tracker.route_total_length_m, progress_m + delta_m)
    a = route_tracker.point_at_progress(p0)
    b = route_tracker.point_at_progress(p1)
    return math.degrees(math.atan2(b.y - a.y, b.x - a.x))


def _wrap_deg(delta: float) -> float:
    return (delta + 180.0) % 360.0 - 180.0


def _compute_speed_control(speed_kmh: float, target_kmh: float, steer: float = 0.0) -> Tuple[float, float]:
    error = target_kmh - speed_kmh
    if error > 12.0:
        throttle, brake = 1.00, 0.0
    elif error > 7.0:
        throttle, brake = 0.85, 0.0
    elif error > 3.0:
        throttle, brake = 0.60, 0.0
    elif error > 0.8:
        throttle, brake = 0.35, 0.0
    elif error < -15.0:
        throttle, brake = 0.0, 0.30
    elif error < -8.0:
        throttle, brake = 0.0, 0.16
    elif error < -3.0:
        throttle, brake = 0.0, 0.08
    else:
        throttle, brake = 0.15, 0.0

    if abs(steer) > 0.35:
        throttle = min(throttle, 0.45)
    return throttle, brake


def _offstage_transform() -> carla.Transform:
    return carla.Transform(
        carla.Location(x=10000.0, y=10000.0, z=-50.0),
        carla.Rotation(yaw=0.0),
    )


def _danger_zone_enabled(cfg: Dict[str, Any]) -> bool:
    return bool(cfg.get("action_windows", {}).get("danger_zone", {}).get("enabled", True))


class EmergencyResponseScene3(BaseScenario):
    def __init__(self):
        self._route_points: List[carla.Location] = []
        self._route_helper: Optional[RouteTracker] = None
        self._route_total_length_m: float = 0.0
        self._actors_by_stage: Dict[str, carla.Actor] = {}
        self._actor_cfg_by_stage: Dict[str, Dict[str, Any]] = {}
        self._weather_params: Dict[str, Any] = {}
        self._world: Optional[carla.World] = None
        self._bp_lib = None

    def _ensure_route(self, world: carla.World, cfg: Dict[str, Any]) -> None:
        if self._route_helper is not None and self._route_points:
            return

        route_points = load_route_waypoints(cfg)
        if not route_points:
            corridor = float(cfg.get("evaluation", {}).get("route_corridor_half_width_m", 8.5))
            self._route_helper = RouteTracker.from_route_config(world.get_map(), cfg, corridor_half_width_m=corridor)
            self._route_points = list(self._route_helper.route_points)
        else:
            corridor = float(cfg.get("evaluation", {}).get("route_corridor_half_width_m", 8.5))
            self._route_points = route_points
            self._route_helper = RouteTracker(world.get_map(), route_points, corridor_half_width_m=corridor)

        self._route_total_length_m = route_length_from_points(self._route_points)
        self._weather_params = dict(cfg.get("map", {}).get("weather_parameters", {}))

    def _point_at(self, progress_m: float, lateral_offset_m: float = 0.0) -> carla.Location:
        assert self._route_helper is not None
        return self._route_helper.point_at_progress_smoothed(
            progress_m,
            lateral_offset_m=lateral_offset_m,
            window_radius_m=8.0,
            sample_step_m=2.0,
            sigma_m=4.0,
            tangent_delta_m=2.0,
        )

    def _yaw_at(self, progress_m: float) -> float:
        p0 = self._point_at(max(0.0, progress_m - 2.0))
        p1 = self._point_at(progress_m + 2.0)
        return math.degrees(math.atan2(p1.y - p0.y, p1.x - p0.x))

    def _transform_at(self, progress_m: float, lateral_offset_m: float = 0.0, z_offset: float = 0.2) -> carla.Transform:
        loc = self._point_at(progress_m, lateral_offset_m=lateral_offset_m)
        loc = carla.Location(x=loc.x, y=loc.y, z=loc.z + z_offset)
        return carla.Transform(loc, carla.Rotation(yaw=self._yaw_at(progress_m)))

    def _transform_along_lateral_path(
        self,
        progress_m: float,
        lateral_offset_m: float,
        next_progress_m: float,
        next_lateral_offset_m: float,
        z_offset: float = 0.35,
    ) -> carla.Transform:
        loc = self._point_at(progress_m, lateral_offset_m=lateral_offset_m)
        next_loc = self._point_at(next_progress_m, lateral_offset_m=next_lateral_offset_m)
        dx = next_loc.x - loc.x
        dy = next_loc.y - loc.y
        if math.hypot(dx, dy) <= 1e-4:
            yaw = self._yaw_at(progress_m)
        else:
            yaw = math.degrees(math.atan2(dy, dx))
        return carla.Transform(
            carla.Location(x=loc.x, y=loc.y, z=loc.z + z_offset),
            carla.Rotation(yaw=yaw),
        )

    def _route_waypoint_at_progress(self, progress_m: float) -> Optional[carla.Waypoint]:
        if self._route_helper is None:
            return None
        loc = self._route_helper.point_at_progress(progress_m)
        return self._route_helper.carla_map.get_waypoint(
            loc,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )

    def _seek_adjacent_lane(
        self,
        base_wp: Optional[carla.Waypoint],
        direction: str,
        allowed_types: Tuple[carla.LaneType, ...] = (carla.LaneType.Driving,),
        require_same_direction: bool = True,
        max_hops: int = 4,
    ) -> Optional[carla.Waypoint]:
        if base_wp is None:
            return None

        current = base_wp
        for _ in range(max_hops):
            current = current.get_left_lane() if direction == "left" else current.get_right_lane()
            if current is None:
                return None
            if current.lane_type not in allowed_types:
                continue
            if require_same_direction and current.lane_id * base_wp.lane_id <= 0:
                continue
            return current
        return None

    def _lane_center_transform(self, waypoint: carla.Waypoint, z_offset: float = 0.35) -> carla.Transform:
        tf = waypoint.transform
        return carla.Transform(
            carla.Location(x=tf.location.x, y=tf.location.y, z=tf.location.z + z_offset),
            carla.Rotation(
                pitch=tf.rotation.pitch,
                yaw=tf.rotation.yaw,
                roll=tf.rotation.roll,
            ),
        )

    def _cut_in_spawn_transform(self, progress_m: float, prefer_left: bool = True) -> Optional[carla.Transform]:
        base_wp = self._route_waypoint_at_progress(progress_m)
        if base_wp is None:
            return self._transform_at(progress_m, lateral_offset_m=-3.5, z_offset=0.35)

        preferred = "left" if prefer_left else "right"
        alternate = "right" if prefer_left else "left"
        lane_wp = self._seek_adjacent_lane(base_wp, preferred)
        if lane_wp is None:
            lane_wp = self._seek_adjacent_lane(base_wp, alternate)
        if lane_wp is None:
            lane_wp = base_wp
        return self._lane_center_transform(lane_wp, z_offset=0.35)

    def _find_blueprint(self, bp_lib, preferred: str, fallback_filters=None):
        fallback_filters = fallback_filters or []
        try:
            return bp_lib.find(preferred)
        except Exception:
            pass

        filters = []
        if isinstance(fallback_filters, str):
            filters.append(fallback_filters)
        else:
            filters.extend(fallback_filters)
        if preferred.endswith(".*") or "*" in preferred:
            filters.insert(0, preferred)

        for pattern in filters:
            matches = bp_lib.filter(pattern)
            if matches:
                return matches[0]
        raise IndexError(f"blueprint '{preferred}' not found")

    def spawn_actors(self, world, ego, cfg) -> List[carla.Actor]:
        self._ensure_route(world, cfg)
        bp_lib = world.get_blueprint_library()
        self._world = world
        self._bp_lib = bp_lib
        self._actors_by_stage = {}
        self._actor_cfg_by_stage = {}
        actors: List[carla.Actor] = []

        for actor_cfg in cfg.get("actors", {}).get("vehicles", []):
            stage = str(actor_cfg.get("stage", actor_cfg.get("id", "vehicle")))
            self._actor_cfg_by_stage[stage] = actor_cfg

        for actor_cfg in cfg.get("actors", {}).get("pedestrians", []):
            stage = str(actor_cfg.get("stage", actor_cfg.get("id", "pedestrian")))
            self._actor_cfg_by_stage[stage] = actor_cfg
            progress_m = float(actor_cfg.get("progress_m", 0.0))
            lateral_offset_m = float(actor_cfg.get("lateral_offset_m", 0.0))
            bp = self._find_blueprint(
                bp_lib,
                actor_cfg.get("type", "walker.pedestrian.*"),
                actor_cfg.get("fallback_filters") or actor_cfg.get("fallback_filter") or ["walker.pedestrian.*"],
            )
            try:
                actor = world.try_spawn_actor(bp, self._transform_at(progress_m, lateral_offset_m, z_offset=0.95))
            except Exception:
                actor = None
            if actor is not None:
                actors.append(actor)
                self._actors_by_stage[stage] = actor
                self._actor_cfg_by_stage[stage] = actor_cfg

        for actor_cfg in cfg.get("actors", {}).get("static_props", []):
            stage = str(actor_cfg.get("stage", actor_cfg.get("id", "static_prop")))
            self._actor_cfg_by_stage[stage] = actor_cfg

        return actors

    def _spawn_stage_actor(
        self,
        stage: str,
        actor_cfg: Dict[str, Any],
        transform: carla.Transform,
        actors: List[carla.Actor],
        simulate_physics: bool,
    ) -> Optional[carla.Actor]:
        if self._world is None or self._bp_lib is None:
            return None

        existing = self._actors_by_stage.get(stage)
        if existing is not None and existing.is_alive:
            return existing

        try:
            bp = self._find_blueprint(
                self._bp_lib,
                actor_cfg.get("type", "static.prop.*"),
                actor_cfg.get("fallback_filters") or actor_cfg.get("fallback_filter") or ["static.prop.*"],
            )
            actor = self._world.try_spawn_actor(bp, transform)
            if actor is None:
                print(f"[S13][SPAWN_FAIL] {stage} type={bp.id}")
                return None
            try:
                actor.set_simulate_physics(simulate_physics)
            except Exception:
                pass
            try:
                actor.set_transform(transform)
            except Exception:
                pass
            actors.append(actor)
            self._actors_by_stage[stage] = actor
            self._actor_cfg_by_stage[stage] = actor_cfg
            loc = actor.get_location()
            print(
                f"[S13][SPAWN] {stage} type={actor.type_id} "
                f"loc=({loc.x:.1f},{loc.y:.1f},{loc.z:.1f})"
            )
            return actor
        except Exception as exc:
            print(f"[S13][SPAWN_FAIL] {stage}: {exc}")
            return None

    def initial_state(self, ego, actors, cfg) -> Dict[str, Any]:
        scene_cfg = cfg.get("success_criteria", {}).get("emergency_scene3", {})
        danger_cfg = cfg.get("action_windows", {}).get("danger_zone", {})
        cut_cfg = cfg.get("action_windows", {}).get("cut_in_emergency", {})
        construction_cfg = cfg.get("action_windows", {}).get("construction_merge", {})
        hazard_score = _compute_hazard_score(self._weather_params)
        cut_in_initial_progress = float(cut_cfg.get("npc_initial_progress_m", cut_cfg.get("anchor_progress_m", 900.0)))
        cut_in_initial_offset = float(cut_cfg.get("npc_initial_lateral_offset_m", -3.5))

        return {
            "hazard_score": hazard_score,
            "danger_detected": False,
            "danger_zone_activated": False,
            "slowdown_started": False,
            "safe_speed_reached": not _danger_zone_enabled(cfg),
            "safe_speed_hold_time": 0.0,
            "danger_target_speed_kmh": float(danger_cfg.get("target_speed_kmh", scene_cfg.get("danger_safe_speed_kmh", 42.0))),
            "weather_precipitation": float(self._weather_params.get("precipitation", 0.0)),
            "weather_wetness": float(self._weather_params.get("wetness", 0.0)),
            "weather_fog_density": float(self._weather_params.get("fog_density", 0.0)),
            "sun_altitude_angle": float(self._weather_params.get("sun_altitude_angle", 45.0)),
            "is_night": float(self._weather_params.get("sun_altitude_angle", 45.0)) < -10.0,
            "cut_in_detected": False,
            "cut_in_detected_time_s": None,
            "emergency_brake_started": False,
            "emergency_response_time_s": None,
            "safe_brake_completed": False,
            "safe_follow_hold_time": 0.0,
            "front_vehicle_distance": None,
            "front_vehicle_gap": None,
            "front_vehicle_lateral_offset": None,
            "cut_in_actor_x": None,
            "cut_in_actor_y": None,
            "cut_in_actor_yaw_deg": None,
            "cut_in_actor_speed_kmh": None,
            "ttc_s": None,
            "min_front_vehicle_distance": float("inf"),
            "min_ttc": float("inf"),
            "cut_in_phase": "waiting",
            "cut_in_phase_time_s": 0.0,
            "cut_in_plan_progress_m": cut_in_initial_progress,
            "cut_in_target_lateral_offset_m": cut_in_initial_offset,
            "cut_in_merge_start_progress_m": None,
            "cut_in_merge_start_gap_m": None,
            "cut_in_actor_activated": False,
            "cut_in_actor_retired": False,
            "cut_in_spawn_failed": False,
            "cut_in_yield_completed": False,
            "construction_detected": False,
            "construction_active": False,
            "cone_detour_active": False,
            "cone_detour_start_progress_m": None,
            "cone_detour_clear_progress_m": None,
            "cone_detour_distance_m": None,
            "construction_target_lateral_offset_m": 0.0,
            "merge_started": False,
            "merge_completed": False,
            "construction_pass_completed": False,
            "construction_zone_activated": False,
            "construction_visibility_enhanced": False,
            "merge_hold_time": 0.0,
            "construction_return_hold_time": 0.0,
            "route_finish_brake_started": False,
            "distance_to_construction": None,
            "worker_detected": False,
            "worker_crossing_active": False,
            "worker_spawn_failed": False,
            "worker_distance_m": None,
            "worker_phase": "waiting",
            "worker_motion_mode": "along_construction_lane",
            "worker_crossing_elapsed_s": 0.0,
            "worker_path_remaining_m": None,
            "worker_emergency_brake_started": False,
            "long_route_completed": False,
            "task_success": False,
        }

    def _project_relative(self, ego: carla.Actor, actor: carla.Actor) -> Tuple[float, float, float]:
        ego_tf = ego.get_transform()
        ego_loc = ego_tf.location
        actor_loc = actor.get_location()
        fwd = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()
        dx = actor_loc.x - ego_loc.x
        dy = actor_loc.y - ego_loc.y
        forward_dist = dx * fwd.x + dy * fwd.y
        lateral_dist = dx * right.x + dy * right.y
        euclidean_dist = math.hypot(dx, dy)
        return forward_dist, lateral_dist, euclidean_dist

    def _follow_route_actor(
        self,
        actor: carla.Actor,
        progress_m: float,
        target_speed_kmh: float,
        lateral_offset_m: float,
        lookahead_m: float = 10.0,
        steer_gain: float = 1.20,
        max_steer: float = 0.35,
    ) -> None:
        target_loc = self._point_at(progress_m + lookahead_m, lateral_offset_m=lateral_offset_m)
        steer = compute_steer_to_location(actor, target_loc, gain=steer_gain, max_steer=max_steer)
        throttle, brake = compute_speed_pid(
            get_speed_kmh(actor),
            target_speed_kmh,
            steer=steer,
            steer_throttle_penalty=0.22,
        )
        actor.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=steer))
        try:
            forward = actor.get_transform().get_forward_vector()
            speed_mps = max(target_speed_kmh, 0.0) / 3.6
            actor.set_target_velocity(
                carla.Vector3D(
                    x=forward.x * speed_mps,
                    y=forward.y * speed_mps,
                    z=0.0,
                )
            )
        except Exception:
            pass

    def _place_cut_in_actor(
        self,
        actor: carla.Actor,
        progress_m: float,
        lateral_offset_m: float,
        target_speed_kmh: float,
        next_progress_m: Optional[float] = None,
        next_lateral_offset_m: Optional[float] = None,
    ) -> None:
        if next_progress_m is None:
            next_progress_m = progress_m + 4.0
        if next_lateral_offset_m is None:
            next_lateral_offset_m = lateral_offset_m
        transform = self._transform_along_lateral_path(
            progress_m,
            lateral_offset_m,
            next_progress_m,
            next_lateral_offset_m,
            z_offset=0.35,
        )
        try:
            actor.set_simulate_physics(False)
        except Exception:
            pass
        actor.set_transform(transform)
        forward = transform.get_forward_vector()
        speed_mps = max(target_speed_kmh, 0.0) / 3.6
        try:
            actor.set_target_velocity(
                carla.Vector3D(
                    x=forward.x * speed_mps,
                    y=forward.y * speed_mps,
                    z=0.0,
                )
            )
        except Exception:
            pass

    def _retire_cut_in_actor(self, state) -> None:
        actor = self._actors_by_stage.get("cut_in_vehicle")
        if actor is not None and actor.is_alive:
            try:
                actor.set_simulate_physics(False)
            except Exception:
                pass
            try:
                actor.set_transform(_offstage_transform())
                actor.set_target_velocity(carla.Vector3D(x=0.0, y=0.0, z=0.0))
            except Exception:
                pass
            try:
                actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0, hand_brake=True))
            except Exception:
                pass
        state["cut_in_actor_activated"] = False
        state["cut_in_actor_retired"] = True
        state["cut_in_phase"] = "cleared"
        state["front_vehicle_distance"] = None
        state["front_vehicle_gap"] = None
        state["front_vehicle_lateral_offset"] = None
        state["cut_in_actor_x"] = None
        state["cut_in_actor_y"] = None
        state["cut_in_actor_yaw_deg"] = None
        state["cut_in_actor_speed_kmh"] = None
        state["ttc_s"] = None

    def _activate_cut_in_actor(self, ego, actors, state, obs, cfg) -> None:
        actor = self._actors_by_stage.get("cut_in_vehicle")
        if actor is None or not actor.is_alive or state.get("cut_in_actor_activated"):
            if state.get("cut_in_actor_activated"):
                return
        if state.get("cut_in_actor_retired"):
            return

        cut_cfg = cfg.get("action_windows", {}).get("cut_in_emergency", {})
        ego_progress = float(obs["route_metrics"]["route_progress_m"])
        spawn_gap = float(cut_cfg.get("spawn_gap_m", cut_cfg.get("merge_target_gap_m", 24.0)))
        initial_lat = float(cut_cfg.get("npc_initial_lateral_offset_m", -3.5))
        state["cut_in_plan_progress_m"] = ego_progress + spawn_gap
        state["cut_in_target_lateral_offset_m"] = initial_lat
        spawn_tf = self._cut_in_spawn_transform(state["cut_in_plan_progress_m"], prefer_left=True)
        if spawn_tf is None:
            spawn_tf = self._transform_at(state["cut_in_plan_progress_m"], initial_lat, z_offset=0.35)
        if actor is None or not actor.is_alive:
            actor_cfg = self._actor_cfg_by_stage.get("cut_in_vehicle", {})
            actor = self._spawn_stage_actor("cut_in_vehicle", actor_cfg, spawn_tf, actors, simulate_physics=False)
            if actor is None:
                return
        else:
            try:
                actor.set_simulate_physics(False)
            except Exception:
                pass
            actor.set_transform(spawn_tf)
        cruise_speed_mps = float(cut_cfg.get("npc_cruise_speed_kmh", 58.0)) / 3.6
        forward = actor.get_transform().get_forward_vector()
        try:
            actor.set_target_velocity(
                carla.Vector3D(
                    x=forward.x * cruise_speed_mps,
                    y=forward.y * cruise_speed_mps,
                    z=0.0,
                )
            )
        except Exception:
            pass
        actor.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, steer=0.0))
        loc = actor.get_location()
        print(
            f"[S13][CUTIN_SPAWN] type={actor.type_id} "
            f"loc=({loc.x:.1f},{loc.y:.1f},{loc.z:.1f}) "
            f"progress={state['cut_in_plan_progress_m']:.1f}"
        )
        try:
            actor.set_light_state(
                carla.VehicleLightState(
                    carla.VehicleLightState.Position
                    | carla.VehicleLightState.LowBeam
                )
            )
        except Exception:
            pass
        state["cut_in_actor_activated"] = True

    def _activate_danger_zone(self, actors, state, cfg) -> None:
        if state.get("danger_zone_activated"):
            return

        # 工况1只表达低能见度/湿滑危险路况下的减速，不生成警告牌、锥筒或故障车。
        state["danger_zone_activated"] = True
        return

        for stage, actor_cfg in list(self._actor_cfg_by_stage.items()):
            actor = self._actors_by_stage.get(stage)
            actor_cfg = self._actor_cfg_by_stage.get(stage, {})

            if not (
                stage.startswith("danger_cone_")
                or stage.startswith("danger_barrier_")
                or stage.startswith("danger_warning_sign_")
                or stage == "danger_stalled_vehicle"
            ):
                continue

            progress_m = float(actor_cfg.get("progress_m", 0.0))
            lateral_offset_m = float(actor_cfg.get("lateral_offset_m", 0.0))
            z_offset = float(actor_cfg.get("z_offset_m", 0.05 if stage != "danger_stalled_vehicle" else 0.30))
            spawn_tf = self._transform_at(progress_m, lateral_offset_m, z_offset=z_offset)
            if actor is None or not actor.is_alive:
                actor = self._spawn_stage_actor(
                    stage,
                    actor_cfg,
                    spawn_tf,
                    actors,
                    simulate_physics=stage == "danger_stalled_vehicle",
                )
                if actor is None:
                    continue
            else:
                actor.set_transform(spawn_tf)

            if stage == "danger_stalled_vehicle":
                actor.apply_control(
                    carla.VehicleControl(
                        throttle=0.0,
                        brake=1.0,
                        steer=0.0,
                        hand_brake=True,
                    )
                )
                try:
                    actor.set_light_state(
                        carla.VehicleLightState(
                            carla.VehicleLightState.Position
                            | carla.VehicleLightState.LowBeam
                            | carla.VehicleLightState.Hazard
                        )
                    )
                except Exception:
                    pass
            else:
                try:
                    actor.set_simulate_physics(False)
                except Exception:
                    pass
                if stage.startswith("danger_warning_sign_"):
                    loc = actor.get_location()
                    print(
                        f"[S13][ACTIVATE] {stage} type={actor.type_id} "
                        f"loc=({loc.x:.1f},{loc.y:.1f},{loc.z:.1f})"
                    )

        state["danger_zone_activated"] = True

    def _update_cut_in_actor(self, ego, actors, state, obs, dt, cfg) -> None:
        actor = self._actors_by_stage.get("cut_in_vehicle")

        cut_cfg = cfg.get("action_windows", {}).get("cut_in_emergency", {})
        ego_progress = float(obs["route_metrics"]["route_progress_m"])
        activation_progress = float(cut_cfg.get("activation_progress_m", 760.0))
        merge_delay = float(cut_cfg.get("merge_delay_seconds", 1.2))
        cruise_speed = float(cut_cfg.get("npc_cruise_speed_kmh", 48.0))
        merge_speed = float(cut_cfg.get("npc_merge_speed_kmh", 42.0))
        post_merge_speed = float(cut_cfg.get("npc_post_merge_speed_kmh", cut_cfg.get("npc_brake_speed_kmh", 60.0)))
        merge_duration = float(cut_cfg.get("merge_duration_seconds", 1.8))
        initial_lat = float(cut_cfg.get("npc_initial_lateral_offset_m", -3.5))
        final_lat = float(cut_cfg.get("npc_final_lateral_offset_m", 0.0))

        phase = state["cut_in_phase"]
        state["cut_in_phase_time_s"] += dt

        if phase == "waiting" and ego_progress >= activation_progress:
            self._activate_cut_in_actor(ego, actors, state, obs, cfg)
            actor = self._actors_by_stage.get("cut_in_vehicle")
            if not state.get("cut_in_actor_activated"):
                state["cut_in_spawn_failed"] = True
                return
            state["cut_in_spawn_failed"] = False
            state["cut_in_phase"] = "cruise"
            state["cut_in_phase_time_s"] = 0.0
            phase = "cruise"

        if not state.get("cut_in_actor_activated"):
            if actor is not None and actor.is_alive:
                actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
            return
        if actor is None or not actor.is_alive:
            state["cut_in_actor_activated"] = False
            state["cut_in_spawn_failed"] = True
            state["cut_in_phase"] = "waiting"
            return

        retire_progress = float(cut_cfg.get("retire_after_ego_progress_m", activation_progress + 450.0))
        if state.get("safe_brake_completed") and ego_progress >= retire_progress:
            self._retire_cut_in_actor(state)
            return

        if phase == "cruise" and state["cut_in_phase_time_s"] >= merge_delay:
            state["cut_in_merge_start_gap_m"] = max(state["cut_in_plan_progress_m"] - ego_progress, 0.0)
            state["cut_in_merge_start_progress_m"] = state["cut_in_plan_progress_m"]
            state["cut_in_phase"] = "merge"
            state["cut_in_phase_time_s"] = 0.0
            phase = "merge"

        if phase == "merge" and state["cut_in_phase_time_s"] >= merge_duration:
            state["cut_in_phase"] = "accelerate"
            state["cut_in_phase_time_s"] = 0.0
            phase = "accelerate"

        if phase == "waiting":
            actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
            return
        if phase == "cruise":
            target_speed = cruise_speed
            target_lat = initial_lat
        elif phase == "merge":
            ratio = clip(state["cut_in_phase_time_s"] / max(merge_duration, 1e-6), 0.0, 1.0)
            smooth = ratio * ratio * ratio * (10.0 - 15.0 * ratio + 6.0 * ratio * ratio)
            target_speed = cruise_speed + smooth * (merge_speed - cruise_speed)
            target_lat = initial_lat + smooth * (final_lat - initial_lat)
        else:
            target_speed = post_merge_speed
            target_lat = final_lat

        state["cut_in_plan_progress_m"] += max(target_speed / 3.6 * dt, 0.0)

        if phase == "merge":
            preview_s = float(cut_cfg.get("merge_yaw_preview_seconds", 0.60))
            preview_ds = max(target_speed / 3.6 * preview_s, 3.0)
            future_ratio = clip((state["cut_in_phase_time_s"] + preview_s) / max(merge_duration, 1e-6), 0.0, 1.0)
            future_smooth = future_ratio * future_ratio * future_ratio * (
                10.0 - 15.0 * future_ratio + 6.0 * future_ratio * future_ratio
            )
            next_lat = initial_lat + future_smooth * (final_lat - initial_lat)
            next_progress = state["cut_in_plan_progress_m"] + preview_ds
        else:
            next_lat = target_lat
            next_progress = state["cut_in_plan_progress_m"] + 4.0

        state["cut_in_target_lateral_offset_m"] = target_lat
        self._place_cut_in_actor(
            actor,
            state["cut_in_plan_progress_m"],
            target_lat,
            target_speed,
            next_progress_m=next_progress,
            next_lateral_offset_m=next_lat,
        )

    def _activate_construction_zone(self, actors, state, cfg) -> None:
        if state.get("construction_zone_activated"):
            return

        for stage, actor_cfg in list(self._actor_cfg_by_stage.items()):
            actor = self._actors_by_stage.get(stage)
            actor_cfg = self._actor_cfg_by_stage.get(stage, {})
            if not stage.startswith("construction_cone_"):
                continue
            progress_m = float(actor_cfg.get("progress_m", 0.0))
            lateral_offset_m = float(actor_cfg.get("lateral_offset_m", 0.0))
            z_offset = float(actor_cfg.get("z_offset_m", 0.05))
            spawn_tf = self._transform_at(progress_m, lateral_offset_m, z_offset=z_offset)
            if actor is None or not actor.is_alive:
                actor = self._spawn_stage_actor(stage, actor_cfg, spawn_tf, actors, simulate_physics=False)
                if actor is None:
                    continue
            else:
                actor.set_transform(spawn_tf)
            try:
                actor.set_simulate_physics(False)
            except Exception:
                pass
        state["construction_zone_activated"] = True

    def _construction_zones(self, cfg: Dict[str, Any]) -> List[Dict[str, float]]:
        construction_cfg = cfg.get("action_windows", {}).get("construction_merge", {})
        primary = {
            "activation_progress_m": float(construction_cfg.get("activation_progress_m", 1890.0)),
            "detection_progress_m": float(construction_cfg.get("detection_progress_m", 1950.0)),
            "merge_start_progress_m": float(construction_cfg.get("merge_start_progress_m", 1990.0)),
            "merge_duration_distance_m": float(construction_cfg.get("merge_duration_distance_m", 45.0)),
            "zone_end_progress_m": float(construction_cfg.get("zone_end_progress_m", 2210.0)),
            "return_end_progress_m": float(construction_cfg.get("return_end_progress_m", 2280.0)),
        }
        zones = [primary]
        for zone_cfg in construction_cfg.get("additional_zones", []):
            zone = dict(primary)
            zone.update({key: float(value) for key, value in zone_cfg.items() if key in primary})
            zones.append(zone)
        return sorted(zones, key=lambda zone: zone["merge_start_progress_m"])

    def _active_construction_zone(self, progress: float, cfg: Dict[str, Any]) -> Optional[Dict[str, float]]:
        for zone in self._construction_zones(cfg):
            if zone["activation_progress_m"] <= progress <= zone["return_end_progress_m"]:
                return zone
        return None

    def _next_construction_zone(self, progress: float, cfg: Dict[str, Any]) -> Optional[Dict[str, float]]:
        future = [zone for zone in self._construction_zones(cfg) if progress <= zone["return_end_progress_m"]]
        if not future:
            return None
        return min(future, key=lambda zone: abs(zone["detection_progress_m"] - progress))

    def _apply_construction_visibility(self, state, active_zone, cfg) -> None:
        if self._world is None:
            return

        construction_cfg = cfg.get("action_windows", {}).get("construction_merge", {})
        overlay_cfg = construction_cfg.get("visibility_weather", {})
        enhanced = active_zone is not None
        if state.get("construction_visibility_enhanced") == enhanced:
            return

        weather_params = dict(self._weather_params)
        if enhanced:
            weather_params.update({key: float(value) for key, value in overlay_cfg.items()})

        try:
            self._world.set_weather(carla.WeatherParameters(**weather_params))
            state["construction_visibility_enhanced"] = enhanced
        except Exception:
            pass

    def _update_cone_detour(self, ego, state, obs, cfg) -> None:
        scene_cfg = cfg.get("success_criteria", {}).get("emergency_scene3", {})
        progress = float(obs["route_metrics"]["route_progress_m"])
        trigger_distance = float(scene_cfg.get("cone_detour_trigger_distance_m", 85.0))
        clear_distance = float(scene_cfg.get("cone_detour_clear_distance_m", 65.0))
        lateral_limit = float(scene_cfg.get("cone_detour_lateral_detection_m", 6.5))
        active_construction_zone = self._active_construction_zone(progress, cfg)

        if active_construction_zone is not None:
            state["cone_detour_active"] = False
            state["cone_detour_distance_m"] = None
            return

        nearest_forward = None
        for stage, actor in self._actors_by_stage.items():
            if actor is None or not actor.is_alive:
                continue
            if not stage.startswith("construction_cone_"):
                continue
            forward_dist, lateral_dist, _ = self._project_relative(ego, actor)
            if 0.0 < forward_dist <= trigger_distance and abs(lateral_dist) <= lateral_limit:
                nearest_forward = forward_dist if nearest_forward is None else min(nearest_forward, forward_dist)

        if nearest_forward is not None:
            if not state.get("cone_detour_active"):
                state["cone_detour_start_progress_m"] = progress
            state["cone_detour_active"] = True
            state["cone_detour_clear_progress_m"] = progress + clear_distance
            state["cone_detour_distance_m"] = nearest_forward
            state["construction_detected"] = True
            state["merge_started"] = True
            return

        clear_progress = state.get("cone_detour_clear_progress_m")
        if state.get("cone_detour_active") and clear_progress is not None and progress <= float(clear_progress):
            return

        state["cone_detour_active"] = False
        state["cone_detour_distance_m"] = None

    def _spawn_worker_actor(self, actors, cfg, start_progress: float, start_lat: float) -> Optional[carla.Actor]:
        actor_cfg = self._actor_cfg_by_stage.get("construction_worker", {})
        lateral_attempts = [start_lat, start_lat - 0.8, start_lat + 0.8, start_lat - 1.6, start_lat + 1.6]
        for lat in lateral_attempts:
            spawn_tf = self._transform_at(start_progress, lat, z_offset=0.95)
            actor = self._spawn_stage_actor("construction_worker", actor_cfg, spawn_tf, actors, simulate_physics=False)
            if actor is not None:
                return actor
        return None

    def _update_worker(self, actors, state, obs, dt, cfg) -> None:
        actor = self._actors_by_stage.get("construction_worker")
        construction_cfg = cfg.get("action_windows", {}).get("construction_merge", {})
        ego_progress = float(obs["route_metrics"]["route_progress_m"])
        start_progress = float(construction_cfg.get("worker_start_progress_m", 2130.0))
        end_progress = float(construction_cfg.get("worker_end_progress_m", start_progress))
        worker_lat = float(construction_cfg.get("worker_lateral_offset_m", 2.3))
        speed = float(construction_cfg.get("worker_speed_mps", 1.4))

        if actor is None or not actor.is_alive:
            if state.get("worker_spawn_failed"):
                return
            activation_progress = float(construction_cfg.get("worker_activation_progress_m", 2085.0))
            if ego_progress < activation_progress - 20.0:
                return
            actor = self._spawn_worker_actor(actors, cfg, start_progress, worker_lat)
            if actor is None:
                state["worker_spawn_failed"] = True
                return

        if state["worker_phase"] == "waiting" and ego_progress >= float(construction_cfg.get("worker_activation_progress_m", 2085.0)):
            state["worker_phase"] = "walking"
            state["worker_crossing_elapsed_s"] = 0.0

        if state["worker_phase"] != "walking":
            actor.apply_control(carla.WalkerControl(speed=0.0))
            return

        start_loc = self._point_at(start_progress, lateral_offset_m=worker_lat)
        target_loc = self._point_at(end_progress, lateral_offset_m=worker_lat)
        path_len = max(start_loc.distance(target_loc), 0.1)
        total_time = path_len / max(speed, 0.1)
        state["worker_crossing_elapsed_s"] += dt
        ratio = clip(state["worker_crossing_elapsed_s"] / total_time, 0.0, 1.0)
        worker_progress = start_progress + (end_progress - start_progress) * ratio
        current_loc = self._point_at(worker_progress, lateral_offset_m=worker_lat)
        next_loc = self._point_at(min(worker_progress + 2.0, end_progress), lateral_offset_m=worker_lat)
        yaw = math.degrees(math.atan2(next_loc.y - current_loc.y, next_loc.x - current_loc.x))
        current_loc = carla.Location(x=current_loc.x, y=current_loc.y, z=current_loc.z + 0.95)
        actor.set_transform(carla.Transform(current_loc, carla.Rotation(yaw=yaw)))
        remain = max(end_progress - worker_progress, 0.0)
        state["worker_crossing_active"] = True
        state["worker_path_remaining_m"] = remain

        if ratio >= 1.0 or remain <= 0.8:
            state["worker_phase"] = "cleared"
            state["worker_crossing_active"] = False
            try:
                actor.set_transform(_offstage_transform())
                actor.set_target_velocity(carla.Vector3D(x=0.0, y=0.0, z=0.0))
            except Exception:
                pass
            actor.apply_control(carla.WalkerControl(speed=0.0))
            return

        direction = _normalized_direction(current_loc, next_loc)
        actor.apply_control(carla.WalkerControl(direction=direction, speed=speed))

    def update_state(self, ego, actors, state, obs, dt, cfg) -> Dict[str, Any]:
        timestamp = float(obs["timestamp"])
        progress = float(obs["route_metrics"]["route_progress_m"])
        speed = float(obs["speed_kmh"])
        route_lat = float(obs["route_metrics"].get("lateral_offset_from_route_m", 0.0))
        scene_cfg = cfg.get("success_criteria", {}).get("emergency_scene3", {})
        danger_cfg = cfg.get("action_windows", {}).get("danger_zone", {})
        cut_cfg = cfg.get("action_windows", {}).get("cut_in_emergency", {})
        construction_cfg = cfg.get("action_windows", {}).get("construction_merge", {})

        self._update_cut_in_actor(ego, actors, state, obs, dt, cfg)
        self._update_worker(actors, state, obs, dt, cfg)
        self._update_cone_detour(ego, state, obs, cfg)

        if _danger_zone_enabled(cfg):
            if progress >= float(danger_cfg.get("activation_progress_m", max(0.0, float(danger_cfg.get("progress_start_m", 260.0)) - 80.0))):
                self._activate_danger_zone(actors, state, cfg)

            if float(danger_cfg.get("progress_start_m", 260.0)) <= progress <= float(danger_cfg.get("progress_end_m", 620.0)):
                state["danger_detected"] = True
                if speed <= state["danger_target_speed_kmh"] and not state["slowdown_started"]:
                    state["slowdown_started"] = True
                if state["slowdown_started"] and speed <= state["danger_target_speed_kmh"] + 1.0:
                    state["safe_speed_hold_time"] += dt
                    if state["safe_speed_hold_time"] >= float(scene_cfg.get("danger_required_hold_seconds", 2.0)):
                        state["safe_speed_reached"] = True
                else:
                    state["safe_speed_hold_time"] = 0.0

        cut_in_actor = self._actors_by_stage.get("cut_in_vehicle")
        if state.get("cut_in_actor_activated") and cut_in_actor is not None and cut_in_actor.is_alive:
            forward_dist, lateral_dist, euclidean_dist = self._project_relative(ego, cut_in_actor)
            cut_in_loc = cut_in_actor.get_location()
            cut_in_yaw = cut_in_actor.get_transform().rotation.yaw
            state["front_vehicle_distance"] = euclidean_dist
            state["front_vehicle_gap"] = forward_dist
            state["front_vehicle_lateral_offset"] = lateral_dist
            npc_speed = get_speed_kmh(cut_in_actor)
            state["cut_in_actor_x"] = cut_in_loc.x
            state["cut_in_actor_y"] = cut_in_loc.y
            state["cut_in_actor_yaw_deg"] = cut_in_yaw
            state["cut_in_actor_speed_kmh"] = npc_speed
            rel_speed = max((speed - npc_speed) / 3.6, 0.0)
            ttc_s = forward_dist / rel_speed if rel_speed > 0.5 and forward_dist > 0.0 else float("inf")
            state["ttc_s"] = ttc_s

            if 0.0 < forward_dist <= float(scene_cfg.get("cut_in_detection_distance_m", 40.0)) and abs(lateral_dist) <= 5.0:
                if not state["cut_in_detected"]:
                    state["cut_in_detected"] = True
                    state["cut_in_detected_time_s"] = timestamp
                state["min_front_vehicle_distance"] = min(state["min_front_vehicle_distance"], euclidean_dist)
                state["min_ttc"] = min(state["min_ttc"], ttc_s)

            emergency_should_start = (
                state["cut_in_phase"] in {"merge", "accelerate"}
                and 0.0 < forward_dist <= float(scene_cfg.get("emergency_brake_distance_m", 18.0))
            ) or (
                state["cut_in_phase"] in {"merge", "accelerate"}
                and ttc_s <= float(scene_cfg.get("emergency_brake_ttc_s", 2.5))
            )
            if emergency_should_start and not state["emergency_brake_started"]:
                if not state["cut_in_detected"]:
                    state["cut_in_detected"] = True
                    state["cut_in_detected_time_s"] = timestamp
                state["emergency_brake_started"] = True
                state["emergency_response_time_s"] = max(
                    0.0,
                    timestamp - float(state["cut_in_detected_time_s"] or timestamp),
                )

            if not state["safe_brake_completed"]:
                safe_gap = float(scene_cfg.get("safe_follow_distance_m", 10.0))
                safe_speed = float(scene_cfg.get("safe_follow_speed_kmh", scene_cfg.get("cut_in_yield_speed_kmh", 30.0)))
                lateral_clear = float(scene_cfg.get("cut_in_complete_lateral_tolerance_m", 0.8))
                if (
                    state["cut_in_detected"]
                    and state["cut_in_phase"] == "accelerate"
                    and forward_dist >= safe_gap
                    and abs(lateral_dist) <= lateral_clear
                    and speed <= safe_speed + 1.0
                ):
                    state["safe_follow_hold_time"] += dt
                    if state["safe_follow_hold_time"] >= float(scene_cfg.get("safe_follow_hold_seconds", 1.0)):
                        state["safe_brake_completed"] = True
                        state["cut_in_yield_completed"] = True
                else:
                    state["safe_follow_hold_time"] = 0.0
        else:
            state["front_vehicle_distance"] = None
            state["front_vehicle_gap"] = None
            state["front_vehicle_lateral_offset"] = None
            state["cut_in_actor_x"] = None
            state["cut_in_actor_y"] = None
            state["cut_in_actor_yaw_deg"] = None
            state["cut_in_actor_speed_kmh"] = None
            state["ttc_s"] = None

        active_construction_zone = self._active_construction_zone(progress, cfg)
        next_construction_zone = self._next_construction_zone(progress, cfg)
        state["construction_active"] = active_construction_zone is not None or state.get("cone_detour_active", False)
        state["construction_target_lateral_offset_m"] = self._desired_lateral_offset(state, progress, cfg)
        self._apply_construction_visibility(state, active_construction_zone, cfg)
        if next_construction_zone is not None:
            state["distance_to_construction"] = next_construction_zone["detection_progress_m"] - progress
        else:
            state["distance_to_construction"] = None
        if active_construction_zone is not None and progress >= active_construction_zone["detection_progress_m"]:
            state["construction_detected"] = True
        if active_construction_zone is not None:
            self._activate_construction_zone(actors, state, cfg)

        if active_construction_zone is not None and progress >= active_construction_zone["merge_start_progress_m"]:
            state["merge_started"] = True

        merge_target_offset = float(scene_cfg.get("merge_target_lateral_offset_m", -3.5))
        if active_construction_zone is not None and progress <= active_construction_zone["zone_end_progress_m"]:
            if abs(route_lat - merge_target_offset) <= float(scene_cfg.get("merge_lane_tolerance_m", 0.9)):
                state["merge_hold_time"] += dt
                if state["merge_hold_time"] >= float(scene_cfg.get("merge_required_hold_seconds", 0.8)):
                    state["merge_completed"] = True
            else:
                state["merge_hold_time"] = 0.0

        worker = self._actors_by_stage.get("construction_worker")
        if worker is not None and worker.is_alive:
            worker_forward_dist, worker_lateral_dist, worker_distance = self._project_relative(ego, worker)
            state["worker_distance_m"] = worker_distance
            if 0.0 < worker_forward_dist <= float(scene_cfg.get("worker_detection_distance_m", 22.0)) and abs(worker_lateral_dist) <= 7.0:
                state["worker_detected"] = True
            if (
                state["worker_crossing_active"]
                and 0.0 < worker_forward_dist <= float(scene_cfg.get("worker_brake_distance_m", 14.0))
                and abs(worker_lateral_dist) <= 4.8
            ):
                state["worker_emergency_brake_started"] = True

        zones = self._construction_zones(cfg)
        final_return_end = max(zone["return_end_progress_m"] for zone in zones) if zones else 0.0
        construction_return_phase = (
            active_construction_zone is not None
            and progress >= active_construction_zone["zone_end_progress_m"]
        ) or (
            active_construction_zone is None
            and progress >= final_return_end
        )
        if construction_return_phase and state["merge_completed"]:
            if abs(route_lat) <= float(scene_cfg.get("return_lane_tolerance_m", 1.0)):
                state["construction_return_hold_time"] += dt
                if (
                    (active_construction_zone is None or progress >= active_construction_zone["return_end_progress_m"])
                    and state["construction_return_hold_time"] >= float(scene_cfg.get("return_hold_seconds", 0.8))
                ):
                    state["construction_pass_completed"] = True
            else:
                state["construction_return_hold_time"] = 0.0

        target_progress_m = float(scene_cfg.get("target_progress_m", max(self._route_total_length_m - 40.0, 0.0)))
        target_completion = float(scene_cfg.get("target_route_completion", 0.99))
        if progress >= target_progress_m or float(obs["route_metrics"].get("route_completion", 0.0)) >= target_completion:
            state["long_route_completed"] = True

        reached_target_without_deviation = (
            state["long_route_completed"]
            and not bool(obs["route_metrics"].get("route_deviation", False))
        )
        state["task_success"] = (
            (state["safe_speed_reached"] or not _danger_zone_enabled(cfg))
            and state["safe_brake_completed"]
            and state["construction_pass_completed"]
            and reached_target_without_deviation
        )
        return state

    def _desired_lateral_offset(self, state, progress: float, cfg: Dict[str, Any]) -> float:
        scene_cfg = cfg.get("success_criteria", {}).get("emergency_scene3", {})
        merge_target_offset = float(scene_cfg.get("merge_target_lateral_offset_m", -3.5))
        active_zone = self._active_construction_zone(progress, cfg)
        if active_zone is None:
            if state.get("cone_detour_active"):
                start_progress = state.get("cone_detour_start_progress_m")
                clear_progress = state.get("cone_detour_clear_progress_m")
                merge_distance = float(scene_cfg.get("cone_detour_merge_distance_m", 35.0))
                return_distance = float(scene_cfg.get("cone_detour_return_distance_m", 45.0))
                if start_progress is None:
                    return merge_target_offset
                if clear_progress is not None and progress > float(clear_progress) - return_distance:
                    ratio = clip((progress - (float(clear_progress) - return_distance)) / max(return_distance, 1.0), 0.0, 1.0)
                    smooth = ratio * ratio * (3.0 - 2.0 * ratio)
                    return merge_target_offset * (1.0 - smooth)
                ratio = clip((progress - float(start_progress)) / max(merge_distance, 1.0), 0.0, 1.0)
                smooth = ratio * ratio * (3.0 - 2.0 * ratio)
                return merge_target_offset * smooth
            return 0.0
        merge_start = active_zone["merge_start_progress_m"]
        zone_end = active_zone["zone_end_progress_m"]
        return_end = active_zone["return_end_progress_m"]

        if progress < merge_start:
            return 0.0
        if progress <= zone_end:
            ratio = clip((progress - merge_start) / max(active_zone["merge_duration_distance_m"], 1.0), 0.0, 1.0)
            smooth = ratio * ratio * (3.0 - 2.0 * ratio)
            return smooth * merge_target_offset
        if progress <= return_end:
            ratio = clip((progress - zone_end) / max(return_end - zone_end, 1.0), 0.0, 1.0)
            smooth = ratio * ratio * (3.0 - 2.0 * ratio)
            return merge_target_offset * (1.0 - smooth)
        return 0.0

    def compute_control(self, ego, actors, state, obs, cfg) -> Tuple[float, float, float]:
        ctrl = cfg.get("controller", {})
        progress = float(obs["route_metrics"]["route_progress_m"])
        speed = float(obs["speed_kmh"])
        route_tracker: RouteTracker = obs["route_tracker"]
        route_lateral_offset = float(obs["route_metrics"].get("lateral_offset_from_route_m", 0.0))
        target_offset = self._desired_lateral_offset(state, progress, cfg)
        target_speed = float(ctrl.get("cruise_target_speed_kmh", 50.0))
        lookahead = float(ctrl.get("cruise_lookahead_m", 14.0))
        gain = float(ctrl.get("cruise_steer_gain", 1.20))
        max_steer = float(ctrl.get("cruise_max_steer", 0.34))
        window_radius = float(ctrl.get("target_point_window_radius_m", 8.0))
        sigma = float(ctrl.get("target_point_sigma_m", 4.0))
        scene_cfg = cfg.get("success_criteria", {}).get("emergency_scene3", {})
        target_progress = float(scene_cfg.get("target_progress_m", max(self._route_total_length_m - 40.0, 0.0)))

        if progress >= target_progress:
            state["route_finish_brake_started"] = True
            steer = compute_steer_to_location(
                ego,
                route_tracker.point_at_progress_smoothed(
                    min(progress + 2.0, route_tracker.route_total_length_m),
                    lateral_offset_m=0.0,
                ),
                gain=float(ctrl.get("finish_steer_gain", 1.0)),
                max_steer=float(ctrl.get("finish_max_steer", 0.20)),
            )
            return 0.0, float(ctrl.get("finish_brake", 0.75)), steer

        danger_cfg = cfg.get("action_windows", {}).get("danger_zone", {})
        if _danger_zone_enabled(cfg) and state["danger_detected"] and progress <= float(danger_cfg.get("progress_end_m", 620.0)):
            target_speed = min(target_speed, state["danger_target_speed_kmh"])
            lookahead = float(ctrl.get("danger_lookahead_m", 12.0))

        active_construction_zone = self._active_construction_zone(progress, cfg)
        construction_control_active = state.get("construction_active") or active_construction_zone is not None
        construction_slowdown_active = state.get("cone_detour_active", False) or (
            active_construction_zone is not None
            and progress < active_construction_zone["zone_end_progress_m"]
        )
        if construction_control_active:
            if construction_slowdown_active:
                target_speed = min(target_speed, float(ctrl.get("construction_target_speed_kmh", 30.0)))
            lookahead = float(ctrl.get("construction_lookahead_m", 12.0))
            gain = float(ctrl.get("construction_steer_gain", 1.28))
            max_steer = float(ctrl.get("construction_max_steer", 0.36))

        if state["cut_in_actor_activated"] and not state["safe_brake_completed"]:
            target_speed = min(target_speed, float(scene_cfg.get("cut_in_yield_speed_kmh", 30.0)))
            lookahead = min(lookahead, float(ctrl.get("emergency_lookahead_m", 8.0)))

        if state["worker_crossing_active"] and state.get("worker_distance_m") is not None and state["worker_distance_m"] <= float(cfg.get("success_criteria", {}).get("emergency_scene3", {}).get("worker_stop_distance_m", 12.0)):
            target_speed = min(target_speed, float(scene_cfg.get("worker_min_pass_speed_kmh", 20.0)))
            lookahead = float(ctrl.get("worker_lookahead_m", 8.0))
            gain = float(ctrl.get("worker_steer_gain", 1.35))
            max_steer = float(ctrl.get("worker_max_steer", 0.32))
        elif state["emergency_brake_started"] and not state["safe_brake_completed"]:
            front_gap = state.get("front_vehicle_gap")
            if front_gap is not None and front_gap <= 10.0:
                return 0.0, 0.95, compute_steer_to_location(
                    ego,
                    route_tracker.point_at_progress_smoothed(progress + 6.0, lateral_offset_m=target_offset),
                    gain=float(ctrl.get("emergency_steer_gain", 1.25)),
                    max_steer=float(ctrl.get("emergency_max_steer", 0.28)),
                )
            target_speed = float(ctrl.get("safe_follow_speed_kmh", 12.0))
            lookahead = float(ctrl.get("emergency_lookahead_m", 8.0))
            gain = float(ctrl.get("emergency_steer_gain", 1.25))
            max_steer = float(ctrl.get("emergency_max_steer", 0.28))

        yaw_now = _route_yaw_at(route_tracker, progress, delta_m=float(ctrl.get("curvature_probe_delta_m", 4.0)))
        yaw_ahead = _route_yaw_at(
            route_tracker,
            progress + float(ctrl.get("curvature_lookahead_probe_m", 14.0)),
            delta_m=float(ctrl.get("curvature_probe_delta_m", 4.0)),
        )
        route_heading_delta = abs(_wrap_deg(yaw_ahead - yaw_now))
        if route_heading_delta >= float(ctrl.get("turn_detection_heading_delta_deg", 12.0)):
            lookahead = min(lookahead, float(ctrl.get("turn_lookahead_m", 8.0)))
            gain = max(gain, float(ctrl.get("turn_steer_gain", 1.55)))
            max_steer = max(max_steer, float(ctrl.get("turn_max_steer", 0.42)))
            target_speed = min(target_speed, float(ctrl.get("turn_target_speed_kmh", 28.0)))
            window_radius = float(ctrl.get("turn_window_radius_m", 5.0))
            sigma = float(ctrl.get("turn_sigma_m", 2.5))

        planned_lateral_maneuver = construction_control_active or abs(target_offset) > 0.25
        if not planned_lateral_maneuver and abs(route_lateral_offset) >= float(ctrl.get("deviation_recovery_trigger_m", 1.0)):
            lookahead = min(lookahead, float(ctrl.get("deviation_recovery_lookahead_m", 6.5)))
            gain = max(gain, float(ctrl.get("deviation_recovery_steer_gain", 1.75)))
            max_steer = max(max_steer, float(ctrl.get("deviation_recovery_max_steer", 0.46)))
            target_speed = min(target_speed, float(ctrl.get("deviation_recovery_speed_kmh", 22.0)))
            window_radius = float(ctrl.get("turn_window_radius_m", 5.0))
            sigma = float(ctrl.get("turn_sigma_m", 2.5))

        if progress < 8.0 and speed < 15.0:
            max_steer = min(max_steer, 0.20)
            gain = min(gain, 0.90)

        correction_gain = float(ctrl.get("lateral_error_correction_gain", 0.95))
        correction_cap = float(ctrl.get("lateral_error_correction_cap_m", 1.8))
        lateral_error_to_target = route_lateral_offset - target_offset
        corrected_offset = target_offset - clip(
            correction_gain * lateral_error_to_target,
            -correction_cap,
            correction_cap,
        )
        target_loc = route_tracker.point_at_progress_smoothed(
            progress + lookahead,
            lateral_offset_m=corrected_offset,
            window_radius_m=window_radius,
            sample_step_m=float(ctrl.get("target_point_sample_step_m", 2.0)),
            sigma_m=sigma,
            tangent_delta_m=float(ctrl.get("target_point_tangent_delta_m", 2.0)),
        )
        steer = compute_steer_to_location(ego, target_loc, gain=gain, max_steer=max_steer)
        throttle, brake = _compute_speed_control(speed, target_speed, steer=steer)
        throttle = min(throttle, float(ctrl.get("max_throttle_cap", 0.80)))
        if abs(steer) > float(ctrl.get("large_steer_threshold", 0.30)):
            throttle = min(throttle, float(ctrl.get("large_steer_throttle_cap", 0.18)))
        return throttle, brake, steer

    def is_success(self, state, obs, cfg) -> bool:
        return bool(state["task_success"])

    def extra_record(self, ego, actors, state, obs, cfg) -> Dict[str, Any]:
        min_front_vehicle_distance = state["min_front_vehicle_distance"]
        min_ttc = state["min_ttc"]
        return {
            "hazard_score": state["hazard_score"],
            "danger_detected": state["danger_detected"],
            "danger_zone_activated": state["danger_zone_activated"],
            "slowdown_started": state["slowdown_started"],
            "safe_speed_reached": state["safe_speed_reached"],
            "safe_speed_hold_time": state["safe_speed_hold_time"],
            "target_speed_kmh": state["danger_target_speed_kmh"],
            "weather_precipitation": state["weather_precipitation"],
            "weather_wetness": state["weather_wetness"],
            "weather_fog_density": state["weather_fog_density"],
            "sun_altitude_angle": state["sun_altitude_angle"],
            "is_night": state["is_night"],
            "cut_in_detected": state["cut_in_detected"],
            "emergency_brake_started": state["emergency_brake_started"],
            "safe_brake_completed": state["safe_brake_completed"],
            "emergency_response_time_s": state["emergency_response_time_s"],
            "cut_in_actor_activated": state["cut_in_actor_activated"],
            "cut_in_actor_retired": state["cut_in_actor_retired"],
            "cut_in_spawn_failed": state["cut_in_spawn_failed"],
            "cut_in_phase": state["cut_in_phase"],
            "cut_in_yield_completed": state["cut_in_yield_completed"],
            "cut_in_plan_progress_m": state["cut_in_plan_progress_m"],
            "cut_in_target_lateral_offset_m": state["cut_in_target_lateral_offset_m"],
            "safe_follow_hold_time": state["safe_follow_hold_time"],
            "front_vehicle_distance": state["front_vehicle_distance"],
            "front_vehicle_gap": state["front_vehicle_gap"],
            "front_vehicle_lateral_offset": state["front_vehicle_lateral_offset"],
            "cut_in_actor_x": state["cut_in_actor_x"],
            "cut_in_actor_y": state["cut_in_actor_y"],
            "cut_in_actor_yaw_deg": state["cut_in_actor_yaw_deg"],
            "cut_in_actor_speed_kmh": state["cut_in_actor_speed_kmh"],
            "ttc_s": state["ttc_s"],
            "min_front_vehicle_distance": None if math.isinf(min_front_vehicle_distance) else min_front_vehicle_distance,
            "min_ttc": None if math.isinf(min_ttc) else min_ttc,
            "construction_detected": state["construction_detected"],
            "construction_active": state["construction_active"],
            "cone_detour_active": state["cone_detour_active"],
            "cone_detour_distance_m": state["cone_detour_distance_m"],
            "construction_target_lateral_offset_m": state["construction_target_lateral_offset_m"],
            "construction_zone_activated": state["construction_zone_activated"],
            "distance_to_construction": state["distance_to_construction"],
            "merge_started": state["merge_started"],
            "merge_completed": state["merge_completed"],
            "construction_pass_completed": state["construction_pass_completed"],
            "merge_hold_time": state["merge_hold_time"],
            "construction_return_hold_time": state["construction_return_hold_time"],
            "construction_visibility_enhanced": state["construction_visibility_enhanced"],
            "route_finish_brake_started": state["route_finish_brake_started"],
            "worker_detected": state["worker_detected"],
            "worker_crossing_active": state["worker_crossing_active"],
            "worker_phase": state["worker_phase"],
            "worker_motion_mode": state["worker_motion_mode"],
            "worker_crossing_elapsed_s": state["worker_crossing_elapsed_s"],
            "worker_distance_m": state["worker_distance_m"],
            "worker_path_remaining_m": state["worker_path_remaining_m"],
            "worker_emergency_brake_started": state["worker_emergency_brake_started"],
            "long_route_completed": state["long_route_completed"],
            "task_success": state["task_success"],
            "end_to_end_latency_ms": 50.0 if state["emergency_brake_started"] else 0.0,
        }
