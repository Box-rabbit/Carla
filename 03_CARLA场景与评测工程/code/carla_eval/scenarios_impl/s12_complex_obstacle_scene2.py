"""S12 – PDF scene 2 complex-obstacle 8km drive without voice input."""

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

import carla

from carla_eval.runtime_metrics import RouteTracker
from .base import BaseScenario, clip, compute_steer_to_location


def _window_active(progress_m: float, window: Dict[str, Any], pad_before: float = 0.0, pad_after: float = 0.0) -> bool:
    return float(window.get("progress_start_m", 0.0)) - pad_before <= progress_m <= float(
        window.get("progress_end_m", 0.0)
    ) + pad_after


def _route_yaw_at(route_tracker: RouteTracker, progress_m: float, delta_m: float = 4.0) -> float:
    p0 = max(0.0, progress_m - delta_m)
    p1 = min(route_tracker.route_total_length_m, progress_m + delta_m)
    a = route_tracker.point_at_progress(p0)
    b = route_tracker.point_at_progress(p1)
    return math.degrees(math.atan2(b.y - a.y, b.x - a.x))


def _wrap_deg(delta: float) -> float:
    return (delta + 180.0) % 360.0 - 180.0


def _yaw_delta_deg(a_deg: float, b_deg: float) -> float:
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0


def _transform_on_route(
    route_tracker: RouteTracker,
    progress_m: float,
    lateral_offset_m: float = 0.0,
    z_lift_m: float = 0.3,
    yaw_offset_deg: float = 0.0,
) -> carla.Transform:
    loc = route_tracker.point_at_progress(progress_m, lateral_offset_m=lateral_offset_m)
    yaw = _route_yaw_at(route_tracker, progress_m) + yaw_offset_deg
    return carla.Transform(
        carla.Location(x=loc.x, y=loc.y, z=loc.z + z_lift_m),
        carla.Rotation(yaw=yaw),
    )


def _rate_limit(current: float, desired: float, dt: float, up_rate: float, down_rate: float) -> float:
    if desired >= current:
        return min(desired, current + up_rate * dt)
    return max(desired, current - down_rate * dt)


def _smoothstep(value: float) -> float:
    x = clip(value, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


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


def _find_blueprint(bp_lib, preferred: str, fallback_pattern: str):
    try:
        return bp_lib.find(preferred)
    except (RuntimeError, IndexError):
        matches = bp_lib.filter(fallback_pattern)
        if not matches:
            raise
        return matches[0]


def _find_blueprint_from_candidates(bp_lib, preferred: str, fallback_patterns: Sequence[str]):
    if preferred:
        try:
            return bp_lib.find(preferred)
        except (RuntimeError, IndexError):
            pass
    for pattern in fallback_patterns:
        matches = bp_lib.filter(pattern)
        if matches:
            return matches[0]
    return None


def _fallback_patterns(actor_cfg: Dict[str, Any], default_pattern: str) -> List[str]:
    patterns = actor_cfg.get("fallback_filters")
    if patterns is None:
        patterns = actor_cfg.get("fallback_filter", default_pattern)
    if isinstance(patterns, str):
        return [patterns]
    return [str(pattern) for pattern in patterns if pattern]


def _configure_bus_blueprint(bp) -> None:
    for attr_name, value in (
        ("role_name", "s12_loaded_bus"),
        ("color", "245,190,40"),
    ):
        if not bp.has_attribute(attr_name):
            continue
        try:
            attr = bp.get_attribute(attr_name)
            if attr.recommended_values and value not in attr.recommended_values:
                bp.set_attribute(attr_name, attr.recommended_values[0])
            else:
                bp.set_attribute(attr_name, value)
        except Exception:
            pass


def _same_direction_lane(reference_wp: Optional[carla.Waypoint], candidate_wp: Optional[carla.Waypoint]) -> bool:
    return (
        reference_wp is not None
        and candidate_wp is not None
        and reference_wp.lane_id != 0
        and candidate_wp.lane_id != 0
        and reference_wp.lane_id * candidate_wp.lane_id > 0
    )


def _lane_center_transform(waypoint: carla.Waypoint, z_lift_m: float = 0.3) -> carla.Transform:
    tf = waypoint.transform
    return carla.Transform(
        carla.Location(
            x=tf.location.x,
            y=tf.location.y,
            z=tf.location.z + z_lift_m,
        ),
        carla.Rotation(
            pitch=tf.rotation.pitch,
            yaw=tf.rotation.yaw,
            roll=tf.rotation.roll,
        ),
    )


def _offset_location_from_waypoint(
    waypoint: carla.Waypoint,
    lateral_offset_m: float,
    z_lift_m: float,
) -> carla.Location:
    tf = waypoint.transform
    right = tf.get_right_vector()
    return carla.Location(
        x=tf.location.x + lateral_offset_m * right.x,
        y=tf.location.y + lateral_offset_m * right.y,
        z=tf.location.z + z_lift_m,
    )


def _offstage_transform() -> carla.Transform:
    return carla.Transform(
        carla.Location(x=10000.0, y=10000.0, z=-50.0),
        carla.Rotation(yaw=0.0),
    )


class ComplexObstacleScene2(BaseScenario):
    """
    PDF scene 2:
    - cloudy dusk
    - urban secondary road + intersection + bus stop
    - mixed complex-obstacle task chain
    - pedestrian caution -> slow-vehicle overtake -> bus-stop caution -> finish 8km
    """

    def _route_waypoint_at_progress(
        self,
        route_tracker: Optional[RouteTracker],
        progress_m: float,
    ) -> Optional[carla.Waypoint]:
        if route_tracker is None:
            return None
        loc = route_tracker.point_at_progress(progress_m)
        return route_tracker.carla_map.get_waypoint(
            loc,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )

    def _seek_adjacent_lane(
        self,
        base_wp: Optional[carla.Waypoint],
        direction: str,
        allowed_types: Sequence[carla.LaneType],
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
            if require_same_direction and not _same_direction_lane(base_wp, current):
                continue
            return current
        return None

    def _resolve_vehicle_waypoint(
        self,
        route_tracker: Optional[RouteTracker],
        progress_m: float,
        lane_role: str = "ego",
    ) -> Optional[carla.Waypoint]:
        base_wp = self._route_waypoint_at_progress(route_tracker, progress_m)
        if base_wp is None:
            return None
        if lane_role == "ego":
            return base_wp
        if lane_role == "left_flow":
            return self._seek_adjacent_lane(
                base_wp,
                direction="left",
                allowed_types=(carla.LaneType.Driving,),
                require_same_direction=True,
            )
        if lane_role == "right_flow":
            return self._seek_adjacent_lane(
                base_wp,
                direction="right",
                allowed_types=(carla.LaneType.Driving,),
                require_same_direction=True,
            )
        if lane_role == "right_stop":
            right_shoulder = self._seek_adjacent_lane(
                base_wp,
                direction="right",
                allowed_types=(carla.LaneType.Parking, carla.LaneType.Shoulder),
                require_same_direction=False,
            )
            if right_shoulder is not None:
                return right_shoulder
            return self._seek_adjacent_lane(
                base_wp,
                direction="right",
                allowed_types=(carla.LaneType.Driving,),
                require_same_direction=True,
            )
        return base_wp

    def _resolve_vehicle_waypoint_with_search(
        self,
        route_tracker: Optional[RouteTracker],
        progress_m: float,
        lane_role: str = "ego",
        search_ahead_m: float = 220.0,
        search_step_m: float = 8.0,
    ) -> Optional[Tuple[carla.Waypoint, float]]:
        waypoint = self._resolve_vehicle_waypoint(route_tracker, progress_m, lane_role=lane_role)
        if waypoint is not None:
            return waypoint, progress_m
        if route_tracker is None or lane_role == "ego":
            return None

        start = max(0.0, float(progress_m))
        end = min(route_tracker.route_total_length_m, start + max(float(search_ahead_m), float(search_step_m)))
        p = start + float(search_step_m)
        while p <= end:
            waypoint = self._resolve_vehicle_waypoint(route_tracker, p, lane_role=lane_role)
            if waypoint is not None:
                return waypoint, p
            p += float(search_step_m)
        return None

    def _lane_role_for_stage(self, stage: str) -> str:
        if stage == "ambient_left_flow":
            return "left_flow"
        if stage == "ambient_right_flow":
            return "right_flow"
        if stage == "bus_stop_bus":
            return "right_stop"
        return "ego"

    def _planned_vehicle_progress(self, actor_cfg: Dict[str, Any], cfg) -> float:
        stage = actor_cfg.get("stage", "")
        windows = cfg.get("action_windows", {})
        scene_cfg = cfg.get("success_criteria", {}).get("complex_scene2", {})
        if stage == "slow_vehicle":
            overtake_win = windows.get("slow_vehicle_overtake", {})
            anchor = float(overtake_win.get("anchor_progress_m", 650.0))
            initial_gap = float(scene_cfg.get("slow_vehicle_initial_gap_m", 42.0))
            return anchor + initial_gap
        if stage == "bus_stop_bus":
            bus_win = windows.get("bus_stop_caution", {})
            return float(bus_win.get("anchor_progress_m", 1100.0))
        if stage.startswith("ambient_"):
            return float(actor_cfg.get("activation_progress_m", 1200.0)) + float(
                actor_cfg.get("base_progress_offset_m", 35.0)
            )
        rel = actor_cfg.get("relative_to_ego", {})
        return max(80.0, float(rel.get("longitudinal_distance_m", 80.0)))

    def _bus_stop_transform(
        self,
        route_tracker: Optional[RouteTracker],
        progress_m: float,
        cfg,
        z_lift_m: float = 0.3,
    ) -> Optional[carla.Transform]:
        waypoint = self._route_waypoint_at_progress(route_tracker, progress_m)
        if waypoint is None:
            return None
        bus_win = cfg.get("action_windows", {}).get("bus_stop_caution", {})
        lateral = float(bus_win.get("bus_lateral_offset_m", 4.8))
        return carla.Transform(
            _offset_location_from_waypoint(waypoint, lateral_offset_m=lateral, z_lift_m=z_lift_m),
            carla.Rotation(
                pitch=waypoint.transform.rotation.pitch,
                yaw=waypoint.transform.rotation.yaw,
                roll=waypoint.transform.rotation.roll,
            ),
        )

    def _bus_stop_prop_transform(
        self,
        route_tracker: Optional[RouteTracker],
        progress_m: float,
        lateral_offset_m: float,
        yaw_offset_deg: float = 0.0,
        z_lift_m: float = 0.05,
    ) -> Optional[carla.Transform]:
        waypoint = self._route_waypoint_at_progress(route_tracker, progress_m)
        if waypoint is None:
            return None
        loc = _offset_location_from_waypoint(waypoint, lateral_offset_m=lateral_offset_m, z_lift_m=z_lift_m)
        return carla.Transform(
            loc,
            carla.Rotation(
                pitch=waypoint.transform.rotation.pitch,
                yaw=waypoint.transform.rotation.yaw + yaw_offset_deg,
                roll=waypoint.transform.rotation.roll,
            ),
        )

    def _bus_stop_pedestrian_offsets(self, stage: str, bus_win: Dict[str, Any]) -> Tuple[float, float]:
        door_lat = float(bus_win.get("pedestrian_bus_door_lateral_offset_m", 4.2))
        sidewalk_lat = float(bus_win.get("pedestrian_sidewalk_lateral_offset_m", 6.4))
        gap = float(bus_win.get("pedestrian_pair_spacing_m", 0.6))
        idx = 1 if str(stage).endswith("_2") else 0
        if "alight" in str(stage):
            return door_lat + idx * 0.35, sidewalk_lat + idx * gap
        return sidewalk_lat + idx * gap, door_lat + idx * 0.35

    def _bus_stop_pedestrian_progress(self, bus_anchor_m: float, actor_index: int, bus_win: Dict[str, Any]) -> float:
        center_offset = float(bus_win.get("pedestrian_progress_offset_m", 5.0))
        spacing = float(bus_win.get("pedestrian_longitudinal_spacing_m", 0.9))
        return bus_anchor_m + center_offset + (float(actor_index) - 1.5) * spacing

    def _planned_vehicle_transform(
        self,
        route_tracker: Optional[RouteTracker],
        actor_cfg: Dict[str, Any],
        cfg,
        progress_override_m: Optional[float] = None,
    ) -> Optional[carla.Transform]:
        stage = actor_cfg.get("stage", "")
        progress_m = float(progress_override_m) if progress_override_m is not None else self._planned_vehicle_progress(actor_cfg, cfg)
        if stage == "bus_stop_bus":
            bus_tf = self._bus_stop_transform(route_tracker, progress_m, cfg, z_lift_m=0.35)
            if bus_tf is not None:
                return bus_tf

        lane_role = self._lane_role_for_stage(stage)
        resolved = self._resolve_vehicle_waypoint_with_search(route_tracker, progress_m, lane_role=lane_role)
        if resolved is None:
            return None
        waypoint, _ = resolved
        return _lane_center_transform(waypoint, z_lift_m=0.35)

    def _vehicle_spawn_candidates(
        self,
        route_tracker: Optional[RouteTracker],
        actor_cfg: Dict[str, Any],
        cfg,
        ego_tf: carla.Transform,
    ) -> List[carla.Transform]:
        progress = self._planned_vehicle_progress(actor_cfg, cfg)
        candidates: List[carla.Transform] = []
        for delta in (0.0, 12.0, -12.0, 26.0, -26.0):
            tf = self._planned_vehicle_transform(route_tracker, actor_cfg, cfg, progress_override_m=max(0.0, progress + delta))
            if tf is not None:
                candidates.append(tf)

        rel = actor_cfg.get("relative_to_ego", {})
        fwd = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()
        long_d = float(rel.get("longitudinal_distance_m", 120.0))
        lat_d = float(rel.get("lateral_offset_m", 0.0))
        candidates.append(
            carla.Transform(
                carla.Location(
                    x=ego_tf.location.x + long_d * fwd.x + lat_d * right.x,
                    y=ego_tf.location.y + long_d * fwd.y + lat_d * right.y,
                    z=ego_tf.location.z + 0.5,
                ),
                carla.Rotation(yaw=ego_tf.rotation.yaw),
            )
        )
        return candidates

    def _set_vehicle_on_lane(
        self,
        actor: Optional[carla.Actor],
        route_tracker: Optional[RouteTracker],
        progress_m: float,
        lane_role: str = "ego",
        z_lift_m: float = 0.3,
    ) -> bool:
        if actor is None:
            return False

        resolved = self._resolve_vehicle_waypoint_with_search(route_tracker, progress_m, lane_role=lane_role)
        if resolved is None:
            return False
        waypoint, _ = resolved

        actor.set_transform(_lane_center_transform(waypoint, z_lift_m=z_lift_m))
        return True

    def _advance_vehicle_along_current_lane(
        self,
        actor: Optional[carla.Actor],
        route_tracker: Optional[RouteTracker],
        speed_mps: float,
        dt: float,
        lane_role: str = "ego",
        z_lift_m: float = 0.3,
        fallback_progress_m: Optional[float] = None,
    ) -> bool:
        if actor is None or route_tracker is None:
            return False
        if actor.get_location().z < -10.0:
            if fallback_progress_m is None:
                return False
            return self._set_vehicle_on_lane(
                actor,
                route_tracker,
                fallback_progress_m,
                lane_role=lane_role,
                z_lift_m=z_lift_m,
            )

        lane_types = carla.LaneType.Driving
        if lane_role == "right_stop":
            lane_types = carla.LaneType.Driving | carla.LaneType.Parking | carla.LaneType.Shoulder

        current_wp = route_tracker.carla_map.get_waypoint(
            actor.get_location(),
            project_to_road=True,
            lane_type=lane_types,
        )
        if current_wp is None:
            if fallback_progress_m is None:
                return False
            return self._set_vehicle_on_lane(
                actor,
                route_tracker,
                fallback_progress_m,
                lane_role=lane_role,
                z_lift_m=z_lift_m,
            )

        step_m = max(0.3, float(speed_mps) * max(float(dt), 0.05))
        candidates = current_wp.next(step_m)
        if not candidates:
            if fallback_progress_m is None:
                return False
            return self._set_vehicle_on_lane(
                actor,
                route_tracker,
                fallback_progress_m,
                lane_role=lane_role,
                z_lift_m=z_lift_m,
            )

        def candidate_score(wp: carla.Waypoint) -> Tuple[float, float, float]:
            direction_penalty = 0.0 if _same_direction_lane(current_wp, wp) else 1000.0
            same_lane_penalty = 0.0 if wp.lane_id == current_wp.lane_id else 15.0
            yaw_penalty = abs(_yaw_delta_deg(wp.transform.rotation.yaw, current_wp.transform.rotation.yaw))
            return direction_penalty, same_lane_penalty, yaw_penalty

        next_wp = min(candidates, key=candidate_score)
        actor.set_transform(_lane_center_transform(next_wp, z_lift_m=z_lift_m))
        return True

    def _make_walker_transform_at_lateral_offset(
        self,
        route_tracker: Optional[RouteTracker],
        anchor_progress_m: float,
        lateral_offset_m: float,
        facing_direction_sign: float,
        z_lift_m: float = 0.9,
    ) -> Optional[carla.Transform]:
        waypoint = self._pedestrian_waypoint(route_tracker, anchor_progress_m)
        if waypoint is None:
            return None

        loc = _offset_location_from_waypoint(waypoint, lateral_offset_m=lateral_offset_m, z_lift_m=z_lift_m)
        yaw_offset = 90.0 if facing_direction_sign >= 0.0 else -90.0
        return carla.Transform(
            loc,
            carla.Rotation(
                pitch=0.0,
                yaw=waypoint.transform.rotation.yaw + yaw_offset,
                roll=0.0,
            ),
        )

    def _pedestrian_waypoint(
        self,
        route_tracker: Optional[RouteTracker],
        anchor_progress_m: float,
    ) -> Optional[carla.Waypoint]:
        return self._route_waypoint_at_progress(route_tracker, anchor_progress_m)

    def _place_walker_at_lateral_offset(
        self,
        actor: Optional[carla.Actor],
        route_tracker: Optional[RouteTracker],
        anchor_progress_m: float,
        lateral_offset_m: float,
        facing_direction_sign: float,
        z_lift_m: float = 0.9,
    ) -> bool:
        if actor is None:
            return False
        transform = self._make_walker_transform_at_lateral_offset(
            route_tracker,
            anchor_progress_m=anchor_progress_m,
            lateral_offset_m=lateral_offset_m,
            facing_direction_sign=facing_direction_sign,
            z_lift_m=z_lift_m,
        )
        if transform is None:
            return False
        actor.set_transform(transform)
        return True

    def _move_walker_towards(
        self,
        actor: Optional[carla.Actor],
        waypoint: Optional[carla.Waypoint],
        desired_lateral_offset_m: float,
        dt: float,
        target_speed_mps: float,
        z_lift_m: float = 0.9,
    ) -> None:
        if actor is None or waypoint is None:
            return

        desired_loc = _offset_location_from_waypoint(
            waypoint,
            lateral_offset_m=desired_lateral_offset_m,
            z_lift_m=z_lift_m,
        )
        current = actor.get_location()
        dx = desired_loc.x - current.x
        dy = desired_loc.y - current.y
        dist = math.hypot(dx, dy)
        if dist <= 0.08:
            actor.apply_control(carla.WalkerControl(direction=carla.Vector3D(), speed=0.0, jump=False))
            return

        direction = carla.Vector3D(dx / dist, dy / dist, 0.0)
        if dist > max(1.5, target_speed_mps * max(dt, 0.05) * 3.0):
            yaw = math.degrees(math.atan2(direction.y, direction.x))
            actor.set_transform(
                carla.Transform(
                    carla.Location(x=desired_loc.x, y=desired_loc.y, z=desired_loc.z),
                    carla.Rotation(pitch=0.0, yaw=yaw, roll=0.0),
                )
            )
            actor.apply_control(carla.WalkerControl(direction=direction, speed=0.0, jump=False))
            return

        actor.apply_control(
            carla.WalkerControl(
                direction=direction,
                speed=min(target_speed_mps, max(0.6, dist / max(dt, 0.05))),
                jump=False,
            )
        )

    def _overtake_progress_markers(self, cfg) -> Tuple[float, float, float, float]:
        windows = cfg.get("action_windows", {})
        scene_cfg = cfg.get("success_criteria", {}).get("complex_scene2", {})
        overtake_win = windows.get("slow_vehicle_overtake", {})
        start = float(overtake_win.get("progress_start_m", 520.0))
        anchor = float(overtake_win.get("anchor_progress_m", start + 130.0))
        lane_change_start = float(scene_cfg.get("overtake_lane_change_start_progress_m", start + 90.0))
        left_full = float(scene_cfg.get("overtake_left_lane_full_progress_m", lane_change_start + 70.0))
        return_start = float(scene_cfg.get("overtake_return_start_progress_m", anchor + 200.0))
        return_end = float(scene_cfg.get("overtake_return_end_progress_m", return_start + 90.0))
        left_full = max(left_full, lane_change_start + 10.0)
        return_start = max(return_start, left_full + 10.0)
        return_end = max(return_end, return_start + 10.0)
        return lane_change_start, left_full, return_start, return_end

    def spawn_actors(self, world, ego, cfg) -> List[carla.Actor]:
        bp_lib = world.get_blueprint_library()
        ego_tf = ego.get_transform()
        fwd = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()
        windows = cfg.get("action_windows", {})
        route_tracker = None
        try:
            route_tracker = RouteTracker.from_route_config(
                world.get_map(),
                cfg,
                corridor_half_width_m=float(cfg.get("evaluation", {}).get("route_corridor_half_width_m", 4.5)),
            )
        except Exception:
            route_tracker = None

        actors: List[carla.Actor] = []
        self._s12_actor_stage_by_id = {}
        self._s12_vehicle_stage_names = set()
        self._s12_pedestrian_stage_names = set()
        for actor_cfg in cfg.get("actors", {}).get("vehicles", []):
            stage = actor_cfg.get("stage")
            fallback_patterns = _fallback_patterns(actor_cfg, "vehicle.*")
            if stage == "bus_stop_bus":
                fallback_patterns = list(
                    dict.fromkeys(
                        fallback_patterns
                        + [
                            "vehicle.mitsubishi.*",
                            "vehicle.*bus*",
                            "vehicle.volkswagen.t2",
                            "vehicle.carlamotors.*",
                            "vehicle.carlamotors.carlacola",
                            "vehicle.mercedes*",
                            "vehicle.bmw.grandtourer",
                            "vehicle.*",
                        ]
                    )
                )
            bp = _find_blueprint_from_candidates(
                bp_lib,
                actor_cfg.get("type", "vehicle.audi.tt"),
                fallback_patterns,
            )
            if bp is None:
                print(f"[S12][WARN] failed to find vehicle blueprint stage={stage}")
                continue
            if stage == "bus_stop_bus":
                _configure_bus_blueprint(bp)
            actor = None
            for spawn_tf in self._vehicle_spawn_candidates(route_tracker, actor_cfg, cfg, ego_tf):
                actor = world.try_spawn_actor(bp, spawn_tf)
                if actor is not None:
                    break
            if actor is None:
                print(f"[S12][WARN] failed to spawn vehicle stage={actor_cfg.get('stage')} type={bp.id}")
                continue
            try:
                actor.set_simulate_physics(False)
            except Exception:
                pass
            loc = actor.get_location()
            print(
                f"[S12][SPAWN] vehicle stage={actor_cfg.get('stage')} type={bp.id} "
                f"loc=({loc.x:.1f},{loc.y:.1f},{loc.z:.1f})"
            )
            actors.append(actor)
            self._s12_actor_stage_by_id[actor.id] = stage
            self._s12_vehicle_stage_names.add(stage)

        bus_stop_ped_spawn_index = 0
        for actor_cfg in cfg.get("actors", {}).get("pedestrians", []):
            bp = _find_blueprint(
                bp_lib,
                actor_cfg.get("type", "walker.pedestrian.0001"),
                actor_cfg.get("fallback_filter", "walker.pedestrian.*"),
            )
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            stage = actor_cfg.get("stage")
            rel = actor_cfg.get("relative_to_ego", {})
            long_d = float(rel.get("longitudinal_distance_m", -10.0))
            lat_d = float(rel.get("lateral_offset_m", 4.0))
            spawn_tf = carla.Transform(
                carla.Location(
                    x=ego_tf.location.x + long_d * fwd.x + lat_d * right.x,
                    y=ego_tf.location.y + long_d * fwd.y + lat_d * right.y,
                    z=ego_tf.location.z + 0.8,
                ),
                carla.Rotation(yaw=ego_tf.rotation.yaw + 90.0),
            )
            if route_tracker is not None:
                if stage == "pedestrian_crossing":
                    ped_win = windows.get("pedestrian_crossing", {})
                    anchor = float(ped_win.get("anchor_progress_m", 1050.0))
                    candidate_tf = self._make_walker_transform_at_lateral_offset(
                        route_tracker,
                        anchor_progress_m=anchor,
                        lateral_offset_m=float(ped_win.get("start_lateral_offset_m", 4.5)),
                        facing_direction_sign=float(ped_win.get("end_lateral_offset_m", -0.8))
                        - float(ped_win.get("start_lateral_offset_m", 4.5)),
                        z_lift_m=0.9,
                    )
                    if candidate_tf is not None:
                        spawn_tf = candidate_tf
                elif str(stage).startswith("bus_stop_pedestrian"):
                    bus_win = windows.get("bus_stop_caution", {})
                    anchor = self._bus_stop_pedestrian_progress(
                        float(bus_win.get("anchor_progress_m", 5220.0)),
                        bus_stop_ped_spawn_index,
                        bus_win,
                    )
                    bus_stop_ped_spawn_index += 1
                    lateral, end_lateral = self._bus_stop_pedestrian_offsets(str(stage), bus_win)
                    candidate_tf = self._make_walker_transform_at_lateral_offset(
                        route_tracker,
                        anchor_progress_m=anchor,
                        lateral_offset_m=lateral,
                        facing_direction_sign=end_lateral - lateral,
                        z_lift_m=0.9,
                    )
                    if candidate_tf is not None:
                        spawn_tf = candidate_tf
            actor = world.try_spawn_actor(bp, spawn_tf)
            if actor is not None:
                loc = actor.get_location()
                print(
                    f"[S12][SPAWN] pedestrian stage={stage} type={bp.id} "
                    f"loc=({loc.x:.1f},{loc.y:.1f},{loc.z:.1f})"
                )
                actors.append(actor)
                self._s12_actor_stage_by_id[actor.id] = stage
                self._s12_pedestrian_stage_names.add(stage)
            else:
                print(f"[S12][WARN] failed to spawn pedestrian stage={stage} type={bp.id}")

        for prop_cfg in cfg.get("actors", {}).get("static_props", []):
            stage = prop_cfg.get("stage", "")
            if not str(stage).startswith("bus_stop"):
                continue
            bp = _find_blueprint_from_candidates(
                bp_lib,
                prop_cfg.get("type", ""),
                prop_cfg.get(
                    "fallback_filters",
                    [
                        "static.prop.bus*",
                        "static.prop.bench*",
                        "static.prop.sign*",
                        "static.prop.streetbarrier*",
                        "static.prop.*",
                    ],
                ),
            )
            if bp is None:
                print(f"[S12][WARN] failed to find static prop blueprint stage={stage}")
                continue
            bus_win = windows.get("bus_stop_caution", {})
            anchor = float(bus_win.get("anchor_progress_m", 880.0)) + float(prop_cfg.get("progress_offset_m", 0.0))
            lateral = float(prop_cfg.get("lateral_offset_m", 7.0))
            prop_tf = self._bus_stop_prop_transform(
                route_tracker,
                anchor,
                lateral_offset_m=lateral,
                yaw_offset_deg=float(prop_cfg.get("yaw_offset_deg", 0.0)),
                z_lift_m=float(prop_cfg.get("z_lift_m", 0.05)),
            )
            if prop_tf is None:
                continue
            actor = world.try_spawn_actor(bp, prop_tf)
            if actor is not None:
                loc = actor.get_location()
                print(
                    f"[S12][SPAWN] static_prop stage={stage} type={bp.id} "
                    f"loc=({loc.x:.1f},{loc.y:.1f},{loc.z:.1f})"
                )
                actors.append(actor)
                self._s12_actor_stage_by_id[actor.id] = stage
            else:
                print(f"[S12][WARN] failed to spawn static_prop stage={stage} type={bp.id}")

        if route_tracker is not None:
            bus_win = windows.get("bus_stop_caution", {})
            bus_anchor = float(bus_win.get("anchor_progress_m", 880.0))
            sign_tf = self._bus_stop_prop_transform(
                route_tracker,
                bus_anchor,
                lateral_offset_m=float(bus_win.get("station_marker_lateral_offset_m", 7.0)),
                z_lift_m=0.2,
            )
            if sign_tf is not None:
                loc = sign_tf.location
                world.debug.draw_string(
                    carla.Location(x=loc.x, y=loc.y, z=loc.z + 2.8),
                    "BUS STOP",
                    draw_shadow=True,
                    color=carla.Color(255, 210, 0),
                    life_time=900.0,
                    persistent_lines=False,
                )
                world.debug.draw_line(
                    carla.Location(x=loc.x, y=loc.y, z=loc.z),
                    carla.Location(x=loc.x, y=loc.y, z=loc.z + 2.6),
                    thickness=0.10,
                    color=carla.Color(255, 210, 0),
                    life_time=900.0,
                    persistent_lines=False,
                )

        return actors

    def initial_state(self, ego, actors, cfg) -> Dict[str, Any]:
        scene_cfg = cfg.get("success_criteria", {}).get("complex_scene2", {})
        ctrl = cfg.get("controller", {})
        initial_target = float(ctrl.get("initial_target_speed_kmh", ctrl.get("cruise_target_speed_kmh", 50.0)))
        return {
            "actors_initialized": False,
            "cruise_target_speed_kmh": float(ctrl.get("cruise_target_speed_kmh", 50.0)),
            "desired_speed_kmh": initial_target,
            "target_speed_kmh": initial_target,
            "target_lateral_offset_m": 0.0,
            "phase": "cruise",
            "slow_vehicle_progress_m": None,
            "slow_vehicle_active": False,
            "slow_vehicle_initialized": False,
            "slow_vehicle_retired": False,
            "ped_cross_started": False,
            "ped_cross_cleared": False,
            "ped_cross_ratio": 0.0,
            "ped_cross_speed_mps": 1.8,
            "pedestrian_detected": False,
            "ped_slowdown_started": False,
            "safe_slowdown_completed": False,
            "ped_hold_s": 0.0,
            "ped_min_distance_m": float("inf"),
            "bus_stop_ped_started": False,
            "bus_stop_ped_ratio": 0.0,
            "bus_stop_ped_speed_mps": 1.2,
            "slow_vehicle_detected": False,
            "lane_change_started": False,
            "overtake_completed": False,
            "return_to_lane_completed": False,
            "return_to_lane_started": False,
            "return_start_progress_m": None,
            "return_hold_s": 0.0,
            "min_front_vehicle_gap_m": float("inf"),
            "bus_stop_detected": False,
            "bus_stop_slowdown_started": False,
            "bus_stop_pass_completed": False,
            "bus_stop_hold_s": 0.0,
            "bus_actor_active": False,
            "ambient_vehicle_progress": {},
            "long_route_completed": False,
            "success": False,
            "scene2_safe_speed_kmh": float(scene_cfg.get("pedestrian_safe_speed_kmh", 30.0)),
            "scene2_bus_stop_target_speed_kmh": float(scene_cfg.get("bus_stop_target_speed_kmh", 30.0)),
        }

    def _ensure_actor_placement(self, actors, state, route_tracker: Optional[RouteTracker], cfg) -> None:
        if state["actors_initialized"] or route_tracker is None:
            return

        vehicle_cfgs = cfg.get("actors", {}).get("vehicles", [])
        ped_cfgs = cfg.get("actors", {}).get("pedestrians", [])
        vehicles_by_stage, pedestrians_by_stage = self._split_actors(actors, cfg)

        for actor_cfg in vehicle_cfgs:
            stage = actor_cfg.get("stage")
            actor = vehicles_by_stage.get(stage)
            if actor is None:
                continue
            if stage == "slow_vehicle":
                self._park_actor(actor, route_tracker, self._inactive_vehicle_progress(0.0, "slow_vehicle", cfg), lane_role="ego")
            elif stage == "bus_stop_bus":
                self._park_actor(actor, route_tracker, self._inactive_vehicle_progress(0.0, "bus_stop_bus", cfg), lane_role="right_stop")
            elif stage.startswith("ambient_"):
                planned_progress = self._planned_vehicle_progress(actor_cfg, cfg)
                state["ambient_vehicle_progress"][stage] = planned_progress
                self._park_actor(
                    actor,
                    route_tracker,
                    planned_progress,
                    lane_role=self._lane_role_for_stage(stage),
                    cfg=cfg,
                    stage=stage,
                )

        bus_stop_ped_index = 0
        for actor_cfg in ped_cfgs:
            stage = actor_cfg.get("stage")
            actor = pedestrians_by_stage.get(stage)
            if actor is None:
                continue
            if stage == "pedestrian_crossing":
                win = cfg.get("action_windows", {}).get("pedestrian_crossing", {})
                anchor = float(win.get("anchor_progress_m", win.get("progress_start_m", 1100.0)))
                start_lat = float(win.get("start_lateral_offset_m", 4.5))
                end_lat = float(win.get("end_lateral_offset_m", -0.8))
                self._place_walker_at_lateral_offset(
                    actor,
                    route_tracker,
                    anchor_progress_m=anchor,
                    lateral_offset_m=start_lat,
                    facing_direction_sign=end_lat - start_lat,
                    z_lift_m=0.9,
                )
            elif str(stage).startswith("bus_stop_pedestrian"):
                win = cfg.get("action_windows", {}).get("bus_stop_caution", {})
                anchor = self._bus_stop_pedestrian_progress(
                    float(win.get("anchor_progress_m", win.get("progress_start_m", 5200.0))),
                    bus_stop_ped_index,
                    win,
                )
                bus_stop_ped_index += 1
                start_lat, end_lat = self._bus_stop_pedestrian_offsets(str(stage), win)
                self._place_walker_at_lateral_offset(
                    actor,
                    route_tracker,
                    anchor_progress_m=anchor,
                    lateral_offset_m=start_lat,
                    facing_direction_sign=end_lat - start_lat,
                    z_lift_m=0.9,
                )

        state["actors_initialized"] = True

    def _split_actors(self, actors, cfg):
        vehicle_cfgs = cfg.get("actors", {}).get("vehicles", [])
        ped_cfgs = cfg.get("actors", {}).get("pedestrians", [])
        stage_by_id = getattr(self, "_s12_actor_stage_by_id", {})
        vehicle_stage_names = set(getattr(self, "_s12_vehicle_stage_names", set()))
        pedestrian_stage_names = set(getattr(self, "_s12_pedestrian_stage_names", set()))
        vehicles_by_stage = {}
        pedestrians_by_stage = {}
        if stage_by_id:
            for actor in actors:
                stage = stage_by_id.get(actor.id)
                if stage in vehicle_stage_names:
                    vehicles_by_stage[stage] = actor
                elif stage in pedestrian_stage_names:
                    pedestrians_by_stage[stage] = actor
            return vehicles_by_stage, pedestrians_by_stage

        vehicles = actors[: len(vehicle_cfgs)]
        pedestrians = actors[len(vehicle_cfgs) : len(vehicle_cfgs) + len(ped_cfgs)]
        for actor, actor_cfg in zip(vehicles, vehicle_cfgs):
            vehicles_by_stage[actor_cfg.get("stage")] = actor
        for actor, actor_cfg in zip(pedestrians, ped_cfgs):
            pedestrians_by_stage[actor_cfg.get("stage")] = actor

        return vehicles_by_stage, pedestrians_by_stage

    def _park_actor(
        self,
        actor,
        route_tracker: RouteTracker,
        progress_m: float,
        lane_role: str = "ego",
        cfg=None,
        stage: str = "",
    ) -> None:
        if actor is None or route_tracker is None:
            return
        if stage == "bus_stop_bus" and cfg is not None:
            bus_tf = self._bus_stop_transform(route_tracker, progress_m, cfg, z_lift_m=0.35)
            if bus_tf is not None:
                actor.set_transform(bus_tf)
                return
        self._set_vehicle_on_lane(actor, route_tracker, progress_m, lane_role=lane_role, z_lift_m=0.35)

    def _inactive_vehicle_progress(self, progress: float, stage: str, cfg) -> float:
        windows = cfg.get("action_windows", {})
        scene_cfg = cfg.get("success_criteria", {}).get("complex_scene2", {})
        if stage == "slow_vehicle":
            overtake_win = windows.get("slow_vehicle_overtake", {})
            anchor = float(overtake_win.get("anchor_progress_m", 2480.0))
            initial_gap = float(scene_cfg.get("slow_vehicle_initial_gap_m", 42.0))
            return max(anchor + initial_gap, progress + 180.0)
        if stage == "bus_stop_bus":
            bus_win = windows.get("bus_stop_caution", {})
            anchor = float(bus_win.get("anchor_progress_m", 5220.0))
            return max(anchor, progress + 200.0)
        if stage.startswith("ambient_"):
            for actor_cfg in cfg.get("actors", {}).get("vehicles", []):
                if actor_cfg.get("stage") == stage:
                    return self._planned_vehicle_progress(actor_cfg, cfg)
            return max(progress + 180.0, 180.0)
        return max(progress + 120.0, 100.0)

    def _update_ambient_flow(self, vehicles_by_stage, state, route_tracker: RouteTracker, progress: float, dt: float, cfg) -> None:
        for actor_cfg in cfg.get("actors", {}).get("vehicles", []):
            stage = actor_cfg.get("stage")
            if not stage.startswith("ambient_"):
                continue
            actor = vehicles_by_stage.get(stage)
            if actor is None:
                continue
            lane_role = "left_flow" if "left" in stage else "right_flow"
            activate_at = float(actor_cfg.get("activation_progress_m", 0.0))
            if progress < activate_at:
                self._park_actor(
                    actor,
                    route_tracker,
                    self._planned_vehicle_progress(actor_cfg, cfg),
                    lane_role=lane_role,
                    cfg=cfg,
                    stage=stage,
                )
                continue

            speed_kmh = float(actor_cfg.get("cruise_speed_kmh", 42.0))
            min_gap = float(actor_cfg.get("min_gap_to_ego_m", 20.0))
            desired_progress = max(
                float(state["ambient_vehicle_progress"].get(stage, progress + min_gap)),
                progress + min_gap,
            )
            desired_progress += (speed_kmh / 3.6) * dt
            state["ambient_vehicle_progress"][stage] = desired_progress
            if not self._set_vehicle_on_lane(
                actor,
                route_tracker,
                desired_progress,
                lane_role=lane_role,
                z_lift_m=0.3,
            ):
                self._park_actor(
                    actor,
                    route_tracker,
                    self._planned_vehicle_progress(actor_cfg, cfg),
                    lane_role=lane_role,
                    cfg=cfg,
                    stage=stage,
                )

    def update_state(self, ego, actors, state, obs, dt, cfg) -> Dict[str, Any]:
        route_tracker: RouteTracker = obs.get("route_tracker")
        self._ensure_actor_placement(actors, state, route_tracker, cfg)

        progress = float(obs["route_metrics"]["route_progress_m"])
        max_progress = float(obs["route_metrics"].get("max_route_progress_m", progress))
        speed = float(obs["speed_kmh"])
        scene_cfg = cfg.get("success_criteria", {}).get("complex_scene2", {})
        windows = cfg.get("action_windows", {})
        ctrl = cfg.get("controller", {})

        ped_win = windows.get("pedestrian_crossing", {})
        overtake_win = windows.get("slow_vehicle_overtake", {})
        bus_win = windows.get("bus_stop_caution", {})
        lane_change_start_m, left_lane_full_m, overtake_return_start_m, overtake_return_end_m = self._overtake_progress_markers(cfg)

        vehicles_by_stage, pedestrians_by_stage = self._split_actors(actors, cfg)
        slow_vehicle = vehicles_by_stage.get("slow_vehicle")
        bus = vehicles_by_stage.get("bus_stop_bus")
        crossing_ped = pedestrians_by_stage.get("pedestrian_crossing")
        bus_stop_peds = [
            (str(stage), actor)
            for stage, actor in pedestrians_by_stage.items()
            if str(stage).startswith("bus_stop_pedestrian")
        ]
        bus_stop_peds.sort(key=lambda item: item[0])

        if route_tracker is not None:
            self._update_ambient_flow(vehicles_by_stage, state, route_tracker, progress, dt, cfg)

        if route_tracker is not None and crossing_ped is not None:
            ped_anchor = float(ped_win.get("anchor_progress_m", 1100.0))
            detection_distance = float(scene_cfg.get("pedestrian_detection_distance_m", 38.0))
            cross_start_distance = float(scene_cfg.get("pedestrian_cross_start_distance_m", detection_distance))
            start_lat = float(ped_win.get("start_lateral_offset_m", 4.5))
            end_lat = float(ped_win.get("end_lateral_offset_m", -0.8))
            if not state["ped_cross_started"] and abs(ped_anchor - progress) <= cross_start_distance:
                state["ped_cross_started"] = True
            if state["ped_cross_started"] and not state["ped_cross_cleared"]:
                cross_duration = max(float(ped_win.get("cross_duration_seconds", 3.5)), dt)
                state["ped_cross_ratio"] = clip(state["ped_cross_ratio"] + dt / cross_duration, 0.0, 1.0)
                lat = start_lat + state["ped_cross_ratio"] * (end_lat - start_lat)
                self._move_walker_towards(
                    crossing_ped,
                    self._pedestrian_waypoint(route_tracker, ped_anchor),
                    desired_lateral_offset_m=lat,
                    dt=dt,
                    target_speed_mps=float(state.get("ped_cross_speed_mps", 1.8)),
                    z_lift_m=0.9,
                )
                final_wp = self._pedestrian_waypoint(route_tracker, ped_anchor)
                if final_wp is not None:
                    final_loc = _offset_location_from_waypoint(final_wp, lateral_offset_m=end_lat, z_lift_m=0.9)
                    if state["ped_cross_ratio"] >= 0.995 and crossing_ped.get_location().distance(final_loc) <= 0.45:
                        state["ped_cross_cleared"] = True
            elif not state["ped_cross_started"]:
                self._place_walker_at_lateral_offset(
                    crossing_ped,
                    route_tracker,
                    anchor_progress_m=ped_anchor,
                    lateral_offset_m=start_lat,
                    facing_direction_sign=end_lat - start_lat,
                    z_lift_m=0.9,
                )
            else:
                self._place_walker_at_lateral_offset(
                    crossing_ped,
                    route_tracker,
                    anchor_progress_m=ped_anchor,
                    lateral_offset_m=end_lat,
                    facing_direction_sign=end_lat - start_lat,
                    z_lift_m=0.9,
                )
            ped_dist = ego.get_location().distance(crossing_ped.get_location())
            state["ped_min_distance_m"] = min(state["ped_min_distance_m"], ped_dist)
            state["distance_to_pedestrian"] = ped_dist
            if _window_active(progress, ped_win, pad_before=60.0, pad_after=20.0) and ped_dist <= detection_distance:
                state["pedestrian_detected"] = True
                state["ped_slowdown_started"] = True

            ped_validation_span = float(scene_cfg.get("pedestrian_validation_span_m", 35.0))
            if (
                state["pedestrian_detected"]
                and abs(progress - ped_anchor) <= ped_validation_span
                and speed <= float(scene_cfg.get("pedestrian_stop_speed_kmh", 1.5))
            ):
                state["ped_hold_s"] += dt
            elif not state["safe_slowdown_completed"]:
                state["ped_hold_s"] = 0.0

            if (
                not state["safe_slowdown_completed"]
                and state["ped_cross_cleared"]
                and state["ped_hold_s"] >= float(scene_cfg.get("pedestrian_required_hold_seconds", 1.0))
                and state["ped_min_distance_m"] >= float(scene_cfg.get("pedestrian_min_safe_distance_m", 6.0))
                and ped_dist >= float(scene_cfg.get("pedestrian_min_safe_distance_m", 6.0))
            ):
                state["safe_slowdown_completed"] = True

        if route_tracker is not None and bus_stop_peds:
            bus_anchor = float(bus_win.get("anchor_progress_m", 5200.0))
            ped_start_distance = float(bus_win.get("pedestrian_start_distance_m", 45.0))
            ped_visible_distance = max(
                ped_start_distance,
                float(bus_win.get("pedestrian_visible_distance_m", 100.0)),
            )
            ped_activity_start_m = bus_anchor - ped_visible_distance
            if not state["bus_stop_ped_started"] and progress >= ped_activity_start_m:
                state["bus_stop_ped_started"] = True
            if state["bus_stop_ped_started"] and not state["bus_stop_pass_completed"]:
                span = max(
                    float(bus_win.get("pedestrian_activity_duration_seconds", bus_win.get("cross_duration_seconds", 3.0))),
                    8.0,
                )
                state["bus_stop_ped_ratio"] = clip(state["bus_stop_ped_ratio"] + dt / span, 0.0, 1.0)
                base_ratio = clip(
                    float(bus_win.get("pedestrian_initial_activity_ratio", 0.18)) + state["bus_stop_ped_ratio"],
                    0.0,
                    0.98,
                )
                for idx, (ped_stage, bus_stop_ped) in enumerate(bus_stop_peds):
                    start_lat, end_lat = self._bus_stop_pedestrian_offsets(ped_stage, bus_win)
                    actor_ratio = clip(base_ratio + 0.04 * (idx - 1.5), 0.04, 0.98)
                    desired_lat = start_lat + actor_ratio * (end_lat - start_lat)
                    self._move_walker_towards(
                        bus_stop_ped,
                        self._pedestrian_waypoint(route_tracker, self._bus_stop_pedestrian_progress(bus_anchor, idx, bus_win)),
                        desired_lateral_offset_m=desired_lat,
                        dt=dt,
                        target_speed_mps=float(state.get("bus_stop_ped_speed_mps", 1.2)),
                        z_lift_m=0.9,
                    )
            elif state["bus_stop_ped_started"]:
                for idx, (ped_stage, bus_stop_ped) in enumerate(bus_stop_peds):
                    _, desired_lat = self._bus_stop_pedestrian_offsets(ped_stage, bus_win)
                    self._move_walker_towards(
                        bus_stop_ped,
                        self._pedestrian_waypoint(route_tracker, self._bus_stop_pedestrian_progress(bus_anchor, idx, bus_win)),
                        desired_lateral_offset_m=desired_lat,
                        dt=dt,
                        target_speed_mps=0.8,
                        z_lift_m=0.9,
                    )
            else:
                for idx, (ped_stage, bus_stop_ped) in enumerate(bus_stop_peds):
                    start_lat, end_lat = self._bus_stop_pedestrian_offsets(ped_stage, bus_win)
                    self._place_walker_at_lateral_offset(
                        bus_stop_ped,
                        route_tracker,
                        anchor_progress_m=self._bus_stop_pedestrian_progress(bus_anchor, idx, bus_win),
                        lateral_offset_m=start_lat,
                        facing_direction_sign=end_lat - start_lat,
                        z_lift_m=0.9,
                    )

        if route_tracker is not None and slow_vehicle is not None:
            activate_progress = float(overtake_win.get("progress_start_m", 2300.0)) - float(
                scene_cfg.get("overtake_activation_margin_m", 180.0)
            )
            if state.get("return_to_lane_completed") and not state.get("slow_vehicle_retired"):
                slow_vehicle.set_transform(_offstage_transform())
                state["slow_vehicle_active"] = False
                state["slow_vehicle_retired"] = True
            elif state.get("slow_vehicle_retired"):
                slow_vehicle.set_transform(_offstage_transform())
            elif state["safe_slowdown_completed"] and progress >= activate_progress:
                if not state["slow_vehicle_initialized"]:
                    state["slow_vehicle_progress_m"] = progress + float(scene_cfg.get("slow_vehicle_initial_gap_m", 42.0))
                    state["slow_vehicle_initialized"] = True
                    state["slow_vehicle_active"] = True
                state["slow_vehicle_progress_m"] += (
                    float(overtake_win.get("slow_vehicle_speed_kmh", 20.0)) / 3.6
                ) * dt
                self._set_vehicle_on_lane(
                    slow_vehicle,
                    route_tracker,
                    float(state["slow_vehicle_progress_m"]),
                    lane_role="ego",
                    z_lift_m=0.3,
                )
            else:
                self._park_actor(
                    slow_vehicle,
                    route_tracker,
                    self._inactive_vehicle_progress(progress, "slow_vehicle", cfg),
                    lane_role="ego",
                    cfg=cfg,
                    stage="slow_vehicle",
                )

            front_gap = float(state["slow_vehicle_progress_m"] - progress) if state["slow_vehicle_initialized"] else 999.0
            front_dist = ego.get_location().distance(slow_vehicle.get_location()) if state["slow_vehicle_initialized"] else 999.0
            state["front_vehicle_gap"] = front_gap
            state["front_vehicle_distance"] = front_dist
            if state["slow_vehicle_active"] and 0.0 < front_gap <= float(scene_cfg.get("overtake_detection_gap_m", 55.0)):
                state["slow_vehicle_detected"] = True
                state["min_front_vehicle_gap_m"] = min(state["min_front_vehicle_gap_m"], front_gap)

            if (
                (state["slow_vehicle_detected"] or state["slow_vehicle_active"])
                and not state["lane_change_started"]
                and front_gap <= float(scene_cfg.get("lane_change_trigger_gap_m", 26.0))
            ):
                state["lane_change_started"] = True

            if state["lane_change_started"] and not state["overtake_completed"]:
                rear_gap_after_pass_m = -front_gap
                ego_faster_than_slow_vehicle = speed > float(overtake_win.get("slow_vehicle_speed_kmh", 20.0)) + float(
                    scene_cfg.get("return_speed_margin_kmh", 2.0)
                )
                left_lane_ready = float(obs["route_metrics"].get("lateral_offset_from_route_m", 0.0)) <= float(
                    scene_cfg.get("overtake_left_lane_ready_offset_m", -2.6)
                )
                if (
                    left_lane_ready
                    and front_gap < 0.0
                    and ego_faster_than_slow_vehicle
                ):
                    state["overtake_completed"] = True
                    if not state["return_to_lane_started"]:
                        state["return_to_lane_started"] = True
                        state["return_start_progress_m"] = progress
                        state["return_rear_gap_m"] = rear_gap_after_pass_m

            if state["overtake_completed"] and not state["return_to_lane_completed"]:
                if abs(float(obs["route_metrics"].get("lateral_offset_from_route_m", 0.0))) <= float(
                    scene_cfg.get("return_lane_tolerance_m", 0.25)
                ):
                    state["return_hold_s"] += dt
                else:
                    state["return_hold_s"] = 0.0
                if state["return_hold_s"] >= float(scene_cfg.get("return_required_hold_seconds", 1.0)):
                    state["return_to_lane_completed"] = True

        if route_tracker is not None and bus is not None:
            bus_anchor = float(bus_win.get("anchor_progress_m", 5200.0))
            bus_activate_progress = float(bus_win.get("progress_start_m", 5050.0)) - float(
                scene_cfg.get("bus_activation_margin_m", 120.0)
            )
            if progress >= bus_activate_progress:
                bus_tf = self._bus_stop_transform(route_tracker, bus_anchor, cfg, z_lift_m=0.35)
                if bus_tf is not None:
                    bus.set_transform(bus_tf)
                    state["bus_actor_active"] = True
                else:
                    state["bus_actor_active"] = self._set_vehicle_on_lane(
                        bus,
                        route_tracker,
                        bus_anchor,
                        lane_role="right_stop",
                        z_lift_m=0.3,
                    )
            else:
                self._park_actor(
                    bus,
                    route_tracker,
                    self._inactive_vehicle_progress(progress, "bus_stop_bus", cfg),
                    lane_role="right_stop",
                    cfg=cfg,
                    stage="bus_stop_bus",
                )

            bus_dist = ego.get_location().distance(bus.get_location())
            bus_progress_gap = bus_anchor - progress
            state["distance_to_bus_stop"] = bus_dist
            state["bus_stop_progress_gap_m"] = bus_progress_gap
            detection_distance = float(scene_cfg.get("bus_stop_detection_distance_m", 50.0))
            if (
                state["bus_actor_active"]
                and _window_active(progress, bus_win, pad_before=detection_distance, pad_after=20.0)
                and (
                    0.0 <= bus_progress_gap <= detection_distance
                    or bus_dist <= detection_distance
                )
            ):
                state["bus_stop_detected"] = True
                state["bus_stop_slowdown_started"] = True

            if (
                state["bus_stop_detected"]
                and _window_active(progress, bus_win)
                and speed <= float(scene_cfg.get("bus_stop_target_speed_kmh", 30.0)) + float(
                    scene_cfg.get("bus_stop_speed_tolerance_kmh", 4.0)
                )
            ):
                state["bus_stop_hold_s"] += dt
            elif not state["bus_stop_pass_completed"]:
                state["bus_stop_hold_s"] = 0.0

            if (
                not state["bus_stop_pass_completed"]
                and progress > bus_anchor + float(scene_cfg.get("bus_stop_pass_progress_margin_m", 45.0))
                and state["bus_stop_hold_s"] >= float(scene_cfg.get("bus_stop_required_hold_seconds", 2.0))
            ):
                state["bus_stop_pass_completed"] = True
                state["bus_stop_detected"] = False
                state["bus_stop_slowdown_started"] = False
                state["target_speed_kmh"] = state["cruise_target_speed_kmh"]

        phase = "cruise"
        desired_speed = state["cruise_target_speed_kmh"]
        target_lateral = 0.0

        pedestrian_stop_active = (
            not state["safe_slowdown_completed"]
            and _window_active(progress, ped_win, pad_before=60.0, pad_after=20.0)
            and state.get("pedestrian_detected")
        )

        if pedestrian_stop_active:
            phase = "pedestrian_caution"
            desired_speed = 0.0
        elif (
            not state["return_to_lane_completed"]
            and (
                state["lane_change_started"]
                or state.get("slow_vehicle_detected")
            )
        ):
            front_gap = float(state.get("front_vehicle_gap", 999.0))
            if not state["lane_change_started"]:
                phase = "follow_slow_vehicle"
                desired_speed = float(scene_cfg.get("follow_slow_vehicle_target_speed_kmh", 30.0))
                if front_gap <= float(scene_cfg.get("follow_brake_gap_m", 14.0)):
                    desired_speed = 20.0
            elif not state["overtake_completed"]:
                phase = "overtake_left"
                desired_speed = float(scene_cfg.get("overtake_target_speed_kmh", 30.0))
                target_lateral = float(scene_cfg.get("overtake_left_lateral_offset_m", -3.5))
            else:
                phase = "return_to_lane"
                desired_speed = float(scene_cfg.get("return_target_speed_kmh", 30.0))
                left_offset = float(scene_cfg.get("overtake_left_lateral_offset_m", -3.5))
                return_start = state.get("return_start_progress_m")
                if return_start is None:
                    return_start = progress
                    state["return_start_progress_m"] = return_start
                    state["return_to_lane_started"] = True
                return_distance = max(float(scene_cfg.get("return_lane_change_distance_m", 65.0)), 20.0)
                return_ratio = _smoothstep((progress - float(return_start)) / return_distance)
                target_lateral = left_offset * (1.0 - return_ratio)
        elif (
            not state["bus_stop_pass_completed"]
            and state.get("bus_stop_detected")
            and _window_active(
                progress,
                bus_win,
                pad_before=float(scene_cfg.get("bus_stop_detection_distance_m", 50.0)),
                pad_after=40.0,
            )
        ):
            phase = "bus_stop_caution"
            desired_speed = float(scene_cfg.get("bus_stop_target_speed_kmh", 30.0))

        state["phase"] = phase
        state["desired_speed_kmh"] = desired_speed
        state["target_lateral_offset_m"] = target_lateral

        ramp_cfg = ctrl.get("speed_target_ramp", {})
        state["target_speed_kmh"] = _rate_limit(
            float(state["target_speed_kmh"]),
            desired_speed,
            dt,
            float(ramp_cfg.get("accelerate_rate_kmh_per_s", 10.0)),
            float(ramp_cfg.get("decelerate_rate_kmh_per_s", 7.0)),
        )

        state["long_route_completed"] = max_progress >= float(scene_cfg.get("target_progress_m", 8000.0))
        if (
            state["safe_slowdown_completed"]
            and state["overtake_completed"]
            and state["return_to_lane_completed"]
            and state["bus_stop_pass_completed"]
            and state["long_route_completed"]
        ):
            state["success"] = True

        return state

    def compute_control(self, ego, actors, state, obs, cfg) -> Tuple[float, float, float]:
        route_tracker: RouteTracker = obs.get("route_tracker")
        progress = float(obs["route_metrics"]["route_progress_m"])
        ctrl = cfg.get("controller", {})
        phase = state.get("phase", "cruise")
        smoothing_cfg = ctrl.get("target_point_smoothing", {})
        lateral_offset = float(obs["route_metrics"].get("lateral_offset_from_route_m", 0.0))

        if phase == "pedestrian_caution":
            lookahead = float(ctrl.get("pedestrian_lookahead_m", 8.0))
            gain = float(ctrl.get("pedestrian_steer_gain", 1.45))
            max_steer = float(ctrl.get("pedestrian_max_steer", 0.40))
            window_radius = float(smoothing_cfg.get("turn_window_radius_m", 5.0))
            sigma = float(smoothing_cfg.get("turn_sigma_m", 2.5))
        elif phase == "follow_slow_vehicle":
            lookahead = float(ctrl.get("follow_lookahead_m", 10.0))
            gain = float(ctrl.get("follow_steer_gain", 1.35))
            max_steer = float(ctrl.get("follow_max_steer", 0.36))
            window_radius = float(smoothing_cfg.get("window_radius_m", 8.0))
            sigma = float(smoothing_cfg.get("sigma_m", 4.0))
        elif phase == "overtake_left":
            lookahead = float(ctrl.get("overtake_lookahead_m", 14.0))
            gain = float(ctrl.get("overtake_steer_gain", 1.18))
            max_steer = float(ctrl.get("overtake_max_steer", 0.34))
            window_radius = float(smoothing_cfg.get("window_radius_m", 8.0))
            sigma = float(smoothing_cfg.get("sigma_m", 4.0))
        elif phase == "return_to_lane":
            lookahead = float(ctrl.get("return_lookahead_m", 12.0))
            gain = float(ctrl.get("return_steer_gain", 1.45))
            max_steer = float(ctrl.get("return_max_steer", 0.36))
            window_radius = float(smoothing_cfg.get("window_radius_m", 8.0))
            sigma = float(smoothing_cfg.get("sigma_m", 4.0))
        elif phase == "bus_stop_caution":
            lookahead = float(ctrl.get("bus_stop_lookahead_m", 11.0))
            gain = float(ctrl.get("bus_stop_steer_gain", 1.35))
            max_steer = float(ctrl.get("bus_stop_max_steer", 0.34))
            window_radius = float(smoothing_cfg.get("window_radius_m", 8.0))
            sigma = float(smoothing_cfg.get("sigma_m", 4.0))
        else:
            lookahead = float(ctrl.get("cruise_lookahead_m", 12.0))
            gain = float(ctrl.get("cruise_steer_gain", 1.28))
            max_steer = float(ctrl.get("cruise_max_steer", 0.34))
            window_radius = float(smoothing_cfg.get("window_radius_m", 8.0))
            sigma = float(smoothing_cfg.get("sigma_m", 4.0))

        if route_tracker is not None:
            yaw_now = _route_yaw_at(route_tracker, progress, delta_m=float(ctrl.get("curvature_probe_delta_m", 4.0)))
            yaw_ahead = _route_yaw_at(
                route_tracker,
                progress + float(ctrl.get("curvature_lookahead_probe_m", 14.0)),
                delta_m=float(ctrl.get("curvature_probe_delta_m", 4.0)),
            )
            route_heading_delta = abs(_wrap_deg(yaw_ahead - yaw_now))
        else:
            route_heading_delta = 0.0

        if route_heading_delta >= float(ctrl.get("turn_detection_heading_delta_deg", 18.0)):
            lookahead = min(lookahead, float(ctrl.get("turn_lookahead_m", 7.0)))
            gain = max(gain, float(ctrl.get("turn_steer_gain", 1.55)))
            max_steer = max(max_steer, float(ctrl.get("turn_max_steer", 0.42)))
            window_radius = float(smoothing_cfg.get("turn_window_radius_m", 5.0))
            sigma = float(smoothing_cfg.get("turn_sigma_m", 2.5))

        if progress < 8.0 and float(obs["speed_kmh"]) < 15.0:
            max_steer = min(max_steer, 0.20)
            gain = min(gain, 0.90)

        desired_offset = float(state.get("target_lateral_offset_m", 0.0))
        if phase == "return_to_lane":
            correction_gain = float(ctrl.get("return_lateral_error_correction_gain", 1.25))
            correction_cap = float(ctrl.get("return_lateral_error_correction_cap_m", 3.0))
        else:
            correction_gain = float(ctrl.get("lateral_error_correction_gain", 0.85))
            correction_cap = float(ctrl.get("lateral_error_correction_cap_m", 1.5))
        corrected_offset = desired_offset - clip(correction_gain * lateral_offset, -correction_cap, correction_cap)

        lane_target_wp = None
        if route_tracker is not None and phase == "overtake_left":
            target_progress = progress + lookahead
            resolved = self._resolve_vehicle_waypoint_with_search(route_tracker, target_progress, lane_role="left_flow")
            lane_target_wp = resolved[0] if resolved is not None else None

        if route_tracker is not None:
            if lane_target_wp is not None:
                target_loc = lane_target_wp.transform.location
            else:
                target_loc = route_tracker.point_at_progress_smoothed(
                    progress + lookahead,
                    lateral_offset_m=corrected_offset,
                    window_radius_m=window_radius,
                    sample_step_m=float(smoothing_cfg.get("sample_step_m", 2.0)),
                    sigma_m=sigma,
                    tangent_delta_m=float(smoothing_cfg.get("tangent_delta_m", 2.0)),
                )
        else:
            target_loc = ego.get_location()

        steer = compute_steer_to_location(ego, target_loc, gain=gain, max_steer=max_steer)
        speed = float(obs["speed_kmh"])
        target_speed = float(state.get("target_speed_kmh", state.get("desired_speed_kmh", 0.0)))
        throttle, brake = _compute_speed_control(speed, target_speed, steer=steer)

        if phase == "pedestrian_caution":
            ped_dist = float(state.get("distance_to_pedestrian", 999.0))
            scene_cfg = cfg.get("success_criteria", {}).get("complex_scene2", {})
            stop_speed_kmh = float(scene_cfg.get("pedestrian_stop_speed_kmh", 1.5))
            if not state.get("safe_slowdown_completed"):
                if speed > stop_speed_kmh + 1.5:
                    return 0.0, 0.95, steer
                if ped_dist <= float(scene_cfg.get("pedestrian_detection_distance_m", 38.0)) or float(state.get("ped_cross_ratio", 0.0)) < 0.995:
                    return 0.0, 0.65, steer
                return 0.0, 0.35, steer

        if phase == "follow_slow_vehicle":
            front_gap = float(state.get("front_vehicle_gap", 999.0))
            if front_gap <= float(cfg.get("success_criteria", {}).get("complex_scene2", {}).get("follow_brake_gap_m", 14.0)):
                return 0.0, 0.35, steer

        return throttle, brake, steer

    def is_success(self, state, obs, cfg) -> bool:
        return bool(state["success"])

    def extra_record(self, ego, actors, state, obs, cfg) -> Dict[str, Any]:
        return {
            "scene2_phase": state.get("phase"),
            "target_speed_kmh": state.get("target_speed_kmh"),
            "desired_speed_kmh": state.get("desired_speed_kmh"),
            "target_lateral_offset_m": state.get("target_lateral_offset_m"),
            "distance_to_pedestrian": state.get("distance_to_pedestrian"),
            "pedestrian_detected": state.get("pedestrian_detected"),
            "ped_slowdown_started": state.get("ped_slowdown_started"),
            "safe_slowdown_completed": state.get("safe_slowdown_completed"),
            "pedestrian_hold_time": state.get("ped_hold_s"),
            "min_distance_to_pedestrian_so_far": (
                None
                if math.isinf(float(state.get("ped_min_distance_m", float("inf"))))
                else state.get("ped_min_distance_m")
            ),
            "slow_vehicle_detected": state.get("slow_vehicle_detected"),
            "slow_vehicle_retired": state.get("slow_vehicle_retired"),
            "lane_change_started": state.get("lane_change_started"),
            "overtake_completed": state.get("overtake_completed"),
            "return_to_lane_started": state.get("return_to_lane_started"),
            "return_start_progress_m": state.get("return_start_progress_m"),
            "return_rear_gap_m": state.get("return_rear_gap_m"),
            "return_to_lane_completed": state.get("return_to_lane_completed"),
            "return_hold_time": state.get("return_hold_s"),
            "front_vehicle_gap": state.get("front_vehicle_gap"),
            "front_vehicle_distance": state.get("front_vehicle_distance"),
            "min_front_vehicle_gap_so_far": (
                None
                if math.isinf(float(state.get("min_front_vehicle_gap_m", float("inf"))))
                else state.get("min_front_vehicle_gap_m")
            ),
            "bus_stop_detected": state.get("bus_stop_detected"),
            "bus_stop_slowdown_started": state.get("bus_stop_slowdown_started"),
            "bus_stop_pass_completed": state.get("bus_stop_pass_completed"),
            "bus_stop_hold_time": state.get("bus_stop_hold_s"),
            "distance_to_bus_stop": state.get("distance_to_bus_stop"),
            "bus_stop_progress_gap_m": state.get("bus_stop_progress_gap_m"),
            "long_route_completed": state.get("long_route_completed"),
            "task_success": state.get("success"),
        }
