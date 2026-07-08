import argparse
import json
import math
from pathlib import Path

import carla
import yaml

try:
    from carla_eval.runtime_metrics import (
        LaneInvasionTracker,
        RedLightViolationTracker,
        RouteTracker,
        apply_weather_from_config,
        get_controller_param,
        get_instruction_trigger_time,
        load_world_for_config,
        make_transform_from_config,
    )
except ModuleNotFoundError:
    from runtime_metrics import (
        LaneInvasionTracker,
        RedLightViolationTracker,
        RouteTracker,
        apply_weather_from_config,
        get_controller_param,
        get_instruction_trigger_time,
        load_world_for_config,
        make_transform_from_config,
    )


def clip(x, low, high):
    return max(low, min(high, x))


def get_speed_kmh(vehicle):
    v = vehicle.get_velocity()
    return 3.6 * (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5


def get_waypoint(carla_map, location):
    return carla_map.get_waypoint(
        location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )


def is_valid_adjacent_lane(wp, adj_wp):
    if wp is None or adj_wp is None:
        return False

    if adj_wp.lane_type != carla.LaneType.Driving:
        return False

    if wp.lane_id * adj_wp.lane_id <= 0:
        return False

    return True


def dot2d(vec, direction):
    return vec.x * direction.x + vec.y * direction.y


def horizontal_distance(a, b):
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5


def fmt(x):
    if x is None:
        return "None"
    return f"{x:.1f}"


def compute_steer_to_location(vehicle, target_loc, steer_gain=1.35, max_steer=0.55):
    tf = vehicle.get_transform()
    loc = tf.location

    forward = tf.get_forward_vector()
    right = tf.get_right_vector()

    dx = target_loc.x - loc.x
    dy = target_loc.y - loc.y

    forward_dot = dx * forward.x + dy * forward.y
    right_dot = dx * right.x + dy * right.y

    angle = math.atan2(right_dot, forward_dot)
    return clip(steer_gain * angle, -max_steer, max_steer)


def compute_control(
    speed_kmh,
    target_speed_kmh,
    min_distance_to_cone,
    min_safe_distance,
    steer=0.0,
    large_steer_threshold=0.35,
    large_steer_throttle_cap=0.20,
    launch_speed_threshold_kmh=4.0,
    launch_throttle_floor=0.30,
):
    if min_distance_to_cone is not None and min_distance_to_cone < min_safe_distance:
        return 0.0, 0.80

    error = target_speed_kmh - speed_kmh

    if error > 8.0:
        throttle, brake = 0.45, 0.0
    elif error > 3.0:
        throttle, brake = 0.30, 0.0
    elif error < -6.0:
        throttle, brake = 0.0, 0.18
    else:
        throttle, brake = 0.20, 0.0

    if (
        brake <= 1e-6
        and speed_kmh < launch_speed_threshold_kmh
        and target_speed_kmh > speed_kmh + 1.0
    ):
        throttle = max(throttle, launch_throttle_floor)
    elif brake <= 1e-6 and abs(steer) > large_steer_threshold:
        throttle = min(throttle, large_steer_throttle_cap)

    return throttle, brake


def get_next_target_location(wp, lookahead):
    next_wps = wp.next(lookahead)
    if not next_wps:
        return wp.transform.location
    return next_wps[0].transform.location


def get_lane_waypoint_at_same_s(carla_map, current_wp, target_road_id, target_lane_id):
    if current_wp is None:
        return None

    if current_wp.road_id == target_road_id and current_wp.lane_id == target_lane_id:
        return current_wp

    if current_wp.lane_id == target_lane_id:
        return current_wp

    try:
        adjacent_candidates = (current_wp.get_left_lane(), current_wp.get_right_lane())
    except RuntimeError:
        adjacent_candidates = ()

    for candidate in adjacent_candidates:
        if candidate is None:
            continue
        if candidate.lane_type != carla.LaneType.Driving:
            continue
        if candidate.road_id == target_road_id and candidate.lane_id == target_lane_id:
            return candidate

    try:
        lane_wp = carla_map.get_waypoint_xodr(
            int(target_road_id),
            int(target_lane_id),
            float(current_wp.s),
        )
    except RuntimeError:
        lane_wp = None

    if lane_wp is not None and lane_wp.lane_type == carla.LaneType.Driving:
        return lane_wp

    return None


def same_lane_identity(wp_a, wp_b):
    if wp_a is None or wp_b is None:
        return False
    return wp_a.road_id == wp_b.road_id and wp_a.lane_id == wp_b.lane_id


def lane_center_error(ego_loc, lane_wp):
    if ego_loc is None or lane_wp is None:
        return None
    return ego_loc.distance(lane_wp.transform.location)


def make_parallel_target_location(ref_loc, ref_forward, ref_right, progress_m, lateral_offset_m):
    return carla.Location(
        x=ref_loc.x + progress_m * ref_forward.x + lateral_offset_m * ref_right.x,
        y=ref_loc.y + progress_m * ref_forward.y + lateral_offset_m * ref_right.y,
        z=ref_loc.z,
    )


def ramp_towards(value, target_magnitude, rate_per_progress, progress_delta):
    return min(target_magnitude, value + max(0.0, progress_delta) * rate_per_progress)


def get_lane_follow_target_location(carla_map, current_wp, target_road_id, target_lane_id, lookahead):
    lane_wp = get_lane_waypoint_at_same_s(
        carla_map,
        current_wp,
        target_road_id,
        target_lane_id,
    )
    if lane_wp is None:
        lane_wp = current_wp
    if lane_wp is None:
        return carla.Location()
    return get_next_target_location(lane_wp, lookahead)


def get_route_waypoint_at_progress(route_tracker, carla_map, progress_m):
    route_loc = route_tracker.point_at_progress(progress_m)
    return get_waypoint(carla_map, route_loc)


def get_route_lane_waypoint_at_progress(
    route_tracker,
    carla_map,
    progress_m,
    target_road_id,
    target_lane_id,
    fallback_wp=None,
):
    route_wp = get_route_waypoint_at_progress(route_tracker, carla_map, progress_m)
    lane_wp = get_lane_waypoint_at_same_s(carla_map, route_wp, target_road_id, target_lane_id)
    if lane_wp is None:
        lane_wp = get_lane_waypoint_at_same_s(carla_map, fallback_wp, target_road_id, target_lane_id)
    return lane_wp


def get_route_lane_target_location(
    route_tracker,
    carla_map,
    progress_m,
    target_road_id,
    target_lane_id,
    lookahead,
    fallback_wp=None,
):
    lane_wp = get_route_lane_waypoint_at_progress(
        route_tracker,
        carla_map,
        progress_m,
        target_road_id,
        target_lane_id,
        fallback_wp=fallback_wp,
    )
    if lane_wp is None:
        lane_wp = fallback_wp
    if lane_wp is None:
        return carla.Location()
    # 这里的 progress_m 应该是“当前沿 route 的参考进度”。
    # 再向前取一次 waypoint.next(lookahead) 就足够了，避免双重前瞻把目标点打到过远位置。
    return get_next_target_location(lane_wp, lookahead)


def make_cone_transform(ego_spawn, longitudinal_distance_m, lateral_offset_m):
    """
    锥桶放在 ego 原始直行轨迹上。
    不用 waypoint.next()，避免在路口被 CARLA 选到右转分支。
    """
    forward = ego_spawn.get_forward_vector()
    right = ego_spawn.get_right_vector()

    loc = carla.Location(
        x=ego_spawn.location.x + longitudinal_distance_m * forward.x + lateral_offset_m * right.x,
        y=ego_spawn.location.y + longitudinal_distance_m * forward.y + lateral_offset_m * right.y,
        z=ego_spawn.location.z + 0.20,
    )

    return carla.Transform(
        loc,
        carla.Rotation(
            pitch=0.0,
            yaw=ego_spawn.rotation.yaw,
            roll=0.0,
        ),
    )


def cone_observation_in_original_lane(
    ego_loc,
    cone_locs,
    ref_loc,
    ref_forward,
    ref_right,
    detection_distance,
    corridor_half_width,
):
    """
    模拟感知：每一帧只判断“当前 ego 前方原始行驶走廊内”是否有锥桶。

    不是用最后一个锥桶的位置做触发。
    """
    ego_rel = carla.Location(
        x=ego_loc.x - ref_loc.x,
        y=ego_loc.y - ref_loc.y,
        z=0.0,
    )
    ego_progress = dot2d(ego_rel, ref_forward)

    detected = []

    for idx, c_loc in enumerate(cone_locs):
        cone_rel = carla.Location(
            x=c_loc.x - ref_loc.x,
            y=c_loc.y - ref_loc.y,
            z=0.0,
        )
        cone_progress = dot2d(cone_rel, ref_forward)
        cone_lateral = dot2d(cone_rel, ref_right)

        gap_ahead = cone_progress - ego_progress

        # 只看当前前方感知范围内、且位于原始车道走廊内的锥桶
        if 0.0 <= gap_ahead <= detection_distance and abs(cone_lateral) <= corridor_half_width:
            detected.append({
                "index": idx,
                "gap_ahead": gap_ahead,
                "cone_lateral": cone_lateral,
                "distance": horizontal_distance(ego_loc, c_loc),
            })

    detected.sort(key=lambda x: x["gap_ahead"])
    return detected, ego_progress


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--spawn-index", type=int, default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--target-speed", type=float, default=None)
    parser.add_argument("--lookahead", type=float, default=None)
    parser.add_argument("--log-id", type=str, default=None)
    parser.add_argument(
        "--config",
        type=str,
        default="configs/scenarios/complex_obstacle/S05_cone_detour.yaml",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["__config_path__"] = str(config_path.resolve())

    scenario_id = cfg.get("scenario_id", config_path.stem)
    category = cfg.get("category", "complex_obstacle")
    runtime_cfg = cfg.get("runtime", {})
    eval_cfg = cfg.get("evaluation", {})
    detour_cfg = cfg.get("success_criteria", {}).get("cone_detour", {})
    primary_instruction_id = cfg.get("instructions", [{}])[0].get("id", "cmd_001")
    trigger_time = get_instruction_trigger_time(cfg, default=5.0)
    target_speed = float(args.target_speed if args.target_speed is not None else get_controller_param(cfg, "target_speed_kmh", 30.0))
    lookahead = float(args.lookahead if args.lookahead is not None else get_controller_param(cfg, "lookahead_m", 14.0))
    approach_lookahead = float(get_controller_param(cfg, "approach_lookahead_m", lookahead))
    change_left_lookahead = float(get_controller_param(cfg, "change_left_lookahead_m", min(lookahead, 8.0)))
    follow_left_lookahead = float(get_controller_param(cfg, "follow_left_lookahead_m", lookahead))
    return_lookahead = float(get_controller_param(cfg, "return_lookahead_m", min(lookahead, 10.0)))
    complete_lookahead = float(get_controller_param(cfg, "complete_lookahead_m", lookahead))
    approach_target_speed = float(get_controller_param(cfg, "approach_target_speed_kmh", min(target_speed, 30.0)))
    change_left_target_speed = float(get_controller_param(cfg, "change_left_target_speed_kmh", min(target_speed, 18.0)))
    follow_left_target_speed = float(get_controller_param(cfg, "follow_left_target_speed_kmh", min(target_speed, 20.0)))
    return_target_speed = float(get_controller_param(cfg, "return_target_speed_kmh", min(target_speed, 24.0)))
    complete_target_speed = float(get_controller_param(cfg, "complete_target_speed_kmh", min(target_speed, 24.0)))
    approach_steer_gain = float(get_controller_param(cfg, "approach_steer_gain", 1.35))
    change_left_steer_gain = float(get_controller_param(cfg, "change_left_steer_gain", 1.55))
    follow_left_steer_gain = float(get_controller_param(cfg, "follow_left_steer_gain", 1.20))
    return_steer_gain = float(get_controller_param(cfg, "return_steer_gain", 1.55))
    complete_steer_gain = float(get_controller_param(cfg, "complete_steer_gain", 1.35))
    max_steer = float(get_controller_param(cfg, "max_steer", 0.55))
    approach_max_steer = float(get_controller_param(cfg, "approach_max_steer", min(max_steer, 0.35)))
    change_left_max_steer = float(get_controller_param(cfg, "change_left_max_steer", min(max_steer, 0.40)))
    follow_left_max_steer = float(get_controller_param(cfg, "follow_left_max_steer", min(max_steer, 0.25)))
    return_max_steer = float(get_controller_param(cfg, "return_max_steer", min(max_steer, 0.38)))
    complete_max_steer = float(get_controller_param(cfg, "complete_max_steer", min(max_steer, 0.28)))
    large_steer_threshold = float(get_controller_param(cfg, "large_steer_threshold", 0.35))
    large_steer_throttle_cap = float(get_controller_param(cfg, "large_steer_throttle_cap", 0.20))
    launch_speed_threshold_kmh = float(get_controller_param(cfg, "launch_speed_threshold_kmh", 4.0))
    launch_throttle_floor = float(get_controller_param(cfg, "launch_throttle_floor", 0.30))
    change_left_lateral_rate = float(get_controller_param(cfg, "change_left_lateral_rate_m_per_m", 0.45))
    follow_left_target_lateral = float(get_controller_param(cfg, "follow_left_target_lateral_offset_m", detour_cfg.get("detour_lateral_offset_m", 3.5)))
    return_lateral_rate = float(get_controller_param(cfg, "return_lateral_rate_m_per_m", 0.55))
    duration = float(args.duration if args.duration is not None else runtime_cfg.get("max_duration_seconds", 45.0))
    post_success_hold_seconds = float(runtime_cfg.get("post_success_hold_seconds", 5.0))
    route_corridor_half_width = float(eval_cfg.get("route_corridor_half_width_m", 6.0))
    red_light_lane_tolerance = float(eval_cfg.get("red_light_lane_tolerance_m", 6.0))

    detection_distance = float(detour_cfg.get("detection_distance_m", 35.0))
    detour_start_distance = float(detour_cfg.get("detour_start_distance_m", 30.0))
    detour_lateral_offset = float(detour_cfg.get("detour_lateral_offset_m", 3.5))
    min_safe_distance = float(detour_cfg.get("min_safe_distance_to_cone_m", 1.5))
    detour_lane_center_tolerance = float(detour_cfg.get("detour_lane_center_tolerance_m", 1.0))
    detour_lane_hold_required = float(detour_cfg.get("detour_lane_hold_seconds", 0.5))
    return_lane_tolerance = float(detour_cfg.get("return_lane_tolerance_m", 1.2))
    required_return_hold = float(detour_cfg.get("required_return_hold_seconds", 1.0))
    no_cone_ahead_hold_required = float(detour_cfg.get("no_cone_ahead_hold_seconds", 1.0))
    corridor_half_width = float(detour_cfg.get("detection_corridor_half_width_m", 1.8))

    log_scenario_id = args.log_id or scenario_id
    out_dir = Path("logs") / category / log_scenario_id
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "frames.jsonl"

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = load_world_for_config(client, cfg)
    apply_weather_from_config(world, cfg)
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = float(runtime_cfg.get("fixed_delta_seconds", 0.05))
    world.apply_settings(settings)
    dt = float(settings.fixed_delta_seconds)

    ego = None
    cones = []
    collision_sensor = None
    lane_invasion_tracker = None

    collision_info = {
        "value": False,
        "other_actor": None,
    }

    try:
        old_actors = []
        old_actors.extend(world.get_actors().filter("vehicle.*"))
        old_actors.extend(world.get_actors().filter("walker.*"))
        old_actors.extend(world.get_actors().filter("sensor.*"))
        old_actors.extend(world.get_actors().filter("static.prop.trafficcone*"))
        for actor in old_actors:
            actor.destroy()

        world.tick()

        spawn = make_transform_from_config(cfg)
        spawn_source = "config"
        spawn_index = None

        if args.spawn_index is not None:
            spawn_points = carla_map.get_spawn_points()
            spawn_index = args.spawn_index % len(spawn_points)
            spawn = spawn_points[spawn_index]
            spawn.location.z += 0.5
            spawn_source = f"spawn_index={spawn_index}"

        spawn_wp = get_waypoint(carla_map, spawn.location)
        if spawn_wp is None:
            raise RuntimeError("Failed to find driving-lane waypoint for S05 spawn point.")
        detour_lane_wp = spawn_wp.get_left_lane()
        if not is_valid_adjacent_lane(spawn_wp, detour_lane_wp):
            raise RuntimeError(
                "S05 spawn point does not have a valid left adjacent driving lane for one-lane detour."
            )

        original_road_id = spawn_wp.road_id
        original_lane_id = spawn_wp.lane_id
        detour_road_id = detour_lane_wp.road_id
        detour_lane_id = detour_lane_wp.lane_id

        vehicle_bp = bp_lib.find(cfg.get("ego", {}).get("vehicle_type", "vehicle.tesla.model3"))
        ego = world.try_spawn_actor(vehicle_bp, spawn)
        if ego is None:
            raise RuntimeError("Failed to spawn ego.")

        for _ in range(20):
            world.tick()

        ref_loc = carla.Location(x=spawn.location.x, y=spawn.location.y, z=spawn.location.z)
        ref_forward = spawn.get_forward_vector()
        ref_right = spawn.get_right_vector()

        print(f"[INFO] map = {carla_map.name}")
        print(f"[INFO] spawn source = {spawn_source}")
        print(f"[INFO] ego spawn = ({spawn.location.x:.2f}, {spawn.location.y:.2f}, {spawn.location.z:.2f})")
        print(
            f"[INFO] original road/lane = {original_road_id}/{original_lane_id}, "
            f"detour target road/lane = {detour_road_id}/{detour_lane_id}"
        )
        print("[INFO] S05 trigger mode = detection-based state machine, no oracle max-cone position")

        cone_bp = bp_lib.find("static.prop.trafficcone01")
        cone_locs = []

        for prop in cfg.get("actors", {}).get("static_props", []):
            rel = prop["relative_to_ego"]
            longitudinal = float(rel.get("longitudinal_distance_m", 30.0))
            lateral = float(rel.get("lateral_offset_m", 0.0))

            tf = make_cone_transform(spawn, longitudinal, lateral)
            cone = world.try_spawn_actor(cone_bp, tf)
            if cone is None:
                raise RuntimeError(f"Failed to spawn {prop.get('id')}")

            cones.append(cone)
            cone_locs.append(carla.Location(x=tf.location.x, y=tf.location.y, z=tf.location.z))

            print(
                f"[INFO] cone planned: {prop.get('id')} "
                f"longitudinal={longitudinal:.1f}m lateral={lateral:.2f}m "
                f"loc=({tf.location.x:.2f}, {tf.location.y:.2f}, {tf.location.z:.2f})"
            )

        for _ in range(10):
            world.tick()

        collision_bp = bp_lib.find("sensor.other.collision")
        collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=ego)

        def on_collision(event):
            collision_info["value"] = True
            collision_info["other_actor"] = event.other_actor.type_id

        collision_sensor.listen(on_collision)
        lane_invasion_tracker = LaneInvasionTracker(world, bp_lib, ego)
        route_tracker = RouteTracker.from_route_config(
            carla_map,
            cfg,
            corridor_half_width_m=route_corridor_half_width,
        )
        red_light_tracker = RedLightViolationTracker(
            world,
            route_tracker.ref_loc,
            route_tracker.ref_forward,
            route_tracker.ref_right,
            lane_tolerance_m=red_light_lane_tolerance,
        )

        max_frames = int(duration / dt)

        state = "APPROACH"

        first_detection_time = None
        detour_started_time = None
        detour_completed_time = None
        return_to_lane_time = None
        success_time = None
        detour_started_progress = None
        detour_completed_progress = None

        cone_detected_once = False
        detour_started_once = False
        detour_completed = False
        return_to_lane_completed = False

        no_cone_ahead_hold_time = 0.0
        detour_lane_hold_time = 0.0
        return_hold_time = 0.0
        max_original_corridor_progress = 0.0
        min_distance_to_cone_seen = float("inf")
        max_lateral_offset_seen = 0.0

        print(f"[START] Running {log_scenario_id}")
        print(f"[INFO] config = {config_path}")
        print(f"[INFO] trigger_time = {trigger_time} s")
        print(f"[INFO] target_speed = {target_speed} km/h")
        print(f"[INFO] lookahead = {lookahead} m")
        print(
            f"[INFO] stage lookahead = approach:{approach_lookahead}m "
            f"change_left:{change_left_lookahead}m "
            f"follow_left:{follow_left_lookahead}m "
            f"return:{return_lookahead}m complete:{complete_lookahead}m"
        )
        print(f"[INFO] detection_distance = {detection_distance} m")
        print(f"[INFO] detour_start_distance = {detour_start_distance} m")
        print(f"[INFO] detour_lateral_offset = {detour_lateral_offset} m")
        print(f"[INFO] no_cone_ahead_hold_required = {no_cone_ahead_hold_required} s")
        print(f"[INFO] log_path = {log_path}")

        with log_path.open("w", encoding="utf-8") as f:
            for frame in range(max_frames):
                timestamp = frame * dt

                ego_loc = ego.get_location()
                current_wp = get_waypoint(carla_map, ego_loc)
                current_road_id = current_wp.road_id if current_wp is not None else None
                current_lane_id = current_wp.lane_id if current_wp is not None else None
                route_metrics_before = route_tracker.measure(ego_loc)
                route_progress_for_target = route_metrics_before["max_route_progress_m"]

                detected_cones, ego_progress = cone_observation_in_original_lane(
                    ego_loc=ego_loc,
                    cone_locs=cone_locs,
                    ref_loc=ref_loc,
                    ref_forward=ref_forward,
                    ref_right=ref_right,
                    detection_distance=detection_distance,
                    corridor_half_width=corridor_half_width,
                )

                cone_detected = len(detected_cones) > 0
                nearest_gap = detected_cones[0]["gap_ahead"] if detected_cones else None
                nearest_cone_distance = detected_cones[0]["distance"] if detected_cones else None

                all_distances = [horizontal_distance(ego_loc, c_loc) for c_loc in cone_locs]
                current_min_distance = min(all_distances) if all_distances else None
                if current_min_distance is not None:
                    min_distance_to_cone_seen = min(min_distance_to_cone_seen, current_min_distance)

                lane_center = carla.Location(
                    x=ref_loc.x + ego_progress * ref_forward.x,
                    y=ref_loc.y + ego_progress * ref_forward.y,
                    z=ref_loc.z,
                )
                lateral_vec = carla.Location(
                    x=ego_loc.x - lane_center.x,
                    y=ego_loc.y - lane_center.y,
                    z=0.0,
                )
                lateral_offset = dot2d(lateral_vec, ref_right)
                max_original_corridor_progress = max(max_original_corridor_progress, ego_progress)
                max_lateral_offset_seen = max(max_lateral_offset_seen, abs(lateral_offset))
                detour_lane_center_error = abs(lateral_offset + detour_lateral_offset)
                original_lane_center_error = abs(lateral_offset)
                in_detour_lane = detour_lane_center_error <= detour_lane_center_tolerance
                in_original_lane = original_lane_center_error <= return_lane_tolerance

                if cone_detected and not cone_detected_once:
                    cone_detected_once = True
                    first_detection_time = timestamp
                    print(
                        f"[EVENT] cone_detected at t={timestamp:.1f}s, "
                        f"gap={nearest_gap:.1f}m, dist={nearest_cone_distance:.1f}m"
                    )

                if state == "APPROACH":
                    target_lateral_offset = 0.0
                    current_target_speed = approach_target_speed
                    current_lookahead = approach_lookahead
                    steer_gain = approach_steer_gain
                    current_max_steer = approach_max_steer

                    if cone_detected and nearest_gap is not None and nearest_gap <= detour_start_distance:
                        state = "CHANGE_LEFT"
                        detour_started_once = True
                        detour_started_time = timestamp
                        detour_started_progress = max_original_corridor_progress
                        print(
                            f"[EVENT] detour_started at t={timestamp:.1f}s, "
                            f"gap={nearest_gap:.1f}m, dist={nearest_cone_distance:.1f}m"
                        )

                if state == "CHANGE_LEFT":
                    detour_forward_progress = (
                        max_original_corridor_progress - detour_started_progress
                        if detour_started_progress is not None
                        else 0.0
                    )
                    target_lateral_offset = -ramp_towards(
                        0.0,
                        detour_lateral_offset,
                        change_left_lateral_rate,
                        detour_forward_progress,
                    )
                    current_target_speed = change_left_target_speed
                    current_lookahead = change_left_lookahead
                    steer_gain = change_left_steer_gain
                    current_max_steer = change_left_max_steer

                    if (
                        in_detour_lane
                    ):
                        detour_lane_hold_time += dt
                    else:
                        detour_lane_hold_time = 0.0

                    if detour_lane_hold_time >= detour_lane_hold_required:
                        state = "FOLLOW_LEFT"

                elif state == "FOLLOW_LEFT":
                    target_lateral_offset = -min(detour_lateral_offset, follow_left_target_lateral)
                    current_target_speed = follow_left_target_speed
                    current_lookahead = follow_left_lookahead
                    steer_gain = follow_left_steer_gain
                    current_max_steer = follow_left_max_steer

                    if cone_detected:
                        no_cone_ahead_hold_time = 0.0
                    else:
                        no_cone_ahead_hold_time += dt

                    if no_cone_ahead_hold_time >= no_cone_ahead_hold_required:
                        state = "RETURN_TO_LANE"
                        detour_completed = True
                        detour_completed_time = timestamp
                        detour_completed_progress = max_original_corridor_progress
                        print(
                            f"[EVENT] detour_completed at t={timestamp:.1f}s, "
                            f"reason=no_cone_ahead_for_{no_cone_ahead_hold_time:.1f}s, "
                            f"min_dist={min_distance_to_cone_seen:.1f}m, "
                            f"max_lat={max_lateral_offset_seen:.1f}m"
                        )

                elif state == "RETURN_TO_LANE":
                    return_forward_progress = (
                        max_original_corridor_progress - detour_completed_progress
                        if detour_completed_progress is not None
                        else 0.0
                    )
                    remaining_lateral = max(0.0, detour_lateral_offset - return_forward_progress * return_lateral_rate)
                    target_lateral_offset = -remaining_lateral
                    current_target_speed = return_target_speed
                    current_lookahead = return_lookahead
                    steer_gain = return_steer_gain
                    current_max_steer = return_max_steer

                    if (
                        in_original_lane
                    ):
                        return_hold_time += dt
                    else:
                        return_hold_time = 0.0

                    if (
                        return_hold_time >= required_return_hold
                        and not collision_info["value"]
                        and min_distance_to_cone_seen >= min_safe_distance
                    ):
                        state = "COMPLETE"
                        return_to_lane_completed = True
                        return_to_lane_time = timestamp
                        success_time = timestamp
                        print(
                            f"[SUCCESS] return to lane completed at t={timestamp:.1f}s, "
                            f"return_hold={return_hold_time:.1f}s, "
                            f"min_dist={min_distance_to_cone_seen:.1f}m, "
                            f"continuing for {post_success_hold_seconds:.1f}s."
                        )

                elif state == "COMPLETE":
                    target_lateral_offset = 0.0
                    current_target_speed = complete_target_speed
                    current_lookahead = complete_lookahead
                    steer_gain = complete_steer_gain
                    current_max_steer = complete_max_steer

                target_progress = max_original_corridor_progress + current_lookahead
                target_loc = make_parallel_target_location(
                    ref_loc,
                    ref_forward,
                    ref_right,
                    target_progress,
                    target_lateral_offset,
                )

                speed_kmh = get_speed_kmh(ego)
                steer = compute_steer_to_location(
                    ego,
                    target_loc,
                    steer_gain=steer_gain,
                    max_steer=current_max_steer,
                )
                path_blocking_cone_distance = None
                if state in {"APPROACH", "CHANGE_LEFT"} and not in_detour_lane:
                    path_blocking_cone_distance = nearest_cone_distance

                throttle, brake = compute_control(
                    speed_kmh=speed_kmh,
                    target_speed_kmh=current_target_speed,
                    min_distance_to_cone=path_blocking_cone_distance,
                    min_safe_distance=min_safe_distance,
                    steer=steer,
                    large_steer_threshold=large_steer_threshold,
                    large_steer_throttle_cap=large_steer_throttle_cap,
                    launch_speed_threshold_kmh=launch_speed_threshold_kmh,
                    launch_throttle_floor=launch_throttle_floor,
                )

                control = carla.VehicleControl()
                control.steer = float(steer)
                control.throttle = float(throttle)
                control.brake = float(brake)
                control.hand_brake = False
                control.reverse = False
                ego.apply_control(control)

                world.tick()

                ego_loc_after = ego.get_location()
                speed_kmh_after = get_speed_kmh(ego)
                lane_invasion_metrics = lane_invasion_tracker.snapshot()
                route_metrics = route_tracker.measure(ego_loc_after)
                red_light_metrics = red_light_tracker.update(ego_loc_after, speed_kmh_after)
                current_wp_after = get_waypoint(carla_map, ego_loc_after)
                current_road_id_after = current_wp_after.road_id if current_wp_after is not None else None
                current_lane_id_after = current_wp_after.lane_id if current_wp_after is not None else None
                ego_rel_after = carla.Location(
                    x=ego_loc_after.x - ref_loc.x,
                    y=ego_loc_after.y - ref_loc.y,
                    z=0.0,
                )
                ego_progress_after = dot2d(ego_rel_after, ref_forward)
                lane_center_after = carla.Location(
                    x=ref_loc.x + ego_progress_after * ref_forward.x,
                    y=ref_loc.y + ego_progress_after * ref_forward.y,
                    z=ref_loc.z,
                )
                lateral_vec_after = carla.Location(
                    x=ego_loc_after.x - lane_center_after.x,
                    y=ego_loc_after.y - lane_center_after.y,
                    z=0.0,
                )
                lateral_offset_after = dot2d(lateral_vec_after, ref_right)
                detour_lane_center_error_after = abs(lateral_offset_after + detour_lateral_offset)
                original_lane_center_error_after = abs(lateral_offset_after)
                excessive_lateral_departure_after = abs(lateral_offset_after) > (detour_lateral_offset + 2.5)

                ego_tf = ego.get_transform()
                forward = ego_tf.get_forward_vector()
                spectator = world.get_spectator()
                spectator.set_transform(
                    carla.Transform(
                        carla.Location(
                            x=ego_loc_after.x - 10.0 * forward.x,
                            y=ego_loc_after.y - 10.0 * forward.y,
                            z=ego_loc_after.z + 5.0,
                        ),
                        carla.Rotation(pitch=-20.0, yaw=ego_tf.rotation.yaw, roll=0.0),
                    )
                )

                record = {
                    "timestamp": timestamp,
                    "frame": frame,
                    "scenario_id": log_scenario_id,
                    "state": state,
                    "instruction_id": primary_instruction_id if timestamp >= trigger_time else None,

                    "ego_x": ego_loc_after.x,
                    "ego_y": ego_loc_after.y,
                    "ego_z": ego_loc_after.z,
                    "ego_speed_kmh": speed_kmh_after,

                    "steer": control.steer,
                    "throttle": control.throttle,
                    "brake": control.brake,

                    "collision": collision_info["value"],
                    "collision_other_actor": collision_info["other_actor"],
                    "lane_invasion": lane_invasion_metrics["lane_invasion"],
                    "crossed_lane_markings": lane_invasion_metrics["crossed_lane_markings"],
                    "red_light_violation": red_light_metrics["red_light_violation"],
                    "active_traffic_light_id": red_light_metrics["active_traffic_light_id"],
                    "active_traffic_light_state": red_light_metrics["active_traffic_light_state"],
                    "active_stop_line_progress_m": red_light_metrics["active_stop_line_progress_m"],
                    "route_deviation": route_metrics["route_deviation"],
                    "route_progress_m": route_metrics["route_progress_m"],
                    "max_route_progress_m": route_metrics["max_route_progress_m"],
                    "route_total_length_m": route_metrics["route_total_length_m"],
                    "route_completion": route_metrics["route_completion"],
                    "lateral_offset_from_route_m": route_metrics["lateral_offset_from_route_m"],
                    "on_driving_lane": route_metrics["on_driving_lane"],
                    "current_road_id": current_road_id_after,
                    "current_lane_id": current_lane_id_after,
                    "original_road_id": original_road_id,
                    "original_lane_id": original_lane_id,
                    "detour_road_id": detour_road_id,
                    "detour_lane_id": detour_lane_id,

                    "cone_detected": cone_detected,
                    "detected_cone_count": len(detected_cones),
                    "nearest_cone_gap_ahead": nearest_gap,
                    "nearest_cone_distance": nearest_cone_distance,
                    "distance_to_cone": current_min_distance,
                    "min_distance_to_cone_so_far": (
                        min_distance_to_cone_seen
                        if min_distance_to_cone_seen < float("inf")
                        else None
                    ),

                    "detour_started": detour_started_once,
                    "detour_completed": detour_completed,
                    "return_to_lane_completed": return_to_lane_completed,

                    "lateral_offset_from_lane_center": lateral_offset,
                    "max_lateral_offset_so_far": max_lateral_offset_seen,
                    "target_lateral_offset": target_lateral_offset,

                    "no_cone_ahead_hold_time": no_cone_ahead_hold_time,
                    "detour_lane_hold_time": detour_lane_hold_time,
                    "return_hold_time": return_hold_time,
                    "in_detour_lane": in_detour_lane,
                    "in_original_lane": in_original_lane,
                    "detour_lane_center_error": detour_lane_center_error,
                    "original_lane_center_error": original_lane_center_error,
                    "detour_lane_center_error_after": detour_lane_center_error_after,
                    "original_lane_center_error_after": original_lane_center_error_after,

                    "first_detection_time": first_detection_time,
                    "detour_started_time": detour_started_time,
                    "detour_completed_time": detour_completed_time,
                    "return_to_lane_time": return_to_lane_time,

                    "asr_latency_ms": 0,
                    "parser_latency_ms": 0,
                    "model_latency_ms": 1,
                    "end_to_end_latency_ms": 1,
                }

                f.write(json.dumps(record, ensure_ascii=False) + "\n")

                if frame % 20 == 0:
                    print(
                        f"t={timestamp:.1f}s "
                        f"state={state} "
                        f"lane={current_lane_id_after} "
                        f"speed={speed_kmh_after:.1f} "
                        f"gap={fmt(nearest_gap)} "
                        f"detected={cone_detected} "
                        f"lat={lateral_offset:.2f} "
                        f"target_lat={target_lateral_offset:.1f} "
                        f"detour_hold={detour_lane_hold_time:.1f}s "
                        f"no_cone_hold={no_cone_ahead_hold_time:.1f}s "
                        f"return_hold={return_hold_time:.1f}s "
                        f"collision={collision_info['value']} "
                        f"other={collision_info['other_actor']}"
                    )

                if success_time is not None and timestamp >= success_time + post_success_hold_seconds:
                    break

                if collision_info["value"] and timestamp > 2.0:
                    print(f"[WARN] collision with {collision_info['other_actor']}, stopping.")
                    break

                if timestamp > 2.0 and (
                    not route_metrics["on_driving_lane"]
                    or excessive_lateral_departure_after
                ):
                    print(
                        "[WARN] ego left allowed driving corridor, stopping this run. "
                        f"road/lane={current_road_id_after}/{current_lane_id_after}, "
                        f"route_deviation={route_metrics['route_deviation']}, "
                        f"on_driving_lane={route_metrics['on_driving_lane']}, "
                        f"detour_err={detour_lane_center_error_after}, "
                        f"origin_err={original_lane_center_error_after}"
                    )
                    break

        print(f"[DONE] frames saved to {log_path}")

    finally:
        if lane_invasion_tracker is not None:
            lane_invasion_tracker.destroy()
        if collision_sensor is not None:
            collision_sensor.destroy()

        for cone in cones:
            cone.destroy()

        if ego is not None:
            ego.destroy()

        world.apply_settings(original_settings)
        print("[CLEANUP] actors destroyed and world settings restored")


if __name__ == "__main__":
    main()
