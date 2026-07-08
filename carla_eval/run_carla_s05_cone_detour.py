import argparse
import json
import math
from pathlib import Path

import carla
import yaml

try:
    from carla_eval.runtime_metrics import LaneInvasionTracker, RedLightViolationTracker, RouteTracker
except ModuleNotFoundError:
    from runtime_metrics import LaneInvasionTracker, RedLightViolationTracker, RouteTracker


def clip(x, low, high):
    return max(low, min(high, x))


def get_speed_kmh(vehicle):
    v = vehicle.get_velocity()
    return 3.6 * (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5


def dot2d(vec, direction):
    return vec.x * direction.x + vec.y * direction.y


def horizontal_distance(a, b):
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5


def fmt(x):
    if x is None:
        return "None"
    return f"{x:.1f}"


def compute_steer_to_location(vehicle, target_loc):
    tf = vehicle.get_transform()
    loc = tf.location

    forward = tf.get_forward_vector()
    right = tf.get_right_vector()

    dx = target_loc.x - loc.x
    dy = target_loc.y - loc.y

    forward_dot = dx * forward.x + dy * forward.y
    right_dot = dx * right.x + dy * right.y

    angle = math.atan2(right_dot, forward_dot)
    return clip(1.35 * angle, -0.55, 0.55)


def compute_control(speed_kmh, target_speed_kmh, min_distance_to_cone, min_safe_distance):
    if min_distance_to_cone is not None and min_distance_to_cone < min_safe_distance:
        return 0.0, 0.80

    error = target_speed_kmh - speed_kmh

    if error > 8.0:
        return 0.45, 0.0
    if error > 3.0:
        return 0.30, 0.0
    if error < -6.0:
        return 0.0, 0.18

    return 0.20, 0.0


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
    parser.add_argument("--spawn-index", type=int, default=49)
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--target-speed", type=float, default=30.0)
    parser.add_argument("--lookahead", type=float, default=14.0)
    parser.add_argument("--log-id", type=str, default="S05_cone_detour")
    parser.add_argument(
        "--config",
        type=str,
        default="configs/scenarios/complex_obstacle/S05_cone_detour.yaml",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    detour_cfg = cfg.get("success_criteria", {}).get("cone_detour", {})
    detection_distance = float(detour_cfg.get("detection_distance_m", 35.0))
    detour_start_distance = float(detour_cfg.get("detour_start_distance_m", 30.0))
    detour_lateral_offset = float(detour_cfg.get("detour_lateral_offset_m", 3.5))
    min_safe_distance = float(detour_cfg.get("min_safe_distance_to_cone_m", 1.5))
    return_lane_tolerance = float(detour_cfg.get("return_lane_tolerance_m", 1.2))
    required_return_hold = float(detour_cfg.get("required_return_hold_seconds", 1.0))
    no_cone_ahead_hold_required = float(detour_cfg.get("no_cone_ahead_hold_seconds", 1.0))
    corridor_half_width = float(detour_cfg.get("detection_corridor_half_width_m", 1.8))

    out_dir = Path("logs/complex_obstacle") / args.log_id
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "frames.jsonl"

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

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

        spawn_points = carla_map.get_spawn_points()
        spawn = spawn_points[args.spawn_index % len(spawn_points)]
        spawn.location.z += 0.5

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
        print(f"[INFO] spawn_index = {args.spawn_index}")
        print(f"[INFO] ego spawn = ({spawn.location.x:.2f}, {spawn.location.y:.2f}, {spawn.location.z:.2f})")
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
        route_tracker = RouteTracker(
            carla_map=carla_map,
            ref_loc=ref_loc,
            ref_forward=ref_forward,
            ref_right=ref_right,
            route_length_m=100.0,
            corridor_half_width_m=6.0,
        )
        red_light_tracker = RedLightViolationTracker(
            world,
            ref_loc,
            ref_forward,
            ref_right,
            lane_tolerance_m=6.0,
        )

        max_frames = int(args.duration / 0.05)

        state = "APPROACH"

        first_detection_time = None
        detour_started_time = None
        detour_completed_time = None
        return_to_lane_time = None

        cone_detected_once = False
        detour_started_once = False
        detour_completed = False
        return_to_lane_completed = False

        no_cone_ahead_hold_time = 0.0
        return_hold_time = 0.0
        min_distance_to_cone_seen = float("inf")
        max_lateral_offset_seen = 0.0

        print(f"[START] Running {args.log_id}")
        print(f"[INFO] detection_distance = {detection_distance} m")
        print(f"[INFO] detour_start_distance = {detour_start_distance} m")
        print(f"[INFO] detour_lateral_offset = {detour_lateral_offset} m")
        print(f"[INFO] no_cone_ahead_hold_required = {no_cone_ahead_hold_required} s")
        print(f"[INFO] log_path = {log_path}")

        with log_path.open("w", encoding="utf-8") as f:
            for frame in range(max_frames):
                timestamp = frame * 0.05

                ego_loc = ego.get_location()

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
                max_lateral_offset_seen = max(max_lateral_offset_seen, abs(lateral_offset))

                if cone_detected and not cone_detected_once:
                    cone_detected_once = True
                    first_detection_time = timestamp
                    print(
                        f"[EVENT] cone_detected at t={timestamp:.1f}s, "
                        f"gap={nearest_gap:.1f}m, dist={nearest_cone_distance:.1f}m"
                    )

                if state == "APPROACH":
                    target_lateral_offset = 0.0
                    current_target_speed = min(args.target_speed, 30.0)

                    if cone_detected and nearest_gap is not None and nearest_gap <= detour_start_distance:
                        state = "DETOUR_LEFT"
                        detour_started_once = True
                        detour_started_time = timestamp
                        print(
                            f"[EVENT] detour_started at t={timestamp:.1f}s, "
                            f"gap={nearest_gap:.1f}m, dist={nearest_cone_distance:.1f}m"
                        )

                if state == "DETOUR_LEFT":
                    target_lateral_offset = -detour_lateral_offset
                    current_target_speed = min(args.target_speed, 20.0)

                    # 返回触发：不是“超过最后一个锥桶”，而是“前方原始行驶走廊内连续一段时间没有检测到锥桶”
                    if cone_detected:
                        no_cone_ahead_hold_time = 0.0
                    else:
                        no_cone_ahead_hold_time += 0.05

                    if no_cone_ahead_hold_time >= no_cone_ahead_hold_required:
                        state = "RETURN_TO_LANE"
                        detour_completed = True
                        detour_completed_time = timestamp
                        print(
                            f"[EVENT] detour_completed at t={timestamp:.1f}s, "
                            f"reason=no_cone_ahead_for_{no_cone_ahead_hold_time:.1f}s, "
                            f"min_dist={min_distance_to_cone_seen:.1f}m, "
                            f"max_lat={max_lateral_offset_seen:.1f}m"
                        )

                elif state == "RETURN_TO_LANE":
                    target_lateral_offset = 0.0
                    current_target_speed = min(args.target_speed, 24.0)

                    if abs(lateral_offset) <= return_lane_tolerance:
                        return_hold_time += 0.05
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
                        print(
                            f"[SUCCESS] return to lane completed at t={timestamp:.1f}s, "
                            f"return_hold={return_hold_time:.1f}s, "
                            f"min_dist={min_distance_to_cone_seen:.1f}m"
                        )

                elif state == "COMPLETE":
                    target_lateral_offset = 0.0
                    current_target_speed = 0.0

                target_progress = ego_progress + args.lookahead
                target_loc = carla.Location(
                    x=ref_loc.x + target_progress * ref_forward.x + target_lateral_offset * ref_right.x,
                    y=ref_loc.y + target_progress * ref_forward.y + target_lateral_offset * ref_right.y,
                    z=ref_loc.z,
                )

                speed_kmh = get_speed_kmh(ego)
                steer = compute_steer_to_location(ego, target_loc)
                throttle, brake = compute_control(
                    speed_kmh=speed_kmh,
                    target_speed_kmh=current_target_speed,
                    min_distance_to_cone=current_min_distance,
                    min_safe_distance=min_safe_distance,
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
                    "scenario_id": args.log_id,
                    "state": state,
                    "instruction_id": "cmd_001" if timestamp >= 5.0 else None,

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
                    "return_hold_time": return_hold_time,

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
                        f"speed={speed_kmh_after:.1f} "
                        f"gap={fmt(nearest_gap)} "
                        f"detected={cone_detected} "
                        f"lat={lateral_offset:.2f} "
                        f"target_lat={target_lateral_offset:.1f} "
                        f"no_cone_hold={no_cone_ahead_hold_time:.1f}s "
                        f"return_hold={return_hold_time:.1f}s "
                        f"collision={collision_info['value']} "
                        f"other={collision_info['other_actor']}"
                    )

                if return_to_lane_completed:
                    break

                if collision_info["value"] and timestamp > 2.0:
                    print(f"[WARN] collision with {collision_info['other_actor']}, stopping.")
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
