import argparse
import json
import math
from pathlib import Path

import carla
import yaml


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


def get_next_location(wp, lookahead):
    next_wps = wp.next(lookahead)
    if not next_wps:
        return wp.transform.location
    return next_wps[0].transform.location


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
    steer = clip(1.30 * angle, -0.45, 0.45)
    return steer


def compute_control(speed_kmh, target_speed_kmh, distance_to_pedestrian, slowdown_distance):
    """
    靠近行人时减速。
    """
    if distance_to_pedestrian <= 10.0:
        return 0.0, 0.85

    if distance_to_pedestrian <= slowdown_distance:
        if speed_kmh > 15.0:
            return 0.0, 0.60
        if speed_kmh > 8.0:
            return 0.0, 0.25
        return 0.0, 0.0

    error = target_speed_kmh - speed_kmh

    if error > 8:
        throttle = 0.55
        brake = 0.0
    elif error > 3:
        throttle = 0.35
        brake = 0.0
    elif error < -5:
        throttle = 0.0
        brake = 0.15
    else:
        throttle = 0.20
        brake = 0.0

    return throttle, brake


def make_pedestrian_transform_from_ego(carla_map, ego_spawn, longitudinal_distance_m, lateral_offset_m):
    """
    在 ego 前方 longitudinal_distance_m 附近的道路中心线上放置行人。
    lateral_offset_m 暂时保留字段，第一版默认放在车道中心附近。
    """
    initial_wp = get_waypoint(carla_map, ego_spawn.location)
    next_wps = initial_wp.next(float(longitudinal_distance_m))

    if next_wps:
        ped_wp = next_wps[0]
        loc = ped_wp.transform.location
        rot = ped_wp.transform.rotation
    else:
        forward = ego_spawn.get_forward_vector()
        right = ego_spawn.get_right_vector()
        loc = carla.Location(
            x=ego_spawn.location.x + longitudinal_distance_m * forward.x + lateral_offset_m * right.x,
            y=ego_spawn.location.y + longitudinal_distance_m * forward.y + lateral_offset_m * right.y,
            z=ego_spawn.location.z,
        )
        rot = ego_spawn.rotation

    # 行人稍微抬高，避免卡进地面
    loc.z += 0.8
    return carla.Transform(loc, carla.Rotation(pitch=0.0, yaw=rot.yaw + 90.0, roll=0.0))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--spawn-index", type=int, default=49)
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--target-speed", type=float, default=35.0)
    parser.add_argument("--lookahead", type=float, default=12.0)
    parser.add_argument("--log-id", type=str, default="S04_pedestrian_slowdown")
    parser.add_argument(
        "--config",
        type=str,
        default="configs/scenarios/complex_obstacle/S04_pedestrian_slowdown.yaml",
    )
    args = parser.parse_args()

    config_path = Path(args.config)

    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    slowdown_cfg = cfg.get("success_criteria", {}).get("pedestrian_slowdown", {})
    detection_distance = float(slowdown_cfg.get("detection_distance_m", 30.0))
    slowdown_distance = float(slowdown_cfg.get("slowdown_distance_m", 22.0))
    safe_speed_kmh = float(slowdown_cfg.get("safe_speed_kmh", 15.0))
    min_safe_distance = float(slowdown_cfg.get("min_safe_distance_m", 6.0))
    required_slowdown_seconds = float(slowdown_cfg.get("required_slowdown_seconds", 1.0))

    pedestrian_cfg = cfg.get("actors", {}).get("pedestrians", [{}])[0]
    ped_type = pedestrian_cfg.get("type", "walker.pedestrian.0001")
    rel = pedestrian_cfg.get("relative_to_ego", {})
    ped_longitudinal_distance = float(rel.get("longitudinal_distance_m", 35.0))
    ped_lateral_offset = float(rel.get("lateral_offset_m", 0.0))

    out_dir = Path("logs/complex_obstacle") / args.log_id
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "frames.jsonl"

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    carla_map = world.get_map()
    blueprint_library = world.get_blueprint_library()

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    ego = None
    pedestrian = None
    collision_sensor = None

    collision_info = {
        "value": False,
        "other_actor": None,
    }

    try:
        # 清理残留 actor
        old_actors = []
        old_actors.extend(world.get_actors().filter("vehicle.*"))
        old_actors.extend(world.get_actors().filter("walker.*"))
        old_actors.extend(world.get_actors().filter("sensor.*"))
        for actor in old_actors:
            actor.destroy()

        world.tick()

        vehicle_type = cfg.get("ego", {}).get("vehicle_type", "vehicle.tesla.model3")
        vehicle_bp = blueprint_library.find(vehicle_type)

        spawn_points = carla_map.get_spawn_points()
        spawn_index = args.spawn_index % len(spawn_points)
        spawn_point = spawn_points[spawn_index]
        spawn_point.location.z += 0.5

        print(f"[INFO] total spawn points = {len(spawn_points)}")
        print(f"[INFO] using spawn_index = {spawn_index}")
        print(
            f"[INFO] ego spawn = "
            f"({spawn_point.location.x:.2f}, {spawn_point.location.y:.2f}, {spawn_point.location.z:.2f})"
        )

        ego = world.try_spawn_actor(vehicle_bp, spawn_point)
        if ego is None:
            raise RuntimeError(f"Failed to spawn ego at index {spawn_index}")

        for _ in range(20):
            world.tick()

        # 放置行人
        ped_transform = make_pedestrian_transform_from_ego(
            carla_map,
            spawn_point,
            ped_longitudinal_distance,
            ped_lateral_offset,
        )

        try:
            ped_bp = blueprint_library.find(ped_type)
        except RuntimeError:
            ped_bp = blueprint_library.filter("walker.pedestrian.*")[0]

        if ped_bp.has_attribute("is_invincible"):
            ped_bp.set_attribute("is_invincible", "false")

        pedestrian = world.try_spawn_actor(ped_bp, ped_transform)
        if pedestrian is None:
            raise RuntimeError("Failed to spawn pedestrian actor.")

        for _ in range(10):
            world.tick()

        ped_loc = pedestrian.get_location()
        print(
            f"[INFO] pedestrian spawned at "
            f"({ped_loc.x:.2f}, {ped_loc.y:.2f}, {ped_loc.z:.2f})"
        )

        collision_bp = blueprint_library.find("sensor.other.collision")
        collision_sensor = world.spawn_actor(
            collision_bp,
            carla.Transform(),
            attach_to=ego,
        )

        def on_collision(event):
            collision_info["value"] = True
            collision_info["other_actor"] = event.other_actor.type_id

        collision_sensor.listen(on_collision)

        max_frames = int(args.duration / 0.05)

        slowdown_hold_time = 0.0
        safe_slowdown_completed = False
        safe_slowdown_completed_time = None
        first_detection_time = None
        slowdown_started_time = None

        print(f"[START] Running {args.log_id}")
        print(f"[INFO] config = {config_path}")
        print(f"[INFO] target_speed = {args.target_speed} km/h")
        print(f"[INFO] detection_distance = {detection_distance} m")
        print(f"[INFO] slowdown_distance = {slowdown_distance} m")
        print(f"[INFO] safe_speed = {safe_speed_kmh} km/h")
        print(f"[INFO] min_safe_distance = {min_safe_distance} m")
        print(f"[INFO] log_path = {log_path}")

        with log_path.open("w", encoding="utf-8") as f:
            for frame in range(max_frames):
                timestamp = frame * 0.05

                ego_loc = ego.get_location()
                ped_loc = pedestrian.get_location()

                distance_to_pedestrian = ego_loc.distance(ped_loc)

                pedestrian_detected = distance_to_pedestrian <= detection_distance
                slowdown_started = distance_to_pedestrian <= slowdown_distance

                if pedestrian_detected and first_detection_time is None:
                    first_detection_time = timestamp
                    print(f"[EVENT] pedestrian_detected at t={timestamp:.1f}s, distance={distance_to_pedestrian:.1f}m")

                if slowdown_started and slowdown_started_time is None:
                    slowdown_started_time = timestamp
                    print(f"[EVENT] slowdown_started at t={timestamp:.1f}s, distance={distance_to_pedestrian:.1f}m")

                current_wp = get_waypoint(carla_map, ego_loc)
                target_loc = get_next_location(current_wp, args.lookahead)

                speed_kmh = get_speed_kmh(ego)
                steer = compute_steer_to_location(ego, target_loc)

                throttle, brake = compute_control(
                    speed_kmh,
                    args.target_speed,
                    distance_to_pedestrian,
                    slowdown_distance,
                )

                control = carla.VehicleControl()
                control.steer = float(steer)
                control.throttle = float(throttle)
                control.brake = float(brake)
                control.hand_brake = False
                control.reverse = False

                ego.apply_control(control)
                world.tick()

                ego_loc = ego.get_location()
                ped_loc = pedestrian.get_location()
                distance_to_pedestrian = ego_loc.distance(ped_loc)
                speed_kmh = get_speed_kmh(ego)

                pedestrian_detected = distance_to_pedestrian <= detection_distance
                slowdown_started = distance_to_pedestrian <= slowdown_distance

                speed_safe = speed_kmh <= safe_speed_kmh
                distance_safe = distance_to_pedestrian >= min_safe_distance

                if slowdown_started and speed_safe and distance_safe and not collision_info["value"]:
                    slowdown_hold_time += 0.05
                else:
                    slowdown_hold_time = 0.0

                if slowdown_hold_time >= required_slowdown_seconds and not safe_slowdown_completed:
                    safe_slowdown_completed = True
                    safe_slowdown_completed_time = timestamp
                    print(
                        f"[SUCCESS] safe slowdown completed at t={timestamp:.1f}s, "
                        f"speed={speed_kmh:.1f}km/h, distance={distance_to_pedestrian:.1f}m"
                    )

                # spectator 跟随 ego
                ego_tf = ego.get_transform()
                forward = ego_tf.get_forward_vector()
                spectator = world.get_spectator()
                camera_loc = carla.Location(
                    x=ego_loc.x - 10.0 * forward.x,
                    y=ego_loc.y - 10.0 * forward.y,
                    z=ego_loc.z + 5.0,
                )
                camera_rot = carla.Rotation(
                    pitch=-20.0,
                    yaw=ego_tf.rotation.yaw,
                    roll=0.0,
                )
                spectator.set_transform(carla.Transform(camera_loc, camera_rot))

                record = {
                    "timestamp": timestamp,
                    "frame": frame,
                    "scenario_id": args.log_id,
                    "instruction_id": "cmd_001" if timestamp >= 5.0 else None,
                    "ego_x": ego_loc.x,
                    "ego_y": ego_loc.y,
                    "ego_z": ego_loc.z,
                    "ego_speed_kmh": speed_kmh,
                    "steer": control.steer,
                    "throttle": control.throttle,
                    "brake": control.brake,
                    "collision": collision_info["value"],
                    "collision_other_actor": collision_info["other_actor"],
                    "lane_invasion": False,
                    "red_light_violation": False,
                    "route_deviation": False,
                    "pedestrian_x": ped_loc.x,
                    "pedestrian_y": ped_loc.y,
                    "pedestrian_z": ped_loc.z,
                    "distance_to_pedestrian": distance_to_pedestrian,
                    "pedestrian_detected": pedestrian_detected,
                    "slowdown_started": slowdown_started,
                    "safe_slowdown_completed": safe_slowdown_completed,
                    "safe_slowdown_completed_time": safe_slowdown_completed_time,
                    "slowdown_hold_time": slowdown_hold_time,
                    "first_detection_time": first_detection_time,
                    "slowdown_started_time": slowdown_started_time,
                    "asr_latency_ms": 0,
                    "parser_latency_ms": 0,
                    "model_latency_ms": 1,
                    "end_to_end_latency_ms": 1,
                }

                f.write(json.dumps(record, ensure_ascii=False) + "\n")

                if frame % 20 == 0:
                    print(
                        f"t={timestamp:.1f}s "
                        f"speed={speed_kmh:.1f} "
                        f"dist_ped={distance_to_pedestrian:.1f} "
                        f"detected={pedestrian_detected} "
                        f"slowdown={slowdown_started} "
                        f"hold={slowdown_hold_time:.1f}s "
                        f"completed={safe_slowdown_completed} "
                        f"collision={collision_info['value']} "
                        f"other={collision_info['other_actor']}"
                    )

                if safe_slowdown_completed:
                    break

                if collision_info["value"] and timestamp > 2.0:
                    print(
                        f"[WARN] collision detected with "
                        f"{collision_info['other_actor']}, stopping this run."
                    )
                    break

        print(f"[DONE] frames saved to {log_path}")

    finally:
        if collision_sensor is not None:
            collision_sensor.destroy()
        if pedestrian is not None:
            pedestrian.destroy()
        if ego is not None:
            ego.destroy()

        world.apply_settings(original_settings)
        print("[CLEANUP] actors destroyed and world settings restored")


if __name__ == "__main__":
    main()
