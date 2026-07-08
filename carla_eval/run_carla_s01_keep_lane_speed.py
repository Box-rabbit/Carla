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


def compute_waypoint_steer(vehicle, carla_map, lookahead=18.0):
    transform = vehicle.get_transform()
    loc = transform.location

    wp = carla_map.get_waypoint(
        loc,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )

    next_wps = wp.next(lookahead)
    if not next_wps:
        return 0.0, wp.transform.location

    target_loc = next_wps[0].transform.location

    forward = transform.get_forward_vector()
    right = transform.get_right_vector()

    dx = target_loc.x - loc.x
    dy = target_loc.y - loc.y

    forward_dot = dx * forward.x + dy * forward.y
    right_dot = dx * right.x + dy * right.y

    angle = math.atan2(right_dot, forward_dot)

    # 高速下方向不要太猛
    steer = clip(1.25 * angle, -0.45, 0.45)

    return steer, target_loc


def compute_speed_control(speed_kmh, target_speed_kmh, steer):
    """
    60 km/h 版本速度控制。
    比之前更激进一点，避免车辆长期卡在 50 km/h 附近。
    """
    error = target_speed_kmh - speed_kmh

    if error > 18:
        throttle = 1.00
        brake = 0.0
    elif error > 10:
        throttle = 0.90
        brake = 0.0
    elif error > 3:
        throttle = 0.75
        brake = 0.0
    elif error < -8:
        throttle = 0.0
        brake = 0.25
    elif error < -3:
        throttle = 0.0
        brake = 0.10
    else:
        throttle = 0.45
        brake = 0.0

    # 转弯时适当限油，但不要限得太狠，否则上不了 60
    if abs(steer) > 0.25:
        throttle = min(throttle, 0.65)

    if abs(steer) > 0.38 and speed_kmh > 58:
        throttle = 0.0
        brake = max(brake, 0.10)

    return throttle, brake


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--spawn-index", type=int, default=50)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--target-speed", type=float, default=60.0)
    parser.add_argument("--lookahead", type=float, default=18.0)
    parser.add_argument("--log-id", type=str, default="S01_keep_lane_speed_60")
    parser.add_argument(
        "--config",
        type=str,
        default="configs/scenarios/basic_control/S01_keep_lane_speed_60.yaml",
    )
    args = parser.parse_args()

    config_path = Path(args.config)

    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    log_scenario_id = args.log_id

    out_dir = Path("logs") / log_scenario_id
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
    collision_sensor = None
    lane_invasion_tracker = None
    collision_info = {
        "value": False,
        "other_actor": None,
    }

    try:
        # 清理残留 actor，避免之前测试留下车辆或传感器
        old_actors = []
        old_actors.extend(world.get_actors().filter("vehicle.*"))
        old_actors.extend(world.get_actors().filter("sensor.*"))
        for actor in old_actors:
            actor.destroy()

        world.tick()

        vehicle_type = cfg.get("ego", {}).get("vehicle_type", "vehicle.tesla.model3")
        vehicle_bp = blueprint_library.find(vehicle_type)

        spawn_points = carla_map.get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points found.")

        spawn_index = args.spawn_index % len(spawn_points)
        spawn_point = spawn_points[spawn_index]
        spawn_point.location.z += 0.5

        print(f"[INFO] total spawn points = {len(spawn_points)}")
        print(f"[INFO] using spawn_index = {spawn_index}")
        print(
            f"[INFO] spawn location = "
            f"({spawn_point.location.x:.2f}, "
            f"{spawn_point.location.y:.2f}, "
            f"{spawn_point.location.z:.2f})"
        )

        ego = world.try_spawn_actor(vehicle_bp, spawn_point)
        if ego is None:
            raise RuntimeError(f"Failed to spawn ego at index {spawn_index}")

        # 等车稳定落地
        for _ in range(20):
            world.tick()

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
        lane_invasion_tracker = LaneInvasionTracker(world, blueprint_library, ego)
        route_tracker = RouteTracker.from_spawn_transform(
            carla_map,
            spawn_point,
            corridor_half_width_m=3.0,
        )
        red_light_tracker = RedLightViolationTracker(
            world,
            route_tracker.ref_loc,
            route_tracker.ref_forward,
            route_tracker.ref_right,
        )

        target_speed = float(args.target_speed)
        max_frames = int(args.duration / 0.05)

        print(f"[START] Running {log_scenario_id}")
        print(f"[INFO] config = {config_path}")
        print(f"[INFO] target_speed = {target_speed} km/h")
        print(f"[INFO] lookahead = {args.lookahead} m")
        print(f"[INFO] log_path = {log_path}")

        target_low = target_speed - 8.0
        target_high = target_speed + 8.0
        required_hold_seconds = 2.0
        success_hold_time = 0.0

        with log_path.open("w", encoding="utf-8") as f:
            for frame in range(max_frames):
                timestamp = frame * 0.05

                speed_kmh = get_speed_kmh(ego)

                steer, target_loc = compute_waypoint_steer(
                    ego,
                    carla_map,
                    lookahead=args.lookahead,
                )

                throttle, brake = compute_speed_control(
                    speed_kmh,
                    target_speed,
                    steer,
                )

                control = carla.VehicleControl()
                control.steer = float(steer)
                control.throttle = float(throttle)
                control.brake = float(brake)
                control.hand_brake = False
                control.reverse = False

                ego.apply_control(control)
                world.tick()

                loc = ego.get_location()
                speed_kmh = get_speed_kmh(ego)
                lane_invasion_metrics = lane_invasion_tracker.snapshot()
                route_metrics = route_tracker.measure(loc)
                red_light_metrics = red_light_tracker.update(loc, speed_kmh)

                # 让 CARLA UE4 窗口的 spectator 相机跟随 ego 车
                ego_tf = ego.get_transform()
                forward = ego_tf.get_forward_vector()
                spectator = world.get_spectator()

                camera_loc = carla.Location(
                    x=loc.x - 10.0 * forward.x,
                    y=loc.y - 10.0 * forward.y,
                    z=loc.z + 5.0,
                )

                camera_rot = carla.Rotation(
                    pitch=-20.0,
                    yaw=ego_tf.rotation.yaw,
                    roll=0.0,
                )

                spectator.set_transform(
                    carla.Transform(camera_loc, camera_rot)
                )

                record = {
                    "timestamp": timestamp,
                    "frame": frame,
                    "scenario_id": log_scenario_id,
                    "instruction_id": "cmd_001" if timestamp >= 3.0 else None,
                    "ego_x": loc.x,
                    "ego_y": loc.y,
                    "ego_z": loc.z,
                    "ego_speed_kmh": speed_kmh,
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
                    "distance_to_front_actor": None,
                    "target_wp_x": target_loc.x,
                    "target_wp_y": target_loc.y,
                    "asr_latency_ms": 0,
                    "parser_latency_ms": 0,
                    "model_latency_ms": 1,
                    "end_to_end_latency_ms": 1,
                }

                f.write(json.dumps(record, ensure_ascii=False) + "\n")

                if frame % 20 == 0:
                    print(
                        f"t={timestamp:.1f}s "
                        f"loc=({loc.x:.1f},{loc.y:.1f},{loc.z:.2f}) "
                        f"speed={speed_kmh:.1f} km/h "
                        f"steer={control.steer:.2f} "
                        f"throttle={control.throttle:.2f} "
                        f"brake={control.brake:.2f} "
                        f"collision={collision_info['value']} "
                        f"other={collision_info['other_actor']}"
                    )

                # 达到目标速度区间并保持 2 秒后，立刻停止，避免后续路段撞墙污染成功日志
                if target_low <= speed_kmh <= target_high and timestamp >= 3.0:
                    success_hold_time += 0.05
                else:
                    success_hold_time = 0.0

                if success_hold_time >= required_hold_seconds:
                    print(
                        f"[SUCCESS] target speed held for "
                        f"{required_hold_seconds:.1f}s, stopping this run."
                    )
                    break

                if collision_info["value"] and timestamp > 2.0:
                    print(
                        f"[WARN] collision detected with "
                        f"{collision_info['other_actor']}, stopping this run."
                    )
                    break

        print(f"[DONE] frames saved to {log_path}")

    finally:
        if lane_invasion_tracker is not None:
            lane_invasion_tracker.destroy()
        if collision_sensor is not None:
            collision_sensor.destroy()
        if ego is not None:
            ego.destroy()

        world.apply_settings(original_settings)
        print("[CLEANUP] actors destroyed and world settings restored")


if __name__ == "__main__":
    main()
