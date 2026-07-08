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
    if adj_wp is None:
        return False

    if adj_wp.lane_type != carla.LaneType.Driving:
        return False

    # 避免变到对向车道
    if wp.lane_id * adj_wp.lane_id <= 0:
        return False

    return True


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

    steer = clip(1.35 * angle, -0.55, 0.55)
    return steer


def compute_speed_control(speed_kmh, target_speed_kmh, steer):
    error = target_speed_kmh - speed_kmh

    if error > 10:
        throttle = 0.65
        brake = 0.0
    elif error > 3:
        throttle = 0.45
        brake = 0.0
    elif error < -8:
        throttle = 0.0
        brake = 0.25
    elif error < -3:
        throttle = 0.0
        brake = 0.10
    else:
        throttle = 0.25
        brake = 0.0

    # 变道时如果方向较大，适当限制油门
    if abs(steer) > 0.35:
        throttle = min(throttle, 0.35)

    return throttle, brake


def get_next_target_location(wp, lookahead):
    next_wps = wp.next(lookahead)
    if not next_wps:
        return wp.transform.location
    return next_wps[0].transform.location


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
        default="configs/scenarios/basic_control/S02_lane_change.yaml",
    )
    args = parser.parse_args()

    config_path = Path(args.config)

    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["__config_path__"] = str(config_path.resolve())

    scenario_id = cfg.get("scenario_id", config_path.stem)
    category = cfg.get("category", "basic_control")
    runtime_cfg = cfg.get("runtime", {})
    eval_cfg = cfg.get("evaluation", {})
    primary_instruction_id = cfg.get("instructions", [{}])[0].get("id", "cmd_001")
    trigger_time = get_instruction_trigger_time(cfg, default=5.0)
    target_speed = float(args.target_speed if args.target_speed is not None else get_controller_param(cfg, "target_speed_kmh", 35.0))
    lookahead = float(args.lookahead if args.lookahead is not None else get_controller_param(cfg, "lookahead_m", 12.0))
    duration = float(args.duration if args.duration is not None else runtime_cfg.get("max_duration_seconds", 45.0))
    post_success_hold_seconds = float(runtime_cfg.get("post_success_hold_seconds", 5.0))
    route_corridor_half_width = float(eval_cfg.get("route_corridor_half_width_m", 6.0))
    red_light_lane_tolerance = float(eval_cfg.get("red_light_lane_tolerance_m", 6.0))
    lane_cfg = cfg.get("success_criteria", {}).get("lane_change", {})
    required_hold_seconds = float(lane_cfg.get("required_hold_seconds", 2.0))
    max_completion_time = float(lane_cfg.get("max_completion_time_seconds", 20.0))

    log_scenario_id = args.log_id or scenario_id
    out_dir = Path("logs") / category / log_scenario_id
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "frames.jsonl"

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = load_world_for_config(client, cfg)
    apply_weather_from_config(world, cfg)
    carla_map = world.get_map()
    blueprint_library = world.get_blueprint_library()

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = float(runtime_cfg.get("fixed_delta_seconds", 0.05))
    world.apply_settings(settings)
    dt = float(settings.fixed_delta_seconds)

    ego = None
    collision_sensor = None
    lane_invasion_tracker = None
    collision_info = {
        "value": False,
        "other_actor": None,
    }

    try:
        # 清理残留 actor
        old_actors = []
        old_actors.extend(world.get_actors().filter("vehicle.*"))
        old_actors.extend(world.get_actors().filter("sensor.*"))
        for actor in old_actors:
            actor.destroy()

        world.tick()

        vehicle_type = cfg.get("ego", {}).get("vehicle_type", "vehicle.tesla.model3")
        vehicle_bp = blueprint_library.find(vehicle_type)

        spawn_point = make_transform_from_config(cfg)
        spawn_source = "config"
        spawn_index = None

        if args.spawn_index is not None:
            spawn_points = carla_map.get_spawn_points()
            spawn_index = args.spawn_index % len(spawn_points)
            spawn_point = spawn_points[spawn_index]
            spawn_point.location.z += 0.5
            spawn_source = f"spawn_index={spawn_index}"

        initial_wp = get_waypoint(carla_map, spawn_point.location)
        target_wp = initial_wp.get_left_lane()

        if not is_valid_adjacent_lane(initial_wp, target_wp):
            raise RuntimeError(
                f"spawn-index {spawn_index} does not have a valid left driving lane. "
                f"initial lane={initial_wp.lane_id}, target={None if target_wp is None else target_wp.lane_id}"
            )

        initial_lane_id = initial_wp.lane_id
        initial_road_id = initial_wp.road_id
        target_lane_id = target_wp.lane_id
        target_road_id = target_wp.road_id

        print(f"[INFO] map = {carla_map.name}")
        print(f"[INFO] spawn source = {spawn_source}")
        print(
            f"[INFO] initial road/lane = {initial_road_id}/{initial_lane_id}, "
            f"target road/lane = {target_road_id}/{target_lane_id}"
        )
        print(
            f"[INFO] spawn location = "
            f"({spawn_point.location.x:.2f}, "
            f"{spawn_point.location.y:.2f}, "
            f"{spawn_point.location.z:.2f})"
        )

        ego = world.try_spawn_actor(vehicle_bp, spawn_point)
        if ego is None:
            raise RuntimeError(f"Failed to spawn ego at index {spawn_index}")

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
        target_lane_hold_time = 0.0
        lane_change_completed = False
        lane_change_completed_time = None
        success_time = None

        print(f"[START] Running {log_scenario_id}")
        print(f"[INFO] config = {config_path}")
        print(f"[INFO] trigger_time = {trigger_time} s")
        print(f"[INFO] target_speed = {target_speed} km/h")
        print(f"[INFO] lookahead = {lookahead} m")
        print(f"[INFO] log_path = {log_path}")

        with log_path.open("w", encoding="utf-8") as f:
            for frame in range(max_frames):
                timestamp = frame * dt
                loc = ego.get_location()
                current_wp = get_waypoint(carla_map, loc)
                route_metrics_before = route_tracker.measure(loc)
                route_target_loc = route_tracker.point_at_progress(
                    route_metrics_before["max_route_progress_m"] + lookahead
                )

                current_lane_id = current_wp.lane_id
                current_road_id = current_wp.road_id

                lane_change_started = timestamp >= trigger_time
                in_target_lane_before = current_lane_id == target_lane_id

                if not lane_change_started:
                    # 触发前：沿当前车道行驶
                    target_loc = get_next_target_location(current_wp, lookahead)
                else:
                    # 触发后：
                    # 1. 用配置 route 完成进入目标车道，避免路口 waypoint 分支歧义；
                    # 2. 一旦已经进入目标车道且 route 已走到尾部，切回目标车道正常 lane keeping，
                    #    避免继续追逐 route 末端固定点而产生回头/转圈。
                    if in_target_lane_before and route_metrics_before["route_completion"] >= 0.98:
                        target_loc = get_next_target_location(current_wp, lookahead)
                    else:
                        target_loc = route_target_loc

                speed_kmh = get_speed_kmh(ego)
                steer = compute_steer_to_location(ego, target_loc)
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
                lane_invasion_metrics = lane_invasion_tracker.snapshot()
                route_metrics = route_tracker.measure(loc)
                current_wp = get_waypoint(carla_map, loc)
                current_lane_id = current_wp.lane_id
                current_road_id = current_wp.road_id
                speed_kmh = get_speed_kmh(ego)
                red_light_metrics = red_light_tracker.update(loc, speed_kmh)

                # 到目标车道中心线的距离
                target_lane_center = current_wp.transform.location
                if current_lane_id != target_lane_id:
                    maybe_left = current_wp.get_left_lane()
                    if maybe_left is not None:
                        target_lane_center = maybe_left.transform.location

                distance_to_target_lane_center = loc.distance(target_lane_center)

                in_target_lane = current_lane_id == target_lane_id

                if lane_change_started and in_target_lane and not collision_info["value"]:
                    target_lane_hold_time += dt
                else:
                    target_lane_hold_time = 0.0

                if (
                    not lane_change_completed
                    and target_lane_hold_time >= required_hold_seconds
                ):
                    lane_change_completed = True
                    lane_change_completed_time = timestamp
                    success_time = timestamp
                    print(
                        f"[SUCCESS] lane change completed and held for "
                        f"{required_hold_seconds:.1f}s at t={timestamp:.1f}s, "
                        f"continuing for {post_success_hold_seconds:.1f}s."
                    )

                # spectator 跟随
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
                spectator.set_transform(carla.Transform(camera_loc, camera_rot))

                record = {
                    "timestamp": timestamp,
                    "frame": frame,
                    "scenario_id": log_scenario_id,
                    "instruction_id": primary_instruction_id if timestamp >= trigger_time else None,
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
                    "current_road_id": current_road_id,
                    "current_lane_id": current_lane_id,
                    "initial_road_id": initial_road_id,
                    "initial_lane_id": initial_lane_id,
                    "target_road_id": target_road_id,
                    "target_lane_id": target_lane_id,
                    "lane_change_started": lane_change_started,
                    "lane_change_completed": lane_change_completed,
                    "lane_change_completed_time": lane_change_completed_time,
                    "target_lane_hold_time": target_lane_hold_time,
                    "distance_to_target_lane_center": distance_to_target_lane_center,
                    "asr_latency_ms": 0,
                    "parser_latency_ms": 0,
                    "model_latency_ms": 1,
                    "end_to_end_latency_ms": 1,
                }

                f.write(json.dumps(record, ensure_ascii=False) + "\n")

                if frame % 20 == 0:
                    print(
                        f"t={timestamp:.1f}s "
                        f"lane={current_lane_id} target={target_lane_id} "
                        f"speed={speed_kmh:.1f} "
                        f"steer={control.steer:.2f} "
                        f"hold={target_lane_hold_time:.1f}s "
                        f"completed={lane_change_completed} "
                        f"collision={collision_info['value']} "
                        f"other={collision_info['other_actor']}"
                    )

                if success_time is not None and timestamp >= success_time + post_success_hold_seconds:
                    break

                if collision_info["value"] and timestamp > 2.0:
                    print(
                        f"[WARN] collision detected with "
                        f"{collision_info['other_actor']}, stopping this run."
                    )
                    break

                if (
                    lane_change_started
                    and timestamp - trigger_time > max_completion_time
                    and not lane_change_completed
                ):
                    print("[WARN] lane change timeout, stopping this run.")
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
