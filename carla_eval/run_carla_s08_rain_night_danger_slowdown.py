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


def get_next_target_location(wp, lookahead):
    next_wps = wp.next(lookahead)
    if not next_wps:
        return wp.transform.location
    return next_wps[0].transform.location


def dot2d(vec, direction):
    return vec.x * direction.x + vec.y * direction.y


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
    return clip(1.30 * angle, -0.45, 0.45)


def compute_speed_control(speed_kmh, target_speed_kmh):
    error = target_speed_kmh - speed_kmh

    if error > 6.0:
        return 0.40, 0.0
    if error > 2.0:
        return 0.25, 0.0
    if error < -5.0:
        return 0.0, 0.35
    if error < -2.0:
        return 0.0, 0.18

    return 0.12, 0.0


def apply_custom_weather(world, weather_cfg):
    weather = carla.WeatherParameters()

    for key, value in weather_cfg.items():
        if hasattr(weather, key):
            setattr(weather, key, float(value))

    world.set_weather(weather)
    return weather


def compute_hazard_score(weather):
    precipitation = getattr(weather, "precipitation", 0.0) / 100.0
    wetness = getattr(weather, "wetness", 0.0) / 100.0
    fog_density = getattr(weather, "fog_density", 0.0) / 100.0
    sun_altitude = getattr(weather, "sun_altitude_angle", 90.0)

    night_risk = 1.0 if sun_altitude < 0.0 else 0.0

    hazard_score = (
        0.35 * precipitation
        + 0.25 * wetness
        + 0.20 * fog_density
        + 0.20 * night_risk
    )

    return {
        "precipitation": precipitation,
        "wetness": wetness,
        "fog_density": fog_density,
        "sun_altitude_angle": sun_altitude,
        "is_night": night_risk > 0.5,
        "hazard_score": hazard_score,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--spawn-index", type=int, default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--normal-speed", type=float, default=None)
    parser.add_argument("--lookahead", type=float, default=None)
    parser.add_argument("--log-id", type=str, default=None)
    parser.add_argument(
        "--config",
        type=str,
        default="configs/scenarios/emergency_response/S08_rain_night_danger_slowdown.yaml",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["__config_path__"] = str(config_path.resolve())

    scenario_id = cfg.get("scenario_id", config_path.stem)
    category = cfg.get("category", "emergency_response")
    runtime_cfg = cfg.get("runtime", {})
    eval_cfg = cfg.get("evaluation", {})
    primary_instruction_id = cfg.get("instructions", [{}])[0].get("id", "cmd_001")
    trigger_time = get_instruction_trigger_time(cfg, default=5.0)
    normal_speed = float(args.normal_speed if args.normal_speed is not None else get_controller_param(cfg, "normal_speed_kmh", 35.0))
    lookahead = float(args.lookahead if args.lookahead is not None else get_controller_param(cfg, "lookahead_m", 14.0))
    duration = float(args.duration if args.duration is not None else runtime_cfg.get("max_duration_seconds", 45.0))
    post_success_hold_seconds = float(runtime_cfg.get("post_success_hold_seconds", 5.0))
    route_corridor_half_width = float(eval_cfg.get("route_corridor_half_width_m", 4.5))
    red_light_lane_tolerance = float(eval_cfg.get("red_light_lane_tolerance_m", 5.5))

    slow_cfg = cfg.get("success_criteria", {}).get("rain_night_slowdown", {})
    hazard_threshold = float(slow_cfg.get("hazard_score_threshold", 0.60))
    safe_speed_kmh = float(slow_cfg.get("safe_speed_kmh", 18.0))
    min_operating_speed_kmh = float(slow_cfg.get("min_operating_speed_kmh", 8.0))
    required_hold = float(slow_cfg.get("required_safe_speed_hold_seconds", 2.0))
    min_travel_distance = float(slow_cfg.get("min_travel_distance_m", 20.0))

    log_scenario_id = args.log_id or scenario_id
    out_dir = Path("logs") / category / log_scenario_id
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "frames.jsonl"

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = load_world_for_config(client, cfg)
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()

    original_settings = world.get_settings()
    original_weather = world.get_weather()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = float(runtime_cfg.get("fixed_delta_seconds", 0.05))
    world.apply_settings(settings)
    dt = float(settings.fixed_delta_seconds)

    ego = None
    collision_sensor = None
    lane_invasion_tracker = None

    collision_info = {"value": False, "other_actor": None}

    try:
        old_actors = []
        old_actors.extend(world.get_actors().filter("vehicle.*"))
        old_actors.extend(world.get_actors().filter("walker.*"))
        old_actors.extend(world.get_actors().filter("sensor.*"))
        old_actors.extend(world.get_actors().filter("static.prop.trafficcone*"))
        for actor in old_actors:
            actor.destroy()

        world.tick()

        apply_weather_from_config(world, cfg)

        spawn = make_transform_from_config(cfg)
        spawn_source = "config"
        spawn_index = None

        if args.spawn_index is not None:
            spawn_points = carla_map.get_spawn_points()
            spawn_index = args.spawn_index % len(spawn_points)
            spawn = spawn_points[spawn_index]
            spawn.location.z += 0.5
            spawn_source = f"spawn_index={spawn_index}"

        ego_bp = bp_lib.find(cfg.get("ego", {}).get("vehicle_type", "vehicle.tesla.model3"))
        ego = world.try_spawn_actor(ego_bp, spawn)
        if ego is None:
            raise RuntimeError("Failed to spawn ego.")

        for _ in range(20):
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

        state = "NORMAL_DRIVE"

        danger_detected = False
        slowdown_started = False
        safe_speed_reached = False
        task_completed = False

        danger_detected_time = None
        slowdown_started_time = None
        safe_speed_reached_time = None
        success_time = None

        safe_speed_hold_time = 0.0
        max_frames = int(duration / dt)
        cumulative_travel_distance = 0.0
        prev_ego_loc = ego.get_location()

        print(f"[INFO] map = {carla_map.name}")
        print(f"[INFO] spawn source = {spawn_source}")
        print(f"[INFO] weather = {cfg.get('map', {}).get('weather')}")
        print("[INFO] S08 trigger mode = weather-risk / hazard-score based")
        print(f"[INFO] config = {config_path}")
        print(f"[INFO] trigger_time = {trigger_time} s")
        print(f"[INFO] normal_speed = {normal_speed} km/h")
        print(f"[INFO] lookahead = {lookahead} m")
        print(f"[INFO] safe_speed = {safe_speed_kmh} km/h")
        print(f"[INFO] hazard_threshold = {hazard_threshold}")
        print(f"[INFO] log_path = {log_path}")

        with log_path.open("w", encoding="utf-8") as f:
            for frame in range(max_frames):
                timestamp = frame * dt

                weather = world.get_weather()
                hazard = compute_hazard_score(weather)
                hazard_score = hazard["hazard_score"]

                ego_loc = ego.get_location()
                current_wp = get_waypoint(carla_map, ego_loc)
                route_metrics_before = route_tracker.measure(ego_loc)
                travelled_distance = cumulative_travel_distance

                if hazard_score >= hazard_threshold and not danger_detected:
                    danger_detected = True
                    danger_detected_time = timestamp
                    print(
                        f"[EVENT] danger_detected at t={timestamp:.1f}s, "
                        f"hazard_score={hazard_score:.2f}"
                    )

                if state == "NORMAL_DRIVE":
                    target_speed = normal_speed

                    if danger_detected:
                        state = "SLOWDOWN"
                        target_speed = safe_speed_kmh
                        slowdown_started = True
                        slowdown_started_time = timestamp
                        print(
                            f"[EVENT] slowdown_started at t={timestamp:.1f}s, "
                            f"target_speed={safe_speed_kmh:.1f}km/h"
                        )

                elif state == "SLOWDOWN":
                    target_speed = safe_speed_kmh

                elif state == "SAFE_LOW_SPEED":
                    target_speed = safe_speed_kmh

                else:
                    target_speed = safe_speed_kmh

                ego_speed_kmh = get_speed_kmh(ego)

                if (
                    slowdown_started
                    and min_operating_speed_kmh <= ego_speed_kmh <= safe_speed_kmh
                    and travelled_distance >= min_travel_distance
                    and not collision_info["value"]
                ):
                    safe_speed_hold_time += dt
                else:
                    safe_speed_hold_time = 0.0

                if safe_speed_hold_time >= required_hold and not task_completed:
                    state = "SAFE_LOW_SPEED"
                    safe_speed_reached = True
                    task_completed = True
                    safe_speed_reached_time = timestamp
                    success_time = timestamp
                    print(
                        f"[SUCCESS] safe low speed completed at t={timestamp:.1f}s, "
                        f"speed={ego_speed_kmh:.1f}km/h, "
                        f"travelled={travelled_distance:.1f}m, "
                        f"hold={safe_speed_hold_time:.1f}s, "
                        f"continuing for {post_success_hold_seconds:.1f}s."
                    )

                if route_metrics_before["route_completion"] >= 0.98 and current_wp is not None:
                    target_loc = get_next_target_location(current_wp, lookahead)
                else:
                    target_progress = route_metrics_before["max_route_progress_m"] + lookahead
                    target_loc = route_tracker.point_at_progress(target_progress)

                steer = compute_steer_to_location(ego, target_loc)
                throttle, brake = compute_speed_control(ego_speed_kmh, target_speed)

                control = carla.VehicleControl()
                control.steer = float(steer)
                control.throttle = float(throttle)
                control.brake = float(brake)
                control.hand_brake = False
                control.reverse = False
                ego.apply_control(control)

                world.tick()

                ego_after = ego.get_location()
                cumulative_travel_distance += ego_after.distance(prev_ego_loc)
                prev_ego_loc = ego_after
                ego_speed_after = get_speed_kmh(ego)
                lane_invasion_metrics = lane_invasion_tracker.snapshot()
                route_metrics = route_tracker.measure(ego_after)
                red_light_metrics = red_light_tracker.update(ego_after, ego_speed_after)

                spectator = world.get_spectator()
                ego_tf = ego.get_transform()
                forward = ego_tf.get_forward_vector()
                spectator.set_transform(
                    carla.Transform(
                        carla.Location(
                            x=ego_after.x - 12.0 * forward.x,
                            y=ego_after.y - 12.0 * forward.y,
                            z=ego_after.z + 6.0,
                        ),
                        carla.Rotation(pitch=-22.0, yaw=ego_tf.rotation.yaw, roll=0.0),
                    )
                )

                record = {
                    "timestamp": timestamp,
                    "frame": frame,
                    "scenario_id": log_scenario_id,
                    "state": state,
                    "instruction_id": primary_instruction_id if timestamp >= trigger_time else None,

                    "ego_x": ego_after.x,
                    "ego_y": ego_after.y,
                    "ego_z": ego_after.z,
                    "ego_speed_kmh": ego_speed_after,
                    "travelled_distance_m": cumulative_travel_distance,

                    "target_speed_kmh": target_speed,
                    "normal_speed_kmh": normal_speed,
                    "safe_speed_kmh": safe_speed_kmh,
                    "min_operating_speed_kmh": min_operating_speed_kmh,

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

                    "weather_precipitation": hazard["precipitation"],
                    "weather_wetness": hazard["wetness"],
                    "weather_fog_density": hazard["fog_density"],
                    "sun_altitude_angle": hazard["sun_altitude_angle"],
                    "is_night": hazard["is_night"],
                    "hazard_score": hazard_score,
                    "hazard_score_threshold": hazard_threshold,

                    "danger_detected": danger_detected,
                    "slowdown_started": slowdown_started,
                    "safe_speed_reached": safe_speed_reached,
                    "safe_speed_hold_time": safe_speed_hold_time,

                    "danger_detected_time": danger_detected_time,
                    "slowdown_started_time": slowdown_started_time,
                    "safe_speed_reached_time": safe_speed_reached_time,

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
                        f"speed={ego_speed_after:.1f} "
                        f"target={target_speed:.1f} "
                        f"hazard={hazard_score:.2f} "
                        f"travel={cumulative_travel_distance:.1f} "
                        f"hold={safe_speed_hold_time:.1f}s "
                        f"collision={collision_info['value']}"
                    )

                if success_time is not None and timestamp >= success_time + post_success_hold_seconds:
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

        if ego is not None:
            ego.destroy()

        world.set_weather(original_weather)
        world.apply_settings(original_settings)
        print("[CLEANUP] actors destroyed, weather/settings restored")


if __name__ == "__main__":
    main()
