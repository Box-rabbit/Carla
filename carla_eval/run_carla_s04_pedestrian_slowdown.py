import argparse
import json
import math
from pathlib import Path

import carla
import yaml

try:
    from carla_eval.lmdrive import (
        LMDriveTriggerRuntime,
        Voice2LMDriveAdapter,
        load_yaml_with_path,
        resolve_config_relative_path,
    )
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
    from lmdrive import (
        LMDriveTriggerRuntime,
        Voice2LMDriveAdapter,
        load_yaml_with_path,
        resolve_config_relative_path,
    )
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


def make_pedestrian_transform_from_route(route_tracker, longitudinal_distance_m, lateral_offset_m):
    """
    按配置 route 放置行人，避免 waypoint 分支带来的隐式位置变化。
    """
    loc = route_tracker.point_at_progress(
        longitudinal_distance_m,
        lateral_offset_m=lateral_offset_m,
    )
    next_loc = route_tracker.point_at_progress(
        longitudinal_distance_m + 1.0,
        lateral_offset_m=lateral_offset_m,
    )
    yaw = math.degrees(math.atan2(next_loc.y - loc.y, next_loc.x - loc.x))
    loc.z += 0.8
    return carla.Transform(
        loc,
        carla.Rotation(pitch=0.0, yaw=yaw + 90.0, roll=0.0),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--spawn-index", type=int, default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--target-speed", type=float, default=None)
    parser.add_argument("--lookahead", type=float, default=None)
    parser.add_argument("--log-id", type=str, default=None)
    parser.add_argument(
        "--lmdrive-config",
        type=str,
        default="configs/lmdrive/scenarios/S04_pedestrian_slowdown.yaml",
    )
    parser.add_argument(
        "--config",
        type=str,
        default="configs/scenarios/complex_obstacle/S04_pedestrian_slowdown.yaml",
    )
    args = parser.parse_args()

    config_path = Path(args.config)

    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["__config_path__"] = str(config_path.resolve())

    lmdrive_cfg = None
    trigger_cfg = None
    trigger_runtime = None
    voice_adapter = None
    lmdrive_config_path = Path(args.lmdrive_config) if args.lmdrive_config else None
    if lmdrive_config_path is not None and lmdrive_config_path.exists():
        lmdrive_cfg = load_yaml_with_path(lmdrive_config_path)
        trigger_file = resolve_config_relative_path(lmdrive_cfg, lmdrive_cfg.get("trigger_file"))
        trigger_cfg = load_yaml_with_path(trigger_file)
        trigger_runtime = LMDriveTriggerRuntime(trigger_cfg)
        voice_adapter = Voice2LMDriveAdapter(lmdrive_cfg)

    scenario_id = cfg.get("scenario_id", config_path.stem)
    category = cfg.get("category", "complex_obstacle")
    runtime_cfg = cfg.get("runtime", {})
    eval_cfg = cfg.get("evaluation", {})
    primary_instruction_id = cfg.get("instructions", [{}])[0].get("id", "cmd_001")
    trigger_time = get_instruction_trigger_time(cfg, default=5.0)
    target_speed = float(args.target_speed if args.target_speed is not None else get_controller_param(cfg, "target_speed_kmh", 35.0))
    lookahead = float(args.lookahead if args.lookahead is not None else get_controller_param(cfg, "lookahead_m", 12.0))
    duration = float(args.duration if args.duration is not None else runtime_cfg.get("max_duration_seconds", 40.0))
    post_success_hold_seconds = float(runtime_cfg.get("post_success_hold_seconds", 5.0))
    route_corridor_half_width = float(eval_cfg.get("route_corridor_half_width_m", 3.5))
    red_light_lane_tolerance = float(eval_cfg.get("red_light_lane_tolerance_m", 5.5))

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
    pedestrian = None
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
        old_actors.extend(world.get_actors().filter("walker.*"))
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

        print(f"[INFO] map = {carla_map.name}")
        print(f"[INFO] spawn source = {spawn_source}")
        print(
            f"[INFO] ego spawn = "
            f"({spawn_point.location.x:.2f}, {spawn_point.location.y:.2f}, {spawn_point.location.z:.2f})"
        )

        ego = world.try_spawn_actor(vehicle_bp, spawn_point)
        if ego is None:
            raise RuntimeError(f"Failed to spawn ego at index {spawn_index}")

        for _ in range(20):
            world.tick()

        route_tracker = RouteTracker.from_route_config(
            carla_map,
            cfg,
            corridor_half_width_m=route_corridor_half_width,
        )

        # 按 route 唯一真源放置行人，保留相对纵向距离和横向偏移。
        ped_transform = make_pedestrian_transform_from_route(
            route_tracker,
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
        lane_invasion_tracker = LaneInvasionTracker(world, blueprint_library, ego)
        red_light_tracker = RedLightViolationTracker(
            world,
            route_tracker.ref_loc,
            route_tracker.ref_forward,
            route_tracker.ref_right,
            lane_tolerance_m=red_light_lane_tolerance,
        )

        max_frames = int(duration / dt)

        slowdown_hold_time = 0.0
        safe_slowdown_completed = False
        safe_slowdown_completed_time = None
        first_detection_time = None
        slowdown_started_time = None
        success_time = None
        ped_route_progress = route_tracker.measure(ped_loc)["route_progress_m"]

        instruction_text = (
            cfg.get("instructions", [{}])[0].get("text_zh")
            or cfg.get("instructions", [{}])[0].get("text_en")
            or scenario_id
        )
        expected_cfg = (trigger_cfg or {}).get("expected", {})
        expected_intents = list(expected_cfg.get("intents", []))
        expected_target_speed_max_kmh = expected_cfg.get("target_speed_max_kmh")
        expected_no_collision = expected_cfg.get("no_collision")

        voice_trigger_time = None
        voice_output = {
            "backend": (lmdrive_cfg or {}).get("voice_backend", {}).get("name", "Voice2LMDrive"),
            "backend_mode": (lmdrive_cfg or {}).get("voice_backend", {}).get("execution_mode", "disabled"),
            "status": "not_triggered",
            "error": None,
            "recognized_text": None,
            "recognized_intents": [],
            "target_speed_max_kmh": None,
            "no_collision": None,
            "asr_latency_ms": 0.0,
            "parser_latency_ms": 0.0,
            "model_latency_ms": 0.0,
            "end_to_end_latency_ms": 0.0,
            "input_mode": (trigger_cfg or {}).get("trigger", {}).get("input_mode"),
            "audio_path": str(resolve_config_relative_path(trigger_cfg, (trigger_cfg or {}).get("trigger", {}).get("audio_path"))) if trigger_cfg else None,
        }
        trigger_payload = None

        print(f"[START] Running {log_scenario_id}")
        print(f"[INFO] config = {config_path}")
        print(f"[INFO] trigger_time = {trigger_time} s")
        print(f"[INFO] target_speed = {target_speed} km/h")
        print(f"[INFO] detection_distance = {detection_distance} m")
        print(f"[INFO] slowdown_distance = {slowdown_distance} m")
        print(f"[INFO] safe_speed = {safe_speed_kmh} km/h")
        print(f"[INFO] min_safe_distance = {min_safe_distance} m")
        print(f"[INFO] lookahead = {lookahead} m")
        print(f"[INFO] log_path = {log_path}")
        if lmdrive_cfg is not None:
            print(f"[INFO] lmdrive_config = {lmdrive_config_path}")
            print(f"[INFO] trigger_file = {resolve_config_relative_path(lmdrive_cfg, lmdrive_cfg.get('trigger_file'))}")
            print(f"[INFO] voice_backend = {voice_output['backend']} ({voice_output['backend_mode']})")

        with log_path.open("w", encoding="utf-8") as f:
            for frame in range(max_frames):
                timestamp = frame * dt

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

                route_metrics_before = route_tracker.measure(ego_loc)
                target_loc = route_tracker.point_at_progress(
                    route_metrics_before["route_progress_m"] + lookahead
                )

                if trigger_runtime is not None:
                    just_triggered, trigger_payload = trigger_runtime.evaluate(
                        timestamp=timestamp,
                        route_progress_m=route_metrics_before["route_progress_m"],
                        anchor_progress_m=ped_route_progress,
                    )
                    if just_triggered and voice_adapter is not None:
                        voice_trigger_time = timestamp
                        voice_output = voice_adapter.run(
                            scenario_id=scenario_id,
                            trigger_cfg=trigger_cfg,
                            instruction_text=instruction_text,
                        )
                        print(
                            f"[EVENT] voice_instruction_triggered at t={timestamp:.1f}s, "
                            f"route_progress={route_metrics_before['route_progress_m']:.1f}m, "
                            f"route_distance_to_anchor={trigger_payload.get('route_distance_to_anchor_m')}, "
                            f"intents={voice_output.get('recognized_intents')}"
                        )

                speed_kmh = get_speed_kmh(ego)
                steer = compute_steer_to_location(ego, target_loc)
                effective_target_speed = target_speed
                if voice_trigger_time is not None and voice_output.get("target_speed_max_kmh") is not None:
                    effective_target_speed = min(
                        effective_target_speed,
                        float(voice_output.get("target_speed_max_kmh")),
                    )

                throttle, brake = compute_control(
                    speed_kmh,
                    effective_target_speed,
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
                lane_invasion_metrics = lane_invasion_tracker.snapshot()
                route_metrics = route_tracker.measure(ego_loc)
                red_light_metrics = red_light_tracker.update(ego_loc, speed_kmh)

                pedestrian_detected = distance_to_pedestrian <= detection_distance
                slowdown_started = distance_to_pedestrian <= slowdown_distance

                speed_safe = speed_kmh <= safe_speed_kmh
                distance_safe = distance_to_pedestrian >= min_safe_distance

                if slowdown_started and speed_safe and distance_safe and not collision_info["value"]:
                    slowdown_hold_time += dt
                else:
                    slowdown_hold_time = 0.0

                if slowdown_hold_time >= required_slowdown_seconds and not safe_slowdown_completed:
                    safe_slowdown_completed = True
                    safe_slowdown_completed_time = timestamp
                    success_time = timestamp
                    print(
                        f"[SUCCESS] safe slowdown completed at t={timestamp:.1f}s, "
                        f"speed={speed_kmh:.1f}km/h, distance={distance_to_pedestrian:.1f}m, "
                        f"continuing for {post_success_hold_seconds:.1f}s."
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

                instruction_active = timestamp >= trigger_time
                if trigger_runtime is not None:
                    instruction_active = voice_trigger_time is not None

                recognized_intents = list(voice_output.get("recognized_intents", []))
                intent_match = None
                if voice_trigger_time is not None and expected_intents:
                    intent_match = set(expected_intents).issubset(set(recognized_intents))

                record = {
                    "timestamp": timestamp,
                    "frame": frame,
                    "scenario_id": log_scenario_id,
                    "instruction_id": primary_instruction_id if instruction_active else None,
                    "ego_x": ego_loc.x,
                    "ego_y": ego_loc.y,
                    "ego_z": ego_loc.z,
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
                    "voice_triggered": voice_trigger_time is not None,
                    "voice_trigger_timestamp": voice_trigger_time,
                    "voice_backend": voice_output.get("backend"),
                    "voice_backend_mode": voice_output.get("backend_mode"),
                    "voice_backend_status": voice_output.get("status"),
                    "voice_error": voice_output.get("error"),
                    "trigger_type": trigger_payload.get("trigger_type") if trigger_payload else None,
                    "trigger_distance_m": trigger_payload.get("trigger_distance_m") if trigger_payload else None,
                    "route_distance_to_trigger_anchor_m": (
                        trigger_payload.get("route_distance_to_anchor_m") if trigger_payload else None
                    ),
                    "voice_input_mode": voice_output.get("input_mode"),
                    "voice_audio_path": voice_output.get("audio_path"),
                    "recognized_text": voice_output.get("recognized_text"),
                    "recognized_intents": recognized_intents,
                    "expected_intents": expected_intents,
                    "intent_match": intent_match,
                    "voice_target_speed_max_kmh": voice_output.get("target_speed_max_kmh"),
                    "expected_target_speed_max_kmh": expected_target_speed_max_kmh,
                    "voice_no_collision": voice_output.get("no_collision"),
                    "expected_no_collision": expected_no_collision,
                    "effective_target_speed_kmh": effective_target_speed,
                    "asr_latency_ms": voice_output.get("asr_latency_ms", 0.0),
                    "parser_latency_ms": voice_output.get("parser_latency_ms", 0.0),
                    "model_latency_ms": voice_output.get("model_latency_ms", 0.0),
                    "end_to_end_latency_ms": voice_output.get("end_to_end_latency_ms", 0.0),
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

                if success_time is not None and timestamp >= success_time + post_success_hold_seconds:
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
        if pedestrian is not None:
            pedestrian.destroy()
        if ego is not None:
            ego.destroy()

        world.apply_settings(original_settings)
        print("[CLEANUP] actors destroyed and world settings restored")


if __name__ == "__main__":
    main()
