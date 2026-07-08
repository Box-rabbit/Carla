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


def get_speed_mps(vehicle):
    v = vehicle.get_velocity()
    return (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5


def get_speed_kmh(vehicle):
    return 3.6 * get_speed_mps(vehicle)


def dot2d(vec, direction):
    return vec.x * direction.x + vec.y * direction.y


def horizontal_distance(a, b):
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5


def fmt(x):
    if x is None:
        return "None"
    return f"{x:.2f}"


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
    return clip(1.35 * angle, -0.45, 0.45)


def compute_cruise_control(speed_kmh, target_speed_kmh):
    error = target_speed_kmh - speed_kmh

    if error > 8.0:
        return 0.45, 0.0
    if error > 3.0:
        return 0.30, 0.0
    if error < -5.0:
        return 0.0, 0.15

    return 0.20, 0.0


def compute_emergency_control(speed_kmh, front_gap, safe_follow_distance):
    """
    ego 的应急制动控制。
    只根据当前帧感知到的前车距离和自身速度决定，不使用 NPC 的脚本时间。
    """
    if front_gap is None:
        if speed_kmh > 5.0:
            return 0.0, 0.35
        return 0.0, 0.0

    if front_gap < safe_follow_distance:
        return 0.0, 0.90

    if speed_kmh > 18.0:
        return 0.0, 0.80

    if speed_kmh > 8.0:
        return 0.0, 0.45

    return 0.0, 0.15


def make_relative_transform(spawn, longitudinal_distance_m, lateral_offset_m):
    forward = spawn.get_forward_vector()
    right = spawn.get_right_vector()

    loc = carla.Location(
        x=spawn.location.x + longitudinal_distance_m * forward.x + lateral_offset_m * right.x,
        y=spawn.location.y + longitudinal_distance_m * forward.y + lateral_offset_m * right.y,
        z=spawn.location.z + 0.20,
    )

    return carla.Transform(
        loc,
        carla.Rotation(
            pitch=0.0,
            yaw=spawn.rotation.yaw,
            roll=0.0,
        ),
    )


def scripted_cut_in_vehicle_state(
    timestamp,
    spawn,
    initial_longitudinal,
    initial_lateral,
    target_lateral,
    cruise_speed_mps,
    cut_in_start_time,
    cut_in_duration,
    brake_start_time,
    brake_strength,
):
    """
    只控制 NPC 场景事件：
    1. 初始在 ego 左前方相邻车道；
    2. cut_in_start_time 后横向切入 ego 原车道；
    3. brake_start_time 后急刹。
    """
    forward = spawn.get_forward_vector()
    right = spawn.get_right_vector()

    decel = max(1.0, 8.0 * brake_strength)

    if timestamp <= brake_start_time:
        progress = initial_longitudinal + cruise_speed_mps * timestamp
        npc_speed_mps = cruise_speed_mps
    else:
        dt = timestamp - brake_start_time
        stop_time = cruise_speed_mps / decel
        progress_at_brake = initial_longitudinal + cruise_speed_mps * brake_start_time

        if dt <= stop_time:
            progress = progress_at_brake + cruise_speed_mps * dt - 0.5 * decel * dt * dt
            npc_speed_mps = max(0.0, cruise_speed_mps - decel * dt)
        else:
            progress = progress_at_brake + cruise_speed_mps * stop_time - 0.5 * decel * stop_time * stop_time
            npc_speed_mps = 0.0

    if timestamp < cut_in_start_time:
        lateral = initial_lateral
    elif timestamp <= cut_in_start_time + cut_in_duration:
        ratio = (timestamp - cut_in_start_time) / cut_in_duration
        ratio = clip(ratio, 0.0, 1.0)
        lateral = initial_lateral + ratio * (target_lateral - initial_lateral)
    else:
        lateral = target_lateral

    loc = carla.Location(
        x=spawn.location.x + progress * forward.x + lateral * right.x,
        y=spawn.location.y + progress * forward.y + lateral * right.y,
        z=spawn.location.z + 0.20,
    )

    tf = carla.Transform(
        loc,
        carla.Rotation(
            pitch=0.0,
            yaw=spawn.rotation.yaw,
            roll=0.0,
        ),
    )

    return tf, progress, lateral, npc_speed_mps


def observe_front_vehicle(
    ego,
    npc_loc,
    npc_speed_mps,
    detection_distance,
    front_corridor_half_width,
):
    """
    ego 感知代理：
    每帧只判断 NPC 是否位于 ego 当前前方行驶走廊内。
    不使用 NPC 的 cut-in_start_time 或 brake_start_time。
    """
    ego_tf = ego.get_transform()
    ego_loc = ego_tf.location
    ego_forward = ego_tf.get_forward_vector()
    ego_right = ego_tf.get_right_vector()

    rel = carla.Location(
        x=npc_loc.x - ego_loc.x,
        y=npc_loc.y - ego_loc.y,
        z=0.0,
    )

    gap_ahead = dot2d(rel, ego_forward)
    lateral_offset = dot2d(rel, ego_right)
    distance = horizontal_distance(ego_loc, npc_loc)

    front_detected = (
        0.0 <= gap_ahead <= detection_distance
        and abs(lateral_offset) <= front_corridor_half_width
    )

    ego_velocity = ego.get_velocity()
    ego_speed_forward = (
        ego_velocity.x * ego_forward.x
        + ego_velocity.y * ego_forward.y
        + ego_velocity.z * ego_forward.z
    )

    closing_speed = ego_speed_forward - npc_speed_mps

    if front_detected and closing_speed > 0.2:
        ttc = gap_ahead / closing_speed
    else:
        ttc = None

    return {
        "front_vehicle_detected": front_detected,
        "front_vehicle_gap": gap_ahead if front_detected else None,
        "front_vehicle_lateral_offset": lateral_offset if front_detected else None,
        "front_vehicle_distance": distance if front_detected else None,
        "relative_speed_mps": closing_speed if front_detected else None,
        "relative_speed_kmh": 3.6 * closing_speed if front_detected else None,
        "ttc_s": ttc,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--spawn-index", type=int, default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--target-speed", type=float, default=None)
    parser.add_argument("--lookahead", type=float, default=None)
    parser.add_argument("--log-id", type=str, default="S07_cut_in_brake_realistic_urgent")
    parser.add_argument(
        "--config",
        type=str,
        default="configs/scenarios/emergency_response/S07_cut_in_brake.yaml",
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
    target_speed = float(args.target_speed if args.target_speed is not None else get_controller_param(cfg, "target_speed_kmh", 30.0))
    lookahead = float(args.lookahead if args.lookahead is not None else get_controller_param(cfg, "lookahead_m", 14.0))
    duration = float(args.duration if args.duration is not None else runtime_cfg.get("max_duration_seconds", 40.0))
    post_success_hold_seconds = float(runtime_cfg.get("post_success_hold_seconds", 5.0))
    route_corridor_half_width = float(eval_cfg.get("route_corridor_half_width_m", 5.5))
    red_light_lane_tolerance = float(eval_cfg.get("red_light_lane_tolerance_m", 5.5))

    cut_cfg = cfg.get("success_criteria", {}).get("cut_in_brake", {})
    detection_distance = float(cut_cfg.get("detection_distance_m", 45.0))
    front_corridor_half_width = float(cut_cfg.get("front_corridor_half_width_m", 1.8))
    emergency_brake_distance = float(cut_cfg.get("emergency_brake_distance_m", 18.0))
    emergency_brake_ttc = float(cut_cfg.get("emergency_brake_ttc_s", 3.0))
    cut_in_attention_distance = float(
        cut_cfg.get("cut_in_attention_distance_m", max(emergency_brake_distance + 10.0, 25.0))
    )
    cut_in_attention_ttc = float(
        cut_cfg.get("cut_in_attention_ttc_s", max(emergency_brake_ttc + 2.5, 5.0))
    )
    safe_follow_distance = float(cut_cfg.get("safe_follow_distance_m", 8.0))
    safe_speed_kmh = float(cut_cfg.get("safe_speed_kmh", 8.0))
    required_safe_follow_seconds = float(cut_cfg.get("required_safe_follow_seconds", 1.0))
    max_response_time_seconds = float(cut_cfg.get("max_response_time_seconds", 3.0))

    vehicle_cfg = cfg.get("actors", {}).get("vehicles", [{}])[0]
    npc_type = vehicle_cfg.get("type", "vehicle.audi.tt")
    rel = vehicle_cfg.get("relative_to_ego", {})
    npc_initial_longitudinal = float(rel.get("longitudinal_distance_m", 32.0))
    npc_initial_lateral = float(rel.get("lateral_offset_m", -3.5))

    behavior = vehicle_cfg.get("cut_in_behavior", {})
    cut_in_start_time = float(behavior.get("cut_in_start_time_s", 3.0))
    cut_in_duration = float(behavior.get("cut_in_duration_s", 2.2))
    target_lateral = float(behavior.get("target_lateral_offset_m", 0.0))
    npc_cruise_speed_kmh = float(behavior.get("cruise_speed_kmh", 22.0))
    npc_cruise_speed_mps = npc_cruise_speed_kmh / 3.6
    brake_start_after_cut_in = float(behavior.get("brake_start_after_cut_in_s", 0.6))
    brake_strength = float(behavior.get("brake_strength", 0.75))
    npc_brake_start_time = cut_in_start_time + cut_in_duration + brake_start_after_cut_in

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
    npc = None
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

        ego_bp = bp_lib.find(cfg.get("ego", {}).get("vehicle_type", "vehicle.tesla.model3"))
        ego = world.try_spawn_actor(ego_bp, spawn)
        if ego is None:
            raise RuntimeError("Failed to spawn ego.")

        for _ in range(20):
            world.tick()

        npc_bp = bp_lib.find(npc_type)
        npc_spawn = make_relative_transform(
            spawn,
            npc_initial_longitudinal,
            npc_initial_lateral,
        )
        npc = world.try_spawn_actor(npc_bp, npc_spawn)
        if npc is None:
            raise RuntimeError("Failed to spawn cut-in vehicle.")

        # NPC 使用脚本控制场景事件，避免自动驾驶不稳定。
        npc.set_simulate_physics(False)

        for _ in range(10):
            world.tick()

        collision_bp = bp_lib.find("sensor.other.collision")
        collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=ego)

        def on_collision(event):
            collision_info["value"] = True
            collision_info["other_actor"] = event.other_actor.type_id

        collision_sensor.listen(on_collision)
        lane_invasion_tracker = LaneInvasionTracker(world, bp_lib, ego)

        ref_loc = carla.Location(x=spawn.location.x, y=spawn.location.y, z=spawn.location.z)
        ref_forward = spawn.get_forward_vector()
        ref_right = spawn.get_right_vector()
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

        state = "CRUISE"

        cut_in_detected_once = False
        emergency_brake_started = False
        safe_brake_completed = False

        first_detection_time = None
        emergency_brake_started_time = None
        safe_brake_completed_time = None
        success_time = None

        safe_follow_hold_time = 0.0

        min_front_vehicle_distance = float("inf")
        min_ttc = float("inf")
        max_brake = 0.0

        print(f"[INFO] map = {carla_map.name}")
        print(f"[INFO] spawn source = {spawn_source}")
        print(f"[INFO] ego spawn = ({spawn.location.x:.2f}, {spawn.location.y:.2f}, {spawn.location.z:.2f})")
        print("[INFO] S07 ego trigger mode = detection/TTC based, no oracle cut-in time")
        print(f"[INFO] config = {config_path}")
        print(f"[INFO] trigger_time = {trigger_time} s")
        print(f"[INFO] target_speed = {target_speed} km/h")
        print(f"[INFO] lookahead = {lookahead} m")
        print(f"[INFO] npc initial longitudinal = {npc_initial_longitudinal} m")
        print(f"[INFO] npc initial lateral = {npc_initial_lateral} m")
        print(f"[INFO] npc cut-in start = {cut_in_start_time} s")
        print(f"[INFO] npc brake start = {npc_brake_start_time} s")
        print(f"[INFO] cut-in attention distance = {cut_in_attention_distance} m")
        print(f"[INFO] cut-in attention TTC = {cut_in_attention_ttc} s")
        print(f"[INFO] log_path = {log_path}")

        with log_path.open("w", encoding="utf-8") as f:
            for frame in range(max_frames):
                timestamp = frame * dt

                npc_tf, npc_progress, npc_lateral, npc_speed_mps = scripted_cut_in_vehicle_state(
                    timestamp=timestamp,
                    spawn=spawn,
                    initial_longitudinal=npc_initial_longitudinal,
                    initial_lateral=npc_initial_lateral,
                    target_lateral=target_lateral,
                    cruise_speed_mps=npc_cruise_speed_mps,
                    cut_in_start_time=cut_in_start_time,
                    cut_in_duration=cut_in_duration,
                    brake_start_time=npc_brake_start_time,
                    brake_strength=brake_strength,
                )
                npc.set_transform(npc_tf)

                ego_loc = ego.get_location()
                npc_loc = npc_tf.location

                obs = observe_front_vehicle(
                    ego=ego,
                    npc_loc=npc_loc,
                    npc_speed_mps=npc_speed_mps,
                    detection_distance=detection_distance,
                    front_corridor_half_width=front_corridor_half_width,
                )

                front_detected = obs["front_vehicle_detected"]
                front_gap = obs["front_vehicle_gap"]
                front_distance = obs["front_vehicle_distance"]
                ttc = obs["ttc_s"]

                if front_distance is not None:
                    min_front_vehicle_distance = min(min_front_vehicle_distance, front_distance)

                if ttc is not None:
                    min_ttc = min(min_ttc, ttc)

                actionable_cut_in_detected = (
                    front_detected
                    and (
                        (front_gap is not None and front_gap <= cut_in_attention_distance)
                        or (ttc is not None and ttc <= cut_in_attention_ttc)
                    )
                )

                if actionable_cut_in_detected and not cut_in_detected_once:
                    cut_in_detected_once = True
                    first_detection_time = timestamp
                    print(
                        f"[EVENT] cut_in_detected at t={timestamp:.1f}s, "
                        f"gap={fmt(front_gap)}m, ttc={fmt(ttc)}s"
                    )

                need_emergency_brake = False
                if front_detected:
                    if front_gap is not None and front_gap <= emergency_brake_distance:
                        need_emergency_brake = True
                    if ttc is not None and ttc <= emergency_brake_ttc:
                        need_emergency_brake = True

                if state == "CRUISE":
                    if need_emergency_brake:
                        state = "EMERGENCY_BRAKE"
                        emergency_brake_started = True
                        emergency_brake_started_time = timestamp

                        response_time = None
                        if first_detection_time is not None:
                            response_time = timestamp - first_detection_time

                        print(
                            f"[EVENT] emergency_brake_started at t={timestamp:.1f}s, "
                            f"gap={fmt(front_gap)}m, ttc={fmt(ttc)}s, "
                            f"response_time={fmt(response_time)}s"
                        )

                ego_speed_kmh = get_speed_kmh(ego)

                if state == "CRUISE":
                    throttle, brake = compute_cruise_control(
                        ego_speed_kmh,
                        target_speed,
                    )
                elif state == "EMERGENCY_BRAKE":
                    throttle, brake = compute_emergency_control(
                        ego_speed_kmh,
                        front_gap,
                        safe_follow_distance,
                    )

                    if (
                        front_detected
                        and front_gap is not None
                        and front_gap >= safe_follow_distance
                        and ego_speed_kmh <= safe_speed_kmh
                        and not collision_info["value"]
                    ):
                        safe_follow_hold_time += dt
                    else:
                        safe_follow_hold_time = 0.0

                    if safe_follow_hold_time >= required_safe_follow_seconds:
                        state = "COMPLETE"
                        safe_brake_completed = True
                        safe_brake_completed_time = timestamp
                        success_time = timestamp
                        print(
                            f"[SUCCESS] safe brake completed at t={timestamp:.1f}s, "
                            f"gap={fmt(front_gap)}m, speed={ego_speed_kmh:.1f}km/h, "
                            f"hold={safe_follow_hold_time:.1f}s, "
                            f"continuing for {post_success_hold_seconds:.1f}s."
                        )
                else:
                    throttle, brake = 0.0, (0.10 if ego_speed_kmh > 1.0 else 0.02)

                max_brake = max(max_brake, brake)

                route_metrics_before = route_tracker.measure(ego_loc)
                ego_progress = route_metrics_before["route_progress_m"]
                target_progress = ego_progress + lookahead
                target_loc = route_tracker.point_at_progress(target_progress)

                steer = compute_steer_to_location(ego, target_loc)

                control = carla.VehicleControl()
                control.steer = float(steer)
                control.throttle = float(throttle)
                control.brake = float(brake)
                control.hand_brake = False
                control.reverse = False

                ego.apply_control(control)

                response_time_s = None
                if first_detection_time is not None and emergency_brake_started_time is not None:
                    response_time_s = emergency_brake_started_time - first_detection_time

                world.tick()

                ego_loc_after = ego.get_location()
                ego_speed_kmh_after = get_speed_kmh(ego)
                npc_loc_after = npc.get_location()
                lane_invasion_metrics = lane_invasion_tracker.snapshot()
                route_metrics = route_tracker.measure(ego_loc_after)
                red_light_metrics = red_light_tracker.update(ego_loc_after, ego_speed_kmh_after)

                ego_tf = ego.get_transform()
                forward = ego_tf.get_forward_vector()
                spectator = world.get_spectator()
                spectator.set_transform(
                    carla.Transform(
                        carla.Location(
                            x=ego_loc_after.x - 12.0 * forward.x,
                            y=ego_loc_after.y - 12.0 * forward.y,
                            z=ego_loc_after.z + 6.0,
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

                    "ego_x": ego_loc_after.x,
                    "ego_y": ego_loc_after.y,
                    "ego_z": ego_loc_after.z,
                    "ego_speed_kmh": ego_speed_kmh_after,

                    "npc_x": npc_loc_after.x,
                    "npc_y": npc_loc_after.y,
                    "npc_z": npc_loc_after.z,
                    "npc_progress": npc_progress,
                    "npc_lateral": npc_lateral,
                    "npc_speed_kmh": 3.6 * npc_speed_mps,

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

                    "front_vehicle_detected": front_detected,
                    "front_vehicle_gap": front_gap,
                    "front_vehicle_distance": front_distance,
                    "front_vehicle_lateral_offset": obs["front_vehicle_lateral_offset"],
                    "relative_speed_kmh": obs["relative_speed_kmh"],
                    "ttc_s": ttc,

                    "cut_in_detected": cut_in_detected_once,
                    "emergency_brake_started": emergency_brake_started,
                    "safe_brake_completed": safe_brake_completed,

                    "first_detection_time": first_detection_time,
                    "emergency_brake_started_time": emergency_brake_started_time,
                    "safe_brake_completed_time": safe_brake_completed_time,
                    "emergency_response_time_s": response_time_s,

                    "safe_follow_hold_time": safe_follow_hold_time,
                    "min_front_vehicle_distance_so_far": (
                        min_front_vehicle_distance
                        if min_front_vehicle_distance < float("inf")
                        else None
                    ),
                    "min_ttc_so_far": min_ttc if min_ttc < float("inf") else None,
                    "max_brake_so_far": max_brake,

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
                        f"ego_v={ego_speed_kmh_after:.1f} "
                        f"npc_v={3.6 * npc_speed_mps:.1f} "
                        f"gap={fmt(front_gap)} "
                        f"ttc={fmt(ttc)} "
                        f"brake={brake:.2f} "
                        f"safe_hold={safe_follow_hold_time:.1f}s "
                        f"collision={collision_info['value']} "
                        f"other={collision_info['other_actor']}"
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

        if npc is not None:
            npc.destroy()

        if ego is not None:
            ego.destroy()

        world.apply_settings(original_settings)
        print("[CLEANUP] actors destroyed and world settings restored")


if __name__ == "__main__":
    main()
