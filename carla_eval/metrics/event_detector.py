import argparse
import json
from pathlib import Path

import yaml


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["__config_path__"] = str(Path(path).resolve())
    return cfg


def load_frames(path):
    frames = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                frames.append(json.loads(line))
    return frames


def get_frame_dt(cfg, frames, default=0.05):
    runtime_dt = cfg.get("runtime", {}).get("fixed_delta_seconds")
    if runtime_dt is not None:
        try:
            runtime_dt = float(runtime_dt)
            if runtime_dt > 0.0:
                return runtime_dt
        except (TypeError, ValueError):
            pass

    last_t = None
    for record in frames:
        timestamp = record.get("timestamp")
        if timestamp is None:
            continue
        timestamp = float(timestamp)
        if last_t is not None and timestamp > last_t:
            return timestamp - last_t
        last_t = timestamp

    return default


def failure_enabled(cfg, key, default=True):
    failure_cfg = cfg.get("failure_criteria", {})
    if key not in failure_cfg:
        return default
    return bool(failure_cfg.get(key))


def first_true_frame(frames, key):
    for record in frames:
        if record.get(key):
            return record
    return None


def has_collision(frames):
    return any(record.get("collision") for record in frames)


def detect_instruction_trigger(frames):
    record = first_true_frame(frames, "voice_triggered")
    if record is None:
        record = next((item for item in frames if item.get("instruction_id") is not None), None)

    if record is None:
        return []

    return [{
        "event": "instruction_triggered",
        "timestamp": record.get("voice_trigger_timestamp", record.get("timestamp", 0.0)),
        "instruction_id": record.get("instruction_id"),
        "trigger_type": record.get("trigger_type"),
        "trigger_distance_m": record.get("trigger_distance_m"),
        "route_distance_to_trigger_anchor_m": record.get("route_distance_to_trigger_anchor_m"),
        "voice_backend": record.get("voice_backend"),
        "voice_backend_mode": record.get("voice_backend_mode"),
        "voice_backend_status": record.get("voice_backend_status"),
        "voice_error": record.get("voice_error"),
        "voice_input_mode": record.get("voice_input_mode"),
        "voice_audio_path": record.get("voice_audio_path"),
        "recognized_text": record.get("recognized_text"),
        "recognized_intents": record.get("recognized_intents"),
        "expected_intents": record.get("expected_intents"),
        "intent_match": record.get("intent_match"),
        "voice_target_speed_max_kmh": record.get("voice_target_speed_max_kmh"),
        "expected_target_speed_max_kmh": record.get("expected_target_speed_max_kmh"),
        "voice_no_collision": record.get("voice_no_collision"),
        "expected_no_collision": record.get("expected_no_collision"),
    }]


def detect_collision(frames):
    events = []
    record = first_true_frame(frames, "collision")
    if record is not None:
        events.append({
            "event": "collision",
            "timestamp": record.get("timestamp", 0.0),
            "other_actor": record.get("collision_other_actor"),
        })
    return events


def detect_speed_target(frames, cfg):
    events = []

    speed_cfg = cfg.get("success_criteria", {}).get("target_speed", {})
    if not speed_cfg.get("enabled", False):
        return events

    target = float(speed_cfg.get("value_kmh", 60.0))
    tol = float(speed_cfg.get("tolerance_kmh", 8.0))
    required_hold = float(speed_cfg.get("required_hold_seconds", 2.0))

    low = target - tol
    high = target + tol

    hold_time = 0.0
    last_t = None
    default_dt = get_frame_dt(cfg, frames)

    for record in frames:
        timestamp = float(record.get("timestamp", 0.0))
        speed = float(record.get("ego_speed_kmh", 0.0))

        dt = default_dt if last_t is None else max(0.0, timestamp - last_t)
        last_t = timestamp

        if low <= speed <= high:
            hold_time += dt
        else:
            hold_time = 0.0

        if hold_time >= required_hold:
            events.append({
                "event": "speed_target_reached",
                "timestamp": timestamp,
                "target_speed_kmh": target,
                "tolerance_kmh": tol,
                "required_hold_seconds": required_hold,
                "actual_speed_kmh": speed,
            })

            if not has_collision(frames):
                events.append({
                    "event": "task_success",
                    "timestamp": timestamp,
                    "success": True,
                })

            return events

    return events


def detect_lane_change(frames, cfg):
    events = []

    lane_cfg = cfg.get("success_criteria", {}).get("lane_change", {})
    if not lane_cfg.get("enabled", False):
        return events

    required_hold = float(lane_cfg.get("required_hold_seconds", 2.0))
    max_completion_time = float(lane_cfg.get("max_completion_time_seconds", 20.0))

    started_frame = first_true_frame(frames, "lane_change_started")
    completed_frame = first_true_frame(frames, "lane_change_completed")

    if started_frame is not None:
        events.append({
            "event": "lane_change_started",
            "timestamp": started_frame.get("timestamp", 0.0),
            "initial_lane_id": started_frame.get("initial_lane_id"),
            "target_lane_id": started_frame.get("target_lane_id"),
            "direction": lane_cfg.get("direction", "left"),
        })

    if completed_frame is not None:
        start_t = float(started_frame.get("timestamp", 0.0)) if started_frame else 0.0
        end_t = float(completed_frame.get("timestamp", 0.0))

        events.append({
            "event": "lane_change_completed",
            "timestamp": end_t,
            "initial_lane_id": completed_frame.get("initial_lane_id"),
            "target_lane_id": completed_frame.get("target_lane_id"),
            "current_lane_id": completed_frame.get("current_lane_id"),
            "lane_change_duration_s": max(0.0, end_t - start_t),
            "target_lane_hold_s": completed_frame.get("target_lane_hold_time", required_hold),
            "required_hold_seconds": required_hold,
        })

        if not has_collision(frames):
            events.append({
                "event": "task_success",
                "timestamp": end_t,
                "success": True,
            })

        return events

    if started_frame is not None:
        last_t = float(frames[-1].get("timestamp", 0.0)) if frames else 0.0
        start_t = float(started_frame.get("timestamp", 0.0))
        if last_t - start_t >= max_completion_time:
            events.append({
                "event": "lane_change_timeout",
                "timestamp": last_t,
                "success": False,
            })

    return events


def detect_pedestrian_slowdown(frames, cfg):
    events = []

    ped_cfg = cfg.get("success_criteria", {}).get("pedestrian_slowdown", {})
    if not ped_cfg.get("enabled", False):
        return events

    detection_distance = float(ped_cfg.get("detection_distance_m", 30.0))
    slowdown_distance = float(ped_cfg.get("slowdown_distance_m", 22.0))
    safe_speed = float(ped_cfg.get("safe_speed_kmh", 15.0))
    min_safe_distance = float(ped_cfg.get("min_safe_distance_m", 6.0))
    required_hold = float(ped_cfg.get("required_slowdown_seconds", 1.0))
    max_response_time = float(ped_cfg.get("max_response_time_seconds", 5.0))

    detected_frame = first_true_frame(frames, "pedestrian_detected")
    slowdown_frame = first_true_frame(frames, "slowdown_started")
    completed_frame = first_true_frame(frames, "safe_slowdown_completed")

    if detected_frame is not None:
        events.append({
            "event": "pedestrian_detected",
            "timestamp": detected_frame.get("timestamp", 0.0),
            "distance_to_pedestrian": detected_frame.get("distance_to_pedestrian"),
            "detection_distance_m": detection_distance,
        })

    if slowdown_frame is not None:
        response_time = None
        if detected_frame is not None:
            response_time = (
                float(slowdown_frame.get("timestamp", 0.0))
                - float(detected_frame.get("timestamp", 0.0))
            )

        events.append({
            "event": "slowdown_started",
            "timestamp": slowdown_frame.get("timestamp", 0.0),
            "distance_to_pedestrian": slowdown_frame.get("distance_to_pedestrian"),
            "slowdown_distance_m": slowdown_distance,
            "response_time_s": response_time,
        })

    if completed_frame is not None:
        speeds = [float(record.get("ego_speed_kmh", 0.0)) for record in frames]
        distances = [
            float(record.get("distance_to_pedestrian"))
            for record in frames
            if record.get("distance_to_pedestrian") is not None
        ]

        events.append({
            "event": "safe_slowdown_completed",
            "timestamp": completed_frame.get("timestamp", 0.0),
            "ego_speed_kmh": completed_frame.get("ego_speed_kmh"),
            "distance_to_pedestrian": completed_frame.get("distance_to_pedestrian"),
            "safe_speed_kmh": safe_speed,
            "min_safe_distance_m": min_safe_distance,
            "slowdown_hold_time": completed_frame.get("slowdown_hold_time", required_hold),
            "required_slowdown_seconds": required_hold,
            "min_distance_to_pedestrian": min(distances) if distances else None,
            "max_speed_before_slowdown": max(speeds) if speeds else None,
            "min_speed": min(speeds) if speeds else None,
        })

        response_ok = True
        if slowdown_frame is not None and detected_frame is not None:
            response_ok = (
                float(slowdown_frame.get("timestamp", 0.0))
                - float(detected_frame.get("timestamp", 0.0))
            ) <= max_response_time

        if not has_collision(frames) and response_ok:
            events.append({
                "event": "task_success",
                "timestamp": completed_frame.get("timestamp", 0.0),
                "success": True,
            })

    return events


def detect_cone_detour(frames, cfg):
    events = []

    cone_cfg = cfg.get("success_criteria", {}).get("cone_detour", {})
    if not cone_cfg.get("enabled", False):
        return events

    detection_distance = float(cone_cfg.get("detection_distance_m", 35.0))
    detour_start_distance = float(cone_cfg.get("detour_start_distance_m", 30.0))
    min_safe_distance = float(cone_cfg.get("min_safe_distance_to_cone_m", 1.5))
    return_lane_tolerance = float(cone_cfg.get("return_lane_tolerance_m", 1.2))
    required_return_hold = float(cone_cfg.get("required_return_hold_seconds", 1.0))
    no_cone_ahead_hold_required = float(cone_cfg.get("no_cone_ahead_hold_seconds", 1.0))

    detected_frame = first_true_frame(frames, "cone_detected")
    detour_started_frame = first_true_frame(frames, "detour_started")
    detour_completed_frame = first_true_frame(frames, "detour_completed")
    return_frame = first_true_frame(frames, "return_to_lane_completed")

    if detected_frame is not None:
        events.append({
            "event": "cone_detected",
            "timestamp": detected_frame.get("timestamp", 0.0),
            "nearest_cone_gap_ahead": detected_frame.get("nearest_cone_gap_ahead"),
            "nearest_cone_distance": detected_frame.get("nearest_cone_distance"),
            "detection_distance_m": detection_distance,
        })

    if detour_started_frame is not None:
        response_time = None
        if detected_frame is not None:
            response_time = (
                float(detour_started_frame.get("timestamp", 0.0))
                - float(detected_frame.get("timestamp", 0.0))
            )

        events.append({
            "event": "detour_started",
            "timestamp": detour_started_frame.get("timestamp", 0.0),
            "nearest_cone_gap_ahead": detour_started_frame.get("nearest_cone_gap_ahead"),
            "nearest_cone_distance": detour_started_frame.get("nearest_cone_distance"),
            "detour_start_distance_m": detour_start_distance,
            "response_time_s": response_time,
        })

    if detour_completed_frame is not None:
        events.append({
            "event": "detour_completed",
            "timestamp": detour_completed_frame.get("timestamp", 0.0),
            "reason": "no_cone_ahead",
            "no_cone_ahead_hold_time": detour_completed_frame.get("no_cone_ahead_hold_time"),
            "required_no_cone_ahead_hold_seconds": no_cone_ahead_hold_required,
            "min_distance_to_cone": detour_completed_frame.get("min_distance_to_cone_so_far"),
            "max_lateral_offset_m": detour_completed_frame.get("max_lateral_offset_so_far"),
        })

    if return_frame is not None:
        events.append({
            "event": "return_to_lane_completed",
            "timestamp": return_frame.get("timestamp", 0.0),
            "return_hold_time": return_frame.get("return_hold_time"),
            "required_return_hold_seconds": required_return_hold,
            "lateral_offset_from_lane_center": return_frame.get("lateral_offset_from_lane_center"),
            "return_lane_tolerance_m": return_lane_tolerance,
            "min_distance_to_cone": return_frame.get("min_distance_to_cone_so_far"),
            "min_safe_distance_to_cone_m": min_safe_distance,
            "max_lateral_offset_m": return_frame.get("max_lateral_offset_so_far"),
        })

        if not has_collision(frames):
            events.append({
                "event": "task_success",
                "timestamp": return_frame.get("timestamp", 0.0),
                "success": True,
            })

    return events


def detect_cut_in_brake(frames, cfg):
    events = []

    cut_cfg = cfg.get("success_criteria", {}).get("cut_in_brake", {})
    if not cut_cfg.get("enabled", False):
        return events

    emergency_distance = float(cut_cfg.get("emergency_brake_distance_m", 18.0))
    emergency_ttc = float(cut_cfg.get("emergency_brake_ttc_s", 3.0))
    safe_follow_distance = float(cut_cfg.get("safe_follow_distance_m", 8.0))
    safe_speed = float(cut_cfg.get("safe_speed_kmh", 8.0))
    required_hold = float(cut_cfg.get("required_safe_follow_seconds", 1.0))
    max_response_time = float(cut_cfg.get("max_response_time_seconds", 3.0))

    detected_frame = first_true_frame(frames, "cut_in_detected")
    brake_frame = first_true_frame(frames, "emergency_brake_started")
    completed_frame = first_true_frame(frames, "safe_brake_completed")

    distances = [
        float(record.get("front_vehicle_distance"))
        for record in frames
        if record.get("front_vehicle_distance") is not None
    ]
    gaps = [
        float(record.get("front_vehicle_gap"))
        for record in frames
        if record.get("front_vehicle_gap") is not None
    ]
    ttcs = [
        float(record.get("ttc_s"))
        for record in frames
        if record.get("ttc_s") is not None
    ]
    brakes = [float(record.get("brake", 0.0)) for record in frames]

    min_distance = min(distances) if distances else None
    min_gap = min(gaps) if gaps else None
    min_ttc = min(ttcs) if ttcs else None
    max_brake = max(brakes) if brakes else None

    if detected_frame is not None:
        events.append({
            "event": "cut_in_detected",
            "timestamp": detected_frame.get("timestamp", 0.0),
            "front_vehicle_gap": detected_frame.get("front_vehicle_gap"),
            "front_vehicle_distance": detected_frame.get("front_vehicle_distance"),
            "front_vehicle_lateral_offset": detected_frame.get("front_vehicle_lateral_offset"),
            "ttc_s": detected_frame.get("ttc_s"),
        })

    if brake_frame is not None:
        response_time = brake_frame.get("emergency_response_time_s")
        if response_time is None and detected_frame is not None:
            response_time = (
                float(brake_frame.get("timestamp", 0.0))
                - float(detected_frame.get("timestamp", 0.0))
            )

        events.append({
            "event": "emergency_brake_started",
            "timestamp": brake_frame.get("timestamp", 0.0),
            "front_vehicle_gap": brake_frame.get("front_vehicle_gap"),
            "front_vehicle_distance": brake_frame.get("front_vehicle_distance"),
            "ttc_s": brake_frame.get("ttc_s"),
            "emergency_brake_distance_m": emergency_distance,
            "emergency_brake_ttc_s": emergency_ttc,
            "response_time_s": response_time,
            "max_response_time_seconds": max_response_time,
            "brake": brake_frame.get("brake"),
        })

    if completed_frame is not None:
        events.append({
            "event": "safe_brake_completed",
            "timestamp": completed_frame.get("timestamp", 0.0),
            "front_vehicle_gap": completed_frame.get("front_vehicle_gap"),
            "front_vehicle_distance": completed_frame.get("front_vehicle_distance"),
            "ego_speed_kmh": completed_frame.get("ego_speed_kmh"),
            "safe_follow_distance_m": safe_follow_distance,
            "safe_speed_kmh": safe_speed,
            "safe_follow_hold_time": completed_frame.get("safe_follow_hold_time"),
            "required_safe_follow_seconds": required_hold,
            "min_front_vehicle_distance": min_distance,
            "min_front_gap": min_gap,
            "min_ttc": min_ttc,
            "max_brake": max_brake,
        })

        response_ok = True
        if brake_frame is not None and detected_frame is not None:
            response_ok = (
                float(brake_frame.get("timestamp", 0.0))
                - float(detected_frame.get("timestamp", 0.0))
            ) <= max_response_time

        if not has_collision(frames) and response_ok:
            events.append({
                "event": "task_success",
                "timestamp": completed_frame.get("timestamp", 0.0),
                "success": True,
            })

    return events


def detect_rain_night_slowdown(frames, cfg):
    events = []

    slow_cfg = cfg.get("success_criteria", {}).get("rain_night_slowdown", {})
    if not slow_cfg.get("enabled", False):
        return events

    hazard_threshold = float(slow_cfg.get("hazard_score_threshold", 0.60))
    safe_speed = float(slow_cfg.get("safe_speed_kmh", 18.0))
    min_operating_speed = float(slow_cfg.get("min_operating_speed_kmh", 8.0))
    required_hold = float(slow_cfg.get("required_safe_speed_hold_seconds", 2.0))
    min_travel_distance = float(slow_cfg.get("min_travel_distance_m", 20.0))
    max_response_time = float(slow_cfg.get("max_response_time_seconds", 3.0))

    danger_frame = first_true_frame(frames, "danger_detected")
    slowdown_frame = first_true_frame(frames, "slowdown_started")
    safe_frame = first_true_frame(frames, "safe_speed_reached")

    speeds = [float(record.get("ego_speed_kmh", 0.0)) for record in frames]
    hazards = [float(record.get("hazard_score", 0.0)) for record in frames]

    if danger_frame is not None:
        events.append({
            "event": "danger_detected",
            "timestamp": danger_frame.get("timestamp", 0.0),
            "hazard_score": danger_frame.get("hazard_score"),
            "hazard_score_threshold": hazard_threshold,
            "weather_precipitation": danger_frame.get("weather_precipitation"),
            "weather_wetness": danger_frame.get("weather_wetness"),
            "weather_fog_density": danger_frame.get("weather_fog_density"),
            "is_night": danger_frame.get("is_night"),
            "sun_altitude_angle": danger_frame.get("sun_altitude_angle"),
        })

    if slowdown_frame is not None:
        response_time = None
        if danger_frame is not None:
            response_time = (
                float(slowdown_frame.get("timestamp", 0.0))
                - float(danger_frame.get("timestamp", 0.0))
            )

        events.append({
            "event": "slowdown_started",
            "timestamp": slowdown_frame.get("timestamp", 0.0),
            "target_speed_kmh": slowdown_frame.get("target_speed_kmh"),
            "safe_speed_kmh": safe_speed,
            "response_time_s": response_time,
            "max_response_time_seconds": max_response_time,
        })

    if safe_frame is not None:
        events.append({
            "event": "safe_speed_reached",
            "timestamp": safe_frame.get("timestamp", 0.0),
            "ego_speed_kmh": safe_frame.get("ego_speed_kmh"),
            "safe_speed_kmh": safe_speed,
            "min_operating_speed_kmh": min_operating_speed,
            "safe_speed_hold_time": safe_frame.get("safe_speed_hold_time"),
            "required_safe_speed_hold_seconds": required_hold,
            "travelled_distance_m": safe_frame.get("travelled_distance_m"),
            "min_travel_distance_m": min_travel_distance,
            "mean_speed_kmh": sum(speeds) / len(speeds) if speeds else None,
            "max_speed_kmh": max(speeds) if speeds else None,
            "min_speed_kmh": min(speeds) if speeds else None,
            "mean_hazard_score": sum(hazards) / len(hazards) if hazards else None,
        })

        response_ok = True
        if slowdown_frame is not None and danger_frame is not None:
            response_ok = (
                float(slowdown_frame.get("timestamp", 0.0))
                - float(danger_frame.get("timestamp", 0.0))
            ) <= max_response_time

        if not has_collision(frames) and response_ok:
            events.append({
                "event": "task_success",
                "timestamp": safe_frame.get("timestamp", 0.0),
                "success": True,
            })

    return events


def detect_composite_long_route(frames, cfg):
    events = []

    long_cfg = cfg.get("success_criteria", {}).get("long_route", {})
    windows = cfg.get("action_windows", {})
    if not long_cfg.get("enabled", False) or not windows:
        return events

    accel_frame = first_true_frame(frames, "reach_target_speed_60")
    left_frame = first_true_frame(frames, "complete_left_turn")
    slow_frame = first_true_frame(frames, "reach_target_speed_30")
    right_frame = first_true_frame(frames, "complete_right_turn")

    accel_cfg = windows.get("accelerate_to_60", {})
    left_cfg = windows.get("left_turn_1", {})
    slow_cfg = windows.get("slow_to_30", {})
    right_cfg = windows.get("right_turn_1", {})

    if accel_frame is not None:
        events.append({
            "event": "speed_60_reached",
            "timestamp": accel_frame.get("timestamp", 0.0),
            "target_speed_kmh": accel_cfg.get("target_speed_kmh"),
            "actual_speed_kmh": accel_frame.get("ego_speed_kmh"),
            "progress_m": accel_frame.get("route_progress_m"),
            "hold_time_s": accel_frame.get("speed_60_hold_time"),
            "required_hold_seconds": accel_cfg.get("required_hold_seconds"),
        })

    if left_frame is not None:
        events.append({
            "event": "left_turn_completed",
            "timestamp": left_frame.get("timestamp", 0.0),
            "progress_m": left_frame.get("route_progress_m"),
            "heading_change_deg": left_frame.get("left_turn_max_delta_deg"),
            "required_heading_change_deg_min": left_cfg.get("expected_heading_change_deg_min"),
        })

    if slow_frame is not None:
        events.append({
            "event": "speed_30_reached",
            "timestamp": slow_frame.get("timestamp", 0.0),
            "target_speed_kmh": slow_cfg.get("target_speed_kmh"),
            "actual_speed_kmh": slow_frame.get("ego_speed_kmh"),
            "progress_m": slow_frame.get("route_progress_m"),
            "hold_time_s": slow_frame.get("speed_30_hold_time"),
            "required_hold_seconds": slow_cfg.get("required_hold_seconds"),
        })

    if right_frame is not None:
        events.append({
            "event": "right_turn_completed",
            "timestamp": right_frame.get("timestamp", 0.0),
            "progress_m": right_frame.get("route_progress_m"),
            "heading_change_deg": right_frame.get("right_turn_min_delta_deg"),
            "required_heading_change_deg_min": right_cfg.get("expected_heading_change_deg_min"),
        })

    final_frame = frames[-1] if frames else None
    route_completion = float(final_frame.get("route_completion", 0.0)) if final_frame else 0.0
    max_route_progress_m = float(final_frame.get("max_route_progress_m", 0.0)) if final_frame else 0.0
    min_length_m = float(long_cfg.get("min_length_m", 0.0))
    target_progress_m = float(long_cfg.get("target_progress_m", 0.0))
    route_total_length_m = float(final_frame.get("route_total_length_m", 0.0)) if final_frame else 0.0
    long_route_done = (
        final_frame is not None
        and (
            (target_progress_m > 0.0 and max_route_progress_m >= target_progress_m)
            or route_completion >= float(cfg.get("success_criteria", {}).get("route_completion_min", 0.98))
        )
        and route_total_length_m >= min_length_m
    )
    if long_route_done:
        events.append({
            "event": "long_route_completed",
            "timestamp": final_frame.get("timestamp", 0.0),
            "route_completion": route_completion,
            "max_route_progress_m": max_route_progress_m,
            "route_total_length_m": route_total_length_m,
            "required_route_length_m": min_length_m,
            "target_progress_m": target_progress_m,
        })

    if (
        accel_frame is not None
        and left_frame is not None
        and slow_frame is not None
        and right_frame is not None
        and long_route_done
        and not has_collision(frames)
    ):
        events.append({
            "event": "task_success",
            "timestamp": max(
                float(accel_frame.get("timestamp", 0.0)),
                float(left_frame.get("timestamp", 0.0)),
                float(slow_frame.get("timestamp", 0.0)),
                float(right_frame.get("timestamp", 0.0)),
                float(final_frame.get("timestamp", 0.0)),
            ),
            "success": True,
        })

    return events


def detect_complex_scene2(frames, cfg):
    events = []

    scene_cfg = cfg.get("success_criteria", {}).get("complex_scene2", {})
    if not scene_cfg.get("enabled", False):
        return events

    ped_detected = first_true_frame(frames, "pedestrian_detected")
    ped_slowdown = first_true_frame(frames, "ped_slowdown_started")
    ped_done = first_true_frame(frames, "safe_slowdown_completed")
    slow_vehicle = first_true_frame(frames, "slow_vehicle_detected")
    lane_change = first_true_frame(frames, "lane_change_started")
    overtake = first_true_frame(frames, "overtake_completed")
    return_lane = first_true_frame(frames, "return_to_lane_completed")
    bus_detected = first_true_frame(frames, "bus_stop_detected")
    bus_slowdown = first_true_frame(frames, "bus_stop_slowdown_started")
    bus_done = first_true_frame(frames, "bus_stop_pass_completed")
    route_done = first_true_frame(frames, "long_route_completed")

    if ped_detected is not None:
        events.append({
            "event": "pedestrian_detected",
            "timestamp": ped_detected.get("timestamp", 0.0),
            "distance_to_pedestrian": ped_detected.get("distance_to_pedestrian"),
            "detection_distance_m": scene_cfg.get("pedestrian_detection_distance_m"),
        })

    if ped_slowdown is not None:
        response_time = None
        if ped_detected is not None:
            response_time = float(ped_slowdown.get("timestamp", 0.0)) - float(ped_detected.get("timestamp", 0.0))
        events.append({
            "event": "slowdown_started",
            "timestamp": ped_slowdown.get("timestamp", 0.0),
            "distance_to_pedestrian": ped_slowdown.get("distance_to_pedestrian"),
            "safe_speed_kmh": scene_cfg.get("pedestrian_safe_speed_kmh"),
            "response_time_s": response_time,
        })

    if ped_done is not None:
        events.append({
            "event": "safe_slowdown_completed",
            "timestamp": ped_done.get("timestamp", 0.0),
            "distance_to_pedestrian": ped_done.get("distance_to_pedestrian"),
            "ego_speed_kmh": ped_done.get("ego_speed_kmh"),
            "slowdown_hold_time": ped_done.get("pedestrian_hold_time"),
            "required_slowdown_seconds": scene_cfg.get("pedestrian_required_hold_seconds"),
            "min_distance_to_pedestrian": ped_done.get("min_distance_to_pedestrian_so_far"),
            "safe_speed_kmh": scene_cfg.get("pedestrian_safe_speed_kmh"),
            "min_safe_distance_m": scene_cfg.get("pedestrian_min_safe_distance_m"),
        })

    if slow_vehicle is not None:
        events.append({
            "event": "slow_vehicle_detected",
            "timestamp": slow_vehicle.get("timestamp", 0.0),
            "front_vehicle_gap": slow_vehicle.get("front_vehicle_gap"),
            "front_vehicle_distance": slow_vehicle.get("front_vehicle_distance"),
        })

    if lane_change is not None:
        response_time = None
        if slow_vehicle is not None:
            response_time = float(lane_change.get("timestamp", 0.0)) - float(slow_vehicle.get("timestamp", 0.0))
        events.append({
            "event": "lane_change_started",
            "timestamp": lane_change.get("timestamp", 0.0),
            "front_vehicle_gap": lane_change.get("front_vehicle_gap"),
            "target_lateral_offset_m": lane_change.get("target_lateral_offset_m"),
            "response_time_s": response_time,
        })

    if overtake is not None:
        events.append({
            "event": "overtake_completed",
            "timestamp": overtake.get("timestamp", 0.0),
            "front_vehicle_gap": overtake.get("front_vehicle_gap"),
            "min_front_vehicle_gap_m": overtake.get("min_front_vehicle_gap_so_far"),
        })

    if return_lane is not None:
        events.append({
            "event": "return_to_lane_completed",
            "timestamp": return_lane.get("timestamp", 0.0),
            "return_hold_time": return_lane.get("return_hold_time"),
            "required_return_hold_seconds": scene_cfg.get("return_required_hold_seconds"),
            "lateral_offset_from_lane_center": return_lane.get("lateral_offset_from_route_m"),
            "return_lane_tolerance_m": scene_cfg.get("return_lane_tolerance_m"),
        })

    if bus_detected is not None:
        events.append({
            "event": "bus_stop_detected",
            "timestamp": bus_detected.get("timestamp", 0.0),
            "distance_to_bus_stop": bus_detected.get("distance_to_bus_stop"),
        })

    if bus_slowdown is not None:
        response_time = None
        if bus_detected is not None:
            response_time = float(bus_slowdown.get("timestamp", 0.0)) - float(bus_detected.get("timestamp", 0.0))
        events.append({
            "event": "bus_stop_slowdown_started",
            "timestamp": bus_slowdown.get("timestamp", 0.0),
            "distance_to_bus_stop": bus_slowdown.get("distance_to_bus_stop"),
            "target_speed_kmh": scene_cfg.get("bus_stop_target_speed_kmh"),
            "response_time_s": response_time,
        })

    if bus_done is not None:
        events.append({
            "event": "bus_stop_pass_completed",
            "timestamp": bus_done.get("timestamp", 0.0),
            "ego_speed_kmh": bus_done.get("ego_speed_kmh"),
            "bus_stop_hold_time": bus_done.get("bus_stop_hold_time"),
            "required_hold_seconds": scene_cfg.get("bus_stop_required_hold_seconds"),
            "target_speed_kmh": scene_cfg.get("bus_stop_target_speed_kmh"),
        })

    if route_done is not None:
        events.append({
            "event": "long_route_completed",
            "timestamp": route_done.get("timestamp", 0.0),
            "max_route_progress_m": route_done.get("max_route_progress_m"),
            "target_progress_m": scene_cfg.get("target_progress_m"),
            "route_completion": route_done.get("route_completion"),
        })

    if (
        ped_done is not None
        and overtake is not None
        and return_lane is not None
        and bus_done is not None
        and route_done is not None
        and not has_collision(frames)
    ):
        events.append({
            "event": "task_success",
            "timestamp": max(
                float(ped_done.get("timestamp", 0.0)),
                float(overtake.get("timestamp", 0.0)),
                float(return_lane.get("timestamp", 0.0)),
                float(bus_done.get("timestamp", 0.0)),
                float(route_done.get("timestamp", 0.0)),
            ),
            "success": True,
        })

    return events


def detect_task_failure(frames, events, cfg):
    route_deviation_frame = first_true_frame(frames, "route_deviation")
    if failure_enabled(cfg, "route_deviation", default=True) and route_deviation_frame is not None:
        events.append({
            "event": "task_failure",
            "timestamp": route_deviation_frame.get("timestamp", 0.0),
            "success": False,
            "failure_reason": "route_deviation",
        })
        return events

    red_light_frame = first_true_frame(frames, "red_light_violation")
    if failure_enabled(cfg, "red_light_violation", default=True) and red_light_frame is not None:
        events.append({
            "event": "task_failure",
            "timestamp": red_light_frame.get("timestamp", 0.0),
            "success": False,
            "failure_reason": "red_light_violation",
        })
        return events

    lane_invasion_frame = first_true_frame(frames, "lane_invasion")
    if failure_enabled(cfg, "lane_invasion", default=True) and lane_invasion_frame is not None:
        events.append({
            "event": "task_failure",
            "timestamp": lane_invasion_frame.get("timestamp", 0.0),
            "success": False,
            "failure_reason": "lane_invasion",
        })
        return events

    collision_event = next((event for event in events if event["event"] == "collision"), None)
    if failure_enabled(cfg, "collision", default=True) and collision_event is not None:
        events.append({
            "event": "task_failure",
            "timestamp": collision_event.get("timestamp", 0.0),
            "success": False,
            "failure_reason": "collision",
        })
        return events

    if failure_enabled(cfg, "lane_change_timeout", default=True) and any(event["event"] == "lane_change_timeout" for event in events):
        timeout_event = next(event for event in events if event["event"] == "lane_change_timeout")
        events.append({
            "event": "task_failure",
            "timestamp": timeout_event.get("timestamp", 0.0),
            "success": False,
            "failure_reason": "lane_change_timeout",
        })
        return events

    if any(event["event"] == "task_success" for event in events):
        return events

    last_timestamp = float(frames[-1].get("timestamp", 0.0)) if frames else 0.0
    events.append({
        "event": "task_failure",
        "timestamp": last_timestamp,
        "success": False,
        "failure_reason": "success_criteria_not_met",
    })
    return events


def build_events(cfg, frames):
    events = []
    events.extend(detect_instruction_trigger(frames))
    events.extend(detect_collision(frames))
    events.extend(detect_speed_target(frames, cfg))
    events.extend(detect_lane_change(frames, cfg))
    events.extend(detect_pedestrian_slowdown(frames, cfg))
    events.extend(detect_cone_detour(frames, cfg))
    events.extend(detect_cut_in_brake(frames, cfg))
    events.extend(detect_rain_night_slowdown(frames, cfg))
    events.extend(detect_composite_long_route(frames, cfg))
    events.extend(detect_complex_scene2(frames, cfg))
    events = detect_task_failure(frames, events, cfg)
    return sorted(events, key=lambda event: (float(event.get("timestamp", 0.0)), event.get("event", "")))


class EventDetector:
    def __init__(self, cfg):
        self.cfg = cfg

    @classmethod
    def from_yaml(cls, path):
        return cls(load_yaml(path))

    def detect(self, frames):
        return build_events(self.cfg, frames)

    def save(self, events, output_path):
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as f:
            json.dump(events, f, ensure_ascii=False, indent=2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--frames", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    detector = EventDetector.from_yaml(args.config)
    frames = load_frames(args.frames)
    events = detector.detect(frames)
    detector.save(events, args.output)

    print(f"[OK] loaded frames: {len(frames)}")
    print(f"[OK] events saved to {args.output}")
    for event in events:
        print(event)


if __name__ == "__main__":
    main()
