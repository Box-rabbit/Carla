import argparse
import json
from pathlib import Path

import yaml


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_frames(path):
    frames = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                frames.append(json.loads(line))
    return frames


def first_true_frame(frames, key):
    for r in frames:
        if r.get(key):
            return r
    return None


def detect_collision(frames):
    events = []
    r = first_true_frame(frames, "collision")
    if r is not None:
        events.append({
            "event": "collision",
            "timestamp": r.get("timestamp", 0.0),
            "other_actor": r.get("collision_other_actor"),
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

    for r in frames:
        t = float(r.get("timestamp", 0.0))
        speed = float(r.get("ego_speed_kmh", 0.0))

        dt = 0.05 if last_t is None else max(0.0, t - last_t)
        last_t = t

        if low <= speed <= high:
            hold_time += dt
        else:
            hold_time = 0.0

        if hold_time >= required_hold:
            events.append({
                "event": "speed_target_reached",
                "timestamp": t,
                "target_speed_kmh": target,
                "tolerance_kmh": tol,
                "required_hold_seconds": required_hold,
                "actual_speed_kmh": speed,
            })

            if not any(x.get("collision") for x in frames):
                events.append({
                    "event": "task_success",
                    "timestamp": t,
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

    started_frame = None
    completed_frame = None

    for r in frames:
        if r.get("lane_change_started"):
            started_frame = r
            break

    if started_frame is not None:
        events.append({
            "event": "lane_change_started",
            "timestamp": started_frame.get("timestamp", 0.0),
            "initial_lane_id": started_frame.get("initial_lane_id"),
            "target_lane_id": started_frame.get("target_lane_id"),
            "direction": lane_cfg.get("direction", "left"),
        })

    for r in frames:
        if r.get("lane_change_completed"):
            completed_frame = r
            break

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

        if not any(r.get("collision") for r in frames):
            events.append({
                "event": "task_success",
                "timestamp": end_t,
                "success": True,
            })

    else:
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

    detected_frame = None
    slowdown_frame = None
    completed_frame = None

    for r in frames:
        if r.get("pedestrian_detected"):
            detected_frame = r
            break

    if detected_frame is not None:
        events.append({
            "event": "pedestrian_detected",
            "timestamp": detected_frame.get("timestamp", 0.0),
            "distance_to_pedestrian": detected_frame.get("distance_to_pedestrian"),
            "detection_distance_m": detection_distance,
        })

    for r in frames:
        if r.get("slowdown_started"):
            slowdown_frame = r
            break

    if slowdown_frame is not None:
        response_time = None
        if detected_frame is not None:
            response_time = float(slowdown_frame.get("timestamp", 0.0)) - float(detected_frame.get("timestamp", 0.0))

        events.append({
            "event": "slowdown_started",
            "timestamp": slowdown_frame.get("timestamp", 0.0),
            "distance_to_pedestrian": slowdown_frame.get("distance_to_pedestrian"),
            "slowdown_distance_m": slowdown_distance,
            "response_time_s": response_time,
        })

    for r in frames:
        if r.get("safe_slowdown_completed"):
            completed_frame = r
            break

    if completed_frame is not None:
        speeds = [float(x.get("ego_speed_kmh", 0.0)) for x in frames]
        distances = [
            float(x.get("distance_to_pedestrian"))
            for x in frames
            if x.get("distance_to_pedestrian") is not None
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

        if not any(r.get("collision") for r in frames):
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

        if not any(r.get("collision") for r in frames):
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
        float(r.get("front_vehicle_distance"))
        for r in frames
        if r.get("front_vehicle_distance") is not None
    ]

    gaps = [
        float(r.get("front_vehicle_gap"))
        for r in frames
        if r.get("front_vehicle_gap") is not None
    ]

    ttcs = [
        float(r.get("ttc_s"))
        for r in frames
        if r.get("ttc_s") is not None
    ]

    brakes = [
        float(r.get("brake", 0.0))
        for r in frames
    ]

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

        if not any(r.get("collision") for r in frames):
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

    speeds = [
        float(r.get("ego_speed_kmh", 0.0))
        for r in frames
    ]
    hazards = [
        float(r.get("hazard_score", 0.0))
        for r in frames
    ]

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

        if not any(r.get("collision") for r in frames):
            events.append({
                "event": "task_success",
                "timestamp": safe_frame.get("timestamp", 0.0),
                "success": True,
            })

    return events


def detect_task_failure(frames, events):
    if any(e["event"] == "task_success" for e in events):
        return events

    collision_event = next((e for e in events if e["event"] == "collision"), None)
    if collision_event is not None:
        events.append({
            "event": "task_failure",
            "timestamp": collision_event.get("timestamp", 0.0),
            "success": False,
            "failure_reason": "collision",
        })
        return events

    if any(e["event"] == "lane_change_timeout" for e in events):
        timeout_event = next(e for e in events if e["event"] == "lane_change_timeout")
        events.append({
            "event": "task_failure",
            "timestamp": timeout_event.get("timestamp", 0.0),
            "success": False,
            "failure_reason": "lane_change_timeout",
        })
        return events

    return events


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--frames", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    frames = load_frames(args.frames)

    print(f"[OK] loaded frames: {len(frames)}")

    events = []
    events.extend(detect_collision(frames))
    events.extend(detect_speed_target(frames, cfg))
    events.extend(detect_lane_change(frames, cfg))
    events.extend(detect_pedestrian_slowdown(frames, cfg))
    events.extend(detect_cone_detour(frames, cfg))
    events.extend(detect_cut_in_brake(frames, cfg))
    events.extend(detect_rain_night_slowdown(frames, cfg))
    events = detect_task_failure(frames, events)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", encoding="utf-8") as f:
        json.dump(events, f, ensure_ascii=False, indent=2)

    print(f"[OK] events saved to {output_path}")
    for e in events:
        print(e)


if __name__ == "__main__":
    main()
