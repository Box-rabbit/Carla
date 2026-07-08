import argparse
import csv
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


def load_events(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def count_true(frames, key):
    return sum(1 for r in frames if r.get(key))


def find_event(events, name):
    for e in events:
        if e.get("event") == name:
            return e
    return None


def mean(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return sum(values) / len(values)


def build_report(cfg, frames, events):
    scenario_id = cfg.get("scenario_id")
    category = cfg.get("category")

    task_success = find_event(events, "task_success")
    task_failure = find_event(events, "task_failure")

    success = bool(task_success and task_success.get("success", False))

    report = {
        "scenario_id": scenario_id,
        "category": category,
        "success": success,
        "task_completion_rate": 1.0 if success else 0.0,
        "collision_count": count_true(frames, "collision"),
        "lane_invasion_count": count_true(frames, "lane_invasion"),
        "red_light_violation_count": count_true(frames, "red_light_violation"),
        "route_deviation_count": count_true(frames, "route_deviation"),
        "target_speed_error_kmh": None,
        "mean_end_to_end_latency_ms": mean(
            [r.get("end_to_end_latency_ms") for r in frames]
        ),
        "failure_reason": task_failure.get("failure_reason") if task_failure else None,
    }

    # S01: speed target
    speed_event = find_event(events, "speed_target_reached")
    if speed_event is not None:
        target_speed = speed_event.get("target_speed_kmh")
        actual_speed = speed_event.get("actual_speed_kmh")
        if target_speed is not None and actual_speed is not None:
            report["target_speed_error_kmh"] = abs(float(actual_speed) - float(target_speed))

        report.update({
            "speed_target_reached": True,
            "target_speed_kmh": target_speed,
            "actual_speed_kmh": actual_speed,
            "speed_required_hold_seconds": speed_event.get("required_hold_seconds"),
        })

    # S02: lane change
    lane_started = find_event(events, "lane_change_started")
    lane_completed = find_event(events, "lane_change_completed")
    if lane_started is not None or lane_completed is not None:
        report.update({
            "lane_change_success": lane_completed is not None and success,
            "lane_change_started_time_s": lane_started.get("timestamp") if lane_started else None,
            "lane_change_completed_time_s": lane_completed.get("timestamp") if lane_completed else None,
            "lane_change_duration_s": lane_completed.get("lane_change_duration_s") if lane_completed else None,
            "target_lane_hold_s": lane_completed.get("target_lane_hold_s") if lane_completed else None,
            "initial_lane_id": lane_completed.get("initial_lane_id") if lane_completed else (
                lane_started.get("initial_lane_id") if lane_started else None
            ),
            "target_lane_id": lane_completed.get("target_lane_id") if lane_completed else (
                lane_started.get("target_lane_id") if lane_started else None
            ),
            "current_lane_id": lane_completed.get("current_lane_id") if lane_completed else None,
        })

    # S04: pedestrian slowdown
    ped_detected = find_event(events, "pedestrian_detected")
    slowdown_started = find_event(events, "slowdown_started")
    safe_slowdown = find_event(events, "safe_slowdown_completed")

    if ped_detected is not None or slowdown_started is not None or safe_slowdown is not None:
        speeds = [float(r.get("ego_speed_kmh", 0.0)) for r in frames]
        distances = [
            float(r.get("distance_to_pedestrian"))
            for r in frames
            if r.get("distance_to_pedestrian") is not None
        ]

        max_speed = max(speeds) if speeds else None
        min_speed = min(speeds) if speeds else None

        report.update({
            "pedestrian_detected": ped_detected is not None,
            "pedestrian_detected_time_s": ped_detected.get("timestamp") if ped_detected else None,
            "slowdown_started": slowdown_started is not None,
            "slowdown_started_time_s": slowdown_started.get("timestamp") if slowdown_started else None,
            "slowdown_response_time_s": slowdown_started.get("response_time_s") if slowdown_started else None,
            "safe_slowdown_completed": safe_slowdown is not None,
            "safe_slowdown_completed_time_s": safe_slowdown.get("timestamp") if safe_slowdown else None,
            "slowdown_success": safe_slowdown is not None and success,
            "min_distance_to_pedestrian": (
                safe_slowdown.get("min_distance_to_pedestrian")
                if safe_slowdown else (min(distances) if distances else None)
            ),
            "distance_to_pedestrian_at_completion": (
                safe_slowdown.get("distance_to_pedestrian") if safe_slowdown else None
            ),
            "ego_speed_at_slowdown_completion_kmh": (
                safe_slowdown.get("ego_speed_kmh") if safe_slowdown else None
            ),
            "safe_speed_kmh": safe_slowdown.get("safe_speed_kmh") if safe_slowdown else None,
            "min_safe_distance_m": safe_slowdown.get("min_safe_distance_m") if safe_slowdown else None,
            "slowdown_hold_time_s": safe_slowdown.get("slowdown_hold_time") if safe_slowdown else None,
            "max_speed_before_slowdown_kmh": (
                safe_slowdown.get("max_speed_before_slowdown") if safe_slowdown else max_speed
            ),
            "min_speed_kmh": safe_slowdown.get("min_speed") if safe_slowdown else min_speed,
            "speed_drop_kmh": (
                (safe_slowdown.get("max_speed_before_slowdown") - safe_slowdown.get("ego_speed_kmh"))
                if safe_slowdown
                and safe_slowdown.get("max_speed_before_slowdown") is not None
                and safe_slowdown.get("ego_speed_kmh") is not None
                else None
            ),
        })


    # S05: cone detour
    cone_detected = find_event(events, "cone_detected")
    detour_started = find_event(events, "detour_started")
    detour_completed = find_event(events, "detour_completed")
    return_completed = find_event(events, "return_to_lane_completed")

    if cone_detected is not None or detour_started is not None or detour_completed is not None or return_completed is not None:
        distances = [
            float(r.get("distance_to_cone"))
            for r in frames
            if r.get("distance_to_cone") is not None
        ]
        lateral_offsets = [
            abs(float(r.get("lateral_offset_from_lane_center")))
            for r in frames
            if r.get("lateral_offset_from_lane_center") is not None
        ]

        report.update({
            "cone_detected": cone_detected is not None,
            "cone_detected_time_s": cone_detected.get("timestamp") if cone_detected else None,
            "detour_started": detour_started is not None,
            "detour_started_time_s": detour_started.get("timestamp") if detour_started else None,
            "detour_response_time_s": detour_started.get("response_time_s") if detour_started else None,
            "detour_completed": detour_completed is not None,
            "detour_completed_time_s": detour_completed.get("timestamp") if detour_completed else None,
            "return_to_lane_completed": return_completed is not None,
            "return_to_lane_completed_time_s": return_completed.get("timestamp") if return_completed else None,
            "detour_success": detour_completed is not None and return_completed is not None and success,
            "return_to_lane_success": return_completed is not None and success,
            "min_distance_to_cone": (
                return_completed.get("min_distance_to_cone")
                if return_completed else (min(distances) if distances else None)
            ),
            "min_safe_distance_to_cone_m": (
                return_completed.get("min_safe_distance_to_cone_m") if return_completed else None
            ),
            "max_lateral_offset_m": (
                return_completed.get("max_lateral_offset_m")
                if return_completed else (max(lateral_offsets) if lateral_offsets else None)
            ),
            "return_hold_time_s": return_completed.get("return_hold_time") if return_completed else None,
            "required_return_hold_seconds": (
                return_completed.get("required_return_hold_seconds") if return_completed else None
            ),
            "no_cone_ahead_hold_time_s": (
                detour_completed.get("no_cone_ahead_hold_time") if detour_completed else None
            ),
            "detection_trigger_mode": "front-cone-detection",
        })



    # S07: cut-in brake emergency response
    cut_in_detected = find_event(events, "cut_in_detected")
    emergency_brake = find_event(events, "emergency_brake_started")
    safe_brake = find_event(events, "safe_brake_completed")

    if cut_in_detected is not None or emergency_brake is not None or safe_brake is not None:
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

        report.update({
            "cut_in_detected": cut_in_detected is not None,
            "cut_in_detected_time_s": cut_in_detected.get("timestamp") if cut_in_detected else None,

            "emergency_brake_started": emergency_brake is not None,
            "emergency_brake_started_time_s": emergency_brake.get("timestamp") if emergency_brake else None,
            "emergency_response_time_s": emergency_brake.get("response_time_s") if emergency_brake else None,
            "emergency_response_latency_ms": (
                emergency_brake.get("response_time_s") * 1000
                if emergency_brake and emergency_brake.get("response_time_s") is not None
                else None
            ),

            "safe_brake_completed": safe_brake is not None,
            "safe_brake_completed_time_s": safe_brake.get("timestamp") if safe_brake else None,

            "brake_reaction_success": emergency_brake is not None and safe_brake is not None and success,
            "safe_follow_success": safe_brake is not None and success,

            "min_front_vehicle_distance": (
                safe_brake.get("min_front_vehicle_distance")
                if safe_brake else (min(distances) if distances else None)
            ),
            "min_front_gap": (
                safe_brake.get("min_front_gap")
                if safe_brake else (min(gaps) if gaps else None)
            ),
            "min_ttc": (
                safe_brake.get("min_ttc")
                if safe_brake else (min(ttcs) if ttcs else None)
            ),
            "max_brake": (
                safe_brake.get("max_brake")
                if safe_brake else (max(brakes) if brakes else None)
            ),

            "safe_follow_distance_m": safe_brake.get("safe_follow_distance_m") if safe_brake else None,
            "safe_speed_kmh": safe_brake.get("safe_speed_kmh") if safe_brake else None,
            "safe_follow_hold_time_s": safe_brake.get("safe_follow_hold_time") if safe_brake else None,
            "required_safe_follow_seconds": safe_brake.get("required_safe_follow_seconds") if safe_brake else None,

            "emergency_trigger_mode": "front-vehicle-distance-or-ttc",
        })



    # S08: rain-night danger slowdown
    danger_detected = find_event(events, "danger_detected")
    rain_slowdown = find_event(events, "slowdown_started")
    safe_speed = find_event(events, "safe_speed_reached")

    if danger_detected is not None or rain_slowdown is not None or safe_speed is not None:
        speeds = [
            float(r.get("ego_speed_kmh", 0.0))
            for r in frames
        ]
        hazards = [
            float(r.get("hazard_score", 0.0))
            for r in frames
        ]

        report.update({
            "rain_night_danger_detected": danger_detected is not None,
            "danger_detected_time_s": danger_detected.get("timestamp") if danger_detected else None,
            "hazard_score": danger_detected.get("hazard_score") if danger_detected else (
                sum(hazards) / len(hazards) if hazards else None
            ),
            "hazard_score_threshold": danger_detected.get("hazard_score_threshold") if danger_detected else None,

            "weather_precipitation": danger_detected.get("weather_precipitation") if danger_detected else None,
            "weather_wetness": danger_detected.get("weather_wetness") if danger_detected else None,
            "weather_fog_density": danger_detected.get("weather_fog_density") if danger_detected else None,
            "is_night": danger_detected.get("is_night") if danger_detected else None,
            "sun_altitude_angle": danger_detected.get("sun_altitude_angle") if danger_detected else None,

            "slowdown_started": rain_slowdown is not None,
            "slowdown_started_time_s": rain_slowdown.get("timestamp") if rain_slowdown else None,
            "slowdown_response_time_s": rain_slowdown.get("response_time_s") if rain_slowdown else None,

            "safe_speed_reached": safe_speed is not None,
            "safe_speed_reached_time_s": safe_speed.get("timestamp") if safe_speed else None,
            "target_speed_limit_success": safe_speed is not None and success,

            "safe_speed_kmh": safe_speed.get("safe_speed_kmh") if safe_speed else None,
            "ego_speed_at_safe_speed_reached_kmh": safe_speed.get("ego_speed_kmh") if safe_speed else None,
            "safe_speed_hold_time_s": safe_speed.get("safe_speed_hold_time") if safe_speed else None,
            "required_safe_speed_hold_seconds": safe_speed.get("required_safe_speed_hold_seconds") if safe_speed else None,

            "travelled_distance_m": safe_speed.get("travelled_distance_m") if safe_speed else (
                frames[-1].get("travelled_distance_m") if frames else None
            ),
            "min_travel_distance_m": safe_speed.get("min_travel_distance_m") if safe_speed else None,

            "mean_speed_kmh": safe_speed.get("mean_speed_kmh") if safe_speed else (
                sum(speeds) / len(speeds) if speeds else None
            ),
            "max_speed_kmh": safe_speed.get("max_speed_kmh") if safe_speed else (
                max(speeds) if speeds else None
            ),
            "min_speed_kmh": safe_speed.get("min_speed_kmh") if safe_speed else (
                min(speeds) if speeds else None
            ),
            "mean_hazard_score": safe_speed.get("mean_hazard_score") if safe_speed else (
                sum(hazards) / len(hazards) if hazards else None
            ),

            "danger_slowdown_success": safe_speed is not None and success,
            "danger_trigger_mode": "weather-hazard-score",
        })


    return report


def save_json(report, output_path):
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)


def save_csv(report, output_path):
    with open(output_path, "w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["metric", "value"])
        for k, v in report.items():
            writer.writerow([k, v])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--frames", required=True)
    parser.add_argument("--events", required=True)
    parser.add_argument("--output_dir", required=True)
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    frames = load_frames(args.frames)
    events = load_events(args.events)

    report = build_report(cfg, frames, events)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    json_path = output_dir / "evaluation_report.json"
    csv_path = output_dir / "evaluation_report.csv"

    save_json(report, json_path)
    save_csv(report, csv_path)

    print(f"[OK] report saved to {json_path}")
    print(f"[OK] report saved to {csv_path}")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
