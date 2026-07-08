import argparse
import csv
import json
import math
import xml.etree.ElementTree as ET
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


def load_events(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def count_true(frames, key):
    return sum(1 for record in frames if record.get(key))


def count_activations(frames, key):
    count = 0
    previous = False
    for record in frames:
        current = bool(record.get(key))
        if current and not previous:
            count += 1
        previous = current
    return count


def find_event(events, name):
    for event in events:
        if event.get("event") == name:
            return event
    return None


def find_all_events(events, name):
    return [event for event in events if event.get("event") == name]


def mean(values):
    values = [value for value in values if value is not None]
    if not values:
        return None
    return sum(values) / len(values)


def euclidean_distance(a, b):
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    dz = float(a[2]) - float(b[2])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def cumulative_travel_distance(frames):
    if len(frames) < 2:
        return 0.0

    total = 0.0
    last = (
        float(frames[0].get("ego_x", 0.0)),
        float(frames[0].get("ego_y", 0.0)),
        float(frames[0].get("ego_z", 0.0)),
    )

    for record in frames[1:]:
        current = (
            float(record.get("ego_x", 0.0)),
            float(record.get("ego_y", 0.0)),
            float(record.get("ego_z", 0.0)),
        )
        total += euclidean_distance(current, last)
        last = current

    return total


def resolve_repo_relative_path(cfg, relative_path):
    if not relative_path:
        return None

    path = Path(relative_path)
    if path.is_absolute() and path.exists():
        return path

    if path.exists():
        return path.resolve()

    config_path = cfg.get("__config_path__")
    if config_path:
        repo_root = Path(config_path).resolve().parents[3]
        candidate = repo_root / relative_path
        if candidate.exists():
            return candidate

    return None


def route_length_from_xml(cfg):
    route_cfg = cfg.get("route", {})
    route_path = resolve_repo_relative_path(cfg, route_cfg.get("route_file"))
    if route_path is None or not route_path.exists():
        return None

    route_id = str(route_cfg.get("route_id", 0))

    try:
        root = ET.parse(route_path).getroot()
    except ET.ParseError:
        return None

    for route in root.findall("route"):
        if route.get("id") != route_id:
            continue

        points = []
        for waypoint in route.findall("waypoint"):
            points.append((
                float(waypoint.get("x", 0.0)),
                float(waypoint.get("y", 0.0)),
                float(waypoint.get("z", 0.0)),
            ))

        if len(points) < 2:
            return 0.0

        return sum(
            euclidean_distance(points[idx], points[idx - 1])
            for idx in range(1, len(points))
        )

    return None


def estimate_route_completion(cfg, frames):
    logged_completion = [
        float(record.get("route_completion"))
        for record in frames
        if record.get("route_completion") is not None
    ]
    if logged_completion:
        return max(logged_completion)

    route_length = route_length_from_xml(cfg)
    travelled_distance = cumulative_travel_distance(frames)

    if route_length is None or route_length <= 0.0:
        return None

    return max(0.0, min(1.0, travelled_distance / route_length))


def first_instruction_trigger_time(cfg, frames):
    instructions = cfg.get("instructions", [])
    trigger_times = []

    for instruction in instructions:
        trigger = instruction.get("trigger", {})
        if trigger.get("type") == "time" and trigger.get("value") is not None:
            trigger_times.append(float(trigger["value"]))

    if trigger_times:
        return min(trigger_times)

    for record in frames:
        if record.get("instruction_id") is not None:
            return float(record.get("timestamp", 0.0))

    return None


def infer_response_latency_ms(cfg, events, frames):
    trigger_time = first_instruction_trigger_time(cfg, frames)
    response_times_s = []

    lane_cfg = cfg.get("success_criteria", {}).get("lane_change", {})
    if lane_cfg.get("enabled", False):
        lane_started = find_event(events, "lane_change_started")
        if lane_started is not None and trigger_time is not None:
            response_times_s.append(max(0.0, float(lane_started.get("timestamp", 0.0)) - trigger_time))

    speed_cfg = cfg.get("success_criteria", {}).get("target_speed", {})
    if speed_cfg.get("enabled", False):
        speed_event = find_event(events, "speed_target_reached")
        if speed_event is not None and trigger_time is not None:
            response_times_s.append(max(0.0, float(speed_event.get("timestamp", 0.0)) - trigger_time))

    for name in ("slowdown_started", "detour_started", "emergency_brake_started"):
        event = find_event(events, name)
        if event is None:
            continue
        if event.get("response_time_s") is not None:
            response_times_s.append(float(event.get("response_time_s")))
        elif trigger_time is not None:
            response_times_s.append(max(0.0, float(event.get("timestamp", 0.0)) - trigger_time))

    response_ms = [value * 1000.0 for value in response_times_s]
    return mean(response_ms)


def infer_subtask_metrics(cfg, events):
    expected = []
    for instruction in cfg.get("instructions", []):
        expected.extend(instruction.get("expected_subtasks", []))

    if not expected:
        return 0.0, 1.0

    event_names = {event.get("event") for event in events}
    mapping = {
        "keep_lane": {"task_success"},
        "reach_target_speed": {"speed_target_reached"},
        "target_speed_control": {"speed_target_reached"},
        "lane_change": {"lane_change_completed"},
        "change_lane_left": {"lane_change_completed"},
        "change_lane_right": {"lane_change_completed"},
        "avoid_pedestrian": {"safe_slowdown_completed"},
        "slow_down": {"safe_slowdown_completed", "safe_speed_reached"},
        "detour_and_return": {"return_to_lane_completed"},
        "return_to_original_lane": {"return_to_lane_completed"},
        "emergency_brake": {"safe_brake_completed"},
        "safe_speed_control": {"safe_speed_reached"},
        "turn": {"turn_completed"},
        "stop": {"stop_completed"},
    }

    completed = 0
    for subtask in expected:
        expected_events = mapping.get(subtask, {subtask})
        if any(name in event_names for name in expected_events):
            completed += 1

    total = len(expected)
    missing = total - completed
    return missing / total, completed / total


def build_report(cfg, frames, events):
    scenario_id = cfg.get("scenario_id")
    category = cfg.get("category")
    log_scenario_id = frames[0].get("scenario_id") if frames else scenario_id

    task_success = find_event(events, "task_success")
    task_failure = find_event(events, "task_failure")
    success = bool(task_success and task_success.get("success", False))

    collision_count = count_activations(frames, "collision")
    lane_invasion_count = count_activations(frames, "lane_invasion")
    red_light_violation_count = count_activations(frames, "red_light_violation")
    route_deviation_count = count_activations(frames, "route_deviation")
    violation_count = lane_invasion_count + red_light_violation_count + route_deviation_count
    route_completion = estimate_route_completion(cfg, frames)
    mean_response_latency_ms = infer_response_latency_ms(cfg, events, frames)
    subtask_missing_rate, action_success_rate = infer_subtask_metrics(cfg, events)

    report = {
        "scenario_id": scenario_id,
        "log_scenario_id": log_scenario_id,
        "category": category,
        "success": success,
        "task_completion_rate": 1.0 if success else 0.0,
        "collision_count": collision_count,
        "lane_invasion_count": lane_invasion_count,
        "red_light_violation_count": red_light_violation_count,
        "route_deviation_count": route_deviation_count,
        "violation_count": violation_count,
        "route_completion": route_completion,
        "target_speed_error_kmh": None,
        "target_speed_error": None,
        "mean_response_latency_ms": mean_response_latency_ms,
        "response_latency_ms": mean_response_latency_ms,
        "mean_end_to_end_latency_ms": mean([record.get("end_to_end_latency_ms") for record in frames]),
        "mean_asr_latency_ms": mean([record.get("asr_latency_ms") for record in frames]),
        "mean_parser_latency_ms": mean([record.get("parser_latency_ms") for record in frames]),
        "mean_model_latency_ms": mean([record.get("model_latency_ms") for record in frames]),
        "subtask_missing_rate": subtask_missing_rate,
        "action_success_rate": action_success_rate,
        "failure_reason": task_failure.get("failure_reason") if task_failure else None,
    }

    speed_cfg = cfg.get("success_criteria", {}).get("target_speed", {})
    if speed_cfg.get("enabled", False):
        speed_event = find_event(events, "speed_target_reached")
        if speed_event is not None:
            target_speed = speed_event.get("target_speed_kmh")
            actual_speed = speed_event.get("actual_speed_kmh")
            if target_speed is not None and actual_speed is not None:
                report["target_speed_error_kmh"] = abs(float(actual_speed) - float(target_speed))
                report["target_speed_error"] = report["target_speed_error_kmh"]

            report.update({
                "speed_target_reached": True,
                "target_speed_kmh": target_speed,
                "actual_speed_kmh": actual_speed,
                "speed_required_hold_seconds": speed_event.get("required_hold_seconds"),
            })

    lane_cfg = cfg.get("success_criteria", {}).get("lane_change", {})
    if lane_cfg.get("enabled", False):
        lane_started = find_event(events, "lane_change_started")
        lane_completed = find_event(events, "lane_change_completed")
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

    ped_cfg = cfg.get("success_criteria", {}).get("pedestrian_slowdown", {})
    if ped_cfg.get("enabled", False):
        ped_detected = find_event(events, "pedestrian_detected")
        slowdown_started = find_event(events, "slowdown_started")
        safe_slowdown = find_event(events, "safe_slowdown_completed")

        speeds = [float(record.get("ego_speed_kmh", 0.0)) for record in frames]
        distances = [
            float(record.get("distance_to_pedestrian"))
            for record in frames
            if record.get("distance_to_pedestrian") is not None
        ]

        max_speed = max(speeds) if speeds else None
        min_speed = min(speeds) if speeds else None
        speed_drop = None
        if (
            safe_slowdown is not None
            and safe_slowdown.get("max_speed_before_slowdown") is not None
            and safe_slowdown.get("ego_speed_kmh") is not None
        ):
            speed_drop = (
                float(safe_slowdown.get("max_speed_before_slowdown"))
                - float(safe_slowdown.get("ego_speed_kmh"))
            )

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
            "speed_drop_kmh": speed_drop,
            "speed_drop": speed_drop,
        })

    cone_cfg = cfg.get("success_criteria", {}).get("cone_detour", {})
    if cone_cfg.get("enabled", False):
        cone_detected = find_event(events, "cone_detected")
        detour_started = find_event(events, "detour_started")
        detour_completed = find_event(events, "detour_completed")
        return_completed = find_event(events, "return_to_lane_completed")

        distances = [
            float(record.get("distance_to_cone"))
            for record in frames
            if record.get("distance_to_cone") is not None
        ]
        lateral_offsets = [
            abs(float(record.get("lateral_offset_from_lane_center")))
            for record in frames
            if record.get("lateral_offset_from_lane_center") is not None
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

    cut_cfg = cfg.get("success_criteria", {}).get("cut_in_brake", {})
    if cut_cfg.get("enabled", False):
        cut_in_detected = find_event(events, "cut_in_detected")
        emergency_brake = find_event(events, "emergency_brake_started")
        safe_brake = find_event(events, "safe_brake_completed")

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

        report.update({
            "cut_in_detected": cut_in_detected is not None,
            "cut_in_detected_time_s": cut_in_detected.get("timestamp") if cut_in_detected else None,
            "emergency_brake_started": emergency_brake is not None,
            "emergency_brake_started_time_s": emergency_brake.get("timestamp") if emergency_brake else None,
            "emergency_response_time_s": emergency_brake.get("response_time_s") if emergency_brake else None,
            "emergency_response_latency_ms": (
                float(emergency_brake.get("response_time_s")) * 1000.0
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
            "min_distance_to_front_vehicle": (
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

    rain_cfg = cfg.get("success_criteria", {}).get("rain_night_slowdown", {})
    if rain_cfg.get("enabled", False):
        danger_detected = find_event(events, "danger_detected")
        slowdown_started = find_event(events, "slowdown_started")
        safe_speed = find_event(events, "safe_speed_reached")

        speeds = [float(record.get("ego_speed_kmh", 0.0)) for record in frames]
        hazards = [float(record.get("hazard_score", 0.0)) for record in frames]

        report.update({
            "rain_night_danger_detected": danger_detected is not None,
            "danger_detected_time_s": danger_detected.get("timestamp") if danger_detected else None,
            "hazard_score": danger_detected.get("hazard_score") if danger_detected else mean(hazards),
            "hazard_score_threshold": danger_detected.get("hazard_score_threshold") if danger_detected else None,
            "weather_precipitation": danger_detected.get("weather_precipitation") if danger_detected else None,
            "weather_wetness": danger_detected.get("weather_wetness") if danger_detected else None,
            "weather_fog_density": danger_detected.get("weather_fog_density") if danger_detected else None,
            "is_night": danger_detected.get("is_night") if danger_detected else None,
            "sun_altitude_angle": danger_detected.get("sun_altitude_angle") if danger_detected else None,
            "slowdown_started": slowdown_started is not None,
            "slowdown_started_time_s": slowdown_started.get("timestamp") if slowdown_started else None,
            "slowdown_response_time_s": slowdown_started.get("response_time_s") if slowdown_started else None,
            "safe_speed_reached": safe_speed is not None,
            "safe_speed_reached_time_s": safe_speed.get("timestamp") if safe_speed else None,
            "target_speed_limit_success": safe_speed is not None and success,
            "safe_speed_kmh": safe_speed.get("safe_speed_kmh") if safe_speed else None,
            "ego_speed_at_safe_speed_reached_kmh": safe_speed.get("ego_speed_kmh") if safe_speed else None,
            "safe_speed_hold_time_s": safe_speed.get("safe_speed_hold_time") if safe_speed else None,
            "safe_speed_hold_time": safe_speed.get("safe_speed_hold_time") if safe_speed else None,
            "required_safe_speed_hold_seconds": safe_speed.get("required_safe_speed_hold_seconds") if safe_speed else None,
            "travelled_distance_m": safe_speed.get("travelled_distance_m") if safe_speed else (
                frames[-1].get("travelled_distance_m") if frames else None
            ),
            "min_travel_distance_m": safe_speed.get("min_travel_distance_m") if safe_speed else None,
            "mean_speed_kmh": safe_speed.get("mean_speed_kmh") if safe_speed else mean(speeds),
            "mean_speed": safe_speed.get("mean_speed_kmh") if safe_speed else mean(speeds),
            "max_speed_kmh": safe_speed.get("max_speed_kmh") if safe_speed else (max(speeds) if speeds else None),
            "min_speed_kmh": safe_speed.get("min_speed_kmh") if safe_speed else (min(speeds) if speeds else None),
            "mean_hazard_score": safe_speed.get("mean_hazard_score") if safe_speed else mean(hazards),
            "danger_slowdown_success": safe_speed is not None and success,
            "danger_trigger_mode": "weather-hazard-score",
        })

    return report


class ReportGenerator:
    def __init__(self, cfg):
        self.cfg = cfg

    @classmethod
    def from_yaml(cls, path):
        return cls(load_yaml(path))

    def generate(self, frames, events):
        return build_report(self.cfg, frames, events)

    def save(self, report, output_dir):
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        json_path = output_dir / "evaluation_report.json"
        csv_path = output_dir / "evaluation_report.csv"

        with json_path.open("w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)

        with csv_path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["metric", "value"])
            for key, value in report.items():
                writer.writerow([key, value])


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

    generator = ReportGenerator(cfg)
    report = generator.generate(frames, events)
    generator.save(report, args.output_dir)

    print(f"[OK] report saved to {Path(args.output_dir) / 'evaluation_report.json'}")
    print(f"[OK] report saved to {Path(args.output_dir) / 'evaluation_report.csv'}")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
