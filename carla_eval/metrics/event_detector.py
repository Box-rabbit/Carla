import argparse
import json
from pathlib import Path
import yaml


def read_jsonl(path):
    records = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                records.append(json.loads(line))
    return records


def get_target_speed(config):
    # 优先从 success_criteria.target_speed 读取
    speed_cfg = config.get("success_criteria", {}).get("target_speed", {})
    if speed_cfg.get("enabled"):
        return float(speed_cfg["value_kmh"]), float(speed_cfg.get("tolerance_kmh", 5))

    # 如果 success_criteria 中没有，就从 instruction intent 中读取
    for ins in config.get("instructions", []):
        intent = ins.get("intent", {})
        if "target_speed_kmh" in intent:
            return float(intent["target_speed_kmh"]), 8.0

    return None, None


def detect_events(config, frames):
    events = []

    if not frames:
        return [{
            "event": "task_failure",
            "timestamp": None,
            "success": False,
            "reason": "empty_frames"
        }]

    # 1. 碰撞检测
    collision_found = False
    for r in frames:
        if r.get("collision"):
            collision_found = True
            events.append({
                "event": "collision_happened",
                "timestamp": r.get("timestamp"),
                "success": False
            })
            break

    # 2. 目标速度检测
    target_speed, tolerance = get_target_speed(config)
    speed_reached = False

    if target_speed is not None:
        for r in frames:
            speed = float(r.get("ego_speed_kmh", 0))
            if abs(speed - target_speed) <= tolerance:
                speed_reached = True
                events.append({
                    "event": "speed_target_reached",
                    "timestamp": r.get("timestamp"),
                    "speed_kmh": speed,
                    "target_speed_kmh": target_speed,
                    "tolerance_kmh": tolerance,
                    "success": True
                })
                break

    # 3. 任务成功/失败
    if collision_found:
        events.append({
            "event": "task_failure",
            "timestamp": frames[-1].get("timestamp"),
            "success": False,
            "reason": "collision"
        })
    elif target_speed is not None and not speed_reached:
        events.append({
            "event": "task_failure",
            "timestamp": frames[-1].get("timestamp"),
            "success": False,
            "reason": "target_speed_not_reached"
        })
    else:
        events.append({
            "event": "task_success",
            "timestamp": frames[-1].get("timestamp"),
            "success": True
        })

    return events


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--frames", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    with open(args.config, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    frames = read_jsonl(args.frames)
    events = detect_events(config, frames)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(events, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"[OK] loaded frames: {len(frames)}")
    print(f"[OK] events saved to {output}")
    for e in events:
        print(e)


if __name__ == "__main__":
    main()
