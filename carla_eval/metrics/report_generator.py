import argparse
import csv
import json
from pathlib import Path
import yaml


def read_jsonl(path: Path):
    records = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                records.append(json.loads(line))
    return records


def calculate_report(config, frames, events):
    collision_count = sum(1 for r in frames if r.get("collision"))
    lane_invasion_count = sum(1 for r in frames if r.get("lane_invasion"))
    red_light_violation_count = sum(1 for r in frames if r.get("red_light_violation"))
    route_deviation_count = sum(1 for r in frames if r.get("route_deviation"))

    success = any(e.get("event") == "task_success" for e in events)

    speed_cfg = config.get("success_criteria", {}).get("target_speed", {})
    target_speed_error = None

    if speed_cfg.get("enabled"):
        target = float(speed_cfg["value_kmh"])
        speeds = [
            float(r.get("ego_speed_kmh", 0))
            for r in frames
            if r.get("instruction_id") is not None
        ]
        if speeds:
            target_speed_error = sum(abs(s - target) for s in speeds) / len(speeds)

    latencies = [
        float(r["end_to_end_latency_ms"])
        for r in frames
        if r.get("end_to_end_latency_ms") is not None
    ]
    mean_latency = sum(latencies) / len(latencies) if latencies else None

    failure_reason = None
    for e in events:
        if e.get("event") == "task_failure":
            failure_reason = e.get("reason")

    report = {
        "scenario_id": config["scenario_id"],
        "category": config["category"],
        "success": success,
        "task_completion_rate": 1.0 if success else 0.0,
        "collision_count": collision_count,
        "lane_invasion_count": lane_invasion_count,
        "red_light_violation_count": red_light_violation_count,
        "route_deviation_count": route_deviation_count,
        "target_speed_error_kmh": target_speed_error,
        "mean_end_to_end_latency_ms": mean_latency,
        "failure_reason": failure_reason,
    }

    return report


def save_report(report, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)

    json_path = output_dir / "evaluation_report.json"
    json_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False),
        encoding="utf-8"
    )

    csv_path = output_dir / "evaluation_report.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(report.keys()))
        writer.writeheader()
        writer.writerow(report)

    print(f"[OK] report saved to {json_path}")
    print(f"[OK] report saved to {csv_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--frames", required=True)
    parser.add_argument("--events", required=True)
    parser.add_argument("--output_dir", required=True)
    args = parser.parse_args()

    with open(args.config, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    frames = read_jsonl(Path(args.frames))
    events = json.loads(Path(args.events).read_text(encoding="utf-8"))

    report = calculate_report(config, frames, events)
    save_report(report, Path(args.output_dir))

    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
