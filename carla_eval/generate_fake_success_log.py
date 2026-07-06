import json
from pathlib import Path


def main():
    scenario_id = "S01_keep_lane_speed"
    out_dir = Path("logs") / scenario_id
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "frames.jsonl"

    records = []

    for i in range(400):
        t = i * 0.05
        speed = min(62.0, t * 4.0)

        record = {
            "timestamp": t,
            "frame": i,
            "scenario_id": scenario_id,
            "instruction_id": "cmd_001" if t >= 3.0 else None,
            "ego_x": t * 3.0,
            "ego_y": 0.0,
            "ego_z": 0.3,
            "ego_speed_kmh": speed,
            "steer": 0.0,
            "throttle": 0.5 if speed < 60 else 0.2,
            "brake": 0.0,
            "collision": False,
            "lane_invasion": False,
            "red_light_violation": False,
            "route_deviation": False,
            "distance_to_front_actor": None,
            "asr_latency_ms": 0,
            "parser_latency_ms": 0,
            "model_latency_ms": 1,
            "end_to_end_latency_ms": 30,
        }

        records.append(record)

    with path.open("w", encoding="utf-8") as f:
        for r in records:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")

    print(f"[OK] fake success log saved to {path}")


if __name__ == "__main__":
    main()
