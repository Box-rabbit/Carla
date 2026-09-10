import argparse
from pathlib import Path
import sys
import yaml


REQUIRED_TOP_KEYS = [
    "scenario_id",
    "category",
    "runtime",
    "map",
    "ego",
    "instructions",
    "success_criteria",
    "failure_criteria",
    "metrics",
]

VALID_CATEGORIES = {
    "basic_control",
    "complex_obstacle",
    "emergency_response",
}


def _validate_s11_design(cfg):
    """Validate the route/voice spacing contract before CARLA is available."""
    if cfg.get("scenario_id") != "S11_basic_control_scene1_5km":
        return True

    ok = True
    route_cfg = cfg.get("route", {})
    global_route = route_cfg.get("global_route", {})
    constraints = cfg.get("action_windows", {}).get("route_constraints", {})
    min_spacing = float(
        constraints.get("min_event_spacing_m", global_route.get("min_scene_spacing_m", 200.0))
    )
    quiet_distance = float(
        constraints.get("min_initial_quiet_distance_m", 200.0)
    )
    instructions = cfg.get("instructions", [])
    trigger_distances = [
        float(item.get("trigger", {}).get("value"))
        for item in instructions
        if item.get("trigger", {}).get("type") == "route_distance"
        and item.get("trigger", {}).get("value") is not None
    ]
    if len(instructions) < 7 or len(instructions) > 9:
        print(f"[FAIL] S11 requires 7-9 voice tasks, found {len(instructions)}")
        ok = False
    if trigger_distances and min(trigger_distances) < quiet_distance:
        print(f"[FAIL] S11 first task must be at least {quiet_distance:.0f}m after start")
        ok = False
    if any(b - a < min_spacing for a, b in zip(sorted(trigger_distances), sorted(trigger_distances)[1:])):
        print(f"[FAIL] S11 task spacing must be at least {min_spacing:.0f}m")
        ok = False

    required_tasks = {
        "keep_lane", "accelerate", "slow_down", "right_turn", "left_turn",
        "lane_change_left", "lane_change_right", "park",
    }
    actual_tasks = {str(item.get("task", "")) for item in instructions}
    missing = sorted(required_tasks - actual_tasks)
    if missing:
        print(f"[FAIL] S11 missing task types: {', '.join(missing)}")
        ok = False
    return ok


def validate_config(path: Path) -> bool:
    with path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    ok = True

    for key in REQUIRED_TOP_KEYS:
        if key not in cfg:
            print(f"[FAIL] missing top-level key: {key}")
            ok = False

    category = cfg.get("category")
    if category not in VALID_CATEGORIES:
        print(f"[FAIL] invalid category: {category}")
        ok = False

    if not cfg.get("instructions"):
        print("[FAIL] instructions is empty")
        ok = False

    if not cfg.get("metrics", {}).get("required"):
        print("[FAIL] metrics.required is empty")
        ok = False

    ok = _validate_s11_design(cfg) and ok

    if ok:
        print(f"[PASS] {path} config valid")

    return ok


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    return 0 if validate_config(Path(args.config)) else 1


if __name__ == "__main__":
    sys.exit(main())
