import argparse
from pathlib import Path
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

    if ok:
        print(f"[PASS] {path} config valid")

    return ok


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    validate_config(Path(args.config))


if __name__ == "__main__":
    main()
