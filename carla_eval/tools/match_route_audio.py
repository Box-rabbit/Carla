"""Match recorded voice commands to benchmark routes and route events.

This script is intentionally offline: it only reads route XML, scenario
annotations, scenario YAML files, and Voice2LMDrive result JSON files.
It does not connect to CARLA and does not modify existing scenario configs.
"""

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


DEFAULT_RESULTS_DIR = Path("results")
DEFAULT_ROUTES = Path("routes/dongfeng_benchmark.xml")
DEFAULT_SCENARIOS = Path("configs/scenario_annotations/dongfeng_benchmark.yaml")
DEFAULT_OUTPUT = Path("configs/lmdrive/route_audio_matches.yaml")
DEFAULT_PREFER_ROUTE = "S11_basic_control_scene1_5km"


INTENT_ALIASES = {
    "GO_STRAIGHT": {"GO_STRAIGHT", "KEEP_LANE", "LONG_ROUTE", "ACCELERATE", "ACCELERATE_80"},
    "KEEP_LANE": {"GO_STRAIGHT", "KEEP_LANE", "LONG_ROUTE"},
    "TURN_LEFT": {"TURN_LEFT", "LEFT_TURN"},
    "LEFT_TURN": {"TURN_LEFT", "LEFT_TURN"},
    "TURN_RIGHT": {"TURN_RIGHT", "RIGHT_TURN"},
    "RIGHT_TURN": {"TURN_RIGHT", "RIGHT_TURN"},
    "LANE_CHANGE_LEFT": {"LANE_CHANGE_LEFT", "CHANGE_LEFT"},
    "PEDESTRIAN_CAUTION": {"PEDESTRIAN_CAUTION", "SLOW_DOWN"},
    "SLOW_DOWN": {"SLOW_DOWN", "SLOW_DOWN_30", "PEDESTRIAN_CAUTION"},
}


ACTION_WINDOW_HINTS = {
    "accelerate": {"GO_STRAIGHT", "KEEP_LANE", "ACCELERATE", "ACCELERATE_80"},
    "speed_up": {"GO_STRAIGHT", "KEEP_LANE", "ACCELERATE", "ACCELERATE_80"},
    "left_turn": {"TURN_LEFT", "LEFT_TURN"},
    "turn_left": {"TURN_LEFT", "LEFT_TURN"},
    "right_turn": {"TURN_RIGHT", "RIGHT_TURN"},
    "turn_right": {"TURN_RIGHT", "RIGHT_TURN"},
    "lane_change_left": {"LANE_CHANGE_LEFT", "CHANGE_LEFT"},
    "change_left": {"LANE_CHANGE_LEFT", "CHANGE_LEFT"},
    "slow": {"SLOW_DOWN", "SLOW_DOWN_30"},
    "brake": {"SLOW_DOWN", "SLOW_DOWN_30", "EMERGENCY_BRAKE"},
}


def _repo_rel(path):
    path = Path(path)
    try:
        return path.resolve().relative_to(Path.cwd().resolve()).as_posix()
    except ValueError:
        return path.as_posix()


def _load_yaml(path):
    with Path(path).open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _load_json(path):
    with Path(path).open("r", encoding="utf-8") as f:
        return json.load(f)


def _expand_intents(intents):
    expanded = set()
    for intent in intents:
        intent = str(intent)
        expanded.add(intent)
        expanded.update(INTENT_ALIASES.get(intent, set()))
    return expanded


def _load_routes(path):
    routes = {}
    root = ET.parse(str(path)).getroot()
    for route in root.findall("route"):
        route_id = str(route.get("id", ""))
        routes[route_id] = {
            "route_id": route_id,
            "town": route.get("town", ""),
            "category": route.get("category", ""),
            "route_file": _repo_rel(path),
        }
    return routes


def _load_annotations(path):
    data = _load_yaml(path) if Path(path).suffix.lower() in (".yaml", ".yml") else _load_json(path)
    annotations = []
    for town_block in data.get("available_scenarios", []):
        for town, scenario_list in town_block.items():
            for item in scenario_list:
                event_cfgs = item.get("available_event_configurations", [])
                event_cfg = event_cfgs[0] if event_cfgs else {}
                annotations.append({
                    "town": town,
                    "scenario_id": str(item.get("scenario_id", item.get("route_id", ""))),
                    "route_id": str(item.get("route_id", item.get("scenario_id", ""))),
                    "scenario_type": str(item.get("scenario_type", "")),
                    "config_path": item.get("config_path"),
                    "trigger": event_cfg.get("trigger", item.get("trigger", {})),
                    "expected": event_cfg.get("expected", item.get("expected", {})),
                })
    return annotations


def _audio_sort_key(path):
    match = re.search(r"record_(\d+)", Path(path).name)
    if match:
        return int(match.group(1))
    return Path(path).name


def _load_voice_results(results_dir):
    records = []
    for json_path in sorted(Path(results_dir).glob("*.json"), key=_audio_sort_key):
        payload = _load_json(json_path)
        wav_path = json_path.with_suffix(".wav")
        if not wav_path.exists():
            raw_audio = str(payload.get("audio_path", "")).replace("\\", "/")
            candidate = Path(raw_audio)
            if candidate.exists():
                wav_path = candidate

        intents = payload.get("all_intents") or []
        if not intents and payload.get("primary_intent"):
            intents = [payload["primary_intent"]]

        speed_values = payload.get("speed_values") or []
        target_speed = None
        if speed_values:
            try:
                target_speed = float(speed_values[0].get("value"))
            except (TypeError, ValueError):
                target_speed = None

        records.append({
            "audio_id": json_path.stem,
            "json_path": _repo_rel(json_path),
            "audio_path": _repo_rel(wav_path),
            "input_text": payload.get("input_text", ""),
            "normalized_text": payload.get("normalized_text", ""),
            "instruction": payload.get("instruction", ""),
            "primary_intent": payload.get("primary_intent", ""),
            "recognized_intents": [str(item) for item in intents],
            "target_speed_kmh": target_speed,
            "confidence": payload.get("confidence"),
            "raw_audio_path": payload.get("audio_path"),
        })
    return records


def _action_intents(action_id, action_cfg):
    action_id_lower = action_id.lower()
    intents = set()
    for key, values in ACTION_WINDOW_HINTS.items():
        if key in action_id_lower:
            intents.update(values)

    direction = action_cfg.get("direction")
    if direction == "left":
        intents.update({"TURN_LEFT", "LEFT_TURN"})
    elif direction == "right":
        intents.update({"TURN_RIGHT", "RIGHT_TURN"})

    target_speed = action_cfg.get("target_speed_kmh")
    if target_speed is not None:
        try:
            speed = float(target_speed)
        except (TypeError, ValueError):
            speed = None
        if speed is not None and speed >= 70.0:
            intents.update({"GO_STRAIGHT", "KEEP_LANE", "ACCELERATE", "ACCELERATE_80"})
        elif speed is not None and speed <= 35.0 and not ({"TURN_LEFT", "TURN_RIGHT"} & intents):
            intents.update({"SLOW_DOWN", "SLOW_DOWN_30"})

    return intents


def _build_event_candidates(annotations, routes, lead_distance_m):
    candidates = []
    for ann in annotations:
        route = routes.get(ann["route_id"], {})
        expected_intents = set(str(item) for item in ann.get("expected", {}).get("intents", []))
        trigger = dict(ann.get("trigger") or {})
        candidates.append({
            "route_id": ann["route_id"],
            "scenario_id": ann["scenario_id"],
            "town": ann["town"],
            "scenario_type": ann["scenario_type"],
            "event_id": ann["scenario_id"],
            "event_source": "scenario_annotation",
            "event_intents": sorted(expected_intents),
            "trigger": trigger,
            "expected": dict(ann.get("expected") or {}),
            "route_file": route.get("route_file"),
            "config_path": ann.get("config_path"),
            "score_bias": 0,
        })

        config_path = ann.get("config_path")
        if not config_path or not Path(config_path).exists():
            continue

        cfg = _load_yaml(config_path)
        for action_id, action_cfg in (cfg.get("action_windows") or {}).items():
            if action_id == "auto_from_route" or not isinstance(action_cfg, dict):
                continue
            if action_cfg.get("enabled", True) is False:
                continue
            if "progress_start_m" not in action_cfg:
                continue

            progress_start = float(action_cfg.get("progress_start_m", 0.0))
            trigger_distance = max(0.0, progress_start - float(lead_distance_m))
            event_intents = _action_intents(action_id, action_cfg)
            expected = {
                "intents": sorted(event_intents),
                "no_collision": True,
            }
            if "target_speed_kmh" in action_cfg:
                expected["target_speed_kmh"] = float(action_cfg["target_speed_kmh"])
            if "min_reached_speed_kmh" in action_cfg:
                expected["min_reached_speed_kmh"] = float(action_cfg["min_reached_speed_kmh"])

            candidates.append({
                "route_id": ann["route_id"],
                "scenario_id": ann["scenario_id"],
                "town": ann["town"],
                "scenario_type": ann["scenario_type"],
                "event_id": action_id,
                "event_source": "scenario_action_window",
                "event_intents": sorted(event_intents),
                "trigger": {
                    "type": "route_distance",
                    "distance_m": round(trigger_distance, 3),
                },
                "expected": expected,
                "route_file": route.get("route_file"),
                "config_path": ann.get("config_path"),
                "action_progress_start_m": progress_start,
                "voice_lead_distance_m": float(lead_distance_m),
                "score_bias": 2,
            })
    return candidates


def _score_match(record, candidate, prefer_route_id=None):
    raw_record_intents = set(record["recognized_intents"])
    raw_candidate_intents = set(candidate.get("event_intents", []))
    exact_overlap = raw_record_intents & raw_candidate_intents

    expanded_record_intents = _expand_intents(record["recognized_intents"])
    expanded_candidate_intents = _expand_intents(candidate.get("event_intents", []))
    alias_overlap = expanded_record_intents & expanded_candidate_intents
    if not exact_overlap and not alias_overlap:
        return 0

    score = 30 * len(exact_overlap) + 5 * len(alias_overlap) + int(candidate.get("score_bias", 0))
    target_speed = record.get("target_speed_kmh")
    expected = candidate.get("expected") or {}
    expected_speed = expected.get("target_speed_kmh") or expected.get("min_reached_speed_kmh")
    if target_speed is not None and expected_speed is not None:
        if abs(float(target_speed) - float(expected_speed)) <= 8.0:
            score += 20
        else:
            score -= 8

    if record.get("primary_intent") in raw_candidate_intents:
        score += 12
    elif record.get("primary_intent") in expanded_candidate_intents:
        score += 3

    if prefer_route_id and candidate.get("route_id") == prefer_route_id:
        score += 6

    return score


def _expected_for_record(record, candidate):
    expected = dict(candidate.get("expected") or {})
    if record["recognized_intents"]:
        expected["recognized_intents"] = list(record["recognized_intents"])
    if record.get("target_speed_kmh") is not None:
        expected["target_speed_kmh"] = float(record["target_speed_kmh"])
    return expected


def _match_records(records, candidates, prefer_route_id=None, prefer_route_hard=True):
    matches = []
    used_audio_ids = set()
    for record in records:
        candidate_pool = candidates
        if prefer_route_id and prefer_route_hard:
            preferred = [item for item in candidates if item.get("route_id") == prefer_route_id]
            if preferred:
                candidate_pool = preferred

        scored = []
        for candidate in candidate_pool:
            score = _score_match(record, candidate, prefer_route_id=prefer_route_id)
            if score > 0:
                scored.append((score, candidate))
        scored.sort(key=lambda item: item[0], reverse=True)

        if not scored:
            matches.append({
                "audio_id": record["audio_id"],
                "status": "unmatched",
                "voice": record,
                "reason": "no route event has overlapping intent",
            })
            continue

        score, candidate = scored[0]
        trigger = dict(candidate.get("trigger") or {})
        trigger["input_mode"] = "wav"
        trigger["audio_path"] = record["audio_path"]

        matches.append({
            "audio_id": record["audio_id"],
            "status": "matched",
            "match_score": score,
            "route_id": candidate["route_id"],
            "scenario_id": candidate["scenario_id"],
            "town": candidate["town"],
            "scenario_type": candidate["scenario_type"],
            "event_id": candidate["event_id"],
            "event_source": candidate["event_source"],
            "route_file": candidate.get("route_file"),
            "scenario_config": candidate.get("config_path"),
            "trigger": trigger,
            "expected": _expected_for_record(record, candidate),
            "voice": record,
        })
        used_audio_ids.add(record["audio_id"])
    return matches


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Match results/*.wav voice commands to route/scenario events."
    )
    parser.add_argument("--results-dir", default=str(DEFAULT_RESULTS_DIR))
    parser.add_argument("--routes", default=str(DEFAULT_ROUTES))
    parser.add_argument("--scenarios", default=str(DEFAULT_SCENARIOS))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument(
        "--prefer-route",
        default=DEFAULT_PREFER_ROUTE,
        help=(
            "Route id used as tie-breaker when multiple routes contain the same action. "
            "Use an empty string to disable."
        ),
    )
    parser.add_argument(
        "--prefer-route-soft",
        action="store_true",
        help="Use preferred route as a score bonus only, instead of restricting matches to that route first.",
    )
    parser.add_argument(
        "--lead-distance-m",
        type=float,
        default=40.0,
        help="Trigger voice this many meters before an action_window starts.",
    )
    args = parser.parse_args(argv)

    routes = _load_routes(Path(args.routes))
    annotations = _load_annotations(Path(args.scenarios))
    records = _load_voice_results(Path(args.results_dir))
    candidates = _build_event_candidates(annotations, routes, args.lead_distance_m)
    prefer_route = args.prefer_route or None
    matches = _match_records(
        records,
        candidates,
        prefer_route_id=prefer_route,
        prefer_route_hard=not args.prefer_route_soft,
    )

    output = {
        "generated_by": "carla_eval/tools/match_route_audio.py",
        "results_dir": _repo_rel(args.results_dir),
        "routes_file": _repo_rel(args.routes),
        "scenario_annotations_file": _repo_rel(args.scenarios),
        "prefer_route": prefer_route,
        "prefer_route_hard": not args.prefer_route_soft,
        "voice_lead_distance_m": float(args.lead_distance_m),
        "matches": matches,
    }

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(output, f, allow_unicode=True, sort_keys=False)

    matched_count = sum(1 for item in matches if item["status"] == "matched")
    print(
        "[DONE] matched "
        f"{matched_count}/{len(matches)} voice records -> {output_path.as_posix()}"
    )
    for item in matches:
        if item["status"] != "matched":
            print(f"  - {item['audio_id']}: unmatched")
            continue
        print(
            "  - "
            f"{item['audio_id']} -> {item['route_id']}::{item['event_id']} "
            f"at {item['trigger'].get('distance_m', item['trigger'].get('value'))}m"
        )
    return 0 if matched_count == len(matches) else 1


if __name__ == "__main__":
    sys.exit(main())
