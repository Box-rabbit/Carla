import json
from pathlib import Path

import yaml


def load_yaml_with_path(path):
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["__config_path__"] = str(path.resolve())
    return cfg


def resolve_config_relative_path(cfg, relative_path):
    if not relative_path:
        return None

    path = Path(relative_path)
    if path.is_absolute():
        return path

    if path.exists():
        return path.resolve()

    config_path = cfg.get("__config_path__")
    if config_path:
        base_dir = Path(config_path).resolve().parent
        candidate = (base_dir / relative_path).resolve()
        if candidate.exists():
            return candidate

        parents = Path(config_path).resolve().parents
        if len(parents) >= 4:
            repo_root = parents[3]
            candidate = (repo_root / relative_path).resolve()
            if candidate.exists():
                return candidate

    return path.resolve()


class LMDriveTriggerRuntime:
    def __init__(self, trigger_cfg):
        self.trigger_cfg = trigger_cfg
        self._triggered = False
        self._trigger_payload = None

    @property
    def triggered(self):
        return self._triggered

    @property
    def payload(self):
        return self._trigger_payload

    def evaluate(self, timestamp, route_progress_m, anchor_progress_m=None):
        if self._triggered:
            return False, self._trigger_payload

        trigger = self.trigger_cfg.get("trigger", {})
        trigger_type = trigger.get("type")

        payload = {
            "timestamp": float(timestamp),
            "trigger_type": trigger_type,
            "trigger_config": json.loads(json.dumps(trigger)),
        }

        if trigger_type == "route_distance":
            threshold = float(trigger.get("distance_m", 0.0))
            payload["trigger_distance_m"] = threshold
            payload["route_progress_m"] = float(route_progress_m)
            payload["anchor_progress_m"] = None if anchor_progress_m is None else float(anchor_progress_m)

            if anchor_progress_m is not None:
                distance_to_anchor = max(0.0, float(anchor_progress_m) - float(route_progress_m))
                payload["route_distance_to_anchor_m"] = distance_to_anchor
                should_trigger = distance_to_anchor <= threshold
            else:
                payload["route_distance_to_anchor_m"] = None
                should_trigger = float(route_progress_m) >= threshold
        elif trigger_type == "time":
            threshold = float(trigger.get("value", 0.0))
            payload["trigger_time_s"] = threshold
            should_trigger = float(timestamp) >= threshold
        else:
            payload["error"] = f"unsupported_trigger_type:{trigger_type}"
            should_trigger = False

        if should_trigger:
            self._triggered = True
            self._trigger_payload = payload
            return True, payload

        return False, payload
