"""Runtime helper for route-progress based voice/audio match configs."""

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml


class RouteAudioRuntime:
    """Activate matched voice records when route progress reaches trigger points."""

    def __init__(self, match_config_path: str, scenario_id: str):
        self.match_config_path = Path(match_config_path)
        self.scenario_id = str(scenario_id)
        self._events: List[Dict[str, Any]] = []
        self._triggered_ids = set()
        self._active: Optional[Dict[str, Any]] = None

        if not self.match_config_path.exists():
            raise FileNotFoundError(f"Route-audio match config not found: {self.match_config_path}")

        with self.match_config_path.open("r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}

        for item in cfg.get("matches", []):
            if item.get("status") != "matched":
                continue
            if str(item.get("route_id")) != self.scenario_id and str(item.get("scenario_id")) != self.scenario_id:
                continue

            trigger = item.get("trigger", {})
            if trigger.get("type") != "route_distance":
                continue

            event = dict(item)
            event["_trigger_distance_m"] = float(trigger.get("distance_m", 0.0))
            self._events.append(event)

        self._events.sort(key=lambda e: e["_trigger_distance_m"])

    @property
    def active(self) -> Optional[Dict[str, Any]]:
        return self._active

    @property
    def events(self) -> List[Dict[str, Any]]:
        return list(self._events)

    def update(self, route_progress_m: float, timestamp: float) -> Tuple[Optional[Dict[str, Any]], bool]:
        """Return (active_event, newly_triggered)."""
        newly_triggered = False
        progress = float(route_progress_m)

        for event in self._events:
            audio_id = event.get("audio_id")
            if audio_id in self._triggered_ids:
                continue
            if progress < event["_trigger_distance_m"]:
                break

            self._triggered_ids.add(audio_id)
            self._active = dict(event)
            self._active["voice_trigger_timestamp"] = float(timestamp)
            self._active["voice_trigger_progress_m"] = progress
            newly_triggered = True

        return self._active, newly_triggered
