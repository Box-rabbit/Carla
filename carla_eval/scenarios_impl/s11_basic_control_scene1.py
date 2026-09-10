"""S11 – PDF scene 1 basic-control 5km drive without voice input."""

import math
from typing import Any, Dict, List, Tuple

import carla

from carla_eval.runtime_metrics import RouteTracker
from .base import BaseScenario, compute_steer_to_location


def _wrap_deg(delta: float) -> float:
    return (delta + 180.0) % 360.0 - 180.0


def _window_active(progress_m: float, window: Dict[str, Any]) -> bool:
    return float(window.get("progress_start_m", 0.0)) <= progress_m <= float(
        window.get("progress_end_m", 0.0)
    )


def _yaw_between(a: carla.Location, b: carla.Location) -> float:
    return math.degrees(math.atan2(b.y - a.y, b.x - a.x))


def _route_yaw_at(route_tracker: RouteTracker, progress_m: float, delta_m: float = 8.0) -> float:
    p0 = max(0.0, progress_m - delta_m)
    p1 = min(route_tracker.route_total_length_m, progress_m + delta_m)
    a = route_tracker.point_at_progress(p0)
    b = route_tracker.point_at_progress(p1)
    return _yaw_between(a, b)


def _rate_limit(current: float, desired: float, dt: float, up_rate: float, down_rate: float) -> float:
    if desired >= current:
        return min(desired, current + up_rate * dt)
    return max(desired, current - down_rate * dt)


def _route_progresses(route_points: List[carla.Location]) -> List[float]:
    progresses = [0.0]
    for prev, cur in zip(route_points, route_points[1:]):
        progresses.append(
            progresses[-1]
            + math.hypot(cur.x - prev.x, cur.y - prev.y)
        )
    return progresses


def _nearest_progress_index(progresses: List[float], target: float) -> int:
    if not progresses:
        return 0
    lo, hi = 0, len(progresses) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if progresses[mid] < target:
            lo = mid + 1
        else:
            hi = mid
    if lo > 0 and abs(progresses[lo - 1] - target) < abs(progresses[lo] - target):
        return lo - 1
    return lo


def _compute_comfort_speed_control(
    speed_kmh: float,
    target_kmh: float,
    steer: float = 0.0,
) -> Tuple[float, float]:
    """S11 uses a less abrupt speed controller than emergency/obstacle scenarios."""
    error = target_kmh - speed_kmh
    if error > 12.0:
        throttle, brake = 1.00, 0.0
    elif error > 7.0:
        throttle, brake = 0.85, 0.0
    elif error > 3.0:
        throttle, brake = 0.60, 0.0
    elif error > 0.8:
        throttle, brake = 0.35, 0.0
    elif error < -14.0:
        throttle, brake = 0.0, 0.22
    elif error < -8.0:
        throttle, brake = 0.0, 0.12
    elif error < -3.0:
        throttle, brake = 0.0, 0.06
    else:
        throttle, brake = 0.18, 0.0

    if abs(steer) > 0.35:
        throttle = min(throttle, 0.45)
    return throttle, brake


def _derive_first_turn_window(route_tracker: RouteTracker, cfg: Dict[str, Any], direction: str) -> Dict[str, Any]:
    auto_cfg = cfg.get("action_windows", {}).get("auto_from_route", {})
    min_progress = float(auto_cfg.get("min_turn_progress_m", 180.0))
    max_progress = min(
        float(auto_cfg.get("max_turn_progress_m", 2200.0)),
        route_tracker.route_total_length_m - 80.0,
    )
    scan_step = float(auto_cfg.get("turn_scan_step_m", 8.0))
    span = float(auto_cfg.get("turn_scan_span_m", 50.0))
    threshold = float(auto_cfg.get("turn_threshold_deg", 28.0))
    half_width = float(auto_cfg.get("turn_window_half_width_m", 85.0))

    best = None
    progress = min_progress
    while progress <= max_progress:
        yaw_before = _route_yaw_at(route_tracker, progress - 0.5 * span)
        yaw_after = _route_yaw_at(route_tracker, progress + 0.5 * span)
        delta = _wrap_deg(yaw_after - yaw_before)
        matches = delta <= -threshold if direction == "right" else delta >= threshold
        if matches and (best is None or abs(delta) > abs(best["delta"])):
            best = {"progress": progress, "delta": delta}
        progress += scan_step

    if best is None:
        fallback = cfg.get("action_windows", {}).get(f"{direction}_turn_1", {})
        return {**fallback, "source": "fallback"}

    return {
        "enabled": True,
        "progress_start_m": max(0.0, best["progress"] - half_width),
        "progress_end_m": best["progress"] + half_width,
        "direction": direction,
        "expected_heading_change_deg_min": float(
            cfg.get("action_windows", {}).get(f"{direction}_turn_1", {}).get(
                "expected_heading_change_deg_min", 30.0
            )
        ),
        "source": "auto_route_heading",
        "detected_progress_m": best["progress"],
        "detected_heading_delta_deg": best["delta"],
    }


def _derive_route_turn_windows(route_tracker: RouteTracker, cfg: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    auto_cfg = cfg.get("action_windows", {}).get("auto_from_route", {})
    points = list(getattr(route_tracker, "route_points", []) or [])
    if len(points) < 3:
        return {}

    progresses = _route_progresses(points)
    min_progress = float(auto_cfg.get("min_turn_progress_m", 80.0))
    max_progress = min(
        float(auto_cfg.get("max_turn_progress_m", 5000.0)),
        progresses[-1] - 40.0,
    )
    scan_step = max(2.0, float(auto_cfg.get("turn_scan_step_m", 8.0)))
    span = float(auto_cfg.get("turn_scan_span_m", 50.0))
    threshold = float(auto_cfg.get("turn_threshold_deg", 35.0))
    min_gap = float(auto_cfg.get("turn_min_gap_m", 70.0))
    half_width = float(auto_cfg.get("turn_window_half_width_m", 45.0))
    left_speed = float(auto_cfg.get("left_turn_target_speed_kmh", 30.0))
    right_speed = float(auto_cfg.get("right_turn_target_speed_kmh", 28.0))

    detected = []
    last_progress = -1e9
    progress = min_progress
    while progress <= max_progress:
        if progress - last_progress < min_gap:
            progress += scan_step
            continue

        before = points[_nearest_progress_index(progresses, progress - 0.5 * span)]
        center = points[_nearest_progress_index(progresses, progress)]
        after = points[_nearest_progress_index(progresses, progress + 0.5 * span)]
        yaw_before = _yaw_between(before, center)
        yaw_after = _yaw_between(center, after)
        delta = _wrap_deg(yaw_after - yaw_before)
        if abs(delta) >= threshold:
            direction = "left" if delta > 0.0 else "right"
            detected.append(
                {
                    "direction": direction,
                    "progress": progress,
                    "delta": delta,
                }
            )
            last_progress = progress

        progress += scan_step

    windows: Dict[str, Dict[str, Any]] = {}
    counts = {"left": 0, "right": 0}
    direction_sequence = [
        str(item).lower()
        for item in auto_cfg.get("turn_direction_sequence", [])
        if str(item).lower() in {"left", "right"}
    ]
    for idx, item in enumerate(detected):
        direction = (
            direction_sequence[idx]
            if idx < len(direction_sequence)
            else item["direction"]
        )
        counts[direction] += 1
        key = f"{direction}_turn_{counts[direction]}"
        windows[key] = {
            "enabled": True,
            "progress_start_m": max(0.0, item["progress"] - half_width),
            "progress_end_m": item["progress"] + half_width,
            "direction": direction,
            "target_speed_kmh": left_speed if direction == "left" else right_speed,
            "expected_heading_change_deg_min": float(
                auto_cfg.get("expected_heading_change_deg_min", threshold)
            ),
            "source": "auto_route_heading_all",
            "detected_progress_m": item["progress"],
            "detected_heading_delta_deg": item["delta"],
        }
    return windows


class BasicControlScene1(BaseScenario):
    """
    PDF scene 1:
    - sunny daytime urban main road
    - no dynamic interference
    - 5km continuous driving
    - right turn, left turn, both lane changes, speed changes, final parking
    """

    def spawn_actors(self, world, ego, cfg) -> List[carla.Actor]:
        return []

    def initial_state(self, ego, actors, cfg) -> Dict[str, Any]:
        windows = cfg.get("action_windows", {})
        ctrl = cfg.get("controller", {})
        initial_target = float(
            ctrl.get("initial_target_speed_kmh", ctrl.get("cruise_target_speed_kmh", 45.0))
        )
        return {
            "windows": dict(windows),
            "windows_initialized": False,
            "cruise_target_speed_kmh": float(ctrl.get("cruise_target_speed_kmh", 45.0)),
            "desired_speed_kmh": initial_target,
            "target_speed_kmh": initial_target,
            "speed_control_target_kmh": initial_target,
            "target_lateral_offset_m": 0.0,
            "active_window": "cruise",
            "turn_windows": [],
            "turn_entry_yaw": {},
            "turn_heading_delta": {},
            "turn_completed": {},
            "complete_all_route_turns": False,
            "right_turn_completed_count": 0,
            "right_turn_total_count": 0,
            "left_turn_completed_count": 0,
            "left_turn_total_count": 0,
            "speed_80_hold_s": 0.0,
            "speed_60_hold_s": 0.0,
            "speed_30_hold_s": 0.0,
            "lane_change_hold_s": 0.0,
            "lane_change_right_hold_s": 0.0,
            "parking_hold_s": 0.0,
            "reach_target_speed_80": False,
            "reach_target_speed_60": False,
            "complete_right_turn": False,
            "complete_left_turn": False,
            "complete_lane_change_left": False,
            "complete_lane_change_right": False,
            "reach_target_speed_30": False,
            "final_parking": False,
            "right_turn_entry_yaw": None,
            "right_turn_min_delta_deg": 0.0,
            "left_turn_entry_yaw": None,
            "left_turn_max_delta_deg": 0.0,
            "right_turn_window_source": "static",
            "success": False,
        }

    def _ensure_windows(self, state, obs, cfg) -> None:
        if state["windows_initialized"]:
            return
        route_tracker = obs.get("route_tracker")
        if route_tracker is None:
            return

        windows = state["windows"]
        if windows.get("auto_from_route", {}).get("enabled", False):
            turn_windows = _derive_route_turn_windows(route_tracker, cfg)
            if not turn_windows:
                right_window = _derive_first_turn_window(route_tracker, cfg, "right")
                turn_windows["right_turn_1"] = right_window
            for key in [
                k for k in list(windows)
                if k.startswith("left_turn_") or k.startswith("right_turn_")
            ]:
                windows.pop(key, None)
            windows.update(turn_windows)
            state["right_turn_window_source"] = windows.get("right_turn_1", {}).get("source", "static")

            lane_cfg = windows.get("lane_change_left", {})
            slow_cfg = windows.get("slow_to_30", {})
            first_turn = min(
                turn_windows.values(),
                key=lambda item: float(item.get("progress_start_m", 1e9)),
                default={},
            )
            right_end = float(first_turn.get("progress_end_m", 600.0))
            if lane_cfg.get("auto_after_right_turn", True):
                start = right_end + float(lane_cfg.get("start_offset_after_turn_m", 140.0))
                windows["lane_change_left"] = {
                    **lane_cfg,
                    "enabled": True,
                    "progress_start_m": start,
                    "progress_end_m": start + float(lane_cfg.get("duration_m", 220.0)),
                }
            if slow_cfg.get("auto_after_lane_change", True):
                lane_end = float(windows["lane_change_left"].get("progress_end_m", right_end + 360.0))
                start = lane_end + float(slow_cfg.get("start_offset_after_lane_change_m", 220.0))
                windows["slow_to_30"] = {
                    **slow_cfg,
                    "enabled": True,
                    "progress_start_m": start,
                    "progress_end_m": start + float(slow_cfg.get("duration_m", 260.0)),
                }

        turn_items = [
            (key, value)
            for key, value in windows.items()
            if (
                isinstance(value, dict)
                and value.get("enabled", False)
                and (key.startswith("left_turn_") or key.startswith("right_turn_"))
            )
        ]
        turn_items.sort(key=lambda item: float(item[1].get("progress_start_m", 0.0)))
        state["turn_windows"] = [
            {
                "key": key,
                "enabled": True,
                "progress_start_m": float(value.get("progress_start_m", 0.0)),
                "progress_end_m": float(value.get("progress_end_m", 0.0)),
                "direction": value.get("direction", "left" if key.startswith("left") else "right"),
                "source": value.get("source", "static"),
            }
            for key, value in turn_items
        ]
        state["turn_completed"] = {key: False for key, _ in turn_items}
        state["right_turn_total_count"] = sum(
            1 for _, value in turn_items if value.get("direction") == "right"
        )
        state["left_turn_total_count"] = sum(
            1 for _, value in turn_items if value.get("direction") == "left"
        )
        turn_summary = ", ".join(
            f"{item['key']}:{item['direction']}@{item['progress_start_m']:.0f}-{item['progress_end_m']:.0f}m"
            for item in state["turn_windows"]
        )
        print(f"[S11] detected route turn windows: {turn_summary or 'none'}")
        state["windows_initialized"] = True

    def update_state(self, ego, actors, state, obs, dt, cfg) -> Dict[str, Any]:
        self._ensure_windows(state, obs, cfg)

        windows = state["windows"]
        progress = float(obs["route_metrics"]["route_progress_m"])
        speed = float(obs["speed_kmh"])
        yaw = float(ego.get_transform().rotation.yaw)
        lateral_offset = float(obs["route_metrics"].get("lateral_offset_from_route_m", 0.0))

        accel = windows.get("accelerate_to_60", windows.get("accelerate_to_80", {}))
        lane = windows.get("lane_change_left", {})
        lane_right = windows.get("lane_change_right", {})
        slow = windows.get("slow_to_30", {})
        parking = windows.get("final_parking", {})
        active_turn_candidates = []
        for turn_item in state.get("turn_windows", []):
            window = windows.get(turn_item["key"], turn_item)
            if window.get("enabled", False) and _window_active(progress, window):
                active_turn_candidates.append(
                    (float(window.get("progress_start_m", 0.0)), turn_item["key"], window)
                )
        active_turn = None
        if active_turn_candidates:
            _, turn_key, turn_window = max(active_turn_candidates, key=lambda item: item[0])
            active_turn = (turn_key, turn_window)

        lane_target_offset = float(lane.get("target_lateral_offset_m", -3.5))
        lane_right_target_offset = float(lane_right.get("target_lateral_offset_m", 0.0))
        active_lane = None
        if lane_right.get("enabled", False) and _window_active(progress, lane_right):
            active_lane = ("lane_change_right", lane_right)
        elif lane.get("enabled", False) and _window_active(progress, lane):
            active_lane = ("lane_change_left", lane)
        state["target_lateral_offset_m"] = 0.0
        if active_lane is not None:
            state["target_lateral_offset_m"] = float(
                active_lane[1].get(
                    "target_lateral_offset_m",
                    lane_right_target_offset if active_lane[0] == "lane_change_right" else lane_target_offset,
                )
            )
        elif state["complete_lane_change_left"] and not state["complete_lane_change_right"]:
            state["target_lateral_offset_m"] = lane_target_offset
        desired_speed = state["cruise_target_speed_kmh"]
        control_speed = desired_speed

        accel_active = accel.get("enabled", False) and (
            _window_active(progress, accel)
            or (accel.get("hold_until_reached", False) and not state["reach_target_speed_60"])
        )

        if active_turn is not None:
            turn_key, turn_window = active_turn
            direction = turn_window.get(
                "direction", "left" if turn_key.startswith("left") else "right"
            )
            state["active_window"] = turn_key
            desired_speed = min(
                state["cruise_target_speed_kmh"],
                float(turn_window.get("target_speed_kmh", 30.0)),
            )
            control_speed = desired_speed
            if turn_key not in state["turn_entry_yaw"]:
                state["turn_entry_yaw"][turn_key] = yaw
                state["turn_heading_delta"][turn_key] = 0.0
            delta = _wrap_deg(yaw - state["turn_entry_yaw"][turn_key])
            if direction == "right":
                state["turn_heading_delta"][turn_key] = min(
                    float(state["turn_heading_delta"].get(turn_key, 0.0)), delta
                )
                completed = state["turn_heading_delta"][turn_key] <= -float(
                    turn_window.get("expected_heading_change_deg_min", 30.0)
                )
                state["right_turn_min_delta_deg"] = min(
                    state["right_turn_min_delta_deg"],
                    state["turn_heading_delta"][turn_key],
                )
            else:
                state["turn_heading_delta"][turn_key] = max(
                    float(state["turn_heading_delta"].get(turn_key, 0.0)), delta
                )
                completed = state["turn_heading_delta"][turn_key] >= float(
                    turn_window.get("expected_heading_change_deg_min", 30.0)
                )
                state["left_turn_max_delta_deg"] = max(
                    state["left_turn_max_delta_deg"],
                    state["turn_heading_delta"][turn_key],
                )
            if completed:
                state["turn_completed"][turn_key] = True

        elif accel_active:
            state["active_window"] = "accelerate_to_60"
            desired_speed = float(accel.get("target_speed_kmh", 60.0))
            control_speed = float(accel.get("control_target_speed_kmh", desired_speed))
            tol = float(accel.get("tolerance_kmh", 6.0))
            min_reached_speed = float(accel.get("min_reached_speed_kmh", desired_speed - tol))
            speed_reached = speed >= min_reached_speed or abs(speed - desired_speed) <= tol
            state["speed_60_hold_s"] = state["speed_60_hold_s"] + dt if speed_reached else 0.0
            if state["speed_60_hold_s"] >= float(accel.get("required_hold_seconds", 2.0)):
                state["reach_target_speed_60"] = True

        elif active_lane is not None:
            lane_key, lane_window = active_lane
            state["active_window"] = lane_key
            desired_speed = float(lane_window.get("target_speed_kmh", 45.0))
            control_speed = desired_speed
            completion_progress = float(lane_window.get("progress_end_m", progress))
            progress_margin = float(lane_window.get("completion_progress_margin_m", 25.0))
            lane_centered = abs(lateral_offset - state["target_lateral_offset_m"]) <= float(
                lane_window.get("completion_lateral_offset_m", 0.8)
            )
            hold_key = (
                "lane_change_right_hold_s"
                if lane_key == "lane_change_right"
                else "lane_change_hold_s"
            )
            state[hold_key] = state[hold_key] + dt if (
                progress >= completion_progress - progress_margin and lane_centered
            ) else 0.0
            if state[hold_key] >= float(lane_window.get("required_hold_seconds", 1.0)):
                state[
                    "complete_lane_change_right"
                    if lane_key == "lane_change_right"
                    else "complete_lane_change_left"
                ] = True

        elif slow.get("enabled", False) and _window_active(progress, slow):
            state["active_window"] = "slow_to_30"
            desired_speed = float(slow.get("target_speed_kmh", 30.0))
            control_speed = desired_speed
            if state["complete_lane_change_left"]:
                state["target_lateral_offset_m"] = lane_target_offset
            tol = float(slow.get("tolerance_kmh", 5.0))
            state["speed_30_hold_s"] = state["speed_30_hold_s"] + dt if abs(speed - desired_speed) <= tol else 0.0
            if state["speed_30_hold_s"] >= float(slow.get("required_hold_seconds", 2.0)):
                state["reach_target_speed_30"] = True

        elif parking.get("enabled", False) and _window_active(progress, parking):
            state["active_window"] = "final_parking"
            desired_speed = float(parking.get("target_speed_kmh", 0.0))
            control_speed = desired_speed
            stop_tolerance = float(parking.get("stop_speed_tolerance_kmh", 1.0))
            state["parking_hold_s"] = state["parking_hold_s"] + dt if speed <= stop_tolerance else 0.0
            if state["parking_hold_s"] >= float(parking.get("required_hold_seconds", 3.0)):
                state["final_parking"] = True

        else:
            state["active_window"] = "cruise"
            desired_speed = state["cruise_target_speed_kmh"]
            control_speed = desired_speed

        state["right_turn_completed_count"] = sum(
            1
            for item in state.get("turn_windows", [])
            if item.get("direction") == "right" and state["turn_completed"].get(item["key"])
        )
        state["left_turn_completed_count"] = sum(
            1
            for item in state.get("turn_windows", [])
            if item.get("direction") == "left" and state["turn_completed"].get(item["key"])
        )
        state["complete_right_turn"] = state["right_turn_completed_count"] > 0
        state["complete_left_turn"] = state["left_turn_completed_count"] > 0
        state["complete_all_route_turns"] = bool(state.get("turn_windows")) and all(
            state["turn_completed"].get(item["key"], False)
            for item in state.get("turn_windows", [])
        )

        ramp_cfg = cfg.get("controller", {}).get("speed_target_ramp", {})
        state["desired_speed_kmh"] = desired_speed
        state["speed_control_target_kmh"] = control_speed
        state["target_speed_kmh"] = _rate_limit(
            float(state["target_speed_kmh"]),
            desired_speed,
            dt,
            float(ramp_cfg.get("accelerate_rate_kmh_per_s", 10.0)),
            float(ramp_cfg.get("decelerate_rate_kmh_per_s", 4.5)),
        )

        long_cfg = cfg.get("success_criteria", {}).get("long_route", {})
        target_progress = float(long_cfg.get("target_progress_m", 5000.0))
        max_progress = float(obs["route_metrics"].get("max_route_progress_m", progress))
        if (
            max_progress >= target_progress
            and state["reach_target_speed_60"]
            and state["complete_right_turn"]
            and state["complete_left_turn"]
            and (
                not cfg.get("success_criteria", {}).get("require_all_detected_turns", False)
                or state["complete_all_route_turns"]
            )
            and state["complete_lane_change_left"]
            and state["complete_lane_change_right"]
            and state["reach_target_speed_30"]
            and state["final_parking"]
        ):
            state["success"] = True

        return state

    def compute_control(self, ego, actors, state, obs, cfg) -> Tuple[float, float, float]:
        route_tracker: RouteTracker = obs.get("route_tracker")
        progress = float(obs["route_metrics"]["route_progress_m"])
        lateral_offset = float(obs["route_metrics"].get("lateral_offset_from_route_m", 0.0))
        ctrl = cfg.get("controller", {})
        smoothing_cfg = ctrl.get("target_point_smoothing", {})
        active = state.get("active_window", "cruise")

        if active.startswith("right_turn_") or active.startswith("left_turn_"):
            lookahead = float(ctrl.get("turn_lookahead_m", 7.0))
            gain = float(ctrl.get("turn_steer_gain", 1.55))
            max_steer = float(ctrl.get("turn_max_steer", ctrl.get("max_steer", 0.46)))
            window_radius = float(smoothing_cfg.get("turn_window_radius_m", 5.0))
            sigma = float(smoothing_cfg.get("turn_sigma_m", 2.5))
        elif active in {"lane_change_left", "lane_change_right"}:
            lookahead = float(ctrl.get("lane_change_lookahead_m", 10.0))
            gain = float(ctrl.get("lane_change_steer_gain", 1.25))
            max_steer = float(ctrl.get("lane_change_max_steer", 0.38))
            window_radius = float(smoothing_cfg.get("window_radius_m", 8.0))
            sigma = float(smoothing_cfg.get("sigma_m", 4.0))
        else:
            lookahead = float(ctrl.get("lookahead_m", 12.0))
            gain = float(ctrl.get("steer_gain", 1.25))
            max_steer = float(ctrl.get("max_steer", 0.46))
            window_radius = float(smoothing_cfg.get("window_radius_m", 8.0))
            sigma = float(smoothing_cfg.get("sigma_m", 4.0))

        if progress < 8.0 and float(obs["speed_kmh"]) < 15.0:
            max_steer = min(max_steer, 0.20)
            gain = min(gain, 0.90)

        if route_tracker is not None:
            target_loc = route_tracker.point_at_progress_smoothed(
                progress + lookahead,
                lateral_offset_m=float(state.get("target_lateral_offset_m", 0.0)),
                window_radius_m=window_radius,
                sample_step_m=float(smoothing_cfg.get("sample_step_m", 2.0)),
                sigma_m=sigma,
                tangent_delta_m=float(smoothing_cfg.get("tangent_delta_m", 2.0)),
            )
        else:
            target_loc = ego.get_location()

        steer = compute_steer_to_location(ego, target_loc, gain=gain, max_steer=max_steer)

        throttle, brake = _compute_comfort_speed_control(
            speed_kmh=float(obs["speed_kmh"]),
            target_kmh=float(state.get("speed_control_target_kmh", state["target_speed_kmh"])),
            steer=steer,
        )

        return throttle, brake, steer

    def is_success(self, state, obs, cfg) -> bool:
        return bool(state["success"])

    def extra_record(self, ego, actors, state, obs, cfg) -> Dict[str, Any]:
        return {
            "target_speed_kmh": state["target_speed_kmh"],
            "desired_speed_kmh": state["desired_speed_kmh"],
            "speed_control_target_kmh": state["speed_control_target_kmh"],
            "target_lateral_offset_m": state["target_lateral_offset_m"],
            "active_window": state["active_window"],
            "speed_80_hold_time": state["speed_80_hold_s"],
            "speed_60_hold_time": state["speed_60_hold_s"],
            "speed_30_hold_time": state["speed_30_hold_s"],
            "lane_change_hold_time": state["lane_change_hold_s"],
            "lane_change_right_hold_time": state["lane_change_right_hold_s"],
            "parking_hold_time": state["parking_hold_s"],
            "reach_target_speed_80": state["reach_target_speed_80"],
            "reach_target_speed_60": state["reach_target_speed_60"],
            "complete_right_turn": state["complete_right_turn"],
            "complete_left_turn": state["complete_left_turn"],
            "complete_all_route_turns": state["complete_all_route_turns"],
            "right_turn_completed_count": state["right_turn_completed_count"],
            "right_turn_total_count": state["right_turn_total_count"],
            "left_turn_completed_count": state["left_turn_completed_count"],
            "left_turn_total_count": state["left_turn_total_count"],
            "turn_completed": dict(state.get("turn_completed", {})),
            "complete_lane_change_left": state["complete_lane_change_left"],
            "complete_lane_change_right": state["complete_lane_change_right"],
            "reach_target_speed_30": state["reach_target_speed_30"],
            "final_parking": state["final_parking"],
            "right_turn_min_delta_deg": state["right_turn_min_delta_deg"],
            "left_turn_max_delta_deg": state["left_turn_max_delta_deg"],
            "right_turn_window_source": state["right_turn_window_source"],
            "task_success": state["success"],
        }
