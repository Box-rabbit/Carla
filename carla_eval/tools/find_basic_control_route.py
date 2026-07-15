"""Find candidate routes for S11 basic-control scene.

The script is read-only: it connects to CARLA, optionally loads a town, scans
spawn points, and reports routes that contain a right turn, a left turn, and a
left-lane-change section.
"""

import argparse
import math
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import carla_eval  # noqa: E402,F401
import carla  # noqa: E402


def _wrap_deg(delta):
    return (delta + 180.0) % 360.0 - 180.0


def _dist(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def _valid_left_lane(wp):
    left = wp.get_left_lane()
    if left is None:
        return False
    if left.lane_type != carla.LaneType.Driving:
        return False
    return wp.lane_id * left.lane_id > 0


def _choice_score(current_wp, candidate_wp, turn_hint):
    yaw_delta = _wrap_deg(candidate_wp.transform.rotation.yaw - current_wp.transform.rotation.yaw)
    same_road_penalty = 0.0 if candidate_wp.road_id == current_wp.road_id else 20.0
    same_lane_penalty = 0.0 if candidate_wp.lane_id == current_wp.lane_id else 5.0

    if turn_hint == "left":
        return -yaw_delta + same_lane_penalty
    if turn_hint == "right":
        return yaw_delta + same_lane_penalty
    if turn_hint == "straight":
        return abs(yaw_delta) + same_road_penalty + same_lane_penalty
    return abs(yaw_delta) + same_road_penalty + same_lane_penalty


def _trace_route(carla_map, start_wp, decisions, step_m, target_length_m, max_steps):
    route = [start_wp]
    progress = [0.0]
    current = start_wp
    decision_index = 0
    last_junction_id = None

    for _ in range(max_steps):
        next_wps = [
            wp for wp in current.next(step_m)
            if wp is not None and wp.lane_type == carla.LaneType.Driving and wp.lane_width >= 2.5
        ]
        if not next_wps:
            break

        turn_hint = None
        if len(next_wps) > 1 and decision_index < len(decisions):
            turn_hint = decisions[decision_index]
            decision_index += 1
        else:
            junction_id = current.junction_id if current.is_junction else None
            if junction_id is not None and junction_id != last_junction_id:
                if decision_index < len(decisions):
                    turn_hint = decisions[decision_index]
                    decision_index += 1
                last_junction_id = junction_id

        if current.is_junction:
            last_junction_id = current.junction_id

        nxt = min(next_wps, key=lambda wp: _choice_score(current, wp, turn_hint))
        route.append(nxt)
        progress.append(progress[-1] + _dist(current.transform.location, nxt.transform.location))
        current = nxt
        if progress[-1] >= target_length_m:
            break

    return route, progress


def _detect_turns(route, progress, min_delta_deg=45.0, window_m=35.0, min_gap_m=80.0):
    turns = []
    last_progress = -1e9

    for idx, wp in enumerate(route):
        p = progress[idx]
        if p - last_progress < min_gap_m:
            continue

        before_idx = idx
        while before_idx > 0 and progress[before_idx] > p - window_m:
            before_idx -= 1
        after_idx = idx
        while after_idx + 1 < len(route) and progress[after_idx] < p + window_m:
            after_idx += 1

        if before_idx == idx or after_idx == idx:
            continue

        before_yaw = route[before_idx].transform.rotation.yaw
        after_yaw = route[after_idx].transform.rotation.yaw
        delta = _wrap_deg(after_yaw - before_yaw)
        if abs(delta) < min_delta_deg:
            continue

        turns.append({
            "kind": "left" if delta > 0 else "right",
            "progress_m": p,
            "delta_deg": delta,
            "x": wp.transform.location.x,
            "y": wp.transform.location.y,
        })
        last_progress = p

    return turns


def _find_left_change_progress(route, progress, min_progress_m, max_progress_m, required_len_m=80.0):
    for idx, wp in enumerate(route):
        p = progress[idx]
        if p < min_progress_m or p > max_progress_m:
            continue
        if not _valid_left_lane(wp):
            continue

        end_progress = p + required_len_m
        ok = True
        j = idx
        while j + 1 < len(route) and progress[j] < end_progress:
            if not _valid_left_lane(route[j]):
                ok = False
                break
            j += 1
        if ok and progress[j] >= end_progress:
            return p
    return None


def _candidate_decision_sets():
    base = ["straight", "straight", "straight", "straight"]
    patterns = []
    for first in ("right", "left"):
        for second in ("left", "right"):
            if first == second:
                continue
            patterns.append([first] + base + [second] + base + [first, "straight", second])
            patterns.append([first, "straight", second, "straight", "straight", first, "straight", second])
    return patterns


def main(argv=None):
    parser = argparse.ArgumentParser(description="Find S11 candidate route with real turns and lane change")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--town", default=None, help="Optional town to load, e.g. Town03 or Town05")
    parser.add_argument("--step-m", type=float, default=3.0)
    parser.add_argument("--target-length-m", type=float, default=5200.0)
    parser.add_argument("--max-candidates", type=int, default=12)
    parser.add_argument("--min-turn-progress-m", type=float, default=120.0)
    parser.add_argument("--lane-change-after-m", type=float, default=120.0)
    args = parser.parse_args(argv)

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    if args.town and not world.get_map().name.endswith(args.town):
        world = client.load_world(args.town)
    carla_map = world.get_map()

    results = []
    for spawn_idx, spawn in enumerate(carla_map.get_spawn_points()):
        start_wp = carla_map.get_waypoint(
            spawn.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        if start_wp is None:
            continue

        for decisions in _candidate_decision_sets():
            route, progress = _trace_route(
                carla_map,
                start_wp,
                decisions,
                step_m=args.step_m,
                target_length_m=args.target_length_m,
                max_steps=int(args.target_length_m / max(args.step_m, 0.5) * 2),
            )
            if not route or progress[-1] < min(2500.0, args.target_length_m * 0.5):
                continue

            turns = [
                t for t in _detect_turns(route, progress)
                if t["progress_m"] >= args.min_turn_progress_m
            ]
            right = next((t for t in turns if t["kind"] == "right"), None)
            left = next((t for t in turns if t["kind"] == "left" and (right is None or t["progress_m"] > right["progress_m"] + 80.0)), None)
            if right is None or left is None:
                continue

            lane_change_progress = _find_left_change_progress(
                route,
                progress,
                min_progress_m=left["progress_m"] + args.lane_change_after_m,
                max_progress_m=min(left["progress_m"] + 1200.0, progress[-1] - 100.0),
            )
            if lane_change_progress is None:
                continue

            score = right["progress_m"] + 0.3 * left["progress_m"] + 0.1 * lane_change_progress
            results.append({
                "score": score,
                "spawn_idx": spawn_idx,
                "spawn": spawn,
                "start_wp": start_wp,
                "decisions": decisions,
                "length_m": progress[-1],
                "right": right,
                "left": left,
                "lane_change_progress_m": lane_change_progress,
                "turns": turns[:5],
            })

    results.sort(key=lambda item: item["score"])

    print(f"[INFO] map={carla_map.name} candidates={len(results)}")
    for rank, item in enumerate(results[:args.max_candidates], start=1):
        spawn = item["spawn"]
        wp = item["start_wp"]
        print(
            f"\n[CANDIDATE {rank}] spawn_idx={item['spawn_idx']} "
            f"length={item['length_m']:.1f}m road/lane={wp.road_id}/{wp.lane_id}"
        )
        print(
            "  spawn_point: "
            f"x={spawn.location.x:.2f} y={spawn.location.y:.2f} "
            f"z={spawn.location.z:.2f} yaw={spawn.rotation.yaw:.1f}"
        )
        print(f"  junction_decisions: {item['decisions']}")
        print(
            "  right_turn: "
            f"progress={item['right']['progress_m']:.1f}m "
            f"delta={item['right']['delta_deg']:.1f}deg "
            f"loc=({item['right']['x']:.1f},{item['right']['y']:.1f})"
        )
        print(
            "  left_turn: "
            f"progress={item['left']['progress_m']:.1f}m "
            f"delta={item['left']['delta_deg']:.1f}deg "
            f"loc=({item['left']['x']:.1f},{item['left']['y']:.1f})"
        )
        print(f"  lane_change_left_progress: {item['lane_change_progress_m']:.1f}m")
        print("  detected_turns:", [
            (t["kind"], round(t["progress_m"], 1), round(t["delta_deg"], 1))
            for t in item["turns"]
        ])

    if not results:
        print("[WARN] no candidate found. Try another --town, e.g. Town03, Town04, Town10HD.")


if __name__ == "__main__":
    main()
