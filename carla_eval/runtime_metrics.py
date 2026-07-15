import heapq
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import carla

_PLANNER_GRAPH_CACHE = {}


def clamp(value, low, high):
    return max(low, min(high, value))


def dot2d(vec, direction):
    return vec.x * direction.x + vec.y * direction.y


def horizontal_distance(a, b):
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def resolve_repo_relative_path(cfg, relative_path):
    if not relative_path:
        return None

    path = Path(relative_path)
    if path.is_absolute() and path.exists():
        return path

    if path.exists():
        return path.resolve()

    config_path = cfg.get("__config_path__")
    if config_path:
        repo_root = Path(config_path).resolve().parents[3]
        candidate = repo_root / relative_path
        if candidate.exists():
            return candidate

    return None


def _location_from_anchor(anchor):
    return carla.Location(
        x=float(anchor.get("x", 0.0)),
        y=float(anchor.get("y", 0.0)),
        z=float(anchor.get("z", 0.5)),
    )


def _yaw_delta_signed_deg(a_deg, b_deg):
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0


def _perpendicular_distance_2d(point, start, end):
    ax, ay = start.x, start.y
    bx, by = end.x, end.y
    px, py = point.x, point.y
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-9:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / denom
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def _rdp_simplify_locations(points, epsilon_m):
    if len(points) <= 2:
        return list(points)

    keep = [False] * len(points)
    keep[0] = True
    keep[-1] = True
    stack = [(0, len(points) - 1)]

    while stack:
        start_idx, end_idx = stack.pop()
        best_dist = 0.0
        best_idx = None
        for idx in range(start_idx + 1, end_idx):
            dist = _perpendicular_distance_2d(points[idx], points[start_idx], points[end_idx])
            if dist > best_dist:
                best_dist = dist
                best_idx = idx
        if best_idx is not None and best_dist > epsilon_m:
            keep[best_idx] = True
            stack.append((start_idx, best_idx))
            stack.append((best_idx, end_idx))

    return [point for point, flag in zip(points, keep) if flag]


def _densify_locations(points, max_segment_length_m):
    if len(points) <= 1:
        return list(points)

    max_segment_length_m = max(float(max_segment_length_m), 1.0)
    dense = [points[0]]
    for start, end in zip(points, points[1:]):
        seg_len = horizontal_distance(start, end)
        steps = max(1, int(math.ceil(seg_len / max_segment_length_m)))
        for step in range(1, steps + 1):
            ratio = step / steps
            dense.append(
                carla.Location(
                    x=start.x + ratio * (end.x - start.x),
                    y=start.y + ratio * (end.y - start.y),
                    z=start.z + ratio * (end.z - start.z),
                )
            )
    return dense


def _build_planner_graph(carla_map, sampling_resolution_m):
    cache_key = (carla_map.name, round(float(sampling_resolution_m), 3))
    if cache_key in _PLANNER_GRAPH_CACHE:
        return _PLANNER_GRAPH_CACHE[cache_key]

    topology = []
    for wp1, wp2 in carla_map.get_topology():
        l1 = wp1.transform.location
        l2 = wp2.transform.location
        x1, y1, z1, x2, y2, z2 = [
            round(value, 0) for value in (l1.x, l1.y, l1.z, l2.x, l2.y, l2.z)
        ]
        seg = {
            "entry": wp1,
            "exit": wp2,
            "entryxyz": (x1, y1, z1),
            "exitxyz": (x2, y2, z2),
            "path": [],
        }
        end_loc = wp2.transform.location
        if wp1.transform.location.distance(end_loc) > sampling_resolution_m:
            next_wps = wp1.next(sampling_resolution_m)
            if next_wps:
                current = next_wps[0]
                while current.transform.location.distance(end_loc) > sampling_resolution_m:
                    seg["path"].append(current)
                    next_wps = current.next(sampling_resolution_m)
                    if not next_wps:
                        break
                    current = next_wps[0]
        else:
            next_wps = wp1.next(sampling_resolution_m)
            if next_wps:
                seg["path"].append(next_wps[0])
        topology.append(seg)

    nodes = {}
    edges = {}
    road_to_edge = {}
    for seg in topology:
        for vertex in (seg["entryxyz"], seg["exitxyz"]):
            if vertex not in nodes:
                nodes[vertex] = len(nodes)
        n1 = nodes[seg["entryxyz"]]
        n2 = nodes[seg["exitxyz"]]
        wp = seg["entry"]
        road_to_edge.setdefault(wp.road_id, {}).setdefault(wp.section_id, {})[wp.lane_id] = (n1, n2)
        edges[(n1, n2)] = {
            "path": [seg["entry"]] + seg["path"] + [seg["exit"]],
            "length": len(seg["path"]) + 1,
        }

    adjacency = {idx: [] for idx in range(len(nodes))}
    vertices = {node_id: xyz for xyz, node_id in nodes.items()}
    for (u, v), edge in edges.items():
        adjacency[u].append((v, edge["length"]))

    payload = {
        "edges": edges,
        "adjacency": adjacency,
        "vertices": vertices,
        "road_to_edge": road_to_edge,
    }
    _PLANNER_GRAPH_CACHE[cache_key] = payload
    return payload


def _planner_localize(carla_map, road_to_edge, location):
    waypoint = carla_map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
    if waypoint is None:
        raise RuntimeError(
            f"Failed to localize planner anchor at ({location.x:.2f}, {location.y:.2f}, {location.z:.2f})"
        )
    return road_to_edge[waypoint.road_id][waypoint.section_id][waypoint.lane_id]


def _planner_localize_candidates(carla_map, road_to_edge, location):
    waypoint = carla_map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
    if waypoint is None:
        raise RuntimeError(
            f"Failed to localize planner anchor at ({location.x:.2f}, {location.y:.2f}, {location.z:.2f})"
        )

    section_edges = road_to_edge.get(waypoint.road_id, {}).get(waypoint.section_id, {})
    if not section_edges:
        raise RuntimeError(
            "Planner graph has no section entry for localized anchor "
            f"road/section/lane={waypoint.road_id}/{waypoint.section_id}/{waypoint.lane_id}"
        )

    same_sign = []
    opposite_sign = []
    for lane_id, edge in section_edges.items():
        bucket = same_sign if lane_id * waypoint.lane_id > 0 else opposite_sign
        bucket.append((abs(lane_id - waypoint.lane_id), abs(lane_id), edge))

    ordered = same_sign or opposite_sign
    ordered.sort()
    candidates = [edge for _, _, edge in ordered]

    exact = section_edges.get(waypoint.lane_id)
    if exact is not None:
        candidates = [exact] + [edge for edge in candidates if edge != exact]
    return candidates


def _planner_astar(adjacency, vertices, start_node, goal_node):
    gx, gy, _ = vertices[goal_node]
    queue = [(0.0, 0.0, start_node)]
    best_cost = {start_node: 0.0}
    parent = {start_node: None}

    while queue:
        _, cost, node = heapq.heappop(queue)
        if cost != best_cost.get(node):
            continue
        if node == goal_node:
            path = []
            while node is not None:
                path.append(node)
                node = parent[node]
            return list(reversed(path))
        for nxt, weight in adjacency.get(node, []):
            new_cost = cost + weight
            if new_cost < best_cost.get(nxt, float("inf")):
                best_cost[nxt] = new_cost
                parent[nxt] = node
                x, y, _ = vertices[nxt]
                heuristic = math.hypot(gx - x, gy - y)
                heapq.heappush(queue, (new_cost + heuristic, new_cost, nxt))
    return None


def _closest_waypoint_index(current_wp, waypoint_list):
    best_dist = float("inf")
    best_idx = 0
    for idx, waypoint in enumerate(waypoint_list):
        dist = waypoint.transform.location.distance(current_wp.transform.location)
        if dist < best_dist:
            best_dist = dist
            best_idx = idx
    return best_idx


def _trace_planner_segment(carla_map, planner_graph, origin, destination):
    edges = planner_graph["edges"]
    adjacency = planner_graph["adjacency"]
    vertices = planner_graph["vertices"]
    road_to_edge = planner_graph["road_to_edge"]

    start_candidates = _planner_localize_candidates(carla_map, road_to_edge, origin)
    end_candidates = _planner_localize_candidates(carla_map, road_to_edge, destination)

    start_edge = None
    end_edge = None
    node_path = None
    for start_candidate in start_candidates[:4]:
        for end_candidate in end_candidates[:4]:
            candidate_path = _planner_astar(
                adjacency,
                vertices,
                start_candidate[0],
                end_candidate[0],
            )
            if candidate_path:
                start_edge = start_candidate
                end_edge = end_candidate
                node_path = candidate_path
                break
        if node_path:
            break

    if not node_path or start_edge is None or end_edge is None:
        raise RuntimeError(
            f"Failed to plan route between anchors ({origin.x:.1f}, {origin.y:.1f})"
            f" -> ({destination.x:.1f}, {destination.y:.1f})"
        )
    node_path.append(end_edge[1])

    current_wp = carla_map.get_waypoint(origin, project_to_road=True, lane_type=carla.LaneType.Driving)
    destination_wp = carla_map.get_waypoint(destination, project_to_road=True, lane_type=carla.LaneType.Driving)
    trace = []

    for idx in range(len(node_path) - 1):
        edge = edges[(node_path[idx], node_path[idx + 1])]
        waypoint_path = edge["path"]
        closest_idx = _closest_waypoint_index(current_wp, waypoint_path)
        for waypoint in waypoint_path[closest_idx:]:
            current_wp = waypoint
            trace.append(waypoint)
            if len(node_path) - idx <= 2 and waypoint.transform.location.distance(destination) < 2.0 * 2.0:
                break
            if (
                len(node_path) - idx <= 2
                and current_wp.road_id == destination_wp.road_id
                and current_wp.section_id == destination_wp.section_id
                and current_wp.lane_id == destination_wp.lane_id
            ):
                destination_idx = _closest_waypoint_index(destination_wp, waypoint_path)
                if closest_idx > destination_idx:
                    break

    return trace

def build_planned_route_waypoints(carla_map, cfg):
    route_cfg = cfg.get("route", {})
    planner_cfg = route_cfg.get("planner", {})
    anchors = planner_cfg.get("anchors", [])
    if len(anchors) < 2:
        return []

    sampling_resolution_m = float(planner_cfg.get("sampling_resolution_m", 2.0))
    simplify_epsilon_m = float(planner_cfg.get("simplify_epsilon_m", 1.0))
    max_segment_length_m = float(planner_cfg.get("max_segment_length_m", 25.0))

    planner_graph = _build_planner_graph(carla_map, sampling_resolution_m)

    dense_trace = []
    for start_anchor, end_anchor in zip(anchors, anchors[1:]):
        start_loc = _location_from_anchor(start_anchor)
        end_loc = _location_from_anchor(end_anchor)
        segment = _trace_planner_segment(carla_map, planner_graph, start_loc, end_loc)
        if dense_trace and segment:
            segment = segment[1:]
        dense_trace.extend(segment)

    route_points = []
    for waypoint in dense_trace:
        loc = waypoint.transform.location
        point = carla.Location(x=loc.x, y=loc.y, z=max(0.5, loc.z))
        if not route_points or horizontal_distance(point, route_points[-1]) > 0.5:
            route_points.append(point)

    simplified = _rdp_simplify_locations(route_points, simplify_epsilon_m)
    return _densify_locations(simplified, max_segment_length_m)


def _lane_trace_choice_score(current_wp, candidate_wp, turn_hint):
    yaw_delta = _yaw_delta_signed_deg(candidate_wp.transform.rotation.yaw, current_wp.transform.rotation.yaw)
    same_road_penalty = 0.0 if candidate_wp.road_id == current_wp.road_id else 20.0
    same_lane_penalty = 0.0 if candidate_wp.lane_id == current_wp.lane_id else 5.0

    if turn_hint == "left":
        return -yaw_delta + same_lane_penalty
    if turn_hint == "right":
        return yaw_delta + same_lane_penalty
    if turn_hint == "straight":
        return abs(yaw_delta) + same_road_penalty + same_lane_penalty
    return abs(yaw_delta) + same_road_penalty + same_lane_penalty


def _lane_trace_candidate_valid(carla_map, candidate_wp):
    if candidate_wp is None or candidate_wp.lane_type != carla.LaneType.Driving:
        return False
    if candidate_wp.lane_width < 2.5:
        return False

    loc = candidate_wp.transform.location
    exact = carla_map.get_waypoint(
        loc,
        project_to_road=False,
        lane_type=carla.LaneType.Driving,
    )
    return exact is not None


def build_lane_trace_route_waypoints(carla_map, cfg):
    route_cfg = cfg.get("route", {})
    trace_cfg = route_cfg.get("lane_trace", {})
    start_cfg = trace_cfg.get("start", {})
    start_loc = _location_from_anchor(start_cfg) if start_cfg else make_transform_from_config(cfg).location
    step_m = float(trace_cfg.get("step_m", 4.0))
    target_length_m = float(trace_cfg.get("target_length_m", 6000.0))
    max_steps = int(trace_cfg.get("max_steps", max(2000, target_length_m / max(step_m, 0.5) * 2)))
    decisions = list(trace_cfg.get("junction_decisions", []))
    decision_index = 0

    current_wp = carla_map.get_waypoint(start_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    if current_wp is None:
        return []

    route_points = []
    travelled_m = 0.0
    last_junction_id = None

    loc = current_wp.transform.location
    route_points.append(carla.Location(x=loc.x, y=loc.y, z=max(0.5, loc.z)))

    for _ in range(max_steps):
        next_wps = current_wp.next(step_m)
        if not next_wps:
            break

        next_wps = [wp for wp in next_wps if _lane_trace_candidate_valid(carla_map, wp)]
        if not next_wps:
            break

        turn_hint = None
        if len(next_wps) > 1 and decision_index < len(decisions):
            # The actionable split often appears before current_wp.is_junction.
            # Consume scripted route decisions at the split, not after it.
            turn_hint = decisions[decision_index]
            decision_index += 1
        else:
            junction_id = current_wp.junction_id if current_wp.is_junction else None
            if junction_id is not None and junction_id != last_junction_id:
                if decision_index < len(decisions):
                    turn_hint = decisions[decision_index]
                    decision_index += 1
                last_junction_id = junction_id

        if current_wp.is_junction:
            last_junction_id = current_wp.junction_id

        next_wp = min(next_wps, key=lambda wp: _lane_trace_choice_score(current_wp, wp, turn_hint))
        next_loc = next_wp.transform.location
        point = carla.Location(x=next_loc.x, y=next_loc.y, z=max(0.5, next_loc.z))
        seg_len = horizontal_distance(route_points[-1], point)
        if seg_len > 0.25:
            route_points.append(point)
            travelled_m += seg_len

        current_wp = next_wp
        if travelled_m >= target_length_m:
            break

    return route_points


def _route_progresses(points):
    progresses = [0.0]
    for start, end in zip(points, points[1:]):
        progresses.append(progresses[-1] + horizontal_distance(start, end))
    return progresses


def _profile_lateral_offset(progress_m, profile):
    offset = 0.0
    for item in profile:
        start = float(item.get("progress_start_m", 0.0))
        end = float(item.get("progress_end_m", start))
        from_offset = float(item.get("from_offset_m", offset))
        to_offset = float(item.get("to_offset_m", from_offset))
        hold_after = bool(item.get("hold_after", True))

        if progress_m < start:
            continue
        if end <= start or progress_m >= end:
            offset = to_offset if hold_after else 0.0
            continue

        ratio = clamp((progress_m - start) / max(end - start, 1e-6), 0.0, 1.0)
        # Smoothstep avoids a visible kink at lane-change start/end.
        smooth = ratio * ratio * (3.0 - 2.0 * ratio)
        offset = from_offset + smooth * (to_offset - from_offset)
    return offset


def apply_lateral_offset_profile(route_points, profile):
    if not route_points or not profile:
        return route_points

    progresses = _route_progresses(route_points)
    shifted = []
    count = len(route_points)
    for idx, point in enumerate(route_points):
        if count == 1:
            shifted.append(point)
            continue

        if idx == 0:
            prev_point = route_points[idx]
            next_point = route_points[idx + 1]
        elif idx == count - 1:
            prev_point = route_points[idx - 1]
            next_point = route_points[idx]
        else:
            prev_point = route_points[idx - 1]
            next_point = route_points[idx + 1]

        dx = next_point.x - prev_point.x
        dy = next_point.y - prev_point.y
        norm = math.hypot(dx, dy)
        if norm <= 1e-6:
            shifted.append(point)
            continue

        offset = _profile_lateral_offset(progresses[idx], profile)
        right = carla.Vector3D(-dy / norm, dx / norm, 0.0)
        shifted.append(
            carla.Location(
                x=point.x + offset * right.x,
                y=point.y + offset * right.y,
                z=point.z,
            )
        )

    return shifted


def load_route_waypoints(cfg):
    route_cfg = cfg.get("route", {})
    mode = route_cfg.get("mode")
    if mode in {"carla_runtime_planner", "carla_lane_trace"}:
        return []
    route_file = resolve_repo_relative_path(cfg, route_cfg.get("route_file"))
    if route_file is None or not route_file.exists():
        return []

    route_id = str(route_cfg.get("route_id", 0))
    root = ET.parse(route_file).getroot()
    for route in root.findall("route"):
        if route.get("id") != route_id:
            continue
        points = []
        for waypoint in route.findall("waypoint"):
            points.append(carla.Location(
                x=float(waypoint.get("x", 0.0)),
                y=float(waypoint.get("y", 0.0)),
                z=float(waypoint.get("z", 0.0)),
            ))
        return points

    return []


def route_length_from_points(points):
    if len(points) < 2:
        return 0.0
    return sum(horizontal_distance(points[idx - 1], points[idx]) for idx in range(1, len(points)))


def make_reference_from_points(points):
    if len(points) >= 2:
        start = points[0]
        next_point = points[1]
        dx = next_point.x - start.x
        dy = next_point.y - start.y
        norm = math.hypot(dx, dy)
        if norm > 1e-6:
            forward = carla.Vector3D(dx / norm, dy / norm, 0.0)
            right = carla.Vector3D(-forward.y, forward.x, 0.0)
            return start, forward, right

    start = points[0] if points else carla.Location()
    return start, carla.Vector3D(1.0, 0.0, 0.0), carla.Vector3D(0.0, 1.0, 0.0)


def make_reference_from_transform(transform):
    ref_loc = carla.Location(
        x=transform.location.x,
        y=transform.location.y,
        z=transform.location.z,
    )
    ref_forward = transform.get_forward_vector()
    ref_right = transform.get_right_vector()
    return ref_loc, ref_forward, ref_right


def make_location_along_transform(transform, distance_m):
    forward = transform.get_forward_vector()
    return carla.Location(
        x=transform.location.x + distance_m * forward.x,
        y=transform.location.y + distance_m * forward.y,
        z=transform.location.z + distance_m * forward.z,
    )


def estimate_forward_route_length(carla_map, spawn_transform, step_m=2.0, max_distance_m=100.0):
    waypoint = carla_map.get_waypoint(
        spawn_transform.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if waypoint is None:
        return max_distance_m

    total = 0.0
    current = waypoint

    while total < max_distance_m:
        next_wps = current.next(step_m)
        if not next_wps:
            break

        next_wp = next_wps[0]
        total += horizontal_distance(current.transform.location, next_wp.transform.location)
        current = next_wp

    return max(total, step_m)


def load_world_for_config(client, cfg):
    desired_town = cfg.get("map", {}).get("town")
    world = client.get_world()

    if desired_town and not world.get_map().name.endswith(desired_town):
        world = client.load_world(desired_town)
    return world


def apply_weather_from_config(world, cfg):
    map_cfg = cfg.get("map", {})
    weather_params = map_cfg.get("weather_parameters")
    if weather_params:
        world.set_weather(carla.WeatherParameters(**weather_params))
        return

    weather_name = map_cfg.get("weather")
    if not weather_name:
        return

    if hasattr(carla.WeatherParameters, weather_name):
        world.set_weather(getattr(carla.WeatherParameters, weather_name))


def make_transform_from_config(cfg):
    spawn = cfg.get("ego", {}).get("spawn_point", {})
    return carla.Transform(
        location=carla.Location(
            x=float(spawn.get("x", 0.0)),
            y=float(spawn.get("y", 0.0)),
            z=float(spawn.get("z", 0.5)),
        ),
        rotation=carla.Rotation(
            pitch=float(spawn.get("pitch", 0.0)),
            yaw=float(spawn.get("yaw", 0.0)),
            roll=float(spawn.get("roll", 0.0)),
        ),
    )


def make_lane_aligned_transform_from_config(carla_map, cfg):
    transform = make_transform_from_config(cfg)
    route_cfg = cfg.get("route", {})
    trace_cfg = route_cfg.get("lane_trace", {})
    align_to_lane = bool(
        trace_cfg.get("align_spawn_to_lane", route_cfg.get("mode") == "carla_lane_trace")
    )
    if not align_to_lane:
        return transform

    start_cfg = trace_cfg.get("start", {})
    start_loc = _location_from_anchor(start_cfg) if start_cfg else transform.location
    waypoint = carla_map.get_waypoint(
        start_loc,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if waypoint is None:
        return transform

    aligned = waypoint.transform
    aligned.location.z = max(float(start_cfg.get("z", transform.location.z)), aligned.location.z + 0.5)
    return aligned


def route_debug_summary(carla_map, route_points, max_samples=8):
    if not route_points:
        return {"point_count": 0, "total_length_m": 0.0, "samples": []}

    point_count = len(route_points)
    if point_count <= max_samples:
        sample_indices = list(range(point_count))
    else:
        step = max(1, int(math.floor((point_count - 1) / max(1, max_samples - 1))))
        sample_indices = list(range(0, point_count, step))[:max_samples]
        if sample_indices[-1] != point_count - 1:
            sample_indices[-1] = point_count - 1

    samples = []
    for idx in sample_indices:
        point = route_points[idx]
        waypoint = carla_map.get_waypoint(
            point,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        sample = {
            "idx": idx,
            "x": round(point.x, 2),
            "y": round(point.y, 2),
            "z": round(point.z, 2),
        }
        if waypoint is not None:
            sample.update({
                "road_id": waypoint.road_id,
                "section_id": waypoint.section_id,
                "lane_id": waypoint.lane_id,
                "is_junction": bool(waypoint.is_junction),
                "yaw": round(waypoint.transform.rotation.yaw, 1),
            })
        samples.append(sample)

    return {
        "point_count": point_count,
        "total_length_m": round(route_length_from_points(route_points), 2),
        "samples": samples,
    }


def get_instruction_trigger_time(cfg, instruction_index=0, default=0.0):
    instructions = cfg.get("instructions", [])
    if instruction_index >= len(instructions):
        return default
    trigger = instructions[instruction_index].get("trigger", {})
    if trigger.get("type") == "time":
        return float(trigger.get("value", default))
    return default


def get_controller_param(cfg, name, default):
    return cfg.get("controller", {}).get(name, default)


class LaneInvasionTracker:
    def __init__(self, world, blueprint_library, ego):
        self.sensor = world.spawn_actor(
            blueprint_library.find("sensor.other.lane_invasion"),
            carla.Transform(),
            attach_to=ego,
        )
        self._triggered = False
        self._markings = []
        self.sensor.listen(self._on_event)

    def _on_event(self, event):
        self._triggered = True
        self._markings = [
            getattr(marking.type, "name", str(marking.type))
            for marking in event.crossed_lane_markings
        ]

    def snapshot(self):
        payload = {
            "lane_invasion": self._triggered,
            "crossed_lane_markings": list(self._markings),
        }
        self._triggered = False
        self._markings = []
        return payload

    def destroy(self):
        if self.sensor is not None:
            self.sensor.destroy()
            self.sensor = None


class RouteTracker:
    def __init__(
        self,
        carla_map,
        route_points,
        corridor_half_width_m=4.5,
        backward_tolerance_m=2.0,
        continuity_search_ahead_m=80.0,
        continuity_search_behind_m=20.0,
    ):
        self.carla_map = carla_map
        self.route_points = route_points
        self.corridor_half_width_m = float(corridor_half_width_m)
        self.backward_tolerance_m = float(backward_tolerance_m)
        self.continuity_search_ahead_m = float(continuity_search_ahead_m)
        self.continuity_search_behind_m = float(continuity_search_behind_m)
        self.route_total_length_m = max(route_length_from_points(route_points), 1.0)
        self.last_progress_m = 0.0
        self.max_progress_m = 0.0
        self.ref_loc, self.ref_forward, self.ref_right = make_reference_from_points(route_points)

    @classmethod
    def from_route_config(cls, carla_map, cfg, corridor_half_width_m=4.5):
        route_cfg = cfg.get("route", {})
        if route_cfg.get("mode") == "carla_runtime_planner":
            route_points = build_planned_route_waypoints(carla_map, cfg)
        elif route_cfg.get("mode") == "carla_lane_trace":
            route_points = build_lane_trace_route_waypoints(carla_map, cfg)
        else:
            route_points = load_route_waypoints(cfg)
        if not route_points:
            spawn = make_transform_from_config(cfg)
            route_points = [
                carla.Location(x=spawn.location.x, y=spawn.location.y, z=spawn.location.z),
                make_location_along_transform(
                    spawn,
                    estimate_forward_route_length(carla_map, spawn),
                ),
            ]
        route_points = apply_lateral_offset_profile(
            route_points,
            route_cfg.get("lateral_offset_profile", []),
        )
        return cls(carla_map, route_points, corridor_half_width_m=corridor_half_width_m)

    def _segment_projection(self, point, seg_start, seg_end):
        sx = seg_start.x
        sy = seg_start.y
        ex = seg_end.x
        ey = seg_end.y
        px = point.x
        py = point.y

        dx = ex - sx
        dy = ey - sy
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq <= 1e-9:
            projected = carla.Location(x=sx, y=sy, z=seg_start.z)
            return 0.0, projected, horizontal_distance(point, projected), 0.0

        t = ((px - sx) * dx + (py - sy) * dy) / seg_len_sq
        t_clamped = clamp(t, 0.0, 1.0)
        proj = carla.Location(
            x=sx + t_clamped * dx,
            y=sy + t_clamped * dy,
            z=seg_start.z,
        )
        dist = horizontal_distance(point, proj)
        seg_len = math.sqrt(seg_len_sq)
        forward = carla.Vector3D(dx / seg_len, dy / seg_len, 0.0)
        right = carla.Vector3D(-forward.y, forward.x, 0.0)
        rel = carla.Location(x=point.x - proj.x, y=point.y - proj.y, z=0.0)
        lateral_offset = dot2d(rel, right)
        return t_clamped, proj, dist, lateral_offset

    def measure(self, ego_loc):
        cumulative = 0.0
        best = None
        best_continuous = None

        # Prefer projections that stay close to the previously selected route
        # progress. This prevents self-intersections or repeated route segments
        # from snapping the ego vehicle to a far-away part of the route.
        expected_progress = self.last_progress_m
        min_progress = max(0.0, expected_progress - self.continuity_search_behind_m)
        max_progress = expected_progress + self.continuity_search_ahead_m

        for idx in range(1, len(self.route_points)):
            start = self.route_points[idx - 1]
            end = self.route_points[idx]
            seg_len = horizontal_distance(start, end)
            t, proj, dist, lateral_offset = self._segment_projection(ego_loc, start, end)
            progress = cumulative + t * seg_len

            if best is None or dist < best["distance"]:
                best = {
                    "distance": dist,
                    "progress": progress,
                    "lateral_offset": lateral_offset,
                }

            if min_progress <= progress <= max_progress:
                if best_continuous is None or dist < best_continuous["distance"]:
                    best_continuous = {
                        "distance": dist,
                        "progress": progress,
                        "lateral_offset": lateral_offset,
                    }

            cumulative += seg_len

        if best_continuous is not None:
            best = best_continuous

        if best is None:
            best = {"distance": 0.0, "progress": 0.0, "lateral_offset": 0.0}

        self.last_progress_m = best["progress"]
        self.max_progress_m = max(self.max_progress_m, best["progress"])
        waypoint = self.carla_map.get_waypoint(
            ego_loc,
            project_to_road=False,
            lane_type=carla.LaneType.Driving,
        )
        on_driving_lane = waypoint is not None
        route_deviation = (
            (not on_driving_lane)
            or abs(best["lateral_offset"]) > self.corridor_half_width_m
            or best["progress"] < -self.backward_tolerance_m
        )

        return {
            "route_progress_m": best["progress"],
            "max_route_progress_m": self.max_progress_m,
            "route_total_length_m": self.route_total_length_m,
            "route_completion": clamp(self.max_progress_m / self.route_total_length_m, 0.0, 1.0),
            "lateral_offset_from_route_m": best["lateral_offset"],
            "route_distance_to_centerline_m": best["distance"],
            "on_driving_lane": on_driving_lane,
            "route_deviation": route_deviation,
        }

    def point_at_progress(self, progress_m, lateral_offset_m=0.0):
        if not self.route_points:
            return carla.Location()

        if len(self.route_points) == 1:
            return carla.Location(
                x=self.route_points[0].x,
                y=self.route_points[0].y,
                z=self.route_points[0].z,
            )

        first_start = self.route_points[0]
        first_end = self.route_points[1]
        last_start = self.route_points[-2]
        last_end = self.route_points[-1]

        first_seg_len = horizontal_distance(first_start, first_end)
        last_seg_len = horizontal_distance(last_start, last_end)

        if progress_m <= 0.0 and first_seg_len > 1e-9:
            dx = first_end.x - first_start.x
            dy = first_end.y - first_start.y
            forward = carla.Vector3D(dx / first_seg_len, dy / first_seg_len, 0.0)
            right = carla.Vector3D(-forward.y, forward.x, 0.0)
            return carla.Location(
                x=first_start.x + progress_m * forward.x + lateral_offset_m * right.x,
                y=first_start.y + progress_m * forward.y + lateral_offset_m * right.y,
                z=first_start.z,
            )

        if progress_m >= self.route_total_length_m and last_seg_len > 1e-9:
            extra_progress = progress_m - self.route_total_length_m
            dx = last_end.x - last_start.x
            dy = last_end.y - last_start.y
            dz = last_end.z - last_start.z
            forward = carla.Vector3D(dx / last_seg_len, dy / last_seg_len, 0.0)
            right = carla.Vector3D(-forward.y, forward.x, 0.0)
            z_forward = dz / last_seg_len
            return carla.Location(
                x=last_end.x + extra_progress * forward.x + lateral_offset_m * right.x,
                y=last_end.y + extra_progress * forward.y + lateral_offset_m * right.y,
                z=last_end.z + extra_progress * z_forward,
            )

        target_progress = progress_m
        cumulative = 0.0

        for idx in range(1, len(self.route_points)):
            start = self.route_points[idx - 1]
            end = self.route_points[idx]
            seg_len = horizontal_distance(start, end)

            if seg_len <= 1e-9:
                continue

            if target_progress <= cumulative + seg_len or idx == len(self.route_points) - 1:
                ratio = clamp((target_progress - cumulative) / seg_len, 0.0, 1.0)
                dx = end.x - start.x
                dy = end.y - start.y
                forward = carla.Vector3D(dx / seg_len, dy / seg_len, 0.0)
                right = carla.Vector3D(-forward.y, forward.x, 0.0)
                return carla.Location(
                    x=start.x + ratio * dx + lateral_offset_m * right.x,
                    y=start.y + ratio * dy + lateral_offset_m * right.y,
                    z=start.z + ratio * (end.z - start.z),
                )

            cumulative += seg_len

        end = self.route_points[-1]
        return carla.Location(x=end.x, y=end.y, z=end.z)

    def _smoothed_center_point_at_progress(self, progress_m, window_radius_m=6.0, sample_step_m=2.0, sigma_m=None):
        if len(self.route_points) <= 2:
            return self.point_at_progress(progress_m)

        window_radius_m = max(float(window_radius_m), 0.0)
        sample_step_m = max(float(sample_step_m), 0.5)
        if window_radius_m <= 1e-3:
            return self.point_at_progress(progress_m)

        if sigma_m is None:
            sigma_m = max(window_radius_m * 0.5, sample_step_m)
        sigma_m = max(float(sigma_m), 1e-3)

        sample_count_each_side = max(1, int(math.ceil(window_radius_m / sample_step_m)))
        sum_w = 0.0
        sum_x = 0.0
        sum_y = 0.0
        sum_z = 0.0

        for sample_idx in range(-sample_count_each_side, sample_count_each_side + 1):
            offset_m = sample_idx * sample_step_m
            if abs(offset_m) > window_radius_m + 1e-6:
                continue
            weight = math.exp(-0.5 * (offset_m / sigma_m) ** 2)
            point = self.point_at_progress(progress_m + offset_m)
            sum_w += weight
            sum_x += weight * point.x
            sum_y += weight * point.y
            sum_z += weight * point.z

        if sum_w <= 1e-9:
            return self.point_at_progress(progress_m)

        return carla.Location(x=sum_x / sum_w, y=sum_y / sum_w, z=sum_z / sum_w)

    def point_at_progress_smoothed(
        self,
        progress_m,
        lateral_offset_m=0.0,
        window_radius_m=6.0,
        sample_step_m=2.0,
        sigma_m=None,
        tangent_delta_m=None,
    ):
        center = self._smoothed_center_point_at_progress(
            progress_m,
            window_radius_m=window_radius_m,
            sample_step_m=sample_step_m,
            sigma_m=sigma_m,
        )

        if abs(float(lateral_offset_m)) <= 1e-6:
            return center

        if tangent_delta_m is None:
            tangent_delta_m = max(sample_step_m, min(window_radius_m, 3.0))
        tangent_delta_m = max(float(tangent_delta_m), 0.5)

        prev_center = self._smoothed_center_point_at_progress(
            progress_m - tangent_delta_m,
            window_radius_m=window_radius_m,
            sample_step_m=sample_step_m,
            sigma_m=sigma_m,
        )
        next_center = self._smoothed_center_point_at_progress(
            progress_m + tangent_delta_m,
            window_radius_m=window_radius_m,
            sample_step_m=sample_step_m,
            sigma_m=sigma_m,
        )
        dx = next_center.x - prev_center.x
        dy = next_center.y - prev_center.y
        norm = math.hypot(dx, dy)
        if norm <= 1e-6:
            return self.point_at_progress(progress_m, lateral_offset_m=lateral_offset_m)

        right = carla.Vector3D(-dy / norm, dx / norm, 0.0)
        return carla.Location(
            x=center.x + float(lateral_offset_m) * right.x,
            y=center.y + float(lateral_offset_m) * right.y,
            z=center.z,
        )


class RedLightViolationTracker:
    def __init__(
        self,
        world,
        ref_loc,
        ref_forward,
        ref_right,
        detection_distance_m=40.0,
        lane_tolerance_m=5.5,
        crossing_buffer_m=1.0,
    ):
        self.ref_loc = ref_loc
        self.ref_forward = ref_forward
        self.ref_right = ref_right
        self.detection_distance_m = float(detection_distance_m)
        self.lane_tolerance_m = float(lane_tolerance_m)
        self.crossing_buffer_m = float(crossing_buffer_m)
        self._traffic_lights = list(world.get_actors().filter("traffic.traffic_light*"))
        self._active_light_id = None
        self._active_stop_progress = None
        self._active_light_state = None

    def _project(self, location):
        rel = carla.Location(
            x=location.x - self.ref_loc.x,
            y=location.y - self.ref_loc.y,
            z=0.0,
        )
        return dot2d(rel, self.ref_forward), dot2d(rel, self.ref_right)

    def _get_light_stop_waypoints(self, light):
        # CARLA Python APIs differ by version. Prefer explicit stop-line waypoints,
        # then fall back to affected lane waypoints if available.
        if hasattr(light, "get_stop_waypoints"):
            try:
                return list(light.get_stop_waypoints())
            except (RuntimeError, TypeError, AttributeError):
                pass

        if hasattr(light, "get_affected_lane_waypoints"):
            try:
                return list(light.get_affected_lane_waypoints())
            except (RuntimeError, TypeError, AttributeError):
                pass

        return []

    def _find_candidate(self, ego_progress):
        best = None
        best_gap = None

        for light in self._traffic_lights:
            if light.state != carla.TrafficLightState.Red:
                continue

            stop_waypoints = self._get_light_stop_waypoints(light)
            if not stop_waypoints:
                continue

            for stop_wp in stop_waypoints:
                stop_loc = stop_wp.transform.location
                stop_progress, stop_lateral = self._project(stop_loc)
                gap = stop_progress - ego_progress

                if gap < -2.0 or gap > self.detection_distance_m:
                    continue
                if abs(stop_lateral) > self.lane_tolerance_m:
                    continue

                if best_gap is None or gap < best_gap:
                    best_gap = gap
                    best = {
                        "light_id": light.id,
                        "state": light.state.name,
                        "stop_progress_m": stop_progress,
                        "gap_ahead_m": gap,
                    }

        return best

    def update(self, ego_loc, ego_speed_kmh):
        ego_progress, _ = self._project(ego_loc)
        candidate = self._find_candidate(ego_progress)

        if candidate is not None:
            self._active_light_id = candidate["light_id"]
            self._active_stop_progress = candidate["stop_progress_m"]
            self._active_light_state = candidate["state"]
        elif self._active_light_id is not None and ego_progress > self._active_stop_progress + 8.0:
            self._active_light_id = None
            self._active_stop_progress = None
            self._active_light_state = None

        violation = False
        if (
            self._active_light_id is not None
            and self._active_stop_progress is not None
            and ego_progress > self._active_stop_progress + self.crossing_buffer_m
            and ego_speed_kmh > 1.0
        ):
            violation = True
            self._active_light_id = None
            self._active_stop_progress = None
            self._active_light_state = None

        return {
            "red_light_violation": violation,
            "active_traffic_light_id": candidate["light_id"] if candidate is not None else self._active_light_id,
            "active_traffic_light_state": candidate["state"] if candidate is not None else self._active_light_state,
            "active_stop_line_progress_m": candidate["stop_progress_m"] if candidate is not None else self._active_stop_progress,
        }
