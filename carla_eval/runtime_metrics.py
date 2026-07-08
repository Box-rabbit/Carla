import math
import xml.etree.ElementTree as ET
from pathlib import Path

import carla


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


def load_route_waypoints(cfg):
    route_cfg = cfg.get("route", {})
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
    def __init__(self, carla_map, route_points, corridor_half_width_m=4.5, backward_tolerance_m=2.0):
        self.carla_map = carla_map
        self.route_points = route_points
        self.corridor_half_width_m = float(corridor_half_width_m)
        self.backward_tolerance_m = float(backward_tolerance_m)
        self.route_total_length_m = max(route_length_from_points(route_points), 1.0)
        self.max_progress_m = 0.0
        self.ref_loc, self.ref_forward, self.ref_right = make_reference_from_points(route_points)

    @classmethod
    def from_route_config(cls, carla_map, cfg, corridor_half_width_m=4.5):
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

            cumulative += seg_len

        if best is None:
            best = {"distance": 0.0, "progress": 0.0, "lateral_offset": 0.0}

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
