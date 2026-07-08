import math

import carla


def clamp(value, low, high):
    return max(low, min(high, value))


def dot2d(vec, direction):
    return vec.x * direction.x + vec.y * direction.y


def horizontal_distance(a, b):
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def make_reference_from_transform(transform):
    ref_loc = carla.Location(
        x=transform.location.x,
        y=transform.location.y,
        z=transform.location.z,
    )
    ref_forward = transform.get_forward_vector()
    ref_right = transform.get_right_vector()
    return ref_loc, ref_forward, ref_right


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
        ref_loc,
        ref_forward,
        ref_right,
        route_length_m,
        corridor_half_width_m=4.5,
        backward_tolerance_m=2.0,
    ):
        self.carla_map = carla_map
        self.ref_loc = ref_loc
        self.ref_forward = ref_forward
        self.ref_right = ref_right
        self.route_length_m = max(float(route_length_m), 1.0)
        self.corridor_half_width_m = float(corridor_half_width_m)
        self.backward_tolerance_m = float(backward_tolerance_m)
        self.max_progress_m = 0.0

    @classmethod
    def from_spawn_transform(
        cls,
        carla_map,
        spawn_transform,
        route_length_m=None,
        corridor_half_width_m=4.5,
    ):
        ref_loc, ref_forward, ref_right = make_reference_from_transform(spawn_transform)
        if route_length_m is None:
            route_length_m = estimate_forward_route_length(carla_map, spawn_transform)
        return cls(
            carla_map=carla_map,
            ref_loc=ref_loc,
            ref_forward=ref_forward,
            ref_right=ref_right,
            route_length_m=route_length_m,
            corridor_half_width_m=corridor_half_width_m,
        )

    def measure(self, ego_loc):
        ego_rel = carla.Location(
            x=ego_loc.x - self.ref_loc.x,
            y=ego_loc.y - self.ref_loc.y,
            z=0.0,
        )
        progress_m = dot2d(ego_rel, self.ref_forward)
        lateral_offset_m = dot2d(ego_rel, self.ref_right)
        self.max_progress_m = max(self.max_progress_m, progress_m)

        waypoint = self.carla_map.get_waypoint(
            ego_loc,
            project_to_road=False,
            lane_type=carla.LaneType.Driving,
        )
        on_driving_lane = waypoint is not None

        route_deviation = (
            (not on_driving_lane)
            or abs(lateral_offset_m) > self.corridor_half_width_m
            or progress_m < -self.backward_tolerance_m
        )

        route_completion = clamp(self.max_progress_m / self.route_length_m, 0.0, 1.0)
        return {
            "route_progress_m": progress_m,
            "max_route_progress_m": self.max_progress_m,
            "route_total_length_m": self.route_length_m,
            "route_completion": route_completion,
            "lateral_offset_from_route_m": lateral_offset_m,
            "on_driving_lane": on_driving_lane,
            "route_deviation": route_deviation,
        }


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

    def _find_candidate(self, ego_progress):
        best = None
        best_gap = None

        for light in self._traffic_lights:
            if light.state != carla.TrafficLightState.Red:
                continue

            try:
                stop_waypoints = light.get_stop_waypoints()
            except RuntimeError:
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
