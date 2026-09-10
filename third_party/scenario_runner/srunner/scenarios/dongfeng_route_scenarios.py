"""Dongfeng scene-side scenarios for the official Leaderboard route runner.

These classes deliberately contain only scenario actors and behavior trees. The
ego vehicle remains controlled by the Leaderboard agent.
"""

import py_trees
import carla

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorDestroy,
    ActorTransformSetter,
    LaneChange,
    KeepVelocity,
    WaypointFollower,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance,
    InTriggerDistanceToLocation,
)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import AtomicCondition

OFFSTAGE_TRANSFORM = carla.Transform(
    carla.Location(x=10000.0, y=10000.0, z=-50.0),
    carla.Rotation(yaw=0.0),
)


def _request_actor_with_fallback(actor_config, spawn_transform=None):
    """Spawn an XML actor, falling back to a compatible CARLA blueprint."""
    transform = spawn_transform or actor_config.transform
    try:
        actor = CarlaDataProvider.request_new_actor(
            actor_config.model, transform
        )
    except Exception:
        actor = None
    if actor is not None:
        return actor

    library = CarlaDataProvider.get_world().get_blueprint_library()
    candidates = []
    model = str(actor_config.model)
    if model.startswith("walker."):
        candidates = ["walker.pedestrian.*"]
    elif model.startswith("vehicle."):
        candidates = ["vehicle.*"]
    elif model.startswith("static.prop."):
        candidates = ["static.prop.*"]
    for pattern in candidates:
        matches = library.filter(pattern)
        if matches:
            try:
                actor = CarlaDataProvider.request_new_actor(
                    matches[0].id, transform
                )
            except Exception:
                actor = None
            if actor is not None:
                return actor
    return None


def _initialize_xml_actors(scenario, config):
    """Spawn actors offstage and retain their route-local target transforms."""
    scenario._configured_transforms = []
    for actor_config in config.other_actors:
        offstage = OFFSTAGE_TRANSFORM
        target = actor_config.transform
        actor_config.transform = offstage
        actor = _request_actor_with_fallback(actor_config)
        if actor is None:
            # CARLA validates spawn locations. Spawn at the intended legal
            # route transform, then hide the actor until the progress gate.
            actor = _request_actor_with_fallback(actor_config, target)
        actor_config.transform = target
        if actor is None:
            raise RuntimeError("Unable to spawn XML actor: {}".format(actor_config.model))
        if not actor.type_id.startswith("static.prop."):
            actor.set_simulate_physics(False)
        actor.set_transform(offstage)
        scenario.other_actors.append(actor)
        scenario._configured_transforms.append(target)


def _set_target_transforms(sequence, scenario, physics=True):
    for actor, transform in zip(
        scenario.other_actors, getattr(scenario, "_configured_transforms", [])
    ):
        actor_physics = physics and not actor.type_id.startswith("static.prop.")
        sequence.add_child(ActorTransformSetter(actor, transform, actor_physics))


class InDongfengRouteProgress(AtomicCondition):
    """Wait for monotonic progress along the official route polyline."""

    def __init__(self, actor, route, target_progress, tolerance=4.0):
        super().__init__("InDongfengRouteProgress")
        self._actor = actor
        self._route = route
        self._target_progress = float(target_progress)
        self._tolerance = float(tolerance)
        self._progresses = [0.0]
        for first, second in zip(route, route[1:]):
            self._progresses.append(
                self._progresses[-1]
                + first[0].location.distance(second[0].location)
            )
        self._last_index = 0

    def update(self):
        location = CarlaDataProvider.get_location(self._actor)
        if location is None or not self._route:
            return py_trees.common.Status.RUNNING

        # Search only forward from the last accepted route point. This avoids
        # jumping to a spatially identical point on a later loop of Town05.
        start = self._last_index
        end = min(len(self._route), start + 120)
        candidates = range(start, end)
        index = min(
            candidates,
            key=lambda i: self._route[i][0].location.distance(location),
        )
        self._last_index = max(self._last_index, index)
        progress = self._progresses[self._last_index]
        if progress + self._tolerance >= self._target_progress:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class DongfengProgressTriggeredScenario(BasicScenario):
    """Common route-progress gate for all Dongfeng official scenarios."""

    def _setup_scenario_trigger(self, config):
        if not self.route_mode or not config.route:
            return super()._setup_scenario_trigger(config)

        progress_data = config.other_parameters.get("route_progress", {})
        target = progress_data.get("value")
        if target is None:
            return super()._setup_scenario_trigger(config)

        # Route XML trigger points are spatial hints. On a looped map the same
        # point can occur again later, so route progress is the authoritative
        # trigger for scene-side actors in official route mode.
        return InDongfengRouteProgress(
            self.ego_vehicles[0], config.route, float(target)
        )


class DongfengSlowVehicle(DongfengProgressTriggeredScenario):
    """Place a slow lead vehicle and keep it moving until the ego passes."""

    timeout = 180

    def __init__(self, world, ego_vehicles, config, debug_mode=False,
                 criteria_enable=True, timeout=180):
        self.timeout = timeout
        self._speed = 5.6  # 20 km/h
        super().__init__("DongfengSlowVehicle", ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        _initialize_xml_actors(self, config)
        if not self.other_actors:
            raise RuntimeError("DongfengSlowVehicle needs one XML vehicle")

    def _create_behavior(self):
        vehicle = self.other_actors[0]
        visible = py_trees.composites.Sequence(name="PlaceSlowVehicle")
        _set_target_transforms(visible, self)
        drive = WaypointFollower(vehicle, self._speed, avoid_collision=True)
        finish = py_trees.composites.Parallel(
            name="SlowVehicleDriveAndFinish",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
        )
        finish.add_child(drive)
        finish.add_child(DriveDistance(self.ego_vehicles[0], 260.0))
        sequence = py_trees.composites.Sequence(name="SlowVehicleBehavior")
        sequence.add_children([visible, finish, ActorDestroy(vehicle)])
        return sequence

    def _create_test_criteria(self):
        return [CollisionTest(self.ego_vehicles[0])] if not self.route_mode else []


class DongfengPedestrianCrossing(DongfengProgressTriggeredScenario):
    """A small group of walkers crosses the route after the ego approaches."""

    timeout = 120

    def __init__(self, world, ego_vehicles, config, debug_mode=False,
                 criteria_enable=True, timeout=120):
        self.timeout = timeout
        super().__init__("DongfengPedestrianCrossing", ego_vehicles, config,
                         world, debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        _initialize_xml_actors(self, config)

    def _create_behavior(self):
        placement = py_trees.composites.Sequence(name="PlacePedestrians")
        _set_target_transforms(placement, self)
        movement = py_trees.composites.Parallel(
            name="PedestrianCrossingMovement",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
        )
        for walker in self.other_actors:
            movement.add_child(KeepVelocity(walker, 1.4, False, 12.0, 12.0))
        sequence = py_trees.composites.Sequence(name="PedestrianCrossingBehavior")
        sequence.add_child(placement)
        sequence.add_child(movement)
        sequence.add_child(DriveDistance(self.ego_vehicles[0], 80.0))
        for walker in self.other_actors:
            sequence.add_child(ActorDestroy(walker))
        return sequence

    def _create_test_criteria(self):
        return [CollisionTest(self.ego_vehicles[0])] if not self.route_mode else []


class DongfengAmbientFlow(DongfengProgressTriggeredScenario):
    """Keep adjacent-lane traffic moving as part of the scene layer."""

    timeout = 240

    def __init__(self, world, ego_vehicles, config, debug_mode=False,
                 criteria_enable=True, timeout=240):
        self.timeout = timeout
        super().__init__("DongfengAmbientFlow", ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        _initialize_xml_actors(self, config)

    def _create_behavior(self):
        placement = py_trees.composites.Sequence(name="PlaceAmbientTraffic")
        _set_target_transforms(placement, self)
        traffic = py_trees.composites.Parallel(
            name="AmbientTrafficMovement",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
        )
        for actor in self.other_actors:
            traffic.add_child(WaypointFollower(actor, 12.0, avoid_collision=True))
        sequence = py_trees.composites.Sequence(name="AmbientFlowBehavior")
        sequence.add_child(placement)
        sequence.add_child(traffic)
        sequence.add_child(DriveDistance(self.ego_vehicles[0], 420.0))
        for actor in self.other_actors:
            sequence.add_child(ActorDestroy(actor))
        return sequence

    def _create_test_criteria(self):
        return [CollisionTest(self.ego_vehicles[0])] if not self.route_mode else []


class DongfengBusStop(DongfengProgressTriggeredScenario):
    """Static bus-stop actors with pedestrians walking along the sidewalk."""

    timeout = 180

    def __init__(self, world, ego_vehicles, config, debug_mode=False,
                 criteria_enable=True, timeout=180):
        self.timeout = timeout
        super().__init__("DongfengBusStop", ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        _initialize_xml_actors(self, config)

    def _create_behavior(self):
        placement = py_trees.composites.Sequence(name="PlaceBusStopActors")
        _set_target_transforms(placement, self)
        sequence = py_trees.composites.Sequence(name="BusStopBehavior")
        sequence.add_child(placement)
        for actor in self.other_actors:
            if actor.type_id.startswith("walker."):
                sequence.add_child(KeepVelocity(actor, 1.0, False, 14.0, 14.0))
        sequence.add_child(DriveDistance(self.ego_vehicles[0], 180.0))
        for actor in self.other_actors:
            sequence.add_child(ActorDestroy(actor))
        return sequence

    def _create_test_criteria(self):
        return [CollisionTest(self.ego_vehicles[0])] if not self.route_mode else []


class DongfengConstructionWorker(DongfengProgressTriggeredScenario):
    """A worker moves a short distance beside the construction zone."""

    timeout = 120

    def __init__(self, world, ego_vehicles, config, debug_mode=False,
                 criteria_enable=True, timeout=120):
        self.timeout = timeout
        super().__init__("DongfengConstructionWorker", ego_vehicles, config,
                         world, debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        _initialize_xml_actors(self, config)
        if not self.other_actors:
            raise RuntimeError("DongfengConstructionWorker needs one XML walker")

    def _create_behavior(self):
        worker = self.other_actors[0]
        sequence = py_trees.composites.Sequence(name="ConstructionWorkerBehavior")
        placement = py_trees.composites.Sequence(name="PlaceConstructionWorker")
        _set_target_transforms(placement, self)
        sequence.add_child(placement)
        sequence.add_child(KeepVelocity(worker, 1.4, False, 18.0, 22.0))
        sequence.add_child(ActorDestroy(worker))
        return sequence

    def _create_test_criteria(self):
        return [CollisionTest(self.ego_vehicles[0])] if not self.route_mode else []


class DongfengCutIn(DongfengProgressTriggeredScenario):
    """A vehicle approaches in the adjacent lane and merges into the route."""

    timeout = 150

    def __init__(self, world, ego_vehicles, config, debug_mode=False,
                 criteria_enable=True, timeout=150):
        self.timeout = timeout
        super().__init__("DongfengCutIn", ego_vehicles, config, world,
                         debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        _initialize_xml_actors(self, config)
        if not self.other_actors:
            raise RuntimeError("DongfengCutIn needs one XML vehicle")

    def _create_behavior(self):
        vehicle = self.other_actors[0]
        root = py_trees.composites.Sequence(name="CutInBehavior")
        placement = py_trees.composites.Sequence(name="PlaceCutInVehicle")
        _set_target_transforms(placement, self)
        root.add_child(placement)
        root.add_child(WaypointFollower(vehicle, 12.5, avoid_collision=True))
        root.add_child(LaneChange(
            vehicle, speed=12.5, direction="right",
            distance_same_lane=5.0, distance_other_lane=45.0,
        ))
        root.add_child(DriveDistance(vehicle, 100.0))
        root.add_child(ActorDestroy(vehicle))
        return root

    def _create_test_criteria(self):
        return [CollisionTest(self.ego_vehicles[0])] if not self.route_mode else []


class DongfengConstructionZone(DongfengProgressTriggeredScenario):
    """Cones remain in place while the worker crosses the construction edge."""

    timeout = 180

    def __init__(self, world, ego_vehicles, config, debug_mode=False,
                 criteria_enable=True, timeout=180):
        self.timeout = timeout
        super().__init__("DongfengConstructionZone", ego_vehicles, config,
                         world, debug_mode, criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        _initialize_xml_actors(self, config)

    def _create_behavior(self):
        placement = py_trees.composites.Sequence(name="PlaceConstructionActors")
        _set_target_transforms(placement, self)
        worker_nodes = []
        for actor in self.other_actors:
            if actor.type_id.startswith("walker."):
                worker_nodes.append(KeepVelocity(actor, 1.4, False, 18.0, 22.0))

        active = py_trees.composites.Parallel(
            name="ConstructionActive",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
        )
        if worker_nodes:
            workers = py_trees.composites.Parallel(
                name="ConstructionWorkers",
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
            )
            workers.add_children(worker_nodes)
            active.add_child(workers)
        active.add_child(DriveDistance(self.ego_vehicles[0], 180.0))

        sequence = py_trees.composites.Sequence(name="ConstructionZoneBehavior")
        sequence.add_child(placement)
        sequence.add_child(active)
        for actor in self.other_actors:
            sequence.add_child(ActorDestroy(actor))
        return sequence

    def _create_test_criteria(self):
        return [CollisionTest(self.ego_vehicles[0])] if not self.route_mode else []
