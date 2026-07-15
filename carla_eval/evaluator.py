"""
ScenarioEvaluator: unified CARLA runner replacing the 6 individual
run_carla_sXX.py main-loop copies.

Each specific scenario provides:
  - spawn_actors(world, ego, cfg) -> list[actor]
  - compute_control(ego, actors, state, cfg) -> (throttle, brake, steer)
  - update_state(ego, actors, state, dt, cfg) -> state
  - is_success(state, cfg) -> bool
  - extra_record(ego, actors, state, cfg) -> dict

The evaluator owns: CARLA connection, world settings, sensor setup,
main tick loop, frame logging, actor cleanup, spectator camera.
"""

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional

import carla
import yaml

from carla_eval.runtime_metrics import (
    LaneInvasionTracker,
    RedLightViolationTracker,
    RouteTracker,
    apply_weather_from_config,
    get_controller_param,
    get_instruction_trigger_time,
    load_world_for_config,
    make_lane_aligned_transform_from_config,
    route_debug_summary,
)
from carla_eval.lmdrive.route_audio_runtime import RouteAudioRuntime
from carla_eval.sensors.observation_builder import ObservationBuilder
from carla_eval.visualization.voice_overlay import FixedVoiceOverlay


def _get_speed_kmh(vehicle: carla.Actor) -> float:
    v = vehicle.get_velocity()
    return 3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


def _update_spectator(world: carla.World, ego: carla.Actor) -> None:
    ego_tf = ego.get_transform()
    forward = ego_tf.get_forward_vector()
    loc = ego_tf.location
    cam_loc = carla.Location(
        x=loc.x - 10.0 * forward.x,
        y=loc.y - 10.0 * forward.y,
        z=loc.z + 5.0,
    )
    world.get_spectator().set_transform(
        carla.Transform(cam_loc, carla.Rotation(pitch=-20.0, yaw=ego_tf.rotation.yaw))
    )


def _draw_route_debug(
    world: carla.World,
    route_points,
    stride: int = 1,
    life_time: float = 600.0,
    with_labels: bool = False,
) -> None:
    if not route_points:
        return

    stride = max(1, int(stride))
    debug = world.debug
    color_mid = carla.Color(0, 200, 255)
    color_start = carla.Color(0, 255, 0)
    color_end = carla.Color(255, 0, 0)
    color_line = carla.Color(0, 120, 255)

    sampled_indices = list(range(0, len(route_points), stride))
    if sampled_indices[-1] != len(route_points) - 1:
        sampled_indices.append(len(route_points) - 1)

    sampled = [route_points[i] for i in sampled_indices]

    for idx, point in enumerate(sampled):
        color = color_mid
        size = 0.10
        if idx == 0:
            color = color_start
            size = 0.16
        elif idx == len(sampled) - 1:
            color = color_end
            size = 0.16

        loc = carla.Location(x=point.x, y=point.y, z=point.z + 0.35)
        debug.draw_point(loc, size=size, color=color, life_time=life_time)

        if with_labels:
            raw_idx = sampled_indices[idx]
            debug.draw_string(
                carla.Location(x=point.x, y=point.y, z=point.z + 0.8),
                str(raw_idx),
                draw_shadow=False,
                color=color,
                life_time=life_time,
                persistent_lines=False,
            )

    for start, end in zip(sampled, sampled[1:]):
        debug.draw_line(
            carla.Location(x=start.x, y=start.y, z=start.z + 0.2),
            carla.Location(x=end.x, y=end.y, z=end.z + 0.2),
            thickness=0.08,
            color=color_line,
            life_time=life_time,
            persistent_lines=False,
        )


def _traffic_light_state_from_config(state_name: str):
    states = {
        "red": carla.TrafficLightState.Red,
        "yellow": carla.TrafficLightState.Yellow,
        "green": carla.TrafficLightState.Green,
        "off": carla.TrafficLightState.Off,
        "unknown": carla.TrafficLightState.Unknown,
    }
    return states.get(str(state_name).strip().lower())


def _apply_traffic_conditions(world: carla.World, cfg: Dict[str, Any]) -> None:
    traffic_cfg = cfg.get("traffic_conditions", {})
    light_cfg = traffic_cfg.get("traffic_lights", {})
    mode = str(light_cfg.get("mode", "preserve")).strip().lower()
    lights = list(world.get_actors().filter("traffic.traffic_light*"))

    if mode in {"", "preserve", "default", "map_default"}:
        print(f"[TRAFFIC] traffic_lights=preserve map defaults, count={len(lights)}")
        return

    if mode not in {"freeze_all", "set_all_state"}:
        print(f"[TRAFFIC] unknown traffic_lights mode={mode}, preserving map defaults")
        return

    state = _traffic_light_state_from_config(light_cfg.get("state", "red"))
    freeze = bool(light_cfg.get("freeze", mode == "freeze_all"))
    for light in lights:
        if state is not None:
            light.set_state(state)
        light.freeze(freeze)

    state_label = str(light_cfg.get("state", "unchanged")) if state is not None else "unchanged"
    print(
        "[TRAFFIC] "
        f"traffic_lights mode={mode}, count={len(lights)}, state={state_label}, freeze={freeze}"
    )


def _cleanup(actors: List[Optional[carla.Actor]]) -> None:
    for a in actors:
        try:
            if a is not None and a.is_alive:
                a.destroy()
        except Exception:
            pass


class ScenarioEvaluator:
    """
    Generic closed-loop evaluator for one scenario.

    Usage
    -----
    evaluator = ScenarioEvaluator(scenario_impl, cfg, config_path)
    evaluator.run()
    """

    def __init__(self, scenario, cfg: dict, config_path: Path):
        self.scenario = scenario
        self.cfg = cfg
        self.config_path = Path(config_path)

    def run(
        self,
        host: str = "localhost",
        port: int = 2000,
        timeout: float = 10.0,
        log_id: Optional[str] = None,
        spawn_index: Optional[int] = None,
        enable_cameras: bool = False,
        agent=None,
        draw_route: bool = False,
        draw_route_stride: int = 1,
        draw_route_lifetime: float = 600.0,
        draw_route_labels: bool = False,
        voice_overlay: bool = False,
        voice_match_config: str = "configs/lmdrive/route_audio_matches.yaml",
    ) -> Dict[str, Any]:
        """
        Run the scenario and return the final summary dict.

        Args:
            enable_cameras: if True, attach RGB cameras via ObservationBuilder
                            (needed when an LMDrive agent is connected)
        """
        cfg = self.cfg
        scenario_id = cfg.get("scenario_id", self.config_path.stem)
        category = cfg.get("category", "uncategorised")
        runtime_cfg = cfg.get("runtime", {})
        eval_cfg = cfg.get("evaluation", {})

        log_scenario_id = log_id or scenario_id
        out_dir = Path("logs") / category / log_scenario_id
        out_dir.mkdir(parents=True, exist_ok=True)
        log_path = out_dir / "frames.jsonl"

        dt = float(runtime_cfg.get("fixed_delta_seconds", 0.05))
        duration = float(runtime_cfg.get("max_duration_seconds", 120.0))
        post_success_hold = float(runtime_cfg.get("post_success_hold_seconds", 5.0))
        route_corridor_half = float(eval_cfg.get("route_corridor_half_width_m", 4.5))
        red_light_tol = float(eval_cfg.get("red_light_lane_tolerance_m", 5.5))
        primary_cmd_id = cfg.get("instructions", [{}])[0].get("id", "cmd_001")
        trigger_time = get_instruction_trigger_time(cfg, default=5.0)
        background_n = int(runtime_cfg.get("background_vehicles", 0))

        client = carla.Client(host, port)
        client.set_timeout(timeout)

        world = load_world_for_config(client, cfg)
        apply_weather_from_config(world, cfg)
        carla_map = world.get_map()
        blueprint_library = world.get_blueprint_library()

        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        world.apply_settings(settings)
        _apply_traffic_conditions(world, cfg)

        # ---------- resource handles ----------
        ego: Optional[carla.Actor] = None
        collision_sensor: Optional[carla.Actor] = None
        lane_invasion_tracker: Optional[LaneInvasionTracker] = None
        obs_builder: Optional[ObservationBuilder] = None
        voice_runtime: Optional[RouteAudioRuntime] = None
        voice_window: Optional[FixedVoiceOverlay] = None
        scenario_actors: List[carla.Actor] = []
        background_actors: List[carla.Actor] = []
        collision_info: Dict[str, Any] = {"value": False, "other_actor": None}

        try:
            # ---- clean up leftover actors ----
            for a in list(world.get_actors().filter("vehicle.*")) + \
                     list(world.get_actors().filter("walker.*")) + \
                     list(world.get_actors().filter("sensor.*")):
                a.destroy()
            world.tick()

            # ---- spawn ego ----
            vehicle_type = cfg.get("ego", {}).get("vehicle_type", "vehicle.tesla.model3")
            vehicle_bp = blueprint_library.find(vehicle_type)

            if spawn_index is not None:
                spawn_points = carla_map.get_spawn_points()
                sp = spawn_points[spawn_index % len(spawn_points)]
                sp.location.z += 0.5
            else:
                sp = make_lane_aligned_transform_from_config(carla_map, cfg)

            ego = world.try_spawn_actor(vehicle_bp, sp)
            if ego is None:
                raise RuntimeError("Failed to spawn ego vehicle.")

            for _ in range(20):
                world.tick()

            # ---- route tracker ----
            route_tracker = RouteTracker.from_route_config(
                carla_map, cfg, corridor_half_width_m=route_corridor_half
            )
            summary = route_debug_summary(carla_map, route_tracker.route_points)
            print(
                "[ROUTE] "
                f"mode={cfg.get('route', {}).get('mode', 'xml')} "
                f"points={summary['point_count']} "
                f"length={summary['total_length_m']}m "
                f"samples={summary['samples']}"
            )
            if draw_route:
                _draw_route_debug(
                    world,
                    route_tracker.route_points,
                    stride=draw_route_stride,
                    life_time=draw_route_lifetime,
                    with_labels=draw_route_labels,
                )

            if voice_overlay:
                try:
                    voice_runtime = RouteAudioRuntime(voice_match_config, scenario_id=scenario_id)
                    voice_window = FixedVoiceOverlay(
                        title=f"Voice Input - {scenario_id}",
                        geometry="1080x320+40+40",
                    )
                    print(
                        "[VOICE_OVERLAY] "
                        f"loaded {len(voice_runtime.events)} voice trigger(s) from {voice_match_config}"
                    )
                except Exception as exc:
                    print(f"[VOICE_OVERLAY] disabled: {exc}")
                    voice_runtime = None
                    voice_window = None

            red_light_tracker = RedLightViolationTracker(
                world,
                route_tracker.ref_loc,
                route_tracker.ref_forward,
                route_tracker.ref_right,
                lane_tolerance_m=red_light_tol,
            )

            # ---- sensors ----
            collision_bp = blueprint_library.find("sensor.other.collision")
            collision_sensor = world.spawn_actor(
                collision_bp, carla.Transform(), attach_to=ego
            )
            collision_sensor.listen(
                lambda e: collision_info.update(
                    {"value": True, "other_actor": e.other_actor.type_id}
                )
            )
            lane_invasion_tracker = LaneInvasionTracker(world, blueprint_library, ego)

            if enable_cameras:
                obs_builder = ObservationBuilder(world, ego)

            # ---- background traffic ----
            if background_n > 0:
                bg_bp = blueprint_library.filter("vehicle.*")
                spawn_pts = carla_map.get_spawn_points()
                import random
                random.shuffle(spawn_pts)
                for pt in spawn_pts[:background_n]:
                    bp = random.choice(bg_bp)
                    a = world.try_spawn_actor(bp, pt)
                    if a is not None:
                        a.set_autopilot(True)
                        background_actors.append(a)
                world.tick()

            # ---- scenario-specific actors ----
            scenario_actors = self.scenario.spawn_actors(world, ego, cfg) or []
            for _ in range(10):
                world.tick()

            # ---- initial state ----
            state = self.scenario.initial_state(ego, scenario_actors, cfg)

            max_frames = int(duration / dt)
            success_time: Optional[float] = None

            print(f"[START] {log_scenario_id} | map={carla_map.name} | dt={dt}s | max={duration}s")

            with log_path.open("w", encoding="utf-8") as log_f:
                for frame in range(max_frames):
                    timestamp = frame * dt

                    # ---- observation ----
                    ego_loc = ego.get_location()
                    speed_kmh = _get_speed_kmh(ego)
                    route_metrics_pre = route_tracker.measure(ego_loc)
                    voice_event = None
                    voice_triggered = False
                    if voice_runtime is not None:
                        voice_event, voice_triggered = voice_runtime.update(
                            route_metrics_pre["route_progress_m"],
                            timestamp,
                        )
                        if voice_triggered:
                            voice = voice_event.get("voice", {}) if voice_event else {}
                            print(
                                "[VOICE_TRIGGER] "
                                f"t={timestamp:.1f}s "
                                f"progress={route_metrics_pre['route_progress_m']:.1f}m "
                                f"audio={voice_event.get('audio_id') if voice_event else None} "
                                f"event={voice_event.get('event_id') if voice_event else None} "
                                f"text={voice.get('input_text', '')}"
                            )
                    if voice_window is not None:
                        voice_window.update(
                            voice_event,
                            route_metrics_pre["route_progress_m"],
                        )

                    obs: Dict[str, Any] = {
                        "ego_loc": ego_loc,
                        "speed_kmh": speed_kmh,
                        "route_metrics": route_metrics_pre,
                        "route_tracker": route_tracker,
                        "timestamp": timestamp,
                    }
                    if obs_builder is not None:
                        obs.update(obs_builder.get())

                    # ---- scenario control ----
                    state = self.scenario.update_state(
                        ego, scenario_actors, state, obs, dt, cfg
                    )
                    if agent is not None:
                        throttle, brake, steer = agent.run_step(
                            {
                                "scenario": self.scenario,
                                "ego": ego,
                                "actors": scenario_actors,
                                "state": state,
                                "obs": obs,
                                "cfg": cfg,
                            },
                            timestamp,
                            instruction=cfg.get("instructions", [{}])[0],
                        )
                    else:
                        throttle, brake, steer = self.scenario.compute_control(
                            ego, scenario_actors, state, obs, cfg
                        )

                    control = carla.VehicleControl(
                        throttle=float(throttle),
                        brake=float(brake),
                        steer=float(steer),
                        hand_brake=False,
                        reverse=False,
                    )
                    ego.apply_control(control)
                    world.tick()

                    # ---- post-tick metrics ----
                    ego_loc = ego.get_location()
                    speed_kmh = _get_speed_kmh(ego)
                    lane_metrics = lane_invasion_tracker.snapshot()
                    route_metrics = route_tracker.measure(ego_loc)
                    red_metrics = red_light_tracker.update(ego_loc, speed_kmh)

                    _update_spectator(world, ego)

                    # ---- base record ----
                    instruction_active = timestamp >= trigger_time
                    record: Dict[str, Any] = {
                        "timestamp": timestamp,
                        "frame": frame,
                        "scenario_id": log_scenario_id,
                        "instruction_id": primary_cmd_id if instruction_active else None,
                        "ego_x": ego_loc.x,
                        "ego_y": ego_loc.y,
                        "ego_z": ego_loc.z,
                        "ego_speed_kmh": speed_kmh,
                        "steer": control.steer,
                        "throttle": control.throttle,
                        "brake": control.brake,
                        "collision": collision_info["value"],
                        "collision_other_actor": collision_info["other_actor"],
                        "lane_invasion": lane_metrics["lane_invasion"],
                        "crossed_lane_markings": lane_metrics["crossed_lane_markings"],
                        "red_light_violation": red_metrics["red_light_violation"],
                        "active_traffic_light_id": red_metrics["active_traffic_light_id"],
                        "active_traffic_light_state": red_metrics["active_traffic_light_state"],
                        "active_stop_line_progress_m": red_metrics["active_stop_line_progress_m"],
                        "route_deviation": route_metrics["route_deviation"],
                        "route_progress_m": route_metrics["route_progress_m"],
                        "max_route_progress_m": route_metrics["max_route_progress_m"],
                        "route_total_length_m": route_metrics["route_total_length_m"],
                        "route_completion": route_metrics["route_completion"],
                        "lateral_offset_from_route_m": route_metrics["lateral_offset_from_route_m"],
                        "on_driving_lane": route_metrics["on_driving_lane"],
                        "asr_latency_ms": 0.0,
                        "parser_latency_ms": 0.0,
                        "model_latency_ms": 0.0,
                        "end_to_end_latency_ms": 0.0,
                    }
                    if voice_event is not None:
                        voice = voice_event.get("voice", {})
                        record.update({
                            "voice_audio_id": voice_event.get("audio_id"),
                            "voice_event_id": voice_event.get("event_id"),
                            "voice_text": voice.get("input_text") or voice.get("normalized_text"),
                            "voice_intents": voice.get("recognized_intents"),
                            "voice_triggered": voice_triggered,
                            "voice_trigger_progress_m": voice_event.get("voice_trigger_progress_m"),
                            "voice_trigger_timestamp": voice_event.get("voice_trigger_timestamp"),
                        })
                    else:
                        record.update({
                            "voice_audio_id": None,
                            "voice_event_id": None,
                            "voice_text": None,
                            "voice_intents": None,
                            "voice_triggered": False,
                            "voice_trigger_progress_m": None,
                            "voice_trigger_timestamp": None,
                        })
                    # scenario-specific fields
                    record.update(
                        self.scenario.extra_record(ego, scenario_actors, state, obs, cfg)
                    )

                    log_f.write(json.dumps(record, ensure_ascii=False) + "\n")

                    if frame % 20 == 0:
                        print(
                            f"  t={timestamp:.1f}s  spd={speed_kmh:.1f}km/h  "
                            f"col={collision_info['value']}  "
                            f"prog={route_metrics['route_completion']:.2%}"
                        )

                    # ---- success / failure ----
                    if success_time is None and self.scenario.is_success(state, obs, cfg):
                        success_time = timestamp
                        print(f"[SUCCESS] at t={timestamp:.1f}s, hold {post_success_hold}s")

                    if success_time is not None and timestamp >= success_time + post_success_hold:
                        break

                    if collision_info["value"] and timestamp > 2.0:
                        print(f"[FAIL] collision with {collision_info['other_actor']}")
                        break

            print(f"[DONE] log -> {log_path}")
            return {
                "scenario_id": log_scenario_id,
                "success": success_time is not None,
                "success_time": success_time,
                "collision": collision_info["value"],
                "route_completion": route_metrics["route_completion"],
                "log_path": str(log_path),
            }

        finally:
            if agent is not None:
                try:
                    agent.destroy()
                except Exception:
                    pass
            if obs_builder is not None:
                obs_builder.destroy()
            if voice_window is not None:
                voice_window.destroy()
            if lane_invasion_tracker is not None:
                lane_invasion_tracker.destroy()
            _cleanup([collision_sensor] + scenario_actors + background_actors + [ego])
            world.apply_settings(original_settings)
            print("[CLEANUP] done")


def load_scenario_config(config_path: str) -> dict:
    """Load a scenario YAML and inject __config_path__ for path resolution."""
    p = Path(config_path)
    with p.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["__config_path__"] = str(p.resolve())
    return cfg
