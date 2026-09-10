"""Official Leaderboard 2.0 agent entry point for the Dongfeng project.

The evaluator owns the simulation. A real SimLingo policy is supplied through
``DONGFENG_POLICY_MODULE`` and must expose ``build_agent(config_path)``.
Without that policy this agent fails explicitly instead of silently driving
with a test controller.
"""

from __future__ import annotations

import importlib
import os
from pathlib import Path

import carla
from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track


def get_entry_point():
    return "DongfengAgent"


class DongfengAgent(AutonomousAgent):
    def setup(self, path_to_conf_file):
        self.track = Track.SENSORS
        self._config_path = str(path_to_conf_file or "")
        module_name = os.environ.get("DONGFENG_POLICY_MODULE", "").strip()
        if not module_name:
            raise RuntimeError(
                "DONGFENG_POLICY_MODULE is not set. "
                "Set it to the SimLingo/agent policy adapter module."
            )
        module = importlib.import_module(module_name)
        factory = getattr(module, "build_agent", None)
        if factory is None:
            raise RuntimeError(
                f"{module_name} must expose build_agent(config_path)"
            )
        self._policy = factory(self._config_path)

    def sensors(self):
        # The official SENSORS track accepts these CARLA sensor definitions.
        return [
            {
                "type": "sensor.camera.rgb",
                "x": 0.7, "y": 0.0, "z": 1.60,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                "width": 1200, "height": 900, "fov": 100,
                "id": "Center",
            },
            {
                "type": "sensor.camera.rgb",
                "x": 0.7, "y": -0.4, "z": 1.60,
                "roll": 0.0, "pitch": 0.0, "yaw": -45.0,
                "width": 400, "height": 300, "fov": 100,
                "id": "Left",
            },
            {
                "type": "sensor.camera.rgb",
                "x": 0.7, "y": 0.4, "z": 1.60,
                "roll": 0.0, "pitch": 0.0, "yaw": 45.0,
                "width": 400, "height": 300, "fov": 100,
                "id": "Right",
            },
            {
                "type": "sensor.other.gnss",
                "x": 0.7, "y": -0.4, "z": 1.60,
                "id": "GPS",
            },
            {
                "type": "sensor.other.imu",
                "x": 0.7, "y": -0.4, "z": 1.60,
                "roll": 0.0, "pitch": 0.0, "yaw": -45.0,
                "id": "IMU",
            },
            {
                "type": "sensor.speedometer",
                "reading_frequency": 20,
                "id": "Speed",
            },
        ]

    def run_step(self, input_data, timestamp):
        output = self._policy.run_step(input_data, timestamp, self._global_plan_world_coord)
        if isinstance(output, carla.VehicleControl):
            return output
        if isinstance(output, dict):
            control = carla.VehicleControl()
            control.throttle = float(output.get("throttle", 0.0))
            control.brake = float(output.get("brake", 0.0))
            control.steer = float(output.get("steer", 0.0))
            return control
        raise TypeError(
            "SimLingo policy must return carla.VehicleControl or "
            "{throttle, brake, steer}"
        )

    def destroy(self):
        policy = getattr(self, "_policy", None)
        if policy is not None and hasattr(policy, "destroy"):
            policy.destroy()
