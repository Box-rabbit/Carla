"""
ObservationBuilder: attach cameras and LiDAR to ego, collect sensor data
each tick into a structured observation dict for VLA model consumption.

Sensor layout mirrors lmdriver_agent.py from LMDrive:
  - rgb_front : 1200×900, FOV 100°, x=1.3 z=2.3 yaw=0
  - rgb_left  :  400×300, FOV 100°, x=1.3 z=2.3 yaw=-60
  - rgb_right :  400×300, FOV 100°, x=1.3 z=2.3 yaw=+60
  - rgb_rear  :  400×300, FOV 100°, x=-1.3 z=2.3 yaw=180
  - lidar     : ray_cast, x=1.3 z=2.5 yaw=-90

Only instantiated when enable_cameras=True in ScenarioEvaluator.
"""

from __future__ import annotations

import queue
from typing import Any, Dict, List, Optional

import carla
import numpy as np


def _make_camera_transform(x: float, y: float, z: float, yaw: float) -> carla.Transform:
    return carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(yaw=yaw),
    )


class ObservationBuilder:
    """
    Attach sensors to ego and provide a get() method that returns the latest
    observation dict suitable for an LMDrive-compatible agent.
    """

    SENSOR_SPECS = [
        dict(id="rgb_front",  type="sensor.camera.rgb", w=1200, h=900,  fov=100, x=1.3, y=0.0, z=2.3, yaw=0.0),
        dict(id="rgb_left",   type="sensor.camera.rgb", w=400,  h=300,  fov=100, x=1.3, y=0.0, z=2.3, yaw=-60.0),
        dict(id="rgb_right",  type="sensor.camera.rgb", w=400,  h=300,  fov=100, x=1.3, y=0.0, z=2.3, yaw=60.0),
        dict(id="rgb_rear",   type="sensor.camera.rgb", w=400,  h=300,  fov=100, x=-1.3, y=0.0, z=2.3, yaw=180.0),
        dict(id="lidar",      type="sensor.lidar.ray_cast",            x=1.3, y=0.0, z=2.5, yaw=-90.0),
    ]

    def __init__(self, world: carla.World, ego: carla.Actor):
        self._sensors: List[carla.Actor] = []
        self._queues: Dict[str, queue.Queue] = {}
        self._latest: Dict[str, Any] = {}

        bp_lib = world.get_blueprint_library()

        for spec in self.SENSOR_SPECS:
            bp = bp_lib.find(spec["type"])
            if "w" in spec:
                bp.set_attribute("image_size_x", str(spec["w"]))
                bp.set_attribute("image_size_y", str(spec["h"]))
                bp.set_attribute("fov", str(spec["fov"]))

            tf = _make_camera_transform(
                spec["x"], spec["y"], spec["z"], spec["yaw"]
            )
            sensor = world.spawn_actor(bp, tf, attach_to=ego)
            sid = spec["id"]
            self._queues[sid] = queue.Queue(maxsize=2)
            self._sensors.append(sensor)

            if spec["type"] == "sensor.camera.rgb":
                sensor.listen(lambda data, s=sid: self._on_image(s, data))
            else:
                sensor.listen(lambda data, s=sid: self._on_lidar(s, data))

    def _on_image(self, sid: str, data: carla.Image) -> None:
        arr = np.frombuffer(data.raw_data, dtype=np.uint8)
        arr = arr.reshape((data.height, data.width, 4))[:, :, :3]  # BGRA → BGR
        self._latest[sid] = arr
        if not self._queues[sid].full():
            self._queues[sid].put_nowait(arr)

    def _on_lidar(self, sid: str, data: carla.LidarMeasurement) -> None:
        pts = np.frombuffer(data.raw_data, dtype=np.float32)
        pts = pts.reshape(-1, 4)   # x,y,z,intensity
        self._latest[sid] = pts
        if not self._queues[sid].full():
            self._queues[sid].put_nowait(pts)

    def get(self) -> Dict[str, Any]:
        """Return the most recent sensor readings (non-blocking)."""
        return dict(self._latest)

    def destroy(self) -> None:
        for s in self._sensors:
            try:
                if s.is_alive:
                    s.destroy()
            except Exception:
                pass
        self._sensors.clear()
