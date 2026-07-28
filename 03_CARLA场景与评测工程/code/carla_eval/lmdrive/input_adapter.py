"""Convert Dongfeng runner observations to LMDrive-style agent inputs.

The target field names follow LMDrive's official ``lmdriver_agent.py``:
``rgb_front``, ``rgb_left``, ``rgb_right``, ``rgb_rear``, ``lidar``,
``num_points``, ``velocity``, ``target_point`` and ``text_input``.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional, Tuple

import numpy as np

from carla_eval.agents.base_agent import BaseAgent


def rotate_lidar(lidar: np.ndarray, angle_deg: float) -> np.ndarray:
    """Rotate ``N x 4`` LiDAR points in the x/y plane."""
    radian = np.deg2rad(angle_deg)
    rot = np.array(
        [
            [np.cos(radian), np.sin(radian), 0.0, 0.0],
            [-np.sin(radian), np.cos(radian), 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    return lidar @ rot


def lidar_to_raw_features(lidar: np.ndarray, max_points: int = 40000) -> Tuple[np.ndarray, int]:
    """Match LMDrive's padded LiDAR tensor preprocessing.

    Input may be ``N x 3`` or ``N x 4``. Output is always ``max_points x 4``.
    Points inside the ego body box are removed, then the cloud is padded/truncated
    and rotated by -90 degrees, mirroring LMDrive's official helper.
    """
    if lidar is None:
        lidar_xyzi = np.zeros((0, 4), dtype=np.float32)
    else:
        lidar_xyzi = np.asarray(lidar, dtype=np.float32)
        if lidar_xyzi.size == 0:
            lidar_xyzi = np.zeros((0, 4), dtype=np.float32)
        elif lidar_xyzi.ndim != 2:
            lidar_xyzi = lidar_xyzi.reshape(-1, lidar_xyzi.shape[-1])
        if lidar_xyzi.shape[1] < 3:
            lidar_xyzi = np.zeros((0, 4), dtype=np.float32)
        elif lidar_xyzi.shape[1] == 3:
            intensity = np.zeros((lidar_xyzi.shape[0], 1), dtype=np.float32)
            lidar_xyzi = np.concatenate([lidar_xyzi, intensity], axis=1)
        elif lidar_xyzi.shape[1] > 4:
            lidar_xyzi = lidar_xyzi[:, :4]

    if lidar_xyzi.size:
        ego_box = (
            (lidar_xyzi[:, 0] > -1.2)
            & (lidar_xyzi[:, 0] < 1.2)
            & (lidar_xyzi[:, 1] > -1.2)
            & (lidar_xyzi[:, 1] < 1.2)
        )
        lidar_xyzi = lidar_xyzi[~ego_box]

    output = np.zeros((max_points, 4), dtype=np.float32)
    num_points = min(max_points, len(lidar_xyzi))
    if num_points:
        output[:num_points, :4] = lidar_xyzi[:num_points, :4]
    output[np.isinf(output)] = 0.0
    output[np.isnan(output)] = 0.0
    return rotate_lidar(output, -90.0).astype(np.float32), int(num_points)


def _bgr_to_rgb(image: Optional[np.ndarray], shape: Tuple[int, int, int]) -> np.ndarray:
    if image is None:
        return np.zeros(shape, dtype=np.uint8)
    arr = np.asarray(image)
    if arr.ndim != 3 or arr.shape[2] < 3:
        return np.zeros(shape, dtype=np.uint8)
    rgb = arr[:, :, :3][:, :, ::-1]
    return np.ascontiguousarray(rgb, dtype=np.uint8)


def _center_crop_front(rgb_front: np.ndarray) -> np.ndarray:
    height, width = rgb_front.shape[:2]
    crop_h = min(240, height)
    crop_w = min(240, width)
    y0 = max(0, (height - crop_h) // 2)
    x0 = max(0, (width - crop_w) // 2)
    return np.ascontiguousarray(rgb_front[y0 : y0 + crop_h, x0 : x0 + crop_w])


def _instruction_text(instruction: Optional[Dict[str, Any]]) -> str:
    if not instruction:
        return "Drive safely."
    return (
        instruction.get("text_en")
        or instruction.get("text_zh")
        or instruction.get("text")
        or "Drive safely."
    )


def _route_hint(input_data: Dict[str, Any], lookahead_m: float) -> Dict[str, Any]:
    ego = input_data["ego"]
    obs = input_data["obs"]
    route_metrics = obs.get("route_metrics", {})
    route_tracker = obs.get("route_tracker")
    progress = float(route_metrics.get("route_progress_m", 0.0))

    ego_tf = ego.get_transform()
    ego_loc = ego_tf.location
    forward = ego_tf.get_forward_vector()
    right = ego_tf.get_right_vector()

    target_loc = ego_loc
    if route_tracker is not None:
        target_loc = route_tracker.point_at_progress(progress + lookahead_m)

    dx = target_loc.x - ego_loc.x
    dy = target_loc.y - ego_loc.y
    local_forward = dx * forward.x + dy * forward.y
    local_right = dx * right.x + dy * right.y

    return {
        "route_progress_m": progress,
        "max_route_progress_m": float(route_metrics.get("max_route_progress_m", progress)),
        "route_total_length_m": float(route_metrics.get("route_total_length_m", 0.0)),
        "route_completion": float(route_metrics.get("route_completion", 0.0)),
        "target_world": [float(target_loc.x), float(target_loc.y), float(target_loc.z)],
        "target_point": np.array([local_forward, local_right], dtype=np.float32),
        "target_point_convention": "ego_frame_forward_right_m",
    }


class LMDriveInputAdapter:
    """Build the LMDrive-style tick/input dictionary from evaluator input data."""

    def __init__(self, route_lookahead_m: float = 30.0, max_lidar_points: int = 40000):
        self.route_lookahead_m = float(route_lookahead_m)
        self.max_lidar_points = int(max_lidar_points)

    def build(
        self,
        input_data: Dict[str, Any],
        timestamp: float,
        instruction: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        obs = input_data["obs"]
        ego = input_data["ego"]
        speed_kmh = float(obs.get("speed_kmh", 0.0))
        speed_mps = speed_kmh / 3.6
        route_hint = _route_hint(input_data, self.route_lookahead_m)

        rgb_front = _bgr_to_rgb(obs.get("rgb_front"), (900, 1200, 3))
        rgb_left = _bgr_to_rgb(obs.get("rgb_left"), (300, 400, 3))
        rgb_right = _bgr_to_rgb(obs.get("rgb_right"), (300, 400, 3))
        rgb_rear = _bgr_to_rgb(obs.get("rgb_rear"), (300, 400, 3))
        rgb_center = _center_crop_front(rgb_front)

        raw_lidar = obs.get("lidar")
        lidar, num_points = lidar_to_raw_features(raw_lidar, max_points=self.max_lidar_points)

        ego_tf = ego.get_transform()
        yaw_rad = math.radians(float(ego_tf.rotation.yaw))
        gps = np.array([float(ego_tf.location.x), float(ego_tf.location.y)], dtype=np.float32)

        return {
            "rgb_front": rgb_front,
            "rgb_left": rgb_left,
            "rgb_right": rgb_right,
            "rgb_rear": rgb_rear,
            "rgb_center": rgb_center,
            "raw_lidar": raw_lidar,
            "lidar": lidar,
            "num_points": num_points,
            "gps": gps,
            "compass": yaw_rad,
            "speed": speed_mps,
            "velocity": np.array([[speed_mps]], dtype=np.float32),
            "measurements": np.array([gps[0], gps[1], yaw_rad, speed_mps], dtype=np.float32),
            "target_point": route_hint["target_point"],
            "route_hint": route_hint,
            "next_command": None,
            "text_input": [_instruction_text(instruction)],
            "instruction": instruction or {},
            "timestamp": float(timestamp),
        }


class LMDriveAgentAdapter(BaseAgent):
    """Adapter shell for plugging a real LMDrive policy into this runner.

    ``policy`` can be any object exposing ``run_step(lmdrive_input, timestamp)`` or
    ``predict(lmdrive_input)`` and returning either a dict/control object or a
    ``(throttle, brake, steer)`` tuple.
    """

    def __init__(self, policy: Optional[Any] = None, adapter: Optional[LMDriveInputAdapter] = None):
        self.policy = policy
        self.input_adapter = adapter or LMDriveInputAdapter()
        self.last_lmdrive_input: Optional[Dict[str, Any]] = None

    def sensors(self):
        from carla_eval.sensors.observation_builder import ObservationBuilder

        return ObservationBuilder.SENSOR_SPECS

    def run_step(
        self,
        input_data: Dict[str, Any],
        timestamp: float,
        instruction: Optional[Dict[str, Any]] = None,
    ) -> Tuple[float, float, float]:
        start = time.time()
        lmdrive_input = self.input_adapter.build(input_data, timestamp, instruction)
        self.last_lmdrive_input = lmdrive_input

        if self.policy is None:
            raise RuntimeError("LMDriveAgentAdapter requires a real LMDrive policy object.")

        if hasattr(self.policy, "run_step"):
            output = self.policy.run_step(lmdrive_input, timestamp)
        elif hasattr(self.policy, "predict"):
            output = self.policy.predict(lmdrive_input)
        else:
            raise TypeError("policy must provide run_step(...) or predict(...)")

        control = self._normalize_control(output)
        self.model_latency_ms = 1000.0 * (time.time() - start)
        return control

    @staticmethod
    def _normalize_control(output: Any) -> Tuple[float, float, float]:
        if isinstance(output, tuple) and len(output) == 3:
            return float(output[0]), float(output[1]), float(output[2])
        if isinstance(output, dict):
            return (
                float(output.get("throttle", 0.0)),
                float(output.get("brake", 0.0)),
                float(output.get("steer", 0.0)),
            )
        if all(hasattr(output, name) for name in ("throttle", "brake", "steer")):
            return float(output.throttle), float(output.brake), float(output.steer)
        raise TypeError(f"Unsupported LMDrive policy output: {type(output)!r}")
