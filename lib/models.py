"""Shared data models, constants, and exceptions."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

GRAVITY_WORLD = np.array([0.0, 0.0, 9.81], dtype=np.float64)

TF_MSGTYPE = "tf2_msgs/msg/TFMessage"
IMU_MSGTYPE = "sensor_msgs/msg/Imu"

PREFERRED_CHILD_FRAMES = (
    "kinect",
    "camera_rgb_optical_frame",
    "camera",
)


@dataclass(frozen=True)
class PoseSample:
    timestamp_ns: int
    position: np.ndarray  # shape: (3,)
    quat_xyzw: np.ndarray  # shape: (4,)
    child_frame_id: str


@dataclass(frozen=True)
class ImuSample:
    timestamp_ns: int
    quat_xyzw: np.ndarray
    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray


@dataclass(frozen=True)
class OutlierConfig:
    enabled: bool
    threshold_sigma: float  # rejection threshold in MAD-sigma units
    method: str             # "mad" or "iqr"


@dataclass(frozen=True)
class ImuConfig:
    accelerometer_noise_density: float
    accelerometer_random_walk: float
    gyroscope_noise_density: float
    gyroscope_random_walk: float
    update_rate: float
    T_i_b: np.ndarray
    outlier_filter: OutlierConfig
    body_frame: str
    imu_frame: str
    imu_frame_mode: bool
    rostopic: str


class ProcessingError(RuntimeError):
    """Raised when a bag cannot be processed due to missing or invalid data."""
