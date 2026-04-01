"""Parse kalibr_imu_chain.yaml into ImuConfig."""
from __future__ import annotations

from pathlib import Path

import numpy as np
from ruamel.yaml import YAML

from lib.models import ImuConfig, OutlierConfig, ProcessingError


def load_imu_config(config_path: Path) -> ImuConfig:
    if not config_path.exists():
        raise ProcessingError(f"IMU config file not found: {config_path}")

    raw_text = config_path.read_text(encoding="utf-8")
    lines = raw_text.splitlines()
    if lines and lines[0].startswith("%YAML:"):
        raw_text = "\n".join(lines[1:])

    yaml = YAML(typ="safe")
    data = yaml.load(raw_text)
    if not isinstance(data, dict) or "imu0" not in data:
        raise ProcessingError(f"Missing 'imu0' section in IMU config: {config_path}")

    imu0 = data["imu0"]
    required_keys = (
        "accelerometer_noise_density",
        "accelerometer_random_walk",
        "gyroscope_noise_density",
        "gyroscope_random_walk",
        "update_rate",
        "T_i_b",
    )
    for key in required_keys:
        if key not in imu0:
            raise ProcessingError(f"Missing key 'imu0.{key}' in IMU config: {config_path}")

    t_i_b = np.array(imu0["T_i_b"], dtype=np.float64)
    if t_i_b.shape != (4, 4):
        raise ProcessingError(
            f"Expected imu0.T_i_b to be 4x4 in {config_path}, got {t_i_b.shape}."
        )

    of_raw = imu0.get("outlier_filter", {})
    outlier_config = OutlierConfig(
        enabled=bool(of_raw.get("enabled", True)),
        threshold_sigma=float(of_raw.get("threshold_sigma", 3.0)),
        method=str(of_raw.get("method", "mad")),
    )

    body_frame = str(imu0.get("body_frame", "kinect"))
    imu_frame = str(imu0.get("imu_frame", "imu"))
    imu_frame_mode = bool(imu0.get("imu_frame_mode", False))

    return ImuConfig(
        accelerometer_noise_density=float(imu0["accelerometer_noise_density"]),
        accelerometer_random_walk=float(imu0["accelerometer_random_walk"]),
        gyroscope_noise_density=float(imu0["gyroscope_noise_density"]),
        gyroscope_random_walk=float(imu0["gyroscope_random_walk"]),
        update_rate=float(imu0["update_rate"]),
        T_i_b=t_i_b[:3, :3],
        outlier_filter=outlier_config,
        body_frame=body_frame,
        imu_frame=imu_frame,
        imu_frame_mode=imu_frame_mode,
    )
