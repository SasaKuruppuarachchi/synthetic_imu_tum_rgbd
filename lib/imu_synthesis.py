"""IMU kinematic synthesis: differentiation, outlier removal, noise model."""
from __future__ import annotations

import hashlib
from collections import Counter
from typing import Sequence

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import RotationSpline

from lib.models import (
    GRAVITY_WORLD,
    PREFERRED_CHILD_FRAMES,
    ImuConfig,
    ImuSample,
    OutlierConfig,
    PoseSample,
    ProcessingError,
)


def stable_seed_from_name(name: str) -> int:
    digest = hashlib.sha256(name.encode("utf-8")).digest()
    return int.from_bytes(digest[:8], byteorder="little", signed=False) % (2**32)


def normalize_frame(frame_id: str) -> str:
    return frame_id.strip().lstrip("/")


def choose_child_frame(samples: Sequence[PoseSample], config_frame: str) -> str:
    candidates = [normalize_frame(sample.child_frame_id) for sample in samples]
    if not candidates:
        raise ProcessingError("No TF transform samples available.")

    counts = Counter(candidates)

    if config_frame in counts:
        return config_frame

    for preferred in PREFERRED_CHILD_FRAMES:
        if preferred in counts:
            return preferred

    fuzzy = [name for name in counts if "kinect" in name or "camera" in name]
    if fuzzy:
        return max(fuzzy, key=lambda x: counts[x])

    return counts.most_common(1)[0][0]


def deduplicate_by_timestamp(samples: Sequence[PoseSample]) -> list[PoseSample]:
    by_ts: dict[int, PoseSample] = {}
    for sample in samples:
        by_ts[sample.timestamp_ns] = sample
    return [by_ts[ts] for ts in sorted(by_ts)]


def apply_imu_noise(
    angular_velocity: np.ndarray,
    linear_acceleration: np.ndarray,
    imu_config: ImuConfig,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """Apply Kalibr white-noise and bias random-walk model to IMU signals."""
    rate = imu_config.update_rate

    # Kalibr discrete-time noise model (https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model):
    #   White noise:    σ_d = σ_c / √Δt = σ_c · √rate
    #   Bias rand walk: σ_b_d = σ_b_c · √Δt = σ_b_c / √rate
    dt = 1.0 / rate
    gyro_noise_std  = imu_config.gyroscope_noise_density     / np.sqrt(dt)
    accel_noise_std = imu_config.accelerometer_noise_density / np.sqrt(dt)
    gyro_bias_std   = imu_config.gyroscope_random_walk       * np.sqrt(dt)
    accel_bias_std  = imu_config.accelerometer_random_walk   * np.sqrt(dt)

    gyro_bias  = np.zeros(3, dtype=np.float64)
    accel_bias = np.zeros(3, dtype=np.float64)

    noisy_gyro  = np.empty_like(angular_velocity)
    noisy_accel = np.empty_like(linear_acceleration)

    for i in range(angular_velocity.shape[0]):
        gyro_bias  += gyro_bias_std  * rng.standard_normal(3)
        accel_bias += accel_bias_std * rng.standard_normal(3)
        noisy_gyro[i]  = angular_velocity[i]   + gyro_bias  + gyro_noise_std  * rng.standard_normal(3)
        noisy_accel[i] = linear_acceleration[i] + accel_bias + accel_noise_std * rng.standard_normal(3)

    return noisy_gyro, noisy_accel, gyro_noise_std, accel_noise_std


def remove_outliers(signal: np.ndarray, config: OutlierConfig) -> np.ndarray:
    """Detect and linearly interpolate over outlier samples per axis (MAD or IQR)."""
    if not config.enabled:
        return signal

    result = signal.copy()
    indices = np.arange(signal.shape[0])

    for col in range(signal.shape[1]):
        x = signal[:, col]

        if config.method == "iqr":
            q25, q75 = np.percentile(x, [25, 75])
            iqr = q75 - q25
            if iqr == 0.0:
                continue
            half_range = config.threshold_sigma * iqr / 1.35
            median = np.median(x)
            mask_bad = np.abs(x - median) > half_range
        else:
            median = np.median(x)
            mad = np.median(np.abs(x - median))
            if mad == 0.0:
                continue
            threshold = config.threshold_sigma * mad / 0.6745
            mask_bad = np.abs(x - median) > threshold

        if not np.any(mask_bad):
            continue

        good_idx = indices[~mask_bad]
        bad_idx  = indices[mask_bad]

        if good_idx.size < 2:
            continue

        result[bad_idx, col] = np.interp(bad_idx, good_idx, x[good_idx])

    return result


def smooth_signal(signal: np.ndarray, window_size: int) -> np.ndarray:
    """Apply a centered moving-average filter per axis with edge padding."""
    if window_size <= 1:
        return signal
    if window_size % 2 == 0:
        window_size += 1

    kernel = np.ones(window_size, dtype=np.float64) / float(window_size)
    pad = window_size // 2
    smoothed = np.empty_like(signal)

    for col in range(signal.shape[1]):
        padded = np.pad(signal[:, col], (pad, pad), mode="edge")
        smoothed[:, col] = np.convolve(padded, kernel, mode="valid")

    return smoothed


def synthesize_imu_samples(
    pose_samples: Sequence[PoseSample],
    bag_start_ns: int,
    bag_end_ns: int,
    imu_config: ImuConfig,
    rng: np.random.Generator,
) -> list[ImuSample]:
    """Full kinematic pipeline: interpolate pose, differentiate at IMU rate, add noise."""
    if bag_end_ns <= bag_start_ns:
        raise ProcessingError("Invalid bag duration: end timestamp is not after start.")

    unique_samples = deduplicate_by_timestamp(pose_samples)
    if len(unique_samples) < 2:
        raise ProcessingError("Need at least 2 pose samples for interpolation.")

    ts_gt_ns = np.array([s.timestamp_ns for s in unique_samples], dtype=np.int64)
    t_gt = ts_gt_ns.astype(np.float64) * 1e-9
    if np.any(np.diff(t_gt) <= 0.0):
        raise ProcessingError("Pose timestamps must be strictly increasing.")

    pos_gt   = np.stack([s.position   for s in unique_samples], axis=0)
    quats_gt = np.stack([s.quat_xyzw  for s in unique_samples], axis=0)

    for i in range(1, quats_gt.shape[0]):
        if np.dot(quats_gt[i], quats_gt[i - 1]) < 0.0:
            quats_gt[i] = -quats_gt[i]

    rot_gt = R.from_quat(quats_gt)
    pos_cs = CubicSpline(t_gt, pos_gt, axis=0)
    rot_spline = RotationSpline(t_gt, rot_gt)

    dt_s  = 1.0 / imu_config.update_rate
    dt_ns = int(round(dt_s * 1e9))

    imu_start_ns = max(bag_start_ns, int(ts_gt_ns[0]))
    imu_end_ns = min(bag_end_ns, int(ts_gt_ns[-1]))
    if imu_end_ns <= imu_start_ns:
        raise ProcessingError("No overlap between bag duration and TF pose range.")

    imu_ts_ns = np.arange(imu_start_ns, imu_end_ns + 1, dt_ns, dtype=np.int64)
    if imu_ts_ns.size < 2:
        imu_ts_ns = np.array([imu_start_ns, imu_end_ns], dtype=np.int64)

    imu_t_s = imu_ts_ns.astype(np.float64) * 1e-9
    rot_imu = rot_spline(imu_t_s)
    quats_xyzw = rot_imu.as_quat()

    accel_world = pos_cs(imu_t_s, 2)
    accel_world = remove_outliers(accel_world, imu_config.outlier_filter)
    accel_smooth_window = max(3, int(round(0.10 * imu_config.update_rate)))
    accel_world = smooth_signal(accel_world, accel_smooth_window)

    accel_body = np.zeros_like(accel_world)
    for i in range(imu_t_s.shape[0]):
        accel_body[i] = rot_imu[i].inv().apply(accel_world[i] + GRAVITY_WORLD)

    omega_body = np.zeros((imu_t_s.shape[0], 3), dtype=np.float64)
    if imu_t_s.shape[0] >= 2:
        for i in range(1, imu_t_s.shape[0]):
            dt_i = imu_t_s[i] - imu_t_s[i - 1]
            delta_r = rot_imu[i - 1].inv() * rot_imu[i]
            omega_body[i] = delta_r.as_rotvec() / dt_i
        omega_body[0] = omega_body[1]
    omega_body = remove_outliers(omega_body, imu_config.outlier_filter)
    omega_smooth_window = max(3, int(round(0.05 * imu_config.update_rate)))
    omega_body = smooth_signal(omega_body, omega_smooth_window)

    if imu_config.imu_frame_mode:
        accel_imu = accel_body
        omega_imu = omega_body
    else:
        R_ib = imu_config.T_i_b   # (3,3) rotation: body -> IMU
        r_ib = imu_config.r_i_b   # (3,) lever arm in body frame

        # Angular acceleration via central differences (uniform IMU-rate grid)
        alpha_body = np.gradient(omega_body, imu_t_s, axis=0)  # shape (N, 3)

        # Full rigid-body lever-arm kinematics (vectorised):
        #   f_i = R_ib @ (f_b + alpha_b x r_ib + omega_b x (omega_b x r_ib))
        coriolis = np.cross(alpha_body, r_ib)  # (N, 3) tangential
        centripetal = np.cross(omega_body, np.cross(omega_body, r_ib))  # (N, 3)
        accel_imu = (R_ib @ (accel_body + coriolis + centripetal).T).T
        omega_imu = (R_ib @ omega_body.T).T

    if imu_config.smooth_imu_mode:
        final_gyro = omega_imu
        final_accel = accel_imu
    else:
        final_gyro, final_accel, _, _ = apply_imu_noise(omega_imu, accel_imu, imu_config, rng)

    return [
        ImuSample(
            timestamp_ns=int(ts_ns),
            quat_xyzw=quats_xyzw[i],
            angular_velocity=final_gyro[i],
            linear_acceleration=final_accel[i],
        )
        for i, ts_ns in enumerate(imu_ts_ns)
    ]
