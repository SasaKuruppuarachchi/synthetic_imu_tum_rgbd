# Synthetic IMU generator for TUM RGB-D Dynamic dataset

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A standalone utility for augmenting TUM RGB-D benchmark ROS 2 MCAP bags with a synthetic `/imu` topic derived from ground-truth camera poses.  No ROS 2 runtime is required at execution time.

> IMU data which is vital for benchmarking VINS packages are missing in the freiburg3 datasets - [“Sensors: RGBD, IMU (not in freiburg3), Ground truth”](https://www.mrpt.org/robotics_datasets?utm_source=chatgpt.com)

Download original `rosbag` version of TUM RGB-D Dataset [here.](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download#freiburg3_walking_halfsphere) 

---

## Overview

Ground-truth trajectories in the TUM RGB-D dataset are provided as rigid-body transforms on the `/tf` topic.  `generate_imu_bag` extracts these transforms, applies a kinematic differentiation pipeline to recover linear acceleration and angular velocity, adds a Kalibr-compatible stochastic noise model, and writes a new MCAP bag containing all original topics plus the synthesised `/imu` stream.

---

## Method

### 1  Pose extraction

Transforms are read from `/tf` and filtered by child frame (priority order: `kinect`, `camera_rgb_optical_frame`, `camera`).  Duplicate timestamps are resolved by retaining the latest sample; the resulting sequence is sorted and validated for strict monotonicity.

### 2  Kinematic differentiation at ground-truth rate

Let $\{(t_k,\, \mathbf{p}_k,\, \mathbf{q}_k)\}_{k=0}^{M-1}$ denote the ground-truth pose sequence at the native sensor rate (~30 Hz).  Differentiation is performed at this rate before upsampling to avoid amplifying interpolation artefacts [2].

**Linear acceleration (world frame)**

$$\mathbf{v}_k = \frac{\Delta\mathbf{p}}{\Delta t}\bigg|_k, \qquad \ddot{\mathbf{p}}_k = \frac{\Delta\mathbf{v}}{\Delta t}\bigg|_k$$

computed with `numpy.gradient` using non-uniform central differences.

**Angular velocity (world frame)**

$$\boldsymbol{\omega}_k = \frac{\log\!\left(R_k R_{k-1}^{-1}\right)^{\vee}}{\Delta t_k}$$

where $\log(\cdot)^{\vee}$ denotes the rotation-vector (axis-angle) of the relative rotation.

### 3  Frame transformation to IMU frame

The specific force measured by the accelerometer in body frame is

$$\mathbf{f}_b = R_{wb}^{-1}\!\left(\ddot{\mathbf{p}}_w + \mathbf{g}_w\right), \qquad \mathbf{g}_w = [0,\, 0,\, 9.81]^\top \text{ m/s}^2$$

Angular velocity is similarly rotated: $\boldsymbol{\omega}_b = R_{wb}^{-1}\boldsymbol{\omega}_w$.

Both signals are then projected into the IMU sensor frame using the body-to-IMU rotation $R_{ib}$ from `T_i_b` in the calibration file:

$$\mathbf{f}_i = R_{ib}\,\mathbf{f}_b, \qquad \boldsymbol{\omega}_i = R_{ib}\,\boldsymbol{\omega}_b$$

### 4  Outlier removal

Before the frame transform, per-axis outliers in $\ddot{\mathbf{p}}$ and $\boldsymbol{\omega}$ are detected using Median Absolute Deviation (MAD) or Inter-Quartile Range (IQR) and replaced by linear interpolation of flanking clean samples.  The detection threshold and method are configurable in `config/kalibr_imu_chain.yaml` under `imu0.outlier_filter`.

### 5  Upsampling to IMU rate

The cleaned GT-rate signals are upsampled to 200 Hz using cubic-spline interpolation; orientation is upsampled with quaternion SLERP.

### 6  Stochastic noise model

The Kalibr discrete-time IMU noise model [1] is applied at each 200 Hz sample:

$$\tilde{\boldsymbol{\omega}}_k = \boldsymbol{\omega}_k + \mathbf{b}_k^g + \mathbf{n}_k^g, \qquad \tilde{\mathbf{f}}_k = \mathbf{f}_k + \mathbf{b}_k^a + \mathbf{n}_k^a$$

where the white-noise and bias random-walk standard deviations derive from the continuous-time Kalibr parameters $(\sigma_g, \sigma_a, \sigma_{bg}, \sigma_{ba})$ via

$$\sigma_g^d = \frac{\sigma_g}{\sqrt{\Delta t}}, \quad \sigma_a^d = \frac{\sigma_a}{\sqrt{\Delta t}}, \quad \sigma_{bg}^d = \sigma_{bg}\sqrt{\Delta t}, \quad \sigma_{ba}^d = \sigma_{ba}\sqrt{\Delta t}$$

Noise realisations are reproducible: the PRNG seed is derived deterministically from the bag name via SHA-256.

### 7  Frame selection and `imu_frame_mode`

By default (`imu_frame_mode: false`) the pipeline extracts the `body_frame` transform from `/tf` and applies the calibrated body-to-IMU rotation $R_{ib}$ (from `T_i_b`) to project signals into the IMU sensor frame.

When `imu_frame_mode: true`, the pipeline instead extracts the `imu_frame` transform directly from `/tf`.  Because the poses are already expressed in the IMU frame, $R_{ib}$ is disregarded and the `T_i_b` field has no effect.  The output `sensor_msgs/msg/Imu` `header.frame_id` is set to `imu_frame` in this mode.

This mode is useful when the dataset provides a dedicated IMU frame transform in its TF tree, enabling a direct comparison against hardware-recorded IMU data without an additional rotational calibration step.

---

## Installation

```bash
git clone https://github.com/SasaKuruppuarachchi/synthetic_imu_tum_rgbd.git
cd synthetic_imu_tum_rgbd
pip install -r requirements.txt
```

Dependencies: `rosbags >= 0.11.0`, `numpy >= 1.21`, `scipy >= 1.7`, `ruamel.yaml`.

---

## Directory layout

```
synthetic_imu_tum_rgbd/
├── generate_imu_bag.py         # main entry point
├── requirements.txt
├── lib/
│   ├── models.py               # dataclasses, constants, ProcessingError
│   ├── config_loader.py        # kalibr_imu_chain.yaml parser
│   ├── bag_io.py               # bag reading, writing, CDR serialization
│   ├── imu_synthesis.py        # kinematic pipeline, noise model, outlier removal
│   └── ros1_converter.py       # ROS1 .bag detection and conversion
├── config/
│   └── kalibr_imu_chain.yaml   # noise + outlier parameters
├── Input/                      # place input bags here
├── output/                     # generated bags (cleared on each run)
└── scripts/
    └── ros1_bag_to_ros2_mcap_convert.sh
```

Place input bags (each a directory containing `metadata.yaml` and an MCAP file) inside `Input/`.  Output bags are written to `output/<bag_name>_imu/` in MCAP format.

---

## Usage

```bash
# Process all bags in Input/ (default)
python generate_imu_bag.py

# Process all bags with ZSTD compression
python generate_imu_bag.py --compress
```

### CLI reference

| Option | Default | Description |
|--------|---------|-------------|
| `--compress` | off | Enable ZSTD message-level compression on the output MCAP |

---

## Test commands

```bash
# 1. Verify the script runs without error on a single bag
python generate_imu_bag.py
# Expected: "=== Processing bag: ... ===" followed by "Done." with exit code 0

# 2. Inspect the output bag with ros2 bag info
ros2 bag info output/rgbd_dataset_freiburg3_sitting_static_mcap_imu/
# Expected: topic /imu listed with ~4700 messages, type sensor_msgs/msg/Imu

# 3. Verify MCAP storage format
python -c "
from rosbags.rosbag2 import Reader
from pathlib import Path
bags = sorted(Path('output').iterdir())
with Reader(bags[0]) as r:
    topics = [c.topic for c in r.connections]
    print('topics:', topics)
    assert '/imu' in topics, '/imu missing'
    print('PASS: /imu present')
"

# 4. Check IMU message count matches expected 200 Hz * bag_duration
python -c "
from rosbags.rosbag2 import Reader
from pathlib import Path
bags = sorted(Path('output').iterdir())
with Reader(bags[0]) as r:
    imu_conns = [c for c in r.connections if c.topic == '/imu']
    count = sum(1 for c, t, d in r.messages(connections=imu_conns))
    print(f'IMU messages: {count}')
"

# 5. Validate noise parameters load correctly from config
python -c "
from pathlib import Path
from lib.config_loader import load_imu_config
cfg = load_imu_config(Path('config/kalibr_imu_chain.yaml'))
print(f'gyro_noise_density : {cfg.gyroscope_noise_density}')
print(f'accel_noise_density: {cfg.accelerometer_noise_density}')
print(f'update_rate        : {cfg.update_rate} Hz')
print(f'outlier_filter     : {cfg.outlier_filter}')
"
```

---

## Configuration

Noise parameters and outlier filter settings are read from `config/kalibr_imu_chain.yaml` at startup (no code change required to tune them).

Key configurable fields under `imu0`:

| Field | Description |
|-------|-------------|
| `gyroscope_noise_density` | Continuous-time gyro white-noise density (rad/s/√Hz) |
| `accelerometer_noise_density` | Continuous-time accel white-noise density (m/s²/√Hz) |
| `gyroscope_random_walk` | Gyro bias random-walk strength (rad/s²/√Hz) |
| `accelerometer_random_walk` | Accel bias random-walk strength (m/s³/√Hz) |
| `update_rate` | IMU output rate (Hz) |
| `T_i_b` | 4×4 homogeneous body-to-IMU transform |
| `outlier_filter.enabled` | Enable/disable outlier rejection (`true`/`false`) |
| `outlier_filter.threshold_sigma` | Rejection threshold in robust-sigma units (default `3.0`) |
| `outlier_filter.method` | Detection method: `"mad"` (default) or `"iqr"` |
| `body_frame` | TF child frame name representing the sensor body (default `"kinect"`) |
| `imu_frame` | TF child frame name of the physical IMU (default `"imu"`) |
| `imu_frame_mode` | When `true`, extract poses from `imu_frame` TF directly and disregard `T_i_b` |

---

## Output message format

Generated `sensor_msgs/msg/Imu` fields:

| Field | Value |
|-------|-------|
| `header.frame_id` | `"kinect"` |
| `orientation` | SLERP-interpolated quaternion |
| `orientation_covariance[0]` | `-1` (unknown, per REP-145) |
| `angular_velocity` | $\tilde{\boldsymbol{\omega}}_k$ (body frame, noisy) |
| `angular_velocity_covariance` | Diagonal: $(\sigma_g^d)^2$ |
| `linear_acceleration` | $\tilde{\mathbf{f}}_k$ (body frame, noisy, gravity-compensated) |
| `linear_acceleration_covariance` | Diagonal: $(\sigma_a^d)^2$ |

---

## References

[1] P. Furgale, J. Rehder, R. Siegwart, "Unified Temporal and Spatial Calibration for Multi-Sensor Systems," IROS 2013.  
    IMU Noise Model: <https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model>

[2] N. Correll Lab, "Use Transformer Model to Generate Next-Step IMU Data from Known Linear Velocity and Orientation," *Medium*, 2023.  
    <https://medium.com/correll-lab/use-transformer-model-to-generate-next-step-imu-data-from-known-linear-velocity-and-orientation-a101e3a578df>
