"""ROS2 MCAP bag reading, writing, and CDR serialization utilities."""
from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation as _Rotation
from rosbags.rosbag2 import Reader, Writer
from rosbags.rosbag2.enums import CompressionFormat, CompressionMode, StoragePlugin

from lib.models import (
    ImuConfig,
    ImuSample,
    IMU_MSGTYPE,
    PoseSample,
    ProcessingError,
    TF_MSGTYPE,
)


def deserialize(rawdata: bytes, msgtype: str, typestore):
    return typestore.deserialize_cdr(rawdata, msgtype)


def serialize(message, msgtype: str, typestore) -> bytes:
    return typestore.serialize_cdr(message, msgtype)


def to_ros_time(timestamp_ns: int, time_cls):
    sec = int(timestamp_ns // 1_000_000_000)
    nanosec = int(timestamp_ns % 1_000_000_000)
    return time_cls(sec=sec, nanosec=nanosec)


def discover_input_bags(input_dir: Path) -> list[Path]:
    return sorted(
        p for p in input_dir.iterdir() if p.is_dir() and (p / "metadata.yaml").exists()
    )


def extract_tf_pose_samples(
    bag_path: Path,
    typestore,
) -> tuple[list[PoseSample], int, int, int]:
    """Read /tf and collect pose samples and bag time span.

    Returns: (samples, tf_msg_count, bag_start_ns, bag_end_ns)
    """
    samples: list[PoseSample] = []
    tf_msg_count = 0
    bag_start_ns: int | None = None
    bag_end_ns: int | None = None

    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if bag_start_ns is None or timestamp < bag_start_ns:
                bag_start_ns = timestamp
            if bag_end_ns is None or timestamp > bag_end_ns:
                bag_end_ns = timestamp

            if connection.topic != "/tf":
                continue

            tf_msg_count += 1
            tf_msg = deserialize(rawdata, TF_MSGTYPE, typestore)
            for transform_stamped in tf_msg.transforms:
                transform = transform_stamped.transform
                child_frame_id = transform_stamped.child_frame_id.strip().lstrip("/")
                sample = PoseSample(
                    timestamp_ns=timestamp,
                    position=np.array(
                        [
                            float(transform.translation.x),
                            float(transform.translation.y),
                            float(transform.translation.z),
                        ],
                        dtype=np.float64,
                    ),
                    quat_xyzw=np.array(
                        [
                            float(transform.rotation.x),
                            float(transform.rotation.y),
                            float(transform.rotation.z),
                            float(transform.rotation.w),
                        ],
                        dtype=np.float64,
                    ),
                    child_frame_id=child_frame_id,
                )
                samples.append(sample)

    if bag_start_ns is None or bag_end_ns is None:
        raise ProcessingError("Input bag does not contain any messages.")

    return samples, tf_msg_count, bag_start_ns, bag_end_ns


def build_imu_raw_messages(
    imu_samples: Sequence[ImuSample],
    frame_id: str,
    imu_config: ImuConfig,
    typestore,
) -> tuple[list[tuple[int, bytes]], float, float]:
    """Pack IMU samples into serialized CDR payloads."""
    Header = typestore.types["std_msgs/msg/Header"]
    Time = typestore.types["builtin_interfaces/msg/Time"]
    Quaternion = typestore.types["geometry_msgs/msg/Quaternion"]
    Vector3 = typestore.types["geometry_msgs/msg/Vector3"]
    Imu = typestore.types[IMU_MSGTYPE]

    if imu_config.smooth_imu_mode:
        _small = 1e-8
        gyro_var = _small
        accel_var = _small
    else:
        gyro_var = (imu_config.gyroscope_noise_density * np.sqrt(imu_config.update_rate)) ** 2
        accel_var = (imu_config.accelerometer_noise_density * np.sqrt(imu_config.update_rate)) ** 2

    orientation_cov = np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    gyro_cov = np.array([gyro_var, 0.0, 0.0, 0.0, gyro_var, 0.0, 0.0, 0.0, gyro_var], dtype=np.float64)
    accel_cov = np.array([accel_var, 0.0, 0.0, 0.0, accel_var, 0.0, 0.0, 0.0, accel_var], dtype=np.float64)

    packed: list[tuple[int, bytes]] = []
    for sample in imu_samples:
        stamp = to_ros_time(sample.timestamp_ns, Time)
        header = Header(stamp=stamp, frame_id=frame_id)
        orientation = Quaternion(
            x=float(sample.quat_xyzw[0]),
            y=float(sample.quat_xyzw[1]),
            z=float(sample.quat_xyzw[2]),
            w=float(sample.quat_xyzw[3]),
        )
        angular_velocity = Vector3(
            x=float(sample.angular_velocity[0]),
            y=float(sample.angular_velocity[1]),
            z=float(sample.angular_velocity[2]),
        )
        linear_acceleration = Vector3(
            x=float(sample.linear_acceleration[0]),
            y=float(sample.linear_acceleration[1]),
            z=float(sample.linear_acceleration[2]),
        )
        imu_msg = Imu(
            header=header,
            orientation=orientation,
            orientation_covariance=orientation_cov,
            angular_velocity=angular_velocity,
            angular_velocity_covariance=gyro_cov,
            linear_acceleration=linear_acceleration,
            linear_acceleration_covariance=accel_cov,
        )
        raw = serialize(imu_msg, IMU_MSGTYPE, typestore)
        packed.append((sample.timestamp_ns, raw))

    return packed, gyro_var, accel_var


def bag_has_imu(bag_path: Path) -> bool:
    with Reader(bag_path) as reader:
        return any(c.msgtype == IMU_MSGTYPE for c in reader.connections)


def _clone_connection(writer: Writer, connection, typestore):
    kwargs = {}
    ext = getattr(connection, "ext", None)

    serialization_format = getattr(ext, "serialization_format", None)
    if serialization_format is not None:
        kwargs["serialization_format"] = serialization_format

    offered_qos_profiles = getattr(ext, "offered_qos_profiles", None)
    if offered_qos_profiles is not None:
        kwargs["offered_qos_profiles"] = offered_qos_profiles

    msgdef = getattr(connection, "msgdef", None)
    if msgdef is not None:
        msgdef_data = getattr(msgdef, "data", msgdef)
        if msgdef_data:
            kwargs["msgdef"] = msgdef_data

    rihs01 = getattr(connection, "rihs01", None)
    if rihs01 is None:
        rihs01 = getattr(connection, "digest", None)
    if rihs01:
        kwargs["rihs01"] = rihs01

    try:
        return writer.add_connection(connection.topic, connection.msgtype, typestore=typestore, **kwargs)
    except Exception:
        kwargs.pop("msgdef", None)
        kwargs.pop("rihs01", None)
        return writer.add_connection(connection.topic, connection.msgtype, typestore=typestore, **kwargs)


def write_output_bag(
    input_bag: Path,
    output_bag: Path,
    imu_raw_messages: Sequence[tuple[int, bytes]],
    typestore,
    compress: bool,
    imu_topic: str = "/imu",
) -> None:
    if output_bag.exists():
        raise ProcessingError(
            f"Output path already exists: {output_bag}. Remove it first or pick another bag."
        )

    writer = Writer(output_bag, version=Writer.VERSION_LATEST, storage_plugin=StoragePlugin.MCAP)
    if compress:
        writer.set_compression(CompressionMode.MESSAGE, CompressionFormat.ZSTD)

    with Reader(input_bag) as reader, writer:
        conn_map = {}
        for connection in reader.connections:
            conn_map[connection.id] = _clone_connection(writer, connection, typestore)

        imu_conn = writer.add_connection(imu_topic, IMU_MSGTYPE, typestore=typestore, serialization_format="cdr")

        imu_idx = 0
        imu_total = len(imu_raw_messages)

        for connection, timestamp, rawdata in reader.messages():
            while imu_idx < imu_total and imu_raw_messages[imu_idx][0] <= timestamp:
                imu_ts, imu_raw = imu_raw_messages[imu_idx]
                writer.write(imu_conn, imu_ts, imu_raw)
                imu_idx += 1
            writer.write(conn_map[connection.id], timestamp, rawdata)

        while imu_idx < imu_total:
            imu_ts, imu_raw = imu_raw_messages[imu_idx]
            writer.write(imu_conn, imu_ts, imu_raw)
            imu_idx += 1

    metadata_path = output_bag / "metadata.yaml"
    if metadata_path.exists():
        metadata_path.write_text(
            metadata_path.read_text().replace("offered_qos_profiles: []", "offered_qos_profiles: ''")
        )


def export_kalibr_yaml(output_bag: Path, imu_config: "ImuConfig") -> Path:
    """Write a Kalibr-compatible imu.yaml file alongside the output bag directory.

    The file is placed at ``output_bag.parent / "imu.yaml"``.
    """
    # Reconstruct full 4x4 T_i_b from stored rotation and lever arm.
    T_full = np.eye(4, dtype=np.float64)
    T_full[:3, :3] = imu_config.T_i_b
    T_full[:3, 3] = imu_config.r_i_b

    rows = [list(map(float, row)) for row in T_full]

    lines: list[str] = [
        "imu0:",
        f"  background: {imu_config.rostopic}",
        "  transport_delay: 0.0",
        f"  update_rate: {float(imu_config.update_rate)}",
        f"  accelerometer_noise_density: {imu_config.accelerometer_noise_density}",
        f"  accelerometer_random_walk: {imu_config.accelerometer_random_walk}",
        f"  gyroscope_noise_density: {imu_config.gyroscope_noise_density}",
        f"  gyroscope_random_walk: {imu_config.gyroscope_random_walk}",
        "  T_i_b:",
    ]
    for row in rows:
        formatted = "[" + ", ".join(f"{v:.10g}" for v in row) + "]"
        lines.append(f"    - {formatted}")

    yaml_text = "\n".join(lines) + "\n"
    yaml_path = output_bag.parent / "imu.yaml"
    yaml_path.write_text(yaml_text, encoding="utf-8")
    return yaml_path


def _make_homogeneous(tx: float, ty: float, tz: float,
                      rx: float, ry: float, rz: float, rw: float) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = _Rotation.from_quat([rx, ry, rz, rw]).as_matrix()
    T[:3, 3] = [tx, ty, tz]
    return T


def extract_static_tf_tree(bag_path: Path, typestore) -> dict:
    """Read /tf_static and /tf; return {child_frame: (parent_frame, T_4x4)}.

    Static transforms (/tf_static) take priority.
    For frames absent from /tf_static, the first occurrence on /tf is used
    (correct for fixed camera-rig frames published on the dynamic topic).
    T_4x4 satisfies: p_parent = T_4x4 @ p_child  (ROS TF convention).
    """
    tree: dict[str, tuple[str, np.ndarray]] = {}
    first_dynamic: dict[str, tuple[str, np.ndarray]] = {}

    with Reader(bag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic not in ("/tf_static", "tf_static", "/tf", "tf"):
                continue
            tf_msg = deserialize(rawdata, TF_MSGTYPE, typestore)
            is_static = connection.topic in ("/tf_static", "tf_static")
            for ts in tf_msg.transforms:
                child = ts.child_frame_id.strip().lstrip("/")
                parent = ts.header.frame_id.strip().lstrip("/")
                tr = ts.transform.translation
                ro = ts.transform.rotation
                T = _make_homogeneous(tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w)
                if is_static:
                    tree[child] = (parent, T)
                elif child not in first_dynamic:
                    first_dynamic[child] = (parent, T)

    # Fill gaps: frames only seen on /tf, not overwriting static entries
    for child, entry in first_dynamic.items():
        if child not in tree:
            tree[child] = entry

    return tree


def lookup_transform(tree: dict, from_frame: str, to_frame: str) -> "np.ndarray | None":
    """Return 4x4 T such that p_{to_frame} = T @ p_{from_frame}.

    Returns None if no path exists between the two frames in the tree.
    """
    if from_frame == to_frame:
        return np.eye(4, dtype=np.float64)

    # Collect ancestors of from_frame (maps frame -> depth)
    ancestors_from: dict[str, int] = {}
    frame: str | None = from_frame
    depth = 0
    while frame is not None:
        ancestors_from[frame] = depth
        entry = tree.get(frame)
        frame = entry[0] if entry else None
        depth += 1

    # Walk up from to_frame until we hit a known ancestor (= LCA)
    path_to = [to_frame]
    frame = to_frame
    while frame not in ancestors_from:
        entry = tree.get(frame)
        if entry is None:
            return None  # disconnected
        frame = entry[0]
        path_to.append(frame)
    lca = frame

    # Walk up from from_frame to LCA
    path_from = [from_frame]
    frame = from_frame
    while frame != lca:
        entry = tree.get(frame)
        if entry is None:
            return None
        frame = entry[0]
        path_from.append(frame)

    # Compose: go UP from from_frame → LCA  (each stored T is T_parent_child)
    T = np.eye(4, dtype=np.float64)
    for i in range(len(path_from) - 1):
        child_f = path_from[i]
        _, T_pc = tree[child_f]      # p_parent = T_pc @ p_child
        T = T_pc @ T

    # Compose: go DOWN from LCA → to_frame  (use inverse of stored T)
    # path_to is [to_frame, ..., lca]; path_to[:-1] reversed gives lca's side → to_frame
    for child_f in reversed(path_to[:-1]):
        _, T_pc = tree[child_f]      # p_parent = T_pc @ p_child  →  p_child = inv(T_pc) @ p_parent
        T = np.linalg.inv(T_pc) @ T

    return T


def extract_camera_info(bag_path: Path, typestore) -> "dict | None":
    """Return camera parameters from the first sensor_msgs/msg/CameraInfo message found.

    Returns dict with keys: intrinsics, distortion, resolution, frame_id, distortion_model, rostopic.
    Returns None if no camera_info topic exists in the bag.
    """
    CAM_MSGTYPE = "sensor_msgs/msg/CameraInfo"

    preferred_topics = ["/camera/rgb/camera_info", "/camera/depth/camera_info"]
    cam_topics: list[str] = []

    with Reader(bag_path) as reader:
        for conn in reader.connections:
            if conn.msgtype == CAM_MSGTYPE:
                if conn.topic in preferred_topics:
                    cam_topics.insert(0, conn.topic)
                else:
                    cam_topics.append(conn.topic)

    if not cam_topics:
        return None

    target_topic = cam_topics[0]

    with Reader(bag_path) as reader:
        cam_conns = [c for c in reader.connections if c.topic == target_topic]
        for _, _, rawdata in reader.messages(connections=cam_conns):
            msg = deserialize(rawdata, CAM_MSGTYPE, typestore)
            K = list(msg.k)                   # 9 floats, row-major
            D = list(msg.d)                   # variable length
            w = int(msg.width)
            h = int(msg.height)
            fid = str(msg.header.frame_id).strip().lstrip("/")
            dmodel_raw = str(getattr(msg, "distortion_model", "plumb_bob"))
            dmodel = "radtan" if dmodel_raw in ("plumb_bob", "radtan") else dmodel_raw
            # Infer image topic: camera_info topic → image topic heuristic
            img_topic = target_topic.replace("camera_info", "image_color")
            return {
                "intrinsics": [K[0], K[4], K[2], K[5]],   # fx, fy, cx, cy
                "distortion": D[:4] if len(D) >= 4 else D,
                "resolution": [w, h],
                "frame_id": fid,
                "distortion_model": dmodel,
                "rostopic": img_topic,
            }

    return None


def export_kalibr_imu_chain_yaml(
    output_bag: Path,
    imu_config: "ImuConfig",
    T_i_b_4x4: np.ndarray,
) -> Path:
    """Write a kalibr_imu_chain.yaml file inside the output bag directory.

    Styled exactly like ref_calib/kalibr_imu_chain.yaml.
    T_i_b_4x4 must satisfy p_imu = T_i_b_4x4 @ p_body.
    """
    def _fmt_row4(row):
        return "[" + ", ".join(f"{v:.10g}" for v in row) + "]"

    def _fmt_row3(row):
        return "[" + ", ".join(f"{v:.10g}" for v in row) + "]"

    rows4 = [list(map(float, T_i_b_4x4[i])) for i in range(4)]
    I3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    Z3 = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

    lines = [
        "%YAML:1.0",
        "",
        "imu0:",
        "  T_i_b:",
    ]
    for row in rows4:
        lines.append(f"    - {_fmt_row4(row)}")
    lines += [
        f"  accelerometer_noise_density: {imu_config.accelerometer_noise_density}",
        f"  accelerometer_random_walk: {imu_config.accelerometer_random_walk}",
        f"  gyroscope_noise_density: {imu_config.gyroscope_noise_density}",
        f"  gyroscope_random_walk: {imu_config.gyroscope_random_walk}",
        f"  rostopic: {imu_config.rostopic}",
        "  time_offset: 0.0",
        f"  update_rate: {int(imu_config.update_rate)}",
        '  model: "kalibr"',
        "  Tw:",
    ]
    for row in I3:
        lines.append(f"    - {_fmt_row3(row)}")
    lines.append("  R_IMUtoGYRO:")
    for row in I3:
        lines.append(f"    - {_fmt_row3(row)}")
    lines.append("  Ta:")
    for row in I3:
        lines.append(f"    - {_fmt_row3(row)}")
    lines.append("  R_IMUtoACC:")
    for row in I3:
        lines.append(f"    - {_fmt_row3(row)}")
    lines.append("  Tg:")
    for row in Z3:
        lines.append(f"    - {_fmt_row3(row)}")

    yaml_text = "\n".join(lines) + "\n"
    yaml_path = output_bag / "kalibr_imu_chain.yaml"
    yaml_path.write_text(yaml_text, encoding="utf-8")
    return yaml_path


def export_kalibr_imucam_chain_yaml(
    output_bag: Path,
    T_cam_imu: np.ndarray,
    cam_info: dict,
) -> Path:
    """Write a kalibr_imucam_chain.yaml file inside the output bag directory.

    Styled exactly like ref_calib/kalibr_imucam_chain.yaml.
    T_cam_imu must satisfy p_cam = T_cam_imu @ p_imu  (Kalibr convention).
    cam_info: dict with keys intrinsics, distortion, resolution, distortion_model, rostopic.
    """
    def _fmt_row4(row):
        vals = [f"{v:.6f}" for v in row]
        return "[" + ", ".join(vals) + "]"

    rows4 = [list(map(float, T_cam_imu[i])) for i in range(4)]
    intr = cam_info["intrinsics"]    # [fx, fy, cx, cy]
    dist = cam_info["distortion"]    # [k1, k2, p1, p2]
    res  = cam_info["resolution"]    # [w, h]

    lines = [
        "%YAML:1.0",
        "",
        "cam0:",
        "  T_cam_imu:",
    ]
    for row in rows4:
        lines.append(f"    - {_fmt_row4(row)}")
    lines += [
        "  cam_overlaps: []",
        "  camera_model: pinhole",
        "  distortion_coeffs: [" + ", ".join(f"{v}" for v in dist) + "]",
        f"  distortion_model: {cam_info['distortion_model']}",
        "  intrinsics: [" + ", ".join(f"{v}" for v in intr) + "]",
        f"  resolution: [{res[0]}, {res[1]}]",
        f"  rostopic: {cam_info['rostopic']}",
        "  timeshift_cam_imu: 0.0",
    ]

    yaml_text = "\n".join(lines) + "\n"
    yaml_path = output_bag / "kalibr_imucam_chain.yaml"
    yaml_path.write_text(yaml_text, encoding="utf-8")
    return yaml_path
