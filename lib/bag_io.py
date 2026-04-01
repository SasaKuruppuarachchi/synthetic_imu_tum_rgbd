"""ROS2 MCAP bag reading, writing, and CDR serialization utilities."""
from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np
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
            metadata_path.read_text().replace("offered_qos_profiles: []", "offered_qos_profiles: null")
        )
