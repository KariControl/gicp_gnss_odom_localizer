#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate the public, fully procedural LiDAR/IMU smoke-test rosbag.

The generator intentionally has no input-bag option. It uses the documented
Pandar-style PointCloud2 layout and nominal message rates. Every coordinate,
motion sample, intensity, and timestamp is generated analytically by this file.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
from typing import Iterable


DATASET_ID = "synthetic_output_pointcloud2_smoke_v1"
GENERATOR_VERSION = 1
DEFAULT_OUTPUT = Path("data/synthetic_output_pointcloud2")

TF_STAMP_NS = 800_000_000
IMU_START_NS = 900_000_000
SCAN_START_NS = 1_000_000_000
SCAN_PERIOD_NS = 50_000_000
SCAN_DURATION_NS = 50_000_000
IMU_PERIOD_NS = 5_000_000
IMU_END_NS = 7_100_000_000
SCAN_END_NS = 7_000_000_000

SPEED_MPS = 0.5
YAW_RATE_RADPS = 0.05
GRAVITY_MPS2 = 9.80665

POINT_STEP = 48
POINT_FIELDS = (
    ("x", 0, 7),
    ("y", 4, 7),
    ("z", 8, 7),
    ("intensity", 16, 7),
    ("ring", 20, 4),
    ("azimuth", 24, 7),
    ("distance", 28, 7),
    ("return_type", 32, 2),
    ("time_stamp", 40, 8),
)

TOPICS = (
    (0, "/tf_static", "tf2_msgs/msg/TFMessage"),
    (1, "/sensor/imu/data_raw", "sensor_msgs/msg/Imu"),
    (2, "/synthetic/ground_truth", "nav_msgs/msg/Odometry"),
    (3, "/pandar_points_ex", "sensor_msgs/msg/PointCloud2"),
)


def import_ros() -> tuple[object, ...]:
    """Import ROS only after argument parsing so ``--help`` works anywhere."""

    try:
        import rosbag2_py
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message, serialize_message
        from sensor_msgs.msg import Imu, PointCloud2, PointField
        from tf2_msgs.msg import TFMessage
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Jazzy Python modules are unavailable. Source "
            "/opt/ros/jazzy/setup.bash before running this generator."
        ) from exc
    return (
        rosbag2_py,
        TransformStamped,
        Odometry,
        deserialize_message,
        serialize_message,
        Imu,
        PointCloud2,
        PointField,
        TFMessage,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"new output directory (default: {DEFAULT_OUTPUT})",
    )
    return parser.parse_args()


def stamp(message: object, stamp_ns: int) -> None:
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000


def pose_at(stamp_ns: int) -> tuple[float, float, float]:
    """Return the analytic base pose relative to the first LiDAR scan."""

    time_sec = (stamp_ns - SCAN_START_NS) * 1.0e-9
    yaw = YAW_RATE_RADPS * time_sec
    radius = SPEED_MPS / YAW_RATE_RADPS
    x = radius * math.sin(yaw)
    y = radius * (1.0 - math.cos(yaw))
    return x, y, yaw


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def procedural_scene() -> list[tuple[float, float, float, float]]:
    """Build an asymmetric synthetic corridor without reading source data."""

    points: list[tuple[float, float, float, float]] = []

    # Ground plane and two curbs.
    for xi in range(-7, 35):
        for yi in range(-8, 9):
            points.append((1.5 * xi, 1.5 * yi, -1.50, 18.0))
    for xi in range(-10, 51):
        x = float(xi)
        for y in (-5.2, 5.2):
            points.append((x, y, -1.15, 70.0))
            points.append((x, y, -0.95, 75.0))

    # Non-parallel walls provide strong yaw and lateral observability.
    for xi in range(-10, 51):
        x = float(xi)
        left_y = 8.0 + 0.015 * x
        right_y = -7.5 + 0.010 * x
        for zi in range(-2, 7):
            z = 0.6 * zi
            points.append((x, left_y, z, 105.0))
            points.append((x, right_y, z, 125.0))

    # Poles deliberately use unequal positions and radii.
    pole_specs = (
        (-3.0, 4.0, 0.18),
        (3.5, -4.8, 0.22),
        (8.0, 5.8, 0.16),
        (13.0, -5.4, 0.25),
        (18.5, 4.5, 0.20),
        (24.0, -4.1, 0.17),
        (31.0, 5.2, 0.24),
        (39.0, -5.7, 0.19),
    )
    for pole_index, (cx, cy, radius) in enumerate(pole_specs):
        for angle_index in range(12):
            angle = math.tau * angle_index / 12.0
            for zi in range(-2, 7):
                points.append(
                    (
                        cx + radius * math.cos(angle),
                        cy + radius * math.sin(angle),
                        0.5 * zi,
                        155.0 + 5.0 * (pole_index % 4),
                    )
                )

    # Three unrelated box-like landmarks break the corridor symmetry.
    boxes = (
        (6.0, 2.8, 1.2, 0.9, 205.0),
        (20.0, -2.5, 1.7, 1.1, 225.0),
        (34.0, 1.7, 1.0, 1.5, 240.0),
    )
    for cx, cy, half_x, half_y, intensity in boxes:
        for u_index in range(-4, 5):
            u = u_index / 4.0
            for zi in range(-2, 6):
                z = 0.5 * zi
                points.append((cx + half_x * u, cy - half_y, z, intensity))
                points.append((cx + half_x * u, cy + half_y, z, intensity))
                points.append((cx - half_x, cy + half_y * u, z, intensity))
                points.append((cx + half_x, cy + half_y * u, z, intensity))

    return points


def world_to_sensor(
    point: tuple[float, float, float, float], stamp_ns: int
) -> tuple[float, float, float, float]:
    px, py, pz, intensity = point
    base_x, base_y, yaw = pose_at(stamp_ns)
    dx = px - base_x
    dy = py - base_y
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * dx + sine * dy, -sine * dx + cosine * dy, pz, intensity


def make_cloud(
    scan_stamp_ns: int,
    scene: Iterable[tuple[float, float, float, float]],
    PointCloud2: object,
    PointField: object,
) -> object:
    # Establish deterministic firing order from start-of-scan azimuth.
    ordered: list[tuple[float, tuple[float, float, float, float]]] = []
    for world_point in scene:
        x, y, z, intensity = world_to_sensor(world_point, scan_stamp_ns)
        distance = math.sqrt(x * x + y * y + z * z)
        if 1.25 <= distance <= 48.0:
            ordered.append((math.atan2(y, x), world_point))
    ordered.sort(key=lambda entry: (entry[0], entry[1][2], entry[1][0], entry[1][1]))
    if len(ordered) < 1000:
        raise RuntimeError(f"synthetic scene yielded only {len(ordered)} visible points")

    data = bytearray(len(ordered) * POINT_STEP)
    denominator = max(1, len(ordered) - 1)
    for index, (_, world_point) in enumerate(ordered):
        relative_sec = (SCAN_DURATION_NS * 1.0e-9) * index / denominator
        acquisition_ns = scan_stamp_ns + round(relative_sec * 1.0e9)
        x, y, z, intensity = world_to_sensor(world_point, acquisition_ns)
        horizontal = math.hypot(x, y)
        distance = math.sqrt(horizontal * horizontal + z * z)
        azimuth = math.atan2(y, x)
        elevation_deg = math.degrees(math.atan2(z, max(horizontal, 1.0e-9)))
        ring = max(0, min(31, round((elevation_deg + 15.0) * 31.0 / 30.0)))
        offset = index * POINT_STEP
        struct.pack_into("<f", data, offset + 0, x)
        struct.pack_into("<f", data, offset + 4, y)
        struct.pack_into("<f", data, offset + 8, z)
        struct.pack_into("<f", data, offset + 16, intensity)
        struct.pack_into("<H", data, offset + 20, ring)
        struct.pack_into("<f", data, offset + 24, azimuth)
        struct.pack_into("<f", data, offset + 28, distance)
        struct.pack_into("<B", data, offset + 32, 1)
        struct.pack_into("<d", data, offset + 40, relative_sec)

    message = PointCloud2()
    stamp(message, scan_stamp_ns)
    message.header.frame_id = "lidar/0"
    message.height = 1
    message.width = len(ordered)
    message.fields = [
        PointField(name=name, offset=offset, datatype=datatype, count=1)
        for name, offset, datatype in POINT_FIELDS
    ]
    message.is_bigendian = False
    message.point_step = POINT_STEP
    message.row_step = POINT_STEP * len(ordered)
    message.data = data
    message.is_dense = True
    return message


def make_imu(stamp_ns: int, Imu: object) -> object:
    message = Imu()
    stamp(message, stamp_ns)
    message.header.frame_id = "imu"
    _, _, yaw = pose_at(stamp_ns)
    qx, qy, qz, qw = yaw_quaternion(yaw)
    message.orientation.x = qx
    message.orientation.y = qy
    message.orientation.z = qz
    message.orientation.w = qw
    message.angular_velocity.z = YAW_RATE_RADPS
    message.linear_acceleration.y = SPEED_MPS * YAW_RATE_RADPS
    message.linear_acceleration.z = GRAVITY_MPS2
    message.orientation_covariance = [
        1.0e-4,
        0.0,
        0.0,
        0.0,
        1.0e-4,
        0.0,
        0.0,
        0.0,
        1.0e-4,
    ]
    message.angular_velocity_covariance = [
        1.0e-6,
        0.0,
        0.0,
        0.0,
        1.0e-6,
        0.0,
        0.0,
        0.0,
        1.0e-6,
    ]
    message.linear_acceleration_covariance = [
        1.0e-4,
        0.0,
        0.0,
        0.0,
        1.0e-4,
        0.0,
        0.0,
        0.0,
        1.0e-4,
    ]
    return message


def make_ground_truth(stamp_ns: int, Odometry: object) -> object:
    message = Odometry()
    stamp(message, stamp_ns)
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    x, y, yaw = pose_at(stamp_ns)
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    qx, qy, qz, qw = yaw_quaternion(yaw)
    message.pose.pose.orientation.x = qx
    message.pose.pose.orientation.y = qy
    message.pose.pose.orientation.z = qz
    message.pose.pose.orientation.w = qw
    message.twist.twist.linear.x = SPEED_MPS
    message.twist.twist.angular.z = YAW_RATE_RADPS
    return message


def make_static_tf(
    TransformStamped: object, TFMessage: object, stamp_ns: int
) -> object:
    transforms = []
    for child_frame in ("lidar/0", "imu"):
        transform = TransformStamped()
        stamp(transform, stamp_ns)
        transform.header.frame_id = "base_link"
        transform.child_frame_id = child_frame
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    return TFMessage(transforms=transforms)


def _hash_bytes(digest: "hashlib._Hash", value: bytes) -> None:
    digest.update(struct.pack("<Q", len(value)))
    digest.update(value)


def _hash_string(digest: "hashlib._Hash", value: str) -> None:
    _hash_bytes(digest, value.encode("utf-8"))


def _hash_header(digest: "hashlib._Hash", message: object) -> None:
    digest.update(
        struct.pack(
            "<q",
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec,
        )
    )
    _hash_string(digest, message.header.frame_id)


def _hash_vector3(digest: "hashlib._Hash", vector: object) -> None:
    digest.update(struct.pack("<3d", vector.x, vector.y, vector.z))


def _hash_quaternion(digest: "hashlib._Hash", quaternion: object) -> None:
    digest.update(
        struct.pack(
            "<4d", quaternion.x, quaternion.y, quaternion.z, quaternion.w
        )
    )


class _CdrPaddingSanitizer:
    """Zero XCDR1 alignment bytes while walking a known ROS message layout."""

    def __init__(self, serialized: bytes) -> None:
        if len(serialized) < 4 or serialized[:4] != b"\x00\x01\x00\x00":
            raise RuntimeError("unexpected CDR encapsulation header")
        self.data = bytearray(serialized)
        self.offset = 4
        self.alignment_origin = 4

    def align(self, alignment: int) -> None:
        relative = self.offset - self.alignment_origin
        aligned = self.alignment_origin + (
            (relative + alignment - 1) // alignment
        ) * alignment
        if aligned > len(self.data):
            raise RuntimeError("CDR alignment exceeds serialized message")
        self.data[self.offset : aligned] = b"\x00" * (aligned - self.offset)
        self.offset = aligned

    def scalar(self, size: int, alignment: int | None = None) -> None:
        self.align(size if alignment is None else alignment)
        self.offset += size
        if self.offset > len(self.data):
            raise RuntimeError("CDR scalar exceeds serialized message")

    def uint32(self) -> int:
        self.align(4)
        value = struct.unpack_from("<I", self.data, self.offset)[0]
        self.offset += 4
        return value

    def string(self) -> None:
        length = self.uint32()
        if length == 0:
            raise RuntimeError("CDR string has no null terminator")
        self.offset += length
        if self.offset > len(self.data) or self.data[self.offset - 1] != 0:
            raise RuntimeError("invalid CDR string encoding")

    def header(self) -> None:
        self.scalar(4)
        self.scalar(4)
        self.string()

    def vector3(self) -> None:
        for _ in range(3):
            self.scalar(8)

    def quaternion(self) -> None:
        for _ in range(4):
            self.scalar(8)

    def double_array(self, length: int) -> None:
        for _ in range(length):
            self.scalar(8)

    def finish(self) -> bytes:
        if self.offset != len(self.data):
            raise RuntimeError(
                "CDR layout contract did not consume the message: "
                f"offset={self.offset}, size={len(self.data)}"
            )
        return bytes(self.data)


def sanitize_cdr_padding(topic: str, serialized: bytes) -> bytes:
    """Remove non-semantic padding bytes from the four fixture interfaces."""

    cursor = _CdrPaddingSanitizer(serialized)
    if topic == "/tf_static":
        transform_count = cursor.uint32()
        for _ in range(transform_count):
            cursor.header()
            cursor.string()
            cursor.vector3()
            cursor.quaternion()
    elif topic == "/sensor/imu/data_raw":
        cursor.header()
        cursor.quaternion()
        cursor.double_array(9)
        cursor.vector3()
        cursor.double_array(9)
        cursor.vector3()
        cursor.double_array(9)
    elif topic == "/synthetic/ground_truth":
        cursor.header()
        cursor.string()
        cursor.vector3()
        cursor.quaternion()
        cursor.double_array(36)
        cursor.vector3()
        cursor.vector3()
        cursor.double_array(36)
    elif topic == "/pandar_points_ex":
        cursor.header()
        cursor.scalar(4)
        cursor.scalar(4)
        field_count = cursor.uint32()
        for _ in range(field_count):
            cursor.string()
            cursor.scalar(4)
            cursor.scalar(1)
            cursor.scalar(4)
        cursor.scalar(1)
        cursor.scalar(4)
        cursor.scalar(4)
        data_length = cursor.uint32()
        cursor.offset += data_length
        if cursor.offset > len(cursor.data):
            raise RuntimeError("PointCloud2 data exceeds serialized message")
        cursor.scalar(1)
    else:
        raise RuntimeError(f"no CDR padding contract for topic: {topic}")
    return cursor.finish()


def update_semantic_stream_hash(
    digest: "hashlib._Hash",
    topic: str,
    type_name: str,
    record_ns: int,
    message: object,
) -> None:
    """Hash message fields without implementation-defined CDR padding bytes."""

    _hash_string(digest, topic)
    _hash_string(digest, type_name)
    digest.update(struct.pack("<q", record_ns))
    if topic == "/tf_static":
        digest.update(struct.pack("<Q", len(message.transforms)))
        for transform in message.transforms:
            _hash_header(digest, transform)
            _hash_string(digest, transform.child_frame_id)
            _hash_vector3(digest, transform.transform.translation)
            _hash_quaternion(digest, transform.transform.rotation)
    elif topic == "/sensor/imu/data_raw":
        _hash_header(digest, message)
        _hash_quaternion(digest, message.orientation)
        digest.update(struct.pack("<9d", *message.orientation_covariance))
        _hash_vector3(digest, message.angular_velocity)
        digest.update(struct.pack("<9d", *message.angular_velocity_covariance))
        _hash_vector3(digest, message.linear_acceleration)
        digest.update(struct.pack("<9d", *message.linear_acceleration_covariance))
    elif topic == "/synthetic/ground_truth":
        _hash_header(digest, message)
        _hash_string(digest, message.child_frame_id)
        _hash_vector3(digest, message.pose.pose.position)
        _hash_quaternion(digest, message.pose.pose.orientation)
        digest.update(struct.pack("<36d", *message.pose.covariance))
        _hash_vector3(digest, message.twist.twist.linear)
        _hash_vector3(digest, message.twist.twist.angular)
        digest.update(struct.pack("<36d", *message.twist.covariance))
    elif topic == "/pandar_points_ex":
        _hash_header(digest, message)
        digest.update(struct.pack("<II", message.height, message.width))
        digest.update(struct.pack("<Q", len(message.fields)))
        for field in message.fields:
            _hash_string(digest, field.name)
            digest.update(
                struct.pack("<I B I", field.offset, field.datatype, field.count)
            )
        digest.update(
            struct.pack(
                "<?II?",
                message.is_bigendian,
                message.point_step,
                message.row_step,
                message.is_dense,
            )
        )
        _hash_bytes(digest, bytes(message.data))
    else:
        raise RuntimeError(f"no semantic hash contract for topic: {topic}")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_manifest(
    output: Path,
    counts: dict[str, int],
    point_counts: list[int],
    stream_sha256: str,
) -> None:
    mcap_files = sorted(output.glob("*.mcap"))
    if len(mcap_files) != 1:
        raise RuntimeError(f"expected one MCAP file, found {len(mcap_files)}")
    metadata = output / "metadata.yaml"
    if not metadata.is_file():
        raise RuntimeError("rosbag2 did not create metadata.yaml")
    mcap = mcap_files[0]
    manifest = {
        "schema_version": 1,
        "dataset_id": DATASET_ID,
        "license": "Apache-2.0",
        "purpose": "Public deterministic LiDAR/IMU localization smoke fixture.",
        "provenance": {
            "generation": "fully_procedural",
            "source_messages_used": False,
            "source_coordinates_used": False,
            "source_trajectory_used": False,
            "source_gnss_used": False,
            "private_timestamp_used": False,
            "reference_scope": [
                "PointCloud2 field names, offsets, datatypes, and point_step",
                "rounded 20 Hz LiDAR and 200 Hz IMU rates",
            ],
            "generator_has_input_bag_option": False,
        },
        "generator": {
            "path": "tools/generate_synthetic_output_pointcloud2.py",
            "source_sha256": sha256(Path(__file__).resolve()),
            "version": GENERATOR_VERSION,
            "stochastic_seed": None,
            "ros_distro": "jazzy",
            "semantic_output_deterministic": True,
            "cdr_alignment_padding_zeroed": True,
        },
        "time": {
            "origin": "fixed synthetic ROS time; not a recording epoch",
            "first_record_ns": TF_STAMP_NS,
            "last_record_ns": IMU_END_NS,
            "duration_sec": round((IMU_END_NS - TF_STAMP_NS) * 1.0e-9, 9),
            "point_time_semantics": "scan-relative seconds",
        },
        "trajectory": {
            "model": "constant-speed planar circular arc",
            "speed_mps": SPEED_MPS,
            "yaw_rate_radps": YAW_RATE_RADPS,
            "ground_truth_topic": "/synthetic/ground_truth",
        },
        "scene": {
            "model": "analytic asymmetric corridor with ground, walls, poles, and boxes",
            "coordinate_source": "procedural_scene()",
        },
        "topics": [
            {
                "name": name,
                "type": type_name,
                "message_count": counts[name],
                "qos": (
                    {
                        "history": "keep_last",
                        "depth": 1,
                        "reliability": "reliable",
                        "durability": "transient_local",
                    }
                    if name == "/tf_static"
                    else {
                        "history": "keep_last",
                        "depth": 10,
                        "reliability": "reliable",
                        "durability": "volatile",
                    }
                ),
            }
            for _, name, type_name in TOPICS
        ],
        "pointcloud": {
            "topic": "/pandar_points_ex",
            "frame_id": "lidar/0",
            "little_endian": True,
            "point_step": POINT_STEP,
            "scan_period_sec": SCAN_PERIOD_NS * 1.0e-9,
            "point_time_span_sec": SCAN_DURATION_NS * 1.0e-9,
            "points_per_scan_min": min(point_counts),
            "points_per_scan_max": max(point_counts),
            "fields": [
                {"name": name, "offset": offset, "datatype": datatype, "count": 1}
                for name, offset, datatype in POINT_FIELDS
            ],
        },
        "interfaces": {
            "static_transforms": ["base_link -> lidar/0", "base_link -> imu"],
            "gnss_present": False,
            "raw_packets_present": False,
        },
        "integrity": {
            "canonical_semantic_stream_sha256": stream_sha256,
            "canonical_semantic_stream_contract": (
                "topic, type, record stamp, and explicit ROS fields; excludes CDR padding"
            ),
            "files": [
                {"path": mcap.name, "bytes": mcap.stat().st_size, "sha256": sha256(mcap)},
                {
                    "path": metadata.name,
                    "bytes": metadata.stat().st_size,
                    "sha256": sha256(metadata),
                },
            ],
        },
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def normalize_rosbag_metadata(
    rosbag2_py: object, output: Path, expected_message_count: int
) -> None:
    """Normalize a Jazzy SequentialWriter single-file count bug fail-closed."""

    metadata_io = rosbag2_py.MetadataIo()
    metadata = metadata_io.read_metadata(str(output))
    topic_total = sum(
        topic.message_count for topic in metadata.topics_with_message_count
    )
    if metadata.message_count != expected_message_count or topic_total != expected_message_count:
        raise RuntimeError(
            "rosbag2 metadata totals disagree with the messages written: "
            f"top={metadata.message_count}, topics={topic_total}, "
            f"expected={expected_message_count}"
        )
    if len(metadata.files) != 1:
        raise RuntimeError(
            f"expected one MCAP file in metadata, found {len(metadata.files)}"
        )
    file_count = metadata.files[0].message_count
    if file_count == 2 * expected_message_count:
        metadata.files[0].message_count = expected_message_count
        metadata_io.write_metadata(str(output), metadata)
    elif file_count != expected_message_count:
        raise RuntimeError(
            "unexpected per-file rosbag2 message count: "
            f"{file_count} (expected {expected_message_count})"
        )
    normalized = metadata_io.read_metadata(str(output))
    if normalized.files[0].message_count != expected_message_count:
        raise RuntimeError("failed to normalize per-file rosbag2 message count")


def generate(output: Path) -> None:
    if output.exists():
        raise RuntimeError(
            f"refusing to overwrite existing output directory: {output}. "
            "Choose a new --output path."
        )
    output.parent.mkdir(parents=True, exist_ok=True)

    (
        rosbag2_py,
        TransformStamped,
        Odometry,
        deserialize_message,
        serialize_message,
        Imu,
        PointCloud2,
        PointField,
        TFMessage,
    ) = import_ros()

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(output),
            storage_id="mcap",
            storage_preset_profile="zstd_small",
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types: dict[str, str] = {}
    for topic_id, name, type_name in TOPICS:
        offered_qos_profiles = [rosbag2_py._storage.QoS(10).reliable()]
        if name == "/tf_static":
            offered_qos_profiles = [
                rosbag2_py._storage.QoS(1).reliable().transient_local()
            ]
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=topic_id,
                name=name,
                type=type_name,
                serialization_format="cdr",
                offered_qos_profiles=offered_qos_profiles,
            )
        )
        topic_types[name] = type_name

    counts = {name: 0 for _, name, _ in TOPICS}
    point_counts: list[int] = []
    digest = hashlib.sha256()

    def write(topic: str, message: object, record_ns: int) -> None:
        serialized = sanitize_cdr_padding(topic, bytes(serialize_message(message)))
        if deserialize_message(serialized, message.__class__) != message:
            raise RuntimeError(f"CDR padding sanitization changed {topic}")
        writer.write(topic, serialized, record_ns)
        counts[topic] += 1
        update_semantic_stream_hash(
            digest, topic, topic_types[topic], record_ns, message
        )

    scene = procedural_scene()
    write(
        "/tf_static",
        make_static_tf(TransformStamped, TFMessage, TF_STAMP_NS),
        TF_STAMP_NS,
    )

    for stamp_ns in range(IMU_START_NS, IMU_END_NS + 1, IMU_PERIOD_NS):
        write("/sensor/imu/data_raw", make_imu(stamp_ns, Imu), stamp_ns)
        if SCAN_START_NS <= stamp_ns <= SCAN_END_NS and (
            stamp_ns - SCAN_START_NS
        ) % SCAN_PERIOD_NS == 0:
            ground_truth = make_ground_truth(stamp_ns, Odometry)
            cloud = make_cloud(stamp_ns, scene, PointCloud2, PointField)
            point_counts.append(cloud.width)
            write("/synthetic/ground_truth", ground_truth, stamp_ns)
            write("/pandar_points_ex", cloud, stamp_ns)

    writer.close()
    normalize_rosbag_metadata(rosbag2_py, output, sum(counts.values()))
    write_manifest(output, counts, point_counts, digest.hexdigest())

    print(f"Generated {DATASET_ID} at {output}")
    print(f"  scans: {counts['/pandar_points_ex']}")
    print(f"  IMU samples: {counts['/sensor/imu/data_raw']}")
    print(f"  points/scan: {min(point_counts)}..{max(point_counts)}")
    for path in sorted(output.iterdir()):
        print(f"  {path.name}: {path.stat().st_size} bytes")


def main() -> int:
    args = parse_args()
    try:
        generate(args.output.resolve())
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
