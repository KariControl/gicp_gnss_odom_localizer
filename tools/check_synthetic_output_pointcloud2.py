#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Validate the public procedural LiDAR/IMU fixture and an optional smoke run."""

from __future__ import annotations

import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
from typing import Any

# Importing the generator exposes the canonical fixture contract. Keep the
# documented checker read-only with respect to the source tree.
sys.dont_write_bytecode = True
import generate_synthetic_output_pointcloud2 as spec


EXPECTED_COUNTS = {
    "/tf_static": 1,
    "/sensor/imu/data_raw": 1241,
    "/synthetic/ground_truth": 121,
    "/pandar_points_ex": 121,
}
PRIVATE_TOKENS = (
    b"/home/",
    b"$GPGGA",
    b"$GNGGA",
    b"$GNRMC",
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def message_stamp_ns(message: Any) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def yaw_from_quaternion(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def angle_error(lhs: float, rhs: float) -> float:
    return math.atan2(math.sin(lhs - rhs), math.cos(lhs - rhs))


def import_ros() -> tuple[Any, ...]:
    try:
        import rosbag2_py
        from diagnostic_msgs.msg import DiagnosticArray
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from sensor_msgs.msg import Imu, PointCloud2
        from tf2_msgs.msg import TFMessage
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Jazzy Python modules are unavailable. Source "
            "/opt/ros/jazzy/setup.bash before running this checker."
        ) from exc
    return (
        rosbag2_py,
        deserialize_message,
        get_message,
        DiagnosticArray,
        Odometry,
        Imu,
        PointCloud2,
        TFMessage,
    )


def open_reader(rosbag2_py: Any, bag: Path) -> Any:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def validate_cloud(message: Any, record_ns: int, widths: list[int]) -> None:
    require(message_stamp_ns(message) == record_ns, "cloud record/header stamp mismatch")
    require(message.header.frame_id == "lidar/0", "unexpected cloud frame")
    require(message.height == 1 and message.width >= 1000, "invalid cloud dimensions")
    require(message.is_bigendian is False, "cloud must be little-endian")
    require(message.is_dense is True, "synthetic cloud must be dense")
    require(message.point_step == spec.POINT_STEP, "unexpected point_step")
    require(message.row_step == message.width * message.point_step, "invalid row_step")
    require(len(message.data) == message.row_step, "invalid cloud data length")
    actual_fields = tuple(
        (field.name, field.offset, field.datatype, field.count)
        for field in message.fields
    )
    expected_fields = tuple((*field, 1) for field in spec.POINT_FIELDS)
    require(actual_fields == expected_fields, "PointCloud2 field layout changed")

    previous_time = -math.inf
    first_time = None
    last_time = None
    for index in range(message.width):
        offset = index * message.point_step
        x = struct.unpack_from("<f", message.data, offset + 0)[0]
        y = struct.unpack_from("<f", message.data, offset + 4)[0]
        z = struct.unpack_from("<f", message.data, offset + 8)[0]
        intensity = struct.unpack_from("<f", message.data, offset + 16)[0]
        ring = struct.unpack_from("<H", message.data, offset + 20)[0]
        azimuth = struct.unpack_from("<f", message.data, offset + 24)[0]
        distance = struct.unpack_from("<f", message.data, offset + 28)[0]
        return_type = struct.unpack_from("<B", message.data, offset + 32)[0]
        point_time = struct.unpack_from("<d", message.data, offset + 40)[0]
        require(
            all(
                math.isfinite(value)
                for value in (x, y, z, intensity, azimuth, distance, point_time)
            ),
            "non-finite synthetic point value",
        )
        require(max(abs(x), abs(y), abs(z)) < 60.0, "synthetic coordinate outside bounds")
        require(0.0 <= intensity <= 255.0, "synthetic intensity outside bounds")
        require(0 <= ring <= 31 and return_type == 1, "invalid ring/return type")
        require(point_time >= previous_time, "point times are not monotonic")
        require(-1.0e-12 <= point_time <= 0.050000000001, "point time outside scan")
        calculated_distance = math.sqrt(x * x + y * y + z * z)
        require(abs(distance - calculated_distance) < 1.0e-4, "distance field is inconsistent")
        require(
            abs(angle_error(azimuth, math.atan2(y, x))) < 1.0e-5,
            "azimuth field is inconsistent",
        )
        if first_time is None:
            first_time = point_time
        last_time = point_time
        previous_time = point_time
    require(first_time == 0.0, "point time does not start at zero")
    require(last_time is not None and abs(last_time - 0.05) < 1.0e-12, "point time span changed")
    widths.append(message.width)


def validate_input_bag(bag: Path) -> dict[str, Any]:
    (
        rosbag2_py,
        deserialize_message,
        get_message,
        _,
        _,
        _,
        _,
        _,
    ) = import_ros()
    manifest_path = bag / "manifest.json"
    require(manifest_path.is_file(), "manifest.json is missing")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    require(manifest.get("dataset_id") == spec.DATASET_ID, "unexpected dataset_id")
    generator = manifest.get("generator", {})
    require(
        generator.get("source_sha256") == sha256(Path(spec.__file__).resolve()),
        "generator source hash mismatch",
    )
    require(
        generator.get("semantic_output_deterministic") is True,
        "semantic determinism is not declared",
    )
    require(
        generator.get("cdr_alignment_padding_zeroed") is True,
        "CDR padding is not declared sanitized",
    )
    provenance = manifest.get("provenance", {})
    require(provenance.get("generation") == "fully_procedural", "dataset is not procedural")
    for key in (
        "source_messages_used",
        "source_coordinates_used",
        "source_trajectory_used",
        "source_gnss_used",
        "private_timestamp_used",
    ):
        require(provenance.get(key) is False, f"unsafe provenance flag: {key}")
    require(
        provenance.get("generator_has_input_bag_option") is False,
        "generator input-bag contract changed",
    )

    integrity_files = manifest.get("integrity", {}).get("files", [])
    require(len(integrity_files) == 2, "unexpected integrity file set")
    for entry in integrity_files:
        path = bag / entry["path"]
        require(path.is_file(), f"missing fixture file: {entry['path']}")
        require(path.stat().st_size == entry["bytes"], f"size mismatch: {entry['path']}")
        require(sha256(path) == entry["sha256"], f"hash mismatch: {entry['path']}")

    for path in bag.iterdir():
        if not path.is_file():
            continue
        content = path.read_bytes()
        for token in PRIVATE_TOKENS:
            require(token not in content, f"private token {token!r} found in {path.name}")

    reader = open_reader(rosbag2_py, bag)
    bag_metadata = rosbag2_py.MetadataIo().read_metadata(str(bag))
    expected_total = sum(EXPECTED_COUNTS.values())
    require(bag_metadata.message_count == expected_total, "top-level metadata count mismatch")
    require(
        sum(item.message_count for item in bag_metadata.topics_with_message_count)
        == expected_total,
        "per-topic metadata counts do not match the bag total",
    )
    require(len(bag_metadata.files) == 1, "fixture must contain exactly one MCAP file")
    require(
        bag_metadata.files[0].message_count == expected_total,
        "per-file metadata count does not match the bag total",
    )
    metadata = reader.get_all_topics_and_types()
    actual_types = {item.name: item.type for item in metadata}
    expected_types = {name: type_name for _, name, type_name in spec.TOPICS}
    require(actual_types == expected_types, "fixture topic/type set changed")
    manifest_topics = {item["name"]: item for item in manifest.get("topics", [])}
    require(set(manifest_topics) == set(expected_types), "manifest topic set changed")
    for topic in expected_types:
        expected_qos = {
            "history": "keep_last",
            "depth": 1 if topic == "/tf_static" else 10,
            "reliability": "reliable",
            "durability": "transient_local" if topic == "/tf_static" else "volatile",
        }
        require(
            manifest_topics[topic].get("qos") == expected_qos,
            f"manifest QoS contract changed on {topic}",
        )
    message_classes = {topic: get_message(type_name) for topic, type_name in actual_types.items()}
    counts: collections.Counter[str] = collections.Counter()
    stamps: dict[str, list[int]] = collections.defaultdict(list)
    widths: list[int] = []
    stream_digest = hashlib.sha256()
    static_children: set[str] = set()
    last_ground_truth = None

    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        require(
            spec.sanitize_cdr_padding(topic, bytes(serialized)) == bytes(serialized),
            f"non-zero CDR alignment padding on {topic}",
        )
        message = deserialize_message(serialized, message_classes[topic])
        counts[topic] += 1
        stamps[topic].append(record_ns)
        spec.update_semantic_stream_hash(
            stream_digest, topic, actual_types[topic], record_ns, message
        )
        if topic == "/pandar_points_ex":
            validate_cloud(message, record_ns, widths)
        elif topic == "/sensor/imu/data_raw":
            require(message_stamp_ns(message) == record_ns, "IMU record/header stamp mismatch")
            require(message.header.frame_id == "imu", "unexpected IMU frame")
            values = (
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            )
            require(all(math.isfinite(value) for value in values), "non-finite IMU value")
            require(
                abs(message.angular_velocity.z - spec.YAW_RATE_RADPS) < 1.0e-12,
                "gyro-Z changed",
            )
        elif topic == "/synthetic/ground_truth":
            require(message_stamp_ns(message) == record_ns, "ground-truth stamp mismatch")
            require(
                message.header.frame_id == "odom" and message.child_frame_id == "base_link",
                "unexpected ground-truth frames",
            )
            expected_x, expected_y, expected_yaw = spec.pose_at(record_ns)
            require(
                abs(message.pose.pose.position.x - expected_x) < 1.0e-12,
                "ground-truth X mismatch",
            )
            require(
                abs(message.pose.pose.position.y - expected_y) < 1.0e-12,
                "ground-truth Y mismatch",
            )
            require(
                abs(
                    angle_error(
                        yaw_from_quaternion(message.pose.pose.orientation),
                        expected_yaw,
                    )
                )
                < 1.0e-12,
                "ground-truth yaw mismatch",
            )
            last_ground_truth = message
        elif topic == "/tf_static":
            require(record_ns == spec.TF_STAMP_NS, "static TF stamp changed")
            for transform in message.transforms:
                require(transform.header.frame_id == "base_link", "unexpected TF parent")
                require(message_stamp_ns(transform) == record_ns, "TF stamp mismatch")
                require(transform.transform.translation.x == 0.0, "non-identity TF translation")
                require(transform.transform.translation.y == 0.0, "non-identity TF translation")
                require(transform.transform.translation.z == 0.0, "non-identity TF translation")
                require(transform.transform.rotation.w == 1.0, "non-identity TF rotation")
                static_children.add(transform.child_frame_id)

    require(dict(counts) == EXPECTED_COUNTS, f"unexpected message counts: {dict(counts)}")
    for topic, topic_stamps in stamps.items():
        require(topic_stamps == sorted(topic_stamps), f"non-monotonic records on {topic}")
    require(stamps["/tf_static"][0] < stamps["/sensor/imu/data_raw"][0], "TF lacks IMU lead-in")
    require(
        stamps["/sensor/imu/data_raw"][0] < stamps["/pandar_points_ex"][0],
        "IMU lacks scan lead-in",
    )
    require(
        all(
            right - left == spec.IMU_PERIOD_NS
            for left, right in zip(
                stamps["/sensor/imu/data_raw"], stamps["/sensor/imu/data_raw"][1:]
            )
        ),
        "IMU cadence changed",
    )
    require(
        all(
            right - left == spec.SCAN_PERIOD_NS
            for left, right in zip(
                stamps["/pandar_points_ex"], stamps["/pandar_points_ex"][1:]
            )
        ),
        "LiDAR cadence changed",
    )
    require(static_children == {"lidar/0", "imu"}, "static TF set changed")
    require(last_ground_truth is not None, "ground truth is missing")
    require(
        stream_digest.hexdigest()
        == manifest["integrity"]["canonical_semantic_stream_sha256"],
        "canonical semantic stream hash mismatch",
    )
    pointcloud_manifest = manifest.get("pointcloud", {})
    require(
        min(widths) == pointcloud_manifest.get("points_per_scan_min"),
        "minimum width mismatch",
    )
    require(
        max(widths) == pointcloud_manifest.get("points_per_scan_max"),
        "maximum width mismatch",
    )

    return {
        "counts": dict(counts),
        "points_per_scan": [min(widths), max(widths)],
        "last_ground_truth": last_ground_truth,
    }


def validate_runtime_result(result_bag: Path, input_summary: dict[str, Any]) -> None:
    (
        rosbag2_py,
        deserialize_message,
        get_message,
        DiagnosticArray,
        Odometry,
        Imu,
        PointCloud2,
        _,
    ) = import_ros()
    reader = open_reader(rosbag2_py, result_bag)
    actual_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_classes = {topic: get_message(type_name) for topic, type_name in actual_types.items()}
    require("/diagnostics" in actual_types, "runtime result lacks /diagnostics")
    require("/localization/gyro_lidar_odom" in actual_types, "runtime result lacks odometry")

    deskew_statuses = []
    gyro_statuses = []
    odometry = []
    corrected_imu = 0
    deskewed_clouds = 0
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        message = deserialize_message(serialized, message_classes[topic])
        if topic == "/diagnostics":
            require(isinstance(message, DiagnosticArray), "diagnostic type mismatch")
            for status in message.status:
                if status.name == "localization/imu_undistortion":
                    deskew_statuses.append(status)
                elif status.name == "localization/gyro_odometer":
                    gyro_statuses.append(status)
        elif topic == "/localization/gyro_lidar_odom":
            require(isinstance(message, Odometry), "odometry type mismatch")
            odometry.append(message)
        elif topic == "/localization/imu_corrected":
            require(isinstance(message, Imu), "corrected IMU type mismatch")
            corrected_imu += 1
        elif topic == "/localization/points_undistorted":
            require(isinstance(message, PointCloud2), "deskewed cloud type mismatch")
            deskewed_clouds += 1

    require(
        len(deskew_statuses) == EXPECTED_COUNTS["/pandar_points_ex"],
        "not every scan was deskewed",
    )
    for status in deskew_statuses:
        values = {value.key: value.value for value in status.values}
        require(int.from_bytes(status.level, "little") == 0, "deskew reported non-OK status")
        require(
            status.message == "deskew OK using per-point timestamps",
            "unexpected deskew result",
        )
        require(values.get("time_field_name") == "time_stamp", "wrong time field used")
        require(values.get("used_linear_fallback") == "false", "deskew used point-order fallback")
        require(values.get("interpreted_as_relative") == "true", "point time was not relative")
        require(values.get("reference_time") == "start", "wrong deskew reference")
    require(gyro_statuses, "odometer diagnostics are missing")
    last_values = {value.key: value.value for value in gyro_statuses[-1].values}
    require(gyro_statuses[-1].message == "running", "odometer did not finish running")
    require(last_values.get("lidar_valid") == "true", "final registration is invalid")
    require(last_values.get("lidar_converged") == "true", "final registration did not converge")
    require(last_values.get("lidar_rejection_reason") == "accepted", "final scan was rejected")
    require(last_values.get("imu_extrinsic_cached") == "true", "IMU TF was not cached")
    accepted = int(last_values.get("external_submap_snapshot_sequence", "0"))
    attempted = EXPECTED_COUNTS["/pandar_points_ex"] - 1
    require(accepted / attempted >= 0.95, "registration acceptance density is below 95%")
    require(
        corrected_imu / EXPECTED_COUNTS["/sensor/imu/data_raw"] >= 0.99,
        "corrected IMU coverage is below 99%",
    )
    if "/localization/points_undistorted" in actual_types:
        require(
            deskewed_clouds == EXPECTED_COUNTS["/pandar_points_ex"],
            "deskewed cloud count mismatch",
        )
    require(odometry, "odometry output is empty")
    final = odometry[-1]
    require(
        final.header.frame_id == "odom" and final.child_frame_id == "base_link",
        "wrong odometry frames",
    )
    values = (
        final.pose.pose.position.x,
        final.pose.pose.position.y,
        final.pose.pose.orientation.x,
        final.pose.pose.orientation.y,
        final.pose.pose.orientation.z,
        final.pose.pose.orientation.w,
    )
    require(all(math.isfinite(value) for value in values), "non-finite final odometry")
    truth = input_summary["last_ground_truth"].pose.pose
    xy_error = math.hypot(
        final.pose.pose.position.x - truth.position.x,
        final.pose.pose.position.y - truth.position.y,
    )
    yaw_error = abs(
        angle_error(
            yaw_from_quaternion(final.pose.pose.orientation),
            yaw_from_quaternion(truth.orientation),
        )
    )
    require(xy_error < 0.5, f"coarse endpoint XY error is {xy_error:.6f} m")
    require(
        yaw_error < math.radians(2.0),
        f"coarse endpoint yaw error is {math.degrees(yaw_error):.6f} deg",
    )
    print(
        "Runtime smoke: "
        f"deskew={len(deskew_statuses)}/{EXPECTED_COUNTS['/pandar_points_ex']}, "
        f"registration={accepted}/{attempted}, corrected_imu={corrected_imu}, "
        f"endpoint_xy={xy_error:.4f} m, endpoint_yaw={math.degrees(yaw_error):.4f} deg"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bag",
        type=Path,
        default=spec.DEFAULT_OUTPUT,
        help=f"synthetic input bag (default: {spec.DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "--runtime-result",
        type=Path,
        help="optional rosbag containing localization outputs and diagnostics",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        summary = validate_input_bag(args.bag.resolve())
        print(
            "Synthetic input: PASS "
            f"({summary['counts']['/pandar_points_ex']} scans, "
            f"{summary['counts']['/sensor/imu/data_raw']} IMU samples, "
            f"{summary['points_per_scan'][0]}..{summary['points_per_scan'][1]} points/scan)"
        )
        if args.runtime_result is not None:
            validate_runtime_result(args.runtime_result.resolve(), summary)
    except (KeyError, OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
