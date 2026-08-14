#!/usr/bin/env python3
"""Fail-closed runtime audit for isolated LiDAR/IMU-only odometry runs."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


@dataclass(frozen=True)
class Check:
    name: str
    passed: bool
    detail: str
    category: str = "hard"


def resolve_bag(path: Path) -> Path:
    path = path.resolve()
    if path.is_file() and path.suffix == ".mcap":
        return path
    if not path.is_dir():
        raise RuntimeError(f"bag path does not exist: {path}")
    if (path / "metadata.yaml").is_file():
        return path
    files = sorted(path.glob("*.mcap"))
    if len(files) == 1:
        return files[0]
    raise RuntimeError(f"cannot resolve bag under {path}: found {len(files)} MCAP files")


def open_bag(path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        # Let rosbag2 select the storage plugin from metadata or file magic.
        # Evaluation inputs may be sqlite3 while recorded outputs use MCAP.
        rosbag2_py.StorageOptions(uri=str(resolve_bag(path)), storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def read_env(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.lstrip().startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key] = value
    return result


def message_types(reader: rosbag2_py.SequentialReader) -> dict[str, str]:
    return {item.name: item.type for item in reader.get_all_topics_and_types()}


def read_input(
    path: Path, points_topic: str, imu_topic: str
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, str]]:
    reader = open_bag(path)
    types = message_types(reader)
    expected = {
        points_topic: "sensor_msgs/msg/PointCloud2",
        imu_topic: "sensor_msgs/msg/Imu",
    }
    for topic, expected_type in expected.items():
        if types.get(topic) != expected_type:
            raise RuntimeError(
                f"missing input {topic} with type {expected_type}; found {types.get(topic)!r}"
            )
    classes = {topic: get_message(type_name) for topic, type_name in expected.items()}
    points: list[int] = []
    imu: list[int] = []
    acceleration_norm: list[float] = []
    frames: dict[str, str] = {}
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in classes:
            continue
        message = deserialize_message(serialized, classes[topic])
        value = stamp_ns(message)
        if topic == points_topic:
            points.append(value)
            frames.setdefault("points", message.header.frame_id)
        else:
            imu.append(value)
            frames.setdefault("imu", message.header.frame_id)
            vector = message.linear_acceleration
            acceleration_norm.append(
                math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
            )
    return (
        np.asarray(points, dtype=np.int64),
        np.asarray(imu, dtype=np.int64),
        np.asarray(acceleration_norm, dtype=float),
        frames,
    )


def read_result(path: Path) -> dict[str, Any]:
    reader = open_bag(path)
    types = message_types(reader)
    required = {
        "/clock": "rosgraph_msgs/msg/Clock",
        "/diagnostics": "diagnostic_msgs/msg/DiagnosticArray",
        "/localization/gyro_lidar_odom": "nav_msgs/msg/Odometry",
        "/localization/gyro_lidar_odom_scan": "nav_msgs/msg/Odometry",
        "/localization/imu_corrected": "sensor_msgs/msg/Imu",
        "/localization/is_stopped": "std_msgs/msg/Bool",
    }
    for topic, expected_type in required.items():
        if types.get(topic) != expected_type:
            raise RuntimeError(
                f"missing result {topic} with type {expected_type}; found {types.get(topic)!r}"
            )
    classes = {topic: get_message(type_name) for topic, type_name in required.items()}
    result: dict[str, Any] = {
        "topic_types": types,
        "clock": [],
        "odom": [],
        "accepted_scan_odom": [],
        "corrected_imu": [],
        "stopped": [],
        "diagnostics": [],
    }
    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        if topic not in classes:
            continue
        message = deserialize_message(serialized, classes[topic])
        if topic == "/clock":
            sim_ns = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)
            result["clock"].append((record_ns, sim_ns))
        elif topic in (
            "/localization/gyro_lidar_odom",
            "/localization/gyro_lidar_odom_scan",
        ):
            position = message.pose.pose.position
            quaternion = message.pose.pose.orientation
            linear = message.twist.twist.linear
            angular = message.twist.twist.angular
            values = (
                float(position.x),
                float(position.y),
                float(position.z),
                float(quaternion.x),
                float(quaternion.y),
                float(quaternion.z),
                float(quaternion.w),
            )
            if topic == "/localization/gyro_lidar_odom_scan":
                values += (
                    float(linear.x),
                    float(linear.y),
                    float(linear.z),
                    float(angular.x),
                    float(angular.y),
                    float(angular.z),
                    *(float(value) for value in message.pose.covariance),
                    *(float(value) for value in message.twist.covariance),
                )
            result_key = (
                "accepted_scan_odom"
                if topic == "/localization/gyro_lidar_odom_scan"
                else "odom"
            )
            result[result_key].append(
                (
                    record_ns,
                    stamp_ns(message),
                    message.header.frame_id,
                    message.child_frame_id,
                    values,
                )
            )
        elif topic == "/localization/imu_corrected":
            vector = message.linear_acceleration
            norm = math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
            result["corrected_imu"].append(
                (record_ns, stamp_ns(message), message.header.frame_id, norm)
            )
        elif topic == "/localization/is_stopped":
            result["stopped"].append((record_ns, bool(message.data)))
        else:
            array_stamp = stamp_ns(message)
            for status in message.status:
                if status.name not in (
                    "localization/imu_undistortion",
                    "localization/gyro_odometer",
                ):
                    continue
                result["diagnostics"].append(
                    {
                        "record_ns": record_ns,
                        "stamp_ns": array_stamp,
                        "name": status.name,
                        "level": int(status.level[0]) if isinstance(status.level, bytes) else int(status.level),
                        "message": status.message,
                        "values": {item.key: item.value for item in status.values},
                    }
                )
    return result


def count_backsteps(values: np.ndarray) -> int:
    return int(np.count_nonzero(np.diff(values) < 0)) if values.size > 1 else 0


def percentile(values: np.ndarray, probability: float) -> float | None:
    finite = values[np.isfinite(values)]
    return float(np.quantile(finite, probability)) if finite.size else None


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    env = read_env(args.run_env)
    points_topic = env.get("points_topic")
    imu_topic = env.get("imu_topic")
    if not points_topic or not imu_topic:
        raise RuntimeError("run.env does not contain points_topic and imu_topic")
    use_deskew = env.get("use_deskew") == "true"
    try:
        expected_acceleration_scale = float(
            env.get(
                "imu_acceleration_scale",
                "9.80665" if args.sensor == "mid360" else "1.0",
            )
        )
        required_deskew_success = float(
            env.get(
                "deskew_success_minimum",
                "0.99" if args.sensor == "mid360" else "0.90",
            )
        )
    except ValueError as error:
        raise RuntimeError(
            "run.env contains a non-numeric IMU scale or deskew threshold"
        ) from error
    if not math.isfinite(expected_acceleration_scale) or expected_acceleration_scale <= 0.0:
        raise RuntimeError("run.env imu_acceleration_scale must be positive and finite")
    if not math.isfinite(required_deskew_success) or not (
        0.0 < required_deskew_success <= 1.0
    ):
        raise RuntimeError("run.env deskew_success_minimum must be in (0, 1]")
    source_points, source_imu, source_acceleration_norm, input_frames = read_input(
        args.input_bag, points_topic, imu_topic
    )
    result = read_result(args.result_bag)
    checks: list[Check] = []

    clock = np.asarray(result["clock"], dtype=np.int64)
    if clock.ndim != 2 or len(clock) < 2:
        raise RuntimeError("result contains fewer than two /clock messages")
    positive = clock[:, 1] > 0
    clock = clock[positive]
    if len(clock) < 2:
        raise RuntimeError("result contains fewer than two positive /clock messages")
    clock_record_ns = clock[:, 0]
    clock_sim_ns = clock[:, 1]
    start_ns = int(clock_sim_ns[0])
    end_ns = int(clock_sim_ns[-1])
    clock_backsteps = count_backsteps(clock_sim_ns)
    sim_duration = (end_ns - start_ns) * 1.0e-9
    wall_duration = (int(clock_record_ns[-1]) - int(clock_record_ns[0])) * 1.0e-9
    observed_rate = sim_duration / wall_duration if wall_duration > 0.0 else math.nan
    checks.append(Check("clock_monotonic", clock_backsteps == 0, f"backsteps={clock_backsteps}"))
    checks.append(
        Check(
            "one_x_realtime",
            math.isfinite(observed_rate) and 0.90 <= observed_rate <= 1.10,
            f"sim/wall={observed_rate:.6f}, sim={sim_duration:.3f}s, wall={wall_duration:.3f}s",
        )
    )

    source_points_run = source_points[(source_points >= start_ns) & (source_points <= end_ns)]
    source_imu_run = source_imu[(source_imu >= start_ns) & (source_imu <= end_ns)]
    corrected = [item for item in result["corrected_imu"] if start_ns <= item[1] <= end_ns]
    corrected_stamp = np.asarray([item[1] for item in corrected], dtype=np.int64)
    corrected_set = set(int(value) for value in corrected_stamp)
    source_imu_set = set(int(value) for value in source_imu_run)
    imu_coverage = len(corrected_set & source_imu_set) / max(1, len(source_imu_set))
    corrected_backsteps = count_backsteps(corrected_stamp)
    corrected_duplicates = len(corrected_stamp) - len(corrected_set)
    acceleration_norm = np.asarray([item[3] for item in corrected], dtype=float)
    acceleration_median = float(np.median(acceleration_norm)) if acceleration_norm.size else math.nan
    checks.extend(
        [
            Check(
                "corrected_imu_exact_stamp_coverage",
                imu_coverage >= 0.99,
                f"coverage={imu_coverage:.6%} ({len(corrected_set & source_imu_set)}/{len(source_imu_set)})",
            ),
            Check(
                "corrected_imu_strict_stamp_order",
                corrected_backsteps == 0 and corrected_duplicates == 0,
                f"backsteps={corrected_backsteps}, duplicates={corrected_duplicates}",
            ),
            Check(
                "corrected_imu_si_acceleration",
                math.isfinite(acceleration_median) and 7.0 <= acceleration_median <= 12.0,
                f"median_norm={acceleration_median:.6f} m/s^2",
            ),
        ]
    )

    odom = [item for item in result["odom"] if start_ns <= item[1] <= end_ns]
    odom_stamp = np.asarray([item[1] for item in odom], dtype=np.int64)
    odom_backsteps = count_backsteps(odom_stamp)
    odom_values = np.asarray([item[4] for item in odom], dtype=float)
    finite_odom = bool(odom_values.size and np.all(np.isfinite(odom_values)))
    quaternion_norm = (
        np.linalg.norm(odom_values[:, 3:7], axis=1) if odom_values.size else np.array([])
    )
    odom_coverage = (
        (int(odom_stamp[-1]) - int(odom_stamp[0])) / max(1, end_ns - start_ns)
        if len(odom_stamp) >= 2
        else 0.0
    )
    odom_frames = Counter((item[2], item[3]) for item in odom)
    checks.extend(
        [
            Check(
                "local_odom_stamp_order",
                len(odom_stamp) > 0 and odom_backsteps == 0,
                f"messages={len(odom_stamp)}, unique={len(set(map(int, odom_stamp)))}, backsteps={odom_backsteps}",
            ),
            Check(
                "local_odom_finite_unit_quaternion",
                finite_odom
                and quaternion_norm.size > 0
                and bool(np.all(np.abs(quaternion_norm - 1.0) <= 1.0e-6)),
                f"finite={finite_odom}, max_norm_error={float(np.max(np.abs(quaternion_norm - 1.0))) if quaternion_norm.size else math.nan:.3e}",
            ),
            Check(
                "local_odom_time_coverage",
                odom_coverage >= 0.98,
                f"coverage={odom_coverage:.6%}",
            ),
            Check(
                "local_odom_frames",
                len(odom_frames) == 1 and next(iter(odom_frames), None) == ("odom", "base_link"),
                f"frames={dict(odom_frames)}",
            ),
        ]
    )

    accepted_scan_odom = [
        item
        for item in result["accepted_scan_odom"]
        if start_ns <= item[1] <= end_ns
    ]
    accepted_scan_record_ns = np.asarray(
        [item[0] for item in accepted_scan_odom], dtype=np.int64
    )
    accepted_scan_stamp = np.asarray(
        [item[1] for item in accepted_scan_odom], dtype=np.int64
    )
    accepted_scan_set = set(map(int, accepted_scan_stamp))
    accepted_scan_backsteps = count_backsteps(accepted_scan_stamp)
    accepted_scan_duplicates = len(accepted_scan_stamp) - len(accepted_scan_set)
    if len(accepted_scan_stamp) and len(source_points_run):
        sorted_source_points = np.sort(source_points_run)
        insertion = np.searchsorted(sorted_source_points, accepted_scan_stamp)
        left = sorted_source_points[
            np.clip(insertion - 1, 0, len(sorted_source_points) - 1)
        ]
        right = sorted_source_points[
            np.clip(insertion, 0, len(sorted_source_points) - 1)
        ]
        accepted_scan_source_delta_ns = np.minimum(
            np.abs(accepted_scan_stamp - left),
            np.abs(accepted_scan_stamp - right),
        )
    else:
        accepted_scan_source_delta_ns = np.array([], dtype=np.int64)
    maximum_source_delta_ns = (
        int(np.max(accepted_scan_source_delta_ns))
        if len(accepted_scan_source_delta_ns)
        else -1
    )
    accepted_scan_values = np.asarray(
        [item[4] for item in accepted_scan_odom], dtype=float
    )
    accepted_scan_finite = bool(
        accepted_scan_values.size and np.all(np.isfinite(accepted_scan_values))
    )
    accepted_scan_quaternion_norm = (
        np.linalg.norm(accepted_scan_values[:, 3:7], axis=1)
        if accepted_scan_values.size
        else np.array([])
    )
    accepted_scan_frames = Counter(
        (item[2], item[3]) for item in accepted_scan_odom
    )
    checks.extend(
        [
            Check(
                "accepted_scan_odom_strict_stamps",
                len(accepted_scan_stamp) > 0
                and accepted_scan_backsteps == 0
                and accepted_scan_duplicates == 0,
                f"messages={len(accepted_scan_stamp)}, "
                f"backsteps={accepted_scan_backsteps}, "
                f"duplicates={accepted_scan_duplicates}",
            ),
            Check(
                "accepted_scan_odom_physical_reference_stamp",
                # The deskewer reconstructs an rclcpp::Time from seconds, so
                # epoch-scale input stamps can be rounded by one double ULP
                # (256 ns in the audited bags).  Keep this bounded tightly
                # while verifying the configured physical scan reference.
                0 <= maximum_source_delta_ns <= 1_000,
                f"reference={'start' if args.sensor == 'mid360' else 'end'}, "
                f"nearest_raw_header_max_delta_ns={maximum_source_delta_ns}, "
                "tolerance_ns=1000",
            ),
            Check(
                "accepted_scan_odom_frames",
                len(accepted_scan_frames) == 1
                and next(iter(accepted_scan_frames), None)
                == ("odom", "base_link"),
                f"frames={dict(accepted_scan_frames)}",
            ),
            Check(
                "accepted_scan_odom_finite_unit_quaternion",
                accepted_scan_finite
                and accepted_scan_quaternion_norm.size > 0
                and bool(
                    np.all(
                        np.abs(accepted_scan_quaternion_norm - 1.0) <= 1.0e-6
                    )
                ),
                f"finite={accepted_scan_finite}, "
                "max_norm_error="
                f"{float(np.max(np.abs(accepted_scan_quaternion_norm - 1.0))) if accepted_scan_quaternion_norm.size else math.nan:.3e}",
            ),
        ]
    )

    diagnostics = [
        item for item in result["diagnostics"] if start_ns <= item["stamp_ns"] <= end_ns
    ]
    deskew_diagnostics = [
        item for item in diagnostics if item["name"] == "localization/imu_undistortion"
    ]
    deskew_ok = [item for item in deskew_diagnostics if item["message"].startswith("deskew OK")]
    deskew_reject = [item for item in deskew_diagnostics if item not in deskew_ok]
    deskew_reasons = Counter(item["message"] for item in deskew_reject)
    deskew_diag_coverage = len(deskew_diagnostics) / max(1, len(source_points_run))
    deskew_success_rate = len(deskew_ok) / max(1, len(deskew_diagnostics))
    if use_deskew:
        checks.extend(
            [
                Check(
                    "deskew_diagnostic_input_coverage",
                    deskew_diag_coverage >= 0.98,
                    f"coverage={deskew_diag_coverage:.6%} ({len(deskew_diagnostics)}/{len(source_points_run)})",
                ),
                Check(
                    "deskew_success_rate",
                    deskew_success_rate >= required_deskew_success,
                    f"rate={deskew_success_rate:.6%} ({len(deskew_ok)}/{len(deskew_diagnostics)}), threshold={required_deskew_success:.2%}",
                ),
            ]
        )
        latest_deskew = deskew_diagnostics[-1]["values"] if deskew_diagnostics else {}
        expected_field = "timestamp" if args.sensor == "mid360" else "time"
        expected_reference = "start" if args.sensor == "mid360" else "end"
        checks.append(
            Check(
                "deskew_time_contract",
                latest_deskew.get("time_field_name") == expected_field
                and latest_deskew.get("reference_time") == expected_reference
                and latest_deskew.get("used_linear_fallback") == "false",
                f"field={latest_deskew.get('time_field_name')}, reference={latest_deskew.get('reference_time')}, fallback={latest_deskew.get('used_linear_fallback')}",
            )
        )
    else:
        checks.append(
            Check(
                "deskew_disabled",
                not deskew_diagnostics,
                f"unexpected_diagnostics={len(deskew_diagnostics)}",
            )
        )

    odom_diagnostics = [
        item for item in diagnostics if item["name"] == "localization/gyro_odometer"
    ]
    final_odom_diagnostic = (
        max(odom_diagnostics, key=lambda item: item["record_ns"])
        if odom_diagnostics
        else None
    )
    latest_odom_diag = final_odom_diagnostic["values"] if final_odom_diagnostic else {}
    try:
        diagnosed_scale = float(latest_odom_diag.get("imu_corrected.linear_acceleration_scale", "nan"))
        accepted_sequence = int(latest_odom_diag.get("external_submap_snapshot_sequence", "0"))
        diagnosed_accepted_count = int(
            latest_odom_diag.get("accepted_scan_odom_published_count", "-1")
        )
    except ValueError:
        diagnosed_scale = math.nan
        accepted_sequence = 0
        diagnosed_accepted_count = -1
    final_diag_record_ns = (
        int(final_odom_diagnostic["record_ns"])
        if final_odom_diagnostic is not None
        else -1
    )
    last_accepted_record_ns = (
        int(accepted_scan_record_ns[-1]) if len(accepted_scan_record_ns) else -1
    )
    final_diag_is_after_accepted_tail = (
        final_diag_record_ns >= last_accepted_record_ns >= 0
    )
    accepted_count_matches = (
        len(accepted_scan_stamp)
        == diagnosed_accepted_count
        == accepted_sequence
    )
    input_to_odometer = len(deskew_ok) if use_deskew else len(source_points_run)
    registration_density = accepted_sequence / max(1, input_to_odometer - 1)
    checks.extend(
        [
            Check(
                "imu_acceleration_scale_contract",
                math.isfinite(diagnosed_scale)
                and abs(diagnosed_scale - expected_acceleration_scale) <= 1.0e-6,
                f"diagnosed={diagnosed_scale}, expected={expected_acceleration_scale}",
            ),
            Check(
                "scan_registration_density",
                registration_density >= 0.95,
                f"accepted_sequence={accepted_sequence}, odometer_inputs={input_to_odometer}, "
                f"density={registration_density:.6%}, threshold=95.00%",
            ),
            Check(
                "accepted_scan_odom_diagnostic_contract",
                latest_odom_diag.get("accepted_scan_odom_enabled") == "true"
                and latest_odom_diag.get("accepted_scan_odom_topic")
                == "/localization/gyro_lidar_odom_scan"
                and latest_odom_diag.get("accepted_scan_odom_stamp_contract")
                == "accepted_input_scan_header_stamp",
                "enabled="
                f"{latest_odom_diag.get('accepted_scan_odom_enabled')}, "
                f"topic={latest_odom_diag.get('accepted_scan_odom_topic')}, "
                f"stamp_contract={latest_odom_diag.get('accepted_scan_odom_stamp_contract')}",
            ),
            Check(
                "accepted_scan_odom_final_count",
                final_diag_is_after_accepted_tail and accepted_count_matches,
                f"recorded={len(accepted_scan_stamp)}, "
                f"diagnosed_published={diagnosed_accepted_count}, "
                f"accepted_sequence={accepted_sequence}, "
                f"final_diag_after_tail={final_diag_is_after_accepted_tail}, "
                f"tail_record_delta_ms={(final_diag_record_ns - last_accepted_record_ns) * 1.0e-6:.3f}",
            ),
            Check(
                "scan_to_scan_only",
                latest_odom_diag.get("lidar_tracking_mode") == "scan_to_scan",
                f"mode={latest_odom_diag.get('lidar_tracking_mode')}",
            ),
        ]
    )

    stopped_values = [item[1] for item in result["stopped"]]
    stopped_fraction = sum(stopped_values) / max(1, len(stopped_values))
    message_age = np.asarray(
        [float(item["values"].get("msg_age_ms", "nan")) for item in deskew_ok], dtype=float
    )
    hard_passed = all(item.passed for item in checks if item.category == "hard")
    return {
        "passed": hard_passed,
        "sensor": args.sensor,
        "inputs": {
            "input_bag": str(args.input_bag.resolve()),
            "result_bag": str(args.result_bag.resolve()),
            "run_env": str(args.run_env.resolve()),
            "points_topic": points_topic,
            "imu_topic": imu_topic,
            "frames": input_frames,
            "source_points_total": int(len(source_points)),
            "source_imu_total": int(len(source_imu)),
            "source_acceleration_norm_median_raw_units": float(np.median(source_acceleration_norm)),
            "recorded_tf_used": env.get("recorded_tf_used"),
        },
        "clock": {
            "start_stamp_sec": start_ns * 1.0e-9,
            "end_stamp_sec": end_ns * 1.0e-9,
            "sim_duration_sec": sim_duration,
            "record_wall_duration_sec": wall_duration,
            "observed_sim_over_wall_rate": observed_rate,
            "messages": int(len(clock)),
            "backsteps": clock_backsteps,
        },
        "imu": {
            "source_in_clock_range": int(len(source_imu_run)),
            "corrected_in_clock_range": int(len(corrected_stamp)),
            "exact_stamp_coverage": imu_coverage,
            "backsteps": corrected_backsteps,
            "duplicates": corrected_duplicates,
            "corrected_acceleration_norm_median_mps2": acceleration_median,
        },
        "deskew": {
            "enabled": use_deskew,
            "source_points_in_clock_range": int(len(source_points_run)),
            "diagnostics": int(len(deskew_diagnostics)),
            "accepted": int(len(deskew_ok)),
            "rejected": int(len(deskew_reject)),
            "success_rate": deskew_success_rate if use_deskew else None,
            "rejection_reasons": dict(deskew_reasons),
            "message_age_ms": {
                "median": percentile(message_age, 0.5),
                "p95": percentile(message_age, 0.95),
                "p99": percentile(message_age, 0.99),
                "max": float(np.nanmax(message_age)) if np.any(np.isfinite(message_age)) else None,
            },
        },
        "odometry": {
            "messages": int(len(odom_stamp)),
            "unique_stamps": int(len(set(map(int, odom_stamp)))),
            "accepted_scan_messages": int(len(accepted_scan_stamp)),
            "accepted_scan_unique_stamps": int(len(accepted_scan_set)),
            "accepted_scan_diagnosed_published_count": diagnosed_accepted_count,
            "accepted_scan_sequence": accepted_sequence,
            "accepted_scan_final_diag_after_tail": final_diag_is_after_accepted_tail,
            "accepted_scan_final_diag_tail_delta_ms": (
                (final_diag_record_ns - last_accepted_record_ns) * 1.0e-6
                if last_accepted_record_ns >= 0
                else None
            ),
            "scan_registration_density": registration_density,
            "time_coverage": odom_coverage,
            "frames": {f"{parent}->{child}": count for (parent, child), count in odom_frames.items()},
            "stopped_publish_fraction": stopped_fraction,
        },
        "checks": [asdict(item) for item in checks],
    }


def write_report(path: Path, result: dict[str, Any]) -> None:
    lines = [
        f"# {result['sensor']} LiDAR/IMU-only runtime audit",
        "",
        f"Overall: **{'PASS' if result['passed'] else 'FAIL'}**",
        "",
        "| Check | Result | Detail |",
        "|---|---|---|",
    ]
    for check in result["checks"]:
        lines.append(
            f"| {check['name']} | {'PASS' if check['passed'] else 'FAIL'} | {check['detail']} |"
        )
    deskew = result["deskew"]
    lines.extend(
        [
            "",
            "## Runtime summary",
            "",
            f"- 1x sim/wall ratio: {result['clock']['observed_sim_over_wall_rate']:.6f}",
            f"- corrected IMU exact-stamp coverage: {result['imu']['exact_stamp_coverage']:.6%}",
            f"- corrected acceleration norm median: {result['imu']['corrected_acceleration_norm_median_mps2']:.6f} m/s²",
            "- accepted-scan odometry recorded/diagnosed/sequence: "
            f"{result['odometry']['accepted_scan_messages']}/"
            f"{result['odometry']['accepted_scan_diagnosed_published_count']}/"
            f"{result['odometry']['accepted_scan_sequence']}",
            f"- scan registration density: {result['odometry']['scan_registration_density']:.6%}",
            f"- deskew accepted/rejected: {deskew['accepted']}/{deskew['rejected']}",
            f"- deskew rejection reasons: `{json.dumps(deskew['rejection_reasons'], sort_keys=True)}`",
            "",
            "Recorded `/tf` and `/tf_static` are not estimator inputs; sensor TF is supplied externally by the runner.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sensor", required=True, choices=("velodyne", "mid360"))
    parser.add_argument("--input-bag", required=True, type=Path)
    parser.add_argument("--result-bag", required=True, type=Path)
    parser.add_argument("--run-env", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error(f"--output-dir already exists: {args.output_dir}")
    args.output_dir.mkdir(parents=True)
    try:
        result = analyze(args)
    except Exception as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    (args.output_dir / "runtime_metrics.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    write_report(args.output_dir / "REPORT.md", result)
    print(f"{'PASS' if result['passed'] else 'FAIL'}: {args.output_dir / 'REPORT.md'}")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
