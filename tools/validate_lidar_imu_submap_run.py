#!/usr/bin/env python3
"""Fail-closed audit for the LiDAR/IMU-only external-submap branch.

The precision-local input is the accepted-scan odometry stream, not the regular
wall-timer odometry stream.  Consequently this validator associates raw and
precision-local poses at physical scan header stamps.  It permits only a 1 us
nearest-stamp tolerance for double/epoch serialization round-off; it never
interpolates either odometry trajectory.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import tempfile
from typing import Any

import numpy as np


RAW = "/localization/gyro_lidar_odom_scan"
SCAN = "/localization/submap_scan"
CORRECTION = "/localization/submap_correction"
LOCAL = "/localization/precision_local_odom"
GLOBAL = "/localization/precision_global_odom"
GLOBAL_POSE = "/localization/precision_global_pose"
DIAGNOSTICS = "/diagnostics"

ODOMETRY_TYPE = "nav_msgs/msg/Odometry"
SCAN_TYPE = "pure_lidar_msgs/msg/SubmapScan"
CORRECTION_TYPE = "pure_lidar_msgs/msg/SubmapCorrection"
GLOBAL_POSE_TYPE = "geometry_msgs/msg/PoseWithCovarianceStamped"

ODOMETER_DIAGNOSTIC = "localization/gyro_odometer"
MATCHER_DIAGNOSTIC = "localization/submap_matcher"
LOCALIZER_DIAGNOSTIC = "localization/precision_global_localizer"


@dataclass(frozen=True)
class Pose:
    stamp_ns: int
    x: float
    y: float
    yaw: float
    frame: str
    child: str


@dataclass(frozen=True)
class ScanRecord:
    key: tuple[int, int, int, int]
    pose: Pose
    cloud_stamp_ns: int


@dataclass(frozen=True)
class CorrectionRecord:
    key: tuple[int, int, int, int]
    matcher_session: int
    submap_generation: int
    correction_id: int
    valid_contract: bool
    position_error_m: float
    orientation_error_rad: float


@dataclass(frozen=True)
class Check:
    name: str
    passed: bool
    detail: str
    category: str = "hard"


@dataclass
class Records:
    topic_types: dict[str, str]
    topic_counts: dict[str, int]
    raw: list[Pose]
    local: list[Pose]
    scans: list[ScanRecord]
    corrections: list[CorrectionRecord]
    diagnostics: dict[str, list[dict[str, str]]]


def wrap(value: np.ndarray | float) -> np.ndarray | float:
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def yaw_of(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (float(quaternion.w) * float(quaternion.z)
               + float(quaternion.x) * float(quaternion.y)),
        1.0 - 2.0 * (float(quaternion.y) ** 2 + float(quaternion.z) ** 2),
    )


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quaternion_tuple(quaternion: Any) -> tuple[float, float, float, float]:
    return (
        float(quaternion.x), float(quaternion.y), float(quaternion.z),
        float(quaternion.w),
    )


def quaternion_product(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_angle(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> float:
    first_norm = math.sqrt(sum(value * value for value in first))
    second_norm = math.sqrt(sum(value * value for value in second))
    if first_norm < 1.0e-12 or second_norm < 1.0e-12:
        return math.inf
    dot = abs(sum(a * b for a, b in zip(first, second))) / (first_norm * second_norm)
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def correction_contract(scan: Any, correction: Any) -> tuple[bool, float, float]:
    transform = correction.precision_from_raw
    raw = scan.raw_pose.pose
    corrected = correction.corrected_pose.pose
    transform_quaternion = quaternion_tuple(transform.rotation)
    raw_quaternion = quaternion_tuple(raw.orientation)
    yaw = yaw_of(transform.rotation)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    expected = np.asarray(
        [
            float(transform.translation.x) + cosine * float(raw.position.x)
            - sine * float(raw.position.y),
            float(transform.translation.y) + sine * float(raw.position.x)
            + cosine * float(raw.position.y),
            float(transform.translation.z) + float(raw.position.z),
        ]
    )
    observed = np.asarray(
        [corrected.position.x, corrected.position.y, corrected.position.z], dtype=float
    )
    position_error = float(np.linalg.norm(observed - expected))
    orientation_error = quaternion_angle(
        quaternion_product(transform_quaternion, raw_quaternion),
        quaternion_tuple(corrected.orientation),
    )
    covariance = np.asarray(correction.corrected_pose.covariance, dtype=float)
    finite = np.all(np.isfinite(observed)) and np.all(np.isfinite(covariance))
    valid = (
        bool(correction.use_yaw)
        and bool(correction.header.frame_id)
        and bool(correction.precision_frame_id)
        and math.isfinite(float(correction.fitness))
        and math.isfinite(float(correction.inlier_ratio))
        and math.isfinite(float(correction.innovation_translation_m))
        and math.isfinite(float(correction.innovation_yaw_rad))
        and int(correction.consistency_count) > 0
        and finite
        and position_error <= 1.0e-6
        and orientation_error <= 1.0e-6
    )
    return valid, position_error, orientation_error


def resolve_bag(path: Path) -> Path:
    path = path.resolve()
    if path.is_file() and path.suffix == ".mcap":
        return path
    if path.is_dir() and (path / "metadata.yaml").is_file():
        return path
    nested = path / "localization_output"
    if nested.is_dir() and (nested / "metadata.yaml").is_file():
        return nested
    raise RuntimeError(f"cannot resolve rosbag from {path}")


def pose_from_message(message: Any) -> Pose:
    position = message.pose.pose.position
    quaternion = message.pose.pose.orientation
    values = np.asarray(
        [position.x, position.y, quaternion.x, quaternion.y, quaternion.z, quaternion.w],
        dtype=float,
    )
    norm = float(np.linalg.norm(values[2:]))
    if not np.all(np.isfinite(values)) or abs(norm - 1.0) > 1.0e-6:
        raise RuntimeError(f"invalid odometry pose at {stamp_ns(message.header.stamp)}")
    return Pose(
        stamp_ns(message.header.stamp), float(position.x), float(position.y),
        yaw_of(quaternion), str(message.header.frame_id), str(message.child_frame_id),
    )


def read_records(path: Path) -> Records:
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as error:
        raise RuntimeError("source ROS 2 and this workspace before validation") from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(resolve_bag(path)), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = {
        RAW: ODOMETRY_TYPE,
        SCAN: SCAN_TYPE,
        CORRECTION: CORRECTION_TYPE,
        LOCAL: ODOMETRY_TYPE,
        DIAGNOSTICS: "diagnostic_msgs/msg/DiagnosticArray",
    }
    missing = {
        topic: (expected, types.get(topic))
        for topic, expected in required.items() if types.get(topic) != expected
    }
    if missing:
        raise RuntimeError(f"missing or mistyped required topics: {missing}")
    classes = {topic: get_message(type_name) for topic, type_name in types.items()}
    raw: list[Pose] = []
    local: list[Pose] = []
    scans: list[ScanRecord] = []
    corrections: list[CorrectionRecord] = []
    scan_messages: dict[tuple[int, int, int, int], Any] = {}
    pending_corrections: list[Any] = []
    diagnostics = {
        ODOMETER_DIAGNOSTIC: [], MATCHER_DIAGNOSTIC: [], LOCALIZER_DIAGNOSTIC: []
    }
    counts = {topic: 0 for topic in types}

    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic not in required:
            continue
        message = deserialize_message(serialized, classes[topic])
        if topic == RAW:
            raw.append(pose_from_message(message))
        elif topic == LOCAL:
            local.append(pose_from_message(message))
        elif topic == SCAN:
            key = (
                int(message.odom_session_id), int(message.odom_generation),
                int(message.sequence), stamp_ns(message.header.stamp),
            )
            pose_message = type("PoseEnvelope", (), {})()
            pose_message.header = message.header
            pose_message.child_frame_id = "base_link"
            pose_message.pose = message.raw_pose
            pose = pose_from_message(pose_message)
            scans.append(ScanRecord(key, pose, stamp_ns(message.cloud.header.stamp)))
            scan_messages[key] = message
        elif topic == CORRECTION:
            pending_corrections.append(message)
        else:
            for status in message.status:
                normalized = str(status.name).split("/")
                values = {str(item.key): str(item.value) for item in status.values}
                for name in diagnostics:
                    if status.name == name or status.name.endswith("/" + name):
                        diagnostics[name].append(values)
                        break

    for message in pending_corrections:
        key = (
            int(message.odom_session_id), int(message.odom_generation),
            int(message.sequence), stamp_ns(message.header.stamp),
        )
        if key in scan_messages:
            valid, position_error, orientation_error = correction_contract(
                scan_messages[key], message
            )
        else:
            valid, position_error, orientation_error = False, math.inf, math.inf
        corrections.append(
            CorrectionRecord(
                key, int(message.matcher_session_id), int(message.submap_generation),
                int(message.correction_id), valid, position_error, orientation_error,
            )
        )
    return Records(types, counts, raw, local, scans, corrections, diagnostics)


def nearest_unique_association(
    source_stamps: np.ndarray, target_stamps: np.ndarray, tolerance_ns: int
) -> tuple[list[tuple[int, int]], np.ndarray]:
    """Associate targets to sources one-to-one, without trajectory interpolation."""
    source_stamps = np.asarray(source_stamps, dtype=np.int64)
    target_stamps = np.asarray(target_stamps, dtype=np.int64)
    if np.any(np.diff(source_stamps) <= 0) or np.any(np.diff(target_stamps) <= 0):
        return [], np.asarray([], dtype=np.int64)
    pairs: list[tuple[int, int]] = []
    deltas: list[int] = []
    used: set[int] = set()
    for target_index, target in enumerate(target_stamps):
        insertion = int(np.searchsorted(source_stamps, target, side="left"))
        candidates = [index for index in (insertion - 1, insertion)
                      if 0 <= index < len(source_stamps)]
        if not candidates:
            continue
        distances = [(abs(int(source_stamps[index]) - int(target)), index)
                     for index in candidates]
        distance, source_index = min(distances)
        equally_near = [index for candidate_distance, index in distances
                        if candidate_distance == distance]
        if distance > tolerance_ns or len(equally_near) != 1 or source_index in used:
            continue
        used.add(source_index)
        pairs.append((source_index, target_index))
        deltas.append(int(source_stamps[source_index]) - int(target))
    return pairs, np.asarray(deltas, dtype=np.int64)


def monotonic_unique(poses: list[Pose]) -> bool:
    return bool(poses) and all(
        second.stamp_ns > first.stamp_ns for first, second in zip(poses, poses[1:])
    )


def number(values: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        return float(values[key])
    except (KeyError, ValueError):
        return default


def latest_diagnostic(
    records: list[dict[str, str]], primary: str, secondary: str
) -> dict[str, str] | None:
    if not records:
        return None
    return max(records, key=lambda values: (number(values, primary, -1),
                                             number(values, secondary, -1)))


def runtime_summary(runtime: dict[str, Any] | None) -> tuple[bool, str]:
    if not isinstance(runtime, dict):
        return False, "runtime_metrics.json missing"
    failed = []
    for item in runtime.get("checks", []):
        if item.get("category", "hard") == "hard" and item.get("passed") is not True:
            failed.append(str(item.get("name", "unnamed")))
    passed = runtime.get("passed") is True and not failed
    return passed, "failed=" + (", ".join(failed) if failed else "none")


def evaluate_records(
    records: Records,
    runtime: dict[str, Any] | None,
    tolerance_ns: int = 1_000,
    maximum_matcher_p99_ms: float = 250.0,
) -> dict[str, Any]:
    checks: list[Check] = []
    add = lambda name, passed, detail, category="hard": checks.append(
        Check(name, bool(passed), detail, category)
    )

    runtime_passed, runtime_detail = runtime_summary(runtime)
    add("base LiDAR/IMU runtime audit passes", runtime_passed, runtime_detail)

    raw, local, scans, corrections = (
        records.raw, records.local, records.scans, records.corrections
    )
    add(
        "raw accepted-scan poses are unique and strictly increasing",
        len(raw) >= 3 and monotonic_unique(raw), f"messages={len(raw)}",
    )
    add(
        "precision-local poses are unique and strictly increasing",
        len(local) >= 3 and monotonic_unique(local), f"messages={len(local)}",
    )
    raw_frames = {(item.frame, item.child) for item in raw}
    local_frames = {(item.frame, item.child) for item in local}
    add("raw accepted-scan frame contract", raw_frames == {("odom", "base_link")},
        f"frames={sorted(raw_frames)}")
    add("precision-local frame contract",
        local_frames == {("odom_precision", "base_link")},
        f"frames={sorted(local_frames)}")

    scan_keys = [item.key for item in scans]
    scan_stamps = np.asarray([item.key[3] for item in scans], dtype=np.int64)
    scan_streams = {(item.key[0], item.key[1]) for item in scans}
    add(
        "SubmapScan exact keys and physical stamps are valid",
        len(scans) >= 3 and len(scan_keys) == len(set(scan_keys))
        and all(key[0] > 0 and key[1] > 0 and key[2] > 0 and key[3] > 0
                for key in scan_keys)
        and bool(len(scan_stamps)) and np.all(np.diff(scan_stamps) > 0)
        and all(item.cloud_stamp_ns == item.key[3] for item in scans),
        f"scans={len(scans)} keys={len(set(scan_keys))} streams={len(scan_streams)}",
    )

    raw_stamps = np.asarray([item.stamp_ns for item in raw], dtype=np.int64)
    scan_pairs, scan_delta = nearest_unique_association(raw_stamps, scan_stamps, tolerance_ns)
    scan_xy_error = []
    scan_yaw_error = []
    for raw_index, scan_index in scan_pairs:
        raw_pose, scan_pose = raw[raw_index], scans[scan_index].pose
        scan_xy_error.append(math.hypot(raw_pose.x - scan_pose.x, raw_pose.y - scan_pose.y))
        scan_yaw_error.append(abs(float(wrap(raw_pose.yaw - scan_pose.yaw))))
    add(
        "SubmapScan snapshots map one-to-one to accepted physical scans",
        len(scan_pairs) == len(scans)
        and (not scan_xy_error or max(scan_xy_error) <= 1.0e-6)
        and (not scan_yaw_error or max(scan_yaw_error) <= 1.0e-6),
        f"matched={len(scan_pairs)}/{len(scans)} max_stamp_delta_ns="
        f"{int(np.max(np.abs(scan_delta))) if len(scan_delta) else -1} "
        f"max_xy_m={max(scan_xy_error, default=math.inf):.3e} "
        f"max_yaw_rad={max(scan_yaw_error, default=math.inf):.3e}",
    )

    correction_keys = [item.key for item in corrections]
    scan_key_set = set(scan_keys)
    correction_key_set = set(correction_keys)
    max_position_error = max((item.position_error_m for item in corrections), default=math.inf)
    max_orientation_error = max(
        (item.orientation_error_rad for item in corrections), default=math.inf
    )
    add(
        "SubmapCorrection exact-key and full-SE2 contracts are valid",
        bool(corrections) and len(correction_keys) == len(correction_key_set)
        and not (correction_key_set - scan_key_set)
        and all(item.matcher_session > 0 and item.submap_generation > 0
                and item.correction_id > 0 and item.valid_contract for item in corrections),
        f"corrections={len(corrections)} duplicates="
        f"{len(correction_keys)-len(correction_key_set)} unknown="
        f"{len(correction_key_set-scan_key_set)} max_position_error_m="
        f"{max_position_error:.3e} max_orientation_error_rad={max_orientation_error:.3e}",
    )
    if scans and corrections:
        first_scan_ns = scans[0].key[3]
        first_correction_ns = min(item.key[3] for item in corrections)
        post_warmup = sum(item.key[3] >= first_correction_ns for item in scans)
        warmup_sec = (first_correction_ns - first_scan_ns) * 1.0e-9
        correction_ratio = len(corrections) / post_warmup if post_warmup else 0.0
    else:
        first_scan_ns, warmup_sec, correction_ratio = 0, math.inf, 0.0
    add(
        "submap initialization and correction coverage are bounded",
        0.0 <= warmup_sec <= 10.0 and correction_ratio >= 0.70,
        f"warmup_sec={warmup_sec:.6f} post_warmup_ratio={correction_ratio:.6f}",
    )

    eligible_raw = [item for item in raw if item.stamp_ns >= first_scan_ns]
    eligible_stamps = np.asarray([item.stamp_ns for item in eligible_raw], dtype=np.int64)
    local_stamps = np.asarray([item.stamp_ns for item in local], dtype=np.int64)
    local_pairs, local_delta = nearest_unique_association(
        local_stamps, eligible_stamps, tolerance_ns
    )
    local_coverage = len(local_pairs) / len(eligible_raw) if eligible_raw else 0.0
    local_extra = max(0, len(local) - len(local_pairs))
    add(
        "precision-local covers accepted physical scans without interpolation",
        local_coverage >= 0.99 and local_extra == 0,
        f"matched={len(local_pairs)}/{len(eligible_raw)} coverage={local_coverage:.6%} "
        f"extra={local_extra} max_stamp_delta_ns="
        f"{int(np.max(np.abs(local_delta))) if len(local_delta) else -1} "
        f"tolerance_ns={tolerance_ns}",
    )
    add(
        "precision-local starts only after the first SubmapScan epoch",
        bool(local) and all(item.stamp_ns >= first_scan_ns - tolerance_ns for item in local),
        f"first_scan_ns={first_scan_ns} first_local_ns="
        f"{local[0].stamp_ns if local else -1}",
    )

    discovered_global = {
        GLOBAL: records.topic_types.get(GLOBAL),
        GLOBAL_POSE: records.topic_types.get(GLOBAL_POSE),
    }
    global_counts = {topic: records.topic_counts.get(topic, 0) for topic in discovered_global}
    add(
        "precision global outputs are discovered but publish zero messages",
        discovered_global == {GLOBAL: ODOMETRY_TYPE, GLOBAL_POSE: GLOBAL_POSE_TYPE}
        and all(value == 0 for value in global_counts.values()),
        f"types={discovered_global} counts={global_counts}",
    )

    odometer = latest_diagnostic(
        records.diagnostics[ODOMETER_DIAGNOSTIC],
        "external_submap_snapshot_published_count", "accepted_scan_odom_published_count",
    )
    if odometer is None:
        add("odometer snapshot diagnostics are consistent", False, "missing diagnostic")
    else:
        add(
            "odometer snapshot diagnostics are consistent",
            odometer.get("external_submap_snapshot_enabled") == "true"
            and odometer.get("lidar_tracking_mode") == "scan_to_scan"
            and int(number(odometer, "external_submap_snapshot_published_count", -1))
            == len(scans)
            and number(odometer, "external_submap_snapshot_conversion_max_ms") < 20.0,
            f"enabled={odometer.get('external_submap_snapshot_enabled')} "
            f"tracking={odometer.get('lidar_tracking_mode')} diag_scans="
            f"{number(odometer, 'external_submap_snapshot_published_count', -1):.0f} "
            f"bag_scans={len(scans)} conversion_max_ms="
            f"{number(odometer, 'external_submap_snapshot_conversion_max_ms'):.6f}",
        )

    matcher = latest_diagnostic(
        records.diagnostics[MATCHER_DIAGNOSTIC], "received_count", "correction_publish_count"
    )
    matcher_attempted: int | None = None
    matcher_accepted: int | None = None
    matcher_rejected: int | None = None
    matcher_accepted_ratio: float | None = None
    matcher_processing_p99_ms: float | None = None
    matcher_latency_p99_ms: float | None = None
    matcher_queue_drop_count: int | None = None
    if matcher is None:
        add("matcher runtime and counter contracts pass", False, "missing diagnostic")
    else:
        received = int(number(matcher, "received_count", -1))
        processed = int(number(matcher, "processed_count", -1))
        attempted = int(number(matcher, "attempted_count", -1))
        accepted = int(number(matcher, "accepted_match_count", -1))
        rejected = int(number(matcher, "rejected_match_count", -1))
        committed = int(number(matcher, "committed_count", -1))
        published = int(number(matcher, "correction_publish_count", -1))
        ratio = accepted / attempted if attempted > 0 else 0.0
        processing_p99 = number(matcher, "processing_p99_ms")
        latency_p99 = number(matcher, "latency_p99_ms")
        queue_drop_count = int(number(matcher, "queue_drop_count", -1))
        matcher_attempted = attempted
        matcher_accepted = accepted
        matcher_rejected = rejected
        matcher_accepted_ratio = ratio
        matcher_processing_p99_ms = processing_p99
        matcher_latency_p99_ms = latency_p99
        matcher_queue_drop_count = queue_drop_count
        add(
            "matcher runtime and counter contracts pass",
            received == processed == len(scans)
            and queue_drop_count == 0
            and int(number(matcher, "malformed_count", -1)) == 0
            and int(number(matcher, "stale_or_duplicate_count", -1)) == 0
            and attempted == accepted + rejected
            and committed == published == len(corrections)
            and ratio >= 0.75
            and processing_p99 <= maximum_matcher_p99_ms
            and latency_p99 <= maximum_matcher_p99_ms,
            f"received/processed/bag={received}/{processed}/{len(scans)} "
            f"attempted/accepted/rejected={attempted}/{accepted}/{rejected} "
            f"committed/published/bag={committed}/{published}/{len(corrections)} "
            f"accepted_ratio={ratio:.6f} processing_p99_ms={processing_p99:.3f} "
            f"latency_p99_ms={latency_p99:.3f} limit_ms={maximum_matcher_p99_ms:.3f}",
        )

    localizer = latest_diagnostic(
        records.diagnostics[LOCALIZER_DIAGNOSTIC], "raw.received", "publish.local"
    )
    if localizer is None:
        add("local compositor counters and global isolation pass", False, "missing diagnostic")
    else:
        add(
            "local compositor counters and global isolation pass",
            int(number(localizer, "submap.scan_received", -1)) == len(scans)
            and int(number(localizer, "submap.scan_rejected", -1)) == 0
            and int(number(localizer, "submap.correction_received", -1)) == len(corrections)
            and int(number(localizer, "local_correction.accepted", -1)) == len(corrections)
            and int(number(localizer, "local_correction.rejected", -1)) == 0
            and int(number(localizer, "local_correction.pending", -1)) == 0
            and int(number(localizer, "raw.received", -1)) == len(raw)
            and int(number(localizer, "raw.invalid", -1)) == len(raw) - len(local)
            and int(number(localizer, "raw.nonmonotonic", -1)) == 0
            and int(number(localizer, "raw.duplicate_stamp", -1)) == 0
            and int(number(localizer, "publish.local", -1)) == len(local)
            and int(number(localizer, "publish.global", -1)) == 0
            and int(number(localizer, "gnss.received", -1)) == 0
            and int(number(localizer, "fusion.health.received", -1)) == 0
            and int(number(localizer, "fusion.sync.existing_global_received", -1)) == 0
            and localizer.get("local_correction.tf_published") == "false",
            f"scans={number(localizer, 'submap.scan_received', -1):.0f}/{len(scans)} "
            f"corrections={number(localizer, 'local_correction.accepted', -1):.0f}/"
            f"{len(corrections)} local={number(localizer, 'publish.local', -1):.0f}/"
            f"{len(local)} raw_received/invalid={number(localizer, 'raw.received', -1):.0f}/"
            f"{number(localizer, 'raw.invalid', -1):.0f} global="
            f"{number(localizer, 'publish.global', -1):.0f} "
            f"gnss={number(localizer, 'gnss.received', -1):.0f} existing_global="
            f"{number(localizer, 'fusion.sync.existing_global_received', -1):.0f}",
        )

    failed = [item.name for item in checks if item.category == "hard" and not item.passed]
    return {
        "passed": not failed,
        "method": {
            "raw_topic": RAW,
            "precision_local_topic": LOCAL,
            "association": (
                "one-to-one accepted physical header stamps; nearest <=1us only for "
                "epoch/double serialization ULP; no odometry interpolation"
            ),
            "precision_coverage_denominator": (
                "raw accepted-scan poses at/after first SubmapScan epoch"
            ),
            "global_policy": "precision global outputs and all global authorities stay zero",
        },
        "counts": {
            "raw_accepted": len(raw), "submap_scans": len(scans),
            "submap_streams": len(scan_streams),
            "submap_corrections": len(corrections), "precision_local": len(local),
            "precision_local_eligible_raw": len(eligible_raw),
            "precision_local_matched": len(local_pairs),
            "precision_local_coverage": local_coverage,
            "global_topics": global_counts,
        },
        "metrics": {
            "scan_stamp_delta_ns_maximum": (
                int(np.max(np.abs(scan_delta))) if len(scan_delta) else None
            ),
            "precision_local_stamp_delta_ns_maximum": (
                int(np.max(np.abs(local_delta))) if len(local_delta) else None
            ),
            "warmup_sec": warmup_sec,
            "post_warmup_correction_ratio": correction_ratio,
            "matcher_attempted": matcher_attempted,
            "matcher_accepted": matcher_accepted,
            "matcher_rejected": matcher_rejected,
            "matcher_accepted_ratio": matcher_accepted_ratio,
            "matcher_processing_p99_ms": matcher_processing_p99_ms,
            "matcher_latency_p99_ms": matcher_latency_p99_ms,
            "matcher_queue_drop_count": matcher_queue_drop_count,
            "snapshot_conversion_max_ms": (
                number(odometer, "external_submap_snapshot_conversion_max_ms")
                if odometer is not None else None
            ),
        },
        "checks": [asdict(item) for item in checks],
        "failed_hard_checks": failed,
    }


def write_markdown(path: Path, result: dict[str, Any]) -> None:
    counts = result["counts"]
    lines = [
        "# LiDAR/IMU external-submap structural acceptance", "",
        f"Result: **{'PASS' if result['passed'] else 'FAIL'}**", "",
        f"- raw accepted scans: {counts['raw_accepted']}",
        f"- sparse SubmapScan snapshots: {counts['submap_scans']}",
        f"- accepted corrections: {counts['submap_corrections']}",
        f"- precision-local exact physical coverage: "
        f"{counts['precision_local_matched']}/{counts['precision_local_eligible_raw']} "
        f"({counts['precision_local_coverage']:.6%} after first epoch)", "",
        "## Checks", "",
    ]
    for item in result["checks"]:
        mark = "PASS" if item["passed"] else ("WARN" if item["category"] == "warn" else "FAIL")
        lines.append(f"- {mark}: {item['name']} — {item['detail']}")
    lines.extend([
        "", "Association uses physical message header stamps. No raw or precision-local "
        "pose is interpolated; the 1 us tolerance exists only for epoch/double ULP round-off.", "",
    ])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def synthetic_records() -> tuple[Records, dict[str, Any]]:
    raw = [Pose(index * 100_000_000, float(index), 0.0, 0.01 * index,
                "odom", "base_link") for index in range(1, 301)]
    scans = [
        ScanRecord((7, 1 if index < 151 else 2, index, raw[index - 1].stamp_ns), raw[index - 1],
                   raw[index - 1].stamp_ns)
        for index in range(1, 301, 5)
    ]
    corrections = [
        CorrectionRecord(item.key, 11, 1, index + 1, True, 0.0, 0.0)
        for index, item in enumerate(scans[3:])
    ]
    local = [Pose(item.stamp_ns, item.x, item.y, item.yaw,
                  "odom_precision", "base_link") for item in raw]
    matcher = {
        "received_count": str(len(scans)), "processed_count": str(len(scans)),
        "queue_drop_count": "0", "malformed_count": "0",
        "stale_or_duplicate_count": "0", "attempted_count": "57",
        "accepted_match_count": "55", "rejected_match_count": "2",
        "committed_count": str(len(corrections)),
        "correction_publish_count": str(len(corrections)),
        "processing_p99_ms": "50", "latency_p99_ms": "100",
    }
    odometer = {
        "external_submap_snapshot_enabled": "true", "lidar_tracking_mode": "scan_to_scan",
        "external_submap_snapshot_published_count": str(len(scans)),
        "accepted_scan_odom_published_count": str(len(raw)),
        "external_submap_snapshot_conversion_max_ms": "2",
    }
    localizer = {
        "submap.scan_received": str(len(scans)), "submap.scan_rejected": "0",
        "submap.correction_received": str(len(corrections)),
        "local_correction.accepted": str(len(corrections)),
        "local_correction.rejected": "0", "local_correction.pending": "0",
        "raw.received": str(len(raw)), "raw.invalid": "0", "raw.nonmonotonic": "0",
        "raw.duplicate_stamp": "0",
        "publish.local": str(len(local)), "publish.global": "0",
        "gnss.received": "0", "fusion.health.received": "0",
        "fusion.sync.existing_global_received": "0", "local_correction.tf_published": "false",
    }
    records = Records(
        {
            RAW: ODOMETRY_TYPE, SCAN: SCAN_TYPE, CORRECTION: CORRECTION_TYPE,
            LOCAL: ODOMETRY_TYPE, GLOBAL: ODOMETRY_TYPE,
            GLOBAL_POSE: GLOBAL_POSE_TYPE, DIAGNOSTICS: "diagnostic_msgs/msg/DiagnosticArray",
        },
        {GLOBAL: 0, GLOBAL_POSE: 0}, raw, local, scans, corrections,
        {ODOMETER_DIAGNOSTIC: [odometer], MATCHER_DIAGNOSTIC: [matcher],
         LOCALIZER_DIAGNOSTIC: [localizer]},
    )
    runtime = {"passed": True, "checks": [{"name": "runtime", "passed": True,
                                              "category": "hard"}]}
    return records, runtime


def self_test() -> None:
    records, runtime = synthetic_records()
    result = evaluate_records(records, runtime)
    assert result["passed"], result["failed_hard_checks"]
    assert result["counts"]["submap_streams"] == 2
    assert result["metrics"]["matcher_attempted"] == 57
    assert result["metrics"]["matcher_accepted"] == 55
    assert result["metrics"]["matcher_rejected"] == 2
    assert math.isclose(result["metrics"]["matcher_accepted_ratio"], 55.0 / 57.0)
    assert result["metrics"]["matcher_processing_p99_ms"] == 50.0
    assert result["metrics"]["matcher_latency_p99_ms"] == 100.0
    assert result["metrics"]["matcher_queue_drop_count"] == 0
    assert result["metrics"]["snapshot_conversion_max_ms"] == 2.0
    shifted = Records(
        records.topic_types, records.topic_counts, records.raw,
        [Pose(item.stamp_ns + 2_000, item.x, item.y, item.yaw, item.frame, item.child)
         for item in records.local],
        records.scans, records.corrections, records.diagnostics,
    )
    failed = evaluate_records(shifted, runtime)
    assert not failed["passed"]
    assert "precision-local covers accepted physical scans without interpolation" in failed[
        "failed_hard_checks"
    ]
    with tempfile.TemporaryDirectory() as directory:
        report = Path(directory) / "REPORT.md"
        write_markdown(report, result)
        assert "Result: **PASS**" in report.read_text(encoding="utf-8")
        run_dir = Path(directory) / "run"
        output_dir = run_dir / "submap_validation"
        output_dir.mkdir(parents=True)
        (output_dir / "validation.json").write_text("{}\n", encoding="utf-8")
        (output_dir / "REPORT.md").write_text("generated\n", encoding="utf-8")
        replace_canonical_output(run_dir, output_dir)
        assert not output_dir.exists()
        output_dir.mkdir()
        (output_dir / "validation.json").write_text("{}\n", encoding="utf-8")
        (output_dir / "REPORT.md").write_text("generated\n", encoding="utf-8")
        (output_dir / "keep.txt").write_text("user data\n", encoding="utf-8")
        try:
            replace_canonical_output(run_dir, output_dir)
        except SystemExit:
            pass
        else:
            raise AssertionError("mixed canonical/user contents were replaced")
        assert (output_dir / "validation.json").is_file()
        assert (output_dir / "REPORT.md").is_file()
        assert (output_dir / "keep.txt").read_text(encoding="utf-8") == "user data\n"
        other = Path(directory) / "other"
        other.mkdir()
        (other / "validation.json").write_text("{}\n", encoding="utf-8")
        (other / "REPORT.md").write_text("generated\n", encoding="utf-8")
        (other / "keep.txt").write_text("user data\n", encoding="utf-8")
        try:
            replace_canonical_output(run_dir, other)
        except SystemExit:
            pass
        else:
            raise AssertionError("non-canonical output replacement was accepted")
        assert (other / "validation.json").is_file()
        assert (other / "REPORT.md").is_file()
        assert (other / "keep.txt").read_text(encoding="utf-8") == "user data\n"


def replace_canonical_output(run_dir: Path, output_dir: Path) -> None:
    if not output_dir.exists():
        return
    try:
        expected_output = run_dir.resolve() / "submap_validation"
        requested_output = output_dir.resolve()
    except OSError:
        expected_output = Path()
        requested_output = Path("different")
    if requested_output != expected_output:
        raise SystemExit(f"refusing to overwrite output directory: {output_dir}")
    expected_files = {"validation.json", "REPORT.md"}
    actual_files = {path.name for path in output_dir.iterdir()}
    if actual_files != expected_files or any(
        not path.is_file() for path in output_dir.iterdir()
    ):
        raise SystemExit(
            "refusing to replace non-canonical contents in output directory: "
            f"{output_dir}"
        )
    # The exact two-file directory is generated by this validator and is the
    # runner's canonical destination. Validate the complete set before making
    # any change so an unexpected user file can never cause a partial delete.
    for name in sorted(expected_files):
        (output_dir / name).unlink()
    output_dir.rmdir()


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--run-dir", type=Path)
    result.add_argument("--bag", type=Path)
    result.add_argument("--runtime-json", type=Path)
    result.add_argument("--output-dir", type=Path)
    result.add_argument("--stamp-tolerance-us", type=float, default=1.0)
    result.add_argument("--maximum-matcher-p99-ms", type=float, default=250.0)
    result.add_argument("--self-test", action="store_true")
    return result


def main() -> int:
    args = parser().parse_args()
    if args.self_test:
        self_test()
        print("PASS: synthetic LiDAR/IMU external-submap structural acceptance")
        return 0
    if args.run_dir is None and args.bag is None:
        raise SystemExit("--run-dir or --bag is required")
    if args.output_dir is None:
        raise SystemExit("--output-dir is required")
    if args.stamp_tolerance_us < 0.0 or args.maximum_matcher_p99_ms <= 0.0:
        raise SystemExit("stamp tolerance must be non-negative and p99 limit positive")
    if args.output_dir.exists() and args.run_dir is not None:
        replace_canonical_output(args.run_dir, args.output_dir)
    bag = args.bag if args.bag is not None else args.run_dir
    runtime_path = args.runtime_json
    if runtime_path is None and args.run_dir is not None:
        runtime_path = args.run_dir / "runtime_analysis" / "runtime_metrics.json"
    runtime = None
    if runtime_path is not None and runtime_path.is_file():
        runtime = json.loads(runtime_path.read_text(encoding="utf-8"))
    records = read_records(bag)
    result = evaluate_records(
        records, runtime, int(round(args.stamp_tolerance_us * 1_000.0)),
        args.maximum_matcher_p99_ms,
    )
    if args.output_dir.exists():
        raise SystemExit(f"refusing to overwrite output directory: {args.output_dir}")
    args.output_dir.mkdir(parents=True)
    (args.output_dir / "validation.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    write_markdown(args.output_dir / "REPORT.md", result)
    print(f"{'PASS' if result['passed'] else 'FAIL'}: {args.output_dir / 'REPORT.md'}")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
