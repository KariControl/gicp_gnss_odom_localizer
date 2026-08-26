#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Validate a recorded Autoware localization-interface output bag (MCAP).

The checker intentionally evaluates interface and estimator-state contracts only.
It does not report localization accuracy because the output bag does not provide
an independently established ground-truth trajectory.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass, field
import math
from pathlib import Path
import sys
from typing import Any, Iterable, Sequence

import numpy as np
import rclpy  # noqa: F401 - importing rclpy makes the required ROS runtime explicit.
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


KINEMATIC_TOPIC = "/localization/kinematic_state"
TWIST_TOPIC = "/localization/twist_with_covariance"
POSE_TOPIC = "/localization/pose_estimator/pose_with_covariance"
ACCEL_TOPIC = "/localization/acceleration"
CLOCK_TOPIC = "/clock"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
DIAGNOSTICS_TOPIC = "/diagnostics"
DIAGNOSTICS_AGG_TOPIC = "/diagnostics_agg"

ADAPTER_DIAGNOSTIC = "localization/localization_interface_adapter"
POSE_INSTABILITY_DIAGNOSTIC = "localization: pose_instability_detector"
LOCALIZATION_ERROR_DIAGNOSTIC = (
    "localization_error_monitor: ellipse_error_status"
)
GNSS_DIAGNOSTIC = "localization/gnss_map_odom_fusion"
DESKEW_DIAGNOSTIC = "localization/imu_undistortion"
TRACKING_DIAGNOSTIC = "localization/gyro_odometer"
AGGREGATED_MONITOR_DIAGNOSTICS = {
    "/pose_instability_detector/localization: pose_instability_detector",
    (
        "/localization_error_monitor/"
        "localization_error_monitor: ellipse_error_status"
    ),
}

BASE_REQUIRED_TOPICS = {
    CLOCK_TOPIC: "rosgraph_msgs/msg/Clock",
    TF_TOPIC: "tf2_msgs/msg/TFMessage",
    TF_STATIC_TOPIC: "tf2_msgs/msg/TFMessage",
    DIAGNOSTICS_TOPIC: "diagnostic_msgs/msg/DiagnosticArray",
    DIAGNOSTICS_AGG_TOPIC: "diagnostic_msgs/msg/DiagnosticArray",
    KINEMATIC_TOPIC: "nav_msgs/msg/Odometry",
    TWIST_TOPIC: "geometry_msgs/msg/TwistWithCovarianceStamped",
    POSE_TOPIC: "geometry_msgs/msg/PoseWithCovarianceStamped",
    ACCEL_TOPIC: "geometry_msgs/msg/AccelWithCovarianceStamped",
}

HESAI_REQUIRED_TOPICS = {
    "/localization/ekf_odom": "nav_msgs/msg/Odometry",
    "/localization/gyro_lidar_odom": "nav_msgs/msg/Odometry",
    "/localization/imu_corrected": "sensor_msgs/msg/Imu",
    "/localization/gnss_fusion_input": "pure_gnss_msgs/msg/GnssFusionInput",
}

HESAI_OPTIONAL_TOPICS = {
    "/localization/points_undistorted": "sensor_msgs/msg/PointCloud2",
    "/localization/gnss_odometry": "nav_msgs/msg/Odometry",
}


@dataclass(frozen=True)
class Check:
    status: str
    name: str
    detail: str


@dataclass
class Report:
    checks: list[Check] = field(default_factory=list)

    def add(self, status: str, name: str, detail: str) -> None:
        self.checks.append(Check(status, name, detail))

    def passed(self, name: str, detail: str) -> None:
        self.add("PASS", name, detail)

    def warned(self, name: str, detail: str) -> None:
        self.add("WARN", name, detail)

    def failed(self, name: str, detail: str) -> None:
        self.add("FAIL", name, detail)

    def info(self, name: str, detail: str) -> None:
        self.add("INFO", name, detail)

    @property
    def failed_count(self) -> int:
        return sum(item.status == "FAIL" for item in self.checks)

    @property
    def warning_count(self) -> int:
        return sum(item.status == "WARN" for item in self.checks)


class ExitOneArgumentParser(argparse.ArgumentParser):
    """Keep the command's documented process statuses limited to zero and one."""

    def error(self, message: str) -> None:
        self.print_usage(sys.stderr)
        self.exit(1, f"{self.prog}: error: {message}\n")


@dataclass
class Problems:
    counts: Counter[str] = field(default_factory=Counter)
    first: dict[str, str] = field(default_factory=dict)

    def add(self, kind: str, detail: str) -> None:
        self.counts[kind] += 1
        self.first.setdefault(kind, detail)

    def describe(self) -> str:
        parts = []
        for kind in sorted(self.counts):
            detail = self.first[kind]
            parts.append(f"{kind}={self.counts[kind]} (first: {detail})")
        return "; ".join(parts)

    def __bool__(self) -> bool:
        return bool(self.counts)


@dataclass(frozen=True)
class PoseRecord:
    stamp_ns: int
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    covariance: tuple[float, ...]


@dataclass(frozen=True)
class KinematicRecord(PoseRecord):
    twist: tuple[float, ...]
    twist_covariance: tuple[float, ...]


@dataclass(frozen=True)
class TwistRecord:
    stamp_ns: int
    twist: tuple[float, ...]
    covariance: tuple[float, ...]


@dataclass(frozen=True)
class TransformRecord:
    stamp_ns: int
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]


@dataclass
class StampedAudit:
    stamps: list[int] = field(default_factory=list)
    records: list[Any] = field(default_factory=list)
    problems: Problems = field(default_factory=Problems)

    def add_stamp(self, stamp_ns: int) -> None:
        if stamp_ns <= 0:
            self.problems.add("non_positive_stamp", format_stamp(stamp_ns))
        if self.stamps and stamp_ns <= self.stamps[-1]:
            self.problems.add(
                "non_increasing_stamp",
                f"{format_stamp(self.stamps[-1])} -> {format_stamp(stamp_ns)}",
            )
        self.stamps.append(stamp_ns)


@dataclass(frozen=True)
class DiagnosticSnapshot:
    bag_timestamp_ns: int
    header_stamp_ns: int
    level: int
    message: str
    values: dict[str, str]


@dataclass
class DiagnosticAudit:
    counts: Counter[str] = field(default_factory=Counter)
    levels: defaultdict[str, Counter[int]] = field(
        default_factory=lambda: defaultdict(Counter)
    )
    last: dict[str, DiagnosticSnapshot] = field(default_factory=dict)
    deskew_fields: Counter[str] = field(default_factory=Counter)
    deskew_fallbacks: Counter[str] = field(default_factory=Counter)
    tracking_modes: Counter[str] = field(default_factory=Counter)
    tracking_outcomes: list[tuple[str, str, str, str, str, str, str]] = field(
        default_factory=list
    )
    gnss_state_transitions: list[str] = field(default_factory=list)
    deskew_messages: Counter[str] = field(default_factory=Counter)
    tracking_reset_evidence: list[str] = field(default_factory=list)

    def consume(self, message: Any, bag_timestamp_ns: int) -> None:
        for status in message.status:
            name = str(status.name).lstrip("/")
            values = {str(entry.key): str(entry.value) for entry in status.values}
            snapshot = DiagnosticSnapshot(
                bag_timestamp_ns=bag_timestamp_ns,
                header_stamp_ns=stamp_to_ns(message.header.stamp),
                level=diagnostic_level(status.level),
                message=str(status.message),
                values=values,
            )
            self.counts[name] += 1
            self.levels[name][snapshot.level] += 1
            self.last[name] = snapshot

            if name == DESKEW_DIAGNOSTIC:
                self.deskew_fields[values.get("time_field_name", "<missing>")] += 1
                self.deskew_fallbacks[values.get("used_linear_fallback", "<missing>")] += 1
                self.deskew_messages[str(status.message)] += 1

            if name == GNSS_DIAGNOSTIC:
                state = values.get("recovery.state", "<missing>")
                if not self.gnss_state_transitions or self.gnss_state_transitions[-1] != state:
                    self.gnss_state_transitions.append(state)

            if name == TRACKING_DIAGNOSTIC:
                self.tracking_modes[values.get("lidar_tracking_mode", "<missing>")] += 1
                outcome = tuple(
                    values.get(key, "<missing>")
                    for key in (
                        "lidar_valid",
                        "lidar_rejection_reason",
                        "lidar_registration_source",
                        "lidar_raw_dx",
                        "lidar_raw_dy",
                        "lidar_raw_dyaw",
                        "lidar_fitness",
                    )
                )
                if not self.tracking_outcomes or self.tracking_outcomes[-1] != outcome:
                    self.tracking_outcomes.append(outcome)
                for key in ("tracking_reset_count", "lidar_tracking_reset_count"):
                    if positive_integer(values.get(key)):
                        self.tracking_reset_evidence.append(
                            f"{key}={values[key]} at {format_stamp(bag_timestamp_ns)}"
                        )
                normalized_message = str(status.message).lower().replace("_", " ")
                if "tracking reset" in normalized_message:
                    self.tracking_reset_evidence.append(
                        f"diagnostic message at {format_stamp(bag_timestamp_ns)}"
                    )


@dataclass
class BagAudit:
    topic_counts: Counter[str] = field(default_factory=Counter)
    total_messages: int = 0
    clock: StampedAudit = field(default_factory=StampedAudit)
    state: StampedAudit = field(default_factory=StampedAudit)
    twist: StampedAudit = field(default_factory=StampedAudit)
    pose: StampedAudit = field(default_factory=StampedAudit)
    acceleration: StampedAudit = field(default_factory=StampedAudit)
    map_base_tf: StampedAudit = field(default_factory=StampedAudit)
    dynamic_base_parents: Counter[str] = field(default_factory=Counter)
    static_transforms: defaultdict[tuple[str, str], list[TransformRecord]] = field(
        default_factory=lambda: defaultdict(list)
    )
    last_header_stamps: dict[str, int] = field(default_factory=dict)
    diagnostics: DiagnosticAudit = field(default_factory=DiagnosticAudit)
    aggregated_diagnostic_names: Counter[str] = field(default_factory=Counter)
    deserialize_problems: Problems = field(default_factory=Problems)


def diagnostic_level(value: Any) -> int:
    if isinstance(value, (bytes, bytearray)):
        return value[0] if value else 0
    return int(value)


def positive_integer(value: str | None) -> bool:
    if value is None:
        return False
    try:
        return int(value) > 0
    except ValueError:
        return False


def format_stamp(stamp_ns: int) -> str:
    sign = "-" if stamp_ns < 0 else ""
    absolute = abs(stamp_ns)
    return f"{sign}{absolute // 1_000_000_000}.{absolute % 1_000_000_000:09d}"


def stamp_to_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def pose_values(pose: Any) -> tuple[tuple[float, ...], tuple[float, ...]]:
    position = (float(pose.position.x), float(pose.position.y), float(pose.position.z))
    orientation = (
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )
    return position, orientation


def twist_values(twist: Any) -> tuple[float, ...]:
    return (
        float(twist.linear.x),
        float(twist.linear.y),
        float(twist.linear.z),
        float(twist.angular.x),
        float(twist.angular.y),
        float(twist.angular.z),
    )


def accel_values(accel: Any) -> tuple[float, ...]:
    return (
        float(accel.linear.x),
        float(accel.linear.y),
        float(accel.linear.z),
        float(accel.angular.x),
        float(accel.angular.y),
        float(accel.angular.z),
    )


def audit_quaternion(
    quaternion: Sequence[float], stamp_ns: int, problems: Problems, prefix: str
) -> None:
    if not all(math.isfinite(value) for value in quaternion):
        problems.add(f"{prefix}_non_finite_quaternion", format_stamp(stamp_ns))
        return
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-6):
        problems.add(
            f"{prefix}_non_unit_quaternion",
            f"stamp={format_stamp(stamp_ns)}, norm={norm:.12g}",
        )


def audit_covariance(
    covariance: Sequence[float], stamp_ns: int, problems: Problems, prefix: str
) -> None:
    values = np.asarray(covariance, dtype=np.float64)
    stamp_text = format_stamp(stamp_ns)
    if values.size != 36:
        problems.add(f"{prefix}_covariance_size", f"stamp={stamp_text}, size={values.size}")
        return
    if not np.isfinite(values).all():
        problems.add(f"{prefix}_covariance_non_finite", f"stamp={stamp_text}")
        return

    matrix = values.reshape((6, 6))
    # Do not scale tolerances from the largest entry: this covariance mixes
    # metres and radians and intentionally carries 1e6 on unobserved axes.
    # A global scale would hide a negative variance or asymmetric small term.
    symmetry_tolerance = 1.0e-9
    maximum_asymmetry = float(np.max(np.abs(matrix - matrix.T)))
    if maximum_asymmetry > symmetry_tolerance:
        problems.add(
            f"{prefix}_covariance_not_symmetric",
            f"stamp={stamp_text}, max_asymmetry={maximum_asymmetry:.12g}",
        )
        return

    symmetric = 0.5 * (matrix + matrix.T)
    minimum_eigenvalue = float(np.linalg.eigvalsh(symmetric)[0])
    psd_tolerance = 1.0e-9
    if minimum_eigenvalue < -psd_tolerance:
        problems.add(
            f"{prefix}_covariance_not_psd",
            f"stamp={stamp_text}, min_eigenvalue={minimum_eigenvalue:.12g}",
        )


def consume_kinematic(message: Any, audit: StampedAudit) -> None:
    stamp_ns = stamp_to_ns(message.header.stamp)
    audit.add_stamp(stamp_ns)
    if message.header.frame_id != "map" or message.child_frame_id != "base_link":
        audit.problems.add(
            "unexpected_frames",
            f"stamp={format_stamp(stamp_ns)}, "
            f"{message.header.frame_id!r}->{message.child_frame_id!r}",
        )

    position, orientation = pose_values(message.pose.pose)
    twist = twist_values(message.twist.twist)
    if not all(math.isfinite(value) for value in position + orientation + twist):
        audit.problems.add("non_finite_state", format_stamp(stamp_ns))
    audit_quaternion(orientation, stamp_ns, audit.problems, "pose")
    audit_covariance(message.pose.covariance, stamp_ns, audit.problems, "pose")
    audit_covariance(message.twist.covariance, stamp_ns, audit.problems, "twist")
    audit.records.append(
        KinematicRecord(
            stamp_ns,
            position,
            orientation,
            tuple(float(x) for x in message.pose.covariance),
            twist,
            tuple(float(x) for x in message.twist.covariance),
        )
    )


def consume_twist(message: Any, audit: StampedAudit) -> None:
    stamp_ns = stamp_to_ns(message.header.stamp)
    audit.add_stamp(stamp_ns)
    if message.header.frame_id != "base_link":
        audit.problems.add(
            "unexpected_frame",
            f"stamp={format_stamp(stamp_ns)}, frame={message.header.frame_id!r}",
        )
    values = twist_values(message.twist.twist)
    if not all(math.isfinite(value) for value in values):
        audit.problems.add("non_finite_twist", format_stamp(stamp_ns))
    audit_covariance(message.twist.covariance, stamp_ns, audit.problems, "twist")
    audit.records.append(
        TwistRecord(
            stamp_ns,
            values,
            tuple(float(x) for x in message.twist.covariance),
        )
    )


def consume_pose(message: Any, audit: StampedAudit) -> None:
    stamp_ns = stamp_to_ns(message.header.stamp)
    audit.add_stamp(stamp_ns)
    if message.header.frame_id != "map":
        audit.problems.add(
            "unexpected_frame",
            f"stamp={format_stamp(stamp_ns)}, frame={message.header.frame_id!r}",
        )
    position, orientation = pose_values(message.pose.pose)
    if not all(math.isfinite(value) for value in position + orientation):
        audit.problems.add("non_finite_pose", format_stamp(stamp_ns))
    audit_quaternion(orientation, stamp_ns, audit.problems, "pose")
    audit_covariance(message.pose.covariance, stamp_ns, audit.problems, "pose")
    audit.records.append(
        PoseRecord(
            stamp_ns,
            position,
            orientation,
            tuple(float(x) for x in message.pose.covariance),
        )
    )


def consume_acceleration(message: Any, audit: StampedAudit) -> None:
    stamp_ns = stamp_to_ns(message.header.stamp)
    audit.add_stamp(stamp_ns)
    if message.header.frame_id != "base_link":
        audit.problems.add(
            "unexpected_frame",
            f"stamp={format_stamp(stamp_ns)}, frame={message.header.frame_id!r}",
        )
    values = accel_values(message.accel.accel)
    if not all(math.isfinite(value) for value in values):
        audit.problems.add("non_finite_acceleration", format_stamp(stamp_ns))
    audit_covariance(message.accel.covariance, stamp_ns, audit.problems, "acceleration")


def consume_clock(message: Any, audit: StampedAudit) -> None:
    audit.add_stamp(stamp_to_ns(message.clock))


def consume_last_header_stamp(message: Any, topic: str, audit: BagAudit) -> None:
    audit.last_header_stamps[topic] = stamp_to_ns(message.header.stamp)


def transform_values(transform: Any) -> tuple[tuple[float, ...], tuple[float, ...]]:
    translation = (
        float(transform.translation.x),
        float(transform.translation.y),
        float(transform.translation.z),
    )
    rotation = (
        float(transform.rotation.x),
        float(transform.rotation.y),
        float(transform.rotation.z),
        float(transform.rotation.w),
    )
    return translation, rotation


def consume_tf(message: Any, audit: BagAudit) -> None:
    for transform in message.transforms:
        if transform.child_frame_id != "base_link":
            continue
        audit.dynamic_base_parents[str(transform.header.frame_id)] += 1
        if transform.header.frame_id != "map":
            continue
        stamp_ns = stamp_to_ns(transform.header.stamp)
        audit.map_base_tf.add_stamp(stamp_ns)
        translation, rotation = transform_values(transform.transform)
        if not all(math.isfinite(value) for value in translation + rotation):
            audit.map_base_tf.problems.add("non_finite_transform", format_stamp(stamp_ns))
        audit_quaternion(
            rotation, stamp_ns, audit.map_base_tf.problems, "transform"
        )
        audit.map_base_tf.records.append(
            TransformRecord(stamp_ns, translation, rotation)
        )


def consume_static_tf(message: Any, audit: BagAudit) -> None:
    for transform in message.transforms:
        stamp_ns = stamp_to_ns(transform.header.stamp)
        translation, rotation = transform_values(transform.transform)
        record = TransformRecord(stamp_ns, translation, rotation)
        frames = (transform.header.frame_id, transform.child_frame_id)
        audit.static_transforms[frames].append(record)


def level_for_profile(
    profile: str, report: Report, passed: bool, name: str, detail: str
) -> None:
    if passed:
        report.passed(name, detail)
    elif profile == "hesai-rosbag23":
        report.failed(name, detail)
    else:
        report.warned(name, detail)


def close_sequence(
    left: Sequence[float], right: Sequence[float], tolerance: float = 1.0e-9
) -> bool:
    return len(left) == len(right) and all(
        math.isclose(a, b, rel_tol=tolerance, abs_tol=tolerance)
        for a, b in zip(left, right, strict=True)
    )


def close_quaternion(
    left: Sequence[float], right: Sequence[float], tolerance: float = 1.0e-9
) -> bool:
    if len(left) != 4 or len(right) != 4:
        return False
    direct = max(abs(a - b) for a, b in zip(left, right, strict=True))
    negated = max(abs(a + b) for a, b in zip(left, right, strict=True))
    return min(direct, negated) <= tolerance


def first_stamp_mismatch(reference: Sequence[int], candidate: Sequence[int]) -> str:
    for index, (left, right) in enumerate(zip(reference, candidate)):
        if left != right:
            return (
                f"index {index}: {format_stamp(left)} != {format_stamp(right)}"
            )
    if len(reference) != len(candidate):
        return f"counts differ: {len(reference)} != {len(candidate)}"
    return "none"


def status_values_match(values: dict[str, str], expected: dict[str, str]) -> list[str]:
    return [
        f"{key}={values.get(key, '<missing>')!r} (expected {wanted!r})"
        for key, wanted in expected.items()
        if values.get(key) != wanted
    ]


def contains_subsequence(values: Sequence[str], expected: Sequence[str]) -> bool:
    index = 0
    for value in values:
        if index < len(expected) and value == expected[index]:
            index += 1
    return index == len(expected)


def check_topics(
    topic_types: dict[str, str], audit: BagAudit, profile: str, report: Report
) -> None:
    required = dict(BASE_REQUIRED_TOPICS)
    if profile == "generic":
        # A generic bag can legitimately use base_link sensor messages and
        # therefore need no static sensor transform at all.
        required.pop(TF_STATIC_TOPIC)
    if profile == "hesai-rosbag23":
        required.update(HESAI_REQUIRED_TOPICS)

    for topic, expected_type in required.items():
        if topic not in topic_types:
            report.failed("required topic", f"{topic}: missing")
            continue
        actual_type = topic_types[topic]
        count = audit.topic_counts[topic]
        if actual_type != expected_type:
            report.failed(
                "required topic",
                f"{topic}: type {actual_type!r}, expected {expected_type!r}",
            )
        elif count == 0:
            report.failed("required topic", f"{topic}: present but contains no messages")
        else:
            report.passed("required topic", f"{topic}: {count} {actual_type}")

    if profile == "hesai-rosbag23":
        for topic, expected_type in HESAI_OPTIONAL_TOPICS.items():
            if topic not in topic_types or audit.topic_counts[topic] == 0:
                report.info("optional evidence topic", f"{topic}: not recorded")
            elif topic_types[topic] != expected_type:
                report.warned(
                    "optional evidence topic",
                    f"{topic}: type {topic_types[topic]!r}, expected {expected_type!r}",
                )
            else:
                report.passed(
                    "optional evidence topic", f"{topic}: {audit.topic_counts[topic]} messages"
                )


def check_interface(audit: BagAudit, profile: str, report: Report) -> None:
    if audit.clock.problems:
        report.failed("simulation clock contract", audit.clock.problems.describe())
    elif audit.clock.stamps:
        report.passed(
            "simulation clock contract",
            f"{len(audit.clock.stamps)} positive, strictly increasing samples",
        )
    else:
        report.info(
            "simulation clock contract", "not evaluated because /clock was unreadable"
        )

    if audit.state.problems:
        report.failed("kinematic state contract", audit.state.problems.describe())
    elif audit.state.stamps:
        report.passed(
            "kinematic state contract",
            f"{len(audit.state.stamps)} map->base_link states; finite, unit quaternion, "
            "symmetric PSD pose/twist covariance, strictly increasing stamps",
        )
    else:
        report.info(
            "kinematic state contract", "not evaluated because no state message was readable"
        )

    for name, stream in (
        ("twist", audit.twist),
        ("pose", audit.pose),
        ("acceleration", audit.acceleration),
    ):
        if stream.problems:
            report.failed(f"{name} message contract", stream.problems.describe())
        elif stream.stamps:
            report.passed(f"{name} message contract", f"{len(stream.stamps)} valid messages")
        else:
            report.info(
                f"{name} message contract",
                "not evaluated because no message was readable",
            )

    if audit.map_base_tf.problems:
        report.failed("map->base_link TF contract", audit.map_base_tf.problems.describe())
    elif audit.map_base_tf.stamps:
        report.passed(
            "map->base_link TF contract",
            f"{len(audit.map_base_tf.stamps)} finite transforms with unit quaternions",
        )
    else:
        report.failed("map->base_link TF contract", "no map->base_link transform in /tf")

    if set(audit.dynamic_base_parents) == {"map"}:
        report.passed(
            "dynamic base_link TF ownership",
            f"only map -> base_link ({audit.dynamic_base_parents['map']} transforms)",
        )
    else:
        observed = ", ".join(
            f"{parent}->base_link={count}"
            for parent, count in sorted(audit.dynamic_base_parents.items())
        ) or "none"
        report.failed(
            "dynamic base_link TF ownership",
            f"expected only map -> base_link; observed {observed}",
        )

    state_stamps = audit.state.stamps
    aligned = {
        "twist": audit.twist.stamps,
        "pose": audit.pose.stamps,
        "acceleration": audit.acceleration.stamps,
        "map->base_link TF": audit.map_base_tf.stamps,
    }
    mismatches = [
        f"{name}: {first_stamp_mismatch(state_stamps, stamps)}"
        for name, stamps in aligned.items()
        if stamps != state_stamps
    ]
    if not state_stamps:
        report.info("adapter count/stamp alignment", "not evaluated without kinematic states")
    elif mismatches:
        report.failed("adapter count/stamp alignment", "; ".join(mismatches))
    else:
        count = len(state_stamps)
        report.passed(
            "adapter count/stamp alignment",
            f"state=twist=pose=acceleration=map->base_link TF={count}; stamps match exactly",
        )

    twist_value_problems = Problems()
    if audit.twist.stamps == state_stamps:
        for state_record, twist_record in zip(
            audit.state.records, audit.twist.records, strict=True
        ):
            if not close_sequence(state_record.twist, twist_record.twist):
                twist_value_problems.add(
                    "value_mismatch", format_stamp(state_record.stamp_ns)
                )
            if not close_sequence(
                state_record.twist_covariance, twist_record.covariance
            ):
                twist_value_problems.add(
                    "covariance_mismatch", format_stamp(state_record.stamp_ns)
                )
    if twist_value_problems:
        report.failed("twist/state value alignment", twist_value_problems.describe())
    elif audit.twist.stamps == state_stamps and state_stamps:
        report.passed(
            "twist/state value alignment",
            "base-frame twist and covariance match every state",
        )

    pose_value_problems = Problems()
    if audit.pose.stamps == state_stamps:
        for state_record, pose_record in zip(
            audit.state.records, audit.pose.records, strict=True
        ):
            if not close_sequence(state_record.position, pose_record.position):
                pose_value_problems.add(
                    "position_mismatch", format_stamp(state_record.stamp_ns)
                )
            if not close_quaternion(state_record.orientation, pose_record.orientation):
                pose_value_problems.add(
                    "orientation_mismatch", format_stamp(state_record.stamp_ns)
                )
            if not close_sequence(state_record.covariance, pose_record.covariance):
                pose_value_problems.add(
                    "covariance_mismatch", format_stamp(state_record.stamp_ns)
                )
    if pose_value_problems:
        report.failed("pose/state value alignment", pose_value_problems.describe())
    elif audit.pose.stamps == state_stamps and state_stamps:
        report.passed("pose/state value alignment", "pose and covariance match every state")

    tf_value_problems = Problems()
    if audit.map_base_tf.stamps == state_stamps:
        for state_record, tf_record in zip(
            audit.state.records, audit.map_base_tf.records, strict=True
        ):
            if not close_sequence(state_record.position, tf_record.translation):
                tf_value_problems.add(
                    "translation_mismatch", format_stamp(state_record.stamp_ns)
                )
            if not close_quaternion(state_record.orientation, tf_record.rotation):
                tf_value_problems.add("rotation_mismatch", format_stamp(state_record.stamp_ns))
    if tf_value_problems:
        report.failed("TF/state value alignment", tf_value_problems.describe())
    elif audit.map_base_tf.stamps == state_stamps and state_stamps:
        report.passed("TF/state value alignment", "transform matches every state pose")

    if audit.clock.stamps and state_stamps:
        final_delta_sec = (audit.clock.stamps[-1] - state_stamps[-1]) * 1.0e-9
        if abs(final_delta_sec) <= 1.0:
            report.passed(
                "end-of-replay state coverage",
                f"last state is {final_delta_sec:.6f} s behind the final /clock",
            )
        else:
            report.failed(
                "end-of-replay state coverage",
                f"last state is {final_delta_sec:.6f} s behind the final /clock",
            )

    if len(state_stamps) >= 2:
        active_duration_sec = (state_stamps[-1] - state_stamps[0]) * 1.0e-9
        if active_duration_sec > 0.0:
            output_rate = (len(state_stamps) - 1) / active_duration_sec
            if profile == "hesai-rosbag23" and output_rate < 45.0:
                report.failed(
                    "Autoware state effective rate",
                    f"{output_rate:.3f} Hz is below the 45 Hz profile floor",
                )
            else:
                report.passed(
                    "Autoware state effective rate",
                    f"{output_rate:.3f} Hz over {active_duration_sec:.3f} s",
                )

        maximum_step = 0.0
        maximum_step_stamp = state_stamps[0]
        for previous, current in zip(audit.state.records, audit.state.records[1:]):
            step = math.hypot(
                current.position[0] - previous.position[0],
                current.position[1] - previous.position[1],
            )
            if step > maximum_step:
                maximum_step = step
                maximum_step_stamp = current.stamp_ns
        step_detail = (
            f"maximum consecutive XY step={maximum_step:.6f} m at "
            f"{format_stamp(maximum_step_stamp)}"
        )
        if profile == "hesai-rosbag23" and maximum_step > 0.55:
            report.failed("consecutive XY step", step_detail + " (limit 0.55 m)")
        elif profile == "hesai-rosbag23" and maximum_step > 0.5:
            report.warned(
                "consecutive XY step", step_detail + " (practical limit 0.55 m)"
            )
        else:
            report.passed("consecutive XY step", step_detail)


def check_hesai_static_transforms(audit: BagAudit, report: Report) -> None:
    imu_half_roll = 0.5 * 3.14159
    expected = {
        ("base_link", "lidar/0"): ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ("base_link", "imu"): (
            (0.0, 0.0, -0.1874),
            (math.sin(imu_half_roll), 0.0, 0.0, math.cos(imu_half_roll)),
        ),
        ("base_link", "gnss/0"): ((0.0, 0.0, -0.1326), (0.0, 0.0, 0.0, 1.0)),
    }
    problems = Problems()
    for frames, (translation, rotation) in expected.items():
        records = audit.static_transforms.get(frames, [])
        frame_text = f"{frames[0]}->{frames[1]}"
        if not records:
            problems.add("missing_transform", frame_text)
            continue
        for record in records:
            if not close_sequence(record.translation, translation, tolerance=1.0e-6):
                problems.add(
                    "translation_mismatch", f"{frame_text}: actual={record.translation!r}"
                )
            if not close_quaternion(record.rotation, rotation, tolerance=1.0e-6):
                problems.add("rotation_mismatch", f"{frame_text}: actual={record.rotation!r}")
        if len(records) > 1:
            problems.add(
                "duplicate_transform", f"{frame_text}: recorded {len(records)} times"
            )

    extras = sorted(set(audit.static_transforms) - set(expected))
    if extras:
        formatted = ", ".join(f"{parent}->{child}" for parent, child in extras)
        problems.add("extra_transform", formatted)
    if problems:
        report.failed("Hesai calibrated static TF", problems.describe())
    else:
        report.passed(
            "Hesai calibrated static TF",
            "base_link->lidar/0, base_link->imu, and base_link->gnss/0 match calibration",
        )


def check_hesai_end_recency(audit: BagAudit, report: Report) -> None:
    if not audit.clock.stamps:
        report.failed("Hesai end-of-replay recency", "final /clock is unavailable")
        return

    final_clock = audit.clock.stamps[-1]
    stream_details = []
    stream_problems = []
    for topic in HESAI_REQUIRED_TOPICS:
        stamp_ns = audit.last_header_stamps.get(topic)
        if stamp_ns is None:
            stream_problems.append(f"{topic}=missing")
            continue
        age_sec = (final_clock - stamp_ns) * 1.0e-9
        stream_details.append(f"{topic}={age_sec:.3f}s")
        if abs(age_sec) > 1.0:
            stream_problems.append(f"{topic}={age_sec:.3f}s")
    if stream_problems:
        report.failed(
            "Hesai estimator stream recency",
            "outside +/-1 s of final /clock: " + ", ".join(stream_problems),
        )
    else:
        report.passed(
            "Hesai estimator stream recency",
            "final ages: " + ", ".join(stream_details),
        )

    diagnostic_details = []
    diagnostic_problems = []
    for name in (
        ADAPTER_DIAGNOSTIC,
        GNSS_DIAGNOSTIC,
        DESKEW_DIAGNOSTIC,
        TRACKING_DIAGNOSTIC,
    ):
        snapshot = audit.diagnostics.last.get(name)
        if snapshot is None:
            diagnostic_problems.append(f"{name}=missing")
            continue
        age_sec = (final_clock - snapshot.header_stamp_ns) * 1.0e-9
        diagnostic_details.append(f"{name}={age_sec:.3f}s")
        if abs(age_sec) > 1.0:
            diagnostic_problems.append(f"{name}={age_sec:.3f}s")
    if diagnostic_problems:
        report.failed(
            "Hesai diagnostic recency",
            "outside +/-1 s of final /clock: " + ", ".join(diagnostic_problems),
        )
    else:
        report.passed(
            "Hesai diagnostic recency",
            "final ages: " + ", ".join(diagnostic_details),
        )


def check_adapter_diagnostic(audit: BagAudit, report: Report) -> None:
    snapshot = audit.diagnostics.last.get(ADAPTER_DIAGNOSTIC)
    if snapshot is None:
        report.failed("adapter final diagnostic", f"{ADAPTER_DIAGNOSTIC}: missing")
        return
    mismatches = status_values_match(
        snapshot.values,
        {"rejected_count": "0", "last_rejection_reason": "none"},
    )
    if mismatches:
        report.failed("adapter final diagnostic", "; ".join(mismatches))
    else:
        report.passed(
            "adapter final diagnostic",
            f"rejected_count=0, last_rejection_reason=none "
            f"({audit.diagnostics.counts[ADAPTER_DIAGNOSTIC]} samples)",
        )


def check_autoware_monitor_diagnostics(audit: BagAudit, report: Report) -> None:
    required_keys = {
        POSE_INSTABILITY_DIAGNOSTIC: {
            "diff_position_x:validation_enabled",
            "diff_position_x:threshold",
            "diff_position_x:value",
            "diff_position_x:status",
            "diff_angle_z:validation_enabled",
            "diff_angle_z:threshold",
            "diff_angle_z:value",
            "diff_angle_z:status",
        },
        LOCALIZATION_ERROR_DIAGNOSTIC: {
            "localization_error_ellipse",
            "localization_error_ellipse_lateral_direction",
        },
    }
    level_names = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
    for name, keys in required_keys.items():
        snapshot = audit.diagnostics.last.get(name)
        if snapshot is None:
            report.failed("Autoware monitor diagnostic", f"{name}: missing")
            continue
        missing_keys = sorted(keys - set(snapshot.values))
        if missing_keys:
            report.failed(
                "Autoware monitor diagnostic schema",
                f"{name}: missing {', '.join(missing_keys)}",
            )
        else:
            report.passed(
                "Autoware monitor diagnostic schema",
                f"{name}: latest schema present; "
                f"{audit.diagnostics.counts[name]} samples observed",
            )

        distribution = audit.diagnostics.levels[name]
        unknown_levels = sorted(set(distribution) - set(level_names))
        if unknown_levels:
            report.failed(
                "Autoware monitor diagnostic level",
                f"{name}: unsupported levels {unknown_levels}",
            )
        formatted = ", ".join(
            f"{level_names.get(level, str(level))}={count}"
            for level, count in sorted(distribution.items())
        )
        # Real-bag levels are characterization, not a wiring acceptance gate:
        # the upstream thresholds may not match this estimator's covariance
        # calibration, and the pose/twist streams share a common source.
        report.info("Autoware monitor level distribution", f"{name}: {formatted}")

        if audit.state.stamps:
            age_sec = (audit.state.stamps[-1] - snapshot.header_stamp_ns) * 1.0e-9
            if abs(age_sec) <= 1.0:
                report.passed(
                    "Autoware monitor end recency",
                    f"{name}: {age_sec:.6f} s behind the final state",
                )
            else:
                report.failed(
                    "Autoware monitor end recency",
                    f"{name}: {age_sec:.6f} s behind the final state",
                )


def check_aggregated_monitor_diagnostics(audit: BagAudit, report: Report) -> None:
    observed = set(audit.aggregated_diagnostic_names)
    missing = sorted(AGGREGATED_MONITOR_DIAGNOSTICS - observed)
    if missing:
        report.failed(
            "aggregated Autoware monitor diagnostics",
            "missing grouped leaves: " + ", ".join(missing),
        )
        return
    details = ", ".join(
        f"{name}={audit.aggregated_diagnostic_names[name]}"
        for name in sorted(AGGREGATED_MONITOR_DIAGNOSTICS)
    )
    report.passed("aggregated Autoware monitor diagnostics", details)


def check_gnss_diagnostic(audit: BagAudit, profile: str, report: Report) -> None:
    snapshot = audit.diagnostics.last.get(GNSS_DIAGNOSTIC)
    if snapshot is None:
        level_for_profile(
            profile,
            report,
            False,
            "GNSS fusion final diagnostic",
            f"{GNSS_DIAGNOSTIC}: missing",
        )
        return
    expected = {
        "recovery.state": "tracking",
        "recovery.mode": "full_se2",
        "recovery.position_fused": "true",
        "recovery.yaw_fused": "true",
    }
    mismatches = status_values_match(snapshot.values, expected)
    if mismatches:
        level_for_profile(
            profile, report, False, "GNSS fusion final diagnostic", "; ".join(mismatches)
        )
    else:
        report.passed(
            "GNSS fusion final diagnostic",
            "tracking/full_se2 with position_fused=true and yaw_fused=true",
        )

    transitions = audit.diagnostics.gnss_state_transitions
    recovery_sequence = ("outage", "reacquiring", "recovering", "tracking")
    if contains_subsequence(transitions, recovery_sequence):
        report.passed(
            "GNSS outage recovery sequence",
            " -> ".join(transitions),
        )
    else:
        level_for_profile(
            profile,
            report,
            False,
            "GNSS outage recovery sequence",
            f"required {' -> '.join(recovery_sequence)}; observed {' -> '.join(transitions)}",
        )

    out_of_order_drops = snapshot.values.get("output.out_of_order_drop_count", "0")
    try:
        drop_count = int(out_of_order_drops)
    except ValueError:
        report.warned(
            "fusion stale-output guard",
            f"invalid output.out_of_order_drop_count={out_of_order_drops!r}",
        )
    else:
        if drop_count > 0:
            report.warned(
                "fusion stale-output guard",
                f"suppressed {drop_count} stale publish request(s); "
                "published output remains monotonic",
            )
        else:
            report.passed("fusion stale-output guard", "no stale publish requests")


def check_deskew_diagnostic(audit: BagAudit, profile: str, report: Report) -> None:
    snapshot = audit.diagnostics.last.get(DESKEW_DIAGNOSTIC)
    if snapshot is None:
        level_for_profile(
            profile, report, False, "deskew diagnostic", f"{DESKEW_DIAGNOSTIC}: missing"
        )
        return

    wrong_fields = sum(
        count
        for value, count in audit.diagnostics.deskew_fields.items()
        if value != "time_stamp"
    )
    wrong_fallbacks = sum(
        count
        for value, count in audit.diagnostics.deskew_fallbacks.items()
        if value != "false"
    )
    final_mismatches = status_values_match(
        snapshot.values,
        {"time_field_name": "time_stamp", "used_linear_fallback": "false"},
    )
    details = []
    if wrong_fields:
        details.append(
            f"{wrong_fields} diagnostic samples did not report time_field_name=time_stamp "
            f"({dict(audit.diagnostics.deskew_fields)!r})"
        )
    if wrong_fallbacks:
        details.append(
            f"{wrong_fallbacks} diagnostic samples did not report used_linear_fallback=false "
            f"({dict(audit.diagnostics.deskew_fallbacks)!r})"
        )
    details.extend(final_mismatches)

    successful = audit.diagnostics.deskew_messages.get(
        "deskew OK using per-point timestamps", 0
    )
    rejected = {
        message: count
        for message, count in audit.diagnostics.deskew_messages.items()
        if message.startswith("deskew rejected:")
    }
    total = audit.diagnostics.counts[DESKEW_DIAGNOSTIC]
    unknown_messages = total - successful - sum(rejected.values())
    success_rate = successful / total if total else 0.0
    unexpected_rejections = {
        message: count
        for message, count in rejected.items()
        if "IMU does not cover the full scan interval" not in message
    }
    if success_rate < 0.99:
        details.append(
            f"deskew success {successful}/{total} ({100.0 * success_rate:.4f}%) is below 99%"
        )
    if unknown_messages:
        details.append(f"{unknown_messages} unclassified deskew diagnostic message(s)")
    if unexpected_rejections:
        details.append(f"unexpected deskew rejection(s): {unexpected_rejections!r}")
    if details:
        level_for_profile(profile, report, False, "deskew diagnostic", "; ".join(details))
    else:
        report.passed(
            "deskew diagnostic",
            f"{successful}/{total} successful ({100.0 * success_rate:.4f}%); "
            "time_stamp used, no linear fallback, only finite-bag IMU coverage rejects",
        )


def check_tracking_diagnostic(
    audit: BagAudit, profile: str, expected_mode: str, report: Report
) -> None:
    snapshot = audit.diagnostics.last.get(TRACKING_DIAGNOSTIC)
    if snapshot is None:
        level_for_profile(
            profile, report, False, "LiDAR tracking diagnostic", f"{TRACKING_DIAGNOSTIC}: missing"
        )
        return

    wrong_modes = sum(
        count
        for value, count in audit.diagnostics.tracking_modes.items()
        if value != expected_mode
    )
    details = []
    if wrong_modes:
        details.append(
            f"{wrong_modes} diagnostic samples were not {expected_mode} "
            f"({dict(audit.diagnostics.tracking_modes)!r})"
        )
    if details:
        report.failed("LiDAR tracking diagnostic", "; ".join(details))
    elif audit.diagnostics.tracking_reset_evidence:
        report.failed(
            "LiDAR tracking reset",
            "recorded evidence: " + ", ".join(audit.diagnostics.tracking_reset_evidence[:3]),
        )
    else:
        report.passed(
            "LiDAR tracking diagnostic",
            f"{expected_mode}; no recorded tracking-reset evidence in "
            f"{audit.diagnostics.counts[TRACKING_DIAGNOSTIC]} samples",
        )

    attempts = [
        outcome
        for outcome in audit.diagnostics.tracking_outcomes
        if outcome[1] not in ("not_evaluated", "<missing>")
    ]
    accepted = sum(
        outcome[0] == "true"
        and outcome[1] == "accepted"
        for outcome in attempts
    )
    acceptance_rate = accepted / len(attempts) if attempts else 0.0
    if not attempts:
        level_for_profile(
            profile,
            report,
            False,
            "LiDAR registration acceptance",
            "no registration attempts reconstructed from diagnostics",
        )
    elif acceptance_rate < 0.99:
        level_for_profile(
            profile,
            report,
            False,
            "LiDAR registration acceptance",
            f"{accepted}/{len(attempts)} ({100.0 * acceptance_rate:.4f}%) is below 99%",
        )
    else:
        report.passed(
            "LiDAR registration acceptance",
            f"{accepted}/{len(attempts)} ({100.0 * acceptance_rate:.4f}%)",
        )

    primary_source_count = sum(
        outcome[0] == "true"
        and outcome[1] == "accepted"
        and outcome[2] == expected_mode
        for outcome in attempts
    )
    if primary_source_count:
        report.passed(
            "LiDAR primary registration source",
            f"{expected_mode} observed in {primary_source_count} reconstructed attempt(s)",
        )
    else:
        report.failed(
            "LiDAR primary registration source",
            f"no {expected_mode} attempt observed",
        )

    # A tracking reset is a failed estimator contract for the supported
    # scan-to-scan odometer path.
    if details and audit.diagnostics.tracking_reset_evidence:
        report.failed(
            "LiDAR tracking reset",
            "recorded evidence: " + ", ".join(audit.diagnostics.tracking_reset_evidence[:3]),
        )


def resolve_bag_uri(argument: str) -> Path:
    path = Path(argument).expanduser().resolve()
    if not path.exists():
        raise RuntimeError(f"bag path does not exist: {path}")
    if path.is_file():
        if path.suffix.lower() != ".mcap":
            raise RuntimeError(f"expected an MCAP file, got: {path}")
        return path
    if not path.is_dir():
        raise RuntimeError(f"bag path is neither a directory nor a file: {path}")

    metadata = path / "metadata.yaml"
    if metadata.is_file():
        metadata_text = metadata.read_text(encoding="utf-8", errors="replace")
        if "storage_identifier: mcap" not in metadata_text:
            raise RuntimeError(f"bag metadata does not declare MCAP storage: {metadata}")
        return path

    files = sorted(path.glob("*.mcap"))
    if len(files) == 1:
        return files[0]
    if not files:
        raise RuntimeError(f"directory has no metadata.yaml or MCAP file: {path}")
    raise RuntimeError(
        f"directory has no metadata.yaml and contains {len(files)} MCAP files; "
        "pass one file explicitly"
    )


def open_reader(uri: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def message_classes(topic_types: dict[str, str]) -> dict[str, type[Any]]:
    expected = {
        CLOCK_TOPIC: BASE_REQUIRED_TOPICS[CLOCK_TOPIC],
        KINEMATIC_TOPIC: BASE_REQUIRED_TOPICS[KINEMATIC_TOPIC],
        TWIST_TOPIC: BASE_REQUIRED_TOPICS[TWIST_TOPIC],
        POSE_TOPIC: BASE_REQUIRED_TOPICS[POSE_TOPIC],
        ACCEL_TOPIC: BASE_REQUIRED_TOPICS[ACCEL_TOPIC],
        TF_TOPIC: BASE_REQUIRED_TOPICS[TF_TOPIC],
        TF_STATIC_TOPIC: BASE_REQUIRED_TOPICS[TF_STATIC_TOPIC],
        DIAGNOSTICS_TOPIC: BASE_REQUIRED_TOPICS[DIAGNOSTICS_TOPIC],
        DIAGNOSTICS_AGG_TOPIC: BASE_REQUIRED_TOPICS[DIAGNOSTICS_AGG_TOPIC],
    }
    expected.update(HESAI_REQUIRED_TOPICS)
    classes = {}
    for topic, expected_type in expected.items():
        if topic_types.get(topic) == expected_type:
            classes[topic] = get_message(expected_type)
    return classes


def read_bag(reader: rosbag2_py.SequentialReader, classes: dict[str, type[Any]]) -> BagAudit:
    audit = BagAudit()
    consumers = {
        CLOCK_TOPIC: lambda message: consume_clock(message, audit.clock),
        KINEMATIC_TOPIC: lambda message: consume_kinematic(message, audit.state),
        TWIST_TOPIC: lambda message: consume_twist(message, audit.twist),
        POSE_TOPIC: lambda message: consume_pose(message, audit.pose),
        ACCEL_TOPIC: lambda message: consume_acceleration(message, audit.acceleration),
        TF_TOPIC: lambda message: consume_tf(message, audit),
        TF_STATIC_TOPIC: lambda message: consume_static_tf(message, audit),
        DIAGNOSTICS_AGG_TOPIC: lambda message: audit.aggregated_diagnostic_names.update(
            str(status.name) for status in message.status
        ),
    }
    for topic in HESAI_REQUIRED_TOPICS:
        consumers[topic] = (
            lambda message, selected_topic=topic: consume_last_header_stamp(
                message, selected_topic, audit
            )
        )
    while reader.has_next():
        topic, serialized, bag_timestamp_ns = reader.read_next()
        audit.total_messages += 1
        audit.topic_counts[topic] += 1
        if topic not in classes:
            continue
        try:
            message = deserialize_message(serialized, classes[topic])
            if topic == DIAGNOSTICS_TOPIC:
                audit.diagnostics.consume(message, bag_timestamp_ns)
            else:
                consumers[topic](message)
        except Exception as error:  # Continue so the report includes all other evidence.
            detail = (
                f"bag_stamp={format_stamp(bag_timestamp_ns)}, "
                f"{type(error).__name__}: {error}"
            )
            audit.deserialize_problems.add(
                topic, detail
            )
    return audit


def print_report(
    requested_path: Path,
    resolved_uri: Path,
    profile: str,
    topic_types: dict[str, str],
    audit: BagAudit,
    report: Report,
) -> None:
    if report.failed_count:
        result = "FAIL"
    elif report.warning_count:
        result = "PASS (with WARN)"
    else:
        result = "PASS"

    print("Autoware localization-interface output acceptance")
    print(f"Bag:      {requested_path}")
    if resolved_uri != requested_path:
        print(f"MCAP:     {resolved_uri}")
    print(f"Profile:  {profile}")
    print(f"Read:     {audit.total_messages} messages on {len(topic_types)} topics")
    print(f"Result:   {result}")
    print()
    print("Checks:")
    for item in report.checks:
        print(f"  [{item.status:<4}] {item.name}: {item.detail}")
    print()
    print(
        "Accuracy: NOT EVALUATED - this output bag has no independent ground-truth trajectory."
    )
    print(
        f"Summary: {report.failed_count} failure(s), {report.warning_count} warning(s), "
        f"{sum(item.status == 'PASS' for item in report.checks)} passed check(s)."
    )


def make_parser() -> argparse.ArgumentParser:
    parser = ExitOneArgumentParser(
        description=(
            "Acceptance-check a recorded Autoware localization-interface "
            "output MCAP bag."
        )
    )
    parser.add_argument("bag", help="rosbag2 directory or a single .mcap file")
    parser.add_argument(
        "--profile",
        choices=("generic", "hesai-rosbag23"),
        default="generic",
        help="acceptance profile (default: generic)",
    )
    parser.add_argument(
        "--tracking-mode",
        choices=("scan_to_scan",),
        default="scan_to_scan",
        help="expected LiDAR registration mode (default: scan_to_scan)",
    )
    return parser


def run(bag_argument: str, profile: str, tracking_mode: str) -> int:
    requested_path = Path(bag_argument).expanduser().resolve()
    resolved_uri = resolve_bag_uri(bag_argument)
    reader = open_reader(resolved_uri)
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    classes = message_classes(topic_types)
    audit = read_bag(reader, classes)

    report = Report()
    check_topics(topic_types, audit, profile, report)
    if audit.deserialize_problems:
        report.failed("message deserialization", audit.deserialize_problems.describe())
    else:
        report.passed("message deserialization", "all inspected messages decoded")
    check_interface(audit, profile, report)
    if profile == "hesai-rosbag23":
        check_hesai_static_transforms(audit, report)
        check_hesai_end_recency(audit, report)
    check_adapter_diagnostic(audit, report)
    check_autoware_monitor_diagnostics(audit, report)
    check_aggregated_monitor_diagnostics(audit, report)
    check_gnss_diagnostic(audit, profile, report)
    check_deskew_diagnostic(audit, profile, report)
    check_tracking_diagnostic(audit, profile, tracking_mode, report)
    report.info(
        "localization accuracy",
        "not judged because no independent ground truth is present",
    )
    print_report(requested_path, resolved_uri, profile, topic_types, audit, report)
    return 1 if report.failed_count else 0


def main(argv: Iterable[str] | None = None) -> int:
    arguments = make_parser().parse_args(argv)
    try:
        return run(arguments.bag, arguments.profile, arguments.tracking_mode)
    except (RuntimeError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    except Exception as error:
        print(f"ERROR: failed to analyze bag: {type(error).__name__}: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
