#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Validate exact-key and real-time contracts in a recorded precision run."""

from __future__ import annotations

import argparse
import math
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np


TOPIC_SCAN = "/localization/submap_scan"
TOPIC_CORRECTION = "/localization/submap_correction"
TOPIC_RAW = "/localization/gyro_lidar_odom"
TOPIC_EXISTING = "/localization/ekf_odom"
TOPIC_LOCAL = "/localization/precision_local_odom"
TOPIC_GLOBAL = "/localization/precision_global_odom"
TOPIC_POSE = "/localization/precision_global_pose"
TOPIC_DIAGNOSTICS = "/diagnostics"
ACTIVATION_SERIALIZATION_TOLERANCE_NS = 20_000_000
FUSION_DIAGNOSTIC_STATUS = "localization/gnss_map_odom_fusion"
FUSION_HEALTH_REQUIRED_KEYS = {
    "recovery.state",
    "anchor_valid",
    "recovery.position_fused",
    "recovery.yaw_fused",
    "last_fix_state",
}
MAP_FUSION_PUBLICATION_COUNTER_KEYS = (
    "output.out_of_order_drop_count",
    "output.covered_odometry_coalesced_count",
    "output.wall_timer_coalesced_count",
    "output.total_suppressed_request_count",
)
MAP_FUSION_STRICT_DROP_KEY = "output.out_of_order_drop_count"
MAP_FUSION_COVERED_COALESCED_KEY = (
    "output.covered_odometry_coalesced_count"
)
MAP_FUSION_WALL_TIMER_COALESCED_KEY = "output.wall_timer_coalesced_count"
MAP_FUSION_TOTAL_SUPPRESSED_KEY = "output.total_suppressed_request_count"

OUTAGE_YAW_GUARD_STATES = {
    "DISARMED",
    "READY",
    "OUTAGE_SLEW",
    "OUTAGE_HOLD",
    "RECOVERY_RELEASE",
}
OUTAGE_YAW_GUARD_ACTIVE_STATES = {
    "OUTAGE_SLEW",
    "OUTAGE_HOLD",
    "RECOVERY_RELEASE",
}
OUTAGE_YAW_GUARD_OUTAGE_STATES = {"OUTAGE_SLEW", "OUTAGE_HOLD"}
OUTAGE_YAW_GUARD_REOUTAGE_FRESH_REASON = (
    "trusted_outage_yaw_reentered_during_release"
)
OUTAGE_YAW_GUARD_REOUTAGE_RETAIN_REASONS = {
    "outage_reentry_without_fresh_reference_held_last_offset",
    "outage_reentry_delta_gate_held_last_offset",
}
OUTAGE_YAW_GUARD_REOUTAGE_REASONS = {
    OUTAGE_YAW_GUARD_REOUTAGE_FRESH_REASON,
    *OUTAGE_YAW_GUARD_REOUTAGE_RETAIN_REASONS,
}
OUTAGE_YAW_GUARD_CONFIG_KEYS = (
    "outage_yaw_guard.config.max_trusted_age_sec",
    "outage_yaw_guard.config.max_trusted_variance_rad2",
    "outage_yaw_guard.config.max_trusted_delta_rad",
    "outage_yaw_guard.config.max_offset_rate_radps",
    "outage_yaw_guard.config.max_offset_step_rad",
    "outage_yaw_guard.config.max_step_dt_sec",
)
OUTAGE_YAW_GUARD_COUNTER_KEYS = (
    "outage_yaw_guard.accepted_reference_count",
    "outage_yaw_guard.rejected_reference_count",
    "outage_yaw_guard.outage_count",
    "outage_yaw_guard.recovery_count",
    "outage_yaw_guard.applied_step_count",
    "outage_yaw_guard.invalid_advance_count",
    "outage_yaw_guard.reset_count",
    "publish.global_suppressed_yaw_guard_invalid",
)
OUTAGE_YAW_GUARD_REQUIRED_KEYS = {
    "outage_yaw_guard.enabled",
    "outage_yaw_guard.state",
    "outage_yaw_guard.active",
    "outage_yaw_guard.reference_source",
    "outage_yaw_guard.propagation_source",
    "outage_yaw_guard.xy_policy",
    "outage_yaw_guard.reference_stamp_sec",
    "outage_yaw_guard.reference_age_sec",
    "outage_yaw_guard.trusted_anchor_yaw_rad",
    "outage_yaw_guard.observed_fusion_anchor_yaw_rad",
    "outage_yaw_guard.observed_delta_rad",
    "outage_yaw_guard.trusted_variance_rad2",
    "outage_yaw_guard.active_reference_variance_rad2",
    "outage_yaw_guard.nominal_global_yaw_rad",
    "outage_yaw_guard.output_global_yaw_rad",
    "outage_yaw_guard.applied_offset_rad",
    "outage_yaw_guard.target_offset_rad",
    "outage_yaw_guard.additional_variance_rad2",
    "outage_yaw_guard.last_reason",
    "global_output_ready",
    "fusion.health.healthy",
    "fusion.anchor.state",
    "local_correction.odom_session_resets",
    *OUTAGE_YAW_GUARD_CONFIG_KEYS,
    *OUTAGE_YAW_GUARD_COUNTER_KEYS,
}


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def integer_value(value: Any) -> int:
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError(f"expected one-byte integer, got {value!r}")
        return value[0]
    return int(value)


def yaw_of(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (float(quaternion.w) * float(quaternion.z)
               + float(quaternion.x) * float(quaternion.y)),
        1.0 - 2.0 * (float(quaternion.y) ** 2 + float(quaternion.z) ** 2),
    )


def wrap(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def keyed(message: Any) -> tuple[int, int, int, int]:
    return (
        int(message.odom_session_id),
        int(message.odom_generation),
        int(message.sequence),
        stamp_ns(message.header.stamp),
    )


def quaternion_values(quaternion: Any) -> tuple[float, float, float, float]:
    return (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )


def quaternion_norm(quaternion: Any) -> float:
    return math.sqrt(sum(value * value for value in quaternion_values(quaternion)))


def quaternion_multiply(
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
    dot = abs(
        sum(a * b for a, b in zip(first, second)) / (first_norm * second_norm)
    )
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def correction_composition_error(scan: Any, correction: Any) -> tuple[float, float]:
    transform = correction.precision_from_raw
    raw = scan.raw_pose.pose
    corrected = correction.corrected_pose.pose
    transform_quaternion = quaternion_values(transform.rotation)
    raw_quaternion = quaternion_values(raw.orientation)
    _, _, qz, qw = transform_quaternion
    yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    expected_position = (
        float(transform.translation.x) + cosine * float(raw.position.x)
        - sine * float(raw.position.y),
        float(transform.translation.y) + sine * float(raw.position.x)
        + cosine * float(raw.position.y),
        float(transform.translation.z) + float(raw.position.z),
    )
    position_error = math.sqrt(
        (float(corrected.position.x) - expected_position[0]) ** 2
        + (float(corrected.position.y) - expected_position[1]) ** 2
        + (float(corrected.position.z) - expected_position[2]) ** 2
    )
    expected_quaternion = quaternion_multiply(transform_quaternion, raw_quaternion)
    orientation_error = quaternion_angle(
        expected_quaternion, quaternion_values(corrected.orientation)
    )
    return position_error, orientation_error


def percentile(values: Iterable[float], quantile: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    index = max(0, min(len(ordered) - 1, math.ceil(quantile * len(ordered)) - 1))
    return ordered[index]


def timer_stamp_order(stamps: list[int]) -> tuple[bool, int]:
    """Allow only a leading /clock==0 prefix, then positive nondecreasing time."""
    first_positive = next(
        (index for index, value in enumerate(stamps) if value > 0), None
    )
    leading_zero_count = first_positive if first_positive is not None else len(stamps)
    valid = (
        first_positive is not None
        and all(value == 0 for value in stamps[:first_positive])
        and all(value > 0 for value in stamps[first_positive:])
        and all(
            later >= earlier
            for earlier, later in zip(
                stamps[first_positive:], stamps[first_positive + 1 :]
            )
        )
    )
    return valid, leading_zero_count


def activation_raw_successor(
    raw_stamps: Iterable[int], activation_ns: int, tolerance_ns: int
) -> tuple[int | None, int | None]:
    """Resolve the serialized activation sample and its next unique raw stamp."""
    ordered = sorted(set(int(value) for value in raw_stamps if int(value) > 0))
    candidates = [
        (abs(value - activation_ns), value)
        for value in ordered if abs(value - activation_ns) <= tolerance_ns
    ]
    if not candidates:
        return None, None
    _, activation_raw = min(candidates)
    index = ordered.index(activation_raw)
    next_raw = ordered[index + 1] if index + 1 < len(ordered) else None
    return activation_raw, next_raw


def exact_key_stream_order(keys: list[tuple[int, int, int, int]]) -> bool:
    """Validate an opaque session, monotonic generation/sequence/stamp stream."""
    if not keys:
        return False
    current_session: int | None = None
    current_generation: int | None = None
    closed_sessions: set[int] = set()
    previous: tuple[int, int, int, int] | None = None
    for key in keys:
        session, generation, sequence, physical_stamp = key
        if min(session, generation, sequence, physical_stamp) <= 0:
            return False
        if current_session is None:
            current_session = session
            current_generation = generation
        elif session != current_session:
            closed_sessions.add(current_session)
            if session in closed_sessions:
                return False
            current_session = session
            current_generation = generation
        elif generation != current_generation:
            if current_generation is None or generation <= current_generation:
                return False
            current_generation = generation
        if previous is not None:
            if physical_stamp <= previous[3]:
                return False
            if session == previous[0] and generation == previous[1]:
                if sequence <= previous[2]:
                    return False
        previous = key
    return True


def import_rosbag_modules():
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python environment is required; source /opt/ros and this workspace"
        ) from error
    return rosbag2_py, deserialize_message, get_message


def open_reader(uri: Path):
    rosbag2_py, _, _ = import_rosbag_modules()
    uri = uri.expanduser().resolve()
    if uri.is_dir() and not (uri / "metadata.yaml").is_file():
        nested = uri / "localization_output"
        if (nested / "metadata.yaml").is_file():
            uri = nested
    if not uri.exists():
        raise RuntimeError(f"bag does not exist: {uri}")
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(uri), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)
    return reader


def diagnostic_values(message: Any, status_name: str) -> dict[str, str] | None:
    for status in message.status:
        if status.name == status_name or status.name.endswith("/" + status_name):
            return {item.key: item.value for item in status.values}
    return None


def diagnostic_status(message: Any, status_name: str) -> Any | None:
    for status in message.status:
        if status.name == status_name or status.name.endswith("/" + status_name):
            return status
    return None


def existing_global_contract_valid(message: Any) -> bool:
    pose = message.pose.pose
    quaternion = pose.orientation
    quaternion_components = np.asarray(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=float
    )
    covariance = np.asarray(message.pose.covariance, dtype=float)
    if (
        stamp_ns(message.header.stamp) <= 0
        or message.header.frame_id != "map"
        or message.child_frame_id != "base_link"
        or not np.all(np.isfinite(quaternion_components))
        or not math.isfinite(float(pose.position.x))
        or not math.isfinite(float(pose.position.y))
        or not math.isfinite(float(pose.position.z))
        or not np.all(np.isfinite(covariance))
    ):
        return False
    quaternion_norm_value = float(np.linalg.norm(quaternion_components))
    if quaternion_norm_value <= 1.0e-6 or abs(quaternion_norm_value - 1.0) > 1.0e-3:
        return False
    projected = covariance[[0, 1, 5, 6, 7, 11, 30, 31, 35]].reshape(3, 3)
    if float(np.linalg.norm(projected - projected.T)) > 1.0e-6:
        return False
    return float(np.min(np.linalg.eigvalsh(projected))) >= -1.0e-9


def existing_global_prefix_accounting(
    records: list[tuple[int, Any]],
) -> dict[str, Any]:
    """Replay the node's strict G callback accounting on a causal bag prefix."""
    accepted = 0
    rejected = 0
    duplicate = 0
    previous_stamp: int | None = None
    errors: list[str] = []
    for _, message in records:
        stamp = stamp_ns(message.header.stamp)
        if not existing_global_contract_valid(message):
            rejected += 1
            errors.append(f"invalid contract at stamp {stamp}")
            continue
        if previous_stamp is not None and stamp < previous_stamp:
            rejected += 1
            errors.append(f"stamp backstep {previous_stamp}->{stamp}")
            continue
        if previous_stamp is not None and stamp == previous_stamp:
            duplicate += 1
            continue
        accepted += 1
        previous_stamp = stamp
    return {
        "valid": not errors,
        "received": len(records),
        "accepted": accepted,
        "rejected": rejected,
        "duplicate": duplicate,
        "errors": errors[:20],
    }


def fusion_health_prefix_accounting(
    records: list[dict[str, Any]],
) -> dict[str, Any]:
    """Replay strict fusion-status receipt through the final precision diagnostic."""
    received = 0
    accepted = 0
    rejected = 0
    zero_stamp = 0
    duplicate_stamp = 0
    snapshot_received = False
    snapshot_stamp: int | None = None
    errors: list[str] = []
    for item in records:
        status_count = int(item["status_count"])
        if status_count != 1:
            rejected += 1
            snapshot_received = False
            snapshot_stamp = None
            errors.append(
                f"expected one fusion status at record {item['record_ns']}, "
                f"got {status_count}"
            )
            continue
        received += 1
        stamp = int(item["stamp_ns"])
        pairs = list(item["values"])
        keys = [key for key, _ in pairs]
        duplicate_keys = len(keys) != len(set(keys))
        missing_keys = sorted(FUSION_HEALTH_REQUIRED_KEYS - set(keys))
        if duplicate_keys or missing_keys:
            errors.append(
                f"malformed fusion health stamp={stamp} duplicate_keys="
                f"{duplicate_keys} missing={missing_keys}"
            )
        if stamp <= 0:
            rejected += 1
            zero_stamp += 1
            snapshot_received = False
            snapshot_stamp = None
            continue
        if snapshot_received and snapshot_stamp is not None and stamp <= snapshot_stamp:
            rejected += 1
            duplicate_stamp += int(stamp == snapshot_stamp)
            if stamp < snapshot_stamp:
                errors.append(f"fusion diagnostic backstep {snapshot_stamp}->{stamp}")
            continue
        if duplicate_keys or missing_keys:
            rejected += 1
            snapshot_received = True
            snapshot_stamp = stamp
            continue
        accepted += 1
        snapshot_received = True
        snapshot_stamp = stamp
    return {
        "valid": not errors,
        "recorded_statuses": len(records),
        "received": received,
        "accepted": accepted,
        "rejected": rejected,
        "zero_stamp": zero_stamp,
        "duplicate_stamp": duplicate_stamp,
        "errors": errors[:20],
    }


def _strict_nonnegative_diagnostic_integer(value: Any) -> int:
    """Parse the canonical decimal form emitted by std::to_string."""
    if not isinstance(value, str):
        raise ValueError(f"expected a string, got {type(value).__name__}")
    if value == "0":
        return 0
    if (
        not value
        or value[0] not in "123456789"
        or any(character not in "0123456789" for character in value[1:])
    ):
        raise ValueError(f"not a canonical non-negative integer: {value!r}")
    return int(value)


def map_fusion_publication_integrity_contract(
    timeline: list[dict[str, Any]],
) -> tuple[bool, dict[str, Any], list[str]]:
    """Validate strict-drop and harmless-coalescing diagnostic accounting.

    Coalesced requests are intentionally report-only: a suppressed request is
    not present in the output bag and therefore cannot be reconstructed from
    output topic counts.  The diagnostic sum identity is the authoritative
    accounting contract for those requests.
    """
    errors: list[str] = []
    previous: dict[str, int] | None = None
    final_counters: dict[str, int] | None = None
    schema_samples = 0

    for index, item in enumerate(timeline):
        if item.get("status_count") != 1:
            errors.append(
                f"map-fusion sample {index} at record {item.get('record_ns')} "
                f"has status_count={item.get('status_count')!r}"
            )

        raw_values = item.get("values", [])
        try:
            pairs = (
                list(raw_values.items())
                if isinstance(raw_values, dict)
                else list(raw_values)
            )
            encoded_by_key = {
                key: [pair[1] for pair in pairs if str(pair[0]) == key]
                for key in MAP_FUSION_PUBLICATION_COUNTER_KEYS
            }
        except (TypeError, ValueError, IndexError) as error:
            errors.append(f"map-fusion sample {index} has malformed values: {error}")
            continue

        missing = [
            key for key in MAP_FUSION_PUBLICATION_COUNTER_KEYS
            if not encoded_by_key[key]
        ]
        duplicate = [
            key for key in MAP_FUSION_PUBLICATION_COUNTER_KEYS
            if len(encoded_by_key[key]) > 1
        ]
        if missing or duplicate:
            errors.append(
                f"map-fusion sample {index} at stamp {item.get('stamp_ns')} "
                f"missing={missing} duplicate={duplicate}"
            )
            continue

        try:
            counters = {
                key: _strict_nonnegative_diagnostic_integer(encoded_by_key[key][0])
                for key in MAP_FUSION_PUBLICATION_COUNTER_KEYS
            }
        except ValueError as error:
            errors.append(
                f"map-fusion sample {index} at stamp {item.get('stamp_ns')} "
                f"has malformed counter: {error}"
            )
            continue

        schema_samples += 1
        strict_drop = counters[MAP_FUSION_STRICT_DROP_KEY]
        covered = counters[MAP_FUSION_COVERED_COALESCED_KEY]
        wall_timer = counters[MAP_FUSION_WALL_TIMER_COALESCED_KEY]
        total = counters[MAP_FUSION_TOTAL_SUPPRESSED_KEY]
        if total != strict_drop + covered + wall_timer:
            errors.append(
                f"map-fusion sample {index} suppression sum mismatch: "
                f"total={total} strict={strict_drop} covered={covered} "
                f"wall_timer={wall_timer}"
            )
        if strict_drop != 0:
            errors.append(
                f"map-fusion sample {index} recorded strict out-of-order drops: "
                f"{strict_drop}"
            )
        if previous is not None:
            for key in MAP_FUSION_PUBLICATION_COUNTER_KEYS:
                if counters[key] < previous[key]:
                    errors.append(
                        f"map-fusion counter {key} backstep at sample {index}: "
                        f"{previous[key]}->{counters[key]}"
                    )
        previous = counters
        final_counters = counters

    summary = {
        "samples": len(timeline),
        "schema_samples": schema_samples,
        "final_counters": final_counters or {},
        "strict_drop_count": (
            final_counters.get(MAP_FUSION_STRICT_DROP_KEY)
            if final_counters is not None else None
        ),
        "coalesced_report_only": {
            "covered_odometry": (
                final_counters.get(MAP_FUSION_COVERED_COALESCED_KEY)
                if final_counters is not None else None
            ),
            "wall_timer": (
                final_counters.get(MAP_FUSION_WALL_TIMER_COALESCED_KEY)
                if final_counters is not None else None
            ),
        },
    }
    if not timeline:
        errors.append("map-fusion diagnostic timeline is empty")
    return bool(timeline) and not errors, summary, errors[:20]


def exact_raw_to_existing_stamp_coverage(
    raw_records: list[tuple[int, Any]],
    existing_records: list[tuple[int, Any]],
    final_diagnostic_record_ns: int | None,
) -> tuple[bool, dict[str, Any], list[str]]:
    """Require exact RAW stamp coverage over a causally closed bag prefix.

    Raw messages before the first positive existing-fusion output are startup
    inputs.  Messages recorded after the final map-fusion diagnostic, or with
    stamps beyond the last output in that prefix, are an open recorder tail.
    Neither region has enough causal evidence for a publication assertion.
    """
    errors: list[str] = []

    def serialize_records(
        records: list[tuple[int, Any]], label: str
    ) -> list[tuple[int, int]]:
        serialized: list[tuple[int, int]] = []
        previous_record_ns: int | None = None
        for index, (record_ns_value, message) in enumerate(records):
            try:
                record_ns = int(record_ns_value)
                physical_stamp = stamp_ns(message.header.stamp)
            except (AttributeError, TypeError, ValueError) as error:
                errors.append(f"{label} record {index} is malformed: {error}")
                continue
            if previous_record_ns is not None and record_ns < previous_record_ns:
                errors.append(
                    f"{label} bag record time backstep at record {index}: "
                    f"{previous_record_ns}->{record_ns}"
                )
            previous_record_ns = record_ns
            serialized.append((record_ns, physical_stamp))
        return serialized

    raw = serialize_records(raw_records, "raw")
    existing = serialize_records(existing_records, "existing")
    try:
        final_record_ns = int(final_diagnostic_record_ns)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        final_record_ns = -1
    if final_record_ns < 0:
        errors.append("final map-fusion diagnostic record time is unavailable")

    existing_prefix = [
        item for item in existing
        if item[0] <= final_record_ns
    ]
    existing_prefix_stamps = [stamp for _, stamp in existing_prefix]
    existing_order_valid, existing_leading_zero = timer_stamp_order(
        existing_prefix_stamps
    )
    if not existing_order_valid:
        errors.append(
            "existing-fusion prefix has no positive nondecreasing stamp stream"
        )
    existing_positive_stamps = [
        stamp for stamp in existing_prefix_stamps if stamp > 0
    ]

    first_existing_stamp: int | None = None
    last_existing_stamp: int | None = None
    eligible_raw_stamps: list[int] = []
    startup_excluded = 0
    stamp_tail_excluded = 0
    if existing_positive_stamps:
        first_existing_stamp = existing_positive_stamps[0]
        last_existing_stamp = existing_positive_stamps[-1]
        raw_prefix_positive = [
            stamp for record_ns, stamp in raw
            if record_ns <= final_record_ns and stamp > 0
        ]
        startup_excluded = sum(
            stamp < first_existing_stamp for stamp in raw_prefix_positive
        )
        stamp_tail_excluded = sum(
            stamp > last_existing_stamp for stamp in raw_prefix_positive
        )
        eligible_raw_stamps = [
            stamp for stamp in raw_prefix_positive
            if first_existing_stamp <= stamp <= last_existing_stamp
        ]
    else:
        errors.append("no positive existing-fusion output in causal prefix")

    eligible_unique = sorted(set(eligible_raw_stamps))
    existing_unique = set(existing_positive_stamps)
    missing = [stamp for stamp in eligible_unique if stamp not in existing_unique]
    if not eligible_unique:
        errors.append("no eligible positive raw stamps in causal prefix")
    if missing:
        errors.append(
            f"{len(missing)} eligible raw stamps lack an exact existing-fusion "
            f"output; first={missing[:10]}"
        )

    raw_zero_excluded = sum(stamp <= 0 for _, stamp in raw)
    raw_record_tail_excluded = sum(
        record_ns > final_record_ns and stamp > 0 for record_ns, stamp in raw
    )
    existing_record_tail_excluded = sum(
        record_ns > final_record_ns and stamp > 0
        for record_ns, stamp in existing
    )
    matched = len(eligible_unique) - len(missing)
    summary = {
        "final_diagnostic_record_ns": (
            final_record_ns if final_record_ns >= 0 else None
        ),
        "first_existing_stamp_ns": first_existing_stamp,
        "last_existing_stamp_ns": last_existing_stamp,
        "existing_prefix_positive_records": len(existing_positive_stamps),
        "existing_prefix_unique_stamps": len(existing_unique),
        "existing_leading_zero_records": existing_leading_zero,
        "eligible_raw_records": len(eligible_raw_stamps),
        "eligible_unique_raw_stamps": len(eligible_unique),
        "duplicate_eligible_raw_records": (
            len(eligible_raw_stamps) - len(eligible_unique)
        ),
        "matched_unique_raw_stamps": matched,
        "missing_unique_raw_stamps": len(missing),
        "missing_examples_ns": missing[:20],
        "coverage_ratio": (
            matched / len(eligible_unique) if eligible_unique else 0.0
        ),
        "startup_excluded_raw_records": startup_excluded,
        "zero_stamp_excluded_raw_records": raw_zero_excluded,
        "stamp_tail_excluded_raw_records": stamp_tail_excluded,
        "record_tail_excluded_raw_records": raw_record_tail_excluded,
        "record_tail_excluded_existing_records": existing_record_tail_excluded,
    }
    return not errors, summary, errors[:20]


@dataclass
class Check:
    name: str
    passed: bool
    detail: str
    warning: bool = False


def startup_transition_contract(
    timeline: list[dict[str, Any]],
) -> tuple[bool, list[dict[str, Any]], list[str]]:
    """Require one internally exact false->true readiness transition per epoch."""
    errors: list[str] = []
    by_epoch: dict[int, list[dict[str, Any]]] = defaultdict(list)
    transitions: list[dict[str, Any]] = []
    for item in timeline:
        values = item["values"]
        try:
            epoch = int(values["activation.epoch"])
        except (KeyError, ValueError):
            continue
        position = values.get("position_initialized", "false") == "true"
        position_fused = values.get("position_fused", "false") == "true"
        yaw = values.get("yaw_publishable", "false") == "true"
        anchor_initialized = values.get("anchor.initialized", "false") == "true"
        anchor_yaw_observed = values.get("anchor.yaw_observed", "false") == "true"
        anchor_yaw = values.get("anchor.yaw_publishable", "false") == "true"
        ready = values.get("global_output_ready", "false") == "true"
        state = values.get("state", "")
        if not (
            ready == position == position_fused == yaw == anchor_initialized
            == anchor_yaw_observed == anchor_yaw
        ):
            errors.append(f"logical mismatch at {item['stamp_ns']}")
        if not ready:
            if item["level"] != 1:
                errors.append(f"pre-ready status is not WARN at {item['stamp_ns']}")
            if state in ("TRACKING_SE2", "TRACKING"):
                errors.append(f"tracking state before readiness at {item['stamp_ns']}")
            if item["message"] not in (
                "waiting_for_first_usable_gnss_position",
                "waiting_for_stable_absolute_yaw",
                "gnss_outage_before_yaw_activation",
                "waiting_for_healthy_existing_fusion",
                "stabilizing_existing_fusion_startup",
            ):
                errors.append(f"unexpected pre-ready reason at {item['stamp_ns']}")
        if ready and state not in (
            "TRACKING_SE2",
            "HOLD_SOFT_GAP",
            "OUTAGE",
            "TRACKING",
            "FROZEN",
            "STABILIZING_RECOVERY",
        ):
            errors.append(f"unexpected ready state {state!r} at {item['stamp_ns']}")
        by_epoch[epoch].append(item)

    previous_epoch: int | None = None
    for epoch, items in sorted(by_epoch.items()):
        if previous_epoch is not None and epoch != previous_epoch + 1:
            errors.append(f"epoch jump {previous_epoch}->{epoch}")
        previous_epoch = epoch
        ready_values = [
            item["values"].get("global_output_ready", "false") == "true"
            for item in items
        ]
        if not ready_values or ready_values[0]:
            errors.append(f"epoch {epoch} has no observed pre-ready diagnostic")
            continue
        changes = [
            index for index in range(1, len(ready_values))
            if ready_values[index] != ready_values[index - 1]
        ]
        if len(changes) != 1 or not ready_values[-1]:
            errors.append(f"epoch {epoch} readiness changes={changes}")
            continue
        index = changes[0]
        values = items[index]["values"]
        previous_values = items[index - 1]["values"]
        try:
            stable = int(values["activation.stable_candidate_count"])
            required = int(values["activation.required_candidate_count"])
            commit = int(values["activation.commit_count"])
            previous_commit = int(previous_values["activation.commit_count"])
            activation_sec = float(values["activation.stamp_sec"])
            candidate_yaw = float(values["activation.candidate_yaw_rad"])
            candidate_delta = float(values["activation.candidate_delta_rad"])
            health_age = float(values["fusion.health.age_sec"])
            existing_global_stamp = float(
                values["fusion.sync.existing_global_stamp_sec"]
            )
            existing_global_age = float(
                values["fusion.sync.existing_global_age_sec"]
            )
            last_valid_stamp = float(values["fusion.sync.last_valid_stamp_sec"])
            last_valid_age = float(values["fusion.sync.last_valid_age_sec"])
        except (KeyError, ValueError):
            errors.append(f"epoch {epoch} activation fields malformed")
            continue
        if (
            required != 3 or stable < required or commit != previous_commit + 1
            or values.get("activation.reason") not in (
                "stable_yaw_activated",
                "existing_fusion_stable_activated",
            )
            or not math.isfinite(activation_sec) or activation_sec <= 0.0
            or not math.isfinite(candidate_yaw)
            or not math.isfinite(candidate_delta)
            or not 0.0 <= candidate_delta <= 0.08
        ):
            errors.append(
                f"epoch {epoch} invalid activation stable={stable}/{required} "
                f"commit={previous_commit}->{commit} reason="
                f"{values.get('activation.reason')!r} delta={candidate_delta}"
            )
        strict_health = (
            values.get("anchor.source") == "existing_fusion"
            and values.get("fallback.gnss_position_enabled") == "false"
            and values.get("fusion.health.healthy") == "true"
            and values.get("fusion.health.level") == "0"
            and values.get("fusion.health.recovery_state") == "tracking"
            and values.get("fusion.health.anchor_valid") == "true"
            and values.get("fusion.health.position_fused") == "true"
            and values.get("fusion.health.yaw_fused") == "true"
            and values.get("fusion.health.last_fix_state") == "good"
            and values.get("fusion.health.reason") == "strict_fusion_health_ok"
            and math.isfinite(health_age)
            and -0.25 <= health_age <= 1.5
            and math.isfinite(existing_global_stamp)
            and existing_global_stamp > 0.0
            and math.isfinite(existing_global_age)
            and -0.05 <= existing_global_age <= 0.25
            and math.isfinite(last_valid_stamp)
            and last_valid_stamp > 0.0
            and math.isfinite(last_valid_age)
            and -0.05 <= last_valid_age <= 0.50
            and values.get("fusion.health.rearm_required") == "false"
        )
        if not strict_health:
            errors.append(
                f"epoch {epoch} activation lacks strict existing-fusion authority "
                f"health_age={health_age} existing_age={existing_global_age} "
                f"last_valid_age={last_valid_age}"
            )
        if len({item["values"].get("publish.global") for item in items[:index]}) != 1:
            errors.append(f"epoch {epoch} global counter changed before readiness")
        transitions.append(
            {
                "epoch": epoch,
                "activation_ns": int(round(activation_sec * 1.0e9)),
                "candidate_yaw_rad": candidate_yaw,
                "anchor_lag_yaw_rad": abs(float(
                    values.get("anchor.correction_lag.yaw_rad", "nan")
                )),
                "diagnostic_stamp_ns": items[index]["stamp_ns"],
            }
        )
    return bool(by_epoch) and not errors, transitions, errors[:20]


def fusion_anchor_freeze_contract(
    timeline: list[dict[str, Any]],
) -> tuple[bool, list[dict[str, Any]], list[str]]:
    """Require target/applied identity and exact hold whenever authority is unhealthy."""
    errors: list[str] = []
    groups: list[list[dict[str, Any]]] = []
    active: list[dict[str, Any]] = []
    keys = (
        "anchor.target.x_m",
        "anchor.target.y_m",
        "anchor.target.yaw_rad",
        "anchor.applied.x_m",
        "anchor.applied.y_m",
        "anchor.applied.yaw_rad",
    )
    for item in timeline:
        values = item["values"]
        ready = values.get("global_output_ready", "false") == "true"
        healthy = values.get("fusion.health.healthy", "false") == "true"
        if ready and not healthy:
            active.append(item)
        elif active:
            groups.append(active)
            active = []
    if active:
        groups.append(active)

    summaries: list[dict[str, Any]] = []
    for group in groups:
        serialized = {
            key: [item["values"].get(key) for item in group] for key in keys
        }
        complete = all(
            all(value is not None for value in values)
            for values in serialized.values()
        )
        target_applied_equal = complete and all(
            item["values"].get(f"anchor.target.{axis}")
            == item["values"].get(f"anchor.applied.{axis}")
            for item in group for axis in ("x_m", "y_m", "yaw_rad")
        )
        exact_hold = complete and all(
            len(set(values)) == 1 for values in serialized.values()
        )
        valid = complete and target_applied_equal and exact_hold
        if not valid:
            errors.append(
                f"unhealthy group {group[0]['stamp_ns']}..{group[-1]['stamp_ns']} "
                f"complete={complete} equal={target_applied_equal} hold={exact_hold}"
            )
        summaries.append(
            {
                "begin_ns": group[0]["stamp_ns"],
                "end_ns": group[-1]["stamp_ns"],
                "samples": len(group),
                "target_applied_equal": target_applied_equal,
                "serialization_exact": exact_hold,
            }
        )
    # A short smoke bag may never enter a post-activation outage. In that case
    # the startup suppression checks remain authoritative and this is vacuous.
    return not errors, summaries, errors[:20]


def outage_yaw_guard_contract(
    timeline: list[dict[str, Any]],
) -> tuple[bool, dict[str, Any], list[str]]:
    """Validate the orientation-only outage guard from runtime diagnostics.

    Diagnostic sampling is slower than raw odometry, so a pair of snapshots
    can span several bounded guard steps.  The net applied-offset change is
    therefore checked against both the reported step-count increase and a
    rate envelope with one configured dt-cap of phase allowance.  Existing
    fusion anchor target/applied immutability remains a separate contract.
    """
    errors: list[str] = []
    parsed: list[dict[str, Any]] = []
    reference_config: tuple[float, ...] | None = None

    def parse_finite(values: dict[str, str], key: str) -> float:
        value = float(values[key])
        if not math.isfinite(value):
            raise ValueError(f"{key} is not finite")
        return value

    def parse_counter(values: dict[str, str], key: str) -> int:
        encoded = values[key]
        value = int(encoded)
        if str(value) != encoded.strip() or value < 0:
            raise ValueError(f"{key} is not a non-negative integer")
        return value

    for index, item in enumerate(timeline):
        values = item.get("values", {})
        missing = sorted(OUTAGE_YAW_GUARD_REQUIRED_KEYS - set(values))
        if missing:
            errors.append(
                f"guard sample {index} at {item.get('stamp_ns')} missing={missing}"
            )
            continue
        active_variance_key = (
            "outage_yaw_guard.active_reference_variance_rad2"
        )
        key_counts = item.get("key_counts")
        for exact_key in (
            active_variance_key,
            "outage_yaw_guard.last_reason",
        ):
            if not isinstance(key_counts, dict) or key_counts.get(exact_key) != 1:
                errors.append(
                    f"guard sample {index} at {item.get('stamp_ns')} requires "
                    f"exactly one {exact_key}; count="
                    f"{key_counts.get(exact_key) if isinstance(key_counts, dict) else None}"
                )
        try:
            stamp = int(item["stamp_ns"])
            enabled_text = values["outage_yaw_guard.enabled"]
            active_text = values["outage_yaw_guard.active"]
            if enabled_text not in {"true", "false"}:
                raise ValueError("outage_yaw_guard.enabled is not boolean")
            if active_text not in {"true", "false"}:
                raise ValueError("outage_yaw_guard.active is not boolean")
            enabled = enabled_text == "true"
            active = active_text == "true"
            state = values["outage_yaw_guard.state"]
            last_reason = values["outage_yaw_guard.last_reason"]
            config = tuple(
                parse_finite(values, key) for key in OUTAGE_YAW_GUARD_CONFIG_KEYS
            )
            counters = {
                key: parse_counter(values, key)
                for key in OUTAGE_YAW_GUARD_COUNTER_KEYS
            }
            odom_session_resets = parse_counter(
                values, "local_correction.odom_session_resets"
            )
            applied_offset = parse_finite(
                values, "outage_yaw_guard.applied_offset_rad"
            )
            target_offset = parse_finite(
                values, "outage_yaw_guard.target_offset_rad"
            )
            additional_variance = parse_finite(
                values, "outage_yaw_guard.additional_variance_rad2"
            )
            active_reference_variance = float(values[active_variance_key])
            if math.isinf(active_reference_variance):
                raise ValueError(f"{active_variance_key} is infinite")
            reference_values = tuple(
                float(values[key])
                for key in (
                    "outage_yaw_guard.reference_stamp_sec",
                    "outage_yaw_guard.reference_age_sec",
                    "outage_yaw_guard.trusted_anchor_yaw_rad",
                    "outage_yaw_guard.observed_fusion_anchor_yaw_rad",
                    "outage_yaw_guard.observed_delta_rad",
                    "outage_yaw_guard.trusted_variance_rad2",
                )
            )
            nominal_yaw = float(values["outage_yaw_guard.nominal_global_yaw_rad"])
            output_yaw = float(values["outage_yaw_guard.output_global_yaw_rad"])
        except (KeyError, TypeError, ValueError) as error:
            errors.append(
                f"malformed guard sample {index} at {item.get('stamp_ns')}: {error}"
            )
            continue

        if not enabled:
            errors.append(f"guard sample {index} reports disabled runtime policy")
        if state not in OUTAGE_YAW_GUARD_STATES:
            errors.append(f"guard sample {index} has unknown state {state!r}")
        expected_active = state in OUTAGE_YAW_GUARD_ACTIVE_STATES
        if active != expected_active:
            errors.append(
                f"guard sample {index} active={active} disagrees with state={state}"
            )
        if values["outage_yaw_guard.reference_source"] != (
            "robust_gnss_position_alignment_yaw"
        ):
            errors.append(f"guard sample {index} has wrong reference source")
        if values["outage_yaw_guard.propagation_source"] != "precision_local_yaw":
            errors.append(f"guard sample {index} has wrong propagation source")
        if values["outage_yaw_guard.xy_policy"] != (
            "existing_fusion_anchor_compose_precision_local"
        ):
            errors.append(f"guard sample {index} has wrong XY policy")
        if not last_reason:
            errors.append(f"guard sample {index} has empty last reason")

        (
            max_trusted_age,
            max_trusted_variance,
            max_trusted_delta,
            max_offset_rate,
            max_offset_step,
            max_step_dt,
        ) = config
        valid_config = (
            max_trusted_age > 0.0
            and max_trusted_variance > 0.0
            and 0.0 < max_trusted_delta <= math.pi
            and max_offset_rate > 0.0
            and 0.0 < max_offset_step <= math.pi
            and max_step_dt > 0.0
        )
        if not valid_config:
            errors.append(f"guard sample {index} has invalid config={config}")
        if reference_config is None:
            reference_config = config
        elif config != reference_config:
            errors.append(
                f"guard sample {index} changed config {reference_config}->{config}"
            )

        authority_tracking = (
            values["fusion.health.healthy"] == "true"
            and values["fusion.anchor.state"] == "TRACKING"
        )
        if values["fusion.health.healthy"] not in {"true", "false"}:
            errors.append(f"guard sample {index} has malformed fusion health")
        if state in OUTAGE_YAW_GUARD_OUTAGE_STATES and authority_tracking:
            errors.append(
                f"guard sample {index} has outage state under tracking authority"
            )
        if state in OUTAGE_YAW_GUARD_OUTAGE_STATES and values[
            "fusion.anchor.state"
        ] not in {"FROZEN", "STABILIZING_RECOVERY"}:
            errors.append(
                f"guard sample {index} has outage state with fusion anchor "
                f"{values['fusion.anchor.state']!r}"
            )
        if state == "RECOVERY_RELEASE" and not authority_tracking:
            errors.append(
                f"guard sample {index} releases without tracking authority"
            )
        if active and values["global_output_ready"] != "true":
            errors.append(f"guard sample {index} is active before global readiness")
        if values["global_output_ready"] not in {"true", "false"}:
            errors.append(f"guard sample {index} has malformed readiness")

        if counters["outage_yaw_guard.invalid_advance_count"] != 0:
            errors.append(
                f"guard sample {index} recorded an invalid advance: "
                f"{counters['outage_yaw_guard.invalid_advance_count']}"
            )
        if counters["publish.global_suppressed_yaw_guard_invalid"] != 0:
            errors.append(
                f"guard sample {index} suppressed invalid global yaw output: "
                f"{counters['publish.global_suppressed_yaw_guard_invalid']}"
            )
        if counters["outage_yaw_guard.recovery_count"] > counters[
            "outage_yaw_guard.outage_count"
        ]:
            errors.append(f"guard sample {index} has recovery count above outage count")
        if state in OUTAGE_YAW_GUARD_OUTAGE_STATES and counters[
            "outage_yaw_guard.outage_count"
        ] <= counters["outage_yaw_guard.recovery_count"]:
            errors.append(f"guard sample {index} has no open outage edge")
        if state == "RECOVERY_RELEASE" and counters[
            "outage_yaw_guard.outage_count"
        ] != counters["outage_yaw_guard.recovery_count"]:
            errors.append(f"guard sample {index} recovery edge accounting differs")
        if counters["outage_yaw_guard.reset_count"] != odom_session_resets + 1:
            errors.append(
                f"guard sample {index} reset/session mismatch reset="
                f"{counters['outage_yaw_guard.reset_count']} odom={odom_session_resets}"
            )

        if abs(applied_offset) > math.pi + 1.0e-12:
            errors.append(f"guard sample {index} applied offset is not wrapped")
        if abs(target_offset) > math.pi + 1.0e-12:
            errors.append(f"guard sample {index} target offset is not wrapped")
        if abs(applied_offset) > max_trusted_delta + 1.0e-12:
            errors.append(f"guard sample {index} applied offset exceeds configured gate")
        if abs(target_offset) > max_trusted_delta + 1.0e-12:
            errors.append(f"guard sample {index} target offset exceeds configured gate")
        if additional_variance < 0.0:
            errors.append(f"guard sample {index} has negative added variance")
        if state in OUTAGE_YAW_GUARD_ACTIVE_STATES:
            if (
                not math.isfinite(active_reference_variance)
                or active_reference_variance < 0.0
                or active_reference_variance > max_trusted_variance + 1.0e-12
            ):
                errors.append(
                    f"guard sample {index} active reference variance is invalid: "
                    f"{active_reference_variance}"
                )
        elif not math.isnan(active_reference_variance):
            errors.append(
                f"guard sample {index} {state} active reference variance "
                f"is not cleared: {active_reference_variance}"
            )
        if state in {"DISARMED", "READY"} and (
            abs(applied_offset) > 1.0e-12
            or abs(target_offset) > 1.0e-12
            or abs(additional_variance) > 1.0e-12
        ):
            errors.append(
                f"guard sample {index} non-active state carries correction "
                f"applied={applied_offset} target={target_offset} "
                f"variance={additional_variance}"
            )
        if state == "OUTAGE_HOLD" and abs(wrap(target_offset - applied_offset)) > 1.0e-12:
            errors.append(f"guard sample {index} outage hold has residual offset")
        if state == "RECOVERY_RELEASE" and abs(target_offset) > 1.0e-12:
            errors.append(f"guard sample {index} recovery target is not zero")

        reference_finite = all(math.isfinite(value) for value in reference_values)
        reference_nan = all(math.isnan(value) for value in reference_values)
        if state == "DISARMED":
            if not reference_nan:
                errors.append(f"guard sample {index} disarmed reference is not cleared")
        elif not reference_finite:
            errors.append(f"guard sample {index} armed reference is incomplete")
        if reference_finite:
            (
                reference_stamp,
                reference_age,
                trusted_yaw,
                observed_yaw,
                observed_delta,
                trusted_variance,
            ) = reference_values
            if reference_stamp <= 0.0 or reference_age < 0.0:
                errors.append(f"guard sample {index} has invalid reference time")
            if state == "READY" and reference_age > max_trusted_age + 1.0e-9:
                errors.append(f"guard sample {index} READY reference is stale")
            if not 0.0 <= trusted_variance <= max_trusted_variance:
                errors.append(f"guard sample {index} has invalid trusted variance")
            if abs(wrap(trusted_yaw - observed_yaw - observed_delta)) > 1.0e-9:
                errors.append(f"guard sample {index} has inconsistent observed delta")
            if abs(observed_delta) > max_trusted_delta + 1.0e-12:
                errors.append(f"guard sample {index} exceeds trusted delta gate")
            if state in OUTAGE_YAW_GUARD_OUTAGE_STATES:
                expected_variance = active_reference_variance + wrap(
                    target_offset - applied_offset
                ) ** 2
                if abs(additional_variance - expected_variance) > 1.0e-9:
                    errors.append(
                        f"guard sample {index} outage variance mismatch "
                        f"actual={additional_variance} expected={expected_variance}"
                    )
        if state == "RECOVERY_RELEASE":
            expected_variance = (
                active_reference_variance + applied_offset * applied_offset
            )
            if abs(additional_variance - expected_variance) > 1.0e-9:
                errors.append(
                    f"guard sample {index} release variance mismatch "
                    f"actual={additional_variance} expected={expected_variance}"
                )

        if math.isfinite(nominal_yaw) != math.isfinite(output_yaw):
            errors.append(f"guard sample {index} has incomplete published yaw pair")
        if active and not (
            math.isfinite(nominal_yaw) and math.isfinite(output_yaw)
        ):
            errors.append(f"guard sample {index} active yaw output is unavailable")
        if math.isfinite(nominal_yaw) and (
            abs(wrap(output_yaw - nominal_yaw - applied_offset)) > 1.0e-9
        ):
            errors.append(f"guard sample {index} violates yaw offset composition")

        current = {
            "index": index,
            "stamp_ns": stamp,
            "state": state,
            "active": active,
            "authority_tracking": authority_tracking,
            "config": config,
            "counters": counters,
            "applied_offset_rad": applied_offset,
            "active_reference_variance_rad2": active_reference_variance,
            "trusted_variance_rad2": reference_values[-1],
            "last_reason": last_reason,
        }
        if parsed:
            previous = parsed[-1]
            if stamp < previous["stamp_ns"]:
                errors.append(
                    f"guard diagnostic stamp backstep {previous['stamp_ns']}->{stamp}"
                )
            counter_delta: dict[str, int] = {}
            for key in OUTAGE_YAW_GUARD_COUNTER_KEYS:
                delta = counters[key] - previous["counters"][key]
                counter_delta[key] = delta
                if delta < 0:
                    errors.append(
                        f"guard counter {key} backstep at sample {index}: {delta}"
                    )

            reset_delta = counter_delta["outage_yaw_guard.reset_count"]
            outage_delta = counter_delta["outage_yaw_guard.outage_count"]
            recovery_delta = counter_delta["outage_yaw_guard.recovery_count"]
            accepted_delta = counter_delta[
                "outage_yaw_guard.accepted_reference_count"
            ]
            step_delta = counter_delta["outage_yaw_guard.applied_step_count"]
            previous_state = previous["state"]
            previous_outage = previous_state in OUTAGE_YAW_GUARD_OUTAGE_STATES
            current_outage = state in OUTAGE_YAW_GUARD_OUTAGE_STATES
            if reset_delta == 0:
                expected_outage_delta = (
                    recovery_delta
                    + int(current_outage)
                    - int(previous_outage)
                )
                if outage_delta != expected_outage_delta:
                    errors.append(
                        "guard outage/recovery endpoint balance mismatch "
                        f"at sample {index}: outage_delta={outage_delta} "
                        f"recovery_delta={recovery_delta} "
                        f"previous_state={previous_state} state={state} "
                        f"expected_outage_delta={expected_outage_delta}"
                    )
            if (
                state in OUTAGE_YAW_GUARD_OUTAGE_STATES
                and previous_state not in OUTAGE_YAW_GUARD_OUTAGE_STATES
                and outage_delta <= 0
            ):
                errors.append(
                    f"guard transition {previous_state}->{state} lacks outage edge"
                )
            if (
                state == "RECOVERY_RELEASE"
                and previous_state != "RECOVERY_RELEASE"
                and recovery_delta <= 0
            ):
                errors.append(
                    f"guard transition {previous_state}->{state} lacks recovery edge"
                )
            if (
                previous_state in OUTAGE_YAW_GUARD_OUTAGE_STATES
                and state in {"READY", "DISARMED"}
                and recovery_delta <= 0
                and reset_delta <= 0
            ):
                errors.append(
                    f"guard transition {previous_state}->{state} skips recovery"
                )
            if (
                previous_state == "RECOVERY_RELEASE"
                and state in OUTAGE_YAW_GUARD_OUTAGE_STATES
                and outage_delta <= 0
            ):
                errors.append(
                    f"guard transition {previous_state}->{state} lacks re-outage edge"
                )
            if previous_state == "DISARMED" and state == "READY" and accepted_delta <= 0:
                errors.append("guard DISARMED->READY transition lacks accepted reference")
            if previous_state == "READY" and state == "RECOVERY_RELEASE" and not (
                outage_delta > 0 and recovery_delta > 0
            ):
                errors.append("guard READY->RECOVERY_RELEASE lacks sampled hidden outage")
            if previous_state == "DISARMED" and state == "RECOVERY_RELEASE" and not (
                accepted_delta > 0 and outage_delta > 0 and recovery_delta > 0
            ):
                errors.append("guard DISARMED->RECOVERY_RELEASE lacks authority edges")
            if (
                previous_state in OUTAGE_YAW_GUARD_OUTAGE_STATES
                and state == "DISARMED"
                and reset_delta <= 0
                and not (recovery_delta > 0 and authority_tracking)
            ):
                errors.append(
                    "guard active outage disarmed without reset or an unobserved "
                    f"healthy recovery completion at sample {index}"
                )

            if reset_delta == 0:
                if previous["active"] and active:
                    previous_active_variance = previous[
                        "active_reference_variance_rad2"
                    ]
                    previous_outage_count = previous["counters"][
                        "outage_yaw_guard.outage_count"
                    ]
                    current_outage_count = counters[
                        "outage_yaw_guard.outage_count"
                    ]
                    if current_outage_count == previous_outage_count:
                        if abs(
                            active_reference_variance
                            - previous_active_variance
                        ) > 1.0e-12:
                            errors.append(
                                "guard active reference variance changed within "
                                f"outage {current_outage_count} at sample {index}: "
                                f"{previous_active_variance}->"
                                f"{active_reference_variance}"
                            )
                    elif (
                        previous_state == "RECOVERY_RELEASE"
                        and state in OUTAGE_YAW_GUARD_OUTAGE_STATES
                        and current_outage_count - previous_outage_count == 1
                        and recovery_delta == 0
                        and last_reason in OUTAGE_YAW_GUARD_REOUTAGE_REASONS
                    ):
                        if last_reason == OUTAGE_YAW_GUARD_REOUTAGE_FRESH_REASON:
                            expected_active_variance = max(
                                previous_active_variance,
                                current["trusted_variance_rad2"],
                            )
                        elif last_reason in OUTAGE_YAW_GUARD_REOUTAGE_RETAIN_REASONS:
                            expected_active_variance = previous_active_variance
                        else:
                            errors.append(
                                "guard visible release re-outage has invalid reason "
                                f"at sample {index}: {last_reason!r}"
                            )
                            expected_active_variance = None
                        if (
                            expected_active_variance is not None
                            and abs(
                                active_reference_variance
                                - expected_active_variance
                            ) > 1.0e-12
                        ):
                            errors.append(
                                "guard re-outage active reference variance violates "
                                f"reason={last_reason!r} at sample {index}: actual="
                                f"{active_reference_variance} expected="
                                f"{expected_active_variance}"
                            )
                    elif current_outage_count > previous_outage_count:
                        # A recovery and subsequent re-outage can both occur
                        # between diagnostic snapshots. Its exact branch reason
                        # is not observable, but all branches retain or increase
                        # the active uncertainty snapshot.
                        if active_reference_variance + 1.0e-12 < (
                            previous_active_variance
                        ):
                            errors.append(
                                "guard hidden re-outage decreased active reference "
                                f"variance at sample {index}: "
                                f"{previous_active_variance}->"
                                f"{active_reference_variance}"
                            )
                offset_delta = abs(wrap(applied_offset - previous["applied_offset_rad"]))
                step_bound = step_delta * max_offset_step + 1.0e-9
                if offset_delta > step_bound:
                    errors.append(
                        f"guard applied offset exceeded step accounting at sample {index}: "
                        f"delta={offset_delta} steps={step_delta} bound={step_bound}"
                    )
                dt_sec = max(0.0, (stamp - previous["stamp_ns"]) * 1.0e-9)
                rate_bound = max_offset_rate * (dt_sec + max_step_dt) + 1.0e-9
                if offset_delta > rate_bound:
                    errors.append(
                        f"guard applied offset exceeded rate envelope at sample {index}: "
                        f"delta={offset_delta} dt={dt_sec} bound={rate_bound}"
                    )
        parsed.append(current)

    stamp_order_valid, leading_zero_count = timer_stamp_order(
        [item["stamp_ns"] for item in parsed]
    )
    if not stamp_order_valid:
        errors.append(
            "guard diagnostic stamps are not a leading-zero prefix followed by "
            "nondecreasing physical time"
        )
    summary = {
        "samples": len(timeline),
        "parsed_samples": len(parsed),
        "leading_zero_stamp_samples": leading_zero_count,
        "states": sorted({item["state"] for item in parsed}),
        "active_samples": sum(item["active"] for item in parsed),
        "outage_samples": sum(
            item["state"] in OUTAGE_YAW_GUARD_OUTAGE_STATES for item in parsed
        ),
        "release_samples": sum(
            item["state"] == "RECOVERY_RELEASE" for item in parsed
        ),
        "maximum_active_reference_variance_rad2": max(
            (
                item["active_reference_variance_rad2"]
                for item in parsed if item["active"]
            ),
            default=None,
        ),
        "config": list(reference_config) if reference_config is not None else None,
    }
    return bool(timeline) and len(parsed) == len(timeline) and not errors, summary, errors[:40]


def fusion_rearm_contract(
    timeline: list[dict[str, Any]],
) -> tuple[bool, dict[str, Any], list[str]]:
    """Prove session rearm uses an explicit fresh unhealthy->healthy edge."""
    errors: list[str] = []
    latest_reset_count = 0
    explicit_unhealthy_status_sec: float | None = None
    completed_resets = 0
    for item in timeline:
        values = item["values"]
        try:
            reset_count = int(values["local_correction.odom_session_resets"])
            reset_stamp = float(values["fusion.health.rearm_reset_stamp_sec"])
            status_stamp = float(values["fusion.health.status_stamp_sec"])
            health_age = float(values["fusion.health.age_sec"])
        except (KeyError, ValueError):
            errors.append(f"malformed rearm diagnostic at {item['stamp_ns']}")
            continue
        required = values.get("fusion.health.rearm_required") == "true"
        saw_unhealthy = values.get("fusion.health.rearm_saw_unhealthy") == "true"
        rearmed = values.get("fusion.health.rearmed") == "true"
        if reset_count < latest_reset_count or reset_count > latest_reset_count + 1:
            errors.append(
                f"session-reset counter jump {latest_reset_count}->{reset_count}"
            )
        if reset_count != latest_reset_count:
            latest_reset_count = reset_count
            explicit_unhealthy_status_sec = None

        if reset_count == 0:
            if required or saw_unhealthy or not rearmed:
                errors.append(
                    f"initial startup incorrectly entered rearm at {item['stamp_ns']}"
                )
            continue

        strict_fields = (
            values.get("fusion.health.level") == "0"
            and values.get("fusion.health.recovery_state") == "tracking"
            and values.get("fusion.health.anchor_valid") == "true"
            and values.get("fusion.health.position_fused") == "true"
            and values.get("fusion.health.yaw_fused") == "true"
            and values.get("fusion.health.last_fix_state") == "good"
        )
        fresh_status_after_reset = (
            math.isfinite(reset_stamp)
            and math.isfinite(status_stamp)
            and status_stamp > reset_stamp
            and math.isfinite(health_age)
            and -0.25 <= health_age <= 1.5
        )
        if fresh_status_after_reset and not strict_fields:
            explicit_unhealthy_status_sec = status_stamp
        if saw_unhealthy and explicit_unhealthy_status_sec is None:
            errors.append(
                f"rearm accepted stale/unavailable without explicit unhealthy status "
                f"at {item['stamp_ns']}"
            )
        if required and rearmed:
            errors.append(f"rearm flags conflict at {item['stamp_ns']}")
        if not required and rearmed and completed_resets < reset_count:
            strict_recovery = (
                explicit_unhealthy_status_sec is not None
                and fresh_status_after_reset
                and status_stamp > explicit_unhealthy_status_sec
                and strict_fields
                and values.get("fusion.health.healthy") == "true"
                and values.get("fusion.health.reason") == "strict_fusion_health_ok"
            )
            if not strict_recovery:
                errors.append(
                    f"session rearmed without fresh strict tracking edge at "
                    f"{item['stamp_ns']}"
                )
            else:
                completed_resets = max(completed_resets, reset_count)
    return (
        bool(timeline) and not errors,
        {
            "observed_samples": len(timeline),
            "session_resets": latest_reset_count,
            "completed_resets": completed_resets,
            "contract": (
                "stale/unavailable never qualifies; fresh explicit unhealthy "
                "then fresh strict TRACKING"
            ),
        },
        errors[:20],
    )


def validate(bag: Path, expected_rate: float) -> list[Check]:
    _, deserialize_message, get_message = import_rosbag_modules()
    reader = open_reader(bag)
    type_by_topic = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    required = {
        TOPIC_SCAN,
        TOPIC_CORRECTION,
        TOPIC_RAW,
        TOPIC_EXISTING,
        TOPIC_LOCAL,
        TOPIC_GLOBAL,
        TOPIC_POSE,
    }
    checks: list[Check] = []
    missing = sorted(required - set(type_by_topic))
    checks.append(Check("required topics", not missing, "missing=" + repr(missing)))
    if missing:
        return checks

    wanted_topics = required | {TOPIC_DIAGNOSTICS}
    message_class = {
        topic: get_message(type_by_topic[topic])
        for topic in wanted_topics
        if topic in type_by_topic
    }
    messages: dict[str, list[tuple[int, Any]]] = defaultdict(list)
    while reader.has_next():
        topic, data, record_ns = reader.read_next()
        if topic in required or topic == TOPIC_DIAGNOSTICS:
            messages[topic].append((record_ns, deserialize_message(data, message_class[topic])))

    scan_by_key: dict[tuple[int, int, int, int], Any] = {}
    duplicate_scan_keys = 0
    for _, scan in messages[TOPIC_SCAN]:
        key = keyed(scan)
        duplicate_scan_keys += int(key in scan_by_key)
        scan_by_key[key] = scan
    correction_by_key: dict[tuple[int, int, int, int], Any] = {}
    duplicate_correction_keys = 0
    for _, correction in messages[TOPIC_CORRECTION]:
        key = keyed(correction)
        duplicate_correction_keys += int(key in correction_by_key)
        correction_by_key[key] = correction

    unknown_corrections = sorted(set(correction_by_key) - set(scan_by_key))
    checks.append(
        Check(
            "exact correction keys",
            duplicate_scan_keys == 0
            and duplicate_correction_keys == 0
            and not unknown_corrections,
            f"scans={len(scan_by_key)} corrections={len(correction_by_key)} "
            f"duplicate_scans={duplicate_scan_keys} "
            f"duplicate_corrections={duplicate_correction_keys} "
            f"unknown_corrections={len(unknown_corrections)}",
        )
    )

    correction_schema_valid = True
    maximum_composition_position_error = 0.0
    maximum_composition_orientation_error = 0.0
    correction_schema_reasons: list[str] = []
    previous_by_matcher: dict[int, tuple[int, int, int]] = {}
    current_matcher_session: int | None = None
    closed_matcher_sessions: set[int] = set()
    precision_frames: set[str] = set()
    raw_frames: set[str] = set()
    for key, correction in correction_by_key.items():
        scan = scan_by_key.get(key)
        precision_frames.add(str(correction.precision_frame_id))
        if scan is not None:
            raw_frames.add(str(scan.header.frame_id))
        transform = correction.precision_from_raw
        transform_values = (
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
            *quaternion_values(transform.rotation),
        )
        pose = correction.corrected_pose.pose
        pose_values = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            *quaternion_values(pose.orientation),
            *[float(value) for value in correction.corrected_pose.covariance],
            float(correction.fitness),
            float(correction.inlier_ratio),
            float(correction.innovation_translation_m),
            float(correction.innovation_x_m),
            float(correction.innovation_y_m),
            float(correction.innovation_yaw_rad),
        )
        valid = (
            scan is not None
            and bool(correction.precision_frame_id)
            and correction.header.frame_id == scan.header.frame_id
            and correction.use_yaw
            and all(math.isfinite(value) for value in transform_values + pose_values)
            and abs(quaternion_norm(transform.rotation) - 1.0) <= 1.0e-6
            and abs(quaternion_norm(pose.orientation) - 1.0) <= 1.0e-6
            and abs(float(transform.translation.z)) <= 1.0e-9
            and abs(float(transform.rotation.x)) <= 1.0e-9
            and abs(float(transform.rotation.y)) <= 1.0e-9
        )
        if scan is not None:
            position_error, orientation_error = correction_composition_error(scan, correction)
            maximum_composition_position_error = max(
                maximum_composition_position_error, position_error
            )
            maximum_composition_orientation_error = max(
                maximum_composition_orientation_error, orientation_error
            )
            valid = valid and position_error <= 1.0e-5 and orientation_error <= 1.0e-5
        matcher_session = int(correction.matcher_session_id)
        valid = valid and matcher_session != 0
        if current_matcher_session is None:
            current_matcher_session = matcher_session
        elif matcher_session != current_matcher_session:
            closed_matcher_sessions.add(current_matcher_session)
            valid = (
                valid
                and matcher_session not in closed_matcher_sessions
                and matcher_session != 0
            )
            current_matcher_session = matcher_session
        current = (
            int(correction.submap_generation),
            int(correction.correction_id),
            key[3],
        )
        previous = previous_by_matcher.get(matcher_session)
        if previous is not None:
            valid = valid and current[0] >= previous[0] and current[1] > previous[1]
            valid = valid and current[2] > previous[2]
        previous_by_matcher[matcher_session] = current
        if not valid and len(correction_schema_reasons) < 5:
            correction_schema_reasons.append(str(key))
        correction_schema_valid = correction_schema_valid and valid
    correction_stream_order = exact_key_stream_order(list(correction_by_key))
    if not correction_stream_order and len(correction_schema_reasons) < 5:
        correction_schema_reasons.append("exact_key_stream_order")
    correction_schema_valid = correction_schema_valid and correction_stream_order
    checks.append(
        Check(
            "correction full-SE2 contract",
            bool(correction_by_key) and correction_schema_valid,
            f"checked={len(correction_by_key)} invalid_examples={correction_schema_reasons} "
            f"max_position_error_m={maximum_composition_position_error:.9g} "
            f"max_orientation_error_rad={maximum_composition_orientation_error:.9g}",
        )
    )
    local_frames = {
        (str(message.header.frame_id), str(message.child_frame_id))
        for _, message in messages[TOPIC_LOCAL]
    }
    global_frames = {
        (str(message.header.frame_id), str(message.child_frame_id))
        for _, message in messages[TOPIC_GLOBAL]
    }
    pose_frames = {
        str(message.header.frame_id) for _, message in messages[TOPIC_POSE]
    }
    checks.append(
        Check(
            "dedicated precision frames",
            len(precision_frames) == 1
            and len(raw_frames) == 1
            and precision_frames.isdisjoint(raw_frames)
            and local_frames == {(next(iter(precision_frames), ""), "base_link")}
            and global_frames == {("map", "base_link")}
            and pose_frames == {"map"},
            f"raw={sorted(raw_frames)} precision={sorted(precision_frames)} "
            f"local={sorted(local_frames)} global={sorted(global_frames)} "
            f"pose={sorted(pose_frames)}",
        )
    )

    ordered_scan_keys = [keyed(scan) for _, scan in messages[TOPIC_SCAN]]
    physical_scan_stamps = [key[3] for key in ordered_scan_keys]
    scan_stream_strict = exact_key_stream_order(ordered_scan_keys)
    checks.append(
        Check(
            "physical scan stream order",
            bool(physical_scan_stamps)
            and scan_stream_strict
            and physical_scan_stamps[0] > 0,
            f"count={len(physical_scan_stamps)} first_ns="
            f"{physical_scan_stamps[0] if physical_scan_stamps else 0}",
        )
    )

    for label, topic in (("raw odom", TOPIC_RAW), ("precision local", TOPIC_LOCAL), ("precision global", TOPIC_GLOBAL)):
        stamps = [stamp_ns(message.header.stamp) for _, message in messages[topic]]
        # The stack intentionally publishes while /clock is still zero during
        # DDS/player startup.  Once the first physical clock value arrives, a
        # stream may repeat a timer stamp but must never backstep or return to
        # zero.
        valid, leading_zero_count = timer_stamp_order(stamps)
        checks.append(
            Check(
                f"{label} stamp order after clock initialization",
                valid,
                f"count={len(stamps)} leading_zero_count={leading_zero_count}",
            )
        )
        checks.append(
            Check(
                f"{label} intentional startup zero stamps",
                True,
                f"leading_zero_count={leading_zero_count}",
                warning=leading_zero_count > 0,
            )
        )

    # Intentional warm-up is measured explicitly rather than pretending the
    # matcher should emit corrections from the first two keyframes.
    if physical_scan_stamps and correction_by_key:
        first_scan = min(physical_scan_stamps)
        first_correction = min(key[3] for key in correction_by_key)
        warmup_sec = (first_correction - first_scan) * 1.0e-9
        post_warmup_scans = sum(
            stamp >= first_correction for stamp in physical_scan_stamps
        )
    else:
        warmup_sec = math.inf
        post_warmup_scans = 0
    correction_coverage = (
        len(correction_by_key) / post_warmup_scans if post_warmup_scans else 0.0
    )
    checks.append(
        Check(
            "intentional initialization coverage",
            math.isfinite(warmup_sec)
            and 0.0 <= warmup_sec <= 10.0
            and correction_coverage >= 0.70,
            f"warmup_sec={warmup_sec:.3f} post-warmup correction ratio="
            f"{correction_coverage:.3f}",
        )
    )

    matcher_diag: dict[str, str] | None = None
    global_diag: dict[str, str] | None = None
    odometer_diag: dict[str, str] | None = None
    global_timeline: list[dict[str, Any]] = []
    fusion_health_records: list[dict[str, Any]] = []
    for diagnostic_record_ns, diagnostic in messages[TOPIC_DIAGNOSTICS]:
        matcher_diag = diagnostic_values(diagnostic, "localization/submap_matcher") or matcher_diag
        global_diag = diagnostic_values(
            diagnostic, "localization/precision_global_localizer"
        ) or global_diag
        global_status = diagnostic_status(
            diagnostic, "localization/precision_global_localizer"
        )
        if global_status is not None:
            global_key_counts: dict[str, int] = defaultdict(int)
            for item in global_status.values:
                global_key_counts[str(item.key)] += 1
            global_timeline.append(
                {
                    "record_ns": diagnostic_record_ns,
                    "stamp_ns": stamp_ns(diagnostic.header.stamp),
                    "level": integer_value(global_status.level),
                    "message": str(global_status.message),
                    "values": {
                        item.key: item.value for item in global_status.values
                    },
                    "key_counts": dict(global_key_counts),
                }
            )
        fusion_statuses = [
            status for status in diagnostic.status
            if status.name == FUSION_DIAGNOSTIC_STATUS
            or status.name.endswith("/" + FUSION_DIAGNOSTIC_STATUS)
        ]
        if fusion_statuses:
            selected = fusion_statuses[0]
            fusion_health_records.append(
                {
                    "record_ns": diagnostic_record_ns,
                    "stamp_ns": stamp_ns(diagnostic.header.stamp),
                    "status_count": len(fusion_statuses),
                    "values": [(item.key, item.value) for item in selected.values],
                }
            )
        odometer_diag = diagnostic_values(
            diagnostic, "localization/gyro_odometer"
        ) or odometer_diag

    publication_valid, publication_summary, publication_errors = (
        map_fusion_publication_integrity_contract(fusion_health_records)
    )
    checks.append(
        Check(
            "map-fusion publication counter integrity",
            publication_valid,
            f"summary={publication_summary} errors={publication_errors}",
        )
    )
    final_map_fusion_diagnostic_record_ns = (
        int(fusion_health_records[-1]["record_ns"])
        if fusion_health_records else None
    )
    coverage_valid, coverage_summary, coverage_errors = (
        exact_raw_to_existing_stamp_coverage(
            messages[TOPIC_RAW],
            messages[TOPIC_EXISTING],
            final_map_fusion_diagnostic_record_ns,
        )
    )
    checks.append(
        Check(
            "map-fusion exact raw stamp publication coverage",
            coverage_valid,
            f"summary={coverage_summary} errors={coverage_errors}",
        )
    )

    if odometer_diag is None:
        checks.append(Check("odometer snapshot diagnostics", False, "status not recorded"))
    else:
        snapshot_enabled = (
            odometer_diag.get("external_submap_snapshot_enabled", "false") == "true"
        )
        snapshot_count = int(
            float(odometer_diag.get("external_submap_snapshot_published_count", "-1"))
        )
        conversion_max = float(
            odometer_diag.get("external_submap_snapshot_conversion_max_ms", "nan")
        )
        tracking_mode = odometer_diag.get("lidar_tracking_mode", "")
        checks.append(
            Check(
                "odometer isolated snapshot contract",
                snapshot_enabled
                and tracking_mode == "scan_to_scan"
                and snapshot_count == len(scan_by_key)
                and math.isfinite(conversion_max)
                and conversion_max < 20.0,
                f"enabled={snapshot_enabled} tracking={tracking_mode} "
                f"diag_scans={snapshot_count} bag_scans={len(scan_by_key)} "
                f"conversion_max_ms={conversion_max:.6f}",
            )
        )

    if matcher_diag is None:
        checks.append(Check("matcher diagnostics", False, "status not recorded"))
    else:
        queue_drop = int(float(matcher_diag.get("queue_drop_count", "0")))
        malformed = int(float(matcher_diag.get("malformed_count", "0")))
        stale = int(float(matcher_diag.get("stale_or_duplicate_count", "-1")))
        received = int(float(matcher_diag.get("received_count", "-1")))
        processed = int(float(matcher_diag.get("processed_count", "-1")))
        stream_reset = int(float(matcher_diag.get("stream_reset_count", "-1")))
        recovery_rebuild = int(
            float(matcher_diag.get("recovery_rebuild_count", "-1"))
        )
        attempted = int(float(matcher_diag.get("attempted_count", "0")))
        accepted = int(float(matcher_diag.get("accepted_match_count", "0")))
        rejected = int(float(matcher_diag.get("rejected_match_count", "-1")))
        committed = int(float(matcher_diag.get("committed_count", "-1")))
        correction_published = int(
            float(matcher_diag.get("correction_publish_count", "-1"))
        )
        submap_generation = int(float(matcher_diag.get("submap_generation", "-1")))
        last_reason = matcher_diag.get("last_reason", "")
        duplicate_policy = matcher_diag.get("late_duplicate_policy", "")
        rejection_policy = matcher_diag.get("rejection_policy", "")
        processing_p99 = float(matcher_diag.get("processing_p99_ms", "nan"))
        latency_p99 = float(matcher_diag.get("latency_p99_ms", "nan"))
        ratio = accepted / attempted if attempted else 0.0
        checks.append(
            Check(
                "matcher stream health",
                queue_drop == 0
                and malformed == 0
                and stale == 0
                and received == processed
                and attempted == accepted + rejected
                and committed == correction_published
                and stream_reset >= recovery_rebuild + 1
                and submap_generation == stream_reset
                and bool(last_reason)
                and duplicate_policy == "ignore_without_stream_or_map_reset"
                and rejection_policy == "preserve_committed_transform_rebuild_map_only"
                and attempted > 0
                and ratio >= 0.75,
                f"queue_drop={queue_drop} malformed={malformed} stale={stale} "
                f"received/processed={received}/{processed} attempted={attempted} "
                f"accepted/rejected={accepted}/{rejected} committed/published="
                f"{committed}/{correction_published} stream_reset/recovery="
                f"{stream_reset}/{recovery_rebuild} generation={submap_generation} "
                f"non_recovery_resets={stream_reset - recovery_rebuild - 1} "
                f"last_reason={last_reason!r} "
                f"accepted_ratio={ratio:.3f}",
            )
        )
        diagnostic_received = int(float(matcher_diag.get("received_count", "-1")))
        diagnostic_corrections = int(
            float(matcher_diag.get("correction_publish_count", "-1"))
        )
        checks.append(
            Check(
                "matcher diagnostic counts match recording",
                diagnostic_received == len(scan_by_key)
                and diagnostic_corrections == len(correction_by_key),
                f"diag_scans={diagnostic_received} bag_scans={len(scan_by_key)} "
                f"diag_corrections={diagnostic_corrections} "
                f"bag_corrections={len(correction_by_key)}",
            )
        )
        # Real-time acceptance is based on the rolling p99, never one scheduler
        # outlier.  At 4 Hz, 250 ms is the no-backlog budget at 1x playback.
        checks.append(
            Check(
                "matcher latency p99",
                math.isfinite(processing_p99)
                and math.isfinite(latency_p99)
                and processing_p99 < 250.0 / expected_rate
                and latency_p99 < 250.0 / expected_rate,
                f"processing_p99_ms={processing_p99:.3f} "
                f"latency_p99_ms={latency_p99:.3f} expected_rate={expected_rate:.3f}",
            )
        )

    if global_diag is None:
        checks.append(Check("precision global diagnostics", False, "status not recorded"))
    else:
        state = global_diag.get("state", "")
        anchor_initialized = global_diag.get("anchor.initialized", "false") == "true"
        position_fused = global_diag.get("position_fused", "false") == "true"
        correction_accepted = int(
            float(global_diag.get("local_correction.accepted", "-1"))
        )
        correction_rejected = int(
            float(global_diag.get("local_correction.rejected", "-1"))
        )
        scan_received = int(float(global_diag.get("submap.scan_received", "-1")))
        correction_received = int(
            float(global_diag.get("submap.correction_received", "-1"))
        )
        raw_nonmonotonic = int(float(global_diag.get("raw.nonmonotonic", "-1")))
        local_published = int(float(global_diag.get("publish.local", "-1")))
        global_published = int(float(global_diag.get("publish.global", "-1")))
        checks.append(
            Check(
                "precision global diagnostics",
                state not in ("", "UNINITIALIZED")
                and anchor_initialized
                and position_fused
                and scan_received == len(scan_by_key)
                and correction_received == len(correction_by_key)
                and correction_accepted == len(correction_by_key)
                and correction_rejected == 0
                and raw_nonmonotonic == 0
                and local_published > 0
                and global_published > 0,
                f"state={state} anchor_initialized={anchor_initialized} "
                f"position_fused={position_fused} "
                f"scan_received={scan_received}/{len(scan_by_key)} "
                f"correction_received={correction_received}/{len(correction_by_key)} "
                f"correction_accepted={correction_accepted}/"
                f"{len(correction_by_key)} correction_rejected={correction_rejected} "
                f"raw_nonmonotonic={raw_nonmonotonic} publish_local={local_published} "
                f"publish_global={global_published}",
            )
        )

        authority_source = global_diag.get("anchor.source", "")
        fallback_enabled = (
            global_diag.get("fallback.gnss_position_enabled", "true") == "true"
        )
        health_received = int(float(global_diag.get("fusion.health.received", "-1")))
        health_accepted = int(float(global_diag.get("fusion.health.accepted", "-1")))
        health_rejected = int(float(global_diag.get("fusion.health.rejected", "-1")))
        existing_received = int(float(
            global_diag.get("fusion.sync.existing_global_received", "-1")
        ))
        existing_accepted = int(float(
            global_diag.get("fusion.sync.existing_global_accepted", "-1")
        ))
        existing_rejected = int(float(
            global_diag.get("fusion.sync.existing_global_rejected", "-1")
        ))
        existing_duplicate = int(float(
            global_diag.get("fusion.sync.existing_global_duplicate_stamp", "-1")
        ))
        fusion_sync_accepted = int(float(
            global_diag.get("fusion.sync.accepted", "-1")
        ))
        anchor_accepted = int(float(
            global_diag.get("fusion.anchor.accepted_count", "-1")
        ))
        target_updates = int(float(
            global_diag.get("fusion.anchor.target_update_count", "-1")
        ))
        activation_commits = int(float(
            global_diag.get("activation.commit_count", "-1")
        ))
        odom_session_resets = int(float(
            global_diag.get("local_correction.odom_session_resets", "-1")
        ))
        activation_watermark_suppressed = int(float(
            global_diag.get(
                "publish.global_suppressed_activation_watermark", "-1"
            )
        ))
        frozen_residual_variances = [
            float(global_diag.get(key, "nan"))
            for key in (
                "fusion.anchor.frozen_residual_variance_x_m2",
                "fusion.anchor.frozen_residual_variance_y_m2",
                "fusion.anchor.frozen_residual_variance_yaw_rad2",
            )
        ]
        authority_invariant = bool(global_timeline) and all(
            item["values"].get("anchor.source") == "existing_fusion"
            and item["values"].get(
                "fallback.gnss_position_enabled", "true"
            ) == "false"
            for item in global_timeline
        )
        no_rearm_needed = (
            odom_session_resets == 0
            and all(
                item["values"].get("fusion.health.rearm_required") == "false"
                and item["values"].get(
                    "fusion.health.rearm_saw_unhealthy"
                ) == "false"
                and item["values"].get("fusion.health.rearmed") == "true"
                for item in global_timeline
            )
        )
        final_diagnostic_record_ns = (
            int(global_timeline[-1]["record_ns"]) if global_timeline else -1
        )
        existing_prefix = [
            item for item in messages[TOPIC_EXISTING]
            if item[0] <= final_diagnostic_record_ns
        ]
        health_prefix = [
            item for item in fusion_health_records
            if int(item["record_ns"]) <= final_diagnostic_record_ns
        ]
        existing_accounting = existing_global_prefix_accounting(existing_prefix)
        health_accounting = fusion_health_prefix_accounting(health_prefix)
        pre_record_startup_rejections = (
            health_received - int(health_accounting["received"])
        )
        existing_tail = len(messages[TOPIC_EXISTING]) - len(existing_prefix)
        health_tail = len(fusion_health_records) - len(health_prefix)
        checks.append(
            Check(
                "existing-fusion global authority",
                authority_invariant
                and authority_source == "existing_fusion"
                and not fallback_enabled
                and health_received > 0
                and health_accepted > 0
                and health_received == health_accepted + health_rejected
                and health_accounting["valid"]
                and pre_record_startup_rejections >= 0
                and health_accepted == health_accounting["accepted"]
                and health_rejected
                == health_accounting["rejected"] + pre_record_startup_rejections
                and existing_accounting["valid"]
                and existing_received == existing_accounting["received"]
                and existing_received
                == existing_accepted + existing_rejected + existing_duplicate
                and existing_accepted == existing_accounting["accepted"]
                and existing_rejected == existing_accounting["rejected"]
                and existing_duplicate == existing_accounting["duplicate"]
                and existing_accepted > 0
                and fusion_sync_accepted >= 3
                and anchor_accepted == fusion_sync_accepted
                and target_updates > 0
                and activation_commits == 1
                and no_rearm_needed
                and activation_watermark_suppressed >= 0
                and all(
                    math.isfinite(value) and value >= 0.0
                    for value in frozen_residual_variances
                ),
                f"source={authority_source!r} fallback={fallback_enabled} "
                f"all_samples_authority={authority_invariant} "
                f"health received/accepted/rejected={health_received}/"
                f"{health_accepted}/{health_rejected} recorded_health="
                f"{health_accounting} pre_record_startup_rejections="
                f"{pre_record_startup_rejections} health_tail={health_tail} "
                f"existing received/accepted/"
                f"rejected/duplicate={existing_received}/{existing_accepted}/"
                f"{existing_rejected}/{existing_duplicate} causal_prefix="
                f"{existing_accounting} existing_tail={existing_tail} "
                f"bag_existing={len(messages[TOPIC_EXISTING])} fusion_sync_accepted="
                f"{fusion_sync_accepted} anchor_accepted={anchor_accepted} "
                f"target_updates={target_updates} activations={activation_commits} "
                f"odom_session_resets={odom_session_resets} "
                f"no_rearm_needed={no_rearm_needed} activation_watermark_suppressed="
                f"{activation_watermark_suppressed} frozen_residual_variances="
                f"{frozen_residual_variances}",
            )
        )

    startup_required_keys = {
        "position_initialized",
        "position_fused",
        "yaw_publishable",
        "global_output_ready",
        "anchor.initialized",
        "anchor.yaw_observed",
        "anchor.yaw_publishable",
        "anchor.correction_lag.yaw_rad",
        "activation.stamp_sec",
        "activation.stable_candidate_count",
        "activation.required_candidate_count",
        "activation.candidate_yaw_rad",
        "activation.candidate_delta_rad",
        "activation.reason",
        "activation.epoch",
        "activation.commit_count",
        "publish.global_suppressed_not_ready",
        "publish.global_suppressed_activation_watermark",
        "publish.global",
        "anchor.source",
        "fusion.health.healthy",
        "fusion.health.age_sec",
        "fusion.health.status_stamp_sec",
        "fusion.health.level",
        "fusion.health.recovery_state",
        "fusion.health.anchor_valid",
        "fusion.health.position_fused",
        "fusion.health.yaw_fused",
        "fusion.health.last_fix_state",
        "fusion.health.reason",
        "fusion.health.rearm_required",
        "fusion.health.rearm_saw_unhealthy",
        "fusion.health.rearmed",
        "fusion.health.rearm_reset_stamp_sec",
        "fusion.anchor.state",
        "fusion.anchor.accepted_count",
        "fusion.anchor.rejected_count",
        "fusion.anchor.target_update_count",
        "fusion.anchor.freeze_count",
        "fusion.anchor.recovery_count",
        "fusion.anchor.frozen_residual_variance_x_m2",
        "fusion.anchor.frozen_residual_variance_y_m2",
        "fusion.anchor.frozen_residual_variance_yaw_rad2",
        "fusion.sync.accepted",
        "fusion.sync.rejected",
        "fusion.sync.existing_global_received",
        "fusion.sync.existing_global_accepted",
        "fusion.sync.existing_global_rejected",
        "fusion.sync.existing_global_duplicate_stamp",
        "fusion.sync.existing_global_stamp_sec",
        "fusion.sync.existing_global_age_sec",
        "fusion.sync.last_valid_stamp_sec",
        "fusion.sync.last_valid_age_sec",
        "local_correction.odom_session_resets",
        "fallback.gnss_position_enabled",
    }
    recorded_startup_keys = (
        set().union(*(set(item["values"]) for item in global_timeline))
        if global_timeline else set()
    )
    missing_startup_keys = sorted(startup_required_keys - recorded_startup_keys)
    incomplete_startup_samples = sum(
        not startup_required_keys.issubset(item["values"])
        for item in global_timeline
    )
    checks.append(
        Check(
            "startup readiness diagnostic schema",
            not missing_startup_keys
            and bool(global_timeline)
            and incomplete_startup_samples == 0,
            f"missing={missing_startup_keys} samples={len(global_timeline)} "
            f"incomplete_samples={incomplete_startup_samples}",
        )
    )
    freeze_valid, freeze_groups, freeze_errors = fusion_anchor_freeze_contract(
        global_timeline
    )
    checks.append(
        Check(
            "existing-fusion anchor freezes outside strict health",
            freeze_valid,
            f"groups={freeze_groups} errors={freeze_errors}",
        )
    )
    guard_valid, guard_summary, guard_errors = outage_yaw_guard_contract(
        global_timeline
    )
    checks.append(
        Check(
            "outage yaw guard runtime contract",
            guard_valid,
            f"summary={guard_summary} errors={guard_errors}",
        )
    )
    rearm_valid, rearm_summary, rearm_errors = fusion_rearm_contract(
        global_timeline
    )
    checks.append(
        Check(
            "existing-fusion session rearm requires an explicit fresh health edge",
            rearm_valid,
            f"summary={rearm_summary} errors={rearm_errors}",
        )
    )
    transition_valid, activation_transitions, transition_errors = (
        startup_transition_contract(global_timeline)
        if not missing_startup_keys and incomplete_startup_samples == 0
        else (False, [], ["missing or incomplete schema"])
    )
    checks.append(
        Check(
            "startup readiness transitions",
            transition_valid,
            f"transitions={activation_transitions} errors={transition_errors}",
        )
    )

    raw_positive_stamps = [
        stamp_ns(message.header.stamp) for _, message in messages[TOPIC_RAW]
        if stamp_ns(message.header.stamp) > 0
    ]
    global_positive_records = [
        (record_ns, message) for record_ns, message in messages[TOPIC_GLOBAL]
        if stamp_ns(message.header.stamp) > 0
    ]
    pose_positive_records = [
        (record_ns, message) for record_ns, message in messages[TOPIC_POSE]
        if stamp_ns(message.header.stamp) > 0
    ]
    global_positive_stamps = [
        stamp_ns(message.header.stamp) for _, message in global_positive_records
    ]
    pose_positive_stamps = [
        stamp_ns(message.header.stamp) for _, message in pose_positive_records
    ]
    local_positive_records = [
        (record_ns, message) for record_ns, message in messages[TOPIC_LOCAL]
        if stamp_ns(message.header.stamp) > 0
    ]
    session_starts: list[int] = []
    prior_session: int | None = None
    for key in ordered_scan_keys:
        if key[0] != prior_session:
            session_starts.append(key[3])
            prior_session = key[0]
    # The first raw odometry stamp precedes the first sparse snapshot and is
    # the correct initialization-delay origin for epoch zero.
    if raw_positive_stamps and session_starts:
        session_starts[0] = raw_positive_stamps[0]
    startup_intervals_valid = (
        transition_valid
        and len(session_starts) == len(activation_transitions)
        and bool(global_positive_stamps)
        and bool(pose_positive_stamps)
    )
    interval_details: list[dict[str, Any]] = []
    if startup_intervals_valid:
        for index, transition in enumerate(activation_transitions):
            begin_ns = session_starts[index]
            end_ns = (
                session_starts[index + 1]
                if index + 1 < len(session_starts) else None
            )
            activation_ns = int(transition["activation_ns"])
            lower_bound = activation_ns - ACTIVATION_SERIALIZATION_TOLERANCE_NS
            interval_raw = [
                stamp for stamp in raw_positive_stamps
                if stamp >= begin_ns and (end_ns is None or stamp < end_ns)
            ]
            activation_raw, expected_first_global = activation_raw_successor(
                interval_raw,
                activation_ns,
                ACTIVATION_SERIALIZATION_TOLERANCE_NS,
            )
            interval_global = [
                stamp for stamp in global_positive_stamps
                if stamp >= begin_ns and (end_ns is None or stamp < end_ns)
            ]
            interval_pose = [
                stamp for stamp in pose_positive_stamps
                if stamp >= begin_ns and (end_ns is None or stamp < end_ns)
            ]
            before_global = sum(stamp < lower_bound for stamp in interval_global)
            before_pose = sum(stamp < lower_bound for stamp in interval_pose)
            first_global = min(interval_global) if interval_global else None
            first_pose = min(interval_pose) if interval_pose else None
            delay_sec = (
                (first_global - begin_ns) * 1.0e-9
                if first_global is not None else math.inf
            )
            first_anchor_error = math.inf
            if first_global is not None:
                first_global_record_ns, global_message = next(
                    (record_ns, message)
                    for record_ns, message in global_positive_records
                    if stamp_ns(message.header.stamp) == first_global
                )
                local_candidates = [
                    (record_ns, message)
                    for record_ns, message in local_positive_records
                    if stamp_ns(message.header.stamp) == first_global
                    and record_ns <= first_global_record_ns
                ]
                if not local_candidates:
                    local_candidates = [
                        (record_ns, message)
                        for record_ns, message in local_positive_records
                        if stamp_ns(message.header.stamp) == first_global
                    ]
                if local_candidates:
                    _, local_message = max(local_candidates, key=lambda item: item[0])
                    first_anchor_error = abs(wrap(
                        yaw_of(global_message.pose.pose.orientation)
                        - yaw_of(local_message.pose.pose.orientation)
                        - float(transition["candidate_yaw_rad"])
                    ))
            interval_ok = (
                before_global == 0
                and before_pose == 0
                and first_global is not None
                and first_pose is not None
                and activation_raw is not None
                and expected_first_global is not None
                and first_global == expected_first_global
                and first_pose == expected_first_global
                and 0.0 <= delay_sec <= 25.0
                and math.isfinite(first_anchor_error)
                and first_anchor_error <= 0.02
                and math.isfinite(float(transition["anchor_lag_yaw_rad"]))
                and float(transition["anchor_lag_yaw_rad"]) <= 0.02
            )
            startup_intervals_valid = startup_intervals_valid and interval_ok
            interval_details.append(
                {
                    "epoch": transition["epoch"],
                    "begin_ns": begin_ns,
                    "activation_ns": activation_ns,
                    "activation_raw_ns": activation_raw,
                    "expected_first_global_ns": expected_first_global,
                    "first_global_ns": first_global,
                    "first_pose_ns": first_pose,
                    "before_global": before_global,
                    "before_pose": before_pose,
                    "delay_sec": delay_sec,
                    "first_anchor_yaw_error_rad": first_anchor_error,
                    "diagnostic_anchor_lag_yaw_rad": transition["anchor_lag_yaw_rad"],
                }
            )
    checks.append(
        Check(
            "startup global publication safety",
            startup_intervals_valid,
            f"session_starts={session_starts} intervals={interval_details} "
            f"activation_serialization_tolerance_ns="
            f"{ACTIVATION_SERIALIZATION_TOLERANCE_NS}",
        )
    )
    if global_diag is not None:
        suppressed = int(float(
            global_diag.get("publish.global_suppressed_not_ready", "-1")
        ))
        checks.append(
            Check(
                "startup global suppression exercised",
                suppressed > 0,
                f"suppressed_not_ready={suppressed}",
            )
        )

    scan_record_times = [record_ns for record_ns, _ in messages[TOPIC_SCAN]]
    if len(scan_record_times) > 1 and physical_scan_stamps:
        wall_duration = (scan_record_times[-1] - scan_record_times[0]) * 1.0e-9
        bag_duration = (max(physical_scan_stamps) - min(physical_scan_stamps)) * 1.0e-9
        observed_rate = bag_duration / wall_duration if wall_duration > 0.0 else math.inf
        checks.append(
            Check(
                "playback rate",
                expected_rate * 0.90 <= observed_rate <= expected_rate * 1.10,
                f"observed={observed_rate:.3f} expected={expected_rate:.3f}",
            )
        )
    else:
        checks.append(Check("playback rate", False, "insufficient scan samples"))
    return checks


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path, help="recorded localization rosbag directory")
    parser.add_argument("--expected-rate", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.expected_rate <= 0.0:
        print("--expected-rate must be positive", file=sys.stderr)
        return 2
    try:
        checks = validate(args.bag, args.expected_rate)
    except Exception as error:  # noqa: BLE001 - command-line validator
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    for check in checks:
        level = "WARN" if check.warning else ("PASS" if check.passed else "FAIL")
        print(f"[{level}] {check.name}: {check.detail}")
    failed = [check for check in checks if not check.passed and not check.warning]
    print(f"summary: {len(checks) - len(failed)}/{len(checks)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
