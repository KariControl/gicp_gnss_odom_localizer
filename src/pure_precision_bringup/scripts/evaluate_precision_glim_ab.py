#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Common-frame speed/isolated-precision A/B evaluation against GLIM.

One calibration transform fitted from the speed run is frozen and reused for
all local trajectories.  A second speed-EKF transform is frozen and reused for
all global trajectories.  Precision is therefore not allowed to choose a more
favourable coordinate alignment.  All association uses message header stamps.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import asdict
import importlib.util
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


RAW = "/localization/gyro_lidar_odom"
EXISTING_GLOBAL = "/localization/ekf_odom"
PRECISION_LOCAL = "/localization/precision_local_odom"
PRECISION_GLOBAL = "/localization/precision_global_odom"
SCAN = "/localization/submap_scan"
CORRECTION = "/localization/submap_correction"
GNSS = "/localization/gnss_fusion_input"
DIAGNOSTICS = "/diagnostics"
FUSION_DIAGNOSTIC = "localization/gnss_map_odom_fusion"
PRECISION_DIAGNOSTIC = "localization/precision_global_localizer"
RPE_DISTANCES = (10.0, 50.0, 100.0)


def load_canonical(repo: Path) -> Any:
    path = repo / "tools" / "evaluate_glim_trajectory.py"
    spec = importlib.util.spec_from_file_location("precision_glim_canonical", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load canonical evaluator: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def import_ros():
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as error:
        raise RuntimeError("source ROS 2 and this workspace before evaluation") from error
    return rosbag2_py, deserialize_message, get_message


def resolve_bag(path: Path) -> Path:
    path = path.expanduser().resolve()
    if path.is_file() and path.suffix == ".mcap":
        return path
    if path.is_dir() and (path / "metadata.yaml").is_file():
        return path
    nested = path / "localization_output"
    if nested.is_dir() and (nested / "metadata.yaml").is_file():
        return nested
    files = sorted(path.glob("*.mcap")) if path.is_dir() else []
    if len(files) == 1:
        return files[0]
    raise RuntimeError(f"cannot resolve one rosbag from {path}")


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def integer_value(value: Any) -> int:
    """Normalize ROS uint8 fields, which some Python generators expose as bytes."""
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise RuntimeError(f"expected one-byte integer field, got {value!r}")
        return value[0]
    return int(value)


def yaw_of(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (float(quaternion.w) * float(quaternion.z)
               + float(quaternion.x) * float(quaternion.y)),
        1.0 - 2.0 * (float(quaternion.y) ** 2 + float(quaternion.z) ** 2),
    )


def wrap(value: np.ndarray | float) -> np.ndarray | float:
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def open_reader(path: Path):
    rosbag2_py, _, _ = import_ros()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(resolve_bag(path)), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def make_trajectory(canonical: Any, groups: dict[int, list[tuple[int, float, float, float]]]):
    if not groups:
        raise RuntimeError("empty trajectory")
    stamps = np.asarray(sorted(groups), dtype=np.int64)
    chosen = [groups[int(key)][-1] for key in stamps]
    values = np.asarray([[item[1], item[2], item[3]] for item in chosen], dtype=float)
    return canonical.Trajectory(stamps, values[:, :2], values[:, 2])


def read_bag(path: Path, canonical: Any, precision: bool) -> dict[str, Any]:
    _, deserialize_message, get_message = import_ros()
    reader = open_reader(path)
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = {RAW, EXISTING_GLOBAL}
    if precision:
        required |= {PRECISION_LOCAL, PRECISION_GLOBAL, SCAN, CORRECTION}
    missing = sorted(required - set(types))
    if missing:
        raise RuntimeError(f"{path} is missing required topics: {missing}")
    wanted = required | {DIAGNOSTICS, GNSS}
    classes = {topic: get_message(types[topic]) for topic in wanted if topic in types}
    pose_topics = {RAW, EXISTING_GLOBAL, PRECISION_LOCAL, PRECISION_GLOBAL}
    groups: dict[str, dict[int, list[tuple[int, float, float, float]]]] = {
        topic: defaultdict(list) for topic in pose_topics if topic in types
    }
    counts: Counter[str] = Counter()
    scans: list[dict[str, Any]] = []
    corrections: list[tuple[int, int, int, int]] = []
    gnss: list[dict[str, Any]] = []
    fusion_states: list[tuple[int, str]] = []
    fusion_diagnostics: list[dict[str, Any]] = []
    precision_diagnostics: list[dict[str, Any]] = []
    duplicate_counts: dict[str, int] = defaultdict(int)
    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        counts[topic] += 1
        if topic not in classes:
            continue
        message = deserialize_message(serialized, classes[topic])
        if topic in groups:
            key = stamp_ns(message.header.stamp)
            if key <= 0:
                continue
            pose = message.pose.pose
            value = (
                int(record_ns),
                float(pose.position.x),
                float(pose.position.y),
                yaw_of(pose.orientation),
            )
            if not np.all(np.isfinite(value[1:])):
                raise RuntimeError(f"non-finite pose on {topic} at {key}")
            duplicate_counts[topic] += int(key in groups[topic])
            groups[topic][key].append(value)
        elif topic == SCAN:
            pose = message.raw_pose.pose
            scans.append(
                {
                    "key": (
                        int(message.odom_session_id),
                        int(message.odom_generation),
                        int(message.sequence),
                        stamp_ns(message.header.stamp),
                    ),
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "yaw": yaw_of(pose.orientation),
                }
            )
        elif topic == CORRECTION:
            corrections.append(
                (
                    int(message.odom_session_id),
                    int(message.odom_generation),
                    int(message.sequence),
                    stamp_ns(message.header.stamp),
                )
            )
        elif topic == GNSS:
            gnss.append(
                {
                    "stamp_ns": stamp_ns(message.header.stamp),
                    "fix_quality": int(message.fix_quality),
                    "usable": bool(message.gnss_usable and message.has_odom),
                    "heading_valid": bool(message.heading_valid),
                }
            )
        elif topic == DIAGNOSTICS:
            diagnostic_stamp = stamp_ns(message.header.stamp)
            for status in message.status:
                if status.name == FUSION_DIAGNOSTIC or status.name.endswith(
                    "/" + FUSION_DIAGNOSTIC
                ):
                    values = {item.key: item.value for item in status.values}
                    state = values.get("recovery.state")
                    if diagnostic_stamp > 0 and state:
                        fusion_states.append((diagnostic_stamp, state))
                        fusion_diagnostics.append(
                            {
                                "stamp_ns": diagnostic_stamp,
                                "level": integer_value(status.level),
                                "message": str(status.message),
                                "values": values,
                            }
                        )
                if status.name == PRECISION_DIAGNOSTIC or status.name.endswith(
                    "/" + PRECISION_DIAGNOSTIC
                ):
                    values = {item.key: item.value for item in status.values}
                    if diagnostic_stamp > 0 and values:
                        precision_diagnostics.append(
                            {
                                "stamp_ns": diagnostic_stamp,
                                "level": integer_value(status.level),
                                "message": str(status.message),
                                "values": values,
                            }
                        )
    trajectories = {topic: make_trajectory(canonical, group) for topic, group in groups.items()}
    return {
        "path": str(resolve_bag(path)),
        "trajectories": trajectories,
        "counts": dict(counts),
        "duplicates": dict(duplicate_counts),
        "scans": scans,
        "corrections": corrections,
        "gnss": gnss,
        "fusion_states": fusion_states,
        "fusion_diagnostics": fusion_diagnostics,
        "precision_diagnostics": precision_diagnostics,
    }


def circular_offset(estimate: np.ndarray, reference: np.ndarray, mask: np.ndarray) -> float:
    delta = wrap(reference[mask] - estimate[mask])
    return math.atan2(float(np.mean(np.sin(delta))), float(np.mean(np.cos(delta))))


def common_fixed_metrics(
    canonical: Any,
    trajectory: Any,
    reference: Any,
    alignment: Any,
    yaw_offset: float,
) -> dict[str, Any]:
    aligned = canonical.apply_alignment(trajectory, alignment)
    xy_error = np.linalg.norm(aligned.xy - reference.xy, axis=1)
    yaw_error = np.degrees(np.abs(wrap(trajectory.yaw + yaw_offset - reference.yaw)))
    full, _, _, _ = canonical.evaluate_aligned(
        trajectory, reference, np.ones(len(reference.stamp_ns), dtype=bool)
    )
    return {
        "samples": len(reference.stamp_ns),
        "fixed_common_xy_error_m": asdict(canonical.stats(xy_error)),
        "common_yaw_offset_error_deg": asdict(canonical.stats(yaw_error)),
        "full_shape_independent_se2": full,
        "rpe": {
            str(int(distance)): canonical.relative_pose_errors(
                trajectory, reference, distance
            )
            for distance in RPE_DISTANCES
        },
    }


def comparison(old: float, new: float) -> dict[str, float]:
    return {
        "old": float(old),
        "new": float(new),
        "change_percent": (new / old - 1.0) * 100.0 if old else math.inf,
        "improvement_percent": (1.0 - new / old) * 100.0 if old else -math.inf,
    }


def compare_evaluations(old: dict[str, Any], new: dict[str, Any]) -> dict[str, Any]:
    result = {
        "fixed_xy_rmse_m": comparison(
            old["fixed_common_xy_error_m"]["rmse"],
            new["fixed_common_xy_error_m"]["rmse"],
        ),
        "yaw_rmse_deg": comparison(
            old["common_yaw_offset_error_deg"]["rmse"],
            new["common_yaw_offset_error_deg"]["rmse"],
        ),
        "full_shape_xy_rmse_m": comparison(
            old["full_shape_independent_se2"]["position_error_m"]["rmse"],
            new["full_shape_independent_se2"]["position_error_m"]["rmse"],
        ),
        "full_shape_yaw_rmse_deg": comparison(
            old["full_shape_independent_se2"]["yaw_error_deg"]["rmse"],
            new["full_shape_independent_se2"]["yaw_error_deg"]["rmse"],
        ),
        "rpe": {},
    }
    for distance in ("10", "50", "100"):
        result["rpe"][distance] = {
            "translation_rmse_m": comparison(
                old["rpe"][distance]["translation_error_m"]["rmse"],
                new["rpe"][distance]["translation_error_m"]["rmse"],
            ),
            "yaw_rmse_deg": comparison(
                old["rpe"][distance]["yaw_error_deg"]["rmse"],
                new["rpe"][distance]["yaw_error_deg"]["rmse"],
            ),
        }
    return result


def direct_ab(canonical: Any, old: Any, new: Any) -> dict[str, Any]:
    reference, values = canonical.common_samples(old, {"new": new}, 0.1)
    estimate = values["new"]
    xy = np.linalg.norm(estimate.xy - reference.xy, axis=1)
    yaw = np.degrees(np.abs(wrap(estimate.yaw - reference.yaw)))
    old_delta_xy = np.diff(reference.xy, axis=0)
    new_delta_xy = np.diff(estimate.xy, axis=0)
    old_delta_yaw = wrap(np.diff(reference.yaw))
    new_delta_yaw = wrap(np.diff(estimate.yaw))
    return {
        "samples": len(reference.stamp_ns),
        "xy_difference_m": asdict(canonical.stats(xy)),
        "yaw_difference_deg": asdict(canonical.stats(yaw)),
        "translation_increment_difference_m": asdict(
            canonical.stats(np.linalg.norm(new_delta_xy - old_delta_xy, axis=1))
        ),
        "yaw_increment_difference_deg": asdict(
            canonical.stats(np.degrees(np.abs(wrap(new_delta_yaw - old_delta_yaw))))
        ),
    }


def snapshot_raw_ab(canonical: Any, speed_raw: Any, scans: list[dict[str, Any]]) -> dict[str, Any]:
    if len(scans) < 3:
        raise RuntimeError("fewer than three physical SubmapScan snapshots")
    ordered = sorted(scans, key=lambda item: item["key"][3])
    stamps = np.asarray([item["key"][3] for item in ordered], dtype=np.int64)
    values = np.asarray([[item["x"], item["y"], item["yaw"]] for item in ordered])
    snapshot = canonical.Trajectory(stamps, values[:, :2], values[:, 2])
    speed_at_scan, valid = canonical.interpolate_trajectory(speed_raw, stamps, 0.1)
    snapshot = snapshot.subset(valid)
    xy = np.linalg.norm(snapshot.xy - speed_at_scan.xy, axis=1)
    yaw = np.degrees(np.abs(wrap(snapshot.yaw - speed_at_scan.yaw)))
    snapshot_delta_xy = np.diff(snapshot.xy, axis=0)
    speed_delta_xy = np.diff(speed_at_scan.xy, axis=0)
    snapshot_delta_yaw = wrap(np.diff(snapshot.yaw))
    speed_delta_yaw = wrap(np.diff(speed_at_scan.yaw))
    return {
        "samples": len(snapshot.stamp_ns),
        "requested_samples": len(stamps),
        "coverage_ratio": len(snapshot.stamp_ns) / len(stamps),
        "xy_difference_m": asdict(canonical.stats(xy)),
        "yaw_difference_deg": asdict(canonical.stats(yaw)),
        "translation_increment_difference_m": asdict(
            canonical.stats(np.linalg.norm(snapshot_delta_xy - speed_delta_xy, axis=1))
        ),
        "yaw_increment_difference_deg": asdict(
            canonical.stats(
                np.degrees(np.abs(wrap(snapshot_delta_yaw - speed_delta_yaw)))
            )
        ),
    }


def outage_intervals(states: list[tuple[int, str]]) -> list[tuple[int, int]]:
    states = sorted(set(states))
    intervals: list[tuple[int, int]] = []
    seen_tracking = False
    start: int | None = None
    last_stamp: int | None = None
    for stamp, state in states:
        if last_stamp is not None and stamp - last_stamp > 3_000_000_000:
            if start is not None:
                intervals.append((start, last_stamp))
            start = None
        tracking = normalized_state(state) == "TRACKING"
        seen_tracking = seen_tracking or tracking
        if seen_tracking and not tracking and start is None:
            start = stamp
        if tracking and start is not None:
            intervals.append((start, stamp))
            start = None
        last_stamp = stamp
    if start is not None and last_stamp is not None:
        intervals.append((start, last_stamp))
    return [(begin, end) for begin, end in intervals if end - begin >= 1_000_000_000]


def longest_intersection(
    first: list[tuple[int, int]], second: list[tuple[int, int]]
) -> tuple[int, int] | None:
    candidates = []
    for a0, a1 in first:
        for b0, b1 in second:
            begin, end = max(a0, b0), min(a1, b1)
            if end > begin:
                candidates.append((begin, end))
    return max(candidates, key=lambda item: item[1] - item[0]) if candidates else None


def true_value(values: dict[str, str], key: str) -> bool:
    return values.get(key, "false").lower() == "true"


def normalized_state(value: Any) -> str:
    return str(value).strip().upper()


def q4_outages(gnss: list[dict[str, Any]], timeout_sec: float = 2.0) -> list[tuple[int, int]]:
    usable = sorted(
        item["stamp_ns"]
        for item in gnss
        if item["stamp_ns"] > 0 and item["usable"] and item["fix_quality"] == 4
    )
    timeout_ns = int(round(timeout_sec * 1.0e9))
    return [
        (first + timeout_ns, second)
        for first, second in zip(usable, usable[1:])
        if second - first > 2 * timeout_ns
    ]


def diagnostic_recovery_delay(
    diagnostics: list[dict[str, Any]], return_stamp: int, legacy: bool
) -> float | None:
    tracking_states = (
        {"TRACKING"}
        if legacy else {"TRACKING", "TRACKING_XY_ONLY", "TRACKING_SE2"}
    )
    for item in diagnostics:
        if item["stamp_ns"] < return_stamp:
            continue
        values = item["values"]
        state_key = "recovery.state" if legacy else "state"
        if normalized_state(values.get(state_key, "")) in tracking_states and (
            legacy or true_value(values, "position_fused")
        ):
            return (item["stamp_ns"] - return_stamp) * 1.0e-9
    return None


def legacy_timeline_ab(
    speed: list[tuple[int, str]], precision: list[tuple[int, str]]
) -> dict[str, Any]:
    if not speed or not precision:
        return {"samples": 0, "coverage_ratio": 0.0, "mismatch_ratio": 1.0}
    precision_stamps = np.asarray([item[0] for item in precision], dtype=np.int64)
    compared = 0
    mismatch = 0
    maximum_stamp_delta_sec = 0.0
    for stamp, state in speed:
        index = int(np.searchsorted(precision_stamps, stamp, side="left"))
        candidates = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(precision)]
        if not candidates:
            continue
        nearest = min(candidates, key=lambda candidate: abs(precision[candidate][0] - stamp))
        delta = abs(precision[nearest][0] - stamp)
        if delta > 1_500_000_000:
            continue
        compared += 1
        mismatch += int(precision[nearest][1] != state)
        maximum_stamp_delta_sec = max(maximum_stamp_delta_sec, delta * 1.0e-9)
    return {
        "samples": compared,
        "speed_samples": len(speed),
        "precision_samples": len(precision),
        "coverage_ratio": compared / len(speed),
        "mismatch_count": mismatch,
        "mismatch_ratio": mismatch / compared if compared else 1.0,
        "maximum_stamp_delta_sec": maximum_stamp_delta_sec,
    }


def precision_timeline_summary(data: dict[str, Any]) -> dict[str, Any]:
    diagnostics = data["precision_diagnostics"]
    raw_start = int(data["trajectories"][RAW].stamp_ns[0])
    initialized = next(
        (
            item
            for item in diagnostics
            if true_value(item["values"], "anchor.initialized")
        ),
        None,
    )
    final = diagnostics[-1] if diagnostics else None
    outages = q4_outages(data["gnss"])
    longest = max(outages, key=lambda value: value[1] - value[0]) if outages else None
    frozen_samples: list[dict[str, Any]] = []
    if longest is not None:
        frozen_samples = [
            item
            for item in diagnostics
            if longest[0] <= item["stamp_ns"] < longest[1]
            and item["values"].get("state") in ("OUTAGE", "FROZEN")
        ]
    freeze_keys = (
        "anchor.target.x_m",
        "anchor.target.y_m",
        "anchor.target.yaw_rad",
        "anchor.applied.x_m",
        "anchor.applied.y_m",
        "anchor.applied.yaw_rad",
    )
    freeze_ranges: dict[str, float] = {}
    freeze_exact: dict[str, bool] = {}
    for key in freeze_keys:
        serialized = [item["values"].get(key) for item in frozen_samples]
        freeze_exact[key] = (
            len(serialized) >= 2
            and all(value is not None for value in serialized)
            and len(set(serialized)) == 1
        )
        values = [float(value if value is not None else "nan") for value in serialized]
        finite = [value for value in values if math.isfinite(value)]
        freeze_ranges[key] = max(finite) - min(finite) if len(finite) >= 2 else math.inf
    recovery_delay = (
        diagnostic_recovery_delay(diagnostics, longest[1], legacy=False)
        if longest is not None else None
    )
    return {
        "diagnostic_samples": len(diagnostics),
        "initialization_delay_sec": (
            (initialized["stamp_ns"] - raw_start) * 1.0e-9
            if initialized is not None else None
        ),
        "final_state": final["values"].get("state") if final else None,
        "final_anchor_initialized": (
            true_value(final["values"], "anchor.initialized") if final else False
        ),
        "final_position_fused": (
            true_value(final["values"], "position_fused") if final else False
        ),
        "final_anchor_source": (
            final["values"].get("anchor.source") if final else None
        ),
        "final_fallback_gnss_position_enabled": (
            true_value(final["values"], "fallback.gnss_position_enabled")
            if final else True
        ),
        "q4_outages": [[begin * 1.0e-9, end * 1.0e-9] for begin, end in outages],
        "longest_q4_outage": (
            {
                "begin_ns": longest[0],
                "end_ns": longest[1],
                "duration_sec": (longest[1] - longest[0]) * 1.0e-9,
                "outage_diagnostic_samples": len(frozen_samples),
                "anchor_ranges": freeze_ranges,
                "anchor_serialization_exact": freeze_exact,
                "recovery_delay_sec": recovery_delay,
            }
            if longest is not None else None
        ),
    }


def interval_stats(
    canonical: Any,
    evaluation: dict[str, Any],
    trajectory: Any,
    reference: Any,
    alignment: Any,
    yaw_offset: float,
    interval: tuple[int, int],
) -> dict[str, Any] | None:
    mask = (reference.stamp_ns >= interval[0]) & (reference.stamp_ns <= interval[1])
    if np.count_nonzero(mask) < 3:
        return None
    selected_reference = reference.subset(mask)
    selected_trajectory = trajectory.subset(mask)
    aligned = canonical.apply_alignment(selected_trajectory, alignment)
    xy = np.linalg.norm(aligned.xy - selected_reference.xy, axis=1)
    yaw = np.degrees(
        np.abs(wrap(selected_trajectory.yaw + yaw_offset - selected_reference.yaw))
    )
    return {
        "samples": int(np.count_nonzero(mask)),
        "xy_error_m": asdict(canonical.stats(xy)),
        "yaw_error_deg": asdict(canonical.stats(yaw)),
    }


def nonregression(old: float, new: float, absolute: float) -> tuple[bool, str]:
    limit = old * 1.05 + absolute
    return new <= limit, f"{new:.6f} <= {limit:.6f} (old={old:.6f})"


def protocol_summary(data: dict[str, Any]) -> dict[str, Any]:
    scans = data["scans"]
    corrections = data["corrections"]
    scan_keys = [item["key"] for item in scans]
    scan_set = set(scan_keys)
    correction_set = set(corrections)
    stamps = [key[3] for key in scan_keys]
    first_correction = min((key[3] for key in corrections), default=None)
    post_warmup_scans = (
        sum(stamp >= first_correction for stamp in stamps)
        if first_correction is not None else 0
    )
    return {
        "scan_count": len(scans),
        "correction_count": len(corrections),
        "duplicate_scan_keys": len(scan_keys) - len(scan_set),
        "duplicate_correction_keys": len(corrections) - len(correction_set),
        "unknown_correction_keys": len(correction_set - scan_set),
        "strict_physical_scan_stamps": bool(stamps)
        and stamps[0] > 0
        and all(b > a for a, b in zip(stamps, stamps[1:])),
        "warmup_sec": (
            (first_correction - stamps[0]) * 1.0e-9
            if first_correction is not None and stamps else math.inf
        ),
        "post_warmup_correction_ratio": (
            len(corrections) / post_warmup_scans if post_warmup_scans else 0.0
        ),
    }


def evaluate_pair(args: argparse.Namespace) -> dict[str, Any]:
    canonical = load_canonical(args.repo.resolve())
    reference = canonical.read_glim_trajectory(
        args.glim_dir.resolve() / args.glim_trajectory
    )
    speed = read_bag(args.speed_bag, canonical, precision=False)
    precision = read_bag(args.precision_bag, canonical, precision=True)

    local_reference, local = canonical.common_samples(
        reference,
        {
            "speed_raw": speed["trajectories"][RAW],
            "precision_raw": precision["trajectories"][RAW],
            "precision_local": precision["trajectories"][PRECISION_LOCAL],
        },
        args.maximum_interpolation_gap_sec,
    )
    global_reference, global_trajectories = canonical.common_samples(
        reference,
        {
            "speed_existing": speed["trajectories"][EXISTING_GLOBAL],
            "precision_existing": precision["trajectories"][EXISTING_GLOBAL],
            "precision_global": precision["trajectories"][PRECISION_GLOBAL],
        },
        args.maximum_interpolation_gap_sec,
    )

    start_ns = int(round(args.calibration_start * 1.0e9))
    end_ns = int(round(args.calibration_end * 1.0e9))
    local_mask = (local_reference.stamp_ns >= start_ns) & (local_reference.stamp_ns <= end_ns)
    global_mask = (
        (global_reference.stamp_ns >= start_ns) & (global_reference.stamp_ns <= end_ns)
    )
    if np.count_nonzero(local_mask) < 3 or np.count_nonzero(global_mask) < 3:
        raise RuntimeError("calibration window has fewer than three common samples")

    local_alignment = canonical.fit_se2(
        local["speed_raw"].xy[local_mask], local_reference.xy[local_mask]
    )
    local_yaw_offset = circular_offset(
        local["speed_raw"].yaw, local_reference.yaw, local_mask
    )
    global_alignment = canonical.fit_se2(
        global_trajectories["speed_existing"].xy[global_mask],
        global_reference.xy[global_mask],
    )
    global_yaw_offset = circular_offset(
        global_trajectories["speed_existing"].yaw, global_reference.yaw, global_mask
    )

    evaluations = {
        name: common_fixed_metrics(
            canonical, trajectory, local_reference, local_alignment, local_yaw_offset
        )
        for name, trajectory in local.items()
    }
    evaluations.update(
        {
            name: common_fixed_metrics(
                canonical,
                trajectory,
                global_reference,
                global_alignment,
                global_yaw_offset,
            )
            for name, trajectory in global_trajectories.items()
        }
    )

    common_outage = longest_intersection(
        outage_intervals(speed["fusion_states"]),
        outage_intervals(precision["fusion_states"]),
    )
    outage: dict[str, Any] | None = None
    if common_outage is not None:
        outage = {
            "start_sec": common_outage[0] * 1.0e-9,
            "end_sec": common_outage[1] * 1.0e-9,
            "duration_sec": (common_outage[1] - common_outage[0]) * 1.0e-9,
        }
        for name, trajectory in local.items():
            outage[name] = interval_stats(
                canonical,
                evaluations[name],
                trajectory,
                local_reference,
                local_alignment,
                local_yaw_offset,
                common_outage,
            )
        for name, trajectory in global_trajectories.items():
            outage[name] = interval_stats(
                canonical,
                evaluations[name],
                trajectory,
                global_reference,
                global_alignment,
                global_yaw_offset,
                common_outage,
            )

    raw_timer_ab = direct_ab(
        canonical, speed["trajectories"][RAW], precision["trajectories"][RAW]
    )
    raw_snapshot_ab = snapshot_raw_ab(
        canonical, speed["trajectories"][RAW], precision["scans"]
    )
    existing_global_timer_ab = direct_ab(
        canonical,
        speed["trajectories"][EXISTING_GLOBAL],
        precision["trajectories"][EXISTING_GLOBAL],
    )
    local_comparison = compare_evaluations(
        evaluations["precision_raw"], evaluations["precision_local"]
    )
    global_comparison = compare_evaluations(
        evaluations["precision_existing"], evaluations["precision_global"]
    )
    existing_global_glim_comparison = compare_evaluations(
        evaluations["speed_existing"], evaluations["precision_existing"]
    )
    protocol = protocol_summary(precision)
    precision_timeline = precision_timeline_summary(precision)
    legacy_timeline = legacy_timeline_ab(
        speed["fusion_states"], precision["fusion_states"]
    )
    speed_fusion_final = (
        speed["fusion_diagnostics"][-1] if speed["fusion_diagnostics"] else None
    )
    precision_fusion_final = (
        precision["fusion_diagnostics"][-1]
        if precision["fusion_diagnostics"] else None
    )

    checks: list[dict[str, Any]] = []

    def add(name: str, passed: bool, detail: str, category: str = "hard") -> None:
        checks.append(
            {"name": name, "passed": bool(passed), "detail": detail, "category": category}
        )

    # A SubmapScan pose is captured at the accepted registration event while
    # RAW is published by a separate timer.  Cross-run interpolation between
    # those two timestamp semantics produced the former false hard failures
    # (~100 ms phase).  Keep this only as reference; the hard gate is the
    # separate exact accepted-scan control/precision evaluator.
    add(
        "physical-event to speed-timer coverage (reference only)",
        raw_snapshot_ab["coverage_ratio"] >= 0.99,
        f"{raw_snapshot_ab['samples']}/{raw_snapshot_ab['requested_samples']} "
        f"({raw_snapshot_ab['coverage_ratio']:.6f})",
        "warn",
    )
    add(
        "physical-event to speed-timer XY (reference only)",
        raw_snapshot_ab["xy_difference_m"]["rmse"] <= 0.05,
        f"{raw_snapshot_ab['xy_difference_m']['rmse']:.6f} <= 0.05 m",
        "warn",
    )
    add(
        "physical-event to speed-timer yaw (reference only)",
        raw_snapshot_ab["yaw_difference_deg"]["rmse"] <= 0.10,
        f"{raw_snapshot_ab['yaw_difference_deg']['rmse']:.6f} <= 0.10 deg",
        "warn",
    )
    add(
        "physical-event to speed-timer increment (reference only)",
        raw_snapshot_ab["translation_increment_difference_m"]["rmse"] <= 0.02
        and raw_snapshot_ab["yaw_increment_difference_deg"]["rmse"] <= 0.05,
        f"translation={raw_snapshot_ab['translation_increment_difference_m']['rmse']:.6f}m "
        f"yaw={raw_snapshot_ab['yaw_increment_difference_deg']['rmse']:.6f}deg",
        "warn",
    )
    add(
        "50Hz raw timer direct difference (reference only)",
        raw_timer_ab["xy_difference_m"]["rmse"] <= 0.05
        and raw_timer_ab["yaw_difference_deg"]["rmse"] <= 0.10,
        f"xy={raw_timer_ab['xy_difference_m']['rmse']:.6f}m "
        f"yaw={raw_timer_ab['yaw_difference_deg']['rmse']:.6f}deg",
        "warn",
    )
    add(
        "50Hz existing fusion timer direct difference (reference only)",
        existing_global_timer_ab["xy_difference_m"]["rmse"] <= 0.05
        and existing_global_timer_ab["yaw_difference_deg"]["rmse"] <= 0.10,
        f"xy={existing_global_timer_ab['xy_difference_m']['rmse']:.6f}m "
        f"yaw={existing_global_timer_ab['yaw_difference_deg']['rmse']:.6f}deg",
        "warn",
    )
    add(
        "legacy fusion final diagnostics present",
        speed_fusion_final is not None and precision_fusion_final is not None,
        f"speed={speed_fusion_final is not None} precision="
        f"{precision_fusion_final is not None}",
    )
    add(
        "legacy fusion recovery timeline non-regression",
        legacy_timeline["coverage_ratio"] >= 0.95
        and legacy_timeline["mismatch_ratio"] <= 0.05,
        json.dumps(legacy_timeline, sort_keys=True),
    )
    if speed_fusion_final is not None and precision_fusion_final is not None:
        speed_values = speed_fusion_final["values"]
        precision_values = precision_fusion_final["values"]
        speed_state = speed_values.get("recovery.state", "")
        precision_state = precision_values.get("recovery.state", "")
        speed_anchor = speed_values.get("anchor_valid", "false")
        precision_anchor = precision_values.get("anchor_valid", "false")
        speed_position_fused = speed_values.get("recovery.position_fused", "false")
        precision_position_fused = precision_values.get(
            "recovery.position_fused", "false"
        )
        speed_yaw_fused = speed_values.get("recovery.yaw_fused", "false")
        precision_yaw_fused = precision_values.get("recovery.yaw_fused", "false")
        speed_fix = speed_values.get("last_fix_state", "")
        precision_fix = precision_values.get("last_fix_state", "")
        speed_drop = int(float(speed_values.get("output.out_of_order_drop_count", "0")))
        precision_drop = int(
            float(precision_values.get("output.out_of_order_drop_count", "0"))
        )
        speed_exit = int(float(speed_values.get("recovery.exit_count", "0")))
        precision_exit = int(float(precision_values.get("recovery.exit_count", "0")))
        add(
            "legacy fusion final recovery state unchanged",
            precision_state == speed_state,
            f"speed={speed_state} precision={precision_state}",
        )
        add(
            "legacy fusion final health non-regression",
            precision_fusion_final["level"] <= speed_fusion_final["level"]
            and precision_anchor == speed_anchor
            and precision_position_fused == speed_position_fused
            and precision_yaw_fused == speed_yaw_fused
            and precision_fix == speed_fix
            and precision_drop <= speed_drop
            and precision_exit == speed_exit,
            f"level={speed_fusion_final['level']}->{precision_fusion_final['level']} "
            f"anchor={speed_anchor}->{precision_anchor} drop={speed_drop}->"
            f"{precision_drop} exits={speed_exit}->{precision_exit} "
            f"position_fused={speed_position_fused}->{precision_position_fused} "
            f"yaw_fused={speed_yaw_fused}->{precision_yaw_fused} "
            f"fix={speed_fix}->{precision_fix}",
        )
    for name, item, absolute in (
        ("fixed-frame XY", existing_global_glim_comparison["fixed_xy_rmse_m"], 0.05),
        ("fixed-frame yaw", existing_global_glim_comparison["yaw_rmse_deg"], 0.05),
        (
            "full-shape independent XY",
            existing_global_glim_comparison["full_shape_xy_rmse_m"],
            0.05,
        ),
        (
            "full-shape independent yaw",
            existing_global_glim_comparison["full_shape_yaw_rmse_deg"],
            0.05,
        ),
    ):
        passed, detail = nonregression(item["old"], item["new"], absolute)
        add(f"existing fusion GLIM {name} non-regression", passed, detail)
    for distance, item in existing_global_glim_comparison["rpe"].items():
        passed_translation, translation_detail = nonregression(
            item["translation_rmse_m"]["old"],
            item["translation_rmse_m"]["new"],
            0.02,
        )
        passed_yaw, yaw_detail = nonregression(
            item["yaw_rmse_deg"]["old"], item["yaw_rmse_deg"]["new"], 0.05
        )
        add(
            f"existing fusion GLIM {distance}m RPE non-regression",
            passed_translation and passed_yaw,
            f"translation: {translation_detail}; yaw: {yaw_detail}",
        )
    add(
        "precision local fixed-frame XY improves >=20%",
        local_comparison["fixed_xy_rmse_m"]["improvement_percent"] >= 20.0,
        f"{local_comparison['fixed_xy_rmse_m']['improvement_percent']:.3f}%",
    )
    add(
        "precision local full-shape XY improves >=20%",
        local_comparison["full_shape_xy_rmse_m"]["improvement_percent"] >= 20.0,
        f"{local_comparison['full_shape_xy_rmse_m']['improvement_percent']:.3f}%",
    )
    passed, detail = nonregression(
        local_comparison["yaw_rmse_deg"]["old"],
        local_comparison["yaw_rmse_deg"]["new"],
        0.05,
    )
    add("precision local yaw non-regression", passed, detail)
    passed, detail = nonregression(
        local_comparison["full_shape_yaw_rmse_deg"]["old"],
        local_comparison["full_shape_yaw_rmse_deg"]["new"],
        0.05,
    )
    add("precision local full-shape yaw non-regression", passed, detail)
    for distance, item in local_comparison["rpe"].items():
        passed, detail = nonregression(
            item["translation_rmse_m"]["old"],
            item["translation_rmse_m"]["new"],
            0.02,
        )
        add(f"precision local {distance}m translation RPE non-regression", passed, detail)
        passed, detail = nonregression(
            item["yaw_rmse_deg"]["old"], item["yaw_rmse_deg"]["new"], 0.05
        )
        add(f"precision local {distance}m yaw RPE non-regression", passed, detail)
    passed, detail = nonregression(
        global_comparison["fixed_xy_rmse_m"]["old"],
        global_comparison["fixed_xy_rmse_m"]["new"],
        0.05,
    )
    add("precision global fixed-frame XY non-regression", passed, detail)
    passed, detail = nonregression(
        global_comparison["full_shape_xy_rmse_m"]["old"],
        global_comparison["full_shape_xy_rmse_m"]["new"],
        0.05,
    )
    add("precision global full-shape XY non-regression", passed, detail)
    passed, detail = nonregression(
        global_comparison["yaw_rmse_deg"]["old"],
        global_comparison["yaw_rmse_deg"]["new"],
        0.05,
    )
    add("precision global yaw non-regression", passed, detail)
    passed, detail = nonregression(
        global_comparison["full_shape_yaw_rmse_deg"]["old"],
        global_comparison["full_shape_yaw_rmse_deg"]["new"],
        0.05,
    )
    add("precision global full-shape yaw non-regression", passed, detail)
    add("exact-key scans present", protocol["scan_count"] > 0, str(protocol["scan_count"]))
    add(
        "exact-key corrections valid",
        protocol["correction_count"] > 0
        and protocol["duplicate_scan_keys"] == 0
        and protocol["duplicate_correction_keys"] == 0
        and protocol["unknown_correction_keys"] == 0
        and protocol["strict_physical_scan_stamps"],
        json.dumps(protocol, sort_keys=True),
    )
    add(
        "initialization warmup bounded",
        0.0 <= protocol["warmup_sec"] <= 10.0,
        f"{protocol['warmup_sec']:.3f} sec",
    )
    initialization_delay = precision_timeline["initialization_delay_sec"]
    add(
        "precision global initializes within 20 sec",
        initialization_delay is not None and 0.0 <= initialization_delay <= 20.0,
        str(initialization_delay),
    )
    add(
        "precision global final position is fused",
        precision_timeline["final_anchor_initialized"]
        and precision_timeline["final_position_fused"]
        and precision_timeline["final_state"] not in (None, "", "UNINITIALIZED"),
        f"state={precision_timeline['final_state']} initialized="
        f"{precision_timeline['final_anchor_initialized']} position_fused="
        f"{precision_timeline['final_position_fused']}",
    )
    add(
        "precision global uses only existing-fusion authority",
        precision_timeline["final_anchor_source"] == "existing_fusion"
        and not precision_timeline["final_fallback_gnss_position_enabled"],
        f"source={precision_timeline['final_anchor_source']} fallback_gnss_position="
        f"{precision_timeline['final_fallback_gnss_position_enabled']}",
    )
    timeline_outage = precision_timeline["longest_q4_outage"]
    add(
        "precision global long q4 outage is observable",
        timeline_outage is not None,
        str(timeline_outage),
    )
    if timeline_outage is not None:
        anchor_ranges = timeline_outage["anchor_ranges"]
        anchor_exact = timeline_outage["anchor_serialization_exact"]
        add(
            "precision anchor target/applied freeze during outage",
            timeline_outage["outage_diagnostic_samples"] >= 2
            and all(anchor_exact.values())
            and all(value == 0.0 for value in anchor_ranges.values()),
            f"samples={timeline_outage['outage_diagnostic_samples']} "
            f"exact={anchor_exact} ranges={anchor_ranges}",
        )
        recovery_delay = timeline_outage["recovery_delay_sec"]
        add(
            "precision q4 return reaches finite XY tracking",
            recovery_delay is not None and 0.0 <= recovery_delay <= 10.0,
            str(recovery_delay),
        )
        if "pointcloud2" in args.label.lower():
            speed_legacy_recovery = diagnostic_recovery_delay(
                speed["fusion_diagnostics"], timeline_outage["end_ns"], legacy=True
            )
            precision_legacy_recovery = diagnostic_recovery_delay(
                precision["fusion_diagnostics"], timeline_outage["end_ns"], legacy=True
            )
            add(
                "PC2 legacy fusion TRACKING recovery retained in both modes",
                speed_legacy_recovery is not None
                and precision_legacy_recovery is not None
                and 0.0 <= speed_legacy_recovery <= 10.0
                and 0.0 <= precision_legacy_recovery <= 10.0,
                f"speed={speed_legacy_recovery} precision={precision_legacy_recovery}",
            )
    if outage is None:
        add("common GNSS outage available", False, "none")
    else:
        old_outage = outage.get("precision_existing")
        new_outage = outage.get("precision_global")
        add(
            "outage precision global samples available",
            old_outage is not None and new_outage is not None,
            f"duration={outage['duration_sec']:.3f} sec",
        )
        if old_outage is not None and new_outage is not None:
            passed, detail = nonregression(
                old_outage["xy_error_m"]["rmse"], new_outage["xy_error_m"]["rmse"], 0.05
            )
            add("outage precision global XY non-regression", passed, detail)
            passed, detail = nonregression(
                old_outage["yaw_error_deg"]["rmse"],
                new_outage["yaw_error_deg"]["rmse"],
                0.05,
            )
            add("outage precision global yaw non-regression", passed, detail)
            improvement = comparison(
                old_outage["xy_error_m"]["rmse"], new_outage["xy_error_m"]["rmse"]
            )["improvement_percent"]
            add(
                "outage precision global XY improves >=10% (desirable)",
                improvement >= 10.0,
                f"{improvement:.3f}%",
                "warn",
            )

    failed = [item for item in checks if item["category"] == "hard" and not item["passed"]]
    result = {
        "label": args.label,
        "passed": not failed,
        "method": {
            "timestamps": "physical ROS header stamps; MCAP record time is not pose time",
            "local_alignment": "one speed/raw calibration-window SE(2) frozen for all local outputs",
            "global_alignment": "one speed/existing-EKF calibration-window SE(2) frozen for all global outputs",
            "yaw_alignment": "separate speed baseline circular offset, frozen per local/global group",
            "scale": "not estimated or applied",
            "glim_caveat": "correlated LiDAR+IMU pseudo-ground-truth",
        },
        "inputs": {
            "speed_bag": speed["path"],
            "precision_bag": precision["path"],
            "glim": str((args.glim_dir / args.glim_trajectory).resolve()),
        },
        "initialization": {
            "precision_local_delay_from_raw_sec": (
                precision["trajectories"][PRECISION_LOCAL].stamp_ns[0]
                - precision["trajectories"][RAW].stamp_ns[0]
            ) * 1.0e-9,
            "local_common_samples": len(local_reference.stamp_ns),
            "global_common_samples": len(global_reference.stamp_ns),
        },
        "evaluations": evaluations,
        "local_comparison": local_comparison,
        "global_comparison": global_comparison,
        "existing_global_glim_comparison": existing_global_glim_comparison,
        "raw_physical_scan_ab": raw_snapshot_ab,
        "raw_timer_ab_reference": raw_timer_ab,
        "existing_global_timer_ab_reference": existing_global_timer_ab,
        "legacy_fusion_final": {
            "speed": speed_fusion_final,
            "precision": precision_fusion_final,
        },
        "legacy_fusion_timeline_ab": legacy_timeline,
        "precision_timeline": precision_timeline,
        "protocol": protocol,
        "outage": outage,
        "topic_counts": {"speed": speed["counts"], "precision": precision["counts"]},
        "checks": checks,
        "hard_gate_count": sum(item["category"] == "hard" for item in checks),
        "failed_hard_gate_count": len(failed),
    }
    return result


def format_change(item: dict[str, float], unit: str) -> str:
    return (
        f"{item['old']:.4f} -> {item['new']:.4f} {unit} "
        f"({item['improvement_percent']:+.2f}% improvement)"
    )


def write_markdown(path: Path, result: dict[str, Any]) -> None:
    local = result["local_comparison"]
    global_result = result["global_comparison"]
    lines = [
        f"# {result['label']} isolated precision A/B",
        "",
        f"- result: **{'PASS' if result['passed'] else 'FAIL'}**",
        f"- local fixed XY: {format_change(local['fixed_xy_rmse_m'], 'm')}",
        f"- local full-shape XY: {format_change(local['full_shape_xy_rmse_m'], 'm')}",
        f"- local yaw: {format_change(local['yaw_rmse_deg'], 'deg')}",
        f"- global fixed XY: {format_change(global_result['fixed_xy_rmse_m'], 'm')}",
        f"- global full-shape XY: {format_change(global_result['full_shape_xy_rmse_m'], 'm')}",
        f"- global yaw: {format_change(global_result['yaw_rmse_deg'], 'deg')}",
        "",
        "## RPE",
        "",
    ]
    for distance, item in local["rpe"].items():
        lines.append(
            f"- {distance} m: translation {format_change(item['translation_rmse_m'], 'm')}; "
            f"yaw {format_change(item['yaw_rmse_deg'], 'deg')}"
        )
    lines.extend(["", "## Acceptance", ""])
    for item in result["checks"]:
        mark = "PASS" if item["passed"] else ("WARN" if item["category"] == "warn" else "FAIL")
        lines.append(f"- {mark}: {item['name']} — {item['detail']}")
    lines.extend(
        [
            "",
            "GLIM and the evaluated odometry share LiDAR/IMU observations; this is a correlated pseudo-ground-truth comparison.",
            "",
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--speed-bag", type=Path, required=True)
    parser.add_argument("--precision-bag", type=Path, required=True)
    parser.add_argument("--glim-dir", type=Path, required=True)
    parser.add_argument("--glim-trajectory", default="traj_lidar.txt")
    parser.add_argument("--calibration-start", type=float, required=True)
    parser.add_argument("--calibration-end", type=float, required=True)
    parser.add_argument("--maximum-interpolation-gap-sec", type=float, default=0.1)
    parser.add_argument("--label", required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-markdown", type=Path, required=True)
    return parser


def main() -> int:
    args = make_parser().parse_args()
    if args.calibration_end <= args.calibration_start:
        raise SystemExit("calibration end must be later than start")
    result = evaluate_pair(args)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    write_markdown(args.output_markdown, result)
    for item in result["checks"]:
        if not item["passed"]:
            print(f"{item['category'].upper()} {item['name']}: {item['detail']}")
    print(
        f"{'PASS' if result['passed'] else 'FAIL'}: "
        f"hard={result['hard_gate_count'] - result['failed_hard_gate_count']}/"
        f"{result['hard_gate_count']}"
    )
    print(args.output_json.resolve())
    print(args.output_markdown.resolve())
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
