#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Common-frame speed/isolated-precision A/B evaluation against GLIM.

The primary exact-initial-pose mode compares correction-before/after streams
from the same precision run at exact common estimator header stamps.  GLIM is
interpolated to those physical stamps; estimator streams are never
interpolated.  One complete initial-pose SE(2) is frozen and shared by each A/B
group.  The former calibration-window method remains available for reproducing
historical results.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import asdict
import importlib.util
import json
import math
import os
from pathlib import Path
import re
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
ALIGNMENT_FROZEN_CALIBRATION = "frozen-calibration-window"
ALIGNMENT_EXACT_INITIAL_POSE = "exact-initial-pose"
EXACT_INITIAL_XY_TOLERANCE_M = 1.0e-9
EXACT_INITIAL_YAW_TOLERANCE_RAD = 1.0e-12
OUTAGE_YAW_GUARD_STATES = (
    "DISARMED",
    "READY",
    "OUTAGE_SLEW",
    "OUTAGE_HOLD",
    "RECOVERY_RELEASE",
)
OUTAGE_YAW_GUARD_ACTIVE_STATES = {
    "OUTAGE_SLEW",
    "OUTAGE_HOLD",
    "RECOVERY_RELEASE",
}
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
OUTAGE_YAW_GUARD_EXPECTED_CONFIG = {
    "max_trusted_age_sec": 2.0,
    "required_fix_quality": 4.0,
    "max_trusted_variance_rad2": 0.0225,
    "max_trusted_delta_rad": 0.35,
    "max_offset_rate_radps": 0.20,
    "max_offset_step_rad": 0.04,
    "max_step_dt_sec": 0.25,
}
OUTAGE_YAW_GUARD_EXPECTED_SEMANTICS = {
    "reference_source": "robust_gnss_position_alignment_yaw",
    "propagation_source": "precision_local_yaw",
    "xy_policy": "existing_fusion_anchor_compose_precision_local",
}
OUTAGE_YAW_GUARD_FLOAT_TOLERANCE = 1.0e-9
FUSION_PUBLICATION_COUNTER_KEYS = (
    "output.out_of_order_drop_count",
    "output.covered_odometry_coalesced_count",
    "output.wall_timer_coalesced_count",
    "output.total_suppressed_request_count",
)


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
    fusion_publication_diagnostics: list[dict[str, Any]] = []
    precision_diagnostics: list[dict[str, Any]] = []
    publication_topics = {RAW, EXISTING_GLOBAL}
    if precision:
        publication_topics.add(PRECISION_GLOBAL)
    publication_records: dict[str, list[dict[str, int]]] = {
        topic: [] for topic in publication_topics
    }
    duplicate_counts: dict[str, int] = defaultdict(int)
    record_index = 0
    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        record_index += 1
        counts[topic] += 1
        if topic not in classes:
            continue
        message = deserialize_message(serialized, classes[topic])
        if topic in groups:
            key = stamp_ns(message.header.stamp)
            if topic in publication_records:
                publication_records[topic].append(
                    {
                        "record_index": record_index,
                        "record_ns": int(record_ns),
                        "stamp_ns": key,
                    }
                )
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
            fusion_status_count = sum(
                status.name == FUSION_DIAGNOSTIC or status.name.endswith(
                    "/" + FUSION_DIAGNOSTIC
                )
                for status in message.status
            )
            for status in message.status:
                if status.name == FUSION_DIAGNOSTIC or status.name.endswith(
                    "/" + FUSION_DIAGNOSTIC
                ):
                    value_pairs = [(item.key, item.value) for item in status.values]
                    values = dict(value_pairs)
                    fusion_publication_diagnostics.append(
                        {
                            "record_index": record_index,
                            "record_ns": int(record_ns),
                            "stamp_ns": diagnostic_stamp,
                            "status_count": fusion_status_count,
                            "value_pairs": value_pairs,
                            "values": values,
                        }
                    )
                    state = values.get("recovery.state")
                    if diagnostic_stamp > 0 and state:
                        fusion_states.append((diagnostic_stamp, state))
                        fusion_diagnostics.append(
                            {
                                "stamp_ns": diagnostic_stamp,
                                "record_index": record_index,
                                "record_ns": int(record_ns),
                                "level": integer_value(status.level),
                                "message": str(status.message),
                                "value_pairs": value_pairs,
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
        "fusion_publication_diagnostics": fusion_publication_diagnostics,
        "precision_diagnostics": precision_diagnostics,
        "publication_records": publication_records,
    }


def circular_offset(estimate: np.ndarray, reference: np.ndarray, mask: np.ndarray) -> float:
    delta = wrap(reference[mask] - estimate[mask])
    return math.atan2(float(np.mean(np.sin(delta))), float(np.mean(np.cos(delta))))


def exact_pose_alignment(
    canonical: Any,
    estimate_xy: np.ndarray,
    estimate_yaw: float,
    reference_xy: np.ndarray,
    reference_yaw: float,
) -> Any:
    """Return the unique unit-scale SE(2) mapping one full pose to another."""
    yaw = float(wrap(float(reference_yaw) - float(estimate_yaw)))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]], dtype=float)
    translation = np.asarray(reference_xy, dtype=float) - rotation @ np.asarray(
        estimate_xy, dtype=float
    )
    return canonical.Alignment(
        rotation=rotation,
        translation=translation,
        yaw_rad=yaw,
        singular_values=np.asarray([], dtype=float),
        diagnostic_similarity_scale=math.nan,
    )


def exact_estimator_common_samples(
    canonical: Any,
    reference: Any,
    estimates: dict[str, Any],
    maximum_gap_sec: float,
) -> tuple[Any, dict[str, Any]]:
    """Associate same-run estimators exactly and interpolate only GLIM.

    The estimator intersection is integer nanosecond equality.  This prevents
    estimator interpolation from hiding timer phase or correction latency.
    """
    if not estimates:
        raise RuntimeError("exact-stamp association requires estimator streams")
    common_stamps: np.ndarray | None = None
    for name, trajectory in estimates.items():
        stamps = np.asarray(trajectory.stamp_ns, dtype=np.int64)
        if len(stamps) < 3 or np.any(np.diff(stamps) <= 0):
            raise RuntimeError(
                f"exact-stamp estimator {name} has fewer than three strictly "
                "increasing samples"
            )
        common_stamps = (
            stamps
            if common_stamps is None
            else np.intersect1d(common_stamps, stamps, assume_unique=True)
        )
    if common_stamps is None or len(common_stamps) < 3:
        raise RuntimeError("fewer than three exact common estimator header stamps")

    interpolated_reference, valid = canonical.interpolate_trajectory(
        reference, common_stamps, maximum_gap_sec
    )
    selected_stamps = common_stamps[valid]
    if len(selected_stamps) < 3:
        raise RuntimeError("fewer than three exact estimator stamps have GLIM coverage")
    if not np.array_equal(interpolated_reference.stamp_ns, selected_stamps):
        raise RuntimeError("GLIM interpolation changed exact estimator stamps")

    selected: dict[str, Any] = {}
    for name, trajectory in estimates.items():
        indices = np.searchsorted(trajectory.stamp_ns, selected_stamps)
        if np.any(indices >= len(trajectory.stamp_ns)) or not np.array_equal(
            trajectory.stamp_ns[indices], selected_stamps
        ):
            raise RuntimeError(f"lost exact estimator association for {name}")
        mask = np.zeros(len(trajectory.stamp_ns), dtype=bool)
        mask[indices] = True
        selected[name] = trajectory.subset(mask)
        if not np.array_equal(selected[name].stamp_ns, selected_stamps):
            raise RuntimeError(f"exact estimator stamps differ for {name}")
    return interpolated_reference, selected


def exact_initial_alignment_metadata(
    canonical: Any,
    reference: Any,
    trajectories: dict[str, Any],
    baseline_name: str,
    alignment: Any,
) -> dict[str, Any]:
    """Verify and describe the first-pose transform shared by an A/B group."""
    if baseline_name not in trajectories:
        raise RuntimeError(f"missing exact alignment baseline {baseline_name}")
    anchor_stamp_ns = int(reference.stamp_ns[0])
    baseline = trajectories[baseline_name]
    residuals: dict[str, Any] = {}
    for name, trajectory in trajectories.items():
        if int(trajectory.stamp_ns[0]) != anchor_stamp_ns:
            raise RuntimeError(f"initial exact stamp differs for {name}")
        aligned = canonical.apply_alignment(trajectory, alignment)
        position = float(np.linalg.norm(aligned.xy[0] - reference.xy[0]))
        yaw = abs(float(wrap(aligned.yaw[0] - reference.yaw[0])))
        native_position_delta = float(
            np.linalg.norm(trajectory.xy[0] - baseline.xy[0])
        )
        native_yaw_delta = abs(
            float(wrap(trajectory.yaw[0] - baseline.yaw[0]))
        )
        identical_to_baseline = bool(
            native_position_delta <= EXACT_INITIAL_XY_TOLERANCE_M
            and native_yaw_delta <= EXACT_INITIAL_YAW_TOLERANCE_RAD
        )
        residuals[name] = {
            "stamp_ns": int(trajectory.stamp_ns[0]),
            "identical_to_baseline": identical_to_baseline,
            "native_position_delta_from_baseline_m": native_position_delta,
            "native_yaw_delta_from_baseline_rad": native_yaw_delta,
            "position_m": position,
            "yaw_rad": yaw,
            "yaw_deg": math.degrees(yaw),
        }
        if (name == baseline_name or identical_to_baseline) and (
            position > EXACT_INITIAL_XY_TOLERANCE_M
            or yaw > EXACT_INITIAL_YAW_TOLERANCE_RAD
        ):
            raise RuntimeError(
                f"exact initial alignment residual for {name} is position="
                f"{position:.3e} m yaw={yaw:.3e} rad"
            )
    return {
        "role": "primary_used_by_metrics_gates_csv_and_plots",
        "type": "single_pose_exact_se2",
        "baseline": baseline_name,
        "shared_across": list(trajectories),
        "anchor_stamp_ns": anchor_stamp_ns,
        "estimate_pose": {
            "x_m": float(baseline.xy[0, 0]),
            "y_m": float(baseline.xy[0, 1]),
            "yaw_rad": float(baseline.yaw[0]),
        },
        "reference_pose": {
            "x_m": float(reference.xy[0, 0]),
            "y_m": float(reference.xy[0, 1]),
            "yaw_rad": float(reference.yaw[0]),
        },
        "transform": {
            "yaw_rad": float(alignment.yaw_rad),
            "yaw_deg": math.degrees(float(alignment.yaw_rad)),
            "translation_x_m": float(alignment.translation[0]),
            "translation_y_m": float(alignment.translation[1]),
        },
        "scale_estimated_or_applied": False,
        "initial_residuals": residuals,
    }


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


def plot_prefix(label: str) -> str:
    """Return a short, filesystem-safe plot prefix without bag-specific policy."""
    pointcloud = re.search(r"pointcloud[_-]?(\d+)", label.lower())
    if pointcloud:
        return f"pc{pointcloud.group(1)}"
    normalized = re.sub(r"[^a-z0-9]+", "_", label.lower()).strip("_")
    return normalized or "precision"


def fixed_shared_series(
    canonical: Any,
    reference: Any,
    trajectories: dict[str, Any],
    alignment: Any,
    yaw_offset: float,
) -> dict[str, dict[str, np.ndarray]]:
    """Build the exact fixed/shared-alignment arrays used by the A/B metrics."""
    result: dict[str, dict[str, np.ndarray]] = {}
    for name, trajectory in trajectories.items():
        if not np.array_equal(trajectory.stamp_ns, reference.stamp_ns):
            raise RuntimeError(f"plot trajectory stamps differ from GLIM for {name}")
        aligned = canonical.apply_alignment(trajectory, alignment)
        aligned_yaw = np.asarray(wrap(trajectory.yaw + yaw_offset), dtype=float)
        xy_error = np.linalg.norm(aligned.xy - reference.xy, axis=1)
        yaw_error = np.degrees(np.abs(wrap(aligned_yaw - reference.yaw)))
        result[name] = {
            "xy": np.asarray(aligned.xy, dtype=float),
            "yaw": aligned_yaw,
            "xy_error_m": np.asarray(xy_error, dtype=float),
            "yaw_error_deg": np.asarray(yaw_error, dtype=float),
        }
    return result


def write_fixed_shared_csv(
    path: Path,
    reference: Any,
    series: dict[str, dict[str, np.ndarray]],
    evaluations: dict[str, Any],
    calibration_interval_ns: tuple[int, int] | None = None,
    outage_interval_ns: tuple[int, int] | None = None,
    tolerance: float = 1.0e-6,
) -> dict[str, dict[str, float]]:
    """Write auditable plot samples and verify their RMSE against JSON metrics."""
    names = list(series)
    if not np.all(np.isfinite(reference.xy)) or not np.all(np.isfinite(reference.yaw)):
        raise RuntimeError("plot reference contains non-finite values")
    if np.any(np.diff(reference.stamp_ns) <= 0):
        raise RuntimeError("plot reference stamps are not strictly increasing")
    cumulative_distance = np.concatenate(
        (
            np.asarray([0.0]),
            np.cumsum(np.linalg.norm(np.diff(reference.xy, axis=0), axis=1)),
        )
    )
    if not np.all(np.isfinite(cumulative_distance)) or np.any(
        np.diff(cumulative_distance) < 0.0
    ):
        raise RuntimeError("GLIM cumulative distance is not finite and monotonic")

    def interval_mask(interval: tuple[int, int] | None) -> np.ndarray:
        if interval is None:
            return np.zeros(len(reference.stamp_ns), dtype=np.int64)
        return (
            (reference.stamp_ns >= interval[0])
            & (reference.stamp_ns <= interval[1])
        ).astype(np.int64)

    header = [
        "stamp_sec",
        "time_from_common_start_sec",
        "glim_cumulative_distance_m",
        "calibration_mask",
        "outage_mask",
        "glim_x",
        "glim_y",
        "glim_yaw_rad",
    ]
    columns: list[np.ndarray] = [
        reference.stamp_ns.astype(np.float64) * 1.0e-9,
        (reference.stamp_ns - reference.stamp_ns[0]).astype(np.float64) * 1.0e-9,
        cumulative_distance,
        interval_mask(calibration_interval_ns),
        interval_mask(outage_interval_ns),
        reference.xy[:, 0],
        reference.xy[:, 1],
        reference.yaw,
    ]
    for name in names:
        header.extend(
            [
                f"{name}_x",
                f"{name}_y",
                f"{name}_yaw_rad",
                f"{name}_xy_error_m",
                f"{name}_yaw_error_deg",
            ]
        )
        columns.extend(
            [
                series[name]["xy"][:, 0],
                series[name]["xy"][:, 1],
                series[name]["yaw"],
                series[name]["xy_error_m"],
                series[name]["yaw_error_deg"],
            ]
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    matrix = np.column_stack(columns)
    if not np.all(np.isfinite(matrix)):
        raise RuntimeError("plot CSV matrix contains non-finite values")
    np.savetxt(
        path,
        matrix,
        delimiter=",",
        header=",".join(header),
        comments="",
        fmt="%.17g",
    )

    loaded = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))
    loaded_stamps = np.asarray(loaded["stamp_sec"], dtype=float)
    loaded_distance = np.asarray(loaded["glim_cumulative_distance_m"], dtype=float)
    if (
        not np.all(np.isfinite(loaded_stamps))
        or np.any(np.diff(loaded_stamps) <= 0.0)
        or not np.all(np.isfinite(loaded_distance))
        or np.any(np.diff(loaded_distance) < 0.0)
    ):
        raise RuntimeError("plot CSV stamp/distance integrity check failed")
    verification: dict[str, dict[str, float]] = {}
    for name in names:
        xy_rmse = float(
            np.sqrt(np.mean(np.asarray(loaded[f"{name}_xy_error_m"]) ** 2))
        )
        yaw_rmse = float(
            np.sqrt(np.mean(np.asarray(loaded[f"{name}_yaw_error_deg"]) ** 2))
        )
        expected_xy = float(evaluations[name]["fixed_common_xy_error_m"]["rmse"])
        expected_yaw = float(
            evaluations[name]["common_yaw_offset_error_deg"]["rmse"]
        )
        xy_difference = abs(xy_rmse - expected_xy)
        yaw_difference = abs(yaw_rmse - expected_yaw)
        if xy_difference > tolerance or yaw_difference > tolerance:
            raise RuntimeError(
                f"CSV RMSE mismatch for {name}: xy {xy_rmse} vs {expected_xy}, "
                f"yaw {yaw_rmse} vs {expected_yaw}"
            )
        verification[name] = {
            "csv_xy_rmse_m": xy_rmse,
            "json_xy_rmse_m": expected_xy,
            "xy_absolute_difference_m": xy_difference,
            "csv_yaw_rmse_deg": yaw_rmse,
            "json_yaw_rmse_deg": expected_yaw,
            "yaw_absolute_difference_deg": yaw_difference,
        }
    return verification


def make_fixed_shared_plot(
    path: Path,
    label: str,
    group: str,
    reference: Any,
    series: dict[str, dict[str, np.ndarray]],
    display_names: dict[str, str],
    series_names: tuple[str, ...],
    old_name: str,
    new_name: str,
    calibration_interval_ns: tuple[int, int] | None,
    outage_interval_ns: tuple[int, int] | None,
    alignment_title: str,
) -> None:
    """Render trajectory, fixed-frame XY error, and fixed-yaw error together."""
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/precision_glim_ab_matplotlib")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    styles = {
        "speed_raw": {"color": "#e68613", "linestyle": "--", "linewidth": 1.2},
        "speed_existing": {
            "color": "#e68613", "linestyle": "--", "linewidth": 1.2
        },
        old_name: {"color": "#969696", "linestyle": "-", "linewidth": 1.1},
        new_name: {"color": "#2764c4", "linestyle": "-", "linewidth": 1.4},
    }
    figure, axes = plt.subplots(1, 3, figsize=(21, 6.2))
    trajectory_axis, xy_axis, yaw_axis = axes
    trajectory_axis.plot(
        reference.xy[:, 0],
        reference.xy[:, 1],
        color="black",
        linewidth=2.0,
        label="GLIM",
        zorder=4,
    )
    for name in series_names:
        trajectory_axis.plot(
            series[name]["xy"][:, 0],
            series[name]["xy"][:, 1],
            label=display_names[name],
            zorder=2 if name != new_name else 3,
            **styles[name],
        )
    trajectory_axis.scatter(
        reference.xy[0, 0], reference.xy[0, 1], color="#24933d", marker="o", s=36,
        label="start", zorder=5,
    )
    trajectory_axis.scatter(
        reference.xy[-1, 0], reference.xy[-1, 1], color="#c9342d", marker="x", s=48,
        label="end", zorder=5,
    )

    def highlight_reference_interval(
        interval: tuple[int, int] | None, color: str, interval_label: str
    ) -> None:
        if interval is None:
            return
        mask = (reference.stamp_ns >= interval[0]) & (
            reference.stamp_ns <= interval[1]
        )
        if np.count_nonzero(mask) < 2:
            return
        trajectory_axis.plot(
            reference.xy[mask, 0],
            reference.xy[mask, 1],
            color=color,
            linewidth=7.0,
            alpha=0.20,
            label=interval_label,
            zorder=1,
        )

    highlight_reference_interval(
        calibration_interval_ns, "#49a65a", "calibration window"
    )
    highlight_reference_interval(
        outage_interval_ns, "#d95f5f", "common GNSS outage"
    )
    trajectory_axis.set_title("Trajectory overlay")
    trajectory_axis.set_xlabel("GLIM x [m]")
    trajectory_axis.set_ylabel("GLIM y [m]")
    trajectory_axis.axis("equal")
    trajectory_axis.grid(True, alpha=0.25)
    trajectory_axis.legend(fontsize=8)

    time = (reference.stamp_ns - reference.stamp_ns[0]) * 1.0e-9

    def shade_intervals(axis: Any) -> None:
        reference_start = int(reference.stamp_ns[0])
        reference_end = int(reference.stamp_ns[-1])
        for interval, color, interval_label in (
            (calibration_interval_ns, "#49a65a", "calibration window"),
            (outage_interval_ns, "#d95f5f", "common GNSS outage"),
        ):
            if interval is None:
                continue
            begin = max(reference_start, interval[0])
            end = min(reference_end, interval[1])
            if end <= begin:
                continue
            axis.axvspan(
                (begin - reference_start) * 1.0e-9,
                (end - reference_start) * 1.0e-9,
                color=color,
                alpha=0.12,
                label=interval_label,
                zorder=0,
            )

    for axis, key, unit, title in (
        (xy_axis, "xy_error_m", "m", "Absolute XY error"),
        (yaw_axis, "yaw_error_deg", "deg", "Absolute yaw error"),
    ):
        for name in series_names:
            values = series[name][key]
            rmse = float(np.sqrt(np.mean(values * values)))
            axis.plot(
                time,
                values,
                label=f"{display_names[name]} (RMSE {rmse:.3f} {unit})",
                zorder=2 if name != new_name else 3,
                **styles[name],
            )
        shade_intervals(axis)
        axis.set_title(
            f"{title}: {float(np.sqrt(np.mean(series[old_name][key] ** 2))):.3f}"
            f" -> {float(np.sqrt(np.mean(series[new_name][key] ** 2))):.3f} {unit}"
        )
        axis.set_xlabel("Time from common evaluation start [s]")
        axis.set_ylabel(f"Error [{unit}]")
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=8)

    figure.suptitle(f"{label} {group}: {alignment_title}")
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=160)
    plt.close(figure)
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"plot was not created or is empty: {path}")


def generate_plot_artifacts(
    args: argparse.Namespace,
    canonical: Any,
    local_reference: Any,
    local: dict[str, Any],
    local_alignment: Any,
    local_yaw_offset: float,
    global_reference: Any,
    global_trajectories: dict[str, Any],
    global_alignment: Any,
    global_yaw_offset: float,
    evaluations: dict[str, Any],
    common_outage: tuple[int, int] | None,
) -> dict[str, Any]:
    output = args.plot_directory.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    prefix = plot_prefix(args.label)
    exact = getattr(
        args, "alignment_mode", ALIGNMENT_FROZEN_CALIBRATION
    ) == ALIGNMENT_EXACT_INITIAL_POSE
    calibration = (
        None
        if exact
        else (
            int(round(args.calibration_start * 1.0e9)),
            int(round(args.calibration_end * 1.0e9)),
        )
    )
    if exact:
        local_names = ("precision_raw", "precision_local")
        global_names = ("precision_existing", "precision_global")
        alignment_description = (
            "one complete baseline first-common-pose SE(2), fixed/shared; "
            "the transform yaw is also the yaw alignment"
        )
        alignment_title = "exact initial-pose SE(2) (fixed/shared alignment)"
    else:
        local_names = ("speed_raw", "precision_raw", "precision_local")
        global_names = (
            "speed_existing", "precision_existing", "precision_global"
        )
        alignment_description = (
            "fixed/shared speed-baseline calibration; yaw uses the separate "
            "frozen circular offset"
        )
        alignment_title = (
            "frozen speed-baseline calibration (fixed/shared alignment)"
        )
    groups = {
        "local": {
            "reference": local_reference,
            "trajectories": local,
            "alignment": local_alignment,
            "yaw_offset": local_yaw_offset,
            "series_names": local_names,
            "old": "precision_raw",
            "new": "precision_local",
            "display": {
                "speed_raw": "speed raw (calibration baseline)",
                "precision_raw": "same-run raw",
                "precision_local": "precision local",
            },
        },
        "global": {
            "reference": global_reference,
            "trajectories": global_trajectories,
            "alignment": global_alignment,
            "yaw_offset": global_yaw_offset,
            "series_names": global_names,
            "old": "precision_existing",
            "new": "precision_global",
            "display": {
                "speed_existing": "speed existing EKF (calibration baseline)",
                "precision_existing": "same-run existing EKF",
                "precision_global": "precision global",
            },
        },
    }
    artifacts: dict[str, Any] = {
        "directory": str(output),
        "alignment": alignment_description,
    }
    for group, config in groups.items():
        series = fixed_shared_series(
            canonical,
            config["reference"],
            {
                name: config["trajectories"][name]
                for name in config["series_names"]
            },
            config["alignment"],
            config["yaw_offset"],
        )
        csv_path = output / f"{prefix}_{group}_glim.csv"
        png_path = output / f"{prefix}_{group}_glim.png"
        verification = write_fixed_shared_csv(
            csv_path,
            config["reference"],
            series,
            evaluations,
            calibration_interval_ns=calibration,
            outage_interval_ns=common_outage,
        )
        make_fixed_shared_plot(
            png_path,
            args.label,
            group,
            config["reference"],
            series,
            config["display"],
            config["series_names"],
            config["old"],
            config["new"],
            calibration,
            common_outage,
            alignment_title,
        )
        artifacts[group] = {
            "png": str(png_path),
            "csv": str(csv_path),
            "samples": len(config["reference"].stamp_ns),
            "csv_rmse_tolerance": 1.0e-6,
            "csv_rmse_verification": verification,
        }
    return artifacts


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


def _outage_yaw_float(values: dict[str, str], key: str) -> float | None:
    try:
        value = float(values[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _outage_yaw_float_or_nan(values: dict[str, str], key: str) -> float | None:
    """Parse a diagnostic float where NaN is an intentional cleared sentinel."""
    try:
        value = float(values[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if not math.isinf(value) else None


def _outage_yaw_counter(values: dict[str, str], key: str) -> int | None:
    try:
        encoded = str(values[key]).strip()
    except (KeyError, TypeError):
        return None
    if re.fullmatch(r"0|[1-9][0-9]*", encoded) is None:
        return None
    return int(encoded)


def _limited_error(errors: list[str], message: str, limit: int = 20) -> None:
    if len(errors) < limit:
        errors.append(message)


def _fusion_publication_counter(values: dict[str, Any], key: str) -> int | None:
    encoded = values.get(key)
    if not isinstance(encoded, str):
        return None
    if re.fullmatch(r"0|[1-9][0-9]*", encoded) is None:
        return None
    return int(encoded)


def fusion_publication_integrity_summary(data: dict[str, Any]) -> dict[str, Any]:
    """Validate map-fusion publication ordering without comparing run schedules.

    The odometry callback and wall timer are intentionally independent request
    sources.  Their benign coalescing counts may vary with DDS and executor
    scheduling, so only the strict odometry-drop counter is required to remain
    zero.  Every recorded counter snapshot must nevertheless carry the exact
    ordered four-key schema and satisfy integer, monotonic and accounting
    invariants.

    Exact raw-to-existing coverage is evaluated in MCAP record order.  The
    causal window opens at the first positive existing-fusion publication and
    closes at the final one; raw records outside that closed window are an
    initialization prefix or recorder-shutdown tail and are reported rather
    than silently mixed into the coverage denominator.
    """
    diagnostics = data.get("fusion_publication_diagnostics", [])
    schema_errors: list[str] = []
    integer_errors: list[str] = []
    counter_errors: list[str] = []
    rows: list[dict[str, int]] = []

    for sample_index, item in enumerate(diagnostics):
        if item.get("status_count") != 1:
            _limited_error(
                schema_errors,
                f"sample {sample_index} status_count={item.get('status_count')!r}",
            )
        values = item.get("values", {})
        raw_pairs = item.get("value_pairs")
        if raw_pairs is None:
            raw_pairs = list(values.items())
        keys = [str(pair[0]) for pair in raw_pairs]
        positions: list[int] = []
        sample_schema_valid = True
        for key in FUSION_PUBLICATION_COUNTER_KEYS:
            occurrences = [index for index, candidate in enumerate(keys) if candidate == key]
            if len(occurrences) != 1:
                sample_schema_valid = False
                _limited_error(
                    schema_errors,
                    f"sample {sample_index} key {key} occurs {len(occurrences)} times",
                )
            else:
                positions.append(occurrences[0])
        if sample_schema_valid and positions != list(
            range(positions[0], positions[0] + len(FUSION_PUBLICATION_COUNTER_KEYS))
        ):
            sample_schema_valid = False
            _limited_error(
                schema_errors,
                f"sample {sample_index} counter keys are not contiguous and ordered: "
                f"positions={positions}",
            )

        row: dict[str, int] = {}
        sample_integer_valid = True
        for key in FUSION_PUBLICATION_COUNTER_KEYS:
            value = _fusion_publication_counter(values, key)
            if value is None:
                sample_integer_valid = False
                _limited_error(
                    integer_errors,
                    f"sample {sample_index} {key} is not a canonical nonnegative integer",
                )
            else:
                row[key] = value
        if sample_schema_valid and sample_integer_valid:
            rows.append(row)

    previous: dict[str, int] | None = None
    accounting_valid = bool(rows) and len(rows) == len(diagnostics)
    monotonic = bool(rows) and len(rows) == len(diagnostics)
    for sample_index, row in enumerate(rows):
        component_sum = sum(row[key] for key in FUSION_PUBLICATION_COUNTER_KEYS[:3])
        if row[FUSION_PUBLICATION_COUNTER_KEYS[3]] != component_sum:
            accounting_valid = False
            _limited_error(
                counter_errors,
                f"sample {sample_index} total={row[FUSION_PUBLICATION_COUNTER_KEYS[3]]} "
                f"but components sum to {component_sum}",
            )
        if previous is not None:
            for key in FUSION_PUBLICATION_COUNTER_KEYS:
                if row[key] < previous[key]:
                    monotonic = False
                    _limited_error(
                        counter_errors,
                        f"sample {sample_index} {key} regressed "
                        f"{previous[key]}->{row[key]}",
                    )
        previous = row

    schema_valid = bool(diagnostics) and not schema_errors
    integer_valid = bool(diagnostics) and not integer_errors
    final_counters = (
        {key: rows[-1][key] for key in FUSION_PUBLICATION_COUNTER_KEYS}
        if rows else
        {key: None for key in FUSION_PUBLICATION_COUNTER_KEYS}
    )
    maxima = (
        {key: max(row[key] for row in rows) for key in FUSION_PUBLICATION_COUNTER_KEYS}
        if rows else
        {key: None for key in FUSION_PUBLICATION_COUNTER_KEYS}
    )
    strict_drop_zero = bool(rows) and all(
        row[FUSION_PUBLICATION_COUNTER_KEYS[0]] == 0 for row in rows
    )

    publications = data.get("publication_records", {})
    raw_records = list(publications.get(RAW, []))
    existing_records = list(publications.get(EXISTING_GLOBAL, []))
    publication_errors: list[str] = []

    def valid_record_stream(records: list[dict[str, Any]], label: str) -> bool:
        if not records:
            _limited_error(publication_errors, f"{label} publication records are empty")
            return False
        indices: list[int] = []
        record_times: list[int] = []
        for record_index, record in enumerate(records):
            index = record.get("record_index")
            record_time = record.get("record_ns")
            stamp = record.get("stamp_ns")
            if (
                not isinstance(index, int) or isinstance(index, bool) or
                not isinstance(record_time, int) or isinstance(record_time, bool) or
                not isinstance(stamp, int) or isinstance(stamp, bool)
            ):
                _limited_error(
                    publication_errors,
                    f"{label} record {record_index} lacks integer "
                    "record_index/record_ns/stamp_ns",
                )
                return False
            indices.append(index)
            record_times.append(record_time)
        if any(current <= previous for previous, current in zip(indices, indices[1:])):
            _limited_error(publication_errors, f"{label} record indices are not increasing")
            return False
        if any(
            current < previous
            for previous, current in zip(record_times, record_times[1:])
        ):
            _limited_error(publication_errors, f"{label} bag record times regress")
            return False
        return True

    raw_record_order_valid = valid_record_stream(raw_records, "raw")
    existing_record_order_valid = valid_record_stream(existing_records, "existing")
    diagnostic_record_indices = [item.get("record_index") for item in diagnostics]
    diagnostic_record_times = [item.get("record_ns") for item in diagnostics]
    diagnostic_record_order_valid = bool(diagnostics) and all(
        isinstance(value, int) and not isinstance(value, bool)
        for value in diagnostic_record_indices + diagnostic_record_times
    ) and all(
        current > previous
        for previous, current in zip(
            diagnostic_record_indices, diagnostic_record_indices[1:]
        )
    ) and all(
        current >= previous
        for previous, current in zip(
            diagnostic_record_times, diagnostic_record_times[1:]
        )
    )
    if not diagnostic_record_order_valid:
        _limited_error(
            publication_errors,
            "map-fusion diagnostic bag record times are unavailable or regress",
        )
    final_diagnostic_record_ns = (
        diagnostic_record_times[-1] if diagnostic_record_order_valid else None
    )
    final_diagnostic_record_index = (
        diagnostic_record_indices[-1] if diagnostic_record_order_valid else None
    )
    existing_prefix = (
        [
            record for record in existing_records
            if record["record_index"] <= final_diagnostic_record_index
        ]
        if existing_record_order_valid and final_diagnostic_record_index is not None else []
    )
    existing_prefix_stamps = [record["stamp_ns"] for record in existing_prefix]
    seen_positive = False
    previous_positive: int | None = None
    existing_stamp_order_valid = bool(existing_prefix)
    for stamp in existing_prefix_stamps:
        if stamp <= 0:
            if seen_positive:
                existing_stamp_order_valid = False
                _limited_error(
                    publication_errors,
                    "existing-fusion zero stamp appears after physical time begins",
                )
            continue
        seen_positive = True
        if previous_positive is not None and stamp < previous_positive:
            existing_stamp_order_valid = False
            _limited_error(
                publication_errors,
                f"existing-fusion header stamp regressed {previous_positive}->{stamp}",
            )
        previous_positive = stamp
    positive_existing = [record for record in existing_prefix if record["stamp_ns"] > 0]
    if not positive_existing:
        existing_stamp_order_valid = False
        _limited_error(publication_errors, "causal prefix has no positive existing output")

    first_existing_stamp = (
        positive_existing[0]["stamp_ns"] if positive_existing else None
    )
    last_existing_stamp = (
        positive_existing[-1]["stamp_ns"] if positive_existing else None
    )
    raw_prefix_positive = (
        [
            record for record in raw_records
            if record["record_index"] <= final_diagnostic_record_index and
            record["stamp_ns"] > 0
        ]
        if raw_record_order_valid and final_diagnostic_record_index is not None else []
    )
    causal_raw = (
        [
            record for record in raw_prefix_positive
            if first_existing_stamp <= record["stamp_ns"] <= last_existing_stamp
        ]
        if first_existing_stamp is not None and last_existing_stamp is not None else []
    )
    startup_raw = (
        [record for record in raw_prefix_positive if record["stamp_ns"] < first_existing_stamp]
        if first_existing_stamp is not None else []
    )
    stamp_tail_raw = (
        [record for record in raw_prefix_positive if record["stamp_ns"] > last_existing_stamp]
        if last_existing_stamp is not None else []
    )
    record_tail_raw = (
        [
            record for record in raw_records
            if record["record_index"] > final_diagnostic_record_index and
            record["stamp_ns"] > 0
        ]
        if raw_record_order_valid and final_diagnostic_record_index is not None else []
    )
    record_tail_existing = (
        [
            record for record in existing_records
            if record["record_index"] > final_diagnostic_record_index and
            record["stamp_ns"] > 0
        ]
        if existing_record_order_valid and final_diagnostic_record_index is not None else []
    )

    causal_raw_unique = {record["stamp_ns"] for record in causal_raw}
    existing_unique = {record["stamp_ns"] for record in positive_existing}
    missing_raw_stamps = sorted(causal_raw_unique - existing_unique)
    coverage_valid = (
        raw_record_order_valid and existing_record_order_valid and
        diagnostic_record_order_valid and existing_stamp_order_valid and
        bool(causal_raw_unique) and bool(existing_unique) and not missing_raw_stamps
    )
    if not causal_raw_unique:
        _limited_error(publication_errors, "causal raw coverage window is empty")
    if missing_raw_stamps:
        _limited_error(
            publication_errors,
            f"{len(missing_raw_stamps)} causal raw stamps lack exact existing output; "
            f"first={missing_raw_stamps[:10]}",
        )

    return {
        "valid": (
            schema_valid and integer_valid and monotonic and accounting_valid and
            strict_drop_zero and existing_stamp_order_valid and coverage_valid
        ),
        "diagnostic_samples": len(diagnostics),
        "counter_contract": {
            "schema_valid": schema_valid,
            "integer_valid": integer_valid,
            "monotonic": monotonic,
            "accounting_valid": accounting_valid,
            "strict_drop_zero": strict_drop_zero,
            "final": final_counters,
            "maxima": maxima,
        },
        "publication_contract": {
            "raw_record_order_valid": raw_record_order_valid,
            "existing_record_order_valid": existing_record_order_valid,
            "diagnostic_record_order_valid": diagnostic_record_order_valid,
            "existing_stamp_order_valid": existing_stamp_order_valid,
            "coverage_valid": coverage_valid,
            "final_diagnostic_record_index": final_diagnostic_record_index,
            "final_diagnostic_record_ns": final_diagnostic_record_ns,
            "first_existing_stamp_ns": first_existing_stamp,
            "last_existing_stamp_ns": last_existing_stamp,
            "raw_zero_stamp_excluded": sum(
                record.get("stamp_ns", 0) <= 0 for record in raw_records
            ),
            "raw_positive_prefix_excluded": len(startup_raw),
            "raw_positive_stamp_tail_excluded": len(stamp_tail_raw),
            "raw_positive_record_tail_excluded": len(record_tail_raw),
            "existing_positive_record_tail_excluded": len(record_tail_existing),
            "causal_raw_messages": len(causal_raw),
            "causal_raw_unique_stamps": len(causal_raw_unique),
            "existing_positive_messages": len(positive_existing),
            "existing_positive_unique_stamps": len(existing_unique),
            "missing_raw_unique_stamps": len(missing_raw_stamps),
            "missing_raw_stamp_examples": missing_raw_stamps[:10],
        },
        "errors": {
            "schema": schema_errors,
            "integer": integer_errors,
            "counters": counter_errors,
            "publication": publication_errors,
        },
    }


def fusion_publication_integrity_checks(
    summary: dict[str, Any], label: str
) -> list[dict[str, Any]]:
    """Return per-run hard gates; coalescing magnitudes remain report-only."""
    counters = summary["counter_contract"]
    publications = summary["publication_contract"]

    def check(name: str, passed: bool, detail: Any) -> dict[str, Any]:
        return {
            "name": f"{label} {name}",
            "passed": bool(passed),
            "detail": detail if isinstance(detail, str) else json.dumps(detail, sort_keys=True),
            "category": "hard",
        }

    return [
        check(
            "map-fusion publication counter schema",
            counters["schema_valid"] and counters["integer_valid"],
            {
                "samples": summary["diagnostic_samples"],
                "schema_errors": summary["errors"]["schema"],
                "integer_errors": summary["errors"]["integer"],
            },
        ),
        check(
            "map-fusion publication counter accounting",
            counters["monotonic"] and counters["accounting_valid"],
            {
                "final": counters["final"],
                "errors": summary["errors"]["counters"],
            },
        ),
        check(
            "map-fusion strict odometry drop is zero",
            counters["strict_drop_zero"],
            f"strict={counters['final'][FUSION_PUBLICATION_COUNTER_KEYS[0]]}",
        ),
        check(
            "existing-fusion published stamps are nondecreasing",
            publications["existing_record_order_valid"] and
            publications["existing_stamp_order_valid"],
            publications,
        ),
        check(
            "causal raw-to-existing exact stamp coverage",
            publications["raw_record_order_valid"] and publications["coverage_valid"],
            publications,
        ),
    ]


def outage_yaw_guard_summary(diagnostics: list[dict[str, Any]]) -> dict[str, Any]:
    """Summarize and validate the runtime outage-yaw diagnostic contract.

    Diagnostics are slower than the odometry output, so per-callback steps
    cannot normally be observed directly.  Consecutive snapshots can still
    prove the cumulative rate bound and, using the cumulative step counter,
    the maximum-step envelope.  Exact output/nominal/offset composition is
    checked independently at every finite snapshot.
    """
    raw_samples = [
        item
        for item in diagnostics
        if "outage_yaw_guard.enabled" in item.get("values", {})
    ]
    counter_keys = {
        "accepted_reference_count": "outage_yaw_guard.accepted_reference_count",
        "rejected_reference_count": "outage_yaw_guard.rejected_reference_count",
        "outage_count": "outage_yaw_guard.outage_count",
        "recovery_count": "outage_yaw_guard.recovery_count",
        "applied_step_count": "outage_yaw_guard.applied_step_count",
        "invalid_advance_count": "outage_yaw_guard.invalid_advance_count",
        "reset_count": "outage_yaw_guard.reset_count",
        "suppressed_invalid_count": "publish.global_suppressed_yaw_guard_invalid",
    }
    samples = list(raw_samples)
    folded_tail_duplicate_samples = 0
    if len(samples) >= 2:
        stamps = [int(item["stamp_ns"]) for item in samples]
        for index in range(1, len(stamps)):
            if stamps[index] != stamps[index - 1]:
                continue
            duplicate_stamp = stamps[index]
            if all(stamp == duplicate_stamp for stamp in stamps[index - 1 :]):
                tail_begin = index - 1
                tail = samples[tail_begin:]
                merged = dict(tail[-1])
                merged_values = dict(tail[-1]["values"])
                # The wall-timer tail is not additional physical evidence, but
                # a cumulative fault counter observed in any repeated snapshot
                # must not be lost by folding it to the final causal sample.
                for key in counter_keys.values():
                    observed = [
                        _outage_yaw_counter(item["values"], key) for item in tail
                    ]
                    finite = [value for value in observed if value is not None]
                    if finite:
                        merged_values[key] = str(max(finite))
                merged["values"] = merged_values
                samples = samples[:tail_begin] + [merged]
                folded_tail_duplicate_samples = len(tail) - 1
            break
    required_keys = {
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
        *counter_keys.values(),
        *(
            f"outage_yaw_guard.config.{name}"
            for name in OUTAGE_YAW_GUARD_EXPECTED_CONFIG
        ),
    }
    field_errors: list[str] = []
    config_errors: list[str] = []
    semantic_errors: list[str] = []
    counter_errors: list[str] = []
    state_errors: list[str] = []
    continuity_errors: list[str] = []
    state_counts: Counter[str] = Counter()
    transition_counts: Counter[str] = Counter()
    enabled_values: set[str] = set()
    observed_config: dict[str, set[float]] = {
        name: set() for name in OUTAGE_YAW_GUARD_EXPECTED_CONFIG
    }
    observed_semantics: dict[str, set[str]] = {
        name: set() for name in OUTAGE_YAW_GUARD_EXPECTED_SEMANTICS
    }
    counter_series: dict[str, list[int]] = {
        name: [] for name in counter_keys
    }
    maximum_abs_applied_offset = 0.0
    maximum_abs_target_offset = 0.0
    maximum_abs_observed_delta = 0.0
    maximum_additional_variance = 0.0
    maximum_active_reference_variance = 0.0
    maximum_composition_residual = 0.0
    maximum_observed_offset_rate = 0.0
    maximum_observed_release_delta = 0.0
    composition_samples = 0
    variance_samples = 0
    bounded_intervals = 0
    release_bounded_intervals = 0
    release_step_diagnostic_samples = 0
    observed_release_step_counter_delta = 0

    previous: dict[str, Any] | None = None
    for index, item in enumerate(samples):
        stamp_ns = int(item["stamp_ns"])
        values = item["values"]
        if stamp_ns <= 0:
            _limited_error(
                field_errors, f"sample {index} has non-positive stamp={stamp_ns}"
            )
        missing = sorted(required_keys - set(values))
        if missing:
            _limited_error(
                field_errors,
                f"sample {index} stamp={stamp_ns} missing {','.join(missing)}",
            )
        enabled_values.add(str(values.get("outage_yaw_guard.enabled", "<missing>")))
        state = normalized_state(values.get("outage_yaw_guard.state", ""))
        state_counts[state or "<MISSING>"] += 1
        if state not in OUTAGE_YAW_GUARD_STATES:
            _limited_error(state_errors, f"stamp={stamp_ns} invalid state={state!r}")
        if previous is not None:
            previous_state = previous["state"]
            if previous_state != state:
                transition_counts[f"{previous_state}->{state}"] += 1
            if stamp_ns <= previous["stamp_ns"]:
                _limited_error(
                    continuity_errors,
                    f"sample {index} diagnostic stamp is not strictly increasing: "
                    f"{previous['stamp_ns']}->{stamp_ns}",
                )

        serialized_active = str(values.get("outage_yaw_guard.active", "<missing>"))
        expected_active = state in OUTAGE_YAW_GUARD_ACTIVE_STATES
        if serialized_active not in {"true", "false"} or (
            serialized_active == "true"
        ) != expected_active:
            _limited_error(
                state_errors,
                f"stamp={stamp_ns} state={state} active={serialized_active}",
            )

        serialized_health = str(values.get("fusion.health.healthy", "<missing>"))
        serialized_ready = str(values.get("global_output_ready", "<missing>"))
        if serialized_health not in {"true", "false"}:
            _limited_error(
                state_errors,
                f"stamp={stamp_ns} invalid fusion health={serialized_health}",
            )
        if serialized_ready not in {"true", "false"}:
            _limited_error(
                state_errors,
                f"stamp={stamp_ns} invalid global readiness={serialized_ready}",
            )
        authority_tracking = (
            serialized_health == "true"
            and normalized_state(values.get("fusion.anchor.state", "")) == "TRACKING"
        )
        if authority_tracking and state in {"OUTAGE_SLEW", "OUTAGE_HOLD"}:
            _limited_error(
                state_errors,
                f"stamp={stamp_ns} authority TRACKING with state={state}",
            )
        if not authority_tracking and state == "RECOVERY_RELEASE":
            _limited_error(
                state_errors,
                f"stamp={stamp_ns} authority not TRACKING with state={state}",
            )
        if expected_active and serialized_ready != "true":
            _limited_error(
                state_errors,
                f"stamp={stamp_ns} active state={state} before global readiness",
            )

        for name, expected in OUTAGE_YAW_GUARD_EXPECTED_CONFIG.items():
            key = f"outage_yaw_guard.config.{name}"
            observed = _outage_yaw_float(values, key)
            if observed is None:
                _limited_error(config_errors, f"stamp={stamp_ns} invalid {key}")
                continue
            observed_config[name].add(observed)
            if not math.isclose(
                observed,
                expected,
                rel_tol=0.0,
                abs_tol=OUTAGE_YAW_GUARD_FLOAT_TOLERANCE,
            ):
                _limited_error(
                    config_errors,
                    f"stamp={stamp_ns} {name}={observed} expected={expected}",
                )
        for name, expected in OUTAGE_YAW_GUARD_EXPECTED_SEMANTICS.items():
            observed = str(values.get(f"outage_yaw_guard.{name}", "<missing>"))
            observed_semantics[name].add(observed)
            if observed != expected:
                _limited_error(
                    semantic_errors,
                    f"stamp={stamp_ns} {name}={observed!r} expected={expected!r}",
                )

        counters: dict[str, int | None] = {}
        for name, key in counter_keys.items():
            counter = _outage_yaw_counter(values, key)
            counters[name] = counter
            if counter is None:
                _limited_error(counter_errors, f"stamp={stamp_ns} invalid {key}")
            else:
                if counter_series[name] and counter < counter_series[name][-1]:
                    _limited_error(
                        counter_errors,
                        f"stamp={stamp_ns} counter {name} regressed "
                        f"{counter_series[name][-1]}->{counter}",
                    )
                counter_series[name].append(counter)
        odom_session_resets = _outage_yaw_counter(
            values, "local_correction.odom_session_resets"
        )
        if odom_session_resets is None:
            _limited_error(
                counter_errors,
                f"stamp={stamp_ns} invalid local_correction.odom_session_resets",
            )
        reset_count = counters.get("reset_count")
        if (
            odom_session_resets is not None
            and reset_count is not None
            and reset_count != odom_session_resets + 1
        ):
            _limited_error(
                counter_errors,
                f"stamp={stamp_ns} reset/session mismatch reset={reset_count} "
                f"odom={odom_session_resets}",
            )
        outage_count = counters.get("outage_count")
        recovery_count = counters.get("recovery_count")
        if (
            outage_count is not None
            and recovery_count is not None
            and recovery_count > outage_count
        ):
            _limited_error(
                counter_errors,
                f"stamp={stamp_ns} recovery={recovery_count} exceeds outage="
                f"{outage_count}",
            )
        if (
            state in {"OUTAGE_SLEW", "OUTAGE_HOLD"}
            and outage_count is not None
            and recovery_count is not None
            and outage_count != recovery_count + 1
        ):
            _limited_error(
                counter_errors,
                f"stamp={stamp_ns} outage state={state} has inconsistent edge "
                f"accounting outage={outage_count} recovery={recovery_count}",
            )
        if (
            state == "RECOVERY_RELEASE"
            and outage_count is not None
            and recovery_count is not None
            and outage_count != recovery_count
        ):
            _limited_error(
                counter_errors,
                f"stamp={stamp_ns} release edge accounting differs outage="
                f"{outage_count} recovery={recovery_count}",
            )

        applied = _outage_yaw_float(values, "outage_yaw_guard.applied_offset_rad")
        target = _outage_yaw_float(values, "outage_yaw_guard.target_offset_rad")
        observed_delta = _outage_yaw_float(values, "outage_yaw_guard.observed_delta_rad")
        additional_variance = _outage_yaw_float(
            values, "outage_yaw_guard.additional_variance_rad2"
        )
        nominal = _outage_yaw_float(values, "outage_yaw_guard.nominal_global_yaw_rad")
        output = _outage_yaw_float(values, "outage_yaw_guard.output_global_yaw_rad")
        trusted_variance = _outage_yaw_float(
            values, "outage_yaw_guard.trusted_variance_rad2"
        )
        active_reference_variance = _outage_yaw_float_or_nan(
            values, "outage_yaw_guard.active_reference_variance_rad2"
        )
        reference_keys = (
            "outage_yaw_guard.reference_stamp_sec",
            "outage_yaw_guard.reference_age_sec",
            "outage_yaw_guard.trusted_anchor_yaw_rad",
            "outage_yaw_guard.observed_fusion_anchor_yaw_rad",
            "outage_yaw_guard.observed_delta_rad",
            "outage_yaw_guard.trusted_variance_rad2",
        )
        try:
            reference_values = tuple(float(values[key]) for key in reference_keys)
        except (KeyError, TypeError, ValueError):
            reference_values = ()
            _limited_error(
                continuity_errors,
                f"stamp={stamp_ns} malformed trusted reference fields",
            )
        last_reason = str(values.get("outage_yaw_guard.last_reason", ""))
        if not last_reason:
            _limited_error(
                state_errors, f"stamp={stamp_ns} empty outage-yaw reason"
            )
        for name, parsed_value in (
            ("applied_offset_rad", applied),
            ("target_offset_rad", target),
            ("additional_variance_rad2", additional_variance),
        ):
            if parsed_value is None:
                _limited_error(
                    continuity_errors,
                    f"stamp={stamp_ns} non-finite outage_yaw_guard.{name}",
                )
        try:
            serialized_nominal = float(
                values["outage_yaw_guard.nominal_global_yaw_rad"]
            )
            serialized_output = float(
                values["outage_yaw_guard.output_global_yaw_rad"]
            )
            if math.isfinite(serialized_nominal) != math.isfinite(serialized_output):
                _limited_error(
                    continuity_errors,
                    f"stamp={stamp_ns} incomplete nominal/output yaw pair",
                )
        except (KeyError, TypeError, ValueError):
            _limited_error(
                continuity_errors,
                f"stamp={stamp_ns} malformed nominal/output yaw pair",
            )
        if applied is not None:
            maximum_abs_applied_offset = max(maximum_abs_applied_offset, abs(applied))
            if abs(applied) > OUTAGE_YAW_GUARD_EXPECTED_CONFIG[
                "max_trusted_delta_rad"
            ] + OUTAGE_YAW_GUARD_FLOAT_TOLERANCE:
                _limited_error(
                    continuity_errors,
                    f"stamp={stamp_ns} applied offset exceeds trusted delta gate: {applied}",
                )
        if target is not None:
            maximum_abs_target_offset = max(maximum_abs_target_offset, abs(target))
            if abs(target) > OUTAGE_YAW_GUARD_EXPECTED_CONFIG[
                "max_trusted_delta_rad"
            ] + OUTAGE_YAW_GUARD_FLOAT_TOLERANCE:
                _limited_error(
                    continuity_errors,
                    f"stamp={stamp_ns} target offset exceeds trusted delta gate: {target}",
                )
        if applied is not None and target is not None and additional_variance is not None:
            if state in {"DISARMED", "READY"} and (
                abs(applied) > OUTAGE_YAW_GUARD_FLOAT_TOLERANCE
                or abs(target) > OUTAGE_YAW_GUARD_FLOAT_TOLERANCE
                or abs(additional_variance) > OUTAGE_YAW_GUARD_FLOAT_TOLERANCE
            ):
                _limited_error(
                    continuity_errors,
                    f"stamp={stamp_ns} inactive state={state} carries applied="
                    f"{applied} target={target} variance={additional_variance}",
                )
            if state == "OUTAGE_HOLD" and abs(float(wrap(target - applied))) > (
                OUTAGE_YAW_GUARD_FLOAT_TOLERANCE
            ):
                _limited_error(
                    continuity_errors,
                    f"stamp={stamp_ns} outage hold residual="
                    f"{float(wrap(target - applied))}",
                )
            if state == "RECOVERY_RELEASE" and abs(target) > (
                OUTAGE_YAW_GUARD_FLOAT_TOLERANCE
            ):
                _limited_error(
                    continuity_errors,
                    f"stamp={stamp_ns} recovery target is nonzero: {target}",
                )
        if observed_delta is not None:
            maximum_abs_observed_delta = max(
                maximum_abs_observed_delta, abs(observed_delta)
            )
        if active_reference_variance is None:
            _limited_error(
                continuity_errors,
                f"stamp={stamp_ns} malformed active reference variance",
            )
        elif state in OUTAGE_YAW_GUARD_ACTIVE_STATES:
            if not math.isfinite(active_reference_variance) or not 0.0 <= (
                active_reference_variance
            ) <= (
                OUTAGE_YAW_GUARD_EXPECTED_CONFIG["max_trusted_variance_rad2"]
                + OUTAGE_YAW_GUARD_FLOAT_TOLERANCE
            ):
                _limited_error(
                    continuity_errors,
                    f"stamp={stamp_ns} active state={state} has invalid active "
                    f"reference variance={active_reference_variance}",
                )
            else:
                maximum_active_reference_variance = max(
                    maximum_active_reference_variance, active_reference_variance
                )
        elif not math.isnan(active_reference_variance):
            _limited_error(
                continuity_errors,
                f"stamp={stamp_ns} inactive state={state} did not clear active "
                f"reference variance={active_reference_variance}",
            )
        if reference_values:
            reference_finite = all(math.isfinite(value) for value in reference_values)
            reference_nan = all(math.isnan(value) for value in reference_values)
            if state == "DISARMED" and not reference_nan:
                _limited_error(
                    continuity_errors,
                    f"stamp={stamp_ns} disarmed reference is not cleared",
                )
            elif state != "DISARMED" and not reference_finite:
                _limited_error(
                    continuity_errors,
                    f"stamp={stamp_ns} armed state={state} has incomplete reference",
                )
            if reference_finite:
                (
                    reference_stamp,
                    reference_age,
                    trusted_yaw,
                    observed_yaw,
                    serialized_delta,
                    serialized_variance,
                ) = reference_values
                if reference_stamp <= 0.0 or reference_age < 0.0:
                    _limited_error(
                        continuity_errors,
                        f"stamp={stamp_ns} invalid reference time stamp="
                        f"{reference_stamp} age={reference_age}",
                    )
                if not 0.0 <= serialized_variance <= (
                    OUTAGE_YAW_GUARD_EXPECTED_CONFIG["max_trusted_variance_rad2"]
                    + OUTAGE_YAW_GUARD_FLOAT_TOLERANCE
                ):
                    _limited_error(
                        continuity_errors,
                        f"stamp={stamp_ns} trusted variance={serialized_variance}",
                    )
                delta_residual = abs(
                    float(wrap(trusted_yaw - observed_yaw - serialized_delta))
                )
                if delta_residual > OUTAGE_YAW_GUARD_FLOAT_TOLERANCE:
                    _limited_error(
                        continuity_errors,
                        f"stamp={stamp_ns} observed delta residual={delta_residual}",
                    )
        if additional_variance is not None:
            maximum_additional_variance = max(
                maximum_additional_variance, additional_variance
            )
            if additional_variance < -OUTAGE_YAW_GUARD_FLOAT_TOLERANCE:
                _limited_error(
                    continuity_errors,
                    f"stamp={stamp_ns} negative additional variance={additional_variance}",
                )
        if nominal is not None and output is not None and applied is not None:
            composition_samples += 1
            residual = abs(float(wrap(output - nominal - applied)))
            maximum_composition_residual = max(maximum_composition_residual, residual)
            if residual > OUTAGE_YAW_GUARD_FLOAT_TOLERANCE:
                _limited_error(
                    continuity_errors,
                    f"stamp={stamp_ns} yaw composition residual={residual}",
                )

        expected_variance: float | None = None
        if additional_variance is not None and applied is not None and target is not None:
            if (
                state in {"OUTAGE_SLEW", "OUTAGE_HOLD"}
                and active_reference_variance is not None
                and math.isfinite(active_reference_variance)
            ):
                expected_variance = (
                    active_reference_variance
                    + float(wrap(target - applied)) ** 2
                )
            elif (
                state == "RECOVERY_RELEASE"
                and active_reference_variance is not None
                and math.isfinite(active_reference_variance)
            ):
                expected_variance = active_reference_variance + applied * applied
            elif state in {"READY", "DISARMED"}:
                expected_variance = 0.0
        if expected_variance is not None:
            variance_samples += 1
            if not math.isclose(
                additional_variance,
                expected_variance,
                rel_tol=0.0,
                abs_tol=OUTAGE_YAW_GUARD_FLOAT_TOLERANCE,
            ):
                _limited_error(
                    continuity_errors,
                    f"stamp={stamp_ns} additional variance={additional_variance} "
                    f"expected={expected_variance}",
                )

        current = {
            "stamp_ns": stamp_ns,
            "state": state,
            "applied": applied,
            "active_reference_variance": active_reference_variance,
            "trusted_variance": trusted_variance,
            "last_reason": last_reason,
            "counters": counters,
        }
        if previous is not None and applied is not None and previous["applied"] is not None:
            dt_sec = (stamp_ns - previous["stamp_ns"]) * 1.0e-9
            reset_before = previous["counters"].get("reset_count")
            reset_after = counters.get("reset_count")
            steps_before = previous["counters"].get("applied_step_count")
            steps_after = counters.get("applied_step_count")
            outage_before = previous["counters"].get("outage_count")
            outage_after = counters.get("outage_count")
            recovery_before = previous["counters"].get("recovery_count")
            recovery_after = counters.get("recovery_count")
            accepted_before = previous["counters"].get("accepted_reference_count")
            accepted_after = counters.get("accepted_reference_count")
            if all(
                value is not None
                for value in (
                    reset_before,
                    reset_after,
                    outage_before,
                    outage_after,
                    recovery_before,
                    recovery_after,
                    accepted_before,
                    accepted_after,
                )
            ):
                reset_delta = reset_after - reset_before
                outage_delta = outage_after - outage_before
                recovery_delta = recovery_after - recovery_before
                accepted_delta = accepted_after - accepted_before
                previous_state = previous["state"]
                previous_outage = previous_state in {"OUTAGE_SLEW", "OUTAGE_HOLD"}
                current_outage = state in {"OUTAGE_SLEW", "OUTAGE_HOLD"}
                if reset_delta == 0:
                    expected_outage_delta = (
                        recovery_delta
                        + int(current_outage)
                        - int(previous_outage)
                    )
                    if outage_delta != expected_outage_delta:
                        _limited_error(
                            counter_errors,
                            f"stamp={stamp_ns} outage/recovery endpoint balance "
                            f"mismatch outage_delta={outage_delta} "
                            f"recovery_delta={recovery_delta} "
                            f"previous_state={previous_state} state={state} "
                            f"expected_outage_delta={expected_outage_delta}",
                        )
                if (
                    reset_delta == 0
                    and previous_state in OUTAGE_YAW_GUARD_ACTIVE_STATES
                    and state in OUTAGE_YAW_GUARD_ACTIVE_STATES
                    and previous["active_reference_variance"] is not None
                    and active_reference_variance is not None
                    and math.isfinite(previous["active_reference_variance"])
                    and math.isfinite(active_reference_variance)
                ):
                    previous_active_variance = previous[
                        "active_reference_variance"
                    ]
                    expected_active_variance = previous_active_variance
                    visible_reoutage = (
                        previous_state == "RECOVERY_RELEASE"
                        and current_outage
                        and outage_delta == 1
                        and recovery_delta == 0
                        and last_reason in OUTAGE_YAW_GUARD_REOUTAGE_REASONS
                    )
                    if visible_reoutage:
                        if last_reason == OUTAGE_YAW_GUARD_REOUTAGE_FRESH_REASON:
                            if trusted_variance is None:
                                _limited_error(
                                    continuity_errors,
                                    f"stamp={stamp_ns} fresh re-outage reason has "
                                    "non-finite trusted variance",
                                )
                            else:
                                expected_active_variance = max(
                                    previous_active_variance, trusted_variance
                                )
                        elif last_reason in OUTAGE_YAW_GUARD_REOUTAGE_RETAIN_REASONS:
                            expected_active_variance = previous_active_variance
                    elif outage_delta > 0:
                        # Diagnostic sampling can hide a release followed by a
                        # re-outage.  Its exact reason is unavailable, but every
                        # runtime branch preserves or increases the snapshot.
                        if (
                            active_reference_variance
                            + OUTAGE_YAW_GUARD_FLOAT_TOLERANCE
                            < previous_active_variance
                        ):
                            _limited_error(
                                continuity_errors,
                                f"stamp={stamp_ns} hidden re-outage decreased active "
                                "reference variance "
                                f"{previous_active_variance}->{active_reference_variance}",
                            )
                        expected_active_variance = None
                    if (
                        expected_active_variance is not None
                        and not math.isclose(
                            active_reference_variance,
                            expected_active_variance,
                            rel_tol=0.0,
                            abs_tol=OUTAGE_YAW_GUARD_FLOAT_TOLERANCE,
                        )
                    ):
                        _limited_error(
                            continuity_errors,
                            f"stamp={stamp_ns} active reference variance="
                            f"{active_reference_variance} expected="
                            f"{expected_active_variance} outage_delta="
                            f"{outage_delta}",
                        )
                if current_outage and not previous_outage and outage_delta <= 0:
                    _limited_error(
                        counter_errors,
                        f"stamp={stamp_ns} transition {previous_state}->{state} "
                        "lacks outage counter edge",
                    )
                if (
                    state == "RECOVERY_RELEASE"
                    and previous_state != "RECOVERY_RELEASE"
                    and recovery_delta <= 0
                ):
                    _limited_error(
                        counter_errors,
                        f"stamp={stamp_ns} transition {previous_state}->{state} "
                        "lacks recovery counter edge",
                    )
                if (
                    previous_outage
                    and state in {"READY", "DISARMED"}
                    and recovery_delta <= 0
                    and reset_delta <= 0
                ):
                    _limited_error(
                        counter_errors,
                        f"stamp={stamp_ns} transition {previous_state}->{state} "
                        "skips recovery",
                    )
                if (
                    previous_state == "RECOVERY_RELEASE"
                    and current_outage
                    and outage_delta <= 0
                ):
                    _limited_error(
                        counter_errors,
                        f"stamp={stamp_ns} release re-entry lacks outage edge",
                    )
                if (
                    previous_state == "DISARMED"
                    and state == "READY"
                    and accepted_delta <= 0
                ):
                    _limited_error(
                        counter_errors,
                        f"stamp={stamp_ns} DISARMED->READY lacks accepted reference",
                    )
                if previous_state == "READY" and state == "RECOVERY_RELEASE" and not (
                    outage_delta > 0 and recovery_delta > 0
                ):
                    _limited_error(
                        counter_errors,
                        f"stamp={stamp_ns} READY->RECOVERY_RELEASE lacks hidden edges",
                    )
            if (
                dt_sec > 0.0
                and reset_before is not None
                and reset_after == reset_before
                and steps_before is not None
                and steps_after is not None
            ):
                bounded_intervals += 1
                offset_delta = abs(float(wrap(applied - previous["applied"])))
                step_count = steps_after - steps_before
                maximum_observed_offset_rate = max(
                    maximum_observed_offset_rate, offset_delta / dt_sec
                )
                rate_limit = (
                    OUTAGE_YAW_GUARD_EXPECTED_CONFIG["max_offset_rate_radps"]
                    * (
                        dt_sec
                        + OUTAGE_YAW_GUARD_EXPECTED_CONFIG["max_step_dt_sec"]
                    )
                )
                step_limit = (
                    OUTAGE_YAW_GUARD_EXPECTED_CONFIG["max_offset_step_rad"]
                    * step_count
                )
                if offset_delta > rate_limit + OUTAGE_YAW_GUARD_FLOAT_TOLERANCE:
                    _limited_error(
                        continuity_errors,
                        f"stamp={stamp_ns} observed offset delta={offset_delta} "
                        f"exceeds rate envelope={rate_limit}",
                    )
                if step_count < 0 or offset_delta > (
                    step_limit + OUTAGE_YAW_GUARD_FLOAT_TOLERANCE
                ):
                    _limited_error(
                        continuity_errors,
                        f"stamp={stamp_ns} observed offset delta={offset_delta} "
                        f"exceeds {step_count} step envelope={step_limit}",
                    )
                if previous["state"] == "RECOVERY_RELEASE":
                    release_bounded_intervals += 1
                    observed_release_step_counter_delta += step_count
                    maximum_observed_release_delta = max(
                        maximum_observed_release_delta, offset_delta
                    )
        if state == "RECOVERY_RELEASE" and str(
            values.get("outage_yaw_guard.last_reason", "")
        ) == "outage_yaw_recovery_release_step":
            release_step_diagnostic_samples += 1
        previous = current

    final_counters = {
        name: (series[-1] if series else None)
        for name, series in counter_series.items()
    }
    return {
        "present": bool(samples),
        "raw_diagnostic_samples": len(raw_samples),
        "diagnostic_samples": len(samples),
        "folded_wall_timer_tail_samples": folded_tail_duplicate_samples,
        "enabled_values": sorted(enabled_values),
        "enabled_all": bool(samples) and enabled_values == {"true"},
        "required_fields_complete": not field_errors,
        "state_counts": dict(sorted(state_counts.items())),
        "state_transition_counts": dict(sorted(transition_counts.items())),
        "counters": final_counters,
        "counter_monotonic": not counter_errors,
        "maxima": {
            "abs_applied_offset_rad": maximum_abs_applied_offset,
            "abs_target_offset_rad": maximum_abs_target_offset,
            "abs_observed_delta_rad": maximum_abs_observed_delta,
            "active_reference_variance_rad2": maximum_active_reference_variance,
            "additional_variance_rad2": maximum_additional_variance,
        },
        "release": {
            "observed_state_samples": state_counts.get("RECOVERY_RELEASE", 0),
            "step_reason_samples": release_step_diagnostic_samples,
            "bounded_diagnostic_intervals": release_bounded_intervals,
            "observed_applied_step_count_delta": observed_release_step_counter_delta,
            "maximum_observed_offset_delta_rad": maximum_observed_release_delta,
        },
        "config": {
            "expected": OUTAGE_YAW_GUARD_EXPECTED_CONFIG,
            "observed": {
                name: sorted(values) for name, values in observed_config.items()
            },
            "valid": bool(samples) and not config_errors,
        },
        "semantics": {
            "expected": OUTAGE_YAW_GUARD_EXPECTED_SEMANTICS,
            "observed": {
                name: sorted(values) for name, values in observed_semantics.items()
            },
            "valid": bool(samples) and not semantic_errors,
        },
        "fusion_health_state_contract": {
            "valid": bool(samples) and not state_errors,
            "violations": state_errors,
        },
        "continuity_and_bounds": {
            "valid": bool(samples) and composition_samples > 0
            and bounded_intervals > 0 and not continuity_errors,
            "composition_samples": composition_samples,
            "variance_samples": variance_samples,
            "bounded_diagnostic_intervals": bounded_intervals,
            "maximum_composition_residual_rad": maximum_composition_residual,
            "maximum_observed_offset_rate_radps": maximum_observed_offset_rate,
            "errors": continuity_errors,
        },
        "errors": {
            "fields": field_errors,
            "config": config_errors,
            "semantics": semantic_errors,
            "counters": counter_errors,
            "state": state_errors,
            "continuity": continuity_errors,
        },
    }


def outage_yaw_guard_contract_checks(
    summary: dict[str, Any], dataset_outage_available: bool
) -> list[dict[str, Any]]:
    """Return hard acceptance checks without changing accuracy gates."""
    counters = summary["counters"]

    def check(name: str, passed: bool, detail: str) -> dict[str, Any]:
        return {
            "name": name,
            "passed": bool(passed),
            "detail": detail,
            "category": "hard",
        }

    exercise_passed = not dataset_outage_available or (
        (counters["outage_count"] or 0) > 0
        and (counters["recovery_count"] or 0) > 0
        and sum(
            summary["state_counts"].get(state, 0)
            for state in ("OUTAGE_SLEW", "OUTAGE_HOLD")
        ) > 0
    )
    return [
        check(
            "outage yaw guard diagnostics present and enabled",
            summary["present"] and summary["enabled_all"],
            f"samples={summary['diagnostic_samples']} enabled="
            f"{summary['enabled_values']}",
        ),
        check(
            "outage yaw guard diagnostic fields and counters valid",
            summary["required_fields_complete"] and summary["counter_monotonic"],
            f"field_errors={summary['errors']['fields']} counter_errors="
            f"{summary['errors']['counters']}",
        ),
        check(
            "outage yaw guard configuration contract",
            summary["config"]["valid"],
            json.dumps(summary["config"], sort_keys=True),
        ),
        check(
            "outage yaw guard source and XY policy contract",
            summary["semantics"]["valid"],
            json.dumps(summary["semantics"], sort_keys=True),
        ),
        check(
            "outage yaw guard accepted trusted references",
            counters["accepted_reference_count"] is not None
            and counters["accepted_reference_count"] > 0,
            f"accepted={counters['accepted_reference_count']} rejected="
            f"{counters['rejected_reference_count']}",
        ),
        check(
            "outage yaw guard exercises outage and recovery",
            exercise_passed,
            f"dataset_outage={dataset_outage_available} states="
            f"{summary['state_counts']} outage_count={counters['outage_count']} "
            f"recovery_count={counters['recovery_count']}",
        ),
        check(
            "outage yaw guard has no invalid or suppressed output",
            counters["invalid_advance_count"] == 0
            and counters["suppressed_invalid_count"] == 0,
            f"invalid={counters['invalid_advance_count']} suppressed="
            f"{counters['suppressed_invalid_count']}",
        ),
        check(
            "outage yaw guard state agrees with fusion authority",
            summary["fusion_health_state_contract"]["valid"],
            json.dumps(summary["fusion_health_state_contract"], sort_keys=True),
        ),
        check(
            "outage yaw guard composition and bounded continuity",
            summary["continuity_and_bounds"]["valid"],
            json.dumps(summary["continuity_and_bounds"], sort_keys=True),
        ),
    ]


def precision_timeline_summary(data: dict[str, Any]) -> dict[str, Any]:
    diagnostics = data["precision_diagnostics"]
    publication_records = data.get("publication_records", {})
    native_initialization_errors: list[str] = []

    def first_native_positive_stamp(topic: str) -> int | None:
        records = publication_records.get(topic)
        if not isinstance(records, list) or not records:
            native_initialization_errors.append(
                f"{topic} native publication records are missing or empty"
            )
            return None
        previous_record_index: int | None = None
        positive_stamps: list[int] = []
        for index, record in enumerate(records):
            if not isinstance(record, dict):
                native_initialization_errors.append(
                    f"{topic} native record {index} is not an object"
                )
                return None
            record_index = record.get("record_index")
            physical_stamp = record.get("stamp_ns")
            if (
                not isinstance(record_index, int) or isinstance(record_index, bool) or
                not isinstance(physical_stamp, int) or isinstance(physical_stamp, bool)
            ):
                native_initialization_errors.append(
                    f"{topic} native record {index} lacks integer "
                    "record_index/stamp_ns"
                )
                return None
            if previous_record_index is not None and record_index <= previous_record_index:
                native_initialization_errors.append(
                    f"{topic} native record order is not strictly increasing at {index}"
                )
                return None
            previous_record_index = record_index
            if physical_stamp > 0:
                positive_stamps.append(physical_stamp)
        if not positive_stamps:
            native_initialization_errors.append(
                f"{topic} has no native positive header stamp"
            )
            return None
        return positive_stamps[0]

    raw_start = first_native_positive_stamp(RAW)
    precision_global_start = first_native_positive_stamp(PRECISION_GLOBAL)
    if (
        raw_start is not None and precision_global_start is not None and
        precision_global_start < raw_start
    ):
        native_initialization_errors.append(
            "precision-global first native stamp precedes raw first native stamp: "
            f"{precision_global_start} < {raw_start}"
        )
    native_initialization_valid = not native_initialization_errors
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
            (precision_global_start - raw_start) * 1.0e-9
            if native_initialization_valid and raw_start is not None and
            precision_global_start is not None else None
        ),
        "initialization_timing_source": (
            "first_native_positive_precision_global_header_stamp_minus_"
            "first_native_positive_raw_header_stamp"
        ),
        "native_initialization_timestamps_valid": native_initialization_valid,
        "native_initialization_errors": native_initialization_errors,
        "raw_first_native_positive_stamp_ns": raw_start,
        "precision_global_first_native_positive_stamp_ns": precision_global_start,
        "diagnostic_initialization_observed_stamp_ns": (
            initialized["stamp_ns"] if initialized is not None else None
        ),
        "diagnostic_initialization_observed_delay_sec": (
            (initialized["stamp_ns"] - raw_start) * 1.0e-9
            if initialized is not None and raw_start is not None else None
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
        "outage_yaw_guard": outage_yaw_guard_summary(diagnostics),
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
    alignment_mode = getattr(
        args, "alignment_mode", ALIGNMENT_FROZEN_CALIBRATION
    )
    if alignment_mode not in (
        ALIGNMENT_FROZEN_CALIBRATION,
        ALIGNMENT_EXACT_INITIAL_POSE,
    ):
        raise RuntimeError(f"unknown alignment mode: {alignment_mode}")
    reference = canonical.read_glim_trajectory(
        args.glim_dir.resolve() / args.glim_trajectory
    )
    speed = read_bag(args.speed_bag, canonical, precision=False)
    precision = read_bag(args.precision_bag, canonical, precision=True)

    if alignment_mode == ALIGNMENT_EXACT_INITIAL_POSE:
        # Accuracy is an exact-stamp before/after comparison within one run.
        # The separately replayed speed run remains available below only for
        # runtime/non-intrusion/protocol checks.
        local_reference, local = exact_estimator_common_samples(
            canonical,
            reference,
            {
                "precision_raw": precision["trajectories"][RAW],
                "precision_local": precision["trajectories"][PRECISION_LOCAL],
            },
            args.maximum_interpolation_gap_sec,
        )
        global_reference, global_trajectories = exact_estimator_common_samples(
            canonical,
            reference,
            {
                "precision_existing": precision["trajectories"][EXISTING_GLOBAL],
                "precision_global": precision["trajectories"][PRECISION_GLOBAL],
            },
            args.maximum_interpolation_gap_sec,
        )
        local_alignment = exact_pose_alignment(
            canonical,
            local["precision_raw"].xy[0],
            float(local["precision_raw"].yaw[0]),
            local_reference.xy[0],
            float(local_reference.yaw[0]),
        )
        global_alignment = exact_pose_alignment(
            canonical,
            global_trajectories["precision_existing"].xy[0],
            float(global_trajectories["precision_existing"].yaw[0]),
            global_reference.xy[0],
            float(global_reference.yaw[0]),
        )
        # A full SE(2) has only one yaw.  Do not fit a separate yaw offset.
        local_yaw_offset = float(local_alignment.yaw_rad)
        global_yaw_offset = float(global_alignment.yaw_rad)
        alignment_metadata = {
            "mode": alignment_mode,
            "estimate_association": (
                "exact integer header-stamp intersection within the precision "
                "run; estimator streams are not interpolated"
            ),
            "reference_association": (
                "GLIM alone is interpolated to exact estimator header stamps"
            ),
            "local": exact_initial_alignment_metadata(
                canonical,
                local_reference,
                local,
                "precision_raw",
                local_alignment,
            ),
            "global": exact_initial_alignment_metadata(
                canonical,
                global_reference,
                global_trajectories,
                "precision_existing",
                global_alignment,
            ),
        }
    else:
        if args.calibration_start is None or args.calibration_end is None:
            raise RuntimeError(
                "frozen-calibration-window requires calibration start and end"
            )
        if args.calibration_end <= args.calibration_start:
            raise RuntimeError("calibration end must be later than start")
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
        local_mask = (
            (local_reference.stamp_ns >= start_ns)
            & (local_reference.stamp_ns <= end_ns)
        )
        global_mask = (
            (global_reference.stamp_ns >= start_ns)
            & (global_reference.stamp_ns <= end_ns)
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
            global_trajectories["speed_existing"].yaw,
            global_reference.yaw,
            global_mask,
        )
        alignment_metadata = {
            "mode": alignment_mode,
            "estimate_association": (
                "historical common GLIM grid with estimator interpolation"
            ),
            "reference_association": "GLIM native header stamps",
            "calibration_interval_ns": [start_ns, end_ns],
        }

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

    if alignment_mode == ALIGNMENT_EXACT_INITIAL_POSE:
        precision_outages = outage_intervals(precision["fusion_states"])
        common_outage = (
            max(precision_outages, key=lambda item: item[1] - item[0])
            if precision_outages
            else None
        )
    else:
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
    existing_global_glim_comparison = (
        None
        if alignment_mode == ALIGNMENT_EXACT_INITIAL_POSE
        else compare_evaluations(
            evaluations["speed_existing"], evaluations["precision_existing"]
        )
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
    fusion_publication_integrity = {
        "speed": fusion_publication_integrity_summary(speed),
        "precision": fusion_publication_integrity_summary(precision),
    }

    checks: list[dict[str, Any]] = []

    def add(name: str, passed: bool, detail: str, category: str = "hard") -> None:
        checks.append(
            {"name": name, "passed": bool(passed), "detail": detail, "category": category}
        )

    for run in ("speed", "precision"):
        checks.extend(
            fusion_publication_integrity_checks(
                fusion_publication_integrity[run], run
            )
        )

    if alignment_mode == ALIGNMENT_EXACT_INITIAL_POSE:
        for group in ("local", "global"):
            metadata = alignment_metadata[group]
            residuals = metadata["initial_residuals"]
            baseline = residuals[metadata["baseline"]]
            identical = [
                item
                for item in residuals.values()
                if item["identical_to_baseline"]
            ]
            add(
                f"{group} primary initial pose maps exactly",
                baseline["position_m"] <= EXACT_INITIAL_XY_TOLERANCE_M
                and baseline["yaw_rad"] <= EXACT_INITIAL_YAW_TOLERANCE_RAD,
                f"stamp_ns={metadata['anchor_stamp_ns']} position="
                f"{baseline['position_m']:.3e}m yaw={baseline['yaw_rad']:.3e}rad",
            )
            add(
                f"{group} identical-start estimates map exactly",
                all(
                    item["position_m"] <= EXACT_INITIAL_XY_TOLERANCE_M
                    and item["yaw_rad"] <= EXACT_INITIAL_YAW_TOLERANCE_RAD
                    for item in identical
                ),
                json.dumps(residuals, sort_keys=True),
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
            and precision_exit == speed_exit,
            f"level={speed_fusion_final['level']}->{precision_fusion_final['level']} "
            f"anchor={speed_anchor}->{precision_anchor} "
            f"exits={speed_exit}->{precision_exit} "
            f"position_fused={speed_position_fused}->{precision_position_fused} "
            f"yaw_fused={speed_yaw_fused}->{precision_yaw_fused} "
            f"fix={speed_fix}->{precision_fix}",
        )
    if existing_global_glim_comparison is not None:
        for name, item, absolute in (
            (
                "fixed-frame XY",
                existing_global_glim_comparison["fixed_xy_rmse_m"],
                0.05,
            ),
            (
                "fixed-frame yaw",
                existing_global_glim_comparison["yaw_rmse_deg"],
                0.05,
            ),
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
                item["yaw_rmse_deg"]["old"],
                item["yaw_rmse_deg"]["new"],
                0.05,
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
        precision_timeline["native_initialization_timestamps_valid"]
        and initialization_delay is not None
        and 0.0 <= initialization_delay <= 20.0,
        f"native={initialization_delay} sec raw_stamp_ns="
        f"{precision_timeline['raw_first_native_positive_stamp_ns']} "
        f"precision_global_stamp_ns="
        f"{precision_timeline['precision_global_first_native_positive_stamp_ns']} "
        f"diagnostic_observed="
        f"{precision_timeline['diagnostic_initialization_observed_delay_sec']} sec "
        f"errors={precision_timeline['native_initialization_errors']}",
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
        legacy_label_compatibility = bool(
            alignment_mode == ALIGNMENT_FROZEN_CALIBRATION
            and "pointcloud2" in args.label.lower()
        )
        require_legacy_tracking_recovery = bool(
            getattr(args, "require_legacy_tracking_recovery", False)
            or legacy_label_compatibility
        )
        if require_legacy_tracking_recovery:
            speed_legacy_recovery = diagnostic_recovery_delay(
                speed["fusion_diagnostics"], timeline_outage["end_ns"], legacy=True
            )
            precision_legacy_recovery = diagnostic_recovery_delay(
                precision["fusion_diagnostics"], timeline_outage["end_ns"], legacy=True
            )
            add(
                (
                    "PC2 legacy fusion TRACKING recovery retained in both modes"
                    if legacy_label_compatibility
                    else "required legacy fusion TRACKING recovery retained in both modes"
                ),
                speed_legacy_recovery is not None
                and precision_legacy_recovery is not None
                and 0.0 <= speed_legacy_recovery <= 10.0
                and 0.0 <= precision_legacy_recovery <= 10.0,
                f"speed={speed_legacy_recovery} precision={precision_legacy_recovery}",
            )
    checks.extend(
        outage_yaw_guard_contract_checks(
            precision_timeline["outage_yaw_guard"],
            dataset_outage_available=timeline_outage is not None,
        )
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
    if alignment_mode == ALIGNMENT_EXACT_INITIAL_POSE:
        method_alignment = {
            "local_alignment": (
                "one precision/raw first-exact-common-pose SE(2), frozen for "
                "same-run precision raw/local outputs"
            ),
            "global_alignment": (
                "one precision/existing-EKF first-exact-common-pose SE(2), frozen "
                "for same-run existing/precision-global outputs"
            ),
            "yaw_alignment": (
                "the yaw of the same complete initial-pose SE(2); no separate "
                "yaw fit"
            ),
            "accuracy_estimator_association": (
                "exact same-run integer header-stamp intersection; no estimator "
                "interpolation"
            ),
            "accuracy_reference_association": (
                "GLIM interpolated to exact estimator header stamps only"
            ),
            "speed_run_role": (
                "runtime/non-intrusion/protocol comparison only; excluded from "
                "primary GLIM RMSE"
            ),
        }
    else:
        method_alignment = {
            "local_alignment": (
                "one speed/raw calibration-window SE(2) frozen for all local outputs"
            ),
            "global_alignment": (
                "one speed/existing-EKF calibration-window SE(2) frozen for all "
                "global outputs"
            ),
            "yaw_alignment": (
                "separate speed baseline circular offset, frozen per local/global group"
            ),
            "accuracy_estimator_association": (
                "historical interpolation onto common GLIM header stamps"
            ),
            "accuracy_reference_association": "native GLIM header stamps",
            "speed_run_role": "alignment baseline and A/B comparison",
        }
    result = {
        "label": args.label,
        "passed": not failed,
        "method": {
            "timestamps": "physical ROS header stamps; MCAP record time is not pose time",
            "alignment_mode": alignment_mode,
            **method_alignment,
            "scale": "not estimated or applied",
            "glim_caveat": "correlated LiDAR+IMU pseudo-ground-truth",
        },
        "primary_alignment": alignment_metadata,
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
        "map_fusion_publication_integrity": fusion_publication_integrity,
        "precision_timeline": precision_timeline,
        "outage_yaw_guard": precision_timeline["outage_yaw_guard"],
        "protocol": protocol,
        "outage": outage,
        "topic_counts": {"speed": speed["counts"], "precision": precision["counts"]},
        "checks": checks,
        "hard_gate_count": sum(item["category"] == "hard" for item in checks),
        "failed_hard_gate_count": len(failed),
    }
    if getattr(args, "plot_directory", None) is not None:
        result["plot_artifacts"] = generate_plot_artifacts(
            args,
            canonical,
            local_reference,
            local,
            local_alignment,
            local_yaw_offset,
            global_reference,
            global_trajectories,
            global_alignment,
            global_yaw_offset,
            evaluations,
            common_outage,
        )
    return result


def format_change(item: dict[str, float], unit: str) -> str:
    return (
        f"{item['old']:.4f} -> {item['new']:.4f} {unit} "
        f"({item['improvement_percent']:+.2f}% improvement)"
    )


def write_markdown(path: Path, result: dict[str, Any]) -> None:
    local = result["local_comparison"]
    global_result = result["global_comparison"]
    outage_yaw_guard = result["outage_yaw_guard"]
    outage_yaw_counters = outage_yaw_guard["counters"]
    publication_integrity = result["map_fusion_publication_integrity"]
    lines = [
        f"# {result['label']} isolated precision A/B",
        "",
        f"- result: **{'PASS' if result['passed'] else 'FAIL'}**",
        f"- alignment mode: `{result['method']['alignment_mode']}`",
        f"- local fixed XY: {format_change(local['fixed_xy_rmse_m'], 'm')}",
        f"- local full-shape XY: {format_change(local['full_shape_xy_rmse_m'], 'm')}",
        f"- local yaw: {format_change(local['yaw_rmse_deg'], 'deg')}",
        f"- global fixed XY: {format_change(global_result['fixed_xy_rmse_m'], 'm')}",
        f"- global full-shape XY: {format_change(global_result['full_shape_xy_rmse_m'], 'm')}",
        f"- global yaw: {format_change(global_result['yaw_rmse_deg'], 'deg')}",
        "- precision global native initialization delay: "
        f"{result['precision_timeline']['initialization_delay_sec']} s "
        "(1 Hz diagnostic observation: "
        f"{result['precision_timeline']['diagnostic_initialization_observed_delay_sec']} s)",
        f"- outage yaw guard states: {outage_yaw_guard['state_counts']}",
        "- outage yaw guard counters: accepted references "
        f"{outage_yaw_counters['accepted_reference_count']}, outages "
        f"{outage_yaw_counters['outage_count']}, recoveries "
        f"{outage_yaw_counters['recovery_count']}, invalid advances "
        f"{outage_yaw_counters['invalid_advance_count']}, suppressed invalid "
        f"{outage_yaw_counters['suppressed_invalid_count']}",
        "- outage yaw guard maximum offsets: applied "
        f"{outage_yaw_guard['maxima']['abs_applied_offset_rad']:.6f} rad, target "
        f"{outage_yaw_guard['maxima']['abs_target_offset_rad']:.6f} rad; "
        "observed recovery step-counter delta "
        f"{outage_yaw_guard['release']['observed_applied_step_count_delta']}",
        "",
    ]
    for run in ("speed", "precision"):
        publication = publication_integrity[run]
        counters = publication["counter_contract"]["final"]
        coverage = publication["publication_contract"]
        lines.append(
            f"- {run} map-fusion publication integrity: "
            f"{'PASS' if publication['valid'] else 'FAIL'}; strict drops "
            f"{counters[FUSION_PUBLICATION_COUNTER_KEYS[0]]}, covered odometry "
            f"coalesces {counters[FUSION_PUBLICATION_COUNTER_KEYS[1]]}, wall-timer "
            f"coalesces {counters[FUSION_PUBLICATION_COUNTER_KEYS[2]]}, exact raw "
            f"coverage {coverage['causal_raw_unique_stamps'] - coverage['missing_raw_unique_stamps']}"
            f"/{coverage['causal_raw_unique_stamps']} (stamp/record tails excluded "
            f"{coverage['raw_positive_stamp_tail_excluded']}/"
            f"{coverage['raw_positive_record_tail_excluded']})"
        )
    lines.append("")
    if result["method"]["alignment_mode"] == ALIGNMENT_EXACT_INITIAL_POSE:
        for group in ("local", "global"):
            alignment = result["primary_alignment"][group]
            baseline = alignment["initial_residuals"][alignment["baseline"]]
            lines.append(
                f"- {group} exact initial anchor: stamp "
                f"`{alignment['anchor_stamp_ns']}`; residual "
                f"{baseline['position_m']:.3e} m / {baseline['yaw_rad']:.3e} rad"
            )
        lines.extend(
            [
                "",
                "Accuracy uses exact same-run estimator header stamps; only GLIM "
                "is interpolated. The separate speed run is excluded from primary "
                "GLIM RMSE.",
                "",
            ]
        )
    lines.extend(["## RPE", ""])
    for distance, item in local["rpe"].items():
        lines.append(
            f"- {distance} m: translation {format_change(item['translation_rmse_m'], 'm')}; "
            f"yaw {format_change(item['yaw_rmse_deg'], 'deg')}"
        )
    artifacts = result.get("plot_artifacts")
    if artifacts:
        lines.extend(
            [
                "",
                "## Fixed/shared-alignment plots",
                "",
                "The plots use the same primary fixed/shared alignment as the "
                "fixed-frame metrics above: " + artifacts["alignment"] + ".",
                "",
            ]
        )
        for group in ("local", "global"):
            item = artifacts[group]
            png = Path(os.path.relpath(item["png"], start=path.parent)).as_posix()
            csv = Path(os.path.relpath(item["csv"], start=path.parent)).as_posix()
            lines.extend(
                [
                    f"### {group.capitalize()}",
                    "",
                    f"![{result['label']} {group} trajectory and errors]({png})",
                    "",
                    f"- aligned samples: [{Path(csv).name}]({csv})",
                    f"- samples: {item['samples']}",
                    "",
                ]
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
    parser.add_argument(
        "--alignment-mode",
        choices=(ALIGNMENT_FROZEN_CALIBRATION, ALIGNMENT_EXACT_INITIAL_POSE),
        default=ALIGNMENT_FROZEN_CALIBRATION,
        help=(
            "primary accuracy alignment; exact-initial-pose uses same-run exact "
            "estimator stamps, while the default preserves historical results"
        ),
    )
    parser.add_argument(
        "--calibration-start",
        type=float,
        help="epoch seconds; required only by frozen-calibration-window",
    )
    parser.add_argument(
        "--calibration-end",
        type=float,
        help="epoch seconds; required only by frozen-calibration-window",
    )
    parser.add_argument("--maximum-interpolation-gap-sec", type=float, default=0.1)
    parser.add_argument("--label", required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-markdown", type=Path, required=True)
    parser.add_argument(
        "--require-legacy-tracking-recovery",
        action="store_true",
        help=(
            "hard-gate TRACKING recovery in both legacy fusion runs after the "
            "longest Q4 outage"
        ),
    )
    parser.add_argument(
        "--plot-directory",
        type=Path,
        help=(
            "optional directory for local/global fixed/shared-alignment PNGs "
            "and auditable aligned CSVs"
        ),
    )
    return parser


def main() -> int:
    args = make_parser().parse_args()
    if args.alignment_mode == ALIGNMENT_FROZEN_CALIBRATION:
        if args.calibration_start is None or args.calibration_end is None:
            raise SystemExit(
                "--calibration-start and --calibration-end are required with "
                "--alignment-mode frozen-calibration-window"
            )
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
