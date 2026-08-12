#!/usr/bin/env python3
"""Evaluate planar localization trajectories against a GLIM trajectory.

GLIM's ``traj_lidar.txt`` uses the TUM trajectory convention::

    timestamp x y z qx qy qz qw

The evaluator associates poses by sensor/header timestamp, never by the MCAP
record timestamp.  It estimates only an SE(2) rigid transform (translation and
yaw); scale is deliberately not corrected.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import asdict, dataclass
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
import json
import math
import os
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


LOCAL_TOPIC = "/localization/gyro_lidar_odom"
EKF_TOPIC = "/localization/ekf_odom"
EXPECTED_TYPE = "nav_msgs/msg/Odometry"


def wrap_angle(value: np.ndarray | float) -> np.ndarray | float:
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def yaw_from_xyzw(quaternion: np.ndarray) -> np.ndarray:
    x = quaternion[:, 0]
    y = quaternion[:, 1]
    z = quaternion[:, 2]
    w = quaternion[:, 3]
    return np.arctan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def message_yaw(message: Any) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


@dataclass(frozen=True)
class Trajectory:
    stamp_ns: np.ndarray
    xy: np.ndarray
    yaw: np.ndarray

    def subset(self, mask: np.ndarray) -> "Trajectory":
        return Trajectory(self.stamp_ns[mask], self.xy[mask], self.yaw[mask])


@dataclass(frozen=True)
class ErrorStats:
    count: int
    rmse: float
    mean: float
    median: float
    p95: float
    p99: float
    maximum: float


@dataclass(frozen=True)
class Alignment:
    rotation: np.ndarray
    translation: np.ndarray
    yaw_rad: float
    singular_values: np.ndarray
    diagnostic_similarity_scale: float


@dataclass(frozen=True)
class DuplicateStats:
    raw_messages: int
    unique_stamps: int
    duplicate_messages: int
    duplicate_stamps: int
    maximum_multiplicity: int
    maximum_xy_spread_m: float
    maximum_yaw_spread_deg: float


def stats(values: np.ndarray) -> ErrorStats:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        raise RuntimeError("cannot summarize an empty error sequence")
    return ErrorStats(
        count=int(finite.size),
        rmse=float(np.sqrt(np.mean(finite * finite))),
        mean=float(np.mean(finite)),
        median=float(np.median(finite)),
        p95=float(np.quantile(finite, 0.95)),
        p99=float(np.quantile(finite, 0.99)),
        maximum=float(np.max(finite)),
    )


def read_glim_trajectory(path: Path) -> Trajectory:
    try:
        rows = []
        stamps = []
        for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            fields = line.split()
            if not fields:
                continue
            if len(fields) != 8:
                raise RuntimeError(
                    f"GLIM trajectory line {line_number} has {len(fields)} columns, expected 8"
                )
            stamps.append(
                int(
                    (Decimal(fields[0]) * Decimal(1_000_000_000)).to_integral_value(
                        rounding=ROUND_HALF_UP
                    )
                )
            )
            rows.append([float(value) for value in fields[1:]])
        values = np.asarray(rows, dtype=float)
    except (OSError, ValueError, InvalidOperation) as error:
        raise RuntimeError(f"failed to read GLIM trajectory {path}: {error}") from error
    if values.ndim != 2 or values.shape[1] != 7:
        raise RuntimeError(
            f"GLIM trajectory contains no valid 8-column rows, got shape {values.shape}: {path}"
        )
    if not np.all(np.isfinite(values)):
        raise RuntimeError(f"GLIM trajectory contains non-finite values: {path}")
    stamp_ns = np.asarray(stamps, dtype=np.int64)
    if np.any(np.diff(stamp_ns) <= 0):
        raise RuntimeError(f"GLIM timestamps are not strictly increasing: {path}")
    quaternion = values[:, 3:7]
    norm = np.linalg.norm(quaternion, axis=1)
    if np.any(norm < 1.0e-9):
        raise RuntimeError(f"GLIM trajectory contains a zero quaternion: {path}")
    quaternion = quaternion / norm[:, None]
    return Trajectory(stamp_ns, values[:, 0:2], yaw_from_xyzw(quaternion))


def resolve_bag(path: Path) -> Path:
    path = path.resolve()
    if path.is_file() and path.suffix == ".mcap":
        return path
    if not path.is_dir():
        raise RuntimeError(f"result bag path does not exist: {path}")
    if (path / "metadata.yaml").is_file():
        return path
    files = sorted(path.glob("*.mcap"))
    if len(files) == 1:
        return files[0]
    raise RuntimeError(
        f"expected metadata.yaml or exactly one MCAP under {path}, found {len(files)}"
    )


def open_bag(path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def maximum_group_spread(group: list[tuple[int, float, float, float]]) -> tuple[float, float]:
    if len(group) < 2:
        return 0.0, 0.0
    pose = np.asarray([[item[1], item[2], item[3]] for item in group], dtype=float)
    maximum_xy = 0.0
    maximum_yaw = 0.0
    for index in range(len(pose)):
        delta_xy = pose[index + 1 :, :2] - pose[index, :2]
        if delta_xy.size:
            maximum_xy = max(maximum_xy, float(np.max(np.linalg.norm(delta_xy, axis=1))))
        delta_yaw = np.abs(wrap_angle(pose[index + 1 :, 2] - pose[index, 2]))
        if delta_yaw.size:
            maximum_yaw = max(maximum_yaw, float(np.max(delta_yaw)))
    return maximum_xy, math.degrees(maximum_yaw)


def read_result_trajectories(
    bag_path: Path,
    evaluation_start_ns: int,
    evaluation_end_ns: int,
    duplicate_policy: str,
    topics: tuple[str, ...] = (LOCAL_TOPIC, EKF_TOPIC),
) -> tuple[dict[str, Trajectory], dict[str, DuplicateStats]]:
    reader = open_bag(resolve_bag(bag_path))
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    for topic in topics:
        if topic_types.get(topic) != EXPECTED_TYPE:
            raise RuntimeError(
                f"missing {topic} with type {EXPECTED_TYPE}; found {topic_types.get(topic)!r}"
            )
    message_class = get_message(EXPECTED_TYPE)
    groups: dict[str, dict[int, list[tuple[int, float, float, float]]]] = {
        topic: defaultdict(list) for topic in topics
    }
    while reader.has_next():
        topic, serialized, record_stamp_ns = reader.read_next()
        if topic not in groups:
            continue
        message = deserialize_message(serialized, message_class)
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        if stamp_ns <= 0:
            continue
        position = message.pose.pose.position
        values = (float(position.x), float(position.y), message_yaw(message))
        if not np.all(np.isfinite(values)):
            raise RuntimeError(f"{topic} contains a non-finite pose at {stamp_ns}")
        groups[topic][stamp_ns].append((record_stamp_ns, *values))

    trajectories: dict[str, Trajectory] = {}
    duplicate_stats: dict[str, DuplicateStats] = {}
    for topic, topic_groups in groups.items():
        if not topic_groups:
            raise RuntimeError(f"result bag contains no positive-stamp messages on {topic}")
        ordered_stamps = sorted(topic_groups)
        selected = []
        duplicate_stamps = 0
        maximum_multiplicity = 1
        maximum_xy_spread = 0.0
        maximum_yaw_spread = 0.0
        raw_messages = 0
        for stamp_ns in ordered_stamps:
            group = topic_groups[stamp_ns]
            raw_messages += len(group)
            maximum_multiplicity = max(maximum_multiplicity, len(group))
            if len(group) > 1:
                duplicate_stamps += 1
            if evaluation_start_ns <= stamp_ns <= evaluation_end_ns and len(group) > 1:
                spread_xy, spread_yaw = maximum_group_spread(group)
                maximum_xy_spread = max(maximum_xy_spread, spread_xy)
                maximum_yaw_spread = max(maximum_yaw_spread, spread_yaw)
            # These result bags can contain more than one pose at one header
            # stamp.  "last" represents the final value observed by a consumer;
            # "first" is exposed for sensitivity analysis.
            chosen = group[-1] if duplicate_policy == "last" else group[0]
            selected.append((stamp_ns, chosen[1], chosen[2], chosen[3]))
        selected_array = np.asarray(selected, dtype=float)
        trajectories[topic] = Trajectory(
            np.asarray(ordered_stamps, dtype=np.int64),
            selected_array[:, 1:3],
            selected_array[:, 3],
        )
        duplicate_stats[topic] = DuplicateStats(
            raw_messages=raw_messages,
            unique_stamps=len(ordered_stamps),
            duplicate_messages=raw_messages - len(ordered_stamps),
            duplicate_stamps=duplicate_stamps,
            maximum_multiplicity=maximum_multiplicity,
            maximum_xy_spread_m=maximum_xy_spread,
            maximum_yaw_spread_deg=maximum_yaw_spread,
        )
    return trajectories, duplicate_stats


def interpolate_trajectory(
    trajectory: Trajectory,
    query_stamp_ns: np.ndarray,
    maximum_gap_sec: float,
) -> tuple[Trajectory, np.ndarray]:
    query_stamp_ns = np.asarray(query_stamp_ns, dtype=np.int64)
    source_stamp_ns = trajectory.stamp_ns
    insertion = np.searchsorted(source_stamp_ns, query_stamp_ns, side="left")
    valid = (insertion > 0) & (insertion < len(source_stamp_ns))
    exact = np.zeros_like(valid)
    within = insertion < len(source_stamp_ns)
    exact[within] = source_stamp_ns[insertion[within]] == query_stamp_ns[within]
    valid |= exact

    left = np.clip(insertion - 1, 0, len(source_stamp_ns) - 1)
    right = np.clip(insertion, 0, len(source_stamp_ns) - 1)
    gap_sec = (source_stamp_ns[right] - source_stamp_ns[left]) * 1.0e-9
    valid &= exact | (gap_sec <= maximum_gap_sec)
    valid &= query_stamp_ns >= source_stamp_ns[0]
    valid &= query_stamp_ns <= source_stamp_ns[-1]
    query = query_stamp_ns[valid]
    if query.size < 3:
        raise RuntimeError("fewer than three timestamp-associated trajectory samples")

    origin_ns = min(int(source_stamp_ns[0]), int(query[0]))
    source_time = (source_stamp_ns - origin_ns) * 1.0e-9
    query_time = (query - origin_ns) * 1.0e-9
    xy = np.column_stack(
        (
            np.interp(query_time, source_time, trajectory.xy[:, 0]),
            np.interp(query_time, source_time, trajectory.xy[:, 1]),
        )
    )
    yaw = wrap_angle(np.interp(query_time, source_time, np.unwrap(trajectory.yaw)))
    return Trajectory(query, xy, yaw), valid


def fit_se2(estimate_xy: np.ndarray, reference_xy: np.ndarray) -> Alignment:
    if estimate_xy.shape != reference_xy.shape or estimate_xy.shape[1] != 2:
        raise RuntimeError("SE(2) alignment inputs must have matching Nx2 shapes")
    if len(estimate_xy) < 2:
        raise RuntimeError("SE(2) alignment requires at least two points")
    estimate_mean = np.mean(estimate_xy, axis=0)
    reference_mean = np.mean(reference_xy, axis=0)
    estimate_centered = estimate_xy - estimate_mean
    reference_centered = reference_xy - reference_mean
    covariance = estimate_centered.T @ reference_centered
    u, singular_values, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = reference_mean - rotation @ estimate_mean
    denominator = float(np.sum(estimate_centered * estimate_centered))
    similarity_scale = (
        float(np.sum(singular_values) / denominator)
        if denominator > 0.0
        else math.nan
    )
    return Alignment(
        rotation=rotation,
        translation=translation,
        yaw_rad=math.atan2(rotation[1, 0], rotation[0, 0]),
        singular_values=singular_values,
        diagnostic_similarity_scale=similarity_scale,
    )


def apply_alignment(trajectory: Trajectory, alignment: Alignment) -> Trajectory:
    xy = (alignment.rotation @ trajectory.xy.T).T + alignment.translation
    yaw = wrap_angle(trajectory.yaw + alignment.yaw_rad)
    return Trajectory(trajectory.stamp_ns, xy, yaw)


def trajectory_path_length(xy: np.ndarray) -> float:
    if len(xy) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))


def evaluate_aligned(
    estimate: Trajectory,
    reference: Trajectory,
    alignment_mask: np.ndarray,
) -> tuple[dict[str, Any], Trajectory, np.ndarray, np.ndarray]:
    alignment = fit_se2(estimate.xy[alignment_mask], reference.xy[alignment_mask])
    aligned = apply_alignment(estimate, alignment)
    position_delta = aligned.xy - reference.xy
    position_error = np.linalg.norm(position_delta, axis=1)
    yaw_error_deg = np.degrees(np.abs(wrap_angle(aligned.yaw - reference.yaw)))
    reference_forward = np.column_stack(
        (np.cos(reference.yaw), np.sin(reference.yaw))
    )
    reference_left = np.column_stack(
        (-np.sin(reference.yaw), np.cos(reference.yaw))
    )
    longitudinal_error = np.sum(position_delta * reference_forward, axis=1)
    lateral_error = np.sum(position_delta * reference_left, axis=1)
    reference_path = trajectory_path_length(reference.xy)
    estimate_path = trajectory_path_length(estimate.xy)
    result = {
        "alignment": {
            "yaw_deg": math.degrees(alignment.yaw_rad),
            "translation_x_m": float(alignment.translation[0]),
            "translation_y_m": float(alignment.translation[1]),
            "diagnostic_similarity_scale_not_applied": alignment.diagnostic_similarity_scale,
            "fit_samples": int(np.count_nonzero(alignment_mask)),
        },
        "position_error_m": asdict(stats(position_error)),
        "longitudinal_error_m": {
            "signed_mean": float(np.mean(longitudinal_error)),
            "absolute": asdict(stats(np.abs(longitudinal_error))),
        },
        "lateral_error_m": {
            "signed_mean": float(np.mean(lateral_error)),
            "absolute": asdict(stats(np.abs(lateral_error))),
        },
        "yaw_error_deg": asdict(stats(yaw_error_deg)),
        "reference_path_m": reference_path,
        "estimate_path_m": estimate_path,
        "path_ratio_estimate_over_reference": estimate_path / reference_path,
        "endpoint_position_error_m": float(position_error[-1]),
        "endpoint_yaw_error_deg": float(yaw_error_deg[-1]),
    }
    return result, aligned, position_error, yaw_error_deg


def relative_pose_errors(
    estimate: Trajectory,
    reference: Trajectory,
    distance_m: float,
) -> dict[str, Any]:
    cumulative = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(reference.xy, axis=0), axis=1)))
    )
    translation_errors = []
    yaw_errors_deg = []
    for start in range(len(cumulative)):
        end = int(np.searchsorted(cumulative, cumulative[start] + distance_m, side="left"))
        if end >= len(cumulative):
            break
        estimate_yaw = estimate.yaw[start]
        reference_yaw = reference.yaw[start]
        estimate_rotation_inverse = np.array(
            [
                [math.cos(estimate_yaw), math.sin(estimate_yaw)],
                [-math.sin(estimate_yaw), math.cos(estimate_yaw)],
            ]
        )
        reference_rotation_inverse = np.array(
            [
                [math.cos(reference_yaw), math.sin(reference_yaw)],
                [-math.sin(reference_yaw), math.cos(reference_yaw)],
            ]
        )
        estimate_delta = estimate_rotation_inverse @ (
            estimate.xy[end] - estimate.xy[start]
        )
        reference_delta = reference_rotation_inverse @ (
            reference.xy[end] - reference.xy[start]
        )
        translation_errors.append(float(np.linalg.norm(estimate_delta - reference_delta)))
        yaw_delta_error = wrap_angle(
            (estimate.yaw[end] - estimate.yaw[start])
            - (reference.yaw[end] - reference.yaw[start])
        )
        yaw_errors_deg.append(abs(math.degrees(float(yaw_delta_error))))
    if not translation_errors:
        return {"distance_m": distance_m, "count": 0}
    translation_array = np.asarray(translation_errors)
    return {
        "distance_m": distance_m,
        "count": len(translation_errors),
        "translation_error_m": asdict(stats(translation_array)),
        "translation_rmse_percent_of_distance": (
            stats(translation_array).rmse / distance_m * 100.0
        ),
        "yaw_error_deg": asdict(stats(np.asarray(yaw_errors_deg))),
    }


def first_distance_mask(reference_xy: np.ndarray, distance_m: float) -> np.ndarray:
    cumulative = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(reference_xy, axis=0), axis=1)))
    )
    end = int(np.searchsorted(cumulative, distance_m, side="left"))
    if end >= len(cumulative):
        raise RuntimeError(
            f"reference travels only {cumulative[-1]:.3f} m, less than initial alignment "
            f"distance {distance_m:.3f} m"
        )
    mask = np.zeros(len(reference_xy), dtype=bool)
    mask[: end + 1] = True
    return mask


def common_samples(
    reference: Trajectory,
    estimates: dict[str, Trajectory],
    maximum_gap_sec: float,
) -> tuple[Trajectory, dict[str, Trajectory]]:
    start_ns = max(trajectory.stamp_ns[0] for trajectory in estimates.values())
    end_ns = min(trajectory.stamp_ns[-1] for trajectory in estimates.values())
    range_mask = (reference.stamp_ns >= start_ns) & (reference.stamp_ns <= end_ns)
    candidate_stamps = reference.stamp_ns[range_mask]
    valid = np.ones(len(candidate_stamps), dtype=bool)
    interpolated: dict[str, Trajectory] = {}
    masks: dict[str, np.ndarray] = {}
    for name, trajectory in estimates.items():
        result, result_mask = interpolate_trajectory(
            trajectory, candidate_stamps, maximum_gap_sec
        )
        interpolated[name] = result
        masks[name] = result_mask
        valid &= result_mask
    if not np.all(valid):
        final_stamps = candidate_stamps[valid]
        for name, trajectory in estimates.items():
            interpolated[name], _ = interpolate_trajectory(
                trajectory, final_stamps, maximum_gap_sec
            )
    else:
        final_stamps = candidate_stamps
    reference_lookup = {int(stamp): index for index, stamp in enumerate(reference.stamp_ns)}
    reference_indices = np.asarray([reference_lookup[int(stamp)] for stamp in final_stamps])
    return reference.subset(np.isin(reference.stamp_ns, final_stamps)), interpolated


def samples_at_local_estimate_stamps(
    reference: Trajectory,
    estimate: Trajectory,
    maximum_gap_sec: float,
) -> tuple[Trajectory, dict[str, Trajectory]]:
    """Interpolate only GLIM at each exact local-estimate header stamp."""
    interpolated_reference, valid = interpolate_trajectory(
        reference, estimate.stamp_ns, maximum_gap_sec
    )
    selected_estimate = estimate.subset(valid)
    if not np.array_equal(interpolated_reference.stamp_ns, selected_estimate.stamp_ns):
        raise RuntimeError("reference/estimate timestamp association is inconsistent")
    return interpolated_reference, {"local": selected_estimate}


def evaluate_glim_internal_consistency(glim_directory: Path) -> dict[str, Any] | None:
    optimized_path = glim_directory / "traj_lidar.txt"
    odometry_path = glim_directory / "odom_lidar.txt"
    if not odometry_path.is_file():
        return None
    optimized = read_glim_trajectory(optimized_path)
    odometry = read_glim_trajectory(odometry_path)
    if not np.array_equal(optimized.stamp_ns, odometry.stamp_ns):
        raise RuntimeError("GLIM traj_lidar and odom_lidar timestamps differ")
    mask = np.ones(len(optimized.stamp_ns), dtype=bool)
    result, _, _, _ = evaluate_aligned(odometry, optimized, mask)
    return result


def write_csv(
    path: Path,
    reference: Trajectory,
    aligned: dict[str, Trajectory],
    position_errors: dict[str, np.ndarray],
    yaw_errors: dict[str, np.ndarray],
) -> None:
    names = tuple(aligned)
    header_columns = [
        "stamp_sec",
        "time_from_start_sec",
        "glim_x",
        "glim_y",
        "glim_yaw_rad",
    ]
    columns: list[np.ndarray] = [
        reference.stamp_ns * 1.0e-9,
        (reference.stamp_ns - reference.stamp_ns[0]) * 1.0e-9,
        reference.xy,
        reference.yaw,
    ]
    for name in names:
        header_columns.extend(
            [
                f"{name}_x",
                f"{name}_y",
                f"{name}_yaw_rad",
                f"{name}_position_error_m",
                f"{name}_yaw_error_deg",
            ]
        )
        columns.extend(
            [
                aligned[name].xy,
                aligned[name].yaw,
                position_errors[name],
                yaw_errors[name],
            ]
        )
    matrix = np.column_stack(columns)
    np.savetxt(
        path,
        matrix,
        delimiter=",",
        header=",".join(header_columns),
        comments="",
        fmt="%.9f",
    )


def make_plots(
    output_directory: Path,
    label: str,
    reference: Trajectory,
    aligned: dict[str, Trajectory],
    position_errors: dict[str, np.ndarray],
    yaw_errors: dict[str, np.ndarray],
    filename_suffix: str = "",
    alignment_title: str = "full-trajectory SE(2) alignment",
) -> None:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/glim_trajectory_evaluator_matplotlib")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"local": "#e7298a", "ekf": "#4565e5"}
    labels = {"local": "gyro_lidar_odom", "ekf": "ekf_odom"}
    figure, axis = plt.subplots(figsize=(10, 8))
    axis.plot(reference.xy[:, 0], reference.xy[:, 1], color="black", linewidth=2.0, label="GLIM")
    for name in aligned:
        axis.plot(
            aligned[name].xy[:, 0],
            aligned[name].xy[:, 1],
            color=colors[name],
            linewidth=1.1,
            label=labels[name],
        )
    axis.scatter(reference.xy[0, 0], reference.xy[0, 1], color="green", marker="o", label="start")
    axis.scatter(reference.xy[-1, 0], reference.xy[-1, 1], color="red", marker="x", label="end")
    axis.set_title(f"{label}: {alignment_title}")
    axis.set_xlabel("GLIM x [m]")
    axis.set_ylabel("GLIM y [m]")
    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_directory / f"trajectory_overlay{filename_suffix}.png", dpi=160)
    plt.close(figure)

    time = (reference.stamp_ns - reference.stamp_ns[0]) * 1.0e-9
    figure, axis = plt.subplots(figsize=(11, 5))
    for name in aligned:
        axis.plot(
            time,
            position_errors[name],
            color=colors[name],
            linewidth=1.0,
            label=labels[name],
        )
    axis.set_title(f"{label}: XY error after {alignment_title}")
    axis.set_xlabel("Time from common evaluation start [s]")
    axis.set_ylabel("XY error [m]")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_directory / f"position_error{filename_suffix}.png", dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(11, 5))
    for name in aligned:
        axis.plot(
            time,
            yaw_errors[name],
            color=colors[name],
            linewidth=1.0,
            label=labels[name],
        )
    axis.set_title(f"{label}: yaw error after {alignment_title}")
    axis.set_xlabel("Time from common evaluation start [s]")
    axis.set_ylabel("Absolute yaw error [deg]")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_directory / f"yaw_error{filename_suffix}.png", dpi=160)
    plt.close(figure)


def format_stats_line(item: dict[str, Any], unit: str) -> str:
    return (
        f"RMSE {item['rmse']:.4f} {unit}, median {item['median']:.4f} {unit}, "
        f"p95 {item['p95']:.4f} {unit}, max {item['maximum']:.4f} {unit}"
    )


def write_report(path: Path, label: str, metrics: dict[str, Any]) -> None:
    names = tuple(
        name for name in ("local", "ekf") if name in metrics["common"]
    )
    local_topic = metrics["inputs"].get("local_topic", LOCAL_TOPIC)
    frame_description = (
        "- frame: GLIMは`traj_lidar`、推定値はlocal odometryのbase frame。"
        "LiDAR/IMU-only評価ではbase frameを評価対象LiDAR frameへ一致させる"
        if names == ("local",)
        else "- frame: `traj_lidar`を使用（このrigでは`base_link -> lidar/0`がidentity）"
    )
    lines = [
        f"# {label} GLIM疑似真値評価",
        "",
        "## 評価条件",
        "",
        f"- GLIM: `{metrics['inputs']['glim_trajectory']}`",
        f"- result bag: `{metrics['inputs']['result_bag']}`",
        "- 対応: GLIM/ROS messageのsensor `header.stamp`で補間",
        frame_description,
        "- 座標合わせ: スケール固定のSE(2)。Sim(2) scale補正は不使用",
        f"- 共通評価sample: {metrics['common']['samples']}",
        "",
        "## 全共通区間を使ったSE(2)合わせ",
        "",
        "| 出力 | XY RMSE | XY median | XY p95 | XY max | yaw RMSE | yaw p95 | path比 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    if local_topic != LOCAL_TOPIC:
        lines.insert(7, f"- local trajectory topic: `{local_topic}`")
    if metrics["inputs"].get("sample_at_estimate_stamps", False):
        sampling_line = (
            "- sample基準: local estimateの各`header.stamp`をそのまま使用し、"
            "GLIMだけを各stampへ補間"
        )
        lines.insert(8 if local_topic != LOCAL_TOPIC else 7, sampling_line)
    for name in names:
        item = metrics["common"][name]["full_alignment"]
        position = item["position_error_m"]
        yaw = item["yaw_error_deg"]
        lines.append(
            f"| {name} | {position['rmse']:.4f} m | {position['median']:.4f} m | "
            f"{position['p95']:.4f} m | {position['maximum']:.4f} m | "
            f"{yaw['rmse']:.3f} deg | {yaw['p95']:.3f} deg | "
            f"{item['path_ratio_estimate_over_reference']:.5f} |"
        )
    lines.extend(
        [
            "",
            "## 先頭20 mだけを使った座標合わせ",
            "",
            "この値は後半のdriftを全軌跡fitで平均化しません。",
            "",
            "| 出力 | XY RMSE | XY p95 | XY max | 終点XY | 終点yaw |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for name in names:
        item = metrics["common"][name]["initial_distance_alignment"]
        position = item["position_error_m"]
        lines.append(
            f"| {name} | {position['rmse']:.4f} m | {position['p95']:.4f} m | "
            f"{position['maximum']:.4f} m | {item['endpoint_position_error_m']:.4f} m | "
            f"{item['endpoint_yaw_error_deg']:.3f} deg |"
        )
    if "calibration_window_alignment" in metrics["common"]["local"]:
        lines.extend(
            [
                "",
                "## 指定した初期時間窓だけを使った座標合わせ",
                "",
                "この変換は後半区間を使わずに固定しています。",
                "",
                "| 出力 | fit sample | XY RMSE | XY p95 | XY max | 終点XY | 終点yaw |",
                "|---|---:|---:|---:|---:|---:|---:|",
            ]
        )
        for name in names:
            item = metrics["common"][name]["calibration_window_alignment"]
            position = item["position_error_m"]
            lines.append(
                f"| {name} | {item['alignment']['fit_samples']} | {position['rmse']:.4f} m | "
                f"{position['p95']:.4f} m | {position['maximum']:.4f} m | "
                f"{item['endpoint_position_error_m']:.4f} m | "
                f"{item['endpoint_yaw_error_deg']:.3f} deg |"
            )
    lines.extend(["", "## Relative Pose Error", ""])
    lines.append("| 出力 | 区間 | translation RMSE | p95 | yaw RMSE | yaw p95 |")
    lines.append("|---|---:|---:|---:|---:|---:|")
    for name in names:
        for item in metrics["common"][name]["rpe"]:
            if item["count"] == 0:
                continue
            translation = item["translation_error_m"]
            yaw = item["yaw_error_deg"]
            lines.append(
                f"| {name} | {item['distance_m']:.0f} m | {translation['rmse']:.4f} m | "
                f"{translation['p95']:.4f} m | {yaw['rmse']:.3f} deg | "
                f"{yaw['p95']:.3f} deg |"
            )
    consistency = metrics.get("glim_internal_consistency")
    if consistency:
        lines.extend(
            [
                "",
                "## GLIM参照軌跡の内部差",
                "",
                "`odom_lidar`を`traj_lidar`へSE(2)で合わせた差です。これはGLIMの",
                "不確かさの厳密な上限ではなく、global optimizationが形状へ与えた",
                "補正量の目安です。",
                "",
                f"- XY: {format_stats_line(consistency['position_error_m'], 'm')}",
                f"- yaw: {format_stats_line(consistency['yaw_error_deg'], 'deg')}",
            ]
        )
    lines.extend(
        [
            "",
            "## 制約",
            "",
            "GLIMも評価対象も同じLiDAR/IMUを使用するため、誤差は",
            "相関しています。本結果は独立ground truthに対する絶対精度ではなく、",
            "GLIMを比較参照とした平面軌跡の整合度です。Z/roll/pitchは",
            "planar estimatorの主評価から除外しています。",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    glim_directory = args.glim_dir.resolve()
    glim_path = glim_directory / args.glim_trajectory
    reference = read_glim_trajectory(glim_path)
    local_only = getattr(args, "local_only", False)
    local_topic = getattr(args, "local_topic", LOCAL_TOPIC)
    sample_at_estimate_stamps = getattr(args, "sample_at_estimate_stamps", False)
    if sample_at_estimate_stamps and not local_only:
        raise RuntimeError("--sample-at-estimate-stamps requires --local-only")
    requested_topics = (local_topic,) if local_only else (local_topic, EKF_TOPIC)
    result_trajectories, duplicates = read_result_trajectories(
        args.result_bag,
        int(reference.stamp_ns[0]),
        int(reference.stamp_ns[-1]),
        args.duplicate_policy,
        requested_topics,
    )
    estimates = {
        "local": result_trajectories[local_topic],
    }
    if not local_only:
        estimates["ekf"] = result_trajectories[EKF_TOPIC]
    names = tuple(estimates)
    if sample_at_estimate_stamps:
        common_reference, common_estimates = samples_at_local_estimate_stamps(
            reference, estimates["local"], args.maximum_interpolation_gap_sec
        )
    else:
        common_reference, common_estimates = common_samples(
            reference, estimates, args.maximum_interpolation_gap_sec
        )
    initial_mask = first_distance_mask(common_reference.xy, args.initial_alignment_distance_m)
    full_mask = np.ones(len(common_reference.stamp_ns), dtype=bool)
    calibration_mask = None
    if args.calibration_start_stamp_sec is not None:
        calibration_start_ns = int(round(args.calibration_start_stamp_sec * 1.0e9))
        calibration_end_ns = int(round(args.calibration_end_stamp_sec * 1.0e9))
        calibration_mask = (
            (common_reference.stamp_ns >= calibration_start_ns)
            & (common_reference.stamp_ns <= calibration_end_ns)
        )
        if np.count_nonzero(calibration_mask) < 3:
            raise RuntimeError("calibration stamp window contains fewer than three samples")
    common_metrics: dict[str, Any] = {
        "samples": len(common_reference.stamp_ns),
        "start_stamp_sec": float(common_reference.stamp_ns[0] * 1.0e-9),
        "end_stamp_sec": float(common_reference.stamp_ns[-1] * 1.0e-9),
        "duration_sec": float(
            (common_reference.stamp_ns[-1] - common_reference.stamp_ns[0]) * 1.0e-9
        ),
        "initial_alignment_distance_m": args.initial_alignment_distance_m,
    }
    aligned_for_plot: dict[str, Trajectory] = {}
    position_errors_for_plot: dict[str, np.ndarray] = {}
    yaw_errors_for_plot: dict[str, np.ndarray] = {}
    calibration_aligned: dict[str, Trajectory] = {}
    calibration_position_errors: dict[str, np.ndarray] = {}
    calibration_yaw_errors: dict[str, np.ndarray] = {}
    for name in names:
        trajectory = common_estimates[name]
        full_result, full_aligned, position_error, yaw_error = evaluate_aligned(
            trajectory, common_reference, full_mask
        )
        initial_result, _, _, _ = evaluate_aligned(
            trajectory, common_reference, initial_mask
        )
        common_metrics[name] = {
            "full_alignment": full_result,
            "initial_distance_alignment": initial_result,
            "rpe": [
                relative_pose_errors(trajectory, common_reference, distance)
                for distance in args.rpe_distances_m
            ],
        }
        if calibration_mask is not None:
            (
                calibration_result,
                calibration_trajectory,
                calibration_position_error,
                calibration_yaw_error,
            ) = evaluate_aligned(
                trajectory, common_reference, calibration_mask
            )
            common_metrics[name]["calibration_window_alignment"] = calibration_result
            calibration_aligned[name] = calibration_trajectory
            calibration_position_errors[name] = calibration_position_error
            calibration_yaw_errors[name] = calibration_yaw_error
        aligned_for_plot[name] = full_aligned
        position_errors_for_plot[name] = position_error
        yaw_errors_for_plot[name] = yaw_error

    output_directory = args.output_dir.resolve()
    output_directory.mkdir(parents=True, exist_ok=False)
    metrics: dict[str, Any] = {
        "label": args.label,
        "method": {
            "alignment": "SE(2), estimate to GLIM, no scale correction",
            "timestamp": (
                "ROS header.stamp to GLIM timestamp, linear XY and "
                "unwrapped-yaw interpolation"
            ),
            "duplicate_stamp_policy": (
                f"{args.duplicate_policy} publication in MCAP record order"
            ),
            "maximum_interpolation_gap_sec": args.maximum_interpolation_gap_sec,
        },
        "inputs": {
            "glim_trajectory": str(glim_path),
            "result_bag": str(args.result_bag.resolve()),
        },
        "glim": {
            "samples": len(reference.stamp_ns),
            "start_stamp_sec": float(reference.stamp_ns[0] * 1.0e-9),
            "end_stamp_sec": float(reference.stamp_ns[-1] * 1.0e-9),
            "duration_sec": float(
                (reference.stamp_ns[-1] - reference.stamp_ns[0]) * 1.0e-9
            ),
            "xy_path_m": trajectory_path_length(reference.xy),
        },
        "duplicates": {
            name: asdict(duplicates[topic])
            for name, topic in zip(names, requested_topics, strict=True)
        },
        "common": common_metrics,
        "glim_internal_consistency": evaluate_glim_internal_consistency(glim_directory),
    }
    if local_only:
        metrics["method"]["evaluated_topics"] = list(requested_topics)
    elif local_topic != LOCAL_TOPIC:
        metrics["method"]["evaluated_topics"] = list(requested_topics)
    if local_topic != LOCAL_TOPIC:
        metrics["inputs"]["local_topic"] = local_topic
    if sample_at_estimate_stamps:
        metrics["method"]["sampling"] = (
            "exact local estimate header.stamps; GLIM reference interpolated "
            "to estimate stamps; estimate poses not interpolated"
        )
        metrics["inputs"]["sample_at_estimate_stamps"] = True
    (output_directory / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    write_csv(
        output_directory / "aligned_samples.csv",
        common_reference,
        aligned_for_plot,
        position_errors_for_plot,
        yaw_errors_for_plot,
    )
    write_report(output_directory / "REPORT.md", args.label, metrics)
    if calibration_mask is not None:
        write_csv(
            output_directory / "aligned_samples_calibration.csv",
            common_reference,
            calibration_aligned,
            calibration_position_errors,
            calibration_yaw_errors,
        )
    if not args.no_plots:
        make_plots(
            output_directory,
            args.label,
            common_reference,
            aligned_for_plot,
            position_errors_for_plot,
            yaw_errors_for_plot,
        )
        if calibration_mask is not None:
            make_plots(
                output_directory,
                args.label,
                common_reference,
                calibration_aligned,
                calibration_position_errors,
                calibration_yaw_errors,
                filename_suffix="_calibration",
                alignment_title="fixed early calibration-window SE(2) alignment",
            )
    return metrics


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Align ROS localization output to GLIM and calculate planar ATE/RPE."
    )
    parser.add_argument("--result-bag", required=True, type=Path)
    parser.add_argument("--glim-dir", required=True, type=Path)
    parser.add_argument("--glim-trajectory", default="traj_lidar.txt")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--label", required=True)
    parser.add_argument("--initial-alignment-distance-m", type=float, default=20.0)
    parser.add_argument(
        "--rpe-distances-m", type=float, nargs="+", default=[10.0, 50.0, 100.0]
    )
    parser.add_argument("--maximum-interpolation-gap-sec", type=float, default=0.1)
    parser.add_argument("--duplicate-policy", choices=("last", "first"), default="last")
    parser.add_argument(
        "--local-topic",
        default=LOCAL_TOPIC,
        help=(
            "Odometry topic evaluated as the local trajectory "
            f"(default: {LOCAL_TOPIC})"
        ),
    )
    parser.add_argument(
        "--local-only",
        action="store_true",
        help=(
            "evaluate only the odometry selected by --local-topic; use this "
            "for LiDAR/IMU-only runs that do not publish /localization/ekf_odom"
        ),
    )
    parser.add_argument(
        "--sample-at-estimate-stamps",
        action="store_true",
        help=(
            "use each exact local-estimate header stamp as an evaluation sample "
            "and interpolate only GLIM; requires --local-only"
        ),
    )
    parser.add_argument("--calibration-start-stamp-sec", type=float)
    parser.add_argument("--calibration-end-stamp-sec", type=float)
    parser.add_argument("--no-plots", action="store_true")
    return parser


def main() -> int:
    parser = build_argument_parser()
    args = parser.parse_args()
    if args.initial_alignment_distance_m <= 0.0:
        parser.error("--initial-alignment-distance-m must be positive")
    if any(distance <= 0.0 for distance in args.rpe_distances_m):
        parser.error("--rpe-distances-m values must be positive")
    if args.maximum_interpolation_gap_sec <= 0.0:
        parser.error("--maximum-interpolation-gap-sec must be positive")
    if args.sample_at_estimate_stamps and not args.local_only:
        parser.error("--sample-at-estimate-stamps requires --local-only")
    if (args.calibration_start_stamp_sec is None) != (
        args.calibration_end_stamp_sec is None
    ):
        parser.error("calibration start/end stamp options must be specified together")
    if (
        args.calibration_start_stamp_sec is not None
        and args.calibration_end_stamp_sec <= args.calibration_start_stamp_sec
    ):
        parser.error("calibration end stamp must be greater than its start stamp")
    if args.output_dir.exists():
        parser.error(f"--output-dir already exists: {args.output_dir}")
    try:
        metrics = evaluate(args)
    except Exception as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    names = ("local",) if args.local_only else ("local", "ekf")
    for name in names:
        item = metrics["common"][name]["full_alignment"]
        print(
            f"{name}: XY RMSE={item['position_error_m']['rmse']:.4f} m, "
            f"p95={item['position_error_m']['p95']:.4f} m, "
            f"yaw RMSE={item['yaw_error_deg']['rmse']:.3f} deg"
        )
    print(f"wrote: {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
