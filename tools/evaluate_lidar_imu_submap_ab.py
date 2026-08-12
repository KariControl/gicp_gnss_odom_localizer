#!/usr/bin/env python3
"""Evaluate LiDAR/IMU external-submap precision-local odometry against GLIM.

The control and precision runs are independent executions, so their accepted
scan sets can differ by a stable whole-frame callback phase.  Estimates are
never interpolated to hide that difference.  GLIM alone is interpolated to
each run's accepted physical header stamps.  One exact SE(2), determined by
mapping the first control pose that has a native physical-stamp counterpart in
all three streams to GLIM at that stamp, is frozen and reused for control raw,
precision raw, and precision local.  Scale is never estimated or applied.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import types
from typing import Any

import numpy as np


RPE_DISTANCES = (10.0, 50.0, 100.0)


def load_module(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"cannot read {path}: {error}") from error
    if not isinstance(value, dict):
        raise RuntimeError(f"{path} does not contain a JSON object")
    return value


def read_env(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise RuntimeError(f"cannot read {path}: {error}") from error
    for line in lines:
        if not line or line.lstrip().startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key.strip()] = value.strip()
    return result


def resolve_bag(run: Path) -> Path:
    if run.is_dir() and (run / "metadata.yaml").is_file():
        return run
    nested = run / "localization_output"
    if nested.is_dir() and (nested / "metadata.yaml").is_file():
        return nested
    if run.is_file() and run.suffix == ".mcap":
        return run
    raise RuntimeError(f"cannot resolve localization output bag from {run}")


def read_control_pose(bag: Path, topic: str, canonical: Any) -> Any:
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as error:
        raise RuntimeError("source ROS 2 and this workspace before evaluation") from error
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(resolve_bag(bag)), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    expected = "nav_msgs/msg/Odometry"
    if types.get(topic) != expected:
        raise RuntimeError(f"{bag} lacks {topic} ({expected})")
    message_class = get_message(expected)
    groups: dict[int, list[tuple[float, float, float]]] = {}
    while reader.has_next():
        name, serialized, _ = reader.read_next()
        if name != topic:
            continue
        message = deserialize_message(serialized, message_class)
        stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        if stamp <= 0:
            continue
        pose = message.pose.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z
                   + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2),
        )
        values = np.asarray([pose.position.x, pose.position.y, yaw], dtype=float)
        if not np.all(np.isfinite(values)):
            raise RuntimeError(f"non-finite {topic} pose at {stamp}")
        groups.setdefault(stamp, []).append(tuple(map(float, values)))
    if len(groups) < 3:
        raise RuntimeError(f"fewer than three positive-stamp messages on {topic}")
    stamps = np.asarray(sorted(groups), dtype=np.int64)
    selected = np.asarray([groups[int(stamp)][-1] for stamp in stamps], dtype=float)
    return canonical.Trajectory(stamps, selected[:, :2], selected[:, 2])


def records_trajectory(records: list[Any], canonical: Any) -> Any:
    stamps = np.asarray([item.stamp_ns for item in records], dtype=np.int64)
    xy = np.asarray([[item.x, item.y] for item in records], dtype=float)
    yaw = np.asarray([item.yaw for item in records], dtype=float)
    return canonical.Trajectory(stamps, xy, yaw)


def associate_glim(canonical: Any, estimate: Any, glim: Any, maximum_gap: float):
    reference, valid = canonical.interpolate_trajectory(
        glim, estimate.stamp_ns, maximum_gap
    )
    return estimate.subset(valid), reference, float(np.count_nonzero(valid) / len(valid))


def fixed_metrics(canonical: Any, estimate: Any, reference: Any, alignment: Any) -> dict[str, Any]:
    aligned = canonical.apply_alignment(estimate, alignment)
    position = np.linalg.norm(aligned.xy - reference.xy, axis=1)
    yaw = np.degrees(np.abs(canonical.wrap_angle(aligned.yaw - reference.yaw)))
    reference_path = canonical.trajectory_path_length(reference.xy)
    estimate_path = canonical.trajectory_path_length(estimate.xy)
    return {
        "samples": len(estimate.stamp_ns),
        "duration_sec": (int(estimate.stamp_ns[-1]) - int(estimate.stamp_ns[0])) * 1.0e-9,
        "reference_path_m": reference_path,
        "estimate_path_m": estimate_path,
        "path_ratio_estimate_over_reference": estimate_path / reference_path,
        "position_error_m": asdict(canonical.stats(position)),
        "yaw_error_deg": asdict(canonical.stats(yaw)),
        "endpoint_position_error_m": float(position[-1]),
        "endpoint_yaw_error_deg": float(yaw[-1]),
        "rpe": {
            str(int(distance)): canonical.relative_pose_errors(aligned, reference, distance)
            for distance in RPE_DISTANCES
        },
    }, aligned, position, yaw


def comparison(old: float, new: float) -> dict[str, float]:
    return {
        "old": float(old), "new": float(new),
        "improvement_percent": (1.0 - new / old) * 100.0 if old > 0.0 else -math.inf,
    }


def compare_metrics(old: dict[str, Any], new: dict[str, Any]) -> dict[str, Any]:
    result = {
        "position_rmse_m": comparison(
            old["position_error_m"]["rmse"], new["position_error_m"]["rmse"]
        ),
        "yaw_rmse_deg": comparison(
            old["yaw_error_deg"]["rmse"], new["yaw_error_deg"]["rmse"]
        ),
        "endpoint_position_error_m": comparison(
            old["endpoint_position_error_m"], new["endpoint_position_error_m"]
        ),
        "rpe": {},
    }
    for distance in map(str, map(int, RPE_DISTANCES)):
        old_rpe, new_rpe = old["rpe"][distance], new["rpe"][distance]
        if old_rpe.get("count", 0) and new_rpe.get("count", 0):
            result["rpe"][distance] = {
                "translation_rmse_m": comparison(
                    old_rpe["translation_error_m"]["rmse"],
                    new_rpe["translation_error_m"]["rmse"],
                ),
                "yaw_rmse_deg": comparison(
                    old_rpe["yaw_error_deg"]["rmse"], new_rpe["yaw_error_deg"]["rmse"]
                ),
            }
    return result


def exact_raw_ab(canonical: Any, validator: Any, control: Any, precision: Any,
                 tolerance_ns: int) -> dict[str, Any]:
    pairs, delta = validator.nearest_unique_association(
        control.stamp_ns, precision.stamp_ns, tolerance_ns
    )
    if len(pairs) < 3:
        return {
            "samples": len(pairs), "coverage": len(pairs) / max(len(control.stamp_ns),
                                                                 len(precision.stamp_ns)),
            "stamp_delta_ns_maximum": None,
        }
    control_index = np.asarray([item[0] for item in pairs], dtype=int)
    precision_index = np.asarray([item[1] for item in pairs], dtype=int)
    xy = np.linalg.norm(
        control.xy[control_index] - precision.xy[precision_index], axis=1
    )
    yaw = np.degrees(np.abs(canonical.wrap_angle(
        control.yaw[control_index] - precision.yaw[precision_index]
    )))
    return {
        "samples": len(pairs),
        "coverage": len(pairs) / max(len(control.stamp_ns), len(precision.stamp_ns)),
        "stamp_delta_ns_maximum": int(np.max(np.abs(delta))),
        "xy_difference_m": asdict(canonical.stats(xy)),
        "yaw_difference_deg": asdict(canonical.stats(yaw)),
    }


def config_contract(control: dict[str, str], precision: dict[str, str], sensor: str):
    fields = (
        "sensor", "bag", "glim_traj_sha256", "deskew",
        "use_deskew", "rate", "points_topic", "imu_topic",
        "imu_param_sha256", "odom_override_sha256", "base_odom_param_sha256",
    )
    if sensor == "mid360":
        fields += ("gicp_epsilon", "mid_yaw_policy")
    mismatches = {
        field: {"control": control.get(field), "precision": precision.get(field)}
        for field in fields if control.get(field) != precision.get(field)
    }
    sensor_ok = control.get("sensor") == precision.get("sensor") == sensor
    mid_ok = sensor != "mid360" or (
        control.get("mid_yaw_policy") == precision.get("mid_yaw_policy")
        == "fixed-bias-direct" and control.get("gicp_epsilon")
        == precision.get("gicp_epsilon") == "default"
    )
    return {"passed": sensor_ok and mid_ok and not mismatches,
            "mismatches": mismatches, "mid_fixed_bias_direct": mid_ok}


def absolute_pass(metrics: dict[str, Any]) -> tuple[bool, list[str]]:
    path = metrics["reference_path_m"]
    failures = []
    requirements = (
        ("path_ratio", 0.95 <= metrics["path_ratio_estimate_over_reference"] <= 1.05),
        ("position_rmse", metrics["position_error_m"]["rmse"] <= max(1.0, 0.01 * path)),
        ("position_p95", metrics["position_error_m"]["p95"] <= max(3.0, 0.03 * path)),
        ("yaw_rmse", metrics["yaw_error_deg"]["rmse"] <= 3.0),
        ("yaw_p95", metrics["yaw_error_deg"]["p95"] <= 5.0),
        ("endpoint_position", metrics["endpoint_position_error_m"] <= max(2.0, 0.02 * path)),
    )
    failures.extend(name for name, passed in requirements if not passed)
    evaluated = 0
    for distance, item in metrics["rpe"].items():
        if item.get("count", 0) == 0:
            continue
        evaluated += 1
        if item["translation_error_m"]["rmse"] > 0.05 * float(distance):
            failures.append(f"rpe_{distance}m_translation")
        if item["yaw_error_deg"]["rmse"] > 2.5:
            failures.append(f"rpe_{distance}m_yaw")
    if evaluated == 0:
        failures.append("no_evaluable_rpe")
    return not failures, failures


def nonregression(old: float, new: float, absolute: float) -> bool:
    return new <= old * 1.05 + absolute


def exact_pose_alignment(
    canonical: Any,
    estimate_xy: np.ndarray,
    estimate_yaw: float,
    reference_xy: np.ndarray,
    reference_yaw: float,
) -> Any:
    """Return the unique SE(2) mapping one complete estimate pose to reference."""
    yaw = float(canonical.wrap_angle(float(reference_yaw) - float(estimate_yaw)))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]], dtype=float)
    translation = np.asarray(reference_xy, dtype=float) - rotation @ np.asarray(
        estimate_xy, dtype=float
    )
    # Similarity scale is deliberately neither estimated nor applied.  NaN is
    # internal only; the serialized primary-alignment schema reports null.
    return canonical.Alignment(
        rotation=rotation,
        translation=translation,
        yaw_rad=yaw,
        singular_values=np.asarray([], dtype=float),
        diagnostic_similarity_scale=math.nan,
    )


def first_common_physical_anchor(
    validator: Any,
    control_stamps: np.ndarray,
    precision_raw_stamps: np.ndarray,
    precision_local_stamps: np.ndarray,
    tolerance_ns: int,
) -> dict[str, Any]:
    """Find the first one-to-one three-stream physical-stamp association.

    Integer equality is preferred.  Nearest unique association up to the
    configured (normally 1 us) tolerance exists only for epoch/double
    serialization round-off; it is not estimate interpolation.
    """
    raw_pairs, raw_delta = validator.nearest_unique_association(
        control_stamps, precision_raw_stamps, tolerance_ns
    )
    local_pairs, local_delta = validator.nearest_unique_association(
        control_stamps, precision_local_stamps, tolerance_ns
    )
    raw_by_control = {
        control_index: (raw_index, int(delta))
        for (control_index, raw_index), delta in zip(raw_pairs, raw_delta)
    }
    local_by_control = {
        control_index: (local_index, int(delta))
        for (control_index, local_index), delta in zip(local_pairs, local_delta)
    }
    common = sorted(
        set(raw_by_control) & set(local_by_control),
        key=lambda index: int(control_stamps[index]),
    )
    if not common:
        raise RuntimeError(
            "no native physical stamp is common to all three streams within "
            f"{tolerance_ns} ns"
        )
    control_index = common[0]
    raw_index, raw_control_minus_stream_ns = raw_by_control[control_index]
    local_index, local_control_minus_stream_ns = local_by_control[control_index]
    control_stamp_ns = int(control_stamps[control_index])
    stream_stamp_ns = {
        "control_raw": control_stamp_ns,
        "precision_raw": int(precision_raw_stamps[raw_index]),
        "precision_local": int(precision_local_stamps[local_index]),
    }
    stream_delta_from_control_ns = {
        "control_raw": 0,
        "precision_raw": -raw_control_minus_stream_ns,
        "precision_local": -local_control_minus_stream_ns,
    }
    return {
        "control_index": control_index,
        "precision_raw_index": raw_index,
        "precision_local_index": local_index,
        "control_stamp_ns": control_stamp_ns,
        "stream_stamp_ns": stream_stamp_ns,
        "stream_delta_from_control_ns": stream_delta_from_control_ns,
        "maximum_absolute_delta_ns": max(
            abs(value) for value in stream_delta_from_control_ns.values()
        ),
        "all_integer_stamps_equal": len(set(stream_stamp_ns.values())) == 1,
        "tolerance_ns": tolerance_ns,
    }


def exact_anchor_residual(
    canonical: Any, aligned: Any, reference: Any, anchor_index: int
) -> dict[str, Any]:
    if int(aligned.stamp_ns[anchor_index]) != int(reference.stamp_ns[anchor_index]):
        return {
            "same_stamp": False,
            "stamp_ns": int(aligned.stamp_ns[anchor_index]),
            "position_m": math.inf,
            "yaw_rad": math.inf,
            "yaw_deg": math.inf,
        }
    position = float(np.linalg.norm(
        aligned.xy[anchor_index] - reference.xy[anchor_index]
    ))
    yaw = abs(float(canonical.wrap_angle(
        aligned.yaw[anchor_index] - reference.yaw[anchor_index]
    )))
    return {
        "same_stamp": True,
        "stamp_ns": int(aligned.stamp_ns[anchor_index]),
        "position_m": position,
        "yaw_rad": yaw,
        "yaw_deg": math.degrees(yaw),
    }


def anchor_sensitivity(
    canonical: Any,
    estimate: Any,
    reference: Any,
    primary_alignment: Any,
    primary_anchor_index: int,
    horizon_sec: float = 2.0,
) -> dict[str, Any]:
    """Diagnostic sensitivity of exact-pose anchors selected in the first 2 s."""
    if not np.array_equal(estimate.stamp_ns, reference.stamp_ns):
        raise RuntimeError("anchor sensitivity requires paired estimate/reference stamps")
    anchor_ns = int(estimate.stamp_ns[primary_anchor_index])
    end_ns = anchor_ns + int(round(horizon_sec * 1.0e9))
    indices = np.flatnonzero(
        (estimate.stamp_ns >= anchor_ns) & (estimate.stamp_ns <= end_ns)
    )
    if len(indices) == 0:
        raise RuntimeError("anchor sensitivity contains no candidate")
    translation_delta = []
    yaw_delta_deg = []
    position_rmse = []
    yaw_rmse_deg = []
    exact_position_residual = []
    exact_yaw_residual = []
    offsets_sec = []
    for index in indices:
        alignment = exact_pose_alignment(
            canonical,
            estimate.xy[index],
            float(estimate.yaw[index]),
            reference.xy[index],
            float(reference.yaw[index]),
        )
        aligned = canonical.apply_alignment(estimate, alignment)
        translation_delta.append(
            float(np.linalg.norm(alignment.translation - primary_alignment.translation))
        )
        yaw_delta_deg.append(
            abs(math.degrees(float(canonical.wrap_angle(
                alignment.yaw_rad - primary_alignment.yaw_rad
            ))))
        )
        position_rmse.append(
            float(np.sqrt(np.mean(np.sum(np.square(aligned.xy - reference.xy), axis=1))))
        )
        yaw_rmse_deg.append(
            float(np.sqrt(np.mean(np.square(np.degrees(canonical.wrap_angle(
                aligned.yaw - reference.yaw
            ))))))
        )
        exact_position_residual.append(
            float(np.linalg.norm(aligned.xy[index] - reference.xy[index]))
        )
        exact_yaw_residual.append(
            abs(float(canonical.wrap_angle(aligned.yaw[index] - reference.yaw[index])))
        )
        offsets_sec.append(
            (int(estimate.stamp_ns[index]) - anchor_ns) * 1.0e-9
        )
    return {
        "role": "secondary_diagnostic_not_used_by_gates_or_primary_plots",
        "horizon_sec": horizon_sec,
        "candidate_count": len(indices),
        "anchor_offset_sec": asdict(canonical.stats(np.asarray(offsets_sec))),
        "transform_translation_delta_m": asdict(
            canonical.stats(np.asarray(translation_delta))
        ),
        "transform_yaw_delta_deg": asdict(canonical.stats(np.asarray(yaw_delta_deg))),
        "control_full_position_rmse_m": asdict(
            canonical.stats(np.asarray(position_rmse))
        ),
        "control_full_yaw_rmse_deg": asdict(
            canonical.stats(np.asarray(yaw_rmse_deg))
        ),
        "anchor_exact_position_residual_m_maximum": max(exact_position_residual),
        "anchor_exact_yaw_residual_rad_maximum": max(exact_yaw_residual),
    }


def secondary_distance_fit(
    canonical: Any,
    associated: dict[str, tuple[Any, Any, float]],
    distance_m: float,
) -> dict[str, Any]:
    """Retain the former first-distance fit only as a non-gating diagnostic."""
    control_estimate, control_reference, _ = associated["control_raw"]
    try:
        mask = canonical.first_distance_mask(control_reference.xy, distance_m)
    except RuntimeError as error:
        return {
            "role": "secondary_diagnostic_not_used_by_gates_or_primary_plots",
            "available": False,
            "distance_m": distance_m,
            "reason": str(error),
        }
    if np.count_nonzero(mask) < 3:
        return {
            "role": "secondary_diagnostic_not_used_by_gates_or_primary_plots",
            "available": False,
            "distance_m": distance_m,
            "reason": "fewer than three control samples in distance-fit window",
        }
    alignment = canonical.fit_se2(
        control_estimate.xy[mask], control_reference.xy[mask]
    )
    metrics = {}
    for name, (estimate, reference, _) in associated.items():
        item, _, _, _ = fixed_metrics(canonical, estimate, reference, alignment)
        metrics[name] = item
    return {
        "role": "secondary_diagnostic_not_used_by_gates_or_primary_plots",
        "available": True,
        "distance_m": distance_m,
        "fit_samples": int(np.count_nonzero(mask)),
        "alignment": {
            "yaw_deg": math.degrees(alignment.yaw_rad),
            "translation_x_m": float(alignment.translation[0]),
            "translation_y_m": float(alignment.translation[1]),
            "diagnostic_similarity_scale_not_applied": (
                alignment.diagnostic_similarity_scale
            ),
        },
        "metrics": metrics,
    }


def evaluate_accuracy(
    canonical: Any,
    validator: Any,
    control: Any,
    precision_raw: Any,
    precision_local: Any,
    glim: Any,
    sensor: str,
    maximum_glim_gap_sec: float = 0.15,
    tolerance_ns: int = 1_000,
    secondary_fit_distance_m: float = 20.0,
) -> dict[str, Any]:
    overlap_start_ns = max(
        int(trajectory.stamp_ns[0])
        for trajectory in (control, precision_raw, precision_local)
    )
    overlap_end_ns = min(
        int(trajectory.stamp_ns[-1])
        for trajectory in (control, precision_raw, precision_local)
    )
    if overlap_end_ns <= overlap_start_ns:
        raise RuntimeError("control/precision trajectories have no common time interval")
    native = {}
    for name, trajectory in {
        "control_raw": control, "precision_raw": precision_raw,
        "precision_local": precision_local,
    }.items():
        mask = (
            (trajectory.stamp_ns >= overlap_start_ns)
            & (trajectory.stamp_ns <= overlap_end_ns)
        )
        if np.count_nonzero(mask) < 3:
            raise RuntimeError(f"fewer than three {name} samples in common time interval")
        native[name] = trajectory.subset(mask)
    associated: dict[str, tuple[Any, Any, float]] = {}
    for name, trajectory in native.items():
        associated[name] = associate_glim(
            canonical, trajectory, glim, maximum_glim_gap_sec
        )
    control_estimate, control_reference, _ = associated["control_raw"]
    physical_anchor = first_common_physical_anchor(
        validator,
        control_estimate.stamp_ns,
        associated["precision_raw"][0].stamp_ns,
        associated["precision_local"][0].stamp_ns,
        tolerance_ns,
    )
    anchor_stamp_ns = physical_anchor["control_stamp_ns"]
    anchor_index = physical_anchor["control_index"]
    if (
        anchor_index >= len(control_estimate.stamp_ns)
        or int(control_estimate.stamp_ns[anchor_index]) != anchor_stamp_ns
        or int(control_reference.stamp_ns[anchor_index]) != anchor_stamp_ns
    ):
        raise RuntimeError("primary anchor estimate/reference stamps differ")
    alignment = exact_pose_alignment(
        canonical,
        control_estimate.xy[anchor_index],
        float(control_estimate.yaw[anchor_index]),
        control_reference.xy[anchor_index],
        float(control_reference.yaw[anchor_index]),
    )

    metrics: dict[str, Any] = {}
    series: dict[str, dict[str, Any]] = {}
    for name, (estimate, reference, coverage) in associated.items():
        item, aligned, position, yaw = fixed_metrics(
            canonical, estimate, reference, alignment
        )
        item["glim_association_coverage"] = coverage
        metrics[name] = item
        series[name] = {
            "estimate": estimate, "reference": reference, "aligned": aligned,
            "position_error": position, "yaw_error": yaw,
        }
    anchor_residual = exact_anchor_residual(
        canonical, series["control_raw"]["aligned"], control_reference, anchor_index
    )
    sensitivity = anchor_sensitivity(
        canonical, control_estimate, control_reference, alignment, anchor_index, 2.0
    )
    distance_fit = secondary_distance_fit(
        canonical, associated, secondary_fit_distance_m
    )
    return {
        "sensor": sensor,
        "method": {
            "estimate_association": "native accepted physical stamps; no estimate interpolation",
            "reference_association": "GLIM SE(2) interpolated to native estimate stamps only",
            "primary_alignment": (
                "one exact control/raw first three-stream physical-stamp-common pose "
                "SE(2) (position+yaw), frozen/shared"
            ),
            "scale": "not estimated or applied",
            "comparison_interval": (
                "intersection of control raw, precision raw, and precision-local native "
                "stamp ranges before GLIM association"
            ),
        },
        "common_interval": {
            "start_ns": overlap_start_ns,
            "end_ns": overlap_end_ns,
            "duration_sec": (overlap_end_ns - overlap_start_ns) * 1.0e-9,
        },
        "primary_alignment": {
            "role": "primary_used_by_metrics_gates_csv_and_plots",
            "type": "single_pose_exact_se2",
            "shared_across": ["control_raw", "precision_raw", "precision_local"],
            "anchor_stamp_ns": anchor_stamp_ns,
            "anchor_native_common_all_streams": physical_anchor[
                "all_integer_stamps_equal"
            ],
            "anchor_physical_stamp_common_all_streams": (
                physical_anchor["maximum_absolute_delta_ns"] <= tolerance_ns
            ),
            "anchor_stamp_association": {
                "method": (
                    "integer equality, or nearest unique <= tolerance only for "
                    "epoch/double serialization round-off; no estimate interpolation"
                ),
                "tolerance_ns": tolerance_ns,
                "stream_stamp_ns": physical_anchor["stream_stamp_ns"],
                "stream_delta_from_control_ns": physical_anchor[
                    "stream_delta_from_control_ns"
                ],
                "maximum_absolute_delta_ns": physical_anchor[
                    "maximum_absolute_delta_ns"
                ],
            },
            "estimate_pose": {
                "x_m": float(control_estimate.xy[anchor_index, 0]),
                "y_m": float(control_estimate.xy[anchor_index, 1]),
                "yaw_rad": float(control_estimate.yaw[anchor_index]),
            },
            "reference_pose": {
                "x_m": float(control_reference.xy[anchor_index, 0]),
                "y_m": float(control_reference.xy[anchor_index, 1]),
                "yaw_rad": float(control_reference.yaw[anchor_index]),
            },
            "yaw_deg": math.degrees(alignment.yaw_rad),
            "translation_x_m": float(alignment.translation[0]),
            "translation_y_m": float(alignment.translation[1]),
            "scale_estimated_or_applied": False,
            "exact_anchor_residual": anchor_residual,
        },
        "secondary_diagnostics": {
            "anchor_0_to_2_sec_sensitivity": sensitivity,
            "first_distance_fit": distance_fit,
        },
        "metrics": metrics,
        "comparison": {
            "raw_nonintrusion_glim": compare_metrics(
                metrics["control_raw"], metrics["precision_raw"]
            ),
            "submap_gain": compare_metrics(
                metrics["precision_raw"], metrics["precision_local"]
            ),
            "end_to_end_gain": compare_metrics(
                metrics["control_raw"], metrics["precision_local"]
            ),
        },
        "exact_control_precision_raw": exact_raw_ab(
            canonical, validator, native["control_raw"], native["precision_raw"],
            tolerance_ns
        ),
        "_series": series,
    }


def accuracy_checks(accuracy: dict[str, Any]) -> list[dict[str, Any]]:
    metrics = accuracy["metrics"]
    checks: list[dict[str, Any]] = []
    add = lambda name, passed, detail, category="hard": checks.append(
        {"name": name, "passed": bool(passed), "detail": detail, "category": category}
    )
    anchor = accuracy["primary_alignment"]["exact_anchor_residual"]
    physical_common = accuracy["primary_alignment"][
        "anchor_physical_stamp_common_all_streams"
    ]
    association = accuracy["primary_alignment"]["anchor_stamp_association"]
    add(
        "primary anchor is first three-stream-common physical stamp and maps exactly",
        physical_common
        and anchor["same_stamp"]
        and anchor["position_m"] <= 1.0e-9
        and anchor["yaw_rad"] <= 1.0e-12,
        f"stamp_ns={anchor['stamp_ns']} same_stamp={anchor['same_stamp']} "
        f"physical_common={physical_common} "
        f"max_stream_delta={association['maximum_absolute_delta_ns']}ns "
        f"position={anchor['position_m']:.3e}m yaw={anchor['yaw_rad']:.3e}rad",
    )
    add(
        "one primary SE2 is shared and scale is not estimated",
        accuracy["primary_alignment"]["shared_across"]
        == ["control_raw", "precision_raw", "precision_local"]
        and not accuracy["primary_alignment"]["scale_estimated_or_applied"],
        f"shared={accuracy['primary_alignment']['shared_across']} "
        f"scale={accuracy['primary_alignment']['scale_estimated_or_applied']}",
    )
    for name, item in metrics.items():
        add(
            f"{name} GLIM stamp coverage",
            item["glim_association_coverage"] >= 0.95,
            f"{item['glim_association_coverage']:.6%} >=95%",
        )
    control_count = metrics["control_raw"]["samples"]
    precision_count = metrics["precision_raw"]["samples"]
    count_ratio = min(control_count, precision_count) / max(control_count, precision_count)
    add("accepted-scan count density non-intrusion", count_ratio >= 0.995,
        f"control={control_count} precision={precision_count} ratio={count_ratio:.6%}")
    exact = accuracy["exact_control_precision_raw"]
    add("exact common control/precision raw physical stamps exist",
        exact["samples"] >= 3,
        f"samples={exact['samples']} coverage={exact['coverage']:.6%}")
    # Independent runs can retain a stable whole-frame phase, so exact coverage
    # is diagnostic.  Density and each run's GLIM result are the hard guards.
    add("exact common physical-stamp coverage (diagnostic)", exact["coverage"] >= 0.95,
        f"{exact['coverage']:.6%}", "warn")

    raw_ab = accuracy["comparison"]["raw_nonintrusion_glim"]
    add("precision branch does not regress raw GLIM XY",
        nonregression(raw_ab["position_rmse_m"]["old"],
                      raw_ab["position_rmse_m"]["new"], 0.05),
        json.dumps(raw_ab["position_rmse_m"], sort_keys=True))
    add("precision branch does not regress raw GLIM yaw",
        nonregression(raw_ab["yaw_rmse_deg"]["old"],
                      raw_ab["yaw_rmse_deg"]["new"], 0.05),
        json.dumps(raw_ab["yaw_rmse_deg"], sort_keys=True))

    local_passed, local_failures = absolute_pass(metrics["precision_local"])
    add("precision-local passes absolute GLIM criteria", local_passed,
        "failures=" + (", ".join(local_failures) if local_failures else "none"))
    control_passed, _ = absolute_pass(metrics["control_raw"])
    gain = accuracy["comparison"]["end_to_end_gain"]
    if control_passed:
        gain_passed = nonregression(
            gain["position_rmse_m"]["old"], gain["position_rmse_m"]["new"], 0.05
        )
        detail = "control already passes; require <=5%+0.05m non-regression: " + json.dumps(
            gain["position_rmse_m"], sort_keys=True
        )
    else:
        gain_passed = gain["position_rmse_m"]["improvement_percent"] >= 20.0
        detail = "control fails; require >=20% XY improvement: " + json.dumps(
            gain["position_rmse_m"], sort_keys=True
        )
    add("precision-local has material or non-regressing XY benefit", gain_passed, detail)
    add("precision-local yaw is non-regressing",
        nonregression(gain["yaw_rmse_deg"]["old"], gain["yaw_rmse_deg"]["new"], 0.05),
        json.dumps(gain["yaw_rmse_deg"], sort_keys=True))
    for distance, values in gain["rpe"].items():
        add(f"precision-local {distance}m translation RPE non-regression",
            nonregression(values["translation_rmse_m"]["old"],
                          values["translation_rmse_m"]["new"], 0.02),
            json.dumps(values["translation_rmse_m"], sort_keys=True))
        add(f"precision-local {distance}m yaw RPE non-regression",
            nonregression(values["yaw_rmse_deg"]["old"],
                          values["yaw_rmse_deg"]["new"], 0.05),
            json.dumps(values["yaw_rmse_deg"], sort_keys=True))
    return checks


def write_plot_csv(
    path: Path, series: dict[str, dict[str, Any]], primary_anchor_ns: int
) -> None:
    lines = [
        "stream,stamp_ns,time_from_primary_anchor_sec,reference_x,reference_y,"
        "reference_yaw_rad,aligned_x,aligned_y,aligned_yaw_rad,position_error_m,"
        "yaw_error_deg"
    ]
    for name, item in series.items():
        estimate, reference, aligned = item["estimate"], item["reference"], item["aligned"]
        for index in range(len(estimate.stamp_ns)):
            values = (
                name, int(estimate.stamp_ns[index]),
                (int(estimate.stamp_ns[index]) - primary_anchor_ns) * 1.0e-9,
                reference.xy[index, 0], reference.xy[index, 1], reference.yaw[index],
                aligned.xy[index, 0], aligned.xy[index, 1], aligned.yaw[index],
                item["position_error"][index], item["yaw_error"][index],
            )
            lines.append(",".join([str(values[0]), str(values[1])] +
                                  [f"{float(value):.17g}" for value in values[2:]]))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_plots(
    output: Path,
    series: dict[str, dict[str, Any]],
    label: str,
    primary_anchor_ns: int,
) -> dict[str, str]:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/lidar_imu_submap_ab_matplotlib")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"control_raw": "#e68613", "precision_raw": "#888888",
              "precision_local": "#2764c4"}
    labels = {"control_raw": "control scan-to-scan", "precision_raw": "precision-run raw",
              "precision_local": "external scan-to-submap local"}
    reference = series["control_raw"]["reference"]
    fig, axis = plt.subplots(figsize=(8.5, 7.0))
    axis.plot(reference.xy[:, 0], reference.xy[:, 1], color="black", linewidth=2,
              label="GLIM")
    for name, item in series.items():
        aligned = item["aligned"]
        axis.plot(aligned.xy[:, 0], aligned.xy[:, 1], color=colors[name], linewidth=1.3,
                  label=labels[name])
    axis.set_aspect("equal", adjustable="datalim")
    axis.set_xlabel("shared GLIM x [m]")
    axis.set_ylabel("shared GLIM y [m]")
    axis.set_title(f"{label}: shared exact first-common-pose SE(2)")
    axis.grid(True, alpha=0.25); axis.legend()
    fig.tight_layout()
    trajectory = output / "trajectory_overlay.png"
    fig.savefig(trajectory, dpi=150); plt.close(fig)

    artifacts = {"trajectory": str(trajectory.resolve())}
    for field, ylabel, filename in (
        ("position_error", "XY error [m]", "position_error.png"),
        ("yaw_error", "yaw error [deg]", "yaw_error.png"),
    ):
        fig, axis = plt.subplots(figsize=(10.5, 5.0))
        for name, item in series.items():
            estimate = item["estimate"]
            time = (estimate.stamp_ns - primary_anchor_ns) * 1.0e-9
            axis.plot(time, item[field], color=colors[name], linewidth=1.1,
                      label=labels[name])
        axis.set_xlabel("time from shared primary anchor [s]"); axis.set_ylabel(ylabel)
        axis.set_title(f"{label}: {ylabel}"); axis.grid(True, alpha=0.25); axis.legend()
        fig.tight_layout()
        path = output / filename
        fig.savefig(path, dpi=150); plt.close(fig)
        artifacts[field] = str(path.resolve())
    csv = output / "aligned_samples.csv"
    write_plot_csv(csv, series, primary_anchor_ns)
    artifacts["csv"] = str(csv.resolve())
    return artifacts


def write_markdown(path: Path, result: dict[str, Any]) -> None:
    metrics = result["accuracy"]["metrics"]
    comparison = result["accuracy"]["comparison"]["end_to_end_gain"]
    primary = result["accuracy"]["primary_alignment"]
    association = primary["anchor_stamp_association"]
    residual = primary["exact_anchor_residual"]
    sensitivity = result["accuracy"]["secondary_diagnostics"][
        "anchor_0_to_2_sec_sensitivity"
    ]
    distance_fit = result["accuracy"]["secondary_diagnostics"]["first_distance_fit"]
    lines = [
        f"# {result['label']} LiDAR/IMU scan-to-submap A/B", "",
        f"Overall: **{'PASS' if result['passed'] else 'FAIL'}**", "",
        "Primary alignment: the first control raw pose with a native physical-stamp "
        "counterpart in all three streams is mapped exactly to GLIM at the control "
        "stamp. Integer equality is used normally; nearest unique <="
        f"{association['tolerance_ns'] / 1000.0:g} us is allowed only for epoch/double "
        "serialization round-off. The resulting one SE(2) is "
        "frozen and shared by all three streams; scale is not estimated.", "",
        f"- anchor stamp: `{primary['anchor_stamp_ns']}`",
        f"- shared transform: x={primary['translation_x_m']:.9f} m, "
        f"y={primary['translation_y_m']:.9f} m, yaw={primary['yaw_deg']:.9f} deg",
        f"- exact residual: {residual['position_m']:.3e} m / "
        f"{residual['yaw_rad']:.3e} rad", "",
        "| Stream | XY RMSE | yaw RMSE | yaw p95 | endpoint XY | path ratio |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name in ("control_raw", "precision_raw", "precision_local"):
        item = metrics[name]
        lines.append(
            f"| {name} | {item['position_error_m']['rmse']:.4f} m | "
            f"{item['yaw_error_deg']['rmse']:.3f} deg | "
            f"{item['yaw_error_deg']['p95']:.3f} deg | "
            f"{item['endpoint_position_error_m']:.4f} m | "
            f"{item['path_ratio_estimate_over_reference']:.5f} |"
        )
    lines.extend([
        "", f"End-to-end XY improvement: "
        f"{comparison['position_rmse_m']['improvement_percent']:+.3f}%", "",
        "![trajectory](trajectory_overlay.png)", "",
        "![position error](position_error.png)", "",
        "![yaw error](yaw_error.png)", "",
        "## Secondary alignment diagnostics", "",
        "These diagnostics do not affect primary metrics, plots, or acceptance gates.", "",
        f"- exact-anchor sensitivity over 0–2 s: {sensitivity['candidate_count']} anchors; "
        f"transform yaw delta max "
        f"{sensitivity['transform_yaw_delta_deg']['maximum']:.6f} deg; translation "
        f"delta max {sensitivity['transform_translation_delta_m']['maximum']:.6f} m",
    ])
    if distance_fit["available"]:
        secondary_local = distance_fit["metrics"]["precision_local"]
        lines.extend([
            f"- former first-{distance_fit['distance_m']:g}m fit: available, "
            f"fit samples={distance_fit['fit_samples']}; precision-local XY RMSE="
            f"{secondary_local['position_error_m']['rmse']:.4f} m, yaw RMSE="
            f"{secondary_local['yaw_error_deg']['rmse']:.3f} deg",
        ])
    else:
        lines.append(
            f"- former first-{distance_fit['distance_m']:g}m fit: unavailable — "
            f"{distance_fit['reason']}"
        )
    lines.extend(["", "## Acceptance", ""])
    for item in result["checks"]:
        mark = "PASS" if item["passed"] else ("WARN" if item["category"] == "warn" else "FAIL")
        lines.append(f"- {mark}: {item['name']} — {item['detail']}")
    lines.extend([
        "", "GLIM and evaluated odometry share LiDAR/IMU observations. This is a "
        "correlated pseudo-ground-truth comparison, not independent absolute ground truth.", "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def strip_internal(accuracy: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in accuracy.items() if key != "_series"}


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    repo = args.repo.resolve()
    canonical = load_module("lidar_imu_submap_canonical", repo / "tools/evaluate_glim_trajectory.py")
    validator = load_module("lidar_imu_submap_validator", repo / "tools/validate_lidar_imu_submap_run.py")
    control_run, precision_run = args.control_run.resolve(), args.precision_run.resolve()
    control_runtime = read_json(control_run / "runtime_analysis/runtime_metrics.json")
    precision_runtime = read_json(precision_run / "runtime_analysis/runtime_metrics.json")
    control_runtime_pass, control_runtime_detail = validator.runtime_summary(control_runtime)
    precision_records = validator.read_records(precision_run)
    structural = validator.evaluate_records(
        precision_records, precision_runtime,
        int(round(args.stamp_tolerance_us * 1_000.0)), args.maximum_matcher_p99_ms,
    )
    control = read_control_pose(control_run, validator.RAW, canonical)
    precision_raw = records_trajectory(precision_records.raw, canonical)
    precision_local = records_trajectory(precision_records.local, canonical)
    glim = canonical.read_glim_trajectory(
        args.glim_dir.resolve() / args.glim_trajectory
    )
    accuracy = evaluate_accuracy(
        canonical, validator, control, precision_raw, precision_local, glim,
        args.sensor, args.maximum_glim_gap_sec,
        int(round(args.stamp_tolerance_us * 1_000.0)), args.secondary_fit_distance_m,
    )
    config = config_contract(
        read_env(control_run / "run.env"), read_env(precision_run / "run.env"), args.sensor
    )
    checks = [
        {"name": "control runtime audit passes", "passed": control_runtime_pass,
         "detail": control_runtime_detail, "category": "hard"},
        {"name": "precision structural/runtime audit passes", "passed": structural["passed"],
         "detail": "failed=" + (", ".join(structural["failed_hard_checks"])
                                    if structural["failed_hard_checks"] else "none"),
         "category": "hard"},
        {"name": "control and precision sensor/configuration are identical",
         "passed": config["passed"], "detail": json.dumps(config, sort_keys=True),
         "category": "hard"},
    ]
    checks.extend(accuracy_checks(accuracy))
    passed = all(item["passed"] for item in checks if item["category"] == "hard")
    return {
        "passed": passed, "label": args.label, "sensor": args.sensor,
        "inputs": {
            "control_run": str(control_run), "precision_run": str(precision_run),
            "glim": str((args.glim_dir / args.glim_trajectory).resolve()),
        },
        "configuration_contract": config,
        "structural_validation": structural,
        "accuracy": accuracy,
        "checks": checks,
    }


def self_test(repo: Path) -> None:
    # Canonical trajectory math is deliberately reusable without a ROS graph.
    # Its bag-reader imports are stubbed only for this synthetic, read-free path
    # when the test is run from an unsourced shell.
    try:
        import rosbag2_py  # type: ignore  # noqa: F401
    except ImportError:
        sys.modules["rosbag2_py"] = types.ModuleType("rosbag2_py")
        rclpy = types.ModuleType("rclpy")
        serialization = types.ModuleType("rclpy.serialization")
        serialization.deserialize_message = lambda *_: None
        rclpy.serialization = serialization
        sys.modules["rclpy"] = rclpy
        sys.modules["rclpy.serialization"] = serialization
        rosidl = types.ModuleType("rosidl_runtime_py")
        utilities = types.ModuleType("rosidl_runtime_py.utilities")
        utilities.get_message = lambda *_: None
        rosidl.utilities = utilities
        sys.modules["rosidl_runtime_py"] = rosidl
        sys.modules["rosidl_runtime_py.utilities"] = utilities
    canonical = load_module("lidar_imu_submap_test_canonical", repo / "tools/evaluate_glim_trajectory.py")
    validator = load_module("lidar_imu_submap_test_validator", repo / "tools/validate_lidar_imu_submap_run.py")
    validator.self_test()
    stamps = np.arange(0, 60_100_000_000, 100_000_000, dtype=np.int64)
    t = stamps * 1.0e-9
    reference_xy = np.column_stack((t, 2.0 * np.sin(t / 8.0)))
    reference_yaw = np.arctan2(np.gradient(reference_xy[:, 1]),
                               np.gradient(reference_xy[:, 0]))
    glim = canonical.Trajectory(stamps, reference_xy, reference_yaw)
    rotation = np.asarray([[math.cos(0.2), math.sin(0.2)],
                           [-math.sin(0.2), math.cos(0.2)]])
    raw_xy = (rotation @ (reference_xy - np.asarray([4.0, -2.0])).T).T
    drift_time = np.maximum(t - 20.0, 0.0)
    drift = np.column_stack((0.10 * drift_time, 0.05 * drift_time))
    control = canonical.Trajectory(
        stamps, raw_xy + drift, reference_yaw - 0.2 + 0.010 * drift_time
    )
    precision_raw = canonical.Trajectory(
        stamps[2:-8], control.xy[2:-8].copy(), control.yaw[2:-8].copy()
    )
    local = canonical.Trajectory(
        stamps[3:-10], (raw_xy + 0.10 * drift)[3:-10],
        (reference_yaw - 0.2 + 0.001 * drift_time)[3:-10],
    )
    result = evaluate_accuracy(canonical, validator, control, precision_raw, local, glim,
                               "velodyne")
    checks = accuracy_checks(result)
    primary = result["primary_alignment"]
    anchor = primary["exact_anchor_residual"]
    assert anchor["same_stamp"]
    assert primary["anchor_native_common_all_streams"]
    assert primary["anchor_physical_stamp_common_all_streams"]
    assert anchor["position_m"] <= 1.0e-12
    assert anchor["yaw_rad"] <= 1.0e-12
    assert abs(primary["yaw_deg"] - math.degrees(0.2)) <= 1.0e-9
    assert np.linalg.norm(
        np.asarray([primary["translation_x_m"], primary["translation_y_m"]])
        - np.asarray([4.0, -2.0])
    ) <= 1.0e-9
    tolerant_anchor = first_common_physical_anchor(
        validator,
        np.asarray([100_000, 200_000, 300_000], dtype=np.int64),
        np.asarray([100_250, 200_250, 300_250], dtype=np.int64),
        np.asarray([99_500, 199_500, 299_500], dtype=np.int64),
        1_000,
    )
    assert tolerant_anchor["control_index"] == 0
    assert tolerant_anchor["maximum_absolute_delta_ns"] == 500
    assert not tolerant_anchor["all_integer_stamps_equal"]
    sensitivity = result["secondary_diagnostics"]["anchor_0_to_2_sec_sensitivity"]
    assert sensitivity["candidate_count"] >= 20
    assert sensitivity["anchor_exact_position_residual_m_maximum"] <= 1.0e-12
    assert sensitivity["anchor_exact_yaw_residual_rad_maximum"] <= 1.0e-12
    assert result["secondary_diagnostics"]["first_distance_fit"]["available"]
    assert result["comparison"]["end_to_end_gain"]["position_rmse_m"][
        "improvement_percent"
    ] > 20.0
    assert result["common_interval"]["start_ns"] == int(stamps[3])
    assert result["common_interval"]["end_ns"] == int(stamps[-11])
    vel_control = {
        "sensor": "velodyne", "bag": "/same", "glim_traj_sha256": "a",
        "deskew": "41ms", "use_deskew": "true", "rate": "1.0",
        "points_topic": "/points", "imu_topic": "/imu",
        "imu_param_sha256": "b", "odom_override_sha256": "c",
        "base_odom_param_sha256": "d",
    }
    vel_precision = dict(vel_control, mid_yaw_policy="n/a", gicp_epsilon="n/a")
    assert config_contract(vel_control, vel_precision, "velodyne")["passed"]
    mid_control = dict(vel_control, sensor="mid360", mid_yaw_policy="fixed-bias-direct",
                       gicp_epsilon="default")
    mid_precision = dict(mid_control)
    assert config_contract(mid_control, mid_precision, "mid360")["passed"]
    mid_precision["mid_yaw_policy"] = "adaptive"
    assert not config_contract(mid_control, mid_precision, "mid360")["passed"]
    assert all(item["passed"] for item in checks if item["category"] == "hard"), checks

    # A local-only initial offset must remain visible.  This proves that the
    # precision-local stream is not independently snapped to GLIM.
    offset_local = canonical.Trajectory(
        local.stamp_ns,
        local.xy + np.asarray([0.50, -0.25]),
        canonical.wrap_angle(local.yaw + 0.10),
    )
    offset_result = evaluate_accuracy(
        canonical, validator, control, precision_raw, offset_local, glim, "velodyne"
    )
    assert offset_result["primary_alignment"] == result["primary_alignment"]
    offset_series = offset_result["_series"]["precision_local"]
    assert float(offset_series["position_error"][0]) > 0.50
    assert float(offset_series["yaw_error"][0]) > 5.0
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)
        artifacts = write_plots(
            path, result["_series"], "synthetic",
            result["primary_alignment"]["anchor_stamp_ns"],
        )
        assert all(Path(value).is_file() for value in artifacts.values())
        assert "time_from_primary_anchor_sec" in (path / "aligned_samples.csv").read_text(
            encoding="utf-8"
        ).splitlines()[0]


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    result.add_argument("--control-run", type=Path)
    result.add_argument("--precision-run", type=Path)
    result.add_argument("--glim-dir", type=Path)
    result.add_argument("--glim-trajectory", default="traj_lidar.txt")
    result.add_argument("--sensor", choices=("velodyne", "mid360"))
    result.add_argument("--output-dir", type=Path)
    result.add_argument("--label", default="LiDAR/IMU external submap")
    result.add_argument("--maximum-glim-gap-sec", type=float, default=0.15)
    result.add_argument("--stamp-tolerance-us", type=float, default=1.0)
    result.add_argument(
        "--secondary-fit-distance-m",
        dest="secondary_fit_distance_m", type=float, default=20.0,
        help=(
            "non-gating legacy distance-fit diagnostic only; the primary alignment "
            "always uses the exact initial control pose"
        ),
    )
    result.add_argument("--maximum-matcher-p99-ms", type=float, default=250.0)
    result.add_argument("--self-test", action="store_true")
    return result


def main() -> int:
    argument_parser = parser()
    if any(
        item == "--initial-alignment-distance-m"
        or item.startswith("--initial-alignment-distance-m=")
        for item in sys.argv[1:]
    ):
        argument_parser.error(
            "--initial-alignment-distance-m was removed because primary alignment "
            "is now the exact first common pose; use --secondary-fit-distance-m "
            "only for the non-gating legacy diagnostic"
        )
    args = argument_parser.parse_args()
    if args.self_test:
        self_test(args.repo.resolve())
        print("PASS: synthetic LiDAR/IMU external-submap A/B")
        return 0
    for name in ("control_run", "precision_run", "glim_dir", "sensor", "output_dir"):
        if getattr(args, name) is None:
            raise SystemExit(f"--{name.replace('_', '-')} is required")
    if args.output_dir.exists():
        raise SystemExit(f"refusing to overwrite: {args.output_dir}")
    result = evaluate(args)
    args.output_dir.mkdir(parents=True)
    artifacts = write_plots(
        args.output_dir, result["accuracy"]["_series"], args.label,
        result["accuracy"]["primary_alignment"]["anchor_stamp_ns"],
    )
    result["artifacts"] = artifacts
    result["accuracy"] = strip_internal(result["accuracy"])
    (args.output_dir / "evaluation.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    write_markdown(args.output_dir / "REPORT.md", result)
    print(f"{'PASS' if result['passed'] else 'FAIL'}: {args.output_dir / 'REPORT.md'}")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
