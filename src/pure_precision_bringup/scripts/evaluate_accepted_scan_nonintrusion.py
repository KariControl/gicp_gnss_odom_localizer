#!/usr/bin/env python3
"""Compare accepted-scan instrumentation with exact and bounded phase checks."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


TOPIC = "/localization/submap_scan"
DIAGNOSTICS = "/diagnostics"
CLOCK = "/clock"
ODOMETER_DIAGNOSTIC = "localization/gyro_odometer"


@dataclass(frozen=True)
class Scan:
    stamp_ns: int
    session: int
    generation: int
    sequence: int
    x: float
    y: float
    yaw: float
    record_lag_sec: float = math.nan


@dataclass(frozen=True)
class Run:
    scans: list[Scan]
    final_accepted_sequence: int
    diagnostic_snapshot_count: int


def wrap(value: np.ndarray | float) -> np.ndarray | float:
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def yaw_of(q: Any) -> float:
    return math.atan2(
        2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
        1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2),
    )


def stats(values: np.ndarray) -> dict[str, Any]:
    if len(values) == 0:
        return {
            "count": 0,
            "mean": math.nan,
            "rmse": math.nan,
            "p95": math.nan,
            "p99": math.nan,
            "maximum": math.nan,
        }
    return {
        "count": int(len(values)),
        "mean": float(np.mean(values)),
        "rmse": math.sqrt(float(np.mean(np.square(values)))),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "maximum": float(np.max(values)),
    }


def resolve(path: Path) -> Path:
    path = path.resolve()
    if path.is_dir() and (path / "metadata.yaml").is_file():
        return path
    nested = path / "localization_output"
    if nested.is_dir() and (nested / "metadata.yaml").is_file():
        return nested
    raise RuntimeError(f"cannot resolve bag: {path}")


def read(path: Path) -> Run:
    import rosbag2_py  # type: ignore
    from rclpy.serialization import deserialize_message  # type: ignore
    from rosidl_runtime_py.utilities import get_message  # type: ignore

    bag = resolve(path)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if TOPIC not in types or DIAGNOSTICS not in types:
        raise RuntimeError(
            f"{bag} lacks {TOPIC} or {DIAGNOSTICS}; use evaluator-only "
            "accepted-scan instrumentation for both control and precision runs"
        )
    classes = {
        TOPIC: get_message(types[TOPIC]),
        DIAGNOSTICS: get_message(types[DIAGNOSTICS]),
    }
    if CLOCK in types:
        classes[CLOCK] = get_message(types[CLOCK])
    scans: list[Scan] = []
    final_accepted_sequence = -1
    diagnostic_snapshot_count = -1
    latest_clock_ns = 0
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in classes:
            continue
        msg = deserialize_message(serialized, classes[topic])
        if topic == CLOCK:
            latest_clock_ns = (
                int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
            )
            continue
        if topic == DIAGNOSTICS:
            for status in msg.status:
                if not (
                    status.name == ODOMETER_DIAGNOSTIC
                    or status.name.endswith("/" + ODOMETER_DIAGNOSTIC)
                ):
                    continue
                values = {item.key: item.value for item in status.values}
                try:
                    sequence = int(values["external_submap_snapshot_sequence"])
                    count = int(values["external_submap_snapshot_published_count"])
                except (KeyError, ValueError):
                    continue
                # The final recorder flush can contain status arrays in a
                # different interleaving order. Sequence is the monotonic
                # accepted-input counter, so retain its greatest observation.
                if sequence >= final_accepted_sequence:
                    final_accepted_sequence = sequence
                    diagnostic_snapshot_count = count
            continue
        stamp = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        pose = msg.raw_pose.pose
        scans.append(
            Scan(
                stamp,
                int(msg.odom_session_id),
                int(msg.odom_generation),
                int(msg.sequence),
                float(pose.position.x),
                float(pose.position.y),
                yaw_of(pose.orientation),
                (
                    max(0, latest_clock_ns - stamp) * 1.0e-9
                    if latest_clock_ns > 0 else math.nan
                ),
            )
        )
    if final_accepted_sequence < 1 or diagnostic_snapshot_count < 1:
        raise RuntimeError(f"{bag} lacks final accepted-snapshot diagnostics")
    return Run(scans, final_accepted_sequence, diagnostic_snapshot_count)


def stream_key(scan: Scan) -> tuple[int, int]:
    """Comparable key across independent processes; session IDs are opaque."""
    return scan.generation, scan.sequence


def snapshot_policy_summary(run: Run, interval: int) -> dict[str, Any]:
    if interval < 1:
        raise ValueError("snapshot interval must be positive")
    scans = run.scans
    stream_ids = {(item.session, item.generation) for item in scans}
    if not scans or len(stream_ids) != 1:
        return {
            "valid": False,
            "reason": f"expected one stream generation, got {sorted(stream_ids)}",
            "observed": len(scans),
            "expected": None,
        }
    first = scans[0].sequence
    final = run.final_accepted_sequence
    expected_sequences = [first]
    expected_sequences.extend(
        range(((first // interval) + 1) * interval, final + 1, interval)
    )
    observed_sequences = [item.sequence for item in scans]
    expected = len(expected_sequences)
    valid = (
        first >= 1
        and final >= observed_sequences[-1]
        and final - observed_sequences[-1] < interval
        and observed_sequences == expected_sequences
        and len(scans) == expected
        and run.diagnostic_snapshot_count == len(scans)
    )
    return {
        "valid": valid,
        "first_sequence": first,
        "final_accepted_sequence": final,
        "last_snapshot_sequence": observed_sequences[-1],
        "observed": len(scans),
        "expected": expected,
        "diagnostic_snapshot_count": run.diagnostic_snapshot_count,
        "interval": interval,
    }


def infer_raw_frame_period(control: list[Scan], precision: list[Scan]) -> float:
    estimates: list[float] = []
    for scans in (control, precision):
        for first, second in zip(scans, scans[1:]):
            sequence_delta = second.sequence - first.sequence
            stamp_delta = second.stamp_ns - first.stamp_ns
            if (
                first.session == second.session
                and first.generation == second.generation
                and sequence_delta > 0
                and stamp_delta > 0
            ):
                estimates.append(stamp_delta * 1.0e-9 / sequence_delta)
    if not estimates:
        return math.nan
    return float(np.median(np.asarray(estimates)))


def phase_stability_summary(
    signed_stamp_offsets_sec: np.ndarray, raw_frame_period_sec: float
) -> dict[str, Any]:
    """Separate a fixed replay phase from drift or processing-rate loss."""
    offsets = np.asarray(signed_stamp_offsets_sec, dtype=float)
    if (
        len(offsets) == 0
        or not math.isfinite(raw_frame_period_sec)
        or raw_frame_period_sec <= 0.0
        or not bool(np.all(np.isfinite(offsets)))
    ):
        return {
            "valid": False,
            "count": int(len(offsets)),
            "absolute_max_frames": math.inf,
            "period_residual_max_sec": math.inf,
            "robust_low_frames": math.nan,
            "robust_high_frames": math.nan,
            "robust_span_frames": math.inf,
            "early_median_frames": math.nan,
            "late_median_frames": math.nan,
            "early_late_drift_frames": math.inf,
        }

    signed_frames = np.rint(offsets / raw_frame_period_sec)
    residual = np.abs(offsets - signed_frames * raw_frame_period_sec)
    robust_low, robust_high = np.quantile(
        signed_frames, [0.01, 0.99], method="nearest"
    )
    window = min(
        len(signed_frames), max(3, int(math.ceil(0.05 * len(signed_frames))))
    )
    early_median = float(np.median(signed_frames[:window]))
    late_median = float(np.median(signed_frames[-window:]))
    return {
        "valid": True,
        "count": int(len(offsets)),
        "absolute_max_frames": float(np.max(np.abs(signed_frames))),
        "period_residual_max_sec": float(np.max(residual)),
        "robust_low_frames": float(robust_low),
        "robust_high_frames": float(robust_high),
        "robust_span_frames": float(robust_high - robust_low),
        "early_median_frames": early_median,
        "late_median_frames": late_median,
        "early_late_drift_frames": abs(late_median - early_median),
    }


def interpolate_scans(
    source: list[Scan], targets: list[Scan], maximum_gap_sec: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Interpolate source SE(2) at target physical stamps without extrapolation."""
    source = sorted(source, key=lambda item: item.stamp_ns)
    source_stamps = np.asarray([item.stamp_ns for item in source], dtype=np.int64)
    source_xy = np.asarray([[item.x, item.y] for item in source])
    source_yaw = np.asarray([item.yaw for item in source])
    xy_values: list[np.ndarray] = []
    yaw_values: list[float] = []
    valid: list[bool] = []
    maximum_gap_ns = int(round(maximum_gap_sec * 1.0e9))
    for target in targets:
        insertion = int(np.searchsorted(source_stamps, target.stamp_ns, side="left"))
        if insertion < len(source) and int(source_stamps[insertion]) == target.stamp_ns:
            xy_values.append(source_xy[insertion])
            yaw_values.append(float(source_yaw[insertion]))
            valid.append(True)
            continue
        if insertion <= 0 or insertion >= len(source):
            xy_values.append(np.asarray([math.nan, math.nan]))
            yaw_values.append(math.nan)
            valid.append(False)
            continue
        left = insertion - 1
        right = insertion
        gap = int(source_stamps[right]) - int(source_stamps[left])
        if gap <= 0 or gap > maximum_gap_ns:
            xy_values.append(np.asarray([math.nan, math.nan]))
            yaw_values.append(math.nan)
            valid.append(False)
            continue
        fraction = (target.stamp_ns - int(source_stamps[left])) / gap
        xy_values.append(
            source_xy[left] + fraction * (source_xy[right] - source_xy[left])
        )
        yaw_delta = float(wrap(float(source_yaw[right]) - float(source_yaw[left])))
        yaw_values.append(float(wrap(float(source_yaw[left]) + fraction * yaw_delta)))
        valid.append(True)
    return (
        np.asarray(xy_values),
        np.asarray(yaw_values),
        np.asarray(valid, dtype=bool),
    )


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    control_run = read(args.control_bag)
    precision_run = read(args.precision_bag)
    control = control_run.scans
    precision = precision_run.scans
    control_record_lag_sec = np.asarray(
        [item.record_lag_sec for item in control if math.isfinite(item.record_lag_sec)]
    )
    precision_record_lag_sec = np.asarray(
        [
            item.record_lag_sec
            for item in precision
            if math.isfinite(item.record_lag_sec)
        ]
    )
    control_by_stamp = {item.stamp_ns: item for item in control}
    precision_by_stamp = {item.stamp_ns: item for item in precision}
    common_stamps = sorted(set(control_by_stamp) & set(precision_by_stamp))
    if len(common_stamps) < 3:
        raise RuntimeError("fewer than three common accepted-scan physical stamps")
    c = [control_by_stamp[stamp] for stamp in common_stamps]
    p = [precision_by_stamp[stamp] for stamp in common_stamps]
    c_xy = np.asarray([[item.x, item.y] for item in c])
    p_xy = np.asarray([[item.x, item.y] for item in p])
    c_yaw = np.asarray([item.yaw for item in c])
    p_yaw = np.asarray([item.yaw for item in p])
    xy = np.linalg.norm(p_xy - c_xy, axis=1)
    yaw = np.degrees(np.abs(wrap(p_yaw - c_yaw)))
    translation_increment = np.linalg.norm(
        np.diff(p_xy, axis=0) - np.diff(c_xy, axis=0), axis=1
    )
    yaw_increment = np.degrees(np.abs(wrap(np.diff(p_yaw) - np.diff(c_yaw))))
    denominator = max(len(control), len(precision))
    coverage = len(common_stamps) / denominator if denominator else 0.0
    sequence_delta = np.asarray(
        [p_item.sequence - c_item.sequence for c_item, p_item in zip(c, p)],
        dtype=np.int64,
    )

    control_by_key = {stream_key(item): item for item in control}
    precision_by_key = {stream_key(item): item for item in precision}
    common_keys = sorted(set(control_by_key) & set(precision_by_key))
    key_denominator = max(len(control_by_key), len(precision_by_key))
    key_coverage = len(common_keys) / key_denominator if key_denominator else 0.0
    key_control = [control_by_key[key] for key in common_keys]
    key_precision = [precision_by_key[key] for key in common_keys]
    raw_frame_period_sec = infer_raw_frame_period(control, precision)
    signed_key_stamp_offsets_sec = np.asarray(
        [
            (p.stamp_ns - c.stamp_ns) * 1.0e-9
            for c, p in zip(key_control, key_precision)
        ]
    )
    key_stamp_offsets_sec = np.abs(signed_key_stamp_offsets_sec)
    phase_summary = phase_stability_summary(
        signed_key_stamp_offsets_sec, raw_frame_period_sec
    )
    nearest_frame_counts = np.rint(
        signed_key_stamp_offsets_sec / raw_frame_period_sec
    )
    phase_residual_sec = np.abs(
        signed_key_stamp_offsets_sec
        - nearest_frame_counts * raw_frame_period_sec
    )

    control_policy = snapshot_policy_summary(control_run, args.snapshot_interval)
    precision_policy = snapshot_policy_summary(precision_run, args.snapshot_interval)
    accepted_sequence_delta = abs(
        precision_run.final_accepted_sequence - control_run.final_accepted_sequence
    )
    accepted_sequence_denominator = max(
        precision_run.final_accepted_sequence, control_run.final_accepted_sequence
    )
    accepted_sequence_delta_ratio = (
        accepted_sequence_delta / accepted_sequence_denominator
        if accepted_sequence_denominator else math.inf
    )

    # During a bounded callback-phase interval, compare both runs at the same
    # physical time. This supplements (and never replaces) exact-stamp metrics.
    interpolated_control_xy, interpolated_control_yaw, interpolation_valid = (
        interpolate_scans(control, key_precision, args.maximum_interpolation_gap)
    )
    target_precision_xy = np.asarray([[item.x, item.y] for item in key_precision])
    target_precision_yaw = np.asarray([item.yaw for item in key_precision])
    interpolated_xy_error = np.linalg.norm(
        target_precision_xy[interpolation_valid]
        - interpolated_control_xy[interpolation_valid],
        axis=1,
    )
    interpolated_yaw_error = np.degrees(
        np.abs(
            wrap(
                target_precision_yaw[interpolation_valid]
                - interpolated_control_yaw[interpolation_valid]
            )
        )
    )
    interpolation_coverage = (
        int(np.count_nonzero(interpolation_valid)) / len(common_keys)
        if common_keys else 0.0
    )
    comparable_pairs = np.asarray(
        [
            first.generation == second.generation
            and second.sequence > first.sequence
            for first, second in zip(key_precision, key_precision[1:])
        ],
        dtype=bool,
    )
    interpolation_pair_valid = (
        interpolation_valid[:-1] & interpolation_valid[1:] & comparable_pairs
    )
    interpolation_pair_indices = np.flatnonzero(interpolation_pair_valid)
    interpolated_translation_increment_error = np.asarray(
        [
            np.linalg.norm(
                (target_precision_xy[index + 1] - target_precision_xy[index])
                - (
                    interpolated_control_xy[index + 1]
                    - interpolated_control_xy[index]
                )
            )
            for index in interpolation_pair_indices
        ]
    )
    interpolated_yaw_increment_error = np.degrees(
        np.abs(
            np.asarray(
                [
                    wrap(
                        (
                            target_precision_yaw[index + 1]
                            - target_precision_yaw[index]
                        )
                        - (
                            interpolated_control_yaw[index + 1]
                            - interpolated_control_yaw[index]
                        )
                    )
                    for index in interpolation_pair_indices
                ]
            )
        )
    )
    interpolation_pair_denominator = int(np.count_nonzero(comparable_pairs))
    interpolation_pair_coverage = (
        len(interpolation_pair_indices) / interpolation_pair_denominator
        if interpolation_pair_denominator else 0.0
    )

    checks = []

    def add(name: str, passed: bool, detail: str, category: str = "hard") -> None:
        checks.append({
            "name": name,
            "passed": bool(passed),
            "detail": detail,
            "category": category,
        })

    add(
        "control accepted-scan stamps are unique and strictly increasing",
        len(control_by_stamp) == len(control)
        and all(b.stamp_ns > a.stamp_ns for a, b in zip(control, control[1:])),
        f"messages={len(control)} unique={len(control_by_stamp)}",
    )
    add(
        "precision accepted-scan stamps are unique and strictly increasing",
        len(precision_by_stamp) == len(precision)
        and all(b.stamp_ns > a.stamp_ns for a, b in zip(precision, precision[1:])),
        f"messages={len(precision)} unique={len(precision_by_stamp)}",
    )
    add(
        "accepted-scan recorder backlog stays below 0.25s (diagnostic)",
        len(control_record_lag_sec) == len(control)
        and len(precision_record_lag_sec) == len(precision)
        and stats(control_record_lag_sec)["maximum"] <= 0.25
        and stats(precision_record_lag_sec)["maximum"] <= 0.25,
        f"control_max={stats(control_record_lag_sec)['maximum']:.9f}s "
        f"precision_max={stats(precision_record_lag_sec)['maximum']:.9f}s",
        "warn",
    )
    add(
        "exact common accepted-scan coverage (reference only)",
        coverage >= 0.99,
        f"common={len(common_stamps)} control={len(control)} "
        f"precision={len(precision)} coverage={coverage:.9f}",
        "warn",
    )
    add(
        "accepted sequence offset is constant on common physical scans",
        len(set(map(int, sequence_delta))) == 1,
        f"offsets={sorted(set(map(int, sequence_delta)))}",
    )
    add(
        "control snapshot count follows interval policy",
        control_policy["valid"],
        json.dumps(control_policy, sort_keys=True),
    )
    add(
        "precision snapshot count follows interval policy",
        precision_policy["valid"],
        json.dumps(precision_policy, sort_keys=True),
    )
    add(
        "generation-sequence key coverage is at least 99.8 percent",
        len(control_by_key) == len(control)
        and len(precision_by_key) == len(precision)
        and key_coverage >= 0.998,
        f"common={len(common_keys)} control={len(control_by_key)} "
        f"precision={len(precision_by_key)} coverage={key_coverage:.9f}",
    )
    add(
        "final accepted-input sequence differs by at most 0.1 percent",
        accepted_sequence_delta_ratio <= 0.001,
        f"control={control_run.final_accepted_sequence} "
        f"precision={precision_run.final_accepted_sequence} "
        f"delta={accepted_sequence_delta} ratio={accepted_sequence_delta_ratio:.9f}",
    )
    add(
        "absolute same-sequence stamp phase is within two raw frames (diagnostic)",
        bool(common_keys)
        and phase_summary["valid"]
        and phase_summary["absolute_max_frames"] <= 2.0,
        f"raw_frame_period={raw_frame_period_sec:.9f}s "
        f"max_frames={phase_summary['absolute_max_frames']:.3f}",
        "warn",
    )
    add(
        "same-sequence stamps follow an integral raw-frame phase",
        bool(common_keys)
        and phase_summary["valid"]
        and phase_summary["period_residual_max_sec"] <= 0.005,
        f"raw_frame_period={raw_frame_period_sec:.9f}s "
        f"max_period_residual="
        f"{phase_summary['period_residual_max_sec']:.9f}s <=0.005000000s",
    )
    add(
        "same-sequence phase is stable and non-accumulating",
        bool(common_keys)
        and phase_summary["valid"]
        and phase_summary["robust_span_frames"] <= 2.0
        and phase_summary["early_late_drift_frames"] <= 2.0,
        f"p01/p99={phase_summary['robust_low_frames']:.3f}/"
        f"{phase_summary['robust_high_frames']:.3f}frames "
        f"robust_span={phase_summary['robust_span_frames']:.3f}frames "
        f"early/late_median={phase_summary['early_median_frames']:.3f}/"
        f"{phase_summary['late_median_frames']:.3f}frames "
        f"drift={phase_summary['early_late_drift_frames']:.3f}frames",
    )
    add(
        "accepted-scan raw XY non-intrusion",
        stats(xy)["rmse"] <= 0.05,
        f"rmse={stats(xy)['rmse']:.9f}m <=0.05m",
    )
    add(
        "accepted-scan raw yaw non-intrusion",
        stats(yaw)["rmse"] <= 0.10,
        f"rmse={stats(yaw)['rmse']:.9f}deg <=0.10deg",
    )
    add(
        "accepted-scan raw increment non-intrusion",
        stats(translation_increment)["rmse"] <= 0.02
        and stats(yaw_increment)["rmse"] <= 0.05,
        f"translation={stats(translation_increment)['rmse']:.9f}m "
        f"yaw={stats(yaw_increment)['rmse']:.9f}deg",
    )
    add(
        "phase-compensated accepted-scan pose non-intrusion",
        interpolation_coverage >= 0.99
        and len(interpolated_xy_error) >= 3
        and stats(interpolated_xy_error)["rmse"] <= 0.05
        and stats(interpolated_yaw_error)["rmse"] <= 0.10,
        f"coverage={int(np.count_nonzero(interpolation_valid))}/{len(common_keys)} "
        f"({interpolation_coverage:.9f}) xy_rmse="
        f"{stats(interpolated_xy_error)['rmse'] if len(interpolated_xy_error) else math.inf:.9f}m "
        f"yaw_rmse="
        f"{stats(interpolated_yaw_error)['rmse'] if len(interpolated_yaw_error) else math.inf:.9f}deg",
    )
    add(
        "phase-compensated accepted-scan increment non-intrusion",
        interpolation_pair_coverage >= 0.99
        and len(interpolated_translation_increment_error) >= 2
        and stats(interpolated_translation_increment_error)["rmse"] <= 0.02
        and stats(interpolated_yaw_increment_error)["rmse"] <= 0.05,
        f"coverage={len(interpolation_pair_indices)}/"
        f"{interpolation_pair_denominator} ({interpolation_pair_coverage:.9f}) "
        f"translation="
        f"{stats(interpolated_translation_increment_error)['rmse'] if len(interpolated_translation_increment_error) else math.inf:.9f}m "
        f"yaw="
        f"{stats(interpolated_yaw_increment_error)['rmse'] if len(interpolated_yaw_increment_error) else math.inf:.9f}deg",
    )
    failed = [
        item for item in checks
        if item["category"] == "hard" and not item["passed"]
    ]
    return {
        "passed": not failed,
        "method": {
            "association": "exact common SubmapScan.header.stamp physical scans",
            "pose": "SubmapScan.raw_pose from the accepted registration event",
            "control": (
                "same snapshot override and recorder load, matcher/global overlay absent"
            ),
            "timer_interpolation": (
                "bounded SE(2) interpolation of physical accepted snapshots only; "
                "timer trajectories are never interpolated"
            ),
            "session_ids": (
                "opaque and intentionally not compared across independent runs"
            ),
            "phase_model": (
                "absolute integer-frame offset is diagnostic; hard gates bound "
                "period residual, robust phase spread, and early-to-late drift"
            ),
            "phase_interpolation": (
                "control accepted-snapshot SE(2) interpolated at precision physical stamps; "
                "pose and increment checked without extrapolation"
            ),
        },
        "inputs": {
            "control_bag": str(resolve(args.control_bag)),
            "precision_bag": str(resolve(args.precision_bag)),
        },
        "counts": {
            "control": len(control),
            "precision": len(precision),
            "common": len(common_stamps),
            "coverage": coverage,
            "generation_sequence_common": len(common_keys),
            "generation_sequence_coverage": key_coverage,
        },
        "metrics": {
            "control_accepted_scan_record_lag_sec": stats(control_record_lag_sec),
            "precision_accepted_scan_record_lag_sec": stats(
                precision_record_lag_sec
            ),
            "xy_difference_m": stats(xy),
            "yaw_difference_deg": stats(yaw),
            "translation_increment_difference_m": stats(translation_increment),
            "yaw_increment_difference_deg": stats(yaw_increment),
            "same_sequence_stamp_offset_sec": stats(key_stamp_offsets_sec),
            "same_sequence_signed_stamp_offset_sec": stats(
                signed_key_stamp_offsets_sec
            ),
            "same_sequence_phase_residual_sec": stats(phase_residual_sec),
            "same_sequence_phase_stability": phase_summary,
            "phase_compensated_xy_difference_m": stats(interpolated_xy_error),
            "phase_compensated_yaw_difference_deg": stats(interpolated_yaw_error),
            "phase_compensated_coverage": interpolation_coverage,
            "phase_compensated_translation_increment_difference_m": stats(
                interpolated_translation_increment_error
            ),
            "phase_compensated_yaw_increment_difference_deg": stats(
                interpolated_yaw_increment_error
            ),
            "phase_compensated_increment_coverage": interpolation_pair_coverage,
        },
        "snapshot_policy": {
            "control": control_policy,
            "precision": precision_policy,
        },
        "checks": checks,
    }


def markdown(result: dict[str, Any], label: str) -> str:
    lines = [
        f"# {label} accepted-scan non-intrusion",
        "",
        f"- result: **{'PASS' if result['passed'] else 'FAIL'}**",
        f"- common coverage: {result['counts']['common']}/"
        f"{max(result['counts']['control'], result['counts']['precision'])} "
        f"({result['counts']['coverage']:.9f})",
        "- hard association: exact physical stamps plus generation/sequence phase bounds",
        "",
        "## Checks",
        "",
    ]
    for item in result["checks"]:
        level = "WARN" if item["category"] == "warn" and not item["passed"] else (
            "PASS" if item["passed"] else "FAIL"
        )
        lines.append(f"- {level}: {item['name']} — {item['detail']}")
    lines.append("")
    return "\n".join(lines)


def parse() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--control-bag", type=Path, required=True)
    parser.add_argument("--precision-bag", type=Path, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-markdown", type=Path, required=True)
    parser.add_argument("--snapshot-interval", type=int, default=5)
    parser.add_argument("--maximum-interpolation-gap", type=float, default=0.40)
    return parser.parse_args()


def main() -> int:
    args = parse()
    if args.snapshot_interval < 1 or args.maximum_interpolation_gap <= 0.0:
        raise SystemExit("snapshot interval and interpolation gap must be positive")
    result = evaluate(args)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    args.output_markdown.write_text(markdown(result, args.label))
    print(args.output_json)
    print(args.output_markdown)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
