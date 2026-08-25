#!/usr/bin/env python3
"""Read-only startup safety acceptance for isolated precision bags.

Activation is proven from evidence latched by the producer at commit time and
an exact typed fusion-authority event.  Live diagnostic health values are not
accepted as historical evidence for activation.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import importlib.util
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


RAW = "/localization/gyro_lidar_odom"
EXISTING = "/localization/ekf_odom"
LOCAL = "/localization/precision_local_odom"
GLOBAL = "/localization/precision_global_odom"
GLOBAL_POSE = "/localization/precision_global_pose"
# Kept as a public compatibility constant.  The committed activation stamp is
# serialized as an integer nanosecond value, so no rounding tolerance applies.
ACTIVATION_SERIALIZATION_TOLERANCE_NS = 0
CALIBRATION_MAXIMUM_INTERPOLATION_GAP_SEC = 0.1
CALIBRATION_ASSOCIATION = (
    "GLIM reference timestamps inside the inclusive explicit window; "
    "speed legacy-global yaw linearly interpolated to those GLIM timestamps"
)


def load(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def true_value(value: Any) -> bool:
    return str(value).strip().lower() == "true"


def integer_value(value: Any) -> int:
    if isinstance(value, (bytes, bytearray)):
        return value[0]
    return int(value)


def wrap(value: np.ndarray | float) -> np.ndarray | float:
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def calibration_window_bounds(start_sec: float, end_sec: float) -> tuple[float, float]:
    """Validate the explicit calibration interval before any bag processing."""
    start = float(start_sec)
    end = float(end_sec)
    if not math.isfinite(start) or not math.isfinite(end):
        raise RuntimeError("calibration window bounds must be finite")
    if end <= start:
        raise RuntimeError("calibration end must be greater than calibration start")
    return start, end


def calibration_window_provenance(
    reference_stamps_ns: Any,
    estimate_yaw_rad: Any,
    reference_yaw_rad: Any,
    start_sec: float,
    end_sec: float,
) -> dict[str, Any]:
    """Derive and fully serialize the frozen startup-yaw calibration."""
    start, end = calibration_window_bounds(start_sec, end_sec)
    stamps = np.asarray(reference_stamps_ns, dtype=np.int64)
    estimate = np.asarray(estimate_yaw_rad, dtype=float)
    reference = np.asarray(reference_yaw_rad, dtype=float)
    if (
        stamps.ndim != 1
        or estimate.ndim != 1
        or reference.ndim != 1
        or not (len(stamps) == len(estimate) == len(reference))
    ):
        raise RuntimeError("calibration common series are not one-dimensional and aligned")
    if (
        len(stamps) == 0
        or np.any(stamps <= 0)
        or np.any(np.diff(stamps) <= 0)
        or not np.all(np.isfinite(estimate))
        or not np.all(np.isfinite(reference))
    ):
        raise RuntimeError("calibration common series are invalid")

    start_ns = int(round(start * 1.0e9))
    end_ns = int(round(end * 1.0e9))
    mask = (stamps >= start_ns) & (stamps <= end_ns)
    common_sample_count = int(np.count_nonzero(mask))
    if common_sample_count < 3:
        raise RuntimeError("calibration window has fewer than three common samples")

    delta = wrap(reference[mask] - estimate[mask])
    yaw_offset_rad = math.atan2(
        float(np.mean(np.sin(delta))), float(np.mean(np.cos(delta)))
    )
    if not math.isfinite(yaw_offset_rad):
        raise RuntimeError("calibration circular yaw offset is not finite")
    return {
        "start_sec": start,
        "end_sec": end,
        "duration_sec": end - start,
        "common_sample_count": common_sample_count,
        "frozen_circular_yaw_offset_rad": yaw_offset_rad,
        "frozen_circular_yaw_offset_deg": math.degrees(yaw_offset_rad),
        "association": CALIBRATION_ASSOCIATION,
        "maximum_interpolation_gap_sec": CALIBRATION_MAXIMUM_INTERPOLATION_GAP_SEC,
        "window_bounds": "inclusive",
        "timestamp_source": "physical ROS header stamps",
    }


def startup_contract(calibration: dict[str, Any]) -> dict[str, Any]:
    """Build the startup gate contract, including replayable calibration data."""
    return {
        "readiness": "position_initialized && yaw_publishable",
        "candidate_count": 3,
        "candidate_delta_max_rad": 0.08,
        "first_output_activation_yaw_application_max_rad": 0.02,
        "activation_stamp_serialization_tolerance_sec": 0.0,
        "activation_timestamp": "producer-latched integer nanoseconds",
        "first_publish_rule": "exactly the next unique raw stamp after activation",
        "first_global_delay_max_sec": 25.0,
        "absolute_yaw_safety_max_deg": 10.0,
        "first_legacy_global_yaw_difference_max_deg": 3.0,
        "alignment": "speed legacy-global calibration yaw offset frozen for GLIM",
        "calibration": dict(calibration),
        "authority": (
            "exact producer-latched typed FULL_SE2_HEALTHY endpoint; existing "
            "fusion only; position-only GNSS fallback disabled"
        ),
        "session_rearm": (
            "fresh explicit unhealthy/non-TRACKING status, then fresh strict TRACKING; "
            "stale/unavailable alone never qualifies"
        ),
    }


def interpolate_one(trajectory: Any, stamp_ns: int) -> tuple[np.ndarray, float] | None:
    source = trajectory.stamp_ns
    insertion = int(np.searchsorted(source, stamp_ns, side="left"))
    if insertion < len(source) and int(source[insertion]) == stamp_ns:
        return trajectory.xy[insertion], float(trajectory.yaw[insertion])
    if insertion <= 0 or insertion >= len(source):
        return None
    left = insertion - 1
    right = insertion
    gap_sec = (int(source[right]) - int(source[left])) * 1.0e-9
    if gap_sec > 0.15:
        return None
    fraction = (stamp_ns - int(source[left])) / (int(source[right]) - int(source[left]))
    xy = trajectory.xy[left] + fraction * (trajectory.xy[right] - trajectory.xy[left])
    yaw_delta = float(wrap(float(trajectory.yaw[right]) - float(trajectory.yaw[left])))
    yaw = float(wrap(float(trajectory.yaw[left]) + fraction * yaw_delta))
    return xy, yaw


def activation_raw_successor(
    raw_stamps: Any, activation_ns: int, tolerance_ns: int
) -> tuple[int | None, int | None]:
    """Resolve the exact committed activation stamp and its next raw sample."""
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


def read_pose_stamps(repo_eval: Any, bag: Path, canonical: Any, precision: bool) -> dict[str, Any]:
    return repo_eval.read_bag(bag, canonical, precision)


def read_output_streams(repo_eval: Any, bag: Path) -> dict[str, list[dict[str, Any]]]:
    _, deserialize_message, get_message = repo_eval.import_ros()
    reader = repo_eval.open_reader(bag)
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = {LOCAL, GLOBAL, GLOBAL_POSE}
    missing = wanted - set(types)
    if missing:
        raise RuntimeError(f"startup output topics missing: {sorted(missing)}")
    classes = {topic: get_message(types[topic]) for topic in wanted}
    result: dict[str, list[dict[str, Any]]] = {topic: [] for topic in wanted}
    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(serialized, classes[topic])
        stamp = repo_eval.stamp_ns(msg.header.stamp)
        if stamp <= 0:
            continue
        pose = msg.pose.pose
        result[topic].append(
            {
                "record_ns": int(record_ns),
                "stamp_ns": stamp,
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": repo_eval.yaw_of(pose.orientation),
            }
        )
    return result


def read_activation_evidence_streams(
    repo_eval: Any, validator: Any, bag: Path
) -> dict[str, Any]:
    """Read exact typed authority, valid G endpoints, and odom-session resets."""
    _, deserialize_message, get_message = repo_eval.import_ros()
    reader = repo_eval.open_reader(bag)
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    authority_topic = validator.TOPIC_AUTHORITY
    existing_topic = validator.TOPIC_EXISTING
    scan_topic = validator.TOPIC_SCAN
    missing = {authority_topic, existing_topic, scan_topic} - set(types)
    if missing:
        raise RuntimeError(
            f"startup activation evidence topics missing: {sorted(missing)}"
        )
    if types[authority_topic] != validator.TYPE_AUTHORITY:
        raise RuntimeError(
            "startup fusion authority has unexpected type: "
            f"expected={validator.TYPE_AUTHORITY} actual={types[authority_topic]}"
        )
    classes = {
        authority_topic: get_message(types[authority_topic]),
        existing_topic: get_message(types[existing_topic]),
        scan_topic: get_message(types[scan_topic]),
    }
    authority_records: list[dict[str, Any]] = []
    existing_global_records: list[tuple[int, Any]] = []
    existing_global_stamps: set[int] = set()
    session_reset_stamps: set[int] = set()
    prior_scan_session: int | None = None
    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        if topic not in classes:
            continue
        message = deserialize_message(serialized, classes[topic])
        if topic == authority_topic:
            authority_records.append(
                validator.fusion_authority_record(record_ns, message)
            )
        elif topic == existing_topic:
            existing_global_records.append((int(record_ns), message))
            physical_stamp_ns = validator.stamp_ns(message.header.stamp)
            if validator.existing_global_contract_valid(message):
                existing_global_stamps.add(physical_stamp_ns)
        else:
            session = int(message.odom_session_id)
            physical_stamp_ns = validator.stamp_ns(message.header.stamp)
            if (
                prior_scan_session is not None
                and session != prior_scan_session
                and physical_stamp_ns > 0
            ):
                session_reset_stamps.add(physical_stamp_ns)
            prior_scan_session = session
    return {
        "authority_records": authority_records,
        "existing_global_stamps": existing_global_stamps,
        "existing_global_stream_contract": (
            validator.existing_global_prefix_accounting(existing_global_records)
        ),
        "session_reset_stamps": session_reset_stamps,
    }


def diagnostic_activation(diagnostics: list[dict[str, Any]]) -> dict[str, Any]:
    required = {
        "position_initialized",
        "position_fused",
        "yaw_publishable",
        "global_output_ready",
        "anchor.initialized",
        "anchor.yaw_observed",
        "anchor.yaw_publishable",
        "activation.required_candidate_count",
        "activation.max_candidate_delta_rad",
        "activation.reason",
        "activation.epoch",
        "activation.commit_count",
        "activation.candidate_yaw_rad",
        "activation.evidence_valid",
        "activation.stamp_ns",
        "activation.committed_stable_candidate_count",
        "activation.committed_candidate_delta_rad",
        "activation.authority_session_id",
        "activation.authority_sequence",
        "activation.authority_stamp_ns",
        "activation.authority_source_stamp_ns",
        "activation.authority_received_stamp_ns",
        "activation.existing_global_lower_stamp_ns",
        "activation.existing_global_upper_stamp_ns",
        "activation.existing_global_watermark_ns",
        "activation.existing_global_max_interpolation_gap_ns",
        "activation.existing_global_mode",
        "publish.global_suppressed_not_ready",
        "publish.global_suppressed_activation_watermark",
        "publish.global",
        "anchor.source",
        "fusion.health.rearm_required",
        "fusion.health.rearm_saw_unhealthy",
        "fusion.health.rearmed",
        "fusion.health.rearm_reset_stamp_sec",
        "fusion.health.rearm.reset_stamp_ns",
        "fusion.health.rearm.unhealthy_evidence_valid",
        "fusion.health.rearm.unhealthy_session_id",
        "fusion.health.rearm.unhealthy_sequence",
        "fusion.health.rearm.unhealthy_stamp_ns",
        "fusion.health.rearm.unhealthy_source_stamp_ns",
        "fusion.health.rearm.unhealthy_received_stamp_ns",
        "fusion.health.rearm.healthy_evidence_valid",
        "fusion.health.rearm.healthy_session_id",
        "fusion.health.rearm.healthy_sequence",
        "fusion.health.rearm.healthy_stamp_ns",
        "fusion.health.rearm.healthy_source_stamp_ns",
        "fusion.health.rearm.healthy_received_stamp_ns",
        "fusion.anchor.state",
        "fusion.anchor.frozen_residual_variance_x_m2",
        "fusion.anchor.frozen_residual_variance_y_m2",
        "fusion.anchor.frozen_residual_variance_yaw_rad2",
        "fusion.sync.existing_global_stamp_ns",
        "local_correction.odom_session_resets",
        "fallback.gnss_position_enabled",
    }
    all_keys = set().union(*(set(item["values"]) for item in diagnostics)) if diagnostics else set()
    missing = sorted(required - all_keys)
    incomplete_samples = sum(
        not required.issubset(item["values"]) for item in diagnostics
    )
    ready = [
        item for item in diagnostics
        if true_value(item["values"].get("global_output_ready", "false"))
    ]
    activation = ready[0] if ready else None
    activation_ns = None
    if activation is not None:
        try:
            value = int(activation["values"].get("activation.stamp_ns", "0"))
        except (TypeError, ValueError):
            value = 0
        if value > 0:
            activation_ns = value
    return {
        "required_keys": sorted(required),
        "missing_keys": missing,
        "incomplete_samples": incomplete_samples,
        "first_ready_diagnostic": activation,
        "activation_ns": activation_ns,
    }


def diagnostic_transition_contract(
    diagnostics: list[dict[str, Any]],
    authority_records: list[dict[str, Any]] | None = None,
    existing_global_stamps: set[int] | None = None,
    validator: Any | None = None,
) -> dict[str, Any]:
    """Validate readiness using the shared committed-evidence contract."""
    epochs = sorted({
        int(item["values"]["activation.epoch"])
        for item in diagnostics
        if str(item["values"].get("activation.epoch", "")).isdigit()
    })
    if authority_records is None or existing_global_stamps is None:
        return {
            "valid": False,
            "epochs": epochs,
            "transitions": [],
            "errors": [
                "typed fusion-authority records and existing-global stamps "
                "are required for committed activation evidence"
            ],
        }
    if validator is None:
        validator = load(
            "startup_precision_validator_contract",
            Path(__file__).with_name("validate_precision_bag.py"),
        )
    valid, transitions, errors = validator.startup_transition_contract(
        diagnostics, authority_records, existing_global_stamps
    )
    return {
        "valid": valid,
        "epochs": epochs,
        "transitions": transitions,
        "errors": errors,
    }


def legacy_derived_activation(diagnostics: list[dict[str, Any]]) -> dict[str, Any] | None:
    """Diagnostic-rate proxy for the proposed three-stable-candidate gate."""
    candidates: list[tuple[dict[str, Any], float]] = []
    for item in diagnostics:
        values = item["values"]
        if not true_value(values.get("anchor.yaw_observed", "false")):
            candidates.clear()
            continue
        try:
            yaw = float(values["anchor.target.yaw_rad"])
        except (KeyError, ValueError):
            candidates.clear()
            continue
        if candidates and abs(float(wrap(yaw - candidates[-1][1]))) > 0.08:
            candidates.clear()
        candidates.append((item, yaw))
        if len(candidates) >= 3:
            selected = candidates[-1][0]
            values = selected["values"]
            return {
                "stamp_ns": int(selected["stamp_ns"]),
                "target_yaw_rad": yaw,
                "applied_yaw_rad": float(values.get("anchor.applied.yaw_rad", "nan")),
                "lag_yaw_rad": abs(float(wrap(
                    yaw - float(values.get("anchor.applied.yaw_rad", "nan"))
                ))),
                "note": "1 Hz diagnostic proxy only; not a substitute for activation event keys",
            }
    return None


def common_yaw_series(canonical: Any, reference: Any, trajectories: dict[str, Any]):
    return canonical.common_samples(
        reference, trajectories, CALIBRATION_MAXIMUM_INTERPOLATION_GAP_SEC
    )


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    calibration_start, calibration_end = calibration_window_bounds(
        args.calibration_start, args.calibration_end
    )
    repo = args.repo.resolve()
    canonical = load("startup_canonical", repo / "tools/evaluate_glim_trajectory.py")
    repo_eval = load(
        "startup_repo_eval",
        repo / "src/pure_precision_bringup/scripts/evaluate_precision_glim_ab.py",
    )
    validator = load(
        "startup_precision_validator",
        repo / "src/pure_precision_bringup/scripts/validate_precision_bag.py",
    )
    speed = read_pose_stamps(repo_eval, args.speed_bag, canonical, False)
    precision = read_pose_stamps(repo_eval, args.precision_bag, canonical, True)
    output_streams = read_output_streams(repo_eval, args.precision_bag)
    activation_evidence_streams = read_activation_evidence_streams(
        repo_eval, validator, args.precision_bag
    )
    authority_records = activation_evidence_streams["authority_records"]
    existing_global_stamps = activation_evidence_streams[
        "existing_global_stamps"
    ]
    existing_global_stream_contract = activation_evidence_streams[
        "existing_global_stream_contract"
    ]
    session_reset_stamps = activation_evidence_streams["session_reset_stamps"]
    authority_stream_contract = validator.fusion_authority_prefix_accounting(
        authority_records
    )
    reference = canonical.read_glim_trajectory(args.glim_trajectory)

    calibration_reference, calibration = common_yaw_series(
        canonical,
        reference,
        {"speed_existing": speed["trajectories"][EXISTING]},
    )
    calibration_provenance = calibration_window_provenance(
        calibration_reference.stamp_ns,
        calibration["speed_existing"].yaw,
        calibration_reference.yaw,
        calibration_start,
        calibration_end,
    )
    yaw_offset = calibration_provenance["frozen_circular_yaw_offset_rad"]
    # Keep this independent implementation comparison as a fail-closed guard
    # against evaluator/helper drift.
    calibration_mask = (
        (calibration_reference.stamp_ns >= int(round(calibration_start * 1.0e9)))
        & (calibration_reference.stamp_ns <= int(round(calibration_end * 1.0e9)))
    )
    evaluator_yaw_offset = repo_eval.circular_offset(
        calibration["speed_existing"].yaw, calibration_reference.yaw, calibration_mask
    )
    if not math.isclose(
        yaw_offset, evaluator_yaw_offset, rel_tol=0.0, abs_tol=1.0e-15
    ):
        raise RuntimeError("calibration circular yaw offset implementations disagree")

    global_reference, global_values = common_yaw_series(
        canonical,
        reference,
        {
            "precision_global": precision["trajectories"][GLOBAL],
            "speed_existing": speed["trajectories"][EXISTING],
        },
    )
    precision_global = global_values["precision_global"]
    speed_existing = global_values["speed_existing"]
    glim_yaw_error = np.degrees(np.abs(wrap(
        precision_global.yaw + yaw_offset - global_reference.yaw
    )))
    existing_yaw_difference = np.degrees(np.abs(wrap(
        precision_global.yaw - speed_existing.yaw
    )))

    # Safety maxima use every published record, not the trajectory helper's
    # last-per-duplicate-stamp representation.
    full_global = output_streams[GLOBAL]
    full_global_stamps = np.asarray(
        [item["stamp_ns"] for item in full_global], dtype=np.int64
    )
    full_global_yaw = np.asarray([item["yaw"] for item in full_global])
    full_reference, full_valid = canonical.interpolate_trajectory(
        reference, full_global_stamps, 0.1
    )
    full_existing, existing_valid = canonical.interpolate_trajectory(
        speed["trajectories"][EXISTING], full_global_stamps, 0.1
    )
    common_valid = full_valid & existing_valid
    if not np.all(common_valid):
        common_stamps = full_global_stamps[common_valid]
        full_reference, _ = canonical.interpolate_trajectory(reference, common_stamps, 0.1)
        full_existing, _ = canonical.interpolate_trajectory(
            speed["trajectories"][EXISTING], common_stamps, 0.1
        )
    full_yaw = full_global_yaw[common_valid]
    full_glim_yaw_error = np.degrees(np.abs(wrap(
        full_yaw + yaw_offset - full_reference.yaw
    )))
    full_existing_yaw_difference = np.degrees(np.abs(wrap(
        full_yaw - full_existing.yaw
    )))

    raw = precision["trajectories"][RAW]
    global_trajectory = precision["trajectories"][GLOBAL]
    first_raw_ns = int(raw.stamp_ns[0])
    if not full_global:
        raise RuntimeError("no positive precision-global output messages")
    first_global_record = full_global[0]
    first_global_ns = int(first_global_record["stamp_ns"])
    activation = diagnostic_activation(precision["precision_diagnostics"])
    transition_contract = diagnostic_transition_contract(
        precision["precision_diagnostics"],
        authority_records,
        existing_global_stamps,
        validator,
    )
    activation_ns = activation["activation_ns"]
    activation_raw_ns, expected_first_global_ns = (
        (None, None)
        if activation_ns is None
        else activation_raw_successor(
            raw.stamp_ns,
            activation_ns,
            0,
        )
    )
    legacy_activation = legacy_derived_activation(precision["precision_diagnostics"])
    freeze_valid, freeze_groups, freeze_errors = validator.fusion_anchor_freeze_contract(
        precision["precision_diagnostics"], authority_records
    )
    rearm_valid, rearm_summary, rearm_errors = validator.fusion_rearm_contract(
        precision["precision_diagnostics"], authority_records, session_reset_stamps
    )

    first_xy = np.asarray([first_global_record["x"], first_global_record["y"]])
    first_yaw = float(first_global_record["yaw"])
    first_local_candidates = [
        item for item in output_streams[LOCAL]
        if item["stamp_ns"] == first_global_ns
        and item["record_ns"] <= first_global_record["record_ns"]
    ]
    if not first_local_candidates:
        first_local_candidates = [
            item for item in output_streams[LOCAL]
            if item["stamp_ns"] == first_global_ns
        ]
    first_local_value = (
        max(first_local_candidates, key=lambda item: item["record_ns"])
        if first_local_candidates else None
    )
    first_glim_value = interpolate_one(reference, first_global_ns)
    first_existing_value = interpolate_one(speed["trajectories"][EXISTING], first_global_ns)
    first_glim_error = None if first_glim_value is None else abs(math.degrees(float(wrap(
        first_yaw + yaw_offset - first_glim_value[1]
    ))))
    first_existing_difference = None if first_existing_value is None else abs(
        math.degrees(float(wrap(first_yaw - first_existing_value[1])))
    )

    global_stamps = [item["stamp_ns"] for item in full_global]
    global_pose_stamps = [item["stamp_ns"] for item in output_streams[GLOBAL_POSE]]
    first_global_pose_ns = global_pose_stamps[0] if global_pose_stamps else None
    activation_lower_bound_ns = activation_ns
    before_ready_odom = None if activation_lower_bound_ns is None else sum(
        stamp < activation_lower_bound_ns for stamp in global_stamps
    )
    before_ready_pose = None if activation_lower_bound_ns is None else sum(
        stamp < activation_lower_bound_ns for stamp in global_pose_stamps
    )
    first_ready_diag = activation["first_ready_diagnostic"]
    committed_stable_count = None
    required_count = None
    committed_candidate_delta = None
    maximum_candidate_delta = None
    activation_reason = None
    candidate_yaw = None
    first_output_anchor_yaw_error = None
    first_ready_existing_fusion_authority = False
    committed_candidate_evidence_valid = False
    activation_authority_valid = False
    activation_authority_endpoint: tuple[int, int] | None = None
    activation_authority_age_ns: int | None = None
    activation_authority_source_age_ns: int | None = None
    activation_authority_transport_age_ns: int | None = None
    activation_existing_global_lower_ns: int | None = None
    activation_existing_global_upper_ns: int | None = None
    activation_existing_global_watermark_ns: int | None = None
    activation_existing_global_watermark_valid = False
    if first_ready_diag is not None:
        values = first_ready_diag["values"]
        try:
            committed_stable_count = int(
                values.get("activation.committed_stable_candidate_count", "-1")
            )
            required_count = int(
                values.get("activation.required_candidate_count", "-1")
            )
            committed_candidate_delta = float(
                values.get("activation.committed_candidate_delta_rad", "nan")
            )
            maximum_candidate_delta = float(
                values.get("activation.max_candidate_delta_rad", "nan")
            )
            authority_session = int(
                values.get("activation.authority_session_id", "0")
            )
            authority_sequence = int(
                values.get("activation.authority_sequence", "0")
            )
            authority_stamp_ns = int(
                values.get("activation.authority_stamp_ns", "0")
            )
            authority_source_stamp_ns = int(
                values.get("activation.authority_source_stamp_ns", "0")
            )
            authority_received_stamp_ns = int(
                values.get("activation.authority_received_stamp_ns", "0")
            )
            activation_existing_global_lower_ns = int(
                values.get("activation.existing_global_lower_stamp_ns", "0")
            )
            activation_existing_global_upper_ns = int(
                values.get("activation.existing_global_upper_stamp_ns", "0")
            )
            activation_existing_global_watermark_ns = int(
                values.get("activation.existing_global_watermark_ns", "0")
            )
            existing_global_max_interpolation_gap_ns = int(
                values.get(
                    "activation.existing_global_max_interpolation_gap_ns", "0"
                )
            )
            existing_global_mode = values.get(
                "activation.existing_global_mode", "none"
            )
        except (TypeError, ValueError):
            committed_stable_count = -1
            required_count = -1
            committed_candidate_delta = math.nan
            maximum_candidate_delta = math.nan
            authority_session = 0
            authority_sequence = 0
            authority_stamp_ns = 0
            authority_source_stamp_ns = 0
            authority_received_stamp_ns = 0
            activation_existing_global_lower_ns = 0
            activation_existing_global_upper_ns = 0
            activation_existing_global_watermark_ns = 0
            existing_global_max_interpolation_gap_ns = 0
            existing_global_mode = "none"
        activation_reason = values.get("activation.reason")
        try:
            candidate_yaw = float(
                values.get("activation.candidate_yaw_rad", "nan")
            )
        except (TypeError, ValueError):
            candidate_yaw = math.nan
        first_ready_existing_fusion_authority = (
            values.get("anchor.source") == "existing_fusion"
            and not true_value(values.get("fallback.gnss_position_enabled", "true"))
        )
        committed_candidate_evidence_valid = (
            true_value(values.get("activation.evidence_valid", "false"))
            and required_count >= 3
            and committed_stable_count == required_count
            and activation_reason == "existing_fusion_stable_activated"
            and math.isfinite(candidate_yaw)
            and math.isfinite(committed_candidate_delta)
            and math.isfinite(maximum_candidate_delta)
            and maximum_candidate_delta > 0.0
            and 0.0 <= committed_candidate_delta <= maximum_candidate_delta
        )
        activation_authority_endpoint = (authority_session, authority_sequence)
        matching_authorities = [
            item for item in authority_records
            if int(item.get("session_id", 0)) == authority_session
            and int(item.get("sequence", 0)) == authority_sequence
        ]
        matching_authority = (
            matching_authorities[0] if len(matching_authorities) == 1 else None
        )
        activation_authority_age_ns = (
            activation_ns - int(matching_authority.get("stamp_ns", 0))
            if activation_ns is not None and matching_authority is not None
            else None
        )
        activation_authority_source_age_ns = (
            int(matching_authority.get("stamp_ns", 0))
            - int(matching_authority.get("source_stamp_ns", 0))
            if matching_authority is not None else None
        )
        activation_authority_transport_age_ns = (
            authority_received_stamp_ns
            - int(matching_authority.get("stamp_ns", 0))
            if matching_authority is not None else None
        )
        activation_authority_valid = (
            authority_session > 0
            and authority_sequence > 0
            and matching_authority is not None
            and int(matching_authority.get("state", -1)) == 1
            and matching_authority.get("reason")
            == "strict_full_se2_authority_ok"
            and matching_authority.get("recovery_state") == "tracking"
            and matching_authority.get("anchor_valid") is True
            and matching_authority.get("position_fused") is True
            and matching_authority.get("yaw_fused") is True
            and int(matching_authority.get("last_fix_state", -1)) == 1
            and int(matching_authority.get("stamp_ns", 0))
            == authority_stamp_ns
            and int(matching_authority.get("source_stamp_ns", 0))
            == authority_source_stamp_ns
            and activation_authority_age_ns is not None
            and -validator.FUSION_AUTHORITY_MAX_FUTURE_SKEW_NS
            <= activation_authority_age_ns
            <= validator.FUSION_AUTHORITY_MAX_SOURCE_AGE_NS
            and activation_authority_source_age_ns is not None
            and -validator.FUSION_AUTHORITY_MAX_FUTURE_SKEW_NS
            <= activation_authority_source_age_ns
            <= validator.FUSION_AUTHORITY_MAX_SOURCE_AGE_NS
            and activation_authority_transport_age_ns is not None
            and -validator.FUSION_AUTHORITY_MAX_FUTURE_SKEW_NS
            <= activation_authority_transport_age_ns
            <= validator.FUSION_AUTHORITY_MAX_SOURCE_AGE_NS
        )
        exact_global_endpoint = (
            existing_global_mode == "exact"
            and activation_existing_global_lower_ns == activation_ns
            and activation_existing_global_upper_ns == activation_ns
        )
        interpolated_global_endpoints = (
            activation_ns is not None
            and existing_global_mode == "interpolated"
            and activation_existing_global_lower_ns < activation_ns
            < activation_existing_global_upper_ns
            and existing_global_max_interpolation_gap_ns > 0
            and activation_existing_global_upper_ns
            - activation_existing_global_lower_ns
            <= existing_global_max_interpolation_gap_ns
        )
        activation_existing_global_watermark_valid = (
            activation_ns is not None
            and activation_existing_global_lower_ns in existing_global_stamps
            and activation_existing_global_upper_ns in existing_global_stamps
            and activation_existing_global_watermark_ns in existing_global_stamps
            and activation_existing_global_watermark_ns
            >= activation_existing_global_upper_ns
            and (exact_global_endpoint or interpolated_global_endpoints)
        )
        if first_local_value is not None and math.isfinite(candidate_yaw):
            first_output_anchor_yaw_error = abs(float(wrap(
                first_yaw - float(first_local_value["yaw"]) - candidate_yaw
            )))

    authority_invariant = bool(precision["precision_diagnostics"]) and all(
        item["values"].get("anchor.source") == "existing_fusion"
        and item["values"].get(
            "fallback.gnss_position_enabled", "true"
        ) == "false"
        for item in precision["precision_diagnostics"]
    )
    no_session_rearm_needed = bool(precision["precision_diagnostics"]) and all(
        item["values"].get("local_correction.odom_session_resets") == "0"
        and item["values"].get("fusion.health.rearm_required") == "false"
        and item["values"].get("fusion.health.rearm_saw_unhealthy") == "false"
        and item["values"].get("fusion.health.rearmed") == "true"
        for item in precision["precision_diagnostics"]
    )

    checks: list[dict[str, Any]] = []

    def add(name: str, passed: bool, detail: str) -> None:
        checks.append({"name": name, "passed": bool(passed), "detail": detail})

    add(
        "startup readiness diagnostics complete",
        not activation["missing_keys"] and activation["incomplete_samples"] == 0,
        f"missing={activation['missing_keys']} "
        f"incomplete_samples={activation['incomplete_samples']}",
    )
    add(
        "readiness diagnostic transitions are exact",
        transition_contract["valid"],
        f"epochs={transition_contract['epochs']} errors={transition_contract['errors']}",
    )
    add(
        "typed fusion-authority stream is exact and ordered",
        authority_stream_contract["valid"],
        f"authority_stream={authority_stream_contract}",
    )
    add(
        "existing-global evidence stream is exact and ordered",
        existing_global_stream_contract["valid"],
        f"existing_global_stream={existing_global_stream_contract}",
    )
    add(
        "one exact integer-nanosecond activation event exists",
        activation_ns is not None,
        f"activation_ns={activation_ns}",
    )
    add(
        "global odom is silent before readiness",
        before_ready_odom == 0,
        f"messages_before_ready={before_ready_odom}",
    )
    add(
        "global pose is silent before readiness",
        before_ready_pose == 0,
        f"messages_before_ready={before_ready_pose}",
    )
    add(
        "first global output follows activation",
        activation_lower_bound_ns is not None
        and first_global_ns >= activation_lower_bound_ns,
        f"first_global={first_global_ns} exact_activation={activation_ns}",
    )
    add(
        "first global output is exactly the next raw sample after activation",
        activation_raw_ns is not None
        and expected_first_global_ns is not None
        and first_global_ns == expected_first_global_ns
        and first_global_pose_ns == expected_first_global_ns,
        f"exact_activation={activation_ns} activation_raw={activation_raw_ns} "
        f"expected_next_raw={expected_first_global_ns} "
        f"global_odom={first_global_ns} global_pose={first_global_pose_ns}",
    )
    add(
        "first global odom and pose are one atomic output pair",
        first_global_pose_ns == first_global_ns,
        f"odom={first_global_ns} pose={first_global_pose_ns}",
    )
    add(
        "first global delay is at most 25 seconds",
        0.0 <= (first_global_ns - first_raw_ns) * 1.0e-9 <= 25.0,
        f"delay={(first_global_ns-first_raw_ns)*1.0e-9:.6f}s",
    )
    add(
        "activation uses committed consecutive stable candidates",
        committed_candidate_evidence_valid,
        f"committed_stable={committed_stable_count} required={required_count} "
        f"committed_delta={committed_candidate_delta} "
        f"maximum_delta={maximum_candidate_delta} reason={activation_reason}",
    )
    add(
        "global anchor authority is existing fusion only",
        first_ready_existing_fusion_authority and authority_invariant,
        f"first_ready_authority={first_ready_existing_fusion_authority} "
        f"all_samples_authority={authority_invariant}",
    )
    add(
        "activation matches an exact strict typed fusion-authority event",
        activation_authority_valid,
        f"endpoint={activation_authority_endpoint} "
        f"activation_authority_age_ns={activation_authority_age_ns} "
        f"authority_source_age_ns={activation_authority_source_age_ns} "
        f"authority_transport_age_ns={activation_authority_transport_age_ns} "
        f"valid={activation_authority_valid}",
    )
    add(
        "activation existing-global watermark is exact",
        activation_existing_global_watermark_valid,
        f"endpoints_ns={activation_existing_global_lower_ns}/"
        f"{activation_existing_global_upper_ns} "
        f"watermark_ns={activation_existing_global_watermark_ns} "
        f"recorded={activation_existing_global_watermark_valid}",
    )
    add(
        "initial startup does not require session rearm",
        no_session_rearm_needed and rearm_valid,
        f"no_session_rearm_needed={no_session_rearm_needed} "
        f"summary={rearm_summary} errors={rearm_errors}",
    )
    add(
        "existing-fusion anchor is exactly frozen whenever strict health is false",
        freeze_valid,
        f"groups={freeze_groups} errors={freeze_errors}",
    )
    add(
        "first global pose applies the activation yaw atomically",
        first_output_anchor_yaw_error is not None
        and math.isfinite(first_output_anchor_yaw_error)
        and first_output_anchor_yaw_error <= 0.02,
        f"pose_local_candidate_error_rad={first_output_anchor_yaw_error}",
    )
    add(
        "first global yaw is within 10 degrees of frozen-frame GLIM",
        first_glim_error is not None and first_glim_error <= 10.0,
        "not covered by GLIM" if first_glim_error is None else f"error={first_glim_error:.6f}deg",
    )
    add(
        "first global yaw is within 3 degrees of legacy global",
        first_existing_difference is not None and first_existing_difference <= 3.0,
        "not covered by legacy global" if first_existing_difference is None else
        f"difference={first_existing_difference:.6f}deg",
    )
    add(
        "no precision-global yaw transient exceeds 10 degrees vs GLIM",
        float(np.max(full_glim_yaw_error)) <= 10.0,
        f"maximum={float(np.max(full_glim_yaw_error)):.6f}deg "
        f"records={len(full_glim_yaw_error)}",
    )
    add(
        "no precision-global yaw transient exceeds 10 degrees vs legacy global",
        float(np.max(full_existing_yaw_difference)) <= 10.0,
        f"maximum={float(np.max(full_existing_yaw_difference)):.6f}deg "
        f"records={len(full_existing_yaw_difference)}",
    )

    return {
        "label": args.label,
        "passed": all(item["passed"] for item in checks),
        "contract": startup_contract(calibration_provenance),
        "inputs": {
            "speed_bag": str(args.speed_bag.resolve()),
            "precision_bag": str(args.precision_bag.resolve()),
            "glim": str(args.glim_trajectory.resolve()),
            "calibration_start_sec": calibration_start,
            "calibration_end_sec": calibration_end,
        },
        "counts": {
            "precision_global_odom_positive_records": len(global_stamps),
            "precision_global_pose_positive_records": len(global_pose_stamps),
            "fusion_authority_records": len(authority_records),
            "existing_global_unique_positive_stamps": len(existing_global_stamps),
        },
        "startup": {
            "first_raw_ns": first_raw_ns,
            "first_global_ns": first_global_ns,
            "first_global_pose_ns": first_global_pose_ns,
            "activation_raw_ns": activation_raw_ns,
            "expected_first_global_ns": expected_first_global_ns,
            "first_global_delay_sec": (first_global_ns - first_raw_ns) * 1.0e-9,
            "activation": activation,
            "committed_activation_evidence": {
                "candidate_valid": committed_candidate_evidence_valid,
                "committed_stable_candidate_count": committed_stable_count,
                "required_candidate_count": required_count,
                "committed_candidate_delta_rad": committed_candidate_delta,
                "maximum_candidate_delta_rad": maximum_candidate_delta,
                "authority_endpoint": activation_authority_endpoint,
                "authority_age_ns": activation_authority_age_ns,
                "authority_source_age_ns": activation_authority_source_age_ns,
                "authority_transport_age_ns": (
                    activation_authority_transport_age_ns
                ),
                "authority_valid": activation_authority_valid,
                "existing_global_lower_stamp_ns": (
                    activation_existing_global_lower_ns
                ),
                "existing_global_upper_stamp_ns": (
                    activation_existing_global_upper_ns
                ),
                "existing_global_watermark_ns": (
                    activation_existing_global_watermark_ns
                ),
                "existing_global_watermark_valid": (
                    activation_existing_global_watermark_valid
                ),
            },
            "diagnostic_transition_contract": transition_contract,
            "typed_fusion_authority_stream_contract": (
                authority_stream_contract
            ),
            "existing_global_stream_contract": existing_global_stream_contract,
            "freeze_contract": {
                "valid": freeze_valid,
                "groups": freeze_groups,
                "errors": freeze_errors,
            },
            "session_rearm_contract": {
                "valid": rearm_valid,
                "summary": rearm_summary,
                "errors": rearm_errors,
            },
            "legacy_derived_activation": legacy_activation,
            "first_global_xy": first_xy.tolist(),
            "first_global_glim_yaw_error_deg": first_glim_error,
            "first_global_existing_yaw_difference_deg": first_existing_difference,
            "first_global_activation_yaw_error_rad": first_output_anchor_yaw_error,
        },
        "yaw_safety": {
            "last_per_stamp_glim_error_deg": asdict(canonical.stats(glim_yaw_error)),
            "last_per_stamp_legacy_global_difference_deg": asdict(
                canonical.stats(existing_yaw_difference)
            ),
            "all_records_glim_error_deg": asdict(canonical.stats(full_glim_yaw_error)),
            "all_records_legacy_global_difference_deg": asdict(
                canonical.stats(full_existing_yaw_difference)
            ),
        },
        "checks": checks,
    }


def markdown(result: dict[str, Any]) -> str:
    calibration = result["contract"]["calibration"]
    lines = [
        f"# {result['label']} startup yaw acceptance",
        "",
        f"- result: **{'PASS' if result['passed'] else 'FAIL'}**",
        f"- first global delay: {result['startup']['first_global_delay_sec']:.6f} s",
        "- first global GLIM yaw error: "
        f"{result['startup']['first_global_glim_yaw_error_deg']} deg",
        "- maximum GLIM yaw error: "
        f"{result['yaw_safety']['all_records_glim_error_deg']['maximum']:.6f} deg",
        "",
        "## Calibration provenance",
        "",
        "- explicit inclusive window: "
        f"{calibration['start_sec']:.9f} to {calibration['end_sec']:.9f} s "
        f"(duration {calibration['duration_sec']:.9f} s)",
        f"- common samples: {calibration['common_sample_count']}",
        "- frozen circular yaw offset: "
        f"{calibration['frozen_circular_yaw_offset_rad']:.12f} rad / "
        f"{calibration['frozen_circular_yaw_offset_deg']:.9f} deg",
        f"- association: {calibration['association']}",
        "- maximum interpolation gap: "
        f"{calibration['maximum_interpolation_gap_sec']:.6f} s",
        f"- timestamp source: {calibration['timestamp_source']}",
        "",
        "## Checks",
        "",
    ]
    for item in result["checks"]:
        lines.append(
            f"- {'PASS' if item['passed'] else 'FAIL'}: {item['name']} — {item['detail']}"
        )
    lines.extend(
        [
            "",
            "The legacy-derived activation is sensitivity evidence only; a production pass ",
            "requires the explicit readiness/activation diagnostic contract.",
            "",
        ]
    )
    return "\n".join(lines)


def parse() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--speed-bag", type=Path, required=True)
    parser.add_argument("--precision-bag", type=Path, required=True)
    parser.add_argument("--glim-trajectory", type=Path, required=True)
    parser.add_argument("--calibration-start", type=float, required=True)
    parser.add_argument("--calibration-end", type=float, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-markdown", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse()
    result = evaluate(args)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    args.output_markdown.write_text(markdown(result))
    print(args.output_json)
    print(args.output_markdown)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
