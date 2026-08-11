#!/usr/bin/env python3
"""Read-only startup safety acceptance prototype for isolated precision bags.

This prototype intentionally supports the pre-fix bags.  Missing readiness
keys are a hard failure; a separately labelled legacy-derived activation is
reported only to prove that the proposed gates detect the prior unsafe output.
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
ACTIVATION_SERIALIZATION_TOLERANCE_NS = 20_000_000


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
    """Resolve a rounded diagnostic activation stamp and its next raw sample."""
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


def diagnostic_activation(diagnostics: list[dict[str, Any]]) -> dict[str, Any]:
    required = {
        "position_initialized",
        "position_fused",
        "yaw_publishable",
        "global_output_ready",
        "anchor.initialized",
        "anchor.yaw_observed",
        "anchor.yaw_publishable",
        "activation.stamp_sec",
        "activation.stable_candidate_count",
        "activation.required_candidate_count",
        "activation.reason",
        "activation.epoch",
        "activation.commit_count",
        "activation.candidate_yaw_rad",
        "activation.candidate_delta_rad",
        "anchor.correction_lag.yaw_rad",
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
        "fusion.anchor.frozen_residual_variance_x_m2",
        "fusion.anchor.frozen_residual_variance_y_m2",
        "fusion.anchor.frozen_residual_variance_yaw_rad2",
        "fusion.sync.existing_global_stamp_sec",
        "fusion.sync.existing_global_age_sec",
        "fusion.sync.last_valid_stamp_sec",
        "fusion.sync.last_valid_age_sec",
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
        value = float(activation["values"].get("activation.stamp_sec", "nan"))
        if math.isfinite(value) and value > 0.0:
            activation_ns = int(round(value * 1.0e9))
    return {
        "required_keys": sorted(required),
        "missing_keys": missing,
        "incomplete_samples": incomplete_samples,
        "first_ready_diagnostic": activation,
        "activation_ns": activation_ns,
    }


def diagnostic_transition_contract(diagnostics: list[dict[str, Any]]) -> dict[str, Any]:
    """Validate readiness as a monotonic, internally exact per-epoch state."""
    errors: list[str] = []
    by_epoch: dict[int, list[dict[str, Any]]] = {}
    transitions: list[dict[str, Any]] = []
    for item in diagnostics:
        values = item["values"]
        try:
            epoch = int(values["activation.epoch"])
        except (KeyError, ValueError):
            continue
        position = true_value(values.get("position_initialized", "false"))
        position_fused = true_value(values.get("position_fused", "false"))
        yaw = true_value(values.get("yaw_publishable", "false"))
        anchor_initialized = true_value(values.get("anchor.initialized", "false"))
        anchor_yaw_observed = true_value(
            values.get("anchor.yaw_observed", "false")
        )
        anchor_yaw = true_value(values.get("anchor.yaw_publishable", "false"))
        ready = true_value(values.get("global_output_ready", "false"))
        state = values.get("state", "")
        if not (
            ready == position == position_fused == yaw == anchor_initialized
            == anchor_yaw_observed == anchor_yaw
        ):
            errors.append(f"logical readiness mismatch at {item['stamp_ns']}")
        if not ready:
            if int(item["level"]) != 1:
                errors.append(f"pre-ready state is not WARN at {item['stamp_ns']}")
            if state in ("TRACKING_SE2", "TRACKING"):
                errors.append(f"tracking state before readiness at {item['stamp_ns']}")
            allowed_messages = (
                "waiting_for_first_usable_gnss_position",
                "waiting_for_stable_absolute_yaw",
                "gnss_outage_before_yaw_activation",
                "waiting_for_healthy_existing_fusion",
                "stabilizing_existing_fusion_startup",
            )
            if item.get("message") not in allowed_messages:
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
        by_epoch.setdefault(epoch, []).append(item)

    previous_epoch: int | None = None
    for epoch, items in sorted(by_epoch.items()):
        if previous_epoch is not None and epoch != previous_epoch + 1:
            errors.append(f"activation epoch jump {previous_epoch}->{epoch}")
        previous_epoch = epoch
        ready_values = [
            true_value(item["values"].get("global_output_ready", "false"))
            for item in items
        ]
        if not ready_values or ready_values[0]:
            errors.append(f"epoch {epoch} lacks an observed pre-ready diagnostic")
            continue
        transition_indices = [
            index for index in range(1, len(ready_values))
            if ready_values[index] != ready_values[index - 1]
        ]
        if len(transition_indices) != 1 or not ready_values[-1]:
            errors.append(
                f"epoch {epoch} readiness transitions={transition_indices} final={ready_values[-1]}"
            )
            continue
        index = transition_indices[0]
        first_ready = items[index]
        first_values = first_ready["values"]
        try:
            stable = int(first_values["activation.stable_candidate_count"])
            required = int(first_values["activation.required_candidate_count"])
            commit = int(first_values["activation.commit_count"])
            pre_commit = int(items[index - 1]["values"]["activation.commit_count"])
            candidate_yaw = float(first_values["activation.candidate_yaw_rad"])
            candidate_delta = float(first_values["activation.candidate_delta_rad"])
            health_age = float(first_values["fusion.health.age_sec"])
            existing_global_stamp = float(
                first_values["fusion.sync.existing_global_stamp_sec"]
            )
            existing_global_age = float(
                first_values["fusion.sync.existing_global_age_sec"]
            )
            last_valid_stamp = float(
                first_values["fusion.sync.last_valid_stamp_sec"]
            )
            last_valid_age = float(first_values["fusion.sync.last_valid_age_sec"])
        except (KeyError, ValueError):
            errors.append(f"epoch {epoch} activation counters malformed")
            continue
        if (
            required != 3
            or stable < required
            or commit != pre_commit + 1
            or not math.isfinite(candidate_yaw)
            or not math.isfinite(candidate_delta)
            or not 0.0 <= candidate_delta <= 0.08
        ):
            errors.append(
                f"epoch {epoch} activation counters stable={stable} required={required} "
                f"commit={pre_commit}->{commit} delta={candidate_delta}"
            )
        if first_values.get("activation.reason") not in (
            "stable_yaw_activated",
            "existing_fusion_stable_activated",
        ):
            errors.append(
                f"epoch {epoch} activation reason={first_values.get('activation.reason')!r}"
            )
        strict_health = (
            first_values.get("anchor.source") == "existing_fusion"
            and first_values.get("fallback.gnss_position_enabled") == "false"
            and first_values.get("fusion.health.healthy") == "true"
            and first_values.get("fusion.health.level") == "0"
            and first_values.get("fusion.health.recovery_state") == "tracking"
            and first_values.get("fusion.health.anchor_valid") == "true"
            and first_values.get("fusion.health.position_fused") == "true"
            and first_values.get("fusion.health.yaw_fused") == "true"
            and first_values.get("fusion.health.last_fix_state") == "good"
            and first_values.get("fusion.health.reason")
            == "strict_fusion_health_ok"
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
            and first_values.get("fusion.health.rearm_required") == "false"
        )
        if not strict_health:
            errors.append(
                f"epoch {epoch} activation lacks strict existing-fusion health "
                f"health_age={health_age} existing_age={existing_global_age} "
                f"last_valid_age={last_valid_age}"
            )
        pre_ready_publish = {
            item["values"].get("publish.global") for item in items[:index]
        }
        if len(pre_ready_publish) != 1:
            errors.append(f"epoch {epoch} publish.global changed before readiness")
        transitions.append(
            {
                "epoch": epoch,
                "first_ready_diagnostic_stamp_ns": int(first_ready["stamp_ns"]),
                "activation_stamp_sec": first_values.get("activation.stamp_sec"),
                "stable_candidate_count": stable,
                "required_candidate_count": required,
                "commit_count": commit,
            }
        )
    return {
        "valid": bool(by_epoch) and not errors,
        "epochs": sorted(by_epoch),
        "transitions": transitions,
        "errors": errors[:20],
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
    return canonical.common_samples(reference, trajectories, 0.1)


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
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
    reference = canonical.read_glim_trajectory(args.glim_trajectory)

    calibration_reference, calibration = common_yaw_series(
        canonical,
        reference,
        {"speed_existing": speed["trajectories"][EXISTING]},
    )
    calibration_mask = (
        (calibration_reference.stamp_ns >= int(round(args.calibration_start * 1.0e9)))
        & (calibration_reference.stamp_ns <= int(round(args.calibration_end * 1.0e9)))
    )
    if np.count_nonzero(calibration_mask) < 3:
        raise RuntimeError("calibration window has fewer than three common samples")
    yaw_offset = repo_eval.circular_offset(
        calibration["speed_existing"].yaw, calibration_reference.yaw, calibration_mask
    )

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
        precision["precision_diagnostics"]
    )
    activation_ns = activation["activation_ns"]
    activation_raw_ns, expected_first_global_ns = (
        (None, None)
        if activation_ns is None
        else activation_raw_successor(
            raw.stamp_ns,
            activation_ns,
            ACTIVATION_SERIALIZATION_TOLERANCE_NS,
        )
    )
    legacy_activation = legacy_derived_activation(precision["precision_diagnostics"])
    freeze_valid, freeze_groups, freeze_errors = validator.fusion_anchor_freeze_contract(
        precision["precision_diagnostics"]
    )
    rearm_valid, rearm_summary, rearm_errors = validator.fusion_rearm_contract(
        precision["precision_diagnostics"]
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
    activation_lower_bound_ns = (
        None if activation_ns is None else
        activation_ns - ACTIVATION_SERIALIZATION_TOLERANCE_NS
    )
    before_ready_odom = None if activation_lower_bound_ns is None else sum(
        stamp < activation_lower_bound_ns for stamp in global_stamps
    )
    before_ready_pose = None if activation_lower_bound_ns is None else sum(
        stamp < activation_lower_bound_ns for stamp in global_pose_stamps
    )
    first_ready_diag = activation["first_ready_diagnostic"]
    first_lag = None
    stable_count = None
    required_count = None
    activation_reason = None
    candidate_yaw = None
    first_output_anchor_yaw_error = None
    first_ready_existing_fusion_authority = False
    first_ready_strict_fusion_health = False
    first_ready_freshness: dict[str, float | None] = {
        "health_age_sec": None,
        "existing_global_age_sec": None,
        "last_valid_age_sec": None,
    }
    if first_ready_diag is not None:
        values = first_ready_diag["values"]
        first_lag = abs(float(values.get("anchor.correction_lag.yaw_rad", "nan")))
        stable_count = int(values.get("activation.stable_candidate_count", "-1"))
        required_count = int(values.get("activation.required_candidate_count", "-1"))
        activation_reason = values.get("activation.reason")
        candidate_yaw = float(values.get("activation.candidate_yaw_rad", "nan"))
        first_ready_existing_fusion_authority = (
            values.get("anchor.source") == "existing_fusion"
            and not true_value(values.get("fallback.gnss_position_enabled", "true"))
        )
        health_age = float(values.get("fusion.health.age_sec", "nan"))
        existing_global_stamp = float(
            values.get("fusion.sync.existing_global_stamp_sec", "nan")
        )
        existing_global_age = float(
            values.get("fusion.sync.existing_global_age_sec", "nan")
        )
        last_valid_stamp = float(
            values.get("fusion.sync.last_valid_stamp_sec", "nan")
        )
        last_valid_age = float(
            values.get("fusion.sync.last_valid_age_sec", "nan")
        )
        first_ready_freshness = {
            "health_age_sec": health_age,
            "existing_global_age_sec": existing_global_age,
            "last_valid_age_sec": last_valid_age,
        }
        first_ready_strict_fusion_health = (
            true_value(values.get("fusion.health.healthy", "false"))
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
        "one finite activation event exists",
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
        f"first_global={first_global_ns} activation={activation_ns} "
        f"serialization_tolerance_ns={ACTIVATION_SERIALIZATION_TOLERANCE_NS}",
    )
    add(
        "first global output is exactly the next raw sample after activation",
        activation_raw_ns is not None
        and expected_first_global_ns is not None
        and first_global_ns == expected_first_global_ns
        and first_global_pose_ns == expected_first_global_ns,
        f"serialized_activation={activation_ns} activation_raw={activation_raw_ns} "
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
        "activation uses required consecutive stable candidates",
        stable_count is not None and required_count == 3
        and stable_count >= required_count
        and activation_reason in (
            "stable_yaw_activated", "existing_fusion_stable_activated"
        ),
        f"stable={stable_count} required={required_count} reason={activation_reason}",
    )
    add(
        "global anchor authority is existing fusion only",
        first_ready_existing_fusion_authority and authority_invariant,
        f"first_ready_authority={first_ready_existing_fusion_authority} "
        f"all_samples_authority={authority_invariant}",
    )
    add(
        "activation occurs under strict existing-fusion health",
        first_ready_strict_fusion_health,
        f"first_ready_strict_health={first_ready_strict_fusion_health} "
        f"freshness={first_ready_freshness}",
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
        "first-ready anchor yaw lag is at most 0.02 rad",
        first_lag is not None and math.isfinite(first_lag) and first_lag <= 0.02,
        f"lag={first_lag}",
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
        "contract": {
            "readiness": "position_initialized && yaw_publishable",
            "candidate_count": 3,
            "candidate_delta_max_rad": 0.08,
            "first_ready_anchor_lag_max_rad": 0.02,
            "activation_stamp_serialization_tolerance_sec": 0.02,
            "first_publish_rule": "exactly the next unique raw stamp after activation",
            "first_global_delay_max_sec": 25.0,
            "absolute_yaw_safety_max_deg": 10.0,
            "first_legacy_global_yaw_difference_max_deg": 3.0,
            "alignment": "speed legacy-global calibration yaw offset frozen for GLIM",
            "authority": "existing fusion only; position-only GNSS fallback disabled",
            "session_rearm": (
                "fresh explicit unhealthy/non-TRACKING status, then fresh strict TRACKING; "
                "stale/unavailable alone never qualifies"
            ),
        },
        "inputs": {
            "speed_bag": str(args.speed_bag.resolve()),
            "precision_bag": str(args.precision_bag.resolve()),
            "glim": str(args.glim_trajectory.resolve()),
        },
        "counts": {
            "precision_global_odom_positive_records": len(global_stamps),
            "precision_global_pose_positive_records": len(global_pose_stamps),
        },
        "startup": {
            "first_raw_ns": first_raw_ns,
            "first_global_ns": first_global_ns,
            "first_global_pose_ns": first_global_pose_ns,
            "activation_raw_ns": activation_raw_ns,
            "expected_first_global_ns": expected_first_global_ns,
            "first_global_delay_sec": (first_global_ns - first_raw_ns) * 1.0e-9,
            "activation": activation,
            "diagnostic_transition_contract": transition_contract,
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
