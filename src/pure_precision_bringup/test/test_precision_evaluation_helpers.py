#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Dependency-light contract tests for the precision evaluation helpers."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def load(name: str, relative: str):
    path = ROOT / relative
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def pose(x: float, y: float, yaw: float):
    quaternion = SimpleNamespace(
        x=0.0, y=0.0, z=math.sin(0.5 * yaw), w=math.cos(0.5 * yaw)
    )
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0), orientation=quaternion
    )


def test_full_se2_composition(validator) -> None:
    assert validator.timer_stamp_order([0, 0, 10, 10, 11]) == (True, 2)
    assert validator.timer_stamp_order([0, 10, 0]) == (False, 1)
    assert validator.timer_stamp_order([0, 11, 10]) == (False, 1)
    assert validator.timer_stamp_order([0, 0]) == (False, 2)
    # Session IDs are opaque: a numerically lower, previously unseen ID is
    # legal, but a retired session may never reappear.
    assert validator.exact_key_stream_order(
        [(9, 1, 1, 10), (9, 1, 2, 20), (3, 1, 1, 30)]
    )
    assert not validator.exact_key_stream_order(
        [(9, 1, 1, 10), (3, 1, 1, 20), (9, 2, 1, 30)]
    )
    assert not validator.exact_key_stream_order(
        [(9, 2, 1, 10), (9, 1, 1, 20)]
    )
    transform_yaw = 0.3
    raw = pose(2.0, -1.0, -0.2)
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=0.5, y=-0.25, z=0.0),
        rotation=SimpleNamespace(
            x=0.0,
            y=0.0,
            z=math.sin(0.5 * transform_yaw),
            w=math.cos(0.5 * transform_yaw),
        ),
    )
    expected_x = 0.5 + math.cos(transform_yaw) * 2.0 + math.sin(transform_yaw)
    expected_y = -0.25 + math.sin(transform_yaw) * 2.0 - math.cos(transform_yaw)
    scan = SimpleNamespace(raw_pose=SimpleNamespace(pose=raw))
    correction = SimpleNamespace(
        precision_from_raw=transform,
        corrected_pose=SimpleNamespace(
            pose=pose(expected_x, expected_y, transform_yaw - 0.2)
        ),
    )
    position_error, orientation_error = validator.correction_composition_error(
        scan, correction
    )
    assert position_error <= 1.0e-12
    assert orientation_error <= 1.0e-12


def test_existing_fusion_causal_accounting(validator) -> None:
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = covariance[35] = 0.1

    def odometry(stamp_ns: int, frame: str = "map"):
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(
                    sec=stamp_ns // 1_000_000_000,
                    nanosec=stamp_ns % 1_000_000_000,
                ),
                frame_id=frame,
            ),
            child_frame_id="base_link",
            pose=SimpleNamespace(
                pose=pose(1.0, 2.0, 0.3),
                covariance=covariance,
            ),
        )

    global_summary = validator.existing_global_prefix_accounting(
        [
            (10, odometry(1_000_000_000)),
            (20, odometry(1_000_000_000)),
            (30, odometry(2_000_000_000)),
        ]
    )
    assert global_summary["valid"], global_summary
    assert global_summary["received"] == 3
    assert global_summary["accepted"] == 2
    assert global_summary["rejected"] == 0
    assert global_summary["duplicate"] == 1
    assert not validator.existing_global_prefix_accounting(
        [(10, odometry(1_000_000_000, "wrong"))]
    )["valid"]

    health_values = [
        ("recovery.state", "tracking"),
        ("anchor_valid", "true"),
        ("recovery.position_fused", "true"),
        ("recovery.yaw_fused", "true"),
        ("last_fix_state", "good"),
    ]

    def health(record_ns: int, stamp_ns: int, values=health_values):
        return {
            "record_ns": record_ns,
            "stamp_ns": stamp_ns,
            "status_count": 1,
            "values": values,
        }

    health_summary = validator.fusion_health_prefix_accounting(
        [
            health(10, 0),
            health(20, 0),
            health(30, 1_000_000_000),
            health(40, 1_000_000_000),
            health(50, 2_000_000_000),
        ]
    )
    assert health_summary["valid"], health_summary
    assert health_summary["received"] == 5
    assert health_summary["accepted"] == 2
    assert health_summary["rejected"] == 3
    assert health_summary["zero_stamp"] == 2
    assert health_summary["duplicate_stamp"] == 1
    # Three pre-recorder zero-stamp callbacks remain exactly distinguishable:
    # they add only to received/rejected, never to accepted.
    assert 8 == health_summary["received"] + 3
    assert 6 == health_summary["rejected"] + 3
    assert not validator.fusion_health_prefix_accounting(
        [health(10, 1_000_000_000, health_values[:-1])]
    )["valid"]


def test_exact_key_protocol(evaluator) -> None:
    assert evaluator.integer_value(b"\x01") == 1
    assert evaluator.integer_value(2) == 2
    assert evaluator.outage_intervals(
        [
            (1_000_000_000, "tracking"),
            (2_000_000_000, "outage"),
            (4_000_000_000, "tracking"),
        ]
    ) == [(2_000_000_000, 4_000_000_000)]
    scans = [
        {"key": (7, 1, sequence, stamp)}
        for sequence, stamp in ((1, 1_000_000_000), (2, 2_000_000_000),
                                (3, 3_000_000_000), (4, 4_000_000_000))
    ]
    summary = evaluator.protocol_summary(
        {
            "scans": scans,
            "corrections": [scans[2]["key"], scans[3]["key"]],
        }
    )
    assert summary["duplicate_scan_keys"] == 0
    assert summary["duplicate_correction_keys"] == 0
    assert summary["unknown_correction_keys"] == 0
    assert summary["strict_physical_scan_stamps"]
    assert summary["warmup_sec"] == 2.0
    assert summary["post_warmup_correction_ratio"] == 1.0


def test_outage_freeze_and_xy_recovery(evaluator) -> None:
    anchor_values = {
        "anchor.target.x_m": "10",
        "anchor.target.y_m": "20",
        "anchor.target.yaw_rad": "0.1",
        "anchor.applied.x_m": "9",
        "anchor.applied.y_m": "19",
        "anchor.applied.yaw_rad": "0.09",
    }
    diagnostics = [
        {
            "stamp_ns": 2_000_000_000,
            "values": {
                "state": "TRACKING",
                "anchor.initialized": "true",
                "position_fused": "true",
                "anchor.source": "existing_fusion",
                "fallback.gnss_position_enabled": "false",
                **anchor_values,
            },
        },
        {
            "stamp_ns": 4_000_000_000,
            "values": {
                "state": "FROZEN",
                "anchor.initialized": "true",
                "position_fused": "true",
                **anchor_values,
            },
        },
        {
            "stamp_ns": 6_000_000_000,
            "values": {
                "state": "FROZEN",
                "anchor.initialized": "true",
                "position_fused": "true",
                **anchor_values,
            },
        },
        {
            "stamp_ns": 10_500_000_000,
            "values": {
                "state": "TRACKING",
                "anchor.initialized": "true",
                "position_fused": "true",
                "anchor.source": "existing_fusion",
                "fallback.gnss_position_enabled": "false",
                **anchor_values,
            },
        },
    ]
    q4 = lambda stamp: {
        "stamp_ns": stamp,
        "fix_quality": 4,
        "usable": True,
        "heading_valid": False,
    }
    summary = evaluator.precision_timeline_summary(
        {
            "trajectories": {
                evaluator.RAW: SimpleNamespace(
                    stamp_ns=np.asarray([1_000_000_000], dtype=np.int64)
                )
            },
            "precision_diagnostics": diagnostics,
            "gnss": [q4(1_000_000_000), q4(10_000_000_000)],
        }
    )
    assert summary["initialization_delay_sec"] == 1.0
    assert summary["final_state"] == "TRACKING"
    assert summary["final_anchor_source"] == "existing_fusion"
    assert not summary["final_fallback_gnss_position_enabled"]
    outage = summary["longest_q4_outage"]
    assert outage is not None
    assert outage["outage_diagnostic_samples"] == 2
    assert all(outage["anchor_serialization_exact"].values())
    assert all(value == 0.0 for value in outage["anchor_ranges"].values())
    assert outage["recovery_delay_sec"] == 0.5


def readiness_item(stamp: int, level: int, message: str, **overrides):
    values = {
        "activation.epoch": "0",
        "activation.stable_candidate_count": "0",
        "activation.required_candidate_count": "3",
        "activation.commit_count": "0",
        "activation.stamp_sec": "nan",
        "activation.candidate_yaw_rad": "nan",
        "activation.candidate_delta_rad": "nan",
        "activation.reason": "waiting_for_position_initialization",
        "position_initialized": "false",
        "position_fused": "false",
        "yaw_publishable": "false",
        "anchor.initialized": "false",
        "anchor.yaw_observed": "false",
        "anchor.yaw_publishable": "false",
        "global_output_ready": "false",
        "state": "UNINITIALIZED",
        "publish.global": "0",
        "anchor.correction_lag.yaw_rad": "0",
        "publish.global_suppressed_not_ready": "1",
        "publish.global_suppressed_activation_watermark": "0",
        "anchor.source": "existing_fusion",
        "fusion.health.healthy": "false",
        "fusion.health.age_sec": "nan",
        "fusion.health.status_stamp_sec": "nan",
        "fusion.health.level": "1",
        "fusion.health.recovery_state": "uninitialized",
        "fusion.health.anchor_valid": "false",
        "fusion.health.position_fused": "false",
        "fusion.health.yaw_fused": "false",
        "fusion.health.last_fix_state": "bad",
        "fusion.health.reason": "fusion_diagnostics_unavailable",
        "fusion.health.rearm_required": "false",
        "fusion.health.rearm_saw_unhealthy": "false",
        "fusion.health.rearmed": "true",
        "fusion.health.rearm_reset_stamp_sec": "nan",
        "fusion.anchor.state": "WAITING_HEALTHY",
        "fusion.anchor.frozen_residual_variance_x_m2": "0",
        "fusion.anchor.frozen_residual_variance_y_m2": "0",
        "fusion.anchor.frozen_residual_variance_yaw_rad2": "0",
        "fusion.sync.existing_global_stamp_sec": "nan",
        "fusion.sync.existing_global_age_sec": "nan",
        "fusion.sync.last_valid_stamp_sec": "nan",
        "fusion.sync.last_valid_age_sec": "nan",
        "local_correction.odom_session_resets": "0",
        "fallback.gnss_position_enabled": "false",
    }
    values.update(overrides)
    return {"stamp_ns": stamp, "level": level, "message": message, "values": values}


def test_startup_readiness_contract(validator, startup) -> None:
    timeline = [
        readiness_item(
            1_000_000_000, 1, "waiting_for_healthy_existing_fusion"
        ),
        readiness_item(
            2_000_000_000,
            1,
            "stabilizing_existing_fusion_startup",
            state="STABILIZING_STARTUP",
            **{"activation.reason": "stable_yaw_candidate_accumulating"},
        ),
        readiness_item(
            3_000_000_000,
            0,
            "tracking_existing_fusion_anchor",
            position_initialized="true",
            position_fused="true",
            yaw_publishable="true",
            **{
                "anchor.initialized": "true",
                "anchor.yaw_observed": "true",
                "anchor.yaw_publishable": "true",
                "global_output_ready": "true",
                "state": "TRACKING",
                "activation.stable_candidate_count": "3",
                "activation.commit_count": "1",
                "activation.stamp_sec": "2.99",
                "activation.candidate_yaw_rad": "-1.2",
                "activation.candidate_delta_rad": "0.001",
                "activation.reason": "existing_fusion_stable_activated",
                "fusion.health.healthy": "true",
                "fusion.health.age_sec": "0.01",
                "fusion.health.status_stamp_sec": "2.98",
                "fusion.health.level": "0",
                "fusion.health.recovery_state": "tracking",
                "fusion.health.anchor_valid": "true",
                "fusion.health.position_fused": "true",
                "fusion.health.yaw_fused": "true",
                "fusion.health.last_fix_state": "good",
                "fusion.health.reason": "strict_fusion_health_ok",
                "fusion.anchor.state": "TRACKING",
                "fusion.sync.existing_global_stamp_sec": "2.99",
                "fusion.sync.existing_global_age_sec": "0.01",
                "fusion.sync.last_valid_stamp_sec": "2.99",
                "fusion.sync.last_valid_age_sec": "0.01",
            },
        ),
    ]
    valid, transitions, errors = validator.startup_transition_contract(timeline)
    assert valid, errors
    assert len(transitions) == 1
    summary = startup.diagnostic_transition_contract(timeline)
    assert summary["valid"], summary["errors"]
    broken = [dict(item) for item in timeline]
    broken[-1] = {
        **broken[-1],
        "values": {**broken[-1]["values"], "global_output_ready": "false"},
    }
    assert not validator.startup_transition_contract(broken)[0]
    assert not startup.diagnostic_transition_contract(broken)["valid"]

    frozen_values = {
        **timeline[-1]["values"],
        "state": "FROZEN",
        "fusion.anchor.state": "FROZEN",
        "fusion.health.healthy": "false",
        "fusion.health.level": "1",
        "fusion.health.recovery_state": "outage",
        "fusion.health.reason": "fusion_level_not_ok",
        "anchor.target.x_m": "10.0",
        "anchor.target.y_m": "20.0",
        "anchor.target.yaw_rad": "-1.2",
        "anchor.applied.x_m": "10.0",
        "anchor.applied.y_m": "20.0",
        "anchor.applied.yaw_rad": "-1.2",
    }
    frozen_timeline = timeline + [
        {
            "stamp_ns": 4_000_000_000,
            "level": 1,
            "message": "fusion_unhealthy_anchor_frozen",
            "values": frozen_values,
        },
        {
            "stamp_ns": 5_000_000_000,
            "level": 1,
            "message": "fusion_unhealthy_anchor_frozen",
            "values": frozen_values,
        },
    ]
    freeze_valid, groups, freeze_errors = validator.fusion_anchor_freeze_contract(
        frozen_timeline
    )
    assert freeze_valid, freeze_errors
    assert groups[-1]["samples"] == 2
    broken_freeze = [dict(item) for item in frozen_timeline]
    broken_freeze[-1] = {
        **broken_freeze[-1],
        "values": {
            **broken_freeze[-1]["values"],
            "anchor.applied.yaw_rad": "-1.1",
        },
    }
    assert not validator.fusion_anchor_freeze_contract(broken_freeze)[0]

    no_reset_valid, no_reset_summary, no_reset_errors = (
        validator.fusion_rearm_contract(timeline)
    )
    assert no_reset_valid, (no_reset_summary, no_reset_errors)

    reset_base = {
        **timeline[-1]["values"],
        "global_output_ready": "false",
        "position_initialized": "false",
        "yaw_publishable": "false",
        "anchor.yaw_publishable": "false",
        "state": "WAITING_HEALTHY",
        "local_correction.odom_session_resets": "1",
        "fusion.health.rearm_required": "true",
        "fusion.health.rearm_saw_unhealthy": "false",
        "fusion.health.rearmed": "false",
        "fusion.health.rearm_reset_stamp_sec": "3.5",
        "fusion.health.healthy": "false",
        "fusion.health.status_stamp_sec": "nan",
        "fusion.health.age_sec": "nan",
        "fusion.health.reason": "fusion_diagnostics_unavailable",
    }
    reset_wait = {
        "stamp_ns": 4_000_000_000,
        "level": 1,
        "message": "waiting_for_healthy_existing_fusion",
        "values": reset_base,
    }
    explicit_unhealthy = {
        "stamp_ns": 5_000_000_000,
        "level": 1,
        "message": "waiting_for_healthy_existing_fusion",
        "values": {
            **reset_base,
            "fusion.health.rearm_saw_unhealthy": "true",
            "fusion.health.status_stamp_sec": "4.9",
            "fusion.health.age_sec": "0.1",
            "fusion.health.level": "1",
            "fusion.health.recovery_state": "outage",
            "fusion.health.anchor_valid": "false",
            "fusion.health.position_fused": "false",
            "fusion.health.yaw_fused": "false",
            "fusion.health.last_fix_state": "bad",
            "fusion.health.reason": "fusion_level_not_ok",
        },
    }
    strict_rearmed = {
        "stamp_ns": 6_000_000_000,
        "level": 1,
        "message": "stabilizing_existing_fusion_startup",
        "values": {
            **explicit_unhealthy["values"],
            "fusion.health.rearm_required": "false",
            "fusion.health.rearmed": "true",
            "fusion.health.status_stamp_sec": "5.9",
            "fusion.health.age_sec": "0.1",
            "fusion.health.level": "0",
            "fusion.health.recovery_state": "tracking",
            "fusion.health.anchor_valid": "true",
            "fusion.health.position_fused": "true",
            "fusion.health.yaw_fused": "true",
            "fusion.health.last_fix_state": "good",
            "fusion.health.healthy": "true",
            "fusion.health.reason": "strict_fusion_health_ok",
        },
    }
    rearm_valid, rearm_summary, rearm_errors = validator.fusion_rearm_contract(
        [reset_wait, explicit_unhealthy, strict_rearmed]
    )
    assert rearm_valid, (rearm_summary, rearm_errors)
    stale_only = {
        **explicit_unhealthy,
        "values": {
            **reset_base,
            "fusion.health.rearm_saw_unhealthy": "true",
            "fusion.health.status_stamp_sec": "3.0",
            "fusion.health.age_sec": "2.0",
            "fusion.health.reason": "fusion_diagnostics_stale",
        },
    }
    assert not validator.fusion_rearm_contract([reset_wait, stale_only])[0]

    activation_raw, next_raw = validator.activation_raw_successor(
        [1_000_000_000, 1_050_000_000, 1_100_000_000],
        1_001_000_000,
        20_000_000,
    )
    assert activation_raw == 1_000_000_000
    assert next_raw == 1_050_000_000
    assert startup.activation_raw_successor(
        [1_000_000_000, 1_050_000_000], 1_001_000_000, 20_000_000
    ) == (1_000_000_000, 1_050_000_000)


def test_accepted_scan_statistics(accepted_scan) -> None:
    values = np.asarray([0.0, 0.0, 0.0])
    result = accepted_scan.stats(values)
    assert result["count"] == 3
    assert result["rmse"] == 0.0
    assert accepted_scan.wrap(2.0 * math.pi) == 0.0


def test_accepted_scan_phase_contract(accepted_scan) -> None:
    def scan(sequence: int, stamp_ns: int, x: float = 0.0):
        return accepted_scan.Scan(
            stamp_ns=stamp_ns,
            session=17,
            generation=1,
            sequence=sequence,
            x=x,
            y=0.0,
            yaw=0.01 * x,
        )

    scans = [
        scan(1, 1_000_000_000, 0.0),
        scan(5, 1_200_000_000, 0.2),
        scan(10, 1_450_000_000, 0.45),
    ]
    run = accepted_scan.Run(
        scans=scans,
        final_accepted_sequence=12,
        diagnostic_snapshot_count=3,
    )
    summary = accepted_scan.snapshot_policy_summary(run, 5)
    assert summary["valid"]
    assert summary["expected"] == 3
    bad_run = accepted_scan.Run(
        scans=[scans[0], scans[2]],
        final_accepted_sequence=12,
        diagnostic_snapshot_count=2,
    )
    assert not accepted_scan.snapshot_policy_summary(bad_run, 5)["valid"]

    targets = [scan(1, 1_100_000_000)]
    xy, yaw, valid = accepted_scan.interpolate_scans(scans, targets, 0.30)
    assert valid.tolist() == [True]
    assert np.allclose(xy[0], [0.1, 0.0])
    assert math.isclose(yaw[0], 0.001)
    assert math.isclose(
        accepted_scan.infer_raw_frame_period(scans, scans), 0.05
    )
    fixed_phase = accepted_scan.phase_stability_summary(
        np.full(200, 0.20), 0.05
    )
    assert fixed_phase["valid"]
    assert fixed_phase["absolute_max_frames"] == 4.0
    assert fixed_phase["robust_span_frames"] == 0.0
    assert fixed_phase["early_late_drift_frames"] == 0.0
    accumulating_phase = accepted_scan.phase_stability_summary(
        np.linspace(0.0, 0.25, 200), 0.05
    )
    assert accumulating_phase["robust_span_frames"] > 2.0
    assert accumulating_phase["early_late_drift_frames"] > 2.0
    off_period = accepted_scan.phase_stability_summary(
        np.asarray([0.006, 0.056, 0.106]), 0.05
    )
    assert off_period["period_residual_max_sec"] > 0.005
    edge_targets = [
        scan(0, 900_000_000),
        scan(15, 1_600_000_000),
    ]
    edge_xy, edge_yaw, edge_valid = accepted_scan.interpolate_scans(
        scans, edge_targets, 0.30
    )
    assert edge_xy.shape == (2, 2)
    assert edge_yaw.shape == (2,)
    assert edge_valid.shape == (2,)
    assert edge_valid.tolist() == [False, False]

    control_run = accepted_scan.Run(
        scans=scans,
        final_accepted_sequence=10,
        diagnostic_snapshot_count=3,
    )
    shifted = [
        accepted_scan.Scan(
            stamp_ns=item.stamp_ns,
            session=99,
            generation=item.generation,
            sequence=item.sequence,
            x=item.x + 1.0,
            y=item.y,
            yaw=item.yaw,
        )
        for item in scans
    ]
    precision_run = accepted_scan.Run(
        scans=shifted,
        final_accepted_sequence=10,
        diagnostic_snapshot_count=3,
    )
    original_read = accepted_scan.read
    original_resolve = accepted_scan.resolve
    try:
        accepted_scan.read = lambda path: (
            control_run if str(path) == "control" else precision_run
        )
        accepted_scan.resolve = lambda path: path
        result = accepted_scan.evaluate(
            SimpleNamespace(
                control_bag=Path("control"),
                precision_bag=Path("precision"),
                snapshot_interval=5,
                maximum_interpolation_gap=0.30,
            )
        )
    finally:
        accepted_scan.read = original_read
        accepted_scan.resolve = original_resolve
    xy_check = next(
        item for item in result["checks"]
        if item["name"] == "accepted-scan raw XY non-intrusion"
    )
    absolute_phase_check = next(
        item for item in result["checks"]
        if item["name"].startswith("absolute same-sequence stamp phase")
    )
    stable_phase_check = next(
        item for item in result["checks"]
        if item["name"] == "same-sequence phase is stable and non-accumulating"
    )
    interpolated_increment_check = next(
        item for item in result["checks"]
        if item["name"]
        == "phase-compensated accepted-scan increment non-intrusion"
    )
    assert xy_check["category"] == "hard"
    assert not xy_check["passed"]
    assert absolute_phase_check["category"] == "warn"
    assert stable_phase_check["category"] == "hard"
    assert stable_phase_check["passed"]
    assert interpolated_increment_check["category"] == "hard"
    assert interpolated_increment_check["passed"]
    assert not result["passed"]
    assert not result["method"]["timer_interpolation"].startswith("none")

    sequences = [1, *range(5, 1000, 5)]
    stable_control_scans = [
        scan(
            sequence,
            1_000_000_000 + (sequence - 1) * 50_000_000,
        )
        for sequence in sequences
    ]
    accumulating_precision_scans = [
        accepted_scan.Scan(
            stamp_ns=item.stamp_ns + (index // 40) * 50_000_000,
            session=99,
            generation=item.generation,
            sequence=item.sequence,
            x=item.x,
            y=item.y,
            yaw=item.yaw,
        )
        for index, item in enumerate(stable_control_scans)
    ]
    stable_control_run = accepted_scan.Run(
        scans=stable_control_scans,
        final_accepted_sequence=999,
        diagnostic_snapshot_count=len(stable_control_scans),
    )
    accumulating_precision_run = accepted_scan.Run(
        scans=accumulating_precision_scans,
        final_accepted_sequence=999,
        diagnostic_snapshot_count=len(accumulating_precision_scans),
    )
    try:
        accepted_scan.read = lambda path: (
            stable_control_run
            if str(path) == "control"
            else accumulating_precision_run
        )
        accepted_scan.resolve = lambda path: path
        accumulating_result = accepted_scan.evaluate(
            SimpleNamespace(
                control_bag=Path("control"),
                precision_bag=Path("precision"),
                snapshot_interval=5,
                maximum_interpolation_gap=0.30,
            )
        )
    finally:
        accepted_scan.read = original_read
        accepted_scan.resolve = original_resolve
    accumulating_phase_check = next(
        item for item in accumulating_result["checks"]
        if item["name"] == "same-sequence phase is stable and non-accumulating"
    )
    assert accumulating_phase_check["category"] == "hard"
    assert not accumulating_phase_check["passed"]
    assert not accumulating_result["passed"]


def main() -> None:
    validator = load("precision_validator_under_test", "scripts/validate_precision_bag.py")
    evaluator = load("precision_evaluator_under_test", "scripts/evaluate_precision_glim_ab.py")
    startup = load(
        "precision_startup_evaluator_under_test",
        "scripts/evaluate_startup_acceptance.py",
    )
    accepted_scan = load(
        "precision_accepted_scan_evaluator_under_test",
        "scripts/evaluate_accepted_scan_nonintrusion.py",
    )
    test_full_se2_composition(validator)
    test_existing_fusion_causal_accounting(validator)
    test_exact_key_protocol(evaluator)
    test_outage_freeze_and_xy_recovery(evaluator)
    test_startup_readiness_contract(validator, startup)
    test_accepted_scan_statistics(accepted_scan)
    test_accepted_scan_phase_contract(accepted_scan)
    print("precision evaluation helper tests PASS")


if __name__ == "__main__":
    main()
