#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Dependency-light contract tests for the retained precision bag validator."""

from __future__ import annotations

import importlib.util
import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace


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
    causal_endpoint = validator.existing_global_causal_endpoint_accounting(
        [
            (10, odometry(1_000_000_000)),
            (20, odometry(2_000_000_000)),
            (30, odometry(2_000_000_000)),
            (40, odometry(2_000_000_000)),
            (50, odometry(3_000_000_000)),
        ],
        2_000_000_000,
    )
    assert causal_endpoint["valid"], causal_endpoint
    assert causal_endpoint["lower"]["received"] == 2
    assert causal_endpoint["upper"]["received"] == 4
    assert causal_endpoint["lower"]["accepted"] == 2
    assert causal_endpoint["upper"]["accepted"] == 2
    assert causal_endpoint["duplicate_tail_allowance"] == 2
    assert not validator.existing_global_causal_endpoint_accounting(
        [(10, odometry(1_000_000_000))], 2_000_000_000
    )["valid"]
    assert not validator.existing_global_causal_endpoint_accounting(
        [(10, odometry(2_000_000_000))], 2.0
    )["valid"]

    def authority(
        record_ns: int, stamp_ns: int, session: int, sequence: int,
        state: int = 1,
    ):
        soft_hold = state == 2
        return {
            "record_ns": record_ns,
            "stamp_ns": stamp_ns,
            "stamp_canonical": True,
            "source_stamp_ns": stamp_ns,
            "source_stamp_canonical": True,
            "frame_id": "map",
            "session_id": session,
            "sequence": sequence,
            "state": state,
            "reason": (
                "gnss_soft_bad_within_grace" if soft_hold
                else "strict_full_se2_authority_ok"
            ),
            "recovery_state": "tracking",
            "anchor_valid": True,
            "position_fused": True,
            "yaw_fused": True,
            "last_fix_state": 2 if soft_hold else 1,
        }

    health_summary = validator.fusion_authority_prefix_accounting(
        [
            authority(10, 1_000_000_000, 10, 1),
            authority(20, 1_000_000_000, 10, 2, state=2),
            authority(30, 2_000_000_000, 11, 1),
        ]
    )
    assert health_summary["valid"], health_summary
    assert health_summary["received"] == 3
    assert health_summary["accepted"] == 3
    assert health_summary["rejected"] == 0
    assert health_summary["soft_bad_hold_count"] == 1

    duplicate = validator.fusion_authority_prefix_accounting(
        [
            authority(10, 1_000_000_000, 10, 1),
            authority(20, 1_100_000_000, 10, 1),
        ]
    )
    assert not duplicate["valid"]
    assert duplicate["accepted"] == 1 and duplicate["rejected"] == 1
    assert not validator.fusion_authority_prefix_accounting(
        [
            authority(10, 1_000_000_000, 10, 1),
            authority(20, 2_000_000_000, 11, 1),
            authority(30, 3_000_000_000, 10, 2),
        ]
    )["valid"]
    assert not validator.fusion_authority_prefix_accounting(
        [
            authority(10, 1_000_000_000, 10, 1),
            authority(20, 1_000_000_000, 11, 1),
        ]
    )["valid"]
    bad_payload = authority(10, 1_000_000_000, 10, 1)
    bad_payload["yaw_fused"] = False
    assert not validator.fusion_authority_prefix_accounting([bad_payload])["valid"]


def test_map_fusion_publication_integrity_contract(validator) -> None:
    keys = validator.MAP_FUSION_PUBLICATION_COUNTER_KEYS

    def counter_values(
        strict_drop: int = 0,
        covered: int = 0,
        wall_timer: int = 0,
        total: int | None = None,
    ):
        total = (
            strict_drop + covered + wall_timer if total is None else total
        )
        return [
            ("unrelated.health", "ok"),
            (keys[0], str(strict_drop)),
            (keys[1], str(covered)),
            (keys[2], str(wall_timer)),
            (keys[3], str(total)),
        ]

    def sample(
        record_ns: int,
        covered: int = 0,
        wall_timer: int = 0,
    ):
        return {
            "record_ns": record_ns,
            "stamp_ns": record_ns,
            "status_count": 1,
            "values": counter_values(
                covered=covered, wall_timer=wall_timer
            ),
        }

    timeline = [
        sample(10),
        sample(20, covered=2, wall_timer=1),
        sample(30, covered=2, wall_timer=4),
    ]
    valid, summary, errors = (
        validator.map_fusion_publication_integrity_contract(timeline)
    )
    assert valid, errors
    assert summary["samples"] == 3
    assert summary["schema_samples"] == 3
    assert summary["strict_drop_count"] == 0
    assert summary["coalesced_report_only"] == {
        "covered_odometry": 2,
        "wall_timer": 4,
    }
    assert summary["final_counters"][keys[3]] == 6

    assert not validator.map_fusion_publication_integrity_contract([])[0]

    missing = copy.deepcopy(timeline)
    missing[1]["values"] = [
        pair for pair in missing[1]["values"] if pair[0] != keys[1]
    ]
    assert not validator.map_fusion_publication_integrity_contract(missing)[0]

    duplicate = copy.deepcopy(timeline)
    duplicate[1]["values"].append((keys[1], "2"))
    assert not validator.map_fusion_publication_integrity_contract(duplicate)[0]

    for malformed_value in ("1.0", "-1", "01", " 1", "nan", ""):
        malformed = copy.deepcopy(timeline)
        malformed[1]["values"] = [
            (key, malformed_value if key == keys[1] else value)
            for key, value in malformed[1]["values"]
        ]
        assert not validator.map_fusion_publication_integrity_contract(
            malformed
        )[0]

    non_string = copy.deepcopy(timeline)
    non_string[1]["values"] = [
        (key, 2 if key == keys[1] else value)
        for key, value in non_string[1]["values"]
    ]
    assert not validator.map_fusion_publication_integrity_contract(non_string)[0]

    negative = copy.deepcopy(timeline)
    negative[1]["values"] = [
        (key, "-1" if key == keys[2] else value)
        for key, value in negative[1]["values"]
    ]
    assert not validator.map_fusion_publication_integrity_contract(negative)[0]

    backstep = copy.deepcopy(timeline)
    backstep[2]["values"] = counter_values(covered=1, wall_timer=4)
    assert not validator.map_fusion_publication_integrity_contract(backstep)[0]

    sum_mismatch = copy.deepcopy(timeline)
    sum_mismatch[2]["values"] = counter_values(
        covered=2, wall_timer=4, total=7
    )
    assert not validator.map_fusion_publication_integrity_contract(sum_mismatch)[0]

    strict_drop = copy.deepcopy(timeline)
    strict_drop[2]["values"] = counter_values(
        strict_drop=1, covered=2, wall_timer=4
    )
    assert not validator.map_fusion_publication_integrity_contract(strict_drop)[0]

    ambiguous_status = copy.deepcopy(timeline)
    ambiguous_status[1]["status_count"] = 2
    assert not validator.map_fusion_publication_integrity_contract(
        ambiguous_status
    )[0]


def test_map_fusion_exact_raw_stamp_coverage(validator) -> None:
    def odometry(physical_stamp_ns: int):
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(
                    sec=physical_stamp_ns // 1_000_000_000,
                    nanosec=physical_stamp_ns % 1_000_000_000,
                )
            )
        )

    raw_records = [
        (5, odometry(0)),
        (10, odometry(100)),       # startup: before first fused output
        (20, odometry(200)),
        (30, odometry(300)),
        (31, odometry(300)),       # duplicate callback is covered by one output
        (40, odometry(400)),
        (50, odometry(500)),       # open stamp tail
        (70, odometry(350)),       # after the final diagnostic record
    ]
    existing_records = [
        (15, odometry(0)),
        (25, odometry(200)),
        (35, odometry(250)),       # wall-timer-only output
        (45, odometry(300)),
        (55, odometry(400)),
        (70, odometry(600)),       # output recorder tail
    ]

    valid, summary, errors = validator.exact_raw_to_existing_stamp_coverage(
        raw_records, existing_records, 60
    )
    assert valid, errors
    assert summary["first_existing_stamp_ns"] == 200
    assert summary["last_existing_stamp_ns"] == 400
    assert summary["eligible_raw_records"] == 4
    assert summary["eligible_unique_raw_stamps"] == 3
    assert summary["duplicate_eligible_raw_records"] == 1
    assert summary["matched_unique_raw_stamps"] == 3
    assert summary["missing_unique_raw_stamps"] == 0
    assert summary["coverage_ratio"] == 1.0
    assert summary["startup_excluded_raw_records"] == 1
    assert summary["zero_stamp_excluded_raw_records"] == 1
    assert summary["stamp_tail_excluded_raw_records"] == 1
    assert summary["record_tail_excluded_raw_records"] == 1
    assert summary["record_tail_excluded_existing_records"] == 1

    # Exact means exact: a nearby timer output cannot stand in for stamp 300.
    missing_exact = [
        item for item in existing_records
        if validator.stamp_ns(item[1].header.stamp) != 300
    ]
    missing_valid, missing_summary, missing_errors = (
        validator.exact_raw_to_existing_stamp_coverage(
            raw_records, missing_exact, 60
        )
    )
    assert not missing_valid
    assert missing_summary["missing_examples_ns"] == [300]
    assert any("lack an exact" in error for error in missing_errors)

    # A shorter diagnostic prefix excludes not-yet-closed raw stamp tails.
    prefix_valid, prefix_summary, prefix_errors = (
        validator.exact_raw_to_existing_stamp_coverage(
            raw_records, existing_records, 35
        )
    )
    assert prefix_valid, prefix_errors
    assert prefix_summary["last_existing_stamp_ns"] == 250
    assert prefix_summary["eligible_unique_raw_stamps"] == 1
    assert prefix_summary["stamp_tail_excluded_raw_records"] == 2

    no_eligible = [(10, odometry(100))]
    assert not validator.exact_raw_to_existing_stamp_coverage(
        no_eligible, [(20, odometry(200))], 30
    )[0]
    assert not validator.exact_raw_to_existing_stamp_coverage(
        raw_records, existing_records, None
    )[0]
    assert not validator.exact_raw_to_existing_stamp_coverage(
        raw_records, [(20, odometry(0))], 60
    )[0]

    existing_backstep = [
        (20, odometry(200)),
        (30, odometry(400)),
        (40, odometry(300)),
    ]
    assert not validator.exact_raw_to_existing_stamp_coverage(
        raw_records, existing_backstep, 60
    )[0]

    raw_record_backstep = copy.deepcopy(raw_records)
    raw_record_backstep[3] = (15, raw_record_backstep[3][1])
    assert not validator.exact_raw_to_existing_stamp_coverage(
        raw_record_backstep, existing_records, 60
    )[0]


def readiness_item(stamp: int, level: int, message: str, **overrides):
    values = {
        "activation.epoch": "0",
        "activation.stable_candidate_count": "0",
        "activation.required_candidate_count": "3",
        "activation.max_candidate_delta_rad": "0.08",
        "activation.commit_count": "0",
        "activation.stamp_sec": "nan",
        "activation.evidence_valid": "false",
        "activation.stamp_ns": "0",
        "activation.committed_stable_candidate_count": "0",
        "activation.committed_candidate_delta_rad": "nan",
        "activation.authority_session_id": "0",
        "activation.authority_sequence": "0",
        "activation.authority_stamp_ns": "0",
        "activation.authority_source_stamp_ns": "0",
        "activation.authority_received_stamp_ns": "0",
        "activation.existing_global_lower_stamp_ns": "0",
        "activation.existing_global_upper_stamp_ns": "0",
        "activation.existing_global_watermark_ns": "0",
        "activation.existing_global_max_interpolation_gap_ns": "0",
        "activation.existing_global_mode": "none",
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
        "fusion.health.authority_state": "unhealthy",
        "fusion.health.authority_source_stamp_sec": "nan",
        "fusion.health.authority_received_stamp_sec": "nan",
        "fusion.health.authority_source_age_sec": "nan",
        "fusion.health.authority_transport_age_sec": "nan",
        "fusion.health.authority_session_id": "0",
        "fusion.health.authority_sequence": "0",
        "fusion.health.authority_reason": "fusion_authority_unavailable",
        "fusion.health.recovery_state": "uninitialized",
        "fusion.health.anchor_valid": "false",
        "fusion.health.position_fused": "false",
        "fusion.health.yaw_fused": "false",
        "fusion.health.last_fix_state": "bad",
        "fusion.health.reason": "fusion_diagnostics_unavailable",
        "fusion.authority.deferred": "0",
        "fusion.authority.pending": "0",
        "fusion.authority.deferred_overflow": "0",
        "fusion.authority.receive_clock_initialized": "true",
        "fusion.authority.startup_overflow_latched": "false",
        "fusion.health.rearm_required": "false",
        "fusion.health.rearm_saw_unhealthy": "false",
        "fusion.health.rearmed": "true",
        "fusion.health.rearm_reset_stamp_sec": "nan",
        "fusion.health.rearm.reset_stamp_ns": "0",
        "fusion.health.rearm.unhealthy_evidence_valid": "false",
        "fusion.health.rearm.unhealthy_session_id": "0",
        "fusion.health.rearm.unhealthy_sequence": "0",
        "fusion.health.rearm.unhealthy_stamp_ns": "0",
        "fusion.health.rearm.unhealthy_source_stamp_ns": "0",
        "fusion.health.rearm.unhealthy_received_stamp_ns": "0",
        "fusion.health.rearm.healthy_evidence_valid": "false",
        "fusion.health.rearm.healthy_session_id": "0",
        "fusion.health.rearm.healthy_sequence": "0",
        "fusion.health.rearm.healthy_stamp_ns": "0",
        "fusion.health.rearm.healthy_source_stamp_ns": "0",
        "fusion.health.rearm.healthy_received_stamp_ns": "0",
        "fusion.anchor.state": "WAITING_HEALTHY",
        "fusion.anchor.frozen_residual_variance_x_m2": "0",
        "fusion.anchor.frozen_residual_variance_y_m2": "0",
        "fusion.anchor.frozen_residual_variance_yaw_rad2": "0",
        "fusion.sync.existing_global_stamp_sec": "nan",
        "fusion.sync.existing_global_stamp_ns": "0",
        "fusion.sync.existing_global_age_sec": "nan",
        "fusion.sync.last_valid_stamp_sec": "nan",
        "fusion.sync.last_valid_age_sec": "nan",
        "local_correction.odom_session_resets": "0",
        "fallback.gnss_position_enabled": "false",
        "outage_yaw_guard.enabled": "true",
        "outage_yaw_guard.state": "DISARMED",
        "outage_yaw_guard.active": "false",
        "outage_yaw_guard.reference_source": "robust_gnss_position_alignment_yaw",
        "outage_yaw_guard.propagation_source": "precision_local_yaw",
        "outage_yaw_guard.xy_policy": (
            "existing_fusion_anchor_compose_precision_local"
        ),
        "outage_yaw_guard.reference_stamp_sec": "nan",
        "outage_yaw_guard.reference_age_sec": "nan",
        "outage_yaw_guard.trusted_anchor_yaw_rad": "nan",
        "outage_yaw_guard.observed_fusion_anchor_yaw_rad": "nan",
        "outage_yaw_guard.observed_delta_rad": "nan",
        "outage_yaw_guard.trusted_variance_rad2": "nan",
        "outage_yaw_guard.active_reference_variance_rad2": "nan",
        "outage_yaw_guard.nominal_global_yaw_rad": "nan",
        "outage_yaw_guard.output_global_yaw_rad": "nan",
        "outage_yaw_guard.applied_offset_rad": "0",
        "outage_yaw_guard.target_offset_rad": "0",
        "outage_yaw_guard.additional_variance_rad2": "0",
        "outage_yaw_guard.last_reason": "waiting_trusted_yaw_reference",
        "outage_yaw_guard.config.max_trusted_age_sec": "2.0",
        "outage_yaw_guard.config.max_trusted_variance_rad2": "0.0225",
        "outage_yaw_guard.config.max_trusted_delta_rad": "0.35",
        "outage_yaw_guard.config.max_offset_rate_radps": "0.20",
        "outage_yaw_guard.config.max_offset_step_rad": "0.04",
        "outage_yaw_guard.config.max_step_dt_sec": "0.25",
        "outage_yaw_guard.accepted_reference_count": "0",
        "outage_yaw_guard.rejected_reference_count": "0",
        "outage_yaw_guard.outage_count": "0",
        "outage_yaw_guard.active_reference_epoch": "0",
        "outage_yaw_guard.recovery_count": "0",
        "outage_yaw_guard.applied_step_count": "0",
        "outage_yaw_guard.invalid_advance_count": "0",
        "outage_yaw_guard.reset_count": "1",
        "publish.global_suppressed_yaw_guard_invalid": "0",
    }
    values.update(overrides)
    return {
        "stamp_ns": stamp,
        "level": level,
        "message": message,
        "values": values,
        "key_counts": {key: 1 for key in values},
    }


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
                "activation.evidence_valid": "true",
                "activation.stamp_ns": "2990000000",
                "activation.committed_stable_candidate_count": "3",
                "activation.committed_candidate_delta_rad": "0.001",
                "activation.authority_session_id": "10",
                "activation.authority_sequence": "3",
                "activation.authority_stamp_ns": "2980000000",
                "activation.authority_source_stamp_ns": "2970000000",
                "activation.authority_received_stamp_ns": "2985000000",
                "activation.existing_global_lower_stamp_ns": "2980000000",
                "activation.existing_global_upper_stamp_ns": "3000000000",
                "activation.existing_global_watermark_ns": "3000000000",
                "activation.existing_global_max_interpolation_gap_ns": "150000000",
                "activation.existing_global_mode": "interpolated",
                "activation.candidate_yaw_rad": "-1.2",
                "activation.candidate_delta_rad": "0.001",
                "activation.reason": "existing_fusion_stable_activated",
                "fusion.health.healthy": "true",
                "fusion.health.age_sec": "0.01",
                "fusion.health.status_stamp_sec": "2.98",
                "fusion.health.level": "0",
                "fusion.health.authority_state": "full_se2_healthy",
                "fusion.health.authority_source_stamp_sec": "2.97",
                "fusion.health.authority_received_stamp_sec": "2.99",
                "fusion.health.authority_source_age_sec": "0.01",
                "fusion.health.authority_transport_age_sec": "0.01",
                "fusion.health.authority_session_id": "10",
                "fusion.health.authority_sequence": "3",
                "fusion.health.authority_reason": "strict_full_se2_authority_ok",
                "fusion.health.recovery_state": "tracking",
                "fusion.health.anchor_valid": "true",
                "fusion.health.position_fused": "true",
                "fusion.health.yaw_fused": "true",
                "fusion.health.last_fix_state": "good",
                "fusion.health.reason": "strict_fusion_health_ok",
                "fusion.anchor.state": "TRACKING",
                "fusion.sync.existing_global_stamp_sec": "2.99",
                "fusion.sync.existing_global_stamp_ns": "3000000000",
                "fusion.sync.existing_global_age_sec": "0.01",
                "fusion.sync.last_valid_stamp_sec": "2.99",
                "fusion.sync.last_valid_age_sec": "0.01",
            },
        ),
    ]
    authority_records = [{
        "record_ns": 2_985_000_000,
        "stamp_ns": 2_980_000_000,
        "source_stamp_ns": 2_970_000_000,
        "session_id": 10,
        "sequence": 3,
        "state": 1,
        "reason": "strict_full_se2_authority_ok",
        "recovery_state": "tracking",
        "anchor_valid": True,
        "position_fused": True,
        "yaw_fused": True,
        "last_fix_state": 1,
    }]
    existing_global_stamps = {2_980_000_000, 3_000_000_000}
    valid, transitions, errors = validator.startup_transition_contract(
        timeline, authority_records, existing_global_stamps
    )
    assert valid, errors
    assert len(transitions) == 1
    publication_valid, publication_events, publication_errors = (
        validator.startup_publication_activation_events(timeline)
    )
    assert publication_valid, publication_errors
    assert publication_events[0]["activation_ns"] == 2_990_000_000
    summary = startup.diagnostic_transition_contract(
        timeline, authority_records, existing_global_stamps, validator
    )
    assert summary["valid"], summary["errors"]
    broken = [dict(item) for item in timeline]
    broken[-1] = {
        **broken[-1],
        "values": {**broken[-1]["values"], "global_output_ready": "false"},
    }
    assert not validator.startup_transition_contract(
        broken, authority_records, existing_global_stamps
    )[0]
    assert not startup.diagnostic_transition_contract(
        broken, authority_records, existing_global_stamps, validator
    )["valid"]
    for key, value in (
        ("activation.evidence_valid", "false"),
        ("activation.stamp_ns", "0"),
        ("activation.committed_stable_candidate_count", "2"),
        ("activation.committed_candidate_delta_rad", "0.081"),
        ("activation.authority_sequence", "0"),
        ("activation.authority_stamp_ns", "2980000001"),
        ("activation.authority_received_stamp_ns", "5000000000"),
        ("activation.existing_global_lower_stamp_ns", "2970000000"),
        ("activation.existing_global_upper_stamp_ns", "2990000000"),
        ("activation.existing_global_watermark_ns", "2500000000"),
        ("activation.existing_global_max_interpolation_gap_ns", "10000000"),
        ("activation.existing_global_mode", "none"),
    ):
        invalid_authority = [dict(item) for item in timeline]
        invalid_authority[-1] = {
            **invalid_authority[-1],
            "values": {**invalid_authority[-1]["values"], key: value},
        }
        assert not validator.startup_transition_contract(
            invalid_authority, authority_records, existing_global_stamps
        )[0]
        if key != "activation.stamp_ns":
            # Control-proof failures do not become false publication-safety
            # failures when the exact activation marker remains available.
            assert validator.startup_publication_activation_events(
                invalid_authority
            )[0]

    soft_authority = [{**authority_records[0], "state": 2}]
    assert not validator.startup_transition_contract(
        timeline, soft_authority, existing_global_stamps
    )[0]
    stale_source_authority = [{
        **authority_records[0], "source_stamp_ns": 1_000_000_000
    }]
    assert not validator.startup_transition_contract(
        timeline, stale_source_authority, existing_global_stamps
    )[0]

    epoch_2_pre = readiness_item(
        4_000_000_000,
        1,
        "waiting_for_healthy_existing_fusion",
        **{"activation.epoch": "1", "activation.commit_count": "1"},
    )
    epoch_2_ready = copy.deepcopy(timeline[-1])
    epoch_2_ready["stamp_ns"] = 5_000_000_000
    epoch_2_ready["values"].update({
        "activation.epoch": "1",
        "activation.commit_count": "2",
        "activation.stamp_sec": "4.99",
        "activation.stamp_ns": "4990000000",
        "activation.authority_sequence": "4",
        "activation.authority_stamp_ns": "4980000000",
        "activation.authority_source_stamp_ns": "4970000000",
        "activation.authority_received_stamp_ns": "4985000000",
        "activation.existing_global_lower_stamp_ns": "4980000000",
        "activation.existing_global_upper_stamp_ns": "5000000000",
        "activation.existing_global_watermark_ns": "5000000000",
    })
    epoch_2_authority = {
        **authority_records[0],
        "record_ns": 4_985_000_000,
        "stamp_ns": 4_980_000_000,
        "source_stamp_ns": 4_970_000_000,
        "sequence": 4,
    }
    two_epoch_authorities = authority_records + [epoch_2_authority]
    two_epoch_globals = existing_global_stamps | {
        4_980_000_000, 5_000_000_000
    }
    ordered_epochs = timeline + [epoch_2_pre, epoch_2_ready]
    assert validator.startup_transition_contract(
        ordered_epochs, two_epoch_authorities, two_epoch_globals
    )[0]
    epoch_regression = [
        timeline[0], epoch_2_pre, epoch_2_ready, timeline[1], timeline[2]
    ]
    assert not validator.startup_transition_contract(
        epoch_regression, two_epoch_authorities, two_epoch_globals
    )[0]

    stale_pre_ready = copy.deepcopy(epoch_2_pre)
    stale_pre_ready["values"].update({
        key: value for key, value in timeline[-1]["values"].items()
        if key.startswith("activation.")
        and key not in {"activation.epoch", "activation.commit_count"}
    })
    assert not validator.startup_transition_contract(
        timeline + [stale_pre_ready, epoch_2_ready],
        two_epoch_authorities,
        two_epoch_globals,
    )[0]

    frozen_values = {
        **timeline[-1]["values"],
        "state": "FROZEN",
        "fusion.anchor.state": "FROZEN",
        "fusion.health.healthy": "false",
        "fusion.health.level": "1",
        "fusion.health.authority_state": "soft_bad_hold",
        "fusion.health.authority_sequence": "4",
        "fusion.health.authority_reason": "gnss_soft_bad_within_grace",
        "fusion.health.recovery_state": "tracking",
        "fusion.health.last_fix_state": "bad",
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
    authority_records = [
        {"session_id": 10, "sequence": 3, "state": 1},
        {"session_id": 10, "sequence": 4, "state": 2},
    ]
    freeze_valid, groups, freeze_errors = validator.fusion_anchor_freeze_contract(
        frozen_timeline, authority_records
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
    assert not validator.fusion_anchor_freeze_contract(
        broken_freeze, authority_records
    )[0]

    # A short FULL edge between two sampled HOLD diagnostics starts a new
    # freeze segment; cross-segment anchor changes must not be joined.
    second_hold = {
        **frozen_timeline[-1],
        "stamp_ns": 6_000_000_000,
        "values": {
            **frozen_timeline[-1]["values"],
            "fusion.health.authority_sequence": "6",
            "anchor.target.x_m": "11.0",
            "anchor.applied.x_m": "11.0",
        },
    }
    split_authorities = authority_records + [
        {"session_id": 10, "sequence": 5, "state": 1},
        {"session_id": 10, "sequence": 6, "state": 2},
    ]
    assert validator.fusion_anchor_freeze_contract(
        [frozen_timeline[-1], second_hold], split_authorities
    )[0]

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
        "fusion.health.rearm.reset_stamp_ns": "3500000000",
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
            "fusion.health.rearm.unhealthy_evidence_valid": "true",
            "fusion.health.rearm.unhealthy_session_id": "20",
            "fusion.health.rearm.unhealthy_sequence": "1",
            "fusion.health.rearm.unhealthy_stamp_ns": "4900000000",
            "fusion.health.rearm.unhealthy_source_stamp_ns": "4800000000",
            "fusion.health.rearm.unhealthy_received_stamp_ns": "4950000000",
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
            "fusion.health.rearm.healthy_evidence_valid": "true",
            "fusion.health.rearm.healthy_session_id": "20",
            "fusion.health.rearm.healthy_sequence": "2",
            "fusion.health.rearm.healthy_stamp_ns": "5900000000",
            "fusion.health.rearm.healthy_source_stamp_ns": "5800000000",
            "fusion.health.rearm.healthy_received_stamp_ns": "5950000000",
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
    rearm_authorities = [
        {
            "record_ns": 4_950_000_000,
            "session_id": 20,
            "sequence": 1,
            "stamp_ns": 4_900_000_000,
            "stamp_canonical": True,
            "source_stamp_ns": 4_800_000_000,
            "source_stamp_canonical": True,
            "frame_id": "map",
            "state": 0,
            "reason": "fusion_not_tracking:outage",
            "recovery_state": "outage",
            "anchor_valid": False,
            "position_fused": False,
            "yaw_fused": False,
            "last_fix_state": 2,
        },
        {
            "record_ns": 5_950_000_000,
            "session_id": 20,
            "sequence": 2,
            "stamp_ns": 5_900_000_000,
            "stamp_canonical": True,
            "source_stamp_ns": 5_800_000_000,
            "source_stamp_canonical": True,
            "frame_id": "map",
            "state": 1,
            "reason": "strict_full_se2_authority_ok",
            "recovery_state": "tracking",
            "anchor_valid": True,
            "position_fused": True,
            "yaw_fused": True,
            "last_fix_state": 1,
        },
    ]
    rearm_valid, rearm_summary, rearm_errors = validator.fusion_rearm_contract(
        [reset_wait, explicit_unhealthy, strict_rearmed],
        rearm_authorities,
        {3_500_000_000},
    )
    assert rearm_valid, (rearm_summary, rearm_errors)
    bad_frame_authorities = copy.deepcopy(rearm_authorities)
    bad_frame_authorities[0]["frame_id"] = "wrong"
    assert not validator.fusion_rearm_contract(
        [strict_rearmed], bad_frame_authorities, {3_500_000_000}
    )[0]
    incomplete_rearm = validator.fusion_rearm_contract(
        [reset_wait, explicit_unhealthy],
        rearm_authorities,
        {3_500_000_000},
    )
    assert not incomplete_rearm[0], incomplete_rearm
    contradictory_flags = copy.deepcopy(explicit_unhealthy)
    contradictory_flags["values"].update({
        "fusion.health.rearm_required": "false",
        "fusion.health.rearmed": "false",
    })
    assert not validator.fusion_rearm_contract(
        [reset_wait, contradictory_flags],
        rearm_authorities,
        {3_500_000_000},
    )[0]
    next_reset = copy.deepcopy(reset_wait)
    next_reset["stamp_ns"] = 7_000_000_000
    next_reset["values"].update({
        "local_correction.odom_session_resets": "2",
        "fusion.health.rearm_reset_stamp_sec": "6.5",
        "fusion.health.rearm.reset_stamp_ns": "6500000000",
    })
    retired_incomplete = validator.fusion_rearm_contract(
        [reset_wait, explicit_unhealthy, next_reset],
        rearm_authorities,
        {3_500_000_000, 6_500_000_000},
    )
    assert not retired_incomplete[0]
    assert any("retired before" in error for error in retired_incomplete[2])
    hidden_edges_valid, hidden_edges_summary, hidden_edges_errors = (
        validator.fusion_rearm_contract(
            [strict_rearmed], rearm_authorities, {3_500_000_000}
        )
    )
    assert hidden_edges_valid, (hidden_edges_summary, hidden_edges_errors)
    assert hidden_edges_summary["completed_resets"] == 1
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
    assert not validator.fusion_rearm_contract(
        [reset_wait, stale_only], rearm_authorities, {3_500_000_000}
    )[0]
    missing_typed_unhealthy = copy.deepcopy(explicit_unhealthy)
    missing_typed_unhealthy["values"].update({
        "fusion.health.rearm.unhealthy_session_id": "99",
        "fusion.health.rearm.unhealthy_sequence": "99",
    })
    assert not validator.fusion_rearm_contract(
        [reset_wait, missing_typed_unhealthy, strict_rearmed],
        rearm_authorities,
        {3_500_000_000},
    )[0]

    activation_raw, next_raw = validator.activation_raw_successor(
        [1_000_000_000, 1_050_000_000, 1_100_000_000],
        1_000_000_000,
        0,
    )
    assert activation_raw == 1_000_000_000
    assert next_raw == 1_050_000_000
    assert startup.activation_raw_successor(
        [1_000_000_000, 1_050_000_000], 1_000_000_000, 0
    ) == (1_000_000_000, 1_050_000_000)


def test_outage_yaw_guard_runtime_contract(validator) -> None:
    assert not validator.outage_yaw_guard_contract([])[0]
    reference_1 = {
        "outage_yaw_guard.reference_stamp_sec": "1.9",
        "outage_yaw_guard.reference_age_sec": "0.1",
        "outage_yaw_guard.trusted_anchor_yaw_rad": "0.30",
        "outage_yaw_guard.observed_fusion_anchor_yaw_rad": "0.20",
        "outage_yaw_guard.observed_delta_rad": "0.10",
        "outage_yaw_guard.trusted_variance_rad2": "0.01",
    }
    reference_2 = {
        "outage_yaw_guard.reference_stamp_sec": "4.9",
        "outage_yaw_guard.reference_age_sec": "0.1",
        "outage_yaw_guard.trusted_anchor_yaw_rad": "0.31",
        "outage_yaw_guard.observed_fusion_anchor_yaw_rad": "0.25",
        "outage_yaw_guard.observed_delta_rad": "0.06",
        "outage_yaw_guard.trusted_variance_rad2": "0.008",
    }
    reference_3 = {
        "outage_yaw_guard.reference_stamp_sec": "5.9",
        "outage_yaw_guard.reference_age_sec": "0.1",
        "outage_yaw_guard.trusted_anchor_yaw_rad": "0.32",
        "outage_yaw_guard.observed_fusion_anchor_yaw_rad": "0.26",
        "outage_yaw_guard.observed_delta_rad": "0.06",
        "outage_yaw_guard.trusted_variance_rad2": "0.008",
    }

    def guard_item(stamp: int, guard_state: str, **updates):
        active = guard_state in {
            "OUTAGE_SLEW",
            "OUTAGE_HOLD",
            "RECOVERY_RELEASE",
        }
        item = readiness_item(stamp, 1, "guard_test")
        item["values"].update(
            {
                "global_output_ready": "true",
                "anchor.initialized": "true",
                "position_fused": "true",
                "outage_yaw_guard.state": guard_state,
                "outage_yaw_guard.active": str(active).lower(),
            }
        )
        item["values"].update(updates)
        if "outage_yaw_guard.active_reference_epoch" not in updates:
            outage_count = int(
                item["values"]["outage_yaw_guard.outage_count"]
            )
            item["values"]["outage_yaw_guard.active_reference_epoch"] = str(
                0 if outage_count == 0 else 1
            )
        item["key_counts"] = {key: 1 for key in item["values"]}
        return item

    timeline = [
        readiness_item(1_000_000_000, 1, "waiting_for_healthy_existing_fusion"),
        guard_item(
            2_000_000_000,
            "READY",
            **reference_1,
            **{
                "fusion.health.healthy": "true",
                "fusion.anchor.state": "TRACKING",
                "outage_yaw_guard.nominal_global_yaw_rad": "0.5",
                "outage_yaw_guard.output_global_yaw_rad": "0.5",
                "outage_yaw_guard.last_reason": "trusted_yaw_reference_ready",
                "outage_yaw_guard.accepted_reference_count": "1",
            },
        ),
        guard_item(
            3_000_000_000,
            "OUTAGE_SLEW",
            **{
                **reference_1,
                "outage_yaw_guard.reference_age_sec": "1.1",
                "fusion.health.healthy": "false",
                "fusion.anchor.state": "FROZEN",
                "outage_yaw_guard.nominal_global_yaw_rad": "0.6",
                "outage_yaw_guard.output_global_yaw_rad": "0.6",
                "outage_yaw_guard.applied_offset_rad": "0",
                "outage_yaw_guard.target_offset_rad": "0.08",
                "outage_yaw_guard.active_reference_variance_rad2": "0.01",
                "outage_yaw_guard.additional_variance_rad2": "0.0164",
                "outage_yaw_guard.last_reason": "trusted_outage_yaw_slew_started",
                "outage_yaw_guard.accepted_reference_count": "1",
                "outage_yaw_guard.outage_count": "1",
            },
        ),
        guard_item(
            4_000_000_000,
            "OUTAGE_HOLD",
            **{
                **reference_1,
                "outage_yaw_guard.reference_age_sec": "2.1",
                # Strict fusion health may return before the anchor completes
                # its N-candidate recovery. It is still not authoritative
                # TRACKING, so the outage snapshot must remain active.
                "fusion.health.healthy": "true",
                "fusion.anchor.state": "STABILIZING_RECOVERY",
                "outage_yaw_guard.nominal_global_yaw_rad": "0.7",
                "outage_yaw_guard.output_global_yaw_rad": "0.78",
                "outage_yaw_guard.applied_offset_rad": "0.08",
                "outage_yaw_guard.target_offset_rad": "0.08",
                "outage_yaw_guard.active_reference_variance_rad2": "0.01",
                "outage_yaw_guard.additional_variance_rad2": "0.01",
                "outage_yaw_guard.last_reason": "trusted_outage_yaw_held",
                "outage_yaw_guard.accepted_reference_count": "1",
                "outage_yaw_guard.outage_count": "1",
                "outage_yaw_guard.applied_step_count": "2",
            },
        ),
        guard_item(
            5_000_000_000,
            "RECOVERY_RELEASE",
            **reference_2,
            **{
                "fusion.health.healthy": "true",
                "fusion.anchor.state": "TRACKING",
                "outage_yaw_guard.nominal_global_yaw_rad": "0.8",
                "outage_yaw_guard.output_global_yaw_rad": "0.88",
                "outage_yaw_guard.applied_offset_rad": "0.08",
                "outage_yaw_guard.target_offset_rad": "0",
                "outage_yaw_guard.active_reference_variance_rad2": "0.01",
                "outage_yaw_guard.additional_variance_rad2": "0.0164",
                "outage_yaw_guard.last_reason": (
                    "trusted_yaw_reference_refreshed_during_release"
                ),
                "outage_yaw_guard.accepted_reference_count": "2",
                "outage_yaw_guard.outage_count": "1",
                "outage_yaw_guard.recovery_count": "1",
                "outage_yaw_guard.applied_step_count": "2",
            },
        ),
        guard_item(
            6_000_000_000,
            "RECOVERY_RELEASE",
            **reference_3,
            **{
                "fusion.health.healthy": "true",
                "fusion.anchor.state": "TRACKING",
                "outage_yaw_guard.nominal_global_yaw_rad": "0.9",
                "outage_yaw_guard.output_global_yaw_rad": "0.94",
                "outage_yaw_guard.applied_offset_rad": "0.04",
                "outage_yaw_guard.target_offset_rad": "0",
                "outage_yaw_guard.active_reference_variance_rad2": "0.01",
                "outage_yaw_guard.additional_variance_rad2": "0.0116",
                "outage_yaw_guard.last_reason": "outage_yaw_recovery_release_step",
                "outage_yaw_guard.accepted_reference_count": "3",
                "outage_yaw_guard.outage_count": "1",
                "outage_yaw_guard.recovery_count": "1",
                "outage_yaw_guard.applied_step_count": "3",
            },
        ),
        guard_item(
            7_000_000_000,
            "READY",
            **{
                **reference_3,
                "outage_yaw_guard.reference_age_sec": "1.1",
                "fusion.health.healthy": "true",
                "fusion.anchor.state": "TRACKING",
                "outage_yaw_guard.nominal_global_yaw_rad": "1.0",
                "outage_yaw_guard.output_global_yaw_rad": "1.0",
                "outage_yaw_guard.last_reason": (
                    "outage_yaw_recovery_release_complete_ready"
                ),
                "outage_yaw_guard.accepted_reference_count": "3",
                "outage_yaw_guard.outage_count": "1",
                "outage_yaw_guard.recovery_count": "1",
                "outage_yaw_guard.applied_step_count": "4",
            },
        ),
    ]

    valid, summary, errors = validator.outage_yaw_guard_contract(timeline)
    assert valid, errors
    assert summary["samples"] == len(timeline)
    assert summary["outage_samples"] == 2
    assert summary["release_samples"] == 2
    assert math.isclose(
        summary["maximum_active_reference_variance_rad2"], 0.01
    )

    reoutage = guard_item(
        6_000_000_000,
        "OUTAGE_HOLD",
        **{
            "outage_yaw_guard.reference_stamp_sec": "5.9",
            "outage_yaw_guard.reference_age_sec": "0.1",
            "outage_yaw_guard.trusted_anchor_yaw_rad": "0.33",
            "outage_yaw_guard.observed_fusion_anchor_yaw_rad": "0.27",
            "outage_yaw_guard.observed_delta_rad": "0.06",
            "outage_yaw_guard.trusted_variance_rad2": "0.012",
            "fusion.health.healthy": "false",
            "fusion.anchor.state": "FROZEN",
            "outage_yaw_guard.nominal_global_yaw_rad": "0.9",
            "outage_yaw_guard.output_global_yaw_rad": "0.98",
            "outage_yaw_guard.applied_offset_rad": "0.08",
            "outage_yaw_guard.target_offset_rad": "0.08",
            "outage_yaw_guard.active_reference_variance_rad2": "0.012",
            "outage_yaw_guard.additional_variance_rad2": "0.012",
            "outage_yaw_guard.last_reason": (
                "trusted_outage_yaw_reentered_during_release"
            ),
            "outage_yaw_guard.accepted_reference_count": "3",
            "outage_yaw_guard.outage_count": "2",
            "outage_yaw_guard.recovery_count": "1",
            "outage_yaw_guard.applied_step_count": "2",
        },
    )
    reoutage_timeline = timeline[:5] + [reoutage]
    assert validator.outage_yaw_guard_contract(reoutage_timeline)[0]
    understated_reoutage = copy.deepcopy(reoutage_timeline)
    understated_reoutage[-1]["values"].update(
        {
            "outage_yaw_guard.active_reference_variance_rad2": "0.011",
            "outage_yaw_guard.additional_variance_rad2": "0.011",
        }
    )
    assert not validator.outage_yaw_guard_contract(understated_reoutage)[0]

    for retain_reason in sorted(
        validator.OUTAGE_YAW_GUARD_REOUTAGE_RETAIN_REASONS
    ):
        retained_reoutage = copy.deepcopy(reoutage_timeline)
        retained_reoutage[-1]["values"].update(
            {
                "outage_yaw_guard.last_reason": retain_reason,
                "outage_yaw_guard.active_reference_variance_rad2": "0.01",
                "outage_yaw_guard.additional_variance_rad2": "0.01",
            }
        )
        assert validator.outage_yaw_guard_contract(retained_reoutage)[0], (
            retain_reason,
            validator.outage_yaw_guard_contract(retained_reoutage)[2],
        )
        wrongly_merged = copy.deepcopy(retained_reoutage)
        wrongly_merged[-1]["values"].update(
            {
                "outage_yaw_guard.active_reference_variance_rad2": "0.012",
                "outage_yaw_guard.additional_variance_rad2": "0.012",
            }
        )
        assert not validator.outage_yaw_guard_contract(wrongly_merged)[0]

    fresh_with_lower_trusted = copy.deepcopy(reoutage_timeline)
    fresh_with_lower_trusted[-1]["values"].update(
        {
            "outage_yaw_guard.trusted_variance_rad2": "0.008",
            "outage_yaw_guard.active_reference_variance_rad2": "0.01",
            "outage_yaw_guard.additional_variance_rad2": "0.01",
        }
    )
    assert validator.outage_yaw_guard_contract(fresh_with_lower_trusted)[0]

    wrong_visible_reason = copy.deepcopy(reoutage_timeline)
    wrong_visible_reason[-1]["values"]["outage_yaw_guard.last_reason"] = (
        "trusted_outage_yaw_held"
    )
    assert validator.outage_yaw_guard_contract(wrong_visible_reason)[0]

    hidden_reoutage = copy.deepcopy(reoutage_timeline)
    hidden_reoutage[1] = copy.deepcopy(timeline[2])
    hidden_reoutage = [hidden_reoutage[1], hidden_reoutage[-1]]
    hidden_reoutage[-1]["values"]["outage_yaw_guard.last_reason"] = (
        "trusted_outage_yaw_held"
    )
    assert validator.outage_yaw_guard_contract(hidden_reoutage)[0]
    hidden_understated = copy.deepcopy(hidden_reoutage)
    hidden_understated[-1]["values"].update(
        {
            "outage_yaw_guard.active_reference_variance_rad2": "0.009",
            "outage_yaw_guard.additional_variance_rad2": "0.009",
        }
    )
    assert not validator.outage_yaw_guard_contract(hidden_understated)[0]

    cross_epoch = copy.deepcopy(hidden_understated)
    cross_epoch[-1]["values"].update(
        {
            "outage_yaw_guard.trusted_variance_rad2": "0.008",
            "outage_yaw_guard.active_reference_variance_rad2": "0.008",
            "outage_yaw_guard.additional_variance_rad2": "0.008",
            "outage_yaw_guard.active_reference_epoch": "2",
        }
    )
    cross_epoch_valid, cross_epoch_summary, cross_epoch_errors = (
        validator.outage_yaw_guard_contract(cross_epoch)
    )
    assert cross_epoch_valid, cross_epoch_errors
    assert cross_epoch_summary["active_reference_epoch_accounting"] == {
        "available": True,
        "mixed_presence": False,
        "unproven_intervals": 0,
        "cross_epoch_intervals": 1,
    }

    legacy_understated = copy.deepcopy(hidden_understated)
    for item in legacy_understated:
        del item["values"]["outage_yaw_guard.active_reference_epoch"]
        item["key_counts"].pop(
            "outage_yaw_guard.active_reference_epoch", None
        )
    legacy_valid, legacy_summary, legacy_errors = (
        validator.outage_yaw_guard_contract(legacy_understated)
    )
    assert legacy_valid, legacy_errors
    assert legacy_summary["active_reference_epoch_accounting"] == {
        "available": False,
        "mixed_presence": False,
        "unproven_intervals": 1,
        "cross_epoch_intervals": 0,
    }

    mixed_epoch = copy.deepcopy(hidden_reoutage)
    del mixed_epoch[0]["values"]["outage_yaw_guard.active_reference_epoch"]
    mixed_epoch[0]["key_counts"].pop(
        "outage_yaw_guard.active_reference_epoch", None
    )
    assert not validator.outage_yaw_guard_contract(mixed_epoch)[0]

    impossible_epoch = copy.deepcopy(hidden_reoutage)
    impossible_epoch[-1]["values"][
        "outage_yaw_guard.active_reference_epoch"
    ] = "3"
    assert not validator.outage_yaw_guard_contract(impossible_epoch)[0]
    hidden_bad_counters = copy.deepcopy(hidden_reoutage)
    hidden_bad_counters[-1]["values"][
        "outage_yaw_guard.recovery_count"
    ] = "0"
    assert not validator.outage_yaw_guard_contract(hidden_bad_counters)[0]

    hidden_release_cycle = [
        timeline[4],
        copy.deepcopy(timeline[5]),
    ]
    hidden_release_cycle[-1]["values"].update(
        {
            "outage_yaw_guard.trusted_variance_rad2": "0.012",
            "outage_yaw_guard.active_reference_variance_rad2": "0.012",
            "outage_yaw_guard.additional_variance_rad2": "0.0136",
            "outage_yaw_guard.outage_count": "2",
            "outage_yaw_guard.recovery_count": "2",
        }
    )
    hidden_release_valid, _, hidden_release_errors = (
        validator.outage_yaw_guard_contract(hidden_release_cycle)
    )
    assert hidden_release_valid, hidden_release_errors

    outage_through_multiple_cycles = [
        timeline[3],
        copy.deepcopy(timeline[4]),
    ]
    outage_through_multiple_cycles[-1]["values"].update(
        {
            "outage_yaw_guard.trusted_variance_rad2": "0.012",
            "outage_yaw_guard.active_reference_variance_rad2": "0.012",
            "outage_yaw_guard.additional_variance_rad2": "0.0184",
            "outage_yaw_guard.outage_count": "3",
            "outage_yaw_guard.recovery_count": "3",
        }
    )
    multiple_cycles_valid, _, multiple_cycles_errors = (
        validator.outage_yaw_guard_contract(outage_through_multiple_cycles)
    )
    assert multiple_cycles_valid, multiple_cycles_errors

    negative_endpoint_balance = copy.deepcopy(outage_through_multiple_cycles)
    negative_endpoint_balance[-1]["values"].update(
        {
            "outage_yaw_guard.outage_count": "1",
            "outage_yaw_guard.recovery_count": "0",
            "outage_yaw_guard.active_reference_variance_rad2": "0.01",
            "outage_yaw_guard.additional_variance_rad2": "0.0164",
        }
    )
    negative_valid, _, negative_errors = validator.outage_yaw_guard_contract(
        negative_endpoint_balance
    )
    assert not negative_valid
    assert any(
        "endpoint balance mismatch" in error
        and "expected_outage_delta=-1" in error
        for error in negative_errors
    )

    hidden_release_complete = guard_item(
        5_000_000_000,
        "DISARMED",
        **{
            "fusion.health.healthy": "true",
            "fusion.anchor.state": "TRACKING",
            "outage_yaw_guard.nominal_global_yaw_rad": "0.8",
            "outage_yaw_guard.output_global_yaw_rad": "0.8",
            "outage_yaw_guard.last_reason": (
                "outage_yaw_recovery_release_complete_disarmed"
            ),
            "outage_yaw_guard.accepted_reference_count": "1",
            "outage_yaw_guard.outage_count": "1",
            "outage_yaw_guard.recovery_count": "1",
            "outage_yaw_guard.applied_step_count": "4",
        },
    )
    hidden_release_timeline = timeline[:4] + [hidden_release_complete]
    hidden_valid, hidden_summary, hidden_errors = (
        validator.outage_yaw_guard_contract(hidden_release_timeline)
    )
    assert hidden_valid, hidden_errors
    assert hidden_summary["release_samples"] == 0

    # A 1 Hz diagnostic can observe OUTAGE_HOLD and then DISARMED even when a
    # short, valid FULL authority recovery happened between the snapshots and
    # the endpoint has already returned to HOLD.  The raw typed stream proves
    # that hidden recovery; the recovery counter alone is insufficient.
    hidden_typed_release = [
        copy.deepcopy(timeline[3]), copy.deepcopy(hidden_release_complete)
    ]
    hidden_typed_release[0]["values"].update({
        "fusion.health.authority_session_id": "10",
        "fusion.health.authority_sequence": "10",
    })
    hidden_typed_release[1]["values"].update({
        "fusion.health.healthy": "false",
        "fusion.anchor.state": "FROZEN",
        "fusion.health.authority_state": "soft_bad_hold",
        "fusion.health.authority_session_id": "10",
        "fusion.health.authority_sequence": "13",
    })
    hidden_authority = [
        {"session_id": 10, "sequence": 10, "state": 2},
        {"session_id": 10, "sequence": 11, "state": 1},
        {"session_id": 10, "sequence": 12, "state": 1},
        {"session_id": 10, "sequence": 13, "state": 2},
    ]
    typed_hidden_valid, typed_hidden_summary, typed_hidden_errors = (
        validator.outage_yaw_guard_contract(
            hidden_typed_release, hidden_authority
        )
    )
    assert typed_hidden_valid, typed_hidden_errors
    assert typed_hidden_summary[
        "hidden_recovery_full_authority_intervals"
    ] == 1
    no_full_authority = [
        {**event, "state": 2} for event in hidden_authority
    ]
    assert not validator.outage_yaw_guard_contract(
        hidden_typed_release, no_full_authority
    )[0]

    # Once the bounded release has completed and the stale reference has been
    # discarded, a later brief authority flap remains nominal and fail-closed.
    disarmed_frozen_flap = guard_item(
        6_000_000_000,
        "DISARMED",
        **{
            "fusion.health.healthy": "false",
            "fusion.anchor.state": "FROZEN",
            "outage_yaw_guard.nominal_global_yaw_rad": "0.9",
            "outage_yaw_guard.output_global_yaw_rad": "0.9",
            "outage_yaw_guard.last_reason": "outage_yaw_guard_disarmed",
            "outage_yaw_guard.accepted_reference_count": "1",
            "outage_yaw_guard.outage_count": "1",
            "outage_yaw_guard.recovery_count": "1",
            "outage_yaw_guard.applied_step_count": "4",
        },
    )
    flap_valid, _, flap_errors = validator.outage_yaw_guard_contract(
        [*hidden_release_timeline, disarmed_frozen_flap]
    )
    assert flap_valid, flap_errors

    hidden_without_recovery = [
        {**item, "values": dict(item["values"])}
        for item in hidden_release_timeline
    ]
    hidden_without_recovery[-1]["values"][
        "outage_yaw_guard.recovery_count"
    ] = "0"
    assert not validator.outage_yaw_guard_contract(hidden_without_recovery)[0]

    hidden_without_tracking = [
        {**item, "values": dict(item["values"])}
        for item in hidden_release_timeline
    ]
    hidden_without_tracking[-1]["values"].update(
        {
            "fusion.health.healthy": "false",
            "fusion.anchor.state": "FROZEN",
        }
    )
    assert not validator.outage_yaw_guard_contract(hidden_without_tracking)[0]

    leading_zero = [
        {
            **timeline[0],
            "stamp_ns": 0,
            "values": dict(timeline[0]["values"]),
        },
        *timeline,
    ]
    leading_zero_valid, leading_zero_summary, leading_zero_errors = (
        validator.outage_yaw_guard_contract(leading_zero)
    )
    assert leading_zero_valid, leading_zero_errors
    assert leading_zero_summary["leading_zero_stamp_samples"] == 1

    def changed(index: int, **updates):
        result = [
            {
                **item,
                "values": dict(item["values"]),
                "key_counts": dict(item.get("key_counts", {})),
            }
            for item in timeline
        ]
        result[index]["values"].update(updates)
        return result

    missing = changed(2)
    del missing[2]["values"]["outage_yaw_guard.target_offset_rad"]
    assert not validator.outage_yaw_guard_contract(missing)[0]

    unknown = changed(2, **{"outage_yaw_guard.state": "UNKNOWN"})
    assert not validator.outage_yaw_guard_contract(unknown)[0]

    returned_to_zero = [
        {**item, "values": dict(item["values"])} for item in timeline
    ]
    returned_to_zero[2]["stamp_ns"] = 0
    assert not validator.outage_yaw_guard_contract(returned_to_zero)[0]

    wrong_authority = changed(
        2,
        **{
            "fusion.health.healthy": "true",
            "fusion.anchor.state": "TRACKING",
        },
    )
    assert not validator.outage_yaw_guard_contract(wrong_authority)[0]

    missing_active_output = changed(
        2,
        **{
            "outage_yaw_guard.nominal_global_yaw_rad": "nan",
            "outage_yaw_guard.output_global_yaw_rad": "nan",
        },
    )
    assert not validator.outage_yaw_guard_contract(missing_active_output)[0]

    illegal_edge = changed(2, **{"outage_yaw_guard.outage_count": "0"})
    assert not validator.outage_yaw_guard_contract(illegal_edge)[0]

    excessive_step = changed(
        3,
        **{
            "outage_yaw_guard.output_global_yaw_rad": "0.9",
            "outage_yaw_guard.applied_offset_rad": "0.2",
            "outage_yaw_guard.target_offset_rad": "0.2",
            "outage_yaw_guard.additional_variance_rad2": "0.01",
            "outage_yaw_guard.applied_step_count": "1",
        },
    )
    assert not validator.outage_yaw_guard_contract(excessive_step)[0]

    invalid_advance = changed(
        4, **{"outage_yaw_guard.invalid_advance_count": "1"}
    )
    assert not validator.outage_yaw_guard_contract(invalid_advance)[0]
    suppressed = changed(
        4, **{"publish.global_suppressed_yaw_guard_invalid": "1"}
    )
    assert not validator.outage_yaw_guard_contract(suppressed)[0]

    config_drift = changed(
        5, **{"outage_yaw_guard.config.max_offset_step_rad": "0.05"}
    )
    assert not validator.outage_yaw_guard_contract(config_drift)[0]

    malformed_counter = changed(
        5, **{"outage_yaw_guard.applied_step_count": "3.0"}
    )
    assert not validator.outage_yaw_guard_contract(malformed_counter)[0]

    missing_active_variance = changed(2)
    del missing_active_variance[2]["values"][
        "outage_yaw_guard.active_reference_variance_rad2"
    ]
    missing_active_variance[2]["key_counts"][
        "outage_yaw_guard.active_reference_variance_rad2"
    ] = 0
    assert not validator.outage_yaw_guard_contract(missing_active_variance)[0]

    duplicate_active_variance = changed(2)
    duplicate_active_variance[2]["key_counts"][
        "outage_yaw_guard.active_reference_variance_rad2"
    ] = 2
    assert not validator.outage_yaw_guard_contract(duplicate_active_variance)[0]

    for malformed in ("nan", "inf", "-0.001"):
        invalid_active_variance = changed(
            2,
            **{
                "outage_yaw_guard.active_reference_variance_rad2": malformed,
            },
        )
        assert not validator.outage_yaw_guard_contract(invalid_active_variance)[0]

    uncleared_ready_variance = changed(
        1,
        **{"outage_yaw_guard.active_reference_variance_rad2": "0.01"},
    )
    assert not validator.outage_yaw_guard_contract(uncleared_ready_variance)[0]

    wrong_outage_variance = changed(
        2, **{"outage_yaw_guard.additional_variance_rad2": "0.0064"}
    )
    assert not validator.outage_yaw_guard_contract(wrong_outage_variance)[0]

    wrong_release_variance = changed(
        4, **{"outage_yaw_guard.additional_variance_rad2": "0.0064"}
    )
    assert not validator.outage_yaw_guard_contract(wrong_release_variance)[0]

    refreshed_snapshot = changed(
        5,
        **{"outage_yaw_guard.active_reference_variance_rad2": "0.008"},
    )
    assert not validator.outage_yaw_guard_contract(refreshed_snapshot)[0]


def main() -> None:
    validator = load(
        "precision_validator_under_test",
        "scripts/validate_precision_bag.py",
    )

    def diagnostic_transition_contract(
        timeline, authority_records, existing_global_stamps, _validator
    ):
        valid, transitions, errors = validator.startup_transition_contract(
            timeline, authority_records, existing_global_stamps
        )
        return {
            "valid": valid,
            "transitions": transitions,
            "errors": errors,
        }

    startup_proxy = SimpleNamespace(
        diagnostic_transition_contract=diagnostic_transition_contract,
        activation_raw_successor=validator.activation_raw_successor,
    )

    test_full_se2_composition(validator)
    test_existing_fusion_causal_accounting(validator)
    test_map_fusion_publication_integrity_contract(validator)
    test_map_fusion_exact_raw_stamp_coverage(validator)
    test_startup_readiness_contract(validator, startup_proxy)
    test_outage_yaw_guard_runtime_contract(validator)
    print("precision bag validator tests PASS")


if __name__ == "__main__":
    main()

