#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Dependency-light tests for the Hesai publication provenance validator."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/validate_hesai_gnss_publication_run.py"


def load_validator():
    spec = importlib.util.spec_from_file_location("hesai_publication_validator", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_run_env_parser(validator) -> None:
    values = validator.parse_run_env_text(
        "dataset=course-2\n"
        "name=Hesai\\ 32-Line\\ +\\ IMU\\ +\\ RTK\\ GNSS\n"
        "empty=''\n"
        "literal='$(touch /tmp/must-not-exist)'\n"
    )
    assert values["dataset"] == "course-2"
    assert values["name"] == "Hesai 32-Line + IMU + RTK GNSS"
    assert values["empty"] == ""
    assert values["literal"] == "$(touch /tmp/must-not-exist)"
    try:
        validator.parse_run_env_text("duplicate=first\nduplicate=second\n")
    except ValueError:
        pass
    else:
        raise AssertionError("duplicate run.env key was accepted")


def test_manifest_and_mode_contracts(validator) -> None:
    text = """schema_version: 1
dataset:
  id: course_1
  display_name: "Hesai 32-Line + IMU + RTK GNSS — Course 1"
  local_bag_hint: rosbag/private_course_1
  duration_sec: 210.757
  topics:
    pointcloud: /pandar_points_ex
    imu: /sensor/imu/data_raw
    nmea: /sensor/gnss/nmea_sentence
  static_transforms:
    base_to_lidar: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    base_to_imu: [0.0, 0.0, -0.1874, 3.14159, 0.0, 0.0]
    base_to_gnss: [0.0, 0.0, -0.1326, 0.0, 0.0, 0.0]
"""
    manifest = validator.parse_dataset_manifest_text(text)
    assert manifest["id"] == "course_1"
    assert manifest["duration_sec"] == 210.757
    assert manifest["topics"]["nmea"] == "/sensor/gnss/nmea_sentence"
    assert manifest["static_transforms"]["base_to_imu"][2] == -0.1874

    baseline = {item.role for item in validator.config_contracts("course-1", "baseline")}
    control = {item.role for item in validator.config_contracts("course-1", "control")}
    precision = {item.role for item in validator.config_contracts("course-1", "precision")}
    assert "precision_profile_manifest" not in baseline
    assert "precision_profile_manifest" in control
    assert "precision_matcher_base" not in control
    assert "precision_matcher_base" in precision
    assert "precision_global_base" not in baseline
    assert "precision_global_base" not in control
    assert "precision_global_base" in precision
    assert "nmea_base" in baseline
    assert "nmea_projector_metadata" in baseline
    assert "nmea_override" in baseline
    assert "nmea_site_override" not in baseline


def test_default_projection_contract(validator) -> None:
    repository = ROOT.parents[1]
    parameter_text = (
        repository / "src/pure_nmea_gnss_conversion/param/param.yaml"
    ).read_text(encoding="utf-8")
    metadata_text = (
        repository
        / "src/pure_nmea_gnss_conversion/config/map_projector_info.yaml"
    ).read_text(encoding="utf-8")
    empty_override_text = (
        repository
        / "src/pure_odometry_bringup/config/autoware_lsim/empty_params.yaml"
    ).read_text(encoding="utf-8")

    parameters = validator.nmea_parameter_semantics(parameter_text)
    metadata = validator.map_projector_metadata_semantics(metadata_text)
    assert parameters["valid"]
    assert metadata["valid"]
    assert validator.projection_semantics_agree(parameters, metadata)
    assert validator.empty_override_semantics(empty_override_text)["valid"]

    site_parameter_text = parameter_text.replace(
        "35.681236", "35.1254574925"
    ).replace("139.767125", "136.8226007017")
    site_parameters = validator.nmea_parameter_semantics(site_parameter_text)
    assert not site_parameters["valid"]
    assert not validator.projection_semantics_agree(site_parameters, metadata)

    site_override_text = """/**:
  ros__parameters:
    map_origin.latitude: 35.1254574925
    map_origin.longitude: 136.8226007017
"""
    assert not validator.empty_override_semantics(site_override_text)["valid"]

    mismatched_metadata_text = metadata_text.replace("139.767125", "139.767126")
    mismatched_metadata = validator.map_projector_metadata_semantics(
        mismatched_metadata_text
    )
    assert not mismatched_metadata["valid"]
    assert not validator.projection_semantics_agree(
        parameters, mismatched_metadata
    )


def test_precision_global_parameter_contract(validator) -> None:
    repository = ROOT.parents[1]
    parameter_text = (
        repository / "src/pure_precision_global_localizer/param/param.yaml"
    ).read_text(encoding="utf-8")
    semantics = validator.precision_global_parameter_semantics(parameter_text)
    assert semantics["valid"]
    assert not semantics["fallback_gnss_position_enabled"]
    assert semantics["outage_yaw_guard_enabled"]
    assert semantics["required_fix_quality"] == 4.0
    assert semantics["limits"] == validator.EXPECTED_OUTAGE_YAW_GUARD_LIMITS

    invalid_replacements = (
        (
            "fallback.gnss_position_enabled: false",
            "fallback.gnss_position_enabled: true",
        ),
        ("outage_yaw_guard.enabled: true", "outage_yaw_guard.enabled: false"),
        (
            "outage_yaw_guard.required_fix_quality: 4",
            "outage_yaw_guard.required_fix_quality: 5",
        ),
        (
            "outage_yaw_guard.max_trusted_age_sec: 2.0",
            "outage_yaw_guard.max_trusted_age_sec: 2.1",
        ),
        (
            "outage_yaw_guard.max_trusted_variance_rad2: 0.0225",
            "outage_yaw_guard.max_trusted_variance_rad2: 0.025",
        ),
        (
            "outage_yaw_guard.max_trusted_delta_rad: 0.35",
            "outage_yaw_guard.max_trusted_delta_rad: 0.36",
        ),
        (
            "outage_yaw_guard.max_offset_rate_radps: 0.20",
            "outage_yaw_guard.max_offset_rate_radps: 0.21",
        ),
        (
            "outage_yaw_guard.max_offset_step_rad: 0.04",
            "outage_yaw_guard.max_offset_step_rad: 0.05",
        ),
        (
            "outage_yaw_guard.max_step_dt_sec: 0.25",
            "outage_yaw_guard.max_step_dt_sec: 0.30",
        ),
    )
    for old, new in invalid_replacements:
        assert old in parameter_text
        altered = parameter_text.replace(old, new, 1)
        assert not validator.precision_global_parameter_semantics(altered)["valid"]

    missing = parameter_text.replace(
        "    outage_yaw_guard.max_step_dt_sec: 0.25\n", "", 1
    )
    try:
        validator.precision_global_parameter_semantics(missing)
    except ValueError:
        pass
    else:
        raise AssertionError("missing outage-yaw guard parameter was accepted")


def diagnostic_snapshot(validator, stamp: int = 1):
    return {
        "stamp_ns": stamp,
        "duplicate_keys": [],
        "values": {
            "projector_type": validator.EXPECTED_PROJECTOR,
            "vertical_datum": validator.EXPECTED_VERTICAL_DATUM,
            "projection_mode": validator.EXPECTED_PROJECTION_MODE,
            "map_origin_latitude": "35.68123600",
            "map_origin_longitude": "139.76712500",
            "scale_factor": "0.9996000",
            "has_last_primary": "true",
            "has_last_output": "true",
            "last_parse_error": "none",
        },
    }


def precision_guard_snapshot(
    validator,
    stamp: int = 1,
    accepted: int = 1,
    *,
    state: str | None = None,
    trusted_variance: float = 0.01,
    active_reference_variance: float | None = None,
    applied_offset: float = 0.0,
    target_offset: float = 0.0,
    outage_count: int = 0,
    active_reference_epoch: int | None = None,
    recovery_count: int = 0,
    reset_count: int = 1,
    reason: str = "trusted_yaw_reference_ready",
):
    if active_reference_epoch is None:
        active_reference_epoch = 0 if outage_count == 0 else 1
    if state is None:
        state = "DISARMED" if accepted == 0 else "READY"
    active = state in validator.OUTAGE_YAW_GUARD_ACTIVE_STATES
    if active_reference_variance is None:
        active_reference_variance = trusted_variance if active else math.nan
    if state in validator.OUTAGE_YAW_GUARD_OUTAGE_STATES:
        residual = (
            target_offset - applied_offset + math.pi
        ) % (2.0 * math.pi) - math.pi
        additional_variance = active_reference_variance + residual * residual
    elif state == "RECOVERY_RELEASE":
        additional_variance = (
            active_reference_variance + applied_offset * applied_offset
        )
    else:
        additional_variance = 0.0
    encoded_trusted_variance = (
        "nan" if state == "DISARMED" else str(trusted_variance)
    )
    values = {
        "fallback.gnss_position_enabled": "false",
        "outage_yaw_guard.enabled": "true",
        "outage_yaw_guard.state": state,
        "outage_yaw_guard.active": str(active).lower(),
        "outage_yaw_guard.reference_source": (
            validator.EXPECTED_OUTAGE_YAW_GUARD_REFERENCE_SOURCE
        ),
        "outage_yaw_guard.propagation_source": (
            validator.EXPECTED_OUTAGE_YAW_GUARD_PROPAGATION_SOURCE
        ),
        "outage_yaw_guard.xy_policy": (
            validator.EXPECTED_OUTAGE_YAW_GUARD_XY_POLICY
        ),
        "outage_yaw_guard.config.required_fix_quality": str(
            validator.EXPECTED_OUTAGE_YAW_GUARD_REQUIRED_FIX_QUALITY
        ),
        **{
            f"outage_yaw_guard.config.{key}": str(value)
            for key, value in validator.EXPECTED_OUTAGE_YAW_GUARD_LIMITS.items()
        },
        "outage_yaw_guard.trusted_variance_rad2": encoded_trusted_variance,
        "outage_yaw_guard.active_reference_variance_rad2": (
            "nan"
            if math.isnan(active_reference_variance)
            else str(active_reference_variance)
        ),
        "outage_yaw_guard.applied_offset_rad": str(applied_offset),
        "outage_yaw_guard.target_offset_rad": str(target_offset),
        "outage_yaw_guard.additional_variance_rad2": str(additional_variance),
        "outage_yaw_guard.last_reason": reason,
        "outage_yaw_guard.accepted_reference_count": str(accepted),
        "outage_yaw_guard.outage_count": str(outage_count),
        "outage_yaw_guard.active_reference_epoch": str(
            active_reference_epoch
        ),
        "outage_yaw_guard.recovery_count": str(recovery_count),
        "outage_yaw_guard.reset_count": str(reset_count),
        "outage_yaw_guard.invalid_advance_count": "0",
        "publish.global_suppressed_yaw_guard_invalid": "0",
    }
    return {
        "stamp_ns": stamp,
        "duplicate_keys": [],
        "key_counts": {key: 1 for key in values},
        "values": values,
    }


def publication_snapshot(
    validator,
    record_index: int,
    stamp: int,
    *,
    dropped: int = 0,
    covered: int = 0,
    wall_timer: int = 0,
):
    drop_key, covered_key, wall_key, total_key = (
        validator.MAP_FUSION_PUBLICATION_COUNTER_KEYS
    )
    values = {
        drop_key: str(dropped),
        covered_key: str(covered),
        wall_key: str(wall_timer),
        total_key: str(dropped + covered + wall_timer),
    }
    return {
        "record_index": record_index,
        "stamp_ns": stamp,
        "duplicate_keys": [],
        "key_counts": {key: 1 for key in values},
        "values": values,
    }


def publication_integrity_fixture(validator):
    raw = [
        {"record_index": 15, "stamp_ns": 110},
        # DDS/recorder interleave may reorder source stamps even though the bag
        # record sequence itself is ordered. Exact output coverage remains the
        # publication contract.
        {"record_index": 21, "stamp_ns": 130},
        {"record_index": 40, "stamp_ns": 120},
    ]
    existing = [
        {"record_index": 10, "stamp_ns": 100},
        {"record_index": 16, "stamp_ns": 110},
        # A timer publication may causally precede the same-stamp odometry
        # callback; its later callback is report-only coalescing, not a drop.
        {"record_index": 20, "stamp_ns": 120},
        {"record_index": 41, "stamp_ns": 130},
    ]
    snapshots = [
        publication_snapshot(validator, 5, 1),
        publication_snapshot(validator, 30, 2, covered=1),
        publication_snapshot(validator, 50, 3, covered=1, wall_timer=2),
    ]
    return raw, existing, snapshots


def test_runtime_diagnostic_contract(validator) -> None:
    snapshots = [diagnostic_snapshot(validator, 1), diagnostic_snapshot(validator, 2)]
    assert validator.nmea_diagnostic_contract(snapshots)["valid"]

    wrong_origin = json.loads(json.dumps(snapshots))
    wrong_origin[-1]["values"]["map_origin_longitude"] = "136.82260070"
    summary = validator.nmea_diagnostic_contract(wrong_origin)
    assert not summary["valid"]
    assert summary["configuration_mismatch_count"] == 1

    parse_error = json.loads(json.dumps(snapshots))
    parse_error[0]["values"]["last_parse_error"] = "checksum_mismatch"
    assert not validator.nmea_diagnostic_contract(parse_error)["valid"]
    backstep = [diagnostic_snapshot(validator, 2), diagnostic_snapshot(validator, 1)]
    assert not validator.nmea_diagnostic_contract(backstep)["valid"]


def test_precision_guard_runtime_contract_is_fail_closed(validator) -> None:
    snapshots = [
        precision_guard_snapshot(validator, 1, 0),
        precision_guard_snapshot(validator, 2, 3),
    ]
    summary = validator.precision_guard_diagnostic_contract(snapshots)
    assert summary["valid"]
    assert summary["accepted_reference_final"] == 3
    assert summary["invalid_advance_maximum"] == 0
    assert summary["suppressed_invalid_maximum"] == 0

    covariance_timeline = [
        precision_guard_snapshot(validator, 1, 2, state="READY"),
        precision_guard_snapshot(
            validator,
            2,
            2,
            state="OUTAGE_SLEW",
            active_reference_variance=0.01,
            applied_offset=0.02,
            target_offset=0.08,
            outage_count=1,
        ),
        precision_guard_snapshot(
            validator,
            3,
            3,
            state="RECOVERY_RELEASE",
            trusted_variance=0.008,
            active_reference_variance=0.01,
            applied_offset=0.04,
            outage_count=1,
            recovery_count=1,
        ),
        precision_guard_snapshot(
            validator,
            4,
            4,
            state="RECOVERY_RELEASE",
            trusted_variance=0.006,
            active_reference_variance=0.01,
            applied_offset=0.02,
            outage_count=1,
            recovery_count=1,
        ),
        precision_guard_snapshot(
            validator,
            5,
            4,
            state="READY",
            trusted_variance=0.006,
            outage_count=1,
            recovery_count=1,
        ),
    ]
    covariance_summary = validator.precision_guard_diagnostic_contract(
        covariance_timeline
    )
    assert covariance_summary["valid"], covariance_summary["mismatch_examples"]
    assert covariance_summary["covariance_issue_sample_count"] == 0
    assert math.isclose(
        covariance_summary["maximum_active_reference_variance_rad2"], 0.01
    )

    def covariance_mutation(index: int, key: str, value: str):
        altered = json.loads(json.dumps(covariance_timeline))
        altered[index]["values"][key] = value
        return altered

    active_variance_key = "outage_yaw_guard.active_reference_variance_rad2"
    missing_active_variance = json.loads(json.dumps(covariance_timeline))
    del missing_active_variance[1]["values"][active_variance_key]
    missing_active_variance[1]["key_counts"][active_variance_key] = 0
    assert not validator.precision_guard_diagnostic_contract(
        missing_active_variance
    )["valid"]
    duplicated_active_variance = json.loads(json.dumps(covariance_timeline))
    duplicated_active_variance[1]["key_counts"][active_variance_key] = 2
    assert not validator.precision_guard_diagnostic_contract(
        duplicated_active_variance
    )["valid"]
    for malformed in ("nan", "inf", "-0.001"):
        assert not validator.precision_guard_diagnostic_contract(
            covariance_mutation(1, active_variance_key, malformed)
        )["valid"]
    assert not validator.precision_guard_diagnostic_contract(
        covariance_mutation(0, active_variance_key, "0.01")
    )["valid"]
    assert not validator.precision_guard_diagnostic_contract(
        covariance_mutation(
            1, "outage_yaw_guard.additional_variance_rad2", "0.0036"
        )
    )["valid"]
    assert not validator.precision_guard_diagnostic_contract(
        covariance_mutation(
            2, "outage_yaw_guard.additional_variance_rad2", "0.0016"
        )
    )["valid"]
    assert not validator.precision_guard_diagnostic_contract(
        covariance_mutation(1, "outage_yaw_guard.outage_count", "0")
    )["valid"]
    assert not validator.precision_guard_diagnostic_contract(
        covariance_mutation(2, "outage_yaw_guard.recovery_count", "0")
    )["valid"]

    refreshed_snapshot = json.loads(json.dumps(covariance_timeline))
    refreshed_snapshot[3]["values"].update(
        {
            active_variance_key: "0.006",
            "outage_yaw_guard.additional_variance_rad2": "0.0064",
        }
    )
    assert not validator.precision_guard_diagnostic_contract(
        refreshed_snapshot
    )["valid"]

    reoutage = precision_guard_snapshot(
        validator,
        4,
        4,
        state="OUTAGE_HOLD",
        trusted_variance=0.012,
        active_reference_variance=0.012,
        applied_offset=0.04,
        target_offset=0.04,
        outage_count=2,
        recovery_count=1,
        reason="trusted_outage_yaw_reentered_during_release",
    )
    reoutage_timeline = [*covariance_timeline[:3], reoutage]
    assert validator.precision_guard_diagnostic_contract(reoutage_timeline)[
        "valid"
    ]
    understated_reoutage = json.loads(json.dumps(reoutage_timeline))
    understated_reoutage[-1]["values"].update(
        {
            active_variance_key: "0.011",
            "outage_yaw_guard.additional_variance_rad2": "0.011",
        }
    )
    assert not validator.precision_guard_diagnostic_contract(
        understated_reoutage
    )["valid"]

    for retain_reason in sorted(
        validator.OUTAGE_YAW_GUARD_REOUTAGE_RETAIN_REASONS
    ):
        retained = precision_guard_snapshot(
            validator,
            4,
            4,
            state="OUTAGE_HOLD",
            trusted_variance=0.012,
            active_reference_variance=0.01,
            applied_offset=0.04,
            target_offset=0.04,
            outage_count=2,
            recovery_count=1,
            reason=retain_reason,
        )
        retained_timeline = [*covariance_timeline[:3], retained]
        assert validator.precision_guard_diagnostic_contract(retained_timeline)[
            "valid"
        ], retain_reason
        overstated_retained = json.loads(json.dumps(retained_timeline))
        overstated_retained[-1]["values"].update(
            {
                active_variance_key: "0.012",
                "outage_yaw_guard.additional_variance_rad2": "0.012",
            }
        )
        assert not validator.precision_guard_diagnostic_contract(
            overstated_retained
        )["valid"]

    fresh_with_lower_trusted = precision_guard_snapshot(
        validator,
        4,
        4,
        state="OUTAGE_HOLD",
        trusted_variance=0.008,
        active_reference_variance=0.01,
        applied_offset=0.04,
        target_offset=0.04,
        outage_count=2,
        recovery_count=1,
        reason="trusted_outage_yaw_reentered_during_release",
    )
    assert validator.precision_guard_diagnostic_contract(
        [*covariance_timeline[:3], fresh_with_lower_trusted]
    )["valid"]

    hidden_reoutage = precision_guard_snapshot(
        validator,
        4,
        4,
        state="OUTAGE_HOLD",
        trusted_variance=0.012,
        active_reference_variance=0.012,
        applied_offset=0.04,
        target_offset=0.04,
        outage_count=2,
        recovery_count=1,
        reason="trusted_outage_yaw_held",
    )
    hidden_reoutage_timeline = [covariance_timeline[1], hidden_reoutage]
    assert validator.precision_guard_diagnostic_contract(
        hidden_reoutage_timeline
    )["valid"]
    hidden_understated = json.loads(json.dumps(hidden_reoutage_timeline))
    hidden_understated[-1]["values"].update(
        {
            active_variance_key: "0.009",
            "outage_yaw_guard.additional_variance_rad2": "0.009",
        }
    )
    assert not validator.precision_guard_diagnostic_contract(
        hidden_understated
    )["valid"]

    cross_epoch = json.loads(json.dumps(hidden_understated))
    cross_epoch[-1]["values"].update(
        {
            "outage_yaw_guard.trusted_variance_rad2": "0.009",
            "outage_yaw_guard.active_reference_epoch": "2",
        }
    )
    cross_epoch_summary = validator.precision_guard_diagnostic_contract(
        cross_epoch
    )
    assert cross_epoch_summary["valid"], cross_epoch_summary[
        "mismatch_examples"
    ]
    assert cross_epoch_summary["active_reference_epoch_accounting"] == {
        "available": True,
        "mixed_presence": False,
        "unproven_intervals": 0,
        "cross_epoch_intervals": 1,
    }

    legacy_understated = json.loads(json.dumps(hidden_understated))
    for snapshot in legacy_understated:
        del snapshot["values"]["outage_yaw_guard.active_reference_epoch"]
        snapshot["key_counts"].pop(
            "outage_yaw_guard.active_reference_epoch", None
        )
    legacy_summary = validator.precision_guard_diagnostic_contract(
        legacy_understated
    )
    assert legacy_summary["valid"], legacy_summary["mismatch_examples"]
    assert legacy_summary["active_reference_epoch_accounting"] == {
        "available": False,
        "mixed_presence": False,
        "unproven_intervals": 1,
        "cross_epoch_intervals": 0,
    }
    legacy_check = validator.Checks()
    legacy_check.add(
        "legacy active-reference epoch accounting",
        False,
        "N/A: unproven",
        warning=True,
    )
    assert legacy_check.passed
    assert legacy_check.items[0]["warning"]
    warning_markdown = validator.markdown(
        {
            "summary": {
                "passed": True,
                "check_count": 1,
                "failed_check_count": 0,
            },
            "dataset": {"key": "legacy"},
            "expected_mode": "precision",
            "run_directory": "legacy-run",
            "checks": legacy_check.items,
        }
    )
    assert "| WARN | N/A: unproven |" in warning_markdown

    mixed_epoch = json.loads(json.dumps(hidden_reoutage_timeline))
    del mixed_epoch[0]["values"]["outage_yaw_guard.active_reference_epoch"]
    mixed_epoch[0]["key_counts"].pop(
        "outage_yaw_guard.active_reference_epoch", None
    )
    assert not validator.precision_guard_diagnostic_contract(mixed_epoch)[
        "valid"
    ]

    impossible_epoch = json.loads(json.dumps(hidden_reoutage_timeline))
    impossible_epoch[-1]["values"][
        "outage_yaw_guard.active_reference_epoch"
    ] = "3"
    assert not validator.precision_guard_diagnostic_contract(impossible_epoch)[
        "valid"
    ]
    hidden_bad_counters = json.loads(json.dumps(hidden_reoutage_timeline))
    hidden_bad_counters[-1]["values"][
        "outage_yaw_guard.recovery_count"
    ] = "0"
    assert not validator.precision_guard_diagnostic_contract(
        hidden_bad_counters
    )["valid"]

    hidden_release_cycle = precision_guard_snapshot(
        validator,
        4,
        4,
        state="RECOVERY_RELEASE",
        trusted_variance=0.012,
        active_reference_variance=0.012,
        applied_offset=0.02,
        outage_count=2,
        recovery_count=2,
        reason="outage_yaw_recovery_release_step",
    )
    hidden_release_summary = validator.precision_guard_diagnostic_contract(
        [covariance_timeline[2], hidden_release_cycle]
    )
    assert hidden_release_summary["valid"], hidden_release_summary[
        "mismatch_examples"
    ]

    outage_through_multiple_cycles = precision_guard_snapshot(
        validator,
        3,
        4,
        state="RECOVERY_RELEASE",
        trusted_variance=0.012,
        active_reference_variance=0.012,
        applied_offset=0.04,
        outage_count=3,
        recovery_count=3,
        reason="outage_yaw_recovery_release_started",
    )
    multiple_cycle_summary = validator.precision_guard_diagnostic_contract(
        [covariance_timeline[1], outage_through_multiple_cycles]
    )
    assert multiple_cycle_summary["valid"], multiple_cycle_summary[
        "mismatch_examples"
    ]

    negative_endpoint_balance = json.loads(
        json.dumps(outage_through_multiple_cycles)
    )
    negative_endpoint_balance["values"].update(
        {
            active_variance_key: "0.01",
            "outage_yaw_guard.additional_variance_rad2": "0.0116",
            "outage_yaw_guard.outage_count": "1",
            "outage_yaw_guard.recovery_count": "0",
        }
    )
    negative_balance_summary = validator.precision_guard_diagnostic_contract(
        [covariance_timeline[1], negative_endpoint_balance]
    )
    assert not negative_balance_summary["valid"]
    assert any(
        "outage_yaw_guard.outage_recovery_endpoint_balance"
        in example["issues"]
        and example["issues"][
            "outage_yaw_guard.outage_recovery_endpoint_balance"
        ]["expected_outage_count_delta"]
        == -1
        for example in negative_balance_summary["mismatch_examples"]
    )

    mutations = (
        ("fallback.gnss_position_enabled", "true"),
        ("outage_yaw_guard.enabled", "false"),
        ("outage_yaw_guard.reference_source", "untrusted_yaw"),
        ("outage_yaw_guard.propagation_source", "raw_odom_yaw"),
        ("outage_yaw_guard.xy_policy", "guard_mutates_xy"),
        ("outage_yaw_guard.config.required_fix_quality", "5"),
        ("outage_yaw_guard.config.max_trusted_age_sec", "2.1"),
        ("outage_yaw_guard.invalid_advance_count", "1"),
        ("publish.global_suppressed_yaw_guard_invalid", "1"),
    )
    for key, value in mutations:
        altered = json.loads(json.dumps(snapshots))
        altered[-1]["values"][key] = value
        assert not validator.precision_guard_diagnostic_contract(altered)["valid"]

    no_reference = [
        precision_guard_snapshot(validator, 1, 0),
        precision_guard_snapshot(validator, 2, 0),
    ]
    assert not validator.precision_guard_diagnostic_contract(no_reference)["valid"]
    counter_backstep = [
        precision_guard_snapshot(validator, 1, 3),
        precision_guard_snapshot(validator, 2, 2),
    ]
    assert not validator.precision_guard_diagnostic_contract(counter_backstep)[
        "valid"
    ]
    missing = json.loads(json.dumps(snapshots))
    del missing[-1]["values"]["outage_yaw_guard.xy_policy"]
    assert not validator.precision_guard_diagnostic_contract(missing)["valid"]
    duplicate = json.loads(json.dumps(snapshots))
    duplicate[-1]["duplicate_keys"] = ["outage_yaw_guard.enabled"]
    assert not validator.precision_guard_diagnostic_contract(duplicate)["valid"]
    timestamp_backstep = [
        precision_guard_snapshot(validator, 2, 1),
        precision_guard_snapshot(validator, 1, 2),
    ]
    assert not validator.precision_guard_diagnostic_contract(timestamp_backstep)[
        "valid"
    ]


def test_counter_contract_is_fail_closed(validator) -> None:
    def counter(status: str, key: str, maximum: int = 0):
        return {
            "status": status,
            "key": key,
            "samples": 2,
            "maximum": maximum,
            "malformed_values": [],
        }

    counters = {
        "matcher:malformed": counter(validator.MATCHER_STATUS, "malformed_count"),
        "global:backstep": counter(
            validator.PRECISION_GLOBAL_STATUS, "raw.backstep"
        ),
        "global:nonmonotonic": counter(
            validator.PRECISION_GLOBAL_STATUS, "raw.nonmonotonic"
        ),
    }
    assert validator.counter_contract(counters, "precision")["valid"]
    missing = dict(counters)
    del missing["global:backstep"]
    assert not validator.counter_contract(missing, "precision")["valid"]
    nonzero = json.loads(json.dumps(counters))
    nonzero["global:backstep"]["maximum"] = 1
    assert not validator.counter_contract(nonzero, "precision")["valid"]
    malformed = json.loads(json.dumps(counters))
    malformed["matcher:malformed"]["malformed_values"] = ["not-a-number"]
    assert not validator.counter_contract(malformed, "precision")["valid"]


def test_map_fusion_publication_integrity_is_fail_closed(validator) -> None:
    raw, existing, snapshots = publication_integrity_fixture(validator)
    summary = validator.map_fusion_publication_integrity_contract(
        raw, existing, snapshots
    )
    assert summary["valid"]
    assert summary["strict_drop_final"] == 0
    assert summary["covered_odometry_coalesced_final"] == 1
    assert summary["wall_timer_coalesced_final"] == 2
    assert summary["causal_raw_stamp_coverage"]["raw_request_count"] == 3
    assert not summary["causal_raw_stamp_coverage"][
        "raw_source_stamp_monotonic_report_only"
    ]
    assert (
        summary["causal_raw_stamp_coverage"][
            "exactly_covered_raw_unique_stamp_count"
        ]
        == 3
    )

    # Raw odometry outside the closed audit window is intentionally ignored:
    # no later diagnostic exists to attest what happened to that request.
    after_final = json.loads(json.dumps(raw))
    after_final.append({"record_index": 60, "stamp_ns": 140})
    assert validator.map_fusion_publication_integrity_contract(
        after_final, existing, snapshots
    )["valid"]

    drop_key, covered_key, wall_key, total_key = (
        validator.MAP_FUSION_PUBLICATION_COUNTER_KEYS
    )
    counter_mutations = []

    missing = json.loads(json.dumps(snapshots))
    missing[-1]["key_counts"][wall_key] = 0
    missing[-1]["values"][wall_key] = None
    counter_mutations.append(missing)

    duplicated = json.loads(json.dumps(snapshots))
    duplicated[-1]["key_counts"][covered_key] = 2
    counter_mutations.append(duplicated)

    noncanonical = json.loads(json.dumps(snapshots))
    noncanonical[-1]["values"][covered_key] = "1.0"
    counter_mutations.append(noncanonical)

    negative = json.loads(json.dumps(snapshots))
    negative[-1]["values"][wall_key] = "-1"
    counter_mutations.append(negative)

    backstep = json.loads(json.dumps(snapshots))
    backstep[1]["values"][covered_key] = "2"
    backstep[1]["values"][total_key] = "2"
    counter_mutations.append(backstep)

    wrong_sum = json.loads(json.dumps(snapshots))
    wrong_sum[-1]["values"][total_key] = "4"
    counter_mutations.append(wrong_sum)

    real_drop = json.loads(json.dumps(snapshots))
    real_drop[-1]["values"][drop_key] = "1"
    real_drop[-1]["values"][total_key] = "4"
    counter_mutations.append(real_drop)

    for altered in counter_mutations:
        assert not validator.map_fusion_publication_integrity_contract(
            raw, existing, altered
        )["valid"]

    missing_output = json.loads(json.dumps(existing))
    missing_output.pop()
    missing_summary = validator.map_fusion_publication_integrity_contract(
        raw, missing_output, snapshots
    )
    assert not missing_summary["valid"]
    assert (
        missing_summary["causal_raw_stamp_coverage"][
            "missing_raw_stamp_examples_ns"
        ][0]
        == 130
    )

    # An exact output recorded only after the final diagnostic is not causal
    # evidence for the diagnostic-bounded request window.
    too_late = json.loads(json.dumps(existing))
    too_late[-1]["record_index"] = 51
    assert not validator.map_fusion_publication_integrity_contract(
        raw, too_late, snapshots
    )["valid"]

    existing_backstep = json.loads(json.dumps(existing))
    existing_backstep[2]["stamp_ns"] = 140
    existing_backstep[3]["stamp_ns"] = 130
    assert not validator.map_fusion_publication_integrity_contract(
        raw, existing_backstep, snapshots
    )["valid"]

    raw_record_backstep = json.loads(json.dumps(raw))
    raw_record_backstep[1]["record_index"] = 14
    assert not validator.map_fusion_publication_integrity_contract(
        raw_record_backstep, existing, snapshots
    )["valid"]

    noncanonical_raw = json.loads(json.dumps(raw))
    noncanonical_raw[1]["stamp_canonical"] = False
    assert not validator.map_fusion_publication_integrity_contract(
        noncanonical_raw, existing, snapshots
    )["valid"]

    raw_with_zero = json.loads(json.dumps(raw))
    raw_with_zero.insert(1, {"record_index": 18, "stamp_ns": 0})
    zero_summary = validator.map_fusion_publication_integrity_contract(
        raw_with_zero, existing, snapshots
    )
    assert zero_summary["valid"]
    assert zero_summary["causal_raw_stamp_coverage"][
        "raw_zero_stamp_excluded_count"
    ] == 1

    diagnostic_backstep = json.loads(json.dumps(snapshots))
    diagnostic_backstep[-1]["stamp_ns"] = 1
    assert not validator.map_fusion_publication_integrity_contract(
        raw, existing, diagnostic_backstep
    )["valid"]


def main() -> int:
    validator = load_validator()
    validator.run_self_test()
    test_run_env_parser(validator)
    test_manifest_and_mode_contracts(validator)
    test_default_projection_contract(validator)
    test_precision_global_parameter_contract(validator)
    test_runtime_diagnostic_contract(validator)
    test_precision_guard_runtime_contract_is_fail_closed(validator)
    test_counter_contract_is_fail_closed(validator)
    test_map_fusion_publication_integrity_is_fail_closed(validator)
    print("Hesai GNSS publication validator helper tests: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
