#!/usr/bin/env python3
"""Build small, publication-ready evaluation assets from local result artifacts.

The source bags, full CSV files, and logs intentionally remain outside Git.  This
script extracts only normalized metrics and renders compact PNG figures whose
labels describe the sensor setup rather than local bag nicknames.
"""

from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
from typing import Any

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / "docs" / "evaluation" / "assets"

VELODYNE_LABEL = "Velodyne 32-Line + External IMU — LiDAR/IMU-Only (No GNSS)"
MID360_LABEL = "Livox MID-360 + Internal IMU — LiDAR/IMU-Only (No GNSS)"
MID360_PLOT_TITLE = "MID-360 + Internal IMU (No GNSS)"
COURSE_LABELS = {
    "course_2": "Hesai 32-Line + IMU + RTK GNSS — Course 2",
}

MID360_RESULT_SET = "mid360_internal_imu_tuning_20260814_final"
MID360_RESULT_ROOT = ROOT / "test_results" / MID360_RESULT_SET
MID360_DATASET_ID = "livox_mid360_internal_imu"
MID360_RETIRED_DATASET_ID = "livox_mid360_external_imu"
MID360_GLIM_TRAJECTORY_SHA256 = (
    "ae9d148a098236251ee79ba8e17e3d18bd664af5adffa5b6ef025f274c50aa6c"
)
MID360_SOURCE_CONTRACT: dict[str, tuple[str, str]] = {
    "final_status": (
        "final/FINAL_STATUS.json",
        "8cc66bf6807f0889b416e8461483126db003104a9b0379fe31d695537d0895be",
    ),
    "status_revalidation": (
        "final/FINAL_STATUS_REVALIDATION.json",
        "3cc0d026b6c24cdad83ea05f6a936a099051ee189a143506435d9c052292d228",
    ),
    "evaluation": (
        "final/ab_initial_pose/evaluation.json",
        "875d431c4351541ea3243e281f1d5c79840ed2d00a979e847e5b670e3f2175a9",
    ),
    "aligned_samples": (
        "final/ab_initial_pose/aligned_samples.csv",
        "010decd3d41be904f73ba64eb5cb78bd8f3a88858d82ef10fe56cdf9c457bbfe",
    ),
    "holdout_metrics": (
        "final/holdout_metrics.json",
        "1cc1e1814fa96932e7331e9516ad57c497e1babc11a9f422122d3b0fc155b05e",
    ),
    "normalized_plan": (
        "plan.normalized.json",
        "b9892698bf38092002c422a197c5c8fd9862fc82c348ead928b1e2edc65c3942",
    ),
    "control_run_environment": (
        "final/scan_to_scan/run.env",
        "72f16f7fce5b9225099bc21da81a15ef081eb1d9de5331181a555f3e8ef39d00",
    ),
    "control_runtime_metrics": (
        "final/scan_to_scan/runtime_analysis/runtime_metrics.json",
        "ce60e426459cf7b8f8ab0ce7027027d61d7a8bf0541c817c201d2497110ca936",
    ),
    "control_lidar_transform_log": (
        "final/scan_to_scan/tf_livox_frame.log",
        "7c794994d90eefd762895849dcd35bbfc69a8159e6cda1914bdf4dc7115ee181",
    ),
    "control_imu_parameters": (
        "final/scan_to_scan/artifacts/imu_param.yaml",
        "6ae5389fb60b192dcc7509343e37d2d9df83049ad898822e42f517cf6af83700",
    ),
    "control_odometry_override": (
        "final/scan_to_scan/artifacts/odom_override.yaml",
        "8c5e3e6c66502fe963ba72224655da1c91fd0ce1c607e33cf1058249717f8504",
    ),
    "control_odometry_tuning": (
        "final/scan_to_scan/artifacts/odom_tuning_override.yaml",
        "465101aeb98d0eefddd2f41d6baf56e821b5b939b5e49127535bb78320ddeefc",
    ),
    "precision_run_environment": (
        "final/scan_to_submap/run.env",
        "a8928a44e33308e59a17167fde2cfabed7014226869adcd02bd37b66979cc067",
    ),
    "precision_runtime_metrics": (
        "final/scan_to_submap/runtime_analysis/runtime_metrics.json",
        "8501ec8cafdaeab7dd4c59e47c88c7bcfa4ca1b5c04cace1d64e4a91db40e9a1",
    ),
    "precision_structural_validation": (
        "final/scan_to_submap/submap_validation/validation.json",
        "e68f02299fea4a1251a32c579d52e7ba2aeb6a0722e5a37d246389059c2bcdef",
    ),
    "precision_lidar_transform_log": (
        "final/scan_to_submap/tf_livox_frame.log",
        "aca3fb9405f7c624ce45ce26cb6b5f0c30ea7d722ccce6c5f4279a4e746960f4",
    ),
    "precision_imu_parameters": (
        "final/scan_to_submap/artifacts/imu_param.yaml",
        "6ae5389fb60b192dcc7509343e37d2d9df83049ad898822e42f517cf6af83700",
    ),
    "precision_odometry_override": (
        "final/scan_to_submap/artifacts/odom_override.yaml",
        "8c5e3e6c66502fe963ba72224655da1c91fd0ce1c607e33cf1058249717f8504",
    ),
    "precision_odometry_tuning": (
        "final/scan_to_submap/artifacts/odom_tuning_override.yaml",
        "465101aeb98d0eefddd2f41d6baf56e821b5b939b5e49127535bb78320ddeefc",
    ),
    "precision_snapshot_parameters": (
        "final/scan_to_submap/artifacts/odom_aux_override.yaml",
        "537eb9636a7ff7312ee72af74ba8bbe5f8fd7b71a0b5a088d8f4efc5a7935c88",
    ),
    "precision_matcher_parameters": (
        "final/scan_to_submap/artifacts/submap_matcher_param.yaml",
        "e4618d3eb5070b1890a15a3822423cda7ad54fad58e7eedb1dae36433d8a0aa7",
    ),
    "precision_matcher_tuning": (
        "final/scan_to_submap/artifacts/submap_matcher_override.yaml",
        "7db51bec16ed2a4bad41857bf51dde33ae04c08efa94b0ef90f614341f39f821",
    ),
}
MID360_STAGE_SUMMARY_CONTRACT: dict[str, str] = {
    "fixed_bias": "2fa215fdb37124ada3f1c4a66c1627a40cac3e0c7aeee9416c82c4511dea757b",
    "lidar_filter": "5af50da7ec378e5beaa37ba2b9fb2af1d32f2554bf8e3103b91a5f56df2edef5",
    "scan_acceptance": "652624898e9f207be4334e39dc67386579c9ceba5af9274e937678fae56dfa32",
    "scan_vgicp": "a1708766dd251721bce48542490e8b6a80a6904b1d75e59d630a338d32d7c28d",
    "smoother": "c459243b12fc7d81838a2fcb2f5feb1b95fc26027ad8094319e8bd9f6c288c63",
    "snapshot_cadence": "9fa22956b8e2a869e4f25826368b076dc3ba4b10f1484cd1b41bcb51efc4c500",
    "stop_detection": "6f4f84b814c5df9c0d7fa117e0fddbd98945f6d34d90fdc09e663856002b044f",
    "submap_correction": "c67483b1257919d13139269d04c9f83c674ad84a8712a0c957f44add90d0a5b3",
    "submap_geometry": "8a245ccdcf13404cd64628be6781fa799f05eb1a28ce828ba0496fba091847ef",
}
MID360_PUBLICATION_FILENAMES = {
    "metrics.json",
    "trajectory.png",
    "xy_error.png",
    "yaw_error.png",
}

GNSS_RESULT_SET = "gnss_outage_yaw_guard_20260812_final"
GNSS_RESULT_ROOT = ROOT / "test_results" / GNSS_RESULT_SET
GNSS_DEFAULT_PROJECTION = {
    "projector_type": "TransverseMercator",
    "vertical_datum": "WGS84",
    "latitude_deg": 35.681236,
    "longitude_deg": 139.767125,
    "altitude_m": 0.0,
    "scale_factor": 0.9996,
}
GNSS_OUTAGE_YAW_GUARD_CONFIG = {
    "required_fix_quality": 4.0,
    "max_trusted_age_sec": 2.0,
    "max_trusted_variance_rad2": 0.0225,
    "max_trusted_delta_rad": 0.35,
    "max_offset_rate_radps": 0.2,
    "max_offset_step_rad": 0.04,
    "max_step_dt_sec": 0.25,
}
GNSS_OUTAGE_YAW_GUARD_SEMANTICS = {
    "reference_source": "robust_gnss_position_alignment_yaw",
    "propagation_source": "precision_local_yaw",
    "xy_policy": "existing_fusion_anchor_compose_precision_local",
}
GNSS_OUTAGE_YAW_GUARD_HARD_GATES = (
    "outage yaw guard diagnostics present and enabled",
    "outage yaw guard diagnostic fields and counters valid",
    "outage yaw guard configuration contract",
    "outage yaw guard source and XY policy contract",
    "outage yaw guard accepted trusted references",
    "outage yaw guard exercises outage and recovery",
    "outage yaw guard has no invalid or suppressed output",
    "outage yaw guard state agrees with fusion authority",
    "outage yaw guard composition and bounded continuity",
)
GNSS_RUNTIME_PASS_LABELS = (
    "required topics",
    "exact correction keys",
    "correction full-SE2 contract",
    "dedicated precision frames",
    "physical scan stream order",
    "raw odom stamp order after clock initialization",
    "precision local stamp order after clock initialization",
    "precision local intentional startup zero stamps",
    "precision global stamp order after clock initialization",
    "precision global intentional startup zero stamps",
    "intentional initialization coverage",
    "map-fusion publication counter integrity",
    "map-fusion exact raw stamp publication coverage",
    "odometer isolated snapshot contract",
    "matcher stream health",
    "matcher diagnostic counts match recording",
    "matcher latency p99",
    "precision global diagnostics",
    "existing-fusion global authority",
    "startup readiness diagnostic schema",
    "existing-fusion anchor freezes outside strict health",
    "outage yaw guard runtime contract",
    "existing-fusion session rearm requires an explicit fresh health edge",
    "startup readiness transitions",
    "startup global publication safety",
    "startup global suppression exercised",
    "playback rate",
)
GNSS_RUNTIME_INTENTIONAL_WARNING_LABEL = (
    "raw odom intentional startup zero stamps"
)
LSIM_CURRENT_EVIDENCE_SCOPE = ["interface", "runtime"]
LSIM_POSTER_FILENAME = "rviz_poster.png"
LSIM_POSTER_SHA256 = (
    "2d944f70f3b352ef0014e94428fb96ac89b107cbfe3171ac581ab394be5aaea2"
)
LSIM_REPLAY_FILENAME = "rviz_replay.webm"
LSIM_REPLAY_SHA256 = (
    "c8762b5a264f49b14240eea01760c955c98e45596000a00b6ae6bd21fe2739e3"
)
LSIM_REPLAY_BYTES = 62_245_501
LSIM_REPLAY_DURATION_SEC = 75.250293115
LSIM_REPLAY_RESOLUTION = [898, 539]
LSIM_REPLAY_FRAME_COUNT = 2_138
LSIM_PUBLICATION_FILENAMES = {
    "metrics.json",
    LSIM_POSTER_FILENAME,
    LSIM_REPLAY_FILENAME,
}

# ``None`` hashes keep LSim regeneration fail closed until the public
# headless run finishes and its report has been reviewed.  Only then are the
# immutable source hashes deliberately adopted below.
LSIM_CURRENT_RUN_CONTRACTS: dict[str, dict[str, Any]] = {
    "course_2": {
        "run_id": "default_projection_course_2_20260812",
        "dataset_label": "Hesai 32-Line + IMU + RTK GNSS — Course 2",
        "sources": {
            "validation_report": (
                "validation.log",
                "cad4d75f6cdfae97d977c3f5c44f9bfadb99d1fa4e6963e392a886f2a183fa8e",
            ),
            "run_environment": (
                "run.env",
                "25d554075d6d900ccd1ba1e290f51aadb42c6254a86881a9ee28171f4d081a91",
            ),
            "effective_configurations": (
                "artifacts/effective_configurations.tsv",
                "5f0805483db85fe4d11369ce1ddc0d85b3e0c750ef65e7818b5c5b71dcff6be1",
            ),
            "nmea_parameters": (
                "artifacts/nmea_gnss_param.yaml",
                "2d04cde50870b45df6f2e913d8d6c75e331caee24e6c52fe3193129fd0f5c754",
            ),
            "map_projector_metadata": (
                "artifacts/map_projector_info.yaml",
                "39ebda9a5d35914256077cd1dc00aaed253849392fdce3f910eee27b3fe275cc",
            ),
            "nmea_override": (
                "artifacts/nmea_override_param.yaml",
                "1808b1d66570c24e5cf551c0f160f309f94c70ba258aab2d1a1746d2ed93e5ea",
            ),
            "output_metadata": (
                "localization_output/metadata.yaml",
                "a38c148cf87b5e93b562dbb0085391bc9a44aa76ea89fabc9123e0519c946805",
            ),
        },
    },
}

# These ignored local artifacts are the immutable evidence set from which the
# committed GNSS figures and metrics are curated.  Pinning every input makes
# regeneration fail closed: a rerun or manual edit must be reviewed and then
# deliberately adopted here instead of silently replacing published evidence.
#
GNSS_SOURCE_CONTRACTS: dict[str, dict[str, Any]] = {
    "course_2": {
        "publication_status": "ACCEPTED",
        "accepted": True,
        "baseline_run_id": "course_2_baseline_final",
        "control_run_id": "course_2_control_final",
        "precision_run_id": "course_2_precision_q4_guard_variance_final",
        "source_accuracy_label": COURSE_LABELS["course_2"],
        "expected_accuracy_hard_gate_count": 53,
        "expected_startup_check_count": 20,
        "expected_nonintrusion_check_count": 17,
        "expected_nonintrusion_hard_gate_count": 14,
        "expected_nonintrusion_failed_warning_count": 0,
        "expected_failed_hard_gates": [],
        "expected_error_plot_timeline": {
            "rtk_q4_unavailable_begin_time_sec": 67.32411003112793,
            "rtk_q4_unavailable_end_time_sec": 184.57655668258667,
            "rtk_q4_unavailable_duration_sec": 117.252446449,
            "fusion_tracking_resume_time_sec": 189.77501726150513,
            "fusion_tracking_resume_delay_sec": 5.198460760000001,
        },
        "expected_runtime": {
            "scans": 851,
            "corrections": 613,
            "processing_p99_ms": 54.277,
            "end_to_end_latency_p99_ms": 128.409,
            "eligible_unique_raw_stamp_count": 7892,
            "covered_odometry_coalesced_count": 0,
            "maximum_active_reference_variance_rad2": 0.00857474820591,
        },
        "sources": {
            "accuracy_result": (
                "course_2_glim_ab_variance_final_acceptance.json",
                "ccbfff228630fb47fa3d3b19f5d57e42baca3e8f16500fefcfe1315dc7b16609",
            ),
            "startup_acceptance": (
                "course_2_startup_acceptance_variance_final.json",
                "bf8b5cc1615c740e5127f1af61ee6452dc72728ae1c4edf12a61a934c029022d",
            ),
            "accepted_scan_nonintrusion": (
                "runtime_variance_final/course_2_accepted_scan_nonintrusion.json",
                "ab45e548d400b4e346c47765a03b0222de4c746f6a80c3b71fcfe554f467afd8",
            ),
            "precision_runtime_validation": (
                "runtime_variance_final/course_2_precision_runtime.log",
                "d458b9dfff5939c022c75298c3005c3d81d6f176442f83b847bdc15309949c56",
            ),
            "baseline_run_provenance": (
                "provenance_variance_final/course_2_baseline.json",
                "aa36e75e310d07c3adc6aea1945f8c3d2e465bad9413f1dd86f27a2a1435308c",
            ),
            "control_run_provenance": (
                "provenance_variance_final/course_2_control.json",
                "3a4574acb73027c66f4cc1de634b5d6b5f07f079ac3d7dbe0136bb9b0a74c89e",
            ),
            "precision_run_provenance": (
                "provenance_variance_final/course_2_precision.json",
                "4e57ec21bf8d1bb11d8583ce75f87908f8a9bd7907ee259ae68d962a28d4f9b1",
            ),
            "aligned_local_samples": (
                "plots_variance_final/course_2/hesai_32_line_imu_rtk_gnss_course_2_local_glim.csv",
                "3d8acd404975e03b6453c45bc4d91e046212b21953e2da8e96d772f83fcc42b2",
            ),
            "aligned_global_samples": (
                "plots_variance_final/course_2/hesai_32_line_imu_rtk_gnss_course_2_global_glim.csv",
                "2795f904ff0882d192080749525e089f0c692f2a18848f06e58fea67022042cf",
            ),
        },
    },
}

GNSS_PUBLICATION_FILENAMES = {
    "global_xy_error.png",
    "global_yaw_error.png",
    "local_xy_error.png",
    "local_yaw_error.png",
    "metrics.json",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def png_chunk_types(path: Path) -> list[str]:
    data = path.read_bytes()
    require(
        data[:8] == b"\x89PNG\r\n\x1a\n",
        f"invalid PNG signature: {path}",
    )
    offset = 8
    chunks: list[str] = []
    while offset < len(data):
        require(offset + 12 <= len(data), f"truncated PNG chunk: {path}")
        length = int.from_bytes(data[offset : offset + 4], "big")
        end = offset + 12 + length
        require(end <= len(data), f"truncated PNG payload: {path}")
        chunks.append(data[offset + 4 : offset + 8].decode("ascii"))
        offset = end
    require(offset == len(data), f"unexpected trailing PNG data: {path}")
    return chunks


def require_gnss_contract_adopted() -> None:
    pending: list[str] = []
    for course, contract in GNSS_SOURCE_CONTRACTS.items():
        if not isinstance(contract.get("publication_status"), str):
            pending.append(f"{course}/publication_status")
        if not isinstance(contract.get("accepted"), bool):
            pending.append(f"{course}/accepted")
        if not isinstance(contract.get("expected_failed_hard_gates"), list):
            pending.append(f"{course}/expected_failed_hard_gates")
        if not isinstance(contract.get("expected_runtime"), dict):
            pending.append(f"{course}/expected_runtime")
        for mode in ("baseline", "control", "precision"):
            run_id = contract.get(f"{mode}_run_id")
            if not isinstance(run_id, str) or not run_id:
                pending.append(f"{course}/{mode}_run_id")
        for source_id, (_, expected_hash) in contract.get("sources", {}).items():
            if not isinstance(expected_hash, str) or re.fullmatch(
                r"[0-9a-f]{64}", expected_hash
            ) is None:
                pending.append(f"{course}/sources/{source_id}/sha256")
    require(
        not pending,
        "canonical GNSS evidence has not been reviewed and adopted: "
        + ", ".join(pending),
    )


def pinned_gnss_sources(course: str) -> dict[str, Path]:
    require_gnss_contract_adopted()
    contract = GNSS_SOURCE_CONTRACTS[course]
    paths: dict[str, Path] = {}
    for source_id, (relative, expected_hash) in contract["sources"].items():
        path = GNSS_RESULT_ROOT / relative
        require(path.is_file(), f"missing pinned GNSS source: {path}")
        actual_hash = sha256(path)
        require(
            actual_hash == expected_hash,
            f"pinned GNSS source hash mismatch for {course}/{source_id}: "
            f"expected={expected_hash} actual={actual_hash}",
        )
        paths[source_id] = path
    return paths


def path_has_suffix(value: Any, suffix: str) -> bool:
    return isinstance(value, str) and Path(value).as_posix().endswith(suffix)


def validate_default_projection(value: Any, context: str) -> None:
    """Require the loaded ROS parameters and map metadata to agree exactly."""
    require(isinstance(value, dict), f"{context}: missing NMEA projection data")
    parameters = value.get("parameters")
    metadata = value.get("map_projector_metadata")
    override = value.get("evaluation_override")
    expected_origin = [
        GNSS_DEFAULT_PROJECTION["latitude_deg"],
        GNSS_DEFAULT_PROJECTION["longitude_deg"],
    ]
    require(
        isinstance(parameters, dict) and parameters.get("valid") is True,
        f"{context}: loaded NMEA parameter validation failed",
    )
    require(
        parameters.get("projector_type")
        == GNSS_DEFAULT_PROJECTION["projector_type"]
        and parameters.get("vertical_datum")
        == GNSS_DEFAULT_PROJECTION["vertical_datum"]
        and parameters.get("map_origin")
        == [*expected_origin, GNSS_DEFAULT_PROJECTION["altitude_m"]]
        and parameters.get("gnss0")
        == [*expected_origin, GNSS_DEFAULT_PROJECTION["altitude_m"]]
        and parameters.get("scale_factor")
        == GNSS_DEFAULT_PROJECTION["scale_factor"]
        and parameters.get("use_legacy_projection_params") is False,
        f"{context}: unexpected loaded NMEA projection: {parameters}",
    )
    require(
        isinstance(metadata, dict) and metadata.get("valid") is True,
        f"{context}: map projector metadata validation failed",
    )
    require(
        metadata.get("projector_type")
        == GNSS_DEFAULT_PROJECTION["projector_type"]
        and metadata.get("vertical_datum")
        == GNSS_DEFAULT_PROJECTION["vertical_datum"]
        and metadata.get("map_origin") == expected_origin
        and metadata.get("scale_factor")
        == GNSS_DEFAULT_PROJECTION["scale_factor"],
        f"{context}: unexpected map projector metadata: {metadata}",
    )
    require(
        value.get("parameters_match_metadata") is True,
        f"{context}: loaded parameters do not match map projector metadata",
    )
    require(
        isinstance(override, dict)
        and override.get("valid") is True
        and override.get("parameter_override_count") == 0,
        f"{context}: evaluation must use an empty NMEA override: {override}",
    )


def validate_map_fusion_publication_integrity(
    provenance: dict[str, Any], context: str
) -> dict[str, Any]:
    integrity = (
        provenance.get("bags", {})
        .get("diagnostics", {})
        .get("map_fusion_publication_integrity")
    )
    require(isinstance(integrity, dict), f"{context}: publication audit missing")
    coverage = integrity.get("causal_raw_stamp_coverage")
    final = integrity.get("final_counters")
    require(isinstance(coverage, dict), f"{context}: causal coverage missing")
    require(isinstance(final, dict), f"{context}: publication counters missing")
    counter_keys = {
        "output.out_of_order_drop_count",
        "output.covered_odometry_coalesced_count",
        "output.wall_timer_coalesced_count",
        "output.total_suppressed_request_count",
    }
    require(
        set(final) == counter_keys
        and all(type(value) is int and value >= 0 for value in final.values()),
        f"{context}: publication counters are incomplete or noncanonical: {final}",
    )
    strict = final["output.out_of_order_drop_count"]
    covered = final["output.covered_odometry_coalesced_count"]
    wall = final["output.wall_timer_coalesced_count"]
    total = final["output.total_suppressed_request_count"]
    require(
        integrity.get("valid") is True
        and int(integrity.get("snapshot_count", 0)) > 0
        and integrity.get("counter_mismatch_count") == 0
        and integrity.get("counter_mismatch_examples") == []
        and integrity.get("diagnostic_record_order_valid") is True
        and integrity.get("diagnostic_stamp_order", {}).get("valid") is True
        and strict == integrity.get("strict_drop_final") == 0
        and covered == integrity.get("covered_odometry_coalesced_final")
        and wall == integrity.get("wall_timer_coalesced_final")
        and total == integrity.get("total_suppressed_request_final")
        and total == strict + covered + wall,
        f"{context}: publication counter/order contract failed: {integrity}",
    )
    raw_unique = coverage.get("raw_unique_stamp_count")
    require(
        coverage.get("valid") is True
        and coverage.get("raw_record_order_valid") is True
        and coverage.get("existing_record_order_valid") is True
        and coverage.get("raw_stamp_canonical") is True
        and coverage.get("existing_stamp_canonical") is True
        and coverage.get("existing_positive_stamp_order_valid") is True
        and type(raw_unique) is int
        and raw_unique > 0
        and coverage.get("exactly_covered_raw_unique_stamp_count") == raw_unique
        and coverage.get("missing_raw_unique_stamp_count") == 0
        and coverage.get("missing_raw_stamp_examples_ns") == [],
        f"{context}: exact causal source-stamp coverage failed: {coverage}",
    )
    return {
        "status": "PASS",
        "strict_drop_count": strict,
        "covered_odometry_coalesced_count": covered,
        "wall_timer_coalesced_count": wall,
        "causal_raw_unique_stamp_count": raw_unique,
        "missing_raw_unique_stamp_count": 0,
    }


def validate_startup_window_provenance(
    startup: dict[str, Any], context: str
) -> dict[str, Any]:
    inputs = startup.get("inputs", {})
    contract = startup.get("contract", {})
    calibration = contract.get("calibration", {})
    require(
        startup.get("label") == f"{COURSE_LABELS[context]} startup yaw-safety acceptance",
        f"{context}: startup label mismatch",
    )
    expected_contract = {
        "absolute_yaw_safety_max_deg": 10.0,
        "activation_stamp_serialization_tolerance_sec": 0.02,
        "alignment": "speed legacy-global calibration yaw offset frozen for GLIM",
        "authority": "existing fusion only; position-only GNSS fallback disabled",
        "candidate_count": 3,
        "candidate_delta_max_rad": 0.08,
        "first_global_delay_max_sec": 25.0,
        "first_legacy_global_yaw_difference_max_deg": 3.0,
        "first_publish_rule": "exactly the next unique raw stamp after activation",
        "first_ready_anchor_lag_max_rad": 0.02,
        "readiness": "position_initialized && yaw_publishable",
        "session_rearm": (
            "fresh explicit unhealthy/non-TRACKING status, then fresh strict "
            "TRACKING; stale/unavailable alone never qualifies"
        ),
    }
    require(
        all(contract.get(key) == value for key, value in expected_contract.items()),
        f"{context}: startup safety contract mismatch",
    )
    start = calibration.get("start_sec")
    end = calibration.get("end_sec")
    duration = calibration.get("duration_sec")
    offset_rad = calibration.get("frozen_circular_yaw_offset_rad")
    offset_deg = calibration.get("frozen_circular_yaw_offset_deg")
    numeric_values = (start, end, duration, offset_rad, offset_deg)
    require(
        all(type(value) in (int, float) and math.isfinite(value) for value in numeric_values),
        f"{context}: startup calibration contains non-finite values",
    )
    require(
        start < end
        and duration == 20.0
        and abs((end - start) - duration) <= 1.0e-6
        and inputs.get("calibration_start_sec") == start
        and inputs.get("calibration_end_sec") == end
        and calibration.get("common_sample_count") == 400
        and calibration.get("maximum_interpolation_gap_sec") == 0.1
        and calibration.get("timestamp_source") == "physical ROS header stamps"
        and calibration.get("window_bounds") == "inclusive"
        and calibration.get("association")
        == (
            "GLIM reference timestamps inside the inclusive explicit window; "
            "speed legacy-global yaw linearly interpolated to those GLIM timestamps"
        )
        and abs(math.degrees(offset_rad) - offset_deg) <= 1.0e-9,
        f"{context}: startup calibration-window provenance mismatch: {calibration}",
    )
    return {
        "status": "PASS",
        "timestamp_source": calibration["timestamp_source"],
        "window_bounds": calibration["window_bounds"],
        "absolute_timestamps_published": False,
        "duration_sec": duration,
        "common_sample_count": calibration["common_sample_count"],
        "maximum_interpolation_gap_sec": calibration[
            "maximum_interpolation_gap_sec"
        ],
        "association": calibration["association"],
        "frozen_circular_yaw_offset_rad": offset_rad,
        "frozen_circular_yaw_offset_deg": offset_deg,
    }


def validate_precision_provenance_yaw_guard(
    provenance: dict[str, Any], context: str
) -> dict[str, Any]:
    configuration = (
        provenance.get("configuration_semantics", {})
        .get("precision_global_outage_yaw_guard")
    )
    runtime = (
        provenance.get("bags", {})
        .get("diagnostics", {})
        .get("precision_outage_yaw_guard")
    )
    require(isinstance(configuration, dict), f"{context}: guard config missing")
    require(isinstance(runtime, dict), f"{context}: guard runtime audit missing")
    expected_limits = {
        key: value
        for key, value in GNSS_OUTAGE_YAW_GUARD_CONFIG.items()
        if key != "required_fix_quality"
    }
    require(
        configuration.get("valid") is True
        and configuration.get("outage_yaw_guard_enabled") is True
        and configuration.get("fallback_gnss_position_enabled") is False
        and configuration.get("required_fix_quality")
        == GNSS_OUTAGE_YAW_GUARD_CONFIG["required_fix_quality"]
        and configuration.get("limits") == expected_limits,
        f"{context}: guard configuration contract failed: {configuration}",
    )
    accepted = runtime.get("accepted_reference_final")
    maximum_active_variance = runtime.get(
        "maximum_active_reference_variance_rad2"
    )
    require(
        runtime.get("valid") is True
        and int(runtime.get("snapshot_count", 0)) > 0
        and runtime.get("configuration_or_counter_mismatch_count") == 0
        and runtime.get("mismatch_examples") == []
        and runtime.get("accepted_reference_observed") is True
        and type(accepted) is int
        and accepted > 0
        and runtime.get("accepted_reference_maximum", 0) >= accepted
        and runtime.get("invalid_advance_maximum") == 0
        and runtime.get("suppressed_invalid_maximum") == 0
        and runtime.get("covariance_issue_sample_count") == 0
        and type(maximum_active_variance) in (int, float)
        and math.isfinite(maximum_active_variance)
        and maximum_active_variance > 0.0
        and maximum_active_variance
        <= GNSS_OUTAGE_YAW_GUARD_CONFIG["max_trusted_variance_rad2"]
        and runtime.get("stamp_order", {}).get("valid") is True,
        f"{context}: guard runtime contract failed: {runtime}",
    )
    return {
        "status": "PASS",
        "configuration_valid": True,
        "runtime_valid": True,
        "accepted_reference_count": accepted,
        "invalid_advance_count": 0,
        "suppressed_invalid_count": 0,
        "covariance_contract": {
            "status": "PASS",
            "issue_sample_count": 0,
            "maximum_active_reference_variance_rad2": maximum_active_variance,
            "outage_formula": (
                "active_reference_variance_rad2 + "
                "wrap(target_offset_rad - applied_offset_rad)^2"
            ),
            "release_formula": (
                "active_reference_variance_rad2 + applied_offset_rad^2"
            ),
        },
    }


def validate_accuracy_yaw_guard(
    result: dict[str, Any], context: str
) -> dict[str, Any]:
    guard_checks = [
        item
        for item in result.get("checks", [])
        if item.get("category") == "hard"
        and isinstance(item.get("name"), str)
        and item["name"].startswith("outage yaw guard")
    ]
    require(
        len(guard_checks) == len(GNSS_OUTAGE_YAW_GUARD_HARD_GATES)
        and {item["name"] for item in guard_checks}
        == set(GNSS_OUTAGE_YAW_GUARD_HARD_GATES)
        and all(item.get("passed") is True for item in guard_checks),
        f"{context}: outage-yaw guard hard-gate set is incomplete or failed",
    )
    guard = result.get("outage_yaw_guard")
    require(isinstance(guard, dict), f"{context}: outage-yaw guard result missing")
    config = guard.get("config", {})
    semantics = guard.get("semantics", {})
    counters = guard.get("counters", {})
    errors = guard.get("errors", {})
    continuity = guard.get("continuity_and_bounds", {})
    maxima = guard.get("maxima", {})
    maximum_active_variance = maxima.get("active_reference_variance_rad2")
    maximum_additional_variance = maxima.get("additional_variance_rad2")
    expected_observed_config = {
        key: [value] for key, value in GNSS_OUTAGE_YAW_GUARD_CONFIG.items()
    }
    expected_observed_semantics = {
        key: [value] for key, value in GNSS_OUTAGE_YAW_GUARD_SEMANTICS.items()
    }
    require(
        config.get("valid") is True
        and config.get("expected") == GNSS_OUTAGE_YAW_GUARD_CONFIG
        and config.get("observed") == expected_observed_config,
        f"{context}: outage-yaw guard config evidence mismatch: {config}",
    )
    require(
        semantics.get("valid") is True
        and semantics.get("expected") == GNSS_OUTAGE_YAW_GUARD_SEMANTICS
        and semantics.get("observed") == expected_observed_semantics,
        f"{context}: outage-yaw guard semantic evidence mismatch: {semantics}",
    )
    expected_error_groups = {
        "config", "continuity", "counters", "fields", "semantics", "state"
    }
    require(
        set(errors) == expected_error_groups
        and all(value == [] for value in errors.values()),
        f"{context}: outage-yaw guard errors present: {errors}",
    )
    require(
        guard.get("present") is True
        and guard.get("required_fields_complete") is True
        and guard.get("enabled_all") is True
        and guard.get("enabled_values") == ["true"]
        and guard.get("counter_monotonic") is True
        and int(guard.get("diagnostic_samples", 0)) > 0
        and guard.get("fusion_health_state_contract", {}).get("valid") is True
        and guard.get("fusion_health_state_contract", {}).get("violations") == []
        and continuity.get("valid") is True
        and continuity.get("errors") == []
        and int(continuity.get("composition_samples", 0)) > 0
        and continuity.get("variance_samples") == guard.get("diagnostic_samples")
        and type(maximum_active_variance) in (int, float)
        and math.isfinite(maximum_active_variance)
        and maximum_active_variance > 0.0
        and maximum_active_variance
        <= GNSS_OUTAGE_YAW_GUARD_CONFIG["max_trusted_variance_rad2"]
        and type(maximum_additional_variance) in (int, float)
        and math.isfinite(maximum_additional_variance)
        and maximum_additional_variance >= maximum_active_variance
        and float(continuity.get("maximum_observed_offset_rate_radps", float("inf")))
        <= GNSS_OUTAGE_YAW_GUARD_CONFIG["max_offset_rate_radps"] + 1.0e-12
        and type(counters.get("accepted_reference_count")) is int
        and counters["accepted_reference_count"] > 0
        and counters.get("invalid_advance_count") == 0
        and counters.get("suppressed_invalid_count") == 0
        and int(counters.get("outage_count", 0)) > 0
        and int(counters.get("recovery_count", 0)) > 0,
        f"{context}: outage-yaw guard runtime evidence failed: {guard}",
    )
    return {
        "status": "PASS",
        **GNSS_OUTAGE_YAW_GUARD_SEMANTICS,
        "config": GNSS_OUTAGE_YAW_GUARD_CONFIG,
        "accepted_reference_count": counters["accepted_reference_count"],
        "outage_count": counters["outage_count"],
        "recovery_count": counters["recovery_count"],
        "invalid_advance_count": 0,
        "suppressed_invalid_count": 0,
        "maximum_observed_offset_rate_radps": continuity[
            "maximum_observed_offset_rate_radps"
        ],
        "covariance_contract": {
            "status": "PASS",
            "variance_sample_count": continuity["variance_samples"],
            "maximum_active_reference_variance_rad2": maximum_active_variance,
            "maximum_additional_variance_rad2": maximum_additional_variance,
            "outage_formula": (
                "active_reference_variance_rad2 + "
                "wrap(target_offset_rad - applied_offset_rad)^2"
            ),
            "release_formula": (
                "active_reference_variance_rad2 + applied_offset_rad^2"
            ),
        },
    }


def parse_runtime_literal(value: str, expected_type: type, context: str) -> Any:
    try:
        parsed = ast.literal_eval(value)
    except (SyntaxError, ValueError) as error:
        raise RuntimeError(f"{context}: invalid literal: {value}") from error
    require(
        isinstance(parsed, expected_type),
        f"{context}: expected {expected_type.__name__}, got {type(parsed).__name__}",
    )
    return parsed


def runtime_fullmatch(
    pattern: str, value: str, context: str
) -> re.Match[str]:
    match = re.fullmatch(pattern, value)
    require(match is not None, f"{context}: malformed runtime evidence: {value}")
    return match


def validate_precision_runtime_log(path: Path, context: str) -> dict[str, Any]:
    """Parse the pinned full-rate validator log and publish normalized evidence."""
    lines = path.read_text(encoding="utf-8").splitlines()
    require(
        lines and all(line and line == line.strip() for line in lines),
        f"{context}: runtime log has blank or padded lines",
    )
    result_lines: list[tuple[str, str, str]] = []
    summary_lines: list[str] = []
    for line in lines:
        result_match = re.fullmatch(r"\[(PASS|WARN|FAIL)\] ([^:]+): (.*)", line)
        if result_match is not None:
            result_lines.append(result_match.groups())
            continue
        if line.startswith("summary: "):
            summary_lines.append(line)
            continue
        raise RuntimeError(f"{context}: unrecognized runtime-log line: {line}")

    pass_entries = [entry for entry in result_lines if entry[0] == "PASS"]
    warning_entries = [entry for entry in result_lines if entry[0] == "WARN"]
    fail_entries = [entry for entry in result_lines if entry[0] == "FAIL"]
    expected_sequence = (
        tuple(("PASS", label) for label in GNSS_RUNTIME_PASS_LABELS[:6])
        + (("WARN", GNSS_RUNTIME_INTENTIONAL_WARNING_LABEL),)
        + tuple(("PASS", label) for label in GNSS_RUNTIME_PASS_LABELS[6:])
    )
    require(
        len(pass_entries) == 27
        and tuple(entry[1] for entry in pass_entries) == GNSS_RUNTIME_PASS_LABELS
        and len(warning_entries) == 1
        and warning_entries[0][1] == GNSS_RUNTIME_INTENTIONAL_WARNING_LABEL
        and not fail_entries
        and len(result_lines) == 28
        and tuple((status, label) for status, label, _ in result_lines)
        == expected_sequence
        and summary_lines == ["summary: 28/28 checks passed"]
        and lines[-1] == "summary: 28/28 checks passed",
        f"{context}: runtime result accounting is not exactly 28/28 with the "
        "single intentional raw-startup warning",
    )
    details = {label: detail for _, label, detail in result_lines}
    require(
        len(details) == 28,
        f"{context}: duplicate runtime check labels",
    )

    require(
        details["required topics"] == "missing=[]",
        f"{context}: required runtime topics are missing",
    )
    correction_keys = runtime_fullmatch(
        r"scans=(\d+) corrections=(\d+) duplicate_scans=(\d+) "
        r"duplicate_corrections=(\d+) unknown_corrections=(\d+)",
        details["exact correction keys"],
        f"{context}/exact correction keys",
    )
    scans, corrections, duplicate_scans, duplicate_corrections, unknown = (
        int(value) for value in correction_keys.groups()
    )
    require(
        scans > 0
        and corrections > 0
        and duplicate_scans == duplicate_corrections == unknown == 0,
        f"{context}: correction-key uniqueness contract failed",
    )

    full_se2 = runtime_fullmatch(
        r"checked=(\d+) invalid_examples=(\[.*\]) "
        r"max_position_error_m=([-+0-9.eE]+) "
        r"max_orientation_error_rad=([-+0-9.eE]+)",
        details["correction full-SE2 contract"],
        f"{context}/full SE2",
    )
    position_error = float(full_se2.group(3))
    orientation_error = float(full_se2.group(4))
    require(
        int(full_se2.group(1)) == corrections
        and parse_runtime_literal(full_se2.group(2), list, context) == []
        and math.isfinite(position_error)
        and 0.0 <= position_error <= 1.0e-12
        and math.isfinite(orientation_error)
        and 0.0 <= orientation_error <= 1.0e-6,
        f"{context}: correction full-SE2 evidence failed",
    )
    require(
        details["dedicated precision frames"]
        == (
            "raw=['odom'] precision=['odom_precision'] "
            "local=[('odom_precision', 'base_link')] "
            "global=[('map', 'base_link')] pose=['map']"
        ),
        f"{context}: dedicated-frame contract changed",
    )

    physical_scans = runtime_fullmatch(
        r"count=(\d+) first_ns=(\d+)",
        details["physical scan stream order"],
        f"{context}/physical scans",
    )
    require(
        int(physical_scans.group(1)) == scans
        and int(physical_scans.group(2)) > 0,
        f"{context}: physical scan ordering evidence failed",
    )
    stamp_counts: dict[str, tuple[int, int]] = {}
    for label in (
        "raw odom stamp order after clock initialization",
        "precision local stamp order after clock initialization",
        "precision global stamp order after clock initialization",
    ):
        match = runtime_fullmatch(
            r"count=(\d+) leading_zero_count=(\d+)", details[label],
            f"{context}/{label}",
        )
        stamp_counts[label] = (int(match.group(1)), int(match.group(2)))
    raw_records, raw_leading_zero = stamp_counts[
        "raw odom stamp order after clock initialization"
    ]
    require(
        raw_records > scans and raw_leading_zero > 0,
        f"{context}: raw odometry startup-stamp evidence was not exercised",
    )
    warning = runtime_fullmatch(
        r"leading_zero_count=(\d+)",
        details[GNSS_RUNTIME_INTENTIONAL_WARNING_LABEL],
        f"{context}/intentional warning",
    )
    require(
        int(warning.group(1)) == raw_leading_zero,
        f"{context}: intentional startup warning count mismatch",
    )
    for prefix in ("precision local", "precision global"):
        ordered_count, leading_zero = stamp_counts[
            f"{prefix} stamp order after clock initialization"
        ]
        require(
            ordered_count > 0
            and leading_zero == 0
            and details[f"{prefix} intentional startup zero stamps"]
            == "leading_zero_count=0",
            f"{context}: {prefix} stamp contract failed",
        )

    initialization = runtime_fullmatch(
        r"warmup_sec=([-+0-9.eE]+) post-warmup correction ratio=([-+0-9.eE]+)",
        details["intentional initialization coverage"],
        f"{context}/initialization coverage",
    )
    warmup_sec = float(initialization.group(1))
    correction_ratio = float(initialization.group(2))
    require(
        warmup_sec == 3.2 and 0.0 < correction_ratio <= 1.0,
        f"{context}: initialization-coverage contract failed",
    )

    counter_match = runtime_fullmatch(
        r"summary=(\{.*\}) errors=(\[.*\])",
        details["map-fusion publication counter integrity"],
        f"{context}/map-fusion counters",
    )
    counter_summary = parse_runtime_literal(counter_match.group(1), dict, context)
    counter_errors = parse_runtime_literal(counter_match.group(2), list, context)
    counters = counter_summary.get("final_counters", {})
    strict_drop = counters.get("output.out_of_order_drop_count")
    covered = counters.get("output.covered_odometry_coalesced_count")
    wall = counters.get("output.wall_timer_coalesced_count")
    total = counters.get("output.total_suppressed_request_count")
    require(
        counter_errors == []
        and counter_summary.get("samples", 0) > 0
        and counter_summary.get("schema_samples") == counter_summary["samples"]
        and set(counters)
        == {
            "output.out_of_order_drop_count",
            "output.covered_odometry_coalesced_count",
            "output.wall_timer_coalesced_count",
            "output.total_suppressed_request_count",
        }
        and strict_drop == counter_summary.get("strict_drop_count") == 0
        and type(covered) is int
        and covered >= 0
        and type(wall) is int
        and wall >= 0
        and total == strict_drop + covered + wall
        and counter_summary.get("coalesced_report_only")
        == {"covered_odometry": covered, "wall_timer": wall},
        f"{context}: map-fusion publication counter integrity failed",
    )

    coverage_match = runtime_fullmatch(
        r"summary=(\{.*\}) errors=(\[.*\])",
        details["map-fusion exact raw stamp publication coverage"],
        f"{context}/map-fusion coverage",
    )
    coverage = parse_runtime_literal(coverage_match.group(1), dict, context)
    coverage_errors = parse_runtime_literal(coverage_match.group(2), list, context)
    eligible = coverage.get("eligible_unique_raw_stamps")
    matched = coverage.get("matched_unique_raw_stamps")
    require(
        coverage_errors == []
        and type(eligible) is int
        and eligible > 0
        and matched == eligible
        and coverage.get("missing_unique_raw_stamps") == 0
        and coverage.get("missing_examples_ns") == []
        and coverage.get("coverage_ratio") == 1.0
        and coverage.get("existing_prefix_unique_stamps", 0) >= eligible
        and coverage.get("eligible_raw_records", 0) >= eligible
        and coverage.get("duplicate_eligible_raw_records")
        == coverage["eligible_raw_records"] - eligible,
        f"{context}: exact causal raw-stamp publication coverage failed",
    )

    odometer = runtime_fullmatch(
        r"enabled=(True|False) tracking=([^ ]+) diag_scans=(\d+) "
        r"bag_scans=(\d+) conversion_max_ms=([-+0-9.eE]+)",
        details["odometer isolated snapshot contract"],
        f"{context}/odometer snapshot",
    )
    require(
        odometer.group(1) == "True"
        and odometer.group(2) == "scan_to_scan"
        and int(odometer.group(3)) == int(odometer.group(4)) == scans
        and math.isfinite(float(odometer.group(5)))
        and float(odometer.group(5)) >= 0.0,
        f"{context}: odometer snapshot contract failed",
    )

    matcher = runtime_fullmatch(
        r"queue_drop=(\d+) malformed=(\d+) stale=(\d+) "
        r"received/processed=(\d+)/(\d+) attempted=(\d+) "
        r"accepted/rejected=(\d+)/(\d+) committed/published=(\d+)/(\d+) "
        r"stream_reset/recovery=(\d+)/(\d+) generation=(\d+) "
        r"non_recovery_resets=(\d+) last_reason='([^']+)' "
        r"accepted_ratio=([-+0-9.eE]+)",
        details["matcher stream health"],
        f"{context}/matcher health",
    )
    matcher_values = [int(matcher.group(index)) for index in range(1, 15)]
    (
        queue_drop,
        malformed,
        stale,
        received,
        processed,
        attempted,
        accepted,
        rejected,
        committed,
        published_count,
        stream_reset,
        recovery,
        generation,
        non_recovery_resets,
    ) = matcher_values
    accepted_ratio = float(matcher.group(16))
    require(
        queue_drop == malformed == stale == 0
        and received == processed == scans
        and attempted == accepted + rejected
        and committed == published_count == corrections
        and 0 < committed <= accepted <= attempted
        and stream_reset == generation == recovery + 1
        and non_recovery_resets == 0
        and matcher.group(15) == "committed"
        and math.isfinite(accepted_ratio)
        and abs(accepted_ratio - accepted / attempted) <= 5.0e-4,
        f"{context}: matcher stream-health contract failed",
    )

    recording_counts = runtime_fullmatch(
        r"diag_scans=(\d+) bag_scans=(\d+) diag_corrections=(\d+) "
        r"bag_corrections=(\d+)",
        details["matcher diagnostic counts match recording"],
        f"{context}/matcher recording counts",
    )
    require(
        [int(value) for value in recording_counts.groups()]
        == [scans, scans, corrections, corrections],
        f"{context}: matcher diagnostic/recording counts disagree",
    )
    latency = runtime_fullmatch(
        r"processing_p99_ms=([-+0-9.eE]+) latency_p99_ms=([-+0-9.eE]+) "
        r"expected_rate=([-+0-9.eE]+)",
        details["matcher latency p99"],
        f"{context}/matcher latency",
    )
    processing_p99_ms, latency_p99_ms, expected_rate = (
        float(value) for value in latency.groups()
    )
    require(
        all(
            math.isfinite(value) and value > 0.0
            for value in (processing_p99_ms, latency_p99_ms, expected_rate)
        )
        and latency_p99_ms >= processing_p99_ms
        and expected_rate == 1.0,
        f"{context}: matcher latency/rate evidence failed",
    )

    global_diag = runtime_fullmatch(
        r"state=([^ ]+) anchor_initialized=(True|False) position_fused=(True|False) "
        r"scan_received=(\d+)/(\d+) correction_received=(\d+)/(\d+) "
        r"correction_accepted=(\d+)/(\d+) correction_rejected=(\d+) "
        r"raw_nonmonotonic=(\d+) publish_local=(\d+) publish_global=(\d+)",
        details["precision global diagnostics"],
        f"{context}/precision global diagnostics",
    )
    require(
        global_diag.group(1) in {"TRACKING", "STABILIZING_RECOVERY"}
        and global_diag.group(2) == global_diag.group(3) == "True"
        and int(global_diag.group(4)) == int(global_diag.group(5)) == scans
        and int(global_diag.group(6)) == int(global_diag.group(7)) == corrections
        and int(global_diag.group(8)) == int(global_diag.group(9)) == corrections
        and int(global_diag.group(10)) == int(global_diag.group(11)) == 0
        and int(global_diag.group(12)) > 0
        and int(global_diag.group(13)) > 0,
        f"{context}: precision-global diagnostic contract failed",
    )

    authority = details["existing-fusion global authority"]
    require(
        authority.startswith(
            "source='existing_fusion' fallback=False all_samples_authority=True "
        )
        and " causal_prefix={'valid': True" in authority
        and "'errors': []}" in authority
        and " odom_session_resets=0 no_rearm_needed=True " in authority
        and " activation_watermark_suppressed=1 " in authority,
        f"{context}: existing-fusion authority contract failed",
    )

    readiness = runtime_fullmatch(
        r"missing=(\[.*\]) samples=(\d+) incomplete_samples=(\d+)",
        details["startup readiness diagnostic schema"],
        f"{context}/startup readiness schema",
    )
    diagnostic_samples = int(readiness.group(2))
    require(
        parse_runtime_literal(readiness.group(1), list, context) == []
        and diagnostic_samples > 0
        and int(readiness.group(3)) == 0,
        f"{context}: startup readiness diagnostic schema failed",
    )

    freeze_match = runtime_fullmatch(
        r"groups=(\[.*\]) errors=(\[.*\])",
        details["existing-fusion anchor freezes outside strict health"],
        f"{context}/anchor freeze",
    )
    freeze_groups = parse_runtime_literal(freeze_match.group(1), list, context)
    require(
        freeze_groups
        and parse_runtime_literal(freeze_match.group(2), list, context) == []
        and all(
            isinstance(group, dict)
            and group.get("samples", 0) > 0
            and group.get("begin_ns", 0) <= group.get("end_ns", -1)
            and group.get("target_applied_equal") is True
            and group.get("serialization_exact") is True
            for group in freeze_groups
        ),
        f"{context}: anchor-freeze evidence failed",
    )

    guard_match = runtime_fullmatch(
        r"summary=(\{.*\}) errors=(\[.*\])",
        details["outage yaw guard runtime contract"],
        f"{context}/outage yaw guard",
    )
    guard = parse_runtime_literal(guard_match.group(1), dict, context)
    guard_variance = guard.get("maximum_active_reference_variance_rad2")
    expected_guard_states = [
        "DISARMED",
        "OUTAGE_HOLD",
        "OUTAGE_SLEW",
        "READY",
        "RECOVERY_RELEASE",
    ]
    require(
        parse_runtime_literal(guard_match.group(2), list, context) == []
        and guard.get("samples") == guard.get("parsed_samples")
        == diagnostic_samples
        and guard.get("leading_zero_stamp_samples", 0) > 0
        and guard.get("states") == expected_guard_states
        and guard.get("active_samples")
        == guard.get("outage_samples", 0) + guard.get("release_samples", 0)
        and guard.get("outage_samples", 0) > 0
        and guard.get("release_samples", 0) > 0
        and type(guard_variance) in (int, float)
        and math.isfinite(guard_variance)
        and 0.0 < guard_variance
        <= GNSS_OUTAGE_YAW_GUARD_CONFIG["max_trusted_variance_rad2"]
        and guard.get("config")
        == [
            GNSS_OUTAGE_YAW_GUARD_CONFIG["max_trusted_age_sec"],
            GNSS_OUTAGE_YAW_GUARD_CONFIG["max_trusted_variance_rad2"],
            GNSS_OUTAGE_YAW_GUARD_CONFIG["max_trusted_delta_rad"],
            GNSS_OUTAGE_YAW_GUARD_CONFIG["max_offset_rate_radps"],
            GNSS_OUTAGE_YAW_GUARD_CONFIG["max_offset_step_rad"],
            GNSS_OUTAGE_YAW_GUARD_CONFIG["max_step_dt_sec"],
        ],
        f"{context}: outage-yaw guard runtime contract failed",
    )

    rearm_match = runtime_fullmatch(
        r"summary=(\{.*\}) errors=(\[.*\])",
        details["existing-fusion session rearm requires an explicit fresh health edge"],
        f"{context}/session rearm",
    )
    rearm = parse_runtime_literal(rearm_match.group(1), dict, context)
    require(
        parse_runtime_literal(rearm_match.group(2), list, context) == []
        and rearm.get("observed_samples") == diagnostic_samples
        and rearm.get("session_resets") == rearm.get("completed_resets") == 0
        and rearm.get("contract")
        == (
            "stale/unavailable never qualifies; fresh explicit unhealthy then "
            "fresh strict TRACKING"
        ),
        f"{context}: existing-fusion session-rearm contract failed",
    )

    transition_match = runtime_fullmatch(
        r"transitions=(\[.*\]) errors=(\[.*\])",
        details["startup readiness transitions"],
        f"{context}/startup transitions",
    )
    transitions = parse_runtime_literal(transition_match.group(1), list, context)
    require(
        len(transitions) == 1
        and parse_runtime_literal(transition_match.group(2), list, context) == []
        and transitions[0].get("epoch") == 1
        and transitions[0].get("activation_ns", 0) > 0
        and transitions[0].get("diagnostic_stamp_ns", 0)
        >= transitions[0]["activation_ns"]
        and math.isfinite(transitions[0].get("candidate_yaw_rad", float("nan")))
        and transitions[0].get("anchor_lag_yaw_rad") == 0.0,
        f"{context}: startup readiness-transition evidence failed",
    )

    startup_match = runtime_fullmatch(
        r"session_starts=(\[.*\]) intervals=(\[.*\]) "
        r"activation_serialization_tolerance_ns=(\d+)",
        details["startup global publication safety"],
        f"{context}/startup global safety",
    )
    session_starts = parse_runtime_literal(startup_match.group(1), list, context)
    intervals = parse_runtime_literal(startup_match.group(2), list, context)
    require(
        len(session_starts) == len(intervals) == 1
        and session_starts[0] > 0
        and intervals[0].get("begin_ns") == session_starts[0]
        and intervals[0].get("activation_ns") == transitions[0]["activation_ns"]
        and intervals[0].get("before_global") == 0
        and intervals[0].get("before_pose") == 0
        and intervals[0].get("delay_sec", 0.0) > 0.0
        and intervals[0].get("expected_first_global_ns")
        == intervals[0].get("first_global_ns")
        == intervals[0].get("first_pose_ns")
        and abs(intervals[0].get("first_anchor_yaw_error_rad", float("inf")))
        <= 1.0e-3
        and intervals[0].get("diagnostic_anchor_lag_yaw_rad") == 0.0
        and int(startup_match.group(3)) == 20_000_000,
        f"{context}: startup global-publication safety failed",
    )
    suppressed = runtime_fullmatch(
        r"suppressed_not_ready=(\d+)",
        details["startup global suppression exercised"],
        f"{context}/startup suppression",
    )
    require(
        int(suppressed.group(1)) > 0,
        f"{context}: startup suppression was not exercised",
    )

    playback = runtime_fullmatch(
        r"observed=([-+0-9.eE]+) expected=([-+0-9.eE]+)",
        details["playback rate"],
        f"{context}/playback rate",
    )
    observed_rate, playback_expected_rate = (
        float(value) for value in playback.groups()
    )
    require(
        observed_rate == playback_expected_rate == expected_rate == 1.0,
        f"{context}: playback rate is not exactly 1.0x",
    )

    return {
        "status": "PASS",
        "successful_check_count": 28,
        "pass_line_count": 27,
        "failed_check_count": 0,
        "intentional_warning_count": 1,
        "intentional_warning": GNSS_RUNTIME_INTENTIONAL_WARNING_LABEL,
        "playback_rate": {
            "observed": observed_rate,
            "expected": playback_expected_rate,
        },
        "matcher": {
            "scans": scans,
            "corrections": corrections,
            "received": received,
            "processed": processed,
            "attempted": attempted,
            "accepted": accepted,
            "rejected": rejected,
            "committed": committed,
            "published": published_count,
            "queue_drop_count": queue_drop,
            "malformed_count": malformed,
            "stale_count": stale,
            "processing_p99_ms": processing_p99_ms,
            "end_to_end_latency_p99_ms": latency_p99_ms,
        },
        "map_fusion_publication": {
            "status": "PASS",
            "counter_samples": counter_summary["samples"],
            "strict_drop_count": strict_drop,
            "covered_odometry_coalesced_count": covered,
            "wall_timer_coalesced_count": wall,
            "total_suppressed_request_count": total,
            "eligible_unique_raw_stamp_count": eligible,
            "matched_unique_raw_stamp_count": matched,
            "missing_unique_raw_stamp_count": 0,
            "exact_causal_coverage_ratio": 1.0,
        },
        "outage_yaw_guard": {
            "status": "PASS",
            "diagnostic_sample_count": diagnostic_samples,
            "states": expected_guard_states,
            "active_sample_count": guard["active_samples"],
            "outage_sample_count": guard["outage_samples"],
            "recovery_release_sample_count": guard["release_samples"],
            "maximum_active_reference_variance_rad2": guard_variance,
        },
        "startup": {
            "intentional_raw_zero_stamp_count": raw_leading_zero,
            "global_suppressed_not_ready_count": int(suppressed.group(1)),
            "first_global_delay_sec": intervals[0]["delay_sec"],
            "published_before_activation_count": 0,
        },
    }


def validate_gnss_evidence(
    course: str, sources: dict[str, Path]
) -> dict[str, Any]:
    """Validate status, method, run identity, and provenance before publishing."""
    contract = GNSS_SOURCE_CONTRACTS[course]
    label = COURSE_LABELS[course]
    result = read_json(sources["accuracy_result"])
    startup = read_json(sources["startup_acceptance"])
    accepted_scan = read_json(sources["accepted_scan_nonintrusion"])
    runtime_validation = validate_precision_runtime_log(
        sources["precision_runtime_validation"],
        f"{course}/precision_runtime_validation",
    )
    baseline_run_id = contract["baseline_run_id"]
    control_run_id = contract["control_run_id"]
    precision_run_id = contract["precision_run_id"]

    runtime_expected = contract["expected_runtime"]
    runtime_matcher = runtime_validation["matcher"]
    runtime_publication = runtime_validation["map_fusion_publication"]
    runtime_guard = runtime_validation["outage_yaw_guard"]
    require(
        runtime_matcher["scans"] == runtime_expected["scans"]
        and runtime_matcher["corrections"] == runtime_expected["corrections"]
        and runtime_matcher["processing_p99_ms"]
        == runtime_expected["processing_p99_ms"]
        and runtime_matcher["end_to_end_latency_p99_ms"]
        == runtime_expected["end_to_end_latency_p99_ms"]
        and runtime_publication["eligible_unique_raw_stamp_count"]
        == runtime_expected["eligible_unique_raw_stamp_count"]
        and runtime_publication["covered_odometry_coalesced_count"]
        == runtime_expected["covered_odometry_coalesced_count"]
        and runtime_guard["maximum_active_reference_variance_rad2"]
        == runtime_expected["maximum_active_reference_variance_rad2"],
        f"{course}: pinned runtime metrics differ from the adopted contract",
    )

    failed_hard_gates = [
        item["name"]
        for item in result.get("checks", [])
        if item.get("category") == "hard" and not item.get("passed", False)
    ]
    require(
        result.get("label") == contract["source_accuracy_label"],
        f"{course}: accuracy label mismatch",
    )
    require(
        result.get("passed") is contract["accepted"],
        f"{course}: unexpected accuracy acceptance status",
    )
    hard_checks = [
        item for item in result.get("checks", [])
        if item.get("category") == "hard"
    ]
    require(
        result.get("hard_gate_count")
        == contract["expected_accuracy_hard_gate_count"]
        == len(hard_checks)
        and result.get("failed_hard_gate_count") == len(failed_hard_gates) == 0
        and all(item.get("passed") is True for item in hard_checks),
        f"{course}: accuracy hard-gate accounting failed",
    )
    require(
        failed_hard_gates == contract["expected_failed_hard_gates"],
        f"{course}: unexpected failed hard gates: {failed_hard_gates}",
    )
    yaw_guard_summary = validate_accuracy_yaw_guard(result, course)

    method = result.get("method", {})
    require(
        method.get("alignment_mode") == "exact-initial-pose",
        f"{course}: primary accuracy alignment is not exact initial pose",
    )
    require(
        method.get("accuracy_estimator_association")
        == "exact same-run integer header-stamp intersection; no estimator interpolation",
        f"{course}: estimator association is not exact same-run stamp intersection",
    )
    require(
        method.get("accuracy_reference_association")
        == "GLIM interpolated to exact estimator header stamps only",
        f"{course}: unexpected GLIM association method",
    )
    require(
        method.get("yaw_alignment")
        == "the yaw of the same complete initial-pose SE(2); no separate yaw fit",
        f"{course}: position and yaw do not share the same alignment transform",
    )
    primary = result.get("primary_alignment", {})
    require(primary.get("mode") == "exact-initial-pose", f"{course}: bad primary mode")
    for group, baseline in (("local", "precision_raw"), ("global", "precision_existing")):
        alignment = primary.get(group, {})
        require(
            alignment.get("type") == "single_pose_exact_se2",
            f"{course}/{group}: alignment is not a single exact-pose SE(2)",
        )
        require(
            alignment.get("baseline") == baseline,
            f"{course}/{group}: unexpected primary alignment baseline",
        )
        require(
            alignment.get("scale_estimated_or_applied") is False,
            f"{course}/{group}: scale must not be estimated or applied",
        )
        residual = alignment.get("initial_residuals", {}).get(baseline, {})
        require(
            float(residual.get("position_m", float("inf"))) <= 1.0e-9
            and abs(float(residual.get("yaw_rad", float("inf")))) <= 1.0e-12,
            f"{course}/{group}: baseline initial pose does not map exactly",
        )

    inputs = result.get("inputs", {})
    require(
        path_has_suffix(
            inputs.get("precision_bag"),
            f"{GNSS_RESULT_SET}/{precision_run_id}/localization_output",
        ),
        f"{course}: accuracy result does not use canonical precision run",
    )
    require(
        path_has_suffix(
            inputs.get("speed_bag"),
            f"{GNSS_RESULT_SET}/{baseline_run_id}/localization_output",
        ),
        f"{course}: accuracy result does not use canonical baseline run",
    )
    csv_specs = (
        ("local", "aligned_local_samples", ("precision_raw", "precision_local")),
        (
            "global",
            "aligned_global_samples",
            ("precision_existing", "precision_global"),
        ),
    )
    for group, source_id, series_names in csv_specs:
        csv_path = sources[source_id]
        require(
            Path(result.get("plot_artifacts", {}).get(group, {}).get("csv", ""))
            == csv_path,
            f"{course}/{group}: result points to an unexpected aligned CSV",
        )
        reference, series = load_wide_csv(csv_path, series_names)
        require(
            len(reference["stamp_sec"])
            == result.get("plot_artifacts", {}).get(group, {}).get("samples"),
            f"{course}/{group}: CSV sample count mismatch",
        )
        require(
            np.all(np.isfinite(reference["stamp_sec"]))
            and np.all(np.diff(reference["stamp_sec"]) > 0.0),
            f"{course}/{group}: estimator stamps are not finite and unique",
        )
        baseline = series_names[0]
        require(
            abs(float(series[baseline]["xy_error"][0])) <= 1.0e-9
            and abs(float(series[baseline]["yaw_error"][0])) <= 1.0e-9,
            f"{course}/{group}: published baseline does not start at exact pose",
        )

    startup_checks = startup.get("checks", [])
    require(
        startup.get("passed") is True
        and len(startup_checks) == contract["expected_startup_check_count"]
        and all(item.get("passed") is True for item in startup_checks),
        f"{course}: startup acceptance failed or is incomplete",
    )
    startup_window_summary = validate_startup_window_provenance(startup, course)
    require(
        path_has_suffix(
            startup.get("inputs", {}).get("precision_bag"),
            f"{GNSS_RESULT_SET}/{precision_run_id}/localization_output",
        )
        and path_has_suffix(
            startup.get("inputs", {}).get("speed_bag"),
            f"{GNSS_RESULT_SET}/{baseline_run_id}/localization_output",
        ),
        f"{course}: startup result does not use canonical precision run",
    )
    accepted_scan_checks = accepted_scan.get("checks", [])
    accepted_scan_hard = [
        item for item in accepted_scan_checks if item.get("category") == "hard"
    ]
    failed_warnings = [
        item for item in accepted_scan_checks
        if item.get("category") == "warn" and item.get("passed") is not True
    ]
    require(
        accepted_scan.get("passed") is True
        and len(accepted_scan_checks)
        == contract["expected_nonintrusion_check_count"]
        and len(accepted_scan_hard)
        == contract["expected_nonintrusion_hard_gate_count"]
        and all(item.get("passed") is True for item in accepted_scan_hard)
        and len(failed_warnings)
        == contract["expected_nonintrusion_failed_warning_count"],
        f"{course}: accepted-scan non-intrusion hard contract failed",
    )
    require(
        path_has_suffix(
            accepted_scan.get("inputs", {}).get("control_bag"),
            f"{GNSS_RESULT_SET}/{control_run_id}/localization_output",
        )
        and path_has_suffix(
            accepted_scan.get("inputs", {}).get("precision_bag"),
            f"{GNSS_RESULT_SET}/{precision_run_id}/localization_output",
        ),
        f"{course}: accepted-scan result uses unexpected runs",
    )

    provenance_specs = (
        ("baseline_run_provenance", "baseline", baseline_run_id),
        ("control_run_provenance", "control", control_run_id),
        ("precision_run_provenance", "precision", precision_run_id),
    )
    provenance_summary: dict[str, Any] = {}
    for source_id, expected_mode, run_id in provenance_specs:
        provenance = read_json(sources[source_id])
        context = f"{course}/{source_id}"
        require(
            provenance.get("summary", {}).get("passed") is True,
            f"{context}: provenance validator failed",
        )
        require(
            provenance.get("summary", {}).get("failed_check_count") == 0,
            f"{context}: provenance has failed checks",
        )
        require(
            provenance.get("expected_mode") == expected_mode,
            f"{context}: mode mismatch",
        )
        require(
            provenance.get("dataset", {}).get("id") == course,
            f"{context}: dataset mismatch",
        )
        require(
            path_has_suffix(
                provenance.get("run_directory"),
                f"{GNSS_RESULT_SET}/{run_id}",
            ),
            f"{context}: run-directory mismatch",
        )
        validate_default_projection(
            provenance.get("configuration_semantics", {}).get("nmea_projection"),
            context,
        )
        require(
            provenance.get("configuration_semantics", {})
            .get("gnss_fusion_single_antenna", {})
            .get("xy_only_recovery_enabled")
            is True,
            f"{context}: single-antenna recovery override not verified",
        )
        configuration_provenance = provenance.get("configuration_provenance", {})
        require(configuration_provenance, f"{context}: no configuration provenance")
        require(
            all(item.get("valid") is True for item in configuration_provenance.values()),
            f"{context}: invalid configuration provenance entry",
        )
        publication_summary = validate_map_fusion_publication_integrity(
            provenance, context
        )
        provenance_summary[expected_mode] = {
            "run_id": run_id,
            "status": "PASS",
            "check_count": provenance["summary"]["check_count"],
            "map_fusion_publication_integrity": publication_summary,
        }
        if expected_mode == "precision":
            provenance_summary[expected_mode]["outage_yaw_guard"] = (
                validate_precision_provenance_yaw_guard(provenance, context)
            )

    accuracy_guard_covariance = yaw_guard_summary["covariance_contract"]
    provenance_guard = provenance_summary["precision"]["outage_yaw_guard"]
    provenance_guard_covariance = provenance_guard["covariance_contract"]
    require(
        provenance_guard["accepted_reference_count"]
        == yaw_guard_summary["accepted_reference_count"]
        and provenance_guard_covariance[
            "maximum_active_reference_variance_rad2"
        ] == accuracy_guard_covariance[
            "maximum_active_reference_variance_rad2"
        ],
        f"{course}: accuracy and provenance outage-yaw covariance evidence disagree",
    )

    return {
        "result": result,
        "startup": startup,
        "accepted_scan": accepted_scan,
        "runtime_validation": runtime_validation,
        "failed_hard_gates": failed_hard_gates,
        "provenance_summary": provenance_summary,
        "outage_yaw_guard": yaw_guard_summary,
        "startup_window_provenance": startup_window_summary,
    }


def stats(metric: dict[str, Any]) -> dict[str, Any]:
    return {
        key: metric[key]
        for key in ("count", "rmse", "mean", "median", "p95", "p99", "maximum")
        if key in metric
    }


def rpe(metrics: dict[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for distance, item in metrics.get("rpe", {}).items():
        entry: dict[str, Any] = {"count": item.get("count", 0)}
        if item.get("count", 0):
            entry["translation_error_m"] = stats(item["translation_error_m"])
            entry["yaw_error_deg"] = stats(item["yaw_error_deg"])
        result[distance] = entry
    return result


def lidar_stream(metrics: dict[str, Any]) -> dict[str, Any]:
    return {
        "samples": metrics["samples"],
        "duration_sec": metrics["duration_sec"],
        "reference_path_m": metrics["reference_path_m"],
        "estimate_path_m": metrics["estimate_path_m"],
        "path_ratio_estimate_over_reference": metrics[
            "path_ratio_estimate_over_reference"
        ],
        "position_error_m": stats(metrics["position_error_m"]),
        "yaw_error_deg": stats(metrics["yaw_error_deg"]),
        "endpoint_position_error_m": metrics["endpoint_position_error_m"],
        "endpoint_yaw_error_deg": metrics["endpoint_yaw_error_deg"],
        "rpe": rpe(metrics),
    }


def gnss_stream(metrics: dict[str, Any]) -> dict[str, Any]:
    return {
        "samples": metrics["samples"],
        "fixed_shared_xy_error_m": stats(metrics["fixed_common_xy_error_m"]),
        "fixed_shared_yaw_error_deg": stats(metrics["common_yaw_offset_error_deg"]),
        "full_shape_independent_xy_error_m": stats(
            metrics["full_shape_independent_se2"]["position_error_m"]
        ),
        "full_shape_independent_yaw_error_deg": stats(
            metrics["full_shape_independent_se2"]["yaw_error_deg"]
        ),
        "rpe": rpe(metrics),
    }


def load_long_csv(path: Path) -> dict[str, dict[str, np.ndarray]]:
    grouped: dict[str, dict[str, list[float]]] = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            item = grouped.setdefault(
                row["stream"],
                {key: [] for key in (
                    "time", "reference_x", "reference_y", "aligned_x", "aligned_y",
                    "position_error", "yaw_error",
                )},
            )
            mapping = {
                "time": "time_from_primary_anchor_sec",
                "reference_x": "reference_x",
                "reference_y": "reference_y",
                "aligned_x": "aligned_x",
                "aligned_y": "aligned_y",
                "position_error": "position_error_m",
                "yaw_error": "yaw_error_deg",
            }
            for key, column in mapping.items():
                item[key].append(float(row[column]))
    return {
        name: {key: np.asarray(values) for key, values in item.items()}
        for name, item in grouped.items()
    }


def load_wide_csv(
    path: Path, names: tuple[str, ...]
) -> tuple[dict[str, np.ndarray], dict[str, dict[str, np.ndarray]]]:
    data = np.genfromtxt(path, delimiter=",", names=True)
    reference = {
        "stamp_sec": np.asarray(data["stamp_sec"], dtype=float),
        "time": np.asarray(data["time_from_common_start_sec"], dtype=float),
        "x": np.asarray(data["glim_x"], dtype=float),
        "y": np.asarray(data["glim_y"], dtype=float),
        "calibration_mask": np.asarray(data["calibration_mask"], dtype=bool),
        "outage_mask": np.asarray(data["outage_mask"], dtype=bool),
    }
    series = {
        name: {
            "x": np.asarray(data[f"{name}_x"], dtype=float),
            "y": np.asarray(data[f"{name}_y"], dtype=float),
            "xy_error": np.asarray(data[f"{name}_xy_error_m"], dtype=float),
            "yaw_error": np.asarray(data[f"{name}_yaw_error_deg"], dtype=float),
        }
        for name in names
    }
    return reference, series


def gnss_error_plot_timeline(
    csv_path: Path,
    result: dict[str, Any],
    expected: dict[str, Any],
) -> dict[str, Any]:
    reference, _ = load_wide_csv(
        csv_path, ("precision_existing", "precision_global")
    )
    require(
        len(reference["stamp_sec"]) >= 2,
        f"{csv_path}: global error CSV has too few samples",
    )
    axis_origins = reference["stamp_sec"] - reference["time"]
    require(
        bool(np.all(np.isfinite(axis_origins))),
        f"{csv_path}: non-finite timestamp-to-plot-axis mapping",
    )
    axis_origin_sec = float(np.median(axis_origins))
    mapping_residual = (
        reference["stamp_sec"] - axis_origin_sec - reference["time"]
    )
    maximum_mapping_residual_sec = float(np.max(np.abs(mapping_residual)))
    require(
        maximum_mapping_residual_sec <= 1.0e-6,
        f"{csv_path}: timestamp-to-plot-axis mapping is not constant",
    )
    outage = result.get("precision_timeline", {}).get("longest_q4_outage")
    require(isinstance(outage, dict), "missing longest RTK-Q4 outage evidence")
    begin_ns = outage.get("begin_ns")
    end_ns = outage.get("end_ns")
    recovery_delay_sec = outage.get("recovery_delay_sec")
    require(
        isinstance(begin_ns, int)
        and isinstance(end_ns, int)
        and isinstance(recovery_delay_sec, (int, float))
        and math.isfinite(recovery_delay_sec),
        "invalid RTK-Q4 outage/recovery timestamps",
    )
    resume_ns = end_ns + int(round(recovery_delay_sec * 1.0e9))
    def plot_time(stamp_ns: int) -> float:
        return stamp_ns * 1.0e-9 - axis_origin_sec

    begin_plot_sec = plot_time(begin_ns)
    end_plot_sec = plot_time(end_ns)
    resume_plot_sec = plot_time(resume_ns)
    observed = {
        "rtk_q4_unavailable_begin_time_sec": begin_plot_sec,
        "rtk_q4_unavailable_end_time_sec": end_plot_sec,
        "rtk_q4_unavailable_duration_sec": (end_ns - begin_ns) * 1.0e-9,
        "fusion_tracking_resume_time_sec": resume_plot_sec,
        "fusion_tracking_resume_delay_sec": recovery_delay_sec,
    }
    require(
        observed == expected,
        "RTK-Q4 error-plot timeline changed from the reviewed source contract: "
        f"observed={observed!r} expected={expected!r}",
    )
    require(
        float(reference["time"][0]) <= begin_plot_sec
        < end_plot_sec
        < resume_plot_sec
        <= float(reference["time"][-1]),
        "RTK-Q4 error-plot annotations are outside the global CSV time axis",
    )
    return {
        "time_axis": {
            "csv_plot_time_column": "time_from_common_start_sec",
            "origin": "first common global evaluation sample",
            "absolute_timestamps_published": False,
            "maximum_mapping_residual_sec": maximum_mapping_residual_sec,
        },
        "rtk_q4_unavailable": {
            "begin_time_from_common_start_sec": begin_plot_sec,
            "end_time_from_common_start_sec": end_plot_sec,
            "duration_sec": (end_ns - begin_ns) * 1.0e-9,
            "definition": (
                "interval from the two-second usable-GNSS timeout until "
                "the next usable high-quality GNSS position sample"
            ),
        },
        "rtk_q4_return": {
            "time_from_common_start_sec": end_plot_sec,
            "definition": "next usable high-quality GNSS position sample",
        },
        "fusion_tracking_resume": {
            "time_from_common_start_sec": resume_plot_sec,
            "delay_from_rtk_q4_return_sec": recovery_delay_sec,
            "definition": (
                "first global-localization diagnostic after usable GNSS "
                "returns with a finite position-fused tracking state"
            ),
        },
    }


def pyplot() -> Any:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/publication_assets_matplotlib")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def render_lidar(
    csv_path: Path,
    output: Path,
    title: str,
    names: tuple[str, ...],
    display: dict[str, str],
) -> None:
    plt = pyplot()
    series = load_long_csv(csv_path)
    missing = set(names) - set(series)
    if missing:
        raise RuntimeError(f"{csv_path} lacks streams: {sorted(missing)}")
    colors = {
        "control_raw": "#e68613",
        "precision_raw": "#888888",
        "precision_local": "#2764c4",
    }
    output.mkdir(parents=True, exist_ok=True)
    reference = series[names[0]]

    figure, axis = plt.subplots(figsize=(8.5, 7.0))
    axis.plot(reference["reference_x"], reference["reference_y"], color="black", linewidth=2.0, label="GLIM reference")
    for name in names:
        item = series[name]
        axis.plot(item["aligned_x"], item["aligned_y"], color=colors[name], linewidth=1.3, label=display[name])
    axis.scatter(reference["reference_x"][0], reference["reference_y"][0], color="#24933d", s=35, label="start", zorder=5)
    axis.set_aspect("equal", adjustable="datalim")
    axis.set_xlabel("Reference x [m]")
    axis.set_ylabel("Reference y [m]")
    axis.set_title(f"{title}\nTrajectory")
    axis.grid(True, alpha=0.25)
    axis.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(output / "trajectory.png", dpi=150, bbox_inches="tight")
    plt.close(figure)

    for field, ylabel, filename, caption in (
        ("position_error", "Absolute XY error [m]", "xy_error.png", "Absolute XY error"),
        ("yaw_error", "Absolute yaw error [deg]", "yaw_error.png", "Absolute yaw error"),
    ):
        figure, axis = plt.subplots(figsize=(10.5, 5.0))
        for name in names:
            item = series[name]
            rmse = float(np.sqrt(np.mean(item[field] ** 2)))
            axis.plot(item["time"], item[field], color=colors[name], linewidth=1.1, label=f"{display[name]} (RMSE {rmse:.3f})")
        axis.set_xlabel("Time from common anchor [s]")
        axis.set_ylabel(ylabel)
        axis.set_title(f"{title}\n{caption}")
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=8)
        figure.tight_layout()
        figure.savefig(output / filename, dpi=150, bbox_inches="tight")
        plt.close(figure)


def render_gnss_errors(
    csv_path: Path,
    output: Path,
    title: str,
    group: str,
    plot_timeline: dict[str, Any] | None = None,
) -> None:
    plt = pyplot()
    if group == "local":
        names = ("precision_raw", "precision_local")
        display = {
            "precision_raw": "Scan-to-scan",
            "precision_local": "Scan-to-submap",
        }
    else:
        names = ("precision_existing", "precision_global")
        display = {
            "precision_existing": "Scan-to-scan",
            "precision_global": "Scan-to-submap",
        }
    colors = {names[0]: "#e68613", names[1]: "#2764c4"}
    linestyles = {names[0]: "-", names[1]: "--"}
    reference, series = load_wide_csv(csv_path, names)
    output.mkdir(parents=True, exist_ok=True)
    if group == "global":
        require(
            plot_timeline is not None,
            "global GNSS error plots require the reviewed outage timeline",
        )
    else:
        require(
            plot_timeline is None,
            "the GNSS outage timeline belongs only on global error plots",
        )

    for key, ylabel, suffix, caption in (
        ("xy_error", "Absolute XY error [m]", "xy_error", "Absolute XY error"),
        ("yaw_error", "Absolute yaw error [deg]", "yaw_error", "Absolute yaw error"),
    ):
        figure, axis = plt.subplots(figsize=(10.5, 5.0))
        for name in names:
            values = series[name][key]
            rmse = float(np.sqrt(np.mean(values ** 2)))
            axis.plot(
                reference["time"],
                values,
                color=colors[name],
                linestyle=linestyles[name],
                linewidth=1.1,
                label=f"{display[name]} (RMSE {rmse:.3f})",
            )
        if plot_timeline is not None:
            unavailable = plot_timeline["rtk_q4_unavailable"]
            returned = plot_timeline["rtk_q4_return"]
            resumed = plot_timeline["fusion_tracking_resume"]
            axis.axvspan(
                unavailable["begin_time_from_common_start_sec"],
                unavailable["end_time_from_common_start_sec"],
                facecolor="#d9d9d9",
                edgecolor="#555555",
                alpha=0.38,
                hatch="///",
                linewidth=0.8,
                zorder=0,
                label=f"GNSS unavailable ({unavailable['duration_sec']:.1f} s)",
            )
            axis.axvline(
                returned["time_from_common_start_sec"],
                color="#9d2035",
                linestyle="-.",
                linewidth=1.5,
                zorder=3,
                label="GNSS returns",
            )
            axis.axvline(
                resumed["time_from_common_start_sec"],
                color="#23713c",
                linestyle=":",
                linewidth=2.0,
                zorder=3,
                label=(
                    "Localization resumes "
                    f"(+{resumed['delay_from_rtk_q4_return_sec']:.1f} s)"
                ),
            )
        axis.set_xlabel("Time from common start [s]")
        axis.set_ylabel(ylabel)
        axis.set_title(
            f"{title}\n{group.capitalize()} {caption} "
            "(same-run exact estimator stamps)"
        )
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=8, ncol=2)
        figure.tight_layout()
        figure.savefig(
            output / f"{group}_{suffix}.png", dpi=150, bbox_inches="tight"
        )
        plt.close(figure)


MID360_RUNTIME_CHECKS = (
    "clock_monotonic",
    "one_x_realtime",
    "corrected_imu_exact_stamp_coverage",
    "corrected_imu_strict_stamp_order",
    "corrected_imu_si_acceleration",
    "local_odom_stamp_order",
    "local_odom_finite_unit_quaternion",
    "local_odom_time_coverage",
    "local_odom_frames",
    "accepted_scan_odom_strict_stamps",
    "accepted_scan_odom_physical_reference_stamp",
    "accepted_scan_odom_frames",
    "accepted_scan_odom_finite_unit_quaternion",
    "deskew_diagnostic_input_coverage",
    "deskew_success_rate",
    "deskew_time_contract",
    "imu_acceleration_scale_contract",
    "scan_registration_density",
    "accepted_scan_odom_diagnostic_contract",
    "accepted_scan_odom_final_count",
    "scan_to_scan_only",
)

MID360_STRUCTURAL_CHECKS = (
    "base LiDAR/IMU runtime audit passes",
    "raw accepted-scan poses are unique and strictly increasing",
    "precision-local poses are unique and strictly increasing",
    "raw accepted-scan frame contract",
    "precision-local frame contract",
    "SubmapScan exact keys and physical stamps are valid",
    "SubmapScan snapshots map one-to-one to accepted physical scans",
    "SubmapCorrection exact-key and full-SE2 contracts are valid",
    "submap initialization and correction coverage are bounded",
    "precision-local covers accepted physical scans without interpolation",
    "precision-local starts only after the first SubmapScan epoch",
    "precision global outputs are discovered but publish zero messages",
    "odometer snapshot diagnostics are consistent",
    "matcher runtime and counter contracts pass",
    "local compositor counters and global isolation pass",
)


def pinned_mid360_sources() -> dict[str, Path]:
    sources: dict[str, Path] = {}
    for source_id, (relative, expected_hash) in MID360_SOURCE_CONTRACT.items():
        require(
            re.fullmatch(r"[0-9a-f]{64}", expected_hash) is not None,
            f"MID-360 source hash has not been adopted: {source_id}",
        )
        path = MID360_RESULT_ROOT / relative
        require(path.is_file(), f"missing pinned MID-360 source: {path}")
        actual_hash = sha256(path)
        require(
            actual_hash == expected_hash,
            f"pinned MID-360 source hash mismatch for {source_id}: "
            f"expected={expected_hash} actual={actual_hash}",
        )
        sources[source_id] = path
    for stage, expected_hash in MID360_STAGE_SUMMARY_CONTRACT.items():
        source_id = f"stage_summary_{stage}"
        path = MID360_RESULT_ROOT / "summaries" / f"{stage}.json"
        require(path.is_file(), f"missing pinned MID-360 source: {path}")
        require(
            sha256(path) == expected_hash,
            f"pinned MID-360 source hash mismatch for {source_id}",
        )
        sources[source_id] = path
    return sources


def parse_run_environment(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), 1
    ):
        if not raw_line or raw_line.startswith("#"):
            continue
        match = re.fullmatch(r"([a-z][a-z0-9_]*)=(.*)", raw_line)
        require(match is not None, f"{path}:{line_number}: malformed run.env line")
        key, value = match.groups()
        require(key not in values, f"{path}:{line_number}: duplicate key {key}")
        require("\x00" not in value, f"{path}:{line_number}: invalid NUL byte")
        values[key] = value
    require(values, f"{path}: empty run environment")
    return values


def require_mid360_runtime(runtime: dict[str, Any], context: str) -> None:
    checks = runtime.get("checks")
    require(isinstance(checks, list), f"{context}: missing runtime checks")
    require(
        tuple(item.get("name") for item in checks) == MID360_RUNTIME_CHECKS,
        f"{context}: runtime check schema changed",
    )
    require(
        all(item.get("category") == "hard" and item.get("passed") is True for item in checks),
        f"{context}: every runtime hard check must pass",
    )
    inputs = runtime.get("inputs", {})
    frames = inputs.get("frames", {})
    clock = runtime.get("clock", {})
    deskew = runtime.get("deskew", {})
    imu = runtime.get("imu", {})
    odometry = runtime.get("odometry", {})
    require(
        runtime.get("passed") is True
        and runtime.get("sensor") == "mid360"
        and inputs.get("points_topic") == "/livox/lidar"
        and inputs.get("imu_topic") == "/livox/imu"
        and inputs.get("recorded_tf_used") == "false"
        and frames == {"imu": "livox_frame", "points": "livox_frame"}
        and abs(float(clock.get("observed_sim_over_wall_rate")) - 1.0) <= 1.0e-5
        and deskew.get("enabled") is True
        and float(deskew.get("success_rate")) >= 0.99
        and int(deskew.get("accepted")) >= 2942
        and int(deskew.get("rejected")) <= 11
        and float(imu.get("exact_stamp_coverage")) == 1.0
        and abs(float(imu.get("corrected_acceleration_norm_median_mps2")) - 9.754307250256623)
        <= 1.0e-12
        and int(odometry.get("accepted_scan_messages")) > 0
        and float(odometry.get("scan_registration_density")) >= 0.95,
        f"{context}: runtime evidence does not satisfy the MID-360 contract",
    )


def require_identity_transform_log(path: Path, child_frame: str) -> None:
    text = path.read_text(encoding="utf-8", errors="strict")
    translation_matches = re.findall(r"translation:\s*(\([^\n]+\))", text)
    rotation_matches = re.findall(r"rotation:\s*(\([^\n]+\))", text)
    frame_matches = re.findall(r"from '([^']+)' to '([^']+)'", text)
    require(
        len(translation_matches) == len(rotation_matches) == len(frame_matches) == 1,
        f"{path}: transform evidence must contain exactly one published transform",
    )
    translation = tuple(float(value) for value in ast.literal_eval(translation_matches[0]))
    rotation = tuple(float(value) for value in ast.literal_eval(rotation_matches[0]))
    require(
        translation == (0.0, 0.0, 0.0)
        and rotation == (0.0, 0.0, 0.0, 1.0)
        and frame_matches[0] == ("base_link", child_frame),
        f"{path}: expected identity base_link->{child_frame} transform",
    )


def require_mid360_parameter_contract(
    imu_path: Path, odometry_path: Path, environment: dict[str, str], context: str
) -> None:
    require(
        sha256(imu_path) == environment.get("imu_param_sha256")
        and sha256(odometry_path) == environment.get("odom_override_sha256"),
        f"{context}: run.env parameter hashes do not match copied artifacts",
    )
    imu_document = yaml.safe_load(imu_path.read_text(encoding="utf-8"))
    odometry_document = yaml.safe_load(odometry_path.read_text(encoding="utf-8"))
    require(isinstance(imu_document, dict), f"{context}: invalid IMU YAML")
    require(isinstance(odometry_document, dict), f"{context}: invalid odometry YAML")
    imu_parameters = imu_document.get("imu_undistorter", {}).get("ros__parameters", {})
    odometry_parameters = odometry_document.get("/**", {}).get("ros__parameters", {})
    required_imu_parameters = {
        "base_frame": "base_link",
        "imu_frame": "livox_frame",
        "scan_frame": "livox_frame",
        "imu_topic": "/imu",
        "time_fields": ["timestamp"],
        "prefer_relative_time": False,
        "time_scale": 1.0e-9,
        "allow_linear_time_fallback": False,
        "cloud_stamp_is_start": True,
        "reference_time": "start",
        "max_imu_gap_sec": 0.02,
        "max_imu_boundary_gap_sec": 0.02,
        "use_translation": False,
        "use_twist_speed": False,
        "allow_default_speed_fallback": False,
    }
    require(
        all(imu_parameters.get(key) == value for key, value in required_imu_parameters.items()),
        f"{context}: internal-IMU deskew frame/time/gap policy changed",
    )
    required_odometry_parameters = {
        "imu_corrected.linear_acceleration_scale": 9.80665,
        "imu_corrected.max_sample_gap_sec": 0.02,
        "imu_corrected.max_boundary_gap_sec": 0.02,
        "lidar_odom.accepted_scan_odom.enable": True,
        "lidar_odom.accepted_scan_odom.topic": "/localization/gyro_lidar_odom_scan",
        "gyro_bias.enable": False,
        "gyro_bias.initial_bg_rad_s": 0.006959692,
        "lidar_odom.smoother.enable": False,
    }
    require(
        all(
            odometry_parameters.get(key) == value
            for key, value in required_odometry_parameters.items()
        ),
        f"{context}: internal-IMU odometry scale/gap/bias policy changed",
    )


def require_mid360_structural_validation(validation: dict[str, Any]) -> None:
    checks = validation.get("checks")
    require(isinstance(checks, list), "MID-360 structural validation lacks checks")
    require(
        tuple(item.get("name") for item in checks) == MID360_STRUCTURAL_CHECKS
        and all(
            item.get("category") == "hard" and item.get("passed") is True
            for item in checks
        ),
        "MID-360 structural validation must pass every reviewed hard check",
    )
    expected_counts = {
        "global_topics": {
            "/localization/precision_global_odom": 0,
            "/localization/precision_global_pose": 0,
        },
        "precision_local": 2926,
        "precision_local_coverage": 0.9996583532627263,
        "precision_local_eligible_raw": 2927,
        "precision_local_matched": 2926,
        "raw_accepted": 2927,
        "submap_corrections": 444,
        "submap_scans": 586,
        "submap_streams": 1,
    }
    expected_method = {
        "association": (
            "one-to-one accepted physical header stamps; nearest <=1us only for "
            "epoch/double serialization ULP; no odometry interpolation"
        ),
        "global_policy": "precision global outputs and all global authorities stay zero",
        "precision_coverage_denominator": (
            "raw accepted-scan poses at/after first SubmapScan epoch"
        ),
        "precision_local_topic": "/localization/precision_local_odom",
        "raw_topic": "/localization/gyro_lidar_odom_scan",
    }
    expected_metrics = {
        "matcher_accepted": 463,
        "matcher_accepted_ratio": 0.847985347985348,
        "matcher_attempted": 546,
        "matcher_latency_p99_ms": 140.265841,
        "matcher_processing_p99_ms": 26.278551,
        "matcher_queue_drop_count": 0,
        "matcher_rejected": 83,
        "post_warmup_correction_ratio": 0.7668393782383419,
        "precision_local_stamp_delta_ns_maximum": 0,
        "scan_stamp_delta_ns_maximum": 0,
        "snapshot_conversion_max_ms": 0.026284,
        "warmup_sec": 3.400035072,
    }
    require(
        validation.get("passed") is True
        and validation.get("failed_hard_checks") == []
        and validation.get("counts") == expected_counts
        and validation.get("method") == expected_method
        and validation.get("metrics") == expected_metrics,
        "MID-360 structural counters/method/metrics differ from reviewed evidence",
    )


def validate_mid360_evidence(sources: dict[str, Path]) -> dict[str, Any]:
    evaluation = read_json(sources["evaluation"])
    final_status = read_json(sources["final_status"])
    revalidation = read_json(sources["status_revalidation"])
    holdout = read_json(sources["holdout_metrics"])
    plan = read_json(sources["normalized_plan"])
    structural = read_json(sources["precision_structural_validation"])
    control_runtime = read_json(sources["control_runtime_metrics"])
    precision_runtime = read_json(sources["precision_runtime_metrics"])
    control_environment = parse_run_environment(sources["control_run_environment"])
    precision_environment = parse_run_environment(sources["precision_run_environment"])

    shared_environment = {
        "sensor": "mid360",
        "glim_traj_sha256": MID360_GLIM_TRAJECTORY_SHA256,
        "deskew": "on",
        "gicp_epsilon": "default",
        "mid_yaw_policy": "fixed-bias-direct",
        "use_deskew": "true",
        "rate": "1.0",
        "points_topic": "/livox/lidar",
        "imu_topic": "/livox/imu",
        "imu_sensor": "internal",
        "imu_acceleration_scale": "9.80665",
        "deskew_success_minimum": "0.99",
        "input_bag_metadata_sha256": (
            "0be5944b5482b4ab22038f97782984d53ce96cefc013229e5ac772401ec4155b"
        ),
        "input_bag_storage_files": "rosbag2_2026_04_08-22_52_42_0.mcap",
        "input_bag_storage_sha256": (
            "b2e7d0bc155c880526dcb9e4a1f496d438d5c1c0cde605fe658da39c1b24b856"
        ),
        "imu_param_sha256": MID360_SOURCE_CONTRACT["control_imu_parameters"][1],
        "odom_override_sha256": MID360_SOURCE_CONTRACT[
            "control_odometry_override"
        ][1],
        "odom_tuning_override_sha256": MID360_SOURCE_CONTRACT[
            "control_odometry_tuning"
        ][1],
        "recorded_tf_used": "false",
        "gnss_started": "false",
        "map_odom_fusion_started": "false",
        "runtime_analysis_status": "0",
        "evaluation_status": "0",
    }
    for context, environment in (
        ("control", control_environment),
        ("precision", precision_environment),
    ):
        require(
            all(environment.get(key) == value for key, value in shared_environment.items()),
            f"MID-360 {context} run.env violates the reviewed internal-IMU contract",
        )
    require(
        control_environment.get("localization_mode") == "baseline"
        and control_environment.get("snapshot_interval") == "n/a"
        and control_environment.get("evaluation_topic")
        == "/localization/gyro_lidar_odom_scan"
        and control_environment.get("submap_validation_executed") == "false"
        and control_environment.get("snapshot_override_sha256") == "n/a"
        and control_environment.get("matcher_param_sha256") == "n/a"
        and control_environment.get("matcher_override_param_sha256") == "n/a"
        and precision_environment.get("localization_mode") == "precision"
        and precision_environment.get("snapshot_interval") == "5"
        and precision_environment.get("evaluation_topic")
        == "/localization/precision_local_odom"
        and precision_environment.get("submap_validation_executed") == "true"
        and precision_environment.get("submap_validation_status") == "0"
        and precision_environment.get("snapshot_override_sha256")
        == MID360_SOURCE_CONTRACT["precision_snapshot_parameters"][1]
        and precision_environment.get("matcher_param_sha256")
        == MID360_SOURCE_CONTRACT["precision_matcher_parameters"][1]
        and precision_environment.get("matcher_override_param_sha256")
        == MID360_SOURCE_CONTRACT["precision_matcher_tuning"][1],
        "MID-360 control/precision mode or interval-5 snapshot policy changed",
    )
    for shared_path_key in ("bag", "glim_dir", "glim_traj"):
        require(
            control_environment.get(shared_path_key)
            == precision_environment.get(shared_path_key),
            f"MID-360 A/B runs use different {shared_path_key}",
        )

    require_mid360_runtime(control_runtime, "MID-360 control")
    require_mid360_runtime(precision_runtime, "MID-360 precision")
    require_mid360_structural_validation(structural)
    require(
        evaluation.get("structural_validation") == structural,
        "MID-360 A/B result embeds a different structural validation result",
    )

    for prefix, environment in (
        ("control", control_environment),
        ("precision", precision_environment),
    ):
        require_identity_transform_log(
            sources[f"{prefix}_lidar_transform_log"], "livox_frame"
        )
        require_mid360_parameter_contract(
            sources[f"{prefix}_imu_parameters"],
            sources[f"{prefix}_odometry_override"],
            environment,
            f"MID-360 {prefix}",
        )
    require(
        sha256(sources["control_imu_parameters"])
        == sha256(sources["precision_imu_parameters"])
        and sha256(sources["control_odometry_override"])
        == sha256(sources["precision_odometry_override"]),
        "MID-360 control and precision runs used different IMU/odometry artifacts",
    )
    require(
        sha256(sources["control_odometry_tuning"])
        == sha256(sources["precision_odometry_tuning"])
        and sha256(sources["control_odometry_tuning"])
        == "465101aeb98d0eefddd2f41d6baf56e821b5b939b5e49127535bb78320ddeefc"
        and sha256(sources["precision_matcher_tuning"])
        == "7db51bec16ed2a4bad41857bf51dde33ae04c08efa94b0ef90f614341f39f821",
        "MID-360 accepted tuning artifacts changed",
    )
    accepted_root = (
        ROOT
        / "src/pure_odometry_bringup/config/evaluation/lidar_imu/mid360"
        / "rosbag2_2026_04_08-22_52_42/accepted"
    )
    packaged_profiles = {
        accepted_root / "deskew.yaml": sources["control_imu_parameters"],
        accepted_root / "odom_fixed_bias_direct.yaml": sources[
            "control_odometry_override"
        ],
        accepted_root / "odom_tuned.yaml": sources["control_odometry_tuning"],
        accepted_root / "submap_matcher_tuned.yaml": sources[
            "precision_matcher_tuning"
        ],
        ROOT / "src/pure_precision_bringup/config/submap_snapshot_override.yaml": sources[
            "precision_snapshot_parameters"
        ],
        ROOT / "src/pure_lidar_submap_matcher/param/param.yaml": sources[
            "precision_matcher_parameters"
        ],
    }
    require(
        all(
            packaged.is_file() and sha256(packaged) == sha256(source)
            for packaged, source in packaged_profiles.items()
        ),
        "MID-360 packaged accepted profiles differ from final-run artifacts",
    )

    stage_selection = {
        stage: read_json(sources[f"stage_summary_{stage}"])
        for stage in MID360_STAGE_SUMMARY_CONTRACT
    }
    require(
        final_status.get("status") == "ACCEPTED"
        and final_status.get("completed") is True
        and final_status.get("candidate_adopted") is True
        and final_status.get("formal_ab_passed") is True
        and final_status.get("formal_ab_status") == 0
        and final_status.get("adopted_winner", {}).get("id") == "c08"
        and final_status.get("adopted_winner", {}).get("effective_runner")
        == {"snapshot_interval": 5}
        and final_status.get("status_revalidation", {}).get("receipt_sha256")
        == MID360_SOURCE_CONTRACT["status_revalidation"][1]
        and revalidation.get("operation") == "final-status-only-revalidation"
        and revalidation.get("replay_executed") is False
        and revalidation.get("candidate_metrics_recomputed") is False
        and revalidation.get("candidate_selection_changed") is False
        and revalidation.get("no_parameter_or_data_changes") is True
        and revalidation.get("formal_ab", {}).get("all_hard_checks_passed") is True
        and revalidation.get("formal_ab", {}).get("hard_check_count") == 20
        and revalidation.get("formal_ab", {}).get("cadence_expected") == 5
        and holdout.get("opened_after_all_stages_completed") is True
        and holdout.get("selection_score_used_holdout") is False
        and plan.get("dataset", {}).get("id") == "mid360_internal_imu_2026_04_08"
        and sum(item.get("candidate_count", 0) for item in stage_selection.values())
        == 48
        and all(
            item.get("completed") is True
            and item.get("candidate_adopted") is True
            and item.get("selection_status") == "ADOPTED"
            for item in stage_selection.values()
        ),
        "MID-360 tuning selection, holdout, or status-revalidation contract changed",
    )

    checks = evaluation.get("checks")
    require(isinstance(checks, list), "MID-360 A/B result lacks hard gates")
    failed_hard = [
        {"name": item.get("name"), "detail": item.get("detail")}
        for item in checks
        if item.get("category") == "hard" and item.get("passed") is not True
    ]
    require(
        evaluation.get("passed") is True
        and evaluation.get("sensor") == "mid360"
        and evaluation.get("label") == "Livox MID-360 + internal IMU"
        and sum(item.get("category") == "hard" for item in checks) == 20
        and failed_hard == []
        and all(
            item.get("passed") is True
            for item in checks
            if item.get("category") != "hard"
        ),
        "MID-360 A/B must pass every reviewed hard gate",
    )
    require(
        evaluation.get("configuration_contract")
        == {
            "mid_fixed_bias_direct": True,
            "mid_policy_accepted": True,
            "mismatches": {},
            "passed": True,
            "precision_matcher_provenance_present": True,
            "provenance_present": True,
        },
        "MID-360 A/B configuration contract changed",
    )
    accuracy = evaluation.get("accuracy", {})
    metrics = accuracy.get("metrics", {})
    primary_alignment = accuracy.get("primary_alignment", {})
    require(
        set(metrics) == {"control_raw", "precision_raw", "precision_local"}
        and primary_alignment.get("type") == "single_pose_exact_se2"
        and primary_alignment.get("anchor_native_common_all_streams") is True
        and primary_alignment.get("anchor_physical_stamp_common_all_streams") is True
        and primary_alignment.get("scale_estimated_or_applied") is False
        and primary_alignment.get("shared_across")
        == ["control_raw", "precision_raw", "precision_local"]
        and primary_alignment.get("exact_anchor_residual", {}).get("same_stamp") is True
        and primary_alignment.get("exact_anchor_residual", {}).get("position_m")
        <= 1.0e-12
        and primary_alignment.get("exact_anchor_residual", {}).get("yaw_rad")
        <= 1.0e-12
        and all(
            float(metrics[name].get("glim_association_coverage")) >= 0.95
            for name in metrics
        )
        and 0.95 <= float(metrics["precision_local"]["path_ratio_estimate_over_reference"])
        <= 1.05,
        "MID-360 alignment, association, or accepted path-ratio contract changed",
    )
    gain = accuracy.get("comparison", {}).get("end_to_end_gain", {})
    require(
        float(gain.get("position_rmse_m", {}).get("improvement_percent")) > 0.0
        and float(gain.get("yaw_rmse_deg", {}).get("improvement_percent")) > 0.0
        and float(gain.get("endpoint_position_error_m", {}).get("improvement_percent"))
        > 0.0
        and float(
            gain.get("rpe", {})
            .get("10", {})
            .get("translation_rmse_m", {})
            .get("improvement_percent")
        )
        > 0.0
        and float(
            gain.get("rpe", {})
            .get("10", {})
            .get("yaw_rmse_deg", {})
            .get("improvement_percent")
        )
        > 0.0,
        "MID-360 scan-to-submap improvement contract changed",
    )

    csv_streams: dict[str, list[tuple[float, float, float, float]]] = {}
    with sources["aligned_samples"].open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        require(
            reader.fieldnames
            == [
                "stream",
                "stamp_ns",
                "time_from_primary_anchor_sec",
                "reference_x",
                "reference_y",
                "reference_yaw_rad",
                "aligned_x",
                "aligned_y",
                "aligned_yaw_rad",
                "position_error_m",
                "yaw_error_deg",
            ],
            "MID-360 aligned CSV schema changed",
        )
        for row in reader:
            require(
                row["stream"]
                in {"control_raw", "precision_raw", "precision_local"},
                "MID-360 aligned CSV contains an unexpected stream",
            )
            numeric_values = [float(row[column]) for column in reader.fieldnames[1:]]
            require(
                all(math.isfinite(value) for value in numeric_values),
                "MID-360 aligned CSV contains a non-finite numeric value",
            )
            csv_streams.setdefault(row["stream"], []).append(
                (
                    numeric_values[2],
                    numeric_values[3],
                    numeric_values[5],
                    numeric_values[6],
                )
            )
    require(
        set(csv_streams) == {"control_raw", "precision_raw", "precision_local"},
        "MID-360 aligned CSV lacks one of the reviewed A/B streams",
    )
    require(
        all(
            len(csv_streams[stream_name]) == int(metrics[stream_name]["samples"])
            for stream_name in ("control_raw", "precision_raw", "precision_local")
        ),
        "MID-360 aligned CSV row counts differ from the A/B metrics",
    )

    micro_motion: dict[str, dict[str, Any]] = {}
    for stream_name in ("control_raw", "precision_local"):
        samples = csv_streams[stream_name]
        interval_count = 0
        reference_path_m = 0.0
        estimate_path_m = 0.0
        for previous, current in zip(samples, samples[1:]):
            reference_step = math.hypot(
                current[0] - previous[0], current[1] - previous[1]
            )
            if reference_step >= 0.002:
                continue
            interval_count += 1
            reference_path_m += reference_step
            estimate_path_m += math.hypot(
                current[2] - previous[2], current[3] - previous[3]
            )
        micro_motion[stream_name] = {
            "interval_count": interval_count,
            "reference_path_m": reference_path_m,
            "estimate_path_m": estimate_path_m,
        }
    expected_micro_motion = {
        "control_raw": {
            "interval_count": 263,
            "reference_path_m": 0.16677875544589313,
            "estimate_path_m": 0.5608420287798032,
        },
        "precision_local": {
            "interval_count": 263,
            "reference_path_m": 0.16677875544589313,
            "estimate_path_m": 0.6077103925815106,
        },
    }
    require(
        micro_motion == expected_micro_motion,
        "MID-360 micro-motion path audit changed from the reviewed CSV",
    )
    return {
        "evaluation": evaluation,
        "structural": structural,
        "control_runtime": control_runtime,
        "precision_runtime": precision_runtime,
        "micro_motion": micro_motion,
        "holdout": holdout,
    }


def retire_mid360_assets() -> None:
    directory = ASSETS / MID360_RETIRED_DATASET_ID
    if not directory.exists():
        return
    require(directory.is_dir() and not directory.is_symlink(), "unsafe retired MID-360 path")
    entries = list(directory.iterdir())
    require(
        all(path.is_file() and not path.is_symlink() for path in entries)
        and {path.name for path in entries} == MID360_PUBLICATION_FILENAMES,
        "retired MID-360 directory contains an unexpected file",
    )
    for path in entries:
        path.unlink()
    directory.rmdir()


def curate_lidar() -> list[dict[str, Any]]:
    result_root = ROOT / "test_results"
    vel_run = result_root / "lidar_imu_clean_prefix_20260812" / "velodyne_pre_first_tracking_fatal_precision_i2" / "ab_initial_pose"
    vel_full = result_root / "lidar_imu_submap_20260812" / "velodyne_default_i2_full" / "ab_initial_pose_v4"
    records: list[dict[str, Any]] = []

    vel_output = ASSETS / "velodyne_32line_external_imu"
    vel = read_json(vel_run / "evaluation.json")
    full = read_json(vel_full / "evaluation.json")
    render_lidar(
        vel_run / "aligned_samples.csv", vel_output, VELODYNE_LABEL,
        ("control_raw", "precision_local"),
        {"control_raw": "Scan-to-scan", "precision_local": "Scan-to-submap"},
    )
    write_json(
        vel_output / "metrics.json",
        {
            "schema_version": 1,
            "dataset_label": VELODYNE_LABEL,
            "reference": {"method": "GLIM trajectory", "independent_ground_truth": False},
            "publication_status": "accepted clean prefix; full-run robustness failed",
            "alignment": {
                "type": "single shared exact first-common-pose SE(2)",
                "position_and_yaw_share_transform": True,
                "scale_applied": False,
            },
            "clean_prefix": {
                "passed": vel["passed"],
                "duration_sec": vel["accuracy"]["common_interval"]["duration_sec"],
                "streams": {
                    "scan_to_scan": lidar_stream(vel["accuracy"]["metrics"]["control_raw"]),
                    "scan_to_submap": lidar_stream(vel["accuracy"]["metrics"]["precision_local"]),
                },
                "comparison": vel["accuracy"]["comparison"]["end_to_end_gain"],
            },
            "full_run_robustness": {
                "passed": full["passed"],
                "interpretation": "Both modes lose absolute tracking after the first fatal generation transition; the submap branch improves the error but does not pass absolute acceptance.",
                "streams": {
                    "scan_to_scan": lidar_stream(full["accuracy"]["metrics"]["control_raw"]),
                    "scan_to_submap": lidar_stream(full["accuracy"]["metrics"]["precision_local"]),
                },
            },
        },
    )
    records.append(
        {
            "directory": vel_output,
            "sources": [
                ("clean_prefix_evaluation.json", vel_run / "evaluation.json"),
                ("clean_prefix_aligned_samples.csv", vel_run / "aligned_samples.csv"),
                ("full_run_evaluation.json", vel_full / "evaluation.json"),
            ],
            "provenance": {
                "source_run_id": "velodyne_pre_first_tracking_fatal_precision_i2",
                "full_run_id": "velodyne_default_i2_full",
            },
        }
    )

    mid_sources = pinned_mid360_sources()
    mid_evidence = validate_mid360_evidence(mid_sources)
    mid = mid_evidence["evaluation"]
    mid_output = ASSETS / MID360_DATASET_ID
    if mid_output.is_dir():
        unexpected_files = {
            path.name for path in mid_output.iterdir() if path.is_file()
        } - MID360_PUBLICATION_FILENAMES
        require(
            not unexpected_files,
            f"{MID360_DATASET_ID}: unexpected publication files: "
            f"{sorted(unexpected_files)}",
        )
    render_lidar(
        mid_sources["aligned_samples"],
        mid_output,
        MID360_PLOT_TITLE,
        ("control_raw", "precision_local"),
        {
            "control_raw": "Scan-to-scan",
            "precision_local": "Scan-to-submap",
        },
    )
    metrics = mid["accuracy"]["metrics"]
    gain = mid["accuracy"]["comparison"]["end_to_end_gain"]
    structural = mid_evidence["structural"]

    def runtime_summary(runtime: dict[str, Any]) -> dict[str, Any]:
        return {
            "status": "PASS",
            "playback_rate_observed": runtime["clock"][
                "observed_sim_over_wall_rate"
            ],
            "deskew": {
                "successful": runtime["deskew"]["accepted"],
                "rejected": runtime["deskew"]["rejected"],
                "success_rate": runtime["deskew"]["success_rate"],
            },
            "imu": {
                "exact_stamp_coverage": runtime["imu"]["exact_stamp_coverage"],
                "corrected_acceleration_norm_median_mps2": runtime["imu"][
                    "corrected_acceleration_norm_median_mps2"
                ],
            },
            "accepted_scan_odometry": {
                "messages": runtime["odometry"]["accepted_scan_messages"],
                "registration_density": runtime["odometry"][
                    "scan_registration_density"
                ],
            },
        }

    micro_motion = mid_evidence["micro_motion"]
    write_json(
        mid_output / "metrics.json",
        {
            "schema_version": 2,
            "dataset_label": MID360_LABEL,
            "publication_status": "ACCEPTED",
            "accepted": True,
            "reference": {
                "method": "GLIM trajectory",
                "independent_ground_truth": False,
                "shares_lidar_and_imu_observations": True,
            },
            "sensor_configuration": {
                "lidar": "Livox MID-360",
                "imu": "MID-360 internal IMU",
                "points_topic": "/livox/lidar",
                "imu_topic": "/livox/imu",
                "imu_acceleration_scale": 9.80665,
                "gyro_z_bias_rad_s": 0.006959692,
                "gyro_bias_adaptation_enabled": False,
                "gnss_used": False,
                "recorded_tf_used": False,
                "static_transforms": {
                    "base_link_to_livox_frame": {
                        "translation_m": [0.0, 0.0, 0.0],
                        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                },
                "playback_rate": 1.0,
                "submap_snapshot_interval_frames": 5,
            },
            "alignment": {
                "type": "single shared exact first-common-pose SE(2)",
                "position_and_yaw_share_transform": True,
                "scale_applied": False,
                "estimate_interpolation": False,
                "reference_interpolation": "GLIM only",
            },
            "evaluation": {
                "overall_ab_passed": True,
                "hard_gate_count": 20,
                "failed_hard_gates": [],
                "duration_sec": mid["accuracy"]["common_interval"]["duration_sec"],
                "streams": {
                    "scan_to_scan": lidar_stream(metrics["control_raw"]),
                    "scan_to_submap": lidar_stream(metrics["precision_local"]),
                },
                "scan_to_submap_gain": gain,
            },
            "holdout_validation": {
                "selection_score_used_holdout": False,
                "opened_after_all_stages_completed": True,
                "scan_to_scan": mid_evidence["holdout"]["scan_to_scan"]["holdout"],
                "scan_to_submap": mid_evidence["holdout"]["scan_to_submap"]["holdout"],
            },
            "micro_motion_path_audit": {
                "reference_step_threshold_m_exclusive": 0.002,
                "definition": (
                    "Consecutive samples whose GLIM reference XY step is below "
                    "the threshold; reference and estimate paths are summed "
                    "independently within each published stream."
                ),
                "streams": {
                    "scan_to_scan": micro_motion["control_raw"],
                    "scan_to_submap": micro_motion["precision_local"],
                },
            },
            "runtime_validation": {
                "control_scan_to_scan": runtime_summary(
                    mid_evidence["control_runtime"]
                ),
                "precision_scan_to_submap": runtime_summary(
                    mid_evidence["precision_runtime"]
                ),
            },
            "structural_validation": {
                "status": "PASS",
                "failed_hard_checks": [],
                "counts": structural["counts"],
                "matcher": {
                    key: structural["metrics"][key]
                    for key in (
                        "matcher_attempted",
                        "matcher_accepted",
                        "matcher_rejected",
                        "matcher_accepted_ratio",
                        "matcher_processing_p99_ms",
                        "matcher_latency_p99_ms",
                        "matcher_queue_drop_count",
                        "post_warmup_correction_ratio",
                    )
                },
            },
            "source_validation": {
                "source_hashes_pinned": True,
                "configuration_contract": "PASS",
                "packaged_profile_contract": "PASS",
                "control_runtime": "PASS",
                "precision_runtime": "PASS",
                "precision_structure": "PASS",
                "identity_transform_contract": "PASS",
                "tuning_candidate_count": 48,
                "tuning_stage_count": 9,
                "holdout_selection_isolation": "PASS",
                "status_revalidated_without_replay": True,
                "a_b_hard_gate_status": "PASS",
            },
        },
    )
    require(
        {path.name for path in mid_output.iterdir() if path.is_file()}
        == MID360_PUBLICATION_FILENAMES,
        f"{MID360_DATASET_ID}: publication must contain exactly four files",
    )
    retire_mid360_assets()
    records.append(
        {
            "directory": mid_output,
            "sources": [
                (source_id, path) for source_id, path in mid_sources.items()
            ],
            "provenance": {
                "publication_status": "ACCEPTED",
                "accepted": True,
                "failed_hard_gates": [],
                "sensor_contract": "MID-360 internal IMU; identity LiDAR/IMU frame; no GNSS",
                "submap_snapshot_policy": "interval 5 accepted frames",
                "canonical_modes": {
                    "control": "scan_to_scan",
                    "candidate": "rolling_scan_to_submap_interval_5_frames",
                },
                "evaluation_method": (
                    "shared exact initial pose; native accepted-scan estimates; "
                    "GLIM-only reference interpolation"
                ),
            },
        }
    )
    return records


def curate_gnss() -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for course in GNSS_SOURCE_CONTRACTS:
        contract = GNSS_SOURCE_CONTRACTS[course]
        sources = pinned_gnss_sources(course)
        evidence = validate_gnss_evidence(course, sources)
        label = COURSE_LABELS[course]
        data = evidence["result"]
        startup = evidence["startup"]
        accepted_scan = evidence["accepted_scan"]
        output = ASSETS / f"hesai_32line_imu_rtk_gnss_{course}"
        local_csv = sources["aligned_local_samples"]
        global_csv = sources["aligned_global_samples"]
        if output.is_dir():
            unexpected_files = {
                path.name for path in output.iterdir() if path.is_file()
            } - GNSS_PUBLICATION_FILENAMES
            require(
                not unexpected_files,
                f"{output.name}: forbidden Hesai publication files: "
                f"{sorted(unexpected_files)}",
            )
        error_plot_timeline = gnss_error_plot_timeline(
            global_csv,
            data,
            contract["expected_error_plot_timeline"],
        )
        render_gnss_errors(local_csv, output, label, "local")
        render_gnss_errors(
            global_csv,
            output,
            label,
            "global",
            error_plot_timeline,
        )
        evaluations = data["evaluations"]
        failed_hard_gate_details = [
            {"name": item["name"], "detail": item["detail"]}
            for item in data["checks"]
            if item.get("category") == "hard" and not item.get("passed", False)
        ]
        outage_accuracy = {
            name: {
                "samples": data["outage"][name]["samples"],
                "xy_error_m": stats(data["outage"][name]["xy_error_m"]),
                "yaw_error_deg": stats(data["outage"][name]["yaw_error_deg"]),
            }
            for name in (
                "precision_raw",
                "precision_local",
                "precision_existing",
                "precision_global",
            )
        }
        published = {
            "schema_version": 2,
            "dataset_label": label,
            "publication_status": contract["publication_status"],
            "publication_acceptance": {
                "accepted": contract["accepted"],
                "accuracy_result_status": "PASS" if data["passed"] else "FAIL",
                "hard_gate_count": data["hard_gate_count"],
                "failed_hard_gate_count": data["failed_hard_gate_count"],
                "failed_hard_gates": failed_hard_gate_details,
            },
            "reference": {
                "method": "GLIM trajectory (correlated LiDAR/IMU reference)",
                "independent_ground_truth": False,
            },
            "nmea_projection": {
                **GNSS_DEFAULT_PROJECTION,
                "runtime_parameter_source": (
                    "pure_nmea_gnss_conversion/param/param.yaml"
                ),
                "metadata_source": (
                    "pure_nmea_gnss_conversion/config/map_projector_info.yaml"
                ),
                "parameters_match_metadata": True,
                "evaluation_override_applied": False,
                "provenance_verified": True,
            },
            "method": {
                "scope": (
                    "primary GLIM accuracy; startup and accepted-scan "
                    "non-intrusion use their dedicated contracts"
                ),
                "estimator_association": data["method"][
                    "accuracy_estimator_association"
                ],
                "reference_association": data["method"][
                    "accuracy_reference_association"
                ],
                "timestamps": data["method"]["timestamps"],
                "speed_run_role": data["method"]["speed_run_role"],
            },
            "alignment": {
                "scope": "primary GLIM accuracy",
                "type": "single complete exact first-common-pose SE(2)",
                "local": data["method"]["local_alignment"],
                "global": data["method"]["global_alignment"],
                "yaw": data["method"]["yaw_alignment"],
                "position_and_yaw_share_transform": True,
                "scale_applied": False,
                "exact_initial_pose": True,
            },
            "canonical_runs": {
                mode: value["run_id"]
                for mode, value in evidence["provenance_summary"].items()
            },
            "source_validation": {
                "accuracy_result": "PASS" if data["passed"] else "FAIL",
                "startup_acceptance": "PASS",
                "startup_window_provenance": evidence[
                    "startup_window_provenance"
                ],
                "runtime_validation": evidence["runtime_validation"],
                "accepted_scan_nonintrusion": "PASS",
                "provenance": evidence["provenance_summary"],
                "source_hashes_pinned": True,
            },
            "outage_yaw_guard": evidence["outage_yaw_guard"],
            "local": {
                "scan_to_scan": gnss_stream(evaluations["precision_raw"]),
                "scan_to_submap": gnss_stream(evaluations["precision_local"]),
                "comparison": {
                    "fixed_shared_xy_rmse_improvement_percent": data[
                        "local_comparison"
                    ]["fixed_xy_rmse_m"]["improvement_percent"],
                    "fixed_shared_yaw_rmse_improvement_percent": data[
                        "local_comparison"
                    ]["yaw_rmse_deg"]["improvement_percent"],
                    "full_shape_xy_rmse_improvement_percent": data[
                        "local_comparison"
                    ]["full_shape_xy_rmse_m"]["improvement_percent"],
                    "full_shape_yaw_rmse_improvement_percent": data[
                        "local_comparison"
                    ]["full_shape_yaw_rmse_deg"]["improvement_percent"],
                },
            },
            "global": {
                "existing_fusion": gnss_stream(evaluations["precision_existing"]),
                "precision_global": gnss_stream(evaluations["precision_global"]),
                "comparison": {
                    "fixed_shared_xy_rmse_improvement_percent": data[
                        "global_comparison"
                    ]["fixed_xy_rmse_m"]["improvement_percent"],
                    "fixed_shared_yaw_rmse_improvement_percent": data[
                        "global_comparison"
                    ]["yaw_rmse_deg"]["improvement_percent"],
                    "full_shape_xy_rmse_improvement_percent": data[
                        "global_comparison"
                    ]["full_shape_xy_rmse_m"]["improvement_percent"],
                    "full_shape_yaw_rmse_improvement_percent": data[
                        "global_comparison"
                    ]["full_shape_yaw_rmse_deg"]["improvement_percent"],
                },
            },
            "outage": {
                "absolute_timestamps_published": False,
                "duration_sec": data["outage"]["duration_sec"],
                "definition": (
                    "longest post-initialization non-TRACKING interval reported "
                    "by gnss_map_odom_fusion in the canonical scan-to-submap "
                    "run; distinct from the raw loss of usable GNSS input"
                ),
                "streams": outage_accuracy,
            },
            "error_plot_annotations": error_plot_timeline,
            "supporting_acceptance": {
                "startup": {
                    "status": "PASS",
                    "calibration_window_provenance": evidence[
                        "startup_window_provenance"
                    ],
                    "first_global_delay_sec": startup["startup"][
                        "first_global_delay_sec"
                    ],
                    "first_global_existing_yaw_difference_deg": startup["startup"][
                        "first_global_existing_yaw_difference_deg"
                    ],
                },
                "accepted_scan_nonintrusion": {
                    "status": "PASS",
                    "failed_reference_only_warning_count": contract[
                        "expected_nonintrusion_failed_warning_count"
                    ],
                    "counts": accepted_scan["counts"],
                    "phase_compensated_coverage": accepted_scan["metrics"][
                        "phase_compensated_coverage"
                    ],
                    "phase_compensated_xy_rmse_m": accepted_scan["metrics"][
                        "phase_compensated_xy_difference_m"
                    ]["rmse"],
                    "phase_compensated_yaw_rmse_deg": accepted_scan["metrics"][
                        "phase_compensated_yaw_difference_deg"
                    ]["rmse"],
                    "exact_physical_stamp_coverage_reference_only": accepted_scan[
                        "counts"
                    ]["coverage"],
                    "interpretation": (
                        "PASS uses the hard-gated phase-compensated physical-scan "
                        "contract; exact common-stamp coverage is reference-only."
                    ),
                },
            }
        }
        write_json(output / "metrics.json", published)
        records.append(
            {
                "directory": output,
                "sources": [
                    (source_id, path) for source_id, path in sources.items()
                ],
                "provenance": {
                    "source_result_set": GNSS_RESULT_SET,
                    "publication_status": contract["publication_status"],
                    "canonical_runs": published["canonical_runs"],
                    "evaluation_method": (
                        "exact initial pose; same-run exact estimator stamps"
                    ),
                    "runtime_validation": evidence["runtime_validation"],
                },
            }
        )
    return records


def require_lsim_contract_adopted() -> None:
    pending: list[str] = []
    for course, contract in LSIM_CURRENT_RUN_CONTRACTS.items():
        for source_id, (_, expected_hash) in contract["sources"].items():
            if not isinstance(expected_hash, str) or re.fullmatch(
                r"[0-9a-f]{64}", expected_hash
            ) is None:
                pending.append(f"{course}/{source_id}/sha256")
    require(
        not pending,
        "current-default LSim evidence has not been reviewed and adopted: "
        + ", ".join(pending),
    )


def pinned_lsim_sources(course: str) -> dict[str, Path]:
    require_lsim_contract_adopted()
    contract = LSIM_CURRENT_RUN_CONTRACTS[course]
    run_root = ROOT / "docker_output" / contract["run_id"]
    sources: dict[str, Path] = {}
    for source_id, (relative, expected_hash) in contract["sources"].items():
        path = run_root / relative
        require(path.is_file(), f"missing pinned LSim source: {path}")
        actual_hash = sha256(path)
        require(
            actual_hash == expected_hash,
            f"pinned LSim source hash mismatch for {course}/{source_id}: "
            f"expected={expected_hash} actual={actual_hash}",
        )
        sources[source_id] = path
    return sources


def parse_shell_assignments(text: str, context: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(text.splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        match = re.fullmatch(r"([A-Za-z_][A-Za-z0-9_]*)=(.*)", line)
        require(match is not None, f"{context}:{line_number}: malformed assignment")
        key, encoded = match.groups()
        require(key not in values, f"{context}:{line_number}: duplicate key {key}")
        try:
            tokens = shlex.split(encoded, posix=True)
        except ValueError as error:
            raise RuntimeError(
                f"{context}:{line_number}: malformed value for {key}: {error}"
            ) from error
        require(
            len(tokens) == 1,
            f"{context}:{line_number}: {key} must decode to one value",
        )
        values[key] = tokens[0]
    require(values, f"{context}: no assignments")
    return values


def yaml_scalar(text: str, key: str) -> str:
    match = re.search(
        rf"^\s*{re.escape(key)}:\s*[\"']?([^\"'#\n]+?)[\"']?\s*$",
        text,
        re.MULTILINE,
    )
    require(match is not None, f"missing YAML scalar {key!r}")
    return match.group(1).strip()


def yaml_numeric_list(text: str, key: str) -> list[float]:
    block = re.search(
        rf"^\s*{re.escape(key)}:\s*$((?:\n[ \t]+-\s*[-+0-9.eE]+[ \t]*)+)",
        text,
        re.MULTILINE,
    )
    require(block is not None, f"missing YAML numeric list {key!r}")
    return [
        float(value)
        for value in re.findall(
            r"^\s+-\s*([-+0-9.eE]+)\s*$", block.group(1), re.MULTILINE
        )
    ]


def lsim_projection_semantics(
    parameter_text: str, metadata_text: str, override_text: str
) -> dict[str, Any]:
    parameters = {
        "projector_type": yaml_scalar(parameter_text, "projector_type"),
        "vertical_datum": yaml_scalar(parameter_text, "vertical_datum"),
        "map_origin": [
            float(yaml_scalar(parameter_text, "map_origin.latitude")),
            float(yaml_scalar(parameter_text, "map_origin.longitude")),
            float(yaml_scalar(parameter_text, "map_origin.altitude")),
        ],
        "scale_factor": float(yaml_scalar(parameter_text, "scale_factor")),
        "use_legacy_projection_params": (
            yaml_scalar(parameter_text, "use_legacy_projection_params").lower()
            == "true"
        ),
        "gnss0": yaml_numeric_list(parameter_text, "gnss0"),
    }
    metadata = {
        "projector_type": yaml_scalar(metadata_text, "projector_type"),
        "vertical_datum": yaml_scalar(metadata_text, "vertical_datum"),
        "map_origin": [
            float(yaml_scalar(metadata_text, "latitude")),
            float(yaml_scalar(metadata_text, "longitude")),
        ],
        "scale_factor": float(yaml_scalar(metadata_text, "scale_factor")),
    }
    override_empty = re.fullmatch(
        r"\s*/\*\*:\s*\n\s+ros__parameters:\s*\{\s*\}\s*",
        override_text,
    ) is not None
    expected_xy = [
        GNSS_DEFAULT_PROJECTION["latitude_deg"],
        GNSS_DEFAULT_PROJECTION["longitude_deg"],
    ]
    valid = (
        parameters["projector_type"]
        == GNSS_DEFAULT_PROJECTION["projector_type"]
        and parameters["vertical_datum"]
        == GNSS_DEFAULT_PROJECTION["vertical_datum"]
        and parameters["map_origin"]
        == [*expected_xy, GNSS_DEFAULT_PROJECTION["altitude_m"]]
        and parameters["gnss0"]
        == [*expected_xy, GNSS_DEFAULT_PROJECTION["altitude_m"]]
        and parameters["scale_factor"]
        == GNSS_DEFAULT_PROJECTION["scale_factor"]
        and parameters["use_legacy_projection_params"] is False
        and metadata["projector_type"] == parameters["projector_type"]
        and metadata["vertical_datum"] == parameters["vertical_datum"]
        and metadata["map_origin"] == parameters["map_origin"][:2]
        and metadata["scale_factor"] == parameters["scale_factor"]
        and override_empty
    )
    require(valid, "LSim run did not use the current default NMEA projection")
    return {
        **GNSS_DEFAULT_PROJECTION,
        "runtime_parameter_source": (
            "pure_nmea_gnss_conversion/param/param.yaml"
        ),
        "metadata_source": (
            "pure_nmea_gnss_conversion/config/map_projector_info.yaml"
        ),
        "parameters_match_metadata": True,
        "evaluation_override_applied": False,
        "provenance_verified": True,
    }


def validate_lsim_configuration_sources(
    sources: dict[str, Path], environment: dict[str, str], context: str
) -> dict[str, Any]:
    expected_environment = {
        "PLAYBACK_RATE": "1.0",
        "TF_POLICY": "isolate-all",
        "TRACKING_MODE": "scan_to_scan",
        "USE_GNSS": "true",
        "USE_IMU_DESKEW": "true",
        "RVIZ": "false",
        "RECORD_OUTPUT": "true",
    }
    mismatches = {
        key: {"expected": value, "actual": environment.get(key)}
        for key, value in expected_environment.items()
        if environment.get(key) != value
    }
    require(not mismatches, f"{context}: run.env mismatch: {mismatches}")
    expected_suffixes = {
        "NMEA_GNSS_PARAM": (
            "/pure_nmea_gnss_conversion/param/param.yaml"
        ),
        "NMEA_PROJECTOR_METADATA": (
            "/pure_nmea_gnss_conversion/config/map_projector_info.yaml"
        ),
        "NMEA_GNSS_OVERRIDE_PARAM": (
            "/pure_odometry_bringup/config/autoware_lsim/empty_params.yaml"
        ),
    }
    for key, suffix in expected_suffixes.items():
        require(
            environment.get(key, "").endswith(suffix),
            f"{context}: unexpected {key}: {environment.get(key)!r}",
        )

    repository_sources = {
        "nmea_parameters": (
            ROOT / "src/pure_nmea_gnss_conversion/param/param.yaml"
        ),
        "map_projector_metadata": (
            ROOT
            / "src/pure_nmea_gnss_conversion/config/map_projector_info.yaml"
        ),
        "nmea_override": (
            ROOT
            / "src/pure_odometry_bringup/config/autoware_lsim/empty_params.yaml"
        ),
    }
    for source_id, repository_path in repository_sources.items():
        require(
            sha256(sources[source_id]) == sha256(repository_path),
            f"{context}: copied {source_id} differs from the repository source",
        )

    projection = lsim_projection_semantics(
        sources["nmea_parameters"].read_text(encoding="utf-8"),
        sources["map_projector_metadata"].read_text(encoding="utf-8"),
        sources["nmea_override"].read_text(encoding="utf-8"),
    )

    with sources["effective_configurations"].open(
        newline="", encoding="utf-8"
    ) as stream:
        rows = list(csv.DictReader(stream, delimiter="\t"))
    require(
        rows and set(rows[0]) == {"role", "source_path", "sha256", "artifact"},
        f"{context}: malformed effective-configuration table",
    )
    by_role = {row["role"]: row for row in rows}
    require(len(by_role) == len(rows), f"{context}: duplicate configuration roles")
    for role, source_id, artifact in (
        ("nmea_base", "nmea_parameters", "nmea_gnss_param.yaml"),
        (
            "nmea_projector_metadata",
            "map_projector_metadata",
            "map_projector_info.yaml",
        ),
        ("nmea_override", "nmea_override", "nmea_override_param.yaml"),
    ):
        row = by_role.get(role, {})
        require(
            row.get("artifact") == artifact
            and row.get("sha256") == sha256(sources[source_id]),
            f"{context}: invalid effective configuration role {role}",
        )
    return projection


def parse_lsim_validation_report(
    text: str, run_id: str, context: str
) -> dict[str, Any]:
    require(
        "Autoware lSIM output acceptance" in text,
        f"{context}: wrong validation report type",
    )
    bag_match = re.search(r"^Bag:\s+(.+?)\s*$", text, re.MULTILINE)
    profile_match = re.search(r"^Profile:\s+(.+?)\s*$", text, re.MULTILINE)
    result_match = re.search(r"^Result:\s+(.+?)\s*$", text, re.MULTILINE)
    summary_match = re.search(
        r"^Summary:\s+(\d+) failure\(s\),\s+(\d+) warning\(s\),\s+"
        r"(\d+) passed check\(s\)\.\s*$",
        text,
        re.MULTILINE,
    )
    rate_match = re.search(
        r"\[PASS\] Autoware state effective rate:\s+"
        r"([-+0-9.eE]+) Hz over ([-+0-9.eE]+) s",
        text,
    )
    step_match = re.search(
        r"\[(PASS|WARN)\] consecutive XY step:\s+"
        r"maximum consecutive XY step="
        r"([-+0-9.eE]+) m",
        text,
    )
    read_match = re.search(
        r"^Read:\s+(\d+) messages on (\d+) topics\s*$", text, re.MULTILINE
    )
    state_match = re.search(
        r"\[PASS\] kinematic state contract:\s+(\d+) map->base_link states;",
        text,
    )
    adapter_match = re.search(
        r"\[PASS\] adapter final diagnostic:\s+rejected_count=(\d+),\s+"
        r"last_rejection_reason=([^\s]+)\s+\((\d+) samples\)",
        text,
    )
    fusion_match = re.search(
        r"\[PASS\] GNSS fusion final diagnostic:\s+([^/\s]+)/([^\s]+)\s+"
        r"with position_fused=(true|false) and yaw_fused=(true|false)",
        text,
    )
    outage_match = re.search(
        r"\[PASS\] GNSS outage recovery sequence:\s+(.+?)\s*$",
        text,
        re.MULTILINE,
    )
    deskew_match = re.search(
        r"\[PASS\] deskew diagnostic:\s+(\d+)/(\d+) successful "
        r"\(([-+0-9.eE]+)%\);",
        text,
    )
    tracking_match = re.search(
        r"\[PASS\] LiDAR tracking diagnostic:\s+scan_to_scan;\s+"
        r"no recorded tracking-reset evidence in (\d+) samples",
        text,
    )
    registration_match = re.search(
        r"\[PASS\] LiDAR registration acceptance:\s+(\d+)/(\d+) "
        r"\(([-+0-9.eE]+)%\)",
        text,
    )
    require(
        all(
            match is not None
            for match in (
                bag_match,
                profile_match,
                result_match,
                summary_match,
                rate_match,
                step_match,
                read_match,
                state_match,
                adapter_match,
                fusion_match,
                outage_match,
                deskew_match,
                tracking_match,
                registration_match,
            )
        ),
        f"{context}: incomplete validation report",
    )
    failures, warnings, passed = map(int, summary_match.groups())
    actual_passed = len(re.findall(r"^\s*\[PASS\]", text, re.MULTILINE))
    actual_warnings = len(re.findall(r"^\s*\[WARN\]", text, re.MULTILINE))
    actual_failures = len(re.findall(r"^\s*\[FAIL\]", text, re.MULTILINE))
    require(
        failures == actual_failures == 0
        and warnings == actual_warnings
        and passed == actual_passed
        and result_match.group(1).startswith("PASS"),
        f"{context}: validation summary is inconsistent or failed",
    )
    require(
        bag_match.group(1).endswith(f"/{run_id}/localization_output"),
        f"{context}: report uses an unexpected output bag",
    )
    require(
        profile_match.group(1) == "hesai-rosbag23",
        f"{context}: unexpected validator profile",
    )
    require(
        "Accuracy: NOT EVALUATED - this output bag has no independent "
        "ground-truth trajectory." in text,
        f"{context}: accuracy scope is not explicit",
    )
    required_checks = (
        "simulation clock contract",
        "kinematic state contract",
        "adapter count/stamp alignment",
        "TF/state value alignment",
        "Hesai calibrated static TF",
        "GNSS outage recovery sequence",
        "LiDAR registration acceptance",
    )
    for name in required_checks:
        require(
            re.search(rf"^\s*\[PASS\] {re.escape(name)}:", text, re.MULTILINE)
            is not None,
            f"{context}: required acceptance check did not pass: {name}",
        )
    warning_details = re.findall(
        r"^\s*\[WARN\]\s+(.+?)\s*$", text, re.MULTILINE
    )
    public_warning_details = [
        re.sub(
            r"\s+at\s+[-+0-9.eE]+(?=\s+\()",
            "",
            detail,
        )
        for detail in warning_details
    ]
    return {
        "status": (
            "PASS"
            if warnings == 0
            else f"PASS with {warnings} warning" + ("s" if warnings != 1 else "")
        ),
        "passed_checks": passed,
        "warnings": warnings,
        "failures": failures,
        "warning_details": public_warning_details,
        "messages_read": int(read_match.group(1)),
        "topic_count": int(read_match.group(2)),
        "kinematic_state_samples": int(state_match.group(1)),
        "effective_state_rate_hz": float(rate_match.group(1)),
        "evaluated_duration_sec": float(rate_match.group(2)),
        "maximum_consecutive_xy_step_m": float(step_match.group(2)),
        "consecutive_xy_step_check_status": step_match.group(1),
        "adapter_final": {
            "rejected_count": int(adapter_match.group(1)),
            "last_rejection_reason": adapter_match.group(2),
            "diagnostic_samples": int(adapter_match.group(3)),
        },
        "gnss_fusion_final": {
            "state": fusion_match.group(1),
            "mode": fusion_match.group(2),
            "position_fused": fusion_match.group(3) == "true",
            "yaw_fused": fusion_match.group(4) == "true",
        },
        "gnss_outage_recovery_sequence": outage_match.group(1),
        "deskew": {
            "successful": int(deskew_match.group(1)),
            "total": int(deskew_match.group(2)),
            "success_percent": float(deskew_match.group(3)),
        },
        "lidar_tracking": {
            "mode": "scan_to_scan",
            "samples": int(tracking_match.group(1)),
            "recorded_reset_evidence": False,
        },
        "lidar_registration": {
            "accepted": int(registration_match.group(1)),
            "attempted": int(registration_match.group(2)),
            "acceptance_percent": float(registration_match.group(3)),
        },
        "accuracy_evaluated": False,
        "accuracy_reason": "No independent ground-truth trajectory is present.",
    }


def validate_current_lsim_run(
    course: str, sources: dict[str, Path]
) -> dict[str, Any]:
    contract = LSIM_CURRENT_RUN_CONTRACTS[course]
    context = f"LSim/{course}"
    environment = parse_shell_assignments(
        sources["run_environment"].read_text(encoding="utf-8"), context
    )
    projection = validate_lsim_configuration_sources(
        sources, environment, context
    )
    acceptance = parse_lsim_validation_report(
        sources["validation_report"].read_text(encoding="utf-8"),
        contract["run_id"],
        context,
    )
    require(
        sources["output_metadata"].stat().st_size > 0,
        f"{context}: output metadata is empty",
    )
    return {
        "dataset_label": contract["dataset_label"],
        "source_run_id": contract["run_id"],
        "configuration": {
            "current_default_projection_equivalent": True,
            "projection": projection,
            "rviz_enabled": False,
            "playback_rate": float(environment["PLAYBACK_RATE"]),
        },
        "acceptance": acceptance,
        "source_hashes_pinned": True,
    }


def curate_lsim() -> list[dict[str, Any]]:
    require_lsim_contract_adopted()
    course = "course_2"
    current_sources = pinned_lsim_sources(course)
    current_run = validate_current_lsim_run(course, current_sources)
    output = ASSETS / "autoware_lsim_hesai_course_2"
    output.mkdir(parents=True, exist_ok=True)
    poster = output / LSIM_POSTER_FILENAME
    require(poster.is_file(), f"missing reviewed RViz poster: {poster}")
    require(
        sha256(poster) == LSIM_POSTER_SHA256,
        "the reviewed RViz poster changed without updating its contract",
    )
    require(
        not set(png_chunk_types(poster))
        & {"tEXt", "zTXt", "iTXt", "eXIf", "tIME"},
        "the reviewed RViz poster contains publication metadata",
    )
    replay = output / LSIM_REPLAY_FILENAME
    require(replay.is_file(), f"missing reviewed RViz replay: {replay}")
    require(
        replay.stat().st_size == LSIM_REPLAY_BYTES,
        "the reviewed RViz replay size changed without updating its contract",
    )
    require(
        sha256(replay) == LSIM_REPLAY_SHA256,
        "the reviewed RViz replay changed without updating its contract",
    )
    require(
        replay.read_bytes()[:4] == b"\x1aE\xdf\xa3",
        "the reviewed RViz replay does not have a WebM/EBML signature",
    )
    write_json(
        output / "metrics.json",
        {
            "schema_version": 2,
            "dataset_label": (
                "Autoware LSim — Hesai 32-Line + IMU + RTK GNSS — Course 2"
            ),
            "current_default_projection_headless": {
                "status": "PASS",
                "current_default_projection_equivalent": True,
                "evidence_scope": LSIM_CURRENT_EVIDENCE_SCOPE,
                "rviz_evaluated": False,
                "accuracy_evaluated": False,
                "accuracy_reason": (
                    "The LSim output contains no independent ground-truth "
                    "trajectory."
                ),
                "runs": {course: current_run},
            },
            "representative_rviz_poster": {
                "status": "PUBLISHED",
                "visualization_only": True,
                "passing_run_evidence": False,
                "accuracy_evaluated": False,
                "sample_vehicle_model": (
                    "illustrative Autoware Lexus mesh; not the recorded vehicle, "
                    "its geometry, or its sensor calibration"
                ),
                "absolute_timestamps_published": False,
                "embedded_metadata_removed": True,
            },
            "representative_rviz_replay": {
                "status": "PUBLISHED",
                "visualization_only": True,
                "passing_run_evidence": False,
                "accuracy_evaluated": False,
                "container": "WebM",
                "video_codec": "VP8",
                "audio_present": False,
                "declared_frame_rate_hz": None,
                "variable_or_unspecified_frame_rate": True,
                "frame_count": LSIM_REPLAY_FRAME_COUNT,
                "duration_sec": LSIM_REPLAY_DURATION_SEC,
                "resolution_px": LSIM_REPLAY_RESOLUTION,
                "bytes": LSIM_REPLAY_BYTES,
                "sha256": LSIM_REPLAY_SHA256,
                "capture_timestamp_published": False,
                "embedded_capture_datetime_removed": True,
            },
        },
    )
    sources: list[tuple[str, Path]] = [
        (f"current_{course}_{source_id}", path)
        for source_id, path in current_sources.items()
    ]
    sources.append(("representative_rviz_poster", poster))
    sources.append(("representative_rviz_replay", replay))
    return [
        {
            "directory": output,
            "sources": sources,
            "provenance": {
                "current_default_projection_headless": {
                    "source_run_id": LSIM_CURRENT_RUN_CONTRACTS[course]["run_id"],
                    "current_default_projection_equivalent": True,
                    "evidence_scope": LSIM_CURRENT_EVIDENCE_SCOPE,
                },
                "representative_rviz_poster": {
                    "visualization_only": True,
                    "passing_run_evidence": False,
                    "accuracy_evaluated": False,
                    "embedded_metadata_removed": True,
                },
                "representative_rviz_replay": {
                    "visualization_only": True,
                    "passing_run_evidence": False,
                    "accuracy_evaluated": False,
                    "container": "WebM",
                    "video_codec": "VP8",
                    "audio_present": False,
                    "declared_frame_rate_hz": None,
                    "variable_or_unspecified_frame_rate": True,
                    "frame_count": LSIM_REPLAY_FRAME_COUNT,
                    "duration_sec": LSIM_REPLAY_DURATION_SEC,
                    "resolution_px": LSIM_REPLAY_RESOLUTION,
                    "capture_timestamp_published": False,
                    "embedded_capture_datetime_removed": True,
                },
            },
        }
    ]


def manifest_entry(record: dict[str, Any]) -> dict[str, Any]:
    directory: Path = record["directory"]
    files = []
    for path in sorted(directory.iterdir()):
        if path.is_file():
            files.append(
                {
                    "path": str(path.relative_to(ASSETS)),
                    "bytes": path.stat().st_size,
                    "sha256": sha256(path),
                }
            )
    sources = []
    for source in record["sources"]:
        if isinstance(source, tuple):
            local_source_id, path = source
        else:
            local_source_id, path = source.name, source
        sources.append(
            {"local_source_id": local_source_id, "sha256": sha256(path)}
        )
    return {
        "dataset": directory.name,
        "files": files,
        "provenance": record["provenance"],
        "source_artifacts": sources,
    }


def build_manifest(
    records: list[dict[str, Any]], *, preserve_unmentioned: bool = False,
    retired_datasets: set[str] | None = None,
) -> None:
    retired_datasets = retired_datasets or set()
    new_entries = {
        record["directory"].name: manifest_entry(record) for record in records
    }
    require(
        len(new_entries) == len(records),
        "duplicate dataset records while building publication manifest",
    )
    entries: list[dict[str, Any]] = []
    if preserve_unmentioned:
        existing_path = ASSETS / "manifest.json"
        require(
            existing_path.is_file(),
            "cannot preserve unmentioned publication datasets without a manifest",
        )
        existing = read_json(existing_path)
        require(
            existing.get("schema_version") == 1,
            "cannot preserve datasets from an unsupported manifest schema",
        )
        seen: set[str] = set()
        for entry in existing.get("datasets", []):
            dataset = entry.get("dataset")
            require(
                isinstance(dataset, str) and dataset not in seen,
                f"invalid or duplicate existing manifest dataset: {dataset!r}",
            )
            seen.add(dataset)
            if dataset in retired_datasets:
                continue
            entries.append(new_entries.pop(dataset, entry))
    entries.extend(new_entries.values())
    entries.sort(key=lambda item: item["dataset"])
    write_json(
        ASSETS / "manifest.json",
        {
            "schema_version": 1,
            "generated_by": "tools/evaluation/curate_publication_assets.py",
            "content_policy": (
                "Curated plots, normalized JSON, and the single pinned RViz "
                "WebM only; no bags, raw logs, or sample CSV files."
            ),
            "datasets": entries,
        },
    )


def check_mid360_publication_contract(manifest: dict[str, Any]) -> None:
    datasets = {
        item.get("dataset"): item for item in manifest.get("datasets", [])
    }
    require(
        MID360_RETIRED_DATASET_ID not in datasets
        and not (ASSETS / MID360_RETIRED_DATASET_ID).exists(),
        "retired external-IMU MID-360 evidence must not remain public",
    )
    require(MID360_DATASET_ID in datasets, "missing internal-IMU MID-360 dataset")
    dataset = datasets[MID360_DATASET_ID]
    expected_paths = {
        f"{MID360_DATASET_ID}/{filename}"
        for filename in MID360_PUBLICATION_FILENAMES
    }
    actual_paths = {item.get("path") for item in dataset.get("files", [])}
    require(
        actual_paths == expected_paths,
        "internal-IMU MID-360 dataset must contain exactly four reviewed files",
    )
    expected_source_hashes = {
        source_id: expected_hash
        for source_id, (_, expected_hash) in MID360_SOURCE_CONTRACT.items()
    }
    expected_source_hashes.update(
        {
            f"stage_summary_{stage}": expected_hash
            for stage, expected_hash in MID360_STAGE_SUMMARY_CONTRACT.items()
        }
    )
    actual_source_hashes = {
        item.get("local_source_id"): item.get("sha256")
        for item in dataset.get("source_artifacts", [])
    }
    require(
        actual_source_hashes == expected_source_hashes,
        "internal-IMU MID-360 manifest source contract mismatch",
    )
    provenance = dataset.get("provenance", {})
    require(
        provenance.get("publication_status") == "ACCEPTED"
        and provenance.get("accepted") is True
        and provenance.get("failed_hard_gates") == []
        and provenance.get("sensor_contract")
        == "MID-360 internal IMU; identity LiDAR/IMU frame; no GNSS"
        and provenance.get("submap_snapshot_policy")
        == "interval 5 accepted frames"
        and provenance.get("canonical_modes")
        == {
            "control": "scan_to_scan",
            "candidate": "rolling_scan_to_submap_interval_5_frames",
        }
        and provenance.get("evaluation_method")
        == (
            "shared exact initial pose; native accepted-scan estimates; "
            "GLIM-only reference interpolation"
        )
        and "source_result_set" not in provenance,
        "internal-IMU MID-360 manifest provenance changed",
    )

    metrics_path = ASSETS / MID360_DATASET_ID / "metrics.json"
    metrics = read_json(metrics_path)
    require(
        metrics.get("schema_version") == 2
        and metrics.get("dataset_label") == MID360_LABEL
        and metrics.get("publication_status") == "ACCEPTED"
        and metrics.get("accepted") is True,
        "internal-IMU MID-360 metrics status/schema/label changed",
    )
    require(
        metrics.get("reference")
        == {
            "method": "GLIM trajectory",
            "independent_ground_truth": False,
            "shares_lidar_and_imu_observations": True,
        }
        and metrics.get("alignment")
        == {
            "type": "single shared exact first-common-pose SE(2)",
            "position_and_yaw_share_transform": True,
            "scale_applied": False,
            "estimate_interpolation": False,
            "reference_interpolation": "GLIM only",
        },
        "internal-IMU MID-360 reference/alignment disclosure changed",
    )
    sensor = metrics.get("sensor_configuration", {})
    expected_transforms = {
        "base_link_to_livox_frame": {
            "translation_m": [0.0, 0.0, 0.0],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
    }
    require(
        sensor
        == {
            "lidar": "Livox MID-360",
            "imu": "MID-360 internal IMU",
            "points_topic": "/livox/lidar",
            "imu_topic": "/livox/imu",
            "imu_acceleration_scale": 9.80665,
            "gyro_z_bias_rad_s": 0.006959692,
            "gyro_bias_adaptation_enabled": False,
            "gnss_used": False,
            "recorded_tf_used": False,
            "static_transforms": expected_transforms,
            "playback_rate": 1.0,
            "submap_snapshot_interval_frames": 5,
        },
        "internal-IMU MID-360 public sensor/TF contract changed",
    )
    evaluation = metrics.get("evaluation", {})
    streams = evaluation.get("streams", {})
    require(
        evaluation.get("overall_ab_passed") is True
        and evaluation.get("hard_gate_count") == 20
        and evaluation.get("failed_hard_gates") == []
        and set(streams) == {"scan_to_scan", "scan_to_submap"}
        and streams["scan_to_scan"]["samples"] == 2917
        and streams["scan_to_submap"]["samples"] == 2916
        and streams["scan_to_scan"]["position_error_m"]["rmse"]
        == 0.6303449121594672
        and streams["scan_to_submap"]["position_error_m"]["rmse"]
        == 0.2940738154352217
        and streams["scan_to_scan"]["yaw_error_deg"]["rmse"]
        == 2.9063988030589183
        and streams["scan_to_submap"]["yaw_error_deg"]["rmse"]
        == 0.6023916185230034
        and streams["scan_to_submap"]["path_ratio_estimate_over_reference"]
        == 1.0057248435050143,
        "internal-IMU MID-360 published headline streams/metrics changed",
    )
    holdout = metrics.get("holdout_validation", {})
    require(
        holdout.get("selection_score_used_holdout") is False
        and holdout.get("opened_after_all_stages_completed") is True
        and holdout.get("scan_to_scan", {}).get("position_error_m", {}).get("rmse")
        == 0.5940821978291224
        and holdout.get("scan_to_submap", {}).get("position_error_m", {}).get("rmse")
        == 0.3066361711828477
        and holdout.get("scan_to_scan", {}).get("yaw_error_deg", {}).get("rmse")
        == 3.07764842104325
        and holdout.get("scan_to_submap", {}).get("yaw_error_deg", {}).get("rmse")
        == 0.6737415550073956,
        "internal-IMU MID-360 holdout contract changed",
    )
    micro_motion = metrics.get("micro_motion_path_audit", {})
    require(
        micro_motion.get("reference_step_threshold_m_exclusive") == 0.002
        and micro_motion.get("streams")
        == {
            "scan_to_scan": {
                "interval_count": 263,
                "reference_path_m": 0.16677875544589313,
                "estimate_path_m": 0.5608420287798032,
            },
            "scan_to_submap": {
                "interval_count": 263,
                "reference_path_m": 0.16677875544589313,
                "estimate_path_m": 0.6077103925815106,
            },
        },
        "internal-IMU MID-360 micro-motion evidence changed",
    )
    source_validation = metrics.get("source_validation", {})
    require(
        source_validation
        == {
            "source_hashes_pinned": True,
            "configuration_contract": "PASS",
            "packaged_profile_contract": "PASS",
            "control_runtime": "PASS",
            "precision_runtime": "PASS",
            "precision_structure": "PASS",
            "identity_transform_contract": "PASS",
            "tuning_candidate_count": 48,
            "tuning_stage_count": 9,
            "holdout_selection_isolation": "PASS",
            "status_revalidated_without_replay": True,
            "a_b_hard_gate_status": "PASS",
        },
        "internal-IMU MID-360 public source validation changed",
    )
    normalized = json.dumps(metrics, sort_keys=True).lower()
    private_markers = (
        "/home/",
        "rosbag2_2026_04_08-22_52_42",
        "mid360_dense_20260812_142237",
        "rosbag2_2026_05_07-19_19_36_first250s",
        "mid360_dense_first250s_20260813_113422",
        "mid360_external_imu_20260813_final",
        "traj_lidar.txt",
        "stamp_ns",
        "precision_raw",
        MID360_RESULT_SET.lower(),
    )
    require(
        not any(marker in normalized for marker in private_markers),
        "private/retired MID-360 identifiers leaked into public metrics",
    )


def check_gnss_publication_contract(manifest: dict[str, Any]) -> None:
    require_gnss_contract_adopted()
    datasets = {
        item.get("dataset"): item for item in manifest.get("datasets", [])
    }
    expected_dataset_ids = {
        f"hesai_32line_imu_rtk_gnss_{course}"
        for course in GNSS_SOURCE_CONTRACTS
    }
    actual_dataset_ids = {
        dataset_id
        for dataset_id in datasets
        if isinstance(dataset_id, str)
        and dataset_id.startswith("hesai_32line_imu_rtk_gnss_")
    }
    require(
        actual_dataset_ids == expected_dataset_ids,
        "unexpected public Hesai GNSS dataset set",
    )
    for course, contract in GNSS_SOURCE_CONTRACTS.items():
        dataset_id = f"hesai_32line_imu_rtk_gnss_{course}"
        require(dataset_id in datasets, f"missing GNSS publication dataset: {dataset_id}")
        dataset = datasets[dataset_id]
        expected_publication_paths = {
            f"{dataset_id}/{filename}"
            for filename in GNSS_PUBLICATION_FILENAMES
        }
        actual_publication_paths = {
            item.get("path") for item in dataset.get("files", [])
        }
        require(
            actual_publication_paths == expected_publication_paths,
            f"{dataset_id}: only XY/yaw error figures and metrics may be published",
        )
        expected_hashes = {
            source_id: expected_hash
            for source_id, (_, expected_hash) in contract["sources"].items()
        }
        actual_hashes = {
            item.get("local_source_id"): item.get("sha256")
            for item in dataset.get("source_artifacts", [])
        }
        require(
            actual_hashes == expected_hashes,
            f"{dataset_id}: source-artifact contract mismatch",
        )
        provenance = dataset.get("provenance", {})
        require(
            provenance.get("source_result_set")
            == GNSS_RESULT_SET,
            f"{dataset_id}: wrong source result set",
        )
        require(
            provenance.get("publication_status") == contract["publication_status"],
            f"{dataset_id}: manifest publication status mismatch",
        )
        expected_runs = {
            "baseline": contract["baseline_run_id"],
            "control": contract["control_run_id"],
            "precision": contract["precision_run_id"],
        }
        require(
            provenance.get("canonical_runs") == expected_runs,
            f"{dataset_id}: manifest canonical-run mismatch",
        )

        metrics_path = ASSETS / dataset_id / "metrics.json"
        metrics = read_json(metrics_path)
        require(
            metrics.get("schema_version") == 2
            and metrics.get("publication_status") == contract["publication_status"],
            f"{dataset_id}: metrics publication status mismatch",
        )
        require(
            metrics.get("publication_acceptance", {}).get("accepted")
            is contract["accepted"],
            f"{dataset_id}: metrics acceptance decision mismatch",
        )
        require(
            metrics.get("canonical_runs") == expected_runs,
            f"{dataset_id}: metrics canonical-run mismatch",
        )
        expected_timeline = contract["expected_error_plot_timeline"]
        annotations = metrics.get("error_plot_annotations", {})
        time_axis = annotations.get("time_axis", {})
        unavailable = annotations.get("rtk_q4_unavailable", {})
        returned = annotations.get("rtk_q4_return", {})
        resumed = annotations.get("fusion_tracking_resume", {})
        expected_begin_sec = expected_timeline[
            "rtk_q4_unavailable_begin_time_sec"
        ]
        expected_end_sec = expected_timeline[
            "rtk_q4_unavailable_end_time_sec"
        ]
        expected_resume_sec = expected_timeline[
            "fusion_tracking_resume_time_sec"
        ]
        require(
            time_axis.get("csv_plot_time_column")
            == "time_from_common_start_sec"
            and time_axis.get("origin")
            == "first common global evaluation sample"
            and time_axis.get("absolute_timestamps_published") is False
            and isinstance(
                time_axis.get("maximum_mapping_residual_sec"), (int, float)
            )
            and 0.0 <= time_axis["maximum_mapping_residual_sec"] <= 1.0e-6
            and unavailable.get("begin_time_from_common_start_sec")
            == expected_begin_sec
            and unavailable.get("end_time_from_common_start_sec")
            == expected_end_sec
            and unavailable.get("duration_sec")
            == expected_timeline["rtk_q4_unavailable_duration_sec"]
            and returned.get("time_from_common_start_sec")
            == expected_end_sec
            and resumed.get("time_from_common_start_sec")
            == expected_resume_sec
            and resumed.get("delay_from_rtk_q4_return_sec")
            == expected_timeline["fusion_tracking_resume_delay_sec"],
            f"{dataset_id}: GNSS error-plot annotation contract mismatch",
        )
        require(
            [
                item.get("name")
                for item in metrics.get("publication_acceptance", {}).get(
                    "failed_hard_gates", []
                )
            ]
            == contract["expected_failed_hard_gates"],
            f"{dataset_id}: failed hard-gate publication mismatch",
        )
        require(
            metrics.get("alignment", {}).get("exact_initial_pose") is True
            and metrics.get("alignment", {}).get(
                "position_and_yaw_share_transform"
            )
            is True
            and metrics.get("alignment", {}).get("scale_applied") is False,
            f"{dataset_id}: published alignment contract mismatch",
        )
        require(
            metrics.get("method", {}).get("estimator_association")
            == "exact same-run integer header-stamp intersection; no estimator interpolation",
            f"{dataset_id}: published association contract mismatch",
        )
        projection = metrics.get("nmea_projection", {})
        require(
            projection.get("projector_type")
            == GNSS_DEFAULT_PROJECTION["projector_type"]
            and projection.get("vertical_datum")
            == GNSS_DEFAULT_PROJECTION["vertical_datum"]
            and projection.get("latitude_deg")
            == GNSS_DEFAULT_PROJECTION["latitude_deg"]
            and projection.get("longitude_deg")
            == GNSS_DEFAULT_PROJECTION["longitude_deg"]
            and projection.get("altitude_m")
            == GNSS_DEFAULT_PROJECTION["altitude_m"]
            and projection.get("scale_factor")
            == GNSS_DEFAULT_PROJECTION["scale_factor"]
            and projection.get("parameters_match_metadata") is True
            and projection.get("evaluation_override_applied") is False
            and projection.get("provenance_verified") is True,
            f"{dataset_id}: default NMEA projection is not published",
        )
        validation = metrics.get("source_validation", {})
        expected_accuracy_status = "PASS" if contract["accepted"] else "FAIL"
        startup_window = validation.get("startup_window_provenance", {})
        runtime_validation = validation.get("runtime_validation", {})
        require(
            validation.get("accuracy_result") == expected_accuracy_status
            and validation.get("startup_acceptance") == "PASS"
            and validation.get("accepted_scan_nonintrusion") == "PASS"
            and validation.get("source_hashes_pinned") is True
            and set(validation.get("provenance", {}))
            == {"baseline", "control", "precision"}
            and all(
                item.get("status") == "PASS"
                for item in validation.get("provenance", {}).values()
            ),
            f"{dataset_id}: supporting validation contract mismatch",
        )
        expected_runtime = contract["expected_runtime"]
        runtime_matcher = runtime_validation.get("matcher", {})
        runtime_publication = runtime_validation.get(
            "map_fusion_publication", {}
        )
        runtime_guard = runtime_validation.get("outage_yaw_guard", {})
        require(
            runtime_validation.get("status") == "PASS"
            and runtime_validation.get("successful_check_count") == 28
            and runtime_validation.get("pass_line_count") == 27
            and runtime_validation.get("failed_check_count") == 0
            and runtime_validation.get("intentional_warning_count") == 1
            and runtime_validation.get("intentional_warning")
            == GNSS_RUNTIME_INTENTIONAL_WARNING_LABEL
            and runtime_validation.get("playback_rate")
            == {"observed": 1.0, "expected": 1.0}
            and runtime_matcher.get("scans") == expected_runtime["scans"]
            and runtime_matcher.get("corrections")
            == expected_runtime["corrections"]
            and runtime_matcher.get("received")
            == runtime_matcher.get("processed")
            == expected_runtime["scans"]
            and runtime_matcher.get("committed")
            == runtime_matcher.get("published")
            == expected_runtime["corrections"]
            and runtime_matcher.get("queue_drop_count") == 0
            and runtime_matcher.get("malformed_count") == 0
            and runtime_matcher.get("stale_count") == 0
            and runtime_matcher.get("processing_p99_ms")
            == expected_runtime["processing_p99_ms"]
            and runtime_matcher.get("end_to_end_latency_p99_ms")
            == expected_runtime["end_to_end_latency_p99_ms"]
            and runtime_publication.get("status") == "PASS"
            and runtime_publication.get("strict_drop_count") == 0
            and runtime_publication.get("eligible_unique_raw_stamp_count")
            == runtime_publication.get("matched_unique_raw_stamp_count")
            == expected_runtime["eligible_unique_raw_stamp_count"]
            and runtime_publication.get("missing_unique_raw_stamp_count") == 0
            and runtime_publication.get("exact_causal_coverage_ratio") == 1.0
            and runtime_publication.get("covered_odometry_coalesced_count")
            == expected_runtime["covered_odometry_coalesced_count"]
            and runtime_publication.get("total_suppressed_request_count")
            == runtime_publication.get("strict_drop_count")
            + runtime_publication.get("covered_odometry_coalesced_count")
            + runtime_publication.get("wall_timer_coalesced_count")
            and runtime_guard.get("status") == "PASS"
            and runtime_guard.get("maximum_active_reference_variance_rad2")
            == expected_runtime["maximum_active_reference_variance_rad2"]
            and runtime_guard.get("active_sample_count")
            == runtime_guard.get("outage_sample_count")
            + runtime_guard.get("recovery_release_sample_count")
            and provenance.get("runtime_validation") == runtime_validation,
            f"{dataset_id}: normalized precision runtime contract mismatch",
        )
        require(
            startup_window.get("status") == "PASS"
            and startup_window.get("timestamp_source")
            == "physical ROS header stamps"
            and startup_window.get("window_bounds") == "inclusive"
            and startup_window.get("absolute_timestamps_published") is False
            and startup_window.get("duration_sec") == 20.0
            and startup_window.get("common_sample_count") == 400
            and startup_window.get("maximum_interpolation_gap_sec") == 0.1
            and "start_sec" not in startup_window
            and "end_sec" not in startup_window
            and metrics.get("supporting_acceptance", {})
            .get("startup", {})
            .get("calibration_window_provenance") == startup_window,
            f"{dataset_id}: startup calibration-window provenance mismatch",
        )
        guard = metrics.get("outage_yaw_guard", {})
        precision_provenance = validation.get("provenance", {}).get(
            "precision", {}
        )
        guard_covariance = guard.get("covariance_contract", {})
        provenance_guard_covariance = (
            precision_provenance.get("outage_yaw_guard", {})
            .get("covariance_contract", {})
        )
        require(
            guard.get("status") == "PASS"
            and {
                key: guard.get(key)
                for key in GNSS_OUTAGE_YAW_GUARD_SEMANTICS
            }
            == GNSS_OUTAGE_YAW_GUARD_SEMANTICS
            and guard.get("config") == GNSS_OUTAGE_YAW_GUARD_CONFIG
            and int(guard.get("accepted_reference_count", 0)) > 0
            and guard.get("invalid_advance_count") == 0
            and guard.get("suppressed_invalid_count") == 0
            and precision_provenance.get("outage_yaw_guard", {}).get("status")
            == "PASS"
            and precision_provenance.get("outage_yaw_guard", {}).get(
                "accepted_reference_count", 0
            ) > 0
            and precision_provenance.get("outage_yaw_guard", {}).get(
                "invalid_advance_count"
            ) == 0
            and precision_provenance.get("outage_yaw_guard", {}).get(
                "suppressed_invalid_count"
            ) == 0
            and guard_covariance.get("status") == "PASS"
            and int(guard_covariance.get("variance_sample_count", 0)) > 0
            and type(
                guard_covariance.get("maximum_active_reference_variance_rad2")
            ) in (int, float)
            and guard_covariance["maximum_active_reference_variance_rad2"] > 0.0
            and guard_covariance["maximum_active_reference_variance_rad2"]
            <= GNSS_OUTAGE_YAW_GUARD_CONFIG["max_trusted_variance_rad2"]
            and guard_covariance.get("maximum_additional_variance_rad2", -1.0)
            >= guard_covariance["maximum_active_reference_variance_rad2"]
            and provenance_guard_covariance.get("status") == "PASS"
            and provenance_guard_covariance.get("issue_sample_count") == 0
            and provenance_guard_covariance.get(
                "maximum_active_reference_variance_rad2"
            ) == guard_covariance["maximum_active_reference_variance_rad2"],
            f"{dataset_id}: outage-yaw guard publication contract mismatch",
        )
        for mode, item in validation.get("provenance", {}).items():
            publication = item.get("map_fusion_publication_integrity", {})
            require(
                publication.get("status") == "PASS"
                and publication.get("strict_drop_count") == 0
                and publication.get("missing_raw_unique_stamp_count") == 0
                and int(publication.get("causal_raw_unique_stamp_count", 0)) > 0,
                f"{dataset_id}/{mode}: map-fusion publication contract mismatch",
            )
        banned = ("provisional", "historical", "site-origin override")
        metrics_text = metrics_path.read_text(encoding="utf-8").lower()
        require(
            not any(word in metrics_text for word in banned),
            f"{dataset_id}: obsolete GNSS qualification text remains",
        )


def check_lsim_publication_contract(manifest: dict[str, Any]) -> None:
    require_lsim_contract_adopted()
    course = "course_2"
    contract = LSIM_CURRENT_RUN_CONTRACTS[course]
    dataset_id = "autoware_lsim_hesai_course_2"
    public_lsim_dataset_ids = {
        item.get("dataset")
        for item in manifest.get("datasets", [])
        if isinstance(item.get("dataset"), str)
        and item["dataset"].startswith("autoware_lsim_hesai_")
    }
    require(
        public_lsim_dataset_ids == {dataset_id},
        "unexpected public Autoware LSim dataset set",
    )
    dataset = next(
        (
            item
            for item in manifest.get("datasets", [])
            if item.get("dataset") == dataset_id
        ),
        None,
    )
    require(dataset is not None, f"missing LSim publication dataset: {dataset_id}")
    actual_paths = {item.get("path") for item in dataset.get("files", [])}
    expected_paths = {
        f"{dataset_id}/{filename}" for filename in LSIM_PUBLICATION_FILENAMES
    }
    require(
        actual_paths == expected_paths,
        "LSim publication dataset must contain exactly metrics, poster, and replay",
    )
    provenance = dataset.get("provenance", {})
    current_provenance = provenance.get(
        "current_default_projection_headless", {}
    )
    require(
        current_provenance.get("source_run_id") == contract["run_id"]
        and current_provenance.get("current_default_projection_equivalent")
        is True
        and current_provenance.get("evidence_scope")
        == LSIM_CURRENT_EVIDENCE_SCOPE,
        "LSim manifest does not identify the current-default headless run",
    )
    poster_provenance = provenance.get("representative_rviz_poster", {})
    require(
        poster_provenance
        == {
            "visualization_only": True,
            "passing_run_evidence": False,
            "accuracy_evaluated": False,
            "embedded_metadata_removed": True,
        },
        "LSim poster provenance is missing or overstated",
    )
    replay_provenance = provenance.get("representative_rviz_replay", {})
    require(
        replay_provenance
        == {
            "visualization_only": True,
            "passing_run_evidence": False,
            "accuracy_evaluated": False,
            "container": "WebM",
            "video_codec": "VP8",
            "audio_present": False,
            "declared_frame_rate_hz": None,
            "variable_or_unspecified_frame_rate": True,
            "frame_count": LSIM_REPLAY_FRAME_COUNT,
            "duration_sec": LSIM_REPLAY_DURATION_SEC,
            "resolution_px": LSIM_REPLAY_RESOLUTION,
            "capture_timestamp_published": False,
            "embedded_capture_datetime_removed": True,
        },
        "LSim replay provenance is missing or overstated",
    )

    expected_source_hashes = {
        f"current_{course}_{source_id}": expected_hash
        for source_id, (_, expected_hash) in contract["sources"].items()
    }
    expected_source_hashes["representative_rviz_poster"] = LSIM_POSTER_SHA256
    expected_source_hashes["representative_rviz_replay"] = LSIM_REPLAY_SHA256
    actual_source_hashes = {
        item.get("local_source_id"): item.get("sha256")
        for item in dataset.get("source_artifacts", [])
    }
    require(
        actual_source_hashes == expected_source_hashes,
        "LSim manifest source-artifact contract mismatch",
    )

    metrics = read_json(ASSETS / dataset_id / "metrics.json")
    require(metrics.get("schema_version") == 2, "unexpected LSim metrics schema")
    current = metrics.get("current_default_projection_headless", {})
    poster = metrics.get("representative_rviz_poster", {})
    replay = metrics.get("representative_rviz_replay", {})
    require(
        current.get("status") == "PASS"
        and current.get("current_default_projection_equivalent") is True
        and current.get("evidence_scope") == LSIM_CURRENT_EVIDENCE_SCOPE
        and current.get("rviz_evaluated") is False
        and current.get("accuracy_evaluated") is False,
        "LSim metrics do not qualify the current-default headless scope",
    )
    require(
        poster.get("status") == "PUBLISHED"
        and poster.get("visualization_only") is True
        and poster.get("passing_run_evidence") is False
        and poster.get("accuracy_evaluated") is False
        and poster.get("absolute_timestamps_published") is False
        and poster.get("embedded_metadata_removed") is True
        and "not the recorded vehicle" in poster.get("sample_vehicle_model", ""),
        "LSim poster scope or privacy contract is incomplete",
    )
    require(
        replay.get("status") == "PUBLISHED"
        and replay.get("visualization_only") is True
        and replay.get("passing_run_evidence") is False
        and replay.get("accuracy_evaluated") is False
        and replay.get("container") == "WebM"
        and replay.get("video_codec") == "VP8"
        and replay.get("audio_present") is False
        and replay.get("declared_frame_rate_hz") is None
        and replay.get("variable_or_unspecified_frame_rate") is True
        and replay.get("frame_count") == LSIM_REPLAY_FRAME_COUNT
        and replay.get("duration_sec") == LSIM_REPLAY_DURATION_SEC
        and replay.get("resolution_px") == LSIM_REPLAY_RESOLUTION
        and replay.get("bytes") == LSIM_REPLAY_BYTES
        and replay.get("sha256") == LSIM_REPLAY_SHA256
        and replay.get("capture_timestamp_published") is False
        and replay.get("embedded_capture_datetime_removed") is True,
        "LSim replay scope or media contract is incomplete",
    )
    runs = current.get("runs", {})
    require(
        set(runs) == {course},
        "LSim metrics contain an unexpected public run set",
    )
    expected_projection = {
        **GNSS_DEFAULT_PROJECTION,
        "runtime_parameter_source": (
            "pure_nmea_gnss_conversion/param/param.yaml"
        ),
        "metadata_source": (
            "pure_nmea_gnss_conversion/config/map_projector_info.yaml"
        ),
        "parameters_match_metadata": True,
        "evaluation_override_applied": False,
        "provenance_verified": True,
    }
    run = runs.get(course, {})
    acceptance = run.get("acceptance", {})
    configuration = run.get("configuration", {})
    adapter = acceptance.get("adapter_final", {})
    fusion = acceptance.get("gnss_fusion_final", {})
    deskew = acceptance.get("deskew", {})
    tracking = acceptance.get("lidar_tracking", {})
    registration = acceptance.get("lidar_registration", {})
    require(
        run.get("dataset_label") == contract["dataset_label"]
        and run.get("source_run_id") == contract["run_id"]
        and run.get("source_hashes_pinned") is True
        and configuration.get("current_default_projection_equivalent") is True
        and configuration.get("projection") == expected_projection
        and configuration.get("rviz_enabled") is False
        and configuration.get("playback_rate") == 1.0
        and str(acceptance.get("status", "")).startswith("PASS")
        and acceptance.get("failures") == 0
        and acceptance.get("accuracy_evaluated") is False
        and int(acceptance.get("passed_checks", 0)) > 0,
        f"LSim metrics failed current-default contract for {course}",
    )
    require(
        int(acceptance.get("messages_read", 0)) > 0
        and acceptance.get("topic_count") == 17
        and int(acceptance.get("kinematic_state_samples", 0)) > 0
        and float(acceptance.get("effective_state_rate_hz", 0.0)) > 0.0
        and float(acceptance.get("evaluated_duration_sec", 0.0)) > 0.0
        and float(acceptance.get("maximum_consecutive_xy_step_m", -1.0)) >= 0.0
        and acceptance.get("consecutive_xy_step_check_status") in {"PASS", "WARN"}
        and adapter.get("rejected_count") == 0
        and adapter.get("last_rejection_reason") == "none"
        and fusion
        == {
            "state": "tracking",
            "mode": "full_se2",
            "position_fused": True,
            "yaw_fused": True,
        }
        and all(
            state in acceptance.get("gnss_outage_recovery_sequence", "")
            for state in ("outage", "recovering", "tracking")
        )
        and int(deskew.get("successful", 0)) > 0
        and deskew.get("successful") <= deskew.get("total", 0)
        and tracking.get("mode") == "scan_to_scan"
        and tracking.get("recorded_reset_evidence") is False
        and int(registration.get("accepted", 0)) > 0
        and registration.get("accepted") <= registration.get("attempted", 0),
        f"LSim runtime evidence is incomplete for {course}",
    )


def check_published_assets() -> None:
    manifest_path = ASSETS / "manifest.json"
    manifest = read_json(manifest_path)
    if manifest.get("schema_version") != 1:
        raise RuntimeError("unsupported publication manifest schema")

    privacy_markers = ("course_1", "course-1", "course 1", "aggregate")
    manifest_text = manifest_path.read_text(encoding="utf-8").lower()
    require(
        not any(marker in manifest_text for marker in privacy_markers),
        "non-public or cross-course evidence leaked into the manifest",
    )

    check_gnss_publication_contract(manifest)
    check_mid360_publication_contract(manifest)
    check_lsim_publication_contract(manifest)

    listed: set[Path] = set()
    for dataset in manifest.get("datasets", []):
        source_ids = [
            item.get("local_source_id") for item in dataset.get("source_artifacts", [])
        ]
        if len(source_ids) != len(set(source_ids)):
            raise RuntimeError(
                f"duplicate source artifact IDs for {dataset.get('dataset')}"
            )
        for item in dataset.get("files", []):
            relative = Path(item["path"])
            if relative.is_absolute() or ".." in relative.parts:
                raise RuntimeError(f"unsafe publication path: {relative}")
            if relative in listed:
                raise RuntimeError(f"duplicate publication path: {relative}")
            listed.add(relative)
            path = ASSETS / relative
            if not path.is_file():
                raise RuntimeError(f"missing published asset: {relative}")
            if path.stat().st_size != item["bytes"]:
                raise RuntimeError(f"published asset size mismatch: {relative}")
            if sha256(path) != item["sha256"]:
                raise RuntimeError(f"published asset hash mismatch: {relative}")
            if path.suffix == ".json":
                read_json(path)
                normalized_text = path.read_text(encoding="utf-8").lower()
                require(
                    not any(marker in normalized_text for marker in privacy_markers),
                    f"non-public or cross-course evidence leaked into {relative}",
                )
            elif path.suffix == ".png":
                chunks = png_chunk_types(path)
                if relative == Path(
                    "autoware_lsim_hesai_course_2/rviz_poster.png"
                ):
                    require(
                        not set(chunks)
                        & {"tEXt", "zTXt", "iTXt", "eXIf", "tIME"},
                        "the public RViz poster contains embedded metadata",
                    )
            elif path.suffix == ".webm":
                require(
                    relative
                    == Path("autoware_lsim_hesai_course_2/rviz_replay.webm")
                    and path.stat().st_size == LSIM_REPLAY_BYTES
                    and sha256(path) == LSIM_REPLAY_SHA256
                    and path.read_bytes()[:4] == b"\x1aE\xdf\xa3",
                    f"unsupported or changed published WebM: {relative}",
                )
            else:
                raise RuntimeError(f"unsupported published asset type: {relative}")

    actual = {
        path.relative_to(ASSETS)
        for path in ASSETS.rglob("*")
        if path.is_file() and path.name not in {"README.md", "manifest.json"}
    }
    if listed != actual:
        missing = sorted(str(path) for path in listed - actual)
        unlisted = sorted(str(path) for path in actual - listed)
        raise RuntimeError(
            f"publication manifest file-set mismatch: missing={missing}, "
            f"unlisted={unlisted}"
        )

    for path in [manifest_path, *(ASSETS / item for item in sorted(actual))]:
        if path.suffix != ".json":
            continue
        text = path.read_text(encoding="utf-8")
        if "/home/" in text or "/Users/" in text:
            raise RuntimeError(f"host-specific path in published JSON: {path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--check",
        action="store_true",
        help="verify committed assets and hashes without requiring local result runs",
    )
    group.add_argument(
        "--check-regeneration",
        action="store_true",
        help="regenerate from ignored local runs and require byte-for-byte stability",
    )
    scope_group = parser.add_mutually_exclusive_group()
    scope_group.add_argument(
        "--lidar-only",
        action="store_true",
        help=(
            "regenerate only the public LiDAR/IMU-only datasets and preserve "
            "the existing GNSS and Autoware manifest entries"
        ),
    )
    scope_group.add_argument(
        "--gnss-only",
        action="store_true",
        help=(
            "regenerate only the public Hesai GNSS dataset and preserve the "
            "existing LiDAR-only and Autoware manifest entries"
        ),
    )
    scope_group.add_argument(
        "--lsim-only",
        action="store_true",
        help=(
            "regenerate only the Autoware LSim dataset and preserve the "
            "existing LiDAR-only and GNSS manifest entries"
        ),
    )
    args = parser.parse_args()
    if args.check:
        if args.lidar_only or args.gnss_only or args.lsim_only:
            parser.error("dataset scopes are only meaningful while regenerating")
        check_published_assets()
        return 0

    before = (
        {
            path.relative_to(ASSETS): sha256(path)
            for path in ASSETS.rglob("*")
            if path.is_file()
        }
        if args.check_regeneration
        else {}
    )
    if args.lidar_only:
        records = curate_lidar()
    elif args.gnss_only:
        records = curate_gnss()
    elif args.lsim_only:
        records = curate_lsim()
    else:
        records = curate_lidar() + curate_gnss() + curate_lsim()
    build_manifest(
        records,
        preserve_unmentioned=args.lidar_only or args.gnss_only or args.lsim_only,
        retired_datasets=(
            {MID360_RETIRED_DATASET_ID} if args.lidar_only else set()
        ),
    )
    check_published_assets()
    if args.check_regeneration:
        after = {
            path.relative_to(ASSETS): sha256(path)
            for path in ASSETS.rglob("*")
            if path.is_file()
        }
        changed = sorted(
            str(path)
            for path in set(before) | set(after)
            if before.get(path) != after.get(path)
        )
        if changed:
            raise SystemExit("non-deterministic or stale generated assets: " + ", ".join(changed))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
