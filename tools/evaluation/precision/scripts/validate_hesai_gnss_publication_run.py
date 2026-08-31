#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fail-closed provenance validation for publishable Hesai GNSS runs.

This validator does not infer a dataset from a bag filename.  The caller must
name the descriptive dataset and run mode explicitly.  It then ties the run
metadata, installed files, copied artifacts, private input-bag contract, output
bag, and runtime diagnostics back to the repository sources byte-for-byte.
"""

from __future__ import annotations

import argparse
import ast
import csv
from dataclasses import dataclass
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import shlex
import sys
import tempfile
from typing import Any, Iterable


EXPECTED_PROFILE = "hesai_32line_rtk"
EXPECTED_ORIGIN = (35.681236, 139.767125)
EXPECTED_PROJECTOR = "TransverseMercator"
EXPECTED_VERTICAL_DATUM = "WGS84"
EXPECTED_PROJECTION_MODE = "transverse_mercator_wgs84"
EXPECTED_SCALE_FACTOR = 0.9996
ORIGIN_DIAGNOSTIC_TOLERANCE_DEG = 5.1e-9
SCALE_DIAGNOSTIC_TOLERANCE = 5.1e-8

EXPECTED_OUTAGE_YAW_GUARD_REQUIRED_FIX_QUALITY = 4
EXPECTED_OUTAGE_YAW_GUARD_LIMITS = {
    "max_trusted_age_sec": 2.0,
    "max_trusted_variance_rad2": 0.0225,
    "max_trusted_delta_rad": 0.35,
    "max_offset_rate_radps": 0.2,
    "max_offset_step_rad": 0.04,
    "max_step_dt_sec": 0.25,
}
EXPECTED_OUTAGE_YAW_GUARD_REFERENCE_SOURCE = (
    "robust_gnss_position_alignment_yaw"
)
EXPECTED_OUTAGE_YAW_GUARD_PROPAGATION_SOURCE = "precision_local_yaw"
EXPECTED_OUTAGE_YAW_GUARD_XY_POLICY = (
    "existing_fusion_anchor_compose_precision_local"
)
OUTAGE_YAW_GUARD_STATES = {
    "DISARMED",
    "READY",
    "OUTAGE_SLEW",
    "OUTAGE_HOLD",
    "RECOVERY_RELEASE",
}
OUTAGE_YAW_GUARD_ACTIVE_STATES = {
    "OUTAGE_SLEW",
    "OUTAGE_HOLD",
    "RECOVERY_RELEASE",
}
OUTAGE_YAW_GUARD_OUTAGE_STATES = {"OUTAGE_SLEW", "OUTAGE_HOLD"}
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

NMEA_STATUS = "localization/nmea_gga_conversion"
FUSION_STATUS = "localization/gnss_map_odom_fusion"
ODOMETER_STATUS = "localization/gyro_odometer"
MATCHER_STATUS = "localization/submap_matcher"
PRECISION_GLOBAL_STATUS = "localization/precision_global_localizer"

RAW_ODOM_TOPIC = "/localization/gyro_lidar_odom"
EXISTING_GLOBAL_ODOM_TOPIC = "/localization/ekf_odom"
FUSION_AUTHORITY_TOPIC = "/localization/gnss_map_odom_fusion_authority"
FUSION_AUTHORITY_TYPE = "pure_gnss_msgs/msg/FusionAuthority"
FUSION_AUTHORITY_STATE_NAMES = {
    0: "UNHEALTHY",
    1: "FULL_SE2_HEALTHY",
    2: "SOFT_BAD_HOLD",
}
FUSION_AUTHORITY_FIX_STATES = {0, 1, 2}
FUSION_AUTHORITY_MAX_SOURCE_AGE_NS = 1_500_000_000
FUSION_AUTHORITY_MAX_FUTURE_SKEW_NS = 250_000_000
MAP_FUSION_PUBLICATION_COUNTER_KEYS = (
    "output.out_of_order_drop_count",
    "output.covered_odometry_coalesced_count",
    "output.wall_timer_coalesced_count",
    "output.total_suppressed_request_count",
)
PRECISION_GUARD_ACTIVE_REFERENCE_EPOCH_KEY = (
    "outage_yaw_guard.active_reference_epoch"
)

PRECISION_GUARD_DIAGNOSTIC_KEYS = (
    "fallback.gnss_position_enabled",
    "outage_yaw_guard.enabled",
    "outage_yaw_guard.state",
    "outage_yaw_guard.active",
    "outage_yaw_guard.reference_source",
    "outage_yaw_guard.propagation_source",
    "outage_yaw_guard.xy_policy",
    "outage_yaw_guard.config.required_fix_quality",
    "outage_yaw_guard.config.max_trusted_age_sec",
    "outage_yaw_guard.config.max_trusted_variance_rad2",
    "outage_yaw_guard.config.max_trusted_delta_rad",
    "outage_yaw_guard.config.max_offset_rate_radps",
    "outage_yaw_guard.config.max_offset_step_rad",
    "outage_yaw_guard.config.max_step_dt_sec",
    "outage_yaw_guard.trusted_variance_rad2",
    "outage_yaw_guard.active_reference_variance_rad2",
    "outage_yaw_guard.applied_offset_rad",
    "outage_yaw_guard.target_offset_rad",
    "outage_yaw_guard.additional_variance_rad2",
    "outage_yaw_guard.last_reason",
    "outage_yaw_guard.accepted_reference_count",
    "outage_yaw_guard.outage_count",
    PRECISION_GUARD_ACTIVE_REFERENCE_EPOCH_KEY,
    "outage_yaw_guard.recovery_count",
    "outage_yaw_guard.reset_count",
    "outage_yaw_guard.invalid_advance_count",
    "publish.global_suppressed_yaw_guard_invalid",
)
PRECISION_GUARD_COVARIANCE_KEYS = (
    "outage_yaw_guard.state",
    "outage_yaw_guard.active",
    "outage_yaw_guard.trusted_variance_rad2",
    "outage_yaw_guard.active_reference_variance_rad2",
    "outage_yaw_guard.applied_offset_rad",
    "outage_yaw_guard.target_offset_rad",
    "outage_yaw_guard.additional_variance_rad2",
    "outage_yaw_guard.outage_count",
    "outage_yaw_guard.recovery_count",
    "outage_yaw_guard.reset_count",
)

EXPECTED_TF_STRINGS = {
    "tf_base_to_lidar_xyz_rpy": "0.0 0.0 0.0 0.0 0.0 0.0",
    "tf_base_to_imu_xyz_rpy": "0.0 0.0 -0.1874 3.14159 0.0 0.0",
    "tf_base_to_gnss_xyz_rpy": "0.0 0.0 -0.1326 0.0 0.0 0.0",
}

COMMON_OUTPUT_TOPICS = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/diagnostics": "diagnostic_msgs/msg/DiagnosticArray",
    "/localization/gyro_lidar_odom": "nav_msgs/msg/Odometry",
    "/localization/ekf_odom": "nav_msgs/msg/Odometry",
    "/localization/gnss_odometry": "nav_msgs/msg/Odometry",
    "/localization/gnss_fusion_input": "pure_gnss_msgs/msg/GnssFusionInput",
    FUSION_AUTHORITY_TOPIC: FUSION_AUTHORITY_TYPE,
    "/localization/global_pose_with_covariance":
        "geometry_msgs/msg/PoseWithCovarianceStamped",
    "/localization/gnss_confidence": "std_msgs/msg/Float32",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}
SNAPSHOT_TOPIC = "/localization/submap_scan"
PRECISION_OUTPUT_TOPICS = {
    SNAPSHOT_TOPIC: "pure_lidar_msgs/msg/SubmapScan",
    "/localization/submap_correction": "pure_lidar_msgs/msg/SubmapCorrection",
    "/localization/precision_local_odom": "nav_msgs/msg/Odometry",
    "/localization/precision_global_odom": "nav_msgs/msg/Odometry",
    "/localization/precision_global_pose":
        "geometry_msgs/msg/PoseWithCovarianceStamped",
}


@dataclass(frozen=True)
class ConfigContract:
    role: str
    env_stem: str
    artifact: str
    source_relative: str


class Checks:
    """Collect checks without letting one missing artifact hide later failures."""

    def __init__(self) -> None:
        self.items: list[dict[str, Any]] = []

    def add(
        self, name: str, passed: bool, detail: str, warning: bool = False
    ) -> None:
        self.items.append(
            {
                "name": name,
                "passed": bool(passed),
                "detail": str(detail),
                "warning": bool(warning),
            }
        )

    @property
    def passed(self) -> bool:
        return bool(self.items) and all(
            item["passed"] or item["warning"] for item in self.items
        )


def true_value(value: Any) -> bool:
    return str(value).strip().lower() == "true"


def integer_value(value: Any) -> int:
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError(f"expected one-byte integer, got {value!r}")
        return value[0]
    return int(value)


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def lexical_absolute(path: Path) -> Path:
    """Return an absolute normalized path without resolving symlink installs."""
    return Path(os.path.abspath(os.path.normpath(os.fspath(path.expanduser()))))


def path_is_within_lexically(path: Path, parent: Path) -> bool:
    path_string = os.fspath(lexical_absolute(path))
    parent_string = os.fspath(lexical_absolute(parent))
    try:
        return os.path.commonpath((path_string, parent_string)) == parent_string
    except ValueError:
        return False


def parse_run_env_text(text: str) -> dict[str, str]:
    """Parse the runner's ``printf %q`` file without executing shell syntax."""
    result: dict[str, str] = {}
    for line_number, raw_line in enumerate(text.splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        match = re.fullmatch(r"([A-Za-z_][A-Za-z0-9_]*)=(.*)", line)
        if match is None:
            raise ValueError(f"run.env line {line_number} is malformed")
        key, encoded = match.groups()
        if key in result:
            raise ValueError(f"run.env key {key!r} is duplicated")
        try:
            tokens = shlex.split(encoded, posix=True)
        except ValueError as error:
            raise ValueError(
                f"run.env value for {key!r} is malformed: {error}"
            ) from error
        if len(tokens) != 1:
            raise ValueError(
                f"run.env value for {key!r} must decode to exactly one token"
            )
        result[key] = tokens[0]
    if not result:
        raise ValueError("run.env is empty")
    return result


def parse_effective_configurations(text: str) -> dict[str, dict[str, str]]:
    reader = csv.DictReader(io.StringIO(text), delimiter="\t")
    expected_fields = ["role", "source_path", "sha256", "artifact"]
    if reader.fieldnames != expected_fields:
        raise ValueError(
            "effective_configurations.tsv header must be "
            + "\\t".join(expected_fields)
        )
    result: dict[str, dict[str, str]] = {}
    artifacts: set[str] = set()
    for line_number, row in enumerate(reader, 2):
        if any(row.get(field) is None for field in expected_fields):
            raise ValueError(f"configuration TSV line {line_number} is malformed")
        role = row["role"]
        artifact = row["artifact"]
        if not role or role in result:
            raise ValueError(f"configuration role {role!r} is empty or duplicated")
        if not artifact or artifact in artifacts:
            raise ValueError(
                f"configuration artifact {artifact!r} is empty or duplicated"
            )
        artifacts.add(artifact)
        result[role] = dict(row)
    if not result:
        raise ValueError("effective configuration table contains no rows")
    return result


def _manifest_scalar(text: str, key: str) -> str:
    matches = re.findall(
        rf"^\s+{re.escape(key)}:\s*(.*?)\s*$", text, flags=re.MULTILINE
    )
    if len(matches) != 1:
        raise ValueError(f"dataset manifest must contain one {key!r} scalar")
    value = matches[0]
    if value.startswith('"'):
        decoded = json.loads(value)
        if not isinstance(decoded, str):
            raise ValueError(f"dataset manifest {key!r} is not a string")
        return decoded
    return value


def parse_dataset_manifest_text(text: str) -> dict[str, Any]:
    schema = re.findall(r"^schema_version:\s*(\d+)\s*$", text, re.MULTILINE)
    if schema != ["1"]:
        raise ValueError("dataset manifest schema_version must be 1")
    duration_text = _manifest_scalar(text, "duration_sec")
    try:
        duration = float(duration_text)
    except ValueError as error:
        raise ValueError("dataset duration_sec is not numeric") from error
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("dataset duration_sec must be finite and positive")

    topics: dict[str, str] = {}
    for key in ("pointcloud", "imu", "nmea"):
        topics[key] = _manifest_scalar(text, key)
        if not topics[key].startswith("/"):
            raise ValueError(f"dataset topic {key!r} is not absolute")

    transforms: dict[str, list[float]] = {}
    for key in ("base_to_lidar", "base_to_imu", "base_to_gnss"):
        encoded = _manifest_scalar(text, key)
        try:
            values = ast.literal_eval(encoded)
            numeric = [float(value) for value in values]
        except (SyntaxError, ValueError, TypeError) as error:
            raise ValueError(f"dataset transform {key!r} is malformed") from error
        if len(numeric) != 6 or not all(math.isfinite(value) for value in numeric):
            raise ValueError(f"dataset transform {key!r} must have six finite values")
        transforms[key] = numeric

    return {
        "id": _manifest_scalar(text, "id"),
        "display_name": _manifest_scalar(text, "display_name"),
        "local_bag_hint": _manifest_scalar(text, "local_bag_hint"),
        "duration_sec": duration,
        "topics": topics,
        "static_transforms": transforms,
    }


def _yaml_number(text: str, key: str) -> float:
    matches = re.findall(
        rf"^\s*{re.escape(key)}:\s*([-+0-9.eE]+)\s*$", text, re.MULTILINE
    )
    if len(matches) != 1:
        raise ValueError(f"configuration must contain one numeric {key!r}")
    return float(matches[0])


def _yaml_string(text: str, key: str) -> str:
    matches = re.findall(
        rf"^\s*{re.escape(key)}:\s*([^#\r\n]+?)\s*$", text, re.MULTILINE
    )
    if len(matches) != 1:
        raise ValueError(f"configuration must contain one string {key!r}")
    encoded = matches[0].strip()
    if encoded.startswith(('"', "'")):
        try:
            value = ast.literal_eval(encoded)
        except (SyntaxError, ValueError) as error:
            raise ValueError(f"configuration string {key!r} is malformed") from error
        if not isinstance(value, str):
            raise ValueError(f"configuration value {key!r} is not a string")
        return value
    return encoded


def _yaml_bool(text: str, key: str) -> bool:
    matches = re.findall(
        rf"^\s*{re.escape(key)}:\s*(true|false)\s*$",
        text,
        re.MULTILINE | re.IGNORECASE,
    )
    if len(matches) != 1:
        raise ValueError(f"configuration must contain one boolean {key!r}")
    return matches[0].lower() == "true"


def _yaml_numeric_list(text: str, key: str) -> list[float]:
    block = re.search(
        rf"^\s*{re.escape(key)}:\s*$((?:\n[ \t]+-\s*[-+0-9.eE]+[ \t]*)+)",
        text,
        re.MULTILINE,
    )
    if block is None:
        raise ValueError(f"configuration list {key!r} must contain numeric values")
    return [
        float(value)
        for value in re.findall(
            r"^\s+-\s*([-+0-9.eE]+)\s*$", block.group(1), re.MULTILINE
        )
    ]


def nmea_parameter_semantics(text: str) -> dict[str, Any]:
    """Read the projection contract from the ROS parameter file actually loaded."""
    projector = _yaml_string(text, "projector_type")
    vertical_datum = _yaml_string(text, "vertical_datum")
    latitude = _yaml_number(text, "map_origin.latitude")
    longitude = _yaml_number(text, "map_origin.longitude")
    altitude = _yaml_number(text, "map_origin.altitude")
    scale_factor = _yaml_number(text, "scale_factor")
    legacy_projection = _yaml_bool(text, "use_legacy_projection_params")
    gnss0 = _yaml_numeric_list(text, "gnss0")
    valid = (
        projector == EXPECTED_PROJECTOR
        and vertical_datum == EXPECTED_VERTICAL_DATUM
        and len(gnss0) == 3
        and abs(latitude - EXPECTED_ORIGIN[0]) <= 1.0e-12
        and abs(longitude - EXPECTED_ORIGIN[1]) <= 1.0e-12
        and abs(altitude) <= 1.0e-12
        and abs(scale_factor - EXPECTED_SCALE_FACTOR) <= 1.0e-12
        and not legacy_projection
        and abs(gnss0[0] - EXPECTED_ORIGIN[0]) <= 1.0e-12
        and abs(gnss0[1] - EXPECTED_ORIGIN[1]) <= 1.0e-12
        and abs(gnss0[2]) <= 1.0e-12
    )
    return {
        "valid": valid,
        "projector_type": projector,
        "vertical_datum": vertical_datum,
        "map_origin": [latitude, longitude, altitude],
        "scale_factor": scale_factor,
        "use_legacy_projection_params": legacy_projection,
        "gnss0": gnss0,
    }


def map_projector_metadata_semantics(text: str) -> dict[str, Any]:
    """Read the canonical human-facing projection metadata."""
    projector = _yaml_string(text, "projector_type")
    vertical_datum = _yaml_string(text, "vertical_datum")
    latitude = _yaml_number(text, "latitude")
    longitude = _yaml_number(text, "longitude")
    scale_factor = _yaml_number(text, "scale_factor")
    valid = (
        projector == EXPECTED_PROJECTOR
        and vertical_datum == EXPECTED_VERTICAL_DATUM
        and abs(latitude - EXPECTED_ORIGIN[0]) <= 1.0e-12
        and abs(longitude - EXPECTED_ORIGIN[1]) <= 1.0e-12
        and abs(scale_factor - EXPECTED_SCALE_FACTOR) <= 1.0e-12
    )
    return {
        "valid": valid,
        "projector_type": projector,
        "vertical_datum": vertical_datum,
        "map_origin": [latitude, longitude],
        "scale_factor": scale_factor,
    }


def projection_semantics_agree(
    parameters: dict[str, Any], metadata: dict[str, Any]
) -> bool:
    """Require the loaded ROS parameters to reproduce the canonical metadata."""
    return (
        parameters.get("valid") is True
        and metadata.get("valid") is True
        and parameters.get("projector_type") == metadata.get("projector_type")
        and parameters.get("vertical_datum") == metadata.get("vertical_datum")
        and parameters.get("map_origin", [])[:2] == metadata.get("map_origin")
        and parameters.get("scale_factor") == metadata.get("scale_factor")
    )


def empty_override_semantics(text: str) -> dict[str, Any]:
    """Accept only a wildcard ROS parameter document with an empty map."""
    valid = re.fullmatch(
        r"\s*/\*\*:\s*\n\s+ros__parameters:\s*\{\s*\}\s*", text
    ) is not None
    return {"valid": valid, "parameter_override_count": 0 if valid else None}


def fusion_override_semantics(text: str) -> dict[str, Any]:
    matches = re.findall(
        r"^\s*xy_only_recovery\.enabled:\s*(true|false)\s*$",
        text,
        re.MULTILINE | re.IGNORECASE,
    )
    return {
        "valid": len(matches) == 1 and matches[0].lower() == "true",
        "xy_only_recovery_enabled": (
            matches[0].lower() == "true" if len(matches) == 1 else None
        ),
    }


def precision_global_parameter_semantics(text: str) -> dict[str, Any]:
    """Read the publication-required outage-yaw guard parameter contract."""
    fallback_enabled = _yaml_bool(text, "fallback.gnss_position_enabled")
    guard_enabled = _yaml_bool(text, "outage_yaw_guard.enabled")
    required_fix_quality = _yaml_number(
        text, "outage_yaw_guard.required_fix_quality"
    )
    limits = {
        key: _yaml_number(text, f"outage_yaw_guard.{key}")
        for key in EXPECTED_OUTAGE_YAW_GUARD_LIMITS
    }
    valid = (
        not fallback_enabled
        and guard_enabled
        and math.isfinite(required_fix_quality)
        and required_fix_quality
        == float(EXPECTED_OUTAGE_YAW_GUARD_REQUIRED_FIX_QUALITY)
        and all(
            math.isfinite(limits[key])
            and abs(limits[key] - expected) <= 1.0e-12
            for key, expected in EXPECTED_OUTAGE_YAW_GUARD_LIMITS.items()
        )
    )
    return {
        "valid": valid,
        "fallback_gnss_position_enabled": fallback_enabled,
        "outage_yaw_guard_enabled": guard_enabled,
        "required_fix_quality": required_fix_quality,
        "limits": limits,
    }


def dataset_identity(dataset: str) -> tuple[str, str]:
    if dataset == "course-1":
        return "course_1", "Hesai 32-Line + IMU + RTK GNSS — Course 1"
    if dataset == "course-2":
        return "course_2", "Hesai 32-Line + IMU + RTK GNSS — Course 2"
    raise ValueError(f"unsupported dataset: {dataset}")


def config_contracts(dataset: str, mode: str) -> list[ConfigContract]:
    dataset_id, _ = dataset_identity(dataset)
    snapshot_enabled = mode in ("control", "precision")
    contracts = [
        ConfigContract(
            "imu_base", "imu_param", "imu_param.yaml",
            "src/pure_imu_undistortion/param/param_xt.yaml",
        ),
        ConfigContract(
            "odometry_base", "odom_param", "odom_param.yaml",
            "src/pure_lidar_gyro_odometer/param/param_xt_lidar_imu_only.yaml",
        ),
        ConfigContract(
            "odometry_override", "odom_override_param", "odom_override_param.yaml",
            (
                "src/pure_precision_bringup/config/submap_snapshot_override.yaml"
                if snapshot_enabled
                else "src/pure_odometry_bringup/config/autoware_lsim/empty_params.yaml"
            ),
        ),
        ConfigContract(
            "nmea_base", "nmea_gnss_param", "nmea_gnss_param.yaml",
            "src/pure_nmea_gnss_conversion/param/param.yaml",
        ),
        ConfigContract(
            "nmea_projector_metadata", "nmea_projector_metadata",
            "map_projector_info.yaml",
            "src/pure_nmea_gnss_conversion/config/map_projector_info.yaml",
        ),
        ConfigContract(
            "nmea_override", "nmea_gnss_override_param", "nmea_override_param.yaml",
            "src/pure_odometry_bringup/config/autoware_lsim/empty_params.yaml",
        ),
        ConfigContract(
            "gnss_fusion_base", "gnss_fusion_param", "gnss_fusion_param.yaml",
            "src/pure_gnss_map_odom_fusion/param/param.yaml",
        ),
        ConfigContract(
            "gnss_fusion_override", "gnss_fusion_override_param",
            "gnss_fusion_single_antenna.yaml",
            "src/pure_localization_evaluation_profiles/config/odometry/lidar_imu_gnss/"
            "hesai_32line_rtk/accepted/gnss_fusion_single_antenna.yaml",
        ),
        ConfigContract(
            "dataset_manifest", "dataset_manifest", "dataset_manifest.yaml",
            "src/pure_localization_evaluation_profiles/config/odometry/lidar_imu_gnss/"
            f"hesai_32line_rtk/datasets/{dataset_id}.yaml",
        ),
        ConfigContract(
            "diagnostic_aggregator", "diagnostic_aggregator_param",
            "diagnostic_aggregator.yaml",
            "src/pure_odometry_bringup/config/diagnostic_aggregator.yaml",
        ),
    ]
    if snapshot_enabled:
        contracts.append(
            ConfigContract(
                "precision_profile_manifest", "precision_profile_manifest",
                "precision_profile.yaml",
                "src/pure_localization_evaluation_profiles/config/precision/lidar_imu_gnss/"
                "hesai_32line_rtk/profile.yaml",
            )
        )
    if mode == "precision":
        contracts.extend(
            [
                ConfigContract(
                    "precision_matcher_base", "precision_matcher_param",
                    "precision_matcher_param.yaml",
                    "src/pure_lidar_submap_matcher/param/param.yaml",
                ),
                ConfigContract(
                    "precision_matcher_override", "precision_matcher_override_param",
                    "precision_matcher_override_param.yaml",
                    "src/pure_precision_bringup/config/empty_params.yaml",
                ),
                ConfigContract(
                    "precision_global_base", "precision_global_param",
                    "precision_global_param.yaml",
                    "src/pure_precision_global_localizer/param/param.yaml",
                ),
                ConfigContract(
                    "precision_global_override", "precision_global_override_param",
                    "precision_global_override_param.yaml",
                    "src/pure_precision_bringup/config/empty_params.yaml",
                ),
            ]
        )
    return contracts


def validate_run_environment(
    env: dict[str, str], repo: Path, dataset: str, mode: str,
    manifest: dict[str, Any], checks: Checks,
) -> dict[str, Any]:
    dataset_id, display_name = dataset_identity(dataset)
    expected_mode = "precision" if mode == "precision" else "baseline"
    expected_control = mode == "control"
    expected = {
        "evaluation_profile": EXPECTED_PROFILE,
        "dataset": dataset,
        "dataset_id": dataset_id,
        "dataset_display_name": display_name,
        "localization_mode": expected_mode,
        "odometer_tracking_mode": "scan_to_scan",
        "accepted_scan_control": str(expected_control).lower(),
        "tf_policy": "isolate_all",
        "record_output": "true",
        "lsim_interface_test": "false",
        "playback_duration": "",
        **EXPECTED_TF_STRINGS,
    }
    mismatches = {
        key: {"expected": value, "actual": env.get(key)}
        for key, value in expected.items()
        if env.get(key) != value
    }
    checks.add(
        "run.env dataset, mode, full-playback, TF, and native-run contract",
        not mismatches,
        "exact contract matched" if not mismatches else json.dumps(mismatches, sort_keys=True),
    )
    try:
        rate = float(env.get("rate", "nan"))
    except ValueError:
        rate = math.nan
    checks.add(
        "real-time playback rate",
        math.isfinite(rate) and abs(rate - 1.0) <= 1.0e-12,
        f"rate={env.get('rate')!r}",
    )
    checks.add(
        "dataset manifest identity",
        manifest["id"] == dataset_id and manifest["display_name"] == display_name,
        f"id={manifest['id']!r} display_name={manifest['display_name']!r}",
    )
    expected_transform_arrays = {
        "base_to_lidar": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "base_to_imu": [0.0, 0.0, -0.1874, 3.14159, 0.0, 0.0],
        "base_to_gnss": [0.0, 0.0, -0.1326, 0.0, 0.0, 0.0],
    }
    checks.add(
        "dataset manifest static transforms",
        manifest["static_transforms"] == expected_transform_arrays,
        json.dumps(manifest["static_transforms"], sort_keys=True),
    )

    bag_path_text = env.get("bag", "")
    actual_bag = lexical_absolute(Path(bag_path_text)) if bag_path_text else None
    expected_bag = lexical_absolute(repo / manifest["local_bag_hint"])
    checks.add(
        "private input bag selected by descriptive manifest",
        actual_bag == expected_bag,
        f"expected={expected_bag} actual={actual_bag}",
    )

    inactive_stems = []
    if mode == "baseline":
        inactive_stems.append("precision_profile_manifest")
    if mode != "precision":
        inactive_stems.extend(
            [
                "precision_matcher_param",
                "precision_matcher_override_param",
                "precision_global_param",
                "precision_global_override_param",
            ]
        )
    inactive_stems.append("lsim_adapter_param")
    inactive_mismatches = {}
    for stem in inactive_stems:
        if env.get(stem) != "" or env.get(stem + "_sha256") != "n/a":
            inactive_mismatches[stem] = {
                "path": env.get(stem),
                "sha256": env.get(stem + "_sha256"),
            }
    checks.add(
        "inactive mode configuration is absent",
        not inactive_mismatches,
        "inactive fields are empty/n/a" if not inactive_mismatches else
        json.dumps(inactive_mismatches, sort_keys=True),
    )
    return {
        "rate": rate,
        "input_bag": str(actual_bag) if actual_bag is not None else None,
        "expected_input_bag": str(expected_bag),
        "mismatches": mismatches,
        "inactive_configuration_mismatches": inactive_mismatches,
    }


def validate_config_provenance(
    repo: Path, run_dir: Path, env: dict[str, str], rows: dict[str, dict[str, str]],
    contracts: list[ConfigContract], checks: Checks,
) -> dict[str, Any]:
    expected_roles = {contract.role for contract in contracts}
    actual_roles = set(rows)
    checks.add(
        "effective configuration role set",
        actual_roles == expected_roles,
        f"expected={sorted(expected_roles)} actual={sorted(actual_roles)}",
    )
    artifact_root = run_dir / "artifacts"
    install_root = repo / "install"
    summaries: dict[str, Any] = {}
    for contract in contracts:
        source = repo / contract.source_relative
        env_path_text = env.get(contract.env_stem, "")
        env_hash = env.get(contract.env_stem + "_sha256", "")
        installed = lexical_absolute(Path(env_path_text)) if env_path_text else None
        row = rows.get(contract.role)
        artifact = artifact_root / contract.artifact
        source_hash = sha256_file(source) if source.is_file() else None
        installed_hash = (
            sha256_file(installed) if installed is not None and installed.is_file() else None
        )
        artifact_hash = sha256_file(artifact) if artifact.is_file() else None
        row_hash = row.get("sha256") if row is not None else None
        row_source = (
            lexical_absolute(Path(row["source_path"]))
            if row is not None and row.get("source_path") else None
        )
        valid = (
            source_hash is not None
            and installed is not None
            and path_is_within_lexically(installed, install_root)
            and installed_hash == source_hash
            and artifact_hash == source_hash
            and env_hash == source_hash
            and row_hash == source_hash
            and row_source == installed
            and row is not None
            and row.get("artifact") == contract.artifact
        )
        detail = (
            f"sha256={source_hash} repo/install/artifact/run.env/TSV agree"
            if valid else
            json.dumps(
                {
                    "repo_sha256": source_hash,
                    "install_path": str(installed) if installed else None,
                    "install_under_workspace": (
                        path_is_within_lexically(installed, install_root)
                        if installed is not None else False
                    ),
                    "install_sha256": installed_hash,
                    "artifact_sha256": artifact_hash,
                    "run_env_sha256": env_hash,
                    "tsv_sha256": row_hash,
                    "tsv_source": str(row_source) if row_source else None,
                    "tsv_artifact": row.get("artifact") if row else None,
                },
                sort_keys=True,
            )
        )
        checks.add(f"configuration provenance: {contract.role}", valid, detail)
        summaries[contract.role] = {
            "source": str(source),
            "installed": str(installed) if installed else None,
            "artifact": str(artifact),
            "sha256": source_hash,
            "valid": valid,
        }
    return summaries


def import_rosbag_modules():
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python environment is required; source /opt/ros and this workspace"
        ) from error
    return rosbag2_py, deserialize_message, get_message


def bag_metadata(path: Path) -> tuple[Any, dict[str, int], dict[str, str]]:
    rosbag2_py, _, _ = import_rosbag_modules()
    if not (path / "metadata.yaml").is_file():
        raise RuntimeError(f"completed bag metadata is missing: {path}")
    metadata = rosbag2_py.Info().read_metadata(str(path), "mcap")
    counts: dict[str, int] = {}
    types: dict[str, str] = {}
    for item in metadata.topics_with_message_count:
        name = str(item.topic_metadata.name)
        if name in counts:
            raise RuntimeError(f"bag metadata duplicates topic {name}")
        counts[name] = int(item.message_count)
        types[name] = str(item.topic_metadata.type)
    return metadata, counts, types


def _status_matches(actual: str, expected: str) -> bool:
    return actual == expected or actual.endswith("/" + expected)


def _is_audited_zero_counter(key: str) -> bool:
    lowered = key.lower()
    return (
        lowered == "malformed_count"
        or lowered.endswith(".malformed_count")
        or "backstep" in lowered
        or lowered == "raw.nonmonotonic"
    )


def _counter_integer(value: str) -> int:
    numeric = float(value)
    if not math.isfinite(numeric) or numeric < 0.0 or numeric != math.floor(numeric):
        raise ValueError(value)
    return int(numeric)


def _strict_diagnostic_counter_integer(value: Any) -> int:
    """Parse the canonical decimal representation emitted by ``to_string``."""
    if not isinstance(value, str) or re.fullmatch(r"(?:0|[1-9][0-9]*)", value) is None:
        raise ValueError(value)
    return int(value)


def scan_output_bag(path: Path) -> dict[str, Any]:
    rosbag2_py, deserialize_message, get_message = import_rosbag_modules()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required_deserialization_types = {
        "/clock": "rosgraph_msgs/msg/Clock",
        "/diagnostics": "diagnostic_msgs/msg/DiagnosticArray",
        RAW_ODOM_TOPIC: "nav_msgs/msg/Odometry",
        EXISTING_GLOBAL_ODOM_TOPIC: "nav_msgs/msg/Odometry",
        FUSION_AUTHORITY_TOPIC: FUSION_AUTHORITY_TYPE,
    }
    missing = set(required_deserialization_types) - set(topic_types)
    wrong_types = {
        topic: {"expected": expected, "actual": topic_types.get(topic)}
        for topic, expected in required_deserialization_types.items()
        if topic in topic_types and topic_types[topic] != expected
    }
    if missing or wrong_types:
        raise RuntimeError(
            f"runtime topics missing={sorted(missing)} wrong_types={wrong_types}"
        )
    classes = {
        topic: get_message(topic_types[topic]) for topic in required_deserialization_types
    }

    clock_values: list[int] = []
    status_counts: dict[str, int] = {}
    status_stamps: dict[str, list[int]] = {}
    nmea_snapshots: list[dict[str, Any]] = []
    precision_guard_snapshots: list[dict[str, Any]] = []
    map_fusion_publication_snapshots: list[dict[str, Any]] = []
    raw_odom_events: list[dict[str, int]] = []
    existing_global_odom_events: list[dict[str, int]] = []
    fusion_authority_events: list[dict[str, Any]] = []
    audited_counters: dict[str, dict[str, Any]] = {}
    duplicate_key_samples = 0
    duplicate_status_samples = 0
    scanned_count = 0
    while reader.has_next():
        topic, serialized, record_stamp_ns = reader.read_next()
        scanned_count += 1
        record_index = scanned_count - 1
        if topic == "/clock":
            message = deserialize_message(serialized, classes[topic])
            clock_values.append(stamp_ns(message.clock))
            continue
        if topic in (RAW_ODOM_TOPIC, EXISTING_GLOBAL_ODOM_TOPIC):
            message = deserialize_message(serialized, classes[topic])
            stamp_sec = int(message.header.stamp.sec)
            stamp_nanosec = int(message.header.stamp.nanosec)
            event = {
                "record_index": record_index,
                "record_stamp_ns": int(record_stamp_ns),
                "stamp_ns": stamp_sec * 1_000_000_000 + stamp_nanosec,
                "stamp_canonical": (
                    stamp_sec >= 0 and 0 <= stamp_nanosec < 1_000_000_000
                ),
            }
            if topic == RAW_ODOM_TOPIC:
                raw_odom_events.append(event)
            else:
                existing_global_odom_events.append(event)
            continue
        if topic == FUSION_AUTHORITY_TOPIC:
            message = deserialize_message(serialized, classes[topic])
            stamp_sec = int(message.header.stamp.sec)
            stamp_nanosec = int(message.header.stamp.nanosec)
            source_sec = int(message.source_stamp.sec)
            source_nanosec = int(message.source_stamp.nanosec)
            fusion_authority_events.append(
                {
                    "record_index": record_index,
                    "record_stamp_ns": int(record_stamp_ns),
                    "stamp_ns": stamp_sec * 1_000_000_000 + stamp_nanosec,
                    "stamp_canonical": (
                        stamp_sec >= 0 and 0 <= stamp_nanosec < 1_000_000_000
                    ),
                    "source_stamp_ns": source_sec * 1_000_000_000 + source_nanosec,
                    "source_stamp_canonical": (
                        source_sec >= 0 and 0 <= source_nanosec < 1_000_000_000
                    ),
                    "frame_id": str(message.header.frame_id),
                    "session_id": int(message.session_id),
                    "sequence": int(message.sequence),
                    "state": int(message.state),
                    "recovery_state": str(message.recovery_state),
                    "anchor_valid": bool(message.anchor_valid),
                    "position_fused": bool(message.position_fused),
                    "yaw_fused": bool(message.yaw_fused),
                    "last_fix_state": int(message.last_fix_state),
                    "reason": str(message.reason),
                }
            )
            continue
        if topic != "/diagnostics":
            continue
        message = deserialize_message(serialized, classes[topic])
        array_stamp = stamp_ns(message.header.stamp)
        names_in_array: set[str] = set()
        for status in message.status:
            status_name = str(status.name)
            if status_name in names_in_array:
                duplicate_status_samples += 1
            names_in_array.add(status_name)
            status_counts[status_name] = status_counts.get(status_name, 0) + 1
            status_stamps.setdefault(status_name, []).append(array_stamp)
            values: dict[str, str] = {}
            duplicate_keys: list[str] = []
            for item in status.values:
                key = str(item.key)
                if key in values:
                    duplicate_keys.append(key)
                values[key] = str(item.value)
            if duplicate_keys:
                duplicate_key_samples += 1
            if _status_matches(status_name, NMEA_STATUS):
                nmea_snapshots.append(
                    {
                        "stamp_ns": array_stamp,
                        "level": integer_value(status.level),
                        "message": str(status.message),
                        "duplicate_keys": sorted(set(duplicate_keys)),
                        "values": {
                            key: values.get(key)
                            for key in (
                                "projector_type",
                                "vertical_datum",
                                "projection_mode",
                                "map_origin_latitude",
                                "map_origin_longitude",
                                "scale_factor",
                                "has_last_primary",
                                "has_last_output",
                                "last_parse_error",
                            )
                        },
                    }
                )
            if _status_matches(status_name, FUSION_STATUS):
                key_counts = {
                    key: 0 for key in MAP_FUSION_PUBLICATION_COUNTER_KEYS
                }
                for item in status.values:
                    key = str(item.key)
                    if key in key_counts:
                        key_counts[key] += 1
                map_fusion_publication_snapshots.append(
                    {
                        "record_index": record_index,
                        "record_stamp_ns": int(record_stamp_ns),
                        "stamp_ns": array_stamp,
                        "duplicate_keys": sorted(set(duplicate_keys)),
                        "key_counts": key_counts,
                        "values": {
                            key: values.get(key)
                            for key in MAP_FUSION_PUBLICATION_COUNTER_KEYS
                        },
                    }
                )
            if _status_matches(status_name, PRECISION_GLOBAL_STATUS):
                guard_key_counts = {
                    key: 0 for key in PRECISION_GUARD_DIAGNOSTIC_KEYS
                }
                for item in status.values:
                    key = str(item.key)
                    if key in guard_key_counts:
                        guard_key_counts[key] += 1
                precision_guard_snapshots.append(
                    {
                        "stamp_ns": array_stamp,
                        "level": integer_value(status.level),
                        "message": str(status.message),
                        "duplicate_keys": sorted(set(duplicate_keys)),
                        "key_counts": guard_key_counts,
                        "values": {
                            key: values.get(key)
                            for key in PRECISION_GUARD_DIAGNOSTIC_KEYS
                            if key != PRECISION_GUARD_ACTIVE_REFERENCE_EPOCH_KEY
                            or key in values
                        },
                    }
                )
            for key, value in values.items():
                if not _is_audited_zero_counter(key):
                    continue
                identity = f"{status_name}:{key}"
                summary = audited_counters.setdefault(
                    identity,
                    {
                        "status": status_name,
                        "key": key,
                        "samples": 0,
                        "maximum": 0,
                        "malformed_values": [],
                    },
                )
                summary["samples"] += 1
                try:
                    summary["maximum"] = max(
                        int(summary["maximum"]), _counter_integer(value)
                    )
                except ValueError:
                    if len(summary["malformed_values"]) < 10:
                        summary["malformed_values"].append(value)
    return {
        "scanned_message_count": scanned_count,
        "clock_values": clock_values,
        "status_counts": status_counts,
        "status_stamps": status_stamps,
        "nmea_snapshots": nmea_snapshots,
        "precision_guard_snapshots": precision_guard_snapshots,
        "map_fusion_publication_snapshots": map_fusion_publication_snapshots,
        "raw_odom_events": raw_odom_events,
        "existing_global_odom_events": existing_global_odom_events,
        "fusion_authority_events": fusion_authority_events,
        "audited_counters": audited_counters,
        "duplicate_key_samples": duplicate_key_samples,
        "duplicate_status_samples": duplicate_status_samples,
    }


def nondecreasing_physical_time(values: Iterable[int]) -> dict[str, Any]:
    sequence = [int(value) for value in values]
    first_positive = next(
        (index for index, value in enumerate(sequence) if value > 0), None
    )
    if first_positive is None:
        return {
            "valid": False,
            "leading_zero_count": len(sequence),
            "positive_count": 0,
            "first_positive_ns": None,
            "last_positive_ns": None,
            "span_sec": None,
        }
    positive = sequence[first_positive:]
    valid = (
        all(value == 0 for value in sequence[:first_positive])
        and all(value > 0 for value in positive)
        and all(later >= earlier for earlier, later in zip(positive, positive[1:]))
    )
    return {
        "valid": valid,
        "leading_zero_count": first_positive,
        "positive_count": len(positive),
        "first_positive_ns": positive[0],
        "last_positive_ns": positive[-1],
        "span_sec": (positive[-1] - positive[0]) * 1.0e-9,
    }


def fusion_authority_stream_contract(
    events: list[dict[str, Any]],
) -> dict[str, Any]:
    """Audit the same fail-closed ordering fields consumed by precision-global."""
    active_session: int | None = None
    active_sequence = 0
    active_stamp_ns = 0
    retired_sessions: set[int] = set()
    observed_sessions: set[int] = set()
    previous_record_index = -1
    state_counts = {name: 0 for name in FUSION_AUTHORITY_STATE_NAMES.values()}
    invalid_state_count = 0
    violation_count = 0
    violation_examples: list[dict[str, Any]] = []

    for index, event in enumerate(events):
        issues: list[str] = []
        record_index = int(event.get("record_index", -1))
        stamp = int(event.get("stamp_ns", 0))
        source_stamp = int(event.get("source_stamp_ns", 0))
        session = int(event.get("session_id", 0))
        sequence = int(event.get("sequence", 0))
        state = int(event.get("state", -1))
        fix_state = int(event.get("last_fix_state", -1))

        if state in FUSION_AUTHORITY_STATE_NAMES:
            state_counts[FUSION_AUTHORITY_STATE_NAMES[state]] += 1
        else:
            invalid_state_count += 1
            issues.append("invalid_state")
        if fix_state not in FUSION_AUTHORITY_FIX_STATES:
            issues.append("invalid_last_fix_state")
        if record_index <= previous_record_index:
            issues.append("record_order_not_increasing")
        previous_record_index = record_index
        if not event.get("stamp_canonical", False) or stamp <= 0:
            issues.append("invalid_publish_stamp")
        if not event.get("source_stamp_canonical", False) or source_stamp <= 0:
            issues.append("invalid_source_stamp")
        if source_stamp > stamp + FUSION_AUTHORITY_MAX_FUTURE_SKEW_NS:
            issues.append("source_stamp_exceeds_future_skew")
        if stamp - source_stamp > FUSION_AUTHORITY_MAX_SOURCE_AGE_NS:
            issues.append("source_stamp_stale")
        if str(event.get("frame_id", "")) != "map":
            issues.append("unexpected_frame_id")
        if session <= 0:
            issues.append("invalid_session_id")
        else:
            observed_sessions.add(session)
        if sequence <= 0:
            issues.append("invalid_sequence")

        recovery_state = str(event.get("recovery_state", ""))
        anchor_valid = event.get("anchor_valid") is True
        position_fused = event.get("position_fused") is True
        yaw_fused = event.get("yaw_fused") is True
        reason = str(event.get("reason", ""))
        if state == 1 and not (
            recovery_state == "tracking"
            and anchor_valid
            and position_fused
            and yaw_fused
            and fix_state == 1
        ):
            issues.append("full_se2_payload_inconsistent")
        if state == 2 and not (
            recovery_state == "tracking"
            and anchor_valid
            and position_fused
            and yaw_fused
            and fix_state == 2
            and reason == "gnss_soft_bad_within_grace"
        ):
            issues.append("soft_bad_hold_payload_inconsistent")

        order_issue: str | None = None
        order_fields_valid = session > 0 and sequence > 0 and stamp > 0
        if order_fields_valid:
            if session in retired_sessions:
                order_issue = "retired_session_event"
            elif active_session is None:
                pass
            elif session == active_session:
                if sequence <= active_sequence:
                    order_issue = "sequence_not_increasing"
                elif stamp + 1 < active_stamp_ns:
                    order_issue = "publish_stamp_backstep"
            elif stamp <= active_stamp_ns + 1:
                order_issue = "new_session_stamp_not_increasing"
        if order_issue is not None:
            issues.append(order_issue)

        if not issues:
            if active_session is not None and session != active_session:
                retired_sessions.add(active_session)
            active_session = session
            active_sequence = sequence
            active_stamp_ns = stamp
        else:
            violation_count += 1
            if len(violation_examples) < 20:
                violation_examples.append(
                    {
                        "event_index": index,
                        "record_index": record_index,
                        "session_id": session,
                        "sequence": sequence,
                        "stamp_ns": stamp,
                        "source_stamp_ns": source_stamp,
                        "issues": issues,
                    }
                )

    return {
        "valid": bool(events) and violation_count == 0,
        "event_count": len(events),
        "session_count": len(observed_sessions),
        "session_ids": sorted(observed_sessions),
        "retired_session_count": len(retired_sessions),
        "state_counts": state_counts,
        "soft_bad_hold_count": state_counts["SOFT_BAD_HOLD"],
        "invalid_state_count": invalid_state_count,
        "violation_count": violation_count,
        "violation_examples": violation_examples,
    }


def nmea_diagnostic_contract(snapshots: list[dict[str, Any]]) -> dict[str, Any]:
    mismatches: list[dict[str, Any]] = []
    primary_count = 0
    output_count = 0
    for index, snapshot in enumerate(snapshots):
        values = snapshot["values"]
        actual = {
            "projector_type": values.get("projector_type"),
            "vertical_datum": values.get("vertical_datum"),
            "projection_mode": values.get("projection_mode"),
        }
        valid = (
            not snapshot.get("duplicate_keys")
            and actual["projector_type"] == EXPECTED_PROJECTOR
            and actual["vertical_datum"] == EXPECTED_VERTICAL_DATUM
            and actual["projection_mode"] == EXPECTED_PROJECTION_MODE
            and values.get("last_parse_error") == "none"
        )
        try:
            latitude = float(values.get("map_origin_latitude", "nan"))
            longitude = float(values.get("map_origin_longitude", "nan"))
            scale = float(values.get("scale_factor", "nan"))
        except (TypeError, ValueError):
            latitude = longitude = scale = math.nan
        valid = valid and (
            math.isfinite(latitude)
            and math.isfinite(longitude)
            and math.isfinite(scale)
            and abs(latitude - EXPECTED_ORIGIN[0]) <= ORIGIN_DIAGNOSTIC_TOLERANCE_DEG
            and abs(longitude - EXPECTED_ORIGIN[1]) <= ORIGIN_DIAGNOSTIC_TOLERANCE_DEG
            and abs(scale - EXPECTED_SCALE_FACTOR) <= SCALE_DIAGNOSTIC_TOLERANCE
        )
        primary_count += true_value(values.get("has_last_primary", "false"))
        output_count += true_value(values.get("has_last_output", "false"))
        if not valid and len(mismatches) < 20:
            mismatches.append(
                {
                    "index": index,
                    "stamp_ns": snapshot.get("stamp_ns"),
                    "values": values,
                    "duplicate_keys": snapshot.get("duplicate_keys"),
                }
            )
    order = nondecreasing_physical_time(
        snapshot.get("stamp_ns", 0) for snapshot in snapshots
    )
    valid = (
        bool(snapshots)
        and not mismatches
        and primary_count > 0
        and output_count > 0
        and order["valid"]
    )
    return {
        "valid": valid,
        "snapshot_count": len(snapshots),
        "primary_observed_count": primary_count,
        "output_observed_count": output_count,
        "configuration_mismatch_count": len(mismatches),
        "configuration_mismatch_examples": mismatches,
        "stamp_order": order,
    }


def precision_guard_diagnostic_contract(
    snapshots: list[dict[str, Any]],
) -> dict[str, Any]:
    """Validate the publication guard configuration, counters, and covariance."""
    expected_strings = {
        "fallback.gnss_position_enabled": "false",
        "outage_yaw_guard.enabled": "true",
        "outage_yaw_guard.reference_source": (
            EXPECTED_OUTAGE_YAW_GUARD_REFERENCE_SOURCE
        ),
        "outage_yaw_guard.propagation_source": (
            EXPECTED_OUTAGE_YAW_GUARD_PROPAGATION_SOURCE
        ),
        "outage_yaw_guard.xy_policy": EXPECTED_OUTAGE_YAW_GUARD_XY_POLICY,
    }
    expected_numbers = {
        "outage_yaw_guard.config.required_fix_quality": float(
            EXPECTED_OUTAGE_YAW_GUARD_REQUIRED_FIX_QUALITY
        ),
        **{
            f"outage_yaw_guard.config.{key}": value
            for key, value in EXPECTED_OUTAGE_YAW_GUARD_LIMITS.items()
        },
    }
    mismatch_count = 0
    mismatch_examples: list[dict[str, Any]] = []
    accepted_counts: list[int] = []
    invalid_advance_counts: list[int] = []
    suppressed_invalid_counts: list[int] = []
    active_reference_variances: list[float] = []
    covariance_issue_count = 0
    previous_counters: dict[str, int] = {}
    previous_guard: dict[str, Any] | None = None
    epoch_presence = [
        PRECISION_GUARD_ACTIVE_REFERENCE_EPOCH_KEY
        in snapshot.get("values", {})
        for snapshot in snapshots
    ]
    epoch_accounting_available = bool(epoch_presence) and all(epoch_presence)
    epoch_accounting_mixed = any(epoch_presence) and not all(epoch_presence)
    unproven_active_reference_intervals = 0
    cross_epoch_active_reference_intervals = 0
    counter_keys = [
        "outage_yaw_guard.accepted_reference_count",
        "outage_yaw_guard.outage_count",
        "outage_yaw_guard.recovery_count",
        "outage_yaw_guard.reset_count",
        "outage_yaw_guard.invalid_advance_count",
        "publish.global_suppressed_yaw_guard_invalid",
    ]
    if epoch_accounting_available:
        counter_keys.append(PRECISION_GUARD_ACTIVE_REFERENCE_EPOCH_KEY)

    for index, snapshot in enumerate(snapshots):
        values = snapshot.get("values", {})
        issues: dict[str, Any] = {}
        covariance_issues: dict[str, Any] = {}
        if snapshot.get("duplicate_keys"):
            issues["duplicate_keys"] = snapshot["duplicate_keys"]
        if index == 0 and epoch_accounting_mixed:
            issues["outage_yaw_guard.active_reference_epoch.presence"] = {
                "expected": "present in every snapshot or absent in every legacy snapshot",
                "actual": "mixed",
            }

        key_counts = snapshot.get("key_counts")
        covariance_keys = list(PRECISION_GUARD_COVARIANCE_KEYS)
        if epoch_accounting_available:
            covariance_keys.append(PRECISION_GUARD_ACTIVE_REFERENCE_EPOCH_KEY)
        for key in covariance_keys:
            count = key_counts.get(key) if isinstance(key_counts, dict) else None
            if count != 1:
                covariance_issues[f"{key}.count"] = {
                    "expected": 1,
                    "actual": count,
                }

        for key, expected in expected_strings.items():
            actual = values.get(key)
            if actual != expected:
                issues[key] = {"expected": expected, "actual": actual}

        for key, expected in expected_numbers.items():
            actual = values.get(key)
            try:
                numeric = float(actual)
            except (TypeError, ValueError):
                numeric = math.nan
            if not math.isfinite(numeric) or abs(numeric - expected) > 1.0e-12:
                issues[key] = {"expected": expected, "actual": actual}

        parsed_counters: dict[str, int] = {}
        for key in counter_keys:
            actual = values.get(key)
            try:
                parsed_counters[key] = _counter_integer(actual)
            except (TypeError, ValueError):
                issues[key] = {"expected": "non-negative integer", "actual": actual}
        for key, counter in parsed_counters.items():
            previous = previous_counters.get(key)
            if previous is not None and counter < previous:
                issues[f"{key}.monotonic"] = {
                    "previous": previous,
                    "actual": counter,
                }
            previous_counters[key] = counter

        accepted = parsed_counters.get(
            "outage_yaw_guard.accepted_reference_count"
        )
        if accepted is not None:
            accepted_counts.append(accepted)

        invalid_advance = parsed_counters.get(
            "outage_yaw_guard.invalid_advance_count"
        )
        if invalid_advance is not None:
            invalid_advance_counts.append(invalid_advance)
            if invalid_advance != 0:
                issues["outage_yaw_guard.invalid_advance_count.zero"] = {
                    "expected": 0,
                    "actual": invalid_advance,
                }

        suppressed_invalid = parsed_counters.get(
            "publish.global_suppressed_yaw_guard_invalid"
        )
        if suppressed_invalid is not None:
            suppressed_invalid_counts.append(suppressed_invalid)
            if suppressed_invalid != 0:
                issues["publish.global_suppressed_yaw_guard_invalid.zero"] = {
                    "expected": 0,
                    "actual": suppressed_invalid,
                }

        state = values.get("outage_yaw_guard.state")
        last_reason = values.get("outage_yaw_guard.last_reason")
        active_text = values.get("outage_yaw_guard.active")
        active = active_text == "true"
        if state not in OUTAGE_YAW_GUARD_STATES:
            covariance_issues["outage_yaw_guard.state"] = {
                "expected": sorted(OUTAGE_YAW_GUARD_STATES),
                "actual": state,
            }
        if active_text not in {"true", "false"}:
            covariance_issues["outage_yaw_guard.active"] = {
                "expected": "boolean",
                "actual": active_text,
            }
        elif state in OUTAGE_YAW_GUARD_STATES and active != (
            state in OUTAGE_YAW_GUARD_ACTIVE_STATES
        ):
            covariance_issues["outage_yaw_guard.active_state"] = {
                "state": state,
                "actual": active_text,
            }
        if not isinstance(last_reason, str) or not last_reason:
            covariance_issues["outage_yaw_guard.last_reason"] = {
                "expected": "non-empty runtime branch reason",
                "actual": last_reason,
            }

        numeric: dict[str, float] = {}
        for key in (
            "outage_yaw_guard.trusted_variance_rad2",
            "outage_yaw_guard.active_reference_variance_rad2",
            "outage_yaw_guard.applied_offset_rad",
            "outage_yaw_guard.target_offset_rad",
            "outage_yaw_guard.additional_variance_rad2",
        ):
            actual = values.get(key)
            try:
                numeric[key] = float(actual)
            except (TypeError, ValueError):
                numeric[key] = math.nan
                covariance_issues[key] = {
                    "expected": "floating-point diagnostic value",
                    "actual": actual,
                }
            if math.isinf(numeric[key]):
                covariance_issues[key] = {
                    "expected": "finite or state-required NaN",
                    "actual": actual,
                }

        trusted_variance = numeric[
            "outage_yaw_guard.trusted_variance_rad2"
        ]
        active_reference_variance = numeric[
            "outage_yaw_guard.active_reference_variance_rad2"
        ]
        applied_offset = numeric["outage_yaw_guard.applied_offset_rad"]
        target_offset = numeric["outage_yaw_guard.target_offset_rad"]
        additional_variance = numeric[
            "outage_yaw_guard.additional_variance_rad2"
        ]
        maximum_trusted_variance = EXPECTED_OUTAGE_YAW_GUARD_LIMITS[
            "max_trusted_variance_rad2"
        ]
        maximum_trusted_delta = EXPECTED_OUTAGE_YAW_GUARD_LIMITS[
            "max_trusted_delta_rad"
        ]

        if state == "DISARMED":
            if not math.isnan(trusted_variance):
                covariance_issues["outage_yaw_guard.trusted_variance_rad2.clear"] = {
                    "expected": "nan",
                    "actual": trusted_variance,
                }
        elif not (
            math.isfinite(trusted_variance)
            and 0.0 <= trusted_variance <= maximum_trusted_variance
        ):
            covariance_issues["outage_yaw_guard.trusted_variance_rad2.range"] = {
                "expected": f"[0,{maximum_trusted_variance}]",
                "actual": trusted_variance,
            }

        if state in OUTAGE_YAW_GUARD_ACTIVE_STATES:
            if not (
                math.isfinite(active_reference_variance)
                and 0.0
                <= active_reference_variance
                <= maximum_trusted_variance
            ):
                covariance_issues[
                    "outage_yaw_guard.active_reference_variance_rad2.range"
                ] = {
                    "expected": f"[0,{maximum_trusted_variance}]",
                    "actual": active_reference_variance,
                }
            else:
                active_reference_variances.append(active_reference_variance)
        elif state in {"DISARMED", "READY"} and not math.isnan(
            active_reference_variance
        ):
            covariance_issues[
                "outage_yaw_guard.active_reference_variance_rad2.clear"
            ] = {"expected": "nan", "actual": active_reference_variance}

        finite_correction = all(
            math.isfinite(value)
            for value in (applied_offset, target_offset, additional_variance)
        )
        if not finite_correction:
            covariance_issues["outage_yaw_guard.covariance_inputs"] = {
                "expected": "finite applied/target/additional values",
                "actual": [applied_offset, target_offset, additional_variance],
            }
        elif (
            additional_variance < 0.0
            or abs(applied_offset) > maximum_trusted_delta + 1.0e-12
            or abs(target_offset) > maximum_trusted_delta + 1.0e-12
        ):
            covariance_issues["outage_yaw_guard.covariance_input_range"] = {
                "applied": applied_offset,
                "target": target_offset,
                "additional_variance": additional_variance,
            }
        elif state in {"DISARMED", "READY"}:
            if any(
                abs(value) > 1.0e-12
                for value in (applied_offset, target_offset, additional_variance)
            ):
                covariance_issues["outage_yaw_guard.inactive_covariance"] = {
                    "expected": [0.0, 0.0, 0.0],
                    "actual": [applied_offset, target_offset, additional_variance],
                }
        elif (
            state in OUTAGE_YAW_GUARD_ACTIVE_STATES
            and math.isfinite(active_reference_variance)
        ):
            if state in OUTAGE_YAW_GUARD_OUTAGE_STATES:
                residual = (target_offset - applied_offset + math.pi) % (
                    2.0 * math.pi
                ) - math.pi
                expected_additional = active_reference_variance + residual * residual
            else:
                expected_additional = (
                    active_reference_variance + applied_offset * applied_offset
                )
            if abs(additional_variance - expected_additional) > 1.0e-9:
                covariance_issues["outage_yaw_guard.additional_variance_formula"] = {
                    "state": state,
                    "expected": expected_additional,
                    "actual": additional_variance,
                }

        outage_count = parsed_counters.get("outage_yaw_guard.outage_count")
        active_reference_epoch = parsed_counters.get(
            PRECISION_GUARD_ACTIVE_REFERENCE_EPOCH_KEY
        )
        recovery_count = parsed_counters.get("outage_yaw_guard.recovery_count")
        reset_count = parsed_counters.get("outage_yaw_guard.reset_count")
        if (
            outage_count is not None
            and recovery_count is not None
            and recovery_count > outage_count
        ):
            covariance_issues["outage_yaw_guard.edge_accounting"] = {
                "outage_count": outage_count,
                "recovery_count": recovery_count,
            }
        if (
            active_reference_epoch is not None
            and outage_count is not None
            and active_reference_epoch > outage_count
        ):
            covariance_issues[
                "outage_yaw_guard.active_reference_epoch.range"
            ] = {
                "maximum": outage_count,
                "actual": active_reference_epoch,
            }
        if active and active_reference_epoch == 0:
            covariance_issues[
                "outage_yaw_guard.active_reference_epoch.active"
            ] = {"expected": ">0", "actual": active_reference_epoch}
        if (
            state in OUTAGE_YAW_GUARD_OUTAGE_STATES
            and outage_count is not None
            and recovery_count is not None
            and outage_count <= recovery_count
        ):
            covariance_issues["outage_yaw_guard.open_outage_accounting"] = {
                "outage_count": outage_count,
                "recovery_count": recovery_count,
            }
        if (
            state == "RECOVERY_RELEASE"
            and outage_count is not None
            and recovery_count is not None
            and outage_count != recovery_count
        ):
            covariance_issues["outage_yaw_guard.release_edge_accounting"] = {
                "outage_count": outage_count,
                "recovery_count": recovery_count,
            }

        current_guard = {
            "state": state,
            "active": active,
            "active_reference_variance_rad2": active_reference_variance,
            "trusted_variance_rad2": trusted_variance,
            "outage_count": outage_count,
            "active_reference_epoch": active_reference_epoch,
            "recovery_count": recovery_count,
            "reset_count": reset_count,
            "last_reason": last_reason,
        }
        if (
            previous_guard is not None
            and previous_guard["active"]
            and active
            and reset_count is not None
            and reset_count == previous_guard["reset_count"]
            and outage_count is not None
            and math.isfinite(active_reference_variance)
            and math.isfinite(
                previous_guard["active_reference_variance_rad2"]
            )
        ):
            previous_active_variance = previous_guard[
                "active_reference_variance_rad2"
            ]
            previous_outage_count = previous_guard["outage_count"]
            previous_recovery_count = previous_guard["recovery_count"]
            previous_epoch = previous_guard["active_reference_epoch"]
            epoch_delta = (
                active_reference_epoch - previous_epoch
                if active_reference_epoch is not None
                and previous_epoch is not None
                else None
            )
            recovery_delta = (
                recovery_count - previous_recovery_count
                if recovery_count is not None
                and previous_recovery_count is not None
                else None
            )
            previous_outage = (
                previous_guard["state"] in OUTAGE_YAW_GUARD_OUTAGE_STATES
            )
            current_outage = state in OUTAGE_YAW_GUARD_OUTAGE_STATES
            if recovery_delta is not None:
                outage_delta = outage_count - previous_outage_count
                expected_outage_delta = (
                    recovery_delta
                    + int(current_outage)
                    - int(previous_outage)
                )
                if outage_delta != expected_outage_delta:
                    covariance_issues[
                        "outage_yaw_guard.outage_recovery_endpoint_balance"
                    ] = {
                        "outage_count_delta": outage_delta,
                        "recovery_count_delta": recovery_delta,
                        "previous_state": previous_guard["state"],
                        "state": state,
                        "expected_outage_count_delta": expected_outage_delta,
                    }
                if epoch_delta is not None and not 0 <= epoch_delta <= outage_delta:
                    covariance_issues[
                        "outage_yaw_guard.active_reference_epoch.delta"
                    ] = {
                        "expected": f"[0,{outage_delta}]",
                        "actual": epoch_delta,
                    }
            if outage_count == previous_outage_count:
                if abs(
                    active_reference_variance - previous_active_variance
                ) > 1.0e-12:
                    covariance_issues[
                        "outage_yaw_guard.active_reference_snapshot"
                    ] = {
                        "expected": previous_active_variance,
                        "actual": active_reference_variance,
                        "outage_count": outage_count,
                    }
            elif (
                (epoch_delta is None or epoch_delta == 0)
                and previous_guard["state"] == "RECOVERY_RELEASE"
                and state in OUTAGE_YAW_GUARD_OUTAGE_STATES
                and outage_count - previous_outage_count == 1
                and recovery_delta == 0
                and last_reason in OUTAGE_YAW_GUARD_REOUTAGE_REASONS
                and math.isfinite(trusted_variance)
            ):
                if last_reason == OUTAGE_YAW_GUARD_REOUTAGE_FRESH_REASON:
                    expected_active = max(
                        previous_active_variance, trusted_variance
                    )
                elif last_reason in OUTAGE_YAW_GUARD_REOUTAGE_RETAIN_REASONS:
                    expected_active = previous_active_variance
                else:
                    covariance_issues[
                        "outage_yaw_guard.reoutage_reason"
                    ] = {
                        "expected": sorted(OUTAGE_YAW_GUARD_REOUTAGE_REASONS),
                        "actual": last_reason,
                    }
                    expected_active = None
                if (
                    expected_active is not None
                    and abs(active_reference_variance - expected_active) > 1.0e-12
                ):
                    covariance_issues[
                        "outage_yaw_guard.reoutage_reference_snapshot"
                    ] = {
                        "reason": last_reason,
                        "expected": expected_active,
                        "actual": active_reference_variance,
                    }
            elif outage_count > previous_outage_count and epoch_delta == 0:
                # Same-epoch re-outage branches retain or increase the active
                # snapshot uncertainty.
                if active_reference_variance + 1.0e-12 < (
                    previous_active_variance
                ):
                    covariance_issues[
                        "outage_yaw_guard.hidden_reoutage_reference_snapshot"
                    ] = {
                        "minimum": previous_active_variance,
                        "actual": active_reference_variance,
                        "outage_count_delta": (
                            outage_count - previous_outage_count
                        ),
                    }
            elif outage_count > previous_outage_count and epoch_delta is not None:
                cross_epoch_active_reference_intervals += 1
            elif outage_count > previous_outage_count:
                # Legacy diagnostics cannot distinguish a same-epoch re-entry
                # from release-complete -> READY -> a new snapshot.
                unproven_active_reference_intervals += 1
        previous_guard = current_guard

        if covariance_issues:
            covariance_issue_count += 1
            issues.update(covariance_issues)

        if issues:
            mismatch_count += 1
            if len(mismatch_examples) < 20:
                mismatch_examples.append(
                    {
                        "index": index,
                        "stamp_ns": snapshot.get("stamp_ns"),
                        "issues": issues,
                    }
                )

    order = nondecreasing_physical_time(
        snapshot.get("stamp_ns", 0) for snapshot in snapshots
    )
    accepted_reference_observed = (
        bool(accepted_counts) and max(accepted_counts) > 0
    )
    valid = (
        bool(snapshots)
        and mismatch_count == 0
        and order["valid"]
        and accepted_reference_observed
    )
    return {
        "valid": valid,
        "snapshot_count": len(snapshots),
        "configuration_or_counter_mismatch_count": mismatch_count,
        "mismatch_examples": mismatch_examples,
        "accepted_reference_observed": accepted_reference_observed,
        "accepted_reference_final": (
            accepted_counts[-1] if accepted_counts else None
        ),
        "accepted_reference_maximum": max(accepted_counts, default=None),
        "invalid_advance_maximum": max(invalid_advance_counts, default=None),
        "suppressed_invalid_maximum": max(
            suppressed_invalid_counts, default=None
        ),
        "covariance_issue_sample_count": covariance_issue_count,
        "active_reference_epoch_accounting": {
            "available": epoch_accounting_available,
            "mixed_presence": epoch_accounting_mixed,
            "unproven_intervals": unproven_active_reference_intervals,
            "cross_epoch_intervals": cross_epoch_active_reference_intervals,
        },
        "maximum_active_reference_variance_rad2": max(
            active_reference_variances, default=None
        ),
        "stamp_order": order,
    }


def map_fusion_publication_integrity_contract(
    raw_odom_events: list[dict[str, Any]],
    existing_global_odom_events: list[dict[str, Any]],
    snapshots: list[dict[str, Any]],
) -> dict[str, Any]:
    """Audit serialized publication ordering and exact source-stamp coverage.

    The event ``record_index`` is the sequential bag-read position.  It is used
    instead of recorder timestamps to bound the causal audit: every raw odometry
    request observed after the first existing-global output and no later than
    the final map-fusion diagnostic must have an existing-global output with the
    exact same source timestamp inside that same publication window.
    """
    mismatch_count = 0
    mismatch_examples: list[dict[str, Any]] = []
    parsed_series = {
        key: [] for key in MAP_FUSION_PUBLICATION_COUNTER_KEYS
    }
    previous: dict[str, int] = {}

    for index, snapshot in enumerate(snapshots):
        issues: dict[str, Any] = {}
        values = snapshot.get("values", {})
        key_counts = snapshot.get("key_counts", {})
        parsed: dict[str, int] = {}
        for key in MAP_FUSION_PUBLICATION_COUNTER_KEYS:
            occurrence_count = key_counts.get(key)
            if occurrence_count != 1:
                issues[f"{key}.occurrence_count"] = {
                    "expected": 1,
                    "actual": occurrence_count,
                }
            actual = values.get(key)
            try:
                numeric = _strict_diagnostic_counter_integer(actual)
            except (TypeError, ValueError):
                issues[key] = {
                    "expected": "canonical non-negative integer",
                    "actual": actual,
                }
                continue
            parsed[key] = numeric
            parsed_series[key].append(numeric)
            prior = previous.get(key)
            if prior is not None and numeric < prior:
                issues[f"{key}.monotonic"] = {
                    "previous": prior,
                    "actual": numeric,
                }
            previous[key] = numeric

        drop_key, covered_key, wall_key, total_key = (
            MAP_FUSION_PUBLICATION_COUNTER_KEYS
        )
        if drop_key in parsed and parsed[drop_key] != 0:
            issues[f"{drop_key}.zero"] = {
                "expected": 0,
                "actual": parsed[drop_key],
            }
        if all(key in parsed for key in MAP_FUSION_PUBLICATION_COUNTER_KEYS):
            component_sum = (
                parsed[drop_key] + parsed[covered_key] + parsed[wall_key]
            )
            if parsed[total_key] != component_sum:
                issues[f"{total_key}.identity"] = {
                    "expected": component_sum,
                    "actual": parsed[total_key],
                }

        if issues:
            mismatch_count += 1
            if len(mismatch_examples) < 20:
                mismatch_examples.append(
                    {
                        "index": index,
                        "record_index": snapshot.get("record_index"),
                        "stamp_ns": snapshot.get("stamp_ns"),
                        "issues": issues,
                    }
                )

    diagnostic_stamp_order = nondecreasing_physical_time(
        snapshot.get("stamp_ns", 0) for snapshot in snapshots
    )

    def record_order_valid(events: list[dict[str, Any]]) -> bool:
        indices = [event.get("record_index") for event in events]
        record_stamps = [
            event.get("record_stamp_ns", event.get("record_index"))
            for event in events
        ]
        return bool(events) and all(
            isinstance(value, int) and value >= 0 for value in indices
        ) and all(
            later > earlier for earlier, later in zip(indices, indices[1:])
        ) and all(
            isinstance(value, int) and value >= 0 for value in record_stamps
        ) and all(
            later >= earlier
            for earlier, later in zip(record_stamps, record_stamps[1:])
        )

    def canonical_stamps(events: list[dict[str, Any]]) -> bool:
        return bool(events) and all(
            isinstance(event.get("stamp_ns"), int)
            and event["stamp_ns"] >= 0
            and event.get("stamp_canonical", True) is True
            for event in events
        )

    diagnostic_record_order_valid = record_order_valid(snapshots)
    raw_record_order_valid = record_order_valid(raw_odom_events)
    existing_record_order_valid = record_order_valid(
        existing_global_odom_events
    )
    raw_stamp_canonical = canonical_stamps(raw_odom_events)
    existing_stamp_canonical = canonical_stamps(existing_global_odom_events)
    final_diagnostic_record_index = (
        snapshots[-1].get("record_index")
        if diagnostic_record_order_valid else None
    )
    existing_prefix = [
        event for event in existing_global_odom_events
        if isinstance(final_diagnostic_record_index, int)
        and event.get("record_index", final_diagnostic_record_index + 1)
        <= final_diagnostic_record_index
    ]
    existing_prefix_stamps = [event["stamp_ns"] for event in existing_prefix]
    first_existing_positive = next(
        (event for event in existing_prefix if event["stamp_ns"] > 0), None
    )
    first_positive_index = next(
        (
            index for index, stamp in enumerate(existing_prefix_stamps)
            if stamp > 0
        ),
        None,
    )
    existing_positive_stamps = (
        existing_prefix_stamps[first_positive_index:]
        if first_positive_index is not None else []
    )
    existing_positive_stamp_order_valid = (
        bool(existing_positive_stamps)
        and all(stamp > 0 for stamp in existing_positive_stamps)
        and all(
            later >= earlier
            for earlier, later in zip(
                existing_positive_stamps, existing_positive_stamps[1:]
            )
        )
    )
    first_existing_record_index = (
        first_existing_positive.get("record_index")
        if first_existing_positive is not None else None
    )
    causal_window_valid = (
        isinstance(first_existing_record_index, int)
        and isinstance(final_diagnostic_record_index, int)
        and final_diagnostic_record_index > first_existing_record_index
    )

    raw_in_window: list[dict[str, Any]] = []
    existing_in_window: list[dict[str, Any]] = []
    if causal_window_valid:
        raw_in_window = [
            event for event in raw_odom_events
            if first_existing_record_index < event["record_index"] <=
            final_diagnostic_record_index and event["stamp_ns"] > 0
        ]
        existing_in_window = [
            event for event in existing_prefix
            if first_existing_record_index <= event["record_index"] <=
            final_diagnostic_record_index and event["stamp_ns"] > 0
        ]

    existing_stamps = {event["stamp_ns"] for event in existing_in_window}
    raw_unique_stamps = {event["stamp_ns"] for event in raw_in_window}
    missing_raw_stamps = sorted(
        stamp for stamp in raw_unique_stamps if stamp not in existing_stamps
    )
    raw_source_stamp_monotonic_report_only = all(
        later >= earlier
        for earlier, later in zip(
            [event["stamp_ns"] for event in raw_in_window],
            [event["stamp_ns"] for event in raw_in_window][1:],
        )
    )
    raw_zero_stamp_excluded = sum(
        event["stamp_ns"] == 0
        and causal_window_valid
        and first_existing_record_index < event["record_index"] <=
        final_diagnostic_record_index
        for event in raw_odom_events
    )
    existing_unique_stamps = {
        event["stamp_ns"] for event in existing_in_window
    }
    matched_raw_unique_stamps = raw_unique_stamps & existing_stamps
    causal_coverage_valid = (
        raw_record_order_valid
        and existing_record_order_valid
        and raw_stamp_canonical
        and existing_stamp_canonical
        and existing_positive_stamp_order_valid
        and causal_window_valid
        and bool(raw_unique_stamps)
        and not missing_raw_stamps
    )

    final_counters = {
        key: values[-1] if values else None
        for key, values in parsed_series.items()
    }
    drop_key, covered_key, wall_key, total_key = (
        MAP_FUSION_PUBLICATION_COUNTER_KEYS
    )
    valid = (
        bool(snapshots)
        and mismatch_count == 0
        and diagnostic_stamp_order["valid"]
        and diagnostic_record_order_valid
        and causal_coverage_valid
    )
    return {
        "valid": valid,
        "snapshot_count": len(snapshots),
        "counter_mismatch_count": mismatch_count,
        "counter_mismatch_examples": mismatch_examples,
        "diagnostic_stamp_order": diagnostic_stamp_order,
        "diagnostic_record_order_valid": diagnostic_record_order_valid,
        "final_counters": final_counters,
        "strict_drop_final": final_counters[drop_key],
        # Coalescing is expected under callback/timer races and is report-only.
        "covered_odometry_coalesced_final": final_counters[covered_key],
        "wall_timer_coalesced_final": final_counters[wall_key],
        "total_suppressed_request_final": final_counters[total_key],
        "causal_raw_stamp_coverage": {
            "valid": causal_coverage_valid,
            "raw_record_order_valid": raw_record_order_valid,
            # DDS/recorder interleave can reorder raw source stamps.  Preserve
            # this observation for review, but do not make it a publication
            # failure; exact set coverage below is the causal contract.
            "raw_source_stamp_monotonic_report_only": (
                raw_source_stamp_monotonic_report_only
            ),
            "existing_record_order_valid": existing_record_order_valid,
            "existing_positive_stamp_order_valid": (
                existing_positive_stamp_order_valid
            ),
            "raw_stamp_canonical": raw_stamp_canonical,
            "existing_stamp_canonical": existing_stamp_canonical,
            "first_existing_record_index": first_existing_record_index,
            "final_diagnostic_record_index": final_diagnostic_record_index,
            "raw_request_count": len(raw_in_window),
            "raw_unique_stamp_count": len(raw_unique_stamps),
            "raw_duplicate_request_count": (
                len(raw_in_window) - len(raw_unique_stamps)
            ),
            "raw_zero_stamp_excluded_count": raw_zero_stamp_excluded,
            "existing_output_count": len(existing_in_window),
            "existing_unique_stamp_count": len(existing_unique_stamps),
            "exactly_covered_raw_unique_stamp_count": (
                len(matched_raw_unique_stamps)
            ),
            "missing_raw_unique_stamp_count": len(missing_raw_stamps),
            "missing_raw_stamp_examples_ns": missing_raw_stamps[:20],
        },
    }


def counter_contract(
    counters: dict[str, dict[str, Any]], mode: str
) -> dict[str, Any]:
    violations = {
        identity: summary
        for identity, summary in counters.items()
        if summary["malformed_values"] or int(summary["maximum"]) != 0
    }
    required: list[tuple[str, str]] = []
    if mode == "precision":
        required = [
            (MATCHER_STATUS, "malformed_count"),
            (PRECISION_GLOBAL_STATUS, "raw.backstep"),
            (PRECISION_GLOBAL_STATUS, "raw.nonmonotonic"),
        ]
    missing_required = []
    for status_name, key in required:
        if not any(
            _status_matches(summary["status"], status_name)
            and summary["key"] == key
            for summary in counters.values()
        ):
            missing_required.append(f"{status_name}:{key}")
    return {
        "valid": not violations and not missing_required,
        "available_counter_count": len(counters),
        "counters": counters,
        "violations": violations,
        "missing_required": missing_required,
    }


def validate_bags(
    run_dir: Path, input_bag: Path, manifest: dict[str, Any], mode: str,
    checks: Checks,
) -> dict[str, Any]:
    input_metadata, input_counts, input_types = bag_metadata(input_bag)
    expected_input_types = {
        manifest["topics"]["pointcloud"]: "sensor_msgs/msg/PointCloud2",
        manifest["topics"]["imu"]: "sensor_msgs/msg/Imu",
        manifest["topics"]["nmea"]: "nmea_msgs/msg/Sentence",
    }
    input_topic_errors = {
        topic: {
            "expected_type": expected_type,
            "actual_type": input_types.get(topic),
            "count": input_counts.get(topic, 0),
        }
        for topic, expected_type in expected_input_types.items()
        if input_types.get(topic) != expected_type or input_counts.get(topic, 0) <= 0
    }
    input_duration_sec = input_metadata.duration.nanoseconds * 1.0e-9
    expected_duration_sec = float(manifest["duration_sec"])
    checks.add(
        "private input bag topic contract",
        not input_topic_errors,
        "all manifest topics present" if not input_topic_errors else
        json.dumps(input_topic_errors, sort_keys=True),
    )
    checks.add(
        "private input bag duration matches manifest",
        abs(input_duration_sec - expected_duration_sec) <= 0.5,
        f"manifest={expected_duration_sec:.6f}s bag={input_duration_sec:.6f}s",
    )

    output_bag = run_dir / "localization_output"
    output_metadata, output_counts, output_types = bag_metadata(output_bag)
    expected_output_types = dict(COMMON_OUTPUT_TOPICS)
    if mode in ("control", "precision"):
        expected_output_types[SNAPSHOT_TOPIC] = PRECISION_OUTPUT_TOPICS[SNAPSHOT_TOPIC]
    if mode == "precision":
        expected_output_types.update(PRECISION_OUTPUT_TOPICS)
    output_topic_errors = {
        topic: {
            "expected_type": expected_type,
            "actual_type": output_types.get(topic),
            "count": output_counts.get(topic, 0),
        }
        for topic, expected_type in expected_output_types.items()
        if output_types.get(topic) != expected_type or output_counts.get(topic, 0) <= 0
    }
    precision_only = set(PRECISION_OUTPUT_TOPICS) - {SNAPSHOT_TOPIC}
    forbidden = set()
    if mode == "baseline":
        forbidden.add(SNAPSHOT_TOPIC)
        forbidden.update(precision_only)
    elif mode == "control":
        forbidden.update(precision_only)
    forbidden_present = sorted(topic for topic in forbidden if output_counts.get(topic, 0) > 0)
    checks.add(
        "mode-specific output topic contract",
        not output_topic_errors and not forbidden_present,
        (
            f"required={len(expected_output_types)} forbidden topics absent"
            if not output_topic_errors and not forbidden_present else
            json.dumps(
                {"required_errors": output_topic_errors, "forbidden_present": forbidden_present},
                sort_keys=True,
            )
        ),
    )

    runtime = scan_output_bag(output_bag)
    checks.add(
        "output bag metadata message accounting",
        runtime["scanned_message_count"] == int(output_metadata.message_count),
        f"metadata={output_metadata.message_count} scanned={runtime['scanned_message_count']}",
    )
    clock = nondecreasing_physical_time(runtime["clock_values"])
    input_start_ns = int(input_metadata.starting_time.nanoseconds)
    input_end_ns = input_start_ns + int(input_metadata.duration.nanoseconds)
    clock_endpoints_valid = (
        clock["first_positive_ns"] is not None
        and clock["last_positive_ns"] is not None
        and abs(int(clock["first_positive_ns"]) - input_start_ns) <= 250_000_000
        and abs(int(clock["last_positive_ns"]) - input_end_ns) <= 250_000_000
    )
    clock_span = clock["span_sec"] if clock["span_sec"] is not None else math.nan
    full_clock_valid = (
        clock["valid"]
        and math.isfinite(clock_span)
        and expected_duration_sec * 0.98 <= clock_span <= expected_duration_sec + 1.0
        and clock_endpoints_valid
    )
    checks.add(
        "full monotonic /clock playback",
        full_clock_valid,
        f"span={clock_span:.6f}s expected={expected_duration_sec:.6f}s "
        f"first={clock['first_positive_ns']} last={clock['last_positive_ns']}",
    )
    output_duration_sec = output_metadata.duration.nanoseconds * 1.0e-9
    checks.add(
        "real-time output recording duration",
        expected_duration_sec * 0.98 <= output_duration_sec <= expected_duration_sec + 30.0,
        f"recording={output_duration_sec:.6f}s expected_input={expected_duration_sec:.6f}s",
    )

    required_statuses = [NMEA_STATUS, FUSION_STATUS, ODOMETER_STATUS]
    if mode == "precision":
        required_statuses.extend([MATCHER_STATUS, PRECISION_GLOBAL_STATUS])
    status_coverage: dict[str, Any] = {}
    status_failures = []
    for expected_status in required_statuses:
        matching_names = [
            name for name in runtime["status_counts"]
            if _status_matches(name, expected_status)
        ]
        stamps = [
            stamp
            for name in matching_names
            for stamp in runtime["status_stamps"].get(name, [])
        ]
        order = nondecreasing_physical_time(stamps)
        span = order["span_sec"] if order["span_sec"] is not None else math.nan
        valid = (
            len(matching_names) == 1
            and order["valid"]
            and math.isfinite(span)
            and span >= expected_duration_sec * 0.95
        )
        status_coverage[expected_status] = {
            "matching_names": matching_names,
            "count": len(stamps),
            "stamp_order": order,
            "valid": valid,
        }
        if not valid:
            status_failures.append(expected_status)
    checks.add(
        "required diagnostic status full-run coverage",
        not status_failures,
        "all required statuses cover the run" if not status_failures else
        f"failed={status_failures}",
    )
    checks.add(
        "diagnostic status and key uniqueness",
        runtime["duplicate_key_samples"] == 0
        and runtime["duplicate_status_samples"] == 0,
        f"duplicate_key_samples={runtime['duplicate_key_samples']} "
        f"duplicate_status_samples={runtime['duplicate_status_samples']}",
    )

    fusion_authority_stream = fusion_authority_stream_contract(
        runtime["fusion_authority_events"]
    )
    checks.add(
        "typed fusion-authority stream contract",
        fusion_authority_stream["valid"],
        f"events={fusion_authority_stream['event_count']} "
        f"sessions={fusion_authority_stream['session_count']} "
        f"states={fusion_authority_stream['state_counts']} "
        f"soft_bad_hold={fusion_authority_stream['soft_bad_hold_count']} "
        f"violations={fusion_authority_stream['violation_count']}",
    )

    publication_integrity = map_fusion_publication_integrity_contract(
        runtime["raw_odom_events"],
        runtime["existing_global_odom_events"],
        runtime["map_fusion_publication_snapshots"],
    )
    causal_coverage = publication_integrity["causal_raw_stamp_coverage"]
    checks.add(
        "map-fusion publication integrity and causal raw-stamp coverage",
        publication_integrity["valid"],
        "snapshots="
        f"{publication_integrity['snapshot_count']} "
        "counter_mismatches="
        f"{publication_integrity['counter_mismatch_count']} "
        f"strict_drop={publication_integrity['strict_drop_final']} "
        "covered_odometry_coalesced="
        f"{publication_integrity['covered_odometry_coalesced_final']} "
        "wall_timer_coalesced="
        f"{publication_integrity['wall_timer_coalesced_final']} "
        "raw_unique_exact_coverage="
        f"{causal_coverage['exactly_covered_raw_unique_stamp_count']}/"
        f"{causal_coverage['raw_unique_stamp_count']} "
        f"missing={causal_coverage['missing_raw_unique_stamp_count']}",
    )

    nmea_summary = nmea_diagnostic_contract(runtime["nmea_snapshots"])
    nmea_span = nmea_summary["stamp_order"]["span_sec"]
    nmea_full_run = (
        nmea_summary["valid"]
        and nmea_span is not None
        and nmea_span >= expected_duration_sec * 0.95
    )
    checks.add(
        "default NMEA projection in every runtime snapshot",
        nmea_full_run,
        f"snapshots={nmea_summary['snapshot_count']} span={nmea_span} "
        f"mismatches={nmea_summary['configuration_mismatch_count']}",
    )
    precision_guard_summary = None
    if mode == "precision":
        precision_guard_summary = precision_guard_diagnostic_contract(
            runtime["precision_guard_snapshots"]
        )
        guard_span = precision_guard_summary["stamp_order"]["span_sec"]
        precision_guard_full_run = (
            precision_guard_summary["valid"]
            and guard_span is not None
            and guard_span >= expected_duration_sec * 0.95
        )
        checks.add(
            "precision outage-yaw guard runtime publication contract",
            precision_guard_full_run,
            f"snapshots={precision_guard_summary['snapshot_count']} "
            f"span={guard_span} "
            "mismatches="
            f"{precision_guard_summary['configuration_or_counter_mismatch_count']} "
            "accepted_reference_final="
            f"{precision_guard_summary['accepted_reference_final']} "
            "invalid_advance_maximum="
            f"{precision_guard_summary['invalid_advance_maximum']} "
            "suppressed_invalid_maximum="
            f"{precision_guard_summary['suppressed_invalid_maximum']}",
        )
        epoch_accounting = precision_guard_summary[
            "active_reference_epoch_accounting"
        ]
        epoch_available = bool(epoch_accounting["available"])
        checks.add(
            "precision outage-yaw guard active-reference epoch accounting",
            epoch_available,
            (
                f"accounting={epoch_accounting}"
                if epoch_available
                else "N/A: legacy diagnostics leave hidden re-outage snapshot "
                f"lineage unproven; accounting={epoch_accounting}"
            ),
            warning=not epoch_available,
        )
    counters = counter_contract(runtime["audited_counters"], mode)
    checks.add(
        "malformed and backstep diagnostic counters",
        counters["valid"],
        f"available={counters['available_counter_count']} "
        f"violations={sorted(counters['violations'])} "
        f"missing_required={counters['missing_required']}",
    )
    return {
        "input": {
            "path": str(input_bag),
            "duration_sec": input_duration_sec,
            "message_count": int(input_metadata.message_count),
            "topic_counts": input_counts,
        },
        "output": {
            "path": str(output_bag),
            "duration_sec": output_duration_sec,
            "message_count": int(output_metadata.message_count),
            "topic_counts": output_counts,
            "clock": clock,
        },
        "diagnostics": {
            "status_coverage": status_coverage,
            "nmea_projection": nmea_summary,
            "fusion_authority_stream": fusion_authority_stream,
            "map_fusion_publication_integrity": publication_integrity,
            "precision_outage_yaw_guard": precision_guard_summary,
            "audited_zero_counters": counters,
        },
    }


def validate(repo: Path, run_dir: Path, dataset: str, mode: str) -> dict[str, Any]:
    repo = repo.expanduser().resolve()
    run_dir = run_dir.expanduser().resolve()
    checks = Checks()
    run_env_path = run_dir / "run.env"
    tsv_path = run_dir / "artifacts/effective_configurations.tsv"
    if not run_env_path.is_file():
        raise RuntimeError(f"run.env is missing: {run_env_path}")
    if not tsv_path.is_file():
        raise RuntimeError(f"effective configuration table is missing: {tsv_path}")
    env = parse_run_env_text(run_env_path.read_text(encoding="utf-8"))
    rows = parse_effective_configurations(tsv_path.read_text(encoding="utf-8"))
    contracts = config_contracts(dataset, mode)
    manifest_contract = next(item for item in contracts if item.role == "dataset_manifest")
    manifest_path = repo / manifest_contract.source_relative
    manifest = parse_dataset_manifest_text(manifest_path.read_text(encoding="utf-8"))

    environment = validate_run_environment(env, repo, dataset, mode, manifest, checks)
    provenance = validate_config_provenance(
        repo, run_dir, env, rows, contracts, checks
    )

    nmea_parameter_path = repo / next(
        item.source_relative for item in contracts if item.role == "nmea_base"
    )
    nmea_parameter_semantic_values = nmea_parameter_semantics(
        nmea_parameter_path.read_text(encoding="utf-8")
    )
    checks.add(
        "canonical default NMEA parameter semantics",
        nmea_parameter_semantic_values["valid"],
        json.dumps(nmea_parameter_semantic_values, sort_keys=True),
    )
    projector_metadata_path = repo / next(
        item.source_relative
        for item in contracts if item.role == "nmea_projector_metadata"
    )
    projector_metadata_semantics = map_projector_metadata_semantics(
        projector_metadata_path.read_text(encoding="utf-8")
    )
    checks.add(
        "canonical map projector metadata semantics",
        projector_metadata_semantics["valid"],
        json.dumps(projector_metadata_semantics, sort_keys=True),
    )
    projection_agreement = projection_semantics_agree(
        nmea_parameter_semantic_values, projector_metadata_semantics
    )
    checks.add(
        "loaded NMEA parameters match map projector metadata",
        projection_agreement,
        (
            "projector, datum, origin, and scale agree"
            if projection_agreement else
            json.dumps(
                {
                    "nmea_parameters": nmea_parameter_semantic_values,
                    "map_projector_metadata": projector_metadata_semantics,
                },
                sort_keys=True,
            )
        ),
    )
    nmea_override_path = repo / next(
        item.source_relative for item in contracts if item.role == "nmea_override"
    )
    nmea_override_semantic_values = empty_override_semantics(
        nmea_override_path.read_text(encoding="utf-8")
    )
    checks.add(
        "canonical NMEA evaluation override is empty",
        nmea_override_semantic_values["valid"],
        json.dumps(nmea_override_semantic_values, sort_keys=True),
    )
    fusion_path = repo / next(
        item.source_relative for item in contracts if item.role == "gnss_fusion_override"
    )
    fusion_semantics = fusion_override_semantics(
        fusion_path.read_text(encoding="utf-8")
    )
    checks.add(
        "canonical single-antenna recovery override semantics",
        fusion_semantics["valid"],
        json.dumps(fusion_semantics, sort_keys=True),
    )

    precision_global_semantics = None
    if mode == "precision":
        precision_global_contract = next(
            item for item in contracts if item.role == "precision_global_base"
        )
        precision_global_effective_path = (
            run_dir / "artifacts" / precision_global_contract.artifact
        )
        precision_global_semantics = precision_global_parameter_semantics(
            precision_global_effective_path.read_text(encoding="utf-8")
        )
        checks.add(
            "effective precision global outage-yaw guard semantics",
            precision_global_semantics["valid"],
            json.dumps(precision_global_semantics, sort_keys=True),
        )

    input_bag_text = environment["input_bag"]
    if input_bag_text is None:
        raise RuntimeError("run.env has no input bag")
    bags = validate_bags(
        run_dir, Path(input_bag_text), manifest, mode, checks
    )
    dataset_id, display_name = dataset_identity(dataset)
    return {
        "schema_version": 1,
        "validator": "hesai_gnss_publication_provenance",
        "dataset": {
            "key": dataset,
            "id": dataset_id,
            "display_name": display_name,
            "manifest": str(manifest_path),
            "expected_duration_sec": manifest["duration_sec"],
        },
        "expected_mode": mode,
        "run_directory": str(run_dir),
        "environment": environment,
        "configuration_provenance": provenance,
        "configuration_semantics": {
            "nmea_projection": {
                "parameters": nmea_parameter_semantic_values,
                "map_projector_metadata": projector_metadata_semantics,
                "parameters_match_metadata": projection_agreement,
                "evaluation_override": nmea_override_semantic_values,
            },
            "gnss_fusion_single_antenna": fusion_semantics,
            "precision_global_outage_yaw_guard": precision_global_semantics,
        },
        "bags": bags,
        "checks": checks.items,
        "summary": {
            "passed": checks.passed,
            "check_count": len(checks.items),
            "passed_check_count": sum(
                item["passed"] for item in checks.items
            ),
            "warning_check_count": sum(
                not item["passed"] and item["warning"]
                for item in checks.items
            ),
            "failed_check_count": sum(
                not item["passed"] and not item["warning"]
                for item in checks.items
            ),
        },
    }


def failure_result(
    repo: Path | None, run_dir: Path | None, dataset: str | None,
    mode: str | None, error: Exception,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "validator": "hesai_gnss_publication_provenance",
        "dataset": {"key": dataset},
        "expected_mode": mode,
        "run_directory": str(run_dir) if run_dir is not None else None,
        "repository": str(repo) if repo is not None else None,
        "checks": [
            {
                "name": "validator completed without fatal input error",
                "passed": False,
                "detail": f"{type(error).__name__}: {error}",
            }
        ],
        "summary": {
            "passed": False,
            "check_count": 1,
            "passed_check_count": 0,
            "warning_check_count": 0,
            "failed_check_count": 1,
        },
    }


def _markdown_escape(value: Any) -> str:
    return str(value).replace("|", "\\|").replace("\n", " ")


def markdown(result: dict[str, Any]) -> str:
    summary = result["summary"]
    dataset = result.get("dataset", {})
    status = "PASS" if summary["passed"] else "FAIL"
    warning_count = summary.get(
        "warning_check_count",
        sum(
            not item["passed"] and item.get("warning", False)
            for item in result["checks"]
        ),
    )
    passed_count = summary.get(
        "passed_check_count",
        summary["check_count"] - summary["failed_check_count"] - warning_count,
    )
    lines = [
        "# Hesai GNSS publication provenance",
        "",
        f"**Status: {status}**",
        "",
        f"- Dataset: {_markdown_escape(dataset.get('display_name') or dataset.get('key'))}",
        f"- Expected mode: `{_markdown_escape(result.get('expected_mode'))}`",
        f"- Run directory: `{_markdown_escape(result.get('run_directory'))}`",
        f"- Checks: {passed_count} passed, {warning_count} warnings, "
        f"{summary['failed_check_count']} failed",
        "",
        "## Checks",
        "",
        "| Check | Result | Detail |",
        "|---|---:|---|",
    ]
    for check in result["checks"]:
        check_status = (
            "PASS"
            if check["passed"]
            else "WARN" if check.get("warning", False) else "FAIL"
        )
        lines.append(
            f"| {_markdown_escape(check['name'])} | "
            f"{check_status} | "
            f"{_markdown_escape(check['detail'])} |"
        )
    lines.append("")
    return "\n".join(lines)


def write_outputs(result: dict[str, Any], output_json: Path, output_markdown: Path) -> None:
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_markdown.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    output_markdown.write_text(markdown(result), encoding="utf-8")


def run_self_test() -> None:
    env = parse_run_env_text(
        "dataset=course-1\n"
        "dataset_display_name=Hesai\\ 32-Line\\ +\\ IMU\n"
        "empty=''\n"
    )
    assert env == {
        "dataset": "course-1",
        "dataset_display_name": "Hesai 32-Line + IMU",
        "empty": "",
    }
    try:
        parse_run_env_text("value=one two\n")
    except ValueError:
        pass
    else:
        raise AssertionError("multi-token run.env value was accepted")

    nmea = nmea_parameter_semantics(
        "/**:\n  ros__parameters:\n"
        "    projector_type: TransverseMercator\n"
        "    vertical_datum: WGS84\n"
        "    map_origin.latitude: 35.681236\n"
        "    map_origin.longitude: 139.767125\n"
        "    map_origin.altitude: 0.0\n"
        "    scale_factor: 0.9996\n"
        "    use_legacy_projection_params: false\n"
        "    gnss0:\n"
        "      - 35.681236\n"
        "      - 139.767125\n"
        "      - 0.0\n"
    )
    assert nmea["valid"]
    projector_metadata = map_projector_metadata_semantics(
        'projector_type: "TransverseMercator"\n'
        'vertical_datum: "WGS84"\n'
        "map_origin:\n"
        "  latitude: 35.681236\n"
        "  longitude: 139.767125\n"
        "scale_factor: 0.9996\n"
    )
    assert projector_metadata["valid"]
    assert projection_semantics_agree(nmea, projector_metadata)
    assert empty_override_semantics(
        "/**:\n  ros__parameters: {}\n"
    )["valid"]
    assert not empty_override_semantics(
        "/**:\n  ros__parameters:\n    map_origin.latitude: 35.0\n"
    )["valid"]
    assert fusion_override_semantics(
        "/**:\n  ros__parameters:\n    xy_only_recovery.enabled: true\n"
    )["valid"]

    precision_global = precision_global_parameter_semantics(
        "precision_global_localizer:\n  ros__parameters:\n"
        "    fallback.gnss_position_enabled: false\n"
        "    outage_yaw_guard.enabled: true\n"
        "    outage_yaw_guard.required_fix_quality: 4\n"
        "    outage_yaw_guard.max_trusted_age_sec: 2.0\n"
        "    outage_yaw_guard.max_trusted_variance_rad2: 0.0225\n"
        "    outage_yaw_guard.max_trusted_delta_rad: 0.35\n"
        "    outage_yaw_guard.max_offset_rate_radps: 0.20\n"
        "    outage_yaw_guard.max_offset_step_rad: 0.04\n"
        "    outage_yaw_guard.max_step_dt_sec: 0.25\n"
    )
    assert precision_global["valid"]

    snapshot = {
        "stamp_ns": 1,
        "duplicate_keys": [],
        "values": {
            "projector_type": EXPECTED_PROJECTOR,
            "vertical_datum": EXPECTED_VERTICAL_DATUM,
            "projection_mode": EXPECTED_PROJECTION_MODE,
            "map_origin_latitude": "35.68123600",
            "map_origin_longitude": "139.76712500",
            "scale_factor": "0.9996000",
            "has_last_primary": "true",
            "has_last_output": "true",
            "last_parse_error": "none",
        },
    }
    assert nmea_diagnostic_contract([snapshot])["valid"]
    bad = json.loads(json.dumps(snapshot))
    bad["values"]["map_origin_latitude"] = "35.1254574925"
    assert not nmea_diagnostic_contract([bad])["valid"]

    def authority_event(
        record_index: int, session: int, sequence: int, stamp: int,
        state: int = 1,
    ) -> dict[str, Any]:
        soft_hold = state == 2
        return {
            "record_index": record_index,
            "record_stamp_ns": stamp,
            "stamp_ns": stamp,
            "stamp_canonical": True,
            "source_stamp_ns": stamp,
            "source_stamp_canonical": True,
            "frame_id": "map",
            "session_id": session,
            "sequence": sequence,
            "state": state,
            "recovery_state": "tracking",
            "anchor_valid": True,
            "position_fused": True,
            "yaw_fused": True,
            "last_fix_state": 2 if soft_hold else 1,
            "reason": "gnss_soft_bad_within_grace" if soft_hold else "self_test",
        }

    authority_good = fusion_authority_stream_contract(
        [
            authority_event(1, 10, 1, 1_000_000_000),
            authority_event(2, 10, 2, 1_000_000_000, state=2),
            authority_event(3, 11, 1, 2_000_000_000),
        ]
    )
    assert authority_good["valid"]
    assert authority_good["soft_bad_hold_count"] == 1
    assert not fusion_authority_stream_contract(
        [
            authority_event(1, 10, 1, 1_000_000_000),
            authority_event(2, 10, 1, 1_100_000_000),
        ]
    )["valid"]
    assert not fusion_authority_stream_contract(
        [
            authority_event(1, 10, 1, 1_000_000_000),
            authority_event(2, 11, 1, 2_000_000_000),
            authority_event(3, 10, 2, 3_000_000_000),
        ]
    )["valid"]
    assert not fusion_authority_stream_contract(
        [
            authority_event(1, 10, 1, 1_000_000_000),
            authority_event(2, 11, 1, 1_000_000_000),
        ]
    )["valid"]
    inconsistent_full = authority_event(1, 10, 1, 1_000_000_000)
    inconsistent_full["yaw_fused"] = False
    assert not fusion_authority_stream_contract([inconsistent_full])["valid"]

    guard_values = {
        "fallback.gnss_position_enabled": "false",
        "outage_yaw_guard.enabled": "true",
        "outage_yaw_guard.state": "READY",
        "outage_yaw_guard.active": "false",
        "outage_yaw_guard.reference_source": (
            EXPECTED_OUTAGE_YAW_GUARD_REFERENCE_SOURCE
        ),
        "outage_yaw_guard.propagation_source": (
            EXPECTED_OUTAGE_YAW_GUARD_PROPAGATION_SOURCE
        ),
        "outage_yaw_guard.xy_policy": EXPECTED_OUTAGE_YAW_GUARD_XY_POLICY,
        "outage_yaw_guard.config.required_fix_quality": "4",
        **{
            f"outage_yaw_guard.config.{key}": str(value)
            for key, value in EXPECTED_OUTAGE_YAW_GUARD_LIMITS.items()
        },
        "outage_yaw_guard.trusted_variance_rad2": "0.01",
        "outage_yaw_guard.active_reference_variance_rad2": "nan",
        "outage_yaw_guard.applied_offset_rad": "0",
        "outage_yaw_guard.target_offset_rad": "0",
        "outage_yaw_guard.additional_variance_rad2": "0",
        "outage_yaw_guard.last_reason": "trusted_yaw_reference_ready",
        "outage_yaw_guard.accepted_reference_count": "1",
        "outage_yaw_guard.outage_count": "0",
        "outage_yaw_guard.active_reference_epoch": "0",
        "outage_yaw_guard.recovery_count": "0",
        "outage_yaw_guard.reset_count": "1",
        "outage_yaw_guard.invalid_advance_count": "0",
        "publish.global_suppressed_yaw_guard_invalid": "0",
    }
    guard_snapshot = {
        "stamp_ns": 1,
        "duplicate_keys": [],
        "key_counts": {key: 1 for key in guard_values},
        "values": guard_values,
    }
    guard_contract = precision_guard_diagnostic_contract([guard_snapshot])
    assert guard_contract["valid"]
    assert guard_contract["active_reference_epoch_accounting"]["available"]
    guard_values["outage_yaw_guard.invalid_advance_count"] = "1"
    assert not precision_guard_diagnostic_contract([guard_snapshot])["valid"]
    counters = {
        f"{MATCHER_STATUS}:malformed_count": {
            "status": MATCHER_STATUS,
            "key": "malformed_count",
            "samples": 1,
            "maximum": 0,
            "malformed_values": [],
        },
        f"{PRECISION_GLOBAL_STATUS}:raw.backstep": {
            "status": PRECISION_GLOBAL_STATUS,
            "key": "raw.backstep",
            "samples": 1,
            "maximum": 0,
            "malformed_values": [],
        },
        f"{PRECISION_GLOBAL_STATUS}:raw.nonmonotonic": {
            "status": PRECISION_GLOBAL_STATUS,
            "key": "raw.nonmonotonic",
            "samples": 1,
            "maximum": 0,
            "malformed_values": [],
        },
    }
    assert counter_contract(counters, "precision")["valid"]
    counters[f"{MATCHER_STATUS}:malformed_count"]["maximum"] = 1
    assert not counter_contract(counters, "precision")["valid"]

    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "payload"
        path.write_bytes(b"publication provenance\n")
        assert sha256_file(path) == hashlib.sha256(path.read_bytes()).hexdigest()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path)
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--dataset", choices=("course-1", "course-2"))
    parser.add_argument(
        "--expected-mode", choices=("baseline", "control", "precision")
    )
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-markdown", type=Path)
    parser.add_argument(
        "--self-test", action="store_true",
        help="run dependency-light helper checks and exit",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.self_test:
        run_self_test()
        print("Hesai GNSS publication provenance self-test: PASS")
        return 0
    missing = [
        name for name in (
            "repo", "run_dir", "dataset", "expected_mode",
            "output_json", "output_markdown",
        )
        if getattr(args, name) is None
    ]
    if missing:
        print(f"missing required arguments: {', '.join(missing)}", file=sys.stderr)
        return 2
    try:
        result = validate(args.repo, args.run_dir, args.dataset, args.expected_mode)
    except Exception as error:  # Always leave a machine-readable fail-closed report.
        result = failure_result(
            args.repo, args.run_dir, args.dataset, args.expected_mode, error
        )
    write_outputs(result, args.output_json, args.output_markdown)
    print(
        f"Hesai GNSS publication provenance: "
        f"{'PASS' if result['summary']['passed'] else 'FAIL'} "
        f"({result['summary']['check_count'] - result['summary']['failed_check_count']}/"
        f"{result['summary']['check_count']} checks)"
    )
    return 0 if result["summary"]["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
