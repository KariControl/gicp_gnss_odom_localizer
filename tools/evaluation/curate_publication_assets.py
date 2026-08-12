#!/usr/bin/env python3
"""Build small, publication-ready evaluation assets from local result artifacts.

The source bags, full CSV files, and logs intentionally remain outside Git.  This
script extracts only normalized metrics and renders compact PNG figures whose
labels describe the sensor setup rather than local bag nicknames.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / "docs" / "evaluation" / "assets"

VELODYNE_LABEL = "Velodyne 32-Line + External IMU — LiDAR/IMU-Only (No GNSS)"
MID360_LABEL = "Livox MID-360 + Internal IMU — LiDAR/IMU-Only (No GNSS)"
COURSE_LABELS = {
    "course_1": "Hesai 32-Line + IMU + RTK GNSS — Course 1",
    "course_2": "Hesai 32-Line + IMU + RTK GNSS — Course 2",
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


def load_wide_csv(path: Path, names: tuple[str, ...]) -> tuple[dict[str, np.ndarray], dict[str, dict[str, np.ndarray]]]:
    data = np.genfromtxt(path, delimiter=",", names=True)
    reference = {
        "time": np.asarray(data["time_from_common_start_sec"], dtype=float),
        "x": np.asarray(data["glim_x"], dtype=float),
        "y": np.asarray(data["glim_y"], dtype=float),
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


def render_gnss(csv_path: Path, output: Path, title: str, group: str) -> None:
    plt = pyplot()
    if group == "local":
        names = ("speed_raw", "precision_raw", "precision_local")
        display = {
            "speed_raw": "Scan-to-scan control",
            "precision_raw": "Scan-to-scan comparison",
            "precision_local": "Scan-to-submap local",
        }
    else:
        names = ("speed_existing", "precision_existing", "precision_global")
        display = {
            "speed_existing": "Existing fusion control",
            "precision_existing": "Existing fusion comparison",
            "precision_global": "Precision global",
        }
    colors = {names[0]: "#e68613", names[1]: "#888888", names[2]: "#2764c4"}
    reference, series = load_wide_csv(csv_path, names)
    output.mkdir(parents=True, exist_ok=True)

    figure, axis = plt.subplots(figsize=(10.5, 4.8))
    axis.plot(reference["x"], reference["y"], color="black", linewidth=2.0, label="GLIM reference")
    for name in names:
        axis.plot(series[name]["x"], series[name]["y"], color=colors[name], linewidth=1.2, label=display[name])
    axis.scatter(reference["x"][0], reference["y"][0], color="#24933d", s=35, label="start", zorder=5)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("Reference x [m]")
    axis.set_ylabel("Reference y [m]")
    axis.set_title(f"{title}\n{group.capitalize()} trajectory (historical provisional alignment)")
    axis.grid(True, alpha=0.25)
    axis.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(
        output / f"{group}_trajectory.png", dpi=150, bbox_inches="tight"
    )
    plt.close(figure)

    for key, ylabel, suffix, caption in (
        ("xy_error", "Absolute XY error [m]", "xy_error", "Absolute XY error"),
        ("yaw_error", "Absolute yaw error [deg]", "yaw_error", "Absolute yaw error"),
    ):
        figure, axis = plt.subplots(figsize=(10.5, 5.0))
        for name in names:
            values = series[name][key]
            rmse = float(np.sqrt(np.mean(values ** 2)))
            axis.plot(reference["time"], values, color=colors[name], linewidth=1.0, label=f"{display[name]} (RMSE {rmse:.3f})")
        axis.set_xlabel("Time from common start [s]")
        axis.set_ylabel(ylabel)
        axis.set_title(f"{title}\n{group.capitalize()} {caption} (historical provisional alignment)")
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=8)
        figure.tight_layout()
        figure.savefig(
            output / f"{group}_{suffix}.png", dpi=150, bbox_inches="tight"
        )
        plt.close(figure)


def curate_lidar() -> list[dict[str, Any]]:
    result_root = ROOT / "test_results"
    vel_run = result_root / "lidar_imu_clean_prefix_20260812" / "velodyne_pre_first_tracking_fatal_precision_i2" / "ab_initial_pose"
    vel_full = result_root / "lidar_imu_submap_20260812" / "velodyne_default_i2_full" / "ab_initial_pose_v4"
    mid_run = result_root / "lidar_imu_submap_20260812" / "mid360_default_i2_full" / "ab_initial_pose_v4"
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

    mid_output = ASSETS / "livox_mid360_internal_imu"
    mid = read_json(mid_run / "evaluation.json")
    render_lidar(
        mid_run / "aligned_samples.csv", mid_output, MID360_LABEL,
        ("control_raw",), {"control_raw": "Scan-to-scan"},
    )
    write_json(
        mid_output / "metrics.json",
        {
            "schema_version": 1,
            "dataset_label": MID360_LABEL,
            "reference": {"method": "GLIM trajectory", "independent_ground_truth": False},
            "publication_status": "scan-to-scan accepted for reporting; scan-to-submap rejected and excluded from figures",
            "alignment": {
                "type": "single shared exact first-common-pose SE(2)",
                "position_and_yaw_share_transform": True,
                "scale_applied": False,
            },
            "scan_to_scan": lidar_stream(mid["accuracy"]["metrics"]["control_raw"]),
            "rejected_scan_to_submap_summary": {
                "included_in_figures": False,
                "overall_ab_passed": mid["passed"],
                "position_rmse_m": mid["accuracy"]["metrics"]["precision_local"]["position_error_m"]["rmse"],
                "yaw_rmse_deg": mid["accuracy"]["metrics"]["precision_local"]["yaw_error_deg"]["rmse"],
                "reason": "The external scan-to-submap branch regressed XY and translation RPE on this dataset.",
            },
        },
    )
    records.append({"directory": mid_output, "sources": [mid_run / "evaluation.json", mid_run / "aligned_samples.csv"], "provenance": {"source_run_id": "mid360_default_i2_full"}})
    return records


def curate_gnss() -> list[dict[str, Any]]:
    root = ROOT / "test_results" / "precision_legacy_anchor_20260812_final"
    records: list[dict[str, Any]] = []
    for course, legacy in (("course_1", "pc2"), ("course_2", "pc3")):
        label = COURSE_LABELS[course]
        source = root / f"{legacy}_glim_ab.json"
        data = read_json(source)
        output = ASSETS / f"hesai_32line_imu_rtk_gnss_{course}"
        local_csv = root / "plots" / f"{legacy}_local_glim.csv"
        global_csv = root / "plots" / f"{legacy}_global_glim.csv"
        render_gnss(local_csv, output, label, "local")
        render_gnss(global_csv, output, label, "global")
        evaluations = data["evaluations"]
        write_json(
            output / "metrics.json",
            {
                "schema_version": 1,
                "dataset_label": label,
                "publication_status": "historical provisional; rerun required with the corrected site-origin profile",
                "reference": {"method": "GLIM trajectory", "independent_ground_truth": False},
                "effective_nmea_origin": {
                    "latitude_deg": 35.681236,
                    "longitude_deg": 139.767125,
                    "altitude_m": 0.0,
                    "source": "base pure_nmea_gnss_conversion parameters",
                    "corrected_shared_site_profile_applied": False,
                },
                "alignment": {
                    "type": "historical frozen calibration-window alignment",
                    "position": "one baseline-fitted SE(2) frozen and shared per local/global group",
                    "yaw": "separate baseline circular yaw offset frozen and shared per local/global group",
                    "position_and_yaw_share_transform": False,
                    "scale_applied": False,
                    "exact_initial_pose": False,
                },
                "local": {
                    "scan_to_scan": gnss_stream(evaluations["precision_raw"]),
                    "scan_to_submap": gnss_stream(evaluations["precision_local"]),
                },
                "global": {
                    "existing_fusion": gnss_stream(evaluations["precision_existing"]),
                    "precision_global": gnss_stream(evaluations["precision_global"]),
                },
                "evaluation_passed_under_historical_protocol": data["passed"],
            },
        )
        records.append({"directory": output, "sources": [source, local_csv, global_csv], "provenance": {"legacy_dataset_id": legacy, "source_result_set": "precision_legacy_anchor_20260812_final"}})
    return records


def curate_lsim() -> list[dict[str, Any]]:
    source = ROOT / "docker_output" / "rosbag2_docker_rviz_e2e_20260810_retry1" / "rviz_window.png"
    output = ASSETS / "autoware_lsim_hesai_course_1"
    output.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, output / "rviz.png")
    write_json(
        output / "metrics.json",
        {
            "schema_version": 1,
            "dataset_label": "Autoware LSim — Hesai 32-Line + IMU + RTK GNSS — Course 1",
            "rviz_evidence": {
                "status": "PASS with one warning",
                "passed_checks": 35,
                "warnings": 1,
                "failures": 0,
                "effective_state_rate_hz": 98.290,
                "maximum_consecutive_xy_step_m": 0.394824,
                "screenshot_resolution_px": [1440, 900],
                "video_available": False,
            },
            "headless_course_1": {
                "status": "PASS with one warning",
                "passed_checks": 35,
                "warnings": 1,
                "failures": 0,
                "effective_state_rate_hz": 52.236,
                "maximum_consecutive_xy_step_m": 0.400703,
            },
            "headless_course_2": {
                "status": "PASS with two warnings",
                "passed_checks": 34,
                "warnings": 2,
                "failures": 0,
                "effective_state_rate_hz": 54.334,
                "maximum_consecutive_xy_step_m": 0.521073,
            },
            "accuracy": "Not evaluated because the LSim output contains no independent ground-truth trajectory.",
        },
    )
    return [{"directory": output, "sources": [source], "provenance": {"source_run_id": "rosbag2_docker_rviz_e2e_20260810_retry1"}}]


def build_manifest(records: list[dict[str, Any]]) -> None:
    entries: list[dict[str, Any]] = []
    for record in records:
        directory: Path = record["directory"]
        files = []
        for path in sorted(directory.iterdir()):
            if path.is_file():
                files.append({"path": str(path.relative_to(ASSETS)), "bytes": path.stat().st_size, "sha256": sha256(path)})
        sources = []
        for source in record["sources"]:
            if isinstance(source, tuple):
                local_source_id, path = source
            else:
                local_source_id, path = source.name, source
            sources.append(
                {"local_source_id": local_source_id, "sha256": sha256(path)}
            )
        entries.append({"dataset": directory.name, "files": files, "provenance": record["provenance"], "source_artifacts": sources})
    write_json(
        ASSETS / "manifest.json",
        {
            "schema_version": 1,
            "generated_by": "tools/evaluation/curate_publication_assets.py",
            "content_policy": "Compact plots and normalized JSON only; no bags, raw logs, or sample CSV files.",
            "datasets": entries,
        },
    )


def check_published_assets() -> None:
    manifest_path = ASSETS / "manifest.json"
    manifest = read_json(manifest_path)
    if manifest.get("schema_version") != 1:
        raise RuntimeError("unsupported publication manifest schema")

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
            elif path.suffix == ".png":
                if path.read_bytes()[:8] != b"\x89PNG\r\n\x1a\n":
                    raise RuntimeError(f"invalid PNG signature: {relative}")
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
    args = parser.parse_args()
    if args.check:
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
    records = curate_lidar() + curate_gnss() + curate_lsim()
    build_manifest(records)
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
