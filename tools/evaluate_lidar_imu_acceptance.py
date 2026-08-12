#!/usr/bin/env python3
"""Aggregate Velodyne and MID360 LiDAR/IMU-only acceptance results.

Each run directory must contain the outputs produced by the runtime and GLIM
evaluators::

    runtime_analysis/runtime_metrics.json
    glim_evaluation/metrics.json

The limits in this file are deliberately fixed so a later run cannot silently
weaken its own acceptance criteria.  GLIM is a correlated comparison reference,
not independent ground truth; the generated report states that limitation.
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import tempfile
from typing import Any, Iterable


CRITERIA_VERSION = "lidar_imu_glim_acceptance_v1"
RUNTIME_RELATIVE_PATH = Path("runtime_analysis/runtime_metrics.json")
GLIM_RELATIVE_PATH = Path("glim_evaluation/metrics.json")


class SchemaError(RuntimeError):
    """Raised when an input cannot support a fail-closed decision."""


@dataclass(frozen=True)
class Check:
    name: str
    passed: bool
    observed: Any
    requirement: str
    detail: str = ""


def _path_text(path: Iterable[str | int]) -> str:
    return ".".join(str(item) for item in path)


def require_path(root: Any, *path: str | int) -> Any:
    value = root
    traversed: list[str | int] = []
    for key in path:
        traversed.append(key)
        if isinstance(key, int):
            if not isinstance(value, list) or key < 0 or key >= len(value):
                raise SchemaError(f"missing or invalid {_path_text(traversed)}")
            value = value[key]
        else:
            if not isinstance(value, dict) or key not in value:
                raise SchemaError(f"missing or invalid {_path_text(traversed)}")
            value = value[key]
    return value


def require_mapping(root: Any, *path: str | int) -> dict[str, Any]:
    value = require_path(root, *path)
    if not isinstance(value, dict):
        raise SchemaError(f"{_path_text(path)} must be an object")
    return value


def require_list(root: Any, *path: str | int) -> list[Any]:
    value = require_path(root, *path)
    if not isinstance(value, list):
        raise SchemaError(f"{_path_text(path)} must be an array")
    return value


def require_bool(root: Any, *path: str | int) -> bool:
    value = require_path(root, *path)
    if not isinstance(value, bool):
        raise SchemaError(f"{_path_text(path)} must be a boolean")
    return value


def require_string(root: Any, *path: str | int) -> str:
    value = require_path(root, *path)
    if not isinstance(value, str) or not value:
        raise SchemaError(f"{_path_text(path)} must be a non-empty string")
    return value


def require_number(root: Any, *path: str | int) -> float:
    value = require_path(root, *path)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SchemaError(f"{_path_text(path)} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise SchemaError(f"{_path_text(path)} must be finite")
    return result


def require_one_number(root: Any, alternatives: tuple[tuple[str, ...], ...]) -> float:
    errors: list[str] = []
    for path in alternatives:
        try:
            return require_number(root, *path)
        except SchemaError as error:
            errors.append(str(error))
    names = " or ".join(_path_text(path) for path in alternatives)
    raise SchemaError(f"missing required numeric field ({names})")


def read_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise SchemaError(f"cannot read {path}: {error}") from error
    if not isinstance(value, dict):
        raise SchemaError(f"{path} must contain a JSON object")
    return value


def add_limit_check(
    checks: list[Check],
    name: str,
    observed: float,
    maximum: float,
    unit: str,
) -> None:
    checks.append(
        Check(
            name=name,
            passed=observed <= maximum,
            observed=observed,
            requirement=f"<= {maximum:.6g} {unit}",
            detail=f"observed={observed:.6g} {unit}",
        )
    )


def add_range_check(
    checks: list[Check],
    name: str,
    observed: float,
    minimum: float,
    maximum: float,
) -> None:
    checks.append(
        Check(
            name=name,
            passed=minimum <= observed <= maximum,
            observed=observed,
            requirement=f"{minimum:.6g} <= value <= {maximum:.6g}",
            detail=f"observed={observed:.6g}",
        )
    )


def evaluate_loaded(
    expected_sensor: str,
    run_dir: Path,
    runtime: dict[str, Any],
    glim: dict[str, Any],
) -> dict[str, Any]:
    checks: list[Check] = []

    actual_sensor = require_string(runtime, "sensor")
    checks.append(
        Check(
            "sensor_identity",
            actual_sensor == expected_sensor,
            actual_sensor,
            f"sensor == {expected_sensor}",
        )
    )

    runtime_passed = require_bool(runtime, "passed")
    runtime_checks = require_list(runtime, "checks")
    if not runtime_checks:
        raise SchemaError("runtime checks must not be empty")
    failed_runtime: list[str] = []
    for index, item in enumerate(runtime_checks):
        if not isinstance(item, dict):
            raise SchemaError(f"checks.{index} must be an object")
        name = require_string(item, "name")
        passed = require_bool(item, "passed")
        category = item.get("category", "hard")
        if not isinstance(category, str):
            raise SchemaError(f"checks.{index}.category must be a string")
        if category == "hard" and not passed:
            failed_runtime.append(name)
    all_runtime_hard_checks = not failed_runtime
    checks.append(
        Check(
            "runtime_all_checks_pass",
            runtime_passed and all_runtime_hard_checks,
            runtime_passed and all_runtime_hard_checks,
            "runtime passed == true and every hard runtime check passed",
            "failed=" + (", ".join(failed_runtime) if failed_runtime else "none"),
        )
    )

    reference_duration = require_number(glim, "glim", "duration_sec")
    reference_samples = require_number(glim, "glim", "samples")
    reference_path = require_number(glim, "glim", "xy_path_m")
    common_duration = require_number(glim, "common", "duration_sec")
    common_samples = require_number(glim, "common", "samples")
    if reference_duration <= 0.0 or reference_samples <= 0.0 or reference_path <= 0.0:
        raise SchemaError("GLIM duration, sample count, and XY path must be positive")
    if common_duration < 0.0 or common_samples < 0.0:
        raise SchemaError("common duration and sample count must be non-negative")
    duration_coverage = common_duration / reference_duration
    sample_coverage = common_samples / reference_samples
    checks.extend(
        [
            Check(
                "glim_duration_coverage",
                duration_coverage >= 0.98,
                duration_coverage,
                ">= 0.98",
                f"common/reference={duration_coverage:.6%}",
            ),
            Check(
                "glim_sample_coverage",
                sample_coverage >= 0.95,
                sample_coverage,
                ">= 0.95",
                f"common/reference={sample_coverage:.6%}",
            ),
        ]
    )

    local = require_mapping(glim, "common", "local")
    full = require_mapping(local, "full_alignment")
    initial = require_mapping(local, "initial_distance_alignment")
    full_alignment = require_mapping(full, "alignment")

    path_ratio = require_number(full, "path_ratio_estimate_over_reference")
    diagnostic_scale = require_one_number(
        full_alignment,
        (
            ("diagnostic_similarity_scale_not_applied",),
            ("diagnostic_similarity_scale",),
        ),
    )
    add_range_check(checks, "path_ratio", path_ratio, 0.95, 1.05)
    add_range_check(checks, "diagnostic_similarity_scale", diagnostic_scale, 0.95, 1.05)

    full_xy_rmse = require_number(full, "position_error_m", "rmse")
    full_yaw_rmse = require_number(full, "yaw_error_deg", "rmse")
    full_yaw_p95 = require_number(full, "yaw_error_deg", "p95")
    full_xy_limit = max(1.0, 0.01 * reference_path)
    add_limit_check(checks, "full_xy_rmse", full_xy_rmse, full_xy_limit, "m")
    add_limit_check(checks, "full_yaw_rmse", full_yaw_rmse, 3.0, "deg")
    add_limit_check(checks, "full_yaw_p95", full_yaw_p95, 5.0, "deg")

    initial_distance = require_number(glim, "common", "initial_alignment_distance_m")
    checks.append(
        Check(
            "initial_alignment_distance_contract",
            abs(initial_distance - 20.0) <= 1.0e-9,
            initial_distance,
            "== 20 m",
            "the drift-sensitive alignment must be fitted only to the first 20 m",
        )
    )
    initial_xy_rmse = require_number(initial, "position_error_m", "rmse")
    initial_xy_p95 = require_number(initial, "position_error_m", "p95")
    initial_endpoint_xy = require_number(initial, "endpoint_position_error_m")
    initial_yaw_rmse = require_number(initial, "yaw_error_deg", "rmse")
    initial_xy_limit = max(1.5, 0.015 * reference_path)
    initial_p95_limit = max(3.0, 0.03 * reference_path)
    initial_endpoint_limit = max(2.0, 0.02 * reference_path)
    add_limit_check(
        checks, "initial_distance_xy_rmse", initial_xy_rmse, initial_xy_limit, "m"
    )
    add_limit_check(
        checks, "initial_distance_xy_p95", initial_xy_p95, initial_p95_limit, "m"
    )
    add_limit_check(
        checks,
        "initial_distance_endpoint_xy",
        initial_endpoint_xy,
        initial_endpoint_limit,
        "m",
    )
    add_limit_check(checks, "initial_distance_yaw_rmse", initial_yaw_rmse, 3.0, "deg")

    rpe_items = require_list(local, "rpe")
    rpe_summary: list[dict[str, Any]] = []
    evaluable_rpe = 0
    for index, item in enumerate(rpe_items):
        if not isinstance(item, dict):
            raise SchemaError(f"common.local.rpe.{index} must be an object")
        distance = require_number(item, "distance_m")
        count = require_number(item, "count")
        if distance <= 0.0 or count < 0.0:
            raise SchemaError(f"common.local.rpe.{index} has invalid distance/count")
        if count == 0.0:
            rpe_summary.append(
                {
                    "distance_m": distance,
                    "count": 0,
                    "evaluated": False,
                    "reason": "reference path is too short for this interval",
                }
            )
            continue
        evaluable_rpe += 1
        translation_rmse = require_number(item, "translation_error_m", "rmse")
        yaw_rmse = require_number(item, "yaw_error_deg", "rmse")
        translation_limit = 0.05 * distance
        distance_name = f"{distance:g}m"
        add_limit_check(
            checks,
            f"rpe_{distance_name}_translation_rmse",
            translation_rmse,
            translation_limit,
            "m",
        )
        add_limit_check(
            checks, f"rpe_{distance_name}_yaw_rmse", yaw_rmse, 2.5, "deg"
        )
        rpe_summary.append(
            {
                "distance_m": distance,
                "count": int(count),
                "evaluated": True,
                "translation_rmse_m": translation_rmse,
                "translation_limit_m": translation_limit,
                "yaw_rmse_deg": yaw_rmse,
                "yaw_limit_deg": 2.5,
            }
        )
    checks.append(
        Check(
            "rpe_evaluable_entries",
            evaluable_rpe > 0,
            evaluable_rpe,
            ">= 1",
            "zero-count intervals are not treated as measured errors",
        )
    )

    result_checks = [asdict(item) for item in checks]
    return {
        "passed": all(item.passed for item in checks),
        "sensor": expected_sensor,
        "run_dir": str(run_dir.resolve()),
        "inputs": {
            "runtime_metrics": str((run_dir / RUNTIME_RELATIVE_PATH).resolve()),
            "glim_metrics": str((run_dir / GLIM_RELATIVE_PATH).resolve()),
        },
        "reference": {
            "duration_sec": reference_duration,
            "samples": int(reference_samples),
            "xy_path_m": reference_path,
        },
        "coverage": {
            "duration": duration_coverage,
            "samples": sample_coverage,
        },
        "metrics": {
            "path_ratio_estimate_over_reference": path_ratio,
            "diagnostic_similarity_scale_not_applied": diagnostic_scale,
            "full_alignment": {
                "position_rmse_m": full_xy_rmse,
                "position_rmse_limit_m": full_xy_limit,
                "yaw_rmse_deg": full_yaw_rmse,
                "yaw_p95_deg": full_yaw_p95,
            },
            "initial_distance_alignment": {
                "alignment_distance_m": initial_distance,
                "position_rmse_m": initial_xy_rmse,
                "position_rmse_limit_m": initial_xy_limit,
                "position_p95_m": initial_xy_p95,
                "position_p95_limit_m": initial_p95_limit,
                "endpoint_position_error_m": initial_endpoint_xy,
                "endpoint_position_limit_m": initial_endpoint_limit,
                "yaw_rmse_deg": initial_yaw_rmse,
            },
            "rpe": rpe_summary,
        },
        "checks": result_checks,
        "failed_checks": [item.name for item in checks if not item.passed],
    }


def evaluate_run(expected_sensor: str, run_dir: Path) -> dict[str, Any]:
    runtime_path = run_dir / RUNTIME_RELATIVE_PATH
    glim_path = run_dir / GLIM_RELATIVE_PATH
    try:
        runtime = read_json_object(runtime_path)
        glim = read_json_object(glim_path)
        return evaluate_loaded(expected_sensor, run_dir, runtime, glim)
    except (SchemaError, KeyError, TypeError, ValueError) as error:
        check = Check(
            "input_schema",
            False,
            None,
            "both evaluator outputs contain every required finite typed field",
            str(error),
        )
        return {
            "passed": False,
            "sensor": expected_sensor,
            "run_dir": str(run_dir.resolve()),
            "inputs": {
                "runtime_metrics": str(runtime_path.resolve()),
                "glim_metrics": str(glim_path.resolve()),
            },
            "checks": [asdict(check)],
            "failed_checks": [check.name],
        }


def criteria_document() -> dict[str, Any]:
    return {
        "runtime": "top-level pass and every hard runtime check must pass",
        "glim_duration_coverage_minimum": 0.98,
        "glim_sample_coverage_minimum": 0.95,
        "path_ratio_range_inclusive": [0.95, 1.05],
        "diagnostic_similarity_scale_range_inclusive": [0.95, 1.05],
        "full_alignment": {
            "position_rmse_m_maximum": "max(1.0, 0.01 * reference_path_m)",
            "yaw_rmse_deg_maximum": 3.0,
            "yaw_p95_deg_maximum": 5.0,
        },
        "initial_distance_alignment": {
            "position_rmse_m_maximum": "max(1.5, 0.015 * reference_path_m)",
            "position_p95_m_maximum": "max(3.0, 0.03 * reference_path_m)",
            "endpoint_position_m_maximum": "max(2.0, 0.02 * reference_path_m)",
            "yaw_rmse_deg_maximum": 3.0,
        },
        "each_evaluable_rpe": {
            "translation_rmse_maximum": "0.05 * interval_distance_m",
            "yaw_rmse_deg_maximum": 2.5,
        },
    }


CAVEATS = [
    (
        "GLIMと評価対象odometryは同じLiDAR/IMU観測を使用するため"
        "誤差が相関しています。これは独立ground truthに対する絶対精度ではなく、"
        "GLIMを比較参照とした平面軌跡の整合度です。"
    ),
    (
        "MID360のGLIM参照生成設定にはLiDAR–IMU並進"
        "(0.011, 0.02329, -0.04412) m（約5.1 cm）が含まれますが、"
        "本試験はユーザー提示条件に従いLiDAR/IMU同一姿勢・位置"
        "（identity外部TF）で実行します。"
        "この外部パラメータ差は比較誤差へ影響し得ます。"
    ),
    "planar評価のためZ/roll/pitchは合否対象外です。",
]


def aggregate(velodyne_run: Path, mid360_run: Path) -> dict[str, Any]:
    runs = {
        "velodyne": evaluate_run("velodyne", velodyne_run),
        "mid360": evaluate_run("mid360", mid360_run),
    }
    return {
        "passed": all(item["passed"] for item in runs.values()),
        "criteria_version": CRITERIA_VERSION,
        "criteria": criteria_document(),
        "runs": runs,
        "caveats": CAVEATS,
    }


def markdown_escape(value: Any) -> str:
    return str(value).replace("|", "\\|").replace("\n", " ")


def fmt(value: Any, digits: int = 4) -> str:
    if value is None:
        return "-"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return f"{float(value):.{digits}f}"
    return str(value)


def write_report(path: Path, result: dict[str, Any]) -> None:
    lines = [
        "# LiDAR/IMU-only GLIM acceptance",
        "",
        f"Overall: **{'PASS' if result['passed'] else 'FAIL'}**",
        "",
        f"Criteria: `{result['criteria_version']}`",
        "",
        (
            "| Sensor | Runtime | Duration coverage | Sample coverage | Full XY RMSE | "
            "Full yaw RMSE | Initial-fit endpoint XY | Result |"
        ),
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for sensor in ("velodyne", "mid360"):
        run = result["runs"][sensor]
        if "metrics" not in run:
            lines.append(
                f"| {sensor} | - | - | - | - | - | - | **FAIL (invalid input)** |"
            )
            continue
        runtime_ok = next(
            item["passed"] for item in run["checks"] if item["name"] == "runtime_all_checks_pass"
        )
        metrics = run["metrics"]
        lines.append(
            f"| {sensor} | {'PASS' if runtime_ok else 'FAIL'} | "
            f"{run['coverage']['duration']:.3%} | {run['coverage']['samples']:.3%} | "
            f"{metrics['full_alignment']['position_rmse_m']:.4f} m | "
            f"{metrics['full_alignment']['yaw_rmse_deg']:.3f} deg | "
            f"{metrics['initial_distance_alignment']['endpoint_position_error_m']:.4f} m | "
            f"**{'PASS' if run['passed'] else 'FAIL'}** |"
        )

    lines.extend(
        [
            "",
            "## Fixed criteria",
            "",
            "- runtime audit: top-level PASSかつ全hard checkがPASS",
            "- GLIM coverage: duration ≥ 98%、samples ≥ 95%",
            "- path ratio / diagnostic similarity scale: 0.95–1.05（両端を含む）",
            "- full SE(2): XY RMSE ≤ max(1 m, pathの1%)、yaw RMSE ≤ 3°、yaw p95 ≤ 5°",
            (
                "- initial 20 m fit: XY RMSE ≤ max(1.5 m, pathの1.5%)、"
                "XY p95 ≤ max(3 m, pathの3%)、"
                "endpoint XY ≤ max(2 m, pathの2%)、yaw RMSE ≤ 3°"
            ),
            "- 各評価可能RPE: translation RMSE ≤ 区間距離の5%、yaw RMSE ≤ 2.5°",
        ]
    )

    for sensor in ("velodyne", "mid360"):
        run = result["runs"][sensor]
        lines.extend(
            [
                "",
                f"## {sensor}",
                "",
                f"Run: `{run['run_dir']}`",
                "",
                "| Check | Result | Observed | Requirement | Detail |",
                "|---|---:|---:|---|---|",
            ]
        )
        for check in run["checks"]:
            lines.append(
                f"| {markdown_escape(check['name'])} | "
                f"{'PASS' if check['passed'] else 'FAIL'} | "
                f"{markdown_escape(fmt(check['observed']))} | "
                f"{markdown_escape(check['requirement'])} | "
                f"{markdown_escape(check['detail'])} |"
            )
        if "metrics" in run:
            skipped = [item for item in run["metrics"]["rpe"] if not item["evaluated"]]
            if skipped:
                intervals = ", ".join(f"{item['distance_m']:g} m" for item in skipped)
                lines.extend(
                    [
                        "",
                        f"RPE除外（reference path不足、count=0）: {intervals}",
                    ]
                )

    lines.extend(["", "## Important limitations", ""])
    lines.extend(f"- {item}" for item in result["caveats"])
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def synthetic_metrics(sensor: str) -> tuple[dict[str, Any], dict[str, Any]]:
    runtime = {
        "passed": True,
        "sensor": sensor,
        "checks": [
            {"name": "synthetic_runtime", "passed": True, "category": "hard", "detail": ""}
        ],
    }
    reference_path = 100.0
    aligned = {
        "alignment": {"diagnostic_similarity_scale_not_applied": 1.05},
        "path_ratio_estimate_over_reference": 0.95,
        "position_error_m": {"rmse": 1.0, "p95": 2.0},
        "yaw_error_deg": {"rmse": 3.0, "p95": 5.0},
        "endpoint_position_error_m": 1.0,
    }
    initial = copy.deepcopy(aligned)
    initial["position_error_m"] = {"rmse": 1.5, "p95": 3.0}
    initial["endpoint_position_error_m"] = 2.0
    initial["yaw_error_deg"] = {"rmse": 3.0, "p95": 4.0}
    glim = {
        "glim": {"duration_sec": 100.0, "samples": 1000, "xy_path_m": reference_path},
        "common": {
            "duration_sec": 98.0,
            "samples": 950,
            "initial_alignment_distance_m": 20.0,
            "local": {
                "full_alignment": aligned,
                "initial_distance_alignment": initial,
                "rpe": [
                    {
                        "distance_m": 10.0,
                        "count": 100,
                        "translation_error_m": {"rmse": 0.5},
                        "yaw_error_deg": {"rmse": 2.5},
                    },
                    {"distance_m": 200.0, "count": 0},
                ],
            },
        },
    }
    return runtime, glim


def self_test() -> None:
    runtime, glim = synthetic_metrics("velodyne")
    passing = evaluate_loaded("velodyne", Path("/synthetic/velodyne"), runtime, glim)
    assert passing["passed"], passing["failed_checks"]
    assert not passing["metrics"]["rpe"][1]["evaluated"]

    failing_glim = copy.deepcopy(glim)
    failing_glim["common"]["local"]["full_alignment"]["position_error_m"]["rmse"] = 1.000001
    failing = evaluate_loaded(
        "velodyne", Path("/synthetic/velodyne-fail"), runtime, failing_glim
    )
    assert not failing["passed"]
    assert "full_xy_rmse" in failing["failed_checks"]

    malformed = copy.deepcopy(glim)
    del malformed["common"]["local"]["full_alignment"]["yaw_error_deg"]
    try:
        evaluate_loaded("velodyne", Path("/synthetic/malformed"), runtime, malformed)
    except SchemaError:
        pass
    else:
        raise AssertionError("malformed metrics did not fail closed")

    mid_runtime, mid_glim = synthetic_metrics("mid360")
    combined = {
        "passed": True,
        "criteria_version": CRITERIA_VERSION,
        "criteria": criteria_document(),
        "runs": {
            "velodyne": passing,
            "mid360": evaluate_loaded(
                "mid360", Path("/synthetic/mid360"), mid_runtime, mid_glim
            ),
        },
        "caveats": CAVEATS,
    }
    with tempfile.TemporaryDirectory() as directory:
        report = Path(directory) / "REPORT.md"
        write_report(report, combined)
        text = report.read_text(encoding="utf-8")
        assert "Overall: **PASS**" in text
        assert "約5.1 cm" in text


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--velodyne-run", type=Path)
    parser.add_argument("--mid360-run", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--self-test", action="store_true")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.self_test:
        self_test()
        print("PASS: synthetic acceptance self-test")
        return 0
    if args.velodyne_run is None or args.mid360_run is None or args.output_dir is None:
        parser.error("--velodyne-run, --mid360-run, and --output-dir are required")
    if args.output_dir.exists():
        parser.error(f"--output-dir already exists: {args.output_dir}")
    args.output_dir.mkdir(parents=True)
    result = aggregate(args.velodyne_run, args.mid360_run)
    (args.output_dir / "acceptance.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    report_path = args.output_dir / "FINAL_ACCEPTANCE.md"
    write_report(report_path, result)
    print(f"{'PASS' if result['passed'] else 'FAIL'}: {report_path}")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
