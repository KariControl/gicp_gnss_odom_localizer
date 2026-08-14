#!/usr/bin/env python3
"""Run a plan-driven, reproducible, storage-bounded MID-360 parameter search.

Each candidate is replayed once over the complete bag.  Metrics are extracted
before any deletion.  Within a stage only the current winner's generated
``localization_output`` bag is retained; a displaced winner and every rejected
candidate are pruned through a fail-closed path/content check.  Parameters,
logs, runtime reports, tuning metrics, and prune receipts are always retained.

This tool ranks candidates.  The final winning scan-to-scan and scan-to-submap
profiles must still be rerun as a retained pair and passed to the formal A/B
evaluator before publication.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from typing import Any

import numpy as np
import yaml


REPO = Path(__file__).resolve().parents[1]
DEFAULT_PLAN = REPO / (
    "src/pure_odometry_bringup/config/evaluation/lidar_imu/mid360/"
    "rosbag2_2026_04_08-22_52_42/experimental/tuning/plan.yaml"
)
WORKSPACE_MARKER = ".mid360_tuning_workspace.json"
SAFE_ID = re.compile(r"^[a-z][a-z0-9_]{0,63}$")
ODOM_ROOT = "/**"
MATCHER_ROOT = "submap_matcher"
MIGRATION_RECEIPT = "workspace_migration.json"
FINAL_STATUS_REVALIDATION_RECEIPT = "FINAL_STATUS_REVALIDATION.json"
FINAL_STATUS_REVALIDATION_REASON = (
    "bugfix: formal A/B evaluator hardcoded snapshot cadence 2 instead of "
    "validating the selected cadence recorded by the precision runner"
)
FINAL_STATUS_REVALIDATION_WINNER = "c08"
FINAL_AB_ARTIFACTS = {
    "REPORT.md",
    "aligned_samples.csv",
    "evaluation.json",
    "position_error.png",
    "trajectory_overlay.png",
    "yaw_error.png",
}
_CONTENT_HASH_CACHE: dict[tuple[str, int, int, int, int], str] = {}


def command_bytes(arguments: list[str]) -> bytes:
    completed = subprocess.run(
        arguments, cwd=REPO, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    return completed.stdout


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def workspace_provenance() -> dict[str, str]:
    return {
        "tuning_tool_sha256": sha256(Path(__file__).resolve()),
        "git_head_sha": command_bytes(["git", "rev-parse", "HEAD"]).decode().strip(),
        "git_dirty_diff_sha256": sha256_bytes(
            command_bytes(["git", "diff", "--binary", "HEAD"])
        ),
        "git_status_sha256": sha256_bytes(
            command_bytes(
                ["git", "status", "--porcelain=v1", "--untracked-files=all"]
            )
        ),
    }


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError(f"expected a JSON object: {path}")
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_stable(path: Path) -> str:
    """Hash immutable evaluation inputs, caching only while full stat identity matches."""
    resolved = path.resolve()
    if not resolved.is_file() or resolved.is_symlink():
        raise RuntimeError(f"dataset input is not a regular file: {path}")
    stat = resolved.stat()
    key = (str(resolved), stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns)
    if key not in _CONTENT_HASH_CACHE:
        digest = sha256(resolved)
        after = resolved.stat()
        after_key = (
            str(resolved), after.st_ino, after.st_size, after.st_mtime_ns,
            after.st_ctime_ns,
        )
        if after_key != key:
            raise RuntimeError(f"dataset input changed while hashing: {path}")
        _CONTENT_HASH_CACHE[key] = digest
    return _CONTENT_HASH_CACHE[key]


def dataset_source_fingerprint(plan: dict[str, Any]) -> dict[str, Any]:
    dataset = plan["dataset"]
    bag = dataset_path(dataset["bag"])
    metadata = bag / "metadata.yaml"
    storage = sorted([*bag.glob("*.db3"), *bag.glob("*.mcap")])
    if not bag.is_dir() or bag.is_symlink() or not metadata.is_file() or not storage:
        raise RuntimeError(f"dataset bag is incomplete: {bag}")
    glim = dataset_path(dataset["glim_dir"]) / "traj_lidar.txt"
    imu_profile = dataset_path(dataset["imu_profile"])
    odom_profile = dataset_path(dataset["odom_profile"])
    runner = (REPO / "script/run_lidar_imu_glim_bag.sh").resolve()
    return {
        "format": 1,
        "dataset_id": dataset["id"],
        "bag": str(bag),
        "bag_metadata_sha256": sha256_stable(metadata),
        "bag_storage": [
            {
                "name": item.name,
                "bytes": item.stat().st_size,
                "sha256": sha256_stable(item),
            }
            for item in storage
        ],
        "glim_trajectory": {"path": str(glim), "sha256": sha256_stable(glim)},
        "imu_profile": {"path": str(imu_profile), "sha256": sha256_stable(imu_profile)},
        "odom_profile": {
            "path": str(odom_profile), "sha256": sha256_stable(odom_profile)
        },
        "runner": {"path": str(runner), "sha256": sha256_stable(runner)},
    }


def assert_dataset_source_fingerprint(
    plan: dict[str, Any], expected: dict[str, Any]
) -> None:
    current = dataset_source_fingerprint(plan)
    if current != expected:
        raise RuntimeError("dataset source fingerprint changed; refusing mixed-source tuning")


def assert_repo_provenance(expected: dict[str, str]) -> None:
    current = workspace_provenance()
    if current != expected:
        raise RuntimeError("repository/tool provenance changed during tuning")


def validate_run_dataset_fingerprint(
    run: Path, fingerprint: dict[str, Any], repo_provenance: dict[str, str]
) -> None:
    env = read_env(run / "run.env")
    storage = fingerprint["bag_storage"]
    expected = {
        "input_bag_metadata_sha256": fingerprint["bag_metadata_sha256"],
        "input_bag_storage_files": ",".join(item["name"] for item in storage),
        "input_bag_storage_sha256": ",".join(item["sha256"] for item in storage),
        "glim_traj_sha256": fingerprint["glim_trajectory"]["sha256"],
        "imu_param_sha256": fingerprint["imu_profile"]["sha256"],
        "odom_override_sha256": fingerprint["odom_profile"]["sha256"],
        "git_head_sha": repo_provenance["git_head_sha"],
        "git_dirty_diff_sha256": repo_provenance["git_dirty_diff_sha256"],
        "git_status_sha256": repo_provenance["git_status_sha256"],
    }
    differences = {
        key: {"expected": value, "actual": env.get(key)}
        for key, value in expected.items() if env.get(key) != value
    }
    if differences:
        raise RuntimeError(f"run.env dataset fingerprint mismatch: {differences}")


def evidence_hashes(workspace: Path) -> dict[str, str]:
    patterns = (
        "summaries/*.json",
        "runs/*/*/tuning_metrics.json",
        "runs/*/*/pruned_localization_output.json",
        "configs/*/*/*.yaml",
        "configs/*/*/candidate.json",
    )
    paths = sorted({path for pattern in patterns for path in workspace.glob(pattern)})
    return {str(path.relative_to(workspace)): sha256(path) for path in paths if path.is_file()}


def validate_completed_evidence(workspace: Path, plan: dict[str, Any]) -> dict[str, str]:
    completed = []
    for stage in plan["stages"]:
        summary_path = workspace / "summaries" / f"{stage['id']}.json"
        if not summary_path.exists():
            break
        summary = read_json(summary_path)
        if not summary.get("completed") or len(summary.get("ranking", [])) != len(
            stage["candidates"]
        ):
            raise RuntimeError(f"incomplete or malformed completed summary: {summary_path}")
        for item in summary["ranking"]:
            run = Path(item["run"])
            if run.resolve().parent.parent != (workspace / "runs").resolve():
                raise RuntimeError(f"summary run escapes workspace: {run}")
            metrics = run / "tuning_metrics.json"
            if not metrics.is_file() or read_json(metrics).get("score") != item["score"]:
                raise RuntimeError(f"summary/metrics mismatch: {run}")
            config = workspace / "configs" / stage["id"] / item["id"]
            manifest = read_json(config / "candidate.json")
            if sha256(config / "odom_tuning_override.yaml") != manifest["odom_sha256"]:
                raise RuntimeError(f"odometry config hash mismatch: {config}")
            if sha256(config / "matcher_override.yaml") != manifest["matcher_sha256"]:
                raise RuntimeError(f"matcher config hash mismatch: {config}")
            bag = run / "localization_output"
            receipt = run / "pruned_localization_output.json"
            if not bag.exists() and not (
                receipt.is_file() and read_json(receipt).get("deleted_files")
            ):
                raise RuntimeError(f"missing bag has no valid prune receipt: {run}")
        completed.append(stage["id"])
    if not completed:
        raise RuntimeError("migration requires at least one completed stage")
    return evidence_hashes(workspace)


def metrics_vector(item: dict[str, Any]) -> dict[str, float]:
    training = item["training"]
    return {
        "score": float(item["score"]),
        "xy_rmse_m": float(training["position_error_m"]["rmse"]),
        "yaw_rmse_deg": float(training["yaw_error_deg"]["rmse"]),
        "path_ratio_absolute_error": abs(
            float(training["path_ratio_estimate_over_reference"]) - 1.0
        ),
        "low_motion_excess_m": float(item["training_low_motion"]["excess_path_m"]),
        "rpe_10m_translation_rmse_m": float(
            item["training_rpe_10m"]["translation_error_m"]["rmse"]
        ),
        "rpe_10m_yaw_rmse_deg": float(
            item["training_rpe_10m"]["yaw_error_deg"]["rmse"]
        ),
    }


def hash_source_run(workspace: Path, stage: str, candidate: str) -> dict[str, str]:
    result = {}
    for relative in (
        f"runs/{stage}/{candidate}/tuning_metrics.json",
        f"runs/{stage}/{candidate}/run.env",
        f"configs/{stage}/{candidate}/candidate.json",
        f"configs/{stage}/{candidate}/odom_tuning_override.yaml",
    ):
        path = workspace / relative
        if not path.is_file():
            raise RuntimeError(f"reconciliation source missing: {path}")
        result[relative] = sha256(path)
    return result


def validate_source_hashes(workspace: Path, expected: dict[str, str]) -> None:
    if not isinstance(expected, dict) or not expected:
        raise RuntimeError("post-Z reconciliation has no source hashes")
    for relative, expected_hash in expected.items():
        path = workspace / relative
        if (
            path.resolve() == workspace.resolve()
            or workspace.resolve() not in path.resolve().parents
            or not path.is_file()
            or sha256(path) != expected_hash
        ):
            raise RuntimeError(f"post-Z reconciliation source changed: {relative}")


def validate_reconciled_semantics(
    winner_id: str,
    effective: dict[str, Any],
    d00_effective: dict[str, Any],
    z_effective: dict[str, Any] | None = None,
    z_has_own_classifier: bool = False,
) -> None:
    if winner_id == "dz_safe":
        expected = dict(d00_effective)
        expected["lidar_odom.smoother.zupt.enable"] = False
        for key in (
            "lidar_odom.smoother.zupt.w_trans",
            "lidar_odom.smoother.zupt.w_yaw",
        ):
            expected.pop(key, None)
        if effective != expected:
            raise RuntimeError("DZ_SAFE is not exact D00 plus ZUPT-off semantics")
        return
    if winner_id not in {"z00", "z01", "z02", "z03"}:
        raise RuntimeError(f"unexpected post-Z winner: {winner_id!r}")
    if effective != z_effective:
        raise RuntimeError("adopted Z winner differs from its exact candidate semantics")
    if not z_has_own_classifier:
        raise RuntimeError("adopted Z winner lacks its own stop-classifier evidence")
    enabled = effective.get("lidar_odom.smoother.zupt.enable")
    if enabled is not (winner_id != "z00"):
        raise RuntimeError("adopted Z winner enable state differs from candidate ID")
    weight_keys = {
        "lidar_odom.smoother.zupt.w_trans",
        "lidar_odom.smoother.zupt.w_yaw",
    }
    if enabled != weight_keys.issubset(effective):
        raise RuntimeError("adopted Z winner weights differ from enabled state")


def validate_post_z_reconciliation(workspace: Path) -> dict[str, Any]:
    path = workspace / "summaries/post_z_reconciliation.json"
    summary_path = workspace / "summaries/zupt.json"
    if not path.is_file() or not summary_path.is_file():
        raise RuntimeError("post-Z reconciliation or Z summary is missing")
    reconciliation = read_json(path)
    summary = read_json(summary_path)
    if reconciliation.get("holdout_read_or_computed") is not False:
        raise RuntimeError("post-Z reconciliation violated holdout policy")
    validate_source_hashes(workspace, reconciliation.get("source_sha256", {}))
    winner = reconciliation.get("winner", {})
    summary_winner = summary.get("winner", {})
    if (
        summary.get("post_z_reconciliation") != str(path)
        or summary_winner.get("id") != winner.get("id")
        or summary_winner.get("effective_odom") != winner.get("effective_odom")
        or summary_winner.get("effective_matcher") != {}
    ):
        raise RuntimeError("Z summary and post-Z winner semantics differ")
    winner_id = winner.get("id")
    effective = winner.get("effective_odom")
    if not isinstance(effective, dict):
        raise RuntimeError("post-Z winner lacks effective odometry")
    d00 = read_json(workspace / "configs/stop_detection/d00/candidate.json")
    z_effective = None
    z_has_own_classifier = False
    if winner_id in {"z00", "z01", "z02", "z03"}:
        candidate = read_json(workspace / f"configs/zupt/{winner_id}/candidate.json")
        metrics = read_json(workspace / f"runs/zupt/{winner_id}/tuning_metrics.json")
        z_effective = candidate["effective_odom"]
        z_has_own_classifier = isinstance(metrics.get("stop_classifier"), dict)
    validate_reconciled_semantics(
        winner_id, effective, d00["effective_odom"], z_effective,
        z_has_own_classifier,
    )
    return reconciliation


def coupled_dz_reconciliation(workspace: Path) -> dict[str, Any]:
    summaries = {
        name: read_json(workspace / "summaries" / f"{name}.json")
        for name in ("smoother", "stop_detection", "zupt")
    }
    by_id = {
        stage: {item["id"]: item for item in summary["ranking"]}
        for stage, summary in summaries.items()
    }
    q01 = by_id["smoother"]["q01"]
    d00 = by_id["stop_detection"]["d00"]
    d03 = by_id["stop_detection"]["d03"]
    q_method = q01["metrics"]["method"]
    d_method = d00["metrics"]["method"]
    repeat_contract = {
        "method_equal": q_method == d_method,
        "anchor_equal": q_method.get("alignment_anchor_stamp_ns")
        == d_method.get("alignment_anchor_stamp_ns"),
        "training_samples_equal": q01["metrics"]["training"]["samples"]
        == d00["metrics"]["training"]["samples"],
        "nonclassifier_hard_gates_equal": {
            key: value for key, value in q01["metrics"]["hard_gates"].items()
        } == {
            key: value for key, value in d00["metrics"]["hard_gates"].items()
            if key != "stop_classifier_passed"
        },
        "semantic_effective_params_equal_except_explicit_d00_stop_and_zupt_off": all(
            d00["effective_odom"].get(key) == value
            for key, value in q01["effective_odom"].items()
        ),
    }
    if not all(repeat_contract.values()):
        raise RuntimeError(f"Q01/D00 safe-repeat contract failed: {repeat_contract}")
    safe_vectors = {"q01": metrics_vector(q01["metrics"]), "d00": metrics_vector(d00["metrics"])}
    limits = {
        key: max(safe_vectors["q01"][key], safe_vectors["d00"][key]) * 1.02
        for key in safe_vectors["q01"]
    }
    adoption = {
        "score_max": safe_vectors["d00"]["score"] * 0.95,
        "low_motion_excess_m_max": safe_vectors["d00"]["low_motion_excess_m"] * 0.90,
    }
    inherited_classifier = d03["metrics"].get("stop_classifier", {})
    comparisons = []
    eligible = []
    for candidate_id in ("z00", "z01", "z02", "z03"):
        item = by_id["zupt"][candidate_id]
        vector = metrics_vector(item["metrics"])
        own_classifier = item["metrics"].get("stop_classifier")
        classifier_available = isinstance(own_classifier, dict)
        classifier_safe = bool(
            classifier_available
            and own_classifier.get("moving_false_positive_rate", math.inf) <= 0.01
            and own_classifier.get("precision", 0.0) >= 0.98
        )
        finite = all(math.isfinite(value) for value in vector.values())
        runtime = all(item["metrics"].get("hard_gates", {}).values())
        zupt_on = candidate_id != "z00"
        recall_ok = bool(
            classifier_available
            and (not zupt_on or own_classifier.get("stationary_recall", 0.0) >= 0.70)
        )
        envelope = all(vector[key] <= limits[key] for key in vector)
        improvement = (
            vector["score"] <= adoption["score_max"]
            and vector["low_motion_excess_m"] <= adoption["low_motion_excess_m_max"]
        )
        passed = finite and runtime and classifier_safe and recall_ok and envelope and improvement
        comparison = {
            "id": candidate_id,
            "source": f"runs/zupt/{candidate_id}",
            "zupt_enabled": zupt_on,
            "classifier_source": (
                f"runs/zupt/{candidate_id} (own output)"
                if classifier_available else "missing; adoption fails closed"
            ),
            "own_stop_classifier": own_classifier,
            "vector": vector,
            "checks": {
                "finite_metrics": finite,
                "runtime_hard_gates": runtime,
                "classifier_fpr_precision": classifier_safe,
                "zupt_on_recall": recall_ok,
                "safe_control_nonregression_envelope": envelope,
                "adoption_improvement": improvement,
            },
            "passed": passed,
        }
        comparisons.append(comparison)
        if passed:
            eligible.append(comparison)
    chosen = min(
        eligible,
        key=lambda item: (
            item["vector"]["score"], item["vector"]["low_motion_excess_m"],
            item["vector"]["path_ratio_absolute_error"], item["id"],
        ),
        default=None,
    )
    d00_config = read_json(workspace / "configs/stop_detection/d00/candidate.json")
    if chosen is None:
        winner_id = "dz_safe"
        effective_odom = dict(d00_config["effective_odom"])
        effective_odom["lidar_odom.smoother.zupt.enable"] = False
        reason = (
            "All D03/Z candidates failed the safe Q01/D00 localization envelope "
            "and mandatory adoption improvement; use D00 with ZUPT disabled."
        )
        source_metrics = d00["metrics"]
    else:
        winner_id = chosen["id"]
        selected = by_id["zupt"][winner_id]
        effective_odom = selected["effective_odom"]
        reason = "Aggressive D03/Z candidate passed every coupled adoption rule."
        source_metrics = selected["metrics"]
    source_hashes = {}
    for stage, candidate in (
        ("smoother", "q01"), ("stop_detection", "d00"),
        ("stop_detection", "d03"), ("zupt", "z00"), ("zupt", "z01"),
        ("zupt", "z02"), ("zupt", "z03"),
    ):
        source_hashes.update(hash_source_run(workspace, stage, candidate))
    reconciliation = {
        "format": 1,
        "holdout_read_or_computed": False,
        "reason": reason,
        "safe_controls": safe_vectors,
        "safe_repeat_contract": repeat_contract,
        "nonregression_limits_2_percent": limits,
        "adoption_limits": adoption,
        "inherited_d03_stop_classifier_diagnostic_only": inherited_classifier,
        "comparisons": comparisons,
        "winner": {"id": winner_id, "effective_odom": effective_odom},
        "source_sha256": source_hashes,
        "note": (
            "stop detection changes gyro-bias adaptation even when ZUPT is disabled; "
            "D03/z00 is therefore not the safe control."
        ),
    }
    path = workspace / "summaries/post_z_reconciliation.json"
    if path.exists() and read_json(path) != reconciliation:
        raise RuntimeError("existing post-Z reconciliation differs")
    path.write_text(json.dumps(reconciliation, indent=2, sort_keys=True) + "\n")
    safe_winner = {
        "id": winner_id,
        "eligible": True,
        "score": float(source_metrics["score"]),
        "effective_odom": effective_odom,
        "effective_matcher": {},
        "metrics": source_metrics,
        "changed_parameter_count": 0,
        "bag_retained": False,
        "reconciliation": str(path),
        "selection_reason": reason,
    }
    summaries["zupt"]["pre_reconciliation_winner"] = summaries["zupt"]["winner"]
    summaries["zupt"]["winner"] = safe_winner
    summaries["zupt"]["report_only_winner"] = None
    summaries["zupt"]["selected_candidate"] = safe_winner
    summaries["zupt"]["candidate_adopted"] = True
    summaries["zupt"]["selection_status"] = "ADOPTED_SAFE_RECONCILIATION"
    summaries["zupt"]["post_z_reconciliation"] = str(path)
    (workspace / "summaries/zupt.json").write_text(
        json.dumps(summaries["zupt"], indent=2, sort_keys=True) + "\n"
    )
    return reconciliation


def load_plan(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("version") != 1:
        raise RuntimeError("tuning plan must be a version-1 mapping")
    dataset = value.get("dataset")
    required_dataset = {
        "id", "sensor", "bag", "glim_dir", "mid_yaw_policy",
        "gicp_epsilon", "label", "imu_profile", "odom_profile",
    }
    if not isinstance(dataset, dict) or set(dataset) != required_dataset:
        raise RuntimeError(
            f"dataset keys must be exactly {sorted(required_dataset)}"
        )
    if dataset["sensor"] != "mid360":
        raise RuntimeError("this tuner currently supports sensor=mid360 only")
    if dataset["mid_yaw_policy"] not in {
        "fixed-bias-direct", "adaptive", "external-imu-adaptive",
    }:
        raise RuntimeError("invalid dataset mid_yaw_policy")
    if dataset["gicp_epsilon"] not in {"default", "strict"}:
        raise RuntimeError("invalid dataset gicp_epsilon")
    if not all(isinstance(dataset[key], str) and dataset[key] for key in dataset):
        raise RuntimeError("dataset values must be non-empty strings")
    execution = value.get("execution")
    if not isinstance(execution, dict) or set(execution) != {
        "minimum_free_gib", "serial", "prune_rejected_bags",
    }:
        raise RuntimeError("execution must define minimum_free_gib/serial/prune_rejected_bags")
    if execution["serial"] is not True or execution["prune_rejected_bags"] is not True:
        raise RuntimeError("tuning must remain serial with rejected-bag pruning enabled")
    if not isinstance(execution["minimum_free_gib"], (int, float)) or execution[
        "minimum_free_gib"
    ] < 50:
        raise RuntimeError("minimum_free_gib must be at least 50")
    stages = value.get("stages")
    if not isinstance(stages, list) or not stages:
        raise RuntimeError("tuning plan must contain non-empty stages")
    seen: set[str] = set()
    previous: str | None = None
    for stage in stages:
        if not isinstance(stage, dict) or not SAFE_ID.fullmatch(str(stage.get("id", ""))):
            raise RuntimeError(f"invalid stage: {stage!r}")
        stage_id = stage["id"]
        if stage_id in seen:
            raise RuntimeError(f"duplicate stage id: {stage_id}")
        dependency = stage.get("depends_on")
        if dependency != previous:
            raise RuntimeError(
                f"{stage_id}: depends_on must be the immediately preceding stage "
                f"({previous!r}), got {dependency!r}"
            )
        mode = stage.get("mode")
        if mode not in ("baseline", "precision"):
            raise RuntimeError(f"{stage_id}: invalid mode {mode!r}")
        fixed_odom = stage.get("fixed_odom", {})
        if not isinstance(fixed_odom, dict):
            raise RuntimeError(f"{stage_id}: fixed_odom must be a mapping")
        candidates = stage.get("candidates")
        if not isinstance(candidates, list) or len(candidates) < 2:
            raise RuntimeError(f"{stage_id}: at least two candidates are required")
        candidate_ids: set[str] = set()
        for candidate in candidates:
            if not isinstance(candidate, dict) or not SAFE_ID.fullmatch(
                str(candidate.get("id", ""))
            ):
                raise RuntimeError(f"{stage_id}: invalid candidate {candidate!r}")
            candidate_id = candidate["id"]
            if candidate_id in candidate_ids:
                raise RuntimeError(f"{stage_id}: duplicate candidate {candidate_id}")
            candidate_ids.add(candidate_id)
            for group in ("odom", "matcher", "runner"):
                parameters = candidate.get(group, {})
                if not isinstance(parameters, dict):
                    raise RuntimeError(f"{stage_id}/{candidate_id}: {group} must be a mapping")
                if not all(isinstance(key, str) and key for key in parameters):
                    raise RuntimeError(f"{stage_id}/{candidate_id}: invalid {group} key")
                if not all(
                    isinstance(item, (bool, int, float, str)) and not isinstance(item, list)
                    for item in parameters.values()
                ):
                    raise RuntimeError(
                        f"{stage_id}/{candidate_id}: parameter values must be scalars"
                    )
            if mode == "baseline" and candidate.get("matcher"):
                raise RuntimeError(f"{stage_id}/{candidate_id}: matcher requires precision mode")
            runner = candidate.get("runner", {})
            if set(runner) - {"snapshot_interval"}:
                raise RuntimeError(f"{stage_id}/{candidate_id}: invalid runner setting")
            if "snapshot_interval" in runner and (
                mode != "precision" or runner["snapshot_interval"] not in (2, 5)
            ):
                raise RuntimeError(f"{stage_id}/{candidate_id}: invalid snapshot interval")
        seen.add(stage_id)
        previous = stage_id
    split = value.get("split_seconds")
    if not isinstance(split, dict) or set(split) != {"training", "holdout"}:
        raise RuntimeError("split_seconds must contain exactly training and holdout")
    for name, intervals in split.items():
        if not isinstance(intervals, list) or not intervals:
            raise RuntimeError(f"empty {name} intervals")
        for interval in intervals:
            if (
                not isinstance(interval, list)
                or len(interval) != 2
                or not all(isinstance(item, (int, float)) for item in interval)
                or interval[0] < 0
                or interval[1] <= interval[0]
            ):
                raise RuntimeError(f"invalid {name} interval: {interval!r}")
    weights = value.get("score_weights")
    expected_weights = {
        "training_xy_rmse_m",
        "training_yaw_rmse_deg",
        "training_rpe_10m_translation_rmse_m",
        "training_rpe_10m_yaw_rmse_deg",
        "training_path_ratio_absolute_error",
        "training_low_motion_excess_path_m",
    }
    if not isinstance(weights, dict) or set(weights) != expected_weights:
        raise RuntimeError(f"score_weights keys must be {sorted(expected_weights)}")
    if not all(isinstance(item, (int, float)) and item >= 0 for item in weights.values()):
        raise RuntimeError("score weights must be finite non-negative numbers")
    scales = value.get("score_scales")
    if not isinstance(scales, dict) or set(scales) != expected_weights or not all(
        isinstance(item, (int, float)) and item > 0 for item in scales.values()
    ):
        raise RuntimeError("score_scales must contain matching positive values")
    runtime_gates = value.get("runtime_hard_gates")
    if not isinstance(runtime_gates, dict) or set(runtime_gates) != {
        "deskew_success_minimum", "registration_density_minimum",
        "association_coverage_minimum",
    }:
        raise RuntimeError("runtime_hard_gates contract is invalid")
    if not all(isinstance(item, (int, float)) and 0 <= item <= 1 for item in runtime_gates.values()):
        raise RuntimeError("runtime hard gates must be ratios in [0, 1]")
    policy = value.get("selection_policy")
    if not isinstance(policy, dict) or set(policy) != {
        "minimum_score_improvement_fraction", "control_candidates",
        "baseline_nonregression", "precision_hard_gates", "final_precision_stage",
    }:
        raise RuntimeError("selection_policy is missing required fail-closed gates")
    controls = policy["control_candidates"]
    if not isinstance(controls, dict) or set(controls) != {stage["id"] for stage in stages}:
        raise RuntimeError("control_candidates must identify one control per stage")
    for stage in stages:
        candidate_ids = {candidate["id"] for candidate in stage["candidates"]}
        if controls[stage["id"]] not in candidate_ids:
            raise RuntimeError(f"{stage['id']}: control candidate is absent")
    improvement = policy["minimum_score_improvement_fraction"]
    if not isinstance(improvement, (int, float)) or not 0 <= improvement < 1:
        raise RuntimeError("minimum score improvement must be in [0, 1)")
    baseline_gates = policy["baseline_nonregression"]
    expected_vectors = {
        "xy_rmse_m", "yaw_rmse_deg", "path_ratio_absolute_error",
        "rpe_10m_translation_rmse_m", "rpe_10m_yaw_rmse_deg",
    }
    if not isinstance(baseline_gates, dict) or set(baseline_gates) != {
        "reference_stage", "reference_candidate", "metrics",
    } or set(baseline_gates.get("metrics", {})) != expected_vectors:
        raise RuntimeError("baseline_nonregression metric contract is invalid")
    for gate in baseline_gates["metrics"].values():
        if not isinstance(gate, dict) or set(gate) != {"max_ratio", "absolute_margin"}:
            raise RuntimeError("baseline gate requires max_ratio and absolute_margin")
        if gate["max_ratio"] < 1 or gate["absolute_margin"] < 0:
            raise RuntimeError("baseline nonregression limits must be non-negative")
    precision_gates = policy["precision_hard_gates"]
    if not isinstance(precision_gates, dict) or set(precision_gates) != {
        "path_ratio_min", "path_ratio_max", "max_xy_rmse_ratio_to_scan",
        "max_rpe_translation_ratio_to_scan", "max_rpe_yaw_ratio_to_scan",
    }:
        raise RuntimeError("precision hard-gate contract is invalid")
    if not 0 < precision_gates["path_ratio_min"] <= 1 <= precision_gates["path_ratio_max"]:
        raise RuntimeError("precision path-ratio interval must contain 1.0")
    for key in (
        "max_xy_rmse_ratio_to_scan", "max_rpe_translation_ratio_to_scan",
        "max_rpe_yaw_ratio_to_scan",
    ):
        if not isinstance(precision_gates[key], (int, float)) or precision_gates[key] <= 0:
            raise RuntimeError(f"invalid precision gate: {key}")
    final_precision_stage = policy["final_precision_stage"]
    if (
        not isinstance(final_precision_stage, str)
        or final_precision_stage != stages[-1]["id"]
        or stages[-1]["mode"] != "precision"
    ):
        raise RuntimeError("final_precision_stage must name the final precision stage")
    return value


def safe_workspace(path: Path) -> Path:
    root = (REPO / "test_results").resolve()
    requested = path.resolve()
    if requested == root or root not in requested.parents:
        raise RuntimeError(f"workspace must be a child of {root}")
    cursor = requested
    while cursor != root:
        if cursor.exists() and cursor.is_symlink():
            raise RuntimeError(f"workspace path contains a symlink: {cursor}")
        cursor = cursor.parent
    return requested


def initialize_workspace(workspace: Path, plan_path: Path, plan: dict[str, Any]) -> None:
    marker = workspace / WORKSPACE_MARKER
    source_fingerprint = dataset_source_fingerprint(plan)
    expected = {
        "format": 1,
        "repo": str(REPO),
        "plan": str(plan_path.resolve()),
        "plan_sha256": sha256(plan_path),
        "first_run_provenance": workspace_provenance(),
        "dataset_source_fingerprint": source_fingerprint,
    }
    if workspace.exists():
        if not workspace.is_dir() or workspace.is_symlink() or not marker.is_file():
            raise RuntimeError(f"refusing existing unmarked workspace: {workspace}")
        actual = read_json(marker)
        if actual.get("format") == 2:
            receipt_path = workspace / actual.get("migration_receipt", "")
            migrated_ok = (
                actual.get("repo") == expected["repo"]
                and actual.get("plan") == expected["plan"]
                and actual.get("plan_sha256") == expected["plan_sha256"]
                and actual.get("first_run_provenance") == expected["first_run_provenance"]
                and actual.get("dataset_source_fingerprint") == source_fingerprint
                and receipt_path.is_file()
                and sha256(receipt_path) == actual.get("migration_receipt_sha256")
            )
            if not migrated_ok:
                raise RuntimeError(f"migrated workspace marker is invalid: {workspace}")
        elif actual != expected:
            raise RuntimeError(f"workspace marker differs from requested plan: {workspace}")
    else:
        workspace.mkdir(parents=True)
        marker.write_text(json.dumps(expected, indent=2, sort_keys=True) + "\n")
    assert_dataset_source_fingerprint(plan, source_fingerprint)
    (workspace / "runs").mkdir(exist_ok=True)
    (workspace / "configs").mkdir(exist_ok=True)
    (workspace / "summaries").mkdir(exist_ok=True)
    # Store the validated normalized plan, not a mutable filesystem link.
    (workspace / "plan.normalized.json").write_text(
        json.dumps(plan, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if (workspace / "summaries/post_z_reconciliation.json").exists():
        validate_post_z_reconciliation(workspace)


def workspace_locked_inputs(
    workspace: Path, plan: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, str]]:
    marker = read_json(workspace / WORKSPACE_MARKER)
    fingerprint = marker.get("dataset_source_fingerprint")
    if not isinstance(fingerprint, dict):
        raise RuntimeError("workspace marker lacks dataset source fingerprint")
    repo_provenance = marker.get("first_run_provenance")
    if not isinstance(repo_provenance, dict):
        raise RuntimeError("workspace marker lacks repository provenance")
    assert_dataset_source_fingerprint(plan, fingerprint)
    assert_repo_provenance(repo_provenance)
    return fingerprint, repo_provenance


def tree_hashes(path: Path) -> dict[str, dict[str, Any]]:
    result = {}
    for item in sorted(path.rglob("*")):
        relative = str(item.relative_to(path))
        if item.is_symlink():
            result[relative] = {"type": "symlink", "target": str(item.readlink())}
        elif item.is_file():
            result[relative] = {
                "type": "file", "bytes": item.stat().st_size, "sha256": sha256(item)
            }
        elif item.is_dir():
            result[relative] = {"type": "directory"}
        else:
            raise RuntimeError(f"unsupported incomplete-run entry: {item}")
    return result


def assert_no_process_references(path: Path) -> None:
    token = str(path.resolve()).encode()
    for command_path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            command = command_path.read_bytes()
        except (OSError, PermissionError):
            continue
        if token in command:
            raise RuntimeError(f"process {command_path.parent.name} still references {path}")


def recover_incomplete_submap_p00(workspace: Path) -> dict[str, Any] | None:
    run = workspace / "runs/submap_geometry/p00"
    command = workspace / "runs/submap_geometry/p00.command.json"
    config = workspace / "configs/submap_geometry/p00"
    if not run.exists():
        return None
    if run.is_symlink() or not run.is_dir() or (run / "tuning_metrics.json").exists():
        raise RuntimeError(f"p00 is not an incomplete recoverable run: {run}")
    required = {"run.env", "launch.log", "play.log", "record.log", "localization_output"}
    if not required <= {item.name for item in run.iterdir()}:
        raise RuntimeError(f"incomplete p00 lacks expected partial-run evidence: {run}")
    if not command.is_file() or not config.is_dir() or config.is_symlink():
        raise RuntimeError("incomplete p00 command/config provenance is missing")
    assert_no_process_references(run)
    recovery = workspace / "recovery/submap_geometry_p00_interrupted"
    if recovery.exists():
        raise RuntimeError(f"recovery target already exists: {recovery}")
    recovery.mkdir(parents=True)
    before = {
        "run": tree_hashes(run),
        "command_sha256": sha256(command),
        "config": tree_hashes(config),
    }
    run.rename(recovery / "run")
    command.rename(recovery / "p00.command.json")
    config.rename(recovery / "config")
    receipt = {
        "format": 1,
        "source": "runs/submap_geometry/p00",
        "reason": "interrupted before tuning metrics; archived intact for safe same-ID replay",
        "holdout_read_or_computed": False,
        "evidence_before_move": before,
        "replay_path_cleared": not run.exists() and not command.exists() and not config.exists(),
    }
    receipt_path = recovery / "RECOVERY.json"
    receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n")
    return {"path": str(receipt_path.relative_to(workspace)), "sha256": sha256(receipt_path)}


def migrate_workspace(workspace: Path, plan_path: Path, plan: dict[str, Any]) -> dict[str, Any]:
    workspace = safe_workspace(workspace)
    marker_path = workspace / WORKSPACE_MARKER
    if not workspace.is_dir() or workspace.is_symlink() or not marker_path.is_file():
        raise RuntimeError(f"cannot migrate unmarked workspace: {workspace}")
    if (workspace / MIGRATION_RECEIPT).exists():
        raise RuntimeError("workspace migration receipt already exists")
    old_marker = read_json(marker_path)
    if old_marker.get("format") != 1:
        raise RuntimeError("only an unmigrated format-1 workspace can be migrated")
    if (
        old_marker.get("repo") != str(REPO)
        or old_marker.get("plan") != str(plan_path.resolve())
        or old_marker.get("plan_sha256") != sha256(plan_path)
    ):
        raise RuntimeError("old workspace plan/repository contract differs")
    normalized = read_json(workspace / "plan.normalized.json")
    if normalized != plan:
        raise RuntimeError("normalized historical plan differs from the requested plan")
    evidence_before = validate_completed_evidence(workspace, plan)
    completed = [path.stem for path in sorted((workspace / "summaries").glob("*.json"))]
    if completed != [
        "lidar_filter", "scan_acceptance", "scan_vgicp", "smoother",
        "stop_detection", "zupt",
    ]:
        raise RuntimeError(f"unexpected completed-stage set for v3 migration: {completed}")
    def contains_revealed_holdout(value: Any, parent_key: str = "") -> bool:
        if isinstance(value, dict):
            if parent_key != "split_seconds" and set(value) & {
                "holdout", "holdout_low_motion", "winner_holdout_metrics",
                "final_holdout_metrics", "opened_after_all_stages_completed",
            }:
                return True
            return any(
                contains_revealed_holdout(item, str(key)) for key, item in value.items()
            )
        if isinstance(value, list):
            return any(contains_revealed_holdout(item, parent_key) for item in value)
        return False
    result_json = list((workspace / "summaries").glob("*.json")) + list(
        (workspace / "runs").glob("*/*/*.json")
    )
    for path in result_json:
        if contains_revealed_holdout(read_json(path)):
            raise RuntimeError(f"holdout evidence exists before authorized finalization: {path}")
    recovery = recover_incomplete_submap_p00(workspace)
    reconciliation = coupled_dz_reconciliation(workspace)
    evidence_after = evidence_hashes(workspace)
    new_provenance = workspace_provenance()
    receipt = {
        "format": 1,
        "reason": "post-Z coupled safety reconciliation and interrupted-p00 recovery",
        "old_marker": old_marker,
        "new_tool_and_git_provenance": new_provenance,
        "plan": str(plan_path.resolve()),
        "plan_sha256": sha256(plan_path),
        "completed_evidence_sha256_before": evidence_before,
        "completed_evidence_sha256_after": evidence_after,
        "recovery": recovery,
        "reconciliation_sha256": sha256(
            workspace / "summaries/post_z_reconciliation.json"
        ),
        "reconciliation_winner": reconciliation["winner"],
        "holdout_read_or_computed": False,
    }
    receipt_path = workspace / MIGRATION_RECEIPT
    receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n")
    marker = {
        "format": 2,
        "repo": str(REPO),
        "plan": str(plan_path.resolve()),
        "plan_sha256": sha256(plan_path),
        "first_run_provenance": new_provenance,
        "dataset_source_fingerprint": dataset_source_fingerprint(plan),
        "previous_first_run_provenance": old_marker.get("first_run_provenance"),
        "migration_receipt": MIGRATION_RECEIPT,
        "migration_receipt_sha256": sha256(receipt_path),
    }
    marker_path.write_text(json.dumps(marker, indent=2, sort_keys=True) + "\n")
    return receipt


def ros_yaml(root: str, parameters: dict[str, Any]) -> dict[str, Any]:
    return {root: {"ros__parameters": dict(sorted(parameters.items()))}}


def write_yaml(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rendered = yaml.safe_dump(document, sort_keys=False)
    if path.exists() and path.read_text(encoding="utf-8") != rendered:
        raise RuntimeError(f"refusing to replace changed candidate config: {path}")
    path.write_text(rendered, encoding="utf-8")


def stage_by_id(plan: dict[str, Any], stage_id: str) -> dict[str, Any]:
    for stage in plan["stages"]:
        if stage["id"] == stage_id:
            return stage
    raise RuntimeError(f"unknown stage: {stage_id}")


def inherited_parameters(
    workspace: Path, plan: dict[str, Any], stage: dict[str, Any],
    allow_dry_run: bool = False,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    dependency = stage.get("depends_on")
    if dependency is None:
        return {}, {}, {}
    summary_path = workspace / "summaries" / f"{dependency}.json"
    if not summary_path.is_file():
        raise RuntimeError(f"run dependency first: {dependency}")
    summary = read_json(summary_path)
    accepted_dependency = summary.get("completed") or (
        allow_dry_run and summary.get("dry_run") is True
    )
    if not accepted_dependency or not summary.get("winner"):
        raise RuntimeError(f"dependency has no accepted winner: {dependency}")
    if stage["id"] == "submap_geometry" and dependency == "zupt":
        reconciliation_path = workspace / "summaries/post_z_reconciliation.json"
        if not reconciliation_path.is_file() or not summary.get("post_z_reconciliation"):
            raise RuntimeError("submap tuning requires completed post-Z reconciliation")
        reconciliation = validate_post_z_reconciliation(workspace)
        winner_odom = summary["winner"]["effective_odom"]
        if winner_odom != reconciliation["winner"]["effective_odom"]:
            raise RuntimeError("Z summary does not match reconciled effective odometry")
        enabled = winner_odom.get("lidar_odom.smoother.zupt.enable")
        winner_id = reconciliation["winner"]["id"]
        expected_enabled = winner_id != "dz_safe" and winner_id != "z00"
        if enabled is not expected_enabled or summary["winner"].get("effective_matcher"):
            raise RuntimeError("reconciled ZUPT state/matcher semantics are inconsistent")
        weight_keys = {
            "lidar_odom.smoother.zupt.w_trans",
            "lidar_odom.smoother.zupt.w_yaw",
        }
        if expected_enabled != weight_keys.issubset(winner_odom):
            raise RuntimeError("reconciled ZUPT weights do not match enabled state")
    return (
        dict(summary["winner"]["effective_odom"]),
        dict(summary["winner"]["effective_matcher"]),
        dict(summary["winner"].get("effective_runner", {})),
    )


def materialize_candidate(
    workspace: Path,
    plan: dict[str, Any],
    stage: dict[str, Any],
    candidate: dict[str, Any],
    allow_dry_run_dependency: bool = False,
) -> dict[str, Any]:
    inherited_odom, inherited_matcher, inherited_runner = inherited_parameters(
        workspace, plan, stage, allow_dry_run_dependency
    )
    effective_odom = inherited_odom | stage.get("fixed_odom", {}) | candidate.get("odom", {})
    effective_matcher = inherited_matcher | candidate.get("matcher", {})
    effective_runner = inherited_runner | candidate.get("runner", {})
    directory = workspace / "configs" / stage["id"] / candidate["id"]
    odom_path = directory / "odom_tuning_override.yaml"
    matcher_path = directory / "matcher_override.yaml"
    write_yaml(odom_path, ros_yaml(ODOM_ROOT, effective_odom))
    write_yaml(matcher_path, ros_yaml(MATCHER_ROOT, effective_matcher))
    manifest = {
        "stage": stage["id"],
        "candidate": candidate["id"],
        "mode": stage["mode"],
        "dependency": stage.get("depends_on"),
        "delta_odom": candidate.get("odom", {}),
        "fixed_odom": stage.get("fixed_odom", {}),
        "delta_matcher": candidate.get("matcher", {}),
        "delta_runner": candidate.get("runner", {}),
        "effective_odom": effective_odom,
        "effective_matcher": effective_matcher,
        "effective_runner": effective_runner,
        "odom_sha256": sha256(odom_path),
        "matcher_sha256": sha256(matcher_path),
    }
    manifest_path = directory / "candidate.json"
    if manifest_path.exists():
        existing = read_json(manifest_path)
        existing_core = {key: value for key, value in existing.items() if key != "execution"}
        if existing_core != manifest:
            raise RuntimeError(f"candidate manifest differs on resume: {manifest_path}")
    else:
        manifest_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    return manifest | {"odom_path": odom_path, "matcher_path": matcher_path}


def update_candidate_manifest_after_run(
    manifest: dict[str, Any], run: Path, command: list[str]
) -> None:
    env = read_env(run / "run.env")
    provenance_fields = (
        "git_head_sha",
        "git_dirty_diff_sha256",
        "git_status_sha256",
        "input_bag_metadata_sha256",
        "input_bag_storage_files",
        "input_bag_storage_sha256",
        "glim_traj_sha256",
        "imu_param_sha256",
        "odom_override_sha256",
        "odom_tuning_override_sha256",
        "matcher_param_sha256",
        "matcher_override_param_sha256",
    )
    missing = [field for field in provenance_fields if not env.get(field)]
    if missing:
        raise RuntimeError(f"run.env lacks audited provenance: {missing}")
    manifest_path = (
        Path(manifest["odom_path"]).parent / "candidate.json"
    )
    audited = {key: value for key, value in manifest.items() if not key.endswith("_path")}
    audited["execution"] = {
        "argv": command,
        "cwd": str(REPO),
        "provenance": {field: env[field] for field in provenance_fields},
    }
    if manifest_path.exists():
        existing = read_json(manifest_path)
        if "execution" in existing and existing != audited:
            raise RuntimeError(f"resume provenance differs: {manifest_path}")
    manifest_path.write_text(
        json.dumps(audited, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def import_canonical() -> Any:
    path = REPO / "tools/evaluate_glim_trajectory.py"
    spec = importlib.util.spec_from_file_location("mid360_tuning_canonical", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def read_env(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if line and not line.lstrip().startswith("#") and "=" in line:
            key, value = line.split("=", 1)
            result[key] = value
    return result


def exact_anchor_alignment(
    canonical: Any, estimate: Any, reference: Any, anchor_stamp_ns: int
) -> tuple[Any, int]:
    indices = np.flatnonzero(estimate.stamp_ns == anchor_stamp_ns)
    if len(indices) != 1:
        raise RuntimeError(
            f"candidate lacks the one exact shared alignment anchor: {anchor_stamp_ns}"
        )
    index = int(indices[0])
    if int(reference.stamp_ns[index]) != anchor_stamp_ns:
        raise RuntimeError("interpolated GLIM anchor stamp differs from the estimate stamp")
    yaw = float(canonical.wrap_angle(reference.yaw[index] - estimate.yaw[index]))
    rotation = np.asarray(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]],
        dtype=float,
    )
    translation = reference.xy[index] - rotation @ estimate.xy[index]
    return canonical.Alignment(
        rotation=rotation,
        translation=translation,
        yaw_rad=yaw,
        singular_values=np.asarray([], dtype=float),
        diagnostic_similarity_scale=1.0,
    ), index


def selected_intervals(time_sec: np.ndarray, intervals: list[list[float]]) -> np.ndarray:
    mask = np.zeros(len(time_sec), dtype=bool)
    for start, end in intervals:
        mask |= (time_sec >= float(start)) & (time_sec < float(end))
    return mask


def interval_path(xy: np.ndarray, selected: np.ndarray) -> float:
    if len(xy) < 2:
        return 0.0
    valid_step = selected[:-1] & selected[1:]
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0)[valid_step], axis=1)))


def error_stats(canonical: Any, values: np.ndarray) -> dict[str, Any]:
    if np.count_nonzero(np.isfinite(values)) < 3:
        raise RuntimeError("metric split contains fewer than three finite samples")
    return asdict(canonical.stats(values))


def block_contained_rpe_10m(
    canonical: Any, estimate: Any, reference: Any, intervals: list[list[float]], origin_ns: int
) -> dict[str, Any]:
    translation, yaw = [], []
    time = (estimate.stamp_ns - origin_ns) * 1.0e-9
    for start_sec, end_sec in intervals:
        mask = (time >= start_sec) & (time < end_sec)
        est, ref = estimate.subset(mask), reference.subset(mask)
        if len(est.stamp_ns) < 3:
            continue
        cumulative = np.concatenate(
            ([0.0], np.cumsum(np.linalg.norm(np.diff(ref.xy, axis=0), axis=1)))
        )
        for start in range(len(cumulative)):
            end = int(np.searchsorted(cumulative, cumulative[start] + 10.0, side="left"))
            if end >= len(cumulative):
                break
            ey, ry = float(est.yaw[start]), float(ref.yaw[start])
            er = np.asarray([[math.cos(ey), math.sin(ey)], [-math.sin(ey), math.cos(ey)]])
            rr = np.asarray([[math.cos(ry), math.sin(ry)], [-math.sin(ry), math.cos(ry)]])
            translation.append(float(np.linalg.norm(
                er @ (est.xy[end] - est.xy[start]) - rr @ (ref.xy[end] - ref.xy[start])
            )))
            yaw.append(abs(math.degrees(float(canonical.wrap_angle(
                (est.yaw[end] - est.yaw[start]) - (ref.yaw[end] - ref.yaw[start])
            )))))
    if not translation:
        raise RuntimeError("training blocks contain no complete 10 m RPE segment")
    return {
        "distance_m": 10.0,
        "segments_cross_holdout_gaps": False,
        "translation_error_m": error_stats(canonical, np.asarray(translation)),
        "yaw_error_deg": error_stats(canonical, np.asarray(yaw)),
    }


def split_metrics(
    canonical: Any,
    aligned: Any,
    reference: Any,
    position_error: np.ndarray,
    yaw_error_deg: np.ndarray,
    mask: np.ndarray,
) -> dict[str, Any]:
    samples = int(np.count_nonzero(mask))
    if samples < 3:
        raise RuntimeError("split contains fewer than three samples")
    reference_path = interval_path(reference.xy, mask)
    estimate_path = interval_path(aligned.xy, mask)
    if reference_path <= 0.0:
        raise RuntimeError("split reference path is zero")
    return {
        "samples": samples,
        "position_error_m": error_stats(canonical, position_error[mask]),
        "yaw_error_deg": error_stats(canonical, yaw_error_deg[mask]),
        "reference_path_m": reference_path,
        "estimate_path_m": estimate_path,
        "path_ratio_estimate_over_reference": estimate_path / reference_path,
    }


def stop_classifier_metrics(
    run: Path, estimate: Any, reference: Any, training_mask: np.ndarray
) -> dict[str, Any]:
    try:
        import rosbag2_py  # type: ignore
        from rclpy.serialization import deserialize_message  # type: ignore
        from rosidl_runtime_py.utilities import get_message  # type: ignore
    except ImportError as error:
        raise RuntimeError("source ROS 2 before stop-classifier evaluation") from error
    bag = run / "localization_output"
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if types.get("/clock") != "rosgraph_msgs/msg/Clock" or types.get(
        "/localization/is_stopped"
    ) != "std_msgs/msg/Bool":
        raise RuntimeError("result lacks /clock or /localization/is_stopped")
    clock_type = get_message(types["/clock"])
    stop_type = get_message(types["/localization/is_stopped"])
    clock_record, clock_sim, stop_record, stop_value = [], [], [], []
    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        if topic == "/clock":
            message = deserialize_message(serialized, clock_type)
            clock_record.append(record_ns)
            clock_sim.append(
                int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)
            )
        elif topic == "/localization/is_stopped":
            message = deserialize_message(serialized, stop_type)
            stop_record.append(record_ns)
            stop_value.append(bool(message.data))
    if len(clock_record) < 3 or len(stop_record) < 3:
        raise RuntimeError("insufficient clock/stopped records")
    clock_record_array = np.asarray(clock_record, dtype=np.int64)
    clock_sim_array = np.asarray(clock_sim, dtype=np.int64)
    unique = np.concatenate(([True], np.diff(clock_record_array) > 0))
    stopped_sim = np.interp(
        np.asarray(stop_record, dtype=np.int64),
        clock_record_array[unique], clock_sim_array[unique],
    ).astype(np.int64)
    order = np.argsort(stopped_sim, kind="stable")
    stopped_sim = stopped_sim[order]
    stopped_value = np.asarray(stop_value, dtype=bool)[order]
    sample_index = np.searchsorted(stopped_sim, estimate.stamp_ns, side="right") - 1
    valid = sample_index >= 0
    predicted = np.zeros(len(estimate.stamp_ns), dtype=bool)
    predicted[valid] = stopped_value[sample_index[valid]]
    step = np.linalg.norm(np.diff(reference.xy, axis=0), axis=1)
    step_training = training_mask[:-1] & training_mask[1:]
    predicted_step = predicted[1:]
    stationary = (step < 0.002) & step_training
    moving = (step >= 0.010) & step_training
    tp = int(np.count_nonzero(predicted_step & stationary))
    fp = int(np.count_nonzero(predicted_step & moving))
    predicted_known = tp + fp
    precision = tp / predicted_known if predicted_known else 0.0
    recall = tp / max(1, int(np.count_nonzero(stationary)))
    false_positive = fp / max(1, int(np.count_nonzero(moving)))
    return {
        "stationary_intervals": int(np.count_nonzero(stationary)),
        "moving_intervals": int(np.count_nonzero(moving)),
        "true_positive": tp,
        "false_positive": fp,
        "precision": precision,
        "stationary_recall": recall,
        "moving_false_positive_rate": false_positive,
        "passed": false_positive <= 0.01 and precision >= 0.98 and recall >= 0.70,
    }


def tuning_metrics(
    run: Path, plan: dict[str, Any], mode: str, include_holdout: bool = False
) -> dict[str, Any]:
    canonical = import_canonical()
    env = read_env(run / "run.env")
    topic = (
        "/localization/gyro_lidar_odom_scan"
        if mode == "baseline"
        else "/localization/precision_local_odom"
    )
    trajectories, duplicates = canonical.read_result_trajectories(
        run / "localization_output", 0, np.iinfo(np.int64).max, "last", (topic,)
    )
    estimate = trajectories[topic]
    glim_path = Path(env["glim_traj"])
    if sha256(glim_path) != env["glim_traj_sha256"]:
        raise RuntimeError("GLIM trajectory changed after the run")
    glim = canonical.read_glim_trajectory(glim_path)
    reference, valid = canonical.interpolate_trajectory(glim, estimate.stamp_ns, 0.15)
    estimate = estimate.subset(valid)
    if len(estimate.stamp_ns) < 3 or len(reference.stamp_ns) != len(estimate.stamp_ns):
        raise RuntimeError("insufficient GLIM-associated samples")
    anchor_stamp_ns = int(plan["alignment_anchor_stamp_ns"])
    alignment, anchor_index = exact_anchor_alignment(
        canonical, estimate, reference, anchor_stamp_ns
    )
    aligned = canonical.apply_alignment(estimate, alignment)
    position_error = np.linalg.norm(aligned.xy - reference.xy, axis=1)
    yaw_error = np.degrees(
        np.abs(canonical.wrap_angle(aligned.yaw - reference.yaw))
    )
    split_origin_ns = int(glim.stamp_ns[0])
    relative_time = (estimate.stamp_ns - split_origin_ns) * 1.0e-9
    training_mask = selected_intervals(relative_time, plan["split_seconds"]["training"])
    holdout_mask = selected_intervals(relative_time, plan["split_seconds"]["holdout"])
    if np.any(training_mask & holdout_mask):
        raise RuntimeError("training and holdout intervals overlap")
    reference_step = np.linalg.norm(np.diff(reference.xy, axis=0), axis=1)
    estimate_step = np.linalg.norm(np.diff(aligned.xy, axis=0), axis=1)
    threshold = float(plan["low_motion_reference_step_threshold_m"])
    training_steps = training_mask[:-1] & training_mask[1:]
    training_low = (reference_step < threshold) & training_steps
    if np.count_nonzero(training_low) < 3:
        raise RuntimeError("fewer than three training low-motion intervals")
    runtime = read_json(run / "runtime_analysis/runtime_metrics.json")
    structural_path = run / "submap_validation/validation.json"
    structural = read_json(structural_path) if structural_path.is_file() else None
    result = {
        "method": {
            "alignment": "exact first associated SE(2) pose; scale not estimated",
            "alignment_anchor_stamp_ns": anchor_stamp_ns,
            "alignment_anchor_index": anchor_index,
            "estimate_sampling": "native estimator header stamps; no estimate interpolation",
            "reference_sampling": "GLIM interpolated at estimator stamps",
            "split_time_origin_stamp_ns": split_origin_ns,
            "split_time_origin": "first GLIM physical timestamp shared by every candidate",
            "split_seconds": plan["split_seconds"],
            "low_motion_reference_step_threshold_m_exclusive": threshold,
            "training_rpe": "computed independently inside each training block",
        },
        "topic": topic,
        "association_coverage": float(np.count_nonzero(valid) / len(valid)),
        "duplicates": asdict(duplicates[topic]),
        "training": split_metrics(
            canonical, aligned, reference, position_error, yaw_error, training_mask
        ),
        "training_rpe_10m": block_contained_rpe_10m(
            canonical, aligned, reference, plan["split_seconds"]["training"],
            split_origin_ns,
        ),
        "training_low_motion": {
            "interval_count": int(np.count_nonzero(training_low)),
            "reference_path_m": float(np.sum(reference_step[training_low])),
            "estimate_path_m": float(np.sum(estimate_step[training_low])),
            "excess_path_m": float(
                np.sum(estimate_step[training_low] - reference_step[training_low])
            ),
            "estimate_step_rmse_m": float(
                np.sqrt(np.mean(np.square(estimate_step[training_low])))
            ),
        },
        "runtime_passed": bool(runtime.get("passed")),
        "registration_density": float(
            runtime.get("odometry", {}).get("scan_registration_density", 0.0)
        ),
        "structural_passed": None if structural is None else bool(structural.get("passed")),
        "structural_metrics": (
            {} if structural is None else structural.get("metrics", {})
        ),
    }
    if run.parent.name in {"stop_detection", "zupt"}:
        result["stop_classifier"] = stop_classifier_metrics(
            run, estimate, reference, training_mask
        )
    weights = plan["score_weights"]
    components = {
        "training_xy_rmse_m": result["training"]["position_error_m"]["rmse"],
        "training_yaw_rmse_deg": result["training"]["yaw_error_deg"]["rmse"],
        "training_rpe_10m_translation_rmse_m": result["training_rpe_10m"]
        ["translation_error_m"]["rmse"],
        "training_rpe_10m_yaw_rmse_deg": result["training_rpe_10m"]
        ["yaw_error_deg"]["rmse"],
        "training_path_ratio_absolute_error": abs(
            result["training"]["path_ratio_estimate_over_reference"] - 1.0
        ),
        "training_low_motion_excess_path_m": max(
            result["training_low_motion"]["excess_path_m"], 0.0
        ),
    }
    result["score_components"] = components
    scales = plan["score_scales"]
    result["score"] = float(sum(
        weights[key] * value / scales[key] for key, value in components.items()
    ))
    deskew_success = float(runtime.get("deskew", {}).get("success_rate", 0.0))
    registration_density = float(
        runtime.get("odometry", {}).get("scan_registration_density", 0.0)
    )
    structural_metrics = {} if structural is None else structural.get("metrics", {})
    runtime_gates = plan["runtime_hard_gates"]
    hard_gates = {
        "runtime_passed": result["runtime_passed"],
        "deskew_success_minimum": deskew_success
        >= float(runtime_gates["deskew_success_minimum"]),
        "registration_density_minimum": registration_density
        >= float(runtime_gates["registration_density_minimum"]),
        "association_coverage_minimum": result["association_coverage"]
        >= float(runtime_gates["association_coverage_minimum"]),
    }
    if "stop_classifier" in result:
        hard_gates["stop_classifier_passed"] = result["stop_classifier"]["passed"]
    if mode == "precision":
        hard_gates |= {
            "structural_passed": result["structural_passed"] is True,
            "matcher_accepted_ratio_at_least_0_80": float(
                structural_metrics.get("matcher_accepted_ratio", 0.0)
            ) >= 0.80,
            "matcher_queue_drop_zero": int(
                structural_metrics.get("matcher_queue_drop_count", -1)
            ) == 0,
            "matcher_latency_p99_at_most_250_ms": float(
                structural_metrics.get("matcher_latency_p99_ms", math.inf)
            ) <= 250.0,
            "post_warmup_correction_ratio_at_least_0_60": float(
                structural_metrics.get("post_warmup_correction_ratio", 0.0)
            ) >= 0.60,
        }
    result["hard_gates"] = hard_gates
    result["eligible"] = bool(
        all(hard_gates.values())
    )
    if include_holdout:
        holdout_steps = holdout_mask[:-1] & holdout_mask[1:]
        holdout_low = (reference_step < threshold) & holdout_steps
        result["holdout"] = split_metrics(
            canonical, aligned, reference, position_error, yaw_error, holdout_mask
        )
        result["full"] = split_metrics(
            canonical, aligned, reference, position_error, yaw_error,
            np.ones(len(estimate.stamp_ns), dtype=bool),
        )
        result["holdout_low_motion"] = {
            "interval_count": int(np.count_nonzero(holdout_low)),
            "reference_path_m": float(np.sum(reference_step[holdout_low])),
            "estimate_path_m": float(np.sum(estimate_step[holdout_low])),
            "excess_path_m": float(
                np.sum(estimate_step[holdout_low] - reference_step[holdout_low])
            ),
        }
    return result


def bag_inventory(bag: Path) -> tuple[list[dict[str, Any]], int]:
    if not bag.is_dir() or bag.is_symlink() or not (bag / "metadata.yaml").is_file():
        raise RuntimeError(f"not a generated rosbag directory: {bag}")
    entries = sorted(bag.iterdir())
    if not entries or any(not item.is_file() or item.is_symlink() for item in entries):
        raise RuntimeError(f"unexpected localization output content: {bag}")
    names = {item.name for item in entries}
    if "metadata.yaml" not in names or not any(item.suffix == ".mcap" for item in entries):
        raise RuntimeError(f"localization output lacks metadata/MCAP: {bag}")
    if any(item.name != "metadata.yaml" and item.suffix != ".mcap" for item in entries):
        raise RuntimeError(f"unexpected localization output file: {bag}")
    inventory = [
        {"name": item.name, "bytes": item.stat().st_size, "sha256": sha256(item)}
        for item in entries
    ]
    return inventory, sum(item["bytes"] for item in inventory)


def prune_rejected_bag(workspace: Path, run: Path, reason: str) -> None:
    expected_parent = (workspace / "runs").resolve()
    run_resolved = run.resolve()
    if run_resolved.parent.parent != expected_parent:
        raise RuntimeError(f"refusing to prune outside stage/candidate run layout: {run}")
    if not SAFE_ID.fullmatch(run_resolved.name) or not SAFE_ID.fullmatch(run_resolved.parent.name):
        raise RuntimeError(f"unsafe run identifiers: {run}")
    bag = run_resolved / "localization_output"
    if not bag.exists():
        receipt = run_resolved / "pruned_localization_output.json"
        if receipt.is_file() and read_json(receipt).get("deleted_files"):
            return
        raise RuntimeError(f"localization output disappeared without prune receipt: {bag}")
    inventory, total = bag_inventory(bag)
    receipt = run_resolved / "pruned_localization_output.json"
    if receipt.exists():
        raise RuntimeError(f"refusing to overwrite prune receipt: {receipt}")
    # Exact files were resolved above; never recurse through an unverified tree.
    for item in sorted(bag.iterdir()):
        item.unlink()
    bag.rmdir()
    receipt.write_text(
        json.dumps(
            {"reason": reason, "deleted_bytes": total, "deleted_files": inventory},
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )


def candidate_result(run: Path, manifest: dict[str, Any]) -> dict[str, Any]:
    metrics = read_json(run / "tuning_metrics.json")
    return {
        "id": manifest["candidate"],
        "eligible": metrics["eligible"],
        "score": metrics["score"],
        "effective_odom": manifest["effective_odom"],
        "effective_matcher": manifest["effective_matcher"],
        "effective_runner": manifest.get("effective_runner", {}),
        "run": str(run),
        "bag_retained": (run / "localization_output").is_dir(),
        "metrics": metrics,
        "changed_parameter_count": (
            len(manifest["delta_odom"]) + len(manifest["delta_matcher"])
            + len(manifest.get("delta_runner", {}))
        ),
    }


def extraction_failure_metrics(error: Exception, runner_status: int) -> dict[str, Any]:
    return {
        "eligible": False,
        "score": math.inf,
        "runner_status": runner_status,
        "extraction_error": str(error),
        "hard_gates": {"metrics_extraction_passed": False},
    }


def ranking_key(item: dict[str, Any]) -> tuple[Any, ...]:
    metrics = item["metrics"]
    if not item["eligible"]:
        return (True, math.inf, math.inf, math.inf, math.inf, item["id"])
    processing = float(
        metrics.get("structural_metrics", {}).get("matcher_processing_p99_ms", math.inf)
    )
    return (
        False,
        metrics["score"],
        processing,
        -float(metrics.get("registration_density", 0.0)),
        item["changed_parameter_count"],
        item["id"],
    )


def pareto_dominated(candidate: dict[str, Any], others: list[dict[str, Any]]) -> bool:
    if not candidate["eligible"]:
        return True
    def vector(item: dict[str, Any]) -> tuple[float, ...]:
        metrics = item["metrics"]
        return (
            metrics["training"]["position_error_m"]["rmse"],
            metrics["training"]["yaw_error_deg"]["rmse"],
            metrics["training_rpe_10m"]["translation_error_m"]["rmse"],
            metrics["training_rpe_10m"]["yaw_error_deg"]["rmse"],
            abs(metrics["training"]["path_ratio_estimate_over_reference"] - 1.0),
            max(metrics["training_low_motion"]["excess_path_m"], 0.0),
        )
    target = vector(candidate)
    for other in others:
        if other is candidate or not other["eligible"]:
            continue
        contender = vector(other)
        if all(left <= right for left, right in zip(contender, target)) and any(
            left < right for left, right in zip(contender, target)
        ):
            return True
    return False


def stop_fallback_safe(candidate: dict[str, Any] | None) -> bool:
    if candidate is None:
        return False
    metrics = candidate.get("metrics", {})
    gates = metrics.get("hard_gates", {})
    return bool(
        metrics.get("extraction_error") is None
        and gates.get("stop_classifier_passed") is False
        and len(gates) > 1
        and all(
            passed
            for name, passed in gates.items()
            if name != "stop_classifier_passed"
        )
    )


def dataset_path(value: str) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (REPO / path).resolve()


def runner_dataset_arguments(plan: dict[str, Any]) -> list[str]:
    dataset = plan["dataset"]
    return [
        "--sensor", dataset["sensor"],
        "--mid-yaw-policy", dataset["mid_yaw_policy"],
        "--gicp-epsilon", dataset["gicp_epsilon"],
        "--bag", str(dataset_path(dataset["bag"])),
        "--glim-dir", str(dataset_path(dataset["glim_dir"])),
    ]


def gate_vector(metrics: dict[str, Any]) -> dict[str, float]:
    return {
        "xy_rmse_m": float(metrics["training"]["position_error_m"]["rmse"]),
        "yaw_rmse_deg": float(metrics["training"]["yaw_error_deg"]["rmse"]),
        "path_ratio_absolute_error": abs(
            float(metrics["training"]["path_ratio_estimate_over_reference"]) - 1.0
        ),
        "rpe_10m_translation_rmse_m": float(
            metrics["training_rpe_10m"]["translation_error_m"]["rmse"]
        ),
        "rpe_10m_yaw_rmse_deg": float(
            metrics["training_rpe_10m"]["yaw_error_deg"]["rmse"]
        ),
    }


def apply_selection_hard_gates(
    workspace: Path,
    plan: dict[str, Any],
    stage: dict[str, Any],
    candidate: dict[str, Any],
    metrics: dict[str, Any],
) -> None:
    """Apply preregistered training-only nonregression gates before ranking."""
    policy = plan["selection_policy"]
    vector = gate_vector(metrics)
    if stage["mode"] == "baseline":
        contract = policy["baseline_nonregression"]
        reference_stage = contract["reference_stage"]
        reference_candidate = contract["reference_candidate"]
        if stage["id"] == reference_stage and candidate["id"] == reference_candidate:
            reference_metrics = metrics
        else:
            reference_path = (
                workspace / "runs" / reference_stage / reference_candidate
                / "tuning_metrics.json"
            )
            if not reference_path.is_file():
                raise RuntimeError(
                    f"baseline gate reference must run first: {reference_path}"
                )
            reference_metrics = read_json(reference_path)
        reference = gate_vector(reference_metrics)
        for name, gate in contract["metrics"].items():
            limit = reference[name] * float(gate["max_ratio"]) + float(
                gate["absolute_margin"]
            )
            metrics["hard_gates"][f"scan_nonregression_{name}"] = bool(
                math.isfinite(vector[name]) and vector[name] <= limit
            )
        metrics["selection_gate_reference"] = {
            "kind": "fixed_scan_to_scan_control",
            "stage": reference_stage,
            "candidate": reference_candidate,
            "vector": reference,
        }
    elif stage["id"] == policy["final_precision_stage"]:
        baseline_stages = [
            item for item in plan["stages"] if item["mode"] == "baseline"
        ]
        if not baseline_stages:
            raise RuntimeError("precision gates require a baseline stage")
        baseline_stage = baseline_stages[-1]["id"]
        scan_summary = read_json(workspace / "summaries" / f"{baseline_stage}.json")
        scan_metrics = scan_summary.get("winner", {}).get("metrics")
        if not isinstance(scan_metrics, dict):
            raise RuntimeError("precision ranking lacks the selected scan-to-scan reference")
        reference = gate_vector(scan_metrics)
        gates = policy["precision_hard_gates"]
        ratio = float(metrics["training"]["path_ratio_estimate_over_reference"])
        metrics["hard_gates"].update({
            "submap_path_ratio_min": math.isfinite(ratio)
            and ratio >= float(gates["path_ratio_min"]),
            "submap_path_ratio_max": math.isfinite(ratio)
            and ratio <= float(gates["path_ratio_max"]),
            "submap_xy_nonregression": vector["xy_rmse_m"]
            <= reference["xy_rmse_m"] * float(gates["max_xy_rmse_ratio_to_scan"]),
            "submap_rpe_translation_nonregression": vector[
                "rpe_10m_translation_rmse_m"
            ] <= reference["rpe_10m_translation_rmse_m"] * float(
                gates["max_rpe_translation_ratio_to_scan"]
            ),
            "submap_rpe_yaw_nonregression": vector["rpe_10m_yaw_rmse_deg"]
            <= reference["rpe_10m_yaw_rmse_deg"] * float(
                gates["max_rpe_yaw_ratio_to_scan"]
            ),
        })
        metrics["selection_gate_reference"] = {
            "kind": "selected_scan_to_scan_training",
            "stage": baseline_stage,
            "candidate": scan_summary["winner"]["id"],
            "vector": reference,
        }
    else:
        metrics["selection_gate_reference"] = {
            "kind": "final_adoption_gates_deferred",
            "final_precision_stage": policy["final_precision_stage"],
        }
    metrics["eligible"] = bool(all(metrics["hard_gates"].values()))


def runner_command(
    workspace: Path,
    plan: dict[str, Any],
    stage: dict[str, Any],
    candidate: dict[str, Any],
    manifest: dict[str, Any],
    rate: float,
) -> list[str]:
    run = workspace / "runs" / stage["id"] / candidate["id"]
    snapshot_interval = manifest.get("effective_runner", {}).get(
        "snapshot_interval", stage.get("snapshot_interval", 2)
    )
    command = [
        "bash",
        str(REPO / "script/run_lidar_imu_glim_bag.sh"),
        *runner_dataset_arguments(plan),
        "--localization-mode",
        stage["mode"],
        "--output",
        str(run),
        "--rate",
        str(rate),
        "--odom-tuning-override",
        str(manifest["odom_path"]),
        "--no-evaluate",
    ]
    if stage["mode"] == "precision":
        command += [
            "--snapshot-interval",
            str(snapshot_interval),
            "--matcher-override",
            str(manifest["matcher_path"]),
        ]
    return command


def write_stage_summary(
    workspace: Path, plan: dict[str, Any], stage: dict[str, Any], rows: list[dict[str, Any]]
) -> dict[str, Any]:
    for item in rows:
        item["pareto_dominated"] = pareto_dominated(item, rows)
    ordered = sorted(rows, key=lambda item: (item["pareto_dominated"],) + ranking_key(item))
    eligible = [
        item for item in ordered if item["eligible"] and not item["pareto_dominated"]
    ]
    winner = eligible[0] if eligible else None
    if winner is not None:
        tied = [item for item in eligible if item["score"] <= winner["score"] * 1.01]
        winner = min(tied, key=lambda item: (
            float(item["metrics"].get("structural_metrics", {}).get(
                "matcher_processing_p99_ms", math.inf
            )),
            -float(item["metrics"].get("registration_density", 0.0)),
            item["changed_parameter_count"], item["id"],
        ))
    # Every stage includes its current/inherited control.  Do not accumulate a
    # parameter change unless the selected score improves by at least 2%.
    control_id = plan["selection_policy"]["control_candidates"][stage["id"]]
    inherited = next((item for item in rows if item["id"] == control_id), None)
    if stage["id"] == "stop_detection" and winner is None:
        # Safety fallback required by the tuning design: no unverified stop
        # threshold may enable ZUPT downstream.
        fallback = next((item for item in rows if item["id"] == "d00"), None)
        if stop_fallback_safe(fallback):
            winner = fallback
            winner["fallback_reason"] = (
                "no stop detector passed FP<=1%, precision>=98%, recall>=70%; "
                "retain current D00 and require Z00 downstream"
            )
    if winner is not None and inherited is not None and inherited["eligible"]:
        improvement = 1.0 - winner["score"] / inherited["score"] if inherited["score"] > 0 else 0.0
        if improvement < float(
            plan["selection_policy"]["minimum_score_improvement_fraction"]
        ):
            winner = inherited
    report_only_winner = None
    is_final_adoption_stage = (
        stage["id"] == plan["selection_policy"]["final_precision_stage"]
    )
    if winner is None and is_final_adoption_stage:
        reportable = []
        for item in rows:
            gates = item["metrics"].get("hard_gates", {})
            non_adoption = {
                name: passed for name, passed in gates.items()
                if not name.startswith("submap_")
            }
            if (
                item["metrics"].get("extraction_error") is None
                and non_adoption
                and all(non_adoption.values())
                and math.isfinite(float(item["score"]))
            ):
                reportable.append(item)
        if reportable:
            report_only_winner = min(reportable, key=lambda item: (
                float(item["score"]),
                float(item["metrics"].get("structural_metrics", {}).get(
                    "matcher_processing_p99_ms", math.inf
                )),
                item["changed_parameter_count"], item["id"],
            ))
            report_only_winner["selection_role"] = "report_only_not_adopted"
    selected_candidate = winner or report_only_winner
    summary = {
        "completed": len(rows) == len(stage["candidates"]),
        "stage": stage["id"],
        "mode": stage["mode"],
        "candidate_count": len(rows),
        "winner": winner,
        "report_only_winner": report_only_winner,
        "selected_candidate": selected_candidate,
        "candidate_adopted": winner is not None,
        "selection_status": (
            "ADOPTED" if winner is not None else
            "REPORTED_WITH_LIMITATIONS" if report_only_winner is not None else
            "NO_VALID_CANDIDATE"
        ),
        "ranking": ordered,
    }
    path = workspace / "summaries" / f"{stage['id']}.json"
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def execute_stage(
    workspace: Path,
    plan: dict[str, Any],
    stage: dict[str, Any],
    rate: float,
    dry_run: bool,
) -> dict[str, Any]:
    source_fingerprint, repo_provenance = workspace_locked_inputs(workspace, plan)
    stage_runs = workspace / "runs" / stage["id"]
    stage_runs.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    for candidate in stage["candidates"]:
        assert_dataset_source_fingerprint(plan, source_fingerprint)
        assert_repo_provenance(repo_provenance)
        manifest = materialize_candidate(
            workspace, plan, stage, candidate, allow_dry_run_dependency=dry_run
        )
        run = stage_runs / candidate["id"]
        command = runner_command(workspace, plan, stage, candidate, manifest, rate)
        if dry_run:
            print(" ".join(subprocess.list2cmdline([item]) for item in command))
            continue
        metrics_path = run / "tuning_metrics.json"
        if metrics_path.is_file():
            validate_run_dataset_fingerprint(run, source_fingerprint, repo_provenance)
            update_candidate_manifest_after_run(manifest, run, command)
            rows.append(candidate_result(run, manifest))
            continue
        if run.exists():
            raise RuntimeError(f"refusing incomplete pre-existing candidate run: {run}")
        free_bytes = shutil.disk_usage(workspace).free
        minimum_free_bytes = int(plan["execution"]["minimum_free_gib"] * 1024**3)
        if free_bytes < minimum_free_bytes:
            raise RuntimeError(
                f"refusing replay with {free_bytes / 1024**3:.2f} GiB free; "
                "at least 50 GiB is required"
            )
        run.parent.mkdir(parents=True, exist_ok=True)
        command_path = run.parent / f"{candidate['id']}.command.json"
        if command_path.exists():
            raise RuntimeError(f"refusing to overwrite command manifest: {command_path}")
        command_path.write_text(
            json.dumps({"argv": command, "cwd": str(REPO)}, indent=2) + "\n",
            encoding="utf-8",
        )
        completed = subprocess.run(command, cwd=REPO, check=False)
        if not (run / "localization_output").is_dir():
            raise RuntimeError(
                f"candidate produced no localization output (status {completed.returncode}): {run}"
            )
        assert_dataset_source_fingerprint(plan, source_fingerprint)
        assert_repo_provenance(repo_provenance)
        validate_run_dataset_fingerprint(run, source_fingerprint, repo_provenance)
        update_candidate_manifest_after_run(manifest, run, command)
        try:
            metrics = tuning_metrics(run, plan, stage["mode"])
        except Exception as error:
            (run / "tuning_error.json").write_text(
                json.dumps({"error": str(error)}, indent=2) + "\n", encoding="utf-8"
            )
            metrics = extraction_failure_metrics(error, completed.returncode)
            metrics_path.write_text(
                json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            prune_rejected_bag(workspace, run, "metrics extraction failed")
            rows.append(candidate_result(run, manifest))
            write_stage_summary(workspace, plan, stage, rows)
            continue
        metrics["runner_status"] = completed.returncode
        if completed.returncode != 0:
            metrics["eligible"] = False
            metrics["hard_gates"]["runner_status_zero"] = False
        if stage["id"] == "zupt":
            dependency = read_json(workspace / "summaries/stop_detection.json")
            if dependency["winner"].get("fallback_reason") and candidate["id"] != "z00":
                metrics["eligible"] = False
                metrics["hard_gates"]["zupt_requires_validated_stop_detector"] = False
        apply_selection_hard_gates(workspace, plan, stage, candidate, metrics)
        metrics_path.write_text(
            json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        rows.append(candidate_result(run, manifest))

        # Preserve both the current full-rule winner and the inherited control;
        # the latter may win via the 2% change threshold after the stage closes.
        current_summary = write_stage_summary(workspace, plan, stage, rows)
        current_winner = current_summary["selected_candidate"]
        control_id = plan["selection_policy"]["control_candidates"][stage["id"]]
        for item in rows:
            item_run = Path(item["run"])
            keep = (
                current_winner is not None and item["id"] == current_winner["id"]
            ) or item["id"] == control_id
            if not keep:
                prune_rejected_bag(
                    workspace,
                    item_run,
                    f"rejected by stage {stage['id']} ranking",
                )
                item["bag_retained"] = False

        write_stage_summary(workspace, plan, stage, rows)
    if dry_run:
        control_id = plan["selection_policy"]["control_candidates"][stage["id"]]
        control = next(item for item in stage["candidates"] if item["id"] == control_id)
        manifest = materialize_candidate(
            workspace, plan, stage, control, allow_dry_run_dependency=True
        )
        # A dry-run chain needs only deterministic inherited semantics; it must
        # never masquerade as completed measurement evidence.
        summary = {
            "completed": False,
            "dry_run": True,
            "stage": stage["id"],
            "mode": stage["mode"],
            "winner": {
                "id": control_id,
                "effective_odom": manifest["effective_odom"],
                "effective_matcher": manifest["effective_matcher"],
                "effective_runner": manifest["effective_runner"],
            },
            "report_only_winner": None,
            "selected_candidate": {
                "id": control_id,
                "effective_odom": manifest["effective_odom"],
                "effective_matcher": manifest["effective_matcher"],
                "effective_runner": manifest["effective_runner"],
            },
            "candidate_adopted": True,
            "selection_status": "DRY_RUN_CONTROL",
            "ranking": [],
        }
        (workspace / "summaries" / f"{stage['id']}.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return summary
    summary = write_stage_summary(workspace, plan, stage, rows)
    selected_candidate = summary["selected_candidate"]
    if selected_candidate is None:
        raise RuntimeError(f"stage has no eligible candidate: {stage['id']}")
    # A resumed run may contain obsolete bags.  Apply the final ranking again.
    for item in summary["ranking"]:
        if item["id"] != selected_candidate["id"]:
            prune_rejected_bag(
                workspace, Path(item["run"]), f"rejected by final {stage['id']} ranking"
            )
    return summary


def final_runner_commands(
    plan: dict[str, Any], winner: dict[str, Any], control: Path, precision: Path,
    odom_path: Path, matcher_path: Path, rate: float,
) -> list[list[str]]:
    snapshot_interval = winner.get("effective_runner", {}).get("snapshot_interval", 2)
    return [
        [
            "bash", str(REPO / "script/run_lidar_imu_glim_bag.sh"),
            *runner_dataset_arguments(plan),
            "--localization-mode", "baseline", "--output", str(control),
            "--rate", str(rate), "--odom-tuning-override", str(odom_path),
            "--no-evaluate",
        ],
        [
            "bash", str(REPO / "script/run_lidar_imu_glim_bag.sh"),
            *runner_dataset_arguments(plan),
            "--localization-mode", "precision", "--snapshot-interval",
            str(snapshot_interval),
            "--output", str(precision), "--rate", str(rate),
            "--odom-tuning-override", str(odom_path),
            "--matcher-override", str(matcher_path), "--no-evaluate",
        ],
    ]


def stat_identity(path: Path) -> dict[str, int]:
    stat = path.stat()
    return {
        "device": stat.st_dev,
        "inode": stat.st_ino,
        "bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
        "ctime_ns": stat.st_ctime_ns,
    }


def require_provenance(
    value: Any, fields: tuple[str, ...], description: str
) -> dict[str, str]:
    if not isinstance(value, dict):
        raise RuntimeError(f"{description} provenance is missing")
    missing = [field for field in fields if not isinstance(value.get(field), str) or not value[field]]
    if missing:
        raise RuntimeError(f"{description} provenance lacks {missing}")
    return value


def audit_tree_before(path: Path, cutoff_ns: int, description: str) -> dict[str, Any]:
    if not path.is_dir() or path.is_symlink():
        raise RuntimeError(f"{description} tree is missing or unsafe: {path}")
    files: list[tuple[Path, int]] = []
    for item in sorted(path.rglob("*")):
        if item.is_symlink():
            target = item.resolve()
            if not target.exists() or (target != path.resolve() and path.resolve() not in target.parents):
                raise RuntimeError(f"{description} contains an unsafe symlink: {item}")
            files.append((item, item.lstat().st_mtime_ns))
        elif item.is_file():
            files.append((item, item.stat().st_mtime_ns))
    if not files:
        raise RuntimeError(f"{description} tree has no files: {path}")
    latest, latest_ns = max(files, key=lambda item: item[1])
    if latest_ns > cutoff_ns:
        raise RuntimeError(
            f"{description} changed after holdout was opened: {latest}"
        )
    return {
        "file_or_safe_symlink_count": len(files),
        "latest_relative_path": str(latest.relative_to(path)),
        "latest_mtime_ns": latest_ns,
        "cutoff_mtime_ns": cutoff_ns,
    }


def hash_file_set(
    base: Path, paths: list[Path], cutoff_ns: int | None = None
) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for path in sorted(paths):
        resolved = path.resolve()
        if (
            path.is_symlink()
            or not path.is_file()
            or (resolved != base.resolve() and base.resolve() not in resolved.parents)
        ):
            raise RuntimeError(f"unsafe final evidence path: {path}")
        identity = stat_identity(path)
        if cutoff_ns is not None and identity["mtime_ns"] > cutoff_ns:
            raise RuntimeError(f"final run evidence changed after holdout: {path}")
        result[str(path.relative_to(base))] = {
            "bytes": identity["bytes"],
            "mtime_ns": identity["mtime_ns"],
            "sha256": sha256(path),
        }
    return result


def read_ros_parameters(path: Path, root: str) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        parameters = document[root]["ros__parameters"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise RuntimeError(f"invalid final ROS parameter artifact {path}: {error}") from error
    if not isinstance(parameters, dict):
        raise RuntimeError(f"final ROS parameter artifact is not a mapping: {path}")
    return parameters


def validate_final_run_evidence(
    workspace: Path,
    run: Path,
    mode: str,
    cutoff_ns: int,
    source_fingerprint: dict[str, Any],
    historical_repo_provenance: dict[str, str],
) -> dict[str, Any]:
    expected_artifacts = {
        "baseline": {
            "base_odom_param.yaml": "base_odom_param_sha256",
            "imu_param.yaml": "imu_param_sha256",
            "odom_aux_override.yaml": "odom_aux_override_sha256",
            "odom_override.yaml": "odom_override_sha256",
            "odom_tuning_override.yaml": "odom_tuning_override_sha256",
        },
        "precision": {
            "base_odom_param.yaml": "base_odom_param_sha256",
            "imu_param.yaml": "imu_param_sha256",
            "odom_aux_override.yaml": "odom_aux_override_sha256",
            "odom_override.yaml": "odom_override_sha256",
            "odom_tuning_override.yaml": "odom_tuning_override_sha256",
            "precision_global_param.yaml": "global_param_sha256",
            "precision_local_only_override.yaml": "global_override_param_sha256",
            "submap_matcher_override.yaml": "matcher_override_param_sha256",
            "submap_matcher_param.yaml": "matcher_param_sha256",
            "submap_snapshot_override.yaml": "snapshot_override_sha256",
        },
    }[mode]
    if not run.is_dir() or run.is_symlink():
        raise RuntimeError(f"final {mode} run is missing or unsafe: {run}")
    env_path = run / "run.env"
    env = read_env(env_path)
    if env.get("localization_mode") != mode:
        raise RuntimeError(f"final {mode} run.env mode differs")
    expected_interval = "5" if mode == "precision" else "n/a"
    if env.get("snapshot_interval") != expected_interval:
        raise RuntimeError(f"final {mode} snapshot interval differs")
    validate_run_dataset_fingerprint(
        run, source_fingerprint, historical_repo_provenance
    )
    artifact_dir = run / "artifacts"
    artifact_entries = (
        list(artifact_dir.iterdir())
        if artifact_dir.is_dir() and not artifact_dir.is_symlink() else []
    )
    actual_artifacts = {item.name for item in artifact_entries}
    if actual_artifacts != set(expected_artifacts):
        raise RuntimeError(
            f"final {mode} artifact set differs: {sorted(actual_artifacts)}"
        )
    if any(item.is_symlink() or not item.is_file() for item in artifact_entries):
        raise RuntimeError(f"final {mode} artifact tree contains a non-file")
    artifact_paths = [artifact_dir / name for name in sorted(expected_artifacts)]
    artifacts = hash_file_set(workspace, artifact_paths, cutoff_ns)
    for name, env_key in expected_artifacts.items():
        actual_hash = artifacts[str((artifact_dir / name).relative_to(workspace))]["sha256"]
        if env.get(env_key) != actual_hash:
            raise RuntimeError(
                f"final {mode} artifact/run.env hash mismatch: {name}"
            )
    bag = run / "localization_output"
    if not bag.is_dir() or bag.is_symlink():
        raise RuntimeError(f"final {mode} localization bag is missing or unsafe")
    bag_entries = list(bag.iterdir())
    bag_files = sorted(bag_entries)
    if (
        any(item.is_symlink() or not item.is_file() for item in bag_entries)
        or not (bag / "metadata.yaml").is_file()
        or not any(item.suffix in {".mcap", ".db3"} for item in bag_files)
        or any(item.name != "metadata.yaml" and item.suffix not in {".mcap", ".db3"}
               for item in bag_files)
    ):
        raise RuntimeError(f"final {mode} localization bag contents differ")
    return {
        "run": str(run),
        "run_env": hash_file_set(workspace, [env_path], cutoff_ns),
        "artifacts": artifacts,
        "localization_bag": hash_file_set(workspace, bag_files, cutoff_ns),
    }


def validate_final_commands(
    workspace: Path, commands_path: Path, cutoff_ns: int
) -> dict[str, Any]:
    document = read_json(commands_path)
    commands = document.get("commands")
    if not isinstance(commands, list) or len(commands) != 2 or not all(
        isinstance(command, list) and all(isinstance(item, str) for item in command)
        for command in commands
    ):
        raise RuntimeError("final replay command provenance is malformed")
    control, precision = commands

    def option(command: list[str], name: str) -> str:
        if command.count(name) != 1:
            raise RuntimeError(f"final command has ambiguous {name}")
        index = command.index(name)
        if index + 1 >= len(command):
            raise RuntimeError(f"final command lacks a value for {name}")
        return command[index + 1]

    runner = str((REPO / "script/run_lidar_imu_glim_bag.sh").resolve())
    expected = (
        (control, "baseline", workspace / "final/scan_to_scan"),
        (precision, "precision", workspace / "final/scan_to_submap"),
    )
    for command, mode, output in expected:
        if len(command) < 2 or command[:2] != ["bash", runner]:
            raise RuntimeError(f"final {mode} command runner differs")
        if option(command, "--localization-mode") != mode:
            raise RuntimeError(f"final {mode} command mode differs")
        if Path(option(command, "--output")).resolve() != output.resolve():
            raise RuntimeError(f"final {mode} command output differs")
        if "--no-evaluate" not in command:
            raise RuntimeError(f"final {mode} command lacks --no-evaluate")
    if "--snapshot-interval" in control:
        raise RuntimeError("final baseline command unexpectedly has snapshot cadence")
    if option(precision, "--snapshot-interval") != "5":
        raise RuntimeError("final precision command did not select snapshot cadence 5")
    return {
        "commands_sha256": sha256(commands_path),
        "commands_mtime_ns": commands_path.stat().st_mtime_ns,
        "commands_before_holdout": commands_path.stat().st_mtime_ns <= cutoff_ns,
        "replay_commands_executed_by_revalidation": [],
    }


def evaluator_revalidation_provenance() -> dict[str, Any]:
    evaluator = REPO / "tools/evaluate_lidar_imu_submap_ab.py"
    relative = str(evaluator.relative_to(REPO))
    git_head_bytes = command_bytes(["git", "show", f"HEAD:{relative}"])
    diff_bytes = command_bytes(
        ["git", "diff", "--binary", "HEAD", "--", relative]
    )
    return {
        "path": str(evaluator),
        "current_evaluator_sha256": sha256(evaluator),
        "current_evaluator_mtime_ns": evaluator.stat().st_mtime_ns,
        "git_head_preimage_sha256": sha256_bytes(git_head_bytes),
        "current_diff_from_git_head_sha256": sha256_bytes(diff_bytes),
        "historical_runtime_evaluator_sha256": None,
        "historical_runtime_hash_recoverable": False,
        "historical_runtime_hash_recovery_detail": (
            "The original run.env and FINAL_STATUS did not record the evaluator hash. "
            "The Git HEAD preimage is recorded, but it predates the uncommitted formal "
            "artifact-contract evaluator and therefore is not claimed as the runtime hash."
        ),
    }


def validate_revalidation_provenance(
    evaluator_provenance: dict[str, Any],
    current_repo_provenance: dict[str, str],
) -> None:
    require_provenance(
        current_repo_provenance,
        (
            "tuning_tool_sha256", "git_head_sha", "git_dirty_diff_sha256",
            "git_status_sha256",
        ),
        "current repository",
    )
    if current_repo_provenance["tuning_tool_sha256"] != sha256(Path(__file__).resolve()):
        raise RuntimeError("current tuning-tool provenance differs from the executing tool")
    required_evaluator = (
        "path", "current_evaluator_sha256", "git_head_preimage_sha256",
        "current_diff_from_git_head_sha256",
        "historical_runtime_hash_recovery_detail",
    )
    require_provenance(evaluator_provenance, required_evaluator, "current evaluator")
    evaluator = (REPO / "tools/evaluate_lidar_imu_submap_ab.py").resolve()
    if (
        Path(evaluator_provenance["path"]).resolve() != evaluator
        or evaluator_provenance["current_evaluator_sha256"] != sha256(evaluator)
        or evaluator_provenance.get("historical_runtime_hash_recoverable") is not False
        or evaluator_provenance.get("historical_runtime_evaluator_sha256") is not None
    ):
        raise RuntimeError("current/historical evaluator provenance is inconsistent")


def validate_revalidation_holdout(holdout: dict[str, Any]) -> None:
    if (
        holdout.get("selection_score_used_holdout") is not False
        or holdout.get("opened_after_all_stages_completed") is not True
        or holdout.get("split_origin_equal") is not True
        or not isinstance(holdout.get("scan_to_scan"), dict)
        or not isinstance(holdout.get("scan_to_submap"), dict)
    ):
        raise RuntimeError("holdout opening/non-selection provenance differs")


def validate_revalidation_evaluation(
    evaluation: dict[str, Any], control: Path, precision: Path,
    glim_trajectory: Path,
) -> list[dict[str, Any]]:
    checks = evaluation.get("checks")
    hard_checks = [
        item for item in checks if isinstance(item, dict) and item.get("category") == "hard"
    ] if isinstance(checks, list) else []
    semantics = evaluation.get("tuning_semantics_contract")
    if (
        evaluation.get("passed") is not True
        or not hard_checks
        or any(item.get("passed") is not True for item in hard_checks)
        or not isinstance(semantics, dict)
        or semantics.get("passed") is not True
        or semantics.get("odometry_semantics_equal") is not True
        or semantics.get("odometry_aux_snapshot_only") is not True
        or semantics.get("snapshot_interval_record_valid") is not True
        or semantics.get("snapshot_interval_recorded") != "5"
        or semantics.get("snapshot_interval_expected") != 5
        or semantics.get("snapshot_interval_matches_artifact") is not True
        or evaluation.get("configuration_contract", {}).get("passed") is not True
    ):
        raise RuntimeError("current formal A/B evaluation or cadence semantics failed")
    inputs = evaluation.get("inputs", {})
    if (
        Path(inputs.get("control_run", "")).resolve() != control.resolve()
        or Path(inputs.get("precision_run", "")).resolve() != precision.resolve()
        or Path(inputs.get("glim", "")).resolve() != glim_trajectory.resolve()
    ):
        raise RuntimeError("current formal A/B evaluation inputs differ")
    return hard_checks


def prepare_final_status_revalidation(
    workspace: Path,
    plan_path: Path,
    plan: dict[str, Any],
    evaluator_provenance: dict[str, Any],
    current_repo_provenance: dict[str, str],
    performed_at_unix_ns: int | None = None,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, int]]:
    """Validate existing evidence and prepare a status-only migration.

    This function deliberately contains no replay, metric extraction, holdout
    computation, candidate selection, or subprocess invocation.
    """
    workspace = safe_workspace(workspace)
    validate_revalidation_provenance(evaluator_provenance, current_repo_provenance)
    marker_path = workspace / WORKSPACE_MARKER
    normalized_path = workspace / "plan.normalized.json"
    marker = read_json(marker_path)
    historical_repo = require_provenance(
        marker.get("first_run_provenance"),
        (
            "tuning_tool_sha256", "git_head_sha", "git_dirty_diff_sha256",
            "git_status_sha256",
        ),
        "historical workspace",
    )
    if (
        marker.get("format") not in {1, 2}
        or marker.get("repo") != str(REPO)
        or marker.get("plan") != str(plan_path.resolve())
        or marker.get("plan_sha256") != sha256(plan_path)
        or read_json(normalized_path) != plan
    ):
        raise RuntimeError("workspace marker/normalized plan provenance differs")
    source_fingerprint = marker.get("dataset_source_fingerprint")
    if not isinstance(source_fingerprint, dict):
        raise RuntimeError("workspace marker lacks dataset source provenance")
    assert_dataset_source_fingerprint(plan, source_fingerprint)
    if current_repo_provenance["git_head_sha"] != historical_repo["git_head_sha"]:
        raise RuntimeError("Git HEAD changed since the final runs")
    stage_ids = [stage["id"] for stage in plan["stages"]]
    actual_summary_ids = sorted(
        path.stem for path in (workspace / "summaries").glob("*.json")
    )
    if len(stage_ids) != 9 or sorted(stage_ids) != actual_summary_ids:
        raise RuntimeError(
            f"status revalidation requires exactly all 9 stage summaries: {actual_summary_ids}"
        )
    completed_evidence = validate_completed_evidence(workspace, plan)
    final_stage = plan["stages"][-1]
    final_summary = read_json(
        workspace / "summaries" / f"{final_stage['id']}.json"
    )
    selected = final_summary.get("selected_candidate")
    winner = final_summary.get("winner")
    if (
        final_summary.get("completed") is not True
        or final_summary.get("candidate_adopted") is not True
        or final_summary.get("selection_status") != "ADOPTED"
        or not isinstance(selected, dict)
        or selected != winner
        or selected.get("id") != FINAL_STATUS_REVALIDATION_WINNER
    ):
        raise RuntimeError("final selected/adopted winner is not the locked c08 candidate")
    final_root = workspace / "final"
    status_path = final_root / "FINAL_STATUS.json"
    holdout_path = final_root / "holdout_metrics.json"
    evaluation_dir = final_root / "ab_initial_pose"
    evaluation_path = evaluation_dir / "evaluation.json"
    for path in (status_path, holdout_path, evaluation_path):
        if not path.is_file() or path.is_symlink():
            raise RuntimeError(f"required final evidence is missing or unsafe: {path}")
    old_status_identity = stat_identity(status_path)
    old_status = read_json(status_path)
    if (
        old_status.get("completed") is not True
        or old_status.get("candidate_adopted") is not True
        or old_status.get("adopted_winner") != winner
        or old_status.get("report_only_winner") != final_summary.get("report_only_winner")
        or old_status.get("retune_after_holdout_reveal_allowed") is not False
        or old_status.get("formal_ab_status") != 1
        or old_status.get("formal_ab_passed") is not False
        or old_status.get("status") != "REPORTED_WITH_LIMITATIONS"
        or Path(old_status.get("holdout_metrics", "")).resolve() != holdout_path.resolve()
        or Path(old_status.get("formal_ab_output", "")).resolve() != evaluation_dir.resolve()
    ):
        raise RuntimeError("previous FINAL_STATUS is not the cadence-bug-only failure state")
    holdout = read_json(holdout_path)
    validate_revalidation_holdout(holdout)
    holdout_mtime_ns = holdout_path.stat().st_mtime_ns
    if holdout_mtime_ns > old_status_identity["mtime_ns"]:
        raise RuntimeError("holdout evidence changed after the previous FINAL_STATUS")
    chronology = {
        name: audit_tree_before(workspace / name, holdout_mtime_ns, name)
        for name in ("runs", "configs", "summaries")
    }
    commands = validate_final_commands(
        workspace, final_root / "commands.json", holdout_mtime_ns
    )
    if commands["commands_before_holdout"] is not True:
        raise RuntimeError("final replay command provenance postdates holdout")
    control = final_root / "scan_to_scan"
    precision = final_root / "scan_to_submap"
    final_runs = {
        "scan_to_scan": validate_final_run_evidence(
            workspace, control, "baseline", holdout_mtime_ns,
            source_fingerprint, historical_repo,
        ),
        "scan_to_submap": validate_final_run_evidence(
            workspace, precision, "precision", holdout_mtime_ns,
            source_fingerprint, historical_repo,
        ),
    }
    final_config = workspace / "configs/final"
    odom_config = final_config / "odom_tuning_override.yaml"
    matcher_config = final_config / "matcher_override.yaml"
    if (
        read_ros_parameters(odom_config, ODOM_ROOT) != winner.get("effective_odom")
        or read_ros_parameters(matcher_config, MATCHER_ROOT)
        != winner.get("effective_matcher")
        or sha256(odom_config)
        != sha256(control / "artifacts/odom_tuning_override.yaml")
        or sha256(odom_config)
        != sha256(precision / "artifacts/odom_tuning_override.yaml")
        or sha256(matcher_config)
        != sha256(precision / "artifacts/submap_matcher_override.yaml")
        or winner.get("effective_runner") != {"snapshot_interval": 5}
    ):
        raise RuntimeError("final run parameters differ from the adopted c08 semantics")
    actual_ab_files = {
        item.name for item in evaluation_dir.iterdir()
        if item.is_file() and not item.is_symlink()
    } if evaluation_dir.is_dir() and not evaluation_dir.is_symlink() else set()
    if actual_ab_files != FINAL_AB_ARTIFACTS:
        raise RuntimeError(f"formal A/B artifact set differs: {sorted(actual_ab_files)}")
    evaluation = read_json(evaluation_path)
    hard_checks = validate_revalidation_evaluation(
        evaluation, control, precision,
        Path(source_fingerprint["glim_trajectory"]["path"]),
    )
    evaluator_mtime_ns = int(evaluator_provenance.get("current_evaluator_mtime_ns", -1))
    if evaluator_mtime_ns < 0 or evaluation_path.stat().st_mtime_ns < evaluator_mtime_ns:
        raise RuntimeError("formal A/B evaluation predates the fixed evaluator")
    ab_artifacts = hash_file_set(
        workspace, [evaluation_dir / name for name in sorted(FINAL_AB_ARTIFACTS)]
    )
    performed_at = time.time_ns() if performed_at_unix_ns is None else performed_at_unix_ns
    receipt = {
        "format": 1,
        "operation": "final-status-only-revalidation",
        "reason": FINAL_STATUS_REVALIDATION_REASON,
        "performed_at_unix_ns": performed_at,
        "workspace": str(workspace),
        "plan": {
            "path": str(plan_path.resolve()),
            "sha256": sha256(plan_path),
            "normalized_sha256": sha256(normalized_path),
            "stage_count": len(stage_ids),
            "stage_ids": stage_ids,
        },
        "previous_final_status": {
            "sha256": sha256(status_path),
            "identity": old_status_identity,
            "document": old_status,
        },
        "historical_workspace_provenance": historical_repo,
        "current_repository_provenance": current_repo_provenance,
        "evaluator_provenance": evaluator_provenance,
        "selection": {
            "final_stage": final_stage["id"],
            "selected_candidate": selected["id"],
            "candidate_adopted": True,
            "selected_semantics_sha256": sha256_bytes(
                json.dumps(selected, sort_keys=True, separators=(",", ":")).encode()
            ),
        },
        "holdout": {
            "sha256": sha256(holdout_path),
            "mtime_ns": holdout_mtime_ns,
            "selection_score_used_holdout": False,
            "opened_after_all_stages_completed": True,
            "retune_after_holdout_reveal_allowed": False,
        },
        "completed_evidence_sha256": completed_evidence,
        "candidate_and_config_chronology": chronology,
        "final_replay_command_provenance": commands,
        "final_run_immutable_evidence": final_runs,
        "formal_ab": {
            "evaluation_passed": True,
            "hard_check_count": len(hard_checks),
            "all_hard_checks_passed": True,
            "cadence_expected": 5,
            "cadence_recorded": "5",
            "cadence_artifact_matches": True,
            "artifacts": ab_artifacts,
        },
        "no_parameter_or_data_changes": True,
        "candidate_metrics_recomputed": False,
        "holdout_recomputed_or_reopened": False,
        "candidate_selection_changed": False,
        "replay_executed": False,
    }
    new_status = dict(old_status)
    new_status.update({
        "status": "ACCEPTED",
        "formal_ab_status": 0,
        "formal_ab_passed": True,
        "status_revalidated_without_replay": True,
    })
    new_status["status_revalidation"] = {
        "reason": FINAL_STATUS_REVALIDATION_REASON,
        "performed_at_unix_ns": performed_at,
        "previous_final_status_sha256": receipt["previous_final_status"]["sha256"],
        "receipt": str(final_root / FINAL_STATUS_REVALIDATION_RECEIPT),
    }
    return receipt, new_status, old_status_identity


def atomic_write_json(path: Path, value: dict[str, Any], replace: bool) -> None:
    if path.is_symlink() or (path.exists() and not replace):
        raise RuntimeError(f"refusing atomic JSON target: {path}")
    temporary = path.with_name(f".{path.name}.tmp")
    if temporary.exists() or temporary.is_symlink():
        raise RuntimeError(f"refusing stale atomic JSON temporary: {temporary}")
    rendered = (json.dumps(value, indent=2, sort_keys=True) + "\n").encode()
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(rendered)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        if temporary.exists() and not temporary.is_symlink():
            temporary.unlink()
        raise


def commit_final_status_revalidation(
    workspace: Path,
    receipt: dict[str, Any],
    new_status: dict[str, Any],
    old_status_identity: dict[str, int],
) -> dict[str, str]:
    final_root = workspace / "final"
    status_path = final_root / "FINAL_STATUS.json"
    receipt_path = final_root / FINAL_STATUS_REVALIDATION_RECEIPT
    if stat_identity(status_path) != old_status_identity:
        raise RuntimeError("FINAL_STATUS changed during status-only revalidation")
    if sha256(status_path) != receipt["previous_final_status"]["sha256"]:
        raise RuntimeError("FINAL_STATUS content changed during status-only revalidation")
    atomic_write_json(receipt_path, receipt, replace=False)
    receipt_hash = sha256(receipt_path)
    if stat_identity(status_path) != old_status_identity:
        raise RuntimeError(
            "FINAL_STATUS changed after receipt creation; status was not replaced"
        )
    new_status["status_revalidation"]["receipt_sha256"] = receipt_hash
    atomic_write_json(status_path, new_status, replace=True)
    return {
        "receipt": str(receipt_path),
        "receipt_sha256": receipt_hash,
        "final_status_sha256": sha256(status_path),
    }


def revalidate_final_status(
    workspace: Path, plan_path: Path, plan: dict[str, Any]
) -> dict[str, str]:
    start_repo_provenance = workspace_provenance()
    evaluator_provenance = evaluator_revalidation_provenance()
    receipt, new_status, old_identity = prepare_final_status_revalidation(
        workspace, plan_path, plan, evaluator_provenance, start_repo_provenance
    )
    if workspace_provenance() != start_repo_provenance:
        raise RuntimeError("repository changed during status-only revalidation")
    return commit_final_status_revalidation(
        workspace, receipt, new_status, old_identity
    )


def finalize(workspace: Path, plan: dict[str, Any], rate: float, dry_run: bool) -> None:
    source_fingerprint, repo_provenance = workspace_locked_inputs(workspace, plan)
    assert_dataset_source_fingerprint(plan, source_fingerprint)
    assert_repo_provenance(repo_provenance)
    final_stage = plan["stages"][-1]
    summary_path = workspace / "summaries" / f"{final_stage['id']}.json"
    if not summary_path.is_file():
        raise RuntimeError(f"complete all stages before finalization: {final_stage['id']}")
    summary = read_json(summary_path)
    selected_candidate = summary.get("selected_candidate")
    if not summary.get("completed") or not isinstance(selected_candidate, dict):
        raise RuntimeError("final stage is not complete")
    winner = selected_candidate
    candidate_adopted = summary.get("candidate_adopted") is True
    config_dir = workspace / "configs" / "final"
    odom_path = config_dir / "odom_tuning_override.yaml"
    matcher_path = config_dir / "matcher_override.yaml"
    write_yaml(odom_path, ros_yaml(ODOM_ROOT, winner["effective_odom"]))
    write_yaml(matcher_path, ros_yaml(MATCHER_ROOT, winner["effective_matcher"]))
    final_root = workspace / "final"
    control = final_root / "scan_to_scan"
    precision = final_root / "scan_to_submap"
    commands = final_runner_commands(
        plan, winner, control, precision, odom_path, matcher_path, rate
    )
    if dry_run:
        for command in commands:
            print(" ".join(subprocess.list2cmdline([item]) for item in command))
        return
    if final_root.exists():
        raise RuntimeError(f"refusing to overwrite finalization directory: {final_root}")
    final_root.mkdir(parents=True)
    (final_root / "commands.json").write_text(
        json.dumps({"commands": commands, "cwd": str(REPO)}, indent=2) + "\n",
        encoding="utf-8",
    )
    for command in commands:
        assert_dataset_source_fingerprint(plan, source_fingerprint)
        assert_repo_provenance(repo_provenance)
        minimum_free = int(plan["execution"]["minimum_free_gib"] * 1024**3)
        if shutil.disk_usage(workspace).free < minimum_free:
            raise RuntimeError(
                "refusing final replay below the plan's minimum free-space limit"
            )
        completed = subprocess.run(command, cwd=REPO, check=False)
        if completed.returncode != 0:
            raise RuntimeError(f"final replay failed with status {completed.returncode}")
        assert_dataset_source_fingerprint(plan, source_fingerprint)
        assert_repo_provenance(repo_provenance)
        validate_run_dataset_fingerprint(
            Path(command[command.index("--output") + 1]), source_fingerprint,
            repo_provenance,
        )
    assert_dataset_source_fingerprint(plan, source_fingerprint)
    assert_repo_provenance(repo_provenance)
    validate_run_dataset_fingerprint(control, source_fingerprint, repo_provenance)
    validate_run_dataset_fingerprint(precision, source_fingerprint, repo_provenance)
    scan = tuning_metrics(control, plan, "baseline", include_holdout=True)
    submap = tuning_metrics(precision, plan, "precision", include_holdout=True)
    assert_dataset_source_fingerprint(plan, source_fingerprint)
    assert_repo_provenance(repo_provenance)
    validate_run_dataset_fingerprint(control, source_fingerprint, repo_provenance)
    validate_run_dataset_fingerprint(precision, source_fingerprint, repo_provenance)
    holdout_path = final_root / "holdout_metrics.json"
    holdout_path.write_text(
        json.dumps(
            {
                "selection_score_used_holdout": False,
                "opened_after_all_stages_completed": True,
                "split_origin_equal": (
                    scan["method"]["split_time_origin_stamp_ns"]
                    == submap["method"]["split_time_origin_stamp_ns"]
                ),
                "scan_to_scan": {
                    "method": scan["method"], "holdout": scan["holdout"],
                    "full": scan["full"],
                    "holdout_low_motion": scan["holdout_low_motion"],
                },
                "scan_to_submap": {
                    "method": submap["method"], "holdout": submap["holdout"],
                    "full": submap["full"],
                    "holdout_low_motion": submap["holdout_low_motion"],
                },
            },
            indent=2,
            sort_keys=True,
        ) + "\n",
        encoding="utf-8",
    )
    glim_dir = Path(read_env(control / "run.env")["glim_dir"])
    assert_dataset_source_fingerprint(plan, source_fingerprint)
    assert_repo_provenance(repo_provenance)
    ab_command = [
        sys.executable, str(REPO / "tools/evaluate_lidar_imu_submap_ab.py"),
        "--control-run", str(control), "--precision-run", str(precision),
        "--glim-dir", str(glim_dir), "--sensor", plan["dataset"]["sensor"],
        "--output-dir", str(final_root / "ab_initial_pose"),
        "--label", plan["dataset"]["label"],
    ]
    completed = subprocess.run(ab_command, cwd=REPO, check=False)
    assert_dataset_source_fingerprint(plan, source_fingerprint)
    assert_repo_provenance(repo_provenance)
    validate_run_dataset_fingerprint(control, source_fingerprint, repo_provenance)
    validate_run_dataset_fingerprint(precision, source_fingerprint, repo_provenance)
    final_status = {
        "completed": True,
        "status": (
            "ACCEPTED" if candidate_adopted and completed.returncode == 0
            else "REPORTED_WITH_LIMITATIONS"
        ),
        "candidate_adopted": candidate_adopted,
        "adopted_winner": summary.get("winner"),
        "report_only_winner": summary.get("report_only_winner"),
        "formal_ab_status": completed.returncode,
        "formal_ab_passed": candidate_adopted and completed.returncode == 0,
        "retune_after_holdout_reveal_allowed": False,
        "holdout_metrics": str(holdout_path),
        "formal_ab_output": str(final_root / "ab_initial_pose"),
    }
    (final_root / "FINAL_STATUS.json").write_text(
        json.dumps(final_status, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if completed.returncode != 0 or not candidate_adopted:
        print(
            "FINAL REPORTED_WITH_LIMITATIONS: candidate adoption or formal A/B gates failed; "
            "retuning on this revealed holdout is prohibited",
            file=sys.stderr,
        )


def self_test() -> None:
    plan = load_plan(DEFAULT_PLAN)
    assert len(plan["stages"]) >= 5
    assert sum(len(stage["candidates"]) for stage in plan["stages"]) == 48
    assert plan["dataset"]["mid_yaw_policy"] == "fixed-bias-direct"
    assert plan["split_seconds"] == {
        "training": [[0, 30], [60, 90], [120, 180], [240, 270]],
        "holdout": [[30, 60], [90, 120], [180, 240], [270, 294.3]],
    }
    synthetic_control = REPO / "test_results/synthetic_status_control"
    synthetic_precision = REPO / "test_results/synthetic_status_precision"
    synthetic_glim = REPO / "test_results/synthetic_status_glim.txt"
    good_evaluation = {
        "passed": True,
        "configuration_contract": {"passed": True},
        "inputs": {
            "control_run": str(synthetic_control),
            "precision_run": str(synthetic_precision),
            "glim": str(synthetic_glim),
        },
        "tuning_semantics_contract": {
            "passed": True,
            "odometry_semantics_equal": True,
            "odometry_aux_snapshot_only": True,
            "snapshot_interval_record_valid": True,
            "snapshot_interval_recorded": "5",
            "snapshot_interval_expected": 5,
            "snapshot_interval_matches_artifact": True,
        },
        "checks": [{"category": "hard", "passed": True, "name": "synthetic"}],
    }
    assert len(validate_revalidation_evaluation(
        good_evaluation, synthetic_control, synthetic_precision, synthetic_glim
    )) == 1
    failed_evaluation = json.loads(json.dumps(good_evaluation))
    failed_evaluation["passed"] = False
    try:
        validate_revalidation_evaluation(
            failed_evaluation, synthetic_control, synthetic_precision, synthetic_glim
        )
    except RuntimeError:
        pass
    else:
        raise AssertionError("failed formal A/B was accepted for status revalidation")
    good_holdout = {
        "selection_score_used_holdout": False,
        "opened_after_all_stages_completed": True,
        "split_origin_equal": True,
        "scan_to_scan": {},
        "scan_to_submap": {},
    }
    validate_revalidation_holdout(good_holdout)
    changed_holdout = dict(good_holdout, selection_score_used_holdout=True)
    try:
        validate_revalidation_holdout(changed_holdout)
    except RuntimeError:
        pass
    else:
        raise AssertionError("changed holdout selection policy was accepted")
    evaluator_provenance = {
        "path": str(REPO / "tools/evaluate_lidar_imu_submap_ab.py"),
        "current_evaluator_sha256": sha256(
            REPO / "tools/evaluate_lidar_imu_submap_ab.py"
        ),
        "git_head_preimage_sha256": "a" * 64,
        "current_diff_from_git_head_sha256": "b" * 64,
        "historical_runtime_evaluator_sha256": None,
        "historical_runtime_hash_recoverable": False,
        "historical_runtime_hash_recovery_detail": "not recorded",
    }
    current_provenance = {
        "tuning_tool_sha256": sha256(Path(__file__).resolve()),
        "git_head_sha": "a" * 40,
        "git_dirty_diff_sha256": "b" * 64,
        "git_status_sha256": "c" * 64,
    }
    validate_revalidation_provenance(evaluator_provenance, current_provenance)
    missing_provenance = dict(current_provenance)
    missing_provenance.pop("git_status_sha256")
    try:
        validate_revalidation_provenance(evaluator_provenance, missing_provenance)
    except RuntimeError:
        pass
    else:
        raise AssertionError("missing revalidation provenance was accepted")
    forbidden_status_only_calls = {
        "execute_stage", "final_runner_commands", "finalize", "runner_command",
        "tuning_metrics",
    }
    assert not forbidden_status_only_calls.intersection(
        prepare_final_status_revalidation.__code__.co_names
    )
    assert not forbidden_status_only_calls.intersection(
        revalidate_final_status.__code__.co_names
    )
    with __import__("tempfile").TemporaryDirectory(
        dir=REPO / "test_results"
    ) as status_directory:
        status_root = Path(status_directory)
        candidate_tree = status_root / "candidate"
        candidate_tree.mkdir()
        candidate_file = candidate_tree / "metrics.json"
        candidate_file.write_text("{}\n", encoding="utf-8")
        cutoff_ns = candidate_file.stat().st_mtime_ns
        audit_tree_before(candidate_tree, cutoff_ns, "synthetic candidate")
        time.sleep(0.002)
        candidate_file.write_text('{"changed": true}\n', encoding="utf-8")
        try:
            audit_tree_before(candidate_tree, cutoff_ns, "synthetic candidate")
        except RuntimeError:
            pass
        else:
            raise AssertionError("candidate changed after holdout was accepted")
        final_root = status_root / "final"
        final_root.mkdir()
        status_path = final_root / "FINAL_STATUS.json"
        old_status = {"status": "REPORTED_WITH_LIMITATIONS"}
        status_path.write_text(
            json.dumps(old_status, sort_keys=True) + "\n", encoding="utf-8"
        )
        old_identity = stat_identity(status_path)
        receipt = {
            "previous_final_status": {"sha256": sha256(status_path)},
            "replay_executed": False,
        }
        new_status = {
            "status": "ACCEPTED",
            "status_revalidation": {"receipt": "synthetic"},
        }
        committed = commit_final_status_revalidation(
            status_root, receipt, new_status, old_identity
        )
        assert read_json(status_path)["status"] == "ACCEPTED"
        assert Path(committed["receipt"]).is_file()
    sample_time = np.asarray([0.0, 1.0, 2.0, 3.0])
    mask = selected_intervals(sample_time, [[0.0, 2.0], [3.0, 4.0]])
    assert mask.tolist() == [True, True, False, True]
    xy = np.asarray([[0.0, 0.0], [1.0, 0.0], [20.0, 0.0], [21.0, 0.0]])
    assert interval_path(xy, mask) == 1.0
    with __import__("tempfile").TemporaryDirectory(dir=REPO / "test_results") as directory:
        workspace = safe_workspace(Path(directory) / "workspace")
        initialize_workspace(workspace, DEFAULT_PLAN, plan)
        marker = workspace / WORKSPACE_MARKER
        original_marker = read_json(marker)
        assert original_marker["first_run_provenance"] == workspace_provenance()
        changed_marker = dict(original_marker)
        changed_marker["first_run_provenance"] = dict(
            original_marker["first_run_provenance"], tuning_tool_sha256="0" * 64
        )
        marker.write_text(
            json.dumps(changed_marker, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        try:
            initialize_workspace(workspace, DEFAULT_PLAN, plan)
        except RuntimeError:
            pass
        else:
            raise AssertionError("workspace resume accepted changed tool provenance")
        marker.write_text(
            json.dumps(original_marker, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        initialize_workspace(workspace, DEFAULT_PLAN, plan)
        source_fingerprint, repo_provenance = workspace_locked_inputs(workspace, plan)
        changed_repo_provenance = dict(repo_provenance, git_head_sha="0" * 40)
        try:
            assert_repo_provenance(changed_repo_provenance)
        except RuntimeError:
            pass
        else:
            raise AssertionError("mutated repository provenance was accepted")
        mutated_marker = json.loads(json.dumps(original_marker))
        mutated_marker["dataset_source_fingerprint"]["bag_metadata_sha256"] = "0" * 64
        marker.write_text(
            json.dumps(mutated_marker, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        try:
            workspace_locked_inputs(workspace, plan)
        except RuntimeError:
            pass
        else:
            raise AssertionError("mutated dataset fingerprint marker was accepted")
        marker.write_text(
            json.dumps(original_marker, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        synthetic = Path(directory) / "synthetic_source"
        synthetic_bag = synthetic / "bag"
        synthetic_glim = synthetic / "glim"
        synthetic_bag.mkdir(parents=True)
        synthetic_glim.mkdir()
        (synthetic_bag / "metadata.yaml").write_text("metadata\n", encoding="utf-8")
        synthetic_storage = synthetic_bag / "data_0.mcap"
        synthetic_storage.write_bytes(b"first")
        (synthetic_glim / "traj_lidar.txt").write_text("trajectory\n", encoding="utf-8")
        synthetic_imu = synthetic / "imu.yaml"
        synthetic_odom = synthetic / "odom.yaml"
        synthetic_imu.write_text("imu\n", encoding="utf-8")
        synthetic_odom.write_text("odom\n", encoding="utf-8")
        synthetic_plan = json.loads(json.dumps(plan))
        synthetic_plan["dataset"].update({
            "id": "synthetic",
            "bag": str(synthetic_bag),
            "glim_dir": str(synthetic_glim),
            "imu_profile": str(synthetic_imu),
            "odom_profile": str(synthetic_odom),
        })
        synthetic_fingerprint = dataset_source_fingerprint(synthetic_plan)
        synthetic_storage.write_bytes(b"second")
        try:
            assert_dataset_source_fingerprint(synthetic_plan, synthetic_fingerprint)
        except RuntimeError:
            pass
        else:
            raise AssertionError("mutated dataset storage was accepted")
        synthetic_run = synthetic / "run"
        synthetic_run.mkdir()
        storage = source_fingerprint["bag_storage"]
        valid_env = {
            "input_bag_metadata_sha256": source_fingerprint["bag_metadata_sha256"],
            "input_bag_storage_files": ",".join(item["name"] for item in storage),
            "input_bag_storage_sha256": ",".join(item["sha256"] for item in storage),
            "glim_traj_sha256": source_fingerprint["glim_trajectory"]["sha256"],
            "imu_param_sha256": source_fingerprint["imu_profile"]["sha256"],
            "odom_override_sha256": source_fingerprint["odom_profile"]["sha256"],
            "git_head_sha": repo_provenance["git_head_sha"],
            "git_dirty_diff_sha256": repo_provenance["git_dirty_diff_sha256"],
            "git_status_sha256": repo_provenance["git_status_sha256"],
        }
        (synthetic_run / "run.env").write_text(
            "".join(f"{key}={value}\n" for key, value in valid_env.items()),
            encoding="utf-8",
        )
        validate_run_dataset_fingerprint(
            synthetic_run, source_fingerprint, repo_provenance
        )
        valid_env["glim_traj_sha256"] = "0" * 64
        (synthetic_run / "run.env").write_text(
            "".join(f"{key}={value}\n" for key, value in valid_env.items()),
            encoding="utf-8",
        )
        try:
            validate_run_dataset_fingerprint(
                synthetic_run, source_fingerprint, repo_provenance
            )
        except RuntimeError:
            pass
        else:
            raise AssertionError("mutated reused-run fingerprint was accepted")
        safe_fallback = {
            "metrics": {
                "hard_gates": {
                    "runtime_passed": True,
                    "association_coverage_at_least_0_95": True,
                    "stop_classifier_passed": False,
                }
            }
        }
        assert stop_fallback_safe(safe_fallback)
        unsafe_fallback = json.loads(json.dumps(safe_fallback))
        unsafe_fallback["metrics"]["hard_gates"]["runtime_passed"] = False
        assert not stop_fallback_safe(unsafe_fallback)
        extraction_failure = json.loads(json.dumps(safe_fallback))
        extraction_failure["metrics"]["extraction_error"] = "missing anchor"
        assert not stop_fallback_safe(extraction_failure)
        source = workspace / "source.json"
        source.write_text('{"value": 1}\n', encoding="utf-8")
        validate_source_hashes(workspace, {"source.json": sha256(source)})
        source.write_text('{"value": 2}\n', encoding="utf-8")
        try:
            validate_source_hashes(workspace, {"source.json": "0" * 64})
        except RuntimeError:
            pass
        else:
            raise AssertionError("changed reconciliation source hash was accepted")
        d00_semantics = {
            "stop.speed_thr_mps": 0.15,
            "stop.hold_sec": 0.5,
            "lidar_odom.smoother.zupt.enable": False,
        }
        validate_reconciled_semantics(
            "dz_safe", dict(d00_semantics), d00_semantics
        )
        z_on = dict(
            d00_semantics,
            **{
                "lidar_odom.smoother.zupt.enable": True,
                "lidar_odom.smoother.zupt.w_trans": 10.0,
                "lidar_odom.smoother.zupt.w_yaw": 10.0,
            },
        )
        validate_reconciled_semantics("z01", z_on, d00_semantics, z_on, True)
        try:
            validate_reconciled_semantics("z01", z_on, d00_semantics, z_on, False)
        except RuntimeError:
            pass
        else:
            raise AssertionError("ZUPT-on semantics without own classifier were accepted")
        first = plan["stages"][0]
        manifest = materialize_candidate(
            workspace, plan, first, first["candidates"][0]
        )
        assert Path(manifest["odom_path"]).is_file()
        command = runner_command(
            workspace, plan, first, first["candidates"][0], manifest, 1.0
        )
        assert "fixed-bias-direct" in command
        assert str(dataset_path(plan["dataset"]["bag"])) in command
        expected_biases = [0.006959692, 0.00635, 0.00665, 0.00727, 0.00757]
        bias_stage = stage_by_id(plan, "fixed_bias")
        assert [
            candidate["odom"]["gyro_bias.initial_bg_rad_s"]
            for candidate in bias_stage["candidates"]
        ] == expected_biases
        # Prove runner-only cadence is inherited independently of each later
        # stage's legacy snapshot_interval default and reaches final replay.
        cadence_manifest = None
        for planned_stage in plan["stages"]:
            control_id = plan["selection_policy"]["control_candidates"][
                planned_stage["id"]
            ]
            selected_id = "r05" if planned_stage["id"] == "snapshot_cadence" else control_id
            selected = next(
                item for item in planned_stage["candidates"] if item["id"] == selected_id
            )
            selected_manifest = materialize_candidate(
                workspace, plan, planned_stage, selected,
                allow_dry_run_dependency=True,
            )
            (workspace / "summaries" / f"{planned_stage['id']}.json").write_text(
                json.dumps({
                    "completed": False,
                    "dry_run": True,
                    "winner": {
                        "id": selected_id,
                        "effective_odom": selected_manifest["effective_odom"],
                        "effective_matcher": selected_manifest["effective_matcher"],
                        "effective_runner": selected_manifest["effective_runner"],
                    },
                }, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            if planned_stage["id"] == "snapshot_cadence":
                cadence_manifest = selected_manifest
            if planned_stage["id"] == "submap_geometry":
                cadence_command = runner_command(
                    workspace, plan, planned_stage, selected, selected_manifest, 1.0
                )
                index = cadence_command.index("--snapshot-interval")
                assert cadence_command[index + 1] == "5"
                break
        assert cadence_manifest is not None
        final_commands = final_runner_commands(
            plan,
            {
                "effective_runner": cadence_manifest["effective_runner"],
                "effective_odom": cadence_manifest["effective_odom"],
                "effective_matcher": cadence_manifest["effective_matcher"],
            },
            workspace / "final/control", workspace / "final/precision",
            Path(cadence_manifest["odom_path"]), Path(cadence_manifest["matcher_path"]),
            1.0,
        )
        precision_command = final_commands[1]
        index = precision_command.index("--snapshot-interval")
        assert precision_command[index + 1] == "5"
        reference_metrics = {
            "training": {
                "position_error_m": {"rmse": 1.0},
                "yaw_error_deg": {"rmse": 2.0},
                "path_ratio_estimate_over_reference": 1.0,
            },
            "training_rpe_10m": {
                "translation_error_m": {"rmse": 0.5},
                "yaw_error_deg": {"rmse": 1.0},
            },
        }
        (workspace / "summaries/stop_detection.json").write_text(
            json.dumps({
                "winner": {"id": "d00", "metrics": reference_metrics}
            }, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        known_bad_precision = {
            "training": {
                "position_error_m": {"rmse": 2.0},
                "yaw_error_deg": {"rmse": 4.0},
                "path_ratio_estimate_over_reference": 1.20,
            },
            "training_rpe_10m": {
                "translation_error_m": {"rmse": 1.0},
                "yaw_error_deg": {"rmse": 2.0},
            },
            "hard_gates": {"runtime_structural_and_coverage": True},
            "eligible": True,
        }
        intermediate_metrics = json.loads(json.dumps(known_bad_precision))
        cadence_stage = stage_by_id(plan, "snapshot_cadence")
        apply_selection_hard_gates(
            workspace, plan, cadence_stage, cadence_stage["candidates"][0],
            intermediate_metrics,
        )
        assert intermediate_metrics["eligible"] is True
        assert intermediate_metrics["selection_gate_reference"]["kind"] == (
            "final_adoption_gates_deferred"
        )
        final_metrics = json.loads(json.dumps(known_bad_precision))
        final_stage = stage_by_id(plan, plan["selection_policy"]["final_precision_stage"])
        apply_selection_hard_gates(
            workspace, plan, final_stage, final_stage["candidates"][0], final_metrics
        )
        assert final_metrics["eligible"] is False
        assert final_metrics["hard_gates"]["submap_path_ratio_max"] is False
        assert final_metrics["hard_gates"]["submap_xy_nonregression"] is False
        report_rows = []
        for candidate_id, score in (("c00", 2.0), ("c02", 1.0)):
            item_metrics = json.loads(json.dumps(final_metrics))
            item_metrics["score"] = score
            item_metrics["structural_metrics"] = {"matcher_processing_p99_ms": 10.0}
            item_metrics["registration_density"] = 1.0
            item_metrics["training_low_motion"] = {"excess_path_m": 0.0}
            report_rows.append({
                "id": candidate_id,
                "eligible": False,
                "score": score,
                "effective_odom": {},
                "effective_matcher": {},
                "effective_runner": {"snapshot_interval": 2},
                "run": str(workspace / "runs/submap_correction" / candidate_id),
                "bag_retained": True,
                "metrics": item_metrics,
                "changed_parameter_count": 0,
            })
        report_summary = write_stage_summary(
            workspace, plan, final_stage, report_rows
        )
        assert report_summary["winner"] is None
        assert report_summary["candidate_adopted"] is False
        assert report_summary["selection_status"] == "REPORTED_WITH_LIMITATIONS"
        assert report_summary["report_only_winner"]["id"] == "c02"
        assert report_summary["selected_candidate"]["id"] == "c02"
        run = workspace / "runs" / first["id"] / first["candidates"][0]["id"]
        bag = run / "localization_output"
        bag.mkdir(parents=True)
        (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
        (bag / "localization_output_0.mcap").write_bytes(b"MCAP-test")
        prune_rejected_bag(workspace, run, "self-test")
        assert not bag.exists()
        assert (run / "pruned_localization_output.json").is_file()
        unsafe = run / "localization_output"
        unsafe.mkdir()
        (unsafe / "metadata.yaml").write_text("x\n")
        (unsafe / "unexpected.txt").write_text("keep\n")
        try:
            prune_rejected_bag(workspace, run, "must fail")
        except RuntimeError:
            pass
        else:
            raise AssertionError("unexpected bag content was pruned")
        assert (unsafe / "unexpected.txt").is_file()


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--plan", type=Path, default=DEFAULT_PLAN)
    result.add_argument("--workspace", type=Path)
    result.add_argument("--stage", help="stage id, or 'all'")
    result.add_argument("--rate", type=float, default=1.0)
    result.add_argument("--execute", action="store_true")
    result.add_argument("--dry-run", action="store_true")
    result.add_argument("--finalize", action="store_true")
    result.add_argument("--revalidate-final-status", action="store_true")
    result.add_argument("--migrate-workspace", action="store_true")
    result.add_argument("--self-test", action="store_true")
    return result


def main() -> int:
    args = parser().parse_args()
    if args.self_test:
        self_test()
        print("PASS: MID-360 tuning infrastructure self-test")
        return 0
    if args.workspace is None or (
        args.stage is None and not args.finalize and not args.revalidate_final_status
        and not args.migrate_workspace
    ):
        raise SystemExit(
            "--workspace and --stage, --finalize, --revalidate-final-status, "
            "or --migrate-workspace are required"
        )
    if sum((
        args.stage is not None, args.finalize, args.revalidate_final_status,
        args.migrate_workspace,
    )) != 1:
        raise SystemExit(
            "--stage, --finalize, --revalidate-final-status, and "
            "--migrate-workspace are mutually exclusive"
        )
    status_only = args.migrate_workspace or args.revalidate_final_status
    if not status_only and args.execute == args.dry_run:
        raise SystemExit("select exactly one of --execute or --dry-run")
    if status_only and (args.execute or args.dry_run):
        raise SystemExit(
            "workspace migration/status revalidation are explicit mutations; "
            "omit --execute/--dry-run"
        )
    if not math.isfinite(args.rate) or args.rate <= 0.0:
        raise SystemExit("--rate must be a positive finite number")
    plan_path = args.plan.resolve()
    plan = load_plan(plan_path)
    workspace = safe_workspace(args.workspace)
    if args.migrate_workspace:
        receipt = migrate_workspace(workspace, plan_path, plan)
        print(
            f"MIGRATED: winner={receipt['reconciliation_winner']['id']} "
            f"receipt={workspace / MIGRATION_RECEIPT}"
        )
        return 0
    if args.revalidate_final_status:
        result = revalidate_final_status(workspace, plan_path, plan)
        print(
            f"REVALIDATED: status=ACCEPTED receipt={result['receipt']} "
            f"final_status_sha256={result['final_status_sha256']}"
        )
        return 0
    initialize_workspace(workspace, plan_path, plan)
    if args.finalize:
        finalize(workspace, plan, args.rate, args.dry_run)
        return 0
    stages = plan["stages"] if args.stage == "all" else [stage_by_id(plan, args.stage)]
    for stage in stages:
        summary = execute_stage(workspace, plan, stage, args.rate, args.dry_run)
        if not args.dry_run:
            if stage["id"] == "zupt":
                coupled_dz_reconciliation(workspace)
                validate_post_z_reconciliation(workspace)
                summary = read_json(workspace / "summaries/zupt.json")
            winner = summary["selected_candidate"]
            print(
                f"{stage['id']}: selected={winner['id']} score={winner['score']:.6f} "
                f"bag={winner['bag_retained']} status={summary['selection_status']} "
                f"candidate_adopted={str(summary['candidate_adopted']).lower()}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
