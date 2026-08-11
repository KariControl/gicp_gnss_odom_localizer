#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Pool two or more isolated-precision GLIM A/B result files."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def pooled_rmse(items: list[dict[str, Any]]) -> float:
    count = sum(int(item["count"]) for item in items)
    if count == 0:
        return math.nan
    return math.sqrt(
        sum(int(item["count"]) * float(item["rmse"]) ** 2 for item in items) / count
    )


def comparison(old: float, new: float) -> dict[str, float]:
    return {
        "old": old,
        "new": new,
        "improvement_percent": (1.0 - new / old) * 100.0 if old else -math.inf,
    }


def aggregate(results: list[dict[str, Any]]) -> dict[str, Any]:
    def pooled(group: str, key: str) -> dict[str, float]:
        old_items = []
        new_items = []
        for result in results:
            old_evaluation = (
                "precision_raw" if group == "local_comparison" else "precision_existing"
            )
            new_evaluation = (
                "precision_local" if group == "local_comparison" else "precision_global"
            )
            if key == "fixed_xy_rmse_m":
                old_items.append(
                    result["evaluations"][old_evaluation]["fixed_common_xy_error_m"]
                )
                new_items.append(
                    result["evaluations"][new_evaluation]["fixed_common_xy_error_m"]
                )
            elif key == "yaw_rmse_deg":
                old_items.append(
                    result["evaluations"][old_evaluation]["common_yaw_offset_error_deg"]
                )
                new_items.append(
                    result["evaluations"][new_evaluation]["common_yaw_offset_error_deg"]
                )
            elif key == "full_shape_xy_rmse_m":
                old_items.append(
                    result["evaluations"][old_evaluation]["full_shape_independent_se2"]
                    ["position_error_m"]
                )
                new_items.append(
                    result["evaluations"][new_evaluation]["full_shape_independent_se2"]
                    ["position_error_m"]
                )
            else:
                old_items.append(
                    result["evaluations"][old_evaluation]["full_shape_independent_se2"]
                    ["yaw_error_deg"]
                )
                new_items.append(
                    result["evaluations"][new_evaluation]["full_shape_independent_se2"]
                    ["yaw_error_deg"]
                )
        return comparison(pooled_rmse(old_items), pooled_rmse(new_items))

    combined = {
        "local_fixed_xy_rmse_m": pooled("local_comparison", "fixed_xy_rmse_m"),
        "local_full_shape_xy_rmse_m": pooled(
            "local_comparison", "full_shape_xy_rmse_m"
        ),
        "local_yaw_rmse_deg": pooled("local_comparison", "yaw_rmse_deg"),
        "global_fixed_xy_rmse_m": pooled("global_comparison", "fixed_xy_rmse_m"),
        "global_full_shape_xy_rmse_m": pooled(
            "global_comparison", "full_shape_xy_rmse_m"
        ),
        "global_yaw_rmse_deg": pooled("global_comparison", "yaw_rmse_deg"),
        "local_rpe": {},
    }
    for distance in ("10", "50", "100"):
        old_translation = [
            result["evaluations"]["precision_raw"]["rpe"][distance][
                "translation_error_m"
            ]
            for result in results
        ]
        new_translation = [
            result["evaluations"]["precision_local"]["rpe"][distance][
                "translation_error_m"
            ]
            for result in results
        ]
        old_yaw = [
            result["evaluations"]["precision_raw"]["rpe"][distance]["yaw_error_deg"]
            for result in results
        ]
        new_yaw = [
            result["evaluations"]["precision_local"]["rpe"][distance]["yaw_error_deg"]
            for result in results
        ]
        combined["local_rpe"][distance] = {
            "translation_rmse_m": comparison(
                pooled_rmse(old_translation), pooled_rmse(new_translation)
            ),
            "yaw_rmse_deg": comparison(pooled_rmse(old_yaw), pooled_rmse(new_yaw)),
        }

    outage_old = []
    outage_new = []
    outage_yaw_old = []
    outage_yaw_new = []
    for result in results:
        outage = result.get("outage")
        if not outage or not outage.get("precision_existing") or not outage.get("precision_global"):
            continue
        outage_old.append(outage["precision_existing"]["xy_error_m"])
        outage_new.append(outage["precision_global"]["xy_error_m"])
        outage_yaw_old.append(outage["precision_existing"]["yaw_error_deg"])
        outage_yaw_new.append(outage["precision_global"]["yaw_error_deg"])
    combined_outage = None
    if outage_old:
        combined_outage = {
            "xy_rmse_m": comparison(pooled_rmse(outage_old), pooled_rmse(outage_new)),
            "yaw_rmse_deg": comparison(
                pooled_rmse(outage_yaw_old), pooled_rmse(outage_yaw_new)
            ),
        }

    checks = []

    def add(name: str, passed: bool, detail: str, category: str = "hard") -> None:
        checks.append(
            {"name": name, "passed": bool(passed), "detail": detail, "category": category}
        )

    add(
        "all bag-level hard gates pass",
        all(result["passed"] for result in results),
        repr([result["passed"] for result in results]),
    )
    add(
        "combined local fixed-frame XY improves >=20%",
        combined["local_fixed_xy_rmse_m"]["improvement_percent"] >= 20.0,
        f"{combined['local_fixed_xy_rmse_m']['improvement_percent']:.3f}%",
    )
    add(
        "combined local full-shape XY improves >=20%",
        combined["local_full_shape_xy_rmse_m"]["improvement_percent"] >= 20.0,
        f"{combined['local_full_shape_xy_rmse_m']['improvement_percent']:.3f}%",
    )
    local_yaw_limit = combined["local_yaw_rmse_deg"]["old"] * 1.05 + 0.05
    add(
        "combined local yaw non-regression",
        combined["local_yaw_rmse_deg"]["new"] <= local_yaw_limit,
        f"{combined['local_yaw_rmse_deg']['new']:.6f} <= {local_yaw_limit:.6f}",
    )
    global_xy_limit = combined["global_fixed_xy_rmse_m"]["old"] * 1.05 + 0.05
    global_full_xy_limit = (
        combined["global_full_shape_xy_rmse_m"]["old"] * 1.05 + 0.05
    )
    global_yaw_limit = combined["global_yaw_rmse_deg"]["old"] * 1.05 + 0.05
    add(
        "combined precision global XY non-regression",
        combined["global_fixed_xy_rmse_m"]["new"] <= global_xy_limit,
        f"{combined['global_fixed_xy_rmse_m']['new']:.6f} <= {global_xy_limit:.6f}",
    )
    add(
        "combined precision global full-shape XY non-regression",
        combined["global_full_shape_xy_rmse_m"]["new"] <= global_full_xy_limit,
        f"{combined['global_full_shape_xy_rmse_m']['new']:.6f} <= "
        f"{global_full_xy_limit:.6f}",
    )
    add(
        "combined precision global yaw non-regression",
        combined["global_yaw_rmse_deg"]["new"] <= global_yaw_limit,
        f"{combined['global_yaw_rmse_deg']['new']:.6f} <= {global_yaw_limit:.6f}",
    )
    if combined_outage is None:
        add("combined outage metrics available", False, "no common outage")
    else:
        outage_xy_limit = combined_outage["xy_rmse_m"]["old"] * 1.05 + 0.05
        outage_yaw_limit = combined_outage["yaw_rmse_deg"]["old"] * 1.05 + 0.05
        add(
            "combined outage precision global XY non-regression",
            combined_outage["xy_rmse_m"]["new"] <= outage_xy_limit,
            f"{combined_outage['xy_rmse_m']['new']:.6f} <= {outage_xy_limit:.6f}",
        )
        add(
            "combined outage precision global yaw non-regression",
            combined_outage["yaw_rmse_deg"]["new"] <= outage_yaw_limit,
            f"{combined_outage['yaw_rmse_deg']['new']:.6f} <= {outage_yaw_limit:.6f}",
        )
        add(
            "combined outage precision global XY improves >=10%",
            combined_outage["xy_rmse_m"]["improvement_percent"] >= 10.0,
            f"{combined_outage['xy_rmse_m']['improvement_percent']:.3f}%",
        )
    failed = [item for item in checks if item["category"] == "hard" and not item["passed"]]
    return {
        "labels": [result["label"] for result in results],
        "passed": not failed,
        "combined": combined,
        "combined_outage": combined_outage,
        "checks": checks,
        "hard_gate_count": sum(item["category"] == "hard" for item in checks),
        "failed_hard_gate_count": len(failed),
    }


def fmt(item: dict[str, float], unit: str) -> str:
    return (
        f"{item['old']:.4f} -> {item['new']:.4f} {unit} "
        f"({item['improvement_percent']:+.2f}%)"
    )


def markdown(result: dict[str, Any]) -> str:
    combined = result["combined"]
    lines = [
        "# Isolated precision aggregate",
        "",
        f"- result: **{'PASS' if result['passed'] else 'FAIL'}**",
        f"- bags: {', '.join(result['labels'])}",
        f"- local XY: {fmt(combined['local_fixed_xy_rmse_m'], 'm')}",
        f"- local full-shape XY: {fmt(combined['local_full_shape_xy_rmse_m'], 'm')}",
        f"- local yaw: {fmt(combined['local_yaw_rmse_deg'], 'deg')}",
        f"- global XY: {fmt(combined['global_fixed_xy_rmse_m'], 'm')}",
        f"- global full-shape XY: {fmt(combined['global_full_shape_xy_rmse_m'], 'm')}",
        f"- global yaw: {fmt(combined['global_yaw_rmse_deg'], 'deg')}",
        "",
        "## Checks",
        "",
    ]
    for item in result["checks"]:
        mark = "PASS" if item["passed"] else ("WARN" if item["category"] == "warn" else "FAIL")
        lines.append(f"- {mark}: {item['name']} — {item['detail']}")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pair-json", type=Path, action="append", required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-markdown", type=Path, required=True)
    args = parser.parse_args()
    if len(args.pair_json) < 2:
        parser.error("provide at least two --pair-json files")
    results = [json.loads(path.read_text(encoding="utf-8")) for path in args.pair_json]
    result = aggregate(results)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_markdown.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    args.output_markdown.write_text(markdown(result), encoding="utf-8")
    print(f"{'PASS' if result['passed'] else 'FAIL'}: {args.output_markdown.resolve()}")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
