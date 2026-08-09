#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Static checks for the CPU-only Autoware LSim Docker profile."""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
ERRORS: list[str] = []


def fail(message: str) -> None:
    ERRORS.append(message)


def load_yaml(path: Path) -> dict:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        fail(f"cannot parse {path.relative_to(ROOT)}: {exc}")
        return {}
    if not isinstance(data, dict):
        fail(f"top-level YAML is not a mapping: {path.relative_to(ROOT)}")
        return {}
    return data


def check_compose() -> None:
    compose_path = ROOT / "docker/autoware_lsim/compose.yaml"
    rviz_path = ROOT / "docker/autoware_lsim/compose.rviz.yaml"
    for path in (compose_path, rviz_path):
        if not path.is_file():
            fail(f"missing Docker Compose file: {path.relative_to(ROOT)}")

    compose = load_yaml(compose_path)
    service = compose.get("services", {}).get("lsim", {})
    if not isinstance(service, dict):
        fail("compose.yaml does not define services.lsim")
        return

    build = service.get("build", {})
    if build.get("context") != "../..":
        fail("Docker build context must remain the repository root (../..)")
    if build.get("dockerfile") != "docker/autoware_lsim/Dockerfile":
        fail("Dockerfile path changed unexpectedly")
    if service.get("network_mode") != "host":
        fail("LSim service must use host networking for single-host DDS")
    if service.get("ipc") != "host":
        fail("LSim service must use host IPC")
    if service.get("privileged") is not True:
        fail("LSim service must remain privileged for the official entrypoint DDS tuning")

    environment = service.get("environment", {})
    required_environment = {
        "BAG_PATH": "/bags/input",
        "OUTPUT_ROOT": "/output",
        "RVIZ": "${RVIZ:-false}",
        "USE_GNSS": "${USE_GNSS:-false}",
        "TRACKING_MODE": "${TRACKING_MODE:-scan_to_scan}",
        "AUTO_INITIAL_POSE": "${AUTO_INITIAL_POSE:-true}",
        "RECORD_OUTPUT": "${RECORD_OUTPUT:-true}",
    }
    for key, expected in required_environment.items():
        if environment.get(key) != expected:
            fail(f"compose environment {key} must be {expected!r}")

    mounts = {
        volume.get("target"): volume
        for volume in service.get("volumes", [])
        if isinstance(volume, dict)
    }
    if mounts.get("/bags/input", {}).get("read_only") is not True:
        fail("input rosbag must be mounted read-only at /bags/input")
    if "/output" not in mounts:
        fail("output directory mount is missing")

    compose_text = compose_path.read_text(encoding="utf-8")
    forbidden_gpu_tokens = ("gpus:", "runtime: nvidia", "NVIDIA_VISIBLE_DEVICES")
    for token in forbidden_gpu_tokens:
        if token in compose_text:
            fail(f"CPU-only compose unexpectedly contains GPU token: {token}")

    rviz = load_yaml(rviz_path).get("services", {}).get("lsim", {})
    rviz_environment = rviz.get("environment", {}) if isinstance(rviz, dict) else {}
    if rviz_environment.get("LIBGL_ALWAYS_SOFTWARE") != "1":
        fail("RViz overlay must force software rendering")
    if rviz_environment.get("RVIZ") != "true":
        fail("RViz overlay must enable RVIZ")


def check_dockerfile() -> None:
    path = ROOT / "docker/autoware_lsim/Dockerfile"
    if not path.is_file():
        fail("Dockerfile is missing")
        return
    text = path.read_text(encoding="utf-8")
    for token in (
        "universe-devel-jazzy-1.9.0",
        "source /opt/autoware/setup.bash",
        "rosdep install",
        "colcon build",
        "-DBUILD_TESTING=OFF",
        "/opt/gicp_gnss_odom_localizer",
        "container_entrypoint.sh",
        "play_localization_bag.sh",
    ):
        if token not in text:
            fail(f"Dockerfile required token missing: {token}")
    for token in ("universe-devel-cuda", "--gpus", "nvidia"):
        if token in text.lower():
            fail(f"CPU-only Dockerfile contains GPU-specific token: {token}")


def check_container_runner() -> None:
    path = ROOT / "docker/autoware_lsim/container_entrypoint.sh"
    if not path.is_file():
        fail("container runner is missing")
        return
    text = path.read_text(encoding="utf-8")
    for token in (
        "autoware_lsim_localization.launch.py",
        "play_localization_bag.sh",
        "ros2 bag record",
        "ros2 topic echo --once /localization/gyro_lidar_odom",
        "ros2 topic pub --once /initialpose",
        "param_scan_to_submap.yaml",
        "TF_POLICY=isolate-all requires LAUNCH_VEHICLE=true",
    ):
        if token not in text:
            fail(f"container runner required behavior missing: {token}")


def check_host_wrapper() -> None:
    wrapper = ROOT / "script/run_autoware_lsim_docker.sh"
    if not wrapper.is_file():
        fail("host Docker wrapper is missing")
        return
    with tempfile.TemporaryDirectory(prefix="autoware_lsim_docker_check_") as directory:
        temporary = Path(directory)
        bag = temporary / "bag"
        output = temporary / "output"
        bag.mkdir()
        output.mkdir()
        environment = dict(os.environ)
        environment.setdefault("DISPLAY", ":99")
        result = subprocess.run(
            [
                str(wrapper),
                "--dry-run",
                "--bag",
                str(bag),
                "--output",
                str(output),
                "--points",
                "/recorded/points",
                "--imu",
                "/recorded/imu",
                "--nmea",
                "/recorded/nmea",
                "--tracking-mode",
                "scan_to_submap",
                "--rviz",
            ],
            cwd=ROOT,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        if result.returncode != 0:
            fail("Docker wrapper dry run failed:\n" + result.stdout + result.stderr)
            return
        for token in (
            "universe-devel-jazzy-1.9.0",
            "Tracking mode:       scan_to_submap",
            "GNSS:                true",
            "compose.rviz.yaml",
            "build lsim",
            "run --rm lsim",
        ):
            if token not in result.stdout:
                fail(f"Docker wrapper dry run missing: {token}")


def main() -> int:
    check_compose()
    check_dockerfile()
    check_container_runner()
    check_host_wrapper()
    if ERRORS:
        print("Docker configuration checks FAILED", file=sys.stderr)
        for error in ERRORS:
            print(f"- {error}", file=sys.stderr)
        return 1
    print("Docker configuration checks PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
