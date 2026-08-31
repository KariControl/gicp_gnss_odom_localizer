#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Launch every supported TF profile and verify its runtime ownership graph."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import unittest


GYRO_OWNER = "/gyro_odometer,odom_frame,odom,base_frame,base_link"
FUSION_OWNER = "/gnss_map_odom_fusion,map_frame,map,odom_frame,odom"


@dataclass(frozen=True)
class Profile:
    name: str
    launch: tuple[str, ...]
    owners: tuple[str, ...] = ()
    disabled_owners: tuple[str, ...] = ()
    required_nodes: tuple[str, ...] = ()
    initialize_map_odom: bool = False


PROFILES = (
    Profile(
        "container_local_deskewed",
        (
            "ros2",
            "launch",
            "pure_odometry_bringup",
            "odometry_container.launch.py",
            "use_sim_time:=false",
            "use_gnss:=false",
            "use_map_odom_fusion:=false",
            "use_imu_deskew:=true",
        ),
        owners=(GYRO_OWNER,),
    ),
    Profile(
        "container_local_direct",
        (
            "ros2",
            "launch",
            "pure_odometry_bringup",
            "odometry_container.launch.py",
            "use_sim_time:=false",
            "use_gnss:=false",
            "use_map_odom_fusion:=false",
            "use_imu_deskew:=false",
        ),
        owners=(GYRO_OWNER,),
    ),
    Profile(
        "container_fused",
        (
            "ros2",
            "launch",
            "pure_odometry_bringup",
            "odometry_container.launch.py",
            "use_sim_time:=false",
            "use_gnss:=false",
            "use_map_odom_fusion:=true",
            "use_imu_deskew:=true",
        ),
        owners=(GYRO_OWNER, FUSION_OWNER),
        initialize_map_odom=True,
    ),
    Profile(
        "container_gnss",
        (
            "ros2",
            "launch",
            "pure_odometry_bringup",
            "odometry_container.launch.py",
            "use_sim_time:=false",
            "use_gnss:=true",
            "use_map_odom_fusion:=false",
            "use_imu_deskew:=true",
        ),
        owners=(GYRO_OWNER, FUSION_OWNER),
        required_nodes=("/nmea_gga_conversion",),
        initialize_map_odom=True,
    ),
    Profile(
        "standalone_local",
        (
            "ros2",
            "launch",
            "pure_odometry_bringup",
            "odometry_standalone.launch.py",
            "use_sim_time:=false",
            "use_gnss:=false",
            "use_map_odom_fusion:=false",
        ),
        owners=(GYRO_OWNER,),
    ),
    Profile(
        "standalone_fused",
        (
            "ros2",
            "launch",
            "pure_odometry_bringup",
            "odometry_standalone.launch.py",
            "use_sim_time:=false",
            "use_gnss:=false",
            "use_map_odom_fusion:=true",
        ),
        owners=(GYRO_OWNER, FUSION_OWNER),
        initialize_map_odom=True,
    ),
    Profile(
        "standalone_with_nmea",
        (
            "ros2",
            "launch",
            "pure_nmea_gnss_conversion",
            "odometry_standalone_with_nmea.launch.py",
            "use_sim_time:=false",
            "use_gnss:=true",
        ),
        owners=(GYRO_OWNER, FUSION_OWNER),
        initialize_map_odom=True,
    ),
    Profile(
        "lidar_imu_only_default",
        (
            "ros2",
            "launch",
            "pure_odometry_bringup",
            "lidar_imu_only.launch.py",
            "use_sim_time:=false",
            "use_imu_deskew:=true",
        ),
        disabled_owners=("/gyro_odometer",),
    ),
    Profile(
        "lidar_imu_only_opt_in_deskewed",
        (
            "ros2",
            "launch",
            "pure_odometry_bringup",
            "lidar_imu_only.launch.py",
            "use_sim_time:=false",
            "use_imu_deskew:=true",
            "odom_publish_tf:=true",
        ),
        owners=(GYRO_OWNER,),
    ),
    Profile(
        "lidar_imu_only_opt_in_direct",
        (
            "ros2",
            "launch",
            "pure_odometry_bringup",
            "lidar_imu_only.launch.py",
            "use_sim_time:=false",
            "use_imu_deskew:=false",
            "odom_publish_tf:=true",
        ),
        owners=(GYRO_OWNER,),
    ),
    Profile(
        "precision_overlay",
        (
            "ros2",
            "launch",
            "pure_precision_bringup",
            "precision_overlay.launch.py",
            "use_sim_time:=false",
        ),
        required_nodes=("/submap_matcher", "/precision_global_localizer"),
    ),
)


class TfOwnershipProfilesTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory(
            prefix="pure_tf_ownership_"
        )
        self.root = Path(self.temporary_directory.name)

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def environment(self, domain_id: int, name: str) -> dict[str, str]:
        environment = dict(os.environ)
        environment["ROS_DOMAIN_ID"] = str(domain_id)
        # Fast DDS cannot discover participants in some restricted CI/sandbox
        # environments when ROS_LOCALHOST_ONLY is forced.  Domain isolation is
        # sufficient here and matches the repository's other runtime tests.
        environment.pop("ROS_LOCALHOST_ONLY", None)
        environment.pop("ROS_AUTOMATIC_DISCOVERY_RANGE", None)
        environment.pop("ROS_STATIC_PEERS", None)
        environment["RCUTILS_COLORIZED_OUTPUT"] = "0"
        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        environment["PYTHONUNBUFFERED"] = "1"
        log_directory = self.root / "ros_logs" / name
        log_directory.mkdir(parents=True, exist_ok=True)
        environment["ROS_LOG_DIR"] = str(log_directory)
        return environment

    @staticmethod
    def stop_process(process: subprocess.Popen[str]) -> None:
        if process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=3.0)
            return
        except subprocess.TimeoutExpired:
            pass
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=2.0)
            return
        except subprocess.TimeoutExpired:
            pass
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=2.0)

    def run_profile(self, profile: Profile, domain_id: int) -> None:
        environment = self.environment(domain_id, profile.name)
        launch_log = self.root / f"{profile.name}.launch.log"
        evidence_path = self.root / f"{profile.name}.json"
        probe_command = [
            "ros2",
            "run",
            "pure_localization_contract",
            "tf_ownership_probe.py",
            "--timeout",
            "12",
            "--settle-sec",
            "0.5",
            "--output",
            str(evidence_path),
        ]
        for owner in profile.owners:
            probe_command.extend(("--owner", owner))
        for owner in profile.disabled_owners:
            probe_command.extend(("--disabled-owner", owner))
        for node in profile.required_nodes:
            probe_command.extend(("--required-node", node))
        if profile.initialize_map_odom:
            probe_command.extend(
                (
                    "--initialize-map-odom-from",
                    "/localization/gyro_lidar_odom",
                )
            )

        with launch_log.open("w", encoding="utf-8") as output:
            launch = subprocess.Popen(
                profile.launch,
                env=environment,
                stdout=output,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            try:
                result = subprocess.run(
                    probe_command,
                    env=environment,
                    text=True,
                    capture_output=True,
                    check=False,
                    timeout=20.0,
                )
                launch_status = launch.poll()
            finally:
                self.stop_process(launch)

        log_text = launch_log.read_text(encoding="utf-8", errors="replace")
        details = (
            f"profile={profile.name}\n"
            f"probe stdout:\n{result.stdout}\n"
            f"probe stderr:\n{result.stderr}\n"
            f"launch log:\n{log_text[-12000:]}"
        )
        self.assertIsNone(launch_status, details)
        self.assertEqual(result.returncode, 0, details)
        self.assertTrue(evidence_path.is_file(), details)
        evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
        self.assertEqual(evidence.get("result"), "PASS", details)
        expected_nodes = sorted(owner.split(",", maxsplit=1)[0] for owner in profile.owners)
        actual_nodes = sorted(
            endpoint["node"] for endpoint in evidence["publisher_endpoints"]
        )
        self.assertEqual(actual_nodes, expected_nodes, details)

    def test_supported_profile_matrix(self) -> None:
        for index, profile in enumerate(PROFILES):
            with self.subTest(profile=profile.name):
                self.run_profile(profile, 180 + index)

    def test_duplicate_same_edge_publisher_is_rejected(self) -> None:
        environment = self.environment(220, "duplicate_same_edge")
        processes: list[subprocess.Popen[str]] = []
        process_statuses: list[int | None] = []
        logs = []
        try:
            for node_name in ("tf_owner_primary", "tf_owner_duplicate"):
                log_path = self.root / f"{node_name}.log"
                log_handle = log_path.open("w", encoding="utf-8")
                logs.append((log_handle, log_path))
                processes.append(
                    subprocess.Popen(
                        (
                            "ros2",
                            "run",
                            "pure_localization_interface_adapter",
                            "localization_interface_adapter_node",
                            "--ros-args",
                            "-r",
                            f"__node:={node_name}",
                            "-p",
                            "publish_tf:=true",
                            "-p",
                            "map_frame:=map",
                            "-p",
                            "base_frame:=base_link",
                        ),
                        env=environment,
                        stdout=log_handle,
                        stderr=subprocess.STDOUT,
                        text=True,
                        start_new_session=True,
                    )
                )

            evidence_path = self.root / "duplicate_same_edge.json"
            result = subprocess.run(
                (
                    "ros2",
                    "run",
                    "pure_localization_contract",
                    "tf_ownership_probe.py",
                    "--owner",
                    "/tf_owner_primary,map_frame,map,base_frame,base_link",
                    "--required-node",
                    "/tf_owner_duplicate",
                    "--skip-edge-samples",
                    "--settle-sec",
                    "0.2",
                    "--timeout",
                    "3",
                    "--output",
                    str(evidence_path),
                ),
                env=environment,
                text=True,
                capture_output=True,
                check=False,
                timeout=8.0,
            )
            process_statuses = [process.poll() for process in processes]
        finally:
            for process in reversed(processes):
                self.stop_process(process)
            for log_handle, _ in logs:
                log_handle.close()

        details = f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        for _, log_path in logs:
            details += f"\n{log_path.name}:\n{log_path.read_text(encoding='utf-8')[-4000:]}"
        self.assertEqual(process_statuses, [None, None], details)
        self.assertNotEqual(result.returncode, 0, details)
        self.assertTrue(evidence_path.is_file(), details)
        evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
        self.assertEqual(evidence.get("result"), "FAIL", details)
        failure_evidence = evidence.get("failure_evidence", {})
        self.assertEqual(failure_evidence.get("missing_nodes"), [], details)
        self.assertEqual(
            failure_evidence.get("expected_publisher_counts"),
            {"/tf_owner_primary": 1},
            details,
        )
        self.assertEqual(
            failure_evidence.get("observed_publisher_counts"),
            {"/tf_owner_duplicate": 1, "/tf_owner_primary": 1},
            details,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
