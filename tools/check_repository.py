#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fail-fast checks that do not require a ROS installation."""
from __future__ import annotations

import ast
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

import yaml


class UniqueKeyLoader(yaml.SafeLoader):
    """YAML loader that rejects duplicate mapping keys."""


def _construct_unique_mapping(loader: UniqueKeyLoader, node: yaml.MappingNode, deep: bool = False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"found duplicate key: {key!r}",
                key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)

ROOT = Path(__file__).resolve().parents[1]
ERRORS: list[str] = []
IGNORED_TREE_PARTS = {
    ".git",
    "build",
    "install",
    "log",
    "rosbag",
    "test_results",
    "docker_output",
}


def is_ignored_tree_path(path: Path) -> bool:
    return any(part in IGNORED_TREE_PARTS for part in path.relative_to(ROOT).parts)


def fail(message: str) -> None:
    ERRORS.append(message)


def check_yaml() -> None:
    for path in sorted(ROOT.rglob("*.yaml")):
        if is_ignored_tree_path(path):
            continue
        try:
            yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueKeyLoader)
        except Exception as exc:  # noqa: BLE001
            fail(f"YAML {path.relative_to(ROOT)}: {exc}")


def check_xml() -> None:
    for path in sorted(ROOT.rglob("package.xml")):
        if is_ignored_tree_path(path):
            continue
        try:
            root = ET.parse(path).getroot()
            is_vendored = path == ROOT / "src/small_gicp/package.xml"
            if not is_vendored and root.findtext("license") != "Apache-2.0":
                fail(f"package license is not Apache-2.0: {path.relative_to(ROOT)}")
            if not root.findtext("maintainer"):
                fail(f"missing maintainer: {path.relative_to(ROOT)}")
        except Exception as exc:  # noqa: BLE001
            fail(f"XML {path.relative_to(ROOT)}: {exc}")


def check_python() -> None:
    for path in sorted(ROOT.rglob("*.py")):
        if is_ignored_tree_path(path):
            continue
        try:
            ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except SyntaxError as exc:
            fail(f"Python {path.relative_to(ROOT)}: {exc}")


def check_launch_construction() -> None:
    result = subprocess.run(
        [sys.executable, str(ROOT / "tools/check_launch_construction.py")],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
        env={**__import__("os").environ, "PYTHONDONTWRITEBYTECODE": "1"},
    )
    if result.returncode != 0:
        fail(
            "launch construction check failed:\n" + result.stdout + result.stderr
        )


def check_docker_configuration() -> None:
    result = subprocess.run(
        [sys.executable, str(ROOT / "tools/check_docker_configuration.py")],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
        env={**__import__("os").environ, "PYTHONDONTWRITEBYTECODE": "1"},
    )
    if result.returncode != 0:
        fail(
            "Docker configuration check failed:\n" + result.stdout + result.stderr
        )


def check_stable_package_identity() -> None:
    """Verify the project package set and retained runtime APIs."""
    expected = {
        Path("src/pure_nmea_gnss_conversion/package.xml"):
            ("pure_nmea_gnss_conversion", "0.3.0"),
        Path("src/pure_gnss_map_odom_fusion/package.xml"):
            ("pure_gnss_map_odom_fusion", "0.3.0"),
        Path("src/pure_gnss_msgs/package.xml"): ("pure_gnss_msgs", "0.3.0"),
        Path("src/pure_imu_undistortion/package.xml"):
            ("pure_imu_undistortion", "0.3.0"),
        Path("src/pure_lidar_gyro_odometer/package.xml"):
            ("pure_lidar_gyro_odometer", "0.3.0"),
        Path("src/pure_odometry_bringup/package.xml"):
            ("pure_odometry_bringup", "0.3.0"),
        Path("src/pure_autoware_localization_adapter/package.xml"):
            ("pure_autoware_localization_adapter", "0.3.0"),
        Path("src/pure_lidar_msgs/package.xml"): ("pure_lidar_msgs", "0.1.0"),
        Path("src/pure_lidar_submap_matcher/package.xml"):
            ("pure_lidar_submap_matcher", "0.1.0"),
        Path("src/pure_precision_global_localizer/package.xml"):
            ("pure_precision_global_localizer", "0.1.0"),
        Path("src/pure_precision_bringup/package.xml"):
            ("pure_precision_bringup", "0.1.0"),
    }
    discovered: dict[Path, str] = {}
    for relative_path, (expected_name, expected_version) in expected.items():
        package_path = ROOT / relative_path
        if not package_path.is_file():
            fail(f"stable ROS package missing: {relative_path}")
            continue
        package = ET.parse(package_path).getroot()
        actual_name = package.findtext("name") or ""
        discovered[relative_path] = actual_name
        if actual_name != expected_name:
            fail(
                f"ROS package identity changed in {relative_path}: "
                f"expected {expected_name}, got {actual_name}"
            )
        if package.findtext("version") != expected_version:
            fail(
                f"project package version must be {expected_version}: {relative_path}"
            )

    package_xmls = {
        path.relative_to(ROOT)
        for path in ROOT.glob("src/*/package.xml")
        if path.parent.name != "small_gicp"
    }
    if package_xmls != set(expected):
        fail(
            "project ROS package set changed: "
            f"expected {sorted(map(str, expected))}, got {sorted(map(str, package_xmls))}"
        )

    forbidden_directories = (
        "gicp_gnss_odom",
        "gicp_gnss_odom_fusion",
        "gicp_gnss_odom_imu_deskew",
        "gicp_gnss_odom_localizer",
        "gicp_gnss_odom_msgs",
        "gicp_gnss_odom_nmea",
        "gicp_gnss_odom_pointcloud_filters",
        "small_lidar_imu_gnss_localizer",
        "small_lidar_imu_gnss_odometry",
        "small_lidar_imu_gnss_msgs",
        "nmea_gnss_conversion",
        "pure_snow_intensity_filter",
    )
    for directory in forbidden_directories:
        if (ROOT / "src" / directory).exists():
            fail(f"obsolete or discarded ROS package directory reintroduced: src/{directory}")

    retired_intensity_root = ROOT / "src/pure_intensity_filter"
    if retired_intensity_root.exists() and any(
        path.is_file() for path in retired_intensity_root.rglob("*")
    ):
        fail("retired intensity-filter package content was reintroduced")

    runtime_roots = [ROOT / "src", ROOT / "script", ROOT / ".github/workflows"]
    forbidden_type_tokens = (
        "gicp_gnss_odom_msgs",
        "small_lidar_imu_gnss_msgs",
    )
    for runtime_root in runtime_roots:
        if not runtime_root.exists():
            continue
        for runtime_path in runtime_root.rglob("*"):
            if not runtime_path.is_file() or "small_gicp" in runtime_path.parts:
                continue
            try:
                runtime_text = runtime_path.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                continue
            for token in forbidden_type_tokens:
                if token in runtime_text:
                    fail(
                        f"discarded custom-message package token in "
                        f"{runtime_path.relative_to(ROOT)}: {token}"
                    )

    # Keep existing executable, node, and include identities so recorded/runtime
    # interfaces do not change.
    nmea_cmake = (ROOT / "src/pure_nmea_gnss_conversion/CMakeLists.txt").read_text()
    for token in (
        "project(pure_nmea_gnss_conversion)",
        "set(NMEA_CONVERSION_EXECUTABLE pure_nmea_gga_conversion)",
    ):
        if token not in nmea_cmake:
            fail(f"NMEA package-only rename contract missing: {token}")
    if not (
        ROOT /
        "src/pure_nmea_gnss_conversion/include/pure_nmea_gga_conversion/nmea_gga_conversion.hpp"
    ).is_file():
        fail("retained NMEA C++ include root is missing")

    container_launch = (
        ROOT / "src/pure_odometry_bringup/launch/odometry_container.launch.py"
    ).read_text()
    for token in (
        'package="pure_nmea_gnss_conversion"',
        'executable="pure_nmea_gga_conversion"',
    ):
        if token not in container_launch:
            fail(f"package/runtime launch compatibility missing: {token}")
    for launch_name in (
        "odometry_container.launch.py",
        "odometry_standalone.launch.py",
    ):
        launch_text = (
            ROOT / "src/pure_odometry_bringup/launch" / launch_name
        ).read_text()
        for token in (
            "pure_intensity_filter",
            "pure_snow_intensity_filter",
            "snow_intensity_filter",
            "use_snow_filter",
        ):
            if token in launch_text:
                fail(f"retired intensity-filter token in {launch_name}: {token}")

    message_path = ROOT / "src/pure_gnss_msgs/msg/GnssFusionInput.msg"
    if not message_path.is_file():
        fail("stable custom message path is missing: pure_gnss_msgs/msg/GnssFusionInput.msg")


def check_no_generated_files() -> None:
    for pattern in ("*.pyc", "*.o", "*.so", "*.a"):
        for path in ROOT.rglob(pattern):
            if not is_ignored_tree_path(path):
                fail(f"generated file in source tree: {path.relative_to(ROOT)}")
    for path in ROOT.rglob("__pycache__"):
        if not is_ignored_tree_path(path):
            fail(f"generated directory in source tree: {path.relative_to(ROOT)}")


def check_cpp_definition_duplicates() -> None:
    classes = {
        ROOT / "src/pure_nmea_gnss_conversion/src/nmea_gga_conversion.cpp": "NmeaGgaConversion",
        ROOT / "src/pure_gnss_map_odom_fusion/src/map_odom_fusion_node.cpp": "MapOdomFusionNode",
        ROOT / "src/pure_imu_undistortion/src/imu_undistorter.cpp": "ImuUndistorter",
        ROOT / "src/pure_lidar_gyro_odometer/src/gyro_odometer_node.cpp": "GyroOdometerNode",
    }
    for path, class_name in classes.items():
        names = re.findall(rf"\b{class_name}::([A-Za-z_]\w*)\s*\(", path.read_text())
        duplicates = sorted(name for name, count in Counter(names).items() if count > 1)
        if duplicates:
            fail(f"duplicate {class_name} definitions: {duplicates}")


def check_required_semantics() -> None:
    message = (ROOT / "src/pure_gnss_msgs/msg/GnssFusionInput.msg").read_text()
    for field in (
        "bool heading_valid",
        "bool position_is_base_link",
        "bool observation_point_valid",
        "geometry_msgs/Vector3 observation_point_in_base",
    ):
        if field not in message:
            fail(f"GnssFusionInput missing: {field}")

    fusion = (ROOT / "src/pure_gnss_map_odom_fusion/src/map_odom_fusion_node.cpp").read_text()
    recovery = (ROOT / "src/pure_gnss_map_odom_fusion/include/pure_gnss_map_odom_fusion/gnss_recovery_controller.hpp").read_text()
    for token in (
        "REACQUIRING",
        "RECOVERING_XY_ONLY",
        "TRACKING_XY_ONLY",
        "applyBoundedCorrection",
        "incrementalRecoveryCovariance",
        "positionYawJacobian",
    ):
        if token not in recovery and token not in fusion:
            fail(f"bounded multi-state GNSS recovery is missing: {token}")
    if re.search(r"force_accept\s*=\s*true", fusion):
        fail("active GNSS force-accept path detected")

    nmea = (ROOT / "src/pure_nmea_gnss_conversion/src/nmea_gga_conversion.cpp").read_text()
    for token in (
        "trajectory_imu_corrected",
        "imu_propagated",
        "heading_valid = false",
        "trajectoryHeadingPointIsUsable",
        "trajectoryHeadingVarianceIsUsable",
    ):
        if token not in nmea:
            fail(f"NMEA semantic path missing: {token}")

    deskew = (ROOT / "src/pure_imu_undistortion/param/param.yaml").read_text()
    if "allow_linear_time_fallback: false" not in deskew:
        fail("strict deskew fallback default changed")

    adapter = (
        ROOT
        / "src/pure_autoware_localization_adapter/src/autoware_localization_adapter_node.cpp"
    ).read_text(encoding="utf-8")
    for token in (
        "/localization/kinematic_state",
        "/localization/acceleration",
        "/localization/pose_estimator/pose_with_covariance",
        "sendTransform",
        "AccelerationEstimator",
    ):
        if token not in adapter:
            fail(f"Autoware localization adapter path missing: {token}")

    lsim_launch = (
        ROOT / "src/pure_odometry_bringup/launch/autoware_lsim_localization.launch.py"
    ).read_text(encoding="utf-8")
    for key in (
        "launch_map",
        "launch_localization",
        "launch_perception",
        "launch_planning",
        "launch_control",
        "launch_system",
        "launch_api",
        "fusion_publish_tf",
    ):
        expected_assignment = f'"{key}": "false"'
        if expected_assignment not in lsim_launch:
            fail(
                "localization-only Autoware LSim guard missing: "
                f"{expected_assignment}"
            )
    for token in (
        "pure_autoware_localization_adapter",
        "use_map_odom_fusion",
        "autoware.launch.xml",
        "hesai_rosbag23",
        "hesai_lidar_static_transform",
        "hesai_imu_static_transform",
        "hesai_gnss_static_transform",
        "nmea_gnss_override_param",
    ):
        if token not in lsim_launch:
            fail(f"Autoware LSim launch path missing: {token}")

    bringup_launch = (
        ROOT / "src/pure_odometry_bringup/launch/odometry_standalone.launch.py"
    ).read_text(encoding="utf-8")
    for token in (
        "use_map_odom_fusion",
        "use_imu_deskew",
        "points_input_topic",
        "odom_override_param",
        "nmea_gnss_override_param",
    ):
        if token not in bringup_launch:
            fail(f"bag-replay launch argument missing: {token}")

    for token in ("duplicate_stamp_drop_count", "out_of_order_stamp"):
        if token not in adapter:
            fail(f"Autoware adapter timestamp handling missing: {token}")

    bringup_package = (ROOT / "src/pure_odometry_bringup/package.xml").read_text(
        encoding="utf-8"
    )
    if "<exec_depend>autoware_launch</exec_depend>" in bringup_package:
        fail("standalone bringup must not acquire a hard Autoware dependency")



def parameter_map(path: Path) -> dict[str, object]:
    data = yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueKeyLoader)
    try:
        return data["/**"]["ros__parameters"]
    except (KeyError, TypeError):
        fail(f"invalid ROS parameter root: {path.relative_to(ROOT)}")
        return {}



def check_parameter_files_match_nodes() -> None:
    """Reject stale YAML keys that no node declares.

    This deliberately checks only YAML -> source. Parameters may be declared in
    code but omitted from a specialized configuration so the node default is
    used.
    """
    groups = (
        (ROOT / "src/pure_nmea_gnss_conversion/src/nmea_gga_conversion.cpp",
         ROOT / "src/pure_nmea_gnss_conversion/param"),
        (ROOT / "src/pure_gnss_map_odom_fusion/src/map_odom_fusion_node.cpp",
         ROOT / "src/pure_gnss_map_odom_fusion/param"),
        (ROOT / "src/pure_imu_undistortion/src/imu_undistorter.cpp",
         ROOT / "src/pure_imu_undistortion/param"),
        (ROOT / "src/pure_lidar_gyro_odometer/src/gyro_odometer_node.cpp",
         ROOT / "src/pure_lidar_gyro_odometer/param"),
        (ROOT / "src/pure_autoware_localization_adapter/src/autoware_localization_adapter_node.cpp",
         ROOT / "src/pure_autoware_localization_adapter/param"),
    )
    declaration = re.compile(
        r'declare_parameter(?:\s*<.*?>)?\s*\(\s*"([^"]+)"', re.DOTALL
    )
    for source_path, parameter_directory in groups:
        declared = set(declaration.findall(source_path.read_text(encoding="utf-8")))
        for parameter_path in sorted(parameter_directory.glob("*.yaml")):
            data = yaml.safe_load(parameter_path.read_text(encoding="utf-8"))
            if not isinstance(data, dict) or len(data) != 1:
                fail(f"invalid ROS parameter root: {parameter_path.relative_to(ROOT)}")
                continue
            node_mapping = next(iter(data.values()))
            if not isinstance(node_mapping, dict) or not isinstance(
                node_mapping.get("ros__parameters"), dict
            ):
                fail(f"invalid ROS parameter root: {parameter_path.relative_to(ROOT)}")
                continue
            stale = sorted(set(node_mapping["ros__parameters"]) - declared)
            if stale:
                fail(
                    f"undeclared parameters in {parameter_path.relative_to(ROOT)}: {stale}"
                )

def check_safe_defaults() -> None:
    imu = parameter_map(ROOT / "src/pure_imu_undistortion/param/param.yaml")
    lidar = parameter_map(ROOT / "src/pure_lidar_gyro_odometer/param/param.yaml")
    nmea = parameter_map(ROOT / "src/pure_nmea_gnss_conversion/param/param.yaml")
    fusion = parameter_map(ROOT / "src/pure_gnss_map_odom_fusion/param/param.yaml")
    adapter = parameter_map(
        ROOT / "src/pure_autoware_localization_adapter/param/param.yaml"
    )
    expected_false = {
        "allow_linear_time_fallback": imu,
        "allow_default_speed_fallback": imu,
        "use_translation": imu,
        "use_twist_speed": imu,
        "wheel_speed.use": lidar,
        "wheel_speed.low_speed.enable": lidar,
        "wheel_speed.scale_estimation.enable": lidar,
        "wheel_speed.observability_assist.enable": lidar,
        "lidar_odom.observability.debug_pub.enable": lidar,
        "out_filtered_odom.enable": lidar,
        "lidar_odom.smoother.zupt.enable": lidar,
        "lidar_odom.smoother.nhc.enable": lidar,
        "lidar_odom.local_map.enable": lidar,
        "allow_parameter_antenna_fallback": nmea,
        "gnss_allow_unknown_observation_point": fusion,
        "gnss_force_accept_allow_float": fusion,
        "xy_only_recovery.enabled": fusion,
    }
    for key, mapping in expected_false.items():
        if mapping.get(key) is not False:
            fail(f"unsafe public default must be false: {key}")
    if imu.get("twist_topic") != "":
        fail("generic IMU deskew twist topic must be empty by default")
    for key in ("primary_antenna_position_base", "secondary_antenna_position_base"):
        if nmea.get(key) not in ([], None):
            fail(f"generic antenna geometry must be empty: {key}")
    if fusion.get("time_out_dead_reckoning") not in (0, 0.0):
        fail("deprecated force-recovery timeout must remain zero")
    for key in (
        "require_expected_frames",
        "publish_tf",
        "publish_pose",
        "publish_acceleration",
    ):
        if adapter.get(key) is not True:
            fail(f"Autoware adapter safe/default interface must remain enabled: {key}")




def check_lidar_tracking_modes() -> None:
    generic_path = ROOT / "src/pure_lidar_gyro_odometer/param/param.yaml"
    submap_path = ROOT / "src/pure_lidar_gyro_odometer/param/param_scan_to_submap.yaml"
    generic = parameter_map(generic_path)
    submap = parameter_map(submap_path)

    if generic.get("lidar_odom.tracking_mode") != "scan_to_scan":
        fail("backward-compatible generic LiDAR mode must remain scan_to_scan")
    if submap.get("lidar_odom.tracking_mode") != "scan_to_submap":
        fail("scan-to-submap profile must select scan_to_submap")

    profile_differences = {
        key
        for key in set(generic) | set(submap)
        if generic.get(key) != submap.get(key)
    }
    if profile_differences != {"lidar_odom.tracking_mode"}:
        fail(
            "scan-to-submap profile must differ from the generic profile only by "
            f"lidar_odom.tracking_mode: {sorted(profile_differences)}"
        )

    for name, mapping in (("generic", generic), ("submap", submap)):
        if mapping.get("wheel_speed.use") is not False:
            fail(f"{name} LiDAR profile must not require wheel speed")
        if mapping.get("lidar_odom.smoother.hessian_information.enable") is not True:
            fail(f"{name} LiDAR profile must keep directional Hessian weighting enabled")
        if mapping.get("wheel_speed.observability_assist.enable") is not False:
            fail(f"{name} LiDAR profile must keep optional wheel assist disabled")
        if mapping.get("lidar_odom.smoother.enable") is not True:
            fail(f"{name} LiDAR profile must keep the fixed-lag smoother enabled")

    selector = (
        ROOT
        / "src/pure_lidar_gyro_odometer/include/pure_lidar_gyro_odometer/tracking_mode.hpp"
    ).read_text(encoding="utf-8")
    for token in (
        "ScanToScanWarmup",
        "ScanToSubmap",
        "ScanToScanInterim",
        "ScanToScanFallback",
    ):
        if token not in selector:
            fail(f"LiDAR tracking selector missing: {token}")

    odometer = (
        ROOT / "src/pure_lidar_gyro_odometer/src/gyro_odometer_node.cpp"
    ).read_text(encoding="utf-8")
    forbidden_binary_tokens = (
        "DegeneracyInfo",
        "degeneracy_state_",
        "weak_observation_streak_",
        "critical_latch_until_",
        "lidarPoseModeToString",
        "out_degeneracy",
        "out_pose_mode",
        "wheel_speed.degeneracy",
        "lidar_odom.degeneracy_detection",
    )
    for token in forbidden_binary_tokens:
        if token in odometer:
            fail(f"binary LiDAR degeneracy logic remains in odometer source: {token}")

    header = (
        ROOT
        / "src/pure_lidar_gyro_odometer/include/pure_lidar_gyro_odometer/gyro_odometer_node.hpp"
    ).read_text(encoding="utf-8")
    for token in forbidden_binary_tokens:
        if token in header:
            fail(f"binary LiDAR degeneracy logic remains in odometer header: {token}")

    for parameter_path in sorted(
        (ROOT / "src/pure_lidar_gyro_odometer/param").glob("*.yaml")
    ):
        parameter_text = parameter_path.read_text(encoding="utf-8")
        for token in (
            "wheel_speed.degeneracy",
            "lidar_odom.degeneracy_detection",
            "out_degeneracy",
            "out_pose_mode",
            "odom_covariance.degenerate_scale",
        ):
            if token in parameter_text:
                fail(
                    f"binary degeneracy parameter remains in "
                    f"{parameter_path.relative_to(ROOT)}: {token}"
                )

    for required in (
        "analyzeDirectionalInformation",
        "observability::weaknessWeight",
        "observability::covarianceScale",
        "information_ratio_min",
        "rejection_reason",
    ):
        if required not in odometer:
            fail(f"continuous observability path missing: {required}")
    for token in (
        "repairLocalMapFromSmootherLocked",
        "reanchorLocalMapLocked",
        "consecutive_scan_to_submap_failures",
    ):
        if token not in odometer:
            fail(f"repairable scan-to-submap path missing: {token}")


def check_gnss_message_access() -> None:
    message_path = ROOT / "src/pure_gnss_msgs/msg/GnssFusionInput.msg"
    fields: set[str] = set()
    for line in message_path.read_text().splitlines():
        content = line.split("#", 1)[0].strip()
        if not content:
            continue
        parts = content.split()
        if len(parts) >= 2:
            fields.add(parts[1])

    nmea = (ROOT / "src/pure_nmea_gnss_conversion/src/nmea_gga_conversion.cpp").read_text()
    start = nmea.find("void NmeaGgaConversion::publishStatusOnly")
    end = nmea.find("void NmeaGgaConversion::publishConfidence", start)
    written = set(re.findall(r"\b(?:input|msg)\.([A-Za-z_]\w*)", nmea[start:end]))
    # Nested ROS-message fields are legal members of known top-level fields.
    written -= {"pose", "twist", "header", "child_frame_id"}
    unknown = written - fields
    if unknown:
        fail(f"NMEA writes unknown GnssFusionInput fields: {sorted(unknown)}")

    fusion = (ROOT / "src/pure_gnss_map_odom_fusion/src/map_odom_fusion_node.cpp").read_text()
    start = fusion.find("void MapOdomFusionNode::onGnssInput")
    end = fusion.find("void MapOdomFusionNode::onLegacyAnchorPose", start)
    read = set(re.findall(r"message->([A-Za-z_]\w*)", fusion[start:end]))
    unknown = read - fields
    if unknown:
        fail(f"fusion reads unknown GnssFusionInput fields: {sorted(unknown)}")


def check_hygiene() -> None:
    conflict = re.compile(r"^(<<<<<<<|=======|>>>>>>>)", re.MULTILINE)
    for path in sorted(ROOT.rglob("*")):
        if (
            not path.is_file()
            or is_ignored_tree_path(path)
            or "small_gicp" in path.parts
        ):
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue
        if conflict.search(text):
            fail(f"merge conflict marker: {path.relative_to(ROOT)}")
    if not (ROOT / "src/small_gicp/LICENSE").is_file():
        fail("vendored small_gicp license is missing")


def check_shell() -> None:
    paths = [
        ROOT / "build_setup.sh",
        *sorted((ROOT / "script").glob("*.sh")),
        *sorted((ROOT / "tools").glob("*.sh")),
        *sorted((ROOT / "docker").rglob("*.sh")),
    ]
    for path in paths:
        result = subprocess.run(["bash", "-n", str(path)], text=True, capture_output=True)
        if result.returncode:
            fail(f"shell syntax {path.relative_to(ROOT)}: {result.stderr.strip()}")


def check_replay_helper() -> None:
    helper = ROOT / "script/play_localization_bag.sh"
    with tempfile.TemporaryDirectory(prefix="localization_bag_check_") as directory:
        bag_path = Path(directory) / "bag"
        bag_path.mkdir()
        result = subprocess.run(
            [
                str(helper),
                "--dry-run",
                "--bag",
                str(bag_path),
                "--points",
                "/recorded/points",
                "--imu",
                "/recorded/imu",
                "--nmea",
                "/recorded/nmea",
            ],
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=False,
        )
        if result.returncode != 0:
            fail(f"bag replay helper dry run failed: {result.stderr.strip()}")
            return
        for token in (
            "/recorded/points:=/points_raw",
            "/recorded/imu:=/imu",
            "/recorded/nmea:=/nmea_sentence",
            "/localization/kinematic_state:=/reference/localization/kinematic_state",
            "/tf:=/reference/tf",
        ):
            if token not in result.stdout:
                fail(f"bag replay helper dry run missing: {token}")
        if "--progress-bar-update-rate" in result.stdout:
            fail("bag replay helper uses unsupported --progress-bar-update-rate")

def check_diff_whitespace() -> None:
    # Archives intentionally do not contain .git, so always perform a source-tree
    # whitespace check and use git's stricter checker only when metadata exists.
    for path in sorted(ROOT.rglob("*")):
        if (
            not path.is_file()
            or "small_gicp" in path.parts
            or is_ignored_tree_path(path)
        ):
            continue
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except UnicodeDecodeError:
            continue
        for lineno, line in enumerate(lines, 1):
            if line.endswith((" ", "\t")):
                fail(f"trailing whitespace: {path.relative_to(ROOT)}:{lineno}")

    if (ROOT / ".git").exists():
        result = subprocess.run(
            ["git", "diff", "--check"], cwd=ROOT, text=True, capture_output=True, check=False
        )
        if result.returncode:
            fail("git diff --check failed:\n" + result.stdout + result.stderr)


def main() -> int:
    check_yaml()
    check_xml()
    check_python()
    check_launch_construction()
    check_docker_configuration()
    check_stable_package_identity()
    check_no_generated_files()
    check_cpp_definition_duplicates()
    check_required_semantics()
    check_parameter_files_match_nodes()
    check_safe_defaults()
    check_lidar_tracking_modes()
    check_gnss_message_access()
    check_hygiene()
    check_shell()
    check_replay_helper()
    check_diff_whitespace()
    if ERRORS:
        print("Repository checks FAILED", file=sys.stderr)
        for error in ERRORS:
            print(f"- {error}", file=sys.stderr)
        return 1
    print("Repository checks PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
