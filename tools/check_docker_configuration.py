#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Static checks for the CPU-only Autoware integration Docker profile."""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
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
        fail(
            "Autoware integration service must use host networking for "
            "single-host DDS"
        )
    if service.get("ipc") != "host":
        fail("Autoware integration service must use host IPC")
    if service.get("privileged") is not True:
        fail(
            "Autoware integration service must remain privileged for the "
            "official entrypoint DDS tuning"
        )

    environment = service.get("environment", {})
    required_environment = {
        "BAG_PATH": "/bags/input",
        "OUTPUT_ROOT": "/output",
        "DATASET_PROFILE": "${DATASET_PROFILE:-generic}",
        "CLOCK_FREQUENCY": "${CLOCK_FREQUENCY:-100.0}",
        "RVIZ": "${RVIZ:-false}",
        "RQT_ROBOT_MONITOR": "${RQT_ROBOT_MONITOR:-false}",
        "RVIZ_SAMPLE_VEHICLE": "${RVIZ_SAMPLE_VEHICLE:-false}",
        "RVIZ_SAMPLE_VEHICLE_Z_OFFSET": "${RVIZ_SAMPLE_VEHICLE_Z_OFFSET:--1.66}",
        "LOCALIZER_IMAGE_ID": "${LOCALIZER_IMAGE_ID:-unknown}",
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
        fail("GUI overlay must force software rendering")
    if rviz_environment.get("DISPLAY") != (
        "${DISPLAY:?DISPLAY is required when a GUI is enabled}"
    ):
        fail("GUI overlay must require DISPLAY")
    if "RVIZ" in rviz_environment:
        fail("shared GUI overlay must not force RVIZ")
    rviz_mounts = {
        volume.get("target"): volume
        for volume in rviz.get("volumes", [])
        if isinstance(volume, dict)
    }
    if "/tmp/.X11-unix" not in rviz_mounts:
        fail("GUI overlay must mount the X11 socket")


def check_dockerfile() -> None:
    path = ROOT / "docker/autoware_lsim/Dockerfile"
    if not path.is_file():
        fail("Dockerfile is missing")
        return
    text = path.read_text(encoding="utf-8")
    for token in (
        "universe-devel-jazzy-1.9.0",
        "source /opt/autoware/setup.bash",
        "rosbag2-storage-mcap",
        "rosdep install",
        "colcon --log-base /tmp/gicp_gnss_odom_ws/log build",
        "--merge-install",
        "-DBUILD_TESTING=OFF",
        "python3-numpy",
        "/opt/gicp_gnss_odom_localizer",
        "container_entrypoint.sh",
        "play_localization_bag.sh",
        "analyze_autoware_lsim_output.py",
    ):
        if token not in text:
            fail(f"Dockerfile required token missing: {token}")
    for token in ("universe-devel-cuda", "--gpus", "nvidia"):
        if token in text.lower():
            fail(f"CPU-only Dockerfile contains GPU-specific token: {token}")

    ignore_path = ROOT / ".dockerignore"
    if not ignore_path.is_file():
        fail(".dockerignore is missing; large bags would enter the build context")
    else:
        ignored = set(ignore_path.read_text(encoding="utf-8").splitlines())
        for required in ("build", "install", "log", "rosbag", "test_results"):
            if required not in ignored:
                fail(f".dockerignore must exclude {required}")


def check_rviz_config() -> None:
    path = (
        ROOT
        / "src/pure_odometry_bringup/config/autoware_lsim/hesai_rosbag23.rviz"
    )
    if not path.is_file():
        fail("Hesai ROSBAG2/3 RViz config is missing")
        return
    text = path.read_text(encoding="utf-8")
    for token in (
        "Fixed Frame: map",
        "rviz_default_plugins/Grid",
        "rviz_default_plugins/PointCloud2",
        "/localization/points_undistorted",
        "Reliability Policy: Best Effort",
        "rviz_default_plugins/Odometry",
        "/localization/kinematic_state",
        "rviz_default_plugins/TF",
        "Reference Frame: base_link",
    ):
        if token not in text:
            fail(f"Hesai RViz config required display missing: {token}")

    sample_path = (
        ROOT
        / "src/pure_odometry_bringup/config/autoware_lsim/hesai_rosbag23_sample_vehicle.rviz"
    )
    if not sample_path.is_file():
        fail("Autoware sample-vehicle RViz config is missing")
        return
    sample_config = load_yaml(sample_path)
    sample_text = sample_path.read_text(encoding="utf-8")
    for token in (
        "Fixed Frame: map",
        "rviz_default_plugins/RobotModel",
        "/localization/visualization/robot_description",
        "Durability Policy: Transient Local",
        "Autoware sample body (illustrative, not calibrated)",
        "Decay Time: 0",
        "/localization/points_undistorted",
        "/localization/kinematic_state",
        "rviz_default_plugins/Path",
        "/localization/visualization/trajectory",
        "rviz_default_plugins/MarkerArray",
        "/localization/visualization/status_markers",
        "XY Position Covariance (2 sigma)",
        "xy_position_covariance_2sigma: true",
        "pure_odometry_bringup/LocalizationStatus",
        "Name: Autoware Localization Status",
        "Diagnostics Topic: /diagnostics",
        "Kinematic State Topic: /localization/kinematic_state",
        "/Autoware Localization Status1/Live Status1",
        "Target Frame: base_link",
        "rviz_default_plugins/XYOrbit",
    ):
        if token not in sample_text:
            fail(f"sample-vehicle RViz config required display missing: {token}")

    displays = (
        sample_config.get("Visualization Manager", {}).get("Displays", [])
        if isinstance(sample_config.get("Visualization Manager"), dict)
        else []
    )
    if not isinstance(displays, list):
        fail("sample-vehicle RViz config Displays must be a list")
        displays = []

    def displays_for(plugin_class: str) -> list[dict]:
        return [
            display
            for display in displays
            if isinstance(display, dict) and display.get("Class") == plugin_class
        ]

    def topic_value(display: dict) -> object:
        topic = display.get("Topic", {})
        return topic.get("Value") if isinstance(topic, dict) else None

    trajectory_displays = displays_for("rviz_default_plugins/Path")
    if not any(
        display.get("Enabled") is True
        and topic_value(display) == "/localization/visualization/trajectory"
        for display in trajectory_displays
    ):
        fail("sample-vehicle RViz must enable the generated line trajectory")

    marker_displays = displays_for("rviz_default_plugins/MarkerArray")
    covariance_displays = [
        display
        for display in marker_displays
        if display.get("Enabled") is True
        and topic_value(display) == "/localization/visualization/status_markers"
    ]
    if len(covariance_displays) != 1:
        fail("sample-vehicle RViz must enable exactly one XY covariance MarkerArray")
    else:
        covariance_display = covariance_displays[0]
        if covariance_display.get("Name") != "XY Position Covariance (2 sigma)":
            fail("sample-vehicle covariance MarkerArray has the wrong display name")
        if covariance_display.get("Namespaces") != {
            "xy_position_covariance_2sigma": True
        }:
            fail("sample-vehicle covariance MarkerArray must contain no status HUD namespace")

    status_displays = displays_for("pure_odometry_bringup/LocalizationStatus")
    if len(status_displays) != 1:
        fail("sample-vehicle RViz must configure exactly one localization-status Display")
    else:
        status_display = status_displays[0]
        expected_status_settings = {
            "Enabled": True,
            "Name": "Autoware Localization Status",
            "Kinematic State Topic": "/localization/kinematic_state",
            "Diagnostics Topic": "/diagnostics",
            "Map Frame": "map",
            "Base Frame": "base_link",
        }
        for key, expected in expected_status_settings.items():
            if status_display.get(key) != expected:
                fail(
                    "sample-vehicle localization-status Display setting is incorrect: "
                    f"{key} must be {expected}"
                )

    if "localization_status_hud" in sample_text:
        fail("sample-vehicle RViz config must not render status text in the 3D scene")

    odometry_displays = displays_for("rviz_default_plugins/Odometry")
    kinematic_odometry_found = False
    for display in odometry_displays:
        covariance = display.get("Covariance", {})
        position = covariance.get("Position", {}) if isinstance(covariance, dict) else {}
        if (
            display.get("Enabled") is True
            and topic_value(display) == "/localization/kinematic_state"
        ):
            kinematic_odometry_found = True
            if display.get("Keep") != 1:
                fail(
                    "sample-vehicle RViz must keep only the current odometry arrow; "
                    "the bounded Path display owns trajectory history"
                )
            if covariance.get("Value") is True or (
                isinstance(position, dict) and position.get("Value") is True
            ):
                fail(
                    "standard 3D odometry covariance must remain disabled; "
                    "the visualization MarkerArray provides the bounded XY ellipse"
                )
    if not kinematic_odometry_found:
        fail("sample-vehicle RViz must retain the kinematic-state odometry display")

    visualization_source = (
        ROOT / "src/pure_odometry_bringup/src/localization_visualization_node.cpp"
    )
    if not visualization_source.is_file():
        fail("localization presentation node source is missing")
    else:
        visualization_text = visualization_source.read_text(encoding="utf-8")
        for token in (
            "xy_position_covariance_2sigma",
            "visualization_msgs::msg::Marker::LINE_STRIP",
            ".reliable().transient_local()",
            'declare_parameter<std::int64_t>("max_path_poses", 2000)',
            'declare_parameter<double>("covariance_sigma", 2.0)',
            'declare_parameter<double>("covariance_max_radius_m", 20.0)',
        ):
            if token not in visualization_text:
                fail(f"localization presentation source contract missing: {token}")
        for retired_hud_token in (
            "localization_status_hud",
            "TEXT_VIEW_FACING",
            "Autoware Localization Interface:",
        ):
            if retired_hud_token in visualization_text:
                fail(
                    "localization presentation node must leave status text to the "
                    f"RViz Display plugin: {retired_hud_token}"
                )

    status_display_source = (
        ROOT / "src/pure_odometry_bringup/src/localization_status_display.cpp"
    )
    if not status_display_source.is_file():
        fail("localization-status RViz Display source is missing")
    else:
        status_display_text = status_display_source.read_text(encoding="utf-8")
        for token in (
            '"Kinematic State Topic", "/localization/kinematic_state"',
            '"Diagnostics Topic", "/diagnostics"',
            '"Autoware Localization Interface"',
            '"map -> base_link"',
            '"Output Rate"',
            '"published_count"',
            '"Registration"',
            '"GNSS State Color"',
            "classifyInterface(",
            "classifyRegistration(",
            "classifyGnss(",
            "lookupTransform(",
            "published_rate_estimator_",
            "PLUGINLIB_EXPORT_CLASS(",
        ):
            if token not in status_display_text:
                fail(f"localization-status RViz Display contract missing: {token}")

    plugin_path = ROOT / "src/pure_odometry_bringup/localization_status_plugin.xml"
    if not plugin_path.is_file():
        fail("localization-status RViz plugin description is missing")
    else:
        try:
            plugin_root = ET.parse(plugin_path).getroot()
        except ET.ParseError as exc:
            fail(f"cannot parse localization-status RViz plugin description: {exc}")
        else:
            plugin_classes = plugin_root.findall("class")
            if plugin_root.tag != "library" or plugin_root.attrib.get("path") != (
                "pure_odometry_localization_status_rviz_plugin"
            ):
                fail("localization-status RViz plugin library path is incorrect")
            if len(plugin_classes) != 1:
                fail("localization-status RViz plugin must export exactly one Display")
            else:
                plugin_class = plugin_classes[0]
                expected_attributes = {
                    "name": "pure_odometry_bringup/LocalizationStatus",
                    "type": "pure_odometry_bringup::LocalizationStatusDisplay",
                    "base_class_type": "rviz_common::Display",
                }
                for attribute, expected in expected_attributes.items():
                    if plugin_class.attrib.get(attribute) != expected:
                        fail(
                            "localization-status RViz plugin class has incorrect "
                            f"{attribute}: expected {expected}"
                        )

    wrapper_path = (
        ROOT
        / "src/pure_odometry_bringup/urdf/autoware_sample_vehicle_body.urdf.xacro"
    )
    if not wrapper_path.is_file():
        fail("Autoware sample-vehicle body xacro wrapper is missing")
        return
    try:
        wrapper_root = ET.fromstring(wrapper_path.read_text(encoding="utf-8"))
    except ET.ParseError as exc:
        fail(f"cannot parse sample-vehicle body xacro wrapper: {exc}")
        return
    if wrapper_root.tag != "robot":
        fail("sample-vehicle body xacro root must be robot")
    if wrapper_root.attrib.get("name") != "autoware_sample_vehicle_body_visualization":
        fail("sample-vehicle body xacro must use the documented visualization robot name")
    wrapper_text = wrapper_path.read_text(encoding="utf-8")
    for token in (
        'name="visual_z_offset" default="-1.66"',
        "xacro.load_yaml",
        "sample_vehicle_description)/config/vehicle_info.param.yaml",
        '<link name="base_link">',
        "package://sample_vehicle_description/mesh/lexus.dae",
        "No sensor kit",
    ):
        if token not in wrapper_text:
            fail(f"sample-vehicle body xacro contract missing: {token}")
    if len(wrapper_root.findall("link")) != 1 or wrapper_root.findall("joint"):
        fail("sample-vehicle body wrapper must generate one link and zero joints")
    if "<joint" in wrapper_text or "sensor_kit" in wrapper_text.lower():
        fail("sample-vehicle body wrapper must not add joints or a sensor kit")

    cmake_text = (ROOT / "src/pure_odometry_bringup/CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    if "install(DIRECTORY launch config urdf" not in cmake_text:
        fail("pure_odometry_bringup must install its visualization xacro")
    for token in (
        "add_executable(localization_visualization_node",
        "install(TARGETS localization_visualization_node",
        "set_target_properties(pure_odometry_localization_status_rviz_plugin PROPERTIES AUTOMOC ON)",
        "find_package(pluginlib REQUIRED)",
        "find_package(Qt5 REQUIRED COMPONENTS Widgets)",
        "find_package(rviz_common REQUIRED)",
        "add_library(pure_odometry_localization_status_rviz_plugin SHARED",
        "install(TARGETS pure_odometry_localization_status_rviz_plugin",
        "pluginlib_export_plugin_description_file(\n  rviz_common localization_status_plugin.xml",
    ):
        if token not in cmake_text:
            fail(f"pure_odometry_bringup presentation executable contract missing: {token}")
    package_text = (ROOT / "src/pure_odometry_bringup/package.xml").read_text(
        encoding="utf-8"
    )
    for dependency in ("robot_state_publisher", "rqt_robot_monitor", "xacro"):
        if f"<exec_depend>{dependency}</exec_depend>" not in package_text:
            fail(f"pure_odometry_bringup missing visualization dependency: {dependency}")
    for dependency in ("pluginlib", "rviz_common"):
        if f"<depend>{dependency}</depend>" not in package_text:
            fail(f"pure_odometry_bringup missing RViz plugin dependency: {dependency}")
    if "<build_depend>qtbase5-dev</build_depend>" not in package_text:
        fail("pure_odometry_bringup missing Qt build dependency for its RViz plugin")
    for dependency in ("libqt5-core", "libqt5-gui", "libqt5-widgets"):
        if f"<exec_depend>{dependency}</exec_depend>" not in package_text:
            fail(f"pure_odometry_bringup missing Qt runtime dependency: {dependency}")
    if "<exec_depend>sample_vehicle_description</exec_depend>" in package_text:
        fail("sample_vehicle_description must remain an optional Autoware-only dependency")

    launch_text = (
        ROOT
        / "src/pure_odometry_bringup/launch/autoware_lsim_localization.launch.py"
    ).read_text(encoding="utf-8")
    for token in (
        '"sample_vehicle_visual_z_offset"',
        'default_value="-1.66"',
        '"launch_localization_visualization"',
        'executable="localization_visualization_node"',
        'name="localization_visualization_node"',
        '"localization_visualization_covariance_z_offset_m"',
    ):
        if token not in launch_text:
            fail(f"Autoware integration presentation contract missing: {token}")


def check_container_runner() -> None:
    path = ROOT / "docker/autoware_lsim/container_entrypoint.sh"
    if not path.is_file():
        fail("container runner is missing")
        return
    text = path.read_text(encoding="utf-8")
    for token in (
        "autoware_lsim_localization.launch.py",
        "set +u",
        "play_localization_bag.sh",
        "ros2 bag record",
        "ros2 topic echo --once /localization/gyro_lidar_odom",
        "ros2 topic pub --once /initialpose",
        "param_xt_lidar_imu_only.yaml",
        'NMEA_GNSS_OVERRIDE_PARAM="$EMPTY_PARAM"',
        "NMEA_PROJECTOR_METADATA",
        "nmea_projector_metadata",
        "map_projector_info.yaml",
        "config/evaluation/lidar_imu_gnss/hesai_32line_rtk/accepted/gnss_fusion_single_antenna.yaml",
        "--clock-frequency",
        "autoware_lsim_output_recorder",
        "first_kinematic_state.yaml",
        "analyze_autoware_lsim_output.py",
        "precision_overlay.launch.py",
        "submap_snapshot_override.yaml",
        "--tracking-mode scan_to_scan",
        "/localization/submap_scan",
        "/localization/submap_correction",
        "/localization/precision_local_odom",
        "/localization/precision_global_odom",
        "/localization/precision_global_pose",
        "precision_launch_was_alive",
        "validate_precision_bag.py",
        "validation.log",
        "launch_was_alive",
        "record_was_alive",
        "check_required_nodes",
        "wait_for_required_nodes",
        "/pointcloud_container",
        "/rviz2",
        "RQT_ROBOT_MONITOR",
        "rqt_robot_monitor",
        "/diagnostic_aggregator",
        "/diagnostics_agg",
        "rqt_robot_monitor_node_info.txt",
        "rqt_robot_monitor_was_alive",
        "RVIZ_SAMPLE_VEHICLE",
        "sample_vehicle_description",
        "/sample_vehicle_body_state_publisher",
        "LOCALIZATION_STATUS_PLUGIN_XML",
        "localization_status_plugin.xml",
        "LOCALIZATION_STATUS_PLUGIN_LIBRARY",
        "libpure_odometry_localization_status_rviz_plugin.so",
        "localization_status_plugin_description_sha256",
        "localization_status_plugin_library_sha256",
        "launch_localization_visualization",
        "sample_vehicle_visual_z_offset",
        "localization_visualization_covariance_z_offset_m",
        "RVIZ_SAMPLE_VEHICLE_Z_OFFSET",
        "/localization_visualization_node",
        "/localization/visualization/robot_description",
        "/localization/visualization/trajectory",
        "/localization/visualization/status_markers",
        "/diagnostics",
        "sample_vehicle_body_node_info.txt",
        "sample_vehicle_description_topic_info.txt",
        "localization_visualization_node_info.txt",
        "localization_trajectory_topic_info.txt",
        "localization_covariance_markers_topic_info.txt",
        "rviz_diagnostics_topic_info.txt",
        "localization_trajectory_snapshot.yaml",
        "localization_covariance_markers_snapshot.yaml",
        "rviz_node_info.txt",
        "docker_runtime.txt",
        "final_nodes.txt",
        "FIRST_STATE_WAIT_SEC",
        "DRAIN_WAIT_SEC",
        "effective_configurations.tsv",
        "sha256sum",
    ):
        if token not in text:
            fail(f"container runner required behavior missing: {token}")

    # Docker retains the historical scan_to_submap UI token as the precision
    # selector. It must start the external overlay while the odometer itself
    # stays on its scan-to-scan parameter file plus the output-only snapshot
    # override.
    for token in (
        'scan_to_submap) ODOM_OVERRIDE_PARAM="$SUBMAP_SNAPSHOT_OVERRIDE_PARAM"',
        'if [[ "$TRACKING_MODE" == scan_to_submap ]]; then',
        "ros2 launch pure_precision_bringup precision_overlay.launch.py",
        "--tracking-mode scan_to_scan",
    ):
        if token not in text:
            fail(f"container precision-isolation contract missing: {token}")
    for retired in (
        "param_scan_to_submap.yaml",
        "scan_to_submap_override.yaml",
        "nmea_site_origin.yaml",
    ):
        if retired in text:
            fail(f"container runner still references retired internal submap path: {retired}")

    record_section_start = text.find("record_topics=(")
    record_section_end = text.find('log "recording evaluation outputs', record_section_start)
    if record_section_start < 0 or record_section_end < 0:
        fail("container runner output-record topic section is missing")
    else:
        record_section = text[record_section_start:record_section_end]
        for live_only_topic in (
            "/localization/visualization/trajectory",
            "/localization/visualization/status_markers",
        ):
            if live_only_topic in record_section:
                fail(
                    "growing presentation topic must remain live-only instead of being "
                    f"recorded: {live_only_topic}"
                )

    override_path = (
        ROOT / "src/pure_precision_bringup/config/submap_snapshot_override.yaml"
    )
    override = load_yaml(override_path)
    parameters = override.get("/**", {}).get("ros__parameters", {})
    if parameters.get("lidar_odom.tracking_mode") != "scan_to_scan":
        fail("Docker precision snapshot override must keep odometer scan_to_scan")
    if parameters.get("lidar_odom.external_submap_snapshot.enable") is not True:
        fail("Docker precision snapshot override must enable accepted-scan output")


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
        environment["DISPLAY"] = ":99"
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
                "--rqt-robot-monitor",
                "--rviz-sample-vehicle",
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
            "RQT Robot Monitor:   true",
            "RViz sample vehicle: true",
            "RViz body Z offset:   -1.66 m",
            "build lsim",
            "run --rm --no-tty lsim",
        ):
            if token not in result.stdout:
                fail(f"Docker wrapper dry run missing: {token}")

        profile_result = subprocess.run(
            [
                str(wrapper),
                "--dry-run",
                "--bag",
                str(bag),
                "--output",
                str(output),
                "--profile",
                "hesai-rosbag23",
                "--rviz",
                "--rviz-sample-vehicle",
            ],
            cwd=ROOT,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        if profile_result.returncode != 0:
            fail(
                "Hesai ROSBAG2/3 Docker profile dry run failed:\n"
                + profile_result.stdout
                + profile_result.stderr
            )
            return
        for token in (
            "Dataset profile:     hesai_rosbag23",
            "Clock frequency:     100.0 Hz",
            "PointCloud input:    /pandar_points_ex",
            "IMU input:           /sensor/imu/data_raw",
            "NMEA input:          /sensor/gnss/nmea_sentence",
            "TF policy:           isolate-all",
            "RViz sample vehicle: true",
            "RViz body Z offset:   -1.66 m",
            "RQT Robot Monitor:   false",
        ):
            if token not in profile_result.stdout:
                fail(f"Hesai profile dry run missing: {token}")

        gui_cases = (
            (
                "headless",
                [],
                False,
                "RViz:                false",
                "RQT Robot Monitor:   false",
            ),
            (
                "RViz only",
                ["--rviz"],
                True,
                "RViz:                true",
                "RQT Robot Monitor:   false",
            ),
            (
                "Robot Monitor only",
                ["--rqt-robot-monitor"],
                True,
                "RViz:                false",
                "RQT Robot Monitor:   true",
            ),
            (
                "both GUI tools",
                ["--rviz", "--rqt-robot-monitor"],
                True,
                "RViz:                true",
                "RQT Robot Monitor:   true",
            ),
        )
        for case_name, options, expects_overlay, rviz_line, rqt_line in gui_cases:
            gui_result = subprocess.run(
                [
                    str(wrapper),
                    "--dry-run",
                    "--bag",
                    str(bag),
                    "--output",
                    str(output),
                    *options,
                ],
                cwd=ROOT,
                env=environment,
                text=True,
                capture_output=True,
                check=False,
            )
            if gui_result.returncode != 0:
                fail(
                    f"Docker wrapper {case_name} dry run failed:\n"
                    + gui_result.stdout
                    + gui_result.stderr
                )
                continue
            has_overlay = "compose.rviz.yaml" in gui_result.stdout
            if has_overlay != expects_overlay:
                fail(
                    f"Docker wrapper {case_name} overlay selection was "
                    f"{has_overlay}, expected {expects_overlay}"
                )
            for expected_line in (rviz_line, rqt_line):
                if expected_line not in gui_result.stdout:
                    fail(
                        f"Docker wrapper {case_name} dry run missing: "
                        f"{expected_line}"
                    )

        no_display_environment = dict(environment)
        no_display_environment.pop("DISPLAY", None)
        no_display_result = subprocess.run(
            [
                str(wrapper),
                "--dry-run",
                "--bag",
                str(bag),
                "--output",
                str(output),
                "--rqt-robot-monitor",
            ],
            cwd=ROOT,
            env=no_display_environment,
            text=True,
            capture_output=True,
            check=False,
        )
        if (
            no_display_result.returncode == 0
            or "require DISPLAY" not in no_display_result.stderr
        ):
            fail("--rqt-robot-monitor must fail closed without DISPLAY")

        no_rviz_result = subprocess.run(
            [
                str(wrapper),
                "--dry-run",
                "--bag",
                str(bag),
                "--output",
                str(output),
                "--rviz-sample-vehicle",
            ],
            cwd=ROOT,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        if no_rviz_result.returncode == 0 or "requires --rviz" not in no_rviz_result.stderr:
            fail("--rviz-sample-vehicle must fail closed without --rviz")

        full_vehicle_result = subprocess.run(
            [
                str(wrapper),
                "--dry-run",
                "--bag",
                str(bag),
                "--output",
                str(output),
                "--rviz",
                "--rviz-sample-vehicle",
                "--launch-vehicle",
            ],
            cwd=ROOT,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        if (
            full_vehicle_result.returncode == 0
            or "cannot be combined with --launch-vehicle"
            not in full_vehicle_result.stderr
        ):
            fail("sample body visualization must reject the full vehicle/sensor launch")

        custom_offset_result = subprocess.run(
            [
                str(wrapper),
                "--dry-run",
                "--bag",
                str(bag),
                "--output",
                str(output),
                "--rviz",
                "--rviz-sample-vehicle",
                "--rviz-sample-vehicle-z-offset",
                "-1.25",
            ],
            cwd=ROOT,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        if custom_offset_result.returncode != 0:
            fail(
                "sample-vehicle custom Z offset dry run failed:\n"
                + custom_offset_result.stdout
                + custom_offset_result.stderr
            )
        elif "RViz body Z offset:   -1.25 m" not in custom_offset_result.stdout:
            fail("sample-vehicle custom Z offset was not resolved by the wrapper")

        for invalid_offset in ("nan", "-3.01", "1.01"):
            invalid_offset_result = subprocess.run(
                [
                    str(wrapper),
                    "--dry-run",
                    "--bag",
                    str(bag),
                    "--output",
                    str(output),
                    "--rviz",
                    "--rviz-sample-vehicle",
                    "--rviz-sample-vehicle-z-offset",
                    invalid_offset,
                ],
                cwd=ROOT,
                env=environment,
                text=True,
                capture_output=True,
                check=False,
            )
            if (
                invalid_offset_result.returncode == 0
                or "must be between -3.0 and 1.0" not in invalid_offset_result.stderr
            ):
                fail(f"sample body visualization must reject Z offset {invalid_offset}")

        orphan_offset_result = subprocess.run(
            [
                str(wrapper),
                "--dry-run",
                "--bag",
                str(bag),
                "--output",
                str(output),
                "--rviz-sample-vehicle-z-offset",
                "-1.25",
            ],
            cwd=ROOT,
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        if (
            orphan_offset_result.returncode == 0
            or "requires --rviz-sample-vehicle" not in orphan_offset_result.stderr
        ):
            fail("sample body Z offset must fail closed without the sample body")


def main() -> int:
    check_compose()
    check_dockerfile()
    check_rviz_config()
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
