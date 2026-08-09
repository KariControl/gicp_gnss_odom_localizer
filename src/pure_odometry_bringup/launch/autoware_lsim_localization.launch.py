# SPDX-License-Identifier: Apache-2.0
"""Localization-only Autoware logging-simulation integration.

Autoware's standard localization, map, perception, planning, control, system and API
components are disabled. The GICP/GNSS stack publishes the Autoware localization
runtime topics through pure_autoware_localization_adapter.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("pure_odometry_bringup")
    adapter_share = get_package_share_directory("pure_autoware_localization_adapter")
    autoware_share = get_package_share_directory("autoware_launch")

    default_empty_map_path = os.path.join(
        bringup_share, "config", "autoware_lsim", "empty_map"
    )
    default_adapter_param = os.path.join(adapter_share, "param", "param.yaml")
    default_odom_param = os.path.join(
        get_package_share_directory("pure_lidar_gyro_odometer"), "param", "param.yaml"
    )
    default_nmea_gnss_param = os.path.join(
        get_package_share_directory("pure_nmea_gnss_conversion"), "param", "param.yaml"
    )
    default_gnss_fusion_param = os.path.join(
        get_package_share_directory("pure_gnss_map_odom_fusion"), "param", "param.yaml"
    )

    launch_autoware = LaunchConfiguration("launch_autoware")
    vehicle_model = LaunchConfiguration("vehicle_model")
    sensor_model = LaunchConfiguration("sensor_model")
    autoware_map_path = LaunchConfiguration("autoware_map_path")
    launch_vehicle = LaunchConfiguration("launch_vehicle")
    launch_sensing = LaunchConfiguration("launch_sensing")
    launch_rviz = LaunchConfiguration("launch_rviz")
    pointcloud_container_name = LaunchConfiguration("pointcloud_container_name")

    use_gnss = LaunchConfiguration("use_gnss")
    use_imu_deskew = LaunchConfiguration("use_imu_deskew")
    points_input_topic = LaunchConfiguration("points_input_topic")
    deskewed_points_topic = LaunchConfiguration("deskewed_points_topic")
    imu_input_topic = LaunchConfiguration("imu_input_topic")
    twist_input_topic = LaunchConfiguration("twist_input_topic")
    odom_param = LaunchConfiguration("odom_param")
    nmea_gnss_param = LaunchConfiguration("nmea_gnss_param")
    gnss_fusion_param = LaunchConfiguration("gnss_fusion_param")
    gnss_primary_gga_topic = LaunchConfiguration("gnss_primary_gga_topic")
    gnss_secondary_gga_topic = LaunchConfiguration("gnss_secondary_gga_topic")
    gnss_fix_velocity_topic = LaunchConfiguration("gnss_fix_velocity_topic")
    fused_odom_topic = LaunchConfiguration("fused_odom_topic")
    adapter_param = LaunchConfiguration("adapter_param")
    log_level = LaunchConfiguration("log_level")

    autoware = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(autoware_share, "launch", "autoware.launch.xml")
        ),
        condition=IfCondition(launch_autoware),
        launch_arguments={
            "map_path": autoware_map_path,
            "vehicle_model": vehicle_model,
            "sensor_model": sensor_model,
            "pointcloud_container_name": pointcloud_container_name,
            "launch_vehicle": launch_vehicle,
            "launch_vehicle_interface": "false",
            "launch_system": "false",
            "launch_map": "false",
            "launch_sensing": launch_sensing,
            "launch_sensing_driver": "false",
            "launch_localization": "false",
            "launch_perception": "false",
            "launch_planning": "false",
            "launch_control": "false",
            "launch_api": "false",
            "use_sim_time": "true",
            "system_run_mode": "logging_simulation",
            "rviz": launch_rviz,
            "is_simulation": "true",
        }.items(),
    )

    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "odometry_container.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_gnss": use_gnss,
            "use_map_odom_fusion": "true",
            "use_imu_deskew": use_imu_deskew,
            "points_input_topic": points_input_topic,
            "deskewed_points_topic": deskewed_points_topic,
            "imu_input_topic": imu_input_topic,
            "twist_input_topic": twist_input_topic,
            "odom_param": odom_param,
            "nmea_gnss_param": nmea_gnss_param,
            "gnss_fusion_param": gnss_fusion_param,
            "gnss_primary_gga_topic": gnss_primary_gga_topic,
            "gnss_secondary_gga_topic": gnss_secondary_gga_topic,
            "gnss_fix_velocity_topic": gnss_fix_velocity_topic,
            "fused_odom_topic": fused_odom_topic,
            # The adapter publishes map -> base_link directly. Avoid a second,
            # incomplete map -> odom TF chain in localization-only LSim.
            "fusion_publish_tf": "false",
            "log_level": log_level,
        }.items(),
    )

    adapter = Node(
        package="pure_autoware_localization_adapter",
        executable="autoware_localization_adapter_node",
        name="autoware_localization_adapter",
        output="screen",
        parameters=[
            adapter_param,
            {
                "use_sim_time": True,
                "input_odom_topic": fused_odom_topic,
                "publish_tf": True,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_autoware", default_value="true"),
            DeclareLaunchArgument("vehicle_model", default_value="sample_vehicle"),
            DeclareLaunchArgument("sensor_model", default_value="sample_sensor_kit"),
            DeclareLaunchArgument("autoware_map_path", default_value=default_empty_map_path),
            DeclareLaunchArgument(
                "launch_vehicle",
                default_value="true",
                description="Launch vehicle/sensor descriptions and their static TFs.",
            ),
            DeclareLaunchArgument(
                "launch_sensing",
                default_value="false",
                description=(
                    "Launch Autoware sensing preprocessing only when the bag requires it. "
                    "The default consumes replayed LiDAR/IMU topics directly."
                ),
            ),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "pointcloud_container_name", default_value="pointcloud_container"
            ),
            DeclareLaunchArgument("use_gnss", default_value="false"),
            DeclareLaunchArgument("use_imu_deskew", default_value="true"),
            DeclareLaunchArgument("points_input_topic", default_value="/points_raw"),
            DeclareLaunchArgument(
                "deskewed_points_topic", default_value="/localization/points_undistorted"
            ),
            DeclareLaunchArgument("imu_input_topic", default_value="/imu"),
            DeclareLaunchArgument("twist_input_topic", default_value=""),
            DeclareLaunchArgument("odom_param", default_value=default_odom_param),
            DeclareLaunchArgument(
                "nmea_gnss_param", default_value=default_nmea_gnss_param
            ),
            DeclareLaunchArgument(
                "gnss_fusion_param", default_value=default_gnss_fusion_param
            ),
            DeclareLaunchArgument("gnss_primary_gga_topic", default_value="/nmea_sentence"),
            DeclareLaunchArgument(
                "gnss_secondary_gga_topic", default_value="/nmea_sentence_secondary"
            ),
            DeclareLaunchArgument(
                "gnss_fix_velocity_topic", default_value="/ublox_gps_node/fix_velocity"
            ),
            DeclareLaunchArgument("fused_odom_topic", default_value="/localization/ekf_odom"),
            DeclareLaunchArgument("adapter_param", default_value=default_adapter_param),
            DeclareLaunchArgument("log_level", default_value="info"),
            autoware,
            localizer,
            adapter,
        ]
    )
