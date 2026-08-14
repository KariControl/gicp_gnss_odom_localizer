# SPDX-License-Identifier: Apache-2.0
"""Minimal, isolated LiDAR/IMU odometry launch for performance evaluation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    default_imu_param = os.path.join(
        get_package_share_directory("pure_imu_undistortion"), "param", "param.yaml"
    )
    default_odom_param = os.path.join(
        get_package_share_directory("pure_lidar_gyro_odometer"), "param", "param.yaml"
    )
    default_empty_param = os.path.join(
        get_package_share_directory("pure_odometry_bringup"),
        "config",
        "autoware_lsim",
        "empty_params.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_imu_deskew = LaunchConfiguration("use_imu_deskew")
    points_input_topic = LaunchConfiguration("points_input_topic")
    deskewed_points_topic = LaunchConfiguration("deskewed_points_topic")
    imu_input_topic = LaunchConfiguration("imu_input_topic")
    imu_param = LaunchConfiguration("imu_param")
    odom_param = LaunchConfiguration("odom_param")
    odom_override_param = LaunchConfiguration("odom_override_param")
    odom_aux_override_param = LaunchConfiguration("odom_aux_override_param")
    odom_tuning_override_param = LaunchConfiguration("odom_tuning_override_param")
    log_level = LaunchConfiguration("log_level")

    common_extra_arguments = [{"use_intra_process_comms": True}]
    imu_undistorter = ComposableNode(
        package="pure_imu_undistortion",
        plugin="pure_imu_undistortion::ImuUndistorter",
        name="imu_undistorter",
        parameters=[
            imu_param,
            {
                "use_sim_time": use_sim_time,
                "points_in_topic": points_input_topic,
                "points_out_topic": deskewed_points_topic,
                "imu_topic": imu_input_topic,
                "twist_topic": "",
            },
        ],
        extra_arguments=common_extra_arguments,
    )
    gyro_deskewed = ComposableNode(
        package="pure_lidar_gyro_odometer",
        plugin="pure_gyro_odometer::GyroOdometerNode",
        name="gyro_odometer",
        parameters=[
            odom_param,
            odom_override_param,
            odom_aux_override_param,
            odom_tuning_override_param,
            {
                "use_sim_time": use_sim_time,
                "points_topic": deskewed_points_topic,
                "imu_topic": imu_input_topic,
            },
        ],
        extra_arguments=common_extra_arguments,
    )
    gyro_direct = ComposableNode(
        package="pure_lidar_gyro_odometer",
        plugin="pure_gyro_odometer::GyroOdometerNode",
        name="gyro_odometer",
        parameters=[
            odom_param,
            odom_override_param,
            odom_aux_override_param,
            odom_tuning_override_param,
            {
                "use_sim_time": use_sim_time,
                "points_topic": points_input_topic,
                "imu_topic": imu_input_topic,
            },
        ],
        extra_arguments=common_extra_arguments,
    )

    deskew_container = ComposableNodeContainer(
        condition=IfCondition(use_imu_deskew),
        name="lidar_imu_only_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[imu_undistorter, gyro_deskewed],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )
    direct_container = ComposableNodeContainer(
        condition=UnlessCondition(use_imu_deskew),
        name="lidar_imu_only_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[gyro_direct],
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_imu_deskew", default_value="true"),
            DeclareLaunchArgument("points_input_topic", default_value="/points_raw"),
            DeclareLaunchArgument(
                "deskewed_points_topic",
                default_value="/localization/points_undistorted",
            ),
            DeclareLaunchArgument("imu_input_topic", default_value="/imu"),
            DeclareLaunchArgument("imu_param", default_value=default_imu_param),
            DeclareLaunchArgument("odom_param", default_value=default_odom_param),
            DeclareLaunchArgument(
                "odom_override_param", default_value=default_empty_param
            ),
            DeclareLaunchArgument(
                "odom_aux_override_param", default_value=default_empty_param
            ),
            DeclareLaunchArgument(
                "odom_tuning_override_param", default_value=default_empty_param
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            deskew_container,
            direct_container,
        ]
    )
