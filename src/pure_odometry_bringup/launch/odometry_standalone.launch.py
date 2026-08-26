# SPDX-License-Identifier: Apache-2.0
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    default_imu_param = os.path.join(
        get_package_share_directory("pure_imu_undistortion"), "param", "param.yaml"
    )
    default_odom_param = os.path.join(
        get_package_share_directory("pure_lidar_gyro_odometer"), "param", "param.yaml"
    )
    default_nmea_gnss_param = os.path.join(
        get_package_share_directory("pure_nmea_gnss_conversion"), "param", "param.yaml"
    )
    default_gnss_fusion_param = os.path.join(
        get_package_share_directory("pure_gnss_map_odom_fusion"), "param", "param.yaml"
    )
    default_empty_param = os.path.join(
        get_package_share_directory("pure_odometry_bringup"),
        "config",
        "autoware_lsim",
        "empty_params.yaml",
    )
    diag_agg_param = os.path.join(
        get_package_share_directory("pure_odometry_bringup"),
        "config",
        "diagnostic_aggregator.yaml",
    )

    imu_param = LaunchConfiguration("imu_param")
    odom_param = LaunchConfiguration("odom_param")
    odom_override_param = LaunchConfiguration("odom_override_param")
    nmea_gnss_param = LaunchConfiguration("nmea_gnss_param")
    nmea_gnss_override_param = LaunchConfiguration("nmea_gnss_override_param")
    gnss_fusion_param = LaunchConfiguration("gnss_fusion_param")
    gnss_fusion_override_param = LaunchConfiguration("gnss_fusion_override_param")
    use_gnss = LaunchConfiguration("use_gnss")
    use_map_odom_fusion = LaunchConfiguration("use_map_odom_fusion")
    use_imu_deskew = LaunchConfiguration("use_imu_deskew")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    points_input_topic = LaunchConfiguration("points_input_topic")
    deskewed_points_topic = LaunchConfiguration("deskewed_points_topic")
    imu_input_topic = LaunchConfiguration("imu_input_topic")
    twist_input_topic = LaunchConfiguration("twist_input_topic")
    odom_publish_tf = LaunchConfiguration("odom_publish_tf")
    fused_odom_topic = LaunchConfiguration("fused_odom_topic")
    fused_pose_topic = LaunchConfiguration("fused_pose_topic")
    fusion_publish_tf = LaunchConfiguration("fusion_publish_tf")
    fusion_xy_only_recovery = LaunchConfiguration("fusion_xy_only_recovery")

    selected_points_topic = PythonExpression(
        [
            "'",
            deskewed_points_topic,
            "' if '",
            use_imu_deskew,
            "'.lower() == 'true' else '",
            points_input_topic,
            "'",
        ]
    )
    use_fusion = PythonExpression(
        [
            "'",
            use_gnss,
            "'.lower() == 'true' or '",
            use_map_odom_fusion,
            "'.lower() == 'true'",
        ]
    )

    gnss_fix_velocity_topic = LaunchConfiguration("gnss_fix_velocity_topic")
    gnss_global_pose_topic = LaunchConfiguration("gnss_global_pose_topic")
    gnss_odom_topic = LaunchConfiguration("gnss_odom_topic")
    gnss_fusion_input_topic = LaunchConfiguration("gnss_fusion_input_topic")
    gnss_primary_gga_topic = LaunchConfiguration("gnss_primary_gga_topic")
    gnss_secondary_gga_topic = LaunchConfiguration("gnss_secondary_gga_topic")
    gnss_global_pose_cov_topic = LaunchConfiguration("gnss_global_pose_cov_topic")
    gnss_confidence_topic = LaunchConfiguration("gnss_confidence_topic")
    use_secondary_gga = LaunchConfiguration("use_secondary_gga")
    use_doppler_heading = LaunchConfiguration("use_doppler_heading")
    use_imu_yaw_rate_heading = LaunchConfiguration("use_imu_yaw_rate_heading")
    imu_corrected_topic = LaunchConfiguration("imu_corrected_topic")
    stopped_topic = LaunchConfiguration("stopped_topic")

    imu_undistorter = Node(
        condition=IfCondition(use_imu_deskew),
        package="pure_imu_undistortion",
        executable="imu_undistorter_node",
        name="imu_undistorter",
        output="screen",
        parameters=[
            imu_param,
            {
                "use_sim_time": use_sim_time,
                "points_in_topic": points_input_topic,
                "points_out_topic": deskewed_points_topic,
                "imu_topic": imu_input_topic,
                "twist_topic": twist_input_topic,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    gyro_odometer = Node(
        package="pure_lidar_gyro_odometer",
        executable="pure_lidar_gyro_odometer_node",
        name="gyro_odometer",
        output="screen",
        parameters=[
            odom_param,
            odom_override_param,
            {
                "use_sim_time": use_sim_time,
                "points_topic": selected_points_topic,
                "imu_topic": imu_input_topic,
                "publish_tf": odom_publish_tf,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    gnss_conversion_nmea = Node(
        condition=IfCondition(use_gnss),
        package="pure_nmea_gnss_conversion",
        executable="pure_nmea_gga_conversion",
        name="nmea_gga_conversion",
        output="screen",
        parameters=[
            nmea_gnss_param,
            nmea_gnss_override_param,
            {
                "use_sim_time": use_sim_time,
                "use_secondary_gga": use_secondary_gga,
                "use_doppler_heading": use_doppler_heading,
                "use_imu_yaw_rate_heading": use_imu_yaw_rate_heading,
            },
        ],
        remappings=[
            ("primary_gga", gnss_primary_gga_topic),
            ("secondary_gga", gnss_secondary_gga_topic),
            ("fix_velocity", gnss_fix_velocity_topic),
            ("imu_corrected", imu_corrected_topic),
            ("stopped", stopped_topic),
            ("global_pose", gnss_global_pose_topic),
            ("global_pose_with_covariance", gnss_global_pose_cov_topic),
            ("gnss_odometry", gnss_odom_topic),
            ("gnss_fusion_input", gnss_fusion_input_topic),
            ("gnss_confidence", gnss_confidence_topic),
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    gnss_map_odom_fusion = Node(
        condition=IfCondition(use_fusion),
        package="pure_gnss_map_odom_fusion",
        executable="pure_gnss_map_odom_fusion_node",
        name="gnss_map_odom_fusion",
        output="screen",
        parameters=[
            gnss_fusion_param,
            {"xy_only_recovery.enabled": fusion_xy_only_recovery},
            gnss_fusion_override_param,
            {
                "use_sim_time": use_sim_time,
                "gnss_input_topic": gnss_fusion_input_topic,
                "out_odom_topic": fused_odom_topic,
                "out_pose_topic": fused_pose_topic,
                "publish_tf": fusion_publish_tf,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    diagnostic_aggregator_node = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        name="diagnostic_aggregator",
        output="screen",
        parameters=[diag_agg_param, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("imu_param", default_value=default_imu_param),
            DeclareLaunchArgument("odom_param", default_value=default_odom_param),
            DeclareLaunchArgument("odom_override_param", default_value=default_empty_param),
            DeclareLaunchArgument("nmea_gnss_param", default_value=default_nmea_gnss_param),
            DeclareLaunchArgument(
                "nmea_gnss_override_param", default_value=default_empty_param
            ),
            DeclareLaunchArgument("gnss_fusion_param", default_value=default_gnss_fusion_param),
            DeclareLaunchArgument(
                "gnss_fusion_override_param", default_value=default_empty_param
            ),
            DeclareLaunchArgument("use_gnss", default_value="false"),
            DeclareLaunchArgument(
                "use_map_odom_fusion",
                default_value="false",
                description=(
                    "Launch map-to-odom fusion without the NMEA frontend. Enable this for "
                    "manual /initialpose anchoring or external GnssFusionInput sources."
                ),
            ),
            DeclareLaunchArgument(
                "use_imu_deskew",
                default_value="true",
                description=(
                    "Run strict IMU deskew. Set false only for an already deskewed point cloud."
                ),
            ),
            DeclareLaunchArgument("points_input_topic", default_value="/points_raw"),
            DeclareLaunchArgument(
                "deskewed_points_topic", default_value="/localization/points_undistorted"
            ),
            DeclareLaunchArgument("imu_input_topic", default_value="/imu"),
            DeclareLaunchArgument("twist_input_topic", default_value=""),
            DeclareLaunchArgument(
                "odom_publish_tf",
                default_value="true",
                description="Publish the odom -> base_link transform from the odometer.",
            ),
            DeclareLaunchArgument("fused_odom_topic", default_value="/localization/ekf_odom"),
            DeclareLaunchArgument("fused_pose_topic", default_value="/localization/ekf_pose"),
            DeclareLaunchArgument("fusion_publish_tf", default_value="true"),
            DeclareLaunchArgument(
                "fusion_xy_only_recovery",
                default_value="false",
                description=(
                    "Deprecated compatibility switch for guarded translation-only "
                    "GNSS recovery. Prefer gnss_fusion_override_param."
                ),
            ),
            DeclareLaunchArgument(
                "gnss_fix_velocity_topic", default_value="/ublox_gps_node/fix_velocity"
            ),
            DeclareLaunchArgument(
                "gnss_global_pose_topic", default_value="/localization/global_pose"
            ),
            DeclareLaunchArgument(
                "gnss_odom_topic", default_value="/localization/gnss_odometry"
            ),
            DeclareLaunchArgument(
                "gnss_fusion_input_topic", default_value="/localization/gnss_fusion_input"
            ),
            DeclareLaunchArgument("gnss_primary_gga_topic", default_value="/nmea_sentence"),
            DeclareLaunchArgument(
                "gnss_secondary_gga_topic", default_value="/nmea_sentence_secondary"
            ),
            DeclareLaunchArgument(
                "gnss_global_pose_cov_topic",
                default_value="/localization/global_pose_with_covariance",
            ),
            DeclareLaunchArgument(
                "gnss_confidence_topic", default_value="/localization/gnss_confidence"
            ),
            DeclareLaunchArgument("use_secondary_gga", default_value="false"),
            DeclareLaunchArgument("use_doppler_heading", default_value="false"),
            DeclareLaunchArgument("use_imu_yaw_rate_heading", default_value="true"),
            DeclareLaunchArgument(
                "imu_corrected_topic", default_value="/localization/imu_corrected"
            ),
            DeclareLaunchArgument(
                "stopped_topic", default_value="/localization/is_stopped"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            imu_undistorter,
            gyro_odometer,
            gnss_conversion_nmea,
            gnss_map_odom_fusion,
            diagnostic_aggregator_node,
        ]
    )
