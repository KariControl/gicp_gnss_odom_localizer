import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    primary_gga_topic = LaunchConfiguration("primary_gga_topic")
    secondary_gga_topic = LaunchConfiguration("secondary_gga_topic")
    fix_velocity_topic = LaunchConfiguration("fix_velocity_topic")

    global_pose_topic = LaunchConfiguration("global_pose_topic")
    global_pose_cov_topic = LaunchConfiguration("global_pose_cov_topic")
    gnss_odom_topic = LaunchConfiguration("gnss_odom_topic")
    gnss_fusion_input_topic = LaunchConfiguration("gnss_fusion_input_topic")
    gnss_confidence_topic = LaunchConfiguration("gnss_confidence_topic")

    use_secondary_gga = LaunchConfiguration("use_secondary_gga")
    use_doppler_heading = LaunchConfiguration("use_doppler_heading")
    use_imu_yaw_rate_heading = LaunchConfiguration("use_imu_yaw_rate_heading")

    imu_corrected_topic = LaunchConfiguration("imu_corrected_topic")
    stopped_topic = LaunchConfiguration("stopped_topic")

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=[
            TextSubstitution(
                text=os.path.join(
                    get_package_share_directory("pure_nmea_gnss_conversion"),
                    "param",
                    "param.yaml",
                )
            )
        ],
        description="pure_nmea_gnss_conversion parameter file path",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time",
    )

    declare_primary_gga_topic = DeclareLaunchArgument(
        "primary_gga_topic",
        default_value="/nmea_sentence",
        description="Primary NMEA Sentence topic. Non-GGA sentences are ignored.",
    )

    declare_secondary_gga_topic = DeclareLaunchArgument(
        "secondary_gga_topic",
        default_value="/nmea_sentence_secondary",
        description="Secondary NMEA Sentence topic for dual-antenna mode.",
    )

    declare_fix_velocity_topic = DeclareLaunchArgument(
        "fix_velocity_topic",
        default_value="/ublox_gps_node/fix_velocity",
        description="Optional GNSS Doppler velocity topic",
    )

    declare_global_pose_topic = DeclareLaunchArgument(
        "global_pose_topic",
        default_value="/localization/global_pose",
        description="Projected GNSS pose output topic",
    )

    declare_global_pose_cov_topic = DeclareLaunchArgument(
        "global_pose_cov_topic",
        default_value="/localization/global_pose_with_covariance",
        description="Projected GNSS pose with covariance output topic",
    )

    declare_gnss_odom_topic = DeclareLaunchArgument(
        "gnss_odom_topic",
        default_value="/localization/gnss_odometry",
        description="Projected GNSS odometry output topic",
    )

    declare_gnss_fusion_input_topic = DeclareLaunchArgument(
        "gnss_fusion_input_topic",
        default_value="/localization/gnss_fusion_input",
        description="Dedicated GNSS input topic for pure_gnss_map_odom_fusion",
    )

    declare_gnss_confidence_topic = DeclareLaunchArgument(
        "gnss_confidence_topic",
        default_value="/localization/gnss_confidence",
        description="FIX+HDOP based confidence output topic",
    )

    declare_use_secondary_gga = DeclareLaunchArgument(
        "use_secondary_gga",
        default_value="false",
        description="Enable dual-antenna mode",
    )

    declare_use_doppler_heading = DeclareLaunchArgument(
        "use_doppler_heading",
        default_value="false",
        description="Use fix_velocity as heading source",
    )

    declare_use_imu_yaw_rate_heading = DeclareLaunchArgument(
        "use_imu_yaw_rate_heading",
        default_value="true",
        description="Use corrected IMU yaw-rate for trajectory correction and bounded propagation",
    )

    declare_imu_corrected_topic = DeclareLaunchArgument(
        "imu_corrected_topic",
        default_value="/localization/imu_corrected",
        description="Bias-corrected IMU topic from pure_lidar_gyro_odometer",
    )

    declare_stopped_topic = DeclareLaunchArgument(
        "stopped_topic",
        default_value="/localization/is_stopped",
        description="Vehicle stop detection topic from pure_lidar_gyro_odometer",
    )

    node = Node(
        package="pure_nmea_gnss_conversion",
        executable="pure_nmea_gga_conversion",
        name="nmea_gga_conversion",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "use_secondary_gga": use_secondary_gga,
                "use_doppler_heading": use_doppler_heading,
                "use_imu_yaw_rate_heading": use_imu_yaw_rate_heading,
            },
        ],
        remappings=[
            ("primary_gga", primary_gga_topic),
            ("secondary_gga", secondary_gga_topic),
            ("fix_velocity", fix_velocity_topic),
            ("imu_corrected", imu_corrected_topic),
            ("stopped", stopped_topic),
            ("global_pose", global_pose_topic),
            ("global_pose_with_covariance", global_pose_cov_topic),
            ("gnss_odometry", gnss_odom_topic),
            ("gnss_fusion_input", gnss_fusion_input_topic),
            ("gnss_confidence", gnss_confidence_topic),
        ],
    )

    return LaunchDescription([
        declare_params_file,
        declare_use_sim_time,
        declare_primary_gga_topic,
        declare_secondary_gga_topic,
        declare_fix_velocity_topic,
        declare_global_pose_topic,
        declare_global_pose_cov_topic,
        declare_gnss_odom_topic,
        declare_gnss_fusion_input_topic,
        declare_gnss_confidence_topic,
        declare_use_secondary_gga,
        declare_use_doppler_heading,
        declare_use_imu_yaw_rate_heading,
        declare_imu_corrected_topic,
        declare_stopped_topic,
        node,
    ])
