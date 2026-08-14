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
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory("pure_odometry_bringup")
    adapter_share = get_package_share_directory("pure_autoware_localization_adapter")

    default_empty_map_path = os.path.join(
        bringup_share, "config", "autoware_lsim", "empty_map"
    )
    default_empty_param = os.path.join(
        bringup_share, "config", "autoware_lsim", "empty_params.yaml"
    )
    default_adapter_param = os.path.join(adapter_share, "param", "param.yaml")
    default_rviz_config = os.path.join(
        bringup_share, "config", "autoware_lsim", "hesai_rosbag23.rviz"
    )
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

    launch_autoware = LaunchConfiguration("launch_autoware")
    vehicle_model = LaunchConfiguration("vehicle_model")
    sensor_model = LaunchConfiguration("sensor_model")
    autoware_map_path = LaunchConfiguration("autoware_map_path")
    launch_vehicle = LaunchConfiguration("launch_vehicle")
    launch_sensing = LaunchConfiguration("launch_sensing")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_sample_vehicle_visualization = LaunchConfiguration(
        "launch_sample_vehicle_visualization"
    )
    launch_localization_visualization = LaunchConfiguration(
        "launch_localization_visualization"
    )
    sample_vehicle_visual_z_offset = LaunchConfiguration(
        "sample_vehicle_visual_z_offset"
    )
    localization_visualization_covariance_z_offset_m = LaunchConfiguration(
        "localization_visualization_covariance_z_offset_m"
    )
    rviz_config = LaunchConfiguration("rviz_config")
    pointcloud_container_name = LaunchConfiguration("pointcloud_container_name")
    sensor_profile = LaunchConfiguration("sensor_profile")

    use_gnss = LaunchConfiguration("use_gnss")
    use_imu_deskew = LaunchConfiguration("use_imu_deskew")
    points_input_topic = LaunchConfiguration("points_input_topic")
    deskewed_points_topic = LaunchConfiguration("deskewed_points_topic")
    imu_input_topic = LaunchConfiguration("imu_input_topic")
    twist_input_topic = LaunchConfiguration("twist_input_topic")
    imu_param = LaunchConfiguration("imu_param")
    odom_param = LaunchConfiguration("odom_param")
    odom_override_param = LaunchConfiguration("odom_override_param")
    nmea_gnss_param = LaunchConfiguration("nmea_gnss_param")
    nmea_gnss_override_param = LaunchConfiguration("nmea_gnss_override_param")
    gnss_fusion_param = LaunchConfiguration("gnss_fusion_param")
    gnss_fusion_override_param = LaunchConfiguration("gnss_fusion_override_param")
    fusion_xy_only_recovery = LaunchConfiguration("fusion_xy_only_recovery")
    gnss_primary_gga_topic = LaunchConfiguration("gnss_primary_gga_topic")
    gnss_secondary_gga_topic = LaunchConfiguration("gnss_secondary_gga_topic")
    gnss_fix_velocity_topic = LaunchConfiguration("gnss_fix_velocity_topic")
    fused_odom_topic = LaunchConfiguration("fused_odom_topic")
    adapter_param = LaunchConfiguration("adapter_param")
    log_level = LaunchConfiguration("log_level")

    autoware = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("autoware_launch"), "launch", "autoware.launch.xml"]
            )
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
            # The standard Autoware RViz profile expects the full sensing and
            # localization stacks.  This launch owns a profile that displays
            # the localization-only topics published below.
            "rviz": "false",
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
            "imu_param": imu_param,
            "odom_param": odom_param,
            "odom_override_param": odom_override_param,
            "nmea_gnss_param": nmea_gnss_param,
            "nmea_gnss_override_param": nmea_gnss_override_param,
            "gnss_fusion_param": gnss_fusion_param,
            "fusion_xy_only_recovery": fusion_xy_only_recovery,
            "gnss_fusion_override_param": gnss_fusion_override_param,
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

    rviz = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_config],
    )

    sample_vehicle_body_xacro = PathJoinSubstitution(
        [
            FindPackageShare("pure_odometry_bringup"),
            "urdf",
            "autoware_sample_vehicle_body.urdf.xacro",
        ]
    )
    sample_vehicle_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                sample_vehicle_body_xacro,
                " visual_z_offset:=",
                sample_vehicle_visual_z_offset,
            ]
        ),
        value_type=str,
    )
    sample_vehicle_visualization_condition = IfCondition(
        PythonExpression(
            [
                "'",
                launch_sample_vehicle_visualization,
                "' == 'true' and '",
                launch_vehicle,
                "' != 'true'",
            ]
        )
    )
    sample_vehicle_visualization = Node(
        condition=sample_vehicle_visualization_condition,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="sample_vehicle_body_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": sample_vehicle_description,
            }
        ],
        remappings=[
            (
                "robot_description",
                "/localization/visualization/robot_description",
            ),
            ("/tf", "/localization/visualization/tf"),
            ("/tf_static", "/localization/visualization/tf_static"),
        ],
    )

    localization_visualization = Node(
        condition=IfCondition(launch_localization_visualization),
        package="pure_odometry_bringup",
        executable="localization_visualization_node",
        name="localization_visualization_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "covariance_z_offset_m": ParameterValue(
                    localization_visualization_covariance_z_offset_m,
                    value_type=float,
                ),
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    hesai_profile_condition = IfCondition(
        PythonExpression(["'", sensor_profile, "' == 'hesai_rosbag23'"])
    )
    hesai_static_transforms = [
        Node(
            condition=hesai_profile_condition,
            package="tf2_ros",
            executable="static_transform_publisher",
            name="hesai_lidar_static_transform",
            output="screen",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "lidar/0",
            ],
        ),
        Node(
            condition=hesai_profile_condition,
            package="tf2_ros",
            executable="static_transform_publisher",
            name="hesai_imu_static_transform",
            output="screen",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "-0.1874",
                "--roll", "3.14159", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "imu",
            ],
        ),
        Node(
            condition=hesai_profile_condition,
            package="tf2_ros",
            executable="static_transform_publisher",
            name="hesai_gnss_static_transform",
            output="screen",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "-0.1326",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "gnss/0",
            ],
        ),
    ]

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
                "launch_sample_vehicle_visualization",
                default_value="false",
                description=(
                    "Publish the Autoware sample Lexus body for RViz only. "
                    "The body is illustrative, not recorded vehicle geometry "
                    "or sensor calibration, and requires launch_vehicle=false."
                ),
            ),
            DeclareLaunchArgument(
                "sample_vehicle_visual_z_offset",
                default_value="-1.66",
                description=(
                    "Visualization-only vertical mesh offset from the estimated "
                    "base_link (the Hesai profile places base_link at the roof LiDAR)."
                ),
            ),
            DeclareLaunchArgument(
                "launch_localization_visualization",
                default_value="false",
                description=(
                    "Publish a downsampled trajectory and XY-only covariance "
                    "ellipse; status is shown in the RViz Displays panel."
                ),
            ),
            DeclareLaunchArgument(
                "localization_visualization_covariance_z_offset_m",
                default_value="0.0",
                description=(
                    "Vertical offset for the XY covariance ellipse only. "
                    "Use sample_vehicle_visual_z_offset when aligning it to the "
                    "illustrative sample vehicle ground plane."
                ),
            ),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument(
                "pointcloud_container_name", default_value="pointcloud_container"
            ),
            DeclareLaunchArgument(
                "sensor_profile",
                default_value="generic",
                description=(
                    "Sensor static-TF profile. Use hesai_rosbag23 only for the "
                    "calibrated ROSBAG2/3 rig."
                ),
            ),
            DeclareLaunchArgument("use_gnss", default_value="false"),
            DeclareLaunchArgument("use_imu_deskew", default_value="true"),
            DeclareLaunchArgument("points_input_topic", default_value="/points_raw"),
            DeclareLaunchArgument(
                "deskewed_points_topic", default_value="/localization/points_undistorted"
            ),
            DeclareLaunchArgument("imu_input_topic", default_value="/imu"),
            DeclareLaunchArgument("twist_input_topic", default_value=""),
            DeclareLaunchArgument("imu_param", default_value=default_imu_param),
            DeclareLaunchArgument("odom_param", default_value=default_odom_param),
            DeclareLaunchArgument("odom_override_param", default_value=default_empty_param),
            DeclareLaunchArgument(
                "nmea_gnss_param", default_value=default_nmea_gnss_param
            ),
            DeclareLaunchArgument(
                "nmea_gnss_override_param", default_value=default_empty_param
            ),
            DeclareLaunchArgument(
                "gnss_fusion_param", default_value=default_gnss_fusion_param
            ),
            DeclareLaunchArgument(
                "gnss_fusion_override_param", default_value=default_empty_param
            ),
            DeclareLaunchArgument(
                "fusion_xy_only_recovery",
                default_value="false",
                description=(
                    "Deprecated compatibility switch. Prefer "
                    "gnss_fusion_override_param for deployment profiles."
                ),
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
            rviz,
            sample_vehicle_visualization,
            localization_visualization,
            *hesai_static_transforms,
            localizer,
            adapter,
        ]
    )
