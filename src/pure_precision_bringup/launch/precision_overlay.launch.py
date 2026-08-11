# SPDX-License-Identifier: Apache-2.0
"""Launch only the additive precision branch.

The existing scan-to-scan odometer and GNSS map->odom fusion remain owned by
pure_odometry_bringup.  This overlay deliberately publishes no TF and uses
separate precision topics, so it cannot replace or feed back into the existing
fusion path during evaluation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    matcher_param_default = os.path.join(
        get_package_share_directory("pure_lidar_submap_matcher"),
        "param",
        "param.yaml",
    )
    global_param_default = os.path.join(
        get_package_share_directory("pure_precision_global_localizer"),
        "param",
        "param.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    matcher_param = LaunchConfiguration("matcher_param")
    global_param = LaunchConfiguration("global_param")
    log_level = LaunchConfiguration("log_level")

    matcher = Node(
        package="pure_lidar_submap_matcher",
        executable="pure_lidar_submap_matcher_node",
        name="submap_matcher",
        output="screen",
        parameters=[matcher_param, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )
    precision_global = Node(
        package="pure_precision_global_localizer",
        executable="pure_precision_global_localizer_node",
        name="precision_global_localizer",
        output="screen",
        parameters=[global_param, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("matcher_param", default_value=matcher_param_default),
            DeclareLaunchArgument("global_param", default_value=global_param_default),
            DeclareLaunchArgument("log_level", default_value="info"),
            matcher,
            precision_global,
        ]
    )
