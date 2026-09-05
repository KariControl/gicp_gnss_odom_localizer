// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace pure_localization_interface_adapter
{

inline geometry_msgs::msg::TwistWithCovarianceStamped makeTwistWithCovarianceStamped(
  const nav_msgs::msg::Odometry & odometry)
{
  geometry_msgs::msg::TwistWithCovarianceStamped output;
  output.header = odometry.header;
  // REP-105/nav_msgs semantics: pose is expressed in header.frame_id, while
  // twist is expressed in child_frame_id.
  output.header.frame_id = odometry.child_frame_id;
  output.twist = odometry.twist;
  return output;
}

}  // namespace pure_localization_interface_adapter
