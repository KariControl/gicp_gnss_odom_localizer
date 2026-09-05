#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include <nav_msgs/msg/odometry.hpp>

namespace pure_gyro_odometer
{

struct AcceptedScanOdometryState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double yaw_rate{0.0};
  double covariance_xy{0.0};
  double covariance_yaw{0.0};
  bool has_twist{false};
};

inline nav_msgs::msg::Odometry makeAcceptedScanOdometry(
  const builtin_interfaces::msg::Time & scan_stamp,
  const std::string & odom_frame,
  const std::string & base_frame,
  const AcceptedScanOdometryState & state)
{
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = scan_stamp;
  odometry.header.frame_id = odom_frame;
  odometry.child_frame_id = base_frame;
  odometry.pose.pose.position.x = state.x;
  odometry.pose.pose.position.y = state.y;
  odometry.pose.pose.position.z = 0.0;
  odometry.pose.pose.orientation.z = std::sin(0.5 * state.yaw);
  odometry.pose.pose.orientation.w = std::cos(0.5 * state.yaw);

  const double covariance_xy = std::max(0.0, state.covariance_xy);
  const double covariance_yaw = std::max(0.0, state.covariance_yaw);
  odometry.pose.covariance.fill(0.0);
  odometry.pose.covariance[0] = covariance_xy;
  odometry.pose.covariance[7] = covariance_xy;
  odometry.pose.covariance[14] = 1.0e6;
  odometry.pose.covariance[21] = 1.0e6;
  odometry.pose.covariance[28] = 1.0e6;
  odometry.pose.covariance[35] = covariance_yaw;

  odometry.twist.twist.linear.x = state.has_twist ? state.vx : 0.0;
  odometry.twist.twist.linear.y = state.has_twist ? state.vy : 0.0;
  odometry.twist.twist.angular.z = state.has_twist ? state.yaw_rate : 0.0;
  odometry.twist.covariance.fill(0.0);
  odometry.twist.covariance[0] = state.has_twist ?
    std::max(1.0e-4, 0.25 * covariance_xy) : 1.0e6;
  odometry.twist.covariance[7] = state.has_twist ?
    std::max(1.0e-4, 0.25 * covariance_xy) : 1.0e6;
  odometry.twist.covariance[14] = 1.0e6;
  odometry.twist.covariance[21] = 1.0e6;
  odometry.twist.covariance[28] = 1.0e6;
  odometry.twist.covariance[35] = state.has_twist ?
    std::max(1.0e-5, 0.25 * covariance_yaw) : 1.0e6;
  return odometry;
}

}  // namespace pure_gyro_odometer
