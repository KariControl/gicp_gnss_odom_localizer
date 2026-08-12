#include "pure_lidar_gyro_odometer/accepted_scan_odometry.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace
{

void require(const bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

bool near(const double lhs, const double rhs, const double tolerance = 1.0e-12)
{
  return std::fabs(lhs - rhs) <= tolerance;
}

void testAcceptedScanStampAndStateArePreserved()
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 123;
  stamp.nanosec = 456789012U;

  pure_gyro_odometer::AcceptedScanOdometryState state;
  state.x = 1.25;
  state.y = -2.5;
  state.yaw = 0.6;
  state.vx = 3.0;
  state.vy = -0.4;
  state.yaw_rate = 0.2;
  state.covariance_xy = 0.08;
  state.covariance_yaw = 0.04;
  state.has_twist = true;

  const auto odometry = pure_gyro_odometer::makeAcceptedScanOdometry(
    stamp, "odom_eval", "base_eval", state);
  require(odometry.header.stamp == stamp, "input scan timestamp was not preserved exactly");
  require(odometry.header.frame_id == "odom_eval", "odom frame mismatch");
  require(odometry.child_frame_id == "base_eval", "base frame mismatch");
  require(near(odometry.pose.pose.position.x, state.x), "pose x mismatch");
  require(near(odometry.pose.pose.position.y, state.y), "pose y mismatch");
  require(near(odometry.pose.pose.orientation.z, std::sin(0.5 * state.yaw)),
    "pose yaw quaternion mismatch");
  require(near(odometry.pose.pose.orientation.w, std::cos(0.5 * state.yaw)),
    "pose yaw quaternion mismatch");
  require(near(odometry.pose.covariance[0], state.covariance_xy),
    "pose XY covariance mismatch");
  require(near(odometry.pose.covariance[35], state.covariance_yaw),
    "pose yaw covariance mismatch");
  require(near(odometry.twist.twist.linear.x, state.vx), "twist vx mismatch");
  require(near(odometry.twist.twist.linear.y, state.vy), "twist vy mismatch");
  require(near(odometry.twist.twist.angular.z, state.yaw_rate),
    "twist yaw-rate mismatch");
}

void testUnavailableTwistIsExplicit()
{
  builtin_interfaces::msg::Time stamp;
  pure_gyro_odometer::AcceptedScanOdometryState state;
  state.vx = 9.0;
  state.vy = 8.0;
  state.yaw_rate = 7.0;
  state.has_twist = false;

  const auto odometry = pure_gyro_odometer::makeAcceptedScanOdometry(
    stamp, "odom", "base_link", state);
  require(near(odometry.twist.twist.linear.x, 0.0), "unavailable vx must be zero");
  require(near(odometry.twist.twist.linear.y, 0.0), "unavailable vy must be zero");
  require(near(odometry.twist.twist.angular.z, 0.0),
    "unavailable yaw rate must be zero");
  require(near(odometry.twist.covariance[0], 1.0e6),
    "unavailable twist must have sentinel covariance");
  require(near(odometry.twist.covariance[35], 1.0e6),
    "unavailable yaw rate must have sentinel covariance");
}

}  // namespace

int main()
{
  testAcceptedScanStampAndStateArePreserved();
  testUnavailableTwistIsExplicit();
  std::cout << "PASS test_accepted_scan_odometry\n";
  return 0;
}
