// SPDX-License-Identifier: Apache-2.0
#include "pure_localization_interface_adapter/odometry_conversion.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main()
{
  try {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp.sec = 42;
    odometry.header.stamp.nanosec = 123U;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";
    odometry.twist.twist.linear.x = 3.5;
    odometry.twist.twist.linear.y = -0.25;
    odometry.twist.twist.angular.z = 0.125;
    for (std::size_t index = 0; index < odometry.twist.covariance.size(); ++index) {
      odometry.twist.covariance[index] = static_cast<double>(index) * 0.01;
    }

    const auto output =
      pure_localization_interface_adapter::makeTwistWithCovarianceStamped(odometry);
    require(output.header.stamp == odometry.header.stamp, "stamp must be preserved");
    require(output.header.frame_id == "base_link", "twist must use child/base frame");
    require(output.twist.twist == odometry.twist.twist, "twist values must be preserved");
    require(
      output.twist.covariance == odometry.twist.covariance,
      "twist covariance must be preserved");
  } catch (const std::exception & exception) {
    std::cerr << "FAIL test_odometry_conversion: " << exception.what() << '\n';
    return EXIT_FAILURE;
  }
  std::cout << "PASS test_odometry_conversion\n";
  return EXIT_SUCCESS;
}
