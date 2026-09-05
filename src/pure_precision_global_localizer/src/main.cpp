// SPDX-License-Identifier: Apache-2.0
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "pure_precision_global_localizer/precision_global_localizer_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    pure_precision_global_localizer::makePrecisionGlobalLocalizerNode());
  rclcpp::shutdown();
  return 0;
}
