// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace pure_precision_global_localizer
{

class PrecisionGlobalLocalizerNode;

std::shared_ptr<rclcpp::Node> makePrecisionGlobalLocalizerNode(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace pure_precision_global_localizer
