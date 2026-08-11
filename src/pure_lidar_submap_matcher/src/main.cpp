#include "pure_lidar_submap_matcher/submap_matcher_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pure_lidar_submap_matcher::SubmapMatcherNode>());
  rclcpp::shutdown();
  return 0;
}
