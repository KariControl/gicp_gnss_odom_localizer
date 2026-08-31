// SPDX-License-Identifier: Apache-2.0
#include "pure_lidar_gyro_odometer/gyro_odometer_node.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace
{

struct CallbackGroupInfo
{
  rclcpp::CallbackGroup::SharedPtr group;
  std::vector<std::string> subscription_topics;
};

using RetainedNodes =
  std::vector<std::shared_ptr<pure_gyro_odometer::GyroOdometerNode>>;

void require(const bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

const CallbackGroupInfo * findSubscriptionGroup(
  const std::vector<CallbackGroupInfo> & groups, const std::string & topic)
{
  const CallbackGroupInfo * result = nullptr;
  for (const auto & group : groups) {
    for (const auto & subscription_topic : group.subscription_topics) {
      if (subscription_topic == topic) {
        require(result == nullptr, "subscription appears in more than one callback group: " + topic);
        result = &group;
      }
    }
  }
  require(result != nullptr, "subscription callback group was not found: " + topic);
  return result;
}

std::vector<CallbackGroupInfo> collectCallbackGroups(
  const std::shared_ptr<pure_gyro_odometer::GyroOdometerNode> & node)
{
  std::vector<CallbackGroupInfo> result;
  node->for_each_callback_group(
    [&result](const rclcpp::CallbackGroup::SharedPtr group) {
      CallbackGroupInfo info;
      info.group = group;
      group->collect_all_ptrs(
        [&info](const rclcpp::SubscriptionBase::SharedPtr & subscription) {
          info.subscription_topics.emplace_back(subscription->get_topic_name());
        },
        [](const rclcpp::ServiceBase::SharedPtr &) {},
        [](const rclcpp::ClientBase::SharedPtr &) {},
        [](const rclcpp::TimerBase::SharedPtr &) {},
        [](const rclcpp::Waitable::SharedPtr &) {});
      result.push_back(std::move(info));
    });
  return result;
}

void testSensorCallbacksAreIsolated(RetainedNodes & retained_nodes)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("imu_topic", "/callback_group_test/imu"),
    rclcpp::Parameter("points_topic", "/callback_group_test/points"),
    rclcpp::Parameter("wheel_speed.use", true),
    rclcpp::Parameter("wheel_speed_topic", "/callback_group_test/wheel"),
    rclcpp::Parameter("wheel_speed.scale_estimation.enable", true),
    rclcpp::Parameter("reference_pose_topic", "/callback_group_test/reference")});

  auto node = std::make_shared<pure_gyro_odometer::GyroOdometerNode>(options);
  retained_nodes.push_back(node);
  const auto groups = collectCallbackGroups(node);
  const auto * imu_group = findSubscriptionGroup(groups, "/callback_group_test/imu");
  const auto * lidar_group = findSubscriptionGroup(groups, "/callback_group_test/points");
  const auto * wheel_group = findSubscriptionGroup(groups, "/callback_group_test/wheel");
  const auto * reference_group =
    findSubscriptionGroup(groups, "/callback_group_test/reference");

  require(
    imu_group->group->type() == rclcpp::CallbackGroupType::MutuallyExclusive,
    "IMU callback group must be mutually exclusive");
  require(
    lidar_group->group->type() == rclcpp::CallbackGroupType::MutuallyExclusive,
    "point-cloud callback group must be mutually exclusive");
  require(
    wheel_group->group->type() == rclcpp::CallbackGroupType::Reentrant,
    "wheel/reference callback group must be reentrant");
  require(
    imu_group->group != lidar_group->group,
    "IMU and point-cloud subscriptions must use different callback groups");
  require(
    imu_group->group != wheel_group->group,
    "auxiliary sensor callbacks must not block IMU delivery");
  require(
    lidar_group->group != wheel_group->group,
    "auxiliary sensor callbacks must not share the GICP callback group");
  require(
    wheel_group->group == reference_group->group,
    "wheel and reference subscriptions should share the auxiliary callback group");
}

void testAcceptedScanOdomPublisherIsOptIn(RetainedNodes & retained_nodes)
{
  auto default_node = std::make_shared<pure_gyro_odometer::GyroOdometerNode>(
    rclcpp::NodeOptions{});
  retained_nodes.push_back(default_node);
  require(
    !default_node->get_parameter("lidar_odom.accepted_scan_odom.enable").as_bool(),
    "accepted-scan odometry must be disabled by default");
  require(
    default_node->get_parameter("lidar_odom.accepted_scan_odom.topic").as_string() ==
    "/localization/gyro_lidar_odom_scan",
    "accepted-scan odometry default topic changed unexpectedly");
  require(
    default_node->count_publishers("/localization/gyro_lidar_odom_scan") == 0U,
    "default-disabled accepted-scan odometry must not create a publisher");

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("lidar_odom.accepted_scan_odom.enable", true),
    rclcpp::Parameter(
      "lidar_odom.accepted_scan_odom.topic", "/callback_group_test/accepted_scan_odom")});
  auto node = std::make_shared<pure_gyro_odometer::GyroOdometerNode>(options);
  retained_nodes.push_back(node);
  require(
    node->count_publishers("/callback_group_test/accepted_scan_odom") == 1U,
    "enabled accepted-scan odometry must create exactly one publisher");
}

}  // namespace

int main(int argc, char ** argv)
{
  // Keep the constructor test runnable in read-only-home CI sandboxes.
  (void)::setenv("ROS_LOG_DIR", "/tmp", 1);
  rclcpp::init(argc, argv);
  RetainedNodes retained_nodes;
  testSensorCallbacksAreIsolated(retained_nodes);
  testAcceptedScanOdomPublisherIsOptIn(retained_nodes);
  rclcpp::shutdown();
  retained_nodes.clear();
  std::cout << "PASS test_callback_groups\n";
  return 0;
}
