// SPDX-License-Identifier: Apache-2.0
#include "pure_lidar_gyro_odometer/gyro_odometer_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

using namespace std::chrono_literals;

void require(const bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

template<typename Predicate>
bool spinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  Predicate predicate,
  const std::chrono::steady_clock::duration timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  executor.spin_some();
  return predicate();
}

void spinFor(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::chrono::steady_clock::duration duration)
{
  (void)spinUntil(executor, []() { return false; }, duration);
}

rclcpp::NodeOptions makeOptions(
  const std::string & odom_frame,
  const std::string & base_frame,
  const std::string & odom_topic,
  const std::optional<bool> publish_tf)
{
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> overrides{
    rclcpp::Parameter("odom_frame", odom_frame),
    rclcpp::Parameter("base_frame", base_frame),
    rclcpp::Parameter("out_odom_topic", odom_topic),
    rclcpp::Parameter("out_deskew_twist_topic", odom_topic + "/deskew_twist"),
    rclcpp::Parameter("publish_rate_hz", 100.0),
    rclcpp::Parameter("lidar_odom.enable", false),
    rclcpp::Parameter("imu_corrected.enable", false)};
  if (publish_tf.has_value()) {
    overrides.emplace_back("publish_tf", *publish_tf);
  }
  options.parameter_overrides(overrides);
  return options;
}

void testTfPublicationIsDisabledByDefault()
{
  constexpr auto kOdomFrame = "tf_test_disabled_odom";
  constexpr auto kBaseFrame = "tf_test_disabled_base";
  constexpr auto kOdomTopic = "/tf_test/disabled_odom";

  auto odometer = std::make_shared<pure_gyro_odometer::GyroOdometerNode>(
    makeOptions(kOdomFrame, kBaseFrame, kOdomTopic, std::nullopt));
  require(
    !odometer->get_parameter("publish_tf").as_bool(),
    "TF publication must be disabled by default");

  auto listener_node = std::make_shared<rclcpp::Node>("tf_publication_disabled_listener");
  auto buffer = std::make_shared<tf2_ros::Buffer>(listener_node->get_clock());
  auto listener =
    std::make_shared<tf2_ros::TransformListener>(*buffer, listener_node, true);
  auto observer = std::make_shared<rclcpp::Node>("tf_publication_disabled_observer");
  bool odometry_received = false;
  const auto subscription = observer->create_subscription<nav_msgs::msg::Odometry>(
    kOdomTopic, 10,
    [&odometry_received](const nav_msgs::msg::Odometry::ConstSharedPtr) {
      odometry_received = true;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(odometer);
  executor.add_node(observer);
  require(
    spinUntil(executor, [&odometry_received]() { return odometry_received; }, 2s),
    "odometer timer did not publish during the disabled-TF test");
  spinFor(executor, 250ms);

  const rclcpp::Time latest(0, 0, listener_node->get_clock()->get_clock_type());
  require(
    !buffer->canTransform(kOdomFrame, kBaseFrame, latest),
    "publish_tf=false must not broadcast odom -> base_link");

  (void)subscription;
  (void)listener;
}

void testTfMatchesRawOdometry()
{
  constexpr auto kOdomFrame = "tf_test_enabled_odom";
  constexpr auto kBaseFrame = "tf_test_enabled_base";
  constexpr auto kOdomTopic = "/tf_test/enabled_odom";

  auto odometer = std::make_shared<pure_gyro_odometer::GyroOdometerNode>(
    makeOptions(kOdomFrame, kBaseFrame, kOdomTopic, true));
  require(
    odometer->get_parameter("publish_tf").as_bool(),
    "publish_tf=true parameter override was not applied");

  auto listener_node = std::make_shared<rclcpp::Node>("tf_publication_enabled_listener");
  auto buffer = std::make_shared<tf2_ros::Buffer>(listener_node->get_clock());
  auto listener =
    std::make_shared<tf2_ros::TransformListener>(*buffer, listener_node, true);
  auto observer = std::make_shared<rclcpp::Node>("tf_publication_enabled_observer");
  std::vector<nav_msgs::msg::Odometry> odometry_messages;
  const auto subscription = observer->create_subscription<nav_msgs::msg::Odometry>(
    kOdomTopic, 10,
    [&odometry_messages](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
      odometry_messages.push_back(*message);
    });

  std::optional<nav_msgs::msg::Odometry> matching_odometry;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(odometer);
  executor.add_node(observer);
  const bool matched = spinUntil(
    executor,
    [&]() {
      for (auto it = odometry_messages.rbegin(); it != odometry_messages.rend(); ++it) {
        const rclcpp::Time stamp(it->header.stamp);
        if (buffer->canTransform(kOdomFrame, kBaseFrame, stamp)) {
          matching_odometry = *it;
          return true;
        }
      }
      return false;
    },
    3s);
  require(matched, "did not receive matching raw odometry and TF samples");

  const rclcpp::Time stamp(matching_odometry->header.stamp);
  const auto transform = buffer->lookupTransform(kOdomFrame, kBaseFrame, stamp);
  require(transform.header.frame_id == kOdomFrame, "TF parent frame differs from raw odometry");
  require(transform.child_frame_id == kBaseFrame, "TF child frame differs from raw odometry");
  require(
    rclcpp::Time(transform.header.stamp).nanoseconds() == stamp.nanoseconds(),
    "TF stamp differs from raw odometry");

  const auto & pose = matching_odometry->pose.pose;
  constexpr double kTolerance = 1.0e-12;
  require(
    std::fabs(transform.transform.translation.x - pose.position.x) <= kTolerance &&
    std::fabs(transform.transform.translation.y - pose.position.y) <= kTolerance &&
    std::fabs(transform.transform.translation.z - pose.position.z) <= kTolerance,
    "TF translation differs from raw odometry");
  require(
    std::fabs(transform.transform.rotation.x - pose.orientation.x) <= kTolerance &&
    std::fabs(transform.transform.rotation.y - pose.orientation.y) <= kTolerance &&
    std::fabs(transform.transform.rotation.z - pose.orientation.z) <= kTolerance &&
    std::fabs(transform.transform.rotation.w - pose.orientation.w) <= kTolerance,
    "TF orientation differs from raw odometry");

  (void)subscription;
  (void)listener;
}

}  // namespace

int main(int argc, char ** argv)
{
  (void)::setenv("ROS_LOG_DIR", "/tmp", 1);
  rclcpp::init(argc, argv);
  testTfPublicationIsDisabledByDefault();
  testTfMatchesRawOdometry();
  rclcpp::shutdown();
  std::cout << "PASS test_tf_publication\n";
  return 0;
}
