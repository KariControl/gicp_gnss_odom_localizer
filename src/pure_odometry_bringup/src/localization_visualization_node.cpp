// SPDX-License-Identifier: Apache-2.0
#include "pure_odometry_bringup/localization_visualization_logic.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace pure_odometry_bringup
{
namespace
{
double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

}  // namespace

class LocalizationVisualizationNode final : public rclcpp::Node
{
public:
  LocalizationVisualizationNode()
  : Node("localization_visualization")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/localization/kinematic_state");
    path_topic_ = declare_parameter<std::string>(
      "path_topic", "/localization/visualization/trajectory");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/localization/visualization/status_markers");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    path_min_distance_m_ = std::max(
      0.0, declare_parameter<double>("path_min_distance_m", 0.15));
    path_max_interval_sec_ = std::max(
      0.05, declare_parameter<double>("path_max_interval_sec", 0.5));
    const auto max_path_poses = declare_parameter<std::int64_t>("max_path_poses", 2000);
    max_path_poses_ = static_cast<std::size_t>(
      std::max<std::int64_t>(2, max_path_poses));
    publish_rate_hz_ = std::clamp(
      declare_parameter<double>("publish_rate_hz", 5.0), 0.5, 30.0);
    covariance_z_offset_m_ = declare_parameter<double>("covariance_z_offset_m", 0.0);
    covariance_sigma_ = std::max(
      0.1, declare_parameter<double>("covariance_sigma", 2.0));
    covariance_max_radius_m_ = std::max(
      0.1, declare_parameter<double>("covariance_max_radius_m", 20.0));

    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(path_topic_, output_qos);
    marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, output_qos);

    state_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile(),
      std::bind(&LocalizationVisualizationNode::onState, this, std::placeholders::_1));

    const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
      std::bind(&LocalizationVisualizationNode::publishVisualization, this));

    RCLCPP_INFO(
      get_logger(),
      "Localization visualization: state=%s path=%s covariance_markers=%s",
      input_topic_.c_str(), path_topic_.c_str(), marker_topic_.c_str());
  }

private:
  void onState(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const double stamp_sec = stampSeconds(message->header.stamp);
    if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_state_ && stamp_sec < stampSeconds(latest_state_->header.stamp)) {
      path_.poses.clear();
    }
    latest_state_ = *message;

    if (path_.header.frame_id.empty() || path_.header.frame_id != message->header.frame_id) {
      path_.poses.clear();
      path_.header.frame_id = message->header.frame_id;
    }
    path_.header.stamp = message->header.stamp;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = message->header;
    pose.pose = message->pose.pose;
    bool append = path_.poses.empty();
    if (!append) {
      const auto & previous = path_.poses.back();
      const double dx = pose.pose.position.x - previous.pose.position.x;
      const double dy = pose.pose.position.y - previous.pose.position.y;
      const double dt = stamp_sec - stampSeconds(previous.header.stamp);
      append = std::hypot(dx, dy) >= path_min_distance_m_ || dt >= path_max_interval_sec_;
    }
    if (append) {
      path_.poses.push_back(std::move(pose));
      while (path_.poses.size() > max_path_poses_) {
        path_.poses.erase(path_.poses.begin());
      }
    }
  }

  visualization_msgs::msg::Marker makeCovarianceMarker(
    const nav_msgs::msg::Odometry & state) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = state.header;
    marker.header.frame_id = map_frame_;
    marker.ns = "xy_position_covariance_2sigma";
    marker.id = 100;
    marker.pose.position = state.pose.pose.position;
    marker.pose.position.z += covariance_z_offset_m_;
    marker.pose.orientation.w = 1.0;

    const auto & covariance = state.pose.covariance;
    const double symmetric_xy = 0.5 * (covariance[1] + covariance[6]);
    const auto ellipse = makeCovarianceEllipse(
      covariance[0], symmetric_xy, covariance[7],
      covariance_sigma_, covariance_max_radius_m_);
    if (!ellipse) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
      return marker;
    }

    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.08;
    marker.color.r = 0.1F;
    marker.color.g = 0.85F;
    marker.color.b = 1.0F;
    marker.color.a = 0.9F;
    constexpr int kSegments = 64;
    const double cos_yaw = std::cos(ellipse->yaw_rad);
    const double sin_yaw = std::sin(ellipse->yaw_rad);
    marker.points.reserve(kSegments + 1);
    for (int index = 0; index <= kSegments; ++index) {
      constexpr double kTwoPi = 6.28318530717958647692;
      const double angle = kTwoPi * static_cast<double>(index) /
        static_cast<double>(kSegments);
      const double major = ellipse->major_radius_m * std::cos(angle);
      const double minor = ellipse->minor_radius_m * std::sin(angle);
      geometry_msgs::msg::Point point;
      point.x = cos_yaw * major - sin_yaw * minor;
      point.y = sin_yaw * major + cos_yaw * minor;
      marker.points.push_back(point);
    }
    return marker;
  }

  void publishVisualization()
  {
    nav_msgs::msg::Odometry state;
    nav_msgs::msg::Path path;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!latest_state_) {
        return;
      }
      state = *latest_state_;
      path = path_;
      if (path.poses.empty() ||
        stampSeconds(path.poses.back().header.stamp) != stampSeconds(state.header.stamp))
      {
        geometry_msgs::msg::PoseStamped endpoint;
        endpoint.header = state.header;
        endpoint.pose = state.pose.pose;
        path.poses.push_back(std::move(endpoint));
        if (path.poses.size() > max_path_poses_) {
          path.poses.erase(path.poses.begin());
        }
      }
    }

    path.header = state.header;
    path_publisher_->publish(path);

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.reserve(1U);
    markers.markers.push_back(makeCovarianceMarker(state));
    marker_publisher_->publish(markers);
  }

  std::string input_topic_;
  std::string path_topic_;
  std::string marker_topic_;
  std::string map_frame_;
  double path_min_distance_m_{0.15};
  double path_max_interval_sec_{0.5};
  std::size_t max_path_poses_{2000U};
  double publish_rate_hz_{5.0};
  double covariance_z_offset_m_{0.0};
  double covariance_sigma_{2.0};
  double covariance_max_radius_m_{20.0};

  std::mutex mutex_;
  std::optional<nav_msgs::msg::Odometry> latest_state_;
  nav_msgs::msg::Path path_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pure_odometry_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pure_odometry_bringup::LocalizationVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
