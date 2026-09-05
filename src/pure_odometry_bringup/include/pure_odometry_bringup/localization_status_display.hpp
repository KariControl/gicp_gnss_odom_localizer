// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "pure_odometry_bringup/localization_visualization_logic.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>

namespace rviz_common::properties
{
class ColorProperty;
class FloatProperty;
class Property;
class RosTopicProperty;
class StringProperty;
class TfFrameProperty;
}  // namespace rviz_common::properties

namespace pure_odometry_bringup
{

class LocalizationStatusDisplay final : public rviz_common::Display
{
  Q_OBJECT

public:
  LocalizationStatusDisplay();
  ~LocalizationStatusDisplay() override = default;

  void update(float wall_dt, float ros_dt) override;
  void reset() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateSubscriptions();

private:
  struct DiagnosticSnapshot
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    std::chrono::steady_clock::time_point received_at;
    std::optional<double> source_stamp_sec;
  };

  void subscribe();
  void unsubscribe();
  void onState(const nav_msgs::msg::Odometry::ConstSharedPtr & message);
  void onDiagnostics(
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & message);
  bool diagnosticIsFresh(
    const std::optional<DiagnosticSnapshot> & snapshot,
    const std::chrono::steady_clock::time_point & current,
    double ros_now_sec,
    double stale_limit_sec) const;
  void showWaitingState();

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  rviz_common::properties::RosTopicProperty * state_topic_property_{nullptr};
  rviz_common::properties::RosTopicProperty * diagnostics_topic_property_{nullptr};
  rviz_common::properties::TfFrameProperty * map_frame_property_{nullptr};
  rviz_common::properties::TfFrameProperty * base_frame_property_{nullptr};
  rviz_common::properties::FloatProperty * output_stale_property_{nullptr};
  rviz_common::properties::FloatProperty * diagnostic_stale_property_{nullptr};
  rviz_common::properties::Property * live_status_property_{nullptr};
  rviz_common::properties::StringProperty * interface_property_{nullptr};
  rviz_common::properties::StringProperty * position_x_property_{nullptr};
  rviz_common::properties::StringProperty * position_y_property_{nullptr};
  rviz_common::properties::StringProperty * yaw_property_{nullptr};
  rviz_common::properties::StringProperty * speed_property_{nullptr};
  rviz_common::properties::StringProperty * transform_property_{nullptr};
  rviz_common::properties::StringProperty * output_rate_property_{nullptr};
  rviz_common::properties::StringProperty * registration_property_{nullptr};
  rviz_common::properties::StringProperty * gnss_property_{nullptr};
  rviz_common::properties::ColorProperty * gnss_color_property_{nullptr};

  std::mutex mutex_;
  std::optional<nav_msgs::msg::Odometry> latest_state_;
  std::chrono::steady_clock::time_point latest_state_received_at_;
  CumulativeRateEstimator published_rate_estimator_{3.0};
  std::optional<DiagnosticSnapshot> adapter_diagnostic_;
  std::optional<DiagnosticSnapshot> registration_diagnostic_;
  std::optional<DiagnosticSnapshot> fusion_diagnostic_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostic_subscription_;
  double update_elapsed_sec_{1.0};
};

}  // namespace pure_odometry_bringup
