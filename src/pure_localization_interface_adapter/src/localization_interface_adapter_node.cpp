// SPDX-License-Identifier: Apache-2.0
#include "pure_localization_interface_adapter/acceleration_estimator.hpp"
#include "pure_localization_interface_adapter/odometry_conversion.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace pure_localization_interface_adapter
{
namespace
{
constexpr auto kDiagnosticName = "localization/localization_interface_adapter";

bool finiteQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  if (!std::isfinite(quaternion.x) || !std::isfinite(quaternion.y) ||
    !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w))
  {
    return false;
  }
  const double squared_norm =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(squared_norm) && squared_norm > 1.0e-12;
}

template<std::size_t Size>
bool finiteArray(const std::array<double, Size> & values)
{
  for (const double value : values) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}

bool finiteOdometry(const nav_msgs::msg::Odometry & message)
{
  const auto & position = message.pose.pose.position;
  const auto & twist = message.twist.twist;
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z) && finiteQuaternion(message.pose.pose.orientation) &&
         std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z) &&
         finiteArray(message.pose.covariance) && finiteArray(message.twist.covariance);
}

void fillDiagonal(
  std::array<double, 36> & covariance,
  double linear_variance,
  double angular_variance)
{
  covariance.fill(0.0);
  covariance[0] = linear_variance;
  covariance[7] = linear_variance;
  covariance[14] = linear_variance;
  covariance[21] = angular_variance;
  covariance[28] = angular_variance;
  covariance[35] = angular_variance;
}
}  // namespace

class LocalizationInterfaceAdapterNode final : public rclcpp::Node
{
public:
  LocalizationInterfaceAdapterNode()
  : Node("localization_interface_adapter")
  {
    input_odom_topic_ = declare_parameter<std::string>(
      "input_odom_topic", "/localization/ekf_odom");
    kinematic_state_topic_ = declare_parameter<std::string>(
      "kinematic_state_topic", "/localization/kinematic_state");
    twist_topic_ = declare_parameter<std::string>(
      "twist_topic", "/localization/twist_with_covariance");
    acceleration_topic_ = declare_parameter<std::string>(
      "acceleration_topic", "/localization/acceleration");
    pose_topic_ = declare_parameter<std::string>(
      "pose_topic", "/localization/pose_estimator/pose_with_covariance");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    require_expected_frames_ = declare_parameter<bool>("require_expected_frames", true);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_pose_ = declare_parameter<bool>("publish_pose", true);
    publish_twist_ = declare_parameter<bool>("publish_twist", true);
    publish_acceleration_ = declare_parameter<bool>("publish_acceleration", true);
    publish_reset_acceleration_ = declare_parameter<bool>(
      "publish_reset_acceleration", true);
    publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", true);
    diagnostic_period_sec_ = declare_parameter<double>("diagnostic_period_sec", 1.0);
    reset_linear_variance_ = declare_parameter<double>(
      "acceleration.reset_linear_variance", 100.0);
    reset_angular_variance_ = declare_parameter<double>(
      "acceleration.reset_angular_variance", 100.0);
    valid_linear_variance_ = declare_parameter<double>(
      "acceleration.valid_linear_variance", 1.0);
    valid_angular_variance_ = declare_parameter<double>(
      "acceleration.valid_angular_variance", 1.0);

    AccelerationEstimatorConfig acceleration_config;
    acceleration_config.min_dt_sec = declare_parameter<double>(
      "acceleration.min_dt_sec", 0.002);
    acceleration_config.max_dt_sec = declare_parameter<double>(
      "acceleration.max_dt_sec", 0.5);
    acceleration_config.lowpass_alpha = declare_parameter<double>(
      "acceleration.lowpass_alpha", 0.25);
    acceleration_config.max_abs_linear_mps2 = declare_parameter<double>(
      "acceleration.max_abs_linear_mps2", 30.0);
    acceleration_config.max_abs_angular_radps2 = declare_parameter<double>(
      "acceleration.max_abs_angular_radps2", 20.0);
    acceleration_estimator_.setConfig(acceleration_config);

    if (input_odom_topic_.empty() || kinematic_state_topic_.empty()) {
      throw std::invalid_argument("input_odom_topic and kinematic_state_topic must not be empty");
    }
    if (map_frame_.empty() || base_frame_.empty()) {
      throw std::invalid_argument("map_frame and base_frame must not be empty");
    }

    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    kinematic_state_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      kinematic_state_topic_, output_qos);
    if (publish_twist_ && !twist_topic_.empty()) {
      twist_publisher_ =
        create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_topic_, output_qos);
    }
    if (publish_pose_ && !pose_topic_.empty()) {
      pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic_, output_qos);
    }
    if (publish_acceleration_ && !acceleration_topic_.empty()) {
      acceleration_publisher_ =
        create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
        acceleration_topic_, output_qos);
    }
    if (publish_tf_) {
      transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    if (publish_diagnostics_) {
      diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);
      const double period = std::max(0.1, diagnostic_period_sec_);
      const auto period_ms = std::chrono::milliseconds(
        static_cast<std::int64_t>(std::llround(period * 1000.0)));
      diagnostic_timer_ = create_wall_timer(
        period_ms,
        std::bind(&LocalizationInterfaceAdapterNode::publishDiagnostics, this));
    }

    input_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      std::bind(&LocalizationInterfaceAdapterNode::onOdometry, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Localization interface adapter: input=%s state=%s twist=%s acceleration=%s pose=%s TF=%s",
      input_odom_topic_.c_str(), kinematic_state_topic_.c_str(),
      twist_topic_.c_str(), acceleration_topic_.c_str(), pose_topic_.c_str(),
      publish_tf_ ? "on" : "off");
  }

private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    ++received_count_;
    const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() == 0) {
      reject("zero_stamp");
      return;
    }
    // A wall/sim-time timer can publish more than once at the same discrete
    // /clock value. Suppress those duplicates without turning an otherwise
    // healthy localization-interface diagnostic WARN. True time reversal
    // remains an input rejection.
    if (has_last_stamp_ && stamp == last_stamp_) {
      ++duplicate_stamp_drop_count_;
      return;
    }
    if (has_last_stamp_ && stamp < last_stamp_) {
      reject("out_of_order_stamp");
      return;
    }
    if (!finiteOdometry(*message)) {
      acceleration_estimator_.reset();
      reject("non_finite_odometry");
      return;
    }
    if (require_expected_frames_ &&
      (message->header.frame_id != map_frame_ || message->child_frame_id != base_frame_))
    {
      reject("unexpected_frames");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejecting odometry frames '%s' -> '%s'; expected '%s' -> '%s'.",
        message->header.frame_id.c_str(), message->child_frame_id.c_str(),
        map_frame_.c_str(), base_frame_.c_str());
      return;
    }

    nav_msgs::msg::Odometry output = *message;
    normalizeQuaternion(output.pose.pose.orientation);
    kinematic_state_publisher_->publish(output);

    if (twist_publisher_) {
      twist_publisher_->publish(makeTwistWithCovarianceStamped(output));
    }

    if (pose_publisher_) {
      geometry_msgs::msg::PoseWithCovarianceStamped pose;
      pose.header = output.header;
      pose.pose = output.pose;
      pose_publisher_->publish(pose);
    }

    const auto acceleration = acceleration_estimator_.update(makeTwistSample(output));
    acceleration_valid_ = acceleration.valid;
    acceleration_reset_ = acceleration.reset;
    last_acceleration_dt_sec_ = acceleration.dt_sec;
    if (acceleration_publisher_ && (acceleration.valid || publish_reset_acceleration_)) {
      publishAcceleration(output, acceleration);
    }

    if (transform_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = output.header;
      transform.child_frame_id = output.child_frame_id;
      transform.transform.translation.x = output.pose.pose.position.x;
      transform.transform.translation.y = output.pose.pose.position.y;
      transform.transform.translation.z = output.pose.pose.position.z;
      transform.transform.rotation = output.pose.pose.orientation;
      transform_broadcaster_->sendTransform(transform);
    }

    last_stamp_ = stamp;
    has_last_stamp_ = true;
    last_rejection_reason_ = "none";
    ++published_count_;
  }

  static TwistSample makeTwistSample(const nav_msgs::msg::Odometry & message)
  {
    TwistSample sample;
    sample.stamp_sec =
      static_cast<double>(message.header.stamp.sec) +
      static_cast<double>(message.header.stamp.nanosec) * 1.0e-9;
    sample.linear = {{
      message.twist.twist.linear.x,
      message.twist.twist.linear.y,
      message.twist.twist.linear.z}};
    sample.angular = {{
      message.twist.twist.angular.x,
      message.twist.twist.angular.y,
      message.twist.twist.angular.z}};
    return sample;
  }

  static void normalizeQuaternion(geometry_msgs::msg::Quaternion & quaternion)
  {
    tf2::Quaternion normalized(
      quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    normalized.normalize();
    quaternion.x = normalized.x();
    quaternion.y = normalized.y();
    quaternion.z = normalized.z();
    quaternion.w = normalized.w();
  }

  void publishAcceleration(
    const nav_msgs::msg::Odometry & odometry,
    const AccelerationEstimate & estimate)
  {
    geometry_msgs::msg::AccelWithCovarianceStamped output;
    output.header = odometry.header;
    output.header.frame_id = odometry.child_frame_id;
    if (estimate.valid) {
      output.accel.accel.linear.x = estimate.linear[0];
      output.accel.accel.linear.y = estimate.linear[1];
      output.accel.accel.linear.z = estimate.linear[2];
      output.accel.accel.angular.x = estimate.angular[0];
      output.accel.accel.angular.y = estimate.angular[1];
      output.accel.accel.angular.z = estimate.angular[2];
      fillDiagonal(
        output.accel.covariance, valid_linear_variance_, valid_angular_variance_);
    } else {
      fillDiagonal(
        output.accel.covariance, reset_linear_variance_, reset_angular_variance_);
    }
    acceleration_publisher_->publish(output);
  }

  void reject(const std::string & reason)
  {
    last_rejection_reason_ = reason;
    ++rejected_count_;
  }

  void publishDiagnostics()
  {
    if (!diagnostic_publisher_) return;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = kDiagnosticName;
    status.hardware_id = "none";
    if (published_count_ == 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting for valid fused odometry";
    } else if (last_rejection_reason_ != "none") {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "recent input rejected: " + last_rejection_reason_;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "localization interface adapter active";
    }

    const auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue entry;
        entry.key = key;
        entry.value = value;
        status.values.push_back(entry);
      };
    add("input_odom_topic", input_odom_topic_);
    add("kinematic_state_topic", kinematic_state_topic_);
    add("twist_topic", twist_topic_);
    add("map_frame", map_frame_);
    add("base_frame", base_frame_);
    add("received_count", std::to_string(received_count_));
    add("published_count", std::to_string(published_count_));
    add("duplicate_stamp_drop_count", std::to_string(duplicate_stamp_drop_count_));
    add("rejected_count", std::to_string(rejected_count_));
    add("last_rejection_reason", last_rejection_reason_);
    add("acceleration_valid", acceleration_valid_ ? "true" : "false");
    add("acceleration_reset", acceleration_reset_ ? "true" : "false");
    add("acceleration_dt_sec", std::to_string(last_acceleration_dt_sec_));
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  std::string input_odom_topic_;
  std::string kinematic_state_topic_;
  std::string twist_topic_;
  std::string acceleration_topic_;
  std::string pose_topic_;
  std::string map_frame_;
  std::string base_frame_;
  bool require_expected_frames_{true};
  bool publish_tf_{true};
  bool publish_pose_{true};
  bool publish_twist_{true};
  bool publish_acceleration_{true};
  bool publish_reset_acceleration_{true};
  bool publish_diagnostics_{true};
  double diagnostic_period_sec_{1.0};
  double reset_linear_variance_{100.0};
  double reset_angular_variance_{100.0};
  double valid_linear_variance_{1.0};
  double valid_angular_variance_{1.0};

  AccelerationEstimator acceleration_estimator_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr input_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr kinematic_state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
    acceleration_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  bool has_last_stamp_{false};
  std::uint64_t received_count_{0U};
  std::uint64_t published_count_{0U};
  std::uint64_t duplicate_stamp_drop_count_{0U};
  std::uint64_t rejected_count_{0U};
  std::string last_rejection_reason_{"none"};
  bool acceleration_valid_{false};
  bool acceleration_reset_{false};
  double last_acceleration_dt_sec_{0.0};
};

}  // namespace pure_localization_interface_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<pure_localization_interface_adapter::LocalizationInterfaceAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
