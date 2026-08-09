#include "pure_imu_undistortion/imu_undistorter.hpp"

#include <functional>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace pure_imu_undistortion
{

double ImuUndistorter::toSec(const rclcpp::Time & t)
{
  return static_cast<double>(t.nanoseconds()) * 1e-9;
}

rclcpp::Time ImuUndistorter::fromSec(const rclcpp::Clock & clock, double sec)
{
  const int64_t ns = static_cast<int64_t>(sec * 1e9);
  return rclcpp::Time(ns, clock.get_clock_type());
}

uint8_t ImuUndistorter::toDiagLevel(const std::string & level)
{
  if (level == "OK" || level == "ok") {
    return diagnostic_msgs::msg::DiagnosticStatus::OK;
  }
  if (level == "WARN" || level == "warn") {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }
  if (level == "ERROR" || level == "error") {
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  if (level == "STALE" || level == "stale") {
    return diagnostic_msgs::msg::DiagnosticStatus::STALE;
  }
  // デフォルトはWARN（運用上無難）
  return diagnostic_msgs::msg::DiagnosticStatus::WARN;
}

ImuUndistorter::ImuUndistorter(const rclcpp::NodeOptions & options)
: rclcpp::Node("imu_undistorter", options)
{
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  imu_frame_ = declare_parameter<std::string>("imu_frame", "");
  scan_frame_ = declare_parameter<std::string>("scan_frame", "");

  points_in_topic_ = declare_parameter<std::string>("points_in_topic", "/points_raw");
  points_out_topic_ = declare_parameter<std::string>("points_out_topic", "/points_undistorted");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
  twist_topic_ = declare_parameter<std::string>("twist_topic", ""); // empty => disabled

  time_fields_ = declare_parameter<std::vector<std::string>>(
    "time_fields", std::vector<std::string>{"time","t","timestamp","offset_time","time_stamp"});

  prefer_relative_time_ = declare_parameter<bool>("prefer_relative_time", true);
  time_scale_ = declare_parameter<double>("time_scale", 0.0); // 0 => auto
  fallback_scan_period_ = declare_parameter<double>("fallback_scan_period", 0.1);
  allow_linear_time_fallback_ = declare_parameter<bool>("allow_linear_time_fallback", false);
  cloud_stamp_is_start_ = declare_parameter<bool>("cloud_stamp_is_start", true);
  reference_time_ = declare_parameter<std::string>("reference_time", "start"); // start|end
  point_time_tolerance_sec_ = declare_parameter<double>("point_time_tolerance_sec", 0.002);

  imu_buffer_sec_ = declare_parameter<double>("imu_buffer_sec", 2.0);
  twist_buffer_sec_ = declare_parameter<double>("twist_buffer_sec", 2.0);
  max_imu_gap_sec_ = declare_parameter<double>("max_imu_gap_sec", 0.02);
  max_imu_boundary_gap_sec_ = declare_parameter<double>("max_imu_boundary_gap_sec", 0.02);
  max_twist_gap_sec_ = declare_parameter<double>("max_twist_gap_sec", 0.1);
  max_time_offset_sec_ = declare_parameter<double>("max_time_offset_sec", 0.2);
  max_abs_gyro_radps_ = declare_parameter<double>("max_abs_gyro_radps", 20.0);

  use_translation_ = declare_parameter<bool>("use_translation", false);
  use_twist_speed_ = declare_parameter<bool>("use_twist_speed", false);
  default_speed_mps_ = declare_parameter<double>("default_speed_mps", 0.0);
  max_speed_mps_ = declare_parameter<double>("max_speed_mps", 40.0);
  allow_default_speed_fallback_ = declare_parameter<bool>("allow_default_speed_fallback", false);

  publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", true);
  diag_throttle_ms_ = declare_parameter<int>("diag_throttle_ms", 1000);
  max_pending_clouds_ = declare_parameter<int>("max_pending_clouds", 4);
  const bool input_qos_best_effort = declare_parameter<bool>("input_qos_best_effort", false);

  if (reference_time_ != "start" && reference_time_ != "end") {
    throw std::invalid_argument("reference_time must be 'start' or 'end'");
  }
  if (!(fallback_scan_period_ > 0.0) || !(max_time_offset_sec_ > 0.0) ||
      !(imu_buffer_sec_ > 0.0) || !(twist_buffer_sec_ > 0.0) ||
      !(max_imu_gap_sec_ > 0.0) || !(max_imu_boundary_gap_sec_ >= 0.0) ||
      max_imu_boundary_gap_sec_ > max_imu_gap_sec_ ||
      !(max_twist_gap_sec_ > 0.0) || !(max_abs_gyro_radps_ > 0.0) ||
      !(max_speed_mps_ > 0.0) || !(point_time_tolerance_sec_ >= 0.0) ||
      max_pending_clouds_ <= 0 || (time_scale_ < 0.0)) {
    throw std::invalid_argument("invalid timing, gyro, speed, or time-scale parameter");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pub_points_ = create_publisher<sensor_msgs::msg::PointCloud2>(points_out_topic_, rclcpp::SensorDataQoS());
  pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

  const rclcpp::QoS input_qos = input_qos_best_effort
    ? rclcpp::SensorDataQoS()
    : rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
  sensor_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sensor_subscription_options;
  sensor_subscription_options.callback_group = sensor_callback_group_;

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, input_qos,
    std::bind(&ImuUndistorter::onImu, this, std::placeholders::_1),
    sensor_subscription_options);

  if (!twist_topic_.empty()) {
    sub_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_topic_, input_qos,
      std::bind(&ImuUndistorter::onTwist, this, std::placeholders::_1),
      sensor_subscription_options);
  }

  sub_points_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    points_in_topic_, input_qos,
    std::bind(&ImuUndistorter::onPoints, this, std::placeholders::_1),
    sensor_subscription_options);

  RCLCPP_INFO(get_logger(),
    "pure_imu_undistortion started. translation=%s twist=%s linear_time_fallback=%s",
    use_translation_ ? "on" : "off",
    (!twist_topic_.empty()) ? "on" : "off",
    allow_linear_time_fallback_ ? "on" : "off");
}

void ImuUndistorter::pruneBuffers(const rclcpp::Time & nowt)
{
  const double now_sec = toSec(nowt);

  while (!imu_buf_.empty()) {
    const double t = toSec(imu_buf_.front().stamp);
    if (now_sec - t > imu_buffer_sec_) imu_buf_.pop_front();
    else break;
  }
  while (!twist_buf_.empty()) {
    const double t = toSec(twist_buf_.front().stamp);
    if (now_sec - t > twist_buffer_sec_) twist_buf_.pop_front();
    else break;
  }
}

void ImuUndistorter::onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  if (stamp.nanoseconds() <= 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected IMU with zero timestamp");
    return;
  }

  const std::string frame = imu_frame_.empty() ? msg->header.frame_id : imu_frame_;
  if (frame.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected IMU with empty frame_id");
    return;
  }
  if (!imu_frame_.empty() && msg->header.frame_id != imu_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejected IMU frame mismatch: expected '%s', got '%s'",
      imu_frame_.c_str(), msg->header.frame_id.c_str());
    return;
  }

  ImuSample s;
  s.stamp = stamp;
  s.gyro = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  if (!s.gyro.allFinite() || s.gyro.norm() > max_abs_gyro_radps_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejected invalid IMU angular velocity (norm=%.3f rad/s)", s.gyro.norm());
    return;
  }

  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!observed_imu_frame_.empty() && observed_imu_frame_ != frame) {
      RCLCPP_WARN(
        get_logger(), "IMU frame changed from '%s' to '%s'; clearing deskew IMU history",
        observed_imu_frame_.c_str(), frame.c_str());
      imu_buf_.clear();
    }
    observed_imu_frame_ = frame;

    if (!imu_buf_.empty()) {
      if (s.stamp < imu_buf_.back().stamp) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected out-of-order IMU sample");
        return;
      }
      if (s.stamp == imu_buf_.back().stamp) {
        imu_buf_.back() = s;
        pruneBuffers(s.stamp);
      } else {
        imu_buf_.push_back(s);
        pruneBuffers(s.stamp);
      }
    } else {
      imu_buf_.push_back(s);
      pruneBuffers(s.stamp);
    }
  }
  retryPendingClouds();
}

void ImuUndistorter::onTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  if (stamp.nanoseconds() <= 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected twist with zero timestamp");
    return;
  }
  double v = msg->twist.linear.x;
  if (!std::isfinite(v)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected non-finite twist speed");
    return;
  }
  v = std::max(-max_speed_mps_, std::min(max_speed_mps_, v));

  TwistSample s;
  s.stamp = stamp;
  s.speed_mps = v;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!twist_buf_.empty()) {
      if (s.stamp < twist_buf_.back().stamp) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected out-of-order twist sample");
        return;
      }
      if (s.stamp == twist_buf_.back().stamp) {
        twist_buf_.back() = s;
        pruneBuffers(s.stamp);
      } else {
        twist_buf_.push_back(s);
        pruneBuffers(s.stamp);
      }
    } else {
      twist_buf_.push_back(s);
      pruneBuffers(s.stamp);
    }
  }
  retryPendingClouds();
}

bool ImuUndistorter::ensureStaticTf(const std::string & scan_frame, const std::string & imu_frame)
{
  if (scan_frame.empty() || imu_frame.empty() || base_frame_.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Cannot deskew with empty base/scan/IMU frame (base='%s', scan='%s', imu='%s')",
      base_frame_.c_str(), scan_frame.c_str(), imu_frame.c_str());
    return false;
  }

  if (cached_scan_frame_ != scan_frame) {
    has_T_base_scan_ = false;
    cached_scan_frame_ = scan_frame;
  }
  if (cached_imu_frame_ != imu_frame) {
    has_T_base_imu_ = false;
    cached_imu_frame_ = imu_frame;
  }

  // base <- scan
  if (!has_T_base_scan_) {
    if (scan_frame == base_frame_) {
      T_base_scan_ = Eigen::Isometry3d::Identity();
      has_T_base_scan_ = true;
    } else {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        base_frame_, scan_frame, tf2::TimePointZero, tf2::durationFromSec(0.2));

      Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                           tf.transform.rotation.y, tf.transform.rotation.z);
      if (!q.coeffs().allFinite() || q.norm() <= 1e-12) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid base<-scan TF quaternion");
        return false;
      }
      q.normalize();

      T_base_scan_ = Eigen::Isometry3d::Identity();
      T_base_scan_.translation() = Eigen::Vector3d(tf.transform.translation.x,
                                                   tf.transform.translation.y,
                                                   tf.transform.translation.z);
      T_base_scan_.linear() = q.toRotationMatrix();

      has_T_base_scan_ = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed (base<-scan): %s", ex.what());
      return false;
    }
    }
  }

  // base <- imu is required unless the IMU already publishes in base_frame.
  if (!has_T_base_imu_) {
    if (imu_frame == base_frame_) {
      T_base_imu_ = Eigen::Isometry3d::Identity();
      has_T_base_imu_ = true;
    } else {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        base_frame_, imu_frame, tf2::TimePointZero, tf2::durationFromSec(0.2));

      Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                           tf.transform.rotation.y, tf.transform.rotation.z);
      if (!q.coeffs().allFinite() || q.norm() <= 1e-12) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid base<-imu TF quaternion");
        return false;
      }
      q.normalize();

      T_base_imu_ = Eigen::Isometry3d::Identity();
      T_base_imu_.translation() = Eigen::Vector3d(tf.transform.translation.x,
                                                  tf.transform.translation.y,
                                                  tf.transform.translation.z);
      T_base_imu_.linear() = q.toRotationMatrix();

      has_T_base_imu_ = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed (base<-imu); deskew is rejected rather than assuming identity: %s", ex.what());
      return false;
    }
    }
  }

  return true;
}

std::size_t ImuUndistorter::datatypeSize(uint8_t datatype)
{
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8:
      return 1U;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16:
      return 2U;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::FLOAT32:
      return 4U;
    case sensor_msgs::msg::PointField::FLOAT64:
      return 8U;
    default:
      return 0U;
  }
}

bool ImuUndistorter::validateCloudLayout(
  const sensor_msgs::msg::PointCloud2 & msg, std::string & out_reason) const
{
  out_reason.clear();
  if (msg.header.frame_id.empty()) {
    out_reason = "empty PointCloud2 frame_id";
    return false;
  }
  const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
  if (stamp.nanoseconds() <= 0) {
    out_reason = "zero PointCloud2 timestamp";
    return false;
  }
  if (msg.is_bigendian) {
    out_reason = "big-endian PointCloud2 is not supported";
    return false;
  }
  if (msg.width == 0U || msg.height == 0U || msg.point_step == 0U) {
    out_reason = "invalid PointCloud2 dimensions or point_step";
    return false;
  }
  const std::size_t min_row_step =
    static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.point_step);
  if (static_cast<std::size_t>(msg.row_step) < min_row_step) {
    out_reason = "row_step is smaller than width * point_step";
    return false;
  }
  const std::size_t required =
    static_cast<std::size_t>(msg.row_step) * static_cast<std::size_t>(msg.height);
  if (msg.data.size() < required) {
    out_reason = "PointCloud2 data is smaller than row_step * height";
    return false;
  }
  const std::size_t point_count =
    static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
  if (point_count > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    out_reason = "PointCloud2 contains too many points for this implementation";
    return false;
  }
  for (const auto & field : msg.fields) {
    const std::size_t field_size = datatypeSize(field.datatype);
    if (field.count == 0U || field_size == 0U) {
      continue;
    }
    if (static_cast<std::size_t>(field.count) >
      std::numeric_limits<std::size_t>::max() / field_size)
    {
      out_reason = "PointCloud2 field count overflows size calculation: " + field.name;
      return false;
    }
    const std::size_t bytes = static_cast<std::size_t>(field.count) * field_size;
    if (static_cast<std::size_t>(field.offset) >
      std::numeric_limits<std::size_t>::max() - bytes)
    {
      out_reason = "PointCloud2 field offset overflows size calculation: " + field.name;
      return false;
    }
    const std::size_t end = static_cast<std::size_t>(field.offset) + bytes;
    if (end > static_cast<std::size_t>(msg.point_step)) {
      out_reason = "PointCloud2 field exceeds point_step: " + field.name;
      return false;
    }
  }
  return true;
}

bool ImuUndistorter::findTimeField(const sensor_msgs::msg::PointCloud2 & msg,
                                   std::string & out_name,
                                   uint8_t & out_datatype,
                                   uint32_t & out_offset) const
{
  for (const auto & name : time_fields_) {
    for (const auto & field : msg.fields) {
      if (field.name != name) {
        continue;
      }
      const std::size_t size = datatypeSize(field.datatype);
      if (field.count == 0U || size == 0U ||
          static_cast<std::size_t>(field.offset) + size > msg.point_step) {
        return false;
      }
      out_name = field.name;
      out_datatype = field.datatype;
      out_offset = field.offset;
      return true;
    }
  }
  return false;
}

bool ImuUndistorter::readFieldAsDouble(const uint8_t * ptr, uint8_t datatype, double & v) const
{
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8: {
      int8_t t; std::memcpy(&t, ptr, sizeof(t)); v = static_cast<double>(t); return true;
    }
    case sensor_msgs::msg::PointField::UINT8: {
      uint8_t t; std::memcpy(&t, ptr, sizeof(t)); v = static_cast<double>(t); return true;
    }
    case sensor_msgs::msg::PointField::INT16: {
      int16_t t; std::memcpy(&t, ptr, sizeof(t)); v = static_cast<double>(t); return true;
    }
    case sensor_msgs::msg::PointField::UINT16: {
      uint16_t t; std::memcpy(&t, ptr, sizeof(t)); v = static_cast<double>(t); return true;
    }
    case sensor_msgs::msg::PointField::INT32: {
      int32_t t; std::memcpy(&t, ptr, sizeof(t)); v = static_cast<double>(t); return true;
    }
    case sensor_msgs::msg::PointField::UINT32: {
      uint32_t t; std::memcpy(&t, ptr, sizeof(t)); v = static_cast<double>(t); return true;
    }
    case sensor_msgs::msg::PointField::FLOAT32: {
      float t; std::memcpy(&t, ptr, sizeof(t)); v = static_cast<double>(t); return true;
    }
    case sensor_msgs::msg::PointField::FLOAT64: {
      double t; std::memcpy(&t, ptr, sizeof(t)); v = t; return true;
    }
    default:
      return false;
  }
}

double ImuUndistorter::estimateTimeScale(double raw_range, double scan_period) const
{
  const double candidates[] = {1.0, 1e-3, 1e-6, 1e-9};
  double best = std::numeric_limits<double>::quiet_NaN();
  double best_score = std::numeric_limits<double>::infinity();

  for (const double scale : candidates) {
    const double range_sec = raw_range * scale;
    if (!std::isfinite(range_sec) || range_sec <= 1e-7 ||
        range_sec > max_time_offset_sec_ + point_time_tolerance_sec_) {
      continue;
    }
    const double score = std::fabs(range_sec - scan_period);
    if (score < best_score) {
      best_score = score;
      best = scale;
    }
  }
  return best;
}

bool ImuUndistorter::preparePointTimeInfo(
  const sensor_msgs::msg::PointCloud2 & msg,
  PointTimeInfo & out,
  std::string & out_reason) const
{
  out = PointTimeInfo{};
  out_reason.clear();

  if (!validateCloudLayout(msg, out_reason)) {
    return false;
  }

  const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
  const double stamp_sec = toSec(stamp);
  const double configured_period = fallback_scan_period_;

  std::string name;
  uint8_t datatype = 0U;
  uint32_t offset = 0U;
  out.has_time_field = findTimeField(msg, name, datatype, offset);

  if (!out.has_time_field) {
    if (!allow_linear_time_fallback_) {
      out_reason = "no supported per-point time field; linear point-order fallback is disabled";
      return false;
    }
    out.used_linear_fallback = true;
    out.interpreted_as_relative = true;
    out.time_scale = 1.0;
    out.raw_origin = 0.0;
    out.scan_period = configured_period;
    out.t0_sec = cloud_stamp_is_start_ ? stamp_sec : stamp_sec - configured_period;
    out.t_ref_sec = reference_time_ == "end" ? out.t0_sec + configured_period : out.t0_sec;
    return true;
  }

  out.field_name = name;
  out.datatype = datatype;
  out.offset = offset;

  const std::size_t point_count =
    static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
  double raw_min = std::numeric_limits<double>::infinity();
  double raw_max = -std::numeric_limits<double>::infinity();

  for (std::size_t index = 0; index < point_count; ++index) {
    const std::size_t row = index / static_cast<std::size_t>(msg.width);
    const std::size_t col = index % static_cast<std::size_t>(msg.width);
    const uint8_t * point = msg.data.data() + row * msg.row_step + col * msg.point_step;
    double raw = 0.0;
    if (!readFieldAsDouble(point + offset, datatype, raw) || !std::isfinite(raw)) {
      out_reason = "invalid per-point time value at point " + std::to_string(index);
      return false;
    }
    raw_min = std::min(raw_min, raw);
    raw_max = std::max(raw_max, raw);
  }

  const double raw_range = raw_max - raw_min;
  if (!std::isfinite(raw_range) || (point_count > 1U && raw_range <= 0.0)) {
    out_reason = "per-point time field has no finite positive span";
    return false;
  }

  // A one-point cloud has no observable time span. It needs no intra-scan
  // correction, so retain the configured scan interval for buffer validation
  // without trying to infer units from a zero range.
  if (point_count == 1U) {
    out.time_scale = time_scale_ > 0.0 ? time_scale_ : 1.0;
  } else {
    out.time_scale = time_scale_ > 0.0 ? time_scale_ :
      estimateTimeScale(raw_range, configured_period);
  }
  if (!std::isfinite(out.time_scale) || out.time_scale <= 0.0) {
    out_reason = "could not infer a safe per-point time scale; configure time_scale explicitly";
    return false;
  }

  const double duration = point_count > 1U ? raw_range * out.time_scale : configured_period;
  if (!std::isfinite(duration) || duration <= 0.0 ||
      duration > max_time_offset_sec_ + point_time_tolerance_sec_) {
    out_reason = "per-point time span is outside the configured safe interval";
    return false;
  }
  out.scan_period = duration;

  const double min_sec = raw_min * out.time_scale;
  const double max_sec = raw_max * out.time_scale;
  const bool epoch_like = std::fabs(min_sec) > 1e6 || std::fabs(max_sec) > 1e6;
  const bool relative_like =
    std::fabs(min_sec) <= max_time_offset_sec_ + point_time_tolerance_sec_ ||
    std::fabs(max_sec) <= max_time_offset_sec_ + point_time_tolerance_sec_;

  out.interpreted_as_relative = epoch_like ? false : (prefer_relative_time_ || relative_like);
  if (out.interpreted_as_relative) {
    out.raw_origin = raw_min;
    out.t0_sec = cloud_stamp_is_start_ ? stamp_sec : stamp_sec - duration;
  } else {
    out.raw_origin = 0.0;
    out.t0_sec = min_sec;
    const double expected_stamp = cloud_stamp_is_start_ ? min_sec : max_sec;
    if (std::fabs(stamp_sec - expected_stamp) > max_time_offset_sec_) {
      out_reason = "absolute point times are inconsistent with the cloud timestamp";
      return false;
    }
  }
  out.t_ref_sec = reference_time_ == "end" ? out.t0_sec + duration : out.t0_sec;
  return true;
}

bool ImuUndistorter::computePointDtSec(const sensor_msgs::msg::PointCloud2 & msg,
                                       const PointTimeInfo & ti,
                                       int point_index,
                                       int point_count,
                                       const uint8_t * point_ptr,
                                       double & out_dt) const
{
  (void)msg;
  if (point_count <= 1) {
    out_dt = 0.0;
    return true;
  }

  if (!ti.has_time_field) {
    if (!ti.used_linear_fallback || !allow_linear_time_fallback_) {
      return false;
    }
    const double alpha = static_cast<double>(point_index) /
      static_cast<double>(point_count - 1);
    out_dt = alpha * ti.scan_period;
    return std::isfinite(out_dt);
  }

  double raw = 0.0;
  if (!readFieldAsDouble(point_ptr + ti.offset, ti.datatype, raw) || !std::isfinite(raw)) {
    return false;
  }

  double dt = ti.interpreted_as_relative ?
    (raw - ti.raw_origin) * ti.time_scale : raw * ti.time_scale - ti.t0_sec;
  if (!std::isfinite(dt) ||
      dt < -point_time_tolerance_sec_ ||
      dt > ti.scan_period + point_time_tolerance_sec_) {
    return false;
  }
  out_dt = std::max(0.0, std::min(ti.scan_period, dt));
  return true;
}

static Eigen::Quaterniond deltaQFromGyro(const Eigen::Vector3d & w_rad_s, double dt)
{
  // small-angle expmap
  const double th = w_rad_s.norm() * dt;
  if (th < 1e-12) {
    return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  }
  const Eigen::Vector3d axis = w_rad_s.normalized();
  const double half = 0.5 * th;
  const double s = std::sin(half);
  return Eigen::Quaterniond(std::cos(half), axis.x()*s, axis.y()*s, axis.z()*s);
}

bool ImuUndistorter::buildBaseTrajectory(double t0_sec, double t1_sec,
                                         std::vector<PoseSample> & out_traj,
                                         std::string & out_reason)
{
  out_traj.clear();
  out_reason.clear();
  if (!std::isfinite(t0_sec) || !std::isfinite(t1_sec) || t1_sec <= t0_sec) {
    out_reason = "invalid trajectory interval";
    return false;
  }
  if (!has_T_base_imu_) {
    out_reason = "base<-imu transform is not available";
    return false;
  }

  std::deque<ImuSample> imu;
  std::deque<TwistSample> twist;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    imu = imu_buf_;
    twist = twist_buf_;
  }
  if (imu.size() < 2U) {
    out_reason = "IMU buffer contains fewer than two samples";
    return false;
  }

  auto imu_time = [](const ImuSample & sample) {return toSec(sample.stamp);};
  const double imu_first = imu_time(imu.front());
  const double imu_last = imu_time(imu.back());
  if (imu_first > t0_sec || imu_last < t1_sec) {
    out_reason = "IMU does not cover the full scan interval";
    return false;
  }

  auto imu_hi_at = [&](double t) {
      return std::lower_bound(
        imu.begin(), imu.end(), t,
        [&](const ImuSample & sample, double value) {return imu_time(sample) < value;});
    };

  auto validate_imu_boundary = [&](double t, const char * label) {
      const auto hi = imu_hi_at(t);
      double nearest = std::numeric_limits<double>::infinity();
      if (hi != imu.end()) {
        nearest = std::min(nearest, std::fabs(imu_time(*hi) - t));
      }
      if (hi != imu.begin()) {
        nearest = std::min(nearest, std::fabs(t - imu_time(*std::prev(hi))));
      }
      if (nearest > max_imu_boundary_gap_sec_) {
        out_reason = std::string("IMU boundary gap is too large at scan ") + label;
        return false;
      }
      return true;
    };
  if (!validate_imu_boundary(t0_sec, "start") || !validate_imu_boundary(t1_sec, "end")) {
    return false;
  }

  auto first_in_window = imu_hi_at(t0_sec);
  if (first_in_window != imu.begin()) {
    --first_in_window;
  }
  for (auto it = first_in_window; std::next(it) != imu.end(); ++it) {
    const auto next = std::next(it);
    const double a = imu_time(*it);
    const double b = imu_time(*next);
    if (b < t0_sec) {
      continue;
    }
    if (a > t1_sec) {
      break;
    }
    const double gap = b - a;
    if (!(gap > 0.0) || gap > max_imu_gap_sec_) {
      std::ostringstream oss;
      oss << "IMU gap " << gap << " s exceeds max_imu_gap_sec";
      out_reason = oss.str();
      return false;
    }
    if (b >= t1_sec) {
      break;
    }
  }

  const Eigen::Matrix3d R_BI = T_base_imu_.linear();
  auto gyro_at = [&](double t, Eigen::Vector3d & gyro_base) {
      const auto hi = imu_hi_at(t);
      if (hi == imu.end()) {
        return false;
      }
      if (std::fabs(imu_time(*hi) - t) <= 1e-12) {
        gyro_base = R_BI * hi->gyro;
        return gyro_base.allFinite();
      }
      if (hi == imu.begin()) {
        return false;
      }
      const auto lo = std::prev(hi);
      const double ta = imu_time(*lo);
      const double tb = imu_time(*hi);
      const double gap = tb - ta;
      if (!(gap > 0.0) || gap > max_imu_gap_sec_) {
        return false;
      }
      const double alpha = (t - ta) / gap;
      const Eigen::Vector3d gyro_imu = (1.0 - alpha) * lo->gyro + alpha * hi->gyro;
      gyro_base = R_BI * gyro_imu;
      return gyro_base.allFinite();
    };

  bool use_default_speed = use_translation_ && !use_twist_speed_;
  if (use_translation_ && use_twist_speed_) {
    if (twist.size() < 2U) {
      if (!allow_default_speed_fallback_) {
        out_reason = "twist buffer contains fewer than two samples";
        return false;
      }
      use_default_speed = true;
    } else {
      const auto twist_time = [](const TwistSample & sample) {return toSec(sample.stamp);};
      if (twist_time(twist.front()) > t0_sec || twist_time(twist.back()) < t1_sec) {
        if (!allow_default_speed_fallback_) {
          out_reason = "twist does not cover the full scan interval";
          return false;
        }
        use_default_speed = true;
      }
      if (!use_default_speed) {
        auto first = std::lower_bound(
          twist.begin(), twist.end(), t0_sec,
          [&](const TwistSample & sample, double value) {return twist_time(sample) < value;});
        if (first != twist.begin()) {
          --first;
        }
        for (auto it = first; std::next(it) != twist.end(); ++it) {
          const auto next = std::next(it);
          const double a = twist_time(*it);
          const double b = twist_time(*next);
          if (b < t0_sec) {
            continue;
          }
          if (a > t1_sec) {
            break;
          }
          const double gap = b - a;
          if (!(gap > 0.0) || gap > max_twist_gap_sec_) {
            if (!allow_default_speed_fallback_) {
              std::ostringstream oss;
              oss << "twist gap " << gap << " s exceeds max_twist_gap_sec";
              out_reason = oss.str();
              return false;
            }
            use_default_speed = true;
            break;
          }
          if (b >= t1_sec) {
            break;
          }
        }
      }
    }
  }

  auto speed_at = [&](double t, double & speed) {
      if (!use_translation_) {
        speed = 0.0;
        return true;
      }
      if (use_default_speed) {
        speed = std::max(-max_speed_mps_, std::min(max_speed_mps_, default_speed_mps_));
        return std::isfinite(speed);
      }
      const auto twist_time = [](const TwistSample & sample) {return toSec(sample.stamp);};
      const auto hi = std::lower_bound(
        twist.begin(), twist.end(), t,
        [&](const TwistSample & sample, double value) {return twist_time(sample) < value;});
      if (hi == twist.end()) {
        return false;
      }
      if (std::fabs(twist_time(*hi) - t) <= 1e-12) {
        speed = hi->speed_mps;
      } else {
        if (hi == twist.begin()) {
          return false;
        }
        const auto lo = std::prev(hi);
        const double ta = twist_time(*lo);
        const double tb = twist_time(*hi);
        const double gap = tb - ta;
        if (!(gap > 0.0) || gap > max_twist_gap_sec_) {
          return false;
        }
        const double alpha = (t - ta) / gap;
        speed = (1.0 - alpha) * lo->speed_mps + alpha * hi->speed_mps;
      }
      speed = std::max(-max_speed_mps_, std::min(max_speed_mps_, speed));
      return std::isfinite(speed);
    };

  std::vector<double> times;
  times.reserve(imu.size() + 2U);
  times.push_back(t0_sec);
  for (const auto & sample : imu) {
    const double t = imu_time(sample);
    if (t > t0_sec && t < t1_sec) {
      times.push_back(t);
    }
  }
  times.push_back(t1_sec);

  Eigen::Quaterniond q_world_base = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_world_base = Eigen::Vector3d::Zero();
  out_traj.reserve(times.size());
  out_traj.push_back(PoseSample{times.front(), q_world_base, p_world_base});

  for (std::size_t index = 1U; index < times.size(); ++index) {
    const double ta = times[index - 1U];
    const double tb = times[index];
    const double dt = tb - ta;
    if (!(dt > 0.0) || dt > max_imu_gap_sec_) {
      out_reason = "invalid integration interval in IMU trajectory";
      out_traj.clear();
      return false;
    }

    Eigen::Vector3d gyro_a;
    Eigen::Vector3d gyro_b;
    if (!gyro_at(ta, gyro_a) || !gyro_at(tb, gyro_b)) {
      out_reason = "failed to interpolate IMU at an integration boundary";
      out_traj.clear();
      return false;
    }
    const Eigen::Vector3d gyro_mid = 0.5 * (gyro_a + gyro_b);
    const Eigen::Quaterniond q_before = q_world_base;
    const Eigen::Quaterniond dq = deltaQFromGyro(gyro_mid, dt);
    q_world_base = (q_world_base * dq).normalized();
    if (!q_world_base.coeffs().allFinite()) {
      out_reason = "non-finite integrated IMU orientation";
      out_traj.clear();
      return false;
    }

    if (use_translation_) {
      double speed = 0.0;
      if (!speed_at(0.5 * (ta + tb), speed)) {
        out_reason = "failed to interpolate twist speed";
        out_traj.clear();
        return false;
      }
      const Eigen::Quaterniond q_mid = q_before.slerp(0.5, q_world_base).normalized();
      p_world_base += q_mid.toRotationMatrix() * Eigen::Vector3d(speed * dt, 0.0, 0.0);
      if (!p_world_base.allFinite()) {
        out_reason = "non-finite integrated translation";
        out_traj.clear();
        return false;
      }
    }
    out_traj.push_back(PoseSample{tb, q_world_base, p_world_base});
  }

  return out_traj.size() >= 2U;
}

bool ImuUndistorter::orientationAt(const std::vector<PoseSample> & traj, double t_sec, Eigen::Quaterniond & q_WB) const
{
  if (traj.empty()) return false;
  if (t_sec <= traj.front().t_sec) { q_WB = traj.front().q_WB; return true; }
  if (t_sec >= traj.back().t_sec) { q_WB = traj.back().q_WB; return true; }

  // find segment
  size_t hi = 1;
  while (hi < traj.size() && traj[hi].t_sec < t_sec) hi++;
  if (hi >= traj.size()) { q_WB = traj.back().q_WB; return true; }
  const size_t lo = hi - 1;

  const double t0 = traj[lo].t_sec;
  const double t1 = traj[hi].t_sec;
  const double a = (t_sec - t0) / std::max(1e-9, (t1 - t0));

  q_WB = traj[lo].q_WB.slerp(a, traj[hi].q_WB).normalized();
  return true;
}

bool ImuUndistorter::positionAt(const std::vector<PoseSample> & traj, double t_sec, Eigen::Vector3d & p_WB) const
{
  if (!use_translation_) { p_WB = Eigen::Vector3d(0,0,0); return true; }
  if (traj.empty()) return false;
  if (t_sec <= traj.front().t_sec) { p_WB = traj.front().p_WB; return true; }
  if (t_sec >= traj.back().t_sec) { p_WB = traj.back().p_WB; return true; }

  size_t hi = 1;
  while (hi < traj.size() && traj[hi].t_sec < t_sec) hi++;
  if (hi >= traj.size()) { p_WB = traj.back().p_WB; return true; }
  const size_t lo = hi - 1;

  const double t0 = traj[lo].t_sec;
  const double t1 = traj[hi].t_sec;
  const double a = (t_sec - t0) / std::max(1e-9, (t1 - t0));

  p_WB = (1.0 - a) * traj[lo].p_WB + a * traj[hi].p_WB;
  return true;
}

bool ImuUndistorter::findXYZOffsets(const sensor_msgs::msg::PointCloud2 & msg,
                                    uint32_t & off_x, uint32_t & off_y, uint32_t & off_z,
                                    uint8_t & dt_x, uint8_t & dt_y, uint8_t & dt_z) const
{
  bool fx=false, fy=false, fz=false;
  for (const auto & field : msg.fields) {
    if (field.count < 1U) {
      continue;
    }
    if (field.name == "x") {
      off_x = field.offset;
      dt_x = field.datatype;
      fx = true;
    } else if (field.name == "y") {
      off_y = field.offset;
      dt_y = field.datatype;
      fy = true;
    } else if (field.name == "z") {
      off_z = field.offset;
      dt_z = field.datatype;
      fz = true;
    }
  }
  return fx && fy && fz;
}

static bool readFloat32(const uint8_t * p, float & v)
{
  std::memcpy(&v, p, 4);
  return std::isfinite(v);
}

static void writeFloat32(uint8_t * p, float v)
{
  std::memcpy(p, &v, 4);
}

bool ImuUndistorter::deskewPointCloud(const sensor_msgs::msg::PointCloud2 & in,
                                      sensor_msgs::msg::PointCloud2 & out,
                                      PointTimeInfo & out_time_info,
                                      std::string & out_reason)
{
  out_reason.clear();
  out_time_info = PointTimeInfo{};

  PointTimeInfo & ti = out_time_info;
  if (!preparePointTimeInfo(in, ti, out_reason)) {
    return false;
  }

  uint32_t off_x = 0U;
  uint32_t off_y = 0U;
  uint32_t off_z = 0U;
  uint8_t dt_x = 0U;
  uint8_t dt_y = 0U;
  uint8_t dt_z = 0U;
  if (!findXYZOffsets(in, off_x, off_y, off_z, dt_x, dt_y, dt_z)) {
    out_reason = "x/y/z fields were not found";
    return false;
  }
  if (dt_x != sensor_msgs::msg::PointField::FLOAT32 ||
      dt_y != sensor_msgs::msg::PointField::FLOAT32 ||
      dt_z != sensor_msgs::msg::PointField::FLOAT32) {
    out_reason = "x/y/z fields must all use FLOAT32";
    return false;
  }
  if (static_cast<std::size_t>(off_x) + sizeof(float) > in.point_step ||
      static_cast<std::size_t>(off_y) + sizeof(float) > in.point_step ||
      static_cast<std::size_t>(off_z) + sizeof(float) > in.point_step) {
    out_reason = "x/y/z field offset exceeds point_step";
    return false;
  }

  const std::size_t point_count_size =
    static_cast<std::size_t>(in.width) * static_cast<std::size_t>(in.height);
  const int point_count = static_cast<int>(point_count_size);

  const double t0 = ti.t0_sec;
  const double t1 = ti.t0_sec + ti.scan_period;
  std::vector<PoseSample> trajectory;
  std::string trajectory_reason;
  if (!buildBaseTrajectory(t0, t1, trajectory, trajectory_reason)) {
    out_reason = "trajectory build failed: " + trajectory_reason;
    return false;
  }

  Eigen::Quaterniond q_ref;
  Eigen::Vector3d p_ref;
  if (!orientationAt(trajectory, ti.t_ref_sec, q_ref) ||
      !positionAt(trajectory, ti.t_ref_sec, p_ref)) {
    out_reason = "reference pose is outside the validated trajectory";
    return false;
  }

  out = in;
  out.header.stamp = fromSec(*get_clock(), ti.t_ref_sec);
  const Eigen::Isometry3d T_scan_base = T_base_scan_.inverse();
  const Eigen::Matrix3d rotation_ref_transpose = q_ref.toRotationMatrix().transpose();

  for (std::size_t index = 0U; index < point_count_size; ++index) {
    const std::size_t row = index / static_cast<std::size_t>(in.width);
    const std::size_t col = index % static_cast<std::size_t>(in.width);
    const std::size_t byte_offset = row * in.row_step + col * in.point_step;
    const uint8_t * point_in = in.data.data() + byte_offset;
    uint8_t * point_out = out.data.data() + byte_offset;

    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    if (!readFloat32(point_in + off_x, x) ||
        !readFloat32(point_in + off_y, y) ||
        !readFloat32(point_in + off_z, z)) {
      // Preserve invalid points; the layout and their time fields were already validated.
      continue;
    }

    double dt = 0.0;
    if (!computePointDtSec(in, ti, static_cast<int>(index), point_count, point_in, dt)) {
      out_reason = "invalid per-point time at point " + std::to_string(index);
      return false;
    }
    const double point_time = ti.t0_sec + dt;

    Eigen::Quaterniond q_point;
    Eigen::Vector3d p_point;
    if (!orientationAt(trajectory, point_time, q_point) ||
        !positionAt(trajectory, point_time, p_point)) {
      out_reason = "point time is outside the validated trajectory";
      return false;
    }

    const Eigen::Matrix3d rotation_relative =
      rotation_ref_transpose * q_point.toRotationMatrix();
    const Eigen::Vector3d translation_relative =
      rotation_ref_transpose * (p_point - p_ref);

    const Eigen::Vector3d point_scan(x, y, z);
    const Eigen::Vector3d point_base =
      (T_base_scan_ * point_scan.homogeneous()).head<3>();
    const Eigen::Vector3d point_base_ref =
      rotation_relative * point_base + translation_relative;
    const Eigen::Vector3d point_scan_ref =
      (T_scan_base * point_base_ref.homogeneous()).head<3>();
    if (!point_scan_ref.allFinite()) {
      out_reason = "deskew produced a non-finite point";
      return false;
    }

    writeFloat32(point_out + off_x, static_cast<float>(point_scan_ref.x()));
    writeFloat32(point_out + off_y, static_cast<float>(point_scan_ref.y()));
    writeFloat32(point_out + off_z, static_cast<float>(point_scan_ref.z()));
  }

  return true;
}

void ImuUndistorter::publishDiag(const rclcpp::Time & stamp,
                                 const std::string & level,
                                 const std::string & msg,
                                 const PointTimeInfo & ti) const
{
  if (!publish_diagnostics_) return;

  diagnostic_msgs::msg::DiagnosticArray da;
  da.header.stamp = stamp;

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "localization/imu_undistortion";
  st.hardware_id = "none";
  st.level = toDiagLevel(level);
  st.message = msg;

  auto addKV = [&](const std::string & k, const std::string & v){
    diagnostic_msgs::msg::KeyValue kv; kv.key=k; kv.value=v; st.values.push_back(kv);
  };
  const double msg_age_ms = (this->now() - stamp).seconds() * 1000.0;
  addKV("msg_age_ms", std::to_string(msg_age_ms));

  addKV("has_time_field", ti.has_time_field ? "true" : "false");
  addKV("time_field_name", ti.has_time_field ? ti.field_name : "");
  addKV("used_linear_fallback", ti.used_linear_fallback ? "true" : "false");
  addKV("allow_linear_time_fallback", allow_linear_time_fallback_ ? "true" : "false");
  addKV("scan_period", std::to_string(ti.scan_period));
  addKV("time_scale", std::to_string(ti.time_scale));
  addKV("interpreted_as_relative", ti.interpreted_as_relative ? "true" : "false");
  addKV("reference_time", reference_time_);
  addKV("use_translation", use_translation_ ? "true" : "false");
  addKV("use_twist_speed", use_twist_speed_ ? "true" : "false");
  addKV("allow_default_speed_fallback", allow_default_speed_fallback_ ? "true" : "false");

  da.status.push_back(st);
  pub_diag_->publish(da);
}

bool ImuUndistorter::shouldWaitForFutureSensorData(
  const PointTimeInfo & time_info,
  const std::string & reason) const
{
  if (!(time_info.scan_period > 0.0) || !std::isfinite(time_info.t0_sec)) {
    return false;
  }

  const double t0_sec = time_info.t0_sec;
  const double t1_sec = t0_sec + time_info.scan_period;
  const bool imu_may_arrive =
    reason == "trajectory build failed: IMU buffer contains fewer than two samples" ||
    reason == "trajectory build failed: IMU does not cover the full scan interval";
  const bool twist_may_arrive =
    reason == "trajectory build failed: twist buffer contains fewer than two samples" ||
    reason == "trajectory build failed: twist does not cover the full scan interval";

  std::lock_guard<std::mutex> lock(mtx_);
  if (imu_may_arrive && !imu_buf_.empty()) {
    const double first_sec = toSec(imu_buf_.front().stamp);
    const double last_sec = toSec(imu_buf_.back().stamp);
    // Only a missing future boundary can be repaired by waiting. Missing data
    // at the scan start is permanent and remains an immediate rejection.
    return first_sec <= t0_sec && last_sec < t1_sec;
  }
  if (twist_may_arrive && use_translation_ && use_twist_speed_ &&
      !allow_default_speed_fallback_ && !twist_topic_.empty()) {
    if (twist_buf_.empty()) {
      return true;
    }
    const double first_sec = toSec(twist_buf_.front().stamp);
    const double last_sec = toSec(twist_buf_.back().stamp);
    return first_sec <= t0_sec && last_sec < t1_sec;
  }
  return false;
}

ImuUndistorter::CloudProcessResult ImuUndistorter::processPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
{

  PointTimeInfo time_info;
  std::string layout_reason;
  if (!validateCloudLayout(*msg, layout_reason)) {
    publishDiag(msg->header.stamp, "WARN", "deskew rejected: " + layout_reason, time_info);
    return CloudProcessResult::Complete;
  }

  const std::string message_scan_frame = msg->header.frame_id;
  if (!scan_frame_.empty() && scan_frame_ != message_scan_frame) {
    publishDiag(
      msg->header.stamp, "ERROR",
      "configured scan_frame does not match PointCloud2.header.frame_id", time_info);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejected cloud frame mismatch: expected '%s', got '%s'",
      scan_frame_.c_str(), message_scan_frame.c_str());
    return CloudProcessResult::Complete;
  }
  const std::string scan_frame = message_scan_frame;

  std::string imu_frame_used;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    imu_frame_used = imu_frame_.empty() ? observed_imu_frame_ : imu_frame_;
  }
  if (imu_frame_used.empty()) {
    publishDiag(msg->header.stamp, "STALE", "deskew rejected: no IMU frame observed yet", time_info);
    return CloudProcessResult::Complete;
  }

  if (!ensureStaticTf(scan_frame, imu_frame_used)) {
    publishDiag(msg->header.stamp, "WARN", "deskew rejected: required static TF is unavailable", time_info);
    return CloudProcessResult::Complete;
  }

  sensor_msgs::msg::PointCloud2 out;
  std::string reason;
  if (!deskewPointCloud(*msg, out, time_info, reason)) {
    if (shouldWaitForFutureSensorData(time_info, reason)) {
      return CloudProcessResult::WaitingForFutureSensorData;
    }
    publishDiag(msg->header.stamp, "WARN", "deskew rejected: " + reason, time_info);
    return CloudProcessResult::Complete;
  }

  pub_points_->publish(out);
  publishDiag(
    out.header.stamp,
    time_info.used_linear_fallback ? "WARN" : "OK",
    time_info.used_linear_fallback ?
    "deskew OK using explicitly enabled point-order timing fallback" :
    "deskew OK using per-point timestamps",
    time_info);
  return CloudProcessResult::Complete;
}

void ImuUndistorter::drainPendingClouds()
{
  while (!pending_clouds_.empty()) {
    if (processPointCloud(pending_clouds_.front()) ==
      CloudProcessResult::WaitingForFutureSensorData)
    {
      break;
    }
    pending_clouds_.pop_front();
  }
}

void ImuUndistorter::retryPendingClouds()
{
  std::unique_lock<std::mutex> callback_lock(points_callback_mtx_, std::try_to_lock);
  if (!callback_lock.owns_lock()) {
    return;
  }
  drainPendingClouds();
}

void ImuUndistorter::onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // A mutually exclusive callback group is not guaranteed when this component
  // is embedded in an arbitrary executor, so serialize scan processing here.
  std::lock_guard<std::mutex> callback_lock(points_callback_mtx_);

  if (pending_clouds_.size() >= static_cast<std::size_t>(max_pending_clouds_)) {
    PointTimeInfo time_info;
    std::string ignored_reason;
    const auto & dropped = pending_clouds_.front();
    (void)preparePointTimeInfo(*dropped, time_info, ignored_reason);
    publishDiag(
      dropped->header.stamp, "WARN",
      "deskew rejected: pending cloud queue filled before sensor coverage arrived",
      time_info);
    pending_clouds_.pop_front();
  }
  pending_clouds_.push_back(msg);
  drainPendingClouds();
}

}  // namespace pure_imu_undistortion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pure_imu_undistortion::ImuUndistorter)
