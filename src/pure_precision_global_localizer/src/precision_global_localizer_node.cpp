// SPDX-License-Identifier: Apache-2.0
#include "pure_precision_global_localizer/precision_global_localizer_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pure_gnss_msgs/msg/gnss_fusion_input.hpp>
#include <pure_lidar_msgs/msg/submap_correction.hpp>
#include <pure_lidar_msgs/msg/submap_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "pure_precision_global_localizer/precision_anchor_estimator.hpp"
#include "pure_precision_global_localizer/existing_fusion_anchor_tracker.hpp"
#include "pure_precision_global_localizer/precision_local_compositor.hpp"

namespace pure_precision_global_localizer
{

namespace
{
using GnssInput = pure_gnss_msgs::msg::GnssFusionInput;
using SubmapCorrection = pure_lidar_msgs::msg::SubmapCorrection;
using SubmapScan = pure_lidar_msgs::msg::SubmapScan;

constexpr std::size_t kScanCacheLimit = 512U;
constexpr std::size_t kAcceptedKeyLimit = 2048U;
constexpr std::size_t kPendingCorrectionLimit = 64U;
constexpr std::size_t kPendingGnssLimit = 64U;
constexpr double kPoseContractTolerance = 1.0e-5;

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

bool validStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U &&
         (stamp.sec != 0 || stamp.nanosec != 0U);
}

struct ScanKey
{
  uint64_t session{0U};
  uint64_t generation{0U};
  uint64_t sequence{0U};
  int32_t sec{0};
  uint32_t nanosec{0U};

  bool operator<(const ScanKey & other) const
  {
    return std::tie(session, generation, sequence, sec, nanosec) <
           std::tie(other.session, other.generation, other.sequence, other.sec, other.nanosec);
  }
  bool operator==(const ScanKey & other) const
  {
    return session == other.session && generation == other.generation &&
           sequence == other.sequence && sec == other.sec && nanosec == other.nanosec;
  }
};

ScanKey keyFrom(const SubmapScan & message)
{
  return {
    message.odom_session_id, message.odom_generation, message.sequence,
    message.header.stamp.sec, message.header.stamp.nanosec};
}

ScanKey keyFrom(const SubmapCorrection & message)
{
  return {
    message.odom_session_id, message.odom_generation, message.sequence,
    message.header.stamp.sec, message.header.stamp.nanosec};
}

struct QuaternionInfo
{
  bool valid{false};
  tf2::Quaternion quaternion;
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  double original_norm{0.0};
};

template<typename QuaternionMessage>
QuaternionInfo quaternionInfo(const QuaternionMessage & message)
{
  QuaternionInfo result;
  if (!std::isfinite(message.x) || !std::isfinite(message.y) ||
    !std::isfinite(message.z) || !std::isfinite(message.w))
  {
    return result;
  }
  tf2::Quaternion quaternion(message.x, message.y, message.z, message.w);
  if (!(quaternion.length2() > 1.0e-12) || !std::isfinite(quaternion.length2())) {
    return result;
  }
  result.original_norm = std::sqrt(quaternion.length2());
  quaternion.normalize();
  tf2::Matrix3x3(quaternion).getRPY(result.roll, result.pitch, result.yaw);
  result.valid = std::isfinite(result.roll) && std::isfinite(result.pitch) &&
    std::isfinite(result.yaw);
  result.quaternion = quaternion;
  return result;
}

template<typename PoseMessage>
bool pose2From(const PoseMessage & message, Pose2 & pose, QuaternionInfo * info = nullptr)
{
  const auto quaternion = quaternionInfo(message.orientation);
  if (!quaternion.valid || !std::isfinite(message.position.x) ||
    !std::isfinite(message.position.y) || !std::isfinite(message.position.z))
  {
    return false;
  }
  pose = {message.position.x, message.position.y, quaternion.yaw};
  if (info != nullptr) {
    *info = quaternion;
  }
  return pose.finite();
}

Pose2 transform2From(const geometry_msgs::msg::Transform & message, bool & valid)
{
  const auto quaternion = quaternionInfo(message.rotation);
  valid = quaternion.valid && std::isfinite(message.translation.x) &&
    std::isfinite(message.translation.y) && std::isfinite(message.translation.z) &&
    std::fabs(message.translation.z) <= kPoseContractTolerance &&
    std::fabs(quaternion.roll) <= kPoseContractTolerance &&
    std::fabs(quaternion.pitch) <= kPoseContractTolerance &&
    std::fabs(quaternion.original_norm - 1.0) <= 1.0e-3;
  return {message.translation.x, message.translation.y, quaternion.yaw};
}

geometry_msgs::msg::Quaternion quaternionMessage(const tf2::Quaternion & quaternion)
{
  geometry_msgs::msg::Quaternion message;
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  message.w = quaternion.w();
  return message;
}

tf2::Quaternion yawQuaternion(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  quaternion.normalize();
  return quaternion;
}

Eigen::Matrix3d covariance3From(const std::array<double, 36> & covariance)
{
  Eigen::Matrix3d result;
  result <<
    covariance[0], covariance[1], covariance[5],
    covariance[6], covariance[7], covariance[11],
    covariance[30], covariance[31], covariance[35];
  return projectCovariancePsd(result);
}

bool strictCovariance3From(
  const std::array<double, 36> & covariance,
  Eigen::Matrix3d & result)
{
  if (!std::all_of(
      covariance.begin(), covariance.end(),
      [](double value) {return std::isfinite(value);}))
  {
    return false;
  }
  result <<
    covariance[0], covariance[1], covariance[5],
    covariance[6], covariance[7], covariance[11],
    covariance[30], covariance[31], covariance[35];
  if ((result - result.transpose()).norm() > 1.0e-6) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(result);
  return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -1.0e-9;
}

void writeCovariance3(
  const Eigen::Matrix3d & covariance, std::array<double, 36> & destination)
{
  const Eigen::Matrix3d safe = projectCovariancePsd(covariance);
  destination[0] = safe(0, 0);
  destination[1] = safe(0, 1);
  destination[5] = safe(0, 2);
  destination[6] = safe(1, 0);
  destination[7] = safe(1, 1);
  destination[11] = safe(1, 2);
  destination[30] = safe(2, 0);
  destination[31] = safe(2, 1);
  destination[35] = safe(2, 2);
}

std::string number(double value)
{
  if (!std::isfinite(value)) {
    return "nan";
  }
  std::ostringstream stream;
  stream << std::setprecision(12) << value;
  return stream.str();
}

struct ScanRecord
{
  ScanKey key;
  std::string raw_frame;
  Pose2 raw_pose;
  double raw_z{0.0};
};

struct LocalRecord
{
  double stamp_sec{0.0};
  Pose2 pose;
  double z{0.0};
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity()};
};

struct InterpolatedLocal
{
  bool valid{false};
  Pose2 pose;
  double z{0.0};
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity()};
};

struct ExistingGlobalRecord
{
  double stamp_sec{0.0};
  Pose2 pose;
  double z{0.0};
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity()};
};

struct InterpolatedGlobal
{
  bool valid{false};
  Pose2 pose;
  double z{0.0};
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity()};
  double sync_error_sec{std::numeric_limits<double>::quiet_NaN()};
  std::string mode{"none"};
};

struct FusionHealthSnapshot
{
  bool received{false};
  bool strict_fields_healthy{false};
  double stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  uint8_t level{diagnostic_msgs::msg::DiagnosticStatus::STALE};
  std::string recovery_state{"unknown"};
  std::string anchor_valid{"false"};
  std::string position_fused{"false"};
  std::string yaw_fused{"false"};
  std::string last_fix_state{"unknown"};
  std::string reason{"fusion_diagnostics_unavailable"};
};

struct Counters
{
  uint64_t raw_received{0U};
  uint64_t raw_invalid{0U};
  uint64_t raw_nonmonotonic{0U};
  uint64_t raw_duplicate_stamp{0U};
  uint64_t local_published{0U};
  uint64_t global_published{0U};
  uint64_t global_suppressed_not_ready{0U};
  uint64_t global_suppressed_activation_watermark{0U};
  uint64_t existing_global_received{0U};
  uint64_t existing_global_accepted{0U};
  uint64_t existing_global_rejected{0U};
  uint64_t existing_global_duplicate_stamp{0U};
  uint64_t fusion_diagnostics_received{0U};
  uint64_t fusion_diagnostics_accepted{0U};
  uint64_t fusion_diagnostics_rejected{0U};
  uint64_t fusion_sync_accepted{0U};
  uint64_t fusion_sync_rejected{0U};
  uint64_t scan_received{0U};
  uint64_t scan_rejected{0U};
  uint64_t correction_received{0U};
  uint64_t correction_accepted{0U};
  uint64_t correction_rejected{0U};
  uint64_t correction_pending{0U};
  uint64_t correction_expired{0U};
  uint64_t session_rebases{0U};
  uint64_t epoch_resets{0U};
  uint64_t odom_session_resets{0U};
  uint64_t gnss_received{0U};
  uint64_t gnss_accepted{0U};
  uint64_t gnss_rejected{0U};
  uint64_t gnss_unsynced{0U};
  uint64_t gnss_pending{0U};
  uint64_t gnss_pending_expired{0U};
  uint64_t ignored_direct_heading{0U};
  uint64_t soft_gap_count{0U};
  uint64_t outage_count{0U};
  uint64_t recovery_count{0U};
};
}  // namespace

class PrecisionGlobalLocalizerNode final : public rclcpp::Node
{
public:
  explicit PrecisionGlobalLocalizerNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("precision_global_localizer", options)
  {
    loadParameters();
    validateParameters();
    compositor_ = std::make_unique<PrecisionLocalCompositor>(local_correction_config_);
    gnss_anchor_ = std::make_unique<PrecisionAnchorEstimator>(anchor_config_);
    fusion_anchor_ = std::make_unique<ExistingFusionAnchorTracker>(fusion_anchor_config_);

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;

    raw_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PrecisionGlobalLocalizerNode::onRawOdom, this, std::placeholders::_1),
      subscription_options);
    scan_subscription_ = create_subscription<SubmapScan>(
      submap_scan_topic_, rclcpp::SensorDataQoS().keep_last(10),
      std::bind(&PrecisionGlobalLocalizerNode::onSubmapScan, this, std::placeholders::_1),
      subscription_options);
    correction_subscription_ = create_subscription<SubmapCorrection>(
      submap_correction_topic_, rclcpp::QoS(50).reliable(),
      std::bind(&PrecisionGlobalLocalizerNode::onSubmapCorrection, this, std::placeholders::_1),
      subscription_options);
    existing_global_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      existing_global_odom_topic_, rclcpp::QoS(100).reliable(),
      std::bind(
        &PrecisionGlobalLocalizerNode::onExistingGlobal, this, std::placeholders::_1),
      subscription_options);
    fusion_diagnostics_subscription_ =
      create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      fusion_diagnostics_topic_, rclcpp::QoS(50).reliable(),
      std::bind(
        &PrecisionGlobalLocalizerNode::onFusionDiagnostics, this, std::placeholders::_1),
      subscription_options);
    if (gnss_position_diagnostics_enabled_) {
      gnss_subscription_ = create_subscription<GnssInput>(
        gnss_input_topic_, rclcpp::SensorDataQoS().keep_last(50),
        std::bind(&PrecisionGlobalLocalizerNode::onGnss, this, std::placeholders::_1),
        subscription_options);
    }

    local_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      precision_local_odom_topic_, 20);
    global_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      precision_global_odom_topic_, 20);
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      precision_global_pose_topic_, 20);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(diagnostics_period_sec_)),
      std::bind(&PrecisionGlobalLocalizerNode::publishDiagnostics, this),
      callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "isolated precision global localizer ready: raw=%s local=%s existing_global=%s "
      "global=%s authority=existing_fusion (no TF)",
      raw_odom_topic_.c_str(), precision_local_odom_topic_.c_str(),
      existing_global_odom_topic_.c_str(), precision_global_odom_topic_.c_str());
  }

private:
  void loadParameters();
  void validateParameters() const;
  void onRawOdom(nav_msgs::msg::Odometry::ConstSharedPtr message);
  void onSubmapScan(SubmapScan::ConstSharedPtr message);
  void onSubmapCorrection(SubmapCorrection::ConstSharedPtr message);
  void onExistingGlobal(nav_msgs::msg::Odometry::ConstSharedPtr message);
  void onFusionDiagnostics(diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message);
  void onGnss(GnssInput::ConstSharedPtr message);
  bool processCorrectionLocked(const SubmapCorrection & message, std::string & reason);
  bool processGnssLocked(const GnssInput & message, std::string & reason);
  InterpolatedLocal interpolateLocalLocked(double stamp_sec) const;
  InterpolatedGlobal interpolateExistingGlobalLocked(double stamp_sec) const;
  void processFusionCandidatesLocked();
  bool strictFusionHealthLocked(double reference_stamp_sec, std::string & reason);
  void updateFusionHealthLocked(double reference_stamp_sec);
  void forceFusionUnhealthyLocked(double reference_stamp_sec, const std::string & reason);
  void processPendingCorrectionsLocked();
  void processPendingGnssLocked();
  nav_msgs::msg::Odometry makePrecisionLocalLocked(
    const nav_msgs::msg::Odometry & raw,
    const LocalCompositionResult & composition,
    const QuaternionInfo & raw_quaternion);
  void publishGlobalLocked(const nav_msgs::msg::Odometry & local, const Pose2 & local_pose);
  void noteStateTransition(AnchorState before, AnchorState after);
  void publishDiagnostics();

  std::mutex mutex_;
  std::unique_ptr<PrecisionLocalCompositor> compositor_;
  std::unique_ptr<PrecisionAnchorEstimator> gnss_anchor_;
  std::unique_ptr<ExistingFusionAnchorTracker> fusion_anchor_;
  LocalCorrectionConfig local_correction_config_;
  AnchorConfig anchor_config_;
  FusionAnchorConfig fusion_anchor_config_;

  std::string raw_odom_topic_;
  std::string submap_scan_topic_;
  std::string submap_correction_topic_;
  std::string gnss_input_topic_;
  std::string existing_global_odom_topic_;
  std::string fusion_diagnostics_topic_;
  std::string fusion_diagnostic_status_name_;
  std::string precision_local_odom_topic_;
  std::string precision_global_odom_topic_;
  std::string precision_global_pose_topic_;
  std::string precision_frame_;
  std::string map_frame_;
  std::string base_frame_;
  double sync_history_sec_{30.0};
  double sync_max_extrapolation_sec_{0.12};
  double sync_max_pending_sec_{2.0};
  double min_gnss_confidence_{0.35};
  double fusion_health_max_age_sec_{1.5};
  double fusion_health_max_future_skew_sec_{0.25};
  double fusion_sync_max_interpolation_gap_sec_{0.15};
  double fusion_sync_max_candidate_age_sec_{0.50};
  double existing_global_max_age_sec_{0.25};
  double existing_global_max_future_skew_sec_{0.05};
  bool gnss_position_diagnostics_enabled_{false};
  double diagnostics_period_sec_{1.0};

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_subscription_;
  rclcpp::Subscription<SubmapScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<SubmapCorrection>::SharedPtr correction_subscription_;
  rclcpp::Subscription<GnssInput>::SharedPtr gnss_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr existing_global_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    fusion_diagnostics_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  std::map<ScanKey, ScanRecord> scan_cache_;
  std::deque<ScanKey> scan_order_;
  std::set<ScanKey> accepted_correction_keys_;
  std::deque<ScanKey> accepted_key_order_;
  std::deque<SubmapCorrection::ConstSharedPtr> pending_corrections_;
  std::deque<LocalRecord> local_history_;
  std::deque<LocalRecord> pending_fusion_locals_;
  std::deque<ExistingGlobalRecord> existing_global_history_;
  std::deque<GnssInput::ConstSharedPtr> pending_gnss_;
  FusionHealthSnapshot fusion_health_;
  FusionRearmState fusion_rearm_state_;
  double fusion_rearm_reset_stamp_sec_{std::numeric_limits<double>::quiet_NaN()};
  Counters counters_;
  bool position_fused_{false};
  bool has_latest_local_stamp_{false};
  double latest_local_stamp_sec_{0.0};
  int8_t last_fix_quality_{-1};
  double last_usable_stamp_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_q4_usable_stamp_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_sync_to_current_sec_{std::numeric_limits<double>::quiet_NaN()};
  double current_anchor_lag_translation_m_{0.0};
  double current_anchor_lag_yaw_rad_{0.0};
  double max_anchor_lag_translation_m_{0.0};
  double max_anchor_lag_yaw_rad_{0.0};
  double current_local_lag_translation_m_{0.0};
  double current_local_lag_yaw_rad_{0.0};
  double max_local_lag_translation_m_{0.0};
  double max_local_lag_yaw_rad_{0.0};
  std::string last_gnss_reason_{"none"};
  std::string last_correction_reason_{"none"};
  std::string last_fusion_sync_reason_{"none"};
  std::string last_fusion_sync_mode_{"none"};
  double last_fusion_sync_error_sec_{std::numeric_limits<double>::quiet_NaN()};
  double fusion_health_age_sec_{std::numeric_limits<double>::quiet_NaN()};
  double existing_global_age_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_valid_fusion_sync_stamp_sec_{std::numeric_limits<double>::quiet_NaN()};
  double valid_fusion_sync_age_sec_{std::numeric_limits<double>::quiet_NaN()};
  bool has_fusion_anchor_local_covariance_ref_{false};
  Eigen::Matrix3d fusion_anchor_local_covariance_ref_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d frozen_anchor_residual_covariance_{Eigen::Matrix3d::Zero()};
  std::string active_raw_frame_;
};

void PrecisionGlobalLocalizerNode::loadParameters()
{
  raw_odom_topic_ = declare_parameter<std::string>(
    "raw_odom_topic", "/localization/gyro_lidar_odom");
  submap_scan_topic_ = declare_parameter<std::string>(
    "submap_scan_topic", "/localization/submap_scan");
  submap_correction_topic_ = declare_parameter<std::string>(
    "submap_correction_topic", "/localization/submap_correction");
  gnss_input_topic_ = declare_parameter<std::string>(
    "gnss_input_topic", "/localization/gnss_fusion_input");
  existing_global_odom_topic_ = declare_parameter<std::string>(
    "existing_global_odom_topic", "/localization/ekf_odom");
  fusion_diagnostics_topic_ = declare_parameter<std::string>(
    "fusion_diagnostics_topic", "/diagnostics");
  fusion_diagnostic_status_name_ = declare_parameter<std::string>(
    "fusion_health.status_name", "localization/gnss_map_odom_fusion");
  precision_local_odom_topic_ = declare_parameter<std::string>(
    "precision_local_odom_topic", "/localization/precision_local_odom");
  precision_global_odom_topic_ = declare_parameter<std::string>(
    "precision_global_odom_topic", "/localization/precision_global_odom");
  precision_global_pose_topic_ = declare_parameter<std::string>(
    "precision_global_pose_topic", "/localization/precision_global_pose");
  precision_frame_ = declare_parameter<std::string>("precision_frame", "odom_precision");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

  sync_history_sec_ = declare_parameter<double>("sync.history_sec", 30.0);
  sync_max_extrapolation_sec_ = declare_parameter<double>(
    "sync.max_extrapolation_sec", 0.12);
  sync_max_pending_sec_ = declare_parameter<double>("sync.max_pending_sec", 2.0);
  min_gnss_confidence_ = declare_parameter<double>("gnss.min_confidence", 0.35);
  anchor_config_.max_position_variance_m2 = declare_parameter<double>(
    "gnss.max_position_variance_m2", 25.0);
  anchor_config_.min_position_variance_m2 = declare_parameter<double>(
    "gnss.min_position_variance_m2", 0.0025);
  anchor_config_.hard_outage_sec = declare_parameter<double>("gnss.hard_outage_sec", 2.0);

  anchor_config_.window_sec = declare_parameter<double>("alignment.window_sec", 20.0);
  anchor_config_.max_sample_gap_sec = declare_parameter<double>(
    "alignment.max_sample_gap_sec", 1.0);
  anchor_config_.yaw_sample_min_interval_sec = declare_parameter<double>(
    "alignment.yaw_sample_min_interval_sec", 0.25);
  const int max_yaw_samples = declare_parameter<int>("alignment.max_yaw_samples", 100);
  anchor_config_.max_yaw_samples = max_yaw_samples > 0 ?
    static_cast<std::size_t>(max_yaw_samples) : 0U;
  const int min_yaw_samples = declare_parameter<int>("alignment.min_yaw_samples", 6);
  anchor_config_.min_yaw_samples = min_yaw_samples > 0 ?
    static_cast<std::size_t>(min_yaw_samples) : 0U;
  anchor_config_.min_local_baseline_m = declare_parameter<double>(
    "alignment.min_local_baseline_m", 3.0);
  anchor_config_.min_map_baseline_m = declare_parameter<double>(
    "alignment.min_map_baseline_m", 3.0);
  anchor_config_.inlier_gate_m = declare_parameter<double>(
    "alignment.inlier_gate_m", 1.5);
  anchor_config_.max_rms_m = declare_parameter<double>("alignment.max_rms_m", 0.75);
  anchor_config_.max_yaw_stddev_rad = declare_parameter<double>(
    "alignment.max_yaw_stddev_rad", 0.15);
  anchor_config_.max_committed_yaw_innovation_rad = declare_parameter<double>(
    "alignment.max_committed_yaw_innovation_rad", 0.70);
  const int activation_min_stable_yaw_candidates = declare_parameter<int>(
    "alignment.activation_min_stable_yaw_candidates", 3);
  anchor_config_.activation_min_stable_yaw_candidates =
    activation_min_stable_yaw_candidates > 0 ?
    static_cast<std::size_t>(activation_min_stable_yaw_candidates) : 0U;
  anchor_config_.activation_max_yaw_candidate_delta_rad = declare_parameter<double>(
    "alignment.activation_max_yaw_candidate_delta_rad", 0.08);
  anchor_config_.bootstrap_yaw_rad = declare_parameter<double>(
    "alignment.bootstrap_yaw_rad", 0.0);

  anchor_config_.max_translation_rate_mps = declare_parameter<double>(
    "anchor.max_translation_rate_mps", 1.0);
  anchor_config_.max_yaw_rate_radps = declare_parameter<double>(
    "anchor.max_yaw_rate_radps", 0.20);
  anchor_config_.max_translation_step_m = declare_parameter<double>(
    "anchor.max_translation_step_m", 0.20);
  anchor_config_.max_yaw_step_rad = declare_parameter<double>(
    "anchor.max_yaw_step_rad", 0.04);
  anchor_config_.max_correction_dt_sec = declare_parameter<double>(
    "anchor.max_correction_dt_sec", 0.25);
  anchor_config_.unobservable_yaw_variance_rad2 = declare_parameter<double>(
    "anchor.unobservable_yaw_variance_rad2", 2.4674011002723395);
  anchor_config_.min_yaw_variance_rad2 = declare_parameter<double>(
    "anchor.min_yaw_variance_rad2", 0.0004);
  anchor_config_.outage_xy_variance_rate_m2ps = declare_parameter<double>(
    "anchor.outage_xy_variance_rate_m2ps", 0.02);
  anchor_config_.outage_yaw_variance_rate_rad2ps = declare_parameter<double>(
    "anchor.outage_yaw_variance_rate_rad2ps", 0.0004);

  const int fusion_stable_candidate_count = declare_parameter<int>(
    "fusion_anchor.stable_candidate_count", 3);
  fusion_anchor_config_.stable_candidate_count = fusion_stable_candidate_count > 0 ?
    static_cast<std::size_t>(fusion_stable_candidate_count) : 0U;
  fusion_anchor_config_.candidate_min_interval_sec = declare_parameter<double>(
    "fusion_anchor.candidate_min_interval_sec", 0.25);
  fusion_anchor_config_.candidate_max_gap_sec = declare_parameter<double>(
    "fusion_anchor.candidate_max_gap_sec", 1.25);
  fusion_anchor_config_.stable_max_base_translation_m = declare_parameter<double>(
    "fusion_anchor.stable_max_base_translation_m", 0.50);
  fusion_anchor_config_.stable_max_yaw_rad = declare_parameter<double>(
    "fusion_anchor.stable_max_yaw_rad", 0.08);
  fusion_anchor_config_.tracking_max_base_translation_m = declare_parameter<double>(
    "fusion_anchor.tracking_max_base_translation_m", 2.0);
  fusion_anchor_config_.tracking_max_yaw_rad = declare_parameter<double>(
    "fusion_anchor.tracking_max_yaw_rad", 0.25);
  fusion_anchor_config_.max_translation_rate_mps = declare_parameter<double>(
    "fusion_anchor.max_translation_rate_mps", 1.0);
  fusion_anchor_config_.max_yaw_rate_radps = declare_parameter<double>(
    "fusion_anchor.max_yaw_rate_radps", 0.20);
  fusion_anchor_config_.max_translation_step_m = declare_parameter<double>(
    "fusion_anchor.max_translation_step_m", 0.20);
  fusion_anchor_config_.max_yaw_step_rad = declare_parameter<double>(
    "fusion_anchor.max_yaw_step_rad", 0.04);
  fusion_anchor_config_.max_step_dt_sec = declare_parameter<double>(
    "fusion_anchor.max_step_dt_sec", 0.25);

  fusion_health_max_age_sec_ = declare_parameter<double>(
    "fusion_health.max_age_sec", 1.5);
  fusion_health_max_future_skew_sec_ = declare_parameter<double>(
    "fusion_health.max_future_skew_sec", 0.25);
  fusion_sync_max_interpolation_gap_sec_ = declare_parameter<double>(
    "fusion_sync.max_interpolation_gap_sec", 0.15);
  fusion_sync_max_candidate_age_sec_ = declare_parameter<double>(
    "fusion_sync.max_candidate_age_sec", 0.50);
  existing_global_max_age_sec_ = declare_parameter<double>(
    "fusion_sync.existing_global_max_age_sec", 0.25);
  existing_global_max_future_skew_sec_ = declare_parameter<double>(
    "fusion_sync.existing_global_max_future_skew_sec", 0.05);
  gnss_position_diagnostics_enabled_ = declare_parameter<bool>(
    "fallback.gnss_position_enabled", false);

  local_correction_config_.max_translation_rate_mps = declare_parameter<double>(
    "local_correction.max_translation_rate_mps", 2.0);
  local_correction_config_.max_yaw_rate_radps = declare_parameter<double>(
    "local_correction.max_yaw_rate_radps", 0.50);
  local_correction_config_.max_translation_step_m = declare_parameter<double>(
    "local_correction.max_translation_step_m", 0.05);
  local_correction_config_.max_yaw_step_rad = declare_parameter<double>(
    "local_correction.max_yaw_step_rad", 0.015);
  local_correction_config_.max_dt_sec = declare_parameter<double>(
    "local_correction.max_dt_sec", 0.05);
  diagnostics_period_sec_ = declare_parameter<double>("diagnostics_period_sec", 1.0);
}

void PrecisionGlobalLocalizerNode::validateParameters() const
{
  const std::array<double, 45> finite_parameters{
    sync_history_sec_, sync_max_extrapolation_sec_, sync_max_pending_sec_,
    min_gnss_confidence_, diagnostics_period_sec_, anchor_config_.window_sec,
    anchor_config_.max_sample_gap_sec, anchor_config_.yaw_sample_min_interval_sec,
    anchor_config_.min_local_baseline_m, anchor_config_.min_map_baseline_m,
    anchor_config_.inlier_gate_m, anchor_config_.max_rms_m,
    anchor_config_.max_yaw_stddev_rad, anchor_config_.max_committed_yaw_innovation_rad,
    anchor_config_.activation_max_yaw_candidate_delta_rad,
    anchor_config_.bootstrap_yaw_rad, anchor_config_.hard_outage_sec,
    anchor_config_.min_position_variance_m2, anchor_config_.max_position_variance_m2,
    anchor_config_.max_translation_rate_mps, anchor_config_.max_yaw_rate_radps,
    anchor_config_.max_translation_step_m, anchor_config_.max_yaw_step_rad,
    anchor_config_.max_correction_dt_sec, anchor_config_.unobservable_yaw_variance_rad2,
    anchor_config_.min_yaw_variance_rad2, anchor_config_.outage_xy_variance_rate_m2ps,
    anchor_config_.outage_yaw_variance_rate_rad2ps,
    fusion_anchor_config_.candidate_min_interval_sec,
    fusion_anchor_config_.candidate_max_gap_sec,
    fusion_anchor_config_.stable_max_base_translation_m,
    fusion_anchor_config_.stable_max_yaw_rad,
    fusion_anchor_config_.tracking_max_base_translation_m,
    fusion_anchor_config_.tracking_max_yaw_rad,
    fusion_anchor_config_.max_translation_rate_mps,
    fusion_anchor_config_.max_yaw_rate_radps,
    fusion_anchor_config_.max_translation_step_m,
    fusion_anchor_config_.max_yaw_step_rad,
    fusion_anchor_config_.max_step_dt_sec,
    fusion_health_max_age_sec_, fusion_health_max_future_skew_sec_,
    fusion_sync_max_interpolation_gap_sec_, fusion_sync_max_candidate_age_sec_,
    existing_global_max_age_sec_,
    existing_global_max_future_skew_sec_};
  const std::array<double, 5> finite_local_parameters{
    local_correction_config_.max_translation_rate_mps,
    local_correction_config_.max_yaw_rate_radps,
    local_correction_config_.max_translation_step_m,
    local_correction_config_.max_yaw_step_rad,
    local_correction_config_.max_dt_sec};
  const bool nonfinite = std::any_of(
    finite_parameters.begin(), finite_parameters.end(),
    [](double value) {return !std::isfinite(value);}) ||
    std::any_of(
    finite_local_parameters.begin(), finite_local_parameters.end(),
    [](double value) {return !std::isfinite(value);});
  const bool invalid_topics = raw_odom_topic_.empty() || submap_scan_topic_.empty() ||
    submap_correction_topic_.empty() || gnss_input_topic_.empty() ||
    existing_global_odom_topic_.empty() || fusion_diagnostics_topic_.empty() ||
    fusion_diagnostic_status_name_.empty() ||
    precision_local_odom_topic_.empty() || precision_global_odom_topic_.empty() ||
    precision_global_pose_topic_.empty() || precision_frame_.empty() || map_frame_.empty() ||
    base_frame_.empty();
  const bool invalid_sync = sync_history_sec_ <= 0.0 ||
    sync_max_extrapolation_sec_ < 0.0 || sync_max_pending_sec_ <= 0.0;
  const bool invalid_alignment = anchor_config_.window_sec <= 0.0 ||
    anchor_config_.max_sample_gap_sec <= 0.0 ||
    anchor_config_.yaw_sample_min_interval_sec <= 0.0 ||
    anchor_config_.max_yaw_samples < anchor_config_.min_yaw_samples ||
    anchor_config_.min_yaw_samples < 3U || anchor_config_.min_local_baseline_m <= 0.0 ||
    anchor_config_.min_map_baseline_m <= 0.0 || anchor_config_.inlier_gate_m <= 0.0 ||
    anchor_config_.max_rms_m <= 0.0 || anchor_config_.max_yaw_stddev_rad <= 0.0 ||
    anchor_config_.hard_outage_sec <= 0.0 ||
    anchor_config_.min_position_variance_m2 <= 0.0 ||
    anchor_config_.max_position_variance_m2 < anchor_config_.min_position_variance_m2;
  const bool invalid_limits = anchor_config_.max_translation_rate_mps <= 0.0 ||
    anchor_config_.max_yaw_rate_radps <= 0.0 || anchor_config_.max_translation_step_m <= 0.0 ||
    anchor_config_.max_yaw_step_rad <= 0.0 || anchor_config_.max_correction_dt_sec <= 0.0 ||
    local_correction_config_.max_translation_rate_mps <= 0.0 ||
    local_correction_config_.max_yaw_rate_radps <= 0.0 ||
    local_correction_config_.max_translation_step_m <= 0.0 ||
    local_correction_config_.max_yaw_step_rad <= 0.0 ||
    local_correction_config_.max_dt_sec <= 0.0 || diagnostics_period_sec_ <= 0.0;
  const bool invalid_fusion_anchor =
    fusion_anchor_config_.stable_candidate_count < 3U ||
    fusion_anchor_config_.stable_candidate_count > 20U ||
    fusion_anchor_config_.candidate_min_interval_sec <= 0.0 ||
    fusion_anchor_config_.candidate_max_gap_sec <=
    fusion_anchor_config_.candidate_min_interval_sec ||
    fusion_anchor_config_.stable_max_base_translation_m <= 0.0 ||
    fusion_anchor_config_.stable_max_yaw_rad <= 0.0 ||
    fusion_anchor_config_.stable_max_yaw_rad > kPi ||
    fusion_anchor_config_.tracking_max_base_translation_m <= 0.0 ||
    fusion_anchor_config_.tracking_max_yaw_rad <= 0.0 ||
    fusion_anchor_config_.tracking_max_yaw_rad > kPi ||
    fusion_anchor_config_.max_translation_rate_mps <= 0.0 ||
    fusion_anchor_config_.max_yaw_rate_radps <= 0.0 ||
    fusion_anchor_config_.max_translation_step_m <= 0.0 ||
    fusion_anchor_config_.max_yaw_step_rad <= 0.0 ||
    fusion_anchor_config_.max_step_dt_sec <= 0.0 ||
    fusion_health_max_age_sec_ <= 0.0 ||
    fusion_health_max_future_skew_sec_ < 0.0 ||
    fusion_sync_max_interpolation_gap_sec_ <= 0.0 ||
    fusion_sync_max_candidate_age_sec_ <= 0.0 ||
    existing_global_max_age_sec_ <= 0.0 ||
    existing_global_max_future_skew_sec_ < 0.0;
  if (nonfinite || invalid_topics || invalid_sync || invalid_alignment || invalid_limits ||
    invalid_fusion_anchor ||
    anchor_config_.max_yaw_samples > 100U ||
    anchor_config_.max_committed_yaw_innovation_rad <= 0.0 ||
    anchor_config_.activation_min_stable_yaw_candidates < 3U ||
    anchor_config_.activation_min_stable_yaw_candidates > 20U ||
    anchor_config_.activation_max_yaw_candidate_delta_rad <= 0.0 ||
    anchor_config_.activation_max_yaw_candidate_delta_rad > 3.14159265358979323846 ||
    anchor_config_.unobservable_yaw_variance_rad2 <= 0.0 ||
    anchor_config_.min_yaw_variance_rad2 <= 0.0 ||
    anchor_config_.outage_xy_variance_rate_m2ps < 0.0 ||
    anchor_config_.outage_yaw_variance_rate_rad2ps < 0.0 ||
    min_gnss_confidence_ < 0.0 || min_gnss_confidence_ > 1.0)
  {
    throw std::invalid_argument("invalid precision global localizer parameter");
  }
}

void PrecisionGlobalLocalizerNode::noteStateTransition(
  AnchorState before, AnchorState after)
{
  if (before == after) {
    return;
  }
  if (after == AnchorState::HOLD_SOFT_GAP) {
    ++counters_.soft_gap_count;
  }
  if (after == AnchorState::OUTAGE) {
    ++counters_.outage_count;
  }
  const bool was_hold = before == AnchorState::HOLD_SOFT_GAP ||
    before == AnchorState::OUTAGE;
  const bool now_tracking = after == AnchorState::TRACKING_XY_ONLY ||
    after == AnchorState::TRACKING_SE2;
  if (was_hold && now_tracking) {
    ++counters_.recovery_count;
  }
}

void PrecisionGlobalLocalizerNode::onSubmapScan(SubmapScan::ConstSharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++counters_.scan_received;
  Pose2 raw_pose;
  QuaternionInfo raw_pose_quaternion;
  const ScanKey key = keyFrom(*message);
  if (key.session == 0U || key.generation == 0U || key.sequence == 0U ||
    !validStamp(message->header.stamp) || message->header.frame_id.empty() ||
    !pose2From(message->raw_pose.pose, raw_pose, &raw_pose_quaternion) ||
    std::fabs(raw_pose_quaternion.original_norm - 1.0) > 1.0e-3)
  {
    ++counters_.scan_rejected;
    return;
  }
  const bool had_epoch = compositor_->hasEpoch();
  const OdomEpoch previous_epoch = compositor_->epoch();
  if (had_epoch && previous_epoch.session == message->odom_session_id &&
    !active_raw_frame_.empty() && message->header.frame_id != active_raw_frame_)
  {
    ++counters_.scan_rejected;
    return;
  }
  const EpochResult epoch_result = compositor_->observeEpoch(
    {message->odom_session_id, message->odom_generation});
  if (epoch_result == EpochResult::REJECTED_RETIRED) {
    ++counters_.scan_rejected;
    return;
  }
  if (epoch_result == EpochResult::RESET) {
    ++counters_.epoch_resets;
    if (had_epoch && previous_epoch.session != message->odom_session_id) {
      // A process/session change has no coordinate relationship to the old raw
      // frame. Reset every global/history state that could otherwise bridge
      // the two sessions. Same-session generation changes intentionally retain
      // continuity and do not enter this branch.
      std::deque<SubmapCorrection::ConstSharedPtr> new_session_corrections;
      for (const auto & pending : pending_corrections_) {
        if (pending->odom_session_id == message->odom_session_id) {
          new_session_corrections.push_back(pending);
        }
      }
      gnss_anchor_->reset("new_odom_session_reset");
      fusion_anchor_->reset("new_odom_session_reset");
      local_history_.clear();
      pending_fusion_locals_.clear();
      existing_global_history_.clear();
      fusion_health_ = FusionHealthSnapshot{};
      fusion_rearm_state_.required = true;
      fusion_rearm_state_.saw_unhealthy = false;
      fusion_rearm_state_.rearmed = false;
      fusion_rearm_reset_stamp_sec_ = stampSeconds(message->header.stamp);
      pending_gnss_.clear();
      pending_corrections_.clear();
      pending_corrections_.swap(new_session_corrections);
      scan_cache_.clear();
      scan_order_.clear();
      accepted_correction_keys_.clear();
      accepted_key_order_.clear();
      position_fused_ = false;
      has_latest_local_stamp_ = false;
      latest_local_stamp_sec_ = 0.0;
      last_usable_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_q4_usable_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_sync_to_current_sec_ = std::numeric_limits<double>::quiet_NaN();
      fusion_health_age_sec_ = std::numeric_limits<double>::quiet_NaN();
      existing_global_age_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_valid_fusion_sync_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
      valid_fusion_sync_age_sec_ = std::numeric_limits<double>::quiet_NaN();
      has_fusion_anchor_local_covariance_ref_ = false;
      fusion_anchor_local_covariance_ref_.setZero();
      frozen_anchor_residual_covariance_.setZero();
      last_fusion_sync_error_sec_ = std::numeric_limits<double>::quiet_NaN();
      last_fusion_sync_mode_ = "none";
      last_fusion_sync_reason_ = "odom_session_reset";
      current_anchor_lag_translation_m_ = 0.0;
      current_anchor_lag_yaw_rad_ = 0.0;
      max_anchor_lag_translation_m_ = 0.0;
      max_anchor_lag_yaw_rad_ = 0.0;
      current_local_lag_translation_m_ = 0.0;
      current_local_lag_yaw_rad_ = 0.0;
      max_local_lag_translation_m_ = 0.0;
      max_local_lag_yaw_rad_ = 0.0;
      counters_.correction_pending = 0U;
      counters_.correction_pending = pending_corrections_.size();
      counters_.gnss_pending = 0U;
      last_gnss_reason_ = "odom_session_reset";
      last_correction_reason_ = "odom_session_reset";
      active_raw_frame_ = message->header.frame_id;
      ++counters_.odom_session_resets;
    }
  }
  if (!had_epoch || active_raw_frame_.empty()) {
    active_raw_frame_ = message->header.frame_id;
  }
  if (scan_cache_.count(key) != 0U) {
    ++counters_.scan_rejected;
    return;
  }
  ScanRecord record;
  record.key = key;
  record.raw_frame = message->header.frame_id;
  record.raw_pose = raw_pose;
  record.raw_z = message->raw_pose.pose.position.z;
  scan_cache_.emplace(key, record);
  scan_order_.push_back(key);
  while (scan_order_.size() > kScanCacheLimit) {
    scan_cache_.erase(scan_order_.front());
    scan_order_.pop_front();
  }
  processPendingCorrectionsLocked();
}

bool PrecisionGlobalLocalizerNode::processCorrectionLocked(
  const SubmapCorrection & message, std::string & reason)
{
  const ScanKey key = keyFrom(message);
  if (key.session == 0U || key.generation == 0U || key.sequence == 0U ||
    message.matcher_session_id == 0U || message.submap_generation == 0U ||
    message.correction_id == 0U || !validStamp(message.header.stamp))
  {
    reason = "zero_or_invalid_key";
    return false;
  }
  if (accepted_correction_keys_.count(key) != 0U) {
    reason = "duplicate_exact_key";
    return false;
  }
  const auto scan_iterator = scan_cache_.find(key);
  if (scan_iterator == scan_cache_.end()) {
    reason = "unknown_exact_scan_key";
    return false;
  }
  const ScanRecord & scan = scan_iterator->second;
  if (message.header.frame_id != scan.raw_frame || !message.use_yaw ||
    message.precision_frame_id != precision_frame_ ||
    !std::isfinite(message.fitness) || !std::isfinite(message.inlier_ratio) ||
    !std::isfinite(message.innovation_translation_m) ||
    !std::isfinite(message.innovation_x_m) ||
    !std::isfinite(message.innovation_y_m) ||
    !std::isfinite(message.innovation_yaw_rad) || message.consistency_count == 0U)
  {
    reason = "frame_or_quality_contract";
    return false;
  }

  bool transform_valid = false;
  const Pose2 precision_from_raw = transform2From(message.precision_from_raw, transform_valid);
  Pose2 corrected_pose;
  QuaternionInfo corrected_quaternion;
  if (!transform_valid ||
    !pose2From(message.corrected_pose.pose, corrected_pose, &corrected_quaternion) ||
    std::fabs(corrected_quaternion.original_norm - 1.0) > 1.0e-3)
  {
    reason = "invalid_transform_or_pose";
    return false;
  }
  const Pose2 expected = compose(precision_from_raw, scan.raw_pose);
  if (poseTranslationDistance(expected, corrected_pose) > kPoseContractTolerance ||
    std::fabs(wrapAngle(expected.yaw - corrected_pose.yaw)) > kPoseContractTolerance ||
    std::fabs(message.corrected_pose.pose.position.z - scan.raw_z) > kPoseContractTolerance)
  {
    reason = "corrected_pose_transform_mismatch";
    return false;
  }

  const auto & pose_covariance = message.corrected_pose.covariance;
  Eigen::Matrix3d correction_covariance;
  correction_covariance <<
    pose_covariance[0], pose_covariance[1], pose_covariance[5],
    pose_covariance[6], pose_covariance[7], pose_covariance[11],
    pose_covariance[30], pose_covariance[31], pose_covariance[35];
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> correction_covariance_solver(
    0.5 * (correction_covariance + correction_covariance.transpose()));
  if (!correction_covariance.allFinite() ||
    (correction_covariance - correction_covariance.transpose()).norm() > 1.0e-6 ||
    correction_covariance_solver.info() != Eigen::Success ||
    correction_covariance_solver.eigenvalues().minCoeff() < -1.0e-9)
  {
    reason = "invalid_correction_covariance";
    return false;
  }
  LocalCorrectionObservation observation;
  observation.epoch = {message.odom_session_id, message.odom_generation};
  observation.sequence = message.sequence;
  observation.matcher_session = message.matcher_session_id;
  observation.correction_id = message.correction_id;
  observation.precision_frame = message.precision_frame_id;
  observation.precision_from_raw = precision_from_raw;
  observation.covariance = correction_covariance;
  const CorrectionResult result = compositor_->acceptCorrection(observation);
  if (result != CorrectionResult::ACCEPTED &&
    result != CorrectionResult::ACCEPTED_REBASED_SESSION)
  {
    reason = "compositor_rejected_" + std::to_string(static_cast<int>(result));
    return false;
  }
  if (result == CorrectionResult::ACCEPTED_REBASED_SESSION) {
    ++counters_.session_rebases;
  }
  accepted_correction_keys_.insert(key);
  accepted_key_order_.push_back(key);
  while (accepted_key_order_.size() > kAcceptedKeyLimit) {
    accepted_correction_keys_.erase(accepted_key_order_.front());
    accepted_key_order_.pop_front();
  }
  reason = result == CorrectionResult::ACCEPTED_REBASED_SESSION ?
    "accepted_session_rebase" : "accepted";
  return true;
}

void PrecisionGlobalLocalizerNode::processPendingCorrectionsLocked()
{
  std::deque<SubmapCorrection::ConstSharedPtr> remaining;
  for (const auto & message : pending_corrections_) {
    std::string reason;
    if (scan_cache_.count(keyFrom(*message)) == 0U) {
      if (has_latest_local_stamp_ && validStamp(message->header.stamp) &&
        latest_local_stamp_sec_ - stampSeconds(message->header.stamp) > sync_history_sec_)
      {
        ++counters_.correction_rejected;
        ++counters_.correction_expired;
        last_correction_reason_ = "exact_scan_key_expired";
        continue;
      }
      remaining.push_back(message);
      continue;
    }
    if (processCorrectionLocked(*message, reason)) {
      ++counters_.correction_accepted;
    } else {
      ++counters_.correction_rejected;
    }
    last_correction_reason_ = reason;
  }
  pending_corrections_.swap(remaining);
  counters_.correction_pending = pending_corrections_.size();
}

void PrecisionGlobalLocalizerNode::onSubmapCorrection(
  SubmapCorrection::ConstSharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++counters_.correction_received;
  const ScanKey incoming_key = keyFrom(*message);
  if (incoming_key.session == 0U || incoming_key.generation == 0U ||
    incoming_key.sequence == 0U || message->matcher_session_id == 0U ||
    message->submap_generation == 0U || message->correction_id == 0U ||
    !validStamp(message->header.stamp))
  {
    ++counters_.correction_rejected;
    last_correction_reason_ = "zero_or_invalid_key";
    return;
  }
  std::string reason;
  if (scan_cache_.count(incoming_key) == 0U) {
    pending_corrections_.push_back(message);
    while (pending_corrections_.size() > kPendingCorrectionLimit) {
      pending_corrections_.pop_front();
      ++counters_.correction_rejected;
      ++counters_.correction_expired;
    }
    counters_.correction_pending = pending_corrections_.size();
    last_correction_reason_ = "waiting_exact_scan_key";
    return;
  }
  if (processCorrectionLocked(*message, reason)) {
    ++counters_.correction_accepted;
  } else {
    ++counters_.correction_rejected;
  }
  last_correction_reason_ = reason;
}

nav_msgs::msg::Odometry PrecisionGlobalLocalizerNode::makePrecisionLocalLocked(
  const nav_msgs::msg::Odometry & raw,
  const LocalCompositionResult & composition,
  const QuaternionInfo & raw_quaternion)
{
  nav_msgs::msg::Odometry output = raw;
  output.header.frame_id = precision_frame_;
  output.child_frame_id = raw.child_frame_id.empty() ? base_frame_ : raw.child_frame_id;
  output.pose.pose.position.x = composition.precision_pose.x;
  output.pose.pose.position.y = composition.precision_pose.y;
  output.pose.pose.position.z = raw.pose.pose.position.z;
  tf2::Quaternion orientation =
    yawQuaternion(composition.applied_precision_from_raw.yaw) * raw_quaternion.quaternion;
  orientation.normalize();
  output.pose.pose.orientation = quaternionMessage(orientation);

  const Eigen::Matrix3d raw_covariance = covariance3From(raw.pose.covariance);
  Eigen::Matrix3d raw_jacobian = Eigen::Matrix3d::Identity();
  raw_jacobian.block<2, 2>(0, 0) =
    rotation2(composition.applied_precision_from_raw.yaw);
  // corrected_pose covariance is an observation uncertainty at base_link, not
  // a transform covariance about the remote odom origin. Add it directly in
  // pose space; do not apply a second yaw lever.
  const Pose2 raw_pose{
    raw.pose.pose.position.x, raw.pose.pose.position.y, raw_quaternion.yaw};
  const Pose2 desired_pose = compose(compositor_->targetCorrection(), raw_pose);
  Eigen::Vector3d correction_lag;
  correction_lag <<
    desired_pose.x - composition.precision_pose.x,
    desired_pose.y - composition.precision_pose.y,
    wrapAngle(desired_pose.yaw - composition.precision_pose.yaw);
  current_local_lag_translation_m_ = correction_lag.head<2>().norm();
  current_local_lag_yaw_rad_ = std::fabs(correction_lag.z());
  max_local_lag_translation_m_ = std::max(
    max_local_lag_translation_m_, current_local_lag_translation_m_);
  max_local_lag_yaw_rad_ = std::max(max_local_lag_yaw_rad_, current_local_lag_yaw_rad_);
  const Eigen::Matrix3d local_covariance = projectCovariancePsd(
    raw_jacobian * raw_covariance * raw_jacobian.transpose() +
    composition.correction_covariance + correction_lag * correction_lag.transpose());
  writeCovariance3(local_covariance, output.pose.covariance);
  return output;
}

void PrecisionGlobalLocalizerNode::publishGlobalLocked(
  const nav_msgs::msg::Odometry & local, const Pose2 & local_pose)
{
  if (!fusion_anchor_->globalOutputReady()) {
    ++counters_.global_suppressed_not_ready;
    return;
  }
  const double local_stamp_sec = stampSeconds(local.header.stamp);
  if (!std::isfinite(fusion_anchor_->activationStamp()) ||
    local_stamp_sec <= fusion_anchor_->activationStamp() + 1.0e-9)
  {
    // Activation can be completed while processing G/P for this raw callback.
    // Start the authoritative stream only at the next unique raw stamp.
    ++counters_.global_suppressed_activation_watermark;
    return;
  }
  const Pose2 global_pose = compose(fusion_anchor_->appliedAnchor(), local_pose);
  nav_msgs::msg::Odometry output = local;
  output.header.frame_id = map_frame_;
  output.child_frame_id = local.child_frame_id.empty() ? base_frame_ : local.child_frame_id;
  output.pose.pose.position.x = global_pose.x;
  output.pose.pose.position.y = global_pose.y;
  output.pose.pose.position.z = local.pose.pose.position.z;
  const auto local_quaternion = quaternionInfo(local.pose.pose.orientation);
  if (!local_quaternion.valid) {
    return;
  }
  tf2::Quaternion orientation =
    yawQuaternion(fusion_anchor_->appliedAnchor().yaw) * local_quaternion.quaternion;
  orientation.normalize();
  output.pose.pose.orientation = quaternionMessage(orientation);
  // The anchor covariance is stored as uncertainty of the synchronized
  // existing-global base observation, not as an independent remote-origin
  // transform covariance. While strict fusion is healthy this preserves the
  // semantics of G instead of double-counting the shared raw odometry in G/P.
  Eigen::Matrix3d global_covariance = fusion_anchor_->anchorCovariance();
  const Eigen::Matrix3d current_local_covariance = covariance3From(local.pose.covariance);
  if (!fusion_anchor_->fusionHealthy() ||
    fusion_anchor_->state() != FusionAnchorState::TRACKING)
  {
    Eigen::Matrix3d local_jacobian = Eigen::Matrix3d::Identity();
    local_jacobian.block<2, 2>(0, 0) = rotation2(fusion_anchor_->appliedAnchor().yaw);
    global_covariance += local_jacobian * current_local_covariance *
      local_jacobian.transpose();
  } else if (has_fusion_anchor_local_covariance_ref_) {
    // Between synchronized healthy candidates, retain G's covariance and add
    // only non-negative local covariance growth since that candidate.
    Eigen::Matrix3d covariance_growth = Eigen::Matrix3d::Zero();
    for (int index = 0; index < 3; ++index) {
      covariance_growth(index, index) = std::max(
        0.0,
        current_local_covariance(index, index) -
        fusion_anchor_local_covariance_ref_(index, index));
    }
    Eigen::Matrix3d local_jacobian = Eigen::Matrix3d::Identity();
    local_jacobian.block<2, 2>(0, 0) = rotation2(fusion_anchor_->appliedAnchor().yaw);
    global_covariance += local_jacobian * covariance_growth * local_jacobian.transpose();
  }
  const Pose2 target_global_pose = compose(fusion_anchor_->targetAnchor(), local_pose);
  Eigen::Vector3d lag;
  lag <<
    target_global_pose.x - global_pose.x,
    target_global_pose.y - global_pose.y,
    wrapAngle(target_global_pose.yaw - global_pose.yaw);
  current_anchor_lag_translation_m_ = lag.head<2>().norm();
  current_anchor_lag_yaw_rad_ = std::fabs(lag.z());
  max_anchor_lag_translation_m_ = std::max(
    max_anchor_lag_translation_m_, current_anchor_lag_translation_m_);
  max_anchor_lag_yaw_rad_ = std::max(max_anchor_lag_yaw_rad_, current_anchor_lag_yaw_rad_);
  // The follower state is known, but while it is intentionally behind a new
  // existing-fusion target its deterministic residual is still an accuracy risk. Report
  // that residual as a conservative rank-one covariance contribution.
  global_covariance = projectCovariancePsd(
    global_covariance + lag * lag.transpose() + frozen_anchor_residual_covariance_);
  writeCovariance3(global_covariance, output.pose.covariance);
  global_publisher_->publish(output);

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header = output.header;
  pose.pose = output.pose;
  pose_publisher_->publish(pose);
  ++counters_.global_published;
}

void PrecisionGlobalLocalizerNode::onRawOdom(
  nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++counters_.raw_received;
  Pose2 raw_pose;
  QuaternionInfo raw_quaternion;
  if (!compositor_->hasEpoch() || !validStamp(message->header.stamp) ||
    active_raw_frame_.empty() || message->header.frame_id != active_raw_frame_ ||
    message->child_frame_id != base_frame_ ||
    !pose2From(message->pose.pose, raw_pose, &raw_quaternion))
  {
    ++counters_.raw_invalid;
    return;
  }
  const double stamp_sec = stampSeconds(message->header.stamp);
  if (has_latest_local_stamp_ && stamp_sec < latest_local_stamp_sec_ - 1.0e-9) {
    ++counters_.raw_nonmonotonic;
    return;
  }
  const bool duplicate_stamp = has_latest_local_stamp_ &&
    std::fabs(stamp_sec - latest_local_stamp_sec_) <= 1.0e-9;
  if (duplicate_stamp) {
    ++counters_.raw_duplicate_stamp;
  }
  const auto composition = compositor_->composeRaw(raw_pose, stamp_sec);
  if (!composition.valid) {
    ++counters_.raw_invalid;
    return;
  }
  const nav_msgs::msg::Odometry local = makePrecisionLocalLocked(
    *message, composition, raw_quaternion);
  local_publisher_->publish(local);
  ++counters_.local_published;

  Pose2 local_pose;
  if (!pose2From(local.pose.pose, local_pose)) {
    ++counters_.raw_invalid;
    return;
  }
  if (!has_latest_local_stamp_ || stamp_sec > latest_local_stamp_sec_) {
    LocalRecord record;
    record.stamp_sec = stamp_sec;
    record.pose = local_pose;
    record.z = local.pose.pose.position.z;
    record.covariance = covariance3From(local.pose.covariance);
    local_history_.push_back(record);
    pending_fusion_locals_.push_back(record);
    latest_local_stamp_sec_ = stamp_sec;
    has_latest_local_stamp_ = true;
    while (!local_history_.empty() &&
      stamp_sec - local_history_.front().stamp_sec > sync_history_sec_)
    {
      local_history_.pop_front();
    }
    while (local_history_.size() > 5000U) {
      local_history_.pop_front();
    }
    while (pending_fusion_locals_.size() > 5000U) {
      pending_fusion_locals_.pop_front();
      ++counters_.fusion_sync_rejected;
      last_fusion_sync_reason_ = "pending_local_overflow";
    }
  }

  updateFusionHealthLocked(stamp_sec);
  processFusionCandidatesLocked();
  if (!duplicate_stamp) {
    (void)fusion_anchor_->advance(stamp_sec, local_pose);
  }
  if (gnss_position_diagnostics_enabled_) {
    const AnchorState before_time = gnss_anchor_->state();
    gnss_anchor_->updateTime(stamp_sec);
    noteStateTransition(before_time, gnss_anchor_->state());
    processPendingGnssLocked();
  }
  publishGlobalLocked(local, local_pose);
}

InterpolatedLocal PrecisionGlobalLocalizerNode::interpolateLocalLocked(
  double stamp_sec) const
{
  InterpolatedLocal result;
  if (local_history_.empty() || !std::isfinite(stamp_sec)) {
    return result;
  }
  const auto upper = std::lower_bound(
    local_history_.begin(), local_history_.end(), stamp_sec,
    [](const LocalRecord & record, double stamp) {return record.stamp_sec < stamp;});
  if (upper == local_history_.begin()) {
    if (std::fabs(upper->stamp_sec - stamp_sec) > sync_max_extrapolation_sec_) {
      return result;
    }
    result.valid = true;
    result.pose = upper->pose;
    result.z = upper->z;
    result.covariance = upper->covariance;
    return result;
  }
  if (upper == local_history_.end()) {
    // Do not extrapolate a past pose from vehicle velocity. The GNSS message is
    // queued until a local sample brackets its observation stamp.
    return result;
  }
  if (std::fabs(upper->stamp_sec - stamp_sec) <= 1.0e-9) {
    result.valid = true;
    result.pose = upper->pose;
    result.z = upper->z;
    result.covariance = upper->covariance;
    return result;
  }
  const auto lower = upper - 1;
  const double interval = upper->stamp_sec - lower->stamp_sec;
  if (!(interval > 0.0) || interval > 2.0 * sync_max_extrapolation_sec_) {
    return result;
  }
  const double ratio = std::clamp((stamp_sec - lower->stamp_sec) / interval, 0.0, 1.0);
  result.valid = true;
  result.pose.x = lower->pose.x + ratio * (upper->pose.x - lower->pose.x);
  result.pose.y = lower->pose.y + ratio * (upper->pose.y - lower->pose.y);
  result.pose.yaw = wrapAngle(
    lower->pose.yaw + ratio * wrapAngle(upper->pose.yaw - lower->pose.yaw));
  result.z = lower->z + ratio * (upper->z - lower->z);
  result.covariance = projectCovariancePsd(
    (1.0 - ratio) * lower->covariance + ratio * upper->covariance);
  return result;
}

InterpolatedGlobal PrecisionGlobalLocalizerNode::interpolateExistingGlobalLocked(
  double stamp_sec) const
{
  InterpolatedGlobal result;
  if (existing_global_history_.empty() || !std::isfinite(stamp_sec)) {
    return result;
  }
  const auto upper = std::lower_bound(
    existing_global_history_.begin(), existing_global_history_.end(), stamp_sec,
    [](const ExistingGlobalRecord & record, double stamp) {
      return record.stamp_sec < stamp;
    });
  if (upper != existing_global_history_.end() &&
    std::fabs(upper->stamp_sec - stamp_sec) <= 1.0e-9)
  {
    result.valid = true;
    result.pose = upper->pose;
    result.z = upper->z;
    result.covariance = upper->covariance;
    result.sync_error_sec = 0.0;
    result.mode = "exact";
    return result;
  }
  // No causal extrapolation: retain the local sample until a future global
  // sample brackets it, then accept only a small, explicitly bounded interval.
  if (upper == existing_global_history_.begin() || upper == existing_global_history_.end()) {
    return result;
  }
  const auto lower = upper - 1;
  const double interval = upper->stamp_sec - lower->stamp_sec;
  if (!(interval > 0.0) || interval > fusion_sync_max_interpolation_gap_sec_) {
    return result;
  }
  const double ratio = std::clamp((stamp_sec - lower->stamp_sec) / interval, 0.0, 1.0);
  result.valid = true;
  result.pose.x = lower->pose.x + ratio * (upper->pose.x - lower->pose.x);
  result.pose.y = lower->pose.y + ratio * (upper->pose.y - lower->pose.y);
  result.pose.yaw = wrapAngle(
    lower->pose.yaw + ratio * wrapAngle(upper->pose.yaw - lower->pose.yaw));
  result.z = lower->z + ratio * (upper->z - lower->z);
  result.covariance = projectCovariancePsd(
    (1.0 - ratio) * lower->covariance + ratio * upper->covariance);
  result.sync_error_sec = std::min(
    stamp_sec - lower->stamp_sec, upper->stamp_sec - stamp_sec);
  result.mode = "interpolated";
  return result;
}

bool PrecisionGlobalLocalizerNode::strictFusionHealthLocked(
  double reference_stamp_sec, std::string & reason)
{
  if (!fusion_health_.received) {
    fusion_health_age_sec_ = std::numeric_limits<double>::quiet_NaN();
    reason = "fusion_diagnostics_unavailable";
    return false;
  }
  const double now_sec = get_clock()->now().seconds();
  const FusionHealthEvaluation evaluation = evaluateExistingFusionHealthFreshness(
    fusion_health_.strict_fields_healthy, fusion_health_.reason,
    fusion_health_.stamp_sec, reference_stamp_sec,
    std::isfinite(now_sec) && now_sec > 0.0 ? now_sec : reference_stamp_sec,
    fusion_health_max_age_sec_, fusion_health_max_future_skew_sec_);
  fusion_health_age_sec_ = evaluation.age_sec;
  reason = evaluation.reason;
  if (!evaluation.healthy) {
    return false;
  }
  if (existing_global_history_.empty()) {
    existing_global_age_sec_ = std::numeric_limits<double>::quiet_NaN();
    reason = "existing_global_unavailable";
    return false;
  }
  const double latest_global_stamp = existing_global_history_.back().stamp_sec;
  const auto global_upper = std::lower_bound(
    existing_global_history_.begin(), existing_global_history_.end(), reference_stamp_sec,
    [](const ExistingGlobalRecord & record, double stamp) {
      return record.stamp_sec < stamp;
    });
  double sample_global_stamp = std::numeric_limits<double>::quiet_NaN();
  if (global_upper != existing_global_history_.end() &&
    global_upper->stamp_sec - reference_stamp_sec <= existing_global_max_future_skew_sec_)
  {
    sample_global_stamp = global_upper->stamp_sec;
  } else if (global_upper != existing_global_history_.begin()) {
    sample_global_stamp = (global_upper - 1)->stamp_sec;
  }
  if (!std::isfinite(sample_global_stamp)) {
    reason = "existing_global_sample_unavailable";
    return false;
  }
  const double global_sample_age_sec = reference_stamp_sec - sample_global_stamp;
  const double effective_now_sec = std::isfinite(now_sec) && now_sec > 0.0 ?
    now_sec : reference_stamp_sec;
  existing_global_age_sec_ = effective_now_sec - latest_global_stamp;
  if (global_sample_age_sec < -existing_global_max_future_skew_sec_ ||
    global_sample_age_sec > existing_global_max_age_sec_)
  {
    reason = "existing_global_sample_age_gate";
    return false;
  }
  if (existing_global_age_sec_ < -existing_global_max_future_skew_sec_ ||
    existing_global_age_sec_ > existing_global_max_age_sec_)
  {
    reason = "existing_global_stale";
    return false;
  }
  if (fusion_anchor_->globalOutputReady()) {
    if (!std::isfinite(last_valid_fusion_sync_stamp_sec_)) {
      valid_fusion_sync_age_sec_ = std::numeric_limits<double>::quiet_NaN();
      reason = "valid_fusion_sync_unavailable";
      return false;
    }
    const double sync_sample_age_sec =
      reference_stamp_sec - last_valid_fusion_sync_stamp_sec_;
    valid_fusion_sync_age_sec_ =
      effective_now_sec - last_valid_fusion_sync_stamp_sec_;
    if (sync_sample_age_sec < -existing_global_max_future_skew_sec_ ||
      sync_sample_age_sec > fusion_sync_max_candidate_age_sec_ ||
      valid_fusion_sync_age_sec_ < -existing_global_max_future_skew_sec_ ||
      valid_fusion_sync_age_sec_ > fusion_sync_max_candidate_age_sec_)
    {
      reason = "valid_fusion_sync_stale";
      return false;
    }
  }
  return true;
}

void PrecisionGlobalLocalizerNode::updateFusionHealthLocked(double reference_stamp_sec)
{
  std::string reason;
  FusionHealthEvaluation evaluation;
  evaluation.healthy = strictFusionHealthLocked(reference_stamp_sec, reason);
  evaluation.reason = reason;
  evaluation.age_sec = fusion_health_age_sec_;
  const bool diagnostic_is_post_reset =
    !fusion_rearm_state_.required ||
    (std::isfinite(fusion_health_.stamp_sec) &&
    std::isfinite(fusion_rearm_reset_stamp_sec_) &&
    fusion_health_.stamp_sec > fusion_rearm_reset_stamp_sec_ + 1.0e-9);
  if (fusion_rearm_state_.required && !diagnostic_is_post_reset) {
    evaluation.healthy = false;
    evaluation.reason = "fusion_rearm_pre_reset_diagnostic";
  }
  const bool qualifying_unhealthy =
    diagnostic_is_post_reset && fusion_health_.received &&
    !fusion_health_.strict_fields_healthy && reason == fusion_health_.reason;
  evaluation = applyExistingFusionRearmGate(
    evaluation, qualifying_unhealthy, fusion_rearm_state_);
  if (!evaluation.healthy) {
    forceFusionUnhealthyLocked(reference_stamp_sec, evaluation.reason);
    return;
  }
  fusion_anchor_->setFusionHealthy(true, reference_stamp_sec, evaluation.reason);
}

void PrecisionGlobalLocalizerNode::forceFusionUnhealthyLocked(
  double reference_stamp_sec, const std::string & reason)
{
  if (fusion_anchor_->fusionHealthy() && fusion_anchor_->globalOutputReady() &&
    !local_history_.empty())
  {
    const Pose2 target_base = compose(
      fusion_anchor_->targetAnchor(), local_history_.back().pose);
    const Pose2 applied_base = compose(
      fusion_anchor_->appliedAnchor(), local_history_.back().pose);
    Eigen::Vector3d residual;
    residual <<
      target_base.x - applied_base.x,
      target_base.y - applied_base.y,
      wrapAngle(target_base.yaw - applied_base.yaw);
    frozen_anchor_residual_covariance_ = residual * residual.transpose();
  }
  fusion_anchor_->setFusionHealthy(false, reference_stamp_sec, reason);
}

void PrecisionGlobalLocalizerNode::processFusionCandidatesLocked()
{
  while (!pending_fusion_locals_.empty()) {
    if (existing_global_history_.empty()) {
      return;
    }
    const LocalRecord local = pending_fusion_locals_.front();
    if (local.stamp_sec > existing_global_history_.back().stamp_sec + 1.0e-9) {
      return;
    }
    pending_fusion_locals_.pop_front();
    if (local.stamp_sec < existing_global_history_.front().stamp_sec - 1.0e-9) {
      ++counters_.fusion_sync_rejected;
      last_fusion_sync_reason_ = "local_before_global_history";
      last_fusion_sync_mode_ = "none";
      last_fusion_sync_error_sec_ = std::numeric_limits<double>::quiet_NaN();
      continue;
    }

    const InterpolatedGlobal global = interpolateExistingGlobalLocked(local.stamp_sec);
    if (!global.valid) {
      ++counters_.fusion_sync_rejected;
      last_fusion_sync_reason_ = "global_interpolation_gap";
      last_fusion_sync_mode_ = "none";
      last_fusion_sync_error_sec_ = std::numeric_limits<double>::quiet_NaN();
      continue;
    }
    last_valid_fusion_sync_stamp_sec_ = local.stamp_sec;
    updateFusionHealthLocked(local.stamp_sec);
    if (!fusion_anchor_->fusionHealthy()) {
      ++counters_.fusion_sync_rejected;
      last_fusion_sync_reason_ = fusion_anchor_->healthReason();
      last_fusion_sync_mode_ = global.mode;
      last_fusion_sync_error_sec_ = global.sync_error_sec;
      continue;
    }

    FusionAnchorCandidate candidate;
    candidate.stamp_sec = local.stamp_sec;
    candidate.anchor = derivePrecisionAnchor(global.pose, local.pose);
    // The synchronized P is used only in A=G*P^-1. Candidate consistency and
    // every bounded correction are evaluated at the latest base pose, avoiding
    // a visible jump when G arrives after the synchronized observation stamp.
    candidate.local_base = local_history_.empty() ? local.pose : local_history_.back().pose;
    // G and P share raw-odometry errors. Treating them as independent here and
    // then adding P again at publication would double-count that common source.
    // Store the synchronized G base-pose uncertainty; unhealthy/frozen output
    // conservatively adds live P uncertainty in publishGlobalLocked().
    candidate.covariance = projectCovariancePsd(global.covariance);
    const FusionAnchorUpdate update = fusion_anchor_->observeCandidate(candidate);
    if (update.anchor_frozen) {
      Eigen::Vector3d residual;
      residual << update.frozen_residual_x_m, update.frozen_residual_y_m,
        update.frozen_residual_yaw_rad;
      frozen_anchor_residual_covariance_ = residual * residual.transpose();
    }
    if (update.target_updated) {
      fusion_anchor_local_covariance_ref_ = local.covariance;
      has_fusion_anchor_local_covariance_ref_ = true;
      frozen_anchor_residual_covariance_.setZero();
    }
    last_fusion_sync_mode_ = global.mode;
    last_fusion_sync_error_sec_ = global.sync_error_sec;
    last_fusion_sync_reason_ = update.reason;
    if (update.accepted) {
      ++counters_.fusion_sync_accepted;
    } else {
      ++counters_.fusion_sync_rejected;
    }
  }
}

void PrecisionGlobalLocalizerNode::onExistingGlobal(
  nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++counters_.existing_global_received;
  Pose2 pose;
  QuaternionInfo quaternion;
  Eigen::Matrix3d covariance;
  if (!validStamp(message->header.stamp) || message->header.frame_id != map_frame_ ||
    message->child_frame_id != base_frame_ ||
    !pose2From(message->pose.pose, pose, &quaternion) ||
    std::fabs(quaternion.original_norm - 1.0) > 1.0e-3 ||
    !strictCovariance3From(message->pose.covariance, covariance))
  {
    ++counters_.existing_global_rejected;
    last_fusion_sync_reason_ = "invalid_existing_global_contract";
    return;
  }
  const double stamp_sec = stampSeconds(message->header.stamp);
  if (!existing_global_history_.empty() &&
    stamp_sec < existing_global_history_.back().stamp_sec - 1.0e-9)
  {
    ++counters_.existing_global_rejected;
    last_fusion_sync_reason_ = "existing_global_backstep";
    return;
  }
  if (!existing_global_history_.empty() &&
    std::fabs(stamp_sec - existing_global_history_.back().stamp_sec) <= 1.0e-9)
  {
    ++counters_.existing_global_duplicate_stamp;
    return;
  }
  existing_global_history_.push_back({
    stamp_sec, pose, message->pose.pose.position.z, covariance});
  ++counters_.existing_global_accepted;
  while (!existing_global_history_.empty() &&
    stamp_sec - existing_global_history_.front().stamp_sec > sync_history_sec_)
  {
    existing_global_history_.pop_front();
  }
  while (existing_global_history_.size() > 10000U) {
    existing_global_history_.pop_front();
  }
  processFusionCandidatesLocked();
}

void PrecisionGlobalLocalizerNode::onFusionDiagnostics(
  diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const diagnostic_msgs::msg::DiagnosticStatus * selected = nullptr;
  for (const auto & status : message->status) {
    if (status.name == fusion_diagnostic_status_name_) {
      if (selected != nullptr) {
        ++counters_.fusion_diagnostics_rejected;
        fusion_health_ = FusionHealthSnapshot{};
        fusion_health_.reason = "duplicate_fusion_status";
        forceFusionUnhealthyLocked(0.0, fusion_health_.reason);
        return;
      }
      selected = &status;
    }
  }
  if (selected == nullptr) {
    return;
  }
  ++counters_.fusion_diagnostics_received;
  if (!validStamp(message->header.stamp)) {
    ++counters_.fusion_diagnostics_rejected;
    fusion_health_ = FusionHealthSnapshot{};
    fusion_health_.reason = "invalid_fusion_diagnostic_stamp";
    forceFusionUnhealthyLocked(0.0, "invalid_fusion_diagnostic_stamp");
    return;
  }
  const double stamp_sec = stampSeconds(message->header.stamp);
  if (fusion_health_.received && stamp_sec <= fusion_health_.stamp_sec + 1.0e-9) {
    ++counters_.fusion_diagnostics_rejected;
    return;
  }

  std::map<std::string, std::string> values;
  for (const auto & value : selected->values) {
    if (!values.emplace(value.key, value.value).second) {
      ++counters_.fusion_diagnostics_rejected;
      fusion_health_ = FusionHealthSnapshot{};
      fusion_health_.received = true;
      fusion_health_.stamp_sec = stamp_sec;
      fusion_health_.reason = "duplicate_fusion_health_key";
      forceFusionUnhealthyLocked(stamp_sec, "duplicate_fusion_health_key");
      return;
    }
  }
  constexpr std::array<const char *, 5> required_keys{
    "recovery.state", "anchor_valid", "recovery.position_fused",
    "recovery.yaw_fused", "last_fix_state"};
  if (!std::all_of(
      required_keys.begin(), required_keys.end(),
      [&values](const char * key) {return values.count(key) == 1U;}))
  {
    ++counters_.fusion_diagnostics_rejected;
    fusion_health_ = FusionHealthSnapshot{};
    fusion_health_.received = true;
    fusion_health_.stamp_sec = stamp_sec;
    fusion_health_.reason = "missing_fusion_health_key";
    forceFusionUnhealthyLocked(stamp_sec, "missing_fusion_health_key");
    return;
  }

  FusionHealthSnapshot snapshot;
  snapshot.received = true;
  snapshot.stamp_sec = stamp_sec;
  snapshot.level = selected->level;
  snapshot.recovery_state = values.at("recovery.state");
  snapshot.anchor_valid = values.at("anchor_valid");
  snapshot.position_fused = values.at("recovery.position_fused");
  snapshot.yaw_fused = values.at("recovery.yaw_fused");
  snapshot.last_fix_state = values.at("last_fix_state");
  ExistingFusionHealthFields fields;
  fields.level = snapshot.level;
  fields.recovery_state = snapshot.recovery_state;
  fields.anchor_valid = snapshot.anchor_valid;
  fields.position_fused = snapshot.position_fused;
  fields.yaw_fused = snapshot.yaw_fused;
  fields.last_fix_state = snapshot.last_fix_state;
  const FusionHealthEvaluation evaluation = evaluateStrictExistingFusionHealth(fields);
  snapshot.strict_fields_healthy = evaluation.healthy;
  snapshot.reason = evaluation.reason;
  fusion_health_ = snapshot;
  ++counters_.fusion_diagnostics_accepted;
  updateFusionHealthLocked(stamp_sec);
}

bool PrecisionGlobalLocalizerNode::processGnssLocked(
  const GnssInput & message, std::string & reason)
{
  if (!message.has_odom || !message.gnss_usable || !validStamp(message.header.stamp)) {
    reason = "not_usable";
    return false;
  }
  if (message.odom.header.stamp.sec != message.header.stamp.sec ||
    message.odom.header.stamp.nanosec != message.header.stamp.nanosec)
  {
    reason = "gnss_header_stamp_mismatch";
    return false;
  }
  if (message.odom.header.frame_id != map_frame_ ||
    !std::isfinite(message.odom.pose.pose.position.x) ||
    !std::isfinite(message.odom.pose.pose.position.y))
  {
    reason = "invalid_map_position_or_frame";
    return false;
  }
  if (message.has_confidence &&
    (!std::isfinite(message.confidence) || message.confidence < min_gnss_confidence_))
  {
    reason = "confidence_gate";
    return false;
  }
  const auto & gnss_covariance = message.odom.pose.covariance;
  Eigen::Matrix2d gnss_xy_covariance;
  gnss_xy_covariance <<
    gnss_covariance[0], gnss_covariance[1],
    gnss_covariance[6], gnss_covariance[7];
  if (!gnss_xy_covariance.allFinite() ||
    (gnss_xy_covariance - gnss_xy_covariance.transpose()).norm() > 1.0e-6)
  {
    reason = "invalid_gnss_covariance";
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> gnss_solver(gnss_xy_covariance);
  if (gnss_solver.info() != Eigen::Success || gnss_solver.eigenvalues().minCoeff() < -1.0e-9 ||
    gnss_solver.eigenvalues().maxCoeff() > anchor_config_.max_position_variance_m2)
  {
    reason = "gnss_covariance_gate";
    return false;
  }

  const double stamp_sec = stampSeconds(message.header.stamp);
  const InterpolatedLocal synchronized = interpolateLocalLocked(stamp_sec);
  if (!synchronized.valid) {
    reason = "awaiting_local_bracket";
    return false;
  }
  if (local_history_.empty()) {
    reason = "no_current_local_base";
    return false;
  }

  Eigen::Vector2d lever = Eigen::Vector2d::Zero();
  Eigen::Vector2d local_observation(synchronized.pose.x, synchronized.pose.y);
  if (!message.position_is_base_link) {
    if (!message.observation_point_valid ||
      !std::isfinite(message.observation_point_in_base.x) ||
      !std::isfinite(message.observation_point_in_base.y) ||
      !std::isfinite(message.observation_point_in_base.z))
    {
      reason = "single_antenna_geometry_missing";
      return false;
    }
    lever = Eigen::Vector2d(
      message.observation_point_in_base.x,
      message.observation_point_in_base.y);
    local_observation += rotation2(synchronized.pose.yaw) * lever;
  }

  const double gnss_position_variance = std::max(
    anchor_config_.min_position_variance_m2,
    0.5 * std::max(0.0, gnss_xy_covariance.trace()));
  const double local_position_variance = std::max(
    0.0, 0.5 * (synchronized.covariance(0, 0) + synchronized.covariance(1, 1)));
  const double lever_yaw_variance =
    lever.squaredNorm() * std::max(0.0, synchronized.covariance(2, 2));
  const double combined_variance =
    gnss_position_variance + local_position_variance + lever_yaw_variance;
  if (!std::isfinite(combined_variance) ||
    combined_variance > anchor_config_.max_position_variance_m2)
  {
    reason = "combined_position_uncertainty_gate";
    return false;
  }

  PositionAlignmentSample sample;
  sample.stamp_sec = stamp_sec;
  sample.local_point = local_observation;
  sample.map_point = Eigen::Vector2d(
    message.odom.pose.pose.position.x,
    message.odom.pose.pose.position.y);
  sample.position_variance_m2 = combined_variance;
  // Alignment uses the synchronized observation point above. Correction
  // continuity is bounded about the latest currently published base pose so a
  // delayed GNSS callback cannot jump a fast-moving vehicle.
  const Pose2 current_local_base = local_history_.back().pose;
  last_sync_to_current_sec_ = std::max(
    0.0, local_history_.back().stamp_sec - stamp_sec);
  const AnchorState before = gnss_anchor_->state();
  const AnchorUpdate update = gnss_anchor_->observePosition(sample, current_local_base);
  noteStateTransition(before, gnss_anchor_->state());
  reason = update.reason;
  if (!update.accepted) {
    return false;
  }
  position_fused_ = true;
  last_usable_stamp_sec_ = stamp_sec;
  if (message.fix_quality == 4) {
    last_q4_usable_stamp_sec_ = stamp_sec;
  }
  return true;
}

void PrecisionGlobalLocalizerNode::processPendingGnssLocked()
{
  if (!has_latest_local_stamp_) {
    counters_.gnss_pending = pending_gnss_.size();
    return;
  }
  std::deque<GnssInput::ConstSharedPtr> remaining;
  for (const auto & message : pending_gnss_) {
    const double stamp_sec = stampSeconds(message->header.stamp);
    if (stamp_sec > latest_local_stamp_sec_) {
      if (stamp_sec - latest_local_stamp_sec_ > sync_max_pending_sec_) {
        ++counters_.gnss_rejected;
        ++counters_.gnss_unsynced;
        ++counters_.gnss_pending_expired;
        last_gnss_reason_ = "future_local_timeout";
      } else {
        remaining.push_back(message);
      }
      continue;
    }
    if (latest_local_stamp_sec_ - stamp_sec > sync_history_sec_) {
      ++counters_.gnss_rejected;
      ++counters_.gnss_unsynced;
      ++counters_.gnss_pending_expired;
      last_gnss_reason_ = "history_expired";
      continue;
    }
    std::string reason;
    if (processGnssLocked(*message, reason)) {
      ++counters_.gnss_accepted;
    } else {
      ++counters_.gnss_rejected;
      if (reason == "awaiting_local_bracket") {
        ++counters_.gnss_unsynced;
      } else if (stamp_sec >= gnss_anchor_->lastUsableStamp()) {
        const AnchorState before = gnss_anchor_->state();
        gnss_anchor_->observeUnusable(stamp_sec);
        noteStateTransition(before, gnss_anchor_->state());
      }
    }
    last_gnss_reason_ = reason;
  }
  pending_gnss_.swap(remaining);
  counters_.gnss_pending = pending_gnss_.size();
}

void PrecisionGlobalLocalizerNode::onGnss(GnssInput::ConstSharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++counters_.gnss_received;
  last_fix_quality_ = message->fix_quality;
  if (message->heading_valid) {
    // Single-antenna trajectory/IMU-derived headings are correlated with the
    // same position stream and are never consumed as independent yaw.
    ++counters_.ignored_direct_heading;
  }
  if (!validStamp(message->header.stamp)) {
    ++counters_.gnss_rejected;
    last_gnss_reason_ = "invalid_stamp";
    return;
  }
  const double stamp_sec = stampSeconds(message->header.stamp);
  if (!message->has_odom || !message->gnss_usable) {
    const AnchorState before = gnss_anchor_->state();
    gnss_anchor_->observeUnusable(stamp_sec);
    noteStateTransition(before, gnss_anchor_->state());
    last_gnss_reason_ = gnss_anchor_->initialized() ?
      gnss_anchor_->lastReason() : "not_usable_before_init";
    return;
  }

  if (!has_latest_local_stamp_ || stamp_sec > latest_local_stamp_sec_) {
    if (has_latest_local_stamp_ &&
      stamp_sec - latest_local_stamp_sec_ > sync_max_pending_sec_)
    {
      ++counters_.gnss_rejected;
      ++counters_.gnss_unsynced;
      last_gnss_reason_ = "future_local_timeout";
      return;
    }
    pending_gnss_.push_back(message);
    while (pending_gnss_.size() > kPendingGnssLimit) {
      pending_gnss_.pop_front();
      ++counters_.gnss_rejected;
      ++counters_.gnss_unsynced;
      ++counters_.gnss_pending_expired;
    }
    counters_.gnss_pending = pending_gnss_.size();
    last_gnss_reason_ = "waiting_local_bracket";
    return;
  }

  std::string reason;
  if (processGnssLocked(*message, reason)) {
    ++counters_.gnss_accepted;
  } else {
    ++counters_.gnss_rejected;
    if (reason == "awaiting_local_bracket") {
      ++counters_.gnss_unsynced;
    } else if (stamp_sec >= gnss_anchor_->lastUsableStamp()) {
      const AnchorState before = gnss_anchor_->state();
      gnss_anchor_->observeUnusable(stamp_sec);
      noteStateTransition(before, gnss_anchor_->state());
    }
  }
  last_gnss_reason_ = reason;
}

void PrecisionGlobalLocalizerNode::publishDiagnostics()
{
  std::lock_guard<std::mutex> lock(mutex_);
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  const double diagnostic_stamp_sec = stampSeconds(array.header.stamp);
  if (diagnostic_stamp_sec > 0.0) {
    updateFusionHealthLocked(diagnostic_stamp_sec);
  }
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "localization/precision_global_localizer";
  status.hardware_id = "none";
  if (fusion_anchor_->state() == FusionAnchorState::WAITING_HEALTHY) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "waiting_for_healthy_existing_fusion";
  } else if (fusion_anchor_->state() == FusionAnchorState::STABILIZING_STARTUP) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "stabilizing_existing_fusion_startup";
  } else if (fusion_anchor_->state() == FusionAnchorState::FROZEN) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "fusion_unhealthy_anchor_frozen";
  } else if (fusion_anchor_->state() == FusionAnchorState::STABILIZING_RECOVERY) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "stabilizing_existing_fusion_recovery";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "tracking_existing_fusion_anchor";
  }
  auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(std::move(entry));
    };
  auto addUnsigned = [&add](const std::string & key, uint64_t value) {
      add(key, std::to_string(value));
    };

  const Pose2 target = fusion_anchor_->targetAnchor();
  const Pose2 applied = fusion_anchor_->appliedAnchor();
  if (!local_history_.empty()) {
    const Pose2 target_base = compose(target, local_history_.back().pose);
    const Pose2 applied_base = compose(applied, local_history_.back().pose);
    current_anchor_lag_translation_m_ = poseTranslationDistance(target_base, applied_base);
    current_anchor_lag_yaw_rad_ = std::fabs(wrapAngle(target.yaw - applied.yaw));
    max_anchor_lag_translation_m_ = std::max(
      max_anchor_lag_translation_m_, current_anchor_lag_translation_m_);
    max_anchor_lag_yaw_rad_ = std::max(
      max_anchor_lag_yaw_rad_, current_anchor_lag_yaw_rad_);
  }
  const double usable_age = has_latest_local_stamp_ && std::isfinite(last_usable_stamp_sec_) ?
    std::max(0.0, latest_local_stamp_sec_ - last_usable_stamp_sec_) :
    std::numeric_limits<double>::quiet_NaN();

  add("state", toString(fusion_anchor_->state()));
  add("position_fused", fusion_anchor_->globalOutputReady() ? "true" : "false");
  add("position_initialized", fusion_anchor_->globalOutputReady() ? "true" : "false");
  add("yaw_publishable", fusion_anchor_->globalOutputReady() ? "true" : "false");
  add("global_output_ready", fusion_anchor_->globalOutputReady() ? "true" : "false");
  add("anchor.source", "existing_fusion");
  add("anchor.initialized", fusion_anchor_->globalOutputReady() ? "true" : "false");
  add("anchor.yaw_observed", fusion_anchor_->globalOutputReady() ? "true" : "false");
  add("anchor.yaw_publishable", fusion_anchor_->globalOutputReady() ? "true" : "false");
  add("anchor.target.x_m", number(target.x));
  add("anchor.target.y_m", number(target.y));
  add("anchor.target.yaw_rad", number(target.yaw));
  add("anchor.applied.x_m", number(applied.x));
  add("anchor.applied.y_m", number(applied.y));
  add("anchor.applied.yaw_rad", number(applied.yaw));
  add("anchor.correction_lag.translation_m", number(current_anchor_lag_translation_m_));
  add("anchor.correction_lag.yaw_rad", number(current_anchor_lag_yaw_rad_));
  add("anchor.correction_lag.max_translation_m", number(max_anchor_lag_translation_m_));
  add("anchor.correction_lag.max_yaw_rad", number(max_anchor_lag_yaw_rad_));
  add("anchor.last_reason", fusion_anchor_->lastReason());
  add("activation.stamp_sec", number(fusion_anchor_->activationStamp()));
  addUnsigned("activation.stable_candidate_count", fusion_anchor_->stableCandidateCount());
  addUnsigned(
    "activation.required_candidate_count", fusion_anchor_->requiredStableCandidateCount());
  add("activation.candidate_yaw_rad", number(fusion_anchor_->activationCandidateYaw()));
  add("activation.candidate_delta_rad", number(fusion_anchor_->candidateYawDelta()));
  add("activation.reason", fusion_anchor_->activationReason());
  addUnsigned("activation.epoch", fusion_anchor_->activationEpoch());
  addUnsigned("activation.commit_count", fusion_anchor_->activationCount());

  add("fusion.health.healthy", fusion_anchor_->fusionHealthy() ? "true" : "false");
  add("fusion.health.age_sec", number(fusion_health_age_sec_));
  add("fusion.health.status_stamp_sec", number(fusion_health_.stamp_sec));
  add("fusion.health.level", std::to_string(static_cast<int>(fusion_health_.level)));
  add("fusion.health.recovery_state", fusion_health_.recovery_state);
  add("fusion.health.anchor_valid", fusion_health_.anchor_valid);
  add("fusion.health.position_fused", fusion_health_.position_fused);
  add("fusion.health.yaw_fused", fusion_health_.yaw_fused);
  add("fusion.health.last_fix_state", fusion_health_.last_fix_state);
  add("fusion.health.reason", fusion_anchor_->healthReason());
  add("fusion.health.rearm_required",
    fusion_rearm_state_.required ? "true" : "false");
  add("fusion.health.rearm_saw_unhealthy",
    fusion_rearm_state_.saw_unhealthy ? "true" : "false");
  add("fusion.health.rearmed", fusion_rearm_state_.rearmed ? "true" : "false");
  add("fusion.health.rearm_reset_stamp_sec", number(fusion_rearm_reset_stamp_sec_));
  addUnsigned("fusion.health.received", counters_.fusion_diagnostics_received);
  addUnsigned("fusion.health.accepted", counters_.fusion_diagnostics_accepted);
  addUnsigned("fusion.health.rejected", counters_.fusion_diagnostics_rejected);

  add("fusion.anchor.state", toString(fusion_anchor_->state()));
  addUnsigned("fusion.anchor.candidate_count", fusion_anchor_->stableCandidateCount());
  addUnsigned(
    "fusion.anchor.required_candidate_count", fusion_anchor_->requiredStableCandidateCount());
  add("fusion.anchor.candidate_base_translation_m",
    number(fusion_anchor_->candidateBaseTranslation()));
  add("fusion.anchor.candidate_yaw_delta_rad", number(fusion_anchor_->candidateYawDelta()));
  add("fusion.anchor.last_candidate_stamp_sec", number(fusion_anchor_->lastCandidateStamp()));
  add("fusion.anchor.last_reason", fusion_anchor_->lastReason());
  addUnsigned("fusion.anchor.accepted_count", fusion_anchor_->acceptedCount());
  addUnsigned("fusion.anchor.rejected_count", fusion_anchor_->rejectedCount());
  addUnsigned("fusion.anchor.target_update_count", fusion_anchor_->targetUpdateCount());
  addUnsigned("fusion.anchor.applied_step_count", fusion_anchor_->appliedStepCount());
  addUnsigned("fusion.anchor.freeze_count", fusion_anchor_->freezeCount());
  addUnsigned("fusion.anchor.recovery_count", fusion_anchor_->recoveryCount());
  add("fusion.anchor.last_applied_base_translation_m",
    number(fusion_anchor_->lastAppliedBaseTranslation()));
  add("fusion.anchor.last_applied_yaw_rad", number(fusion_anchor_->lastAppliedYaw()));
  add("fusion.anchor.frozen_residual_variance_x_m2",
    number(frozen_anchor_residual_covariance_(0, 0)));
  add("fusion.anchor.frozen_residual_variance_y_m2",
    number(frozen_anchor_residual_covariance_(1, 1)));
  add("fusion.anchor.frozen_residual_variance_yaw_rad2",
    number(frozen_anchor_residual_covariance_(2, 2)));

  addUnsigned("fusion.sync.existing_global_received", counters_.existing_global_received);
  addUnsigned("fusion.sync.existing_global_accepted", counters_.existing_global_accepted);
  addUnsigned("fusion.sync.existing_global_rejected", counters_.existing_global_rejected);
  addUnsigned(
    "fusion.sync.existing_global_duplicate_stamp",
    counters_.existing_global_duplicate_stamp);
  addUnsigned("fusion.sync.accepted", counters_.fusion_sync_accepted);
  addUnsigned("fusion.sync.rejected", counters_.fusion_sync_rejected);
  addUnsigned("fusion.sync.pending", pending_fusion_locals_.size());
  add("fusion.sync.existing_global_stamp_sec",
    existing_global_history_.empty() ? "nan" :
    number(existing_global_history_.back().stamp_sec));
  add("fusion.sync.existing_global_age_sec", number(existing_global_age_sec_));
  add("fusion.sync.last_valid_stamp_sec", number(last_valid_fusion_sync_stamp_sec_));
  add("fusion.sync.last_valid_age_sec", number(valid_fusion_sync_age_sec_));
  add("fusion.sync.last_error_sec", number(last_fusion_sync_error_sec_));
  add("fusion.sync.last_mode", last_fusion_sync_mode_);
  add("fusion.sync.last_reason", last_fusion_sync_reason_);
  add("fallback.gnss_position_enabled",
    gnss_position_diagnostics_enabled_ ? "true" : "false");

  add("gnss.last_usable_stamp_sec", number(last_usable_stamp_sec_));
  add("gnss.last_usable_age_sec", number(usable_age));
  add("gnss.last_q4_usable_stamp_sec", number(last_q4_usable_stamp_sec_));
  add("gnss.last_sync_to_current_sec", number(last_sync_to_current_sec_));
  add("gnss.last_fix_quality", std::to_string(static_cast<int>(last_fix_quality_)));
  addUnsigned("gnss.received", counters_.gnss_received);
  addUnsigned("gnss.accepted", counters_.gnss_accepted);
  addUnsigned("gnss.rejected", counters_.gnss_rejected);
  addUnsigned("gnss.unsynced", counters_.gnss_unsynced);
  addUnsigned("gnss.pending", counters_.gnss_pending);
  addUnsigned("gnss.pending_expired", counters_.gnss_pending_expired);
  addUnsigned("gnss.direct_heading_ignored", counters_.ignored_direct_heading);
  add("gnss.last_reason", last_gnss_reason_);

  addUnsigned("state.soft_gap_count", counters_.soft_gap_count);
  addUnsigned("state.outage_count", counters_.outage_count);
  addUnsigned("state.recovery_count", counters_.recovery_count);

  addUnsigned("local_correction.accepted", counters_.correction_accepted);
  addUnsigned("local_correction.rejected", counters_.correction_rejected);
  addUnsigned("local_correction.pending", counters_.correction_pending);
  addUnsigned("local_correction.pending_expired", counters_.correction_expired);
  addUnsigned("local_correction.session_rebases", counters_.session_rebases);
  addUnsigned("local_correction.epoch_resets", counters_.epoch_resets);
  addUnsigned("local_correction.odom_session_resets", counters_.odom_session_resets);
  addUnsigned("local_correction.matcher_session", compositor_->matcherSession());
  addUnsigned("local_correction.correction_id", compositor_->correctionId());
  add("local_correction.last_reason", last_correction_reason_);
  add("local_correction.lag.translation_m", number(current_local_lag_translation_m_));
  add("local_correction.lag.yaw_rad", number(current_local_lag_yaw_rad_));
  add("local_correction.lag.max_translation_m", number(max_local_lag_translation_m_));
  add("local_correction.lag.max_yaw_rad", number(max_local_lag_yaw_rad_));
  add("local_correction.precision_frame", precision_frame_);
  add("local_correction.tf_published", "false");
  addUnsigned("submap.scan_received", counters_.scan_received);
  addUnsigned("submap.scan_rejected", counters_.scan_rejected);
  addUnsigned("submap.correction_received", counters_.correction_received);
  addUnsigned("raw.received", counters_.raw_received);
  addUnsigned("raw.invalid", counters_.raw_invalid);
  addUnsigned("raw.backstep", counters_.raw_nonmonotonic);
  addUnsigned("raw.nonmonotonic", counters_.raw_nonmonotonic);
  addUnsigned("raw.duplicate_stamp", counters_.raw_duplicate_stamp);
  addUnsigned("publish.local", counters_.local_published);
  addUnsigned("publish.global", counters_.global_published);
  addUnsigned("publish.global_suppressed_not_ready", counters_.global_suppressed_not_ready);
  addUnsigned(
    "publish.global_suppressed_activation_watermark",
    counters_.global_suppressed_activation_watermark);

  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);
}

std::shared_ptr<rclcpp::Node> makePrecisionGlobalLocalizerNode(
  const rclcpp::NodeOptions & options)
{
  return std::make_shared<PrecisionGlobalLocalizerNode>(options);
}

}  // namespace pure_precision_global_localizer
