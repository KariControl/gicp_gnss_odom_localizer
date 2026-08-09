#include "pure_gnss_map_odom_fusion/map_odom_fusion_node.hpp"

#include <functional>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace pure_gnss_map_odom_fusion
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kUnknownVariance = 1.0e6;
constexpr double kDisabledVariance = 1.0e12;
constexpr double kMinimumVariance = 1.0e-9;

bool quaternionIsValid(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(norm_squared) && norm_squared > 1.0e-12 &&
         std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
         std::isfinite(quaternion.z) && std::isfinite(quaternion.w);
}

double sanitizedNonNegative(double value)
{
  return std::isfinite(value) && value >= 0.0 ? value : 0.0;
}

bool twistIsFinite(const geometry_msgs::msg::Twist & twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

std::array<double, 36> sanitizedCovariance(
  const std::array<double, 36> & covariance)
{
  std::array<double, 36> output{};
  for (std::size_t index = 0; index < covariance.size(); ++index) {
    const bool diagonal = index % 7U == 0U;
    if (std::isfinite(covariance[index])) {
      output[index] = diagonal ? std::max(0.0, covariance[index]) : covariance[index];
    } else {
      output[index] = diagonal ? kUnknownVariance : 0.0;
    }
  }
  return output;
}

std::string fixStateToString(int state)
{
  switch (state) {
    case 1:
      return "good";
    case 2:
      return "bad";
    default:
      return "unknown";
  }
}


template<typename MatrixType>
bool fixedLdltIsPositive(const Eigen::LDLT<MatrixType> & ldlt)
{
  return ldlt.info() == Eigen::Success && ldlt.vectorD().allFinite() &&
         (ldlt.vectorD().array() > 0.0).all();
}
}  // namespace

double MapOdomFusionNode::wrapAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double MapOdomFusionNode::yawFromIso(const Eigen::Isometry3d & transform)
{
  return wrapAngle(std::atan2(transform.linear()(1, 0), transform.linear()(0, 0)));
}

Eigen::Quaterniond MapOdomFusionNode::quatFromYaw(double yaw)
{
  Eigen::Quaterniond quaternion(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  quaternion.normalize();
  return quaternion;
}

Eigen::Isometry3d MapOdomFusionNode::poseMsgToIso(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1.0e-9) {
    quaternion = Eigen::Quaterniond::Identity();
  }
  quaternion.normalize();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = quaternion.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

geometry_msgs::msg::Pose MapOdomFusionNode::isoToPoseMsg(
  const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();

  Eigen::Quaterniond quaternion(transform.linear());
  if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1.0e-9) {
    quaternion = Eigen::Quaterniond::Identity();
  }
  quaternion.normalize();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

Eigen::Isometry3d MapOdomFusionNode::deltaFromTwist(
  const geometry_msgs::msg::Twist & twist,
  double dt_sec)
{
  Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
  if (!std::isfinite(dt_sec) || std::fabs(dt_sec) <= 1.0e-12) {
    return delta;
  }

  const double vx = std::isfinite(twist.linear.x) ? twist.linear.x : 0.0;
  const double vy = std::isfinite(twist.linear.y) ? twist.linear.y : 0.0;
  const double yaw_rate = std::isfinite(twist.angular.z) ? twist.angular.z : 0.0;
  const double delta_yaw = yaw_rate * dt_sec;

  if (std::fabs(yaw_rate) < 1.0e-9 || std::fabs(delta_yaw) < 1.0e-9) {
    delta.translation() = Eigen::Vector3d(vx * dt_sec, vy * dt_sec, 0.0);
    return delta;
  }

  const double sine = std::sin(delta_yaw);
  const double cosine = std::cos(delta_yaw);
  delta.translation() = Eigen::Vector3d(
    (sine * vx - (1.0 - cosine) * vy) / yaw_rate,
    ((1.0 - cosine) * vx + sine * vy) / yaw_rate,
    0.0);
  delta.linear() = quatFromYaw(delta_yaw).toRotationMatrix();
  return delta;
}

bool MapOdomFusionNode::isFiniteTransform(const Eigen::Isometry3d & transform)
{
  return transform.matrix().allFinite() &&
         std::isfinite(transform.linear().determinant());
}

Eigen::Matrix3d MapOdomFusionNode::sanitizeMeasurementCovariance(
  const Eigen::Matrix3d & covariance,
  double fallback_xy,
  double fallback_yaw,
  bool yaw_valid)
{
  const std::array<double, 3> fallback{
    std::max(kMinimumVariance, fallback_xy),
    std::max(kMinimumVariance, fallback_xy),
    std::max(kMinimumVariance, yaw_valid ? fallback_yaw : kUnknownVariance)};

  Eigen::Matrix3d output = 0.5 * (covariance + covariance.transpose());
  for (int index = 0; index < 3; ++index) {
    if (!std::isfinite(output(index, index)) || output(index, index) <= 0.0) {
      output(index, index) = fallback[static_cast<std::size_t>(index)];
    }
  }

  if (!yaw_valid) {
    output.row(2).setZero();
    output.col(2).setZero();
    output(2, 2) = fallback[2];
  }

  for (int row = 0; row < 3; ++row) {
    for (int column = row + 1; column < 3; ++column) {
      double value = output(row, column);
      if (!std::isfinite(value)) {
        value = 0.0;
      }
      const double correlation_limit = 0.999 * std::sqrt(
        std::max(0.0, output(row, row) * output(column, column)));
      value = std::clamp(value, -correlation_limit, correlation_limit);
      output(row, column) = value;
      output(column, row) = value;
    }
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(output);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    return Eigen::Vector3d(fallback[0], fallback[1], fallback[2]).asDiagonal();
  }

  Eigen::Vector3d eigenvalues = solver.eigenvalues();
  for (int index = 0; index < 3; ++index) {
    eigenvalues(index) = std::max(kMinimumVariance, eigenvalues(index));
  }
  output = solver.eigenvectors() * eigenvalues.asDiagonal() *
    solver.eigenvectors().transpose();
  return 0.5 * (output + output.transpose());
}

Eigen::Vector3d MapOdomFusionNode::observationPointInOdom(
  const OdomSample & odom,
  const AbsolutePoseMeasurement & measurement)
{
  Eigen::Vector3d point = odom.T_odom_base.translation();
  if (!measurement.position_is_base_link && measurement.observation_point_valid) {
    point += odom.T_odom_base.linear() * measurement.observation_point_in_base;
  }
  return point;
}

Eigen::Isometry3d MapOdomFusionNode::transformFromAlignment(
  const RecoveryAlignmentResult & alignment)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().x() = alignment.tx_m;
  transform.translation().y() = alignment.ty_m;
  transform.linear() = quatFromYaw(alignment.yaw_rad).toRotationMatrix();
  return transform;
}

Eigen::Matrix3d MapOdomFusionNode::recoveryAlignmentCovarianceFloor(
  const RecoveryAlignmentResult & alignment,
  const AbsolutePoseMeasurement & measurement) const
{
  const double alignment_position_variance = std::max(
    {anchor_cov_xy_default_, measurement.cov_xy,
      alignment.position_rms_m * alignment.position_rms_m});

  double yaw_sigma = std::sqrt(anchor_cov_yaw_default_);
  if (measurement.yaw_valid && std::isfinite(measurement.cov_yaw)) {
    yaw_sigma = std::max(yaw_sigma, std::sqrt(std::max(0.0, measurement.cov_yaw)));
  }
  if (alignment.used_direct_heading && std::isfinite(alignment.heading_std_rad)) {
    yaw_sigma = std::max(yaw_sigma, alignment.heading_std_rad);
  }
  if (alignment.used_position_heading) {
    yaw_sigma = std::max(
      yaw_sigma,
      std::atan2(
        std::max(
          alignment.position_rms_m,
          std::sqrt(std::max(0.0, measurement.cov_xy))),
        std::max(1.0e-3, alignment.odom_baseline_m)));
  }

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance(0, 0) = alignment_position_variance;
  covariance(1, 1) = alignment_position_variance;
  covariance(2, 2) = std::max(anchor_cov_yaw_default_, yaw_sigma * yaw_sigma);
  return covariance;
}

Eigen::Matrix3d MapOdomFusionNode::conservativeRecoveryTargetCovariance(
  const RecoveryAlignmentResult & alignment,
  const AbsolutePoseMeasurement & measurement,
  const OdomSample & odom,
  const AnchorState & anchor) const
{
  const double elapsed = std::max(0.0, (measurement.stamp - anchor.stamp).seconds());
  const double odom_cov_xy_since = std::max(
    0.0, odom.cov_xy_total - anchor.odom_cov_xy_ref);
  const double odom_cov_yaw_since = std::max(
    0.0, odom.cov_yaw_total - anchor.odom_cov_yaw_ref);
  const double prior_position_variance = std::max(
    anchor.P(0, 0), anchor.P(1, 1)) +
    cov_xy_drift_per_sec_ * no_fix_cov_drift_scale_ * elapsed +
    odom_cov_xy_since;
  const double prior_yaw_variance = anchor.P(2, 2) +
    cov_yaw_drift_per_sec_ * no_fix_cov_drift_scale_ * elapsed +
    odom_cov_yaw_since;

  Eigen::Matrix3d covariance = recoveryAlignmentCovarianceFloor(alignment, measurement);
  covariance(0, 0) = std::max(covariance(0, 0), prior_position_variance);
  covariance(1, 1) = std::max(covariance(1, 1), prior_position_variance);
  covariance(2, 2) = std::max(covariance(2, 2), prior_yaw_variance);
  return covariance;
}

Eigen::Matrix3d MapOdomFusionNode::conservativeXyOnlyTargetCovariance(
  const FixedYawTranslationResult & alignment,
  const AbsolutePoseMeasurement & measurement,
  const OdomSample & odom,
  const AnchorState & anchor) const
{
  const double elapsed = std::max(0.0, (measurement.stamp - anchor.stamp).seconds());
  const double odom_cov_xy_since = std::max(
    0.0, odom.cov_xy_total - anchor.odom_cov_xy_ref);
  const double odom_cov_yaw_since = std::max(
    0.0, odom.cov_yaw_total - anchor.odom_cov_yaw_ref);
  const double lever_arm_xy = measurement.position_is_base_link ? 0.0 :
    measurement.observation_point_in_base.head<2>().norm();

  Eigen::Matrix3d covariance = anchor.P;
  covariance(0, 0) +=
    cov_xy_drift_per_sec_ * no_fix_cov_drift_scale_ * elapsed + odom_cov_xy_since;
  covariance(1, 1) +=
    cov_xy_drift_per_sec_ * no_fix_cov_drift_scale_ * elapsed + odom_cov_xy_since;
  covariance(2, 2) +=
    cov_yaw_drift_per_sec_ * no_fix_cov_drift_scale_ * elapsed + odom_cov_yaw_since;

  // A fixed but uncertain yaw rotates a horizontal antenna lever arm into an
  // additional position uncertainty. Keep that uncertainty in XY and retain
  // the complete prior yaw variance; a position-only correction must never
  // claim a new absolute yaw observation.
  const double lever_arm_variance =
    lever_arm_xy * lever_arm_xy * std::max(0.0, covariance(2, 2));
  const double target_position_variance = std::max(
    {anchor_cov_xy_default_, measurement.cov_xy,
      alignment.position_rms_m * alignment.position_rms_m + lever_arm_variance});
  covariance(0, 0) = std::max(covariance(0, 0), target_position_variance);
  covariance(1, 1) = std::max(covariance(1, 1), target_position_variance);
  covariance = 0.5 * (covariance + covariance.transpose());
  return covariance;
}

MapOdomFusionNode::MapOdomFusionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("map_odom_fusion", options)
{
  declareAndLoadParameters();
  validateParameters();

  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&MapOdomFusionNode::onOdom, this, std::placeholders::_1));

  if (!gnss_input_topic_.empty()) {
    gnss_subscription_ = create_subscription<pure_gnss_msgs::msg::GnssFusionInput>(
      gnss_input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MapOdomFusionNode::onGnssInput, this, std::placeholders::_1));
  }

  if (!legacy_anchor_topic_.empty()) {
    legacy_anchor_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      legacy_anchor_topic_, 10,
      std::bind(&MapOdomFusionNode::onLegacyAnchorPose, this, std::placeholders::_1));
  }

  if (!initialpose_topic_.empty()) {
    initialpose_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, 10,
      std::bind(&MapOdomFusionNode::onInitialPose, this, std::placeholders::_1));
  }

  pose_publisher_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(out_pose_topic_, 10);
  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(out_odom_topic_, 10);
  diagnostic_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

  if (publish_tf_) {
    transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&MapOdomFusionNode::onPublishTimer, this));
  heartbeat_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / heartbeat_hz_),
    std::bind(&MapOdomFusionNode::onHeartbeat, this));

  RCLCPP_INFO(
    get_logger(),
    "GNSS map/odom fusion started: odom=%s gnss=%s output=%s",
    odom_topic_.c_str(),
    gnss_input_topic_.empty() ? "<disabled>" : gnss_input_topic_.c_str(),
    out_odom_topic_.c_str());
}

void MapOdomFusionNode::declareAndLoadParameters()
{
  map_frame_ = declare_parameter<std::string>("map_frame", map_frame_);
  odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
  base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
  odom_topic_ = declare_parameter<std::string>("odom_topic", odom_topic_);
  gnss_input_topic_ = declare_parameter<std::string>("gnss_input_topic", gnss_input_topic_);
  legacy_anchor_topic_ = declare_parameter<std::string>(
    "legacy_anchor_topic", legacy_anchor_topic_);
  initialpose_topic_ = declare_parameter<std::string>("initialpose_topic", initialpose_topic_);
  out_pose_topic_ = declare_parameter<std::string>("out_pose_topic", out_pose_topic_);
  out_odom_topic_ = declare_parameter<std::string>("out_odom_topic", out_odom_topic_);

  publish_tf_ = declare_parameter<bool>("publish_tf", publish_tf_);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", publish_rate_hz_);
  heartbeat_hz_ = declare_parameter<double>("heartbeat_hz", heartbeat_hz_);

  odom_buffer_sec_ = declare_parameter<double>("odom_buffer_sec", odom_buffer_sec_);
  odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", odom_timeout_sec_);
  odom_max_extrapolate_sec_ = declare_parameter<double>(
    "odom_max_extrapolate_sec", odom_max_extrapolate_sec_);
  measurement_max_age_sec_ = declare_parameter<double>(
    "measurement_max_age_sec", measurement_max_age_sec_);
  measurement_future_tolerance_sec_ = declare_parameter<double>(
    "measurement_future_tolerance_sec", measurement_future_tolerance_sec_);
  measurement_reorder_tolerance_sec_ = declare_parameter<double>(
    "measurement_reorder_tolerance_sec", measurement_reorder_tolerance_sec_);

  anchor_cov_xy_default_ = declare_parameter<double>(
    "anchor_cov_xy_default", anchor_cov_xy_default_);
  anchor_cov_yaw_default_ = declare_parameter<double>(
    "anchor_cov_yaw_default", anchor_cov_yaw_default_);
  cov_xy_drift_per_sec_ = declare_parameter<double>(
    "cov_xy_drift_per_sec", cov_xy_drift_per_sec_);
  cov_yaw_drift_per_sec_ = declare_parameter<double>(
    "cov_yaw_drift_per_sec", cov_yaw_drift_per_sec_);
  no_fix_cov_drift_scale_ = declare_parameter<double>(
    "no_fix_cov_drift_scale", no_fix_cov_drift_scale_);

  use_gnss_status_ = declare_parameter<bool>("use_gnss_status", use_gnss_status_);
  gnss_status_timeout_sec_ = declare_parameter<double>(
    "gnss_status_timeout_sec", gnss_status_timeout_sec_);
  gnss_fix_min_status_ = declare_parameter<int>("gnss_fix_min_status", gnss_fix_min_status_);
  gnss_init_require_fix_ = declare_parameter<bool>(
    "gnss_init_require_fix", gnss_init_require_fix_);
  gnss_allow_unknown_observation_point_ = declare_parameter<bool>(
    "gnss_allow_unknown_observation_point", gnss_allow_unknown_observation_point_);

  const int initialization_min_samples = declare_parameter<int>(
    "initialization_min_samples", static_cast<int>(initialization_min_samples_));
  const int initialization_min_heading_samples = declare_parameter<int>(
    "initialization_min_heading_samples",
    static_cast<int>(initialization_min_heading_samples_));
  initialization_min_samples_ = initialization_min_samples > 0 ?
    static_cast<std::size_t>(initialization_min_samples) : 0U;
  initialization_min_heading_samples_ = initialization_min_heading_samples > 0 ?
    static_cast<std::size_t>(initialization_min_heading_samples) : 0U;

  gnss_max_cov_xy_ = declare_parameter<double>("gnss_max_cov_xy", gnss_max_cov_xy_);
  gnss_max_cov_yaw_ = declare_parameter<double>("gnss_max_cov_yaw", gnss_max_cov_yaw_);
  gnss_position_nis_threshold_ = declare_parameter<double>(
    "gnss_position_nis_threshold", gnss_position_nis_threshold_);
  gnss_yaw_nis_threshold_ = declare_parameter<double>(
    "gnss_yaw_nis_threshold", gnss_yaw_nis_threshold_);
  gnss_position_jump_reject_m_ = declare_parameter<double>(
    "gnss_position_jump_reject_m", gnss_position_jump_reject_m_);
  gnss_yaw_jump_reject_rad_ = declare_parameter<double>(
    "gnss_yaw_jump_reject_rad", gnss_yaw_jump_reject_rad_);
  gnss_jump_reject_sigma_scale_ = declare_parameter<double>(
    "gnss_jump_reject_sigma_scale", gnss_jump_reject_sigma_scale_);
  const int tracking_rejection_min_samples = declare_parameter<int>(
    "tracking_rejection_min_samples",
    static_cast<int>(tracking_rejection_min_samples_));
  tracking_rejection_min_samples_ = tracking_rejection_min_samples > 0 ?
    static_cast<std::size_t>(tracking_rejection_min_samples) : 0U;

  gnss_outage_timeout_sec_ = declare_parameter<double>(
    "gnss_outage_timeout_sec", gnss_outage_timeout_sec_);
  const int recovery_min_samples = declare_parameter<int>(
    "recovery_min_samples", static_cast<int>(recovery_alignment_config_.min_samples));
  const int recovery_min_heading_samples = declare_parameter<int>(
    "recovery_min_heading_samples",
    static_cast<int>(recovery_alignment_config_.min_heading_samples));
  recovery_alignment_config_.min_samples = recovery_min_samples > 0 ?
    static_cast<std::size_t>(recovery_min_samples) : 0U;
  recovery_alignment_config_.min_heading_samples = recovery_min_heading_samples > 0 ?
    static_cast<std::size_t>(recovery_min_heading_samples) : 0U;
  recovery_alignment_config_.max_sample_gap_sec = declare_parameter<double>(
    "recovery_max_sample_gap_sec", recovery_alignment_config_.max_sample_gap_sec);
  recovery_alignment_config_.min_odom_baseline_m = declare_parameter<double>(
    "recovery_min_odom_baseline_m", recovery_alignment_config_.min_odom_baseline_m);
  recovery_alignment_config_.max_position_rms_m = declare_parameter<double>(
    "recovery_max_position_rms_m", recovery_alignment_config_.max_position_rms_m);
  recovery_alignment_config_.max_position_residual_m = declare_parameter<double>(
    "recovery_max_position_residual_m",
    recovery_alignment_config_.max_position_residual_m);
  recovery_alignment_config_.max_heading_std_rad = declare_parameter<double>(
    "recovery_max_heading_std_rad", recovery_alignment_config_.max_heading_std_rad);
  recovery_alignment_config_.max_heading_source_disagreement_rad =
    declare_parameter<double>(
    "recovery_max_heading_source_disagreement_rad",
    recovery_alignment_config_.max_heading_source_disagreement_rad);
  recovery_alignment_config_.allow_single_outlier_rejection = declare_parameter<bool>(
    "recovery_allow_single_outlier_rejection",
    recovery_alignment_config_.allow_single_outlier_rejection);
  recovery_candidate_window_sec_ = declare_parameter<double>(
    "recovery_candidate_window_sec", recovery_candidate_window_sec_);
  const int recovery_candidate_max_samples = declare_parameter<int>(
    "recovery_candidate_max_samples", static_cast<int>(recovery_candidate_max_samples_));
  recovery_candidate_max_samples_ = recovery_candidate_max_samples > 0 ?
    static_cast<std::size_t>(recovery_candidate_max_samples) : 0U;
  recovery_max_target_translation_m_ = declare_parameter<double>(
    "recovery_max_target_translation_m", recovery_max_target_translation_m_);
  recovery_max_target_yaw_rad_ = declare_parameter<double>(
    "recovery_max_target_yaw_rad", recovery_max_target_yaw_rad_);
  recovery_max_target_refresh_translation_m_ = declare_parameter<double>(
    "recovery_max_target_refresh_translation_m",
    recovery_max_target_refresh_translation_m_);
  recovery_max_target_refresh_yaw_rad_ = declare_parameter<double>(
    "recovery_max_target_refresh_yaw_rad", recovery_max_target_refresh_yaw_rad_);
  recovery_max_correction_per_update_m_ = declare_parameter<double>(
    "recovery_max_correction_per_update_m", recovery_max_correction_per_update_m_);
  recovery_max_correction_per_sec_m_ = declare_parameter<double>(
    "recovery_max_correction_per_sec_m", recovery_max_correction_per_sec_m_);
  recovery_max_yaw_correction_per_update_rad_ = declare_parameter<double>(
    "recovery_max_yaw_correction_per_update_rad",
    recovery_max_yaw_correction_per_update_rad_);
  recovery_max_yaw_correction_per_sec_rad_ = declare_parameter<double>(
    "recovery_max_yaw_correction_per_sec_rad", recovery_max_yaw_correction_per_sec_rad_);
  recovery_exit_position_m_ = declare_parameter<double>(
    "recovery_exit_position_m", recovery_exit_position_m_);
  recovery_exit_yaw_rad_ = declare_parameter<double>(
    "recovery_exit_yaw_rad", recovery_exit_yaw_rad_);
  const int recovery_exit_min_samples = declare_parameter<int>(
    "recovery_exit_min_samples", static_cast<int>(recovery_exit_min_samples_));
  recovery_exit_min_samples_ = recovery_exit_min_samples > 0 ?
    static_cast<std::size_t>(recovery_exit_min_samples) : 0U;

  xy_only_recovery_enabled_ = declare_parameter<bool>(
    "xy_only_recovery.enabled", xy_only_recovery_enabled_);
  xy_only_required_fix_quality_ = declare_parameter<int>(
    "xy_only_recovery.required_fix_quality", xy_only_required_fix_quality_);
  const int xy_only_min_samples = declare_parameter<int>(
    "xy_only_recovery.min_samples", 10);
  xy_only_alignment_config_.min_samples = xy_only_min_samples > 0 ?
    static_cast<std::size_t>(xy_only_min_samples) : 0U;
  xy_only_alignment_config_.max_sample_gap_sec = declare_parameter<double>(
    "xy_only_recovery.max_sample_gap_sec", 0.50);
  xy_only_alignment_config_.max_position_rms_m = declare_parameter<double>(
    "xy_only_recovery.max_position_rms_m", 0.25);
  xy_only_alignment_config_.max_position_residual_m = declare_parameter<double>(
    "xy_only_recovery.max_position_residual_m", 0.75);
  xy_only_alignment_config_.allow_single_outlier_rejection = declare_parameter<bool>(
    "xy_only_recovery.allow_single_outlier_rejection", true);
  xy_only_min_candidate_span_sec_ = declare_parameter<double>(
    "xy_only_recovery.min_candidate_span_sec", xy_only_min_candidate_span_sec_);
  xy_only_stationary_dwell_sec_ = declare_parameter<double>(
    "xy_only_recovery.stationary_dwell_sec", xy_only_stationary_dwell_sec_);
  xy_only_max_speed_mps_ = declare_parameter<double>(
    "xy_only_recovery.max_speed_mps", xy_only_max_speed_mps_);
  xy_only_max_yaw_rate_radps_ = declare_parameter<double>(
    "xy_only_recovery.max_yaw_rate_radps", xy_only_max_yaw_rate_radps_);
  xy_only_max_stationary_displacement_m_ = declare_parameter<double>(
    "xy_only_recovery.max_stationary_displacement_m",
    xy_only_max_stationary_displacement_m_);
  xy_only_max_odom_yaw_change_rad_ = declare_parameter<double>(
    "xy_only_recovery.max_odom_yaw_change_rad",
    xy_only_max_odom_yaw_change_rad_);
  xy_only_max_odom_gap_sec_ = declare_parameter<double>(
    "xy_only_recovery.max_odom_gap_sec", xy_only_max_odom_gap_sec_);
  xy_only_max_anchor_yaw_variance_rad2_ = declare_parameter<double>(
    "xy_only_recovery.max_anchor_yaw_variance_rad2",
    xy_only_max_anchor_yaw_variance_rad2_);
  xy_only_max_outage_sec_ = declare_parameter<double>(
    "xy_only_recovery.max_outage_sec", xy_only_max_outage_sec_);
  xy_only_max_lever_arm_error_m_ = declare_parameter<double>(
    "xy_only_recovery.max_lever_arm_error_m", xy_only_max_lever_arm_error_m_);
  xy_only_max_cov_xy_ = declare_parameter<double>(
    "xy_only_recovery.max_cov_xy", xy_only_max_cov_xy_);
  xy_only_soft_bad_grace_sec_ = declare_parameter<double>(
    "xy_only_recovery.soft_bad_grace_sec", xy_only_soft_bad_grace_sec_);
  xy_only_max_target_translation_m_ = declare_parameter<double>(
    "xy_only_recovery.max_target_translation_m",
    xy_only_max_target_translation_m_);
  xy_only_max_target_refresh_translation_m_ = declare_parameter<double>(
    "xy_only_recovery.max_target_refresh_translation_m",
    xy_only_max_target_refresh_translation_m_);
  xy_only_exit_position_m_ = declare_parameter<double>(
    "xy_only_recovery.exit_position_m", xy_only_exit_position_m_);
  const int xy_only_exit_min_samples = declare_parameter<int>(
    "xy_only_recovery.exit_min_samples",
    static_cast<int>(xy_only_exit_min_samples_));
  xy_only_exit_min_samples_ = xy_only_exit_min_samples > 0 ?
    static_cast<std::size_t>(xy_only_exit_min_samples) : 0U;

  // These names existed in the experimental implementation. They are declared
  // as no-ops so old launch files fail gracefully instead of restoring the
  // unsafe single-fix force-accept behavior.
  const double deprecated_dead_reckoning_timeout = declare_parameter<double>(
    "time_out_dead_reckoning", 0.0);
  const bool deprecated_force_float = declare_parameter<bool>(
    "gnss_force_accept_allow_float", false);
  const double deprecated_force_cov = declare_parameter<double>(
    "gnss_force_accept_max_cov_xy", 0.0);
  declare_parameter<std::vector<double>>(
    "gnss_confidence_alpha_table", std::vector<double>{});
  if (deprecated_dead_reckoning_timeout > 0.0 || deprecated_force_float ||
    deprecated_force_cov > 0.0)
  {
    RCLCPP_WARN(
      get_logger(),
      "Deprecated forced-GNSS-accept parameters are ignored. Configure the "
      "recovery_* consistency and correction limits instead.");
  }
}

void MapOdomFusionNode::validateParameters() const
{
  const bool valid =
    !map_frame_.empty() && !odom_frame_.empty() && !base_frame_.empty() &&
    !odom_topic_.empty() && publish_rate_hz_ > 0.0 && heartbeat_hz_ > 0.0 &&
    odom_buffer_sec_ > 0.0 && odom_timeout_sec_ > 0.0 &&
    odom_max_extrapolate_sec_ >= 0.0 && measurement_max_age_sec_ >= 0.0 &&
    measurement_future_tolerance_sec_ >= 0.0 &&
    measurement_reorder_tolerance_sec_ >= 0.0 && anchor_cov_xy_default_ > 0.0 &&
    anchor_cov_yaw_default_ > 0.0 && cov_xy_drift_per_sec_ >= 0.0 &&
    cov_yaw_drift_per_sec_ >= 0.0 && no_fix_cov_drift_scale_ >= 1.0 &&
    gnss_status_timeout_sec_ > 0.0 &&
    initialization_min_samples_ > 0U && initialization_min_heading_samples_ > 0U &&
    gnss_max_cov_xy_ > 0.0 && gnss_max_cov_yaw_ > 0.0 &&
    gnss_position_nis_threshold_ > 0.0 && gnss_yaw_nis_threshold_ > 0.0 &&
    gnss_position_jump_reject_m_ > 0.0 && gnss_yaw_jump_reject_rad_ > 0.0 &&
    gnss_jump_reject_sigma_scale_ >= 0.0 && tracking_rejection_min_samples_ > 0U &&
    gnss_outage_timeout_sec_ > 0.0 &&
    recovery_alignment_config_.min_samples > 0U &&
    recovery_alignment_config_.min_heading_samples > 0U &&
    recovery_alignment_config_.max_sample_gap_sec > 0.0 &&
    recovery_alignment_config_.min_odom_baseline_m >= 0.0 &&
    recovery_alignment_config_.max_position_rms_m > 0.0 &&
    recovery_alignment_config_.max_position_residual_m > 0.0 &&
    recovery_alignment_config_.max_heading_std_rad > 0.0 &&
    recovery_alignment_config_.max_heading_source_disagreement_rad > 0.0 &&
    recovery_candidate_window_sec_ > 0.0 && recovery_candidate_max_samples_ > 0U &&
    recovery_max_target_translation_m_ > 0.0 && recovery_max_target_yaw_rad_ > 0.0 &&
    recovery_max_target_refresh_translation_m_ > 0.0 &&
    recovery_max_target_refresh_yaw_rad_ > 0.0 &&
    recovery_max_correction_per_update_m_ > 0.0 &&
    recovery_max_correction_per_sec_m_ > 0.0 &&
    recovery_max_yaw_correction_per_update_rad_ > 0.0 &&
    recovery_max_yaw_correction_per_sec_rad_ > 0.0 &&
    recovery_exit_position_m_ >= 0.0 && recovery_exit_yaw_rad_ >= 0.0 &&
    recovery_exit_min_samples_ > 0U &&
    xy_only_required_fix_quality_ > 0 &&
    xy_only_alignment_config_.min_samples > 0U &&
    xy_only_alignment_config_.max_sample_gap_sec > 0.0 &&
    xy_only_alignment_config_.max_position_rms_m > 0.0 &&
    xy_only_alignment_config_.max_position_residual_m > 0.0 &&
    xy_only_min_candidate_span_sec_ > 0.0 &&
    xy_only_min_candidate_span_sec_ <= recovery_candidate_window_sec_ &&
    xy_only_stationary_dwell_sec_ > 0.0 &&
    xy_only_stationary_dwell_sec_ <= odom_buffer_sec_ &&
    xy_only_max_speed_mps_ >= 0.0 && xy_only_max_yaw_rate_radps_ >= 0.0 &&
    xy_only_max_stationary_displacement_m_ >= 0.0 &&
    xy_only_max_odom_yaw_change_rad_ >= 0.0 && xy_only_max_odom_gap_sec_ > 0.0 &&
    xy_only_max_anchor_yaw_variance_rad2_ > 0.0 &&
    xy_only_max_outage_sec_ > 0.0 && xy_only_max_lever_arm_error_m_ >= 0.0 &&
    xy_only_max_cov_xy_ > 0.0 && xy_only_soft_bad_grace_sec_ >= 0.0 &&
    xy_only_max_target_translation_m_ > 0.0 &&
    xy_only_max_target_refresh_translation_m_ > 0.0 &&
    xy_only_exit_position_m_ >= 0.0 && xy_only_exit_min_samples_ > 0U;
  if (!valid) {
    throw std::invalid_argument("invalid pure_gnss_map_odom_fusion parameter");
  }
}

std::optional<MapOdomFusionNode::OdomSample> MapOdomFusionNode::interpolateOdom(
  const rclcpp::Time & stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (odom_buffer_.empty()) {
    return std::nullopt;
  }

  const double before_first = (odom_buffer_.front().stamp - stamp).seconds();
  if (before_first > 0.0) {
    if (before_first > odom_max_extrapolate_sec_) {
      return std::nullopt;
    }
    OdomSample output = odom_buffer_.front();
    output.stamp = stamp;
    output.T_odom_base = odom_buffer_.front().T_odom_base *
      deltaFromTwist(odom_buffer_.front().twist, -before_first);
    return output;
  }

  for (std::size_t index = 1; index < odom_buffer_.size(); ++index) {
    const OdomSample & previous = odom_buffer_[index - 1U];
    const OdomSample & next = odom_buffer_[index];
    if (previous.stamp <= stamp && stamp <= next.stamp) {
      const double interval = (next.stamp - previous.stamp).seconds();
      const double ratio = interval > 1.0e-9 ?
        std::clamp((stamp - previous.stamp).seconds() / interval, 0.0, 1.0) : 0.0;

      OdomSample output = previous;
      output.stamp = stamp;
      output.T_odom_base.translation() =
        (1.0 - ratio) * previous.T_odom_base.translation() +
        ratio * next.T_odom_base.translation();
      const double previous_yaw = yawFromIso(previous.T_odom_base);
      const double next_yaw = yawFromIso(next.T_odom_base);
      const double interpolated_yaw = wrapAngle(
        previous_yaw + ratio * wrapAngle(next_yaw - previous_yaw));
      output.T_odom_base.linear() = quatFromYaw(interpolated_yaw).toRotationMatrix();
      output.twist = ratio < 0.5 ? previous.twist : next.twist;
      output.twist_covariance = ratio < 0.5 ?
        previous.twist_covariance : next.twist_covariance;
      output.cov_xy_total =
        (1.0 - ratio) * previous.cov_xy_total + ratio * next.cov_xy_total;
      output.cov_yaw_total =
        (1.0 - ratio) * previous.cov_yaw_total + ratio * next.cov_yaw_total;
      return output;
    }
  }

  const OdomSample & latest = odom_buffer_.back();
  const double forward = (stamp - latest.stamp).seconds();
  if (forward < 0.0 || forward > odom_max_extrapolate_sec_) {
    return std::nullopt;
  }
  OdomSample output = latest;
  output.stamp = stamp;
  output.T_odom_base = latest.T_odom_base * deltaFromTwist(latest.twist, forward);
  return output;
}

MapOdomFusionNode::AbsolutePoseMeasurement
MapOdomFusionNode::measurementFromGnssInput(
  const pure_gnss_msgs::msg::GnssFusionInput & message) const
{
  AbsolutePoseMeasurement measurement;
  measurement.stamp = rclcpp::Time(message.header.stamp);

  measurement.T_map_observation = poseMsgToIso(message.odom.pose.pose);
  measurement.source = "gnss";
  measurement.has_gnss_fix_status = true;
  measurement.gnss_fix_status = static_cast<int>(message.fix_status);
  measurement.fix_quality = static_cast<int>(message.fix_quality);
  measurement.has_confidence = message.has_confidence;
  measurement.confidence = message.confidence;
  measurement.gnss_usable = message.gnss_usable;
  measurement.heading_source =
    message.heading_source.empty() ? "none" : message.heading_source;
  measurement.position_is_base_link = message.position_is_base_link;
  measurement.observation_point_valid = message.observation_point_valid;
  measurement.observation_point_in_base = Eigen::Vector3d(
    message.observation_point_in_base.x,
    message.observation_point_in_base.y,
    message.observation_point_in_base.z);
  if (!measurement.observation_point_in_base.allFinite()) {
    measurement.observation_point_valid = false;
    measurement.observation_point_in_base.setZero();
  }

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance(0, 0) = message.odom.pose.covariance[0];
  covariance(0, 1) = message.odom.pose.covariance[1];
  covariance(0, 2) = message.odom.pose.covariance[5];
  covariance(1, 0) = message.odom.pose.covariance[6];
  covariance(1, 1) = message.odom.pose.covariance[7];
  covariance(1, 2) = message.odom.pose.covariance[11];
  covariance(2, 0) = message.odom.pose.covariance[30];
  covariance(2, 1) = message.odom.pose.covariance[31];
  covariance(2, 2) = message.odom.pose.covariance[35];

  measurement.yaw_valid = message.heading_valid &&
    quaternionIsValid(message.odom.pose.pose.orientation) &&
    std::isfinite(covariance(2, 2)) && covariance(2, 2) > 0.0 &&
    covariance(2, 2) <= gnss_max_cov_yaw_;
  measurement.R_xy_yaw = sanitizeMeasurementCovariance(
    covariance, kUnknownVariance, kUnknownVariance, measurement.yaw_valid);
  measurement.cov_xy = 0.5 * (
    measurement.R_xy_yaw(0, 0) + measurement.R_xy_yaw(1, 1));
  measurement.cov_yaw = measurement.R_xy_yaw(2, 2);
  return measurement;
}

MapOdomFusionNode::AbsolutePoseMeasurement MapOdomFusionNode::measurementFromPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & message,
  const std::string & source) const
{
  AbsolutePoseMeasurement measurement;
  measurement.stamp = rclcpp::Time(message.header.stamp);
  if (measurement.stamp.nanoseconds() == 0) {
    measurement.stamp = now();
  }
  measurement.T_map_observation = poseMsgToIso(message.pose.pose);
  measurement.source = source;
  measurement.position_is_base_link = true;
  measurement.yaw_valid = quaternionIsValid(message.pose.pose.orientation);
  measurement.heading_source = source;

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance(0, 0) = message.pose.covariance[0];
  covariance(0, 1) = message.pose.covariance[1];
  covariance(0, 2) = message.pose.covariance[5];
  covariance(1, 0) = message.pose.covariance[6];
  covariance(1, 1) = message.pose.covariance[7];
  covariance(1, 2) = message.pose.covariance[11];
  covariance(2, 0) = message.pose.covariance[30];
  covariance(2, 1) = message.pose.covariance[31];
  covariance(2, 2) = message.pose.covariance[35];
  measurement.R_xy_yaw = sanitizeMeasurementCovariance(
    covariance, anchor_cov_xy_default_, anchor_cov_yaw_default_, measurement.yaw_valid);
  measurement.cov_xy = 0.5 * (
    measurement.R_xy_yaw(0, 0) + measurement.R_xy_yaw(1, 1));
  measurement.cov_yaw = measurement.R_xy_yaw(2, 2);
  return measurement;
}

bool MapOdomFusionNode::validateMeasurement(
  const AbsolutePoseMeasurement & measurement,
  std::string & reason) const
{
  if (measurement.stamp.nanoseconds() == 0) {
    reason = "zero_measurement_stamp";
    return false;
  }
  if (!isFiniteTransform(measurement.T_map_observation) ||
    !measurement.R_xy_yaw.allFinite())
  {
    reason = "non_finite_measurement";
    return false;
  }
  if (!measurement.position_is_base_link &&
    !measurement.observation_point_valid &&
    !gnss_allow_unknown_observation_point_)
  {
    reason = "observation_point_unknown";
    return false;
  }
  if (!std::isfinite(measurement.cov_xy) || measurement.cov_xy <= 0.0) {
    reason = "invalid_position_covariance";
    return false;
  }

  const rclcpp::Time current = now();
  if (current.nanoseconds() != 0 && current.get_clock_type() == measurement.stamp.get_clock_type()) {
    const double age = (current - measurement.stamp).seconds();
    if (measurement_max_age_sec_ > 0.0 && age > measurement_max_age_sec_) {
      reason = "stale_measurement";
      return false;
    }
    if (age < -measurement_future_tolerance_sec_) {
      reason = "future_measurement";
      return false;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_measurement_stamp_.nanoseconds() != 0 &&
      (last_measurement_stamp_ - measurement.stamp).seconds() >
      measurement_reorder_tolerance_sec_)
    {
      reason = "out_of_order_measurement";
      return false;
    }
  }
  reason = "ok";
  return true;
}

MapOdomFusionNode::GnssFixState MapOdomFusionNode::gnssFixState(
  const AbsolutePoseMeasurement & measurement) const
{
  if (measurement.source != "gnss") {
    return GnssFixState::GOOD;
  }
  if (!measurement.gnss_usable) {
    return GnssFixState::BAD;
  }
  if (!use_gnss_status_) {
    return GnssFixState::GOOD;
  }
  if (measurement.has_gnss_fix_status) {
    return measurement.gnss_fix_status >= gnss_fix_min_status_ ?
      GnssFixState::GOOD : GnssFixState::BAD;
  }
  return liveGnssFixState(measurement.stamp);
}

MapOdomFusionNode::GnssFixState MapOdomFusionNode::liveGnssFixState(
  const rclcpp::Time & stamp) const
{
  if (!use_gnss_status_) {
    return GnssFixState::GOOD;
  }

  GnssStatusState status;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status = gnss_status_;
  }
  if (!status.valid) {
    return GnssFixState::UNKNOWN;
  }
  if (std::fabs((stamp - status.stamp).seconds()) > gnss_status_timeout_sec_) {
    return GnssFixState::BAD;
  }
  return status.status >= gnss_fix_min_status_ ?
    GnssFixState::GOOD : GnssFixState::BAD;
}

bool MapOdomFusionNode::isHardGnssFailure(
  const AbsolutePoseMeasurement & measurement) const
{
  if (measurement.source != "gnss") {
    return false;
  }
  if (measurement.fix_quality <= 0) {
    return true;
  }
  return use_gnss_status_ && measurement.has_gnss_fix_status &&
         measurement.gnss_fix_status < gnss_fix_min_status_;
}

bool MapOdomFusionNode::tolerateSoftBadGnss(
  const AbsolutePoseMeasurement & measurement)
{
  if (!xy_only_recovery_enabled_ || xy_only_soft_bad_grace_sec_ <= 0.0 ||
    isHardGnssFailure(measurement))
  {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!anchor_) {
    return false;
  }
  const rclcpp::Time reference = last_candidate_gnss_stamp_.nanoseconds() != 0 ?
    last_candidate_gnss_stamp_ : last_good_gnss_stamp_;
  if (reference.nanoseconds() == 0 || measurement.stamp < reference ||
    (measurement.stamp - reference).seconds() > xy_only_soft_bad_grace_sec_)
  {
    return false;
  }

  // A fix still exists, but confidence temporarily fell below the usable
  // threshold. Skip this sample without erasing an otherwise consistent RTK
  // recovery window. A true NO_FIX remains an immediate outage.
  last_fix_state_ = GnssFixState::BAD;
  last_measurement_source_ = measurement.source;
  last_measurement_cov_xy_ = measurement.cov_xy;
  last_measurement_cov_yaw_ = measurement.cov_yaw;
  last_rejection_reason_ = "gnss_soft_bad_within_grace";
  last_recovery_reason_ = "holding_recovery_window_during_soft_bad";
  return true;
}

void MapOdomFusionNode::handleMeasurement(
  const AbsolutePoseMeasurement & measurement,
  bool allow_pending)
{
  std::string reason;
  if (!validateMeasurement(measurement, reason)) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_rejection_reason_ = reason;
      last_measurement_source_ = measurement.source;
    }
    if (measurement.source == "gnss" &&
      (reason == "stale_measurement" || reason == "out_of_order_measurement"))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejected GNSS measurement: %s", reason.c_str());
    }
    return;
  }

  const auto odom = interpolateOdom(measurement.stamp);
  if (!odom) {
    if (allow_pending) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pending_measurement_ || measurement.stamp >= pending_measurement_->stamp) {
        pending_measurement_ = measurement;
      }
      last_rejection_reason_ = "waiting_for_matching_odom";
    }
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (measurement.stamp > last_measurement_stamp_) {
      last_measurement_stamp_ = measurement.stamp;
    }
    if (pending_measurement_ && pending_measurement_->stamp == measurement.stamp) {
      pending_measurement_.reset();
    }
  }
  applyMeasurement(measurement, *odom);
}

bool MapOdomFusionNode::resolvePendingMeasurement()
{
  std::optional<AbsolutePoseMeasurement> pending;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending = pending_measurement_;
  }
  if (!pending) {
    return false;
  }

  const auto odom = interpolateOdom(pending->stamp);
  if (!odom) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_measurement_ && pending_measurement_->stamp == pending->stamp) {
      pending_measurement_.reset();
    }
    if (pending->stamp > last_measurement_stamp_) {
      last_measurement_stamp_ = pending->stamp;
    }
  }
  applyMeasurement(*pending, *odom);
  return true;
}

bool MapOdomFusionNode::initializeManualAnchor(
  const AbsolutePoseMeasurement & measurement,
  const OdomSample & odom)
{
  if (!measurement.yaw_valid || !measurement.position_is_base_link) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rejection_reason_ = "manual_anchor_requires_base_pose_with_heading";
    return false;
  }

  const Eigen::Isometry3d map_base = measurement.T_map_observation;
  AnchorState anchor;
  anchor.stamp = measurement.stamp;
  anchor.T_map_odom = map_base * odom.T_odom_base.inverse();
  anchor.T_map_odom.linear() = quatFromYaw(yawFromIso(anchor.T_map_odom)).toRotationMatrix();
  anchor.P = sanitizeMeasurementCovariance(
    measurement.R_xy_yaw,
    anchor_cov_xy_default_,
    anchor_cov_yaw_default_,
    true);
  anchor.odom_cov_xy_ref = odom.cov_xy_total;
  anchor.odom_cov_yaw_ref = odom.cov_yaw_total;
  anchor.source = measurement.source + "_reset";

  if (!isFiniteTransform(anchor.T_map_odom) || !anchor.P.allFinite()) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rejection_reason_ = "manual_anchor_non_finite";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    anchor_ = anchor;
    pending_measurement_.reset();
    recovery_candidates_.clear();
    recovery_target_.reset();
    recovery_target_covariance_.setIdentity();
    recovery_exit_count_ = 0U;
    tracking_rejection_count_ = 0U;
    last_recovery_step_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_good_gnss_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_candidate_gnss_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    recovery_state_ = GnssRecoveryState::OUTAGE;
    last_recovery_reason_ = "manual_anchor_waiting_for_gnss_validation";
    last_rejection_reason_ = "none";
    last_measurement_source_ = measurement.source;
    last_measurement_cov_xy_ = measurement.cov_xy;
    last_measurement_cov_yaw_ = measurement.cov_yaw;
    last_fix_state_ = GnssFixState::GOOD;
    last_gain_xy_ = 1.0;
    last_gain_yaw_ = 1.0;
    last_correction_xy_m_ = 0.0;
    last_correction_yaw_rad_ = 0.0;
  }
  return true;
}

bool MapOdomFusionNode::initializeFromCandidates(
  const AbsolutePoseMeasurement & latest_measurement,
  const OdomSample & latest_odom)
{
  RecoveryAlignmentConfig config = recovery_alignment_config_;
  config.min_samples = initialization_min_samples_;
  config.min_heading_samples = initialization_min_heading_samples_;

  std::vector<RecoveryAlignmentSample> candidates;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    candidates = recovery_candidates_;
  }
  const RecoveryAlignmentResult result = estimateRecoveryAlignmentRobust(candidates, config);

  if (!result.valid) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_alignment_result_ = result;
    last_recovery_reason_ = "initialization_" + result.reason;
    last_rejection_reason_ = "initialization_waiting_for_consistent_window";
    return false;
  }
  if (newestSampleWasRejected(result)) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_alignment_result_ = result;
    last_recovery_reason_ = "initialization_latest_sample_rejected";
    last_rejection_reason_ = "initialization_waiting_for_newest_inlier";
    return false;
  }

  AnchorState anchor;
  anchor.stamp = latest_measurement.stamp;
  anchor.T_map_odom = transformFromAlignment(result);
  const Eigen::Vector3d latest_observation_in_odom =
    observationPointInOdom(latest_odom, latest_measurement);
  anchor.T_map_odom.translation().z() =
    latest_measurement.T_map_observation.translation().z() -
    latest_observation_in_odom.z();
  anchor.T_map_odom.linear() = quatFromYaw(yawFromIso(anchor.T_map_odom)).toRotationMatrix();

  const double position_variance = std::max(
    {anchor_cov_xy_default_, latest_measurement.cov_xy,
      result.position_rms_m * result.position_rms_m}) +
    std::max(0.0, latest_odom.cov_xy_total);

  double yaw_sigma = std::sqrt(anchor_cov_yaw_default_);
  if (latest_measurement.yaw_valid && std::isfinite(latest_measurement.cov_yaw)) {
    yaw_sigma = std::max(yaw_sigma, std::sqrt(latest_measurement.cov_yaw));
  }
  if (result.used_direct_heading && std::isfinite(result.heading_std_rad)) {
    yaw_sigma = std::max(yaw_sigma, result.heading_std_rad);
  }
  if (result.used_position_heading) {
    const double geometric_sigma = std::atan2(
      std::max(result.position_rms_m, std::sqrt(std::max(0.0, latest_measurement.cov_xy))),
      std::max(1.0e-3, result.odom_baseline_m));
    yaw_sigma = std::max(yaw_sigma, geometric_sigma);
  }
  const double yaw_variance = yaw_sigma * yaw_sigma +
    std::max(0.0, latest_odom.cov_yaw_total);

  anchor.P = Eigen::Matrix3d::Zero();
  anchor.P(0, 0) = position_variance;
  anchor.P(1, 1) = position_variance;
  anchor.P(2, 2) = std::max(anchor_cov_yaw_default_, yaw_variance);
  anchor.odom_cov_xy_ref = latest_odom.cov_xy_total;
  anchor.odom_cov_yaw_ref = latest_odom.cov_yaw_total;
  anchor.source = "gnss_consistent_window_init";

  if (!isFiniteTransform(anchor.T_map_odom) || !anchor.P.allFinite()) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_alignment_result_ = result;
    last_rejection_reason_ = "initialization_non_finite";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    anchor_ = anchor;
    pending_measurement_.reset();
    recovery_candidates_.clear();
    recovery_target_.reset();
    recovery_target_covariance_.setIdentity();
    recovery_exit_count_ = 0U;
    last_recovery_step_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    tracking_rejection_count_ = 0U;
    recovery_state_ = GnssRecoveryState::TRACKING;
    last_good_gnss_stamp_ = latest_measurement.stamp;
    last_candidate_gnss_stamp_ = latest_measurement.stamp;
    last_alignment_result_ = result;
    last_recovery_reason_ = "initialized_from_consistent_gnss_window";
    last_rejection_reason_ = "none";
    last_measurement_source_ = latest_measurement.source;
    last_measurement_cov_xy_ = latest_measurement.cov_xy;
    last_measurement_cov_yaw_ = latest_measurement.cov_yaw;
    last_fix_state_ = GnssFixState::GOOD;
    last_gain_xy_ = 1.0;
    last_gain_yaw_ = result.used_direct_heading || result.used_position_heading ? 1.0 : 0.0;
    last_correction_xy_m_ = 0.0;
    last_correction_yaw_rad_ = 0.0;
  }
  return true;
}

bool MapOdomFusionNode::normalEkfUpdate(
  const AbsolutePoseMeasurement & measurement,
  const OdomSample & odom,
  GnssFixState fix_state)
{
  AnchorState current_anchor;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!anchor_) {
      return false;
    }
    current_anchor = *anchor_;
  }

  if (measurement.cov_xy > gnss_max_cov_xy_) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rejection_reason_ = "position_covariance_gate";
    return false;
  }

  bool use_yaw = measurement.yaw_valid && measurement.cov_yaw <= gnss_max_cov_yaw_;
  bool yaw_component_rejected = false;
  const Eigen::Vector3d point_odom = observationPointInOdom(odom, measurement);
  const double anchor_yaw = yawFromIso(current_anchor.T_map_odom);
  const double odom_yaw = yawFromIso(odom.T_odom_base);
  const Eigen::Rotation2Dd map_odom_rotation(anchor_yaw);
  const Eigen::Vector2d predicted_position =
    current_anchor.T_map_odom.translation().head<2>() +
    map_odom_rotation * point_odom.head<2>();
  const double predicted_yaw = wrapAngle(anchor_yaw + odom_yaw);

  Eigen::Vector3d innovation = Eigen::Vector3d::Zero();
  innovation.head<2>() =
    measurement.T_map_observation.translation().head<2>() - predicted_position;
  if (use_yaw) {
    innovation.z() = wrapAngle(yawFromIso(measurement.T_map_observation) - predicted_yaw);
  }

  Eigen::Matrix3d observation_jacobian = Eigen::Matrix3d::Zero();
  observation_jacobian(0, 0) = 1.0;
  observation_jacobian(1, 1) = 1.0;
  observation_jacobian(0, 2) =
    -point_odom.x() * std::sin(anchor_yaw) -
    point_odom.y() * std::cos(anchor_yaw);
  observation_jacobian(1, 2) =
    point_odom.x() * std::cos(anchor_yaw) -
    point_odom.y() * std::sin(anchor_yaw);
  if (use_yaw) {
    observation_jacobian(2, 2) = 1.0;
  }

  const double dt = std::max(0.0, (measurement.stamp - current_anchor.stamp).seconds());
  const double odom_cov_xy_since = std::max(
    0.0, odom.cov_xy_total - current_anchor.odom_cov_xy_ref);
  const double odom_cov_yaw_since = std::max(
    0.0, odom.cov_yaw_total - current_anchor.odom_cov_yaw_ref);
  Eigen::Matrix3d predicted_covariance = current_anchor.P;
  predicted_covariance(0, 0) += cov_xy_drift_per_sec_ * dt + odom_cov_xy_since;
  predicted_covariance(1, 1) += cov_xy_drift_per_sec_ * dt + odom_cov_xy_since;
  predicted_covariance(2, 2) += cov_yaw_drift_per_sec_ * dt + odom_cov_yaw_since;
  predicted_covariance = 0.5 * (predicted_covariance + predicted_covariance.transpose());

  Eigen::Matrix3d measurement_covariance = sanitizeMeasurementCovariance(
    measurement.R_xy_yaw, gnss_max_cov_xy_, gnss_max_cov_yaw_, use_yaw);
  if (!use_yaw) {
    observation_jacobian.row(2).setZero();
    measurement_covariance.row(2).setZero();
    measurement_covariance.col(2).setZero();
    measurement_covariance(2, 2) = kDisabledVariance;
  }

  Eigen::Matrix3d innovation_covariance =
    observation_jacobian * predicted_covariance * observation_jacobian.transpose() +
    measurement_covariance;
  innovation_covariance = 0.5 * (
    innovation_covariance + innovation_covariance.transpose());

  const Eigen::Matrix2d position_covariance = innovation_covariance.topLeftCorner<2, 2>();
  Eigen::LDLT<Eigen::Matrix2d> position_ldlt(position_covariance);
  if (!fixedLdltIsPositive(position_ldlt)) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rejection_reason_ = "invalid_position_innovation_covariance";
    return false;
  }
  const double position_nis = innovation.head<2>().dot(
    position_ldlt.solve(innovation.head<2>()));
  const double position_sigma = std::sqrt(std::max(
    0.0, 0.5 * (position_covariance(0, 0) + position_covariance(1, 1))));
  const double position_fixed_gate = gnss_jump_reject_sigma_scale_ > 0.0 ?
    std::max(
    gnss_position_jump_reject_m_, gnss_jump_reject_sigma_scale_ * position_sigma) :
    gnss_position_jump_reject_m_;
  if (!std::isfinite(position_nis) || position_nis > gnss_position_nis_threshold_ ||
    innovation.head<2>().norm() > position_fixed_gate)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rejection_reason_ = "position_innovation_gate";
    last_innovation_xy_m_ = innovation.head<2>().norm();
    last_innovation_yaw_rad_ = use_yaw ? std::fabs(innovation.z()) : 0.0;
    return false;
  }

  if (use_yaw) {
    const double yaw_variance = innovation_covariance(2, 2);
    const double yaw_nis = yaw_variance > kMinimumVariance ?
      innovation.z() * innovation.z() / yaw_variance :
      std::numeric_limits<double>::infinity();
    const double yaw_sigma = std::sqrt(std::max(0.0, yaw_variance));
    const double yaw_fixed_gate = gnss_jump_reject_sigma_scale_ > 0.0 ?
      std::max(
      gnss_yaw_jump_reject_rad_, gnss_jump_reject_sigma_scale_ * yaw_sigma) :
      gnss_yaw_jump_reject_rad_;
    if (!std::isfinite(yaw_nis) || yaw_nis > gnss_yaw_nis_threshold_ ||
      std::fabs(innovation.z()) > yaw_fixed_gate)
    {
      // A bad heading estimate must not discard a valid position fix.
      yaw_component_rejected = true;
      use_yaw = false;
      innovation.z() = 0.0;
      observation_jacobian.row(2).setZero();
      measurement_covariance.row(2).setZero();
      measurement_covariance.col(2).setZero();
      measurement_covariance(2, 2) = kDisabledVariance;
      innovation_covariance =
        observation_jacobian * predicted_covariance * observation_jacobian.transpose() +
        measurement_covariance;
      innovation_covariance = 0.5 * (
        innovation_covariance + innovation_covariance.transpose());
    }
  }

  Eigen::LDLT<Eigen::Matrix3d> ldlt(innovation_covariance);
  if (!fixedLdltIsPositive(ldlt)) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rejection_reason_ = "invalid_innovation_covariance";
    return false;
  }

  const Eigen::Matrix3d kalman_gain = predicted_covariance *
    observation_jacobian.transpose() * ldlt.solve(Eigen::Matrix3d::Identity());
  const Eigen::Vector3d correction = kalman_gain * innovation;
  if (!correction.allFinite()) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rejection_reason_ = "non_finite_kalman_correction";
    return false;
  }

  AnchorState updated = current_anchor;
  updated.stamp = measurement.stamp;
  updated.T_map_odom.translation().x() += correction.x();
  updated.T_map_odom.translation().y() += correction.y();
  updated.T_map_odom.linear() = quatFromYaw(
    wrapAngle(anchor_yaw + correction.z())).toRotationMatrix();
  const Eigen::Matrix3d identity_minus_gain =
    Eigen::Matrix3d::Identity() - kalman_gain * observation_jacobian;
  updated.P = identity_minus_gain * predicted_covariance *
    identity_minus_gain.transpose() +
    kalman_gain * measurement_covariance * kalman_gain.transpose();
  updated.P = 0.5 * (updated.P + updated.P.transpose());
  if (!updated.P.allFinite()) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rejection_reason_ = "non_finite_posterior_covariance";
    return false;
  }
  updated.odom_cov_xy_ref = odom.cov_xy_total;
  updated.odom_cov_yaw_ref = odom.cov_yaw_total;
  updated.yaw_unobserved = false;
  updated.source = measurement.source + "_tracking";

  {
    std::lock_guard<std::mutex> lock(mutex_);
    anchor_ = updated;
    recovery_state_ = GnssRecoveryState::TRACKING;
    recovery_candidates_.clear();
    recovery_target_.reset();
    recovery_exit_count_ = 0U;
    tracking_rejection_count_ = 0U;
    last_good_gnss_stamp_ = measurement.stamp;
    last_candidate_gnss_stamp_ = measurement.stamp;
    last_recovery_reason_ = "tracking_update";
    last_rejection_reason_ = yaw_component_rejected ?
      "yaw_component_rejected_position_retained" : "none";
    last_measurement_source_ = measurement.source;
    last_measurement_cov_xy_ = measurement.cov_xy;
    last_measurement_cov_yaw_ = measurement.cov_yaw;
    last_fix_state_ = fix_state;
    last_innovation_xy_m_ = innovation.head<2>().norm();
    last_innovation_yaw_rad_ = use_yaw ? std::fabs(innovation.z()) : 0.0;
    last_correction_xy_m_ = correction.head<2>().norm();
    last_correction_yaw_rad_ = std::fabs(correction.z());
    last_gain_xy_ = 0.5 * (
      std::fabs(kalman_gain(0, 0)) + std::fabs(kalman_gain(1, 1)));
    last_gain_yaw_ = use_yaw ? std::fabs(kalman_gain(2, 2)) : 0.0;
  }
  return true;
}

void MapOdomFusionNode::enterOutage(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  recovery_state_ = anchor_ ? GnssRecoveryState::OUTAGE : GnssRecoveryState::UNINITIALIZED;
  recovery_candidates_.clear();
  recovery_target_.reset();
  recovery_target_covariance_.setIdentity();
  last_xy_only_alignment_result_ = FixedYawTranslationResult{};
  recovery_exit_count_ = 0U;
  tracking_rejection_count_ = 0U;
  last_candidate_gnss_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_recovery_step_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_recovery_reason_ = reason;
}

void MapOdomFusionNode::beginReacquisition(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!anchor_) {
    recovery_state_ = GnssRecoveryState::UNINITIALIZED;
    return;
  }
  recovery_state_ = GnssRecoveryState::REACQUIRING;
  recovery_candidates_.clear();
  recovery_target_.reset();
  recovery_target_covariance_.setIdentity();
  last_xy_only_alignment_result_ = FixedYawTranslationResult{};
  recovery_exit_count_ = 0U;
  tracking_rejection_count_ = 0U;
  last_candidate_gnss_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_recovery_step_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_recovery_reason_ = reason;
}

void MapOdomFusionNode::appendRecoveryCandidate(
  const AbsolutePoseMeasurement & measurement,
  const OdomSample & odom)
{
  RecoveryAlignmentSample sample;
  sample.stamp_sec = static_cast<double>(measurement.stamp.nanoseconds()) * 1.0e-9;
  const Eigen::Vector3d observation_point_odom = observationPointInOdom(odom, measurement);
  sample.odom_x_m = observation_point_odom.x();
  sample.odom_y_m = observation_point_odom.y();
  sample.map_x_m = measurement.T_map_observation.translation().x();
  sample.map_y_m = measurement.T_map_observation.translation().y();
  sample.position_weight = 1.0 / std::max(kMinimumVariance, measurement.cov_xy);
  sample.has_heading = measurement.yaw_valid && measurement.cov_yaw <= gnss_max_cov_yaw_;
  if (sample.has_heading) {
    sample.map_odom_yaw_rad = wrapAngle(
      yawFromIso(measurement.T_map_observation) - yawFromIso(odom.T_odom_base));
    sample.heading_weight = 1.0 / std::max(kMinimumVariance, measurement.cov_yaw);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!recovery_candidates_.empty()) {
    const double gap = sample.stamp_sec - recovery_candidates_.back().stamp_sec;
    if (gap <= 0.0) {
      last_recovery_reason_ = "reacquisition_non_monotonic_sample";
      return;
    }
    if (gap > recovery_alignment_config_.max_sample_gap_sec) {
      recovery_candidates_.clear();
      recovery_target_.reset();
      recovery_target_covariance_.setIdentity();
      recovery_exit_count_ = 0U;
      last_recovery_reason_ = "reacquisition_reset_after_sample_gap";
    }
  }
  recovery_candidates_.push_back(sample);
  while (!recovery_candidates_.empty() &&
    sample.stamp_sec - recovery_candidates_.front().stamp_sec >
    recovery_candidate_window_sec_)
  {
    recovery_candidates_.erase(recovery_candidates_.begin());
  }
  if (recovery_candidates_.size() > recovery_candidate_max_samples_) {
    recovery_candidates_.erase(
      recovery_candidates_.begin(),
      recovery_candidates_.begin() + static_cast<std::ptrdiff_t>(
        recovery_candidates_.size() - recovery_candidate_max_samples_));
  }
  last_candidate_gnss_stamp_ = measurement.stamp;
  last_measurement_cov_xy_ = measurement.cov_xy;
  last_measurement_cov_yaw_ = measurement.cov_yaw;
  last_measurement_source_ = measurement.source;
  last_fix_state_ = GnssFixState::GOOD;
}

RecoveryAlignmentResult MapOdomFusionNode::estimateRecoveryTarget() const
{
  std::vector<RecoveryAlignmentSample> candidates;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    candidates = recovery_candidates_;
  }
  return estimateRecoveryAlignmentRobust(candidates, recovery_alignment_config_);
}

FixedYawTranslationResult MapOdomFusionNode::estimateXyOnlyRecoveryTarget(
  double fixed_yaw_rad) const
{
  std::vector<RecoveryAlignmentSample> candidates;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    candidates = recovery_candidates_;
  }
  return estimateFixedYawTranslationRobust(
    candidates, fixed_yaw_rad, xy_only_alignment_config_);
}

bool MapOdomFusionNode::xyOnlyMotionIsSafe(
  const rclcpp::Time & stamp,
  std::string & reason) const
{
  struct MotionPoint
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Eigen::Vector2d position{Eigen::Vector2d::Zero()};
    double yaw{0.0};
    double speed{0.0};
    double yaw_rate{0.0};
  };

  std::vector<MotionPoint> points;
  const rclcpp::Time window_start = stamp - rclcpp::Duration::from_seconds(
    xy_only_stationary_dwell_sec_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (odom_buffer_.empty() || odom_buffer_.front().stamp > window_start ||
      odom_buffer_.back().stamp < stamp - rclcpp::Duration::from_seconds(
        xy_only_max_odom_gap_sec_))
    {
      reason = "odom_window_not_covered";
      return false;
    }

    std::optional<OdomSample> before_start;
    for (const auto & sample : odom_buffer_) {
      if (sample.stamp <= window_start) {
        before_start = sample;
        continue;
      }
      if (sample.stamp > stamp) {
        break;
      }
      if (before_start && points.empty()) {
        const auto & first = *before_start;
        points.push_back(MotionPoint{
          first.stamp,
          first.T_odom_base.translation().head<2>(),
          yawFromIso(first.T_odom_base),
          std::hypot(first.twist.linear.x, first.twist.linear.y),
          std::fabs(first.twist.angular.z)});
      }
      points.push_back(MotionPoint{
        sample.stamp,
        sample.T_odom_base.translation().head<2>(),
        yawFromIso(sample.T_odom_base),
        std::hypot(sample.twist.linear.x, sample.twist.linear.y),
        std::fabs(sample.twist.angular.z)});
    }
  }

  const double query_stamp_sec = static_cast<double>(stamp.nanoseconds()) * 1.0e-9;
  const double latest_motion_stamp_sec = points.empty() ?
    -std::numeric_limits<double>::infinity() :
    static_cast<double>(points.back().stamp.nanoseconds()) * 1.0e-9;
  if (!latestPastSampleCoversTimestamp(
      query_stamp_sec, latest_motion_stamp_sec, xy_only_max_odom_gap_sec_))
  {
    reason = "odom_stamp_not_covered";
    return false;
  }

  if (points.size() < 2U ||
    (points.back().stamp - points.front().stamp).seconds() <
    xy_only_stationary_dwell_sec_ - xy_only_max_odom_gap_sec_)
  {
    reason = "odom_stationary_dwell_not_met";
    return false;
  }

  double max_speed = 0.0;
  double max_yaw_rate = 0.0;
  double max_displacement = 0.0;
  double max_yaw_change = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    max_speed = std::max(max_speed, points[i].speed);
    max_yaw_rate = std::max(max_yaw_rate, points[i].yaw_rate);
    if (i > 0 &&
      (points[i].stamp - points[i - 1U].stamp).seconds() > xy_only_max_odom_gap_sec_)
    {
      reason = "odom_gap_during_stationary_dwell";
      return false;
    }
    for (std::size_t j = i + 1U; j < points.size(); ++j) {
      max_displacement = std::max(
        max_displacement, (points[j].position - points[i].position).norm());
      max_yaw_change = std::max(
        max_yaw_change, std::fabs(wrapAngle(points[j].yaw - points[i].yaw)));
    }
  }

  if (max_speed > xy_only_max_speed_mps_) {
    reason = "vehicle_speed_too_high";
    return false;
  }
  if (max_yaw_rate > xy_only_max_yaw_rate_radps_) {
    reason = "vehicle_yaw_rate_too_high";
    return false;
  }
  if (max_displacement > xy_only_max_stationary_displacement_m_) {
    reason = "vehicle_displacement_too_large";
    return false;
  }
  if (max_yaw_change > xy_only_max_odom_yaw_change_rad_) {
    reason = "vehicle_yaw_change_too_large";
    return false;
  }
  reason = "ok";
  return true;
}

bool MapOdomFusionNode::xyOnlyRecoveryIsSafe(
  const AbsolutePoseMeasurement & measurement,
  const OdomSample & odom,
  std::string & reason) const
{
  if (!xy_only_recovery_enabled_) {
    reason = "disabled";
    return false;
  }
  if (measurement.fix_quality != xy_only_required_fix_quality_) {
    reason = "fix_quality_gate";
    return false;
  }
  if (measurement.cov_xy > xy_only_max_cov_xy_) {
    reason = "position_covariance_gate";
    return false;
  }
  if (!measurement.position_is_base_link && !measurement.observation_point_valid) {
    reason = "observation_point_unknown";
    return false;
  }
  if (!xyOnlyMotionIsSafe(measurement.stamp, reason)) {
    return false;
  }

  AnchorState anchor;
  rclcpp::Time last_good_stamp(0, 0, RCL_ROS_TIME);
  double candidate_span = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!anchor_) {
      reason = "anchor_missing";
      return false;
    }
    anchor = *anchor_;
    last_good_stamp = last_good_gnss_stamp_;
    if (recovery_candidates_.size() >= 2U) {
      candidate_span = recovery_candidates_.back().stamp_sec -
        recovery_candidates_.front().stamp_sec;
    }
  }
  if (candidate_span < xy_only_min_candidate_span_sec_) {
    reason = "candidate_span_too_short";
    return false;
  }
  if (last_good_stamp.nanoseconds() == 0 || measurement.stamp < last_good_stamp ||
    (measurement.stamp - last_good_stamp).seconds() > xy_only_max_outage_sec_)
  {
    reason = "outage_too_long";
    return false;
  }

  const double elapsed = std::max(0.0, (measurement.stamp - anchor.stamp).seconds());
  const double effective_yaw_variance = anchor.P(2, 2) +
    cov_yaw_drift_per_sec_ * no_fix_cov_drift_scale_ * elapsed +
    std::max(0.0, odom.cov_yaw_total - anchor.odom_cov_yaw_ref);
  if (!std::isfinite(effective_yaw_variance) ||
    effective_yaw_variance > xy_only_max_anchor_yaw_variance_rad2_)
  {
    reason = "anchor_yaw_variance_gate";
    return false;
  }

  const double lever_arm_xy = measurement.position_is_base_link ? 0.0 :
    measurement.observation_point_in_base.head<2>().norm();
  const double lever_arm_error = lever_arm_xy * std::sqrt(
    std::max(0.0, effective_yaw_variance));
  if (!std::isfinite(lever_arm_error) ||
    lever_arm_error > xy_only_max_lever_arm_error_m_)
  {
    reason = "lever_arm_yaw_uncertainty_gate";
    return false;
  }
  reason = "ok";
  return true;
}

bool MapOdomFusionNode::startRecoveryFromCandidates(
  const AbsolutePoseMeasurement & latest_measurement,
  const OdomSample & latest_odom)
{
  const RecoveryAlignmentResult result = estimateRecoveryTarget();
  std::lock_guard<std::mutex> lock(mutex_);
  last_alignment_result_ = result;
  if (!result.valid || !anchor_) {
    last_recovery_reason_ = "reacquisition_" + result.reason;
    return false;
  }
  if (newestSampleWasRejected(result)) {
    last_recovery_reason_ = "reacquisition_latest_sample_rejected";
    last_rejection_reason_ = "reacquisition_waiting_for_newest_inlier";
    return false;
  }

  const Eigen::Isometry3d target = transformFromAlignment(result);
  // Target estimation remains synchronized to the GNSS measurement, but the
  // continuity gate protects the base pose that the next publisher will use.
  // A delayed GNSS measurement must not bound an old pivot while rotating the
  // latest odometry pose by an unbounded lever about the odom origin.
  const OdomSample & continuity_odom =
    odom_buffer_.empty() ? latest_odom : odom_buffer_.back();
  const Eigen::Vector2d odom_base =
    continuity_odom.T_odom_base.translation().head<2>();
  const BasePivotResidual target_delta = basePivotResidual(
    anchor_->T_map_odom.translation().x(),
    anchor_->T_map_odom.translation().y(),
    yawFromIso(anchor_->T_map_odom),
    target.translation().x(),
    target.translation().y(),
    yawFromIso(target),
    odom_base.x(),
    odom_base.y());
  if (!target_delta.valid) {
    last_recovery_reason_ = "reacquisition_target_non_finite_base_geometry";
    return false;
  }
  if (target_delta.position_m > recovery_max_target_translation_m_) {
    last_recovery_reason_ = "reacquisition_target_translation_gate";
    return false;
  }
  if (target_delta.yaw_rad > recovery_max_target_yaw_rad_) {
    last_recovery_reason_ = "reacquisition_target_yaw_gate";
    return false;
  }

  recovery_target_covariance_ = conservativeRecoveryTargetCovariance(
    result, latest_measurement, latest_odom, *anchor_);

  recovery_target_ = target;
  recovery_state_ = GnssRecoveryState::RECOVERING;
  recovery_exit_count_ = 0U;
  last_recovery_step_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_recovery_reason_ = "reacquisition_alignment_accepted";
  return true;
}

bool MapOdomFusionNode::startXyOnlyRecoveryFromCandidates(
  const AbsolutePoseMeasurement & latest_measurement,
  const OdomSample & latest_odom)
{
  std::string eligibility_reason;
  if (!xyOnlyRecoveryIsSafe(latest_measurement, latest_odom, eligibility_reason)) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_recovery_reason_ = "xy_only_blocked_" + eligibility_reason;
    return false;
  }

  double fixed_yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!anchor_ || last_alignment_result_.reason != "yaw_unobservable") {
      last_recovery_reason_ = "xy_only_requires_yaw_unobservable_full_alignment";
      return false;
    }
    fixed_yaw = yawFromIso(anchor_->T_map_odom);
  }
  const FixedYawTranslationResult result = estimateXyOnlyRecoveryTarget(fixed_yaw);

  std::lock_guard<std::mutex> lock(mutex_);
  last_xy_only_alignment_result_ = result;
  if (!result.valid || !anchor_) {
    last_recovery_reason_ = "xy_only_reacquisition_" + result.reason;
    return false;
  }
  if (newestSampleWasRejected(result)) {
    last_recovery_reason_ = "xy_only_latest_sample_rejected";
    last_rejection_reason_ = "xy_only_waiting_for_newest_inlier";
    return false;
  }

  Eigen::Isometry3d target = anchor_->T_map_odom;
  target.translation().x() = result.tx_m;
  target.translation().y() = result.ty_m;
  const double translation_delta = (
    target.translation().head<2>() - anchor_->T_map_odom.translation().head<2>()).norm();
  if (translation_delta > xy_only_max_target_translation_m_) {
    last_recovery_reason_ = "xy_only_target_translation_gate";
    return false;
  }

  recovery_target_covariance_ = conservativeXyOnlyTargetCovariance(
    result, latest_measurement, latest_odom, *anchor_);
  recovery_target_ = target;
  anchor_->yaw_unobserved = true;
  anchor_->xy_reference_odom_position =
    latest_odom.T_odom_base.translation().head<2>();
  recovery_state_ = GnssRecoveryState::RECOVERING_XY_ONLY;
  recovery_exit_count_ = 0U;
  last_recovery_step_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_recovery_reason_ = "xy_only_alignment_accepted";
  return true;
}

void MapOdomFusionNode::applyRecoveryStep(
  const AbsolutePoseMeasurement & measurement,
  const OdomSample & odom)
{
  // Refresh the target with the current sliding candidate window when it is
  // still self-consistent. A rejected refresh does not destroy the last valid
  // target, but is visible in diagnostics.
  const RecoveryAlignmentResult refreshed = estimateRecoveryTarget();

  std::lock_guard<std::mutex> lock(mutex_);
  if (!anchor_ || recovery_state_ != GnssRecoveryState::RECOVERING || !recovery_target_) {
    return;
  }
  if (newestSampleWasRejected(refreshed)) {
    last_alignment_result_ = refreshed;
    last_recovery_reason_ = "bounded_recovery_latest_sample_rejected";
    last_rejection_reason_ = "recovery_waiting_for_newest_inlier";
    recovery_exit_count_ = 0U;
    return;
  }

  const bool first_recovery_step = last_recovery_step_stamp_.nanoseconds() == 0;
  const double previous_step_sec = first_recovery_step ?
    0.0 : (measurement.stamp - last_recovery_step_stamp_).seconds();

  // startRecoveryFromCandidates() materializes covariance through the first
  // accepted measurement. Later steps add only their incremental drift and
  // odometry covariance. Keep this target covariance separate from the
  // temporary residual covariance published while correction is in flight.
  const RecoveryCovarianceIncrement covariance_increment =
    incrementalRecoveryCovariance(
    first_recovery_step,
    previous_step_sec,
    cov_xy_drift_per_sec_ * no_fix_cov_drift_scale_,
    cov_yaw_drift_per_sec_ * no_fix_cov_drift_scale_,
    odom.cov_xy_total,
    anchor_->odom_cov_xy_ref,
    odom.cov_yaw_total,
    anchor_->odom_cov_yaw_ref);
  recovery_target_covariance_(0, 0) += covariance_increment.xy_variance;
  recovery_target_covariance_(1, 1) += covariance_increment.xy_variance;
  recovery_target_covariance_(2, 2) += covariance_increment.yaw_variance;

  bool refresh_accepted = false;
  bool refresh_geometry_valid = true;
  if (refreshed.valid) {
    const Eigen::Isometry3d candidate_target = transformFromAlignment(refreshed);
    const OdomSample & continuity_odom =
      odom_buffer_.empty() ? odom : odom_buffer_.back();
    const Eigen::Vector2d odom_base =
      continuity_odom.T_odom_base.translation().head<2>();
    const BasePivotResidual candidate_delta = basePivotResidual(
      anchor_->T_map_odom.translation().x(),
      anchor_->T_map_odom.translation().y(),
      yawFromIso(anchor_->T_map_odom),
      candidate_target.translation().x(),
      candidate_target.translation().y(),
      yawFromIso(candidate_target),
      odom_base.x(),
      odom_base.y());
    const BasePivotResidual refresh_delta = basePivotResidual(
      recovery_target_->translation().x(),
      recovery_target_->translation().y(),
      yawFromIso(*recovery_target_),
      candidate_target.translation().x(),
      candidate_target.translation().y(),
      yawFromIso(candidate_target),
      odom_base.x(),
      odom_base.y());
    refresh_geometry_valid = candidate_delta.valid && refresh_delta.valid;
    if (refresh_geometry_valid &&
      candidate_delta.position_m <= recovery_max_target_translation_m_ &&
      candidate_delta.yaw_rad <= recovery_max_target_yaw_rad_ &&
      refresh_delta.position_m <= recovery_max_target_refresh_translation_m_ &&
      refresh_delta.yaw_rad <= recovery_max_target_refresh_yaw_rad_)
    {
      recovery_target_ = candidate_target;
      last_alignment_result_ = refreshed;

      // Evaluate only the new alignment/measurement floor here. The target
      // covariance already contains the prior outage uncertainty; using the
      // published anchor covariance would feed its temporary residual term
      // back into the target on every bounded step.
      recovery_target_covariance_ = recovery_target_covariance_.cwiseMax(
        recoveryAlignmentCovarianceFloor(refreshed, measurement));
      refresh_accepted = true;
    }
  }

  recovery_target_covariance_ = sanitizeMeasurementCovariance(
    recovery_target_covariance_, anchor_cov_xy_default_, anchor_cov_yaw_default_, true);

  const double nominal_period = 1.0 / std::max(1.0, publish_rate_hz_);
  const double step_dt = std::max(nominal_period, std::max(0.0, previous_step_sec));
  const double max_translation_step = std::min(
    recovery_max_correction_per_update_m_,
    recovery_max_correction_per_sec_m_ * step_dt);
  const double max_yaw_step = std::min(
    recovery_max_yaw_correction_per_update_rad_,
    recovery_max_yaw_correction_per_sec_rad_ * step_dt);

  const double current_yaw = yawFromIso(anchor_->T_map_odom);
  const double target_yaw = yawFromIso(*recovery_target_);
  const OdomSample & continuity_odom =
    odom_buffer_.empty() ? odom : odom_buffer_.back();
  const Eigen::Vector2d odom_base =
    continuity_odom.T_odom_base.translation().head<2>();
  const BasePivotBoundedCorrectionResult bounded = applyBasePivotBoundedCorrection(
    anchor_->T_map_odom.translation().x(),
    anchor_->T_map_odom.translation().y(),
    current_yaw,
    recovery_target_->translation().x(),
    recovery_target_->translation().y(),
    target_yaw,
    odom_base.x(),
    odom_base.y(),
    max_translation_step,
    max_yaw_step);
  if (!bounded.valid) {
    recovery_state_ = GnssRecoveryState::REACQUIRING;
    recovery_target_.reset();
    recovery_target_covariance_.setIdentity();
    recovery_exit_count_ = 0U;
    last_recovery_step_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_recovery_reason_ = "bounded_recovery_non_finite_base_geometry";
    last_rejection_reason_ = "bounded_recovery_non_finite_base_geometry";
    return;
  }

  anchor_->stamp = measurement.stamp;
  anchor_->T_map_odom.translation().x() = bounded.tx_m;
  anchor_->T_map_odom.translation().y() = bounded.ty_m;
  anchor_->T_map_odom.linear() = quatFromYaw(bounded.yaw_rad).toRotationMatrix();
  anchor_->odom_cov_xy_ref = odom.cov_xy_total;
  anchor_->odom_cov_yaw_ref = odom.cov_yaw_total;
  anchor_->source = "gnss_recovering";

  const double residual_position = bounded.residual_base_translation_m;
  const double residual_yaw = bounded.residual_yaw_rad;

  // The frame transform is deliberately corrected over several cycles. Until
  // it reaches the accepted target, include the remaining discrepancy in the
  // published covariance instead of reporting a falsely precise pose.
  const double residual_position_variance = residual_position * residual_position;
  const double residual_yaw_variance = residual_yaw * residual_yaw;
  anchor_->P = recovery_target_covariance_;
  anchor_->P(0, 0) += residual_position_variance;
  anchor_->P(1, 1) += residual_position_variance;
  anchor_->P(2, 2) += residual_yaw_variance;
  anchor_->P = sanitizeMeasurementCovariance(
    anchor_->P, anchor_cov_xy_default_, anchor_cov_yaw_default_, true);

  last_recovery_step_stamp_ = measurement.stamp;
  last_good_gnss_stamp_ = measurement.stamp;
  last_candidate_gnss_stamp_ = measurement.stamp;
  if (refresh_accepted) {
    last_recovery_reason_ = "bounded_recovery_update";
  } else if (!refreshed.valid) {
    last_recovery_reason_ = "bounded_recovery_using_last_valid_target_" + refreshed.reason;
  } else if (!refresh_geometry_valid) {
    last_recovery_reason_ = "bounded_recovery_target_refresh_non_finite_base_geometry";
  } else {
    last_recovery_reason_ = "bounded_recovery_target_refresh_jump_rejected";
  }
  last_rejection_reason_ = "none";
  last_measurement_source_ = measurement.source;
  last_measurement_cov_xy_ = measurement.cov_xy;
  last_measurement_cov_yaw_ = measurement.cov_yaw;
  last_fix_state_ = GnssFixState::GOOD;
  last_correction_xy_m_ = bounded.applied_base_translation_m;
  last_correction_yaw_rad_ = std::fabs(bounded.applied_yaw_rad);
  last_gain_xy_ = 0.0;
  last_gain_yaw_ = 0.0;

  last_innovation_xy_m_ = residual_position;
  last_innovation_yaw_rad_ = residual_yaw;

  if (bounded.reached_target &&
    residual_position <= recovery_exit_position_m_ &&
    residual_yaw <= recovery_exit_yaw_rad_)
  {
    ++recovery_exit_count_;
  } else {
    recovery_exit_count_ = 0U;
    tracking_rejection_count_ = 0U;
  }

  if (recovery_exit_count_ >= recovery_exit_min_samples_) {
    recovery_state_ = GnssRecoveryState::TRACKING;
    recovery_candidates_.clear();
    recovery_target_.reset();
    recovery_exit_count_ = 0U;
    anchor_->source = "gnss_recovery_complete";
    anchor_->yaw_unobserved = false;
    anchor_->P = sanitizeMeasurementCovariance(
      recovery_target_covariance_, anchor_cov_xy_default_, anchor_cov_yaw_default_, true);
    recovery_target_covariance_.setIdentity();
    last_candidate_gnss_stamp_ = last_good_gnss_stamp_;
    last_recovery_reason_ = "recovery_complete";
  }
}

void MapOdomFusionNode::applyXyOnlyRecoveryStep(
  const AbsolutePoseMeasurement & measurement,
  const OdomSample & odom)
{
  std::string eligibility_reason;
  if (!xyOnlyRecoveryIsSafe(measurement, odom, eligibility_reason)) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (recovery_state_ == GnssRecoveryState::RECOVERING_XY_ONLY ||
      recovery_state_ == GnssRecoveryState::TRACKING_XY_ONLY)
    {
      recovery_state_ = GnssRecoveryState::REACQUIRING;
      recovery_target_.reset();
      recovery_target_covariance_.setIdentity();
      recovery_exit_count_ = 0U;
      last_recovery_step_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_recovery_reason_ = "xy_only_suspended_" + eligibility_reason;
    }
    return;
  }

  double fixed_yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!anchor_) {
      return;
    }
    fixed_yaw = yawFromIso(anchor_->T_map_odom);
  }
  const FixedYawTranslationResult refreshed = estimateXyOnlyRecoveryTarget(fixed_yaw);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!anchor_ ||
    (recovery_state_ != GnssRecoveryState::RECOVERING_XY_ONLY &&
    recovery_state_ != GnssRecoveryState::TRACKING_XY_ONLY) ||
    !recovery_target_)
  {
    return;
  }
  last_xy_only_alignment_result_ = refreshed;
  if (newestSampleWasRejected(refreshed)) {
    last_recovery_reason_ = "xy_only_latest_sample_rejected";
    last_rejection_reason_ = "xy_only_waiting_for_newest_inlier";
    recovery_exit_count_ = 0U;
    return;
  }

  bool refresh_accepted = false;
  if (refreshed.valid) {
    Eigen::Isometry3d candidate_target = anchor_->T_map_odom;
    candidate_target.translation().x() = refreshed.tx_m;
    candidate_target.translation().y() = refreshed.ty_m;
    const double candidate_translation = (
      candidate_target.translation().head<2>() -
      anchor_->T_map_odom.translation().head<2>()).norm();
    const double refresh_translation = (
      candidate_target.translation().head<2>() -
      recovery_target_->translation().head<2>()).norm();
    if (candidate_translation <= xy_only_max_target_translation_m_ &&
      refresh_translation <= xy_only_max_target_refresh_translation_m_)
    {
      recovery_target_ = candidate_target;
      const double lever_arm_xy = measurement.position_is_base_link ? 0.0 :
        measurement.observation_point_in_base.head<2>().norm();
      const double lever_arm_variance = lever_arm_xy * lever_arm_xy *
        std::max(0.0, recovery_target_covariance_(2, 2));
      const double refreshed_position_variance = std::max(
        {anchor_cov_xy_default_, measurement.cov_xy,
          refreshed.position_rms_m * refreshed.position_rms_m + lever_arm_variance});
      recovery_target_covariance_(0, 0) = std::max(
        recovery_target_covariance_(0, 0), refreshed_position_variance);
      recovery_target_covariance_(1, 1) = std::max(
        recovery_target_covariance_(1, 1), refreshed_position_variance);
      refresh_accepted = true;
    }
  }

  const bool first_xy_only_step = last_recovery_step_stamp_.nanoseconds() == 0;
  const double previous_step_sec = first_xy_only_step ?
    0.0 : (measurement.stamp - last_recovery_step_stamp_).seconds();
  const double nominal_period = 1.0 / std::max(1.0, publish_rate_hz_);
  const double step_dt = std::max(nominal_period, std::max(0.0, previous_step_sec));
  const double max_translation_step = std::min(
    recovery_max_correction_per_update_m_,
    recovery_max_correction_per_sec_m_ * step_dt);
  const double anchor_yaw_before = yawFromIso(anchor_->T_map_odom);
  const BoundedCorrectionResult bounded = applyBoundedCorrection(
    anchor_->T_map_odom.translation().x(),
    anchor_->T_map_odom.translation().y(),
    anchor_yaw_before,
    recovery_target_->translation().x(),
    recovery_target_->translation().y(),
    anchor_yaw_before,
    max_translation_step,
    0.0);

  const GnssRecoveryState previous_state = recovery_state_;
  // The target covariance created by startXyOnlyRecoveryFromCandidates()
  // already materializes all drift and odometry covariance through the
  // latest measurement. Do not add the whole outage a second time on the
  // first bounded step. Once the anchor references are advanced below, each
  // later step contributes only its incremental covariance.
  const RecoveryCovarianceIncrement covariance_increment =
    incrementalRecoveryCovariance(
    first_xy_only_step,
    previous_step_sec,
    0.0,
    cov_yaw_drift_per_sec_ * no_fix_cov_drift_scale_,
    odom.cov_xy_total,
    anchor_->odom_cov_xy_ref,
    odom.cov_yaw_total,
    anchor_->odom_cov_yaw_ref);
  recovery_target_covariance_(0, 0) += covariance_increment.xy_variance;
  recovery_target_covariance_(1, 1) += covariance_increment.xy_variance;
  recovery_target_covariance_(2, 2) += covariance_increment.yaw_variance;
  anchor_->stamp = measurement.stamp;
  anchor_->T_map_odom.translation().x() = bounded.tx_m;
  anchor_->T_map_odom.translation().y() = bounded.ty_m;
  // Intentionally do not rewrite the rotation matrix. XY-only recovery must
  // preserve the stored map->odom yaw exactly.
  anchor_->odom_cov_xy_ref = odom.cov_xy_total;
  anchor_->odom_cov_yaw_ref = odom.cov_yaw_total;
  anchor_->yaw_unobserved = true;
  anchor_->xy_reference_odom_position = odom.T_odom_base.translation().head<2>();
  anchor_->source = "gnss_xy_only_recovering";

  const double residual_position = (
    recovery_target_->translation().head<2>() -
    anchor_->T_map_odom.translation().head<2>()).norm();
  const double residual_position_variance = residual_position * residual_position;
  anchor_->P(0, 0) = recovery_target_covariance_(0, 0) + residual_position_variance;
  anchor_->P(1, 1) = recovery_target_covariance_(1, 1) + residual_position_variance;
  // Do not reduce yaw variance or its cross-covariance from a position-only
  // observation. The target covariance has already materialized outage drift.
  anchor_->P(2, 2) = std::max(anchor_->P(2, 2), recovery_target_covariance_(2, 2));
  anchor_->P = 0.5 * (anchor_->P + anchor_->P.transpose());

  last_recovery_step_stamp_ = measurement.stamp;
  last_good_gnss_stamp_ = measurement.stamp;
  last_candidate_gnss_stamp_ = measurement.stamp;
  last_rejection_reason_ = "none";
  last_measurement_source_ = measurement.source;
  last_measurement_cov_xy_ = measurement.cov_xy;
  last_measurement_cov_yaw_ = measurement.cov_yaw;
  last_fix_state_ = GnssFixState::GOOD;
  last_correction_xy_m_ = bounded.applied_translation_m;
  last_correction_yaw_rad_ = 0.0;
  last_gain_xy_ = 0.0;
  last_gain_yaw_ = 0.0;
  last_innovation_xy_m_ = residual_position;
  last_innovation_yaw_rad_ = 0.0;

  if (previous_state == GnssRecoveryState::TRACKING_XY_ONLY &&
    residual_position <= xy_only_exit_position_m_)
  {
    recovery_state_ = GnssRecoveryState::TRACKING_XY_ONLY;
    recovery_exit_count_ = 0U;
    anchor_->source = "gnss_xy_only_tracking";
    last_recovery_reason_ = "xy_only_tracking_update_yaw_unobserved";
  } else {
    recovery_state_ = GnssRecoveryState::RECOVERING_XY_ONLY;
    if (residual_position <= xy_only_exit_position_m_) {
      ++recovery_exit_count_;
    } else {
      recovery_exit_count_ = 0U;
    }
  }

  if (recovery_state_ == GnssRecoveryState::RECOVERING_XY_ONLY &&
    recovery_exit_count_ >= xy_only_exit_min_samples_)
  {
    recovery_state_ = GnssRecoveryState::TRACKING_XY_ONLY;
    recovery_exit_count_ = 0U;
    anchor_->source = "gnss_xy_only_tracking";
    last_recovery_reason_ = "xy_only_position_converged_yaw_unobserved";
  } else if (recovery_state_ == GnssRecoveryState::RECOVERING_XY_ONLY && refresh_accepted) {
    last_recovery_reason_ = "xy_only_bounded_recovery_update";
  } else if (recovery_state_ == GnssRecoveryState::RECOVERING_XY_ONLY && !refreshed.valid) {
    last_recovery_reason_ = "xy_only_using_last_valid_target_" + refreshed.reason;
  } else if (recovery_state_ == GnssRecoveryState::RECOVERING_XY_ONLY) {
    last_recovery_reason_ = "xy_only_target_refresh_jump_rejected";
  }
}

void MapOdomFusionNode::updateRecoveryModeFromClock(const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!anchor_) {
    recovery_state_ = GnssRecoveryState::UNINITIALIZED;
    return;
  }
  const rclcpp::Time reference_stamp =
    (recovery_state_ == GnssRecoveryState::REACQUIRING ||
    recovery_state_ == GnssRecoveryState::RECOVERING ||
    recovery_state_ == GnssRecoveryState::RECOVERING_XY_ONLY ||
    recovery_state_ == GnssRecoveryState::TRACKING_XY_ONLY) &&
    last_candidate_gnss_stamp_.nanoseconds() != 0 ?
    last_candidate_gnss_stamp_ : last_good_gnss_stamp_;
  if (reference_stamp.nanoseconds() == 0 || stamp < reference_stamp) {
    return;
  }

  const double age = (stamp - reference_stamp).seconds();
  if ((recovery_state_ == GnssRecoveryState::TRACKING ||
    recovery_state_ == GnssRecoveryState::REACQUIRING ||
    recovery_state_ == GnssRecoveryState::RECOVERING ||
    recovery_state_ == GnssRecoveryState::RECOVERING_XY_ONLY ||
    recovery_state_ == GnssRecoveryState::TRACKING_XY_ONLY) &&
    age > gnss_outage_timeout_sec_)
  {
    recovery_state_ = GnssRecoveryState::OUTAGE;
    recovery_candidates_.clear();
    recovery_target_.reset();
    recovery_target_covariance_.setIdentity();
    recovery_exit_count_ = 0U;
    last_xy_only_alignment_result_ = FixedYawTranslationResult{};
    last_recovery_reason_ = "gnss_timeout";
  }
  const bool full_reacquiring = recovery_state_ == GnssRecoveryState::REACQUIRING;
  const bool xy_only_active =
    recovery_state_ == GnssRecoveryState::RECOVERING_XY_ONLY ||
    recovery_state_ == GnssRecoveryState::TRACKING_XY_ONLY;
  const double candidate_gap_limit = xy_only_active ?
    xy_only_alignment_config_.max_sample_gap_sec :
    recovery_alignment_config_.max_sample_gap_sec;
  if ((full_reacquiring || xy_only_active) &&
    !recovery_candidates_.empty() &&
    static_cast<double>(stamp.nanoseconds()) * 1.0e-9 -
    recovery_candidates_.back().stamp_sec > candidate_gap_limit)
  {
    recovery_state_ = GnssRecoveryState::OUTAGE;
    recovery_candidates_.clear();
    recovery_target_.reset();
    recovery_target_covariance_.setIdentity();
    recovery_exit_count_ = 0U;
    last_xy_only_alignment_result_ = FixedYawTranslationResult{};
    last_recovery_reason_ = xy_only_active ?
      "xy_only_candidate_timeout" : "reacquisition_timeout";
  }
}

void MapOdomFusionNode::applyMeasurement(
  const AbsolutePoseMeasurement & measurement,
  const OdomSample & odom)
{
  const bool is_gnss = measurement.source == "gnss";
  const GnssFixState fix_state = gnssFixState(measurement);

  if (is_gnss && fix_state == GnssFixState::BAD) {
    if (tolerateSoftBadGnss(measurement)) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_fix_state_ = fix_state;
      last_measurement_source_ = measurement.source;
      last_measurement_cov_xy_ = measurement.cov_xy;
      last_measurement_cov_yaw_ = measurement.cov_yaw;
      last_rejection_reason_ = "gnss_quality_bad";
    }
    enterOutage("gnss_quality_bad");
    return;
  }
  if (is_gnss && fix_state == GnssFixState::UNKNOWN && gnss_init_require_fix_) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_fix_state_ = fix_state;
    last_rejection_reason_ = "gnss_status_unknown";
    return;
  }
  bool has_anchor = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_anchor = anchor_.has_value();
  }

  // Manual initial poses are explicit reset commands, not noisy measurements.
  if (!is_gnss) {
    (void)initializeManualAnchor(measurement, odom);
    return;
  }
  if (!has_anchor) {
    if (measurement.cov_xy > gnss_max_cov_xy_) {
      std::lock_guard<std::mutex> lock(mutex_);
      last_rejection_reason_ = "gnss_position_covariance_gate";
      last_fix_state_ = fix_state;
      return;
    }
    appendRecoveryCandidate(measurement, odom);
    (void)initializeFromCandidates(measurement, odom);
    return;
  }

  updateRecoveryModeFromClock(measurement.stamp);
  GnssRecoveryState state;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = recovery_state_;
  }

  if (state == GnssRecoveryState::TRACKING) {
    if (!normalEkfUpdate(measurement, odom, fix_state)) {
      bool threshold_reached = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++tracking_rejection_count_;
        threshold_reached = tracking_rejection_count_ >= tracking_rejection_min_samples_;
        last_recovery_reason_ = "tracking_rejection_" +
          std::to_string(tracking_rejection_count_) + "_of_" +
          std::to_string(tracking_rejection_min_samples_);
      }
      if (threshold_reached) {
        beginReacquisition("tracking_rejection_threshold_reached");
        appendRecoveryCandidate(measurement, odom);
      }
    }
    return;
  }

  if (measurement.cov_xy > gnss_max_cov_xy_) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rejection_reason_ = "gnss_position_covariance_gate";
    last_fix_state_ = fix_state;
    return;
  }

  if (state == GnssRecoveryState::OUTAGE) {
    beginReacquisition("good_gnss_returned");
    appendRecoveryCandidate(measurement, odom);
    if (startRecoveryFromCandidates(measurement, odom)) {
      applyRecoveryStep(measurement, odom);
    } else if (startXyOnlyRecoveryFromCandidates(measurement, odom)) {
      applyXyOnlyRecoveryStep(measurement, odom);
    }
    return;
  }

  if (state == GnssRecoveryState::REACQUIRING) {
    appendRecoveryCandidate(measurement, odom);
    if (startRecoveryFromCandidates(measurement, odom)) {
      applyRecoveryStep(measurement, odom);
    } else if (startXyOnlyRecoveryFromCandidates(measurement, odom)) {
      applyXyOnlyRecoveryStep(measurement, odom);
    }
    return;
  }

  if (state == GnssRecoveryState::RECOVERING_XY_ONLY ||
    state == GnssRecoveryState::TRACKING_XY_ONLY)
  {
    appendRecoveryCandidate(measurement, odom);
    // A newly observable full SE(2) target always takes priority over the
    // degraded fixed-yaw mode.
    if (startRecoveryFromCandidates(measurement, odom)) {
      applyRecoveryStep(measurement, odom);
    } else {
      applyXyOnlyRecoveryStep(measurement, odom);
    }
    return;
  }

  if (state == GnssRecoveryState::RECOVERING) {
    appendRecoveryCandidate(measurement, odom);
    applyRecoveryStep(measurement, odom);
    return;
  }

  // The only remaining state without an anchor is UNINITIALIZED. Build the
  // initial map->odom transform from a consistent GNSS/odom window instead
  // of accepting a single fix.
  appendRecoveryCandidate(measurement, odom);
  (void)initializeFromCandidates(measurement, odom);
}

void MapOdomFusionNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr message)
{
  if (message->header.frame_id != odom_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring odometry with frame_id='%s'; expected '%s'.",
      message->header.frame_id.c_str(), odom_frame_.c_str());
    return;
  }
  if (message->child_frame_id != base_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring odometry with child_frame_id='%s'; expected '%s'.",
      message->child_frame_id.c_str(), base_frame_.c_str());
    return;
  }

  OdomSample sample;
  sample.stamp = rclcpp::Time(message->header.stamp);
  if (sample.stamp.nanoseconds() == 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring odometry with a zero timestamp.");
    return;
  }
  if (!quaternionIsValid(message->pose.pose.orientation)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring odometry with an invalid pose quaternion.");
    return;
  }
  sample.T_odom_base = poseMsgToIso(message->pose.pose);
  if (!isFiniteTransform(sample.T_odom_base)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Ignoring non-finite odometry pose.");
    return;
  }
  if (!twistIsFinite(message->twist.twist)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring odometry with a non-finite twist.");
    return;
  }
  sample.twist = message->twist.twist;
  sample.twist_covariance = sanitizedCovariance(message->twist.covariance);
  sample.cov_xy_total = 0.5 * (
    sanitizedNonNegative(message->pose.covariance[0]) +
    sanitizedNonNegative(message->pose.covariance[7]));
  sample.cov_yaw_total = sanitizedNonNegative(message->pose.covariance[35]);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!odom_buffer_.empty() && sample.stamp < odom_buffer_.back().stamp) {
      // A deque used for interpolation must stay strictly monotonic. Replacing
      // its newest element with an older sample silently breaks that invariant.
      last_rejection_reason_ = "out_of_order_odom";
      return;
    } else if (!odom_buffer_.empty() && sample.stamp == odom_buffer_.back().stamp) {
      odom_buffer_.back() = sample;
    } else {
      odom_buffer_.push_back(sample);
    }
    while (!odom_buffer_.empty() &&
      (sample.stamp - odom_buffer_.front().stamp).seconds() > odom_buffer_sec_)
    {
      odom_buffer_.pop_front();
    }
  }

  updateRecoveryModeFromClock(sample.stamp);
  resolvePendingMeasurement();
  publishFused(sample.stamp);
}

void MapOdomFusionNode::onGnssInput(
  const pure_gnss_msgs::msg::GnssFusionInput::SharedPtr message)
{
  const rclcpp::Time header_stamp(message->header.stamp);
  const rclcpp::Time odom_stamp(message->odom.header.stamp);

  // Status-only updates may omit odometry, but they still need a real source
  // timestamp. Substituting reception time would hide sensor/transport faults.
  if (header_stamp.nanoseconds() == 0) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rejection_reason_ = "gnss_zero_header_stamp";
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring GNSS input with a zero header timestamp.");
    return;
  }

  bool status_updated = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!gnss_status_.valid || header_stamp >= gnss_status_.stamp) {
      gnss_status_.stamp = header_stamp;
      gnss_status_.status = static_cast<int>(message->fix_status);
      gnss_status_.valid = true;
      status_updated = true;
    }
  }

  if (!message->has_odom) {
    if (status_updated &&
      (message->fix_status < gnss_fix_min_status_ || !message->gnss_usable))
    {
      enterOutage("status_only_bad_fix");
    }
    return;
  }

  if (odom_stamp.nanoseconds() == 0) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rejection_reason_ = "gnss_zero_odom_stamp";
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring GNSS observation with a zero odometry timestamp.");
    return;
  }
  if (std::fabs((header_stamp - odom_stamp).seconds()) >
    measurement_reorder_tolerance_sec_)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rejection_reason_ = "gnss_header_odom_stamp_mismatch";
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring GNSS observation whose header and odometry timestamps differ by %.3f s.",
      std::fabs((header_stamp - odom_stamp).seconds()));
    return;
  }

  if (message->odom.header.frame_id != map_frame_) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rejection_reason_ = "gnss_header_frame_mismatch";
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring GNSS observation in frame '%s'; expected '%s'.",
      message->odom.header.frame_id.c_str(), map_frame_.c_str());
    return;
  }
  if (message->position_is_base_link && message->observation_point_valid) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_rejection_reason_ = "gnss_conflicting_observation_semantics";
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring GNSS observation marked as both base_link position and lever-arm point.");
    return;
  }
  if (message->position_is_base_link) {
    if (message->odom.child_frame_id != base_frame_) {
      std::lock_guard<std::mutex> lock(mutex_);
      last_rejection_reason_ = "gnss_base_child_frame_mismatch";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring base-referenced GNSS observation with child_frame_id='%s'; expected '%s'.",
        message->odom.child_frame_id.c_str(), base_frame_.c_str());
      return;
    }
  } else {
    if (message->odom.child_frame_id.empty()) {
      std::lock_guard<std::mutex> lock(mutex_);
      last_rejection_reason_ = "gnss_observation_child_frame_empty";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring antenna-referenced GNSS observation with an empty child frame.");
      return;
    }
    if (!message->observation_point_valid && !gnss_allow_unknown_observation_point_) {
      std::lock_guard<std::mutex> lock(mutex_);
      last_rejection_reason_ = "gnss_observation_point_unknown";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring antenna-referenced GNSS observation without a base_link lever arm.");
      return;
    }
  }

  handleMeasurement(measurementFromGnssInput(*message));
}

void MapOdomFusionNode::onLegacyAnchorPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
  if (message->header.frame_id != map_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring legacy anchor in frame '%s'; expected '%s'.",
      message->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }
  handleMeasurement(measurementFromPose(*message, "legacy_anchor"));
}

void MapOdomFusionNode::onInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
  if (message->header.frame_id != map_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring initial pose in frame '%s'; expected '%s'.",
      message->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }
  handleMeasurement(measurementFromPose(*message, "initialpose"));
}

void MapOdomFusionNode::publishFused(const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> publish_lock(publish_mutex_);
  if (last_published_stamp_.nanoseconds() != 0 && stamp < last_published_stamp_) {
    out_of_order_publish_drop_count_.fetch_add(1U, std::memory_order_relaxed);
    return;
  }

  updateRecoveryModeFromClock(stamp);

  AnchorState anchor;
  GnssRecoveryState recovery_state;
  rclcpp::Time last_good_stamp(0, 0, RCL_ROS_TIME);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!anchor_) {
      return;
    }
    anchor = *anchor_;
    recovery_state = recovery_state_;
    last_good_stamp = last_good_gnss_stamp_;
  }

  const auto odom = interpolateOdom(stamp);
  if (!odom) {
    return;
  }

  const Eigen::Isometry3d map_base = anchor.T_map_odom * odom->T_odom_base;
  const double anchor_age = std::max(0.0, (stamp - anchor.stamp).seconds());
  const double odom_cov_xy_since = std::max(
    0.0, odom->cov_xy_total - anchor.odom_cov_xy_ref);
  const double odom_cov_yaw_since = std::max(
    0.0, odom->cov_yaw_total - anchor.odom_cov_yaw_ref);
  const GnssFixState live_fix = liveGnssFixState(stamp);
  const bool position_degraded = recovery_state == GnssRecoveryState::OUTAGE ||
    recovery_state == GnssRecoveryState::REACQUIRING ||
    recovery_state == GnssRecoveryState::RECOVERING ||
    recovery_state == GnssRecoveryState::RECOVERING_XY_ONLY ||
    live_fix == GnssFixState::BAD;
  const bool yaw_degraded = position_degraded ||
    recovery_state == GnssRecoveryState::TRACKING_XY_ONLY;
  const double position_drift_scale = position_degraded ? no_fix_cov_drift_scale_ : 1.0;
  const double yaw_drift_scale = yaw_degraded ? no_fix_cov_drift_scale_ : 1.0;

  Eigen::Matrix3d published_covariance = anchor.P;
  published_covariance(0, 0) +=
    cov_xy_drift_per_sec_ * position_drift_scale * anchor_age + odom_cov_xy_since;
  published_covariance(1, 1) +=
    cov_xy_drift_per_sec_ * position_drift_scale * anchor_age + odom_cov_xy_since;
  published_covariance(2, 2) +=
    cov_yaw_drift_per_sec_ * yaw_drift_scale * anchor_age + odom_cov_yaw_since;
  if (anchor.yaw_unobserved) {
    const Eigen::Vector2d delta_odom =
      odom->T_odom_base.translation().head<2>() - anchor.xy_reference_odom_position;
    const PositionYawJacobian position_yaw = positionYawJacobian(
      delta_odom.x(), delta_odom.y(), yawFromIso(anchor.T_map_odom));
    Eigen::Matrix3d reference_jacobian = Eigen::Matrix3d::Identity();
    reference_jacobian(0, 2) = position_yaw.dx_dyaw;
    reference_jacobian(1, 2) = position_yaw.dy_dyaw;
    published_covariance = reference_jacobian * published_covariance *
      reference_jacobian.transpose();
  }
  published_covariance = 0.5 * (
    published_covariance + published_covariance.transpose());

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = map_frame_;
  pose.pose.pose = isoToPoseMsg(map_base);
  pose.pose.covariance.fill(0.0);
  pose.pose.covariance[0] = published_covariance(0, 0);
  pose.pose.covariance[1] = published_covariance(0, 1);
  pose.pose.covariance[5] = published_covariance(0, 2);
  pose.pose.covariance[6] = published_covariance(1, 0);
  pose.pose.covariance[7] = published_covariance(1, 1);
  pose.pose.covariance[11] = published_covariance(1, 2);
  pose.pose.covariance[14] = kUnknownVariance;
  pose.pose.covariance[21] = kUnknownVariance;
  pose.pose.covariance[28] = kUnknownVariance;
  pose.pose.covariance[30] = published_covariance(2, 0);
  pose.pose.covariance[31] = published_covariance(2, 1);
  pose.pose.covariance[35] = published_covariance(2, 2);
  pose_publisher_->publish(pose);

  nav_msgs::msg::Odometry output_odom;
  output_odom.header = pose.header;
  output_odom.child_frame_id = base_frame_;
  output_odom.pose = pose.pose;
  output_odom.twist.twist = odom->twist;
  output_odom.twist.covariance = odom->twist_covariance;
  odom_publisher_->publish(output_odom);

  if (publish_tf_ && transform_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    const geometry_msgs::msg::Pose anchor_pose = isoToPoseMsg(anchor.T_map_odom);
    transform.transform.translation.x = anchor_pose.position.x;
    transform.transform.translation.y = anchor_pose.position.y;
    transform.transform.translation.z = anchor_pose.position.z;
    transform.transform.rotation = anchor_pose.orientation;
    transform_broadcaster_->sendTransform(transform);
  }

  last_published_stamp_ = stamp;
  (void)last_good_stamp;
}

void MapOdomFusionNode::onPublishTimer()
{
  publishFused(now());
}

void MapOdomFusionNode::publishDiagnostics(
  uint8_t level,
  const std::string & message)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = level;
  status.name = "localization/gnss_map_odom_fusion";
  status.message = message;
  status.hardware_id = "none";

  auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(entry);
    };

  std::optional<AnchorState> anchor;
  std::optional<AbsolutePoseMeasurement> pending;
  std::optional<OdomSample> latest_odom;
  GnssStatusState gnss_status;
  GnssRecoveryState recovery_state;
  RecoveryAlignmentResult alignment;
  FixedYawTranslationResult xy_only_alignment;
  std::size_t recovery_candidates = 0U;
  std::size_t recovery_exit_count = 0U;
  std::size_t tracking_rejection_count = 0U;
  rclcpp::Time last_good_stamp(0, 0, RCL_ROS_TIME);
  std::string recovery_reason;
  std::string rejection_reason;
  std::string measurement_source;
  double innovation_xy = 0.0;
  double innovation_yaw = 0.0;
  double correction_xy = 0.0;
  double correction_yaw = 0.0;
  double gain_xy = 0.0;
  double gain_yaw = 0.0;
  double measurement_cov_xy = 0.0;
  double measurement_cov_yaw = 0.0;
  GnssFixState last_fix_state = GnssFixState::UNKNOWN;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    anchor = anchor_;
    pending = pending_measurement_;
    if (!odom_buffer_.empty()) {
      latest_odom = odom_buffer_.back();
    }
    gnss_status = gnss_status_;
    recovery_state = recovery_state_;
    alignment = last_alignment_result_;
    xy_only_alignment = last_xy_only_alignment_result_;
    recovery_candidates = recovery_candidates_.size();
    recovery_exit_count = recovery_exit_count_;
    tracking_rejection_count = tracking_rejection_count_;
    last_good_stamp = last_good_gnss_stamp_;
    recovery_reason = last_recovery_reason_;
    rejection_reason = last_rejection_reason_;
    measurement_source = last_measurement_source_;
    innovation_xy = last_innovation_xy_m_;
    innovation_yaw = last_innovation_yaw_rad_;
    correction_xy = last_correction_xy_m_;
    correction_yaw = last_correction_yaw_rad_;
    gain_xy = last_gain_xy_;
    gain_yaw = last_gain_yaw_;
    measurement_cov_xy = last_measurement_cov_xy_;
    measurement_cov_yaw = last_measurement_cov_yaw_;
    last_fix_state = last_fix_state_;
  }

  add("recovery.state", toString(recovery_state));
  const bool xy_only_state =
    recovery_state == GnssRecoveryState::RECOVERING_XY_ONLY ||
    recovery_state == GnssRecoveryState::TRACKING_XY_ONLY;
  add("recovery.mode", xy_only_state ? "xy_only" : "full_se2");
  add("recovery.position_fused",
    (recovery_state == GnssRecoveryState::TRACKING ||
    recovery_state == GnssRecoveryState::RECOVERING || xy_only_state) ? "true" : "false");
  add("recovery.yaw_fused",
    (recovery_state == GnssRecoveryState::TRACKING ||
    recovery_state == GnssRecoveryState::RECOVERING) ? "true" : "false");
  add("recovery.reason", recovery_reason);
  add("recovery.candidate_count", std::to_string(recovery_candidates));
  add("recovery.exit_count", std::to_string(recovery_exit_count));
  add("tracking.rejection_count", std::to_string(tracking_rejection_count));
  add("tracking.rejection_threshold", std::to_string(tracking_rejection_min_samples_));
  add("recovery.alignment_valid", alignment.valid ? "true" : "false");
  add("recovery.alignment_reason", alignment.reason);
  add("recovery.position_rms_m", std::to_string(alignment.position_rms_m));
  add("recovery.odom_baseline_m", std::to_string(alignment.odom_baseline_m));
  add("recovery.heading_std_rad", std::to_string(alignment.heading_std_rad));
  add("recovery.heading_samples", std::to_string(alignment.heading_sample_count));
  add("recovery.rejected_sample_count", std::to_string(alignment.rejected_sample_count));
  add("recovery.rejected_sample_index",
    alignment.rejected_sample_count > 0U ?
    std::to_string(alignment.rejected_sample_index) : std::string("none"));
  add("xy_only.enabled", xy_only_recovery_enabled_ ? "true" : "false");
  add("xy_only.alignment_valid", xy_only_alignment.valid ? "true" : "false");
  add("xy_only.alignment_reason", xy_only_alignment.reason);
  add("xy_only.position_rms_m", std::to_string(xy_only_alignment.position_rms_m));
  add("xy_only.max_position_residual_m",
    std::to_string(xy_only_alignment.max_position_residual_m));
  add("xy_only.rejected_sample_count",
    std::to_string(xy_only_alignment.rejected_sample_count));
  add("last_rejection_reason", rejection_reason);
  add("last_measurement_source", measurement_source);
  add("last_fix_state", fixStateToString(static_cast<int>(last_fix_state)));
  add("last_innovation_xy_m", std::to_string(innovation_xy));
  add("last_innovation_yaw_rad", std::to_string(innovation_yaw));
  add("last_correction_xy_m", std::to_string(correction_xy));
  add("last_correction_yaw_rad", std::to_string(correction_yaw));
  add("last_gain_xy", std::to_string(gain_xy));
  add("last_gain_yaw", std::to_string(gain_yaw));
  add("last_measurement_cov_xy", std::to_string(measurement_cov_xy));
  add("last_measurement_cov_yaw", std::to_string(measurement_cov_yaw));
  add("pending_measurement", pending ? "true" : "false");
  add("output.out_of_order_drop_count", std::to_string(
      out_of_order_publish_drop_count_.load(std::memory_order_relaxed)));
  add("anchor_valid", anchor ? "true" : "false");

  const rclcpp::Time current = now();
  if (latest_odom) {
    add("odom_age_sec", std::to_string(
      std::fabs((current - latest_odom->stamp).seconds())));
  }
  if (last_good_stamp.nanoseconds() != 0) {
    add("last_good_gnss_age_sec", std::to_string(
      std::max(0.0, (current - last_good_stamp).seconds())));
  }
  add("gnss_status_valid", gnss_status.valid ? "true" : "false");
  if (gnss_status.valid) {
    add("gnss_status", std::to_string(gnss_status.status));
    add("gnss_status_age_sec", std::to_string(
      std::fabs((current - gnss_status.stamp).seconds())));
  }
  if (anchor) {
    add("anchor_source", anchor->source);
    add("anchor_x", std::to_string(anchor->T_map_odom.translation().x()));
    add("anchor_y", std::to_string(anchor->T_map_odom.translation().y()));
    add("anchor_yaw", std::to_string(yawFromIso(anchor->T_map_odom)));
    add("anchor_cov_x", std::to_string(anchor->P(0, 0)));
    add("anchor_cov_y", std::to_string(anchor->P(1, 1)));
    add("anchor_cov_yaw", std::to_string(anchor->P(2, 2)));
    add("xy_only.yaw_unobserved", anchor->yaw_unobserved ? "true" : "false");
    if (latest_odom) {
      const double xy_reference_distance = anchor->yaw_unobserved ?
        (latest_odom->T_odom_base.translation().head<2>() -
        anchor->xy_reference_odom_position).norm() : 0.0;
      add("xy_only.distance_from_reference_m",
        std::to_string(xy_reference_distance));
      const double elapsed = std::max(0.0, (current - anchor->stamp).seconds());
      const double effective_yaw_variance = anchor->P(2, 2) +
        cov_yaw_drift_per_sec_ * no_fix_cov_drift_scale_ * elapsed +
        std::max(0.0, latest_odom->cov_yaw_total - anchor->odom_cov_yaw_ref);
      add("xy_only.effective_yaw_variance_rad2",
        std::to_string(effective_yaw_variance));
    }
  }

  array.status.push_back(status);
  diagnostic_publisher_->publish(array);
}

void MapOdomFusionNode::onHeartbeat()
{
  const rclcpp::Time current = now();
  updateRecoveryModeFromClock(current);

  bool has_anchor = false;
  bool has_odom = false;
  bool pending = false;
  rclcpp::Time latest_odom_stamp(0, 0, RCL_ROS_TIME);
  GnssRecoveryState state;
  std::string rejection_reason;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_anchor = anchor_.has_value();
    has_odom = !odom_buffer_.empty();
    if (has_odom) {
      latest_odom_stamp = odom_buffer_.back().stamp;
    }
    pending = pending_measurement_.has_value();
    state = recovery_state_;
    rejection_reason = last_rejection_reason_;
  }

  if (!has_odom || std::fabs((current - latest_odom_stamp).seconds()) > odom_timeout_sec_) {
    publishDiagnostics(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "relative odometry is missing or stale");
    return;
  }
  if (pending) {
    publishDiagnostics(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "waiting for odometry at the GNSS measurement time");
    return;
  }
  if (!has_anchor || state == GnssRecoveryState::UNINITIALIZED) {
    publishDiagnostics(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "waiting for the first usable absolute observation");
    return;
  }

  switch (state) {
    case GnssRecoveryState::OUTAGE:
      publishDiagnostics(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "GNSS outage; publishing dead reckoning with growing covariance");
      return;
    case GnssRecoveryState::REACQUIRING:
      publishDiagnostics(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "GNSS returned; validating a multi-sample alignment before correction");
      return;
    case GnssRecoveryState::RECOVERING_XY_ONLY:
      publishDiagnostics(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "applying bounded GNSS XY recovery; yaw remains dead-reckoned");
      return;
    case GnssRecoveryState::TRACKING_XY_ONLY:
      publishDiagnostics(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "GNSS position is tracking; absolute yaw is still unobserved");
      return;
    case GnssRecoveryState::RECOVERING:
      publishDiagnostics(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "applying bounded map-to-odom recovery corrections");
      return;
    case GnssRecoveryState::TRACKING:
      if (liveGnssFixState(current) == GnssFixState::BAD) {
        publishDiagnostics(
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "GNSS status is stale or invalid; outage transition is pending");
      } else if (rejection_reason != "none") {
        publishDiagnostics(
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "tracking with a recently rejected absolute observation");
      } else {
        publishDiagnostics(
          diagnostic_msgs::msg::DiagnosticStatus::OK,
          "GNSS/odometry fusion tracking");
      }
      return;
    case GnssRecoveryState::UNINITIALIZED:
      break;
  }
  publishDiagnostics(
    diagnostic_msgs::msg::DiagnosticStatus::WARN,
    "fusion state is uninitialized");
}

}  // namespace pure_gnss_map_odom_fusion
