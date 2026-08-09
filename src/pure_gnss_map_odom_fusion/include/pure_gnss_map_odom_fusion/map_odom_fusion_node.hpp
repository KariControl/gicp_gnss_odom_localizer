#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pure_gnss_msgs/msg/gnss_fusion_input.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "pure_gnss_map_odom_fusion/gnss_recovery_controller.hpp"

namespace pure_gnss_map_odom_fusion
{

class MapOdomFusionNode : public rclcpp::Node
{
public:
  explicit MapOdomFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct OdomSample
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Eigen::Isometry3d T_odom_base{Eigen::Isometry3d::Identity()};
    geometry_msgs::msg::Twist twist;
    std::array<double, 36> twist_covariance{};
    double cov_xy_total{0.0};
    double cov_yaw_total{0.0};
  };

  struct AbsolutePoseMeasurement
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};

    // Translation is either base_link or a GNSS observation point. When
    // yaw_valid=true, orientation always represents base_link yaw.
    Eigen::Isometry3d T_map_observation{Eigen::Isometry3d::Identity()};
    Eigen::Matrix3d R_xy_yaw{Eigen::Matrix3d::Identity() * 1.0e6};
    double cov_xy{1.0e6};
    double cov_yaw{1.0e6};
    bool yaw_valid{false};
    bool position_is_base_link{true};
    bool observation_point_valid{false};
    Eigen::Vector3d observation_point_in_base{Eigen::Vector3d::Zero()};
    std::string heading_source{"none"};

    bool has_gnss_fix_status{false};
    int gnss_fix_status{sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX};
    int fix_quality{-1};
    bool has_confidence{false};
    float confidence{0.0F};
    bool gnss_usable{true};
    std::string source{"none"};
  };

  struct AnchorState
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Eigen::Isometry3d T_map_odom{Eigen::Isometry3d::Identity()};
    Eigen::Matrix3d P{Eigen::Matrix3d::Identity()};
    double odom_cov_xy_ref{0.0};
    double odom_cov_yaw_ref{0.0};
    bool yaw_unobserved{false};
    Eigen::Vector2d xy_reference_odom_position{Eigen::Vector2d::Zero()};
    std::string source{"none"};
  };

  struct GnssStatusState
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    int status{sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX};
    bool valid{false};
  };

  enum class GnssFixState
  {
    UNKNOWN,
    GOOD,
    BAD,
  };

  static double wrapAngle(double angle);
  static double yawFromIso(const Eigen::Isometry3d & transform);
  static Eigen::Quaterniond quatFromYaw(double yaw);
  static Eigen::Isometry3d poseMsgToIso(const geometry_msgs::msg::Pose & pose);
  static geometry_msgs::msg::Pose isoToPoseMsg(const Eigen::Isometry3d & transform);
  static Eigen::Isometry3d deltaFromTwist(
    const geometry_msgs::msg::Twist & twist, double dt_sec);
  static bool isFiniteTransform(const Eigen::Isometry3d & transform);
  static Eigen::Matrix3d sanitizeMeasurementCovariance(
    const Eigen::Matrix3d & covariance,
    double fallback_xy,
    double fallback_yaw,
    bool yaw_valid);
  static Eigen::Vector3d observationPointInOdom(
    const OdomSample & odom,
    const AbsolutePoseMeasurement & measurement);
  static Eigen::Isometry3d transformFromAlignment(const RecoveryAlignmentResult & alignment);
  Eigen::Matrix3d recoveryAlignmentCovarianceFloor(
    const RecoveryAlignmentResult & alignment,
    const AbsolutePoseMeasurement & measurement) const;
  Eigen::Matrix3d conservativeRecoveryTargetCovariance(
    const RecoveryAlignmentResult & alignment,
    const AbsolutePoseMeasurement & measurement,
    const OdomSample & odom,
    const AnchorState & anchor) const;
  Eigen::Matrix3d conservativeXyOnlyTargetCovariance(
    const FixedYawTranslationResult & alignment,
    const AbsolutePoseMeasurement & measurement,
    const OdomSample & odom,
    const AnchorState & anchor) const;

  void declareAndLoadParameters();
  void validateParameters() const;

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onGnssInput(const pure_gnss_msgs::msg::GnssFusionInput::SharedPtr msg);
  void onLegacyAnchorPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onPublishTimer();
  void onHeartbeat();

  std::optional<OdomSample> interpolateOdom(const rclcpp::Time & stamp) const;
  AbsolutePoseMeasurement measurementFromGnssInput(
    const pure_gnss_msgs::msg::GnssFusionInput & msg) const;
  AbsolutePoseMeasurement measurementFromPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
    const std::string & source) const;
  bool validateMeasurement(
    const AbsolutePoseMeasurement & measurement,
    std::string & reason) const;
  GnssFixState gnssFixState(const AbsolutePoseMeasurement & measurement) const;
  GnssFixState liveGnssFixState(const rclcpp::Time & stamp) const;
  bool isHardGnssFailure(const AbsolutePoseMeasurement & measurement) const;
  bool tolerateSoftBadGnss(const AbsolutePoseMeasurement & measurement);

  void handleMeasurement(const AbsolutePoseMeasurement & measurement, bool allow_pending = true);
  bool resolvePendingMeasurement();
  void applyMeasurement(
    const AbsolutePoseMeasurement & measurement,
    const OdomSample & odom);
  bool initializeManualAnchor(
    const AbsolutePoseMeasurement & measurement,
    const OdomSample & odom);
  bool initializeFromCandidates(
    const AbsolutePoseMeasurement & latest_measurement,
    const OdomSample & latest_odom);
  bool normalEkfUpdate(
    const AbsolutePoseMeasurement & measurement,
    const OdomSample & odom,
    GnssFixState fix_state);

  void enterOutage(const std::string & reason);
  void beginReacquisition(const std::string & reason);
  void appendRecoveryCandidate(
    const AbsolutePoseMeasurement & measurement,
    const OdomSample & odom);
  RecoveryAlignmentResult estimateRecoveryTarget() const;
  FixedYawTranslationResult estimateXyOnlyRecoveryTarget(double fixed_yaw_rad) const;
  bool xyOnlyMotionIsSafe(const rclcpp::Time & stamp, std::string & reason) const;
  bool xyOnlyRecoveryIsSafe(
    const AbsolutePoseMeasurement & measurement,
    const OdomSample & odom,
    std::string & reason) const;
  bool startRecoveryFromCandidates(
    const AbsolutePoseMeasurement & latest_measurement,
    const OdomSample & latest_odom);
  bool startXyOnlyRecoveryFromCandidates(
    const AbsolutePoseMeasurement & latest_measurement,
    const OdomSample & latest_odom);
  void applyRecoveryStep(
    const AbsolutePoseMeasurement & measurement,
    const OdomSample & odom);
  void applyXyOnlyRecoveryStep(
    const AbsolutePoseMeasurement & measurement,
    const OdomSample & odom);
  void updateRecoveryModeFromClock(const rclcpp::Time & stamp);

  void publishFused(const rclcpp::Time & stamp);
  void publishDiagnostics(uint8_t level, const std::string & message);

  // Frames/topics.
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string odom_topic_{"/localization/gyro_lidar_odom"};
  std::string gnss_input_topic_{"/localization/gnss_fusion_input"};
  std::string legacy_anchor_topic_{};
  std::string initialpose_topic_{"/initialpose"};
  std::string out_pose_topic_{"/localization/ekf_pose"};
  std::string out_odom_topic_{"/localization/ekf_odom"};

  bool publish_tf_{true};
  double publish_rate_hz_{50.0};
  double heartbeat_hz_{2.0};

  // Input timing.
  double odom_buffer_sec_{15.0};
  double odom_timeout_sec_{1.0};
  double odom_max_extrapolate_sec_{0.2};
  double measurement_max_age_sec_{5.0};
  double measurement_future_tolerance_sec_{0.5};
  double measurement_reorder_tolerance_sec_{0.05};

  // EKF covariance and gates.
  double anchor_cov_xy_default_{0.25};
  double anchor_cov_yaw_default_{0.04};
  double cov_xy_drift_per_sec_{0.005};
  double cov_yaw_drift_per_sec_{0.002};
  double no_fix_cov_drift_scale_{3.0};
  bool use_gnss_status_{true};
  double gnss_status_timeout_sec_{3.0};
  int gnss_fix_min_status_{sensor_msgs::msg::NavSatStatus::STATUS_FIX};
  bool gnss_init_require_fix_{true};
  bool gnss_allow_unknown_observation_point_{false};
  double gnss_max_cov_xy_{100.0};
  double gnss_max_cov_yaw_{10.0};
  double gnss_position_nis_threshold_{11.83};  // chi-square, 2 DoF, 99.73%
  double gnss_yaw_nis_threshold_{9.0};          // chi-square, 1 DoF, 99.73%
  double gnss_position_jump_reject_m_{9.0};
  double gnss_yaw_jump_reject_rad_{1.57};
  double gnss_jump_reject_sigma_scale_{3.0};
  std::size_t tracking_rejection_min_samples_{3};

  // Reacquisition and bounded recovery.
  double gnss_outage_timeout_sec_{2.0};
  std::size_t initialization_min_samples_{3};
  std::size_t initialization_min_heading_samples_{2};
  RecoveryAlignmentConfig recovery_alignment_config_{};
  double recovery_candidate_window_sec_{15.0};
  std::size_t recovery_candidate_max_samples_{50};
  // Full-SE(2) translation gates and limits are evaluated at the synchronized
  // map->base_link position. Yaw limits remain map->odom yaw differences.
  double recovery_max_target_translation_m_{100.0};
  double recovery_max_target_yaw_rad_{3.14159265358979323846};
  // A refreshed sliding-window target must remain close to the last accepted
  // target. This prevents one late outlier from moving the recovery goal while
  // bounded correction is already in progress.
  double recovery_max_target_refresh_translation_m_{2.0};
  double recovery_max_target_refresh_yaw_rad_{0.35};
  double recovery_max_correction_per_update_m_{0.50};
  double recovery_max_correction_per_sec_m_{1.00};
  double recovery_max_yaw_correction_per_update_rad_{0.05};
  double recovery_max_yaw_correction_per_sec_rad_{0.20};
  double recovery_exit_position_m_{0.10};
  double recovery_exit_yaw_rad_{0.02};
  std::size_t recovery_exit_min_samples_{3};

  // Optional degraded recovery for a single-antenna vehicle that is stopped
  // or moving very slowly when high-quality GNSS position returns. It holds
  // map->odom yaw fixed, corrects translation only, and never claims that yaw
  // was re-observed. The public default remains disabled.
  bool xy_only_recovery_enabled_{false};
  int xy_only_required_fix_quality_{4};
  FixedYawTranslationConfig xy_only_alignment_config_{};
  double xy_only_min_candidate_span_sec_{1.0};
  double xy_only_stationary_dwell_sec_{1.0};
  double xy_only_max_speed_mps_{0.50};
  double xy_only_max_yaw_rate_radps_{0.05};
  double xy_only_max_stationary_displacement_m_{0.50};
  double xy_only_max_odom_yaw_change_rad_{0.05};
  double xy_only_max_odom_gap_sec_{0.10};
  double xy_only_max_anchor_yaw_variance_rad2_{1.44};
  double xy_only_max_outage_sec_{120.0};
  double xy_only_max_lever_arm_error_m_{0.50};
  double xy_only_max_cov_xy_{0.25};
  double xy_only_soft_bad_grace_sec_{0.50};
  double xy_only_max_target_translation_m_{10.0};
  double xy_only_max_target_refresh_translation_m_{1.0};
  double xy_only_exit_position_m_{0.10};
  std::size_t xy_only_exit_min_samples_{3};

  // Shared state.
  mutable std::mutex mutex_;
  // onOdom() and the wall timer can request publications at different ROS
  // timestamps. Serialize the complete output set and suppress a late request
  // whose stamp would move downstream time backwards.
  mutable std::mutex publish_mutex_;
  rclcpp::Time last_published_stamp_{0, 0, RCL_ROS_TIME};
  std::atomic<std::size_t> out_of_order_publish_drop_count_{0U};
  std::deque<OdomSample> odom_buffer_;
  std::optional<AnchorState> anchor_;
  std::optional<AbsolutePoseMeasurement> pending_measurement_;
  GnssStatusState gnss_status_;
  GnssRecoveryState recovery_state_{GnssRecoveryState::UNINITIALIZED};
  std::vector<RecoveryAlignmentSample> recovery_candidates_;
  std::optional<Eigen::Isometry3d> recovery_target_;
  Eigen::Matrix3d recovery_target_covariance_{Eigen::Matrix3d::Identity()};
  RecoveryAlignmentResult last_alignment_result_{};
  FixedYawTranslationResult last_xy_only_alignment_result_{};
  rclcpp::Time last_good_gnss_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_candidate_gnss_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_measurement_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_recovery_step_stamp_{0, 0, RCL_ROS_TIME};
  std::size_t recovery_exit_count_{0};
  std::size_t tracking_rejection_count_{0};

  // Diagnostics.
  std::string last_recovery_reason_{"none"};
  std::string last_rejection_reason_{"none"};
  std::string last_measurement_source_{"none"};
  double last_innovation_xy_m_{0.0};
  double last_innovation_yaw_rad_{0.0};
  double last_correction_xy_m_{0.0};
  double last_correction_yaw_rad_{0.0};
  double last_gain_xy_{0.0};
  double last_gain_yaw_{0.0};
  double last_measurement_cov_xy_{0.0};
  double last_measurement_cov_yaw_{0.0};
  GnssFixState last_fix_state_{GnssFixState::UNKNOWN};

  // ROS interfaces.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<pure_gnss_msgs::msg::GnssFusionInput>::SharedPtr
  gnss_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
  legacy_anchor_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
  initialpose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace pure_gnss_map_odom_fusion
