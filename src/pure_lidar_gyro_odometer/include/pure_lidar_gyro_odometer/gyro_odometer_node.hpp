#pragma once

#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pure_lidar_gyro_odometer/se2_fixed_lag_smoother.hpp"
#include "pure_lidar_gyro_odometer/tracking_mode.hpp"
#include "pure_lidar_gyro_odometer/yaw_rate_integrator.hpp"

namespace pure_gyro_odometer
{

class GyroOdometerNode : public rclcpp::Node
{
public:
  explicit GyroOdometerNode(const rclcpp::NodeOptions & options);

private:
  struct ImuSample
  {
    rclcpp::Time stamp;
    double gyro_z{0.0};
    double yaw_rate_corrected{0.0};
    Eigen::Vector3d acc{0.0, 0.0, 0.0};
  };

  struct WheelSample
  {
    rclcpp::Time stamp;
    double v_raw{0.0};
  };

  struct ObservabilityInfo
  {
    bool enabled{false};
    bool has_hessian{false};
    bool has_directional_information{false};
    bool wheel_prior_available{false};
    bool wheel_assisted{false};
    bool stationary_now{false};
    double information_ratio_min{1.0};
    double information_ratio_mid{1.0};
    double information_ratio_max{1.0};
    double information_deficit{0.0};
    double wheel_assist_blend{0.0};
    double wheel_assist_correction_metric{0.0};
    double wheel_distance{0.0};
    double prior_dx{0.0};
    double prior_dy{0.0};
    double prior_dyaw{0.0};
    double prior_difference_metric{0.0};
    double prior_difference_translation{0.0};
    double prior_difference_yaw{0.0};
    double stationary_projected_motion_metric{0.0};
    double scan_speed_mps{0.0};
    double wheel_speed_mps{0.0};
    double speed_difference_mps{0.0};
  };

  struct LidarOdomSample
  {
    rclcpp::Time stamp{0};
    double dt{0.0};
    bool valid{false};
    bool converged{false};
    double fitness{0.0};
    double dx{0.0};
    double dy{0.0};
    double dyaw{0.0};
    double v{0.0};
    double vx{0.0};
    double vy{0.0};
    double yaw_rate{0.0};
    double raw_dx{0.0};
    double raw_dy{0.0};
    double raw_dyaw{0.0};
    double raw_vx{0.0};
    double raw_vy{0.0};
    double raw_yaw_rate{0.0};
    double inlier_ratio{0.0};
    std::string registration_source{"none"};
    bool used_submap{false};
    bool used_scan_to_scan_fallback{false};
    std::string rejection_reason{"not_evaluated"};
    ObservabilityInfo observability;
  };

  struct LocalMapKeyframe
  {
    rclcpp::Time stamp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    // Pose of base_link in the active submap anchor frame. The submap is never
    // baked in odom coordinates, so a smoother correction moves the anchor as a
    // whole instead of distorting already inserted keyframes.
    se2::Pose anchor_base_pose;
    std::uint64_t pose_sequence{0};
  };

  struct LocalMapObservation
  {
    bool ready{false};
    bool attempted{false};
    bool accepted{false};
    se2::Pose anchor_base_pose;
    se2::Pose odom_base_pose;
    Eigen::Matrix4f T_prev_curr_scan{Eigen::Matrix4f::Identity()};
    Eigen::Matrix<double, 6, 6> hessian_scan{
      Eigen::Matrix<double, 6, 6>::Zero()};
    bool has_hessian{false};
    double fitness{std::numeric_limits<double>::infinity()};
    double inlier_ratio{0.0};
    double correction_translation_m{0.0};
    double correction_yaw_rad{0.0};
    double correction_z_m{0.0};
    double correction_roll_rad{0.0};
    double correction_pitch_rad{0.0};
    double scan_disagreement_translation_m{0.0};
    double scan_disagreement_yaw_rad{0.0};
    double factor_weight_xy{0.0};
    double factor_weight_yaw{0.0};
    std::string reason{"not_attempted"};
  };

  using ScanFactor = se2::RelativeFactor;

  static double normalizeYaw(double a);
  static double yawFromRot(const Eigen::Matrix3d & R);
  static Eigen::Quaterniond quatFromYaw(double yaw);

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onWheelTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void onReferencePose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void onPublishTimer();
  void updateStopState(const rclcpp::Time & nowt);
  void publishDiagnostics(const rclcpp::Time & stamp, const std::string & level, const std::string & msg);
  void publishObservabilityDebug(
    const rclcpp::Time & stamp, const LidarOdomSample & sample, const std::string & guess_mode_used,
    const std::string & next_guess_mode);

  bool computeAccVariance(const rclcpp::Time & nowt, double window_sec, double & out_var) const;
  bool computeImuDeltaYaw(const rclcpp::Time & t0, const rclcpp::Time & t1, double & out_dyaw) const;
  bool computeWheelDistance(const rclcpp::Time & t0, const rclcpp::Time & t1, double & out_dist) const;
  bool updateMiniSmootherLocked(const ScanFactor & factor);
  void resetLidarTrackingLocked();
  void validateParameters() const;
  LocalMapObservation matchAgainstLocalMap(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
    const Eigen::Matrix4f & T_base_scan, const Eigen::Matrix4f & T_scan_base,
    const se2::Pose & predicted_odom_base_pose, int frame_sequence,
    bool primary_tracking_attempt);
  void initializeLocalMapLocked(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
    const rclcpp::Time & stamp, const se2::Pose & odom_base_pose,
    std::uint64_t pose_sequence);
  void maybeAddLocalMapKeyframeLocked(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
    const rclcpp::Time & stamp, const se2::Pose & anchor_base_pose,
    std::uint64_t pose_sequence);
  void repairLocalMapFromSmootherLocked(std::uint64_t newest_pose_sequence);
  void rebuildLocalMapLocked();
  void reanchorLocalMapLocked();
  bool localMapRequired() const;
  bool localMapReadyLocked() const;
  se2::Pose odomPoseToLocalMapAnchorLocked(const se2::Pose & odom_base_pose) const;
  void alignLocalMapAnchorToPoseLocked(
    const se2::Pose & odom_base_pose, const se2::Pose & anchor_base_pose);
  void resetLocalMapLocked(const std::string & reason);

  // -------- Parameters --------
  std::string base_frame_;
  std::string odom_frame_;

  std::string imu_topic_;
  std::string wheel_speed_topic_;
  std::string reference_pose_topic_;
  std::string points_topic_;

  std::string out_odom_topic_;  // raw odom
  std::string out_filtered_odom_topic_;
  std::string out_stopped_topic_;

  // Publish IMU corrected into base_frame (TF-applied + yaw gyro bias corrected)
  std::string out_imu_topic_;
  bool imu_corrected_enable_{true};
  bool imu_corrected_apply_tf_{true};
  bool imu_corrected_transform_orientation_{false};
  double imu_max_abs_yaw_rate_radps_{8.0};
  double imu_max_sample_gap_sec_{0.10};
  double imu_max_boundary_gap_sec_{0.03};

  double publish_rate_hz_{50.0};
  bool out_filtered_odom_enable_{false};
  bool filtered_odom_zero_when_stopped_{true};
  double filtered_odom_lowpass_alpha_{0.85};
  double filtered_odom_linear_rate_limit_mps2_{4.0};
  double filtered_odom_lateral_rate_limit_mps2_{2.0};
  double filtered_odom_yaw_rate_limit_radps2_{1.5};
  double filtered_odom_reset_gap_sec_{1.0};

  // Stop detection
  bool stop_enable_{true};
  double stop_speed_thr_mps_{0.15};
  double stop_gyro_abs_thr_rad_s_{0.05};
  double stop_acc_var_thr_{0.0225};
  double stop_hold_sec_{0.5};

  // Gyro bias
  bool gyro_bias_enable_{true};
  double gyro_bias_tau_sec_{10.0};
  double gyro_bias_max_abs_rad_s_{0.5};

  // Wheel speed
  bool use_wheel_speed_{false};
  double wheel_speed_scale_{1.0};
  double wheel_speed_timeout_sec_{0.2};

  bool wheel_low_speed_enable_{false};
  double wheel_low_speed_deadband_mps_{0.12};
  double wheel_low_speed_acc_thr_mps2_{0.2};
  double wheel_low_speed_blend_{0.5};
  double wheel_low_speed_max_corr_mps_{0.3};

  // Online scale estimation (optional)
  bool wheel_scale_est_enable_{false};
  double wheel_scale_est_tau_sec_{30.0};
  double wheel_scale_est_min_ref_dist_m_{2.0};
  double wheel_scale_est_min_wheel_dist_m_{1.0};
  double wheel_scale_min_{0.5};
  double wheel_scale_max_{2.0};

  // Optional wheel-speed correction along continuously weak Hessian directions.
  // No discrete observability state or threshold-triggered source switch is used.
  bool wheel_observability_assist_enable_{false};
  double wheel_observability_assist_min_wheel_dist_m_{0.1};
  double wheel_observability_assist_max_blend_{0.5};
  double wheel_observability_assist_power_{2.0};
  bool wheel_registration_recovery_use_current_prior_{true};

  // Continuous Hessian/observability diagnostics.
  bool lidar_observability_debug_pub_enable_{false};
  std::string lidar_observability_debug_topic_{"/localization/lidar_observability_debug"};

  // Cumulative local odometry covariance (published in out_odom_topic).
  double odom_cov_base_xy_step_{1.0e-4};
  double odom_cov_base_yaw_step_{2.5e-5};
  double odom_cov_xy_per_meter_{5.0e-4};
  double odom_cov_yaw_per_rad_{5.0e-3};
  double odom_cov_fitness_xy_scale_{5.0e-4};
  double odom_cov_fitness_yaw_scale_{5.0e-4};
  double odom_cov_observability_max_scale_{2.0};
  double odom_cov_wheel_assist_scale_{1.5};
  double odom_cov_invalid_xy_step_{2.5e-2};
  double odom_cov_invalid_yaw_step_{1.0e-2};
  // Deprecated no-op compatibility parameters. Timer-based pose integration was removed.
  double odom_cov_deadreckon_xy_per_sec_{2.0e-2};
  double odom_cov_deadreckon_yaw_per_sec_{5.0e-3};

  // LiDAR odometry
  bool lidar_odom_enable_{true};
  std::string lidar_backend_{"SMALL_GICP"};
  std::string lidar_registration_type_{"VGICP"};
  int lidar_num_threads_{8};
  double lidar_timeout_sec_{0.5};
  double lidar_min_range_m_{2.0};
  double lidar_max_range_m_{80.0};
  double lidar_voxel_leaf_m_{0.4};
  double gicp_max_corr_dist_m_{2.5};
  int gicp_max_iterations_{30};
  double gicp_trans_eps_{1e-3};
  double gicp_rot_eps_{2e-3};
  int gicp_corr_randomness_{20};
  double gicp_fitness_max_{5.0};
  double gicp_voxel_resolution_{1.0};
  bool lidar_pose_se2_enable_{true};
  double lidar_yaw_blend_imu_{0.0};
  bool lidar_guess_use_imu_yaw_only_{true};
  std::string lidar_tracking_mode_name_{"scan_to_scan"};
  tracking::Mode lidar_tracking_mode_{tracking::Mode::ScanToScan};
  bool lidar_scan_to_submap_fallback_enable_{true};
  int lidar_scan_to_submap_match_interval_frames_{1};
  int lidar_scan_to_submap_max_consecutive_failures_{5};
  double lidar_scan_to_submap_max_scan_disagreement_translation_m_{1.0};
  double lidar_scan_to_submap_max_scan_disagreement_yaw_rad_{0.35};

  // Lightweight fixed-lag smoothing for local odometry
  bool lidar_smoother_enable_{true};
  int lidar_smoother_window_size_{20};
  int lidar_smoother_max_iter_{5};
  double lidar_smoother_w_imu_{2.0};
  double lidar_smoother_w_scan_{1.0};
  double lidar_smoother_lambda_{0.5};
  double lidar_smoother_fitness_sigma_{1.0};
  double lidar_smoother_min_scan_weight_{0.1};
  double lidar_smoother_max_scan_weight_{5.0};
  bool lidar_smoother_zupt_enable_{false};
  double lidar_smoother_zupt_w_trans_{25.0};
  double lidar_smoother_zupt_w_yaw_{25.0};
  bool lidar_smoother_nhc_enable_{false};
  double lidar_smoother_nhc_w_lateral_{2.0};
  double lidar_smoother_nhc_huber_delta_m_{0.1};
  double lidar_smoother_local_huber_delta_xy_m_{0.35};
  double lidar_smoother_local_huber_delta_yaw_rad_{0.12};
  double lidar_smoother_max_position_correction_m_{2.0};
  double lidar_smoother_max_yaw_correction_rad_{0.35};
  bool lidar_smoother_hessian_enable_{true};
  double lidar_smoother_hessian_yaw_metric_m_{2.0};
  double lidar_smoother_hessian_min_direction_ratio_{1.0e-4};

  // Active local submap. In scan_to_submap mode it is the primary registration
  // target. In scan_to_scan mode, lidar_odom.local_map.enable retains the
  // optional periodic consistency factor from 0.2.0-rc1.
  bool lidar_local_map_enable_{false};
  int lidar_local_map_match_interval_frames_{5};
  int lidar_local_map_min_keyframes_{3};
  int lidar_local_map_max_keyframes_{15};
  int lidar_local_map_min_points_{200};
  int lidar_local_map_keyframe_min_interval_frames_{3};
  int lidar_local_map_keyframe_max_interval_frames_{20};
  double lidar_local_map_keyframe_min_translation_m_{0.75};
  double lidar_local_map_keyframe_min_yaw_rad_{0.15};
  double lidar_local_map_voxel_leaf_m_{0.35};
  int lidar_local_map_max_points_{200000};
  double lidar_local_map_max_corr_dist_m_{1.0};
  int lidar_local_map_max_iterations_{30};
  double lidar_local_map_max_fitness_{1.0};
  double lidar_local_map_min_inlier_ratio_{0.25};
  double lidar_local_map_max_correction_translation_m_{0.50};
  double lidar_local_map_max_correction_yaw_rad_{0.12};
  double lidar_local_map_max_correction_z_m_{0.25};
  double lidar_local_map_max_correction_roll_pitch_rad_{0.15};
  double lidar_local_map_factor_weight_xy_{6.0};
  double lidar_local_map_factor_weight_yaw_{10.0};
  double lidar_local_map_fitness_sigma_{0.50};

  // -------- ROS I/F --------
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_wheel_twist_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_ref_pose_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_raw_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_filtered_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stopped_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_corrected_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_observability_debug_;


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_deskew_twist_;
  std::string out_deskew_twist_topic_;

  // -------- State --------
  mutable std::mutex mtx_;
  std::mutex lidar_callback_mtx_;

  std::deque<ImuSample> imu_buf_;
  double bg_est_{0.0};
  rclcpp::Time last_imu_stamp_;
  bool has_last_imu_{false};

  // IMU-integrated yaw (bias corrected)
  double yaw_imu_{0.0};

  // Cached IMU extrinsic (base <- imu_frame)
  bool has_imu_extrinsic_{false};
  std::string imu_frame_id_;
  Eigen::Quaterniond q_base_imu_{1.0, 0.0, 0.0, 0.0};
  Eigen::Matrix3d R_base_imu_{Eigen::Matrix3d::Identity()};

  // Wheel speed
  std::deque<WheelSample> wheel_buf_;
  WheelSample last_wheel_;
  bool has_wheel_{false};
  double v_acc_est_{0.0};
  bool has_v_acc_{false};

  bool has_filtered_publish_state_{false};
  double filtered_pub_x_{0.0};
  double filtered_pub_y_{0.0};
  double filtered_pub_yaw_{0.0};
  double filtered_pub_dx_{0.0};
  double filtered_pub_dy_{0.0};
  double filtered_pub_dyaw_{0.0};
  double filtered_pub_vx_{0.0};
  double filtered_pub_vy_{0.0};
  double filtered_pub_yaw_rate_{0.0};
  rclcpp::Time last_filtered_publish_stamp_;

  // Online scale estimation
  bool has_ref_pose_{false};
  rclcpp::Time last_ref_stamp_;
  double last_ref_x_{0.0};
  double last_ref_y_{0.0};
  double wheel_dist_since_ref_{0.0};

  // LiDAR odometry
  LidarOdomSample last_lidar_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
  rclcpp::Time prev_cloud_stamp_;
  Eigen::Matrix4f last_gicp_guess_{Eigen::Matrix4f::Identity()};
  bool has_prev_cloud_{false};
  bool lidar_active_{false};

  // LiDAR extrinsic (base <- scan_frame)
  bool has_scan_extrinsic_{false};
  std::string scan_frame_id_;
  Eigen::Matrix4f T_base_scan_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f T_scan_base_{Eigen::Matrix4f::Identity()};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Integrated odom pose. It is updated only by accepted LiDAR/factor-graph steps.
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  bool has_odom_pose_{false};
  std::uint64_t lidar_pose_sequence_{0};
  bool next_icp_use_full_guess_{false};
  std::string last_icp_guess_mode_{"identity"};
  std::string next_icp_guess_mode_{"yaw_only"};
  double odom_cov_total_xy_{0.0};
  double odom_cov_total_yaw_{0.0};

  // Fixed-lag SE(2) factor graph (ROS/PCL independent and unit-tested).
  se2::FixedLagSmoother lidar_smoother_;
  bool lidar_smoother_initialized_{false};

  // Active submap in its own anchor frame. The odom<-anchor transform is
  // adjusted after smoothing; keyframe geometry stays in anchor coordinates.
  std::deque<LocalMapKeyframe> local_map_keyframes_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_cloud_;
  bool local_map_initialized_{false};
  se2::Pose local_map_odom_anchor_pose_{};
  se2::Pose local_map_last_tracking_anchor_pose_{};
  bool has_local_map_last_tracking_pose_{false};
  int local_map_frame_sequence_{0};
  int local_map_frames_since_keyframe_{0};
  int local_map_consecutive_failures_{0};
  std::uint64_t local_map_match_accepted_count_{0};
  std::uint64_t local_map_match_rejected_count_{0};
  std::uint64_t scan_to_submap_primary_count_{0};
  std::uint64_t scan_to_scan_interim_count_{0};
  std::uint64_t scan_to_scan_fallback_count_{0};
  std::uint64_t scan_to_scan_warmup_count_{0};
  LocalMapObservation last_local_map_observation_;
  std::string last_registration_source_{"none"};
  std::string local_map_reset_reason_{"startup"};

  // Stop state
  bool is_stopped_{false};
  bool has_stop_candidate_since_{false};
  rclcpp::Time stop_candidate_since_;
};

}  // namespace pure_gyro_odometer
