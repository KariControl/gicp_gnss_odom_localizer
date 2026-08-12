#include "pure_lidar_gyro_odometer/gyro_odometer_node.hpp"
#include "pure_lidar_gyro_odometer/accepted_scan_odometry.hpp"
#include "pure_lidar_gyro_odometer/imu_linear_acceleration.hpp"
#include "pure_lidar_gyro_odometer/accepted_scan_snapshot_policy.hpp"
#include "pure_lidar_gyro_odometer/observability_policy.hpp"

#include <functional>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <small_gicp/pcl/pcl_registration.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Eigenvalues>

namespace pure_gyro_odometer
{

static uint8_t diagLevelFromString(const std::string & level)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (level == "OK" || level == "ok") return DiagnosticStatus::OK;
  if (level == "WARN" || level == "warn") return DiagnosticStatus::WARN;
  if (level == "ERROR" || level == "error") return DiagnosticStatus::ERROR;
  if (level == "STALE" || level == "stale") return DiagnosticStatus::STALE;
  return DiagnosticStatus::WARN;
}

namespace
{

std::string boolString(bool v)
{
  return v ? "true" : "false";
}

std::string formatDouble(double v, int precision = 6)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << v;
  return oss.str();
}

double clamp01(double v)
{
  return std::max(0.0, std::min(1.0, v));
}

double wrapYaw(double a)
{
  constexpr double kPi = 3.14159265358979323846;
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3d = Eigen::Matrix3d;

struct DirectionalInformationAnalysis
{
  bool valid{false};
  Matrix3d normalized_information{Matrix3d::Identity()};
  Matrix3d weakness_projector{Matrix3d::Zero()};
  Eigen::Vector3d information_ratios{Eigen::Vector3d::Ones()};
  double information_deficit{0.0};
};

Matrix6d adjointRotationFirst(const Eigen::Matrix4d & T)
{
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  const Eigen::Vector3d t = T.block<3, 1>(0, 3);

  Eigen::Matrix3d tx = Eigen::Matrix3d::Zero();
  tx(0, 1) = -t.z();
  tx(0, 2) = t.y();
  tx(1, 0) = t.z();
  tx(1, 2) = -t.x();
  tx(2, 0) = -t.y();
  tx(2, 1) = t.x();

  Matrix6d A = Matrix6d::Zero();
  A.block<3, 3>(0, 0) = R;
  A.block<3, 3>(3, 0) = tx * R;
  A.block<3, 3>(3, 3) = R;
  return A;
}

Matrix3d symmetrize(const Matrix3d & M)
{
  return 0.5 * (M + M.transpose());
}

bool reduceHessianToSe2Info(const Matrix6d & H_base, Matrix3d & out_info)
{
  constexpr std::array<int, 3> q_idx{{3, 4, 2}};  // tx, ty, rz
  constexpr std::array<int, 3> n_idx{{0, 1, 5}};  // rx, ry, tz

  Matrix3d A = Matrix3d::Zero();
  Matrix3d B = Matrix3d::Zero();
  Matrix3d C = Matrix3d::Zero();

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      A(r, c) = H_base(q_idx[r], q_idx[c]);
      B(r, c) = H_base(q_idx[r], n_idx[c]);
      C(r, c) = H_base(n_idx[r], n_idx[c]);
    }
  }

  Matrix3d schur = symmetrize(A);
  Eigen::LDLT<Matrix3d> ldlt(symmetrize(C) + 1e-9 * Matrix3d::Identity());
  if (ldlt.info() == Eigen::Success) {
    schur = symmetrize(A - B * ldlt.solve(B.transpose()));
  }

  Eigen::SelfAdjointEigenSolver<Matrix3d> eig(schur);
  if (eig.info() != Eigen::Success) {
    return false;
  }

  const Eigen::Vector3d evals = eig.eigenvalues().cwiseMax(0.0);
  out_info = eig.eigenvectors() * evals.asDiagonal() * eig.eigenvectors().transpose();
  return out_info.allFinite();
}

Eigen::Vector3d arcMotionPrior(double distance, double dyaw)
{
  if (!std::isfinite(distance) || !std::isfinite(dyaw)) {
    return Eigen::Vector3d::Zero();
  }

  if (std::fabs(dyaw) < 1e-6) {
    return Eigen::Vector3d(distance, 0.0, dyaw);
  }

  const double dx = distance * std::sin(dyaw) / dyaw;
  const double dy = distance * (1.0 - std::cos(dyaw)) / dyaw;
  return Eigen::Vector3d(dx, dy, dyaw);
}

Eigen::Matrix4f se2DeltaToScanGuess(
  const Eigen::Vector3d & delta_base,
  const Eigen::Matrix4f & T_base_scan,
  const Eigen::Matrix4f & T_scan_base)
{
  Eigen::Matrix4f T_prev_curr_base = Eigen::Matrix4f::Identity();
  T_prev_curr_base(0, 3) = static_cast<float>(delta_base.x());
  T_prev_curr_base(1, 3) = static_cast<float>(delta_base.y());
  T_prev_curr_base.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(static_cast<float>(delta_base.z()), Eigen::Vector3f::UnitZ()).toRotationMatrix();

  Eigen::Matrix4f T_prev_curr_scan = T_scan_base * T_prev_curr_base * T_base_scan;
  if (!T_prev_curr_scan.allFinite()) {
    return Eigen::Matrix4f::Identity();
  }
  return T_prev_curr_scan;
}

Eigen::Vector3d fuseWeakDirections(
  const Eigen::Vector3d & scan_delta,
  const Eigen::Vector3d & prior_delta,
  const Matrix3d & weak_projector_param)
{
  return scan_delta + weak_projector_param * (prior_delta - scan_delta);
}

double se2MetricNorm(const Eigen::Vector3d & delta, double yaw_metric_m)
{
  const double L = std::max(1e-3, yaw_metric_m);
  return std::sqrt(
    delta.x() * delta.x() + delta.y() * delta.y() +
    (L * delta.z()) * (L * delta.z()));
}

void integrateSe2Delta(double & x, double & y, double & yaw, const Eigen::Vector3d & delta_local)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  x += c * delta_local.x() - s * delta_local.y();
  y += s * delta_local.x() + c * delta_local.y();
  yaw = wrapYaw(yaw + delta_local.z());
}

DirectionalInformationAnalysis analyzeDirectionalInformation(
  const Matrix3d & information_se2,
  double yaw_metric_m,
  double minimum_direction_ratio,
  double weakness_power)
{
  DirectionalInformationAnalysis analysis;
  const double metric = std::max(1.0e-3, yaw_metric_m);
  const double floor_ratio = clamp01(minimum_direction_ratio);
  const double power = std::max(1.0e-3, weakness_power);
  const Matrix3d scale = (Matrix3d() <<
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, metric).finished();
  const Matrix3d inverse_scale = (Matrix3d() <<
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0 / metric).finished();
  const Matrix3d metric_information =
    symmetrize(inverse_scale.transpose() * information_se2 * inverse_scale);

  Eigen::SelfAdjointEigenSolver<Matrix3d> eigen_solver(metric_information);
  if (eigen_solver.info() != Eigen::Success ||
    !eigen_solver.eigenvalues().allFinite() ||
    !eigen_solver.eigenvectors().allFinite())
  {
    return analysis;
  }

  const Eigen::Vector3d eigenvalues = eigen_solver.eigenvalues().cwiseMax(0.0);
  const double maximum = eigenvalues.maxCoeff();
  if (!std::isfinite(maximum) || maximum <= 1.0e-12) {
    return analysis;
  }

  analysis.information_ratios = (eigenvalues / maximum).cwiseMax(0.0).cwiseMin(1.0);
  Eigen::Vector3d floored_ratios;
  Eigen::Vector3d weakness;
  for (int index = 0; index < 3; ++index) {
    floored_ratios(index) = std::max(floor_ratio, analysis.information_ratios(index));
    weakness(index) = observability::weaknessWeight(
      analysis.information_ratios(index), power);
  }

  const Matrix3d normalized_metric_information =
    eigen_solver.eigenvectors() * floored_ratios.asDiagonal() *
    eigen_solver.eigenvectors().transpose();
  analysis.normalized_information =
    symmetrize(scale.transpose() * normalized_metric_information * scale);

  const Matrix3d weakness_metric =
    eigen_solver.eigenvectors() * weakness.asDiagonal() *
    eigen_solver.eigenvectors().transpose();
  analysis.weakness_projector = inverse_scale * weakness_metric * scale;
  analysis.information_deficit =
    ((Eigen::Vector3d::Ones() - analysis.information_ratios).sum()) / 3.0;
  analysis.valid = analysis.normalized_information.allFinite() &&
    analysis.weakness_projector.allFinite() &&
    std::isfinite(analysis.information_deficit);
  return analysis;
}

std::array<double, 9> matrixToArray(const Matrix3d & matrix)
{
  return {{
    matrix(0, 0), matrix(0, 1), matrix(0, 2),
    matrix(1, 0), matrix(1, 1), matrix(1, 2),
    matrix(2, 0), matrix(2, 1), matrix(2, 2)}};
}

void rotationToRollPitchYaw(
  const Eigen::Matrix3f & rotation, double & roll, double & pitch, double & yaw)
{
  tf2::Matrix3x3 matrix(
    static_cast<double>(rotation(0, 0)), static_cast<double>(rotation(0, 1)),
    static_cast<double>(rotation(0, 2)), static_cast<double>(rotation(1, 0)),
    static_cast<double>(rotation(1, 1)), static_cast<double>(rotation(1, 2)),
    static_cast<double>(rotation(2, 0)), static_cast<double>(rotation(2, 1)),
    static_cast<double>(rotation(2, 2)));
  matrix.getRPY(roll, pitch, yaw);
  yaw = wrapYaw(yaw);
}

}  // namespace

double GyroOdometerNode::normalizeYaw(double a)
{
  constexpr double kPi = 3.14159265358979323846;
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

double GyroOdometerNode::yawFromRot(const Eigen::Matrix3d & R)
{
  return normalizeYaw(std::atan2(R(1, 0), R(0, 0)));
}

Eigen::Quaterniond GyroOdometerNode::quatFromYaw(double yaw)
{
  Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  q.normalize();
  return q;
}

GyroOdometerNode::GyroOdometerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("gyro_odometer", options)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Frames
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");

  // Topics
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
  wheel_speed_topic_ = declare_parameter<std::string>("wheel_speed_topic", "");
  reference_pose_topic_ = declare_parameter<std::string>("reference_pose_topic", "");
  points_topic_ = declare_parameter<std::string>("points_topic", "/localization/points_undistorted");

  out_odom_topic_ = declare_parameter<std::string>("out_odom_topic", "/localization/gyro_lidar_odom");
  out_filtered_odom_topic_ = declare_parameter<std::string>("out_filtered_odom_topic", "/localization/gyro_lidar_odom_filtered");
  out_stopped_topic_ = declare_parameter<std::string>("out_stopped_topic", "/localization/is_stopped");
  out_imu_topic_ = declare_parameter<std::string>("out_imu_topic", "/localization/imu_corrected");

  imu_corrected_enable_ = declare_parameter<bool>("imu_corrected.enable", true);
  imu_corrected_apply_tf_ = declare_parameter<bool>("imu_corrected.apply_tf", true);
  imu_corrected_transform_orientation_ =
    declare_parameter<bool>("imu_corrected.transform_orientation", false);
  imu_linear_acceleration_scale_ = declare_parameter<double>(
    "imu_corrected.linear_acceleration_scale", imu_linear_acceleration_scale_);
  imu_max_abs_yaw_rate_radps_ = declare_parameter<double>(
    "imu_corrected.max_abs_yaw_rate_radps", imu_max_abs_yaw_rate_radps_);
  imu_max_sample_gap_sec_ = declare_parameter<double>(
    "imu_corrected.max_sample_gap_sec", imu_max_sample_gap_sec_);
  imu_max_boundary_gap_sec_ = declare_parameter<double>(
    "imu_corrected.max_boundary_gap_sec", imu_max_boundary_gap_sec_);

  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
  out_filtered_odom_enable_ =
    declare_parameter<bool>("out_filtered_odom.enable", false);
  filtered_odom_zero_when_stopped_ =
    declare_parameter<bool>("out_filtered_odom.zero_when_stopped", true);
  filtered_odom_lowpass_alpha_ =
    declare_parameter<double>("out_filtered_odom.lowpass_alpha", 0.85);
  filtered_odom_linear_rate_limit_mps2_ =
    declare_parameter<double>("out_filtered_odom.linear_rate_limit_mps2", 4.0);
  filtered_odom_lateral_rate_limit_mps2_ =
    declare_parameter<double>("out_filtered_odom.lateral_rate_limit_mps2", 2.0);
  filtered_odom_yaw_rate_limit_radps2_ =
    declare_parameter<double>("out_filtered_odom.yaw_rate_limit_radps2", 1.5);
  filtered_odom_reset_gap_sec_ =
    declare_parameter<double>("out_filtered_odom.reset_gap_sec", 1.0);

  // Stop detection
  stop_enable_ = declare_parameter<bool>("stop.enable", true);
  stop_speed_thr_mps_ = declare_parameter<double>("stop.speed_thr_mps", 0.15);
  stop_gyro_abs_thr_rad_s_ = declare_parameter<double>("stop.gyro_abs_thr_rad_s", 0.05);
  stop_acc_var_thr_ = declare_parameter<double>("stop.acc_var_thr", 0.0225);
  stop_hold_sec_ = declare_parameter<double>("stop.hold_sec", 0.5);

  // Gyro bias
  gyro_bias_enable_ = declare_parameter<bool>("gyro_bias.enable", true);
  gyro_bias_tau_sec_ = declare_parameter<double>("gyro_bias.tau_sec", 10.0);
  gyro_bias_max_abs_rad_s_ = declare_parameter<double>("gyro_bias.max_abs_rad_s", 0.5);
  bg_est_ = declare_parameter<double>("gyro_bias.initial_bg_rad_s", 0.0);

  // Wheel speed
  use_wheel_speed_ = declare_parameter<bool>("wheel_speed.use", false);
  wheel_speed_scale_ = declare_parameter<double>("wheel_speed.scale_factor", 1.0);
  wheel_speed_timeout_sec_ = declare_parameter<double>("wheel_speed.timeout_sec", 0.2);

  wheel_low_speed_enable_ = declare_parameter<bool>("wheel_speed.low_speed.enable", false);
  wheel_low_speed_deadband_mps_ = declare_parameter<double>("wheel_speed.low_speed.deadband_mps", 0.12);
  wheel_low_speed_acc_thr_mps2_ = declare_parameter<double>("wheel_speed.low_speed.acc_thr_mps2", 0.2);
  wheel_low_speed_blend_ = declare_parameter<double>("wheel_speed.low_speed.blend", 0.5);
  wheel_low_speed_max_corr_mps_ = declare_parameter<double>("wheel_speed.low_speed.max_corr_mps", 0.3);

  wheel_scale_est_enable_ = declare_parameter<bool>("wheel_speed.scale_estimation.enable", false);
  wheel_scale_est_tau_sec_ = declare_parameter<double>("wheel_speed.scale_estimation.tau_sec", 30.0);
  wheel_scale_est_min_ref_dist_m_ =
    declare_parameter<double>("wheel_speed.scale_estimation.min_ref_dist_m", 2.0);
  wheel_scale_est_min_wheel_dist_m_ =
    declare_parameter<double>("wheel_speed.scale_estimation.min_wheel_dist_m", 1.0);
  wheel_scale_min_ = declare_parameter<double>("wheel_speed.scale_estimation.min_scale", 0.5);
  wheel_scale_max_ = declare_parameter<double>("wheel_speed.scale_estimation.max_scale", 2.0);

  wheel_observability_assist_enable_ = declare_parameter<bool>(
    "wheel_speed.observability_assist.enable", false);
  wheel_observability_assist_min_wheel_dist_m_ = declare_parameter<double>(
    "wheel_speed.observability_assist.min_wheel_dist_m", 0.1);
  wheel_observability_assist_max_blend_ = declare_parameter<double>(
    "wheel_speed.observability_assist.max_blend", 0.5);
  wheel_observability_assist_power_ = declare_parameter<double>(
    "wheel_speed.observability_assist.power", 2.0);
  wheel_registration_recovery_use_current_prior_ = declare_parameter<bool>(
    "wheel_speed.registration_recovery.use_current_prior", true);
  lidar_observability_debug_pub_enable_ = declare_parameter<bool>(
    "lidar_odom.observability.debug_pub.enable", false);
  lidar_observability_debug_topic_ = declare_parameter<std::string>(
    "lidar_odom.observability.debug_pub.topic",
    "/localization/lidar_observability_debug");

  odom_cov_base_xy_step_ = declare_parameter<double>("odom_covariance.base_xy_step", odom_cov_base_xy_step_);
  odom_cov_base_yaw_step_ = declare_parameter<double>("odom_covariance.base_yaw_step", odom_cov_base_yaw_step_);
  odom_cov_xy_per_meter_ = declare_parameter<double>("odom_covariance.xy_per_meter", odom_cov_xy_per_meter_);
  odom_cov_yaw_per_rad_ = declare_parameter<double>("odom_covariance.yaw_per_rad", odom_cov_yaw_per_rad_);
  odom_cov_fitness_xy_scale_ =
    declare_parameter<double>("odom_covariance.fitness_xy_scale", odom_cov_fitness_xy_scale_);
  odom_cov_fitness_yaw_scale_ =
    declare_parameter<double>("odom_covariance.fitness_yaw_scale", odom_cov_fitness_yaw_scale_);
  odom_cov_observability_max_scale_ = declare_parameter<double>(
    "odom_covariance.observability_max_scale", odom_cov_observability_max_scale_);
  odom_cov_wheel_assist_scale_ =
    declare_parameter<double>("odom_covariance.wheel_assist_scale", odom_cov_wheel_assist_scale_);
  odom_cov_invalid_xy_step_ =
    declare_parameter<double>("odom_covariance.invalid_xy_step", odom_cov_invalid_xy_step_);
  odom_cov_invalid_yaw_step_ =
    declare_parameter<double>("odom_covariance.invalid_yaw_step", odom_cov_invalid_yaw_step_);
  odom_cov_deadreckon_xy_per_sec_ = declare_parameter<double>(
    "odom_covariance.deadreckon_xy_per_sec", odom_cov_deadreckon_xy_per_sec_);
  odom_cov_deadreckon_yaw_per_sec_ = declare_parameter<double>(
    "odom_covariance.deadreckon_yaw_per_sec", odom_cov_deadreckon_yaw_per_sec_);

  // LiDAR odometry. Internal registration is scan-to-scan only; isolated
  // precision submap matching consumes the accepted-scan snapshot output.
  lidar_odom_enable_ = declare_parameter<bool>("lidar_odom.enable", true);
  lidar_backend_ = declare_parameter<std::string>("lidar_odom.backend", lidar_backend_);
  lidar_registration_type_ = declare_parameter<std::string>("lidar_odom.registration_type", lidar_registration_type_);
  lidar_num_threads_ = declare_parameter<int>("lidar_odom.num_threads", lidar_num_threads_);
  lidar_timeout_sec_ = declare_parameter<double>("lidar_odom.timeout_sec", 0.5);
  lidar_min_range_m_ = declare_parameter<double>("lidar_odom.min_range_m", 2.0);
  lidar_max_range_m_ = declare_parameter<double>("lidar_odom.max_range_m", 80.0);
  lidar_voxel_leaf_m_ = declare_parameter<double>("lidar_odom.voxel_leaf_m", 0.4);

  gicp_max_corr_dist_m_ = declare_parameter<double>("lidar_odom.gicp.max_corr_dist_m", 2.5);
  gicp_max_iterations_ = declare_parameter<int>("lidar_odom.gicp.max_iterations", 30);
  gicp_trans_eps_ = declare_parameter<double>("lidar_odom.gicp.trans_eps", 1e-3);
  gicp_rot_eps_ = declare_parameter<double>("lidar_odom.gicp.rot_eps", 2e-3);
  gicp_corr_randomness_ = declare_parameter<int>("lidar_odom.gicp.corr_randomness", 20);
  gicp_fitness_max_ = declare_parameter<double>("lidar_odom.gicp.max_fitness", 5.0);
  gicp_voxel_resolution_ = declare_parameter<double>("lidar_odom.gicp.voxel_resolution", 1.0);
  lidar_pose_se2_enable_ = declare_parameter<bool>("lidar_odom.pose_se2.enable", true);
  lidar_yaw_blend_imu_ = declare_parameter<double>("lidar_odom.pose_se2.yaw_blend_imu", 0.0);
  lidar_guess_use_imu_yaw_only_ = declare_parameter<bool>("lidar_odom.pose_se2.guess_use_imu_yaw_only", true);
  lidar_tracking_mode_name_ = declare_parameter<std::string>(
    "lidar_odom.tracking_mode", lidar_tracking_mode_name_);
  lidar_tracking_mode_name_ = tracking::toString(
    tracking::parseMode(lidar_tracking_mode_name_));

  lidar_smoother_enable_ = declare_parameter<bool>("lidar_odom.smoother.enable", true);
  lidar_smoother_window_size_ = declare_parameter<int>("lidar_odom.smoother.window_size", 20);
  lidar_smoother_max_iter_ = declare_parameter<int>("lidar_odom.smoother.max_iterations", 5);
  lidar_smoother_w_imu_ = declare_parameter<double>("lidar_odom.smoother.w_imu", 3.0);
  lidar_smoother_w_scan_ = declare_parameter<double>("lidar_odom.smoother.w_scan", 1.0);
  lidar_smoother_lambda_ = declare_parameter<double>("lidar_odom.smoother.lambda", 0.5);
  lidar_smoother_fitness_sigma_ = declare_parameter<double>("lidar_odom.smoother.fitness_sigma", 1.0);
  lidar_smoother_min_scan_weight_ = declare_parameter<double>("lidar_odom.smoother.min_scan_weight", 0.05);
  lidar_smoother_max_scan_weight_ = declare_parameter<double>("lidar_odom.smoother.max_scan_weight", 5.0);
  lidar_smoother_zupt_enable_ = declare_parameter<bool>("lidar_odom.smoother.zupt.enable", false);
  lidar_smoother_zupt_w_trans_ = declare_parameter<double>("lidar_odom.smoother.zupt.w_trans", 25.0);
  lidar_smoother_zupt_w_yaw_ = declare_parameter<double>("lidar_odom.smoother.zupt.w_yaw", 25.0);
  lidar_smoother_nhc_enable_ = declare_parameter<bool>("lidar_odom.smoother.nhc.enable", false);
  lidar_smoother_nhc_w_lateral_ = declare_parameter<double>("lidar_odom.smoother.nhc.w_lateral", 2.0);
  lidar_smoother_nhc_huber_delta_m_ = declare_parameter<double>("lidar_odom.smoother.nhc.huber_delta_m", 0.10);

  lidar_smoother_max_position_correction_m_ = declare_parameter<double>(
    "lidar_odom.smoother.max_solution_position_correction_m",
    lidar_smoother_max_position_correction_m_);
  lidar_smoother_max_yaw_correction_rad_ = declare_parameter<double>(
    "lidar_odom.smoother.max_solution_yaw_correction_rad",
    lidar_smoother_max_yaw_correction_rad_);
  lidar_smoother_hessian_enable_ = declare_parameter<bool>(
    "lidar_odom.smoother.hessian_information.enable", lidar_smoother_hessian_enable_);
  lidar_smoother_hessian_yaw_metric_m_ = declare_parameter<double>(
    "lidar_odom.smoother.hessian_information.yaw_metric_m",
    lidar_smoother_hessian_yaw_metric_m_);
  lidar_smoother_hessian_min_direction_ratio_ = declare_parameter<double>(
    "lidar_odom.smoother.hessian_information.min_direction_ratio",
    lidar_smoother_hessian_min_direction_ratio_);

  se2::Config smoother_config;
  smoother_config.window_size = lidar_smoother_window_size_;
  smoother_config.max_iterations = lidar_smoother_max_iter_;
  smoother_config.scan_weight = lidar_smoother_w_scan_;
  smoother_config.imu_weight = lidar_smoother_w_imu_;
  smoother_config.smoothness_weight = lidar_smoother_lambda_;
  smoother_config.fitness_sigma = lidar_smoother_fitness_sigma_;
  smoother_config.min_scan_weight = lidar_smoother_min_scan_weight_;
  smoother_config.max_scan_weight = lidar_smoother_max_scan_weight_;
  smoother_config.zupt_enable = lidar_smoother_zupt_enable_;
  smoother_config.zupt_weight_translation = lidar_smoother_zupt_w_trans_;
  smoother_config.zupt_weight_yaw = lidar_smoother_zupt_w_yaw_;
  smoother_config.nhc_enable = lidar_smoother_nhc_enable_;
  smoother_config.nhc_weight_lateral = lidar_smoother_nhc_w_lateral_;
  smoother_config.nhc_huber_delta_m = lidar_smoother_nhc_huber_delta_m_;
  smoother_config.max_solution_position_correction_m =
    lidar_smoother_max_position_correction_m_;
  smoother_config.max_solution_yaw_correction_rad = lidar_smoother_max_yaw_correction_rad_;
  lidar_smoother_.setConfig(smoother_config);

  external_submap_snapshot_enable_ = declare_parameter<bool>(
    "lidar_odom.external_submap_snapshot.enable", false);
  external_submap_snapshot_topic_ = declare_parameter<std::string>(
    "lidar_odom.external_submap_snapshot.topic",
    external_submap_snapshot_topic_);
  external_submap_snapshot_publish_interval_frames_ = declare_parameter<int>(
    "lidar_odom.external_submap_snapshot.publish_interval_frames",
    external_submap_snapshot_publish_interval_frames_);
  accepted_scan_odom_enable_ = declare_parameter<bool>(
    "lidar_odom.accepted_scan_odom.enable", false);
  accepted_scan_odom_topic_ = declare_parameter<std::string>(
    "lidar_odom.accepted_scan_odom.topic", accepted_scan_odom_topic_);
  if (external_submap_snapshot_enable_) {
    external_submap_odom_session_id_ = static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
    if (external_submap_odom_session_id_ == 0) {
      external_submap_odom_session_id_ = 1;
    }
    external_submap_odom_generation_ = 1;
  }

  validateParameters();
  RCLCPP_INFO(
    get_logger(), "LiDAR tracking mode: %s", lidar_tracking_mode_name_.c_str());

  if (wheel_observability_assist_enable_) {
    const bool backend_has_hessian =
      lidar_backend_ == "SMALL_GICP" || lidar_backend_ == "small_gicp" ||
      lidar_backend_ == "VGICP" || lidar_backend_ == "GICP";
    if (!backend_has_hessian) {
      RCLCPP_WARN(
        get_logger(),
        "wheel_speed.observability_assist.enable=true, but lidar_odom.backend=%s "
        "does not expose a small_gicp Hessian. Continuous wheel assist will remain inactive.",
        lidar_backend_.c_str());
    }
    if (!use_wheel_speed_) {
      RCLCPP_WARN(
        get_logger(),
        "wheel_speed.observability_assist.enable=true while wheel_speed.use=false. "
        "LiDAR/IMU localization continues without wheel correction.");
    }
  }
  if (lidar_observability_debug_pub_enable_ && lidar_observability_debug_topic_.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "lidar_odom.observability.debug_pub.enable=true but its topic is empty. "
      "Disabling the observability debug publisher.");
    lidar_observability_debug_pub_enable_ = false;
  }

  // Publishers
  pub_odom_raw_ = create_publisher<nav_msgs::msg::Odometry>(out_odom_topic_, 10);
  if (accepted_scan_odom_enable_) {
    pub_accepted_scan_odom_ =
      create_publisher<nav_msgs::msg::Odometry>(accepted_scan_odom_topic_, 10);
  }
  if (external_submap_snapshot_enable_) {
    pub_external_submap_snapshot_ =
      create_publisher<pure_lidar_msgs::msg::SubmapScan>(
      external_submap_snapshot_topic_, rclcpp::SensorDataQoS().keep_last(1));
  }
  if (out_filtered_odom_enable_ && !out_filtered_odom_topic_.empty() && out_filtered_odom_topic_ != out_odom_topic_) {
    pub_odom_filtered_ = create_publisher<nav_msgs::msg::Odometry>(out_filtered_odom_topic_, 10);
  } else if (out_filtered_odom_enable_ && !out_filtered_odom_topic_.empty() && out_filtered_odom_topic_ == out_odom_topic_) {
    RCLCPP_WARN(get_logger(), "out_filtered_odom_topic matches out_odom_topic; filtered odom publisher is disabled.");
  }
  pub_stopped_ = create_publisher<std_msgs::msg::Bool>(out_stopped_topic_, 10);
  if (imu_corrected_enable_) {
    pub_imu_corrected_ = create_publisher<sensor_msgs::msg::Imu>(out_imu_topic_, rclcpp::SensorDataQoS());
  }
  pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  if (lidar_observability_debug_pub_enable_) {
    pub_observability_debug_ = create_publisher<std_msgs::msg::String>(
      lidar_observability_debug_topic_, 10);
  }

  out_deskew_twist_topic_ =
    declare_parameter<std::string>("out_deskew_twist_topic", "/localization/deskew_twist");

  pub_deskew_twist_ =
    create_publisher<geometry_msgs::msg::TwistStamped>(
      out_deskew_twist_topic_, rclcpp::SensorDataQoS());

  // Subscribers
  // A GICP callback can take longer than one IMU period. Keep IMU delivery
  // independent from registration, while preventing the multithreaded
  // component executor from starting a second registration concurrently.
  imu_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  lidar_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auxiliary_sensor_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions imu_subscription_options;
  imu_subscription_options.callback_group = imu_callback_group_;
  rclcpp::SubscriptionOptions lidar_subscription_options;
  lidar_subscription_options.callback_group = lidar_callback_group_;
  rclcpp::SubscriptionOptions auxiliary_subscription_options;
  auxiliary_subscription_options.callback_group = auxiliary_sensor_callback_group_;
  rclcpp::SensorDataQoS imu_input_qos;
  // At 200 Hz, the default SensorDataQoS depth of five represents only 25 ms.
  // Keep enough IMU history queued while a GICP callback is using the CPU.
  imu_input_qos.keep_last(200);
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, imu_input_qos,
    std::bind(&GyroOdometerNode::onImu, this, std::placeholders::_1),
    imu_subscription_options);

  if (use_wheel_speed_) {
    if (wheel_speed_topic_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "wheel_speed.use is true but wheel_speed_topic is empty. Wheel assist will stay inactive; LiDAR-only localization continues.");
    } else {
      sub_wheel_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        wheel_speed_topic_, rclcpp::SensorDataQoS(),
        std::bind(&GyroOdometerNode::onWheelTwist, this, std::placeholders::_1),
        auxiliary_subscription_options);
    }
  }

  if (wheel_scale_est_enable_) {
    if (reference_pose_topic_.empty()) {
      RCLCPP_WARN(get_logger(), "wheel_speed.scale_estimation.enable is true but reference_pose_topic is empty. Disabling.");
      wheel_scale_est_enable_ = false;
    } else {
      sub_ref_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        reference_pose_topic_, 10,
        std::bind(&GyroOdometerNode::onReferencePose, this, std::placeholders::_1),
        auxiliary_subscription_options);
    }
  }

  if (lidar_odom_enable_) {
    sub_points_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GyroOdometerNode::onPoints, this, std::placeholders::_1),
      lidar_subscription_options);
    lidar_active_ = true;
  } else {
    lidar_active_ = false;
  }

  // Timer
  const double period = (publish_rate_hz_ > 1e-3) ? (1.0 / publish_rate_hz_) : 0.02;
  timer_ = create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&GyroOdometerNode::onPublishTimer, this));

  publishDiagnostics(now(), "OK", "pure_gyro_odometer started");
}

void GyroOdometerNode::onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const std::string imu_frame = msg->header.frame_id;
  if (imu_frame.empty()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[pure_gyro_odometer] Rejecting IMU with empty frame_id.");
    return;
  }

  const bool values_finite =
    std::isfinite(msg->angular_velocity.x) && std::isfinite(msg->angular_velocity.y) &&
    std::isfinite(msg->angular_velocity.z) && std::isfinite(msg->linear_acceleration.x) &&
    std::isfinite(msg->linear_acceleration.y) && std::isfinite(msg->linear_acceleration.z);
  if (!values_finite) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[pure_gyro_odometer] Rejecting IMU with non-finite angular velocity or acceleration.");
    return;
  }

  Eigen::Quaterniond q_base_imu = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d R_base_imu = Eigen::Matrix3d::Identity();
  if (imu_frame != base_frame_) {
    if (!imu_corrected_apply_tf_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[pure_gyro_odometer] IMU frame '%s' differs from base_frame '%s' while "
        "imu_corrected.apply_tf=false. Rejecting the sample.",
        imu_frame.c_str(), base_frame_.c_str());
      return;
    }

    bool cached = false;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      cached = has_imu_extrinsic_ && imu_frame_id_ == imu_frame;
      if (cached) {
        q_base_imu = q_base_imu_;
        R_base_imu = R_base_imu_;
      }
    }

    if (!cached) {
      try {
        const auto transform =
          tf_buffer_->lookupTransform(base_frame_, imu_frame, tf2::TimePointZero);
        q_base_imu = Eigen::Quaterniond(
          static_cast<double>(transform.transform.rotation.w),
          static_cast<double>(transform.transform.rotation.x),
          static_cast<double>(transform.transform.rotation.y),
          static_cast<double>(transform.transform.rotation.z));
        if (!q_base_imu.coeffs().allFinite() || q_base_imu.norm() < 1.0e-9) {
          throw tf2::TransformException("non-finite or zero-norm IMU rotation");
        }
        q_base_imu.normalize();
        R_base_imu = q_base_imu.toRotationMatrix();

        std::lock_guard<std::mutex> lock(mtx_);
        has_imu_extrinsic_ = true;
        imu_frame_id_ = imu_frame;
        q_base_imu_ = q_base_imu;
        R_base_imu_ = R_base_imu;
      } catch (const tf2::TransformException & exception) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[pure_gyro_odometer] TF lookup failed for %s <- %s: %s. Rejecting IMU sample.",
          base_frame_.c_str(), imu_frame.c_str(), exception.what());
        return;
      }
    }
  }

  const Eigen::Vector3d gyro_imu(
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  const Eigen::Vector3d acceleration_imu(
    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const Eigen::Vector3d gyro_base = R_base_imu * gyro_imu;
  const Eigen::Vector3d acceleration_base = imu::transformLinearAcceleration(
    R_base_imu, acceleration_imu, imu_linear_acceleration_scale_);
  if (!gyro_base.allFinite() || !acceleration_base.allFinite() ||
    std::fabs(gyro_base.z()) > imu_max_abs_yaw_rate_radps_)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[pure_gyro_odometer] Rejecting transformed IMU: non-finite value or |yaw_rate| exceeds %.3f rad/s.",
      imu_max_abs_yaw_rate_radps_);
    return;
  }

  ImuSample sample;
  sample.stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
  if (sample.stamp.nanoseconds() == 0) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[pure_gyro_odometer] Rejecting IMU with zero timestamp.");
    return;
  }
  sample.gyro_z = gyro_base.z();
  sample.acc = acceleration_base;

  double bias_for_output = 0.0;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (has_last_imu_ && sample.stamp <= last_imu_stamp_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[pure_gyro_odometer] Rejecting out-of-order IMU sample (current=%.9f, last=%.9f).",
        sample.stamp.seconds(), last_imu_stamp_.seconds());
      return;
    }

    const bool had_previous = has_last_imu_;
    const double dt = had_previous ? (sample.stamp - last_imu_stamp_).seconds() : 0.0;
    const double previous_corrected_rate =
      !imu_buf_.empty() ? imu_buf_.back().yaw_rate_corrected : (sample.gyro_z - bg_est_);

    if (had_previous && std::isfinite(dt) && dt > 0.0) {
      updateStopState(sample.stamp);
      if (gyro_bias_enable_ && is_stopped_) {
        const double tau = std::max(1.0e-6, gyro_bias_tau_sec_);
        const double alpha = dt / (tau + dt);
        bg_est_ = (1.0 - alpha) * bg_est_ + alpha * sample.gyro_z;
        bg_est_ = std::max(
          -gyro_bias_max_abs_rad_s_, std::min(gyro_bias_max_abs_rad_s_, bg_est_));
      }
    }

    sample.yaw_rate_corrected = sample.gyro_z - bg_est_;
    if (had_previous && std::isfinite(dt) && dt > 0.0 && dt <= imu_max_sample_gap_sec_) {
      yaw_imu_ = normalizeYaw(
        yaw_imu_ + 0.5 * (previous_corrected_rate + sample.yaw_rate_corrected) * dt);
      if (has_v_acc_) {
        const double acceleration_x = std::max(-5.0, std::min(5.0, sample.acc.x()));
        v_acc_est_ = std::max(-50.0, std::min(50.0, v_acc_est_ + acceleration_x * dt));
      }
    } else if (had_previous && dt > imu_max_sample_gap_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[pure_gyro_odometer] IMU gap %.6f s exceeds %.6f s; yaw integration was not bridged.",
        dt, imu_max_sample_gap_sec_);
    }

    if (!had_previous) {
      v_acc_est_ = 0.0;
      has_v_acc_ = true;
      has_last_imu_ = true;
    }

    imu_buf_.push_back(sample);
    const double keep_sec = 5.0;
    while (!imu_buf_.empty() &&
      (sample.stamp - imu_buf_.front().stamp).seconds() > keep_sec)
    {
      imu_buf_.pop_front();
    }
    last_imu_stamp_ = sample.stamp;
    bias_for_output = bg_est_;
  }

  if (!imu_corrected_enable_ || !pub_imu_corrected_) return;

  sensor_msgs::msg::Imu output;
  output.header = msg->header;
  output.header.frame_id = base_frame_;
  output.angular_velocity.x = gyro_base.x();
  output.angular_velocity.y = gyro_base.y();
  output.angular_velocity.z = gyro_base.z() - bias_for_output;
  output.linear_acceleration.x = acceleration_base.x();
  output.linear_acceleration.y = acceleration_base.y();
  output.linear_acceleration.z = acceleration_base.z();
  output.angular_velocity_covariance = imu::rotateAndScaleCovariance(
    msg->angular_velocity_covariance, R_base_imu);
  output.linear_acceleration_covariance = imu::rotateAndScaleCovariance(
    msg->linear_acceleration_covariance, R_base_imu, imu_linear_acceleration_scale_);

  const Eigen::Quaterniond orientation_input(
    static_cast<double>(msg->orientation.w),
    static_cast<double>(msg->orientation.x),
    static_cast<double>(msg->orientation.y),
    static_cast<double>(msg->orientation.z));
  const bool orientation_known = msg->orientation_covariance[0] >= 0.0 &&
    orientation_input.coeffs().allFinite() && orientation_input.norm() > 1.0e-6;
  if (orientation_known && (imu_frame == base_frame_ || imu_corrected_transform_orientation_)) {
    Eigen::Quaterniond orientation = orientation_input.normalized();
    if (imu_frame != base_frame_) {
      orientation = orientation * q_base_imu.inverse();
      orientation.normalize();
      output.orientation_covariance = imu::rotateAndScaleCovariance(
        msg->orientation_covariance, R_base_imu);
    } else {
      output.orientation_covariance = msg->orientation_covariance;
    }
    output.orientation.w = orientation.w();
    output.orientation.x = orientation.x();
    output.orientation.y = orientation.y();
    output.orientation.z = orientation.z();
  } else {
    output.orientation.w = 1.0;
    output.orientation.x = 0.0;
    output.orientation.y = 0.0;
    output.orientation.z = 0.0;
    output.orientation_covariance.fill(0.0);
    output.orientation_covariance[0] = -1.0;
  }

  pub_imu_corrected_->publish(output);
}

void GyroOdometerNode::onWheelTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  double v_raw = msg->twist.linear.x;
  if (!std::isfinite(v_raw)) v_raw = 0.0;

  std::lock_guard<std::mutex> lk(mtx_);

  // accumulate distance for optional scale estimation
  if (wheel_scale_est_enable_ && has_wheel_) {
    const double dt = (stamp - last_wheel_.stamp).seconds();
    if (std::isfinite(dt) && dt > 0.0 && dt < 1.0) {
      wheel_dist_since_ref_ += std::fabs(v_raw) * dt;
    }
  }

  WheelSample sample;
  sample.stamp = stamp;
  sample.v_raw = v_raw;
  wheel_buf_.push_back(sample);
  while (!wheel_buf_.empty()) {
    if ((stamp - wheel_buf_.front().stamp).seconds() > 5.0) {
      wheel_buf_.pop_front();
    } else {
      break;
    }
  }

  last_wheel_ = sample;
  has_wheel_ = true;

  // anchor accel-integrated speed estimate to wheel speed when not in deadband
  if (wheel_low_speed_enable_) {
    if (std::fabs(v_raw) > wheel_low_speed_deadband_mps_) {
      v_acc_est_ = v_raw * wheel_speed_scale_;
      has_v_acc_ = true;
    }
  }
}

void GyroOdometerNode::onReferencePose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!wheel_scale_est_enable_) return;

  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;

  std::lock_guard<std::mutex> lk(mtx_);

  if (!has_ref_pose_) {
    has_ref_pose_ = true;
    last_ref_stamp_ = stamp;
    last_ref_x_ = x;
    last_ref_y_ = y;
    wheel_dist_since_ref_ = 0.0;
    return;
  }

  const double dt = (stamp - last_ref_stamp_).seconds();
  if (!std::isfinite(dt) || dt <= 0.0) {
    last_ref_stamp_ = stamp;
    last_ref_x_ = x;
    last_ref_y_ = y;
    wheel_dist_since_ref_ = 0.0;
    return;
  }

  const double ref_dist = std::hypot(x - last_ref_x_, y - last_ref_y_);

  if (ref_dist >= wheel_scale_est_min_ref_dist_m_ && wheel_dist_since_ref_ >= wheel_scale_est_min_wheel_dist_m_) {
    const double k_meas = ref_dist / std::max(1e-6, wheel_dist_since_ref_);
    const double alpha = dt / (wheel_scale_est_tau_sec_ + dt);
    const double k_new = (1.0 - alpha) * wheel_speed_scale_ + alpha * k_meas;
    wheel_speed_scale_ = std::max(wheel_scale_min_, std::min(wheel_scale_max_, k_new));
  }

  last_ref_stamp_ = stamp;
  last_ref_x_ = x;
  last_ref_y_ = y;
  wheel_dist_since_ref_ = 0.0;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr filterAndDownsample(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & in,
  double min_range, double max_range,
  double voxel_leaf)
{
  auto filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  filtered->reserve(in->size());
  const double min_r2 = min_range * min_range;
  const double max_r2 = max_range * max_range;
  for (const auto & p : in->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const double r2 = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y + static_cast<double>(p.z) * p.z;
    if (r2 < min_r2 || r2 > max_r2) continue;
    filtered->push_back(p);
  }

  if (voxel_leaf > 1e-6) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(static_cast<float>(voxel_leaf), static_cast<float>(voxel_leaf), static_cast<float>(voxel_leaf));
    vg.setInputCloud(filtered);
    auto ds = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    vg.filter(*ds);
    return ds;
  }
  return filtered;
}

bool GyroOdometerNode::computeImuDeltaYaw(
  const rclcpp::Time & t0, const rclcpp::Time & t1, double & out_dyaw) const
{
  out_dyaw = 0.0;
  if (t1 <= t0) return false;

  std::vector<TimedYawRate> samples;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    samples.reserve(imu_buf_.size());
    for (const auto & sample : imu_buf_) {
      samples.push_back({sample.stamp.seconds(), sample.yaw_rate_corrected});
    }
  }

  const auto result = integrateYawRateStrict(
    samples, t0.seconds(), t1.seconds(),
    imu_max_sample_gap_sec_, imu_max_boundary_gap_sec_);
  if (!result.valid) return false;

  out_dyaw = normalizeYaw(result.delta_yaw_rad);
  return std::isfinite(out_dyaw);
}

bool GyroOdometerNode::computeWheelDistance(const rclcpp::Time & t0, const rclcpp::Time & t1, double & out_dist) const
{
  out_dist = 0.0;
  if (t1 <= t0) return false;

  std::lock_guard<std::mutex> lk(mtx_);
  if (wheel_buf_.empty()) return false;

  const double t0s = t0.seconds();
  const double t1s = t1.seconds();
  const double min_t = wheel_buf_.front().stamp.seconds();
  const double max_t = wheel_buf_.back().stamp.seconds();
  if (t0s < min_t) return false;
  if (t1s > max_t + std::max(0.0, wheel_speed_timeout_sec_)) return false;

  std::vector<std::pair<double, double>> samples;
  samples.reserve(wheel_buf_.size());
  for (const auto & s : wheel_buf_) {
    samples.emplace_back(s.stamp.seconds(), s.v_raw * wheel_speed_scale_);
  }
  if (samples.empty()) return false;

  auto interp_speed = [&](double ts) -> double {
    if (ts <= samples.front().first) return samples.front().second;
    if (ts >= samples.back().first) return samples.back().second;
    auto it = std::lower_bound(
      samples.begin(), samples.end(), ts,
      [](const auto & a, double v) { return a.first < v; });
    if (it == samples.begin()) return it->second;
    auto it0 = std::prev(it);
    const double dt = it->first - it0->first;
    if (dt <= 1e-9) return it->second;
    const double r = (ts - it0->first) / dt;
    return (1.0 - r) * it0->second + r * it->second;
  };

  double last_t = t0s;
  double last_v = interp_speed(t0s);
  for (const auto & s : samples) {
    if (s.first <= t0s || s.first >= t1s) continue;
    out_dist += 0.5 * (last_v + s.second) * (s.first - last_t);
    last_t = s.first;
    last_v = s.second;
  }
  const double end_v = interp_speed(t1s);
  out_dist += 0.5 * (last_v + end_v) * (t1s - last_t);
  return std::isfinite(out_dist);
}

bool GyroOdometerNode::updateMiniSmootherLocked(const ScanFactor & factor)
{
  if (!lidar_smoother_enable_) return false;
  if (!has_odom_pose_) {
    odom_yaw_ = yaw_imu_;
    has_odom_pose_ = true;
  }

  if (!lidar_smoother_initialized_) {
    lidar_smoother_.reset(se2::Pose{odom_x_, odom_y_, odom_yaw_});
    lidar_smoother_initialized_ = true;
  }

  se2::Pose optimized;
  if (!lidar_smoother_.addFactor(factor, optimized)) {
    // The smoother rolls back internally. Pose fallback is performed by the caller so
    // a failed solve cannot apply the same relative motion twice.
    return false;
  }

  odom_x_ = optimized.x;
  odom_y_ = optimized.y;
  odom_yaw_ = normalizeYaw(optimized.yaw);
  return true;
}

void GyroOdometerNode::resetLidarTrackingLocked()
{
  prev_cloud_.reset();
  has_prev_cloud_ = false;
  last_gicp_guess_ = Eigen::Matrix4f::Identity();
  next_icp_use_full_guess_ = false;
  last_icp_guess_mode_ = "identity";
  next_icp_guess_mode_ = lidar_guess_use_imu_yaw_only_ ? "yaw_only" : "full";
  last_lidar_ = LidarOdomSample{};
  lidar_smoother_initialized_ = false;
  if (has_odom_pose_) {
    lidar_smoother_.reset(se2::Pose{odom_x_, odom_y_, odom_yaw_});
  }
  if (external_submap_snapshot_enable_) {
    ++external_submap_odom_generation_;
    if (external_submap_odom_generation_ == 0) {
      external_submap_odom_generation_ = 1;
    }
    external_submap_generation_has_snapshot_ = false;
  }
}

void GyroOdometerNode::validateParameters() const
{
  auto require_positive = [](double value, const char * name) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and > 0");
      }
    };
  auto require_non_negative = [](double value, const char * name) {
      if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and >= 0");
      }
    };

  if (base_frame_.empty() || odom_frame_.empty()) {
    throw std::invalid_argument("base_frame and odom_frame must be non-empty");
  }
  require_positive(publish_rate_hz_, "publish_rate_hz");
  require_positive(
    imu_linear_acceleration_scale_, "imu_corrected.linear_acceleration_scale");
  require_positive(imu_max_abs_yaw_rate_radps_, "imu_corrected.max_abs_yaw_rate_radps");
  require_positive(imu_max_sample_gap_sec_, "imu_corrected.max_sample_gap_sec");
  require_non_negative(imu_max_boundary_gap_sec_, "imu_corrected.max_boundary_gap_sec");
  require_non_negative(lidar_min_range_m_, "lidar_odom.min_range_m");
  require_positive(lidar_max_range_m_, "lidar_odom.max_range_m");
  if (lidar_max_range_m_ <= lidar_min_range_m_) {
    throw std::invalid_argument("lidar_odom.max_range_m must exceed min_range_m");
  }
  require_positive(lidar_timeout_sec_, "lidar_odom.timeout_sec");
  require_positive(lidar_voxel_leaf_m_, "lidar_odom.voxel_leaf_m");
  require_positive(gicp_max_corr_dist_m_, "lidar_odom.gicp.max_corr_dist_m");
  require_positive(gicp_fitness_max_, "lidar_odom.gicp.max_fitness");
  require_non_negative(gicp_trans_eps_, "lidar_odom.gicp.trans_eps");
  require_non_negative(gicp_rot_eps_, "lidar_odom.gicp.rot_eps");
  if (gicp_max_iterations_ < 1 || lidar_num_threads_ < 1 || gicp_corr_randomness_ < 1) {
    throw std::invalid_argument("LiDAR registration integer parameters must be >= 1");
  }
  if (lidar_smoother_window_size_ < 1 || lidar_smoother_max_iter_ < 1) {
    throw std::invalid_argument("LiDAR smoother window and iterations must be >= 1");
  }
  if (external_submap_snapshot_publish_interval_frames_ < 1) {
    throw std::invalid_argument(
            "lidar_odom.external_submap_snapshot.publish_interval_frames must be >= 1");
  }
  if (external_submap_snapshot_enable_ && external_submap_snapshot_topic_.empty()) {
    throw std::invalid_argument(
            "lidar_odom.external_submap_snapshot.topic must be non-empty when enabled");
  }
  if (external_submap_snapshot_enable_ &&
    (!lidar_odom_enable_ || !lidar_pose_se2_enable_))
  {
    throw std::invalid_argument(
            "external_submap_snapshot requires enabled SE2 LiDAR in scan_to_scan mode");
  }
  if (accepted_scan_odom_enable_ && accepted_scan_odom_topic_.empty()) {
    throw std::invalid_argument(
            "lidar_odom.accepted_scan_odom.topic must be non-empty when enabled");
  }
  if (accepted_scan_odom_enable_ &&
    (!lidar_odom_enable_ || !lidar_pose_se2_enable_))
  {
    throw std::invalid_argument(
            "accepted_scan_odom requires enabled SE2 LiDAR odometry");
  }
  require_positive(
    lidar_smoother_max_position_correction_m_,
    "lidar_odom.smoother.max_solution_position_correction_m");
  require_positive(
    lidar_smoother_max_yaw_correction_rad_,
    "lidar_odom.smoother.max_solution_yaw_correction_rad");
  require_positive(
    lidar_smoother_hessian_yaw_metric_m_,
    "lidar_odom.smoother.hessian_information.yaw_metric_m");
  if (!std::isfinite(lidar_smoother_hessian_min_direction_ratio_) ||
    lidar_smoother_hessian_min_direction_ratio_ < 0.0 ||
    lidar_smoother_hessian_min_direction_ratio_ > 1.0)
  {
    throw std::invalid_argument(
            "lidar_odom.smoother.hessian_information.min_direction_ratio must be in [0, 1]");
  }
  require_non_negative(
    wheel_observability_assist_min_wheel_dist_m_,
    "wheel_speed.observability_assist.min_wheel_dist_m");
  if (!std::isfinite(wheel_observability_assist_max_blend_) ||
    wheel_observability_assist_max_blend_ < 0.0 ||
    wheel_observability_assist_max_blend_ > 1.0)
  {
    throw std::invalid_argument(
            "wheel_speed.observability_assist.max_blend must be in [0, 1]");
  }
  require_positive(
    wheel_observability_assist_power_,
    "wheel_speed.observability_assist.power");
  if (!std::isfinite(odom_cov_observability_max_scale_) ||
    odom_cov_observability_max_scale_ < 1.0)
  {
    throw std::invalid_argument(
            "odom_covariance.observability_max_scale must be finite and >= 1");
  }
}

void GyroOdometerNode::onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!lidar_odom_enable_) return;
  std::lock_guard<std::mutex> callback_lock(lidar_callback_mtx_);

  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  if (stamp.nanoseconds() == 0) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[pure_gyro_odometer] Rejecting point cloud with zero timestamp.");
    return;
  }

  // Resolve the static LiDAR extrinsic (base <- scan_frame). Missing or changing
  // frames fail closed; silently assuming identity corrupts both tracking modes.
  const std::string scan_frame = msg->header.frame_id;
  if (scan_frame.empty()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[pure_gyro_odometer] Rejecting point cloud with empty frame_id.");
    return;
  }

  Eigen::Matrix4f T_base_scan = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T_scan_base = Eigen::Matrix4f::Identity();
  bool has_extrinsic = false;
  bool frame_changed = false;
  std::string cached_scan_frame;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    cached_scan_frame = scan_frame_id_;
    frame_changed = has_scan_extrinsic_ && (scan_frame_id_ != scan_frame);
    has_extrinsic = has_scan_extrinsic_ && (scan_frame_id_ == scan_frame);
    if (has_extrinsic) {
      T_base_scan = T_base_scan_;
      T_scan_base = T_scan_base_;
    }
  }
  if (frame_changed) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[pure_gyro_odometer] LiDAR frame changed from '%s' to '%s' at runtime. "
      "Rejecting cloud to avoid mixing scan frames.",
      cached_scan_frame.c_str(), scan_frame.c_str());
    return;
  }

  if (!has_extrinsic) {
    if (scan_frame == base_frame_) {
      has_extrinsic = true;
    } else {
      try {
        const auto transform =
          tf_buffer_->lookupTransform(base_frame_, scan_frame, tf2::TimePointZero);
        Eigen::Quaternionf quaternion(
          static_cast<float>(transform.transform.rotation.w),
          static_cast<float>(transform.transform.rotation.x),
          static_cast<float>(transform.transform.rotation.y),
          static_cast<float>(transform.transform.rotation.z));
        const bool translation_finite =
          std::isfinite(transform.transform.translation.x) &&
          std::isfinite(transform.transform.translation.y) &&
          std::isfinite(transform.transform.translation.z);
        if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1.0e-9F ||
          !translation_finite)
        {
          throw tf2::TransformException("non-finite LiDAR extrinsic");
        }
        quaternion.normalize();
        T_base_scan.setIdentity();
        T_base_scan.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
        T_base_scan(0, 3) = static_cast<float>(transform.transform.translation.x);
        T_base_scan(1, 3) = static_cast<float>(transform.transform.translation.y);
        T_base_scan(2, 3) = static_cast<float>(transform.transform.translation.z);
        T_scan_base = T_base_scan.inverse();
        has_extrinsic = T_base_scan.allFinite() && T_scan_base.allFinite();
        if (!has_extrinsic) {
          throw tf2::TransformException(
                  "LiDAR extrinsic inversion produced non-finite values");
        }
      } catch (const tf2::TransformException & exception) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[pure_gyro_odometer] TF lookup failed for %s <- %s: %s. "
          "Rejecting point cloud.",
          base_frame_.c_str(), scan_frame.c_str(), exception.what());
        return;
      }
    }

    std::lock_guard<std::mutex> lock(mtx_);
    has_scan_extrinsic_ = true;
    scan_frame_id_ = scan_frame;
    T_base_scan_ = T_base_scan;
    T_scan_base_ = T_scan_base;
  }

  auto cloud_in = pcl::PointCloud<pcl::PointXYZ>::Ptr(
    new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud_in);
  auto cloud = filterAndDownsample(
    cloud_in, lidar_min_range_m_, lidar_max_range_m_, lidar_voxel_leaf_m_);
  if (!cloud || cloud->size() < 50U) {
    std::lock_guard<std::mutex> lock(mtx_);
    last_lidar_ = LidarOdomSample{};
    last_lidar_.stamp = stamp;
    last_lidar_.registration_source = "insufficient_source_points";
    last_lidar_.rejection_reason = "insufficient_source_points";
    next_icp_use_full_guess_ = true;
    next_icp_guess_mode_ = "full_due_to_insufficient_source_points";
    last_registration_source_ = "insufficient_source_points";
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[pure_gyro_odometer] Rejecting filtered point cloud with %zu points; "
      "the previous accepted scan state is retained.",
      cloud ? cloud->size() : 0U);
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud;
  Eigen::Matrix4f scan_to_scan_guess = Eigen::Matrix4f::Identity();
  rclcpp::Time previous_lidar_stamp = stamp;
  std::string guess_mode_used{"full"};
  bool use_full_guess = false;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_prev_cloud_ || !prev_cloud_ || prev_cloud_->empty()) {
      prev_cloud_ = cloud;
      has_prev_cloud_ = true;
      prev_cloud_stamp_ = stamp;
      last_lidar_ = LidarOdomSample{};
      last_lidar_.stamp = stamp;
      last_lidar_.registration_source = "initialization";
      last_gicp_guess_ = Eigen::Matrix4f::Identity();
      next_icp_use_full_guess_ = false;
      last_icp_guess_mode_ = "identity";
      next_icp_guess_mode_ = lidar_guess_use_imu_yaw_only_ ? "yaw_only" : "full";
      if (!has_odom_pose_) {
        odom_yaw_ = yaw_imu_;
        has_odom_pose_ = true;
      }
      last_registration_source_ = "initialization";
      return;
    }

    if (stamp <= prev_cloud_stamp_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[pure_gyro_odometer] Rejecting out-of-order point cloud "
        "(current=%.9f, previous=%.9f).",
        stamp.seconds(), prev_cloud_stamp_.seconds());
      return;
    }

    const double scan_gap = (stamp - prev_cloud_stamp_).seconds();
    if (!std::isfinite(scan_gap) || scan_gap > lidar_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[pure_gyro_odometer] LiDAR gap %.6f s exceeds timeout %.6f s; "
        "reinitializing scan-to-scan tracking.",
        scan_gap, lidar_timeout_sec_);
      resetLidarTrackingLocked();
      prev_cloud_ = cloud;
      prev_cloud_stamp_ = stamp;
      has_prev_cloud_ = true;
      last_lidar_.stamp = stamp;
      last_lidar_.registration_source = "reinitialized_after_gap";
      if (!has_odom_pose_) {
        odom_yaw_ = yaw_imu_;
        has_odom_pose_ = true;
      }
      last_registration_source_ = "reinitialized_after_gap";
      return;
    }

    previous_cloud = prev_cloud_;
    scan_to_scan_guess = last_gicp_guess_;
    previous_lidar_stamp = prev_cloud_stamp_;
    use_full_guess = next_icp_use_full_guess_;
  }

  double imu_delta_yaw_measurement = 0.0;
  const bool has_imu_delta_yaw = computeImuDeltaYaw(
    previous_lidar_stamp, stamp, imu_delta_yaw_measurement);

  double wheel_distance_guess = 0.0;
  const bool has_wheel_distance = use_wheel_speed_ && computeWheelDistance(
    previous_lidar_stamp, stamp, wheel_distance_guess);
  const bool has_current_wheel_imu_prior = has_imu_delta_yaw && has_wheel_distance;
  const Eigen::Vector3d wheel_imu_prior = has_current_wheel_imu_prior ?
    arcMotionPrior(wheel_distance_guess, imu_delta_yaw_measurement) :
    Eigen::Vector3d::Zero();

  if (lidar_guess_use_imu_yaw_only_ && has_imu_delta_yaw && !use_full_guess) {
    scan_to_scan_guess = Eigen::Matrix4f::Identity();
    scan_to_scan_guess.block<3, 3>(0, 0) = Eigen::AngleAxisf(
      static_cast<float>(imu_delta_yaw_measurement),
      Eigen::Vector3f::UnitZ()).toRotationMatrix();
    guess_mode_used = "yaw_only";
  } else if (use_full_guess && wheel_registration_recovery_use_current_prior_ &&
    has_current_wheel_imu_prior)
  {
    scan_to_scan_guess = se2DeltaToScanGuess(
      wheel_imu_prior, T_base_scan, T_scan_base);
    guess_mode_used = "full_wheel_imu_prior_due_to_previous_registration_reject";
  } else if (use_full_guess) {
    guess_mode_used = "full_due_to_previous_registration_reject";
  } else if (lidar_guess_use_imu_yaw_only_ && !has_imu_delta_yaw) {
    guess_mode_used = "full_no_imu_delta";
  }

  if (!previous_cloud || previous_cloud->size() < 50U) {
    std::lock_guard<std::mutex> lock(mtx_);
    resetLidarTrackingLocked();
    prev_cloud_ = cloud;
    has_prev_cloud_ = true;
    prev_cloud_stamp_ = stamp;
    last_lidar_.stamp = stamp;
    last_lidar_.registration_source = "reinitialized_invalid_previous_scan";
    last_registration_source_ = "reinitialized_invalid_previous_scan";
    return;
  }

  double dt = (stamp - previous_lidar_stamp).seconds();
  if (!std::isfinite(dt) || dt <= 1.0e-6) dt = 0.0;

  // Internal LiDAR odometry is scan-to-scan only.
  pcl::PointCloud<pcl::PointXYZ> scan_to_scan_aligned;
  bool scan_to_scan_converged = false;
  double scan_to_scan_fitness = std::numeric_limits<double>::infinity();
  double scan_to_scan_inlier_ratio = 0.0;
  Eigen::Matrix4f T_prev_curr_scan_scan = scan_to_scan_guess;
  Matrix6d scan_to_scan_hessian = Matrix6d::Zero();
  bool scan_to_scan_has_hessian = false;
  try {
    small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> registration;
    registration.setNumThreads(lidar_num_threads_);
    registration.setRegistrationType(lidar_registration_type_);
    registration.setMaxCorrespondenceDistance(gicp_max_corr_dist_m_);
    registration.setMaximumIterations(gicp_max_iterations_);
    registration.setTransformationEpsilon(gicp_trans_eps_);
    registration.setRotationEpsilon(gicp_rot_eps_);
    registration.setCorrespondenceRandomness(gicp_corr_randomness_);
    registration.setVoxelResolution(gicp_voxel_resolution_);
    registration.setInputTarget(previous_cloud);
    registration.setInputSource(cloud);
    registration.align(scan_to_scan_aligned, scan_to_scan_guess);

    const auto & result = registration.getRegistrationResult();
    scan_to_scan_converged = registration.hasConverged() && result.converged;
    scan_to_scan_fitness = registration.getFitnessScore(gicp_max_corr_dist_m_);
    scan_to_scan_inlier_ratio = cloud->empty() ? 0.0 : clamp01(
      static_cast<double>(result.num_inliers) / static_cast<double>(cloud->size()));
    T_prev_curr_scan_scan = result.T_target_source.matrix().cast<float>();
    scan_to_scan_hessian = result.H;
    scan_to_scan_has_hessian = result.H.allFinite();
  } catch (const std::exception & exception) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[pure_gyro_odometer] scan-to-scan registration failed: %s",
      exception.what());
    scan_to_scan_converged = false;
    scan_to_scan_fitness = std::numeric_limits<double>::infinity();
    scan_to_scan_inlier_ratio = 0.0;
    T_prev_curr_scan_scan = Eigen::Matrix4f::Identity();
    scan_to_scan_has_hessian = false;
  }

  const bool converged = scan_to_scan_converged;
  const double fitness = scan_to_scan_fitness;
  const double inlier_ratio = scan_to_scan_inlier_ratio;
  const Eigen::Matrix4f T_prev_curr_scan = T_prev_curr_scan_scan;
  const Matrix6d hessian_scan = scan_to_scan_hessian;
  const bool has_hessian = scan_to_scan_has_hessian;

  const Eigen::Matrix4f T_prev_curr_base =
    T_base_scan * T_prev_curr_scan * T_scan_base;
  const Eigen::Matrix3d rotation =
    T_prev_curr_base.block<3, 3>(0, 0).cast<double>();
  const Eigen::Vector3d translation =
    T_prev_curr_base.block<3, 1>(0, 3).cast<double>();
  const double raw_dx = translation.x();
  const double raw_dy = translation.y();
  const double dz = translation.z();
  const double raw_dyaw = yawFromRot(rotation);

  bool stationary_now = false;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    updateStopState(stamp);
    stationary_now = is_stopped_;
  }

  ObservabilityInfo observability;
  observability.enabled = lidar_smoother_hessian_enable_;
  observability.has_hessian = has_hessian;
  observability.stationary_now = stationary_now;

  Eigen::Vector3d delta_scan(raw_dx, raw_dy, raw_dyaw);
  Eigen::Vector3d delta_used = delta_scan;
  Eigen::Vector3d delta_prior = Eigen::Vector3d::Zero();
  Matrix3d scan_information_se2 = Matrix3d::Zero();
  bool has_scan_information_se2 = false;

  if (has_hessian) {
    const Matrix6d adjoint = adjointRotationFirst(T_scan_base.cast<double>());
    const Matrix6d transformed = adjoint.transpose() * hessian_scan * adjoint;
    const Matrix6d hessian_base = 0.5 * (transformed + transformed.transpose());
    has_scan_information_se2 = reduceHessianToSe2Info(
      hessian_base, scan_information_se2);
  }

  const DirectionalInformationAnalysis directional_information =
    lidar_smoother_hessian_enable_ && has_scan_information_se2 ?
    analyzeDirectionalInformation(
      scan_information_se2, lidar_smoother_hessian_yaw_metric_m_,
      lidar_smoother_hessian_min_direction_ratio_,
      wheel_observability_assist_power_) :
    DirectionalInformationAnalysis{};

  const bool has_smoother_scan_information = directional_information.valid;
  std::array<double, 9> smoother_scan_information{{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0}};
  if (has_smoother_scan_information) {
    smoother_scan_information = matrixToArray(
      directional_information.normalized_information);
    observability.has_directional_information = true;
    observability.information_ratio_min =
      directional_information.information_ratios(0);
    observability.information_ratio_mid =
      directional_information.information_ratios(1);
    observability.information_ratio_max =
      directional_information.information_ratios(2);
    observability.information_deficit =
      clamp01(directional_information.information_deficit);
    const Eigen::Vector3d projected_motion =
      directional_information.weakness_projector * delta_scan;
    observability.stationary_projected_motion_metric = se2MetricNorm(
      projected_motion, lidar_smoother_hessian_yaw_metric_m_);
  }

  const bool wheel_prior_available =
    use_wheel_speed_ && has_imu_delta_yaw && has_wheel_distance;
  if (wheel_prior_available) {
    observability.wheel_prior_available = true;
    observability.wheel_distance = wheel_distance_guess;
    delta_prior = arcMotionPrior(wheel_distance_guess, imu_delta_yaw_measurement);
    observability.prior_dx = delta_prior.x();
    observability.prior_dy = delta_prior.y();
    observability.prior_dyaw = normalizeYaw(delta_prior.z());

    const Eigen::Vector3d raw_prior_difference = delta_scan - delta_prior;
    Eigen::Vector3d prior_difference = raw_prior_difference;
    if (has_smoother_scan_information) {
      prior_difference =
        directional_information.weakness_projector * raw_prior_difference;
    }
    observability.prior_difference_translation =
      prior_difference.head<2>().norm();
    observability.prior_difference_yaw = std::fabs(prior_difference.z());
    observability.prior_difference_metric = se2MetricNorm(
      prior_difference, lidar_smoother_hessian_yaw_metric_m_);

    if (dt > 1.0e-6) {
      observability.scan_speed_mps = std::hypot(raw_dx, raw_dy) / dt;
      observability.wheel_speed_mps = std::fabs(wheel_distance_guess) / dt;
      observability.speed_difference_mps = std::fabs(
        observability.scan_speed_mps - observability.wheel_speed_mps);
    }
  }

  std::string rejection_reason{"accepted"};
  if (!converged) {
    rejection_reason = "not_converged";
  } else if (!std::isfinite(fitness)) {
    rejection_reason = "nonfinite_fitness";
  } else if (fitness > gicp_fitness_max_) {
    rejection_reason = "fitness_gate";
  } else if (dt <= 0.0) {
    rejection_reason = "invalid_dt";
  } else if (!T_prev_curr_base.allFinite()) {
    rejection_reason = "nonfinite_transform";
  }
  const bool registration_rejected = rejection_reason != "accepted";

  const bool wheel_assist_available =
    wheel_observability_assist_enable_ && wheel_prior_available &&
    has_smoother_scan_information && !stationary_now && !registration_rejected &&
    std::fabs(wheel_distance_guess) >=
    wheel_observability_assist_min_wheel_dist_m_;
  if (wheel_assist_available) {
    const double assist_blend =
      clamp01(wheel_observability_assist_max_blend_);
    const Matrix3d correction_projector =
      assist_blend * directional_information.weakness_projector;
    const Eigen::Vector3d assisted_delta = fuseWeakDirections(
      delta_scan, delta_prior, correction_projector);
    if (assisted_delta.allFinite()) {
      const Eigen::Vector3d correction = assisted_delta - delta_scan;
      delta_used = assisted_delta;
      delta_used.z() = normalizeYaw(delta_used.z());
      observability.wheel_assist_correction_metric = se2MetricNorm(
        correction, lidar_smoother_hessian_yaw_metric_m_);
      observability.wheel_assisted =
        observability.wheel_assist_correction_metric > 1.0e-9;
      observability.wheel_assist_blend = observability.wheel_assisted ?
        assist_blend : 0.0;
    }
  }
  if (!delta_used.allFinite()) {
    delta_used = delta_scan;
    observability.wheel_assisted = false;
    observability.wheel_assist_blend = 0.0;
    observability.wheel_assist_correction_metric = 0.0;
  }
  delta_used.z() = normalizeYaw(delta_used.z());

  Eigen::Matrix4f next_scan_to_scan_guess = T_prev_curr_scan;
  if (observability.wheel_assisted) {
    Eigen::Matrix4f T_prev_curr_base_assisted = T_prev_curr_base;
    T_prev_curr_base_assisted(0, 3) = static_cast<float>(delta_used.x());
    T_prev_curr_base_assisted(1, 3) = static_cast<float>(delta_used.y());

    double raw_roll = 0.0;
    double raw_pitch = 0.0;
    double raw_yaw_unused = 0.0;
    rotationToRollPitchYaw(
      rotation.cast<float>(), raw_roll, raw_pitch, raw_yaw_unused);
    (void)raw_yaw_unused;
    Eigen::Quaternionf assisted_rotation(
      Eigen::AngleAxisf(static_cast<float>(delta_used.z()), Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(static_cast<float>(raw_pitch), Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(static_cast<float>(raw_roll), Eigen::Vector3f::UnitX()));
    assisted_rotation.normalize();
    T_prev_curr_base_assisted.block<3, 3>(0, 0) =
      assisted_rotation.toRotationMatrix();
    next_scan_to_scan_guess =
      T_scan_base * T_prev_curr_base_assisted * T_base_scan;
    if (!next_scan_to_scan_guess.allFinite()) {
      next_scan_to_scan_guess = T_prev_curr_scan;
    }
  }

  LidarOdomSample output;
  output.stamp = stamp;
  output.dt = dt;
  output.valid = !registration_rejected && delta_used.allFinite();
  output.converged = converged;
  output.fitness = fitness;
  output.inlier_ratio = inlier_ratio;
  output.registration_source = "scan_to_scan";
  output.rejection_reason = rejection_reason;
  output.raw_dx = raw_dx;
  output.raw_dy = raw_dy;
  output.raw_dyaw = raw_dyaw;
  output.dx = delta_used.x();
  output.dy = delta_used.y();
  output.dyaw = delta_used.z();
  if (dt > 1.0e-6) {
    output.raw_vx = output.raw_dx / dt;
    output.raw_vy = output.raw_dy / dt;
    output.raw_yaw_rate = output.raw_dyaw / dt;
    output.vx = output.dx / dt;
    output.vy = output.dy / dt;
    output.yaw_rate = output.dyaw / dt;
  }
  output.observability = observability;
  if (output.valid && dt > 1.0e-6) {
    output.v = std::sqrt(
      output.dx * output.dx + output.dy * output.dy + dz * dz) / dt;
  }

  double fused_delta_yaw = has_imu_delta_yaw ?
    imu_delta_yaw_measurement : output.dyaw;
  if (has_imu_delta_yaw && std::fabs(lidar_yaw_blend_imu_) > 1.0e-6) {
    const double blend = clamp01(lidar_yaw_blend_imu_);
    fused_delta_yaw = normalizeYaw(
      (1.0 - blend) * imu_delta_yaw_measurement + blend * output.dyaw);
  }

  const bool use_full_guess_next = !output.valid;
  std::string next_guess_mode{"full"};
  if (lidar_guess_use_imu_yaw_only_) {
    next_guess_mode = output.valid ?
      "yaw_only" : "full_due_to_current_registration_reject";
  }

  bool publish_external_submap_snapshot = false;
  pure_lidar_msgs::msg::SubmapScan external_submap_snapshot;
  bool publish_accepted_scan_odom = false;
  nav_msgs::msg::Odometry accepted_scan_odom;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_odom_pose_) {
      odom_yaw_ = yaw_imu_;
      has_odom_pose_ = true;
    }
    updateStopState(stamp);

    bool pose_updated = false;
    if (output.valid && lidar_pose_se2_enable_) {
      if (lidar_smoother_enable_) {
        ScanFactor factor;
        factor.dx = output.dx;
        factor.dy = output.dy;
        factor.dyaw_scan = output.dyaw;
        factor.dyaw_imu = has_imu_delta_yaw ?
          imu_delta_yaw_measurement : output.dyaw;
        factor.has_imu_yaw = has_imu_delta_yaw;
        factor.fitness = output.fitness;
        factor.converged = output.converged;
        factor.stationary = is_stopped_;
        factor.wheel_assisted = output.observability.wheel_assisted;
        factor.has_scan_information = has_smoother_scan_information;
        factor.scan_information = smoother_scan_information;

        pose_updated = updateMiniSmootherLocked(factor);
        if (!pose_updated) {
          const double yaw_mid = odom_yaw_ + 0.5 * fused_delta_yaw;
          odom_x_ +=
            std::cos(yaw_mid) * output.dx -
            std::sin(yaw_mid) * output.dy;
          odom_y_ +=
            std::sin(yaw_mid) * output.dx +
            std::cos(yaw_mid) * output.dy;
          odom_yaw_ = normalizeYaw(odom_yaw_ + fused_delta_yaw);
          lidar_smoother_.reset(se2::Pose{odom_x_, odom_y_, odom_yaw_});
          lidar_smoother_initialized_ = true;
          pose_updated = true;
        }
      } else {
        const double yaw_mid = odom_yaw_ + 0.5 * fused_delta_yaw;
        odom_x_ +=
          std::cos(yaw_mid) * output.dx -
          std::sin(yaw_mid) * output.dy;
        odom_y_ +=
          std::sin(yaw_mid) * output.dx +
          std::cos(yaw_mid) * output.dy;
        odom_yaw_ = normalizeYaw(odom_yaw_ + fused_delta_yaw);
        pose_updated = true;
      }
    }

    const bool accepted_pose = output.valid && pose_updated;
    if (accepted_pose) {
      ++lidar_pose_sequence_;
    }

    last_registration_source_ = output.registration_source;

    const double motion_distance = std::hypot(output.dx, output.dy);
    if (output.valid) {
      double step_xy = odom_cov_base_xy_step_;
      double step_yaw = odom_cov_base_yaw_step_;
      step_xy += odom_cov_xy_per_meter_ * motion_distance;
      step_yaw += odom_cov_yaw_per_rad_ * std::fabs(output.dyaw);
      if (std::isfinite(output.fitness) && output.fitness > 0.0) {
        step_xy += odom_cov_fitness_xy_scale_ * output.fitness;
        step_yaw += odom_cov_fitness_yaw_scale_ * output.fitness;
      }
      if (output.observability.has_directional_information) {
        const double observability_scale = observability::covarianceScale(
          output.observability.information_deficit,
          odom_cov_observability_max_scale_);
        step_xy *= observability_scale;
        step_yaw *= observability_scale;
      }
      if (output.observability.wheel_assisted) {
        step_xy *= std::max(1.0, odom_cov_wheel_assist_scale_);
        step_yaw *= std::max(1.0, odom_cov_wheel_assist_scale_);
      }
      if (is_stopped_ && lidar_smoother_zupt_enable_) {
        step_xy *= 0.25;
        step_yaw *= 0.25;
      }
      odom_cov_total_xy_ += std::max(0.0, step_xy);
      odom_cov_total_yaw_ += std::max(0.0, step_yaw);
    } else {
      odom_cov_total_xy_ += std::max(0.0, odom_cov_invalid_xy_step_);
      odom_cov_total_yaw_ += std::max(0.0, odom_cov_invalid_yaw_step_);
    }

    if (accepted_pose && pub_accepted_scan_odom_) {
      publish_accepted_scan_odom = true;
      const bool has_twist = std::isfinite(output.dt) && output.dt > 1.0e-6;
      AcceptedScanOdometryState state;
      state.x = odom_x_;
      state.y = odom_y_;
      state.yaw = odom_yaw_;
      state.vx = output.vx;
      state.vy = output.vy;
      state.yaw_rate = has_twist ? fused_delta_yaw / output.dt : 0.0;
      state.covariance_xy = odom_cov_total_xy_;
      state.covariance_yaw = odom_cov_total_yaw_;
      state.has_twist = has_twist;
      accepted_scan_odom = makeAcceptedScanOdometry(
        msg->header.stamp, odom_frame_, base_frame_, state);
      ++accepted_scan_odom_published_count_;
    }

    if (accepted_pose && pub_external_submap_snapshot_ &&
      snapshot_policy::due(
        external_submap_generation_has_snapshot_, lidar_pose_sequence_,
        static_cast<std::uint64_t>(external_submap_snapshot_publish_interval_frames_)))
    {
      publish_external_submap_snapshot = true;
      external_submap_generation_has_snapshot_ = true;
      external_submap_snapshot.header = msg->header;
      external_submap_snapshot.header.frame_id = odom_frame_;
      external_submap_snapshot.odom_session_id = external_submap_odom_session_id_;
      external_submap_snapshot.odom_generation = external_submap_odom_generation_;
      external_submap_snapshot.sequence = lidar_pose_sequence_;
      external_submap_snapshot.raw_pose.pose.position.x = odom_x_;
      external_submap_snapshot.raw_pose.pose.position.y = odom_y_;
      external_submap_snapshot.raw_pose.pose.position.z = 0.0;
      const auto quaternion = quatFromYaw(odom_yaw_);
      external_submap_snapshot.raw_pose.pose.orientation.x = quaternion.x();
      external_submap_snapshot.raw_pose.pose.orientation.y = quaternion.y();
      external_submap_snapshot.raw_pose.pose.orientation.z = quaternion.z();
      external_submap_snapshot.raw_pose.pose.orientation.w = quaternion.w();
      external_submap_snapshot.raw_pose.covariance.fill(0.0);
      external_submap_snapshot.raw_pose.covariance[0] =
        std::max(0.0, odom_cov_total_xy_);
      external_submap_snapshot.raw_pose.covariance[7] =
        std::max(0.0, odom_cov_total_xy_);
      external_submap_snapshot.raw_pose.covariance[14] = 1.0e6;
      external_submap_snapshot.raw_pose.covariance[21] = 1.0e6;
      external_submap_snapshot.raw_pose.covariance[28] = 1.0e6;
      external_submap_snapshot.raw_pose.covariance[35] =
        std::max(0.0, odom_cov_total_yaw_);
    }

    last_lidar_ = output;
    last_icp_guess_mode_ = guess_mode_used;
    next_icp_use_full_guess_ = use_full_guess_next;
    next_icp_guess_mode_ = next_guess_mode;

    if (output.valid) {
      // A rejected scan is never promoted, so one bad match cannot cascade.
      prev_cloud_ = cloud;
      has_prev_cloud_ = true;
      prev_cloud_stamp_ = stamp;
      last_gicp_guess_ = next_scan_to_scan_guess;
    }
  }

  if (publish_accepted_scan_odom && pub_accepted_scan_odom_) {
    pub_accepted_scan_odom_->publish(accepted_scan_odom);
  }

  if (publish_external_submap_snapshot && pub_external_submap_snapshot_) {
    const auto conversion_start = std::chrono::steady_clock::now();
    try {
      pcl::toROSMsg(*cloud, external_submap_snapshot.cloud);
      const double conversion_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - conversion_start).count();
      external_submap_snapshot.cloud.header = msg->header;
      pub_external_submap_snapshot_->publish(external_submap_snapshot);
      std::lock_guard<std::mutex> lock(mtx_);
      ++external_submap_snapshot_published_count_;
      external_submap_snapshot_conversion_last_ms_ = conversion_ms;
      external_submap_snapshot_conversion_sum_ms_ += conversion_ms;
      external_submap_snapshot_conversion_max_ms_ = std::max(
        external_submap_snapshot_conversion_max_ms_, conversion_ms);
    } catch (const std::exception & exception) {
      std::lock_guard<std::mutex> lock(mtx_);
      external_submap_generation_has_snapshot_ = false;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[pure_gyro_odometer] Failed to serialize accepted-scan snapshot: %s",
        exception.what());
    }
  }

  publishObservabilityDebug(stamp, output, guess_mode_used, next_guess_mode);
}

bool GyroOdometerNode::computeAccVariance(const rclcpp::Time & nowt, double window_sec, double & out_var) const
{
  out_var = 1e9;
  if (imu_buf_.size() < 10) return false;

  const double t_now = nowt.seconds();

  std::vector<double> ax;
  std::vector<double> ay;
  std::vector<double> az;
  std::vector<double> gz;
  ax.reserve(64);
  ay.reserve(64);
  az.reserve(64);
  gz.reserve(64);

  for (int i = static_cast<int>(imu_buf_.size()) - 1; i >= 0; --i) {
    const double ti = imu_buf_[i].stamp.seconds();
    if ((t_now - ti) > window_sec) break;
    ax.push_back(imu_buf_[i].acc.x());
    ay.push_back(imu_buf_[i].acc.y());
    az.push_back(imu_buf_[i].acc.z());
    gz.push_back(imu_buf_[i].gyro_z);
  }
  if (gz.size() < 10) return false;

  auto variance = [](const std::vector<double> & v) -> double {
    if (v.size() < 2) return 1e9;
    double m = 0.0;
    for (double x : v) m += x;
    m /= static_cast<double>(v.size());
    double s = 0.0;
    for (double x : v) s += (x - m) * (x - m);
    return s / std::max(1.0, static_cast<double>(v.size() - 1));
  };

  out_var = variance(ax) + variance(ay) + variance(az);
  return true;
}

void GyroOdometerNode::updateStopState(const rclcpp::Time & nowt)
{
  if (!stop_enable_) {
    is_stopped_ = false;
    has_stop_candidate_since_ = false;
    return;
  }

  // Prefer speed if available (wheel or lidar)
  bool has_v = false;
  double v = 0.0;

  if (use_wheel_speed_ && has_wheel_) {
    const double age = std::fabs((nowt - last_wheel_.stamp).seconds());
    if (std::isfinite(age) && age <= wheel_speed_timeout_sec_) {
      v = last_wheel_.v_raw * wheel_speed_scale_;
      has_v = true;
    }
  }
  if (!has_v && lidar_odom_enable_ && last_lidar_.valid) {
    const double age = std::fabs((nowt - last_lidar_.stamp).seconds());
    if (std::isfinite(age) && age <= lidar_timeout_sec_) {
      v = last_lidar_.v;
      has_v = true;
    }
  }

  bool stop_candidate = false;
  if (has_v) {
    stop_candidate = (std::fabs(v) < stop_speed_thr_mps_);
  } else {
    // fallback: IMU-based stop detection
    double var_a = 1e9;
    const bool ok = computeAccVariance(nowt, 0.3, var_a);
    if (!ok) {
      stop_candidate = false;
    } else {
      // gyro mean over same window
      double gz_mean = 0.0;
      int n = 0;
      const double t_now = nowt.seconds();
      for (int i = static_cast<int>(imu_buf_.size()) - 1; i >= 0; --i) {
        const double ti = imu_buf_[i].stamp.seconds();
        if ((t_now - ti) > 0.3) break;
        gz_mean += imu_buf_[i].gyro_z;
        ++n;
      }
      if (n < 10) {
        stop_candidate = false;
      } else {
        gz_mean /= static_cast<double>(n);
        stop_candidate = (var_a < stop_acc_var_thr_) && (std::fabs(gz_mean) < stop_gyro_abs_thr_rad_s_);
      }
    }
  }

  // Hold time hysteresis
  if (stop_candidate) {
    if (!has_stop_candidate_since_) {
      stop_candidate_since_ = nowt;
      has_stop_candidate_since_ = true;
    }
    const double held = (nowt - stop_candidate_since_).seconds();
    is_stopped_ = (std::isfinite(held) && held >= stop_hold_sec_);
  } else {
    has_stop_candidate_since_ = false;
    is_stopped_ = false;
  }

  // When stopped, reset accel-based speed estimate.
  if (is_stopped_) {
    v_acc_est_ = 0.0;
    has_v_acc_ = true;
  }
}

void GyroOdometerNode::onPublishTimer()
{
  const rclcpp::Time nowt = now();

  nav_msgs::msg::Odometry odom_raw;
  nav_msgs::msg::Odometry odom_filtered;
  std_msgs::msg::Bool stopped;

  double yaw = 0.0;
  double bg = 0.0;
  bool has_yaw_rate_out = false;
  double yaw_rate_out = 0.0;
  bool imu_extrinsic_cached = false;
  std::string imu_frame_id;
  bool stopped_now = false;
  bool has_v_pred = false;
  double v_pred = 0.0;
  double v_raw = 0.0;
  double v_wheel = 0.0;
  double v_lidar = 0.0;
  bool has_wheel = false;
  bool has_lidar = false;
  double wheel_scale = 1.0;
  double odom_x = 0.0;
  double odom_y = 0.0;
  double odom_yaw = 0.0;
  double filtered_x = 0.0;
  double filtered_y = 0.0;
  double filtered_yaw = 0.0;
  bool have_filtered_pose = false;
  double raw_twist_x = 0.0;
  double raw_twist_y = 0.0;
  double raw_twist_yaw = 0.0;
  bool has_raw_twist = false;
  double filtered_twist_x = 0.0;
  double filtered_twist_y = 0.0;
  double filtered_twist_yaw = 0.0;
  bool has_filtered_twist = false;
  double odom_cov_xy_total = 0.0;
  double odom_cov_yaw_total = 0.0;
  LidarOdomSample lidar = {};
  std::string last_registration_source;
  std::string speed_source{"none"};
  std::string last_icp_guess_mode;
  std::string next_icp_guess_mode;
  std::uint64_t external_snapshot_session = 0;
  std::uint64_t external_snapshot_generation = 0;
  std::uint64_t external_snapshot_sequence = 0;
  std::uint64_t external_snapshot_published_count = 0;
  std::uint64_t accepted_scan_odom_published_count = 0;
  double external_snapshot_conversion_last_ms = 0.0;
  double external_snapshot_conversion_sum_ms = 0.0;
  double external_snapshot_conversion_max_ms = 0.0;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    updateStopState(nowt);
    stopped_now = is_stopped_;
    yaw = yaw_imu_;
    bg = bg_est_;
    imu_extrinsic_cached = has_imu_extrinsic_;
    imu_frame_id = imu_frame_id_;

    if (!imu_buf_.empty()) {
      yaw_rate_out = imu_buf_.back().gyro_z - bg_est_;
      has_yaw_rate_out = true;
    }
    wheel_scale = wheel_speed_scale_;
    odom_x = odom_x_;
    odom_y = odom_y_;
    odom_yaw = has_odom_pose_ ? odom_yaw_ : yaw_imu_;
    odom_cov_xy_total = odom_cov_total_xy_;
    odom_cov_yaw_total = odom_cov_total_yaw_;

    has_wheel = (use_wheel_speed_ && has_wheel_);
    if (has_wheel) {
      const double age = std::fabs((nowt - last_wheel_.stamp).seconds());
      if (std::isfinite(age) && age <= wheel_speed_timeout_sec_) {
        v_raw = last_wheel_.v_raw;
      } else {
        has_wheel = false;
      }
    }

    lidar = last_lidar_;
    last_registration_source = last_registration_source_;
    last_icp_guess_mode = last_icp_guess_mode_;
    next_icp_guess_mode = next_icp_guess_mode_;
    external_snapshot_session = external_submap_odom_session_id_;
    external_snapshot_generation = external_submap_odom_generation_;
    external_snapshot_sequence = lidar_pose_sequence_;
    external_snapshot_published_count = external_submap_snapshot_published_count_;
    accepted_scan_odom_published_count = accepted_scan_odom_published_count_;
    external_snapshot_conversion_last_ms = external_submap_snapshot_conversion_last_ms_;
    external_snapshot_conversion_sum_ms = external_submap_snapshot_conversion_sum_ms_;
    external_snapshot_conversion_max_ms = external_submap_snapshot_conversion_max_ms_;
    has_lidar = (lidar_odom_enable_ && lidar.valid);
    if (has_lidar) {
      const double age = std::fabs((nowt - lidar.stamp).seconds());
      if (!(std::isfinite(age) && age <= lidar_timeout_sec_)) {
        has_lidar = false;
      }
    }
    if (has_lidar) {
      v_lidar = lidar.v;
    }

    bool has_wheel_speed = false;
    if (has_wheel) {
      double v_scaled = v_raw * wheel_speed_scale_;
      if (wheel_low_speed_enable_) {
        if (std::fabs(v_raw) < wheel_low_speed_deadband_mps_ && !stopped_now && has_v_acc_) {
          double ax_abs = 0.0;
          if (!imu_buf_.empty()) {
            ax_abs = std::fabs(imu_buf_.back().acc.x());
          }
          if (ax_abs > wheel_low_speed_acc_thr_mps2_) {
            const double corr = std::max(-wheel_low_speed_max_corr_mps_,
              std::min(wheel_low_speed_max_corr_mps_, (v_acc_est_ - v_scaled)));
            v_scaled = v_scaled + wheel_low_speed_blend_ * corr;
          }
        }
      }
      v_wheel = v_scaled;
      has_wheel_speed = true;
    }

    if (has_lidar) {
      v_pred = v_lidar;
      has_v_pred = true;
      speed_source = "lidar";
    } else if (has_wheel_speed) {
      v_pred = v_wheel;
      has_v_pred = true;
      speed_source = "wheel";
    } else {
      v_pred = 0.0;
      has_v_pred = false;
      speed_source = "none";
    }

    // Publish the last accepted scan/factor-graph pose. The timer must not
    // integrate wheel speed or IMU again: doing so makes the state depend on
    // timer jitter and silently dead-reckons through rejected LiDAR scans.
    odom_x = odom_x_;
    odom_y = odom_y_;
    odom_yaw = has_odom_pose_ ? odom_yaw_ : yaw_imu_;

    if (has_lidar && std::isfinite(lidar.dt) && lidar.dt > 1e-6) {
      raw_twist_x = lidar.vx;
      raw_twist_y = lidar.vy;
      raw_twist_yaw = has_yaw_rate_out ? yaw_rate_out : lidar.yaw_rate;
      has_raw_twist = true;
    } else if (has_v_pred || has_yaw_rate_out) {
      raw_twist_x = has_v_pred ? v_pred : 0.0;
      raw_twist_y = 0.0;
      raw_twist_yaw = has_yaw_rate_out ? yaw_rate_out : 0.0;
      has_raw_twist = has_v_pred || has_yaw_rate_out;
    } else {
      raw_twist_x = 0.0;
      raw_twist_y = 0.0;
      raw_twist_yaw = 0.0;
      has_raw_twist = false;
    }

    if (out_filtered_odom_enable_ && !out_filtered_odom_topic_.empty() && out_filtered_odom_topic_ != out_odom_topic_) {
      const double dt_filtered = has_filtered_publish_state_ ? (nowt - last_filtered_publish_stamp_).seconds() : 0.0;
      const bool reset_filtered =
        !has_filtered_publish_state_ ||
        !std::isfinite(dt_filtered) ||
        dt_filtered <= 1e-4 ||
        dt_filtered > std::max(0.1, filtered_odom_reset_gap_sec_);

      if (reset_filtered) {
        filtered_pub_x_ = odom_x;
        filtered_pub_y_ = odom_y;
        filtered_pub_yaw_ = odom_yaw;
        filtered_pub_dx_ = 0.0;
        filtered_pub_dy_ = 0.0;
        filtered_pub_dyaw_ = 0.0;
        if (filtered_odom_zero_when_stopped_ && stopped_now) {
          filtered_pub_vx_ = 0.0;
          filtered_pub_vy_ = 0.0;
          filtered_pub_yaw_rate_ = 0.0;
        } else {
          filtered_pub_vx_ = has_raw_twist ? raw_twist_x : 0.0;
          filtered_pub_vy_ = has_raw_twist ? raw_twist_y : 0.0;
          filtered_pub_yaw_rate_ = has_raw_twist ? raw_twist_yaw : (has_yaw_rate_out ? yaw_rate_out : 0.0);
        }
        has_filtered_publish_state_ = true;
      } else if (filtered_odom_zero_when_stopped_ && stopped_now) {
        filtered_pub_dx_ = 0.0;
        filtered_pub_dy_ = 0.0;
        filtered_pub_dyaw_ = 0.0;
        filtered_pub_vx_ = 0.0;
        filtered_pub_vy_ = 0.0;
        filtered_pub_yaw_rate_ = 0.0;
      } else {
        const double target_vx = has_raw_twist ? raw_twist_x : 0.0;
        const double target_vy = has_raw_twist ? raw_twist_y : 0.0;
        const double target_w = has_raw_twist ? raw_twist_yaw : (has_yaw_rate_out ? yaw_rate_out : 0.0);

        const double alpha = clamp01(filtered_odom_lowpass_alpha_);
        double cmd_vx = alpha * filtered_pub_vx_ + (1.0 - alpha) * target_vx;
        double cmd_vy = alpha * filtered_pub_vy_ + (1.0 - alpha) * target_vy;
        double cmd_w = alpha * filtered_pub_yaw_rate_ + (1.0 - alpha) * target_w;

        const double vx_step = std::max(0.0, filtered_odom_linear_rate_limit_mps2_) * dt_filtered;
        const double vy_step = std::max(0.0, filtered_odom_lateral_rate_limit_mps2_) * dt_filtered;
        const double w_step = std::max(0.0, filtered_odom_yaw_rate_limit_radps2_) * dt_filtered;

        cmd_vx = filtered_pub_vx_ + std::max(-vx_step, std::min(vx_step, cmd_vx - filtered_pub_vx_));
        cmd_vy = filtered_pub_vy_ + std::max(-vy_step, std::min(vy_step, cmd_vy - filtered_pub_vy_));
        cmd_w = filtered_pub_yaw_rate_ + std::max(-w_step, std::min(w_step, cmd_w - filtered_pub_yaw_rate_));

        filtered_pub_dx_ = cmd_vx * dt_filtered;
        filtered_pub_dy_ = cmd_vy * dt_filtered;
        filtered_pub_dyaw_ = cmd_w * dt_filtered;
        integrateSe2Delta(filtered_pub_x_, filtered_pub_y_, filtered_pub_yaw_,
          Eigen::Vector3d(filtered_pub_dx_, filtered_pub_dy_, filtered_pub_dyaw_));
        filtered_pub_vx_ = cmd_vx;
        filtered_pub_vy_ = cmd_vy;
        filtered_pub_yaw_rate_ = cmd_w;
      }

      last_filtered_publish_stamp_ = nowt;
      filtered_x = filtered_pub_x_;
      filtered_y = filtered_pub_y_;
      filtered_yaw = filtered_pub_yaw_;
      filtered_twist_x = filtered_pub_vx_;
      filtered_twist_y = filtered_pub_vy_;
      filtered_twist_yaw = filtered_pub_yaw_rate_;
      has_filtered_twist = has_filtered_publish_state_;
      have_filtered_pose = has_filtered_publish_state_;
    }
  }

  auto fillOdom = [&](nav_msgs::msg::Odometry & odom_msg,
                      double x, double y, double yaw_angle,
                      double vx, double vy, double wz,
                      bool has_twist) {
    odom_msg.header.stamp = nowt;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    const auto q = quatFromYaw(yaw_angle);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
    odom_msg.pose.covariance[0] = std::max(0.0, odom_cov_xy_total);
    odom_msg.pose.covariance[7] = std::max(0.0, odom_cov_xy_total);
    odom_msg.pose.covariance[14] = 1.0e6;
    odom_msg.pose.covariance[21] = 1.0e6;
    odom_msg.pose.covariance[28] = 1.0e6;
    odom_msg.pose.covariance[35] = std::max(0.0, odom_cov_yaw_total);

    std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
    odom_msg.twist.covariance[0] = has_twist ? std::max(1.0e-4, 0.25 * odom_cov_xy_total) : 1.0e6;
    odom_msg.twist.covariance[7] = has_twist ? std::max(1.0e-4, 0.25 * odom_cov_xy_total) : 1.0e6;
    odom_msg.twist.covariance[14] = 1.0e6;
    odom_msg.twist.covariance[21] = 1.0e6;
    odom_msg.twist.covariance[28] = 1.0e6;
    odom_msg.twist.covariance[35] = has_twist ? std::max(1.0e-5, 0.25 * odom_cov_yaw_total) : 1.0e6;

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = wz;
  };

  fillOdom(odom_raw, odom_x, odom_y, odom_yaw, raw_twist_x, raw_twist_y, raw_twist_yaw, has_raw_twist);
  pub_odom_raw_->publish(odom_raw);
  if (pub_deskew_twist_) {
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = nowt;
    twist.header.frame_id = base_frame_;
    twist.twist.linear.x = raw_twist_x;
    twist.twist.linear.y = raw_twist_y;
    twist.twist.angular.z = raw_twist_yaw;
    pub_deskew_twist_->publish(twist);
  }

  if (pub_odom_filtered_ && have_filtered_pose) {
    fillOdom(
      odom_filtered, filtered_x, filtered_y, filtered_yaw,
      filtered_twist_x, filtered_twist_y, filtered_twist_yaw, has_filtered_twist);
    pub_odom_filtered_->publish(odom_filtered);
  }

  stopped.data = stopped_now;
  pub_stopped_->publish(stopped);

  {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = nowt;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (lidar_odom_enable_ && !has_lidar) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    status.name = "localization/gyro_odometer";
    if (!lidar.valid && lidar.rejection_reason != "not_evaluated") {
      status.message = stopped_now ?
        "registration unavailable while stationary; holding last accepted pose" :
        "registration unavailable; holding last accepted pose";
    } else if (stopped_now) {
      status.message = "stopped";
    } else if (lidar_odom_enable_ && !has_lidar) {
      status.message = "waiting for a valid LiDAR registration; holding pose";
    } else {
      status.message = "running";
    }
    status.hardware_id = "none";

    auto add = [&](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(entry);
    };

    add("use_wheel_speed", boolString(use_wheel_speed_));
    add("lidar_odom_enabled", boolString(lidar_odom_enable_));
    add("yaw_imu", std::to_string(yaw));
    add("bg_est", std::to_string(bg));
    add("yaw_rate_out", std::to_string(has_yaw_rate_out ? yaw_rate_out : 0.0));
    add("v_pred", std::to_string(v_pred));
    add("has_v_pred", boolString(has_v_pred));
    add("twist_speed_source", speed_source);
    add("raw_twist_x", std::to_string(raw_twist_x));
    add("raw_twist_y", std::to_string(raw_twist_y));
    add("raw_twist_yaw", std::to_string(raw_twist_yaw));
    add("filtered_twist_x", std::to_string(filtered_twist_x));
    add("filtered_twist_y", std::to_string(filtered_twist_y));
    add("filtered_twist_yaw", std::to_string(filtered_twist_yaw));
    add("odom_cov_xy_total", std::to_string(odom_cov_xy_total));
    add("odom_cov_yaw_total", std::to_string(odom_cov_yaw_total));
    add("raw_odom_topic", out_odom_topic_);
    add("accepted_scan_odom_enabled", boolString(accepted_scan_odom_enable_));
    add(
      "accepted_scan_odom_topic",
      accepted_scan_odom_enable_ ? accepted_scan_odom_topic_ : std::string("disabled"));
    add(
      "accepted_scan_odom_published_count",
      std::to_string(accepted_scan_odom_published_count));
    add("accepted_scan_odom_stamp_contract", "accepted_input_scan_header_stamp");
    add("external_submap_snapshot_enabled", boolString(external_submap_snapshot_enable_));
    add("external_submap_snapshot_topic", external_submap_snapshot_topic_);
    add("external_submap_snapshot_exact_key_contract",
      "session+generation+sequence+accepted_scan_stamp");
    add("external_submap_snapshot_session", std::to_string(external_snapshot_session));
    add("external_submap_snapshot_generation", std::to_string(external_snapshot_generation));
    add("external_submap_snapshot_sequence", std::to_string(external_snapshot_sequence));
    add("external_submap_snapshot_published_count",
      std::to_string(external_snapshot_published_count));
    add("external_submap_snapshot_conversion_last_ms",
      std::to_string(external_snapshot_conversion_last_ms));
    add("external_submap_snapshot_conversion_mean_ms",
      std::to_string(external_snapshot_published_count == 0 ? 0.0 :
      external_snapshot_conversion_sum_ms /
      static_cast<double>(external_snapshot_published_count)));
    add("external_submap_snapshot_conversion_max_ms",
      std::to_string(external_snapshot_conversion_max_ms));
    add(
      "filtered_odom_topic",
      pub_odom_filtered_ ? out_filtered_odom_topic_ : std::string("disabled"));

    add("imu_corrected.enable", boolString(imu_corrected_enable_));
    add("imu_corrected.apply_tf", boolString(imu_corrected_apply_tf_));
    add(
      "imu_corrected.linear_acceleration_scale",
      std::to_string(imu_linear_acceleration_scale_));
    add("imu_extrinsic_cached", boolString(imu_extrinsic_cached));
    add("imu_frame", imu_extrinsic_cached ? imu_frame_id : std::string(""));

    if (use_wheel_speed_) {
      add("wheel_scale", std::to_string(wheel_scale));
      add("wheel_raw", std::to_string(v_raw));
      add("wheel_v_scaled", std::to_string(v_wheel));
      add(
        "wheel_observability_assist_enabled",
        boolString(wheel_observability_assist_enable_));
    }

    if (lidar_odom_enable_) {
      add("icp_guess_mode_last", last_icp_guess_mode);
      add("icp_guess_mode_next", next_icp_guess_mode);
      add(
        "observability_debug_pub_enabled",
        boolString(lidar_observability_debug_pub_enable_));
      add(
        "observability_debug_topic",
        lidar_observability_debug_pub_enable_ ?
        lidar_observability_debug_topic_ : std::string(""));
      add("lidar_valid", boolString(lidar.valid));
      add("lidar_rejection_reason", lidar.rejection_reason);
      add("lidar_converged", boolString(lidar.converged));
      add("lidar_fitness", std::to_string(lidar.fitness));
      add("lidar_inlier_ratio", std::to_string(lidar.inlier_ratio));
      add("lidar_tracking_mode", lidar_tracking_mode_name_);
      add("lidar_registration_source", lidar.registration_source);
      add("lidar_last_registration_source", last_registration_source);
      add("lidar_dx", std::to_string(lidar.dx));
      add("lidar_dy", std::to_string(lidar.dy));
      add("lidar_dyaw", std::to_string(lidar.dyaw));
      add("lidar_raw_dx", std::to_string(lidar.raw_dx));
      add("lidar_raw_dy", std::to_string(lidar.raw_dy));
      add("lidar_raw_dyaw", std::to_string(lidar.raw_dyaw));
      add("lidar_v", std::to_string(lidar.v));

      add("observability_enabled", boolString(lidar.observability.enabled));
      add("lidar_has_hessian", boolString(lidar.observability.has_hessian));
      add(
        "has_directional_information",
        boolString(lidar.observability.has_directional_information));
      add(
        "information_ratio_min",
        std::to_string(lidar.observability.information_ratio_min));
      add(
        "information_ratio_mid",
        std::to_string(lidar.observability.information_ratio_mid));
      add(
        "information_ratio_max",
        std::to_string(lidar.observability.information_ratio_max));
      add(
        "information_deficit",
        std::to_string(lidar.observability.information_deficit));
      add(
        "stationary_projected_motion_metric",
        std::to_string(lidar.observability.stationary_projected_motion_metric));
      add(
        "wheel_prior_available",
        boolString(lidar.observability.wheel_prior_available));
      add(
        "wheel_assist_active",
        boolString(lidar.observability.wheel_assisted));
      add(
        "wheel_assist_blend",
        std::to_string(lidar.observability.wheel_assist_blend));
      add(
        "wheel_assist_correction_metric",
        std::to_string(lidar.observability.wheel_assist_correction_metric));
      add("wheel_distance", std::to_string(lidar.observability.wheel_distance));
      add("wheel_prior_dx", std::to_string(lidar.observability.prior_dx));
      add("wheel_prior_dy", std::to_string(lidar.observability.prior_dy));
      add("wheel_prior_dyaw", std::to_string(lidar.observability.prior_dyaw));
      add(
        "prior_difference_metric",
        std::to_string(lidar.observability.prior_difference_metric));
      add(
        "prior_difference_translation",
        std::to_string(lidar.observability.prior_difference_translation));
      add(
        "prior_difference_yaw",
        std::to_string(lidar.observability.prior_difference_yaw));
      add("scan_speed_mps", std::to_string(lidar.observability.scan_speed_mps));
      add("wheel_speed_mps", std::to_string(lidar.observability.wheel_speed_mps));
      add(
        "speed_difference_mps",
        std::to_string(lidar.observability.speed_difference_mps));

    }

    arr.status.push_back(status);
    pub_diag_->publish(arr);
  }
}

void GyroOdometerNode::publishObservabilityDebug(
  const rclcpp::Time & stamp, const LidarOdomSample & sample,
  const std::string & guess_mode_used, const std::string & next_guess_mode)
{
  if (!pub_observability_debug_) {
    return;
  }

  std_msgs::msg::String message;
  std::ostringstream stream;
  stream << "stamp: " << formatDouble(stamp.seconds(), 6) << "\n";
  stream << "guess_mode_used: " << guess_mode_used << "\n";
  stream << "next_guess_mode: " << next_guess_mode << "\n";
  stream << "tracking_mode: " << lidar_tracking_mode_name_ << "\n";
  stream << "registration_source: " << sample.registration_source << "\n";
  stream << "registration_valid: " << boolString(sample.valid) << "\n";
  stream << "rejection_reason: " << sample.rejection_reason << "\n";
  stream << "converged: " << boolString(sample.converged) << "\n";
  stream << "fitness: " << formatDouble(sample.fitness) << "\n";
  stream << "inlier_ratio: " << formatDouble(sample.inlier_ratio) << "\n";
  stream << "observability_enabled: " <<
    boolString(sample.observability.enabled) << "\n";
  stream << "has_hessian: " <<
    boolString(sample.observability.has_hessian) << "\n";
  stream << "has_directional_information: " <<
    boolString(sample.observability.has_directional_information) << "\n";
  stream << "information_ratio_min: " <<
    formatDouble(sample.observability.information_ratio_min) << "\n";
  stream << "information_ratio_mid: " <<
    formatDouble(sample.observability.information_ratio_mid) << "\n";
  stream << "information_ratio_max: " <<
    formatDouble(sample.observability.information_ratio_max) << "\n";
  stream << "information_deficit: " <<
    formatDouble(sample.observability.information_deficit) << "\n";
  stream << "raw_dx: " << formatDouble(sample.raw_dx) << "\n";
  stream << "raw_dy: " << formatDouble(sample.raw_dy) << "\n";
  stream << "raw_dyaw: " << formatDouble(sample.raw_dyaw) << "\n";
  stream << "used_dx: " << formatDouble(sample.dx) << "\n";
  stream << "used_dy: " << formatDouble(sample.dy) << "\n";
  stream << "used_dyaw: " << formatDouble(sample.dyaw) << "\n";
  stream << "stationary_now: " <<
    boolString(sample.observability.stationary_now) << "\n";
  stream << "stationary_projected_motion_metric: " <<
    formatDouble(sample.observability.stationary_projected_motion_metric) << "\n";
  stream << "wheel_prior_available: " <<
    boolString(sample.observability.wheel_prior_available) << "\n";
  stream << "wheel_assist_active: " <<
    boolString(sample.observability.wheel_assisted) << "\n";
  stream << "wheel_assist_blend: " <<
    formatDouble(sample.observability.wheel_assist_blend) << "\n";
  stream << "wheel_assist_correction_metric: " <<
    formatDouble(sample.observability.wheel_assist_correction_metric) << "\n";
  stream << "wheel_distance: " <<
    formatDouble(sample.observability.wheel_distance) << "\n";
  stream << "wheel_prior_dx: " <<
    formatDouble(sample.observability.prior_dx) << "\n";
  stream << "wheel_prior_dy: " <<
    formatDouble(sample.observability.prior_dy) << "\n";
  stream << "wheel_prior_dyaw: " <<
    formatDouble(sample.observability.prior_dyaw) << "\n";
  stream << "prior_difference_metric: " <<
    formatDouble(sample.observability.prior_difference_metric) << "\n";
  stream << "prior_difference_translation: " <<
    formatDouble(sample.observability.prior_difference_translation) << "\n";
  stream << "prior_difference_yaw: " <<
    formatDouble(sample.observability.prior_difference_yaw) << "\n";
  stream << "scan_speed_mps: " <<
    formatDouble(sample.observability.scan_speed_mps) << "\n";
  stream << "wheel_speed_mps: " <<
    formatDouble(sample.observability.wheel_speed_mps) << "\n";
  stream << "speed_difference_mps: " <<
    formatDouble(sample.observability.speed_difference_mps) << "\n";
  stream << "yaw_metric_m: " <<
    formatDouble(lidar_smoother_hessian_yaw_metric_m_) << "\n";
  stream << "minimum_direction_ratio: " <<
    formatDouble(lidar_smoother_hessian_min_direction_ratio_) << "\n";
  stream << "wheel_assist_max_blend: " <<
    formatDouble(wheel_observability_assist_max_blend_) << "\n";
  stream << "wheel_assist_power: " <<
    formatDouble(wheel_observability_assist_power_) << "\n";
  message.data = stream.str();
  pub_observability_debug_->publish(message);
}

void GyroOdometerNode::publishDiagnostics(const rclcpp::Time & stamp, const std::string & level, const std::string & msg)
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.level = diagLevelFromString(level);
  st.name = "localization/gyro_odometer";
  st.message = msg;
  st.hardware_id = "none";
  arr.status.push_back(st);
  pub_diag_->publish(arr);
}

}  // namespace pure_gyro_odometer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pure_gyro_odometer::GyroOdometerNode)
