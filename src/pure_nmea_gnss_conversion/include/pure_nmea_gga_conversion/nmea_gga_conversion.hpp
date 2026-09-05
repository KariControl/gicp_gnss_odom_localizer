#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "pure_nmea_gga_conversion/imu_yaw_integrator.hpp"
#include "pure_nmea_gga_conversion/trajectory_heading_quality.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <pure_gnss_msgs/msg/gnss_fusion_input.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class NmeaGgaConversion : public rclcpp::Node
{
public:
  explicit NmeaGgaConversion(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct GgaMeasurement
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    double hdop{std::numeric_limits<double>::quiet_NaN()};
    int fix_quality{0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double position_confidence{0.0};
    double sigma_xy_m{1.0e3};
    double differential_age_sec{std::numeric_limits<double>::quiet_NaN()};
    int8_t navsat_status{sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX};
    bool valid_fix{false};
  };

  struct DopplerMeasurement
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    double speed{0.0};
    double speed_variance{1.0e6};
    double heading{0.0};
    double heading_variance{1.0e6};
    bool has_valid_heading{false};
  };

  struct MatchedMeasurement
  {
    GgaMeasurement primary;
    bool has_secondary{false};
    GgaMeasurement secondary;
    bool has_doppler{false};
    DopplerMeasurement doppler;
  };

  struct OutputPose
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};

    // Observation pose in map. Translation is either the measured antenna
    // position (position_is_base_link=false) or a derived base_link position.
    // Orientation contains the estimated base_link yaw only when
    // heading_valid=true.
    Eigen::Isometry3d T_map_observation{Eigen::Isometry3d::Identity()};
    Eigen::Matrix3d covariance_xy_yaw{Eigen::Matrix3d::Zero()};
    double cov_z{1.0e6};
    double speed{0.0};
    double speed_variance{1.0e6};
    float position_confidence{0.0F};
    int8_t fix_status{sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX};
    int primary_fix_quality{0};
    bool dual_antenna_used{false};
    bool heading_valid{false};
    bool position_is_base_link{false};
    bool observation_point_valid{false};
    Eigen::Vector3d observation_point_in_base{Eigen::Vector3d::Zero()};
    std::string heading_source{"none"};
    std::string note{"none"};
  };

  // Single IMU sample stored for yaw-rate integration.
  struct ImuSample
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    double gyro_z{0.0};
  };

  void declareParameters();
  void loadParameters();
  void validateParameters() const;
  void initializeProjection();

  void primarySentenceCallback(const nmea_msgs::msg::Sentence::SharedPtr msg);
  void secondarySentenceCallback(const nmea_msgs::msg::Sentence::SharedPtr msg);
  void dopplerCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void imuCorrectedCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void stoppedCallback(const std_msgs::msg::Bool::SharedPtr msg);

  bool parseGgaSentence(
    const nmea_msgs::msg::Sentence & msg, GgaMeasurement & output, std::string & error) const;
  bool validateNmeaChecksum(const std::string & sentence) const;
  bool parseLatitude(const std::string & raw, const std::string & hemi, double & lat_deg) const;
  bool parseLongitude(const std::string & raw, const std::string & hemi, double & lon_deg) const;
  void gaussKruger(double rad_phi, double rad_lambda, double & x, double & y) const;

  double fixScore(int fix_quality) const;
  double hdopScore(double hdop) const;
  double positionConfidence(int fix_quality, double hdop) const;
  double horizontalSigmaMeters(int fix_quality, double hdop) const;
  double differentialAgeScale(double age_sec) const;
  int8_t ggaQualityToNavSatStatus(int fix_quality) const;

  void onFlushTimer();
  void onHeartbeat();
  void publishStatusOnly(const rclcpp::Time & stamp, int8_t fix_status, float confidence);
  void publishOutput(const OutputPose & output);
  void publishConfidence(float confidence);

  void updateLatestInputStampLocked(const rclcpp::Time & stamp);
  void pruneBuffersLocked();
  void bufferPrimaryMeasurementLocked(const GgaMeasurement & primary);
  void bufferSecondaryMeasurementLocked(const GgaMeasurement & secondary);
  void bufferDopplerMeasurementLocked(const DopplerMeasurement & doppler);
  void observeTrajectoryHeadingMeasurementLocked(const GgaMeasurement & measurement);
  void applyTrajectoryHeadingEpochResetLocked(
    const pure_nmea_gga_conversion::TrajectoryHeadingObservationResult & reset);
  bool findClosestSecondaryMeasurementLocked(
    const rclcpp::Time & stamp, std::size_t & index) const;
  bool findClosestDopplerMeasurementLocked(
    const rclcpp::Time & stamp, std::size_t & index) const;
  std::vector<MatchedMeasurement> collectReadyMatchesLocked(const rclcpp::Time & now);

  OutputPose buildOutputPose(const MatchedMeasurement & matched);
  bool resolveAntennaPosition(
    const std::string & antenna_frame_id,
    const std::vector<double> & fallback_position_base,
    Eigen::Vector3d & antenna_position_base) const;

  rclcpp::Time resolveStamp(const builtin_interfaces::msg::Time & stamp) const;
  int64_t syncToleranceNanoseconds() const;
  int64_t bufferRetentionNanoseconds() const;

  // Integrates the corrected gyro_z stream over [t_start, t_end].
  // Caller must hold mutex_. The result fails closed when the stream does not
  // bracket the interval or contains an excessive sample gap.
  pure_nmea_gga_conversion::ImuYawIntegrationResult integrateImuYawLocked(
    const rclcpp::Time & t_start, const rclcpp::Time & t_end) const;

  void setSingleAntennaObservation(
    OutputPose & output,
    const GgaMeasurement & primary,
    const Eigen::Vector3d & antenna_offset_base,
    bool has_antenna_geometry,
    double yaw,
    double yaw_variance,
    const std::string & heading_source,
    const std::string & note) const;
  static Eigen::Matrix3d baseCovarianceFromAntennaObservation(
    const Eigen::Matrix3d & observation_covariance,
    const Eigen::Vector2d & antenna_offset_base,
    double yaw);

  static double deg2rad(double degree);
  static double sanitizeVariance(double value);
  static double sanitizeCovariance(double value);
  static double normalizeAngle(double angle);
  static Eigen::Quaterniond quatFromYaw(double yaw);

  // Parameters
  std::string projector_type_{"TransverseMercator"};
  std::string vertical_datum_{"WGS84"};
  double map_origin_lat_deg_{35.681236};
  double map_origin_lon_deg_{139.767125};
  double map_origin_altitude_ignored_{0.0};
  double scale_factor_{0.9996};
  bool use_legacy_projection_params_{false};

  // Legacy aliases kept for backward compatibility.
  std::vector<double> p0_;
  std::vector<double> gnss0_;
  double a_{6378137.0};
  double F_{298.257223563};
  double m0_{0.9996};
  double min_x_{0.0};
  double min_y_{0.0};
  bool subtract_min_offset_{false};

  std::string frame_id_{"map"};
  std::string child_frame_id_{"base_link"};

  bool use_secondary_gga_{false};
  bool use_doppler_heading_{false};
  bool use_tf_for_antenna_geometry_{true};
  bool allow_parameter_antenna_fallback_{false};
  std::string primary_antenna_frame_id_{"gnss_primary_antenna"};
  std::string secondary_antenna_frame_id_{"gnss_secondary_antenna"};
  std::vector<double> primary_antenna_position_base_;
  std::vector<double> secondary_antenna_position_base_;

  // Unknown heading is represented by heading_valid=false and a large
  // covariance. Legacy initial/previous-heading fallback parameters are
  // accepted as no-ops for migration but never create a heading observation.
  double yaw_variance_without_heading_{1.0e6};

  // Trajectory-based heading estimation (position-difference method).
  // Used when dual-antenna and Doppler heading are both unavailable.
  // Heading = atan2(dy, dx) between two GNSS fixes separated by min_baseline_m.
  // σ_yaw = sqrt(σ1² + σ2²) / baseline  (error propagation, with floor).
  bool use_trajectory_heading_{true};
  double trajectory_heading_min_baseline_m_{3.0};
  // Rolling history retention and current-heading reference eligibility are
  // intentionally separate. A point can remain in history without being a
  // valid reference for the current tangent estimate.
  double trajectory_heading_max_age_sec_{5.0};
  double trajectory_heading_max_reference_age_sec_{5.0};
  double trajectory_heading_epoch_max_gap_sec_{0.5};
  double trajectory_heading_input_max_gap_sec_{2.0};
  double trajectory_heading_max_turn_activity_rad_{0.7853981633974483};
  double trajectory_heading_max_seed_innovation_rad_{0.07};
  double trajectory_heading_sigma_floor_deg_{3.0};
  double trajectory_heading_min_confidence_{0.3};
  double trajectory_heading_max_position_sigma_m_{5.0};
  double trajectory_heading_max_yaw_variance_rad2_{10.0};

  double doppler_min_speed_mps_{1.33};
  double doppler_speed_sigma_th_{10.0};
  double doppler_heading_sigma_deg_th_{10.0};

  // IMU yaw-rate heading parameters.
  bool use_imu_yaw_rate_heading_{false};
  double imu_yaw_rate_deadband_radps_{0.002};
  double imu_yaw_rate_max_integration_sec_{15.0};
  double imu_yaw_rate_sigma_per_sqrt_sec_{0.05};
  double imu_yaw_rate_max_abs_radps_{3.0};
  double imu_yaw_rate_max_sample_gap_sec_{0.10};
  double imu_yaw_rate_max_boundary_gap_sec_{0.05};

  std::vector<double> fix_confidence_table_;
  std::vector<double> fix_sigma_xy_table_m_;
  double hdop_good_threshold_{1.2};
  double hdop_confidence_exponent_{2.0};
  double min_hdop_for_sigma_{0.5};
  double max_hdop_for_sigma_{20.0};
  double vertical_sigma_scale_{2.0};

  double differential_age_warn_sec_{1.0};
  double differential_age_invalid_sec_{2.0};

  double gnss_usable_confidence_threshold_{0.3};

  double dual_antenna_min_baseline_m_{0.3};
  double dual_antenna_baseline_tolerance_m_{2.0};
  double dual_antenna_heading_sigma_floor_deg_{0.5};
  // Scale factor for heading sigma inflation due to baseline stretch.
  // heading_sigma *= (1 + scale * baseline_error / base_baseline)
  // Guards against perpendicular position errors that corrupt heading
  // without triggering the baseline_error gate.
  double dual_antenna_heading_sigma_baseline_inflate_scale_{2.0};

  double sync_tolerance_sec_{0.03};
  double buffer_retention_sec_{1.0};
  int max_buffer_size_{100};

  bool publish_global_pose_{true};
  bool publish_pose_with_covariance_{true};
  bool publish_gnss_odometry_{true};
  bool publish_gnss_fusion_input_{true};
  bool publish_confidence_{true};
  bool publish_diagnostics_{true};

  double heartbeat_hz_{2.0};
  double flush_timer_hz_{50.0};
  double input_stale_timeout_sec_{2.0};

  bool enable_checksum_validation_{true};
  bool use_ellipsoid_height_from_geoid_separation_{true};
  bool allow_reception_time_for_zero_stamp_{false};

  // Projection state
  double origin_lat_rad_{0.0};
  double origin_lon_rad_{0.0};
  double offset_z_{0.0};
  std::array<double, 5> alpha_{};
  std::array<double, 6> A_{};
  double A_bar_{0.0};
  double S_bar_phi0_{0.0};
  double kt_{0.0};
  Eigen::Rotation2Dd R_{0.0};

  // Runtime state
  mutable std::mutex mutex_;
  std::deque<GgaMeasurement> primary_buffer_;
  std::deque<GgaMeasurement> secondary_buffer_;
  std::deque<DopplerMeasurement> doppler_buffer_;
  bool has_latest_input_stamp_{false};
  rclcpp::Time latest_input_stamp_{0, 0, RCL_ROS_TIME};

  bool has_last_primary_{false};
  bool has_last_secondary_{false};
  bool has_last_doppler_{false};
  bool has_last_output_{false};
  bool has_last_valid_heading_{false};
  bool last_valid_heading_seed_is_trajectory_{false};
  std::size_t last_valid_heading_trajectory_epoch_{0U};

  // IMU yaw-rate integration state (protected by mutex_).
  std::deque<ImuSample> imu_buf_;
  bool is_stopped_{false};
  rclcpp::Time last_valid_heading_stamp_{0, 0, RCL_ROS_TIME};
  double last_valid_heading_cov_{1.0e6};

  pure_nmea_gga_conversion::TrajectoryHeadingHistory trajectory_heading_history_;

  rclcpp::Time last_primary_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_output_stamp_{0, 0, RCL_ROS_TIME};

  int last_primary_fix_quality_{0};
  double last_primary_hdop_{std::numeric_limits<double>::quiet_NaN()};
  double last_primary_differential_age_sec_{std::numeric_limits<double>::quiet_NaN()};
  float last_position_confidence_{0.0F};
  double last_output_cov_xy_{1.0e6};
  double last_output_cov_yaw_{1.0e6};
  int8_t last_fix_status_{sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX};
  double last_valid_heading_rad_{0.0};
  double last_speed_mps_{0.0};
  double last_imu_integration_max_gap_sec_{0.0};
  std::size_t last_imu_integration_sample_count_{0U};
  std::string last_imu_integration_reason_{"not_used"};
  double last_trajectory_reference_age_sec_{std::numeric_limits<double>::quiet_NaN()};
  double last_trajectory_baseline_m_{0.0};
  double last_trajectory_turn_activity_rad_{std::numeric_limits<double>::quiet_NaN()};
  double last_trajectory_seed_innovation_rad_{std::numeric_limits<double>::quiet_NaN()};
  std::size_t trajectory_heading_epoch_reset_count_{0U};
  std::string last_trajectory_epoch_reset_reason_{"none"};
  std::string last_trajectory_heading_reject_reason_{"not_evaluated"};
  std::string last_trajectory_continuity_reason_{"not_evaluated"};
  std::string last_heading_source_{"none"};
  std::string last_note_{"none"};
  std::string last_parse_error_{"none"};
  bool last_dual_antenna_used_{false};

  // ROS interface
  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr sub_primary_gga_;
  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr sub_secondary_gga_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_doppler_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_corrected_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stopped_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<pure_gnss_msgs::msg::GnssFusionInput>::SharedPtr pub_gnss_fusion_input_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_confidence_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;

  rclcpp::TimerBase::SharedPtr flush_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
