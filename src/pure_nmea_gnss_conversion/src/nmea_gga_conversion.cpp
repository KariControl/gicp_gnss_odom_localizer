#include "pure_nmea_gga_conversion/nmea_gga_conversion.hpp"
#include "pure_nmea_gga_conversion/trajectory_heading_quality.hpp"

#include <functional>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2/exceptions.h>

namespace
{

constexpr double kUnknownVariance = 1.0e6;
constexpr double kPi = 3.14159265358979323846;

std::string trim(const std::string & s)
{
  const auto first = std::find_if_not(s.begin(), s.end(), [](unsigned char ch) {
      return std::isspace(ch) != 0;
    });
  if (first == s.end()) {
    return std::string{};
  }
  const auto last = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char ch) {
      return std::isspace(ch) != 0;
    }).base();
  return std::string(first, last);
}

std::string toLowerCopy(const std::string & s)
{
  std::string out = s;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
  return out;
}

std::vector<std::string> split(const std::string & s, char delimiter)
{
  std::vector<std::string> fields;
  std::string current;
  std::stringstream ss(s);
  while (std::getline(ss, current, delimiter)) {
    fields.push_back(current);
  }
  if (!s.empty() && s.back() == delimiter) {
    fields.emplace_back();
  }
  return fields;
}

bool parseDouble(const std::string & text, double & value)
{
  if (text.empty()) {
    return false;
  }
  try {
    size_t consumed = 0;
    value = std::stod(text, &consumed);
    return consumed == text.size() && std::isfinite(value);
  } catch (const std::exception &) {
    return false;
  }
}

bool parseInt(const std::string & text, int & value)
{
  if (text.empty()) {
    return false;
  }
  try {
    size_t consumed = 0;
    value = std::stoi(text, &consumed);
    return consumed == text.size();
  } catch (const std::exception &) {
    return false;
  }
}

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

std::string toStringWithPrecision(double value, int precision = 6)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

}  // namespace

NmeaGgaConversion::NmeaGgaConversion(const rclcpp::NodeOptions & options)
: rclcpp::Node("nmea_gga_conversion", options)
{
  declareParameters();
  loadParameters();
  validateParameters();
  trajectory_heading_history_.configure(
    trajectory_heading_max_age_sec_,
    trajectory_heading_epoch_max_gap_sec_,
    trajectory_heading_input_max_gap_sec_);
  initializeProjection();

  if (use_tf_for_antenna_geometry_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  sub_primary_gga_ = this->create_subscription<nmea_msgs::msg::Sentence>(
    "primary_gga", rclcpp::SensorDataQoS(),
    std::bind(&NmeaGgaConversion::primarySentenceCallback, this, std::placeholders::_1));

  if (use_secondary_gga_) {
    sub_secondary_gga_ = this->create_subscription<nmea_msgs::msg::Sentence>(
      "secondary_gga", rclcpp::SensorDataQoS(),
      std::bind(&NmeaGgaConversion::secondarySentenceCallback, this, std::placeholders::_1));
  }

  if (use_doppler_heading_) {
    sub_doppler_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "fix_velocity", rclcpp::SensorDataQoS(),
      std::bind(&NmeaGgaConversion::dopplerCallback, this, std::placeholders::_1));
  }

  if (use_imu_yaw_rate_heading_) {
    sub_imu_corrected_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_corrected", rclcpp::SensorDataQoS(),
      std::bind(&NmeaGgaConversion::imuCorrectedCallback, this, std::placeholders::_1));
    sub_stopped_ = this->create_subscription<std_msgs::msg::Bool>(
      "stopped", rclcpp::SensorDataQoS(),
      std::bind(&NmeaGgaConversion::stoppedCallback, this, std::placeholders::_1));
  }

  if (publish_global_pose_) {
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("global_pose", 10);
  }
  if (publish_pose_with_covariance_) {
    pub_pose_cov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "global_pose_with_covariance", 10);
  }
  if (publish_gnss_odometry_) {
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("gnss_odometry", 10);
  }
  if (publish_gnss_fusion_input_) {
    pub_gnss_fusion_input_ = this->create_publisher<pure_gnss_msgs::msg::GnssFusionInput>(
      "gnss_fusion_input", 10);
  }
  if (publish_confidence_) {
    pub_confidence_ = this->create_publisher<std_msgs::msg::Float32>("gnss_confidence", 10);
  }
  if (publish_diagnostics_) {
    pub_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  }

  flush_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / flush_timer_hz_),
    std::bind(&NmeaGgaConversion::onFlushTimer, this));

  if (publish_diagnostics_) {
    heartbeat_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / heartbeat_hz_),
      std::bind(&NmeaGgaConversion::onHeartbeat, this));
  }

  RCLCPP_INFO(
    this->get_logger(),
    "nmea_gga_conversion started. frame_id=%s child_frame_id=%s secondary=%s doppler=%s tf_geometry=%s checksum=%s projector=%s datum=%s origin=(%.8f, %.8f) scale_factor=%.7f legacy_projection=%s",
    frame_id_.c_str(), child_frame_id_.c_str(),
    use_secondary_gga_ ? "true" : "false",
    use_doppler_heading_ ? "true" : "false",
    use_tf_for_antenna_geometry_ ? "true" : "false",
    enable_checksum_validation_ ? "true" : "false",
    projector_type_.c_str(), vertical_datum_.c_str(),
    use_legacy_projection_params_ ? gnss0_[0] : map_origin_lat_deg_,
    use_legacy_projection_params_ ? gnss0_[1] : map_origin_lon_deg_,
    use_legacy_projection_params_ ? m0_ : scale_factor_,
    use_legacy_projection_params_ ? "true" : "false");
}

void NmeaGgaConversion::declareParameters()
{
  // Preferred projection schema aligned with TransverseMercator + WGS84 map metadata.
  this->declare_parameter("projector_type", std::string("TransverseMercator"));
  this->declare_parameter("vertical_datum", std::string("WGS84"));
  this->declare_parameter("map_origin.latitude", 35.681236);
  this->declare_parameter("map_origin.longitude", 139.767125);
  this->declare_parameter("map_origin.altitude", 0.0);
  this->declare_parameter("scale_factor", 0.9996);

  // Legacy aliases kept for compatibility with the original pure_gnss_conversion style.
  this->declare_parameter("use_legacy_projection_params", false);
  this->declare_parameter("p0", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("gnss0", std::vector<double>{35.681236, 139.767125, 0.0});
  this->declare_parameter("a", 6378137.0);
  this->declare_parameter("F", 298.257223563);
  this->declare_parameter("m0", 0.9996);

  // Optional local XY offset stage, useful only when matching legacy shifted maps.
  this->declare_parameter("min_x", 0.0);
  this->declare_parameter("min_y", 0.0);
  this->declare_parameter("subtract_min_offset", false);

  this->declare_parameter("frame_id", std::string("map"));
  this->declare_parameter("child_frame_id", std::string("base_link"));

  this->declare_parameter("use_secondary_gga", false);
  this->declare_parameter("use_doppler_heading", false);
  this->declare_parameter("use_tf_for_antenna_geometry", true);
  this->declare_parameter("allow_parameter_antenna_fallback", false);
  this->declare_parameter("primary_antenna_frame_id", std::string("gnss_primary_antenna"));
  this->declare_parameter("secondary_antenna_frame_id", std::string("gnss_secondary_antenna"));
  this->declare_parameter("primary_antenna_position_base", std::vector<double>{});
  this->declare_parameter("secondary_antenna_position_base", std::vector<double>{});

  // Deprecated no-op parameters retained so older YAML files still load.
  // Unknown heading is never promoted to a measurement from these values.
  this->declare_parameter("initial_heading_deg", 0.0);
  this->declare_parameter("use_previous_heading_fallback", false);
  this->declare_parameter("apply_lever_arm_without_heading", false);
  this->declare_parameter("no_heading_lever_arm_extra_cov_scale", 1.0);
  this->declare_parameter("yaw_variance_without_heading", kUnknownVariance);

  this->declare_parameter("use_trajectory_heading", use_trajectory_heading_);
  this->declare_parameter("trajectory_heading_min_baseline_m", trajectory_heading_min_baseline_m_);
  this->declare_parameter("trajectory_heading_max_age_sec", trajectory_heading_max_age_sec_);
  this->declare_parameter(
    "trajectory_heading_max_reference_age_sec", trajectory_heading_max_reference_age_sec_);
  this->declare_parameter(
    "trajectory_heading_epoch_max_gap_sec", trajectory_heading_epoch_max_gap_sec_);
  this->declare_parameter(
    "trajectory_heading_input_max_gap_sec", trajectory_heading_input_max_gap_sec_);
  this->declare_parameter(
    "trajectory_heading_max_turn_activity_rad", trajectory_heading_max_turn_activity_rad_);
  this->declare_parameter(
    "trajectory_heading_max_seed_innovation_rad",
    trajectory_heading_max_seed_innovation_rad_);
  this->declare_parameter("trajectory_heading_sigma_floor_deg", trajectory_heading_sigma_floor_deg_);
  this->declare_parameter("trajectory_heading_min_confidence", trajectory_heading_min_confidence_);
  this->declare_parameter(
    "trajectory_heading_max_position_sigma_m", trajectory_heading_max_position_sigma_m_);
  this->declare_parameter(
    "trajectory_heading_max_yaw_variance_rad2",
    trajectory_heading_max_yaw_variance_rad2_);

  this->declare_parameter("doppler_min_speed_mps", 1.33);
  this->declare_parameter("doppler_speed_sigma_th", 10.0);
  this->declare_parameter("doppler_heading_sigma_deg_th", 10.0);

  this->declare_parameter("use_imu_yaw_rate_heading", false);
  this->declare_parameter("imu_yaw_rate_deadband_radps", imu_yaw_rate_deadband_radps_);
  this->declare_parameter("imu_yaw_rate_max_integration_sec", imu_yaw_rate_max_integration_sec_);
  this->declare_parameter("imu_yaw_rate_sigma_per_sqrt_sec", imu_yaw_rate_sigma_per_sqrt_sec_);
  this->declare_parameter("imu_yaw_rate_max_abs_radps", imu_yaw_rate_max_abs_radps_);
  this->declare_parameter("imu_yaw_rate_max_sample_gap_sec", imu_yaw_rate_max_sample_gap_sec_);
  this->declare_parameter("imu_yaw_rate_max_boundary_gap_sec", imu_yaw_rate_max_boundary_gap_sec_);

  this->declare_parameter(
    "fix_confidence_table",
    std::vector<double>{0.0, 0.35, 0.55, 0.65, 1.00, 0.80, 0.25, 0.10, 0.05});
  this->declare_parameter(
    "fix_sigma_xy_table_m",
    std::vector<double>{1000.0, 5.0, 2.5, 1.5, 0.05, 0.20, 10.0, 50.0, 100.0});
  this->declare_parameter("hdop_good_threshold", 1.2);
  this->declare_parameter("hdop_confidence_exponent", 2.0);
  this->declare_parameter("min_hdop_for_sigma", 0.5);
  this->declare_parameter("max_hdop_for_sigma", 20.0);
  this->declare_parameter("vertical_sigma_scale", 2.0);

  this->declare_parameter("dual_antenna_min_baseline_m", 0.3);
  this->declare_parameter("dual_antenna_baseline_tolerance_m", 2.0);
  this->declare_parameter("dual_antenna_heading_sigma_floor_deg", 0.5);
  this->declare_parameter(
    "dual_antenna_heading_sigma_baseline_inflate_scale",
    dual_antenna_heading_sigma_baseline_inflate_scale_);

  this->declare_parameter("sync_tolerance_sec", 0.03);
  this->declare_parameter("buffer_retention_sec", 1.0);
  this->declare_parameter("max_buffer_size", 100);

  this->declare_parameter("publish_global_pose", true);
  this->declare_parameter("publish_global_pose_without_heading", false);
  this->declare_parameter("publish_pose_with_covariance", true);
  this->declare_parameter("publish_gnss_odometry", true);
  this->declare_parameter("publish_gnss_fusion_input", true);
  this->declare_parameter("publish_confidence", true);
  this->declare_parameter("publish_diagnostics", true);

  this->declare_parameter("heartbeat_hz", 2.0);
  this->declare_parameter("flush_timer_hz", 50.0);
  this->declare_parameter("input_stale_timeout_sec", 2.0);

  this->declare_parameter("enable_checksum_validation", true);
  this->declare_parameter("use_ellipsoid_height_from_geoid_separation", true);
  this->declare_parameter(
    "allow_reception_time_for_zero_stamp", allow_reception_time_for_zero_stamp_);

  this->declare_parameter("differential_age_warn_sec", 1.0);
  this->declare_parameter("differential_age_invalid_sec", 2.0);

  this->declare_parameter("gnss_usable_confidence_threshold", 0.3);
}

void NmeaGgaConversion::loadParameters()
{
  this->get_parameter("projector_type", projector_type_);
  this->get_parameter("vertical_datum", vertical_datum_);
  this->get_parameter("map_origin.latitude", map_origin_lat_deg_);
  this->get_parameter("map_origin.longitude", map_origin_lon_deg_);
  this->get_parameter("map_origin.altitude", map_origin_altitude_ignored_);
  this->get_parameter("scale_factor", scale_factor_);
  this->get_parameter("use_legacy_projection_params", use_legacy_projection_params_);

  this->get_parameter("p0", p0_);
  this->get_parameter("gnss0", gnss0_);
  this->get_parameter("a", a_);
  this->get_parameter("F", F_);
  this->get_parameter("m0", m0_);
  this->get_parameter("min_x", min_x_);
  this->get_parameter("min_y", min_y_);
  this->get_parameter("subtract_min_offset", subtract_min_offset_);

  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("child_frame_id", child_frame_id_);

  this->get_parameter("use_secondary_gga", use_secondary_gga_);
  this->get_parameter("use_doppler_heading", use_doppler_heading_);
  this->get_parameter("use_tf_for_antenna_geometry", use_tf_for_antenna_geometry_);
  this->get_parameter(
    "allow_parameter_antenna_fallback", allow_parameter_antenna_fallback_);
  this->get_parameter("primary_antenna_frame_id", primary_antenna_frame_id_);
  this->get_parameter("secondary_antenna_frame_id", secondary_antenna_frame_id_);
  this->get_parameter("primary_antenna_position_base", primary_antenna_position_base_);
  this->get_parameter("secondary_antenna_position_base", secondary_antenna_position_base_);

  double deprecated_initial_heading_deg = 0.0;
  bool deprecated_previous_heading_fallback = false;
  bool deprecated_lever_arm_without_heading = false;
  double deprecated_no_heading_cov_scale = 1.0;
  this->get_parameter("initial_heading_deg", deprecated_initial_heading_deg);
  this->get_parameter(
    "use_previous_heading_fallback", deprecated_previous_heading_fallback);
  this->get_parameter(
    "apply_lever_arm_without_heading", deprecated_lever_arm_without_heading);
  this->get_parameter(
    "no_heading_lever_arm_extra_cov_scale", deprecated_no_heading_cov_scale);
  if (std::fabs(deprecated_initial_heading_deg) > 1.0e-12 ||
    deprecated_previous_heading_fallback || deprecated_lever_arm_without_heading ||
    std::fabs(deprecated_no_heading_cov_scale - 1.0) > 1.0e-12)
  {
    RCLCPP_WARN(
      get_logger(),
      "Deprecated guessed-heading/lever-arm parameters are ignored. Unknown "
      "heading remains heading_valid=false until a physical heading source is available.");
  }
  this->get_parameter("yaw_variance_without_heading", yaw_variance_without_heading_);

  this->get_parameter("use_trajectory_heading", use_trajectory_heading_);
  this->get_parameter("trajectory_heading_min_baseline_m", trajectory_heading_min_baseline_m_);
  this->get_parameter("trajectory_heading_max_age_sec", trajectory_heading_max_age_sec_);
  this->get_parameter(
    "trajectory_heading_max_reference_age_sec", trajectory_heading_max_reference_age_sec_);
  this->get_parameter(
    "trajectory_heading_epoch_max_gap_sec", trajectory_heading_epoch_max_gap_sec_);
  this->get_parameter(
    "trajectory_heading_input_max_gap_sec", trajectory_heading_input_max_gap_sec_);
  this->get_parameter(
    "trajectory_heading_max_turn_activity_rad", trajectory_heading_max_turn_activity_rad_);
  this->get_parameter(
    "trajectory_heading_max_seed_innovation_rad",
    trajectory_heading_max_seed_innovation_rad_);
  this->get_parameter("trajectory_heading_sigma_floor_deg", trajectory_heading_sigma_floor_deg_);
  this->get_parameter("trajectory_heading_min_confidence", trajectory_heading_min_confidence_);
  this->get_parameter(
    "trajectory_heading_max_position_sigma_m", trajectory_heading_max_position_sigma_m_);
  this->get_parameter(
    "trajectory_heading_max_yaw_variance_rad2",
    trajectory_heading_max_yaw_variance_rad2_);

  this->get_parameter("doppler_min_speed_mps", doppler_min_speed_mps_);
  this->get_parameter("doppler_speed_sigma_th", doppler_speed_sigma_th_);
  this->get_parameter("doppler_heading_sigma_deg_th", doppler_heading_sigma_deg_th_);

  this->get_parameter("use_imu_yaw_rate_heading", use_imu_yaw_rate_heading_);
  this->get_parameter("imu_yaw_rate_deadband_radps", imu_yaw_rate_deadband_radps_);
  this->get_parameter("imu_yaw_rate_max_integration_sec", imu_yaw_rate_max_integration_sec_);
  this->get_parameter("imu_yaw_rate_sigma_per_sqrt_sec", imu_yaw_rate_sigma_per_sqrt_sec_);
  this->get_parameter("imu_yaw_rate_max_abs_radps", imu_yaw_rate_max_abs_radps_);
  this->get_parameter("imu_yaw_rate_max_sample_gap_sec", imu_yaw_rate_max_sample_gap_sec_);
  this->get_parameter("imu_yaw_rate_max_boundary_gap_sec", imu_yaw_rate_max_boundary_gap_sec_);

  this->get_parameter("fix_confidence_table", fix_confidence_table_);
  this->get_parameter("fix_sigma_xy_table_m", fix_sigma_xy_table_m_);
  this->get_parameter("hdop_good_threshold", hdop_good_threshold_);
  this->get_parameter("hdop_confidence_exponent", hdop_confidence_exponent_);
  this->get_parameter("min_hdop_for_sigma", min_hdop_for_sigma_);
  this->get_parameter("max_hdop_for_sigma", max_hdop_for_sigma_);
  this->get_parameter("vertical_sigma_scale", vertical_sigma_scale_);

  this->get_parameter("dual_antenna_min_baseline_m", dual_antenna_min_baseline_m_);
  this->get_parameter("dual_antenna_baseline_tolerance_m", dual_antenna_baseline_tolerance_m_);
  this->get_parameter(
    "dual_antenna_heading_sigma_floor_deg", dual_antenna_heading_sigma_floor_deg_);
  this->get_parameter(
    "dual_antenna_heading_sigma_baseline_inflate_scale",
    dual_antenna_heading_sigma_baseline_inflate_scale_);

  this->get_parameter("sync_tolerance_sec", sync_tolerance_sec_);
  this->get_parameter("buffer_retention_sec", buffer_retention_sec_);
  this->get_parameter("max_buffer_size", max_buffer_size_);

  this->get_parameter("publish_global_pose", publish_global_pose_);
  bool deprecated_publish_without_heading = false;
  this->get_parameter(
    "publish_global_pose_without_heading", deprecated_publish_without_heading);
  if (deprecated_publish_without_heading) {
    RCLCPP_WARN(
      get_logger(),
      "publish_global_pose_without_heading is ignored because PoseStamped cannot "
      "represent an invalid quaternion. Use gnss_fusion_input for position-only fixes.");
  }
  this->get_parameter("publish_pose_with_covariance", publish_pose_with_covariance_);
  this->get_parameter("publish_gnss_odometry", publish_gnss_odometry_);
  this->get_parameter("publish_gnss_fusion_input", publish_gnss_fusion_input_);
  this->get_parameter("publish_confidence", publish_confidence_);
  this->get_parameter("publish_diagnostics", publish_diagnostics_);

  this->get_parameter("heartbeat_hz", heartbeat_hz_);
  this->get_parameter("flush_timer_hz", flush_timer_hz_);
  this->get_parameter("input_stale_timeout_sec", input_stale_timeout_sec_);

  this->get_parameter("enable_checksum_validation", enable_checksum_validation_);
  this->get_parameter(
    "use_ellipsoid_height_from_geoid_separation",
    use_ellipsoid_height_from_geoid_separation_);
  this->get_parameter(
    "allow_reception_time_for_zero_stamp", allow_reception_time_for_zero_stamp_);

  this->get_parameter("differential_age_warn_sec", differential_age_warn_sec_);
  this->get_parameter("differential_age_invalid_sec", differential_age_invalid_sec_);

  this->get_parameter("gnss_usable_confidence_threshold", gnss_usable_confidence_threshold_);
}

void NmeaGgaConversion::validateParameters() const
{
  auto requireThree = [](const std::vector<double> & v, const std::string & name) {
      if (v.size() < 3) {
        throw std::invalid_argument("parameter '" + name + "' must have 3 elements");
      }
    };

  const std::string projector_type = toLowerCopy(trim(projector_type_));
  const std::string vertical_datum = toLowerCopy(trim(vertical_datum_));

  if (use_legacy_projection_params_) {
    requireThree(p0_, "p0");
    requireThree(gnss0_, "gnss0");
    if (a_ <= 0.0) {
      throw std::invalid_argument("parameter 'a' must be positive");
    }
    if (F_ <= 0.0) {
      throw std::invalid_argument("parameter 'F' must be positive");
    }
    if (m0_ <= 0.0) {
      throw std::invalid_argument("parameter 'm0' must be positive");
    }
  } else {
    if (projector_type != "transversemercator") {
      throw std::invalid_argument(
        "parameter 'projector_type' must be 'TransverseMercator'");
    }
    if (vertical_datum != "wgs84") {
      throw std::invalid_argument(
        "parameter 'vertical_datum' must be 'WGS84'");
    }
    if (!std::isfinite(map_origin_lat_deg_) || map_origin_lat_deg_ < -90.0 || map_origin_lat_deg_ > 90.0) {
      throw std::invalid_argument(
        "parameter 'map_origin.latitude' must be finite and within [-90, 90]");
    }
    if (!std::isfinite(map_origin_lon_deg_) || map_origin_lon_deg_ < -180.0 || map_origin_lon_deg_ > 180.0) {
      throw std::invalid_argument(
        "parameter 'map_origin.longitude' must be finite and within [-180, 180]");
    }
    if (!std::isfinite(scale_factor_) || scale_factor_ <= 0.0) {
      throw std::invalid_argument("parameter 'scale_factor' must be positive");
    }
  }

  auto requireEmptyOrThree = [](const std::vector<double> & v, const std::string & name) {
      if (!v.empty() && v.size() != 3U) {
        throw std::invalid_argument(
                "parameter '" + name + "' must be empty or contain exactly 3 elements");
      }
      for (const double value : v) {
        if (!std::isfinite(value)) {
          throw std::invalid_argument("parameter '" + name + "' must contain finite values");
        }
      }
    };
  requireEmptyOrThree(primary_antenna_position_base_, "primary_antenna_position_base");
  requireEmptyOrThree(secondary_antenna_position_base_, "secondary_antenna_position_base");

  if (frame_id_.empty()) {
    throw std::invalid_argument("parameter 'frame_id' must not be empty");
  }
  if (child_frame_id_.empty()) {
    throw std::invalid_argument("parameter 'child_frame_id' must not be empty");
  }
  if (sync_tolerance_sec_ < 0.0) {
    throw std::invalid_argument("parameter 'sync_tolerance_sec' must be non-negative");
  }
  if (buffer_retention_sec_ <= 0.0) {
    throw std::invalid_argument("parameter 'buffer_retention_sec' must be positive");
  }
  if (buffer_retention_sec_ < sync_tolerance_sec_) {
    throw std::invalid_argument(
      "parameter 'buffer_retention_sec' must be >= 'sync_tolerance_sec'");
  }
  if (max_buffer_size_ <= 0) {
    throw std::invalid_argument("parameter 'max_buffer_size' must be positive");
  }
  if (heartbeat_hz_ <= 0.0) {
    throw std::invalid_argument("parameter 'heartbeat_hz' must be positive");
  }
  if (flush_timer_hz_ <= 0.0) {
    throw std::invalid_argument("parameter 'flush_timer_hz' must be positive");
  }
  if (input_stale_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("parameter 'input_stale_timeout_sec' must be positive");
  }
  if (fix_confidence_table_.size() < 9U) {
    throw std::invalid_argument("parameter 'fix_confidence_table' must contain at least 9 elements");
  }
  for (const double confidence : fix_confidence_table_) {
    if (!std::isfinite(confidence) || confidence < 0.0 || confidence > 1.0) {
      throw std::invalid_argument(
        "parameter 'fix_confidence_table' values must be finite and within [0, 1]");
    }
  }
  if (fix_sigma_xy_table_m_.size() < 9U) {
    throw std::invalid_argument("parameter 'fix_sigma_xy_table_m' must contain at least 9 elements");
  }
  for (const double sigma : fix_sigma_xy_table_m_) {
    if (!std::isfinite(sigma) || sigma <= 0.0) {
      throw std::invalid_argument(
        "parameter 'fix_sigma_xy_table_m' values must be finite and positive");
    }
  }
  if (hdop_good_threshold_ <= 0.0) {
    throw std::invalid_argument("parameter 'hdop_good_threshold' must be positive");
  }
  if (hdop_confidence_exponent_ <= 0.0) {
    throw std::invalid_argument("parameter 'hdop_confidence_exponent' must be positive");
  }
  if (min_hdop_for_sigma_ <= 0.0 || max_hdop_for_sigma_ <= 0.0) {
    throw std::invalid_argument("HDOP sigma range parameters must be positive");
  }
  if (max_hdop_for_sigma_ < min_hdop_for_sigma_) {
    throw std::invalid_argument(
      "parameter 'max_hdop_for_sigma' must be >= 'min_hdop_for_sigma'");
  }
  if (vertical_sigma_scale_ <= 0.0) {
    throw std::invalid_argument("parameter 'vertical_sigma_scale' must be positive");
  }
  if (dual_antenna_min_baseline_m_ <= 0.0) {
    throw std::invalid_argument("parameter 'dual_antenna_min_baseline_m' must be positive");
  }
  if (dual_antenna_baseline_tolerance_m_ < 0.0) {
    throw std::invalid_argument(
      "parameter 'dual_antenna_baseline_tolerance_m' must be non-negative");
  }
  if (dual_antenna_heading_sigma_floor_deg_ < 0.0) {
    throw std::invalid_argument(
      "parameter 'dual_antenna_heading_sigma_floor_deg' must be non-negative");
  }
  if (dual_antenna_heading_sigma_baseline_inflate_scale_ < 0.0) {
    throw std::invalid_argument(
      "parameter 'dual_antenna_heading_sigma_baseline_inflate_scale' must be non-negative");
  }
  if (yaw_variance_without_heading_ < 0.0) {
    throw std::invalid_argument("parameter 'yaw_variance_without_heading' must be non-negative");
  }
  if (trajectory_heading_min_baseline_m_ <= 0.0) {
    throw std::invalid_argument(
      "parameter 'trajectory_heading_min_baseline_m' must be positive");
  }
  if (trajectory_heading_max_age_sec_ <= 0.0) {
    throw std::invalid_argument(
      "parameter 'trajectory_heading_max_age_sec' must be positive");
  }
  if (!std::isfinite(trajectory_heading_max_reference_age_sec_) ||
    trajectory_heading_max_reference_age_sec_ <= 0.0 ||
    trajectory_heading_max_reference_age_sec_ > trajectory_heading_max_age_sec_)
  {
    throw std::invalid_argument(
      "parameter 'trajectory_heading_max_reference_age_sec' must be finite, positive, "
      "and no greater than 'trajectory_heading_max_age_sec'");
  }
  if (!std::isfinite(trajectory_heading_epoch_max_gap_sec_) ||
    trajectory_heading_epoch_max_gap_sec_ <= 0.0)
  {
    throw std::invalid_argument(
      "parameter 'trajectory_heading_epoch_max_gap_sec' must be finite and positive");
  }
  if (!std::isfinite(trajectory_heading_input_max_gap_sec_) ||
    trajectory_heading_input_max_gap_sec_ <= 0.0)
  {
    throw std::invalid_argument(
      "parameter 'trajectory_heading_input_max_gap_sec' must be finite and positive");
  }
  if (!std::isfinite(trajectory_heading_max_turn_activity_rad_) ||
    trajectory_heading_max_turn_activity_rad_ <= 0.0)
  {
    throw std::invalid_argument(
      "parameter 'trajectory_heading_max_turn_activity_rad' must be finite and positive");
  }
  if (!std::isfinite(trajectory_heading_max_seed_innovation_rad_) ||
    trajectory_heading_max_seed_innovation_rad_ <= 0.0)
  {
    throw std::invalid_argument(
      "parameter 'trajectory_heading_max_seed_innovation_rad' must be finite and positive");
  }
  if (trajectory_heading_sigma_floor_deg_ < 0.0) {
    throw std::invalid_argument(
      "parameter 'trajectory_heading_sigma_floor_deg' must be non-negative");
  }
  if (!std::isfinite(trajectory_heading_min_confidence_) ||
    trajectory_heading_min_confidence_ < 0.0 || trajectory_heading_min_confidence_ > 1.0)
  {
    throw std::invalid_argument(
      "parameter 'trajectory_heading_min_confidence' must be within [0, 1]");
  }
  if (!std::isfinite(trajectory_heading_max_position_sigma_m_) ||
    trajectory_heading_max_position_sigma_m_ <= 0.0)
  {
    throw std::invalid_argument(
      "parameter 'trajectory_heading_max_position_sigma_m' must be finite and positive");
  }
  if (!std::isfinite(trajectory_heading_max_yaw_variance_rad2_) ||
    trajectory_heading_max_yaw_variance_rad2_ <= 0.0)
  {
    throw std::invalid_argument(
      "parameter 'trajectory_heading_max_yaw_variance_rad2' must be finite and positive");
  }
  if (imu_yaw_rate_max_integration_sec_ <= 0.0) {
    throw std::invalid_argument(
      "parameter 'imu_yaw_rate_max_integration_sec' must be positive");
  }
  if (imu_yaw_rate_sigma_per_sqrt_sec_ <= 0.0) {
    throw std::invalid_argument(
      "parameter 'imu_yaw_rate_sigma_per_sqrt_sec' must be positive");
  }
  if (imu_yaw_rate_max_abs_radps_ <= 0.0) {
    throw std::invalid_argument(
      "parameter 'imu_yaw_rate_max_abs_radps' must be positive");
  }
  if (imu_yaw_rate_max_sample_gap_sec_ <= 0.0) {
    throw std::invalid_argument(
      "parameter 'imu_yaw_rate_max_sample_gap_sec' must be positive");
  }
  if (imu_yaw_rate_max_boundary_gap_sec_ < 0.0 ||
    imu_yaw_rate_max_boundary_gap_sec_ > imu_yaw_rate_max_sample_gap_sec_)
  {
    throw std::invalid_argument(
      "parameter 'imu_yaw_rate_max_boundary_gap_sec' must be within [0, max_sample_gap]");
  }
  if (differential_age_warn_sec_ < 0.0) {
    throw std::invalid_argument("parameter 'differential_age_warn_sec' must be non-negative");
  }
  if (differential_age_invalid_sec_ < differential_age_warn_sec_) {
    throw std::invalid_argument(
      "parameter 'differential_age_invalid_sec' must be >= 'differential_age_warn_sec'");
  }
  if (!std::isfinite(gnss_usable_confidence_threshold_) ||
    gnss_usable_confidence_threshold_ < 0.0 || gnss_usable_confidence_threshold_ > 1.0)
  {
    throw std::invalid_argument(
      "parameter 'gnss_usable_confidence_threshold' must be within [0, 1]");
  }
}

void NmeaGgaConversion::initializeProjection()
{
  if (use_legacy_projection_params_) {
    origin_lat_rad_ = deg2rad(gnss0_[0]);
    origin_lon_rad_ = deg2rad(gnss0_[1]);
    offset_z_ = p0_[2] - gnss0_[2];
  } else {
    // Requested coordinate system:
    //   projector_type: TransverseMercator
    //   vertical_datum: WGS84 (ellipsoid height)
    //   map_origin: latitude / longitude only
    //   scale_factor: 0.9996
    // map_origin.altitude is intentionally ignored and treated as 0.0.
    origin_lat_rad_ = deg2rad(map_origin_lat_deg_);
    origin_lon_rad_ = deg2rad(map_origin_lon_deg_);
    a_ = 6378137.0;
    F_ = 298.257223563;
    m0_ = scale_factor_;
    offset_z_ = 0.0;

    if (std::abs(map_origin_altitude_ignored_) > 1.0e-9) {
      RCLCPP_WARN(
        this->get_logger(),
        "map_origin.altitude=%.3f was provided but is ignored. This node keeps Z in WGS84 ellipsoid height.",
        map_origin_altitude_ignored_);
    }
    if (!use_ellipsoid_height_from_geoid_separation_) {
      RCLCPP_WARN(
        this->get_logger(),
        "vertical_datum=WGS84 is configured, but use_ellipsoid_height_from_geoid_separation=false. The published Z will no longer be WGS84 ellipsoid height unless your receiver already outputs ellipsoid height in GGA.");
    }
  }

  const double n = 1.0 / (2.0 * F_ - 1.0);
  const double n2 = std::pow(n, 2);
  const double n3 = std::pow(n, 3);
  const double n4 = std::pow(n, 4);
  const double n5 = std::pow(n, 5);

  alpha_[0] = n / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0 + 41.0 * n4 / 180.0 - 127.0 * n5 / 288.0;
  alpha_[1] = 13.0 * n2 / 48.0 - 3.0 * n3 / 5.0 + 557.0 * n4 / 1440.0 + 281.0 * n5 / 630.0;
  alpha_[2] = 61.0 * n3 / 240.0 - 103.0 * n4 / 140.0 + 15061.0 * n5 / 26880.0;
  alpha_[3] = 49561.0 * n4 / 161280.0 - 179.0 * n5 / 168.0;
  alpha_[4] = 34729.0 * n5 / 80640.0;

  A_[0] = 1.0 + n2 / 4.0 + n4 / 64.0;
  A_[1] = -3.0 / 2.0 * (n - n3 / 8.0 - n5 / 64.0);
  A_[2] = 15.0 / 16.0 * (n2 - n4 / 4.0);
  A_[3] = -35.0 / 48.0 * (n3 - 5.0 * n5 / 16.0);
  A_[4] = 315.0 * n4 / 512.0;
  A_[5] = -693.0 * n5 / 1280.0;

  A_bar_ = m0_ * a_ * A_[0] / (1.0 + n);

  S_bar_phi0_ = A_[0] * origin_lat_rad_;
  for (int i = 1; i <= 5; ++i) {
    S_bar_phi0_ += A_[i] * std::sin(2.0 * i * origin_lat_rad_);
  }
  S_bar_phi0_ *= m0_ * a_ / (1.0 + n);

  kt_ = 2.0 * std::sqrt(n) / (1.0 + n);
  K_.setIdentity();
  R_ = Eigen::Rotation2Dd(-kPi / 2.0);
}

void NmeaGgaConversion::primarySentenceCallback(const nmea_msgs::msg::Sentence::SharedPtr msg)
{
  GgaMeasurement measurement;
  std::string error;
  if (!parseGgaSentence(*msg, measurement, error)) {
    if (error != "not_gga") {
      std::lock_guard<std::mutex> lock(mutex_);
      last_parse_error_ = error;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "primary_gga skipped: %s", error.c_str());
    } else {
      RCLCPP_DEBUG(this->get_logger(), "primary_gga skipped: not_gga");
    }
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_last_primary_ = true;
    last_primary_stamp_ = measurement.stamp;
    last_primary_fix_quality_ = measurement.fix_quality;
    last_primary_hdop_ = measurement.hdop;
    last_primary_differential_age_sec_ = measurement.differential_age_sec;
    last_fix_status_ = measurement.navsat_status;
    last_parse_error_ = "none";
    updateLatestInputStampLocked(measurement.stamp);
    observeTrajectoryHeadingMeasurementLocked(measurement);

    if (!measurement.valid_fix) {
      // No-fix still updates status, but it does not enter the synchronization buffer.
      pruneBuffersLocked();
    } else {
      bufferPrimaryMeasurementLocked(measurement);
      pruneBuffersLocked();
    }
  }

  if (!measurement.valid_fix) {
    publishStatusOnly(measurement.stamp, measurement.navsat_status, 0.0F);
    return;
  }

  onFlushTimer();
}

void NmeaGgaConversion::secondarySentenceCallback(const nmea_msgs::msg::Sentence::SharedPtr msg)
{
  GgaMeasurement measurement;
  std::string error;
  if (!parseGgaSentence(*msg, measurement, error)) {
    if (error != "not_gga") {
      std::lock_guard<std::mutex> lock(mutex_);
      last_parse_error_ = error;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "secondary_gga skipped: %s", error.c_str());
    } else {
      RCLCPP_DEBUG(this->get_logger(), "secondary_gga skipped: not_gga");
    }
    return;
  }

  if (!measurement.valid_fix) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_last_secondary_ = true;
    last_secondary_stamp_ = measurement.stamp;
    last_parse_error_ = "none";
    updateLatestInputStampLocked(measurement.stamp);
    bufferSecondaryMeasurementLocked(measurement);
    pruneBuffersLocked();
  }

  onFlushTimer();
}

void NmeaGgaConversion::dopplerCallback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  const double ve = msg->twist.twist.linear.x;
  const double vn = msg->twist.twist.linear.y;

  if (!std::isfinite(ve) || !std::isfinite(vn)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "fix_velocity contains NaN/Inf. Doppler sample skipped.");
    return;
  }

  DopplerMeasurement doppler;
  doppler.stamp = resolveStamp(msg->header.stamp);
  if (doppler.stamp.nanoseconds() == 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "fix_velocity has a zero timestamp; sample ignored.");
    return;
  }

  const double var_e = sanitizeVariance(msg->twist.covariance[0]);
  const double var_n = sanitizeVariance(msg->twist.covariance[7]);
  const double cov_en = sanitizeCovariance(msg->twist.covariance[1]);

  const double speed_sq = ve * ve + vn * vn;
  const double speed = std::sqrt(std::max(speed_sq, 0.0));
  doppler.speed = speed;

  if (speed_sq < 1.0e-12) {
    doppler.speed_variance = var_e + var_n;
    doppler.heading = 0.0;
    doppler.heading_variance = kUnknownVariance;
    doppler.has_valid_heading = false;
  } else {
    const double speed_variance =
      ((ve * ve) * var_e + 2.0 * ve * vn * cov_en + (vn * vn) * var_n) / speed_sq;
    const double heading_variance =
      ((vn * vn) * var_e - 2.0 * ve * vn * cov_en + (ve * ve) * var_n) /
      (speed_sq * speed_sq);

    doppler.speed_variance = std::max(speed_variance, 0.0);
    doppler.heading = std::atan2(vn, ve);
    doppler.heading_variance = std::max(heading_variance, 0.0);

    const double speed_sigma = std::sqrt(std::max(doppler.speed_variance, 0.0));
    const double heading_sigma_deg =
      std::sqrt(std::max(doppler.heading_variance, 0.0)) * 180.0 / kPi;

    doppler.has_valid_heading =
      speed >= doppler_min_speed_mps_ &&
      speed_sigma <= doppler_speed_sigma_th_ &&
      heading_sigma_deg <= doppler_heading_sigma_deg_th_;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_last_doppler_ = true;
    last_doppler_stamp_ = doppler.stamp;
    last_speed_mps_ = doppler.speed;
    updateLatestInputStampLocked(doppler.stamp);
    bufferDopplerMeasurementLocked(doppler);
    pruneBuffersLocked();
  }

  onFlushTimer();
}

void NmeaGgaConversion::imuCorrectedCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  double gyro_z = msg->angular_velocity.z;

  if (stamp.nanoseconds() == 0 || !std::isfinite(gyro_z) ||
    std::abs(gyro_z) > imu_yaw_rate_max_abs_radps_)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "imu_corrected sample has a zero stamp, non-finite gyro_z, or exceeds "
      "imu_yaw_rate_max_abs_radps; sample ignored.");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (!imu_buf_.empty() && stamp <= imu_buf_.back().stamp) {
    if (stamp == imu_buf_.back().stamp) {
      // Replace an exact duplicate rather than integrating it twice.
      imu_buf_.pop_back();
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Out-of-order imu_corrected sample ignored (dt=%.6f s).",
        (stamp - imu_buf_.back().stamp).seconds());
      return;
    }
  }

  if (is_stopped_ || std::abs(gyro_z) < imu_yaw_rate_deadband_radps_) {
    gyro_z = 0.0;
  }

  imu_buf_.push_back(ImuSample{stamp, gyro_z});

  const double retention =
    imu_yaw_rate_max_integration_sec_ + imu_yaw_rate_max_sample_gap_sec_ + 1.0;
  while (imu_buf_.size() > 1) {
    if ((stamp - imu_buf_.front().stamp).seconds() > retention) {
      imu_buf_.pop_front();
    } else {
      break;
    }
  }
}

void NmeaGgaConversion::stoppedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  is_stopped_ = msg->data;
}

pure_nmea_gga_conversion::ImuYawIntegrationResult
NmeaGgaConversion::integrateImuYawLocked(
  const rclcpp::Time & t_start, const rclcpp::Time & t_end) const
{
  std::vector<pure_nmea_gga_conversion::TimedYawRate> samples;
  samples.reserve(imu_buf_.size());
  for (const auto & sample : imu_buf_) {
    samples.push_back(
      pure_nmea_gga_conversion::TimedYawRate{
        static_cast<double>(sample.stamp.nanoseconds()) * 1.0e-9,
        sample.gyro_z});
  }

  return pure_nmea_gga_conversion::integrateYawRate(
    samples,
    static_cast<double>(t_start.nanoseconds()) * 1.0e-9,
    static_cast<double>(t_end.nanoseconds()) * 1.0e-9,
    imu_yaw_rate_max_sample_gap_sec_,
    imu_yaw_rate_max_boundary_gap_sec_);
}

bool NmeaGgaConversion::parseGgaSentence(
  const nmea_msgs::msg::Sentence & msg, GgaMeasurement & output, std::string & error) const
{
  output = GgaMeasurement{};
  const std::string sentence = trim(msg.sentence);
  if (sentence.empty()) {
    error = "empty_sentence";
    return false;
  }

  // Strip '$' prefix and checksum '*XX' suffix to obtain the payload fields.
  // Sentence type is verified BEFORE checksum validation so that non-GGA
  // sentences (RMC, VTG, GSV, …) are discarded immediately without running
  // the checksum or writing to last_parse_error_.
  std::string payload = sentence;
  if (!payload.empty() && payload.front() == '$') {
    payload.erase(payload.begin());
  }
  const auto star_pos = payload.find('*');
  if (star_pos != std::string::npos) {
    payload = payload.substr(0, star_pos);
  }

  const std::vector<std::string> fields = split(payload, ',');

  // Early exit for non-GGA sentences (cheap check — no checksum work needed).
  if (fields.empty() ||
      fields[0].size() < 3U ||
      fields[0].substr(fields[0].size() - 3U) != "GGA")
  {
    error = "not_gga";
    return false;
  }

  // Confirmed GGA: now validate the checksum.
  if (enable_checksum_validation_ && !validateNmeaChecksum(sentence)) {
    error = "checksum_error";
    return false;
  }

  if (fields.size() < 10U) {
    error = "too_few_fields";
    return false;
  }

  output.stamp = resolveStamp(msg.header.stamp);
  if (output.stamp.nanoseconds() == 0) {
    error = "zero_timestamp";
    return false;
  }
  output.frame_id = msg.header.frame_id;
  output.raw_sentence = sentence;

  int fix_quality = 0;
  (void)parseInt(fields[6], fix_quality);
  fix_quality = static_cast<int>(clamp(static_cast<double>(fix_quality), 0.0, 8.0));
  output.fix_quality = fix_quality;
  output.navsat_status = ggaQualityToNavSatStatus(fix_quality);

  int num_satellites = 0;
  (void)parseInt(fields[7], num_satellites);
  output.num_satellites = num_satellites;

  double hdop = max_hdop_for_sigma_;
  (void)parseDouble(fields[8], hdop);
  hdop = clamp(hdop, min_hdop_for_sigma_, max_hdop_for_sigma_);
  output.hdop = hdop;

  // Parse differential age (GGA field [13]: age of differential data in seconds)
  double differential_age = std::numeric_limits<double>::quiet_NaN();
  if (fields.size() > 13U && !fields[13].empty()) {
    double tmp = 0.0;
    if (parseDouble(fields[13], tmp) && tmp >= 0.0) {
      differential_age = tmp;
    }
  }
  output.differential_age_sec = differential_age;
  output.position_confidence = positionConfidence(output.fix_quality, output.hdop);

  if (fix_quality <= 0) {
    output.valid_fix = false;
    output.sigma_xy_m = horizontalSigmaMeters(output.fix_quality, output.hdop);
    return true;
  }

  double lat_deg = 0.0;
  double lon_deg = 0.0;
  if (!parseLatitude(fields[2], fields[3], lat_deg)) {
    error = "latitude_parse_error";
    return false;
  }
  if (!parseLongitude(fields[4], fields[5], lon_deg)) {
    error = "longitude_parse_error";
    return false;
  }

  double altitude_msl = 0.0;
  if (!parseDouble(fields[9], altitude_msl)) {
    error = "altitude_parse_error";
    return false;
  }

  double geoid_separation = 0.0;
  const bool has_geoid_separation =
    fields.size() > 11U && parseDouble(fields[11], geoid_separation);

  const double altitude =
    (use_ellipsoid_height_from_geoid_separation_ && has_geoid_separation) ?
    (altitude_msl + geoid_separation) :
    altitude_msl;

  double x = 0.0;
  double y = 0.0;
  gaussKruger(deg2rad(lat_deg), deg2rad(lon_deg), x, y);
  if (subtract_min_offset_) {
    x -= min_x_;
    y -= min_y_;
  }

  output.lat_deg = lat_deg;
  output.lon_deg = lon_deg;
  output.alt_m = altitude;
  output.x = x;
  output.y = y;
  output.z = altitude + offset_z_;
  output.sigma_xy_m = horizontalSigmaMeters(output.fix_quality, output.hdop);

  // Apply differential age penalty to both confidence and sigma.
  // Confidence and sigma are both derived from fix_quality+HDOP, so age
  // must be reflected in sigma as well to avoid under-reporting uncertainty.
  const double age_scale = differentialAgeScale(output.differential_age_sec);
  output.position_confidence *= age_scale;
  if (age_scale <= 0.0) {
    // Age is fully stale: inflate sigma to at least GPS-level (fix_quality=1)
    output.sigma_xy_m = std::max(output.sigma_xy_m, horizontalSigmaMeters(1, output.hdop));
  } else if (age_scale < 1.0) {
    // Partial age penalty: inflate sigma inversely, consistent with confidence reduction
    output.sigma_xy_m /= age_scale;
  }

  output.valid_fix = true;
  return true;
}

bool NmeaGgaConversion::validateNmeaChecksum(const std::string & sentence) const
{
  if (sentence.empty() || sentence.front() != '$') {
    return false;
  }

  const auto star_pos = sentence.find('*');
  if (star_pos == std::string::npos || (star_pos + 2U) >= sentence.size()) {
    return false;
  }

  uint8_t checksum = 0U;
  for (std::size_t i = 1; i < star_pos; ++i) {
    checksum ^= static_cast<uint8_t>(sentence[i]);
  }

  std::string checksum_text = sentence.substr(star_pos + 1U, 2U);
  try {
    const unsigned long parsed = std::stoul(checksum_text, nullptr, 16);
    return checksum == static_cast<uint8_t>(parsed & 0xFFU);
  } catch (const std::exception &) {
    return false;
  }
}

bool NmeaGgaConversion::parseLatitude(
  const std::string & raw, const std::string & hemi, double & lat_deg) const
{
  double value = 0.0;
  if (!parseDouble(raw, value)) {
    return false;
  }

  const double abs_value = std::fabs(value);
  const int degrees = static_cast<int>(abs_value / 100.0);
  const double minutes = abs_value - static_cast<double>(degrees) * 100.0;
  if (degrees > 90 || minutes < 0.0 || minutes >= 60.0) {
    return false;
  }
  lat_deg = static_cast<double>(degrees) + minutes / 60.0;

  if (hemi == "S") {
    lat_deg = -lat_deg;
  } else if (hemi != "N") {
    return false;
  }
  return std::isfinite(lat_deg) && std::fabs(lat_deg) <= 90.0;
}

bool NmeaGgaConversion::parseLongitude(
  const std::string & raw, const std::string & hemi, double & lon_deg) const
{
  double value = 0.0;
  if (!parseDouble(raw, value)) {
    return false;
  }

  const double abs_value = std::fabs(value);
  const int degrees = static_cast<int>(abs_value / 100.0);
  const double minutes = abs_value - static_cast<double>(degrees) * 100.0;
  if (degrees > 180 || minutes < 0.0 || minutes >= 60.0) {
    return false;
  }
  lon_deg = static_cast<double>(degrees) + minutes / 60.0;

  if (hemi == "W") {
    lon_deg = -lon_deg;
  } else if (hemi != "E") {
    return false;
  }
  return std::isfinite(lon_deg) && std::fabs(lon_deg) <= 180.0;
}

void NmeaGgaConversion::gaussKruger(double rad_phi, double rad_lambda, double & x, double & y) const
{
  const double t = std::sinh(
    std::atanh(std::sin(rad_phi)) - kt_ * std::atanh(kt_ * std::sin(rad_phi)));
  const double t_bar = std::sqrt(1.0 + std::pow(t, 2));
  const double diff_lambda = rad_lambda - origin_lon_rad_;
  const double lambda_cos = std::cos(diff_lambda);
  const double lambda_sin = std::sin(diff_lambda);
  const double zeta = std::atan2(t, lambda_cos);
  const double eta = std::atanh(lambda_sin / t_bar);

  Eigen::Vector2d p;
  p(0) = zeta;
  p(1) = eta;

  for (int i = 1; i <= 5; ++i) {
    p(0) += alpha_[i - 1] * std::sin(2.0 * i * zeta) * std::cosh(2.0 * i * eta);
    p(1) += alpha_[i - 1] * std::cos(2.0 * i * zeta) * std::sinh(2.0 * i * eta);
  }

  p(0) = p(0) * A_bar_ - S_bar_phi0_;
  p(1) *= A_bar_;
  p = K_ * R_ * p;

  const double map_offset_x = use_legacy_projection_params_ ? p0_[0] : 0.0;
  const double map_offset_y = use_legacy_projection_params_ ? p0_[1] : 0.0;
  x = p(0) + map_offset_x;
  y = -p(1) + map_offset_y;
}

double NmeaGgaConversion::fixScore(int fix_quality) const
{
  const std::size_t idx = static_cast<std::size_t>(
    clamp(static_cast<double>(fix_quality), 0.0, static_cast<double>(fix_confidence_table_.size() - 1U)));
  return clamp(fix_confidence_table_[idx], 0.0, 1.0);
}

double NmeaGgaConversion::hdopScore(double hdop) const
{
  if (!std::isfinite(hdop) || hdop < 0.0) {
    return 0.0;
  }

  const double excess = std::max(0.0, hdop - hdop_good_threshold_);
  if (excess <= 0.0) {
    return 1.0;
  }

  const double normalized = excess / std::max(hdop_good_threshold_, 1.0e-6);
  return 1.0 / (1.0 + std::pow(normalized, hdop_confidence_exponent_));
}

double NmeaGgaConversion::positionConfidence(int fix_quality, double hdop) const
{
  if (fix_quality <= 0) {
    return 0.0;
  }
  return clamp(fixScore(fix_quality) * hdopScore(hdop), 0.0, 1.0);
}

double NmeaGgaConversion::horizontalSigmaMeters(int fix_quality, double hdop) const
{
  const std::size_t idx = static_cast<std::size_t>(
    clamp(static_cast<double>(fix_quality), 0.0, static_cast<double>(fix_sigma_xy_table_m_.size() - 1U)));

  const double base_sigma = std::max(0.0, fix_sigma_xy_table_m_[idx]);
  if (fix_quality <= 0 || !std::isfinite(base_sigma)) {
    return 1.0e3;
  }

  const double hdop_for_sigma = clamp(hdop, min_hdop_for_sigma_, max_hdop_for_sigma_);
  return clamp(base_sigma * hdop_for_sigma, 0.01, 1.0e3);
}

double NmeaGgaConversion::differentialAgeScale(double age_sec) const
{
  // Returns a scale factor in [0, 1] based on the differential correction age.
  // NaN (field absent) means differential is not in use — no penalty applied.
  if (!std::isfinite(age_sec) || age_sec < 0.0) {
    return 1.0;
  }
  if (age_sec <= differential_age_warn_sec_) {
    return 1.0;
  }
  if (age_sec >= differential_age_invalid_sec_) {
    return 0.0;
  }
  // Linear decay between warn and invalid thresholds
  const double range = differential_age_invalid_sec_ - differential_age_warn_sec_;
  return 1.0 - (age_sec - differential_age_warn_sec_) / range;
}

int8_t NmeaGgaConversion::ggaQualityToNavSatStatus(int fix_quality) const
{
  switch (fix_quality) {
    case 0:
      return sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    case 2:
    case 3:
      return sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
    case 4:
    case 5:
      return sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    default:
      return sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  }
}

void NmeaGgaConversion::onFlushTimer()
{
  std::vector<MatchedMeasurement> ready_matches;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ready_matches = collectReadyMatchesLocked(this->now());
  }

  for (const auto & matched : ready_matches) {
    const OutputPose output = buildOutputPose(matched);
    publishOutput(output);
  }
}

void NmeaGgaConversion::onHeartbeat()
{
  if (!publish_diagnostics_) {
    return;
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "publishing GGA-derived GNSS pose";

  bool has_last_primary = false;
  bool has_last_secondary = false;
  bool has_last_doppler = false;
  bool has_last_output = false;
  rclcpp::Time last_primary_stamp(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_secondary_stamp(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_doppler_stamp(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_output_stamp(0, 0, RCL_ROS_TIME);
  int last_primary_fix_quality = 0;
  double last_primary_hdop = std::numeric_limits<double>::quiet_NaN();
  double last_primary_differential_age = std::numeric_limits<double>::quiet_NaN();
  float last_position_confidence = 0.0F;
  double last_output_cov_xy = kUnknownVariance;
  double last_output_cov_yaw = kUnknownVariance;
  int8_t last_fix_status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  double last_speed = 0.0;
  std::string last_heading_source;
  std::string last_note;
  std::string last_parse_error;
  bool last_dual = false;
  std::size_t primary_buffer_size = 0U;
  std::size_t secondary_buffer_size = 0U;
  std::size_t doppler_buffer_size = 0U;
  double last_imu_integration_max_gap = 0.0;
  std::size_t last_imu_integration_sample_count = 0U;
  std::string last_imu_integration_reason;
  double last_trajectory_reference_age = std::numeric_limits<double>::quiet_NaN();
  double last_trajectory_baseline = 0.0;
  double last_trajectory_turn_activity = std::numeric_limits<double>::quiet_NaN();
  double last_trajectory_seed_innovation = std::numeric_limits<double>::quiet_NaN();
  std::size_t trajectory_history_size = 0U;
  std::size_t trajectory_epoch = 0U;
  std::size_t trajectory_epoch_reset_count = 0U;
  std::string last_trajectory_epoch_reset_reason;
  std::string last_trajectory_heading_reject_reason;
  std::string last_trajectory_continuity_reason;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_last_primary = has_last_primary_;
    has_last_secondary = has_last_secondary_;
    has_last_doppler = has_last_doppler_;
    has_last_output = has_last_output_;
    last_primary_stamp = last_primary_stamp_;
    last_secondary_stamp = last_secondary_stamp_;
    last_doppler_stamp = last_doppler_stamp_;
    last_output_stamp = last_output_stamp_;
    last_primary_fix_quality = last_primary_fix_quality_;
    last_primary_hdop = last_primary_hdop_;
    last_primary_differential_age = last_primary_differential_age_sec_;
    last_position_confidence = last_position_confidence_;
    last_output_cov_xy = last_output_cov_xy_;
    last_output_cov_yaw = last_output_cov_yaw_;
    last_fix_status = last_fix_status_;
    last_speed = last_speed_mps_;
    last_heading_source = last_heading_source_;
    last_note = last_note_;
    last_parse_error = last_parse_error_;
    last_dual = last_dual_antenna_used_;
    primary_buffer_size = primary_buffer_.size();
    secondary_buffer_size = secondary_buffer_.size();
    doppler_buffer_size = doppler_buffer_.size();
    last_imu_integration_max_gap = last_imu_integration_max_gap_sec_;
    last_imu_integration_sample_count = last_imu_integration_sample_count_;
    last_imu_integration_reason = last_imu_integration_reason_;
    last_trajectory_reference_age = last_trajectory_reference_age_sec_;
    last_trajectory_baseline = last_trajectory_baseline_m_;
    last_trajectory_turn_activity = last_trajectory_turn_activity_rad_;
    last_trajectory_seed_innovation = last_trajectory_seed_innovation_rad_;
    trajectory_history_size = trajectory_heading_history_.size();
    trajectory_epoch = trajectory_heading_history_.epoch();
    trajectory_epoch_reset_count = trajectory_heading_epoch_reset_count_;
    last_trajectory_epoch_reset_reason = last_trajectory_epoch_reset_reason_;
    last_trajectory_heading_reject_reason = last_trajectory_heading_reject_reason_;
    last_trajectory_continuity_reason = last_trajectory_continuity_reason_;
  }

  const rclcpp::Time nowt = this->now();
  const double primary_age =
    has_last_primary ? std::fabs((nowt - last_primary_stamp).seconds()) : std::numeric_limits<double>::infinity();
  const double output_age =
    has_last_output ? std::fabs((nowt - last_output_stamp).seconds()) : std::numeric_limits<double>::infinity();

  if (!has_last_primary) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "waiting for primary GGA input";
  } else if (primary_age > input_stale_timeout_sec_) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "primary GGA input is stale";
  } else if (has_last_primary && !has_last_output && last_primary_fix_quality > 0) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "received GGA but no pose output yet";
  } else if (has_last_output && output_age > input_stale_timeout_sec_) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "GNSS pose output is stale";
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = nowt;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = level;
  status.name = "localization/nmea_gga_conversion";
  status.message = message;
  status.hardware_id = "none";

  auto add = [&](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(kv);
    };

  add("use_secondary_gga", use_secondary_gga_ ? "true" : "false");
  add("use_doppler_heading", use_doppler_heading_ ? "true" : "false");
  add("projector_type", projector_type_);
  add("vertical_datum", vertical_datum_);
  add("projection_mode", use_legacy_projection_params_ ? "legacy" : "transverse_mercator_wgs84");
  add("map_origin_latitude", toStringWithPrecision(use_legacy_projection_params_ ? gnss0_[0] : map_origin_lat_deg_, 8));
  add("map_origin_longitude", toStringWithPrecision(use_legacy_projection_params_ ? gnss0_[1] : map_origin_lon_deg_, 8));
  add("scale_factor", toStringWithPrecision(use_legacy_projection_params_ ? m0_ : scale_factor_, 7));
  add("has_last_primary", has_last_primary ? "true" : "false");
  add("has_last_secondary", has_last_secondary ? "true" : "false");
  add("has_last_doppler", has_last_doppler ? "true" : "false");
  add("has_last_output", has_last_output ? "true" : "false");
  if (has_last_primary) {
    add("primary_age_sec", toStringWithPrecision(primary_age));
  }
  if (has_last_output) {
    add("output_age_sec", toStringWithPrecision(output_age));
  }
  add("last_primary_fix_quality", std::to_string(last_primary_fix_quality));
  add("last_primary_hdop", std::isfinite(last_primary_hdop) ? toStringWithPrecision(last_primary_hdop) : "nan");
  add("last_primary_differential_age_sec", std::isfinite(last_primary_differential_age) ? toStringWithPrecision(last_primary_differential_age) : "nan");
  add("differential_age_warn_sec", toStringWithPrecision(differential_age_warn_sec_));
  add("differential_age_invalid_sec", toStringWithPrecision(differential_age_invalid_sec_));
  add("last_position_confidence", toStringWithPrecision(last_position_confidence));
  add("last_output_cov_xy", toStringWithPrecision(last_output_cov_xy));
  add("last_output_cov_yaw", toStringWithPrecision(last_output_cov_yaw));
  add("last_fix_status", std::to_string(last_fix_status));
  add("last_speed_mps", toStringWithPrecision(last_speed));
  add("last_heading_source", last_heading_source);
  add("last_note", last_note);
  add("last_parse_error", last_parse_error);
  add("last_dual_antenna_used", last_dual ? "true" : "false");
  add(
    "trajectory_heading.max_position_sigma_m",
    toStringWithPrecision(trajectory_heading_max_position_sigma_m_));
  add(
    "trajectory_heading.max_yaw_variance_rad2",
    toStringWithPrecision(trajectory_heading_max_yaw_variance_rad2_));
  add(
    "trajectory_heading.buffer_max_age_sec",
    toStringWithPrecision(trajectory_heading_max_age_sec_));
  add(
    "trajectory_heading.max_reference_age_sec",
    toStringWithPrecision(trajectory_heading_max_reference_age_sec_));
  add(
    "trajectory_heading.epoch_max_gap_sec",
    toStringWithPrecision(trajectory_heading_epoch_max_gap_sec_));
  add(
    "trajectory_heading.input_max_gap_sec",
    toStringWithPrecision(trajectory_heading_input_max_gap_sec_));
  add(
    "trajectory_heading.max_turn_activity_rad",
    toStringWithPrecision(trajectory_heading_max_turn_activity_rad_));
  add(
    "trajectory_heading.max_seed_innovation_rad",
    toStringWithPrecision(trajectory_heading_max_seed_innovation_rad_));
  add(
    "trajectory_heading.last_reference_age_sec",
    std::isfinite(last_trajectory_reference_age) ?
    toStringWithPrecision(last_trajectory_reference_age) : "nan");
  add(
    "trajectory_heading.last_baseline_m",
    toStringWithPrecision(last_trajectory_baseline));
  add(
    "trajectory_heading.last_turn_activity_rad",
    std::isfinite(last_trajectory_turn_activity) ?
    toStringWithPrecision(last_trajectory_turn_activity) : "nan");
  add(
    "trajectory_heading.last_seed_innovation_rad",
    std::isfinite(last_trajectory_seed_innovation) ?
    toStringWithPrecision(last_trajectory_seed_innovation) : "nan");
  add(
    "trajectory_heading.last_continuity_reason",
    last_trajectory_continuity_reason);
  add("trajectory_heading.history_size", std::to_string(trajectory_history_size));
  add("trajectory_heading.epoch", std::to_string(trajectory_epoch));
  add(
    "trajectory_heading.epoch_reset_count",
    std::to_string(trajectory_epoch_reset_count));
  add(
    "trajectory_heading.last_epoch_reset_reason",
    last_trajectory_epoch_reset_reason);
  add(
    "trajectory_heading.last_reject_reason",
    last_trajectory_heading_reject_reason);
  add("primary_buffer_size", std::to_string(primary_buffer_size));
  add("secondary_buffer_size", std::to_string(secondary_buffer_size));
  add("doppler_buffer_size", std::to_string(doppler_buffer_size));
  add("imu_heading.enabled", use_imu_yaw_rate_heading_ ? "true" : "false");
  add("imu_heading.last_result", last_imu_integration_reason);
  add("imu_heading.last_sample_count", std::to_string(last_imu_integration_sample_count));
  add("imu_heading.last_max_gap_sec", toStringWithPrecision(last_imu_integration_max_gap));

  array.status.push_back(status);
  pub_diag_->publish(array);
}

void NmeaGgaConversion::publishDiagnostics(uint8_t level, const std::string & message)
{
  if (!publish_diagnostics_ || !pub_diag_) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = level;
  status.name = "localization/nmea_gga_conversion";
  status.message = message;
  status.hardware_id = "none";
  array.status.push_back(status);

  pub_diag_->publish(array);
}

void NmeaGgaConversion::publishStatusOnly(
  const rclcpp::Time & stamp, int8_t fix_status, float confidence)
{
  publishConfidence(confidence);

  if (!publish_gnss_fusion_input_ || !pub_gnss_fusion_input_) {
    return;
  }

  pure_gnss_msgs::msg::GnssFusionInput msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  msg.has_odom = false;
  msg.fix_status = fix_status;
  msg.fix_quality = -1;
  msg.has_confidence = true;
  msg.confidence = confidence;
  msg.gnss_usable = false;
  msg.heading_valid = false;
  msg.heading_source = "none";
  msg.position_is_base_link = false;
  msg.observation_point_valid = false;
  pub_gnss_fusion_input_->publish(msg);
}

void NmeaGgaConversion::publishOutput(const OutputPose & output)
{
  const Eigen::Matrix3d observation_covariance =
    0.5 * (output.covariance_xy_yaw + output.covariance_xy_yaw.transpose());

  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_last_output_ = true;
    last_output_stamp_ = output.stamp;
    last_position_confidence_ = output.position_confidence;
    last_output_cov_xy_ =
      0.5 * (observation_covariance(0, 0) + observation_covariance(1, 1));
    last_output_cov_yaw_ = observation_covariance(2, 2);
    last_fix_status_ = output.fix_status;
    last_heading_source_ = output.heading_source;
    last_note_ = output.note;
    last_dual_antenna_used_ = output.dual_antenna_used;
    last_speed_mps_ = output.speed;
  }

  publishConfidence(output.position_confidence);

  Eigen::Quaterniond observation_q(output.T_map_observation.linear());
  if (!output.heading_valid || observation_q.norm() < 1.0e-9 ||
    !observation_q.coeffs().allFinite())
  {
    // Quaternion is only a serialization placeholder when heading_valid=false.
    observation_q = Eigen::Quaterniond::Identity();
  }
  observation_q.normalize();

  auto fillPoseCovariance = [](const Eigen::Matrix3d & covariance, double cov_z,
      std::array<double, 36> & ros_covariance) {
      ros_covariance.fill(0.0);
      ros_covariance[0] = covariance(0, 0);
      ros_covariance[1] = covariance(0, 1);
      ros_covariance[5] = covariance(0, 2);
      ros_covariance[6] = covariance(1, 0);
      ros_covariance[7] = covariance(1, 1);
      ros_covariance[11] = covariance(1, 2);
      ros_covariance[14] = cov_z;
      ros_covariance[21] = kUnknownVariance;
      ros_covariance[28] = kUnknownVariance;
      ros_covariance[30] = covariance(2, 0);
      ros_covariance[31] = covariance(2, 1);
      ros_covariance[35] = covariance(2, 2);
    };

  // The dedicated fusion input always carries the physical observation. For a
  // single antenna this is the antenna point, not a prematurely corrected
  // base_link pose. The fusion node applies the lever arm in its measurement
  // model and can therefore consume position-only GGA fixes safely.
  nav_msgs::msg::Odometry observation_odom;
  observation_odom.header.stamp = output.stamp;
  observation_odom.header.frame_id = frame_id_;
  observation_odom.child_frame_id =
    output.position_is_base_link ? child_frame_id_ : primary_antenna_frame_id_;
  observation_odom.pose.pose.position.x = output.T_map_observation.translation().x();
  observation_odom.pose.pose.position.y = output.T_map_observation.translation().y();
  observation_odom.pose.pose.position.z = output.T_map_observation.translation().z();
  observation_odom.pose.pose.orientation.x = observation_q.x();
  observation_odom.pose.pose.orientation.y = observation_q.y();
  observation_odom.pose.pose.orientation.z = observation_q.z();
  observation_odom.pose.pose.orientation.w = observation_q.w();
  fillPoseCovariance(
    observation_covariance, output.cov_z, observation_odom.pose.covariance);
  observation_odom.twist.twist.linear.x = output.speed;
  observation_odom.twist.covariance.fill(0.0);
  observation_odom.twist.covariance[0] = output.speed_variance;
  observation_odom.twist.covariance[7] = kUnknownVariance;
  observation_odom.twist.covariance[14] = kUnknownVariance;
  observation_odom.twist.covariance[21] = kUnknownVariance;
  observation_odom.twist.covariance[28] = kUnknownVariance;
  observation_odom.twist.covariance[35] = kUnknownVariance;

  if (publish_gnss_fusion_input_ && pub_gnss_fusion_input_) {
    pure_gnss_msgs::msg::GnssFusionInput input;
    input.header = observation_odom.header;
    input.has_odom = true;
    input.odom = observation_odom;
    input.fix_status = output.fix_status;
    input.fix_quality = static_cast<int8_t>(output.primary_fix_quality);
    input.has_confidence = true;
    input.confidence = output.position_confidence;
    input.gnss_usable =
      output.fix_status != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX &&
      output.primary_fix_quality > 0 &&
      output.position_confidence >= static_cast<float>(gnss_usable_confidence_threshold_);
    input.heading_valid = output.heading_valid;
    input.heading_source = output.heading_source;
    input.position_is_base_link = output.position_is_base_link;
    input.observation_point_valid = output.observation_point_valid;
    input.observation_point_in_base.x = output.observation_point_in_base.x();
    input.observation_point_in_base.y = output.observation_point_in_base.y();
    input.observation_point_in_base.z = output.observation_point_in_base.z();
    pub_gnss_fusion_input_->publish(input);
  }

  // Legacy Pose/Odometry topics are explicitly base_link poses. Suppress them
  // until heading and antenna geometry are both available; publishing an
  // antenna coordinate with an identity quaternion as base_link was the main
  // semantic bug in the previous implementation.
  Eigen::Isometry3d T_map_base = output.T_map_observation;
  Eigen::Matrix3d base_covariance = observation_covariance;
  bool base_pose_available = output.position_is_base_link;
  if (!output.position_is_base_link && output.heading_valid &&
    output.observation_point_valid)
  {
    const double yaw = std::atan2(
      output.T_map_observation.linear()(1, 0),
      output.T_map_observation.linear()(0, 0));
    const Eigen::Rotation2Dd rotation(yaw);
    const Eigen::Vector2d base_xy = output.T_map_observation.translation().head<2>() -
      rotation * output.observation_point_in_base.head<2>();
    T_map_base.translation() = Eigen::Vector3d(
      base_xy.x(), base_xy.y(),
      output.T_map_observation.translation().z() - output.observation_point_in_base.z());
    base_covariance = baseCovarianceFromAntennaObservation(
      observation_covariance, output.observation_point_in_base.head<2>(), yaw);
    base_pose_available = true;
  }

  if (!output.heading_valid || !base_pose_available) {
    return;
  }

  Eigen::Quaterniond base_q(T_map_base.linear());
  if (base_q.norm() < 1.0e-9 || !base_q.coeffs().allFinite()) {
    return;
  }
  base_q.normalize();

  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = output.stamp;
  pose_cov.header.frame_id = frame_id_;
  pose_cov.pose.pose.position.x = T_map_base.translation().x();
  pose_cov.pose.pose.position.y = T_map_base.translation().y();
  pose_cov.pose.pose.position.z = T_map_base.translation().z();
  pose_cov.pose.pose.orientation.x = base_q.x();
  pose_cov.pose.pose.orientation.y = base_q.y();
  pose_cov.pose.pose.orientation.z = base_q.z();
  pose_cov.pose.pose.orientation.w = base_q.w();
  fillPoseCovariance(base_covariance, output.cov_z, pose_cov.pose.covariance);

  if (publish_global_pose_ && pub_pose_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = pose_cov.header;
    pose.pose = pose_cov.pose.pose;
    pub_pose_->publish(pose);
  }
  if (publish_pose_with_covariance_ && pub_pose_cov_) {
    pub_pose_cov_->publish(pose_cov);
  }

  if (publish_gnss_odometry_ && pub_odom_) {
    nav_msgs::msg::Odometry odom;
    odom.header = pose_cov.header;
    odom.child_frame_id = child_frame_id_;
    odom.pose = pose_cov.pose;
    odom.twist = observation_odom.twist;
    pub_odom_->publish(odom);
  }
}

void NmeaGgaConversion::publishConfidence(float confidence)
{
  if (!publish_confidence_ || !pub_confidence_) {
    return;
  }

  std_msgs::msg::Float32 msg;
  msg.data = confidence;
  pub_confidence_->publish(msg);
}

void NmeaGgaConversion::updateLatestInputStampLocked(const rclcpp::Time & stamp)
{
  if (!has_latest_input_stamp_ || stamp.nanoseconds() > latest_input_stamp_.nanoseconds()) {
    latest_input_stamp_ = stamp;
    has_latest_input_stamp_ = true;
  }
}

void NmeaGgaConversion::pruneBuffersLocked()
{
  if (!has_latest_input_stamp_) {
    return;
  }

  const int64_t retention_ns = bufferRetentionNanoseconds();
  const int64_t reference_ns = latest_input_stamp_.nanoseconds();

  auto primary_is_stale = [reference_ns, retention_ns](const GgaMeasurement & item) {
      return (reference_ns - item.stamp.nanoseconds()) > retention_ns;
    };
  auto secondary_is_stale = [reference_ns, retention_ns](const GgaMeasurement & item) {
      return (reference_ns - item.stamp.nanoseconds()) > retention_ns;
    };
  auto doppler_is_stale = [reference_ns, retention_ns](const DopplerMeasurement & item) {
      return (reference_ns - item.stamp.nanoseconds()) > retention_ns;
    };

  primary_buffer_.erase(
    std::remove_if(primary_buffer_.begin(), primary_buffer_.end(), primary_is_stale),
    primary_buffer_.end());
  secondary_buffer_.erase(
    std::remove_if(secondary_buffer_.begin(), secondary_buffer_.end(), secondary_is_stale),
    secondary_buffer_.end());
  doppler_buffer_.erase(
    std::remove_if(doppler_buffer_.begin(), doppler_buffer_.end(), doppler_is_stale),
    doppler_buffer_.end());
}

void NmeaGgaConversion::bufferPrimaryMeasurementLocked(const GgaMeasurement & primary)
{
  const int64_t stamp_ns = primary.stamp.nanoseconds();
  const auto insert_it = std::upper_bound(
    primary_buffer_.begin(), primary_buffer_.end(), stamp_ns,
    [](int64_t value, const GgaMeasurement & item) {
      return value < item.stamp.nanoseconds();
    });
  primary_buffer_.insert(insert_it, primary);
  while (primary_buffer_.size() > static_cast<std::size_t>(max_buffer_size_)) {
    primary_buffer_.pop_front();
  }
}

void NmeaGgaConversion::bufferSecondaryMeasurementLocked(const GgaMeasurement & secondary)
{
  const int64_t stamp_ns = secondary.stamp.nanoseconds();
  const auto insert_it = std::upper_bound(
    secondary_buffer_.begin(), secondary_buffer_.end(), stamp_ns,
    [](int64_t value, const GgaMeasurement & item) {
      return value < item.stamp.nanoseconds();
    });
  secondary_buffer_.insert(insert_it, secondary);
  while (secondary_buffer_.size() > static_cast<std::size_t>(max_buffer_size_)) {
    secondary_buffer_.pop_front();
  }
}

void NmeaGgaConversion::bufferDopplerMeasurementLocked(const DopplerMeasurement & doppler)
{
  const int64_t stamp_ns = doppler.stamp.nanoseconds();
  const auto insert_it = std::upper_bound(
    doppler_buffer_.begin(), doppler_buffer_.end(), stamp_ns,
    [](int64_t value, const DopplerMeasurement & item) {
      return value < item.stamp.nanoseconds();
    });
  doppler_buffer_.insert(insert_it, doppler);
  while (doppler_buffer_.size() > static_cast<std::size_t>(max_buffer_size_)) {
    doppler_buffer_.pop_front();
  }
}

void NmeaGgaConversion::observeTrajectoryHeadingMeasurementLocked(
  const GgaMeasurement & measurement)
{
  if (!use_trajectory_heading_) {
    return;
  }

  const bool usable = measurement.valid_fix &&
    pure_nmea_gga_conversion::trajectoryHeadingPointIsUsable(
    measurement.position_confidence,
    measurement.sigma_xy_m,
    trajectory_heading_min_confidence_,
    trajectory_heading_max_position_sigma_m_);
  const pure_nmea_gga_conversion::TrajectoryHeadingSample sample{
    static_cast<double>(measurement.stamp.nanoseconds()) * 1.0e-9,
    measurement.x,
    measurement.y,
    measurement.sigma_xy_m,
    measurement.position_confidence};
  const auto observation = trajectory_heading_history_.observe(
    sample, usable, !measurement.valid_fix);

  applyTrajectoryHeadingEpochResetLocked(observation);
  if (!observation.inserted) {
    last_trajectory_heading_reject_reason_ = observation.reject_reason;
  }
}

void NmeaGgaConversion::applyTrajectoryHeadingEpochResetLocked(
  const pure_nmea_gga_conversion::TrajectoryHeadingObservationResult & reset)
{
  if (!reset.epoch_reset) {
    return;
  }

  ++trajectory_heading_epoch_reset_count_;
  last_trajectory_epoch_reset_reason_ = reset.epoch_reset_reason;

  if (has_last_valid_heading_ && last_valid_heading_seed_is_trajectory_) {
    if (reset.preserve_heading_seed) {
      // Geometry discontinuities invalidate the chord segment, not the
      // previously trusted absolute seed. Bind that seed to the new segment
      // so corrected-IMU propagation remains available while a fresh baseline
      // is accumulated.
      last_valid_heading_trajectory_epoch_ = trajectory_heading_history_.epoch();
    } else {
      // A trajectory-derived absolute seed belongs to the trajectory epoch that
      // produced it. It must not be propagated through a GNSS outage as though
      // it were a fresh absolute observation. Independent dual-antenna and
      // Doppler seeds remain valid and retain their normal age bound.
      has_last_valid_heading_ = false;
      last_valid_heading_seed_is_trajectory_ = false;
      last_imu_integration_reason_ =
        "trajectory_seed_invalidated_epoch_reset:" + reset.epoch_reset_reason;
    }
  }
  if (!reset.preserve_heading_seed) {
    last_trajectory_continuity_reason_ = "epoch_reset:" + reset.epoch_reset_reason;
    last_trajectory_seed_innovation_rad_ =
      std::numeric_limits<double>::quiet_NaN();
  }
}

bool NmeaGgaConversion::findClosestSecondaryMeasurementLocked(
  const rclcpp::Time & stamp, std::size_t & index, int64_t & delta_ns) const
{
  if (secondary_buffer_.empty()) {
    return false;
  }

  const int64_t target_ns = stamp.nanoseconds();
  const int64_t tolerance_ns = syncToleranceNanoseconds();
  bool found = false;
  int64_t best_dt = std::numeric_limits<int64_t>::max();
  std::size_t best_index = 0U;

  for (std::size_t i = 0U; i < secondary_buffer_.size(); ++i) {
    const int64_t dt = std::llabs(secondary_buffer_[i].stamp.nanoseconds() - target_ns);
    if (dt <= tolerance_ns && dt < best_dt) {
      best_dt = dt;
      best_index = i;
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  index = best_index;
  delta_ns = best_dt;
  return true;
}

bool NmeaGgaConversion::findClosestDopplerMeasurementLocked(
  const rclcpp::Time & stamp, std::size_t & index, int64_t & delta_ns) const
{
  if (doppler_buffer_.empty()) {
    return false;
  }

  const int64_t target_ns = stamp.nanoseconds();
  const int64_t tolerance_ns = syncToleranceNanoseconds();
  bool found = false;
  int64_t best_dt = std::numeric_limits<int64_t>::max();
  std::size_t best_index = 0U;

  for (std::size_t i = 0U; i < doppler_buffer_.size(); ++i) {
    const int64_t dt = std::llabs(doppler_buffer_[i].stamp.nanoseconds() - target_ns);
    if (dt <= tolerance_ns && dt < best_dt) {
      best_dt = dt;
      best_index = i;
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  index = best_index;
  delta_ns = best_dt;
  return true;
}

std::vector<NmeaGgaConversion::MatchedMeasurement>
NmeaGgaConversion::collectReadyMatchesLocked(const rclcpp::Time & nowt)
{
  std::vector<MatchedMeasurement> ready;
  if (primary_buffer_.empty()) {
    return ready;
  }

  const int64_t wait_ns =
    (use_secondary_gga_ || use_doppler_heading_) ? syncToleranceNanoseconds() : 0;
  const int64_t now_ns = nowt.nanoseconds();
  const int64_t ref_ns = has_latest_input_stamp_ ? latest_input_stamp_.nanoseconds() : now_ns;

  while (!primary_buffer_.empty()) {
    const int64_t primary_ns = primary_buffer_.front().stamp.nanoseconds();
    const bool ready_by_latest = (ref_ns - primary_ns) >= wait_ns;
    const bool ready_by_now = (now_ns - primary_ns) >= wait_ns;

    if (!ready_by_latest && !ready_by_now) {
      break;
    }

    MatchedMeasurement matched;
    matched.primary = primary_buffer_.front();
    primary_buffer_.pop_front();

    if (use_secondary_gga_) {
      std::size_t index = 0U;
      int64_t delta_ns = 0;
      if (findClosestSecondaryMeasurementLocked(matched.primary.stamp, index, delta_ns)) {
        matched.has_secondary = true;
        matched.secondary = secondary_buffer_[index];
        secondary_buffer_.erase(secondary_buffer_.begin() + static_cast<std::ptrdiff_t>(index));
      }
    }

    if (use_doppler_heading_) {
      std::size_t index = 0U;
      int64_t delta_ns = 0;
      if (findClosestDopplerMeasurementLocked(matched.primary.stamp, index, delta_ns)) {
        matched.has_doppler = true;
        matched.doppler = doppler_buffer_[index];
        doppler_buffer_.erase(doppler_buffer_.begin() + static_cast<std::ptrdiff_t>(index));
      }
    }

    ready.push_back(matched);
  }

  return ready;
}

Eigen::Matrix3d NmeaGgaConversion::baseCovarianceFromAntennaObservation(
  const Eigen::Matrix3d & observation_covariance,
  const Eigen::Vector2d & antenna_offset_base,
  double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double rx = antenna_offset_base.x();
  const double ry = antenna_offset_base.y();

  // p_base = p_antenna - R(yaw) * r_base_to_antenna.
  // The full Jacobian preserves position/yaw cross-covariance instead of only
  // inflating the diagonal terms.
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  J(0, 2) = s * rx + c * ry;
  J(1, 2) = -c * rx + s * ry;
  Eigen::Matrix3d covariance = J * observation_covariance * J.transpose();
  return 0.5 * (covariance + covariance.transpose());
}

void NmeaGgaConversion::setSingleAntennaObservation(
  OutputPose & output,
  const GgaMeasurement & primary,
  const Eigen::Vector3d & antenna_offset_base,
  bool has_antenna_geometry,
  double yaw,
  double yaw_variance,
  const std::string & heading_source,
  const std::string & note) const
{
  // Keep the raw GGA coordinate as the antenna observation. Lever-arm
  // correction belongs in the observation model; converting it here would
  // hide the dependence on yaw and can underestimate covariance.
  output.T_map_observation = Eigen::Isometry3d::Identity();
  output.T_map_observation.linear() = quatFromYaw(yaw).toRotationMatrix();
  output.T_map_observation.translation() = Eigen::Vector3d(primary.x, primary.y, primary.z);
  output.covariance_xy_yaw(2, 2) = std::max(0.0, yaw_variance);
  output.heading_valid = true;
  output.heading_source = heading_source;
  output.note = note;
  output.position_is_base_link = false;
  output.observation_point_valid = has_antenna_geometry;
  output.observation_point_in_base = antenna_offset_base;
}

NmeaGgaConversion::OutputPose NmeaGgaConversion::buildOutputPose(
  const MatchedMeasurement & matched)
{
  OutputPose output;
  output.stamp = matched.primary.stamp;
  output.fix_status = matched.primary.navsat_status;
  output.primary_fix_quality = matched.primary.fix_quality;
  output.primary_hdop = matched.primary.hdop;
  output.secondary_fix_quality = matched.has_secondary ? matched.secondary.fix_quality : 0;
  output.secondary_hdop =
    matched.has_secondary ? matched.secondary.hdop : std::numeric_limits<double>::quiet_NaN();
  output.position_confidence = static_cast<float>(matched.primary.position_confidence);
  output.speed = matched.has_doppler ? matched.doppler.speed : 0.0;
  output.speed_variance =
    matched.has_doppler ? matched.doppler.speed_variance : kUnknownVariance;

  const double sigma_xy_primary = matched.primary.sigma_xy_m;
  output.covariance_xy_yaw = Eigen::Matrix3d::Zero();
  output.covariance_xy_yaw(0, 0) = sigma_xy_primary * sigma_xy_primary;
  output.covariance_xy_yaw(1, 1) = sigma_xy_primary * sigma_xy_primary;
  output.covariance_xy_yaw(2, 2) = yaw_variance_without_heading_;
  output.cov_z = std::pow(vertical_sigma_scale_ * sigma_xy_primary, 2);

  Eigen::Vector3d primary_position_base = Eigen::Vector3d::Zero();
  std::string primary_geom_status;
  const bool has_primary_geometry = resolveAntennaPosition(
    primary_antenna_frame_id_, primary_antenna_position_base_,
    primary_position_base, primary_geom_status);
  output.position_is_base_link = false;
  output.observation_point_valid = has_primary_geometry;
  output.observation_point_in_base = primary_position_base;

  bool pose_built = false;

  // 1) Dual antenna: direct base heading and position when both antenna
  // geometries and the measured baseline pass consistency checks.
  if (matched.has_secondary) {
    Eigen::Vector3d secondary_position_base = Eigen::Vector3d::Zero();
    std::string secondary_geom_status;
    const bool has_secondary_geometry = resolveAntennaPosition(
      secondary_antenna_frame_id_, secondary_antenna_position_base_,
      secondary_position_base, secondary_geom_status);

    if (has_primary_geometry && has_secondary_geometry) {
      const Eigen::Vector2d v_base =
        secondary_position_base.head<2>() - primary_position_base.head<2>();
      const Eigen::Vector2d v_map(
        matched.secondary.x - matched.primary.x,
        matched.secondary.y - matched.primary.y);
      const double base_baseline = v_base.norm();
      const double map_baseline = v_map.norm();
      const double baseline_error = std::fabs(map_baseline - base_baseline);

      if (base_baseline >= dual_antenna_min_baseline_m_ &&
        map_baseline >= dual_antenna_min_baseline_m_ &&
        baseline_error <= dual_antenna_baseline_tolerance_m_)
      {
        const double yaw = normalizeAngle(
          std::atan2(v_map.y(), v_map.x()) - std::atan2(v_base.y(), v_base.x()));
        const Eigen::Rotation2Dd rotation(yaw);

        const double sigma_xy_secondary = matched.secondary.sigma_xy_m;
        const double var1 = sigma_xy_primary * sigma_xy_primary;
        const double var2 = sigma_xy_secondary * sigma_xy_secondary;
        const double w1 = (var1 > 1.0e-12) ? 1.0 / var1 : 1.0e12;
        const double w2 = (var2 > 1.0e-12) ? 1.0 / var2 : 1.0e12;
        const double w_total = w1 + w2;

        const Eigen::Vector2d pos1_corrected =
          Eigen::Vector2d(matched.primary.x, matched.primary.y) -
          rotation * primary_position_base.head<2>();
        const Eigen::Vector2d pos2_corrected =
          Eigen::Vector2d(matched.secondary.x, matched.secondary.y) -
          rotation * secondary_position_base.head<2>();
        const Eigen::Vector2d base_origin_xy =
          (w1 * pos1_corrected + w2 * pos2_corrected) / w_total;
        const double base_origin_z =
          (w1 * (matched.primary.z - primary_position_base.z()) +
          w2 * (matched.secondary.z - secondary_position_base.z())) / w_total;

        const double fused_sigma_xy = 1.0 / std::sqrt(w_total);
        const double heading_sigma_floor_rad =
          deg2rad(dual_antenna_heading_sigma_floor_deg_);
        const double heading_sigma_base = std::max(
          heading_sigma_floor_rad,
          std::sqrt(var1 + var2) /
          std::max(base_baseline, dual_antenna_min_baseline_m_));
        const double baseline_error_ratio =
          baseline_error / std::max(base_baseline, 1.0e-6);
        const double heading_sigma_rad =
          heading_sigma_base *
          (1.0 + dual_antenna_heading_sigma_baseline_inflate_scale_ *
          baseline_error_ratio);
        const double yaw_variance = heading_sigma_rad * heading_sigma_rad;

        output.T_map_observation = Eigen::Isometry3d::Identity();
        output.T_map_observation.linear() = quatFromYaw(yaw).toRotationMatrix();
        output.T_map_observation.translation() =
          Eigen::Vector3d(base_origin_xy.x(), base_origin_xy.y(), base_origin_z);
        output.covariance_xy_yaw = Eigen::Matrix3d::Zero();
        output.covariance_xy_yaw(0, 0) = fused_sigma_xy * fused_sigma_xy;
        output.covariance_xy_yaw(1, 1) = fused_sigma_xy * fused_sigma_xy;
        output.covariance_xy_yaw(2, 2) = yaw_variance;
        const Eigen::Vector2d effective_offset =
          (w1 * primary_position_base.head<2>() +
          w2 * secondary_position_base.head<2>()) / w_total;
        output.covariance_xy_yaw = baseCovarianceFromAntennaObservation(
          output.covariance_xy_yaw, effective_offset, yaw);
        output.cov_z = std::pow(vertical_sigma_scale_ * fused_sigma_xy, 2);
        output.heading_valid = true;
        output.heading_source = "dual_antenna";
        output.note = "dual_antenna";
        output.dual_antenna_used = true;
        output.position_is_base_link = true;
        output.observation_point_valid = false;

        const double baseline_score =
          dual_antenna_baseline_tolerance_m_ > 1.0e-6 ?
          clamp(1.0 - baseline_error / dual_antenna_baseline_tolerance_m_, 0.0, 1.0) :
          1.0;
        const double conf_weighted =
          (w1 * std::max(0.0, matched.primary.position_confidence) +
          w2 * std::max(0.0, matched.secondary.position_confidence)) / w_total;
        output.position_confidence = static_cast<float>(
          clamp(conf_weighted * baseline_score, 0.0, 1.0));

        {
          std::lock_guard<std::mutex> lock(mutex_);
          has_last_valid_heading_ = true;
          last_valid_heading_rad_ = yaw;
          last_valid_heading_stamp_ = matched.primary.stamp;
          last_valid_heading_cov_ = yaw_variance;
          last_valid_heading_seed_is_trajectory_ = false;
        }
        pose_built = true;
      } else {
        output.note = "dual_baseline_rejected";
      }
    } else {
      output.note = "dual_geometry_unavailable";
    }
  }

  // 2) Doppler course-over-ground. It is an independent heading source only
  // above the configured speed and uncertainty gates.
  if (!pose_built && matched.has_doppler && matched.doppler.has_valid_heading) {
    const double yaw = normalizeAngle(matched.doppler.heading);
    const double yaw_variance = std::max(0.0, matched.doppler.heading_variance);
    setSingleAntennaObservation(
      output, matched.primary, primary_position_base, has_primary_geometry,
      yaw, yaw_variance, "doppler",
      has_primary_geometry ? "doppler_with_lever_arm" : "doppler_antenna_reference");

    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_last_valid_heading_ = true;
      last_valid_heading_rad_ = yaw;
      last_valid_heading_stamp_ = matched.primary.stamp;
      last_valid_heading_cov_ = yaw_variance;
      last_valid_heading_seed_is_trajectory_ = false;
    }
    pose_built = true;
  }

  // The primary callback keeps a quality-gated trajectory history even while
  // another heading source is active. Only samples from the current trusted
  // epoch can be selected here; the longer history-retention window does not
  // make an old point eligible as a current-tangent reference.
  const bool trajectory_point_is_usable =
    use_trajectory_heading_ &&
    pure_nmea_gga_conversion::trajectoryHeadingPointIsUsable(
      matched.primary.position_confidence,
      matched.primary.sigma_xy_m,
      trajectory_heading_min_confidence_,
      trajectory_heading_max_position_sigma_m_);
  // 3) Single-antenna trajectory heading. The GNSS chord is corrected to an
  // approximate tangent using half of the corrected IMU delta-yaw when the IMU
  // interval is fully covered. High turn activity rejects the chord because a
  // finite chord is then not a reliable current vehicle tangent. Every new
  // trajectory candidate is also compared with the trusted heading seed
  // propagated to the candidate stamp. A geometry rejection starts a new
  // chord segment at the current fix while preserving that seed for bounded
  // IMU propagation.
  bool trajectory_heading_variance_rejected = false;
  std::string trajectory_heading_reject_reason{"none"};
  if (!pose_built && trajectory_point_is_usable) {
    const pure_nmea_gga_conversion::TrajectoryHeadingSample current_point{
      static_cast<double>(matched.primary.stamp.nanoseconds()) * 1.0e-9,
      matched.primary.x,
      matched.primary.y,
      matched.primary.sigma_xy_m,
      matched.primary.position_confidence};
    pure_nmea_gga_conversion::TrajectoryHeadingReferenceResult reference;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      reference = trajectory_heading_history_.selectReference(
        current_point,
        trajectory_heading_min_baseline_m_,
        trajectory_heading_max_reference_age_sec_);
      last_trajectory_reference_age_sec_ = reference.reference_age_sec;
      last_trajectory_baseline_m_ = reference.baseline_m;
      last_trajectory_turn_activity_rad_ = std::numeric_limits<double>::quiet_NaN();
      last_trajectory_heading_reject_reason_ = reference.valid ? "none" : reference.reason;
      if (!reference.valid) {
        last_trajectory_continuity_reason_ = "not_evaluated_" + reference.reason;
        last_trajectory_seed_innovation_rad_ =
          std::numeric_limits<double>::quiet_NaN();
      }
    }
    trajectory_heading_reject_reason = reference.valid ? "none" : reference.reason;

    if (reference.valid) {
      const double dx = matched.primary.x - reference.reference.x;
      const double dy = matched.primary.y - reference.reference.y;
      const double baseline = reference.baseline_m;
      const double chord_yaw = normalizeAngle(std::atan2(dy, dx));
      double yaw_variance =
        pure_nmea_gga_conversion::trajectoryHeadingVariance(
        reference.reference.sigma_xy_m,
        matched.primary.sigma_xy_m,
        baseline,
        deg2rad(trajectory_heading_sigma_floor_deg_));
      double heading_yaw = chord_yaw;
      std::string source = "trajectory";
      std::string note = "trajectory_gnss_only";
      bool trajectory_candidate_rejected = false;

      auto restart_trajectory_segment = [&](const std::string & reason) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto reset = trajectory_heading_history_.restartSegmentAt(
          current_point, reason);
        applyTrajectoryHeadingEpochResetLocked(reset);
        last_trajectory_heading_reject_reason_ = reason;
      };

      if (use_imu_yaw_rate_heading_) {
        pure_nmea_gga_conversion::ImuYawIntegrationResult imu_result;
        const rclcpp::Time reference_stamp(
          static_cast<int64_t>(std::llround(reference.reference.stamp_sec * 1.0e9)),
          RCL_ROS_TIME);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          imu_result = integrateImuYawLocked(reference_stamp, matched.primary.stamp);
          last_imu_integration_max_gap_sec_ = imu_result.max_sample_gap_sec;
          last_imu_integration_sample_count_ = imu_result.used_sample_count;
          last_imu_integration_reason_ = "trajectory:" + imu_result.reason;
          last_trajectory_turn_activity_rad_ = imu_result.valid ?
            imu_result.absolute_yaw_activity_rad :
            std::numeric_limits<double>::quiet_NaN();
        }
        if (imu_result.valid &&
          !pure_nmea_gga_conversion::trajectoryHeadingTurnActivityIsUsable(
            imu_result.absolute_yaw_activity_rad,
            trajectory_heading_max_turn_activity_rad_))
        {
          trajectory_candidate_rejected = true;
          trajectory_heading_reject_reason = "turn_activity_exceeded";
          restart_trajectory_segment(trajectory_heading_reject_reason);
          std::lock_guard<std::mutex> lock(mutex_);
          last_trajectory_continuity_reason_ = "not_evaluated_turn_activity_exceeded";
          last_trajectory_seed_innovation_rad_ =
            std::numeric_limits<double>::quiet_NaN();
        } else if (imu_result.valid) {
          heading_yaw = normalizeAngle(chord_yaw + 0.5 * imu_result.delta_yaw_rad);
          const double imu_variance =
            imu_yaw_rate_sigma_per_sqrt_sec_ * imu_yaw_rate_sigma_per_sqrt_sec_ *
            imu_result.duration_sec;
          yaw_variance += 0.25 * imu_variance;
          source = "trajectory_imu_corrected";
          note = "trajectory_with_corrected_imu";
        } else {
          // With IMU correction enabled, accepting the uncorrected GNSS chord
          // here would bypass the turn-activity gate. A course chord is not a
          // guaranteed current vehicle tangent on a curved path. GNSS-only
          // trajectory heading remains an explicit mode when IMU yaw-rate
          // heading is disabled.
          trajectory_candidate_rejected = true;
          trajectory_heading_reject_reason = "imu_turn_gate_unavailable";
          std::lock_guard<std::mutex> lock(mutex_);
          last_trajectory_heading_reject_reason_ = trajectory_heading_reject_reason;
          last_trajectory_continuity_reason_ = "not_evaluated_imu_unavailable";
          last_trajectory_seed_innovation_rad_ =
            std::numeric_limits<double>::quiet_NaN();
        }
      }

      const bool trajectory_variance_usable =
        pure_nmea_gga_conversion::trajectoryHeadingVarianceIsUsable(
        yaw_variance, trajectory_heading_max_yaw_variance_rad2_);

      if (!trajectory_candidate_rejected && trajectory_variance_usable &&
        use_imu_yaw_rate_heading_)
      {
        bool has_trusted_seed = false;
        double trusted_seed_yaw = 0.0;
        double trusted_seed_age_sec = std::numeric_limits<double>::quiet_NaN();
        rclcpp::Time trusted_seed_stamp(0, 0, RCL_ROS_TIME);
        pure_nmea_gga_conversion::ImuYawIntegrationResult seed_imu_result;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          const bool trajectory_seed_epoch_valid =
            !last_valid_heading_seed_is_trajectory_ ||
            last_valid_heading_trajectory_epoch_ == trajectory_heading_history_.epoch();
          has_trusted_seed = has_last_valid_heading_ && trajectory_seed_epoch_valid;
          if (has_trusted_seed) {
            trusted_seed_yaw = last_valid_heading_rad_;
            trusted_seed_stamp = last_valid_heading_stamp_;
            trusted_seed_age_sec =
              (matched.primary.stamp - trusted_seed_stamp).seconds();
            if (std::isfinite(trusted_seed_age_sec) && trusted_seed_age_sec >= 0.0 &&
              trusted_seed_age_sec <= imu_yaw_rate_max_integration_sec_)
            {
              seed_imu_result = integrateImuYawLocked(
                trusted_seed_stamp, matched.primary.stamp);
            }
          }
        }

        if (has_trusted_seed) {
          const auto seed_gate =
            pure_nmea_gga_conversion::evaluateTrajectoryHeadingSeedGate(
            trusted_seed_age_sec,
            imu_yaw_rate_max_integration_sec_,
            seed_imu_result.valid);
          if (seed_gate.reject_candidate) {
            trajectory_candidate_rejected = true;
            trajectory_heading_reject_reason = seed_gate.reason;
            std::lock_guard<std::mutex> lock(mutex_);
            last_trajectory_heading_reject_reason_ = trajectory_heading_reject_reason;
            last_trajectory_continuity_reason_ = trajectory_heading_reject_reason;
            last_trajectory_seed_innovation_rad_ =
              std::numeric_limits<double>::quiet_NaN();
          } else if (seed_gate.allow_fresh_bootstrap) {
            // The old seed cannot constrain heading indefinitely. This
            // candidate has already formed a fresh quality-contiguous
            // baseline and passed the turn/variance gates, so it may replace
            // an explicitly expired seed. An in-window IMU coverage failure
            // takes the reject branch above instead.
            std::lock_guard<std::mutex> lock(mutex_);
            last_trajectory_continuity_reason_ = seed_gate.reason;
            last_trajectory_seed_innovation_rad_ =
              std::numeric_limits<double>::quiet_NaN();
          } else if (seed_gate.compare_candidate) {
            const double propagated_seed_yaw = normalizeAngle(
              trusted_seed_yaw + seed_imu_result.delta_yaw_rad);
            const auto continuity =
              pure_nmea_gga_conversion::evaluateTrajectoryHeadingContinuity(
              heading_yaw,
              propagated_seed_yaw,
              trajectory_heading_max_seed_innovation_rad_);
            {
              std::lock_guard<std::mutex> lock(mutex_);
              last_trajectory_continuity_reason_ = continuity.reason;
              last_trajectory_seed_innovation_rad_ = continuity.innovation_rad;
            }
            if (!continuity.accept_candidate) {
              trajectory_candidate_rejected = true;
              trajectory_heading_reject_reason = continuity.reason;
              if (continuity.restart_segment) {
                restart_trajectory_segment(continuity.reason);
              } else {
                std::lock_guard<std::mutex> lock(mutex_);
                last_trajectory_heading_reject_reason_ = continuity.reason;
              }
            }
          }
        } else {
          std::lock_guard<std::mutex> lock(mutex_);
          last_trajectory_continuity_reason_ = "no_trusted_seed";
          last_trajectory_seed_innovation_rad_ =
            std::numeric_limits<double>::quiet_NaN();
        }
      }

      if (!trajectory_candidate_rejected && trajectory_variance_usable)
      {
        setSingleAntennaObservation(
          output, matched.primary, primary_position_base, has_primary_geometry,
          heading_yaw, yaw_variance, source, note);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          has_last_valid_heading_ = true;
          last_valid_heading_rad_ = heading_yaw;
          last_valid_heading_stamp_ = matched.primary.stamp;
          last_valid_heading_cov_ = yaw_variance;
          last_valid_heading_seed_is_trajectory_ = true;
          last_valid_heading_trajectory_epoch_ = trajectory_heading_history_.epoch();
          last_trajectory_heading_reject_reason_ = "none";
          if (!use_imu_yaw_rate_heading_) {
            last_trajectory_continuity_reason_ = "disabled";
            last_trajectory_seed_innovation_rad_ =
              std::numeric_limits<double>::quiet_NaN();
          }
        }
        pose_built = true;
      } else if (!trajectory_candidate_rejected) {
        // Do not let a numerically valid but unusably uncertain chord overwrite
        // a previous good absolute-heading seed. This observation falls through
        // to the explicit position-only representation below.
        trajectory_heading_variance_rejected = true;
        trajectory_heading_reject_reason = "yaw_variance_exceeded";
        std::lock_guard<std::mutex> lock(mutex_);
        last_trajectory_heading_reject_reason_ = trajectory_heading_reject_reason;
        last_trajectory_continuity_reason_ = "not_evaluated_yaw_variance_exceeded";
        last_trajectory_seed_innovation_rad_ =
          std::numeric_limits<double>::quiet_NaN();
      }
    }
  } else if (!pose_built && use_trajectory_heading_) {
    trajectory_heading_reject_reason = "current_point_unusable";
    std::lock_guard<std::mutex> lock(mutex_);
    last_trajectory_reference_age_sec_ = std::numeric_limits<double>::quiet_NaN();
    last_trajectory_baseline_m_ = 0.0;
    last_trajectory_turn_activity_rad_ = std::numeric_limits<double>::quiet_NaN();
    last_trajectory_heading_reject_reason_ = trajectory_heading_reject_reason;
    last_trajectory_continuity_reason_ = "not_evaluated_current_point_unusable";
    last_trajectory_seed_innovation_rad_ = std::numeric_limits<double>::quiet_NaN();
  }

  // 4) Bounded corrected-IMU propagation from the latest absolute heading.
  // This is intentionally time-limited and is rejected when the IMU stream has
  // a gap; it is not an unbounded replacement for a heading observation.
  if (!pose_built && use_imu_yaw_rate_heading_ &&
    !trajectory_heading_variance_rejected)
  {
    double seed_yaw = 0.0;
    double seed_variance = yaw_variance_without_heading_;
    rclcpp::Time seed_stamp(0, 0, RCL_ROS_TIME);
    bool has_seed = false;
    pure_nmea_gga_conversion::ImuYawIntegrationResult imu_result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool trajectory_seed_epoch_valid =
        !last_valid_heading_seed_is_trajectory_ ||
        last_valid_heading_trajectory_epoch_ == trajectory_heading_history_.epoch();
      has_seed = has_last_valid_heading_ && trajectory_seed_epoch_valid;
      if (has_seed) {
        seed_yaw = last_valid_heading_rad_;
        seed_variance = last_valid_heading_cov_;
        seed_stamp = last_valid_heading_stamp_;
        const double duration = (matched.primary.stamp - seed_stamp).seconds();
        if (duration >= 0.0 && duration <= imu_yaw_rate_max_integration_sec_) {
          imu_result = integrateImuYawLocked(seed_stamp, matched.primary.stamp);
          last_imu_integration_max_gap_sec_ = imu_result.max_sample_gap_sec;
          last_imu_integration_sample_count_ = imu_result.used_sample_count;
          last_imu_integration_reason_ = "bounded_propagation:" + imu_result.reason;
        } else {
          last_imu_integration_reason_ = "seed_too_old";
          last_imu_integration_sample_count_ = 0U;
          last_imu_integration_max_gap_sec_ = 0.0;
        }
      } else {
        last_imu_integration_reason_ =
          has_last_valid_heading_ ?
          "trajectory_seed_from_previous_epoch" : "no_absolute_heading_seed";
        last_imu_integration_sample_count_ = 0U;
        last_imu_integration_max_gap_sec_ = 0.0;
      }
    }

    if (has_seed && imu_result.valid &&
      imu_result.duration_sec <= imu_yaw_rate_max_integration_sec_)
    {
      const double yaw = normalizeAngle(seed_yaw + imu_result.delta_yaw_rad);
      const double yaw_variance = seed_variance +
        imu_yaw_rate_sigma_per_sqrt_sec_ * imu_yaw_rate_sigma_per_sqrt_sec_ *
        imu_result.duration_sec;
      setSingleAntennaObservation(
        output, matched.primary, primary_position_base, has_primary_geometry,
        yaw, yaw_variance, "imu_propagated", "corrected_imu_bounded_propagation");
      pose_built = true;
    }
  }

  // 5) Position-only GGA. Never manufacture a base_link pose from an initial or
  // previous yaw. The fusion node receives the antenna lever arm explicitly and
  // can update position while keeping yaw unobserved.
  if (!pose_built) {
    output.T_map_observation = Eigen::Isometry3d::Identity();
    output.T_map_observation.translation() = Eigen::Vector3d(
      matched.primary.x, matched.primary.y, matched.primary.z);
    output.covariance_xy_yaw(2, 2) = yaw_variance_without_heading_;
    output.heading_valid = false;
    output.heading_source = "none";
    if (trajectory_heading_variance_rejected) {
      output.note = has_primary_geometry ?
        "trajectory_heading_variance_rejected_position_only_with_offset" :
        "trajectory_heading_variance_rejected_position_only_offset_unknown";
    } else if (trajectory_heading_reject_reason != "none") {
      output.note = "trajectory_heading_" + trajectory_heading_reject_reason +
        (has_primary_geometry ?
        "_position_only_with_offset" : "_position_only_offset_unknown");
    } else {
      output.note = has_primary_geometry ?
        "position_only_antenna_with_offset" : "position_only_antenna_offset_unknown";
    }
    output.position_is_base_link = false;
    output.observation_point_valid = has_primary_geometry;
    output.observation_point_in_base = primary_position_base;
  }

  // Guard against small numerical asymmetry introduced by covariance
  // propagation before serializing to ROS row-major covariance fields.
  output.covariance_xy_yaw =
    0.5 * (output.covariance_xy_yaw + output.covariance_xy_yaw.transpose());
  return output;
}


bool NmeaGgaConversion::resolveAntennaPosition(
  const std::string & antenna_frame_id,
  const std::vector<double> & fallback_position_base,
  Eigen::Vector3d & antenna_position_base,
  std::string & status) const
{
  if (use_tf_for_antenna_geometry_ && tf_buffer_) {
    try {
      const geometry_msgs::msg::TransformStamped tf =
        tf_buffer_->lookupTransform(child_frame_id_, antenna_frame_id, tf2::TimePointZero);
      antenna_position_base = Eigen::Vector3d(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z);
      status = "tf";
      return true;
    } catch (const tf2::TransformException & ex) {
      status = ex.what();
    }
    if (!allow_parameter_antenna_fallback_) {
      status = "tf_unavailable_and_parameter_fallback_disabled: " + status;
      return false;
    }
  }

  if (fallback_position_base.size() >= 3U) {
    antenna_position_base = Eigen::Vector3d(
      fallback_position_base[0], fallback_position_base[1], fallback_position_base[2]);
    if (!antenna_position_base.allFinite()) {
      status = "non_finite_parameter_geometry";
      return false;
    }
    status = "param";
    return true;
  }

  status = "unavailable";
  return false;
}

rclcpp::Time NmeaGgaConversion::resolveStamp(const builtin_interfaces::msg::Time & stamp) const
{
  const rclcpp::Time resolved(stamp);
  if (resolved.nanoseconds() == 0 && allow_reception_time_for_zero_stamp_) {
    return this->now();
  }
  return resolved;
}

int64_t NmeaGgaConversion::syncToleranceNanoseconds() const
{
  return static_cast<int64_t>(std::llround(sync_tolerance_sec_ * 1.0e9));
}

int64_t NmeaGgaConversion::bufferRetentionNanoseconds() const
{
  return static_cast<int64_t>(std::llround(buffer_retention_sec_ * 1.0e9));
}

double NmeaGgaConversion::deg2rad(double degree)
{
  return degree * kPi / 180.0;
}

double NmeaGgaConversion::sanitizeVariance(double value)
{
  if (!std::isfinite(value)) {
    return kUnknownVariance;
  }
  return std::max(value, 0.0);
}

double NmeaGgaConversion::sanitizeCovariance(double value)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return value;
}

double NmeaGgaConversion::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Quaterniond NmeaGgaConversion::quatFromYaw(double yaw)
{
  Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  q.normalize();
  return q;
}
