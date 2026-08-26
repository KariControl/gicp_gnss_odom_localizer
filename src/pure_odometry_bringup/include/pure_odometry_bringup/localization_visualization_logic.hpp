// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>

namespace pure_odometry_bringup
{

struct Rgba
{
  float r{1.0F};
  float g{1.0F};
  float b{1.0F};
  float a{1.0F};
};

struct CovarianceEllipse
{
  double major_radius_m{0.0};
  double minor_radius_m{0.0};
  double yaw_rad{0.0};
};

inline std::optional<CovarianceEllipse> makeCovarianceEllipse(
  double covariance_xx,
  double covariance_xy,
  double covariance_yy,
  double sigma_scale,
  double max_radius_m)
{
  if (!std::isfinite(covariance_xx) || !std::isfinite(covariance_xy) ||
    !std::isfinite(covariance_yy) || !std::isfinite(sigma_scale) ||
    !std::isfinite(max_radius_m) || sigma_scale <= 0.0 || max_radius_m <= 0.0)
  {
    return std::nullopt;
  }

  const double discriminant = std::hypot(
    covariance_xx - covariance_yy, 2.0 * covariance_xy);
  const double trace = covariance_xx + covariance_yy;
  double major_variance = 0.5 * (trace + discriminant);
  double minor_variance = 0.5 * (trace - discriminant);
  constexpr double kPsdTolerance = 1.0e-9;
  if (major_variance < -kPsdTolerance || minor_variance < -kPsdTolerance) {
    return std::nullopt;
  }
  major_variance = std::max(0.0, major_variance);
  minor_variance = std::max(0.0, minor_variance);

  CovarianceEllipse ellipse;
  ellipse.major_radius_m = std::min(
    max_radius_m, sigma_scale * std::sqrt(major_variance));
  ellipse.minor_radius_m = std::min(
    max_radius_m, sigma_scale * std::sqrt(minor_variance));
  ellipse.yaw_rad = 0.5 * std::atan2(
    2.0 * covariance_xy, covariance_xx - covariance_yy);
  return ellipse;
}

enum class InterfaceVisualState
{
  WAITING,
  ACTIVE,
  DEGRADED,
  STALE,
};

enum class RegistrationVisualState
{
  WAITING,
  ACCEPTED,
  REJECTED,
  STALE,
};

enum class GnssVisualState
{
  WAITING,
  TRACKING,
  OUTAGE,
  REACQUIRING,
  RECOVERING,
  ERROR,
  STALE,
};

inline std::optional<std::string> diagnosticValue(
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key)
{
  const auto iterator = std::find_if(
    status.values.begin(), status.values.end(),
    [&key](const auto & value) {return value.key == key;});
  if (iterator == status.values.end()) {
    return std::nullopt;
  }
  return iterator->value;
}

inline InterfaceVisualState classifyInterface(
  const diagnostic_msgs::msg::DiagnosticStatus * status,
  bool fresh)
{
  if (status == nullptr) {
    return InterfaceVisualState::WAITING;
  }
  if (!fresh) {
    return InterfaceVisualState::STALE;
  }
  if (status->level == diagnostic_msgs::msg::DiagnosticStatus::OK &&
    status->message == "localization interface adapter active")
  {
    return InterfaceVisualState::ACTIVE;
  }
  return InterfaceVisualState::DEGRADED;
}

inline InterfaceVisualState combineInterfaceWithOutput(
  InterfaceVisualState diagnostic_state,
  bool has_output,
  bool output_fresh)
{
  if (!has_output) {
    return InterfaceVisualState::WAITING;
  }
  if (!output_fresh) {
    return InterfaceVisualState::STALE;
  }
  return diagnostic_state;
}

inline RegistrationVisualState classifyRegistration(
  const diagnostic_msgs::msg::DiagnosticStatus * status,
  bool fresh)
{
  if (status == nullptr) {
    return RegistrationVisualState::WAITING;
  }
  if (!fresh) {
    return RegistrationVisualState::STALE;
  }
  const auto valid = diagnosticValue(*status, "lidar_valid");
  const auto reason = diagnosticValue(*status, "lidar_rejection_reason");
  if (valid && *valid == "true") {
    // The odometer retains the last successful registration values after
    // their LiDAR timeout. In that case lidar_valid remains true while the
    // current diagnostic level changes to WARN. Never present that stale
    // result as a current acceptance.
    if (status->level == diagnostic_msgs::msg::DiagnosticStatus::OK &&
      (!reason || *reason == "accepted"))
    {
      return RegistrationVisualState::ACCEPTED;
    }
    return RegistrationVisualState::STALE;
  }
  if (!reason || *reason == "not_evaluated" || *reason == "initialization") {
    return RegistrationVisualState::WAITING;
  }
  return RegistrationVisualState::REJECTED;
}

inline GnssVisualState classifyGnss(
  const diagnostic_msgs::msg::DiagnosticStatus * status,
  bool fresh)
{
  if (status == nullptr) {
    return GnssVisualState::WAITING;
  }
  if (!fresh) {
    return GnssVisualState::STALE;
  }
  if (status->level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    return GnssVisualState::ERROR;
  }
  const auto state = diagnosticValue(*status, "recovery.state");
  if (!state || *state == "uninitialized") {
    return GnssVisualState::WAITING;
  }
  if (*state == "tracking" || *state == "tracking_xy_only") {
    return GnssVisualState::TRACKING;
  }
  if (*state == "outage") {
    return GnssVisualState::OUTAGE;
  }
  if (*state == "reacquiring") {
    return GnssVisualState::REACQUIRING;
  }
  if (*state == "recovering" || *state == "recovering_xy_only") {
    return GnssVisualState::RECOVERING;
  }
  return GnssVisualState::WAITING;
}

inline const char * interfaceLabel(InterfaceVisualState state)
{
  switch (state) {
    case InterfaceVisualState::ACTIVE:
      return "ACTIVE";
    case InterfaceVisualState::DEGRADED:
      return "DEGRADED";
    case InterfaceVisualState::STALE:
      return "STALE";
    case InterfaceVisualState::WAITING:
      return "WAITING";
  }
  return "WAITING";
}

inline const char * registrationLabel(RegistrationVisualState state)
{
  switch (state) {
    case RegistrationVisualState::ACCEPTED:
      return "ACCEPTED";
    case RegistrationVisualState::REJECTED:
      return "REJECTED";
    case RegistrationVisualState::STALE:
      return "STALE";
    case RegistrationVisualState::WAITING:
      return "WAITING";
  }
  return "WAITING";
}

inline const char * gnssLabel(GnssVisualState state)
{
  switch (state) {
    case GnssVisualState::TRACKING:
      return "TRACKING";
    case GnssVisualState::OUTAGE:
      return "OUTAGE";
    case GnssVisualState::REACQUIRING:
      return "REACQUIRING";
    case GnssVisualState::RECOVERING:
      return "RECOVERING";
    case GnssVisualState::ERROR:
      return "ERROR";
    case GnssVisualState::STALE:
      return "STALE";
    case GnssVisualState::WAITING:
      return "WAITING";
  }
  return "WAITING";
}

inline Rgba interfaceColor(InterfaceVisualState state)
{
  switch (state) {
    case InterfaceVisualState::ACTIVE:
      return {0.15F, 1.0F, 0.25F, 1.0F};
    case InterfaceVisualState::DEGRADED:
      return {1.0F, 0.75F, 0.1F, 1.0F};
    case InterfaceVisualState::STALE:
      return {1.0F, 0.45F, 0.1F, 1.0F};
    case InterfaceVisualState::WAITING:
      return {0.7F, 0.7F, 0.7F, 1.0F};
  }
  return {};
}

inline Rgba registrationColor(RegistrationVisualState state)
{
  switch (state) {
    case RegistrationVisualState::ACCEPTED:
      return {0.15F, 1.0F, 0.25F, 1.0F};
    case RegistrationVisualState::REJECTED:
      return {1.0F, 0.75F, 0.1F, 1.0F};
    case RegistrationVisualState::STALE:
      return {1.0F, 0.45F, 0.1F, 1.0F};
    case RegistrationVisualState::WAITING:
      return {0.7F, 0.7F, 0.7F, 1.0F};
  }
  return {};
}

inline Rgba gnssColor(GnssVisualState state)
{
  switch (state) {
    case GnssVisualState::TRACKING:
      return {0.15F, 1.0F, 0.25F, 1.0F};
    case GnssVisualState::OUTAGE:
    case GnssVisualState::REACQUIRING:
      return {1.0F, 0.8F, 0.1F, 1.0F};
    case GnssVisualState::RECOVERING:
      return {0.2F, 0.65F, 1.0F, 1.0F};
    case GnssVisualState::ERROR:
      return {1.0F, 0.2F, 0.2F, 1.0F};
    case GnssVisualState::STALE:
      return {1.0F, 0.45F, 0.1F, 1.0F};
    case GnssVisualState::WAITING:
      return {0.7F, 0.7F, 0.7F, 1.0F};
  }
  return {};
}

inline double yawFromQuaternion(double x, double y, double z, double w)
{
  const double norm_squared = x * x + y * y + z * z + w * w;
  if (!std::isfinite(norm_squared) || norm_squared <= 1.0e-12) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double scale = 1.0 / std::sqrt(norm_squared);
  x *= scale;
  y *= scale;
  z *= scale;
  w *= scale;
  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}

inline bool sourceTimeFresh(double current_sec, double source_sec, double limit_sec)
{
  if (!std::isfinite(current_sec) || !std::isfinite(source_sec) ||
    !std::isfinite(limit_sec) || current_sec <= 0.0 || source_sec <= 0.0 ||
    limit_sec <= 0.0)
  {
    return false;
  }

  // A message can be a few milliseconds ahead of the latest /clock sample.
  // Accept that bounded skew, but reject a genuinely future timestamp.
  const double age_sec = current_sec - source_sec;
  constexpr double kFutureToleranceSec = 0.1;
  return age_sec >= -kFutureToleranceSec && age_sec <= limit_sec;
}

class OutputRateEstimator
{
public:
  explicit OutputRateEstimator(double window_sec = 2.0)
  : window_sec_(std::max(0.1, window_sec))
  {
  }

  void setWindow(double window_sec)
  {
    window_sec_ = std::max(0.1, window_sec);
    trim();
  }

  void observe(double stamp_sec)
  {
    if (!std::isfinite(stamp_sec)) {
      return;
    }
    if (!stamps_.empty() && stamp_sec < stamps_.back()) {
      stamps_.clear();
    }
    if (!stamps_.empty() && stamp_sec == stamps_.back()) {
      return;
    }
    stamps_.push_back(stamp_sec);
    trim();
  }

  [[nodiscard]] double rateHz() const
  {
    if (stamps_.size() < 2U) {
      return 0.0;
    }
    const double span = stamps_.back() - stamps_.front();
    return span > 0.0 ? static_cast<double>(stamps_.size() - 1U) / span : 0.0;
  }

  [[nodiscard]] std::size_t sampleCount() const {return stamps_.size();}

private:
  void trim()
  {
    if (stamps_.empty()) {
      return;
    }
    const double earliest = stamps_.back() - window_sec_;
    while (stamps_.size() > 2U && stamps_[1] < earliest) {
      stamps_.pop_front();
    }
  }

  double window_sec_{2.0};
  std::deque<double> stamps_;
};

class CumulativeRateEstimator
{
public:
  explicit CumulativeRateEstimator(double window_sec = 3.0)
  : window_sec_(std::max(0.1, window_sec))
  {
  }

  void observe(double stamp_sec, std::uint64_t count)
  {
    if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
      return;
    }
    if (!samples_.empty() &&
      (stamp_sec < samples_.back().stamp_sec || count < samples_.back().count))
    {
      samples_.clear();
    }
    if (!samples_.empty() && stamp_sec == samples_.back().stamp_sec) {
      samples_.back().count = count;
      return;
    }
    samples_.push_back({stamp_sec, count});
    trim();
  }

  [[nodiscard]] double rateHz() const
  {
    if (samples_.size() < 2U) {
      return 0.0;
    }
    const double span_sec = samples_.back().stamp_sec - samples_.front().stamp_sec;
    const std::uint64_t count_delta = samples_.back().count - samples_.front().count;
    return span_sec > 0.0 ? static_cast<double>(count_delta) / span_sec : 0.0;
  }

  [[nodiscard]] std::size_t sampleCount() const {return samples_.size();}

private:
  struct Sample
  {
    double stamp_sec;
    std::uint64_t count;
  };

  void trim()
  {
    if (samples_.empty()) {
      return;
    }
    const double earliest = samples_.back().stamp_sec - window_sec_;
    while (samples_.size() > 2U && samples_[1].stamp_sec < earliest) {
      samples_.pop_front();
    }
  }

  double window_sec_{3.0};
  std::deque<Sample> samples_;
};

}  // namespace pure_odometry_bringup
