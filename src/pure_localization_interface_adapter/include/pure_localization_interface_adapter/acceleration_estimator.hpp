// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace pure_localization_interface_adapter
{

struct TwistSample
{
  double stamp_sec{0.0};
  std::array<double, 3> linear{{0.0, 0.0, 0.0}};
  std::array<double, 3> angular{{0.0, 0.0, 0.0}};
};

struct AccelerationEstimate
{
  bool valid{false};
  bool reset{false};
  double dt_sec{0.0};
  std::array<double, 3> linear{{0.0, 0.0, 0.0}};
  std::array<double, 3> angular{{0.0, 0.0, 0.0}};
};

struct AccelerationEstimatorConfig
{
  double min_dt_sec{1.0e-3};
  double max_dt_sec{0.5};
  double lowpass_alpha{0.25};
  double max_abs_linear_mps2{30.0};
  double max_abs_angular_radps2{20.0};
};

class AccelerationEstimator
{
public:
  explicit AccelerationEstimator(
    const AccelerationEstimatorConfig & config = AccelerationEstimatorConfig{})
  : config_(sanitize(config))
  {
  }

  void setConfig(const AccelerationEstimatorConfig & config)
  {
    config_ = sanitize(config);
    reset();
  }

  void reset()
  {
    has_previous_ = false;
    has_filtered_ = false;
    previous_ = TwistSample{};
    filtered_linear_.fill(0.0);
    filtered_angular_.fill(0.0);
  }

  AccelerationEstimate update(const TwistSample & sample)
  {
    AccelerationEstimate output;
    if (!finite(sample)) {
      reset();
      output.reset = true;
      return output;
    }

    if (!has_previous_) {
      previous_ = sample;
      has_previous_ = true;
      output.reset = true;
      return output;
    }

    const double dt = sample.stamp_sec - previous_.stamp_sec;
    if (!std::isfinite(dt) || dt < config_.min_dt_sec || dt > config_.max_dt_sec) {
      previous_ = sample;
      has_filtered_ = false;
      filtered_linear_.fill(0.0);
      filtered_angular_.fill(0.0);
      output.reset = true;
      output.dt_sec = dt;
      return output;
    }

    output.dt_sec = dt;
    for (std::size_t index = 0; index < 3; ++index) {
      const double raw_linear = std::clamp(
        (sample.linear[index] - previous_.linear[index]) / dt,
        -config_.max_abs_linear_mps2,
        config_.max_abs_linear_mps2);
      const double raw_angular = std::clamp(
        (sample.angular[index] - previous_.angular[index]) / dt,
        -config_.max_abs_angular_radps2,
        config_.max_abs_angular_radps2);

      if (!has_filtered_) {
        filtered_linear_[index] = raw_linear;
        filtered_angular_[index] = raw_angular;
      } else {
        filtered_linear_[index] =
          config_.lowpass_alpha * raw_linear +
          (1.0 - config_.lowpass_alpha) * filtered_linear_[index];
        filtered_angular_[index] =
          config_.lowpass_alpha * raw_angular +
          (1.0 - config_.lowpass_alpha) * filtered_angular_[index];
      }
      output.linear[index] = filtered_linear_[index];
      output.angular[index] = filtered_angular_[index];
    }

    previous_ = sample;
    has_filtered_ = true;
    output.valid = true;
    return output;
  }

private:
  static AccelerationEstimatorConfig sanitize(AccelerationEstimatorConfig config)
  {
    config.min_dt_sec = std::max(1.0e-6, config.min_dt_sec);
    config.max_dt_sec = std::max(config.min_dt_sec, config.max_dt_sec);
    config.lowpass_alpha = std::clamp(config.lowpass_alpha, 0.0, 1.0);
    config.max_abs_linear_mps2 = std::max(1.0e-6, config.max_abs_linear_mps2);
    config.max_abs_angular_radps2 = std::max(1.0e-6, config.max_abs_angular_radps2);
    return config;
  }

  static bool finite(const TwistSample & sample)
  {
    if (!std::isfinite(sample.stamp_sec)) {
      return false;
    }
    for (const double value : sample.linear) {
      if (!std::isfinite(value)) return false;
    }
    for (const double value : sample.angular) {
      if (!std::isfinite(value)) return false;
    }
    return true;
  }

  AccelerationEstimatorConfig config_;
  bool has_previous_{false};
  bool has_filtered_{false};
  TwistSample previous_;
  std::array<double, 3> filtered_linear_{{0.0, 0.0, 0.0}};
  std::array<double, 3> filtered_angular_{{0.0, 0.0, 0.0}};
};

}  // namespace pure_localization_interface_adapter
