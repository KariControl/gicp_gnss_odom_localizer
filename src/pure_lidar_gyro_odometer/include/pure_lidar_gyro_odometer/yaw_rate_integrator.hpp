// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace pure_gyro_odometer
{

struct TimedYawRate
{
  double stamp_sec{0.0};
  double yaw_rate_radps{0.0};
};

struct YawIntegrationResult
{
  bool valid{false};
  double delta_yaw_rad{0.0};
  double duration_sec{0.0};
  double max_sample_gap_sec{std::numeric_limits<double>::infinity()};
  std::size_t used_sample_count{0};
  std::string reason{"not_evaluated"};
};

// Strict trapezoidal integration over [start_sec, end_sec]. Samples must be
// finite and strictly time ordered. Interpolation is allowed only when the
// bracketing interval is bounded; a short nearest-sample hold is allowed at an
// interval boundary. This fails closed on stale, sparse, or reordered IMU data.
inline YawIntegrationResult integrateYawRateStrict(
  const std::vector<TimedYawRate> & samples,
  double start_sec,
  double end_sec,
  double max_sample_gap_sec,
  double max_boundary_gap_sec)
{
  YawIntegrationResult out;
  out.duration_sec = end_sec - start_sec;

  if (!std::isfinite(start_sec) || !std::isfinite(end_sec) ||
    !std::isfinite(max_sample_gap_sec) || !std::isfinite(max_boundary_gap_sec) ||
    end_sec <= start_sec || max_sample_gap_sec <= 0.0 || max_boundary_gap_sec < 0.0 ||
    samples.empty())
  {
    out.reason = "invalid_argument";
    return out;
  }

  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (!std::isfinite(samples[i].stamp_sec) || !std::isfinite(samples[i].yaw_rate_radps)) {
      out.reason = "non_finite_sample";
      return out;
    }
    if (i > 0 && samples[i].stamp_sec <= samples[i - 1].stamp_sec) {
      out.reason = "samples_not_strictly_increasing";
      return out;
    }
  }

  std::string interpolation_failure{"boundary_not_covered"};
  auto rate_at = [&](double stamp, double & rate) -> bool {
      auto it = std::lower_bound(
        samples.begin(), samples.end(), stamp,
        [](const TimedYawRate & sample, double value) {return sample.stamp_sec < value;});

      if (it != samples.end() && std::fabs(it->stamp_sec - stamp) <= 1.0e-12) {
        rate = it->yaw_rate_radps;
        return true;
      }
      if (it == samples.begin()) {
        if (it == samples.end() || it->stamp_sec - stamp > max_boundary_gap_sec) return false;
        rate = it->yaw_rate_radps;
        return true;
      }
      if (it == samples.end()) {
        const auto & last = samples.back();
        if (stamp - last.stamp_sec > max_boundary_gap_sec) return false;
        rate = last.yaw_rate_radps;
        return true;
      }

      const auto & after = *it;
      const auto & before = *(it - 1);
      const double gap = after.stamp_sec - before.stamp_sec;
      if (!(gap > 0.0) || gap > max_sample_gap_sec) {
        interpolation_failure = "interpolation_gap_too_large";
        return false;
      }
      const double ratio = (stamp - before.stamp_sec) / gap;
      rate = (1.0 - ratio) * before.yaw_rate_radps + ratio * after.yaw_rate_radps;
      return std::isfinite(rate);
    };

  double start_rate = 0.0;
  double end_rate = 0.0;
  if (!rate_at(start_sec, start_rate) || !rate_at(end_sec, end_rate)) {
    out.reason = interpolation_failure;
    return out;
  }

  std::vector<TimedYawRate> knots;
  knots.reserve(samples.size() + 2);
  knots.push_back({start_sec, start_rate});
  for (const auto & sample : samples) {
    if (sample.stamp_sec > start_sec && sample.stamp_sec < end_sec) knots.push_back(sample);
  }
  knots.push_back({end_sec, end_rate});

  double integral = 0.0;
  double max_gap = 0.0;
  for (std::size_t i = 1; i < knots.size(); ++i) {
    const double dt = knots[i].stamp_sec - knots[i - 1].stamp_sec;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > max_sample_gap_sec) {
      out.reason = "integration_gap_too_large";
      return out;
    }
    max_gap = std::max(max_gap, dt);
    integral += 0.5 * (knots[i - 1].yaw_rate_radps + knots[i].yaw_rate_radps) * dt;
  }

  if (!std::isfinite(integral)) {
    out.reason = "non_finite_integral";
    return out;
  }

  out.valid = true;
  out.delta_yaw_rad = integral;
  out.max_sample_gap_sec = max_gap;
  out.used_sample_count = knots.size();
  out.reason = "ok";
  return out;
}

}  // namespace pure_gyro_odometer
