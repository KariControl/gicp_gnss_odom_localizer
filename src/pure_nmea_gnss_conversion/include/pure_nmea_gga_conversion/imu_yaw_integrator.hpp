// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace pure_nmea_gga_conversion
{

struct TimedYawRate
{
  double stamp_sec{0.0};
  double yaw_rate_radps{0.0};
};

struct ImuYawIntegrationResult
{
  bool valid{false};
  double delta_yaw_rad{0.0};
  // Integral of |yaw_rate| over the interval. Unlike |delta_yaw_rad| this
  // remains large when left/right turns cancel in the signed integral, which
  // makes it suitable for rejecting trajectory chords across active turns.
  double absolute_yaw_activity_rad{0.0};
  double duration_sec{0.0};
  double max_sample_gap_sec{std::numeric_limits<double>::infinity()};
  std::size_t used_sample_count{0};
  std::string reason{"not_evaluated"};
};

// Integrates a time-ordered, bias-corrected yaw-rate stream over [start, end].
// Boundary values are linearly interpolated when bracketing samples exist. A
// bounded nearest-sample hold is permitted only at a boundary; internal gaps
// always have to satisfy max_sample_gap_sec. This deliberately fails closed so
// a sparse or stale IMU stream cannot masquerade as a valid heading update.
inline ImuYawIntegrationResult integrateYawRate(
  const std::vector<TimedYawRate> & samples,
  double start_sec,
  double end_sec,
  double max_sample_gap_sec,
  double max_boundary_gap_sec)
{
  ImuYawIntegrationResult out;
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
    if (!std::isfinite(samples[i].stamp_sec) ||
      !std::isfinite(samples[i].yaw_rate_radps))
    {
      out.reason = "non_finite_sample";
      return out;
    }
    if (i > 0 && samples[i].stamp_sec <= samples[i - 1].stamp_sec) {
      out.reason = "samples_not_strictly_increasing";
      return out;
    }
  }

  std::string interpolation_failure{"boundary_not_covered"};
  auto rate_at = [&](double t, double & rate) -> bool {
      auto it = std::lower_bound(
        samples.begin(), samples.end(), t,
        [](const TimedYawRate & sample, double value) {return sample.stamp_sec < value;});

      if (it != samples.end() && std::fabs(it->stamp_sec - t) <= 1.0e-12) {
        rate = it->yaw_rate_radps;
        return true;
      }

      if (it == samples.begin()) {
        if (it == samples.end() || it->stamp_sec - t > max_boundary_gap_sec) {
          return false;
        }
        rate = it->yaw_rate_radps;
        return true;
      }

      if (it == samples.end()) {
        const auto & last = samples.back();
        if (t - last.stamp_sec > max_boundary_gap_sec) {
          return false;
        }
        rate = last.yaw_rate_radps;
        return true;
      }

      const auto & b = *it;
      const auto & a = *(it - 1);
      const double gap = b.stamp_sec - a.stamp_sec;
      if (gap <= 0.0 || gap > max_sample_gap_sec) {
        interpolation_failure = "interpolation_gap_too_large";
        return false;
      }
      const double ratio = (t - a.stamp_sec) / gap;
      rate = (1.0 - ratio) * a.yaw_rate_radps + ratio * b.yaw_rate_radps;
      return true;
    };

  double start_rate = 0.0;
  double end_rate = 0.0;
  if (!rate_at(start_sec, start_rate) || !rate_at(end_sec, end_rate))
  {
    out.reason = interpolation_failure;
    return out;
  }

  std::vector<TimedYawRate> knots;
  knots.reserve(samples.size() + 2);
  knots.push_back({start_sec, start_rate});
  for (const auto & sample : samples) {
    if (sample.stamp_sec > start_sec && sample.stamp_sec < end_sec) {
      knots.push_back(sample);
    }
  }
  knots.push_back({end_sec, end_rate});

  if (knots.size() < 2) {
    out.reason = "not_enough_knots";
    return out;
  }

  double max_gap = 0.0;
  double integral = 0.0;
  double absolute_integral = 0.0;
  for (std::size_t i = 1; i < knots.size(); ++i) {
    const double dt = knots[i].stamp_sec - knots[i - 1].stamp_sec;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > max_sample_gap_sec) {
      out.reason = "integration_gap_too_large";
      return out;
    }
    max_gap = std::max(max_gap, dt);
    const double rate_a = knots[i - 1].yaw_rate_radps;
    const double rate_b = knots[i].yaw_rate_radps;
    integral += 0.5 * (rate_a + rate_b) * dt;

    // Exact integral of the absolute value of the linearly interpolated rate.
    // A plain trapezoid of |rate| would overestimate intervals that cross zero.
    if (rate_a * rate_b >= 0.0) {
      absolute_integral += 0.5 * (std::fabs(rate_a) + std::fabs(rate_b)) * dt;
    } else {
      const double abs_a = std::fabs(rate_a);
      const double abs_b = std::fabs(rate_b);
      const double magnitude_sum = abs_a + abs_b;
      if (magnitude_sum > 0.0) {
        absolute_integral +=
          0.5 * dt * (abs_a * abs_a + abs_b * abs_b) / magnitude_sum;
      }
    }
  }

  if (!std::isfinite(integral) || !std::isfinite(absolute_integral)) {
    out.reason = "non_finite_integral";
    return out;
  }

  out.valid = true;
  out.delta_yaw_rad = integral;
  out.absolute_yaw_activity_rad = absolute_integral;
  out.max_sample_gap_sec = max_gap;
  out.used_sample_count = knots.size();
  out.reason = "ok";
  return out;
}

}  // namespace pure_nmea_gga_conversion
