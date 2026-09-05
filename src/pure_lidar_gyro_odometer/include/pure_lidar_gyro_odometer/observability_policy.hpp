// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>

namespace pure_gyro_odometer::observability
{

inline double clampUnit(double value)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::max(0.0, std::min(1.0, value));
}

// Continuous complement of directional information. There is deliberately no
// threshold or boolean classification: a ratio of 1 is fully observed and a
// ratio of 0 is maximally weak.
inline double weaknessWeight(double information_ratio, double power)
{
  const double bounded_ratio = clampUnit(information_ratio);
  const double bounded_power =
    std::isfinite(power) && power > 0.0 ? power : 1.0;
  return std::pow(1.0 - bounded_ratio, bounded_power);
}

// Monotonic covariance inflation driven by a continuous information deficit.
inline double covarianceScale(double information_deficit, double maximum_scale)
{
  const double bounded_maximum =
    std::isfinite(maximum_scale) ? std::max(1.0, maximum_scale) : 1.0;
  return 1.0 + (bounded_maximum - 1.0) * clampUnit(information_deficit);
}

}  // namespace pure_gyro_odometer::observability
