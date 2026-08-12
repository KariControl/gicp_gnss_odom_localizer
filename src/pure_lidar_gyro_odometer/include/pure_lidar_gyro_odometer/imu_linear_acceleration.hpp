// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cmath>
#include <limits>

#include <Eigen/Core>

namespace pure_gyro_odometer::imu
{

inline bool validLinearAccelerationScale(const double scale)
{
  return std::isfinite(scale) && scale > 0.0;
}

inline Eigen::Vector3d transformLinearAcceleration(
  const Eigen::Matrix3d & rotation, const Eigen::Vector3d & acceleration,
  const double scale)
{
  if (!validLinearAccelerationScale(scale) || !rotation.allFinite() ||
    !acceleration.allFinite())
  {
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  }
  return scale * (rotation * acceleration);
}

inline std::array<double, 9> unknownCovariance()
{
  std::array<double, 9> covariance{};
  covariance[0] = -1.0;
  return covariance;
}

inline std::array<double, 9> rotateAndScaleCovariance(
  const std::array<double, 9> & input, const Eigen::Matrix3d & rotation,
  const double scale = 1.0)
{
  // REP-145 uses a negative first element to mark an unknown 3x3 covariance.
  // Preserve the complete input so this sentinel is not turned into a matrix.
  if (input[0] < 0.0) {
    return input;
  }
  if (!validLinearAccelerationScale(scale) || !rotation.allFinite()) {
    return unknownCovariance();
  }

  Eigen::Matrix3d covariance;
  covariance <<
    input[0], input[1], input[2],
    input[3], input[4], input[5],
    input[6], input[7], input[8];
  if (!covariance.allFinite()) {
    return unknownCovariance();
  }

  const Eigen::Matrix3d transformed =
    (scale * scale) * rotation * covariance * rotation.transpose();
  if (!transformed.allFinite()) {
    return unknownCovariance();
  }

  std::array<double, 9> output{};
  output[0] = transformed(0, 0);
  output[1] = transformed(0, 1);
  output[2] = transformed(0, 2);
  output[3] = transformed(1, 0);
  output[4] = transformed(1, 1);
  output[5] = transformed(1, 2);
  output[6] = transformed(2, 0);
  output[7] = transformed(2, 1);
  output[8] = transformed(2, 2);
  return output;
}

}  // namespace pure_gyro_odometer::imu
