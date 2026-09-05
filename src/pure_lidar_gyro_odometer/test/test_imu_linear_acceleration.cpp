// SPDX-License-Identifier: Apache-2.0
#include "pure_lidar_gyro_odometer/imu_linear_acceleration.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

#include <Eigen/Core>

namespace
{

using pure_gyro_odometer::imu::rotateAndScaleCovariance;
using pure_gyro_odometer::imu::transformLinearAcceleration;
using pure_gyro_odometer::imu::validLinearAccelerationScale;

void require(const bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void requireNear(const double actual, const double expected, const std::string & message)
{
  require(std::fabs(actual - expected) < 1.0e-12, message);
}

Eigen::Matrix3d quarterTurnAboutZ()
{
  Eigen::Matrix3d rotation;
  rotation <<
    0.0, -1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, 1.0;
  return rotation;
}

void testVectorIsRotatedAndScaled()
{
  const Eigen::Vector3d transformed = transformLinearAcceleration(
    quarterTurnAboutZ(), Eigen::Vector3d{1.0, 2.0, 3.0}, 2.0);
  requireNear(transformed.x(), -4.0, "rotated/scaled vector x");
  requireNear(transformed.y(), 2.0, "rotated/scaled vector y");
  requireNear(transformed.z(), 6.0, "rotated/scaled vector z");
}

void testCovarianceIsRotatedAndScaledByScaleSquared()
{
  const std::array<double, 9> input{
    1.0, 0.1, 0.2,
    0.1, 4.0, 0.3,
    0.2, 0.3, 9.0};
  const auto transformed = rotateAndScaleCovariance(input, quarterTurnAboutZ(), 2.0);
  const std::array<double, 9> expected{
    16.0, -0.4, -1.2,
    -0.4, 4.0, 0.8,
    -1.2, 0.8, 36.0};
  for (std::size_t index = 0; index < expected.size(); ++index) {
    requireNear(transformed[index], expected[index], "rotated/scaled covariance");
  }

  const auto rotation_only = rotateAndScaleCovariance(input, quarterTurnAboutZ());
  for (std::size_t index = 0; index < expected.size(); ++index) {
    requireNear(rotation_only[index], expected[index] / 4.0, "unit-scale covariance");
  }
}

void testUnknownCovarianceIsPreserved()
{
  const std::array<double, 9> unknown{-1.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0};
  require(
    rotateAndScaleCovariance(unknown, quarterTurnAboutZ(), 9.80665) == unknown,
    "unknown covariance sentinel and payload must remain unchanged");
}

void testInvalidInputsFailClosed()
{
  require(validLinearAccelerationScale(1.0), "unit scale is valid");
  require(validLinearAccelerationScale(9.80665), "gravity scale is valid");
  require(!validLinearAccelerationScale(0.0), "zero scale is invalid");
  require(!validLinearAccelerationScale(-1.0), "negative scale is invalid");
  require(
    !validLinearAccelerationScale(std::numeric_limits<double>::infinity()),
    "infinite scale is invalid");
  require(
    !validLinearAccelerationScale(std::numeric_limits<double>::quiet_NaN()),
    "NaN scale is invalid");

  const Eigen::Vector3d invalid_vector = transformLinearAcceleration(
    Eigen::Matrix3d::Identity(), Eigen::Vector3d::Ones(), 0.0);
  require(!invalid_vector.allFinite(), "invalid vector scale must not produce usable data");

  const std::array<double, 9> covariance{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  const auto invalid_scale = rotateAndScaleCovariance(
    covariance, Eigen::Matrix3d::Identity(), 0.0);
  require(invalid_scale[0] < 0.0, "invalid covariance scale must produce unknown covariance");

  auto nonfinite = covariance;
  nonfinite[4] = std::numeric_limits<double>::quiet_NaN();
  const auto invalid_covariance = rotateAndScaleCovariance(
    nonfinite, Eigen::Matrix3d::Identity(), 1.0);
  require(invalid_covariance[0] < 0.0, "non-finite covariance must become unknown");
}

}  // namespace

int main()
{
  testVectorIsRotatedAndScaled();
  testCovarianceIsRotatedAndScaledByScaleSquared();
  testUnknownCovarianceIsPreserved();
  testInvalidInputsFailClosed();
  std::cout << "PASS test_imu_linear_acceleration\n";
  return 0;
}
