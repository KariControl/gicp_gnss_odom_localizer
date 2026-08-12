// SPDX-License-Identifier: Apache-2.0
#include "pure_lidar_gyro_odometer/se2_fixed_lag_smoother.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace
{
using pure_gyro_odometer::se2::Config;
using pure_gyro_odometer::se2::FixedLagSmoother;
using pure_gyro_odometer::se2::Pose;
using pure_gyro_odometer::se2::RelativeFactor;

void require(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void testRejectsNonFiniteFactorWithoutStateCorruption()
{
  Config config;
  FixedLagSmoother smoother(config);
  smoother.reset(Pose{1.0, 2.0, 0.3});

  RelativeFactor valid;
  valid.dx = 1.0;
  valid.dyaw_scan = 0.01;
  valid.dyaw_imu = 0.01;
  valid.fitness = 0.1;
  valid.converged = true;
  Pose before{};
  require(smoother.addFactor(valid, before), "valid factor must optimize");
  const std::size_t count_before = smoother.factorCount();

  RelativeFactor invalid = valid;
  invalid.dx = std::numeric_limits<double>::quiet_NaN();
  Pose ignored{};
  require(!smoother.addFactor(invalid, ignored), "NaN factor must be rejected");
  require(smoother.factorCount() == count_before, "rejected factor must not alter window");
  const Pose after = smoother.pose();
  require(std::fabs(after.x - before.x) < 1.0e-12, "rejected factor must preserve x");
  require(std::fabs(after.y - before.y) < 1.0e-12, "rejected factor must preserve y");
  require(std::fabs(after.yaw - before.yaw) < 1.0e-12, "rejected factor must preserve yaw");
}

void testZuptSuppressesStationaryDrift()
{
  Config config;
  config.zupt_enable = true;
  config.zupt_weight_translation = 100.0;
  config.zupt_weight_yaw = 100.0;
  config.scan_weight = 1.0;
  config.imu_weight = 10.0;
  FixedLagSmoother smoother(config);
  smoother.reset(Pose{});

  Pose output{};
  for (int step = 0; step < 20; ++step) {
    RelativeFactor factor;
    factor.dx = 0.02;
    factor.dy = -0.01;
    factor.dyaw_scan = 0.002;
    factor.dyaw_imu = 0.0;
    factor.fitness = 0.3;
    factor.converged = true;
    factor.stationary = true;
    require(smoother.addFactor(factor, output), "stationary factor must optimize");
  }
  require(std::hypot(output.x, output.y) < 0.03, "ZUPT must suppress stationary translation drift");
  require(std::fabs(output.yaw) < 0.01, "ZUPT must suppress stationary yaw drift");
}

void testRejectsIndefiniteInformationWithoutStateCorruption()
{
  FixedLagSmoother smoother(Config{});
  smoother.reset(Pose{});
  RelativeFactor valid;
  valid.dx = 1.0;
  valid.dyaw_scan = 0.0;
  valid.dyaw_imu = 0.0;
  valid.fitness = 0.0;
  valid.converged = true;
  Pose before{};
  require(smoother.addFactor(valid, before), "baseline factor must optimize");
  const std::size_t count_before = smoother.factorCount();

  RelativeFactor invalid = valid;
  invalid.has_scan_information = true;
  invalid.scan_information = {{
    1.0, 2.0, 0.0,
    2.0, 1.0, 0.0,
    0.0, 0.0, 1.0}};  // one negative eigenvalue
  Pose ignored{};
  require(!smoother.addFactor(invalid, ignored), "indefinite information must be rejected");
  require(smoother.factorCount() == count_before, "invalid information must not alter the window");
  require(std::fabs(smoother.pose().x - before.x) < 1.0e-12, "state must roll back");
}

}  // namespace

int main()
{
  testRejectsNonFiniteFactorWithoutStateCorruption();
  testZuptSuppressesStationaryDrift();
  testRejectsIndefiniteInformationWithoutStateCorruption();
  std::cout << "PASS test_se2_fixed_lag_smoother\n";
  return 0;
}
