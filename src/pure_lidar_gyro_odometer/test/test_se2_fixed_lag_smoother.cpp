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

Pose runBiasedTrack(bool use_local_factors)
{
  Config config;
  config.window_size = 25;
  config.max_iterations = 8;
  config.scan_weight = 8.0;
  config.imu_weight = 20.0;
  config.smoothness_weight = 0.05;
  config.local_huber_delta_xy_m = 0.50;
  config.local_huber_delta_yaw_rad = 0.15;

  FixedLagSmoother smoother(config);
  smoother.reset(Pose{});
  Pose output{};
  for (int step = 1; step <= 100; ++step) {
    RelativeFactor factor;
    factor.dx = 1.01;  // 1% scan-scale bias
    factor.dy = 0.002;
    factor.dyaw_scan = 0.0005;
    factor.dyaw_imu = 0.0;
    factor.fitness = 0.05;
    factor.converged = true;
    if (use_local_factors && step % 10 == 0) {
      factor.has_local_pose = true;
      factor.local_pose = Pose{static_cast<double>(step), 0.0, 0.0};
      factor.local_weight_xy = 20.0;
      factor.local_weight_yaw = 20.0;
    }
    require(smoother.addFactor(factor, output), "biased-track factor must optimize");
    require(smoother.factorCount() <= 25U, "fixed-lag window must remain bounded");
  }
  return output;
}

void testLocalFactorReducesDrift()
{
  const Pose without_local = runBiasedTrack(false);
  const Pose with_local = runBiasedTrack(true);
  const double error_without = std::hypot(without_local.x - 100.0, without_local.y);
  const double error_with = std::hypot(with_local.x - 100.0, with_local.y);
  require(error_without > 0.5, "synthetic scan bias must create measurable drift");
  require(error_with < 0.45 * error_without, "local pose factors must reduce endpoint drift");
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

Pose runCompetingLateralObservation(double lateral_information)
{
  Config config;
  config.scan_weight = 10.0;
  config.imu_weight = 10.0;
  config.smoothness_weight = 0.0;
  config.local_huber_delta_xy_m = 10.0;
  config.max_solution_position_correction_m = 5.0;
  FixedLagSmoother smoother(config);
  smoother.reset(Pose{});

  RelativeFactor factor;
  factor.dx = 1.0;
  factor.dy = 1.0;
  factor.dyaw_scan = 0.0;
  factor.dyaw_imu = 0.0;
  factor.fitness = 0.0;
  factor.converged = true;
  factor.has_scan_information = true;
  factor.scan_information = {{
    1.0, 0.0, 0.0,
    0.0, lateral_information, 0.0,
    0.0, 0.0, 1.0}};
  factor.has_local_pose = true;
  factor.local_pose = Pose{1.0, 0.0, 0.0};
  factor.local_weight_xy = 4.0;
  factor.local_weight_yaw = 4.0;

  Pose output{};
  require(smoother.addFactor(factor, output), "competing observation must optimize");
  return output;
}

void testAnisotropicInformationDoesNotOvertrustWeakDirection()
{
  const Pose isotropic = runCompetingLateralObservation(1.0);
  const Pose weak_lateral = runCompetingLateralObservation(0.01);
  require(
    std::fabs(weak_lateral.y) < 0.35 * std::fabs(isotropic.y),
    "weak Hessian direction must yield more strongly to the local-map observation");
}


void testOptimizedPoseWindowIsChronologicalAndBounded()
{
  Config config;
  config.window_size = 3;
  config.smoothness_weight = 0.0;
  FixedLagSmoother smoother(config);
  smoother.reset(Pose{2.0, -1.0, 0.1});

  RelativeFactor factor;
  factor.dx = 1.0;
  factor.dy = 0.0;
  factor.dyaw_scan = 0.0;
  factor.dyaw_imu = 0.0;
  factor.has_imu_yaw = true;
  factor.fitness = 0.0;
  factor.converged = true;

  Pose output{};
  for (int step = 0; step < 6; ++step) {
    require(smoother.addFactor(factor, output), "window factor must optimize");
    const auto poses = smoother.optimizedPoses();
    require(!poses.empty(), "optimized pose window must be available");
    require(poses.size() == smoother.factorCount() + 1, "pose/factor count mismatch");
    require(poses.size() <= 4U, "fixed-lag pose window must remain bounded");
    require(
      std::fabs(poses.back().x - smoother.pose().x) < 1.0e-12 &&
      std::fabs(poses.back().y - smoother.pose().y) < 1.0e-12 &&
      std::fabs(poses.back().yaw - smoother.pose().yaw) < 1.0e-12,
      "last optimized pose must equal smoother output");
    for (std::size_t index = 1; index < poses.size(); ++index) {
      require(poses[index].x > poses[index - 1].x, "pose window must be chronological");
    }
  }
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
  testLocalFactorReducesDrift();
  testRejectsNonFiniteFactorWithoutStateCorruption();
  testZuptSuppressesStationaryDrift();
  testAnisotropicInformationDoesNotOvertrustWeakDirection();
  testOptimizedPoseWindowIsChronologicalAndBounded();
  testRejectsIndefiniteInformationWithoutStateCorruption();
  std::cout << "PASS test_se2_fixed_lag_smoother\n";
  return 0;
}
