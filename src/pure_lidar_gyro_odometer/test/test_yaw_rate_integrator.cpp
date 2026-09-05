// SPDX-License-Identifier: Apache-2.0
#include "pure_lidar_gyro_odometer/yaw_rate_integrator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace
{
using pure_gyro_odometer::TimedYawRate;
using pure_gyro_odometer::integrateYawRateStrict;

void require(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void testConstantRate()
{
  std::vector<TimedYawRate> samples{{0.0, 0.2}, {0.1, 0.2}, {0.2, 0.2}, {0.3, 0.2}};
  const auto result = integrateYawRateStrict(samples, 0.05, 0.25, 0.11, 0.06);
  require(result.valid, "bounded constant-rate stream must integrate");
  require(std::fabs(result.delta_yaw_rad - 0.04) < 1.0e-12, "constant-rate integral");
}

void testRejectsGap()
{
  std::vector<TimedYawRate> samples{{0.0, 0.2}, {0.1, 0.2}, {0.4, 0.2}};
  const auto result = integrateYawRateStrict(samples, 0.05, 0.35, 0.11, 0.06);
  require(!result.valid, "internal IMU gap must be rejected");
}

void testRejectsReorderedSamples()
{
  std::vector<TimedYawRate> samples{{0.0, 0.2}, {0.2, 0.2}, {0.1, 0.2}};
  const auto result = integrateYawRateStrict(samples, 0.0, 0.2, 0.25, 0.01);
  require(!result.valid, "reordered samples must be rejected");
}

}  // namespace

int main()
{
  testConstantRate();
  testRejectsGap();
  testRejectsReorderedSamples();
  std::cout << "PASS test_yaw_rate_integrator\n";
  return 0;
}
