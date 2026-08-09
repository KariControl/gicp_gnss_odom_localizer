// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "pure_nmea_gga_conversion/imu_yaw_integrator.hpp"

namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void requireNear(double actual, double expected, double tolerance, const std::string & message)
{
  require(std::isfinite(actual) && std::fabs(actual - expected) <= tolerance, message);
}
}  // namespace

int main()
{
  using pure_nmea_gga_conversion::TimedYawRate;
  using pure_nmea_gga_conversion::integrateYawRate;

  {
    std::vector<TimedYawRate> samples{{0.0, 0.2}, {0.1, 0.2}, {0.2, 0.2}, {0.3, 0.2}};
    const auto result = integrateYawRate(samples, 0.05, 0.25, 0.11, 0.06);
    require(result.valid, "constant-rate interval should be valid");
    requireNear(result.delta_yaw_rad, 0.04, 1.0e-12, "constant-rate integral");
    requireNear(
      result.absolute_yaw_activity_rad, 0.04, 1.0e-12,
      "constant-rate absolute activity");
  }

  {
    // Signed yaw cancels, but the turn activity must remain observable. The
    // linearly interpolated rate crosses zero at t=0.5, producing two 0.25-rad
    // triangular areas.
    std::vector<TimedYawRate> samples{{0.0, -1.0}, {1.0, 1.0}};
    const auto result = integrateYawRate(samples, 0.0, 1.0, 1.1, 0.0);
    require(result.valid, "sign-changing yaw-rate interval should be valid");
    requireNear(result.delta_yaw_rad, 0.0, 1.0e-12, "signed turn should cancel");
    requireNear(
      result.absolute_yaw_activity_rad, 0.5, 1.0e-12,
      "absolute turn activity must not cancel");
  }

  {
    std::vector<TimedYawRate> samples{{0.0, 0.0}, {0.1, 0.1}, {0.2, 0.2}};
    const auto result = integrateYawRate(samples, 0.0, 0.2, 0.11, 0.0);
    require(result.valid, "linearly changing rate should be valid");
    requireNear(result.delta_yaw_rad, 0.02, 1.0e-12, "trapezoidal integral");
  }

  {
    std::vector<TimedYawRate> samples{{0.1, 0.1}, {0.2, 0.1}};
    const auto result = integrateYawRate(samples, 0.0, 0.2, 0.11, 0.02);
    require(!result.valid && result.reason == "boundary_not_covered",
      "stale boundary hold must fail closed");
  }

  {
    std::vector<TimedYawRate> samples{{0.0, 0.1}, {0.3, 0.1}};
    const auto result = integrateYawRate(samples, 0.0, 0.3, 0.1, 0.0);
    require(!result.valid, "large internal gap must be rejected");
  }

  {
    std::vector<TimedYawRate> samples{{0.0, 0.1}, {0.0, 0.1}};
    const auto result = integrateYawRate(samples, 0.0, 0.1, 0.1, 0.1);
    require(!result.valid && result.reason == "samples_not_strictly_increasing",
      "duplicate timestamps must be rejected");
  }

  std::cout << "PASS test_imu_yaw_integrator\n";
  return EXIT_SUCCESS;
}
