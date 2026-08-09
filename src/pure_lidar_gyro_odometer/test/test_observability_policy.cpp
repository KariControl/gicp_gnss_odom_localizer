// SPDX-License-Identifier: Apache-2.0
#include "pure_lidar_gyro_odometer/observability_policy.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace
{

void require(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAIL " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void testWeaknessIsContinuousAndMonotonic()
{
  using pure_gyro_odometer::observability::weaknessWeight;
  const double left = weaknessWeight(0.499999, 2.0);
  const double right = weaknessWeight(0.500001, 2.0);
  require(std::fabs(left - right) < 1.0e-5, "weakness must have no threshold jump");
  require(weaknessWeight(0.0, 2.0) > weaknessWeight(0.5, 2.0),
    "less information must produce more weakness");
  require(weaknessWeight(0.5, 2.0) > weaknessWeight(1.0, 2.0),
    "fully observed direction must have zero weakness");
  require(std::fabs(weaknessWeight(1.0, 2.0)) < 1.0e-12,
    "fully observed direction must not be corrected");
}

void testCovarianceScaleIsContinuousAndBounded()
{
  using pure_gyro_odometer::observability::covarianceScale;
  require(std::fabs(covarianceScale(0.0, 3.0) - 1.0) < 1.0e-12,
    "zero deficit must preserve covariance");
  require(std::fabs(covarianceScale(1.0, 3.0) - 3.0) < 1.0e-12,
    "full deficit must reach configured maximum");
  require(std::fabs(covarianceScale(0.5, 3.0) - 2.0) < 1.0e-12,
    "mid deficit must interpolate linearly");
  require(covarianceScale(-1.0, 3.0) == 1.0,
    "negative deficit must clamp safely");
  require(covarianceScale(2.0, 3.0) == 3.0,
    "excess deficit must clamp safely");
}

}  // namespace

int main()
{
  testWeaknessIsContinuousAndMonotonic();
  testCovarianceScaleIsContinuousAndBounded();
  std::cout << "PASS test_observability_policy\n";
  return EXIT_SUCCESS;
}
