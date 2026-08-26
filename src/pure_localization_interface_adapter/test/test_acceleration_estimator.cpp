// SPDX-License-Identifier: Apache-2.0
#include "pure_localization_interface_adapter/acceleration_estimator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

using pure_localization_interface_adapter::AccelerationEstimator;
using pure_localization_interface_adapter::AccelerationEstimatorConfig;
using pure_localization_interface_adapter::TwistSample;

namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

bool near(double lhs, double rhs, double tolerance = 1.0e-9)
{
  return std::fabs(lhs - rhs) <= tolerance;
}

TwistSample sample(double stamp, double vx, double wz = 0.0)
{
  TwistSample output;
  output.stamp_sec = stamp;
  output.linear[0] = vx;
  output.angular[2] = wz;
  return output;
}

void testFirstSampleAndConstantSpeed()
{
  AccelerationEstimator estimator;
  const auto first = estimator.update(sample(1.0, 2.0));
  require(!first.valid && first.reset, "first sample must reset the derivative");
  const auto second = estimator.update(sample(1.1, 2.0));
  require(second.valid, "second ordered sample must be valid");
  require(near(second.linear[0], 0.0), "constant velocity must give zero acceleration");
}

void testRampAndLowPass()
{
  AccelerationEstimatorConfig config;
  config.lowpass_alpha = 1.0;
  AccelerationEstimator estimator(config);
  (void)estimator.update(sample(0.0, 0.0, 0.0));
  const auto output = estimator.update(sample(0.2, 1.0, 0.4));
  require(output.valid, "ramp sample must be valid");
  require(near(output.linear[0], 5.0), "linear derivative mismatch");
  require(near(output.angular[2], 2.0), "angular derivative mismatch");
}

void testGapAndReorderReset()
{
  AccelerationEstimatorConfig config;
  config.max_dt_sec = 0.25;
  AccelerationEstimator estimator(config);
  (void)estimator.update(sample(1.0, 0.0));
  const auto gap = estimator.update(sample(2.0, 1.0));
  require(!gap.valid && gap.reset, "large gap must reset");
  const auto reordered = estimator.update(sample(1.5, 2.0));
  require(!reordered.valid && reordered.reset, "time reversal must reset");
}

void testSaturation()
{
  AccelerationEstimatorConfig config;
  config.lowpass_alpha = 1.0;
  config.max_abs_linear_mps2 = 3.0;
  config.max_abs_angular_radps2 = 4.0;
  AccelerationEstimator estimator(config);
  (void)estimator.update(sample(0.0, 0.0, 0.0));
  const auto output = estimator.update(sample(0.1, 100.0, -100.0));
  require(near(output.linear[0], 3.0), "linear acceleration must be saturated");
  require(near(output.angular[2], -4.0), "angular acceleration must be saturated");
}
}  // namespace

int main()
{
  try {
    testFirstSampleAndConstantSpeed();
    testRampAndLowPass();
    testGapAndReorderReset();
    testSaturation();
  } catch (const std::exception & exception) {
    std::cerr << "FAIL test_acceleration_estimator: " << exception.what() << '\n';
    return EXIT_FAILURE;
  }
  std::cout << "PASS test_acceleration_estimator\n";
  return EXIT_SUCCESS;
}
