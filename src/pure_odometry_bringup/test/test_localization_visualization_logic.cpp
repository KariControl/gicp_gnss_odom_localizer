// SPDX-License-Identifier: Apache-2.0
#include "pure_odometry_bringup/localization_visualization_logic.hpp"

#include <diagnostic_msgs/msg/key_value.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace
{

diagnostic_msgs::msg::DiagnosticStatus makeStatus(
  const std::string & message,
  uint8_t level,
  const std::string & key = {},
  const std::string & value = {})
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.message = message;
  status.level = level;
  if (!key.empty()) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = value;
    status.values.push_back(entry);
  }
  return status;
}

TEST(LocalizationVisualizationLogic, InterfaceRequiresFreshLiveDiagnostic)
{
  using pure_odometry_bringup::InterfaceVisualState;
  using pure_odometry_bringup::classifyInterface;
  using pure_odometry_bringup::combineInterfaceWithOutput;
  EXPECT_EQ(classifyInterface(nullptr, false), InterfaceVisualState::WAITING);
  auto active = makeStatus(
    "localization interface adapter active",
    diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(classifyInterface(&active, true), InterfaceVisualState::ACTIVE);
  EXPECT_EQ(classifyInterface(&active, false), InterfaceVisualState::STALE);
  active.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  EXPECT_EQ(classifyInterface(&active, true), InterfaceVisualState::DEGRADED);

  EXPECT_EQ(
    combineInterfaceWithOutput(InterfaceVisualState::ACTIVE, false, false),
    InterfaceVisualState::WAITING);
  EXPECT_EQ(
    combineInterfaceWithOutput(InterfaceVisualState::ACTIVE, true, false),
    InterfaceVisualState::STALE);
  EXPECT_EQ(
    combineInterfaceWithOutput(InterfaceVisualState::ACTIVE, true, true),
    InterfaceVisualState::ACTIVE);
}

TEST(LocalizationVisualizationLogic, RegistrationParsesRealDiagnosticFields)
{
  using pure_odometry_bringup::RegistrationVisualState;
  using pure_odometry_bringup::classifyRegistration;
  auto accepted = makeStatus("running", 0U, "lidar_valid", "true");
  diagnostic_msgs::msg::KeyValue accepted_reason;
  accepted_reason.key = "lidar_rejection_reason";
  accepted_reason.value = "accepted";
  accepted.values.push_back(accepted_reason);
  EXPECT_EQ(classifyRegistration(&accepted, true), RegistrationVisualState::ACCEPTED);
  accepted.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  EXPECT_EQ(classifyRegistration(&accepted, true), RegistrationVisualState::STALE);
  accepted.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  auto rejected = makeStatus("holding", 1U, "lidar_valid", "false");
  diagnostic_msgs::msg::KeyValue reason;
  reason.key = "lidar_rejection_reason";
  reason.value = "fitness_gate";
  rejected.values.push_back(reason);
  EXPECT_EQ(classifyRegistration(&rejected, true), RegistrationVisualState::REJECTED);
  reason.value = "not_evaluated";
  rejected.values.back() = reason;
  EXPECT_EQ(classifyRegistration(&rejected, true), RegistrationVisualState::WAITING);
  EXPECT_EQ(classifyRegistration(&accepted, false), RegistrationVisualState::STALE);
}

TEST(LocalizationVisualizationLogic, GnssStateAndRequestedColorsAreStable)
{
  using pure_odometry_bringup::GnssVisualState;
  using pure_odometry_bringup::classifyGnss;
  using pure_odometry_bringup::gnssColor;
  auto status = makeStatus("tracking", 0U, "recovery.state", "tracking");
  EXPECT_EQ(classifyGnss(&status, true), GnssVisualState::TRACKING);
  const auto green = gnssColor(GnssVisualState::TRACKING);
  EXPECT_GT(green.g, green.r);
  status.values.front().value = "outage";
  EXPECT_EQ(classifyGnss(&status, true), GnssVisualState::OUTAGE);
  const auto yellow = gnssColor(GnssVisualState::OUTAGE);
  EXPECT_GT(yellow.r, yellow.b);
  EXPECT_GT(yellow.g, yellow.b);
  status.values.front().value = "reacquiring";
  EXPECT_EQ(classifyGnss(&status, true), GnssVisualState::REACQUIRING);
  status.values.front().value = "recovering";
  EXPECT_EQ(classifyGnss(&status, true), GnssVisualState::RECOVERING);
  const auto blue = gnssColor(GnssVisualState::RECOVERING);
  EXPECT_GT(blue.b, blue.r);
}

TEST(LocalizationVisualizationLogic, RateUsesSourceStampWindowAndResetsOnReversal)
{
  pure_odometry_bringup::OutputRateEstimator estimator(2.0);
  for (int index = 0; index <= 180; ++index) {
    estimator.observe(static_cast<double>(index) / 90.0);
  }
  EXPECT_NEAR(estimator.rateHz(), 90.0, 1.0e-9);
  estimator.observe(0.5);
  EXPECT_EQ(estimator.sampleCount(), 1U);
  EXPECT_DOUBLE_EQ(estimator.rateHz(), 0.0);
}

TEST(LocalizationVisualizationLogic, CumulativePublishedCountGivesPublisherRate)
{
  pure_odometry_bringup::CumulativeRateEstimator estimator(3.0);
  estimator.observe(100.0, 1000U);
  estimator.observe(101.0, 1050U);
  estimator.observe(102.0, 1100U);
  EXPECT_NEAR(estimator.rateHz(), 50.0, 1.0e-12);

  // A restarted publisher or reversed simulation clock starts a new window.
  estimator.observe(1.0, 2U);
  EXPECT_EQ(estimator.sampleCount(), 1U);
  EXPECT_DOUBLE_EQ(estimator.rateHz(), 0.0);
}

TEST(LocalizationVisualizationLogic, FreshnessUsesSimulationTimeAndBoundedClockSkew)
{
  using pure_odometry_bringup::sourceTimeFresh;
  EXPECT_TRUE(sourceTimeFresh(100.0, 99.2, 1.0));
  EXPECT_TRUE(sourceTimeFresh(100.0, 100.05, 1.0));
  EXPECT_FALSE(sourceTimeFresh(100.0, 98.9, 1.0));
  EXPECT_FALSE(sourceTimeFresh(100.0, 100.2, 1.0));
  EXPECT_FALSE(sourceTimeFresh(std::nan(""), 100.0, 1.0));
}

TEST(LocalizationVisualizationLogic, YawNormalizesQuaternion)
{
  constexpr double kPi = 3.14159265358979323846;
  const double half = kPi / 4.0;
  EXPECT_NEAR(
    pure_odometry_bringup::yawFromQuaternion(0.0, 0.0, 2.0 * std::sin(half),
      2.0 * std::cos(half)),
    kPi / 2.0, 1.0e-12);
  EXPECT_TRUE(std::isnan(
      pure_odometry_bringup::yawFromQuaternion(0.0, 0.0, 0.0, 0.0)));
}

TEST(LocalizationVisualizationLogic, CovarianceEllipseUsesOnlyFinitePsdXyBlock)
{
  const auto ellipse = pure_odometry_bringup::makeCovarianceEllipse(
    4.0, 0.0, 1.0, 2.0, 20.0);
  ASSERT_TRUE(ellipse);
  EXPECT_DOUBLE_EQ(ellipse->major_radius_m, 4.0);
  EXPECT_DOUBLE_EQ(ellipse->minor_radius_m, 2.0);
  EXPECT_DOUBLE_EQ(ellipse->yaw_rad, 0.0);

  const auto rotated = pure_odometry_bringup::makeCovarianceEllipse(
    2.5, 1.5, 2.5, 2.0, 20.0);
  ASSERT_TRUE(rotated);
  EXPECT_NEAR(rotated->yaw_rad, 3.14159265358979323846 / 4.0, 1.0e-12);
  EXPECT_FALSE(pure_odometry_bringup::makeCovarianceEllipse(
      1.0, 2.0, 1.0, 2.0, 20.0));
  EXPECT_FALSE(pure_odometry_bringup::makeCovarianceEllipse(
      std::nan(""), 0.0, 1.0, 2.0, 20.0));
}

}  // namespace
