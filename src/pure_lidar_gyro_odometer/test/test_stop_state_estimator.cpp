// SPDX-License-Identifier: Apache-2.0
#include "pure_lidar_gyro_odometer/stop_state_estimator.hpp"

#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

namespace
{
using pure_gyro_odometer::stop::Config;
using pure_gyro_odometer::stop::ImuSample;
using pure_gyro_odometer::stop::StopStateEstimator;
using pure_gyro_odometer::stop::TimedSpeed;

constexpr std::int64_t kSecond = 1000000000LL;
constexpr std::int64_t kTenMilliseconds = 10000000LL;

void require(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

Config testConfig()
{
  Config config;
  config.imu_window_sec = 0.3;
  config.min_imu_samples = 3;
  config.speed_threshold_mps = 0.15;
  config.gyro_mean_threshold_rad_s = 0.05;
  config.acceleration_variance_threshold = 0.01;
  config.hold_sec = 0.0;
  config.decision_history_sec = 5.0;
  config.max_imu_history_size = 64;
  config.max_decision_history_size = 64;
  return config;
}

ImuSample quietSample(std::int64_t stamp_ns)
{
  return {stamp_ns, {{0.0, 0.0, 9.81}}, 0.0};
}

bool feedThreeQuiet(
  StopStateEstimator & estimator,
  const std::optional<double> & speed_mps,
  std::int64_t start_ns = kSecond)
{
  bool stopped = false;
  for (int index = 0; index < 3; ++index) {
    const auto stamp = start_ns + index * kTenMilliseconds;
    std::optional<TimedSpeed> speed;
    if (speed_mps.has_value()) {
      speed = TimedSpeed{stamp, *speed_mps, 0.1};
    }
    stopped = estimator.update(quietSample(stamp), speed).stopped;
  }
  return stopped;
}

void testRequiredTruthTable()
{
  {
    StopStateEstimator estimator(testConfig());
    require(feedThreeQuiet(estimator, 0.05), "quiet IMU AND low speed must stop");
  }
  {
    StopStateEstimator estimator(testConfig());
    require(!feedThreeQuiet(estimator, 1.0), "quiet IMU AND high speed must not stop");
  }
  {
    StopStateEstimator estimator(testConfig());
    require(feedThreeQuiet(estimator, std::nullopt), "quiet IMU without speed must use IMU fallback");
  }
  {
    StopStateEstimator estimator(testConfig());
    estimator.update(quietSample(kSecond), TimedSpeed{kSecond, 0.05, 0.1});
    estimator.update(quietSample(kSecond + kTenMilliseconds),
      TimedSpeed{kSecond + kTenMilliseconds, 0.05, 0.1});
    auto noisy = quietSample(kSecond + 2 * kTenMilliseconds);
    noisy.gyro_z_rad_s = 1.0;
    const auto result = estimator.update(
      noisy, TimedSpeed{noisy.stamp_ns, 0.05, 0.1});
    require(!result.imu_quiet, "high gyro mean must fail the IMU quiet gate");
    require(!result.stopped, "low speed cannot bypass a noisy IMU gate");
  }
  {
    StopStateEstimator estimator(testConfig());
    for (int index = 0; index < 3; ++index) {
      const auto stamp = kSecond + index * kTenMilliseconds;
      auto varying_acceleration = quietSample(stamp);
      varying_acceleration.acceleration[0] = static_cast<double>(index - 1);
      const auto result = estimator.update(
        varying_acceleration, TimedSpeed{stamp, 0.05, 0.1});
      if (index == 2) {
        require(!result.imu_quiet, "acceleration variance must fail the IMU quiet gate");
        require(!result.stopped, "low speed cannot bypass acceleration variance");
      }
    }
  }
}

void testFutureAndStaleSpeedAreRejected()
{
  StopStateEstimator estimator(testConfig());
  estimator.update(quietSample(kSecond));
  estimator.update(quietSample(kSecond + kTenMilliseconds));
  const auto query_stamp = kSecond + 2 * kTenMilliseconds;
  const auto future = estimator.update(
    quietSample(query_stamp), TimedSpeed{query_stamp + kTenMilliseconds, 1.0, 0.1});
  require(!future.speed_available, "future speed must not be used");
  require(future.stopped, "rejected future speed leaves the specified IMU-only fallback");

  StopStateEstimator stale_estimator(testConfig());
  stale_estimator.update(quietSample(2 * kSecond));
  stale_estimator.update(quietSample(2 * kSecond + kTenMilliseconds));
  const auto stale = stale_estimator.update(
    quietSample(2 * kSecond + 2 * kTenMilliseconds),
    TimedSpeed{2 * kSecond - 200000000LL, 1.0, 0.1});
  require(!stale.speed_available, "stale speed must not be used");
}

void testQuietPastIsNotChangedByNoisyFuture()
{
  StopStateEstimator estimator(testConfig());
  require(feedThreeQuiet(estimator, std::nullopt), "quiet prefix must stop");
  const auto quiet_stamp = kSecond + 2 * kTenMilliseconds;
  const auto quiet_decision = estimator.decisionAt(quiet_stamp, 0.02);
  require(quiet_decision.has_value() && quiet_decision->stopped,
    "quiet historical decision must be present");

  for (int index = 3; index < 8; ++index) {
    auto noisy = quietSample(kSecond + index * kTenMilliseconds);
    noisy.gyro_z_rad_s = 1.0;
    noisy.acceleration[0] = (index % 2 == 0) ? 1.0 : -1.0;
    estimator.update(noisy);
  }
  const auto current = estimator.decisionAt(kSecond + 7 * kTenMilliseconds, 0.01);
  require(current.has_value() && !current->stopped, "noisy future must clear current stop state");
  const auto historical = estimator.decisionAt(quiet_stamp, 0.02);
  require(historical.has_value() && historical->stopped,
    "future IMU samples must not leak into an older timestamp query");
}

void testOutOfOrderUpdateCannotRewindHistory()
{
  StopStateEstimator estimator(testConfig());
  feedThreeQuiet(estimator, std::nullopt);
  const auto latest_stamp = kSecond + 3 * kTenMilliseconds;
  auto noisy = quietSample(latest_stamp);
  noisy.gyro_z_rad_s = 1.0;
  const auto latest = estimator.update(noisy);
  require(latest.accepted && !latest.stopped, "new noisy event must advance to moving");
  const auto history_size = estimator.decisionHistorySize();

  const auto late = estimator.update(quietSample(kSecond + kTenMilliseconds));
  require(!late.accepted, "out-of-order IMU update must be rejected");
  require(estimator.decisionHistorySize() == history_size,
    "rejected update must not mutate decision history");
  const auto current = estimator.decisionAt(latest_stamp, 0.01);
  require(current.has_value() && !current->stopped,
    "rejected old update must not rewind the live decision");
  const auto historical = estimator.decisionAt(kSecond + 2 * kTenMilliseconds, 0.01);
  require(historical.has_value() && historical->stopped,
    "historical query must still return its original decision");
  require(!estimator.decisionAt(latest_stamp + 200000000LL, 0.1).has_value(),
    "stale history query must fail closed");
}

void testHistoryIsBounded()
{
  Config config = testConfig();
  config.max_imu_history_size = 4;
  config.max_decision_history_size = 5;
  StopStateEstimator estimator(config);
  for (int index = 0; index < 20; ++index) {
    estimator.update(quietSample(kSecond + index * kTenMilliseconds));
  }
  require(estimator.imuHistorySize() <= 4, "IMU history must obey its hard size bound");
  require(estimator.decisionHistorySize() <= 5, "decision history must obey its hard size bound");
}

void testHoldAdvancesOnlyOnMonotonicImuEvents()
{
  Config config = testConfig();
  config.min_imu_samples = 2;
  config.hold_sec = 0.2;
  StopStateEstimator estimator(config);
  estimator.update(quietSample(kSecond));
  const auto candidate_start = estimator.update(quietSample(kSecond + kTenMilliseconds));
  require(!candidate_start.stopped, "hold must not pass when quiet candidate starts");
  estimator.update(quietSample(kSecond + 100000000LL));
  const auto future_query = estimator.decisionAt(kSecond + 500000000LL, 1.0);
  require(future_query.has_value() && !future_query->stopped,
    "a read-only future query must not advance the hold timer");
  const auto before_hold = estimator.update(quietSample(kSecond + 200000000LL));
  require(!before_hold.stopped, "hold must remain false before the full event-time duration");
  const auto after_hold = estimator.update(quietSample(kSecond + 210000000LL));
  require(after_hold.stopped, "monotonic IMU event must pass hold after the configured duration");
}

void testImuGapClearsLatchAndHold()
{
  Config config = testConfig();
  config.max_imu_gap_sec = 0.05;
  StopStateEstimator estimator(config);
  require(feedThreeQuiet(estimator, std::nullopt), "quiet prefix must establish stopped state");
  const auto after_gap = estimator.update(quietSample(kSecond + 200000000LL));
  require(after_gap.accepted && !after_gap.stopped,
    "IMU discontinuity must clear the stop latch and restart sample qualification");
  require(after_gap.imu_sample_count == 1,
    "IMU discontinuity must discard the pre-gap quiet window");
}

}  // namespace

int main()
{
  testRequiredTruthTable();
  testFutureAndStaleSpeedAreRejected();
  testQuietPastIsNotChangedByNoisyFuture();
  testOutOfOrderUpdateCannotRewindHistory();
  testHistoryIsBounded();
  testHoldAdvancesOnlyOnMonotonicImuEvents();
  testImuGapClearsLatchAndHold();
  std::cout << "PASS test_stop_state_estimator\n";
  return 0;
}
