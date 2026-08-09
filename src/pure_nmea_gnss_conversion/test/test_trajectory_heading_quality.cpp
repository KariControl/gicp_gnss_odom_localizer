// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "pure_nmea_gga_conversion/trajectory_heading_quality.hpp"

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
  using pure_nmea_gga_conversion::trajectoryHeadingPointIsUsable;
  using pure_nmea_gga_conversion::trajectoryHeadingTurnActivityIsUsable;
  using pure_nmea_gga_conversion::trajectoryHeadingVariance;
  using pure_nmea_gga_conversion::trajectoryHeadingVarianceIsUsable;
  using pure_nmea_gga_conversion::evaluateTrajectoryHeadingContinuity;
  using pure_nmea_gga_conversion::evaluateTrajectoryHeadingSeedGate;
  using pure_nmea_gga_conversion::TrajectoryHeadingHistory;
  using pure_nmea_gga_conversion::TrajectoryHeadingSample;

  require(
    trajectoryHeadingPointIsUsable(0.3, 5.0, 0.3, 5.0),
    "quality thresholds must be inclusive");
  require(
    !trajectoryHeadingPointIsUsable(0.29, 0.05, 0.3, 5.0),
    "low-confidence point must be rejected");
  require(
    !trajectoryHeadingPointIsUsable(1.0, 1000.0, 0.3, 5.0),
    "high-confidence point with inflated sigma must be rejected");
  require(
    !trajectoryHeadingPointIsUsable(
      1.0, std::numeric_limits<double>::infinity(), 0.3, 5.0),
    "non-finite sigma must be rejected");

  const double floor_rad = 3.0 * 3.14159265358979323846 / 180.0;
  const double precise_variance = trajectoryHeadingVariance(0.05, 0.05, 3.0, floor_rad);
  requireNear(
    precise_variance, floor_rad * floor_rad, 1.0e-15,
    "precise RTK chord should use the yaw-sigma floor");
  require(
    trajectoryHeadingVarianceIsUsable(precise_variance, 10.0),
    "precise RTK chord must pass the variance gate");

  const double inflated_variance = trajectoryHeadingVariance(1000.0, 0.05, 3.0, floor_rad);
  require(
    std::isfinite(inflated_variance) && inflated_variance > 10.0,
    "inflated reference sigma must produce a large yaw variance");
  require(
    !trajectoryHeadingVarianceIsUsable(inflated_variance, 10.0),
    "large yaw variance must not become a valid heading seed");

  double stored_seed_variance = precise_variance;
  if (trajectoryHeadingVarianceIsUsable(inflated_variance, 10.0)) {
    stored_seed_variance = inflated_variance;
  }
  requireNear(
    stored_seed_variance, precise_variance, 1.0e-15,
    "rejected heading must not overwrite a good seed");

  require(
    trajectoryHeadingVarianceIsUsable(10.0, 10.0),
    "maximum yaw variance threshold must be inclusive");
  require(
    !trajectoryHeadingVarianceIsUsable(
      std::numeric_limits<double>::infinity(), 10.0),
    "non-finite yaw variance must be rejected");
  require(
    trajectoryHeadingTurnActivityIsUsable(0.5, 0.5),
    "turn-activity threshold must be inclusive");
  require(
    !trajectoryHeadingTurnActivityIsUsable(0.5001, 0.5),
    "excessive turn activity must reject a trajectory chord");
  require(
    !trajectoryHeadingTurnActivityIsUsable(
      std::numeric_limits<double>::quiet_NaN(), 0.5),
    "non-finite turn activity must fail closed");

  {
    const auto small = evaluateTrajectoryHeadingContinuity(1.02, 1.0, 0.07);
    require(small.valid && small.accept_candidate && !small.restart_segment,
      "a small seed innovation must accept the trajectory candidate");
    requireNear(small.innovation_rad, 0.02, 1.0e-12,
      "small seed innovation value");

    const auto boundary = evaluateTrajectoryHeadingContinuity(1.07, 1.0, 0.07);
    require(boundary.valid && boundary.accept_candidate,
      "the hard innovation limit must be inclusive");

    const auto large = evaluateTrajectoryHeadingContinuity(1.093, 1.0, 0.07);
    require(
      large.valid && !large.accept_candidate && large.restart_segment &&
      large.preserve_heading_seed && large.reason == "seed_innovation_exceeded",
      "a large seed innovation must reject and request a seed-preserving restart");

    const auto wrap = evaluateTrajectoryHeadingContinuity(
      -3.14159265358979323846 + 0.02,
      3.14159265358979323846 - 0.02,
      0.07);
    require(wrap.valid && wrap.accept_candidate,
      "continuity comparison must use the wrapped SO(2) innovation");
    requireNear(wrap.innovation_rad, 0.04, 1.0e-12,
      "wrapped seed innovation value");

    const auto invalid = evaluateTrajectoryHeadingContinuity(
      1.0, 1.0, std::numeric_limits<double>::quiet_NaN());
    require(!invalid.valid && !invalid.accept_candidate,
      "invalid gate inputs must fail closed");

    const auto within_window_gap = evaluateTrajectoryHeadingSeedGate(5.0, 15.0, false);
    require(
      within_window_gap.valid && within_window_gap.reject_candidate &&
      !within_window_gap.allow_fresh_bootstrap &&
      within_window_gap.reason == "seed_propagation_unavailable",
      "an in-window corrected-IMU coverage gap must remain fail-closed");

    const auto stale_seed = evaluateTrajectoryHeadingSeedGate(15.1, 15.0, false);
    require(
      stale_seed.valid && !stale_seed.reject_candidate &&
      stale_seed.allow_fresh_bootstrap && !stale_seed.compare_candidate &&
      stale_seed.reason == "trusted_seed_stale_rebootstrap",
      "an explicitly expired seed must allow a fresh quality-gated bootstrap");

    const auto negative_age = evaluateTrajectoryHeadingSeedGate(-0.1, 15.0, true);
    require(!negative_age.valid && negative_age.reject_candidate,
      "a negative seed age must fail closed");
  }

  {
    TrajectoryHeadingHistory history;
    history.configure(5.0, 0.5, 2.0);
    // All points stay in one epoch. The 5 s history is deliberately longer
    // than the 2 s current-heading reference window.
    for (int i = 0; i <= 6; ++i) {
      const double stamp = 0.5 * static_cast<double>(i);
      const auto observed = history.observe(
        TrajectoryHeadingSample{stamp, stamp, 0.0, 0.05, 1.0}, true);
      require(!observed.epoch_reset, "continuous trusted fixes must stay in one epoch");
    }
    const auto selected = history.selectReference(
      TrajectoryHeadingSample{3.0, 3.0, 0.0, 0.05, 1.0}, 1.5, 2.0);
    require(selected.valid, "recent continuous history should provide a reference");
    requireNear(selected.reference_age_sec, 2.0, 1.0e-12,
      "selector should retain the longest precise baseline inside the recent window");
    requireNear(selected.baseline_m, 2.0, 1.0e-12,
      "selected recent baseline");
  }

  {
    TrajectoryHeadingHistory history;
    history.configure(5.0, 5.0, 2.0);
    history.observe({0.0, 0.0, 0.0, 0.05, 1.0}, true);
    const auto selected = history.selectReference(
      TrajectoryHeadingSample{3.0, 3.0, 0.0, 0.05, 1.0}, 1.5, 2.0);
    require(!selected.valid && selected.reason == "no_reference_within_max_age",
      "a retained old point must be ineligible, not treated as a current tangent");
    require(history.size() == 1U,
      "reference-age rejection must not erase an otherwise trusted history point");
  }

  {
    TrajectoryHeadingHistory history;
    history.configure(5.0, 0.5, 2.0);
    const std::vector<TrajectoryHeadingSample> samples{
      {0.0, 0.0, 0.0, 1.0, 0.4},
      {0.5, 0.5, 0.0, 0.05, 1.0},
      {1.0, 1.0, 0.0, 1.0, 0.4},
      {1.5, 1.5, 0.0, 1.0, 0.4},
      {2.0, 2.0, 0.0, 0.05, 1.0}};
    for (const auto & sample : samples) {
      history.observe(sample, true);
    }
    const auto selected = history.selectReference(samples.back(), 1.0, 2.0);
    require(selected.valid, "quality-ranked selector should find a reference");
    requireNear(selected.reference.stamp_sec, 0.5, 1.0e-12,
      "better-quality reference should beat a slightly longer noisy baseline");
  }

  {
    TrajectoryHeadingHistory history;
    history.configure(5.0, 0.5, 2.0);
    history.observe({0.0, 0.0, 0.0, 0.05, 1.0}, true);
    history.observe({0.2, 0.2, 0.0, 1000.0, 0.0}, false);
    const auto recovered = history.observe({0.4, 0.4, 0.0, 0.05, 1.0}, true);
    require(!recovered.epoch_reset && history.epoch() == 0U,
      "a brief unusable-fix flicker must not split the trusted epoch");

    history.observe({0.6, 0.6, 0.0, 1000.0, 0.0}, false);
    const auto sustained = history.observe({1.2, 1.2, 0.0, 1000.0, 0.0}, false);
    require(sustained.epoch_reset && sustained.epoch_reset_reason == "unusable_gap",
      "a sustained unusable interval must split the trusted epoch");
    require(history.size() == 0U,
      "pre-gap trajectory points must not survive an epoch reset");
  }

  {
    TrajectoryHeadingHistory history;
    history.configure(5.0, 0.5, 2.0);
    history.observe({0.0, 0.0, 0.0, 0.05, 1.0}, true);
    const auto no_fix = history.observe(
      {0.1, 0.0, 0.0, 1000.0, 0.0}, false, true);
    require(no_fix.epoch_reset && no_fix.epoch_reset_reason == "hard_unusable",
      "an explicit no-fix must invalidate the trajectory epoch immediately");
    require(!no_fix.preserve_heading_seed,
      "a no-fix epoch reset must not preserve a trajectory heading seed");
    const auto repeated_no_fix = history.observe(
      {0.2, 0.0, 0.0, 1000.0, 0.0}, false, true);
    require(!repeated_no_fix.epoch_reset,
      "one continuous no-fix run must not create repeated empty epochs");
  }

  {
    TrajectoryHeadingHistory history;
    history.configure(5.0, 0.5, 2.0);
    history.observe({0.0, 0.0, 0.0, 0.05, 1.0}, true);
    history.observe({1.0, 1.0, 0.0, 0.05, 1.0}, true);
    history.observe({2.0, 2.0, 0.0, 0.05, 1.0}, true);

    const auto restart = history.restartSegmentAt(
      {2.0, 2.0, 0.0, 0.05, 1.0}, "seed_innovation_exceeded");
    require(
      restart.epoch_reset && restart.inserted && restart.preserve_heading_seed &&
      restart.epoch_reset_reason == "seed_innovation_exceeded",
      "a geometry restart must preserve the trusted seed and start a new epoch");
    require(history.size() == 1U && history.epoch() == 1U,
      "a geometry restart must retain only the current point");

    const auto no_old_baseline = history.selectReference(
      {2.0, 2.0, 0.0, 0.05, 1.0}, 1.5, 5.0);
    require(!no_old_baseline.valid,
      "the rejected segment must not be immediately reusable");

    history.observe({3.0, 3.0, 0.0, 0.05, 1.0}, true);
    const auto still_short = history.selectReference(
      {3.0, 3.0, 0.0, 0.05, 1.0}, 1.5, 5.0);
    require(!still_short.valid,
      "trajectory heading must remain unavailable before a fresh baseline");

    history.observe({4.0, 4.0, 0.0, 0.05, 1.0}, true);
    const auto fresh = history.selectReference(
      {4.0, 4.0, 0.0, 0.05, 1.0}, 1.5, 5.0);
    require(fresh.valid && fresh.reference.stamp_sec == 2.0,
      "trajectory heading may return only after the new segment forms a baseline");
  }

  {
    TrajectoryHeadingHistory history;
    history.configure(5.0, 0.5, 2.0);
    const auto restart = history.restartSegmentAt(
      {0.0, 0.0, 0.0, 0.05, 1.0}, "turn_activity_exceeded");
    require(restart.epoch_reset && restart.preserve_heading_seed,
      "a turn restart must preserve the trusted seed");

    // Stay stationary long enough for the preserved seed to expire, then move
    // slowly. The rolling history must still form a fresh 1.5 m baseline, and
    // the expired seed must not permanently block its acceptance.
    for (int i = 1; i <= 16; ++i) {
      history.observe(
        {static_cast<double>(i), 0.0, 0.0, 0.05, 1.0}, true);
    }
    history.observe({17.0, 0.5, 0.0, 0.05, 1.0}, true);
    history.observe({18.0, 1.0, 0.0, 0.05, 1.0}, true);
    history.observe({19.0, 1.5, 0.0, 0.05, 1.0}, true);
    const auto fresh = history.selectReference(
      {19.0, 1.5, 0.0, 0.05, 1.0}, 1.5, 5.0);
    require(fresh.valid && fresh.reference_age_sec <= 5.0,
      "low-speed motion after a long stop must form a fresh bounded-age baseline");
    const auto stale_gate = evaluateTrajectoryHeadingSeedGate(19.0, 15.0, false);
    require(stale_gate.allow_fresh_bootstrap && !stale_gate.reject_candidate,
      "the fresh low-speed chord must rebootstrap after the old seed expires");
  }

  {
    TrajectoryHeadingHistory history;
    history.configure(5.0, 0.5, 2.0);
    history.observe({0.0, 0.0, 0.0, 0.05, 1.0}, true);
    const auto gap = history.observe({2.1, 2.1, 0.0, 0.05, 1.0}, true);
    require(gap.epoch_reset && gap.epoch_reset_reason == "input_gap",
      "a primary-input time gap must start a new epoch");
    require(history.size() == 1U,
      "the first post-gap trusted point must seed only the new epoch");

    const auto regression = history.observe({0.9, 0.9, 0.0, 0.05, 1.0}, true);
    require(regression.epoch_reset && regression.epoch_reset_reason == "time_not_increasing",
      "non-increasing timestamps must start a new epoch");
    require(history.size() == 1U,
      "a regressed point must not form a chord with the prior epoch");
  }

  {
    TrajectoryHeadingHistory history;
    history.configure(5.0, 0.5, 2.0);
    // A normal 1 Hz receiver must not look like a sustained unusable period.
    for (int i = 0; i <= 4; ++i) {
      const double stamp = static_cast<double>(i);
      const auto observed = history.observe(
        {stamp, 0.5 * stamp, 0.0, 0.05, 1.0}, true);
      require(!observed.epoch_reset,
        "continuous 1 Hz trusted fixes must stay in one epoch");
    }
    const auto low_speed = history.selectReference(
      {4.0, 2.0, 0.0, 0.05, 1.0}, 1.5, 5.0);
    require(low_speed.valid && low_speed.reference_age_sec >= 3.0,
      "low-speed 1 Hz motion must retain a sufficiently long heading baseline");

    TrajectoryHeadingHistory boundary;
    boundary.configure(5.0, 0.5, 2.0);
    boundary.observe({0.0, 0.0, 0.0, 0.05, 1.0}, true);
    const auto inclusive = boundary.observe({2.0, 1.0, 0.0, 0.05, 1.0}, true);
    require(!inclusive.epoch_reset,
      "input gap exactly at the configured maximum must be accepted");
  }

  std::cout << "PASS test_trajectory_heading_quality\n";
  return EXIT_SUCCESS;
}
