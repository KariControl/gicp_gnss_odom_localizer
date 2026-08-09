// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "pure_gnss_map_odom_fusion/gnss_recovery_controller.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

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

std::vector<pure_gnss_map_odom_fusion::RecoveryAlignmentSample> makeTrajectory(
  double tx, double ty, double yaw, std::size_t count)
{
  std::vector<pure_gnss_map_odom_fusion::RecoveryAlignmentSample> samples;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  for (std::size_t i = 0; i < count; ++i) {
    const double x = static_cast<double>(i);
    const double y = 0.15 * static_cast<double>(i * i);
    pure_gnss_map_odom_fusion::RecoveryAlignmentSample sample;
    sample.stamp_sec = 0.2 * static_cast<double>(i);
    sample.odom_x_m = x;
    sample.odom_y_m = y;
    sample.map_x_m = tx + c * x - s * y;
    sample.map_y_m = ty + s * x + c * y;
    sample.position_weight = 4.0;
    samples.push_back(sample);
  }
  return samples;
}
}  // namespace

int main()
{
  using namespace pure_gnss_map_odom_fusion;

  RecoveryAlignmentConfig config;
  config.min_samples = 5;
  config.min_heading_samples = 2;
  config.max_sample_gap_sec = 0.5;
  config.min_odom_baseline_m = 3.0;
  config.max_position_rms_m = 0.2;
  config.max_position_residual_m = 0.5;

  FixedYawTranslationConfig fixed_yaw_config;
  fixed_yaw_config.min_samples = 5;
  fixed_yaw_config.max_sample_gap_sec = 0.5;
  fixed_yaw_config.max_position_rms_m = 0.2;
  fixed_yaw_config.max_position_residual_m = 0.5;

  {
    const auto samples = makeTrajectory(12.0, -3.5, 0.25, 7);
    const auto result = estimateRecoveryAlignmentRobust(samples, config);
    require(result.valid, "position-only trajectory alignment should be valid");
    require(result.used_position_heading, "trajectory must provide yaw observability");
    requireNear(result.tx_m, 12.0, 1.0e-9, "trajectory tx");
    requireNear(result.ty_m, -3.5, 1.0e-9, "trajectory ty");
    requireNear(wrapAngle(result.yaw_rad - 0.25), 0.0, 1.0e-9, "trajectory yaw");
  }

  {
    auto samples = makeTrajectory(4.0, 2.0, -0.4, 7);
    samples[3].map_x_m += 20.0;
    samples[3].map_y_m -= 15.0;
    const auto strict = estimateRecoveryAlignment(samples, config);
    require(!strict.valid, "full window with a gross outlier must fail");
    const auto robust = estimateRecoveryAlignmentRobust(samples, config);
    require(robust.valid, "single isolated outlier should be removable");
    require(robust.rejected_sample_count == 1U && robust.rejected_sample_index == 3U,
      "robust alignment must identify the isolated sample");
    requireNear(robust.tx_m, 4.0, 1.0e-9, "robust tx");
    requireNear(robust.ty_m, 2.0, 1.0e-9, "robust ty");
  }

  {
    auto samples = makeTrajectory(4.0, 2.0, -0.4, 7);
    samples.back().map_x_m += 20.0;
    samples.back().map_y_m -= 15.0;
    const auto robust = estimateRecoveryAlignmentRobust(samples, config);
    require(robust.valid, "a newest isolated outlier should be identifiable");
    require(newestSampleWasRejected(robust),
      "reacquisition must be able to distinguish a rejected newest sample");
  }

  {
    std::vector<RecoveryAlignmentSample> samples;
    for (std::size_t i = 0; i < 5; ++i) {
      RecoveryAlignmentSample sample;
      sample.stamp_sec = 0.2 * static_cast<double>(i);
      sample.odom_x_m = 1.0;
      sample.odom_y_m = -2.0;
      sample.map_x_m = 8.0;
      sample.map_y_m = 5.0;
      sample.has_heading = true;
      sample.map_odom_yaw_rad = 0.7 + (i == 4 ? 0.01 : 0.0);
      sample.heading_weight = 10.0;
      samples.push_back(sample);
    }
    const auto result = estimateRecoveryAlignmentRobust(samples, config);
    require(result.valid && result.used_direct_heading,
      "direct heading should initialize while stationary");
    requireNear(wrapAngle(result.yaw_rad - 0.702), 0.0, 0.01, "stationary heading");
  }

  {
    auto samples = makeTrajectory(0.0, 0.0, 0.0, 6);
    for (auto & sample : samples) {
      sample.has_heading = true;
      sample.map_odom_yaw_rad = 1.2;
      sample.heading_weight = 10.0;
    }
    const auto result = estimateRecoveryAlignmentRobust(samples, config);
    require(!result.valid && result.reason == "heading_sources_disagree",
      "trajectory and direct heading sources must agree before recovery");
  }

  {
    auto samples = makeTrajectory(0.0, 0.0, 0.0, 6);
    for (auto & sample : samples) {
      sample.has_heading = true;
      sample.map_odom_yaw_rad = 1.2;
      sample.heading_weight = 10.0;
    }
    const auto result = estimateRecoveryAlignmentRobust(samples, config);
    require(!result.valid && result.reason == "heading_sources_disagree",
      "trajectory and direct heading disagreement must fail closed");
  }

  {
    auto samples = makeTrajectory(0.0, 0.0, 0.0, 6);
    samples[3].stamp_sec += 1.0;
    const auto result = estimateRecoveryAlignmentRobust(samples, config);
    require(!result.valid, "non-contiguous sample window must fail");
  }

  {
    const auto samples = makeTrajectory(2.0, -1.0, 0.3, 4);
    const auto result = estimateFixedYawTranslationRobust(
      samples, 0.3, fixed_yaw_config);
    require(!result.valid && result.reason == "not_enough_samples",
      "fixed-yaw recovery must reject too few samples");
  }

  {
    auto samples = makeTrajectory(6.0, -2.5, 0.35, 7);
    samples[2].map_x_m += 8.0;
    samples[2].map_y_m -= 6.0;
    const auto strict = estimateFixedYawTranslation(samples, 0.35, fixed_yaw_config);
    require(!strict.valid, "fixed-yaw full window with one gross outlier must fail");
    const auto robust = estimateFixedYawTranslationRobust(
      samples, 0.35, fixed_yaw_config);
    require(robust.valid, "fixed-yaw recovery should remove one isolated outlier");
    require(robust.rejected_sample_count == 1U && robust.rejected_sample_index == 2U,
      "fixed-yaw recovery must identify the isolated position sample");
    requireNear(robust.tx_m, 6.0, 1.0e-9, "fixed-yaw robust tx");
    requireNear(robust.ty_m, -2.5, 1.0e-9, "fixed-yaw robust ty");
    requireNear(wrapAngle(robust.yaw_rad - 0.35), 0.0, 1.0e-12,
      "fixed-yaw recovery must preserve yaw");
  }

  {
    auto samples = makeTrajectory(1.0, 2.0, -0.2, 7);
    for (std::size_t i = 0; i < samples.size(); ++i) {
      if (i % 2U == 0U) {
        samples[i].map_x_m += 4.0;
      } else {
        samples[i].map_y_m -= 4.0;
      }
    }
    const auto result = estimateFixedYawTranslationRobust(
      samples, -0.2, fixed_yaw_config);
    require(!result.valid,
      "fixed-yaw recovery must fail closed when more than one sample is inconsistent");
  }

  {
    std::vector<RecoveryAlignmentSample> samples;
    const double fixed_yaw = 0.8;
    const double c = std::cos(fixed_yaw);
    const double s = std::sin(fixed_yaw);
    const double odom_x = 1.25;
    const double odom_y = -0.75;
    for (std::size_t i = 0; i < 6; ++i) {
      RecoveryAlignmentSample sample;
      sample.stamp_sec = 0.2 * static_cast<double>(i);
      sample.odom_x_m = odom_x;
      sample.odom_y_m = odom_y;
      sample.map_x_m = 9.0 + c * odom_x - s * odom_y;
      sample.map_y_m = -4.0 + s * odom_x + c * odom_y;
      sample.position_weight = 5.0 + static_cast<double>(i);
      samples.push_back(sample);
    }
    const auto translation = estimateFixedYawTranslationRobust(
      samples, fixed_yaw, fixed_yaw_config);
    require(translation.valid, "fixed-yaw translation must be observable while stationary");
    requireNear(translation.tx_m, 9.0, 1.0e-9, "stationary fixed-yaw tx");
    requireNear(translation.ty_m, -4.0, 1.0e-9, "stationary fixed-yaw ty");
    requireNear(wrapAngle(translation.yaw_rad - fixed_yaw), 0.0, 1.0e-12,
      "stationary fixed-yaw result must preserve the supplied yaw");

    const auto se2 = estimateRecoveryAlignmentRobust(samples, config);
    require(!se2.valid && se2.reason == "yaw_unobservable",
      "existing SE2 estimator must still reject stationary position-only yaw");
  }

  {
    const auto samples = makeTrajectory(-3.0, 5.0, -0.45, 7);
    const auto before = estimateRecoveryAlignmentRobust(samples, config);
    require(before.valid && before.used_position_heading,
      "existing moving SE2 recovery must remain valid");
    requireNear(before.tx_m, -3.0, 1.0e-9, "existing SE2 tx non-regression");
    requireNear(before.ty_m, 5.0, 1.0e-9, "existing SE2 ty non-regression");
    requireNear(wrapAngle(before.yaw_rad + 0.45), 0.0, 1.0e-9,
      "existing SE2 yaw non-regression");
  }

  {
    const auto step = applyBoundedCorrection(
      0.0, 0.0, 0.0, 3.0, 4.0, kPi / 2.0, 0.5, 0.1);
    requireNear(step.applied_translation_m, 0.5, 1.0e-12, "bounded translation");
    requireNear(step.applied_yaw_rad, 0.1, 1.0e-12, "bounded yaw");
  }

  {
    // A yaw correction about the odom origin has a large lever arm here. The
    // legacy component-wise limiter would move this base pose by roughly
    // 0.55 m for a 0.01 rad yaw step even with zero translation correction.
    const double odom_base_x = 55.0;
    const double odom_base_y = 0.0;
    const auto initial = basePivotResidual(
      0.0, 0.0, 0.0, 10.0, -2.0, 0.20, odom_base_x, odom_base_y);
    require(initial.valid && initial.position_m > 10.0,
      "base-space gate must include the yaw lever about the odom origin");

    const auto step = applyBasePivotBoundedCorrection(
      0.0, 0.0, 0.0, 10.0, -2.0, 0.20,
      odom_base_x, odom_base_y, 0.05, 0.01);
    require(step.valid, "finite base-pivot correction must be valid");
    requireNear(step.applied_base_translation_m, 0.05, 1.0e-12,
      "visible base_link translation must obey its per-step bound");
    requireNear(step.applied_yaw_rad, 0.01, 1.0e-12,
      "base-pivot yaw must obey its per-step bound");
    const auto applied = basePivotResidual(
      0.0, 0.0, 0.0, step.tx_m, step.ty_m, step.yaw_rad,
      odom_base_x, odom_base_y);
    require(applied.valid, "reconstructed map->odom must remain finite");
    requireNear(applied.position_m, 0.05, 1.0e-12,
      "reconstructed transform must produce exactly the bounded base step");
    require(std::hypot(step.tx_m, step.ty_m) > 0.50,
      "map->odom translation may compensate yaw to protect base continuity");
  }

  {
    // A delayed measurement may have been synchronized near the odom origin
    // while the next published base pose is already far away. Bounding the
    // stale pivot does not protect that current consumer pose.
    const auto stale_pivot_step = applyBasePivotBoundedCorrection(
      0.0, 0.0, 0.0, 0.0, 0.0, 0.20,
      0.0, 0.0, 0.05, 0.01);
    const auto stale_step_at_latest_base = basePivotResidual(
      0.0, 0.0, 0.0,
      stale_pivot_step.tx_m, stale_pivot_step.ty_m, stale_pivot_step.yaw_rad,
      55.0, 0.0);
    require(stale_step_at_latest_base.valid && stale_step_at_latest_base.position_m > 0.50,
      "a stale measurement pivot must expose the delayed-yaw lever-arm risk");

    const auto latest_pivot_step = applyBasePivotBoundedCorrection(
      0.0, 0.0, 0.0, 0.0, 0.0, 0.20,
      55.0, 0.0, 0.05, 0.01);
    const auto latest_step_at_latest_base = basePivotResidual(
      0.0, 0.0, 0.0,
      latest_pivot_step.tx_m, latest_pivot_step.ty_m, latest_pivot_step.yaw_rad,
      55.0, 0.0);
    require(latest_pivot_step.valid && latest_step_at_latest_base.valid,
      "latest-pivot delayed measurement correction must remain valid");
    requireNear(latest_step_at_latest_base.position_m, 0.05, 1.0e-12,
      "continuity must be bounded at the latest published base pose");
  }

  {
    const auto step = applyBasePivotBoundedCorrection(
      1.0, -2.0, 0.30, 4.0, 5.0, -0.40,
      32.0, -17.0, 100.0, 1.0);
    require(step.valid && step.reached_target,
      "a reachable base-pivot target must complete");
    requireNear(step.tx_m, 4.0, 0.0, "completed recovery must use exact target tx");
    requireNear(step.ty_m, 5.0, 0.0, "completed recovery must use exact target ty");
    requireNear(step.yaw_rad, -0.40, 0.0, "completed recovery must use exact target yaw");
    requireNear(step.residual_base_translation_m, 0.0, 0.0,
      "completed recovery must leave no base position residue");
    requireNear(step.residual_yaw_rad, 0.0, 0.0,
      "completed recovery must leave no yaw residue");
  }

  {
    // The continuity pivot is base_link even when target estimation used an
    // antenna point. Passing the antenna point (base + lever arm) here would
    // protect a different consumer frame.
    const double odom_base_x = 20.0;
    const double odom_base_y = -4.0;
    const auto step = applyBasePivotBoundedCorrection(
      -3.0, 8.0, 0.2, 6.0, -1.0, 0.45,
      odom_base_x, odom_base_y, 0.10, 0.02);
    const auto applied_at_base = basePivotResidual(
      -3.0, 8.0, 0.2, step.tx_m, step.ty_m, step.yaw_rad,
      odom_base_x, odom_base_y);
    require(step.valid && applied_at_base.valid,
      "base pivot must remain valid with a GNSS lever-arm target");
    requireNear(applied_at_base.position_m, 0.10, 1.0e-12,
      "GNSS lever-arm semantics must not change the protected base frame");
    const auto applied_at_antenna = basePivotResidual(
      -3.0, 8.0, 0.2, step.tx_m, step.ty_m, step.yaw_rad,
      odom_base_x + 1.5, odom_base_y + 0.5);
    require(applied_at_antenna.valid &&
      std::fabs(applied_at_antenna.position_m - applied_at_base.position_m) > 1.0e-4,
      "the antenna lever arm must not be silently substituted as the continuity pivot");
  }

  {
    const auto non_finite = applyBasePivotBoundedCorrection(
      0.0, 0.0, 0.0, 1.0, 2.0,
      std::numeric_limits<double>::quiet_NaN(), 3.0, 4.0, 0.1, 0.1);
    require(!non_finite.valid, "non-finite recovery geometry must fail closed");
    const auto negative_limit = applyBasePivotBoundedCorrection(
      0.0, 0.0, 0.0, 1.0, 2.0, 0.3, 3.0, 4.0, -0.1, 0.1);
    require(!negative_limit.valid, "negative recovery limits must fail closed");
  }

  {
    const auto first = incrementalRecoveryCovariance(
      true, 12.0, 0.3, 0.4, 8.0, 2.0, 9.0, 3.0);
    requireNear(first.xy_variance, 0.0, 0.0,
      "first recovery step must not duplicate accumulated XY covariance");
    requireNear(first.yaw_variance, 0.0, 0.0,
      "first recovery step must not duplicate accumulated yaw covariance");

    const auto later = incrementalRecoveryCovariance(
      false, 0.25, 0.3, 0.4, 2.6, 2.0, 3.8, 3.0);
    requireNear(later.xy_variance, 0.675, 1.0e-12,
      "later recovery step must add only incremental XY covariance");
    requireNear(later.yaw_variance, 0.9, 1.0e-12,
      "later recovery step must add only incremental yaw covariance");
  }

  {
    require(latestPastSampleCoversTimestamp(10.0, 9.9, 0.1),
      "a past odometry sample exactly at the age limit must cover the query");
    require(!latestPastSampleCoversTimestamp(10.0, 9.899, 0.1),
      "a stale past odometry sample must not be hidden by future buffer data");
    require(!latestPastSampleCoversTimestamp(10.0, 10.001, 0.1),
      "a future odometry sample cannot cover a past measurement timestamp");
    require(!latestPastSampleCoversTimestamp(
        std::numeric_limits<double>::quiet_NaN(), 9.9, 0.1),
      "non-finite timestamps must fail closed");
  }

  {
    const auto at_reference = positionYawJacobian(0.0, 0.0, 1.2);
    requireNear(at_reference.dx_dyaw, 0.0, 0.0,
      "yaw uncertainty must not inflate XY at its GNSS reference");
    requireNear(at_reference.dy_dyaw, 0.0, 0.0,
      "yaw uncertainty must not inflate XY at its GNSS reference");

    const auto displaced = positionYawJacobian(3.0, 4.0, 0.7);
    requireNear(
      std::hypot(displaced.dx_dyaw, displaced.dy_dyaw), 5.0, 1.0e-12,
      "position/yaw Jacobian norm must equal distance from the XY reference");
  }

  std::cout << "PASS test_gnss_recovery_controller\n";
  return EXIT_SUCCESS;
}
