// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace pure_gnss_map_odom_fusion
{

enum class GnssRecoveryState
{
  UNINITIALIZED,
  TRACKING,
  OUTAGE,
  REACQUIRING,
  RECOVERING_XY_ONLY,
  TRACKING_XY_ONLY,
  RECOVERING,
};

inline const char * toString(GnssRecoveryState state)
{
  switch (state) {
    case GnssRecoveryState::UNINITIALIZED:
      return "uninitialized";
    case GnssRecoveryState::TRACKING:
      return "tracking";
    case GnssRecoveryState::OUTAGE:
      return "outage";
    case GnssRecoveryState::REACQUIRING:
      return "reacquiring";
    case GnssRecoveryState::RECOVERING_XY_ONLY:
      return "recovering_xy_only";
    case GnssRecoveryState::TRACKING_XY_ONLY:
      return "tracking_xy_only";
    case GnssRecoveryState::RECOVERING:
      return "recovering";
  }
  return "unknown";
}

inline double wrapAngle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

struct RecoveryAlignmentSample
{
  double stamp_sec{0.0};
  double odom_x_m{0.0};
  double odom_y_m{0.0};
  double map_x_m{0.0};
  double map_y_m{0.0};
  double position_weight{1.0};
  bool has_heading{false};
  // Direct map->odom yaw candidate: measured map yaw - odom base yaw.
  double map_odom_yaw_rad{0.0};
  double heading_weight{1.0};
};

struct RecoveryAlignmentConfig
{
  std::size_t min_samples{5};
  std::size_t min_heading_samples{2};
  double max_sample_gap_sec{1.0};
  double min_odom_baseline_m{3.0};
  double max_position_rms_m{1.5};
  double max_position_residual_m{3.0};
  double max_heading_std_rad{0.35};
  double max_heading_source_disagreement_rad{0.52};
  bool allow_single_outlier_rejection{true};
};

struct RecoveryAlignmentResult
{
  bool valid{false};
  double tx_m{0.0};
  double ty_m{0.0};
  double yaw_rad{0.0};
  double position_rms_m{std::numeric_limits<double>::infinity()};
  double max_position_residual_m{std::numeric_limits<double>::infinity()};
  double odom_baseline_m{0.0};
  double heading_std_rad{std::numeric_limits<double>::infinity()};
  double heading_source_disagreement_rad{0.0};
  std::size_t sample_count{0};
  std::size_t heading_sample_count{0};
  bool used_position_heading{false};
  bool used_direct_heading{false};
  std::size_t rejected_sample_count{0};
  std::size_t rejected_sample_index{std::numeric_limits<std::size_t>::max()};
  std::string reason{"not_evaluated"};
};

inline bool newestSampleWasRejected(const RecoveryAlignmentResult & result)
{
  return result.rejected_sample_count > 0U && result.sample_count > 0U &&
         result.rejected_sample_index + 1U == result.sample_count;
}

// Weighted planar rigid alignment. It can recover yaw from vehicle motion even
// when the GNSS message is position-only, or use independent heading samples
// when the vehicle is stationary. The output is intentionally rejected unless
// the whole candidate window is temporally contiguous and self-consistent.
inline RecoveryAlignmentResult estimateRecoveryAlignment(
  const std::vector<RecoveryAlignmentSample> & samples,
  const RecoveryAlignmentConfig & config)
{
  RecoveryAlignmentResult result;
  result.sample_count = samples.size();

  if (samples.size() < std::max<std::size_t>(1, config.min_samples)) {
    result.reason = "not_enough_samples";
    return result;
  }

  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto & sample = samples[i];
    if (!std::isfinite(sample.stamp_sec) ||
      !std::isfinite(sample.odom_x_m) || !std::isfinite(sample.odom_y_m) ||
      !std::isfinite(sample.map_x_m) || !std::isfinite(sample.map_y_m) ||
      !std::isfinite(sample.position_weight) || sample.position_weight <= 0.0 ||
      (sample.has_heading &&
      (!std::isfinite(sample.map_odom_yaw_rad) ||
      !std::isfinite(sample.heading_weight) || sample.heading_weight <= 0.0)))
    {
      result.reason = "non_finite_sample";
      return result;
    }
    if (i > 0) {
      const double dt = sample.stamp_sec - samples[i - 1].stamp_sec;
      if (dt <= 0.0 || dt > config.max_sample_gap_sec) {
        result.reason = "sample_gap";
        return result;
      }
    }
  }

  // Maximum displacement in odom is a simple, robust observability test for
  // position-only yaw. Windows are intentionally small, so O(N^2) is fine.
  for (std::size_t i = 0; i < samples.size(); ++i) {
    for (std::size_t j = i + 1; j < samples.size(); ++j) {
      result.odom_baseline_m = std::max(
        result.odom_baseline_m,
        std::hypot(
          samples[j].odom_x_m - samples[i].odom_x_m,
          samples[j].odom_y_m - samples[i].odom_y_m));
    }
  }

  double sum_w = 0.0;
  double mean_ox = 0.0;
  double mean_oy = 0.0;
  double mean_mx = 0.0;
  double mean_my = 0.0;
  for (const auto & sample : samples) {
    const double w = sample.position_weight;
    sum_w += w;
    mean_ox += w * sample.odom_x_m;
    mean_oy += w * sample.odom_y_m;
    mean_mx += w * sample.map_x_m;
    mean_my += w * sample.map_y_m;
  }
  if (!(sum_w > 0.0) || !std::isfinite(sum_w)) {
    result.reason = "invalid_weight_sum";
    return result;
  }
  mean_ox /= sum_w;
  mean_oy /= sum_w;
  mean_mx /= sum_w;
  mean_my /= sum_w;

  double position_c = 0.0;
  double position_s = 0.0;
  for (const auto & sample : samples) {
    const double ox = sample.odom_x_m - mean_ox;
    const double oy = sample.odom_y_m - mean_oy;
    const double mx = sample.map_x_m - mean_mx;
    const double my = sample.map_y_m - mean_my;
    position_c += sample.position_weight * (ox * mx + oy * my);
    position_s += sample.position_weight * (ox * my - oy * mx);
  }
  const double position_signal = std::hypot(position_c, position_s);
  const bool position_yaw_observable =
    result.odom_baseline_m >= config.min_odom_baseline_m &&
    position_signal > 1.0e-9;
  const double position_yaw = std::atan2(position_s, position_c);

  double heading_c = 0.0;
  double heading_s = 0.0;
  double heading_weight_sum = 0.0;
  for (const auto & sample : samples) {
    if (!sample.has_heading) {
      continue;
    }
    ++result.heading_sample_count;
    heading_weight_sum += sample.heading_weight;
    heading_c += sample.heading_weight * std::cos(sample.map_odom_yaw_rad);
    heading_s += sample.heading_weight * std::sin(sample.map_odom_yaw_rad);
  }
  const double heading_resultant = std::hypot(heading_c, heading_s);
  const bool direct_yaw_observable =
    result.heading_sample_count >= std::max<std::size_t>(1, config.min_heading_samples) &&
    heading_weight_sum > 0.0 && heading_resultant > 1.0e-9;
  const double direct_yaw = std::atan2(heading_s, heading_c);
  if (direct_yaw_observable) {
    const double normalized_resultant = std::clamp(
      heading_resultant / heading_weight_sum, 1.0e-12, 1.0);
    result.heading_std_rad = std::sqrt(std::max(0.0, -2.0 * std::log(normalized_resultant)));
    if (result.heading_std_rad > config.max_heading_std_rad) {
      result.reason = "heading_inconsistent";
      return result;
    }
  }

  if (!position_yaw_observable && !direct_yaw_observable) {
    result.reason = "yaw_unobservable";
    return result;
  }

  if (position_yaw_observable && direct_yaw_observable) {
    result.heading_source_disagreement_rad = std::fabs(wrapAngle(
      position_yaw - direct_yaw));
    if (result.heading_source_disagreement_rad >
      config.max_heading_source_disagreement_rad)
    {
      result.reason = "heading_sources_disagree";
      return result;
    }

    // Circularly combine two independent yaw cues. Normalize the position
    // signal so a long trajectory does not numerically swamp heading weights.
    const double position_mix_weight = std::max(1.0, position_signal / sum_w);
    const double heading_mix_weight = std::max(1.0, heading_resultant);
    result.yaw_rad = std::atan2(
      position_mix_weight * std::sin(position_yaw) +
      heading_mix_weight * std::sin(direct_yaw),
      position_mix_weight * std::cos(position_yaw) +
      heading_mix_weight * std::cos(direct_yaw));
    result.used_position_heading = true;
    result.used_direct_heading = true;
  } else if (position_yaw_observable) {
    result.yaw_rad = position_yaw;
    result.used_position_heading = true;
  } else {
    result.yaw_rad = direct_yaw;
    result.used_direct_heading = true;
  }
  result.yaw_rad = wrapAngle(result.yaw_rad);

  const double c = std::cos(result.yaw_rad);
  const double s = std::sin(result.yaw_rad);
  result.tx_m = mean_mx - (c * mean_ox - s * mean_oy);
  result.ty_m = mean_my - (s * mean_ox + c * mean_oy);

  double weighted_error = 0.0;
  result.max_position_residual_m = 0.0;
  for (const auto & sample : samples) {
    const double px = result.tx_m + c * sample.odom_x_m - s * sample.odom_y_m;
    const double py = result.ty_m + s * sample.odom_x_m + c * sample.odom_y_m;
    const double ex = sample.map_x_m - px;
    const double ey = sample.map_y_m - py;
    const double residual = std::hypot(ex, ey);
    result.max_position_residual_m = std::max(
      result.max_position_residual_m, residual);
    weighted_error += sample.position_weight * residual * residual;
  }
  result.position_rms_m = std::sqrt(weighted_error / sum_w);
  if (!std::isfinite(result.position_rms_m) ||
    result.position_rms_m > config.max_position_rms_m)
  {
    result.reason = "position_rms_too_large";
    return result;
  }
  if (!std::isfinite(result.max_position_residual_m) ||
    result.max_position_residual_m > config.max_position_residual_m)
  {
    result.reason = "position_outlier";
    return result;
  }

  result.valid = true;
  result.reason = "ok";
  return result;
}


// Fail closed on a globally inconsistent window, but optionally tolerate one
// isolated GNSS/heading outlier. Every leave-one-out candidate still has to
// satisfy the full temporal, observability, RMS, maximum-residual and heading
// consistency gates; this is not a force-accept path.
inline RecoveryAlignmentResult estimateRecoveryAlignmentRobust(
  const std::vector<RecoveryAlignmentSample> & samples,
  const RecoveryAlignmentConfig & config)
{
  RecoveryAlignmentResult full = estimateRecoveryAlignment(samples, config);
  const bool outlier_eligible_failure =
    full.reason == "position_rms_too_large" ||
    full.reason == "position_outlier" ||
    full.reason == "heading_inconsistent" ||
    full.reason == "heading_sources_disagree";
  if (full.valid || !config.allow_single_outlier_rejection ||
    !outlier_eligible_failure ||
    samples.size() <= std::max<std::size_t>(1, config.min_samples))
  {
    return full;
  }

  bool found = false;
  RecoveryAlignmentResult best;
  double best_score = std::numeric_limits<double>::infinity();
  for (std::size_t rejected = 0; rejected < samples.size(); ++rejected) {
    std::vector<RecoveryAlignmentSample> subset;
    subset.reserve(samples.size() - 1U);
    for (std::size_t index = 0; index < samples.size(); ++index) {
      if (index != rejected) subset.push_back(samples[index]);
    }

    RecoveryAlignmentResult candidate = estimateRecoveryAlignment(subset, config);
    if (!candidate.valid) continue;
    const double heading_penalty = std::isfinite(candidate.heading_std_rad) ?
      candidate.heading_std_rad : 0.0;
    const double score = candidate.position_rms_m +
      0.1 * candidate.max_position_residual_m + 0.1 * heading_penalty;
    if (!found || score < best_score) {
      found = true;
      best_score = score;
      best = candidate;
      best.sample_count = samples.size();
      best.rejected_sample_count = 1U;
      best.rejected_sample_index = rejected;
      best.reason = "ok_after_single_outlier_rejection";
    }
  }

  return found ? best : full;
}

// Translation-only recovery deliberately keeps yaw fixed to an externally
// supplied prior. This is useful when GNSS position is good but neither motion
// nor an independent heading makes yaw observable. Callers must retain the yaw
// uncertainty and must not treat this result as a new absolute yaw observation.
struct FixedYawTranslationConfig
{
  std::size_t min_samples{5};
  double max_sample_gap_sec{1.0};
  double max_position_rms_m{1.5};
  double max_position_residual_m{3.0};
  bool allow_single_outlier_rejection{true};
};

struct FixedYawTranslationResult
{
  bool valid{false};
  double tx_m{0.0};
  double ty_m{0.0};
  double yaw_rad{0.0};
  double position_rms_m{std::numeric_limits<double>::infinity()};
  double max_position_residual_m{std::numeric_limits<double>::infinity()};
  std::size_t sample_count{0};
  std::size_t rejected_sample_count{0};
  std::size_t rejected_sample_index{std::numeric_limits<std::size_t>::max()};
  std::string reason{"not_evaluated"};
};

inline bool newestSampleWasRejected(const FixedYawTranslationResult & result)
{
  return result.rejected_sample_count > 0U && result.sample_count > 0U &&
         result.rejected_sample_index + 1U == result.sample_count;
}

// Estimate only the translation of map->odom for a fixed map->odom yaw. Each
// sample independently proposes t = p_map - R(fixed_yaw) * p_odom; their
// weighted mean is accepted only when the complete window is temporally
// contiguous and positionally self-consistent.
inline FixedYawTranslationResult estimateFixedYawTranslation(
  const std::vector<RecoveryAlignmentSample> & samples,
  double fixed_yaw_rad,
  const FixedYawTranslationConfig & config)
{
  FixedYawTranslationResult result;
  result.sample_count = samples.size();

  if (!std::isfinite(fixed_yaw_rad)) {
    result.reason = "non_finite_fixed_yaw";
    return result;
  }
  result.yaw_rad = wrapAngle(fixed_yaw_rad);

  if (samples.size() < std::max<std::size_t>(1, config.min_samples)) {
    result.reason = "not_enough_samples";
    return result;
  }

  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto & sample = samples[i];
    if (!std::isfinite(sample.stamp_sec) ||
      !std::isfinite(sample.odom_x_m) || !std::isfinite(sample.odom_y_m) ||
      !std::isfinite(sample.map_x_m) || !std::isfinite(sample.map_y_m) ||
      !std::isfinite(sample.position_weight) || sample.position_weight <= 0.0)
    {
      result.reason = "non_finite_sample";
      return result;
    }
    if (i > 0) {
      const double dt = sample.stamp_sec - samples[i - 1].stamp_sec;
      if (dt <= 0.0 || dt > config.max_sample_gap_sec) {
        result.reason = "sample_gap";
        return result;
      }
    }
  }

  const double c = std::cos(result.yaw_rad);
  const double s = std::sin(result.yaw_rad);
  double sum_w = 0.0;
  for (const auto & sample : samples) {
    const double proposed_tx = sample.map_x_m -
      (c * sample.odom_x_m - s * sample.odom_y_m);
    const double proposed_ty = sample.map_y_m -
      (s * sample.odom_x_m + c * sample.odom_y_m);
    sum_w += sample.position_weight;
    result.tx_m += sample.position_weight * proposed_tx;
    result.ty_m += sample.position_weight * proposed_ty;
  }
  if (!(sum_w > 0.0) || !std::isfinite(sum_w)) {
    result.reason = "invalid_weight_sum";
    return result;
  }
  result.tx_m /= sum_w;
  result.ty_m /= sum_w;
  if (!std::isfinite(result.tx_m) || !std::isfinite(result.ty_m)) {
    result.reason = "non_finite_translation";
    return result;
  }

  double weighted_error = 0.0;
  result.max_position_residual_m = 0.0;
  for (const auto & sample : samples) {
    const double px = result.tx_m + c * sample.odom_x_m - s * sample.odom_y_m;
    const double py = result.ty_m + s * sample.odom_x_m + c * sample.odom_y_m;
    const double residual = std::hypot(sample.map_x_m - px, sample.map_y_m - py);
    result.max_position_residual_m = std::max(
      result.max_position_residual_m, residual);
    weighted_error += sample.position_weight * residual * residual;
  }
  result.position_rms_m = std::sqrt(weighted_error / sum_w);
  if (!std::isfinite(result.position_rms_m) ||
    result.position_rms_m > config.max_position_rms_m)
  {
    result.reason = "position_rms_too_large";
    return result;
  }
  if (!std::isfinite(result.max_position_residual_m) ||
    result.max_position_residual_m > config.max_position_residual_m)
  {
    result.reason = "position_outlier";
    return result;
  }

  result.valid = true;
  result.reason = "ok";
  return result;
}

// Optionally remove one isolated position outlier. The remaining samples still
// have to pass the full count, temporal and residual checks at the same fixed
// yaw; this never estimates or relaxes yaw.
inline FixedYawTranslationResult estimateFixedYawTranslationRobust(
  const std::vector<RecoveryAlignmentSample> & samples,
  double fixed_yaw_rad,
  const FixedYawTranslationConfig & config)
{
  FixedYawTranslationResult full = estimateFixedYawTranslation(samples, fixed_yaw_rad, config);
  const bool outlier_eligible_failure =
    full.reason == "position_rms_too_large" || full.reason == "position_outlier";
  if (full.valid || !config.allow_single_outlier_rejection ||
    !outlier_eligible_failure ||
    samples.size() <= std::max<std::size_t>(1, config.min_samples))
  {
    return full;
  }

  bool found = false;
  FixedYawTranslationResult best;
  double best_score = std::numeric_limits<double>::infinity();
  for (std::size_t rejected = 0; rejected < samples.size(); ++rejected) {
    std::vector<RecoveryAlignmentSample> subset;
    subset.reserve(samples.size() - 1U);
    for (std::size_t index = 0; index < samples.size(); ++index) {
      if (index != rejected) subset.push_back(samples[index]);
    }

    FixedYawTranslationResult candidate =
      estimateFixedYawTranslation(subset, fixed_yaw_rad, config);
    if (!candidate.valid) continue;
    const double score = candidate.position_rms_m +
      0.1 * candidate.max_position_residual_m;
    if (!found || score < best_score) {
      found = true;
      best_score = score;
      best = candidate;
      best.sample_count = samples.size();
      best.rejected_sample_count = 1U;
      best.rejected_sample_index = rejected;
      best.reason = "ok_after_single_outlier_rejection";
    }
  }

  return found ? best : full;
}

struct BoundedCorrectionResult
{
  double tx_m{0.0};
  double ty_m{0.0};
  double yaw_rad{0.0};
  double applied_translation_m{0.0};
  double applied_yaw_rad{0.0};
};

struct BasePivotResidual
{
  bool valid{false};
  double dx_m{0.0};
  double dy_m{0.0};
  double position_m{0.0};
  double yaw_rad{0.0};
};

struct BasePivotBoundedCorrectionResult
{
  bool valid{false};
  bool reached_target{false};
  double tx_m{0.0};
  double ty_m{0.0};
  double yaw_rad{0.0};
  double applied_base_translation_m{0.0};
  double applied_yaw_rad{0.0};
  double residual_base_translation_m{0.0};
  double residual_yaw_rad{0.0};
};

struct RecoveryCovarianceIncrement
{
  double xy_variance{0.0};
  double yaw_variance{0.0};
};

struct PositionYawJacobian
{
  double dx_dyaw{0.0};
  double dy_dyaw{0.0};
};

// Derivative of R(yaw) * delta_position with respect to yaw. XY-only recovery
// anchors position at its latest GNSS reference; this Jacobian propagates the
// retained yaw uncertainty as the vehicle moves away from that reference.
inline PositionYawJacobian positionYawJacobian(
  double delta_odom_x_m,
  double delta_odom_y_m,
  double map_odom_yaw_rad)
{
  const double c = std::cos(map_odom_yaw_rad);
  const double s = std::sin(map_odom_yaw_rad);
  return PositionYawJacobian{
    -s * delta_odom_x_m - c * delta_odom_y_m,
    c * delta_odom_x_m - s * delta_odom_y_m};
}

// A recovery target is initialized with covariance propagated through its
// first measurement stamp. Therefore its first bounded correction contributes
// no additional covariance. Later calls add only the drift and odometry
// increments since the anchor references advanced by the preceding call.
inline RecoveryCovarianceIncrement incrementalRecoveryCovariance(
  bool first_step,
  double elapsed_sec,
  double xy_drift_variance_per_sec,
  double yaw_drift_variance_per_sec,
  double odom_cov_xy_total,
  double odom_cov_xy_reference,
  double odom_cov_yaw_total,
  double odom_cov_yaw_reference)
{
  RecoveryCovarianceIncrement out;
  if (first_step) {
    return out;
  }
  const double elapsed = std::max(0.0, elapsed_sec);
  out.xy_variance = xy_drift_variance_per_sec * elapsed +
    std::max(0.0, odom_cov_xy_total - odom_cov_xy_reference);
  out.yaw_variance = yaw_drift_variance_per_sec * elapsed +
    std::max(0.0, odom_cov_yaw_total - odom_cov_yaw_reference);
  return out;
}

// A motion gate evaluated at a measurement timestamp needs an odometry
// sample at or immediately before that timestamp. A future sample elsewhere
// in the buffer must not hide a stale past sample.
inline bool latestPastSampleCoversTimestamp(
  double query_stamp_sec,
  double latest_past_sample_stamp_sec,
  double max_gap_sec)
{
  if (!std::isfinite(query_stamp_sec) ||
    !std::isfinite(latest_past_sample_stamp_sec) ||
    !std::isfinite(max_gap_sec) || max_gap_sec < 0.0)
  {
    return false;
  }
  const double age = query_stamp_sec - latest_past_sample_stamp_sec;
  return age >= 0.0 && age <= max_gap_sec;
}

inline BoundedCorrectionResult applyBoundedCorrection(
  double current_tx_m,
  double current_ty_m,
  double current_yaw_rad,
  double target_tx_m,
  double target_ty_m,
  double target_yaw_rad,
  double max_translation_step_m,
  double max_yaw_step_rad)
{
  BoundedCorrectionResult out;
  const double dx = target_tx_m - current_tx_m;
  const double dy = target_ty_m - current_ty_m;
  const double distance = std::hypot(dx, dy);
  const double translation_scale =
    distance > std::max(0.0, max_translation_step_m) && distance > 1.0e-12 ?
    max_translation_step_m / distance : 1.0;
  out.tx_m = current_tx_m + translation_scale * dx;
  out.ty_m = current_ty_m + translation_scale * dy;
  out.applied_translation_m = translation_scale * distance;

  const double dyaw = wrapAngle(target_yaw_rad - current_yaw_rad);
  const double bounded_dyaw = std::clamp(
    dyaw, -std::max(0.0, max_yaw_step_rad), std::max(0.0, max_yaw_step_rad));
  out.yaw_rad = wrapAngle(current_yaw_rad + bounded_dyaw);
  out.applied_yaw_rad = bounded_dyaw;
  return out;
}

// Evaluate two map->odom transforms at the same odom->base_link position.
// Recovery consumers observe map->base_link, so gating raw map->odom
// translation is insufficient: changing map->odom yaw rotates the entire
// odometry trajectory about the odom origin.  At a long distance from that
// origin, even a small bounded yaw step can otherwise move base_link by many
// metres.
inline BasePivotResidual basePivotResidual(
  double current_tx_m,
  double current_ty_m,
  double current_yaw_rad,
  double target_tx_m,
  double target_ty_m,
  double target_yaw_rad,
  double odom_base_x_m,
  double odom_base_y_m)
{
  BasePivotResidual out;
  if (!std::isfinite(current_tx_m) || !std::isfinite(current_ty_m) ||
    !std::isfinite(current_yaw_rad) || !std::isfinite(target_tx_m) ||
    !std::isfinite(target_ty_m) || !std::isfinite(target_yaw_rad) ||
    !std::isfinite(odom_base_x_m) || !std::isfinite(odom_base_y_m))
  {
    return out;
  }

  const double current_c = std::cos(current_yaw_rad);
  const double current_s = std::sin(current_yaw_rad);
  const double target_c = std::cos(target_yaw_rad);
  const double target_s = std::sin(target_yaw_rad);
  const double current_base_x = current_tx_m +
    current_c * odom_base_x_m - current_s * odom_base_y_m;
  const double current_base_y = current_ty_m +
    current_s * odom_base_x_m + current_c * odom_base_y_m;
  const double target_base_x = target_tx_m +
    target_c * odom_base_x_m - target_s * odom_base_y_m;
  const double target_base_y = target_ty_m +
    target_s * odom_base_x_m + target_c * odom_base_y_m;
  if (!std::isfinite(current_base_x) || !std::isfinite(current_base_y) ||
    !std::isfinite(target_base_x) || !std::isfinite(target_base_y))
  {
    return out;
  }

  out.dx_m = target_base_x - current_base_x;
  out.dy_m = target_base_y - current_base_y;
  out.position_m = std::hypot(out.dx_m, out.dy_m);
  out.yaw_rad = std::fabs(wrapAngle(target_yaw_rad - current_yaw_rad));
  out.valid = std::isfinite(out.position_m) && std::isfinite(out.yaw_rad);
  return out;
}

// Apply a planar recovery correction about the synchronized base_link
// position.  Translation is bounded in the map->base_link space visible to
// downstream consumers; map->odom translation is reconstructed after the yaw
// step so that yaw cannot create an unbounded position excursion.  The odom
// point passed here is deliberately base_link, not a GNSS antenna observation
// point: lever-arm handling belongs to target estimation, while continuity is
// guaranteed at the frame consumed by localization users.
inline BasePivotBoundedCorrectionResult applyBasePivotBoundedCorrection(
  double current_tx_m,
  double current_ty_m,
  double current_yaw_rad,
  double target_tx_m,
  double target_ty_m,
  double target_yaw_rad,
  double odom_base_x_m,
  double odom_base_y_m,
  double max_base_translation_step_m,
  double max_yaw_step_rad)
{
  BasePivotBoundedCorrectionResult out;
  if (!std::isfinite(max_base_translation_step_m) ||
    !std::isfinite(max_yaw_step_rad) ||
    max_base_translation_step_m < 0.0 || max_yaw_step_rad < 0.0)
  {
    return out;
  }

  const BasePivotResidual initial = basePivotResidual(
    current_tx_m, current_ty_m, current_yaw_rad,
    target_tx_m, target_ty_m, target_yaw_rad,
    odom_base_x_m, odom_base_y_m);
  if (!initial.valid) {
    return out;
  }

  const double current_c = std::cos(current_yaw_rad);
  const double current_s = std::sin(current_yaw_rad);
  const double current_base_x = current_tx_m +
    current_c * odom_base_x_m - current_s * odom_base_y_m;
  const double current_base_y = current_ty_m +
    current_s * odom_base_x_m + current_c * odom_base_y_m;
  const double translation_scale =
    initial.position_m > max_base_translation_step_m && initial.position_m > 1.0e-12 ?
    max_base_translation_step_m / initial.position_m : 1.0;
  const double bounded_base_x = current_base_x + translation_scale * initial.dx_m;
  const double bounded_base_y = current_base_y + translation_scale * initial.dy_m;

  const double signed_yaw_residual = wrapAngle(target_yaw_rad - current_yaw_rad);
  const double bounded_dyaw = std::clamp(
    signed_yaw_residual, -max_yaw_step_rad, max_yaw_step_rad);
  const double bounded_yaw = wrapAngle(current_yaw_rad + bounded_dyaw);
  const double bounded_c = std::cos(bounded_yaw);
  const double bounded_s = std::sin(bounded_yaw);

  out.tx_m = bounded_base_x -
    (bounded_c * odom_base_x_m - bounded_s * odom_base_y_m);
  out.ty_m = bounded_base_y -
    (bounded_s * odom_base_x_m + bounded_c * odom_base_y_m);
  out.yaw_rad = bounded_yaw;
  out.applied_base_translation_m = translation_scale * initial.position_m;
  out.applied_yaw_rad = bounded_dyaw;

  const bool position_reached = translation_scale >= 1.0;
  const bool yaw_reached = std::fabs(signed_yaw_residual) <= max_yaw_step_rad;
  if (position_reached && yaw_reached) {
    // Avoid leaving a floating-point reconstruction residue when recovery is
    // complete.  This also makes completion exactly equal to the accepted
    // map->odom target.
    out.tx_m = target_tx_m;
    out.ty_m = target_ty_m;
    out.yaw_rad = wrapAngle(target_yaw_rad);
    out.residual_base_translation_m = 0.0;
    out.residual_yaw_rad = 0.0;
    out.reached_target = true;
  } else {
    const BasePivotResidual remaining = basePivotResidual(
      out.tx_m, out.ty_m, out.yaw_rad,
      target_tx_m, target_ty_m, target_yaw_rad,
      odom_base_x_m, odom_base_y_m);
    if (!remaining.valid) {
      return BasePivotBoundedCorrectionResult{};
    }
    out.residual_base_translation_m = remaining.position_m;
    out.residual_yaw_rad = remaining.yaw_rad;
  }

  out.valid = std::isfinite(out.tx_m) && std::isfinite(out.ty_m) &&
    std::isfinite(out.yaw_rad) && std::isfinite(out.applied_base_translation_m) &&
    std::isfinite(out.applied_yaw_rad) &&
    std::isfinite(out.residual_base_translation_m) &&
    std::isfinite(out.residual_yaw_rad);
  return out;
}

}  // namespace pure_gnss_map_odom_fusion
