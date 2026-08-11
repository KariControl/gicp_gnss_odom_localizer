// SPDX-License-Identifier: Apache-2.0
#include "pure_precision_global_localizer/precision_anchor_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

#include <Eigen/Eigenvalues>

namespace pure_precision_global_localizer
{

namespace
{
constexpr double kEpsilon = 1.0e-12;

double median(std::vector<double> values)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if ((values.size() % 2U) != 0U) {
    return upper;
  }
  std::nth_element(values.begin(), values.begin() + middle - 1U, values.end());
  return 0.5 * (upper + values[middle - 1U]);
}

bool validSample(const PositionAlignmentSample & sample, const AnchorConfig & config)
{
  return std::isfinite(sample.stamp_sec) && sample.local_point.allFinite() &&
         sample.map_point.allFinite() && std::isfinite(sample.position_variance_m2) &&
         sample.position_variance_m2 >= 0.0 &&
         sample.position_variance_m2 <= config.max_position_variance_m2;
}

double maximumBaseline(
  const std::vector<PositionAlignmentSample> & samples,
  const std::vector<std::size_t> & indices,
  bool use_local)
{
  double baseline = 0.0;
  for (std::size_t i = 0; i < indices.size(); ++i) {
    const auto & first = use_local ?
      samples[indices[i]].local_point : samples[indices[i]].map_point;
    for (std::size_t j = i + 1U; j < indices.size(); ++j) {
      const auto & second = use_local ?
        samples[indices[j]].local_point : samples[indices[j]].map_point;
      baseline = std::max(baseline, (second - first).norm());
    }
  }
  return baseline;
}

Pose2 translationForYawMedian(
  const std::vector<PositionAlignmentSample> & samples,
  double yaw)
{
  const Eigen::Matrix2d rotation = rotation2(yaw);
  std::vector<double> x_values;
  std::vector<double> y_values;
  x_values.reserve(samples.size());
  y_values.reserve(samples.size());
  for (const auto & sample : samples) {
    const Eigen::Vector2d translation = sample.map_point - rotation * sample.local_point;
    x_values.push_back(translation.x());
    y_values.push_back(translation.y());
  }
  return {median(std::move(x_values)), median(std::move(y_values)), wrapAngle(yaw)};
}

std::vector<double> residuals(
  const std::vector<PositionAlignmentSample> & samples,
  const Pose2 & anchor)
{
  std::vector<double> values;
  values.reserve(samples.size());
  for (const auto & sample : samples) {
    values.push_back((transformPoint(anchor, sample.local_point) - sample.map_point).norm());
  }
  return values;
}

std::vector<std::size_t> inlierIndices(
  const std::vector<double> & values, double gate)
{
  std::vector<std::size_t> indices;
  indices.reserve(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (std::isfinite(values[i]) && values[i] <= gate) {
      indices.push_back(i);
    }
  }
  return indices;
}

Pose2 weightedSe2(
  const std::vector<PositionAlignmentSample> & samples,
  const std::vector<std::size_t> & indices)
{
  double total_weight = 0.0;
  Eigen::Vector2d local_center = Eigen::Vector2d::Zero();
  Eigen::Vector2d map_center = Eigen::Vector2d::Zero();
  for (const std::size_t index : indices) {
    const double weight = 1.0 / std::max(samples[index].position_variance_m2, 1.0e-6);
    total_weight += weight;
    local_center += weight * samples[index].local_point;
    map_center += weight * samples[index].map_point;
  }
  if (!(total_weight > 0.0)) {
    return {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
  }
  local_center /= total_weight;
  map_center /= total_weight;

  double dot = 0.0;
  double cross = 0.0;
  for (const std::size_t index : indices) {
    const double weight = 1.0 / std::max(samples[index].position_variance_m2, 1.0e-6);
    const Eigen::Vector2d local = samples[index].local_point - local_center;
    const Eigen::Vector2d map = samples[index].map_point - map_center;
    dot += weight * local.dot(map);
    cross += weight * (local.x() * map.y() - local.y() * map.x());
  }
  const double yaw = std::atan2(cross, dot);
  const Eigen::Vector2d translation = map_center - rotation2(yaw) * local_center;
  return {translation.x(), translation.y(), wrapAngle(yaw)};
}

AlignmentEstimate summarizeEstimate(
  const std::vector<PositionAlignmentSample> & samples,
  const std::vector<std::size_t> & inliers,
  const Pose2 & anchor,
  bool estimate_yaw,
  const AnchorConfig & config)
{
  AlignmentEstimate result;
  result.anchor = anchor;
  result.inlier_count = inliers.size();
  result.rejected_count = samples.size() - inliers.size();
  if (!anchor.finite() || inliers.empty()) {
    result.reason = "invalid_solution";
    return result;
  }

  double squared_error = 0.0;
  double mean_variance = 0.0;
  Eigen::Vector2d local_center = Eigen::Vector2d::Zero();
  for (const std::size_t index : inliers) {
    const Eigen::Vector2d error =
      transformPoint(anchor, samples[index].local_point) - samples[index].map_point;
    squared_error += error.squaredNorm();
    mean_variance += std::max(
      samples[index].position_variance_m2, config.min_position_variance_m2);
    local_center += samples[index].local_point;
  }
  const double count = static_cast<double>(inliers.size());
  result.rms_m = std::sqrt(squared_error / count);
  mean_variance /= count;
  local_center /= count;
  result.local_reference = local_center;
  double local_scatter = 0.0;
  for (const std::size_t index : inliers) {
    local_scatter += (samples[index].local_point - local_center).squaredNorm();
  }

  if (estimate_yaw) {
    result.local_baseline_m = maximumBaseline(samples, inliers, true);
    result.map_baseline_m = maximumBaseline(samples, inliers, false);
  }
  const double translation_variance = std::max(
    config.min_position_variance_m2,
    (mean_variance + result.rms_m * result.rms_m) / count);
  double yaw_variance = config.unobservable_yaw_variance_rad2;
  if (estimate_yaw) {
    yaw_variance = std::max(
      config.min_yaw_variance_rad2,
      (mean_variance + result.rms_m * result.rms_m) /
      std::max(local_scatter, kEpsilon));
  }
  result.yaw_stddev_rad = std::sqrt(yaw_variance);
  result.covariance.setZero();
  result.covariance(0, 0) = translation_variance;
  result.covariance(1, 1) = translation_variance;
  if (estimate_yaw) {
    // The fit is best constrained at its local centroid. Expressing the
    // covariance at the anchor origin requires the translation/yaw cross term
    // below. During output propagation d + R*J*p becomes R*J*(p-centroid), so
    // a far-away odom origin does not cause double-counted uncertainty.
    const Eigen::Vector2d yaw_translation_derivative =
      -rotation2(anchor.yaw) * Eigen::Vector2d(-local_center.y(), local_center.x());
    result.covariance.block<2, 2>(0, 0) +=
      yaw_variance * yaw_translation_derivative * yaw_translation_derivative.transpose();
    result.covariance.block<2, 1>(0, 2) = yaw_variance * yaw_translation_derivative;
    result.covariance.block<1, 2>(2, 0) =
      result.covariance.block<2, 1>(0, 2).transpose();
    result.covariance(2, 2) = yaw_variance;
  }

  if (result.rms_m > config.max_rms_m) {
    result.reason = "rms_gate";
    return result;
  }
  result.valid = true;
  result.reason = estimate_yaw ? "se2" : "translation_only";
  return result;
}

Eigen::Vector2d boundedVector(const Eigen::Vector2d & value, double limit)
{
  if (!(limit > 0.0) || !std::isfinite(limit)) {
    return Eigen::Vector2d::Zero();
  }
  const double norm = value.norm();
  return norm > limit ? value * (limit / norm) : value;
}
}  // namespace

const char * toString(AnchorState state)
{
  switch (state) {
    case AnchorState::UNINITIALIZED:
      return "UNINITIALIZED";
    case AnchorState::TRACKING_XY_ONLY:
      return "TRACKING_XY_ONLY";
    case AnchorState::TRACKING_SE2:
      return "TRACKING_SE2";
    case AnchorState::HOLD_SOFT_GAP:
      return "HOLD_SOFT_GAP";
    case AnchorState::OUTAGE:
      return "OUTAGE";
  }
  return "UNKNOWN";
}

AlignmentEstimate estimateRobustSe2(
  const std::vector<PositionAlignmentSample> & samples,
  const AnchorConfig & config)
{
  AlignmentEstimate result;
  if (samples.size() < config.min_yaw_samples) {
    result.reason = "not_enough_samples";
    return result;
  }
  for (const auto & sample : samples) {
    if (!validSample(sample, config)) {
      result.reason = "invalid_sample";
      return result;
    }
  }

  std::vector<double> candidate_yaws;
  candidate_yaws.reserve(samples.size() * (samples.size() - 1U) / 2U);
  const double pair_local_gate = std::max(0.5, 0.5 * config.min_local_baseline_m);
  const double pair_map_gate = std::max(0.5, 0.5 * config.min_map_baseline_m);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    for (std::size_t j = i + 1U; j < samples.size(); ++j) {
      const Eigen::Vector2d local_delta = samples[j].local_point - samples[i].local_point;
      const Eigen::Vector2d map_delta = samples[j].map_point - samples[i].map_point;
      if (local_delta.norm() < pair_local_gate || map_delta.norm() < pair_map_gate) {
        continue;
      }
      candidate_yaws.push_back(wrapAngle(
        std::atan2(map_delta.y(), map_delta.x()) -
        std::atan2(local_delta.y(), local_delta.x())));
    }
  }
  if (candidate_yaws.empty()) {
    result.reason = "yaw_unobservable";
    return result;
  }

  double best_score = std::numeric_limits<double>::infinity();
  Pose2 best_anchor;
  for (const double yaw : candidate_yaws) {
    const Pose2 candidate = translationForYawMedian(samples, yaw);
    const double score = median(residuals(samples, candidate));
    if (std::isfinite(score) && score < best_score) {
      best_score = score;
      best_anchor = candidate;
    }
  }
  if (!std::isfinite(best_score)) {
    result.reason = "invalid_solution";
    return result;
  }

  auto inliers = inlierIndices(residuals(samples, best_anchor), config.inlier_gate_m);
  if (inliers.size() < config.min_yaw_samples) {
    result.reason = "not_enough_inliers";
    return result;
  }
  Pose2 estimate = weightedSe2(samples, inliers);
  inliers = inlierIndices(residuals(samples, estimate), config.inlier_gate_m);
  if (inliers.size() < config.min_yaw_samples) {
    result.reason = "not_enough_inliers";
    return result;
  }
  estimate = weightedSe2(samples, inliers);
  result = summarizeEstimate(samples, inliers, estimate, true, config);
  if (!result.valid) {
    return result;
  }
  if (result.local_baseline_m < config.min_local_baseline_m ||
    result.map_baseline_m < config.min_map_baseline_m)
  {
    result.valid = false;
    result.reason = "yaw_unobservable";
    return result;
  }
  if (result.yaw_stddev_rad > config.max_yaw_stddev_rad) {
    result.valid = false;
    result.reason = "yaw_uncertainty_gate";
  }
  return result;
}

AlignmentEstimate estimateRobustTranslation(
  const std::vector<PositionAlignmentSample> & samples,
  double committed_yaw,
  const AnchorConfig & config)
{
  AlignmentEstimate result;
  if (samples.empty() || !std::isfinite(committed_yaw)) {
    result.reason = "not_enough_samples";
    return result;
  }
  for (const auto & sample : samples) {
    if (!validSample(sample, config)) {
      result.reason = "invalid_sample";
      return result;
    }
  }
  const Pose2 median_anchor = translationForYawMedian(samples, committed_yaw);
  const auto inliers = inlierIndices(
    residuals(samples, median_anchor), config.inlier_gate_m);
  if (inliers.empty()) {
    result.reason = "not_enough_inliers";
    return result;
  }

  double total_weight = 0.0;
  Eigen::Vector2d translation = Eigen::Vector2d::Zero();
  const Eigen::Matrix2d rotation = rotation2(committed_yaw);
  for (const std::size_t index : inliers) {
    const double weight = 1.0 / std::max(
      samples[index].position_variance_m2, config.min_position_variance_m2);
    translation += weight * (samples[index].map_point - rotation * samples[index].local_point);
    total_weight += weight;
  }
  if (!(total_weight > 0.0)) {
    result.reason = "invalid_weights";
    return result;
  }
  translation /= total_weight;
  const Pose2 estimate{translation.x(), translation.y(), wrapAngle(committed_yaw)};
  result = summarizeEstimate(samples, inliers, estimate, false, config);
  result.anchor.yaw = wrapAngle(committed_yaw);
  return result;
}

BoundedAnchorStep stepAnchorAtBase(
  const Pose2 & applied,
  const Pose2 & target,
  const Pose2 & current_local_base,
  double max_base_translation_m,
  double max_yaw_rad)
{
  BoundedAnchorStep result;
  if (!applied.finite() || !target.finite() || !current_local_base.finite()) {
    return result;
  }
  const Pose2 current_global = compose(applied, current_local_base);
  const Pose2 desired_global = compose(target, current_local_base);
  const Eigen::Vector2d requested_translation(
    desired_global.x - current_global.x,
    desired_global.y - current_global.y);
  const Eigen::Vector2d applied_translation = boundedVector(
    requested_translation, max_base_translation_m);
  const double applied_yaw = clampMagnitude(
    wrapAngle(target.yaw - applied.yaw), max_yaw_rad);
  const double new_yaw = wrapAngle(applied.yaw + applied_yaw);
  const Eigen::Vector2d new_global_base =
    Eigen::Vector2d(current_global.x, current_global.y) + applied_translation;
  const Eigen::Vector2d new_anchor_translation =
    new_global_base - rotation2(new_yaw) *
    Eigen::Vector2d(current_local_base.x, current_local_base.y);
  result.valid = true;
  result.anchor = {
    new_anchor_translation.x(), new_anchor_translation.y(), new_yaw};
  result.base_translation_m = applied_translation.norm();
  result.yaw_rad = applied_yaw;
  return result;
}

PrecisionAnchorEstimator::PrecisionAnchorEstimator(AnchorConfig config)
: config_(config)
{
  reset("waiting_for_position_initialization");
  last_reason_ = "uninitialized";
}

void PrecisionAnchorEstimator::reset(const std::string & reason)
{
  ++activation_epoch_;
  samples_.clear();
  yaw_samples_.clear();
  initialized_ = false;
  yaw_observed_ = false;
  state_ = AnchorState::UNINITIALIZED;
  applied_anchor_ = Pose2{};
  target_anchor_ = Pose2{};
  anchor_covariance_.setZero();
  has_last_usable_stamp_ = false;
  last_usable_stamp_sec_ = 0.0;
  has_last_correction_stamp_ = false;
  last_correction_stamp_sec_ = 0.0;
  yaw_evaluation_count_ = 0U;
  activation_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
  resetActivationCandidate(reason);
  last_reason_ = reason;
}

void PrecisionAnchorEstimator::resetActivationCandidate(const std::string & reason)
{
  has_activation_candidate_ = false;
  activation_candidate_ = AlignmentEstimate{};
  stable_yaw_candidate_count_ = 0U;
  activation_candidate_delta_rad_ = std::numeric_limits<double>::quiet_NaN();
  activation_reason_ = reason;
}

bool PrecisionAnchorEstimator::observeActivationCandidate(
  const AlignmentEstimate & estimate,
  const PositionAlignmentSample & sample,
  const Pose2 & current_local_base,
  AnchorUpdate & update)
{
  if (!estimate.valid || yaw_observed_) {
    return false;
  }

  update.yaw_activation_candidate = true;
  if (!has_activation_candidate_) {
    stable_yaw_candidate_count_ = 1U;
    activation_candidate_delta_rad_ = std::numeric_limits<double>::quiet_NaN();
    activation_reason_ = "stable_yaw_candidate_started";
  } else {
    activation_candidate_delta_rad_ = std::fabs(wrapAngle(
      estimate.anchor.yaw - activation_candidate_.anchor.yaw));
    if (activation_candidate_delta_rad_ <=
      config_.activation_max_yaw_candidate_delta_rad)
    {
      ++stable_yaw_candidate_count_;
      activation_reason_ = "stable_yaw_candidate_accumulating";
    } else {
      stable_yaw_candidate_count_ = 1U;
      activation_reason_ = "yaw_candidate_jump_restarted";
    }
  }
  activation_candidate_ = estimate;
  has_activation_candidate_ = true;
  update.stable_yaw_candidate_count = stable_yaw_candidate_count_;

  if (stable_yaw_candidate_count_ <
    config_.activation_min_stable_yaw_candidates)
  {
    return false;
  }

  // No authoritative global pose has been published yet. Commit the first
  // observable yaw atomically so the very first output cannot expose the
  // arbitrary bootstrap yaw or a rate-limited catch-up transient. Subsequent
  // target refreshes continue through applyTowardTarget().
  const Pose2 previous_applied = applied_anchor_;
  const Pose2 previous_global_base = compose(previous_applied, current_local_base);
  target_anchor_ = estimate.anchor;
  applied_anchor_ = target_anchor_;
  anchor_covariance_ = estimate.covariance;
  yaw_observed_ = true;
  ++activation_commit_count_;
  activation_stamp_sec_ = sample.stamp_sec;
  activation_reason_ = "stable_yaw_activated";
  state_ = AnchorState::TRACKING_SE2;
  has_last_correction_stamp_ = true;
  last_correction_stamp_sec_ = sample.stamp_sec;

  const Pose2 activated_global_base = compose(applied_anchor_, current_local_base);
  update.accepted = true;
  update.target_updated = true;
  update.yaw_updated = true;
  update.yaw_activation_committed = true;
  update.reason = "stable_yaw_activated";
  update.state = state_;
  update.applied_base_translation_m = std::hypot(
    activated_global_base.x - previous_global_base.x,
    activated_global_base.y - previous_global_base.y);
  update.applied_yaw_rad = wrapAngle(applied_anchor_.yaw - previous_applied.yaw);
  return true;
}

void PrecisionAnchorEstimator::pruneWindow(double newest_stamp_sec)
{
  while (!samples_.empty() &&
    newest_stamp_sec - samples_.front().stamp_sec > config_.window_sec)
  {
    samples_.pop_front();
  }
  while (samples_.size() > 512U) {
    samples_.pop_front();
  }
  while (!yaw_samples_.empty() &&
    newest_stamp_sec - yaw_samples_.front().stamp_sec > config_.window_sec)
  {
    yaw_samples_.pop_front();
  }
  while (yaw_samples_.size() > config_.max_yaw_samples) {
    yaw_samples_.pop_front();
  }
}

std::vector<PositionAlignmentSample> PrecisionAnchorEstimator::contiguousYawWindow() const
{
  std::vector<PositionAlignmentSample> result;
  if (yaw_samples_.empty()) {
    return result;
  }
  auto begin = yaw_samples_.end() - 1;
  while (begin != yaw_samples_.begin()) {
    const auto previous = begin - 1;
    if (begin->stamp_sec - previous->stamp_sec > config_.max_sample_gap_sec) {
      break;
    }
    begin = previous;
  }
  result.assign(begin, yaw_samples_.end());
  return result;
}

void PrecisionAnchorEstimator::bootstrap(
  const PositionAlignmentSample & sample,
  const Pose2 &,
  AnchorUpdate & update)
{
  const double yaw = wrapAngle(config_.bootstrap_yaw_rad);
  const Eigen::Vector2d translation = sample.map_point - rotation2(yaw) * sample.local_point;
  target_anchor_ = {translation.x(), translation.y(), yaw};
  applied_anchor_ = target_anchor_;
  anchor_covariance_.setZero();
  const double position_variance = std::max(
    sample.position_variance_m2, config_.min_position_variance_m2);
  const double yaw_variance = config_.unobservable_yaw_variance_rad2;
  const Eigen::Vector2d derivative =
    -rotation2(yaw) * Eigen::Vector2d(-sample.local_point.y(), sample.local_point.x());
  anchor_covariance_.block<2, 2>(0, 0) =
    Eigen::Matrix2d::Identity() * position_variance +
    yaw_variance * derivative * derivative.transpose();
  anchor_covariance_.block<2, 1>(0, 2) = yaw_variance * derivative;
  anchor_covariance_.block<1, 2>(2, 0) =
    anchor_covariance_.block<2, 1>(0, 2).transpose();
  anchor_covariance_(2, 2) = yaw_variance;
  initialized_ = true;
  yaw_observed_ = false;
  state_ = AnchorState::TRACKING_XY_ONLY;
  has_last_correction_stamp_ = true;
  last_correction_stamp_sec_ = sample.stamp_sec;
  update.accepted = true;
  update.initialized = true;
  update.target_updated = true;
  update.reason = "bootstrapped_xy_only";
  update.state = state_;
}

void PrecisionAnchorEstimator::applyTowardTarget(
  double stamp_sec,
  const Pose2 & current_local_base,
  AnchorUpdate & update)
{
  if (!has_last_correction_stamp_ || stamp_sec <= last_correction_stamp_sec_) {
    has_last_correction_stamp_ = true;
    last_correction_stamp_sec_ = stamp_sec;
    return;
  }
  const double dt_sec = std::min(
    stamp_sec - last_correction_stamp_sec_, config_.max_correction_dt_sec);
  const double translation_limit = std::min(
    config_.max_translation_step_m, config_.max_translation_rate_mps * dt_sec);
  const double yaw_limit = std::min(
    config_.max_yaw_step_rad, config_.max_yaw_rate_radps * dt_sec);
  const auto step = stepAnchorAtBase(
    applied_anchor_, target_anchor_, current_local_base,
    translation_limit, yaw_limit);
  if (step.valid) {
    applied_anchor_ = step.anchor;
    update.applied_base_translation_m = step.base_translation_m;
    update.applied_yaw_rad = step.yaw_rad;
  }
  last_correction_stamp_sec_ = stamp_sec;
}

AnchorUpdate PrecisionAnchorEstimator::observePosition(
  const PositionAlignmentSample & sample,
  const Pose2 & current_local_base)
{
  AnchorUpdate update;
  update.state = state_;
  if (!validSample(sample, config_) || !current_local_base.finite()) {
    update.reason = "invalid_observation";
    last_reason_ = update.reason;
    return update;
  }
  if (has_last_usable_stamp_ && sample.stamp_sec <= last_usable_stamp_sec_) {
    update.reason = "nonmonotonic_observation";
    last_reason_ = update.reason;
    return update;
  }

  const bool hard_gap = has_last_usable_stamp_ &&
    sample.stamp_sec - last_usable_stamp_sec_ > config_.hard_outage_sec;
  if (hard_gap) {
    samples_.clear();
    yaw_samples_.clear();
    if (!yaw_observed_) {
      resetActivationCandidate("hard_gap_reset");
    }
  }
  samples_.push_back(sample);
  const bool yaw_sample_added = yaw_samples_.empty() ||
    sample.stamp_sec - yaw_samples_.back().stamp_sec >=
    config_.yaw_sample_min_interval_sec;
  if (yaw_sample_added) {
    yaw_samples_.push_back(sample);
  }
  pruneWindow(sample.stamp_sec);

  if (!initialized_) {
    bootstrap(sample, current_local_base, update);
    has_last_usable_stamp_ = true;
    last_usable_stamp_sec_ = sample.stamp_sec;
    last_reason_ = update.reason;
    return update;
  }

  AlignmentEstimate se2;
  if (yaw_sample_added) {
    ++yaw_evaluation_count_;
    se2 = estimateRobustSe2(contiguousYawWindow(), config_);
  } else {
    se2.reason = "yaw_downsample_hold";
  }

  if (!yaw_observed_ && yaw_sample_added) {
    if (se2.valid) {
      if (observeActivationCandidate(se2, sample, current_local_base, update)) {
        has_last_usable_stamp_ = true;
        last_usable_stamp_sec_ = sample.stamp_sec;
        last_reason_ = update.reason;
        return update;
      }
      // A gated full-SE(2) candidate is a usable position observation even
      // though it is not authoritative yet. Hold the unpublished bootstrap
      // anchor until the required independent estimates agree; attempting a
      // fixed-bootstrap-yaw translation fit here can fail precisely because
      // the newly observable yaw is far from that arbitrary bootstrap value.
      update.accepted = true;
      update.reason = activation_reason_;
      update.state = AnchorState::TRACKING_XY_ONLY;
      state_ = update.state;
      has_last_usable_stamp_ = true;
      last_usable_stamp_sec_ = sample.stamp_sec;
      last_reason_ = update.reason;
      return update;
    } else if (stable_yaw_candidate_count_ > 0U) {
      resetActivationCandidate("yaw_candidate_invalid_" + se2.reason);
    } else {
      activation_reason_ = "waiting_for_" + se2.reason;
    }
  }

  bool use_se2 = se2.valid && yaw_observed_;
  if (use_se2 && yaw_observed_ &&
    std::fabs(wrapAngle(se2.anchor.yaw - target_anchor_.yaw)) >
    config_.max_committed_yaw_innovation_rad)
  {
    use_se2 = false;
  }

  AlignmentEstimate selected;
  if (use_se2) {
    selected = se2;
    target_anchor_ = selected.anchor;
    anchor_covariance_ = selected.covariance;
    yaw_observed_ = true;
    update.yaw_updated = true;
    update.reason = "se2_target";
  } else {
    std::vector<PositionAlignmentSample> translation_samples(samples_.begin(), samples_.end());
    selected = estimateRobustTranslation(
      translation_samples, target_anchor_.yaw, config_);
    if (!selected.valid) {
      update.reason = use_se2 ? "se2_rejected" : selected.reason;
      last_reason_ = update.reason;
      has_last_usable_stamp_ = true;
      last_usable_stamp_sec_ = sample.stamp_sec;
      state_ = yaw_observed_ ? AnchorState::TRACKING_SE2 : AnchorState::TRACKING_XY_ONLY;
      update.state = state_;
      return update;
    }
    target_anchor_.x = selected.anchor.x;
    target_anchor_.y = selected.anchor.y;
    // Re-reference the held-yaw covariance at this translation fit's local
    // centroid. This retains yaw uncertainty while avoiding a second lever-arm
    // addition when the global pose is evaluated near the fitted samples.
    const double yaw_variance = std::max(
      anchor_covariance_(2, 2), config_.min_yaw_variance_rad2);
    const Eigen::Vector2d derivative =
      -rotation2(target_anchor_.yaw) *
      Eigen::Vector2d(-selected.local_reference.y(), selected.local_reference.x());
    anchor_covariance_.setZero();
    anchor_covariance_.block<2, 2>(0, 0) =
      selected.covariance.block<2, 2>(0, 0) +
      yaw_variance * derivative * derivative.transpose();
    anchor_covariance_.block<2, 1>(0, 2) = yaw_variance * derivative;
    anchor_covariance_.block<1, 2>(2, 0) =
      anchor_covariance_.block<2, 1>(0, 2).transpose();
    anchor_covariance_(2, 2) = yaw_variance;
    if (!yaw_observed_ && yaw_sample_added) {
      update.reason = activation_reason_;
      update.yaw_activation_candidate = se2.valid;
      update.stable_yaw_candidate_count = stable_yaw_candidate_count_;
    } else {
      update.reason = yaw_observed_ ? "xy_target_yaw_held" : "xy_only_target";
    }
  }

  update.accepted = true;
  update.target_updated = true;
  applyTowardTarget(sample.stamp_sec, current_local_base, update);
  has_last_usable_stamp_ = true;
  last_usable_stamp_sec_ = sample.stamp_sec;
  state_ = yaw_observed_ ? AnchorState::TRACKING_SE2 : AnchorState::TRACKING_XY_ONLY;
  update.state = state_;
  last_reason_ = update.reason;
  return update;
}

void PrecisionAnchorEstimator::observeUnusable(double stamp_sec)
{
  if (!std::isfinite(stamp_sec) || !initialized_) {
    return;
  }
  if (!yaw_observed_) {
    resetActivationCandidate("soft_gap_reset");
  }
  const double age = has_last_usable_stamp_ ?
    std::max(0.0, stamp_sec - last_usable_stamp_sec_) :
    std::numeric_limits<double>::infinity();
  state_ = age > config_.hard_outage_sec ?
    AnchorState::OUTAGE : AnchorState::HOLD_SOFT_GAP;
  last_reason_ = state_ == AnchorState::OUTAGE ? "hard_outage_hold" : "soft_gap_hold";
}

void PrecisionAnchorEstimator::updateTime(double stamp_sec)
{
  if (!initialized_ || !has_last_usable_stamp_ || !std::isfinite(stamp_sec)) {
    return;
  }
  if (stamp_sec - last_usable_stamp_sec_ > config_.hard_outage_sec) {
    if (!yaw_observed_ && state_ != AnchorState::OUTAGE) {
      resetActivationCandidate("hard_outage_clock_reset");
    }
    state_ = AnchorState::OUTAGE;
    last_reason_ = "hard_outage_hold";
  }
}

Eigen::Matrix3d PrecisionAnchorEstimator::effectiveAnchorCovariance(double stamp_sec) const
{
  Eigen::Matrix3d covariance = anchor_covariance_;
  if (!initialized_) {
    return Eigen::Matrix3d::Identity() * 1.0e6;
  }
  if (has_last_usable_stamp_ && std::isfinite(stamp_sec)) {
    const double outage_duration = std::max(
      0.0, stamp_sec - last_usable_stamp_sec_ - config_.hard_outage_sec);
    covariance(0, 0) += config_.outage_xy_variance_rate_m2ps * outage_duration;
    covariance(1, 1) += config_.outage_xy_variance_rate_m2ps * outage_duration;
    covariance(2, 2) += config_.outage_yaw_variance_rate_rad2ps * outage_duration;
  }
  return projectCovariancePsd(covariance);
}

Eigen::Matrix3d projectCovariancePsd(
  const Eigen::Matrix3d & covariance,
  double minimum_eigenvalue,
  double fallback_variance)
{
  if (!covariance.allFinite()) {
    return Eigen::Matrix3d::Identity() * fallback_variance;
  }
  const Eigen::Matrix3d symmetric = 0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
    !solver.eigenvectors().allFinite())
  {
    return Eigen::Matrix3d::Identity() * fallback_variance;
  }
  const Eigen::Vector3d eigenvalues =
    solver.eigenvalues().cwiseMax(std::max(0.0, minimum_eigenvalue));
  return solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}

Eigen::Matrix3d propagateGlobalCovariance(
  const Pose2 & local_pose,
  const Pose2 & anchor,
  const Eigen::Matrix3d & local_covariance,
  const Eigen::Matrix3d & anchor_covariance)
{
  if (!local_pose.finite() || !anchor.finite()) {
    return Eigen::Matrix3d::Identity() * 1.0e6;
  }
  const double c = std::cos(anchor.yaw);
  const double s = std::sin(anchor.yaw);
  Eigen::Matrix3d local_jacobian = Eigen::Matrix3d::Zero();
  local_jacobian << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
  Eigen::Matrix3d anchor_jacobian = Eigen::Matrix3d::Identity();
  anchor_jacobian(0, 2) = -s * local_pose.x - c * local_pose.y;
  anchor_jacobian(1, 2) = c * local_pose.x - s * local_pose.y;
  const Eigen::Matrix3d covariance =
    local_jacobian * projectCovariancePsd(local_covariance) * local_jacobian.transpose() +
    anchor_jacobian * projectCovariancePsd(anchor_covariance) * anchor_jacobian.transpose();
  return projectCovariancePsd(covariance);
}

}  // namespace pure_precision_global_localizer
