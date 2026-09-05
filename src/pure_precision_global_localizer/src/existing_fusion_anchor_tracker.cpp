// SPDX-License-Identifier: Apache-2.0
#include "pure_precision_global_localizer/existing_fusion_anchor_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <Eigen/Eigenvalues>

#include "pure_precision_global_localizer/precision_anchor_estimator.hpp"

namespace pure_precision_global_localizer
{

namespace
{
bool finitePositive(double value)
{
  return std::isfinite(value) && value > 0.0;
}
}  // namespace

const char * toString(FusionAnchorState state)
{
  switch (state) {
    case FusionAnchorState::WAITING_HEALTHY:
      return "WAITING_HEALTHY";
    case FusionAnchorState::STABILIZING_STARTUP:
      return "STABILIZING_STARTUP";
    case FusionAnchorState::TRACKING:
      return "TRACKING";
    case FusionAnchorState::FROZEN:
      return "FROZEN";
    case FusionAnchorState::STABILIZING_RECOVERY:
      return "STABILIZING_RECOVERY";
  }
  return "UNKNOWN";
}

Pose2 derivePrecisionAnchor(
  const Pose2 & existing_global,
  const Pose2 & precision_local)
{
  if (!existing_global.finite() || !precision_local.finite()) {
    return {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
  }
  return compose(existing_global, inverse(precision_local));
}

FusionHealthEvaluation evaluateStrictExistingFusionHealth(
  const ExistingFusionHealthFields & fields)
{
  FusionHealthEvaluation result;
  if (fields.authority_state == 2) {
    result.reason = "fusion_soft_bad_hold";
  } else if (fields.authority_state != 1) {
    result.reason = "fusion_authority_unhealthy";
  } else if (fields.recovery_state != "tracking") {
    result.reason = "fusion_not_tracking";
  } else if (fields.anchor_valid != "true") {
    result.reason = "fusion_anchor_invalid";
  } else if (fields.position_fused != "true") {
    result.reason = "fusion_position_not_fused";
  } else if (fields.yaw_fused != "true") {
    result.reason = "fusion_yaw_not_fused";
  } else if (fields.last_fix_state != "good") {
    result.reason = "fusion_fix_not_good";
  } else {
    result.healthy = true;
    result.reason = "strict_fusion_health_ok";
  }
  return result;
}

FusionHealthEvaluation evaluateExistingFusionHealthFreshness(
  bool strict_fields_healthy,
  const std::string & strict_fields_reason,
  double diagnostic_stamp_sec,
  double sample_stamp_sec,
  double now_sec,
  double max_age_sec,
  double max_future_skew_sec)
{
  FusionHealthEvaluation result;
  if (!std::isfinite(diagnostic_stamp_sec) || diagnostic_stamp_sec <= 0.0 ||
    !std::isfinite(sample_stamp_sec) || sample_stamp_sec <= 0.0 ||
    !std::isfinite(now_sec) || now_sec <= 0.0 || !finitePositive(max_age_sec) ||
    !std::isfinite(max_future_skew_sec) || max_future_skew_sec < 0.0)
  {
    result.reason = "fusion_diagnostics_invalid_time";
    return result;
  }
  const double sample_age_sec = sample_stamp_sec - diagnostic_stamp_sec;
  const double now_age_sec = now_sec - diagnostic_stamp_sec;
  result.age_sec = now_age_sec;
  if (sample_age_sec < -max_future_skew_sec || sample_age_sec > max_age_sec) {
    result.reason = "fusion_diagnostics_sample_age_gate";
  } else if (now_age_sec < -max_future_skew_sec || now_age_sec > max_age_sec) {
    result.reason = "fusion_diagnostics_stale";
  } else if (!strict_fields_healthy) {
    result.reason = strict_fields_reason;
  } else {
    result.healthy = true;
    result.reason = "strict_fusion_health_ok";
  }
  return result;
}

FusionAuthorityTimingEvaluation evaluateFusionAuthorityTiming(
  double source_stamp_sec,
  double publish_stamp_sec,
  double received_stamp_sec,
  double max_age_sec,
  double max_future_skew_sec)
{
  FusionAuthorityTimingEvaluation result;
  if (!std::isfinite(source_stamp_sec) || source_stamp_sec <= 0.0 ||
    !std::isfinite(publish_stamp_sec) || publish_stamp_sec <= 0.0 ||
    !std::isfinite(received_stamp_sec) || received_stamp_sec <= 0.0 ||
    !finitePositive(max_age_sec) || !std::isfinite(max_future_skew_sec) ||
    max_future_skew_sec < 0.0)
  {
    result.reason = "fusion_authority_invalid_time";
    return result;
  }
  result.source_age_sec = publish_stamp_sec - source_stamp_sec;
  result.transport_age_sec = received_stamp_sec - publish_stamp_sec;
  if (result.source_age_sec < -max_future_skew_sec ||
    result.source_age_sec > max_age_sec)
  {
    result.reason = "fusion_authority_source_stamp_age_gate";
  } else if (result.transport_age_sec < -max_future_skew_sec ||
    result.transport_age_sec > max_age_sec)
  {
    result.reason = "fusion_authority_receive_age_gate";
  } else {
    result.valid = true;
    result.reason = "fusion_authority_timing_ok";
  }
  return result;
}

FusionAuthorityOrderEvaluation evaluateFusionAuthorityOrder(
  bool previous_received,
  std::uint64_t previous_session_id,
  std::uint64_t previous_sequence,
  std::uint64_t previous_stamp_ns,
  bool session_retired,
  std::uint64_t session_id,
  std::uint64_t sequence,
  std::uint64_t stamp_ns)
{
  FusionAuthorityOrderEvaluation result;
  if (session_id == 0U || sequence == 0U || stamp_ns == 0U) {
    result.reason = "fusion_authority_invalid_order_fields";
  } else if (session_retired) {
    result.reason = "fusion_authority_retired_session_event";
  } else if (!previous_received) {
    result.accepted = true;
    result.reason = "fusion_authority_first_event";
  } else if (session_id == previous_session_id && sequence <= previous_sequence) {
    result.reason = "fusion_authority_sequence_not_increasing";
  } else if (session_id == previous_session_id &&
    (previous_stamp_ns == 0U || stamp_ns < previous_stamp_ns))
  {
    result.reason = "fusion_authority_stamp_backstep";
  } else if (session_id != previous_session_id &&
    (previous_stamp_ns == 0U || stamp_ns <= previous_stamp_ns))
  {
    result.reason = "fusion_authority_new_session_stamp_not_increasing";
  } else {
    result.accepted = true;
    result.reason = session_id == previous_session_id ?
      "fusion_authority_sequence_ok" : "fusion_authority_new_session";
  }
  return result;
}

FusionHealthEvaluation applyExistingFusionRearmGate(
  const FusionHealthEvaluation & input,
  bool qualifying_unhealthy_observation,
  FusionRearmState & state)
{
  if (!state.required) {
    return input;
  }
  if (!input.healthy) {
    if (qualifying_unhealthy_observation) {
      state.saw_unhealthy = true;
    }
    state.rearmed = false;
    return input;
  }
  if (!state.saw_unhealthy) {
    FusionHealthEvaluation blocked = input;
    blocked.healthy = false;
    blocked.reason = "fusion_rearm_waiting_for_unhealthy_transition";
    state.rearmed = false;
    return blocked;
  }
  state.required = false;
  state.rearmed = true;
  return input;
}

ExistingFusionAnchorTracker::ExistingFusionAnchorTracker(FusionAnchorConfig config)
: config_(std::move(config))
{
  if (config_.stable_candidate_count < 3U || config_.stable_candidate_count > 20U ||
    !finitePositive(config_.candidate_min_interval_sec) ||
    !finitePositive(config_.candidate_max_gap_sec) ||
    config_.candidate_max_gap_sec <= config_.candidate_min_interval_sec ||
    !finitePositive(config_.stable_max_base_translation_m) ||
    !finitePositive(config_.stable_max_yaw_rad) || config_.stable_max_yaw_rad > kPi ||
    !finitePositive(config_.tracking_max_base_translation_m) ||
    !finitePositive(config_.tracking_max_yaw_rad) || config_.tracking_max_yaw_rad > kPi ||
    !finitePositive(config_.max_translation_rate_mps) ||
    !finitePositive(config_.max_yaw_rate_radps) ||
    !finitePositive(config_.max_translation_step_m) ||
    !finitePositive(config_.max_yaw_step_rad) ||
    !finitePositive(config_.max_step_dt_sec))
  {
    throw std::invalid_argument("invalid existing-fusion anchor configuration");
  }
  reset("waiting_healthy_fusion");
}

void ExistingFusionAnchorTracker::reset(const std::string & reason)
{
  ++activation_epoch_;
  fusion_healthy_ = false;
  global_output_ready_ = false;
  state_ = FusionAnchorState::WAITING_HEALTHY;
  target_anchor_ = Pose2{};
  applied_anchor_ = Pose2{};
  anchor_covariance_.setIdentity();
  has_last_step_stamp_ = false;
  last_step_stamp_sec_ = 0.0;
  activation_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
  activation_candidate_yaw_rad_ = std::numeric_limits<double>::quiet_NaN();
  last_applied_base_translation_m_ = 0.0;
  last_applied_yaw_rad_ = 0.0;
  health_reason_ = reason;
  activation_reason_ = reason;
  resetStableCandidates(reason);
  last_reason_ = reason;
}

void ExistingFusionAnchorTracker::resetStableCandidates(const std::string & reason)
{
  has_stable_reference_ = false;
  stable_reference_anchor_ = Pose2{};
  stable_candidate_count_ = 0U;
  has_last_candidate_stamp_ = false;
  last_candidate_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
  candidate_base_translation_m_ = std::numeric_limits<double>::quiet_NaN();
  candidate_yaw_delta_rad_ = std::numeric_limits<double>::quiet_NaN();
  last_reason_ = reason;
}

void ExistingFusionAnchorTracker::setFusionHealthy(
  bool healthy, double stamp_sec, const std::string & reason)
{
  if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
    healthy = false;
  }
  health_reason_ = reason;
  if (healthy == fusion_healthy_) {
    return;
  }

  fusion_healthy_ = healthy;
  if (!healthy) {
    // Discard any unapplied target at the health boundary. During an outage,
    // reacquisition, recovery, or stale diagnostic interval both anchor
    // transforms are bit-for-bit frozen at the last published transform.
    if (global_output_ready_) {
      target_anchor_ = applied_anchor_;
      state_ = FusionAnchorState::FROZEN;
      ++freeze_count_;
    } else {
      state_ = FusionAnchorState::WAITING_HEALTHY;
      activation_reason_ = reason;
    }
    resetStableCandidates(reason);
    has_last_step_stamp_ = std::isfinite(stamp_sec) && stamp_sec > 0.0;
    last_step_stamp_sec_ = has_last_step_stamp_ ? stamp_sec : 0.0;
    return;
  }

  resetStableCandidates(global_output_ready_ ?
    "waiting_stable_recovery_candidates" : "waiting_stable_startup_candidates");
  state_ = global_output_ready_ ?
    FusionAnchorState::STABILIZING_RECOVERY :
    FusionAnchorState::STABILIZING_STARTUP;
  if (!global_output_ready_) {
    activation_reason_ = "waiting_stable_startup_candidates";
  }
  // This prevents a long unhealthy interval from becoming correction dt.
  has_last_step_stamp_ = true;
  last_step_stamp_sec_ = stamp_sec;
}

bool ExistingFusionAnchorTracker::candidateFiniteAndPsd(
  const FusionAnchorCandidate & candidate) const
{
  if (!std::isfinite(candidate.stamp_sec) || candidate.stamp_sec <= 0.0 ||
    !candidate.anchor.finite() || !candidate.local_base.finite() ||
    !candidate.covariance.allFinite() ||
    (candidate.covariance - candidate.covariance.transpose()).norm() > 1.0e-6)
  {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(candidate.covariance);
  return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -1.0e-9;
}

FusionAnchorUpdate ExistingFusionAnchorTracker::observeCandidate(
  const FusionAnchorCandidate & candidate)
{
  FusionAnchorUpdate update;
  update.state = state_;
  update.stable_candidate_count = stable_candidate_count_;
  if (!fusion_healthy_) {
    ++rejected_count_;
    update.reason = "fusion_not_healthy";
    last_reason_ = update.reason;
    return update;
  }
  if (!candidateFiniteAndPsd(candidate)) {
    ++rejected_count_;
    update.reason = "invalid_candidate";
    if (state_ == FusionAnchorState::STABILIZING_STARTUP ||
      state_ == FusionAnchorState::STABILIZING_RECOVERY)
    {
      resetStableCandidates(update.reason);
    }
    last_reason_ = update.reason;
    return update;
  }
  if (has_last_candidate_stamp_ &&
    candidate.stamp_sec <= last_candidate_stamp_sec_ + 1.0e-9)
  {
    ++rejected_count_;
    update.reason = "duplicate_or_nonmonotonic_candidate";
    last_reason_ = update.reason;
    return update;
  }
  if (state_ != FusionAnchorState::TRACKING && has_last_candidate_stamp_ &&
    candidate.stamp_sec - last_candidate_stamp_sec_ <
    config_.candidate_min_interval_sec - 1.0e-9)
  {
    ++rejected_count_;
    update.reason = "candidate_interval_gate";
    last_reason_ = update.reason;
    return update;
  }
  const bool candidate_gap = has_last_candidate_stamp_ &&
    candidate.stamp_sec - last_candidate_stamp_sec_ > config_.candidate_max_gap_sec;

  update.independent = true;
  has_last_candidate_stamp_ = true;
  last_candidate_stamp_sec_ = candidate.stamp_sec;

  if (state_ == FusionAnchorState::TRACKING) {
    const Pose2 old_target_base = compose(target_anchor_, candidate.local_base);
    const Pose2 candidate_base = compose(candidate.anchor, candidate.local_base);
    candidate_base_translation_m_ = poseTranslationDistance(old_target_base, candidate_base);
    candidate_yaw_delta_rad_ = std::fabs(wrapAngle(
      candidate.anchor.yaw - target_anchor_.yaw));
    update.candidate_base_translation_m = candidate_base_translation_m_;
    update.candidate_yaw_delta_rad = candidate_yaw_delta_rad_;
    if (candidate_base_translation_m_ > config_.tracking_max_base_translation_m ||
      candidate_yaw_delta_rad_ > config_.tracking_max_yaw_rad)
    {
      ++rejected_count_;
      // A persistent new solution must not leave the tracker reporting healthy
      // TRACKING while rejecting it forever. Freeze the published transform and
      // treat this gated observation as the first recovery hypothesis. A spike
      // is discarded naturally when the next normal hypothesis restarts the
      // fixed-reference stability run.
      const Pose2 old_target_base = compose(target_anchor_, candidate.local_base);
      const Pose2 old_applied_base = compose(applied_anchor_, candidate.local_base);
      update.anchor_frozen = true;
      update.frozen_residual_x_m = old_target_base.x - old_applied_base.x;
      update.frozen_residual_y_m = old_target_base.y - old_applied_base.y;
      update.frozen_residual_yaw_rad = wrapAngle(
        old_target_base.yaw - old_applied_base.yaw);
      target_anchor_ = applied_anchor_;
      state_ = FusionAnchorState::STABILIZING_RECOVERY;
      ++freeze_count_;
      resetStableCandidates("tracking_innovation_started_recovery");
      has_stable_reference_ = true;
      stable_reference_anchor_ = candidate.anchor;
      stable_candidate_count_ = 1U;
      has_last_candidate_stamp_ = true;
      last_candidate_stamp_sec_ = candidate.stamp_sec;
      candidate_base_translation_m_ = update.candidate_base_translation_m;
      candidate_yaw_delta_rad_ = update.candidate_yaw_delta_rad;
      has_last_step_stamp_ = true;
      last_step_stamp_sec_ = candidate.stamp_sec;
      update.stable_candidate_count = 1U;
      update.state = state_;
      update.reason = "tracking_innovation_started_recovery";
      last_reason_ = update.reason;
      return update;
    }
    ++accepted_count_;
    ++target_update_count_;
    target_anchor_ = candidate.anchor;
    anchor_covariance_ = projectCovariancePsd(candidate.covariance);
    update.accepted = true;
    update.target_updated = true;
    update.reason = "tracking_target_updated";
    update.state = state_;
    last_reason_ = update.reason;
    return update;
  }

  if (state_ != FusionAnchorState::STABILIZING_STARTUP &&
    state_ != FusionAnchorState::STABILIZING_RECOVERY)
  {
    ++rejected_count_;
    update.reason = "candidate_in_inactive_state";
    last_reason_ = update.reason;
    return update;
  }

  bool stable = !candidate_gap;
  if (has_stable_reference_) {
    const Pose2 reference_base = compose(stable_reference_anchor_, candidate.local_base);
    const Pose2 candidate_base = compose(candidate.anchor, candidate.local_base);
    candidate_base_translation_m_ = poseTranslationDistance(reference_base, candidate_base);
    candidate_yaw_delta_rad_ = std::fabs(wrapAngle(
      candidate.anchor.yaw - stable_reference_anchor_.yaw));
    stable = stable &&
      candidate_base_translation_m_ <= config_.stable_max_base_translation_m &&
      candidate_yaw_delta_rad_ <= config_.stable_max_yaw_rad;
  } else {
    candidate_base_translation_m_ = std::numeric_limits<double>::quiet_NaN();
    candidate_yaw_delta_rad_ = std::numeric_limits<double>::quiet_NaN();
  }
  if (!has_stable_reference_ || !stable) {
    stable_reference_anchor_ = candidate.anchor;
    has_stable_reference_ = true;
    stable_candidate_count_ = 1U;
  } else {
    ++stable_candidate_count_;
  }
  ++accepted_count_;
  update.accepted = true;
  update.candidate_base_translation_m = candidate_base_translation_m_;
  update.candidate_yaw_delta_rad = candidate_yaw_delta_rad_;
  update.stable_candidate_count = stable_candidate_count_;

  // Refresh the step clock at each independent recovery observation so the
  // eventual first correction is bounded by one live observation interval,
  // never by the accumulated outage duration.
  has_last_step_stamp_ = true;
  last_step_stamp_sec_ = candidate.stamp_sec;

  if (stable_candidate_count_ < config_.stable_candidate_count) {
    update.reason = stable ? "stable_candidate_accumulating" :
      (candidate_gap ? "candidate_gap_restarted" : "candidate_jump_restarted");
    last_reason_ = update.reason;
    if (!global_output_ready_) {
      activation_reason_ = update.reason;
    }
    return update;
  }

  ++target_update_count_;
  target_anchor_ = candidate.anchor;
  anchor_covariance_ = projectCovariancePsd(candidate.covariance);
  update.target_updated = true;
  if (!global_output_ready_) {
    // Nothing authoritative has been exposed yet. Make the first global pose
    // exact in the same transaction that opens the publication gate.
    applied_anchor_ = target_anchor_;
    global_output_ready_ = true;
    state_ = FusionAnchorState::TRACKING;
    activation_stamp_sec_ = candidate.stamp_sec;
    activation_candidate_yaw_rad_ = candidate.anchor.yaw;
    ++activation_count_;
    update.startup_activated = true;
    update.reason = "existing_fusion_stable_activated";
    activation_reason_ = update.reason;
    last_applied_base_translation_m_ = 0.0;
    last_applied_yaw_rad_ = 0.0;
  } else {
    // A previously published anchor must never snap after an outage. Open the
    // follower only after stable recovery evidence. The node advances it on a
    // subsequent raw-odometry callback about the latest published base pose;
    // a delayed synchronized candidate is never used as a correction pivot.
    state_ = FusionAnchorState::TRACKING;
    ++recovery_count_;
    update.recovery_resumed = true;
    update.reason = "existing_fusion_stable_recovery";
  }
  update.state = state_;
  last_reason_ = update.reason;
  return update;
}

FusionAnchorUpdate ExistingFusionAnchorTracker::advanceInternal(
  double stamp_sec, const Pose2 & current_local_base)
{
  FusionAnchorUpdate update;
  update.state = state_;
  if (!fusion_healthy_ || state_ != FusionAnchorState::TRACKING ||
    !global_output_ready_ || !std::isfinite(stamp_sec) || stamp_sec <= 0.0 ||
    !current_local_base.finite())
  {
    update.reason = "follower_not_active";
    return update;
  }
  if (!has_last_step_stamp_) {
    has_last_step_stamp_ = true;
    last_step_stamp_sec_ = stamp_sec;
    update.reason = "follower_clock_initialized";
    return update;
  }
  if (stamp_sec <= last_step_stamp_sec_ + 1.0e-9) {
    update.reason = "follower_duplicate_or_nonmonotonic_stamp";
    return update;
  }

  const double dt_sec = std::min(
    stamp_sec - last_step_stamp_sec_, config_.max_step_dt_sec);
  const double translation_limit = std::min(
    config_.max_translation_step_m, config_.max_translation_rate_mps * dt_sec);
  const double yaw_limit = std::min(
    config_.max_yaw_step_rad, config_.max_yaw_rate_radps * dt_sec);
  const BoundedAnchorStep step = stepAnchorAtBase(
    applied_anchor_, target_anchor_, current_local_base,
    translation_limit, yaw_limit);
  last_step_stamp_sec_ = stamp_sec;
  if (!step.valid) {
    update.reason = "invalid_follower_step";
    return update;
  }
  applied_anchor_ = step.anchor;
  last_applied_base_translation_m_ = step.base_translation_m;
  last_applied_yaw_rad_ = step.yaw_rad;
  update.applied_base_translation_m = step.base_translation_m;
  update.applied_yaw_rad = step.yaw_rad;
  const bool moved = step.base_translation_m > 1.0e-12 ||
    std::fabs(step.yaw_rad) > 1.0e-12;
  if (moved) {
    ++applied_step_count_;
    update.follower_advanced = true;
  }
  update.reason = moved ? "bounded_follower_step" : "follower_at_target";
  return update;
}

FusionAnchorUpdate ExistingFusionAnchorTracker::advance(
  double stamp_sec, const Pose2 & current_local_base)
{
  return advanceInternal(stamp_sec, current_local_base);
}

}  // namespace pure_precision_global_localizer
