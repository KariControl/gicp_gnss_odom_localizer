// SPDX-License-Identifier: Apache-2.0
#include "pure_precision_global_localizer/precision_local_compositor.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Eigenvalues>

namespace pure_precision_global_localizer
{

namespace
{
bool validCovariance(const Eigen::Matrix3d & covariance)
{
  if (!covariance.allFinite()) {
    return false;
  }
  const Eigen::Matrix3d symmetric = 0.5 * (covariance + covariance.transpose());
  if ((covariance - covariance.transpose()).norm() > 1.0e-6) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
  return solver.info() == Eigen::Success && solver.eigenvalues().allFinite() &&
         solver.eigenvalues().minCoeff() >= -1.0e-9;
}

Eigen::Vector2d boundedVector(const Eigen::Vector2d & value, double limit)
{
  if (!(limit > 0.0) || !std::isfinite(limit)) {
    return Eigen::Vector2d::Zero();
  }
  const double norm = value.norm();
  if (!(norm > limit)) {
    return value;
  }
  return value * (limit / norm);
}
}  // namespace

PrecisionLocalCompositor::PrecisionLocalCompositor(LocalCorrectionConfig config)
: config_(config)
{
}

void PrecisionLocalCompositor::resetForEpoch(const OdomEpoch & epoch)
{
  epoch_ = epoch;
  has_epoch_ = true;
  has_matcher_session_ = false;
  matcher_session_ = 0U;
  has_correction_id_ = false;
  last_correction_id_ = 0U;
  last_sequence_ = 0U;
  retired_matcher_sessions_.clear();
  matcher_session_rebase_ = {};
  applied_correction_ = {};
  target_correction_ = {};
  correction_covariance_.setZero();
  matcher_session_base_covariance_.setZero();
  precision_frame_.clear();
  has_raw_stamp_ = false;
  last_raw_stamp_sec_ = 0.0;
}

EpochResult PrecisionLocalCompositor::observeEpoch(const OdomEpoch & epoch)
{
  if (!has_epoch_) {
    resetForEpoch(epoch);
    return EpochResult::INITIALIZED;
  }
  if (epoch == epoch_) {
    return EpochResult::UNCHANGED;
  }
  if (retired_odom_sessions_.count(epoch.session) != 0U) {
    return EpochResult::REJECTED_RETIRED;
  }
  if (epoch.session == epoch_.session && epoch.generation < epoch_.generation) {
    return EpochResult::REJECTED_RETIRED;
  }
  if (retired_epochs_.count(epoch) != 0U) {
    return EpochResult::REJECTED_RETIRED;
  }
  retired_epochs_.insert(epoch_);
  if (epoch.session == epoch_.session) {
    // generation denotes a matcher/trajectory epoch, not a raw odom frame
    // change. Keep the already-visible precision frame continuous while
    // discarding all ordering and session-rebase state. The first correction
    // in the new generation is an absolute persistent transform and therefore
    // must not be composed onto the old target a second time.
    epoch_ = epoch;
    has_matcher_session_ = false;
    matcher_session_ = 0U;
    has_correction_id_ = false;
    last_correction_id_ = 0U;
    last_sequence_ = 0U;
    retired_matcher_sessions_.clear();
    matcher_session_rebase_ = {};
    matcher_session_base_covariance_.setZero();
    return EpochResult::RESET;
  }
  // A new odom session has no guaranteed coordinate continuity. Fail closed
  // at identity rather than applying a transform from the retired raw frame.
  retired_odom_sessions_.insert(epoch_.session);
  resetForEpoch(epoch);
  return EpochResult::RESET;
}

CorrectionResult PrecisionLocalCompositor::acceptCorrection(
  const LocalCorrectionObservation & observation)
{
  if (observation.epoch.session == 0U || observation.epoch.generation == 0U ||
    observation.matcher_session == 0U ||
    observation.sequence == 0U || observation.correction_id == 0U ||
    !observation.precision_from_raw.finite() ||
    observation.precision_frame.empty() ||
    !validCovariance(observation.covariance))
  {
    return CorrectionResult::REJECTED_INVALID;
  }
  if (!has_epoch_) {
    return CorrectionResult::REJECTED_NO_EPOCH;
  }
  if (!(observation.epoch == epoch_)) {
    return CorrectionResult::REJECTED_WRONG_EPOCH;
  }
  if (!precision_frame_.empty() && observation.precision_frame != precision_frame_) {
    return CorrectionResult::REJECTED_FRAME;
  }
  if (has_matcher_session_ && observation.matcher_session != matcher_session_ &&
    retired_matcher_sessions_.count(observation.matcher_session) != 0U)
  {
    return CorrectionResult::REJECTED_RETIRED_SESSION;
  }

  // matcher session ids are opaque. The exact scan sequence is the writer-
  // independent ordering fence, so check it before accepting/rebasing a new
  // session and clearing its local correction-id state.
  if (has_correction_id_ && observation.sequence <= last_sequence_) {
    return CorrectionResult::REJECTED_STALE;
  }

  bool rebased_session = false;
  if (!has_matcher_session_) {
    has_matcher_session_ = true;
    matcher_session_ = observation.matcher_session;
    matcher_session_rebase_ = {};
    // Cold start and same-session odom-generation changes provide an absolute
    // persistent transform observation. Replace its covariance; accumulating
    // the prior absolute observation at every generation would double-count
    // the same uncertainty. A matcher *process restart* follows the branch
    // below and deliberately retains old + new relative uncertainty.
    matcher_session_base_covariance_.setZero();
    has_correction_id_ = false;
  } else if (observation.matcher_session != matcher_session_) {
    if (retired_matcher_sessions_.count(observation.matcher_session) != 0U) {
      return CorrectionResult::REJECTED_RETIRED_SESSION;
    }
    retired_matcher_sessions_.insert(matcher_session_);
    matcher_session_ = observation.matcher_session;
    matcher_session_rebase_ = compose(
      applied_correction_, inverse(observation.precision_from_raw));
    matcher_session_base_covariance_ = correction_covariance_;
    has_correction_id_ = false;
    rebased_session = true;
  }

  if (has_correction_id_ &&
    (observation.correction_id <= last_correction_id_ ||
    observation.sequence <= last_sequence_))
  {
    return CorrectionResult::REJECTED_STALE;
  }

  precision_frame_ = observation.precision_frame;
  target_correction_ = compose(
    matcher_session_rebase_, observation.precision_from_raw);
  correction_covariance_ = matcher_session_base_covariance_ +
    0.5 * (observation.covariance + observation.covariance.transpose());
  has_correction_id_ = true;
  last_correction_id_ = observation.correction_id;
  last_sequence_ = observation.sequence;
  return rebased_session ?
    CorrectionResult::ACCEPTED_REBASED_SESSION : CorrectionResult::ACCEPTED;
}

LocalCompositionResult PrecisionLocalCompositor::composeRaw(
  const Pose2 & raw_pose, double stamp_sec)
{
  LocalCompositionResult result;
  if (!raw_pose.finite() || !std::isfinite(stamp_sec)) {
    return result;
  }

  bool can_advance = false;
  double dt_sec = 0.0;
  if (has_raw_stamp_ && stamp_sec > last_raw_stamp_sec_) {
    dt_sec = std::min(stamp_sec - last_raw_stamp_sec_, config_.max_dt_sec);
    can_advance = dt_sec > 0.0;
  }

  if (can_advance) {
    const Pose2 previous_precision = compose(applied_correction_, raw_pose);
    const Pose2 desired_precision = compose(target_correction_, raw_pose);
    const double translation_limit = std::min(
      config_.max_translation_step_m,
      config_.max_translation_rate_mps * dt_sec);
    const double yaw_limit = std::min(
      config_.max_yaw_step_rad,
      config_.max_yaw_rate_radps * dt_sec);
    const Eigen::Vector2d base_delta = boundedVector(
      Eigen::Vector2d(
        desired_precision.x - previous_precision.x,
        desired_precision.y - previous_precision.y),
      translation_limit);
    Pose2 bounded_precision;
    bounded_precision.x = previous_precision.x + base_delta.x();
    bounded_precision.y = previous_precision.y + base_delta.y();
    bounded_precision.yaw = wrapAngle(
      previous_precision.yaw + clampMagnitude(
        wrapAngle(desired_precision.yaw - previous_precision.yaw), yaw_limit));
    applied_correction_ = compose(bounded_precision, inverse(raw_pose));
    result.correction_advanced =
      base_delta.norm() > 0.0 ||
      std::fabs(wrapAngle(bounded_precision.yaw - previous_precision.yaw)) > 0.0;
  }

  if (!has_raw_stamp_ || stamp_sec > last_raw_stamp_sec_) {
    has_raw_stamp_ = true;
    last_raw_stamp_sec_ = stamp_sec;
  }
  result.valid = true;
  result.precision_pose = compose(applied_correction_, raw_pose);
  result.applied_precision_from_raw = applied_correction_;
  result.correction_covariance = correction_covariance_;
  return result;
}

}  // namespace pure_precision_global_localizer
