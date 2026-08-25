// SPDX-License-Identifier: Apache-2.0
#include "pure_precision_global_localizer/outage_yaw_guard.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "pure_precision_global_localizer/se2.hpp"

namespace pure_precision_global_localizer
{

namespace
{
constexpr double kStampToleranceSec = 1.0e-9;
constexpr double kSettledToleranceRad = 1.0e-12;

bool finitePositive(double value)
{
  return std::isfinite(value) && value > 0.0;
}
}  // namespace

const char * toString(OutageYawState state)
{
  switch (state) {
    case OutageYawState::DISARMED:
      return "DISARMED";
    case OutageYawState::READY:
      return "READY";
    case OutageYawState::OUTAGE_SLEW:
      return "OUTAGE_SLEW";
    case OutageYawState::OUTAGE_HOLD:
      return "OUTAGE_HOLD";
    case OutageYawState::RECOVERY_RELEASE:
      return "RECOVERY_RELEASE";
  }
  return "UNKNOWN";
}

OutageYawGuard::OutageYawGuard(OutageYawGuardConfig config)
: config_(std::move(config))
{
  if (!finitePositive(config_.max_trusted_age_sec) ||
    !finitePositive(config_.max_trusted_variance_rad2) ||
    !finitePositive(config_.max_trusted_delta_rad) ||
    config_.max_trusted_delta_rad > kPi ||
    !finitePositive(config_.max_offset_rate_radps) ||
    !finitePositive(config_.max_offset_step_rad) ||
    config_.max_offset_step_rad > kPi ||
    !finitePositive(config_.max_step_dt_sec))
  {
    throw std::invalid_argument("invalid outage-yaw guard configuration");
  }
  reset("waiting_trusted_yaw_reference");
}

void OutageYawGuard::clearReference()
{
  has_trusted_reference_ = false;
  trusted_reference_stamp_sec_ = std::numeric_limits<double>::quiet_NaN();
  trusted_reference_age_sec_ = std::numeric_limits<double>::quiet_NaN();
  trusted_anchor_yaw_rad_ = std::numeric_limits<double>::quiet_NaN();
  observed_fusion_anchor_yaw_rad_ = std::numeric_limits<double>::quiet_NaN();
  observed_delta_rad_ = std::numeric_limits<double>::quiet_NaN();
  trusted_yaw_variance_rad2_ = std::numeric_limits<double>::quiet_NaN();
  clearActiveReference();
}

void OutageYawGuard::clearActiveReference()
{
  active_reference_variance_rad2_ = std::numeric_limits<double>::quiet_NaN();
}

void OutageYawGuard::reset(const std::string & reason)
{
  state_ = OutageYawState::DISARMED;
  clearReference();
  applied_offset_rad_ = 0.0;
  target_offset_rad_ = 0.0;
  has_last_advance_stamp_ = false;
  last_advance_stamp_sec_ = 0.0;
  last_reason_ = reason;
  ++reset_count_;
}

void OutageYawGuard::disarm(const std::string & reason)
{
  state_ = OutageYawState::DISARMED;
  clearReference();
  applied_offset_rad_ = 0.0;
  target_offset_rad_ = 0.0;
  last_reason_ = reason;
}

bool OutageYawGuard::observeTrustedReference(
  double stamp_sec,
  double fusion_anchor_yaw_rad,
  double trusted_anchor_yaw_rad,
  double trusted_yaw_variance_rad2)
{
  const bool outage_active = state_ == OutageYawState::OUTAGE_SLEW ||
    state_ == OutageYawState::OUTAGE_HOLD;
  if (outage_active) {
    ++rejected_reference_count_;
    last_reason_ = "trusted_reference_ignored_during_outage";
    return false;
  }

  if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0 ||
    !std::isfinite(fusion_anchor_yaw_rad) ||
    !std::isfinite(trusted_anchor_yaw_rad) ||
    !std::isfinite(trusted_yaw_variance_rad2) ||
    trusted_yaw_variance_rad2 < 0.0 ||
    trusted_yaw_variance_rad2 > config_.max_trusted_variance_rad2)
  {
    ++rejected_reference_count_;
    if (state_ != OutageYawState::RECOVERY_RELEASE) {
      disarm("invalid_trusted_yaw_reference");
    } else {
      last_reason_ = "invalid_trusted_yaw_reference_during_release";
    }
    return false;
  }

  if (has_trusted_reference_ &&
    stamp_sec <= trusted_reference_stamp_sec_ + kStampToleranceSec)
  {
    ++rejected_reference_count_;
    if (stamp_sec < trusted_reference_stamp_sec_ - kStampToleranceSec &&
      state_ != OutageYawState::RECOVERY_RELEASE)
    {
      disarm("nonmonotonic_trusted_yaw_reference");
    } else {
      last_reason_ = "duplicate_trusted_yaw_reference";
    }
    return false;
  }

  const double delta = wrapAngle(trusted_anchor_yaw_rad - fusion_anchor_yaw_rad);
  if (std::fabs(delta) > config_.max_trusted_delta_rad) {
    ++rejected_reference_count_;
    if (state_ != OutageYawState::RECOVERY_RELEASE) {
      disarm("trusted_yaw_delta_gate");
    } else {
      last_reason_ = "trusted_yaw_delta_gate_during_release";
    }
    return false;
  }

  has_trusted_reference_ = true;
  trusted_reference_stamp_sec_ = stamp_sec;
  trusted_reference_age_sec_ = 0.0;
  trusted_anchor_yaw_rad_ = wrapAngle(trusted_anchor_yaw_rad);
  observed_fusion_anchor_yaw_rad_ = wrapAngle(fusion_anchor_yaw_rad);
  observed_delta_rad_ = delta;
  trusted_yaw_variance_rad2_ = trusted_yaw_variance_rad2;
  ++accepted_reference_count_;
  if (state_ != OutageYawState::RECOVERY_RELEASE) {
    state_ = OutageYawState::READY;
    applied_offset_rad_ = 0.0;
    target_offset_rad_ = 0.0;
    clearActiveReference();
    last_reason_ = "trusted_yaw_reference_ready";
  } else {
    last_reason_ = "trusted_yaw_reference_refreshed_during_release";
  }
  return true;
}

bool OutageYawGuard::referenceFresh(double stamp_sec)
{
  if (!has_trusted_reference_ || !std::isfinite(stamp_sec)) {
    trusted_reference_age_sec_ = std::numeric_limits<double>::quiet_NaN();
    return false;
  }
  trusted_reference_age_sec_ = stamp_sec - trusted_reference_stamp_sec_;
  return trusted_reference_age_sec_ >= -kStampToleranceSec &&
         trusted_reference_age_sec_ <= config_.max_trusted_age_sec;
}

bool OutageYawGuard::startOutage(
  double stamp_sec, double fusion_anchor_yaw_rad, std::string & reason)
{
  if (!referenceFresh(stamp_sec)) {
    disarm("trusted_yaw_reference_stale_at_outage");
    reason = last_reason_;
    return false;
  }
  const double delta = wrapAngle(trusted_anchor_yaw_rad_ - fusion_anchor_yaw_rad);
  if (!std::isfinite(delta) || std::fabs(delta) > config_.max_trusted_delta_rad) {
    disarm("trusted_yaw_delta_gate_at_outage");
    reason = last_reason_;
    return false;
  }
  target_offset_rad_ = delta;
  active_reference_variance_rad2_ = trusted_yaw_variance_rad2_;
  state_ = std::fabs(wrapAngle(target_offset_rad_ - applied_offset_rad_)) <=
    kSettledToleranceRad ? OutageYawState::OUTAGE_HOLD : OutageYawState::OUTAGE_SLEW;
  last_advance_stamp_sec_ = stamp_sec;
  has_last_advance_stamp_ = true;
  ++outage_count_;
  ++active_reference_epoch_;
  reason = state_ == OutageYawState::OUTAGE_HOLD ?
    "trusted_outage_yaw_already_held" : "trusted_outage_yaw_slew_started";
  last_reason_ = reason;
  return true;
}

bool OutageYawGuard::stepTowardTarget(double stamp_sec)
{
  if (!has_last_advance_stamp_ || stamp_sec <= last_advance_stamp_sec_) {
    return false;
  }
  const double dt_sec = std::min(
    stamp_sec - last_advance_stamp_sec_, config_.max_step_dt_sec);
  const double limit = std::min(
    config_.max_offset_step_rad, config_.max_offset_rate_radps * dt_sec);
  const double requested = wrapAngle(target_offset_rad_ - applied_offset_rad_);
  const double step = clampMagnitude(requested, limit);
  applied_offset_rad_ = wrapAngle(applied_offset_rad_ + step);
  if (std::fabs(wrapAngle(target_offset_rad_ - applied_offset_rad_)) <=
    kSettledToleranceRad)
  {
    applied_offset_rad_ = target_offset_rad_;
  }
  last_advance_stamp_sec_ = stamp_sec;
  if (std::fabs(step) > kSettledToleranceRad) {
    ++applied_step_count_;
    return true;
  }
  return false;
}

double OutageYawGuard::additionalVariance() const
{
  if (state_ == OutageYawState::OUTAGE_SLEW ||
    state_ == OutageYawState::OUTAGE_HOLD)
  {
    const double residual = wrapAngle(target_offset_rad_ - applied_offset_rad_);
    // Every active-state entry snapshots a finite gated reference variance.
    // Use the configured maximum as a fail-closed fallback rather than
    // understating covariance if that private invariant is ever violated.
    const double reference_variance =
      std::isfinite(active_reference_variance_rad2_) ?
      active_reference_variance_rad2_ : config_.max_trusted_variance_rad2;
    return reference_variance + residual * residual;
  }
  if (state_ == OutageYawState::RECOVERY_RELEASE) {
    const double reference_variance =
      std::isfinite(active_reference_variance_rad2_) ?
      active_reference_variance_rad2_ : config_.max_trusted_variance_rad2;
    return reference_variance + applied_offset_rad_ * applied_offset_rad_;
  }
  return 0.0;
}

OutageYawUpdate OutageYawGuard::makeUpdate(
  double nominal_global_yaw_rad,
  const std::string & reason,
  bool outage_started,
  bool recovery_started,
  bool offset_advanced) const
{
  OutageYawUpdate update;
  update.valid = std::isfinite(nominal_global_yaw_rad) &&
    std::isfinite(applied_offset_rad_);
  update.outage_started = outage_started;
  update.recovery_started = recovery_started;
  update.offset_advanced = offset_advanced;
  update.settled = std::fabs(wrapAngle(target_offset_rad_ - applied_offset_rad_)) <=
    kSettledToleranceRad;
  if (update.valid) {
    update.output_yaw_rad = wrapAngle(nominal_global_yaw_rad + applied_offset_rad_);
  }
  update.applied_offset_rad = applied_offset_rad_;
  update.target_offset_rad = target_offset_rad_;
  update.additional_variance_rad2 = additionalVariance();
  update.state = state_;
  update.reason = reason;
  return update;
}

OutageYawUpdate OutageYawGuard::advance(
  double stamp_sec,
  double fusion_anchor_yaw_rad,
  double nominal_global_yaw_rad,
  bool authority_tracking)
{
  if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0 ||
    !std::isfinite(fusion_anchor_yaw_rad) ||
    !std::isfinite(nominal_global_yaw_rad))
  {
    ++invalid_advance_count_;
    last_reason_ = "invalid_outage_yaw_input";
    OutageYawUpdate update = makeUpdate(nominal_global_yaw_rad, last_reason_);
    update.valid = false;
    update.output_yaw_rad = std::numeric_limits<double>::quiet_NaN();
    return update;
  }

  if (has_last_advance_stamp_ &&
    stamp_sec < last_advance_stamp_sec_ - kStampToleranceSec)
  {
    ++invalid_advance_count_;
    last_reason_ = "nonmonotonic_outage_yaw_stamp";
    OutageYawUpdate update = makeUpdate(nominal_global_yaw_rad, last_reason_);
    update.valid = false;
    update.output_yaw_rad = std::numeric_limits<double>::quiet_NaN();
    return update;
  }
  if (!has_last_advance_stamp_) {
    has_last_advance_stamp_ = true;
    last_advance_stamp_sec_ = stamp_sec;
  }

  // A repeated physical stamp can still carry a later typed-authority
  // sequence. Process that control edge; stepTowardTarget() independently
  // rejects zero dt, so an equal stamp can never double-apply yaw correction.

  bool outage_started = false;
  bool recovery_started = false;
  bool offset_advanced = false;
  std::string reason;

  if (authority_tracking) {
    if (state_ == OutageYawState::OUTAGE_SLEW ||
      state_ == OutageYawState::OUTAGE_HOLD)
    {
      state_ = OutageYawState::RECOVERY_RELEASE;
      target_offset_rad_ = 0.0;
      last_advance_stamp_sec_ = stamp_sec;
      ++recovery_count_;
      recovery_started = true;
      reason = "outage_yaw_recovery_release_started";
    } else if (state_ == OutageYawState::RECOVERY_RELEASE) {
      offset_advanced = stepTowardTarget(stamp_sec);
      if (std::fabs(applied_offset_rad_) <= kSettledToleranceRad) {
        applied_offset_rad_ = 0.0;
        target_offset_rad_ = 0.0;
        if (referenceFresh(stamp_sec)) {
          state_ = OutageYawState::READY;
          clearActiveReference();
          reason = "outage_yaw_recovery_release_complete_ready";
        } else {
          disarm("outage_yaw_recovery_release_complete_disarmed");
          reason = last_reason_;
        }
      } else {
        reason = offset_advanced ?
          "outage_yaw_recovery_release_step" : "outage_yaw_recovery_release_hold";
      }
    } else if (state_ == OutageYawState::READY) {
      if (!referenceFresh(stamp_sec)) {
        disarm("trusted_yaw_reference_stale");
      }
      reason = last_reason_;
    } else {
      reason = "outage_yaw_guard_disarmed";
    }
  } else {
    if (state_ == OutageYawState::READY) {
      outage_started = startOutage(stamp_sec, fusion_anchor_yaw_rad, reason);
    } else if (state_ == OutageYawState::OUTAGE_SLEW) {
      offset_advanced = stepTowardTarget(stamp_sec);
      if (std::fabs(wrapAngle(target_offset_rad_ - applied_offset_rad_)) <=
        kSettledToleranceRad)
      {
        state_ = OutageYawState::OUTAGE_HOLD;
        reason = "trusted_outage_yaw_hold_reached";
      } else {
        reason = offset_advanced ?
          "trusted_outage_yaw_slew_step" : "trusted_outage_yaw_slew_hold";
      }
    } else if (state_ == OutageYawState::OUTAGE_HOLD) {
      last_advance_stamp_sec_ = stamp_sec;
      reason = "trusted_outage_yaw_held";
    } else if (state_ == OutageYawState::RECOVERY_RELEASE) {
      // A second outage during release must not snap the visible offset. Use a
      // fresh trusted anchor when possible; otherwise hold the last trusted-
      // derived output until a complete authority recovery.
      if (referenceFresh(stamp_sec)) {
        const double delta = wrapAngle(trusted_anchor_yaw_rad_ - fusion_anchor_yaw_rad);
        if (std::fabs(delta) <= config_.max_trusted_delta_rad) {
          // The visible offset still carries uncertainty from the prior outage,
          // while the new target is authorized by the refreshed reference. With
          // no cross-correlation model, max(previous, new) is the conservative
          // envelope that cannot claim less uncertainty than either authority;
          // the squared target/applied residual accounts for their deterministic
          // disagreement during the bounded transition.
          const double previous_active_variance =
            std::isfinite(active_reference_variance_rad2_) ?
            active_reference_variance_rad2_ : config_.max_trusted_variance_rad2;
          active_reference_variance_rad2_ = std::max(
            previous_active_variance, trusted_yaw_variance_rad2_);
          target_offset_rad_ = delta;
          state_ = std::fabs(wrapAngle(target_offset_rad_ - applied_offset_rad_)) <=
            kSettledToleranceRad ? OutageYawState::OUTAGE_HOLD :
            OutageYawState::OUTAGE_SLEW;
          reason = "trusted_outage_yaw_reentered_during_release";
        } else {
          target_offset_rad_ = applied_offset_rad_;
          state_ = OutageYawState::OUTAGE_HOLD;
          reason = "outage_reentry_delta_gate_held_last_offset";
        }
      } else {
        target_offset_rad_ = applied_offset_rad_;
        state_ = OutageYawState::OUTAGE_HOLD;
        reason = "outage_reentry_without_fresh_reference_held_last_offset";
      }
      last_advance_stamp_sec_ = stamp_sec;
      ++outage_count_;
      outage_started = true;
    } else {
      reason = "outage_yaw_guard_disarmed";
    }
  }

  last_reason_ = reason;
  return makeUpdate(
    nominal_global_yaw_rad, reason, outage_started, recovery_started, offset_advanced);
}

}  // namespace pure_precision_global_localizer
