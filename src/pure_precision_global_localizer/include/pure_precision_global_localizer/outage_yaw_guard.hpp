// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <limits>
#include <string>

namespace pure_precision_global_localizer
{

enum class OutageYawState
{
  DISARMED,
  READY,
  OUTAGE_SLEW,
  OUTAGE_HOLD,
  RECOVERY_RELEASE
};

const char * toString(OutageYawState state);

struct OutageYawGuardConfig
{
  double max_trusted_age_sec{2.0};
  double max_trusted_variance_rad2{0.0225};
  double max_trusted_delta_rad{0.35};
  double max_offset_rate_radps{0.20};
  double max_offset_step_rad{0.04};
  double max_step_dt_sec{0.25};
};

struct OutageYawUpdate
{
  bool valid{false};
  bool outage_started{false};
  bool recovery_started{false};
  bool offset_advanced{false};
  bool settled{false};
  double output_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double applied_offset_rad{0.0};
  double target_offset_rad{0.0};
  double additional_variance_rad2{0.0};
  OutageYawState state{OutageYawState::DISARMED};
  std::string reason{"not_evaluated"};
};

// Orientation-only guard for a global pose whose nominal yaw is formed from
// an existing-fusion anchor and precision-local yaw. While that authority is
// healthy, a separately gated position-alignment anchor can be observed as a
// trusted yaw reference. On authority loss, the reference is snapshotted and
// introduced only as a bounded output-yaw offset. Translation is deliberately
// absent from this API so the nominal global XY path cannot be mutated.
class OutageYawGuard
{
public:
  explicit OutageYawGuard(OutageYawGuardConfig config = {});

  void reset(const std::string & reason = "reset");

  // Records an independently gated anchor-yaw reference. The caller must only
  // provide position-alignment estimates that have passed its own baseline,
  // inlier, residual, uncertainty and activation gates. References cannot
  // mutate an active outage snapshot.
  bool observeTrustedReference(
    double stamp_sec,
    double fusion_anchor_yaw_rad,
    double trusted_anchor_yaw_rad,
    double trusted_yaw_variance_rad2);

  // authority_tracking is true only for the strict authoritative TRACKING
  // state. FROZEN and stabilization states must pass false. The first callback
  // at each authority edge changes state without changing the applied offset,
  // preserving exact yaw continuity; later monotonic callbacks slew it.
  OutageYawUpdate advance(
    double stamp_sec,
    double fusion_anchor_yaw_rad,
    double nominal_global_yaw_rad,
    bool authority_tracking);

  [[nodiscard]] OutageYawState state() const {return state_;}
  [[nodiscard]] bool hasTrustedReference() const {return has_trusted_reference_;}
  [[nodiscard]] double trustedReferenceStamp() const {return trusted_reference_stamp_sec_;}
  [[nodiscard]] double trustedReferenceAge() const {return trusted_reference_age_sec_;}
  [[nodiscard]] double trustedAnchorYaw() const {return trusted_anchor_yaw_rad_;}
  [[nodiscard]] double observedFusionAnchorYaw() const
  {
    return observed_fusion_anchor_yaw_rad_;
  }
  [[nodiscard]] double observedDelta() const {return observed_delta_rad_;}
  [[nodiscard]] double trustedYawVariance() const {return trusted_yaw_variance_rad2_;}
  [[nodiscard]] double activeReferenceVariance() const
  {
    return active_reference_variance_rad2_;
  }
  [[nodiscard]] double appliedOffset() const {return applied_offset_rad_;}
  [[nodiscard]] double targetOffset() const {return target_offset_rad_;}
  [[nodiscard]] double additionalVariance() const;
  [[nodiscard]] const std::string & lastReason() const {return last_reason_;}
  [[nodiscard]] uint64_t acceptedReferenceCount() const
  {
    return accepted_reference_count_;
  }
  [[nodiscard]] uint64_t rejectedReferenceCount() const
  {
    return rejected_reference_count_;
  }
  [[nodiscard]] uint64_t outageCount() const {return outage_count_;}
  [[nodiscard]] uint64_t recoveryCount() const {return recovery_count_;}
  [[nodiscard]] uint64_t appliedStepCount() const {return applied_step_count_;}
  [[nodiscard]] uint64_t invalidAdvanceCount() const {return invalid_advance_count_;}
  [[nodiscard]] uint64_t resetCount() const {return reset_count_;}

private:
  bool referenceFresh(double stamp_sec);
  bool startOutage(double stamp_sec, double fusion_anchor_yaw_rad, std::string & reason);
  bool stepTowardTarget(double stamp_sec);
  void clearReference();
  void clearActiveReference();
  void disarm(const std::string & reason);
  OutageYawUpdate makeUpdate(
    double nominal_global_yaw_rad,
    const std::string & reason,
    bool outage_started = false,
    bool recovery_started = false,
    bool offset_advanced = false) const;

  OutageYawGuardConfig config_;
  OutageYawState state_{OutageYawState::DISARMED};
  bool has_trusted_reference_{false};
  double trusted_reference_stamp_sec_{std::numeric_limits<double>::quiet_NaN()};
  double trusted_reference_age_sec_{std::numeric_limits<double>::quiet_NaN()};
  double trusted_anchor_yaw_rad_{std::numeric_limits<double>::quiet_NaN()};
  double observed_fusion_anchor_yaw_rad_{std::numeric_limits<double>::quiet_NaN()};
  double observed_delta_rad_{std::numeric_limits<double>::quiet_NaN()};
  double trusted_yaw_variance_rad2_{std::numeric_limits<double>::quiet_NaN()};
  // Frozen uncertainty of the trusted reference that authorized the current
  // outage/release episode. A reference refresh during release is next-episode
  // evidence and cannot retroactively reduce this snapshot.
  double active_reference_variance_rad2_{std::numeric_limits<double>::quiet_NaN()};
  double applied_offset_rad_{0.0};
  double target_offset_rad_{0.0};
  bool has_last_advance_stamp_{false};
  double last_advance_stamp_sec_{0.0};
  std::string last_reason_{"not_initialized"};

  uint64_t accepted_reference_count_{0U};
  uint64_t rejected_reference_count_{0U};
  uint64_t outage_count_{0U};
  uint64_t recovery_count_{0U};
  uint64_t applied_step_count_{0U};
  uint64_t invalid_advance_count_{0U};
  uint64_t reset_count_{0U};
};

}  // namespace pure_precision_global_localizer
