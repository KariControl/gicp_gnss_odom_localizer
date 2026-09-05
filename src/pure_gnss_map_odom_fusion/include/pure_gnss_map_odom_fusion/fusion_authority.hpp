// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <string>

#include "pure_gnss_map_odom_fusion/gnss_recovery_controller.hpp"

namespace pure_gnss_map_odom_fusion
{

enum class FusionAuthorityState : std::uint8_t
{
  UNHEALTHY = 0,
  FULL_SE2_HEALTHY = 1,
  SOFT_BAD_HOLD = 2,
};

inline const char * toString(FusionAuthorityState state)
{
  switch (state) {
    case FusionAuthorityState::UNHEALTHY:
      return "unhealthy";
    case FusionAuthorityState::FULL_SE2_HEALTHY:
      return "full_se2_healthy";
    case FusionAuthorityState::SOFT_BAD_HOLD:
      return "soft_bad_hold";
  }
  return "unknown";
}

struct FusionAuthorityInput
{
  bool odometry_fresh{false};
  bool pending_measurement{false};
  bool anchor_valid{false};
  GnssRecoveryState recovery_state{GnssRecoveryState::UNINITIALIZED};
  bool live_fix_good{false};
  bool last_fix_good{false};
  bool last_fix_bad{false};
  std::string rejection_reason{"none"};
};

struct FusionAuthorityEvaluation
{
  FusionAuthorityState state{FusionAuthorityState::UNHEALTHY};
  std::string reason{"not_evaluated"};
  bool position_fused{false};
  bool yaw_fused{false};
};

inline bool fusionAuthorityLiveFixGood(
  bool use_gnss_status,
  bool live_status_good,
  bool last_fix_good)
{
  return use_gnss_status ? live_status_good : last_fix_good;
}

// This is deliberately stricter than the map/odom recovery-window policy.
// A soft-bad GNSS observation may preserve that internal window, but it never
// authorizes a downstream full-SE(2) anchor update.
inline FusionAuthorityEvaluation evaluateFusionAuthority(
  const FusionAuthorityInput & input)
{
  FusionAuthorityEvaluation result;
  const bool xy_only_state =
    input.recovery_state == GnssRecoveryState::RECOVERING_XY_ONLY ||
    input.recovery_state == GnssRecoveryState::TRACKING_XY_ONLY;
  result.position_fused =
    input.recovery_state == GnssRecoveryState::TRACKING ||
    input.recovery_state == GnssRecoveryState::RECOVERING || xy_only_state;
  result.yaw_fused =
    input.recovery_state == GnssRecoveryState::TRACKING ||
    input.recovery_state == GnssRecoveryState::RECOVERING;

  if (!input.odometry_fresh) {
    result.reason = "relative_odometry_unavailable";
  } else if (input.pending_measurement) {
    result.reason = "pending_absolute_measurement";
  } else if (!input.anchor_valid) {
    result.reason = "fusion_anchor_invalid";
  } else if (input.recovery_state != GnssRecoveryState::TRACKING) {
    result.reason = std::string("fusion_not_tracking:") + toString(input.recovery_state);
  } else if (
    input.rejection_reason == "gnss_soft_bad_within_grace" && input.last_fix_bad)
  {
    result.state = FusionAuthorityState::SOFT_BAD_HOLD;
    result.reason = "gnss_soft_bad_within_grace";
  } else if (!input.live_fix_good) {
    result.reason = "live_gnss_fix_not_good";
  } else if (input.rejection_reason != "none") {
    result.reason = std::string("absolute_observation_rejected:") + input.rejection_reason;
  } else if (!input.last_fix_good) {
    result.reason = "last_fix_not_good";
  } else {
    result.state = FusionAuthorityState::FULL_SE2_HEALTHY;
    result.reason = "strict_full_se2_authority_ok";
  }
  return result;
}

}  // namespace pure_gnss_map_odom_fusion
