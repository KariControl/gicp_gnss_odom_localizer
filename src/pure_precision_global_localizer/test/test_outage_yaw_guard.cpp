// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "pure_precision_global_localizer/outage_yaw_guard.hpp"
#include "pure_precision_global_localizer/se2.hpp"

namespace
{
using pure_precision_global_localizer::OutageYawGuard;
using pure_precision_global_localizer::OutageYawGuardConfig;
using pure_precision_global_localizer::OutageYawState;
using pure_precision_global_localizer::wrapAngle;

void require(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void requireNear(double actual, double expected, double tolerance, const std::string & message)
{
  require(
    std::isfinite(actual) && std::fabs(actual - expected) <= tolerance,
    message);
}

OutageYawGuardConfig testConfig()
{
  OutageYawGuardConfig config;
  config.max_trusted_age_sec = 10.0;
  config.max_trusted_variance_rad2 = 0.04;
  config.max_trusted_delta_rad = 0.35;
  config.max_offset_rate_radps = 0.20;
  config.max_offset_step_rad = 0.04;
  config.max_step_dt_sec = 0.25;
  return config;
}
}  // namespace

int main()
{
  {
    auto invalid = testConfig();
    invalid.max_trusted_age_sec = 0.0;
    bool threw = false;
    try {
      (void)OutageYawGuard(invalid);
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    require(threw, "zero trusted-reference age is rejected");
    invalid = testConfig();
    invalid.max_trusted_delta_rad = 4.0;
    threw = false;
    try {
      (void)OutageYawGuard(invalid);
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    require(threw, "trusted-yaw delta above pi is rejected");
  }

  {
    OutageYawGuard guard(testConfig());
    require(guard.state() == OutageYawState::DISARMED,
      "guard starts disarmed");
    require(std::isnan(guard.activeReferenceVariance()),
      "disarmed guard has no active outage-reference variance");
    require(guard.observeTrustedReference(10.0, 0.20, 0.30, 0.01),
      "finite independently gated reference is accepted");
    require(guard.state() == OutageYawState::READY,
      "accepted reference arms the guard");
    require(std::isnan(guard.activeReferenceVariance()),
      "ready reference is not an active outage snapshot");
    requireNear(guard.observedDelta(), 0.10, 1.0e-12,
      "observed reference delta is recorded");

    const auto healthy = guard.advance(10.05, 0.20, 1.20, true);
    require(healthy.valid && healthy.state == OutageYawState::READY,
      "healthy authority leaves ready guard transparent");
    requireNear(healthy.output_yaw_rad, 1.20, 0.0,
      "healthy output yaw is bit-exact nominal yaw");
    requireNear(healthy.additional_variance_rad2, 0.0, 0.0,
      "transparent guard adds no variance");

    // Recompute the target against the actual fusion anchor at the authority
    // edge, rather than retaining the slightly older observed delta.
    const auto edge = guard.advance(10.10, 0.22, 1.25, false);
    require(edge.valid && edge.outage_started &&
      edge.state == OutageYawState::OUTAGE_SLEW,
      "authority loss starts bounded outage slew");
    requireNear(edge.target_offset_rad, 0.08, 1.0e-12,
      "outage target uses trusted anchor minus current fusion anchor");
    requireNear(edge.applied_offset_rad, 0.0, 0.0,
      "outage edge cannot move the applied offset");
    requireNear(edge.output_yaw_rad, 1.25, 0.0,
      "first outage output is exactly continuous");
    requireNear(edge.additional_variance_rad2, 0.01 + 0.08 * 0.08, 1.0e-12,
      "trusted uncertainty and deterministic slew lag are reported");
    requireNear(guard.activeReferenceVariance(), 0.01, 0.0,
      "outage entry snapshots trusted-reference variance");

    const auto first_step = guard.advance(10.20, 0.22, 1.26, false);
    require(first_step.offset_advanced, "monotonic outage callback advances offset");
    requireNear(first_step.applied_offset_rad, 0.02, 1.0e-12,
      "outage step obeys 0.2 rad/s rate bound");
    requireNear(first_step.output_yaw_rad, 1.28, 1.0e-12,
      "orientation-only offset is applied to nominal yaw");
    requireNear(
      first_step.additional_variance_rad2, 0.01 + 0.06 * 0.06, 1.0e-12,
      "outage slew variance uses active snapshot plus remaining residual squared");
    requireNear(guard.activeReferenceVariance(), 0.01, 0.0,
      "outage step cannot mutate active reference variance");

    const double frozen_target = guard.targetOffset();
    require(!guard.observeTrustedReference(10.21, 0.22, -0.10, 0.001),
      "active outage rejects a new reference");
    requireNear(guard.targetOffset(), frozen_target, 0.0,
      "unhealthy data cannot mutate snapshotted target");
    requireNear(guard.activeReferenceVariance(), 0.01, 0.0,
      "rejected outage reference cannot mutate active variance");

    const auto duplicate = guard.advance(10.20, 0.22, 1.26, false);
    require(duplicate.valid && !duplicate.offset_advanced,
      "duplicate timestamp holds the current correction");
    requireNear(duplicate.applied_offset_rad, 0.02, 1.0e-15,
      "duplicate timestamp cannot double-apply a step");
    const auto backstep = guard.advance(10.19, 0.22, 1.26, false);
    require(!backstep.valid &&
      backstep.state == OutageYawState::OUTAGE_SLEW,
      "backstep fails closed without changing the state");
    requireNear(guard.appliedOffset(), 0.02, 1.0e-15,
      "backstep cannot mutate the visible offset");

    (void)guard.advance(10.40, 0.22, 1.30, false);
    const auto held = guard.advance(10.60, 0.22, 1.32, false);
    require(held.state == OutageYawState::OUTAGE_HOLD && held.settled,
      "bounded slew reaches immutable outage hold");
    requireNear(held.applied_offset_rad, 0.08, 1.0e-12,
      "outage hold reaches the trusted target exactly");
    requireNear(held.additional_variance_rad2, 0.01, 1.0e-12,
      "settled outage retains the snapshotted reference uncertainty");

    const auto recovery_edge = guard.advance(10.70, 0.25, 1.40, true);
    require(recovery_edge.recovery_started &&
      recovery_edge.state == OutageYawState::RECOVERY_RELEASE,
      "strict TRACKING return starts recovery release");
    requireNear(recovery_edge.applied_offset_rad, 0.08, 1.0e-15,
      "recovery edge cannot snap the visible offset");
    requireNear(recovery_edge.output_yaw_rad, 1.48, 1.0e-12,
      "first recovery output is exactly continuous with held offset");
    requireNear(
      recovery_edge.additional_variance_rad2, 0.01 + 0.08 * 0.08, 1.0e-12,
      "release edge retains active reference variance plus visible offset squared");

    require(guard.observeTrustedReference(10.75, 0.25, 0.31, 0.008),
      "healthy direct reference may refresh during release");
    require(guard.state() == OutageYawState::RECOVERY_RELEASE,
      "reference refresh cannot bypass bounded release");
    requireNear(guard.trustedYawVariance(), 0.008, 0.0,
      "release refresh updates the next-outage trusted variance");
    requireNear(guard.activeReferenceVariance(), 0.01, 0.0,
      "release refresh cannot overwrite the active outage snapshot");
    const auto release_step = guard.advance(10.80, 0.25, 1.41, true);
    requireNear(release_step.applied_offset_rad, 0.06, 1.0e-12,
      "release obeys the same rate bound");
    requireNear(
      release_step.additional_variance_rad2, 0.01 + 0.06 * 0.06, 1.0e-12,
      "release step variance uses the original snapshot plus applied offset squared");
    (void)guard.advance(11.00, 0.25, 1.42, true);
    const auto released = guard.advance(11.20, 0.25, 1.43, true);
    require(released.state == OutageYawState::READY && released.settled,
      "fresh reference leaves completed release ready for another outage");
    requireNear(released.applied_offset_rad, 0.0, 0.0,
      "release returns exactly to transparent nominal yaw");
    requireNear(released.output_yaw_rad, 1.43, 0.0,
      "completed release is bit-exact transparent");
    requireNear(released.additional_variance_rad2, 0.0, 0.0,
      "completed transparent release adds no variance");
    require(std::isnan(guard.activeReferenceVariance()),
      "completed release clears active outage-reference variance");
  }

  {
    auto config = testConfig();
    config.max_trusted_age_sec = 0.5;
    OutageYawGuard guard(config);
    require(guard.observeTrustedReference(1.0, 0.0, 0.1, 0.01),
      "staleness test reference accepted");
    const auto stale = guard.advance(2.0, 0.0, 0.4, false);
    require(stale.valid && stale.state == OutageYawState::DISARMED,
      "stale reference cannot authorize outage correction");
    requireNear(stale.output_yaw_rad, 0.4, 0.0,
      "stale-reference failure remains nominal and continuous");
    require(!guard.hasTrustedReference(), "stale reference is discarded");

    require(!guard.observeTrustedReference(2.1, 0.0, 0.1, 0.05),
      "excessive trusted-reference variance is rejected");
    require(!guard.observeTrustedReference(
        2.2, 0.0, std::numeric_limits<double>::quiet_NaN(), 0.01),
      "non-finite trusted reference is rejected");
  }

  {
    OutageYawGuard guard(testConfig());
    require(guard.observeTrustedReference(5.0, 3.10, -3.08, 0.01),
      "wrapped trusted reference is accepted");
    const auto edge = guard.advance(5.1, 3.11, 3.12, false);
    const double expected = wrapAngle(-3.08 - 3.11);
    requireNear(edge.target_offset_rad, expected, 1.0e-12,
      "outage target follows the shortest wrapped yaw delta");
    require(std::fabs(edge.target_offset_rad) < 0.1,
      "pi crossing cannot request a full-turn correction");
  }

  {
    OutageYawGuard guard(testConfig());
    require(guard.observeTrustedReference(20.0, 0.0, 0.2, 0.02),
      "re-entry test reference accepted");
    const auto entry = guard.advance(20.1, 0.0, 0.0, false);
    requireNear(entry.additional_variance_rad2, 0.02 + 0.2 * 0.2, 1.0e-12,
      "re-entry scenario snapshots its initial reference variance");
    const auto outage_step = guard.advance(20.3, 0.0, 0.0, false);
    requireNear(outage_step.applied_offset_rad, 0.04, 1.0e-12,
      "re-entry scenario creates a visible held correction");
    const auto recovery = guard.advance(20.4, 0.0, 0.1, true);
    const double before_reentry = recovery.output_yaw_rad;
    requireNear(guard.activeReferenceVariance(), 0.02, 0.0,
      "recovery retains the original active variance");

    // A lower-variance reference refresh is next-outage evidence only. On a
    // re-outage, max(previous, new) keeps the covariance from decreasing.
    require(guard.observeTrustedReference(20.45, 0.0, 0.12, 0.005),
      "lower-variance next-outage reference is accepted during release");
    requireNear(guard.trustedYawVariance(), 0.005, 0.0,
      "trusted reference refresh records its own variance");
    requireNear(guard.activeReferenceVariance(), 0.02, 0.0,
      "release refresh cannot reduce active variance");
    const auto reentry = guard.advance(20.5, 0.0, 0.1, false);
    require(reentry.outage_started &&
      (reentry.state == OutageYawState::OUTAGE_SLEW ||
      reentry.state == OutageYawState::OUTAGE_HOLD),
      "outage during release re-enters an outage state");
    requireNear(reentry.output_yaw_rad, before_reentry, 0.0,
      "outage re-entry cannot jump the current visible yaw");
    requireNear(guard.activeReferenceVariance(), 0.02, 0.0,
      "lower-variance re-entry preserves the conservative prior snapshot");
    const double reentry_residual = wrapAngle(
      reentry.target_offset_rad - reentry.applied_offset_rad);
    requireNear(
      reentry.additional_variance_rad2,
      0.02 + reentry_residual * reentry_residual, 1.0e-12,
      "re-outage variance uses conservative active snapshot and new residual");

    guard.reset("new_odom_session_reset");
    require(guard.state() == OutageYawState::DISARMED &&
      !guard.hasTrustedReference(),
      "session reset removes old-frame yaw authority");
    requireNear(guard.appliedOffset(), 0.0, 0.0,
      "session reset restores transparent offset");
    require(std::isnan(guard.activeReferenceVariance()),
      "session reset clears active outage-reference variance");
    requireNear(guard.additionalVariance(), 0.0, 0.0,
      "session reset clears additional yaw variance");
  }

  {
    OutageYawGuard guard(testConfig());
    require(guard.observeTrustedReference(30.0, 0.0, 0.2, 0.005),
      "higher-variance re-entry scenario starts with low uncertainty");
    (void)guard.advance(30.1, 0.0, 0.0, false);
    (void)guard.advance(30.3, 0.0, 0.0, false);
    const auto recovery = guard.advance(30.4, 0.0, 0.1, true);
    requireNear(recovery.additional_variance_rad2, 0.005 + 0.04 * 0.04, 1.0e-12,
      "release starts with the low active snapshot");
    require(guard.observeTrustedReference(30.45, 0.0, 0.12, 0.03),
      "higher-variance next-outage reference is accepted during release");
    requireNear(guard.activeReferenceVariance(), 0.005, 0.0,
      "higher-variance refresh is deferred until a re-outage");
    const auto reentry = guard.advance(30.5, 0.0, 0.1, false);
    requireNear(guard.activeReferenceVariance(), 0.03, 0.0,
      "re-outage raises active variance to conservative maximum");
    const double residual = wrapAngle(
      reentry.target_offset_rad - reentry.applied_offset_rad);
    requireNear(
      reentry.additional_variance_rad2, 0.03 + residual * residual, 1.0e-12,
      "higher-variance re-outage applies the outage covariance formula");
  }

  std::cout << "PASS: outage yaw guard tests\n";
  return EXIT_SUCCESS;
}
