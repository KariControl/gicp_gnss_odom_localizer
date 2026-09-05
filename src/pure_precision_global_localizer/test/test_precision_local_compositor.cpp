// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "pure_precision_global_localizer/precision_local_compositor.hpp"

namespace
{
using namespace pure_precision_global_localizer;

void require(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void requirePoseNear(
  const Pose2 & actual, const Pose2 & expected, double tolerance,
  const std::string & message)
{
  require(
    poseTranslationDistance(actual, expected) <= tolerance &&
    std::fabs(wrapAngle(actual.yaw - expected.yaw)) <= tolerance,
    message);
}

LocalCorrectionObservation correction(
  OdomEpoch epoch, uint64_t sequence, uint64_t session,
  uint64_t id, Pose2 transform)
{
  LocalCorrectionObservation result;
  result.epoch = epoch;
  result.sequence = sequence;
  result.matcher_session = session;
  result.correction_id = id;
  result.precision_frame = "odom_precision";
  result.precision_from_raw = transform;
  result.covariance.diagonal() << 0.04, 0.04, 0.0025;
  return result;
}
}  // namespace

int main()
{
  LocalCorrectionConfig config;
  config.max_translation_rate_mps = 1000.0;
  config.max_yaw_rate_radps = 100.0;
  config.max_translation_step_m = 100.0;
  config.max_yaw_step_rad = 3.0;
  config.max_dt_sec = 1.0;
  PrecisionLocalCompositor compositor(config);

  const OdomEpoch epoch_a{100U, 4U};
  require(compositor.observeEpoch(epoch_a) == EpochResult::INITIALIZED,
    "first scan establishes odom epoch");

  const Pose2 correction_a{2.0, -0.5, 0.2};
  require(compositor.acceptCorrection(
    correction(epoch_a, 10U, 1000U, 1U, correction_a)) == CorrectionResult::ACCEPTED,
    "first matcher session accepts absolute correction");
  const Pose2 raw{55.0, -4.0, 0.3};
  compositor.composeRaw(raw, 1.0);
  const auto applied = compositor.composeRaw(raw, 1.1);
  require(applied.valid, "raw composition valid");
  requirePoseNear(compositor.appliedCorrection(), correction_a, 1.0e-12,
    "first correction reaches applied transform");

  // Matcher B restarts from identity. Its first sample must be rebased onto the
  // currently applied A correction, not rewind precision local odometry.
  const Pose2 before_restart = compositor.appliedCorrection();
  require(compositor.acceptCorrection(
    correction(epoch_a, 20U, 2000U, 1U, Pose2{})) ==
    CorrectionResult::ACCEPTED_REBASED_SESSION,
    "new matcher session establishes a fixed continuity rebase");
  requirePoseNear(compositor.targetCorrection(), before_restart, 1.0e-12,
    "B identity preserves A correction");
  const auto after_restart = compositor.composeRaw(raw, 1.2);
  requirePoseNear(after_restart.precision_pose, applied.precision_pose, 1.0e-12,
    "matcher restart has no output rewind at fixed raw pose");

  require(compositor.acceptCorrection(
    correction(epoch_a, 11U, 1000U, 2U, Pose2{3.0, 0.0, 0.3})) ==
    CorrectionResult::REJECTED_RETIRED_SESSION,
    "late matcher A correction cannot resurrect a retired session");

  require(compositor.acceptCorrection(
    correction(epoch_a, 19U, 2500U, 1U, Pose2{})) ==
    CorrectionResult::REJECTED_STALE,
    "opaque new matcher session cannot bypass exact scan sequence ordering");

  const Pose2 b_relative{0.5, 0.1, -0.05};
  require(compositor.acceptCorrection(
    correction(epoch_a, 21U, 2000U, 2U, b_relative)) == CorrectionResult::ACCEPTED,
    "matcher B subsequent correction accepted");
  requirePoseNear(
    compositor.targetCorrection(), compose(before_restart, b_relative), 1.0e-12,
    "B correction composes exactly once onto fixed session rebase");

  require(compositor.acceptCorrection(
    correction(epoch_a, 21U, 2000U, 2U, b_relative)) == CorrectionResult::REJECTED_STALE,
    "duplicate key/id rejected");

  const OdomEpoch epoch_b{100U, 5U};
  require(compositor.observeEpoch(epoch_b) == EpochResult::RESET,
    "odom generation change resets local correction state");
  const Pose2 before_generation = compositor.appliedCorrection();
  const Pose2 target_before_generation = compositor.targetCorrection();
  requirePoseNear(compositor.appliedCorrection(), before_generation, 0.0,
    "same-session generation keeps visible correction continuous");
  requirePoseNear(compositor.targetCorrection(), target_before_generation, 0.0,
    "same-session generation holds target until a fresh absolute correction");
  require(compositor.acceptCorrection(
    correction(epoch_a, 22U, 2000U, 3U, Pose2{})) ==
    CorrectionResult::REJECTED_WRONG_EPOCH,
    "late old-generation correction rejected");
  require(compositor.observeEpoch(epoch_a) == EpochResult::REJECTED_RETIRED,
    "late old-generation scan cannot roll epoch backward");
  require(compositor.observeEpoch(OdomEpoch{100U, 3U}) == EpochResult::REJECTED_RETIRED,
    "previously unseen lower generation cannot roll active session backward");

  const Pose2 new_generation_correction = target_before_generation;
  require(compositor.acceptCorrection(
    correction(epoch_b, 1U, 3000U, 1U, new_generation_correction)) ==
    CorrectionResult::ACCEPTED,
    "new generation first correction accepted independently");
  requirePoseNear(compositor.targetCorrection(), new_generation_correction, 1.0e-12,
    "preserved generation transform is accepted absolutely, not double-composed");
  const auto generation_covariance = compositor.composeRaw(raw, 1.3).correction_covariance;
  require(std::fabs(generation_covariance(0, 0) - 0.04) < 1.0e-12,
    "absolute generation covariance replaces rather than double-counts old covariance");

  const OdomEpoch new_session{101U, 1U};
  require(compositor.observeEpoch(new_session) == EpochResult::RESET,
    "new odom session explicitly resets unknown raw-frame continuity");
  requirePoseNear(compositor.appliedCorrection(), Pose2{}, 0.0,
    "new odom session fails closed at identity");
  require(compositor.observeEpoch(OdomEpoch{100U, 6U}) == EpochResult::REJECTED_RETIRED,
    "retired odom session id is rejected for every unseen generation");

  std::cout << "PASS: precision local compositor tests\n";
  return EXIT_SUCCESS;
}
