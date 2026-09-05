// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include <Eigen/Eigenvalues>

#include "pure_precision_global_localizer/precision_anchor_estimator.hpp"

namespace
{
using pure_precision_global_localizer::AnchorConfig;
using pure_precision_global_localizer::AnchorState;
using pure_precision_global_localizer::AnchorUpdate;
using pure_precision_global_localizer::Pose2;
using pure_precision_global_localizer::PositionAlignmentSample;
using pure_precision_global_localizer::PrecisionAnchorEstimator;
using pure_precision_global_localizer::compose;
using pure_precision_global_localizer::propagateGlobalCovariance;
using pure_precision_global_localizer::stepAnchorAtBase;
using pure_precision_global_localizer::transformPoint;
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
  require(std::isfinite(actual) && std::fabs(actual - expected) <= tolerance, message);
}

PositionAlignmentSample makeSample(
  double stamp, const Pose2 & local_base, const Pose2 & true_anchor,
  const Eigen::Vector2d & lever = Eigen::Vector2d::Zero(), double variance = 0.01)
{
  PositionAlignmentSample sample;
  sample.stamp_sec = stamp;
  sample.local_point = Eigen::Vector2d(local_base.x, local_base.y) +
    pure_precision_global_localizer::rotation2(local_base.yaw) * lever;
  sample.map_point = transformPoint(true_anchor, sample.local_point);
  sample.position_variance_m2 = variance;
  return sample;
}

AnchorConfig testConfig()
{
  AnchorConfig config;
  config.window_sec = 20.0;
  config.max_sample_gap_sec = 1.0;
  config.min_yaw_samples = 5U;
  config.min_local_baseline_m = 2.0;
  config.min_map_baseline_m = 2.0;
  config.inlier_gate_m = 0.5;
  config.max_rms_m = 0.2;
  config.max_yaw_stddev_rad = 0.2;
  config.activation_min_stable_yaw_candidates = 3U;
  config.activation_max_yaw_candidate_delta_rad = 0.08;
  config.hard_outage_sec = 2.0;
  config.max_translation_rate_mps = 1.0;
  config.max_yaw_rate_radps = 0.2;
  config.max_translation_step_m = 0.2;
  config.max_yaw_step_rad = 0.04;
  config.max_correction_dt_sec = 0.25;
  return config;
}
}  // namespace

int main()
{
  {
    // A single-antenna position initializes XY immediately with a deliberately
    // unobservable yaw. The antenna lever is formed in the precision-local frame.
    const Pose2 truth{10.0, -4.0, 0.0};
    const Pose2 local{2.0, 1.0, 0.3};
    const Eigen::Vector2d lever(1.2, 0.2);
    PrecisionAnchorEstimator estimator(testConfig());
    const auto sample = makeSample(1.0, local, truth, lever);
    const auto update = estimator.observePosition(
      sample, local);
    require(update.accepted && update.initialized, "first usable position initializes");
    require(estimator.state() == AnchorState::TRACKING_XY_ONLY,
      "initial yaw must remain explicitly unobservable");
    require(!estimator.yawObserved(), "single point must not produce yaw");
    requireNear(estimator.appliedAnchor().x, truth.x, 1.0e-12, "bootstrap tx");
    requireNear(estimator.appliedAnchor().y, truth.y, 1.0e-12, "bootstrap ty");
    const Eigen::Matrix3d bootstrap_global_covariance = propagateGlobalCovariance(
      Pose2{sample.local_point.x(), sample.local_point.y(), local.yaw},
      estimator.appliedAnchor(), Eigen::Matrix3d::Zero(), estimator.anchorCovariance());
    require(bootstrap_global_covariance(0, 0) < 0.1 &&
      bootstrap_global_covariance(1, 1) < 0.1,
      "bootstrap yaw/translation cross cancels at the observed antenna point");
  }

  {
    // A 20 Hz, 210 s stream must not drive the cubic robust estimator at input
    // rate or allow an unbounded yaw window. This is a deterministic work-count
    // guard rather than a machine-dependent wall-clock assertion.
    AnchorConfig config = testConfig();
    config.window_sec = 20.0;
    config.yaw_sample_min_interval_sec = 0.25;
    config.max_yaw_samples = 100U;
    PrecisionAnchorEstimator estimator(config);
    const Pose2 stationary_truth{2.0, 3.0, 0.0};
    for (int i = 0; i < 4200; ++i) {
      const double stamp = 0.05 * static_cast<double>(i);
      const Pose2 local{0.0001 * i, 0.0, 0.0};
      estimator.observePosition(makeSample(stamp, local, stationary_truth), local);
    }
    require(estimator.yawEvaluationCount() <= 841U,
      "20 Hz yaw solve count is bounded by 0.25 s downsampling");
    require(estimator.yawWindowSize() <= 81U,
      "20 s yaw geometry window stays near 81 samples");
  }

  {
    // Low-speed motion below both baseline gates updates translation while yaw
    // remains exactly committed; trajectory/IMU labels are not direct yaw inputs.
    const Pose2 truth{4.0, 7.0, 0.8};
    PrecisionAnchorEstimator estimator(testConfig());
    for (int i = 0; i < 8; ++i) {
      const Pose2 local{0.04 * i, 0.01 * i, 0.1};
      const auto update = estimator.observePosition(
        makeSample(1.0 + 0.2 * i, local, truth), local);
      require(update.accepted, "low-speed GNSS position remains usable for XY");
    }
    require(!estimator.yawObserved(), "sub-gate baseline cannot claim absolute yaw");
    requireNear(estimator.targetAnchor().yaw, 0.0, 1.0e-15,
      "low-speed XY updates preserve committed bootstrap yaw");
  }

  {
    // PC2-like startup: the true map->precision yaw is about -70 degrees.
    // Position initialization remains available internally, but no global yaw
    // is publishable while the bootstrap yaw is still wrong. Only independent
    // downsampled SE(2) evaluations count toward activation.
    AnchorConfig config = testConfig();
    config.min_local_baseline_m = 3.0;
    config.min_map_baseline_m = 3.0;
    const Pose2 pc2_truth{-268297.0, -57714.0, -1.222};
    PrecisionAnchorEstimator estimator(config);

    for (int i = 0; i <= 4; ++i) {
      const Pose2 local{0.75 * i, 0.02 * i * i, 0.01 * i};
      const auto update = estimator.observePosition(
        makeSample(1.0 + 0.3 * i, local, pc2_truth), local);
      if (i == 0 || i == 4) {
        require(update.accepted,
          "bootstrap and first gated PC2 yaw candidate remain usable");
      }
    }
    require(estimator.positionInitialized(), "PC2 XY initializes before yaw");
    require(!estimator.yawPublishable(), "first valid yaw candidate cannot publish global");
    require(estimator.stableYawCandidateCount() == 1U,
      "only one independent yaw estimate has been observed");
    require(std::fabs(wrapAngle(
      estimator.activationCandidateYaw() - estimator.appliedAnchor().yaw)) > 1.0,
      "unpublished bootstrap retains the roughly 70 degree mismatch");

    const Pose2 too_soon_local{3.2, 0.36, 0.045};
    (void)estimator.observePosition(
      makeSample(2.35, too_soon_local, pc2_truth), too_soon_local);
    require(estimator.stableYawCandidateCount() == 1U,
      "sub-downsample observation cannot duplicate stable-yaw evidence");
    require(!estimator.yawPublishable(),
      "sub-downsample observation cannot activate global output");

    estimator.observeUnusable(2.4);
    require(estimator.state() == AnchorState::HOLD_SOFT_GAP,
      "pre-activation soft gap enters hold");
    require(estimator.stableYawCandidateCount() == 0U,
      "pre-activation soft gap resets consecutive yaw evidence");
    require(!estimator.yawPublishable(), "soft gap cannot activate global yaw");

    AnchorUpdate activation;
    for (int i = 5; i <= 7; ++i) {
      const Pose2 local{0.75 * i, 0.02 * i * i, 0.01 * i};
      activation = estimator.observePosition(
        makeSample(1.0 + 0.3 * i, local, pc2_truth), local);
      require(activation.accepted, "stable PC2 yaw candidate accepted");
      if (i < 7) {
        require(!estimator.yawPublishable(),
          "global remains silent before the third stable yaw estimate");
        requireNear(estimator.appliedAnchor().yaw, 0.0, 1.0e-15,
          "rate-limited yaw catch-up cannot begin before activation");
      }
    }
    require(activation.yaw_activation_committed,
      "third stable yaw candidate atomically activates global output");
    require(estimator.yawObserved() && estimator.yawPublishable(),
      "activated robust yaw is authoritative");
    requireNear(wrapAngle(
      estimator.appliedAnchor().yaw - estimator.targetAnchor().yaw), 0.0, 0.0,
      "activation target/applied yaw lag is exactly zero");
    requireNear(pure_precision_global_localizer::poseTranslationDistance(
      estimator.appliedAnchor(), estimator.targetAnchor()), 0.0, 0.0,
      "activation target/applied XY lag is exactly zero");
    requireNear(wrapAngle(estimator.appliedAnchor().yaw - pc2_truth.yaw), 0.0, 1.0e-9,
      "first publishable PC2 yaw has no 70 degree transient");
    requireNear(estimator.activationStamp(), 3.1, 1.0e-12,
      "activation records the exact third-candidate observation stamp");
    require(estimator.activationCommitCount() == 1U,
      "one authoritative yaw activation is recorded");

    const Pose2 frozen = estimator.appliedAnchor();
    estimator.updateTime(6.0);
    require(estimator.state() == AnchorState::OUTAGE,
      "post-activation long GNSS gap enters outage");
    require(estimator.yawPublishable(),
      "outage retains the previously observed authoritative yaw");
    requireNear(estimator.appliedAnchor().yaw, frozen.yaw, 0.0,
      "outage freezes the publishable yaw anchor");

    const uint64_t old_epoch = estimator.activationEpoch();
    estimator.reset("new_odom_session_reset");
    require(estimator.activationEpoch() == old_epoch + 1U,
      "new odom session advances the activation epoch");
    require(!estimator.positionInitialized() && !estimator.yawPublishable(),
      "new odom session suppresses global output until fresh alignment");
    require(estimator.stableYawCandidateCount() == 0U,
      "new odom session discards old stable-yaw evidence");
    require(!std::isfinite(estimator.activationStamp()),
      "new odom session clears the prior activation stamp");
    require(estimator.activationReason() == "new_odom_session_reset",
      "new odom session exposes its activation reset reason");
  }

  {
    // A very large yaw and remote coordinate origin must use the same atomic
    // first commit; base-pivot rate limits apply only after publication starts.
    AnchorConfig config = testConfig();
    const Pose2 truth_large{-450000.0, 910000.0, 2.8};
    PrecisionAnchorEstimator estimator(config);
    AnchorUpdate final_update;
    for (int i = 0; i < 7; ++i) {
      const Pose2 local{100.0 + 0.9 * i, 50.0 + 0.04 * i * i, -0.2 + 0.01 * i};
      final_update = estimator.observePosition(
        makeSample(10.0 + 0.3 * i, local, truth_large), local);
    }
    require(final_update.yaw_activation_committed,
      "large-offset third stable estimate activates atomically");
    requireNear(wrapAngle(estimator.appliedAnchor().yaw - truth_large.yaw), 0.0, 1.0e-9,
      "large yaw offset is snapped before first global output");
    requireNear(wrapAngle(
      estimator.targetAnchor().yaw - estimator.appliedAnchor().yaw), 0.0, 0.0,
      "large-offset activation has zero yaw follower lag");
    requireNear(pure_precision_global_localizer::poseTranslationDistance(
      estimator.targetAnchor(), estimator.appliedAnchor()), 0.0, 0.0,
      "large remote origin has zero activation XY follower lag");
  }

  PrecisionAnchorEstimator tracking_estimator(testConfig());
  const Pose2 truth{12.0, -3.5, 0.25};
  for (int i = 0; i < 13; ++i) {
    const Pose2 local{0.7 * i, 0.08 * i * i, 0.03 * i};
    auto sample = makeSample(10.0 + 0.2 * i, local, truth);
    if (i == 5) {
      sample.map_point += Eigen::Vector2d(20.0, -15.0);
    }
    tracking_estimator.observePosition(sample, local);
  }
  require(tracking_estimator.yawObserved(), "robust moving geometry observes yaw");
  requireNear(tracking_estimator.targetAnchor().x, truth.x, 1.0e-9,
    "robust SE2 rejects isolated tx outlier");
  requireNear(tracking_estimator.targetAnchor().y, truth.y, 1.0e-9,
    "robust SE2 rejects isolated ty outlier");
  requireNear(wrapAngle(tracking_estimator.targetAnchor().yaw - truth.yaw), 0.0, 1.0e-9,
    "robust SE2 yaw");

  {
    const Pose2 before = tracking_estimator.appliedAnchor();
    const std::size_t window_before = tracking_estimator.windowSize();
    tracking_estimator.observeUnusable(13.0);
    require(tracking_estimator.state() == AnchorState::HOLD_SOFT_GAP,
      "soft unusable status enters a hold, not reacquisition");
    requireNear(tracking_estimator.appliedAnchor().x, before.x, 0.0,
      "soft gap freezes anchor x");
    requireNear(tracking_estimator.appliedAnchor().yaw, before.yaw, 0.0,
      "soft gap freezes anchor yaw");
    require(tracking_estimator.windowSize() == window_before,
      "soft gap retains alignment history");
  }

  {
    const Pose2 before = tracking_estimator.appliedAnchor();
    const Eigen::Matrix3d covariance_before =
      tracking_estimator.effectiveAnchorCovariance(13.0);
    tracking_estimator.updateTime(20.0);
    require(tracking_estimator.state() == AnchorState::OUTAGE,
      "long age enters outage");
    requireNear(tracking_estimator.appliedAnchor().x, before.x, 0.0,
      "hard outage freezes translation");
    requireNear(tracking_estimator.appliedAnchor().yaw, before.yaw, 0.0,
      "hard outage freezes yaw");
    const Eigen::Matrix3d covariance_after =
      tracking_estimator.effectiveAnchorCovariance(20.0);
    require(covariance_after(0, 0) > covariance_before(0, 0),
      "outage increases anchor XY uncertainty");
    require(covariance_after(2, 2) > covariance_before(2, 2),
      "outage increases anchor yaw uncertainty");
  }

  {
    // The first good sample after a hard outage immediately updates XY using
    // held yaw. There is intentionally no REACQUIRING state that can deadlock.
    const double committed_yaw = tracking_estimator.targetAnchor().yaw;
    const Pose2 applied_before = tracking_estimator.appliedAnchor();
    const Pose2 local{15.0, 3.0, 0.4};
    auto recovered = makeSample(20.2, local, truth);
    recovered.map_point += Eigen::Vector2d(3.0, -1.0);
    const auto update = tracking_estimator.observePosition(recovered, local);
    require(update.accepted && update.target_updated, "first post-outage XY fix is live");
    require(update.state == AnchorState::TRACKING_SE2,
      "held observed yaw returns directly to tracking");
    requireNear(tracking_estimator.targetAnchor().yaw, committed_yaw, 1.0e-15,
      "XY-only recovery cannot alter yaw");
    require(update.applied_base_translation_m <= 0.200000000001,
      "post-outage visible correction obeys hard step bound");
    require(!update.yaw_updated,
      "post-outage stationary XY recovery cannot create a new yaw observation");
    require(std::fabs(update.applied_yaw_rad) <= 0.040000000001,
      "any pre-outage committed-yaw catch-up remains bounded");
    require(pure_precision_global_localizer::poseTranslationDistance(
      tracking_estimator.appliedAnchor(), applied_before) < 5.0,
      "bounded recovery cannot jump directly to distant target");
  }

  {
    // Bounding is evaluated at base_link, not the remote odom origin.
    const Pose2 applied{0.0, 0.0, 0.0};
    const Pose2 target{10.0, -2.0, 0.2};
    const Pose2 current_local{55.0, 0.0, 0.0};
    const auto step = stepAnchorAtBase(applied, target, current_local, 0.05, 0.01);
    require(step.valid, "base-pivot step finite");
    requireNear(step.base_translation_m, 0.05, 1.0e-12,
      "base-pivot visible XY bound");
    requireNear(step.yaw_rad, 0.01, 1.0e-12, "base-pivot yaw bound");
    const Pose2 before_global = compose(applied, current_local);
    const Pose2 after_global = compose(step.anchor, current_local);
    requireNear(std::hypot(
      after_global.x - before_global.x,
      after_global.y - before_global.y), 0.05, 1.0e-12,
      "yaw lever cannot bypass visible XY bound");
  }

  {
    Eigen::Matrix3d local_covariance;
    local_covariance << 0.04, 0.01, 0.0, 0.01, 0.09, 0.002, 0.0, 0.002, 0.01;
    Eigen::Matrix3d anchor_covariance = Eigen::Matrix3d::Zero();
    anchor_covariance.diagonal() << 0.25, 0.16, 0.04;
    const Eigen::Matrix3d covariance = propagateGlobalCovariance(
      Pose2{30.0, -12.0, 0.3}, Pose2{5.0, 2.0, 0.4},
      local_covariance, anchor_covariance);
    require(covariance.allFinite(), "global covariance finite");
    require((covariance - covariance.transpose()).norm() < 1.0e-10,
      "global covariance symmetric");
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    require(solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= 0.0,
      "global covariance PSD");
    require(covariance(0, 0) > anchor_covariance(0, 0),
      "yaw uncertainty propagates through current-base lever");

    const Eigen::Matrix3d covariance_100m = propagateGlobalCovariance(
      Pose2{100.0, 0.0, 0.0}, Pose2{}, Eigen::Matrix3d::Zero(),
      anchor_covariance);
    require(covariance_100m(1, 1) > 300.0,
      "100 m outage reports held-yaw lever uncertainty once");
    require(covariance_100m(1, 1) < 500.0,
      "100 m outage yaw lever is not double-counted in anchor XY");

    AnchorConfig fit_config = testConfig();
    std::vector<PositionAlignmentSample> far_samples;
    const Pose2 far_truth{8.0, -2.0, 0.35};
    for (int i = 0; i < 8; ++i) {
      const Pose2 local{100.0 + 0.6 * i, 0.2 * i * i, 0.0};
      far_samples.push_back(makeSample(0.2 * i, local, far_truth));
    }
    const auto fit = pure_precision_global_localizer::estimateRobustSe2(
      far_samples, fit_config);
    require(fit.valid, "far-origin SE2 covariance fit valid");
    const Pose2 at_reference{fit.local_reference.x(), fit.local_reference.y(), 0.0};
    const Eigen::Matrix3d reference_covariance = propagateGlobalCovariance(
      at_reference, fit.anchor, Eigen::Matrix3d::Zero(), fit.covariance);
    require(reference_covariance(0, 0) < 0.1 && reference_covariance(1, 1) < 0.1,
      "anchor translation/yaw cross cancels origin lever at fit centroid");
  }

  std::cout << "PASS: precision anchor estimator tests\n";
  return EXIT_SUCCESS;
}
