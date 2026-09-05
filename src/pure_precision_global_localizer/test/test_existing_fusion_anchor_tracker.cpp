// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "pure_precision_global_localizer/existing_fusion_anchor_tracker.hpp"
#include "pure_precision_global_localizer/pre_clock_event_buffer.hpp"

namespace
{
using pure_precision_global_localizer::ExistingFusionAnchorTracker;
using pure_precision_global_localizer::FusionAnchorCandidate;
using pure_precision_global_localizer::FusionAnchorConfig;
using pure_precision_global_localizer::FusionAnchorState;
using pure_precision_global_localizer::ExistingFusionHealthFields;
using pure_precision_global_localizer::FusionHealthEvaluation;
using pure_precision_global_localizer::FusionRearmState;
using pure_precision_global_localizer::PreClockEventBuffer;
using pure_precision_global_localizer::PreClockReceiveAction;
using pure_precision_global_localizer::PreClockReceiveGate;
using pure_precision_global_localizer::Pose2;
using pure_precision_global_localizer::compose;
using pure_precision_global_localizer::derivePrecisionAnchor;
using pure_precision_global_localizer::evaluateExistingFusionHealthFreshness;
using pure_precision_global_localizer::evaluateFusionAuthorityOrder;
using pure_precision_global_localizer::evaluateFusionAuthorityTiming;
using pure_precision_global_localizer::evaluateStrictExistingFusionHealth;
using pure_precision_global_localizer::applyExistingFusionRearmGate;
using pure_precision_global_localizer::inverse;
using pure_precision_global_localizer::poseTranslationDistance;
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

void requirePoseNear(
  const Pose2 & actual, const Pose2 & expected, double translation_tolerance,
  double yaw_tolerance, const std::string & message)
{
  require(
    poseTranslationDistance(actual, expected) <= translation_tolerance &&
    std::fabs(wrapAngle(actual.yaw - expected.yaw)) <= yaw_tolerance,
    message);
}

FusionAnchorConfig testConfig()
{
  FusionAnchorConfig config;
  config.stable_candidate_count = 3U;
  config.candidate_min_interval_sec = 0.25;
  config.candidate_max_gap_sec = 1.25;
  config.stable_max_base_translation_m = 0.5;
  config.stable_max_yaw_rad = 0.08;
  config.tracking_max_base_translation_m = 2.0;
  config.tracking_max_yaw_rad = 0.25;
  config.max_translation_rate_mps = 1.0;
  config.max_yaw_rate_radps = 0.2;
  config.max_translation_step_m = 0.2;
  config.max_yaw_step_rad = 0.04;
  config.max_step_dt_sec = 0.25;
  return config;
}

FusionAnchorCandidate candidate(
  double stamp, const Pose2 & anchor, const Pose2 & local = Pose2{10.0, 2.0, 0.3})
{
  FusionAnchorCandidate result;
  result.stamp_sec = stamp;
  result.anchor = anchor;
  result.local_base = local;
  result.covariance = Eigen::Matrix3d::Identity() * 0.01;
  return result;
}

void activate(
  ExistingFusionAnchorTracker & tracker, const Pose2 & anchor,
  double start_stamp = 1.0)
{
  tracker.setFusionHealthy(true, start_stamp - 0.1, "strict_fusion_health_ok");
  for (int index = 0; index < 3; ++index) {
    const auto update = tracker.observeCandidate(candidate(
      start_stamp + 0.25 * static_cast<double>(index), anchor));
    require(update.accepted, "startup candidate accepted");
  }
  require(tracker.globalOutputReady(), "startup helper activates output");
}
}  // namespace

int main()
{
  {
    PreClockReceiveGate receive_gate;
    require(receive_gate.observe(-1) == PreClockReceiveAction::DEFER,
      "a negative ROS receive stamp is deferred before clock initialization");
    require(receive_gate.observe(0) == PreClockReceiveAction::DEFER,
      "a zero ROS receive stamp is deferred at startup");
    require(receive_gate.observe(1) == PreClockReceiveAction::PROCESS,
      "the first positive ROS receive stamp releases startup events");
    require(receive_gate.observedPositive(),
      "positive-clock observation is latched");
    require(receive_gate.observe(0) == PreClockReceiveAction::PROCESS,
      "clock zero after initialization reaches the strict timing rejection gate");

    PreClockEventBuffer<int> buffer(3U);
    require(!buffer.defer(1).has_value(), "the first startup event is buffered");
    require(!buffer.defer(2).has_value(), "the second startup event is buffered");
    require(!buffer.defer(3).has_value(), "the bounded buffer accepts its capacity");
    const auto evicted = buffer.defer(4);
    require(evicted.has_value() && *evicted == 1,
      "overflow explicitly returns the oldest event for fail-closed rejection");
    std::vector<int> drained;
    buffer.drain([&drained](int event) {drained.push_back(event);});
    require(drained == std::vector<int>({2, 3, 4}),
      "deferred authority events drain exactly once in DDS arrival order");
    require(buffer.empty() && buffer.size() == 0U,
      "draining leaves no hidden startup authority event");
  }

  {
    ExistingFusionHealthFields fields;
    fields.authority_state = 1;
    fields.recovery_state = "tracking";
    fields.anchor_valid = "true";
    fields.position_fused = "true";
    fields.yaw_fused = "true";
    fields.last_fix_state = "good";
    require(evaluateStrictExistingFusionHealth(fields).healthy,
      "typed healthy authority and every full-SE(2) field are required and sufficient");
    fields.authority_state = 2;
    const auto soft_hold = evaluateStrictExistingFusionHealth(fields);
    require(!soft_hold.healthy && soft_hold.reason == "fusion_soft_bad_hold",
      "soft-bad hold freezes precision authority even while map fusion remains tracking");
    fields.authority_state = 0;
    const auto unhealthy = evaluateStrictExistingFusionHealth(fields);
    require(!unhealthy.healthy && unhealthy.reason == "fusion_authority_unhealthy",
      "typed unhealthy authority fails closed");
    fields.authority_state = 1;
    fields.yaw_fused = "false";
    const auto position_only = evaluateStrictExistingFusionHealth(fields);
    require(!position_only.healthy && position_only.reason == "fusion_yaw_not_fused",
      "position-only existing fusion cannot authorize a precision global anchor");

    const auto jitter_fresh = evaluateExistingFusionHealthFreshness(
      true, "strict_fusion_health_ok", 100.0, 100.8, 101.005, 1.5, 0.25);
    require(jitter_fresh.healthy,
      "1.005 second healthy heartbeat jitter remains fresh under 1.5 second default");
    const auto stale = evaluateExistingFusionHealthFreshness(
      true, "strict_fusion_health_ok", 100.0, 100.8, 101.6, 1.5, 0.25);
    require(!stale.healthy && stale.reason == "fusion_diagnostics_stale",
      "stale diagnostics fail closed even when their fields were healthy");
    const auto future = evaluateExistingFusionHealthFreshness(
      true, "strict_fusion_health_ok", 100.0, 99.7, 100.0, 1.5, 0.25);
    require(!future.healthy && future.reason == "fusion_diagnostics_sample_age_gate",
      "excessively future diagnostic cannot authorize an older sample");

    const auto authority_timing = evaluateFusionAuthorityTiming(
      99.8, 100.0, 100.1, 1.5, 0.25);
    require(authority_timing.valid,
      "fresh source, publication, and receive stamps pass typed-authority timing");
    const auto stale_source = evaluateFusionAuthorityTiming(
      98.0, 100.0, 100.1, 1.5, 0.25);
    require(!stale_source.valid &&
      stale_source.reason == "fusion_authority_source_stamp_age_gate",
      "a stale source event fails closed");
    const auto stale_transport = evaluateFusionAuthorityTiming(
      99.8, 100.0, 101.6, 1.5, 0.25);
    require(!stale_transport.valid &&
      stale_transport.reason == "fusion_authority_receive_age_gate",
      "a typed event received too late fails closed");

    const auto first_authority = evaluateFusionAuthorityOrder(
      false, 0U, 0U, 0U, false, 10U, 1U, 100000000000U);
    require(first_authority.accepted, "the first valid authority event is accepted");
    const auto same_stamp_next_sequence = evaluateFusionAuthorityOrder(
      true, 10U, 1U, 100000000000U, false, 10U, 2U, 100000000000U);
    require(same_stamp_next_sequence.accepted,
      "a later sequence preserves two transitions at the same ROS stamp");
    const auto duplicate_sequence = evaluateFusionAuthorityOrder(
      true, 10U, 2U, 100000000000U, false, 10U, 2U, 100100000000U);
    require(!duplicate_sequence.accepted &&
      duplicate_sequence.reason == "fusion_authority_sequence_not_increasing",
      "duplicate or reordered authority sequence is rejected");
    const auto same_session_backstep = evaluateFusionAuthorityOrder(
      true, 10U, 2U, 100000000000U, false, 10U, 3U, 99900000000U);
    require(!same_session_backstep.accepted &&
      same_session_backstep.reason == "fusion_authority_stamp_backstep",
      "a higher sequence cannot hide a same-session publication-stamp backstep");
    const auto retired_session = evaluateFusionAuthorityOrder(
      true, 11U, 4U, 101000000000U, true, 10U, 99U, 102000000000U);
    require(!retired_session.accepted &&
      retired_session.reason == "fusion_authority_retired_session_event",
      "a retired producer session is rejected even with a newer timestamp");
    const auto new_session_bad_stamp = evaluateFusionAuthorityOrder(
      true, 10U, 2U, 100000000000U, false, 11U, 1U, 100000000000U);
    require(!new_session_bad_stamp.accepted &&
      new_session_bad_stamp.reason ==
      "fusion_authority_new_session_stamp_not_increasing",
      "a new producer session must advance the publication timestamp");

    struct DeferredAuthority
    {
      std::uint64_t sequence;
      double source_stamp_sec;
      double publication_stamp_sec;
      std::uint64_t publication_stamp_ns;
    };
    PreClockReceiveGate startup_gate;
    PreClockEventBuffer<DeferredAuthority> pending_authorities(4U);
    for (std::uint64_t sequence = 1U; sequence <= 3U; ++sequence) {
      require(startup_gate.observe(0) == PreClockReceiveAction::DEFER,
        "authority received before positive ROS time remains deferred");
      require(!pending_authorities.defer(
          DeferredAuthority{sequence, 99.8, 100.0, 100000000000U}).has_value(),
        "valid startup authority fits in the bounded queue");
    }
    require(startup_gate.observe(100100000000LL) == PreClockReceiveAction::PROCESS,
      "the first positive receive time opens deterministic queue draining");
    bool have_ordered_authority = false;
    std::uint64_t accepted_sequence = 0U;
    std::uint64_t accepted_stamp_ns = 0U;
    std::vector<std::uint64_t> accepted_sequences;
    pending_authorities.drain(
      [&](const DeferredAuthority & event) {
        const auto timing = evaluateFusionAuthorityTiming(
          event.source_stamp_sec, event.publication_stamp_sec, 100.1, 1.5, 0.25);
        require(timing.valid,
          "deferred authority is checked with the first defensible receive time");
        const auto ordered = evaluateFusionAuthorityOrder(
          have_ordered_authority, 10U, accepted_sequence, accepted_stamp_ns,
          false, 10U, event.sequence, event.publication_stamp_ns);
        require(ordered.accepted,
          "same-stamp authority transitions retain FIFO sequence order");
        have_ordered_authority = true;
        accepted_sequence = event.sequence;
        accepted_stamp_ns = event.publication_stamp_ns;
        accepted_sequences.push_back(event.sequence);
      });
    require(accepted_sequences == std::vector<std::uint64_t>({1U, 2U, 3U}),
      "deferred authority is timing- and order-gated exactly once in arrival order");

    PreClockReceiveGate stale_startup_gate;
    PreClockEventBuffer<DeferredAuthority> stale_pending(1U);
    require(stale_startup_gate.observe(0) == PreClockReceiveAction::DEFER,
      "stale startup fixture begins before ROS time");
    require(!stale_pending.defer(
        DeferredAuthority{1U, 99.8, 100.0, 100000000000U}).has_value(),
      "stale fixture is queued without bypassing validation");
    require(stale_startup_gate.observe(102000000000LL) == PreClockReceiveAction::PROCESS,
      "a late positive clock still releases the queued fixture");
    stale_pending.drain(
      [](const DeferredAuthority & event) {
        const auto timing = evaluateFusionAuthorityTiming(
          event.source_stamp_sec, event.publication_stamp_sec, 102.0, 1.5, 0.25);
        require(!timing.valid &&
          timing.reason == "fusion_authority_receive_age_gate",
          "queueing never exempts an authority event from the original age gate");
      });

    FusionRearmState rearm{true, false, false};
    FusionHealthEvaluation healthy{true, "strict_fusion_health_ok", 0.0};
    const auto old_tracking = applyExistingFusionRearmGate(healthy, false, rearm);
    require(!old_tracking.healthy && rearm.required && !rearm.rearmed,
      "new odom session cannot re-arm from uninterrupted old tracking");
    FusionHealthEvaluation stale_health{false, "fusion_diagnostics_stale", 2.0};
    (void)applyExistingFusionRearmGate(stale_health, false, rearm);
    require(!rearm.saw_unhealthy,
      "diagnostic transport staleness alone cannot prove fusion reinitialization");
    FusionHealthEvaluation delayed_pre_reset_warn{false, "fusion_not_tracking", 0.0};
    (void)applyExistingFusionRearmGate(delayed_pre_reset_warn, false, rearm);
    require(!rearm.saw_unhealthy,
      "queued pre-reset unhealthy status cannot satisfy the session fence");
    FusionHealthEvaluation explicit_unhealthy{false, "fusion_not_tracking", 0.0};
    (void)applyExistingFusionRearmGate(explicit_unhealthy, true, rearm);
    require(rearm.saw_unhealthy && !rearm.rearmed,
      "fresh explicit non-tracking status arms the tracking edge");
    const auto fresh_edge = applyExistingFusionRearmGate(healthy, false, rearm);
    require(fresh_edge.healthy && !rearm.required && rearm.rearmed,
      "fresh strict tracking after unhealthy observation completes re-arm");
  }

  {
    // If G=A_raw*R and P=C*R, then G*P^-1=A_raw*C^-1 for every raw pose R.
    const Pose2 raw_global_anchor{-268300.0, -57700.0, -1.18};
    const Pose2 precision_from_raw{2.0, -0.7, 0.11};
    const Pose2 expected = compose(raw_global_anchor, inverse(precision_from_raw));
    for (int index = 0; index < 20; ++index) {
      const Pose2 raw{
        0.8 * index, 0.03 * index * index,
        wrapAngle(-0.4 + 0.07 * index)};
      const Pose2 existing_global = compose(raw_global_anchor, raw);
      const Pose2 precision_local = compose(precision_from_raw, raw);
      requirePoseNear(
        derivePrecisionAnchor(existing_global, precision_local), expected,
        1.0e-9, 1.0e-12, "shared raw odometry cancels algebraically");
    }
  }

  {
    // PC2-like -70 degree startup remains silent until three independent
    // candidates, then exposes the exact anchor without a follower transient.
    const Pose2 pc2_anchor{-268297.0, -57714.0, -1.222};
    ExistingFusionAnchorTracker tracker(testConfig());
    tracker.setFusionHealthy(true, 0.5, "strict_fusion_health_ok");
    auto first = tracker.observeCandidate(candidate(1.0, pc2_anchor));
    require(first.accepted && first.stable_candidate_count == 1U,
      "first PC2 candidate starts stabilization");
    require(!tracker.globalOutputReady(), "first candidate cannot publish");

    const auto duplicate = tracker.observeCandidate(candidate(1.0, pc2_anchor));
    require(!duplicate.accepted, "duplicate timer stamp is rejected");
    const auto too_soon = tracker.observeCandidate(candidate(1.1, pc2_anchor));
    require(!too_soon.accepted, "sub-0.25 second candidate is not independent");
    require(tracker.stableCandidateCount() == 1U,
      "duplicate and fast samples do not advance startup evidence");

    const Pose2 pc2_second{pc2_anchor.x, pc2_anchor.y, pc2_anchor.yaw + 0.01};
    const Pose2 pc2_third{pc2_anchor.x, pc2_anchor.y, pc2_anchor.yaw + 0.015};
    (void)tracker.observeCandidate(candidate(1.3, pc2_second));
    require(!tracker.globalOutputReady(), "second independent candidate remains silent");
    const auto activation = tracker.observeCandidate(candidate(1.6, pc2_third));
    require(activation.startup_activated, "third candidate activates atomically");
    require(activation.stable_candidate_count == 3U,
      "activation update preserves the committed stability count");
    requireNear(activation.candidate_yaw_delta_rad, 0.015, 1.0e-12,
      "activation update preserves the committed fixed-reference yaw delta");
    require(tracker.globalOutputReady(), "PC2 startup becomes publishable");
    requirePoseNear(tracker.appliedAnchor(), pc2_third, 0.0, 0.0,
      "first applied PC2 anchor is exact");
    requirePoseNear(tracker.targetAnchor(), tracker.appliedAnchor(), 0.0, 0.0,
      "startup target/applied lag is exactly zero");
    requireNear(tracker.activationStamp(), 1.6, 1.0e-12,
      "activation retains third independent candidate stamp");
    requireNear(tracker.activationCandidateYaw(), pc2_third.yaw, 0.0,
      "activation yaw records the committed third candidate, not the first");
    require(tracker.activationCount() == 1U, "one startup activation is counted");
    tracker.setFusionHealthy(false, 1.7, "fusion_level_not_ok");
    requireNear(tracker.activationCandidateYaw(), pc2_third.yaw, 0.0,
      "committed activation yaw persists while the anchor is frozen");
  }

  {
    // Stability is relative to the first candidate in the run, preventing a
    // chain of individually small steps from accumulating into a bad commit.
    ExistingFusionAnchorTracker tracker(testConfig());
    tracker.setFusionHealthy(true, 1.0, "strict_fusion_health_ok");
    (void)tracker.observeCandidate(candidate(1.1, Pose2{0.0, 0.0, 0.0}));
    (void)tracker.observeCandidate(candidate(1.4, Pose2{0.49, 0.0, 0.0}));
    const auto chained = tracker.observeCandidate(
      candidate(1.7, Pose2{0.98, 0.0, 0.0}));
    require(chained.stable_candidate_count == 1U && !tracker.globalOutputReady(),
      "A,A+0.49,A+0.98 cannot chain into startup activation");

    (void)tracker.observeCandidate(candidate(2.0, Pose2{0.98, 0.0, 0.0}));
    require(tracker.stableCandidateCount() == 2U,
      "new stable run accumulates from restarted reference");
    const auto gap = tracker.observeCandidate(candidate(3.5, Pose2{0.98, 0.0, 0.0}));
    require(gap.stable_candidate_count == 1U && !tracker.globalOutputReady(),
      "candidate max-gap breaks consecutiveness");

    auto invalid = candidate(3.8, Pose2{0.98, 0.0, 0.0});
    invalid.covariance(0, 0) = -1.0;
    require(!tracker.observeCandidate(invalid).accepted,
      "non-PSD candidate is rejected");
    require(tracker.stableCandidateCount() == 0U,
      "invalid candidate clears startup evidence");
  }

  {
    // A moving, remote vehicle does not destabilize an identical anchor:
    // reference and candidate are compared at the same current base pivot.
    ExistingFusionAnchorTracker tracker(testConfig());
    tracker.setFusionHealthy(true, 1.0, "strict_fusion_health_ok");
    const Pose2 remote_anchor{-400000.0, 800000.0, 2.7};
    for (int index = 0; index < 3; ++index) {
      const Pose2 moving_local{
        1000.0 + 200.0 * index, -500.0 + 90.0 * index, 0.4 * index};
      const auto update = tracker.observeCandidate(candidate(
        1.1 + 0.3 * index, remote_anchor, moving_local));
      require(update.accepted, "same remote anchor remains stable while base moves");
    }
    require(tracker.globalOutputReady(),
      "moving base pivot does not prevent valid startup activation");
  }

  {
    // Normal healthy tracking updates the target but only moves the published
    // anchor by the configured latest-base pivot limits.
    ExistingFusionAnchorTracker tracker(testConfig());
    const Pose2 initial{5.0, -3.0, 0.2};
    activate(tracker, initial);
    const Pose2 changed{5.35, -3.10, 0.28};
    const auto update = tracker.observeCandidate(candidate(1.75, changed));
    require(update.accepted && update.target_updated,
      "healthy tracking candidate updates target");
    requirePoseNear(tracker.appliedAnchor(), initial, 0.0, 0.0,
      "candidate callback does not move the published anchor");
    const Pose2 latest_base{120.0, -35.0, 0.6};
    const Pose2 global_before = compose(tracker.appliedAnchor(), latest_base);
    const auto step = tracker.advance(2.0, latest_base);
    const Pose2 global_after = compose(tracker.appliedAnchor(), latest_base);
    require(step.applied_base_translation_m <= 0.2000000001,
      "tracking translation step is base-pivot bounded");
    require(std::fabs(step.applied_yaw_rad) <= 0.0400000001,
      "tracking yaw step is bounded");
    require(poseTranslationDistance(global_before, global_after) <= 0.2000000001,
      "delayed candidate is applied about the latest base pivot");
    require(poseTranslationDistance(tracker.targetAnchor(), tracker.appliedAnchor()) > 0.0 ||
      std::fabs(wrapAngle(tracker.targetAnchor().yaw - tracker.appliedAnchor().yaw)) > 0.0,
      "tracking target is not snapped into applied state");
    const uint64_t updates_before = tracker.targetUpdateCount();
    const auto continuous = tracker.observeCandidate(candidate(
      1.80, Pose2{5.36, -3.10, 0.281}, latest_base));
    require(continuous.accepted && tracker.targetUpdateCount() == updates_before + 1U,
      "tracking accepts every unique synchronized candidate below 0.25 seconds");
  }

  {
    // Outage/stale diagnostics discard an outstanding target and freeze both
    // transforms exactly. Recovery evidence cannot mutate either until N=3.
    ExistingFusionAnchorTracker tracker(testConfig());
    const Pose2 initial{12.0, -8.0, -0.4};
    activate(tracker, initial);
    (void)tracker.observeCandidate(candidate(1.75, Pose2{12.4, -8.0, -0.32}));
    tracker.setFusionHealthy(false, 1.80, "fusion_diagnostics_stale");
    require(tracker.state() == FusionAnchorState::FROZEN,
      "stale diagnostics freeze an activated anchor");
    const Pose2 frozen = tracker.appliedAnchor();
    requirePoseNear(tracker.targetAnchor(), frozen, 0.0, 0.0,
      "health loss discards outstanding target lag");
    (void)tracker.advance(100.0, Pose2{100.0, 20.0, 1.0});
    requirePoseNear(tracker.appliedAnchor(), frozen, 0.0, 0.0,
      "long outage cannot advance the anchor");
    require(!tracker.observeCandidate(candidate(100.1, initial)).accepted,
      "unhealthy candidate is fail-closed");

    tracker.setFusionHealthy(true, 100.2, "strict_fusion_health_ok");
    require(tracker.state() == FusionAnchorState::STABILIZING_RECOVERY,
      "healthy return starts recovery stabilization");
    const Pose2 recovered{12.8, -8.3, -0.26};
    (void)tracker.observeCandidate(candidate(100.3, recovered));
    (void)tracker.observeCandidate(candidate(100.6, recovered));
    requirePoseNear(tracker.targetAnchor(), frozen, 0.0, 0.0,
      "first two recovery candidates leave target frozen");
    requirePoseNear(tracker.appliedAnchor(), frozen, 0.0, 0.0,
      "first two recovery candidates leave output frozen");
    const auto resume = tracker.observeCandidate(candidate(100.9, recovered));
    require(resume.recovery_resumed && tracker.globalOutputReady(),
      "third recovery candidate resumes an already-ready output");
    requirePoseNear(tracker.appliedAnchor(), frozen, 0.0, 0.0,
      "recovery candidate callback itself remains frozen");
    const Pose2 latest_recovery_base{350.0, -120.0, 1.2};
    const Pose2 recovery_global_before = compose(frozen, latest_recovery_base);
    const auto recovery_step = tracker.advance(101.0, latest_recovery_base);
    const Pose2 recovery_global_after = compose(tracker.appliedAnchor(), latest_recovery_base);
    require(recovery_step.applied_base_translation_m <= 0.1000000001,
      "recovery translation uses one bounded live step");
    require(std::fabs(recovery_step.applied_yaw_rad) <= 0.0200000001,
      "recovery yaw never snaps after publication");
    require(poseTranslationDistance(recovery_global_before, recovery_global_after) <=
      0.2000000001,
      "recovery uses the latest base rather than the delayed candidate pivot");
    require(std::fabs(wrapAngle(
      tracker.appliedAnchor().yaw - recovered.yaw)) > 1.0e-3,
      "large recovery yaw remains behind target after first step");
  }

  {
    // A large but persistent healthy-fusion change cannot be silently rejected
    // forever. It freezes and enters the same N-candidate bounded recovery path.
    ExistingFusionAnchorTracker tracker(testConfig());
    const Pose2 initial{2.0, -1.0, 0.1};
    activate(tracker, initial);
    (void)tracker.observeCandidate(candidate(1.7, Pose2{2.3, -1.0, 0.15}));
    const Pose2 large_change{7.0, -3.0, 0.7};
    const auto gated = tracker.observeCandidate(candidate(1.8, large_change));
    require(!gated.accepted &&
      tracker.state() == FusionAnchorState::STABILIZING_RECOVERY &&
      tracker.stableCandidateCount() == 1U,
      "large tracking innovation becomes first recovery hypothesis");
    require(gated.anchor_frozen &&
      std::hypot(gated.frozen_residual_x_m, gated.frozen_residual_y_m) > 0.0 &&
      std::fabs(gated.frozen_residual_yaw_rad) > 0.0,
      "innovation freeze exposes the discarded follower residual for covariance");
    requirePoseNear(tracker.targetAnchor(), tracker.appliedAnchor(), 0.0, 0.0,
      "innovation transition freezes target and applied exactly");
    (void)tracker.observeCandidate(candidate(2.1, large_change));
    const Pose2 before_resume = tracker.appliedAnchor();
    const auto resume = tracker.observeCandidate(candidate(2.4, large_change));
    require(resume.recovery_resumed && tracker.state() == FusionAnchorState::TRACKING,
      "persistent large change resumes after three stable hypotheses");
    requirePoseNear(tracker.appliedAnchor(), before_resume, 0.0, 0.0,
      "persistent change still waits for next raw follower callback");
    const auto step = tracker.advance(2.45, Pose2{500.0, 100.0, -0.2});
    require(step.applied_base_translation_m <= 0.0500000001 &&
      std::fabs(step.applied_yaw_rad) <= 0.0100000001,
      "large persistent change resumes with a per-output bounded step");
  }

  {
    // An alternating unhealthy heartbeat breaks consecutiveness, matching the
    // strict level/state/fused/fix health contract used by the bag pipeline.
    ExistingFusionAnchorTracker tracker(testConfig());
    const Pose2 anchor{1.0, 2.0, 0.7};
    tracker.setFusionHealthy(true, 1.0, "strict_fusion_health_ok");
    (void)tracker.observeCandidate(candidate(1.1, anchor));
    (void)tracker.observeCandidate(candidate(1.4, anchor));
    tracker.setFusionHealthy(false, 1.5, "fusion_level_not_ok");
    require(tracker.stableCandidateCount() == 0U,
      "unhealthy heartbeat resets startup evidence");
    tracker.setFusionHealthy(true, 2.0, "strict_fusion_health_ok");
    (void)tracker.observeCandidate(candidate(2.1, anchor));
    (void)tracker.observeCandidate(candidate(2.4, anchor));
    require(!tracker.globalOutputReady(),
      "two post-warning candidates cannot inherit old evidence");
    (void)tracker.observeCandidate(candidate(2.7, anchor));
    require(tracker.globalOutputReady(),
      "three fresh consecutive candidates activate after warning");

    const uint64_t old_epoch = tracker.activationEpoch();
    tracker.reset("new_odom_session_reset");
    require(tracker.activationEpoch() == old_epoch + 1U,
      "new odom session advances authority epoch");
    require(!tracker.globalOutputReady() &&
      tracker.state() == FusionAnchorState::WAITING_HEALTHY,
      "new odom session silences global until fresh fusion evidence");
  }

  std::cout << "PASS: existing fusion anchor tracker tests\n";
  return EXIT_SUCCESS;
}
