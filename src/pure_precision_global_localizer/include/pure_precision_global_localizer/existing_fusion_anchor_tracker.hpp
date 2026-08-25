// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include <Eigen/Core>

#include "pure_precision_global_localizer/se2.hpp"

namespace pure_precision_global_localizer
{

enum class FusionAnchorState
{
  WAITING_HEALTHY,
  STABILIZING_STARTUP,
  TRACKING,
  FROZEN,
  STABILIZING_RECOVERY
};

const char * toString(FusionAnchorState state);

struct FusionAnchorConfig
{
  std::size_t stable_candidate_count{3U};
  double candidate_min_interval_sec{0.25};
  double candidate_max_gap_sec{1.25};
  double stable_max_base_translation_m{0.50};
  double stable_max_yaw_rad{0.08};
  double tracking_max_base_translation_m{2.0};
  double tracking_max_yaw_rad{0.25};

  double max_translation_rate_mps{1.0};
  double max_yaw_rate_radps{0.20};
  double max_translation_step_m{0.20};
  double max_yaw_step_rad{0.04};
  double max_step_dt_sec{0.25};
};

struct FusionAnchorCandidate
{
  double stamp_sec{0.0};
  Pose2 anchor;
  Pose2 local_base;
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity()};
};

struct FusionAnchorUpdate
{
  bool accepted{false};
  bool independent{false};
  bool target_updated{false};
  bool startup_activated{false};
  bool recovery_resumed{false};
  bool follower_advanced{false};
  bool anchor_frozen{false};
  std::string reason{"not_evaluated"};
  FusionAnchorState state{FusionAnchorState::WAITING_HEALTHY};
  std::size_t stable_candidate_count{0U};
  double candidate_base_translation_m{std::numeric_limits<double>::quiet_NaN()};
  double candidate_yaw_delta_rad{std::numeric_limits<double>::quiet_NaN()};
  double applied_base_translation_m{0.0};
  double applied_yaw_rad{0.0};
  double frozen_residual_x_m{0.0};
  double frozen_residual_y_m{0.0};
  double frozen_residual_yaw_rad{0.0};
};

struct ExistingFusionHealthFields
{
  // Numeric values intentionally match pure_gnss_msgs/FusionAuthority.
  int authority_state{0};
  std::string recovery_state{"unknown"};
  std::string anchor_valid{"false"};
  std::string position_fused{"false"};
  std::string yaw_fused{"false"};
  std::string last_fix_state{"unknown"};
};

struct FusionHealthEvaluation
{
  bool healthy{false};
  std::string reason{"not_evaluated"};
  double age_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct FusionAuthorityTimingEvaluation
{
  bool valid{false};
  std::string reason{"not_evaluated"};
  double source_age_sec{std::numeric_limits<double>::quiet_NaN()};
  double transport_age_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct FusionAuthorityOrderEvaluation
{
  bool accepted{false};
  std::string reason{"not_evaluated"};
};

struct FusionRearmState
{
  bool required{false};
  bool saw_unhealthy{false};
  bool rearmed{true};
};

FusionHealthEvaluation evaluateStrictExistingFusionHealth(
  const ExistingFusionHealthFields & fields);

FusionHealthEvaluation evaluateExistingFusionHealthFreshness(
  bool strict_fields_healthy,
  const std::string & strict_fields_reason,
  double diagnostic_stamp_sec,
  double sample_stamp_sec,
  double now_sec,
  double max_age_sec,
  double max_future_skew_sec);

FusionAuthorityTimingEvaluation evaluateFusionAuthorityTiming(
  double source_stamp_sec,
  double publish_stamp_sec,
  double received_stamp_sec,
  double max_age_sec,
  double max_future_skew_sec);

FusionAuthorityOrderEvaluation evaluateFusionAuthorityOrder(
  bool previous_received,
  std::uint64_t previous_session_id,
  std::uint64_t previous_sequence,
  std::uint64_t previous_stamp_ns,
  bool session_retired,
  std::uint64_t session_id,
  std::uint64_t sequence,
  std::uint64_t stamp_ns);

FusionHealthEvaluation applyExistingFusionRearmGate(
  const FusionHealthEvaluation & input,
  bool qualifying_unhealthy_observation,
  FusionRearmState & state);

// existing_global is map<-base and precision_local is precision<-base.
// The returned transform is therefore map<-precision. The shared raw odometry
// pose cancels algebraically when both inputs were formed from that same pose.
Pose2 derivePrecisionAnchor(
  const Pose2 & existing_global,
  const Pose2 & precision_local);

class ExistingFusionAnchorTracker
{
public:
  explicit ExistingFusionAnchorTracker(FusionAnchorConfig config = {});

  void reset(const std::string & reason = "reset");
  void setFusionHealthy(bool healthy, double stamp_sec, const std::string & reason);
  FusionAnchorUpdate observeCandidate(const FusionAnchorCandidate & candidate);
  FusionAnchorUpdate advance(double stamp_sec, const Pose2 & current_local_base);

  [[nodiscard]] bool fusionHealthy() const {return fusion_healthy_;}
  [[nodiscard]] bool globalOutputReady() const {return global_output_ready_;}
  [[nodiscard]] FusionAnchorState state() const {return state_;}
  [[nodiscard]] const Pose2 & targetAnchor() const {return target_anchor_;}
  [[nodiscard]] const Pose2 & appliedAnchor() const {return applied_anchor_;}
  [[nodiscard]] const Eigen::Matrix3d & anchorCovariance() const {return anchor_covariance_;}
  [[nodiscard]] std::size_t stableCandidateCount() const {return stable_candidate_count_;}
  [[nodiscard]] std::size_t requiredStableCandidateCount() const
  {
    return config_.stable_candidate_count;
  }
  [[nodiscard]] double activationStamp() const {return activation_stamp_sec_;}
  [[nodiscard]] double lastCandidateStamp() const {return last_candidate_stamp_sec_;}
  [[nodiscard]] double candidateBaseTranslation() const
  {
    return candidate_base_translation_m_;
  }
  [[nodiscard]] double candidateYawDelta() const {return candidate_yaw_delta_rad_;}
  [[nodiscard]] double activationCandidateYaw() const
  {
    return activation_candidate_yaw_rad_;
  }
  [[nodiscard]] const std::string & activationReason() const {return activation_reason_;}
  [[nodiscard]] const std::string & lastReason() const {return last_reason_;}
  [[nodiscard]] const std::string & healthReason() const {return health_reason_;}
  [[nodiscard]] uint64_t activationEpoch() const {return activation_epoch_;}
  [[nodiscard]] uint64_t activationCount() const {return activation_count_;}
  [[nodiscard]] uint64_t acceptedCount() const {return accepted_count_;}
  [[nodiscard]] uint64_t rejectedCount() const {return rejected_count_;}
  [[nodiscard]] uint64_t freezeCount() const {return freeze_count_;}
  [[nodiscard]] uint64_t recoveryCount() const {return recovery_count_;}
  [[nodiscard]] uint64_t targetUpdateCount() const {return target_update_count_;}
  [[nodiscard]] uint64_t appliedStepCount() const {return applied_step_count_;}
  [[nodiscard]] double lastAppliedBaseTranslation() const
  {
    return last_applied_base_translation_m_;
  }
  [[nodiscard]] double lastAppliedYaw() const {return last_applied_yaw_rad_;}

private:
  void resetStableCandidates(const std::string & reason);
  FusionAnchorUpdate advanceInternal(double stamp_sec, const Pose2 & current_local_base);
  bool candidateFiniteAndPsd(const FusionAnchorCandidate & candidate) const;

  FusionAnchorConfig config_;
  bool fusion_healthy_{false};
  bool global_output_ready_{false};
  FusionAnchorState state_{FusionAnchorState::WAITING_HEALTHY};
  Pose2 target_anchor_;
  Pose2 applied_anchor_;
  Eigen::Matrix3d anchor_covariance_{Eigen::Matrix3d::Identity()};

  bool has_stable_reference_{false};
  Pose2 stable_reference_anchor_;
  std::size_t stable_candidate_count_{0U};
  bool has_last_candidate_stamp_{false};
  double last_candidate_stamp_sec_{std::numeric_limits<double>::quiet_NaN()};
  double candidate_base_translation_m_{std::numeric_limits<double>::quiet_NaN()};
  double candidate_yaw_delta_rad_{std::numeric_limits<double>::quiet_NaN()};

  bool has_last_step_stamp_{false};
  double last_step_stamp_sec_{0.0};
  double activation_stamp_sec_{std::numeric_limits<double>::quiet_NaN()};
  double activation_candidate_yaw_rad_{std::numeric_limits<double>::quiet_NaN()};
  double last_applied_base_translation_m_{0.0};
  double last_applied_yaw_rad_{0.0};
  std::string last_reason_{"waiting_healthy_fusion"};
  std::string health_reason_{"fusion_health_unavailable"};
  std::string activation_reason_{"waiting_healthy_fusion"};

  uint64_t activation_epoch_{0U};
  uint64_t activation_count_{0U};
  uint64_t accepted_count_{0U};
  uint64_t rejected_count_{0U};
  uint64_t freeze_count_{0U};
  uint64_t recovery_count_{0U};
  uint64_t target_update_count_{0U};
  uint64_t applied_step_count_{0U};
};

}  // namespace pure_precision_global_localizer
