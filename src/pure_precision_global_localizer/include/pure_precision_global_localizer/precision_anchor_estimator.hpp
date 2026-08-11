// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "pure_precision_global_localizer/se2.hpp"

namespace pure_precision_global_localizer
{

enum class AnchorState
{
  UNINITIALIZED,
  TRACKING_XY_ONLY,
  TRACKING_SE2,
  HOLD_SOFT_GAP,
  OUTAGE
};

const char * toString(AnchorState state);

struct AnchorConfig
{
  double window_sec{20.0};
  double max_sample_gap_sec{1.0};
  double yaw_sample_min_interval_sec{0.25};
  std::size_t max_yaw_samples{100U};
  std::size_t min_yaw_samples{6U};
  double min_local_baseline_m{3.0};
  double min_map_baseline_m{3.0};
  double inlier_gate_m{1.5};
  double max_rms_m{0.75};
  double max_yaw_stddev_rad{0.15};
  double max_committed_yaw_innovation_rad{0.70};
  std::size_t activation_min_stable_yaw_candidates{3U};
  double activation_max_yaw_candidate_delta_rad{0.08};
  double bootstrap_yaw_rad{0.0};
  double hard_outage_sec{2.0};
  double min_position_variance_m2{0.0025};
  double max_position_variance_m2{25.0};

  double max_translation_rate_mps{1.0};
  double max_yaw_rate_radps{0.20};
  double max_translation_step_m{0.20};
  double max_yaw_step_rad{0.04};
  double max_correction_dt_sec{0.25};

  double unobservable_yaw_variance_rad2{2.4674011002723395};
  double min_yaw_variance_rad2{0.0004};
  double outage_xy_variance_rate_m2ps{0.02};
  double outage_yaw_variance_rate_rad2ps{0.0004};
};

struct PositionAlignmentSample
{
  double stamp_sec{0.0};
  Eigen::Vector2d local_point{Eigen::Vector2d::Zero()};
  Eigen::Vector2d map_point{Eigen::Vector2d::Zero()};
  double position_variance_m2{1.0};
};

struct AlignmentEstimate
{
  bool valid{false};
  std::string reason{"not_evaluated"};
  Pose2 anchor;
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Zero()};
  Eigen::Vector2d local_reference{Eigen::Vector2d::Zero()};
  std::size_t inlier_count{0U};
  std::size_t rejected_count{0U};
  double local_baseline_m{0.0};
  double map_baseline_m{0.0};
  double rms_m{0.0};
  double yaw_stddev_rad{0.0};
};

AlignmentEstimate estimateRobustSe2(
  const std::vector<PositionAlignmentSample> & samples,
  const AnchorConfig & config);

AlignmentEstimate estimateRobustTranslation(
  const std::vector<PositionAlignmentSample> & samples,
  double committed_yaw,
  const AnchorConfig & config);

struct AnchorUpdate
{
  bool accepted{false};
  bool initialized{false};
  bool target_updated{false};
  bool yaw_updated{false};
  bool yaw_activation_candidate{false};
  bool yaw_activation_committed{false};
  std::string reason;
  AnchorState state{AnchorState::UNINITIALIZED};
  std::size_t stable_yaw_candidate_count{0U};
  double applied_base_translation_m{0.0};
  double applied_yaw_rad{0.0};
};

struct BoundedAnchorStep
{
  bool valid{false};
  Pose2 anchor;
  double base_translation_m{0.0};
  double yaw_rad{0.0};
};

BoundedAnchorStep stepAnchorAtBase(
  const Pose2 & applied,
  const Pose2 & target,
  const Pose2 & current_local_base,
  double max_base_translation_m,
  double max_yaw_rad);

class PrecisionAnchorEstimator
{
public:
  explicit PrecisionAnchorEstimator(AnchorConfig config = {});

  AnchorUpdate observePosition(
    const PositionAlignmentSample & sample,
    const Pose2 & current_local_base);
  void observeUnusable(double stamp_sec);
  void updateTime(double stamp_sec);
  void reset(const std::string & reason = "reset");

  [[nodiscard]] bool initialized() const {return initialized_;}
  [[nodiscard]] bool positionInitialized() const {return initialized_;}
  [[nodiscard]] bool yawObserved() const {return yaw_observed_;}
  [[nodiscard]] bool yawPublishable() const {return yaw_observed_;}
  [[nodiscard]] AnchorState state() const {return state_;}
  [[nodiscard]] const Pose2 & appliedAnchor() const {return applied_anchor_;}
  [[nodiscard]] const Pose2 & targetAnchor() const {return target_anchor_;}
  [[nodiscard]] const Eigen::Matrix3d & anchorCovariance() const {return anchor_covariance_;}
  [[nodiscard]] std::size_t windowSize() const {return samples_.size();}
  [[nodiscard]] std::size_t yawWindowSize() const {return yaw_samples_.size();}
  [[nodiscard]] uint64_t yawEvaluationCount() const {return yaw_evaluation_count_;}
  [[nodiscard]] double lastUsableStamp() const {return last_usable_stamp_sec_;}
  [[nodiscard]] const std::string & lastReason() const {return last_reason_;}
  [[nodiscard]] std::size_t stableYawCandidateCount() const
  {
    return stable_yaw_candidate_count_;
  }
  [[nodiscard]] std::size_t requiredStableYawCandidateCount() const
  {
    return config_.activation_min_stable_yaw_candidates;
  }
  [[nodiscard]] double activationStamp() const {return activation_stamp_sec_;}
  [[nodiscard]] double activationCandidateYaw() const
  {
    return has_activation_candidate_ ? activation_candidate_.anchor.yaw :
           std::numeric_limits<double>::quiet_NaN();
  }
  [[nodiscard]] double activationCandidateDelta() const
  {
    return activation_candidate_delta_rad_;
  }
  [[nodiscard]] const std::string & activationReason() const {return activation_reason_;}
  [[nodiscard]] uint64_t activationEpoch() const {return activation_epoch_;}
  [[nodiscard]] uint64_t activationCommitCount() const {return activation_commit_count_;}

  Eigen::Matrix3d effectiveAnchorCovariance(double stamp_sec) const;

private:
  void pruneWindow(double newest_stamp_sec);
  std::vector<PositionAlignmentSample> contiguousYawWindow() const;
  void bootstrap(
    const PositionAlignmentSample & sample,
    const Pose2 & current_local_base,
    AnchorUpdate & update);
  void applyTowardTarget(
    double stamp_sec,
    const Pose2 & current_local_base,
    AnchorUpdate & update);
  void resetActivationCandidate(const std::string & reason);
  bool observeActivationCandidate(
    const AlignmentEstimate & estimate,
    const PositionAlignmentSample & sample,
    const Pose2 & current_local_base,
    AnchorUpdate & update);

  AnchorConfig config_;
  std::deque<PositionAlignmentSample> samples_;
  std::deque<PositionAlignmentSample> yaw_samples_;
  bool initialized_{false};
  bool yaw_observed_{false};
  AnchorState state_{AnchorState::UNINITIALIZED};
  Pose2 applied_anchor_;
  Pose2 target_anchor_;
  Eigen::Matrix3d anchor_covariance_{Eigen::Matrix3d::Zero()};
  bool has_last_usable_stamp_{false};
  double last_usable_stamp_sec_{0.0};
  bool has_last_correction_stamp_{false};
  double last_correction_stamp_sec_{0.0};
  uint64_t yaw_evaluation_count_{0U};
  std::string last_reason_{"uninitialized"};
  bool has_activation_candidate_{false};
  AlignmentEstimate activation_candidate_;
  std::size_t stable_yaw_candidate_count_{0U};
  double activation_candidate_delta_rad_{std::numeric_limits<double>::quiet_NaN()};
  double activation_stamp_sec_{std::numeric_limits<double>::quiet_NaN()};
  std::string activation_reason_{"waiting_for_position_initialization"};
  uint64_t activation_epoch_{0U};
  uint64_t activation_commit_count_{0U};
};

Eigen::Matrix3d propagateGlobalCovariance(
  const Pose2 & local_pose,
  const Pose2 & anchor,
  const Eigen::Matrix3d & local_covariance,
  const Eigen::Matrix3d & anchor_covariance);

Eigen::Matrix3d projectCovariancePsd(
  const Eigen::Matrix3d & covariance,
  double minimum_eigenvalue = 1.0e-9,
  double fallback_variance = 1.0e6);

}  // namespace pure_precision_global_localizer
