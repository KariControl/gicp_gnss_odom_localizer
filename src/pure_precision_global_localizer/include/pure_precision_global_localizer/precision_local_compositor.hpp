// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <set>
#include <string>
#include <utility>

#include <Eigen/Core>

#include "pure_precision_global_localizer/se2.hpp"

namespace pure_precision_global_localizer
{

struct LocalCorrectionConfig
{
  double max_translation_rate_mps{2.0};
  double max_yaw_rate_radps{0.5};
  double max_translation_step_m{0.05};
  double max_yaw_step_rad{0.015};
  double max_dt_sec{0.05};
};

struct OdomEpoch
{
  uint64_t session{0U};
  uint64_t generation{0U};

  bool operator==(const OdomEpoch & other) const
  {
    return session == other.session && generation == other.generation;
  }

  bool operator<(const OdomEpoch & other) const
  {
    return std::tie(session, generation) < std::tie(other.session, other.generation);
  }
};

enum class EpochResult
{
  INITIALIZED,
  UNCHANGED,
  RESET,
  REJECTED_RETIRED
};

enum class CorrectionResult
{
  ACCEPTED,
  ACCEPTED_REBASED_SESSION,
  REJECTED_NO_EPOCH,
  REJECTED_WRONG_EPOCH,
  REJECTED_RETIRED_SESSION,
  REJECTED_STALE,
  REJECTED_INVALID,
  REJECTED_FRAME
};

struct LocalCorrectionObservation
{
  OdomEpoch epoch;
  uint64_t sequence{0U};
  uint64_t matcher_session{0U};
  uint64_t correction_id{0U};
  std::string precision_frame;
  Pose2 precision_from_raw;
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Zero()};
};

struct LocalCompositionResult
{
  bool valid{false};
  bool correction_advanced{false};
  Pose2 precision_pose;
  Pose2 applied_precision_from_raw;
  Eigen::Matrix3d correction_covariance{Eigen::Matrix3d::Zero()};
};

class PrecisionLocalCompositor
{
public:
  explicit PrecisionLocalCompositor(LocalCorrectionConfig config = {});

  EpochResult observeEpoch(const OdomEpoch & epoch);
  CorrectionResult acceptCorrection(const LocalCorrectionObservation & observation);
  LocalCompositionResult composeRaw(const Pose2 & raw_pose, double stamp_sec);

  [[nodiscard]] bool hasEpoch() const {return has_epoch_;}
  [[nodiscard]] OdomEpoch epoch() const {return epoch_;}
  [[nodiscard]] const Pose2 & appliedCorrection() const {return applied_correction_;}
  [[nodiscard]] const Pose2 & targetCorrection() const {return target_correction_;}
  [[nodiscard]] const std::string & precisionFrame() const {return precision_frame_;}
  [[nodiscard]] uint64_t matcherSession() const {return matcher_session_;}
  [[nodiscard]] uint64_t correctionId() const {return last_correction_id_;}

private:
  void resetForEpoch(const OdomEpoch & epoch);

  LocalCorrectionConfig config_;
  bool has_epoch_{false};
  OdomEpoch epoch_;
  std::set<OdomEpoch> retired_epochs_;
  std::set<uint64_t> retired_odom_sessions_;

  bool has_matcher_session_{false};
  uint64_t matcher_session_{0U};
  bool has_correction_id_{false};
  uint64_t last_correction_id_{0U};
  uint64_t last_sequence_{0U};
  std::set<uint64_t> retired_matcher_sessions_;
  Pose2 matcher_session_rebase_;

  Pose2 applied_correction_;
  Pose2 target_correction_;
  Eigen::Matrix3d correction_covariance_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d matcher_session_base_covariance_{Eigen::Matrix3d::Zero()};
  std::string precision_frame_;
  bool has_raw_stamp_{false};
  double last_raw_stamp_sec_{0.0};
};

}  // namespace pure_precision_global_localizer
