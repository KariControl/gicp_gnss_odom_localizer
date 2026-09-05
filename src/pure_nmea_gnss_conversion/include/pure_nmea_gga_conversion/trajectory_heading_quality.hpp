// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <string>

namespace pure_nmea_gga_conversion
{

// Both endpoints of a trajectory-heading chord must carry a meaningful,
// bounded position uncertainty. Confidence alone is insufficient because a
// receiver mode can deliberately publish a high confidence score together
// with a very large position sigma.
inline bool trajectoryHeadingPointIsUsable(
  double confidence,
  double sigma_xy_m,
  double min_confidence,
  double max_sigma_xy_m)
{
  return std::isfinite(confidence) &&
         std::isfinite(sigma_xy_m) &&
         std::isfinite(min_confidence) &&
         std::isfinite(max_sigma_xy_m) &&
         confidence >= min_confidence &&
         sigma_xy_m > 0.0 &&
         sigma_xy_m <= max_sigma_xy_m &&
         max_sigma_xy_m > 0.0;
}

inline double trajectoryHeadingVariance(
  double reference_sigma_xy_m,
  double current_sigma_xy_m,
  double baseline_m,
  double sigma_floor_rad)
{
  if (!std::isfinite(reference_sigma_xy_m) || reference_sigma_xy_m <= 0.0 ||
    !std::isfinite(current_sigma_xy_m) || current_sigma_xy_m <= 0.0 ||
    !std::isfinite(baseline_m) || baseline_m <= 0.0 ||
    !std::isfinite(sigma_floor_rad) || sigma_floor_rad < 0.0)
  {
    return std::numeric_limits<double>::infinity();
  }

  const double propagated_sigma =
    std::hypot(reference_sigma_xy_m, current_sigma_xy_m) / baseline_m;
  const double yaw_sigma = std::max(sigma_floor_rad, propagated_sigma);
  return yaw_sigma * yaw_sigma;
}

inline bool trajectoryHeadingVarianceIsUsable(
  double yaw_variance_rad2,
  double max_yaw_variance_rad2)
{
  return std::isfinite(yaw_variance_rad2) &&
         std::isfinite(max_yaw_variance_rad2) &&
         yaw_variance_rad2 >= 0.0 &&
         max_yaw_variance_rad2 > 0.0 &&
         yaw_variance_rad2 <= max_yaw_variance_rad2;
}

inline bool trajectoryHeadingTurnActivityIsUsable(
  double absolute_yaw_activity_rad,
  double max_turn_activity_rad)
{
  return std::isfinite(absolute_yaw_activity_rad) &&
         std::isfinite(max_turn_activity_rad) &&
         absolute_yaw_activity_rad >= 0.0 &&
         max_turn_activity_rad > 0.0 &&
         absolute_yaw_activity_rad <= max_turn_activity_rad;
}

// A newly formed trajectory chord is an absolute-heading candidate, not an
// automatically continuous replacement for a corrected-IMU propagated
// heading. Compare the two on SO(2) and fail closed on invalid inputs. A large
// innovation starts a fresh chord segment while preserving the trusted seed;
// the caller can therefore keep publishing bounded IMU propagation until the
// new segment has accumulated its own baseline.
struct TrajectoryHeadingContinuityResult
{
  bool valid{false};
  bool accept_candidate{false};
  bool restart_segment{false};
  bool preserve_heading_seed{false};
  double innovation_rad{std::numeric_limits<double>::quiet_NaN()};
  std::string reason{"invalid_argument"};
};

inline TrajectoryHeadingContinuityResult evaluateTrajectoryHeadingContinuity(
  double candidate_yaw_rad,
  double propagated_seed_yaw_rad,
  double max_innovation_rad)
{
  TrajectoryHeadingContinuityResult out;
  if (!std::isfinite(candidate_yaw_rad) ||
    !std::isfinite(propagated_seed_yaw_rad) ||
    !std::isfinite(max_innovation_rad) || max_innovation_rad <= 0.0)
  {
    return out;
  }

  out.valid = true;
  out.innovation_rad = std::atan2(
    std::sin(candidate_yaw_rad - propagated_seed_yaw_rad),
    std::cos(candidate_yaw_rad - propagated_seed_yaw_rad));
  constexpr double kComparisonTolerance = 1.0e-12;
  if (std::fabs(out.innovation_rad) <= max_innovation_rad + kComparisonTolerance) {
    out.accept_candidate = true;
    out.reason = "ok";
    return out;
  }

  out.restart_segment = true;
  out.preserve_heading_seed = true;
  out.reason = "seed_innovation_exceeded";
  return out;
}

// Determines whether an existing trusted seed must be compared with a fresh
// trajectory candidate. Expiry is different from an IMU failure: once the
// configured propagation horizon has elapsed, the old seed is no longer a
// valid continuity reference and a quality-gated fresh chord may bootstrap a
// new seed. Inside the horizon, missing IMU coverage remains fail-closed.
struct TrajectoryHeadingSeedGateResult
{
  bool valid{false};
  bool compare_candidate{false};
  bool allow_fresh_bootstrap{false};
  bool reject_candidate{true};
  std::string reason{"invalid_seed_timing"};
};

inline TrajectoryHeadingSeedGateResult evaluateTrajectoryHeadingSeedGate(
  double seed_age_sec,
  double max_propagation_sec,
  bool imu_propagation_valid)
{
  TrajectoryHeadingSeedGateResult out;
  if (!std::isfinite(seed_age_sec) || seed_age_sec < 0.0 ||
    !std::isfinite(max_propagation_sec) || max_propagation_sec <= 0.0)
  {
    return out;
  }

  out.valid = true;
  if (seed_age_sec > max_propagation_sec) {
    out.allow_fresh_bootstrap = true;
    out.reject_candidate = false;
    out.reason = "trusted_seed_stale_rebootstrap";
  } else if (imu_propagation_valid) {
    out.compare_candidate = true;
    out.reject_candidate = false;
    out.reason = "compare_with_propagated_seed";
  } else {
    out.reason = "seed_propagation_unavailable";
  }
  return out;
}

// A trusted trajectory epoch is deliberately distinct from the rolling
// history retention window. Old points can remain useful for diagnostics and
// future policies without being eligible as a current-heading reference.
struct TrajectoryHeadingSample
{
  double stamp_sec{0.0};
  double x{0.0};
  double y{0.0};
  double sigma_xy_m{std::numeric_limits<double>::infinity()};
  double confidence{0.0};
};

struct TrajectoryHeadingObservationResult
{
  bool inserted{false};
  bool epoch_reset{false};
  bool preserve_heading_seed{false};
  std::string epoch_reset_reason{"none"};
  std::string reject_reason{"none"};
};

struct TrajectoryHeadingReferenceResult
{
  bool valid{false};
  TrajectoryHeadingSample reference;
  double reference_age_sec{std::numeric_limits<double>::quiet_NaN()};
  double baseline_m{0.0};
  double relative_position_sigma{std::numeric_limits<double>::infinity()};
  std::string reason{"not_evaluated"};
};

// Maintains quality-contiguous GNSS trajectory epochs. A brief unusable sample
// is tolerated, but a sustained unusable/no-fix interval, an input time gap,
// or non-increasing time starts a new epoch. Input cadence is intentionally
// configured separately from explicit unusable duration so a normal 1 Hz GGA
// stream is not mistaken for a quality outage.
class TrajectoryHeadingHistory
{
public:
  void configure(
    double buffer_max_age_sec,
    double epoch_max_unusable_duration_sec,
    double input_max_gap_sec)
  {
    buffer_max_age_sec_ = buffer_max_age_sec;
    epoch_max_unusable_duration_sec_ = epoch_max_unusable_duration_sec;
    input_max_gap_sec_ = input_max_gap_sec;
  }

  TrajectoryHeadingObservationResult observe(
    const TrajectoryHeadingSample & sample, bool usable, bool hard_unusable = false)
  {
    TrajectoryHeadingObservationResult result;
    if (!sampleIsFinite(sample)) {
      result.reject_reason = "non_finite_sample";
      return result;
    }

    auto reset_epoch = [&](const std::string & reason) {
        samples_.clear();
        has_last_usable_stamp_ = false;
        has_unusable_streak_ = false;
        ++epoch_;
        result.epoch_reset = true;
        result.epoch_reset_reason = reason;
      };

    if (has_last_input_stamp_) {
      const double input_dt = sample.stamp_sec - last_input_stamp_sec_;
      if (input_dt <= 0.0) {
        reset_epoch("time_not_increasing");
      } else if (input_dt > input_max_gap_sec_) {
        reset_epoch("input_gap");
      }
    }

    if (hard_unusable) {
      result.reject_reason = "hard_unusable";
      if (!result.epoch_reset && !hard_unusable_active_) {
        reset_epoch("hard_unusable");
      }
      hard_unusable_active_ = true;
      last_input_stamp_sec_ = sample.stamp_sec;
      has_last_input_stamp_ = true;
      prune(sample.stamp_sec);
      return result;
    }
    hard_unusable_active_ = false;

    if (usable) {
      if (!result.epoch_reset && has_unusable_streak_ &&
        sample.stamp_sec - unusable_streak_start_sec_ >
        epoch_max_unusable_duration_sec_)
      {
        reset_epoch("unusable_gap");
      }
      samples_.push_back(sample);
      has_last_usable_stamp_ = true;
      has_unusable_streak_ = false;
      result.inserted = true;
    } else {
      result.reject_reason = "point_unusable";
      if (!result.epoch_reset && has_last_usable_stamp_) {
        if (!has_unusable_streak_) {
          has_unusable_streak_ = true;
          unusable_streak_start_sec_ = sample.stamp_sec;
        } else if (
          sample.stamp_sec - unusable_streak_start_sec_ >
          epoch_max_unusable_duration_sec_)
        {
          reset_epoch("unusable_gap");
        }
      }
    }

    last_input_stamp_sec_ = sample.stamp_sec;
    has_last_input_stamp_ = true;
    prune(sample.stamp_sec);
    return result;
  }

  // Starts a geometry segment at an already trusted current fix. Unlike a
  // quality/input outage, this reset deliberately preserves the last trusted
  // heading seed so corrected-IMU propagation can bridge the time needed to
  // form a fresh baseline. The current sample is the only point carried into
  // the new segment.
  TrajectoryHeadingObservationResult restartSegmentAt(
    const TrajectoryHeadingSample & current,
    const std::string & reason)
  {
    TrajectoryHeadingObservationResult result;
    if (!sampleIsFinite(current) || reason.empty()) {
      result.reject_reason = "invalid_segment_restart";
      return result;
    }

    samples_.clear();
    has_last_usable_stamp_ = false;
    has_unusable_streak_ = false;
    hard_unusable_active_ = false;
    ++epoch_;

    samples_.push_back(current);
    last_input_stamp_sec_ = current.stamp_sec;
    has_last_input_stamp_ = true;
    has_last_usable_stamp_ = true;

    result.inserted = true;
    result.epoch_reset = true;
    result.preserve_heading_seed = true;
    result.epoch_reset_reason = reason;
    return result;
  }

  TrajectoryHeadingReferenceResult selectReference(
    const TrajectoryHeadingSample & current,
    double min_baseline_m,
    double max_reference_age_sec) const
  {
    TrajectoryHeadingReferenceResult out;
    if (!sampleIsFinite(current) || !std::isfinite(min_baseline_m) ||
      !std::isfinite(max_reference_age_sec) || min_baseline_m <= 0.0 ||
      max_reference_age_sec <= 0.0)
    {
      out.reason = "invalid_argument";
      return out;
    }
    if (samples_.empty()) {
      out.reason = "history_empty";
      return out;
    }

    bool has_past_point = false;
    bool has_recent_point = false;
    double max_recent_baseline = 0.0;
    constexpr double kComparisonTolerance = 1.0e-12;
    for (const auto & candidate : samples_) {
      const double age = current.stamp_sec - candidate.stamp_sec;
      if (age <= 0.0) {
        continue;
      }
      has_past_point = true;
      if (age > max_reference_age_sec) {
        continue;
      }
      has_recent_point = true;

      const double baseline = std::hypot(current.x - candidate.x, current.y - candidate.y);
      // Retain the best observed recent baseline in diagnostics even when no
      // candidate reaches the configured minimum.
      max_recent_baseline = std::max(max_recent_baseline, baseline);
      if (baseline < min_baseline_m) {
        continue;
      }

      const double relative_sigma =
        std::hypot(candidate.sigma_xy_m, current.sigma_xy_m) / baseline;
      const bool better_quality =
        relative_sigma + kComparisonTolerance < out.relative_position_sigma;
      const bool equal_quality =
        std::fabs(relative_sigma - out.relative_position_sigma) <= kComparisonTolerance;
      const bool better_baseline = equal_quality && baseline > out.baseline_m;
      const bool equally_good_more_recent =
        equal_quality && std::fabs(baseline - out.baseline_m) <= kComparisonTolerance &&
        (!out.valid || age < out.reference_age_sec);

      if (!out.valid || better_quality || better_baseline || equally_good_more_recent) {
        out.valid = true;
        out.reference = candidate;
        out.reference_age_sec = age;
        out.baseline_m = baseline;
        out.relative_position_sigma = relative_sigma;
      }
    }

    if (out.valid) {
      out.reason = "ok";
    } else if (!has_past_point) {
      out.reason = "no_past_reference";
    } else if (!has_recent_point) {
      out.reason = "no_reference_within_max_age";
    } else {
      out.reason = "baseline_too_short";
    }
    if (!out.valid) {
      out.baseline_m = max_recent_baseline;
    }
    return out;
  }

  std::size_t size() const {return samples_.size();}
  std::size_t epoch() const {return epoch_;}

private:
  static bool sampleIsFinite(const TrajectoryHeadingSample & sample)
  {
    return std::isfinite(sample.stamp_sec) && std::isfinite(sample.x) &&
           std::isfinite(sample.y) && std::isfinite(sample.sigma_xy_m) &&
           sample.sigma_xy_m > 0.0 && std::isfinite(sample.confidence);
  }

  void prune(double current_stamp_sec)
  {
    while (!samples_.empty() &&
      current_stamp_sec - samples_.front().stamp_sec > buffer_max_age_sec_)
    {
      samples_.pop_front();
    }
  }

  double buffer_max_age_sec_{5.0};
  double epoch_max_unusable_duration_sec_{0.5};
  double input_max_gap_sec_{2.0};
  std::deque<TrajectoryHeadingSample> samples_;
  bool has_last_input_stamp_{false};
  bool has_last_usable_stamp_{false};
  bool has_unusable_streak_{false};
  bool hard_unusable_active_{false};
  double last_input_stamp_sec_{0.0};
  double unusable_streak_start_sec_{0.0};
  std::size_t epoch_{0U};
};

}  // namespace pure_nmea_gga_conversion
