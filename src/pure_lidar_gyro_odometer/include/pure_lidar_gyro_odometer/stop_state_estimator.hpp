// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>

namespace pure_gyro_odometer::stop
{

struct ImuSample
{
  std::int64_t stamp_ns{0};
  std::array<double, 3> acceleration{{0.0, 0.0, 0.0}};
  double gyro_z_rad_s{0.0};
};

struct TimedSpeed
{
  std::int64_t stamp_ns{0};
  double speed_mps{0.0};
  double max_age_sec{0.0};
};

struct Config
{
  bool enabled{true};
  double imu_window_sec{0.3};
  double max_imu_gap_sec{0.1};
  std::size_t min_imu_samples{10};
  double speed_threshold_mps{0.15};
  double gyro_mean_threshold_rad_s{0.05};
  double acceleration_variance_threshold{0.0225};
  double hold_sec{0.5};
  double decision_history_sec{5.0};
  std::size_t max_imu_history_size{4096};
  std::size_t max_decision_history_size{4096};
};

struct Decision
{
  std::int64_t stamp_ns{0};
  bool stopped{false};
};

struct UpdateResult
{
  bool accepted{false};
  bool stopped{false};
  bool imu_quiet{false};
  bool speed_available{false};
  bool speed_low{false};
  std::size_t imu_sample_count{0};
};

inline bool isCausalFresh(
  std::int64_t query_stamp_ns, std::int64_t sample_stamp_ns, double max_age_sec)
{
  if (query_stamp_ns <= 0 || sample_stamp_ns <= 0 ||
    !std::isfinite(max_age_sec) || max_age_sec < 0.0 ||
    sample_stamp_ns > query_stamp_ns)
  {
    return false;
  }
  const double age_sec =
    static_cast<double>(query_stamp_ns - sample_stamp_ns) * 1.0e-9;
  return std::isfinite(age_sec) && age_sec <= max_age_sec;
}

// Event-time stop detector. Only strictly increasing IMU samples advance the
// FSM. Historical consumers query the immutable, timestamped decision history
// instead of rewinding the live state.
class StopStateEstimator
{
public:
  explicit StopStateEstimator(const Config & config = Config{})
  : config_(config)
  {
    validateConfig(config_);
  }

  void configure(const Config & config)
  {
    validateConfig(config);
    config_ = config;
    reset();
  }

  void reset()
  {
    imu_history_.clear();
    decision_history_.clear();
    has_last_update_stamp_ = false;
    last_update_stamp_ns_ = 0;
    has_candidate_since_ = false;
    candidate_since_ns_ = 0;
    stopped_ = false;
  }

  UpdateResult update(
    const ImuSample & sample,
    const std::optional<TimedSpeed> & speed = std::nullopt)
  {
    UpdateResult result;
    result.stopped = stopped_;

    if (!validSample(sample) ||
      (has_last_update_stamp_ && sample.stamp_ns <= last_update_stamp_ns_))
    {
      return result;
    }

    const bool imu_gap = has_last_update_stamp_ &&
      static_cast<double>(sample.stamp_ns - last_update_stamp_ns_) * 1.0e-9 >
      config_.max_imu_gap_sec;
    has_last_update_stamp_ = true;
    last_update_stamp_ns_ = sample.stamp_ns;
    result.accepted = true;

    if (imu_gap) {
      // Do not carry a stop latch or an in-progress hold interval across an
      // interval that the IMU integrator itself considers discontinuous.
      imu_history_.clear();
      has_candidate_since_ = false;
      stopped_ = false;
    }

    if (!config_.enabled) {
      imu_history_.clear();
      has_candidate_since_ = false;
      stopped_ = false;
      appendDecision(sample.stamp_ns, false);
      result.stopped = false;
      return result;
    }

    imu_history_.push_back(sample);
    pruneImuHistory(sample.stamp_ns);

    std::array<double, 3> mean{{0.0, 0.0, 0.0}};
    double gyro_mean = 0.0;
    std::size_t count = 0;
    for (const auto & item : imu_history_) {
      // Explicitly exclude future data even if a caller corrupts/reorders the
      // backing history in a later refactor.
      if (item.stamp_ns > sample.stamp_ns) {
        continue;
      }
      const double age_sec =
        static_cast<double>(sample.stamp_ns - item.stamp_ns) * 1.0e-9;
      if (age_sec > config_.imu_window_sec) {
        continue;
      }
      for (std::size_t axis = 0; axis < mean.size(); ++axis) {
        mean[axis] += item.acceleration[axis];
      }
      gyro_mean += item.gyro_z_rad_s;
      ++count;
    }
    result.imu_sample_count = count;

    bool imu_quiet = false;
    if (count >= config_.min_imu_samples && count >= 2U) {
      const double denominator = static_cast<double>(count);
      for (double & value : mean) {
        value /= denominator;
      }
      gyro_mean /= denominator;

      double acceleration_variance = 0.0;
      for (const auto & item : imu_history_) {
        if (item.stamp_ns > sample.stamp_ns) {
          continue;
        }
        const double age_sec =
          static_cast<double>(sample.stamp_ns - item.stamp_ns) * 1.0e-9;
        if (age_sec > config_.imu_window_sec) {
          continue;
        }
        for (std::size_t axis = 0; axis < mean.size(); ++axis) {
          const double residual = item.acceleration[axis] - mean[axis];
          acceleration_variance += residual * residual;
        }
      }
      acceleration_variance /= static_cast<double>(count - 1U);
      imu_quiet =
        std::isfinite(acceleration_variance) && std::isfinite(gyro_mean) &&
        acceleration_variance < config_.acceleration_variance_threshold &&
        std::fabs(gyro_mean) < config_.gyro_mean_threshold_rad_s;
    }
    result.imu_quiet = imu_quiet;

    bool speed_low = false;
    if (speed.has_value() && std::isfinite(speed->speed_mps) &&
      isCausalFresh(sample.stamp_ns, speed->stamp_ns, speed->max_age_sec))
    {
      result.speed_available = true;
      speed_low = std::fabs(speed->speed_mps) < config_.speed_threshold_mps;
    }
    result.speed_low = speed_low;

    // Required policy: when a causal vehicle-speed estimate exists, both the
    // IMU and speed gates must pass. Without speed, fall back to IMU only.
    const bool stop_candidate =
      imu_quiet && (!result.speed_available || speed_low);
    if (stop_candidate) {
      if (!has_candidate_since_) {
        candidate_since_ns_ = sample.stamp_ns;
        has_candidate_since_ = true;
      }
      const double held_sec =
        static_cast<double>(sample.stamp_ns - candidate_since_ns_) * 1.0e-9;
      stopped_ = std::isfinite(held_sec) && held_sec >= config_.hold_sec;
    } else {
      has_candidate_since_ = false;
      stopped_ = false;
    }

    appendDecision(sample.stamp_ns, stopped_);
    result.stopped = stopped_;
    return result;
  }

  std::optional<Decision> decisionAt(
    std::int64_t query_stamp_ns, double max_age_sec) const
  {
    if (query_stamp_ns <= 0 || !std::isfinite(max_age_sec) || max_age_sec < 0.0) {
      return std::nullopt;
    }
    for (auto it = decision_history_.rbegin(); it != decision_history_.rend(); ++it) {
      if (it->stamp_ns <= query_stamp_ns) {
        if (!isCausalFresh(query_stamp_ns, it->stamp_ns, max_age_sec)) {
          return std::nullopt;
        }
        return *it;
      }
    }
    return std::nullopt;
  }

  std::size_t imuHistorySize() const {return imu_history_.size();}
  std::size_t decisionHistorySize() const {return decision_history_.size();}

private:
  static void validateConfig(const Config & config)
  {
    const bool valid =
      std::isfinite(config.imu_window_sec) && config.imu_window_sec > 0.0 &&
      std::isfinite(config.max_imu_gap_sec) && config.max_imu_gap_sec > 0.0 &&
      config.min_imu_samples >= 2U &&
      std::isfinite(config.speed_threshold_mps) && config.speed_threshold_mps >= 0.0 &&
      std::isfinite(config.gyro_mean_threshold_rad_s) &&
      config.gyro_mean_threshold_rad_s >= 0.0 &&
      std::isfinite(config.acceleration_variance_threshold) &&
      config.acceleration_variance_threshold >= 0.0 &&
      std::isfinite(config.hold_sec) && config.hold_sec >= 0.0 &&
      std::isfinite(config.decision_history_sec) && config.decision_history_sec > 0.0 &&
      config.max_imu_history_size > 0U && config.max_decision_history_size > 0U;
    if (!valid) {
      throw std::invalid_argument("invalid stop-state estimator configuration");
    }
  }

  static bool validSample(const ImuSample & sample)
  {
    return sample.stamp_ns > 0 &&
      std::isfinite(sample.acceleration[0]) &&
      std::isfinite(sample.acceleration[1]) &&
      std::isfinite(sample.acceleration[2]) &&
      std::isfinite(sample.gyro_z_rad_s);
  }

  void pruneImuHistory(std::int64_t newest_stamp_ns)
  {
    while (!imu_history_.empty()) {
      const double age_sec =
        static_cast<double>(newest_stamp_ns - imu_history_.front().stamp_ns) * 1.0e-9;
      if (age_sec <= config_.imu_window_sec &&
        imu_history_.size() <= config_.max_imu_history_size)
      {
        break;
      }
      imu_history_.pop_front();
    }
  }

  void appendDecision(std::int64_t stamp_ns, bool stopped)
  {
    decision_history_.push_back({stamp_ns, stopped});
    while (!decision_history_.empty()) {
      const double age_sec =
        static_cast<double>(stamp_ns - decision_history_.front().stamp_ns) * 1.0e-9;
      if (age_sec <= config_.decision_history_sec &&
        decision_history_.size() <= config_.max_decision_history_size)
      {
        break;
      }
      decision_history_.pop_front();
    }
  }

  Config config_;
  std::deque<ImuSample> imu_history_;
  std::deque<Decision> decision_history_;
  bool has_last_update_stamp_{false};
  std::int64_t last_update_stamp_ns_{0};
  bool has_candidate_since_{false};
  std::int64_t candidate_since_ns_{0};
  bool stopped_{false};
};

}  // namespace pure_gyro_odometer::stop
