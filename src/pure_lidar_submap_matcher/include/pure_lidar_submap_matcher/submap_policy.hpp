#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace pure_lidar_submap_matcher
{
inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

struct Se2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline Se2 compose(const Se2 & a, const Se2 & b)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y,
    normalizeAngle(a.yaw + b.yaw)};
}

inline Se2 inverse(const Se2 & value)
{
  const double c = std::cos(value.yaw);
  const double s = std::sin(value.yaw);
  return {-c * value.x - s * value.y, s * value.x - c * value.y,
    normalizeAngle(-value.yaw)};
}

struct StreamKey
{
  std::uint64_t session{0};
  std::uint64_t generation{0};
  std::uint64_t sequence{0};
  std::int64_t stamp_ns{0};
};

enum class StreamAction {accept, reset, stale};

inline StreamAction streamAction(const StreamKey & previous, const StreamKey & current)
{
  if (current.session != previous.session) {
    return StreamAction::reset;
  }
  // Generations are monotonic within one odometer process.  A late packet from
  // a retired generation must never roll the active submap back in time.
  if (current.generation < previous.generation) return StreamAction::stale;
  if (current.generation > previous.generation) return StreamAction::reset;
  if (current.sequence <= previous.sequence || current.stamp_ns <= previous.stamp_ns) {
    return StreamAction::stale;
  }
  return StreamAction::accept;
}

inline StreamAction guardedStreamAction(
  const StreamKey & previous, const StreamKey & current,
  const std::set<std::uint64_t> & retired_sessions)
{
  // Session ids are opaque rather than ordered.  Remember every process that
  // has already been left so a delayed DDS sample cannot resurrect it.
  if (retired_sessions.count(current.session) != 0U) return StreamAction::stale;
  return streamAction(previous, current);
}

struct KeyframePolicy
{
  double min_interval_sec{0.25};
  double max_interval_sec{1.0};
  double min_translation_m{0.75};
  double min_yaw_rad{0.15};
};

inline bool keyframeDue(
  const KeyframePolicy & p, double dt, double translation, double yaw)
{
  if (!std::isfinite(dt) || dt < 0.0) return false;
  return dt >= p.max_interval_sec ||
    (dt >= p.min_interval_sec &&
    (translation >= p.min_translation_m || std::fabs(normalizeAngle(yaw)) >= p.min_yaw_rad));
}

struct MatchLimits
{
  double max_fitness{1.0};
  double min_inlier_ratio{0.25};
  double max_translation_m{0.75};
  double max_yaw_rad{0.15};
  double max_z_m{0.25};
  double max_roll_pitch_rad{0.15};
};

struct MatchMetrics
{
  bool converged{false};
  bool finite{false};
  double fitness{0.0};
  double inlier_ratio{0.0};
  double translation_m{0.0};
  double yaw_rad{0.0};
  double z_m{0.0};
  double roll_rad{0.0};
  double pitch_rad{0.0};
};

inline std::string matchRejection(const MatchLimits & l, const MatchMetrics & m)
{
  if (!m.converged) return "not_converged";
  if (!m.finite || !std::isfinite(m.fitness) || !std::isfinite(m.inlier_ratio) ||
    !std::isfinite(m.translation_m) || !std::isfinite(m.yaw_rad)) return "nonfinite";
  if (m.fitness > l.max_fitness) return "fitness_gate";
  if (m.inlier_ratio < l.min_inlier_ratio) return "inlier_gate";
  if (m.translation_m > l.max_translation_m) return "translation_gate";
  if (std::fabs(m.yaw_rad) > l.max_yaw_rad) return "yaw_gate";
  if (std::fabs(m.z_m) > l.max_z_m) return "z_gate";
  if (std::fabs(m.roll_rad) > l.max_roll_pitch_rad ||
    std::fabs(m.pitch_rad) > l.max_roll_pitch_rad) return "roll_pitch_gate";
  return {};
}

struct RobustConfig
{
  std::size_t window_size{7};
  std::size_t min_consistent{3};
  double consistency_translation_m{0.30};
  double consistency_yaw_rad{0.06};
  double huber_translation_m{0.15};
  double huber_yaw_rad{0.03};
  double max_commit_pivot_step_m{0.50};
  double max_commit_yaw_step_rad{0.12};
};

struct CommitResult
{
  bool committed{false};
  Se2 transform;
  std::size_t consistent_count{0};
  double pivot_step_m{0.0};
  double yaw_step_rad{0.0};
  std::string reason{"insufficient_consistency"};
};

class RobustSe2Committer
{
public:
  explicit RobustSe2Committer(const RobustConfig & config = RobustConfig())
  : config_(config)
  {
    if (config_.window_size < config_.min_consistent || config_.min_consistent == 0 ||
      config_.consistency_translation_m <= 0.0 || config_.consistency_yaw_rad <= 0.0 ||
      config_.huber_translation_m <= 0.0 || config_.huber_yaw_rad <= 0.0)
      throw std::invalid_argument("invalid robust SE2 committer configuration");
  }

  CommitResult add(const Se2 & measured_transform, const Se2 & raw_pivot)
  {
    candidates_.push_back(measured_transform);
    while (candidates_.size() > config_.window_size) candidates_.pop_front();
    const Se2 seed_output = compose(measured_transform, raw_pivot);
    std::vector<Se2> consistent_outputs;
    consistent_outputs.reserve(candidates_.size());
    for (const auto & candidate : candidates_) {
      const Se2 output = compose(candidate, raw_pivot);
      if (std::hypot(output.x - seed_output.x, output.y - seed_output.y) <=
        config_.consistency_translation_m &&
        std::fabs(normalizeAngle(output.yaw - seed_output.yaw)) <=
        config_.consistency_yaw_rad)
      {
        consistent_outputs.push_back(output);
      }
    }
    CommitResult result;
    result.consistent_count = consistent_outputs.size();
    result.transform = committed_;
    if (consistent_outputs.size() < config_.min_consistent) return result;

    // Two robust IRLS passes around the newest accepted registration.
    Se2 mean = seed_output;
    for (int pass = 0; pass < 2; ++pass) {
      double sum_w = 0.0, sum_x = 0.0, sum_y = 0.0, sum_sin = 0.0, sum_cos = 0.0;
      for (const auto & output : consistent_outputs) {
        const double rt = std::hypot(output.x - mean.x, output.y - mean.y);
        const double ry = std::fabs(normalizeAngle(output.yaw - mean.yaw));
        const double wt = rt <= config_.huber_translation_m ? 1.0 :
          config_.huber_translation_m / std::max(rt, 1.0e-12);
        const double wy = ry <= config_.huber_yaw_rad ? 1.0 :
          config_.huber_yaw_rad / std::max(ry, 1.0e-12);
        const double w = wt * wy;
        sum_w += w;
        sum_x += w * output.x;
        sum_y += w * output.y;
        sum_sin += w * std::sin(output.yaw);
        sum_cos += w * std::cos(output.yaw);
      }
      if (sum_w <= 0.0) return result;
      mean = {sum_x / sum_w, sum_y / sum_w, std::atan2(sum_sin, sum_cos)};
    }
    const Se2 proposed = compose(mean, inverse(raw_pivot));
    if (has_committed_) {
      const Se2 old_output = compose(committed_, raw_pivot);
      result.pivot_step_m = std::hypot(mean.x - old_output.x, mean.y - old_output.y);
      result.yaw_step_rad = std::fabs(normalizeAngle(mean.yaw - old_output.yaw));
      if (result.pivot_step_m > config_.max_commit_pivot_step_m ||
        result.yaw_step_rad > config_.max_commit_yaw_step_rad)
      {
        result.reason = "commit_step_gate";
        return result;
      }
    }
    committed_ = proposed;
    has_committed_ = true;
    result.committed = true;
    result.transform = committed_;
    result.reason = "committed";
    return result;
  }

  void clearCandidates() {candidates_.clear();}
  void resetAll() {candidates_.clear(); committed_ = {}; has_committed_ = false;}
  const Se2 & committed() const {return committed_;}
  bool hasCommitted() const {return has_committed_;}
  std::size_t candidateCount() const {return candidates_.size();}

private:
  RobustConfig config_;
  std::deque<Se2> candidates_;
  Se2 committed_;
  bool has_committed_{false};
};

}  // namespace pure_lidar_submap_matcher
