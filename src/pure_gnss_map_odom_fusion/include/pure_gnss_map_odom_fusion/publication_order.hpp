// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>

namespace pure_gnss_map_odom_fusion
{

enum class PublicationTrigger
{
  ODOMETRY,
  WALL_TIMER,
};

enum class PublicationOrderDecision
{
  PUBLISH,
  DROP_OUT_OF_ORDER_ODOMETRY,
  COALESCE_ALREADY_PUBLISHED_ODOMETRY,
  COALESCE_STALE_WALL_TIMER,
};

// A wall timer may retry the newest completely processed source sample, but it
// must never advance the output watermark to ROS "now". Source odometry can be
// computed and delivered after /clock has moved past its physical stamp; using
// now would then make that still-valid source sample look out of order.
inline std::int64_t wallTimerSourceStamp(
  std::int64_t ros_now_ns,
  std::int64_t latest_completed_odometry_stamp_ns)
{
  if (ros_now_ns <= 0 || latest_completed_odometry_stamp_ns <= 0 ||
    latest_completed_odometry_stamp_ns > ros_now_ns)
  {
    return 0;
  }
  return latest_completed_odometry_stamp_ns;
}

// A late source-stamped odometry request is a data-ordering defect unless its
// exact physical stamp was already published. Wall-timer retries at or behind
// the output watermark and already-covered odometry requests are coalescing,
// not lost data.
inline PublicationOrderDecision classifyPublicationOrder(
  std::int64_t requested_stamp_ns,
  std::int64_t last_published_stamp_ns,
  PublicationTrigger trigger,
  bool requested_stamp_already_published = false)
{
  if (last_published_stamp_ns != 0 && requested_stamp_ns <= last_published_stamp_ns) {
    if (trigger == PublicationTrigger::ODOMETRY && requested_stamp_already_published) {
      return PublicationOrderDecision::COALESCE_ALREADY_PUBLISHED_ODOMETRY;
    }
    if (trigger == PublicationTrigger::WALL_TIMER) {
      return PublicationOrderDecision::COALESCE_STALE_WALL_TIMER;
    }
    if (requested_stamp_ns < last_published_stamp_ns) {
      return PublicationOrderDecision::DROP_OUT_OF_ORDER_ODOMETRY;
    }
  }
  return PublicationOrderDecision::PUBLISH;
}

// Own the publication watermark and its bounded exact-stamp history so the
// runtime and deterministic ordering tests exercise the same state machine.
class PublicationOrderTracker
{
public:
  explicit PublicationOrderTracker(std::size_t history_limit)
  : history_limit_(history_limit == 0U ? 1U : history_limit) {}

  PublicationOrderDecision classify(
    std::int64_t requested_stamp_ns,
    PublicationTrigger trigger) const
  {
    const bool already_published = std::binary_search(
      published_stamp_history_ns_.begin(), published_stamp_history_ns_.end(),
      requested_stamp_ns);
    return classifyPublicationOrder(
      requested_stamp_ns, last_published_stamp_ns_, trigger, already_published);
  }

  void commit(std::int64_t published_stamp_ns)
  {
    last_published_stamp_ns_ = published_stamp_ns;
    if (published_stamp_history_ns_.empty() ||
      published_stamp_history_ns_.back() != published_stamp_ns)
    {
      published_stamp_history_ns_.push_back(published_stamp_ns);
      if (published_stamp_history_ns_.size() > history_limit_) {
        published_stamp_history_ns_.pop_front();
      }
    }
  }

  std::int64_t lastPublishedStamp() const {return last_published_stamp_ns_;}

  bool contains(std::int64_t stamp_ns) const
  {
    return std::binary_search(
      published_stamp_history_ns_.begin(), published_stamp_history_ns_.end(), stamp_ns);
  }

private:
  std::size_t history_limit_;
  std::int64_t last_published_stamp_ns_{0};
  std::deque<std::int64_t> published_stamp_history_ns_;
};

}  // namespace pure_gnss_map_odom_fusion
