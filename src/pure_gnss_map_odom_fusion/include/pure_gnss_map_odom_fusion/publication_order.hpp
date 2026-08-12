// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

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

// A late source-stamped odometry request is a data-ordering defect unless the
// exact physical stamp was already published by the timer. A wall timer can
// also observe an older ROS /clock value after a newer odometry callback has
// published. Both already-covered requests are coalescing, not lost data.
inline PublicationOrderDecision classifyPublicationOrder(
  std::int64_t requested_stamp_ns,
  std::int64_t last_published_stamp_ns,
  PublicationTrigger trigger,
  bool requested_stamp_already_published = false)
{
  if (last_published_stamp_ns != 0 && requested_stamp_ns < last_published_stamp_ns) {
    if (trigger == PublicationTrigger::ODOMETRY && requested_stamp_already_published) {
      return PublicationOrderDecision::COALESCE_ALREADY_PUBLISHED_ODOMETRY;
    }
    return trigger == PublicationTrigger::WALL_TIMER ?
           PublicationOrderDecision::COALESCE_STALE_WALL_TIMER :
           PublicationOrderDecision::DROP_OUT_OF_ORDER_ODOMETRY;
  }
  return PublicationOrderDecision::PUBLISH;
}

}  // namespace pure_gnss_map_odom_fusion
