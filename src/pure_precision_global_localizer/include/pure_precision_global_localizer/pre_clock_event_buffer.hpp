// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <utility>

namespace pure_precision_global_localizer
{

enum class PreClockReceiveAction
{
  DEFER,
  PROCESS
};

// Only the initial, never-observed-a-positive-clock state may defer events.
// Once a positive ROS timestamp has been seen, a later zero/rewound clock must
// flow through the normal timing and ordering gates rather than looking like a
// second startup. Those gates reject invalid, stale, or backstepping events
// while retaining their configured future-skew tolerance.
class PreClockReceiveGate
{
public:
  [[nodiscard]] PreClockReceiveAction observe(std::int64_t receive_stamp_ns)
  {
    if (receive_stamp_ns > 0) {
      observed_positive_ = true;
    }
    return observed_positive_ ? PreClockReceiveAction::PROCESS :
           PreClockReceiveAction::DEFER;
  }

  [[nodiscard]] bool observedPositive() const {return observed_positive_;}

private:
  bool observed_positive_{false};
};

// A subscriber using ROS time can receive a transient-local sample before its
// local /clock subscription has delivered the first positive timestamp. Keep
// those samples in DDS order until a defensible receive timestamp exists.
template<typename Event>
class PreClockEventBuffer
{
public:
  explicit PreClockEventBuffer(std::size_t capacity)
  : capacity_(capacity)
  {
    if (capacity_ == 0U) {
      throw std::invalid_argument("pre-clock event buffer capacity must be positive");
    }
  }

  // Returns the oldest event if bounded storage overflows. The caller owns
  // rejection accounting and fail-closed health handling for that event.
  [[nodiscard]] std::optional<Event> defer(Event event)
  {
    std::optional<Event> evicted;
    if (events_.size() == capacity_) {
      evicted.emplace(std::move(events_.front()));
      events_.pop_front();
    }
    events_.push_back(std::move(event));
    return evicted;
  }

  template<typename Handler>
  void drain(Handler && handler)
  {
    while (!events_.empty()) {
      Event event = std::move(events_.front());
      events_.pop_front();
      handler(event);
    }
  }

  [[nodiscard]] std::size_t size() const {return events_.size();}
  [[nodiscard]] bool empty() const {return events_.empty();}

private:
  std::size_t capacity_;
  std::deque<Event> events_;
};

}  // namespace pure_precision_global_localizer
