#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>

namespace pure_lidar_submap_matcher
{
template<class T>
class LatestOnlyQueue
{
public:
  bool push(T value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool replaced = value_.has_value();
    value_ = std::move(value);
    condition_.notify_one();
    return replaced;
  }

  std::optional<T> waitPop()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] {return closed_ || value_.has_value();});
    if (!value_) return std::nullopt;
    auto output = std::move(value_);
    value_.reset();
    return output;
  }

  void close()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
    condition_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<T> value_;
  bool closed_{false};
};
}  // namespace pure_lidar_submap_matcher
