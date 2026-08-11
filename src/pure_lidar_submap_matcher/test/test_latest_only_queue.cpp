#include "pure_lidar_submap_matcher/latest_only_queue.hpp"

#include <cstdlib>
#include <iostream>
#include <thread>

int main()
{
  pure_lidar_submap_matcher::LatestOnlyQueue<int> queue;
  if (queue.push(1)) return 1;
  if (!queue.push(2)) return 2;
  const auto value = queue.waitPop();
  if (!value || *value != 2) return 3;
  std::thread closer([&queue] {queue.close();});
  const auto closed = queue.waitPop();
  closer.join();
  if (closed) return 4;
  std::cout << "latest-only queue tests passed\n";
  return 0;
}
