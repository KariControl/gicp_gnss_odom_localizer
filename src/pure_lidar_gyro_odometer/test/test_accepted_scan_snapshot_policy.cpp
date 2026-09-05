#include "pure_lidar_gyro_odometer/accepted_scan_snapshot_policy.hpp"

#include <cstdlib>
#include <iostream>

namespace policy = pure_gyro_odometer::snapshot_policy;

int main()
{
  if (!policy::due(false, 1, 5)) return EXIT_FAILURE;
  if (!policy::due(false, 7, 5)) return EXIT_FAILURE;
  if (policy::due(true, 7, 5)) return EXIT_FAILURE;
  if (!policy::due(true, 10, 5)) return EXIT_FAILURE;
  if (policy::due(true, 10, 0)) return EXIT_FAILURE;
  std::cout << "accepted scan snapshot policy tests passed\n";
  return EXIT_SUCCESS;
}
