// SPDX-License-Identifier: Apache-2.0
#include "pure_lidar_gyro_odometer/tracking_mode.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace
{
using pure_gyro_odometer::tracking::Mode;

void require(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void testParsing()
{
  using pure_gyro_odometer::tracking::parseMode;
  require(parseMode("scan_to_scan") == Mode::ScanToScan, "scan_to_scan parse");
  require(parseMode("SCAN-TO-SCAN") == Mode::ScanToScan, "normalized scan parse");

  bool legacy_rejected = false;
  try {
    (void)parseMode("scan_to_submap");
  } catch (const std::invalid_argument &) {
    legacy_rejected = true;
  }
  require(legacy_rejected, "removed scan_to_submap mode must be rejected");

  bool unknown_rejected = false;
  try {
    (void)parseMode("submap");
  } catch (const std::invalid_argument &) {
    unknown_rejected = true;
  }
  require(unknown_rejected, "unknown tracking mode must be rejected");
}
}  // namespace

int main()
{
  testParsing();
  std::cout << "PASS test_tracking_mode\n";
  return 0;
}
