// SPDX-License-Identifier: Apache-2.0
#include "pure_lidar_gyro_odometer/tracking_mode.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace
{
using pure_gyro_odometer::tracking::Mode;
using pure_gyro_odometer::tracking::RegistrationPath;

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
  require(parseMode("SCAN-TO-SUBMAP") == Mode::ScanToSubmap, "normalized submap parse");

  bool rejected = false;
  try {
    (void)parseMode("submap");
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "unknown tracking mode must be rejected");
}

void testScanToScanMode()
{
  using pure_gyro_odometer::tracking::selectRegistrationPath;
  require(
    selectRegistrationPath(Mode::ScanToScan, true, true, true, true, true) ==
    RegistrationPath::ScanToScan,
    "scan-to-scan mode must not silently switch to submap");
  require(
    selectRegistrationPath(Mode::ScanToScan, true, true, true, false, true) ==
    RegistrationPath::None,
    "scan-to-scan mode must reject when its primary measurement is invalid");
}

void testScanToSubmapMode()
{
  using pure_gyro_odometer::tracking::selectRegistrationPath;
  require(
    selectRegistrationPath(Mode::ScanToSubmap, false, false, false, true, true) ==
    RegistrationPath::ScanToScanWarmup,
    "submap warmup must use scan-to-scan");
  require(
    selectRegistrationPath(Mode::ScanToSubmap, true, true, true, true, true) ==
    RegistrationPath::ScanToSubmap,
    "valid submap measurement must be primary");
  require(
    selectRegistrationPath(Mode::ScanToSubmap, true, false, false, true, false) ==
    RegistrationPath::ScanToScanInterim,
    "scheduled interval skips must propagate with scan-to-scan even when failure fallback is disabled");
  require(
    selectRegistrationPath(Mode::ScanToSubmap, true, true, false, true, true) ==
    RegistrationPath::ScanToScanFallback,
    "invalid submap measurement must use configured fallback");
  require(
    selectRegistrationPath(Mode::ScanToSubmap, true, true, false, true, false) ==
    RegistrationPath::None,
    "fallback disable must be respected");
  require(
    selectRegistrationPath(Mode::ScanToSubmap, true, true, false, false, true) ==
    RegistrationPath::None,
    "both invalid measurements must be rejected");
}
}  // namespace

int main()
{
  testParsing();
  testScanToScanMode();
  testScanToSubmapMode();
  std::cout << "PASS test_tracking_mode\n";
  return 0;
}
