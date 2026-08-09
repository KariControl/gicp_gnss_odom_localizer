// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

namespace pure_gyro_odometer
{
namespace tracking
{

enum class Mode
{
  ScanToScan,
  ScanToSubmap,
};

enum class RegistrationPath
{
  None,
  ScanToScan,
  ScanToScanWarmup,
  ScanToSubmap,
  ScanToScanInterim,
  ScanToScanFallback,
};

inline std::string normalizeModeName(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(), [](unsigned char character) {
      if (character == '-') return static_cast<char>('_');
      return static_cast<char>(std::tolower(character));
    });
  return value;
}

inline Mode parseMode(const std::string & value)
{
  const std::string normalized = normalizeModeName(value);
  if (normalized == "scan_to_scan") return Mode::ScanToScan;
  if (normalized == "scan_to_submap") return Mode::ScanToSubmap;
  throw std::invalid_argument(
          "lidar_odom.tracking_mode must be 'scan_to_scan' or 'scan_to_submap'");
}

inline const char * toString(Mode mode)
{
  return mode == Mode::ScanToSubmap ? "scan_to_submap" : "scan_to_scan";
}

inline const char * toString(RegistrationPath path)
{
  switch (path) {
    case RegistrationPath::ScanToScan:
      return "scan_to_scan";
    case RegistrationPath::ScanToScanWarmup:
      return "scan_to_scan_warmup";
    case RegistrationPath::ScanToSubmap:
      return "scan_to_submap";
    case RegistrationPath::ScanToScanInterim:
      return "scan_to_scan_interim";
    case RegistrationPath::ScanToScanFallback:
      return "scan_to_scan_fallback";
    case RegistrationPath::None:
    default:
      return "none";
  }
}

inline RegistrationPath selectRegistrationPath(
  Mode mode, bool submap_ready, bool submap_attempted, bool submap_valid,
  bool scan_to_scan_valid, bool fallback_to_scan_to_scan)
{
  if (mode == Mode::ScanToScan) {
    return scan_to_scan_valid ? RegistrationPath::ScanToScan : RegistrationPath::None;
  }

  if (!submap_ready) {
    return scan_to_scan_valid ? RegistrationPath::ScanToScanWarmup : RegistrationPath::None;
  }
  if (submap_valid) return RegistrationPath::ScanToSubmap;
  if (!submap_attempted) {
    return scan_to_scan_valid ? RegistrationPath::ScanToScanInterim : RegistrationPath::None;
  }
  if (fallback_to_scan_to_scan && scan_to_scan_valid) {
    return RegistrationPath::ScanToScanFallback;
  }
  return RegistrationPath::None;
}

inline bool usesSubmapMeasurement(RegistrationPath path)
{
  return path == RegistrationPath::ScanToSubmap;
}

inline bool usesScanToScanMeasurement(RegistrationPath path)
{
  return path == RegistrationPath::ScanToScan ||
         path == RegistrationPath::ScanToScanWarmup ||
         path == RegistrationPath::ScanToScanInterim ||
         path == RegistrationPath::ScanToScanFallback;
}

}  // namespace tracking
}  // namespace pure_gyro_odometer
