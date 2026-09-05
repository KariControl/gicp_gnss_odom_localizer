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
  throw std::invalid_argument(
          "lidar_odom.tracking_mode must be 'scan_to_scan'; the legacy internal "
          "scan_to_submap implementation has been removed");
}

inline const char * toString(Mode)
{
  return "scan_to_scan";
}

}  // namespace tracking
}  // namespace pure_gyro_odometer
