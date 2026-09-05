#pragma once

#include <cstdint>

namespace pure_gyro_odometer::snapshot_policy
{

// The first accepted pose in every stream generation is a mandatory bootstrap.
// Later snapshots follow the configured accepted-pose sequence interval.
inline bool due(
  bool generation_has_snapshot, std::uint64_t accepted_sequence,
  std::uint64_t interval)
{
  return !generation_has_snapshot ||
         (interval > 0 && accepted_sequence % interval == 0);
}

}  // namespace pure_gyro_odometer::snapshot_policy
