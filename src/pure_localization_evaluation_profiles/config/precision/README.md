# Precision evaluation configuration

- `lidar_imu/` contains additive precision overrides for LiDAR/IMU evaluation.
- `lidar_imu_gnss/` records how shared GNSS evaluation profiles resolve the
  package-owned snapshot, matcher, and global-localizer defaults.

Component base YAML remains in the owning packages. This directory contains
only evaluation-specific overrides and manifests.
