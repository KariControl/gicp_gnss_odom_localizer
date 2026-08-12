# Evaluation configuration

- `lidar_imu/` contains sensor- and recording-specific LiDAR/IMU evaluation
  overrides.
- `lidar_imu_gnss/` contains shared rig/site overrides plus private-dataset
  manifests for LiDAR/IMU/GNSS evaluation.

These files layer on top of component defaults. Each component package remains
the owner of its base `param/` or `config/` YAML; evaluation profiles do not
replace or duplicate those defaults.
