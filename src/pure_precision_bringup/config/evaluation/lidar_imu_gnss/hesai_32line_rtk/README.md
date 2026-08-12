# Precision profile for Hesai 32-Line + IMU + RTK GNSS evaluation

The precision branch uses the same shared rig and site profile documented in
`pure_odometry_bringup/config/evaluation/lidar_imu_gnss/hesai_32line_rtk`.
Course 1 and course 2 intentionally do not have duplicate matcher or global
localizer parameter files.

The effective precision configuration is assembled from unchanged package
files:

- `pure_precision_bringup/config/submap_snapshot_override.yaml`
- `pure_lidar_submap_matcher/param/param.yaml`
- `pure_precision_global_localizer/param/param.yaml`

The runner records the resolved paths and SHA-256 digest of every effective
file. Historical result manifests remain valid and are not rewritten.
