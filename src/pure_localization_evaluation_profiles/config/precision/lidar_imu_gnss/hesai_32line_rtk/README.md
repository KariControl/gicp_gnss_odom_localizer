# Precision profile for Hesai 32-Line + IMU + RTK GNSS evaluation

The precision branch uses the same calibrated rig and default-projection
contract documented in
`pure_localization_evaluation_profiles/config/odometry/lidar_imu_gnss/hesai_32line_rtk`.
Individual recordings intentionally do not have duplicate matcher or global
localizer parameter files. The NMEA package owns the runtime projection, and
the evaluation applies no origin override.

The committed accuracy artifact used
`gyro_bias.initial_bg_rad_s=-0.00210` (`rad/s`), adaptive gyro-bias
estimation and the fixed-lag smoother enabled, and ZUPT/NHC disabled. Its global
path consumed the typed fusion-authority stream. The adopted result has 8,477
exact local and 7,805 exact global common samples and passes all 55/55 GLIM A/B
hard gates.

The effective precision configuration is assembled from unchanged package
files:

- `pure_precision_bringup/config/submap_snapshot_override.yaml`
- `pure_lidar_submap_matcher/param/param.yaml`
- `pure_precision_global_localizer/param/param.yaml`

In `profile.yaml`, the unqualified value
`config/submap_snapshot_override.yaml` is relative to the
`pure_precision_bringup` package share. It is not relative to this data-only
profile package. Revalidate all three effective files when adopting the
profile for another recording or deployment.
