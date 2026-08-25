# Precision profile for Hesai 32-Line + IMU + RTK GNSS evaluation

The precision branch uses the same calibrated rig and default-projection
contract documented in
`pure_odometry_bringup/config/evaluation/lidar_imu_gnss/hesai_32line_rtk`.
Individual recordings intentionally do not have duplicate matcher or global
localizer parameter files. Course 2 is the public evaluation example; other
recordings remain anonymous internal validation inputs. The NMEA package owns
the runtime projection, and the evaluation applies no origin override.

The committed public accuracy artifact is the adopted 2026-08-25 Course 2 run.
Its odometer used `gyro_bias.initial_bg_rad_s=-0.00210` (`rad/s`), adaptive gyro-bias
estimation and the fixed-lag smoother enabled, and ZUPT/NHC disabled. Its global
path consumed the typed fusion-authority stream. The adopted result has 8,477
exact local and 7,805 exact global common samples and passes all 55/55 GLIM A/B
hard gates.

The effective precision configuration is assembled from unchanged package
files:

- `pure_precision_bringup/config/submap_snapshot_override.yaml`
- `pure_lidar_submap_matcher/param/param.yaml`
- `pure_precision_global_localizer/param/param.yaml`

The runner records the resolved paths and SHA-256 digest of every effective
file. Published result assets pin the hashes of the adopted run instead of
silently relabelling older run directories.
