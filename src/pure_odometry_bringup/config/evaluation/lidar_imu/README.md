# LiDAR/IMU evaluation profiles

These parameter files reproduce the isolated LiDAR/IMU evaluations. They are
evaluation inputs, not production defaults. The reference trajectory provider
is part of the run manifest rather than this directory name.

Profiles are grouped by LiDAR model, input bag, and evaluation status:

- `accepted/` contains the canonical profile selected for the reported A/B.
- `experimental/` contains sensitivity or superseded profiles retained for
  reproducibility.

`accepted` describes profile selection only. It does not mean that the
corresponding localization mode passed every performance-adoption gate.

The MID360 fixed gyro bias was measured from the first stationary second of
`rosbag2_2026_04_08-22_52_42`. Do not apply that profile to another recording
without recalibrating and revalidating it. The Velodyne IMU-gap limits likewise
encode the audited behavior of `rosbag2_2025_02_20-16_06_35_vel`.

Historical result directories retain their original `run.env`, copied YAML,
and SHA-256 values. This directory layout governs new runs only.

`script/run_lidar_imu_glim_bag.sh` selects the `accepted` profile for each
canonical default bag. MID360 `--bag` overrides require an explicit
`--mid-yaw-policy`, so the recorded fixed bias cannot be applied silently to a
different session. Experimental profiles are selected only by explicit runner
options.
