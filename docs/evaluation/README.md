# Evaluation Results

This directory is the public entry point for localization evaluation results.
Only curated, reviewable evidence is published here. Raw rosbags, complete test
runs, MCAP output, and ROS logs are intentionally excluded from GitHub.

## Result pages

- [Evaluation methodology](methodology.md) defines timestamp matching,
  trajectory alignment, metrics, runtime checks, and CPU measurement rules.
- [LiDAR/IMU-only evaluation](lidar_imu.md) covers scan-to-scan and the isolated
  scan-to-submap overlay.
- [LiDAR/IMU/GNSS evaluation](lidar_imu_gnss.md) covers local and global A/B
  results for two Hesai courses.
- [Autoware Logging Simulation](autoware_lsim.md) covers headless end-to-end
  validation and RViz evidence.
- [Published evaluation assets](assets/README.md) defines the stable plot and
  metrics layout.
- [Repository validation](../validation.md) describes the maintained source,
  build, test, and documentation checks.

## Datasets

The source recordings are private evaluation inputs and are not distributed in
this repository. Dataset labels describe the sensor configuration and course so
that the results remain understandable without the original bag directory names.

| Dataset | Published evaluation scope | Current status |
|---|---|---|
| Velodyne 32-Line + External IMU — LiDAR/IMU-Only (No GNSS) | Scan-to-scan and isolated scan-to-submap | Limited pass on the valid prefix; full-recording recovery failed |
| Livox MID-360 + Internal IMU — LiDAR/IMU-Only (No GNSS) | Scan-to-scan only | Measured with a recording-specific fixed gyro-bias diagnostic profile |
| Hesai 32-Line + IMU + RTK GNSS — Course 1 | Local scan-to-scan/scan-to-submap and global GNSS A/B | Historical accuracy result is provisional; corrected-profile rerun required |
| Hesai 32-Line + IMU + RTK GNSS — Course 2 | Local scan-to-scan/scan-to-submap and global GNSS A/B | Historical accuracy result is provisional; corrected-profile rerun required |
| Autoware Logging Simulation with Hesai + IMU + RTK GNSS | Course 1 and Course 2 interface validation | Headless and RViz checks passed; video is not yet available |

The historical native Hesai accuracy runs did not apply the intended site-local
NMEA-origin override. The reorganized evaluation configuration and runner now
apply that override, but the corrected profile has not yet been rerun. Those
accuracy values are therefore retained as provisional measurements, not final
acceptance evidence. The Autoware Logging Simulation profile did apply the
intended site-local origin.

## Evaluation configuration policy

Package-owned default YAML files remain in each node package. Evaluation-only
overrides are separated below each bringup package's `config/evaluation/` tree
and grouped by sensor configuration and dataset. Directory names describe the
runtime mode; they do not include the name of the reference-trajectory tool.

- [LiDAR/IMU odometry evaluation profiles](../../src/pure_odometry_bringup/config/evaluation/lidar_imu/README.md)
- [LiDAR/IMU precision-overlay profiles](../../src/pure_precision_bringup/config/evaluation/lidar_imu/README.md)
- [Hesai LiDAR/IMU/GNSS odometry and dataset profiles](../../src/pure_odometry_bringup/config/evaluation/lidar_imu_gnss/hesai_32line_rtk/README.md)
- [Hesai LiDAR/IMU/GNSS precision-overlay profile](../../src/pure_precision_bringup/config/evaluation/lidar_imu_gnss/hesai_32line_rtk/README.md)

An `accepted/` directory means that a profile was selected for that specific
recording. It does not make the values a universal production calibration.
Recording-specific IMU-gap or gyro-bias values must not be reused silently with
another sensor, recording, temperature, or startup condition.

## Publication state

- Published figures and small machine-readable summaries live under
  [`assets/`](assets/README.md), with stable descriptive names.
- Main pages embed only representative trajectories. Error plots are linked so
  result pages remain readable.
- Raw recordings, generated run directories, MCAP files, aligned-sample CSVs,
  and ROS logs are not part of the GitHub publication set.
- Matcher processing time and end-to-end latency are reported as timing metrics,
  not as CPU utilization.
- CPU utilization and RSS remain **not measured** until controlled 1.0x A/B
  sampling is repeated on the same idle host.
- The GNSS plots retain frozen calibration-window alignment. They must not be
  relabeled as exact-initial-pose plots.
- An RViz screenshot is published. A representative RViz video still needs to
  be recorded and released separately.
