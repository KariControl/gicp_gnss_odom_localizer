# Evaluation

This directory is the public entry point for the project's evaluation. Each
result page describes its test conditions, metrics, limitations, and current
status; [the methodology](methodology.md) defines the common timestamp,
alignment, metric, runtime, and timing rules.

## Evaluation pages

| Evaluation target | Result page | Scope | Status |
|---|---|---|---|
| Velodyne 32-Line + External IMU | [LiDAR/IMU-only evaluation](lidar_imu.md) | Scan-to-scan and isolated scan-to-submap odometry | Limited to the valid recording prefix |
| Livox MID-360 + Internal IMU | [LiDAR/IMU-only evaluation](lidar_imu.md) | Tuned scan-to-scan and isolated rolling scan-to-submap odometry | Accepted for the documented recording-specific profile |
| Hesai 32-Line + IMU + RTK GNSS — Course 2 | [LiDAR/IMU/GNSS evaluation](lidar_imu_gnss.md) | Local scan-to-scan/scan-to-submap and guarded global GNSS localization | Adopted profile; 55/55 GLIM A/B hard gates passed |

The source recordings are private and are not distributed in this repository.
Dataset labels describe only the sensor configuration and course; private bag
names, measurements, run directories, logs, and sample-level outputs are not
part of the public documentation set. The repository's
[procedural LiDAR/IMU rosbag sample](../../data/README.md) shares no measurements
with these recordings and is intended for functional demonstration, not as an
accuracy dataset or ground truth.

## How to interpret the results

- GLIM is a correlated LiDAR/IMU pseudo-reference, not independent ground
  truth.
- The common alignment, interpolation, error, and timing rules are defined in
  [the evaluation methodology](methodology.md). Detailed metrics and gates
  belong to the result pages and their normalized `metrics.json` files.
- The Hesai evaluations use the packaged NMEA projection: Transverse Mercator,
  WGS84, origin `35.681236, 139.767125`, and scale `0.9996`. Publication checks
  require the runtime parameters to match `map_projector_info.yaml` and require
  an empty evaluation-origin override.
- The committed Hesai accuracy assets are pinned to the documented profile:
  `gyro_bias.initial_bg_rad_s=-0.00210` (`rad/s`), adaptive gyro bias and
  smoother enabled, ZUPT/NHC disabled, and typed fusion authority active. The
  result remains valid only for the documented recording and configuration.
- Matcher processing time and end-to-end latency are timing metrics, not CPU
  utilization. CPU utilization and RSS are not reported without a controlled
  same-host measurement.

## Evaluation profiles

Package defaults remain with each node package. Recording-specific overrides
are installed by the data-only `pure_localization_evaluation_profiles` package:

- [LiDAR/IMU odometry profiles](../../src/pure_localization_evaluation_profiles/config/odometry/lidar_imu/README.md)
- [LiDAR/IMU scan-to-submap profiles](../../src/pure_localization_evaluation_profiles/config/precision/lidar_imu/README.md)
- [Hesai LiDAR/IMU/GNSS odometry and dataset profiles](../../src/pure_localization_evaluation_profiles/config/odometry/lidar_imu_gnss/hesai_32line_rtk/README.md)
- [Hesai LiDAR/IMU/GNSS scan-to-submap profile](../../src/pure_localization_evaluation_profiles/config/precision/lidar_imu_gnss/hesai_32line_rtk/README.md)

An `accepted/` profile is accepted only for its documented recording. Sensor
calibration, IMU-gap, gyro-bias, and other recording-specific values must not be
silently reused for another sensor, recording, temperature, or startup state.

Publication assets and their verification workflow are documented in the
[asset policy](assets/README.md). Repository-level build and test
commands are documented in [validation](../validation.md).
