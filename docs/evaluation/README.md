# Evaluation Results

This directory is the public entry point for localization evaluation results.
Only curated, reviewable evidence is published here. Raw rosbags, complete test
runs, MCAP output, and ROS logs are intentionally excluded from GitHub.

## Result pages

- [Evaluation methodology](methodology.md) defines timestamp matching,
  trajectory alignment, metrics, runtime checks, and CPU measurement rules.
- [LiDAR/IMU-only evaluation](lidar_imu.md) covers scan-to-scan and the isolated
  scan-to-submap overlay.
- [LiDAR/IMU/GNSS evaluation](lidar_imu_gnss.md) covers the published local and
  global A/B result for a Hesai recording.
- [Autoware Logging Simulation](autoware_lsim.md) covers current-default-
  projection headless validation of the published Hesai recording.
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
| Hesai 32-Line + IMU + RTK GNSS — Course 2 | Local scan-to-scan/scan-to-submap and guarded global GNSS A/B | **Accepted:** 53/53 accuracy hard gates and all supporting hard gates passed |
| Autoware Logging Simulation with Hesai + IMU + RTK GNSS | Current Course 2 headless interface validation | The default-projection run passed; no public RViz recording is available |

The native Hesai accuracy results use the packaged NMEA runtime parameters:
Transverse Mercator, WGS84, origin `35.681236, 139.767125`, and scale `0.9996`.
The provenance validator confirms that these values match the packaged map-
projector metadata and that the evaluation NMEA override is empty. It also
checks copied configuration files, source and install hashes, dataset mode, TF,
full 1.0x playback, diagnostics, and causal raw-to-fused stamp coverage. For
Course 2, the baseline, accepted-scan control, and precision provenance suites
passed 33/33, 34/34, and 40/40 checks. It also passed 20/20 startup yaw-safety
checks, 28/28 full-rate runtime checks, and 17/17 accepted-scan non-intrusion
checks. The precision checks include the orientation-only guard's active-
reference variance snapshot and added-yaw-covariance formulas.

Additional private recordings may be used for internal regression and release
validation. Their dataset identities, measurements, and artifacts are outside
the public documentation set.

The Autoware Logging Simulation headless evidence was regenerated with the same
packaged default projection and an empty NMEA evaluation override. The
published Course 2 run passed the interface, runtime-completion, and GNSS-
recovery checks. No public RViz recording is currently available.

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
- Published GNSS plots use the same exact-initial-pose rule as the LiDAR/IMU
  plots: one baseline-derived SE(2) shared across each A/B pair, with GLIM-only
  interpolation and no estimator interpolation or scale fit.
- The GNSS startup yaw-safety check uses an explicit, inclusive 20-second
  physical-header-stamp window with 400 GLIM samples and a frozen legacy-
  global/GLIM yaw calibration. It is supporting safety evidence and is not used
  by the primary exact-initial-pose RMSE, RPE, CSV, or plot calculations.
- GLIM is a correlated LiDAR/IMU pseudo-reference, not independent ground
  truth.
- A current-default-projection RViz run and representative video still need to
  be recorded and released separately.
