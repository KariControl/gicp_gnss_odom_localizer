# LiDAR/IMU-Only Evaluation

## Conclusion

- For **Velodyne 32-Line + External IMU — LiDAR/IMU-Only (No GNSS)**,
  isolated scan-to-submap matching reduced XY and yaw error on the continuous
  prefix before the first fatal tracking event. It is not selected as the
  default full-recording output because it did not recover motion across a
  generation boundary caused by missing IMU data.
- For **Livox MID-360 + Internal IMU — LiDAR/IMU-Only (No GNSS)**, the tuned
  scan-to-scan and isolated rolling scan-to-submap result is **accepted**. The
  rolling output reduced full-interval XY/yaw RMSE by 53.347%/79.274%, and all
  formal A/B hard gates passed.
- Both metric tables use [exact-initial-pose alignment](methodology.md#alignment-a-exact-initial-pose).
- CPU utilization and RSS were not measured.

## Common conditions

- Inputs are LiDAR and IMU only. GNSS, existing global fusion, and recorded
  `/tf` and `/tf_static` are not used.
- Evaluation uses the physical `header.stamp` of accepted scans. Estimator poses
  are not interpolated.
- Only the GLIM reference is interpolated to each timestamp. No scale correction
  is applied.
- GLIM uses the same LiDAR and IMU observations and is therefore correlated
  pseudo-ground truth.
- The base odometer always remains scan-to-scan. Scan-to-submap matching runs in
  an isolated external process and publishes a separate output.

## Velodyne 32-Line + External IMU — LiDAR/IMU-Only (No GNSS)

### Valid continuous prefix

The headline interval ends before the first fatal tracking event caused by an
IMU gap. It covers 45.991 seconds and 49.135 metres. The extracted prefix
contains 11,365 records and was verified against the source prefix for topic,
timestamp, order, and serialized CDR payload.

| Output | XY RMSE | XY p95 | Yaw RMSE | Yaw p95 | Endpoint XY | Path ratio |
|---|---:|---:|---:|---:|---:|---:|
| Scan-to-scan | 0.4946 m | 0.8573 m | 0.3140 deg | 0.5112 deg | 0.9455 m | 0.99902 |
| Scan-to-submap | 0.2156 m | 0.3491 m | 0.1227 deg | 0.1875 deg | 0.2382 m | 1.01411 |
| Relative improvement | 56.41% | 59.28% | 60.91% | 63.32% | 74.81% | — |

At 10 metres, translation RPE improved from 0.2831 m to 0.1861 m and yaw RPE
improved from 0.2805 deg to 0.1515 deg. The interval is too short to evaluate
50/100-metre RPE or generation recovery.

The representative trajectory below uses one shared exact-initial-pose
transform for the reference and both A/B streams.

![Velodyne 32-Line with external IMU, valid-prefix trajectory](assets/velodyne_32line_external_imu/trajectory.png)

Detailed curated evidence:

- [XY error](assets/velodyne_32line_external_imu/xy_error.png)
- [Yaw error](assets/velodyne_32line_external_imu/yaw_error.png)
- [Machine-readable metrics and validation summary](assets/velodyne_32line_external_imu/metrics.json)

### Full-recording robustness warning

The full recording moved in the same improvement direction, but both outputs
failed the absolute accuracy gate. This is reported separately from the valid
prefix and must not be combined with its headline metrics.

| Output | XY RMSE | Yaw RMSE | Endpoint XY | Decision |
|---|---:|---:|---:|---|
| Scan-to-scan | 6.1593 m | 8.2224 deg | 18.5201 m | Absolute failure |
| Scan-to-submap | 4.2936 m | 5.4745 deg | 12.6934 m | Improved, but absolute failure |

The matcher does not bridge the old submap to the first scan of a new
generation. Consequently, it cannot reconstruct motion that occurred during
the missing-data interval. Merely widening a registration gate does not address
that failure mode.

## Livox MID-360 + Internal IMU — LiDAR/IMU-Only (No GNSS)

The common A/B interval is 295.099192 seconds; each evaluated stream spans
294.099157 seconds and the GLIM reference path is 91.179 m. Inputs are
`/livox/lidar` and the MID-360's internal `/livox/imu`;
the latter is converted to SI acceleration with scale 9.80665. The configured
`base_link` to `livox_frame` transform is identity. GNSS and recorded `/tf` and
`/tf_static` are not used.

The requested scan-to-map mode is evaluated as the package's isolated online
rolling scan-to-submap matcher. It builds its submap from recent accepted scans;
it does not localize against a static prebuilt point-cloud map.

### Accepted full-recording A/B

| Output | Samples | XY RMSE | XY p95 | Yaw RMSE | Yaw p95 | Endpoint XY | Path ratio |
|---|---:|---:|---:|---:|---:|---:|---:|
| Tuned scan-to-scan control | 2,917 | 0.630345 m | 1.342970 m | 2.906399 deg | 3.738106 deg | 0.909710 m | 0.997404 |
| Tuned rolling scan-to-submap | 2,916 | 0.294074 m | 0.625669 m | 0.602392 deg | 0.908593 deg | 0.675762 m | 1.005725 |
| Relative improvement | — | 53.347% | 53.411% | 79.274% | 75.694% | 25.717% | — |

| Distance | Scan-to-scan translation/yaw RPE | Scan-to-submap translation/yaw RPE | Relative improvement |
|---|---:|---:|---:|
| 10 m | 0.249495 m / 1.129510 deg | 0.168680 m / 0.233278 deg | 32.392% / 79.347% |
| 50 m | 0.632831 m / 1.650119 deg | 0.360651 m / 0.452356 deg | 43.010% / 72.586% |

The path is shorter than 100 m, so a 100 m RPE is unavailable. Both replays
completed at 1.0x, the isolated scan-to-submap branch preserved the scan-to-scan
odometry contract, and all formal A/B hard checks passed.

### Parameter search and selected profile

The parameter search covered LiDAR filtering, scan-to-scan VGICP, scan
acceptance, gyro-bias policy, smoothing, stop detection, snapshot cadence,
submap geometry, and correction gating. Candidate selection did not modify the
source recording or reference.

| Area | Selected values |
|---|---|
| LiDAR and scan VGICP | 30 m maximum range; 0.15 m input voxel; 0.6 m correspondence distance; correspondence randomness 15; 0.5 m VGICP voxel; maximum fitness 5.0 |
| IMU and motion policy | Fixed gyro-Z bias 0.006959692 rad/s; adaptive bias, fixed-lag smoother, and ZUPT disabled; stop threshold 0.15 m/s held for 0.5 s |
| Snapshot and submap geometry | One snapshot per five accepted scans; 0.2 m map voxel; 0.4 m matcher correspondence distance and voxel; keyframes at most 0.6 s apart, or after 0.3 m / 0.06 rad |
| Correction gates | At most 0.1 m translation, 0.03 rad yaw, 0.05 rad roll/pitch, and 0.1 m Z; fitness at most 0.08 and inlier ratio at least 0.8; robust consistency requires 3 of 5 samples |

All stop-detector candidates missed the preregistered false-positive,
precision, or recall gates. The workflow therefore retained the safe existing
0.15 m/s and 0.5 s thresholds rather than treating a failed classifier as a
winner, and required ZUPT to remain off.

Training and holdout blocks were fixed before execution. Candidate selection
used only training metrics; the holdout was first opened after every stage and
the final settings were locked. No candidate was replayed and no parameter was
changed after its reveal; the locked settings were then used for the independent
final A/B.

| Holdout output | XY RMSE | Yaw RMSE | Path ratio | Low-motion excess path |
|---|---:|---:|---:|---:|
| Scan-to-scan | 0.594082 m | 3.077648 deg | 0.988341 | 0.283470 m |
| Rolling scan-to-submap | 0.306636 m | 0.673742 deg | 0.997425 | 0.326195 m |

The holdout confirms 48.385% lower XY RMSE and 78.109% lower yaw RMSE, but the
low-motion excess path increased by 15.072%. That localized regression remains
a limitation even though the full and holdout accuracy/path-ratio gates pass.

The previously published fixed-bias scan-to-scan run reported 0.779471 m XY
RMSE, 2.905670 deg yaw RMSE, and 0.183010 m endpoint XY error. The newly tuned
control improves XY RMSE by 19.132%, leaves yaw effectively unchanged (0.025%
worse), and worsens endpoint error to 0.909710 m. Those retrospective values
were not used for candidate selection and must be read alongside the accepted
A/B and holdout results.

GLIM consumes the same LiDAR and internal-IMU observations, so it is a
correlated pseudo-reference rather than independent ground truth. CPU
utilization and RSS were not measured.

Detailed curated evidence:

- [Tuned scan-to-scan and rolling scan-to-submap trajectory](assets/livox_mid360_internal_imu/trajectory.png)
- [XY error](assets/livox_mid360_internal_imu/xy_error.png)
- [Yaw error](assets/livox_mid360_internal_imu/yaw_error.png)
- [Machine-readable metrics and validation summary](assets/livox_mid360_internal_imu/metrics.json)

## Runtime and unmeasured CPU load

| Dataset and mode | 1.0x runtime | Registration evidence | CPU | RSS |
|---|---|---|---|---|
| Velodyne 32-Line + External IMU — LiDAR/IMU-Only (No GNSS), scan-to-scan | Pass | Accepted-scan density 98.87% | Not measured | Not measured |
| Velodyne 32-Line + External IMU — LiDAR/IMU-Only (No GNSS), scan-to-submap | Pass | Matcher processing p99 128.452 ms; latency p99 231.531 ms; queue drops 0 | Not measured | Not measured |
| Livox MID-360 + Internal IMU — LiDAR/IMU-Only (No GNSS), tuned scan-to-scan | Pass | Deskew 2,943/2,953; 2,928 accepted sequences from 2,943 odometer inputs; registration density 99.524% | Not measured | Not measured |
| Livox MID-360 + Internal IMU — LiDAR/IMU-Only (No GNSS), tuned rolling scan-to-submap | Pass | Deskew 2,942/2,953; 2,927 accepted sequences; matcher accepted 463/546 attempts, published 444 corrections, processing/latency p99 26.279/140.266 ms, queue drops 0 | Not measured | Not measured |

Processing p99 and latency p99 are not CPU-utilization measurements. A CPU and
memory comparison requires controlled replays with the base component container
and additional scan-to-submap processes sampled separately, as defined in the
[evaluation methodology](methodology.md#runtime-latency-cpu-and-memory).
