# LiDAR/IMU-Only Evaluation

## Conclusion

- For **Velodyne 32-Line + External IMU — LiDAR/IMU-Only (No GNSS)**,
  isolated scan-to-submap matching reduced XY and yaw error on the continuous
  prefix before the first fatal tracking event. It is not selected as the
  default full-recording output because it did not recover motion across a
  generation boundary caused by missing IMU data.
- For **Livox MID-360 + Internal IMU — LiDAR/IMU-Only (No GNSS)**, only the
  scan-to-scan result is retained. The investigated scan-to-submap result
  over-corrected translation and was rejected.
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
| Scan-to-scan control | 0.4946 m | 0.8573 m | 0.3140 deg | 0.5112 deg | 0.9455 m | 0.99902 |
| Scan-to-submap precision-local | 0.2156 m | 0.3491 m | 0.1227 deg | 0.1875 deg | 0.2382 m | 1.01411 |
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
- [Machine-readable metrics and provenance](assets/velodyne_32line_external_imu/metrics.json)

### Full-recording robustness warning

The full recording moved in the same improvement direction, but both outputs
failed the absolute accuracy gate. This is reported separately from the valid
prefix and must not be combined with its headline metrics.

| Output | XY RMSE | Yaw RMSE | Endpoint XY | Decision |
|---|---:|---:|---:|---|
| Scan-to-scan control | 6.1593 m | 8.2224 deg | 18.5201 m | Baseline; absolute failure |
| Scan-to-submap precision-local | 4.2936 m | 5.4745 deg | 12.6934 m | Improved, but absolute failure |

The matcher does not bridge the old submap to the first scan of a new
generation. Consequently, it cannot reconstruct motion that occurred during
the missing-data interval. Merely widening a registration gate does not address
that failure mode.

## Livox MID-360 + Internal IMU — LiDAR/IMU-Only (No GNSS)

The retained full-recording result is scan-to-scan only.

| Output | XY RMSE | XY p95 | Yaw RMSE | Yaw p95 | Endpoint XY | Path ratio |
|---|---:|---:|---:|---:|---:|---:|
| Scan-to-scan control | 0.7795 m | 1.2744 m | 2.9057 deg | 3.7396 deg | 0.1830 m | 1.02815 |

Translation/yaw RPE is 0.4471 m / 1.1280 deg at 10 metres and
0.8672 m / 1.6478 deg at 50 metres.

This diagnostic run fixed the gyro-Z bias to 0.006959692 rad/s from the first
stationary second and disabled adaptive bias updates and the fixed-lag smoother.
That value is specific to this recording; it is not a production calibration
that can be generalized to another recording, temperature, startup attitude, or
IMU. A permanent bias policy requires validation on additional recordings.

Detailed curated evidence:

- [Selected scan-to-scan trajectory](assets/livox_mid360_internal_imu/trajectory.png)
- [XY error](assets/livox_mid360_internal_imu/xy_error.png)
- [Yaw error](assets/livox_mid360_internal_imu/yaw_error.png)
- [Machine-readable metrics and provenance](assets/livox_mid360_internal_imu/metrics.json)

## Runtime and unmeasured CPU load

| Dataset and mode | 1.0x runtime | Registration evidence | CPU | RSS |
|---|---|---|---|---|
| Velodyne 32-Line + External IMU — LiDAR/IMU-Only (No GNSS), scan-to-scan | Pass | Accepted-scan density 98.87% | Not measured | Not measured |
| Velodyne 32-Line + External IMU — LiDAR/IMU-Only (No GNSS), scan-to-submap | Pass | Matcher processing p99 128.452 ms; latency p99 231.531 ms; queue drops 0 | Not measured | Not measured |
| Livox MID-360 + Internal IMU — LiDAR/IMU-Only (No GNSS), scan-to-scan | Pass | Accepted-scan density 96.60% | Not measured | Not measured |

Processing p99 and latency p99 are not CPU-utilization measurements. A CPU and
memory comparison requires controlled replays with the base component container
and additional precision processes sampled separately, as defined in the
[evaluation methodology](methodology.md#runtime-latency-cpu-and-memory).

