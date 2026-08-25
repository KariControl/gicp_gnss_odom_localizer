# LiDAR/IMU/GNSS Evaluation

## Conclusion

The adopted 2026-08-25 evaluation of the private Hesai 32-Line, IMU, and
RTK-GNSS Course 2 recording used a 1.0x replay with the packaged default NMEA
projection: Transverse Mercator, WGS84, origin `35.681236, 139.767125`, and
scale `0.9996`.
The runtime values from
`pure_nmea_gnss_conversion/param/param.yaml` were verified against
`config/map_projector_info.yaml`, and the evaluation NMEA override was empty.

The adopted profile used `gyro_bias.initial_bg_rad_s=-0.00210` (`rad/s`), adaptive
gyro-bias estimation and the fixed-lag smoother enabled, ZUPT and NHC disabled,
and the typed fusion-authority startup contract active. The orientation-only
GNSS-outage yaw guard was enabled. All 55/55 GLIM A/B hard gates passed over
8,477 exact local and 7,805 exact global common samples. Source rosbags remain
private and are not distributed on GitHub.

Additional private recordings may be used for internal regression and release
validation. Their identities, measurements, and artifacts are intentionally
excluded from the public documentation.

## Evaluation method

The evaluation used a scan-to-scan reference run and a scan-to-submap run under
the packaged default projection. The separate scan-to-scan run is used only for
runtime and protocol reference checks and is excluded from the primary GLIM
RMSE.

Primary accuracy is measured within the scan-to-submap run:

- local A/B uses the exact integer-header-stamp intersection of scan-to-scan
  odometry and scan-to-submap output;
- global A/B uses the exact integer-header-stamp intersection of the
  GNSS-anchored scan-to-scan and scan-to-submap outputs;
- estimator streams are never interpolated;
- GLIM alone is interpolated to the exact estimator stamps;
- one complete SE(2) transform maps the scan-to-scan output's first common pose
  onto the GLIM pose and is shared with the scan-to-submap output;
- the same transform supplies position and yaw alignment, and no scale is
  estimated or applied.

The initial aligned scan-to-scan residual is numerically zero. GLIM uses the
same LiDAR and IMU observations, so it is a correlated pseudo-reference rather
than independent ground truth. See the [common methodology](methodology.md).

Initialization timing is evaluated independently of the RMSE alignment. It uses
native positive ROS header stamps from the first scan-to-scan sample to the
first scan-to-submap global output; the 1 Hz diagnostic observation is retained
only as a secondary timing value. Neither timing value changes the primary
exact-initial-pose transform.

## Dataset and acceptance

| Dataset | Exact local/global samples | GLIM A/B hard gates | Accuracy result |
|---|---:|---:|---|
| Hesai 32-Line + IMU + RTK GNSS — Course 2 (2026-08-25 profile) | 8,477 / 7,805 | 55/55 | **Adopted** |

The publication contract checks dataset and mode selection, full 1.0x playback,
TF, effective-configuration equivalence and reviewed source hashes, projection
metadata, the empty NMEA evaluation override, required topics, diagnostics, and
causal raw-to-fused stamp coverage. Detailed gate outcomes are retained in the
published [metrics](assets/hesai_32line_imu_rtk_gnss_course_2/metrics.json).

## Local odometry A/B

| Output | XY RMSE | Yaw RMSE | XY improvement |
|---|---:|---:|---:|
| Scan-to-scan | 1.457149 m | 0.917636 deg | — |
| Scan-to-submap | 0.389683 m | 0.792790 deg | 73.26% |

Translation/yaw RPE RMSE at fixed path distances:

| Output | 10 m | 50 m | 100 m |
|---|---:|---:|---:|
| Scan-to-scan | 0.240190 m / 0.974225 deg | 0.921966 m / 1.188099 deg | 1.584968 m / 1.211733 deg |
| Scan-to-submap | 0.158736 m / 0.945435 deg | 0.589835 m / 1.085159 deg | 0.970040 m / 1.088113 deg |

## Global GNSS A/B

| Output | XY RMSE | Yaw RMSE | XY improvement |
|---|---:|---:|---:|
| Scan-to-scan | 1.406461 m | 2.099877 deg | — |
| Scan-to-submap | 0.516555 m | 1.568195 deg | 63.27% |

Global translation/yaw RPE RMSE:

| Output | 10 m | 50 m | 100 m |
|---|---:|---:|---:|
| Scan-to-scan | 0.449585 m / 2.698667 deg | 1.363021 m / 2.668188 deg | 2.004111 m / 2.588170 deg |
| Scan-to-submap | 0.258439 m / 1.952173 deg | 1.020329 m / 2.019132 deg | 1.531579 m / 1.918841 deg |

## GNSS-outage yaw guard

The scan-to-submap global XY path retains the standard GNSS fusion's
frozen-anchor composition. The guard changes orientation only. While GNSS
fusion is strictly healthy, it estimates a robust SE(2) yaw reference from
usable high-quality GNSS positions and scan-to-submap positions. A candidate
must pass its displacement, inlier, residual, uncertainty, and stability gates.
Unhealthy samples cannot refresh the trusted reference.

When fusion becomes unhealthy, the trusted reference is propagated with
scan-to-submap yaw and applied with bounded yaw-only steps. Recovery releases
the offset through the same bounded policy. At outage entry, the guard
snapshots the trusted reference variance and retains it through the complete
outage and release, including while healthy observations refresh the next
trusted reference. During outage the added yaw variance is
`active_reference_variance + wrap(target - applied)^2`; during release it is
`active_reference_variance + applied^2`. `READY` and `DISARMED` clear the
active snapshot and add zero guard variance. This prevents the applied yaw
correction from understating published orientation covariance.

The adopted configuration requires fix quality 4 and uses a 2.0 s maximum
reference age, `0.0225 rad²` maximum reference variance,
`0.35 rad` maximum trusted delta, `0.2 rad/s` rate limit, `0.04 rad` step limit,
and `0.25 s` step-time cap. Runtime diagnostics verified the orientation-only
source, XY policy, active variance snapshot, covariance formulas, and clearing
policy.

| Guard evidence | Observed value |
|---|---:|
| Diagnostic state samples | 228 |
| Accepted/rejected trusted references | 158 / 3 |
| Outage/recovery edge counters | 24 / 23 |
| Maximum observed offset rate | 0.065109218 rad/s |
| Maximum active reference variance | 0.009053136 rad² |
| Maximum added yaw variance | 0.012102297 rad² |
| Invalid advances / suppressed invalid outputs | 0 / 0 |

The recording ends during a final `STABILIZING_RECOVERY` episode, which
explains the 24/23 edge counters; its anchor remains initialized and position
fused. Earlier outage/recovery cycles returned to tracking.

## Outage accuracy and recovery

Outage RMSE uses the longest post-initialization non-`TRACKING` interval from
`gnss_map_odom_fusion` in the accepted scan-to-submap run, not the raw loss of
usable GNSS positioning. The evaluated fusion-outage metric window was
121.500028 s. This is intentionally distinct from the RTK-Q4 gap below.

| Metric | Scan-to-scan | Scan-to-submap | Change |
|---|---:|---:|---:|
| XY RMSE | 1.760346 m | 0.579715 m | 67.07% lower |
| Yaw RMSE | 1.229303 deg | 0.861855 deg | 29.89% lower |

Anchor target and applied values remained serialization-exact and unchanged
whenever strict existing-fusion health was false. The actual RTK-Q4 gap lasted
117.252446 s; finite global localization returned 4.147737 s after Q4 resumed.

## Initialization and authority

The native output delay is measured from the first positive scan-to-scan header
stamp to the first positive scan-to-submap global header stamp. The slower 1 Hz
diagnostic observation is secondary evidence.

| Native first-output delay | 1 Hz diagnostic observation | Global authority source | GNSS-position fallback |
|---:|---:|---|---|
| 17.824388 s | 17.949373 s | Typed existing-fusion authority | Disabled |

The first global output passed the 20 s initialization gate. Runtime diagnostics
confirmed that existing fusion remained the sole global authority and that the
typed authority contract was active; the final anchor remained initialized and
position fused.

## Runtime and protocol

| Runtime/protocol evidence | Scan-to-scan run | Scan-to-submap run |
|---|---:|---:|
| Strict map-fusion timestamp drops | 0 | 0 |
| Exact causal raw-stamp coverage | 7,892 / 7,892 | 7,869 / 7,869 |
| Covered-odometry coalesces | 0 | 2,619 |
| Wall-timer coalesces | 0 | 10,486 |

The exact-key protocol contained 851 physical scans and 602 valid corrections,
with no duplicate or unknown keys. After the 3.199960 s warmup, the correction
ratio was 0.718377. Covered-odometry and wall-timer coalesces are accounted
suppressed requests, not strict timestamp drops.

The scan-to-submap runtime validator measured **55.117 ms matcher-processing
p99** and **127.668 ms end-to-end matcher p99** at 1.0x replay. These are timing
observations from this run, not CPU-load measurements or a controlled A/B
timing comparison. CPU utilization and RSS were not measured.

## Curated plots and metrics

![Hesai Course 2 scan-to-scan and scan-to-submap global XY error during GNSS outage and recovery](assets/hesai_32line_imu_rtk_gnss_course_2/global_xy_error.png)

![Hesai Course 2 scan-to-scan and scan-to-submap global yaw error during GNSS outage and recovery](assets/hesai_32line_imu_rtk_gnss_course_2/global_yaw_error.png)

Both figures use the aligned global evaluation time axis. The hatched interval
marks the 117.252446 s RTK-Q4 gap. The red marker shows Q4 returning, and the
green marker shows finite global localization resuming 4.147737 s later. The
121.500028 s fusion-outage RMSE window follows fusion state and therefore has
different boundaries. No Hesai trajectory plot or RViz2 recording is published
as native accuracy evidence. The separate Autoware evaluation page links a
representative visualization-only replay.

Additional retained assets:

- [Local XY error](assets/hesai_32line_imu_rtk_gnss_course_2/local_xy_error.png)
- [Local yaw error](assets/hesai_32line_imu_rtk_gnss_course_2/local_yaw_error.png)
- [Global XY error](assets/hesai_32line_imu_rtk_gnss_course_2/global_xy_error.png)
- [Global yaw error](assets/hesai_32line_imu_rtk_gnss_course_2/global_yaw_error.png)
- [Machine-readable metrics and provenance](assets/hesai_32line_imu_rtk_gnss_course_2/metrics.json)

This result does not establish generalization to another LiDAR, calibration,
environment, weather condition, or speed range. The source recording and full
validation run are private and are not published on GitHub.
