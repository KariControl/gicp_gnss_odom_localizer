# LiDAR/IMU/GNSS Evaluation

## Conclusion

The published evaluation of the private Hesai 32-Line, IMU, and RTK-GNSS
Course 2 recording used a 1.0x replay with the packaged default NMEA projection:
Transverse Mercator, WGS84, origin `35.681236, 139.767125`, and scale `0.9996`.
The runtime values from
`pure_nmea_gnss_conversion/param/param.yaml` were verified against
`config/map_projector_info.yaml`, and the evaluation NMEA override was empty.

Course 2 is accepted with the orientation-only GNSS-outage yaw guard. Accuracy,
startup, runtime, and accepted-scan non-intrusion checks passed. Source rosbags
remain private and are not distributed on GitHub.

Additional private recordings may be used for internal regression and release
validation. Their identities, measurements, and artifacts are intentionally
excluded from the public documentation.

## Evaluation method

The evaluation used a scan-to-scan reference run, an accepted-scan control run,
and a scan-to-submap run from the same Release build and default-projection
configuration.

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

Startup yaw-safety is a separate acceptance test. It freezes a yaw offset from
an explicit, inclusive 20-second legacy-global/GLIM calibration window, then
checks the first scan-to-submap global output and every startup output record.
GLIM provides 400 physical-header-stamp samples in the window; speed-run
legacy-global yaw is interpolated to those stamps with a 0.1 s maximum gap.
That calibration is not used by the primary exact-initial-pose accuracy
metrics, plots, or gates.

## Dataset and acceptance

| Dataset | Duration | Exact local/global samples | Accuracy result |
|---|---:|---:|---|
| Hesai 32-Line + IMU + RTK GNSS — Course 2 | 213.635 s | 8,500 / 7,772 | **Accepted** |

The fail-closed provenance validator verifies dataset and mode selection, full
1.0x playback, TF, copied configuration files and SHA-256 values, projection
metadata, the empty NMEA evaluation override, required topics, duration,
diagnostics, and causal raw-to-fused stamp coverage. Detailed gate outcomes are
retained in the published [metrics](assets/hesai_32line_imu_rtk_gnss_course_2/metrics.json).

## Local odometry A/B

| Output | XY RMSE | Yaw RMSE | XY improvement |
|---|---:|---:|---:|
| Scan-to-scan | 1.8672 m | 1.5489 deg | — |
| Scan-to-submap | 0.4785 m | 0.7390 deg | 74.38% |

Translation/yaw RPE RMSE at fixed path distances:

| Output | 10 m | 50 m | 100 m |
|---|---:|---:|---:|
| Scan-to-scan | 0.2395 m / 0.9329 deg | 0.9349 m / 1.2210 deg | 1.6961 m / 1.4210 deg |
| Scan-to-submap | 0.1535 m / 0.9002 deg | 0.5616 m / 1.0470 deg | 0.9410 m / 1.0900 deg |

## Global GNSS A/B

| Output | XY RMSE | Yaw RMSE | XY improvement |
|---|---:|---:|---:|
| Scan-to-scan | 1.7105 m | 2.4647 deg | — |
| Scan-to-submap | 0.5060 m | 1.4484 deg | 70.42% |

Global translation/yaw RPE RMSE:

| Output | 10 m | 50 m | 100 m |
|---|---:|---:|---:|
| Scan-to-scan | 0.5023 m / 2.6330 deg | 1.3672 m / 2.6420 deg | 2.0858 m / 2.6179 deg |
| Scan-to-submap | 0.2447 m / 1.8840 deg | 0.9457 m / 1.9871 deg | 1.4178 m / 1.8572 deg |

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

The accepted configuration required fix quality 4 and used a 2.0 s maximum
reference age, `0.0225 rad²` maximum reference variance, `0.35 rad` maximum
trusted delta, `0.2 rad/s` rate limit, `0.04 rad` step limit, and `0.25 s`
step-time cap. Runtime diagnostics verified the orientation-only source, XY
policy, active variance snapshot, covariance formulas, and clearing policy.

| Guard evidence | Observed value |
|---|---:|
| Diagnostic state samples | 214 |
| Accepted/rejected trusted references | 131 / 0 |
| Outage/recovery edge counters | 18 / 17 |
| Maximum observed offset rate | 0.084477 rad/s |
| Maximum active reference variance | 0.008574748 rad² |
| Maximum added yaw variance | 0.008574748 rad² |
| Invalid advances / suppressed invalid outputs | 0 / 0 |

The recording ends during a final `STABILIZING_RECOVERY` episode, which
explains the 18/17 edge counters; its anchor remains initialized and position
fused. Earlier outage/recovery cycles returned to tracking.

## Outage accuracy and recovery

Outage RMSE uses the longest post-initialization non-`TRACKING` interval from
`gnss_map_odom_fusion` in the accepted scan-to-submap run, not the raw loss of
usable GNSS positioning. The evaluated fusion-outage window was 122.500 s.

| Metric | Scan-to-scan | Scan-to-submap | Change |
|---|---:|---:|---:|
| XY RMSE | 2.1502 m | 0.6144 m | 71.42% lower |
| Yaw RMSE | 2.0770 deg | 0.7077 deg | 65.93% lower |

Anchor target and applied values remained serialization-exact and unchanged
whenever strict existing-fusion health was false. The longest greater-than-two-
second usable-GNSS gap lasted 117.252 s; finite global tracking returned 5.198 s
after usable GNSS positioning resumed.

## Startup yaw safety

The dedicated startup evaluation passed. Native output delay is
measured from the first positive raw header stamp to the first positive
scan-to-submap global header stamp; the slower 1 Hz diagnostic observation is
reported only as secondary timing.

| Native first-output delay | 1 Hz diagnostic observation | First/max GLIM yaw error | First/max legacy-global difference |
|---:|---:|---:|---:|
| 19.199991 s | 19.974988 s | 0.485945 / 6.958743 deg | 0.014447 / 7.103558 deg |

The declared calibration window was an inclusive 20.0-second physical ROS
header-stamp interval. It contained 400 GLIM samples and produced a frozen
circular yaw offset of -3.874033 deg. Absolute header timestamps are not part of
the public summary. This window-derived offset belongs only to the startup
yaw-safety calibration; the primary RMSE, RPE, and plots use their separate
exact-initial-pose alignment.

No scan-to-submap global odometry or pose was published before readiness. The
first odom and pose were an atomic pair at exactly the next unique raw stamp
after activation, under healthy existing-fusion authority and three stable
candidates.

## Runtime and non-intrusion

| Scan-to-submap validator | Matcher processing p99 | End-to-end latency p99 | Queue drops |
|---|---:|---:|---:|
| Pass at 1.0x | 54.277 ms | 128.409 ms | 0 |

The runtime validator also confirmed zero strict map-fusion odometry drops and
exact causal raw-stamp coverage of 7,892/7,892 unique stamps.

All 17 accepted-scan checks passed: all 851/851 physical scans shared
generation/sequence keys and exact stamps. The accepted-pose XY/yaw RMSE
difference was 0.000780 m / 0.000831 deg, and the accepted-increment difference
was 0.000006 m / 0.000004 deg.

Matcher processing and latency are timing metrics, not CPU utilization. CPU
utilization and RSS were not measured, so no CPU-load comparison is claimed.

## Curated plots and metrics

![Hesai Course 2 scan-to-scan and scan-to-submap global XY error during GNSS outage and recovery](assets/hesai_32line_imu_rtk_gnss_course_2/global_xy_error.png)

![Hesai Course 2 scan-to-scan and scan-to-submap global yaw error during GNSS outage and recovery](assets/hesai_32line_imu_rtk_gnss_course_2/global_yaw_error.png)

Both figures use the aligned global CSV's `time_from_common_start_sec` axis.
The hatched interval marks 117.252 s without usable GNSS positioning. The red
marker shows GNSS returning at 184.577 s, and the green marker shows global
localization resuming 5.198 s later at 189.775 s. No Hesai trajectory plot or
RViz2 recording is published as native accuracy evidence. The separate
Autoware evaluation page links a representative visualization-only replay.

Additional retained assets:

- [Local XY error](assets/hesai_32line_imu_rtk_gnss_course_2/local_xy_error.png)
- [Local yaw error](assets/hesai_32line_imu_rtk_gnss_course_2/local_yaw_error.png)
- [Global XY error](assets/hesai_32line_imu_rtk_gnss_course_2/global_xy_error.png)
- [Global yaw error](assets/hesai_32line_imu_rtk_gnss_course_2/global_yaw_error.png)
- [Machine-readable metrics and provenance](assets/hesai_32line_imu_rtk_gnss_course_2/metrics.json)

This result does not establish generalization to another LiDAR, calibration,
environment, weather condition, or speed range. The source recording and full
validation run are private and are not published on GitHub.
