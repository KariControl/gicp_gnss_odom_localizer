# Autoware Localization-Interface Evaluation

> **Historical pre-Stage-A result:** this page records the 2026-08-12 output
> schema. It predates the separate twist-with-covariance stream, the two real
> Autoware monitor diagnostics, and the adapter rename introduced by Stage A.
> It remains evidence for the stated simulation-time, full-rate replay, and
> GNSS-recovery checks only. Re-run the private source bag with the current
> runner before claiming the current Stage A schema and this GNSS scenario in
> one execution.

## Conclusion

On 2026-08-12, the private Hesai Course 2 recording was replayed end to end at
1.0x with the packaged default NMEA projection and Autoware 1.9.0 in a
localization-interface-only configuration. The headless output-bag validation
passed with no failures and one bounded-step warning.

This evidence verifies compatibility with Autoware localization topics, TF,
simulation time, full-rate replay, and GNSS-outage recovery. It does not launch
a PCD/Lanelet2 map, perception, planning, or control. It therefore does not
establish absolute localization accuracy, CPU utilization, or closed-loop
driving readiness.

The run loaded the runtime `param.yaml`, matched its Transverse Mercator/WGS84
origin `35.681236, 139.767125` and scale `0.9996` to
`map_projector_info.yaml`, and applied no evaluation-origin override. Native
accuracy results are documented separately in the
[LiDAR/IMU/GNSS evaluation](lidar_imu_gnss.md).

The coordinates above are the configured projection origin needed to reproduce
the parameter/metadata equivalence check; they are not asserted to be the
recording location.

Additional private recordings may be exercised internally, but their
identities, measurements, and artifacts are not part of the public evaluation.
A representative visualization-only RViz poster and replay are published. They
are not additional accuracy, CPU, or passing-run evidence.

## 2026-08-12 pre-Stage-A headless test conditions

| Property | Value |
|---|---|
| Date | 2026-08-12 |
| Autoware | 1.9.0 with `autoware_launch` 0.52.0 |
| ROS | Jazzy |
| Execution | CPU-only Docker, localization-interface-only |
| Dataset profile | `hesai-rosbag23` |
| Input | Private Hesai 32-Line + IMU + RTK GNSS Course 2 recording; not distributed on GitHub |
| Localization | LiDAR + IMU + GNSS, `scan_to_scan` |
| Playback | 1.0x with a 100 Hz simulation clock |
| TF | Recorded TF was isolated; the profile published three calibrated static transforms |
| Projection | Packaged Transverse Mercator/WGS84 origin `35.681236, 139.767125`, scale `0.9996`; runtime parameters match map-projector metadata; empty evaluation override |

`scan_to_scan` is the primary LiDAR-odometer registration source in this run.
This is an integration test, not a scan-to-scan versus isolated scan-to-submap
A/B test.

## 2026-08-12 pre-Stage-A default-projection headless result

| Dataset | Analyzed messages | Kinematic states | Evaluated span | Effective output rate | Maximum XY step | Automated validation |
|---|---:|---:|---:|---:|---:|---|
| Hesai 32-Line + IMU + RTK GNSS — Course 2 | 238,367 | 19,554 | 197.280 s | 99.113 Hz | 0.515010 m | Pass with one warning |

The run passed finite-pose, covariance, `map -> base_link` TF, monotonic
timestamp, final-time coverage, and adapter-consistency checks.

| Check | Result |
|---|---:|
| Successful deskew | 4,271 / 4,273 (99.9532%) |
| Accepted LiDAR registration | 4,250 / 4,270 (99.5316%) |
| Tracking resets | 0 |
| Adapter rejects | 0 |
| Final GNSS fusion | `tracking / full_se2` |

The run passed two outage/recovery cycles and ended in `TRACKING`. Its only
warning was the 0.515010-m maximum XY step, which remained below the 0.55-m
practical limit. No tracking reset or adapter rejection was recorded.

The effective output rate demonstrates full 1.0x completion for this run. It
depends on output-timer scheduling and is not a CPU-utilization measurement;
CPU utilization and RSS were not recorded.

## RViz and video publication status

A [representative RViz poster](assets/autoware_lsim_hesai_course_2/rviz_poster.png)
is published as visualization only. It is not an additional accuracy, CPU, or
passing-run measurement, and its sample Lexus mesh does not represent the
recorded vehicle, its geometry, or its sensor calibration.

The corresponding [RViz replay](assets/autoware_lsim_hesai_course_2/rviz_replay.webm)
is a 75.250-second, 898x539, 2,138-frame VP8/WebM visualization with no audio.
The stream does not declare a fixed frame rate. Its capture timestamp was
removed from the public container metadata. It is committed for demonstration,
not treated as evidence from the separately validated headless run.

Any replacement must be privacy-reviewed and update the poster, WebM metadata,
hashes, and publication manifest together. The maintainer procedure is in the
[published-asset policy](assets/README.md).

## Related documentation and implementation

- [Reusable rosbag and Autoware workflow](../rosbag_and_autoware_lsim_evaluation.md)
- [Docker profile](../../docker/autoware_lsim/README.md)
- [Docker runner](../../script/run_autoware_lsim_docker.sh)
- [Autoware localization-interface launch](../../src/pure_odometry_bringup/launch/autoware_lsim_localization.launch.py)
- [Hesai RViz configuration](../../src/pure_odometry_bringup/config/autoware_lsim/hesai_rosbag23.rviz)
- [GUI Compose overlay](../../docker/autoware_lsim/compose.rviz.yaml)
- [Output-bag validator](../../tools/analyze_autoware_lsim_output.py)
