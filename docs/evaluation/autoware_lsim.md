# Autoware Logging Simulation Evaluation

## Conclusion

On 2026-08-12, the private Hesai Course 2 recording was replayed end to end at
1.0x with the current packaged default NMEA projection and Autoware 1.9.0 in a
localization-interface-only configuration. The headless output-bag validation
passed 35 checks with no failures and one warning.

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

Additional private recordings may be exercised internally, but their
identities, measurements, and artifacts are not part of the public evaluation.
No public RViz screenshot or video is currently available.

## Current headless test conditions

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

## Current default-projection headless result

| Dataset | Analyzed messages | Kinematic states | Evaluated span | Effective output rate | Maximum XY step | Automated validation |
|---|---:|---:|---:|---:|---:|---|
| Hesai 32-Line + IMU + RTK GNSS — Course 2 | 238,367 | 19,554 | 197.280 s | 99.113 Hz | 0.515010 m | Pass: 35 pass, 0 fail, 1 warning |

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

A public RViz recording has not been produced. A future publication should be
generated from a fresh passing Course 2 run using the current default
projection, with projection provenance validation enabled, and a command of
this form:

```bash
ROS_DOMAIN_ID=84 ./script/run_autoware_lsim_docker.sh \
  --bag <hesai_course_2_bag> \
  --profile hesai-rosbag23 \
  --tracking-mode scan_to_scan \
  --run-name hesai_course_2_lsim_rviz_publication \
  --rviz
```

The recording should:

- capture only the RViz window, not the full desktop;
- use a representative 20-to-40-second interval after the relevant RViz
  statuses become `Ok`;
- show the deskewed cloud, moving kinematic-state trail, and `base_link` axes;
- avoid notifications, personal information, terminals containing secrets,
  and unnecessary audio;
- record duration, resolution, frame rate, codec, file size, SHA-256, repository
  commit, container image, playback rate, ROS time interval, and runner options;
- state explicitly that visualization is functional evidence, not a CPU or
  absolute-accuracy measurement;
- record a passing provenance result that confirms the runtime NMEA parameters
  match `map_projector_info.yaml` and that no evaluation-origin override was
  applied.

The video should be published as an external release asset rather than
committed to Git. Its version-controlled poster and manifest belong under
`docs/evaluation/assets/`.

## Related documentation and implementation

- [Reusable rosbag and Autoware workflow](../rosbag_and_autoware_lsim_evaluation.md)
- [Docker profile](../../docker/autoware_lsim/README.md)
- [Docker runner](../../script/run_autoware_lsim_docker.sh)
- [Autoware Logging Simulation launch](../../src/pure_odometry_bringup/launch/autoware_lsim_localization.launch.py)
- [Hesai RViz configuration](../../src/pure_odometry_bringup/config/autoware_lsim/hesai_rosbag23.rviz)
- [RViz Compose override](../../docker/autoware_lsim/compose.rviz.yaml)
- [Output-bag validator](../../tools/analyze_autoware_lsim_output.py)
