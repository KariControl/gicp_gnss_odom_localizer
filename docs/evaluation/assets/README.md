# Published evaluation assets

This directory contains the small, reviewable artifacts used by the evaluation
documentation. It is intentionally independent of the ignored `test_results/`,
`docker_output/`, and `rosbag/` directories.

The directories are named by the physical sensor setup and course:

- `velodyne_32line_external_imu`: Velodyne 32-Line + External IMU —
  LiDAR/IMU-Only (No GNSS);
- `livox_mid360_internal_imu`: Livox MID-360 + Internal IMU —
  LiDAR/IMU-Only (No GNSS);
- `hesai_32line_imu_rtk_gnss_course_2`: Hesai 32-Line + IMU + RTK GNSS —
  Course 2;
- `autoware_lsim_hesai_course_2`: Course 2 headless Autoware Logging
  Simulation interface/runtime evidence.

Each accuracy directory contains separate trajectory, XY-error, and yaw-error
figures plus normalized `metrics.json`. The root `manifest.json` records hashes
for both published files and their local source artifacts. It uses local source
IDs only; it does not expose host paths or publish the private source rosbags.

The Hesai GNSS assets come from full-rate native reruns using the packaged
default NMEA projection: Transverse Mercator, WGS84, origin `35.681236,
139.767125`, and scale `0.9996`. Every baseline, control, and precision run was
verified to load the runtime `param.yaml`, match `map_projector_info.yaml`, and
apply no evaluation-origin override. Their primary GLIM accuracy metrics use
exact same-run estimator header-stamp intersections and one complete first-
common-pose SE(2) per local/global group; the same transform aligns position and
yaw, and no scale is applied.

Course 2 is accepted by the canonical public evaluation. It passed 53/53
accuracy hard gates, 20/20 startup yaw-safety checks, 28/28 runtime checks,
17/17 accepted-scan non-intrusion checks, and the baseline/control/precision
provenance suites at 33/33, 34/34, and 40/40. The accepted guard changes
orientation only: unhealthy samples cannot refresh its trusted RTK-Q4/
precision-local yaw reference, and global XY retains the existing-fusion
frozen-anchor composition. The active reference variance is snapshotted for
the complete outage/release episode and is retained in the additional
published yaw covariance.

The Course 2 Autoware LSim headless result uses the packaged default projection,
applies no evaluation-origin override, and provides interface, runtime-
completion, and GNSS-recovery evidence. It does not measure absolute
localization accuracy, CPU utilization, or RSS, and no public RViz recording is
available.

Additional private recordings remain available for internal validation, but
their identities, metrics, plots, and provenance artifacts are excluded from
the public documentation set.

Verify the committed assets and manifest from a clean public checkout with:

```bash
python3 tools/evaluation/curate_publication_assets.py --check
```

Regenerate the assets from the pinned preserved local results, which are
intentionally ignored by Git, and then verify byte-for-byte stability with:

```bash
python3 tools/evaluation/curate_publication_assets.py
python3 tools/evaluation/curate_publication_assets.py --check-regeneration
```

Regeneration fails if a required GNSS accuracy, startup, accepted-scan, run
provenance, LSim validation, or LSim projection-provenance status changes, or if
any pinned source-artifact hash changes. To adopt a new evaluation run, review
it and update the source contract in the generator deliberately.

Only compact PNG and JSON files belong here. Do not add bags, MCAP files, raw
logs, profiling traces, or sample-level CSV files. GLIM is a correlated
LiDAR/IMU reference method in these evaluations, not independent ground truth.
