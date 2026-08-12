# Published evaluation assets

This directory contains the small, reviewable artifacts used by the evaluation
documentation. It is intentionally independent of the ignored `test_results/`,
`docker_output/`, and `rosbag/` directories.

The directories are named by the physical sensor setup and course:

- `velodyne_32line_external_imu`: Velodyne 32-Line + External IMU —
  LiDAR/IMU-Only (No GNSS);
- `livox_mid360_internal_imu`: Livox MID-360 + Internal IMU —
  LiDAR/IMU-Only (No GNSS);
- `hesai_32line_imu_rtk_gnss_course_1`: Hesai 32-Line + IMU + RTK GNSS —
  Course 1;
- `hesai_32line_imu_rtk_gnss_course_2`: Hesai 32-Line + IMU + RTK GNSS —
  Course 2;
- `autoware_lsim_hesai_course_1`: the Autoware LSim RViz evidence.

Each accuracy directory contains separate trajectory, XY-error, and yaw-error
figures plus normalized `metrics.json`. The root `manifest.json` records hashes
for both published files and their local source artifacts. It uses local source
IDs only; it does not expose host paths or publish the source ROS bags.

The Hesai GNSS results are explicitly marked historical and provisional. Their
native runs used the base Tokyo NMEA origin and the plots use a frozen
calibration-window alignment with separate position and yaw alignment. They
must not be described as exact-initial-pose results or as corrected-site-profile
results. A corrected-profile rerun should replace them before final acceptance.

Verify the committed assets and manifest from a clean public checkout with:

```bash
python3 tools/evaluation/curate_publication_assets.py --check
```

Regenerate the assets from preserved local results, which are intentionally
ignored by Git, and then verify byte-for-byte stability with:

```bash
python3 tools/evaluation/curate_publication_assets.py
python3 tools/evaluation/curate_publication_assets.py --check-regeneration
```

Only compact PNG and JSON files belong here. Do not add bags, MCAP files, raw
logs, profiling traces, or sample-level CSV files. GLIM is a correlated
LiDAR/IMU reference method in these evaluations, not independent ground truth.
