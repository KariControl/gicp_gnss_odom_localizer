# Evaluation asset maintenance

This directory contains the curated artifacts embedded by the public
evaluation pages. It is separate from the ignored `test_results/`,
`docker_output/`, and `rosbag/` work areas and is intended for maintainers, not
as another result summary.

## Layout

- `velodyne_32line_external_imu`: Velodyne 32-Line + External IMU plots and
  normalized metrics.
- `livox_mid360_internal_imu`: Livox MID-360 + Internal IMU plots and normalized
  metrics.
- `hesai_32line_imu_rtk_gnss_course_2`: Hesai Course 2 local/global XY and yaw
  error plots plus normalized metrics.
- `autoware_lsim_hesai_course_2`: Autoware localization-interface evidence and the
  representative RViz media.
- `manifest.json`: hashes and publication contracts for committed artifacts and
  their pinned local sources.

Use the public result pages for interpretation:
[LiDAR/IMU-only](../lidar_imu.md),
[LiDAR/IMU/GNSS](../lidar_imu_gnss.md), and
[Autoware localization-interface evaluation](../autoware_lsim.md).

## Publication policy

- Commit only curated PNG and JSON artifacts and the single manifest-pinned
  RViz WebM. Do not commit source rosbags, MCAP files, raw logs, generated run
  directories, profiling traces, or sample-level CSV files here.
- The manifest uses local source identifiers and hashes, never host paths or
  private bag names. Additional private recordings and their artifacts remain
  outside the public documentation set.
- The Hesai directory intentionally has no trajectory plot. Its fail-closed
  publication contract permits only the reviewed local/global XY and yaw error
  plots and normalized metrics.
- The Hesai accuracy directory is pinned to the adopted 2026-08-25 Course 2
  run. Its source profile used `gyro_bias.initial_bg_rad_s=-0.00210` (`rad/s`),
  adaptive gyro bias and the smoother enabled, ZUPT/NHC disabled, and typed
  fusion authority active. Replacing it requires another reviewed source
  contract and regenerated assets.
- Hesai assets must retain the packaged NMEA projection contract: Transverse
  Mercator, WGS84, origin `35.681236, 139.767125`, scale `0.9996`, matching
  `map_projector_info.yaml`, with no evaluation-origin override. Alignment and
  interpolation must follow [the common methodology](../methodology.md).
- User-facing labels must use **scan-to-scan** and **scan-to-submap**. The
  global Hesai plots must distinguish the GNSS-unavailable interval, GNSS
  return, and localization resumption without depending on color alone.
- `rviz_poster.png` must be metadata-free and `rviz_replay.webm` must remain
  timestamp-scrubbed. Both are visualization-only and must not be described as
  passing-run, accuracy, CPU, or RSS evidence. The WebM is a fixed, manually
  reviewed asset; its no-audio, frame-count, frame-rate, size, and hash contract
  is pinned in the publication generator.
- GLIM must be described as a correlated LiDAR/IMU pseudo-reference, not
  independent ground truth.

## Verification and regeneration

Verify a clean public checkout with:

```bash
python3 tools/evaluation/curate_publication_assets.py --check
```

To regenerate plots and normalized JSON from the pinned local results, then
verify byte-for-byte stability:

```bash
python3 tools/evaluation/curate_publication_assets.py
python3 tools/evaluation/curate_publication_assets.py --check-regeneration
```

The local source results are intentionally ignored by Git. Regeneration fails
closed when a required evaluation or provenance status changes, or when a
pinned source-artifact hash changes. It does not recreate the RViz poster or
WebM; it verifies their exact bytes and privacy/media contract.

Adopting a new run or media file therefore requires deliberate review and a
corresponding source-contract update in the publication generator. Do not
replace a pinned artifact by copying a new file into this directory alone.
