# Changelog

Notable user-visible changes are recorded here. Only tagged repository releases
and the current unreleased changes are listed; internal release-candidate
iterations, recording-specific experiment logs, and private dataset identifiers
are intentionally excluded.

## Unreleased

These changes are not backward-compatible with the historical `v1.0.0`
repository release and should use a new major repository tag when released.

### Added

- Added an isolated scan-to-submap branch in `pure_precision_bringup`,
  `pure_lidar_submap_matcher`, and `pure_precision_global_localizer`. It
  publishes separate outputs and does not feed corrections into the primary
  scan-to-scan odometer or GNSS fusion.
- Added `pure_localization_interface_adapter` and a deterministic Autoware
  localization-interface contract test.
- Added runtime TF-ownership checks for supported launch profiles.
- Added a fully procedural public LiDAR/IMU rosbag, generator, validator, and
  end-to-end smoke test.
- Added guarded single-antenna XY-only GNSS recovery, an outage-yaw guard, and
  typed fusion-authority messages with explicit session and sequence identity.
- Added trajectory-derived NMEA heading quality gates and bounded full-SE(2)
  recovery without feeding unapplied residuals back into the target.
- Added source, launch, Docker, publication-asset, and ROS 2 Jazzy CI checks.

### Changed

- Removed the retired in-odometer scan-to-submap implementation. The odometer
  now supports only `scan_to_scan`; the optional submap matcher is a separate,
  isolated process.
- Renamed the unreleased
  `pure_autoware_localization_adapter` package to the vendor-neutral
  `pure_localization_interface_adapter`. Topics and configurable interfaces
  are unchanged.
- Kept precision recording validation under `tools/evaluation/precision` so
  normal bringup packages do not install rosbag analysis dependencies.
- Made stop detection advance only on strictly increasing IMU events and
  require a fresh causal speed estimate when one is available.
- Removed maintainer-only replay/tuning scripts, private historical Autoware
  evaluation records, and documentation that depended on undistributed
  recordings or local workspaces. Retained the reviewed RViz README demo as a
  GitHub-hosted, visualization-only replay with a local poster.

### Fixed

- Made Docker replay use Compose's portable `-T` option instead of
  `--no-tty`.
- Updated GitHub artifact upload to the Node.js 24-based action release and
  made `.dockerignore` changes trigger the Autoware workflow.
- Installed the MCAP storage plugin in Jazzy CI so the committed synthetic bag
  can be checked.
- Corrected the Jazzy rosdep invocation to use `--default-yes`.
- Kept NMEA runtime projection parameters synchronized with
  `map_projector_info.yaml`.
- Made optional empty twist topics safe in ROS launch construction.

### Migration

- Replace the `v1.0.0` package, executable, and node names
  `pure_gnss_conversion` / `pure_gnss_conversion` / `gnss_conversion` with
  `pure_nmea_gnss_conversion` / `pure_nmea_gga_conversion` /
  `nmea_gga_conversion`. Update downstream package dependencies and launch
  files to the new names and review the documented NMEA input contract.
- `pure_gnss_msgs/msg/GnssFusionInput` has additional observation-semantics
  fields and is not type-compatible with the `v1.0.0` definition. Rebuild this
  repository and every downstream package from a clean workspace; do not mix
  generated message artifacts from the two versions. Custom publishers must
  populate the new quality, heading-validity, and observation-point semantics.
- Replace `pure_autoware_localization_adapter` with
  `pure_localization_interface_adapter`,
  `autoware_localization_adapter_node` with
  `localization_interface_adapter_node`, and the ROS node name
  `/autoware_localization_adapter` with
  `/localization_interface_adapter`.
- Remove downstream uses of `param_scan_to_submap.yaml`,
  `scan_to_submap_override.yaml`, `lidar_odom.scan_to_submap.*`,
  `lidar_odom.local_map.*`, and `lidar_odom.smoother.local_pose.*`.
- Keep `lidar_odom.tracking_mode=scan_to_scan`. Select the isolated precision
  branch through `pure_precision_bringup` or the Docker compatibility option
  `--tracking-mode scan_to_submap`.
- Maintainer-only Hesai and GLIM host runners were not a supported public API
  and have been removed. Use `script/play_localization_bag.sh` for generic
  replay or the documented Docker runner for Autoware integration.
- Evaluation profile paths containing capture timestamps moved to
  `mid360/internal_imu_evaluation` and
  `velodyne/external_imu_evaluation`.

## v1.0.0 repository tag - 2026-04-06

- Historical initial repository release. Of its six project-owned ROS
  packages, `pure_lidar_gyro_odometer` used package version `0.2.0`; the other
  five used `0.1.0`. Repository tags and per-package versions were not aligned.
- This snapshot predates the current package names, message definitions, and
  release metadata and is not compatible with the unreleased interfaces above.
