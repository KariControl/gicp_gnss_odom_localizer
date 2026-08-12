# Changelog

## Unreleased

### Added

- Added a canonical evaluation index with separate LiDAR/IMU-only,
  LiDAR/IMU/GNSS, and Autoware LSim result pages, including explicit alignment,
  artifact, CPU-measurement, and RViz-video publication requirements.
- Added sensor- and recording-scoped LiDAR/IMU evaluation profiles, separated
  into canonical `accepted` inputs and retained `experimental` inputs.
- Added shared Hesai 32-Line + IMU + RTK GNSS evaluation overrides and
  descriptive private Course 1/2 manifests under
  `config/evaluation/lidar_imu_gnss`; component parameter defaults remain in
  their owning packages.
- Added the `hesai-rosbag23` Autoware Logging Simulation profile for the private
  Hesai 32-Line + IMU + RTK GNSS Course 1/2 recordings, including calibrated
  static transforms, XT parameters, a shared site-local GNSS origin, a 100 Hz
  simulation clock, input-topic allowlisting, and English operating guidance.
- Added an Autoware localization-interface output bag analyzer and a native
  `--lsim-interface-test` path for environments where the official Autoware
  Docker image cannot be started.
- Added an opt-in guarded XY-only GNSS recovery mode for a stopped
  single-antenna vehicle that regains RTK position while yaw remains
  unobservable.
- Added explicit `RECOVERING_XY_ONLY` and `TRACKING_XY_ONLY` diagnostics,
  stationary-motion and covariance gates, bounded translation-only correction,
  soft quality-gap tolerance, and automatic promotion to full SE(2) recovery.
- Added fixed-yaw translation, outlier, bounded-correction, and covariance
  propagation regression tests.

### Changed

- The LiDAR/IMU evaluation runner now selects the canonical interval-2 external
  snapshot profile by default in precision mode. For its canonical MID360 bag
  it selects the fixed-bias direct diagnostic profile; an overridden MID360 bag
  requires an explicit yaw policy because that bias is bag-specific.
- Refactored `pure_nmea_gnss_conversion` by removing write-only state,
  uncalled private APIs, unused output parameters, an identity projection
  multiply, an unreachable IMU-integration branch, and duplicate buffer
  maintenance without changing its topics, parameters, or numerical output.
- Removed the legacy in-odometer scan-to-submap/local-map registration path,
  its periodic local-pose factor, parameters, profile, override, diagnostics,
  and tests. `pure_lidar_gyro_odometer` now accepts only `scan_to_scan` while
  retaining its fixed-lag smoother and the output-only accepted-scan snapshot
  bridge required by the isolated precision packages.
- Renamed the host Hesai bag runner from
  `run_vehicle_localizer_hesai_nmea_gnss_no_snow_lp.sh` to
  `run_hesai_localization_bag.sh`. Its primary selector is now
  `--localization-mode baseline|precision`; the former `--tracking-mode`
  values remain warning-emitting argument aliases only.
- Autoware lSIM now waits for recorder discovery and the first valid
  kinematic-state message, records GNSS/deskew/fusion diagnostics, preserves
  the selected sensor profile when switching tracking mode, and restores
  output ownership to the host user.
- RViz lSIM uses a localization-specific display profile for the deskewed
  point cloud, Autoware state trail, TF tree, and vehicle axes; an enabled
  RViz node must remain alive through replay completion.
- The Autoware adapter treats equal simulation timestamps as expected
  duplicate timer outputs while continuing to reject true timestamp reversal.
- Trajectory-derived NMEA headings now exclude high-position-uncertainty fixes
  and cannot replace a valid heading seed when their yaw variance exceeds the
  configured limit.
- The supplied single-antenna NMEA profile uses a 1.5 m trajectory-heading
  baseline. The generated heading still has to pass the position-sigma and
  yaw-variance gates before full SE(2) recovery can use it.
- Full SE(2) bounded recovery now keeps target covariance separate from the
  unapplied correction residual, preventing residual variance from feeding
  back into the target on every refresh.
- XY-only stationary gating now requires fresh odometry immediately before the
  GNSS timestamp, even when newer odometry is already buffered.
- Fused pose, odometry, and TF publication is serialized and suppresses a
  stale request that would make output timestamps move backwards.
- The ROS-bag single-antenna runner explicitly enables XY-only recovery while
  the generic fusion and launch defaults remain disabled.
- LiDAR/IMU evaluation configuration now uses `config/evaluation/lidar_imu`
  rather than encoding the reference provider in the directory name.

### Migration

- Update external automation to invoke `script/run_hesai_localization_bag.sh`;
  the old script path has been removed.
- Replace runner arguments `--tracking-mode scan_to_scan` and
  `--tracking-mode scan_to_submap` with `--localization-mode baseline` and
  `--localization-mode precision`, respectively.
- Remove downstream uses of `param_scan_to_submap.yaml`,
  `scan_to_submap_override.yaml`, `lidar_odom.scan_to_submap.*`,
  `lidar_odom.local_map.*`, and `lidar_odom.smoother.local_pose.*`.
- Keep the odometer parameter `lidar_odom.tracking_mode` set to
  `scan_to_scan`; isolated precision mode is selected by its bringup overlay,
  not by changing the odometer registration mode.

### Fixed

- Fixed the native Hesai evaluation runner to pass its validated NMEA site
  origin override to the launch. Historical results produced before this fix
  remain provisional until they are rerun; this change does not claim a rerun.
- Fixed the Autoware overlay image to use colcon's global `--log-base` syntax
  and a merged install tree, and made ROS setup sourcing safe with strict shell
  mode enabled.
- Empty optional twist topics are no longer emitted as malformed ROS launch
  arguments.
- The lSIM RViz PointCloud2 display now uses Jazzy's explicit best-effort,
  volatile QoS schema so the deskewed SensorDataQoS cloud is rendered.

## 0.3.0-rc6 - 2026-08-08

### Changed

- Renamed ROS package `pure_nmea_gga_conversion` to `pure_nmea_gnss_conversion`.
- Removed the unused optional intensity-filter package and its bringup branches.
- Updated source directories, package dependencies, launch package references, CI, scripts, repository checks, and documentation.
- Preserved existing estimator executable names, node names, C++ include roots/namespaces, component plugin types, topic names, parameter keys, and numerical behavior.

### Migration

- Delete `build/`, `install/`, and `log/` before rebuilding.
- Update downstream NMEA package dependencies and `ros2 pkg prefix` calls to the new package name.
- Remove downstream references to the retired intensity-filter package and `use_snow_filter` launch argument.

## 0.3.0-rc5 - 2026-08-08

### Added

- CPU-only Autoware Jazzy Docker image pinned to
  `ghcr.io/autowarefoundation/autoware:universe-devel-jazzy-1.9.0`.
- Docker Compose profiles for headless LSim and optional RViz software
  rendering.
- `run_autoware_lsim_docker.sh`, which builds the overlay, launches the
  localization replacement, replays a bag, automatically publishes an initial
  map anchor without GNSS, records test/reference outputs, and shuts down
  cleanly.
- Static Dockerfile, Compose, container-runner, and wrapper dry-run checks.

### Changed

- Autoware LSim is now documented as a Docker stage; native rosbag-only testing
  remains the first validation stage.
- RViz is disabled by default for CPU-only/headless timing evaluation.

## 0.3.0-rc4 - 2026-08-08

### Added

- Added `pure_autoware_localization_adapter`, which converts fused `map -> base_link` odometry to Autoware's `/localization/kinematic_state`, pose-with-covariance, acceleration, and direct TF interfaces using standard ROS messages.
- Added `autoware_lsim_localization.launch.py` for localization-only Autoware logging simulation with Autoware map/localization, perception, planning, control, system, and API modules disabled.
- Added `use_map_odom_fusion`, `use_imu_deskew`, input-topic, fused-output, and fusion-TF launch arguments so standalone bag replay and Autoware integration use the same estimator configuration.
- Added `script/play_localization_bag.sh` to normalize bag input topics, preserve sensor static TF by default, and isolate recorded localization outputs under `/reference/...`.
- Added a staged standalone-to-Autoware evaluation guide and a ROS-independent acceleration-estimator test.

### Compatibility

- Existing `pure_*` package names, estimator C++ namespaces, custom GNSS message type, estimator topics, frames, and YAML keys remain unchanged.
- `pure_autoware_localization_adapter` is additive and has no compile-time dependency on Autoware message packages.
- The Autoware launch resolves `autoware_launch` only when that launch file is invoked; standalone builds and bag tests do not require an Autoware workspace.
- This is a localization-only Logging Simulation integration. Full planning/control use still requires map/routing, initialization/status integration, vehicle interfaces, and separate validation.

## 0.3.0-rc3 - 2026-08-08

### Changed

- Renamed the repository and display name to `gicp_gnss_odom_localizer` / **GICP–GNSS Odom Localizer**.
- Preserved the established ROS package names instead of adding a `gicp_gnss_odom_` prefix to every package.
- Preserved CMake project names, C++ namespaces, include paths, component plugin types, executable names, custom message type `pure_gnss_msgs/msg/GnssFusionInput`, node names, topics, frames, and parameter keys.
- Updated project-owned package versions to `0.3.0` without changing their package identities.
- Added repository checks that reject accidental reintroduction of the discarded fully-prefixed package layout.
- Kept all RC3 localization algorithms and numerical configuration unchanged.

### Compatibility

- Existing deployments that use the `pure_*` package names do not need source-level package or message-type migration.
- Do not overlay this tree on the discarded `gicp_gnss_odom_localizer-0.3.0-rc2` build/install directories. Remove `build/`, `install/`, and `log/`, then rebuild cleanly.
- The previous `mapless_iv_localizer-0.2.0-rc3` and this release have equivalent estimator source/configuration except for project metadata, documentation, package versions, and identity checks.

## 0.2.0-rc3 - 2026-08-07

### Changed

- Removed the binary LiDAR `degenerate/normal` state, entry/clear streaks, critical latch, and threshold-triggered source/initial-guess behavior.
- Removed binary `/localization/lidar_degenerate`, `/localization/lidar_pose_mode`, and `/localization/lidar_degeneracy_debug` outputs and their public YAML switches.
- Retained the selected registration Hessian as a continuous SE(2) directional-information matrix for fixed-lag weighting.
- Replaced binary covariance multiplication with a bounded continuous scale derived from normalized information deficit.
- Kept registration acceptance separate from observability: a scan is rejected only for normal registration failures such as no selected path, non-convergence, non-finite output, invalid timing, or the mode-specific fitness gate.
- Made optional wheel correction a continuous weak-direction blend. It remains disabled by default and requires wheel speed explicitly.
- Added continuous observability diagnostics and a ROS-independent continuity/bounds test.

### Compatibility

- Delete obsolete `lidar_odom.degeneracy_detection.*`, `wheel_speed.degeneracy.*`, `out_degeneracy*`, and `out_pose_mode*` keys from deployment YAML files; they are no longer declared.
- Use `/diagnostics` for continuous information ratios and registration rejection reasons. An optional string stream is available at `/localization/lidar_observability_debug`.
- `lidar_odom.tracking_mode`, both primary tracking implementations, GNSS/NMEA processing, and the optional-wheel requirement are unchanged from RC2.

## 0.2.0-rc2 - 2026-08-07

### Added

- Added `lidar_odom.tracking_mode` with `scan_to_scan` and `scan_to_submap` primary paths.
- Added `param_scan_to_submap.yaml` while retaining `param.yaml` as the backward-compatible scan-to-scan profile.
- Added scan-to-scan warm-up, scheduled interim propagation, configurable failure fallback, and controlled submap reset states.
- Added tracking-mode selection tests and numerical submap-anchor/re-anchoring checks.

### Changed

- Reworked the local map into an independent anchor frame rather than permanently baking keyframes into `odom`.
- Rebuilds recent submap keyframe placement from the fixed-lag optimizer's retained pose window and re-anchors the rolling map when old keyframes are removed.
- Uses a successful scan-to-submap result as the selected relative graph factor instead of adding it as a second fake absolute observation beside scan-to-scan.
- Keeps rejected fallback scans out of an already-ready submap.
- Decoupled LiDAR Hessian degeneracy detection from wheel-speed availability; wheel speed remains optional.
- Rejects undersized filtered clouds without replacing the previous accepted scan target.
- Expanded tracking/submap diagnostics, validation, documentation, and parameter checks.

### Compatibility

- Existing parameter files continue to select `scan_to_scan`. Setting `lidar_odom.tracking_mode: scan_to_scan` restores the retained primary path.
- `lidar_odom.local_map.enable` remains the optional periodic local-map factor switch for `scan_to_scan`; `scan_to_submap` creates its required active submap independently.
- Tracking mode is selected at startup and is not a validated dynamic parameter transition.

## 0.2.0-rc1 - 2026-08-07

### Changed

- Reworked GNSS initialization and outage return around a multi-observation state machine.
- Removed active timeout/single-fix GNSS force acceptance.
- Added robust weighted SE(2) recovery alignment, one-isolated-outlier rejection, heading-source consistency checks, and bounded corrections.
- Made antenna-vs-base observation semantics explicit in `GnssFusionInput`.
- Preserved NMEA position/covariance and dual-antenna, Doppler, trajectory, corrected-IMU trajectory correction, and bounded IMU propagation paths.
- Suppressed legacy base-pose outputs while yaw or antenna geometry is unknown.
- Added strict timestamp, TF, point layout, and IMU/twist coverage checks to deskew.
- Added scan-Hessian anisotropic information to fixed-lag SE(2) smoothing.
- Added optional gated local-keyframe registration factors while retaining scan-to-scan primary tracking.
- Added graph rollback and long-scan-gap tracker reset.
- Disabled experimental control shaping, wheel scale estimation, ZUPT, NHC, and local-map correction in public defaults.
- Added ROS-independent algorithm tests, repository checks, CI, architecture/tuning/migration documentation, and public project metadata.

### Compatibility

- `GnssFusionInput.msg` changed and requires rebuilding all dependent workspaces.
- A point cloud without per-point timing is rejected unless `allow_linear_time_fallback=true`.
- Missing static sensor TFs are no longer treated as identity.
- Position-only single-antenna GGA no longer appears on legacy base-pose topics.
- Deprecated GNSS force-accept and guessed-heading parameters are accepted only as no-op migration keys.
