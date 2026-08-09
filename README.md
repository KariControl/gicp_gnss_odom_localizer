# gicp_gnss_odom_localizer

`gicp_gnss_odom_localizer` is a ROS 2 workspace for prior-map-free planar GICP LiDAR–IMU odometry with optional NMEA GNSS anchoring. LiDAR tracking can run with either scan-to-scan or scan-to-local-submap as the primary registration path, selected in YAML. The workspace also includes a bounded fixed-lag SE(2) smoother and treats GNSS outage recovery as a multi-observation alignment problem rather than a one-fix reset.

The workspace is derived from `KariControl/small_lidar_inertial_dead_reckoning`; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

## Release status

This `0.3.0-rc6` release keeps the repository/display name `gicp_gnss_odom_localizer`, renames `pure_nmea_gga_conversion` to `pure_nmea_gnss_conversion`, and removes the unused optional intensity-filter package. Existing estimator node names, topic names, parameter keys, C++ include roots/namespaces, component plugin types, and custom message types remain unchanged. Read [docs/migration_0_3.md](docs/migration_0_3.md) before rebuilding.

The estimator is intended for research and engineering evaluation. It is not a safety-certified localization system and must not be used as the sole input to a safety function.

### ROS package identity

The repository name is `gicp_gnss_odom_localizer`, while the internal ROS packages use short responsibility-based names. RC6 changes the NMEA package identity listed above and removes the intensity-filter package. The NMEA executable/node remains `pure_nmea_gga_conversion` / `nmea_gga_conversion`. Estimator topics, frames, YAML parameter names, and `pure_gnss_msgs/msg/GnssFusionInput` are unchanged.

## Main changes in 0.2

- Strict IMU deskew: per-point timing, IMU coverage, data gaps, point-cloud layout, and static TFs are validated before a cloud is accepted.
- `lidar_odom.tracking_mode` selects `scan_to_scan` or `scan_to_submap`; `scan_to_scan` remains the backward-compatible default.
- In `scan_to_submap` mode, scan-to-scan is retained for submap warm-up, scheduled interim propagation, consistency monitoring, and failure fallback.
- The active submap is stored in an independent anchor frame and rebuilt from fixed-lag-optimized keyframe poses, avoiding the stale-odom map distortion that degraded the earlier submap implementation.
- Scan Hessian anisotropy is retained as continuous directional information in the SE(2) smoother and covariance. No binary LiDAR “degenerate/normal” state is used.
- GNSS initialization and return from outage require a contiguous multi-fix window with observable and self-consistent SE(2) alignment.
- GNSS recovery corrections are bounded per update and per second. The old timeout-triggered single-fix force-accept path is removed.
- Single-antenna GGA positions remain antenna observations. The fusion node applies the lever arm in its measurement model.
- NMEA heading sources remain available: dual antenna, Doppler course over ground, GNSS trajectory, trajectory corrected by `/localization/imu_corrected`, and bounded corrected-IMU propagation from a valid absolute heading seed.
- Unknown heading is represented explicitly by `heading_valid=false`; it is not manufactured from an initial or previous yaw.

## Architecture

```text
/points_raw --------> pure_imu_undistortion --------> /localization/points_undistorted
/imu ---------------^

/localization/points_undistorted --> pure_lidar_gyro_odometer --> /localization/gyro_lidar_odom
/imu -----------------------------^                              --> /localization/imu_corrected
                                                               --> /localization/is_stopped

Optional GNSS:
/nmea_sentence --------------------> pure_nmea_gnss_conversion --> /localization/gnss_fusion_input
/fix_velocity --------------------^                            --> legacy base-pose topics only
/localization/imu_corrected -------^                                when yaw + lever arm are valid
/localization/is_stopped ----------^

/localization/gyro_lidar_odom -----> pure_gnss_map_odom_fusion --> /localization/ekf_odom
/localization/gnss_fusion_input ---^                            --> map -> odom TF
```

See [docs/architecture.md](docs/architecture.md) for state and observation details.

## Packages

| Package | Purpose |
|---|---|
| `pure_imu_undistortion` | Strict rotational deskew and optional time-aligned translational deskew |
| `pure_lidar_gyro_odometer` | Selectable scan-to-scan or scan-to-submap GICP/VGICP, corrected IMU, continuous directional observability weighting, covariance, fixed-lag smoothing |
| `pure_nmea_gnss_conversion` | NMEA GGA projection, covariance, heading-source selection, antenna-observation output |
| `pure_gnss_msgs` | Explicit GNSS observation message used by fusion |
| `pure_gnss_map_odom_fusion` | Planar map-to-odom filtering, initialization, outage detection, bounded recovery |
| `pure_odometry_bringup` | Composable and standalone launch files |
| `pure_autoware_localization_adapter` | Standard-message bridge from fused odometry to Autoware localization topics, acceleration, and direct map-to-base TF |
| `small_gicp` | Registration dependency, included as a Git submodule/source tree |

## Supported assumptions

The public configuration assumes a ground vehicle whose main localization state is planar `(x, y, yaw)`. Roll, pitch, and height can be present in sensor transforms and registration, but the smoother and GNSS anchor are SE(2). NHC and ZUPT are optional and disabled by default because they are vehicle- and deployment-dependent.

Required data:

- `sensor_msgs/msg/PointCloud2` with a valid per-point time field for strict deskew;
- `sensor_msgs/msg/Imu` with monotonic timestamps;
- static transforms from `base_link` to the LiDAR and IMU frames;
- for single-antenna GNSS fusion, a static transform from `base_link` to the antenna frame, or an explicitly enabled parameter fallback.

## Build

The repository is a workspace root. ROS 2 Jazzy on Ubuntu 24.04 is the primary target of the included CI.

```bash
git clone --recurse-submodules <repository-url> gicp_gnss_odom_localizer
cd gicp_gnss_odom_localizer

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

For an archive containing `src/small_gicp`, no submodule command is needed.

## Run LiDAR + IMU only

The default parameter file keeps the existing scan-to-scan primary mode:

```bash
ros2 launch pure_odometry_bringup odometry_container.launch.py \
  use_gnss:=false
```

To use scan-to-submap as the primary mode, pass the supplied profile:

```bash
ros2 launch pure_odometry_bringup odometry_container.launch.py \
  use_gnss:=false \
  odom_param:=$(ros2 pkg prefix pure_lidar_gyro_odometer)/share/pure_lidar_gyro_odometer/param/param_scan_to_submap.yaml
```

The standalone launch accepts the same `odom_param` argument. Use `use_sim_time:=true` when replaying a bag with `/clock`. The mode can also be changed in any odometer YAML:

```yaml
lidar_odom.tracking_mode: scan_to_submap  # or scan_to_scan
```

The selection is read at node startup; it is not a live runtime mode switch.
`scan_to_submap` requires the bundled SE(2) smoother to remain enabled.

## Rosbag-first and Autoware Logging Simulation evaluation

Validate the estimator by itself before replacing Autoware localization. Stage A
uses the normal ROS 2 workspace and `play_localization_bag.sh`. Stage B is fully
containerized, so the host does not need an Autoware source workspace.

Standalone replay starts with:

```bash
ros2 launch pure_odometry_bringup odometry_standalone.launch.py \
  use_sim_time:=true \
  use_gnss:=false

./script/play_localization_bag.sh \
  --bag <bag_path> \
  --points <recorded_pointcloud_topic> \
  --imu <recorded_imu_topic>
```

After standalone scan-to-scan and scan-to-submap comparisons pass, run the
localization-only Autoware LSim in Docker:

```bash
./script/run_autoware_lsim_docker.sh \
  --bag /absolute/path/to/bag \
  --points /sensing/lidar/top/pointcloud_raw \
  --imu /sensing/imu/tamagawa/imu_raw
```

The Docker profile is pinned to the official CPU-only Autoware
`universe-devel-jazzy-1.9.0` image. It disables Autoware's map/localization,
perception, planning, control, system, API, and sensor drivers; it requires
neither a CUDA GPU nor a PCD/Lanelet2 map. Without GNSS, it publishes
`/initialpose` automatically after the first LiDAR odometry sample. Evaluation
outputs and isolated `/reference/...` topics are recorded below
`docker_output/` by default.

Use `--tracking-mode scan_to_submap` for the alternative primary registration
mode, `--nmea <topic>` to enable GNSS, `--already-deskewed` for a pre-corrected
cloud, and `--rviz` only when CPU software-rendered visualization is needed.

See [docs/rosbag_and_autoware_lsim_evaluation.md](docs/rosbag_and_autoware_lsim_evaluation.md)
and [docker/autoware_lsim/README.md](docker/autoware_lsim/README.md) for exact
commands, TF policies, output layout, metrics, and limitations.

## Run with NMEA GNSS

```bash
ros2 launch pure_odometry_bringup odometry_container.launch.py \
  use_gnss:=true \
  use_imu_yaw_rate_heading:=true
```

Before enabling GNSS, set the map origin and antenna frame in `src/pure_nmea_gnss_conversion/param/param.yaml`, then publish the calibrated static extrinsics. Example frame topology:

```text
map -> odom -> base_link -> lidar
                         -> imu
                         -> gnss/0
```

### What the NMEA node estimates

The NMEA path intentionally remains in this release:

1. **Position:** GGA latitude/longitude are projected to the configured local map origin.
2. **Position covariance/confidence:** GGA fix quality, HDOP, and differential-correction age are converted to a configurable horizontal sigma and confidence.
3. **Absolute yaw sources, in priority order:**
   - dual-antenna baseline;
   - Doppler course over ground, after speed and uncertainty gates;
   - GNSS trajectory chord, after minimum-baseline and confidence gates.
4. **Corrected IMU use:** `/localization/imu_corrected` is strictly integrated only when it covers the full interval with bounded sample and boundary gaps. It can:
   - curve-correct a trajectory chord using half of the integrated yaw change;
   - propagate the most recent valid absolute heading for a bounded duration.

The corrected IMU does **not** create absolute yaw by itself. A dual-antenna, Doppler, or trajectory seed is required first. See [docs/nmea_heading_and_covariance.md](docs/nmea_heading_and_covariance.md).

### Position-only GGA behavior

A GGA fix without valid yaw is still sent to `/localization/gnss_fusion_input` with:

- `heading_valid=false`;
- `position_is_base_link=false`;
- `observation_point_in_base` set to the antenna lever arm when available.

The legacy `/localization/global_pose`, pose-with-covariance, and GNSS odometry outputs are suppressed until a valid base-link yaw and antenna geometry exist. This prevents an antenna coordinate plus identity quaternion from being mislabeled as a `base_link` pose.

## GNSS outage and return

The fusion state machine is:

```text
UNINITIALIZED -> TRACKING -> OUTAGE -> REACQUIRING -> RECOVERING -> TRACKING
                                          |
                                          +-> RECOVERING_XY_ONLY -> TRACKING_XY_ONLY
```

Initialization and reacquisition use a contiguous window of synchronized GNSS/odom samples. Yaw must be observable from consistent direct headings, sufficient odometry/GNSS motion, or both. The window is rejected for time gaps, excessive RMS, excessive single-point residual, inconsistent headings, or disagreement between heading sources.

During `RECOVERING`, `map -> odom` approaches the accepted target with both per-update and per-second limits. A late sliding-window target is also gated against the current recovery target. Details are in [docs/gnss_recovery.md](docs/gnss_recovery.md).

An opt-in, fail-closed XY-only branch supports a stopped single-antenna vehicle
that regains RTK position while yaw remains unobservable. It requires strict
fix, covariance, stationary-motion, timing, residual, retained-yaw, and
lever-arm gates. It corrects translation only, preserves yaw and yaw
uncertainty, reports a degraded `*_XY_ONLY` state, and yields immediately to the
full SE(2) path when yaw becomes observable. Generic bringup keeps it disabled;
the Hesai single-antenna parking-bag runner enables it explicitly.

## LiDAR odometry and drift control

Two primary tracking modes are available:

- `scan_to_scan` registers every accepted scan against the previous accepted scan. It is the default so existing deployments can return to the original behavior by changing one YAML value.
- `scan_to_submap` registers the current scan against a rolling local keyframe submap after a scan-to-scan warm-up. A valid submap result is the primary factor. Scan-to-scan remains available for scheduled interim frames and as a configurable fallback after an attempted submap registration fails.

The active submap is not baked directly into `odom`. Keyframes are stored in a submap-anchor frame, recent keyframe poses are repaired from the fixed-lag optimizer, the map is rebuilt after meaningful corrections, and rolling re-anchoring keeps coordinates bounded. Consecutive failed submap attempts trigger a controlled submap reset and warm-up rather than contaminating the target with failed fallback scans.

Both modes use:

- corrected-IMU yaw factors;
- a bounded fixed-lag SE(2) graph;
- the reduced registration Hessian as an anisotropic information matrix;
- continuous Hessian-derived directional information weighting, independent of whether wheel speed exists;
- optional ZUPT, NHC, and wheel assistance;
- transactional graph rollback and ordinary registration-quality rejection for non-convergence, non-finite output, invalid timing, or fitness-gate failure.

Wheel speed remains optional in both modes. See [docs/lidar_odometry.md](docs/lidar_odometry.md) and [docs/tuning.md](docs/tuning.md) for mode behavior, gates, diagnostics, and bag comparison.

## Strict deskew behavior

By default a cloud is rejected when any of these conditions holds:

- no supported per-point time field;
- point times are non-finite, have no usable span, or disagree with the cloud timestamp;
- IMU samples do not cover the entire scan or contain a gap over the configured threshold;
- required `base_link <- scan` or `base_link <- imu` static TF is unavailable;
- the `PointCloud2` layout is malformed or big-endian;
- translation deskew is enabled but the selected speed source does not cover the interval.

`allow_linear_time_fallback` and `allow_default_speed_fallback` are explicit compatibility switches and remain false in bundled configurations.

## Important defaults

- `use_gnss:=false` at bringup level;
- corrected-IMU heading support enabled when the GNSS node is launched;
- `lidar_odom.tracking_mode: scan_to_scan` in the generic profile;
- the legacy periodic local-map factor disabled in scan-to-scan mode;
- ZUPT and NHC disabled;
- binary LiDAR degeneracy state, threshold-triggered mode switching, and binary pose-mode topics removed;
- optional continuous wheel observability assistance, wheel-speed scale learning, and low-speed shaping disabled;
- control-oriented filtered odometry disabled;
- single-fix GNSS force acceptance unavailable;
- parameter antenna fallback disabled unless explicitly selected.

## Tests

ROS-independent algorithm tests can run without ROS:

```bash
./tools/run_reference_tests.sh
```

Repository checks:

```bash
python3 tools/check_repository.py
```

A full release check in a ROS environment should include:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Bag-level acceptance criteria are listed in [docs/known_limitations.md](docs/known_limitations.md).

## Documentation

- [Architecture](docs/architecture.md)
- [NMEA heading and covariance](docs/nmea_heading_and_covariance.md)
- [GNSS outage/recovery](docs/gnss_recovery.md)
- [LiDAR odometry and smoother](docs/lidar_odometry.md)
- [Tuning and validation](docs/tuning.md)
- [Known limitations](docs/known_limitations.md)
- [Rosbag and Autoware Logging Simulation evaluation](docs/rosbag_and_autoware_lsim_evaluation.md)
- [0.2 migration guide](docs/migration_0_2.md)
- [Single-antenna algorithm notes](docs/algorithm_spec_single_antenna.md)

## Contributing and security

See [CONTRIBUTING.md](CONTRIBUTING.md) and [SECURITY.md](SECURITY.md). The project is licensed under Apache-2.0.
