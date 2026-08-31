# GICP–GNSS Odom Localizer

<p align="center">
  <a href="docs/evaluation/assets/autoware_lsim_hesai_course_2/rviz_replay.webm"><img src="docs/evaluation/assets/autoware_lsim_hesai_course_2/rviz_poster.png" alt="Hesai Course 2 Autoware localization-interface evaluation poster in RViz" width="900"></a>
</p>

`gicp_gnss_odom_localizer` is a ROS 2 planar LiDAR–IMU–GNSS localization stack
for research and engineering evaluation. It provides LiDAR/IMU local odometry,
optional GNSS-based global localization, and an isolated scan-to-submap output
through standard ROS 2 messages and TF. Autoware support is an optional
downstream localization-interface adapter, not a core dependency.

The stack is prior-map-free: **it does not require a PCD map. Its
standard profiles also do not require wheel speed or a CUDA-capable GPU**.

## Intended uses and key capabilities

- **Robot odometry:** estimate local `x`, `y`, and yaw from LiDAR and IMU only.
- **Global vehicle or robot localization:** anchor LiDAR/IMU odometry with NMEA
  GGA/RTK GNSS while applying the calibrated antenna lever arm.
- **No required PCD map or wheel speed:** it does not require a PCD map. Its
  standard profiles also do not require wheel speed or a CUDA-capable GPU.
- **Optional scan-to-submap output:** run rolling-submap matching in isolated
  processes without feeding corrections into the scan-to-scan odometer.
- **General ROS 2 integration:** consume the odometry topics and TF directly in
  a robot, navigation, mapping, or automated-driving stack.
- **Optional Autoware example:** use the separate adapter to translate the same
  estimator outputs into Autoware pose, kinematic-state,
  twist-with-covariance, acceleration, and TF interfaces. The core estimators
  do not depend on Autoware.
- **CPU-only operation:** the estimator has no CUDA dependency; CPU-only
  operation has also been demonstrated.

### Operating modes

| Mode | Processing path | Intended trade-off |
|---|---|---|
| **Scan-to-scan** | Strict deskew, scan-to-scan GICP, and the SE(2) smoother | Default path with fewer processes; publishes `/localization/gyro_lidar_odom` and can optionally feed GNSS fusion. CPU and RSS are not yet quantified. |
| **Scan-to-submap** | Keeps the scan-to-scan path unchanged and adds accepted-scan snapshots, an external rolling-submap matcher, and isolated local/global compositors | Adds computation and latency for separate scan-to-submap outputs at `/localization/precision_local_odom` and, with a healthy global anchor, `/localization/precision_global_odom`. |

The scan-to-submap mode runs alongside the primary scan-to-scan odometer. Its
corrections never feed back into scan-to-scan, and the mode is selected at
launch rather than switched while running.

The estimator is planar SE(2), intended for research and engineering
evaluation, and is not a safety-certified localization system. CPU utilization
and RSS have not yet been measured; CPU-only support is not a claim of low CPU
load.

## Build

ROS 2 Jazzy on Ubuntu 24.04 is the primary target.

```bash
git clone --recurse-submodules \
  https://github.com/KariControl/gicp_gnss_odom_localizer.git
cd gicp_gnss_odom_localizer

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Quick start

Every terminal must source ROS 2 and `install/setup.bash`. Publish the calibrated
sensor static transforms before expecting accepted localization output.

The managed Hesai, LiDAR/IMU GLIM, and Docker runners accept
`--rqt-robot-monitor` to open the aggregated `/diagnostics_agg` view in a
graphical session. The option is disabled by default so headless and timing
evaluation runs are unchanged.

### Scan-to-scan mode: LiDAR + IMU local odometry

```bash
ros2 launch pure_odometry_bringup odometry_container.launch.py \
  use_gnss:=false \
  points_input_topic:=/points_raw \
  imu_input_topic:=/imu
```

The primary output is `/localization/gyro_lidar_odom` in the `odom` frame.

For rosbag replay, start the estimator with simulated time:

```bash
ros2 launch pure_odometry_bringup odometry_container.launch.py \
  use_sim_time:=true \
  use_gnss:=false \
  points_input_topic:=/points_raw \
  imu_input_topic:=/imu
```

Then use the replay helper in a second sourced terminal:

```bash
BAG_DIR=/path/to/recording
POINTS_TOPIC=/recorded/pointcloud
IMU_TOPIC=/recorded/imu

./script/play_localization_bag.sh \
  --bag "$BAG_DIR" --points "$POINTS_TOPIC" --imu "$IMU_TOPIC"
```

The helper isolates recorded dynamic localization TF by default while retaining
recorded static sensor transforms.

### Scan-to-submap mode: rolling-submap output

Keep the scan-to-scan odometer in `scan_to_scan`, enable only its accepted-scan
snapshot bridge, and start the overlay in another sourced terminal:

```bash
ros2 launch pure_odometry_bringup odometry_container.launch.py \
  use_gnss:=false \
  odom_override_param:=$(ros2 pkg prefix pure_precision_bringup)/share/pure_precision_bringup/config/submap_snapshot_override.yaml

ros2 launch pure_precision_bringup precision_overlay.launch.py
```

This adds `/localization/precision_local_odom`; it does not replace or feed back
into `/localization/gyro_lidar_odom`.

### LiDAR + IMU + GNSS global localization

When the packaged projection matches the deployment's map metadata, launch with
the calibrated GNSS static TF and no evaluation-origin override:

```bash
ros2 launch pure_odometry_bringup odometry_container.launch.py \
  use_gnss:=true
```

For a different map projection, create a deployment-owned ROS parameter override
whose projector, datum, origin, and scale match that map's projector metadata;
do not pass `map_projector_info.yaml` directly as a ROS parameter file.

The principal global output is `/localization/ekf_odom`, with the corresponding
`map -> odom` TF when TF publication is enabled. See
[NMEA observation semantics](docs/nmea_heading_and_covariance.md) and
[GNSS initialization and recovery](docs/gnss_recovery.md) before deployment.

### Optional integration example: Autoware localization-interface integration

Autoware is one possible downstream consumer, not a dependency of the
localization stack. The optional Docker workflow connects the standard fused
output to Autoware localization interfaces without requiring a host Autoware
workspace.

The retained `autoware_lsim` filenames and command names are project-local
compatibility identifiers, not names of an official Autoware component or
workflow.

Before replaying a recording, the version-selected Autoware 1.9.0 interface and
diagnostic contract can be exercised without a bag:

```bash
./script/run_autoware_localization_contract_docker.sh
```

This deterministic test launches the real Autoware
`pose_instability_detector` and `localization_error_monitor`, checks all adapter
outputs, requires exactly one runtime `/tf` publisher endpoint owned by the
adapter for `map -> base_link`, confirms that no competing dynamic `base_link`
parent is observed, and drives both monitors through normal, injected-fault,
and recovery phases. The Jazzy package tests additionally launch the container,
standalone, main GNSS, NMEA-wrapper, LiDAR-IMU-only, and precision-overlay
profiles in isolated ROS domains. They check the exact `/tf` endpoint multiset,
each owner's effective `publish_tf` and frame parameters, and the globally
emitted dynamic-edge set. Jazzy's Python API cannot attribute an individual TF
sample to its endpoint GID, and a bag alone cannot identify two publishers that
emit the same TF edge. This is
an interface and diagnostic-response test; it does not launch planning or
control, assess localization accuracy, or detect input dropout.

A minimal run is:

```bash
BAG_DIR=/path/to/recording

./script/run_autoware_lsim_docker.sh \
  --bag "$BAG_DIR" \
  --points /recorded/pointcloud \
  --imu /recorded/imu
```

See the [Docker guide](docker/autoware_lsim/README.md) for GNSS, sensor profiles,
RViz, and scan-to-submap options.

For the ROSBAG2/3 presentation view, add `--rviz --rviz-sample-vehicle`. It shows
the illustrative Autoware Lexus body, a generated line trajectory, live
pose/yaw/speed and interface/TF/rate/registration/GNSS status in a dedicated
RViz Displays entry, and a bounded XY-only covariance ellipse in the 3D view.
The current Hesai profile applies a visual-only `-1.66 m` mesh offset fitted to
the observed ground. The sample mesh is not evidence of the recorded vehicle
model or geometry, and the offset is not a sensor-height calibration.

### Replay the public synthetic rosbag

Complete the [build steps](#build) first. Then start the LiDAR/IMU estimator
from the repository root in terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch pure_odometry_bringup lidar_imu_only.launch.py \
  use_sim_time:=true \
  use_imu_deskew:=true \
  points_input_topic:=/points_raw \
  imu_input_topic:=/imu
```

Replay the included synthetic rosbag from terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

./script/play_localization_bag.sh \
  --bag data/synthetic_output_pointcloud2 \
  --points /pandar_points_ex \
  --imu /sensor/imu/data_raw \
  --clock-frequency 100 \
  --tf-policy isolate-dynamic \
  -- --disable-keyboard-controls \
  --topics /pandar_points_ex /sensor/imu/data_raw /tf_static
```

Keep `--tf-policy isolate-dynamic`; `isolate-all` removes the static sensor
transforms required by the fail-closed deskewer and odometer. The resulting
local odometry is published on `/localization/gyro_lidar_odom`.

## Evaluation results

The evaluation uses [GLIM](https://github.com/koide3/glim) as a correlated
LiDAR/IMU pseudo-reference; it is not independent ground truth. The RMSE values
compare the estimator output with that pseudo-reference. Representative results
are summarized below.

The published Hesai primary accuracy result uses exact-initial-pose alignment.
Its initialization timing and typed-authority checks are evaluated separately
and do not contribute to the primary RMSE values or plots.

| Sensor | Scope | XY RMSE (scan-to-scan mode) | XY RMSE (submap mode) | Yaw RMSE (scan-to-scan mode) | Yaw RMSE (submap mode) |
|---|---|---:|---:|---:|---:|
| Velodyne 32-Line + External IMU (No GNSS) | local | 0.495 m | 0.216 m | 0.314° | 0.123° |
| MID-360  + Internal IMU (No GNSS) | local | 0.630 m | 0.294 m | 2.906° | 0.602° |
| Hesai + External IMU + RTK-GNSS | local (current 2026-08-25 profile) | 1.457 m | 0.389 m | 0.917° | 0.792° |
| Hesai + External IMU + RTK-GNSS | global (current 2026-08-25 profile) | 1.406 m | 0.516 m | 2.099° | 1.568° |

### Representative plots

#### Velodyne 32-Line + External IMU

[![Velodyne LiDAR and external IMU scan-to-scan and scan-to-submap trajectories](docs/evaluation/assets/velodyne_32line_external_imu/trajectory.png)](docs/evaluation/assets/velodyne_32line_external_imu/trajectory.png)

*Scan-to-scan and scan-to-submap.*

**Assessment:** Scan-to-submap met the project's predefined accuracy criteria
over the accepted 45.991 s prefix, with **0.2156 m XY RMSE** and **0.1227 deg yaw
RMSE**.

#### Livox MID-360 + Internal IMU

[![Livox MID-360 and internal IMU tuned scan-to-scan and rolling scan-to-submap trajectories](docs/evaluation/assets/livox_mid360_internal_imu/trajectory.png)](docs/evaluation/assets/livox_mid360_internal_imu/trajectory.png)

*Tuned scan-to-scan and rolling scan-to-submap.*

**Assessment:** Rolling scan-to-submap passed all predefined acceptance
criteria over the complete evaluated interval, with **0.2941 m XY RMSE** and
**0.6024 deg yaw RMSE**. This is sufficient for this recorded LiDAR/IMU-only
evaluation.

#### Hesai 32-Line + IMU + RTK GNSS

[Open the GitHub-hosted Hesai Course 2 RViz replay](https://github.com/user-attachments/assets/6ce8b916-13e5-4779-87e9-c0f477a6e14b)

The global plots compare the GNSS-anchored **scan-to-scan** and
**scan-to-submap** outputs.

**Hesai Course 2 global XY error**

[![Hesai 32-Line, IMU, and RTK GNSS Course 2 scan-to-scan and scan-to-submap global XY error during GNSS outage and recovery](docs/evaluation/assets/hesai_32line_imu_rtk_gnss_course_2/global_xy_error.png)](docs/evaluation/assets/hesai_32line_imu_rtk_gnss_course_2/global_xy_error.png)

**Hesai Course 2 global yaw error**

[![Hesai 32-Line, IMU, and RTK GNSS Course 2 scan-to-scan and scan-to-submap global yaw error during GNSS outage and recovery](docs/evaluation/assets/hesai_32line_imu_rtk_gnss_course_2/global_yaw_error.png)](docs/evaluation/assets/hesai_32line_imu_rtk_gnss_course_2/global_yaw_error.png)

*The hatched span marks the 117.252446 s RTK-Q4-unavailable interval. The red
marker shows Q4 returning; the green marker shows finite global localization
resuming 4.147737 s later. The separately evaluated fusion-outage RMSE window
is 121.500028 s because it follows the fusion-state definition rather than the
raw Q4-gap boundaries.*

**Assessment:** Across 7,805 exact common global samples, scan-to-submap reduced
XY/yaw RMSE from **1.406461 m / 2.099877 deg** to
**0.516555 m / 1.568195 deg**. During the 121.500028 s fusion-outage metric
window, its XY/yaw RMSE was **0.579715 m / 0.861855 deg**, and finite global
localization resumed 4.147737 s after RTK-Q4 returned.

The local plots compare **scan-to-scan** with **scan-to-submap** odometry.

**Hesai Course 2 local XY error**

[![Hesai 32-Line, IMU, and RTK GNSS Course 2 scan-to-scan and scan-to-submap local XY error](docs/evaluation/assets/hesai_32line_imu_rtk_gnss_course_2/local_xy_error.png)](docs/evaluation/assets/hesai_32line_imu_rtk_gnss_course_2/local_xy_error.png)

**Hesai Course 2 local yaw error**

[![Hesai 32-Line, IMU, and RTK GNSS Course 2 scan-to-scan and scan-to-submap local yaw error](docs/evaluation/assets/hesai_32line_imu_rtk_gnss_course_2/local_yaw_error.png)](docs/evaluation/assets/hesai_32line_imu_rtk_gnss_course_2/local_yaw_error.png)

**Assessment:** Across 8,477 exact common local samples, scan-to-submap reduced
XY/yaw RMSE from **1.457149 m / 0.917636 deg** to
**0.389683 m / 0.792790 deg**.

These are dataset-scoped engineering acceptance results. They do not establish
fitness for safety-critical use or accuracy across other datasets,
environments, or sensor installations.

See the [evaluation pages](docs/evaluation/README.md) for full-size plots, error
distributions, methodology, provenance, and limitations.

## Required data

The normal LiDAR/IMU configuration requires:

- `sensor_msgs/msg/PointCloud2` with a valid per-point time field;
- monotonic `sensor_msgs/msg/Imu` samples covering each scan;
- calibrated static transforms from `base_link` to the LiDAR and IMU frames.

GNSS operation additionally requires NMEA GGA input, a projection configuration
that matches the deployment's map metadata, and a calibrated `base_link` to
GNSS-antenna transform. The packaged runtime parameters in
[`param/param.yaml`](src/pure_nmea_gnss_conversion/param/param.yaml) match the
projector type, datum, origin, and scale in
[`map_projector_info.yaml`](src/pure_nmea_gnss_conversion/config/map_projector_info.yaml).
The metadata file is not itself a ROS parameter file. Optional Doppler or
secondary-antenna topics can provide additional heading observations.

See [known limitations](docs/known_limitations.md) before selecting this stack
for a new sensor, environment, or vehicle.

## Public synthetic rosbag sample

The repository includes a 6.4 MiB
[fully procedural LiDAR/IMU rosbag](data/synthetic_output_pointcloud2/) for
trying the localization stack without a private sensor recording. It reproduces
only the PointCloud2 field layout and rounded LiDAR/IMU rates observed in the
private `output_pointcloud2` input. It contains no copied or transformed source
points, trajectory, GNSS, recording timestamp, calibration, or scene geometry.

The current regression run deskewed 121/121 scans without fallback, corrected
1,241/1,241 IMU samples, accepted 118/120 post-initialization registrations,
and ended within 0.0667 m / 0.359 deg of the analytic endpoint. These results
cover one artificial scene and are not a sensor benchmark or real-world
accuracy claim. Follow [Replay the public synthetic
rosbag](#replay-the-public-synthetic-rosbag) for the commands. See the
[dataset card](data/README.md) for provenance, schema, hashes, regeneration, and
limitations.

## Packages

The usual entry points are `pure_odometry_bringup` for scan-to-scan and
`pure_precision_bringup` for the optional scan-to-submap overlay. The remaining
packages are components, interfaces, and narrowly scoped validation support
installed from the same repository.

| Package | Role |
|---|---|
| `pure_odometry_bringup` | Primary launch, configuration, and RViz entry point for the LiDAR–IMU–GNSS localization stack. |
| `pure_precision_bringup` | Launch and configuration entry point for the optional isolated scan-to-submap overlay. |
| `pure_localization_contract` | Reusable runtime TF ownership probe and supported-profile system test. |
| `pure_localization_evaluation_profiles` | Data-only package for recording-specific evaluation profiles and provenance manifests. |
| `pure_localization_interface_adapter` | Converts fused odometry into configurable kinematic-state, pose, twist-with-covariance, acceleration, and `map -> base_link` interfaces; the optional Autoware workflow consumes these outputs. |
| `pure_imu_undistortion` | Validates point timing and deskews LiDAR scans from IMU motion, with optional translation compensation. |
| `pure_lidar_gyro_odometer` | Produces scan-to-scan planar LiDAR–IMU odometry with fixed-lag SE(2) smoothing. |
| `pure_nmea_gnss_conversion` | Converts NMEA GGA and optional heading observations into explicit GNSS fusion inputs. |
| `pure_gnss_map_odom_fusion` | Anchors local odometry in `map` using multi-sample GNSS initialization and bounded outage recovery. |
| `pure_lidar_submap_matcher` | Computes isolated rolling-submap SE(2) corrections from accepted LiDAR scans. |
| `pure_precision_global_localizer` | Composes scan-to-submap local output and guarded GNSS-anchored scan-to-submap global output without publishing TF. |
| `pure_gnss_msgs` | Defines the GNSS observation messages shared by the conversion and fusion packages. |
| `pure_lidar_msgs` | Defines the exact-key scan and correction messages used by the scan-to-submap branch. |

`small_gicp` is an external MIT-licensed dependency included as a Git submodule;
it is not a first-party package in this project.

## Validated environments

| Use case | Operating system | ROS 2 | Autoware | Status and scope |
|---|---|---|---|---|
| Standalone localization | Ubuntu 24.04 | Jazzy | Not required | Primary source-build and rosbag-replay target. |
| Optional Autoware integration | Ubuntu 24.04 | Jazzy | 1.9.0 | Stage A localization-interface and diagnostic-monitor integration in the supplied CPU-only Docker workflow. |
| Other combinations | — | — | — | Not currently claimed; they may work but have not been validated by this project. |

The standalone estimator path does not require Autoware. Stage A has separate
pieces of evidence: the deterministic contract covers localization-facing
topics, TF consistency, and the two real localization monitors; the current
public synthetic replay covers the live localizer path and simulation time but
does not use GNSS. The published private Course 2 result is a historical
2026-08-12 pre-Stage-A run that covers simulation time and GNSS-outage recovery,
but not the new twist stream or monitor contract. No single run currently
combines all three scopes. None establishes full-stack integration, closed-loop
driving, planning/control readiness, or safety certification. CPU-only
execution was demonstrated, but CPU utilization and memory usage were not
measured.

## Sensor-to-output flow

These diagrams intentionally show functional roles and major signal flow, not
the exact ROS node graph or message contract. Exact interfaces and package
mapping are in [Architecture](docs/architecture.md#data-flow). Without GNSS,
the LiDAR/IMU path still publishes local odometry. In the figures, GNSS
"heading" is optional; a valid position-only observation may carry no heading.
The exact sensor input types are `nmea_msgs/msg/Sentence`,
`sensor_msgs/msg/Imu`, and `sensor_msgs/msg/PointCloud2`.

### Scan-to-scan mode

[![Scan-to-scan LiDAR, IMU, and optional GNSS localization data flow](docs/assets/architecture/normal.png)](docs/assets/architecture/normal.png)

The deskew function publishes only the deskewed point cloud. The LiDAR–gyro
odometry function is the source of both the base-frame, yaw-bias-corrected IMU
stream and the stop state used by the single-antenna heading logic. Stop
detection requires a quiet IMU and, when a causal wheel or LiDAR speed estimate
exists, a speed below its threshold; with neither speed source it deliberately
falls back to IMU-only detection.

### Scan-to-submap mode

[![Isolated scan-to-submap precision localization data flow](docs/assets/architecture/precision.png)](docs/assets/architecture/precision.png)

Each `SubmapScan` carries an accepted filtered cloud, its unmodified
scan-to-scan pose, and the immutable exact key `(odom_session_id,
odom_generation, sequence, header.stamp)`. Both precision nodes consume that
snapshot: the matcher estimates a
persistent full-SE(2) correction, while the local/global compositor validates
the corresponding key before applying it to continuous raw odometry. The
typed `FusionAuthority` permits global-anchor updates only while the existing
fusion is fresh and fully healthy. Direct GNSS input is only an outage-yaw
guard in the default profile, not an alternative global-position authority.

The scan-to-submap branch publishes separate message outputs, no TF, and no
feedback into the scan-to-scan odometer or existing GNSS fusion. Full frame,
component, and failure-isolation details are in
[Architecture](docs/architecture.md); the complete stop rule is documented in
[LiDAR/IMU odometry](docs/lidar_odometry.md#event-time-stop-decision).

The container, standalone, and standalone-with-NMEA launches assign
`odom -> base_link` to the gyro odometer. The evaluation-oriented
`lidar_imu_only` launch leaves it disabled by default. The Autoware profile also
disables gyro-odometer and fusion TF publication; its launch configuration
assigns the direct `map -> base_link` transform to the adapter.

## Validation

```bash
python3 tools/check_repository.py
./tools/run_reference_tests.sh
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

See [validation scope](docs/validation.md) for the full release and recording
gates.

## Documentation

- [Evaluation results and published plots](docs/evaluation/README.md)
- [Architecture](docs/architecture.md)
- [LiDAR/IMU odometry and scan-to-submap isolation](docs/lidar_odometry.md)
- [NMEA position, covariance, and heading](docs/nmea_heading_and_covariance.md)
- [GNSS initialization and outage recovery](docs/gnss_recovery.md)
- [Tuning](docs/tuning.md) and [known limitations](docs/known_limitations.md)
- [Rosbag and Autoware localization-interface workflow](docs/rosbag_and_autoware_lsim_evaluation.md)
- [Changelog and migration notes](CHANGELOG.md#unreleased)

## Citation and related work

Machine-readable citation metadata is provided in [`CITATION.cff`](CITATION.cff).
The registration path builds on
[Generalized-ICP (Segal et al., 2009)](https://doi.org/10.15607/RSS.2009.V.021)
through
[`small_gicp` (Koide, 2024)](https://doi.org/10.21105/joss.06948).
Published trajectory evaluation uses
[GLIM (Koide et al., 2024)](https://doi.org/10.1016/j.robot.2024.104750)
only as a correlated LiDAR/IMU pseudo-reference.

[LIO-SAM (Shan et al., 2020)](https://doi.org/10.1109/IROS45743.2020.9341176)
and
[FAST-LIO2 (Xu et al., 2022)](https://doi.org/10.1109/TRO.2022.3141876)
are representative tightly coupled LiDAR-inertial systems included as related
architectural context. They have not been evaluated as head-to-head baselines
in this repository.

See [CONTRIBUTING.md](CONTRIBUTING.md) and [SECURITY.md](SECURITY.md). Licensed
under Apache-2.0. Third-party attribution is in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
