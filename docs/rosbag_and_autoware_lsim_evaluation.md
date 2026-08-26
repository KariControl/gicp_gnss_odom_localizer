# Rosbag and Autoware Localization-Interface Workflow

> Canonical result tables and publication status are indexed at
> [docs/evaluation](evaluation/README.md). This page defines the reusable
> execution workflow; dated result summaries live under that evaluation index.

This workflow deliberately uses the same estimator configuration for a
standalone prerequisite and Stage A integration:

1. validate the localization stack by itself against a recorded bag;
2. keep the estimator unchanged and connect its fused output to Autoware's
   localization runtime interface while Autoware's standard map/localization,
   perception, planning, and control modules remain disabled.

Neither stage requires a prior PCD point-cloud map. The second stage is a
**localization-interface evaluation**, not a full closed-loop Autoware drive.

## 1. Inspect the bag before launching anything

```bash
ros2 bag info <bag_path>
```

Identify at least:

- raw or already deskewed `sensor_msgs/msg/PointCloud2`;
- `sensor_msgs/msg/Imu`;
- `/tf_static` containing calibrated `base_link` to LiDAR and IMU extrinsics, or
  an equivalent vehicle/sensor description that will publish those transforms;
- optional NMEA GGA, secondary GGA, Doppler velocity, and reference localization;
- whether the point cloud contains a supported per-point time field.

Do not replay a recorded localization TF and publish the test localizer's TF on
the same names. The supplied replay helper isolates recorded outputs under
`/reference/...`.

## 2. Prerequisite: standalone rosbag test

Build and source the workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2.1 Start with scan-to-scan and no GNSS

```bash
ros2 launch pure_odometry_bringup odometry_standalone.launch.py \
  use_sim_time:=true \
  use_gnss:=false \
  use_map_odom_fusion:=false \
  use_imu_deskew:=true \
  points_input_topic:=/points_raw \
  imu_input_topic:=/imu
```

Replay a bag whose source topics are, for example,
`/sensing/lidar/top/pointcloud_raw` and `/sensing/imu/imu_raw`:

```bash
./script/play_localization_bag.sh \
  --bag <bag_path> \
  --points /sensing/lidar/top/pointcloud_raw \
  --imu /sensing/imu/imu_raw
```

The primary output is:

```text
/localization/gyro_lidar_odom   nav_msgs/msg/Odometry, odom -> base_link
```

If the input point cloud is already deskewed and has no per-point time field,
turn the internal deskewer off and pass that cloud directly:

```bash
ros2 launch pure_odometry_bringup odometry_standalone.launch.py \
  use_sim_time:=true \
  use_gnss:=false \
  use_map_odom_fusion:=false \
  use_imu_deskew:=false \
  points_input_topic:=/points_raw \
  imu_input_topic:=/imu
```

Do not enable the point-order timing fallback for a quantitative comparison.

### 2.2 Add the isolated scan-to-submap overlay

Keep the odometer in `scan_to_scan`, load the accepted-scan snapshot override,
and launch the scan-to-submap processes separately:

```bash
ros2 launch pure_odometry_bringup odometry_standalone.launch.py \
  use_sim_time:=true \
  use_gnss:=false \
  use_map_odom_fusion:=false \
  odom_override_param:=$(ros2 pkg prefix pure_precision_bringup)/share/pure_precision_bringup/config/submap_snapshot_override.yaml

ros2 launch pure_precision_bringup precision_overlay.launch.py \
  use_sim_time:=true
```

Use the identical bag, replay rate, input remaps, TFs, scan-to-scan odometer YAML,
and metric code for both runs. Compare at least:

- relative pose error and endpoint error;
- yaw error, especially after turns and long corridors;
- registration rejection count;
- accepted-snapshot key coverage and scan-to-scan non-intrusion;
- external matcher acceptance, robust commits, and recovery rebuilds;
- real-time factor and callback latency;
- CPU and memory use.

The external matcher publishes only separate scan-to-submap topics and no TF. A
submap result is not automatically better; retain scan-to-scan output until
representative bags pass both accuracy and non-intrusion gates.

### 2.3 Add GNSS only after LiDAR-IMU odometry is stable

```bash
ros2 launch pure_odometry_bringup odometry_standalone.launch.py \
  use_sim_time:=true \
  use_gnss:=true \
  use_map_odom_fusion:=true \
  fusion_publish_tf:=true
```

Example replay:

```bash
./script/play_localization_bag.sh \
  --bag <bag_path> \
  --points /sensing/lidar/top/pointcloud_raw \
  --imu /sensing/imu/imu_raw \
  --nmea /sensing/gnss/nmea_sentence \
  --fix-velocity /sensing/gnss/fix_velocity
```

The fused output is:

```text
/localization/ekf_odom   nav_msgs/msg/Odometry, map -> base_link
```

Test normal tracking, GNSS loss, FLOAT/no-fix, delayed fixes, one isolated
outlier, stationary return, moving return, and repeated short outages.

## 3. Stage A: Autoware localization-interface integration in Docker

The Autoware stage is containerized. The host needs Docker Engine and the
Docker Compose v2 plugin, but it does **not** need ROS 2, Autoware, CUDA, or a
point-cloud map installed. The supplied image overlays this repository on the
pinned CPU-only Autoware release image:

```text
ghcr.io/autowarefoundation/autoware:universe-devel-jazzy-1.9.0
```

The container starts the same `autoware_lsim_localization.launch.py` used by the
native workflow. Autoware's standard map/localization, perception, planning,
control, system, API, and sensor drivers remain disabled.

### Deterministic interface and diagnostic contract

Before using a recording, exercise the version-selected contract:

```bash
./script/run_autoware_localization_contract_docker.sh
```

It launches the production adapter and the actual Autoware 1.9.0
`pose_instability_detector` and `localization_error_monitor` nodes. Synthetic
normal, pose-fault, covariance-fault, and recovery phases verify adapter
messages, stamps and frames, agreement of `map -> base_link` with the input
pose, absence of a competing dynamic `base_link` parent, Autoware diagnostic
identities/keys, and `OK -> ERROR -> OK` behavior. The launch assigns that TF
edge to the adapter; a bag cannot identify duplicate publishers of the same
edge. This is a deterministic interface contract, not a recording-level
accuracy test.

### 3.1 One-command headless run

From the repository root:

```bash
./script/run_autoware_lsim_docker.sh \
  --bag <bag_path> \
  --points /sensing/lidar/top/pointcloud_raw \
  --imu /sensing/imu/tamagawa/imu_raw
```

The wrapper performs the following sequence:

1. builds a local Docker image using the pinned no-CUDA Autoware image;
2. launches localization-only Autoware plus this estimator, adapter, and the
   two localization monitor nodes;
3. waits until `/localization/kinematic_state` and
   `/localization/twist_with_covariance` have publishers;
4. starts output recording;
5. replays the input bag with recorded localization moved to `/reference/...`;
6. when GNSS is disabled, waits for a positive `/clock` and a nonzero-stamped
   local odometry sample, then publishes the configured `/initialpose`;
7. stops the recorder and launch cleanly after playback finishes.

The default result root is:

```text
docker_output/<bag>_<tracking-mode>_<date>/
```

It contains launch/replay/record logs, the resolved run settings, input bag
information, and a `localization_output` rosbag.

### 3.2 Compare scan-to-scan and isolated scan-to-submap

Run the same bag twice and change only the localization mode. The production
odometer remains scan-to-scan in both runs; scan-to-submap mode enables its
output-only snapshot bridge and starts the external matcher/global overlay:

```bash
./script/run_autoware_lsim_docker.sh \
  --bag <bag_path> \
  --points /sensing/lidar/top/pointcloud_raw \
  --imu /sensing/imu/tamagawa/imu_raw \
  --tracking-mode scan_to_scan \
  --run-name autoware_interface_scan_to_scan
```

```bash
./script/run_autoware_lsim_docker.sh \
  --bag <bag_path> \
  --points /sensing/lidar/top/pointcloud_raw \
  --imu /sensing/imu/tamagawa/imu_raw \
  --tracking-mode scan_to_submap \
  --run-name autoware_interface_scan_to_submap
```

The image build is cached. Add `--no-build` after the first successful build to
skip the explicit build step.

### 3.3 GNSS and outage recovery

Providing the primary NMEA topic enables the GNSS frontend:

```bash
./script/run_autoware_lsim_docker.sh \
  --bag <bag_path> \
  --points /sensing/lidar/top/pointcloud_raw \
  --imu /sensing/imu/tamagawa/imu_raw \
  --nmea /sensing/gnss/nmea_sentence \
  --fix-velocity /sensing/gnss/fix_velocity \
  --run-name autoware_interface_gnss
```

Optional dual-antenna input uses `--nmea-secondary`. Automatic `/initialpose`
is not used when GNSS is enabled; the normal multi-observation initialization
and bounded outage-recovery state machine owns the map anchor.

### 3.4 Already-deskewed point clouds

When the replayed cloud has already been motion-corrected and does not contain a
valid per-point time field, bypass the internal deskewer:

```bash
./script/run_autoware_lsim_docker.sh \
  --bag <bag_path> \
  --points <deskewed_pointcloud_topic> \
  --imu <imu_topic> \
  --already-deskewed
```

Do not use point-order time approximation for a quantitative comparison.

### 3.5 TF source selection

The default is:

```text
--tf-policy isolate-dynamic
Autoware vehicle/sensor description disabled
```

This keeps the bag's calibrated `/tf_static` and moves recorded dynamic `/tf`
to `/reference/tf`.

Use Autoware's vehicle/sensor descriptions only when they exactly match the
recorded installation:

```bash
./script/run_autoware_lsim_docker.sh \
  --bag <bag_path> \
  --points <pointcloud_topic> \
  --imu <imu_topic> \
  --launch-vehicle \
  --vehicle-model <vehicle_model> \
  --sensor-model <sensor_model> \
  --tf-policy isolate-all
```

`isolate-all` without `--launch-vehicle` is rejected because it would remove the
sensor extrinsics required by deskew and registration.

### 3.6 GUI diagnostics and RViz without a discrete GPU

To inspect the aggregated localization diagnostics during replay, launch Robot
Monitor inside the container:

```bash
./script/run_autoware_lsim_docker.sh ... --rqt-robot-monitor
```

It subscribes to `/diagnostics_agg` and may be combined with `--rviz`. The
runner treats the requested monitor process and subscription as required until
replay completes. Either GUI option requires a working host X11 `DISPLAY` and
`xhost` command.

The default Docker run is headless. For occasional visualization, use CPU
software rendering:

```bash
./script/run_autoware_lsim_docker.sh ... --rviz
```

Either GUI option adds `compose.rviz.yaml`, forwards X11, and sets
`LIBGL_ALWAYS_SOFTWARE=1`. Keep both GUI tools disabled when measuring estimator
timing or CPU utilization.

For a presentation view with the Autoware sample Lexus body, add
`--rviz-sample-vehicle`. In addition to the body, the presentation profile shows
a generated line trajectory, current pose/yaw/speed, output rate,
`map -> base_link` state, live interface/registration status, a color-coded GNSS
state, and a bounded 2-sigma XY-only position-covariance ellipse at the visual
ground offset. The live status values appear under the dedicated
`Autoware Localization Status` item in RViz's Displays panel instead of over the
3D scene. The trajectory and status properties come from the actual
kinematic-state, TF, and diagnostic streams; stale data is not reported as
healthy.

The standard 3D odometry covariance display remains off because the planar
interface intentionally carries a very large Z variance. The runner stores
single transient-local trajectory and covariance-marker snapshots for evidence
instead of recording the continuously growing presentation topics into the
output bag. It also requires RViz to subscribe directly to `/diagnostics`, which
fails the run if the custom status Display was not loaded.

For `hesai-rosbag23`, `base_link` is colocated with the LiDAR frame. Robust
ground-plane fits evaluated at the Lexus wheel-contact locations gave median Z
values of `-1.6641 m` for ROSBAG2 and `-1.6565 m` for ROSBAG3. After accounting
for the mesh's approximately `+0.00394 m` minimum Z, the rounded default
visual-only body offset is `-1.66 m`. This is a rendering fit, not a physical
sensor-height calibration. Override it for another recording with
`--rviz-sample-vehicle-z-offset <metres>` after measuring its ground plane. The
offset changes only the mesh rendering, not localization or sensor transforms.

This is a visualization-only integration: it neither launches the sample sensor
kit nor changes the calibrated TF tree. The Lexus is an illustrative Autoware
sample model; it does not represent the recorded vehicle, its geometry, or its
sensor calibration, and the option cannot be combined with `--launch-vehicle`.

### 3.7 Docker controls

Useful options are:

```text
--dry-run          validate paths and print Docker commands without running Docker
--pull-base        pull the pinned Autoware image before building
--build-jobs 1     reduce build parallelism and memory pressure
--build-only       build the image and exit
--no-build         reuse an existing image
--shell            open a shell in the prepared image
--no-record        disable the output rosbag
--output <dir>     select the host result root
--rviz-sample-vehicle-z-offset <m>
                   override the visual-only body/ellipse ground height
```

The Compose service uses host networking and host IPC. It is privileged because
the official Autoware entrypoint enables loopback multicast and applies DDS
network tuning. Use this profile on a trusted local development machine.

## 4. Autoware-facing outputs

`pure_localization_interface_adapter` converts the fused odometry into:

```text
/localization/kinematic_state                         nav_msgs/msg/Odometry
/localization/pose_estimator/pose_with_covariance     geometry_msgs/msg/PoseWithCovarianceStamped
/localization/twist_with_covariance                    geometry_msgs/msg/TwistWithCovarianceStamped
/localization/acceleration                            geometry_msgs/msg/AccelWithCovarianceStamped
map -> base_link                                      TF
```

The acceleration is a bounded, low-pass derivative of the fused twist. It is
provided for interface evaluation, not as a calibrated inertial acceleration
measurement. The adapter resets the derivative on the first sample, time
reversal, or a large timestamp gap and publishes a high covariance for that
reset sample.

Both gyro-odometer `odom -> base_link` TF and fusion `map -> odom` TF output are
disabled in this launch. Its launch configuration assigns the direct
`map -> base_link` TF to the adapter. Runtime validation requires exactly one
adapter `/tf` endpoint and rejects any competing dynamic `base_link` parent.
The container, standalone, and
standalone-with-NMEA launches instead enable gyro-odometer publication of
`odom -> base_link`; the evaluation-oriented `lidar_imu_only` launch does not
enable it by default.

The Docker run launches the real Autoware 1.9.0
`pose_instability_detector`, which compares kinematic state with the separately
published twist-with-covariance, and `localization_error_monitor`, which checks
the kinematic-state covariance ellipse. Pose and twist originate in the same
fused odometry, so the comparison has a common-mode limitation and is not an
independent accuracy check. Neither monitor detects input dropout.

Treat diagnostic levels from a real bag as characterization, not as an
acceptance gate, unless covariance has been independently calibrated to the
monitor assumptions. The current default planar position variance of
`0.25 m^2` gives `sigma = 0.5 m` and a default three-sigma ellipse of `1.5 m`,
which already exceeds the Autoware 1.9.0 lateral error threshold of `0.30 m`.

## 5. Inspect recorded outputs

By default the Docker runner records:

```text
/clock
/tf
/tf_static
/diagnostics
/localization/gyro_lidar_odom
/localization/ekf_odom
/localization/kinematic_state
/localization/pose_estimator/pose_with_covariance
/localization/twist_with_covariance
/localization/acceleration
/reference/tf
/reference/tf_static
/reference/diagnostics
/reference/localization/...
```

The replay helper moves known recorded localization outputs to `/reference/...`,
so metric code can consume both reference and newly computed results in one
output bag. Confirm reference topic and frame semantics before treating any
recorded estimator result as ground truth. Published measurements and
limitations are kept in the [evaluation pages](evaluation/README.md), rather
than duplicated in this operating guide.

For scan-to-scan and scan-to-submap runs compare at least:

- relative pose error and endpoint/outage error;
- yaw error after turns and in corridors;
- scan-to-scan registration rejection and accepted-snapshot non-intrusion;
- external matcher acceptance, commit, and rebuild counts;
- GNSS recovery convergence, peak correction rate, and post-return error;
- CPU, memory, callback latency, and real-time factor.

## 6. What this does not validate

This configuration does not launch a point-cloud map loader, Lanelet2 map,
Autoware NDT localization, perception, planning, or control. It therefore tests:

- compatibility with Autoware localization topics and single-owner TF;
- deterministic normal/fault/recovery responses from the two pinned Autoware
  localization monitors;
- behavior during rosbag replay with simulated time;
- coexistence with selected vehicle/sensor descriptions;
- estimator accuracy and timing on the same bag.

It does not prove closed-loop driving readiness. Enabling planning/control later
requires the relevant vector map, routing context, localization initialization
status/API integration, vehicle interface, and independent safety validation.
The two included monitors also do not establish input availability or
independent localization correctness: they do not detect dropout, and the
pose-instability detector receives pose and twist derived from the same source.
