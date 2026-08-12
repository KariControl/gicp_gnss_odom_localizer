# Rosbag and Autoware Logging-Simulation Evaluation

> Canonical result tables and publication status are indexed at
> [docs/evaluation](evaluation/README.md). This page defines the reusable
> execution workflow; dated result summaries live under that evaluation index.

This workflow deliberately uses the same estimator configuration in two stages:

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

## 2. Stage A: standalone rosbag test

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

### 2.2 Add the isolated precision overlay

Keep the odometer in `scan_to_scan`, load the accepted-scan snapshot override,
and launch the precision processes separately:

```bash
ros2 launch pure_odometry_bringup odometry_standalone.launch.py \
  use_sim_time:=true \
  use_gnss:=false \
  use_map_odom_fusion:=false \
  odom_override_param:=$(ros2 pkg prefix pure_precision_bringup)/share/pure_precision_bringup/config/submap_snapshot_override.yaml

ros2 launch pure_precision_bringup precision_overlay.launch.py \
  use_sim_time:=true
```

Use the identical bag, replay rate, input remaps, TFs, baseline odometer YAML,
and metric code for both runs. Compare at least:

- relative pose error and endpoint error;
- yaw error, especially after turns and long corridors;
- registration rejection count;
- accepted-snapshot key coverage and baseline non-intrusion;
- external matcher acceptance, robust commits, and recovery rebuilds;
- real-time factor and callback latency;
- CPU and memory use.

The external matcher publishes only separate precision topics and no TF. A
submap result is not automatically better; retain baseline output until
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

## 3. Stage B: replace Autoware localization in Docker

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
2. launches localization-only Autoware plus this estimator and adapter;
3. waits until `/localization/kinematic_state` has a publisher;
4. starts output recording;
5. replays the input bag with recorded localization moved to `/reference/...`;
6. when GNSS is disabled, waits for the first LiDAR odometry sample and
   publishes the configured `/initialpose` automatically;
7. stops the recorder and launch cleanly after playback finishes.

The default result root is:

```text
docker_output/<bag>_<tracking-mode>_<date>/
```

It contains launch/replay/record logs, the resolved run settings, input bag
information, and a `localization_output` rosbag.

### 3.2 Compare baseline and isolated precision

Run the same bag twice and change only the localization mode. The production
odometer remains scan-to-scan in both runs; precision mode enables its output-only
snapshot bridge and starts the external matcher/global overlay:

```bash
./script/run_autoware_lsim_docker.sh \
  --bag <bag_path> \
  --points /sensing/lidar/top/pointcloud_raw \
  --imu /sensing/imu/tamagawa/imu_raw \
  --tracking-mode scan_to_scan \
  --run-name lsim_baseline
```

```bash
./script/run_autoware_lsim_docker.sh \
  --bag <bag_path> \
  --points /sensing/lidar/top/pointcloud_raw \
  --imu /sensing/imu/tamagawa/imu_raw \
  --tracking-mode scan_to_submap \
  --run-name lsim_precision
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
  --run-name lsim_gnss
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

### 3.6 RViz without a discrete GPU

The default Docker run is headless. For occasional visualization, use CPU
software rendering:

```bash
./script/run_autoware_lsim_docker.sh ... --rviz
```

This adds `compose.rviz.yaml`, forwards X11, and sets
`LIBGL_ALWAYS_SOFTWARE=1`. Keep RViz disabled when measuring estimator timing or
CPU utilization.

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
```

The Compose service uses host networking and host IPC. It is privileged because
the official Autoware entrypoint enables loopback multicast and applies DDS
network tuning. Use this profile on a trusted local development machine.

## 4. Autoware-facing outputs

`pure_autoware_localization_adapter` converts the fused odometry into:

```text
/localization/kinematic_state                         nav_msgs/msg/Odometry
/localization/pose_estimator/pose_with_covariance     geometry_msgs/msg/PoseWithCovarianceStamped
/localization/acceleration                            geometry_msgs/msg/AccelWithCovarianceStamped
map -> base_link                                      TF
```

The acceleration is a bounded, low-pass derivative of the fused twist. It is
provided for interface evaluation, not as a calibrated inertial acceleration
measurement. The adapter resets the derivative on the first sample, time
reversal, or a large timestamp gap and publishes a high covariance for that
reset sample.

The fusion node's own TF output is disabled in this launch so that there is no
second `map -> odom` chain competing with the adapter's direct
`map -> base_link` TF.

## 5. Recorded results

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
/localization/acceleration
/reference/tf
/reference/tf_static
/reference/diagnostics
/reference/localization/...
```

The replay helper moves known recorded localization outputs to `/reference/...`,
so metric code can consume both reference and newly computed results in one
output bag. Confirm reference topic and frame semantics before treating any
recorded estimator result as ground truth.

For baseline and precision runs compare at least:

- relative pose error and endpoint/outage error;
- yaw error after turns and in corridors;
- baseline registration rejection and accepted-snapshot non-intrusion;
- external matcher acceptance, commit, and rebuild counts;
- GNSS recovery convergence, peak correction rate, and post-return error;
- CPU, memory, callback latency, and real-time factor.

## 6. What this does not validate

This configuration does not launch a point-cloud map loader, Lanelet2 map,
Autoware NDT localization, perception, planning, or control. It therefore tests:

- compatibility with Autoware localization topics and TF;
- behavior under Autoware logging-simulation time;
- coexistence with selected vehicle/sensor descriptions;
- estimator accuracy and timing on the same bag.

It does not prove closed-loop driving readiness. Enabling planning/control later
requires the relevant vector map, routing context, localization initialization
status/API integration, vehicle interface, and independent safety validation.
