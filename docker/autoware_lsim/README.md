# Autoware Logging Simulation Docker profile

This profile runs the localization-only Autoware integration and rosbag replay
inside one CPU-only Docker container. The host needs Docker Engine and the
Docker Compose v2 plugin, but it does not need ROS 2 or Autoware installed.

The image is built on the pinned no-CUDA development image:

```text
ghcr.io/autowarefoundation/autoware:universe-devel-jazzy-1.9.0
```

Override it with `--autoware-image` only when intentionally testing another
Autoware release. Keeping the release pin makes launch arguments and runtime
interfaces reproducible.

## One-command run

From the repository root:

```bash
./script/run_autoware_lsim_docker.sh \
  --bag <bag_path> \
  --points /sensing/lidar/top/pointcloud_raw \
  --imu /sensing/imu/tamagawa/imu_raw
```

The wrapper builds the overlay image, starts the localization-only Autoware
launch, replays the bag with conflicting recorded localization topics isolated
under `/reference`, publishes a zero map anchor after the first LiDAR odometry
sample when GNSS is disabled, records evaluation outputs, and shuts everything
down when playback ends.

Results are written below `docker_output/` unless `--output` is supplied.

## Private Hesai evaluation profile

The `hesai-rosbag23` profile is used for private Hesai 32-Line + IMU + RTK GNSS
evaluation recordings. The recordings are not distributed on GitHub. The
profile supplies three calibrated sensor transforms, the source topics, Hesai
XT parameters, and GNSS. NMEA conversion uses the packaged runtime projection,
which is provenance-checked against the packaged `map_projector_info.yaml`; the
profile applies no evaluation-origin override. Do not combine it with
`--launch-vehicle` or `--already-deskewed`.

```bash
ROS_DOMAIN_ID=83 ./script/run_autoware_lsim_docker.sh \
  --bag <hesai_course_2_bag> \
  --profile hesai-rosbag23 \
  --run-name hesai_course_2_lsim \
  --no-build
```

The profile replays only PointCloud2, IMU, and NMEA, generates `/clock` at
100 Hz, waits for recorder discovery, and records estimator diagnostics and
the Autoware localization interface. The recorded bag is acceptance-checked
automatically and the report is saved as `validation.log`. See the
[reusable evaluation workflow](../../docs/rosbag_and_autoware_lsim_evaluation.md)
for preflight checks and execution details, and the
[curated evaluation results](../../docs/evaluation/autoware_lsim.md) for measured
outcomes and limitations.

## Isolated precision overlay

The Docker wrapper temporarily retains `scan_to_submap` as its compatibility UI
token. It does not enable an internal odometer tracking path: the odometer stays
on `scan_to_scan`, publishes accepted-scan snapshots, and the separate precision
overlay performs rolling-submap matching.

```bash
./script/run_autoware_lsim_docker.sh \
  --bag <bag_path> \
  --points /sensing/lidar/top/pointcloud_raw \
  --imu /sensing/imu/tamagawa/imu_raw \
  --tracking-mode scan_to_submap
```

## GNSS

```bash
./script/run_autoware_lsim_docker.sh \
  --bag <bag_path> \
  --points /sensing/lidar/top/pointcloud_raw \
  --imu /sensing/imu/tamagawa/imu_raw \
  --nmea /sensing/gnss/nmea_sentence \
  --fix-velocity /sensing/gnss/fix_velocity
```

Providing `--nmea` enables the GNSS frontend. Automatic `/initialpose` is used
only when GNSS is disabled.

## TF policy

The default is:

```text
--tf-policy isolate-dynamic
--launch-vehicle disabled
```

This keeps the bag's `/tf_static` sensor extrinsics and moves its dynamic `/tf`
to `/reference/tf`. Use `--tf-policy isolate-all --launch-vehicle` only when the
selected Autoware vehicle and sensor models exactly match the recorded sensor
installation. The `hesai-rosbag23` profile is the other supported
`isolate-all` case because it publishes its own calibrated static transforms.

## RViz without a discrete GPU

RViz is disabled by default. Enable CPU software rendering with:

```bash
./script/run_autoware_lsim_docker.sh ... --rviz
```

To add the Autoware sample Lexus body to the localization view, use the
presentation visualization option:

```bash
./script/run_autoware_lsim_docker.sh ... \
  --rviz \
  --rviz-sample-vehicle
```

This option loads only the single-link body from the Autoware image's
`sample_vehicle_description` package. It does not launch
`sample_sensor_kit`, publish vehicle sensor transforms, or change estimator
inputs. The model follows the existing `map -> base_link` estimate through a
dedicated transient-local robot-description topic. Its TF outputs are isolated
from the localization TF tree, and the runner requires both the model publisher
and the RViz subscription to remain alive through replay completion.

The same option starts a visualization-only node and enables the following live
RViz evidence:

- a continuous line trajectory generated from
  `/localization/kinematic_state`;
- a dedicated `Autoware Localization Status` entry in the RViz Displays panel
  with current XY position, yaw, speed, output rate, `map -> base_link`
  availability, registration state, and
  `Autoware Localization Interface: ACTIVE` status;
- a diagnostic-driven GNSS property whose indicator is green for `TRACKING`,
  yellow for `OUTAGE` or `REACQUIRING`, and blue for `RECOVERING`;
- a bounded 2-sigma XY-only position-covariance ellipse generated from the
  Autoware kinematic state and drawn at the same visual ground offset.

The custom Display derives its properties from live kinematic-state, TF, and
diagnostic inputs; these are not fixed presentation strings. Missing or stale
input is shown as unavailable rather than as a healthy state. The standard RViz
odometry covariance display stays disabled because the planar interface
intentionally gives Z a very large variance; rendering it would create a
misleading 3D ellipsoid. At replay completion, the runner checks the
visualization node, both generated topics, the RViz visualization subscriptions,
and the custom Display's direct `/diagnostics` subscription. It then saves one
bounded trajectory and one covariance-marker snapshot as evidence. The
continuously growing presentation topics are deliberately not added to the
output rosbag.

The existing `/localization/visualization/status_markers` topic name is retained
for configuration compatibility, but its payload now contains only the spatial
XY covariance marker; status text is not drawn in the 3D scene. The custom
Display is implemented in `pure_odometry_bringup` using standard ROS 2, RViz,
pluginlib, and Qt dependencies. It introduces no compile-time Autoware package
dependency and is loaded only by the opt-in presentation profile.

For this Hesai profile, `base_link` is deliberately colocated with the LiDAR
frame. Robust ground-plane fits evaluated at the Lexus wheel-contact locations
gave median Z values of `-1.6641 m` for ROSBAG2 and `-1.6565 m` for ROSBAG3. The
mesh's lowest point is approximately `+0.00394 m` relative to its body frame, so
the visual-only Lexus body uses a rounded `-1.66 m` default Z offset:

```bash
./script/run_autoware_lsim_docker.sh ... \
  --rviz \
  --rviz-sample-vehicle \
  --rviz-sample-vehicle-z-offset -1.66
```

Adjust the value after measuring the ground plane for another recording. The
accepted range is `-3.0` to `1.0 m`. This offset moves only the rendered mesh; it
does not alter `base_link`, calibrated sensor TFs, the point cloud, or estimator
output. It is an RViz ground-alignment estimate, not a vehicle dimension or
sensor-height calibration.

The Lexus body is illustrative only: it is not the recorded vehicle geometry,
`base_link` convention, or sensor calibration. The recording is believed to
have been captured with a roof-mounted LiDAR on a Yaris, so neither the Lexus
shape nor the `-1.66 m` visual alignment should be presented as a measured
vehicle model. Do not combine
`--rviz-sample-vehicle` with `--launch-vehicle`. The mesh remains in the pinned
Autoware image and is not copied into this repository; the upstream
`sample_vehicle_description` package is Apache 2.0 licensed.

The wrapper adds `compose.rviz.yaml`, mounts the X11 socket, and sets
`LIBGL_ALWAYS_SOFTWARE=1`. A localization-specific RViz profile displays the
deskewed point cloud, generated line trajectory, an XY-only covariance ellipse,
kinematic-state arrows, the dedicated localization-status entry in the Displays
panel, and `base_link` axes in the `map` frame. Its point-cloud subscription
explicitly uses best-effort/volatile SensorDataQoS. `/rviz2` is a required node
at startup and replay completion, so a display or OpenGL failure makes the run
fail instead of silently passing. Headless evaluation is preferred for timing
and CPU measurements.

## Useful controls

```text
--already-deskewed      bypass internal point-cloud deskew
--rviz-sample-vehicle   enable the body, trajectory, Displays status, and XY ellipse
--rviz-sample-vehicle-z-offset <m>
                        override only the rendered body/ellipse height
--no-record             do not create an output rosbag
--no-build              reuse the existing local image
--pull-base             refresh the pinned Autoware image before building
--build-jobs 1          reduce build memory pressure
--shell                 open a shell in the prepared image
--dry-run               print the resolved Docker commands without Docker
```

The Compose service deliberately uses host networking and host IPC so ROS 2 DDS
and shared-memory transports behave like a native single-host LSim deployment.
It is marked privileged because the official Autoware entrypoint configures
loopback multicast and DDS-related kernel settings. This profile is for a local
development machine, not an untrusted multi-tenant host.
