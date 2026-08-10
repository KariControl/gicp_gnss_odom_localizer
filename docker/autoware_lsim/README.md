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
  --bag /absolute/path/to/bag \
  --points /sensing/lidar/top/pointcloud_raw \
  --imu /sensing/imu/tamagawa/imu_raw
```

The wrapper builds the overlay image, starts the localization-only Autoware
launch, replays the bag with conflicting recorded localization topics isolated
under `/reference`, publishes a zero map anchor after the first LiDAR odometry
sample when GNSS is disabled, records evaluation outputs, and shuts everything
down when playback ends.

Results are written below `docker_output/` unless `--output` is supplied.

## Included Hesai ROSBAG2/3

The `hesai-rosbag23` profile is required for the repository's
`rosbag/output_pointcloud2` and `rosbag/output_pointcloud3`. It supplies their
three calibrated sensor transforms, exact source topics, Hesai XT parameters,
GNSS, and a shared site-local origin. Do not combine it with
`--launch-vehicle` or `--already-deskewed`.

```bash
ROS_DOMAIN_ID=82 ./script/run_autoware_lsim_docker.sh \
  --bag "$PWD/rosbag/output_pointcloud2" \
  --profile hesai-rosbag23 \
  --run-name rosbag2_lsim

ROS_DOMAIN_ID=83 ./script/run_autoware_lsim_docker.sh \
  --bag "$PWD/rosbag/output_pointcloud3" \
  --profile hesai-rosbag23 \
  --run-name rosbag3_lsim \
  --no-build
```

The profile replays only PointCloud2, IMU, and NMEA, generates `/clock` at
100 Hz, waits for recorder discovery, and records estimator diagnostics and
the Autoware localization interface. The recorded bag is acceptance-checked
automatically and the report is saved as `validation.log`. See the
[detailed Japanese runbook](../../docs/autoware_lsim_rosbag2_rosbag3.md) for
preflight checks, acceptance criteria, output analysis, and limitations.

## Scan-to-submap

```bash
./script/run_autoware_lsim_docker.sh \
  --bag /absolute/path/to/bag \
  --points /sensing/lidar/top/pointcloud_raw \
  --imu /sensing/imu/tamagawa/imu_raw \
  --tracking-mode scan_to_submap
```

## GNSS

```bash
./script/run_autoware_lsim_docker.sh \
  --bag /absolute/path/to/bag \
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

The wrapper adds `compose.rviz.yaml`, mounts the X11 socket, and sets
`LIBGL_ALWAYS_SOFTWARE=1`. A localization-specific RViz profile displays the
deskewed point cloud, Autoware kinematic-state trail, TF tree, and `base_link`
axes in the `map` frame. Its point-cloud subscription explicitly uses
best-effort/volatile SensorDataQoS. `/rviz2` is a required node at startup and replay
completion, so a display or OpenGL failure makes the run fail instead of
silently passing. Headless evaluation is preferred for timing and CPU
measurements.

## Useful controls

```text
--already-deskewed      bypass internal point-cloud deskew
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
