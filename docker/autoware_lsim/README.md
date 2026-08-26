# Docker-Based Autoware Localization-Interface Evaluation

This profile runs the Stage A localization-interface integration and rosbag
replay inside one CPU-only Docker container. The host needs Docker Engine and
the Docker Compose v2 plugin, but it does not need ROS 2 or Autoware installed.

The retained `autoware_lsim` paths and command names are project-local
compatibility identifiers, not names of an official Autoware component or
workflow.

The image is built from the Autoware 1.9.0 no-CUDA development-image tag:

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
under `/reference`, waits for a positive `/clock` and a nonzero-stamped local
odometry sample before publishing a zero map anchor when GNSS is disabled,
records evaluation outputs, and shuts everything down when playback ends.

The Docker launch uses the real Autoware 1.9.0
`autoware_pose_instability_detector` and
`autoware_localization_error_monitor` packages. It also records the adapter's
`/localization/twist_with_covariance` output used by the pose-instability
detector.

Results are written below `docker_output/` unless `--output` is supplied.

## Deterministic interface and diagnostic contract

Run the version-selected contract before a recording-specific evaluation:

```bash
./script/run_autoware_localization_contract_docker.sh
```

The test launches the production adapter plus the two actual Autoware 1.9.0
monitor nodes, injects deterministic fused odometry, and verifies:

- presence and declared frames for every adapter output, preservation of the
  tested pose/twist/covariance fields, finite derived acceleration, and the
  tested TF stamp and pose fields;
- that `/localization_interface_adapter` owns exactly one runtime `/tf`
  publisher endpoint configured for `map -> base_link`, the emitted TF matches
  the input pose, and no competing dynamic `base_link` parent is observed;
- `pose_instability_detector` transitions `OK -> ERROR -> OK` for a pose jump;
- `localization_error_monitor` transitions `OK -> ERROR -> OK` for an injected
  covariance fault, including its expected diagnostic keys.

This is a synthetic contract test, not a localization-accuracy benchmark. It
does not launch Autoware planning/control or prove closed-loop readiness. The
pose and twist supplied to the instability detector share one fused-odometry
source, so common-mode estimator errors can remain invisible. Neither of these
two Autoware monitors detects topic dropout.

The exact diagnostic identities checked by the contract are
`localization: pose_instability_detector` and
`localization_error_monitor: ellipse_error_status`.

By default, evidence is retained under
`docker_output/autoware_localization_contract_<UTC timestamp>/`: `result.json`,
`tf_ownership.json`, `launch.log`, `probe.log`, `tf_ownership.log`, and
`runner_status.txt`. The result embeds the ownership evidence and records the
resolved local image ID and both monitor package versions. CI uploads the same
directory as an artifact for 30 days. The CI job then reuses that image to run
the committed public synthetic bag through the production localizer launch,
including simulated time, automatic initialization, TF endpoint ownership both
before and after replay, TF policy, adapter, and both monitors. The production
run retains `tf_ownership.json` and `tf_ownership_completion.json`.

The current analyzer expects the Stage A recording schema. Output bags created
before Stage A do not contain the separate twist output, the two monitor
diagnostics, or the renamed adapter identity, so rechecking such historical
bags with the current analyzer fails by design. Regenerate the output bag with
the current runner; do not interpret that schema failure as a change in the
historical trajectory metrics.

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
  --run-name hesai_course_2_autoware_interface \
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

The container, standalone, and standalone-with-NMEA launches enable the gyro
odometer's `odom -> base_link` TF; the evaluation-oriented `lidar_imu_only`
launch leaves it disabled by default. In this Autoware profile, gyro-odometer
and fusion TF publication are both disabled, and the launch assigns the direct
`map -> base_link` transform to the adapter. Before replay starts, the runner
queries the live ROS graph and effective node parameters: the adapter must own
the only `/tf` endpoint, while gyro odometer and fusion must both report
`publish_tf=false`.

## Interpreting Autoware monitor results

The deterministic normal/fault/recovery contract is a pass/fail gate. Monitor
levels observed during a real bag are characterization, not a localization
accuracy acceptance criterion: their thresholds require covariance semantics
and calibration that match Autoware's assumptions. For example, the current
default planar position variance of `0.25 m^2` has `sigma = 0.5 m`; with the
monitor's default scale of three, its `3 sigma` ellipse is `1.5 m`, already far
above the Autoware 1.9.0 lateral error threshold of `0.30 m`. Do not tune the
published covariance or monitor thresholds merely to make the diagnostic
green; establish an independent covariance-calibration method first.

## GUI diagnostics and RViz without a discrete GPU

GUI tools are disabled by default. To inspect the aggregated localization
diagnostics in a separate Robot Monitor window, use:

```bash
./script/run_autoware_lsim_docker.sh ... --rqt-robot-monitor
```

The monitor runs inside the container and subscribes to `/diagnostics_agg`.
The runner checks that the GUI process and its subscription remain available
through replay completion. It can be combined with `--rviz`. Either GUI option
requires a working host X11 `DISPLAY` and `xhost` command.

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

When either GUI is requested, the wrapper adds `compose.rviz.yaml`, mounts the
X11 socket, and sets `LIBGL_ALWAYS_SOFTWARE=1`. A localization-specific RViz
profile displays the deskewed point cloud, generated line trajectory, an XY-only
covariance ellipse, kinematic-state arrows, the dedicated localization-status
entry in the Displays panel, and `base_link` axes in the `map` frame. Its
point-cloud subscription explicitly uses best-effort/volatile SensorDataQoS.
`/rviz2` is a required node at startup and replay completion, so a display or
OpenGL failure makes the run fail instead of silently passing. Headless
evaluation is preferred for timing and CPU measurements.

## Useful controls

```text
--already-deskewed      bypass internal point-cloud deskew
--rqt-robot-monitor     show the /diagnostics_agg tree in Robot Monitor
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
and shared-memory transports behave like a single-host Autoware integration.
It is marked privileged because the official Autoware entrypoint configures
loopback multicast and DDS-related kernel settings. This profile is for a local
development machine, not an untrusted multi-tenant host.
