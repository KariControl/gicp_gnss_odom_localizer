# Validation

This page is the maintained checklist for validating source changes, ROS 2
builds, the public synthetic rosbag, recording-level evaluation, and published
documentation. Evaluation results and dataset-specific gate counts belong in
the [evaluation pages](evaluation/README.md) and their machine-readable metrics;
they are not duplicated here.

Dated terminal logs, private recordings, and complete generated runs are not
committed. Only curated plots, compact metrics, hashes, and reproducible public
inputs are stored in the repository.

## Validation layers

| Layer | Purpose | Maintained entry point |
|---|---|---|
| Repository checks | Detect stale package names, invalid references, and unsupported configuration combinations | `tools/check_repository.py` |
| ROS-independent reference tests | Exercise numerical helpers and state-machine behavior without ROS | `tools/run_reference_tests.sh` |
| Numerical reference checks | Check deterministic mathematical and policy invariants | `tools/reference_checks.py` |
| Launch construction | Construct the supported launch files and arguments | `tools/check_launch_construction.py` |
| Docker configuration | Validate Compose files, wrapper contracts, and pinned Autoware integration | `tools/check_docker_configuration.py` |
| ROS build and package tests | Compile the workspace and run package-level tests | `colcon build`, `colcon test`, and `colcon test-result` |
| Runtime TF ownership matrix | Launch supported profiles in isolated ROS domains and check the endpoint multiset, owner parameters, and emitted edge set | `pure_odometry_bringup/test_tf_ownership_profiles` through `colcon test` |
| Public synthetic rosbag | Validate the committed PointCloud2/IMU/TF input and exercise the local-odometry path | `tools/check_synthetic_output_pointcloud2.py` and `script/run_synthetic_lidar_imu_smoke.sh` |
| Autoware interface contract | Exercise adapter outputs, TF ownership, and real Autoware 1.9.0 monitor normal/fault/recovery behavior | `script/run_autoware_localization_contract_docker.sh` |
| Recording-level evaluation | Validate accuracy, timestamps, non-intrusion, GNSS behavior, and runtime on controlled recordings | [Evaluation methodology](evaluation/methodology.md) |
| Documentation publication | Check links, stable assets, privacy, and publication boundaries | Repository and asset checks plus the checklist below |

## Fast source checks

Run these commands from the repository root:

```bash
python3 tools/check_repository.py
./tools/run_reference_tests.sh
python3 tools/reference_checks.py
python3 tools/check_launch_construction.py
python3 tools/check_docker_configuration.py
python3 tools/evaluation/curate_publication_assets.py --check
git diff --check
```

These checks do not replace a ROS build or recording replay.

## Autoware localization-interface contract

Run the deterministic contract using the Autoware 1.9.0 CPU-only image tag:

```bash
./script/run_autoware_localization_contract_docker.sh
```

It verifies kinematic-state, pose, twist-with-covariance, acceleration and
direct `map -> base_link` TF behavior, requires exactly one adapter `/tf`
publisher endpoint, checks that no competing dynamic `base_link` parent
appears, and checks normal/error/recovery transitions from
the real Autoware `pose_instability_detector` and
`localization_error_monitor` nodes.
This gate validates the message/TF/diagnostic contract only. It does not run
planning/control, measure localization accuracy, test topic dropout, or make
the pose-instability detector independent: its pose and twist inputs are both
derived from the fused odometry.

The default evidence directory is
`docker_output/autoware_localization_contract_<UTC timestamp>/` and contains
`result.json`, `tf_ownership.json`, launch/probe/ownership logs, and runner
status. The result embeds the ownership evidence and records the resolved image
ID and monitor package versions; CI retains the directory as an artifact for 30
days. The image release tag is version-selected, while its resolved image ID
supplies the immutable identity for a particular run.
The same CI job then reuses the built image for the committed public synthetic
bag and production localizer launch. That replay covers simulated time,
automatic initialization, TF endpoint ownership both before and after replay,
TF isolation, adapter alignment, and monitor availability. It retains
`tf_ownership.json` and `tf_ownership_completion.json`; it is still a functional
integration test without independent ground truth, not an accuracy result.

For real bags, record and report monitor-level distributions as
characterization rather than pass/fail criteria until covariance is calibrated
against an independent reference. In particular, a default planar variance of
`0.25 m^2` becomes a `1.5 m` three-sigma ellipse, which exceeds the pinned
monitor's `0.30 m` lateral error threshold.

The analyzer is fail-closed against the current Stage A output schema. A bag
recorded by an earlier revision lacks the separate twist stream, monitor
diagnostics, and renamed adapter identity and therefore fails the current
schema check. Replay the source bag with the current runner before comparing
integration results; the failure alone does not invalidate previously reported
trajectory metrics.

## ROS 2 build and package tests

ROS 2 Jazzy on Ubuntu 24.04 is the primary target.

The `ROS 2 Jazzy CI` workflow runs on pushes and pull requests targeting
`main` or `develop`, and can also be started manually. Each job uses a fresh
Ubuntu 24.04 runner, recursively checks out the pinned `small_gicp` submodule,
installs dependencies through rosdep, verifies that no `build`, `install`, or
`log` tree exists, performs a Release build, and runs every package test
registered with colcon. It intentionally does not restore build products from
a cache.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

`pure_odometry_bringup` includes a runtime TF ownership matrix. It covers
container and standalone local/fused profiles, the main container GNSS path and
the NMEA-wrapper profile,
`lidar_imu_only` with TF disabled and enabled, both composable deskew branches,
and the precision overlay. For every profile it checks the exact `/tf` endpoint
multiset, each owner's effective `publish_tf` and frame parameters, and the
emitted dynamic edge set. A negative test starts two publishers configured for
the same `map -> base_link` edge, verifies that both processes and endpoints are
present, and requires the ownership probe to reject that graph.

After changing installed YAML or launch files in a symlink build, rebuild the
affected package. Do not remove unrelated build products or user artifacts.

## Public synthetic rosbag

The committed [procedural fixture](../data/README.md) provides a reproducible
PointCloud2/IMU/TF input for functional validation. It is artificial smoke-test
data, not a real-world accuracy benchmark.

After sourcing ROS 2 Jazzy and the built workspace, validate the fixture's
hashes, schema, timestamps, transforms, metadata, privacy constraints, and
semantic stream:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/check_synthetic_output_pointcloud2.py
```

Exercise launch, replay, output recording, and runtime validation with:

```bash
./script/run_synthetic_lidar_imu_smoke.sh
```

The fixture contract, maintained runtime gates, manual rosbag replay commands,
and regeneration procedure are documented in [data/README.md](../data/README.md).

## Recording-level requirements

Private source rosbags are not distributed on GitHub. A release claim must keep
its reproducible provenance locally while publishing only non-identifying,
reviewable summaries.

For each supported recording, verify at minimum:

- the intended point-cloud, IMU, GNSS, and TF inputs and policies;
- resolved parameter values, configuration hashes, and source revision;
- monotonic physical timestamps and sufficient reference coverage;
- finite poses and covariances with the declared frames;
- registration acceptance, rejection, reset, queue-drop, and deadline behavior;
- scan-to-scan non-intrusion when the isolated scan-to-submap path is enabled;
- declared trajectory alignment and the same A/B transform where required;
- GNSS initialization, unavailability, return, and bounded localization recovery
  when GNSS is used;
- complete 1.0x playback without silent input loss; and
- CPU and RSS only when sampled under the controlled procedure in
  [evaluation/methodology.md](evaluation/methodology.md#runtime-latency-cpu-and-memory).

Matcher processing time and end-to-end latency must not be presented as CPU
utilization. Known failures and unmeasured quantities remain part of the
published limitations rather than being omitted.

## Published evaluation evidence

Dataset-specific conditions, results, gate status, and limitations are
maintained at the following canonical locations:

| Evaluation target | Human-readable report | Machine-readable evidence |
|---|---|---|
| Velodyne 32-Line and Livox MID-360 LiDAR/IMU-only localization | [LiDAR/IMU-only evaluation](evaluation/lidar_imu.md) | [Velodyne metrics](evaluation/assets/velodyne_32line_external_imu/metrics.json), [MID-360 metrics](evaluation/assets/livox_mid360_internal_imu/metrics.json) |
| Hesai 32-Line LiDAR/IMU/GNSS localization | [LiDAR/IMU/GNSS evaluation](evaluation/lidar_imu_gnss.md) | [Hesai metrics](evaluation/assets/hesai_32line_imu_rtk_gnss_course_2/metrics.json) |
| Autoware localization-interface integration | [Autoware localization-interface evaluation](evaluation/autoware_lsim.md) | [Autoware metrics](evaluation/assets/autoware_lsim_hesai_course_2/metrics.json) |

The [evaluation index](evaluation/README.md) is the status overview. Asset
provenance and hashes are defined by the [asset policy](evaluation/assets/README.md)
and [`manifest.json`](evaluation/assets/manifest.json).

## Documentation and publication checklist

Before publishing a commit, verify that tracked project documentation:

- contains no Japanese explanatory text;
- uses `scan-to-scan` and `scan-to-submap` for user-facing mode names;
- uses descriptive sensor-and-course labels rather than internal bag directory
  names;
- contains no absolute host paths, capture epochs, or private identifiers;
- does not link to raw test runs, generated output directories, private
  recordings, or local-only files;
- resolves every relative link and image from a clean checkout;
- includes only curated assets and compact machine-readable summaries;
- distinguishes synthetic smoke testing from real-recording evaluation;
- states alignment, reference, sampling, and known limitations for every
  published metric; and
- labels visualization-only media separately from evidence-producing runs.

Run the fast source checks again after documentation or asset changes. Before a
release, also review the staged file list from a clean checkout so that logs,
raw recordings, and generated run directories are not accidentally published.
