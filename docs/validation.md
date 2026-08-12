# Validation

This page defines the maintained validation scope for the repository. It
replaces the obsolete root-level RC6 migration report, which described an older
package-renaming change rather than the current localization implementation.

Dated terminal logs and complete generated runs are not committed. Public
evaluation evidence is curated under [evaluation/assets](evaluation/assets/README.md),
while the [evaluation index](evaluation/README.md) records remaining rerun and
publication gates.

## Validation layers

| Layer | Purpose | Required evidence |
|---|---|---|
| Repository checks | Detect stale package names, invalid references, and unsupported configuration combinations | `tools/check_repository.py` passes |
| ROS-independent reference tests | Exercise numerical helpers and state-machine behavior without a ROS installation | `tools/run_reference_tests.sh` passes |
| Numerical reference checks | Check deterministic mathematical and policy invariants | `tools/reference_checks.py` passes |
| Launch construction | Verify launch files can be constructed with supported arguments | `tools/check_launch_construction.py` passes |
| Docker configuration | Validate Compose files, wrapper contracts, and pinned Autoware integration | `tools/check_docker_configuration.py` passes |
| ROS build and package tests | Compile the workspace and run package-level unit tests | `colcon build`, `colcon test`, and `colcon test-result` pass |
| Recording-level evaluation | Check accuracy, timestamps, non-intrusion, queue behavior, GNSS outage/recovery, and runtime | Dataset-specific gates documented under `docs/evaluation/` pass |
| Documentation publication | Ensure published pages are English, use stable assets, and do not depend on private inputs or generated output trees | Markdown/link and publication checks pass |

## Fast source checks

Run these from the repository root:

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

## ROS 2 build and unit tests

ROS 2 Jazzy on Ubuntu 24.04 is the primary target.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

After changing installed YAML or launch files in a symlink build, remove only
stale generated install links associated with the affected package or rebuild
that package cleanly. Do not delete unrelated user artifacts.

## Recording-level gates

Source rosbags are private and are not distributed on GitHub. A release claim
must retain private provenance locally while publishing only compact metrics,
plots, and hashes that do not expose host-specific paths.

At minimum, each supported dataset must verify:

- the intended point-cloud, IMU, GNSS, and TF policy;
- resolved parameter values and configuration hashes;
- monotonic physical timestamps and sufficient reference coverage;
- finite poses and covariances with correct frames;
- registration acceptance, rejection, reset, queue-drop, and deadline behavior;
- baseline non-intrusion when the isolated precision overlay is enabled;
- declared trajectory alignment and consistent A/B transforms;
- GNSS initialization, outage, reacquisition, and bounded recovery when used;
- 1.0x playback completion;
- CPU/RSS only when sampled explicitly under the controlled method in
  [evaluation/methodology.md](evaluation/methodology.md#runtime-latency-cpu-and-memory).

Matcher processing time and end-to-end latency must never be presented as CPU
utilization.

## Current publication-specific gates

- The historical Hesai Course 1 and Course 2 native accuracy results must be
  repeated with the intended site-local NMEA-origin override before becoming
  final acceptance evidence.
- The GNSS plot alignment must remain labeled frozen calibration window unless
  the metrics and plots are regenerated with exact-initial-pose alignment.
- The recording-specific MID-360 fixed gyro bias must be generalized and tested
  on additional recordings before being treated as production calibration.
- Velodyne full-recording generation recovery remains a failed robustness gate,
  even though the valid-prefix scan-to-submap metrics improved.
- Baseline/precision CPU and RSS measurements are still unavailable.
- Autoware headless and RViz integration checks passed on the recorded evidence,
  but an RViz video has not yet been published.

## Documentation checks

Before publishing a commit, verify that tracked project documentation:

- contains no Japanese explanatory text;
- uses descriptive sensor-and-course dataset labels rather than internal bag
  directory names;
- contains no absolute host paths;
- does not link to raw test runs, Docker output directories, or private rosbags;
- resolves every relative link and image against a clean checkout;
- includes only curated assets and small machine-readable summaries.
