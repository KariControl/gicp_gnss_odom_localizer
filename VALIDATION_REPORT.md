# gicp_gnss_odom_localizer 0.3.0-rc6 Validation Report

- Validation date: 2026-08-08 (JST)
- Source baseline: `gicp_gnss_odom_localizer-0.3.0-rc5`
- Baseline snapshot: `4aba944861939d2af4637856cdf5222625160691`
- Intended ROS target: ROS 2 Jazzy / Ubuntu 24.04
- Intended Autoware target: Autoware 1.9.0 localization-only Logging Simulation

## Result

RC6 performs the requested package-identity cleanup:

```text
pure_nmea_gga_conversion   -> pure_nmea_gnss_conversion
pure_snow_intensity_filter / pure_intensity_filter -> removed
```

The NMEA source directory, `package.xml` identity, CMake project name, package
dependencies, launch package references, scripts, CI package lists, repository
checks, and documentation were updated. The unused optional intensity-filter
package and its bringup branches were removed.

This is intentionally an **estimator-neutral runtime migration**. The estimator
algorithms and numerical configuration are unchanged from RC5.

A direct byte comparison confirms that the NMEA C++ source, headers, tests,
parameter YAML, and non-launch configuration files are identical to RC5 after
accounting only for the moved package directory. The intensity-filter source is
intentionally absent.

## Retained runtime and source interfaces

To avoid unrelated downstream breakage, RC6 retains:

- NMEA executable `pure_nmea_gga_conversion`;
- NMEA node/diagnostic name `nmea_gga_conversion`;
- NMEA C++ include root and helper namespace `pure_nmea_gga_conversion`;
- all estimator topic names, frame names, launch argument names, YAML node names and
  parameter keys;
- custom message type `pure_gnss_msgs/msg/GnssFusionInput`.

Downstream NMEA package dependencies, launch package names, and `ros2 pkg prefix`
references must use the new package identity. References to the retired
intensity-filter package must be removed. A clean build is required.

## Process model confirmed from launch source

`odometry_container.launch.py` loads the high-bandwidth point-cloud pipeline
into one multithreaded ROS 2 component-container process:

```text
component_container_mt / pure_odometry_container
  - imu_undistorter                 (when enabled)
  - gyro_odometer
```

The NMEA converter, GNSS map-to-odom fusion, and diagnostic aggregator are
ordinary `Node` actions and run as separate Linux processes. The Autoware
localization adapter is also a separate process. Under RC5/RC6 Docker LSim,
these processes share one Docker container, but they are not one Linux process.

`odometry_standalone.launch.py` runs every node as a separate process. The
composed launch currently does not set `use_intra_process_comms=true`; sharing a
component container therefore does not by itself enable explicit ROS 2
intra-process/zero-copy transport.

## Checks executed successfully

```text
$ python3 tools/check_repository.py
Repository checks PASS

$ ./tools/run_reference_tests.sh
PASS test_imu_yaw_integrator
PASS test_trajectory_heading_quality
PASS test_gnss_recovery_controller
PASS test_yaw_rate_integrator
PASS test_se2_fixed_lag_smoother
PASS test_tracking_mode
PASS test_observability_policy
PASS test_acceleration_estimator
Reference tests PASS

$ CXX=clang++ ./tools/run_reference_tests.sh
not run in this environment (`clang++` is unavailable)

$ python3 tools/reference_checks.py
PASS reference_checks

$ python3 tools/check_launch_construction.py
PASS launch construction odometry_standalone.launch.py
PASS launch construction odometry_container.launch.py
PASS launch construction autoware_lsim_localization.launch.py

$ python3 tools/check_docker_configuration.py
Docker configuration checks PASS

$ git diff --check
PASS
```

Repository checks now assert the current package directory/package.xml identities
and also assert that the retained NMEA executable and include-root interfaces
have not been accidentally renamed.

## Release-artifact verification

- The RC5-to-RC6 patch passes `git apply --check` on a clean RC5 tree.
- Applying that patch produces a tree identical to the RC6 release tree.
- The finalized ZIP was extracted into a clean directory and repository checks,
  all seven ROS-independent tests with GCC and Clang, numerical reference checks,
  launch construction, Docker configuration checks, and shell syntax checks were
  rerun successfully.
- The ZIP contains no `.git`, `build/`, `install/`, `log/`, Python cache, object,
  static-library, or shared-library artifacts.

## Validation still required

The available execution environment does not contain ROS 2, colcon, PCL/Eigen
development packages, Docker daemon access, Autoware, or target bags. The
following remain release gates:

```bash
rm -rf build install log
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Then verify:

1. `ros2 pkg prefix pure_nmea_gnss_conversion` resolves.
2. `ros2 run pure_nmea_gnss_conversion pure_nmea_gga_conversion` starts.
3. Standalone, composed, and Docker Autoware LSim launches complete on a real
   bag without package-discovery or install-path errors.
