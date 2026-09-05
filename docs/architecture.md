# Architecture

## Coordinate frames

- `map`: globally anchored planar frame.
- `odom`: locally continuous dead-reckoning frame.
- `base_link`: vehicle state frame.
- LiDAR, IMU, and GNSS antenna frames: calibrated children of `base_link`.
- `odom_precision`: isolated local frame used only by the optional scan-to-submap
  branch.

The production transform topology is:

```text
map -> odom -> base_link -> lidar
                         -> imu
                         -> gnss/0
```

Before enabling GNSS, configure a map origin that belongs to the deployment
site or map projection and publish calibrated static transforms from
`base_link` to the LiDAR, IMU, and GNSS antenna frames. A recording-specific
origin or extrinsic must not be reused for another site or sensor installation
without validation.

The standard `odometry_container.launch.py` and
`odometry_standalone.launch.py` profiles enable
`pure_lidar_gyro_odometer` as the `odom -> base_link` publisher.
`pure_gnss_map_odom_fusion` estimates and optionally publishes `map -> odom`.
Each dynamic transform has one configured owner. The scan-to-submap branch
publishes no TF, so it cannot compete with the production transform tree.

## Data flow

1. `pure_imu_undistortion` validates point timing and IMU coverage, transforms
   angular velocity into `base_link`, integrates a relative trajectory, and
   re-expresses points at the configured scan reference time.
2. `pure_lidar_gyro_odometer` registers each accepted scan against the previous
   accepted scan, attaches corrected-IMU yaw and optional constraints, and solves
   a fixed-lag SE(2) graph.
3. `pure_nmea_gnss_conversion` projects GGA, assigns covariance/confidence,
   selects a valid heading source, and publishes a physical GNSS observation.
4. `pure_gnss_map_odom_fusion` synchronizes that observation against odometry
   and updates the `map -> odom` anchor.

In optional scan-to-submap mode, the odometer additionally publishes exact-key
accepted-scan snapshots without changing its scan-to-scan state. The separate
`pure_lidar_submap_matcher` produces a persistent full-SE(2)
`odom_precision <- odom` correction, and
`pure_precision_global_localizer` composes separate scan-to-submap local/global
outputs. Neither scan-to-submap node feeds the production odometer or fusion.

## Process and component-container model

`pure_odometry_bringup/launch/odometry_container.launch.py` composes the
high-bandwidth LiDAR path into one `rclcpp_components`
`component_container_mt` process:

```text
pure_odometry_container  (one Linux process, multithreaded executor)
  |-- imu_undistorter                  # when enabled
  `-- gyro_odometer
```

The NMEA conversion node, GNSS map-to-odom fusion node, and diagnostic
aggregator are ordinary `launch_ros.actions.Node` actions and therefore run as
separate Linux processes. In the Autoware integration launch,
`pure_localization_interface_adapter` is another separate process, and Autoware
starts its own processes/containers. All of them may run inside the same Docker
container, but a Docker container is not the same thing as a ROS 2 component
container.

`odometry_standalone.launch.py` uses one process per node, including the LiDAR
path. The composed launch does not currently set
`use_intra_process_comms=true`; the nodes share a process, but explicit ROS 2
intra-process message passing/zero-copy is not enabled by the launch file.

This split is intentional: composition reduces process and transport overhead
for large PointCloud2 data, while the lower-rate GNSS and integration nodes
retain a separate failure domain.

## Scan-to-scan and isolated scan-to-submap branch

```text
current scan --> scan-to-scan + fixed-lag SE(2) --> odom -> base_link
                    |
                    `-- accepted-scan snapshot (scan-to-submap mode only)
                                      |
                                      v
                        external rolling submap matcher
                                      |
                                      v
                  odom_precision <- odom correction
                                      |
                 +--------------------+--------------------+
                 v                                         v
     scan-to-submap local output        existing fusion health/anchor
                                                           |
                                                           v
                                          scan-to-submap global output
```

The production odometer always uses the previous accepted scan as its primary
target. `lidar_odom.tracking_mode` remains as a compatibility/diagnostic field,
but only `scan_to_scan` is accepted. The retired in-odometer submap path and
periodic local-map factor are not supported.

## Isolated scan-to-submap state

Every snapshot carries the odometer session, generation, accepted-pose sequence,
physical scan stamp, raw pose, and filtered cloud. The external matcher rejects
stale or retired streams, keeps rejected scans out of its map, robustly commits a
persistent SE(2) correction, and rebuilds its rolling target without resetting
the last committed correction. A matcher restart is explicitly rebased by the
consumer so it cannot silently jump the scan-to-submap trajectory.

The scan-to-submap global branch accepts an anchor only from a fresh, strictly
healthy existing-fusion state. It freezes the target and applied anchor through
GNSS outages and requires a stable multi-candidate recovery before bounded
updates resume. Its internal `map -> odom_precision` anchor is not broadcast.

## GNSS observation model

For a single antenna, the measurement is

`p_map_antenna = p_map_base + R(yaw_map_base) * r_base_antenna + noise`.

The message therefore carries the antenna position, base-yaw validity, and
`r_base_antenna` separately. The fusion Jacobian includes yaw-to-position
lever-arm coupling. A position-only observation updates position without
pretending to observe yaw.

For a valid dual-antenna solution, the converter can derive a base pose and set
`position_is_base_link=true`.

## Failure model

Sensor boundary failures are rejected rather than silently substituted.
Estimator state remains at the last accepted solution and diagnostics identify
the reason. Graph optimization uses transactional insertion: an invalid factor
or failed solve does not mutate the retained window.

A rejected scan is never promoted to the next scan-to-scan target. A rejected
external submap candidate does not mutate production odometry and is not added
to the scan-to-submap target. Repeated external-match rejection rebuilds only
the external target while preserving its last committed correction.

Wheel speed is optional. Hessian-derived directional information, external
scan-to-submap matching, IMU yaw factors, and GNSS recovery do not depend on a
wheel topic being present. The odometer does not create a binary LiDAR observability
class: Hessian information is used continuously for factor weighting, covariance
scaling, and diagnostics.

## Autoware localization-interface integration

The optional `pure_localization_interface_adapter` is outside the estimator core.
It consumes fused `map -> base_link` odometry and republishes standard Autoware
localization-facing topics:

```text
/localization/ekf_odom
        |
        v
pure_localization_interface_adapter
        |-- /localization/kinematic_state
        |-- /localization/pose_estimator/pose_with_covariance
        |-- /localization/twist_with_covariance
        |-- /localization/acceleration
        `-- map -> base_link TF
```

In the localization-only Autoware integration launch, the gyro odometer and
`pure_gnss_map_odom_fusion` keep publishing their odometry messages, but both TF
outputs are disabled. The launch assigns the direct `map -> base_link`
transform to the adapter. Without GNSS, `/initialpose` creates the initial map
anchor only after a positive simulation clock and nonzero-stamped local
odometry have been observed.

The pinned Docker profile also launches the real Autoware 1.9.0
`pose_instability_detector` against kinematic state and
twist-with-covariance, and `localization_error_monitor` against kinematic state.
The deterministic contract test verifies adapter message/frame/stamp copying,
TF edge consistency, the adapter's unique runtime `/tf` endpoint, absence of a
competing `base_link` parent, diagnostic names and keys, and
normal/error/recovery transitions. The Jazzy package tests separately launch
the supported profile matrix and compare exact endpoint counts, owner frame
parameters, emitted edges, and deliberately disabled owners. A recorded TF
stream alone cannot identify duplicate publishers of the same
`map -> base_link` edge; that identity check therefore uses the live ROS graph.
The pose and twist inputs are both derived from the same fused odometry, so the
pose-instability comparison is not independent and cannot expose every
common-mode estimator error. Neither monitor checks topic dropout; availability
monitoring remains a separate integration requirement.

Autoware map/localization, perception, planning, control, system, and API modules
are disabled in this profile. The adapter is therefore an integration boundary,
not a second estimator and not a complete full-stack localization-state API.
