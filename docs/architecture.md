# Architecture

## Coordinate frames

- `map`: globally anchored planar frame.
- `odom`: locally continuous dead-reckoning frame.
- `base_link`: vehicle state frame.
- LiDAR, IMU, and GNSS antenna frames: calibrated children of `base_link`.
- active submap anchor: internal SE(2) frame used only inside
  `pure_lidar_gyro_odometer`.

`pure_lidar_gyro_odometer` publishes `odom -> base_link`.
`pure_gnss_map_odom_fusion` estimates and optionally publishes `map -> odom`.
The two nodes must not both publish the same transform. The submap anchor is not
a TF frame and is never published as a competing global transform.

## Data flow

1. `pure_imu_undistortion` validates point timing and IMU coverage, transforms
   angular velocity into `base_link`, integrates a relative trajectory, and
   re-expresses points at the configured scan reference time.
2. `pure_lidar_gyro_odometer` selects either scan-to-scan or scan-to-submap as
   its primary LiDAR observation, attaches corrected-IMU yaw and optional
   constraints, and solves a fixed-lag SE(2) graph.
3. `pure_nmea_gnss_conversion` projects GGA, assigns covariance/confidence,
   selects a valid heading source, and publishes a physical GNSS observation.
4. `pure_gnss_map_odom_fusion` synchronizes that observation against odometry
   and updates the `map -> odom` anchor.


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
separate Linux processes. In the Autoware LSim launch,
`pure_autoware_localization_adapter` is another separate process, and Autoware
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

## Selectable LiDAR front end

```text
                                +-----------------------+
current scan -----------------> | scan-to-scan monitor  |
        |                       +-----------------------+
        |                                  |
        |                     warm-up / interim / fallback
        v                                  |
+-----------------------+                  |
| rolling local submap  | <--- repaired keyframes
+-----------------------+                  |
        |                                  |
        +-------- scan-to-submap ----------+
                         |
                  selected factor only
                         v
              fixed-lag SE(2) smoother
                         |
                  odom -> base_link
```

In `scan_to_scan` mode, the previous accepted scan is the primary target. In
`scan_to_submap` mode, scan-to-scan first builds the map, then the accepted
submap result becomes the primary measurement. Scan-to-scan remains available
between scheduled submap attempts and after a failed attempt.

## Active-submap state

The active map stores keyframe clouds and base poses in an internal anchor
frame. It also stores the accepted-pose sequence associated with each keyframe.
The fixed-lag smoother exposes its retained optimized pose window so recent
keyframes can be reprojected and the point target rebuilt after optimization.

The `odom <- anchor` transform can move rigidly without changing map geometry.
When the rolling keyframe deque drops old entries, the map is re-anchored to the
oldest retained keyframe. This keeps coordinates bounded and avoids baking
stale odometry into every point.

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

A rejected scan is never promoted to the next scan-to-scan target. A failed
submap fallback is not inserted into an already-ready submap. Repeated submap
attempt failures reset only the active local map and return the front end to its
scan-to-scan warm-up state; they do not reset the accumulated `odom` pose.

Wheel speed is optional. Hessian-derived directional information, submap tracking,
IMU yaw factors, and GNSS recovery do not depend on a wheel topic being present.
The odometer does not create a binary LiDAR observability class: Hessian information is
used continuously for factor weighting, covariance scaling, and diagnostics.

## Autoware logging-simulation interface

The optional `pure_autoware_localization_adapter` is outside the estimator core.
It consumes fused `map -> base_link` odometry and republishes standard Autoware
localization-facing topics:

```text
/localization/ekf_odom
        |
        v
pure_autoware_localization_adapter
        |-- /localization/kinematic_state
        |-- /localization/pose_estimator/pose_with_covariance
        |-- /localization/acceleration
        `-- map -> base_link TF
```

In the localization-only LSim launch, `pure_gnss_map_odom_fusion` keeps publishing
the fused odometry message but its `map -> odom` TF output is disabled. The
adapter is the only tested global TF publisher and sends a direct
`map -> base_link` transform. Without GNSS, `/initialpose` creates the initial
map anchor after local odometry has started.

Autoware map/localization, perception, planning, control, system, and API modules
are disabled in this profile. The adapter is therefore an integration boundary,
not a second estimator and not a complete full-stack localization-state API.
