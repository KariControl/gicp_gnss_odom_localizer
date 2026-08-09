# Migration to 0.2

## Rebuild interfaces

`pure_gnss_msgs/GnssFusionInput` now carries heading validity and antenna
observation semantics. Rebuild this workspace and every dependent workspace
from a clean `build/`, `install/`, and `log/` state.

## Add static transforms

Deskew now requires valid static `base_link <- LiDAR` and `base_link <- IMU`
transforms. Single-antenna GNSS fusion requires the calibrated
`base_link <- antenna` geometry. Parameter fallback is disabled by default;
enable it only with measured offsets.

## Point time

Clouds without a supported per-point time field are rejected by default. Set the
correct `time_fields`, `time_scale`, cloud stamp convention, and scan period.
`allow_linear_time_fallback=true` restores approximate point-order timing but is
not recommended for release use.

## NMEA output change

A position-only GGA remains on `gnss_fusion_input`, but legacy base-pose outputs
are withheld until yaw and antenna geometry are valid. Consumers that previously
read identity-quaternion poses must switch to the dedicated observation or
fused odometry.

`initial_heading_deg`, `use_previous_heading_fallback`, and old
lever-arm-without-heading options no longer create yaw. They are accepted only
as migration no-ops.

## GNSS return

Remove expectations around `time_out_dead_reckoning` and
`gnss_force_accept_*`. Recovery now needs a configurable candidate window and
applies bounded corrections. Tune the `recovery_*` group against outage bags.

## LiDAR tracking modes

The generic configuration remains backward compatible:

```yaml
lidar_odom.tracking_mode: scan_to_scan
```

The new primary local-submap path is selected with:

```yaml
lidar_odom.tracking_mode: scan_to_submap
```

A ready-made `param_scan_to_submap.yaml` is installed with the package. The mode
is read at startup and requires the SE(2) pose integrator and fixed-lag smoother
to remain enabled.

In submap mode:

- scan-to-scan is used for map warm-up;
- an accepted scan-to-submap result becomes the primary relative factor;
- scheduled interval gaps use scan-to-scan interim propagation;
- an attempted submap failure can use scan-to-scan fallback;
- repeated attempts reset and rebuild the local map without resetting `odom`;
- fixed-lag pose corrections repair recent keyframe placement.

`lidar_odom.local_map.enable` retains its old meaning only in `scan_to_scan`
mode: it enables the optional periodic local-map pose factor. The new
`scan_to_submap` mode activates its required rolling submap regardless of that
legacy flag.

## RC3 observability migration

RC3 removes the binary LiDAR `degenerate/normal` classifier and its stateful
latch. Delete the following obsolete keys from deployment YAML files because
they are no longer declared:

```text
lidar_odom.degeneracy_detection.*
wheel_speed.degeneracy.*
out_degeneracy*
out_pose_mode*
odom_covariance.degenerate_scale
```

The following binary topics are also removed:

```text
/localization/lidar_degenerate
/localization/lidar_pose_mode
/localization/lidar_degeneracy_debug
```

The selected registration Hessian is still used, but only as continuous
SE(2) directional information. The relevant public keys are:

```yaml
lidar_odom.smoother.hessian_information.enable: true
lidar_odom.smoother.hessian_information.yaw_metric_m: 2.0
lidar_odom.smoother.hessian_information.min_direction_ratio: 0.02
odom_covariance.observability_max_scale: 2.0
```

Continuous ratios, information deficit, assist amount, and explicit registration
rejection reasons are reported through `/diagnostics`. Optional verbose output is
available with:

```yaml
lidar_odom.observability.debug_pub.enable: true
lidar_odom.observability.debug_pub.topic: /localization/lidar_observability_debug
```

Wheel speed remains optional. Its continuous weak-direction assistance is an
explicit opt-in and is disabled in the public profiles:

```yaml
wheel_speed.use: false
wheel_speed.observability_assist.enable: false
```

A low Hessian information ratio no longer rejects a scan, switches the primary
tracking source, changes the next-frame initial guess, or creates a critical
state. Ordinary registration rejection for non-convergence, non-finite output,
invalid timing, or a configured fitness/submap gate remains unchanged.

## Optional constraints

The smoother uses anisotropic Hessian information by default. NHC, ZUPT, wheel
scale learning, low-speed shaping, filtered control odometry, and the legacy
periodic local-map factor remain disabled in the generic public parameters.
Enable one feature at a time with a regression test.

## Launch files

Both container and standalone bringup accept an `odom_param` YAML path. They
also remap `/localization/imu_corrected` and `/localization/is_stopped` into the
NMEA node. Corrected-IMU heading support defaults on when the GNSS node is
launched, but GNSS as a whole remains off by default in the main bringup.
