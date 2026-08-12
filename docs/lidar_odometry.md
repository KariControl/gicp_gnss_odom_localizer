# LiDAR–IMU odometry

## Scan-to-scan contract

`pure_lidar_gyro_odometer` has one supported primary registration path:

```yaml
lidar_odom.tracking_mode: scan_to_scan
```

The parameter is retained for configuration compatibility and diagnostics, but
any other value is rejected at startup. The former in-node scan-to-submap path,
its rolling local map, and the optional periodic local-map factor have been
retired. High-precision localization is provided by a separate process branch;
it does not change the production odometer's registration source.

Every accepted scan is downsampled and registered against the previous accepted
scan using small_gicp GICP or VGICP. Corrected IMU yaw supplies the rotational
initial guess when the full prior is not requested. Only a valid registration
becomes the next target. A rejected or malformed scan is not promoted, so one
bad match cannot directly poison the next scan pair. A gap longer than
`lidar_odom.timeout_sec` resets the scan pair and smoother at the current
accepted odometry pose.

## Strict deskew input contract

When IMU deskew is enabled, a point cloud is rejected rather than approximated
if any of these conditions holds:

- no supported per-point time field is present;
- point times are non-finite, have no usable span, or disagree with the cloud
  timestamp;
- IMU samples do not cover the complete scan interval, or an internal IMU gap
  exceeds the configured limit;
- the required `base_link <- scan` or `base_link <- imu` static transform is
  unavailable;
- the `PointCloud2` field layout is malformed or the message is big-endian;
- translation deskew is enabled but the selected speed source does not cover
  the scan interval.

The compatibility parameters `allow_linear_time_fallback` and
`allow_default_speed_fallback` remain `false` in the supplied configurations.
Do not enable them to hide a timestamp, driver, TF, or speed-source defect in a
quantitative evaluation.

## Fixed-lag SE(2) graph

The graph contains poses `(x, y, yaw)` and relative factors for:

- scan-to-scan LiDAR registration;
- corrected-IMU yaw increment;
- optional stationary ZUPT;
- optional non-holonomic lateral displacement;
- weak smoothness regularization.

Factor insertion is transactional. Non-finite data, indefinite information,
solve failure, or an optimized correction over the configured bounds restores
the previous factor window and pose. The caller then applies the selected
LiDAR/IMU relative motion once and resets the smoother at that deterministic
fallback pose.

## Continuous anisotropic registration information

The registration Hessian is reduced from SE(3) to SE(2), metric-scaled, and
eigendecomposed. Each normalized information ratio remains continuous in
`[0, 1]`; it is not thresholded into a `degenerate/normal` class. Directions
with less information receive less weight in the fixed-lag graph, while a small
configurable floor keeps the linear system solvable:

```yaml
lidar_odom.smoother.hessian_information.enable: true
lidar_odom.smoother.hessian_information.yaw_metric_m: 2.0
lidar_odom.smoother.hessian_information.min_direction_ratio: 0.02
odom_covariance.observability_max_scale: 2.0
```

Low directional information alone does not reject a registration, switch its
source, latch a critical state, or force a different next-frame guess.
Registration acceptance remains based on convergence, finite output, valid
timing, and the scan-to-scan fitness gate.

The published covariance increment is scaled smoothly from `1.0` to
`odom_covariance.observability_max_scale` using the mean normalized information
deficit. This is a conservative quality indicator, not a calibrated covariance
derived from independent observations.

Wheel speed is not required. When explicitly enabled, the optional continuous
assist can blend only the lower-information Hessian directions toward a
wheel-plus-IMU prior:

```yaml
wheel_speed.use: true
wheel_speed.observability_assist.enable: true
wheel_speed.observability_assist.max_blend: 0.5
wheel_speed.observability_assist.power: 2.0
```

The public profiles keep this assist disabled. There is no threshold that turns
it on automatically.

## Isolated precision snapshot bridge

The optional `lidar_odom.external_submap_snapshot.*` publisher is deliberately
not a registration path. When enabled by
`pure_precision_bringup/config/submap_snapshot_override.yaml`, it copies an
accepted scan and its unmodified scan-to-scan pose to
`/localization/submap_scan` at the configured accepted-pose interval.

Each message contains an exact identity tuple:

```text
odom_session_id + odom_generation + sequence + accepted scan stamp
```

The first accepted pose in every odometer generation is always published. A
tracking reset increments the generation, preventing the external consumer from
mixing scans across discontinuous source streams. Cloud conversion and
publication occur only when the bridge is enabled; every normal odometer profile
keeps it disabled.

`pure_lidar_submap_matcher` consumes these snapshots in a separate process. It
builds its own rolling map, gates and robustly commits a persistent full-SE(2)
`odom_precision <- odom` correction, and publishes
`/localization/submap_correction`. `pure_precision_global_localizer` then emits
the separate `/localization/precision_local_odom` and
`/localization/precision_global_odom` outputs. Neither node publishes TF or feeds
back into `/localization/gyro_lidar_odom` or the existing GNSS fusion.

This isolation is a required safety property. Do not enable snapshots and then
set the odometer tracking mode to anything other than `scan_to_scan`; startup
validation rejects that combination.

## Diagnostics

The odometer diagnostic status reports at least:

- the configured `scan_to_scan` mode, registration validity, source, and an
  explicit rejection reason;
- normalized directional information ratios and the continuous information
  deficit;
- optional wheel-prior difference and continuous assist amount;
- graph, pose, covariance, and next-guess health;
- whether accepted-scan snapshots are enabled, their exact-key contract,
  session/generation/sequence, publish count, and conversion cost.

The former binary `/localization/lidar_degenerate` and
`/localization/lidar_pose_mode` topics are removed. Continuous values are
available in `/diagnostics`; an optional verbose string stream can be enabled at
`/localization/lidar_observability_debug`.

The external matcher and precision-global localizer publish their own
diagnostics. Use those statuses for submap acceptance/rebuild and global-anchor
health instead of expecting internal-submap counters from the odometer.

## Scope

The baseline remains local scan-to-scan dead reckoning. The isolated rolling
submap reduces short-horizon accumulation but supplies no global absolute
observation. Without GNSS, another external anchor, or loop closure,
long-duration global drift remains possible in both local outputs.
