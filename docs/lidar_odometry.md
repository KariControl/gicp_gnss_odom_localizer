# LiDAR–IMU odometry

## Tracking-mode selection

The primary LiDAR registration path is selected at node startup:

```yaml
lidar_odom.tracking_mode: scan_to_scan   # backward-compatible default
# or
lidar_odom.tracking_mode: scan_to_submap
```

`src/pure_lidar_gyro_odometer/param/param.yaml` selects `scan_to_scan` and
`param_scan_to_submap.yaml` selects `scan_to_submap`. Both modes use the same
odometry topic, covariance output, IMU input, and GNSS fusion interface.
`scan_to_submap` requires `lidar_odom.pose_se2.enable: true` and
`lidar_odom.smoother.enable: true` so keyframe placement can be repaired from
the retained optimized pose window. Changing the mode does not require changing
downstream consumers.

The mode is not dynamically changed while the node is running. Restart the
odometer after changing the YAML.

## Scan-to-scan primary mode

Every accepted scan is downsampled and registered against the previous accepted
scan using small_gicp GICP or VGICP. Corrected IMU yaw supplies the rotational
initial guess when the full prior is not requested.

Only a valid registration becomes the next target. A rejected or malformed scan
is not promoted, so one bad match cannot directly poison the next scan pair.
A gap longer than `lidar_odom.timeout_sec` resets the scan pair, smoother anchor,
and optional local map at the current accepted odometry pose.

The optional legacy periodic local-map factor is controlled by:

```yaml
lidar_odom.local_map.enable: true
```

This lower-rate factor exists only as an additional consistency observation in
`scan_to_scan` mode. It is not needed to activate the submap in
`scan_to_submap` mode.

## Scan-to-submap primary mode

### State sequence

The submap path deliberately retains scan-to-scan instead of deleting it:

```text
startup / submap reset
        |
        v
scan-to-scan warm-up ----> active submap ready
                                  |
                    +-------------+-------------+
                    |                           |
                    v                           v
          scan-to-submap accepted       submap attempt rejected
             primary measurement          scan-to-scan fallback
                    |
                    v
       repaired rolling keyframe submap
```

When `lidar_odom.scan_to_submap.match_interval_frames` is greater than one,
non-submap frames use `scan_to_scan_interim`. These scheduled interim frames are
not counted as submap failures and are used even when failure fallback is
disabled.

### Primary observation

Once the map contains at least `lidar_odom.local_map.min_keyframes` and
`lidar_odom.local_map.min_points`, the current scan is registered against the
rolling target. A candidate must pass all of the following before it is the
primary factor:

- small_gicp convergence;
- finite transform and Hessian;
- local-map fitness and inlier-ratio gates;
- maximum planar translation and yaw correction from the motion prediction;
- maximum vertical, roll, and pitch correction;
- optional disagreement gates against the scan-to-scan monitor.

The accepted absolute pose in the submap anchor is converted into a relative
transform from the previous accepted vehicle pose. Only that selected relative
measurement is inserted into the fixed-lag graph; the scan-to-scan result is not
also inserted as a second independent factor for the same interval.

### Motion prediction and monitor

Scan-to-scan is evaluated in submap mode for three reasons:

1. it bootstraps the submap;
2. it supplies a close initial pose for local-map registration;
3. it provides scheduled interim propagation and a configurable failure
   fallback.

A large scan-to-scan/submap disagreement can reject the submap candidate when
the configured disagreement limit is positive. Set either limit to zero to
disable that particular gate. This comparison is a safety gate, not a claim
that scan-to-scan is statistically independent of the submap result.

### Repairable submap representation

Keyframe clouds are stored in an independent submap-anchor frame rather than
being permanently transformed into `odom` at insertion time. Each keyframe also
records the corresponding accepted-pose sequence number.

After a successful fixed-lag solve:

1. retained optimized poses are associated with recent keyframes;
2. corrected keyframe poses are re-expressed in the active anchor frame;
3. the local point cloud is rebuilt when a retained keyframe changed;
4. the anchor-to-odom transform is adjusted as a rigid transform;
5. when old keyframes leave the rolling window, the map is re-anchored on the
   oldest retained keyframe.

This prevents the previous failure mode in which scans were permanently baked
with stale odometry poses, producing doubled walls and a progressively warped
registration target.

Only keyframes still represented by the fixed-lag window receive individual
pose repair. Older retained keyframes move rigidly with the submap anchor. This
is a local odometry mechanism, not global pose-graph SLAM.

### Failure containment

If a submap attempt fails and
`lidar_odom.scan_to_submap.fallback_to_scan_to_scan` is true, a valid
scan-to-scan result can propagate the pose for that interval. A failed-fallback
scan is not inserted into an already-ready submap, which limits target
contamination.

After `lidar_odom.scan_to_submap.max_consecutive_failures` failed submap
attempts, the active map is discarded and rebuilt from the current accepted
pose. Tracking then returns to scan-to-scan warm-up. If failure fallback is
disabled and neither path is valid, the estimator holds the last accepted pose.

## Fixed-lag SE(2) graph

The graph contains poses `(x, y, yaw)` and relative factors for:

- the selected LiDAR registration path;
- corrected-IMU yaw increment;
- optional periodic local-map pose in scan-to-scan mode;
- optional stationary ZUPT;
- optional non-holonomic lateral displacement;
- weak smoothness regularization.

Factor insertion is transactional. Non-finite data, indefinite information,
solve failure, or an optimized correction over the configured bounds restores
the previous factor window and pose. The caller then applies the selected
LiDAR/IMU relative motion once and resets the smoother at that deterministic
fallback pose.

## Continuous anisotropic registration information

The selected registration Hessian is reduced from SE(3) to SE(2), metric-scaled,
and eigendecomposed. Each normalized information ratio remains continuous in
`[0, 1]`; the ratios are not thresholded into a `degenerate/normal` class.
Directions with less information receive less weight in the fixed-lag graph,
while a small configurable numerical floor keeps the linear system solvable:

```yaml
lidar_odom.smoother.hessian_information.enable: true
lidar_odom.smoother.hessian_information.yaw_metric_m: 2.0
lidar_odom.smoother.hessian_information.min_direction_ratio: 0.02
odom_covariance.observability_max_scale: 2.0
```

Low directional information alone does not reject a registration, change the
tracking source, latch a critical state, or force a different next-frame guess.
Registration acceptance remains based on convergence, finite output, valid
timing, and the mode-specific fitness and submap gates.

The published covariance increment is scaled smoothly from `1.0` to
`odom_covariance.observability_max_scale` using the mean normalized information
deficit. This is a conservative quality indicator, not a calibrated statistical
covariance derived from independent observations.

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

## Diagnostics

The odometer diagnostic status reports at least:

- configured tracking mode, selected registration source, validity, and explicit
  registration rejection reason;
- `scan_to_submap`, warm-up, interim, and fallback counts;
- local-map readiness, keyframe/point count, accepted/rejected attempts, and
  consecutive failures;
- last submap rejection reason, fitness, inlier ratio, and disagreement;
- normalized minimum/middle/maximum directional information ratios and continuous
  information deficit;
- optional wheel-prior difference and continuous assist amount;
- graph and pose health through the existing odometry status fields.

The former binary `/localization/lidar_degenerate` and
`/localization/lidar_pose_mode` topics are removed. Continuous values are always
available in `/diagnostics`; an optional verbose string stream can be enabled at
`/localization/lidar_observability_debug`.

Use the actual `lidar_last_registration_source` rather than assuming that the
configured mode succeeded on every frame.

## What scan-to-submap does not solve

A rolling local map reduces short-horizon accumulation and makes registration
less dependent on one immediately preceding scan. It does not provide a global
absolute observation. Without GNSS, another external anchor, or loop closure,
long-duration global drift remains possible.
