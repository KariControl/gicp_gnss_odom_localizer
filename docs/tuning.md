# Tuning and validation

## Order of work

1. Verify timestamps, frame IDs, and calibrated static TFs.
2. Tune strict deskew and inspect moving vertical edges before tuning
   registration.
3. Establish a stable `scan_to_scan` baseline.
4. Validate corrected-IMU yaw and bias behavior during stationary and turning
   segments.
5. Enable the fixed-lag smoother with ZUPT, NHC, wheel assistance, and the
   legacy periodic local-map factor still disabled.
6. Evaluate the supplied `scan_to_submap` profile against exactly the same bags.
7. Calibrate GNSS covariance tables and validate GNSS initialization/outage
   return.
8. Enable robot-specific ZUPT, NHC, or wheel factors one at a time.

## Mode switch

The backward-compatible baseline is:

```yaml
lidar_odom.tracking_mode: scan_to_scan
```

The rolling-submap primary path is:

```yaml
lidar_odom.tracking_mode: scan_to_submap
lidar_odom.pose_se2.enable: true
lidar_odom.smoother.enable: true
lidar_odom.scan_to_submap.fallback_to_scan_to_scan: true
lidar_odom.scan_to_submap.match_interval_frames: 1
lidar_odom.scan_to_submap.max_consecutive_failures: 5
```

Use `param_scan_to_submap.yaml` for an initial experiment. Do not change the
registration voxel size, smoother weights, and tracking mode in the same first
comparison; otherwise the source of a change is ambiguous.

## Deskew checks

Measure rejected-cloud reasons, IMU boundary/internal gaps, point-time span, and
output edge sharpness. Do not enable point-order timing fallback merely to
suppress rejections; add or convert the actual per-point time field.

## Common LiDAR metrics

For both modes record:

- absolute trajectory error where a trusted reference is available;
- relative pose error over fixed time and distance intervals;
- endpoint and 100 m normalized drift;
- yaw drift and lateral corridor error;
- registration rejection rate;
- fixed-lag rollback count;
- maximum frame-to-frame pose correction;
- CPU time, callback latency, and dropped point clouds.

Do not tune only on one straight sequence. Include turns, long corridors, open
spaces, repeated structures, stop/start, people or vehicles, and degraded point
clouds.

## Scan-to-submap tuning

### Build-up

The map becomes ready only after both minimum keyframe and point counts are met:

```yaml
lidar_odom.local_map.min_keyframes: 3
lidar_odom.local_map.min_points: 200
```

Keyframes are selected by minimum frame interval plus translation/yaw movement,
with a maximum interval as a backstop. Too-dense keyframes increase cost and
correlation; too-sparse keyframes reduce overlap.

### Registration gates

Start with tight values and inspect rejection reasons:

```yaml
lidar_odom.local_map.max_corr_dist_m: 1.0
lidar_odom.local_map.max_fitness: 1.0
lidar_odom.local_map.min_inlier_ratio: 0.25
lidar_odom.local_map.max_correction_translation_m: 0.5
lidar_odom.local_map.max_correction_yaw_rad: 0.12
```

A high rejection rate is not solved automatically by loosening every gate. First
check extrinsics, deskew, prediction error, point density, dynamic objects, and
whether the local target contains doubled structures.

### Scan-to-scan disagreement

The monitor limits are:

```yaml
lidar_odom.scan_to_submap.max_scan_to_scan_disagreement_translation_m: 1.0
lidar_odom.scan_to_submap.max_scan_to_scan_disagreement_yaw_rad: 0.35
```

A value of zero disables that individual gate. Because the two estimates share
point data, disagreement is a sanity check rather than an independent voting
system. Inspect both fitness and inlier ratio before deciding which path was
wrong.

### Match rate and CPU

`match_interval_frames: 1` runs a submap match every accepted frame. The current
implementation also evaluates scan-to-scan for prediction and fallback, so this
mode can require roughly two registrations per frame. Measure on the target CPU.

When increasing the interval, scheduled non-submap frames use
`scan_to_scan_interim` and are not counted as failures. Compare drift and CPU
rather than assuming a lower rate is always sufficient.

### Failure reset

Track:

- `local_map_last_reason`;
- `local_map_consecutive_failures`;
- `scan_to_scan_fallback_count`;
- `local_map_reset_reason`.

Repeated failure should cause a controlled `consecutive_scan_to_submap_failures`
reset followed by warm-up. It must not create a large `odom` discontinuity.

## Continuous directional-information weighting

Keep Hessian weighting enabled for the LiDAR–IMU-only baseline:

```yaml
wheel_speed.use: false
lidar_odom.smoother.hessian_information.enable: true
lidar_odom.smoother.hessian_information.yaw_metric_m: 2.0
lidar_odom.smoother.hessian_information.min_direction_ratio: 0.02
odom_covariance.observability_max_scale: 2.0
```

Inspect the normalized directional information ratios in corridors, open areas,
turns, and repetitive structures. They are continuous quality indicators, not a
binary failure decision. Low information alone must not cause registration
rejection, a source switch, or a next-frame initial-guess change.

Tune `min_direction_ratio` only as the numerical/factor-weight floor: making it
too large over-trusts genuinely weak directions, while making it extremely small
can make the graph poorly conditioned. Tune `observability_max_scale` as a bounded
published-covariance inflation, not as an estimator gain switch.

Wheel speed, when present, remains optional. First validate the LiDAR–IMU-only
baseline. Then enable `wheel_speed.observability_assist.enable` only with a
separate bag comparison; its blend varies continuously with directional
information and is disabled in all public profiles.

## ZUPT and NHC

ZUPT can reduce stationary drift only when the stop detector is reliable at the
robot's minimum commanded speed. NHC is appropriate for a non-holonomic ground
vehicle but wrong for an omni-directional platform, hand-held scanner, or a
vehicle with material lateral slip. Keep both disabled in the generic profile
and validate them separately.

## GNSS covariance

For each fix quality and environment, compute empirical horizontal error
distributions against a trusted reference. Set sigma so normalized innovations
are plausible; do not equate RTK status with guaranteed accuracy. Validate
differential-age behavior and urban multipath separately.

## GNSS outage test

Replay at least these cases:

- startup stationary with position-only GGA;
- startup moving without direct yaw;
- good Fix to total outage to good Fix;
- outage returning with one gross outlier;
- heading return inconsistent with trajectory;
- RTK status with high covariance/multipath;
- reverse motion, if trajectory heading is expected to support it.

Run every outage bag once in `scan_to_scan` and once in `scan_to_submap` mode.
Acceptance should include no single-frame pose jump over configured bounds, no
one-fix reanchor, correct state transitions, and eventual convergence when the
window is consistent.
