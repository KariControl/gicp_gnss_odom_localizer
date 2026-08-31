# Tuning guide

## Order of work

1. Verify timestamps, frame IDs, and calibrated static TFs.
2. Tune strict deskew and inspect moving vertical edges before registration.
3. Establish stable `scan_to_scan` odometry.
4. Validate corrected-IMU yaw and bias during stationary and turning segments.
5. Tune the fixed-lag smoother with ZUPT, NHC, and wheel assistance disabled.
6. Evaluate the isolated scan-to-submap overlay with the same bags.
7. Calibrate GNSS covariance tables and validate initialization/outage return.
8. Enable robot-specific ZUPT, NHC, or wheel factors one at a time.

## Scan-to-scan contract

The odometer supports only:

```yaml
lidar_odom.tracking_mode: scan_to_scan
```

Do not use retired `lidar_odom.scan_to_submap.*` or
`lidar_odom.local_map.*` parameters. Submap matching now belongs to
`pure_lidar_submap_matcher` and consumes the optional exact-key accepted-scan
stream without changing the scan-to-scan odometer.

The generic profile is intentionally conservative. The following
vehicle-specific or control-oriented features are disabled and must be enabled
and validated independently:

```yaml
wheel_speed.use: false
wheel_speed.low_speed.enable: false
wheel_speed.scale_estimation.enable: false
wheel_speed.observability_assist.enable: false
lidar_odom.smoother.zupt.enable: false
lidar_odom.smoother.nhc.enable: false
out_filtered_odom.enable: false
```

These switches cover wheel assistance, wheel-scale learning, low-speed wheel
shaping, ZUPT, NHC, and the control-oriented filtered-odometry output. None is
enabled automatically from a low observability score or a detected stop.

## Deskew checks

Measure rejected-cloud reasons, IMU boundary/internal gaps, point-time span, and
output edge sharpness. Do not enable point-order timing fallback merely to
suppress rejections; add or convert the actual per-point time field.

## Common LiDAR metrics

For scan-to-scan and scan-to-submap runs record:

- absolute trajectory error where a trusted reference is available;
- relative pose error over fixed time and distance intervals;
- endpoint and 100 m normalized drift;
- yaw drift and lateral corridor error;
- registration rejection rate;
- fixed-lag rollback count;
- maximum frame-to-frame pose correction;
- CPU time, callback latency, and dropped point clouds.

Do not tune only on one straight sequence. Include turns, long corridors, open
spaces, repeated structures, stop/start, people or vehicles, and degraded
point clouds.

## Isolated scan-to-submap matcher

The odometer snapshot bridge is enabled only for scan-to-submap evaluation:

```yaml
lidar_odom.tracking_mode: scan_to_scan
lidar_odom.external_submap_snapshot.enable: true
lidar_odom.external_submap_snapshot.publish_interval_frames: 5
```

Keep the bridge's interval fixed while tuning the external matcher. Validate
that accepted-scan publication does not change the scan-to-scan trajectory and
that every matcher correction uses the exact session/generation/sequence/stamp
contract.

### Build-up and map size

The external map becomes ready only after both limits are satisfied:

```yaml
min_keyframes: 3
min_points: 200
max_keyframes: 15
map.voxel_leaf_m: 0.35
map.max_points: 80000
```

Keyframes use minimum time plus translation/yaw motion, with a maximum time as a
backstop. Too-dense keyframes increase cost and correlation; too-sparse
keyframes reduce overlap.

### Registration gates

Start with conservative values and inspect matcher rejection reasons:

```yaml
registration.max_corr_dist_m: 1.0
gate.max_fitness: 1.0
gate.min_inlier_ratio: 0.25
gate.max_correction_translation_m: 0.75
gate.max_correction_yaw_rad: 0.15
gate.max_correction_z_m: 0.25
gate.max_correction_roll_pitch_rad: 0.15
```

A high rejection rate is not automatically solved by loosening every gate.
First inspect extrinsics, deskew, the raw-pose prediction, point density,
dynamic objects, and doubled structures in the rolling target.

### Robust commit and recovery

The matcher commits a persistent `odom_precision <- odom` transform only after
multiple consistent candidates:

```yaml
robust.window_size: 7
robust.min_consistent: 3
robust.consistency_translation_m: 0.30
robust.consistency_yaw_rad: 0.06
robust.max_commit_pivot_step_m: 0.50
robust.max_commit_yaw_step_rad: 0.12
consecutive_rejections_before_rebuild: 8
```

Tune consistency thresholds before increasing commit-step bounds. A rejected
scan must never enter the map. Consecutive rejection rebuilds the external map
but preserves the last committed correction, so verify both continuity and
eventual reacquisition. Track matcher diagnostics including accepted/rejected
counts, committed count, recovery rebuilds, stream resets, queue drops, and
publish count.

### Match rate and CPU

`match_every: 1` attempts an external match for every received snapshot. The
snapshot publisher itself may already downsample accepted scans by its interval,
so measure both settings together. The primary scan-to-scan registration still
runs once in the odometer; the external matcher is additional CPU work isolated
in another process.

## Continuous directional-information weighting

Keep Hessian weighting enabled for scan-to-scan LiDAR–IMU odometry:

```yaml
wheel_speed.use: false
lidar_odom.smoother.hessian_information.enable: true
lidar_odom.smoother.hessian_information.yaw_metric_m: 2.0
lidar_odom.smoother.hessian_information.min_direction_ratio: 0.02
odom_covariance.observability_max_scale: 2.0
```

Inspect normalized directional information in corridors, open areas, turns, and
repetitive structures. It is a continuous quality indicator, not a binary
failure decision. Tune `min_direction_ratio` only as a numerical/factor-weight
floor and `observability_max_scale` only as bounded published-covariance
inflation.

Wheel speed remains optional. Enable
`wheel_speed.observability_assist.enable` only with a separate bag comparison;
all public profiles keep it disabled.

## ZUPT and NHC

ZUPT can reduce stationary drift only when the stop detector is reliable at the
robot's minimum commanded speed. NHC is appropriate for a non-holonomic ground
vehicle but wrong for an omni-directional platform, hand-held scanner, or a
vehicle with material lateral slip. Keep both disabled in the generic profile
and validate them separately.

## GNSS covariance and outage tests

For each fix quality and environment, compute empirical horizontal error
distributions against a trusted reference. Do not equate RTK status with
guaranteed accuracy. Validate at least startup with and without observable yaw,
good Fix to outage to good Fix, isolated outliers, inconsistent heading return,
multipath, stationary return, moving return, and reverse motion where relevant.

Run every representative outage bag once with scan-to-scan and once with the
isolated scan-to-submap overlay. Acceptance must cover no unbounded pose jump,
stable initial global yaw, exact anchor freeze outside strict fusion health,
bounded recovery, scan-to-scan non-intrusion, local/global GLIM error, and
runtime health. Use the repository tools under
`tools/evaluation/precision/scripts` rather than comparing only endpoint error.
