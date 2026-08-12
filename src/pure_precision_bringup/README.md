# Pure precision bringup

This package starts an isolated precision branch next to the existing
scan-to-scan/GNSS localization stack.  It does not remap the existing odometry
or GNSS-fusion inputs, and neither precision node publishes TF.

The base odometer must be launched with
`config/submap_snapshot_override.yaml`.  That enables an exact-key snapshot of
accepted scans while explicitly keeping `lidar_odom.tracking_mode` set to
`scan_to_scan`.

Outputs used for evaluation are:

- `/localization/precision_local_odom`
- `/localization/precision_global_odom`
- `/localization/precision_global_pose`

The existing `/localization/gyro_lidar_odom` and `/localization/ekf_odom`
remain unchanged and are recorded alongside these topics for A/B comparison.

The precision-global branch owns a dedicated `map -> odom_precision` anchor
(represented internally; it is not broadcast as TF). Its authoritative
candidate is
`map<-precision = map<-base(existing fusion) * inverse(precision<-base)`.
The shared scan-to-scan odometry term therefore cancels algebraically instead
of being composed back into the precision result as drift. Candidates are
accepted only while the existing fusion diagnostic is fresh and strictly
healthy: level OK, recovery state `tracking`, valid anchor, position and yaw
fused, and a good fix. Startup requires three stable candidates at least
0.25 seconds apart. Loss of any health condition freezes target and applied
anchors exactly; recovery requires three new stable candidates and resumes with
bounded steps. The original position-only GNSS estimator is disabled as an
authority by default.

The initial activation is atomic: the activation raw sample itself remains
suppressed and the first odometry/pose pair is published on exactly the next
unique raw stamp. A new odometer process/session clears the old anchor and
requires a fresh, explicit unhealthy or non-`tracking` fusion status followed
by a fresh strict-`tracking` status before candidates may resume. Diagnostic
staleness or unavailability alone never satisfies that rearm edge. Ordinary
fusion outages do not invoke the session-rearm path; they use the exact
anchor-freeze and three-candidate bounded-recovery path above.

This is a one-way observation of the existing global result. Neither the
dedicated anchor nor either precision output is subscribed by the existing
odometer or GNSS fusion, so there is no feedback into the production path.

For the Hesai bag runner, select the branch with
`--localization-mode precision`; `baseline` starts no precision process and
does not publish or record precision topics. The odometer remains explicitly
configured for `scan_to_scan` in both modes. Both modes use the requested bag
playback rate, normally `--rate 1.0`. The former
`--tracking-mode scan_to_scan|scan_to_submap` interface remains temporarily as
a warning-producing compatibility alias for `baseline|precision`.

The LiDAR/IMU-only GLIM runner can exercise the same isolated local branch
without starting GNSS or global fusion:

```bash
script/run_lidar_imu_glim_bag.sh \
  --sensor velodyne --localization-mode precision --snapshot-interval 2 \
  --output /path/to/result
```

It feeds the compositor from the accepted-scan odometry topic, records no
precision-global output, and runs `validate_lidar_imu_submap_run.py` before
accepting a result. `evaluate_lidar_imu_submap_ab.py` performs the corresponding
physical-stamp GLIM A/B against a baseline run. The 2026-08-12 Velodyne and
MID360 evaluation did not pass the accuracy adoption gates, so this command is
an evaluation path and does not enable precision-local as the production
LiDAR/IMU output.

Raw non-intrusion is evaluated with an additional instrumented control run:

```bash
script/run_hesai_localization_bag.sh \
  --localization-mode baseline --accepted-scan-control \
  --bag /path/to/input_bag --rate 1.0 --output /path/to/control_result
```

This uses the same accepted-scan snapshot publisher and recorder load as the
precision run, but starts neither precision node. It is an evaluation-only
control; normal `baseline` remains free of snapshot conversion/publishing.

The precision recorder is checked with:

```bash
ros2 run pure_precision_bringup validate_precision_bag.py \
  /path/to/run/localization_output --expected-rate 1.0
```

Authority counters are compared at the causal boundary of the final recorded
precision diagnostic, not against messages recorded afterward during the
shutdown tail. Existing-global receipt, unique positive stamps, duplicates,
and rejects must match that prefix exactly. Fusion-health accounting also
replays its positive unique/duplicate/zero-stamp rules exactly. The only
permitted unrecorded difference is rejected `/clock==0` startup health status
received before the runner starts its recorder; it may increase
received/rejected but never accepted. Malformed health schema, backsteps, or
unconserved counters remain hard failures.

`evaluate_precision_glim_ab.py` then compares one speed bag and one precision
bag on physical ROS header stamps. It freezes the speed-run calibration for
both trajectories, reports fixed-frame and independently aligned shape ATE,
yaw, 10/50/100 m RPE, outage behavior, and treats 50 Hz timer-to-timer
differences as reference warnings rather than raw non-intrusion gates.
With optional `--plot-directory /path/to/plots`, it also writes local/global
three-panel PNGs (trajectory, absolute XY error, and absolute yaw error) and
the aligned samples as CSV. These artifacts use the same frozen shared
speed-baseline SE(2) and separate circular yaw offsets as the fixed-frame JSON
metrics; CSV readback RMSE is checked against those metrics before the
evaluation report is accepted. Calibration and common GNSS-outage intervals
are shaded in the error panels.

The raw non-intrusion check preserves the exact-common pose/increment gates and
adds interval-policy counts, at least 99.8% `(generation, sequence)` coverage,
a 0.1% final accepted-input bound, integral raw-frame phase residual within
5 ms, and hard non-accumulation bounds: the signed phase's robust 1st-to-99th
percentile span and early-to-late median drift must each remain within two raw
frames. The absolute integer-frame phase is diagnostic-only because independent
1.0x replays may reject a few different startup registrations and then retain a
constant sequence offset. A bounded SE(2) interpolation of physical accepted
snapshots checks both pose and increment through that constant phase; it never
interpolates timer odometry. Exact-stamp coverage is also a warning, while
snapshot policy, accepted density, phase stability, and the trajectory checks
remain hard gates. Recorder lag against the latest observed `/clock` is reported
as an explicit warning, so a transient replay-start backlog is visible even
when it does not accumulate or alter the bounded physical trajectory:

```bash
ros2 run pure_precision_bringup evaluate_accepted_scan_nonintrusion.py \
  --control-bag /path/to/control_result/localization_output \
  --precision-bag /path/to/precision_result/localization_output \
  --label example --output-json /tmp/accepted_scan.json \
  --output-markdown /tmp/accepted_scan.md
```

`evaluate_startup_acceptance.py` adds the global-yaw publication safety gates:
no global odometry/pose before explicit readiness, atomic target/applied yaw at
the first output on the next raw sample, strict existing-fusion authority and
freshness, exact anchor freeze outside strict health, activation within 25
seconds, and no greater than 10 degree transient against the frozen-frame GLIM
and legacy-global references.
