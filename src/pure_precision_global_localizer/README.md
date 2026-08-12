# pure_precision_global_localizer

This node keeps the existing scan-to-scan fusion path isolated while producing
precision-local and precision-global odometry on separate topics. It publishes
no TF.

The authoritative precision-global anchor is

`map <- odom_precision = (map <- base from /localization/ekf_odom) *
inverse(odom_precision <- base)`.

Candidates are accepted only while the exact
`localization/gnss_map_odom_fusion` diagnostic status is fresh and `OK`, with
`recovery.state=tracking`, `anchor_valid=true`, both position and yaw fused, and
`last_fix_state=good`. Three independent stable candidates are required before
the first global output. The first anchor is committed atomically; subsequent
updates and recovery use latest-base-pivot bounded steps.

During outage, reacquisition, recovery, stale diagnostics, stale existing-global
output, or missing G/P synchronization, the published anchor is frozen. The
precision-local output continues independently. Global XY therefore retains the
same fail-closed frozen-anchor composition.

Published global yaw has an additional outage guard. While existing fusion is
strictly healthy, the node independently estimates a robust SE(2) yaw from the
GNSS position sequence and precision-local position sequence. Only estimates
that pass the baseline, inlier, RMS, uncertainty, and stable-candidate gates can
refresh this trusted yaw reference. When existing fusion becomes unhealthy, the
last trusted reference is held and introduced through bounded yaw-only steps;
on recovery the offset is released through the same bounded policy. Unhealthy
samples cannot refresh the trusted reference, and this guard cannot alter
global XY or the existing-fusion anchor state.

At outage entry, the guard snapshots the trusted yaw-reference variance. That
snapshot remains unchanged throughout the outage and recovery release, even if
healthy observations refresh the next trusted reference during release. During
an outage, added yaw variance is the snapshot plus the squared unapplied
target-offset residual. During release, it is the snapshot plus the squared
applied offset. `READY` and `DISARMED` clear the active snapshot and add no
guard variance. This prevents the bounded yaw correction from understating the
published orientation covariance.

## Restart contract

Raw odometry messages do not carry an odometer session identifier. A new
`SubmapScan` odometer session therefore resets and silences precision-global
output. Re-arming requires a subsequently received, fresh, valid status that
explicitly reports non-tracking/unhealthy fusion, followed by a fresh
strict-tracking edge and three new stable candidates. Missing or stale
diagnostics never satisfy this handshake. In deployment,
restart `pure_gnss_map_odom_fusion` whenever the odometer is restarted so that
it explicitly reports `uninitialized`/non-tracking before returning to
tracking. If that transition is not observed, precision-global remains silent
by design.

The legacy position-only global-output fallback remains disabled by default
(`fallback.gnss_position_enabled=false`). Its gated position-alignment estimator
is shared only as the outage guard's trusted yaw observer; it is never mixed
into the authoritative existing-fusion anchor or global XY.
