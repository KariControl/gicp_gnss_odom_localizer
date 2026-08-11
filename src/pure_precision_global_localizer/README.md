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
precision-local output continues independently.

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

The legacy position-only GNSS estimator is diagnostic-only and disabled by
default (`fallback.gnss_position_enabled=false`); it is never mixed into the
authoritative existing-fusion anchor.
