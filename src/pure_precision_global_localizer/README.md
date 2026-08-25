# pure_precision_global_localizer

This node keeps the existing scan-to-scan fusion path isolated while producing
precision-local and precision-global odometry on separate topics. It publishes
no TF.

The authoritative precision-global anchor is

`map <- odom_precision = (map <- base from /localization/ekf_odom) *
inverse(odom_precision <- base)`.

Candidates are accepted only while the fresh, typed
`/localization/gnss_map_odom_fusion_authority` event reports
`FULL_SE2_HEALTHY`, with `recovery_state=tracking`, a valid anchor, both
position and yaw fused, and a good last fix. A `SOFT_BAD_HOLD` event is
explicitly unhealthy here: it freezes the last trusted anchor even though the
map/odom fusion node may retain its internal recovery window. Three independent
stable candidates are required before the first global output. The first anchor
is committed atomically; subsequent updates and recovery use latest-base-pivot
bounded steps.

Authority changes caused by GNSS/odometry input are published immediately from
those callbacks. The periodic path also evaluates clock-driven fusion timeouts
and publishes a heartbeat for the freshness gate; it does not delay an
input-driven quality transition until the next timer tick. Each event carries a
producer session and monotonic sequence, so two transitions at the same ROS
time are not collapsed. `/diagnostics` mirrors the accepted typed state,
sequence and reason for audit, but is not itself a control input.

With simulated time, a transient-local authority event can reach this
subscriber before its own `/clock` callback has established a positive receive
time. Only during that initial state, events are retained in a bounded FIFO and
then passed, in arrival order, through the unchanged source/publication/receive
freshness and session/sequence gates at the first observed positive time. They
do not authorize fusion while pending. Once positive time has been observed, a
zero or rewound clock is not treated as a second startup. FIFO overflow rejects
all retained events and latches the authority unhealthy instead of silently
skipping an event.

On startup commit, `/diagnostics` also retains a separate immutable activation
evidence record: exact integer-nanosecond activation and authority timestamps,
including consumer receive time, authority session/sequence, stable-candidate
count and delta, plus the exact existing-global interpolation endpoints and
accepted input watermark. Session rearm retains the exact reset, unhealthy, and
healthy endpoints in the same way. Offline checks use these records and the raw
typed/input events rather than treating a later diagnostic snapshot as
commit-time health.
The current existing-global accounting endpoint is also diagnosed as exact
integer nanoseconds, avoiding epoch-sized floating-point serialization loss.

During outage, reacquisition, recovery, stale authority, stale existing-global
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
output. Re-arming requires a subsequently received, fresh, valid authority event that
explicitly reports non-tracking/unhealthy fusion, followed by a fresh
strict-tracking edge and three new stable candidates. Missing or stale
authority events never satisfy this handshake. In deployment,
restart `pure_gnss_map_odom_fusion` whenever the odometer is restarted so that
it explicitly reports `uninitialized`/non-tracking before returning to
tracking. If that transition is not observed, precision-global remains silent
by design.

The legacy position-only global-output fallback remains disabled by default
(`fallback.gnss_position_enabled=false`). Its gated position-alignment estimator
is shared only as the outage guard's trusted yaw observer; it is never mixed
into the authoritative existing-fusion anchor or global XY.
