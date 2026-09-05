# GNSS initialization and outage recovery

## State machine

- `UNINITIALIZED`: no accepted `map -> odom` anchor.
- `TRACKING`: normal covariance-aware GNSS updates.
- `OUTAGE`: good GNSS has exceeded the configured timeout or quality gates fail.
- `REACQUIRING`: collect synchronized, contiguous GNSS/odom candidates.
- `RECOVERING`: approach an accepted recovered anchor with bounded corrections.
- `RECOVERING_XY_ONLY`: approach a position-only target while retaining the
  existing map-to-odom yaw and its uncertainty.
- `TRACKING_XY_ONLY`: fuse position in a degraded mode while yaw remains dead
  reckoned.

## Multi-sample alignment

Each candidate stores odom base position, map observation position converted to a base-consistent point model, position weight, and optional direct map-to-odom yaw. Weighted planar rigid alignment estimates translation and yaw.

Yaw is observable from:

- a minimum number of consistent direct heading samples;
- sufficient odometry/GNSS baseline;
- or both, provided they agree.

When only position observations are available during initialization, the node
waits for enough corresponding GNSS and odometry motion to estimate
map-to-odom yaw. Otherwise it requires enough independent, consistent heading
samples before accepting an anchor.

A window fails for non-finite data, timestamp gaps, insufficient samples, unobservable yaw, excessive heading dispersion, heading-source disagreement, excessive RMS, or excessive maximum residual. One isolated candidate may be removed only when the remaining window passes every gate.
A single remaining fix is never sufficient for alignment.

Full SE(2) recovery is always evaluated first. If it becomes observable while
XY-only recovery is active, the node promotes the same candidate window to the
normal `RECOVERING` path.

## Guarded XY-only recovery

A stopped single-antenna vehicle can regain a high-quality RTK position without
having either an absolute heading or enough motion to estimate yaw. Optional
XY-only recovery handles that case without inventing a yaw observation. It is
disabled in the generic parameter file and must be enabled explicitly with
`xy_only_recovery.enabled` through an explicit GNSS-fusion override. The Hesai
single-antenna evaluation profile uses
[`gnss_fusion_single_antenna.yaml`](../src/pure_localization_evaluation_profiles/config/odometry/lidar_imu_gnss/hesai_32line_rtk/accepted/gnss_fusion_single_antenna.yaml),
passed with the bringup argument `gnss_fusion_override_param`.

The default-false `fusion_xy_only_recovery` launch argument remains only as a
deprecated compatibility switch. Bringup applies it before
`gnss_fusion_override_param`, so an explicit override YAML has final precedence.
New deployment and evaluation profiles should use the override YAML instead of
the compatibility argument.

The fallback is considered only when full alignment fails specifically because
yaw is unobservable. The default gates additionally require:

- the configured RTK fix quality (quality 4 by default) and bounded XY
  covariance;
- a contiguous, positionally consistent candidate window;
- a complete one-second odometry window below the speed, yaw-rate,
  displacement, yaw-change, and sample-gap limits;
- a valid antenna observation point, bounded outage duration, bounded retained
  yaw variance, and bounded lever-arm error caused by that yaw uncertainty.

The correction changes translation only. It keeps the map-to-odom rotation
bit-for-bit unchanged, applies no yaw gain, and never reduces yaw covariance.
The state remains explicitly `*_XY_ONLY`, and diagnostics remain WARN because
position is fused while yaw is dead reckoned. Motion suspends this fallback;
normal SE(2) recovery must then observe yaw before full `TRACKING` resumes.
Published XY covariance also carries the retained yaw uncertainty through the
distance travelled from the latest XY reference, so movement after a
position-only correction cannot appear globally well localized.

A brief usable-position dropout may preserve the recovery window for
`xy_only_recovery.soft_bad_grace_sec`. A hard no-fix status, invalid observation
semantics, excessive sample gap, or timeout still clears the window and enters
`OUTAGE`.

This grace applies only to the map/odom fusion node's internal recovery window.
Its typed `/localization/gnss_map_odom_fusion_authority` output immediately
reports `SOFT_BAD_HOLD`, which is unhealthy for downstream anchor updates.
Precision-global therefore freezes its last trusted anchor and consumes no new
authority candidate until a good full-SE(2) tracking event returns. Authority
transitions caused by GNSS/odometry input are published in those callbacks.
The periodic path evaluates clock-driven timeouts and republishes the resulting
state for liveness; it does not sample or postpone short input-driven
GNSS-quality transitions.

Under `use_sim_time`, downstream authority consumers may receive the first
transient-local event before their own ROS clock has processed its first
positive `/clock` sample. Such startup-only events are held in a bounded FIFO,
remain non-authoritative, and are evaluated in arrival order with the first
positive consumer receive time. The ordinary timing and session/sequence gates
are not relaxed. An authority event received after a later zero/rewound clock is
not deferred again: it passes through the ordinary timing and ordering gates,
which reject invalid, stale, or backstepping events. A FIFO overflow fails closed
and latches authority unhealthy.

## Normal tracking

Position and yaw use separate innovation tests. A rejected yaw can be omitted while retaining a valid position update. Measurement covariance includes lever-arm Jacobians and cross-covariance. Absolute jump gates supplement NIS gates.

## Bounded recovery

After accepting a target `map -> odom`, the node limits translation and yaw corrections by both per-update and elapsed-time rates. Refreshed sliding-window targets must remain close to the current target. Recovery exits only after the residual remains below position/yaw thresholds for multiple accepted samples.

Target covariance is propagated independently from the temporary squared
residual of a bounded correction. The published covariance contains both while
the transform is converging, but only the target covariance is carried into
`TRACKING`; an unapplied residual is never fed back into the next target.

Pose, odometry, and TF output sets are serialized. A source-stamped odometry
request older than the last successfully published timestamp is dropped and
counted in `output.out_of_order_drop_count`, so output timestamps are
non-decreasing. The wall timer retries only the newest completely processed raw
odometry stamp; it never advances the output header to ROS `now()`. This source
watermark prevents a delayed odometry result from being overtaken when `/clock`
has already advanced beyond that result's physical stamp. A timer retry at or
behind the published watermark is suppressed and reported separately as
`output.wall_timer_coalesced_count`. A duplicate odometry request whose exact
physical stamp is already present is reported as
`output.covered_odometry_coalesced_count`. Only an older odometry stamp absent
from the bounded publication history increments the strict
`output.out_of_order_drop_count`.
Consequently, fused pose/odometry/TF output is source-odometry driven; the
`publish_rate_hz` timer retries an output only when the newest completed source
stamp has not yet been published. Liveness is carried independently by typed
fusion-authority and diagnostic heartbeats. A fixed-rate forward prediction, if
required by another integration, must use a separate topic so it cannot violate
this exact-source/non-decreasing contract.
Clock rewind, rosbag loop, and seek without restarting the node are not
supported because all time-indexed estimator buffers would need a coordinated
reset.

There is no path that force-accepts one RTK Fix after a timeout. Deprecated parameters with names such as `gnss_force_accept_*` are parsed only so old YAML files fail gracefully; they have no effect.

## NMEA heading seed quality

Trajectory headings use only fixes whose finite position sigma is below
`trajectory_heading_max_position_sigma_m`. A generated heading whose variance
exceeds `trajectory_heading_max_yaw_variance_rad2` is published as
position-only and does not replace a previous valid heading seed. This prevents
float/standalone fixes with very large configured covariance from contaminating
later corrected-IMU propagation.
