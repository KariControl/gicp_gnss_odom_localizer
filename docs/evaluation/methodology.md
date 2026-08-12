# Evaluation Methodology

This document defines the common rules used to interpret the published metrics
and plots. Two trajectory-alignment methods exist in the current evidence, so
every result must identify its alignment method.

## Reference trajectory and timestamp matching

- GLIM provides the reference trajectory. It uses the same LiDAR and IMU
  observations as the evaluated estimator, so it is a correlated pseudo-ground
  truth rather than independent absolute ground truth.
- Estimator poses use the physical ROS message `header.stamp`; MCAP record time
  is not treated as the pose timestamp.
- Estimator poses are not interpolated. Only the reference trajectory is
  interpolated to each evaluated timestamp.
- No scale is estimated or corrected. Evaluation is planar SE(2): XY and yaw
  are scored, while Z, roll, and pitch are outside the acceptance scope.

## Alignment A: exact initial pose

The LiDAR/IMU-only results use exact-initial-pose alignment. The first physical
timestamp shared by the control and comparison streams is the anchor. A single
transform maps the control pose at that timestamp exactly onto the reference
pose.

For initial control position `p0`, initial control yaw `yaw0`, reference
position `p_ref0`, and reference yaw `yaw_ref0`:

```text
theta = yaw_ref0 - yaw0
t     = p_ref0 - R(theta) * p0
p'    = R(theta) * p + t
yaw'  = yaw + theta
```

The same transform is fixed for every A/B stream. The precision output is not
fitted independently, so subsequent drift and correction differences remain in
the score. Earlier exploratory full-trajectory and first-20-metre fits are not
used for published RMSE values or acceptance decisions.

## Alignment B: frozen calibration window

The current LiDAR/IMU/GNSS plots use frozen calibration-window alignment. A
scale-fixed SE(2) transform is fitted once between the scan-to-scan control and
the GLIM reference over an initial calibration window, then held fixed for all
local A/B streams. A separate transform is derived from the existing-fusion
control and held fixed for all global A/B streams. The local and global yaw
offsets are also derived from their respective controls and shared by the
comparison streams.

This prevents an independent precision-only refit and keeps each A/B comparison
fair. Because the method minimizes error over a window, the first plotted XY
and yaw errors are not necessarily zero. Absolute RMSE from this alignment must
not be compared directly with exact-initial-pose RMSE without stating the
alignment difference.

| Property | Exact initial pose | Frozen calibration window |
|---|---|---|
| Current use | LiDAR/IMU-only datasets | Hesai 32-Line + IMU + RTK GNSS — Course 1 and Course 2 |
| Transform source | One initial control pose | Least-squares fit of the control over an initial window |
| Shared across A/B streams | Yes | Yes |
| Scale correction | None | None |
| Initial plotted error | Zero by construction | Not necessarily zero |
| Independent precision refit | Prohibited | Prohibited |

If a single public alignment rule is required, the Hesai local and global
metrics and plots must be regenerated from the first common physical timestamp
with a control-derived exact SE(2) transform. Existing frozen-window values must
not simply be renamed.

## Metrics

| Metric | Definition and purpose |
|---|---|
| XY RMSE | Root mean squared horizontal position error after the shared alignment |
| Yaw RMSE | Root mean squared wrapped absolute yaw error |
| p95 | Upper-tail behavior of the error distribution |
| Endpoint XY | Horizontal error at the end of the common evaluation interval |
| Path ratio | Estimated path length divided by reference path length |
| RPE at 10/50/100 m | Relative translation and yaw error over fixed path lengths |
| Full-shape XY | Diagnostic independent SE(2) shape fit; never a substitute for shared-alignment RMSE |

RMSE alone is not an acceptance decision. Timestamp coverage, accepted-scan
non-intrusion, queue drops, frames, finite values, initialization, GNSS outage,
and recovery behavior are hard gates where applicable.

## Runtime, latency, CPU, and memory

Current evidence verifies 1.0x playback, counter consistency, queue drops,
matcher processing time, and end-to-end latency. **CPU utilization and RSS were
not measured.**

`processing_p99_ms` is the wall-clock time required to process one matcher
request; it is not CPU utilization. `latency_p99_ms` measures message-stamp to
publication delay; it is also not CPU utilization. Neither value captures
aggregate multi-core CPU, scheduling interference with the base component
container, or memory growth.

A publishable CPU/RSS A/B measurement must fix the following conditions:

- the same host, power mode, CPU governor, OS, ROS release, build type, commit,
  and parameter hashes;
- the same recording at 1.0x, repeated at least three times with identical
  recording and monitoring overhead;
- separate accounting for the base component container, submap matcher,
  precision-global process, bag player, and recorder;
- per-process CPU mean/p95/max, RSS peak, host CPU, deadline misses, and queue
  drops;
- explicit container-level attribution when composable nodes are not split into
  separate processes.

## Evidence publication rules

- Tables are checked against curated machine-readable metrics.
- Figure titles or captions identify sensor configuration, course, output, and
  alignment method.
- Main pages show only trajectories needed to support the conclusion; error
  plots and metrics are linked.
- Exploratory profiles, smoke tests, obsolete alignments, and superseded runs
  are excluded from headline tables.
- Published manifests record source-run provenance, configuration hashes,
  generation method, and asset SHA-256 values without exposing private bag
  paths.
- Source rosbags and complete generated output directories are not distributed
  on GitHub.

