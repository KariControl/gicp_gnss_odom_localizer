# Evaluation Methodology

This document defines the common rules used to interpret the published metrics
and plots. Current LiDAR/IMU and LiDAR/IMU/GNSS results use one common
exact-initial-pose rule.

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

The published results use exact-initial-pose alignment. The first physical
timestamp shared by the baseline and comparison streams is the anchor. A
single transform maps the baseline pose at that timestamp exactly onto the
reference pose.

For initial control position `p0`, initial control yaw `yaw0`, reference
position `p_ref0`, and reference yaw `yaw_ref0`:

```text
theta = yaw_ref0 - yaw0
t     = p_ref0 - R(theta) * p0
p'    = R(theta) * p + t
yaw'  = yaw + theta
```

The same transform is fixed for both A/B streams. The precision output is not
fitted independently, so subsequent drift and correction differences remain in
the score. For GNSS evaluation, local and global pairs use separate anchors
because they are different coordinate streams, but each pair shares exactly
one transform. Earlier exploratory full-trajectory, first-20-metre, and frozen
calibration-window fits are not used for published RMSE values or acceptance
decisions.

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
