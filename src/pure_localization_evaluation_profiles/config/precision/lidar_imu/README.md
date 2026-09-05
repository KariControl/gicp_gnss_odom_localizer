# Precision overrides for LiDAR/IMU evaluation

These archived overrides document the isolated scan-to-submap A/B
configuration. They are evaluation-only and do not select a production
localization output.

- `accepted_scan_local_only.yaml` consumes causal accepted-scan odometry and
  disables GNSS fallback authority for the local-only evaluation.
- `external_snapshot_i2.yaml` publishes every second accepted scan for the
  canonical interval-2 experiment.

The `pure_precision_bringup` package's `submap_snapshot_override.yaml` remains
the production interval-5 profile, and its `empty_params.yaml` remains the
neutral launch default. Neither file is replaced by the evaluation overrides
here.
Do not reuse the interval-2 experiment as a deployment default. Select a
snapshot interval from measured motion, scene overlap, and runtime capacity.
