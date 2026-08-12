# Precision overrides for LiDAR/IMU evaluation

These overrides support the isolated external scan-to-submap A/B runner. They
are evaluation-only and do not select a production localization output.

- `accepted_scan_local_only.yaml` consumes causal accepted-scan odometry and
  disables GNSS fallback authority for the local-only evaluation.
- `external_snapshot_i2.yaml` publishes every second accepted scan for the
  canonical interval-2 experiment.

The package-root `submap_snapshot_override.yaml` remains the production
interval-5 profile. The package-root `empty_params.yaml` remains the neutral
launch default. Neither file is replaced by the evaluation overrides here.
The LiDAR/IMU runner uses interval 2 for its canonical precision A/B unless
an interval is explicitly selected; this does not change the production
interval-5 profile.
