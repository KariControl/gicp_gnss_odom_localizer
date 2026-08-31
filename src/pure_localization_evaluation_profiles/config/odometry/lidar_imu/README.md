# LiDAR/IMU evaluation profiles

These parameter files reproduce the isolated LiDAR/IMU evaluations. They are
evaluation inputs, not production defaults. The reference trajectory provider
is part of the run manifest rather than this directory name.

Profiles are grouped by LiDAR model, input bag, and evaluation status:

- `accepted/` contains the canonical profile selected for the reported A/B.
- `experimental/` contains sensitivity or superseded profiles retained for
  reproducibility.

`accepted` describes dataset-specific profile selection; it does not make the
values a universal sensor calibration or production default.

The accepted MID-360 profile uses the internal `/livox/imu` stream. Its raw
acceleration is converted with scale 9.80665, the configured base-to-`livox_frame`
transform is identity, and GNSS and recorded TF are disabled. The odometer uses
the recording-specific fixed gyro-Z bias 0.006959692 rad/s, disables adaptive
bias updates, the fixed-lag smoother, and ZUPT, and retains the safe stop
thresholds of 0.15 m/s for 0.5 s. Precision mode emits a snapshot every five
accepted scans and loads the selected rolling-submap matcher override.

For the canonical MID-360 recording, `accepted/odom_tuned.yaml` contains the
selected scan-to-scan override and `accepted/submap_matcher_tuned.yaml` contains
the selected matcher override. Recalibrate and revalidate the fixed bias before
using it with another recording, temperature, startup condition, or IMU. The
Velodyne IMU-gap limits likewise describe only their audited recording.

Historical result directories retain their original `run.env`, copied YAML,
and SHA-256 values. This directory layout governs new runs only.

The MID-360 `experimental/tuning/plan.yaml` defines a fixed 48-candidate search
across nine stages with preregistered training/holdout blocks.
`tools/tune_mid360_lidar_imu.py` executes it serially, preserves source and
configuration provenance, and prunes rejected candidate bags only after their
metrics and deletion receipts are durable. The holdout remains unopened until
the stages and final settings are locked, and post-holdout retuning is
forbidden. Every stop-detector candidate missed the classifier gates, so the
workflow retained the safe existing thresholds and required ZUPT to remain
off. The selected full-rate A/B passed all 20 formal hard gates.

`script/run_lidar_imu_glim_bag.sh` selects the `accepted` profile for each
canonical default bag. MID-360 `--bag` overrides require explicit
`--mid-yaw-policy` and `--glim-dir` arguments, so neither a bag-specific bias
seed nor an unrelated reference can be selected silently. Explicit runner
overrides take precedence over the accepted defaults; experimental profiles
are selected only by explicit options.
