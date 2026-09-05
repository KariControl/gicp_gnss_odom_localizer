# LiDAR/IMU evaluation profiles

These archived parameter overrides document the configurations used for the
published LiDAR/IMU evaluations. They are not production defaults and are not,
by themselves, sufficient to reproduce results without the source recording
and reference.

Profiles are grouped by neutral sensor configuration and status:

- `mid360/internal_imu_evaluation/accepted` contains the selected MID-360
  overrides.
- `velodyne/external_imu_evaluation/accepted` contains the selected Velodyne
  overrides.
- The corresponding `experimental` directories contain sensitivity profiles.

`accepted` describes dataset-specific profile selection; it does not make the
values a universal sensor calibration or production default.

The accepted MID-360 profile uses the internal `/livox/imu` stream. Its raw
acceleration is converted with scale 9.80665, the configured base-to-`livox_frame`
transform is identity, and GNSS and recorded TF are disabled. The odometer uses
the recording-specific fixed gyro-Z bias 0.006959692 rad/s, disables adaptive
bias updates, the fixed-lag smoother, and ZUPT, and retains the safe stop
thresholds of 0.15 m/s for 0.5 s. Precision mode emits a snapshot every five
accepted scans and loads the selected rolling-submap matcher override.

For the documented MID-360 evaluation,
`mid360/internal_imu_evaluation/accepted/odom_tuned.yaml` contains the selected
scan-to-scan override and
`mid360/internal_imu_evaluation/accepted/submap_matcher_tuned.yaml` contains
the selected matcher override. Recalibrate and revalidate the fixed bias before
using it with another recording, temperature, startup condition, or IMU. The
Velodyne IMU-gap limits likewise describe only their audited recording.

Treat these files as published evaluation provenance, not as a ready-to-use
sensor profile. Copy only the parameters you understand into a
deployment-owned override, then recalibrate and validate them against that
deployment's own reference data.
