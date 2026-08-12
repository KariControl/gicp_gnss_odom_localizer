# NMEA position, covariance, and heading

## Position

`pure_nmea_gnss_conversion` accepts GGA sentences, validates checksum when enabled, maps GGA quality to `NavSatStatus`, and projects latitude/longitude with the configured Transverse Mercator origin. Altitude can use ellipsoid height derived from GGA altitude plus geoid separation.

## Covariance and confidence

Horizontal sigma is selected from `fix_sigma_xy_table_m` by GGA quality, scaled by bounded HDOP, and inflated or invalidated according to differential-correction age. Confidence combines `fix_confidence_table`, HDOP score, and correction-age scale. Vertical sigma is `vertical_sigma_scale * sigma_xy`.

These values are configurable receiver models, not receiver-reported covariance. They must be calibrated against data from the actual GNSS receiver, antenna, environment, and correction service.

## Heading priority

1. **Dual antenna:** map baseline minus calibrated base-frame baseline. Both lever arms, minimum baseline, and baseline-length consistency are required.
2. **Doppler:** course over ground from `TwistWithCovarianceStamped`, accepted only over the minimum speed and under speed/heading uncertainty gates.
3. **Trajectory:** direction between quality-gated GGA positions separated by a minimum baseline. Yaw uncertainty is derived from endpoint position uncertainty divided by baseline, with a configurable floor.
4. **Trajectory + corrected IMU:** when `/localization/imu_corrected` strictly covers the trajectory interval, half the integrated yaw change converts the chord direction toward an interval tangent and its uncertainty is added.
5. **Bounded corrected-IMU propagation:** a recent valid absolute heading may be propagated for at most `imu_yaw_rate_max_integration_sec` with growing covariance.

Corrected IMU angular velocity is never an absolute-heading source by itself. It requires a previously established dual-antenna, Doppler, or trajectory heading.
The NMEA node consumes only the corrected `angular_velocity.z`; it does not fuse the IMU orientation quaternion and does not estimate roll or pitch.

Both endpoints of a trajectory chord must pass the confidence gate and have a
finite sigma no larger than `trajectory_heading_max_position_sigma_m`. The
resulting yaw variance must not exceed
`trajectory_heading_max_yaw_variance_rad2`. A chord that fails the latter gate
remains a position-only observation and cannot overwrite the last valid
absolute-heading seed.

The supplied single-antenna parameter file uses a 1.5 m trajectory baseline;
the C++ fallback remains 3.0 m. A shorter baseline can restore full SE(2)
heading sooner after GNSS returns, but it is more sensitive to position noise
and low-speed path curvature. Tune it for the receiver and site, and retain the
endpoint-sigma, yaw-variance, age, and confidence gates. If those conditions do
not produce a trustworthy heading, the fusion node must remain position-only
or use its separately guarded XY-only recovery mode.

## IMU acceptance

The corrected-IMU buffer rejects zero, non-finite, over-range, reordered samples. Duplicate timestamps replace rather than double-integrate. Integration fails when samples do not bracket the interval, an internal gap exceeds `imu_yaw_rate_max_sample_gap_sec`, or a boundary is farther than `imu_yaw_rate_max_boundary_gap_sec`.

## Output semantics

`/localization/gnss_fusion_input` is authoritative. Use `heading_valid`,
`position_is_base_link`, and `observation_point_valid`; never inspect a
placeholder quaternion to infer validity.

A valid GGA fix without a valid yaw remains a usable position observation. It
is published with `heading_valid=false` and `position_is_base_link=false`. When
the calibrated antenna transform is available,
`observation_point_in_base` contains the antenna lever arm and
`observation_point_valid=true`, allowing fusion to model the antenna position
without pretending that it is the vehicle origin.

The generic configuration keeps `allow_parameter_antenna_fallback=false`.
A calibrated TF is therefore required unless a deployment explicitly opts into
and validates parameter-based antenna geometry.

Legacy global-pose, pose-with-covariance, and GNSS-odometry outputs represent
`base_link` and are withheld until both yaw and required antenna geometry are
valid. This prevents an antenna coordinate with an identity placeholder
quaternion from being mislabeled as a `base_link` pose.
