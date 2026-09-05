# Hesai 32-Line + IMU + RTK GNSS evaluation profile

This directory contains shared deployment overrides and dataset contracts for
Hesai 32-Line + IMU + RTK GNSS evaluation.

The committed accuracy artifact uses an effective odometer profile with
`gyro_bias.initial_bg_rad_s=-0.00210` (`rad/s`), adaptive gyro-bias estimation and
the fixed-lag smoother enabled, and ZUPT/NHC disabled. The global path used the
typed fusion-authority startup contract. All 55/55 GLIM A/B hard gates passed;
the result remains specific to the documented recording and effective files.

The rosbag files are private and are not distributed with this repository. The
dataset contracts provide descriptive identities, topic contracts, reference
metadata, and audited static transforms; they contain no sensor data or local
filesystem paths.

The recordings use one calibrated sensor rig, so parameter files are not
duplicated per recording. The NMEA conversion keeps the projection in
`pure_nmea_gnss_conversion/param/param.yaml`, which mirrors
`pure_nmea_gnss_conversion/config/map_projector_info.yaml`; the evaluation does
not override its map origin. `accepted/gnss_fusion_single_antenna.yaml`
explicitly enables the guarded position-only recovery profile used by this
single-antenna rig.

Component defaults remain in their owning packages and are loaded first:

- `pure_imu_undistortion/param/param_xt.yaml`
- `pure_lidar_gyro_odometer/param/param_xt_lidar_imu_only.yaml`
- `pure_nmea_gnss_conversion/param/param.yaml`
- `pure_gnss_map_odom_fusion/param/param.yaml`

The GNSS fusion file under `accepted/` is an evaluation override. It must not
be treated as a generic default for another sensor rig.
