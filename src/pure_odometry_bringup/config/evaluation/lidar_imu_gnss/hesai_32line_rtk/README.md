# Hesai 32-Line + IMU + RTK GNSS evaluation profile

This directory contains the shared deployment overrides and private-dataset
manifests used by two evaluation courses:

- `course_1.yaml`: **Hesai 32-Line + IMU + RTK GNSS — Course 1**
- `course_2.yaml`: **Hesai 32-Line + IMU + RTK GNSS — Course 2**

The rosbag files are private and are not distributed with this repository. The
dataset manifests provide descriptive identities, local default paths, topic
contracts, reference metadata, and audited static transforms; they contain no
sensor data.

The two courses use one calibrated sensor rig and site, so parameter files are
not duplicated per course. `accepted/nmea_site_origin.yaml` defines the shared
local map origin. `accepted/gnss_fusion_single_antenna.yaml` explicitly enables
the guarded position-only recovery profile used by this single-antenna rig.

Component defaults remain in their owning packages and are loaded first:

- `pure_imu_undistortion/param/param_xt.yaml`
- `pure_lidar_gyro_odometer/param/param_xt_lidar_imu_only.yaml`
- `pure_nmea_gnss_conversion/param/param.yaml`
- `pure_gnss_map_odom_fusion/param/param.yaml`

The files under `accepted/` are layered on top by the evaluation runners. They
must not be treated as generic defaults for another site or sensor rig.
