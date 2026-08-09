# Migration to gicp_gnss_odom_localizer 0.3

## Scope

The project/repository display name changed from `mapless_iv_localizer` to `gicp_gnss_odom_localizer`. This release intentionally does **not** prefix every internal ROS package with `gicp_gnss_odom_`.

## ROS package identities in 0.3.0-rc6

The repository still does **not** prefix every package with `gicp_gnss_odom_`. RC6 renames the NMEA package and retires the optional intensity-filter package:

```text
pure_nmea_gga_conversion      -> pure_nmea_gnss_conversion
pure_snow_intensity_filter / pure_intensity_filter -> removed
```

The complete package set is now:

```text
pure_imu_undistortion
pure_lidar_gyro_odometer
pure_nmea_gnss_conversion
pure_gnss_msgs
pure_gnss_map_odom_fusion
pure_odometry_bringup
pure_autoware_localization_adapter
small_gicp
```

Perform a clean build because ament package resource names and install paths changed:

```bash
rm -rf build install log
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Update downstream `package.xml`, CMake, launch, and `ros2 pkg prefix` references to the new NMEA package name, and remove references to the retired intensity-filter package and `use_snow_filter` launch argument. These estimator runtime/source interfaces are intentionally retained:

- NMEA executable `pure_nmea_gga_conversion`;
- NMEA node and diagnostic name `nmea_gga_conversion`;
- NMEA C++ include root/namespace `pure_nmea_gga_conversion`;
- existing estimator topic names, frame names, launch argument names, YAML node keys, and parameter keys;
- custom message type `pure_gnss_msgs/msg/GnssFusionInput`.

The normal launch command remains:

```bash
ros2 launch pure_odometry_bringup odometry_container.launch.py use_gnss:=true
```

## From mapless_iv_localizer 0.2.0-rc3

Use the new repository/archive directory name and perform a clean build. Estimator source and numerical configuration are intentionally unchanged.

```bash
rm -rf build install log
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

No downstream source change is required solely because of the repository rename; RC6 still requires the NMEA package-dependency update and removal of intensity-filter references documented above.

## From the discarded fully-prefixed 0.3.0-rc2 artifact

That artifact renamed every package and changed the custom ROS message type. This corrected release restores the stable package identities above. Do not reuse its `build/`, `install/`, or `log/` directories.

Any temporary downstream changes that referenced names such as the following must be reverted:

```text
gicp_gnss_odom
gicp_gnss_odom_msgs/msg/GnssFusionInput
gicp_gnss_odom_fusion
gicp_gnss_odom_localizer (as a ROS bringup package)
```

Return those references to the corresponding `pure_*` package names. The repository itself remains named `gicp_gnss_odom_localizer`.

## Algorithm compatibility

This naming correction does not alter:

- scan-to-scan or scan-to-submap registration;
- fixed-lag SE(2) smoothing and continuous Hessian weighting;
- IMU deskew and corrected-yaw processing;
- NMEA position, covariance, and heading estimation;
- GNSS initialization, outage detection, reacquisition, or bounded recovery;
- public YAML numerical values.

## Additive Autoware integration in 0.3.0-rc4

`pure_autoware_localization_adapter` is a new package. No existing package or
custom message was renamed. Existing standalone launch commands continue to
work; the new `autoware_lsim_localization.launch.py` is invoked only for the
localization-only Autoware evaluation path.

## Dockerized Autoware LSim in 0.3.0-rc5

RC5 adds only deployment and evaluation files for the Autoware stage. It does
not rename a ROS package, change a custom message type, or alter estimator
parameters. The standalone rosbag workflow remains available.

The host-side entry point is:

```bash
./script/run_autoware_lsim_docker.sh --bag <bag> --points <points_topic> --imu <imu_topic>
```

The wrapper builds an overlay on the pinned CPU-only Autoware development image,
runs the localization-only logging-simulation profile, replays the bag, and
writes logs and an output rosbag below `docker_output/`. Existing native
Autoware workspaces are neither required nor modified by this Docker path.

## NMEA rename and intensity-filter removal in 0.3.0-rc6

RC6 changes the ament/ROS package identity and source-directory name for NMEA conversion and removes the unused optional intensity-filter package. Estimator algorithms, numerical YAML values, node names, topics, and component plugin types are unchanged. Recorded estimator topic interfaces and the NMEA runtime executable/include identities remain compatible.
