# Known limitations

- The global fusion and local smoother are planar SE(2); this is not a full 3D
  inertial navigation system.
- There is no accelerometer bias/state propagation, gravity alignment, velocity
  state, or full IMU preintegration factor.
- There is no global loop closure or persistent map. The isolated scan-to-submap
  matcher uses a rolling local target and cannot eliminate unlimited
  long-duration drift.
- The external submap is built from accepted scan-to-scan snapshots, so its
  observations remain temporally correlated with the raw local trajectory. The
  robust commit gates outliers but does not model all map correlations.
- The external submap does not perform semantic or dynamic-object removal.
  Moving people, vehicles, doors, vegetation, rain, or snow can corrupt its
  registration.
- Repeated rolling-submap corrections can increase native-sample path length
  during stationary or near-stationary periods even when position and yaw RMSE
  improve. Treat path-ratio and low-motion jitter as separate acceptance gates;
  do not infer a clean stop from aggregate RMSE alone.
- Scan-to-submap mode adds accepted-cloud serialization/transport and another
  registration process. Target-hardware timing and scheduling interference must
  be measured even though the estimator states are isolated.
- `pure_lidar_gyro_odometer` supports only scan-to-scan registration.
  Scan-to-submap mode is selected by bringup and is not a validated live
  transition.
- GNSS covariance is modeled from configurable GGA quality/HDOP tables; GGA
  itself does not carry a full covariance matrix.
- Trajectory heading estimates direction of motion. Side slip, reverse motion,
  near-zero motion, and curved low-baseline motion require careful gating and
  may not equal body yaw.
- Corrected-IMU heading propagation remains dead reckoning and is deliberately
  time-limited.
- Dual-antenna covariance neglects some correlation introduced by deriving
  position and yaw from the same two fixes.
- Deskew translation uses forward speed only; it is not full 6-DoF motion
  compensation.
- The optional snow filter and legacy wheel/control-shaping paths require
  independent deployment-specific validation.
- The Autoware adapter derives acceleration by differentiating fused twist. It is
  bounded and filtered but is not a calibrated accelerometer measurement.
- The supplied Autoware launch validates localization topics and TF only. It does
  not implement every localization initialization/status API needed for full
  planning and control integration.
- The localization-only Autoware profile deliberately disables map, standard
  localization, perception, planning, control, system, and API modules. A prior
  PCD is not needed for that profile, but this does not make standard Autoware
  planning mapless.
- Bag replay must isolate recorded localization outputs and competing dynamic TFs.
  Incorrect TF policy can make a valid estimator appear to jump or can hide its
  output behind recorded transforms.
- No local run in a non-ROS environment can replace `colcon build`, ROS message
  generation, component loading, TF integration tests, and rosbag regression.

Before field deployment, perform a clean ROS build, unit tests, launch smoke
tests, malformed-input tests, and bag validation using the exact sensor drivers
and parameter files. Compare scan-to-scan and isolated scan-to-submap outputs
on the same reference trajectories, including the accepted-scan non-intrusion
gates.
