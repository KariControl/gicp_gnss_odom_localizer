# Known limitations

- The global fusion and local smoother are planar SE(2); this is not a full 3D
  inertial navigation system.
- There is no accelerometer bias/state propagation, gravity alignment, velocity
  state, or full IMU preintegration factor.
- There is no global loop closure or persistent map. Scan-to-submap uses a
  rolling local target and cannot eliminate unlimited long-duration drift.
- The submap and scan-to-scan observations are derived from overlapping LiDAR
  data and are statistically correlated. The implementation selects one primary
  relative factor per interval rather than treating both as independent, but it
  does not model all temporal map correlations.
- Only keyframes still represented in the fixed-lag optimized-pose window are
  individually repaired. Older retained keyframes move rigidly with the submap
  anchor.
- The active submap does not perform semantic or dynamic-object removal. Moving
  people, vehicles, doors, vegetation, rain, or snow can corrupt registration.
- Scan-to-submap currently evaluates scan-to-scan for prediction and fallback,
  so CPU cost can approach two registrations per frame. Target hardware timing
  has not been measured in this environment.
- Tracking mode is selected at startup; there is no validated live dynamic mode
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
and parameter files. Compare `scan_to_scan` and `scan_to_submap` on the same
reference trajectories before choosing a production default.
