# Manual submap matcher harness

`synthetic_snapshot_matcher_e2e.py` is a manual ROS publisher/observer harness.
It publishes an already-deskewed synthetic cloud, then verifies exact-key
accepted-scan snapshots and matcher corrections. It is not registered as an
automated package test because the odometer and precision overlay must remain
running in separate terminals.

After building and sourcing the workspace, start the odometer with internal
deskew disabled and the accepted-scan snapshot override enabled:

```bash
ros2 launch pure_odometry_bringup odometry_standalone.launch.py \
  use_imu_deskew:=false \
  points_input_topic:=/localization/points_undistorted \
  odom_override_param:="$(ros2 pkg prefix pure_precision_bringup)/share/pure_precision_bringup/config/submap_snapshot_override.yaml"
```

In a second terminal, start the matcher and isolated precision compositor:

```bash
ros2 launch pure_precision_bringup precision_overlay.launch.py
```

Run the harness from the repository root in a third sourced terminal:

```bash
python3 tools/manual/submap_matcher/synthetic_snapshot_matcher_e2e.py
```

A successful run exits zero after observing at least four snapshots, at least
one correction, and no exact-key or corrected-pose consistency error.
