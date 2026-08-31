# Precision recording validator

`validate_precision_bag.py` validates the message and timing contract of a
recorded isolated scan-to-submap run. It is repository tooling and is not
installed by `pure_precision_bringup`, so normal localization deployments do
not inherit rosbag Python dependencies.

On ROS 2 Jazzy/Ubuntu 24.04, install the optional dependencies with:

```bash
sudo apt install \
  python3-numpy \
  ros-jazzy-rosbag2-py \
  ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-rosidl-runtime-py
```

Run it against the output bag from a precision replay:

```bash
python3 tools/evaluation/precision/scripts/validate_precision_bag.py \
  /path/to/localization_output \
  --expected-rate 1.0
```

The validator checks exact-key accounting, publication order, authority
freshness, startup state, session resets, and shutdown completeness. It does
not measure trajectory accuracy or replace an independent reference.

Run its dependency-light contract tests with:

```bash
python3 -B tools/evaluation/precision/test/test_validate_precision_bag.py
```
