# Public synthetic LiDAR/IMU fixture

[`synthetic_output_pointcloud2`](synthetic_output_pointcloud2/) is a compact,
fully procedural ROS 2 bag for source-build and localization smoke tests. It is
not a recording, a sensor benchmark, or evidence of real-world accuracy.

The private `output_pointcloud2` recording was consulted only for its ROS
interface shape: the PointCloud2 field names, offsets, datatypes,
`point_step`, and rounded 20 Hz LiDAR / 200 Hz IMU rates. The generator cannot
read an input bag. It does not copy or transform source points, trajectories,
GNSS, timestamps, calibration, distributions, or scene geometry.

## Contents

| Topic | Type | Count | Synthetic role |
|---|---|---:|---|
| `/pandar_points_ex` | `sensor_msgs/msg/PointCloud2` | 121 | Analytic asymmetric corridor, 20 Hz, `lidar/0` |
| `/sensor/imu/data_raw` | `sensor_msgs/msg/Imu` | 1,241 | Analytic circular-arc motion, 200 Hz, `imu` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 1 | Identity `base_link -> lidar/0` and `base_link -> imu` transforms |
| `/synthetic/ground_truth` | `nav_msgs/msg/Odometry` | 121 | Analytic reference pose for smoke-test gating only |

All publishers are recorded as reliable. `/tf_static` is additionally
transient-local with depth 1; the other topics are volatile with depth 10.

The 48-byte point layout is:

| Field | Offset | Datatype | Meaning |
|---|---:|---|---|
| `x`, `y`, `z` | 0, 4, 8 | `FLOAT32` | Procedural Cartesian coordinates |
| `intensity` | 16 | `FLOAT32` | Procedural surface class |
| `ring` | 20 | `UINT16` | Synthetic 0–31 elevation bin |
| `azimuth`, `distance` | 24, 28 | `FLOAT32` | Values recomputed from synthetic XYZ |
| `return_type` | 32 | `UINT8` | Fixed synthetic return code |
| `time_stamp` | 40 | `FLOAT64` | Scan-relative seconds from 0.0 to 0.05 |

The bag uses a fixed, non-identifying ROS time origin below 10 seconds. It
contains no NMEA, GNSS position, raw packet, hostname, absolute filesystem
path, capture date, or real capture epoch. The exact file hashes, semantic
stream hash, rates, schema, and provenance flags are recorded in
[`manifest.json`](synthetic_output_pointcloud2/manifest.json).

## Validate and replay

The smoke tooling uses `ros2bag`, `rosbag2_py`, and the MCAP storage plugin;
they are declared as test dependencies of `pure_odometry_bringup`. After
sourcing ROS 2 Jazzy and this workspace:

```bash
python3 tools/check_synthetic_output_pointcloud2.py

ros2 launch pure_odometry_bringup lidar_imu_only.launch.py \
  use_sim_time:=true \
  use_imu_deskew:=true \
  points_input_topic:=/points_raw \
  imu_input_topic:=/imu
```

In a second sourced terminal:

```bash
./script/play_localization_bag.sh \
  --bag data/synthetic_output_pointcloud2 \
  --points /pandar_points_ex \
  --imu /sensor/imu/data_raw \
  --clock-frequency 100 \
  --tf-policy isolate-dynamic \
  -- --disable-keyboard-controls \
  --topics /pandar_points_ex /sensor/imu/data_raw /tf_static
```

Do not use `--tf-policy isolate-all`: the fixture's static sensor transforms
are required by the fail-closed deskewer and odometer.

For a self-contained launch, replay, recording, and validation run:

```bash
./script/run_synthetic_lidar_imu_smoke.sh
```

The maintained smoke gates require all 121 clouds to deskew from the
`time_stamp` field without fallback, at least 99% corrected-IMU coverage, at
least 95% accepted registrations after initialization, a healthy final
registration, and a coarse endpoint error below 0.5 m / 2 deg. The current
Jazzy run produced 121/121 successful deskews, 1,241/1,241 corrected IMU
samples, 118/120 accepted registrations, and 0.0667 m / 0.359 deg endpoint
error. These values are regression evidence for this artificial scene only.

## Regenerate

The generator refuses to overwrite any existing output directory:

```bash
python3 tools/generate_synthetic_output_pointcloud2.py \
  --output /tmp/synthetic_output_pointcloud2
python3 tools/check_synthetic_output_pointcloud2.py \
  --bag /tmp/synthetic_output_pointcloud2
```

Generation zeroes non-semantic CDR alignment padding and verifies that
deserialization is unchanged. The canonical semantic stream hash excludes CDR
padding; the per-file SHA-256 values pin the committed MCAP and metadata.
The fixture and generator are licensed under the repository's Apache-2.0
license.
