#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/.." && pwd)"
bag_dir="$repo_root/data/synthetic_output_pointcloud2"
work_dir="$(mktemp -d /tmp/pure-odom-synthetic.XXXXXX)"
result_dir="$work_dir/recorded"
# Choose an isolated domain by default so a stale or concurrent ROS graph cannot
# duplicate the fixture. An explicit caller-provided ROS_DOMAIN_ID still wins.
synthetic_domain_id="${ROS_DOMAIN_ID:-$((50 + BASHPID % 150))}"

command -v ros2 >/dev/null 2>&1 || {
  echo "ros2 is unavailable; source /opt/ros/jazzy/setup.bash first" >&2
  exit 1
}
command -v timeout >/dev/null 2>&1 || {
  echo "GNU timeout is required" >&2
  exit 1
}
ros2 pkg prefix pure_odometry_bringup >/dev/null 2>&1 || {
  echo "pure_odometry_bringup is unavailable; source install/setup.bash first" >&2
  exit 1
}

export ROS_DOMAIN_ID="$synthetic_domain_id"
export ROS_LOG_DIR="$work_dir/ros_logs"
mkdir -p "$ROS_LOG_DIR"

launch_pid=""
recorder_pid=""
cleanup() {
  if [[ -n "$recorder_pid" ]]; then
    kill -TERM "$recorder_pid" 2>/dev/null || true
    wait "$recorder_pid" 2>/dev/null || true
  fi
  if [[ -n "$launch_pid" ]]; then
    kill -TERM "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

python3 "$repo_root/tools/check_synthetic_output_pointcloud2.py" --bag "$bag_dir"

timeout --signal=INT --kill-after=5 22 \
  ros2 launch pure_odometry_bringup lidar_imu_only.launch.py \
  use_sim_time:=true \
  use_imu_deskew:=true \
  points_input_topic:=/points_raw \
  imu_input_topic:=/imu \
  log_level:=warn >"$work_dir/launch.log" 2>&1 &
launch_pid=$!
sleep 3

timeout --signal=INT --kill-after=5 16 \
  ros2 bag record --storage mcap --output "$result_dir" --topics \
  /diagnostics \
  /localization/gyro_lidar_odom \
  /localization/imu_corrected >"$work_dir/record.log" 2>&1 &
recorder_pid=$!
sleep 2

"$repo_root/script/play_localization_bag.sh" \
  --bag "$bag_dir" \
  --points /pandar_points_ex \
  --imu /sensor/imu/data_raw \
  --clock-frequency 100 \
  --tf-policy isolate-dynamic \
  -- --disable-keyboard-controls --delay 1 \
  --topics /pandar_points_ex /sensor/imu/data_raw /tf_static \
  >"$work_dir/play.log" 2>&1

wait "$recorder_pid" || true
recorder_pid=""
wait "$launch_pid" 2>/dev/null || true
launch_pid=""
trap - EXIT INT TERM

test -f "$result_dir/metadata.yaml" || {
  echo "runtime recording did not finish cleanly; inspect $work_dir" >&2
  exit 1
}
python3 "$repo_root/tools/check_synthetic_output_pointcloud2.py" \
  --bag "$bag_dir" \
  --runtime-result "$result_dir"
echo "Smoke-test artifacts: $work_dir"
