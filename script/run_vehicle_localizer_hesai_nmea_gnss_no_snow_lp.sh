#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -Eeuo pipefail

usage() {
  cat <<'USAGE'
Run the ROS 2 LiDAR/IMU/NMEA localization stack against a Hesai rosbag.

Usage:
  run_vehicle_localizer_hesai_nmea_gnss_no_snow_lp.sh [options]

Options:
  --bag <path>                 Input rosbag directory
                               default: rosbag/output_pointcloud2
  --rate <factor>             Playback rate (default: 1.0)
  --output <directory>        Result directory
                               default: test_results/output_pointcloud2_<timestamp>
  --playback-duration <sec>   Stop after this many seconds of bag time
  --lsim-interface-test       Run the LSim localizer/Autoware adapter launch
                              without requiring the Autoware installation
  --no-record                 Run without recording localization outputs
  --dry-run                   Validate inputs and print commands only
  -h, --help                  Show this help

The default profile uses LiDAR and IMU only for local odometry. It neither
subscribes to /vehicle/twist nor supplies another translational deskew speed.
Static transforms are the calibrated values for rosbag/output_pointcloud2.
USAGE
}

fail() {
  printf '[vehicle-localizer] ERROR: %s\n' "$*" >&2
  exit 2
}

log() {
  printf '[vehicle-localizer] %s\n' "$*"
}

require_positive_number() {
  local name="$1"
  local value="$2"
  if ! [[ "$value" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] ||
    ! awk -v value="$value" 'BEGIN { exit !(value > 0.0) }'
  then
    fail "$name must be a positive number: $value"
  fi
}

print_command() {
  printf '[vehicle-localizer] command:'
  printf ' %q' "$@"
  printf '\n'
}

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bag="$ROOT/rosbag/output_pointcloud2"
rate="1.0"
output=""
playback_duration=""
record_output=true
lsim_interface_test=false
dry_run=false

while (($# > 0)); do
  case "$1" in
    --bag)
      [[ $# -ge 2 ]] || fail "--bag requires a value"
      bag="$2"
      shift 2
      ;;
    --rate)
      [[ $# -ge 2 ]] || fail "--rate requires a value"
      rate="$2"
      shift 2
      ;;
    --output)
      [[ $# -ge 2 ]] || fail "--output requires a value"
      output="$2"
      shift 2
      ;;
    --playback-duration)
      [[ $# -ge 2 ]] || fail "--playback-duration requires a value"
      playback_duration="$2"
      shift 2
      ;;
    --lsim-interface-test)
      lsim_interface_test=true
      shift
      ;;
    --no-record)
      record_output=false
      shift
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "unknown option: $1"
      ;;
  esac
done

require_positive_number "--rate" "$rate"
if [[ -n "$playback_duration" ]]; then
  require_positive_number "--playback-duration" "$playback_duration"
fi

[[ -e "$bag" ]] || fail "bag does not exist: $bag"
bag="$(realpath -e -- "$bag")"
if [[ -z "$output" ]]; then
  bag_name="$(basename "$bag")"
  output="$ROOT/test_results/${bag_name}_$(date +%Y%m%d_%H%M%S)"
fi
output="$(realpath -m -- "$output")"
[[ ! -e "$output" ]] || fail "output directory already exists: $output"

[[ -f /opt/ros/jazzy/setup.bash ]] || fail "ROS 2 Jazzy setup was not found"
[[ -f "$ROOT/install/setup.bash" ]] || fail "workspace is not built: $ROOT/install/setup.bash"
command -v setsid >/dev/null 2>&1 || fail "setsid is required for process-group cleanup"

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"

IMU_PARAM="$(ros2 pkg prefix pure_imu_undistortion)/share/pure_imu_undistortion/param/param_xt.yaml"
ODOM_PARAM="$(ros2 pkg prefix pure_lidar_gyro_odometer)/share/pure_lidar_gyro_odometer/param/param_xt_lidar_imu_only.yaml"
GNSS_FUSION_SHARE="$(ros2 pkg prefix pure_gnss_map_odom_fusion)/share/pure_gnss_map_odom_fusion"
if [[ -f "$GNSS_FUSION_SHARE/param/param.yaml" ]]; then
  GNSS_FUSION_PARAM="$GNSS_FUSION_SHARE/param/param.yaml"
else
  # Older installs placed the contents of param/ directly in the share root.
  GNSS_FUSION_PARAM="$GNSS_FUSION_SHARE/param.yaml"
fi
NMEA_GNSS_PARAM="$(ros2 pkg prefix pure_nmea_gnss_conversion)/share/pure_nmea_gnss_conversion/param/param.yaml"
BRINGUP_SHARE="$(ros2 pkg prefix pure_odometry_bringup)/share/pure_odometry_bringup"
NMEA_GNSS_OVERRIDE_PARAM="$BRINGUP_SHARE/config/autoware_lsim/hesai_rosbag23_nmea_override.yaml"
for parameter_file in "$IMU_PARAM" "$ODOM_PARAM" "$GNSS_FUSION_PARAM" "$NMEA_GNSS_PARAM"; do
  [[ -f "$parameter_file" ]] || fail "parameter file does not exist: $parameter_file"
done
[[ -f "$NMEA_GNSS_OVERRIDE_PARAM" ]] ||
  fail "parameter file does not exist: $NMEA_GNSS_OVERRIDE_PARAM"

tf_lidar_command=(
  ros2 run tf2_ros static_transform_publisher
  --x 0.0 --y 0.0 --z 0.0
  --roll 0.0 --pitch 0.0 --yaw 0.0
  --frame-id base_link --child-frame-id lidar/0
)
tf_imu_command=(
  ros2 run tf2_ros static_transform_publisher
  --x 0.0 --y 0.0 --z -0.1874
  --roll 3.14159 --pitch 0.0 --yaw 0.0
  --frame-id base_link --child-frame-id imu
)
tf_gnss_command=(
  ros2 run tf2_ros static_transform_publisher
  --x 0.0 --y 0.0 --z -0.1326
  --roll 0.0 --pitch 0.0 --yaw 0.0
  --frame-id base_link --child-frame-id gnss/0
)
if [[ "$lsim_interface_test" == true ]]; then
  launch_command=(
    ros2 launch pure_odometry_bringup autoware_lsim_localization.launch.py
    launch_autoware:=false
    launch_vehicle:=false
    launch_sensing:=false
    launch_rviz:=false
    sensor_profile:=hesai_rosbag23
    use_gnss:=true
    use_imu_deskew:=true
    points_input_topic:=/points_raw
    imu_input_topic:=/imu
    imu_param:="$IMU_PARAM"
    odom_param:="$ODOM_PARAM"
    gnss_fusion_param:="$GNSS_FUSION_PARAM"
    nmea_gnss_param:="$NMEA_GNSS_PARAM"
    nmea_gnss_override_param:="$NMEA_GNSS_OVERRIDE_PARAM"
    gnss_primary_gga_topic:=/nmea_sentence
    fusion_xy_only_recovery:=true
  )
else
  launch_command=(
    ros2 launch pure_odometry_bringup odometry_container.launch.py
    use_gnss:=true
    use_imu_deskew:=true
    use_sim_time:=true
    points_input_topic:=/points_raw
    imu_input_topic:=/imu
    imu_param:="$IMU_PARAM"
    odom_param:="$ODOM_PARAM"
    gnss_fusion_param:="$GNSS_FUSION_PARAM"
    nmea_gnss_param:="$NMEA_GNSS_PARAM"
    gnss_primary_gga_topic:=/nmea_sentence
    use_secondary_gga:=false
    use_doppler_heading:=false
    use_imu_yaw_rate_heading:=true
    # This single-antenna parking profile may regain RTK position while stopped.
    # Enable guarded XY-only recovery; generic bringup remains disabled by default.
    fusion_xy_only_recovery:=true
  )
fi
record_directory="$output/localization_output"
record_topics=(
  /clock
  /tf
  /tf_static
  /diagnostics
  /diagnostics_agg
  /localization/gyro_lidar_odom
  /localization/imu_corrected
  /localization/is_stopped
  /localization/ekf_odom
  /localization/ekf_pose
  /localization/gnss_odometry
  /localization/gnss_fusion_input
  /localization/global_pose_with_covariance
  /localization/gnss_confidence
  /localization/kinematic_state
  /localization/pose_estimator/pose_with_covariance
  /localization/acceleration
)
record_command=(
  ros2 bag record
  --storage mcap
  --output "$record_directory"
  --node-name vehicle_localizer_output_recorder
  --topics "${record_topics[@]}"
)
play_command=(
  bash "$ROOT/script/play_localization_bag.sh"
  --bag "$bag"
  --points /pandar_points_ex
  --imu /sensor/imu/data_raw
  --nmea /sensor/gnss/nmea_sentence
  --rate "$rate"
  --tf-policy isolate-all
)
if [[ "$lsim_interface_test" == true ]]; then
  play_command+=(--clock-frequency 100.0)
fi
play_command+=(
  --
  --disable-keyboard-controls
  # Give the bag player's publishers time to match existing subscribers so
  # the first IMU samples are not lost during DDS discovery.
  --delay 2
  --topics
  /pandar_points_ex
  /sensor/imu/data_raw
  /sensor/gnss/nmea_sentence
)
if [[ -n "$playback_duration" ]]; then
  play_command+=(--playback-duration "$playback_duration")
fi

log "bag: $bag"
log "output: $output"
log "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

if [[ "$dry_run" == true ]]; then
  if [[ "$lsim_interface_test" != true ]]; then
    print_command "${tf_lidar_command[@]}"
    print_command "${tf_imu_command[@]}"
    print_command "${tf_gnss_command[@]}"
  fi
  print_command "${launch_command[@]}"
  if [[ "$record_output" == true ]]; then
    print_command "${record_command[@]}"
  fi
  print_command "${play_command[@]}"
  exit 0
fi

mkdir -p "$output/ros_logs"
export ROS_LOG_DIR="$output/ros_logs"
{
  printf 'bag=%q\n' "$bag"
  printf 'rate=%q\n' "$rate"
  printf 'playback_duration=%q\n' "$playback_duration"
  printf 'record_output=%q\n' "$record_output"
  printf 'lsim_interface_test=%q\n' "$lsim_interface_test"
  printf 'ROS_DOMAIN_ID=%q\n' "$ROS_DOMAIN_ID"
} > "$output/run.env"
ros2 bag info "$bag" > "$output/input_bag_info.txt" 2>&1 || true

tf_lidar_pid=""
tf_imu_pid=""
tf_gnss_pid=""
launch_pid=""
record_pid=""
play_pid=""
started_pid=""

start_process_group() {
  local log_file="$1"
  shift
  setsid stdbuf -oL -eL "$@" > "$log_file" 2>&1 &
  started_pid=$!
}

process_group_alive() {
  local pid="$1"
  [[ "$pid" =~ ^[1-9][0-9]*$ ]] && kill -0 -- "-$pid" 2>/dev/null
}

process_alive() {
  local pid="$1"
  local state
  [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 1
  state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d '[:space:]')" || return 1
  [[ -n "$state" && "$state" != Z* ]]
}

stop_process_group() {
  local pid="$1"
  local label="$2"
  [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 0

  if process_group_alive "$pid"; then
    log "stopping $label (process group $pid)"
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in {1..50}; do
      process_alive "$pid" || break
      sleep 0.1
    done
  fi
  if process_alive "$pid" && process_group_alive "$pid"; then
    kill -TERM -- "-$pid" 2>/dev/null || true
    for _ in {1..20}; do
      process_alive "$pid" || break
      sleep 0.1
    done
  fi
  if process_alive "$pid" && process_group_alive "$pid"; then
    kill -KILL -- "-$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true

  # A launcher may exit before every child in its process group. Do not leave
  # such children attached to the ROS graph after the run finishes.
  if process_group_alive "$pid"; then
    kill -TERM -- "-$pid" 2>/dev/null || true
    sleep 0.2
    process_group_alive "$pid" && kill -KILL -- "-$pid" 2>/dev/null || true
  fi
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  stop_process_group "$play_pid" "bag player"
  stop_process_group "$record_pid" "bag recorder"
  stop_process_group "$launch_pid" "localization launch"
  stop_process_group "$tf_gnss_pid" "GNSS static TF"
  stop_process_group "$tf_imu_pid" "IMU static TF"
  stop_process_group "$tf_lidar_pid" "LiDAR static TF"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_for_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local deadline=$((SECONDS + timeout_sec))
  while ((SECONDS < deadline)); do
    process_alive "$launch_pid" || return 2
    if ros2 topic list --no-daemon --spin-time 1 2>/dev/null | grep -Fx "$topic" >/dev/null; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

wait_for_node() {
  local node="$1"
  local pid="$2"
  local timeout_sec="$3"
  local deadline=$((SECONDS + timeout_sec))
  while ((SECONDS < deadline)); do
    process_alive "$pid" || return 2
    if ros2 node list --no-daemon --spin-time 1 2>/dev/null | grep -Fx "$node" >/dev/null; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

if [[ "$lsim_interface_test" != true ]]; then
  start_process_group "$output/tf_lidar.log" "${tf_lidar_command[@]}"
  tf_lidar_pid=$started_pid
  start_process_group "$output/tf_imu.log" "${tf_imu_command[@]}"
  tf_imu_pid=$started_pid
  start_process_group "$output/tf_gnss.log" "${tf_gnss_command[@]}"
  tf_gnss_pid=$started_pid
  sleep 0.2
  process_group_alive "$tf_lidar_pid" || fail "LiDAR static TF publisher failed; see $output/tf_lidar.log"
  process_group_alive "$tf_imu_pid" || fail "IMU static TF publisher failed; see $output/tf_imu.log"
  process_group_alive "$tf_gnss_pid" || fail "GNSS static TF publisher failed; see $output/tf_gnss.log"
fi

start_process_group "$output/launch.log" "${launch_command[@]}"
launch_pid=$started_pid

if ! wait_for_topic /localization/gyro_lidar_odom 60; then
  fail "localization stack did not become ready; see $output/launch.log"
fi
if ! wait_for_topic /localization/gnss_fusion_input 60; then
  fail "GNSS frontend did not become ready; see $output/launch.log"
fi
if ! wait_for_topic /localization/ekf_odom 60; then
  fail "GNSS fusion did not become ready; see $output/launch.log"
fi
if [[ "$lsim_interface_test" == true ]] &&
  ! wait_for_topic /localization/kinematic_state 60
then
  fail "Autoware localization adapter did not become ready; see $output/launch.log"
fi
log "localization publishers are ready"

if [[ "$record_output" == true ]]; then
  start_process_group "$output/record.log" "${record_command[@]}"
  record_pid=$started_pid
  if ! wait_for_node /vehicle_localizer_output_recorder "$record_pid" 15; then
    fail "bag recorder did not become ready; see $output/record.log"
  fi
  log "recording localization outputs to $record_directory"
fi

start_process_group "$output/play.log" "${play_command[@]}"
play_pid=$started_pid
log "bag playback started"

set +e
wait "$play_pid"
play_status=$?
set -e
play_pid=""

# Give the recorder a final scheduling cycle before sending SIGINT so that it
# can write the last messages and finalize metadata.yaml.
sleep 1
stop_process_group "$record_pid" "bag recorder"
record_pid=""
stop_process_group "$launch_pid" "localization launch"
launch_pid=""
stop_process_group "$tf_gnss_pid" "GNSS static TF"
tf_gnss_pid=""
stop_process_group "$tf_imu_pid" "IMU static TF"
tf_imu_pid=""
stop_process_group "$tf_lidar_pid" "LiDAR static TF"
tf_lidar_pid=""

if ((play_status != 0)); then
  fail "bag playback exited with status $play_status; see $output/play.log"
fi
log "localization run completed: $output"
