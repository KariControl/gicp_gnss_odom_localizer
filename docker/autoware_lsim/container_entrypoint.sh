#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -Eeuo pipefail

log() {
  printf '[autoware-lsim] %s\n' "$*"
}

warn() {
  printf '[autoware-lsim] WARN: %s\n' "$*" >&2
}

fail() {
  printf '[autoware-lsim] ERROR: %s\n' "$*" >&2
  exit 2
}

normalize_bool() {
  case "${1,,}" in
    1|true|yes|on) printf 'true' ;;
    0|false|no|off) printf 'false' ;;
    *) fail "invalid boolean value: $1" ;;
  esac
}

require_positive_number() {
  local name="$1"
  local value="$2"
  if ! [[ "$value" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
    fail "$name must be a positive number: $value"
  fi
  if ! awk -v value="$value" 'BEGIN { exit !(value > 0.0) }'; then
    fail "$name must be greater than zero: $value"
  fi
}

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /opt/autoware/setup.bash
source "${GICP_GNSS_ODOM_INSTALL:-/opt/gicp_gnss_odom_localizer}/setup.bash"

BAG_PATH="${BAG_PATH:-/bags/input}"
OUTPUT_ROOT="${OUTPUT_ROOT:-/output}"
RUN_NAME="${RUN_NAME:-autoware_lsim}"
POINTS_SOURCE_TOPIC="${POINTS_SOURCE_TOPIC:-/sensing/lidar/top/pointcloud_raw}"
IMU_SOURCE_TOPIC="${IMU_SOURCE_TOPIC:-/sensing/imu/tamagawa/imu_raw}"
NMEA_SOURCE_TOPIC="${NMEA_SOURCE_TOPIC:-}"
NMEA_SECONDARY_SOURCE_TOPIC="${NMEA_SECONDARY_SOURCE_TOPIC:-}"
FIX_VELOCITY_SOURCE_TOPIC="${FIX_VELOCITY_SOURCE_TOPIC:-}"
TWIST_SOURCE_TOPIC="${TWIST_SOURCE_TOPIC:-}"
PLAYBACK_RATE="${PLAYBACK_RATE:-1.0}"
TF_POLICY="${TF_POLICY:-isolate-dynamic}"
TRACKING_MODE="${TRACKING_MODE:-scan_to_scan}"
USE_GNSS="$(normalize_bool "${USE_GNSS:-false}")"
USE_IMU_DESKEW="$(normalize_bool "${USE_IMU_DESKEW:-true}")"
LAUNCH_VEHICLE="$(normalize_bool "${LAUNCH_VEHICLE:-false}")"
LAUNCH_SENSING="$(normalize_bool "${LAUNCH_SENSING:-false}")"
RVIZ="$(normalize_bool "${RVIZ:-false}")"
RECORD_OUTPUT="$(normalize_bool "${RECORD_OUTPUT:-true}")"
AUTO_INITIAL_POSE="$(normalize_bool "${AUTO_INITIAL_POSE:-true}")"
KEEP_RECORDED_LOCALIZATION="$(normalize_bool "${KEEP_RECORDED_LOCALIZATION:-false}")"
VEHICLE_MODEL="${VEHICLE_MODEL:-sample_vehicle}"
SENSOR_MODEL="${SENSOR_MODEL:-sample_sensor_kit}"
INITIAL_X="${INITIAL_X:-0.0}"
INITIAL_Y="${INITIAL_Y:-0.0}"
INITIAL_Z="${INITIAL_Z:-0.0}"
INITIAL_YAW="${INITIAL_YAW:-0.0}"
LOG_LEVEL="${LOG_LEVEL:-info}"
STARTUP_WAIT_SEC="${STARTUP_WAIT_SEC:-90}"
INITIALPOSE_WAIT_SEC="${INITIALPOSE_WAIT_SEC:-90}"

[[ -e "$BAG_PATH" ]] || fail "mounted bag does not exist: $BAG_PATH"
mkdir -p "$OUTPUT_ROOT"
[[ -w "$OUTPUT_ROOT" ]] || fail "output directory is not writable: $OUTPUT_ROOT"
require_positive_number "PLAYBACK_RATE" "$PLAYBACK_RATE"
require_positive_number "STARTUP_WAIT_SEC" "$STARTUP_WAIT_SEC"
require_positive_number "INITIALPOSE_WAIT_SEC" "$INITIALPOSE_WAIT_SEC"

case "$TF_POLICY" in
  keep|isolate-dynamic|isolate-all) ;;
  *) fail "TF_POLICY must be keep, isolate-dynamic, or isolate-all: $TF_POLICY" ;;
esac

case "$TRACKING_MODE" in
  scan_to_scan)
    ODOM_PARAM="${GICP_GNSS_ODOM_INSTALL}/share/pure_lidar_gyro_odometer/param/param.yaml"
    ;;
  scan_to_submap)
    ODOM_PARAM="${GICP_GNSS_ODOM_INSTALL}/share/pure_lidar_gyro_odometer/param/param_scan_to_submap.yaml"
    ;;
  *) fail "TRACKING_MODE must be scan_to_scan or scan_to_submap: $TRACKING_MODE" ;;
esac

if [[ "$USE_GNSS" == true && -z "$NMEA_SOURCE_TOPIC" ]]; then
  warn "USE_GNSS=true but NMEA_SOURCE_TOPIC is empty; GNSS initialization will not occur."
fi
if [[ "$TF_POLICY" == isolate-all && "$LAUNCH_VEHICLE" != true ]]; then
  fail "TF_POLICY=isolate-all requires LAUNCH_VEHICLE=true or another static-TF publisher."
fi
if [[ "$RVIZ" == true && -z "${DISPLAY:-}" ]]; then
  fail "RVIZ=true requires DISPLAY and the RViz compose overlay."
fi

safe_run_name="$(printf '%s' "$RUN_NAME" | tr -cs 'A-Za-z0-9._-' '_')"
[[ -n "$safe_run_name" ]] || safe_run_name="autoware_lsim"
run_directory="$OUTPUT_ROOT/$safe_run_name"
if [[ -e "$run_directory" ]]; then
  run_directory="${run_directory}_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$run_directory"

launch_pid=""
bag_pid=""
record_pid=""

stop_process() {
  local pid="$1"
  local signal="${2:-INT}"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill "-$signal" "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  stop_process "$bag_pid" INT
  stop_process "$record_pid" INT
  stop_process "$launch_pid" INT
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

write_manifest() {
  local manifest="$run_directory/run.env"
  {
    printf 'AUTOWARE_IMAGE=%q\n' "${AUTOWARE_IMAGE:-ghcr.io/autowarefoundation/autoware:universe-devel-jazzy-1.9.0}"
    printf 'BAG_PATH=%q\n' "$BAG_PATH"
    printf 'POINTS_SOURCE_TOPIC=%q\n' "$POINTS_SOURCE_TOPIC"
    printf 'IMU_SOURCE_TOPIC=%q\n' "$IMU_SOURCE_TOPIC"
    printf 'NMEA_SOURCE_TOPIC=%q\n' "$NMEA_SOURCE_TOPIC"
    printf 'NMEA_SECONDARY_SOURCE_TOPIC=%q\n' "$NMEA_SECONDARY_SOURCE_TOPIC"
    printf 'FIX_VELOCITY_SOURCE_TOPIC=%q\n' "$FIX_VELOCITY_SOURCE_TOPIC"
    printf 'TWIST_SOURCE_TOPIC=%q\n' "$TWIST_SOURCE_TOPIC"
    printf 'PLAYBACK_RATE=%q\n' "$PLAYBACK_RATE"
    printf 'TF_POLICY=%q\n' "$TF_POLICY"
    printf 'TRACKING_MODE=%q\n' "$TRACKING_MODE"
    printf 'USE_GNSS=%q\n' "$USE_GNSS"
    printf 'USE_IMU_DESKEW=%q\n' "$USE_IMU_DESKEW"
    printf 'LAUNCH_VEHICLE=%q\n' "$LAUNCH_VEHICLE"
    printf 'LAUNCH_SENSING=%q\n' "$LAUNCH_SENSING"
    printf 'VEHICLE_MODEL=%q\n' "$VEHICLE_MODEL"
    printf 'SENSOR_MODEL=%q\n' "$SENSOR_MODEL"
    printf 'RVIZ=%q\n' "$RVIZ"
    printf 'RECORD_OUTPUT=%q\n' "$RECORD_OUTPUT"
    printf 'AUTO_INITIAL_POSE=%q\n' "$AUTO_INITIAL_POSE"
    printf 'INITIAL_X=%q\n' "$INITIAL_X"
    printf 'INITIAL_Y=%q\n' "$INITIAL_Y"
    printf 'INITIAL_Z=%q\n' "$INITIAL_Z"
    printf 'INITIAL_YAW=%q\n' "$INITIAL_YAW"
    printf 'ROS_DOMAIN_ID=%q\n' "${ROS_DOMAIN_ID:-0}"
    printf 'RMW_IMPLEMENTATION=%q\n' "${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  } > "$manifest"
  ros2 bag info "$BAG_PATH" > "$run_directory/input_bag_info.txt" 2>&1 || true
}

wait_for_topic() {
  local topic="$1"
  local deadline=$((SECONDS + STARTUP_WAIT_SEC))
  while ((SECONDS < deadline)); do
    if [[ -n "$launch_pid" ]] && ! kill -0 "$launch_pid" 2>/dev/null; then
      return 2
    fi
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

publish_initial_pose() {
  local quaternion
  quaternion="$(python3 - "$INITIAL_YAW" <<'PY'
import math
import sys

yaw = float(sys.argv[1])
print(f"{math.sin(0.5 * yaw):.17g} {math.cos(0.5 * yaw):.17g}")
PY
)"
  local qz qw
  read -r qz qw <<< "$quaternion"

  log "publishing automatic initial map pose: x=$INITIAL_X y=$INITIAL_Y yaw=$INITIAL_YAW"
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: ${INITIAL_X}, y: ${INITIAL_Y}, z: ${INITIAL_Z}}, orientation: {x: 0.0, y: 0.0, z: ${qz}, w: ${qw}}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04]}}" \
    > "$run_directory/initialpose.log" 2>&1
}

write_manifest

launch_command=(
  ros2 launch pure_odometry_bringup autoware_lsim_localization.launch.py
  launch_autoware:=true
  vehicle_model:="$VEHICLE_MODEL"
  sensor_model:="$SENSOR_MODEL"
  launch_vehicle:="$LAUNCH_VEHICLE"
  launch_sensing:="$LAUNCH_SENSING"
  launch_rviz:="$RVIZ"
  use_gnss:="$USE_GNSS"
  use_imu_deskew:="$USE_IMU_DESKEW"
  points_input_topic:=/points_raw
  imu_input_topic:=/imu
  twist_input_topic:="$(if [[ -n "$TWIST_SOURCE_TOPIC" ]]; then printf '/localization/input_twist'; fi)"
  odom_param:="$ODOM_PARAM"
  log_level:="$LOG_LEVEL"
)

printf '[autoware-lsim] launch:'
printf ' %q' "${launch_command[@]}"
printf '\n'
stdbuf -oL -eL "${launch_command[@]}" \
  > >(tee "$run_directory/launch.log") \
  2>&1 &
launch_pid=$!

if ! wait_for_topic /localization/kinematic_state; then
  fail "Autoware/localizer launch did not create /localization/kinematic_state; see $run_directory/launch.log"
fi
log "Autoware localization interface is ready."

if [[ "$RECORD_OUTPUT" == true ]]; then
  record_directory="$run_directory/localization_output"
  record_topics=(
    /clock
    /tf
    /tf_static
    /diagnostics
    /localization/gyro_lidar_odom
    /localization/ekf_odom
    /localization/kinematic_state
    /localization/pose_estimator/pose_with_covariance
    /localization/acceleration
    /reference/tf
    /reference/tf_static
    /reference/diagnostics
    /reference/localization/gyro_lidar_odom
    /reference/localization/ekf_odom
    /reference/localization/kinematic_state
    /reference/localization/pose_estimator/pose_with_covariance
  )
  log "recording evaluation outputs to $record_directory"
  ros2 bag record --output "$record_directory" "${record_topics[@]}" \
    > "$run_directory/record.log" 2>&1 &
  record_pid=$!
fi

play_command=(
  "${GICP_GNSS_ODOM_INSTALL}/bin/play_localization_bag.sh"
  --bag "$BAG_PATH"
  --points "$POINTS_SOURCE_TOPIC"
  --imu "$IMU_SOURCE_TOPIC"
  --rate "$PLAYBACK_RATE"
  --tf-policy "$TF_POLICY"
)
[[ -n "$NMEA_SOURCE_TOPIC" ]] && play_command+=(--nmea "$NMEA_SOURCE_TOPIC")
[[ -n "$NMEA_SECONDARY_SOURCE_TOPIC" ]] && \
  play_command+=(--nmea-secondary "$NMEA_SECONDARY_SOURCE_TOPIC")
[[ -n "$FIX_VELOCITY_SOURCE_TOPIC" ]] && \
  play_command+=(--fix-velocity "$FIX_VELOCITY_SOURCE_TOPIC")
[[ -n "$TWIST_SOURCE_TOPIC" ]] && play_command+=(--twist "$TWIST_SOURCE_TOPIC")
[[ "$KEEP_RECORDED_LOCALIZATION" == true ]] && \
  play_command+=(--keep-recorded-localization)

printf '[autoware-lsim] replay:'
printf ' %q' "${play_command[@]}"
printf '\n'
stdbuf -oL -eL "${play_command[@]}" \
  > >(tee "$run_directory/replay.log") \
  2>&1 &
bag_pid=$!

if [[ "$USE_GNSS" != true && "$AUTO_INITIAL_POSE" == true ]]; then
  if timeout "$INITIALPOSE_WAIT_SEC" \
    ros2 topic echo --once /localization/gyro_lidar_odom > /dev/null 2>&1; then
    publish_initial_pose
  else
    warn "no LiDAR odometry arrived before the initial-pose timeout; fused Autoware output may remain unavailable"
  fi
fi

set +e
wait "$bag_pid"
bag_status=$?
set -e
bag_pid=""

# Give recorder subscriptions one final scheduling cycle, then stop cleanly so
# rosbag2 writes metadata before the container exits.
sleep 1
stop_process "$record_pid" INT
record_pid=""
stop_process "$launch_pid" INT
launch_pid=""

if ((bag_status != 0)); then
  fail "rosbag playback exited with status $bag_status; see $run_directory/replay.log"
fi
log "evaluation complete: $run_directory"
