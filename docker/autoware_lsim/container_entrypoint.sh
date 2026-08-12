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

# ROS-generated setup files are not nounset-safe.  Keep strict mode for the
# runner itself, but suspend it only while the three overlays are sourced.
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /opt/autoware/setup.bash
source "${GICP_GNSS_ODOM_INSTALL:-/opt/gicp_gnss_odom_localizer}/setup.bash"
set -u

BAG_PATH="${BAG_PATH:-/bags/input}"
OUTPUT_ROOT="${OUTPUT_ROOT:-/output}"
RUN_NAME="${RUN_NAME:-autoware_lsim}"
DATASET_PROFILE="${DATASET_PROFILE:-generic}"
POINTS_SOURCE_TOPIC="${POINTS_SOURCE_TOPIC:-/sensing/lidar/top/pointcloud_raw}"
IMU_SOURCE_TOPIC="${IMU_SOURCE_TOPIC:-/sensing/imu/tamagawa/imu_raw}"
NMEA_SOURCE_TOPIC="${NMEA_SOURCE_TOPIC:-}"
NMEA_SECONDARY_SOURCE_TOPIC="${NMEA_SECONDARY_SOURCE_TOPIC:-}"
FIX_VELOCITY_SOURCE_TOPIC="${FIX_VELOCITY_SOURCE_TOPIC:-}"
TWIST_SOURCE_TOPIC="${TWIST_SOURCE_TOPIC:-}"
PLAYBACK_RATE="${PLAYBACK_RATE:-1.0}"
CLOCK_FREQUENCY="${CLOCK_FREQUENCY:-100.0}"
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
LOCALIZER_IMAGE_ID="${LOCALIZER_IMAGE_ID:-unknown}"
STARTUP_WAIT_SEC="${STARTUP_WAIT_SEC:-90}"
INITIALPOSE_WAIT_SEC="${INITIALPOSE_WAIT_SEC:-90}"
DRAIN_WAIT_SEC="${DRAIN_WAIT_SEC:-5}"

[[ -e "$BAG_PATH" ]] || fail "mounted bag does not exist: $BAG_PATH"
mkdir -p "$OUTPUT_ROOT"
[[ -w "$OUTPUT_ROOT" ]] || fail "output directory is not writable: $OUTPUT_ROOT"
require_positive_number "PLAYBACK_RATE" "$PLAYBACK_RATE"
require_positive_number "CLOCK_FREQUENCY" "$CLOCK_FREQUENCY"
require_positive_number "STARTUP_WAIT_SEC" "$STARTUP_WAIT_SEC"
require_positive_number "INITIALPOSE_WAIT_SEC" "$INITIALPOSE_WAIT_SEC"
require_positive_number "DRAIN_WAIT_SEC" "$DRAIN_WAIT_SEC"

FIRST_STATE_WAIT_SEC="$(awk -v base="$STARTUP_WAIT_SEC" -v rate="$PLAYBACK_RATE" \
  'BEGIN { scaled = base / rate; if (scaled < base) scaled = base; printf "%.3f", scaled }')"
INITIALPOSE_DATA_WAIT_SEC="$(awk -v base="$INITIALPOSE_WAIT_SEC" -v rate="$PLAYBACK_RATE" \
  'BEGIN { scaled = base / rate; if (scaled < base) scaled = base; printf "%.3f", scaled }')"

case "$TF_POLICY" in
  keep|isolate-dynamic|isolate-all) ;;
  *) fail "TF_POLICY must be keep, isolate-dynamic, or isolate-all: $TF_POLICY" ;;
esac

BRINGUP_SHARE="${GICP_GNSS_ODOM_INSTALL}/share/pure_odometry_bringup"
PRECISION_BRINGUP_SHARE="${GICP_GNSS_ODOM_INSTALL}/share/pure_precision_bringup"
RVIZ_CONFIG="$BRINGUP_SHARE/config/autoware_lsim/hesai_rosbag23.rviz"
EMPTY_PARAM="$BRINGUP_SHARE/config/autoware_lsim/empty_params.yaml"
SUBMAP_SNAPSHOT_OVERRIDE_PARAM="$PRECISION_BRINGUP_SHARE/config/submap_snapshot_override.yaml"
PRECISION_MATCHER_PARAM="${GICP_GNSS_ODOM_INSTALL}/share/pure_lidar_submap_matcher/param/param.yaml"
PRECISION_GLOBAL_PARAM="${GICP_GNSS_ODOM_INSTALL}/share/pure_precision_global_localizer/param/param.yaml"
NMEA_GNSS_PARAM="${GICP_GNSS_ODOM_INSTALL}/share/pure_nmea_gnss_conversion/param/param.yaml"
NMEA_PROJECTOR_METADATA="${GICP_GNSS_ODOM_INSTALL}/share/pure_nmea_gnss_conversion/config/map_projector_info.yaml"
GNSS_FUSION_PARAM="${GICP_GNSS_ODOM_INSTALL}/share/pure_gnss_map_odom_fusion/param/param.yaml"
DIAGNOSTIC_AGGREGATOR_PARAM="$BRINGUP_SHARE/config/diagnostic_aggregator.yaml"
AUTOWARE_ADAPTER_PARAM="${GICP_GNSS_ODOM_INSTALL}/share/pure_autoware_localization_adapter/param/param.yaml"
ODOM_OVERRIDE_PARAM="$EMPTY_PARAM"
PRECISION_MATCHER_OVERRIDE_PARAM="$PRECISION_BRINGUP_SHARE/config/empty_params.yaml"
PRECISION_GLOBAL_OVERRIDE_PARAM="$PRECISION_BRINGUP_SHARE/config/empty_params.yaml"

case "$TRACKING_MODE" in
  scan_to_scan) ;;
  scan_to_submap) ODOM_OVERRIDE_PARAM="$SUBMAP_SNAPSHOT_OVERRIDE_PARAM" ;;
  *) fail "TRACKING_MODE must be scan_to_scan or scan_to_submap: $TRACKING_MODE" ;;
esac

case "$DATASET_PROFILE" in
  generic)
    SENSOR_PROFILE="generic"
    IMU_PARAM="${GICP_GNSS_ODOM_INSTALL}/share/pure_imu_undistortion/param/param.yaml"
    ODOM_PARAM="${GICP_GNSS_ODOM_INSTALL}/share/pure_lidar_gyro_odometer/param/param.yaml"
    NMEA_GNSS_OVERRIDE_PARAM="$EMPTY_PARAM"
    GNSS_FUSION_OVERRIDE_PARAM="$EMPTY_PARAM"
    ;;
  hesai_rosbag23)
    [[ "$POINTS_SOURCE_TOPIC" == /pandar_points_ex ]] ||
      fail "hesai_rosbag23 requires POINTS_SOURCE_TOPIC=/pandar_points_ex"
    [[ "$IMU_SOURCE_TOPIC" == /sensor/imu/data_raw ]] ||
      fail "hesai_rosbag23 requires IMU_SOURCE_TOPIC=/sensor/imu/data_raw"
    [[ "$NMEA_SOURCE_TOPIC" == /sensor/gnss/nmea_sentence ]] ||
      fail "hesai_rosbag23 requires NMEA_SOURCE_TOPIC=/sensor/gnss/nmea_sentence"
    [[ "$USE_GNSS" == true ]] || fail "hesai_rosbag23 requires USE_GNSS=true"
    [[ "$USE_IMU_DESKEW" == true ]] || fail "hesai_rosbag23 requires USE_IMU_DESKEW=true"
    [[ "$LAUNCH_VEHICLE" != true ]] ||
      fail "hesai_rosbag23 publishes calibrated TFs and cannot launch sample vehicle TFs"
    SENSOR_PROFILE="hesai_rosbag23"
    IMU_PARAM="${GICP_GNSS_ODOM_INSTALL}/share/pure_imu_undistortion/param/param_xt.yaml"
    ODOM_PARAM="${GICP_GNSS_ODOM_INSTALL}/share/pure_lidar_gyro_odometer/param/param_xt_lidar_imu_only.yaml"
    # Preserve the projection in the NMEA component's base parameter file.
    NMEA_GNSS_OVERRIDE_PARAM="$EMPTY_PARAM"
    GNSS_FUSION_OVERRIDE_PARAM="$BRINGUP_SHARE/config/evaluation/lidar_imu_gnss/hesai_32line_rtk/accepted/gnss_fusion_single_antenna.yaml"
    ;;
  *) fail "DATASET_PROFILE must be generic or hesai_rosbag23: $DATASET_PROFILE" ;;
esac

for parameter_file in \
  "$EMPTY_PARAM" "$IMU_PARAM" "$ODOM_PARAM" "$ODOM_OVERRIDE_PARAM" \
  "$NMEA_GNSS_PARAM" "$NMEA_PROJECTOR_METADATA" \
  "$NMEA_GNSS_OVERRIDE_PARAM" "$GNSS_FUSION_PARAM" \
  "$GNSS_FUSION_OVERRIDE_PARAM" "$DIAGNOSTIC_AGGREGATOR_PARAM" \
  "$AUTOWARE_ADAPTER_PARAM"
do
  [[ -f "$parameter_file" ]] || fail "parameter file does not exist: $parameter_file"
done
if [[ "$TRACKING_MODE" == scan_to_submap ]]; then
  for parameter_file in \
    "$PRECISION_MATCHER_PARAM" "$PRECISION_MATCHER_OVERRIDE_PARAM" \
    "$PRECISION_GLOBAL_PARAM" "$PRECISION_GLOBAL_OVERRIDE_PARAM"
  do
    [[ -f "$parameter_file" ]] || fail "parameter file does not exist: $parameter_file"
  done
fi
[[ -f "$RVIZ_CONFIG" ]] || fail "RViz config does not exist: $RVIZ_CONFIG"

if [[ "$USE_GNSS" == true && -z "$NMEA_SOURCE_TOPIC" ]]; then
  warn "USE_GNSS=true but NMEA_SOURCE_TOPIC is empty; GNSS initialization will not occur."
fi
if [[ "$TF_POLICY" == isolate-all && "$LAUNCH_VEHICLE" != true && \
  "$SENSOR_PROFILE" == generic ]]; then
  fail "TF_POLICY=isolate-all requires LAUNCH_VEHICLE=true or a calibrated sensor profile."
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
precision_launch_pid=""
bag_pid=""
record_pid=""

stop_process() {
  local pid="$1"
  local signal="${2:-INT}"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill "-$signal" "$pid" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
      for _ in {1..20}; do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.1
      done
    fi
    kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
  fi
  [[ -n "$pid" ]] && wait "$pid" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  stop_process "$bag_pid" INT
  stop_process "$record_pid" INT
  stop_process "$precision_launch_pid" INT
  stop_process "$launch_pid" INT
  if [[ "${HOST_UID:-}" =~ ^[0-9]+$ && "${HOST_GID:-}" =~ ^[0-9]+$ ]]; then
    chown -R "${HOST_UID}:${HOST_GID}" "$run_directory" 2>/dev/null ||
      warn "could not restore host ownership for $run_directory"
  fi
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

write_manifest() {
  local manifest="$run_directory/run.env"
  {
    printf 'AUTOWARE_IMAGE=%q\n' "${AUTOWARE_IMAGE:-ghcr.io/autowarefoundation/autoware:universe-devel-jazzy-1.9.0}"
    printf 'LOCALIZER_IMAGE_ID=%q\n' "$LOCALIZER_IMAGE_ID"
    printf 'BAG_PATH=%q\n' "$BAG_PATH"
    printf 'DATASET_PROFILE=%q\n' "$DATASET_PROFILE"
    printf 'POINTS_SOURCE_TOPIC=%q\n' "$POINTS_SOURCE_TOPIC"
    printf 'IMU_SOURCE_TOPIC=%q\n' "$IMU_SOURCE_TOPIC"
    printf 'NMEA_SOURCE_TOPIC=%q\n' "$NMEA_SOURCE_TOPIC"
    printf 'NMEA_SECONDARY_SOURCE_TOPIC=%q\n' "$NMEA_SECONDARY_SOURCE_TOPIC"
    printf 'FIX_VELOCITY_SOURCE_TOPIC=%q\n' "$FIX_VELOCITY_SOURCE_TOPIC"
    printf 'TWIST_SOURCE_TOPIC=%q\n' "$TWIST_SOURCE_TOPIC"
    printf 'PLAYBACK_RATE=%q\n' "$PLAYBACK_RATE"
    printf 'CLOCK_FREQUENCY=%q\n' "$CLOCK_FREQUENCY"
    printf 'FIRST_STATE_WAIT_SEC=%q\n' "$FIRST_STATE_WAIT_SEC"
    printf 'DRAIN_WAIT_SEC=%q\n' "$DRAIN_WAIT_SEC"
    printf 'TF_POLICY=%q\n' "$TF_POLICY"
    printf 'TRACKING_MODE=%q\n' "$TRACKING_MODE"
    printf 'SENSOR_PROFILE=%q\n' "$SENSOR_PROFILE"
    printf 'IMU_PARAM=%q\n' "$IMU_PARAM"
    printf 'ODOM_PARAM=%q\n' "$ODOM_PARAM"
    printf 'ODOM_OVERRIDE_PARAM=%q\n' "$ODOM_OVERRIDE_PARAM"
    printf 'PRECISION_MATCHER_PARAM=%q\n' "$PRECISION_MATCHER_PARAM"
    printf 'PRECISION_GLOBAL_PARAM=%q\n' "$PRECISION_GLOBAL_PARAM"
    printf 'NMEA_GNSS_PARAM=%q\n' "$NMEA_GNSS_PARAM"
    printf 'NMEA_PROJECTOR_METADATA=%q\n' "$NMEA_PROJECTOR_METADATA"
    printf 'NMEA_GNSS_OVERRIDE_PARAM=%q\n' "$NMEA_GNSS_OVERRIDE_PARAM"
    printf 'GNSS_FUSION_PARAM=%q\n' "$GNSS_FUSION_PARAM"
    printf 'GNSS_FUSION_OVERRIDE_PARAM=%q\n' "$GNSS_FUSION_OVERRIDE_PARAM"
    printf 'DIAGNOSTIC_AGGREGATOR_PARAM=%q\n' "$DIAGNOSTIC_AGGREGATOR_PARAM"
    printf 'AUTOWARE_ADAPTER_PARAM=%q\n' "$AUTOWARE_ADAPTER_PARAM"
    printf 'PRECISION_MATCHER_OVERRIDE_PARAM=%q\n' "$PRECISION_MATCHER_OVERRIDE_PARAM"
    printf 'PRECISION_GLOBAL_OVERRIDE_PARAM=%q\n' "$PRECISION_GLOBAL_OVERRIDE_PARAM"
    printf 'USE_GNSS=%q\n' "$USE_GNSS"
    printf 'USE_IMU_DESKEW=%q\n' "$USE_IMU_DESKEW"
    printf 'LAUNCH_VEHICLE=%q\n' "$LAUNCH_VEHICLE"
    printf 'LAUNCH_SENSING=%q\n' "$LAUNCH_SENSING"
    printf 'VEHICLE_MODEL=%q\n' "$VEHICLE_MODEL"
    printf 'SENSOR_MODEL=%q\n' "$SENSOR_MODEL"
    printf 'RVIZ=%q\n' "$RVIZ"
    printf 'RVIZ_CONFIG=%q\n' "$RVIZ_CONFIG"
    printf 'RECORD_OUTPUT=%q\n' "$RECORD_OUTPUT"
    printf 'AUTO_INITIAL_POSE=%q\n' "$AUTO_INITIAL_POSE"
    printf 'INITIAL_X=%q\n' "$INITIAL_X"
    printf 'INITIAL_Y=%q\n' "$INITIAL_Y"
    printf 'INITIAL_Z=%q\n' "$INITIAL_Z"
    printf 'INITIAL_YAW=%q\n' "$INITIAL_YAW"
    printf 'ROS_DOMAIN_ID=%q\n' "${ROS_DOMAIN_ID:-0}"
    printf 'RMW_IMPLEMENTATION=%q\n' "${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  } > "$manifest"
  {
    printf 'localizer_image_id=%s\n' "$LOCALIZER_IMAGE_ID"
    printf 'autoware_image=%s\n' "${AUTOWARE_IMAGE:-unknown}"
    printf 'autoware_launch_prefix=%s\n' "$(ros2 pkg prefix autoware_launch)"
    printf 'autoware_launch_version=%s\n' "$(ros2 pkg xml --tag version autoware_launch)"
    printf 'ros_distro=%s\n' "${ROS_DISTRO:-jazzy}"
  } > "$run_directory/docker_runtime.txt"
  ros2 bag info "$BAG_PATH" > "$run_directory/input_bag_info.txt" 2>&1 || true
}

copy_effective_configurations() {
  local artifact_directory="$run_directory/artifacts"
  mkdir -p "$artifact_directory"
  printf 'role\tsource_path\tsha256\tartifact\n' \
    > "$artifact_directory/effective_configurations.tsv"

  local role source_path artifact_name digest
  while IFS=$'\t' read -r role source_path artifact_name; do
    digest="$(sha256sum "$source_path" | awk '{print $1}')"
    install -m 0644 "$source_path" "$artifact_directory/$artifact_name"
    printf '%s\t%s\t%s\t%s\n' \
      "$role" "$source_path" "$digest" "$artifact_name" \
      >> "$artifact_directory/effective_configurations.tsv"
  done <<EOF
imu_base	$IMU_PARAM	imu_param.yaml
odometry_base	$ODOM_PARAM	odom_param.yaml
odometry_override	$ODOM_OVERRIDE_PARAM	odom_override_param.yaml
nmea_base	$NMEA_GNSS_PARAM	nmea_gnss_param.yaml
nmea_projector_metadata	$NMEA_PROJECTOR_METADATA	map_projector_info.yaml
nmea_override	$NMEA_GNSS_OVERRIDE_PARAM	nmea_override_param.yaml
gnss_fusion_base	$GNSS_FUSION_PARAM	gnss_fusion_param.yaml
gnss_fusion_override	$GNSS_FUSION_OVERRIDE_PARAM	gnss_fusion_override_param.yaml
diagnostic_aggregator	$DIAGNOSTIC_AGGREGATOR_PARAM	diagnostic_aggregator.yaml
autoware_adapter_base	$AUTOWARE_ADAPTER_PARAM	autoware_adapter_param.yaml
EOF

  if [[ "$TRACKING_MODE" == scan_to_submap ]]; then
    for role_path_name in \
      "precision_matcher_base|$PRECISION_MATCHER_PARAM|precision_matcher_param.yaml" \
      "precision_matcher_override|$PRECISION_MATCHER_OVERRIDE_PARAM|precision_matcher_override_param.yaml" \
      "precision_global_base|$PRECISION_GLOBAL_PARAM|precision_global_param.yaml" \
      "precision_global_override|$PRECISION_GLOBAL_OVERRIDE_PARAM|precision_global_override_param.yaml"
    do
      IFS='|' read -r role source_path artifact_name <<< "$role_path_name"
      digest="$(sha256sum "$source_path" | awk '{print $1}')"
      install -m 0644 "$source_path" "$artifact_directory/$artifact_name"
      printf '%s\t%s\t%s\t%s\n' \
        "$role" "$source_path" "$digest" "$artifact_name" \
        >> "$artifact_directory/effective_configurations.tsv"
    done
  fi
}

wait_for_node() {
  local node="$1"
  local pid="$2"
  local deadline=$((SECONDS + STARTUP_WAIT_SEC))
  while ((SECONDS < deadline)); do
    kill -0 "$pid" 2>/dev/null || return 2
    if ros2 node list --no-daemon --spin-time 1 2>/dev/null | grep -Fxq "$node"; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

wait_for_topic() {
  local topic="$1"
  local owner_pid="${2:-$launch_pid}"
  local deadline=$((SECONDS + STARTUP_WAIT_SEC))
  while ((SECONDS < deadline)); do
    if [[ -n "$owner_pid" ]] && ! kill -0 "$owner_pid" 2>/dev/null; then
      return 2
    fi
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

missing_required_nodes=""
check_required_nodes() {
  missing_required_nodes=""
  local snapshot
  if ! snapshot="$(ros2 node list --no-daemon --spin-time 2 2>/dev/null)"; then
    missing_required_nodes="ROS graph query failed"
    return 1
  fi
  printf '%s\n' "$snapshot" > "$run_directory/final_nodes.txt"

  local required=(
    /pointcloud_container
    /pure_odometry_container
    /gyro_odometer
    /gnss_map_odom_fusion
    /autoware_localization_adapter
  )
  if [[ "$USE_IMU_DESKEW" == true ]]; then
    required+=(/imu_undistorter)
  fi
  if [[ "$USE_GNSS" == true ]]; then
    required+=(/nmea_gga_conversion)
  fi
  if [[ "$TRACKING_MODE" == scan_to_submap ]]; then
    required+=(/submap_matcher /precision_global_localizer)
  fi
  if [[ "$DATASET_PROFILE" == hesai_rosbag23 ]]; then
    required+=(
      /hesai_lidar_static_transform
      /hesai_imu_static_transform
      /hesai_gnss_static_transform
    )
  fi
  if [[ "$RVIZ" == true ]]; then
    required+=(/rviz2)
  fi

  local node
  local missing=()
  for node in "${required[@]}"; do
    if ! grep -Fxq "$node" <<< "$snapshot"; then
      missing+=("$node")
    fi
  done
  if [[ "$TRACKING_MODE" == scan_to_submap ]]; then
    local topic_snapshot
    if ! topic_snapshot="$(ros2 topic list --no-daemon --spin-time 2 2>/dev/null)"; then
      missing+=("precision ROS topic graph query failed")
    else
      local required_topic
      for required_topic in \
        /localization/submap_scan \
        /localization/submap_correction \
        /localization/precision_local_odom \
        /localization/precision_global_odom \
        /localization/precision_global_pose
      do
        if ! grep -Fxq "$required_topic" <<< "$topic_snapshot"; then
          missing+=("$required_topic")
        fi
      done
    fi
  fi
  if ((${#missing[@]} > 0)); then
    missing_required_nodes="${missing[*]}"
    return 1
  fi
  return 0
}

wait_for_required_nodes() {
  local deadline=$((SECONDS + STARTUP_WAIT_SEC))
  while ((SECONDS < deadline)); do
    if [[ -n "$launch_pid" ]] && ! kill -0 "$launch_pid" 2>/dev/null; then
      return 2
    fi
    if check_required_nodes; then
      return 0
    fi
    sleep 0.5
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
copy_effective_configurations

launch_command=(
  ros2 launch pure_odometry_bringup autoware_lsim_localization.launch.py
  launch_autoware:=true
  vehicle_model:="$VEHICLE_MODEL"
  sensor_model:="$SENSOR_MODEL"
  launch_vehicle:="$LAUNCH_VEHICLE"
  launch_sensing:="$LAUNCH_SENSING"
  launch_rviz:="$RVIZ"
  rviz_config:="$RVIZ_CONFIG"
  sensor_profile:="$SENSOR_PROFILE"
  use_gnss:="$USE_GNSS"
  use_imu_deskew:="$USE_IMU_DESKEW"
  points_input_topic:=/points_raw
  imu_input_topic:=/imu
  imu_param:="$IMU_PARAM"
  odom_param:="$ODOM_PARAM"
  odom_override_param:="$ODOM_OVERRIDE_PARAM"
  nmea_gnss_param:="$NMEA_GNSS_PARAM"
  nmea_gnss_override_param:="$NMEA_GNSS_OVERRIDE_PARAM"
  gnss_fusion_param:="$GNSS_FUSION_PARAM"
  gnss_fusion_override_param:="$GNSS_FUSION_OVERRIDE_PARAM"
  log_level:="$LOG_LEVEL"
)
if [[ -n "$TWIST_SOURCE_TOPIC" ]]; then
  launch_command+=(twist_input_topic:=/localization/input_twist)
fi

printf '[autoware-lsim] launch:'
printf ' %q' "${launch_command[@]}"
printf '\n'
stdbuf -oL -eL "${launch_command[@]}" \
  > >(tee "$run_directory/launch.log") \
  2>&1 &
launch_pid=$!

if [[ "$TRACKING_MODE" == scan_to_submap ]]; then
  precision_launch_command=(
    ros2 launch pure_precision_bringup precision_overlay.launch.py
    use_sim_time:=true
    matcher_param:="$PRECISION_MATCHER_PARAM"
    matcher_override_param:="$PRECISION_MATCHER_OVERRIDE_PARAM"
    global_param:="$PRECISION_GLOBAL_PARAM"
    global_override_param:="$PRECISION_GLOBAL_OVERRIDE_PARAM"
    log_level:="$LOG_LEVEL"
  )
  printf '[autoware-lsim] precision overlay:'
  printf ' %q' "${precision_launch_command[@]}"
  printf '\n'
  stdbuf -oL -eL "${precision_launch_command[@]}" \
    > >(tee "$run_directory/precision_launch.log") \
    2>&1 &
  precision_launch_pid=$!
fi

if ! wait_for_topic /localization/kinematic_state; then
  fail "Autoware/localizer launch did not create /localization/kinematic_state; see $run_directory/launch.log"
fi
if [[ "$TRACKING_MODE" == scan_to_submap ]]; then
  if ! wait_for_topic /localization/submap_scan "$launch_pid"; then
    fail "exact-key snapshot publisher did not start; see $run_directory/launch.log"
  fi
  if ! wait_for_topic /localization/submap_correction "$precision_launch_pid"; then
    fail "submap matcher did not start; see $run_directory/precision_launch.log"
  fi
  if ! wait_for_topic /localization/precision_local_odom "$precision_launch_pid"; then
    fail "precision local output did not start; see $run_directory/precision_launch.log"
  fi
  if ! wait_for_topic /localization/precision_global_odom "$precision_launch_pid"; then
    fail "precision global output did not start; see $run_directory/precision_launch.log"
  fi
  if ! wait_for_topic /localization/precision_global_pose "$precision_launch_pid"; then
    fail "precision global pose did not start; see $run_directory/precision_launch.log"
  fi
fi
if ! wait_for_required_nodes; then
  fail "required Autoware/localizer node(s) did not start: $missing_required_nodes"
fi
log "Autoware localization interface is ready."

if [[ "$RECORD_OUTPUT" == true ]]; then
  record_directory="$run_directory/localization_output"
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
    /localization/gnss_fusion_input
    /localization/gnss_odometry
    /localization/global_pose_with_covariance
    /localization/gnss_confidence
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
  if [[ "$TRACKING_MODE" == scan_to_submap ]]; then
    record_topics+=(
      /localization/submap_scan
      /localization/submap_correction
      /localization/precision_local_odom
      /localization/precision_global_odom
      /localization/precision_global_pose
    )
  fi
  log "recording evaluation outputs to $record_directory"
  ros2 bag record --storage mcap --output "$record_directory" \
    --node-name autoware_lsim_output_recorder --topics "${record_topics[@]}" \
    > "$run_directory/record.log" 2>&1 &
  record_pid=$!
  if ! wait_for_node /autoware_lsim_output_recorder "$record_pid"; then
    fail "output recorder did not become ready; see $run_directory/record.log"
  fi
  log "output recorder is ready."
fi

play_command=(
  "${GICP_GNSS_ODOM_INSTALL}/bin/play_localization_bag.sh"
  --bag "$BAG_PATH"
  --points "$POINTS_SOURCE_TOPIC"
  --imu "$IMU_SOURCE_TOPIC"
  --rate "$PLAYBACK_RATE"
  --clock-frequency "$CLOCK_FREQUENCY"
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
if [[ "$DATASET_PROFILE" == hesai_rosbag23 ]]; then
  play_command+=(
    --
    --disable-keyboard-controls
    --delay 2
    --topics
    "$POINTS_SOURCE_TOPIC"
    "$IMU_SOURCE_TOPIC"
    "$NMEA_SOURCE_TOPIC"
  )
else
  play_command+=(-- --disable-keyboard-controls --delay 2)
fi

printf '[autoware-lsim] replay:'
printf ' %q' "${play_command[@]}"
printf '\n'
stdbuf -oL -eL "${play_command[@]}" \
  > >(tee "$run_directory/replay.log") \
  2>&1 &
bag_pid=$!

if [[ "$USE_GNSS" != true && "$AUTO_INITIAL_POSE" == true ]]; then
  if timeout "$INITIALPOSE_DATA_WAIT_SEC" \
    ros2 topic echo --once /localization/gyro_lidar_odom > /dev/null 2>&1; then
    publish_initial_pose
  else
    warn "no LiDAR odometry arrived before the initial-pose timeout; fused Autoware output may remain unavailable"
  fi
fi

if timeout "$FIRST_STATE_WAIT_SEC" ros2 topic echo --once \
  /localization/kinematic_state > "$run_directory/first_kinematic_state.yaml" 2>&1
then
  log "received the first valid Autoware kinematic state."
else
  fail "no valid Autoware kinematic state arrived; see launch/replay logs"
fi

set +e
wait "$bag_pid"
bag_status=$?
set -e
bag_pid=""

# Let queued callbacks drain before stopping the recorder and launch. The
# analyzer below still enforces that the last state reaches the final /clock.
sleep "$DRAIN_WAIT_SEC"
launch_was_alive="false"
precision_launch_was_alive="true"
record_was_alive="true"
required_nodes_were_alive="true"
if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
  launch_was_alive="true"
fi
if [[ "$TRACKING_MODE" == scan_to_submap ]] &&
  { [[ -z "$precision_launch_pid" ]] || ! kill -0 "$precision_launch_pid" 2>/dev/null; }
then
  precision_launch_was_alive="false"
fi
if [[ "$RECORD_OUTPUT" == true ]] &&
  { [[ -z "$record_pid" ]] || ! kill -0 "$record_pid" 2>/dev/null; }
then
  record_was_alive="false"
fi
if ! check_required_nodes; then
  required_nodes_were_alive="false"
fi
if [[ "$RVIZ" == true && "$required_nodes_were_alive" == true ]]; then
  if ! ros2 node info /rviz2 > "$run_directory/rviz_node_info.txt" 2>&1; then
    required_nodes_were_alive="false"
    missing_required_nodes="/rviz2 node info failed"
  fi
fi
stop_process "$record_pid" INT
record_pid=""
stop_process "$precision_launch_pid" INT
precision_launch_pid=""
stop_process "$launch_pid" INT
launch_pid=""

if ((bag_status != 0)); then
  fail "rosbag playback exited with status $bag_status; see $run_directory/replay.log"
fi
if [[ "$launch_was_alive" != true ]]; then
  fail "Autoware/localizer launch exited before replay completed; see $run_directory/launch.log"
fi
if [[ "$precision_launch_was_alive" != true ]]; then
  fail "precision overlay exited before replay completed; see $run_directory/precision_launch.log"
fi
if [[ "$record_was_alive" != true ]]; then
  fail "output recorder exited before replay completed; see $run_directory/record.log"
fi
if [[ "$required_nodes_were_alive" != true ]]; then
  fail "required localization node(s) missing at replay completion: $missing_required_nodes"
fi

if [[ "$RECORD_OUTPUT" == true ]]; then
  validation_profile="${DATASET_PROFILE//_/-}"
  log "validating recorded Autoware localization output."
  if ! "${GICP_GNSS_ODOM_INSTALL}/bin/analyze_autoware_lsim_output.py" \
    "$record_directory" --profile "$validation_profile" \
    --tracking-mode scan_to_scan \
    2>&1 | tee "$run_directory/validation.log"
  then
    fail "recorded output failed acceptance checks; see $run_directory/validation.log"
  fi
  if [[ "$TRACKING_MODE" == scan_to_submap ]]; then
    if ! ros2 run pure_precision_bringup validate_precision_bag.py \
      "$record_directory" --expected-rate "$PLAYBACK_RATE" \
      > "$run_directory/precision_validation.log" 2>&1
    then
      fail "precision output failed acceptance checks; see $run_directory/precision_validation.log"
    fi
    log "precision output validation passed."
  fi
fi
log "evaluation complete: $run_directory"
