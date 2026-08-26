#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

usage() {
  cat <<'USAGE'
Replay a localization bag on stable input topics while isolating recorded localization outputs.

Usage:
  play_localization_bag.sh --bag <bag_path> [options] [-- <extra ros2 bag play options>]

Options:
  --points <topic>           Source PointCloud2 topic (default: /points_raw)
  --imu <topic>              Source IMU topic (default: /imu)
  --nmea <topic>             Source primary NMEA sentence topic; empty disables remap
  --nmea-secondary <topic>   Source secondary NMEA sentence topic; empty disables remap
  --fix-velocity <topic>     Source Doppler/fix-velocity topic; empty disables remap
  --twist <topic>            Source deskew twist topic; empty disables remap
  --rate <factor>            Playback rate (default: 1.0)
  --clock-frequency <hz>     Publish /clock at this rate; when omitted rosbag2
                             uses its default rate
  --tf-policy <policy>       keep | isolate-dynamic | isolate-all
                             default: isolate-dynamic
  --keep-recorded-localization
                             Do not remap recorded estimator/Autoware outputs
  --dry-run                  Print the ros2 bag play command without executing it
  -h, --help                 Show this help

Stable destinations:
  point cloud      /points_raw
  IMU              /imu
  primary NMEA     /nmea_sentence
  secondary NMEA   /nmea_sentence_secondary
  fix velocity     /ublox_gps_node/fix_velocity
  deskew twist     /localization/input_twist

The default TF policy keeps /tf_static for calibrated sensor extrinsics and moves
recorded dynamic /tf to /reference/tf so that recorded localization does not
conflict with the estimator under test. Use isolate-all only when the launched
vehicle/sensor model or selected calibrated profile provides every required
static transform.
USAGE
}

bag=""
points_topic="/points_raw"
imu_topic="/imu"
nmea_topic=""
nmea_secondary_topic=""
fix_velocity_topic=""
twist_topic=""
rate="1.0"
clock_frequency=""
tf_policy="isolate-dynamic"
keep_recorded_localization=false
dry_run=false
extra_args=()

while (($# > 0)); do
  case "$1" in
    --bag)
      [[ $# -ge 2 ]] || { echo "--bag requires a value" >&2; exit 2; }
      bag="$2"; shift 2 ;;
    --points)
      [[ $# -ge 2 ]] || { echo "--points requires a value" >&2; exit 2; }
      points_topic="$2"; shift 2 ;;
    --imu)
      [[ $# -ge 2 ]] || { echo "--imu requires a value" >&2; exit 2; }
      imu_topic="$2"; shift 2 ;;
    --nmea)
      [[ $# -ge 2 ]] || { echo "--nmea requires a value" >&2; exit 2; }
      nmea_topic="$2"; shift 2 ;;
    --nmea-secondary)
      [[ $# -ge 2 ]] || { echo "--nmea-secondary requires a value" >&2; exit 2; }
      nmea_secondary_topic="$2"; shift 2 ;;
    --fix-velocity)
      [[ $# -ge 2 ]] || { echo "--fix-velocity requires a value" >&2; exit 2; }
      fix_velocity_topic="$2"; shift 2 ;;
    --twist)
      [[ $# -ge 2 ]] || { echo "--twist requires a value" >&2; exit 2; }
      twist_topic="$2"; shift 2 ;;
    --rate)
      [[ $# -ge 2 ]] || { echo "--rate requires a value" >&2; exit 2; }
      rate="$2"; shift 2 ;;
    --clock-frequency)
      [[ $# -ge 2 ]] || { echo "--clock-frequency requires a value" >&2; exit 2; }
      clock_frequency="$2"; shift 2 ;;
    --tf-policy)
      [[ $# -ge 2 ]] || { echo "--tf-policy requires a value" >&2; exit 2; }
      tf_policy="$2"; shift 2 ;;
    --keep-recorded-localization)
      keep_recorded_localization=true; shift ;;
    --dry-run)
      dry_run=true; shift ;;
    --)
      shift
      extra_args=("$@")
      break ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2 ;;
  esac
done

[[ -n "$bag" ]] || { echo "--bag is required" >&2; usage >&2; exit 2; }
[[ -e "$bag" ]] || { echo "Bag does not exist: $bag" >&2; exit 2; }
case "$tf_policy" in
  keep|isolate-dynamic|isolate-all) ;;
  *) echo "Invalid --tf-policy: $tf_policy" >&2; exit 2 ;;
esac
if ! [[ "$rate" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
  echo "Invalid playback rate: $rate" >&2
  exit 2
fi
if ! awk -v value="$rate" 'BEGIN { exit !(value > 0.0) }'; then
  echo "Playback rate must be greater than zero: $rate" >&2
  exit 2
fi
if [[ -n "$clock_frequency" ]]; then
  if ! [[ "$clock_frequency" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] ||
    ! awk -v value="$clock_frequency" 'BEGIN { exit !(value > 0.0) }'
  then
    echo "Clock frequency must be a positive number: $clock_frequency" >&2
    exit 2
  fi
fi

remaps=()
add_remap() {
  local source="$1"
  local destination="$2"
  if [[ -n "$source" && "$source" != "$destination" ]]; then
    # Explicit sensor/input mappings are added before output isolation. Keep
    # the first mapping for a source so a recorded Autoware output selected as
    # an input (notably twist-with-covariance) cannot be redirected twice.
    local existing prefix="${source}:="
    for existing in "${remaps[@]}"; do
      [[ "$existing" == "$prefix"* ]] && return
    done
    remaps+=("${source}:=${destination}")
  fi
}

add_remap "$points_topic" "/points_raw"
add_remap "$imu_topic" "/imu"
add_remap "$nmea_topic" "/nmea_sentence"
add_remap "$nmea_secondary_topic" "/nmea_sentence_secondary"
add_remap "$fix_velocity_topic" "/ublox_gps_node/fix_velocity"
add_remap "$twist_topic" "/localization/input_twist"

if [[ "$keep_recorded_localization" != true ]]; then
  recorded_outputs=(
    /initialpose
    /localization/gyro_lidar_odom
    /localization/points_undistorted
    /localization/imu_corrected
    /localization/is_stopped
    /localization/ekf_odom
    /localization/ekf_pose
    /localization/gnss_map_odom_fusion_authority
    /localization/gnss_fusion_input
    /localization/gnss_odometry
    /localization/global_pose
    /localization/global_pose_with_covariance
    /localization/gnss_confidence
    /localization/kinematic_state
    /localization/twist_with_covariance
    /localization/acceleration
    /localization/pose_estimator/pose_with_covariance
    /diagnostics
    /diagnostics_agg
  )
  for topic in "${recorded_outputs[@]}"; do
    add_remap "$topic" "/reference${topic}"
  done
fi

case "$tf_policy" in
  keep) ;;
  isolate-dynamic)
    add_remap "/tf" "/reference/tf" ;;
  isolate-all)
    add_remap "/tf" "/reference/tf"
    add_remap "/tf_static" "/reference/tf_static" ;;
esac

command=(ros2 bag play "$bag" --clock)
if [[ -n "$clock_frequency" ]]; then
  command+=("$clock_frequency")
fi
command+=(--rate "$rate")
if ((${#remaps[@]} > 0)); then
  command+=(--remap "${remaps[@]}")
fi
command+=("${extra_args[@]}")

printf 'Executing:'
printf ' %q' "${command[@]}"
printf '\n'
if [[ "$dry_run" == true ]]; then
  exit 0
fi
exec "${command[@]}"
