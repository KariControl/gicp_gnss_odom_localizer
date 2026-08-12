#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -Eeuo pipefail

usage() {
  cat <<'USAGE'
Run the ROS 2 LiDAR/IMU/NMEA localization stack against a Hesai rosbag.

Usage:
  run_hesai_localization_bag.sh [options]

Options:
  --dataset <name>             course-1 | course-2 (default: course-1)
                               Selects a descriptive private-dataset manifest
                               and its local default bag path
  --bag <path>                 Input rosbag directory
                               Overrides the dataset's local default path
  --rate <factor>             Playback rate (default: 1.0)
  --localization-mode <mode>  baseline | precision (default: baseline)
                               precision starts the isolated submap-matcher
                               overlay; the odometer remains scan_to_scan
  --tracking-mode <mode>      Deprecated compatibility alias:
                               scan_to_scan -> baseline
                               scan_to_submap -> precision
  --accepted-scan-control     Evaluation-only scan_to_scan control: enable and
                              record the same accepted-scan snapshots as the
                              precision run, without starting its overlay
  --output <directory>        Result directory
                               default: test_results/<dataset>_<timestamp>
  --playback-duration <sec>   Stop after this many seconds of bag time
  --lsim-interface-test       Run the LSim localizer/Autoware adapter launch
                              without requiring the Autoware installation
  --no-record                 Run without recording localization outputs
  --dry-run                   Validate inputs and print commands only
  -h, --help                  Show this help

The default profile uses LiDAR and IMU only for local odometry. It neither
subscribes to /vehicle/twist nor supplies another translational deskew speed.
Both datasets use the calibrated Hesai 32-Line + IMU + RTK GNSS rig. Rosbags are
private evaluation inputs and are not distributed with this repository.
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
dataset="course-1"
dataset_was_set=false
bag=""
rate="1.0"
localization_mode="baseline"
mode_option=""
output=""
playback_duration=""
record_output=true
lsim_interface_test=false
dry_run=false
accepted_scan_control=false

while (($# > 0)); do
  case "$1" in
    --dataset)
      [[ $# -ge 2 ]] || fail "--dataset requires a value"
      dataset="$2"
      dataset_was_set=true
      shift 2
      ;;
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
    --localization-mode)
      [[ $# -ge 2 ]] || fail "--localization-mode requires a value"
      [[ -z "$mode_option" ]] ||
        fail "--localization-mode cannot be combined with $mode_option"
      localization_mode="$2"
      mode_option="--localization-mode"
      shift 2
      ;;
    --tracking-mode)
      [[ $# -ge 2 ]] || fail "--tracking-mode requires a value"
      [[ -z "$mode_option" ]] ||
        fail "--tracking-mode cannot be combined with $mode_option"
      case "$2" in
        scan_to_scan) localization_mode="baseline" ;;
        scan_to_submap) localization_mode="precision" ;;
        *) fail "--tracking-mode must be scan_to_scan or scan_to_submap: $2" ;;
      esac
      mode_option="--tracking-mode"
      printf '%s\n' \
        '[vehicle-localizer] WARNING: --tracking-mode is deprecated; use --localization-mode baseline|precision' \
        >&2
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
    --accepted-scan-control)
      accepted_scan_control=true
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
case "$dataset" in
  course-1)
    dataset_id="course_1"
    dataset_display_name="Hesai 32-Line + IMU + RTK GNSS — Course 1"
    dataset_default_bag="$ROOT/rosbag/output_pointcloud2"
    ;;
  course-2)
    dataset_id="course_2"
    dataset_display_name="Hesai 32-Line + IMU + RTK GNSS — Course 2"
    dataset_default_bag="$ROOT/rosbag/output_pointcloud3"
    ;;
  *) fail "--dataset must be course-1 or course-2: $dataset" ;;
esac
if [[ -z "$bag" ]]; then
  bag="$dataset_default_bag"
elif [[ "$dataset_was_set" != true ]]; then
  case "$(basename -- "$bag")" in
    output_pointcloud2)
      dataset="course-1"
      dataset_id="course_1"
      dataset_display_name="Hesai 32-Line + IMU + RTK GNSS — Course 1"
      ;;
    output_pointcloud3)
      dataset="course-2"
      dataset_id="course_2"
      dataset_display_name="Hesai 32-Line + IMU + RTK GNSS — Course 2"
      ;;
  esac
fi
case "$localization_mode" in
  baseline|precision) ;;
  *) fail "--localization-mode must be baseline or precision: $localization_mode" ;;
esac
if [[ -n "$playback_duration" ]]; then
  require_positive_number "--playback-duration" "$playback_duration"
fi
if [[ "$accepted_scan_control" == true ]]; then
  [[ "$localization_mode" == "baseline" ]] ||
    fail "--accepted-scan-control requires --localization-mode baseline"
  [[ "$record_output" == true ]] ||
    fail "--accepted-scan-control requires recording"
fi

[[ -e "$bag" ]] || fail "bag does not exist: $bag"
bag="$(realpath -e -- "$bag")"
if [[ -z "$output" ]]; then
  output="$ROOT/test_results/${dataset_id}_$(date +%Y%m%d_%H%M%S)"
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
EVALUATION_PROFILE_ROOT="$BRINGUP_SHARE/config/evaluation/lidar_imu_gnss/hesai_32line_rtk"
DATASET_MANIFEST="$EVALUATION_PROFILE_ROOT/datasets/${dataset_id}.yaml"
DIAGNOSTIC_AGGREGATOR_PARAM="$BRINGUP_SHARE/config/diagnostic_aggregator.yaml"
ODOM_OVERRIDE_PARAM="$BRINGUP_SHARE/config/autoware_lsim/empty_params.yaml"
PRECISION_BRINGUP_SHARE=""
PRECISION_MATCHER_PARAM=""
PRECISION_MATCHER_OVERRIDE_PARAM=""
PRECISION_GLOBAL_PARAM=""
PRECISION_GLOBAL_OVERRIDE_PARAM=""
LSIM_ADAPTER_PARAM=""
if [[ "$localization_mode" == "precision" || "$accepted_scan_control" == true ]]; then
  PRECISION_BRINGUP_SHARE="$(ros2 pkg prefix pure_precision_bringup)/share/pure_precision_bringup"
  ODOM_OVERRIDE_PARAM="$PRECISION_BRINGUP_SHARE/config/submap_snapshot_override.yaml"
fi
if [[ "$localization_mode" == "precision" ]]; then
  PRECISION_MATCHER_PARAM="$(ros2 pkg prefix pure_lidar_submap_matcher)/share/pure_lidar_submap_matcher/param/param.yaml"
  PRECISION_GLOBAL_PARAM="$(ros2 pkg prefix pure_precision_global_localizer)/share/pure_precision_global_localizer/param/param.yaml"
  PRECISION_MATCHER_OVERRIDE_PARAM="$PRECISION_BRINGUP_SHARE/config/empty_params.yaml"
  PRECISION_GLOBAL_OVERRIDE_PARAM="$PRECISION_BRINGUP_SHARE/config/empty_params.yaml"
fi
if [[ "$lsim_interface_test" == true ]]; then
  LSIM_ADAPTER_PARAM="$(ros2 pkg prefix pure_autoware_localization_adapter)/share/pure_autoware_localization_adapter/param/param.yaml"
fi
NMEA_GNSS_OVERRIDE_PARAM="$EVALUATION_PROFILE_ROOT/accepted/nmea_site_origin.yaml"
GNSS_FUSION_OVERRIDE_PARAM="$EVALUATION_PROFILE_ROOT/accepted/gnss_fusion_single_antenna.yaml"
PRECISION_PROFILE_MANIFEST=""
if [[ -n "$PRECISION_BRINGUP_SHARE" ]]; then
  PRECISION_PROFILE_MANIFEST="$PRECISION_BRINGUP_SHARE/config/evaluation/lidar_imu_gnss/hesai_32line_rtk/profile.yaml"
fi
for parameter_file in \
  "$IMU_PARAM" \
  "$ODOM_PARAM" \
  "$ODOM_OVERRIDE_PARAM" \
  "$GNSS_FUSION_PARAM" \
  "$NMEA_GNSS_PARAM" \
  "$NMEA_GNSS_OVERRIDE_PARAM" \
  "$GNSS_FUSION_OVERRIDE_PARAM" \
  "$DATASET_MANIFEST" \
  "$DIAGNOSTIC_AGGREGATOR_PARAM"
do
  [[ -f "$parameter_file" ]] || fail "parameter file does not exist: $parameter_file"
done
if [[ -n "$PRECISION_PROFILE_MANIFEST" ]]; then
  [[ -f "$PRECISION_PROFILE_MANIFEST" ]] ||
    fail "profile manifest does not exist: $PRECISION_PROFILE_MANIFEST"
fi
if [[ "$localization_mode" == "precision" ]]; then
  [[ -f "$PRECISION_MATCHER_PARAM" ]] ||
    fail "parameter file does not exist: $PRECISION_MATCHER_PARAM"
  [[ -f "$PRECISION_GLOBAL_PARAM" ]] ||
    fail "parameter file does not exist: $PRECISION_GLOBAL_PARAM"
  [[ -f "$PRECISION_MATCHER_OVERRIDE_PARAM" ]] ||
    fail "parameter file does not exist: $PRECISION_MATCHER_OVERRIDE_PARAM"
  [[ -f "$PRECISION_GLOBAL_OVERRIDE_PARAM" ]] ||
    fail "parameter file does not exist: $PRECISION_GLOBAL_OVERRIDE_PARAM"
fi
if [[ -n "$LSIM_ADAPTER_PARAM" ]]; then
  [[ -f "$LSIM_ADAPTER_PARAM" ]] ||
    fail "parameter file does not exist: $LSIM_ADAPTER_PARAM"
fi

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
    odom_override_param:="$ODOM_OVERRIDE_PARAM"
    gnss_fusion_param:="$GNSS_FUSION_PARAM"
    gnss_fusion_override_param:="$GNSS_FUSION_OVERRIDE_PARAM"
    nmea_gnss_param:="$NMEA_GNSS_PARAM"
    nmea_gnss_override_param:="$NMEA_GNSS_OVERRIDE_PARAM"
    gnss_primary_gga_topic:=/nmea_sentence
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
    odom_override_param:="$ODOM_OVERRIDE_PARAM"
    gnss_fusion_param:="$GNSS_FUSION_PARAM"
    gnss_fusion_override_param:="$GNSS_FUSION_OVERRIDE_PARAM"
    nmea_gnss_param:="$NMEA_GNSS_PARAM"
    nmea_gnss_override_param:="$NMEA_GNSS_OVERRIDE_PARAM"
    gnss_primary_gga_topic:=/nmea_sentence
    use_secondary_gga:=false
    use_doppler_heading:=false
    use_imu_yaw_rate_heading:=true
  )
fi
precision_launch_command=()
if [[ "$localization_mode" == "precision" ]]; then
  precision_launch_command=(
    ros2 launch pure_precision_bringup precision_overlay.launch.py
    use_sim_time:=true
    matcher_param:="$PRECISION_MATCHER_PARAM"
    matcher_override_param:="$PRECISION_MATCHER_OVERRIDE_PARAM"
    global_param:="$PRECISION_GLOBAL_PARAM"
    global_override_param:="$PRECISION_GLOBAL_OVERRIDE_PARAM"
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
if [[ "$localization_mode" == "precision" || "$accepted_scan_control" == true ]]; then
  record_topics+=(
    /localization/submap_scan
  )
fi
if [[ "$localization_mode" == "precision" ]]; then
  record_topics+=(
    /localization/submap_correction
    /localization/precision_local_odom
    /localization/precision_global_odom
    /localization/precision_global_pose
  )
fi
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
log "dataset: $dataset_display_name"
log "evaluation profile: hesai_32line_rtk"
log "output: $output"
log "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
log "localization mode: $localization_mode"
log "odometer tracking mode: scan_to_scan"
log "accepted-scan control instrumentation: $accepted_scan_control"

if [[ "$dry_run" == true ]]; then
  if [[ "$lsim_interface_test" != true ]]; then
    print_command "${tf_lidar_command[@]}"
    print_command "${tf_imu_command[@]}"
    print_command "${tf_gnss_command[@]}"
  fi
  print_command "${launch_command[@]}"
  if [[ "$localization_mode" == "precision" ]]; then
    print_command "${precision_launch_command[@]}"
  fi
  if [[ "$record_output" == true ]]; then
    print_command "${record_command[@]}"
  fi
  print_command "${play_command[@]}"
  exit 0
fi

mkdir -p "$output/ros_logs"
export ROS_LOG_DIR="$output/ros_logs"
artifact_directory="$output/artifacts"
mkdir -p "$artifact_directory"

sha256_of() {
  sha256sum "$1" | awk '{print $1}'
}

copy_effective_config() {
  local role="$1"
  local source_path="$2"
  local artifact_name="$3"
  local digest
  digest="$(sha256_of "$source_path")"
  install -m 0644 "$source_path" "$artifact_directory/$artifact_name"
  printf '%s\t%s\t%s\t%s\n' \
    "$role" "$source_path" "$digest" "$artifact_name" \
    >> "$artifact_directory/effective_configurations.tsv"
}

printf 'role\tsource_path\tsha256\tartifact\n' \
  > "$artifact_directory/effective_configurations.tsv"
copy_effective_config imu_base "$IMU_PARAM" imu_param.yaml
copy_effective_config odometry_base "$ODOM_PARAM" odom_param.yaml
copy_effective_config odometry_override "$ODOM_OVERRIDE_PARAM" odom_override_param.yaml
copy_effective_config nmea_base "$NMEA_GNSS_PARAM" nmea_gnss_param.yaml
copy_effective_config nmea_site_override "$NMEA_GNSS_OVERRIDE_PARAM" nmea_site_origin.yaml
copy_effective_config gnss_fusion_base "$GNSS_FUSION_PARAM" gnss_fusion_param.yaml
copy_effective_config gnss_fusion_override "$GNSS_FUSION_OVERRIDE_PARAM" \
  gnss_fusion_single_antenna.yaml
copy_effective_config dataset_manifest "$DATASET_MANIFEST" dataset_manifest.yaml
copy_effective_config diagnostic_aggregator "$DIAGNOSTIC_AGGREGATOR_PARAM" \
  diagnostic_aggregator.yaml
if [[ -n "$LSIM_ADAPTER_PARAM" ]]; then
  copy_effective_config autoware_adapter_base "$LSIM_ADAPTER_PARAM" \
    autoware_adapter_param.yaml
fi
if [[ -n "$PRECISION_PROFILE_MANIFEST" ]]; then
  copy_effective_config precision_profile_manifest "$PRECISION_PROFILE_MANIFEST" \
    precision_profile.yaml
fi
if [[ "$localization_mode" == "precision" ]]; then
  copy_effective_config precision_matcher_base "$PRECISION_MATCHER_PARAM" \
    precision_matcher_param.yaml
  copy_effective_config precision_matcher_override \
    "$PRECISION_MATCHER_OVERRIDE_PARAM" precision_matcher_override_param.yaml
  copy_effective_config precision_global_base "$PRECISION_GLOBAL_PARAM" \
    precision_global_param.yaml
  copy_effective_config precision_global_override \
    "$PRECISION_GLOBAL_OVERRIDE_PARAM" precision_global_override_param.yaml
fi

IMU_PARAM_SHA256="$(sha256_of "$IMU_PARAM")"
ODOM_PARAM_SHA256="$(sha256_of "$ODOM_PARAM")"
ODOM_OVERRIDE_PARAM_SHA256="$(sha256_of "$ODOM_OVERRIDE_PARAM")"
NMEA_GNSS_PARAM_SHA256="$(sha256_of "$NMEA_GNSS_PARAM")"
NMEA_GNSS_OVERRIDE_PARAM_SHA256="$(sha256_of "$NMEA_GNSS_OVERRIDE_PARAM")"
GNSS_FUSION_PARAM_SHA256="$(sha256_of "$GNSS_FUSION_PARAM")"
GNSS_FUSION_OVERRIDE_PARAM_SHA256="$(sha256_of "$GNSS_FUSION_OVERRIDE_PARAM")"
DATASET_MANIFEST_SHA256="$(sha256_of "$DATASET_MANIFEST")"
PRECISION_MATCHER_PARAM_SHA256="n/a"
PRECISION_GLOBAL_PARAM_SHA256="n/a"
PRECISION_MATCHER_OVERRIDE_PARAM_SHA256="n/a"
PRECISION_GLOBAL_OVERRIDE_PARAM_SHA256="n/a"
PRECISION_PROFILE_MANIFEST_SHA256="n/a"
DIAGNOSTIC_AGGREGATOR_PARAM_SHA256="$(sha256_of "$DIAGNOSTIC_AGGREGATOR_PARAM")"
LSIM_ADAPTER_PARAM_SHA256="n/a"
if [[ -n "$LSIM_ADAPTER_PARAM" ]]; then
  LSIM_ADAPTER_PARAM_SHA256="$(sha256_of "$LSIM_ADAPTER_PARAM")"
fi
if [[ -n "$PRECISION_PROFILE_MANIFEST" ]]; then
  PRECISION_PROFILE_MANIFEST_SHA256="$(sha256_of "$PRECISION_PROFILE_MANIFEST")"
fi
if [[ "$localization_mode" == "precision" ]]; then
  PRECISION_MATCHER_PARAM_SHA256="$(sha256_of "$PRECISION_MATCHER_PARAM")"
  PRECISION_GLOBAL_PARAM_SHA256="$(sha256_of "$PRECISION_GLOBAL_PARAM")"
  PRECISION_MATCHER_OVERRIDE_PARAM_SHA256="$(sha256_of "$PRECISION_MATCHER_OVERRIDE_PARAM")"
  PRECISION_GLOBAL_OVERRIDE_PARAM_SHA256="$(sha256_of "$PRECISION_GLOBAL_OVERRIDE_PARAM")"
fi
{
  printf 'evaluation_profile=%q\n' "hesai_32line_rtk"
  printf 'dataset=%q\n' "$dataset"
  printf 'dataset_id=%q\n' "$dataset_id"
  printf 'dataset_display_name=%q\n' "$dataset_display_name"
  printf 'dataset_manifest=%q\n' "$DATASET_MANIFEST"
  printf 'dataset_manifest_sha256=%q\n' "$DATASET_MANIFEST_SHA256"
  printf 'bag=%q\n' "$bag"
  printf 'rate=%q\n' "$rate"
  printf 'localization_mode=%q\n' "$localization_mode"
  printf 'odometer_tracking_mode=%q\n' "scan_to_scan"
  printf 'accepted_scan_control=%q\n' "$accepted_scan_control"
  printf 'odom_param=%q\n' "$ODOM_PARAM"
  printf 'odom_param_sha256=%q\n' "$ODOM_PARAM_SHA256"
  printf 'odom_override_param=%q\n' "$ODOM_OVERRIDE_PARAM"
  printf 'odom_override_param_sha256=%q\n' "$ODOM_OVERRIDE_PARAM_SHA256"
  printf 'imu_param=%q\n' "$IMU_PARAM"
  printf 'imu_param_sha256=%q\n' "$IMU_PARAM_SHA256"
  printf 'nmea_gnss_param=%q\n' "$NMEA_GNSS_PARAM"
  printf 'nmea_gnss_param_sha256=%q\n' "$NMEA_GNSS_PARAM_SHA256"
  printf 'nmea_gnss_override_param=%q\n' "$NMEA_GNSS_OVERRIDE_PARAM"
  printf 'nmea_gnss_override_param_sha256=%q\n' "$NMEA_GNSS_OVERRIDE_PARAM_SHA256"
  printf 'gnss_fusion_param=%q\n' "$GNSS_FUSION_PARAM"
  printf 'gnss_fusion_param_sha256=%q\n' "$GNSS_FUSION_PARAM_SHA256"
  printf 'gnss_fusion_override_param=%q\n' "$GNSS_FUSION_OVERRIDE_PARAM"
  printf 'gnss_fusion_override_param_sha256=%q\n' "$GNSS_FUSION_OVERRIDE_PARAM_SHA256"
  printf 'precision_matcher_param=%q\n' "$PRECISION_MATCHER_PARAM"
  printf 'precision_matcher_param_sha256=%q\n' "$PRECISION_MATCHER_PARAM_SHA256"
  printf 'precision_matcher_override_param=%q\n' "$PRECISION_MATCHER_OVERRIDE_PARAM"
  printf 'precision_matcher_override_param_sha256=%q\n' "$PRECISION_MATCHER_OVERRIDE_PARAM_SHA256"
  printf 'precision_global_param=%q\n' "$PRECISION_GLOBAL_PARAM"
  printf 'precision_global_param_sha256=%q\n' "$PRECISION_GLOBAL_PARAM_SHA256"
  printf 'precision_global_override_param=%q\n' "$PRECISION_GLOBAL_OVERRIDE_PARAM"
  printf 'precision_global_override_param_sha256=%q\n' "$PRECISION_GLOBAL_OVERRIDE_PARAM_SHA256"
  printf 'precision_profile_manifest=%q\n' "$PRECISION_PROFILE_MANIFEST"
  printf 'precision_profile_manifest_sha256=%q\n' "$PRECISION_PROFILE_MANIFEST_SHA256"
  printf 'diagnostic_aggregator_param=%q\n' "$DIAGNOSTIC_AGGREGATOR_PARAM"
  printf 'diagnostic_aggregator_param_sha256=%q\n' "$DIAGNOSTIC_AGGREGATOR_PARAM_SHA256"
  printf 'lsim_adapter_param=%q\n' "$LSIM_ADAPTER_PARAM"
  printf 'lsim_adapter_param_sha256=%q\n' "$LSIM_ADAPTER_PARAM_SHA256"
  printf 'tf_policy=%q\n' "isolate_all"
  printf 'tf_base_to_lidar_xyz_rpy=%q\n' "0.0 0.0 0.0 0.0 0.0 0.0"
  printf 'tf_base_to_imu_xyz_rpy=%q\n' "0.0 0.0 -0.1874 3.14159 0.0 0.0"
  printf 'tf_base_to_gnss_xyz_rpy=%q\n' "0.0 0.0 -0.1326 0.0 0.0 0.0"
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
precision_launch_pid=""
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
  stop_process_group "$precision_launch_pid" "precision overlay launch"
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
  local owner_pid="${3:-$launch_pid}"
  local deadline=$((SECONDS + timeout_sec))
  while ((SECONDS < deadline)); do
    process_alive "$owner_pid" || return 2
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
if [[ "$localization_mode" == "precision" ]]; then
  start_process_group "$output/precision_launch.log" "${precision_launch_command[@]}"
  precision_launch_pid=$started_pid
fi

if ! wait_for_topic /localization/gyro_lidar_odom 60; then
  fail "localization stack did not become ready; see $output/launch.log"
fi
if ! wait_for_topic /localization/gnss_fusion_input 60; then
  fail "GNSS frontend did not become ready; see $output/launch.log"
fi
if ! wait_for_topic /localization/ekf_odom 60; then
  fail "GNSS fusion did not become ready; see $output/launch.log"
fi
if [[ "$localization_mode" == "precision" || "$accepted_scan_control" == true ]]; then
  if ! wait_for_topic /localization/submap_scan 60 "$launch_pid"; then
    fail "exact-key scan publisher did not become ready; see $output/launch.log"
  fi
fi
if [[ "$localization_mode" == "precision" ]]; then
  if ! wait_for_topic /localization/submap_correction 60 "$precision_launch_pid"; then
    fail "submap matcher did not become ready; see $output/precision_launch.log"
  fi
  if ! wait_for_topic /localization/precision_local_odom 60 "$precision_launch_pid"; then
    fail "precision localizer did not become ready; see $output/precision_launch.log"
  fi
  if ! wait_for_topic /localization/precision_global_odom 60 "$precision_launch_pid"; then
    fail "precision global output did not become ready; see $output/precision_launch.log"
  fi
  if ! wait_for_topic /localization/precision_global_pose 60 "$precision_launch_pid"; then
    fail "precision global pose did not become ready; see $output/precision_launch.log"
  fi
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
# can write the last messages and finalize metadata.yaml. Precision mode waits
# for at least one complete 1 Hz diagnostics period so bag/diagnostic counters
# can be checked exactly.
if [[ "$localization_mode" == "precision" ]]; then
  sleep 2
else
  sleep 1
fi
precision_runtime_healthy=true
precision_runtime_detail=""
if [[ "$localization_mode" == "precision" ]]; then
  if ! process_alive "$precision_launch_pid"; then
    precision_runtime_healthy=false
    precision_runtime_detail+=" precision_overlay_exited"
  fi
  precision_nodes="$(ros2 node list --no-daemon --spin-time 2 2>/dev/null || true)"
  precision_topics="$(ros2 topic list --no-daemon --spin-time 2 2>/dev/null || true)"
  for required_node in /submap_matcher /precision_global_localizer; do
    if ! grep -Fx "$required_node" <<< "$precision_nodes" >/dev/null; then
      precision_runtime_healthy=false
      precision_runtime_detail+=" missing_node=$required_node"
    fi
  done
  for required_topic in \
    /localization/submap_scan \
    /localization/submap_correction \
    /localization/precision_local_odom \
    /localization/precision_global_odom \
    /localization/precision_global_pose
  do
    if ! grep -Fx "$required_topic" <<< "$precision_topics" >/dev/null; then
      precision_runtime_healthy=false
      precision_runtime_detail+=" missing_topic=$required_topic"
    fi
  done
fi
stop_process_group "$record_pid" "bag recorder"
record_pid=""
stop_process_group "$precision_launch_pid" "precision overlay launch"
precision_launch_pid=""
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
if [[ "$precision_runtime_healthy" != true ]]; then
  fail "precision runtime health check failed:$precision_runtime_detail"
fi
if [[ "$localization_mode" == "precision" && "$record_output" == true ]]; then
  if ! ros2 run pure_precision_bringup validate_precision_bag.py \
    "$record_directory" --expected-rate "$rate" \
    > "$output/precision_validation.log" 2>&1
  then
    fail "precision output validation failed; see $output/precision_validation.log"
  fi
  log "precision output validation passed"
fi
log "localization run completed: $output"
