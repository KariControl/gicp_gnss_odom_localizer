#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

usage() {
  cat <<'USAGE'
Run the isolated LiDAR/IMU-only odometer against a known bag and GLIM reference.

Usage:
  run_lidar_imu_glim_bag.sh --sensor velodyne|mid360 --output <directory> [options]

Options:
  --deskew <mode>           Velodyne: off | 41ms | 61ms (default: 41ms)
                            MID360: off | on (default: on)
  --gicp-epsilon <mode>     MID360: strict | default
                            (adaptive default: strict; fixed requires default)
  --mid-yaw-policy <mode>   MID360: fixed-bias-direct | adaptive
                            (default bag: fixed-bias-direct; adaptive is experimental)
  --localization-mode <m>   baseline | precision (default: baseline)
                            precision adds the isolated external submap branch
  --snapshot-interval <n>   Precision snapshot interval: 2 | 5 (default: 2)
  --matcher-override <yaml> Optional precision matcher parameter override
  --bag <directory>         Override the sensor's default input bag
  --rate <factor>           rosbag playback rate (default: 1.0)
  --playback-duration <s>   Limit playback for a smoke test
  --no-evaluate             Skip GLIM evaluation after recording
  --dry-run                 Print commands without starting ROS processes
  -h, --help                Show this help

Recorded TF is always isolated.  The runner publishes only the audited static
sensor transforms and never starts GNSS or map->odom fusion.
USAGE
}

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sensor=""
output=""
deskew=""
gicp_epsilon=""
mid_yaw_policy=""
localization_mode="baseline"
snapshot_interval=""
matcher_override=""
bag_override=""
rate="1.0"
playback_duration=""
evaluate=true
dry_run=false

while (($# > 0)); do
  case "$1" in
    --sensor)
      [[ $# -ge 2 ]] || { echo "--sensor requires a value" >&2; exit 2; }
      sensor="$2"; shift 2 ;;
    --output)
      [[ $# -ge 2 ]] || { echo "--output requires a value" >&2; exit 2; }
      output="$2"; shift 2 ;;
    --deskew)
      [[ $# -ge 2 ]] || { echo "--deskew requires a value" >&2; exit 2; }
      deskew="$2"; shift 2 ;;
    --gicp-epsilon)
      [[ $# -ge 2 ]] || { echo "--gicp-epsilon requires a value" >&2; exit 2; }
      gicp_epsilon="$2"; shift 2 ;;
    --mid-yaw-policy)
      [[ $# -ge 2 ]] || { echo "--mid-yaw-policy requires a value" >&2; exit 2; }
      mid_yaw_policy="$2"; shift 2 ;;
    --localization-mode)
      [[ $# -ge 2 ]] || { echo "--localization-mode requires a value" >&2; exit 2; }
      localization_mode="$2"; shift 2 ;;
    --snapshot-interval)
      [[ $# -ge 2 ]] || { echo "--snapshot-interval requires a value" >&2; exit 2; }
      snapshot_interval="$2"; shift 2 ;;
    --matcher-override)
      [[ $# -ge 2 ]] || { echo "--matcher-override requires a value" >&2; exit 2; }
      matcher_override="$2"; shift 2 ;;
    --bag)
      [[ $# -ge 2 ]] || { echo "--bag requires a value" >&2; exit 2; }
      bag_override="$2"; shift 2 ;;
    --rate)
      [[ $# -ge 2 ]] || { echo "--rate requires a value" >&2; exit 2; }
      rate="$2"; shift 2 ;;
    --playback-duration)
      [[ $# -ge 2 ]] || { echo "--playback-duration requires a value" >&2; exit 2; }
      playback_duration="$2"; shift 2 ;;
    --no-evaluate) evaluate=false; shift ;;
    --dry-run) dry_run=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$sensor" == "velodyne" || "$sensor" == "mid360" ]] || {
  echo "--sensor must be velodyne or mid360" >&2; exit 2;
}
[[ -n "$output" ]] || { echo "--output is required" >&2; exit 2; }
[[ "$localization_mode" == "baseline" || "$localization_mode" == "precision" ]] || {
  echo "--localization-mode must be baseline or precision" >&2; exit 2;
}
if [[ "$localization_mode" == "baseline" && -n "$matcher_override" ]]; then
  echo "--matcher-override requires --localization-mode precision" >&2
  exit 2
fi
if [[ "$localization_mode" == "baseline" && -n "$snapshot_interval" ]]; then
  echo "--snapshot-interval requires --localization-mode precision" >&2
  exit 2
fi
if [[ "$localization_mode" == "precision" ]]; then
  [[ -n "$snapshot_interval" ]] || snapshot_interval=2
  [[ "$snapshot_interval" == 2 || "$snapshot_interval" == 5 ]] || {
    echo "--snapshot-interval must be 2 or 5" >&2; exit 2;
  }
else
  snapshot_interval=n/a
fi
if ! [[ "$rate" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] ||
  ! awk -v value="$rate" 'BEGIN { exit !(value > 0.0) }'
then
  echo "--rate must be positive" >&2
  exit 2
fi
if [[ -n "$playback_duration" ]] && {
  ! [[ "$playback_duration" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] ||
    ! awk -v value="$playback_duration" 'BEGIN { exit !(value > 0.0) }';
}; then
  echo "--playback-duration must be positive" >&2
  exit 2
fi

odometry_evaluation_root="$ROOT/src/pure_odometry_bringup/config/evaluation/lidar_imu"
velodyne_profile_root="$odometry_evaluation_root/velodyne/rosbag2_2025_02_20-16_06_35_vel"
mid360_profile_root="$odometry_evaluation_root/mid360/rosbag2_2026_04_08-22_52_42"
empty_odom_override="$ROOT/src/pure_odometry_bringup/config/autoware_lsim/empty_params.yaml"
precision_config_root="$ROOT/src/pure_precision_bringup/config"
precision_evaluation_root="$precision_config_root/evaluation/lidar_imu"
case "$sensor" in
  velodyne)
    [[ -z "$gicp_epsilon" ]] || {
      echo "--gicp-epsilon is only supported for MID360" >&2; exit 2;
    }
    [[ -z "$mid_yaw_policy" ]] || {
      echo "--mid-yaw-policy is only supported for MID360" >&2; exit 2;
    }
    [[ -n "$deskew" ]] || deskew=41ms
    case "$deskew" in
      off)
        use_deskew=false
        imu_param="$velodyne_profile_root/accepted/deskew_gap_41ms.yaml"
        odom_override="$velodyne_profile_root/accepted/odom_gap_41ms.yaml"
        ;;
      41ms)
        use_deskew=true
        imu_param="$velodyne_profile_root/accepted/deskew_gap_41ms.yaml"
        odom_override="$velodyne_profile_root/accepted/odom_gap_41ms.yaml"
        ;;
      61ms)
        use_deskew=true
        imu_param="$velodyne_profile_root/experimental/deskew_gap_61ms.yaml"
        odom_override="$velodyne_profile_root/experimental/odom_gap_61ms.yaml"
        ;;
      *) echo "Velodyne --deskew must be off, 41ms, or 61ms" >&2; exit 2 ;;
    esac
    bag="$ROOT/rosbag/rosbag2_2025_02_20-16_06_35_vel"
    glim_dir="/home/motoya/loc_ws/glim_map/vel_dense_20260812_135437"
    points_topic=/velodyne_points
    imu_topic=/imu/data_raw
    default_domain=131
    tf_commands=(
      "0 0 0 0 0 0 base_link velodyne"
      "0 0 -0.1874 3.141592653589793 0 0 base_link imu"
    )
    ;;
  mid360)
    [[ -n "$deskew" ]] || deskew=on
    case "$deskew" in
      off) use_deskew=false ;;
      on) use_deskew=true ;;
      *) echo "MID360 --deskew must be off or on" >&2; exit 2 ;;
    esac
    if [[ -z "$mid_yaw_policy" ]]; then
      if [[ -n "$bag_override" ]]; then
        echo "MID360 --bag requires an explicit --mid-yaw-policy; the fixed bias is bag-specific" >&2
        exit 2
      fi
      mid_yaw_policy=fixed-bias-direct
    fi
    case "$mid_yaw_policy" in
      adaptive)
        [[ -n "$gicp_epsilon" ]] || gicp_epsilon=strict
        case "$gicp_epsilon" in
          strict)
            odom_override="$mid360_profile_root/experimental/odom_adaptive_strict.yaml"
            ;;
          default)
            odom_override="$mid360_profile_root/experimental/odom_adaptive_default_epsilon.yaml"
            ;;
          *) echo "MID360 --gicp-epsilon must be strict or default" >&2; exit 2 ;;
        esac
        ;;
      fixed-bias-direct)
        [[ -n "$gicp_epsilon" ]] || gicp_epsilon=default
        [[ "$gicp_epsilon" == default ]] || {
          echo "MID360 fixed-bias-direct requires --gicp-epsilon default" >&2
          exit 2
        }
        odom_override="$mid360_profile_root/accepted/odom_fixed_bias_direct.yaml"
        ;;
      *)
        echo "MID360 --mid-yaw-policy must be adaptive or fixed-bias-direct" >&2
        exit 2
        ;;
    esac
    imu_param="$mid360_profile_root/accepted/deskew.yaml"
    bag="$ROOT/rosbag/rosbag2_2026_04_08-22_52_42"
    glim_dir="/home/motoya/loc_ws/glim_map/mid360_dense_20260812_142237"
    points_topic=/livox/lidar
    imu_topic=/livox/imu
    default_domain=132
    tf_commands=("0 0 0 0 0 0 base_link livox_frame")
    ;;
esac

if [[ -n "$bag_override" ]]; then
  bag="$(realpath -m "$bag_override")"
fi

odom_aux_override="$empty_odom_override"
snapshot_override="n/a"
matcher_param="n/a"
matcher_override_param="n/a"
global_param="n/a"
global_override_param="n/a"
evaluation_topic=/localization/gyro_lidar_odom_scan
if [[ "$localization_mode" == "precision" ]]; then
  case "$snapshot_interval" in
    2) snapshot_override="$precision_evaluation_root/external_snapshot_i2.yaml" ;;
    5) snapshot_override="$precision_config_root/submap_snapshot_override.yaml" ;;
  esac
  odom_aux_override="$snapshot_override"
  matcher_param="$ROOT/src/pure_lidar_submap_matcher/param/param.yaml"
  matcher_override_param="${matcher_override:-$precision_config_root/empty_params.yaml}"
  global_param="$ROOT/src/pure_precision_global_localizer/param/param.yaml"
  global_override_param="$precision_evaluation_root/accepted_scan_local_only.yaml"
  evaluation_topic=/localization/precision_local_odom
fi

[[ -d "$bag" ]] || { echo "bag does not exist: $bag" >&2; exit 2; }
[[ -f "$glim_dir/traj_lidar.txt" ]] || {
  echo "GLIM trajectory does not exist: $glim_dir/traj_lidar.txt" >&2; exit 2;
}
required_configs=("$imu_param" "$odom_override" "$odom_aux_override")
if [[ "$localization_mode" == "precision" ]]; then
  required_configs+=(
    "$matcher_param" "$matcher_override_param" "$global_param" "$global_override_param"
  )
fi
for path in "${required_configs[@]}"; do
  [[ -f "$path" ]] || { echo "configuration does not exist: $path" >&2; exit 2; }
done
output="$(realpath -m "$output")"
[[ ! -e "$output" ]] || { echo "refusing to overwrite: $output" >&2; exit 2; }

set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$default_domain}"

launch_command=(
  ros2 launch pure_odometry_bringup lidar_imu_only.launch.py
  use_sim_time:=true
  use_imu_deskew:="$use_deskew"
  points_input_topic:=/points_raw
  imu_input_topic:=/imu
  imu_param:="$imu_param"
  odom_param:="$ROOT/src/pure_lidar_gyro_odometer/param/param.yaml"
  odom_override_param:="$odom_override"
  odom_aux_override_param:="$odom_aux_override"
  log_level:=info
)
precision_launch_command=()
if [[ "$localization_mode" == "precision" ]]; then
  precision_launch_command=(
    ros2 launch pure_precision_bringup precision_overlay.launch.py
    use_sim_time:=true
    matcher_param:="$matcher_param"
    matcher_override_param:="$matcher_override_param"
    global_param:="$global_param"
    global_override_param:="$global_override_param"
    log_level:=info
  )
fi
record_directory="$output/localization_output"
record_command=(
  ros2 bag record --storage mcap --output "$record_directory"
  --node-name lidar_imu_glim_output_recorder --topics
  /clock /diagnostics /localization/gyro_lidar_odom
  /localization/gyro_lidar_odom_scan
  /localization/imu_corrected /localization/is_stopped
)
if [[ "$localization_mode" == "precision" ]]; then
  record_command+=(
    /localization/submap_scan /localization/submap_correction
    /localization/precision_local_odom
    /localization/precision_global_odom /localization/precision_global_pose
  )
fi
play_command=(
  bash "$ROOT/script/play_localization_bag.sh"
  --bag "$bag" --points "$points_topic" --imu "$imu_topic"
  --rate "$rate" --clock-frequency 100 --tf-policy isolate-all --
  --disable-keyboard-controls --delay 2 --topics "$points_topic" "$imu_topic"
)
if [[ -n "$playback_duration" ]]; then
  play_command+=(--playback-duration "$playback_duration")
fi

print_command() {
  printf ' %q' "$@"
  printf '\n'
}
if [[ "$dry_run" == true ]]; then
  for tf in "${tf_commands[@]}"; do
    read -r x y z roll pitch yaw parent child <<< "$tf"
    print_command ros2 run tf2_ros static_transform_publisher \
      --x "$x" --y "$y" --z "$z" --roll "$roll" --pitch "$pitch" --yaw "$yaw" \
      --frame-id "$parent" --child-frame-id "$child"
  done
  print_command "${launch_command[@]}"
  if [[ "$localization_mode" == "precision" ]]; then
    print_command "${precision_launch_command[@]}"
  fi
  print_command "${record_command[@]}"
  print_command "${play_command[@]}"
  exit 0
fi

mkdir -p "$output/ros_logs"
export ROS_LOG_DIR="$output/ros_logs"
mkdir -p "$output/artifacts"
base_odom_param="$ROOT/src/pure_lidar_gyro_odometer/param/param.yaml"
install -m 0644 "$imu_param" "$output/artifacts/imu_param.yaml"
install -m 0644 "$odom_override" "$output/artifacts/odom_override.yaml"
install -m 0644 "$base_odom_param" "$output/artifacts/base_odom_param.yaml"
install -m 0644 "$odom_aux_override" "$output/artifacts/odom_aux_override.yaml"
read -r imu_param_sha256 _ < <(sha256sum "$imu_param")
read -r odom_override_sha256 _ < <(sha256sum "$odom_override")
read -r base_odom_param_sha256 _ < <(sha256sum "$base_odom_param")
read -r odom_aux_override_sha256 _ < <(sha256sum "$odom_aux_override")
read -r glim_traj_sha256 _ < <(sha256sum "$glim_dir/traj_lidar.txt")
snapshot_override_sha256=n/a
matcher_param_sha256=n/a
matcher_override_param_sha256=n/a
global_param_sha256=n/a
global_override_param_sha256=n/a
if [[ "$localization_mode" == "precision" ]]; then
  install -m 0644 "$snapshot_override" "$output/artifacts/submap_snapshot_override.yaml"
  install -m 0644 "$matcher_param" "$output/artifacts/submap_matcher_param.yaml"
  install -m 0644 "$matcher_override_param" \
    "$output/artifacts/submap_matcher_override.yaml"
  install -m 0644 "$global_param" "$output/artifacts/precision_global_param.yaml"
  install -m 0644 "$global_override_param" \
    "$output/artifacts/precision_local_only_override.yaml"
  read -r snapshot_override_sha256 _ < <(sha256sum "$snapshot_override")
  read -r matcher_param_sha256 _ < <(sha256sum "$matcher_param")
  read -r matcher_override_param_sha256 _ < <(sha256sum "$matcher_override_param")
  read -r global_param_sha256 _ < <(sha256sum "$global_param")
  read -r global_override_param_sha256 _ < <(sha256sum "$global_override_param")
fi
cat > "$output/run.env" <<EOF
sensor=$sensor
localization_mode=$localization_mode
snapshot_interval=${snapshot_interval:-n/a}
evaluation_topic=$evaluation_topic
bag=$bag
glim_dir=$glim_dir
glim_traj=$glim_dir/traj_lidar.txt
glim_traj_sha256=$glim_traj_sha256
deskew=$deskew
gicp_epsilon=$gicp_epsilon
mid_yaw_policy=${mid_yaw_policy:-n/a}
use_deskew=$use_deskew
rate=$rate
playback_duration=$playback_duration
points_topic=$points_topic
imu_topic=$imu_topic
imu_param=$imu_param
imu_param_sha256=$imu_param_sha256
odom_override=$odom_override
odom_override_sha256=$odom_override_sha256
base_odom_param=$base_odom_param
base_odom_param_sha256=$base_odom_param_sha256
odom_aux_override=$odom_aux_override
odom_aux_override_sha256=$odom_aux_override_sha256
snapshot_override=$snapshot_override
snapshot_override_sha256=$snapshot_override_sha256
matcher_param=$matcher_param
matcher_param_sha256=$matcher_param_sha256
matcher_override_param=$matcher_override_param
matcher_override_param_sha256=$matcher_override_param_sha256
global_param=$global_param
global_param_sha256=$global_param_sha256
global_override_param=$global_override_param
global_override_param_sha256=$global_override_param_sha256
ros_domain_id=$ROS_DOMAIN_ID
recorded_tf_used=false
gnss_started=false
map_odom_fusion_started=false
EOF

pids=()
launch_pid=""
precision_pid=""
record_pid=""
play_pid=""
tf_pids=()
started_pid=""

process_alive() {
  local pid="$1"
  [[ "$pid" =~ ^[1-9][0-9]*$ ]] && kill -0 "$pid" 2>/dev/null
}

process_group_alive() {
  local pid="$1"
  [[ "$pid" =~ ^[1-9][0-9]*$ ]] && kill -0 -- "-$pid" 2>/dev/null
}

start_process_group() {
  local log_path="$1"
  shift
  setsid "$@" > "$log_path" 2>&1 &
  started_pid=$!
  pids+=("$started_pid")
}

stop_process_group() {
  local pid="$1"
  [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 0
  if process_group_alive "$pid"; then
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in {1..100}; do
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
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  stop_process_group "$play_pid"
  stop_process_group "$launch_pid"
  stop_process_group "$precision_pid"
  local pid
  for pid in "${tf_pids[@]}"; do stop_process_group "$pid"; done
  stop_process_group "$record_pid"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_for_topic() {
  local topic="$1"
  local owner_pid="$2"
  local deadline=$((SECONDS + 60))
  while ((SECONDS < deadline)); do
    process_alive "$owner_pid" || return 2
    if ros2 topic list --no-daemon --spin-time 1 2>/dev/null | grep -Fx "$topic" >/dev/null; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

for tf in "${tf_commands[@]}"; do
  read -r x y z roll pitch yaw parent child <<< "$tf"
  start_process_group "$output/tf_${child}.log" \
    ros2 run tf2_ros static_transform_publisher \
    --x "$x" --y "$y" --z "$z" --roll "$roll" --pitch "$pitch" --yaw "$yaw" \
    --frame-id "$parent" --child-frame-id "$child"
  tf_pids+=("$started_pid")
done

start_process_group "$output/launch.log" "${launch_command[@]}"
launch_pid=$started_pid
if ! wait_for_topic /localization/gyro_lidar_odom "$launch_pid"; then
  echo "odometer did not become ready; see $output/launch.log" >&2
  exit 1
fi
if ! wait_for_topic /localization/gyro_lidar_odom_scan "$launch_pid"; then
  echo "accepted-scan odometry did not become ready; see $output/launch.log" >&2
  exit 1
fi
if ! wait_for_topic /localization/imu_corrected "$launch_pid"; then
  echo "corrected IMU publisher did not become ready; see $output/launch.log" >&2
  exit 1
fi
if [[ "$use_deskew" == true ]] &&
  ! wait_for_topic /localization/points_undistorted "$launch_pid"
then
  echo "deskew publisher did not become ready; see $output/launch.log" >&2
  exit 1
fi

if [[ "$localization_mode" == "precision" ]]; then
  if ! wait_for_topic /localization/submap_scan "$launch_pid"; then
    echo "submap snapshot publisher did not become ready; see $output/launch.log" >&2
    exit 1
  fi
  start_process_group "$output/precision_overlay.log" "${precision_launch_command[@]}"
  precision_pid=$started_pid
  if ! wait_for_topic /localization/submap_correction "$precision_pid"; then
    echo "submap matcher did not become ready; see $output/precision_overlay.log" >&2
    exit 1
  fi
  if ! wait_for_topic /localization/precision_local_odom "$precision_pid"; then
    echo "precision-local compositor did not become ready; see $output/precision_overlay.log" >&2
    exit 1
  fi
fi

start_process_group "$output/record.log" "${record_command[@]}"
record_pid=$started_pid
sleep 2
process_alive "$record_pid" || { echo "recorder exited; see $output/record.log" >&2; exit 1; }

play_wall_start="$(date +%s.%N)"
start_process_group "$output/play.log" "${play_command[@]}"
play_pid=$started_pid
set +e
wait "$play_pid"
play_status=$?
set -e
play_pid=""
play_wall_end="$(date +%s.%N)"
printf 'play_wall_start_sec=%s\nplay_wall_end_sec=%s\n' \
  "$play_wall_start" "$play_wall_end" >> "$output/run.env"
if ((play_status != 0)); then
  echo "bag playback failed with status $play_status; see $output/play.log" >&2
  exit 1
fi

# Let the final LiDAR callback and a later wall-timer diagnostic drain while
# the recorder remains alive.  /clock is frozen here, so regular timer topics
# repeat its final stamp; accepted-scan odometry remains one message per scan.
sleep 2
stop_process_group "$launch_pid"
launch_pid=""
if [[ "$localization_mode" == "precision" ]]; then
  # No new snapshots can arrive after the odometer exits.  Keep the recorder
  # and precision branch alive for one final matcher/compositor diagnostic.
  sleep 1
  stop_process_group "$precision_pid"
  precision_pid=""
fi
for pid in "${tf_pids[@]}"; do stop_process_group "$pid"; done
tf_pids=()
sleep 0.2
stop_process_group "$record_pid"
record_pid=""

ros2 bag info "$bag" > "$output/input_bag_info.txt"
ros2 bag info "$record_directory" > "$output/output_bag_info.txt"
runtime_status=0
submap_validation_status=0
submap_validation_executed=false
evaluation_status=0
set +e
python3 "$ROOT/tools/analyze_lidar_imu_run.py" \
  --sensor "$sensor" --input-bag "$bag" --result-bag "$record_directory" \
  --run-env "$output/run.env" --output-dir "$output/runtime_analysis"
runtime_status=$?

if [[ "$localization_mode" == "precision" ]]; then
  submap_validation_executed=true
  python3 "$ROOT/tools/validate_lidar_imu_submap_run.py" \
    --run-dir "$output" --output-dir "$output/submap_validation"
  submap_validation_status=$?
fi

if [[ "$evaluate" == true ]]; then
  python3 "$ROOT/tools/evaluate_glim_trajectory.py" \
    --local-only --result-bag "$record_directory" --glim-dir "$glim_dir" \
    --local-topic "$evaluation_topic" \
    --sample-at-estimate-stamps \
    --maximum-interpolation-gap-sec 0.15 \
    --output-dir "$output/glim_evaluation" --label "$sensor LiDAR/IMU-only ($deskew)"
  evaluation_status=$?
fi
set -e
printf 'runtime_analysis_status=%s\nsubmap_validation_executed=%s\nsubmap_validation_status=%s\nevaluation_status=%s\n' \
  "$runtime_status" "$submap_validation_executed" \
  "$submap_validation_status" "$evaluation_status" >> "$output/run.env"

echo "$output"
if ((runtime_status != 0 || submap_validation_status != 0 || evaluation_status != 0)); then
  echo "run completed with runtime status=$runtime_status, submap status=$submap_validation_status, evaluation status=$evaluation_status" >&2
  exit 1
fi
