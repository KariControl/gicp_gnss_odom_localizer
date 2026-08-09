#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

usage() {
  cat <<'USAGE'
Build and run localization-only Autoware Logging Simulation in Docker.

Usage:
  run_autoware_lsim_docker.sh --bag <bag_path> [options]

Required:
  --bag <path>                  Rosbag2 directory or single MCAP/DB3 file

Sensor topic options:
  --points <topic>              PointCloud2 source topic
                                default: /sensing/lidar/top/pointcloud_raw
  --imu <topic>                 IMU source topic
                                default: /sensing/imu/tamagawa/imu_raw
  --nmea <topic>                Primary NMEA sentence source and enable GNSS
  --nmea-secondary <topic>      Secondary NMEA sentence source
  --fix-velocity <topic>        Doppler/fix-velocity source topic
  --twist <topic>               Twist source used by optional translational deskew

Estimator options:
  --tracking-mode <mode>        scan_to_scan | scan_to_submap
  --already-deskewed            Bypass internal IMU point-cloud deskew
  --rate <factor>               Rosbag playback rate (default: 1.0)
  --tf-policy <policy>          keep | isolate-dynamic | isolate-all
                                default: isolate-dynamic
  --keep-recorded-localization  Do not move recorded localization outputs to /reference
  --launch-vehicle              Publish static TF from Autoware vehicle/sensor models
  --vehicle-model <name>        Default: sample_vehicle
  --sensor-model <name>         Default: sample_sensor_kit
  --initial-pose <x> <y> <yaw>  Automatic map anchor used only without GNSS
  --no-auto-initial-pose        Require manual /initialpose

Docker/output options:
  --output <directory>          Host result root (default: ./docker_output)
  --run-name <name>             Result subdirectory name
  --rviz                        Enable RViz with CPU software rendering
  --no-record                   Do not record localization output topics
  --autoware-image <image>      Base devel image
                                default: universe-devel-jazzy-1.9.0
  --image <image>               Resulting local Docker image name
  --build-jobs <count>          Build parallelism (default: 2)
  --no-build                    Reuse an already built local image
  --pull-base                   Pull the pinned Autoware base before building
  --build-only                  Build image and exit
  --shell                       Open a shell in the image instead of running LSim
  --dry-run                     Validate and print Docker commands only
  -h, --help                    Show this help

The default path is headless, CPU-only, map-free, and records both test and
/reference localization outputs. Host ROS 2 and Autoware installations are not
required for this Docker stage.
USAGE
}

fail() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 2
}

normalize_path() {
  python3 - "$1" <<'PY'
from pathlib import Path
import sys
print(Path(sys.argv[1]).expanduser().resolve())
PY
}

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_BASE="$ROOT/docker/autoware_lsim/compose.yaml"
COMPOSE_RVIZ="$ROOT/docker/autoware_lsim/compose.rviz.yaml"

bag=""
points_topic="/sensing/lidar/top/pointcloud_raw"
imu_topic="/sensing/imu/tamagawa/imu_raw"
nmea_topic=""
nmea_secondary_topic=""
fix_velocity_topic=""
twist_topic=""
tracking_mode="scan_to_scan"
use_imu_deskew="true"
playback_rate="1.0"
tf_policy="isolate-dynamic"
keep_recorded_localization="false"
launch_vehicle="false"
vehicle_model="sample_vehicle"
sensor_model="sample_sensor_kit"
auto_initial_pose="true"
initial_x="0.0"
initial_y="0.0"
initial_yaw="0.0"
output_directory="$ROOT/docker_output"
run_name=""
rviz="false"
record_output="true"
autoware_image="ghcr.io/autowarefoundation/autoware:universe-devel-jazzy-1.9.0"
localizer_image="gicp-gnss-odom-localizer:autoware-1.9.0"
build_jobs="2"
build_image="true"
pull_base="false"
build_only="false"
open_shell="false"
dry_run="false"

while (($# > 0)); do
  case "$1" in
    --bag) [[ $# -ge 2 ]] || fail "--bag requires a value"; bag="$2"; shift 2 ;;
    --points) [[ $# -ge 2 ]] || fail "--points requires a value"; points_topic="$2"; shift 2 ;;
    --imu) [[ $# -ge 2 ]] || fail "--imu requires a value"; imu_topic="$2"; shift 2 ;;
    --nmea) [[ $# -ge 2 ]] || fail "--nmea requires a value"; nmea_topic="$2"; shift 2 ;;
    --nmea-secondary) [[ $# -ge 2 ]] || fail "--nmea-secondary requires a value"; nmea_secondary_topic="$2"; shift 2 ;;
    --fix-velocity) [[ $# -ge 2 ]] || fail "--fix-velocity requires a value"; fix_velocity_topic="$2"; shift 2 ;;
    --twist) [[ $# -ge 2 ]] || fail "--twist requires a value"; twist_topic="$2"; shift 2 ;;
    --tracking-mode) [[ $# -ge 2 ]] || fail "--tracking-mode requires a value"; tracking_mode="$2"; shift 2 ;;
    --already-deskewed) use_imu_deskew="false"; shift ;;
    --rate) [[ $# -ge 2 ]] || fail "--rate requires a value"; playback_rate="$2"; shift 2 ;;
    --tf-policy) [[ $# -ge 2 ]] || fail "--tf-policy requires a value"; tf_policy="$2"; shift 2 ;;
    --keep-recorded-localization) keep_recorded_localization="true"; shift ;;
    --launch-vehicle) launch_vehicle="true"; shift ;;
    --vehicle-model) [[ $# -ge 2 ]] || fail "--vehicle-model requires a value"; vehicle_model="$2"; shift 2 ;;
    --sensor-model) [[ $# -ge 2 ]] || fail "--sensor-model requires a value"; sensor_model="$2"; shift 2 ;;
    --initial-pose)
      [[ $# -ge 4 ]] || fail "--initial-pose requires x y yaw"
      initial_x="$2"; initial_y="$3"; initial_yaw="$4"; shift 4 ;;
    --no-auto-initial-pose) auto_initial_pose="false"; shift ;;
    --output) [[ $# -ge 2 ]] || fail "--output requires a value"; output_directory="$2"; shift 2 ;;
    --run-name) [[ $# -ge 2 ]] || fail "--run-name requires a value"; run_name="$2"; shift 2 ;;
    --rviz) rviz="true"; shift ;;
    --no-record) record_output="false"; shift ;;
    --autoware-image) [[ $# -ge 2 ]] || fail "--autoware-image requires a value"; autoware_image="$2"; shift 2 ;;
    --image) [[ $# -ge 2 ]] || fail "--image requires a value"; localizer_image="$2"; shift 2 ;;
    --build-jobs) [[ $# -ge 2 ]] || fail "--build-jobs requires a value"; build_jobs="$2"; shift 2 ;;
    --no-build) build_image="false"; shift ;;
    --pull-base) pull_base="true"; shift ;;
    --build-only) build_only="true"; shift ;;
    --shell) open_shell="true"; shift ;;
    --dry-run) dry_run="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "unknown option: $1" ;;
  esac
done

if [[ -z "$bag" && "$build_only" == true ]]; then
  bag="$ROOT"
fi
[[ -n "$bag" ]] || fail "--bag is required"
[[ -e "$bag" ]] || fail "bag does not exist: $bag"
case "$tracking_mode" in scan_to_scan|scan_to_submap) ;; *) fail "invalid tracking mode: $tracking_mode" ;; esac
case "$tf_policy" in keep|isolate-dynamic|isolate-all) ;; *) fail "invalid TF policy: $tf_policy" ;; esac
[[ "$build_jobs" =~ ^[1-9][0-9]*$ ]] || fail "--build-jobs must be a positive integer"
if ! [[ "$playback_rate" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || \
  ! awk -v value="$playback_rate" 'BEGIN { exit !(value > 0.0) }'; then
  fail "--rate must be a positive number"
fi
if [[ "$tf_policy" == isolate-all && "$launch_vehicle" != true ]]; then
  fail "--tf-policy isolate-all requires --launch-vehicle"
fi
if [[ "$rviz" == true && -z "${DISPLAY:-}" ]]; then
  fail "--rviz requires DISPLAY"
fi

HOST_BAG_PATH="$(normalize_path "$bag")"
mkdir -p "$output_directory"
HOST_OUTPUT_DIR="$(normalize_path "$output_directory")"
if [[ -z "$run_name" ]]; then
  bag_name="$(basename "$HOST_BAG_PATH")"
  run_name="${bag_name%.*}_${tracking_mode}_$(date +%Y%m%d_%H%M%S)"
fi

export HOST_BAG_PATH HOST_OUTPUT_DIR
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
export POINTS_SOURCE_TOPIC="$points_topic"
export IMU_SOURCE_TOPIC="$imu_topic"
export NMEA_SOURCE_TOPIC="$nmea_topic"
export NMEA_SECONDARY_SOURCE_TOPIC="$nmea_secondary_topic"
export FIX_VELOCITY_SOURCE_TOPIC="$fix_velocity_topic"
export TWIST_SOURCE_TOPIC="$twist_topic"
export PLAYBACK_RATE="$playback_rate"
export TF_POLICY="$tf_policy"
export KEEP_RECORDED_LOCALIZATION="$keep_recorded_localization"
export TRACKING_MODE="$tracking_mode"
export USE_GNSS="$(if [[ -n "$nmea_topic" ]]; then printf true; else printf false; fi)"
export USE_IMU_DESKEW="$use_imu_deskew"
export LAUNCH_VEHICLE="$launch_vehicle"
export LAUNCH_SENSING="false"
export VEHICLE_MODEL="$vehicle_model"
export SENSOR_MODEL="$sensor_model"
export RVIZ="$rviz"
export RECORD_OUTPUT="$record_output"
export AUTO_INITIAL_POSE="$auto_initial_pose"
export INITIAL_X="$initial_x" INITIAL_Y="$initial_y" INITIAL_Z="0.0" INITIAL_YAW="$initial_yaw"
export RUN_NAME="$run_name"
export AUTOWARE_IMAGE="$autoware_image"
export LOCALIZER_LSIM_IMAGE="$localizer_image"
export COLCON_PARALLEL_WORKERS="$build_jobs"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

compose=(docker compose -f "$COMPOSE_BASE")
if [[ "$rviz" == true ]]; then
  compose+=(-f "$COMPOSE_RVIZ")
fi

printf 'Autoware base image: %s\n' "$AUTOWARE_IMAGE"
printf 'Local image:         %s\n' "$LOCALIZER_LSIM_IMAGE"
printf 'Bag:                 %s\n' "$HOST_BAG_PATH"
printf 'Output root:         %s\n' "$HOST_OUTPUT_DIR"
printf 'Tracking mode:       %s\n' "$TRACKING_MODE"
printf 'GNSS:                %s\n' "$USE_GNSS"
printf 'RViz:                %s\n' "$RVIZ"

if [[ "$dry_run" == true ]]; then
  if [[ "$pull_base" == true ]]; then
    printf 'Pull command: docker pull %q\n' "$AUTOWARE_IMAGE"
  fi
  if [[ "$build_image" == true ]]; then
    printf 'Build command:'
    printf ' %q' "${compose[@]}" build lsim
    printf '\n'
  fi
  if [[ "$build_only" != true ]]; then
    printf 'Run command:'
    if [[ "$open_shell" == true ]]; then
      printf ' %q' "${compose[@]}" run --rm lsim bash
    else
      printf ' %q' "${compose[@]}" run --rm lsim
    fi
    printf '\n'
  fi
  exit 0
fi

command -v docker >/dev/null 2>&1 || fail "docker is not installed"
docker compose version >/dev/null 2>&1 || fail "Docker Compose v2 plugin is not available"
docker info >/dev/null 2>&1 || fail "Docker daemon is not accessible by the current user"

if [[ "$rviz" == true ]]; then
  command -v xhost >/dev/null 2>&1 || fail "xhost is required for --rviz"
  xhost +local:docker >/dev/null
  trap 'xhost -local:docker >/dev/null 2>&1 || true' EXIT
fi

if [[ "$pull_base" == true ]]; then
  docker pull "$AUTOWARE_IMAGE"
fi
if [[ "$build_image" == true ]]; then
  "${compose[@]}" build lsim
fi
if [[ "$build_only" == true ]]; then
  exit 0
fi
if [[ "$open_shell" == true ]]; then
  "${compose[@]}" run --rm lsim bash
else
  "${compose[@]}" run --rm lsim
fi
