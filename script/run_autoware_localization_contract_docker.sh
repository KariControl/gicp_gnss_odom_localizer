#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="gicp-gnss-odom-localizer:autoware-1.9.0"
autoware_image="ghcr.io/autowarefoundation/autoware:universe-devel-jazzy-1.9.0"
build_jobs=2
build=true
pull=false
output_dir=""

while (($# > 0)); do
  case "$1" in
    --image) image="$2"; shift 2 ;;
    --autoware-image) autoware_image="$2"; shift 2 ;;
    --build-jobs) build_jobs="$2"; shift 2 ;;
    --output-dir) output_dir="$2"; shift 2 ;;
    --no-build) build=false; shift ;;
    --pull-base) pull=true; shift ;;
    -h|--help)
      printf '%s\n' \
        'Usage: run_autoware_localization_contract_docker.sh [options]' \
        '  --no-build                 reuse the existing image' \
        '  --pull-base                refresh the pinned base image' \
        '  --image <name>             local result image' \
        '  --autoware-image <name>    Autoware base image' \
        '  --build-jobs <count>       colcon parallel workers' \
        '  --output-dir <path>        retained JSON and logs'
      exit 0
      ;;
    *) printf 'ERROR: unknown option: %s\n' "$1" >&2; exit 2 ;;
  esac
done

[[ "$build_jobs" =~ ^[1-9][0-9]*$ ]] || {
  printf 'ERROR: --build-jobs must be a positive integer\n' >&2
  exit 2
}

if [[ -z "$output_dir" ]]; then
  output_dir="$ROOT/docker_output/autoware_localization_contract_$(date -u +%Y%m%dT%H%M%SZ)"
fi
mkdir -p "$output_dir"
output_dir="$(cd "$output_dir" && pwd)"

if [[ "$build" == true ]]; then
  build_command=(
    docker build
    --file "$ROOT/docker/autoware_lsim/Dockerfile"
    --build-arg "AUTOWARE_IMAGE=$autoware_image"
    --build-arg "ROS_DISTRO=jazzy"
    --build-arg "COLCON_PARALLEL_WORKERS=$build_jobs"
    --tag "$image"
  )
  [[ "$pull" == true ]] && build_command+=(--pull)
  build_command+=("$ROOT")
  "${build_command[@]}"
fi

if ! localizer_image_id="$(docker image inspect "$image" --format '{{.Id}}')"; then
  printf 'ERROR: contract image is unavailable: %s\n' "$image" >&2
  exit 1
fi

set +e
docker run --rm \
  --network host \
  --ipc host \
  --privileged \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-93}" \
  --env "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
  --env "AUTOWARE_IMAGE=$autoware_image" \
  --env "LOCALIZER_IMAGE_ID=$localizer_image_id" \
  --env "HOST_UID=$(id -u)" \
  --env "HOST_GID=$(id -g)" \
  --env "CONTRACT_OUTPUT_DIR=/output" \
  --volume "$output_dir:/output" \
  "$image" \
  /opt/gicp_gnss_odom_localizer/bin/run_localization_contract_test.sh
contract_status=$?
set -e

printf 'Contract artifacts: %s\n' "$output_dir"
exit "$contract_status"
