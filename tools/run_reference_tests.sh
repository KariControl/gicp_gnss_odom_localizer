#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${TMPDIR:-/tmp}/gicp_gnss_odom_localizer_reference_tests"
rm -rf "$build_dir"
mkdir -p "$build_dir"

cxx="${CXX:-g++}"
flags=(-std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror)

build_and_run() {
  local output="$1"
  local include_dir="$2"
  local source="$3"
  "$cxx" "${flags[@]}" -I"$root/$include_dir" "$root/$source" -o "$build_dir/$output"
  "$build_dir/$output"
}

build_and_run test_nmea_imu \
  src/pure_nmea_gnss_conversion/include \
  src/pure_nmea_gnss_conversion/test/test_imu_yaw_integrator.cpp
build_and_run test_nmea_trajectory_heading_quality \
  src/pure_nmea_gnss_conversion/include \
  src/pure_nmea_gnss_conversion/test/test_trajectory_heading_quality.cpp
build_and_run test_gnss_recovery \
  src/pure_gnss_map_odom_fusion/include \
  src/pure_gnss_map_odom_fusion/test/test_gnss_recovery_controller.cpp
build_and_run test_lidar_imu \
  src/pure_lidar_gyro_odometer/include \
  src/pure_lidar_gyro_odometer/test/test_yaw_rate_integrator.cpp
build_and_run test_se2_smoother \
  src/pure_lidar_gyro_odometer/include \
  src/pure_lidar_gyro_odometer/test/test_se2_fixed_lag_smoother.cpp
build_and_run test_tracking_mode \
  src/pure_lidar_gyro_odometer/include \
  src/pure_lidar_gyro_odometer/test/test_tracking_mode.cpp
build_and_run test_observability_policy \
  src/pure_lidar_gyro_odometer/include \
  src/pure_lidar_gyro_odometer/test/test_observability_policy.cpp
build_and_run test_acceleration_estimator \
  src/pure_localization_interface_adapter/include \
  src/pure_localization_interface_adapter/test/test_acceleration_estimator.cpp

echo "Reference tests PASS"
