#!/usr/bin/env bash
# set -euo pipefail

source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
