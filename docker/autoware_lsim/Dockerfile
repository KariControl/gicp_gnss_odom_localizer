# syntax=docker/dockerfile:1.7
# SPDX-License-Identifier: Apache-2.0

ARG AUTOWARE_IMAGE=ghcr.io/autowarefoundation/autoware:universe-devel-jazzy-1.9.0
FROM ${AUTOWARE_IMAGE}

ARG ROS_DISTRO=jazzy
ARG COLCON_PARALLEL_WORKERS=2

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
USER root

# Keep the estimator in a separate overlay. Autoware remains in /opt/autoware,
# so replacing the localizer never mutates the upstream installation.
COPY --chown=aw:aw src /tmp/gicp_gnss_odom_ws/src

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-numpy \
      "ros-${ROS_DISTRO}-rosbag2-py" \
      "ros-${ROS_DISTRO}-rosidl-runtime-py" \
      "ros-${ROS_DISTRO}-rosbag2-storage-mcap" && \
    source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    source /opt/autoware/setup.bash && \
    rosdep install \
      --from-paths /tmp/gicp_gnss_odom_ws/src \
      --ignore-src \
      --rosdistro "${ROS_DISTRO}" \
      -t build -t build_export -t buildtool -t buildtool_export -t exec \
      -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    source /opt/autoware/setup.bash && \
    MAKEFLAGS="-j${COLCON_PARALLEL_WORKERS}" \
    colcon --log-base /tmp/gicp_gnss_odom_ws/log build \
      --merge-install \
      --base-paths /tmp/gicp_gnss_odom_ws/src \
      --build-base /tmp/gicp_gnss_odom_ws/build \
      --install-base /opt/gicp_gnss_odom_localizer \
      --parallel-workers "${COLCON_PARALLEL_WORKERS}" \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF && \
    rm -rf /tmp/gicp_gnss_odom_ws

COPY --chown=aw:aw script/play_localization_bag.sh \
  /opt/gicp_gnss_odom_localizer/bin/play_localization_bag.sh
COPY --chown=aw:aw tools/analyze_autoware_lsim_output.py \
  /opt/gicp_gnss_odom_localizer/bin/analyze_autoware_lsim_output.py
COPY --chown=aw:aw tools/evaluation/precision/scripts/validate_precision_bag.py \
  /opt/gicp_gnss_odom_localizer/bin/validate_precision_bag.py
COPY --chown=aw:aw docker/autoware_lsim/container_entrypoint.sh \
  /opt/gicp_gnss_odom_localizer/bin/run_autoware_lsim.sh
COPY --chown=aw:aw docker/autoware_lsim/autoware_localization_contract_test.py \
  /opt/gicp_gnss_odom_localizer/bin/autoware_localization_contract_test.py
COPY --chown=aw:aw docker/autoware_lsim/run_localization_contract_test.sh \
  /opt/gicp_gnss_odom_localizer/bin/run_localization_contract_test.sh

RUN chmod 0755 \
      /opt/gicp_gnss_odom_localizer/bin/play_localization_bag.sh \
      /opt/gicp_gnss_odom_localizer/bin/analyze_autoware_lsim_output.py \
      /opt/gicp_gnss_odom_localizer/bin/validate_precision_bag.py \
      /opt/gicp_gnss_odom_localizer/bin/autoware_localization_contract_test.py \
      /opt/gicp_gnss_odom_localizer/bin/run_autoware_lsim.sh \
      /opt/gicp_gnss_odom_localizer/bin/run_localization_contract_test.sh && \
    mkdir -p /bags /output && \
    chown -R aw:aw /bags /output /opt/gicp_gnss_odom_localizer

ENV AUTOWARE_RUNTIME=1
ENV GICP_GNSS_ODOM_INSTALL=/opt/gicp_gnss_odom_localizer
WORKDIR /home/aw

CMD ["/opt/gicp_gnss_odom_localizer/bin/run_autoware_lsim.sh"]
