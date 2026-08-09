#!/bin/bash
# デュアルアンテナ テスト: 横向き構成
#   Mainアンテナ: 進行方向に対して右側
#   ロスバッグ: rosbag/rosbag2_2026_05_19-10_18_18_pointcloud
#
# センサーキット内部キャリブレーション (センサーキット原点=LiDAR):
#   pandar_ot128_top: (0, 0, 0, yaw=1.57096)
#   imu:              (0, 0.002, -0.3)
#   gnss_main:        (0, -0.48, -0.3)
#   gnss_sub:         (0,  0.48, -0.3)
#   ベースライン: 0.96m
#
# 実測取付値:
#   Lidar_Y = 0.76m  (base_linkからLiDAR原点のY)
#   IMU_z   = 1.62m  (base_linkからIMUのZ) → LiDAR_Z = 1.62 + 0.3 = 1.92m

source /opt/ros/jazzy/setup.bash
. install/setup.bash
export ROS_DOMAIN_ID=30

wait_for_topic_once() {
  local topic="$1"
  shift || true
  local extra=("$@")
  echo "[wait] waiting for first message on ${topic} ..."
  until timeout 3s ros2 topic echo "${topic}" --once "${extra[@]}" >/dev/null 2>&1; do
    sleep 1
  done
  echo "[wait] ${topic} is available"
}

IMU_PARAM=$(pwd)/src/pure_imu_undistortion/param/param_ot128.yaml
ODOM_PARAM=$(ros2 pkg prefix pure_lidar_gyro_odometer)/share/pure_lidar_gyro_odometer/param/param_xt_lidar_imu_only.yaml
GNSS_FUSION_PARAM=$(ros2 pkg prefix pure_gnss_map_odom_fusion)/share/pure_gnss_map_odom_fusion/param/param.yaml
NMEA_GNSS_PARAM=$(ros2 pkg prefix pure_nmea_gnss_conversion)/share/pure_nmea_gnss_conversion/param/param_ouster128_dual_antenna.yaml

# === Static TFs: 横向き (センサーキット yaw=0) ===
# LiDAR: base_link → pandar_ot128_top_base_link
gnome-terminal -- ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.76 --z 1.92 \
  --roll 0.0 --pitch 0.0 --yaw 1.57096 \
  --frame-id base_link --child-frame-id pandar_ot128_top

# IMU: base_link → imu
gnome-terminal -- ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.762 --z 1.62 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id tamagawa/imu_link

# プライマリ GNSSアンテナ (右側): base_link → gnss
gnome-terminal -- ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.28 --z 1.62 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id gnss

# セカンダリ GNSSアンテナ (左側): base_link → gnss_secondary
gnome-terminal -- ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 1.24 --z 1.62 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id gnss_secondary

# === Launch localization stack ===
gnome-terminal -- ros2 launch pure_odometry_bringup odometry_container.launch.py \
  use_gnss:=true \
  use_sim_time:=true \
  imu_param:=${IMU_PARAM} \
  odom_param:=${ODOM_PARAM} \
  gnss_fusion_param:=${GNSS_FUSION_PARAM} \
  nmea_gnss_param:=${NMEA_GNSS_PARAM} \
  gnss_primary_gga_topic:=/nmea_sentence \
  gnss_secondary_gga_topic:=/nmea_sentence_secondary \
  use_secondary_gga:=true \
  use_doppler_heading:=false \
  use_imu_yaw_rate_heading:=true \
  stopped_topic:=/localization/is_stopped
sleep 8s

# === Play rosbag ===
gnome-terminal -- ros2 run rosbag2_transport player --ros-args \
  -p storage.uri:=rosbag/rosbag2_2026_05_19-10_18_18_pointcloud \
  -p storage.storage_id:=sqlite3 \
  -p play.rate:=1.0 \
  -p play.start_paused:=false \
  -p play.clock_publish_frequency:=40.0 \
  -p play.qos_profile_overrides_path:=$(pwd)/script/qos_overrides_ot128.yaml \
  --remap /lidar/top/aw_points_ex:=/points_raw \
  --remap /imu/tamagawa/imu_raw:=/imu \
  --remap /gnss/nmea_sentence_main:=/nmea_sentence \
  --remap /gnss/nmea_sentence_sub:=/nmea_sentence_secondary

wait_for_topic_once /localization/points_undistorted --qos-profile sensor_data

gnome-terminal -- ros2 bag record \
  /localization/gyro_lidar_odom \
  /localization/gyro_lidar_odom_filtered \
  /diagnostics \
  /localization/ekf_odom \
  /localization/ekf_pose \
  /localization/gnss_odometry \
  /localization/gnss_fusion_input \
  /localization/global_pose_with_covariance \
  /localization/gnss_confidence

QT_QPA_PLATFORM=xcb ros2 run rqt_robot_monitor rqt_robot_monitor --force-discover
