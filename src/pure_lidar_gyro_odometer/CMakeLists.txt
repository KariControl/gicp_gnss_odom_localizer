cmake_minimum_required(VERSION 3.14)
project(pure_lidar_gyro_odometer)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pure_lidar_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(small_gicp REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED
  src/gyro_odometer_node.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  diagnostic_msgs
  pure_lidar_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  small_gicp
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
if(TARGET small_gicp::small_gicp)
  target_link_libraries(${PROJECT_NAME} small_gicp::small_gicp)
endif()

rclcpp_components_register_nodes(${PROJECT_NAME} "pure_gyro_odometer::GyroOdometerNode")

add_executable(${PROJECT_NAME}_node
  src/gyro_odometer_node_main.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_se2_fixed_lag_smoother test/test_se2_fixed_lag_smoother.cpp)
  target_include_directories(test_se2_fixed_lag_smoother PRIVATE include)
  target_compile_features(test_se2_fixed_lag_smoother PRIVATE cxx_std_17)
  add_test(NAME test_se2_fixed_lag_smoother COMMAND test_se2_fixed_lag_smoother)

  add_executable(test_yaw_rate_integrator test/test_yaw_rate_integrator.cpp)
  target_include_directories(test_yaw_rate_integrator PRIVATE include)
  target_compile_features(test_yaw_rate_integrator PRIVATE cxx_std_17)
  add_test(NAME test_yaw_rate_integrator COMMAND test_yaw_rate_integrator)

  add_executable(
    test_imu_linear_acceleration
    test/test_imu_linear_acceleration.cpp)
  target_include_directories(test_imu_linear_acceleration PRIVATE include)
  target_compile_features(test_imu_linear_acceleration PRIVATE cxx_std_17)
  target_link_libraries(test_imu_linear_acceleration Eigen3::Eigen)
  add_test(
    NAME test_imu_linear_acceleration
    COMMAND test_imu_linear_acceleration)

  add_executable(test_callback_groups test/test_callback_groups.cpp)
  target_include_directories(test_callback_groups PRIVATE include)
  target_compile_features(test_callback_groups PRIVATE cxx_std_17)
  target_link_libraries(test_callback_groups ${PROJECT_NAME})
  ament_target_dependencies(test_callback_groups rclcpp)
  add_test(NAME test_callback_groups COMMAND test_callback_groups)

  add_executable(test_tf_publication test/test_tf_publication.cpp)
  target_include_directories(test_tf_publication PRIVATE include)
  target_compile_features(test_tf_publication PRIVATE cxx_std_17)
  target_link_libraries(test_tf_publication ${PROJECT_NAME})
  ament_target_dependencies(test_tf_publication rclcpp nav_msgs tf2_ros)
  add_test(NAME test_tf_publication COMMAND test_tf_publication)

  add_executable(test_tracking_mode test/test_tracking_mode.cpp)
  target_include_directories(test_tracking_mode PRIVATE include)
  target_compile_features(test_tracking_mode PRIVATE cxx_std_17)
  add_test(NAME test_tracking_mode COMMAND test_tracking_mode)

  add_executable(test_stop_state_estimator test/test_stop_state_estimator.cpp)
  target_include_directories(test_stop_state_estimator PRIVATE include)
  target_compile_features(test_stop_state_estimator PRIVATE cxx_std_17)
  add_test(NAME test_stop_state_estimator COMMAND test_stop_state_estimator)

  add_executable(test_observability_policy test/test_observability_policy.cpp)
  target_include_directories(test_observability_policy PRIVATE include)
  target_compile_features(test_observability_policy PRIVATE cxx_std_17)
  add_test(NAME test_observability_policy COMMAND test_observability_policy)

  add_executable(
    test_accepted_scan_snapshot_policy
    test/test_accepted_scan_snapshot_policy.cpp)
  target_include_directories(test_accepted_scan_snapshot_policy PRIVATE include)
  target_compile_features(test_accepted_scan_snapshot_policy PRIVATE cxx_std_17)
  add_test(
    NAME test_accepted_scan_snapshot_policy
    COMMAND test_accepted_scan_snapshot_policy)

  add_executable(
    test_accepted_scan_odometry
    test/test_accepted_scan_odometry.cpp)
  target_include_directories(test_accepted_scan_odometry PRIVATE include)
  target_compile_features(test_accepted_scan_odometry PRIVATE cxx_std_17)
  ament_target_dependencies(test_accepted_scan_odometry nav_msgs)
  add_test(
    NAME test_accepted_scan_odometry
    COMMAND test_accepted_scan_odometry)
endif()

ament_export_dependencies(
  rclcpp rclcpp_components sensor_msgs geometry_msgs nav_msgs std_msgs diagnostic_msgs
  pure_lidar_msgs
  tf2 tf2_ros tf2_geometry_msgs pcl_conversions small_gicp
  eigen3_cmake_module Eigen3
)

ament_package()
