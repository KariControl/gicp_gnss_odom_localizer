cmake_minimum_required(VERSION 3.8)
project(pure_nmea_gnss_conversion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(pure_gnss_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(NMEA_CONVERSION_EXECUTABLE pure_nmea_gga_conversion)

add_executable(${NMEA_CONVERSION_EXECUTABLE}
  src/nmea_gga_conversion_main.cpp
  src/nmea_gga_conversion.cpp
)

target_include_directories(${NMEA_CONVERSION_EXECUTABLE}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${NMEA_CONVERSION_EXECUTABLE} Eigen3::Eigen)

ament_target_dependencies(${NMEA_CONVERSION_EXECUTABLE}
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  nmea_msgs
  pure_gnss_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

install(TARGETS ${NMEA_CONVERSION_EXECUTABLE}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch param config
  DESTINATION share/${PROJECT_NAME}
)


ament_export_dependencies(
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  nmea_msgs
  pure_gnss_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  eigen3_cmake_module
  Eigen3
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_imu_yaw_integrator test/test_imu_yaw_integrator.cpp)
  target_include_directories(test_imu_yaw_integrator PRIVATE include)
  target_compile_features(test_imu_yaw_integrator PRIVATE cxx_std_17)
  add_test(NAME test_imu_yaw_integrator COMMAND test_imu_yaw_integrator)

  add_executable(
    test_trajectory_heading_quality
    test/test_trajectory_heading_quality.cpp
  )
  target_include_directories(test_trajectory_heading_quality PRIVATE include)
  target_compile_features(test_trajectory_heading_quality PRIVATE cxx_std_17)
  add_test(NAME test_trajectory_heading_quality COMMAND test_trajectory_heading_quality)
endif()

ament_package()
