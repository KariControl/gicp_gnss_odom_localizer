cmake_minimum_required(VERSION 3.14)
project(pure_localization_interface_adapter)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(localization_interface_adapter_node
  src/localization_interface_adapter_node.cpp
)
target_compile_features(localization_interface_adapter_node PUBLIC cxx_std_17)
target_include_directories(localization_interface_adapter_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(localization_interface_adapter_node
  rclcpp
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_ros
)

install(TARGETS localization_interface_adapter_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_acceleration_estimator test/test_acceleration_estimator.cpp)
  target_include_directories(test_acceleration_estimator PRIVATE include)
  target_compile_features(test_acceleration_estimator PRIVATE cxx_std_17)
  add_test(NAME test_acceleration_estimator COMMAND test_acceleration_estimator)

  add_executable(test_odometry_conversion test/test_odometry_conversion.cpp)
  target_include_directories(test_odometry_conversion PRIVATE include)
  target_compile_features(test_odometry_conversion PRIVATE cxx_std_17)
  ament_target_dependencies(test_odometry_conversion geometry_msgs nav_msgs)
  add_test(NAME test_odometry_conversion COMMAND test_odometry_conversion)
endif()

ament_package()
