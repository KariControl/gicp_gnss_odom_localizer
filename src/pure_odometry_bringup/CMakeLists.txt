cmake_minimum_required(VERSION 3.8)
project(pure_odometry_bringup)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(localization_visualization_node
  src/localization_visualization_node.cpp
)
target_compile_features(localization_visualization_node PUBLIC cxx_std_17)
target_include_directories(localization_visualization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(localization_visualization_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  visualization_msgs
)

add_library(pure_odometry_localization_status_rviz_plugin SHARED
  include/pure_odometry_bringup/localization_status_display.hpp
  src/localization_status_display.cpp
)
set_target_properties(pure_odometry_localization_status_rviz_plugin PROPERTIES AUTOMOC ON)
target_compile_features(pure_odometry_localization_status_rviz_plugin PUBLIC cxx_std_17)
target_include_directories(pure_odometry_localization_status_rviz_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pure_odometry_localization_status_rviz_plugin
  diagnostic_msgs
  nav_msgs
  pluginlib
  rclcpp
  rviz_common
  tf2
)
target_link_libraries(pure_odometry_localization_status_rviz_plugin Qt5::Widgets)

install(TARGETS localization_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS pure_odometry_localization_status_rviz_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(
  rviz_common localization_status_plugin.xml
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_localization_visualization_logic
    test/test_localization_visualization_logic.cpp
  )
  target_include_directories(test_localization_visualization_logic PRIVATE include)
  ament_target_dependencies(test_localization_visualization_logic diagnostic_msgs)
endif()

ament_package()
