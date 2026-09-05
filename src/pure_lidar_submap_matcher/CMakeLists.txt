cmake_minimum_required(VERSION 3.14)
project(pure_lidar_submap_matcher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(pure_lidar_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(small_gicp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pure_lidar_submap_matcher_node
  src/main.cpp
  src/submap_matcher_node.cpp
)
target_include_directories(pure_lidar_submap_matcher_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(pure_lidar_submap_matcher_node
  diagnostic_msgs geometry_msgs pcl_conversions pure_lidar_msgs rclcpp sensor_msgs
  small_gicp std_msgs tf2 tf2_ros
)
target_link_libraries(pure_lidar_submap_matcher_node Eigen3::Eigen ${PCL_LIBRARIES})
if(TARGET small_gicp::small_gicp)
  target_link_libraries(pure_lidar_submap_matcher_node small_gicp::small_gicp)
endif()

install(TARGETS pure_lidar_submap_matcher_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY param DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_submap_policy test/test_submap_policy.cpp)
  target_include_directories(test_submap_policy PRIVATE include)
  add_test(NAME test_submap_policy COMMAND test_submap_policy)
  add_executable(test_latest_only_queue test/test_latest_only_queue.cpp)
  target_include_directories(test_latest_only_queue PRIVATE include)
  add_test(NAME test_latest_only_queue COMMAND test_latest_only_queue)
endif()

ament_export_dependencies(pure_lidar_msgs)
ament_package()
