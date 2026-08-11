cmake_minimum_required(VERSION 3.14)
project(pure_precision_global_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pure_gnss_msgs REQUIRED)
find_package(pure_lidar_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

add_library(${PROJECT_NAME}_core SHARED
  src/existing_fusion_anchor_tracker.cpp
  src/precision_anchor_estimator.cpp
  src/precision_local_compositor.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)

add_library(${PROJECT_NAME} SHARED
  src/precision_global_localizer_node.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  pure_gnss_msgs
  pure_lidar_msgs
  rclcpp
  tf2
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY param/ DESTINATION share/${PROJECT_NAME}/param)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()

  add_executable(test_precision_anchor_estimator
    test/test_precision_anchor_estimator.cpp)
  target_compile_features(test_precision_anchor_estimator PRIVATE cxx_std_17)
  target_link_libraries(test_precision_anchor_estimator
    ${PROJECT_NAME}_core Eigen3::Eigen)
  add_test(NAME test_precision_anchor_estimator
    COMMAND test_precision_anchor_estimator)

  add_executable(test_precision_local_compositor
    test/test_precision_local_compositor.cpp)
  target_compile_features(test_precision_local_compositor PRIVATE cxx_std_17)
  target_link_libraries(test_precision_local_compositor
    ${PROJECT_NAME}_core Eigen3::Eigen)
  add_test(NAME test_precision_local_compositor
    COMMAND test_precision_local_compositor)

  add_executable(test_existing_fusion_anchor_tracker
    test/test_existing_fusion_anchor_tracker.cpp)
  target_compile_features(test_existing_fusion_anchor_tracker PRIVATE cxx_std_17)
  target_link_libraries(test_existing_fusion_anchor_tracker
    ${PROJECT_NAME}_core Eigen3::Eigen)
  add_test(NAME test_existing_fusion_anchor_tracker
    COMMAND test_existing_fusion_anchor_tracker)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core ${PROJECT_NAME})
ament_export_dependencies(
  diagnostic_msgs
  eigen3_cmake_module
  Eigen3
  geometry_msgs
  nav_msgs
  pure_gnss_msgs
  pure_lidar_msgs
  rclcpp
  tf2
)

ament_package()
