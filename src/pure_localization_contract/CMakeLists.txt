cmake_minimum_required(VERSION 3.8)
project(pure_localization_contract)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/tf_ownership_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_tf_ownership_profiles
    COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_tf_ownership_profiles.py"
  )
  set_tests_properties(test_tf_ownership_profiles PROPERTIES TIMEOUT 300)
endif()

ament_package()
