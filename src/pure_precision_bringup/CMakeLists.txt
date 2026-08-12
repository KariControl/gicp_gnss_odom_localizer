cmake_minimum_required(VERSION 3.8)
project(pure_precision_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/aggregate_precision_glim_ab.py
  scripts/evaluate_accepted_scan_nonintrusion.py
  scripts/evaluate_precision_glim_ab.py
  scripts/evaluate_startup_acceptance.py
  scripts/validate_hesai_gnss_publication_run.py
  scripts/validate_precision_bag.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_precision_evaluation_helpers
    COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_precision_evaluation_helpers.py"
  )
  add_test(
    NAME test_hesai_gnss_publication_validator
    COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_hesai_gnss_publication_validator.py"
  )
  add_test(
    NAME test_hesai_gnss_publication_validator_self_test
    COMMAND "${Python3_EXECUTABLE}" -B
      "${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_hesai_gnss_publication_run.py"
      --self-test
  )
endif()

ament_package()
