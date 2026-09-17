cmake_minimum_required(VERSION 3.8)
project(kai_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/instantiate_vehicle.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_instantiate_vehicle test/test_instantiate_vehicle.py
    TIMEOUT 120)
  # The demo needs the vehicle packages; the module skips itself without them.
  # NOCAPTURE so the launch output printed at teardown reaches the test logs.
  ament_add_pytest_test(test_multi_vehicle_demo test/test_multi_vehicle_demo.py
    TIMEOUT 420 NOCAPTURE)
endif()

ament_package()
