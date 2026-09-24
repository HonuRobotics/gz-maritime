cmake_minimum_required(VERSION 3.16)
project(gz_wind)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No CMAKE_BUILD_TYPE given; defaulting to RelWithDebInfo")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
    STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED COMPONENTS graphics)

add_library(gz-maritime-wind-system SHARED src/Wind.cc)
target_include_directories(gz-maritime-wind-system
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_link_libraries(gz-maritime-wind-system
  PRIVATE
    gz-sim::core
    gz-plugin::register
    gz-math::gz-math
    gz-common::gz-common
    gz-common::graphics)

install(TARGETS gz-maritime-wind-system
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(gz_transport_vendor REQUIRED)
  find_package(gz-transport REQUIRED)
  find_package(gz_msgs_vendor REQUIRED)
  find_package(gz-msgs REQUIRED)

  ament_add_gtest(wind_test test/wind_test.cc)
  if(TARGET wind_test)
    target_compile_definitions(wind_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_link_libraries(wind_test
      gz-sim::core
      gz-math::gz-math
      gz-transport::core
      gz-msgs::core)
    # The plugin library is loaded by name from the build tree so the test
    # runs before install.
    set_tests_properties(wind_test PROPERTIES
      ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}")
  endif()
endif()

# GZ_SIM_SYSTEM_PLUGIN_PATH hook so gz-sim finds this package's system plugin
# library in an isolated colcon layout.
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_package()
