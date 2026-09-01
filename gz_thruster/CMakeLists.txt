cmake_minimum_required(VERSION 3.16)
project(gz_thruster)

# A plain colcon build sets no build type, which compiles the hot paths
# without optimization. Default to an optimized build with debug info;
# an explicit -DCMAKE_BUILD_TYPE always wins.
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
find_package(gz-common REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

# --------------------------------------------------------------------------
# The thruster system (gz-maritime-thruster-system). Upstream builds this file
# with gz_add_system(), a gz-sim-internal macro that is not available out of
# tree, so the target is declared directly. The source is otherwise vendored
# unmodified; see PROVENANCE.md.
#
# The library name deliberately differs from gz-sim's own
# gz-sim-thruster-system: both end up on GZ_SIM_SYSTEM_PLUGIN_PATH, and a model
# selects one by <plugin filename="...">.
# --------------------------------------------------------------------------
add_library(gz-maritime-thruster-system SHARED src/Thruster.cc)
target_include_directories(gz-maritime-thruster-system PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_link_libraries(gz-maritime-thruster-system
  PRIVATE
    gz-sim::core
    gz-plugin::register
    gz-math::gz-math
    gz-common::gz-common
    gz-transport::gz-transport
    gz-msgs::gz-msgs)

install(TARGETS gz-maritime-thruster-system
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  # Gazebo (Allman) style, matching the vendored upstream source. Enforcing
  # this repository's style here would mean reformatting a file whose value
  # is that it is diffable against upstream.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# GZ_SIM_SYSTEM_PLUGIN_PATH hook so gz-sim finds this package's system plugin
# library in an isolated colcon layout.
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_package()
