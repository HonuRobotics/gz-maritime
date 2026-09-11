cmake_minimum_required(VERSION 3.16)
project(gz_buoyancy)

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
# The buoyancy system (gz-maritime-buoyancy-system). Upstream builds this file
# with gz_add_system(), a gz-sim-internal macro that is not available out of
# tree, so the target is declared directly. The source is otherwise vendored
# with a small additive delta; see PROVENANCE.md.
#
# The library name deliberately differs from gz-sim's own
# gz-sim-buoyancy-system: both end up on GZ_SIM_SYSTEM_PLUGIN_PATH, and a world
# selects one by <plugin filename="...">.
# --------------------------------------------------------------------------
add_library(gz-maritime-buoyancy-system SHARED src/Buoyancy.cc)
target_include_directories(gz-maritime-buoyancy-system PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_link_libraries(gz-maritime-buoyancy-system
  PRIVATE
    gz-sim::core
    gz-plugin::register
    gz-math::gz-math
    gz-common::gz-common
    gz-transport::gz-transport
    gz-msgs::gz-msgs)

# --------------------------------------------------------------------------
# The model-side registration plugin (gz-maritime-buoyancy-enable-system).
# Not vendored: it is this package's own, and exists so a vehicle can declare
# its displacement links to the world system above without the world naming it.
# --------------------------------------------------------------------------
add_library(gz-maritime-buoyancy-enable-system SHARED src/BuoyancyEnable.cc)
target_include_directories(gz-maritime-buoyancy-enable-system PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_link_libraries(gz-maritime-buoyancy-enable-system
  PRIVATE
    gz-sim::core
    gz-plugin::register
    gz-math::gz-math
    gz-common::gz-common
    gz-transport::gz-transport
    gz-msgs::gz-msgs)

install(TARGETS
    gz-maritime-buoyancy-system
    gz-maritime-buoyancy-enable-system
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

  find_package(ament_cmake_gtest REQUIRED)

  # The test drives a headless server over the worlds in test/worlds, which it
  # finds through this path rather than a copy in the build tree.
  ament_add_gtest(buoyancy_test test/buoyancy_test.cc)
  if(TARGET buoyancy_test)
    target_compile_definitions(buoyancy_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_link_libraries(buoyancy_test
      gz-sim::core
      gz-math::gz-math
      gz-common::gz-common
      gz-transport::gz-transport
      gz-msgs::gz-msgs)
    # The server loads the two plugins by filename off
    # GZ_SIM_SYSTEM_PLUGIN_PATH, which the install hook only sets for a sourced
    # workspace. Point it at the build tree so the test runs before install.
    set_tests_properties(buoyancy_test PROPERTIES
      ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}")
  endif()
endif()

# GZ_SIM_SYSTEM_PLUGIN_PATH hook so gz-sim finds this package's system plugin
# libraries in an isolated colcon layout.
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_package()
