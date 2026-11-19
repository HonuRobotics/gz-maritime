cmake_minimum_required(VERSION 3.16)
project(gz_lens_drops)

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

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering REQUIRED COMPONENTS ogre2)

# Discover the Ogre Next headers. We must compile against the SAME Ogre Next the
# resolved gz-rendering-ogre2 uses (mixing patch versions silently swaps in slow
# Hlms/barrier paths). The previous approach parsed gz-rendering::ogre2's
# LOCATION, but that property is empty for imported/interface targets (it is
# NOTFOUND on the ROS gz vendor + Gazebo Jetty), so it wrongly fell back to a
# hardcoded /usr/include/OGRE-2.3. Instead, search for the OGRE-Next headers,
# seeded by the ogre2 target's own interface include dirs. PATH_SUFFIXES exclude
# a bare "OGRE" so we never pick up classic Ogre 1.x (/usr/include/OGRE,
# rviz_ogre_vendor), whose ABI-incompatible headers would break the build.
get_target_property(_ogre2_incs gz-rendering::ogre2 INTERFACE_INCLUDE_DIRECTORIES)

# A cached OGRE_NEXT_INCLUDE_DIR can go stale (e.g. the vendor was uninstalled);
# find_path would keep the old value even though the dir is gone. Drop it first
# so the search below re-runs whenever OgreRoot.h is no longer where we cached.
if(OGRE_NEXT_INCLUDE_DIR AND NOT EXISTS "${OGRE_NEXT_INCLUDE_DIR}/OgreRoot.h")
  unset(OGRE_NEXT_INCLUDE_DIR CACHE)
endif()

# Pass 1: vendor / target-hinted headers only. NO_DEFAULT_PATH skips the system
# dirs so a stray /usr/include/OGRE-2.x (a plain libogre-next-dev) can't shadow
# the gz vendor copy that gz-rendering-ogre2 was actually built against — mixing
# patch versions silently swaps in the slow Hlms/barrier paths.
find_path(OGRE_NEXT_INCLUDE_DIR
  NAMES OgreRoot.h
  HINTS ${_ogre2_incs}
  PATHS /opt/ros/$ENV{ROS_DISTRO}/opt/gz_ogre_next_vendor/include
  PATH_SUFFIXES OGRE-Next OGRE-2.4 OGRE-2.3
  NO_DEFAULT_PATH
)
# Pass 2: fall back to the system paths (a plain libogre-next-dev with no vendor
# present). Only runs when pass 1 left OGRE_NEXT_INCLUDE_DIR unset.
find_path(OGRE_NEXT_INCLUDE_DIR
  NAMES OgreRoot.h
  PATH_SUFFIXES OGRE-Next OGRE-2.4 OGRE-2.3
)
if(NOT OGRE_NEXT_INCLUDE_DIR)
  message(FATAL_ERROR
    "OGRE-Next headers (OgreRoot.h) not found. Install libogre-next-dev or the "
    "gz_ogre_next_vendor package.")
endif()
message(STATUS "Ogre Next headers: ${OGRE_NEXT_INCLUDE_DIR}")

# --------------------------------------------------------------------------
# Core library: the drops on a lens, apart from any world or renderer.
# --------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/DropMap.cc
  src/Drops.cc
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# --------------------------------------------------------------------------
# The Ogre Next bridge: the render pass, the only code that links
# gz-rendering-ogre2 and Ogre. The system loads it with dlopen, so its
# symbols never enter the system's link map; a DT_NEEDED on
# libgz-rendering-ogre2.so breaks gz-rendering's own engine loader (see
# gz_waves_rendering).
# --------------------------------------------------------------------------
add_library(lens-drops-ogre2-bridge SHARED src/systems/Ogre2LensDropsBridge.cc)
target_link_libraries(lens-drops-ogre2-bridge PRIVATE
  gz-rendering::gz-rendering gz-rendering::ogre2 gz-common::gz-common)
target_include_directories(lens-drops-ogre2-bridge SYSTEM PRIVATE ${OGRE_NEXT_INCLUDE_DIR})
target_compile_definitions(lens-drops-ogre2-bridge PRIVATE OGRE_IGNORE_UNKNOWN_DEBUG)
install(TARGETS lens-drops-ogre2-bridge LIBRARY DESTINATION lib)
install(DIRECTORY share/shaders DESTINATION share/${PROJECT_NAME})

# --------------------------------------------------------------------------
# The system: inside a camera sensor, it steps the drops and adds the render
# pass to the camera. Never link gz-rendering::ogre2 here.
# --------------------------------------------------------------------------
add_library(gz-maritime-lens-drops-system SHARED src/systems/LensDrops.cc)
target_compile_definitions(gz-maritime-lens-drops-system PRIVATE
  "LENS_DROPS_SHADER_DIR=\"${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/shaders\"")
target_link_libraries(gz-maritime-lens-drops-system PRIVATE
  ${PROJECT_NAME}
  gz-sim::core
  gz-plugin::register
  gz-rendering::gz-rendering
  gz-common::gz-common
  gz-transport::core
  gz-msgs::core
  ${CMAKE_DL_LIBS})
install(TARGETS gz-maritime-lens-drops-system
  LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # The drop model, without a world or a renderer.
  ament_add_gtest(lens_drops_core_test test/lens_drops_core_test.cc)
  if(TARGET lens_drops_core_test)
    target_link_libraries(lens_drops_core_test ${PROJECT_NAME})
  endif()

  # The drop map, on the CPU.
  ament_add_gtest(drop_map_test test/drop_map_test.cc)
  if(TARGET drop_map_test)
    target_link_libraries(drop_map_test ${PROJECT_NAME})
  endif()

  # The system in a world that does not render. The test sets a
  # GZ_PARTITION of its own in its main, one per process.
  ament_add_gtest(lens_drops_test test/lens_drops_test.cc TIMEOUT 120
    SKIP_LINKING_MAIN_LIBRARIES)
  if(TARGET lens_drops_test)
    target_compile_definitions(lens_drops_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_link_libraries(lens_drops_test
      gz-sim::core gz-transport::core gz-msgs::core)
    # The system is loaded by name from the build tree, so the test runs
    # before install.
    set_tests_properties(lens_drops_test PROPERTIES
      ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}")
  endif()

  # The drops show in a camera's images. Needs a GPU; skips without one.
  # The bridge and the shaders are found in the build and source trees.
  ament_add_gtest(lens_drops_camera_test test/lens_drops_camera_test.cc
    TIMEOUT 180 SKIP_LINKING_MAIN_LIBRARIES)
  if(TARGET lens_drops_camera_test)
    target_compile_definitions(lens_drops_camera_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_link_libraries(lens_drops_camera_test
      gz-sim::core gz-transport::core gz-msgs::core)
    set(_camera_test_env
      "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}"
      "LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}"
      "GZ_LENS_DROPS_SHADER_PATH=${CMAKE_CURRENT_SOURCE_DIR}/share/shaders")
    set_tests_properties(lens_drops_camera_test PROPERTIES
      ENVIRONMENT "${_camera_test_env}")
  endif()
endif()

# GZ_SIM_SYSTEM_PLUGIN_PATH, so gz-sim finds the system, and
# GZ_LENS_DROPS_SHADER_PATH, so it finds the shaders.
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
