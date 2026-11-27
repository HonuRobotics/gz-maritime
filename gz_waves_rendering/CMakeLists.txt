cmake_minimum_required(VERSION 3.16)
project(gz_waves_rendering)

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
find_package(gz_waves REQUIRED)   # IWaveField + WaveField2D + Wavefield component
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering REQUIRED COMPONENTS ogre2)
# NOTE: this package is provider-agnostic — it does NOT depend on the wave
# engines. WaterVisual rebuilds its private engine through the gz_waves core
# registry (CreateWaveSimulation); the GUI-side factory registration lives in
# the per-engine GUI plugins (gz-sim-waves-<engine>-gui), loaded by the world.

# The Ogre Next headers, found as every package that talks to Ogre directly
# finds them: cmake/gz_ogre_next-extras.cmake, installed for them.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/gz_ogre_next-extras.cmake)
gz_find_ogre_next()

# The Ogre Next dynamic heightmap-texture helper lives in a SEPARATE shared
# library. Linking it directly into WaterVisual would force the plugin's
# DT_NEEDED to include libgz-rendering-ogre2.so, which interferes with
# gz-rendering's own engine-plugin loader (same SONAME already in process when
# gz-rendering tries to dlopen its engine plugin → the engine never initialises
# and the GUI shows a default-material plane). WaterVisual loads this bridge on
# demand via dlopen, so its symbols never enter the WaterVisual plugin's link
# map (mirrors the asv_wave_sim / gz_fft_waves rendering-ogre2 split).
add_library(waves-ogre2-bridge SHARED
  src/systems/Ogre2HeightMapBridge.cc
)
target_link_libraries(waves-ogre2-bridge
  PRIVATE
    gz-rendering::gz-rendering
    gz-rendering::ogre2
    gz-common::gz-common
)
# SYSTEM so Ogre Next's own headers (extra ';', unused params, etc.) don't
# drown our -Wall -Wextra -Wpedantic output — they are upstream Ogre, not ours.
target_include_directories(waves-ogre2-bridge SYSTEM PRIVATE
  ${OGRE_NEXT_INCLUDE_DIR}
  # HlmsPbs headers transitively include Hlms/Common/OgreHlmsBufferManager.h
  # by unqualified name — that subdirectory must also be on the path.
  ${OGRE_NEXT_INCLUDE_DIR}/Hlms/Common
  ${OGRE_NEXT_INCLUDE_DIR}/Hlms/Pbs
)
# Ogre can't infer debug/release from our flags and emits a #pragma message in
# every TU; silence it (RelWithDebInfo is not an Ogre debug build).
target_compile_definitions(waves-ogre2-bridge PRIVATE OGRE_IGNORE_UNKNOWN_DEBUG)
install(
  TARGETS waves-ogre2-bridge
  LIBRARY DESTINATION lib
)

# WaterVisual: a gz-sim rendering system. Reads the wave field from the
# Wavefield component through IWaveField::Field() (gz_waves core) and uploads
# it as a displacement heightmap. Links only the engine-agnostic gz-rendering
# core; the Ogre2 bridge is loaded via dlopen (CMAKE_DL_LIBS).
add_library(gz-sim-water-visual-system SHARED
  src/systems/WaterVisual.cc
  src/systems/HeightMapTexture.cc
)
target_include_directories(gz-sim-water-visual-system PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/systems>
  ${gz_waves_INCLUDE_DIRS})
target_link_libraries(gz-sim-water-visual-system
  PRIVATE
    gz_waves::gz_waves        # IWaveField + Field() + Wavefield component +
                              # CreateWaveSimulation (engine-agnostic registry)
    gz-sim::core
    gz-math::gz-math
    gz-plugin::register
    gz-rendering::gz-rendering
    gz-common::gz-common
    ${CMAKE_DL_LIBS}
)
install(
  TARGETS gz-sim-water-visual-system
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Water-surface model + shaders + meshes (model://water_surface).
install(
  DIRECTORY share/models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The drawn surface against the wave field, through a depth camera; it is
  # skipped when the camera gives no image. The visual plugin, its bridge
  # and the water model come from this tree, so the test runs before
  # install, and it brings its own wave field: no engine is needed. Its main
  # sets a GZ_PARTITION of its own, one per process.
  ament_add_gtest(drawn_surface_test test/drawn_surface_test.cc TIMEOUT 300
    SKIP_LINKING_MAIN_LIBRARIES)
  if(TARGET drawn_surface_test)
    target_compile_definitions(drawn_surface_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_include_directories(drawn_surface_test PRIVATE
      ${gz_waves_INCLUDE_DIRS})
    target_link_libraries(drawn_surface_test gz_waves::gz_waves gz-sim::core)
    set(_drawn_surface_env
      "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{GZ_SIM_SYSTEM_PLUGIN_PATH}"
      "LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}"
      "GZ_SIM_RESOURCE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/share/models:$ENV{GZ_SIM_RESOURCE_PATH}")
    set_tests_properties(drawn_surface_test PROPERTIES
      ENVIRONMENT "${_drawn_surface_env}")
  endif()
endif()

# GZ_SIM_RESOURCE_PATH (water_surface model) + GZ_SIM_SYSTEM_PLUGIN_PATH hooks.
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_package(CONFIG_EXTRAS cmake/gz_ogre_next-extras.cmake)
