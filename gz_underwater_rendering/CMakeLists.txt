cmake_minimum_required(VERSION 3.16)
project(gz_underwater_rendering)

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
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering REQUIRED COMPONENTS ogre2)
find_package(gz_water_optics REQUIRED)
# The Wavefield component, for whether the world has waves
find_package(gz_waves REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

# OGRE-Next headers: the copy gz-rendering-ogre2 was built against, found as
# gz_waves_rendering finds them.
find_package(gz_waves_rendering REQUIRED)
gz_find_ogre_next()

set(PIECES_DIR share/${PROJECT_NAME}/hlms/Underwater)

# --------------------------------------------------------------------------
# The Ogre side, a library of its own loaded with dlopen, so the plugin
# never links gz-rendering's ogre2 library (see gz_waves_rendering).
# --------------------------------------------------------------------------
add_library(underwater-ogre2-bridge SHARED
  src/systems/UnderwaterOgre2Bridge.cc)
target_link_libraries(underwater-ogre2-bridge
  PRIVATE
    gz-rendering::gz-rendering
    gz-rendering::ogre2
    gz-common::gz-common)
target_include_directories(underwater-ogre2-bridge SYSTEM PRIVATE
  ${OGRE_NEXT_INCLUDE_DIR}
  ${OGRE_NEXT_INCLUDE_DIR}/Hlms/Common
  ${OGRE_NEXT_INCLUDE_DIR}/Hlms/Pbs)
target_compile_definitions(underwater-ogre2-bridge PRIVATE OGRE_IGNORE_UNKNOWN_DEBUG)
# Ogre's own libraries: gz-rendering links them privately, so their symbols
# (Vector3::ZERO, HlmsPbs, the ArchiveManager) must be linked here.
get_filename_component(_ogre_root "${OGRE_NEXT_INCLUDE_DIR}/../.." ABSOLUTE)
find_library(OGRE_NEXT_MAIN NAMES OgreNextMain OgreMain
  HINTS ${_ogre_root}/lib /opt/ros/$ENV{ROS_DISTRO}/opt/gz_ogre_next_vendor/lib)
find_library(OGRE_NEXT_HLMS_PBS NAMES OgreNextHlmsPbs OgreHlmsPbs
  HINTS ${_ogre_root}/lib /opt/ros/$ENV{ROS_DISTRO}/opt/gz_ogre_next_vendor/lib)
if(NOT OGRE_NEXT_MAIN OR NOT OGRE_NEXT_HLMS_PBS)
  message(FATAL_ERROR "OGRE-Next libraries (OgreNextMain, OgreNextHlmsPbs) not found")
endif()
target_link_libraries(underwater-ogre2-bridge PRIVATE ${OGRE_NEXT_MAIN} ${OGRE_NEXT_HLMS_PBS})

# --------------------------------------------------------------------------
# The plugin, on a visual: in the server's sensors and in the GUI.
# --------------------------------------------------------------------------
add_library(gz-maritime-underwater-rendering-system SHARED
  src/systems/UnderwaterRendering.cc)
target_link_libraries(gz-maritime-underwater-rendering-system
  PRIVATE
    gz_water_optics::gz_water_optics
    gz_waves::gz_waves
    gz-transport::core
    gz-msgs::core
    gz-sim::core
    gz-plugin::register
    gz-rendering::gz-rendering
    gz-common::gz-common
    ${CMAKE_DL_LIBS})
target_compile_definitions(gz-maritime-underwater-rendering-system PRIVATE
  "UW_PIECES_DIR=\"${CMAKE_INSTALL_PREFIX}/${PIECES_DIR}\"")

install(TARGETS underwater-ogre2-bridge gz-maritime-underwater-rendering-system
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY share/hlms/ DESTINATION share/${PROJECT_NAME}/hlms)
install(DIRECTORY share/models/ DESTINATION share/${PROJECT_NAME}/models)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(gz_underwater_camera REQUIRED)

  # The GPU path against the reference camera, and what it must leave alone.
  # Needs a GPU. Runs the same scene without and with the GPU path, in one
  # process, one after the other.
  ament_add_gtest(underwater_rendering_test test/underwater_rendering_test.cc
    TIMEOUT 600)
  if(TARGET underwater_rendering_test)
    target_compile_definitions(underwater_rendering_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\""
      "DOCS_REFERENCE=\"${CMAKE_CURRENT_SOURCE_DIR}/../docs/reference/launch-and-world.md\"")
    target_link_libraries(underwater_rendering_test
      gz_underwater_camera::gz_underwater_camera
      gz_water_optics::gz_water_optics
      gz-sim::core
      gz-transport::core
      gz-msgs::core)
    set_tests_properties(underwater_rendering_test PROPERTIES
      ENVIRONMENT
      "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{GZ_SIM_SYSTEM_PLUGIN_PATH};LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH};GZ_UNDERWATER_HLMS_DIR=${CMAKE_CURRENT_SOURCE_DIR}/share/hlms/Underwater;GZ_SIM_RESOURCE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/share/models:$ENV{GZ_SIM_RESOURCE_PATH};GZ_PARTITION=gz_underwater_rendering_test")
  endif()

  # Lamps at night: the lit plate faded both ways and the beam, against the
  # model integrated in the test. Needs a GPU.
  ament_add_gtest(lamps_test test/lamps_test.cc TIMEOUT 600)
  if(TARGET lamps_test)
    target_compile_definitions(lamps_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_link_libraries(lamps_test
      gz_underwater_camera::gz_underwater_camera
      gz_water_optics::gz_water_optics
      gz-sim::core
      gz-transport::core
      gz-msgs::core)
    set_tests_properties(lamps_test PROPERTIES
      ENVIRONMENT
      "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{GZ_SIM_SYSTEM_PLUGIN_PATH};LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH};GZ_UNDERWATER_HLMS_DIR=${CMAKE_CURRENT_SOURCE_DIR}/share/hlms/Underwater;GZ_SIM_RESOURCE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/share/models:$ENV{GZ_SIM_RESOURCE_PATH};GZ_PARTITION=gz_underwater_lamps_test")
  endif()

  # The surface from below: Snell's window, in gz_waves_rendering's water
  # shader. Needs a GPU.
  ament_add_gtest(snell_test test/snell_test.cc TIMEOUT 300)
  if(TARGET snell_test)
    target_compile_definitions(snell_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_link_libraries(snell_test gz-sim::core gz-transport::core gz-msgs::core)
    set_tests_properties(snell_test PROPERTIES
      ENVIRONMENT "GZ_PARTITION=gz_underwater_snell_test")
  endif()

  # Caustics and marine snow. Needs a GPU.
  ament_add_gtest(effects_test test/effects_test.cc TIMEOUT 600)
  if(TARGET effects_test)
    target_compile_definitions(effects_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_link_libraries(effects_test
      gz_underwater_camera::gz_underwater_camera
      gz-sim::core gz-transport::core gz-msgs::core)
    set_tests_properties(effects_test PROPERTIES
      ENVIRONMENT
      "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{GZ_SIM_SYSTEM_PLUGIN_PATH};LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH};GZ_UNDERWATER_HLMS_DIR=${CMAKE_CURRENT_SOURCE_DIR}/share/hlms/Underwater;GZ_SIM_RESOURCE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/share/models:$ENV{GZ_SIM_RESOURCE_PATH};GZ_PARTITION=gz_underwater_effects_test")
  endif()

  # The renderer cannot install (no pieces there): the reference camera
  # draws the water. A process of its own, so nothing is installed before.
  ament_add_gtest(underwater_rendering_fallback_test
    test/underwater_rendering_fallback_test.cc TIMEOUT 300)
  if(TARGET underwater_rendering_fallback_test)
    target_compile_definitions(underwater_rendering_fallback_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_link_libraries(underwater_rendering_fallback_test
      gz-sim::core
      gz-transport::core
      gz-msgs::core)
    set_tests_properties(underwater_rendering_fallback_test PROPERTIES
      ENVIRONMENT
      "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{GZ_SIM_SYSTEM_PLUGIN_PATH};LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH};GZ_UNDERWATER_HLMS_DIR=${CMAKE_CURRENT_BINARY_DIR}/no_such_pieces;GZ_SIM_RESOURCE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/share/models:$ENV{GZ_SIM_RESOURCE_PATH};GZ_PARTITION=gz_underwater_rendering_fallback_test")
  endif()
  # Turned off and on while running, and its model removed. A process of
  # its own, so nothing is installed before.
  ament_add_gtest(underwater_rendering_runtime_test
    test/underwater_rendering_runtime_test.cc TIMEOUT 600)
  if(TARGET underwater_rendering_runtime_test)
    target_compile_definitions(underwater_rendering_runtime_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_link_libraries(underwater_rendering_runtime_test
      gz-sim::core
      gz-transport::core
      gz-msgs::core)
    set_tests_properties(underwater_rendering_runtime_test PROPERTIES
      ENVIRONMENT
      "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{GZ_SIM_SYSTEM_PLUGIN_PATH};LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH};GZ_UNDERWATER_HLMS_DIR=${CMAKE_CURRENT_SOURCE_DIR}/share/hlms/Underwater;GZ_SIM_RESOURCE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/share/models:$ENV{GZ_SIM_RESOURCE_PATH};GZ_PARTITION=gz_underwater_rendering_runtime_test")
  endif()
endif()

ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_package()
