cmake_minimum_required(VERSION 3.16)
project(gz_sonar)

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
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(Threads REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_sensors_vendor REQUIRED)
find_package(gz-sensors REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

# --------------------------------------------------------------------------
# The sonar engine: ray hits in, echo intensity per beam and range sample
# out, and the readouts of the imaging, sidescan and profiling sonars. No
# Gazebo in the loop, so it can be tested and run on its own; the synthetic
# scene stands in for gz-rendering.
# --------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/CameraSampler.cc
  src/Config.cc
  src/Engine.cc
  src/Random.cc
  src/Rays.cc
  src/Readout.cc
  src/SyntheticScene.cc
  src/Water.cc
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC gz-math::gz-math
  PRIVATE Threads::Threads)

# Renders one ping of a synthetic scene to files, for the contributor
# walkthrough "Render a ping without Gazebo".
add_executable(gz_sonar_render_ping examples/render_ping.cc)
target_link_libraries(gz_sonar_render_ping ${PROJECT_NAME})

# --------------------------------------------------------------------------
# The sonar messages: gz.maritime.sonar.SonarImage and SonarLabels, the
# target gz_sonar-msgs (C++, Python, a descriptor set).
# --------------------------------------------------------------------------
gz_msgs_generate_messages(
  TARGET msgs
  PROTO_PACKAGE "gz.maritime.sonar"
  MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
  MSGS_PROTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/maritime/sonar/sonar_image.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/maritime/sonar/sonar_labels.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/maritime/sonar/sonar_detections.proto
  DEPENDENCIES gz-msgs::gz-msgs
)
# The function installs the Python modules under gz/msgs/, where the
# vendor's gz.msgs package shadows them; install them under their own path.
install(DIRECTORY
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-msgs_genmsg/python/gz/maritime
  DESTINATION lib/python/gz)
# Outside a gz-cmake project the function exports no usable include
# directory and no CMake config; export the target the ament way.
target_include_directories(${PROJECT_NAME}-msgs PUBLIC
  "$<INSTALL_INTERFACE:include>")

# --------------------------------------------------------------------------
# The world system and the sensor: finds the sonars, draws their cameras on
# gz-sim's rendering thread, and runs the engine on what they see.
# --------------------------------------------------------------------------
add_library(gz-maritime-sonar-system SHARED
  src/systems/Sonar.cc
  src/systems/SonarSensor.cc
  src/systems/SonarSdf.cc
)
target_include_directories(gz-maritime-sonar-system PRIVATE src/systems)
target_link_libraries(gz-maritime-sonar-system
  PRIVATE
    ${PROJECT_NAME}
    ${PROJECT_NAME}-msgs
    gz-sim::gz-sim
    gz-sensors::gz-sensors
    gz-rendering::gz-rendering
    gz-plugin::register
    gz-common::gz-common
    gz-transport::core
    gz-msgs::core)
install(TARGETS gz-maritime-sonar-system
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Sonar target models and the demo worlds of the walkthroughs; the models
# are found through GZ_SIM_RESOURCE_PATH (hooks).
install(DIRECTORY models worlds DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS gz_sonar_render_ping DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS examples/draw_ping.py examples/capture_sonar.py
  examples/survey_lawnmower.py examples/record_survey.py
  examples/sidescan_mosaic.py examples/soundings_cloud.py examples/compare_wreck.py
  DESTINATION lib/${PROJECT_NAME})
# The mapping examples' shared pose helper, beside them so they import it.
install(FILES examples/survey_pose.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # cpplint and uncrustify as ament configures them check ROS 2 style, and
  # this code follows the Gazebo (Allman) style of AGENTS.md: uncrustify
  # stops on the per-member access specifiers, and cpplint flags an else on
  # its own line and the blank line after each public: member.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # The shipwreck's build script runs in Blender's own Python and keeps the
  # style it was written in; it regenerates the model, which is checked
  # against the glTF profile by its build.sh, not by these linters.
  list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/models/shipwreck/build_shipwreck.py)
  ament_lint_auto_find_test_dependencies()

  # The analytic cases of the testing plan, on synthetic scenes.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(engine_test test/engine_test.cc)
  if(TARGET engine_test)
    target_link_libraries(engine_test ${PROJECT_NAME})
  endif()

  # The camera sampler on synthetic depth and label images.
  ament_add_gtest(sampler_test test/sampler_test.cc)
  if(TARGET sampler_test)
    target_link_libraries(sampler_test ${PROJECT_NAME})
  endif()

  # The camera layout: its limits, and what it refuses before building.
  ament_add_gtest(layout_test test/layout_test.cc)
  if(TARGET layout_test)
    target_link_libraries(layout_test ${PROJECT_NAME})
  endif()

  # The <gz:sonar> parser and its validation. The sweep pings a few hundred
  # configurations; the timeout leaves room for a busy machine.
  ament_add_gtest(sdf_test test/sdf_test.cc src/systems/SonarSdf.cc
    TIMEOUT 300)
  if(TARGET sdf_test)
    target_include_directories(sdf_test PRIVATE src/systems)
    target_link_libraries(sdf_test ${PROJECT_NAME} gz-sim::gz-sim gz-common::gz-common)
  endif()

  # The world system and the sensor in a headless server on scene B. Needs
  # a GPU or llvmpipe; skips without one. The plugin and library paths put
  # this build ahead of an installed copy the environment may also reach.
  ament_add_gtest(sonar_system_test test/sonar_system_test.cc TIMEOUT 120)
  if(TARGET sonar_system_test)
    target_compile_definitions(sonar_system_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\""
      "BUILD_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/\""
      "INSTALL_DIR=\"${CMAKE_INSTALL_PREFIX}/lib/\"")
    target_link_libraries(sonar_system_test ${PROJECT_NAME}-msgs
      gz-sim::gz-sim gz-common::gz-common gz-transport::core gz-msgs::core)
    add_dependencies(sonar_system_test gz-maritime-sonar-system)
    set_tests_properties(sonar_system_test PROPERTIES
      ENVIRONMENT "GZ_PARTITION=gz_sonar_sonar_system_test"
      ENVIRONMENT_MODIFICATION
        "GZ_SIM_SYSTEM_PLUGIN_PATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR};LD_LIBRARY_PATH=path_list_prepend:${CMAKE_CURRENT_BINARY_DIR}")
  endif()
endif()

# GZ_SIM_SYSTEM_PLUGIN_PATH for the system, PYTHONPATH for the messages.
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_targets(${PROJECT_NAME}-msgs HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(gz_math_vendor gz-math gz_msgs_vendor gz-msgs)

ament_package()
