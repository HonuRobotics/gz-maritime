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

# --------------------------------------------------------------------------
# The sonar engine: ray hits in, echo intensity per beam and range sample
# out, and the readouts of the imaging, sidescan and profiling sonars. No
# Gazebo in the loop, so it can be tested and run on its own; the synthetic
# scene stands in for gz-rendering.
# --------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
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
target_link_libraries(${PROJECT_NAME} PUBLIC gz-math::gz-math)

# Renders one ping of a synthetic scene to files, for the contributor
# walkthrough "Render a ping without Gazebo".
add_executable(gz_sonar_render_ping examples/render_ping.cc)
target_link_libraries(gz_sonar_render_ping ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS gz_sonar_render_ping DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS examples/draw_ping.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # The analytic cases of the testing plan, on synthetic scenes.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(engine_test test/engine_test.cc)
  if(TARGET engine_test)
    target_link_libraries(engine_test ${PROJECT_NAME})
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(gz_math_vendor gz-math)

ament_package()
