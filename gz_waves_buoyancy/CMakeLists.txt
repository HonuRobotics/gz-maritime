cmake_minimum_required(VERSION 3.16)
project(gz_waves_buoyancy)

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
find_package(sdformat_vendor REQUIRED)
find_package(sdformat REQUIRED)
find_package(gz_waves REQUIRED)
find_package(gz_marked_shapes REQUIRED)

# --------------------------------------------------------------------------
# How much of a shape is under a sloping water surface: internal, built into
# the system and its test.
# --------------------------------------------------------------------------
add_library(wave_displacement STATIC src/Displacement.cc)
set_target_properties(wave_displacement PROPERTIES
  POSITION_INDEPENDENT_CODE ON)
target_include_directories(wave_displacement PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(wave_displacement
  PUBLIC
    gz_marked_shapes::gz_marked_shapes
    gz-math::gz-math
    sdformat::sdformat
)

# --------------------------------------------------------------------------
# The world system: buoyancy on the wave field.
# --------------------------------------------------------------------------
# The WaveCoupling tag, a header only component: the wave buoyancy writes
# it, and a system that damps against the water reads it.
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE gz-sim::core)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

add_library(gz-maritime-wave-buoyancy-system SHARED
  src/systems/WaveBuoyancy.cc
)
target_link_libraries(gz-maritime-wave-buoyancy-system
  PRIVATE
    ${PROJECT_NAME}
    wave_displacement
    gz_waves::gz_waves
    gz-sim::core
    gz-plugin::register
    gz-common::gz-common)

install(TARGETS gz-maritime-wave-buoyancy-system
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # The volume under a sloping surface, without a world.
  ament_add_gtest(displacement_test test/displacement_test.cc)
  if(TARGET displacement_test)
    target_link_libraries(displacement_test wave_displacement)
  endif()

  # The system on waves.
  ament_add_gtest(wave_buoyancy_test test/wave_buoyancy_test.cc TIMEOUT 300)
  if(TARGET wave_buoyancy_test)
    target_compile_definitions(wave_buoyancy_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_link_libraries(wave_buoyancy_test gz_waves::gz_waves gz-sim::core)
    # The plugin library is loaded by name from the build tree so the test
    # runs before install; the wave engine comes from the install.
    set_tests_properties(wave_buoyancy_test PROPERTIES
      ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{GZ_SIM_SYSTEM_PLUGIN_PATH}")
  endif()
endif()

ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(gz_sim_vendor)

ament_package()
