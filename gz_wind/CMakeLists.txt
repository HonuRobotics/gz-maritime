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
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_sensors_vendor REQUIRED)
find_package(gz-sensors REQUIRED)

# --------------------------------------------------------------------------
# Core library: the wind as a recipe (the Windfield component), the registry
# of wind models, the models built in, and the sampler any system uses to ask
# the wind at a point. Every plugin that reads or writes the wind links it, so
# the component is registered in whichever process loads one.
# --------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/components.cc
  src/Windfield.cc
  src/WindModel.cc
  src/WindSampler.cc
  src/StandardModel.cc
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    gz-sim::core
    gz-math::gz-math
  PRIVATE
    gz-common::gz-common
)

# --------------------------------------------------------------------------
# The wind system: owns the recipe, the wind topic and the ground truth, and
# pushes on the collisions a vehicle marks gz:wind="true", and reads the
# anemometers, a gz-sensors custom sensor.
# --------------------------------------------------------------------------
add_library(gz-maritime-wind-system SHARED
  src/systems/Wind.cc
  src/systems/Anemometer.cc
)
target_link_libraries(gz-maritime-wind-system
  PRIVATE
    ${PROJECT_NAME}
    gz-plugin::register
    gz-common::gz-common
    gz-common::graphics
    gz-sensors::gz-sensors
    gz-transport::core
    gz-msgs::core)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
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

  # The recipe, the registry, the models and the sampler, without a world.
  ament_add_gtest(wind_core_test test/wind_core_test.cc)
  if(TARGET wind_core_test)
    target_link_libraries(wind_core_test ${PROJECT_NAME})
  endif()

  # The system in running worlds.
  ament_add_gtest(wind_test test/wind_test.cc)
  if(TARGET wind_test)
    target_compile_definitions(wind_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_link_libraries(wind_test
      ${PROJECT_NAME}
      gz-transport::core
      gz-msgs::core)
    # The plugin library is loaded by name from the build tree so the test
    # runs before install.
    set_tests_properties(wind_test PROPERTIES
      ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}")
  endif()
endif()

# GZ_SIM_SYSTEM_PLUGIN_PATH hook so gz-sim finds the system plugin library in
# an isolated colcon layout.
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(gz_sim_vendor gz_math_vendor)

ament_package()
