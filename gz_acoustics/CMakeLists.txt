cmake_minimum_required(VERSION 3.16)
project(gz_acoustics)

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
find_package(gz_sensors_vendor REQUIRED)
find_package(gz-sensors REQUIRED)

# --------------------------------------------------------------------------
# Core library: the pinger as a component, and the rules of who hears what,
# apart from any world. Every plugin that reads or writes a pinger links it,
# so the component is registered in whichever process loads one.
# --------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/components.cc
  src/Pinger.cc
  src/Hearing.cc
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    gz-sim::core
    gz-math::gz-math
)

# --------------------------------------------------------------------------
# The pinger: a model plugin that puts a pinger on a link.
# --------------------------------------------------------------------------
add_library(gz-maritime-acoustic-pinger-system SHARED
  src/systems/AcousticPinger.cc
)
target_link_libraries(gz-maritime-acoustic-pinger-system
  PRIVATE
    ${PROJECT_NAME}
    gz-plugin::register
    gz-common::gz-common
    gz-transport::core
    gz-msgs::core)

# --------------------------------------------------------------------------
# The acoustics: a world system that makes pingers ping and reads the
# acoustic receivers, a gz-sensors custom sensor.
# --------------------------------------------------------------------------
add_library(gz-maritime-acoustics-system SHARED
  src/systems/Acoustics.cc
  src/systems/AcousticReceiver.cc
)
target_link_libraries(gz-maritime-acoustics-system
  PRIVATE
    ${PROJECT_NAME}
    gz-plugin::register
    gz-common::gz-common
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
install(TARGETS gz-maritime-acoustic-pinger-system gz-maritime-acoustics-system
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # The pinger and the rules, without a world.
  ament_add_gtest(acoustics_core_test test/acoustics_core_test.cc)
  if(TARGET acoustics_core_test)
    target_link_libraries(acoustics_core_test ${PROJECT_NAME})
  endif()

  # The systems in running worlds.
  ament_add_gtest(acoustics_test test/acoustics_test.cc TIMEOUT 300)
  if(TARGET acoustics_test)
    target_compile_definitions(acoustics_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_link_libraries(acoustics_test
      ${PROJECT_NAME}
      gz-transport::core
      gz-msgs::core)
    # The plugin libraries are loaded by name from the build tree so the
    # test runs before install.
    set_tests_properties(acoustics_test PROPERTIES
      ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}")
  endif()
endif()

# GZ_SIM_SYSTEM_PLUGIN_PATH hook so gz-sim finds the system plugin libraries
# in an isolated colcon layout.
ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(gz_sim_vendor gz_math_vendor)

ament_package()
