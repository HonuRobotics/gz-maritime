cmake_minimum_required(VERSION 3.16)
project(gz_underwater_camera)

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
find_package(gz_water_optics REQUIRED)

# --------------------------------------------------------------------------
# The image library: one in-air frame (colour and z-depth) in, the frame the
# camera sees through the water out. No Gazebo in it, so every stage is
# tested on its own.
# --------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/UnderwaterImage.cc
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC gz-math::gz-math)

# --------------------------------------------------------------------------
# The plugin that goes inside an rgbd_camera sensor.
# --------------------------------------------------------------------------
add_library(gz-maritime-underwater-camera-system SHARED
  src/systems/UnderwaterCamera.cc
)
target_link_libraries(gz-maritime-underwater-camera-system
  PRIVATE
    ${PROJECT_NAME}
    gz_water_optics::gz_water_optics
    gz-sim::core
    gz-plugin::register
    gz-common::gz-common
    gz-transport::core
    gz-msgs::core)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS gz-maritime-underwater-camera-system
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Every stage of the image pipeline, and its throughput.
  ament_add_gtest(underwater_image_test test/underwater_image_test.cc)
  if(TARGET underwater_image_test)
    target_link_libraries(underwater_image_test ${PROJECT_NAME})
    target_compile_definitions(underwater_image_test PRIVATE
      "DOCS_REFERENCE=\"${CMAKE_CURRENT_SOURCE_DIR}/../docs/reference/launch-and-world.md\""
      "PACKAGE_README=\"${CMAKE_CURRENT_SOURCE_DIR}/README.md\"")
  endif()

  # The plugin in a rendered world: the published image against the model
  # computed here pixel by pixel from the in-air frame. Needs a GPU.
  ament_add_gtest(underwater_camera_test test/underwater_camera_test.cc
    TIMEOUT 300)
  if(TARGET underwater_camera_test)
    target_compile_definitions(underwater_camera_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_link_libraries(underwater_camera_test
      ${PROJECT_NAME}
      gz_water_optics::gz_water_optics
      gz-sim::core
      gz-transport::core
      gz-msgs::core)
    set_tests_properties(underwater_camera_test PROPERTIES
      ENVIRONMENT
      "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{GZ_SIM_SYSTEM_PLUGIN_PATH};GZ_PARTITION=gz_underwater_camera_test")
  endif()

  # The plugin fed frames by the test, in worlds with nothing rendered: no
  # GPU needed.
  ament_add_gtest(underwater_camera_frames_test
    test/underwater_camera_frames_test.cc TIMEOUT 300)
  if(TARGET underwater_camera_frames_test)
    target_compile_definitions(underwater_camera_frames_test PRIVATE
      "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/worlds\"")
    target_link_libraries(underwater_camera_frames_test
      gz-sim::core
      gz-transport::core
      gz-msgs::core)
    set_tests_properties(underwater_camera_frames_test PROPERTIES
      ENVIRONMENT
      "GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{GZ_SIM_SYSTEM_PLUGIN_PATH};GZ_PARTITION=gz_underwater_camera_frames_test")
  endif()
endif()

ament_environment_hooks("hooks/hook.dsv.in")
ament_environment_hooks("hooks/hook.sh.in")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(gz_math_vendor gz-math)

ament_package()
