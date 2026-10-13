cmake_minimum_required(VERSION 3.16)
project(gz_marked_shapes)

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
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED COMPONENTS graphics)

# --------------------------------------------------------------------------
# The collisions a vehicle marks for a fluid (gz:wind="true",
# gz:ocean_current="true"): their shapes and areas, the cut at the water
# level, the links that carry them, found whenever they show up, and the
# quadratic drag the fluid applies to them. Shared by the wind and the ocean
# current systems, so neither carries a copy.
# --------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED src/MarkedShapes.cc)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    gz-sim::core
    gz-math::gz-math
  PRIVATE
    gz-common::gz-common
    gz-common::graphics
)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # The cut at the water level, the shapes and the links, without a world.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(marked_shapes_test test/marked_shapes_test.cc)
  if(TARGET marked_shapes_test)
    target_compile_definitions(marked_shapes_test PRIVATE
      "TEST_MEDIA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/media\"")
    target_link_libraries(marked_shapes_test ${PROJECT_NAME}
      gz-common::gz-common)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
# The library's public targets link gz-sim::core and gz-math::gz-math, so a
# consumer's find_package(gz_marked_shapes) must find them too: the vendor
# packages only put them on the path. gz-common stays private.
ament_export_dependencies(gz_sim_vendor gz-sim gz_math_vendor gz-math)

ament_package()
