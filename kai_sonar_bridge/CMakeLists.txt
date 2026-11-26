cmake_minimum_required(VERSION 3.16)
project(kai_sonar_bridge)

# The same default as gz_sonar: the conversions transpose and draw every
# ping, which an unoptimized build does much more slowly.
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
find_package(gz_sonar REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# --------------------------------------------------------------------------
# The bridge: one node per vehicle, converting every sonar it is given from
# its gz-transport topic to marine_acoustic_msgs, with optional images for
# viewing. Composable, and an executable of the same name.
# --------------------------------------------------------------------------
# The conversions, free functions with no node, so they are tested alone
# and another package (a viewer, say) can use them: the header is installed
# and the target exported as kai_sonar_bridge::kai_sonar_bridge_conversions.
add_library(${PROJECT_NAME}_conversions SHARED src/Conversions.cc)
target_include_directories(${PROJECT_NAME}_conversions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}_conversions PUBLIC
  gz_sonar::gz_sonar-msgs
  gz-msgs::core
  ${geometry_msgs_TARGETS}
  ${marine_acoustic_msgs_TARGETS}
  ${sensor_msgs_TARGETS})

add_library(${PROJECT_NAME} SHARED src/SonarBridge.cc)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}_conversions
  rclcpp::rclcpp)
target_link_libraries(${PROJECT_NAME} PRIVATE
  gz-transport::core
  rclcpp_components::component)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "kai_sonar_bridge::SonarBridge"
  EXECUTABLE sonar_bridge)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS ${PROJECT_NAME}_conversions
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  gz_sonar gz_msgs_vendor gz-msgs geometry_msgs marine_acoustic_msgs
  sensor_msgs)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# An example ROS 2 consumer of the sonar messages: ros2 run kai_sonar_bridge read_sonar.py
install(PROGRAMS examples/read_sonar.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # cpplint and uncrustify as ament configures them check ROS 2 style, and
  # this code follows the Gazebo (Allman) style of AGENTS.md: uncrustify
  # stops on the per-member access specifiers, and cpplint flags an else on
  # its own line and the blank line after each public: member.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(conversions_test test/conversions_test.cc)
  target_link_libraries(conversions_test ${PROJECT_NAME}_conversions)

  # The node in process, fed over gz-transport. Its own ROS domain, so the
  # topics of anything else running on the machine stay out; the test sets
  # its own gz partition.
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_gtest(sonar_bridge_test test/sonar_bridge_test.cc
    TIMEOUT 180)
  target_link_libraries(sonar_bridge_test
    ${PROJECT_NAME} gz-transport::core)
endif()

ament_package()
